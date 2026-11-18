/*
 * Xiongmai XM8536E display block as xm_vo.ko drives it: 0x31000000, 576 KiB.
 *
 *   +0x00000 display 0, +0x20000 display 1 (same layout as the XM530V200's):
 *            +0x4018 run, +0x4040 frame IRQ enable, +0x4044 frame IRQ status
 *            (the handler reads it, then writes 0)
 *   +0x45000 HDMI: +0x45320 bit 1 hot-plug IRQ, bit 7 sink present
 *            +0x456A4 and +0x456BC bit 0: PHY PLLs locked, polled for 120 ms
 *            by set_pub_attr
 *   +0x71000 write-back: +0x7104C done status, cleared by writing 0
 *
 * Display 0 is drawn on the QEMU console, bottom to top:
 *   +0x0010 background, 0x00YYUUVV
 *   +0x00c0 + 0x80*n, n < 16: video window n, NV12
 *            +0x00/+0x0c three Y / three C buffer addresses
 *            +0x18/+0x1c Y/C stride, +0x20..+0x2c x0 x1 y0 y1 (inclusive)
 *            +0x30 enable
 *   +0x0040 + 4*n: buffer (0..2) window n shows, set by the frame interrupt
 *   +0x400c/+0x4014 active width/height
 *   +0x6000 GUI layer enable, +0x6014 stride, +0x6018..+0x6024 x0 x1 y0 y1,
 *            +0x6028 address; ARGB1555, bit 15 set = transparent
 *   +0x9000 cursor enable, +0x9010..+0x901c x0 x1 y0 y1,
 *            +0xa000 32x32 ARGB1555 bitmap held in registers, 0 = transparent
 * XM8520D (layout=1, its older xm_vo.ko): window n at +0x1000 + 0x80*n,
 * three buffers of three words, Y at +0x0C * b and C at +0x04 + 0x0C * b
 * (b from +0x0040 + 4*n, as above), +0x30 / +0x34 strides, +0x40..+0x4c
 * x0 x1 y0 y1, +0x50 enable.
 * Windows are shown 1:1: the decoder/VPSS already scaled them.  The sink
 * stays unplugged, so Sofia uses its default mode.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

#define TYPE_XM_VO2 "xm-vo2"
OBJECT_DECLARE_SIMPLE_TYPE(XmVo2State, XM_VO2)

#define VO2_SIZE        0x90000
#define VO2_DEVS        2
#define VO2_DEV(n)      ((n) * 0x20000)
#define VO2_RUN         0x4018
#define VO2_IE          0x4040
#define VO2_ST          0x4044
#define VO2_PLL0        0x456A4
#define VO2_PLL1        0x456BC
#define VO2_WBC_ST      0x7104C
#define VO2_FRAME_NS    (NANOSECONDS_PER_SECOND / 60)
#define VO2_BG          0x10
#define VO2_WIN(n)      (0xc0 + (n) * 0x80)
#define VO2_WIN_IDX(n)  (0x40 + (n) * 4)
#define VO2_WINS        16
#define VO2_W           0x400c
#define VO2_H           0x4014
#define VO2_GUI         0x6000
#define VO2_CUR         0x9000
#define VO2_CUR_BMP     0xa000
#define VO2_MAX_W       4096

struct XmVo2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[VO2_DEVS];
    qemu_irq wbc_irq;
    qemu_irq hdmi_irq;
    QEMUTimer *timer;
    uint32_t regs[VO2_SIZE / 4];
    unsigned long seen[BITS_TO_LONGS(VO2_SIZE / 4)];
    QemuConsole *con;
    uint32_t layout;            /* 0: XM8536E / XM8521DV200, 1: XM8520D */
    uint8_t line[VO2_MAX_W * 2];
    uint8_t cline[VO2_MAX_W];
};

#define R(o) (s->regs[(o) / 4])

static inline uint8_t vo2_clip(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* BT.601 limited range */
static inline uint32_t vo2_yuv(int y, int u, int v)
{
    int c = 298 * (y - 16), d = u - 128, e = v - 128;

    return vo2_clip((c + 409 * e + 128) >> 8) << 16 |
           vo2_clip((c - 100 * d - 208 * e + 128) >> 8) << 8 |
           vo2_clip((c + 516 * d + 128) >> 8);
}

static inline uint32_t vo2_1555(uint16_t p)
{
    uint32_t r = p >> 10 & 31, g = p >> 5 & 31, b = p & 31;

    return (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
}

/* Clip an inclusive rectangle to the screen; false when nothing is left */
static bool vo2_rect(XmVo2State *s, hwaddr o, int w, int h,
                     int *x0, int *x1, int *y0, int *y1)
{
    *x0 = R(o);
    *x1 = MIN((int)R(o + 4), w - 1);
    *y0 = R(o + 8);
    *y1 = MIN((int)R(o + 12), h - 1);
    return *x0 <= *x1 && *y0 <= *y1 && *x1 - *x0 < VO2_MAX_W;
}

static void xm_vo2_draw_win(XmVo2State *s, int n, uint32_t *fb, int fs,
                            int w, int h)
{
    hwaddr o = VO2_WIN(n), ya, ca;
    int idx = R(VO2_WIN_IDX(n)) % 3, x0, x1, y0, y1, x, y, len;
    uint32_t ys, cs;

    if (s->layout == 1) {
        o = 0x1000 + n * 0x80;
        if (!R(o + 0x50) || !vo2_rect(s, o + 0x40, w, h, &x0, &x1, &y0, &y1)) {
            return;
        }
        ya = R(o + idx * 0x0c);
        ca = R(o + 4 + idx * 0x0c);
        ys = R(o + 0x30);
        cs = R(o + 0x34);
    } else {
        if (!R(o + 0x30) || !vo2_rect(s, o + 0x20, w, h, &x0, &x1, &y0, &y1)) {
            return;
        }
        ya = R(o + idx * 4);
        ca = R(o + 0x0c + idx * 4);
        ys = R(o + 0x18);
        cs = R(o + 0x1c);
    }
    len = x1 - x0 + 1;
    for (y = y0; y <= y1; y++) {
        uint32_t *d = fb + y * fs + x0;

        address_space_read(&address_space_memory,
                           ya + (hwaddr)(y - y0) * ys,
                           MEMTXATTRS_UNSPECIFIED, s->line, len);
        if (!((y - y0) & 1)) {
            address_space_read(&address_space_memory,
                               ca + (hwaddr)((y - y0) >> 1) * cs,
                               MEMTXATTRS_UNSPECIFIED, s->cline,
                               (len + 1) & ~1);
        }
        for (x = 0; x < len; x++) {
            d[x] = vo2_yuv(s->line[x], s->cline[x & ~1], s->cline[x | 1]);
        }
    }
}

static void xm_vo2_draw_gui(XmVo2State *s, uint32_t *fb, int fs, int w, int h)
{
    int x0, x1, y0, y1, x, y, len;

    if (!R(VO2_GUI) || !vo2_rect(s, VO2_GUI + 0x18, w, h, &x0, &x1, &y0, &y1)) {
        return;
    }
    len = x1 - x0 + 1;
    for (y = y0; y <= y1; y++) {
        uint32_t *d = fb + y * fs + x0;

        address_space_read(&address_space_memory,
                           R(VO2_GUI + 0x28) + (hwaddr)(y - y0) * R(VO2_GUI + 0x14),
                           MEMTXATTRS_UNSPECIFIED, s->line, len * 2);
        for (x = 0; x < len; x++) {
            uint16_t p = lduw_le_p(s->line + x * 2);

            if (!(p & 0x8000)) {
                d[x] = vo2_1555(p);
            }
        }
    }
}

static void xm_vo2_draw_cursor(XmVo2State *s, uint32_t *fb, int fs, int w, int h)
{
    int x0, x1, y0, y1, x, y;

    if (!R(VO2_CUR) || !vo2_rect(s, VO2_CUR + 0x10, w, h, &x0, &x1, &y0, &y1)) {
        return;
    }
    for (y = y0; y <= y1 && y - y0 < 32; y++) {
        for (x = x0; x <= x1 && x - x0 < 32; x++) {
            int i = (y - y0) * 32 + (x - x0);
            uint16_t p = R(VO2_CUR_BMP + (i & ~1) * 2) >> (i & 1) * 16;

            if (p) {
                fb[y * fs + x] = vo2_1555(p);
            }
        }
    }
}

static void xm_vo2_gfx_update(void *opaque)
{
    XmVo2State *s = opaque;
    DisplaySurface *ds;
    uint32_t *fb, bg;
    int w = R(VO2_W), h = R(VO2_H), fs, n, i;

    if (!R(VO2_RUN) || w < 16 || h < 16 || w > VO2_MAX_W || h > VO2_MAX_W) {
        return;
    }
    ds = qemu_console_surface(s->con);
    if (surface_width(ds) != w || surface_height(ds) != h) {
        qemu_console_resize(s->con, w, h);
        ds = qemu_console_surface(s->con);
    }
    fb = surface_data(ds);
    fs = surface_stride(ds) / 4;
    bg = vo2_yuv(R(VO2_BG) >> 16 & 0xff, R(VO2_BG) >> 8 & 0xff, R(VO2_BG) & 0xff);
    for (i = 0; i < h; i++) {
        for (n = 0; n < w; n++) {
            fb[i * fs + n] = bg;
        }
    }
    for (n = 0; n < VO2_WINS; n++) {
        xm_vo2_draw_win(s, n, fb, fs, w, h);
    }
    xm_vo2_draw_gui(s, fb, fs, w, h);
    xm_vo2_draw_cursor(s, fb, fs, w, h);
    dpy_gfx_update_full(s->con);
}

static const GraphicHwOps xm_vo2_gfx_ops = {
    .gfx_update = xm_vo2_gfx_update,
};

static void xm_vo2_update(XmVo2State *s)
{
    int n;

    for (n = 0; n < VO2_DEVS; n++) {
        qemu_set_irq(s->irq[n], s->regs[(VO2_DEV(n) + VO2_ST) / 4] != 0);
    }
    qemu_set_irq(s->wbc_irq, s->regs[VO2_WBC_ST / 4] != 0);
}

static void xm_vo2_frame(void *opaque)
{
    XmVo2State *s = opaque;
    int n;

    for (n = 0; n < VO2_DEVS; n++) {
        if (s->regs[(VO2_DEV(n) + VO2_RUN) / 4] &&
            s->regs[(VO2_DEV(n) + VO2_IE) / 4]) {
            s->regs[(VO2_DEV(n) + VO2_ST) / 4] = 1;
        }
    }
    xm_vo2_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VO2_FRAME_NS);
}

static void xm_vo2_touch(XmVo2State *s, hwaddr off, uint64_t val, bool write)
{
    /* The register map is only known from the driver: note first touches */
    if (!test_and_set_bit(off / 4, s->seen)) {
        qemu_log_mask(LOG_UNIMP, "xm-vo2: first %s 0x%05" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", write ? "write" : "read", off, val);
    }
}

static uint64_t xm_vo2_read(void *opaque, hwaddr off, unsigned size)
{
    XmVo2State *s = opaque;
    uint64_t val = s->regs[off / 4];

    if (off == VO2_PLL0 || off == VO2_PLL1) {
        val |= 1;
    }
    xm_vo2_touch(s, off, val, false);
    return val;
}

static void xm_vo2_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmVo2State *s = opaque;

    xm_vo2_touch(s, off, val, true);
    /* dev+0 is the update strobe: channel show/hide polls 120 ms for it to
     * read back other than 1 and fails otherwise.  Taken at once. */
    if (off == VO2_DEV(0) || off == VO2_DEV(1)) {
        val = 0;
    }
    s->regs[off / 4] = val;
    xm_vo2_update(s);
}

static const MemoryRegionOps xm_vo2_ops = {
    .read = xm_vo2_read,
    .write = xm_vo2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_vo2_reset(DeviceState *dev)
{
    XmVo2State *s = XM_VO2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VO2_FRAME_NS);
    xm_vo2_update(s);
}

static void xm_vo2_init(Object *obj)
{
    XmVo2State *s = XM_VO2(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &xm_vo2_ops, s, TYPE_XM_VO2,
                          VO2_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (n = 0; n < VO2_DEVS; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
    }
    sysbus_init_irq(sbd, &s->wbc_irq);
    sysbus_init_irq(sbd, &s->hdmi_irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_vo2_frame, s);
}

static void xm_vo2_realize(DeviceState *dev, Error **errp)
{
    XmVo2State *s = XM_VO2(dev);

    s->con = graphic_console_init(dev, 0, &xm_vo2_gfx_ops, s);
    qemu_console_resize(s->con, 1280, 720);
}

static const Property xm_vo2_props[] = {
    DEFINE_PROP_UINT32("layout", XmVo2State, layout, 0),
};

static void xm_vo2_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = xm_vo2_realize;
    device_class_set_props(DEVICE_CLASS(klass), xm_vo2_props);
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_vo2_reset);
}

static const TypeInfo xm_vo2_info = {
    .name = TYPE_XM_VO2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmVo2State),
    .instance_init = xm_vo2_init,
    .class_init = xm_vo2_class_init,
};

static void xm_vo2_register(void)
{
    type_register_static(&xm_vo2_info);
}

type_init(xm_vo2_register)
