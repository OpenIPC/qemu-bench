/*
 * Novatek NA51089 (NT9856x) IDE, the image display engine (0xF0800000,
 * SPI 48): the LCD / MIPI DSI output path, scanned out to a QEMU console.
 *
 * Registers from kdrv_videoout/ide/include/ide_reg.h, as the closed
 * ide2.o / ide_idec.o / disp1.o drive them (the vendor U-Boot's
 * drv_videoout has the same driver as source):
 *
 *   0x000 CTRL     bit 0 DMA disable, 1 OSD1 on, 2 DMA idle (read only),
 *                  3 / 4 video 1 / 2 on, [10:7] the device (1 serial RGB,
 *                  9 parallel, 0xD MIPI DSI), 22 RGBD, 26 through, 27 the
 *                  engine on, 28 load
 *   0x0BC          background Y [7:0], Cb [15:8], Cr [23:16]
 *   0x0D0 / 0x0D8  horizontal active start [11:0] / (odd field) vertical
 *                  active start [10:0] (ide_reg.h: 11 bits), and end
 *                  [28:12]
 *   0x0FC INT      enables [15:0], status [31:16] (write 1 to clear):
 *                  bit 7 / 23 frame end
 *   layers V1 / V2 / OSD1:
 *     addresses    0xD00 + 0xD10 / 0xD40 + 0xD50 / 0x05C (Y, CbCr / ARGB)
 *     buffer size  0x028 / 0x054 / 0x060: width [12:0], height [27:16]
 *     line offset  0x02C / 0x058 / 0x064 [12:0], 32-bit words; bit 24
 *                  mirrors left-right, bit 25 top-bottom
 *     window       0x0A8 / 0x0B4 / 0x090: width - 1 [12:0], height - 1
 *                  [27:16], format [31:28]; position 0x0AC / 0x0B8 /
 *                  0x094: x [12:0], y [27:16]
 *   0x1B4 / 0x1B8  blending: OSD1's alpha select [6:4] and global alpha
 *                  [31:24] (ARGB1555 A = 0: 0x1B8 [7:0]); V2's [1:0] and
 *                  [15:8]
 *   0x130 / 0x138  OSD1 colour key: R, G, B, A; enable bit 0, compare
 *                  alpha too (op 4)
 *   0x200          the palette, 256 x (Cr, Cb, Y, A)
 *
 * Formats: video 9 (YUV 4:2:2, CbCr interleaved) and 10 (4:2:0, NV12);
 * OSD1 3 (8-bit palette), 11 ARGB8888, 12 ARGB4444, 13 ARGB1555.  Others
 * are logged as unimplemented.
 *
 * Most registers take effect at the next frame: the driver sets load
 * and waits, with no timeout, for frame end; its handler wakes it only
 * if load reads 0 by then.  So each frame latches the registers into the
 * active set, clears load, and only then raises frame end.  Frames run at
 * 60 Hz while the engine is on.
 *
 * The EVB's panel is 8-bit serial RGB: 960 clocks a line, one colour
 * each, for 320 pixels, and the windows are 960 wide.  Out of such a
 * device the console shows every third column.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "ui/console.h"

#define TYPE_NVT_IDE "nvt-ide"
OBJECT_DECLARE_SIMPLE_TYPE(NvtIdeState, NVT_IDE)

#define IDE_SIZE            0x1000

#define IDE_CTRL            0x000
#define IDE_BG              0x0BC
#define IDE_HVALID          0x0D0
#define IDE_VVALID          0x0D8
#define IDE_INT             0x0FC
#define IDE_CUR_ADDR        0x100
#define IDE_BLEND           0x1B4
#define IDE_BLEND2          0x1B8
#define IDE_CKEY            0x130
#define IDE_CKEY_CTL        0x138
#define IDE_PALETTE         0x200

#define CTRL_DMA_DIS        (1u << 0)
#define CTRL_O1_EN          (1u << 1)
#define CTRL_DMA_IDLE       (1u << 2)
#define CTRL_V1_EN          (1u << 3)
#define CTRL_V2_EN          (1u << 4)
#define CTRL_RGBD           (1u << 22)
#define CTRL_THROUGH        (1u << 26)
#define CTRL_EN             (1u << 27)
#define CTRL_LOAD           (1u << 28)

#define INT_VS              (1u << 0)
#define INT_FRAME_END       (1u << 7)

#define DEV_SERIAL_RGB      1

#define IDE_REFRESH_NS      (NANOSECONDS_PER_SECOND / 60)
#define IDE_DEFAULT_W       960
#define IDE_DEFAULT_H       240
#define IDE_MAX             4096

enum {
    FMT_PAL8 = 3, FMT_YUV422P = 9, FMT_YUV420P = 10, FMT_ARGB8888 = 11,
    FMT_ARGB4444 = 12, FMT_ARGB1555 = 13,
};

/* a layer's registers */
typedef struct {
    uint32_t en;                /* its CTRL bit */
    hwaddr addr, caddr;         /* luma / ARGB, chroma */
    hwaddr dim, attr, win, pos;
    bool osd;
} NvtIdeLayer;

static const NvtIdeLayer nvt_ide_layers[] = {
    { CTRL_V1_EN, 0xD00, 0xD10, 0x028, 0x02C, 0x0A8, 0x0AC, false },
    { CTRL_V2_EN, 0xD40, 0xD50, 0x054, 0x058, 0x0B4, 0x0B8, false },
    { CTRL_O1_EN, 0x05C, 0, 0x060, 0x064, 0x090, 0x094, true },
};

struct NvtIdeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QemuConsole *con;
    QEMUTimer *frame;
    uint32_t regs[IDE_SIZE / 4];
    uint32_t active[IDE_SIZE / 4];      /* as latched at the last frame */
    uint32_t width, height;             /* the console's */
    bool trace;
};

static uint32_t *reg(NvtIdeState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static uint32_t act(NvtIdeState *s, hwaddr off)
{
    return s->active[off / 4];
}

static void nvt_ide_update_irq(NvtIdeState *s)
{
    uint32_t v = *reg(s, IDE_INT);

    qemu_set_irq(s->irq, ((v >> 16) & v & 0x7FF) != 0);
}

static void nvt_ide_latch(NvtIdeState *s)
{
    memcpy(s->active, s->regs, sizeof(s->active));
}

static void nvt_ide_frame(void *opaque)
{
    NvtIdeState *s = opaque;

    if (!(*reg(s, IDE_CTRL) & CTRL_EN)) {
        return;
    }
    /* the order the driver's handler depends on: load clear, then end */
    if (*reg(s, IDE_CTRL) & CTRL_LOAD) {
        *reg(s, IDE_CTRL) &= ~CTRL_LOAD;
        nvt_ide_latch(s);
    }
    *reg(s, IDE_INT) |= (INT_VS | INT_FRAME_END) << 16;
    nvt_ide_update_irq(s);
    timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              IDE_REFRESH_NS);
}

/* ---- scanout ---- */

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* BT.601 limited range, as the rest of the media path */
static uint32_t yuv_rgb(int y, int u, int v)
{
    int c = y - 16;

    u -= 128;
    v -= 128;
    return clamp8((298 * c + 409 * v + 128) >> 8) << 16 |
           clamp8((298 * c - 100 * u - 208 * v + 128) >> 8) << 8 |
           clamp8((298 * c + 516 * u + 128) >> 8);
}

static uint32_t blend(uint32_t src, uint32_t dst, unsigned a)
{
    uint32_t out = 0;
    int i;

    for (i = 0; i < 24; i += 8) {
        out |= (((src >> i & 0xFF) * a + (dst >> i & 0xFF) * (255 - a) +
                 127) / 255) << i;
    }
    return out;
}

/* one source pixel of an OSD1 row as RGB, and its alpha */
static uint32_t osd_pixel(NvtIdeState *s, const uint8_t *row, uint32_t x,
                          unsigned fmt, unsigned *alpha)
{
    uint32_t blend2 = act(s, IDE_BLEND2), ga = act(s, IDE_BLEND) >> 24;
    uint32_t p, r, g, b;

    switch (fmt) {
    case FMT_ARGB8888:
        p = ldl_le_p(row + 4 * x);
        *alpha = p >> 24;
        return p & 0xFFFFFF;
    case FMT_ARGB4444:
        p = lduw_le_p(row + 2 * x);
        *alpha = (p >> 12) * 17;
        return ((p >> 8) & 0xF) * 17 << 16 | ((p >> 4) & 0xF) * 17 << 8 |
               (p & 0xF) * 17;
    case FMT_ARGB1555:
        p = lduw_le_p(row + 2 * x);
        *alpha = p & 0x8000 ? ga : blend2 & 0xFF;
        r = (p >> 10) & 0x1F;
        g = (p >> 5) & 0x1F;
        b = p & 0x1F;
        return (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 |
               (b << 3 | b >> 2);
    default:                            /* FMT_PAL8: (Cr, Cb, Y, A) */
        p = act(s, IDE_PALETTE + 4 * row[x]);
        *alpha = p >> 24;
        return yuv_rgb((p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF);
    }
}

static unsigned osd_bpp(unsigned fmt)
{
    return fmt == FMT_ARGB8888 ? 4 : fmt == FMT_PAL8 ? 1 : 2;
}

/*
 * A layer into the w x h output: its buffer scaled into its window,
 * nearest neighbour, over what is there.
 */
static void nvt_ide_draw(NvtIdeState *s, const NvtIdeLayer *l, uint32_t *out,
                         uint32_t w, uint32_t h)
{
    uint32_t dim = act(s, l->dim), attr = act(s, l->attr);
    uint32_t win = act(s, l->win), pos = act(s, l->pos);
    uint32_t sw = dim & 0x1FFF, sh = (dim >> 16) & 0xFFF;
    uint32_t stride = (attr & 0x1FFF) * 4;
    uint32_t ww = (win & 0x1FFF) + 1, wh = ((win >> 16) & 0xFFF) + 1;
    uint32_t x0 = pos & 0x1FFF, y0 = (pos >> 16) & 0xFFF;
    unsigned fmt = win >> 28, sel, ga, a = 255;
    hwaddr addr = act(s, l->addr) & 0x7FFFFFFF;
    hwaddr caddr = l->caddr ? act(s, l->caddr) & 0x7FFFFFFF : 0;
    bool l2r = attr & (1u << 24), t2b = attr & (1u << 25);
    uint32_t ckey = act(s, IDE_CKEY), ckctl = act(s, IDE_CKEY_CTL);
    g_autofree uint8_t *row = NULL, *crow = NULL;
    uint32_t dx, dy, row_bytes;

    if (!addr || !sw || !sh || !stride || x0 >= w || y0 >= h) {
        return;
    }
    if (l->osd ? fmt != FMT_PAL8 && fmt != FMT_ARGB8888 &&
                 fmt != FMT_ARGB4444 && fmt != FMT_ARGB1555
               : fmt != FMT_YUV422P && fmt != FMT_YUV420P) {
        qemu_log_mask(LOG_UNIMP, "nvt-ide: %s format %u\n",
                      l->osd ? "OSD" : "video", fmt);
        return;
    }
    row_bytes = l->osd ? sw * osd_bpp(fmt) : sw;
    if (row_bytes > stride) {
        row_bytes = stride;
        sw = l->osd ? stride / osd_bpp(fmt) : stride;
    }
    row = g_malloc(row_bytes);
    crow = g_malloc(row_bytes);
    if (l->osd) {
        sel = (act(s, IDE_BLEND) >> 4) & 7;
        ga = act(s, IDE_BLEND) >> 24;
    } else {
        sel = l->en == CTRL_V2_EN ? act(s, IDE_BLEND) & 3 : 0;
        ga = (act(s, IDE_BLEND) >> 8) & 0xFF;
    }
    for (dy = y0; dy < MIN(h, y0 + wh); dy++) {
        uint32_t sy = (uint64_t)(dy - y0) * sh / wh;

        if (t2b) {
            sy = sh - 1 - sy;
        }
        address_space_read(&address_space_memory, addr + (hwaddr)sy * stride,
                           MEMTXATTRS_UNSPECIFIED, row, row_bytes);
        if (!l->osd) {
            uint32_t cy = fmt == FMT_YUV420P ? sy / 2 : sy;

            address_space_read(&address_space_memory,
                               caddr + (hwaddr)cy * stride,
                               MEMTXATTRS_UNSPECIFIED, crow, row_bytes);
        }
        for (dx = x0; dx < MIN(w, x0 + ww); dx++) {
            uint32_t sx = (uint64_t)(dx - x0) * sw / ww, rgb;
            uint32_t *o = &out[(size_t)dy * w + dx];

            if (l2r) {
                sx = sw - 1 - sx;
            }
            if (l->osd) {
                unsigned pa;

                rgb = osd_pixel(s, row, sx, fmt, &pa);
                if ((ckctl & 1) && rgb == (((ckey & 0xFF) << 16) |
                                           (ckey & 0xFF00) |
                                           ((ckey >> 16) & 0xFF)) &&
                    (((ckctl >> 1) & 7) != 4 || pa == ckey >> 24)) {
                    continue;           /* the colour key: transparent */
                }
                /* alpha: 0 opaque, 1 / 2 global, 3 / 4 the pixel's */
                a = sel == 0 ? 255 : sel <= 2 ? ga : pa;
                if (fmt == FMT_ARGB1555 && sel >= 3) {
                    a = pa;
                }
            } else {
                rgb = yuv_rgb(row[sx], crow[sx & ~1u],
                              crow[MIN(sx | 1, row_bytes - 1)]);
                a = sel == 1 || sel == 2 ? ga : 255;
            }
            *o = a == 255 ? rgb : blend(rgb, *o, a);
        }
    }
}

static void nvt_ide_gfx_update(void *opaque)
{
    NvtIdeState *s = opaque;
    uint32_t ctrl = act(s, IDE_CTRL), hv = act(s, IDE_HVALID);
    uint32_t vv = act(s, IDE_VVALID), bg = act(s, IDE_BG);
    uint32_t w = ((hv >> 12) & 0x1FFFF) - (hv & 0xFFF) + 1;
    uint32_t h = ((vv >> 12) & 0x1FFFF) - (vv & 0x7FF) + 1;
    uint32_t step, cw, x, y, i;
    g_autofree uint32_t *out = NULL;
    DisplaySurface *surface;
    uint8_t *dst;

    if (!hv || !vv || w > IDE_MAX || h > IDE_MAX || !w || !h) {
        w = IDE_DEFAULT_W;
        h = IDE_DEFAULT_H;
    }
    /* serial RGB: three clocks a pixel */
    step = ((ctrl >> 7) & 0xF) == DEV_SERIAL_RGB &&
           !(ctrl & (CTRL_THROUGH | CTRL_RGBD)) && w >= 3 ? 3 : 1;
    cw = w / step;
    if (cw != s->width || h != s->height) {
        s->width = cw;
        s->height = h;
        qemu_console_resize(s->con, cw, h);
    }
    out = g_new(uint32_t, (size_t)w * h);
    for (i = 0; i < w * h; i++) {
        out[i] = yuv_rgb(bg & 0xFF, (bg >> 8) & 0xFF, (bg >> 16) & 0xFF);
    }
    if ((ctrl & CTRL_EN) && !(ctrl & CTRL_DMA_DIS)) {
        for (i = 0; i < ARRAY_SIZE(nvt_ide_layers); i++) {
            if (ctrl & nvt_ide_layers[i].en) {
                nvt_ide_draw(s, &nvt_ide_layers[i], out, w, h);
            }
        }
    } else {
        memset(out, 0, (size_t)w * h * 4);      /* the panel dark */
    }
    surface = qemu_console_surface(s->con);
    dst = surface_data(surface);
    for (y = 0; y < h; y++) {
        uint32_t *d = (uint32_t *)(dst + (size_t)y * surface_stride(surface));

        for (x = 0; x < cw; x++) {
            d[x] = 0xFF000000 | out[(size_t)y * w + x * step];
        }
    }
    dpy_gfx_update_full(s->con);
}

static const GraphicHwOps nvt_ide_gfx_ops = {
    .gfx_update = nvt_ide_gfx_update,
};

/* ---- registers ---- */

static uint64_t nvt_ide_read(void *opaque, hwaddr off, unsigned size)
{
    NvtIdeState *s = opaque;
    uint32_t v;

    off &= ~3;
    switch (off) {
    case IDE_CTRL:
        v = *reg(s, off) & ~CTRL_DMA_IDLE;
        if ((v & CTRL_DMA_DIS) || !(v & CTRL_EN)) {
            v |= CTRL_DMA_IDLE;
        }
        return v;
    case IDE_CUR_ADDR:                  /* where each layer is reading */
        return act(s, 0xD00);
    case IDE_CUR_ADDR + 0x04:
        return act(s, 0xD10);
    case IDE_CUR_ADDR + 0x0C:
        return act(s, 0xD40);
    case IDE_CUR_ADDR + 0x10:
        return act(s, 0xD50);
    case IDE_CUR_ADDR + 0x18:
        return act(s, 0x05C);
    case IDE_CUR_ADDR + 0x20:
        return act(s, 0x1BC);
    }
    return *reg(s, off);
}

static void nvt_ide_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    NvtIdeState *s = opaque;
    uint32_t val = value, old;

    off &= ~3;
    if (s->trace) {
        qemu_log_mask(LOG_UNIMP, "nvt-ide: 0x%03x <- 0x%08x\n",
                      (unsigned)off, val);
    }
    switch (off) {
    case IDE_CTRL:
        old = *reg(s, off);
        *reg(s, off) = val & ~CTRL_DMA_IDLE;
        if ((val & CTRL_EN) && !(old & CTRL_EN)) {
            /* on: what was set up shows at once, frames start */
            *reg(s, off) &= ~CTRL_LOAD;
            nvt_ide_latch(s);
            timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      IDE_REFRESH_NS);
        } else if (!(val & CTRL_EN)) {
            timer_del(s->frame);
            s->active[IDE_CTRL / 4] = *reg(s, off);
        }
        return;
    case IDE_INT:
        *reg(s, off) = (val & 0xFFFF) |
                       (*reg(s, off) & 0xFFFF0000 & ~(val & 0xFFFF0000));
        nvt_ide_update_irq(s);
        return;
    }
    if (off >= IDE_CUR_ADDR && off <= IDE_CUR_ADDR + 0x20) {
        return;                         /* read only */
    }
    *reg(s, off) = val;
}

static const MemoryRegionOps nvt_ide_ops = {
    .read = nvt_ide_read,
    .write = nvt_ide_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_ide_reset(DeviceState *dev)
{
    NvtIdeState *s = NVT_IDE(dev);

    timer_del(s->frame);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->active, 0, sizeof(s->active));
    qemu_irq_lower(s->irq);
}

static void nvt_ide_realize(DeviceState *dev, Error **errp)
{
    NvtIdeState *s = NVT_IDE(dev);

    s->width = IDE_DEFAULT_W / 3;
    s->height = IDE_DEFAULT_H;
    s->con = graphic_console_init(dev, 0, &nvt_ide_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_ide_frame, s);
}

static void nvt_ide_init(Object *obj)
{
    NvtIdeState *s = NVT_IDE(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ide_ops, s, TYPE_NVT_IDE,
                          IDE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property nvt_ide_properties[] = {
    DEFINE_PROP_BOOL("trace", NvtIdeState, trace, false),
};

static const VMStateDescription vmstate_nvt_ide = {
    .name = TYPE_NVT_IDE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtIdeState, IDE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(active, NvtIdeState, IDE_SIZE / 4),
        VMSTATE_TIMER_PTR(frame, NvtIdeState),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_ide_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_ide_realize;
    device_class_set_legacy_reset(dc, nvt_ide_reset);
    device_class_set_props(dc, nvt_ide_properties);
    dc->vmsd = &vmstate_nvt_ide;
}

static const TypeInfo nvt_ide_info = {
    .name = TYPE_NVT_IDE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtIdeState),
    .instance_init = nvt_ide_init,
    .class_init = nvt_ide_class_init,
};

static void nvt_ide_register_types(void)
{
    type_register_static(&nvt_ide_info);
}

type_init(nvt_ide_register_types)
