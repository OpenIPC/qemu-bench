/*
 * Novatek NA51103 LCD300 display controller (0x2_F0800000, SPI 18): the
 * HDMI / VGA output path the NVR GUI draws through.
 *
 * Driven by the HDAL's kdrv_videoout/lcd310/flcd300-{common,drv}.ko and
 * kflow_videoout/lcd310/flcd300-pip.ko (not stripped), and by the vendor
 * U-Boot's boot logo.  Registers so far:
 *
 *   0x000  CTRL      bit 31 register update request: software sets it
 *                    after changing the configuration and the controller
 *                    clears it at the next frame, once it has taken the
 *                    new values.  The driver's tasklet will not program
 *                    anything while it reads high
 *                    (dev_check_regupdate_high()), so it must drop.
 *   0x004  output size: [27:16] height, [11:0] width (0x02D00500 = 720p)
 *   0x010  the input channels, 0x28 apart (see below)
 *   0x1A4  interrupt status; bit 5 frame end (bits 0x2F are the ones the
 *          driver's handler takes as "frame done")
 *   0x1A8  interrupt enable (the kernel sets bit 5)
 *   0x1AC  write 1 to clear status
 *   0x840  [27:16] vertical total, [12:0] horizontal total
 *
 * The frame-end interrupt is not optional: flcd300-pip only writes a
 * plane's new framebuffer address from its handler, so without it the
 * GUI never reaches the registers.  Frames run at 60 Hz.
 *
 * Scanout composes the enabled channels in order.  trace=on logs
 * every register write (-d unimp), for working out the rest of the plane
 * layout from a running firmware.
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

#define TYPE_NVT_LCD300 "nvt-lcd300"
OBJECT_DECLARE_SIMPLE_TYPE(NvtLcd300State, NVT_LCD300)

#define LCD_SIZE            0x10000
#define LCD_CTRL            0x000
#define LCD_OUT_SIZE        0x004
#define LCD_INT_STS         0x1A4
#define LCD_INT_EN          0x1A8
#define LCD_INT_CLR         0x1AC

#define CTRL_UPDATE         (1u << 31)
#define INT_FRAME_END       (1u << 5)

#define LCD_REFRESH_NS      (NANOSECONDS_PER_SECOND / 60)
#define LCD_DEFAULT_W       1280
#define LCD_DEFAULT_H       720
#define LCD_MAX_W           1920
#define LCD_MAX_H           1088

struct NvtLcd300State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QemuConsole *con;
    QEMUTimer *vsync;
    uint32_t regs[LCD_SIZE / 4];
    uint32_t width, height;
    bool trace;
};

static uint32_t *reg(NvtLcd300State *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_lcd300_update_irq(NvtLcd300State *s)
{
    qemu_set_irq(s->irq, (*reg(s, LCD_INT_STS) & *reg(s, LCD_INT_EN)) != 0);
}

static void nvt_lcd300_vsync(void *opaque)
{
    NvtLcd300State *s = opaque;

    *reg(s, LCD_CTRL) &= ~CTRL_UPDATE;
    *reg(s, LCD_INT_STS) |= INT_FRAME_END;
    nvt_lcd300_update_irq(s);
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              LCD_REFRESH_NS);
}

/*
 * The three input channels, 0x28 apart from 0x010 (dev_channel_cfg()):
 * +0x00 / +0x04 the buffer (luma / chroma for YUV), +0x08 the source size
 * ([27:16] height, [11:0] width), +0x1C the size shown, +0x24 control:
 * bit 31 enable, [3:0] the format.  As the NVR firmware sets them:
 * channel 0 is video, YUV 4:2:0 with chroma in its own plane (format 9);
 * channel 1 the GUI framebuffer, ARGB1555 (format 1); channel 2 the
 * mouse cursor, 64x64 (format 5).
 */
#define LCD_CH_BASE(n)      (0x010 + 0x28 * (n))
#define LCD_CH_ADDR0        0x00
#define LCD_CH_ADDR1        0x04
#define LCD_CH_SIZE         0x08
#define LCD_CH_CTRL         0x24
#define LCD_CHANNELS        3

#define CH_EN               (1u << 31)
#define FMT_ARGB1555        1
#define FMT_ARGB8888        5
#define FMT_YUV420SP        9

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static void nvt_lcd300_draw_yuv(hwaddr y_addr, hwaddr c_addr, uint32_t *dst,
                                uint32_t w, uint32_t h, uint32_t stride)
{
    g_autofree uint8_t *yl = g_malloc(w);
    g_autofree uint8_t *cl = g_malloc(w);
    uint32_t x, y;

    for (y = 0; y < h; y++, dst += stride / 4) {
        address_space_read(&address_space_memory, y_addr + (hwaddr)y * w,
                           MEMTXATTRS_UNSPECIFIED, yl, w);
        address_space_read(&address_space_memory,
                           c_addr + (hwaddr)(y / 2) * w,
                           MEMTXATTRS_UNSPECIFIED, cl, w);
        for (x = 0; x < w; x++) {
            int c = yl[x] - 16, u = cl[x & ~1] - 128, v = cl[x | 1] - 128;

            dst[x] = 0xFF000000 |
                     clamp8((298 * c + 409 * v + 128) >> 8) << 16 |
                     clamp8((298 * c - 100 * u - 208 * v + 128) >> 8) << 8 |
                     clamp8((298 * c + 516 * u + 128) >> 8);
        }
    }
}

/* Over what is already there: pixels with alpha clear are transparent. */
static void nvt_lcd300_draw_argb(hwaddr fb, bool argb8888, uint32_t *dst,
                                 uint32_t w, uint32_t h, uint32_t stride)
{
    uint32_t bpp = argb8888 ? 4 : 2;
    g_autofree uint8_t *line = g_malloc(w * bpp);
    uint32_t x, y;

    for (y = 0; y < h; y++, dst += stride / 4) {
        address_space_read(&address_space_memory, fb + (hwaddr)y * w * bpp,
                           MEMTXATTRS_UNSPECIFIED, line, w * bpp);
        for (x = 0; x < w; x++) {
            if (argb8888) {
                uint32_t p = ldl_le_p(line + x * 4);

                if (p >> 24) {
                    dst[x] = 0xFF000000 | p;
                }
            } else {
                uint16_t p = lduw_le_p(line + x * 2);
                uint32_t r = (p >> 10) & 0x1F, g = (p >> 5) & 0x1F;
                uint32_t b = p & 0x1F;

                if (p & 0x8000) {
                    dst[x] = 0xFF000000 | (r << 3 | r >> 2) << 16 |
                             (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
                }
            }
        }
    }
}

static void nvt_lcd300_scanout(NvtLcd300State *s, uint32_t *dst,
                               uint32_t w, uint32_t h, uint32_t stride)
{
    int n;

    for (n = 0; n < h; n++) {
        memset((uint8_t *)dst + n * stride, 0, w * 4);
    }
    for (n = 0; n < LCD_CHANNELS; n++) {
        hwaddr base = LCD_CH_BASE(n);
        uint32_t ctrl = *reg(s, base + LCD_CH_CTRL);
        uint32_t sz = *reg(s, base + LCD_CH_SIZE);
        uint32_t cw = MIN(sz & 0xFFF, w), ch = MIN((sz >> 16) & 0xFFF, h);
        hwaddr a0 = *reg(s, base + LCD_CH_ADDR0);

        if (!(ctrl & CH_EN) || !a0 || !cw || !ch) {
            continue;
        }
        switch (ctrl & 0xF) {
        case FMT_YUV420SP:
            nvt_lcd300_draw_yuv(a0, *reg(s, base + LCD_CH_ADDR1), dst, cw,
                                ch, stride);
            break;
        case FMT_ARGB1555:
        case FMT_ARGB8888:
            nvt_lcd300_draw_argb(a0, (ctrl & 0xF) == FMT_ARGB8888, dst, cw,
                                 ch, stride);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "nvt-lcd300: channel %d format %u\n",
                          n, ctrl & 0xF);
            break;
        }
    }
}

static void nvt_lcd300_gfx_update(void *opaque)
{
    NvtLcd300State *s = opaque;
    DisplaySurface *surface;
    uint32_t sz = *reg(s, LCD_OUT_SIZE);
    uint32_t w = sz & 0xFFF, h = (sz >> 16) & 0xFFF;

    if (!w || !h || w > LCD_MAX_W || h > LCD_MAX_H) {
        w = LCD_DEFAULT_W;
        h = LCD_DEFAULT_H;
    }
    if (w != s->width || h != s->height) {
        s->width = w;
        s->height = h;
        qemu_console_resize(s->con, w, h);
    }
    surface = qemu_console_surface(s->con);
    nvt_lcd300_scanout(s, (uint32_t *)surface_data(surface), w, h,
                       surface_stride(surface));
    dpy_gfx_update_full(s->con);
}

static const GraphicHwOps nvt_lcd300_gfx_ops = {
    .gfx_update = nvt_lcd300_gfx_update,
};

static uint64_t nvt_lcd300_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtLcd300State *s = opaque;

    return *reg(s, offset & ~3);
}

static void nvt_lcd300_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    NvtLcd300State *s = opaque;
    uint32_t val = value;

    offset &= ~3;
    if (s->trace) {
        qemu_log_mask(LOG_UNIMP, "nvt-lcd300: 0x%04x <- 0x%08x\n",
                      (unsigned)offset, val);
    }
    switch (offset) {
    case LCD_INT_STS:
        return;                             /* read-only */
    case LCD_INT_CLR:
        *reg(s, LCD_INT_STS) &= ~val;
        nvt_lcd300_update_irq(s);
        return;
    }
    *reg(s, offset) = val;
    if (offset == LCD_INT_EN) {
        nvt_lcd300_update_irq(s);
    }
}

static const MemoryRegionOps nvt_lcd300_ops = {
    .read = nvt_lcd300_read,
    .write = nvt_lcd300_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_lcd300_reset(DeviceState *dev)
{
    NvtLcd300State *s = NVT_LCD300(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_lcd300_realize(DeviceState *dev, Error **errp)
{
    NvtLcd300State *s = NVT_LCD300(dev);

    s->width = LCD_DEFAULT_W;
    s->height = LCD_DEFAULT_H;
    s->con = graphic_console_init(dev, 0, &nvt_lcd300_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    s->vsync = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_lcd300_vsync, s);
    timer_mod(s->vsync, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              LCD_REFRESH_NS);
}

static void nvt_lcd300_init(Object *obj)
{
    NvtLcd300State *s = NVT_LCD300(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_lcd300_ops, s,
                          TYPE_NVT_LCD300, LCD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property nvt_lcd300_properties[] = {
    DEFINE_PROP_BOOL("trace", NvtLcd300State, trace, false),
};

static const VMStateDescription vmstate_nvt_lcd300 = {
    .name = TYPE_NVT_LCD300,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtLcd300State, LCD_SIZE / 4),
        VMSTATE_TIMER_PTR(vsync, NvtLcd300State),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_lcd300_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_lcd300_realize;
    device_class_set_legacy_reset(dc, nvt_lcd300_reset);
    device_class_set_props(dc, nvt_lcd300_properties);
    dc->vmsd = &vmstate_nvt_lcd300;
}

static const TypeInfo nvt_lcd300_info = {
    .name           = TYPE_NVT_LCD300,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(NvtLcd300State),
    .instance_init  = nvt_lcd300_init,
    .class_init     = nvt_lcd300_class_init,
};

static void nvt_lcd300_register_types(void)
{
    type_register_static(&nvt_lcd300_info);
}

type_init(nvt_lcd300_register_types)
