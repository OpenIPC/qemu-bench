/*
 * Xiongmai XM530 video input (VI), 0x30000000, as xm530_vi.ko drives it.
 *
 * What the model does was learnt from the register writes the driver makes
 * at run time (logged with -d unimp), not from its code.  Per channel c, in
 * the window W = 0x84000 + c * 0x1000:
 *
 *   W+0x20/+0x28   luma / chroma stride, in 8-byte units
 *   W+0x24         width in pixels
 *   W+0x34         channel enable
 *   W+0x40/+0x44   buffer 0/1 luma,   (address - 0x80000000) >> 3
 *   W+0xC0/+0xC4   buffer 0/1 chroma, likewise (NV12: CbCr interleaved)
 *   W+0x08         buffer-done flags, one bit per buffer
 *   0xF0080+c*0x20 interrupt enable; +4 the height plus one
 *   0xF008C+c*0x20 interrupt status: set with a frame, cleared by writing 0
 *
 * When a channel is enabled, a timer delivers frames at frame-rate: the
 * next of its two buffers gets a test pattern (moving colour bars over a
 * frame counter), its done bit is set and the channel's line raised
 * (SPI 33 + c; the driver's vi_back_int takes the channel as dev_id).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "system/dma.h"

#define TYPE_XM_VI "xm-vi"
OBJECT_DECLARE_SIMPLE_TYPE(XmViState, XM_VI)

#define XM_VI_SIZE      0x100000
#define XM_VI_CHANNELS  4
#define XM_VI_WIN(c)    (0x84000 + (c) * 0x1000)
#define XM_VI_INT(c)    (0xF0080 + (c) * 0x20)
#define XM_VI_DDR       0x80000000u

struct XmViState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[XM_VI_CHANNELS];
    uint32_t regs[XM_VI_SIZE / 4];
    QEMUTimer *timer;
    uint32_t fps;
    uint32_t frame[XM_VI_CHANNELS];
};

#define R(s, off) ((s)->regs[(off) / 4])

static bool xm_vi_enabled(XmViState *s, int c)
{
    return (R(s, XM_VI_WIN(c) + 0x34) & 1) && (R(s, XM_VI_INT(c)) & 1);
}

/* Moving SMPTE-ish bars, a frame counter in the top-left as binary
 * blocks, so a stuck or repeated frame shows. */
static void xm_vi_pattern(XmViState *s, int c)
{
    static const uint8_t bars[8][3] = {           /* Y, Cb, Cr */
        { 235, 128, 128 }, { 210, 16, 146 }, { 170, 166, 16 },
        { 145, 54, 34 }, { 106, 202, 222 }, { 81, 90, 240 },
        { 41, 240, 110 }, { 16, 128, 128 },
    };
    const uint32_t win = XM_VI_WIN(c);
    const int w = R(s, win + 0x24);
    const int h = (int)R(s, XM_VI_INT(c) + 4) - 1;
    const int ys = R(s, win + 0x20) * 8, cs = R(s, win + 0x28) * 8;
    const int k = s->frame[c] & 1;
    const hwaddr ya = XM_VI_DDR + (hwaddr)R(s, win + 0x40 + 4 * k) * 8;
    const hwaddr ca = XM_VI_DDR + (hwaddr)R(s, win + 0xC0 + 4 * k) * 8;
    const int shift = (s->frame[c] * 4) % MAX(w, 1);
    uint8_t *line;
    int x, y;

    if (w <= 0 || h <= 0 || w > 4096 || h > 4096 || ys < w || cs < w) {
        return;
    }
    line = g_malloc(MAX(ys, cs));
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const int bar = ((x + shift) % w) * 8 / w;
            uint8_t v = bars[bar][0];
            /* 16 binary blocks of the frame number, 16 px square */
            if (y < 16 && x < 256) {
                v = (s->frame[c] >> (15 - x / 16)) & 1 ? 235 : 16;
            }
            line[x] = v;
        }
        dma_memory_write(&address_space_memory, ya + (hwaddr)y * ys, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (y = 0; y < h / 2; y++) {
        for (x = 0; x < w / 2; x++) {
            const int bar = ((2 * x + shift) % w) * 8 / w;
            const bool blk = y < 8 && 2 * x < 256;
            line[2 * x] = blk ? 128 : bars[bar][1];
            line[2 * x + 1] = blk ? 128 : bars[bar][2];
        }
        dma_memory_write(&address_space_memory, ca + (hwaddr)y * cs, line,
                         w & ~1, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(line);
}

static void xm_vi_tick(void *opaque)
{
    XmViState *s = opaque;
    int c;

    for (c = 0; c < XM_VI_CHANNELS; c++) {
        if (!xm_vi_enabled(s, c)) {
            continue;
        }
        xm_vi_pattern(s, c);
        R(s, XM_VI_WIN(c) + 0x08) = 1u << (s->frame[c] & 1);
        R(s, XM_VI_INT(c) + 0x0C) = 1;
        qemu_irq_raise(s->irq[c]);
        s->frame[c]++;
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              1000 / MAX(s->fps, 1));
}

static uint64_t xm_vi_read(void *opaque, hwaddr off, unsigned size)
{
    XmViState *s = XM_VI(opaque);

    return s->regs[off / 4];
}

static void xm_vi_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmViState *s = XM_VI(opaque);

    int c;

    qemu_log_mask(LOG_UNIMP, "xm-vi: [0x%05" HWADDR_PRIx "] = 0x%08" PRIx64
                  "\n", off, val);
    s->regs[off / 4] = val;
    for (c = 0; c < XM_VI_CHANNELS; c++) {
        if (off == XM_VI_INT(c) + 0x0C && !val) {
            qemu_irq_lower(s->irq[c]);
        }
    }
}

static const MemoryRegionOps xm_vi_ops = {
    .read = xm_vi_read,
    .write = xm_vi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_vi_reset(DeviceState *dev)
{
    XmViState *s = XM_VI(dev);
    int n;

    memset(s->regs, 0, sizeof(s->regs));
    for (n = 0; n < XM_VI_CHANNELS; n++) {
        qemu_irq_lower(s->irq[n]);
        s->frame[n] = 0;
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void xm_vi_init(Object *obj)
{
    XmViState *s = XM_VI(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &xm_vi_ops, s, TYPE_XM_VI,
                          XM_VI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (n = 0; n < XM_VI_CHANNELS; n++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[n]);
    }
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, xm_vi_tick, s);
}

static const Property xm_vi_props[] = {
    DEFINE_PROP_UINT32("fps", XmViState, fps, 10),
};

static void xm_vi_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_vi_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_vi_props);
}

static const TypeInfo xm_vi_info = {
    .name          = TYPE_XM_VI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmViState),
    .instance_init = xm_vi_init,
    .class_init    = xm_vi_class_init,
};

static void xm_vi_register_types(void)
{
    type_register_static(&xm_vi_info);
}

type_init(xm_vi_register_types)
