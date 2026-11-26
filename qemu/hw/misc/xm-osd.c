/*
 * Xiongmai second-generation OSD engine (0x21200000, 4 KiB), as rgn.ko's
 * osd_process drives it: the job's registers, then 1 to +0x60 starts it;
 * the driver waits for +0xF0 (done) and its osd_int (SPI 62) checks +0xFC
 * and acknowledges by writing 0 to +0xF4.
 *
 * A job blends up to two regions (+0x64: their number - 1) into a frame in
 * place, NV12; region n's words at +0x100 * n (read from Sofia's clock and
 * title jobs):
 *   +0x00  enable              +0x04  4: 2 bits a pixel
 *   +0x10..+0x1C  the four colours, A Y U V from the top byte (0 clear)
 *   +0x24 / +0x28  1: doubled across / down
 *   +0x2C  the frame's stride  +0x30  the bitmap's stride, bytes
 *   +0x34  output height - 1   +0x4C  output width
 *   +0x40  the bitmap, most significant pixel first
 *   +0x44 / +0x48  luma / chroma of the region's top-left, read from
 *   +0x50 / +0x54  ...and written to (the same, in place)
 *
 * Jobs complete a millisecond after they start.  Without the completion
 * each job waits out rgn.ko's timeout ("osd: timeout"), which throttles the
 * whole encode pipeline.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/xm-regtrace.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

#define TYPE_XM_OSD "xm-osd"
OBJECT_DECLARE_SIMPLE_TYPE(XmOsdState, XM_OSD)

#define XM_OSD_SIZE     0x1000
#define OSD_START       0x60
#define OSD_DONE        0xF0
#define OSD_INT_ACK     0xF4
#define OSD_INT_ST      0xFC

struct XmOsdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_OSD_SIZE / 4];
    bool trace;
};

static void xm_osd_update(XmOsdState *s)
{
    qemu_set_irq(s->irq, s->regs[OSD_INT_ST / 4] != 0);
}

#define OSD_REGIONS     0x64
#define OSD_REGION(n)   (0x100 * (n))

static uint8_t xm_osd_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

static void xm_osd_region(XmOsdState *s, int n)
{
    const uint32_t *r = &s->regs[OSD_REGION(n) / 4];
    const int sx2 = r[0x24 / 4] & 1, sy2 = r[0x28 / 4] & 1;
    const int stride = r[0x2C / 4], bstride = r[0x30 / 4];
    const int h = r[0x34 / 4] + 1, w = r[0x4C / 4];
    const int bh = (h + sy2) >> sy2;
    uint32_t pal[4];

    /* bitmap pixels a row: rounded up when doubled across */
    if (!r[0] || r[0x04 / 4] != 4 || w <= 0 || h <= 0 || w > 4096 ||
        h > 4096 || stride < w || stride > 16384 || bstride > 4096 ||
        bstride * 4 < ((w + sx2) >> sx2) || !r[0x40 / 4]) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        pal[i] = r[(0x10 + 4 * i) / 4];
    }
    g_autofree uint8_t *bmp = g_malloc((size_t)bstride * bh);
    g_autofree uint8_t *line = g_malloc(w);
    dma_memory_read(&address_space_memory, r[0x40 / 4], bmp,
                    (size_t)bstride * bh, MEMTXATTRS_UNSPECIFIED);
    for (int y = 0; y < h; y++) {
        const uint8_t *b = bmp + (size_t)(y >> sy2) * bstride;
        const hwaddr in = r[0x44 / 4] + (hwaddr)y * stride;
        const hwaddr out = r[0x50 / 4] + (hwaddr)y * stride;

        dma_memory_read(&address_space_memory, in, line, w,
                        MEMTXATTRS_UNSPECIFIED);
        for (int x = 0; x < w; x++) {
            const int bx = x >> sx2;
            const uint32_t c = pal[(b[bx / 4] >> (6 - 2 * (bx % 4))) & 3];

            line[x] = xm_osd_blend(line[x], c >> 16, c >> 24);
        }
        dma_memory_write(&address_space_memory, out, line, w,
                         MEMTXATTRS_UNSPECIFIED);
        if (y & 1) {
            continue;
        }
        /* chroma: a CbCr pair for each 2x2, from the pair's left pixel */
        const hwaddr cin = r[0x48 / 4] + (hwaddr)(y / 2) * stride;
        const hwaddr cout = r[0x54 / 4] + (hwaddr)(y / 2) * stride;

        dma_memory_read(&address_space_memory, cin, line, w & ~1,
                        MEMTXATTRS_UNSPECIFIED);
        for (int x = 0; x + 1 < w; x += 2) {
            const int bx = x >> sx2;
            const uint32_t c = pal[(b[bx / 4] >> (6 - 2 * (bx % 4))) & 3];

            line[x] = xm_osd_blend(line[x], c >> 8, c >> 24);
            line[x + 1] = xm_osd_blend(line[x + 1], c, c >> 24);
        }
        dma_memory_write(&address_space_memory, cout, line, w & ~1,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void xm_osd_done(void *opaque)
{
    XmOsdState *s = XM_OSD(opaque);

    s->regs[OSD_DONE / 4] = 1;
    s->regs[OSD_INT_ST / 4] = 1;
    xm_osd_update(s);
}

static uint64_t xm_osd_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_OSD(opaque)->regs[off / 4];
}

static void xm_osd_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    XmOsdState *s = XM_OSD(opaque);

    xm_regtrace("osd", off, val);
    if (s->trace) {
        qemu_log("xm-osd: +0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n", off, val);
    }
    switch (off) {
    case OSD_START:
        s->regs[off / 4] = val;
        if (val & 1) {
            const int regions = MIN(s->regs[OSD_REGIONS / 4] + 1, 2);

            for (int n = 0; n < regions; n++) {
                xm_osd_region(s, n);
            }
            s->regs[OSD_DONE / 4] = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCALE_MS);
        }
        break;
    case OSD_INT_ACK:
        s->regs[off / 4] = val;
        if (!val) {
            s->regs[OSD_INT_ST / 4] = 0;
        }
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_osd_update(s);
}

static const MemoryRegionOps xm_osd_ops = {
    .read = xm_osd_read,
    .write = xm_osd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_osd_reset(DeviceState *dev)
{
    XmOsdState *s = XM_OSD(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    xm_osd_update(s);
}

static void xm_osd_init(Object *obj)
{
    XmOsdState *s = XM_OSD(obj);

    memory_region_init_io(&s->iomem, obj, &xm_osd_ops, s, TYPE_XM_OSD,
                          XM_OSD_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_osd_done, s);
}

static const Property xm_osd_props[] = {
    DEFINE_PROP_BOOL("trace", XmOsdState, trace, false),
};

static void xm_osd_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_osd_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_osd_props);
}

static const TypeInfo xm_osd_info = {
    .name          = TYPE_XM_OSD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmOsdState),
    .instance_init = xm_osd_init,
    .class_init    = xm_osd_class_init,
};

static void xm_osd_register_types(void)
{
    type_register_static(&xm_osd_info);
}

type_init(xm_osd_register_types)
