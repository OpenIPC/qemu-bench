/*
 * Xiongmai DVR VPSS (xm_vpss.ko, 0x33000000, 1 MiB, IRQ 65): a memory-to-
 * memory scaler, one job at a time.  The driver writes a job, writes 1 to
 * +0x20 and sleeps until +0x20 reads 0; its interrupt handler clears +0x44.
 *
 * XM8521DV200 / XM8536E (layout 0):
 *   +0x1000 / +0x1010  source Y / C, phys - 0x80000000 (NV12)
 *   +0x1008 / +0x1018  source Y / C stride
 *   +0x2028 / +0x1020  source width / height
 *   output c, 0..3: store O = 0x30000 + c * 0x2000 (+0x00 Y, +0x10 C, as
 *   the source; +0x20 stride; +0x34 1 = written by this job), window
 *   W = 0x20000 + c * 0x1000 (+0x0 x0, +0x4 x1, +0x8 y0, +0xC y1; the end
 *   exclusive, -1 = off)
 *   +0x3F01C reads 0x58000 (the AXI idle state the driver checks)
 * XM8520D (layout 1): addresses and strides in 16-byte units ((phys -
 * 0x80000000) >> 4); no source size registers: the height is (C - Y) /
 * stride, the width the processing width +0x0C (scaled by the pre-scaler
 * +0x3000 when that is not 0x400); five outputs, store at 0x24000 + c *
 * 0x1000 (+0x34 active), window at 0x10000 + c * 0x1000.
 *
 * A job scales the whole source (nearest) into each active output's window
 * size; output 0 may be the source buffer itself (same size: nothing to
 * do).  It completes at once.  Every other word is RAM-backed.  Read from
 * the vendor module (no public source).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_XM_DVR_VPSS "xm-dvr-vpss"
OBJECT_DECLARE_SIMPLE_TYPE(XmDvrVpssState, XM_DVR_VPSS)

#define VPSS_SIZE       (1 * MiB)
#define VPSS_MAX_W      4096
#define DDR_BASE        0x80000000ULL

struct XmDvrVpssState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t *regs;
    uint32_t layout;
    uint64_t jobs;
    uint8_t src[VPSS_MAX_W], dst[VPSS_MAX_W];
};

#define R(o) (s->regs[(o) / 4])

static void xm_dvr_vpss_scale(XmDvrVpssState *s, hwaddr sy, hwaddr sc,
                              uint32_t sys, uint32_t scs, uint32_t sw,
                              uint32_t sh, hwaddr dy, hwaddr dc, uint32_t ds,
                              uint32_t dw, uint32_t dh)
{
    uint32_t x, y, prev = UINT32_MAX;

    if (dy == sy && dw == sw && dh == sh) {
        return;                 /* chn0 pass-through: the source itself */
    }
    for (y = 0; y < dh; y++) {
        uint32_t r = y * (uint64_t)sh / dh;

        if (r != prev) {
            address_space_read(&address_space_memory, sy + (hwaddr)r * sys,
                               MEMTXATTRS_UNSPECIFIED, s->src, sw);
            for (x = 0; x < dw; x++) {
                s->dst[x] = s->src[x * (uint64_t)sw / dw];
            }
            prev = r;
        }
        address_space_write(&address_space_memory, dy + (hwaddr)y * ds,
                            MEMTXATTRS_UNSPECIFIED, s->dst, dw);
    }
    prev = UINT32_MAX;
    for (y = 0; y < dh / 2; y++) {
        uint32_t r = y * (uint64_t)(sh / 2) / (dh / 2);

        if (r != prev) {
            address_space_read(&address_space_memory, sc + (hwaddr)r * scs,
                               MEMTXATTRS_UNSPECIFIED, s->src, sw & ~1u);
            for (x = 0; x < dw / 2; x++) {
                uint32_t sx = x * (uint64_t)(sw / 2) / (dw / 2);

                s->dst[2 * x] = s->src[2 * sx];
                s->dst[2 * x + 1] = s->src[2 * sx + 1];
            }
            prev = r;
        }
        address_space_write(&address_space_memory, dc + (hwaddr)y * ds,
                            MEMTXATTRS_UNSPECIFIED, s->dst, dw & ~1u);
    }
}

static void xm_dvr_vpss_run(XmDvrVpssState *s)
{
    hwaddr sy = DDR_BASE + R(0x1000), sc = DDR_BASE + R(0x1010);
    uint32_t sw = R(0x2028), sh = R(0x1020);
    uint32_t sys = R(0x1008), scs = R(0x1018);
    int c, outs = s->layout ? 5 : 4;

    /* v100: addresses and strides in 16-byte units, and no source size:
     * the height is (C - Y) / stride, the width the processing width
     * (+0x0C) unless the pre-scaler (+0x3000, 0x400 = 1:1) shrinks it */
    if (s->layout) {
        sy = DDR_BASE + ((hwaddr)R(0x1000) << 4);
        sc = DDR_BASE + ((hwaddr)R(0x1010) << 4);
        sys = R(0x1008) << 4;
        scs = R(0x1018) << 4;
        sh = R(0x1008) && R(0x1010) > R(0x1000) ?
             (R(0x1010) - R(0x1000)) / R(0x1008) : 0;
        sw = !R(0x3000) || R(0x3000) == 0x400 ? R(0x0c) :
             R(0x3000) * R(0x0c) / 1024 + 4;
        sw = MIN(sw, sys);
    }

    if (s->jobs++ < 4) {
        qemu_log_mask(LOG_UNIMP, "xm-dvr-vpss: job src %ux%u at 0x%" HWADDR_PRIx
                      " stride %u\n", sw, sh, sy, sys);
    }
    if (!sw || !sh || sw > VPSS_MAX_W || sys < sw || R(0x1000) == 0) {
        return;
    }
    for (c = 0; c < outs; c++) {
        hwaddr o = s->layout ? 0x24000 + c * 0x1000 : 0x30000 + c * 0x2000;
        hwaddr w = s->layout ? 0x10000 + c * 0x1000 : 0x20000 + c * 0x1000;
        uint32_t x0 = R(w), x1 = R(w + 4), y0 = R(w + 8), y1 = R(w + 12);
        uint32_t dw, dh, ds = s->layout ? R(o + 0x20) << 4 : R(o + 0x20);
        hwaddr dy = s->layout ? (hwaddr)R(o) << 4 : R(o);
        hwaddr dc = s->layout ? (hwaddr)R(o + 0x10) << 4 : R(o + 0x10);

        if (!R(o + 0x34) || x1 == UINT32_MAX || y1 == UINT32_MAX ||
            x1 <= x0 || y1 <= y0) {
            continue;
        }
        dw = x1 - x0;
        dh = y1 - y0;
        if (dw > VPSS_MAX_W || ds < dw) {
            continue;
        }
        if (s->jobs < 5) {
            qemu_log_mask(LOG_UNIMP, "xm-dvr-vpss:  out %d %ux%u at 0x%x "
                          "stride %u\n", c, dw, dh, R(o), ds);
        }
        xm_dvr_vpss_scale(s, sy, sc, sys, scs, sw, sh, DDR_BASE + dy,
                          DDR_BASE + dc, ds, dw, dh);
    }
}

static uint64_t xm_dvr_vpss_read(void *opaque, hwaddr off, unsigned size)
{
    XmDvrVpssState *s = opaque;

    if (off == 0x3F01C && !s->layout) {
        return 0x58000;
    }
    return R(off);
}

static void xm_dvr_vpss_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    XmDvrVpssState *s = opaque;

    R(off) = val;
    if (off == 0x20 && (val & 1)) {
        xm_dvr_vpss_run(s);
        R(0x20) = 0;
        R(0x44) = 1;
    }
    qemu_set_irq(s->irq, R(0x44) != 0);
}

static const MemoryRegionOps xm_dvr_vpss_ops = {
    .read = xm_dvr_vpss_read,
    .write = xm_dvr_vpss_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_dvr_vpss_reset(DeviceState *dev)
{
    XmDvrVpssState *s = XM_DVR_VPSS(dev);

    memset(s->regs, 0, VPSS_SIZE);
    qemu_irq_lower(s->irq);
}

static void xm_dvr_vpss_realize(DeviceState *dev, Error **errp)
{
    XmDvrVpssState *s = XM_DVR_VPSS(dev);

    s->regs = g_malloc0(VPSS_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm_dvr_vpss_ops, s,
                          TYPE_XM_DVR_VPSS, VPSS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const Property xm_dvr_vpss_props[] = {
    DEFINE_PROP_UINT32("layout", XmDvrVpssState, layout, 0),
};

static void xm_dvr_vpss_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_dvr_vpss_realize;
    device_class_set_legacy_reset(dc, xm_dvr_vpss_reset);
    device_class_set_props(dc, xm_dvr_vpss_props);
}

static const TypeInfo xm_dvr_vpss_info = {
    .name = TYPE_XM_DVR_VPSS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmDvrVpssState),
    .class_init = xm_dvr_vpss_class_init,
};

static void xm_dvr_vpss_register(void)
{
    type_register_static(&xm_dvr_vpss_info);
}

type_init(xm_dvr_vpss_register)
