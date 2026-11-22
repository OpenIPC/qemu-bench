/*
 * Xiongmai XM232 "VTA" neural network accelerator (XmVtaMte.ko's
 * vta_ip_base 0x22000000, vta_irq_num 59 = SPI 27), which Sofia's person
 * detector (pd_l_d1_xmvta2_0.*) drives through /dev/mem: a colour
 * converter that makes the network's input from a VI channel, modelled,
 * and the network engine, whose runs complete with nothing computed.
 * Without it the detector's runs time out, its outputs stay unset and
 * Sofia's Smart_Proc thread faults on them about 30 s after start.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_XM_VTA "xm-vta"
OBJECT_DECLARE_SIMPLE_TYPE(XmVtaState, XM_VTA)

#define VTA_SIZE    0x1000

/*
 * The colour converter at +0x800, which turns a VI channel (NV12) into the
 * network's int8 input, a job a picture:
 *   +0x808 / +0x80C  width / height       +0x810 / +0x814  luma / chroma
 *   +0x818           their stride         +0x81C           output
 *   +0x820           output stride (3 bytes a pixel)
 *   three rows of coefficients, each output byte in turn: +0x82C Y, Cb,
 *   offset, shift; +0x83C Y, Cr, Cb, offset, shift; +0x850 Y, Cr, offset,
 *   shift (Q12, the offsets folding in the chroma's -128); +0x860..+0x874
 *   each byte's minimum and maximum
 *   +0x8F0           1 before the start  +0x800  1: start
 *   +0x8F4           done; written 1 to clear (Sofia polls it, then
 *                    clears it and waits for it to read 0)
 */
#define CSC_START   0x800
#define CSC_W       0x808
#define CSC_H       0x80C
#define CSC_Y       0x810
#define CSC_C       0x814
#define CSC_STRIDE  0x818
#define CSC_OUT     0x81C
#define CSC_OSTRIDE 0x820
#define CSC_ROW0    0x82C
#define CSC_ROW1    0x83C
#define CSC_ROW2    0x850
#define CSC_CLAMP   0x860
#define CSC_DONE    0x8F4
/*
 * The network engine, as Sofia's runtime runs it (FUN_00410868): +0x018
 * the instruction count - 1 and +0x020 the instructions' address (the
 * model's .insn file, loaded into the MMZ), +0x028..+0x048, +0x100 and
 * +0x210 zero, then +0x000 1.  It waits up to 5 s for the interrupt on
 * XmVtaMte.ko's device (without the device it polls +0x008 for 1); the
 * driver's handler writes 0 to +0x008.  On a timeout it writes +0x010 1 to
 * halt the engine, polls +0x218 for bits 0x30 (idle), resets it through
 * the system controller (0x20000124) and drops the picture.  Nothing is
 * computed: a run completes a millisecond after its start, its outputs as
 * they were, so the person detector finds nobody.
 */
#define ENG_START   0x000
#define ENG_IRQ     0x008     /* 1: done, the interrupt; written 0 */
#define ENG_HALT    0x010
#define ENG_STATUS  0x218     /* 0x30: idle */
#define ENG_IDLE    0x30

struct XmVtaState {
    SysBusDevice parent_obj;
    MemoryRegion io;
    uint32_t regs[VTA_SIZE / 4];
    qemu_irq irq;
    QEMUTimer *timer;
    bool trace;
};

#define R(off) ((int32_t)s->regs[(off) / 4])

static uint8_t xm_vta_clamp(XmVtaState *s, int k, int64_t v)
{
    return MIN(MAX(v, R(CSC_CLAMP + 8 * k)), R(CSC_CLAMP + 8 * k + 4));
}

static void xm_vta_csc(XmVtaState *s)
{
    const int w = R(CSC_W), h = R(CSC_H), st = R(CSC_STRIDE);
    const int ost = R(CSC_OSTRIDE);
    const int s0 = R(CSC_ROW0 + 12) & 31, s1 = R(CSC_ROW1 + 16) & 31;
    const int s2 = R(CSC_ROW2 + 12) & 31;

    /* NV12: whole chroma pairs and rows; strides no longer than a row of
     * the largest picture needs */
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096 || (w & 1) || (h & 1) ||
        st < w || st > 4096 || ost < 3 * w || ost > 3 * 4096) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-vta: conversion %dx%d (strides "
                      "%d / %d) not done\n", w, h, st, ost);
        return;
    }
    g_autofree uint8_t *y = g_malloc((size_t)st * h);
    g_autofree uint8_t *c = g_malloc((size_t)st * (h / 2));
    g_autofree uint8_t *o = g_malloc((size_t)ost);

    dma_memory_read(&address_space_memory, (uint32_t)R(CSC_Y), y,
                    (size_t)st * h, MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, (uint32_t)R(CSC_C), c,
                    (size_t)st * (h / 2), MEMTXATTRS_UNSPECIFIED);
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            const int64_t Y = y[(size_t)j * st + i];
            const uint8_t *uv = &c[(size_t)(j / 2) * st + (i & ~1)];
            const int64_t cb = uv[0], cr = uv[1];

            o[3 * i] = xm_vta_clamp(s, 0, (R(CSC_ROW0) * Y +
                                           R(CSC_ROW0 + 4) * cb +
                                           R(CSC_ROW0 + 8)) >> s0);
            o[3 * i + 1] = xm_vta_clamp(s, 1, (R(CSC_ROW1) * Y +
                                               R(CSC_ROW1 + 4) * cr +
                                               R(CSC_ROW1 + 8) * cb +
                                               R(CSC_ROW1 + 12)) >> s1);
            o[3 * i + 2] = xm_vta_clamp(s, 2, (R(CSC_ROW2) * Y +
                                               R(CSC_ROW2 + 4) * cr +
                                               R(CSC_ROW2 + 8)) >> s2);
        }
        dma_memory_write(&address_space_memory,
                         (uint32_t)R(CSC_OUT) + (hwaddr)j * ost, o,
                         (size_t)w * 3, MEMTXATTRS_UNSPECIFIED);
    }
}

static uint64_t xm_vta_read(void *opaque, hwaddr off, unsigned size)
{
    XmVtaState *s = XM_VTA(opaque);

    if (s->trace) {
        fprintf(stderr, "xm-vta: R %03" HWADDR_PRIx " = %08x\n", off,
                s->regs[off / 4]);
    }
    return s->regs[off / 4];
}

static void xm_vta_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    XmVtaState *s = XM_VTA(opaque);

    if (s->trace) {
        fprintf(stderr, "xm-vta: W %03" HWADDR_PRIx " %08" PRIx64 "\n", off,
                val);
    }
    if (off == ENG_IRQ) {
        s->regs[off / 4] = val;
        qemu_set_irq(s->irq, val != 0);
        return;
    }
    if (off == ENG_START && (val & 1)) {
        s->regs[ENG_STATUS / 4] = 0;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
    }
    if (off == ENG_HALT && (val & 1)) {
        timer_del(s->timer);
        s->regs[ENG_STATUS / 4] = ENG_IDLE;
    }
    if (off == CSC_DONE) {                  /* write 1 to clear */
        s->regs[off / 4] &= ~(uint32_t)val;
        return;
    }
    s->regs[off / 4] = val;
    if (off == CSC_START) {
        if (val & 1) {
            xm_vta_csc(s);
            s->regs[CSC_DONE / 4] = 1;
        } else {
            s->regs[CSC_DONE / 4] = 0;
        }
    }
}

static void xm_vta_done(void *opaque)
{
    XmVtaState *s = XM_VTA(opaque);

    s->regs[ENG_STATUS / 4] = ENG_IDLE;
    s->regs[ENG_IRQ / 4] = 1;
    qemu_irq_raise(s->irq);
}

static const MemoryRegionOps xm_vta_ops = {
    .read = xm_vta_read,
    .write = xm_vta_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_vta_reset(DeviceState *dev)
{
    XmVtaState *s = XM_VTA(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[ENG_STATUS / 4] = ENG_IDLE;
    timer_del(s->timer);
    qemu_irq_lower(s->irq);
}

static void xm_vta_init(Object *obj)
{
    XmVtaState *s = XM_VTA(obj);

    memory_region_init_io(&s->io, obj, &xm_vta_ops, s, "xm-vta", VTA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->io);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_vta_done, s);
}

static const Property xm_vta_props[] = {
    DEFINE_PROP_BOOL("trace", XmVtaState, trace, false),
};

static void xm_vta_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_vta_reset);
    device_class_set_props(dc, xm_vta_props);
}

static const TypeInfo xm_vta_info = {
    .name          = TYPE_XM_VTA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmVtaState),
    .instance_init = xm_vta_init,
    .class_init    = xm_vta_class_init,
};

static void xm_vta_register_types(void)
{
    type_register_static(&xm_vta_info);
}

type_init(xm_vta_register_types)
