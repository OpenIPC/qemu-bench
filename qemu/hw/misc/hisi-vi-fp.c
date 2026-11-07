/*
 * HiSilicon Video Input Frame Producer (QEMU emulation helper).
 *
 * MVP scaffolding to wake the vendor VI / VPSS / VENC pipeline on QEMU.
 *
 * On real silicon the MIPI receiver delivers a CSI-2 frame to a VI capture
 * buffer in DDR, and the VI hardware fires the VI_CAP IRQ (followed by
 * VI_PROC then VPSS, and VEDU once the frame walks through the pipeline).
 * Without any of that hardware on QEMU the vendor VENC blob blocks forever
 * in its "wait for next encoded frame" loop, hence Majestic's repeating
 *   "Timeout from venc channel 0"
 * messages.
 *
 * The three VI / VPSS IRQ lines are fired on a periodic timer matching the
 * configured sensor framerate (default 5 fps).  Per-line delivery is
 * gated by the QOM property `autoclear_mask`:
 *   bit 0 = VI_CAP    (always safe — handler validates with status read)
 *   bit 1 = VI_PROC
 *   bit 2 = VPSS      (vendor handler NULL-derefs without group state)
 *
 * VEDU is not among them: hisi-vedu encodes the pictures the driver
 * starts and raises its own end-of-picture interrupt.
 *
 * Each enabled line uses a small MMIO subregion overlapping the IRQ
 * status register the vendor handler reads — a guest read returns the
 * magic poke value and lowers the corresponding IRQ line, mimicking
 * real silicon's "auto-clear on read" semantics.  This is required
 * because QEMU's GIC has one input level per SPI; without per-line
 * deassert we'd starve all but the highest-priority pending IRQ.
 *
 * Override at runtime: `-global hisi-vi-fp.autoclear_mask=0xN`.
 * Default 0x1 (VI_CAP only) is the conservative baseline — enabling
 * VI_PROC / VPSS silently reboots the kernel because their
 * vendor handlers expect channel state allocated lazily on first
 * frame, which we don't model.
 *
 * Wired only on hi3516ev300 as MVP (HisiSoCConfig.vi_fp_base != 0).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"

#define TYPE_HISI_VI_FP "hisi-vi-fp"
OBJECT_DECLARE_SIMPLE_TYPE(HisiViFpState, HISI_VI_FP)

#define VI_FP_REGION_SIZE  0x100

/* IRQ line indices.  Order matches autoclear_mask bit layout. */
enum {
    VI_FP_IRQ_CAP,
    VI_FP_IRQ_PROC,
    VI_FP_IRQ_VPSS,
    VI_FP_NUM_IRQS,
};

/*
 * Per-line auto-clear status overlay.  A small (4-byte) MMIO region
 * placed at higher priority than the underlying regbank RAM so guest
 * reads of the magic IRQ status register hit our handler:
 *
 *   read  -> return current value, then clear it and lower the IRQ
 *   write -> ignored (vendor occasionally W1C-acks, no-op for us)
 *
 * The frame_timer poke (re-)arms 'value' to poke_value once per pulse.
 *
 * Active only when the corresponding bit of autoclear_mask is set.
 */
typedef struct AutoClearReg {
    struct HisiViFpState *parent;
    int                   irq_idx;
    uint32_t              poke_value;
    uint32_t              value;
    bool                  active;
    MemoryRegion          mr;
    const char           *name;
    hwaddr                addr;
} AutoClearReg;

/* Status registers we overlay.  Reverse-engineered from
 *   /home/john-1/git/openhisilicon/kernel/{vi,vpss}/{vi,vpss}.o
 *
 *   VI_CAP0+0xF0 : VI_HAL_GetCapIntStatus / VI_DRV_IsCapValidInt
 *                  validates (status & 0xF00) != 0; without this the
 *                  kernel disables IRQ #42 as spurious.
 *   VI_PROC0+0x310 : VI_HAL_GetProcIrqStatus / VI_HAL_IsProcIrqValid
 *                  validates status != 0; without this VI_COMM_ProcIrqRoute
 *                  panics at vi.o:1993.  Real-cam capture: 0x06732001.
 *   VPSS+0x314   : VPSS_HAL_GetIntStatus reads masked IRQ status.
 *                  Bit 0 = EOF (end of frame). */
static const struct {
    hwaddr      addr;
    uint32_t    poke_value;
    int         irq_idx;
    const char *name;
} hisi_vi_fp_autoclear_descs[VI_FP_NUM_IRQS] = {
    [VI_FP_IRQ_CAP]  = { 0x110000F0, 0x00000F00, VI_FP_IRQ_CAP,  "VI_CAP0+0xF0"   },
    [VI_FP_IRQ_PROC] = { 0x11200310, 0x06732001, VI_FP_IRQ_PROC, "VI_PROC0+0x310" },
    [VI_FP_IRQ_VPSS] = { 0x11400314, 0x00000001, VI_FP_IRQ_VPSS, "VPSS+0x314"     },
};

struct HisiViFpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* IRQ outputs, in pipeline order.  Indexed by VI_FP_IRQ_*. */
    qemu_irq     irqs[VI_FP_NUM_IRQS];

    /* Sensor framerate (default 5 fps). */
    uint32_t     fps;

    /* Bitmask of IRQs to fire AND give per-line autoclear-on-read.
     * Bit N = VI_FP_IRQ_*.  Bits not set in the mask have no IRQ
     * delivery from the heartbeat at all — useful for narrowing
     * down which vendor IRQ handler crashes when invoked without
     * its expected channel state. */
    uint32_t     autoclear_mask;

    QEMUTimer   *frame_timer;

    uint32_t     enable;
    uint64_t     pulses;

    AutoClearReg autoclears[VI_FP_NUM_IRQS];
};

#define VI_FP_REG_CTRL    0x000
#define VI_FP_REG_STATUS  0x004
#define VI_FP_REG_FPS     0x008

static uint64_t hisi_vi_fp_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViFpState *s = HISI_VI_FP(opaque);
    switch (offset) {
    case VI_FP_REG_CTRL:   return s->enable;
    case VI_FP_REG_STATUS: return (uint32_t)(s->pulses & 0xFFFFFFFF);
    case VI_FP_REG_FPS:    return s->fps;
    }
    return 0;
}

static void hisi_vi_fp_arm_timer(HisiViFpState *s)
{
    uint32_t period_ms = s->fps ? (1000U / s->fps) : 200U;
    timer_mod(s->frame_timer,
              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + period_ms);
}

static void hisi_vi_fp_lower_all_irqs(HisiViFpState *s)
{
    int i;
    for (i = 0; i < VI_FP_NUM_IRQS; i++) {
        qemu_set_irq(s->irqs[i], 0);
        s->autoclears[i].value = 0;
    }
}

static void hisi_vi_fp_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    HisiViFpState *s = HISI_VI_FP(opaque);

    switch (offset) {
    case VI_FP_REG_CTRL: {
        uint32_t prev = s->enable;
        s->enable = val & 1;
        if (s->enable && !prev) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-vi-fp: heartbeat ENABLED at %u fps "
                          "(autoclear_mask=0x%x)\n",
                          s->fps, s->autoclear_mask);
            hisi_vi_fp_arm_timer(s);
        } else if (!s->enable && prev) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-vi-fp: heartbeat DISABLED (pulses=%llu)\n",
                          (unsigned long long)s->pulses);
            timer_del(s->frame_timer);
            hisi_vi_fp_lower_all_irqs(s);
        }
        return;
    }
    case VI_FP_REG_FPS:
        if (val) {
            s->fps = val;
        }
        return;
    }
}

static const MemoryRegionOps hisi_vi_fp_ops = {
    .read = hisi_vi_fp_read,
    .write = hisi_vi_fp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t hisi_vi_fp_autoclear_read(void *opaque, hwaddr offset,
                                          unsigned size)
{
    AutoClearReg *ac = opaque;
    HisiViFpState *s = ac->parent;
    uint32_t val = ac->value;
    ac->value = 0;
    qemu_set_irq(s->irqs[ac->irq_idx], 0);
    return val;
}

static void hisi_vi_fp_autoclear_write(void *opaque, hwaddr offset,
                                       uint64_t val, unsigned size)
{
    /* Vendor may W1C the same register; redundant in our model. */
}

static const MemoryRegionOps hisi_vi_fp_autoclear_ops = {
    .read = hisi_vi_fp_autoclear_read,
    .write = hisi_vi_fp_autoclear_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vi_fp_tick(void *opaque)
{
    HisiViFpState *s = HISI_VI_FP(opaque);
    int i;

    if (!s->enable) {
        return;
    }

    /* Only fire lines selected by the mask.  Bits OUT of the mask are
     * NOT delivered to the GIC at all — empirically, invoking the
     * VPSS vendor handler without its channel state silently
     * reboots the kernel.  Default 0x1 = VI_CAP only matches what
     * worked in PR #58 plus per-line auto-clear (instead of the
     * fallback deassert timer that PR used). */
    for (i = 0; i < VI_FP_NUM_IRQS; i++) {
        if (s->autoclear_mask & (1u << i)) {
            s->autoclears[i].value =
                hisi_vi_fp_autoclear_descs[i].poke_value;
            qemu_set_irq(s->irqs[i], 1);
        }
    }

    s->pulses++;
    if (s->pulses <= 5 || (s->pulses % 25) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-vi-fp: pulse #%llu (mask=0x%x)\n",
                      (unsigned long long)s->pulses, s->autoclear_mask);
    }

    hisi_vi_fp_arm_timer(s);
}

static void hisi_vi_fp_realize(DeviceState *dev, Error **errp)
{
    HisiViFpState *s = HISI_VI_FP(dev);
    int i;

    s->frame_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                  hisi_vi_fp_tick, s);

    /* Map per-line auto-clear overlays for all three lines (mask is
     * checked at tick/read time, not realize time, so users can flip
     * it at runtime via QOM if needed). */
    for (i = 0; i < VI_FP_NUM_IRQS; i++) {
        AutoClearReg *ac = &s->autoclears[i];
        ac->parent     = s;
        ac->irq_idx    = hisi_vi_fp_autoclear_descs[i].irq_idx;
        ac->poke_value = hisi_vi_fp_autoclear_descs[i].poke_value;
        ac->value      = 0;
        ac->name       = hisi_vi_fp_autoclear_descs[i].name;
        ac->addr       = hisi_vi_fp_autoclear_descs[i].addr;
        memory_region_init_io(&ac->mr, OBJECT(s),
                              &hisi_vi_fp_autoclear_ops, ac,
                              ac->name, 4);
        memory_region_add_subregion_overlap(get_system_memory(),
                                            ac->addr, &ac->mr, 1);
    }
}

static void hisi_vi_fp_init(Object *obj)
{
    HisiViFpState *s = HISI_VI_FP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &hisi_vi_fp_ops, s,
                          "hisi-vi-fp", VI_FP_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    for (i = 0; i < VI_FP_NUM_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irqs[i]);
    }
}

static void hisi_vi_fp_reset(DeviceState *dev)
{
    HisiViFpState *s = HISI_VI_FP(dev);
    s->pulses = 0;
    s->enable = 0;
    if (s->frame_timer) {
        timer_del(s->frame_timer);
    }
    hisi_vi_fp_lower_all_irqs(s);
}

static const Property hisi_vi_fp_properties[] = {
    DEFINE_PROP_UINT32("fps", HisiViFpState, fps, 5),
    /* Bit 0 = VI_CAP, 1 = VI_PROC, 2 = VPSS.
     * Default 0x1 — only VI_CAP, the only line whose vendor handler
     * is empirically safe to invoke without channel state set up. */
    DEFINE_PROP_UINT32("autoclear_mask", HisiViFpState, autoclear_mask, 0x1),
};

static void hisi_vi_fp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = hisi_vi_fp_realize;
    device_class_set_legacy_reset(dc, hisi_vi_fp_reset);
    device_class_set_props(dc, hisi_vi_fp_properties);
}

static const TypeInfo hisi_vi_fp_info = {
    .name          = TYPE_HISI_VI_FP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiViFpState),
    .instance_init = hisi_vi_fp_init,
    .class_init    = hisi_vi_fp_class_init,
};

static void hisi_vi_fp_register_types(void)
{
    type_register_static(&hisi_vi_fp_info);
}

type_init(hisi_vi_fp_register_types)
