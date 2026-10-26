/*
 * Grain Media FTINTC030 interrupt controller.
 *
 * Faraday's FTINTC030 as the GM813x parts carry it at 0x96000000
 * (arch/arm/mach-GM/ftintc030.c, include/mach/ftintc030.h and
 * entry-macro.S of the GM 3.3 kernel).  Interrupts come in groups of 32;
 * group g's registers sit at g * 0x20 for the first two groups and at
 * (g + 1) * 0x20 after that (0x40 is skipped):
 *
 *   +0x00 SOURCE  raw input levels (read only)
 *   +0x04 ENABLE  the kernel writes all ones and leaves it
 *   +0x08 CLEAR   write 1 to clear an edge-triggered input's latch
 *   +0x0C MODE    1 = edge triggered
 *   +0x10 LEVEL   1 = active low / falling edge
 *   +0x14 STATUS  pending and enabled
 *
 * Masking is done per CPU in the target registers: 0x420 + 4g routes
 * group g to CPU 0's FIQ, 0x45C + 4g to its IRQ.  The kernel's mask and
 * unmask rewrite 0x45C (and read it back, an "IC bug" workaround), and its
 * IRQ entry ANDs STATUS with the same word -- so the line to the CPU is
 * STATUS & target.  nsboot routes interrupt 8 (the PMU's frequency change
 * done) this way to wake the core from its WFI.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_INTC "gm-intc"
OBJECT_DECLARE_SIMPLE_TYPE(GmIntcState, GM_INTC)

#define GM_INTC_MAX_GROUPS  4
#define GM_INTC_SIZE        0x1000

#define R_SOURCE            0x00
#define R_ENABLE            0x04
#define R_CLEAR             0x08
#define R_MODE              0x0C
#define R_LEVEL             0x10
#define R_STATUS            0x14

#define R_CPU0_FIQ          0x420
#define R_CPU0_IRQ          0x45C

struct GmIntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;
    uint32_t num_irq;

    uint32_t input[GM_INTC_MAX_GROUPS];     /* line levels from devices */
    uint32_t latch[GM_INTC_MAX_GROUPS];     /* edge-triggered pending */
    uint32_t enable[GM_INTC_MAX_GROUPS];
    uint32_t mode[GM_INTC_MAX_GROUPS];
    uint32_t level[GM_INTC_MAX_GROUPS];
    uint32_t irq_target[GM_INTC_MAX_GROUPS];
    uint32_t fiq_target[GM_INTC_MAX_GROUPS];
};

static unsigned gm_intc_groups(GmIntcState *s)
{
    return DIV_ROUND_UP(s->num_irq, 32);
}

/* The group whose register block holds off, or -1. */
static int gm_intc_group_of(GmIntcState *s, hwaddr off)
{
    int g = off / 0x20;

    if (g == 2) {
        return -1;
    }
    if (g > 2) {
        g--;
    }
    return g < gm_intc_groups(s) ? g : -1;
}

static uint32_t gm_intc_pending(GmIntcState *s, int g)
{
    /* Level inputs follow the line, through the polarity bit. */
    uint32_t lvl = (s->input[g] ^ s->level[g]) & ~s->mode[g];

    return (lvl | (s->latch[g] & s->mode[g])) & s->enable[g];
}

static void gm_intc_update(GmIntcState *s)
{
    bool irq = false, fiq = false;
    int g;

    for (g = 0; g < gm_intc_groups(s); g++) {
        uint32_t p = gm_intc_pending(s, g);

        irq |= (p & s->irq_target[g]) != 0;
        fiq |= (p & s->fiq_target[g]) != 0;
    }
    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->fiq, fiq);
}

static void gm_intc_set_irq(void *opaque, int n, int level)
{
    GmIntcState *s = opaque;
    int g = n / 32;
    uint32_t bit = 1u << (n % 32);
    uint32_t old = s->input[g];

    if (level) {
        s->input[g] |= bit;
    } else {
        s->input[g] &= ~bit;
    }
    /* Edge mode latches the active edge: rising, or falling with LEVEL. */
    if ((old ^ s->input[g]) & bit) {
        bool active = !!(s->input[g] & bit) != !!(s->level[g] & bit);

        if (active) {
            s->latch[g] |= bit;
        }
    }
    gm_intc_update(s);
}

static uint64_t gm_intc_read(void *opaque, hwaddr off, unsigned size)
{
    GmIntcState *s = opaque;
    int g;

    if (off >= R_CPU0_FIQ && off < R_CPU0_FIQ + 4 * GM_INTC_MAX_GROUPS) {
        return s->fiq_target[(off - R_CPU0_FIQ) / 4];
    }
    if (off >= R_CPU0_IRQ && off < R_CPU0_IRQ + 4 * GM_INTC_MAX_GROUPS) {
        return s->irq_target[(off - R_CPU0_IRQ) / 4];
    }
    g = off < 0x200 ? gm_intc_group_of(s, off) : -1;
    if (g < 0) {
        qemu_log_mask(LOG_UNIMP, "%s: read 0x%" HWADDR_PRIx "\n",
                      __func__, off);
        return 0;
    }
    switch (off & 0x1F) {
    case R_SOURCE:
        return s->input[g];
    case R_ENABLE:
        return s->enable[g];
    case R_MODE:
        return s->mode[g];
    case R_LEVEL:
        return s->level[g];
    case R_STATUS:
        return gm_intc_pending(s, g);
    }
    return 0;
}

static void gm_intc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmIntcState *s = opaque;
    int g;

    if (off >= R_CPU0_FIQ && off < R_CPU0_FIQ + 4 * GM_INTC_MAX_GROUPS) {
        s->fiq_target[(off - R_CPU0_FIQ) / 4] = val;
        gm_intc_update(s);
        return;
    }
    if (off >= R_CPU0_IRQ && off < R_CPU0_IRQ + 4 * GM_INTC_MAX_GROUPS) {
        s->irq_target[(off - R_CPU0_IRQ) / 4] = val;
        gm_intc_update(s);
        return;
    }
    g = off < 0x200 ? gm_intc_group_of(s, off) : -1;
    if (g < 0) {
        qemu_log_mask(LOG_UNIMP, "%s: write 0x%" HWADDR_PRIx " = 0x%"
                      PRIx64 "\n", __func__, off, val);
        return;
    }
    switch (off & 0x1F) {
    case R_ENABLE:
        s->enable[g] = val;
        break;
    case R_CLEAR:
        s->latch[g] &= ~val;
        break;
    case R_MODE:
        s->mode[g] = val;
        break;
    case R_LEVEL:
        s->level[g] = val;
        break;
    }
    gm_intc_update(s);
}

static const MemoryRegionOps gm_intc_ops = {
    .read = gm_intc_read,
    .write = gm_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_intc_reset(DeviceState *dev)
{
    GmIntcState *s = GM_INTC(dev);

    memset(s->latch, 0, sizeof(s->latch));
    memset(s->enable, 0, sizeof(s->enable));
    memset(s->mode, 0, sizeof(s->mode));
    memset(s->level, 0, sizeof(s->level));
    memset(s->irq_target, 0, sizeof(s->irq_target));
    memset(s->fiq_target, 0, sizeof(s->fiq_target));
    gm_intc_update(s);
}

static void gm_intc_realize(DeviceState *dev, Error **errp)
{
    GmIntcState *s = GM_INTC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!s->num_irq || s->num_irq > 32 * GM_INTC_MAX_GROUPS) {
        error_setg(errp, "gm-intc: num-irq must be 1..%d",
                   32 * GM_INTC_MAX_GROUPS);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &gm_intc_ops, s,
                          TYPE_GM_INTC, GM_INTC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(dev, gm_intc_set_irq, s->num_irq);
}

static const Property gm_intc_props[] = {
    DEFINE_PROP_UINT32("num-irq", GmIntcState, num_irq, 64),
};

static const VMStateDescription vmstate_gm_intc = {
    .name = TYPE_GM_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(input, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(latch, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(enable, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(mode, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(level, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(irq_target, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_UINT32_ARRAY(fiq_target, GmIntcState, GM_INTC_MAX_GROUPS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_intc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_intc_realize;
    device_class_set_legacy_reset(dc, gm_intc_reset);
    device_class_set_props(dc, gm_intc_props);
    dc->vmsd = &vmstate_gm_intc;
}

static const TypeInfo gm_intc_info = {
    .name           = TYPE_GM_INTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmIntcState),
    .class_init     = gm_intc_class_init,
};

static void gm_intc_register_types(void)
{
    type_register_static(&gm_intc_info);
}

type_init(gm_intc_register_types)
