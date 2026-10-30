/*
 * Ingenic XBurst2 core complex: the CCU at 0x12200000 and the per-core
 * interrupt controller at 0x12300000 (T40, T41).
 *
 * The core INTC is the INTC of the older parts once per core, core n at
 * +0x100 n: two banks of ISR/IMR/IMSR/IMCR/IPR, 0x20 apart (drivers/
 * irqchip/irq-ingenic-chip.c).  Every core sees the same source levels
 * through a mask of its own; the 4.4 kernel unmasks a line on one core
 * only and moves it by masking it there and unmasking it on another.
 *
 * The CCU (arch/mips/xburst2/core/include/ccu.h, core/smp.c) adds:
 *
 *   0x020 CSSR  core sleep status: bit n while core n is halted
 *   0x040 CSRR  core reset, 1 = held; core 1 comes out of reset held
 *   0x100 PIPR / 0x120 PIMR   INTC pending / enable to IP2, bit per core
 *   0x140 MIPR / 0x160 MIMR   mailbox pending / enable to IP3
 *   0x180 OIPR / 0x1A0 OIMR   core OST pending / enable to IP4
 *   0xF00 RER   reset entry: where a core released from reset starts
 *   0x1000 + 4 n  MBOX n: IP3 of core n while non-zero (the IPIs)
 *
 * Nothing sets core 0's bits in PIMR and OIMR, so they come out of reset
 * set.  Clearing a core's CSRR bit starts it at RER (a KSEG0 address),
 * as from a reset, and setting it stops the core again; the kernel's
 * boot_secondary() sets the bit, writes RER and clears it again.  The other registers are storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/cpu.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_CCU "ingenic-ccu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicCcuState, INGENIC_CCU)

#define MAX_CORES       2
#define INTC_NR         64

#define R_CSSR          0x020
#define R_CSRR          0x040
#define R_PIPR          0x100
#define R_PIMR          0x120
#define R_MIPR          0x140
#define R_MIMR          0x160
#define R_OIPR          0x180
#define R_OIMR          0x1A0
#define R_RER           0xF00
#define R_MBOX          0x1000
#define CCU_REGS        0x1000

struct IngenicCcuState {
    SysBusDevice parent_obj;

    MemoryRegion ccu_mr, intc_mr;
    uint32_t ncores;
    qemu_irq ip2[MAX_CORES], ip3[MAX_CORES], ip4[MAX_CORES];

    uint32_t regs[CCU_REGS / 4];
    uint32_t mbox[MAX_CORES];
    uint32_t level[2];                  /* device lines, shared */
    uint32_t mask[MAX_CORES][2];        /* per-core IMR */
    uint32_t ost;                       /* core OST lines, bit per core */
};

static uint32_t *ccu_reg(IngenicCcuState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static bool core_intc_pending(IngenicCcuState *s, int c)
{
    return ((s->level[0] & ~s->mask[c][0]) |
            (s->level[1] & ~s->mask[c][1])) != 0;
}

static void ccu_update(IngenicCcuState *s)
{
    uint32_t pimr = *ccu_reg(s, R_PIMR), mimr = *ccu_reg(s, R_MIMR);
    uint32_t oimr = *ccu_reg(s, R_OIMR);
    uint32_t pipr = 0, mipr = 0;
    int c;

    for (c = 0; c < s->ncores; c++) {
        if (core_intc_pending(s, c)) {
            pipr |= 1u << c;
        }
        if (s->mbox[c]) {
            mipr |= 1u << c;
        }
        qemu_set_irq(s->ip2[c], (pipr & pimr) >> c & 1);
        qemu_set_irq(s->ip3[c], (mipr & mimr) >> c & 1);
        qemu_set_irq(s->ip4[c], (s->ost & oimr) >> c & 1);
    }
    *ccu_reg(s, R_PIPR) = pipr;
    *ccu_reg(s, R_MIPR) = mipr;
    *ccu_reg(s, R_OIPR) = s->ost;
}

static void ccu_set_irq(void *opaque, int n, int level)
{
    IngenicCcuState *s = opaque;

    s->level[n / 32] = deposit32(s->level[n / 32], n % 32, 1, !!level);
    ccu_update(s);
}

static void ccu_set_ost(void *opaque, int n, int level)
{
    IngenicCcuState *s = opaque;

    s->ost = deposit32(s->ost, n, 1, !!level);
    ccu_update(s);
}

/* --- core start --------------------------------------------------------- */

static void ccu_start_core_work(CPUState *cs, run_on_cpu_data data)
{
    cpu_reset(cs);
    cpu_set_pc(cs, (int32_t)data.host_int);
    cs->halted = 0;
}

/* Back into reset: the core stops where it is (it starts powered off). */
static void ccu_hold_core_work(CPUState *cs, run_on_cpu_data data)
{
    cpu_reset(cs);
    cs->halted = 1;
}

/*
 * A CSRR bit going 1 -> 0 starts that core at RER; going 0 -> 1 holds it.
 * The T41 SPL starts core 1 from the on-chip RAM before the kernel, which
 * then puts it back in reset before starting it on its own entry.
 */
static void ccu_csrr_changed(IngenicCcuState *s, uint32_t old, uint32_t new)
{
    int c;

    for (c = 1; c < s->ncores; c++) {
        CPUState *cs = qemu_get_cpu(c);

        if (!cs) {
            continue;
        }
        if ((old & ~new) >> c & 1) {
            async_safe_run_on_cpu(cs, ccu_start_core_work,
                                  RUN_ON_CPU_HOST_INT(*ccu_reg(s, R_RER)));
        } else if ((new & ~old) >> c & 1) {
            async_safe_run_on_cpu(cs, ccu_hold_core_work, RUN_ON_CPU_NULL);
        }
    }
}

/* --- CCU registers ------------------------------------------------------ */

static uint64_t ccu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicCcuState *s = opaque;
    uint32_t v = 0;
    int c;

    if (off >= R_MBOX) {
        c = (off - R_MBOX) / 4;
        return c < s->ncores ? s->mbox[c] : 0;
    }
    switch (off) {
    case R_CSSR:
        for (c = 0; c < s->ncores; c++) {
            CPUState *cs = qemu_get_cpu(c);

            if (cs && cs->halted) {
                v |= 1u << c;
            }
        }
        return v;
    default:
        return *ccu_reg(s, off);
    }
}

static void ccu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    IngenicCcuState *s = opaque;
    uint32_t old;
    int c;

    if (off >= R_MBOX) {
        c = (off - R_MBOX) / 4;
        if (c < s->ncores) {
            s->mbox[c] = val;
        }
    } else {
        switch (off) {
        case R_CSSR:
        case R_PIPR:
        case R_MIPR:
        case R_OIPR:
            break;
        case R_CSRR:
            old = *ccu_reg(s, R_CSRR);
            *ccu_reg(s, R_CSRR) = val;
            ccu_csrr_changed(s, old, val);
            break;
        default:
            *ccu_reg(s, off) = val;
            break;
        }
    }
    ccu_update(s);
}

/* --- core INTC ---------------------------------------------------------- */

static uint64_t core_intc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicCcuState *s = opaque;
    int c = off / 0x100, b = (off & 0xff) / 0x20;

    if (c >= s->ncores || b > 1) {
        return 0;
    }
    switch (off & 0x1f) {
    case 0x00:
        return s->level[b];
    case 0x04:
        return s->mask[c][b];
    case 0x10:
        return s->level[b] & ~s->mask[c][b];
    default:
        return 0;
    }
}

static void core_intc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    IngenicCcuState *s = opaque;
    int c = off / 0x100, b = (off & 0xff) / 0x20;

    if (c >= s->ncores || b > 1) {
        return;
    }
    switch (off & 0x1f) {
    case 0x04:
        s->mask[c][b] = val;
        break;
    case 0x08:
        s->mask[c][b] |= val;
        break;
    case 0x0C:
        s->mask[c][b] &= ~val;
        break;
    default:
        return;
    }
    ccu_update(s);
}

static const MemoryRegionOps ccu_ops = {
    .read = ccu_read,
    .write = ccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps core_intc_ops = {
    .read = core_intc_read,
    .write = core_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ingenic_ccu_reset(DeviceState *dev)
{
    IngenicCcuState *s = INGENIC_CCU(dev);
    int c;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->mbox, 0, sizeof(s->mbox));
    for (c = 0; c < MAX_CORES; c++) {
        s->mask[c][0] = s->mask[c][1] = 0xffffffff;
    }
    *ccu_reg(s, R_CSRR) = ~1u & ((1u << MAX_CORES) - 1);
    *ccu_reg(s, R_PIMR) = 1;
    *ccu_reg(s, R_OIMR) = 1;
    ccu_update(s);
}

static void ingenic_ccu_realize(DeviceState *dev, Error **errp)
{
    IngenicCcuState *s = INGENIC_CCU(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int c;

    if (s->ncores < 1 || s->ncores > MAX_CORES) {
        error_setg(errp, "ingenic-ccu: 1 or 2 cores");
        return;
    }
    for (c = 0; c < s->ncores; c++) {
        sysbus_init_irq(sbd, &s->ip2[c]);
        sysbus_init_irq(sbd, &s->ip3[c]);
        sysbus_init_irq(sbd, &s->ip4[c]);
    }
    qdev_init_gpio_in(dev, ccu_set_irq, INTC_NR);
    qdev_init_gpio_in_named(dev, ccu_set_ost, "ost", s->ncores);
}

static void ingenic_ccu_init(Object *obj)
{
    IngenicCcuState *s = INGENIC_CCU(obj);

    memory_region_init_io(&s->ccu_mr, obj, &ccu_ops, s, "ingenic-ccu",
                          0x2000);
    memory_region_init_io(&s->intc_mr, obj, &core_intc_ops, s,
                          "ingenic-core-intc", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->ccu_mr);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->intc_mr);
}

static const Property ingenic_ccu_props[] = {
    DEFINE_PROP_UINT32("num-cores", IngenicCcuState, ncores, 2),
};

static const VMStateDescription vmstate_ingenic_ccu = {
    .name = TYPE_INGENIC_CCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicCcuState, CCU_REGS / 4),
        VMSTATE_UINT32_ARRAY(mbox, IngenicCcuState, MAX_CORES),
        VMSTATE_UINT32_ARRAY(level, IngenicCcuState, 2),
        VMSTATE_UINT32_2DARRAY(mask, IngenicCcuState, MAX_CORES, 2),
        VMSTATE_UINT32(ost, IngenicCcuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ccu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_ccu_realize;
    device_class_set_legacy_reset(dc, ingenic_ccu_reset);
    device_class_set_props(dc, ingenic_ccu_props);
    dc->vmsd = &vmstate_ingenic_ccu;
}

static const TypeInfo ingenic_ccu_info = {
    .name = TYPE_INGENIC_CCU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicCcuState),
    .instance_init = ingenic_ccu_init,
    .class_init = ingenic_ccu_class_init,
};

static void ingenic_ccu_register_types(void)
{
    type_register_static(&ingenic_ccu_info);
}

type_init(ingenic_ccu_register_types)
