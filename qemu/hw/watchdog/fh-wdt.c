/*
 * Fullhan watchdog: a DesignWare APB watchdog (drivers/watchdog/fh_wdt.c).
 *
 *   0x00 WDT_CR    bit 0 enable (set once, cleared only by reset), bit 1
 *                  response mode: 1 = interrupt first, reset on the
 *                  second expiry
 *   0x04 WDT_TORR  timeout 2^(16 + TOP) clocks, TOP in 3:0
 *   0x08 WDT_CCVR  the count       0x0C WDT_CRR  0x76 restarts it
 *   0x10 WDT_STAT  interrupt status                 0x14 WDT_EOI  read clears
 *   0xF4-0xFC      component parameters / version / type
 *
 * The counter stops while the "pause" input is high: the PMU's WDT_CTRL
 * bit 8, which the driver sets around the times nobody pings it.  It runs
 * at "freq"; the real clock is the AHB clock / PMU CLK_DIV5[15:8] + 1,
 * which the vendor U-Boot leaves around 1 MHz.  Running no faster than
 * the clock the driver believes keeps its timeouts at least as long.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/watchdog.h"

#define TYPE_FH_WDT "fh-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(FhWdtState, FH_WDT)

#define FH_WDT_SIZE         0x1000

#define R_CR                0x00
#define R_TORR              0x04
#define R_CCVR              0x08
#define R_CRR               0x0C
#define R_STAT              0x10
#define R_EOI               0x14

#define CR_EN               (1u << 0)
#define CR_RMOD             (1u << 1)
#define CRR_KICK            0x76

struct FhWdtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state *pt;

    uint32_t cr;
    uint32_t torr;
    bool stat;
    bool paused;
    uint32_t freq;
};

static uint32_t fh_wdt_period(FhWdtState *s)
{
    return (1u << (16 + (s->torr & 0xF))) - 1;
}

static void fh_wdt_update(FhWdtState *s)
{
    qemu_set_irq(s->irq, s->stat);
}

/* Called inside a ptimer transaction. */
static void fh_wdt_restart(FhWdtState *s)
{
    ptimer_set_limit(s->pt, fh_wdt_period(s), 1);
    if ((s->cr & CR_EN) && !s->paused) {
        ptimer_run(s->pt, 0);
    } else {
        ptimer_stop(s->pt);
    }
}

static void fh_wdt_expire(void *opaque)
{
    FhWdtState *s = opaque;

    if ((s->cr & CR_RMOD) && !s->stat) {
        s->stat = true;
        fh_wdt_update(s);
        return;
    }
    qemu_log_mask(CPU_LOG_RESET, "fh-wdt: timed out, resetting\n");
    watchdog_perform_action();
}

static uint64_t fh_wdt_read(void *opaque, hwaddr off, unsigned size)
{
    FhWdtState *s = opaque;

    switch (off) {
    case R_CR:
        return s->cr;
    case R_TORR:
        return s->torr;
    case R_CCVR:
        return ptimer_get_count(s->pt);
    case R_STAT:
        return s->stat;
    case R_EOI:
        s->stat = false;
        fh_wdt_update(s);
        return 0;
    case 0xF4:                  /* COMP_PARAMS_1: 32-bit counter, TOP 0..15 */
        return 0x10 << 8 | 0x2;
    case 0xF8:
        return 0x3130332A;      /* "103*" */
    case 0xFC:
        return 0x44570120;
    default:
        return 0;
    }
}

static void fh_wdt_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhWdtState *s = opaque;

    ptimer_transaction_begin(s->pt);
    switch (off) {
    case R_CR:
        /* The enable bit sticks once set. */
        s->cr = (val & 0x1F) | (s->cr & CR_EN);
        fh_wdt_restart(s);
        break;
    case R_TORR:
        s->torr = val & 0xFF;
        break;
    case R_CRR:
        if ((val & 0xFF) == CRR_KICK) {
            s->stat = false;
            fh_wdt_restart(s);
        }
        break;
    }
    ptimer_transaction_commit(s->pt);
    fh_wdt_update(s);
}

static const MemoryRegionOps fh_wdt_ops = {
    .read = fh_wdt_read,
    .write = fh_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_wdt_pause(void *opaque, int n, int level)
{
    FhWdtState *s = opaque;

    s->paused = level;
    ptimer_transaction_begin(s->pt);
    if (s->paused) {
        ptimer_stop(s->pt);
    } else if (s->cr & CR_EN) {
        ptimer_run(s->pt, 0);
    }
    ptimer_transaction_commit(s->pt);
}

static void fh_wdt_reset(DeviceState *dev)
{
    FhWdtState *s = FH_WDT(dev);

    s->cr = 0;
    s->torr = 0;
    s->stat = false;
    ptimer_transaction_begin(s->pt);
    fh_wdt_restart(s);
    ptimer_transaction_commit(s->pt);
    fh_wdt_update(s);
}

static void fh_wdt_realize(DeviceState *dev, Error **errp)
{
    FhWdtState *s = FH_WDT(dev);

    s->pt = ptimer_init(fh_wdt_expire, s,
                        PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                        PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                        PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
    ptimer_transaction_begin(s->pt);
    ptimer_set_freq(s->pt, s->freq);
    ptimer_transaction_commit(s->pt);
}

static void fh_wdt_init(Object *obj)
{
    FhWdtState *s = FH_WDT(obj);

    memory_region_init_io(&s->iomem, obj, &fh_wdt_ops, s, TYPE_FH_WDT,
                          FH_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), fh_wdt_pause, "pause", 1);
}

static const Property fh_wdt_props[] = {
    DEFINE_PROP_UINT32("freq", FhWdtState, freq, 1000000),
};

static const VMStateDescription vmstate_fh_wdt = {
    .name = TYPE_FH_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(pt, FhWdtState),
        VMSTATE_UINT32(cr, FhWdtState),
        VMSTATE_UINT32(torr, FhWdtState),
        VMSTATE_BOOL(stat, FhWdtState),
        VMSTATE_BOOL(paused, FhWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_wdt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_wdt_realize;
    device_class_set_legacy_reset(dc, fh_wdt_reset);
    device_class_set_props(dc, fh_wdt_props);
    dc->vmsd = &vmstate_fh_wdt;
}

static const TypeInfo fh_wdt_info = {
    .name           = TYPE_FH_WDT,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhWdtState),
    .instance_init  = fh_wdt_init,
    .class_init     = fh_wdt_class_init,
};

static void fh_wdt_register_types(void)
{
    type_register_static(&fh_wdt_info);
}

type_init(fh_wdt_register_types)
