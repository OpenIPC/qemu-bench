/*
 * Fullhan timers: a DesignWare APB timers block.
 *
 * Channels 0x14 apart (drivers/clocksource/timer-fh.c, U-Boot
 * arch/arm/cpu/fh/fh/timer.c):
 *
 *   +0x00 LOADCOUNT   +0x04 CURRENTVALUE   +0x08 CONTROL
 *   +0x0C EOI (read clears)                +0x10 INTSTATUS
 *
 * CONTROL: bit 0 enable, bit 1 mode (1 = user defined: reload LOADCOUNT,
 * 0 = free running: reload 0xFFFFFFFF), bit 2 interrupt mask.  Then the
 * block-wide TIMERSINTSTATUS 0xA0, TIMERSEOI 0xA4, TIMERSRAWINTSTATUS
 * 0xA8 and the component version 0xAC.
 *
 * Enabling a channel loads LOADCOUNT; a disabled one reads 0, which both
 * drivers wait for after clearing the enable bit before they write a new
 * count.  The count runs at the "clk" input -- the PMU's tmr0 clock,
 * which follows its divider: the vendor U-Boots leave 1 MHz, the V100 and
 * FH865x kernels set 50 MHz -- or at "freq" with nothing connected.  The
 * channels share one interrupt line: the V200 kernels take their tick from
 * channel 0, the older ones from channel 1.  A pulse on "reset" (the PMU's
 * timer reset, which the FH8632 kernel pulses after programming its tick)
 * restarts every running channel from LOADCOUNT.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/clock.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_TIMER "fh-timer"
OBJECT_DECLARE_SIMPLE_TYPE(FhTimerState, FH_TIMER)

#define FH_TIMER_MAX        8
#define FH_TIMER_SIZE       0x1000
#define FH_TIMER_STRIDE     0x14

#define CTRL_EN             (1u << 0)
#define CTRL_USER           (1u << 1)
#define CTRL_MASK           (1u << 2)

typedef struct FhTimerChan {
    struct FhTimerState *s;
    ptimer_state *pt;
    uint32_t load;
    uint32_t ctrl;
    bool raw;
} FhTimerChan;

struct FhTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    FhTimerChan chan[FH_TIMER_MAX];
    uint32_t num;
    uint32_t freq;
};

static void fh_timer_update(FhTimerChan *c)
{
    FhTimerState *s = c->s;
    bool level = false;
    int i;

    for (i = 0; i < s->num; i++) {
        if (s->chan[i].raw && !(s->chan[i].ctrl & CTRL_MASK)) {
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

/* Called inside a ptimer transaction. */
static void fh_timer_set_rate(FhTimerState *s, FhTimerChan *c)
{
    if (clock_has_source(s->clk)) {
        ptimer_set_period_from_clock(c->pt, s->clk, 1);
    } else {
        ptimer_set_freq(c->pt, s->freq);
    }
}

static void fh_timer_clk_update(void *opaque, ClockEvent event)
{
    FhTimerState *s = opaque;
    int i;

    for (i = 0; i < FH_TIMER_MAX; i++) {
        ptimer_transaction_begin(s->chan[i].pt);
        fh_timer_set_rate(s, &s->chan[i]);
        ptimer_transaction_commit(s->chan[i].pt);
    }
}

static void fh_timer_tick(void *opaque)
{
    FhTimerChan *c = opaque;

    c->raw = true;
    if (!(c->ctrl & CTRL_USER)) {
        /* Free running: the next period counts down from the top. */
        ptimer_set_limit(c->pt, 0xFFFFFFFF, 1);
    }
    fh_timer_update(c);
}

/* Called inside a ptimer transaction. */
static void fh_timer_set_ctrl(FhTimerChan *c, uint32_t val)
{
    bool was = c->ctrl & CTRL_EN;

    c->ctrl = val & 7;
    if (!(c->ctrl & CTRL_EN)) {
        ptimer_stop(c->pt);
        ptimer_set_count(c->pt, 0);
        return;
    }
    if (!was) {
        ptimer_set_limit(c->pt, c->load ? c->load : 1, 1);
        ptimer_run(c->pt, 0);
    }
}

static uint32_t fh_timer_int_status(FhTimerState *s, bool raw)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < s->num; i++) {
        FhTimerChan *c = &s->chan[i];

        if (c->raw && (raw || !(c->ctrl & CTRL_MASK))) {
            v |= 1u << i;
        }
    }
    return v;
}

static uint64_t fh_timer_read(void *opaque, hwaddr off, unsigned size)
{
    FhTimerState *s = opaque;
    FhTimerChan *c;
    uint32_t v;
    int i;

    if (off >= 0xA0) {
        switch (off) {
        case 0xA0:
            return fh_timer_int_status(s, false);
        case 0xA4:              /* TIMERSEOI */
            v = fh_timer_int_status(s, false);
            for (i = 0; i < s->num; i++) {
                s->chan[i].raw = false;
                fh_timer_update(&s->chan[i]);
            }
            return v;
        case 0xA8:
            return fh_timer_int_status(s, true);
        case 0xAC:
            return 0x3230392A;  /* "209*" */
        default:
            return 0;
        }
    }
    i = off / FH_TIMER_STRIDE;
    if (i >= s->num) {
        return 0;
    }
    c = &s->chan[i];
    switch (off % FH_TIMER_STRIDE) {
    case 0x00:
        return c->load;
    case 0x04:
        return (c->ctrl & CTRL_EN) ? ptimer_get_count(c->pt) : 0;
    case 0x08:
        return c->ctrl;
    case 0x0C:                  /* EOI */
        c->raw = false;
        fh_timer_update(c);
        return 0;
    case 0x10:
        return c->raw && !(c->ctrl & CTRL_MASK);
    }
    return 0;
}

static void fh_timer_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    FhTimerState *s = opaque;
    FhTimerChan *c;
    int i = off / FH_TIMER_STRIDE;

    if (off >= 0xA0 || i >= s->num) {
        return;
    }
    c = &s->chan[i];
    ptimer_transaction_begin(c->pt);
    switch (off % FH_TIMER_STRIDE) {
    case 0x00:
        /* A running channel takes the new count at its next reload. */
        c->load = val;
        if ((c->ctrl & (CTRL_EN | CTRL_USER)) == (CTRL_EN | CTRL_USER)) {
            ptimer_set_limit(c->pt, val ? val : 1, 0);
        }
        break;
    case 0x08:
        fh_timer_set_ctrl(c, val);
        break;
    }
    ptimer_transaction_commit(c->pt);
    fh_timer_update(c);
}

static const MemoryRegionOps fh_timer_ops = {
    .read = fh_timer_read,
    .write = fh_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_timer_reset(DeviceState *dev)
{
    FhTimerState *s = FH_TIMER(dev);
    int i;

    for (i = 0; i < s->num; i++) {
        FhTimerChan *c = &s->chan[i];

        ptimer_transaction_begin(c->pt);
        ptimer_stop(c->pt);
        ptimer_set_count(c->pt, 0);
        ptimer_transaction_commit(c->pt);
        c->load = 0;
        c->ctrl = 0;
        c->raw = false;
        fh_timer_update(c);
    }
}

static void fh_timer_realize(DeviceState *dev, Error **errp)
{
    FhTimerState *s = FH_TIMER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    if (!s->num || s->num > FH_TIMER_MAX) {
        error_setg(errp, "fh-timer: num-timers must be 1..%d", FH_TIMER_MAX);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &fh_timer_ops, s,
                          TYPE_FH_TIMER, FH_TIMER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    /* Every slot gets its ptimer, so the vmstate array is uniform. */
    for (i = 0; i < FH_TIMER_MAX; i++) {
        FhTimerChan *c = &s->chan[i];

        c->s = s;
        c->pt = ptimer_init(fh_timer_tick, c,
                            PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                            PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                            PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                            PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(c->pt);
        fh_timer_set_rate(s, c);
        ptimer_transaction_commit(c->pt);
    }
    sysbus_init_irq(sbd, &s->irq);
}

static void fh_timer_reload(void *opaque, int n, int level)
{
    FhTimerState *s = opaque;
    int i;

    if (!level) {
        return;
    }
    for (i = 0; i < s->num; i++) {
        FhTimerChan *c = &s->chan[i];

        if (c->ctrl & CTRL_EN) {
            ptimer_transaction_begin(c->pt);
            ptimer_set_limit(c->pt, c->load ? c->load : 1, 1);
            ptimer_transaction_commit(c->pt);
        }
    }
}

static void fh_timer_init(Object *obj)
{
    FhTimerState *s = FH_TIMER(obj);

    qdev_init_gpio_in_named(DEVICE(obj), fh_timer_reload, "reset", 1);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", fh_timer_clk_update, s,
                                ClockUpdate);
}

static const Property fh_timer_props[] = {
    DEFINE_PROP_UINT32("num-timers", FhTimerState, num, 2),
    DEFINE_PROP_UINT32("freq", FhTimerState, freq, 1000000),
};

static const VMStateDescription vmstate_fh_timer_chan = {
    .name = "fh-timer-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(pt, FhTimerChan),
        VMSTATE_UINT32(load, FhTimerChan),
        VMSTATE_UINT32(ctrl, FhTimerChan),
        VMSTATE_BOOL(raw, FhTimerChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_fh_timer = {
    .name = TYPE_FH_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, FhTimerState),
        VMSTATE_STRUCT_ARRAY(chan, FhTimerState, FH_TIMER_MAX, 1,
                             vmstate_fh_timer_chan, FhTimerChan),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_timer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_timer_realize;
    device_class_set_legacy_reset(dc, fh_timer_reset);
    device_class_set_props(dc, fh_timer_props);
    dc->vmsd = &vmstate_fh_timer;
}

static const TypeInfo fh_timer_info = {
    .name           = TYPE_FH_TIMER,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhTimerState),
    .instance_init  = fh_timer_init,
    .class_init     = fh_timer_class_init,
};

static void fh_timer_register_types(void)
{
    type_register_static(&fh_timer_info);
}

type_init(fh_timer_register_types)
