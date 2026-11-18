/*
 * Grain Media FTTMR010 timers.
 *
 * Faraday's three-channel timer at 0x90D00000 on the GM813x parts
 * (arch/arm/mach-GM/fttmr010.c, include/mach/fttmr010.h; U-Boot
 * board/gm/gm8136/timer.c).  Channel n's registers are at n * 0x10:
 *
 *   +0x00 COUNTER  +0x04 LOAD  +0x08 MATCH1  +0x0C MATCH2
 *
 * then the shared CR 0x30 (per channel n, bits 3n: enable, 3n+1: clock
 * source, 3n+2: overflow interrupt enable; bits 9 + n: count up),
 * INTR_STATE 0x34 (per channel n, bits 3n: match 1, 3n+1: match 2, 3n+2:
 * overflow; write 1 to clear) and INTR_MASK 0x38 (1 masks).
 *
 * A channel counts COUNTER down at PCLK -- the "clk" input, the PMU's
 * APB clock, which is what both the kernel (pmu_get_apb_clk()) and U-Boot
 * (u32PMU_ReadPCLK()) take the rate from -- or at "ext-freq" with the
 * clock-source bit set.  Counting past zero reloads LOAD and raises the
 * overflow status.  The kernel's clockevent runs a channel periodic with
 * LOAD = rate / HZ - 1, or one-shot by writing COUNTER with LOAD = 0; a
 * channel that reloads 0 therefore stops until COUNTER is written again.
 * Its clocksource is a free-running channel read as ~COUNTER, U-Boot's a
 * down-counter reloading PCLK.
 *
 * The match registers are only modelled where both drivers leave them,
 * at 0: they raise their status bits when the count reaches zero.  The
 * three channels share one interrupt line on the GM813x (14).
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
#include "hw/clock.h"
#include "hw/ptimer.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_TIMER "gm-timer"
OBJECT_DECLARE_SIMPLE_TYPE(GmTimerState, GM_TIMER)

#define GM_TIMER_NUM        3
#define GM_TIMER_SIZE       0x1000

#define R_CR                0x30
#define R_INTR_STATE        0x34
#define R_INTR_MASK         0x38
#define R_REVISION          0x3C

#define CR_EN(n)            (1u << (3 * (n)))
#define CR_EXTCLK(n)        (1u << (3 * (n) + 1))
#define CR_OFEN(n)          (1u << (3 * (n) + 2))
#define CR_UP(n)            (1u << (9 + (n)))

#define ST_MATCH1(n)        (1u << (3 * (n)))
#define ST_MATCH2(n)        (1u << (3 * (n) + 1))
#define ST_OVERFLOW(n)      (1u << (3 * (n) + 2))

typedef struct GmTimerChan {
    struct GmTimerState *s;
    ptimer_state *pt;
    int id;
    uint32_t load;
    uint32_t match[2];
    uint32_t stopped;           /* count a halted channel holds */
} GmTimerChan;

struct GmTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    GmTimerChan chan[GM_TIMER_NUM];
    uint32_t cr;
    uint32_t state;
    uint32_t mask;
    uint32_t freq;
    uint32_t ext_freq;
};

static void gm_timer_update(GmTimerState *s)
{
    qemu_set_irq(s->irq, (s->state & ~s->mask) != 0);
}

/* Called inside a ptimer transaction. */
static void gm_timer_set_rate(GmTimerChan *c)
{
    GmTimerState *s = c->s;

    if (s->cr & CR_EXTCLK(c->id)) {
        ptimer_set_freq(c->pt, s->ext_freq);
    } else if (clock_has_source(s->clk)) {
        ptimer_set_period_from_clock(c->pt, s->clk, 1);
    } else {
        ptimer_set_freq(c->pt, s->freq);
    }
}

static void gm_timer_clk_update(void *opaque, ClockEvent event)
{
    GmTimerState *s = opaque;
    int i;

    for (i = 0; i < GM_TIMER_NUM; i++) {
        ptimer_transaction_begin(s->chan[i].pt);
        gm_timer_set_rate(&s->chan[i]);
        ptimer_transaction_commit(s->chan[i].pt);
    }
}

/*
 * The down-counting ptimer behind a channel holds the distance to the
 * next overflow: COUNTER itself counting down, ~COUNTER counting up.
 */
static uint32_t gm_timer_to_reg(GmTimerChan *c, uint32_t v)
{
    return (c->s->cr & CR_UP(c->id)) ? ~v : v;
}

/* Called inside a ptimer transaction: (re)start from count. */
static void gm_timer_start(GmTimerChan *c, uint32_t count)
{
    uint32_t reload = gm_timer_to_reg(c, c->load);

    ptimer_stop(c->pt);
    if (reload) {
        ptimer_set_limit(c->pt, reload, 0);
        ptimer_set_count(c->pt, count);
        ptimer_run(c->pt, 0);
    } else {
        /* LOAD 0: one-shot, then hold at 0 */
        ptimer_set_limit(c->pt, 0xFFFFFFFF, 0);
        ptimer_set_count(c->pt, count);
        ptimer_run(c->pt, 1);
    }
}

static uint32_t gm_timer_get_count(GmTimerChan *c)
{
    GmTimerState *s = c->s;

    if (!(s->cr & CR_EN(c->id))) {
        return gm_timer_to_reg(c, c->stopped);
    }
    return gm_timer_to_reg(c, ptimer_get_count(c->pt));
}

static void gm_timer_tick(void *opaque)
{
    GmTimerChan *c = opaque;
    GmTimerState *s = c->s;

    if (s->cr & CR_OFEN(c->id)) {
        s->state |= ST_OVERFLOW(c->id);
    }
    if (!c->match[0]) {
        s->state |= ST_MATCH1(c->id);
    }
    if (!c->match[1]) {
        s->state |= ST_MATCH2(c->id);
    }
    gm_timer_update(s);
}

static uint64_t gm_timer_read(void *opaque, hwaddr off, unsigned size)
{
    GmTimerState *s = opaque;
    GmTimerChan *c;

    switch (off) {
    case R_CR:
        return s->cr;
    case R_INTR_STATE:
        return s->state;
    case R_INTR_MASK:
        return s->mask;
    case R_REVISION:
        return 0x00010105;
    }
    if (off >= R_CR) {
        return 0;
    }
    c = &s->chan[off / 0x10];
    switch (off & 0xF) {
    case 0x0:
        return gm_timer_get_count(c);
    case 0x4:
        return c->load;
    case 0x8:
        return c->match[0];
    case 0xC:
        return c->match[1];
    }
    return 0;
}

static void gm_timer_write_cr(GmTimerState *s, uint32_t val)
{
    uint32_t old = s->cr;
    int i;

    for (i = 0; i < GM_TIMER_NUM; i++) {
        GmTimerChan *c = &s->chan[i];
        uint32_t cur;

        /* COUNTER as the old CR shows it, kept across the change */
        s->cr = old;
        cur = gm_timer_get_count(c);
        s->cr = val;
        ptimer_transaction_begin(c->pt);
        gm_timer_set_rate(c);
        if (!(val & CR_EN(i))) {
            ptimer_stop(c->pt);
            c->stopped = gm_timer_to_reg(c, cur);
        } else if (!(old & CR_EN(i)) ||
                   ((old ^ val) & CR_UP(i))) {
            gm_timer_start(c, gm_timer_to_reg(c, cur));
        }
        ptimer_transaction_commit(c->pt);
    }
    s->cr = val;
}

static void gm_timer_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    GmTimerState *s = opaque;
    GmTimerChan *c;

    switch (off) {
    case R_CR:
        gm_timer_write_cr(s, val);
        return;
    case R_INTR_STATE:
        s->state &= ~val;
        gm_timer_update(s);
        return;
    case R_INTR_MASK:
        s->mask = val & 0x1FF;
        gm_timer_update(s);
        return;
    }
    if (off >= R_CR) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, off);
        return;
    }
    c = &s->chan[off / 0x10];
    switch (off & 0xF) {
    case 0x0:
        if (s->cr & CR_EN(c->id)) {
            ptimer_transaction_begin(c->pt);
            gm_timer_start(c, gm_timer_to_reg(c, val));
            ptimer_transaction_commit(c->pt);
        } else {
            c->stopped = gm_timer_to_reg(c, val);
        }
        break;
    case 0x4:
        /* a running channel takes the new value at its next reload */
        c->load = val;
        if (s->cr & CR_EN(c->id)) {
            ptimer_transaction_begin(c->pt);
            if (gm_timer_to_reg(c, val)) {
                ptimer_set_limit(c->pt, gm_timer_to_reg(c, val), 0);
                ptimer_run(c->pt, 0);
            } else {                        /* LOAD 0: one-shot */
                ptimer_set_limit(c->pt, 0xFFFFFFFF, 0);
                ptimer_run(c->pt, 1);
            }
            ptimer_transaction_commit(c->pt);
        }
        break;
    case 0x8:
        c->match[0] = val;
        break;
    case 0xC:
        c->match[1] = val;
        break;
    }
}

static const MemoryRegionOps gm_timer_ops = {
    .read = gm_timer_read,
    .write = gm_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_timer_reset(DeviceState *dev)
{
    GmTimerState *s = GM_TIMER(dev);
    int i;

    s->cr = 0;
    s->state = 0;
    s->mask = 0;
    for (i = 0; i < GM_TIMER_NUM; i++) {
        GmTimerChan *c = &s->chan[i];

        ptimer_transaction_begin(c->pt);
        ptimer_stop(c->pt);
        gm_timer_set_rate(c);
        ptimer_transaction_commit(c->pt);
        c->load = 0;
        c->match[0] = c->match[1] = 0;
        c->stopped = 0;
    }
    gm_timer_update(s);
}

static void gm_timer_realize(DeviceState *dev, Error **errp)
{
    GmTimerState *s = GM_TIMER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_timer_ops, s,
                          TYPE_GM_TIMER, GM_TIMER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    for (i = 0; i < GM_TIMER_NUM; i++) {
        GmTimerChan *c = &s->chan[i];

        c->s = s;
        c->id = i;
        c->pt = ptimer_init(gm_timer_tick, c,
                            PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                            PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                            PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                            PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(c->pt);
        gm_timer_set_rate(c);
        ptimer_transaction_commit(c->pt);
    }
}

static void gm_timer_init(Object *obj)
{
    GmTimerState *s = GM_TIMER(obj);

    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", gm_timer_clk_update, s,
                                ClockUpdate);
}

static const Property gm_timer_props[] = {
    DEFINE_PROP_UINT32("freq", GmTimerState, freq, 100000000),
    DEFINE_PROP_UINT32("ext-freq", GmTimerState, ext_freq, 30000000),
};

static const VMStateDescription vmstate_gm_timer_chan = {
    .name = "gm-timer-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(pt, GmTimerChan),
        VMSTATE_UINT32(load, GmTimerChan),
        VMSTATE_UINT32_ARRAY(match, GmTimerChan, 2),
        VMSTATE_UINT32(stopped, GmTimerChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gm_timer = {
    .name = TYPE_GM_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, GmTimerState),
        VMSTATE_STRUCT_ARRAY(chan, GmTimerState, GM_TIMER_NUM, 1,
                             vmstate_gm_timer_chan, GmTimerChan),
        VMSTATE_UINT32(cr, GmTimerState),
        VMSTATE_UINT32(state, GmTimerState),
        VMSTATE_UINT32(mask, GmTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_timer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_timer_realize;
    device_class_set_legacy_reset(dc, gm_timer_reset);
    device_class_set_props(dc, gm_timer_props);
    dc->vmsd = &vmstate_gm_timer;
}

static const TypeInfo gm_timer_info = {
    .name           = TYPE_GM_TIMER,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmTimerState),
    .instance_init  = gm_timer_init,
    .class_init     = gm_timer_class_init,
};

static void gm_timer_register_types(void)
{
    type_register_static(&gm_timer_info);
}

type_init(gm_timer_register_types)
