/*
 * SigmaStar / MStar PM timers and watchdog (RIU bank 0x0030, 0x1F006000).
 *
 * One bank holds the watchdog at registers 0x00-0x0F and three PIU
 * timers at 0x10, 0x20 and 0x30:
 *
 *   timer +0  CTRL  bit 0 EN, bit 1 TRIG (restart from 0, self-clearing),
 *                   bit 8 interrupt enable
 *         +1  HIT   bit 0 set when the counter wraps, write 1 to clear
 *         +2/+3 MAX low/high: the counter wraps after MAX and raises its
 *                   interrupt there
 *         +4/+5 counter low/high      +6 DIVIDE: clock / (N + 1)
 *
 * The counter only runs while EN is set.  Out of reset the timers count
 * the 12 MHz crystal; the IPL moves timer 0 onto the 432 MHz MPLL and
 * sets DIVIDE to 35, which is 12 MHz again, so "freq" is the crystal and
 * DIVIDE is honoured.  Switching the clock source does not rebase a
 * running counter: the IPL switches it once, before it times anything.  The semantics are Daniel Palmer's, measured on an
 * SSD202D (github.com/fifteenhex/qemu mstar-clean, docs/.../timer.rst and
 * ipl.rst); the layout is the vendor U-Boot's and kernel's
 * (arch/arm/cpu/armv7/infinity2m/timer.c, drivers/sstar/clocksource).
 *
 *   wdt +0  CLR (bit 0 kicks)   +4/+5 MAX_PRD low/high in 12 MHz ticks,
 *           0 = off (drivers/sstar/watchdog/mdrv_wdt.c)
 *
 * The watchdog resets the machine when it runs out.  Interrupt outputs:
 * timer 0, timer 1, watchdog, timer 2 (MS_FIQ 0, 1, 2 and 12).
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/runstate.h"
#include "hw/misc/sstar-riu.h"

#define TYPE_SSTAR_TIMER "sstar-timer"
OBJECT_DECLARE_SIMPLE_TYPE(SstarTimerState, SSTAR_TIMER)

#define NUM_TIMERS          3

#define T_CTRL              0
#define T_HIT               1
#define T_MAX_L             2
#define T_MAX_H             3
#define T_CNT_L             4
#define T_CNT_H             5
#define T_DIVIDE            6

#define CTRL_EN             (1u << 0)
#define CTRL_TRIG           (1u << 1)
#define CTRL_INT            (1u << 8)

#define WDT_CLR             0x00
#define WDT_MAX_L           0x04
#define WDT_MAX_H           0x05

typedef struct SstarPiuTimer {
    SstarTimerState *s;
    QEMUTimer *timer;
    uint16_t regs[0x10];
    int64_t start_ns;
    uint16_t latch;
    bool running;
    qemu_irq irq;
} SstarPiuTimer;

struct SstarTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SstarPiuTimer t[NUM_TIMERS];
    uint16_t wdt[0x10];
    QEMUTimer *wdt_timer;
    qemu_irq wdt_irq;
    SstarRiuState *riu;
    uint32_t freq;
    uint32_t mpll_freq;
};

static uint32_t piu_max(SstarPiuTimer *t)
{
    return t->regs[T_MAX_L] | ((uint32_t)t->regs[T_MAX_H] << 16);
}

/*
 * The input clock: the crystal, or for timer 0 the MPLL once clkgen
 * 0x1F207004 (bank 0x1038 reg 0x01) selects it with 0x30.
 */
static uint64_t piu_freq(SstarTimerState *s, SstarPiuTimer *t)
{
    if (t == &s->t[0] && s->riu &&
        (sstar_riu_get(s->riu, 0x1038, 0x01) & 0x30) == 0x30) {
        return s->mpll_freq;
    }
    return s->freq;
}

/* Ticks since the counter last started from 0. */
static uint64_t piu_ticks(SstarTimerState *s, SstarPiuTimer *t)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - t->start_ns;

    return muldiv64(ns, piu_freq(s, t), NANOSECONDS_PER_SECOND) /
           (t->regs[T_DIVIDE] + 1);
}

static uint32_t piu_count(SstarTimerState *s, SstarPiuTimer *t)
{
    uint64_t max = piu_max(t);

    if (!t->running) {
        return t->regs[T_CNT_L] | ((uint32_t)t->regs[T_CNT_H] << 16);
    }
    return piu_ticks(s, t) % (max + 1);
}

static int64_t piu_ns(SstarTimerState *s, SstarPiuTimer *t, uint64_t ticks)
{
    return muldiv64(ticks * (t->regs[T_DIVIDE] + 1), NANOSECONDS_PER_SECOND,
                    piu_freq(s, t));
}

/* Arm the QEMU timer for the next time the counter reaches MAX. */
static void piu_schedule(SstarTimerState *s, SstarPiuTimer *t)
{
    uint64_t period = (uint64_t)piu_max(t) + 1;
    uint64_t ticks;

    if (!t->running || !(t->regs[T_CTRL] & CTRL_INT)) {
        timer_del(t->timer);
        return;
    }
    /*
     * The counter reaches MAX at ticks k * period - 1; the next such k is
     * after the one just reached, so an expiry does not re-arm itself.
     */
    ticks = piu_ticks(s, t);
    timer_mod(t->timer, t->start_ns +
              piu_ns(s, t, ((ticks + 1) / period + 1) * period - 1) + 1);
}

static void piu_timer_cb(void *opaque)
{
    SstarPiuTimer *t = opaque;
    SstarTimerState *s = t->s;

    t->regs[T_HIT] |= 1;
    qemu_irq_pulse(t->irq);
    piu_schedule(s, t);
}

/* Restart from 0, keeping the phase of a counter that is not running. */
static void piu_start(SstarTimerState *s, SstarPiuTimer *t, uint32_t from)
{
    t->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - piu_ns(s, t, from);
    t->running = true;
    piu_schedule(s, t);
}

static void piu_stop(SstarTimerState *s, SstarPiuTimer *t)
{
    uint32_t cnt = piu_count(s, t);

    t->running = false;
    t->regs[T_CNT_L] = cnt;
    t->regs[T_CNT_H] = cnt >> 16;
    timer_del(t->timer);
}

static void wdt_kick(SstarTimerState *s)
{
    uint32_t max = s->wdt[WDT_MAX_L] | ((uint32_t)s->wdt[WDT_MAX_H] << 16);

    if (!max) {
        timer_del(s->wdt_timer);
        return;
    }
    timer_mod(s->wdt_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64(max, NANOSECONDS_PER_SECOND, s->freq));
}

static void wdt_expire(void *opaque)
{
    SstarTimerState *s = opaque;

    qemu_log_mask(CPU_LOG_RESET, "sstar-timer: watchdog reset\n");
    qemu_irq_pulse(s->wdt_irq);
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static uint16_t sstar_timer_reg_read(SstarTimerState *s, int reg)
{
    SstarPiuTimer *t;
    uint32_t cnt;

    if (reg < 0x10) {
        return s->wdt[reg];
    }
    t = &s->t[(reg >> 4) - 1];
    switch (reg & 0xF) {
    case T_CNT_L:
        /*
         * Latch the high half, so that a low-then-high read is one value:
         * U-Boot's get_timer() takes any step back for a 32-bit wrap.
         */
        cnt = piu_count(s, t);
        t->latch = cnt >> 16;
        return cnt;
    case T_CNT_H:
        return t->latch;
    default:
        return t->regs[reg & 0xF];
    }
}

static void sstar_timer_reg_write(SstarTimerState *s, int reg, uint16_t v)
{
    SstarPiuTimer *t;

    if (reg < 0x10) {
        s->wdt[reg] = v;
        if ((reg == WDT_CLR && (v & 1)) ||
            reg == WDT_MAX_L || reg == WDT_MAX_H) {
            wdt_kick(s);
        }
        return;
    }
    t = &s->t[(reg >> 4) - 1];
    switch (reg & 0xF) {
    case T_CTRL: {
        uint32_t cnt = piu_count(s, t);

        if ((v & CTRL_TRIG) ||
            ((v & CTRL_EN) && !(t->regs[T_CTRL] & CTRL_EN))) {
            cnt = 0;
        }
        t->regs[T_CTRL] = v & ~CTRL_TRIG;
        if (v & CTRL_EN) {
            piu_start(s, t, cnt);
        } else {
            if (t->running) {
                piu_stop(s, t);
            }
            t->regs[T_CNT_L] = cnt;
            t->regs[T_CNT_H] = cnt >> 16;
        }
        break;
    }
    case T_HIT:
        t->regs[T_HIT] &= ~v;
        break;
    case T_CNT_L:
    case T_CNT_H:
        break;
    default: {
        uint32_t cnt = piu_count(s, t);

        t->regs[reg & 0xF] = v;
        /* MAX and DIVIDE change the rate or period, not the count. */
        if (t->running) {
            piu_start(s, t, cnt % ((uint64_t)piu_max(t) + 1));
        }
        break;
    }
    }
}

static uint64_t sstar_timer_read(void *opaque, hwaddr off, unsigned size)
{
    SstarTimerState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = sstar_timer_reg_read(s, (off >> 2) & 0x3F);
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_timer_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    SstarTimerState *s = opaque;
    int reg = (off >> 2) & 0x3F;
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        uint16_t cur = sstar_timer_reg_read(s, reg);

        v = (off & 1) ? (cur & 0x00FF) | ((val & 0xFF) << 8)
                      : (cur & 0xFF00) | (val & 0xFF);
    }
    sstar_timer_reg_write(s, reg, v);
}

static const MemoryRegionOps sstar_timer_ops = {
    .read = sstar_timer_read,
    .write = sstar_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_timer_reset(DeviceState *dev)
{
    SstarTimerState *s = SSTAR_TIMER(dev);
    int i;

    for (i = 0; i < NUM_TIMERS; i++) {
        memset(s->t[i].regs, 0, sizeof(s->t[i].regs));
        s->t[i].running = false;
        timer_del(s->t[i].timer);
    }
    memset(s->wdt, 0, sizeof(s->wdt));
    timer_del(s->wdt_timer);
}

static void sstar_timer_init(Object *obj)
{
    SstarTimerState *s = SSTAR_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &sstar_timer_ops, s, "sstar-timer",
                          0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    for (i = 0; i < NUM_TIMERS; i++) {
        s->t[i].s = s;
        s->t[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, piu_timer_cb,
                                     &s->t[i]);
    }
    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_expire, s);
    /* Outputs in MS_FIQ order: timer 0, timer 1, watchdog, timer 2. */
    sysbus_init_irq(sbd, &s->t[0].irq);
    sysbus_init_irq(sbd, &s->t[1].irq);
    sysbus_init_irq(sbd, &s->wdt_irq);
    sysbus_init_irq(sbd, &s->t[2].irq);
}

static const Property sstar_timer_props[] = {
    DEFINE_PROP_UINT32("freq", SstarTimerState, freq, 12000000),
    DEFINE_PROP_UINT32("mpll-freq", SstarTimerState, mpll_freq, 432000000),
    /* Where the clkgen timer source select lives. */
    DEFINE_PROP_LINK("riu", SstarTimerState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
};

static void sstar_timer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_timer_reset);
    device_class_set_props(dc, sstar_timer_props);
}

static const TypeInfo sstar_timer_info = {
    .name           = TYPE_SSTAR_TIMER,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarTimerState),
    .instance_init  = sstar_timer_init,
    .class_init     = sstar_timer_class_init,
};

static void sstar_timer_register_types(void)
{
    type_register_static(&sstar_timer_info);
}

type_init(sstar_timer_register_types)
