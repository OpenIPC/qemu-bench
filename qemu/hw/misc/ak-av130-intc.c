/*
 * Anyka AK3918AV130 interrupt controller and system timers, at 0x21100000
 * (the kernel's AK_VA_INT_TIMER).
 *
 * Source: the AnyCloud39AV130 SDK, kernel drivers/irqchip/ak_irq_v2.c and
 * drivers/clocksource/ak_timer*.[ch] (CONFIG_MACH_AK3918AV130), checked
 * against the first AV130 flash (owner report r-5uwdebux), whose vendor
 * U-Boot keeps time with timer 0 and whose kernel boots on all of it.  It
 * takes over from ak-tick.c, which modelled timer 0 as U-Boot alone uses it.
 *
 * Interrupts: a top bank and cascaded sub-banks, each a mask (1 = enabled)
 * and a status register.  A device tree specifier is <bit bank>.
 *   +0x00 FIQ mask   +0x04 IRQ mask   +0x08 top status        (bank 0)
 *   +0x0C / +0x10 TWI (I2C) mask / status, top bit 13         (bank 6)
 *   +0x14 / +0x18 SSS, top bit 17                             (bank 2)
 *   +0x1C / +0x20 NSS, top bit 0                              (bank 5)
 *   +0x24 / +0x28 ISS, top bit 1                              (bank 4)
 *   +0x2C / +0x30 ESS, top bit 2                              (bank 3)
 * Bank 1 is the system controller's: its mask and status stay there
 * (0x0800002C / 0x30) and it drives input 3, the top bit for the bank.  Top status is the raw inputs plus a cascade bit for each
 * sub-bank with an enabled interrupt pending.
 *
 * Timers: five channels at +0x10000 + n * 0x1000, 12 MHz, counting down,
 * interrupts on top bits 27..31.  Timer 0 is the kernel's clocksource,
 * timer 1 its clock event; U-Boot uses timer 0 for get_timer().
 *   +0x00 LOAD   +0x04 VALUE   +0x08 CTRL: 0 enable, 2:1 mode (free,
 *   periodic, one-shot), 3 reload VALUE from LOAD now (self-clearing)
 *   +0x0C INTCLR (1 clears)   +0x10 INT_EN   +0x14 DIV   +0x18 INT_STA
 * VALUE counts from where CTRL (or a LOAD write) left it down to 0, then
 * free mode goes on from 0xFFFFFFFF and periodic from LOAD -- a period of
 * LOAD + 1 ticks, as the kernel's TIMER_CNT - 1 has it -- and one-shot
 * stops.  INT_STA is set as VALUE reaches 0.  Both are worked out from the
 * virtual clock and one base (12 MHz is not a whole number of nanoseconds
 * a tick, which a ptimer's count would round): VALUE when it is read, and
 * the next zero for the timer that raises INT_STA.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_AV130_INTC "ak-av130-intc"
OBJECT_DECLARE_SIMPLE_TYPE(AkAv130IntcState, AK_AV130_INTC)

#define INTC_SIZE           0x20000
#define NSUB                5
#define NTIMERS             5
#define TIMER_BASE          0x10000
#define TIMER_STRIDE        0x1000
#define TIMER_FREQ          12000000
#define TIMER_TOP_BIT       27

#define T_LOAD              0x00
#define T_VALUE             0x04
#define T_CTRL              0x08
#define T_INTCLR            0x0C
#define T_INT_EN            0x10
#define T_DIV               0x14
#define T_INT_STA           0x18

#define CTRL_EN             (1u << 0)
#define CTRL_MODE(c)        (((c) >> 1) & 3)
#define CTRL_RELOAD         (1u << 3)
#define MODE_FREE           0
#define MODE_PERIOD         1
#define MODE_ONESHOT        2

/* Sub-banks in register order (+0x0C upwards) and their top bits. */
static const int sub_top_bit[NSUB] = { 13, 17, 0, 1, 2 };   /* TWI SSS NSS ISS ESS */

typedef struct AkAv130Timer {
    struct AkAv130IntcState *s;
    QEMUTimer *timer;
    uint32_t load, ctrl, int_en, div, sta;
    /* VALUE: base_count at base_ns, counting while CTRL_EN */
    int64_t base_ns;
    uint32_t base_count;
} AkAv130Timer;

struct AkAv130IntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq, fiq;

    uint32_t fiq_mask, irq_mask;
    uint32_t level;                 /* raw top-bank inputs */
    uint32_t sub_mask[NSUB], sub_level[NSUB];
    AkAv130Timer timer[NTIMERS];
};

static uint32_t ak_av130_top_status(AkAv130IntcState *s)
{
    uint32_t st = s->level;
    int i;

    for (i = 0; i < NSUB; i++) {
        if (s->sub_level[i] & s->sub_mask[i]) {
            st |= 1u << sub_top_bit[i];
        }
    }
    for (i = 0; i < NTIMERS; i++) {
        AkAv130Timer *t = &s->timer[i];

        if (t->sta & t->int_en & 1) {
            st |= 1u << (TIMER_TOP_BIT + i);
        }
    }
    return st;
}

static void ak_av130_update(AkAv130IntcState *s)
{
    uint32_t st = ak_av130_top_status(s);

    qemu_set_irq(s->irq, !!(st & s->irq_mask));
    qemu_set_irq(s->fiq, !!(st & s->fiq_mask));
}

static void ak_av130_set_irq(void *opaque, int n, int level)
{
    AkAv130IntcState *s = opaque;

    if (n < 32) {
        s->level = deposit32(s->level, n, 1, !!level);
    } else {
        /* 32 + 32 * sub + bit: a sub-bank input (I2C n is TWI bit n). */
        int sub = (n - 32) / 32;

        s->sub_level[sub] = deposit32(s->sub_level[sub], (n - 32) % 32, 1,
                                      !!level);
    }
    ak_av130_update(s);
}

static uint32_t ak_av130_timer_value(AkAv130Timer *t)
{
    uint64_t ticks;

    if (!(t->ctrl & CTRL_EN)) {
        return t->base_count;
    }
    ticks = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - t->base_ns,
                     TIMER_FREQ, NANOSECONDS_PER_SECOND);
    switch (CTRL_MODE(t->ctrl)) {
    case MODE_PERIOD:
        if (ticks <= t->base_count) {
            return t->base_count - ticks;
        }
        return t->load - (ticks - t->base_count - 1) % ((uint64_t)t->load + 1);
    case MODE_ONESHOT:
        return ticks < t->base_count ? t->base_count - ticks : 0;
    default:
        return t->base_count - (uint32_t)ticks;     /* wraps to 0xFFFFFFFF */
    }
}

/* Ticks from the base to the next time VALUE reaches 0, or 0 for none. */
static uint64_t ak_av130_timer_next_zero(AkAv130Timer *t, uint64_t elapsed)
{
    uint64_t period, first = t->base_count;

    switch (CTRL_MODE(t->ctrl)) {
    case MODE_ONESHOT:
        first = MAX(first, 1);
        return elapsed < first ? first : 0;
    case MODE_PERIOD:
        period = (uint64_t)t->load + 1;
        break;
    default:
        period = 1ull << 32;
        break;
    }
    if (!first) {
        first = period;
    }
    if (elapsed < first) {
        return first;
    }
    return first + ((elapsed - first) / period + 1) * period;
}

static void ak_av130_timer_schedule(AkAv130Timer *t)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t elapsed, next;

    if (!(t->ctrl & CTRL_EN)) {
        timer_del(t->timer);
        return;
    }
    elapsed = muldiv64(now - t->base_ns, TIMER_FREQ, NANOSECONDS_PER_SECOND);
    next = ak_av130_timer_next_zero(t, elapsed);
    if (!next) {
        timer_del(t->timer);
        return;
    }
    /* rounded up, so VALUE reads 0 when it fires */
    timer_mod(t->timer, t->base_ns +
              (int64_t)muldiv64_round_up(next, NANOSECONDS_PER_SECOND,
                                         TIMER_FREQ));
}

static void ak_av130_timer_tick(void *opaque)
{
    AkAv130Timer *t = opaque;

    t->sta |= 1;
    if (CTRL_MODE(t->ctrl) == MODE_ONESHOT) {
        t->base_count = 0;
        t->ctrl &= ~CTRL_EN;
    }
    ak_av130_timer_schedule(t);
    ak_av130_update(t->s);
}

/* VALUE carries on from now, from where it is or from start. */
static void ak_av130_timer_rebase(AkAv130Timer *t, bool reload)
{
    t->base_count = reload ? t->load : ak_av130_timer_value(t);
    t->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void ak_av130_timer_ctrl(AkAv130Timer *t, uint32_t val)
{
    ak_av130_timer_rebase(t, val & CTRL_RELOAD);
    t->ctrl = val & ~CTRL_RELOAD;
    ak_av130_timer_schedule(t);
}

static uint64_t ak_av130_read(void *opaque, hwaddr off, unsigned size)
{
    AkAv130IntcState *s = opaque;

    if (off >= TIMER_BASE && off < TIMER_BASE + NTIMERS * TIMER_STRIDE) {
        AkAv130Timer *t = &s->timer[(off - TIMER_BASE) / TIMER_STRIDE];

        switch (off & (TIMER_STRIDE - 1)) {
        case T_LOAD:    return t->load;
        case T_VALUE:   return ak_av130_timer_value(t);
        case T_CTRL:    return t->ctrl;
        case T_INT_EN:  return t->int_en;
        case T_DIV:     return t->div;
        case T_INT_STA: return t->sta;
        default:        return 0;
        }
    }
    switch (off) {
    case 0x00: return s->fiq_mask;
    case 0x04: return s->irq_mask;
    case 0x08: return ak_av130_top_status(s);
    case 0x0C ... 0x30:
        if ((off - 0x0C) % 8 == 0) {
            return s->sub_mask[(off - 0x0C) / 8];
        }
        return s->sub_level[(off - 0x0C) / 8];
    default:
        qemu_log_mask(LOG_UNIMP, "ak-av130-intc: read of 0x%05" HWADDR_PRIx
                      "\n", off);
        return 0;
    }
}

static void ak_av130_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    AkAv130IntcState *s = opaque;

    if (off >= TIMER_BASE && off < TIMER_BASE + NTIMERS * TIMER_STRIDE) {
        AkAv130Timer *t = &s->timer[(off - TIMER_BASE) / TIMER_STRIDE];

        switch (off & (TIMER_STRIDE - 1)) {
        case T_LOAD:
            /*
             * A running timer finishes the count it is in; the periods
             * after it are the new LOAD's.
             */
            ak_av130_timer_rebase(t, false);
            t->load = val;
            ak_av130_timer_schedule(t);
            break;
        case T_CTRL:
            ak_av130_timer_ctrl(t, val);
            break;
        case T_INTCLR:
            if (val & 1) {
                t->sta = 0;
            }
            break;
        case T_INT_EN:
            t->int_en = val;
            break;
        case T_DIV:
            t->div = val;
            break;
        default:
            break;
        }
        ak_av130_update(s);
        return;
    }
    switch (off) {
    case 0x00:
        s->fiq_mask = val;
        break;
    case 0x04:
        s->irq_mask = val;
        break;
    case 0x0C ... 0x30:
        if ((off - 0x0C) % 8 == 0) {
            s->sub_mask[(off - 0x0C) / 8] = val;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-av130-intc: write 0x%08" PRIx64
                      " to 0x%05" HWADDR_PRIx "\n", val, off);
        break;
    }
    ak_av130_update(s);
}

static const MemoryRegionOps ak_av130_ops = {
    .read = ak_av130_read,
    .write = ak_av130_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ak_av130_reset(DeviceState *dev)
{
    AkAv130IntcState *s = AK_AV130_INTC(dev);
    int i;

    s->fiq_mask = s->irq_mask = 0;
    memset(s->sub_mask, 0, sizeof(s->sub_mask));
    for (i = 0; i < NTIMERS; i++) {
        AkAv130Timer *t = &s->timer[i];

        timer_del(t->timer);
        t->load = t->ctrl = t->int_en = t->div = t->sta = 0;
        t->base_count = 0;
        t->base_ns = 0;
    }
    ak_av130_update(s);
}

static void ak_av130_init(Object *obj)
{
    AkAv130IntcState *s = AK_AV130_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &ak_av130_ops, s,
                          TYPE_AK_AV130_INTC, INTC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), ak_av130_set_irq, 32 + 32 * NSUB);
    for (i = 0; i < NTIMERS; i++) {
        AkAv130Timer *t = &s->timer[i];

        t->s = s;
        t->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ak_av130_timer_tick, t);
    }
}

static const VMStateDescription vmstate_ak_av130_timer = {
    .name = "ak-av130-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, AkAv130Timer),
        VMSTATE_UINT32(load, AkAv130Timer),
        VMSTATE_UINT32(ctrl, AkAv130Timer),
        VMSTATE_UINT32(int_en, AkAv130Timer),
        VMSTATE_UINT32(div, AkAv130Timer),
        VMSTATE_UINT32(sta, AkAv130Timer),
        VMSTATE_INT64(base_ns, AkAv130Timer),
        VMSTATE_UINT32(base_count, AkAv130Timer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ak_av130_intc = {
    .name = TYPE_AK_AV130_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(fiq_mask, AkAv130IntcState),
        VMSTATE_UINT32(irq_mask, AkAv130IntcState),
        VMSTATE_UINT32(level, AkAv130IntcState),
        VMSTATE_UINT32_ARRAY(sub_mask, AkAv130IntcState, NSUB),
        VMSTATE_UINT32_ARRAY(sub_level, AkAv130IntcState, NSUB),
        VMSTATE_STRUCT_ARRAY(timer, AkAv130IntcState, NTIMERS, 1,
                             vmstate_ak_av130_timer, AkAv130Timer),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_av130_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_av130_intc;
    device_class_set_legacy_reset(dc, ak_av130_reset);
}

static const TypeInfo ak_av130_intc_info = {
    .name          = TYPE_AK_AV130_INTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkAv130IntcState),
    .instance_init = ak_av130_init,
    .class_init    = ak_av130_class_init,
};

static void ak_av130_register_types(void)
{
    type_register_static(&ak_av130_intc_info);
}

type_init(ak_av130_register_types)
