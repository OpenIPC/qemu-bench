/*
 * Ambarella interval timers: eight down-counters behind one control word.
 *
 * Each timer has STATUS (the live count), RELOAD, MATCH1 and MATCH2.  The
 * first three sit at 0x00, 0x10 and 0x20, the control word at 0x30 with a
 * nibble per timer, and the rest from 0x34.  A count passing zero reloads
 * and, with OF set, pulses the timer's line.  CSL picks an external clock,
 * which nothing here uses.
 *
 * Linux keeps timer 7 free-running as its clocksource, reading -STATUS, and
 * timer 8 as a clockevent that takes a new STATUS for each event.  They
 * count gclk_apb, 90 MHz at the PLL rates amb-rct resets to.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define TYPE_AMB_TIMER "amb-timer"
OBJECT_DECLARE_SIMPLE_TYPE(AmbTimerState, AMB_TIMER)

#define AMB_TIMERS      8
#define R_CTRL          0x30
#define CTRL_EN         0x1
#define CTRL_CSL        0x2
#define CTRL_OF         0x4

/*
 * The shortest a reloading timer repeats.  The silicon would go faster with
 * RELOAD near 0, but at 11 ns a tick that only leaves the host spinning.
 */
#define AMB_TIMER_MIN_PERIOD_NS 1000

typedef struct AmbTimer {
    struct AmbTimerState *owner;
    QEMUTimer *timer;
    qemu_irq irq;
    int64_t base_ns;    /* when count was taken */
    uint32_t count;
    uint32_t reload;
    uint32_t match1;
    uint32_t match2;
} AmbTimer;

struct AmbTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t ctrl;
    uint32_t freq;
    AmbTimer t[AMB_TIMERS];
};

static const uint16_t amb_timer_base[AMB_TIMERS] = {
    0x00, 0x10, 0x20, 0x34, 0x44, 0x54, 0x64, 0x74,
};

static uint32_t amb_timer_ctrl(AmbTimerState *s, int n)
{
    return (s->ctrl >> (n * 4)) & 0xf;
}

static int amb_timer_index(AmbTimer *t)
{
    return t - t->owner->t;
}

static int64_t amb_timer_ticks_to_ns(AmbTimerState *s, uint64_t ticks)
{
    /* Rounded up, so a deadline is never before its last tick */
    return muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq) + 1;
}

/* Bring count up to now, following any reloads since base_ns. */
static void amb_timer_sync(AmbTimer *t)
{
    AmbTimerState *s = t->owner;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ticks;

    if (!(amb_timer_ctrl(s, amb_timer_index(t)) & CTRL_EN)) {
        t->base_ns = now;
        return;
    }
    ticks = muldiv64(now - t->base_ns, s->freq, NANOSECONDS_PER_SECOND);
    if (ticks <= t->count) {
        t->count -= ticks;
    } else {
        uint64_t past = ticks - t->count - 1;
        uint64_t period = (uint64_t)t->reload + 1;

        t->count = t->reload - past % period;
    }
    /* Only the whole ticks: the rest of one still counts towards the next */
    t->base_ns += muldiv64(ticks, NANOSECONDS_PER_SECOND, s->freq);
}

static void amb_timer_arm(AmbTimer *t)
{
    AmbTimerState *s = t->owner;

    if (amb_timer_ctrl(s, amb_timer_index(t)) & CTRL_EN) {
        timer_mod(t->timer, t->base_ns +
                  amb_timer_ticks_to_ns(s, (uint64_t)t->count + 1));
    } else {
        timer_del(t->timer);
    }
}

static void amb_timer_expire(void *opaque)
{
    AmbTimer *t = opaque;
    AmbTimerState *s = t->owner;

    /*
     * The count passed zero: that is why this ran.  Working it out from the
     * clock instead can come to zero ticks, the conversions being truncated,
     * and re-arm at the same instant for ever.
     */
    t->count = t->reload;
    t->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (amb_timer_ctrl(s, amb_timer_index(t)) & CTRL_OF) {
        qemu_irq_pulse(t->irq);
    }
    amb_timer_arm(t);
    if (timer_pending(t->timer)) {
        timer_mod(t->timer, MAX(timer_expire_time_ns(t->timer),
                                t->base_ns + AMB_TIMER_MIN_PERIOD_NS));
    }
}

static AmbTimer *amb_timer_decode(AmbTimerState *s, hwaddr off, unsigned *reg)
{
    for (int i = 0; i < AMB_TIMERS; i++) {
        if (off >= amb_timer_base[i] && off < amb_timer_base[i] + 0x10) {
            *reg = off - amb_timer_base[i];
            return &s->t[i];
        }
    }
    return NULL;
}

static uint64_t amb_timer_read(void *opaque, hwaddr off, unsigned size)
{
    AmbTimerState *s = opaque;
    unsigned reg;
    AmbTimer *t;

    if (off == R_CTRL) {
        return s->ctrl;
    }
    t = amb_timer_decode(s, off, &reg);
    if (!t) {
        qemu_log_mask(LOG_UNIMP, "amb-timer: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
    switch (reg) {
    case 0x0:
        amb_timer_sync(t);
        return t->count;
    case 0x4:
        return t->reload;
    case 0x8:
        return t->match1;
    default:
        return t->match2;
    }
}

static void amb_timer_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    AmbTimerState *s = opaque;
    unsigned reg;
    AmbTimer *t;

    if (off == R_CTRL) {
        for (int i = 0; i < AMB_TIMERS; i++) {
            amb_timer_sync(&s->t[i]);
        }
        s->ctrl = val;
        for (int i = 0; i < AMB_TIMERS; i++) {
            amb_timer_arm(&s->t[i]);
        }
        return;
    }
    t = amb_timer_decode(s, off, &reg);
    if (!t) {
        qemu_log_mask(LOG_UNIMP, "amb-timer: write 0x%" HWADDR_PRIx "\n", off);
        return;
    }
    amb_timer_sync(t);
    switch (reg) {
    case 0x0:
        t->count = val;
        break;
    case 0x4:
        t->reload = val;
        break;
    case 0x8:
        t->match1 = val;
        break;
    default:
        t->match2 = val;
        break;
    }
    amb_timer_arm(t);
}

static const MemoryRegionOps amb_timer_ops = {
    .read = amb_timer_read,
    .write = amb_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_timer_reset(DeviceState *dev)
{
    AmbTimerState *s = AMB_TIMER(dev);

    s->ctrl = 0;
    for (int i = 0; i < AMB_TIMERS; i++) {
        timer_del(s->t[i].timer);
        s->t[i].count = s->t[i].reload = 0;
        s->t[i].match1 = s->t[i].match2 = 0;
        s->t[i].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void amb_timer_init(Object *obj)
{
    AmbTimerState *s = AMB_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &amb_timer_ops, s, TYPE_AMB_TIMER,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < AMB_TIMERS; i++) {
        s->t[i].owner = s;
        s->t[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, amb_timer_expire,
                                     &s->t[i]);
        sysbus_init_irq(sbd, &s->t[i].irq);
    }
}

static const VMStateDescription amb_timer_one_vmstate = {
    .name = "amb-timer-one",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, AmbTimer),
        VMSTATE_INT64(base_ns, AmbTimer),
        VMSTATE_UINT32(count, AmbTimer),
        VMSTATE_UINT32(reload, AmbTimer),
        VMSTATE_UINT32(match1, AmbTimer),
        VMSTATE_UINT32(match2, AmbTimer),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription amb_timer_vmstate = {
    .name = TYPE_AMB_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, AmbTimerState),
        VMSTATE_STRUCT_ARRAY(t, AmbTimerState, AMB_TIMERS, 1,
                             amb_timer_one_vmstate, AmbTimer),
        VMSTATE_END_OF_LIST()
    },
};

static const Property amb_timer_props[] = {
    DEFINE_PROP_UINT32("freq", AmbTimerState, freq, 90000000),
};

static void amb_timer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, amb_timer_reset);
    device_class_set_props(dc, amb_timer_props);
    dc->vmsd = &amb_timer_vmstate;
}

static const TypeInfo amb_timer_info = {
    .name = TYPE_AMB_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbTimerState),
    .instance_init = amb_timer_init,
    .class_init = amb_timer_class_init,
};

static void amb_timer_register(void)
{
    type_register_static(&amb_timer_info);
}

type_init(amb_timer_register)
