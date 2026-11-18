/*
 * Faraday FTTMR010 timer, as on Novatek's NA51068 (NT98321 / NT98323).
 *
 * Three 32-bit timers at 0x10 apart, one interrupt line for all three
 * (drivers/clocksource/fttmr010.c in the NA51068 SDK kernel):
 *
 *   n*0x10 + 0x00 COUNTER   reads the count; a write loads it
 *   n*0x10 + 0x04 LOAD      reload value
 *   n*0x10 + 0x08 MATCH1, 0x0C MATCH2
 *   0x30 CR          per timer n (shift 3n): bit 0 enable, 1 external
 *                    clock, 2 overflow interrupt; bits 9 + n count up
 *   0x34 INTR_STATE  per timer n (shift 3n): bit 0 match 1, 1 match 2,
 *                    2 overflow; writing ones clears them
 *   0x38 INTR_MASK   the same bits, set = masked
 *
 * The kernel runs timer 1 as a free-running down counter from 0xFFFFFFFF
 * (its clocksource reads ~COUNTER) and timer 0 as the clockevent: it
 * writes COUNTER with the delta and takes the overflow interrupt.  A
 * timer that counts down to zero sets its overflow bit and reloads from
 * LOAD; with LOAD 0 (the clockevent's one-shot mode) it then stops at 0
 * rather than interrupting at every tick until the next COUNTER write.
 *
 * Counting up (CR bit 9 + n), as the loader runs timers 0 and 1 for its
 * udelay() (CR 0x61B, LOAD 0xFFFFFFFF, COUNTER 0): COUNTER climbs from
 * what was last written, wrapping at 2^32, with no interrupt.
 *
 * Not modelled: the match interrupts (no NA51068 driver uses them), an
 * up-counting timer's overflow interrupt, and a separate APB clock --
 * both clocks run at "freq", the tree's clock-frequency (12 MHz).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_FTTMR010 "nvt-fttmr010"
OBJECT_DECLARE_SIMPLE_TYPE(NvtFttmr010State, NVT_FTTMR010)

#define FTTMR_TIMERS        3
#define FTTMR_COUNTER       0x00
#define FTTMR_LOAD          0x04
#define FTTMR_MATCH1        0x08
#define FTTMR_MATCH2        0x0C
#define FTTMR_CR            0x30
#define FTTMR_INTR_STATE    0x34
#define FTTMR_INTR_MASK     0x38

#define CR_ENABLE(n)        (1u << (3 * (n)))
#define CR_OFENABLE(n)      (4u << (3 * (n)))
#define CR_UP(n)            (1u << (9 + (n)))
#define ST_OVERFLOW(n)      (4u << (3 * (n)))
#define INTR_BITS           0x1FF

typedef struct NvtFttmr010State NvtFttmr010State;

typedef struct {
    NvtFttmr010State *s;
    ptimer_state *ptimer;
    uint32_t load;
    uint32_t match[2];
    uint32_t up_base;       /* counting up: COUNTER at up_ns */
    int64_t up_ns;
    int n;
} NvtFttmrTimer;

struct NvtFttmr010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NvtFttmrTimer t[FTTMR_TIMERS];
    uint32_t cr;
    uint32_t state;
    uint32_t mask;
    uint32_t freq;
};

static void nvt_fttmr_update_irq(NvtFttmr010State *s)
{
    qemu_set_irq(s->irq, !!(s->state & ~s->mask & INTR_BITS));
}

static bool nvt_fttmr_up(NvtFttmrTimer *t)
{
    uint32_t cr = t->s->cr;

    return (cr & CR_UP(t->n)) && (cr & CR_ENABLE(t->n));
}

static uint32_t nvt_fttmr_up_count(NvtFttmrTimer *t)
{
    return t->up_base + muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                 t->up_ns, t->s->freq,
                                 NANOSECONDS_PER_SECOND);
}

/* Called inside the timer's ptimer transaction. */
static void nvt_fttmr_run(NvtFttmrTimer *t)
{
    NvtFttmr010State *s = t->s;

    if (nvt_fttmr_up(t)) {
        ptimer_stop(t->ptimer);     /* counted from the clock instead */
    } else if ((s->cr & CR_ENABLE(t->n)) && ptimer_get_count(t->ptimer)) {
        /* Periodic from LOAD; with LOAD 0 it stops at the next zero. */
        ptimer_set_limit(t->ptimer, t->load, 0);
        ptimer_run(t->ptimer, t->load == 0);
    } else {
        ptimer_stop(t->ptimer);
    }
}

static void nvt_fttmr_expired(void *opaque)
{
    NvtFttmrTimer *t = opaque;
    NvtFttmr010State *s = t->s;

    if (s->cr & CR_OFENABLE(t->n)) {
        s->state |= ST_OVERFLOW(t->n);
        nvt_fttmr_update_irq(s);
    }
}

static uint64_t nvt_fttmr_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtFttmr010State *s = opaque;
    NvtFttmrTimer *t;

    if (offset < FTTMR_CR) {
        t = &s->t[offset >> 4];
        switch (offset & 0xF) {
        case FTTMR_COUNTER:
            return nvt_fttmr_up(t) ? nvt_fttmr_up_count(t)
                                   : ptimer_get_count(t->ptimer);
        case FTTMR_LOAD:
            return t->load;
        case FTTMR_MATCH1:
            return t->match[0];
        default:
            return t->match[1];
        }
    }
    switch (offset) {
    case FTTMR_CR:
        return s->cr;
    case FTTMR_INTR_STATE:
        return s->state;
    case FTTMR_INTR_MASK:
        return s->mask;
    default:
        qemu_log_mask(LOG_UNIMP, "nvt-fttmr010: read 0x%02" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void nvt_fttmr_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    NvtFttmr010State *s = opaque;
    NvtFttmrTimer *t;
    int n;

    if (offset < FTTMR_CR) {
        t = &s->t[offset >> 4];
        ptimer_transaction_begin(t->ptimer);
        switch (offset & 0xF) {
        case FTTMR_COUNTER:
            t->up_base = val;
            t->up_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            ptimer_set_count(t->ptimer, val);
            nvt_fttmr_run(t);
            break;
        case FTTMR_LOAD:
            t->load = val;
            ptimer_set_limit(t->ptimer, val, 0);
            break;
        case FTTMR_MATCH1:
            t->match[0] = val;
            break;
        default:
            t->match[1] = val;
            break;
        }
        ptimer_transaction_commit(t->ptimer);
        return;
    }
    switch (offset) {
    case FTTMR_CR:
        for (n = 0; n < FTTMR_TIMERS; n++) {
            NvtFttmrTimer *tn = &s->t[n];
            bool was_up = nvt_fttmr_up(tn);
            uint32_t now = was_up ? nvt_fttmr_up_count(tn)
                                  : ptimer_get_count(tn->ptimer);

            s->cr = (s->cr & ~(CR_ENABLE(n) | CR_UP(n))) |
                    (val & (CR_ENABLE(n) | CR_UP(n)));
            ptimer_transaction_begin(tn->ptimer);
            if (nvt_fttmr_up(tn) && !was_up) {
                /* starts counting up from where it is */
                tn->up_base = now;
                tn->up_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            } else if (was_up && !nvt_fttmr_up(tn)) {
                ptimer_set_count(tn->ptimer, now);  /* and stays there */
            }
            ptimer_transaction_commit(tn->ptimer);
        }
        s->cr = val;
        for (n = 0; n < FTTMR_TIMERS; n++) {
            ptimer_transaction_begin(s->t[n].ptimer);
            nvt_fttmr_run(&s->t[n]);
            ptimer_transaction_commit(s->t[n].ptimer);
        }
        break;
    case FTTMR_INTR_STATE:
        s->state &= ~val;
        break;
    case FTTMR_INTR_MASK:
        s->mask = val & INTR_BITS;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "nvt-fttmr010: write 0x%02" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", offset, val);
        return;
    }
    nvt_fttmr_update_irq(s);
}

static const MemoryRegionOps nvt_fttmr_ops = {
    .read = nvt_fttmr_read,
    .write = nvt_fttmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_fttmr_reset(DeviceState *dev)
{
    NvtFttmr010State *s = NVT_FTTMR010(dev);
    int n;

    s->cr = 0;
    s->state = 0;
    s->mask = 0;
    for (n = 0; n < FTTMR_TIMERS; n++) {
        NvtFttmrTimer *t = &s->t[n];

        t->load = 0;
        t->match[0] = t->match[1] = 0;
        ptimer_transaction_begin(t->ptimer);
        ptimer_stop(t->ptimer);
        ptimer_set_limit(t->ptimer, 0, 1);
        ptimer_transaction_commit(t->ptimer);
    }
    nvt_fttmr_update_irq(s);
}

static void nvt_fttmr_realize(DeviceState *dev, Error **errp)
{
    NvtFttmr010State *s = NVT_FTTMR010(dev);
    int n;

    for (n = 0; n < FTTMR_TIMERS; n++) {
        NvtFttmrTimer *t = &s->t[n];

        t->s = s;
        t->n = n;
        t->ptimer = ptimer_init(nvt_fttmr_expired, t,
                                PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                                PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(t->ptimer);
        ptimer_set_freq(t->ptimer, s->freq);
        ptimer_transaction_commit(t->ptimer);
    }
}

static void nvt_fttmr_init(Object *obj)
{
    NvtFttmr010State *s = NVT_FTTMR010(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_fttmr_ops, s,
                          TYPE_NVT_FTTMR010, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_nvt_fttmr_timer = {
    .name = "nvt-fttmr010-timer",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(ptimer, NvtFttmrTimer),
        VMSTATE_UINT32(load, NvtFttmrTimer),
        VMSTATE_UINT32_ARRAY(match, NvtFttmrTimer, 2),
        VMSTATE_UINT32(up_base, NvtFttmrTimer),
        VMSTATE_INT64(up_ns, NvtFttmrTimer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_nvt_fttmr = {
    .name = TYPE_NVT_FTTMR010,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(t, NvtFttmr010State, FTTMR_TIMERS, 2,
                             vmstate_nvt_fttmr_timer, NvtFttmrTimer),
        VMSTATE_UINT32(cr, NvtFttmr010State),
        VMSTATE_UINT32(state, NvtFttmr010State),
        VMSTATE_UINT32(mask, NvtFttmr010State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_fttmr_properties[] = {
    DEFINE_PROP_UINT32("freq", NvtFttmr010State, freq, 12000000),
};

static void nvt_fttmr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_fttmr_realize;
    device_class_set_legacy_reset(dc, nvt_fttmr_reset);
    device_class_set_props(dc, nvt_fttmr_properties);
    dc->vmsd = &vmstate_nvt_fttmr;
}

static const TypeInfo nvt_fttmr_info = {
    .name          = TYPE_NVT_FTTMR010,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtFttmr010State),
    .instance_init = nvt_fttmr_init,
    .class_init    = nvt_fttmr_class_init,
};

static void nvt_fttmr_register_types(void)
{
    type_register_static(&nvt_fttmr_info);
}

type_init(nvt_fttmr_register_types)
