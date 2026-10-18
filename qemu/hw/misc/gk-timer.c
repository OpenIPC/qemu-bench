/*
 * Goke GK710x interval timers.
 *
 * Three 32-bit down-counters clocked from APB, in the Ambarella layout the
 * silicon really has (the Goke kernel and U-Boot reach it through the
 * vendor HAL, which shows them a shuffled one -- include/hw/arm/goke.h):
 *
 *   0x00 T1 STATUS   0x04 T1 RELOAD   0x08 T1 MATCH1   0x0C T1 MATCH2
 *   0x10 T2 ...      0x20 T3 ...      0x30 CONTROL
 *
 * STATUS is the live count and may be written.  When it passes zero the
 * counter reloads from RELOAD and, if the timer's overflow-interrupt bit is
 * set, pulses its interrupt line (the kernel registers the timers rising
 * edge and acks them at the VIC).  CONTROL holds a nibble per timer at
 * 4 * n: bit 0 enable, bit 1 external clock, bit 2 overflow interrupt.
 * The match registers are kept but do not interrupt: neither the kernel
 * (plat-goke/timer.c) nor U-Boot programs anything but 0 into them.
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

#define TYPE_GK_TIMER "gk-timer"
OBJECT_DECLARE_SIMPLE_TYPE(GkTimerState, GK_TIMER)

#define GK_TIMER_NUM    3
#define GK_TIMER_CTRL   0x30

#define CTRL_EN         (1u << 0)
#define CTRL_OF         (1u << 2)

typedef struct GkTimerUnit {
    struct GkTimerState *parent;
    int index;
    ptimer_state *ptimer;
    uint32_t count;     /* STATUS while stopped */
    uint32_t reload;
    uint32_t match[2];
    qemu_irq irq;
} GkTimerUnit;

struct GkTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    GkTimerUnit unit[GK_TIMER_NUM];
    uint32_t control;
    uint32_t freq;
};

static uint32_t gk_timer_ctrl(GkTimerUnit *t)
{
    return (t->parent->control >> (4 * t->index)) & 0xF;
}

static void gk_timer_tick(void *opaque)
{
    GkTimerUnit *t = opaque;

    if (gk_timer_ctrl(t) & CTRL_OF) {
        qemu_irq_pulse(t->irq);
    }
}

static uint32_t gk_timer_get_count(GkTimerUnit *t)
{
    if (gk_timer_ctrl(t) & CTRL_EN) {
        return ptimer_get_count(t->ptimer);
    }
    return t->count;
}

static uint64_t gk_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    GkTimerState *s = opaque;
    GkTimerUnit *t;

    if (offset == GK_TIMER_CTRL) {
        return s->control;
    }
    if (offset >= GK_TIMER_CTRL) {
        qemu_log_mask(LOG_GUEST_ERROR, "gk-timer: read of 0x%02" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
    t = &s->unit[offset >> 4];
    switch (offset & 0xF) {
    case 0x0:
        return gk_timer_get_count(t);
    case 0x4:
        return t->reload;
    case 0x8:
        return t->match[0];
    default:
        return t->match[1];
    }
}

static void gk_timer_set_control(GkTimerState *s, uint32_t val)
{
    uint32_t old = s->control;
    int n;

    s->control = val;
    for (n = 0; n < GK_TIMER_NUM; n++) {
        GkTimerUnit *t = &s->unit[n];
        bool was = (old >> (4 * n)) & CTRL_EN;
        bool now = (val >> (4 * n)) & CTRL_EN;

        if (was == now) {
            continue;
        }
        ptimer_transaction_begin(t->ptimer);
        if (now) {
            ptimer_set_limit(t->ptimer, t->reload, 0);
            ptimer_set_count(t->ptimer, t->count);
            ptimer_run(t->ptimer, 0);
        } else {
            t->count = ptimer_get_count(t->ptimer);
            ptimer_stop(t->ptimer);
        }
        ptimer_transaction_commit(t->ptimer);
    }
}

static void gk_timer_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    GkTimerState *s = opaque;
    GkTimerUnit *t;

    if (offset == GK_TIMER_CTRL) {
        gk_timer_set_control(s, val);
        return;
    }
    if (offset >= GK_TIMER_CTRL) {
        qemu_log_mask(LOG_GUEST_ERROR, "gk-timer: write 0x%08" PRIx64
                      " to 0x%02" HWADDR_PRIx "\n", val, offset);
        return;
    }
    t = &s->unit[offset >> 4];
    switch (offset & 0xF) {
    case 0x0:
        t->count = val;
        if (gk_timer_ctrl(t) & CTRL_EN) {
            ptimer_transaction_begin(t->ptimer);
            ptimer_set_count(t->ptimer, val);
            ptimer_transaction_commit(t->ptimer);
        }
        break;
    case 0x4:
        t->reload = val;
        ptimer_transaction_begin(t->ptimer);
        ptimer_set_limit(t->ptimer, val, 0);
        ptimer_transaction_commit(t->ptimer);
        break;
    case 0x8:
        t->match[0] = val;
        break;
    default:
        t->match[1] = val;
        break;
    }
}

static const MemoryRegionOps gk_timer_ops = {
    .read = gk_timer_read,
    .write = gk_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gk_timer_reset(DeviceState *dev)
{
    GkTimerState *s = GK_TIMER(dev);
    int n;

    gk_timer_set_control(s, 0);
    for (n = 0; n < GK_TIMER_NUM; n++) {
        GkTimerUnit *t = &s->unit[n];

        t->count = 0;
        t->reload = 0;
        t->match[0] = t->match[1] = 0;
    }
}

static void gk_timer_init(Object *obj)
{
    GkTimerState *s = GK_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    memory_region_init_io(&s->iomem, obj, &gk_timer_ops, s, TYPE_GK_TIMER,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    for (n = 0; n < GK_TIMER_NUM; n++) {
        sysbus_init_irq(sbd, &s->unit[n].irq);
    }
}

static void gk_timer_realize(DeviceState *dev, Error **errp)
{
    GkTimerState *s = GK_TIMER(dev);
    int n;

    for (n = 0; n < GK_TIMER_NUM; n++) {
        GkTimerUnit *t = &s->unit[n];

        t->parent = s;
        t->index = n;
        /*
         * A reload of 0 keeps reloading rather than stopping, and a timer
         * enabled at 0 waits a period instead of firing at once: the
         * kernel's one-shot mode arms STATUS = 0 before the first event.
         */
        t->ptimer = ptimer_init(gk_timer_tick, t,
                                PTIMER_POLICY_CONTINUOUS_TRIGGER |
                                PTIMER_POLICY_NO_IMMEDIATE_TRIGGER);
        ptimer_transaction_begin(t->ptimer);
        ptimer_set_freq(t->ptimer, s->freq);
        ptimer_transaction_commit(t->ptimer);
    }
}

static const Property gk_timer_properties[] = {
    /* APB clock: what U-Boot assumes (APB_CLK, timer.c) and what the
     * kernel falls back to when the PLL registers read zero (rct.c). */
    DEFINE_PROP_UINT32("freq", GkTimerState, freq, 69000000),
};

static const VMStateDescription vmstate_gk_timer_unit = {
    .name = "gk-timer-unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(ptimer, GkTimerUnit),
        VMSTATE_UINT32(count, GkTimerUnit),
        VMSTATE_UINT32(reload, GkTimerUnit),
        VMSTATE_UINT32_ARRAY(match, GkTimerUnit, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gk_timer = {
    .name = TYPE_GK_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(unit, GkTimerState, GK_TIMER_NUM, 1,
                             vmstate_gk_timer_unit, GkTimerUnit),
        VMSTATE_UINT32(control, GkTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void gk_timer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = gk_timer_realize;
    device_class_set_legacy_reset(dc, gk_timer_reset);
    device_class_set_props(dc, gk_timer_properties);
    dc->vmsd = &vmstate_gk_timer;
}

static const TypeInfo gk_timer_info = {
    .name          = TYPE_GK_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkTimerState),
    .instance_init = gk_timer_init,
    .class_init    = gk_timer_class_init,
};

static void gk_timer_register_types(void)
{
    type_register_static(&gk_timer_info);
}

type_init(gk_timer_register_types)
