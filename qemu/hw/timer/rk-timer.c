/*
 * Rockchip timer ("rockchip,rk3288-timer": RV1126 0xFF660000, six
 * channels 0x20 apart), which the vendor kernels use as a clockevent
 * (drivers/clocksource/timer-rockchip.c): the broadcast timer cpuidle
 * hands over to.  Without it the first idle sleeps for ever.
 *
 * One channel:
 *
 *   0x00 LOAD_COUNT0 / 0x04 LOAD_COUNT1   64-bit start value
 *   0x08 CURRENT_VALUE0 / 0x0c CURRENT_VALUE1   counting down to 0
 *   0x10 CONTROL   bit 0 enable, bit 1 user-defined count (one shot:
 *                  stop at 0), else free running (reload), bit 2 IRQ
 *   0x18 INT_STATUS  bit 0, write 1 to clear
 *
 * The kernel's sched clock reads ~CURRENT_VALUE0, so the count goes
 * down; at 0 the channel raises INT_STATUS and either stops or reloads.
 * The clock is "freq" (24 MHz, xin24m).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
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

#define TYPE_RK_TIMER "rk-timer"
OBJECT_DECLARE_SIMPLE_TYPE(RkTimerState, RK_TIMER)

#define NCHAN           6
#define CHAN_SIZE       0x20
#define R_LOAD0         0x00
#define R_LOAD1         0x04
#define R_CUR0          0x08
#define R_CUR1          0x0c
#define R_CONTROL       0x10
#define R_INT_STATUS    0x18

#define CTL_ENABLE      (1u << 0)
#define CTL_USER        (1u << 1)
#define CTL_IRQ         (1u << 2)

typedef struct RkTimerChan {
    struct RkTimerState *s;
    ptimer_state *pt;
    qemu_irq irq;
    uint64_t load;
    uint32_t control;
    uint32_t int_status;
} RkTimerChan;

struct RkTimerState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    RkTimerChan ch[NCHAN];
    uint32_t freq;
};

static void rk_timer_update_irq(RkTimerChan *c)
{
    qemu_set_irq(c->irq, c->int_status && (c->control & CTL_IRQ));
}

static void rk_timer_tick(void *opaque)
{
    RkTimerChan *c = opaque;

    c->int_status = 1;
    if (c->control & CTL_USER) {
        ptimer_stop(c->pt);         /* one shot: stays at 0 */
    }
    rk_timer_update_irq(c);
}

/* Called inside a ptimer transaction. */
static void rk_timer_apply(RkTimerChan *c)
{
    if (c->control & CTL_ENABLE) {
        uint64_t n = c->load ? c->load : 1;

        ptimer_set_limit(c->pt, n, 1);
        ptimer_run(c->pt, (c->control & CTL_USER) ? 1 : 0);
    } else {
        ptimer_stop(c->pt);
    }
}

static uint64_t rk_timer_read(void *opaque, hwaddr off, unsigned size)
{
    RkTimerState *s = opaque;
    RkTimerChan *c = &s->ch[off / CHAN_SIZE];
    uint64_t cur;

    switch (off % CHAN_SIZE) {
    case R_LOAD0:
        return (uint32_t)c->load;
    case R_LOAD1:
        return c->load >> 32;
    case R_CUR0:
    case R_CUR1:
        cur = (c->control & CTL_ENABLE) ? ptimer_get_count(c->pt) : c->load;
        return (off % CHAN_SIZE) == R_CUR0 ? (uint32_t)cur : cur >> 32;
    case R_CONTROL:
        return c->control;
    case R_INT_STATUS:
        return c->int_status;
    }
    return 0;
}

static void rk_timer_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkTimerState *s = opaque;
    RkTimerChan *c = &s->ch[off / CHAN_SIZE];

    ptimer_transaction_begin(c->pt);
    switch (off % CHAN_SIZE) {
    case R_LOAD0:
        c->load = deposit64(c->load, 0, 32, val);
        break;
    case R_LOAD1:
        c->load = deposit64(c->load, 32, 32, val);
        break;
    case R_CONTROL:
        c->control = val & 7;
        rk_timer_apply(c);
        break;
    case R_INT_STATUS:
        if (val & 1) {
            c->int_status = 0;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rk-timer: write 0x%" HWADDR_PRIx
                      "\n", off);
        break;
    }
    ptimer_transaction_commit(c->pt);
    rk_timer_update_irq(c);
}

static const MemoryRegionOps rk_timer_ops = {
    .read = rk_timer_read,
    .write = rk_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_timer_reset(DeviceState *dev)
{
    RkTimerState *s = RK_TIMER(dev);
    int i;

    for (i = 0; i < NCHAN; i++) {
        RkTimerChan *c = &s->ch[i];

        ptimer_transaction_begin(c->pt);
        ptimer_stop(c->pt);
        ptimer_set_freq(c->pt, s->freq);
        ptimer_transaction_commit(c->pt);
        c->load = 0;
        c->control = 0;
        c->int_status = 0;
        rk_timer_update_irq(c);
    }
}

static void rk_timer_realize(DeviceState *dev, Error **errp)
{
    RkTimerState *s = RK_TIMER(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_timer_ops, s,
                          TYPE_RK_TIMER, NCHAN * CHAN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    for (i = 0; i < NCHAN; i++) {
        RkTimerChan *c = &s->ch[i];

        c->s = s;
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &c->irq);
        c->pt = ptimer_init(rk_timer_tick, c,
                            PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                            PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                            PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                            PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
    }
}

static const Property rk_timer_properties[] = {
    DEFINE_PROP_UINT32("freq", RkTimerState, freq, 24000000),
};

static const VMStateDescription rk_timer_chan_vmstate = {
    .name = "rk-timer-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PTIMER(pt, RkTimerChan),
        VMSTATE_UINT64(load, RkTimerChan),
        VMSTATE_UINT32(control, RkTimerChan),
        VMSTATE_UINT32(int_status, RkTimerChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription rk_timer_vmstate = {
    .name = TYPE_RK_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, RkTimerState, NCHAN, 1,
                             rk_timer_chan_vmstate, RkTimerChan),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_timer_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_timer_realize;
    device_class_set_legacy_reset(dc, rk_timer_reset);
    device_class_set_props(dc, rk_timer_properties);
    dc->vmsd = &rk_timer_vmstate;
}

static const TypeInfo rk_timer_info = {
    .name = TYPE_RK_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkTimerState),
    .class_init = rk_timer_class_init,
};

static void rk_timer_register_types(void)
{
    type_register_static(&rk_timer_info);
}

type_init(rk_timer_register_types)
