/*
 * MStar infinity / infinity3 RTC (PM bank 0x0012, 0x1F002400).
 *
 * A 32-bit seconds counter in the PM domain, as the vendor kernel's
 * drivers/mstar/rtc/ms_rtc.c ("mstar,infinity-rtc") drives it:
 *
 *   0x00  control: bit 0 SOFT_RSTZ, 1 CNT_EN, 2 WRAP_EN, 3 LOAD_EN,
 *         4 READ_EN, 5 INT_MASK, 6 INT_FORCE, 7 INT_CLEAR
 *   0x01/0x02  FREQ_CW: the input clock in Hz, what makes a second
 *   0x03/0x04  LOAD_VAL: taken into the counter by LOAD_EN
 *   0x05/0x06  MATCH_VAL: the alarm, an interrupt while INT_MASK is clear
 *   0x08/0x09  CNT_VAL: the counter as latched by READ_EN
 *
 * LOAD_EN and READ_EN are requests the hardware clears once done; the
 * driver spins on them with no timeout.  INT_CLEAR clears itself.  The
 * counter starts at the host's time, like an RTC that kept its battery.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_SSTAR_RTC "sstar-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(SstarRtcState, SSTAR_RTC)

#define RTC_CTRL            0x00
#define RTC_LOAD_L          0x03
#define RTC_LOAD_H          0x04
#define RTC_MATCH_L         0x05
#define RTC_MATCH_H         0x06
#define RTC_CNT_L           0x08
#define RTC_CNT_H           0x09
#define RTC_NREGS           0x10

#define CTRL_SOFT_RSTZ      (1u << 0)
#define CTRL_LOAD_EN        (1u << 3)
#define CTRL_READ_EN        (1u << 4)
#define CTRL_INT_MASK       (1u << 5)
#define CTRL_INT_FORCE      (1u << 6)
#define CTRL_INT_CLEAR      (1u << 7)

struct SstarRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint16_t regs[RTC_NREGS];
    int64_t tick_offset;        /* counter - host seconds */
    bool pending;
};

static uint32_t sstar_rtc_now(SstarRtcState *s)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND +
           s->tick_offset;
}

static uint32_t sstar_rtc_match(SstarRtcState *s)
{
    return s->regs[RTC_MATCH_L] | ((uint32_t)s->regs[RTC_MATCH_H] << 16);
}

static void sstar_rtc_update_irq(SstarRtcState *s)
{
    qemu_set_irq(s->irq, s->pending &&
                 !(s->regs[RTC_CTRL] & CTRL_INT_MASK));
}

/* Arm the timer for the next second the counter equals MATCH_VAL. */
static void sstar_rtc_arm(SstarRtcState *s)
{
    uint32_t now = sstar_rtc_now(s);
    uint32_t delta = sstar_rtc_match(s) - now;

    if (s->regs[RTC_CTRL] & CTRL_INT_MASK || !delta) {
        timer_del(s->timer);
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(rtc_clock) +
              (int64_t)delta * NANOSECONDS_PER_SECOND);
}

static void sstar_rtc_alarm(void *opaque)
{
    SstarRtcState *s = opaque;

    s->pending = true;
    sstar_rtc_update_irq(s);
}

static uint64_t sstar_rtc_read(void *opaque, hwaddr off, unsigned size)
{
    SstarRtcState *s = opaque;
    int reg = (off >> 2) & (RTC_NREGS - 1);
    uint16_t v;

    if ((off & 3) >= 2 || off >= RTC_NREGS * 4) {
        return 0;
    }
    v = s->regs[reg];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_rtc_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarRtcState *s = opaque;
    int reg = (off >> 2) & (RTC_NREGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2 || off >= RTC_NREGS * 4) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case RTC_CNT_L:
    case RTC_CNT_H:
        return;                 /* read only */
    case RTC_CTRL:
        if (v & CTRL_READ_EN) {
            uint32_t now = sstar_rtc_now(s);

            s->regs[RTC_CNT_L] = now;
            s->regs[RTC_CNT_H] = now >> 16;
        }
        if (v & CTRL_LOAD_EN) {
            uint32_t load = s->regs[RTC_LOAD_L] |
                            ((uint32_t)s->regs[RTC_LOAD_H] << 16);

            s->tick_offset = (int64_t)load -
                qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
        }
        if (v & CTRL_INT_FORCE) {
            s->pending = true;
        }
        if (v & CTRL_INT_CLEAR) {
            s->pending = false;
        }
        s->regs[reg] = v & ~(CTRL_READ_EN | CTRL_LOAD_EN | CTRL_INT_CLEAR);
        sstar_rtc_arm(s);
        sstar_rtc_update_irq(s);
        return;
    }
    s->regs[reg] = v;
    if (reg == RTC_MATCH_L || reg == RTC_MATCH_H) {
        sstar_rtc_arm(s);
    }
}

static const MemoryRegionOps sstar_rtc_ops = {
    .read = sstar_rtc_read,
    .write = sstar_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_rtc_reset(DeviceState *dev)
{
    SstarRtcState *s = SSTAR_RTC(dev);

    /* The PM domain and its counter survive a chip reset; the rest not. */
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[RTC_CTRL] = CTRL_INT_MASK;
    s->pending = false;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void sstar_rtc_init(Object *obj)
{
    SstarRtcState *s = SSTAR_RTC(obj);
    struct tm tm;

    memory_region_init_io(&s->iomem, obj, &sstar_rtc_ops, s,
                          TYPE_SSTAR_RTC, 0x40);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(rtc_clock, sstar_rtc_alarm, s);
    qemu_get_timedate(&tm, 0);
    s->tick_offset = mktimegm(&tm) -
        qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
}

static const VMStateDescription vmstate_sstar_rtc = {
    .name = TYPE_SSTAR_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarRtcState, RTC_NREGS),
        VMSTATE_INT64(tick_offset, SstarRtcState),
        VMSTATE_BOOL(pending, SstarRtcState),
        VMSTATE_TIMER_PTR(timer, SstarRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_rtc_reset);
    dc->vmsd = &vmstate_sstar_rtc;
}

static const TypeInfo sstar_rtc_info = {
    .name           = TYPE_SSTAR_RTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarRtcState),
    .instance_init  = sstar_rtc_init,
    .class_init     = sstar_rtc_class_init,
};

static void sstar_rtc_register_types(void)
{
    type_register_static(&sstar_rtc_info);
}

type_init(sstar_rtc_register_types)
