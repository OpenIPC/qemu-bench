/*
 * Grain Media FTRTC011 real-time clock.
 *
 * The RTC at 0x91C00000 on the GM813x (module/RTC/FTRTC011/ftrtc011.c):
 *
 *   0x00-0x0C SEC / MIN / HOUR / DAY   the running time: seconds since
 *             1970 = SEC + MIN * 60 + HOUR * 3600 + DAY * 86400
 *   0x10-0x18 ALARM_SEC / MIN / HOUR
 *   0x20 CR   0 enable, 1-4 per second / minute / hour / day interrupts,
 *             5 alarm interrupt, 6 load WSEC..WDAY into the counters,
 *             7 refresh
 *   0x24-0x30 WSEC / WMIN / WHOUR / WDAY   the time to load
 *   0x34 INTR_STATE  0-3 second / minute / hour / day, 4 alarm; write 1
 *             to clear
 *   0x3C REVISION, 0x40 RWSTATUS (bit 8 busy: never)
 *
 * The time runs on QEMU's RTC clock (-rtc base=...), starting from it.
 * The driver enables the per-second interrupt (CR = 0x83) and acknowledges
 * through INTR_STATE; the alarm compares on each second.  Everything goes
 * to interrupt 50, which is the only one the driver requests.  (The
 * Longse kernel lacks the RTC class, so its rtc-ftrtc011.ko does not load:
 * the time is then only read by kernels that have it.)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_GM_RTC "gm-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(GmRtcState, GM_RTC)

#define GM_RTC_SIZE         0x1000
#define GM_RTC_NREGS        (0x50 / 4)

#define R_SEC               0x00
#define R_MIN               0x04
#define R_HOUR              0x08
#define R_DAY               0x0C
#define R_ALARM_SEC         0x10
#define R_ALARM_MIN         0x14
#define R_ALARM_HOUR        0x18
#define R_CR                0x20
#define R_WSEC              0x24
#define R_WMIN              0x28
#define R_WHOUR             0x2C
#define R_WDAY              0x30
#define R_INTR_STATE        0x34
#define R_REVISION          0x3C
#define R_RWSTATUS          0x40

#define CR_EN               (1u << 0)
#define CR_SEC_INT          (1u << 1)
#define CR_ALARM_INT        (1u << 5)
#define CR_LOAD             (1u << 6)

struct GmRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_RTC_NREGS];
    int64_t offset;             /* seconds to add to the RTC clock */
};

static int64_t gm_rtc_now(GmRtcState *s)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND + s->offset;
}

static void gm_rtc_update(GmRtcState *s)
{
    qemu_set_irq(s->irq, s->regs[R_INTR_STATE / 4] != 0);
}

static void gm_rtc_arm(GmRtcState *s)
{
    const int64_t now = qemu_clock_get_ns(rtc_clock);

    timer_mod(s->timer, now - now % NANOSECONDS_PER_SECOND +
                        NANOSECONDS_PER_SECOND);
}

static void gm_rtc_tick(void *opaque)
{
    GmRtcState *s = opaque;
    const uint32_t cr = s->regs[R_CR / 4];
    const int64_t t = gm_rtc_now(s);

    if (cr & CR_EN) {
        if (cr & CR_SEC_INT) {
            s->regs[R_INTR_STATE / 4] |= 1;
        }
        if ((cr & (2u << 1)) && t % 60 == 0) {
            s->regs[R_INTR_STATE / 4] |= 2;
        }
        if ((cr & (4u << 1)) && t % 3600 == 0) {
            s->regs[R_INTR_STATE / 4] |= 4;
        }
        if ((cr & (8u << 1)) && t % 86400 == 0) {
            s->regs[R_INTR_STATE / 4] |= 8;
        }
        if ((cr & CR_ALARM_INT) &&
            t % 60 == s->regs[R_ALARM_SEC / 4] &&
            t / 60 % 60 == s->regs[R_ALARM_MIN / 4] &&
            t / 3600 % 24 == s->regs[R_ALARM_HOUR / 4]) {
            s->regs[R_INTR_STATE / 4] |= 0x10;
        }
        gm_rtc_update(s);
    }
    gm_rtc_arm(s);
}

static uint64_t gm_rtc_read(void *opaque, hwaddr off, unsigned size)
{
    GmRtcState *s = opaque;
    const int64_t t = gm_rtc_now(s);

    switch (off) {
    case R_SEC:
        return t % 60;
    case R_MIN:
        return t / 60 % 60;
    case R_HOUR:
        return t / 3600 % 24;
    case R_DAY:
        return t / 86400;
    case R_REVISION:
        return 0x00010100;
    case R_RWSTATUS:
        return 0;
    }
    return off < GM_RTC_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_rtc_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmRtcState *s = opaque;

    switch (off) {
    case R_SEC:
    case R_MIN:
    case R_HOUR:
    case R_DAY:
    case R_REVISION:
    case R_RWSTATUS:
        return;
    case R_INTR_STATE:
        s->regs[off / 4] &= ~val;
        gm_rtc_update(s);
        return;
    case R_CR:
        if (val & CR_LOAD) {
            const int64_t want = (int64_t)s->regs[R_WDAY / 4] * 86400 +
                                 s->regs[R_WHOUR / 4] * 3600 +
                                 s->regs[R_WMIN / 4] * 60 +
                                 s->regs[R_WSEC / 4];

            s->offset += want - gm_rtc_now(s);
        }
        s->regs[off / 4] = val & ~CR_LOAD;
        return;
    }
    if (off < GM_RTC_NREGS * 4) {
        s->regs[off / 4] = val;
    }
}

static const MemoryRegionOps gm_rtc_ops = {
    .read = gm_rtc_read,
    .write = gm_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_rtc_reset(DeviceState *dev)
{
    GmRtcState *s = GM_RTC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    gm_rtc_update(s);
}

static void gm_rtc_realize(DeviceState *dev, Error **errp)
{
    GmRtcState *s = GM_RTC(dev);
    struct tm tm;

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_rtc_ops, s, TYPE_GM_RTC,
                          GM_RTC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qemu_get_timedate(&tm, 0);
    s->offset = mktimegm(&tm) -
                qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
    s->timer = timer_new_ns(rtc_clock, gm_rtc_tick, s);
    gm_rtc_arm(s);
}

static const VMStateDescription vmstate_gm_rtc = {
    .name = TYPE_GM_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmRtcState, GM_RTC_NREGS),
        VMSTATE_INT64(offset, GmRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_rtc_realize;
    device_class_set_legacy_reset(dc, gm_rtc_reset);
    dc->vmsd = &vmstate_gm_rtc;
}

static const TypeInfo gm_rtc_info = {
    .name           = TYPE_GM_RTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmRtcState),
    .class_init     = gm_rtc_class_init,
};

static void gm_rtc_register_types(void)
{
    type_register_static(&gm_rtc_info);
}

type_init(gm_rtc_register_types)
