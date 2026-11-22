/*
 * Rockchip RV1106 RTC ("rockchip,rv1106-rtc", 0xFF1C0000): the RK808-style
 * BCD calendar with its 32 kHz oscillator trim test.
 *
 * Register file, from the vendor kernel's drivers/rtc/rtc-rockchip.c:
 *
 *   0x00..0x1c  seconds, minutes, hours, days, months, year low, year
 *               high, weekday (BCD; year = high * 100 + low + 2000)
 *   0x20..0x38  alarm, same layout without the weekday
 *   0x3c CTRL   bit 0 START, bit 2 COMP_EN, bit 6 GET_TIME, bit 7 READSEL
 *   0x40 STATUS0  bit 4 alarm (write 1 to clear)
 *   0x44 STATUS1  bit 0: the calendar is running (follows CTRL.START)
 *   0x48 INT0_EN  bit 7 alarm
 *   0x58..0x60  hour / day / month compensation
 *   0x64..0x80  analog: LDO, XO trim, VPTAT, enables, CLK32K test
 *   0x84 TEST_ST  bit 0 start, bit 2 done
 *   0x88 TEST_LEN
 *   0x8c..0x98  CNT_0..3: 24 MHz cycles over (TEST_LEN + 1) 32k seconds
 *
 * Only the low byte of a write counts: the driver ORs 0xc4522900 into
 * every value, a key the silicon checks.  The calendar runs off the
 * host's clock (-rtc base=...).  Writes to the time registers while the
 * calendar is stopped set it; START takes the new time.  STATUS1 follows
 * START at once (the driver polls it, and gives up after 3 seconds
 * without it), and the trim test completes at once with an exact
 * 32768 Hz count, so the driver's compensation comes out as none.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bcd.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_RK_RTC "rk-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(RkRtcState, RK_RTC)

#define R_SECONDS       0x00
#define R_WEEKS         0x1c
#define R_ALARM_SECONDS 0x20
#define R_ALARM_YEARH   0x38
#define R_CTRL          0x3c
#define R_STATUS0       0x40
#define R_STATUS1       0x44
#define R_INT0_EN       0x48
#define R_TEST_ST       0x84
#define R_TEST_LEN      0x88
#define R_CNT_0         0x8c
#define R_CNT_3         0x98
#define REGS_SIZE       0x1000
#define NREGS           (0x100 / 4)

#define CTRL_START      (1u << 0)
#define ST0_ALARM       (1u << 4)
#define INT_ALARM       (1u << 7)
#define TEST_START      (1u << 0)
#define TEST_DONE       (1u << 2)

#define REF_CLK         24000000u

struct RkRtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *alarm_timer;

    uint32_t regs[NREGS];
    int64_t offset;             /* calendar - host, seconds */
};

static void rk_rtc_now(RkRtcState *s, struct tm *tm)
{
    qemu_get_timedate(tm, s->offset);
}

/* The calendar from the time registers as written. */
static void rk_rtc_regs_tm(RkRtcState *s, struct tm *tm, bool alarm)
{
    const uint32_t *r = &s->regs[(alarm ? R_ALARM_SECONDS : R_SECONDS) / 4];

    memset(tm, 0, sizeof(*tm));
    tm->tm_sec = from_bcd(r[0] & 0x7f);
    tm->tm_min = from_bcd(r[1] & 0x7f);
    tm->tm_hour = from_bcd(r[2] & 0x3f);
    tm->tm_mday = from_bcd(r[3] & 0x3f);
    tm->tm_mon = from_bcd(r[4] & 0x1f) - 1;
    tm->tm_year = from_bcd(r[6] & 0xff) * 100 + from_bcd(r[5] & 0xff) + 100;
    if (!alarm) {
        tm->tm_wday = from_bcd(r[7] & 0x7);
    }
}

static void rk_rtc_tm_regs(RkRtcState *s, const struct tm *tm)
{
    uint32_t *r = &s->regs[R_SECONDS / 4];
    int y = tm->tm_year - 100;

    r[0] = to_bcd(tm->tm_sec);
    r[1] = to_bcd(tm->tm_min);
    r[2] = to_bcd(tm->tm_hour);
    r[3] = to_bcd(tm->tm_mday);
    r[4] = to_bcd(tm->tm_mon + 1);
    r[5] = to_bcd(y % 100);
    r[6] = to_bcd(y / 100);
    r[7] = to_bcd(tm->tm_wday);
}

static void rk_rtc_update_irq(RkRtcState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_STATUS0 / 4] & ST0_ALARM) &&
                         (s->regs[R_INT0_EN / 4] & INT_ALARM));
}

/* Arm the timer for the alarm's second, if the alarm is on. */
static void rk_rtc_arm_alarm(RkRtcState *s)
{
    struct tm tm;
    int64_t delta;

    if (!(s->regs[R_INT0_EN / 4] & INT_ALARM) ||
        !(s->regs[R_CTRL / 4] & CTRL_START)) {
        timer_del(s->alarm_timer);
        return;
    }
    rk_rtc_regs_tm(s, &tm, true);
    delta = qemu_timedate_diff(&tm) - s->offset;
    if (delta < 0) {
        delta = 0;
    }
    timer_mod(s->alarm_timer, qemu_clock_get_ms(rtc_clock) + delta * 1000);
}

static void rk_rtc_alarm_cb(void *opaque)
{
    RkRtcState *s = opaque;

    s->regs[R_STATUS0 / 4] |= ST0_ALARM;
    rk_rtc_update_irq(s);
}

static uint64_t rk_rtc_read(void *opaque, hwaddr off, unsigned size)
{
    RkRtcState *s = opaque;

    if (off >= NREGS * 4) {
        return 0;
    }
    switch (off) {
    case R_SECONDS ... R_WEEKS:
        if (s->regs[R_CTRL / 4] & CTRL_START) {
            struct tm tm;

            rk_rtc_now(s, &tm);
            rk_rtc_tm_regs(s, &tm);
        }
        break;
    case R_STATUS1:
        return s->regs[R_CTRL / 4] & CTRL_START;
    case R_CNT_0 ... R_CNT_3: {
        uint64_t n = (uint64_t)REF_CLK * ((s->regs[R_TEST_LEN / 4] & 0xff) + 1);

        return (n >> ((off - R_CNT_0) * 2)) & 0xff;
    }
    }
    return s->regs[off / 4];
}

static void rk_rtc_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkRtcState *s = opaque;
    uint32_t v = val & 0xff;

    if (off >= NREGS * 4) {
        return;
    }
    switch (off) {
    case R_SECONDS ... R_WEEKS:
        if (s->regs[R_CTRL / 4] & CTRL_START) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "rk-rtc: time written while running\n");
        }
        s->regs[off / 4] = v;
        return;
    case R_CTRL: {
        bool was = s->regs[R_CTRL / 4] & CTRL_START;
        struct tm tm;

        if (was && !(v & CTRL_START)) {
            /* stop: freeze the registers at the current time */
            rk_rtc_now(s, &tm);
            rk_rtc_tm_regs(s, &tm);
        } else if (!was && (v & CTRL_START)) {
            rk_rtc_regs_tm(s, &tm, false);
            s->offset = qemu_timedate_diff(&tm);
        }
        s->regs[R_CTRL / 4] = v;
        rk_rtc_arm_alarm(s);
        return;
    }
    case R_STATUS0:
        s->regs[R_STATUS0 / 4] &= ~v;
        rk_rtc_update_irq(s);
        return;
    case R_INT0_EN:
        s->regs[R_INT0_EN / 4] = v;
        rk_rtc_arm_alarm(s);
        rk_rtc_update_irq(s);
        return;
    case R_ALARM_SECONDS ... R_ALARM_YEARH:
        s->regs[off / 4] = v;
        rk_rtc_arm_alarm(s);
        return;
    case R_TEST_ST:
        if (v & TEST_START) {
            v = (v & ~TEST_START) | TEST_DONE;
        }
        s->regs[R_TEST_ST / 4] = v;
        return;
    case R_CNT_0 ... R_CNT_3:
        return;
    }
    s->regs[off / 4] = v;
}

static const MemoryRegionOps rk_rtc_ops = {
    .read = rk_rtc_read,
    .write = rk_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_rtc_reset(DeviceState *dev)
{
    RkRtcState *s = RK_RTC(dev);
    struct tm tm;

    /*
     * The calendar keeps running across a SoC reset on the backup
     * supply: it comes up started, at the host's time.
     */
    memset(s->regs, 0, sizeof(s->regs));
    s->offset = 0;
    rk_rtc_now(s, &tm);
    rk_rtc_tm_regs(s, &tm);
    s->regs[R_CTRL / 4] = CTRL_START;
    timer_del(s->alarm_timer);
}

static void rk_rtc_init(Object *obj)
{
    RkRtcState *s = RK_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &rk_rtc_ops, s, TYPE_RK_RTC,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->alarm_timer = timer_new_ms(rtc_clock, rk_rtc_alarm_cb, s);
}

static const VMStateDescription rk_rtc_vmstate = {
    .name = TYPE_RK_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkRtcState, NREGS),
        VMSTATE_INT64(offset, RkRtcState),
        VMSTATE_TIMER_PTR(alarm_timer, RkRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_rtc_reset);
    dc->vmsd = &rk_rtc_vmstate;
}

static const TypeInfo rk_rtc_info = {
    .name = TYPE_RK_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkRtcState),
    .instance_init = rk_rtc_init,
    .class_init = rk_rtc_class_init,
};

static void rk_rtc_register_types(void)
{
    type_register_static(&rk_rtc_info);
}

type_init(rk_rtc_register_types)
