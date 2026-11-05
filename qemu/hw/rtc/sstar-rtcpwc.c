/*
 * SigmaStar Iford RTC and power controller (RTCPWC, PM bank 0x0034).
 *
 * From the vendor's drivers/sstar/rtc/iford/hal_rtcpwc.c.  The RTC domain
 * sits behind an isolation cell: software writes a command into the 0x00 and
 * 0x01 flags, its data into 0x04/0x05, and hands it over with the ISO
 * handshake (0x30 bit 0 trigger, 0x31 bit 0 ack, 0x32 bit 0 clear):
 *
 *   0x00  bit 1 base write, 2 base read, 3 counter reset, 4 alarm write,
 *         5/6 SW0/SW1 write, 7/8 SW0/SW1 read, 9/10 SW2/SW3 write
 *   0x01  bit 1 alarm read, 2 alarm enable, 3 clear the alarm interrupt
 *   0x08  bit 0 alarm enabled, 1 alarm interrupt   0x09/0x0A read data
 *   0x0B  bit 0 counter updating (never)
 *   0x0C/0x0D the counter, latched by 0x0E bit 0
 *   0x13  bit 1 the 32 kHz clock is there
 *   0x41/0x42 SW2, SW3
 *
 * Time is a base plus a counter of seconds since the last reset; the alarm
 * is relative to the base, so it matches the counter.  The domain keeps its
 * battery: its registers survive a machine reset, and the counter runs while
 * the chip is off.
 *
 * Power: the kernel turns the chip off by clearing 0x0F bit 0 (PWR_EN), then
 * touching the ISO or SW registers (sram_suspend_imi; hal_rtc_power_off).
 * Here that suspends the machine if an alarm is still to come: it brings the
 * chip back by waking the machine, a cold start of the CPUs that DRAM
 * survives, so the IPL finds the kernel's resume record.  0x0F bit 1 (alarm
 * power-on) is not consulted: hal_rtc_power_off sets it itself and
 * sram_suspend_imi does not, yet the alarm ends that suspend.  With nothing
 * to wake it the machine shuts down.  The kernel enables the alarm from init,
 * so a power off usually finds one enabled that matched long ago; it matches
 * the counter, so that one never fires.  The PM reset flag in bank 0x0600
 * (reg 1 bit 3) that the kernel sets is the RIU's to keep.
 *
 * infinity2m has the same bank and flags, from its ms_rtcpwc.c, with another
 * handshake (iso-fsm): software steps 0x03 through 000, 001, 011, 111, 101,
 * 001, 000 and 0x08 bit 3 acks each step, reading as the state's parity.
 * The command goes across when the sequence ends.  0x15 bit 0 is ISO_EN and
 * bit 5 the 1 kHz clock; 0x01 bit 0 asks for the counter, which needs no
 * more here.  Its driver has no alarm, 0x08 bit 3 sits where Iford keeps
 * nothing, and what 0x0F and the ISO registers at 0x30 do there is not known,
 * so that mode leaves the alarm and power off out.  The driver reads a base
 * only if SW0 holds its magic, 0xBABE; a board that kept its battery has it,
 * and the base is the machine's RTC time, as sstar-rtc's counter is.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/cutils.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "system/system.h"
#include "system/runstate.h"

#define TYPE_SSTAR_RTCPWC "sstar-rtcpwc"
OBJECT_DECLARE_SIMPLE_TYPE(SstarRtcpwcState, SSTAR_RTCPWC)

#define R_CTRL0             0x00
#define  C0_BASE_WR         (1u << 1)
#define  C0_BASE_RD         (1u << 2)
#define  C0_CNT_RST         (1u << 3)
#define  C0_ALARM_WR        (1u << 4)
#define  C0_SW0_WR          (1u << 5)
#define  C0_SW1_WR          (1u << 6)
#define  C0_SW0_RD          (1u << 7)
#define  C0_SW1_RD          (1u << 8)
#define  C0_SW2_WR          (1u << 9)
#define  C0_SW3_WR          (1u << 10)
#define R_CTRL1             0x01
#define  C1_ALARM_RD        (1u << 1)
#define  C1_ALARM_EN        (1u << 2)
#define  C1_INT_CLR         (1u << 3)
#define R_ISO_CTRL          0x03    /* iso-fsm */
#define  ISO_STATES         7u
#define R_WRDATA_L          0x04
#define R_WRDATA_H          0x05
#define R_ALARM_INT         0x08
#define  ISO_FSM_ACK        (1u << 3)
#define  AI_ENABLED         (1u << 0)
#define  AI_INT             (1u << 1)
#define R_RDDATA_L          0x09
#define R_RDDATA_H          0x0A
#define R_CNT_L             0x0C
#define R_CNT_H             0x0D
#define R_CNT_TRIG          0x0E
#define R_PWR_EN            0x0F
#define  PWR_EN             (1u << 0)
#define R_STATE2            0x13
#define R_TESTBUS           0x15    /* iso-fsm */
#define  TESTBUS_ISO_EN     (1u << 0)
#define  TESTBUS_CLK_1K     (1u << 5)
#define  STATE2_32K_OK      (1u << 1)
#define R_ISO_TRIG          0x30
#define R_ISO_ACK           0x31
#define R_ISO_CLR           0x32
#define R_SW2               0x41
#define R_SW3               0x42
#define R_SW_CTRL           0x49
#define RTCPWC_REGS         0x80
#define RTC_PASSWORD        0xBABE  /* infinity2m's driver, in SW0 */

struct SstarRtcpwcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *alarm_timer;
    uint16_t regs[RTCPWC_REGS];
    uint32_t base, alarm, sw[4];
    int64_t epoch;                  /* rtc_clock seconds at counter zero */
    uint32_t saved_count;           /* the counter, across a migration */
    bool off_pending, sleeping;
    bool iso_fsm;                   /* the property: infinity2m */
    int8_t iso_step;                /* in iso_seq, -1 out of sequence */
};

static int64_t rtcpwc_now(void)
{
    return qemu_clock_get_ms(rtc_clock) / 1000;
}

static uint32_t rtcpwc_count(SstarRtcpwcState *s)
{
    return rtcpwc_now() - s->epoch;
}

static void rtcpwc_arm(SstarRtcpwcState *s)
{
    uint32_t now = rtcpwc_count(s);

    timer_del(s->alarm_timer);
    if (!(s->regs[R_ALARM_INT] & AI_ENABLED) ||
        (s->regs[R_ALARM_INT] & AI_INT) || s->alarm < now) {
        return;
    }
    timer_mod(s->alarm_timer, qemu_clock_get_ms(rtc_clock) +
              (s->alarm > now ? (int64_t)(s->alarm - now) * 1000 : 1));
}

static void rtcpwc_alarm(void *opaque)
{
    SstarRtcpwcState *s = opaque;

    /* the power off is on its way: only a suspended machine can wake */
    if (s->sleeping && !runstate_check(RUN_STATE_SUSPENDED)) {
        timer_mod(s->alarm_timer, qemu_clock_get_ms(rtc_clock) + 1);
        return;
    }
    s->regs[R_ALARM_INT] |= AI_INT;
    qemu_irq_raise(s->irq);
    if (s->sleeping) {
        s->sleeping = false;
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_RTC, &error_abort);
    }
}

static void rtcpwc_power_off(SstarRtcpwcState *s)
{
    s->off_pending = false;
    if ((s->regs[R_ALARM_INT] & AI_ENABLED) &&
        !(s->regs[R_ALARM_INT] & AI_INT) && s->alarm >= rtcpwc_count(s)) {
        s->sleeping = true;
        qemu_system_wakeup_enable(QEMU_WAKEUP_REASON_RTC, true);
        qemu_system_suspend_request();
    } else {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

/* the ISO handshake hands the flags to the RTC domain */
static void rtcpwc_iso(SstarRtcpwcState *s)
{
    uint16_t c0 = s->regs[R_CTRL0], c1 = s->regs[R_CTRL1];
    uint32_t data = s->regs[R_WRDATA_L] |
                    (uint32_t)s->regs[R_WRDATA_H] << 16;
    uint32_t rd = 0;

    if (c0 & C0_BASE_WR) {
        s->base = data;
    }
    if (c0 & C0_CNT_RST) {
        s->epoch = rtcpwc_now();
    }
    if (c0 & C0_ALARM_WR) {
        s->alarm = data;
    }
    if (c0 & C0_SW0_WR) {
        s->sw[0] = data;
    }
    if (c0 & C0_SW1_WR) {
        s->sw[1] = data;
    }
    if (c0 & C0_SW2_WR) {
        s->sw[2] = data;
    }
    if (c0 & C0_SW3_WR) {
        s->sw[3] = data;
    }
    if (c0 & C0_BASE_RD) {
        rd = s->base;
    } else if (c0 & C0_SW0_RD) {
        rd = s->sw[0];
    } else if (c0 & C0_SW1_RD) {
        rd = s->sw[1];
    } else if (c1 & C1_ALARM_RD) {
        rd = s->alarm;
    }
    s->regs[R_RDDATA_L] = rd;
    s->regs[R_RDDATA_H] = rd >> 16;
    if (c1 & C1_ALARM_EN) {
        s->regs[R_ALARM_INT] |= AI_ENABLED;
    } else {
        s->regs[R_ALARM_INT] &= ~AI_ENABLED;
    }
    if (c1 & C1_INT_CLR) {
        s->regs[R_ALARM_INT] &= ~AI_INT;
        qemu_irq_lower(s->irq);
    }
    rtcpwc_arm(s);
    s->regs[R_ISO_ACK] = 1;
}

static uint64_t rtcpwc_read(void *opaque, hwaddr off, unsigned size)
{
    SstarRtcpwcState *s = opaque;
    int reg = (off >> 2) & (RTCPWC_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    if (s->iso_fsm && reg == R_ALARM_INT) {
        v = ctpop8(s->regs[R_ISO_CTRL] & ISO_STATES) & 1 ? ISO_FSM_ACK : 0;
        return off & 1 ? v >> 8 : (size == 1 ? (v & 0xFF) : v);
    }
    if (s->iso_fsm && reg == R_TESTBUS) {
        return off & 1 ? 0 : TESTBUS_ISO_EN | TESTBUS_CLK_1K;
    }
    switch (reg) {
    case R_STATE2:
        v = STATE2_32K_OK;
        break;
    case R_SW2:
        v = s->sw[2];
        break;
    case R_SW3:
        v = s->sw[3];
        break;
    default:
        v = s->regs[reg];
        break;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

/*
 * The sequence again from 000 is how the driver retries, so that always
 * starts over; anything else out of order sends nothing.
 */
static void rtcpwc_fsm_step(SstarRtcpwcState *s, uint8_t state)
{
    static const uint8_t iso_seq[] = { 0, 1, 3, 7, 5, 1, 0 };

    if (state == iso_seq[s->iso_step + 1]) {
        s->iso_step++;
    } else {
        s->iso_step = state ? -1 : 0;
    }
    if (s->iso_step == ARRAY_SIZE(iso_seq) - 1) {
        s->iso_step = 0;
        rtcpwc_iso(s);
    }
}

/* infinity2m: the command goes across as the state sequence ends */
static void rtcpwc_fsm_write(SstarRtcpwcState *s, int reg, uint16_t v)
{
    uint16_t old = s->regs[reg];

    switch (reg) {
    case R_ISO_CTRL:
        s->regs[reg] = v;
        if ((v ^ old) & ISO_STATES) {
            rtcpwc_fsm_step(s, v & ISO_STATES);
        }
        break;
    case R_CNT_TRIG:
        if (v & 1) {
            uint32_t c = rtcpwc_count(s);

            s->regs[R_CNT_L] = c;
            s->regs[R_CNT_H] = c >> 16;
        }
        break;
    case R_ALARM_INT:
    case R_RDDATA_L:
    case R_RDDATA_H:
        break;
    default:
        s->regs[reg] = v;
        break;
    }
}

static void rtcpwc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarRtcpwcState *s = opaque;
    int reg = (off >> 2) & (RTCPWC_REGS - 1);
    uint16_t v = s->regs[reg];

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = (off & 1) ? (v & 0x00FF) | ((val & 0xFF) << 8)
                      : (v & 0xFF00) | (val & 0xFF);
    } else {
        v = val;
    }
    if (s->iso_fsm) {
        rtcpwc_fsm_write(s, reg, v);
        return;
    }
    switch (reg) {
    case R_ISO_TRIG:
        if (v & 1) {
            rtcpwc_iso(s);
        }
        return;
    case R_ISO_CLR:
        if (v & 1) {
            s->regs[R_ISO_ACK] = 0;
        }
        if (s->off_pending) {
            rtcpwc_power_off(s);
        }
        return;
    case R_ISO_ACK:
    case R_ALARM_INT:
    case R_RDDATA_L:
    case R_RDDATA_H:
        return;
    case R_CNT_TRIG:
        if (v & 1) {
            uint32_t c = rtcpwc_count(s);

            s->regs[R_CNT_L] = c;
            s->regs[R_CNT_H] = c >> 16;
        }
        return;
    case R_PWR_EN:
        if ((s->regs[reg] & PWR_EN) && !(v & PWR_EN)) {
            s->off_pending = true;
        }
        break;
    case R_SW_CTRL:
        if (s->off_pending) {
            rtcpwc_power_off(s);
        }
        break;
    default:
        break;
    }
    s->regs[reg] = v;
}

static const MemoryRegionOps rtcpwc_ops = {
    .read = rtcpwc_read,
    .write = rtcpwc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void rtcpwc_reset(DeviceState *dev)
{
    SstarRtcpwcState *s = SSTAR_RTCPWC(dev);

    /* the RTC domain has its battery: only the power request goes */
    s->regs[R_PWR_EN] |= PWR_EN;
    s->regs[R_ISO_ACK] = 0;
    if (s->iso_fsm) {
        s->regs[R_ISO_CTRL] = 0;
    }
    s->iso_step = 0;
    s->off_pending = false;
    s->sleeping = false;
    if (!(s->regs[R_ALARM_INT] & AI_INT)) {
        qemu_irq_lower(s->irq);
    }
}

static void rtcpwc_realize(DeviceState *dev, Error **errp)
{
    SstarRtcpwcState *s = SSTAR_RTCPWC(dev);

    s->alarm_timer = timer_new_ms(rtc_clock, rtcpwc_alarm, s);
    s->epoch = rtcpwc_now();
    if (s->iso_fsm) {
        struct tm tm;

        qemu_get_timedate(&tm, 0);
        s->base = mktimegm(&tm);
        s->sw[0] = RTC_PASSWORD;
    } else {
        qemu_register_wakeup_support();
    }
}

static int rtcpwc_pre_save(void *opaque)
{
    SstarRtcpwcState *s = opaque;

    /* The counter, not a reading of this host's rtc_clock */
    s->saved_count = rtcpwc_count(s);
    return 0;
}

static int rtcpwc_post_load(void *opaque, int version_id)
{
    SstarRtcpwcState *s = opaque;

    s->epoch = rtcpwc_now() - s->saved_count;
    rtcpwc_arm(s);
    return 0;
}

static const VMStateDescription vmstate_rtcpwc = {
    .name = TYPE_SSTAR_RTCPWC,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = rtcpwc_pre_save,
    .post_load = rtcpwc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarRtcpwcState, RTCPWC_REGS),
        VMSTATE_UINT32(base, SstarRtcpwcState),
        VMSTATE_UINT32(alarm, SstarRtcpwcState),
        VMSTATE_UINT32_ARRAY(sw, SstarRtcpwcState, 4),
        VMSTATE_UINT32(saved_count, SstarRtcpwcState),
        VMSTATE_BOOL(off_pending, SstarRtcpwcState),
        VMSTATE_BOOL(sleeping, SstarRtcpwcState),
        VMSTATE_INT8(iso_step, SstarRtcpwcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property rtcpwc_props[] = {
    DEFINE_PROP_BOOL("iso-fsm", SstarRtcpwcState, iso_fsm, false),
};

static void rtcpwc_init(Object *obj)
{
    SstarRtcpwcState *s = SSTAR_RTCPWC(obj);

    memory_region_init_io(&s->iomem, obj, &rtcpwc_ops, s, TYPE_SSTAR_RTCPWC,
                          RTCPWC_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void rtcpwc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rtcpwc_realize;
    dc->vmsd = &vmstate_rtcpwc;
    device_class_set_props(dc, rtcpwc_props);
    device_class_set_legacy_reset(dc, rtcpwc_reset);
}

static const TypeInfo rtcpwc_info = {
    .name = TYPE_SSTAR_RTCPWC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarRtcpwcState),
    .instance_init = rtcpwc_init,
    .class_init = rtcpwc_class_init,
};

static void rtcpwc_register(void)
{
    type_register_static(&rtcpwc_info);
}

type_init(rtcpwc_register)
