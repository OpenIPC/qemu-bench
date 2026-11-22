/*
 * NXP PCF8563 / Belling BM8563 I2C real-time clock (7-bit address 0x51),
 * as on the RV1109 camera board's I2C0 ("bm8563_dev_init: write error"
 * without it).  QOM type "pcf8563".
 *
 *   0x00 CONTROL_1   0x01 CONTROL_2 (bit 3 AF alarm flag, bit 1 AIE)
 *   0x02 VL_SECONDS  bit 7 VL: clock integrity lost (cleared by a write)
 *   0x03 MINUTES     0x04 HOURS     0x05 DAYS     0x06 WEEKDAYS
 *   0x07 CENTURY_MONTHS  bit 7 century (set: 19xx)
 *   0x08 YEARS       BCD 00..99 (20xx)
 *   0x09..0x0c       minute / hour / day / weekday alarm, bit 7 disables
 *   0x0d CLKOUT      0x0e TIMER_CONTROL   0x0f TIMER
 *
 * Registers auto-increment, wrapping after 0x0f.  The calendar runs off
 * the host clock (-rtc base=...); writing any time register sets it.
 * The alarm sets AF on the minute it matches (checked when read) and
 * drives the "int" GPIO low while AIE is on.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bcd.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "system/rtc.h"
#include "qom/object.h"

#define TYPE_PCF8563 "pcf8563"
OBJECT_DECLARE_SIMPLE_TYPE(Pcf8563State, PCF8563)

#define NREGS       16
#define R_CTRL2     0x01
#define R_SECONDS   0x02
#define R_YEARS     0x08
#define R_ALARM_MIN 0x09
#define R_ALARM_WD  0x0c

#define CTRL2_AIE   (1u << 1)
#define CTRL2_AF    (1u << 3)

struct Pcf8563State {
    I2CSlave parent_obj;
    qemu_irq irq;

    uint8_t regs[NREGS];
    int64_t offset;         /* calendar - host, seconds */
    uint8_t ptr;
    bool addr_phase;
    bool time_written;      /* a time register changed in this transfer */
};

static void pcf8563_now(Pcf8563State *s, struct tm *tm)
{
    qemu_get_timedate(tm, s->offset);
}

static void pcf8563_check_alarm(Pcf8563State *s, const struct tm *tm)
{
    const uint8_t *a = &s->regs[R_ALARM_MIN];
    /* AE (bit 7) set disables a field: disabled, or matching, it holds */
    bool hit = (a[0] & 0x80) || from_bcd(a[0] & 0x7f) == tm->tm_min;

    hit &= (a[1] & 0x80) || from_bcd(a[1] & 0x3f) == tm->tm_hour;
    hit &= (a[2] & 0x80) || from_bcd(a[2] & 0x3f) == tm->tm_mday;
    hit &= (a[3] & 0x80) || (a[3] & 7) == tm->tm_wday;
    if (hit && (a[0] & a[1] & a[2] & a[3] & 0x80) == 0) {
        s->regs[R_CTRL2] |= CTRL2_AF;
    }
    qemu_set_irq(s->irq, !((s->regs[R_CTRL2] & CTRL2_AF) &&
                           (s->regs[R_CTRL2] & CTRL2_AIE)));
}

/* The time registers from the running calendar. */
static void pcf8563_latch(Pcf8563State *s)
{
    struct tm tm;
    uint8_t *r = &s->regs[R_SECONDS];
    uint8_t vl = r[0] & 0x80;

    pcf8563_now(s, &tm);
    r[0] = vl | to_bcd(tm.tm_sec);
    r[1] = to_bcd(tm.tm_min);
    r[2] = to_bcd(tm.tm_hour);
    r[3] = to_bcd(tm.tm_mday);
    r[4] = tm.tm_wday;
    r[5] = (tm.tm_year < 100 ? 0x80 : 0) | to_bcd(tm.tm_mon + 1);
    r[6] = to_bcd(tm.tm_year % 100);
    pcf8563_check_alarm(s, &tm);
}

/* Set the calendar from the time registers. */
static void pcf8563_set(Pcf8563State *s)
{
    const uint8_t *r = &s->regs[R_SECONDS];
    struct tm tm = {
        .tm_sec = from_bcd(r[0] & 0x7f),
        .tm_min = from_bcd(r[1] & 0x7f),
        .tm_hour = from_bcd(r[2] & 0x3f),
        .tm_mday = from_bcd(r[3] & 0x3f),
        .tm_wday = r[4] & 7,
        .tm_mon = from_bcd(r[5] & 0x1f) - 1,
        .tm_year = from_bcd(r[6]) + ((r[5] & 0x80) ? 0 : 100),
    };

    s->offset = qemu_timedate_diff(&tm);
}

static int pcf8563_event(I2CSlave *i2c, enum i2c_event event)
{
    Pcf8563State *s = PCF8563(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->addr_phase = true;
        s->time_written = false;
        break;
    case I2C_START_RECV:
        pcf8563_latch(s);
        break;
    case I2C_FINISH:
        if (s->time_written) {
            pcf8563_set(s);
            s->time_written = false;
        }
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t pcf8563_recv(I2CSlave *i2c)
{
    Pcf8563State *s = PCF8563(i2c);
    uint8_t v = s->regs[s->ptr];

    s->ptr = (s->ptr + 1) % NREGS;
    return v;
}

static int pcf8563_send(I2CSlave *i2c, uint8_t data)
{
    Pcf8563State *s = PCF8563(i2c);

    if (s->addr_phase) {
        s->ptr = data % NREGS;
        s->addr_phase = false;
        return 0;
    }
    if (s->ptr >= R_SECONDS && s->ptr <= R_YEARS) {
        if (!s->time_written) {
            pcf8563_latch(s);       /* the registers not written keep time */
        }
        s->time_written = true;
        if (s->ptr == R_SECONDS) {
            data &= 0x7f;           /* writing the seconds clears VL */
        }
    }
    s->regs[s->ptr] = data;
    if (s->ptr == R_CTRL2) {
        struct tm tm;

        pcf8563_now(s, &tm);
        pcf8563_check_alarm(s, &tm);
    }
    s->ptr = (s->ptr + 1) % NREGS;
    return 0;
}

static void pcf8563_reset(DeviceState *dev)
{
    Pcf8563State *s = PCF8563(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_ALARM_MIN] = s->regs[R_ALARM_MIN + 1] = 0x80;
    s->regs[R_ALARM_MIN + 2] = s->regs[R_ALARM_WD] = 0x80;
    s->ptr = 0;
    s->addr_phase = false;
    s->time_written = false;
}

static void pcf8563_init(Object *obj)
{
    Pcf8563State *s = PCF8563(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "int", 1);
}

static const VMStateDescription pcf8563_vmstate = {
    .name = TYPE_PCF8563,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Pcf8563State),
        VMSTATE_UINT8_ARRAY(regs, Pcf8563State, NREGS),
        VMSTATE_INT64(offset, Pcf8563State),
        VMSTATE_UINT8(ptr, Pcf8563State),
        VMSTATE_BOOL(addr_phase, Pcf8563State),
        VMSTATE_BOOL(time_written, Pcf8563State),
        VMSTATE_END_OF_LIST()
    }
};

static void pcf8563_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = pcf8563_event;
    k->recv = pcf8563_recv;
    k->send = pcf8563_send;
    device_class_set_legacy_reset(dc, pcf8563_reset);
    dc->vmsd = &pcf8563_vmstate;
}

static const TypeInfo pcf8563_info = {
    .name = TYPE_PCF8563,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Pcf8563State),
    .instance_init = pcf8563_init,
    .class_init = pcf8563_class_init,
};

static void pcf8563_register_types(void)
{
    type_register_static(&pcf8563_info);
}

type_init(pcf8563_register_types)
