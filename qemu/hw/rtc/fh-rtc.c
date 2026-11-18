/*
 * Fullhan RTC (FH885xV200 / FH865x, the kernel's drivers/rtc/rtc-fh_v2.c).
 *
 * An APB wrapper in front of an RTC core on the always-on domain, reached
 * one command at a time:
 *
 *   0x00 INT_STATUS  bit 5 CORE_IDLE: the core has finished the last
 *                    command; the driver writes it 0 before each command
 *                    and polls it back to 1
 *   0x04 INT_EN      0x08-0x10 DEBUG0-2
 *   0x14 CMD         bits 3:0 1 read, 2 write, 3 measure the temperature;
 *                    bits 8:4 the core register
 *   0x18 RD_DATA     0x1C WR_DATA
 *
 * Core registers: 0 the counter, 1 OFFSET, 2 ALARM_CFG, 3 TEMP_INFO,
 * 4 TEMP_CFG, 5 ANA_CFG, 6 INT_STATUS, 7 INT_EN, 8 DEBUG, and from 9 the
 * twelve words of the temperature-offset LUT (rtc_adjust() loads them
 * at OFFSET_LUT + i).
 * The counter and the alarm pack the time as days since 1970 in 31:17,
 * hours 16:12, minutes 11:6 and seconds 5:0.  Commands finish at once;
 * the counter runs from the host clock (-rtc base=), and an alarm is kept
 * but never fires.
 *
 * "v1" is the older block (FH8632, the kernel's drivers/rtc/rtc-fh.c),
 * its registers direct: COUNTER 0x00 and ALARM 0x0C in the same packing,
 * OFFSET 0x04, POWER_FAIL 0x08, INT_STAT 0x10, INT_EN 0x14, SYNC 0x18
 * (bit 0 a read, bit 1 a write has reached the core -- at once here),
 * DEBUG 0x1C, and USER_REG 0x20, where the driver keeps 0x55555555 to
 * say the RTC was set up: a battery-backed RTC that has been, here.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "system/system.h"

#define TYPE_FH_RTC "fh-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(FhRtcState, FH_RTC)

#define FH_RTC_SIZE         0x1000

#define R_INT_STATUS        0x00
#define R_INT_EN            0x04
#define R_DEBUG0            0x08
#define R_CMD               0x14
#define R_RD_DATA           0x18
#define R_WR_DATA           0x1C

#define ST_CORE_IDLE        (1u << 5)

#define CMD_READ            1
#define CMD_WRITE           2
#define CMD_TEMP            3

#define CORE_COUNTER        0
#define CORE_ALARM          2
#define CORE_TEMP_INFO      3
#define CORE_REGS           32

/* What the temperature sensor reads at about 25 C (OUT_TEMP(y) * ...). */
#define TEMP_RAW            0x993

struct FhRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t int_status;
    uint32_t int_en;
    uint32_t debug[3];
    uint32_t cmd;
    uint32_t rd_data;
    uint32_t wr_data;
    uint32_t core[CORE_REGS];
    int64_t offset;             /* RTC seconds - host seconds */
    uint32_t v1_regs[9];
    bool v1;
    bool no_core;               /* a board without the 32.768 kHz crystal */
};

#define V1_COUNTER          0x00
#define V1_SYNC             0x18
#define V1_USER_REG         0x20
#define V1_USER_MAGIC       0x55555555

static uint32_t fh_rtc_pack(int64_t secs)
{
    uint32_t days = secs / 86400, rem = secs % 86400;

    return (days << 17) | ((rem / 3600) << 12) | (((rem / 60) % 60) << 6) |
           (rem % 60);
}

static int64_t fh_rtc_unpack(uint32_t v)
{
    return (int64_t)(v >> 17) * 86400 + ((v >> 12) & 0x1F) * 3600 +
           ((v >> 6) & 0x3F) * 60 + (v & 0x3F);
}

static int64_t fh_rtc_host_secs(void)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
}

static void fh_rtc_command(FhRtcState *s, uint32_t cmd)
{
    unsigned reg = (cmd >> 4) & (CORE_REGS - 1);

    /*
     * The core lives in its own domain, clocked by an external 32.768 kHz
     * crystal; boards built without it (most cheap cameras) never answer,
     * so CORE_IDLE, once the driver clears it for a command, stays clear.
     */
    if (s->no_core) {
        return;
    }

    switch (cmd & 0xF) {
    case CMD_READ:
        if (reg == CORE_COUNTER) {
            s->rd_data = fh_rtc_pack(fh_rtc_host_secs() + s->offset);
        } else {
            s->rd_data = s->core[reg];
        }
        break;
    case CMD_WRITE:
        if (reg == CORE_COUNTER) {
            s->offset = fh_rtc_unpack(s->wr_data) - fh_rtc_host_secs();
        } else {
            s->core[reg] = s->wr_data;
        }
        break;
    case CMD_TEMP:
        s->core[CORE_TEMP_INFO] = TEMP_RAW;
        s->rd_data = TEMP_RAW;
        break;
    }
    s->int_status |= ST_CORE_IDLE;
}

static uint64_t fh_rtc_v1_read(FhRtcState *s, hwaddr off)
{
    switch (off) {
    case V1_COUNTER:
        return fh_rtc_pack(fh_rtc_host_secs() + s->offset);
    case V1_SYNC:
        return 3;
    default:
        return off <= V1_USER_REG ? s->v1_regs[off >> 2] : 0;
    }
}

static void fh_rtc_v1_write(FhRtcState *s, hwaddr off, uint32_t val)
{
    switch (off) {
    case V1_COUNTER:
        s->offset = fh_rtc_unpack(val) - fh_rtc_host_secs();
        break;
    case V1_SYNC:
        break;
    default:
        if (off <= V1_USER_REG) {
            s->v1_regs[off >> 2] = val;
        }
        break;
    }
}

static uint64_t fh_rtc_read(void *opaque, hwaddr off, unsigned size)
{
    FhRtcState *s = opaque;

    if (s->v1) {
        return fh_rtc_v1_read(s, off);
    }

    switch (off) {
    case R_INT_STATUS:      return s->int_status;
    case R_INT_EN:          return s->int_en;
    case R_DEBUG0 ... R_DEBUG0 + 8:
        return s->debug[(off - R_DEBUG0) >> 2];
    case R_CMD:             return s->cmd;
    case R_RD_DATA:         return s->rd_data;
    case R_WR_DATA:         return s->wr_data;
    default:                return 0;
    }
}

static void fh_rtc_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhRtcState *s = opaque;

    if (s->v1) {
        fh_rtc_v1_write(s, off, val);
        return;
    }
    switch (off) {
    case R_INT_STATUS:
        s->int_status = val;
        break;
    case R_INT_EN:
        s->int_en = val;
        break;
    case R_DEBUG0 ... R_DEBUG0 + 8:
        s->debug[(off - R_DEBUG0) >> 2] = val;
        break;
    case R_CMD:
        s->cmd = val;
        fh_rtc_command(s, val);
        break;
    case R_WR_DATA:
        s->wr_data = val;
        break;
    }
}

static const MemoryRegionOps fh_rtc_ops = {
    .read = fh_rtc_read,
    .write = fh_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_rtc_reset(DeviceState *dev)
{
    FhRtcState *s = FH_RTC(dev);

    s->int_status = ST_CORE_IDLE;
    s->int_en = 0;
    memset(s->debug, 0, sizeof(s->debug));
    s->cmd = s->rd_data = s->wr_data = 0;
}

static const Property fh_rtc_props[] = {
    DEFINE_PROP_BOOL("no-core", FhRtcState, no_core, false),
    DEFINE_PROP_BOOL("v1", FhRtcState, v1, false),
};

static void fh_rtc_realize(DeviceState *dev, Error **errp)
{
    FhRtcState *s = FH_RTC(dev);
    struct tm tm;

    /* The core keeps time across resets: start it from -rtc base=. */
    qemu_get_timedate(&tm, 0);
    s->offset = mktimegm(&tm) - fh_rtc_host_secs();
    s->v1_regs[V1_USER_REG >> 2] = V1_USER_MAGIC;
}

static void fh_rtc_init(Object *obj)
{
    FhRtcState *s = FH_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &fh_rtc_ops, s, TYPE_FH_RTC,
                          FH_RTC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_fh_rtc = {
    .name = TYPE_FH_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_status, FhRtcState),
        VMSTATE_UINT32(int_en, FhRtcState),
        VMSTATE_UINT32_ARRAY(debug, FhRtcState, 3),
        VMSTATE_UINT32(cmd, FhRtcState),
        VMSTATE_UINT32(rd_data, FhRtcState),
        VMSTATE_UINT32(wr_data, FhRtcState),
        VMSTATE_UINT32_ARRAY(core, FhRtcState, CORE_REGS),
        VMSTATE_INT64(offset, FhRtcState),
        VMSTATE_UINT32_ARRAY(v1_regs, FhRtcState, 9),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_rtc_realize;
    device_class_set_legacy_reset(dc, fh_rtc_reset);
    device_class_set_props(dc, fh_rtc_props);
    dc->vmsd = &vmstate_fh_rtc;
}

static const TypeInfo fh_rtc_info = {
    .name           = TYPE_FH_RTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhRtcState),
    .instance_init  = fh_rtc_init,
    .class_init     = fh_rtc_class_init,
};

static void fh_rtc_register_types(void)
{
    type_register_static(&fh_rtc_info);
}

type_init(fh_rtc_register_types)
