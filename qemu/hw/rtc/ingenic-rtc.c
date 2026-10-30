/*
 * Ingenic RTC and hibernation controller (0x10003000; 0x132A0000 on T40).
 *
 *   RTCCR 0x00   7 WRDY (write ready), 0 RTCE (count)
 *   RTCSR 0x04   seconds
 *   RTCSAR 0x08, RTCGR 0x0C
 *   HCR 0x20     0 PD: power down
 *   HWFCR 0x24, HRCR 0x28, HWCR 0x2C, HWRSR 0x30, HSPR 0x34
 *   WENR 0x3C    write 0xA55A, then bit 31 (WEN) unlocks one write
 *
 * Every register write waits for WRDY, and the unlock waits for WEN, both
 * without a timeout in the 3.10 kernels' power-off path (soc-t31/common/
 * reset.c rtc_write_reg).  So WRDY is always set and WEN follows the
 * magic.  Writing HCR.PD powers the machine off.  RTCSR counts seconds
 * from the host's clock while RTCE is set.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "qemu/cutils.h"
#include "system/rtc.h"
#include "system/system.h"
#include "system/runstate.h"

#define TYPE_INGENIC_RTC "ingenic-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRtcState, INGENIC_RTC)

#define R_RTCCR     0x00
#define R_RTCSR     0x04
#define R_HCR       0x20
#define R_WENR      0x3C

#define RTCCR_WRDY  (1u << 7)
#define RTCCR_RTCE  (1u << 0)
#define HCR_PD      (1u << 0)
#define WENR_MAGIC  0xa55a
#define WENR_WEN    (1u << 31)

#define RTC_REGS    0x50

struct IngenicRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[RTC_REGS / 4];
    int64_t offset;         /* RTCSR = host seconds + offset */
};

static uint32_t rtc_seconds(IngenicRtcState *s)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND + s->offset;
}

static uint64_t ingenic_rtc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicRtcState *s = opaque;

    if (off >= RTC_REGS) {
        return 0;
    }
    switch (off) {
    case R_RTCCR:
        return s->regs[0] | RTCCR_WRDY;
    case R_RTCSR:
        return (s->regs[0] & RTCCR_RTCE) ? rtc_seconds(s) : s->regs[1];
    default:
        return s->regs[off >> 2];
    }
}

static void ingenic_rtc_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicRtcState *s = opaque;

    if (off >= RTC_REGS) {
        return;
    }
    switch (off) {
    case R_RTCSR:
        s->regs[1] = val;
        s->offset = (int64_t)(uint32_t)val -
                    qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
        break;
    case R_RTCCR:
        if ((val ^ s->regs[0]) & RTCCR_RTCE) {
            /* freeze or restart RTCSR where it is */
            uint32_t now = ingenic_rtc_read(s, R_RTCSR, 4);

            s->regs[1] = now;
            s->offset = (int64_t)now -
                        qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
        }
        s->regs[0] = val & ~RTCCR_WRDY;
        break;
    case R_WENR:
        s->regs[off >> 2] = (val & 0xffff) == WENR_MAGIC ? WENR_WEN | val : 0;
        break;
    case R_HCR:
        s->regs[off >> 2] = val;
        if (val & HCR_PD) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    default:
        s->regs[off >> 2] = val;
        break;
    }
}

static const MemoryRegionOps ingenic_rtc_ops = {
    .read = ingenic_rtc_read,
    .write = ingenic_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_rtc_reset(DeviceState *dev)
{
    IngenicRtcState *s = INGENIC_RTC(dev);
    struct tm tm;

    memset(s->regs, 0, sizeof(s->regs));
    qemu_get_timedate(&tm, 0);
    s->offset = mktimegm(&tm) -
                qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND;
    s->regs[0] = RTCCR_RTCE;
}

static void ingenic_rtc_init(Object *obj)
{
    IngenicRtcState *s = INGENIC_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_rtc_ops, s,
                          TYPE_INGENIC_RTC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ingenic_rtc = {
    .name = TYPE_INGENIC_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicRtcState, RTC_REGS / 4),
        VMSTATE_INT64(offset, IngenicRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_rtc_reset);
    dc->vmsd = &vmstate_ingenic_rtc;
}

static const TypeInfo ingenic_rtc_info = {
    .name = TYPE_INGENIC_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicRtcState),
    .instance_init = ingenic_rtc_init,
    .class_init = ingenic_rtc_class_init,
};

static void ingenic_rtc_register_types(void)
{
    type_register_static(&ingenic_rtc_info);
}

type_init(ingenic_rtc_register_types)
