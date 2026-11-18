/*
 * Ingenic RTC and hibernation controller (0x10003000; 0x132A0000 on T40).
 *
 *   RTCCR 0x00   7 WRDY (write ready), 0 RTCE (count)
 *   RTCSR 0x04   seconds
 *   RTCSAR 0x08, RTCGR 0x0C
 *   HCR 0x20     0 PD: power down
 *   HWFCR 0x24, HRCR 0x28, HWCR 0x2C, HWRSR 0x30, HSPR 0x34
 *   WENR 0x3C    write 0xA55A, then bit 31 (WEN) unlocks one write (a
 *                write without it is dropped, and the write uses it up)
 *
 * Every register write waits for WRDY, and the unlock waits for WEN, both
 * without a timeout in the 3.10 kernels' power-off path (soc-t31/common/
 * reset.c rtc_write_reg).  So WRDY is always set and WEN follows the
 * magic.  Writing HCR.PD powers the machine off.  RTCSR counts seconds
 * from the host's clock while RTCE is set.
 *
 * Every second (rtc-jz.h): RTCCR.1HZ sets, and with AE an RTCSR equal to
 * RTCSAR sets RTCCR.AF; the line (INTC 32 on T10..T31, 3 on T41) is high
 * while AF and AIE, or 1HZ and 1HZIE; the driver clears a flag by
 * writing it 0.  An alarm with HWCR.EALM also latches HWRSR.ALM and wakes
 * a suspended machine.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
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
#define RTCCR_1HZ   (1u << 6)
#define RTCCR_1HZIE (1u << 5)
#define RTCCR_AF    (1u << 4)
#define RTCCR_AIE   (1u << 3)
#define RTCCR_AE    (1u << 2)
#define RTCCR_RTCE  (1u << 0)
#define R_RTCSAR    0x08
#define R_HWCR      0x2C
#define R_HWRSR     0x30
#define HWCR_EALM   (1u << 0)
#define HWRSR_ALM   (1u << 0)
#define HCR_PD      (1u << 0)
#define WENR_MAGIC  0xa55a
#define WENR_WEN    (1u << 31)

#define RTC_REGS    0x50

struct IngenicRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *tick;        /* each second of RTCSR */
    uint32_t regs[RTC_REGS / 4];
    int64_t offset;         /* RTCSR = host seconds + offset */
};

static uint32_t rtc_seconds(IngenicRtcState *s)
{
    return qemu_clock_get_ns(rtc_clock) / NANOSECONDS_PER_SECOND + s->offset;
}

static void rtc_update_irq(IngenicRtcState *s)
{
    uint32_t cr = s->regs[0];

    qemu_set_irq(s->irq, ((cr & RTCCR_AF) && (cr & RTCCR_AIE)) ||
                         ((cr & RTCCR_1HZ) && (cr & RTCCR_1HZIE)));
}

/* the next second boundary of RTCSR, on rtc_clock */
static void rtc_arm(IngenicRtcState *s)
{
    int64_t now = qemu_clock_get_ns(rtc_clock);

    timer_mod(s->tick, now - now % NANOSECONDS_PER_SECOND +
              NANOSECONDS_PER_SECOND);
}

static void rtc_tick(void *opaque)
{
    IngenicRtcState *s = opaque;
    uint32_t *cr = &s->regs[0];

    if (*cr & RTCCR_RTCE) {
        *cr |= RTCCR_1HZ;
        if ((*cr & RTCCR_AE) && rtc_seconds(s) == s->regs[R_RTCSAR >> 2]) {
            *cr |= RTCCR_AF;
            if (s->regs[R_HWCR >> 2] & HWCR_EALM) {
                s->regs[R_HWRSR >> 2] |= HWRSR_ALM;
                qemu_system_wakeup_request(QEMU_WAKEUP_REASON_RTC, NULL);
            }
        }
        rtc_update_irq(s);
    }
    rtc_arm(s);
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
    if (off != R_WENR) {
        /* every other register takes one write per unlock */
        if (!(s->regs[R_WENR >> 2] & WENR_WEN)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-rtc: write of 0x%"
                          HWADDR_PRIx " without the WENR unlock\n", off);
            return;
        }
        s->regs[R_WENR >> 2] &= ~WENR_WEN;
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
        rtc_update_irq(s);
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
    rtc_update_irq(s);
    rtc_arm(s);
}

static void ingenic_rtc_init(Object *obj)
{
    IngenicRtcState *s = INGENIC_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_rtc_ops, s,
                          TYPE_INGENIC_RTC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->tick = timer_new_ns(rtc_clock, rtc_tick, s);
}

static const VMStateDescription vmstate_ingenic_rtc = {
    .name = TYPE_INGENIC_RTC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicRtcState, RTC_REGS / 4),
        VMSTATE_INT64(offset, IngenicRtcState),
        VMSTATE_TIMER_PTR(tick, IngenicRtcState),
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
