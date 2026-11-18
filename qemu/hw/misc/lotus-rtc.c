/*
 * Lotus (Xiongmai / Goke GK7206 "xmorca") RTC behind an SPI bridge.
 *
 * Kernel driver: SDK lotus/drivers/rtc/rtc-lotus.c ("lotus,rtc", DT
 * rtc@120e0000, SPI 0).  Like the HiSilicon V4 RTC (hisi-rtc.c) the APB side
 * is a two-register SPI bridge:
 *
 *   0x000  SPI_CLK_DIV   divider (driver writes 4)
 *   0x004  SPI_RW        [7:0] wdata  [15:8] rdata  [22:16] addr
 *                        [23] 1=read 0=write  [24] start  [31] busy
 *
 * and the RTC registers live behind it, but the internal map is shifted by
 * one against hisi-rtc (10MS_COUN is 0x01, not 0x00), which is why the
 * hisi model made hctosys fail ("unable to read the hardware clock"):
 *
 *   0x01 10MS  0x02 S  0x03 M  0x04 H  0x05 D_L  0x06 D_H   live counters
 *   0x07..0x0c  MR_* (alarm match)      0x0d..0x12 LR_* (load value)
 *   0x13 LORD   bit0 LOAD (copy LR_* to counter, self-clears)
 *               bit1 LOCK (latch counter into 0x01..0x06, self-clears)
 *               bit2 LOCK_BYPASS
 *   0x14 IMSC (bit0 alarm, bit1 low-voltage)  0x15 INT_CLR  0x16 INT
 *   0x17 INT_RAW  0x18 CLK  0x19 POR_N  0x1a SAR_CTRL  0x1b CLK_CFG
 *   0x50/0x51 FREQ_H/L   0x64..0x67 REG_LOCK1..4
 *
 * Time is days:hours:minutes:seconds since the epoch, kept as an offset to
 * QEMU's RTC clock (so -rtc base=/clock= apply).  The RTC sits in the
 * battery domain: the offset and the register file survive system_reset,
 * only the APB bridge registers are reset — so a time written with
 * hwclock -w is still there after a guest reboot.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "system/system.h"
#include "system/rtc.h"
#include "migration/vmstate.h"

#define TYPE_LOTUS_RTC "lotus-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(LotusRtcState, LOTUS_RTC)

#define SPI_CLK_DIV     0x000
#define SPI_RW          0x004

#define SPI_RDATA_SHIFT 8
#define SPI_ADDR_SHIFT  16
#define SPI_ADDR_MASK   0x7f
#define SPI_RW_READ     (1u << 23)
#define SPI_START       (1u << 24)
#define SPI_BUSY        (1u << 31)

#define RTC_10MS_COUN   0x01
#define RTC_S_COUNT     0x02
#define RTC_M_COUNT     0x03
#define RTC_H_COUNT     0x04
#define RTC_D_COUNT_L   0x05
#define RTC_D_COUNT_H   0x06
#define RTC_MR_10MS     0x07
#define RTC_MR_S        0x08
#define RTC_MR_M        0x09
#define RTC_MR_H        0x0a
#define RTC_MR_D_L      0x0b
#define RTC_MR_D_H      0x0c
#define RTC_LR_10MS     0x0d
#define RTC_LR_S        0x0e
#define RTC_LR_M        0x0f
#define RTC_LR_H        0x10
#define RTC_LR_D_L      0x11
#define RTC_LR_D_H      0x12
#define RTC_LORD        0x13
#define RTC_IMSC        0x14
#define RTC_INT_CLR     0x15
#define RTC_INT         0x16
#define RTC_INT_RAW     0x17

#define LORD_LOAD       (1 << 0)
#define LORD_LOCK       (1 << 1)
#define INT_ALARM       (1 << 0)

#define RTC_NREGS       128

struct LotusRtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *alarm_timer;

    uint32_t spi_clk_div;
    uint32_t spi_rw;
    uint8_t regs[RTC_NREGS];
    int64_t offset_ns;      /* RTC time = rtc clock + offset_ns */
};

/* Guest RTC time in ns since the epoch. */
static int64_t lotus_rtc_now_ns(LotusRtcState *s)
{
    return qemu_clock_get_ns(rtc_clock) + s->offset_ns;
}

static void lotus_rtc_update_irq(LotusRtcState *s)
{
    s->regs[RTC_INT] = s->regs[RTC_INT_RAW] & s->regs[RTC_IMSC];
    qemu_set_irq(s->irq, s->regs[RTC_INT] != 0);
}

static void lotus_rtc_latch(LotusRtcState *s)
{
    int64_t ns = lotus_rtc_now_ns(s);
    int64_t secs = ns / NANOSECONDS_PER_SECOND;
    int64_t days = secs / 86400;
    int rem = secs % 86400;

    s->regs[RTC_10MS_COUN] = (ns % NANOSECONDS_PER_SECOND) / 10000000;
    s->regs[RTC_S_COUNT] = rem % 60;
    s->regs[RTC_M_COUNT] = (rem / 60) % 60;
    s->regs[RTC_H_COUNT] = rem / 3600;
    s->regs[RTC_D_COUNT_L] = days & 0xff;
    s->regs[RTC_D_COUNT_H] = (days >> 8) & 0xff;
}

static int64_t lotus_rtc_regs_to_ns(LotusRtcState *s, int base)
{
    /* base points at the 10MS register of an MR_* / LR_* group */
    int64_t day = s->regs[base + 4] | (s->regs[base + 5] << 8);
    int64_t secs = day * 86400 + s->regs[base + 3] * 3600 +
                   s->regs[base + 2] * 60 + s->regs[base + 1];

    return secs * NANOSECONDS_PER_SECOND +
           (int64_t)(s->regs[base] % 100) * 10000000;
}

static void lotus_rtc_arm_alarm(LotusRtcState *s)
{
    int64_t delta;

    if (!(s->regs[RTC_IMSC] & INT_ALARM)) {
        timer_del(s->alarm_timer);
        return;
    }
    delta = lotus_rtc_regs_to_ns(s, RTC_MR_10MS) - lotus_rtc_now_ns(s);
    if (delta < 0) {
        /* match register already in the past: never fires (not a >= cmp) */
        timer_del(s->alarm_timer);
        return;
    }
    timer_mod_ns(s->alarm_timer, qemu_clock_get_ns(rtc_clock) + delta);
}

static void lotus_rtc_alarm_cb(void *opaque)
{
    LotusRtcState *s = opaque;

    s->regs[RTC_INT_RAW] |= INT_ALARM;
    lotus_rtc_update_irq(s);
}

static void lotus_rtc_spi_write(LotusRtcState *s, uint8_t addr, uint8_t val)
{
    switch (addr) {
    case RTC_10MS_COUN ... RTC_D_COUNT_H:
    case RTC_INT:
    case RTC_INT_RAW:
        return;                         /* read-only */
    case RTC_LORD:
        s->regs[addr] = val & ~(LORD_LOAD | LORD_LOCK);
        if (val & LORD_LOAD) {
            s->offset_ns = lotus_rtc_regs_to_ns(s, RTC_LR_10MS) -
                           qemu_clock_get_ns(rtc_clock);
            lotus_rtc_arm_alarm(s);
        }
        if (val & LORD_LOCK) {
            lotus_rtc_latch(s);
        }
        return;
    case RTC_INT_CLR:
        s->regs[RTC_INT_RAW] &= ~val;
        lotus_rtc_update_irq(s);
        return;
    case RTC_IMSC:
        s->regs[addr] = val;
        lotus_rtc_update_irq(s);
        lotus_rtc_arm_alarm(s);
        return;
    case RTC_MR_10MS ... RTC_MR_D_H:
        s->regs[addr] = val;
        lotus_rtc_arm_alarm(s);
        return;
    default:
        s->regs[addr] = val;
        return;
    }
}

static uint64_t lotus_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    LotusRtcState *s = LOTUS_RTC(opaque);

    switch (offset) {
    case SPI_CLK_DIV:
        return s->spi_clk_div;
    case SPI_RW:
        return s->spi_rw;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "lotus-rtc: bad read 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void lotus_rtc_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    LotusRtcState *s = LOTUS_RTC(opaque);
    uint8_t addr;

    switch (offset) {
    case SPI_CLK_DIV:
        s->spi_clk_div = val;
        break;
    case SPI_RW:
        if (!(val & SPI_START)) {
            s->spi_rw = val & ~SPI_BUSY;
            break;
        }
        /* transfers complete at once: busy never reads back as set */
        addr = (val >> SPI_ADDR_SHIFT) & SPI_ADDR_MASK;
        val &= ~(SPI_START | SPI_BUSY | (0xffu << SPI_RDATA_SHIFT));
        if (val & SPI_RW_READ) {
            val |= (uint32_t)s->regs[addr] << SPI_RDATA_SHIFT;
        } else {
            lotus_rtc_spi_write(s, addr, val & 0xff);
        }
        s->spi_rw = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "lotus-rtc: bad write 0x%" HWADDR_PRIx
                      "\n", offset);
        break;
    }
}

static const MemoryRegionOps lotus_rtc_ops = {
    .read = lotus_rtc_read,
    .write = lotus_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void lotus_rtc_init(Object *obj)
{
    LotusRtcState *s = LOTUS_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lotus_rtc_ops, s, "lotus-rtc",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void lotus_rtc_realize(DeviceState *dev, Error **errp)
{
    LotusRtcState *s = LOTUS_RTC(dev);
    struct tm tm;

    s->alarm_timer = timer_new_ns(rtc_clock, lotus_rtc_alarm_cb, s);
    /* power-on value: host time, or -rtc base= */
    qemu_get_timedate(&tm, 0);
    s->offset_ns = (int64_t)mktimegm(&tm) * NANOSECONDS_PER_SECOND -
                   qemu_clock_get_ns(rtc_clock);
    lotus_rtc_latch(s);
}

/* Bridge only: the RTC macro is battery-backed and keeps time and regs. */
static void lotus_rtc_reset(DeviceState *dev)
{
    LotusRtcState *s = LOTUS_RTC(dev);

    s->spi_clk_div = 0;
    s->spi_rw = 0;
}

static const VMStateDescription vmstate_lotus_rtc = {
    .name = TYPE_LOTUS_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(spi_clk_div, LotusRtcState),
        VMSTATE_UINT32(spi_rw, LotusRtcState),
        VMSTATE_UINT8_ARRAY(regs, LotusRtcState, RTC_NREGS),
        VMSTATE_INT64(offset_ns, LotusRtcState),
        VMSTATE_TIMER_PTR(alarm_timer, LotusRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void lotus_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lotus_rtc_realize;
    dc->vmsd = &vmstate_lotus_rtc;
    device_class_set_legacy_reset(dc, lotus_rtc_reset);
}

static const TypeInfo lotus_rtc_info = {
    .name          = TYPE_LOTUS_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LotusRtcState),
    .instance_init = lotus_rtc_init,
    .class_init    = lotus_rtc_class_init,
};

static void lotus_rtc_register_types(void)
{
    type_register_static(&lotus_rtc_info);
}

type_init(lotus_rtc_register_types)
