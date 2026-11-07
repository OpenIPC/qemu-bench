/*
 * Xiongmai XM8536E RTC (0x100A0000): a 32.768 kHz counter, 64 bits wide in
 * the registers.  From the board's xm_rtc.ko; the seconds are count >> 15.
 *
 *   +0x80  CMD      1 load the count from WRLO/WRHI, 2 latch it to RDLO/RDHI
 *   +0x84  WRLO, +0x88  WRHI
 *   +0x8C  RDLO, +0x90  RDHI
 *   +0x94  STATUS   bit 0 load done, bit 1 latch done, set by the command
 *
 * Other words are plain storage (init_module's +0xCEC, +0xCF0 are at
 * the far end of the block).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/system.h"

#define TYPE_XM_RTC2 "xm-rtc2"
OBJECT_DECLARE_SIMPLE_TYPE(XmRtc2State, XM_RTC2)

#define XM_RTC2_SIZE 0x10000
#define RTC_CMD      0x80
#define RTC_WRLO     0x84
#define RTC_WRHI     0x88
#define RTC_RDLO     0x8C
#define RTC_RDHI     0x90
#define RTC_STATUS   0x94
#define RTC_HZ       32768

struct XmRtc2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_RTC2_SIZE / 4];
    int64_t offset_ticks;
};

static uint64_t xm_rtc2_now(XmRtc2State *s)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_HOST);

    return muldiv64(ns, RTC_HZ, NANOSECONDS_PER_SECOND) + s->offset_ticks;
}

static uint64_t xm_rtc2_read(void *opaque, hwaddr offset, unsigned size)
{
    XmRtc2State *s = opaque;

    return s->regs[offset / 4];
}

static void xm_rtc2_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    XmRtc2State *s = opaque;

    s->regs[offset / 4] = value;
    if (offset != RTC_CMD) {
        return;
    }
    if (value == 1) {
        uint64_t want = ((uint64_t)s->regs[RTC_WRHI / 4] << 32) |
                        s->regs[RTC_WRLO / 4];

        s->offset_ticks += want - xm_rtc2_now(s);
        s->regs[RTC_STATUS / 4] |= 1;
    } else if (value == 2) {
        uint64_t now = xm_rtc2_now(s);

        s->regs[RTC_RDLO / 4] = now;
        s->regs[RTC_RDHI / 4] = now >> 32;
        s->regs[RTC_STATUS / 4] |= 2;
    }
}

static const MemoryRegionOps xm_rtc2_ops = {
    .read = xm_rtc2_read,
    .write = xm_rtc2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void xm_rtc2_init(Object *obj)
{
    XmRtc2State *s = XM_RTC2(obj);

    memory_region_init_io(&s->iomem, obj, &xm_rtc2_ops, s, TYPE_XM_RTC2,
                          XM_RTC2_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const TypeInfo xm_rtc2_info = {
    .name = TYPE_XM_RTC2,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmRtc2State),
    .instance_init = xm_rtc2_init,
};

static void xm_rtc2_register(void)
{
    type_register_static(&xm_rtc2_info);
}

type_init(xm_rtc2_register)
