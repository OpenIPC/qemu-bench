/*
 * Ingenic bus MONITOR (T40, T41: 0x130A0000, core INTC source 53): the
 * AXI/AHB bandwidth and address-watch unit of the SDKs' sample_bus_monitor
 * driver (drivers/misc/sample_bus_monitor, bus_monitor.h).
 *
 * Six AXI channels (0..4, 6: 0x100 n each) watch the AHB0 masters --
 * tiziano, radix, ipu/msc/i2d/drawbox, bscaler/lcd/vo, ... -- and one AHB
 * channel (5, at 0x500) the AHB2 masters:
 *
 *   0x00 CTRL    31 clock, 30 soft reset, 29 enable, 18 watch, 17 monitor,
 *                16 ID mode, [15:8] ID mask, 2 capture, 1 clear, 0 lock
 *   0x04 SLOT_CYC_NUM   the counting window, in bus cycles
 *   0x08 RD_WR_ID  0x0C BUF_LIMIT_CYC  0x50/0x54 watch range: storage
 *   0x10..0x28 / 0x2C..0x48  read / write counters: cycles, commands,
 *                ARVALID/AWVALID, RVALID/WVALID, data, last, (bresp),
 *                data-to-last
 *   0x60..0x68 watch matches  0x70 INT_STA  0x74 INT_MASK (1 masks)
 *   0x78 INT_CLR (write 1)
 *   AHB (0x500): CTRL, watch range at 0x504/0x508, matches 0x510..0x518,
 *   INT_STA 0x570, INT_MASK 0x574, INT_CLR 0x578
 *
 * Counting runs while CTRL enables it in monitor mode: the cycle counters
 * follow virtual time at "clock-hz" (300 MHz), and the window's end raises
 * INT_STA bit 4 (TIME_SLOT_REACH).  QEMU's memory accesses go past no bus
 * here, so the traffic counters stay 0 and a watched range never matches.
 * Capture latches the counters for reading while lock is set; clear zeroes
 * them and clears itself.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_MONITOR "ingenic-monitor"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMonitorState, INGENIC_MONITOR)

#define NCHAN           7
#define AHB_CHAN        5
#define CHAN_REGS       0x80

#define R_CTRL          0x00
#define R_SLOT          0x04
#define R_RD_CYC        0x10
#define R_WR_CYC        0x2C
#define R_INT_STA       0x70
#define R_INT_MASK      0x74
#define R_INT_CLR       0x78

#define CTRL_RESET      (1u << 30)
#define CTRL_EN         (1u << 29)
#define CTRL_MON        (1u << 17)
#define CTRL_CAPTURE    (1u << 2)
#define CTRL_CLEAR      (1u << 1)
#define CTRL_LOCK       (1u << 0)

#define INT_SLOT        (1u << 4)

struct IngenicMonitorState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer[NCHAN];
    uint32_t regs[NCHAN][CHAN_REGS / 4];
    uint32_t latched[NCHAN][2];     /* the captured cycle counters */
    int64_t start[NCHAN];           /* when counting (re)started, ns */
    uint32_t clock_hz;
};

static bool mon_counting(IngenicMonitorState *s, int c)
{
    uint32_t ctrl = s->regs[c][R_CTRL / 4];

    return c != AHB_CHAN && (ctrl & CTRL_EN) && (ctrl & CTRL_MON) &&
           !(ctrl & CTRL_RESET);
}

static uint32_t mon_cycles(IngenicMonitorState *s, int c)
{
    int64_t ns;

    if (!mon_counting(s, c)) {
        return 0;
    }
    ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->start[c];
    return MIN(muldiv64(ns, s->clock_hz, NANOSECONDS_PER_SECOND), UINT32_MAX);
}

static void mon_update(IngenicMonitorState *s)
{
    bool level = false;
    int c;

    for (c = 0; c < NCHAN; c++) {
        level |= (s->regs[c][R_INT_STA / 4] & ~s->regs[c][R_INT_MASK / 4] &
                  0x7f) != 0;
    }
    qemu_set_irq(s->irq, level);
}

static void mon_arm(IngenicMonitorState *s, int c)
{
    uint32_t slot = s->regs[c][R_SLOT / 4];

    if (mon_counting(s, c) && slot) {
        timer_mod(s->timer[c], s->start[c] +
                  muldiv64(slot, NANOSECONDS_PER_SECOND, s->clock_hz));
    } else {
        timer_del(s->timer[c]);
    }
}

static void mon_slot(void *opaque)
{
    IngenicMonitorState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int c;

    for (c = 0; c < NCHAN; c++) {
        if (mon_counting(s, c) && !timer_pending(s->timer[c]) &&
            mon_cycles(s, c) >= s->regs[c][R_SLOT / 4]) {
            s->regs[c][R_INT_STA / 4] |= INT_SLOT;
            s->start[c] = now;
            mon_arm(s, c);
        }
    }
    mon_update(s);
}

static int mon_chan(hwaddr addr, hwaddr *off)
{
    int c = addr >> 8;

    if (c >= NCHAN) {
        return -1;
    }
    *off = addr & 0xff;                 /* AHB: the same offsets at 0x500 */
    return *off < CHAN_REGS ? c : -1;
}

static uint64_t ingenic_monitor_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicMonitorState *s = opaque;
    hwaddr off;
    int c = mon_chan(addr, &off);

    if (c < 0) {
        return 0;
    }
    if (c != AHB_CHAN && (off == R_RD_CYC || off == R_WR_CYC)) {
        if (s->regs[c][R_CTRL / 4] & CTRL_LOCK) {
            return s->latched[c][off == R_WR_CYC];
        }
        return mon_cycles(s, c);
    }
    if (c != AHB_CHAN && off > R_RD_CYC && off <= 0x48) {
        return 0;                       /* traffic: no bus to count */
    }
    return s->regs[c][off / 4];
}

static void ingenic_monitor_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    IngenicMonitorState *s = opaque;
    hwaddr off;
    int c = mon_chan(addr, &off);
    uint32_t old;

    if (c < 0) {
        return;
    }
    switch (off) {
    case R_CTRL:
        old = s->regs[c][R_CTRL / 4];
        if (val & CTRL_CAPTURE) {
            s->latched[c][0] = s->latched[c][1] = mon_cycles(s, c);
        }
        s->regs[c][R_CTRL / 4] = val & ~(CTRL_CAPTURE | CTRL_CLEAR);
        if ((val & CTRL_CLEAR) || (val & CTRL_RESET) ||
            ((val & CTRL_EN) && !(old & CTRL_EN))) {
            s->start[c] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            if (val & CTRL_RESET) {
                s->latched[c][0] = s->latched[c][1] = 0;
            }
        }
        mon_arm(s, c);
        break;
    case R_SLOT:
        s->regs[c][off / 4] = val;
        mon_arm(s, c);
        break;
    case R_INT_STA:
        break;
    case R_INT_CLR:
        s->regs[c][R_INT_STA / 4] &= ~val;
        break;
    default:
        s->regs[c][off / 4] = val;
        break;
    }
    mon_update(s);
}

static const MemoryRegionOps ingenic_monitor_ops = {
    .read = ingenic_monitor_read,
    .write = ingenic_monitor_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_monitor_reset(DeviceState *dev)
{
    IngenicMonitorState *s = INGENIC_MONITOR(dev);
    int c;

    for (c = 0; c < NCHAN; c++) {
        timer_del(s->timer[c]);
        s->start[c] = 0;
    }
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->latched, 0, sizeof(s->latched));
    mon_update(s);
}

static void ingenic_monitor_init(Object *obj)
{
    IngenicMonitorState *s = INGENIC_MONITOR(obj);
    int c;

    memory_region_init_io(&s->iomem, obj, &ingenic_monitor_ops, s,
                          TYPE_INGENIC_MONITOR, 0x8000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (c = 0; c < NCHAN; c++) {
        s->timer[c] = timer_new_ns(QEMU_CLOCK_VIRTUAL, mon_slot, s);
    }
}

static void ingenic_monitor_realize(DeviceState *dev, Error **errp)
{
    IngenicMonitorState *s = INGENIC_MONITOR(dev);

    if (!s->clock_hz) {
        s->clock_hz = 300000000;
    }
}

static const Property ingenic_monitor_props[] = {
    DEFINE_PROP_UINT32("clock-hz", IngenicMonitorState, clock_hz, 300000000),
};

static const VMStateDescription vmstate_ingenic_monitor = {
    .name = TYPE_INGENIC_MONITOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, IngenicMonitorState, NCHAN,
                               CHAN_REGS / 4),
        VMSTATE_UINT32_2DARRAY(latched, IngenicMonitorState, NCHAN, 2),
        VMSTATE_INT64_ARRAY(start, IngenicMonitorState, NCHAN),
        VMSTATE_TIMER_PTR_ARRAY(timer, IngenicMonitorState, NCHAN),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_monitor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_monitor_realize;
    device_class_set_legacy_reset(dc, ingenic_monitor_reset);
    device_class_set_props(dc, ingenic_monitor_props);
    dc->vmsd = &vmstate_ingenic_monitor;
}

static const TypeInfo ingenic_monitor_info = {
    .name = TYPE_INGENIC_MONITOR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicMonitorState),
    .instance_init = ingenic_monitor_init,
    .class_init = ingenic_monitor_class_init,
};

static void ingenic_monitor_register_types(void)
{
    type_register_static(&ingenic_monitor_info);
}

type_init(ingenic_monitor_register_types)
