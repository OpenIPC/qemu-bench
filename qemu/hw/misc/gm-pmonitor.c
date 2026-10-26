/*
 * Grain Media pMonitor, the AXI/AHB bandwidth monitor.
 *
 * At 0x91D00000 on the GM813x (module/pmonitor/process_monitor.c):
 *
 *   0x00 CTRL     bit 0 start, 1 interrupt clear (self-clearing),
 *                 2 interrupt enable
 *   0x04 PERIOD   the measuring window in APB clocks
 *   0x0C-0xC8     16 blocks x 3 counters of the last window
 *
 * A started window ends after PERIOD APB clocks: the monitor stops and,
 * with the interrupt enabled, raises interrupt 44, which the driver's
 * handler clears before reading the counters and starting the next
 * window.  The bus is not watched, so the counters read 0.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/clock.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_PMONITOR "gm-pmonitor"
OBJECT_DECLARE_SIMPLE_TYPE(GmPmonState, GM_PMONITOR)

#define GM_PMON_SIZE        0x1000
#define R_CTRL              0x00
#define R_PERIOD            0x04

#define CTRL_START          (1u << 0)
#define CTRL_INT_CLR        (1u << 1)
#define CTRL_INT_EN         (1u << 2)

struct GmPmonState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    Clock *clk;
    uint32_t ctrl;
    uint32_t period;
    bool pending;
};

static void gm_pmon_update(GmPmonState *s)
{
    qemu_set_irq(s->irq, s->pending && (s->ctrl & CTRL_INT_EN));
}

static void gm_pmon_window(void *opaque)
{
    GmPmonState *s = opaque;

    s->ctrl &= ~CTRL_START;
    s->pending = true;
    gm_pmon_update(s);
}

static uint64_t gm_pmon_read(void *opaque, hwaddr off, unsigned size)
{
    GmPmonState *s = opaque;

    switch (off) {
    case R_CTRL:
        return s->ctrl;
    case R_PERIOD:
        return s->period;
    }
    return 0;
}

static void gm_pmon_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmPmonState *s = opaque;

    switch (off) {
    case R_CTRL:
        if (val & CTRL_INT_CLR) {
            s->pending = false;
        }
        if ((val & CTRL_START) && !(s->ctrl & CTRL_START)) {
            uint64_t ns = clock_ticks_to_ns(s->clk, MAX(s->period, 1));

            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                MAX(ns, 1000000));
        } else if (!(val & CTRL_START)) {
            timer_del(s->timer);
        }
        s->ctrl = val & (CTRL_START | CTRL_INT_EN);
        gm_pmon_update(s);
        break;
    case R_PERIOD:
        s->period = val;
        break;
    }
}

static const MemoryRegionOps gm_pmon_ops = {
    .read = gm_pmon_read,
    .write = gm_pmon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_pmon_reset(DeviceState *dev)
{
    GmPmonState *s = GM_PMONITOR(dev);

    timer_del(s->timer);
    s->ctrl = s->period = 0;
    s->pending = false;
    gm_pmon_update(s);
}

static void gm_pmon_init(Object *obj)
{
    GmPmonState *s = GM_PMONITOR(obj);

    memory_region_init_io(&s->iomem, obj, &gm_pmon_ops, s, TYPE_GM_PMONITOR,
                          GM_PMON_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", NULL, NULL, 0);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_pmon_window, s);
}

static const VMStateDescription vmstate_gm_pmon = {
    .name = TYPE_GM_PMONITOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, GmPmonState),
        VMSTATE_TIMER_PTR(timer, GmPmonState),
        VMSTATE_UINT32(ctrl, GmPmonState),
        VMSTATE_UINT32(period, GmPmonState),
        VMSTATE_BOOL(pending, GmPmonState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_pmon_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, gm_pmon_reset);
    dc->vmsd = &vmstate_gm_pmon;
}

static const TypeInfo gm_pmon_info = {
    .name           = TYPE_GM_PMONITOR,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmPmonState),
    .instance_init  = gm_pmon_init,
    .class_init     = gm_pmon_class_init,
};

static void gm_pmon_register_types(void)
{
    type_register_static(&gm_pmon_info);
}

type_init(gm_pmon_register_types)
