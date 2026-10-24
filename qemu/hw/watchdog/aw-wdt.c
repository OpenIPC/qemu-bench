/*
 * Allwinner sun6i/sun50i watchdog (Linux sunxi_wdt.c "sun50i-wdt"; the
 * V831's at 0x030090A0, behind the timer block).
 *
 *   0x00 IRQ_EN, 0x04 IRQ_STA
 *   0x10 CTRL  bit 0 restart, with key 0xA57 in bits 12:1
 *   0x14 CFG   bits 1:0 = 1: a timeout resets the system (2: interrupt)
 *   0x18 MODE  bit 0 enable, bits 7:4 interval (0.5, 1, 2, ..., 16 s)
 *
 * U-Boot and Linux reboot through it: whole-system mode, the shortest
 * interval, enable.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "system/runstate.h"
#include "qom/object.h"

#define TYPE_AW_WDT "aw-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(AwWdtState, AW_WDT)

#define AW_WDT_SIZE     0x20
#define REG_IRQ_EN      0x00
#define REG_IRQ_STA     0x04
#define REG_CTRL        0x10
#define REG_CFG         0x14
#define REG_MODE        0x18

#define CTRL_KEY        (0xA57u << 1)
#define CTRL_RESTART    1u
#define MODE_EN         1u

struct AwWdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t irq_en, irq_sta, cfg, mode;
};

/* MODE bits 7:4, in milliseconds. */
static const uint32_t aw_wdt_ms[16] = {
    500, 1000, 2000, 3000, 4000, 5000, 6000, 8000,
    10000, 12000, 14000, 16000, 16000, 16000, 16000, 16000,
};

static void aw_wdt_restart(AwWdtState *s)
{
    if (s->mode & MODE_EN) {
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  aw_wdt_ms[(s->mode >> 4) & 0xF]);
    } else {
        timer_del(s->timer);
    }
}

static void aw_wdt_expired(void *opaque)
{
    AwWdtState *s = opaque;

    if ((s->cfg & 3) == 1) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    s->irq_sta |= 1;
    qemu_set_irq(s->irq, !!(s->irq_sta & s->irq_en));
}

static uint64_t aw_wdt_read(void *opaque, hwaddr addr, unsigned size)
{
    AwWdtState *s = opaque;

    switch (addr) {
    case REG_IRQ_EN:
        return s->irq_en;
    case REG_IRQ_STA:
        return s->irq_sta;
    case REG_CFG:
        return s->cfg;
    case REG_MODE:
        return s->mode;
    }
    return 0;
}

static void aw_wdt_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwWdtState *s = opaque;

    switch (addr) {
    case REG_IRQ_EN:
        s->irq_en = val & 1;
        break;
    case REG_IRQ_STA:
        s->irq_sta &= ~val;
        break;
    case REG_CTRL:
        if ((val & 0x1FFE) == CTRL_KEY && (val & CTRL_RESTART)) {
            aw_wdt_restart(s);
        }
        break;
    case REG_CFG:
        s->cfg = val & 3;
        break;
    case REG_MODE:
        s->mode = val & 0xF1;
        aw_wdt_restart(s);
        break;
    }
    qemu_set_irq(s->irq, !!(s->irq_sta & s->irq_en));
}

static const MemoryRegionOps aw_wdt_ops = {
    .read = aw_wdt_read,
    .write = aw_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_wdt_reset_hold(Object *obj, ResetType type)
{
    AwWdtState *s = AW_WDT(obj);

    timer_del(s->timer);
    s->irq_en = s->irq_sta = 0;
    s->cfg = 1;
    s->mode = 0;
}

static void aw_wdt_reset_exit(Object *obj, ResetType type)
{
    AwWdtState *s = AW_WDT(obj);

    qemu_set_irq(s->irq, !!(s->irq_sta & s->irq_en));
}

static void aw_wdt_init(Object *obj)
{
    AwWdtState *s = AW_WDT(obj);

    memory_region_init_io(&s->iomem, obj, &aw_wdt_ops, s, TYPE_AW_WDT,
                          AW_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, aw_wdt_expired, s);
}

static void aw_wdt_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_wdt_reset_hold;
    rc->phases.exit = aw_wdt_reset_exit;
}

static const TypeInfo aw_wdt_info = {
    .name           = TYPE_AW_WDT,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwWdtState),
    .instance_init  = aw_wdt_init,
    .class_init     = aw_wdt_class_init,
};

static void aw_wdt_register_types(void)
{
    type_register_static(&aw_wdt_info);
}

type_init(aw_wdt_register_types)
