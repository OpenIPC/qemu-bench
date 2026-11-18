/*
 * Allwinner PIO (pin controller and GPIO), the sunxi layout: banks of
 * 0x24 bytes from 0, each
 *
 *   0x00..0x0C MUX   4 bits a pin, 0 = input, 1 = output
 *   0x10       DATA
 *   0x14..0x18 DRV   0x1C..0x20 PULL  2 bits a pin, 1 = up, 2 = down
 *
 * and the external interrupt registers from 0x200 (storage).
 *
 * Nothing is wired to the pins, so an input reads what a board with
 * pull-ups on its buttons and straps reads at rest: high, unless the pin
 * is pulled down internally.  A pin in another function reads back what
 * was written.  With plain storage every input read 0, which on the V831
 * YI camera is its reset button held down: the firmware factory-reset
 * itself on every boot.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AW_PIO "aw-pio"
OBJECT_DECLARE_SIMPLE_TYPE(AwPioState, AW_PIO)

#define AW_PIO_SIZE         0x400
#define AW_PIO_BANK         0x24
#define AW_PIO_BANKS        (0x200 / AW_PIO_BANK)

struct AwPioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[AW_PIO_SIZE / 4];
};

static uint32_t aw_pio_data(AwPioState *s, unsigned bank)
{
    uint32_t *b = &s->regs[bank * AW_PIO_BANK / 4];
    uint32_t v = b[0x10 / 4];
    unsigned pin;

    for (pin = 0; pin < 32; pin++) {
        uint32_t mux = (b[pin / 8] >> ((pin % 8) * 4)) & 0xF;
        uint32_t pull = (b[(0x1C / 4) + pin / 16] >> ((pin % 16) * 2)) & 3;

        if (mux == 0) {
            v = deposit32(v, pin, 1, pull != 2);
        }
    }
    return v;
}

static uint64_t aw_pio_read(void *opaque, hwaddr addr, unsigned size)
{
    AwPioState *s = opaque;

    if (addr < AW_PIO_BANKS * AW_PIO_BANK && addr % AW_PIO_BANK == 0x10) {
        return aw_pio_data(s, addr / AW_PIO_BANK);
    }
    return s->regs[addr >> 2];
}

static void aw_pio_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    AwPioState *s = opaque;

    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps aw_pio_ops = {
    .read = aw_pio_read,
    .write = aw_pio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_pio_reset_hold(Object *obj, ResetType type)
{
    AwPioState *s = AW_PIO(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aw_pio_init(Object *obj)
{
    AwPioState *s = AW_PIO(obj);

    memory_region_init_io(&s->iomem, obj, &aw_pio_ops, s, TYPE_AW_PIO,
                          AW_PIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void aw_pio_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_pio_reset_hold;
}

static const TypeInfo aw_pio_info = {
    .name           = TYPE_AW_PIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwPioState),
    .instance_init  = aw_pio_init,
    .class_init     = aw_pio_class_init,
};

static void aw_pio_register_types(void)
{
    type_register_static(&aw_pio_info);
}

type_init(aw_pio_register_types)
