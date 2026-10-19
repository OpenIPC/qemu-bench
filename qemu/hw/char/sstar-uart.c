/*
 * SigmaStar / MStar UART.
 *
 * A 16550 with each register on its own 16-bit RIU slot, eight bytes
 * apart (drivers/sstar/serial/ms_uart.c: RBR/THR/DLL at 0x00, IER/DLH at
 * 0x08, IIR/FCR 0x10, LCR 0x18, MCR 0x20, LSR 0x28, MSR 0x30), followed
 * by DesignWare-style extras: USR at 0x38 (bit 0 busy, bit 1 TX FIFO not
 * full, bit 2 TX FIFO empty), TFL 0x40, RFL 0x48, RST 0x50.  The console
 * write path spins on USR "not full" (ms_uart.c line 916) and set_termios
 * on "not busy", so USR reports an idle UART with room in its FIFO.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/char/serial-mm.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_SSTAR_UART "sstar-uart"
OBJECT_DECLARE_SIMPLE_TYPE(SstarUartState, SSTAR_UART)

#define SSTAR_UART_SIZE     0x100
#define SSTAR_UART_EXTRA    0x38

#define USR_TFNF            (1u << 1)
#define USR_TFE             (1u << 2)

struct SstarUartState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion extra;
    SerialMM serial;
};

static uint64_t sstar_uart_extra_read(void *opaque, hwaddr off,
                                      unsigned size)
{
    switch (off) {
    case 0x00:                  /* USR */
        return USR_TFNF | USR_TFE;
    default:                    /* TFL, RFL, RST: empty, idle */
        return 0;
    }
}

static void sstar_uart_extra_write(void *opaque, hwaddr off, uint64_t val,
                                   unsigned size)
{
}

static const MemoryRegionOps sstar_uart_extra_ops = {
    .read = sstar_uart_extra_read,
    .write = sstar_uart_extra_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_uart_init(Object *obj)
{
    SstarUartState *s = SSTAR_UART(obj);

    object_initialize_child(obj, "serial", &s->serial, TYPE_SERIAL_MM);
    object_property_add_alias(obj, "chardev", OBJECT(&s->serial), "chardev");
}

static void sstar_uart_realize(DeviceState *dev, Error **errp)
{
    SstarUartState *s = SSTAR_UART(dev);
    DeviceState *sdev = DEVICE(&s->serial);

    qdev_prop_set_uint8(sdev, "regshift", 3);
    /* clk_uart 172.8 MHz; the console divisor is 172800000 / 16 / 115200. */
    qdev_prop_set_uint32(sdev, "baudbase", 172800000 / 16);
    qdev_prop_set_uint8(sdev, "endianness", DEVICE_LITTLE_ENDIAN);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->serial), errp)) {
        return;
    }
    sysbus_pass_irq(SYS_BUS_DEVICE(dev), SYS_BUS_DEVICE(&s->serial));

    memory_region_init(&s->container, OBJECT(s), "sstar-uart",
                       SSTAR_UART_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->container);
    memory_region_add_subregion(&s->container, 0,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->serial), 0));
    memory_region_init_io(&s->extra, OBJECT(s), &sstar_uart_extra_ops, s,
                          "sstar-uart.extra",
                          SSTAR_UART_SIZE - SSTAR_UART_EXTRA);
    /* Over the 16550's scratch register, which shares USR's slot. */
    memory_region_add_subregion_overlap(&s->container, SSTAR_UART_EXTRA,
                                        &s->extra, 1);
}

static void sstar_uart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_uart_realize;
}

static const TypeInfo sstar_uart_info = {
    .name           = TYPE_SSTAR_UART,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarUartState),
    .instance_init  = sstar_uart_init,
    .class_init     = sstar_uart_class_init,
};

static void sstar_uart_register_types(void)
{
    type_register_static(&sstar_uart_info);
}

type_init(sstar_uart_register_types)
