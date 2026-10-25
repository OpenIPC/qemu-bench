/*
 * Fullhan UART: a DesignWare APB UART.
 *
 * The 16550 registers four bytes apart (drivers/tty/serial/fh_serial.c:
 * RBR/THR/DLL 0x00, IER/DLH 0x04, IIR/FCR 0x08, LCR 0x0C, MCR 0x10,
 * LSR 0x14, MSR 0x18, SCR 0x1C), then the DesignWare extras from 0x70:
 * USR 0x7C (bit 0 busy, 1 TX FIFO not full, 2 TX FIFO empty, 3 RX FIFO
 * not empty, 4 RX FIFO full), TFL 0x80, RFL 0x84, SRR 0x88, and the
 * component registers CPR 0xF4, UCV 0xF8, CTR 0xFC.
 *
 * The vendor drivers poll USR rather than LSR: the console and the TX
 * path wait for "TX FIFO not full", the RX path reads while "RX FIFO not
 * empty", and LCR / divisor writes are refused while "busy".  Bytes leave
 * at once here, so USR reports an idle UART with an empty TX FIFO, and
 * the RX bits follow the 16550's receive FIFO.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/fifo8.h"
#include "hw/char/serial-mm.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_FH_UART "fh-uart"
OBJECT_DECLARE_SIMPLE_TYPE(FhUartState, FH_UART)

#define FH_UART_SIZE        0x100
#define FH_UART_EXTRA       0x70

#define USR_TFNF            (1u << 1)
#define USR_TFE             (1u << 2)
#define USR_RFNE            (1u << 3)
#define USR_RFF             (1u << 4)

/* hw/char/serial.c keeps these to itself. */
#define UART_LSR_DR         0x01
#define UART_LSR_BI         0x10
#define UART_FCR_FE         0x01

#define SRR_UR              (1u << 0)
#define SRR_RFR             (1u << 1)
#define SRR_XFR             (1u << 2)

struct FhUartState {
    SysBusDevice parent_obj;

    MemoryRegion container;
    MemoryRegion extra;
    SerialMM serial;
    uint32_t clock;
};

static uint32_t fh_uart_rx_level(FhUartState *s)
{
    SerialState *ss = &s->serial.serial;

    if (ss->fcr & UART_FCR_FE) {
        return fifo8_num_used(&ss->recv_fifo);
    }
    return (ss->lsr & UART_LSR_DR) ? 1 : 0;
}

static uint64_t fh_uart_extra_read(void *opaque, hwaddr off, unsigned size)
{
    FhUartState *s = opaque;
    uint32_t rfl = fh_uart_rx_level(s);
    uint32_t v;

    switch (off + FH_UART_EXTRA) {
    case 0x7C:                  /* USR */
        v = USR_TFNF | USR_TFE;
        if (rfl) {
            v |= USR_RFNE;
        }
        if (rfl >= UART_FIFO_LENGTH) {
            v |= USR_RFF;
        }
        return v;
    case 0x84:                  /* RFL */
        return rfl;
    case 0xF4:                  /* CPR: 16-byte FIFOs, APB 32 bit, THRE mode */
        return 0x00010000 | (1 << 4) | 2;
    case 0xF8:                  /* UCV */
        return 0x3331312A;
    case 0xFC:                  /* CTR */
        return 0x44570110;
    default:                    /* TFL, shadow registers: empty */
        return 0;
    }
}

static void fh_uart_extra_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    FhUartState *s = opaque;
    SerialState *ss = &s->serial.serial;

    if (off + FH_UART_EXTRA == 0x88) {  /* SRR */
        if (val & (SRR_UR | SRR_RFR)) {
            fifo8_reset(&ss->recv_fifo);
            ss->lsr &= ~(UART_LSR_DR | UART_LSR_BI);
        }
    }
}

static const MemoryRegionOps fh_uart_extra_ops = {
    .read = fh_uart_extra_read,
    .write = fh_uart_extra_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void fh_uart_init(Object *obj)
{
    FhUartState *s = FH_UART(obj);

    object_initialize_child(obj, "serial", &s->serial, TYPE_SERIAL_MM);
    object_property_add_alias(obj, "chardev", OBJECT(&s->serial), "chardev");
}

static void fh_uart_realize(DeviceState *dev, Error **errp)
{
    FhUartState *s = FH_UART(dev);
    DeviceState *sdev = DEVICE(&s->serial);

    qdev_prop_set_uint8(sdev, "regshift", 2);
    qdev_prop_set_uint32(sdev, "baudbase", s->clock / 16);
    qdev_prop_set_uint8(sdev, "endianness", DEVICE_LITTLE_ENDIAN);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->serial), errp)) {
        return;
    }
    sysbus_pass_irq(SYS_BUS_DEVICE(dev), SYS_BUS_DEVICE(&s->serial));

    memory_region_init(&s->container, OBJECT(s), "fh-uart", FH_UART_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->container);
    memory_region_add_subregion(&s->container, 0,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->serial), 0));
    memory_region_init_io(&s->extra, OBJECT(s), &fh_uart_extra_ops, s,
                          "fh-uart.extra", FH_UART_SIZE - FH_UART_EXTRA);
    memory_region_add_subregion(&s->container, FH_UART_EXTRA, &s->extra);
}

static const Property fh_uart_props[] = {
    /* sysp_div12 / 6: the 16666667 Hz of the DTS and the U-Boot header */
    DEFINE_PROP_UINT32("clock", FhUartState, clock, 16666667),
};

static void fh_uart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = fh_uart_realize;
    device_class_set_props(dc, fh_uart_props);
}

static const TypeInfo fh_uart_info = {
    .name           = TYPE_FH_UART,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhUartState),
    .instance_init  = fh_uart_init,
    .class_init     = fh_uart_class_init,
};

static void fh_uart_register_types(void)
{
    type_register_static(&fh_uart_info);
}

type_init(fh_uart_register_types)
