/*
 * Fullhan GPIO: a DesignWare APB GPIO, ports A and B.
 *
 * drivers/gpio/fh_gpio.c and U-Boot's arch-fh/gpio.h:
 *
 *   0x00 SWPORTA_DR   0x04 SWPORTA_DDR   0x0C SWPORTB_DR   0x10 SWPORTB_DDR
 *   0x30 INTEN        0x34 INTMASK       0x38 INTTYPE_LEVEL (1 = edge)
 *   0x3C INT_POLARITY (1 = high / rising)
 *   0x40 INTSTATUS    0x44 RAWINTSTATUS  0x48 DEBOUNCE    0x4C PORTA_EOI
 *   0x50 EXT_PORTA    0x54 EXT_PORTB     0x60 LS_SYNC     0x68 INT_BOTH
 *
 * Port A interrupts; port B is plain I/O.  qdev GPIO lines 0-31 are port
 * A, 32-63 port B, both ways: an output line follows the pin while its
 * DDR bit makes it an output and goes high (the pull-up) otherwise, and
 * EXT_PORT reads the driven level of outputs and the input line of the
 * rest.  The SPI chip selects are GPIOs on these SoCs, so the machine
 * wires the flash's select to an output here.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_GPIO "fh-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(FhGpioState, FH_GPIO)

#define FH_GPIO_SIZE        0x1000
#define FH_GPIO_LINES       64

struct FhGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[FH_GPIO_LINES];
    uint32_t dr[2];
    uint32_t ddr[2];
    uint32_t in[2];             /* levels on the input lines */
    uint32_t inten;
    uint32_t intmask;
    uint32_t inttype;
    uint32_t intpol;
    uint32_t intboth;
    uint32_t debounce;
    uint32_t ls_sync;
    uint32_t edge;              /* latched edge interrupts, port A */
    uint32_t outlevel[2];       /* what the output lines last carried */
};

static uint32_t fh_gpio_pins(FhGpioState *s, int port)
{
    return (s->dr[port] & s->ddr[port]) | (s->in[port] & ~s->ddr[port]);
}

static uint32_t fh_gpio_raw(FhGpioState *s)
{
    uint32_t pins = fh_gpio_pins(s, 0);
    uint32_t level = ~s->inttype & (pins ^ ~s->intpol);

    return ((s->edge & s->inttype) | level) & s->inten;
}

static void fh_gpio_update(FhGpioState *s)
{
    int port, i;

    for (port = 0; port < 2; port++) {
        uint32_t lv = s->dr[port] | ~s->ddr[port];
        uint32_t chg = lv ^ s->outlevel[port];

        s->outlevel[port] = lv;
        for (i = 0; chg && i < 32; i++) {
            if (chg & (1u << i)) {
                qemu_set_irq(s->out[port * 32 + i], (lv >> i) & 1);
            }
        }
    }
    qemu_set_irq(s->irq, (fh_gpio_raw(s) & ~s->intmask) != 0);
}

static void fh_gpio_edges(FhGpioState *s, uint32_t old, uint32_t now)
{
    uint32_t rise = ~old & now, fall = old & ~now;
    uint32_t hit = (rise & (s->intpol | s->intboth)) |
                   (fall & (~s->intpol | s->intboth));

    s->edge |= hit & s->inttype & s->inten;
}

static void fh_gpio_set_in(void *opaque, int n, int level)
{
    FhGpioState *s = opaque;
    int port = n / 32;
    uint32_t old = fh_gpio_pins(s, 0);

    s->in[port] = deposit32(s->in[port], n % 32, 1, !!level);
    if (!port) {
        fh_gpio_edges(s, old, fh_gpio_pins(s, 0));
    }
    fh_gpio_update(s);
}

static uint64_t fh_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    FhGpioState *s = opaque;

    switch (off) {
    case 0x00: return s->dr[0];
    case 0x04: return s->ddr[0];
    case 0x0C: return s->dr[1];
    case 0x10: return s->ddr[1];
    case 0x30: return s->inten;
    case 0x34: return s->intmask;
    case 0x38: return s->inttype;
    case 0x3C: return s->intpol;
    case 0x40: return fh_gpio_raw(s) & ~s->intmask;
    case 0x44: return fh_gpio_raw(s);
    case 0x48: return s->debounce;
    case 0x50: return fh_gpio_pins(s, 0);
    case 0x54: return fh_gpio_pins(s, 1);
    case 0x60: return s->ls_sync;
    case 0x68: return s->intboth;
    default:   return 0;
    }
}

static void fh_gpio_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    FhGpioState *s = opaque;
    uint32_t old = fh_gpio_pins(s, 0);

    switch (off) {
    case 0x00: s->dr[0] = val; break;
    case 0x04: s->ddr[0] = val; break;
    case 0x0C: s->dr[1] = val; break;
    case 0x10: s->ddr[1] = val; break;
    case 0x30: s->inten = val; s->edge &= val; break;
    case 0x34: s->intmask = val; break;
    case 0x38: s->inttype = val; break;
    case 0x3C: s->intpol = val; break;
    case 0x48: s->debounce = val; break;
    case 0x4C: s->edge &= ~val; break;
    case 0x60: s->ls_sync = val; break;
    case 0x68: s->intboth = val; break;
    default:   return;
    }
    fh_gpio_edges(s, old, fh_gpio_pins(s, 0));
    fh_gpio_update(s);
}

static const MemoryRegionOps fh_gpio_ops = {
    .read = fh_gpio_read,
    .write = fh_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_gpio_reset(DeviceState *dev)
{
    FhGpioState *s = FH_GPIO(dev);

    s->dr[0] = s->dr[1] = 0;
    s->ddr[0] = s->ddr[1] = 0;
    s->inten = s->intmask = s->inttype = s->intpol = s->intboth = 0;
    s->debounce = s->ls_sync = s->edge = 0;
    fh_gpio_update(s);
}

static void fh_gpio_init(Object *obj)
{
    FhGpioState *s = FH_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &fh_gpio_ops, s, TYPE_FH_GPIO,
                          FH_GPIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(DEVICE(obj), fh_gpio_set_in, FH_GPIO_LINES);
    qdev_init_gpio_out(DEVICE(obj), s->out, FH_GPIO_LINES);
    /* Inputs float high, like the pads' pull-ups. */
    s->in[0] = s->in[1] = 0xFFFFFFFF;
    s->outlevel[0] = s->outlevel[1] = 0xFFFFFFFF;
}

static const VMStateDescription vmstate_fh_gpio = {
    .name = TYPE_FH_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(dr, FhGpioState, 2),
        VMSTATE_UINT32_ARRAY(ddr, FhGpioState, 2),
        VMSTATE_UINT32_ARRAY(in, FhGpioState, 2),
        VMSTATE_UINT32(inten, FhGpioState),
        VMSTATE_UINT32(intmask, FhGpioState),
        VMSTATE_UINT32(inttype, FhGpioState),
        VMSTATE_UINT32(intpol, FhGpioState),
        VMSTATE_UINT32(intboth, FhGpioState),
        VMSTATE_UINT32(debounce, FhGpioState),
        VMSTATE_UINT32(ls_sync, FhGpioState),
        VMSTATE_UINT32(edge, FhGpioState),
        VMSTATE_UINT32_ARRAY(outlevel, FhGpioState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, fh_gpio_reset);
    dc->vmsd = &vmstate_fh_gpio;
}

static const TypeInfo fh_gpio_info = {
    .name           = TYPE_FH_GPIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhGpioState),
    .instance_init  = fh_gpio_init,
    .class_init     = fh_gpio_class_init,
};

static void fh_gpio_register_types(void)
{
    type_register_static(&fh_gpio_info);
}

type_init(fh_gpio_register_types)
