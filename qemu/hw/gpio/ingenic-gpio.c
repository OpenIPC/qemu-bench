/*
 * Ingenic GPIO: the T-series pin controller at 0x10010000.
 *
 * One block of registers per 32-pin port, port n at n * stride (0x1000;
 * 0x100 on T10/T20), each register with a set (+4) and a clear (+8)
 * alias (soc-t31/common/gpio.c, drivers/pinctrl/pinctrl-ingenic.h):
 *
 *   0x00 PIN (read only)   0x10 INT   0x20 MSK   0x30 PAT1   0x40 PAT0
 *   0x50 FLG (0x58 clears) 0x110 PUEN 0x120 PDEN 0x130.. drive, slew, ...
 *
 * A pin is a function (INT 0, MSK 0, PAT1:PAT0 = function), a GPIO output
 * (INT 0, MSK 1, PAT1 0, PAT0 = level), a GPIO input (INT 0, MSK 1,
 * PAT1 1) or an interrupt (INT 1; PAT1:PAT0 = low, high, falling,
 * rising; MSK masks it).  Functions are changed through the shadow port
 * "Z" (at 0x7000; 0xF00 on T10/T20): writes to its INT/MSK/PAT1/PAT0
 * set and clear aliases collect bits, and GID2LD (Z + 0xF0) = port
 * applies them to that port at once.
 *
 * PIN reads an output's level and an input's external level; nothing
 * drives the inputs unless a board wires a gpio-in line, so they read
 * high (pulled up) -- a card-detect reads "no card".  Each port has one
 * interrupt line, high while FLG & ~MSK.  The "drive" outputs (one per
 * pin) give what the SoC drives onto each pin: low for a GPIO output at
 * 0, else high (released, pulled up) -- an open-drain line such as the
 * T10 kernel's bit-banged I2C (i2c-gpio on PA12/PA13), which a board
 * wires to QEMU's gpio_i2c.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_GPIO "ingenic-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicGpioState, INGENIC_GPIO)

#define MAX_PORTS       6
#define PORT_REGS       0x200       /* register space kept per port */

#define R_PIN           0x00
#define R_INT           0x10
#define R_MSK           0x20
#define R_PAT1          0x30
#define R_PAT0          0x40
#define R_FLG           0x50
#define R_GID2LD        0xF0

struct IngenicGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[MAX_PORTS];
    uint32_t nports, stride, shadow;

    uint32_t regs[MAX_PORTS][PORT_REGS / 4];
    uint32_t input[MAX_PORTS];
    uint32_t zset[4], zclr[4];      /* INT, MSK, PAT1, PAT0 */
    uint32_t drive[MAX_PORTS];      /* the levels last sent on "drive" */
    qemu_irq drive_out[MAX_PORTS * 32];
};

static uint32_t *gp_reg(IngenicGpioState *s, int port, hwaddr off)
{
    return &s->regs[port][off >> 2];
}

static uint32_t gpio_pin(IngenicGpioState *s, int p)
{
    uint32_t in = *gp_reg(s, p, R_INT), msk = *gp_reg(s, p, R_MSK);
    uint32_t pat1 = *gp_reg(s, p, R_PAT1), pat0 = *gp_reg(s, p, R_PAT0);
    uint32_t out = ~in & msk & ~pat1;

    return (out & pat0) | (~out & s->input[p]);
}

static void gpio_update(IngenicGpioState *s, int p)
{
    uint32_t in = *gp_reg(s, p, R_INT), msk = *gp_reg(s, p, R_MSK);
    uint32_t pat1 = *gp_reg(s, p, R_PAT1), pat0 = *gp_reg(s, p, R_PAT0);
    uint32_t out = ~in & msk & ~pat1;
    uint32_t drive = (out & pat0) | ~out, changed = drive ^ s->drive[p];
    int i;

    qemu_set_irq(s->irq[p],
                 (*gp_reg(s, p, R_FLG) & ~*gp_reg(s, p, R_MSK)) != 0);
    s->drive[p] = drive;
    for (i = 0; changed && i < 32; i++) {
        if (changed & (1u << i)) {
            qemu_set_irq(s->drive_out[p * 32 + i], extract32(drive, i, 1));
        }
    }
}

/* Latch interrupt-pin events for a port whose input levels went old -> new */
static void gpio_events(IngenicGpioState *s, int p, uint32_t old, uint32_t new)
{
    uint32_t irqpin = *gp_reg(s, p, R_INT);
    uint32_t pat1 = *gp_reg(s, p, R_PAT1), pat0 = *gp_reg(s, p, R_PAT0);
    uint32_t level = ~pat1 & ((pat0 & new) | (~pat0 & ~new));
    uint32_t edge = pat1 & ((pat0 & ~old & new) | (~pat0 & old & ~new));

    *gp_reg(s, p, R_FLG) |= irqpin & (level | edge);
    gpio_update(s, p);
}

static void gpio_set_input(void *opaque, int n, int level)
{
    IngenicGpioState *s = opaque;
    int p = n / 32;
    uint32_t old = s->input[p];

    s->input[p] = deposit32(old, n % 32, 1, !!level);
    gpio_events(s, p, old, s->input[p]);
}

static uint64_t ingenic_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicGpioState *s = opaque;
    int p = off / s->stride;
    hwaddr r = off % s->stride;

    if (off >= s->shadow || p >= s->nports || r >= PORT_REGS) {
        return 0;
    }
    if (r == R_PIN) {
        return gpio_pin(s, p);
    }
    return *gp_reg(s, p, r & ~0xf);
}

static void gpio_shadow_write(IngenicGpioState *s, hwaddr r, uint32_t val)
{
    int i = (r >> 4) - 1;
    int p;

    if (r == R_GID2LD) {
        p = val;
        if (p >= s->nports) {
            return;
        }
        for (i = 0; i < 4; i++) {
            uint32_t *reg = gp_reg(s, p, R_INT + i * 0x10);

            *reg = (*reg | s->zset[i]) & ~s->zclr[i];
            s->zset[i] = s->zclr[i] = 0;
        }
        gpio_update(s, p);
        return;
    }
    if (i < 0 || i > 3) {
        return;
    }
    switch (r & 0xf) {
    case 0x4:
        s->zset[i] |= val;
        s->zclr[i] &= ~val;
        break;
    case 0x8:
        s->zclr[i] |= val;
        s->zset[i] &= ~val;
        break;
    }
}

static void ingenic_gpio_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicGpioState *s = opaque;
    int p = off / s->stride;
    hwaddr r = off % s->stride;
    uint32_t *reg;

    if (off >= s->shadow) {
        gpio_shadow_write(s, off - s->shadow, val);
        return;
    }
    if (p >= s->nports || r >= PORT_REGS || r == R_PIN) {
        return;
    }
    reg = gp_reg(s, p, r & ~0xf);
    switch (r & 0xf) {
    case 0x0:
        *reg = val;
        break;
    case 0x4:
        *reg |= val;
        break;
    case 0x8:
        *reg &= ~val;
        break;
    default:
        return;
    }
    gpio_update(s, p);
}

static const MemoryRegionOps ingenic_gpio_ops = {
    .read = ingenic_gpio_read,
    .write = ingenic_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_gpio_reset(DeviceState *dev)
{
    IngenicGpioState *s = INGENIC_GPIO(dev);
    int p;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->zset, 0, sizeof(s->zset));
    memset(s->zclr, 0, sizeof(s->zclr));
    for (p = 0; p < MAX_PORTS; p++) {
        /* out of reset every pin is a pulled GPIO input */
        *gp_reg(s, p, R_MSK) = 0xffffffff;
        *gp_reg(s, p, R_PAT1) = 0xffffffff;
        gpio_update(s, p);
    }
}

static void ingenic_gpio_realize(DeviceState *dev, Error **errp)
{
    IngenicGpioState *s = INGENIC_GPIO(dev);
    int p;

    if (s->nports > MAX_PORTS || s->stride < PORT_REGS / 2 ||
        s->shadow < s->nports * s->stride) {
        error_setg(errp, "ingenic-gpio: bad geometry");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &ingenic_gpio_ops, s,
                          TYPE_INGENIC_GPIO, s->shadow + 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    for (p = 0; p < s->nports; p++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[p]);
    }
    qdev_init_gpio_in(dev, gpio_set_input, s->nports * 32);
    qdev_init_gpio_out_named(dev, s->drive_out, "drive", s->nports * 32);
    for (p = 0; p < MAX_PORTS; p++) {
        s->input[p] = 0xffffffff;
        s->drive[p] = 0xffffffff;
    }
}

static const Property ingenic_gpio_props[] = {
    DEFINE_PROP_UINT32("num-ports", IngenicGpioState, nports, 4),
    DEFINE_PROP_UINT32("stride", IngenicGpioState, stride, 0x1000),
    DEFINE_PROP_UINT32("shadow", IngenicGpioState, shadow, 0x7000),
};

static const VMStateDescription vmstate_ingenic_gpio = {
    .name = TYPE_INGENIC_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, IngenicGpioState, MAX_PORTS,
                               PORT_REGS / 4),
        VMSTATE_UINT32_ARRAY(input, IngenicGpioState, MAX_PORTS),
        VMSTATE_UINT32_ARRAY(zset, IngenicGpioState, 4),
        VMSTATE_UINT32_ARRAY(zclr, IngenicGpioState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_gpio_realize;
    device_class_set_legacy_reset(dc, ingenic_gpio_reset);
    device_class_set_props(dc, ingenic_gpio_props);
    dc->vmsd = &vmstate_ingenic_gpio;
}

static const TypeInfo ingenic_gpio_info = {
    .name = TYPE_INGENIC_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicGpioState),
    .class_init = ingenic_gpio_class_init,
};

static void ingenic_gpio_register_types(void)
{
    type_register_static(&ingenic_gpio_info);
}

type_init(ingenic_gpio_register_types)
