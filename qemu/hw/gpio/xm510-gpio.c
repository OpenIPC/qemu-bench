/*
 * Xiongmai XM510 GPIO and pin mux, 0x10020000: banked, eight pins a port,
 * unlike XM530's register per pad (xm-gpio).
 *
 *   +P*4                 mux word of port P: pin b is a GPIO while bits b,
 *                        8+b and 16+b (its three functions) are all clear
 *   +0x40 + P*0x10 + 0   data: writes latch the output levels, reads give
 *                        the pins -- the latched level where the output is
 *                        enabled, the input line elsewhere
 *   +0x40 + P*0x10 + 4   output enable, a bit a pin (1 = output)
 *
 * The layout is the vendor's sdio1_sd.c / sdio2_wifi.c helpers plus a
 * register dump of a lab XM510 at rest (port 8 drives pin 1 high, everything
 * else is an input). There are nine ports, 0..8; the vendor helpers
 * themselves only take 0..6. +0xD0, just past port 8, is not a port: the
 * xmmac driver sets its bit 0 for 100 Mb/s (mac_speed_o). What +8 and +C of
 * a port do is not known: they, +0xD0 and everything past them stay plain
 * RAM.
 *
 * Pin b of port P is line P*8 + b. The input line's level is qdev GPIO input
 * N (for qtest and boards). A change in what a pin drives raises or lowers
 * qdev GPIO output N and is logged under -d unimp as "xm510-gpio: gpio N
 * high/low/released"; a pin drives only while it is an enabled output and
 * not muxed to a function.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qapi/error.h"

#define TYPE_XM510_GPIO "xm510-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(Xm510GpioState, XM510_GPIO)

#define XM510_GPIO_SIZE     0x10000
#define XM510_GPIO_PORTS    9
#define XM510_GPIO_LINES    (XM510_GPIO_PORTS * 8)

#define MUX(p)              ((p) * 4)
#define GROUP(p)            (0x40 + (p) * 0x10)
#define GROUP_DATA          0x0
#define GROUP_OE            0x4

struct Xm510GpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM510_GPIO_SIZE / 4];
    uint8_t in_level[XM510_GPIO_PORTS];
    /* What each line drives now: -1 undriven, else 0/1. */
    int8_t driven[XM510_GPIO_LINES];
    qemu_irq out[XM510_GPIO_LINES];
};

static uint32_t reg(Xm510GpioState *s, hwaddr off)
{
    return s->regs[off / 4];
}

static bool is_gpio(Xm510GpioState *s, int port, int bit)
{
    const uint32_t fn = (1u << bit) | (1u << (8 + bit)) | (1u << (16 + bit));

    return !(reg(s, MUX(port)) & fn);
}

static int line_driven(Xm510GpioState *s, int port, int bit)
{
    if (!is_gpio(s, port, bit) ||
        !(reg(s, GROUP(port) + GROUP_OE) & (1u << bit))) {
        return -1;
    }
    return !!(reg(s, GROUP(port) + GROUP_DATA) & (1u << bit));
}

/* After any write that can change what a pin drives. */
static void xm510_gpio_update(Xm510GpioState *s, int port)
{
    for (int bit = 0; bit < 8; bit++) {
        const int n = port * 8 + bit;
        const int now = line_driven(s, port, bit);

        if (now == s->driven[n]) {
            continue;
        }
        s->driven[n] = now;
        if (now >= 0) {
            qemu_log_mask(LOG_UNIMP, "xm510-gpio: gpio %d %s\n", n,
                          now ? "high" : "low");
        } else {
            qemu_log_mask(LOG_UNIMP, "xm510-gpio: gpio %d released\n", n);
        }
        qemu_set_irq(s->out[n], now > 0);
    }
}

/* The port a data or output-enable word belongs to, or -1. */
static int group_port(hwaddr off, hwaddr *field)
{
    if (off < GROUP(0) || off >= GROUP(XM510_GPIO_PORTS)) {
        return -1;
    }
    *field = (off - GROUP(0)) % 0x10;
    return (off - GROUP(0)) / 0x10;
}

static uint64_t xm510_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    Xm510GpioState *s = XM510_GPIO(opaque);
    hwaddr field;
    const int port = group_port(off, &field);

    if (port >= 0 && field == GROUP_DATA) {
        const uint32_t oe = reg(s, GROUP(port) + GROUP_OE) & 0xff;

        return (reg(s, off) & ~0xffu) | (reg(s, off) & oe) |
               (s->in_level[port] & ~oe & 0xff);
    }
    return reg(s, off);
}

static void xm510_gpio_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    Xm510GpioState *s = XM510_GPIO(opaque);
    hwaddr field;
    int port = group_port(off, &field);

    s->regs[off / 4] = val;
    if (port < 0 && off < MUX(XM510_GPIO_PORTS)) {
        port = off / 4;                         /* a mux word */
    }
    if (port >= 0) {
        xm510_gpio_update(s, port);
    }
}

static const MemoryRegionOps xm510_gpio_ops = {
    .read = xm510_gpio_read,
    .write = xm510_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm510_gpio_set_input(void *opaque, int n, int level)
{
    Xm510GpioState *s = XM510_GPIO(opaque);

    if (n < 0 || n >= XM510_GPIO_LINES) {
        return;
    }
    if (level) {
        s->in_level[n / 8] |= 1u << (n % 8);
    } else {
        s->in_level[n / 8] &= ~(1u << (n % 8));
    }
}

static void xm510_gpio_reset(DeviceState *dev)
{
    Xm510GpioState *s = XM510_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* Every pin comes out of reset an undriven input, so whatever its output
     * line was holding drops with it -- logged like any other release, so a
     * reset in the middle of a drive is visible in the trace. */
    for (int n = 0; n < XM510_GPIO_LINES; n++) {
        if (s->driven[n] >= 0) {
            qemu_log_mask(LOG_UNIMP, "xm510-gpio: gpio %d released\n", n);
        }
        s->driven[n] = -1;
        qemu_set_irq(s->out[n], 0);
    }
}

static void xm510_gpio_realize(DeviceState *dev, Error **errp)
{
    Xm510GpioState *s = XM510_GPIO(dev);

    /* Undriven before the first reset, too: the reset reads it. */
    memset(s->driven, -1, sizeof(s->driven));
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm510_gpio_ops, s,
                          TYPE_XM510_GPIO, XM510_GPIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_in(dev, xm510_gpio_set_input, XM510_GPIO_LINES);
    qdev_init_gpio_out(dev, s->out, XM510_GPIO_LINES);
}

static void xm510_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm510_gpio_realize;
    device_class_set_legacy_reset(dc, xm510_gpio_reset);
}

static const TypeInfo xm510_gpio_info = {
    .name          = TYPE_XM510_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Xm510GpioState),
    .class_init    = xm510_gpio_class_init,
};

static void xm510_gpio_register_types(void)
{
    type_register_static(&xm510_gpio_info);
}

type_init(xm510_gpio_register_types)
