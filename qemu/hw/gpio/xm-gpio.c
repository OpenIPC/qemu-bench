/*
 * Xiongmai XM530/XM550 pad controller, 0x10020000: one 32-bit register per
 * pad, which is both its pin mux and its GPIO.
 *
 *   bits 1-3   MUX1..MUX3 enable; all clear is plain GPIO
 *   bits 7:6   drive strength
 *   bit 10     output enable
 *   bit 11     output level
 *   bit 12     input enable
 *   bit 13     input level (read only)
 *
 * The layout is the vendor kernel's mach-xm530/include/mach/platform.h plus
 * what its kmotor.ko reads back: gpio_get_value() takes bit 13.  A lab XM550
 * populates pads 0..99; the words past them are control registers (0x3C0,
 * 0x3E0, the light-sensor ADC at 0x480) and stay plain RAM here.
 *
 * Bit 13 reads the pad: its driven level while output is enabled, otherwise
 * the level on its input line (qdev GPIO input N, for qtest and boards).
 * Each output change raises or lowers qdev GPIO output N and is logged under
 * -d unimp as "xm-gpio: pad N high/low", which is how a test sees which pads
 * the kernel, majestic or the vendor ISP actually drive.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qapi/error.h"

#define TYPE_XM_GPIO "xm-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(XmGpioState, XM_GPIO)

#define XM_GPIO_SIZE        0x10000
#define XM_GPIO_MAX_PADS    128

#define PAD_OUT_EN          (1u << 10)
#define PAD_OUT_HIGH        (1u << 11)
#define PAD_IN_EN           (1u << 12)
#define PAD_IN_LEVEL        (1u << 13)

struct XmGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_GPIO_SIZE / 4];
    uint32_t pads;
    bool in_level[XM_GPIO_MAX_PADS];
    qemu_irq out[XM_GPIO_MAX_PADS];
};

/* The level a driven pad shows, or -1 when it is not driven. */
static int xm_gpio_driven(uint32_t reg)
{
    return reg & PAD_OUT_EN ? !!(reg & PAD_OUT_HIGH) : -1;
}

static uint64_t xm_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    XmGpioState *s = XM_GPIO(opaque);
    const uint32_t pad = off / 4;
    uint32_t v = s->regs[pad];

    if (pad < s->pads) {
        const int driven = xm_gpio_driven(v);
        const bool level = driven >= 0 ? driven : s->in_level[pad];

        v = (v & ~PAD_IN_LEVEL) | (level ? PAD_IN_LEVEL : 0);
    }
    return v;
}

static void xm_gpio_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmGpioState *s = XM_GPIO(opaque);
    const uint32_t pad = off / 4;

    if (pad >= s->pads) {
        s->regs[pad] = val;
        return;
    }

    const int before = xm_gpio_driven(s->regs[pad]);

    s->regs[pad] = val & ~PAD_IN_LEVEL;

    const int after = xm_gpio_driven(s->regs[pad]);

    if (after != before && after >= 0) {
        qemu_log_mask(LOG_UNIMP, "xm-gpio: pad %u %s\n", pad,
                      after ? "high" : "low");
        qemu_set_irq(s->out[pad], after);
    } else if (after != before) {
        qemu_log_mask(LOG_UNIMP, "xm-gpio: pad %u released\n", pad);
        qemu_set_irq(s->out[pad], 0);
    }
}

static const MemoryRegionOps xm_gpio_ops = {
    .read = xm_gpio_read,
    .write = xm_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_gpio_set_input(void *opaque, int pad, int level)
{
    XmGpioState *s = XM_GPIO(opaque);

    if (pad >= 0 && (uint32_t)pad < s->pads) {
        s->in_level[pad] = level;
    }
}

static void xm_gpio_reset(DeviceState *dev)
{
    XmGpioState *s = XM_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void xm_gpio_realize(DeviceState *dev, Error **errp)
{
    XmGpioState *s = XM_GPIO(dev);

    if (!s->pads || s->pads > XM_GPIO_MAX_PADS) {
        error_setg(errp, "xm-gpio: pads must be 1..%d, not %u",
                   XM_GPIO_MAX_PADS, s->pads);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &xm_gpio_ops, s,
                          TYPE_XM_GPIO, XM_GPIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_in(dev, xm_gpio_set_input, s->pads);
    qdev_init_gpio_out(dev, s->out, s->pads);
}

static const Property xm_gpio_props[] = {
    DEFINE_PROP_UINT32("pads", XmGpioState, pads, 100),
};

static void xm_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_gpio_realize;
    device_class_set_legacy_reset(dc, xm_gpio_reset);
    device_class_set_props(dc, xm_gpio_props);
}

static const TypeInfo xm_gpio_info = {
    .name          = TYPE_XM_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmGpioState),
    .class_init    = xm_gpio_class_init,
};

static void xm_gpio_register_types(void)
{
    type_register_static(&xm_gpio_info);
}

type_init(xm_gpio_register_types)
