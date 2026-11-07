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
 * Two pads can carry a bit-banged I2C bus ("i2c-scl" / "i2c-sda"), open
 * drain with pull-ups: a pad not driven is high unless a slave on the
 * child bus "i2c" holds SDA low.  XM8536E boards hang an AT24C16 there,
 * which at24c.ko checks at load.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/bitbang_i2c.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "qapi/error.h"

#define TYPE_XM_GPIO "xm-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(XmGpioState, XM_GPIO)

#define XM_GPIO_SIZE        0x10000
#define XM_GPIO_MAX_PADS    256

#define PAD_OUT_EN          (1u << 10)
#define PAD_OUT_HIGH        (1u << 11)
#define PAD_IN_EN           (1u << 12)
#define PAD_IN_LEVEL        (1u << 13)

struct XmGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_GPIO_SIZE / 4];
    uint32_t pads;
    uint32_t adc_off;           /* 0, or the ADC data word: reads full scale */
    bool in_level[XM_GPIO_MAX_PADS];
    qemu_irq out[XM_GPIO_MAX_PADS];
    uint32_t i2c_scl, i2c_sda;  /* bit-banged I2C pads, or UINT32_MAX */
    I2CBus *i2c;
    bitbang_i2c_interface bb;
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

    /* The ADC the vendor adc.ko reads (XM533 / XM535: select +0xCC0, data
     * +0xCC4) is where Sofia's ADC keys sit -- the reset key among them,
     * pressed at 0.  Nothing pulls it down here: full scale, released. */
    if (s->adc_off && off == s->adc_off) {
        return 0x3ff;
    }
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

    if (s->i2c && (pad == s->i2c_scl || pad == s->i2c_sda) &&
        after != before) {
        /* open drain: released is the pull-up; the slave's SDA comes back */
        const int line = pad == s->i2c_scl ? BITBANG_I2C_SCL : BITBANG_I2C_SDA;
        const int sda = bitbang_i2c_set(&s->bb, line, after != 0);

        s->in_level[s->i2c_sda] = sda;
        s->in_level[s->i2c_scl] = 1;
        return;
    }

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
    uint32_t pad;

    memset(s->regs, 0, sizeof(s->regs));
    /* Every pad comes out of reset undriven, so whatever its output line
     * was holding drops with it. */
    for (pad = 0; pad < s->pads; pad++) {
        qemu_set_irq(s->out[pad], 0);
    }
    if (s->i2c) {
        s->in_level[s->i2c_scl] = 1;
        s->in_level[s->i2c_sda] = 1;
    }
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
    if (s->i2c_scl < s->pads && s->i2c_sda < s->pads) {
        s->i2c = i2c_init_bus(dev, "i2c");
        bitbang_i2c_init(&s->bb, s->i2c);
        s->in_level[s->i2c_scl] = 1;
        s->in_level[s->i2c_sda] = 1;
    }
}

static const Property xm_gpio_props[] = {
    DEFINE_PROP_UINT32("pads", XmGpioState, pads, 100),
    DEFINE_PROP_UINT32("adc-offset", XmGpioState, adc_off, 0),
    DEFINE_PROP_UINT32("i2c-scl", XmGpioState, i2c_scl, UINT32_MAX),
    DEFINE_PROP_UINT32("i2c-sda", XmGpioState, i2c_sda, UINT32_MAX),
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
