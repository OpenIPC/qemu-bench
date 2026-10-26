/*
 * Grain Media FTGPIO010 GPIO controller.
 *
 * Faraday's 32-pin GPIO block, twice on the GM813x (0x91000000 and
 * 0x91900000; drivers/gpio/gpio-ftgpio010.c of the GM 3.3 kernel):
 *
 *   0x00 data out    0x04 data in      0x08 direction (1 = out)
 *   0x0C bypass      0x10 set          0x14 clear
 *   0x18 pull enable 0x1C pull type    0x20 interrupt enable
 *   0x24 raw status  0x28 masked       0x2C mask       0x30 clear
 *   0x34 trigger (1 = level)           0x38 both edges
 *   0x3C 1 = falling edge / low level  0x40 debounce   0x44 prescale
 *   0x48 revision
 *
 * Pins read their output when driven and their input otherwise.
 * Undriven inputs read "input-default" (pulled high), as on the boards,
 * where buttons such as reset short a pull-up to ground; the "in" GPIO
 * inputs move them.  Outputs leave as GPIO outputs (U-Boot pulses GPIO1
 * pin 18 to reset the Ethernet PHY).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_GPIO "gm-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(GmGpioState, GM_GPIO)

#define GM_GPIO_SIZE        0x1000
#define GM_GPIO_PINS        32

#define R_DATA_OUT          0x00
#define R_DATA_IN           0x04
#define R_DIR               0x08
#define R_BYPASS            0x0C
#define R_DATA_SET          0x10
#define R_DATA_CLR          0x14
#define R_PULL_EN           0x18
#define R_PULL_TYPE         0x1C
#define R_INT_EN            0x20
#define R_INT_RAW           0x24
#define R_INT_MASKED        0x28
#define R_INT_MASK          0x2C
#define R_INT_CLR           0x30
#define R_INT_TRIG          0x34
#define R_INT_BOTH          0x38
#define R_INT_RISENEG       0x3C
#define R_BOUNCE_EN         0x40
#define R_BOUNCE_PRE        0x44
#define R_REVISION          0x48

struct GmGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[GM_GPIO_PINS];

    uint32_t dout;
    uint32_t dir;
    uint32_t bypass;
    uint32_t pull_en;
    uint32_t pull_type;
    uint32_t int_en;
    uint32_t int_raw;
    uint32_t int_mask;
    uint32_t int_trig;
    uint32_t int_both;
    uint32_t int_riseneg;
    uint32_t bounce_en;
    uint32_t bounce_pre;
    uint32_t input;
    uint32_t input_default;
};

static uint32_t gm_gpio_pins(GmGpioState *s)
{
    return (s->dout & s->dir) | (s->input & ~s->dir);
}

static void gm_gpio_update(GmGpioState *s)
{
    uint32_t pins = gm_gpio_pins(s);
    /* level-triggered inputs follow the pin */
    uint32_t level = s->int_trig & (pins ^ s->int_riseneg);

    s->int_raw = (s->int_raw & ~s->int_trig) | level;
    qemu_set_irq(s->irq, (s->int_raw & s->int_en & ~s->int_mask) != 0);
}

/*
 * Update the output lines: an output pin drives its level, a pin that is
 * (or turned) an input drives low.
 */
static void gm_gpio_drive(GmGpioState *s, uint32_t old_pins, uint32_t old_dir)
{
    const uint32_t was = old_pins & old_dir;
    const uint32_t now = gm_gpio_pins(s) & s->dir;
    const uint32_t changed = (was ^ now) | (old_dir ^ s->dir);
    int i;

    for (i = 0; i < GM_GPIO_PINS; i++) {
        if (changed & (1u << i)) {
            qemu_set_irq(s->out[i], !!(now & (1u << i)));
        }
    }
}

static void gm_gpio_set_input(void *opaque, int n, int level)
{
    GmGpioState *s = opaque;
    uint32_t bit = 1u << n;
    uint32_t old = gm_gpio_pins(s);
    uint32_t edge;

    if (level) {
        s->input |= bit;
    } else {
        s->input &= ~bit;
    }
    edge = (old ^ gm_gpio_pins(s)) & bit & ~s->dir & ~s->int_trig;
    if (edge) {
        bool rising = !!level;

        if ((s->int_both & bit) ||
            (rising == !(s->int_riseneg & bit))) {
            s->int_raw |= bit;
        }
    }
    gm_gpio_update(s);
}

static uint64_t gm_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    GmGpioState *s = opaque;

    switch (off) {
    case R_DATA_OUT:
        return s->dout;
    case R_DATA_IN:
        return gm_gpio_pins(s);
    case R_DIR:
        return s->dir;
    case R_BYPASS:
        return s->bypass;
    case R_PULL_EN:
        return s->pull_en;
    case R_PULL_TYPE:
        return s->pull_type;
    case R_INT_EN:
        return s->int_en;
    case R_INT_RAW:
        return s->int_raw;
    case R_INT_MASKED:
        return s->int_raw & s->int_en & ~s->int_mask;
    case R_INT_MASK:
        return s->int_mask;
    case R_INT_TRIG:
        return s->int_trig;
    case R_INT_BOTH:
        return s->int_both;
    case R_INT_RISENEG:
        return s->int_riseneg;
    case R_BOUNCE_EN:
        return s->bounce_en;
    case R_BOUNCE_PRE:
        return s->bounce_pre;
    case R_REVISION:
        return 0x0001000F;
    }
    return 0;
}

static void gm_gpio_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmGpioState *s = opaque;
    uint32_t old = gm_gpio_pins(s), old_dir = s->dir;

    switch (off) {
    case R_DATA_OUT:
        s->dout = val;
        break;
    case R_DIR:
        s->dir = val;
        break;
    case R_BYPASS:
        s->bypass = val;
        break;
    case R_DATA_SET:
        s->dout |= val;
        break;
    case R_DATA_CLR:
        s->dout &= ~val;
        break;
    case R_PULL_EN:
        s->pull_en = val;
        break;
    case R_PULL_TYPE:
        s->pull_type = val;
        break;
    case R_INT_EN:
        s->int_en = val;
        break;
    case R_INT_MASK:
        s->int_mask = val;
        break;
    case R_INT_CLR:
        s->int_raw &= ~(val & ~s->int_trig);
        break;
    case R_INT_TRIG:
        s->int_trig = val;
        break;
    case R_INT_BOTH:
        s->int_both = val;
        break;
    case R_INT_RISENEG:
        s->int_riseneg = val;
        break;
    case R_BOUNCE_EN:
        s->bounce_en = val;
        break;
    case R_BOUNCE_PRE:
        s->bounce_pre = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, off);
        return;
    }
    gm_gpio_drive(s, old, old_dir);
    gm_gpio_update(s);
}

static const MemoryRegionOps gm_gpio_ops = {
    .read = gm_gpio_read,
    .write = gm_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_gpio_reset(DeviceState *dev)
{
    GmGpioState *s = GM_GPIO(dev);
    const uint32_t old = gm_gpio_pins(s), old_dir = s->dir;

    s->dout = 0;
    s->dir = 0;
    s->bypass = 0;
    s->pull_en = 0;
    s->pull_type = 0;
    s->int_en = 0;
    s->int_raw = 0;
    s->int_mask = 0;
    s->int_trig = 0;
    s->int_both = 0;
    s->int_riseneg = 0;
    s->bounce_en = 0;
    s->bounce_pre = 0;
    s->input = s->input_default;
    gm_gpio_drive(s, old, old_dir);         /* every line released */
    gm_gpio_update(s);
}

static void gm_gpio_realize(DeviceState *dev, Error **errp)
{
    GmGpioState *s = GM_GPIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_gpio_ops, s,
                          TYPE_GM_GPIO, GM_GPIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, gm_gpio_set_input, "in", GM_GPIO_PINS);
    qdev_init_gpio_out(dev, s->out, GM_GPIO_PINS);
}

static const Property gm_gpio_props[] = {
    DEFINE_PROP_UINT32("input-default", GmGpioState, input_default,
                       0xFFFFFFFF),
};

static const VMStateDescription vmstate_gm_gpio = {
    .name = TYPE_GM_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dout, GmGpioState),
        VMSTATE_UINT32(dir, GmGpioState),
        VMSTATE_UINT32(bypass, GmGpioState),
        VMSTATE_UINT32(pull_en, GmGpioState),
        VMSTATE_UINT32(pull_type, GmGpioState),
        VMSTATE_UINT32(int_en, GmGpioState),
        VMSTATE_UINT32(int_raw, GmGpioState),
        VMSTATE_UINT32(int_mask, GmGpioState),
        VMSTATE_UINT32(int_trig, GmGpioState),
        VMSTATE_UINT32(int_both, GmGpioState),
        VMSTATE_UINT32(int_riseneg, GmGpioState),
        VMSTATE_UINT32(bounce_en, GmGpioState),
        VMSTATE_UINT32(bounce_pre, GmGpioState),
        VMSTATE_UINT32(input, GmGpioState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_gpio_realize;
    device_class_set_legacy_reset(dc, gm_gpio_reset);
    device_class_set_props(dc, gm_gpio_props);
    dc->vmsd = &vmstate_gm_gpio;
}

static const TypeInfo gm_gpio_info = {
    .name           = TYPE_GM_GPIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmGpioState),
    .class_init     = gm_gpio_class_init,
};

static void gm_gpio_register_types(void)
{
    type_register_static(&gm_gpio_info);
}

type_init(gm_gpio_register_types)
