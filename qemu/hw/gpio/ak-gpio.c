/*
 * Anyka AK39xx GPIO controller.
 *
 * Up to 96 pins in three 32-bit banks, each register a bank every 4 bytes
 * (kernel mach-ak39 gpio.h, U-Boot drivers/gpio/ak39_gpio.c):
 *
 *   0x00 DIR (1 = output)   0x0C OUT   0x18 IN
 *   0x24 interrupt enable   0x30 mode (1 = edge)   0x3C polarity (1 = low /
 *   falling)                0x48 edge status
 *
 * Nothing drives the pins from outside, so an input reads what its pull-up
 * gives it, 1, unless input-low (a mask of pins 0..63) names it: that is
 * "button not pressed" and "no SD card" (both active low on the boards
 * seen).  The kernel's level interrupts are compared against that level;
 * edges never happen.
 *
 * Each pin is also an output line carrying its level, for things wired to
 * a GPIO: the EV330 kernels drive the boot flash's chip select on GPIO 25
 * (cs-gpios in its device tree).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_GPIO "ak-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(AkGpioState, AK_GPIO)

#define AK_GPIO_BANKS   3
#define AK_GPIO_PINS    (32 * AK_GPIO_BANKS)

enum {
    R_DIR = 0, R_OUT, R_IN, R_INTEN, R_MODE, R_POL, R_EDGE, R_NUM
};

struct AkGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[AK_GPIO_PINS];

    uint32_t regs[R_NUM][AK_GPIO_BANKS];
    uint64_t input_low;
};

static uint32_t ak_gpio_level(AkGpioState *s, int b)
{
    uint32_t in = ~(uint32_t)(s->input_low >> (32 * b));

    if (b == 2) {
        in = 0xFFFFFFFF;
    }
    return (s->regs[R_DIR][b] & s->regs[R_OUT][b]) |
           (~s->regs[R_DIR][b] & in);
}

static void ak_gpio_update(AkGpioState *s)
{
    bool level = false;
    int b, n;

    for (b = 0; b < AK_GPIO_BANKS; b++) {
        uint32_t v = ak_gpio_level(s, b);

        for (n = 0; n < 32; n++) {
            qemu_set_irq(s->out[32 * b + n], (v >> n) & 1);
        }
        uint32_t active = v ^ s->regs[R_POL][b];
        uint32_t pend = (~s->regs[R_MODE][b] & active) |
                        (s->regs[R_MODE][b] & s->regs[R_EDGE][b]);

        level |= !!(pend & s->regs[R_INTEN][b]);
    }
    qemu_set_irq(s->irq, level);
}

static uint64_t ak_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    AkGpioState *s = opaque;
    unsigned r = offset / 12, b = (offset % 12) / 4;

    if (r >= R_NUM) {
        qemu_log_mask(LOG_UNIMP, "ak-gpio: read of 0x%03" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
    if (r == R_IN) {
        return ak_gpio_level(s, b);
    }
    return s->regs[r][b];
}

static void ak_gpio_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    AkGpioState *s = opaque;
    unsigned r = offset / 12, b = (offset % 12) / 4;

    if (r >= R_NUM) {
        qemu_log_mask(LOG_UNIMP, "ak-gpio: write 0x%08" PRIx64 " to 0x%03"
                      HWADDR_PRIx "\n", val, offset);
        return;
    }
    switch (r) {
    case R_IN:
        return;
    case R_EDGE:
        s->regs[R_EDGE][b] &= ~val;
        break;
    default:
        s->regs[r][b] = val;
        break;
    }
    ak_gpio_update(s);
}

static const MemoryRegionOps ak_gpio_ops = {
    .read = ak_gpio_read,
    .write = ak_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_gpio_reset(DeviceState *dev)
{
    AkGpioState *s = AK_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    ak_gpio_update(s);
}

static void ak_gpio_init(Object *obj)
{
    AkGpioState *s = AK_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_gpio_ops, s, TYPE_AK_GPIO,
                          0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out(DEVICE(obj), s->out, AK_GPIO_PINS);
}

static const Property ak_gpio_properties[] = {
    DEFINE_PROP_UINT64("input-low", AkGpioState, input_low, 0),
};

static const VMStateDescription vmstate_ak_gpio = {
    .name = TYPE_AK_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, AkGpioState, R_NUM, AK_GPIO_BANKS),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_gpio;
    device_class_set_legacy_reset(dc, ak_gpio_reset);
    device_class_set_props(dc, ak_gpio_properties);
}

static const TypeInfo ak_gpio_info = {
    .name          = TYPE_AK_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkGpioState),
    .instance_init = ak_gpio_init,
    .class_init    = ak_gpio_class_init,
};

static void ak_gpio_register_types(void)
{
    type_register_static(&ak_gpio_info);
}

type_init(ak_gpio_register_types)
