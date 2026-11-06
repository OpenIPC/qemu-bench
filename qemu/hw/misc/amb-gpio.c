/*
 * Ambarella S3L GPIO: four banks of 32 lines, 128 in all.  Pads are numbered
 * through the banks, so pad n is bit n % 32 of bank n / 32.
 *
 * A bank is PL061-like.  AFSEL hands a pad to its peripheral, DIR makes it an
 * output, and a write to DATA changes only the bits set in MASK.  Interrupts
 * are not modelled: nothing here needs one, and the registers are storage.
 *
 * An output drives its named output line.  A pad not driven reads what the
 * board pulls it to, which is low: that is what holds the sensor in reset on
 * the IP4M-1026E until something releases GPIO 107.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_AMB_GPIO "amb-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(AmbGpioState, AMB_GPIO)

#define AMB_GPIO_BANKS  4
#define AMB_GPIO_PADS   (AMB_GPIO_BANKS * 32)

#define R_DATA          0x00
#define R_DIR           0x04
#define R_IS            0x08
#define R_IBE           0x0c
#define R_IEV           0x10
#define R_IE            0x14
#define R_AFSEL         0x18
#define R_RIS           0x1c
#define R_MIS           0x20
#define R_IC            0x24
#define R_MASK          0x28
#define R_ENABLE        0x2c

typedef struct AmbGpioBank {
    struct AmbGpioState *owner;
    MemoryRegion iomem;
    int index;
    uint32_t data;
    uint32_t dir;
    uint32_t is;
    uint32_t ibe;
    uint32_t iev;
    uint32_t ie;
    uint32_t afsel;
    uint32_t mask;
    uint32_t enable;
} AmbGpioBank;

struct AmbGpioState {
    SysBusDevice parent_obj;

    AmbGpioBank bank[AMB_GPIO_BANKS];
    qemu_irq out[AMB_GPIO_PADS];
};

static void amb_gpio_update(AmbGpioBank *b)
{
    uint32_t driven = b->data & b->dir & ~b->afsel;

    for (int i = 0; i < 32; i++) {
        qemu_set_irq(b->owner->out[b->index * 32 + i], (driven >> i) & 1);
    }
}

static uint64_t amb_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    AmbGpioBank *b = opaque;

    switch (off) {
    case R_DATA:
        return b->data & b->dir & b->mask;
    case R_DIR:
        return b->dir;
    case R_IS:
        return b->is;
    case R_IBE:
        return b->ibe;
    case R_IEV:
        return b->iev;
    case R_IE:
        return b->ie;
    case R_AFSEL:
        return b->afsel;
    case R_RIS:
    case R_MIS:
        return 0;
    case R_MASK:
        return b->mask;
    case R_ENABLE:
        return b->enable;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-gpio: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void amb_gpio_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    AmbGpioBank *b = opaque;

    switch (off) {
    case R_DATA:
        b->data = (b->data & ~b->mask) | (val & b->mask);
        break;
    case R_DIR:
        b->dir = val;
        break;
    case R_IS:
        b->is = val;
        break;
    case R_IBE:
        b->ibe = val;
        break;
    case R_IEV:
        b->iev = val;
        break;
    case R_IE:
        b->ie = val;
        break;
    case R_AFSEL:
        b->afsel = val;
        break;
    case R_IC:
        break;
    case R_MASK:
        b->mask = val;
        break;
    case R_ENABLE:
        b->enable = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-gpio: write 0x%" HWADDR_PRIx "\n", off);
        return;
    }
    amb_gpio_update(b);
}

static const MemoryRegionOps amb_gpio_ops = {
    .read = amb_gpio_read,
    .write = amb_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_gpio_reset(DeviceState *dev)
{
    AmbGpioState *s = AMB_GPIO(dev);

    for (int i = 0; i < AMB_GPIO_BANKS; i++) {
        AmbGpioBank *b = &s->bank[i];

        b->data = b->dir = b->is = b->ibe = b->iev = b->ie = 0;
        b->afsel = b->mask = b->enable = 0;
        amb_gpio_update(b);
    }
}

static void amb_gpio_init(Object *obj)
{
    AmbGpioState *s = AMB_GPIO(obj);

    for (int i = 0; i < AMB_GPIO_BANKS; i++) {
        s->bank[i].owner = s;
        s->bank[i].index = i;
        memory_region_init_io(&s->bank[i].iomem, obj, &amb_gpio_ops,
                              &s->bank[i], TYPE_AMB_GPIO, 0x1000);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank[i].iomem);
    }
    qdev_init_gpio_out(DEVICE(obj), s->out, AMB_GPIO_PADS);
}

static int amb_gpio_post_load(void *opaque, int version_id)
{
    AmbGpioState *s = opaque;

    for (int i = 0; i < AMB_GPIO_BANKS; i++) {
        amb_gpio_update(&s->bank[i]);
    }
    return 0;
}

static const VMStateDescription amb_gpio_bank_vmstate = {
    .name = "amb-gpio-bank",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(data, AmbGpioBank),
        VMSTATE_UINT32(dir, AmbGpioBank),
        VMSTATE_UINT32(is, AmbGpioBank),
        VMSTATE_UINT32(ibe, AmbGpioBank),
        VMSTATE_UINT32(iev, AmbGpioBank),
        VMSTATE_UINT32(ie, AmbGpioBank),
        VMSTATE_UINT32(afsel, AmbGpioBank),
        VMSTATE_UINT32(mask, AmbGpioBank),
        VMSTATE_UINT32(enable, AmbGpioBank),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription amb_gpio_vmstate = {
    .name = TYPE_AMB_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = amb_gpio_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(bank, AmbGpioState, AMB_GPIO_BANKS, 1,
                             amb_gpio_bank_vmstate, AmbGpioBank),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, amb_gpio_reset);
    dc->vmsd = &amb_gpio_vmstate;
}

static const TypeInfo amb_gpio_info = {
    .name = TYPE_AMB_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbGpioState),
    .instance_init = amb_gpio_init,
    .class_init = amb_gpio_class_init,
};

static void amb_gpio_register(void)
{
    type_register_static(&amb_gpio_info);
}

type_init(amb_gpio_register)
