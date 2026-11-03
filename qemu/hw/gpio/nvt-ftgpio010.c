/*
 * Faraday FTGPIO010, as on Novatek's NA51068 (NT98321 / NT98323): four
 * banks of 32 lines at 0xFE400000, 0xFE420000, 0xFE440000, 0xFE640000
 * (SPI 31 / 54 / 55 / 87), drivers/gpio/gpio-ftgpio010.c:
 *
 *   0x00 DATAOUT       0x04 DATAIN     reads the pins: outputs read back
 *                                      what they drive, inputs their level
 *   0x08 PINDIR        1 = output      0x0C PINBYPASS
 *   0x10 DATASET       0x14 DATACLEAR  write 1 to set / clear DATAOUT bits
 *   0x18 PULLENABLE    0x1C PULLTYPE   storage
 *   0x20 INTRENABLE
 *   0x24 INTRRAWSTATE  enabled lines that have triggered
 *   0x28 INTRMASKEDSTATE  RAW & ~MASK, what drives the bank's interrupt
 *   0x2C INTRMASK      1 = masked      0x30 INTRCLEAR  write 1 to clear
 *   0x34 INTRTRIGGER   0 edge, 1 level
 *   0x38 INTRBOTH      1 = both edges  0x3C INTRRISENEG 0 rising / high,
 *                                      1 falling / low
 *   0x40 BOUNCEENABLE  0x44 BOUNCEPRESCALE  storage (no debounce here)
 *   0x7C REVISION
 *
 * Lines are qdev GPIOs: inputs 0..31 are the pins as driven from outside
 * (they read as pulled up, 1, until something drives them), outputs 0..31
 * follow DATAOUT on output pins.  A level interrupt follows the pin; an
 * edge one latches until INTRCLEAR.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_FTGPIO010 "nvt-ftgpio010"
OBJECT_DECLARE_SIMPLE_TYPE(NvtFtgpioState, NVT_FTGPIO010)

#define FTGPIO_DATAOUT      0x00
#define FTGPIO_DATAIN       0x04
#define FTGPIO_PINDIR       0x08
#define FTGPIO_PINBYPASS    0x0C
#define FTGPIO_DATASET      0x10
#define FTGPIO_DATACLEAR    0x14
#define FTGPIO_PULLENABLE   0x18
#define FTGPIO_PULLTYPE     0x1C
#define FTGPIO_INTRENABLE   0x20
#define FTGPIO_INTRRAW      0x24
#define FTGPIO_INTRMASKED   0x28
#define FTGPIO_INTRMASK     0x2C
#define FTGPIO_INTRCLEAR    0x30
#define FTGPIO_INTRTRIGGER  0x34
#define FTGPIO_INTRBOTH     0x38
#define FTGPIO_INTRRISENEG  0x3C
#define FTGPIO_BOUNCEENABLE 0x40
#define FTGPIO_BOUNCEPRESC  0x44
#define FTGPIO_REVISION     0x7C
#define FTGPIO_NREGS        (0x80 / 4)

#define FTGPIO_LINES        32
#define FTGPIO_REV          0x00010A00  /* FTGPIO010 1.10 */

struct NvtFtgpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[FTGPIO_LINES];
    uint32_t regs[FTGPIO_NREGS];
    uint32_t in;            /* the pins as driven from outside */
    uint32_t latched;       /* edge-triggered status */
    uint32_t last;          /* pin levels at the last update */
};

static uint32_t nvt_ftgpio_pins(NvtFtgpioState *s)
{
    uint32_t dir = s->regs[FTGPIO_PINDIR / 4];

    return (s->regs[FTGPIO_DATAOUT / 4] & dir) | (s->in & ~dir);
}

static uint32_t nvt_ftgpio_raw(NvtFtgpioState *s)
{
    uint32_t pins = nvt_ftgpio_pins(s);
    uint32_t level = s->regs[FTGPIO_INTRTRIGGER / 4];
    uint32_t neg = s->regs[FTGPIO_INTRRISENEG / 4];
    uint32_t active = (pins & ~neg) | (~pins & neg);

    return ((active & level) | (s->latched & ~level)) &
           s->regs[FTGPIO_INTRENABLE / 4];
}

static void nvt_ftgpio_update(NvtFtgpioState *s)
{
    uint32_t pins = nvt_ftgpio_pins(s);
    uint32_t changed = pins ^ s->last;
    uint32_t edge = ~s->regs[FTGPIO_INTRTRIGGER / 4];
    uint32_t both = s->regs[FTGPIO_INTRBOTH / 4];
    uint32_t neg = s->regs[FTGPIO_INTRRISENEG / 4];
    uint32_t rose = changed & pins, fell = changed & ~pins;
    uint32_t hit = (rose & (both | ~neg)) | (fell & (both | neg));
    uint32_t dir = s->regs[FTGPIO_PINDIR / 4];
    int n;

    s->latched |= hit & edge & s->regs[FTGPIO_INTRENABLE / 4];
    for (n = 0; n < FTGPIO_LINES; n++) {
        if ((changed & dir) & (1u << n)) {
            qemu_set_irq(s->out[n], (pins >> n) & 1);
        }
    }
    s->last = pins;
    qemu_set_irq(s->irq, !!(nvt_ftgpio_raw(s) &
                            ~s->regs[FTGPIO_INTRMASK / 4]));
}

static void nvt_ftgpio_set(void *opaque, int line, int level)
{
    NvtFtgpioState *s = opaque;

    s->in = deposit32(s->in, line, 1, level != 0);
    nvt_ftgpio_update(s);
}

static uint64_t nvt_ftgpio_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtFtgpioState *s = opaque;

    switch (offset) {
    case FTGPIO_DATAIN:
        return nvt_ftgpio_pins(s);
    case FTGPIO_INTRRAW:
        return nvt_ftgpio_raw(s);
    case FTGPIO_INTRMASKED:
        return nvt_ftgpio_raw(s) & ~s->regs[FTGPIO_INTRMASK / 4];
    case FTGPIO_DATASET:
    case FTGPIO_DATACLEAR:
    case FTGPIO_INTRCLEAR:
        return 0;
    case FTGPIO_REVISION:
        return FTGPIO_REV;
    default:
        if (offset < FTGPIO_NREGS * 4) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftgpio010: read 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void nvt_ftgpio_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    NvtFtgpioState *s = opaque;
    uint32_t old_level = s->regs[FTGPIO_INTRTRIGGER / 4];

    switch (offset) {
    case FTGPIO_DATASET:
        s->regs[FTGPIO_DATAOUT / 4] |= val;
        break;
    case FTGPIO_DATACLEAR:
        s->regs[FTGPIO_DATAOUT / 4] &= ~val;
        break;
    case FTGPIO_INTRCLEAR:
        s->latched &= ~val;
        break;
    case FTGPIO_DATAIN:
    case FTGPIO_INTRRAW:
    case FTGPIO_INTRMASKED:
    case FTGPIO_REVISION:
        return;
    default:
        if (offset >= FTGPIO_NREGS * 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftgpio010: write 0x%"
                          HWADDR_PRIx "\n", offset);
            return;
        }
        s->regs[offset / 4] = val;
        if (offset == FTGPIO_INTRTRIGGER) {
            /* a line moved to level drops what it latched as an edge */
            s->latched &= ~(val & ~old_level);
        }
        break;
    }
    nvt_ftgpio_update(s);
}

static const MemoryRegionOps nvt_ftgpio_ops = {
    .read = nvt_ftgpio_read,
    .write = nvt_ftgpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_ftgpio_reset(DeviceState *dev)
{
    NvtFtgpioState *s = NVT_FTGPIO010(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->latched = 0;
    s->last = nvt_ftgpio_pins(s);
    qemu_irq_lower(s->irq);
}

static void nvt_ftgpio_init(Object *obj)
{
    NvtFtgpioState *s = NVT_FTGPIO010(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ftgpio_ops, s,
                          TYPE_NVT_FTGPIO010, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in(DEVICE(obj), nvt_ftgpio_set, FTGPIO_LINES);
    qdev_init_gpio_out(DEVICE(obj), s->out, FTGPIO_LINES);
    s->in = 0xFFFFFFFF;     /* pulled up until driven */
}

static const VMStateDescription vmstate_nvt_ftgpio = {
    .name = TYPE_NVT_FTGPIO010,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtFtgpioState, FTGPIO_NREGS),
        VMSTATE_UINT32(in, NvtFtgpioState),
        VMSTATE_UINT32(latched, NvtFtgpioState),
        VMSTATE_UINT32(last, NvtFtgpioState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_ftgpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_ftgpio_reset);
    dc->vmsd = &vmstate_nvt_ftgpio;
}

static const TypeInfo nvt_ftgpio_info = {
    .name          = TYPE_NVT_FTGPIO010,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtFtgpioState),
    .instance_init = nvt_ftgpio_init,
    .class_init    = nvt_ftgpio_class_init,
};

static void nvt_ftgpio_register_types(void)
{
    type_register_static(&nvt_ftgpio_info);
}

type_init(nvt_ftgpio_register_types)
