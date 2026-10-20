/*
 * Novatek NA51055 / NA51089 GPIO controller.
 *
 * plat-na51089/nvt-gpio.h and drivers/gpio/gpio-nvt-na51089.c.  Pins are
 * numbered in groups of 32, one register word per group: C_GPIO at 0,
 * P 0x20, S 0x40, L 0x60, D 0x80, H 0xA0, A 0xC0, DSI 0xE0.
 *
 *   0x00..0x1C DATA   pad level (writes set the output latch)
 *   0x20..0x3C DIR    1 = output
 *   0x40..0x5C SET    write 1s to drive high
 *   0x60..0x7C CLR    write 1s to drive low
 *   0x80 STS_CPU      interrupt status, write 1 to clear
 *   0x90 INTEN_CPU    0xA0 INT_TYPE (1 level) 0xA4 INT_POL (1 low/falling)
 *   0xA8 EDGE_TYPE    1 = both edges
 *   0xC0..0xE8        the same for the eight DGPIO lines
 *
 * Only 40 pins can interrupt: status bit n stands for the pin in
 * nvt_gpio_irq_table[n] (bits 32..39 being DGPIO bits 0..7).  All of them
 * share one GIC line.
 *
 * GPIO lines, by pin number: "in" 0..255 drive the pads of input pins;
 * "out" 0..255 are the pad drivers, the latch while a pin is an output
 * and low while it is an input.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-core.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_GPIO "nvt-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(NvtGpioState, NVT_GPIO)

#define NVT_GPIO_BANKS      8
#define NVT_GPIO_PINS       (NVT_GPIO_BANKS * 32)
#define NVT_GPIO_INTS       40

#define GPIO_DATA           0x00
#define GPIO_DIR            0x20
#define GPIO_SET            0x40
#define GPIO_CLR            0x60
#define GPIO_STS            0x80
#define GPIO_INTEN          0x90
#define GPIO_INT_TYPE       0xA0
#define GPIO_INT_POL        0xA4
#define GPIO_EDGE_TYPE      0xA8
#define DGPIO_STS           0xC0
#define DGPIO_INTEN         0xD0
#define DGPIO_INT_TYPE      0xE0
#define DGPIO_INT_POL       0xE4
#define DGPIO_EDGE_TYPE     0xE8
#define GPIO_REGS_END       0x100

#define C_GPIO(p)   (p)
#define P_GPIO(p)   ((p) + 0x20)
#define S_GPIO(p)   ((p) + 0x40)
#define L_GPIO(p)   ((p) + 0x60)
#define D_GPIO(p)   ((p) + 0x80)
#define H_GPIO(p)   ((p) + 0xA0)
#define DSI_GPIO(p) ((p) + 0xE0)

/* gpio-nvt-na51089.c nvt_gpio_irq_table */
static const uint8_t nvt_gpio_int_pin[NVT_GPIO_INTS] = {
    C_GPIO(3),   C_GPIO(5),   C_GPIO(7),   C_GPIO(9),   C_GPIO(12),
    C_GPIO(14),  C_GPIO(16),  C_GPIO(18),  C_GPIO(20),  C_GPIO(22),
    H_GPIO(0),   H_GPIO(11),  S_GPIO(1),   S_GPIO(4),   S_GPIO(6),
    S_GPIO(8),   S_GPIO(2),   P_GPIO(3),   P_GPIO(7),   P_GPIO(8),
    P_GPIO(9),   P_GPIO(11),  P_GPIO(20),  P_GPIO(17),  P_GPIO(18),
    P_GPIO(24),  DSI_GPIO(1), DSI_GPIO(6), DSI_GPIO(10), L_GPIO(4),
    L_GPIO(8),   L_GPIO(0),   D_GPIO(0),   D_GPIO(1),   D_GPIO(2),
    D_GPIO(3),   D_GPIO(4),   D_GPIO(5),   D_GPIO(6),   D_GPIO(7),
};

struct NvtGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[NVT_GPIO_PINS];

    uint32_t latch[NVT_GPIO_BANKS];
    uint32_t dir[NVT_GPIO_BANKS];
    uint32_t input[NVT_GPIO_BANKS];
    uint32_t pad[NVT_GPIO_BANKS];       /* last level seen, for edges */
    uint32_t driven[NVT_GPIO_BANKS];    /* what the "out" lines carry */
    /* Interrupt words: [0] GPIO at 0x80.., [1] DGPIO at 0xC0.. */
    uint32_t regs[GPIO_REGS_END / 4];
};

static uint32_t nvt_gpio_level(NvtGpioState *s, int bank)
{
    return (s->latch[bank] & s->dir[bank]) | (s->input[bank] & ~s->dir[bank]);
}

static uint32_t *nvt_gpio_ireg(NvtGpioState *s, int n, hwaddr gpio_off)
{
    hwaddr off = n < 32 ? gpio_off : gpio_off - GPIO_STS + DGPIO_STS;

    return &s->regs[off / 4];
}

static void nvt_gpio_update_irq(NvtGpioState *s)
{
    bool level = (s->regs[GPIO_STS / 4] & s->regs[GPIO_INTEN / 4]) ||
                 (s->regs[DGPIO_STS / 4] & s->regs[DGPIO_INTEN / 4] & 0xFF);

    qemu_set_irq(s->irq, level);
}

/* Latch interrupt status from pad changes, then drive outputs. */
static void nvt_gpio_update(NvtGpioState *s)
{
    uint32_t level[NVT_GPIO_BANKS];
    int b, n;

    for (b = 0; b < NVT_GPIO_BANKS; b++) {
        level[b] = nvt_gpio_level(s, b);
    }

    for (n = 0; n < NVT_GPIO_INTS; n++) {
        const int pin = nvt_gpio_int_pin[n];
        const uint32_t bit = 1u << (n & 31);
        const bool now = level[pin / 32] & (1u << (pin % 32));
        const bool was = s->pad[pin / 32] & (1u << (pin % 32));
        const bool low = *nvt_gpio_ireg(s, n, GPIO_INT_POL) & bit;
        bool fire;

        if (*nvt_gpio_ireg(s, n, GPIO_INT_TYPE) & bit) {
            fire = now != low;
        } else if (*nvt_gpio_ireg(s, n, GPIO_EDGE_TYPE) & bit) {
            fire = now != was;
        } else {
            fire = now != was && now != low;
        }
        if (fire) {
            *nvt_gpio_ireg(s, n, GPIO_STS) |= bit;
        }
    }

    for (b = 0; b < NVT_GPIO_BANKS; b++) {
        const uint32_t drive = s->latch[b] & s->dir[b];
        const uint32_t changed = drive ^ s->driven[b];

        s->pad[b] = level[b];
        s->driven[b] = drive;
        for (n = 0; n < 32; n++) {
            if (changed & (1u << n)) {
                qemu_set_irq(s->out[b * 32 + n], (drive >> n) & 1);
            }
        }
    }
    nvt_gpio_update_irq(s);
}

static uint64_t nvt_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtGpioState *s = opaque;
    const int bank = (offset & 0x1F) / 4;

    switch (offset & ~0x1FULL) {
    case GPIO_DATA:
        return nvt_gpio_level(s, bank);
    case GPIO_DIR:
        return s->dir[bank];
    case GPIO_SET:
    case GPIO_CLR:
        return 0;
    default:
        return offset < GPIO_REGS_END ? s->regs[offset / 4] : 0;
    }
}

static void nvt_gpio_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    NvtGpioState *s = opaque;
    const int bank = (offset & 0x1F) / 4;

    switch (offset & ~0x1FULL) {
    case GPIO_DATA:
        s->latch[bank] = val;
        break;
    case GPIO_DIR:
        s->dir[bank] = val;
        break;
    case GPIO_SET:
        s->latch[bank] |= val;
        break;
    case GPIO_CLR:
        s->latch[bank] &= ~val;
        break;
    default:
        if (offset >= GPIO_REGS_END) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-gpio: write 0x%08" PRIx64
                          " to 0x%03" HWADDR_PRIx "\n", val, offset);
            return;
        }
        if (offset == GPIO_STS || offset == DGPIO_STS) {
            s->regs[offset / 4] &= ~val;
        } else {
            s->regs[offset / 4] = val;
        }
        break;
    }
    nvt_gpio_update(s);
}

static const MemoryRegionOps nvt_gpio_ops = {
    .read = nvt_gpio_read,
    .write = nvt_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_gpio_set_input(void *opaque, int line, int level)
{
    NvtGpioState *s = opaque;
    const uint32_t bit = 1u << (line % 32);

    if (level) {
        s->input[line / 32] |= bit;
    } else {
        s->input[line / 32] &= ~bit;
    }
    nvt_gpio_update(s);
}

static void nvt_gpio_reset(DeviceState *dev)
{
    NvtGpioState *s = NVT_GPIO(dev);
    int n;

    /* Every pin comes out of reset an input: release what was driven. */
    for (n = 0; n < NVT_GPIO_PINS; n++) {
        if (s->driven[n / 32] & (1u << (n % 32))) {
            qemu_irq_lower(s->out[n]);
        }
    }
    memset(s->driven, 0, sizeof(s->driven));
    memset(s->latch, 0, sizeof(s->latch));
    memset(s->dir, 0, sizeof(s->dir));
    memset(s->regs, 0, sizeof(s->regs));
    memcpy(s->pad, s->input, sizeof(s->pad));
    qemu_irq_lower(s->irq);
}

static void nvt_gpio_init(Object *obj)
{
    NvtGpioState *s = NVT_GPIO(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_gpio_ops, s, TYPE_NVT_GPIO,
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(dev, nvt_gpio_set_input, "in", NVT_GPIO_PINS);
    qdev_init_gpio_out_named(dev, s->out, "out", NVT_GPIO_PINS);
}

static const VMStateDescription vmstate_nvt_gpio = {
    .name = TYPE_NVT_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(latch, NvtGpioState, NVT_GPIO_BANKS),
        VMSTATE_UINT32_ARRAY(dir, NvtGpioState, NVT_GPIO_BANKS),
        VMSTATE_UINT32_ARRAY(input, NvtGpioState, NVT_GPIO_BANKS),
        VMSTATE_UINT32_ARRAY(pad, NvtGpioState, NVT_GPIO_BANKS),
        VMSTATE_UINT32_ARRAY(driven, NvtGpioState, NVT_GPIO_BANKS),
        VMSTATE_UINT32_ARRAY(regs, NvtGpioState, GPIO_REGS_END / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_gpio_reset);
    dc->vmsd = &vmstate_nvt_gpio;
}

static const TypeInfo nvt_gpio_info = {
    .name = TYPE_NVT_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtGpioState),
    .instance_init = nvt_gpio_init,
    .class_init = nvt_gpio_class_init,
};

static void nvt_gpio_register_types(void)
{
    type_register_static(&nvt_gpio_info);
}

type_init(nvt_gpio_register_types)
