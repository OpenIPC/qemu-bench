/*
 * Rockchip GPIO bank, version 2 ("rockchip,gpio-bank", ID 0x01000C2B and
 * later: RV1106 GPIO0 at 0xFF380000, GPIO1..4 at 0xFF530000 + n * 0x10000).
 *
 * Register file, from the kernel's drivers/gpio/gpio-rockchip.c:
 *
 *   0x00 DR        output level          0x08 DDR        1 = output
 *   0x10 INT_EN    0x18 INT_MASK         0x20 INT_TYPE   1 = edge
 *   0x28 INT_POLARITY  1 = high / rising 0x30 INT_BOTHEDGE  edge, both
 *                                        ways, whatever INT_TYPE says
 *   0x38 DEBOUNCE  0x40 DBCLK_DIV_EN     0x48 DBCLK_DIV_CON
 *   0x50 INT_STATUS (masked)             0x58 INT_RAWSTATUS
 *   0x60 PORT_EOI  write 1 to clear an edge
 *   0x70 EXT_PORT  pin levels            0x78 VERSION_ID
 *
 * Each 32-bit control register is two hiword-masked halves: the word at
 * the offset holds pins 0..15 (bits 31:16 enable the write of bits 15:0),
 * the word at +4 pins 16..31.  STATUS, RAWSTATUS, EXT_PORT and VERSION
 * are plain 32-bit reads.
 *
 * Pins are qdev GPIOs both ways: inputs set what an input pin reads
 * (property "input-levels" for their state at reset), outputs follow DR
 * on the pins set to output.
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

#define TYPE_RK_GPIO "rk-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(RkGpioState, RK_GPIO)

enum {
    G_DR, G_DDR, G_INT_EN, G_INT_MASK, G_INT_TYPE, G_INT_POL, G_INT_BOTH,
    G_DEBOUNCE, G_DBDIV_EN, G_DBDIV_CON, G_NREGS
};

#define R_INT_STATUS    0x50
#define R_INT_RAW       0x58
#define R_PORT_EOI      0x60
#define R_EXT_PORT      0x70
#define R_VERSION       0x78
#define REGS_SIZE       0x100
#define VERSION_V2      0x01000C2B

struct RkGpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[32];

    uint32_t regs[G_NREGS];
    uint32_t in;            /* levels driven onto the pins from outside */
    uint32_t raw;           /* latched edges and current levels */
    uint32_t last_level;    /* pin levels at the last update, for edges */
    uint32_t input_levels;  /* property: inputs' levels at reset */
};

static uint32_t rk_gpio_levels(RkGpioState *s)
{
    uint32_t ddr = s->regs[G_DDR];

    return (s->regs[G_DR] & ddr) | (s->in & ~ddr);
}

static void rk_gpio_update(RkGpioState *s)
{
    uint32_t lv = rk_gpio_levels(s);
    uint32_t changed = lv ^ s->last_level;
    uint32_t both = s->regs[G_INT_BOTH];
    /* BOTHEDGE makes a pin edge-triggered whatever INT_TYPE says: Linux
     * sets only it for IRQ_TYPE_EDGE_BOTH, leaving INT_TYPE at level */
    uint32_t edge = s->regs[G_INT_TYPE] | both;
    uint32_t pol = s->regs[G_INT_POL];
    uint32_t rise = changed & lv, fall = changed & ~lv;
    uint32_t hit;
    int i;

    /* edges latch; levels follow the pin */
    hit = (both & changed) | (~both & ((pol & rise) | (~pol & fall)));
    s->raw |= hit & edge & s->regs[G_INT_EN];
    s->raw &= edge;
    s->raw |= ~edge & s->regs[G_INT_EN] & ~(lv ^ pol);

    /*
     * What the pin drives: its output level, or nothing -- the board's
     * pull-up -- as an input.  An open-drain line (a bit-banged I2C SDA)
     * is released by turning the pin to an input.
     */
    for (i = 0; i < 32; i++) {
        qemu_set_irq(s->out[i], s->regs[G_DDR] & (1u << i)
                                ? (lv >> i) & 1 : 1);
    }
    s->last_level = lv;
    qemu_set_irq(s->irq, !!(s->raw & ~s->regs[G_INT_MASK]));
}

static void rk_gpio_set_input(void *opaque, int n, int level)
{
    RkGpioState *s = opaque;

    s->in = deposit32(s->in, n, 1, !!level);
    rk_gpio_update(s);
}

static int rk_gpio_reg_index(hwaddr off)
{
    switch (off & ~7) {
    case 0x00: return G_DR;
    case 0x08: return G_DDR;
    case 0x10: return G_INT_EN;
    case 0x18: return G_INT_MASK;
    case 0x20: return G_INT_TYPE;
    case 0x28: return G_INT_POL;
    case 0x30: return G_INT_BOTH;
    case 0x38: return G_DEBOUNCE;
    case 0x40: return G_DBDIV_EN;
    case 0x48: return G_DBDIV_CON;
    }
    return -1;
}

static uint64_t rk_gpio_read(void *opaque, hwaddr off, unsigned size)
{
    RkGpioState *s = opaque;
    int i;

    switch (off) {
    case R_INT_STATUS:
        return s->raw & ~s->regs[G_INT_MASK];
    case R_INT_RAW:
        return s->raw;
    case R_EXT_PORT:
        return rk_gpio_levels(s);
    case R_VERSION:
        return VERSION_V2;
    case R_PORT_EOI:
    case R_PORT_EOI + 4:
        return 0;
    }
    i = rk_gpio_reg_index(off);
    if (i < 0) {
        return 0;
    }
    return (off & 4) ? s->regs[i] >> 16 : s->regs[i] & 0xffff;
}

static void rk_gpio_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkGpioState *s = opaque;
    uint32_t mask = (val >> 16) & 0xffff;
    uint32_t bits = val & mask;
    int shift = (off & 4) ? 16 : 0;
    int i;

    if (off == R_PORT_EOI || off == R_PORT_EOI + 4) {
        s->raw &= ~((bits << shift) &
                    (s->regs[G_INT_TYPE] | s->regs[G_INT_BOTH]));
        rk_gpio_update(s);
        return;
    }
    i = rk_gpio_reg_index(off);
    if (i < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-gpio: write 0x%" HWADDR_PRIx "\n",
                      off);
        return;
    }
    s->regs[i] = (s->regs[i] & ~(mask << shift)) | (bits << shift);
    rk_gpio_update(s);
}

static const MemoryRegionOps rk_gpio_ops = {
    .read = rk_gpio_read,
    .write = rk_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_gpio_reset(DeviceState *dev)
{
    RkGpioState *s = RK_GPIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->in = s->input_levels;
    s->raw = 0;
    s->last_level = rk_gpio_levels(s);
}

static void rk_gpio_init(Object *obj)
{
    RkGpioState *s = RK_GPIO(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rk_gpio_ops, s, TYPE_RK_GPIO,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in(dev, rk_gpio_set_input, 32);
    qdev_init_gpio_out(dev, s->out, 32);
}

static const Property rk_gpio_properties[] = {
    DEFINE_PROP_UINT32("input-levels", RkGpioState, input_levels, 0),
};

static const VMStateDescription rk_gpio_vmstate = {
    .name = TYPE_RK_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkGpioState, G_NREGS),
        VMSTATE_UINT32(in, RkGpioState),
        VMSTATE_UINT32(raw, RkGpioState),
        VMSTATE_UINT32(last_level, RkGpioState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_gpio_reset);
    device_class_set_props(dc, rk_gpio_properties);
    dc->vmsd = &rk_gpio_vmstate;
}

static const TypeInfo rk_gpio_info = {
    .name = TYPE_RK_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkGpioState),
    .instance_init = rk_gpio_init,
    .class_init = rk_gpio_class_init,
};

static void rk_gpio_register_types(void)
{
    type_register_static(&rk_gpio_info);
}

type_init(rk_gpio_register_types)
