/*
 * MStar infinity3 GPIO pads, SAR ADC and the PM-sleep interrupt
 * controller.
 *
 * Read off the vendor kernel's drivers/mstar (gpio/infinity3/mhal_gpio.c,
 * irqchip/infinity3/irqchip.c, sar/mdrv_sar.c, ircut/ms_ircut.c; the I3
 * SDK MSC3XX_SDK_V_1711291828).  RIU banks of 16-bit registers on a
 * 4-byte stride; the GPIO HAL reads and writes them a byte at a time.
 *
 * PADGPIO, bank 0x103C: one register per pad (GPIO0-15 at 0x00-0x0F,
 *   PWM0/1 0x10/0x11, FUART 0x14-0x17, UART0/1 0x18-0x1B, SR_IO 0x20-0x31,
 *   NAND 0x40-0x4E, SD 0x50-0x55, I2C0/1 0x60-0x63, SPI0/1 0x70-0x77):
 *   bit 0 the level in, bit 4 the level out, bit 5 output disabled.
 * PM GPIO, bank 0x000F: one register per pad (PM_GPIO0-10 0x00-0x0A,
 *   PM_IRIN 0x14, PM_SPI 0x18-0x1B, SPI_WPZ/HLD 0x44/0x45, PM_SD_CDZ
 *   0x47, LED0/1 0x4A/0x4B): bit 0 output disabled, bit 1 out, bit 2
 *   in; and the pad's PM-sleep interrupt: bit 4 masked, bit 6 clear,
 *   bit 7 falling edge (else rising).
 * SAR, bank 0x0014: reg 0x00 bit 14 starts a conversion and reads busy
 *   (ms_sar_get); 0x11 [3:0] the pads in ADC mode, [11:8] SAR_GPIO0-3
 *   output disabled; 0x12 [3:0] out, [11:8] in; 0x40-0x43 the channels'
 *   10-bit data (/dev/sar channel n reads 0x40 + n).
 *
 * An output reads back what it drives.  An input reads its pad, which
 * nothing drives: high, as the board's pull-ups hold it, unless the
 * machine sets it through the named GPIO inputs "pad", "pm-pad" (by
 * register) and "sar-pad".  So PM_SD_CDZ, the SD card detect (active low,
 * hal_card_platform_iNF3.c), reads "no card" unless "sd-card" is set.
 *
 * PM-sleep interrupt controller, PMSLEEP bank 0x000E regs 0x00-0x0F (the
 * rest of the bank, its reset key at 0x2E included, stays RIU storage).
 * It is cascaded into the main controller's MS_IRQ 2 (INT_IRQ_PM_SLEEP,
 * GIC SPI 34), and its domain has two groups:
 *
 *   hwirq 0-77, edge: 0 IR (mask 0x00 bit 0, clear 0x02 bit 0, falling
 *     0x03 bit 0), 1 DVI_CK_DET (bit 1 of the same), 2-77 the PM GPIO pad
 *     at register hwirq - 2 (the bits above): PM_GPIO0-10 are 2-12,
 *     PM_IRIN 22.  Final status (raw & ~mask) at 0x04 / 0x05 for 0-31.
 *   hwirq 78-85, level: SAR 79, WOL 80, RTC 82, SAR_GPIO 84; mask 0x08,
 *     active low 0x09 [7:0], final status 0x0E [7:0].  Nothing drives
 *     them here but the "pm-irq" GPIO inputs.
 *
 * ms_handle_cascade_pm_irq reads the final status and BUG()s if none is
 * set, so the line is exactly "some final status bit is set".  The clears
 * are written with SETREG16 and never written back to 0: they act once
 * and read as 0.  An edge latches whether or not the pad is masked, as a
 * status register does; the kernel acks before it unmasks (ms_pm.c) or
 * takes the one stale edge harmlessly (ms_ircut.c).
 *
 * Day and night (property "night", which the machine's daynight= sets and
 * which can be changed while running): the IR-cut driver (DTS ircut,
 * ircut-gpio-num = PAD_PM_IRIN, interrupt INT_PMSLEEP_IRIN) makes PM_IRIN
 * an input and flips its interrupt between rising and falling after each
 * edge; PM_IRIN reads high by day and low by night.  The camera
 * application (jco_server's encode_infrared) does not use it: it polls
 * /dev/sar channel 2 (register 0x42) as a light sensor and calls night
 * below its threshold (dwTurnOnLux, with a hysteresis of 3), so channel 2
 * reads 0x3FF by day and 0x008 by night.  The other channels read 0x3FF.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_I3_GPIO "sstar-i3-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI3GpioState, SSTAR_I3_GPIO)

#define BANK_REGS           0x80
#define PMSLEEP_REGS        0x10

/* PMSLEEP */
#define PS_MASK             0x00        /* hwirq 0, 1 */
#define PS_CLEAR            0x02
#define PS_FALLING          0x03
#define PS_FINAL_LO         0x04
#define PS_FINAL_HI         0x05
#define PS_IRQ_MASK         0x08        /* hwirq 78-85 */
#define PS_IRQ_LOW          0x09
#define PS_IRQ_FINAL        0x0E

/* PM GPIO pad register */
#define PM_OEN              (1u << 0)
#define PM_OUT              (1u << 1)
#define PM_IN               (1u << 2)
#define PM_INT_MASK         (1u << 4)
#define PM_INT_CLR          (1u << 6)
#define PM_INT_FALLING      (1u << 7)
#define PM_PAD_IRIN         0x14
#define PM_PAD_SD_CDZ       0x47

/* PADGPIO pad register */
#define PAD_IN              (1u << 0)
#define PAD_OUT             (1u << 4)
#define PAD_OEN             (1u << 5)

/* SAR */
#define SAR_CTRL0           0x00
#define SAR_BUSY            (1u << 14)
#define SAR_MODE            0x11
#define SAR_GPIO            0x12
#define SAR_DATA            0x40
#define SAR_CHANNELS        8
#define SAR_LIGHT           2           /* jco_server's light sensor */
#define SAR_DAY             0x3FF
#define SAR_NIGHT           0x008

/* the PM GPIO registers that are pads (mhal_gpio.c GPIO77-97) */
static const uint8_t pm_pads[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
    PM_PAD_IRIN, 0x18, 0x19, 0x1A, 0x1B, 0x44, 0x45, PM_PAD_SD_CDZ,
    0x4A, 0x4B,
};

/* the PADGPIO registers that are pads (mhal_gpio.c GPIO0-76) */
static bool is_pad(int r)
{
    return r <= 0x11 || (r >= 0x14 && r <= 0x1B) ||
           (r >= 0x20 && r <= 0x31) || (r >= 0x40 && r <= 0x4E) ||
           (r >= 0x50 && r <= 0x55) || (r >= 0x60 && r <= 0x63) ||
           (r >= 0x70 && r <= 0x77);
}

static bool is_pm_pad(int r)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(pm_pads); i++) {
        if (pm_pads[i] == r) {
            return true;
        }
    }
    return false;
}

struct SstarI3GpioState {
    SysBusDevice parent_obj;

    MemoryRegion pmsleep_mr;
    MemoryRegion pmgpio_mr;
    MemoryRegion padgpio_mr;
    MemoryRegion sar_mr;
    qemu_irq irq;

    uint16_t pmsleep[PMSLEEP_REGS];
    uint16_t pmgpio[BANK_REGS];
    uint16_t padgpio[BANK_REGS];
    uint16_t sar[BANK_REGS];
    uint32_t raw;               /* latched edges, hwirq 0-31 */
    uint8_t irq_in;             /* hwirq 78-85 input levels */

    /* the world outside the pads, kept over resets */
    uint8_t pm_ext[BANK_REGS];
    uint8_t pad_ext[BANK_REGS];
    uint8_t sar_ext;
    uint16_t adc[SAR_CHANNELS];
    bool night;
    bool sd_card;
};

static bool pm_level(SstarI3GpioState *s, int r)
{
    uint16_t v = s->pmgpio[r];

    return v & PM_OEN ? s->pm_ext[r] : !!(v & PM_OUT);
}

static bool pad_level(SstarI3GpioState *s, int r)
{
    uint16_t v = s->padgpio[r];

    return v & PAD_OEN ? s->pad_ext[r] : !!(v & PAD_OUT);
}

static uint8_t sar_levels(SstarI3GpioState *s)
{
    uint8_t oen = s->sar[SAR_MODE] >> 8;
    uint8_t out = s->sar[SAR_GPIO];

    return ((oen & s->sar_ext) | (~oen & out)) & 0xF;
}

static uint32_t ps_fiq_final(SstarI3GpioState *s)
{
    uint32_t mask = s->pmsleep[PS_MASK] & 3;
    int r;

    for (r = 0; r < 30; r++) {
        if (!is_pm_pad(r) || (s->pmgpio[r] & PM_INT_MASK)) {
            mask |= 1u << (r + 2);
        }
    }
    return s->raw & ~mask;
}

static uint8_t ps_irq_final(SstarI3GpioState *s)
{
    return (s->irq_in ^ s->pmsleep[PS_IRQ_LOW]) & ~s->pmsleep[PS_IRQ_MASK];
}

static void ps_update(SstarI3GpioState *s)
{
    qemu_set_irq(s->irq, ps_fiq_final(s) || ps_irq_final(s));
}

/* a PM pad's level went from @was: latch the edge its polarity wants */
static void pm_edge(SstarI3GpioState *s, int r, bool was)
{
    bool now = pm_level(s, r);
    bool falling = s->pmgpio[r] & PM_INT_FALLING;

    /* the final status, 0x04/0x05, has bits for hwirq 0-31 only */
    if (r < 30 && is_pm_pad(r) && was != now && now != falling) {
        s->raw |= 1u << (r + 2);
    }
}

/* ---- PMSLEEP ---- */

static uint64_t pmsleep_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (PMSLEEP_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    switch (reg) {
    case PS_FINAL_LO:
        v = ps_fiq_final(s);
        break;
    case PS_FINAL_HI:
        v = ps_fiq_final(s) >> 16;
        break;
    case PS_IRQ_FINAL:
        v = ps_irq_final(s);
        break;
    default:
        v = s->pmsleep[reg];
        break;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static uint16_t merge(uint16_t old, hwaddr off, uint64_t val, unsigned size)
{
    if (size == 1) {
        return off & 1 ? (old & 0x00FF) | ((val & 0xFF) << 8)
                       : (old & 0xFF00) | (val & 0xFF);
    }
    return val;
}

static void pmsleep_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (PMSLEEP_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    v = merge(s->pmsleep[reg], off, val, size);
    switch (reg) {
    case PS_CLEAR:
        s->raw &= ~(uint32_t)(v & 3);
        v &= ~3;
        break;
    case PS_FINAL_LO:
    case PS_FINAL_HI:
    case PS_IRQ_FINAL:
        return;
    }
    s->pmsleep[reg] = v;
    ps_update(s);
}

/* ---- PM GPIO ---- */

static uint64_t pmgpio_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->pmgpio[reg];
    if (is_pm_pad(reg)) {
        v = (v & ~PM_IN) | (pm_level(s, reg) ? PM_IN : 0);
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void pmgpio_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    bool was;
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    was = pm_level(s, reg);
    v = merge(s->pmgpio[reg], off, val, size);
    if (!is_pm_pad(reg)) {
        s->pmgpio[reg] = v;
        return;
    }
    if ((v & PM_INT_CLR) && reg < 30) {
        s->raw &= ~(1u << (reg + 2));
    }
    s->pmgpio[reg] = v & ~(PM_INT_CLR | PM_IN);
    pm_edge(s, reg, was);
    ps_update(s);
}

/* ---- PADGPIO ---- */

static uint64_t padgpio_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->padgpio[reg];
    if (is_pad(reg)) {
        v = (v & ~PAD_IN) | (pad_level(s, reg) ? PAD_IN : 0);
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void padgpio_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);

    if ((off & 3) >= 2) {
        return;
    }
    s->padgpio[reg] = merge(s->padgpio[reg], off, val, size);
    if (is_pad(reg)) {
        s->padgpio[reg] &= ~PAD_IN;
    }
}

/* ---- SAR ---- */

static uint64_t sar_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v = s->sar[reg];

    if ((off & 3) >= 2) {
        return 0;
    }
    if (reg == SAR_CTRL0) {
        v &= ~SAR_BUSY;                 /* converted at once */
    } else if (reg == SAR_GPIO) {
        v = (v & 0x00FF) | (uint16_t)sar_levels(s) << 8;
    } else if (reg >= SAR_DATA && reg < SAR_DATA + SAR_CHANNELS) {
        v = s->adc[reg - SAR_DATA];
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sar_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarI3GpioState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);

    if ((off & 3) >= 2) {
        return;
    }
    s->sar[reg] = merge(s->sar[reg], off, val, size);
}

#define RIU_OPS(name)                                       \
    static const MemoryRegionOps name##_ops = {             \
        .read = name##_read,                                \
        .write = name##_write,                              \
        .endianness = DEVICE_LITTLE_ENDIAN,                 \
        .impl.min_access_size = 1,                          \
        .impl.max_access_size = 2,                          \
        .valid.min_access_size = 1,                         \
        .valid.max_access_size = 4,                         \
    }

RIU_OPS(pmsleep);
RIU_OPS(pmgpio);
RIU_OPS(padgpio);
RIU_OPS(sar);

/* ---- the outside world ---- */

static void set_pm_pad(void *opaque, int n, int level)
{
    SstarI3GpioState *s = opaque;
    bool was = pm_level(s, n);

    s->pm_ext[n] = !!level;
    pm_edge(s, n, was);
    ps_update(s);
}

static void set_pad(void *opaque, int n, int level)
{
    SstarI3GpioState *s = opaque;

    s->pad_ext[n] = !!level;
}

static void set_sar_pad(void *opaque, int n, int level)
{
    SstarI3GpioState *s = opaque;

    s->sar_ext = deposit32(s->sar_ext, n, 1, !!level);
}

static void set_pm_irq(void *opaque, int n, int level)
{
    SstarI3GpioState *s = opaque;

    s->irq_in = deposit32(s->irq_in, n, 1, !!level);
    ps_update(s);
}

static void sstar_i3_gpio_daynight(SstarI3GpioState *s)
{
    s->adc[SAR_LIGHT] = s->night ? SAR_NIGHT : SAR_DAY;
    set_pm_pad(s, PM_PAD_IRIN, !s->night);
}

static bool get_night(Object *obj, Error **errp)
{
    return SSTAR_I3_GPIO(obj)->night;
}

static void set_night(Object *obj, bool value, Error **errp)
{
    SstarI3GpioState *s = SSTAR_I3_GPIO(obj);

    s->night = value;
    sstar_i3_gpio_daynight(s);
}

static void sstar_i3_gpio_reset(DeviceState *dev)
{
    SstarI3GpioState *s = SSTAR_I3_GPIO(dev);
    int i;

    memset(s->pmsleep, 0, sizeof(s->pmsleep));
    memset(s->pmgpio, 0, sizeof(s->pmgpio));
    memset(s->sar, 0, sizeof(s->sar));
    /* pads come up as inputs, their interrupts masked */
    for (i = 0; i < ARRAY_SIZE(pm_pads); i++) {
        s->pmgpio[pm_pads[i]] = PM_OEN | PM_INT_MASK;
    }
    for (i = 0; i < BANK_REGS; i++) {
        s->padgpio[i] = is_pad(i) ? PAD_OEN : 0;
    }
    s->sar[SAR_MODE] = 0x0F00;
    s->pmsleep[PS_MASK] = 3;
    s->pmsleep[PS_IRQ_MASK] = 0xFF;
    s->raw = 0;
    ps_update(s);
}

static void sstar_i3_gpio_realize(DeviceState *dev, Error **errp)
{
    SstarI3GpioState *s = SSTAR_I3_GPIO(dev);
    int i;

    memset(s->pm_ext, 1, sizeof(s->pm_ext));
    memset(s->pad_ext, 1, sizeof(s->pad_ext));
    s->sar_ext = 0xF;
    for (i = 0; i < SAR_CHANNELS; i++) {
        s->adc[i] = SAR_DAY;
    }
    s->pm_ext[PM_PAD_SD_CDZ] = !s->sd_card;
    sstar_i3_gpio_daynight(s);
}

static void sstar_i3_gpio_init(Object *obj)
{
    SstarI3GpioState *s = SSTAR_I3_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->pmsleep_mr, obj, &pmsleep_ops, s,
                          "sstar-i3-pmsleep", PMSLEEP_REGS * 4);
    memory_region_init_io(&s->pmgpio_mr, obj, &pmgpio_ops, s,
                          "sstar-i3-pmgpio", BANK_REGS * 4);
    memory_region_init_io(&s->padgpio_mr, obj, &padgpio_ops, s,
                          "sstar-i3-padgpio", BANK_REGS * 4);
    memory_region_init_io(&s->sar_mr, obj, &sar_ops, s,
                          "sstar-i3-sar", BANK_REGS * 4);
    sysbus_init_mmio(sbd, &s->pmsleep_mr);
    sysbus_init_mmio(sbd, &s->pmgpio_mr);
    sysbus_init_mmio(sbd, &s->padgpio_mr);
    sysbus_init_mmio(sbd, &s->sar_mr);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, set_pm_pad, "pm-pad", BANK_REGS);
    qdev_init_gpio_in_named(dev, set_pad, "pad", BANK_REGS);
    qdev_init_gpio_in_named(dev, set_sar_pad, "sar-pad", 4);
    qdev_init_gpio_in_named(dev, set_pm_irq, "pm-irq", 8);
    object_property_add_bool(obj, "night", get_night, set_night);
    object_property_set_description(obj, "night",
        "Night: PM_IRIN reads low and SAR channel 2 (the light sensor) "
        "dark; settable while running");
}

static const Property sstar_i3_gpio_props[] = {
    DEFINE_PROP_BOOL("sd-card", SstarI3GpioState, sd_card, false),
};

static const VMStateDescription vmstate_sstar_i3_gpio = {
    .name = TYPE_SSTAR_I3_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(pmsleep, SstarI3GpioState, PMSLEEP_REGS),
        VMSTATE_UINT16_ARRAY(pmgpio, SstarI3GpioState, BANK_REGS),
        VMSTATE_UINT16_ARRAY(padgpio, SstarI3GpioState, BANK_REGS),
        VMSTATE_UINT16_ARRAY(sar, SstarI3GpioState, BANK_REGS),
        VMSTATE_UINT32(raw, SstarI3GpioState),
        VMSTATE_UINT8(irq_in, SstarI3GpioState),
        VMSTATE_UINT8_ARRAY(pm_ext, SstarI3GpioState, BANK_REGS),
        VMSTATE_UINT8_ARRAY(pad_ext, SstarI3GpioState, BANK_REGS),
        VMSTATE_UINT8(sar_ext, SstarI3GpioState),
        VMSTATE_UINT16_ARRAY(adc, SstarI3GpioState, SAR_CHANNELS),
        VMSTATE_BOOL(night, SstarI3GpioState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i3_gpio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_i3_gpio_realize;
    device_class_set_legacy_reset(dc, sstar_i3_gpio_reset);
    device_class_set_props(dc, sstar_i3_gpio_props);
    dc->vmsd = &vmstate_sstar_i3_gpio;
}

static const TypeInfo sstar_i3_gpio_info = {
    .name           = TYPE_SSTAR_I3_GPIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI3GpioState),
    .instance_init  = sstar_i3_gpio_init,
    .class_init     = sstar_i3_gpio_class_init,
};

static void sstar_i3_gpio_register_types(void)
{
    type_register_static(&sstar_i3_gpio_info);
}

type_init(sstar_i3_gpio_register_types)
