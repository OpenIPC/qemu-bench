/*
 * SigmaStar / MStar "main" interrupt controller (RIU bank 0x1009).
 *
 * Not a controller in its own right: every peripheral line goes on to a
 * GIC SPI of the same number, and this block only masks it and sets its
 * polarity on the way (the vendor irqchip is a hierarchy domain stacked
 * on the GIC, drivers/sstar/irqchip/infinity2m/irqchip.c).  Two groups:
 *
 *   MS_IRQ  64 level lines, GIC SPI 32-95:  mask 0x54-0x57, polarity
 *           0x58-0x5B (1 = active low), raw status 0x5C-0x5F
 *   MS_FIQ  32 edge lines,  GIC SPI 96-127: mask 0x44-0x45, polarity
 *           0x48-0x49 (1 = falling edge), status 0x4C-0x4D, write 1 to
 *           clear -- the irq_ack/irq_eoi hooks do that for every FIQ
 *
 * Register numbers are 16-bit RIU registers, 4 bytes apart.  Lines come
 * out masked: the kernel unmasks each one as it is requested.
 *
 * Iford ("iford") has 128 MS_IRQ lines, GIC SPI 32-159, and MS_FIQ at SPI
 * 160-191.  Its driver (drivers/sstar/irqchip/iford/irq-main.c) indexes the
 * same registers past their old ends: masks 0x54-0x5B, polarity 0x58-0x5F,
 * raw status 0x5C-0x63.  So the upper masks and the lower polarities are one
 * register, and the model takes those as masks and ignores polarity: no
 * driver on this SoC asks for an active-low line.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_INTC "sstar-intc"
OBJECT_DECLARE_SIMPLE_TYPE(SstarIntcState, SSTAR_INTC)

#define MAX_IRQ             128
#define NUM_FIQ             32
#define MAX_LINES           (MAX_IRQ + NUM_FIQ)

#define R_FIQ_MASK          0x44
#define R_FIQ_POL           0x48
#define R_FIQ_STATUS        0x4C
#define R_IRQ_MASK          0x54
#define R_IRQ_POL           0x58
#define R_IRQ_STATUS        0x5C

struct SstarIntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq out[MAX_LINES];
    uint16_t regs[0x80];
    bool iford;
    int num_irq;                /* MS_IRQ lines */
    uint64_t irq_level[2];      /* raw input levels, MS_IRQ */
    uint32_t fiq_level;         /* raw input levels, MS_FIQ */
    uint32_t fiq_status;        /* latched edges */
};

static uint32_t sstar_intc_get32(SstarIntcState *s, int reg)
{
    return s->regs[reg] | ((uint32_t)s->regs[reg + 1] << 16);
}

static bool sstar_intc_irq_out(SstarIntcState *s, int i)
{
    bool level = s->irq_level[i / 64] >> (i % 64) & 1;

    if (s->regs[R_IRQ_MASK + i / 16] >> (i % 16) & 1) {
        return false;
    }
    if (!s->iford) {
        level ^= s->regs[R_IRQ_POL + i / 16] >> (i % 16) & 1;
    }
    return level;
}

static void sstar_intc_update(SstarIntcState *s)
{
    uint32_t fiq = s->fiq_status & ~sstar_intc_get32(s, R_FIQ_MASK);
    int i;

    for (i = 0; i < s->num_irq; i++) {
        qemu_set_irq(s->out[i], sstar_intc_irq_out(s, i));
    }
    for (i = 0; i < NUM_FIQ; i++) {
        qemu_set_irq(s->out[s->num_irq + i], (fiq >> i) & 1);
    }
}

static void sstar_intc_set_irq(void *opaque, int n, int level)
{
    SstarIntcState *s = opaque;

    if (n < s->num_irq) {
        s->irq_level[n / 64] = deposit64(s->irq_level[n / 64], n % 64, 1,
                                         !!level);
    } else {
        uint32_t bit = 1u << (n - s->num_irq);
        bool falling = sstar_intc_get32(s, R_FIQ_POL) & bit;
        bool was = s->fiq_level & bit;

        if (level && !was && !falling) {
            s->fiq_status |= bit;
        } else if (!level && was && falling) {
            s->fiq_status |= bit;
        }
        s->fiq_level = level ? (s->fiq_level | bit) : (s->fiq_level & ~bit);
    }
    sstar_intc_update(s);
}

static uint64_t sstar_intc_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIntcState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    switch (reg) {
    case R_FIQ_STATUS:
    case R_FIQ_STATUS + 1:
        v = s->fiq_status >> (16 * (reg - R_FIQ_STATUS));
        break;
    case R_IRQ_STATUS ... R_IRQ_STATUS + 7:
        if (reg - R_IRQ_STATUS < s->num_irq / 16) {
            int w = reg - R_IRQ_STATUS;

            v = s->irq_level[w / 4] >> (16 * (w % 4));
            break;
        }
        /* fall through */
    default:
        v = s->regs[reg];
        break;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_intc_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarIntcState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = (off & 1) ? (val & 0xFF) << 8 : (val & 0xFF);
    } else {
        v = val;
    }
    switch (reg) {
    case R_FIQ_STATUS:
    case R_FIQ_STATUS + 1:
        /* Write 1 to clear; an edge input still high stays cleared. */
        s->fiq_status &= ~((uint32_t)v << (16 * (reg - R_FIQ_STATUS)));
        break;
    default:
        if (size == 1) {
            uint16_t keep = (off & 1) ? 0x00FF : 0xFF00;

            v |= s->regs[reg] & keep;
        }
        s->regs[reg] = v;
        break;
    }
    sstar_intc_update(s);
}

static const MemoryRegionOps sstar_intc_ops = {
    .read = sstar_intc_read,
    .write = sstar_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_intc_reset(DeviceState *dev)
{
    SstarIntcState *s = SSTAR_INTC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_FIQ_MASK] = s->regs[R_FIQ_MASK + 1] = 0xFFFF;
    memset(s->regs + R_IRQ_MASK, 0xFF, s->num_irq / 8);
    s->fiq_status = 0;
    sstar_intc_update(s);
}

static void sstar_intc_init(Object *obj)
{
    SstarIntcState *s = SSTAR_INTC(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_intc_ops, s, "sstar-intc",
                          0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in(DEVICE(obj), sstar_intc_set_irq, MAX_LINES);
    qdev_init_gpio_out(DEVICE(obj), s->out, MAX_LINES);
}

static void sstar_intc_realize(DeviceState *dev, Error **errp)
{
    SstarIntcState *s = SSTAR_INTC(dev);

    s->num_irq = s->iford ? 128 : 64;
}

static const Property sstar_intc_props[] = {
    DEFINE_PROP_BOOL("iford", SstarIntcState, iford, false),
};

static const VMStateDescription vmstate_sstar_intc = {
    .name = TYPE_SSTAR_INTC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarIntcState, 0x80),
        VMSTATE_UINT64_ARRAY(irq_level, SstarIntcState, 2),
        VMSTATE_UINT32(fiq_level, SstarIntcState),
        VMSTATE_UINT32(fiq_status, SstarIntcState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_intc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_intc_realize;
    device_class_set_props(dc, sstar_intc_props);
    device_class_set_legacy_reset(dc, sstar_intc_reset);
    dc->vmsd = &vmstate_sstar_intc;
}

static const TypeInfo sstar_intc_info = {
    .name           = TYPE_SSTAR_INTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarIntcState),
    .instance_init  = sstar_intc_init,
    .class_init     = sstar_intc_class_init,
};

static void sstar_intc_register_types(void)
{
    type_register_static(&sstar_intc_info);
}

type_init(sstar_intc_register_types)
