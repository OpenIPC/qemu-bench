/*
 * Fullhan interrupt controller: a DesignWare APB ICTL with 64 IRQ sources.
 *
 * drivers/irqchip/irq-fh.c (and the 3.0 kernels' mach-fh/irq.c), each
 * register a low / high pair:
 *
 *   0x00 / 0x04  IRQ_INTEN       0x18 / 0x1C  IRQ_RAWSTATUS
 *   0x08 / 0x0C  IRQ_INTMASK     0x20 / 0x24  IRQ_STATUS
 *   0x10 / 0x14  IRQ_INTFORCE    0x28 / 0x2C  IRQ_MASKSTATUS
 *                                0x30 / 0x34  IRQ_FINALSTATUS
 *
 * RAWSTATUS is the input levels (or forced), STATUS = RAW & EN, the masked
 * and final status further & ~MASK; the handler picks the first bit of
 * FINALSTATUS.  Every source is level triggered.  The FIQ block at 0x2C0
 * is the same arrangement for eight lines nothing here raises; it keeps
 * what is written.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_INTC "fh-intc"
OBJECT_DECLARE_SIMPLE_TYPE(FhIntcState, FH_INTC)

#define FH_INTC_NUM_IRQS    64
#define FH_INTC_SIZE        0x1000
#define FH_INTC_FIQ         0x2C0

struct FhIntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;
    uint64_t level;
    uint64_t enable;
    uint64_t mask;
    uint64_t force;
    uint32_t fiq_regs[8];
};

static uint64_t fh_intc_raw(FhIntcState *s)
{
    return s->level | s->force;
}

static uint64_t fh_intc_final(FhIntcState *s)
{
    return fh_intc_raw(s) & s->enable & ~s->mask;
}

static void fh_intc_update(FhIntcState *s)
{
    qemu_set_irq(s->irq, fh_intc_final(s) != 0);
}

static void fh_intc_set_irq(void *opaque, int n, int level)
{
    FhIntcState *s = opaque;

    s->level = deposit64(s->level, n, 1, !!level);
    fh_intc_update(s);
}

static uint32_t fh_intc_half(uint64_t v, hwaddr off)
{
    return (off & 4) ? v >> 32 : (uint32_t)v;
}

static uint32_t fh_intc_read_reg(FhIntcState *s, hwaddr off)
{
    switch (off) {
    case 0x00:
    case 0x04:
        return fh_intc_half(s->enable, off);
    case 0x08:
    case 0x0C:
        return fh_intc_half(s->mask, off);
    case 0x10:
    case 0x14:
        return fh_intc_half(s->force, off);
    case 0x18:
    case 0x1C:
        return fh_intc_half(fh_intc_raw(s), off);
    case 0x20:
    case 0x24:
        return fh_intc_half(fh_intc_raw(s) & s->enable, off);
    case 0x28:
    case 0x2C:
    case 0x30:
    case 0x34:
        return fh_intc_half(fh_intc_final(s), off);
    case FH_INTC_FIQ ... FH_INTC_FIQ + 0x1C:
        return s->fiq_regs[(off - FH_INTC_FIQ) >> 2];
    default:
        return 0;
    }
}

static uint64_t fh_intc_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t v = fh_intc_read_reg(opaque, off & ~3);

    return extract32(v, (off & 3) * 8, size * 8);
}

static void fh_intc_set_half(uint64_t *reg, hwaddr off, uint32_t val)
{
    *reg = deposit64(*reg, (off & 4) ? 32 : 0, 32, val);
}

static void fh_intc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    FhIntcState *s = opaque;

    if (size != 4 || (off & 3)) {
        return;
    }
    switch (off) {
    case 0x00:
    case 0x04:
        fh_intc_set_half(&s->enable, off, val);
        break;
    case 0x08:
    case 0x0C:
        fh_intc_set_half(&s->mask, off, val);
        break;
    case 0x10:
    case 0x14:
        fh_intc_set_half(&s->force, off, val);
        break;
    case FH_INTC_FIQ ... FH_INTC_FIQ + 0x1C:
        s->fiq_regs[(off - FH_INTC_FIQ) >> 2] = val;
        break;
    default:
        return;
    }
    fh_intc_update(s);
}

/*
 * The kernel's handler finds the pending source with find_first_bit()
 * straight on FINALSTATUS, and ARM's implementation loads bytes.
 */
static const MemoryRegionOps fh_intc_ops = {
    .read = fh_intc_read,
    .write = fh_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void fh_intc_reset(DeviceState *dev)
{
    FhIntcState *s = FH_INTC(dev);

    s->enable = 0;
    s->mask = 0;
    s->force = 0;
    memset(s->fiq_regs, 0, sizeof(s->fiq_regs));
    fh_intc_update(s);
}

static void fh_intc_init(Object *obj)
{
    FhIntcState *s = FH_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &fh_intc_ops, s, TYPE_FH_INTC,
                          FH_INTC_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), fh_intc_set_irq, FH_INTC_NUM_IRQS);
}

static const VMStateDescription vmstate_fh_intc = {
    .name = TYPE_FH_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(level, FhIntcState),
        VMSTATE_UINT64(enable, FhIntcState),
        VMSTATE_UINT64(mask, FhIntcState),
        VMSTATE_UINT64(force, FhIntcState),
        VMSTATE_UINT32_ARRAY(fiq_regs, FhIntcState, 8),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_intc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, fh_intc_reset);
    dc->vmsd = &vmstate_fh_intc;
}

static const TypeInfo fh_intc_info = {
    .name           = TYPE_FH_INTC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhIntcState),
    .instance_init  = fh_intc_init,
    .class_init     = fh_intc_class_init,
};

static void fh_intc_register_types(void)
{
    type_register_static(&fh_intc_info);
}

type_init(fh_intc_register_types)
