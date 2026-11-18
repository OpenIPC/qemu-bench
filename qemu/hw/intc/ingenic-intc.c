/*
 * Ingenic INTC: the JZ47xx / T-series interrupt controller.
 *
 * 64 level-sensitive sources in two banks of 32, 0x20 apart, each bank
 * with the same five registers (arch/mips/xburst/soc-t31/common/irq.c):
 *
 *   ISR  0x00  raw source levels (read only)
 *   IMR  0x04  mask, 1 = masked; out of reset everything is masked
 *   IMSR 0x08  write 1 to set mask bits
 *   IMCR 0x0C  write 1 to clear mask bits
 *   IPR  0x10  pending: ISR & ~IMR (read only)
 *
 * Any pending bit raises the one output, which the machine wires to the
 * CPU's IP2.  The kernel masks with IMSR/IMCR only; U-Boot writes IMR
 * directly.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_INTC "ingenic-intc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIntcState, INGENIC_INTC)

#define INTC_BANKS      2
#define INTC_NR         (32 * INTC_BANKS)
#define INTC_BANK_STEP  0x20

#define R_ISR           0x00
#define R_IMR           0x04
#define R_IMSR          0x08
#define R_IMCR          0x0C
#define R_IPR           0x10

struct IngenicIntcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq out;
    uint32_t level[INTC_BANKS];
    uint32_t mask[INTC_BANKS];
};

static void ingenic_intc_update(IngenicIntcState *s)
{
    bool pending = false;
    int i;

    for (i = 0; i < INTC_BANKS; i++) {
        pending |= (s->level[i] & ~s->mask[i]) != 0;
    }
    qemu_set_irq(s->out, pending);
}

static void ingenic_intc_set_irq(void *opaque, int n, int level)
{
    IngenicIntcState *s = opaque;
    uint32_t bit = 1u << (n % 32);

    if (level) {
        s->level[n / 32] |= bit;
    } else {
        s->level[n / 32] &= ~bit;
    }
    ingenic_intc_update(s);
}

static uint64_t ingenic_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicIntcState *s = opaque;
    int bank = addr / INTC_BANK_STEP;

    if (bank >= INTC_BANKS) {
        return 0;
    }
    switch (addr % INTC_BANK_STEP) {
    case R_ISR:
        return s->level[bank];
    case R_IMR:
        return s->mask[bank];
    case R_IPR:
        return s->level[bank] & ~s->mask[bank];
    default:
        return 0;
    }
}

static void ingenic_intc_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    IngenicIntcState *s = opaque;
    int bank = addr / INTC_BANK_STEP;

    if (bank >= INTC_BANKS) {
        return;
    }
    switch (addr % INTC_BANK_STEP) {
    case R_IMR:
        s->mask[bank] = val;
        break;
    case R_IMSR:
        s->mask[bank] |= val;
        break;
    case R_IMCR:
        s->mask[bank] &= ~val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-intc: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 " to a read-only register\n", addr, val);
        return;
    }
    ingenic_intc_update(s);
}

static const MemoryRegionOps ingenic_intc_ops = {
    .read = ingenic_intc_read,
    .write = ingenic_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ingenic_intc_reset(DeviceState *dev)
{
    IngenicIntcState *s = INGENIC_INTC(dev);
    int i;

    for (i = 0; i < INTC_BANKS; i++) {
        s->mask[i] = 0xffffffff;
    }
    ingenic_intc_update(s);
}

static void ingenic_intc_init(Object *obj)
{
    IngenicIntcState *s = INGENIC_INTC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_intc_ops, s,
                          TYPE_INGENIC_INTC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->out);
    qdev_init_gpio_in(DEVICE(obj), ingenic_intc_set_irq, INTC_NR);
}

static const VMStateDescription vmstate_ingenic_intc = {
    .name = TYPE_INGENIC_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(level, IngenicIntcState, INTC_BANKS),
        VMSTATE_UINT32_ARRAY(mask, IngenicIntcState, INTC_BANKS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_intc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_intc_reset);
    dc->vmsd = &vmstate_ingenic_intc;
}

static const TypeInfo ingenic_intc_info = {
    .name = TYPE_INGENIC_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIntcState),
    .instance_init = ingenic_intc_init,
    .class_init = ingenic_intc_class_init,
};

static void ingenic_intc_register_types(void)
{
    type_register_static(&ingenic_intc_info);
}

type_init(ingenic_intc_register_types)
