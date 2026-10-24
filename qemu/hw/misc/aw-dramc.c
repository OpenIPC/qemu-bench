/*
 * Allwinner sun8iw19 DRAM controller (V831/V833): the controller and its
 * DesignWare-PUB-like PHY at 0x04003000, which boot0's libdram trains.
 *
 * Storage for every register, plus the status the training waits on: no
 * DRAM to train, so every step is done at once and without error.
 *
 *   0x010 PGSR0  bit 0 IDONE once a PIR (0x000) step is started
 *   0x018 STATR  bits 2:0 = 1, normal operating mode
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_AW_DRAMC "aw-dramc"
OBJECT_DECLARE_SIMPLE_TYPE(AwDramcState, AW_DRAMC)

#define AW_DRAMC_SIZE       0x2000

#define AW_DRAMC_PIR        0x000
#define AW_DRAMC_PGSR0      0x010
#define AW_DRAMC_STATR      0x018

struct AwDramcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[AW_DRAMC_SIZE / 4];
};

static uint64_t aw_dramc_read(void *opaque, hwaddr addr, unsigned size)
{
    AwDramcState *s = opaque;
    uint32_t val = s->regs[addr >> 2];

    switch (addr) {
    case AW_DRAMC_PGSR0:
        val |= 1;
        break;
    case AW_DRAMC_STATR:
        val = (val & ~7u) | 1;
        break;
    }
    return val;
}

static void aw_dramc_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    AwDramcState *s = opaque;

    switch (addr) {
    case AW_DRAMC_PIR:
        /* The step runs and finishes: INIT (bit 0) clears itself. */
        val &= ~1u;
        break;
    }
    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps aw_dramc_ops = {
    .read = aw_dramc_read,
    .write = aw_dramc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_dramc_reset_hold(Object *obj, ResetType type)
{
    AwDramcState *s = AW_DRAMC(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aw_dramc_init(Object *obj)
{
    AwDramcState *s = AW_DRAMC(obj);

    memory_region_init_io(&s->iomem, obj, &aw_dramc_ops, s, TYPE_AW_DRAMC,
                          AW_DRAMC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void aw_dramc_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_dramc_reset_hold;
}

static const TypeInfo aw_dramc_info = {
    .name           = TYPE_AW_DRAMC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwDramcState),
    .instance_init  = aw_dramc_init,
    .class_init     = aw_dramc_class_init,
};

static void aw_dramc_register_types(void)
{
    type_register_static(&aw_dramc_info);
}

type_init(aw_dramc_register_types)
