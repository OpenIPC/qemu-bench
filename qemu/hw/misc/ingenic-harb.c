/*
 * Ingenic AHB0 bus controller (HARB0, 0x13000000) and its SoC ID word.
 *
 * Bus priorities and the like are storage.  0x2C is the SoC ID, the word
 * software identifies the die by (ipctool src/hal/ingenic.c, after the
 * vendor's libsysutils):
 *
 *   bits 31:28  1
 *   bits 27:12  0x0005 on T10, 0x2000 on T20; otherwise bits 19:12 are
 *               the family -- 0x21, 0x23, 0x30, 0x31, 0x40, 0x41
 *   bits 11:0   the revision
 *
 * The grade within a family (T31N/L/X/A/..., T40N/XP/A, T41LQ/NQ/...)
 * is in CPM CPPSR's low byte and the EFUSE's SUBSOCTYPE words.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_HARB "ingenic-harb"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicHarbState, INGENIC_HARB)

#define HARB_REGS       0x100
#define R_SOC_ID        0x2C

struct IngenicHarbState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t soc_id;
    uint32_t regs[HARB_REGS / 4];
};

static uint64_t ingenic_harb_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicHarbState *s = opaque;

    if (off == R_SOC_ID) {
        return s->soc_id;
    }
    return off < HARB_REGS ? s->regs[off >> 2] : 0;
}

static void ingenic_harb_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicHarbState *s = opaque;

    if (off < HARB_REGS && off != R_SOC_ID) {
        s->regs[off >> 2] = val;
    }
}

static const MemoryRegionOps ingenic_harb_ops = {
    .read = ingenic_harb_read,
    .write = ingenic_harb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_harb_reset(DeviceState *dev)
{
    IngenicHarbState *s = INGENIC_HARB(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ingenic_harb_init(Object *obj)
{
    IngenicHarbState *s = INGENIC_HARB(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_harb_ops, s,
                          TYPE_INGENIC_HARB, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property ingenic_harb_props[] = {
    DEFINE_PROP_UINT32("soc-id", IngenicHarbState, soc_id, 0x10031003),
};

static const VMStateDescription vmstate_ingenic_harb = {
    .name = TYPE_INGENIC_HARB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicHarbState, HARB_REGS / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_harb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_harb_reset);
    device_class_set_props(dc, ingenic_harb_props);
    dc->vmsd = &vmstate_ingenic_harb;
}

static const TypeInfo ingenic_harb_info = {
    .name = TYPE_INGENIC_HARB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicHarbState),
    .instance_init = ingenic_harb_init,
    .class_init = ingenic_harb_class_init,
};

static void ingenic_harb_register_types(void)
{
    type_register_static(&ingenic_harb_info);
}

type_init(ingenic_harb_register_types)
