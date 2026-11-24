/*
 * Faraday FTSDC010 SD host of the Grain Media GM8126 (0x92800000), with no
 * card in the slot.
 *
 * Enough for ftsdc010.ko to probe and find the slot empty: the reset and
 * command bits of 0x00 clear themselves, every command times out (status
 * 0x28 bit 2, cleared through 0x2C, masked by 0x30), status bit 17 (bus
 * idle, which the driver waits for after each request) is set and bit 11
 * (a card in a micro-SD slot) clear.  Cards are not modelled.
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

#define TYPE_GM_SDC010 "gm-sdc010"
OBJECT_DECLARE_SIMPLE_TYPE(GmSdc010State, GM_SDC010)

#define SDC_SIZE    0x1000
#define SDC_NREGS   (0x100 / 4)

#define R_CMD       0x00
#define R_STATUS    0x28
#define R_CLEAR     0x2C
#define R_MASK      0x30

struct GmSdc010State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[SDC_NREGS];
};

static void sdc_update(GmSdc010State *s)
{
    qemu_set_irq(s->irq,
                 (s->regs[R_STATUS / 4] & s->regs[R_MASK / 4]) != 0);
}

static uint64_t sdc_read(void *opaque, hwaddr off, unsigned size)
{
    GmSdc010State *s = opaque;

    if (off == R_STATUS) {
        return s->regs[R_STATUS / 4] | (1u << 17);
    }
    return off / 4 < SDC_NREGS ? s->regs[off / 4] : 0;
}

static void sdc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    GmSdc010State *s = opaque;

    switch (off) {
    case R_CMD:
        if (val & (1u << 10)) {
            s->regs[R_STATUS / 4] = 0;
        } else if (val & (1u << 9)) {
            s->regs[R_STATUS / 4] |= 1u << 2;
        }
        s->regs[R_CMD / 4] = val & ~0x600u;
        break;
    case R_STATUS:
        break;
    case R_CLEAR:
        s->regs[R_STATUS / 4] &= ~val;
        break;
    default:
        if (off / 4 < SDC_NREGS) {
            s->regs[off / 4] = val;
        }
    }
    sdc_update(s);
}

static const MemoryRegionOps sdc_ops = {
    .read = sdc_read,
    .write = sdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void sdc_reset(DeviceState *dev)
{
    GmSdc010State *s = GM_SDC010(dev);

    memset(s->regs, 0, sizeof(s->regs));
    sdc_update(s);
}

static void sdc_realize(DeviceState *dev, Error **errp)
{
    GmSdc010State *s = GM_SDC010(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &sdc_ops, s, TYPE_GM_SDC010,
                          SDC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const VMStateDescription vmstate_sdc = {
    .name = TYPE_GM_SDC010,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmSdc010State, SDC_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void sdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sdc_realize;
    device_class_set_legacy_reset(dc, sdc_reset);
    dc->vmsd = &vmstate_sdc;
}

static const TypeInfo sdc_info = {
    .name = TYPE_GM_SDC010,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GmSdc010State),
    .class_init = sdc_class_init,
};

static void sdc_register_types(void)
{
    type_register_static(&sdc_info);
}

type_init(sdc_register_types)
