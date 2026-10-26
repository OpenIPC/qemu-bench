/*
 * Grain Media FTDDRC010 DDR controller.
 *
 * nsboot (set_ddr() in entry.c) writes one of its DDR_arg tables into the
 * controller at 0x9A100000, writes 1 to 0x04 and polls bit 8 there for
 * the initialisation to finish.  Register 0x10's low bits hold the DRAM
 * size -- 2 for 64 MiB, 3 for 128 MiB, 4 for 256 MiB, 5 for 512 MiB in
 * the tables -- which U-Boot's commandline_mode() reads to pick the
 * kernel command line (cmd1..cmd4) for the fitted memory.  It comes out
 * of reset with the machine's RAM size, so a boot that skips nsboot sees
 * the same; nsboot then writes its own table's value.  Everything else is
 * storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GM_DDRC "gm-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(GmDdrcState, GM_DDRC)

#define GM_DDRC_SIZE        0x1000
#define GM_DDRC_NREGS       (0x200 / 4)

#define R_CMD               0x04
#define R_CONFIG            0x10

#define CMD_INIT            (1u << 0)
#define CMD_INIT_DONE       (1u << 8)

struct GmDdrcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[GM_DDRC_NREGS];
    uint64_t ram_size;
};

static uint64_t gm_ddrc_read(void *opaque, hwaddr off, unsigned size)
{
    GmDdrcState *s = opaque;

    return off < GM_DDRC_NREGS * 4 ? s->regs[off / 4] : 0;
}

static void gm_ddrc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmDdrcState *s = opaque;

    if (off >= GM_DDRC_NREGS * 4) {
        return;
    }
    if (off == R_CMD && (val & CMD_INIT)) {
        val |= CMD_INIT_DONE;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps gm_ddrc_ops = {
    .read = gm_ddrc_read,
    .write = gm_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_ddrc_reset(DeviceState *dev)
{
    GmDdrcState *s = GM_DDRC(dev);
    uint32_t code = 2;          /* 64 MiB */

    while (code < 7 && (64 * MiB << (code - 2)) < s->ram_size) {
        code++;
    }
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_CMD / 4] = CMD_INIT_DONE;
    s->regs[R_CONFIG / 4] = (code << 12) | (code << 8) | (code << 4) | code;
}

static void gm_ddrc_realize(DeviceState *dev, Error **errp)
{
    GmDdrcState *s = GM_DDRC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_ddrc_ops, s,
                          TYPE_GM_DDRC, GM_DDRC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property gm_ddrc_props[] = {
    DEFINE_PROP_UINT64("ram-size", GmDdrcState, ram_size, 128 * MiB),
};

static const VMStateDescription vmstate_gm_ddrc = {
    .name = TYPE_GM_DDRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, GmDdrcState, GM_DDRC_NREGS),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_ddrc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_ddrc_realize;
    device_class_set_legacy_reset(dc, gm_ddrc_reset);
    device_class_set_props(dc, gm_ddrc_props);
    dc->vmsd = &vmstate_gm_ddrc;
}

static const TypeInfo gm_ddrc_info = {
    .name           = TYPE_GM_DDRC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmDdrcState),
    .class_init     = gm_ddrc_class_init,
};

static void gm_ddrc_register_types(void)
{
    type_register_static(&gm_ddrc_info);
}

type_init(gm_ddrc_register_types)
