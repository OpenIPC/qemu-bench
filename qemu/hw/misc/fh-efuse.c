/*
 * Fullhan eFuse controller (0xF1600000; the kernel's drivers/misc/
 * fh_efuse.c, and the FH865x ramboot, which reads it before U-Boot).
 *
 *   0x00 CMD      bits 3:0 the operation, 9:4 the entry, 19:12 a byte to
 *                 program: 1 load the user bits, 2 program a byte, 3 read
 *                 an entry into DOUT, 4 move AES keys, 8 check which bits
 *                 can still be programmed
 *   0x04 CONFIG   0x08 MATCH_KEY   0x0C-0x24 TIMING0-6
 *   0x28 DOUT     0x2C STATUS0: bit <op> once that operation is done,
 *                 23:20 / 27:24 the APB / AES lock bits
 *   0x30 / 0x34 STATUS1-2: the programmable-bit map of the last check
 *
 * Operations finish at once.  The 64 entries start blank, as on a part
 * nobody has programmed -- no secure boot, no keys -- and programming
 * ORs bits in.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_FH_EFUSE "fh-efuse"
OBJECT_DECLARE_SIMPLE_TYPE(FhEfuseState, FH_EFUSE)

#define FH_EFUSE_SIZE       0x1000
#define FH_EFUSE_ENTRIES    64

#define R_CMD               0x00
#define R_DOUT              0x28
#define R_STATUS0           0x2C
#define R_STATUS1           0x30
#define R_STATUS2           0x34
#define R_LAST              0x40

#define OP_LOAD             1
#define OP_WRITE            2
#define OP_READ             3
#define OP_AES              4
#define OP_CHECK            8

struct FhEfuseState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[R_LAST / 4];
    uint8_t fuse[FH_EFUSE_ENTRIES];
};

static void fh_efuse_cmd(FhEfuseState *s, uint32_t cmd)
{
    unsigned op = cmd & 0xF, entry = (cmd >> 4) & 0x3F;

    switch (op) {
    case OP_WRITE:
        s->fuse[entry] |= (cmd >> 12) & 0xFF;
        break;
    case OP_READ:
        s->regs[R_DOUT >> 2] = s->fuse[entry];
        break;
    case OP_CHECK:
        s->regs[R_STATUS1 >> 2] = 0xFFFFFFFF;
        s->regs[R_STATUS2 >> 2] = 0xFFFFFFFF;
        break;
    }
    if (op < 32) {
        s->regs[R_STATUS0 >> 2] = (s->regs[R_STATUS0 >> 2] & 0x0FF00000) |
                                  (1u << op);
    }
}

static uint64_t fh_efuse_read(void *opaque, hwaddr off, unsigned size)
{
    FhEfuseState *s = opaque;

    return off < R_LAST ? s->regs[off >> 2] : 0;
}

static void fh_efuse_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    FhEfuseState *s = opaque;

    switch (off) {
    case R_CMD:
        s->regs[0] = val;
        fh_efuse_cmd(s, val);
        break;
    case R_DOUT ... R_STATUS2:
        break;
    default:
        if (off < R_LAST) {
            s->regs[off >> 2] = val;
        }
        break;
    }
}

static const MemoryRegionOps fh_efuse_ops = {
    .read = fh_efuse_read,
    .write = fh_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_efuse_reset(DeviceState *dev)
{
    FhEfuseState *s = FH_EFUSE(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void fh_efuse_init(Object *obj)
{
    FhEfuseState *s = FH_EFUSE(obj);

    memory_region_init_io(&s->iomem, obj, &fh_efuse_ops, s, TYPE_FH_EFUSE,
                          FH_EFUSE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_fh_efuse = {
    .name = TYPE_FH_EFUSE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhEfuseState, R_LAST / 4),
        VMSTATE_UINT8_ARRAY(fuse, FhEfuseState, FH_EFUSE_ENTRIES),
        VMSTATE_END_OF_LIST()
    }
};

static void fh_efuse_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, fh_efuse_reset);
    dc->vmsd = &vmstate_fh_efuse;
}

static const TypeInfo fh_efuse_info = {
    .name           = TYPE_FH_EFUSE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(FhEfuseState),
    .instance_init  = fh_efuse_init,
    .class_init     = fh_efuse_class_init,
};

static void fh_efuse_register_types(void)
{
    type_register_static(&fh_efuse_info);
}

type_init(fh_efuse_register_types)
