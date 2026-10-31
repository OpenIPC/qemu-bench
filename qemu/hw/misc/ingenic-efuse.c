/*
 * Ingenic EFUSE controller at 0x13540000, unprogrammed.
 *
 *   CTRL 0x00 (address, length, 0 read / 1 write enable, self-clearing)
 *   CFG 0x04
 *   STATE 0x08  0 read done, 1 write done            DATA 0x0C..
 *   0x200..     the fuse array, also read directly
 *
 * The driver (drivers/misc/jz_efuse_v13.c) and the vendor kernels' boot
 * code wait for STATE bit 0 after every read and bit 1 after every
 * program, with no timeout, so both read as done; the vendor kernels also
 * wait for CTRL's enable bit to drop before the next read.  The fuses read zero:
 * the MAC's PHY clock selection on T41 (0x22C, 0x250), and
 * the hash word (0x23C) that the binary-only MXU check in the 3.10
 * kernels (arch/mips/xburst/core/mxu-v2-ex.obj) compares only when it is
 * not zero.  Programming is accepted and changes nothing.
 *
 * The chip ID -- the 96-bit segment at fuse offset 0 (0x200..0x208,
 * jz_efuse_v13.c's CHIP_ID; libsysutils' SU_Base_GetDevID, the SDKs'
 * sample-chipid, /proc/jz/efuse/efuse_chip_id) -- is the "chipid0..2"
 * properties: unique per die on silicon, a fixed value per machine here
 * (the SoC ID, then "OpenIPC!"), -global ingenic-efuse.chipidN=... to
 * set another.
 *
 * Two words are not blank: the grade codes that tell a T31N from a T31X
 * or a T40XP from a T40A (ipctool src/hal/ingenic.c) -- SUBSOCTYPE1 at
 * 0x238 on T10..T31, SUBSOCTYPE2 at 0x250 on XBurst2, the code in the
 * upper half.  A word reads both directly at 0x200 + 4 n and through
 * CTRL (index in bits 27:21) and DATA.
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

#define TYPE_INGENIC_EFUSE "ingenic-efuse"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicEfuseState, INGENIC_EFUSE)

#define R_CTRL      0x00
#define R_CFG       0x04
#define R_STATE     0x08
#define R_DATA      0x0C
#define R_FUSES     0x200
#define R_CHIPID    0x200       /* three words */
#define R_SUBSOCTYPE1   0x238
#define R_SUBSOCTYPE2   0x250

#define CTRL_RD_EN      (1u << 0)
#define CTRL_WR_EN      (1u << 1)
#define STATE_RD_DONE   (1u << 0)
#define STATE_WR_DONE   (1u << 1)

struct IngenicEfuseState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t ctrl, cfg;
    uint32_t subsoctype1, subsoctype2;
    uint32_t chipid[3];
};

static uint32_t efuse_word(IngenicEfuseState *s, hwaddr off)
{
    switch (off & ~3) {
    case R_CHIPID ... R_CHIPID + 8:
        return s->chipid[((off & ~3) - R_CHIPID) >> 2];
    case R_SUBSOCTYPE1:
        return s->subsoctype1;
    case R_SUBSOCTYPE2:
        return s->subsoctype2;
    default:
        return 0;
    }
}

static uint64_t ingenic_efuse_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicEfuseState *s = opaque;

    switch (off) {
    case R_CTRL:
        return s->ctrl & ~(CTRL_RD_EN | CTRL_WR_EN);
    case R_CFG:
        return s->cfg;
    case R_STATE:
        return STATE_RD_DONE | STATE_WR_DONE;
    case R_DATA:
        return efuse_word(s, R_FUSES + 4 * extract32(s->ctrl, 21, 7));
    default:
        return extract32(efuse_word(s, off), (off & 3) * 8, size * 8);
    }
}

static void ingenic_efuse_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    IngenicEfuseState *s = opaque;

    switch (off) {
    case R_CTRL:
        s->ctrl = val;
        break;
    case R_CFG:
        s->cfg = val;
        break;
    }
}

static const MemoryRegionOps ingenic_efuse_ops = {
    .read = ingenic_efuse_read,
    .write = ingenic_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.unaligned = true,
};

static void ingenic_efuse_reset(DeviceState *dev)
{
    IngenicEfuseState *s = INGENIC_EFUSE(dev);

    s->ctrl = s->cfg = 0;
}

static void ingenic_efuse_init(Object *obj)
{
    IngenicEfuseState *s = INGENIC_EFUSE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_efuse_ops, s,
                          TYPE_INGENIC_EFUSE, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ingenic_efuse = {
    .name = TYPE_INGENIC_EFUSE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicEfuseState),
        VMSTATE_UINT32(cfg, IngenicEfuseState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_efuse_props[] = {
    DEFINE_PROP_UINT32("subsoctype1", IngenicEfuseState, subsoctype1, 0),
    DEFINE_PROP_UINT32("subsoctype2", IngenicEfuseState, subsoctype2, 0),
    DEFINE_PROP_UINT32("chipid0", IngenicEfuseState, chipid[0], 0),
    DEFINE_PROP_UINT32("chipid1", IngenicEfuseState, chipid[1], 0x6e65704f),
    DEFINE_PROP_UINT32("chipid2", IngenicEfuseState, chipid[2], 0x21435049),
};

static void ingenic_efuse_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_efuse_reset);
    device_class_set_props(dc, ingenic_efuse_props);
    dc->vmsd = &vmstate_ingenic_efuse;
}

static const TypeInfo ingenic_efuse_info = {
    .name = TYPE_INGENIC_EFUSE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicEfuseState),
    .instance_init = ingenic_efuse_init,
    .class_init = ingenic_efuse_class_init,
};

static void ingenic_efuse_register_types(void)
{
    type_register_static(&ingenic_efuse_info);
}

type_init(ingenic_efuse_register_types)
