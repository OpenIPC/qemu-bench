/*
 * Ingenic DDR controller and PHY, as far as the SPL's DDR init sees them.
 *
 * Two windows: the PHY and the DDRC's APB side at 0x13010000-0x13012FFF
 * (PHY at +0x1000, APB at +0x2000), and the DDRC proper at 0x134F0000.
 * Every register is storage; what the SPL polls (no timeouts, or a hang on
 * expiry) reads as finished.  Per family (U-Boot arch/mips/cpu/xburst/
 * ddr_dwc.c, ddr_innophy.c, xburst2/ddr_innophy.c):
 *
 *   DWC PHY (T10, T20)    PHY+0x0C PGSR = IDONE|DLDONE|ZCDONE|DIDONE|DTDONE,
 *                         no DTERR/DTIERR; PHY+0x188 ZQXSR0 bit 30 clear
 *   Inno PHY (T21..T31)   PHY+0xC8 bit 3 PLL lock, APB+0x8C bits 1-2,
 *                         PHY+0xD0 != 0, PHY+0xCC == 3 and PHY+0xC0 == 3
 *                         (calibration, write levelling), DDRC LMR (+0x0C)
 *                         bit 0 self-clearing
 *   T40                   PHY+0x108 bit 3, PHY+0x10C low nibble 0xF,
 *                         APB+0x04 bit 0 (DFI init), DDRC LMR (+0x18)
 *   T41                   PHY+0x180 bit 2, PHY+0x184 low nibble 3,
 *                         APB+0x04 bit 0, DDRC LMR (+0x18)
 *
 * The remap registers the SPL swaps around are kept as written and remap
 * nothing.  The DRAM itself is the machine's RAM: present from power-on,
 * sized by the machine, not by these registers.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/mips/ingenic.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_DDR "ingenic-ddr"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDdrState, INGENIC_DDR)

#define PHY_WIN         0x3000      /* 0x13010000 */
#define DDRC_WIN        0x1000      /* 0x134F0000 */
#define PHY             0x1000
#define APB             0x2000

typedef struct DdrForce {
    uint16_t off;
    uint32_t mask, val;     /* read = (stored & ~mask) | val */
} DdrForce;

struct IngenicDdrState {
    SysBusDevice parent_obj;

    MemoryRegion phy_mr, ddrc_mr;
    uint32_t soc;
    uint32_t phy[PHY_WIN / 4];
    uint32_t ddrc[DDRC_WIN / 4];
};

static const DdrForce force_dwc[] = {
    { PHY + 0x0C, 0x7f, 0x1f },
    { PHY + 0x188, 1u << 30, 0 },
};
static const DdrForce force_inno[] = {
    { PHY + 0xC8, 1u << 3, 1u << 3 },
    { PHY + 0xD0, 1u << 0, 1u << 0 },
    { PHY + 0xCC, 0xffffffff, 3 },
    { PHY + 0xC0, 0xffffffff, 3 },
    { APB + 0x8C, 6, 6 },
};
static const DdrForce force_t40[] = {
    { PHY + 0x108, 1u << 3, 1u << 3 },
    { PHY + 0x10C, 0xf, 0xf },
    { APB + 0x04, 1, 1 },
};
static const DdrForce force_t41[] = {
    { PHY + 0x180, 1u << 2, 1u << 2 },
    { PHY + 0x184, 0xf, 3 },
    { APB + 0x04, 1, 1 },
};

static void ddr_forces(IngenicDdrState *s, const DdrForce **f, int *n)
{
    switch (s->soc) {
    case INGENIC_T10:
    case INGENIC_T20:
        *f = force_dwc;
        *n = ARRAY_SIZE(force_dwc);
        break;
    case INGENIC_T40:
        *f = force_t40;
        *n = ARRAY_SIZE(force_t40);
        break;
    case INGENIC_T41:
        *f = force_t41;
        *n = ARRAY_SIZE(force_t41);
        break;
    default:
        *f = force_inno;
        *n = ARRAY_SIZE(force_inno);
        break;
    }
}

static uint64_t ingenic_ddr_phy_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDdrState *s = opaque;
    uint32_t v = s->phy[off >> 2];
    const DdrForce *f;
    int i, n;

    ddr_forces(s, &f, &n);
    for (i = 0; i < n; i++) {
        if (f[i].off == (off & ~3)) {
            v = (v & ~f[i].mask) | f[i].val;
        }
    }
    return v;
}

static void ingenic_ddr_phy_write(void *opaque, hwaddr off, uint64_t val,
                                  unsigned size)
{
    IngenicDdrState *s = opaque;

    s->phy[off >> 2] = val;
}

/* LMR's start bit: +0x0C on the T10..T31 DDRC, +0x18 on XBurst2 */
static hwaddr ddr_lmr(IngenicDdrState *s)
{
    return ingenic_is_xburst2(s->soc) ? 0x18 : 0x0C;
}

static uint64_t ingenic_ddrc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDdrState *s = opaque;
    uint32_t v = s->ddrc[off >> 2];

    if ((off & ~3) == ddr_lmr(s)) {
        v &= ~1u;
    }
    return v;
}

static void ingenic_ddrc_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicDdrState *s = opaque;

    s->ddrc[off >> 2] = val;
}

static const MemoryRegionOps ingenic_ddr_phy_ops = {
    .read = ingenic_ddr_phy_read,
    .write = ingenic_ddr_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const MemoryRegionOps ingenic_ddrc_ops = {
    .read = ingenic_ddrc_read,
    .write = ingenic_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_ddr_reset(DeviceState *dev)
{
    IngenicDdrState *s = INGENIC_DDR(dev);

    memset(s->phy, 0, sizeof(s->phy));
    memset(s->ddrc, 0, sizeof(s->ddrc));
}

static void ingenic_ddr_init(Object *obj)
{
    IngenicDdrState *s = INGENIC_DDR(obj);

    memory_region_init_io(&s->phy_mr, obj, &ingenic_ddr_phy_ops, s,
                          "ingenic-ddr-phy", PHY_WIN);
    memory_region_init_io(&s->ddrc_mr, obj, &ingenic_ddrc_ops, s,
                          "ingenic-ddrc", DDRC_WIN);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->phy_mr);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->ddrc_mr);
}

static const Property ingenic_ddr_props[] = {
    DEFINE_PROP_UINT32("soc", IngenicDdrState, soc, INGENIC_T31),
};

static const VMStateDescription vmstate_ingenic_ddr = {
    .name = TYPE_INGENIC_DDR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(phy, IngenicDdrState, PHY_WIN / 4),
        VMSTATE_UINT32_ARRAY(ddrc, IngenicDdrState, DDRC_WIN / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ddr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_ddr_reset);
    device_class_set_props(dc, ingenic_ddr_props);
    dc->vmsd = &vmstate_ingenic_ddr;
}

static const TypeInfo ingenic_ddr_info = {
    .name = TYPE_INGENIC_DDR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDdrState),
    .instance_init = ingenic_ddr_init,
    .class_init = ingenic_ddr_class_init,
};

static void ingenic_ddr_register_types(void)
{
    type_register_static(&ingenic_ddr_info);
}

type_init(ingenic_ddr_register_types)
