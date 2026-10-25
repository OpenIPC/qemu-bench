/*
 * Allwinner DRAM controllers: the DesignWare-PUB-like PHY of the H3
 * generation (V3s) and of the D1 generation (V831/V833, V851s), and the
 * geometry register in front of them.
 *
 * aw-dramc, the controller/PHY block (0x01C63000 V3s, 0x04003000 V831,
 * 0x03103000 V851s): storage for every register, plus the status the
 * training waits on.  No DRAM to train, so every step is done at once.
 *
 *   0x010 PGSR0  bit 0 IDONE once a PIR (0x000) step is started
 *   0x018 STATR  bits 2:0 = 1, normal operating mode
 *
 * A training step reports what a single-rank, full-width part reports when
 * DTCR (0x0C0) bits 25:24 ask for two ranks: a DQS gate error (PGSR0 bit
 * 22) and rank 1 missing in both byte lanes' GSR0 (bits 25:24 = 2, at
 * 0x348 and 0x3C8).  That is how boot0's libdram, mainline's D1 driver and
 * mainline's DesignWare driver find out the rank count.
 *
 * aw-dramcom, the "COM" block (0x01C62000, 0x04002000, 0x03102000):
 * storage, and the address geometry its register 0 sets (the same fields
 * in all three: rank 1:0, eight banks bit 2, rows - 1 in 7:4, page size
 * code in 11:8).  The drivers size the DRAM by setting a wide geometry and
 * seeing where a pattern repeats.  While the geometry set is not the
 * part's, the DRAM window is translated as the controller does it: row,
 * bank and column taken by the geometry set, the bits the part does not
 * have dropped, so the pattern repeats where it would on silicon.  With
 * the part's geometry (or none yet) the RAM is mapped as it is.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define TYPE_AW_DRAMC "aw-dramc"
OBJECT_DECLARE_SIMPLE_TYPE(AwDramcState, AW_DRAMC)

#define AW_DRAMC_SIZE       0x2000

#define AW_DRAMC_PIR        0x000
#define AW_DRAMC_PGSR0      0x010
#define AW_DRAMC_STATR      0x018
#define AW_DRAMC_DTCR       0x0c0
#define AW_DRAMC_DX0GSR0    0x348
#define AW_DRAMC_DX1GSR0    0x3c8

#define AW_PGSR0_QSGERR     (1u << 22)
#define AW_DXGSR0_RANK1     (2u << 24)

struct AwDramcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[AW_DRAMC_SIZE / 4];
    uint32_t ranks;             /* ranks the part has */
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

/* the outcome of a training step: rank 1 answers only if the part has it */
static void aw_dramc_train(AwDramcState *s)
{
    bool rank1 = (s->regs[AW_DRAMC_DTCR >> 2] >> 24) & 2;
    bool missing = rank1 && s->ranks < 2;
    uint32_t *pgsr0 = &s->regs[AW_DRAMC_PGSR0 >> 2];
    uint32_t *dx0 = &s->regs[AW_DRAMC_DX0GSR0 >> 2];
    uint32_t *dx1 = &s->regs[AW_DRAMC_DX1GSR0 >> 2];

    *pgsr0 &= ~(0xffu << 20);
    *dx0 &= ~(3u << 24);
    *dx1 &= ~(3u << 24);
    if (missing) {
        *pgsr0 |= AW_PGSR0_QSGERR;
        *dx0 |= AW_DXGSR0_RANK1;
        *dx1 |= AW_DXGSR0_RANK1;
    }
}

static void aw_dramc_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    AwDramcState *s = opaque;

    switch (addr) {
    case AW_DRAMC_PIR:
        /* The step runs and finishes: INIT (bit 0) clears itself. */
        if (val & 1) {
            aw_dramc_train(s);
        }
        val &= ~1u;
        break;
    case AW_DRAMC_PGSR0:
        return;
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

static const Property aw_dramc_properties[] = {
    DEFINE_PROP_UINT32("ranks", AwDramcState, ranks, 1),
};

static void aw_dramc_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(DEVICE_CLASS(klass), aw_dramc_properties);
    rc->phases.hold = aw_dramc_reset_hold;
}

static const TypeInfo aw_dramc_info = {
    .name           = TYPE_AW_DRAMC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwDramcState),
    .instance_init  = aw_dramc_init,
    .class_init     = aw_dramc_class_init,
};

/* ---------------------------------------------------------------------- */

#define TYPE_AW_DRAMCOM "aw-dramcom"
OBJECT_DECLARE_SIMPLE_TYPE(AwDramcomState, AW_DRAMCOM)

#define AW_DRAMCOM_SIZE     0x1000
#define AW_DRAM_BASE        0x40000000
#define AW_DRAM_WINDOW      0x80000000

struct AwDramcomState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion xlate;         /* the window, while the geometry differs */
    MemoryRegion *ram;
    uint8_t *ram_ptr;
    uint64_t ram_size;
    uint32_t regs[AW_DRAMCOM_SIZE / 4];
    /* the part's geometry */
    uint32_t page;              /* bytes per row of the bus */
    uint32_t bank_bits;
    uint32_t row_bits;
    /* the geometry register 0 sets */
    uint32_t cfg_page, cfg_bank_bits, cfg_row_bits;
};

static hwaddr aw_dramcom_map(AwDramcomState *s, hwaddr off)
{
    uint64_t col = off % s->cfg_page;
    uint64_t line = off / s->cfg_page;
    uint64_t bank = line & ((1u << s->cfg_bank_bits) - 1);
    uint64_t row = (line >> s->cfg_bank_bits) &
                   ((1ull << s->cfg_row_bits) - 1);

    col &= s->page - 1;
    bank &= (1u << s->bank_bits) - 1;
    row &= (1ull << s->row_bits) - 1;

    return (((row << s->bank_bits) | bank) * s->page + col) % s->ram_size;
}

static uint64_t aw_dramcom_xlate_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    AwDramcomState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val |= (uint64_t)s->ram_ptr[aw_dramcom_map(s, addr + i)] << (8 * i);
    }
    return val;
}

static void aw_dramcom_xlate_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    AwDramcomState *s = opaque;
    unsigned i;

    for (i = 0; i < size; i++) {
        s->ram_ptr[aw_dramcom_map(s, addr + i)] = val >> (8 * i);
    }
}

static const MemoryRegionOps aw_dramcom_xlate_ops = {
    .read = aw_dramcom_xlate_read,
    .write = aw_dramcom_xlate_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static void aw_dramcom_geometry(AwDramcomState *s, uint32_t cr)
{
    uint32_t page_code = (cr >> 8) & 0xf;
    bool differs;

    /* no geometry set yet: the RAM as it is */
    if (page_code < 6 || page_code > 10 || ((cr >> 4) & 0xf) < 10) {
        memory_region_set_enabled(&s->xlate, false);
        return;
    }
    s->cfg_page = 1u << (page_code + 3);
    s->cfg_bank_bits = (cr & 4) ? 3 : 2;
    s->cfg_row_bits = ((cr >> 4) & 0xf) + 1;

    differs = s->cfg_page != s->page || s->cfg_bank_bits != s->bank_bits ||
              s->cfg_row_bits != s->row_bits;
    memory_region_set_enabled(&s->xlate, differs);
}

static uint64_t aw_dramcom_read(void *opaque, hwaddr addr, unsigned size)
{
    AwDramcomState *s = opaque;

    return s->regs[addr >> 2];
}

static void aw_dramcom_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    AwDramcomState *s = opaque;

    s->regs[addr >> 2] = val;
    if (addr == 0) {
        aw_dramcom_geometry(s, val);
    }
}

static const MemoryRegionOps aw_dramcom_ops = {
    .read = aw_dramcom_read,
    .write = aw_dramcom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_dramcom_realize(DeviceState *dev, Error **errp)
{
    AwDramcomState *s = AW_DRAMCOM(dev);

    if (!s->ram) {
        error_setg(errp, "aw-dramcom: needs the DRAM ('ram' link)");
        return;
    }
    s->ram_ptr = memory_region_get_ram_ptr(s->ram);
    s->ram_size = memory_region_size(s->ram);
    memory_region_init_io(&s->xlate, OBJECT(s), &aw_dramcom_xlate_ops, s,
                          "aw-dramcom.xlate", AW_DRAM_WINDOW);
    memory_region_add_subregion_overlap(get_system_memory(), AW_DRAM_BASE,
                                        &s->xlate, 1);
    memory_region_set_enabled(&s->xlate, false);
}

static void aw_dramcom_reset_hold(Object *obj, ResetType type)
{
    AwDramcomState *s = AW_DRAMCOM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    if (s->ram_ptr) {
        memory_region_set_enabled(&s->xlate, false);
    }
}

static void aw_dramcom_init(Object *obj)
{
    AwDramcomState *s = AW_DRAMCOM(obj);

    memory_region_init_io(&s->iomem, obj, &aw_dramcom_ops, s,
                          TYPE_AW_DRAMCOM, AW_DRAMCOM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property aw_dramcom_properties[] = {
    DEFINE_PROP_LINK("ram", AwDramcomState, ram, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("page", AwDramcomState, page, 2048),
    DEFINE_PROP_UINT32("bank-bits", AwDramcomState, bank_bits, 2),
    DEFINE_PROP_UINT32("row-bits", AwDramcomState, row_bits, 13),
};

static void aw_dramcom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aw_dramcom_realize;
    device_class_set_props(dc, aw_dramcom_properties);
    rc->phases.hold = aw_dramcom_reset_hold;
}

static const TypeInfo aw_dramcom_info = {
    .name           = TYPE_AW_DRAMCOM,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwDramcomState),
    .instance_init  = aw_dramcom_init,
    .class_init     = aw_dramcom_class_init,
};

static void aw_dramc_register_types(void)
{
    type_register_static(&aw_dramc_info);
    type_register_static(&aw_dramcom_info);
}

type_init(aw_dramc_register_types)
