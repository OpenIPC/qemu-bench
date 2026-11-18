/*
 * Allwinner sunxi IOMMU (V831/V833 at 0x030F0000; drivers/iommu/
 * sunxi-iommu.c in the vendor kernel, sun50i-iommu.c in mainline).
 *
 *   0x020 ENABLE    bit 0
 *   0x030 BYPASS    one bit per master: that master's addresses are physical
 *   0x050 TTB       the page directory, physical
 *   0x080 TLB_FLUSH, 0x098 TLB_IVLD, 0x0A8 PC_IVLD: start bits that the
 *                   driver polls, done at once
 *
 * Two levels: 4096 directory entries of 1 MiB (bits 31:10 point at a
 * 256-entry table, bits 1:0 = 1 when valid), then 4 KiB pages (bits
 * 31:12, valid bit 1).  Nothing
 * is cached, so invalidations have nothing to do.
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
#include "hw/misc/aw-iommu.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(AwIommuState, AW_IOMMU)

#define AW_IOMMU_SIZE       0x400
#define REG_ENABLE          0x020
#define REG_BYPASS          0x030
#define REG_TTB             0x050
#define REG_TLB_FLUSH       0x080
#define REG_TLB_IVLD        0x098
#define REG_PC_IVLD         0x0A8

#define PTE_VALID           (1u << 1)

struct AwIommuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[AW_IOMMU_SIZE / 4];
};

static bool aw_iommu_active(AwIommuState *s, int master)
{
    return s && (s->regs[REG_ENABLE >> 2] & 1) &&
           !(s->regs[REG_BYPASS >> 2] & (1u << master));
}

/* Physical address of @iova, or false if it is not mapped. */
static bool aw_iommu_walk(AwIommuState *s, uint32_t iova, hwaddr *pa)
{
    hwaddr ttb = s->regs[REG_TTB >> 2] & ~0x3FFFu;
    uint32_t pde, pte;

    pde = address_space_ldl_le(&address_space_memory,
                               ttb + (iova >> 20) * 4,
                               MEMTXATTRS_UNSPECIFIED, NULL);
    if ((pde & 3) != 1) {
        return false;
    }
    pte = address_space_ldl_le(&address_space_memory,
                               (pde & ~0x3FFu) + ((iova >> 12) & 0xFF) * 4,
                               MEMTXATTRS_UNSPECIFIED, NULL);
    if (!(pte & PTE_VALID)) {
        return false;
    }
    *pa = (pte & ~0xFFFu) | (iova & 0xFFF);
    return true;
}

static void aw_iommu_dma(DeviceState *dev, int master, uint32_t addr,
                         void *buf, uint32_t len, bool write)
{
    AwIommuState *s = dev ? AW_IOMMU(dev) : NULL;
    uint8_t *p = buf;

    if (!aw_iommu_active(s, master)) {
        dma_memory_rw(&address_space_memory, addr, buf, len,
                      write ? DMA_DIRECTION_FROM_DEVICE
                            : DMA_DIRECTION_TO_DEVICE,
                      MEMTXATTRS_UNSPECIFIED);
        return;
    }
    while (len) {
        uint32_t chunk = MIN(len, 0x1000 - (addr & 0xFFF));
        hwaddr pa;

        if (aw_iommu_walk(s, addr, &pa)) {
            dma_memory_rw(&address_space_memory, pa, p, chunk,
                          write ? DMA_DIRECTION_FROM_DEVICE
                                : DMA_DIRECTION_TO_DEVICE,
                          MEMTXATTRS_UNSPECIFIED);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "aw-iommu: master %d: 0x%08x not mapped\n",
                          master, addr);
        }
        addr += chunk;
        p += chunk;
        len -= chunk;
    }
}

void aw_iommu_dma_write(DeviceState *iommu, int master, uint32_t addr,
                        const void *buf, uint32_t len)
{
    aw_iommu_dma(iommu, master, addr, (void *)buf, len, true);
}

void aw_iommu_dma_read(DeviceState *iommu, int master, uint32_t addr,
                       void *buf, uint32_t len)
{
    aw_iommu_dma(iommu, master, addr, buf, len, false);
}

static uint64_t aw_iommu_read(void *opaque, hwaddr addr, unsigned size)
{
    AwIommuState *s = opaque;

    return s->regs[addr >> 2];
}

static void aw_iommu_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    AwIommuState *s = opaque;

    switch (addr) {
    case REG_TLB_FLUSH:
    case REG_TLB_IVLD:
    case REG_PC_IVLD:
        val = 0;            /* done at once: nothing is cached */
        break;
    }
    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps aw_iommu_ops = {
    .read = aw_iommu_read,
    .write = aw_iommu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_iommu_reset_hold(Object *obj, ResetType type)
{
    AwIommuState *s = AW_IOMMU(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void aw_iommu_init(Object *obj)
{
    AwIommuState *s = AW_IOMMU(obj);

    memory_region_init_io(&s->iomem, obj, &aw_iommu_ops, s, TYPE_AW_IOMMU,
                          AW_IOMMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void aw_iommu_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_iommu_reset_hold;
}

static const TypeInfo aw_iommu_info = {
    .name           = TYPE_AW_IOMMU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwIommuState),
    .instance_init  = aw_iommu_init,
    .class_init     = aw_iommu_class_init,
};

static void aw_iommu_register_types(void)
{
    type_register_static(&aw_iommu_info);
}

type_init(aw_iommu_register_types)
