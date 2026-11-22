/*
 * Rockchip IOMMU ("rockchip,iommu"), one MMU window of 0x40 bytes, as in
 * front of the RV1126's VOP, ISPP, video encoders and CIF.  The register
 * interface of the kernel's drivers/iommu/rockchip-iommu.c:
 *
 *   0x00 DTE_ADDR     directory table base; reads back masked (v1 keeps
 *                     address bits 31:12, v2 31:4: property "v2")
 *   0x04 STATUS       bit 0 paging enabled, 1 page fault active, 2 stall
 *                     active, 3 idle, 4 replay buffer empty, 31 stall
 *                     not active
 *   0x08 COMMAND      0 enable paging, 1 disable, 2 enable stall,
 *                     3 disable stall, 4 zap IOTLB, 5 page fault done,
 *                     6 force reset (DTE_ADDR and the state back to 0)
 *   0x0c PAGE_FAULT_ADDR  0x10 ZAP_ONE_LINE
 *   0x14 INT_RAWSTAT  0x18 INT_CLEAR  0x1c INT_MASK  0x20 INT_STATUS
 *   0x24 AUTO_GATING
 *
 * The probe's raw-reset test writes 0xCAFEBABE to DTE_ADDR, expects it
 * back masked, force-resets and polls for DTE_ADDR == 0 ("MMU_DTE_ADDR is
 * not functioning" without it).
 *
 * Translation (include/hw/misc/rk-iommu.h) is for the models of the
 * masters behind an MMU, the VOP's scan-out first: v1's two levels, a
 * directory entry per 4 MiB (iova bits 31:22) pointing at a page table,
 * an entry per 4 KiB page (bits 21:12), both "address bits 31:12 | 1
 * valid".  Masters that are not modelled never ask, so no fault is ever
 * raised; the state machine is idle at every step.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/misc/rk-iommu.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_RK_IOMMU "rk-iommu"
OBJECT_DECLARE_SIMPLE_TYPE(RkIommuState, RK_IOMMU)

#define R_DTE_ADDR      0x00
#define R_STATUS        0x04
#define R_COMMAND       0x08
#define R_PF_ADDR       0x0c
#define R_ZAP_ONE_LINE  0x10
#define R_INT_RAWSTAT   0x14
#define R_INT_CLEAR     0x18
#define R_INT_MASK      0x1c
#define R_INT_STATUS    0x20
#define R_AUTO_GATING   0x24
#define REGS_SIZE       0x40

#define ST_PAGING       (1u << 0)
#define ST_STALL        (1u << 2)
#define ST_IDLE         (1u << 3)
#define ST_REPLAY_EMPTY (1u << 4)
#define ST_NOT_STALLED  (1u << 31)

struct RkIommuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    bool v2;
    uint32_t dte_addr;
    bool paging, stall;
    uint32_t int_mask;
    uint32_t auto_gating;
};

static uint64_t rk_iommu_read(void *opaque, hwaddr off, unsigned size)
{
    RkIommuState *s = opaque;

    switch (off) {
    case R_DTE_ADDR:
        return s->dte_addr;
    case R_STATUS:
        return (s->paging ? ST_PAGING : 0) |
               (s->stall ? ST_STALL : ST_NOT_STALLED) |
               ST_IDLE | ST_REPLAY_EMPTY;
    case R_INT_MASK:
        return s->int_mask;
    case R_AUTO_GATING:
        return s->auto_gating;
    }
    return 0;               /* no page fault, no interrupt */
}

static void rk_iommu_reset_state(RkIommuState *s)
{
    s->dte_addr = 0;
    s->paging = s->stall = false;
    s->int_mask = 0;
}

static void rk_iommu_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkIommuState *s = opaque;

    switch (off) {
    case R_DTE_ADDR:
        s->dte_addr = val & (s->v2 ? 0xfffffff0u : 0xfffff000u);
        break;
    case R_COMMAND:
        switch (val & 7) {
        case 0:
            s->paging = true;
            break;
        case 1:
            s->paging = false;
            break;
        case 2:
            s->stall = true;
            break;
        case 3:
            s->stall = false;
            break;
        case 6:
            rk_iommu_reset_state(s);
            break;
        }
        break;
    case R_INT_MASK:
        s->int_mask = val;
        break;
    case R_AUTO_GATING:
        s->auto_gating = val;
        break;
    case R_INT_CLEAR:
    case R_ZAP_ONE_LINE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rk-iommu: write 0x%" HWADDR_PRIx
                      "\n", off);
    }
}

bool rk_iommu_translate(Object *obj, uint32_t iova, hwaddr *pa)
{
    RkIommuState *s = RK_IOMMU(obj);
    uint32_t dte, pte;

    if (!s->paging) {
        *pa = iova;
        return true;
    }
    if (dma_memory_read(&address_space_memory,
                        s->dte_addr + (iova >> 22) * 4, &dte, 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
        !(le32_to_cpu(dte) & 1)) {
        return false;
    }
    dte = le32_to_cpu(dte);
    if (dma_memory_read(&address_space_memory,
                        (dte & 0xfffff000u) + ((iova >> 12) & 0x3ff) * 4,
                        &pte, 4, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
        !(le32_to_cpu(pte) & 1)) {
        return false;
    }
    *pa = (le32_to_cpu(pte) & 0xfffff000u) | (iova & 0xfff);
    return true;
}

bool rk_iommu_dma_read(Object *obj, uint32_t iova, void *buf, uint32_t len)
{
    uint8_t *p = buf;

    while (len) {
        uint32_t n = MIN(len, 0x1000 - (iova & 0xfff));
        hwaddr pa;

        if (!rk_iommu_translate(obj, iova, &pa)) {
            return false;
        }
        if (dma_memory_read(&address_space_memory, pa, p, n,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return false;
        }
        p += n;
        iova += n;
        len -= n;
    }
    return true;
}

bool rk_iommu_dma_write(Object *obj, uint32_t iova, const void *buf,
                        uint32_t len)
{
    const uint8_t *p = buf;

    while (len) {
        uint32_t n = MIN(len, 0x1000 - (iova & 0xfff));
        hwaddr pa;

        if (!rk_iommu_translate(obj, iova, &pa)) {
            return false;
        }
        if (dma_memory_write(&address_space_memory, pa, p, n,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            return false;
        }
        p += n;
        iova += n;
        len -= n;
    }
    return true;
}

static const MemoryRegionOps rk_iommu_ops = {
    .read = rk_iommu_read,
    .write = rk_iommu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_iommu_reset(DeviceState *dev)
{
    RkIommuState *s = RK_IOMMU(dev);

    rk_iommu_reset_state(s);
    s->auto_gating = 0;
}

static void rk_iommu_init(Object *obj)
{
    RkIommuState *s = RK_IOMMU(obj);

    memory_region_init_io(&s->iomem, obj, &rk_iommu_ops, s, TYPE_RK_IOMMU,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property rk_iommu_properties[] = {
    DEFINE_PROP_BOOL("v2", RkIommuState, v2, false),
};

static const VMStateDescription rk_iommu_vmstate = {
    .name = TYPE_RK_IOMMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dte_addr, RkIommuState),
        VMSTATE_BOOL(paging, RkIommuState),
        VMSTATE_BOOL(stall, RkIommuState),
        VMSTATE_UINT32(int_mask, RkIommuState),
        VMSTATE_UINT32(auto_gating, RkIommuState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_iommu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_iommu_reset);
    device_class_set_props(dc, rk_iommu_properties);
    dc->vmsd = &rk_iommu_vmstate;
}

static const TypeInfo rk_iommu_info = {
    .name = TYPE_RK_IOMMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkIommuState),
    .instance_init = rk_iommu_init,
    .class_init = rk_iommu_class_init,
};

static void rk_iommu_register_types(void)
{
    type_register_static(&rk_iommu_info);
}

type_init(rk_iommu_register_types)
