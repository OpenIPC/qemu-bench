/*
 * Allwinner sunxi IOMMU (V831/V833): translation for the masters behind it.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_AW_IOMMU_H
#define HW_MISC_AW_IOMMU_H

#include "exec/hwaddr.h"
#include "exec/memattrs.h"

#define TYPE_AW_IOMMU "aw-iommu"

/* Master numbers (the vendor driver's sun8iw19 master[] table). */
#define AW_IOMMU_MASTER_VE      3
#define AW_IOMMU_MASTER_CSI     4
#define AW_IOMMU_MASTER_ISP     5
#define AW_IOMMU_MASTER_G2D     6

/*
 * Write @len bytes at device address @addr for @master, through the
 * IOMMU's page tables when it is on for that master (a NULL @iommu, or a
 * master it bypasses, writes physical memory).  Unmapped pages are
 * skipped, as the IOMMU would fault them.
 */
void aw_iommu_dma_write(DeviceState *iommu, int master, uint32_t addr,
                        const void *buf, uint32_t len);
void aw_iommu_dma_read(DeviceState *iommu, int master, uint32_t addr,
                       void *buf, uint32_t len);

#endif
