/*
 * Rockchip IOMMU (hw/misc/rk-iommu.c): address translation for the
 * models of the masters behind one.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_RK_IOMMU_H
#define HW_MISC_RK_IOMMU_H

#include "exec/hwaddr.h"
#include "qom/object.h"

/*
 * The physical address of @iova through @iommu, an "rk-iommu": with
 * paging off it is @iova itself.  False when the page is not mapped.
 */
bool rk_iommu_translate(Object *iommu, uint32_t iova, hwaddr *pa);

/* Read @len bytes at @iova as the master sees them, page by page. */
bool rk_iommu_dma_read(Object *iommu, uint32_t iova, void *buf, uint32_t len);

/* Write @len bytes at @iova as the master sees them, page by page. */
bool rk_iommu_dma_write(Object *iommu, uint32_t iova, const void *buf,
                        uint32_t len);

#endif
