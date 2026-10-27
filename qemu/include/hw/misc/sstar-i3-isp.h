/*
 * MStar infinity3 ISP (hw/misc/sstar-i3-isp.c): the picture it hands the
 * scaler.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_I3_ISP_H
#define HW_MISC_SSTAR_I3_ISP_H

#include "qom/object.h"

#define TYPE_SSTAR_I3_ISP "sstar-i3-isp"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI3IspState, SSTAR_I3_ISP)

/*
 * The last frame the pipeline made: 8-bit Y, Cb and Cr planes of
 * @w x @h each (4:4:4).  False while the ISP makes none (no sensor, the
 * ISP off), and the scaler then draws its test pattern.
 */
bool sstar_i3_isp_frame(SstarI3IspState *s, const uint8_t **y,
                        const uint8_t **cb, const uint8_t **cr, int *w,
                        int *h);

#endif /* HW_MISC_SSTAR_I3_ISP_H */
