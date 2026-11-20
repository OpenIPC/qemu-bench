/*
 * HiSilicon VIU of the Hi3516CV200 / Hi3518EV200 family: what VPSS uses
 * of it when the two run online (vi_vpss_online=1), where the picture
 * goes from capture into VPSS without a DDR frame between them.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_HISI_VIU_H
#define HW_MISC_HISI_VIU_H

#include "qom/object.h"

#define TYPE_HISI_VIU "hisi-viu"
OBJECT_DECLARE_SIMPLE_TYPE(HisiViuState, HISI_VIU)

/* Called at each frame start, before the frame's exposure is taken */
void hisi_viu_set_frame_hook(HisiViuState *s, void (*fn)(void *),
                             void *opaque);

/*
 * Paints the scene as the last frame saw it into an NV21 picture of
 * w x h at ya / ca (strides ys / cs), frame number n in the top squares.
 */
void hisi_viu_paint(HisiViuState *s, uint32_t ya, uint32_t ca, int w, int h,
                    int ys, int cs, uint32_t n);

#endif
