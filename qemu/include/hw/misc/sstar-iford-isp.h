/*
 * SigmaStar Iford image pipe (hw/misc/sstar-iford-isp.c).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SSTAR_IFORD_ISP_H
#define HW_MISC_SSTAR_IFORD_ISP_H

#include "qom/object.h"

#define TYPE_SSTAR_IFORD_ISP "sstar-iford-isp"

/* One frame through the pipe, for an owner with no "frame" input wired. */
void sstar_iford_isp_frame(Object *isp);

/*
 * The ISP's last picture, 8-bit NV12 at half the sensor's window.  @y has
 * *w x *h bytes, @uv *w x *h / 2.  The pointers hold until the next frame.
 */
void sstar_iford_isp_picture(Object *isp, const uint8_t **y,
                             const uint8_t **uv, int *w, int *h);

#endif
