/*
 * SigmaStar infinity6e scaler output (hw/misc/sstar-i6e-scl.c).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_SSTAR_I6E_SCL_H
#define HW_MISC_SSTAR_I6E_SCL_H

/*
 * The frame a hardware ring is carrying, as NV12 of @w x @h (stride @w) in
 * @y and @uv, if @ya is the ring the scaler writes.  A ring is a few lines
 * shared with the encoder, which never holds a frame in memory.
 */
bool sstar_i6e_scl_ring(uint64_t ya, int w, int h, uint8_t *y, uint8_t *uv);

/* The same from the Iford scaler, which infinity6c has */
bool sstar_iford_scl_ring(uint64_t ya, int w, int h, uint8_t *y, uint8_t *uv);

#endif
