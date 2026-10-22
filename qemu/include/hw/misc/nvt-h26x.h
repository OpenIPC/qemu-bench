/*
 * Novatek NA51089 H.26x encoder pictures (hw/misc/nvt-h26x.c), for the
 * nvt-media H.26x engine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_H26X_H
#define HW_MISC_NVT_H26X_H

typedef struct NvtH26x NvtH26x;

typedef struct {
    uint32_t len;               /* bytes written to the bitstream buffer */
    uint32_t qp_sum;            /* slice QP x macroblocks / CTBs */
    bool full;                  /* the buffer could not take the picture */
} NvtH26xResult;

NvtH26x *nvt_h26x_new(void);
void nvt_h26x_free(NvtH26x *h);

/*
 * Forget every stream: after a reset each starts again at an IDR.  A
 * picture still coding is dropped.
 */
void nvt_h26x_reset(NvtH26x *h);

typedef void NvtH26xDone(void *opaque, const NvtH26xResult *r);

/*
 * Encode the picture the engine's registers (regs[off / 4]) describe and
 * write it to the bitstream buffer, with its NAL length list; then call
 * done, at once or -- when an encoder codes it, in a worker thread --
 * later, from the main loop.  After a reset, a picture still coding is
 * dropped without its call.
 */
void nvt_h26x_encode(NvtH26x *h, const uint32_t *regs, NvtH26xDone *done,
                     void *opaque);

#endif
