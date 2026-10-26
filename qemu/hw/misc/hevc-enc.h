/*
 * A minimal HEVC intra encoder for the HEVC engine models; see hevc-enc.c.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_HEVC_ENC_H
#define HW_MISC_HEVC_ENC_H

/*
 * What the vendor library's parameter sets choose, which the slice data
 * has to follow: the CTB size (TBs up to it, CUs always 8x8), SAO (coded
 * off in every CTB), the deblocking tc offset and VUI timing.
 */
typedef struct HevcEncParams {
    int log2_ctb;               /* 4 or 5 */
    bool sao;
    int beta_offset_div2;
    int tc_offset_div2;
    bool vui;
} HevcEncParams;

/* XM535 / XM533's library: CTB 16, no SAO; XM530V200's: CTB 32, SAO */
extern const HevcEncParams hevc_params_ctb16;
extern const HevcEncParams hevc_params_ctb32_sao;

/*
 * A stream's decoded picture: what its next P picture refers to.  A job
 * passes the reference it codes against (NULL, or a picture of another
 * size: every CU intra) and the one to keep its own decoded picture in
 * (NULL: not kept).
 */
typedef struct HevcEncRef HevcEncRef;
HevcEncRef *hevc_enc_ref_new(void);
void hevc_enc_ref_free(HevcEncRef *r);

/*
 * slice_segment_data() of one picture of w x h (each a multiple of 8) from
 * an NV12 source of sw x sh (y stride ys, interleaved CbCr stride cs),
 * scaled to w x h by nearest neighbour, emulation prevention applied: what
 * the HEVC engines write after the slice header the vendor library puts in
 * front.  I slices code intra DC CUs; P slices skip the CUs whose source is
 * the reference's and code the others intra.
 * Returns the bytes it needed (more than cap: truncated).
 */
size_t hevc_enc_slice_data(const HevcEncParams *prm,
                           int w, int h, int qp, bool p_slice,
                           const HevcEncRef *ref, HevcEncRef *recon,
                           const uint8_t *y, int sw, int sh, int ys,
                           const uint8_t *uv, int cs,
                           uint8_t *out, size_t cap);

/*
 * A whole access unit the way the library assembles one: for an I picture
 * VPS, SPS, PPS and an IDR slice, for a P picture a TRAIL_R slice, each
 * with the library's own header fields.  For tests and dumps.
 */
size_t hevc_enc_frame(const HevcEncParams *prm,
                      int w, int h, int qp, bool p_slice, int poc,
                      const HevcEncRef *ref, HevcEncRef *recon,
                      const uint8_t *y, int sw, int sh, int ys,
                      const uint8_t *uv, int cs,
                      uint8_t *out, size_t cap);

#endif
