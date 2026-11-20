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
 * has to follow: the CTB size (TBs up to it, CUs always 8x8), the
 * cu_qp_delta group size, SAO (coded off in every CTB, for luma alone if
 * the slices say so), the chroma QP offset, the deblocking offsets and
 * VUI timing.
 */
typedef struct HevcEncParams {
    int log2_ctb;               /* 4, 5 or 6 */
    int log2_qg;                /* the quantization group; 0: the CTB */
    bool sao;
    bool sao_luma_only;         /* slice_sao_chroma_flag 0 */
    int chroma_qp_offset;       /* pps_cb_qp_offset, the same for Cr */
    int beta_offset_div2;
    int tc_offset_div2;
    bool vui;
    /* slice data only (hevc_enc_slice_data), what other libraries' SPS,
     * PPS and slice headers say: max_transform_hierarchy_depth_intra (a
     * split_transform_flag of 0 is coded when it is not 0),
     * transform_skip_enabled_flag (4x4 TBs code transform_skip_flag 0),
     * a P slice's cabac_init_flag (initType 2) */
    int trafo_depth_intra;
    bool transform_skip;
    bool cabac_init_flag;
    /*
     * P slices with real inter coding (hevc-enc-inter.c.inc), for engines
     * that ask for it: CUs skipped or merged from the merge list, or coded
     * with a motion vector (AMVP) and a residual, or intra DC; what the
     * library's SPS / PPS / slice headers say: scaling_list_enabled_flag
     * with the default lists, slice_temporal_mvp_enabled_flag,
     * max_transform_hierarchy_depth_inter, MaxNumMergeCand (0: 5),
     * sign_data_hiding_enabled_flag; cu_qp: the CUs' QP (0: the slice's,
     * at most HEVC_QP_MAX for I pictures)
     */
    bool inter;
    bool scaling_list;
    bool tmvp;
    int trafo_depth_inter;
    int max_merge_cand;
    bool sign_hiding;
    int cu_qp;
    /* every CU of a P slice skipped (merge candidate 0): the least a
     * picture can take, for a stream buffer with no more room */
    bool all_skip;
    /*
     * A deadline, g_get_monotonic_time() microseconds, 0 for none: once
     * it has passed, the CUs of a P slice not yet coded are skipped (the
     * reference's picture carries over there for one frame) -- an engine
     * whose guest driver gives up on a job that runs too long. Only the
     * intra / skip coder (inter false) looks at it.
     */
    int64_t deadline_us;
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
/* its planes (w x h luma, w/2 x h/2 chroma), NULL before it holds one */
void hevc_enc_ref_planes(const HevcEncRef *r, const uint8_t **y,
                         const uint8_t **cb, const uint8_t **cr);

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
 * A B slice's data (prm->inter only): every inter CU bi-predicted, L0 from
 * ref0 and L1 from ref1 (ref1_same: the same picture), by AMVP with one
 * reference in each list -- no merge, no skip, so no candidate needs POC
 * scaling -- or intra DC.  What the slice header has to say:
 * num_ref_idx_active 1 / 1, mvd_l1_zero_flag 0 and
 * slice_temporal_mvp_enabled_flag 0 (the model has no POCs); with
 * cabac_init_flag initType 1, else 2.
 */
size_t hevc_enc_slice_data_b(const HevcEncParams *prm, int w, int h, int qp,
                             const HevcEncRef *ref0, const HevcEncRef *ref1,
                             bool ref1_same, HevcEncRef *recon,
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

/*
 * The VPS / SPS / PPS a vendor library wrote itself (in software, in front
 * of the engine's output) and the slice header fields they call for: what
 * a stream's slices have to follow, since those -- not ours -- are what
 * the RTSP SDP's sprop-vps/sps/pps carry to a client that joins between
 * two IDRs.  prm: the slice data's parameters, from the same sets.
 */
typedef struct HevcEncHdr {
    HevcEncParams prm;
    int w, h;
    int pps_id;
    int log2_poc_lsb;
    int num_rps;            /* the SPS's short-term RPSs */
    int rps_idx;            /* one of them that is {POC - 1}, or -1 */
    int num_lt_sps;
    bool long_term;
    bool tmvp;
    bool output_flag;       /* output_flag_present_flag */
    int extra_bits;         /* num_extra_slice_header_bits */
    bool cabac_init_present;
    int num_ref_l0;         /* num_ref_idx_l0_default_active */
    int init_qp;
    bool slice_chroma_qp;
    bool deblocking_override;
    bool lf_across_slices;
    bool slice_ext;
    bool weighted_pred;     /* P slices carry a pred_weight_table() */
} HevcEncHdr;

/*
 * Parse the Annex B VPS / SPS / PPS in buf into h; false (h unusable) when
 * one is missing or chooses something the slice data cannot follow.
 */
bool hevc_enc_parse_headers(HevcEncHdr *h, const uint8_t *buf, size_t len);

/*
 * One picture as a single slice NAL unit (IDR_W_RADL or TRAIL_R) with a
 * slice header for h's parameter sets, and no parameter sets of its own;
 * poc: the picture's POC (0 for the IDR, + 1 per P picture).
 */
size_t hevc_enc_hdr_frame(const HevcEncHdr *h, int qp, bool p_slice, int poc,
                          const HevcEncRef *ref, HevcEncRef *recon,
                          const uint8_t *y, int sw, int sh, int ys,
                          const uint8_t *uv, int cs,
                          uint8_t *out, size_t cap);

#endif
