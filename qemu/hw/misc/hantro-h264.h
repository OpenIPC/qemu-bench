/*
 * A minimal H.264 intra encoder for the Hantro H1 model; see hantro-h264.c.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_HANTRO_H264_H
#define HW_MISC_HANTRO_H264_H

typedef struct H264Enc {
    /* from the SPS / PPS the guest's encoder library wrote */
    bool have_sps, have_pps, unsupported;
    int log2_max_frame_num, poc_type, log2_max_poc_lsb;
    int pps_id, pic_init_qp, chroma_qp_offset;
    bool cabac, bottom_field_poc_present, deblocking_control;
    bool redundant_pic_cnt_present;
    /* picture numbering across jobs */
    int frame_num, poc, idr_pic_id;
} H264Enc;

/* Parse every SPS and PPS in a byte stream (start codes, escaped NALs). */
void h264_enc_parse_headers(H264Enc *e, const uint8_t *buf, size_t len);

/*
 * Encode one picture from NV12 (y stride ys, interleaved CbCr stride cs) as
 * one I slice, an IDR if idr: a byte-stream NAL with its start code.
 * Returns the bytes it needed (more than cap means the output is
 * truncated), or 0 when the headers seen do not allow it.
 */
size_t h264_enc_frame(H264Enc *e, bool idr, int qp, int mbw, int mbh,
                      const uint8_t *y, int ys, const uint8_t *uv, int cs,
                      uint8_t *out, size_t cap);

/*
 * Slice data only, for engines whose drivers write the headers: the
 * macroblock layer of an mbw x mbh slice with its trailing bits, as RBSP.
 * The CABAC ones return bytes (to follow cabac_alignment_one_bits); the
 * CAVLC ones start bit_offset bits into out's first byte, where the
 * stream is, and return the bits out holds, those unused ones included.  I slices code I_16x16 DC macroblocks from NV12, P
 * slices skip every macroblock.
 */
size_t h264_slice_i16dc_cabac(int qp, int chroma_qp_offset, int mbw, int mbh,
                              const uint8_t *y, int ys, const uint8_t *uv,
                              int cs, uint8_t *out, size_t cap);
size_t h264_slice_pskip_cabac(int qp, int cabac_init_idc, int nmb,
                              uint8_t *out, size_t cap);
size_t h264_slice_i16dc_cavlc(int qp, int chroma_qp_offset, int mbw, int mbh,
                              const uint8_t *y, int ys, const uint8_t *uv,
                              int cs, int bit_offset, uint8_t *out, size_t cap);
size_t h264_slice_pskip_cavlc(int nmb, int bit_offset, uint8_t *out,
                              size_t cap);

/*
 * P slices that follow the source: macroblocks that differ from ref (the
 * decoder's picture for these rows, NV12 at stride mbw * 16, updated) as
 * I_PCM, the rest skipped.  Return bits (CAVLC) or bytes (CABAC), as above.
 */
size_t h264_slice_ppcm_cavlc(int mbw, int mbh, const uint8_t *y, int ys,
                             const uint8_t *uv, int cs, uint8_t *ry,
                             uint8_t *ruv, int bit_offset, uint8_t *out,
                             size_t cap);
size_t h264_slice_ppcm_cabac(int qp, int cabac_init_idc, int mbw, int mbh,
                             const uint8_t *y, int ys, const uint8_t *uv,
                             int cs, uint8_t *ry, uint8_t *ruv, uint8_t *out,
                             size_t cap);


/* the slice header's deblocking: disable_deblocking_filter_idc and the
 * FilterOffsetA / FilterOffsetB (twice the _div2 syntax elements) */
typedef struct H264Deblock {
    int idc, alpha_off, beta_off;
} H264Deblock;

typedef struct H264PStats {
    int skip, inter, intra;     /* macroblocks of each kind */
    int qp;                     /* the QP the coded macroblocks ended at */
} H264PStats;

/*
 * An I slice of I_16x16 DC macroblocks (as h264_slice_i16dc_cabac) that
 * also leaves the decoder's picture, deblocked as db says, in ry / ruv
 * (NV12 at stride mbw * 16): the reference for h264_slice_p_cabac.
 */
size_t h264_slice_i16dc_cabac_ref(int qp, int chroma_qp_offset,
                                  const H264Deblock *db, int mbw, int mbh,
                                  const uint8_t *y, int ys, const uint8_t *uv,
                                  int cs, uint8_t *ry, uint8_t *ruv,
                                  uint8_t *out, size_t cap);

/*
 * A P slice with inter coding (hantro-h264-inter.c.inc): P_Skip,
 * P_L0_16x16 with motion search and residual, I_16x16 DC where nothing
 * matches; one reference, ry / ruv (the previous picture, NV12 at stride
 * mbw * 16), which is replaced by this picture's reconstruction.  The QP
 * rises (mb_qp_delta) until the slice fits cap.  CABAC, cabac_init_idc
 * from the slice header; returns bytes, as the others; st may be NULL.
 */
size_t h264_slice_p_cabac(int qp, int chroma_qp_offset, int cabac_init_idc,
                          bool transform_8x8_mode, const H264Deblock *db,
                          int mbw, int mbh, const uint8_t *y, int ys,
                          const uint8_t *uv, int cs, uint8_t *ry, uint8_t *ruv,
                          uint8_t *out, size_t cap, H264PStats *st);

/*
 * A picture split into slices of whole macroblock rows, coded one slice at
 * a time (h264_pic_slice_cabac: I or P, CABAC, from row r0; returns bytes
 * as the others), then filtered and kept as the next reference
 * (h264_pic_finish, deblocking across the slices' edges for idc 0, not for
 * idc 2).  ry / ruv given to h264_pic_new are the reference (NV12 at
 * stride mbw * 16, copied), or NULL: P slices then code every macroblock
 * intra.
 */
typedef struct H264Pic H264Pic;
H264Pic *h264_pic_new(int mbw, int mbh, const uint8_t *y, int ys,
                      const uint8_t *uv, int cs, const uint8_t *ry,
                      const uint8_t *ruv, int chroma_qp_offset,
                      bool transform_8x8_mode);
size_t h264_pic_slice_cabac(H264Pic *pic, bool p_slice, int r0, int rows,
                            int qp, int cabac_init_idc, uint8_t *out,
                            size_t cap, H264PStats *st);
void h264_pic_finish(H264Pic *pic, const H264Deblock *db, uint8_t *ry,
                     uint8_t *ruv);
void h264_pic_free(H264Pic *pic);

#endif
