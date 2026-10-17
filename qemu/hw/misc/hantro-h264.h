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

#endif
