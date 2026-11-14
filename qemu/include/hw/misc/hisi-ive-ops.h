/*
 * HiSilicon IVE image ops on host buffers, shared by the V4 IVE
 * (hisi-ive, register driven) and the V5 one (hisi-ive-npu, NPU tasks).
 * Pictures are w x h with rows packed; the callers move them in and out
 * of guest memory with their own strides.  The semantics are those of
 * the SDK's ot_common_ive.h / hi_ive.h.
 *
 * Copyright (c) 2026 OpenIPC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_HISI_IVE_OPS_H
#define HW_MISC_HISI_IVE_OPS_H

enum { IVE_AND, IVE_OR, IVE_XOR };

/* and / or / xor, |a - b| (or with shift (a - b) >> 1 as S8), weighted add */
void ive_bitwise(int op, const uint8_t *a, const uint8_t *b, uint8_t *out,
                 size_t n);
void ive_sub(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             bool shift);
void ive_add(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             uint16_t x, uint16_t y);

void ive_hist(const uint8_t *a, size_t n, uint32_t bins[256]);

/* Sum of absolute differences per blk x blk block, (w / blk) x (h / blk) */
void ive_sad(const uint8_t *a, const uint8_t *b, uint32_t w, uint32_t h,
             int blk, uint32_t *sad);

/* ot_ive_threshold_mode: 0 binary .. 7 orig/mid/orig */
void ive_thresh(const uint8_t *src, uint8_t *out, size_t n, int mode,
                uint8_t lo, uint8_t hi, uint8_t min, uint8_t mid,
                uint8_t max);
/* ot_ive_threshold_s16_mode: 0 / 1 to S8, 2 / 3 to U8; min/mid/max, or orig */
void ive_thresh_s16(const int16_t *src, uint8_t *out, size_t n, int mode,
                    int16_t lo, int16_t hi, int min, int mid, int max);
/* ot_ive_threshold_u16_mode: 0 min/mid/max, 1 min/orig/max */
void ive_thresh_u16(const uint16_t *src, uint8_t *out, size_t n, int mode,
                    uint16_t lo, uint16_t hi, uint8_t min, uint8_t mid,
                    uint8_t max);
/*
 * ot_ive_16bit_to_8bit_mode: 0 S16 to S8, 1 |S16| to U8, 2 S16 + bias to
 * U8, 3 U16 to U8; scaled by num / den as a Q16 ratio
 */
void ive_16bit_to_8bit(const uint16_t *src, uint8_t *out, size_t n, int mode,
                       uint32_t ratio_q16, int8_t bias);

/* sum(a * b), sum(a * a), sum(b * b): ot_ive_ncc_dst_mem */
void ive_ncc(const uint8_t *a, const uint8_t *b, size_t n, uint64_t out[3]);

/* Integral images, each pixel summing the rectangle up to and including it */
void ive_integ(const uint8_t *src, uint32_t w, uint32_t h, uint32_t *sum,
               uint64_t *sqsum);

/*
 * Dilate / erode over a 5x5 mask (0 or 255 per tap) of a binary image:
 * the OR of the masked neighbours, or the AND (the mask's zero taps
 * ignored); edges replicated
 */
void ive_morph(const uint8_t *src, uint32_t w, uint32_t h,
               const uint8_t mask[25], bool dilate, uint8_t *out);

/*
 * Correlate with a kw x kh mask (row-major, kw, kh odd and at most 5),
 * edges replicated; out[i] is the raw sum
 */
void ive_correlate(const uint8_t *src, uint32_t w, uint32_t h,
                   const int8_t *mask, int kw, int kh, int32_t *out);

/* 3x3 median (0), max (1) or min (2), edges replicated */
void ive_ord_stat(const uint8_t *src, uint32_t w, uint32_t h, int mode,
                  uint8_t *out);

/*
 * 3x3 LBP (ot_ive_lbp_ctrl): bit 7 is the top-left neighbour, then
 * clockwise to bit 0 (left).  Normal mode (0) sets a bit where
 * neighbour - centre >= thr (thr as S8), abs mode (1) where
 * |neighbour - centre| >= thr; edges replicated.
 */
void ive_lbp(const uint8_t *src, uint32_t w, uint32_t h, int mode, int thr,
             uint8_t *out);

/*
 * MAG_AND_ANG: the gradient from a 5x5 mask (horizontal) and its
 * transpose (vertical), edges replicated.  The magnitude is |gh| + |gv|
 * saturated to U16, zero below thr; the angle is the gradient's octant,
 * 0 for +x then one per 45 degrees anticlockwise (atan2 rounded to the
 * nearest multiple of 45), and is not thresholded.  Either output may be
 * NULL.
 */
void ive_mag_and_ang(const uint8_t *src, uint32_t w, uint32_t h,
                     const int8_t mask[25], uint16_t thr, uint16_t *mag,
                     uint8_t *ang);

/*
 * NORM_GRAD: the same two gradients, each shifted right by norm with
 * halves rounded up and saturated to S8.  Either output may be NULL.
 */
void ive_norm_grad(const uint8_t *src, uint32_t w, uint32_t h,
                   const int8_t mask[25], int norm, int8_t *gh, int8_t *gv);

/*
 * Bilinear resize, exactly as the engine walks it: a Q27 ratio per axis
 * and a Q27 start offset (negative when upscaling), the position's low 27
 * bits taken as a Q12 weight, the two rows interpolated into a 7-bit
 * intermediate and the column weights applied to that.
 */
void ive_resize(const uint8_t *src, uint32_t sw, uint32_t sh, uint8_t *dst,
                uint32_t dw, uint32_t dh, uint32_t xratio, int32_t xoff,
                uint32_t yratio, int32_t yoff);

/*
 * ACC (ot_ive_acc_mode): 0 adds, 1 subtracts, 2 adds the second
 * picture's low bit, each saturated to U8
 */
void ive_acc(const uint8_t *a, const uint8_t *b, uint8_t *out, size_t n,
             int mode);

/* IMAGE_DOT: (a * b + 2^29) >> 30 of two S17Q15 pictures, saturated */
void ive_image_dot(const int32_t *a, const int32_t *b, int32_t *out,
                   size_t n);

/* ALPHA_BLENDING: (a * alpha + b * (255 - alpha) + 128) >> 8 */
void ive_alpha_blend(const uint8_t *a, const uint8_t *b, const uint8_t *alpha,
                     uint8_t *out, size_t n);

/*
 * CSC (ot_ive_csc_mode) between YUV420SP / YUV422SP (Y plane, then V and
 * U interleaved) and RGB packed or planar as B, G, R.  Both directions
 * are a Q10 matrix plus a per-channel constant; the chroma is replicated
 * on the way out of 420 and taken from each block's top-left pixel on the
 * way in.  ive_csc_yuv_to_rgb() takes the Y plane and the chroma plane
 * already upsampled to w x h; ive_csc_rgb_to_yuv() writes the chroma
 * subsampled.  Modes 0 / 1 give video-range RGB (16..235), 2 / 3 full
 * range; 8 / 9 take RGB to full-range YUV and 10 / 11 to video range.
 * BT.601 is the even mode of each pair, BT.709 the odd one.
 */
bool ive_csc_mode_valid(int mode);
void ive_csc_yuv_to_rgb(const uint8_t *y, const uint8_t *u, const uint8_t *v,
                        uint8_t *out[3], int step, size_t n, int mode);
void ive_csc_rgb_to_yuv(const uint8_t *b, const uint8_t *g, const uint8_t *r,
                        int rgb_stride, uint32_t w, uint32_t h, int mode,
                        uint8_t *yout, uint8_t *cout);

/*
 * CANNY_HYS_EDGE: the thinning and hysteresis the engine does on
 * mag_and_ang's two outputs (the U16 magnitude and the octant angle it
 * wrote first).  Each pixel comes out 1 (not an edge), 0 (a possible
 * edge) or 2 (an edge, and a seed for the walk ss_mpi_ive_canny_edge
 * then does over the stack).  `stack` takes the seeds in raster order,
 * at most `max_pts` of them, and `stack_size` their number.
 *
 * Non-maximum suppression keeps a pixel whose magnitude beats its two
 * neighbours along the gradient's octant: strictly on the side that
 * comes earlier in raster order for the axis-aligned octants, and
 * strictly on both sides for the diagonal ones.  A kept pixel above the
 * low threshold is a possible edge, above the high one an edge, and an
 * edge is a seed unless the pixel to its left is also one (seeded or
 * not) or the pixel above it is a seed -- the engine thins the seeds, so
 * a straight edge alternates.  Which of a run's pixels is the seed does
 * not change what the walk then produces.
 */
void ive_canny_hys_edge(const uint16_t *mag, const uint8_t *ang, uint32_t w,
                        uint32_t h, uint16_t lo, uint16_t hi, uint8_t *edge,
                        uint32_t *stack, uint32_t max_pts,
                        uint32_t *stack_size);

/* One pixel's Gaussians, as the engine keeps them: 8 bytes each */
typedef struct {
    uint16_t mean;              /* U9Q7 */
    uint16_t var;               /* U9Q7 */
    uint16_t freq;
    uint16_t life;
} IveGmm2Model;

typedef struct {
    uint16_t life_factor, life_threshold;
    uint16_t freq_init, freq_reduce, freq_add, freq_threshold;
    uint16_t var_rate, max_var, min_var;
    uint8_t sns_factor, model_num;
} IveGmm2Ctrl;

/*
 * GMM2 for one pixel: picks the first Gaussian whose squared distance
 * passes the sensitivity gate, updates it towards the sample and decays
 * the others, or starts a new Gaussian in the last free slot (the
 * weakest, if none is free).  Returns the foreground flag, and through
 * `bg` and `info` the background value and the match info the engine
 * writes: the frequency leader's index in the high 7 bits and the match
 * flag in the low one, both from the state before the update.
 */
uint8_t ive_gmm2_pixel(IveGmm2Model *g, uint8_t v, const IveGmm2Ctrl *c,
                       uint8_t *bg, uint8_t *info);

/* One level of an LK pyramid: a picture and how to walk it */
typedef struct {
    const uint8_t *img;
    uint32_t w, h, stride;
} IveLkLevel;

/*
 * LK optical flow for one point, over a pyramid of up to 4 levels
 * (coarsest last in `prev` / `next`, both arrays level 0 first).  The
 * point and the result are S25Q7, as the SDK's ot_svp_point_s25q7.  Each
 * level iterates at most `iters` times over a 7x7 window, stopping when
 * a step moves less than `eps` Q7 units; the gradients are the Sobel of
 * the previous picture over 8, the system is solved by Cramer's rule and
 * the step truncated towards minus infinity, which is what a board does
 * to within a Q7 unit or two.  Returns 1 when the point was tracked and
 * 0 when the window left the picture or the structure tensor was too
 * weak (its lesser eigenvalue below min_eig), and writes the mean
 * absolute residual over the window, in Q7, through `err`.
 */
int ive_lk_track(const IveLkLevel *prev, const IveLkLevel *next, int levels,
                 int32_t *px, int32_t *py, int iters, int eps,
                 uint32_t min_eig, uint16_t *err);

/*
 * ST_BOX_FLT_AND_EIGEN: the 3x3 box filter of the interleaved S8
 * gradient pair's products and the corner measure of the resulting
 * structure tensor, one U16 per pixel, plus the picture's maximum
 * through `max_out` (which may be NULL).  `grad` is w x h pairs, gh
 * then gv.  The measure is `t1 + 16 * t1^2 / tr` with
 * `t1 = det / (16 * tr)` -- see hisi-ive-ops.c; it matches a CV608
 * exactly on 84.6% of pixels and within one count on 95.4% (#300).
 */
void ive_st_eigen(const int8_t *grad, uint32_t w, uint32_t h, uint16_t *out,
                  uint16_t *max_out);

/*
 * CANDIDATE_CORNER: `(a == b && a > 0) ? a : 0`, the non-maximum
 * suppression st_cand_corner ends with -- `a` the thresholded measure,
 * `b` its 3x3 maximum.
 */
void ive_cand_corner(const uint8_t *a, const uint8_t *b, uint8_t *out,
                     size_t n);

#endif
