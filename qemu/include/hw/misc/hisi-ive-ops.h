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

#endif
