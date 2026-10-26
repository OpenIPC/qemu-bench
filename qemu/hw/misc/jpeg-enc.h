/*
 * A baseline JPEG encoder for the encoder models; see jpeg-enc.c.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_JPEG_ENC_H
#define HW_MISC_JPEG_ENC_H

/*
 * The entropy-coded scan of a w x h 4:2:0 picture from planar luma (stride
 * ys) and chroma (stride cs), quantisers in natural (row-major) order,
 * byte-stuffed and padded with 1s, then EOI if asked: what the hardware
 * writes after the header its driver puts in front.  Edges are replicated
 * out to whole 16x16 MCUs.  Returns the bytes written; *overflow if cap
 * was too small (the output is then cut short).
 */
size_t jpeg_enc_scan(const uint8_t *y, int ys, const uint8_t *cb,
                     const uint8_t *cr, int cs, int w, int h,
                     const uint8_t ql[64], const uint8_t qc[64], bool eoi,
                     uint8_t *out, size_t cap, bool *overflow);

/* SOI, DQT, SOF0, DHT (Annex K) and SOS for such a scan */
void jpeg_enc_header(GByteArray *b, int w, int h, const uint8_t ql[64],
                     const uint8_t qc[64]);

#endif
