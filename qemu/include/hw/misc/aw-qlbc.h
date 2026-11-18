/*
 * The picture the emulated V831/V833 video input leaves in a buffer in
 * LBC mode, for the emulated video engine's H.264 encoder to read.
 *
 * LBC is Allwinner's line-buffer compression of the video input's frames;
 * its bitstream format is not public (the SDK has only its configuration),
 * and it is only ever written by the video input and read by the encoder.
 * So both models agree on a private stand-in instead: the picture as
 * block averages, which is what the encoder model codes anyway.
 *
 *   AwQlbcHeader
 *   luma:   bh rows of bw bytes, the averages of 4x4 blocks
 *   Cb, Cr: ch rows of cw bytes each, of 8x8 luma (4x4 chroma) blocks
 *
 * with bw = ceil(w / 4), bh = ceil(h / 4), cw = ceil(w / 8),
 * ch = ceil(h / 8): 194 KiB at 1920x1080, well inside the YI firmware's
 * LBC buffers of that size (2 MiB apart).  Fields are little-endian.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AW_QLBC_H
#define HW_MISC_AW_QLBC_H

#define AW_QLBC_MAGIC       0x43424c51      /* "QLBC" */

typedef struct AwQlbcHeader {
    uint32_t magic;
    uint16_t w, h;              /* picture size */
    uint32_t frame;
    uint16_t bw, bh;            /* luma blocks */
    uint16_t cw, ch;            /* chroma blocks */
} QEMU_PACKED AwQlbcHeader;

static inline uint32_t aw_qlbc_size(unsigned w, unsigned h)
{
    unsigned bw = (w + 3) / 4, bh = (h + 3) / 4;
    unsigned cw = (w + 7) / 8, ch = (h + 7) / 8;

    return sizeof(AwQlbcHeader) + bw * bh + 2 * cw * ch;
}

#endif
