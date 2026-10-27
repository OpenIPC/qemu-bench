/*
 * The CABAC arithmetic encoder of the SigmaStar/MStar video encoder
 * models (sstar-vhe's H.265, sstar-mfe's H.264), with the MSB-first bit
 * writer it puts its bytes through.
 *
 * The engine is the same in both standards (H.264 9.3.4, H.265 9.3.4.4):
 * rangeTabLPS, transIdxLPS, a 9-bit range, and a flush after a terminate
 * bin of 1 whose last written bit is a one.  It is written the way the
 * HEVC reference encoder (HM) does it, with a 32-bit low and the carry
 * resolved per byte.  sstar_cabac_finish() writes the flush up to that
 * last bit: a terminate bin of 1 followed by sstar_cabac_finish() and
 * sstar_bits_one_align() is EncodeFlush() plus the zero bits to the byte
 * boundary, which is what both standards put before PCM samples and at
 * the end of a slice.  After PCM samples sstar_cabac_start() restarts the
 * engine; the context variables keep their states (H.264 9.3.1.2).
 *
 * The context variables are initialised by the caller from its
 * standard's tables through sstar_cabac_init_ctx() (m, n, slice QP).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_CABAC_H
#define HW_MISC_SSTAR_CABAC_H

/* An MSB-first bit writer appending to a byte array */
typedef struct SstarBits {
    GByteArray *buf;
    uint32_t acc;
    int n;                              /* bits in acc, 0..7 */
} SstarBits;

static inline void sstar_bits_put(SstarBits *b, uint32_t v, int n)
{
    while (n > 0) {
        int k = MIN(n, 8);

        b->acc = (b->acc << k) | ((v >> (n - k)) & ((1u << k) - 1));
        b->n += k;
        n -= k;
        if (b->n >= 8) {
            uint8_t byte = b->acc >> (b->n - 8);

            g_byte_array_append(b->buf, &byte, 1);
            b->n -= 8;
            b->acc &= (1u << b->n) - 1;
        }
    }
}

static inline void sstar_bits_ue(SstarBits *b, uint32_t v)
{
    int len = 32 - clz32(v + 1);

    sstar_bits_put(b, 0, len - 1);
    sstar_bits_put(b, v + 1, len);
}

static inline void sstar_bits_se(SstarBits *b, int v)
{
    sstar_bits_ue(b, v > 0 ? 2 * v - 1 : -2 * v);
}

/* zero bits up to the byte boundary */
static inline void sstar_bits_zero_align(SstarBits *b)
{
    if (b->n) {
        sstar_bits_put(b, 0, 8 - b->n);
    }
}

/* rbsp_trailing_bits() and byte_alignment(): a one, then zeros */
static inline void sstar_bits_one_align(SstarBits *b)
{
    sstar_bits_put(b, 1, 1);
    sstar_bits_zero_align(b);
}

/* the context variables a coder can have (H.265 uses 135, H.264 89) */
#define SSTAR_CABAC_CTX_MAX     256

typedef struct SstarCabac {
    SstarBits *bw;
    uint32_t low, range;
    int bits_left;
    int buffered, num_buffered;
    uint8_t state[SSTAR_CABAC_CTX_MAX];     /* pStateIdx */
    uint8_t mps[SSTAR_CABAC_CTX_MAX];       /* valMPS */
} SstarCabac;

/* rangeTabLPS (H.264 Table 9-44, H.265 Table 9-46) */
static const uint8_t sstar_cabac_lps[64][4] = {
    { 128, 176, 208, 240 }, { 128, 167, 197, 227 }, { 128, 158, 187, 216 },
    { 123, 150, 178, 205 }, { 116, 142, 169, 195 }, { 111, 135, 160, 185 },
    { 105, 128, 152, 175 }, { 100, 122, 144, 166 }, { 95, 116, 137, 158 },
    { 90, 110, 130, 150 }, { 85, 104, 123, 142 }, { 81, 99, 117, 135 },
    { 77, 94, 111, 128 }, { 73, 89, 105, 122 }, { 69, 85, 100, 116 },
    { 66, 80, 95, 110 }, { 62, 76, 90, 104 }, { 59, 72, 86, 99 },
    { 56, 69, 81, 94 }, { 53, 65, 77, 89 }, { 51, 62, 73, 85 },
    { 48, 59, 69, 80 }, { 46, 56, 66, 76 }, { 43, 53, 63, 72 },
    { 41, 50, 59, 69 }, { 39, 48, 56, 65 }, { 37, 45, 54, 62 },
    { 35, 43, 51, 59 }, { 33, 41, 48, 56 }, { 32, 39, 46, 53 },
    { 30, 37, 43, 50 }, { 29, 35, 41, 48 }, { 27, 33, 39, 45 },
    { 26, 31, 37, 43 }, { 24, 30, 35, 41 }, { 23, 28, 33, 39 },
    { 22, 27, 32, 37 }, { 21, 26, 30, 35 }, { 20, 24, 29, 33 },
    { 19, 23, 27, 31 }, { 18, 22, 26, 30 }, { 17, 21, 25, 28 },
    { 16, 20, 23, 27 }, { 15, 19, 22, 25 }, { 14, 18, 21, 24 },
    { 14, 17, 20, 23 }, { 13, 16, 19, 22 }, { 12, 15, 18, 21 },
    { 12, 14, 17, 20 }, { 11, 14, 16, 19 }, { 11, 13, 15, 18 },
    { 10, 12, 15, 17 }, { 10, 12, 14, 16 }, { 9, 11, 13, 15 },
    { 9, 11, 12, 14 }, { 8, 10, 12, 14 }, { 8, 9, 11, 13 },
    { 7, 9, 11, 12 }, { 7, 9, 10, 12 }, { 7, 8, 10, 11 },
    { 6, 8, 9, 11 }, { 6, 7, 9, 10 }, { 6, 7, 8, 9 }, { 2, 2, 2, 2 },
};

/* transIdxLPS (H.264 Table 9-45, H.265 Table 9-47) */
static const uint8_t sstar_cabac_lps_next[64] = {
    0, 0, 1, 2, 2, 4, 4, 5, 6, 7, 8, 9, 9, 11, 11, 12,
    13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
    24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
};

/* (Re)start the arithmetic coder, writing to @bw; contexts untouched. */
static inline void sstar_cabac_start(SstarCabac *c, SstarBits *bw)
{
    c->bw = bw;
    c->low = 0;
    c->range = 510;
    c->bits_left = 23;
    c->buffered = 0xFF;
    c->num_buffered = 0;
}

/* Context @i from its (m, n) at slice QP @qp (H.264 9.3.1.1, 9.3.2.2) */
static inline void sstar_cabac_init_ctx(SstarCabac *c, int i, int m, int n,
                                        int qp)
{
    int pre = MIN(MAX(((m * MIN(MAX(qp, 0), 51)) >> 4) + n, 1), 126);

    c->mps[i] = pre > 63;
    c->state[i] = c->mps[i] ? pre - 64 : 63 - pre;
}

static inline void sstar_cabac_write_out(SstarCabac *c)
{
    uint32_t lead = c->low >> (24 - c->bits_left);

    c->bits_left += 8;
    c->low &= 0xFFFFFFFFu >> c->bits_left;
    if (lead == 0xFF) {
        c->num_buffered++;
    } else if (c->num_buffered > 0) {
        uint32_t carry = lead >> 8;

        sstar_bits_put(c->bw, c->buffered + carry, 8);
        c->buffered = lead & 0xFF;
        while (c->num_buffered > 1) {
            sstar_bits_put(c->bw, (0xFF + carry) & 0xFF, 8);
            c->num_buffered--;
        }
    } else {
        c->num_buffered = 1;
        c->buffered = lead;
    }
}

static inline void sstar_cabac_test_out(SstarCabac *c)
{
    if (c->bits_left < 12) {
        sstar_cabac_write_out(c);
    }
}

/* A context-coded bin */
static inline void sstar_cabac_bin(SstarCabac *c, int bin, int ctx)
{
    uint32_t lps = sstar_cabac_lps[c->state[ctx]][(c->range >> 6) & 3];

    c->range -= lps;
    if (bin != c->mps[ctx]) {
        int nb = 0;

        while ((lps << nb) < 256) {
            nb++;
        }
        c->low = (c->low + c->range) << nb;
        c->range = lps << nb;
        if (!c->state[ctx]) {
            c->mps[ctx] ^= 1;
        }
        c->state[ctx] = sstar_cabac_lps_next[c->state[ctx]];
        c->bits_left -= nb;
    } else {
        c->state[ctx] = MIN(c->state[ctx] + 1, 62);
        if (c->range >= 256) {
            return;
        }
        c->low <<= 1;
        c->range <<= 1;
        c->bits_left--;
    }
    sstar_cabac_test_out(c);
}

static inline void sstar_cabac_bypass(SstarCabac *c, int bin)
{
    c->low <<= 1;
    if (bin) {
        c->low += c->range;
    }
    c->bits_left--;
    sstar_cabac_test_out(c);
}

static inline void sstar_cabac_bypass_bits(SstarCabac *c, uint32_t v, int n)
{
    while (n--) {
        sstar_cabac_bypass(c, (v >> n) & 1);
    }
}

/* A terminate bin; after a 1 the caller finishes (flushes) the coder */
static inline void sstar_cabac_terminate(SstarCabac *c, int bin)
{
    c->range -= 2;
    if (bin) {
        c->low += c->range;
        c->low <<= 7;
        c->range = 2 << 7;
        c->bits_left -= 7;
    } else if (c->range >= 256) {
        return;
    } else {
        c->low <<= 1;
        c->range <<= 1;
        c->bits_left--;
    }
    sstar_cabac_test_out(c);
}

/* Write out what is pending, up to (not including) the flush's last 1 */
static inline void sstar_cabac_finish(SstarCabac *c)
{
    if (c->low >> (32 - c->bits_left)) {
        sstar_bits_put(c->bw, c->buffered + 1, 8);
        while (c->num_buffered > 1) {
            sstar_bits_put(c->bw, 0x00, 8);
            c->num_buffered--;
        }
        c->low -= 1u << (32 - c->bits_left);
    } else {
        if (c->num_buffered > 0) {
            sstar_bits_put(c->bw, c->buffered, 8);
        }
        while (c->num_buffered > 1) {
            sstar_bits_put(c->bw, 0xFF, 8);
            c->num_buffered--;
        }
    }
    sstar_bits_put(c->bw, c->low >> 8, 24 - c->bits_left);
}

#endif /* HW_MISC_SSTAR_CABAC_H */
