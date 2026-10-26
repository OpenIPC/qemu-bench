/*
 * Novatek NA51089 media pixels for the nvt-media engines: the sensor test
 * pattern the SIE captures, RAW packing, the IPP's Bayer-to-YUV path and
 * the JPEG codec's baseline encoder (include/hw/misc/nvt-pixel.h).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/misc/nvt-pixel.h"

/* ---- the test pattern ---- */

/* 75 % bars: white, yellow, cyan, green, magenta, red, blue, black */
static const uint16_t bars[8][3] = {
    { 3071, 3071, 3071 }, { 3071, 3071, 0 }, { 0, 3071, 3071 },
    { 0, 3071, 0 }, { 3071, 0, 3071 }, { 3071, 0, 0 }, { 0, 0, 3071 },
    { 0, 0, 0 },
};

static void pattern_rgb(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                        uint32_t frame, uint16_t rgb[3])
{
    uint32_t box = MAX(h / 6, 2u);
    uint32_t bx = (frame * 24) % MAX(w - box, 1u);
    uint32_t by = h / 3 - box / 2;

    if (x >= bx && x < bx + box && y >= by && y < by + box) {
        rgb[0] = rgb[1] = rgb[2] = 4095;
    } else if (y < h * 2 / 3) {
        memcpy(rgb, bars[x * 8 / w], sizeof(bars[0]));
    } else {
        rgb[0] = rgb[1] = rgb[2] = w > 1 ? x * 4095 / (w - 1) : 0;
    }
}

void nvt_pixel_pattern_line(uint16_t *out, uint32_t width, uint32_t height,
                            uint32_t y, uint32_t frame)
{
    uint16_t rgb[3];
    uint32_t x;

    for (x = 0; x < width; x++) {
        pattern_rgb(x & ~1u, y & ~1u, width, height, frame, rgb);
        /* RGGB: R G / G B */
        if (y & 1) {
            out[x] = (x & 1) ? rgb[2] : rgb[1];
        } else {
            out[x] = (x & 1) ? rgb[1] : rgb[0];
        }
    }
}

/*
 * ---- RAW packing: samples of 'bits' bits, little-endian bit stream ----
 * The samples here are 12-bit; narrower depths drop low bits, 16 bits
 * shifts them up to fill the range.
 */

static uint32_t to_depth(uint16_t v, unsigned bits)
{
    return (bits >= 12 ? (uint32_t)v << (bits - 12) : v >> (12 - bits)) &
           ((1u << bits) - 1);
}

static uint16_t from_depth(uint32_t v, unsigned bits)
{
    return bits >= 12 ? v >> (bits - 12) : v << (12 - bits);
}

size_t nvt_pixel_packed_size(uint32_t n, unsigned bits)
{
    return ((uint64_t)n * bits + 7) / 8;
}

void nvt_pixel_pack(uint8_t *dst, const uint16_t *src, uint32_t n,
                    unsigned bits)
{
    uint32_t acc = 0, i;
    unsigned have = 0;

    for (i = 0; i < n; i++) {
        acc |= to_depth(src[i], bits) << have;
        have += bits;
        while (have >= 8) {
            *dst++ = acc;
            acc >>= 8;
            have -= 8;
        }
    }
    if (have) {
        *dst = acc;
    }
}

void nvt_pixel_unpack(uint16_t *dst, const uint8_t *src, uint32_t n,
                      unsigned bits)
{
    uint32_t acc = 0, i;
    unsigned have = 0;

    for (i = 0; i < n; i++) {
        while (have < bits) {
            acc |= (uint32_t)*src++ << have;
            have += 8;
        }
        dst[i] = from_depth(acc & ((1u << bits) - 1), bits);
        acc >>= bits;
        have -= bits;
    }
}

/* ---- Bayer to YUV 4:2:0 ---- */

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

void nvt_pixel_bayer_to_nv12(const uint16_t *bayer, uint32_t in_stride,
                             uint32_t in_w, uint32_t in_h,
                             uint8_t *y, uint32_t y_stride,
                             uint8_t *uv, uint32_t uv_stride,
                             uint32_t out_w, uint32_t out_h)
{
    uint32_t ox, oy;

    in_w &= ~1u;
    in_h &= ~1u;
    for (oy = 0; oy < out_h; oy++) {
        uint32_t qy = MIN((uint64_t)oy * in_h / out_h, in_h - 1) & ~1u;

        for (ox = 0; ox < out_w; ox++) {
            uint32_t qx = MIN((uint64_t)ox * in_w / out_w, in_w - 1) & ~1u;
            const uint16_t *p = bayer + (size_t)qy * in_stride + qx;
            int r = p[0] >> 4;
            int g = ((p[1] + p[in_stride]) / 2) >> 4;
            int b = p[in_stride + 1] >> 4;

            /* BT.601, limited range */
            y[(size_t)oy * y_stride + ox] =
                clamp8(16 + ((66 * r + 129 * g + 25 * b + 128) >> 8));
            if (!(ox & 1) && !(oy & 1)) {
                uint8_t *c = uv + (size_t)(oy / 2) * uv_stride + ox;

                c[0] = clamp8(128 + ((-38 * r - 74 * g + 112 * b + 128) >> 8));
                c[1] = clamp8(128 + ((112 * r - 94 * g - 18 * b + 128) >> 8));
            }
        }
    }
}

/* ---- baseline JPEG ---- */

static const uint8_t zigzag[64] = {
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* ITU-T T.81 Annex K.3 */
static const uint8_t dc_l_bits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_c_bits[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const uint8_t ac_l_bits[16] = {
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
};
static const uint8_t ac_l_vals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};
static const uint8_t ac_c_bits[16] = {
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
};
static const uint8_t ac_c_vals[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

typedef struct {
    uint16_t code[256];
    uint8_t len[256];
} HuffTable;

/* Annex C: code lengths and codes from BITS / HUFFVAL */
static void huff_build(HuffTable *t, const uint8_t bits[16],
                       const uint8_t *vals)
{
    uint16_t code = 0;
    int l, i, k = 0;

    memset(t, 0, sizeof(*t));
    for (l = 1; l <= 16; l++) {
        for (i = 0; i < bits[l - 1]; i++, k++) {
            t->code[vals[k]] = code++;
            t->len[vals[k]] = l;
        }
        code <<= 1;
    }
}

typedef struct {
    uint8_t *out;
    size_t room, n;
    uint32_t acc;
    int have;
    bool full;
} BitWriter;

static void put_byte(BitWriter *w, uint8_t b)
{
    if (w->n + 2 > w->room) {
        w->full = true;
        return;
    }
    w->out[w->n++] = b;
    if (b == 0xFF) {
        w->out[w->n++] = 0;     /* byte stuffing */
    }
}

static void put_bits(BitWriter *w, uint32_t v, int n)
{
    w->acc = (w->acc << n) | (v & ((1u << n) - 1));
    w->have += n;
    while (w->have >= 8) {
        put_byte(w, w->acc >> (w->have - 8));
        w->have -= 8;
    }
}

static int magnitude(int v)
{
    int n = 0;

    v = v < 0 ? -v : v;
    while (v) {
        n++;
        v >>= 1;
    }
    return n;
}

static void put_value(BitWriter *w, int v, int n)
{
    if (n) {
        put_bits(w, v < 0 ? v + (1 << n) - 1 : v, n);
    }
}

/* One 8x8 block: forward DCT, quantise, Huffman-code. */
static void encode_block(BitWriter *w, const int16_t blk[64],
                         const uint8_t q[64], int *dc_pred,
                         const HuffTable *dc, const HuffTable *ac)
{
    static double cosv[8][8];
    static bool init;
    double tmp[8][8];
    int coef[64], u, v, x, y, i, run, n;

    if (!init) {
        for (u = 0; u < 8; u++) {
            for (x = 0; x < 8; x++) {
                cosv[u][x] = cos((2 * x + 1) * u * G_PI / 16) *
                             (u ? 0.5 : 0.5 / sqrt(2));
            }
        }
        init = true;
    }
    /* separable: rows, then columns */
    for (y = 0; y < 8; y++) {
        for (u = 0; u < 8; u++) {
            double sum = 0;

            for (x = 0; x < 8; x++) {
                sum += (blk[y * 8 + x] - 128) * cosv[u][x];
            }
            tmp[y][u] = sum;
        }
    }
    for (v = 0; v < 8; v++) {
        for (u = 0; u < 8; u++) {
            double sum = 0;

            for (y = 0; y < 8; y++) {
                sum += tmp[y][u] * cosv[v][y];
            }
            coef[v * 8 + u] = (int)lround(sum);
        }
    }
    /* q[] is in zig-zag order */
    for (i = 0; i < 64; i++) {
        int c = coef[zigzag[i]];
        int d = MAX(q[i], 1);

        coef[zigzag[i]] = c >= 0 ? (c + d / 2) / d : -((-c + d / 2) / d);
    }
    n = magnitude(coef[0] - *dc_pred);
    put_bits(w, dc->code[n], dc->len[n]);
    put_value(w, coef[0] - *dc_pred, n);
    *dc_pred = coef[0];
    for (run = 0, i = 1; i < 64; i++) {
        int c = coef[zigzag[i]];

        if (!c) {
            run++;
            continue;
        }
        while (run > 15) {
            put_bits(w, ac->code[0xF0], ac->len[0xF0]);     /* ZRL */
            run -= 16;
        }
        n = magnitude(c);
        put_bits(w, ac->code[(run << 4) | n], ac->len[(run << 4) | n]);
        put_value(w, c, n);
        run = 0;
    }
    if (run) {
        put_bits(w, ac->code[0], ac->len[0]);               /* EOB */
    }
}

/* Byte-align with one bits, as before a marker. */
static void pad_ones(BitWriter *w)
{
    if (w->have) {
        put_bits(w, 0x7F, 8 - w->have);
    }
}

size_t nvt_pixel_jpeg_scan(uint8_t *out, size_t room,
                           const uint8_t *y, uint32_t y_stride,
                           const uint8_t *uv, uint32_t uv_stride,
                           uint32_t mcu_w, uint32_t mcu_h,
                           const uint8_t qy[64], const uint8_t qc[64])
{
    return nvt_pixel_jpeg_scan_rst(out, room, y, y_stride, uv, uv + 1,
                                   uv_stride, 2, mcu_w, mcu_h, qy, qc, 0);
}

size_t nvt_pixel_jpeg_scan_rst(uint8_t *out, size_t room,
                               const uint8_t *y, uint32_t y_stride,
                               const uint8_t *cb, const uint8_t *cr,
                               uint32_t c_stride, uint32_t c_step,
                               uint32_t mcu_w, uint32_t mcu_h,
                               const uint8_t qy[64], const uint8_t qc[64],
                               uint32_t restart)
{
    static HuffTable dcl, dcc, acl, acc;
    static bool init;
    BitWriter w = { .out = out, .room = room };
    int dc_y = 0, dc_u = 0, dc_v = 0;
    int16_t blk[64];
    uint32_t mx, my, bx, by, px, py, mcus = 0, rst = 0;

    if (!init) {
        huff_build(&dcl, dc_l_bits, dc_vals);
        huff_build(&dcc, dc_c_bits, dc_vals);
        huff_build(&acl, ac_l_bits, ac_l_vals);
        huff_build(&acc, ac_c_bits, ac_c_vals);
        init = true;
    }
    for (my = 0; my < mcu_h && !w.full; my++) {
        for (mx = 0; mx < mcu_w && !w.full; mx++) {
            if (restart && mcus && mcus % restart == 0) {
                pad_ones(&w);
                w.acc = 0;
                if (w.n + 2 > w.room) {
                    w.full = true;
                    break;
                }
                w.out[w.n++] = 0xFF;                /* RSTn, unstuffed */
                w.out[w.n++] = 0xD0 + (rst++ & 7);
                dc_y = dc_u = dc_v = 0;
            }
            mcus++;
            for (by = 0; by < 2; by++) {
                for (bx = 0; bx < 2; bx++) {
                    for (py = 0; py < 8; py++) {
                        for (px = 0; px < 8; px++) {
                            blk[py * 8 + px] =
                                y[(size_t)(my * 16 + by * 8 + py) * y_stride +
                                  mx * 16 + bx * 8 + px];
                        }
                    }
                    encode_block(&w, blk, qy, &dc_y, &dcl, &acl);
                }
            }
            for (bx = 0; bx < 2; bx++) {            /* Cb, then Cr */
                for (py = 0; py < 8; py++) {
                    for (px = 0; px < 8; px++) {
                        blk[py * 8 + px] = (bx ? cr : cb)
                            [(size_t)(my * 8 + py) * c_stride +
                             (mx * 8 + px) * c_step];
                    }
                }
                encode_block(&w, blk, qc, bx ? &dc_v : &dc_u, &dcc, &acc);
            }
        }
    }
    pad_ones(&w);
    if (w.full || w.n + 2 > room) {
        return 0;
    }
    out[w.n++] = 0xFF;
    out[w.n++] = 0xD9;
    return w.n;
}
