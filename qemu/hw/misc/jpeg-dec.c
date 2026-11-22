/*
 * A baseline JPEG decoder: see jpeg-dec.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "jpeg-dec.h"

typedef struct JdHuff {
    uint8_t bits[17];           /* codes of each length 1..16 */
    uint8_t val[256];
    int32_t maxcode[18], valptr[17], mincode[17];
    bool ok;
} JdHuff;

typedef struct JdBits {
    const uint8_t *p, *end;
    uint32_t acc;
    int n;
    bool marker;                /* a marker reached: zeros from here */
} JdBits;

static const uint8_t jd_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static void jd_huff_build(JdHuff *h)
{
    int code = 0, k = 0, l;

    for (l = 1; l <= 16; l++) {
        h->valptr[l] = k;
        h->mincode[l] = code;
        code += h->bits[l];
        k += h->bits[l];
        h->maxcode[l] = h->bits[l] ? code - 1 : -1;
        code <<= 1;
    }
    h->maxcode[17] = INT32_MAX;
    h->ok = true;
}

static int jd_bit(JdBits *b)
{
    if (!b->n) {
        uint8_t c = 0;

        if (!b->marker && b->p < b->end) {
            c = *b->p++;
            if (c == 0xff) {
                uint8_t d = b->p < b->end ? *b->p : 0;

                if (d == 0) {
                    b->p++;             /* a stuffed 0xff */
                } else {
                    b->p--;             /* a marker: stop before it */
                    b->marker = true;
                    c = 0;
                }
            }
        }
        b->acc = c;
        b->n = 8;
    }
    b->n--;
    return (b->acc >> b->n) & 1;
}

static int jd_bits(JdBits *b, int n)
{
    int v = 0;

    while (n--) {
        v = v << 1 | jd_bit(b);
    }
    return v;
}

static int jd_extend(int v, int n)
{
    return n && v < (1 << (n - 1)) ? v - (1 << n) + 1 : v;
}

static int jd_sym(JdBits *b, const JdHuff *h)
{
    int code = 0, l;

    for (l = 1; l <= 16; l++) {
        code = code << 1 | jd_bit(b);
        if (code <= h->maxcode[l]) {
            return h->val[h->valptr[l] + code - h->mincode[l]];
        }
    }
    return -1;
}

/* The inverse DCT of one block, into 8 x 8 bytes at @out (@stride) */
static void jd_idct(const int32_t in[64], uint8_t *out, int stride)
{
    static double c[8][8];
    static gsize init;
    double tmp[64];
    int x, y, u;

    if (g_once_init_enter(&init)) {
        for (x = 0; x < 8; x++) {
            for (u = 0; u < 8; u++) {
                c[x][u] = (u ? 1 : M_SQRT1_2) *
                          cos((2 * x + 1) * u * M_PI / 16) / 2;
            }
        }
        g_once_init_leave(&init, 1);
    }
    for (y = 0; y < 8; y++) {           /* rows */
        for (x = 0; x < 8; x++) {
            double s = 0;

            for (u = 0; u < 8; u++) {
                s += c[x][u] * in[y * 8 + u];
            }
            tmp[y * 8 + x] = s;
        }
    }
    for (x = 0; x < 8; x++) {           /* columns */
        for (y = 0; y < 8; y++) {
            double s = 0;
            long v;

            for (u = 0; u < 8; u++) {
                s += c[y][u] * tmp[u * 8 + x];
            }
            v = lrint(s + 128);
            out[y * stride + x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

bool jpeg_dec_decode(const uint8_t *buf, size_t len, JpegDecImage *img,
                     const char **err)
{
    uint16_t qt[4][64] = { { 0 } };
    JdHuff hdc[4] = { { { 0 } } }, hac[4] = { { { 0 } } };
    int cid[3] = { 0 }, cq[3] = { 0 }, ctd[3] = { 0 }, cta[3] = { 0 };
    int restart = 0, i, k;
    size_t p = 2;
    bool sof = false;

    memset(img, 0, sizeof(*img));
    if (len < 4 || buf[0] != 0xff || buf[1] != 0xd8) {
        *err = "no SOI";
        return false;
    }
    while (p + 4 <= len) {
        uint8_t m;
        size_t seg, end;

        if (buf[p] != 0xff) {
            *err = "marker expected";
            return false;
        }
        m = buf[p + 1];
        if (m == 0xff) {
            p++;                        /* fill bytes */
            continue;
        }
        if (m == 0xd9) {
            *err = "EOI before SOS";
            return false;
        }
        seg = buf[p + 2] << 8 | buf[p + 3];
        end = p + 2 + seg;
        if (seg < 2 || end > len) {
            *err = "segment overruns the stream";
            return false;
        }
        p += 4;
        switch (m) {
        case 0xdb:                      /* DQT */
            while (p < end) {
                int pq = buf[p] >> 4, tq = buf[p] & 3;

                p++;
                if (end - p < (pq ? 128 : 64)) {
                    *err = "short DQT";
                    return false;
                }
                for (i = 0; i < 64; i++) {
                    qt[tq][jd_zigzag[i]] = pq ? buf[p] << 8 | buf[p + 1]
                                              : buf[p];
                    p += pq ? 2 : 1;
                }
            }
            break;
        case 0xc4:                      /* DHT */
            while (p + 17 <= end) {
                int tc = buf[p] >> 4, th = buf[p] & 3, n = 0;
                JdHuff *h = tc ? &hac[th] : &hdc[th];

                for (i = 1; i <= 16; i++) {
                    h->bits[i] = buf[p + i];
                    n += h->bits[i];
                }
                p += 17;
                if (n > 256 || p + n > end) {
                    *err = "bad DHT";
                    return false;
                }
                memcpy(h->val, buf + p, n);
                p += n;
                jd_huff_build(h);
            }
            break;
        case 0xdd:                      /* DRI */
            if (end - p < 2) {
                *err = "short DRI";
                return false;
            }
            restart = buf[p] << 8 | buf[p + 1];
            break;
        case 0xc0:                      /* SOF0, SOF1: baseline / extended */
        case 0xc1:
            if (end - p < 6 || end - p < 6 + 3 * (size_t)buf[p + 5]) {
                *err = "short SOF";
                return false;
            }
            if (buf[p] != 8) {
                *err = "not 8-bit";
                return false;
            }
            img->h = buf[p + 1] << 8 | buf[p + 2];
            img->w = buf[p + 3] << 8 | buf[p + 4];
            img->ncomp = buf[p + 5];
            if ((img->ncomp != 1 && img->ncomp != 3) || !img->w || !img->h ||
                img->w > 8192 || img->h > 8192) {
                *err = "unsupported frame";
                return false;
            }
            img->hmax = img->vmax = 1;
            for (i = 0; i < img->ncomp; i++) {
                const uint8_t *c = buf + p + 6 + 3 * i;

                cid[i] = c[0];
                img->hs[i] = c[1] >> 4;
                img->vs[i] = c[1] & 15;
                cq[i] = c[2] & 3;
                if (img->hs[i] < 1 || img->hs[i] > 2 || img->vs[i] < 1 ||
                    img->vs[i] > 2) {
                    *err = "unsupported sampling";
                    return false;
                }
                img->hmax = MAX(img->hmax, img->hs[i]);
                img->vmax = MAX(img->vmax, img->vs[i]);
            }
            sof = true;
            break;
        case 0xc2: case 0xc3: case 0xc5: case 0xc6: case 0xc7:
        case 0xc9: case 0xca: case 0xcb: case 0xcd: case 0xce: case 0xcf:
            *err = "not a baseline JPEG";
            return false;
        case 0xda: {                    /* SOS: the scan */
            int ns = p < end ? buf[p] : 0, mx, my, mcux, mcuy, rst = 0;
            int32_t pred[3] = { 0 };
            JdBits b;

            if (end - p < 1 || end - p < 1 + 2 * (size_t)ns) {
                *err = "short SOS";
                return false;
            }
            if (!sof || ns != img->ncomp) {
                *err = "unsupported scan";
                return false;
            }
            for (i = 0; i < ns; i++) {
                int id = buf[p + 1 + 2 * i], t = buf[p + 2 + 2 * i];

                for (k = 0; k < img->ncomp && cid[k] != id; k++) {
                    continue;
                }
                if (k == img->ncomp) {
                    *err = "scan names no component";
                    return false;
                }
                ctd[k] = t >> 4 & 3;
                cta[k] = t & 3;
                if (!hdc[ctd[k]].ok || !hac[cta[k]].ok) {
                    *err = "missing Huffman table";
                    return false;
                }
            }
            if (ns == 1) {              /* one component: no interleave */
                img->hs[0] = img->vs[0] = img->hmax = img->vmax = 1;
            }
            mcux = DIV_ROUND_UP(img->w, 8 * img->hmax);
            mcuy = DIV_ROUND_UP(img->h, 8 * img->vmax);
            for (i = 0; i < img->ncomp; i++) {
                img->pw[i] = mcux * img->hs[i] * 8;
                img->ph[i] = mcuy * img->vs[i] * 8;
                img->plane[i] = g_malloc0((size_t)img->pw[i] * img->ph[i]);
            }
            b = (JdBits) { .p = buf + end, .end = buf + len };
            for (my = 0; my < mcuy; my++) {
                for (mx = 0; mx < mcux; mx++) {
                    if (restart && rst == restart) {
                        /* RSTn: byte aligned, the predictions reset */
                        b.n = 0;
                        if (b.marker && b.p + 1 < b.end &&
                            (b.p[1] & 0xf8) == 0xd0) {
                            b.p += 2;
                            b.marker = false;
                        }
                        memset(pred, 0, sizeof(pred));
                        rst = 0;
                    }
                    rst++;
                    for (i = 0; i < img->ncomp; i++) {
                        int bx, by;

                        for (by = 0; by < img->vs[i]; by++) {
                            for (bx = 0; bx < img->hs[i]; bx++) {
                                int32_t blk[64] = { 0 };
                                int s = jd_sym(&b, &hdc[ctd[i]]), j;

                                if (s < 0 || s > 11) {
                                    *err = "bad DC code";
                                    goto fail;
                                }
                                pred[i] += jd_extend(jd_bits(&b, s), s);
                                blk[0] = pred[i] * qt[cq[i]][0];
                                for (j = 1; j < 64; ) {
                                    int rs = jd_sym(&b, &hac[cta[i]]);
                                    int r = rs >> 4, sz = rs & 15;

                                    if (rs < 0) {
                                        *err = "bad AC code";
                                        goto fail;
                                    }
                                    if (!sz) {
                                        if (r != 15) {
                                            break;      /* EOB */
                                        }
                                        j += 16;
                                        continue;
                                    }
                                    j += r;
                                    if (j > 63) {
                                        *err = "AC past the block";
                                        goto fail;
                                    }
                                    blk[jd_zigzag[j]] =
                                        jd_extend(jd_bits(&b, sz), sz) *
                                        qt[cq[i]][jd_zigzag[j]];
                                    j++;
                                }
                                jd_idct(blk, img->plane[i] +
                                        (size_t)((my * img->vs[i] + by) * 8) *
                                        img->pw[i] +
                                        (mx * img->hs[i] + bx) * 8,
                                        img->pw[i]);
                            }
                        }
                    }
                }
            }
            return true;
        }
        default:                        /* APPn, COM, ... */
            break;
        }
        p = end;
    }
    *err = "no scan";
    return false;
fail:
    jpeg_dec_free(img);
    return false;
}

void jpeg_dec_free(JpegDecImage *img)
{
    int i;

    for (i = 0; i < 3; i++) {
        g_free(img->plane[i]);
        img->plane[i] = NULL;
    }
}
