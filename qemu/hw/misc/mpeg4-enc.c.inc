/*
 * MPEG-4 part 2 (ISO/IEC 14496-2) simple profile encoder: I and P VOPs,
 * H.263 quantisation, zero motion, one quantiser per VOP.
 *
 * Used by the Grain Media MCP100 model (gm-mcp100).  P macroblocks are
 * predicted from the same place in the reference reconstruction and either
 * skipped, coded as an inter residual, or intra coded.  AC prediction is
 * off.  The VLC tables are the standard's (annex B).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

static const uint16_t m4_intra_vlc[103][2] = {
    { 0x2, 2 }, { 0x6, 3 }, { 0xf, 4 }, { 0xd, 5 }, { 0xc, 5 }, { 0x15, 6
    }, { 0x13, 6 }, { 0x12, 6 }, { 0x17, 7 }, { 0x1f, 8 }, { 0x1e, 8 }, {
    0x1d, 8 }, { 0x25, 9 }, { 0x24, 9 }, { 0x23, 9 }, { 0x21, 9 }, { 0x21,
    10 }, { 0x20, 10 }, { 0xf, 10 }, { 0xe, 10 }, { 0x7, 11 }, { 0x6, 11
    }, { 0x20, 11 }, { 0x21, 11 }, { 0x50, 12 }, { 0x51, 12 }, { 0x52, 12
    }, { 0xe, 4 }, { 0x14, 6 }, { 0x16, 7 }, { 0x1c, 8 }, { 0x20, 9 }, {
    0x1f, 9 }, { 0xd, 10 }, { 0x22, 11 }, { 0x53, 12 }, { 0x55, 12 }, {
    0xb, 5 }, { 0x15, 7 }, { 0x1e, 9 }, { 0xc, 10 }, { 0x56, 12 }, { 0x11,
    6 }, { 0x1b, 8 }, { 0x1d, 9 }, { 0xb, 10 }, { 0x10, 6 }, { 0x22, 9 },
    { 0xa, 10 }, { 0xd, 6 }, { 0x1c, 9 }, { 0x8, 10 }, { 0x12, 7 }, {
    0x1b, 9 }, { 0x54, 12 }, { 0x14, 7 }, { 0x1a, 9 }, { 0x57, 12 }, {
    0x19, 8 }, { 0x9, 10 }, { 0x18, 8 }, { 0x23, 11 }, { 0x17, 8 }, {
    0x19, 9 }, { 0x18, 9 }, { 0x7, 10 }, { 0x58, 12 }, { 0x7, 4 }, { 0xc,
    6 }, { 0x16, 8 }, { 0x17, 9 }, { 0x6, 10 }, { 0x5, 11 }, { 0x4, 11 },
    { 0x59, 12 }, { 0xf, 6 }, { 0x16, 9 }, { 0x5, 10 }, { 0xe, 6 }, { 0x4,
    10 }, { 0x11, 7 }, { 0x24, 11 }, { 0x10, 7 }, { 0x25, 11 }, { 0x13, 7
    }, { 0x5a, 12 }, { 0x15, 8 }, { 0x5b, 12 }, { 0x14, 8 }, { 0x13, 8 },
    { 0x1a, 8 }, { 0x15, 9 }, { 0x14, 9 }, { 0x13, 9 }, { 0x12, 9 }, {
    0x11, 9 }, { 0x26, 11 }, { 0x27, 11 }, { 0x5c, 12 }, { 0x5d, 12 }, {
    0x5e, 12 }, { 0x5f, 12 }, { 0x3, 7 }
};

static const int8_t m4_intra_level[102] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4,
    5, 1, 2, 3, 4, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 1, 2, 1, 1,
    1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 1, 2, 1, 2, 1, 2, 1, 2, 1,
    2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

static const int8_t m4_intra_run[102] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 10, 11, 12, 13, 14, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20
};

static const uint16_t m4_inter_vlc[103][2] = {
    { 0x2, 2 }, { 0xf, 4 }, { 0x15, 6 }, { 0x17, 7 }, { 0x1f, 8 }, { 0x25,
    9 }, { 0x24, 9 }, { 0x21, 10 }, { 0x20, 10 }, { 0x7, 11 }, { 0x6, 11
    }, { 0x20, 11 }, { 0x6, 3 }, { 0x14, 6 }, { 0x1e, 8 }, { 0xf, 10 }, {
    0x21, 11 }, { 0x50, 12 }, { 0xe, 4 }, { 0x1d, 8 }, { 0xe, 10 }, {
    0x51, 12 }, { 0xd, 5 }, { 0x23, 9 }, { 0xd, 10 }, { 0xc, 5 }, { 0x22,
    9 }, { 0x52, 12 }, { 0xb, 5 }, { 0xc, 10 }, { 0x53, 12 }, { 0x13, 6 },
    { 0xb, 10 }, { 0x54, 12 }, { 0x12, 6 }, { 0xa, 10 }, { 0x11, 6 }, {
    0x9, 10 }, { 0x10, 6 }, { 0x8, 10 }, { 0x16, 7 }, { 0x55, 12 }, {
    0x15, 7 }, { 0x14, 7 }, { 0x1c, 8 }, { 0x1b, 8 }, { 0x21, 9 }, { 0x20,
    9 }, { 0x1f, 9 }, { 0x1e, 9 }, { 0x1d, 9 }, { 0x1c, 9 }, { 0x1b, 9 },
    { 0x1a, 9 }, { 0x22, 11 }, { 0x23, 11 }, { 0x56, 12 }, { 0x57, 12 }, {
    0x7, 4 }, { 0x19, 9 }, { 0x5, 11 }, { 0xf, 6 }, { 0x4, 11 }, { 0xe, 6
    }, { 0xd, 6 }, { 0xc, 6 }, { 0x13, 7 }, { 0x12, 7 }, { 0x11, 7 }, {
    0x10, 7 }, { 0x1a, 8 }, { 0x19, 8 }, { 0x18, 8 }, { 0x17, 8 }, { 0x16,
    8 }, { 0x15, 8 }, { 0x14, 8 }, { 0x13, 8 }, { 0x18, 9 }, { 0x17, 9 },
    { 0x16, 9 }, { 0x15, 9 }, { 0x14, 9 }, { 0x13, 9 }, { 0x12, 9 }, {
    0x11, 9 }, { 0x7, 10 }, { 0x6, 10 }, { 0x5, 10 }, { 0x4, 10 }, { 0x24,
    11 }, { 0x25, 11 }, { 0x26, 11 }, { 0x27, 11 }, { 0x58, 12 }, { 0x59,
    12 }, { 0x5a, 12 }, { 0x5b, 12 }, { 0x5c, 12 }, { 0x5d, 12 }, { 0x5e,
    12 }, { 0x5f, 12 }, { 0x3, 7 }
};

static const int8_t m4_inter_level[102] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6, 1, 2, 3, 4,
    1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 1, 2, 1, 2, 1, 2, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

static const int8_t m4_inter_run[102] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3,
    3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 0, 0, 0, 1, 1, 2,
    3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
    39, 40
};

static const uint8_t m4_dc_lum[13][2] = {
    { 0x3, 3 }, { 0x3, 2 }, { 0x2, 2 }, { 0x2, 3 }, { 0x1, 3 }, { 0x1, 4
    }, { 0x1, 5 }, { 0x1, 6 }, { 0x1, 7 }, { 0x1, 8 }, { 0x1, 9 }, { 0x1,
    10 }, { 0x1, 11 }
};

static const uint8_t m4_dc_chrom[13][2] = {
    { 0x3, 2 }, { 0x2, 2 }, { 0x1, 2 }, { 0x1, 3 }, { 0x1, 4 }, { 0x1, 5
    }, { 0x1, 6 }, { 0x1, 7 }, { 0x1, 8 }, { 0x1, 9 }, { 0x1, 10 }, { 0x1,
    11 }, { 0x1, 12 }
};

static const uint8_t m4_cbpy[16][2] = {
    { 0x3, 4 }, { 0x5, 5 }, { 0x4, 5 }, { 0x9, 4 }, { 0x3, 5 }, { 0x7, 4
    }, { 0x2, 6 }, { 0xb, 4 }, { 0x2, 5 }, { 0x3, 6 }, { 0x5, 4 }, { 0xa,
    4 }, { 0x4, 4 }, { 0x8, 4 }, { 0x6, 4 }, { 0x3, 2 }
};

static const uint8_t m4_i_mcbpc_code[9] = {
    1, 1, 2, 3, 1, 1, 2, 3, 1
};

static const uint8_t m4_i_mcbpc_bits[9] = {
    1, 3, 3, 3, 4, 6, 6, 6, 9
};

static const uint8_t m4_p_mcbpc_code[28] = {
    1, 3, 2, 5, 3, 4, 3, 3, 3, 7, 6, 5, 4, 4, 3, 2, 2, 5, 4, 5, 1, 0, 0,
    0, 2, 12, 14, 15
};

static const uint8_t m4_p_mcbpc_bits[28] = {
    1, 4, 4, 6, 5, 8, 8, 7, 3, 7, 7, 9, 6, 9, 9, 9, 3, 7, 7, 8, 9, 0, 0,
    0, 11, 13, 13, 13
};

static const uint8_t m4_y_dc_scale[32] = {
    0, 8, 8, 8, 8, 10, 12, 14, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 34, 36, 38, 40, 42, 44, 46
};

static const uint8_t m4_c_dc_scale[32] = {
    0, 8, 8, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15,
    16, 16, 17, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25
};

typedef struct Mpeg4Enc {
    uint8_t *out;
    size_t cap, len;
    uint32_t acc;
    int nacc;

    int w, h;                       /* multiples of 16 */
    int q;
    bool intra_vop;
    const uint8_t *src[3];          /* Y, Cb, Cr; strides w, w/2, w/2 */
    const uint8_t *ref[3];
    uint8_t *rec[3];
    int16_t *dc[3];                 /* reconstructed DC, -1: not intra */
} Mpeg4Enc;

static void m4_put(Mpeg4Enc *e, uint32_t v, int n)
{
    while (n--) {
        e->acc = e->acc << 1 | ((v >> n) & 1);
        if (++e->nacc == 8) {
            if (e->len < e->cap) {
                e->out[e->len] = e->acc;
            }
            e->len++;
            e->acc = e->nacc = 0;
        }
    }
}

/* next_start_code(): a zero bit, then ones up to the byte */
static void m4_stuff(Mpeg4Enc *e)
{
    m4_put(e, 0, 1);
    while (e->nacc) {
        m4_put(e, 1, 1);
    }
}

static const uint8_t m4_zigzag[64] = {
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

static double m4_cos[8][8];

static void m4_init(void)
{
    int u, x;

    if (m4_cos[0][0]) {
        return;
    }
    for (u = 0; u < 8; u++) {
        for (x = 0; x < 8; x++) {
            m4_cos[u][x] = (u ? 0.5 : sqrt(0.125)) *
                           cos((2 * x + 1) * u * M_PI / 16);
        }
    }
}

static void m4_fdct(const int *in, int *out)
{
    double t[64];
    int u, v, x;

    for (v = 0; v < 8; v++) {
        for (u = 0; u < 8; u++) {
            double s = 0;

            for (x = 0; x < 8; x++) {
                s += in[v * 8 + x] * m4_cos[u][x];
            }
            t[v * 8 + u] = s;
        }
    }
    for (u = 0; u < 8; u++) {
        for (v = 0; v < 8; v++) {
            double s = 0;

            for (x = 0; x < 8; x++) {
                s += t[x * 8 + u] * m4_cos[v][x];
            }
            out[v * 8 + u] = lrint(s);
        }
    }
}

static void m4_idct(const int *in, int *out)
{
    double t[64];
    int u, x, y;

    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            double s = 0;

            for (u = 0; u < 8; u++) {
                s += in[y * 8 + u] * m4_cos[u][x];
            }
            t[y * 8 + x] = s;
        }
    }
    for (x = 0; x < 8; x++) {
        for (y = 0; y < 8; y++) {
            double s = 0;

            for (u = 0; u < 8; u++) {
                s += t[u * 8 + x] * m4_cos[u][y];
            }
            out[y * 8 + x] = lrint(s);
        }
    }
}

static int m4_dequant(int l, int q)
{
    int v;

    if (!l) {
        return 0;
    }
    v = q * (2 * abs(l) + 1) - !(q & 1);
    v = MIN(v, 2047);
    return l < 0 ? -v : v;
}

/* the coefficients from scan position first on; levels in scan order */
static void m4_coefs(Mpeg4Enc *e, const int *lev, int first, bool intra)
{
    const uint16_t (*vlc)[2] = intra ? m4_intra_vlc : m4_inter_vlc;
    const int8_t *runs = intra ? m4_intra_run : m4_inter_run;
    const int8_t *levels = intra ? m4_intra_level : m4_inter_level;
    const int last0 = intra ? 67 : 58;
    int i, j, run = 0, end = 63;

    while (end >= first && !lev[end]) {
        end--;
    }
    for (i = first; i <= end; i++) {
        const int l = abs(lev[i]), last = i == end;

        if (!l) {
            run++;
            continue;
        }
        for (j = last ? last0 : 0; j < (last ? 102 : last0); j++) {
            if (runs[j] == run && levels[j] == l) {
                break;
            }
        }
        if (j < (last ? 102 : last0)) {
            m4_put(e, vlc[j][0], vlc[j][1]);
            m4_put(e, lev[i] < 0, 1);
        } else {                        /* escape, fixed length */
            m4_put(e, vlc[102][0], vlc[102][1]);
            m4_put(e, 3, 2);
            m4_put(e, last, 1);
            m4_put(e, run, 6);
            m4_put(e, 1, 1);
            m4_put(e, lev[i] & 0xFFF, 12);
            m4_put(e, 1, 1);
        }
        run = 0;
    }
}

/* the DC array has a border row and column that read as "not intra" */
static int m4_dc_pred(Mpeg4Enc *e, int c, int bx, int by)
{
    const int bw = (c ? e->w / 16 : e->w / 8) + 1;
    const int16_t *p = &e->dc[c][(by + 1) * bw + bx + 1];
    const int a = p[-1] < 0 ? 1024 : p[-1];
    const int b = p[-bw - 1] < 0 ? 1024 : p[-bw - 1];
    const int t = p[-bw] < 0 ? 1024 : p[-bw];

    return abs(a - b) < abs(b - t) ? t : a;
}

static void m4_dc_set(Mpeg4Enc *e, int c, int bx, int by, int v)
{
    const int bw = (c ? e->w / 16 : e->w / 8) + 1;

    e->dc[c][(by + 1) * bw + bx + 1] = v;
}

static void m4_put_dc(Mpeg4Enc *e, int c, int diff)
{
    const uint8_t (*tab)[2] = c ? m4_dc_chrom : m4_dc_lum;
    int size = diff ? 32 - clz32(abs(diff)) : 0;

    m4_put(e, tab[size][0], tab[size][1]);
    if (size) {
        m4_put(e, diff < 0 ? diff + (1 << size) - 1 : diff, size);
        if (size > 8) {
            m4_put(e, 1, 1);
        }
    }
}

static void m4_block(Mpeg4Enc *e, int mx, int my, int n, int *c, int *bx,
                     int *by, int *stride)
{
    *c = n < 4 ? 0 : n - 3;
    *bx = n < 4 ? mx * 2 + (n & 1) : mx;
    *by = n < 4 ? my * 2 + (n >> 1) : my;
    *stride = n < 4 ? e->w : e->w / 2;
}

static void m4_mb(Mpeg4Enc *e, int mx, int my)
{
    const int q = e->q;
    int lev[6][64], dc[6] = { 0 };
    bool intra = e->intra_vop;
    int n, i, cbp = 0, c, bx, by, st;

    if (!intra) {                       /* inter unless intra is closer */
        int sad = 0, dev = 0, mean = 0;

        for (i = 0; i < 256; i++) {
            int o = (my * 16 + i / 16) * e->w + mx * 16 + i % 16;

            sad += abs(e->src[0][o] - e->ref[0][o]);
            mean += e->src[0][o];
        }
        mean = (mean + 128) / 256;
        for (i = 0; i < 256; i++) {
            int o = (my * 16 + i / 16) * e->w + mx * 16 + i % 16;

            dev += abs(e->src[0][o] - mean);
        }
        intra = dev + 512 < sad;
    }
    for (n = 0; n < 6; n++) {
        int in[64], co[64];

        m4_block(e, mx, my, n, &c, &bx, &by, &st);
        for (i = 0; i < 64; i++) {
            int o = (by * 8 + i / 8) * st + bx * 8 + i % 8;

            in[i] = e->src[c][o] - (intra ? 0 : e->ref[c][o]);
        }
        m4_fdct(in, co);
        for (i = 0; i < 64; i++) {
            int v = co[m4_zigzag[i]], l = abs(v);

            l = intra ? l / (2 * q) : MAX(l - q / 2, 0) / (2 * q);
            l = MIN(l, 2047);
            lev[n][i] = v < 0 ? -l : l;
        }
        if (intra) {
            int sc = c ? m4_c_dc_scale[q] : m4_y_dc_scale[q];

            dc[n] = MAX(co[0] + sc / 2, 0) / sc;
            lev[n][0] = 0;
        }
        for (i = intra ? 1 : 0; i < 64; i++) {
            if (lev[n][i]) {
                cbp |= 32 >> n;
                break;
            }
        }
    }

    if (e->intra_vop) {
        m4_put(e, m4_i_mcbpc_code[cbp & 3], m4_i_mcbpc_bits[cbp & 3]);
    } else if (!intra && !cbp) {
        m4_put(e, 1, 1);                /* not coded */
    } else {
        i = (cbp & 3) + (intra ? 4 : 0);
        m4_put(e, 0, 1);
        m4_put(e, m4_p_mcbpc_code[i], m4_p_mcbpc_bits[i]);
    }
    if (intra) {
        m4_put(e, 0, 1);                /* no AC prediction */
        m4_put(e, m4_cbpy[cbp >> 2][0], m4_cbpy[cbp >> 2][1]);
    } else if (cbp) {
        m4_put(e, m4_cbpy[(cbp >> 2) ^ 15][0], m4_cbpy[(cbp >> 2) ^ 15][1]);
        m4_put(e, 3, 2);                /* a zero vector difference */
    }

    for (n = 0; n < 6; n++) {
        int co[64] = { 0 }, px[64];

        m4_block(e, mx, my, n, &c, &bx, &by, &st);
        for (i = 0; i < 64; i++) {
            co[m4_zigzag[i]] = m4_dequant(lev[n][i], q);
        }
        if (intra) {
            int sc = c ? m4_c_dc_scale[q] : m4_y_dc_scale[q];
            int pred = (m4_dc_pred(e, c, bx, by) + sc / 2) / sc;

            m4_put_dc(e, c, dc[n] - pred);
            co[0] = MIN(dc[n] * sc, 2047);
            m4_dc_set(e, c, bx, by, co[0]);
        } else {
            m4_dc_set(e, c, bx, by, -1);
        }
        if (cbp & (32 >> n)) {
            m4_coefs(e, lev[n], intra ? 1 : 0, intra);
        }
        m4_idct(co, px);
        for (i = 0; i < 64; i++) {
            int o = (by * 8 + i / 8) * st + bx * 8 + i % 8;
            int v = px[i] + (intra ? 0 : e->ref[c][o]);

            e->rec[c][o] = MIN(MAX(v, 0), 255);
        }
    }
}

/* the macroblocks of a VOP whose header is written */
static void m4_vop_data(Mpeg4Enc *e)
{
    int c, mx, my;

    m4_init();
    for (c = 0; c < 3; c++) {
        size_t n = (e->w / 8 + 1) * (e->h / 8 + 1);

        e->dc[c] = g_new(int16_t, n);
        memset(e->dc[c], 0xFF, n * sizeof(int16_t));
    }
    for (my = 0; my < e->h / 16; my++) {
        for (mx = 0; mx < e->w / 16; mx++) {
            m4_mb(e, mx, my);
        }
    }
    for (c = 0; c < 3; c++) {
        g_free(e->dc[c]);
    }
    m4_stuff(e);
}
