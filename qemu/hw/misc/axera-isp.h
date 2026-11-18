/*
 * Axera camera models: the ISP's colour pipeline, as far as its settings
 * show in the picture.
 *
 * axera-vin.c paints the test scene (axera-scene.h) as the ISP's output.
 * The scene is taken as what the ISP makes of the light with its
 * reference settings, those the camera starts with; the stages here
 * re-run it with the settings the command lists program now, so that the
 * camera's own image controls -- and the AE's exposure -- show in the
 * picture:
 *
 *   raw   light x (sensor exposure / the reference's), then the white
 *         balance gains times the digital gain, clipped at full scale
 *   gamma the tone curve, 129 points; input spacing linear or the
 *         hardware's "exponential" one (doubling segments, GAM_LINEAR)
 *   YUV   BT.601, then a luma curve (33 points over 10-bit luma) and a
 *         2x2 matrix on (Cb, Cr): brightness, contrast, saturation, hue
 *
 * Each stage is undone with the reference settings and redone with the
 * current ones, so the reference settings give back the scene exactly,
 * whatever their absolute scale. Noise reduction, sharpening, WDR,
 * dehaze, 3DNR, AI-ISP, the CCM (an angle-wise matrix list on AX620E)
 * and the RGB-to-YUV matrix are taken as they are, i.e. as in the scene.
 *
 * Not bit-exact: a picture model, evaluated once per distinct colour
 * (the scene has a few thousand) and cached.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AXERA_ISP_H
#define HW_MISC_AXERA_ISP_H

#include "hw/misc/axera-fbuf.h"

#define AXERA_ISP_GAM_N     129
#define AXERA_ISP_YLUT_N    33
#define AXERA_ISP_CACHE     65536

typedef struct AxeraIspParams {
    double wb[3];           /* R, G, B gain (white balance x digital gain) */
    bool wb_set;
    uint16_t gam[AXERA_ISP_GAM_N];  /* U8.4 out */
    bool gam_exp, gam_set;
    uint16_t ylut[AXERA_ISP_YLUT_N];    /* 10-bit, input step 32 */
    bool ylut_set;
    double cm[2][2];        /* (Cb, Cr) out = cm x (Cb, Cr) in */
    bool cm_set;
    double k;               /* exposure over the reference exposure */
} AxeraIspParams;

typedef struct AxeraIsp {
    AxeraIspParams ref, cur;
    bool ref_set;
    uint32_t gen;           /* bumped whenever cur changes */
    struct {
        uint32_t key;       /* Y << 16 | Cb << 8 | Cr, plus 1 << 24 */
        uint8_t y, u, v;
    } *cache;
} AxeraIsp;

/*
 * The gamma's input at each of its points in "exponential" mode, U12:
 * libax_proton's own GAM_LINEAR preset (the identity curve), 8 points
 * per segment, each segment twice as wide as the one before.
 */
static const uint16_t axera_isp_gam_exp_x[AXERA_ISP_GAM_N] = {
    0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
    37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 67, 71, 75, 79,
    83, 87, 91, 95, 99, 103, 107, 111, 115, 119, 123, 127, 135, 143, 151,
    159, 167, 175, 183, 191, 199, 207, 215, 223, 231, 239, 247, 255, 271,
    287, 303, 319, 335, 351, 367, 383, 399, 415, 431, 447, 463, 479, 495,
    511, 543, 575, 607, 639, 671, 703, 735, 767, 799, 831, 863, 895, 927,
    959, 991, 1023, 1087, 1151, 1215, 1279, 1343, 1407, 1471, 1535, 1599,
    1663, 1727, 1791, 1855, 1919, 1983, 2047, 2175, 2303, 2431, 2559, 2687,
    2815, 2943, 3071, 3199, 3327, 3455, 3583, 3711, 3839, 3967, 4095,
};

static inline double axera_isp_gam_xi(const AxeraIspParams *p, int i)
{
    return p->gam_exp ? axera_isp_gam_exp_x[i] / 4095.0
                      : i / (double)(AXERA_ISP_GAM_N - 1);
}

/* The tone curve at linear @x (0..1): U8.4 over 4095 */
static inline double axera_isp_gam_fwd(const AxeraIspParams *p, double x)
{
    int lo = 0, hi = AXERA_ISP_GAM_N - 1;
    double x0, x1;

    if (!p->gam_set) {
        return pow(x, 1 / 2.2);
    }
    if (x <= 0) {
        return p->gam[0] / 4095.0;
    }
    if (x >= 1) {
        return p->gam[hi] / 4095.0;
    }
    while (hi - lo > 1) {
        int m = (lo + hi) / 2;

        if (axera_isp_gam_xi(p, m) <= x) {
            lo = m;
        } else {
            hi = m;
        }
    }
    x0 = axera_isp_gam_xi(p, lo);
    x1 = axera_isp_gam_xi(p, hi);
    return (p->gam[lo] + (p->gam[hi] - p->gam[lo]) * (x - x0) / (x1 - x0)) /
           4095.0;
}

/* The linear input the tone curve takes to @y (0..1 over 4095) */
static inline double axera_isp_gam_inv(const AxeraIspParams *p, double y)
{
    int i;

    if (!p->gam_set) {
        return pow(MAX(y, 0), 2.2);
    }
    y *= 4095;
    for (i = 0; i < AXERA_ISP_GAM_N - 1; i++) {
        if (y < p->gam[i + 1] && p->gam[i + 1] > p->gam[i]) {
            double f = MAX(y - p->gam[i], 0) / (p->gam[i + 1] - p->gam[i]);
            double x0 = axera_isp_gam_xi(p, i), x1 = axera_isp_gam_xi(p, i + 1);

            return x0 + (x1 - x0) * f;
        }
    }
    return 1;
}

/* Luma curve, 10-bit in and out */
static inline double axera_isp_ylut_fwd(const AxeraIspParams *p, double y)
{
    int i;
    double f;

    if (!p->ylut_set) {
        return y;
    }
    y = MIN(MAX(y, 0), 1023.999);
    i = MIN((int)(y / 32), AXERA_ISP_YLUT_N - 2);
    f = (y - 32 * i) / 32;
    return p->ylut[i] + (p->ylut[i + 1] - p->ylut[i]) * f;
}

static inline double axera_isp_ylut_inv(const AxeraIspParams *p, double y)
{
    int i;

    if (!p->ylut_set) {
        return y;
    }
    for (i = 0; i < AXERA_ISP_YLUT_N - 1; i++) {
        if (y < p->ylut[i + 1] && p->ylut[i + 1] > p->ylut[i]) {
            return 32 * (i + MAX(y - p->ylut[i], 0) /
                             (p->ylut[i + 1] - p->ylut[i]));
        }
    }
    return 1023;
}

static inline uint8_t axera_isp_u8(double v)
{
    return v <= 0 ? 0 : v >= 255 ? 255 : (uint8_t)(v + 0.5);
}

/* One colour of the scene (BT.601 limited range) through the pipeline */
static inline void axera_isp_colour(const AxeraIsp *isp, uint8_t *py,
                                    uint8_t *pu, uint8_t *pv)
{
    const AxeraIspParams *r = &isp->ref, *c = &isp->cur;
    double y, cb, cr, rgb[3], top;
    int i;

    /* undo the reference's YUV stage */
    y = axera_isp_ylut_inv(r, *py * 4.0) / 4;
    cb = *pu - 128.0;
    cr = *pv - 128.0;
    if (r->cm_set && c->cm_set) {
        double det = r->cm[0][0] * r->cm[1][1] - r->cm[0][1] * r->cm[1][0];

        if (fabs(det) > 1e-6) {
            double b = (r->cm[1][1] * cb - r->cm[0][1] * cr) / det;

            cr = (-r->cm[1][0] * cb + r->cm[0][0] * cr) / det;
            cb = b;
        }
    }
    /* to R'G'B', 0..1 */
    y = (y - 16) * (255.0 / 219);
    rgb[0] = y + 1.402 * (255.0 / 224) * cr;
    rgb[1] = y - (255.0 / 224) * (0.344136 * cb + 0.714136 * cr);
    rgb[2] = y + 1.772 * (255.0 / 224) * cb;

    /* to the light, and back through the current raw stage and curve */
    top = r->gam_set ? r->gam[AXERA_ISP_GAM_N - 1] / 4095.0 : 1;
    for (i = 0; i < 3; i++) {
        double v = MIN(MAX(rgb[i] / 255, 0), 1);
        double x = axera_isp_gam_inv(r, v * top);

        if (r->wb_set && c->wb_set && r->wb[i] > 0) {
            x = x / r->wb[i] * c->wb[i];
        }
        x *= c->k / r->k;
        rgb[i] = 255 * axera_isp_gam_fwd(c, MIN(x, 1)) / top;
    }

    /* to YUV, and the current YUV stage */
    y = 16 + (219.0 / 255) * (0.299 * rgb[0] + 0.587 * rgb[1] +
                              0.114 * rgb[2]);
    cb = (224.0 / 255) * (-0.168736 * rgb[0] - 0.331264 * rgb[1] +
                          0.5 * rgb[2]);
    cr = (224.0 / 255) * (0.5 * rgb[0] - 0.418688 * rgb[1] -
                          0.081312 * rgb[2]);
    y = axera_isp_ylut_fwd(c, y * 4) / 4;
    if (r->cm_set && c->cm_set) {
        double b = c->cm[0][0] * cb + c->cm[0][1] * cr;

        cr = c->cm[1][0] * cb + c->cm[1][1] * cr;
        cb = b;
    }
    *py = axera_isp_u8(y);
    *pu = axera_isp_u8(128 + cb);
    *pv = axera_isp_u8(128 + cr);
}

/* Set the current settings; true if they differ from the last ones */
static inline bool axera_isp_update(AxeraIsp *isp, const AxeraIspParams *p)
{
    if (!isp->cache) {
        isp->cache = g_malloc0(sizeof(*isp->cache) * AXERA_ISP_CACHE);
    }
    if (!memcmp(&isp->cur, p, sizeof(*p))) {
        return false;
    }
    isp->cur = *p;
    isp->gen++;
    memset(isp->cache, 0, sizeof(*isp->cache) * AXERA_ISP_CACHE);
    return true;
}

/* Rows [r0, r1) of @src through the pipeline into @dst (same size) */
static inline void axera_isp_rows(AxeraIsp *isp, const AxeraPicture *src,
                                  AxeraPicture *dst, uint32_t r0, uint32_t r1)
{
    uint32_t r, c, w = src->w;

    for (r = r0 & ~1u; r < MIN(r1, src->h); r++) {
        const uint8_t *sy = src->y + (size_t)r * w;
        const uint8_t *suv = src->uv + (size_t)(r / 2) * w;
        uint8_t *dy = dst->y + (size_t)r * w;
        uint8_t *duv = (r & 1) ? NULL : dst->uv + (size_t)(r / 2) * w;
        uint32_t last = 0;
        uint8_t oy = 0, ou = 0, ov = 0;

        for (c = 0; c < w; ) {
            uint32_t key = 1u << 24 | sy[c] << 16 | suv[c & ~1u] << 8 |
                           suv[(c & ~1u) + 1];
            uint32_t e = c + 1;

            if (key != last) {
                uint32_t h = (key * 2654435761u) >> 16;

                if (isp->cache[h].key == key) {
                    oy = isp->cache[h].y;
                    ou = isp->cache[h].u;
                    ov = isp->cache[h].v;
                } else {
                    oy = sy[c];
                    ou = suv[c & ~1u];
                    ov = suv[(c & ~1u) + 1];
                    axera_isp_colour(isp, &oy, &ou, &ov);
                    isp->cache[h].key = key;
                    isp->cache[h].y = oy;
                    isp->cache[h].u = ou;
                    isp->cache[h].v = ov;
                }
                last = key;
            }
            /* the run of this colour: flat background, mostly */
            if (!(c & 1)) {
                while (e + 1 < w && sy[e] == sy[c] && sy[e + 1] == sy[c] &&
                       suv[e + 1] == suv[c] && suv[e + 2] == suv[c + 1]) {
                    e += 2;
                }
            }
            memset(dy + c, oy, e - c);
            if (duv) {
                uint32_t i;

                for (i = (c + 1) & ~1u; i < e; i += 2) {
                    duv[i] = ou;
                    duv[i + 1] = ov;
                }
            }
            c = e;
        }
    }
}

#endif /* HW_MISC_AXERA_ISP_H */
