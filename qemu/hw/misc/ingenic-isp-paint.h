/*
 * The test picture of the Ingenic image pipeline models (ingenic-isp.c,
 * ingenic-isp-apical.c): hw/misc/sstar-pattern.h's scene -- the OpenIPC
 * logo on a navy gradient, a dot sliding beneath it -- scaled to the
 * picture asked for, and a six-digit frame counter in 3x5 cells at the
 * top left (a cell is height / 54), white on a black box -- an overlay
 * that reads the same whatever the exposure.  Written NV12 (or NV21) by DMA.
 * The scene is lit by @gain (1.0 = 256): what the sensor's exposure and
 * the light make of it, in linear light (isp_lin(); the counter stays
 * white).  In a two-exposure WDR mode @sgain is the short frame's (0 in
 * a linear mode): the long frame where it does not clip, the short one
 * scaled by the exposure ratio where it does, and a tone curve that
 * brings the ratio's range back into the output's (isp_wdr_tone()).
 * A white the long frame clips keeps its shading.
 *
 * Below the dot, at the bottom left, a grey step wedge (six neutral
 * patches, painted luma 60 to 210): the mid-tone neutrals a real test
 * chart has.  The navy, the logo and the dot alone leave an AWB nothing
 * to go by once the AE exposes for the navy and the white saturates;
 * at any exposure some of these patches are neutral and unclipped.
 * Every model's statistics see the scene through ingenic_scene_pixel().
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_INGENIC_ISP_PAINT_H
#define HW_MISC_INGENIC_ISP_PAINT_H

#include "system/address-spaces.h"
#include "system/dma.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"

typedef struct IngenicIspPainter {
    uint8_t *line;              /* one row, as wide as the widest stride */
    uint32_t size;
} IngenicIspPainter;

/* the grey wedge, in sstar-pattern.h's 1920 x 1080 coordinates */
#define ISP_WEDGE_N     6
#define ISP_WEDGE_X     32
#define ISP_WEDGE_PITCH 112
#define ISP_WEDGE_W     96
#define ISP_WEDGE_Y     956
#define ISP_WEDGE_H     80

static inline bool isp_wedge_row(int h, int y)
{
    int py = (int64_t)y * SSTAR_PAT_H / MAX(h, 1);

    return py >= ISP_WEDGE_Y && py < ISP_WEDGE_Y + ISP_WEDGE_H;
}

/* the patch at (x, y) of a row isp_wedge_row() takes, -1: none */
static inline int isp_wedge_patch(int w, int x)
{
    int px = (int64_t)x * SSTAR_PAT_W / MAX(w, 1) - ISP_WEDGE_X;
    int i = px / ISP_WEDGE_PITCH;

    return px >= 0 && i < ISP_WEDGE_N && px % ISP_WEDGE_PITCH < ISP_WEDGE_W ?
           i : -1;
}

/* The scene's YCbCr at (x, y) of a @w x @h picture: the pattern, the wedge */
static inline void ingenic_scene_pixel(uint32_t frame, int w, int h, int x,
                                       int y, uint8_t *yuv)
{
    int i;

    sstar_pattern_pixel(frame, w, h, x, y, yuv);
    if (isp_wedge_row(h, y) && (i = isp_wedge_patch(w, x)) >= 0) {
        yuv[0] = 60 + 30 * i;
        yuv[1] = yuv[2] = 128;
    }
}

/* 3x5 digits, rows top down, bit 2 the left column */
static const uint8_t isp_digits[10][5] = {
    { 7, 5, 5, 5, 7 }, { 2, 6, 2, 2, 7 }, { 7, 1, 7, 4, 7 },
    { 7, 1, 7, 1, 7 }, { 5, 5, 7, 1, 1 }, { 7, 4, 7, 1, 7 },
    { 7, 4, 7, 5, 7 }, { 7, 1, 1, 1, 1 }, { 7, 5, 7, 5, 7 },
    { 7, 5, 7, 1, 7 },
};

#define CNT_DIGITS      6

/* Is (x, y) of a @w x @h picture lit in the frame counter? */
static inline bool isp_counter_lit(uint32_t frame, int w, int h, int x,
                                   int y)
{
    int cell = MAX(h / 54, 1);          /* 20 px on 1080 lines */
    int x0 = cell * 2, y0 = cell * 2;
    int cx = (x - x0) / cell, cy = (y - y0) / cell;
    int d, i;

    if (x < x0 || y < y0 || cy >= 5 || cx >= CNT_DIGITS * 4) {
        return false;
    }
    if (cx % 4 == 3) {
        return false;
    }
    d = CNT_DIGITS - 1 - cx / 4;
    for (i = 0; i < d; i++) {
        frame /= 10;
    }
    return isp_digits[frame % 10][cy] & (4 >> (cx % 4));
}

/*
 * The painted scene is display-referred: gamma 2.2 over limited-range
 * luma.  Light scales its linear light, white clips, and the camera
 * gamma-encodes it again.
 */
static inline double isp_lin(int v)
{
    double n = (v - 16) / 219.0;

    return pow(MIN(MAX(n, 0), 1), 2.2);
}

static inline int isp_disp(double l)
{
    return 16 + (int)lround(219 * pow(MIN(MAX(l, 0), 1), 1 / 2.2));
}

/*
 * The WDR fusion's tone curve: an extended Reinhard curve whose white is
 * the exposure ratio @w, the brightest light the short frame still holds
 * (x = w maps to 1)
 */
static inline double isp_wdr_tone(double x, double w)
{
    return x * (1 + x / (w * w)) / (1 + x);
}

/*
 * The linear light of painted luma @v under the long frame's @gain and
 * the short frame's @sgain (1.0 = 256; @sgain 0: a linear mode), after
 * the WDR fusion and its tone curve
 */
static inline double isp_wdr_light(int v, uint32_t gain, uint32_t sgain)
{
    double l = isp_lin(v), x = l * gain / 256, w;

    if (!sgain || sgain >= gain) {
        return MIN(x, 1);
    }
    w = (double)gain / sgain;
    if (x > 1) {
        /* where the long frame clips, the short one, scaled */
        x = MIN(l * sgain / 256, 1) * w;
    }
    return isp_wdr_tone(x, w);
}

/*
 * Luma @y[v] for painted luma v under @gain (1.0 = 256; @sgain the WDR
 * short frame's, 0 in a linear mode), and the factor (1.0 = 256) that
 * pixel's chroma scales by
 */
static inline void isp_light_table(uint32_t gain, uint32_t sgain, uint8_t *y,
                                   uint16_t *cs)
{
    int v;

    for (v = 0; v < 256; v++) {
        y[v] = isp_disp(isp_wdr_light(v, gain, sgain));
        cs[v] = v > 16 ? (y[v] - 16) * 256 / (v - 16) : 256;
    }
}

static inline uint8_t isp_chroma(int c, uint16_t cs)
{
    int o = 128 + (c - 128) * (int)cs / 256;

    return MIN(MAX(o, 16), 240);
}

/* Is (x, y) in the counter's black box (a cell around its digits)? */
static inline bool isp_counter_box(int w, int h, int x, int y)
{
    int cell = MAX(h / 54, 1);

    return x >= cell && x < cell * (3 + CNT_DIGITS * 4) &&
           y >= cell && y < cell * 8;
}

static inline uint8_t *isp_line(IngenicIspPainter *p, uint32_t n)
{
    if (n > p->size) {
        p->line = g_realloc(p->line, n);
        p->size = n;
    }
    return p->line;
}

/*
 * Frame @frame, @w x @h, into the buffer at @ya (Y) / @uva (CbCr pairs,
 * CrCb with @vu).  A stride of 0 is the width; a negative one (vertical
 * flip) walks up from the address.
 */
static inline void ingenic_isp_paint(IngenicIspPainter *p, uint32_t frame,
                                     uint32_t w, uint32_t h, int32_t ys,
                                     int32_t uvs, hwaddr ya, hwaddr uva,
                                     bool vu, uint32_t gain, uint32_t sgain)
{
    int dot_x = sstar_pattern_dot_x(frame);
    int cell = MAX(h / 54, 1);
    uint32_t x, y;
    uint8_t *l;
    uint8_t yuv[3];
    uint8_t lit[256];
    uint16_t cs[256];

    isp_light_table(gain, sgain, lit, cs);
    ys = ys ? ys : (int32_t)w;
    uvs = uvs ? uvs : (int32_t)w;
    l = isp_line(p, w + 2);
    for (y = 0; y < h; y++) {
        int py = (int64_t)y * SSTAR_PAT_H / h;
        bool scene = (py >= SSTAR_PAT_LOGO_Y &&
                      py < SSTAR_PAT_LOGO_Y + AW_VINC_LOGO_H) ||
                     abs(py - SSTAR_PAT_DOT_Y) <= SSTAR_PAT_DOT_R + 1;
        bool count = y < (uint32_t)cell * 8;
        bool wedge = isp_wedge_row(h, y);

        sstar_pattern_pixel(frame, w, h, 0, y, yuv);
        memset(l, lit[yuv[0]], w);
        for (x = 0; wedge && x < w; x++) {
            int i = isp_wedge_patch(w, x);

            if (i >= 0) {
                l[x] = lit[60 + 30 * i];
            }
        }
        for (x = 0; (scene || count) && x < w; x++) {
            if (scene) {
                unsigned a = sstar_pattern_cover(dot_x, w, h, x, y);

                l[x] = lit[yuv[0] + (235 - yuv[0]) * (int)a / 255];
            }
            if (count && isp_counter_box(w, h, x, y)) {
                l[x] = isp_counter_lit(frame, w, h, x, y) ? 235 : 16;
            }
        }
        dma_memory_write(&address_space_memory, ya + (int64_t)y * ys, l, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (y = 0; y < h / 2; y++) {
        int py = (int64_t)(2 * y) * SSTAR_PAT_H / h;
        bool scene = (py >= SSTAR_PAT_LOGO_Y - 2 &&
                      py < SSTAR_PAT_LOGO_Y + AW_VINC_LOGO_H) ||
                     abs(py - SSTAR_PAT_DOT_Y) <= SSTAR_PAT_DOT_R + 2;
        bool count = 2 * y < (uint32_t)cell * 8;
        bool wedge = isp_wedge_row(h, 2 * y);
        uint8_t bg[3];

        sstar_pattern_pixel(frame, w, h, 0, 2 * y, bg);
        for (x = 0; x + 1 < w; x += 2) {
            if (scene || wedge) {
                ingenic_scene_pixel(frame, w, h, x, 2 * y, yuv);
            } else {
                memcpy(yuv, bg, sizeof(yuv));
            }
            if (count && isp_counter_box(w, h, x, 2 * y)) {
                yuv[1] = yuv[2] = 128;
            } else if (gain != 256 || sgain) {
                yuv[1] = isp_chroma(yuv[1], cs[yuv[0]]);
                yuv[2] = isp_chroma(yuv[2], cs[yuv[0]]);
            }
            l[x] = yuv[vu ? 2 : 1];
            l[x + 1] = yuv[vu ? 1 : 2];
        }
        dma_memory_write(&address_space_memory, uva + (int64_t)y * uvs, l,
                         w & ~1u, MEMTXATTRS_UNSPECIFIED);
    }
}

#endif /* HW_MISC_INGENIC_ISP_PAINT_H */
