/*
 * The Xiongmai VI models' test picture (xm-isp2, xm-vimu), included by each.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <math.h>
#include "system/dma.h"
#include "aw-vinc-logo.h"

/*
 * The test picture, as aw-vinc.c's: the OpenIPC logo, white on a navy
 * gradient, with a round dot sliding to and fro beneath it, laid out on a
 * 1920x1080 canvas and scaled to each buffer.  Background and logo are
 * drawn once per size; each frame copies them and adds the dot (frame n).
 */
#define PAT_W               1920
#define PAT_H               1080
#define PAT_LOGO_X          ((PAT_W - AW_VINC_LOGO_W) / 2)
#define PAT_LOGO_Y          (PAT_H * 2 / 5 - AW_VINC_LOGO_H / 2)
#define PAT_DOT_R           64
#define PAT_DOT_Y           (PAT_H * 4 / 5)
#define PAT_DOT_X0          (PAT_LOGO_X + PAT_DOT_R)
#define PAT_DOT_RANGE       (AW_VINC_LOGO_W - 2 * PAT_DOT_R)

static uint8_t *xm_logo;        /* coverage 0..255, AW_VINC_LOGO_W x _H */

static void xm_logo_init(void)
{
    const unsigned sc = AW_VINC_LOGO_SCALE;
    const unsigned bw = AW_VINC_LOGO_W * sc;
    gsize n, i = 0;
    g_autofree guchar *runs = g_base64_decode(aw_vinc_logo_runs_b64, &n);
    g_autofree uint8_t *count = g_malloc0(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    uint64_t pos = 0, end = (uint64_t)bw * AW_VINC_LOGO_H * sc;
    bool set = false;

    while (i < n && pos < end) {
        uint64_t run = 0;
        unsigned shift = 0;

        do {
            run |= (uint64_t)(runs[i] & 0x7F) << shift;
            shift += 7;
        } while ((runs[i++] & 0x80) && i < n && shift < 35);
        for (; set && run && pos < end; run--, pos++) {
            count[(pos / bw / sc) * AW_VINC_LOGO_W + (pos % bw) / sc]++;
        }
        pos += run;
        set = !set;
    }
    xm_logo = g_malloc(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    for (i = 0; i < AW_VINC_LOGO_W * AW_VINC_LOGO_H; i++) {
        xm_logo[i] = count[i] * 255 / (sc * sc);
    }
}

static uint8_t xm_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* Y, Cb, Cr of the canvas at (px, py), the dot centred at dot_x (or none) */
static void xm_pattern_pixel(int px, int py, int dot_x, uint8_t *yuv)
{
    unsigned a = 0;

    yuv[0] = 50 - 14 * py / PAT_H;      /* navy, lighter at the top */
    yuv[1] = 146;
    yuv[2] = 122;
    if (px >= PAT_LOGO_X && px < PAT_LOGO_X + AW_VINC_LOGO_W &&
        py >= PAT_LOGO_Y && py < PAT_LOGO_Y + AW_VINC_LOGO_H) {
        a = xm_logo[(py - PAT_LOGO_Y) * AW_VINC_LOGO_W + px - PAT_LOGO_X];
    } else if (dot_x >= 0) {
        const int dx = px - dot_x, dy = py - PAT_DOT_Y;

        if (abs(dx) <= PAT_DOT_R + 1 && abs(dy) <= PAT_DOT_R + 1) {
            const int d = (int)(sqrt(dx * dx + dy * dy) * 256);

            a = d <= PAT_DOT_R * 256 - 128 ? 255 :
                d >= PAT_DOT_R * 256 + 128 ? 0 :
                (PAT_DOT_R * 256 + 128 - d) * 255 / 256;
        }
    }
    if (a) {
        yuv[0] = xm_blend(yuv[0], 235, a);
        yuv[1] = xm_blend(yuv[1], 128, a);
        yuv[2] = xm_blend(yuv[2], 128, a);
    }
}

/* the picture without the dot, NV12, w x h (one size at a time) */
static struct {
    int w, h;
    uint8_t *y, *c;
} xm_pat;

static void xm_pattern_static(int w, int h)
{
    uint8_t yuv[3];

    if (xm_pat.w == w && xm_pat.h == h) {
        return;
    }
    if (!xm_logo) {
        xm_logo_init();
    }
    g_free(xm_pat.y);
    g_free(xm_pat.c);
    xm_pat.w = w;
    xm_pat.h = h;
    xm_pat.y = g_malloc((size_t)w * h);
    xm_pat.c = g_malloc((size_t)w * (h / 2));
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            xm_pattern_pixel(x * PAT_W / w, y * PAT_H / h, -1, yuv);
            xm_pat.y[(size_t)y * w + x] = yuv[0];
            if (!(y & 1) && !(x & 1)) {
                xm_pat.c[(size_t)(y / 2) * w + x] = yuv[1];
                xm_pat.c[(size_t)(y / 2) * w + x + 1] = yuv[2];
            }
        }
    }
}

static void xm_pattern_draw(hwaddr ya, hwaddr ca, int w, int h, int ys, int cs,
                         uint32_t n, int lens)
{
    /* each lens sees the dot a third of a sweep later than the last */
    const uint32_t t = (n * 8 + lens * 2 * PAT_DOT_RANGE / 3) %
                       (2 * PAT_DOT_RANGE);
    const int dot_x = PAT_DOT_X0 +
                      (t < PAT_DOT_RANGE ? t : 2 * PAT_DOT_RANGE - t);
    /* the rows and columns the dot can touch, in buffer pixels; columns
     * from an even one, where the CbCr pairs start */
    const int y0 = (PAT_DOT_Y - PAT_DOT_R - 2) * h / PAT_H;
    const int y1 = (PAT_DOT_Y + PAT_DOT_R + 2) * h / PAT_H + 1;
    const int x0 = MAX((dot_x - PAT_DOT_R - 2) * w / PAT_W, 0) & ~1;
    const int x1 = MIN((dot_x + PAT_DOT_R + 2) * w / PAT_W + 2, w) & ~1;
    uint8_t yuv[3];

    if (w <= 1 || h <= 1 || w > 4096 || h > 4096 || ys < w || cs < w) {
        return;
    }
    xm_pattern_static(w, h);
    g_autofree uint8_t *line = g_malloc(w);
    for (int y = 0; y < h; y++) {
        memcpy(line, xm_pat.y + (size_t)y * w, w);
        if (y >= y0 && y <= y1) {
            for (int x = x0; x < x1; x++) {
                xm_pattern_pixel(x * PAT_W / w, y * PAT_H / h, dot_x, yuv);
                line[x] = yuv[0];
            }
        }
        dma_memory_write(&address_space_memory, ya + (hwaddr)y * ys, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (int y = 0; y < h / 2; y++) {
        memcpy(line, xm_pat.c + (size_t)y * w, w & ~1);
        if (2 * y >= y0 && 2 * y <= y1) {
            for (int x = x0; x < x1; x += 2) {
                xm_pattern_pixel(x * PAT_W / w, 2 * y * PAT_H / h, dot_x, yuv);
                line[x] = yuv[1];
                line[x + 1] = yuv[2];
            }
        }
        dma_memory_write(&address_space_memory, ca + (hwaddr)y * cs, line,
                         w & ~1, MEMTXATTRS_UNSPECIFIED);
    }
}

