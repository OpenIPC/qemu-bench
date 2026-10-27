/*
 * The test picture the SigmaStar/MStar video models make in place of a
 * camera's, the scene of the Allwinner and Grain Media capture models
 * (hw/misc/aw-vinc.c, gm-vcap.c): the OpenIPC logo, white on a navy
 * gradient, with a round dot that slides to and fro beneath it.  It is
 * laid out in 1920x1080 and scaled to the picture asked for; the dot's
 * place comes from the frame number.  The logo stands still and the
 * gradient runs down the picture only, so most macroblocks are flat or
 * repeat the row above, which the encoders' cheap modes code exactly.
 *
 * The logo is the run-length bitmap of hw/misc/aw-vinc-logo.h, which the
 * including file includes first; each file decodes it into its coverage
 * once.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_PATTERN_H
#define HW_MISC_SSTAR_PATTERN_H

#ifndef HW_MISC_AW_VINC_LOGO_H
#error "include aw-vinc-logo.h before sstar-pattern.h"
#endif

#include <math.h>

#define SSTAR_PAT_W         1920
#define SSTAR_PAT_H         1080
#define SSTAR_PAT_LOGO_X    ((SSTAR_PAT_W - AW_VINC_LOGO_W) / 2)
#define SSTAR_PAT_LOGO_Y    (SSTAR_PAT_H * 2 / 5 - AW_VINC_LOGO_H / 2)
#define SSTAR_PAT_DOT_R     64
#define SSTAR_PAT_DOT_Y     (SSTAR_PAT_H * 4 / 5)
#define SSTAR_PAT_DOT_X0    (SSTAR_PAT_LOGO_X + SSTAR_PAT_DOT_R)
#define SSTAR_PAT_DOT_RANGE (AW_VINC_LOGO_W - 2 * SSTAR_PAT_DOT_R)

/* the logo's coverage, 0..255, AW_VINC_LOGO_W x AW_VINC_LOGO_H */
static inline const uint8_t *sstar_pattern_logo(void)
{
    static uint8_t *logo;
    const unsigned sc = AW_VINC_LOGO_SCALE;
    const unsigned bw = AW_VINC_LOGO_W * sc;
    gsize n, i = 0;
    g_autofree guchar *runs = NULL;
    g_autofree uint8_t *count = NULL;
    uint64_t pos = 0, end = (uint64_t)bw * AW_VINC_LOGO_H * sc;
    bool set = false;

    if (logo) {
        return logo;
    }
    runs = g_base64_decode(aw_vinc_logo_runs_b64, &n);
    count = g_malloc0(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
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
    logo = g_malloc(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    for (i = 0; i < AW_VINC_LOGO_W * AW_VINC_LOGO_H; i++) {
        logo[i] = count[i] * 255 / (sc * sc);
    }
    return logo;
}

/* the dot's centre in frame @frame, 1920x1080 pixels */
static inline int sstar_pattern_dot_x(uint32_t frame)
{
    uint32_t t = (frame * 8) % (2 * SSTAR_PAT_DOT_RANGE);

    return SSTAR_PAT_DOT_X0 +
           (t < SSTAR_PAT_DOT_RANGE ? t : 2 * SSTAR_PAT_DOT_RANGE - t);
}

/*
 * How white (x, y) of a @w x @h picture is, 0..255: the logo's coverage
 * or the dot's, its edge anti-aliased over a pixel; @dot_x from
 * sstar_pattern_dot_x().
 */
static inline unsigned sstar_pattern_cover(int dot_x, int w, int h, int x,
                                           int y)
{
    int px = (int64_t)x * SSTAR_PAT_W / MAX(w, 1);
    int py = (int64_t)y * SSTAR_PAT_H / MAX(h, 1);
    int dx = px - dot_x, dy = py - SSTAR_PAT_DOT_Y;
    int d;

    if (px >= SSTAR_PAT_LOGO_X && px < SSTAR_PAT_LOGO_X + AW_VINC_LOGO_W &&
        py >= SSTAR_PAT_LOGO_Y && py < SSTAR_PAT_LOGO_Y + AW_VINC_LOGO_H) {
        return sstar_pattern_logo()[(py - SSTAR_PAT_LOGO_Y) * AW_VINC_LOGO_W +
                                    px - SSTAR_PAT_LOGO_X];
    }
    if (abs(dx) > SSTAR_PAT_DOT_R + 1 || abs(dy) > SSTAR_PAT_DOT_R + 1) {
        return 0;
    }
    d = (int)(sqrt(dx * dx + dy * dy) * 256);
    return d <= SSTAR_PAT_DOT_R * 256 - 128 ? 255 :
           d >= SSTAR_PAT_DOT_R * 256 + 128 ? 0 :
           (SSTAR_PAT_DOT_R * 256 + 128 - d) * 255 / 256;
}

/* Y, Cb, Cr at (x, y) of a @w x @h picture in frame @frame */
static inline void sstar_pattern_pixel(uint32_t frame, int w, int h, int x,
                                       int y, uint8_t *yuv)
{
    unsigned a = sstar_pattern_cover(sstar_pattern_dot_x(frame), w, h, x, y);
    int py = (int64_t)y * SSTAR_PAT_H / MAX(h, 1);

    /* navy, a little lighter at the top; blended to white */
    yuv[0] = 50 - 14 * py / SSTAR_PAT_H;
    yuv[1] = 146;
    yuv[2] = 122;
    yuv[0] += (235 - yuv[0]) * (int)a / 255;
    yuv[1] += (128 - yuv[1]) * (int)a / 255;
    yuv[2] += (128 - yuv[2]) * (int)a / 255;
}

#endif /* HW_MISC_SSTAR_PATTERN_H */
