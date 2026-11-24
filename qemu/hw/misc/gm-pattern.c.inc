/*
 * Test picture of the Grain Media capture models (gm-vcap, gm-cap020).
 * Included by each; needs osdep.h and aw-vinc-logo.h.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* ---- the test picture ---------------------------------------------------- */

/*
 * The same scene as the Allwinner capture model (hw/misc/aw-vinc.c): the
 * OpenIPC logo, white on a navy gradient, with a round dot that slides to
 * and fro beneath it.  It is drawn in the sensor's 1920x1080 and scaled to
 * each path's scaler output, and the dot's place comes from the frame
 * count, so the main and sub streams show the same scene.  The logo is the
 * run-length bitmap of aw-vinc-logo.h, averaged into its coverage once.
 */
#define PAT_W               1920
#define PAT_H               1080
#define PAT_LOGO_X          ((PAT_W - AW_VINC_LOGO_W) / 2)
#define PAT_LOGO_Y          (PAT_H * 2 / 5 - AW_VINC_LOGO_H / 2)
#define PAT_DOT_R           64
#define PAT_DOT_Y           (PAT_H * 4 / 5)
#define PAT_DOT_X0          (PAT_LOGO_X + PAT_DOT_R)
#define PAT_DOT_RANGE       (AW_VINC_LOGO_W - 2 * PAT_DOT_R)

static uint8_t *gm_vcap_logo;           /* coverage 0..255 */

static void gm_vcap_logo_init(void)
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
    gm_vcap_logo = g_malloc(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    for (i = 0; i < AW_VINC_LOGO_W * AW_VINC_LOGO_H; i++) {
        gm_vcap_logo[i] = count[i] * 255 / (sc * sc);
    }
}

static uint8_t gm_vcap_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* Y, Cb, Cr at sensor pixel (px, py) with the dot centred at dot_x */
static void gm_vcap_pixel(int px, int py, int dot_x, uint8_t *yuv)
{
    const int dx = px - dot_x, dy = py - PAT_DOT_Y;
    unsigned a = 0;

    yuv[0] = 50 - 14 * py / PAT_H;      /* navy, lighter at the top */
    yuv[1] = 146;
    yuv[2] = 122;
    if (px >= PAT_LOGO_X && px < PAT_LOGO_X + AW_VINC_LOGO_W &&
        py >= PAT_LOGO_Y && py < PAT_LOGO_Y + AW_VINC_LOGO_H) {
        a = gm_vcap_logo[(py - PAT_LOGO_Y) * AW_VINC_LOGO_W +
                         px - PAT_LOGO_X];
    } else if (abs(dx) <= PAT_DOT_R + 1 && abs(dy) <= PAT_DOT_R + 1) {
        /* the dot, its edge anti-aliased over a pixel */
        const int d = (int)(sqrt(dx * dx + dy * dy) * 256);

        a = d <= PAT_DOT_R * 256 - 128 ? 255 :
            d >= PAT_DOT_R * 256 + 128 ? 0 :
            (PAT_DOT_R * 256 + 128 - d) * 255 / 256;
    }
    if (a) {
        yuv[0] = gm_vcap_blend(yuv[0], 235, a);
        yuv[1] = gm_vcap_blend(yuv[1], 128, a);
        yuv[2] = gm_vcap_blend(yuv[2], 128, a);
    }
}
