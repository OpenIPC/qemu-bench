/*
 * The Axera models' test picture, the scene the other OpenIPC camera
 * models paint in place of a sensor's (include/hw/misc/sstar-pattern.h):
 * the OpenIPC logo, white on a navy gradient, with a round dot sliding
 * to and fro beneath it, laid out in 1920x1080 and scaled to the picture
 * asked for.  Written as NV12 (Y plane, then interleaved Cb/Cr).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_AXERA_SCENE_H
#define HW_MISC_AXERA_SCENE_H

#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"

/*
 * Paint frame @frame of the scene, @w x @h, into @y (stride @stride_y)
 * and @uv (stride @stride_c, h / 2 rows).  Only the logo's and the dot's
 * boxes need the per-pixel coverage; the rest of each row is the
 * gradient's flat navy.
 */
static inline void axera_scene_nv12_rows(uint8_t *y, uint32_t stride_y,
                                         uint8_t *uv, uint32_t stride_c,
                                         uint32_t w, uint32_t h,
                                         uint32_t frame, uint32_t r0,
                                         uint32_t r1)
{
    const int dot_x = sstar_pattern_dot_x(frame);
    uint32_t r, c;

    for (r = r0 & ~1u; r < MIN(r1, h); r++) {
        int py = (int64_t)r * SSTAR_PAT_H / MAX(h, 1);
        uint8_t navy = 50 - 14 * py / SSTAR_PAT_H;
        uint8_t *yl = y + (size_t)r * stride_y;
        uint8_t *cl = (r & 1) ? NULL : uv + (size_t)(r / 2) * stride_c;
        bool logo_row = py >= SSTAR_PAT_LOGO_Y &&
                        py < SSTAR_PAT_LOGO_Y + AW_VINC_LOGO_H;
        bool dot_row = abs(py - SSTAR_PAT_DOT_Y) <= SSTAR_PAT_DOT_R + 1;

        memset(yl, navy, w);
        if (cl) {
            for (c = 0; c + 1 < w; c += 2) {
                cl[c] = 146;
                cl[c + 1] = 122;
            }
        }
        if (!logo_row && !dot_row) {
            continue;
        }
        for (c = 0; c < w; c++) {
            unsigned a = sstar_pattern_cover(dot_x, w, h, c, r);

            if (!a) {
                continue;
            }
            yl[c] = navy + (235 - navy) * (int)a / 255;
            if (cl && !(c & 1)) {
                cl[c] = 146 + (128 - 146) * (int)a / 255;
                cl[c + 1] = 122 + (128 - 122) * (int)a / 255;
            }
        }
    }
}

static inline void axera_scene_nv12(uint8_t *y, uint32_t stride_y,
                                    uint8_t *uv, uint32_t stride_c,
                                    uint32_t w, uint32_t h, uint32_t frame)
{
    axera_scene_nv12_rows(y, stride_y, uv, stride_c, w, h, frame, 0, h);
}

/*
 * The rows of an @h-row picture the dot passes through: all that
 * changes from one frame to the next (the dot only moves sideways).
 */
static inline void axera_scene_dot_rows(uint32_t h, uint32_t *r0,
                                        uint32_t *r1)
{
    int64_t a = (int64_t)(SSTAR_PAT_DOT_Y - SSTAR_PAT_DOT_R - 2) * h /
                SSTAR_PAT_H;
    int64_t b = (int64_t)(SSTAR_PAT_DOT_Y + SSTAR_PAT_DOT_R + 3) * h /
                SSTAR_PAT_H + 2;

    *r0 = MAX(a, 0) & ~1u;
    *r1 = MIN(b, (int64_t)h);
}

#endif /* HW_MISC_AXERA_SCENE_H */
