/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_INGENIC_COLOR_H
#define HW_MISC_INGENIC_COLOR_H

typedef struct IngenicRGB {
    int r, g, b;
} IngenicRGB;

/* BT.601 limited-range YCbCr to full-range RGB, Q10 coefficients. */
static inline IngenicRGB ingenic_yuv_to_rgb(uint8_t y, uint8_t cb, uint8_t cr)
{
    int c = (y - 16) * 1192;
    int d = cb - 128, e = cr - 128;

    return (IngenicRGB) {
        .r = CLAMP((c + 1634 * e) >> 10, 0, 255),
        .g = CLAMP((c - 401 * d - 833 * e) >> 10, 0, 255),
        .b = CLAMP((c + 2066 * d) >> 10, 0, 255),
    };
}

#endif
