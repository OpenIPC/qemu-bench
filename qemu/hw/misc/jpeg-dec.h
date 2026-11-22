/*
 * A baseline JPEG decoder (ITU T.81, sequential Huffman, 8-bit), for the
 * device models whose hardware decodes JPEG: each component's plane at
 * its own sampling, padded to whole MCUs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_JPEG_DEC_H
#define HW_MISC_JPEG_DEC_H

typedef struct JpegDecImage {
    int w, h;                   /* the frame's size */
    int ncomp;                  /* 1 (grey) or 3 (Y, Cb, Cr) */
    int hs[3], vs[3];           /* sampling factors */
    int hmax, vmax;
    int pw[3], ph[3];           /* each plane's size, whole MCUs */
    uint8_t *plane[3];          /* row-major, pw bytes a row */
} JpegDecImage;

/*
 * Decode the JPEG file @buf (SOI .. EOI) into @img; false with @err set
 * (a static string) when it is not one this decoder takes (progressive,
 * arithmetic, 12-bit, more than three components) or is broken.
 */
bool jpeg_dec_decode(const uint8_t *buf, size_t len, JpegDecImage *img,
                     const char **err);
void jpeg_dec_free(JpegDecImage *img);

#endif
