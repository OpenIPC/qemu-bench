/*
 * The scene the Rockchip camera models image (hw/misc/rk-scene.c): what
 * the ISP measures and the ISP / ISPP outputs show, lit by the sensor's
 * exposure so that the vendor AE closes its loop on it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_RK_SCENE_H
#define HW_MISC_RK_SCENE_H

#include "qom/object.h"

#define RK_SCENE_LEVEL_MAX      4095.0  /* 12-bit raw */
#define RK_SCENE_NOMINAL_LINES  1000.0  /* rows at which a grey card is 2/3 */

/*
 * Reflectance per colour (R, G, B) at (@x, @y), 0..1 each way, in frame
 * @seq: a grey gradient, six colour bars, a lamp that clips and a box
 * that moves with the frame count.
 */
void rk_scene_rgb(uint32_t seq, double x, double y, double rgb[3]);

/*
 * Play a YUV4MPEG2 (4:2:0) recording, looped, in place of the test
 * pattern (the machine's scene=).
 */
bool rk_scene_set_source(const char *path, Error **errp);
bool rk_scene_has_source(void);

/*
 * Raw level per unit reflectance at @sensor's exposure (integration lines
 * x gain; a grey card at RK_SCENE_NOMINAL_LINES and 1x is two thirds of
 * full scale).  Nominal without a sensor or before it is programmed.
 */
double rk_scene_scale(Object *sensor);

/* A raw level through the (square root) gamma, 0..255. */
uint8_t rk_scene_gamma(double level);

/* Y, Cb, Cr of raw levels R, G, B: BT.601, limited or full range. */
void rk_scene_yuv(const double level[3], bool limited, uint8_t yuv[3]);

/* The same from R, G, B already through a gamma curve, 0..255. */
void rk_scene_yuv8(const double v[3], bool limited, uint8_t yuv[3]);

#endif
