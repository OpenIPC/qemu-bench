/*
 * The world in front of an emulated HiSilicon camera: what the sensor
 * looks at and the camera's state for the current frame.
 * docs/hisilicon-digital-twin.md.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_HISI_SCENE_H
#define HW_MISC_HISI_SCENE_H

#include "qom/object.h"

#define TYPE_HISI_SCENE "hisi-scene"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSceneState, HISI_SCENE)

/*
 * The scene's linear reflectance at w x h, three floats a pixel (R, G, B),
 * 1.0 = white; an HDR source may go above.  Owned by the scene, valid until
 * the source or the size asked for changes.
 */
const float *hisi_scene_grid(HisiSceneState *s, int w, int h);

/*
 * A frame begins (from the ISP, before it measures the scene): the
 * timeline's events that are due apply.  Returns the timeline's clock,
 * seconds since the first frame.
 */
double hisi_scene_frame_begin(HisiSceneState *s);

/* The light: white's raw level a row at 1x gain, and its R / G / B tint */
double hisi_scene_light(HisiSceneState *s, double tint[3]);

/*
 * The current frame, from the ISP: raw level of white (16-bit units), the
 * light's tint it was captured under, and the white balance gains the
 * driver programmed (1.0 = none).  The picture is painted with these, not
 * with the scene's properties as they stand later.
 */
void hisi_scene_set_frame(HisiSceneState *s, double level,
                          const double tint[3], double wb_r, double wb_b);

/*
 * What the ISP makes of the scene for display: a row of w pixels of the
 * grid as BT.601 limited-range Y, and for every second pixel V and U
 * (NV21 order), through the frame's level, tint, white balance and a
 * 1/2.2 gamma.  uv may be NULL.
 */
void hisi_scene_display_row(HisiSceneState *s, const float *rgb, int w,
                            uint8_t *y, uint8_t *vu);

/*
 * hisi_scene_display_row() as tables, for a model that renders the same
 * grid under many exposures: component c (R, G, B) of reflectance
 * p = (i / (n - 1))^2 * pmax adds lut[c * n + i] = { Y, V, U } in 16.16
 * fixed point; Y = 16 + the sum, V and U = 128 + theirs (clamped to
 * +-112).  The frame's level, tint and white balance are applied.
 */
void hisi_scene_display_lut(HisiSceneState *s, int n, double pmax,
                            int32_t (*lut)[3]);

/*
 * How many frames hisi_scene_set_frame() has described: a picture model
 * that sees it stand still knows no ISP model sets the frame's exposure.
 */
uint32_t hisi_scene_frame_seq(HisiSceneState *s);

/* Changes whenever the source changes (a cached rendering is stale) */
uint32_t hisi_scene_generation(HisiSceneState *s);

/* The current frame as hisi_scene_set_frame() left it: level, tint, WB */
double hisi_scene_get_frame(HisiSceneState *s, double tint[3], double wb[3]);

#endif
