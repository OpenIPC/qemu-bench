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

#endif
