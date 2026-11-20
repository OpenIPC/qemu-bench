/*
 * The scene the Rockchip camera models image: see include/hw/misc/rk-scene.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/i2c/image-sensor.h"
#include "hw/misc/rk-scene.h"

void rk_scene_rgb(uint32_t seq, double x, double y, double rgb[3])
{
    static const double bars[6][3] = {
        { 0.40, 0.06, 0.06 }, { 0.06, 0.40, 0.06 }, { 0.06, 0.06, 0.40 },
        { 0.06, 0.40, 0.40 }, { 0.40, 0.06, 0.40 }, { 0.40, 0.40, 0.06 },
    };
    double box = fmod(seq * 0.01, 0.9);
    double g = 0.06 + 0.30 * y;

    rgb[0] = rgb[1] = rgb[2] = g;
    if (y >= 0.40 && y < 0.60 && x >= 0.10 && x < 0.90) {
        memcpy(rgb, bars[MIN((int)((x - 0.10) / (0.80 / 6)), 5)],
               sizeof(bars[0]));
    }
    if (x >= 0.75 && x < 0.85 && y >= 0.08 && y < 0.18) {
        rgb[0] = rgb[1] = rgb[2] = 4.0;         /* the lamp */
    }
    if (x >= box && x < box + 0.08 && y >= 0.70 && y < 0.84) {
        rgb[0] = rgb[1] = rgb[2] = 0.70;        /* the moving box */
    }
}

double rk_scene_scale(Object *sensor)
{
    ImageSensorExposure e = { 0 };

    if (!sensor || !image_sensor_exposure(sensor, &e) || !e.lines) {
        e.lines = RK_SCENE_NOMINAL_LINES;
        e.gain = 1024;
    }
    return RK_SCENE_LEVEL_MAX * 2.0 / 3 / RK_SCENE_NOMINAL_LINES * e.lines *
           (e.gain / 1024.0);
}

uint8_t rk_scene_gamma(double v)
{
    return v <= 0 ? 0 : v >= RK_SCENE_LEVEL_MAX ? 255 :
           sqrt(v / RK_SCENE_LEVEL_MAX) * 255;
}

void rk_scene_yuv(const double level[3], bool limited, uint8_t yuv[3])
{
    double v[3];
    int c;

    for (c = 0; c < 3; c++) {
        v[c] = rk_scene_gamma(level[c]);
    }
    if (limited) {
        yuv[0] = 16 + (65.481 * v[0] + 128.553 * v[1] + 24.966 * v[2]) / 255;
        yuv[1] = 128 + (-37.797 * v[0] - 74.203 * v[1] + 112.0 * v[2]) / 255;
        yuv[2] = 128 + (112.0 * v[0] - 93.786 * v[1] - 18.214 * v[2]) / 255;
    } else {
        yuv[0] = MIN(0.299 * v[0] + 0.587 * v[1] + 0.114 * v[2], 255);
        yuv[1] = 128 + (-0.168736 * v[0] - 0.331264 * v[1] + 0.5 * v[2]);
        yuv[2] = 128 + (0.5 * v[0] - 0.418688 * v[1] - 0.081312 * v[2]);
    }
}
