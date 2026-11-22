/*
 * The scene the Rockchip camera models image: see include/hw/misc/rk-scene.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/i2c/image-sensor.h"
#include "hw/misc/rk-scene.h"
#include "qapi/error.h"

/*
 * scene=FILE: a YUV4MPEG2 recording (4:2:0, 8-bit) in place of the test
 * pattern, frame seq % its frames.  Its samples are BT.601 limited range
 * through a 2.2 gamma: back to linear, white at reflectance 0.9.  The
 * sensor's exposure still lights it, so AE closes its loop as before.
 */
static struct {
    FILE *f;
    uint32_t w, h, frames, cur;
    long first;
    size_t fsize;
    uint8_t *pic;
    float lin[256];
} rk_src;

bool rk_scene_has_source(void)
{
    return rk_src.f != NULL;
}

bool rk_scene_set_source(const char *path, Error **errp)
{
    char hdr[256], *tok;
    long end;
    int i;

    rk_src.f = fopen(path, "rb");
    if (!rk_src.f) {
        error_setg_errno(errp, errno, "rk-scene: %s", path);
        return false;
    }
    if (!fgets(hdr, sizeof(hdr), rk_src.f) ||
        strncmp(hdr, "YUV4MPEG2 ", 10)) {
        error_setg(errp, "rk-scene: %s is not YUV4MPEG2", path);
        return false;
    }
    for (tok = strtok(hdr + 10, " \n"); tok; tok = strtok(NULL, " \n")) {
        if (tok[0] == 'W') {
            rk_src.w = atoi(tok + 1);
        } else if (tok[0] == 'H') {
            rk_src.h = atoi(tok + 1);
        } else if (tok[0] == 'C' && strncmp(tok, "C420", 4)) {
            error_setg(errp, "rk-scene: %s is %s, not 4:2:0", path, tok);
            return false;
        }
    }
    if (!rk_src.w || !rk_src.h || rk_src.w > 8192 || rk_src.h > 8192 ||
        (rk_src.w | rk_src.h) & 1) {
        error_setg(errp, "rk-scene: %s is %ux%u", path, rk_src.w, rk_src.h);
        return false;
    }
    rk_src.first = ftell(rk_src.f);
    rk_src.fsize = (size_t)rk_src.w * rk_src.h * 3 / 2;
    fseek(rk_src.f, 0, SEEK_END);
    end = ftell(rk_src.f);
    rk_src.frames = (end - rk_src.first) / (rk_src.fsize + 6);
    if (!rk_src.frames) {
        error_setg(errp, "rk-scene: %s has no frames", path);
        return false;
    }
    rk_src.pic = g_malloc(rk_src.fsize);
    rk_src.cur = UINT32_MAX;
    for (i = 0; i < 256; i++) {
        rk_src.lin[i] = 0.9 * pow(i / 255.0, 2.2);
    }
    return true;
}

static void rk_scene_src_rgb(uint32_t seq, double x, double y, double rgb[3])
{
    uint32_t n = seq % rk_src.frames, px, py;
    const uint8_t *c;
    double yy, u, v, r, g, b;

    if (n != rk_src.cur) {
        /* "FRAME\n" before each picture: the frames are all one size */
        fseek(rk_src.f, rk_src.first + (long)n * (rk_src.fsize + 6) + 6,
              SEEK_SET);
        if (fread(rk_src.pic, 1, rk_src.fsize, rk_src.f) != rk_src.fsize) {
            memset(rk_src.pic, 128, rk_src.fsize);
        }
        rk_src.cur = n;
    }
    px = MIN((uint32_t)(x * rk_src.w), rk_src.w - 1);
    py = MIN((uint32_t)(y * rk_src.h), rk_src.h - 1);
    c = rk_src.pic + (size_t)rk_src.w * rk_src.h;
    yy = (rk_src.pic[(size_t)py * rk_src.w + px] - 16) * (255.0 / 219);
    u = c[(size_t)(py / 2) * (rk_src.w / 2) + px / 2] - 128.0;
    v = c[(size_t)rk_src.w * rk_src.h / 4 +
          (size_t)(py / 2) * (rk_src.w / 2) + px / 2] - 128.0;
    r = yy + 1.596 * v;
    g = yy - 0.392 * u - 0.813 * v;
    b = yy + 2.017 * u;
    rgb[0] = rk_src.lin[(int)MIN(MAX(lrint(r), 0), 255)];
    rgb[1] = rk_src.lin[(int)MIN(MAX(lrint(g), 0), 255)];
    rgb[2] = rk_src.lin[(int)MIN(MAX(lrint(b), 0), 255)];
}

void rk_scene_rgb(uint32_t seq, double x, double y, double rgb[3])
{
    static const double bars[6][3] = {
        { 0.40, 0.06, 0.06 }, { 0.06, 0.40, 0.06 }, { 0.06, 0.06, 0.40 },
        { 0.06, 0.40, 0.40 }, { 0.40, 0.06, 0.40 }, { 0.40, 0.40, 0.06 },
    };
    double box = fmod(seq * 0.01, 0.9);
    double g = 0.06 + 0.30 * y;

    if (rk_src.f) {
        rk_scene_src_rgb(seq, x, y, rgb);
        return;
    }

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
    rk_scene_yuv8(v, limited, yuv);
}

/* rounded to the nearest level: truncated, a grey's 127.9999 came out 127 */
static uint8_t rk_scene_lvl(double x)
{
    return lrint(MIN(MAX(x, 0), 255));
}

void rk_scene_yuv8(const double v[3], bool limited, uint8_t yuv[3])
{
    if (limited) {
        yuv[0] = rk_scene_lvl(16 + (65.481 * v[0] + 128.553 * v[1] +
                                    24.966 * v[2]) / 255);
        yuv[1] = rk_scene_lvl(128 + (-37.797 * v[0] - 74.203 * v[1] +
                                     112.0 * v[2]) / 255);
        yuv[2] = rk_scene_lvl(128 + (112.0 * v[0] - 93.786 * v[1] -
                                     18.214 * v[2]) / 255);
    } else {
        yuv[0] = rk_scene_lvl(0.299 * v[0] + 0.587 * v[1] + 0.114 * v[2]);
        yuv[1] = rk_scene_lvl(128 + (-0.168736 * v[0] - 0.331264 * v[1] +
                                     0.5 * v[2]));
        yuv[2] = rk_scene_lvl(128 + (0.5 * v[0] - 0.418688 * v[1] -
                                     0.081312 * v[2]));
    }
}
