/*
 * Grain Media ISP328 image signal processor.
 *
 * The ISP at 0x93930000 on the GM813x (fisp328.ko, fisp_algorithm.ko;
 * their headers isp_common.h, isp_dev.h) sits between the sensor and the
 * capture engine.  Its window is register file and lookup tables
 * (0x0000-0x2FFF), which the model keeps as memory, apart from:
 *
 *   0x08 func0   bit 31 ISP on; 11-14 histogram / AE / AWB / AF
 *   0x48 input DMA   [31:8] a command table's address, bit 0 runs it
 *   0x4C output DMA  [31:8] the statistics copy table's address
 *   0x50 / 0x54  interrupt status 0 / 1, write 1 to clear
 *   0x58 / 0x5C  their masks, 1 = masked
 *   0x68 DMA control: reads 3 (DMA idle and disabled)
 *
 * Status 0: 0-3 AE / AWB / AF / histogram ready, 16 frame start, 17 frame
 * end, 18 input DMA done, 19 output DMA done; interrupt 43.  Frames run
 * at "fps" whether or not the ISP is on -- the driver waits for frame
 * start and end around every stop and resize.  At each frame start the
 * driver queues its register updates as an input DMA table:
 *
 *   header [31] last, [30] burst, [27:16] words incl. the header,
 *          [15:2] register offset; a burst has a dummy word, then the
 *          values for consecutive registers; otherwise the value follows
 *          and further offset / value pairs, the last offset flagged
 *
 * At each frame end the model fills the statistics registers, runs the
 * output DMA table -- pairs of [31] last | [27:16] words | [15:0]
 * register offset and a destination address, which the AE / AWB / AF
 * threads then read -- and raises frame end with the ready bits of the
 * enabled units (the driver handles those at the next frame start).
 *
 * The statistics: per AE window i (0x28C + 8i / 0x290 + 8i, start and end
 * x | y << 16) sums of R, G and B at 0x2800 + 0x18i, a quarter of the
 * window's pixels each, so that (R + 2G + B) / area is the mean level;
 * the histogram (0x2000, 16-bit bins; 0x2400, 32-bit) of the window at
 * 0x264, a count per 64 pixels; AWB (0x2A00) over the 2x2 blocks of its
 * window (0x2E0): ratio sums R/G and B/G in 1/128 (rg_lvl, bg_lvl, rg,
 * bg), the block count, and R, G, B sums; AF (0x2B00) a texture figure
 * per window (0x314).
 *
 * With a sensor (the "sensor" link) they measure the scene in front of
 * the camera (hisi-scene, docs/grainmedia.md "The digital twin"): its
 * reflectance times the light times the exposure the sensor applied to
 * the frame (image_sensor_exposure(); writes take effect a frame later,
 * image_sensor_frame_start()), clipped at the sensor's full scale, then
 * through what the driver programs (fisp328.ko's isp_hal_*):
 *
 *   0x250  global gain (isp_hal_set_global_gain), 1x = "gain-one"
 *   0x254  colour gains R [9:0], G [21:12]; 0x258 B [9:0]
 *          (isp_hal_set_color_gain), 1x = "gain-one"
 *   0x640  AE pre-gamma, 0x670 histogram pre-gamma
 *          (isp_hal_set_ae / hist_pre_gamma): 19 halfwords, knots x[6]
 *          and y[6] on an 8-bit scale and seven slopes in 1/256, y =
 *          slope0 x below x0, else y[i] + slope[i + 1] (x - x[i]); the
 *          AE's default (fisp_algorithm.ko) is close to 1 / 2.2
 *
 * The scene is measured while the ISP is on (func0 bit 31), so its clock
 * (the timeline's) starts with the driver's ISP; frames run at the
 * sensor's own rate (HTS x VTS / pclk) once it streams.  Without a
 * sensor, or before, the statistics describe a neutral grey of level
 * "luma" (the algorithms' default target is 100): AE settles at once and
 * AWB keeps unity gains.
 *
 * "record" writes a CSV line a frame, the columns of hisi-vi's record
 * (qemu-boot/twin-metrics.py reads either); "stats-trace" prints what
 * AE / AWB see once a second.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "qemu/error-report.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/hisi-scene.h"

#define TYPE_GM_ISP "gm-isp"
OBJECT_DECLARE_SIMPLE_TYPE(GmIspState, GM_ISP)

#define GM_ISP_SIZE         0x3000
#define GM_ISP_NREGS        (GM_ISP_SIZE / 4)
#define GM_ISP_MAX_WORDS    16384

#define R_FUNC0             0x008
#define R_IDMA              0x048
#define R_ODMA              0x04C
#define R_STATUS0           0x050
#define R_STATUS1           0x054
#define R_MASK0             0x058
#define R_MASK1             0x05C
#define R_DMACTL            0x068
#define R_HIST              0x2000
#define R_CE_HIST           0x2400
#define R_AE_SUM            0x2800
#define R_AE_CNT            0x2900
#define R_AWB               0x2A00
#define R_AF                0x2B00
#define R_HIST_WIN          0x264
#define R_AE_WIN            0x28C
#define R_AWB_WIN           0x2E0
#define R_AF_WIN            0x314
#define R_GLOBAL_GAIN       0x250
#define R_COLOR_GAIN0       0x254
#define R_COLOR_GAIN1       0x258
#define R_AE_PRE_GAMMA      0x640
#define R_HIST_PRE_GAMMA    0x670

/* the scene is measured on a grid of one sample per GRID_DIV^2 pixels */
#define GRID_DIV            8

#define FUNC0_ON            (1u << 31)
#define FUNC0_HIST          (1u << 11)
#define FUNC0_AE            (1u << 12)
#define FUNC0_AWB           (1u << 13)
#define FUNC0_AF            (1u << 14)

#define ST_AE               (1u << 0)
#define ST_AWB              (1u << 1)
#define ST_AF               (1u << 2)
#define ST_HIST             (1u << 3)
#define ST_VSTART           (1u << 16)
#define ST_VEND             (1u << 17)
#define ST_IDMA_DONE        (1u << 18)
#define ST_ODMA_DONE        (1u << 19)

#define DMA_GO              (1u << 0)

struct GmIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_ISP_NREGS];
    bool in_frame;              /* between frame start and end */
    uint32_t fps;
    uint32_t luma;
    uint32_t gain_one;          /* colour / global gain register at 1x */

    /* the digital twin */
    Object *sensor;
    HisiSceneState *scene;
    char *record;
    FILE *record_f;
    bool stats_trace;
    ImageSensorExposure expo;       /* written, for the next frame */
    ImageSensorExposure frame_expo; /* what the frame ending now had */
    uint32_t frames;
    double scene_t;
    double light;
    double tint[3];
    double level;
    double wb[3];
    uint32_t raw_mean[3];       /* the frame's raw means, 12-bit, for the
                                 * record */
    int fw, fh;                 /* the frame, pixels */
    int gw, gh;                 /* the grid: a sample per GRID_DIV^2 pixels,
                                 * the last row / column partial */
    uint8_t *grid;              /* the frame's samples after the ISP gains
                                 * and the AE pre-gamma, R G B */
    uint16_t *lin;              /* the same before the pre-gamma, 12-bit
                                 * as the ISP's pipeline, for AWB */
    uint8_t *hist_y;            /* each sample's luma, histogram pre-gamma */
};

static void gm_isp_update(GmIspState *s)
{
    qemu_set_irq(s->irq,
                 (s->regs[R_STATUS0 / 4] & ~s->regs[R_MASK0 / 4]) ||
                 (s->regs[R_STATUS1 / 4] & ~s->regs[R_MASK1 / 4]));
}

/* a window register pair: start x | y << 16, end (inclusive) */
static uint64_t gm_isp_win_area(GmIspState *s, uint32_t off,
                                uint32_t *w, uint32_t *h)
{
    const uint32_t a = s->regs[off / 4], b = s->regs[off / 4 + 1];
    const uint32_t x0 = extract32(a, 0, 13), y0 = extract32(a, 16, 13);
    const uint32_t x1 = extract32(b, 0, 13), y1 = extract32(b, 16, 13);

    *w = x1 >= x0 ? x1 - x0 + 1 : 0;
    *h = y1 >= y0 ? y1 - y0 + 1 : 0;
    return (uint64_t)*w * *h;
}

static void gm_isp_put64(GmIspState *s, uint32_t off, uint64_t v)
{
    s->regs[off / 4] = v;
    s->regs[off / 4 + 1] = v >> 32;
}

/* a window in grid samples, clipped to the grid; false if empty */
static bool gm_isp_win_grid(GmIspState *s, uint32_t off, int *gx0, int *gy0,
                            int *gx1, int *gy1)
{
    const uint32_t a = s->regs[off / 4], b = s->regs[off / 4 + 1];

    *gx0 = MIN(extract32(a, 0, 13) / GRID_DIV, s->gw);
    *gy0 = MIN(extract32(a, 16, 13) / GRID_DIV, s->gh);
    *gx1 = MIN(extract32(b, 0, 13) / GRID_DIV + 1, s->gw);
    *gy1 = MIN(extract32(b, 16, 13) / GRID_DIV + 1, s->gh);
    return *gx1 > *gx0 && *gy1 > *gy0;
}

/* the pixels grid sample (x, y) stands for: fewer at a partial edge */
static uint32_t gm_isp_cell(GmIspState *s, int x, int y)
{
    return MIN(GRID_DIV, s->fw - x * GRID_DIV) *
           MIN(GRID_DIV, s->fh - y * GRID_DIV);
}

/* a pre-gamma curve as the driver programs it (see the top) */
static uint32_t gm_isp_pre_gamma(GmIspState *s, uint32_t base, uint32_t v)
{
    uint16_t c[19];
    int i;

    for (i = 0; i < 19; i++) {
        c[i] = s->regs[base / 4 + i / 2] >> (16 * (i & 1));
    }
    if (!c[0] && !c[5]) {
        return v;                       /* not programmed: linear */
    }
    if (v < c[0]) {
        return MIN((c[12] * v) >> 8, 255);
    }
    for (i = 5; i > 0 && v < c[i]; i--) {
        ;
    }
    return MIN(c[6 + i] + ((c[13 + i] * (v - c[i])) >> 8), 255);
}

/*
 * A frame ends: the exposure it had is what the sensor applied for it
 * (or, for a sensor without frame timing, what was written a frame
 * earlier), and the scene is sampled through it and the ISP's gains.
 */
static void gm_isp_scene_frame(GmIspState *s)
{
    ImageSensorExposure now = { 0 };
    const float *refl;
    double global, k[3], rsum[3];
    uint8_t lut[256];
    int i, c, n;

    s->tint[0] = 0.55;
    s->tint[1] = 1.0;
    s->tint[2] = 0.7;
    s->light = 600;
    if (s->scene) {
        s->scene_t = hisi_scene_frame_begin(s->scene);
        s->light = hisi_scene_light(s->scene, s->tint);
    }
    image_sensor_exposure(s->sensor, &now);
    if (image_sensor_frame_start(s->sensor)) {
        s->frame_expo = now;
    } else {
        s->frame_expo = s->expo;
        s->expo = now;
    }
    s->level = (double)s->frame_expo.lines * s->frame_expo.gain / 1024 *
               s->light;

    /* what the driver programmed: white balance and the global gain */
    s->wb[0] = extract32(s->regs[R_COLOR_GAIN0 / 4], 0, 10);
    s->wb[1] = extract32(s->regs[R_COLOR_GAIN0 / 4], 12, 10);
    s->wb[2] = extract32(s->regs[R_COLOR_GAIN1 / 4], 0, 10);
    for (c = 0; c < 3; c++) {
        s->wb[c] = s->wb[c] ? s->wb[c] / s->gain_one : 1.0;
    }
    global = s->regs[R_GLOBAL_GAIN / 4] ?
              (double)s->regs[R_GLOBAL_GAIN / 4] / s->gain_one : 1.0;
    if (s->scene) {
        /* the picture: clipped at the sensor, then the same gains */
        double g[3];

        for (c = 0; c < 3; c++) {
            g[c] = s->wb[c] * global;
        }
        hisi_scene_set_frame_gains(s->scene, s->level, s->tint, g);
    }

    /* a sample's 8-bit value: raw (clipped at full scale), the gains,
     * the AE pre-gamma */
    for (c = 0; c < 3; c++) {
        k[c] = s->tint[c] * s->level / 65535.0;
    }
    for (i = 0; i < 256; i++) {
        lut[i] = gm_isp_pre_gamma(s, R_AE_PRE_GAMMA, i);
    }
    s->fw = s->frame_expo.width ? s->frame_expo.width : 1920;
    s->fh = s->frame_expo.height ? s->frame_expo.height : 1080;
    s->gw = DIV_ROUND_UP(s->fw, GRID_DIV);
    s->gh = DIV_ROUND_UP(s->fh, GRID_DIV);
    n = s->gw * s->gh;
    s->grid = g_renew(uint8_t, s->grid, (size_t)n * 3);
    s->lin = g_renew(uint16_t, s->lin, (size_t)n * 3);
    s->hist_y = g_renew(uint8_t, s->hist_y, n);
    refl = s->scene ? hisi_scene_grid(s->scene, s->gw, s->gh) : NULL;
    memset(rsum, 0, sizeof(rsum));
    for (i = 0; i < n; i++) {
        uint32_t lin[3];

        for (c = 0; c < 3; c++) {
            const double r = refl ? refl[i * 3 + c] : 0.5;
            const double raw = MIN(r * k[c], 1.0);
            const double v = MIN(raw * s->wb[c] * global, 1.0);

            rsum[c] += raw;

            lin[c] = lrint(v * 255);
            s->lin[i * 3 + c] = lrint(v * 4095);
            s->grid[i * 3 + c] = lut[lin[c]];
        }
        s->hist_y[i] = gm_isp_pre_gamma(s, R_HIST_PRE_GAMMA,
                                        (lin[0] + 2 * lin[1] + lin[2]) / 4);
    }
    for (c = 0; c < 3; c++) {
        s->raw_mean[c] = lrint(rsum[c] * 4095 / n);
    }
}

static void gm_isp_record(GmIspState *s, uint32_t r, uint32_t g, uint32_t b)
{
    uint64_t violations = 0;

    if (!s->record || !*s->record) {
        return;
    }
    if (!s->record_f) {
        s->record_f = fopen(s->record, "w");
        if (!s->record_f) {
            warn_report("gm-isp: cannot write the record '%s'", s->record);
            g_free(s->record);
            s->record = NULL;
            return;
        }
        fprintf(s->record_f, "t,frame,light,light_r,light_b,lines,gain,"
                "r,g,b,wb_r,wb_b,violations\n");
    }
    if (object_property_find(s->sensor, "violations")) {
        violations = object_property_get_uint(s->sensor, "violations", NULL);
    }
    fprintf(s->record_f, "%.4f,%u,%.0f,%.0f,%.0f,%u,%.3f,%u,%u,%u,%.3f,%.3f,"
            "%" PRIu64 "\n", s->scene_t, s->frames, s->light,
            s->tint[0] * 1000, s->tint[2] * 1000, s->frame_expo.lines,
            s->frame_expo.gain / 1024.0, r, g, b, s->wb[0] / s->wb[1],
            s->wb[2] / s->wb[1], violations);
    fflush(s->record_f);
}

/* the statistics of the scene, sampled by gm_isp_scene_frame() */
static void gm_isp_scene_statistics(GmIspState *s)
{
    uint64_t sum[3], all[3] = { 0 }, nall = 0;
    uint32_t hist[256] = { 0 };
    int gx0, gy0, gx1, gy1, x, y, c, i;

    /* AE: per window, a quarter of its pixels per colour */
    for (i = 0; i < 9; i++) {
        memset(sum, 0, sizeof(sum));
        if (gm_isp_win_grid(s, R_AE_WIN + 8 * i, &gx0, &gy0, &gx1, &gy1)) {
            for (y = gy0; y < gy1; y++) {
                for (x = gx0; x < gx1; x++) {
                    const uint32_t px = gm_isp_cell(s, x, y);

                    for (c = 0; c < 3; c++) {
                        sum[c] += s->grid[(y * s->gw + x) * 3 + c] * px;
                    }
                    nall += px;
                }
            }
            for (c = 0; c < 3; c++) {
                all[c] += sum[c];
            }
        }
        for (c = 0; c < 3; c++) {
            /* a quarter of the pixels are of this colour */
            gm_isp_put64(s, R_AE_SUM + 0x18 * i + 8 * c, sum[c] / 4);
        }
        memset(&s->regs[(R_AE_CNT + 0x10 * i) / 4], 0, 16);
    }

    /* the histogram: pixels, then a count per 64 of them */
    memset(&s->regs[R_HIST / 4], 0, 0x200);
    memset(&s->regs[R_CE_HIST / 4], 0, 0x400);
    if (gm_isp_win_grid(s, R_HIST_WIN, &gx0, &gy0, &gx1, &gy1)) {
        for (y = gy0; y < gy1; y++) {
            for (x = gx0; x < gx1; x++) {
                hist[s->hist_y[y * s->gw + x]] += gm_isp_cell(s, x, y);
            }
        }
    }
    for (i = 0; i < 256; i++) {
        hist[i] /= 64;                  /* a count per 64 pixels */
        s->regs[R_HIST / 4 + i / 2] |= MIN(hist[i], 0xFFFF) << (16 * (i & 1));
        s->regs[R_CE_HIST / 4 + i] = hist[i];
    }

    /* AWB: over 2x2 blocks, 16 to a full sample */
    memset(sum, 0, sizeof(sum));
    {
        uint64_t rg = 0, bg = 0, blocks = 0;

        if (gm_isp_win_grid(s, R_AWB_WIN, &gx0, &gy0, &gx1, &gy1)) {
            for (y = gy0; y < gy1; y++) {
                for (x = gx0; x < gx1; x++) {
                    const uint16_t *p = &s->lin[(y * s->gw + x) * 3];
                    const uint32_t g = MAX(p[1], 1);
                    const uint32_t nb = gm_isp_cell(s, x, y) / 4;

                    rg += (uint64_t)p[0] * 128 / g * nb;
                    bg += (uint64_t)p[2] * 128 / g * nb;
                    for (c = 0; c < 3; c++) {
                        sum[c] += (p[c] >> 4) * nb;     /* 8-bit */
                    }
                    blocks += nb;
                }
            }
        }
        s->regs[R_AWB / 4 + 0] = rg;            /* rg_lvl */
        s->regs[R_AWB / 4 + 1] = bg;            /* bg_lvl */
        s->regs[R_AWB / 4 + 2] = blocks;        /* pixel_num_lvl */
        s->regs[R_AWB / 4 + 3] = rg;
        s->regs[R_AWB / 4 + 4] = bg;
        s->regs[R_AWB / 4 + 5] = sum[0];
        s->regs[R_AWB / 4 + 6] = sum[1];
        s->regs[R_AWB / 4 + 7] = sum[2];
        s->regs[R_AWB / 4 + 8] = blocks;
        s->regs[R_AWB / 4 + 9] = 0;
    }

    /* AF: the window's horizontal gradient energy */
    for (i = 0; i < 9; i++) {
        uint64_t e = 0;

        if (gm_isp_win_grid(s, R_AF_WIN + 8 * i, &gx0, &gy0, &gx1, &gy1)) {
            for (y = gy0; y < gy1; y++) {
                for (x = gx0 + 1; x < gx1; x++) {
                    e += abs(s->grid[(y * s->gw + x) * 3 + 1] -
                             s->grid[(y * s->gw + x - 1) * 3 + 1]);
                }
            }
        }
        s->regs[R_AF / 4 + i] = MIN(e * GRID_DIV * GRID_DIV, 0xFFFFFFFF);
    }
    s->regs[R_AF / 4 + 9] = 0;

    s->frames++;
    if (nall) {
        for (c = 0; c < 3; c++) {
            all[c] /= nall;
        }
    }
    gm_isp_record(s, s->raw_mean[0], s->raw_mean[1], s->raw_mean[2]);
    if (s->stats_trace && s->frames % 30 == 0) {
        fprintf(stderr, "gm-isp: frame %u t %.2f light %.0f lines %u gain "
                "%.2f -> AE mean R %" PRIu64 " G %" PRIu64 " B %" PRIu64
                " (Y %" PRIu64 "), wb %.3f / %.3f / %.3f global 0x%x\n",
                s->frames, s->scene_t, s->light, s->frame_expo.lines,
                s->frame_expo.gain / 1024.0, all[0], all[1], all[2],
                (all[0] + 2 * all[1] + all[2]) / 4, s->wb[0], s->wb[1],
                s->wb[2], s->regs[R_GLOBAL_GAIN / 4]);
    }
}

static void gm_isp_statistics(GmIspState *s)
{
    const uint32_t y = MIN(s->luma, 255);
    uint32_t w, h, i;
    uint64_t area, n;

    if (s->sensor && (s->regs[R_FUNC0 / 4] & FUNC0_ON)) {
        gm_isp_scene_frame(s);
        gm_isp_scene_statistics(s);
        return;
    }

    /* histograms: a narrow bell around the level */
    area = gm_isp_win_area(s, R_HIST_WIN, &w, &h);
    memset(&s->regs[R_HIST / 4], 0, 0x200);
    memset(&s->regs[R_CE_HIST / 4], 0, 0x400);
    for (i = 0; i < 256; i++) {
        const int d = abs((int)i - (int)y);
        const uint64_t v = d < 8 ? area / 64 * (8 - d) / 64 : 0;

        s->regs[R_HIST / 4 + i / 2] |= MIN(v, 0xFFFF) << (16 * (i & 1));
        s->regs[R_CE_HIST / 4 + i] = MIN(v, 0xFFFFFFFF);
    }
    /* AE: R, G, B sums giving (R + 2G + B) / area = the level */
    for (i = 0; i < 9; i++) {
        const uint64_t sum = gm_isp_win_area(s, R_AE_WIN + 8 * i, &w, &h) *
                             y / 4;

        gm_isp_put64(s, R_AE_SUM + 0x18 * i, sum);
        gm_isp_put64(s, R_AE_SUM + 0x18 * i + 8, sum);
        gm_isp_put64(s, R_AE_SUM + 0x18 * i + 16, sum);
        memset(&s->regs[(R_AE_CNT + 0x10 * i) / 4], 0, 16);
    }
    /* AWB: neutral, 2x2 blocks */
    gm_isp_win_area(s, R_AWB_WIN, &w, &h);
    n = (uint64_t)(w / 2) * (h / 2);
    s->regs[R_AWB / 4 + 0] = n * 128;           /* rg_lvl */
    s->regs[R_AWB / 4 + 1] = n * 128;           /* bg_lvl */
    s->regs[R_AWB / 4 + 2] = n;                 /* pixel_num_lvl */
    s->regs[R_AWB / 4 + 3] = n * 128;           /* rg */
    s->regs[R_AWB / 4 + 4] = n * 128;           /* bg */
    s->regs[R_AWB / 4 + 5] = n * y;             /* r, g, b */
    s->regs[R_AWB / 4 + 6] = n * y;
    s->regs[R_AWB / 4 + 7] = n * y;
    s->regs[R_AWB / 4 + 8] = n;                 /* pixel_num */
    s->regs[R_AWB / 4 + 9] = 0;
    /* AF: a flat texture figure */
    for (i = 0; i < 9; i++) {
        s->regs[R_AF / 4 + i] = gm_isp_win_area(s, R_AF_WIN + 8 * i,
                                                &w, &h) * 8;
    }
    s->regs[R_AF / 4 + 9] = 0;
}

/* the statistics copy: register blocks to the algorithms' buffers */
static void gm_isp_output_dma(GmIspState *s)
{
    hwaddr tbl = s->regs[R_ODMA / 4] & ~0xFFu;
    int n;

    if (!tbl) {
        return;
    }
    for (n = 0; n < 64; n++, tbl += 8) {
        uint32_t e[2], off, words;

        dma_memory_read(&address_space_memory, tbl, e, 8,
                        MEMTXATTRS_UNSPECIFIED);
        e[0] = le32_to_cpu(e[0]);
        e[1] = le32_to_cpu(e[1]);
        off = e[0] & 0xFFFC;
        words = extract32(e[0], 16, 12);

        if (off + words * 4 <= GM_ISP_SIZE) {
            uint32_t buf[0x1000];       /* words is 12 bits */
            uint32_t i;

            for (i = 0; i < words; i++) {
                buf[i] = cpu_to_le32(s->regs[off / 4 + i]);
            }
            dma_memory_write(&address_space_memory, e[1], buf, words * 4,
                             MEMTXATTRS_UNSPECIFIED);
        }
        if (e[0] & (1u << 31)) {
            break;
        }
    }
    s->regs[R_STATUS0 / 4] |= ST_ODMA_DONE;
}

static void gm_isp_reg_write(GmIspState *s, uint32_t off, uint32_t val);

static uint32_t ldl_le_dma_or_zero(hwaddr a)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, a, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* the driver's register jobs */
static void gm_isp_input_dma(GmIspState *s)
{
    hwaddr p = s->regs[R_IDMA / 4] & ~0xFFu;
    int budget = GM_ISP_MAX_WORDS;

    while (budget > 0) {
        uint32_t hdr, off, words, v, i;

        hdr = ldl_le_dma_or_zero(p);
        off = hdr & 0xFFFC;
        words = extract32(hdr, 16, 12);
        if (hdr & (1u << 30)) {                 /* burst */
            for (i = 0; i + 2 < words && budget > 0; i++, budget--) {
                v = ldl_le_dma_or_zero(p + 8 + 4 * i);
                gm_isp_reg_write(s, off + 4 * i, v);
            }
            p += 4 * MAX(words, 2);
            budget -= 2;
        } else {                                /* offset / value pairs */
            v = ldl_le_dma_or_zero(p + 4);
            gm_isp_reg_write(s, off, v);
            p += 8;
            budget -= 2;
        }
        if (hdr & (1u << 31)) {
            break;
        }
    }
    s->regs[R_IDMA / 4] &= ~DMA_GO;
    s->regs[R_STATUS0 / 4] |= ST_IDMA_DONE;
}

/* a frame's length: the sensor's, once it streams */
static int64_t gm_isp_period(GmIspState *s)
{
    int64_t ns = 0;

    if (s->sensor && image_sensor_streaming(s->sensor)) {
        ns = image_sensor_frame_ns(s->sensor);
    }
    return ns > 0 ? ns : NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
}

static void gm_isp_frame(void *opaque)
{
    GmIspState *s = opaque;
    const int64_t period = gm_isp_period(s);
    const uint32_t f0 = s->regs[R_FUNC0 / 4];

    if (!s->in_frame) {
        s->regs[R_STATUS0 / 4] |= ST_VSTART;
        s->in_frame = true;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            period * 3 / 4);
    } else {
        gm_isp_statistics(s);
        gm_isp_output_dma(s);
        s->regs[R_STATUS0 / 4] |= ST_VEND;
        if (f0 & FUNC0_ON) {
            s->regs[R_STATUS0 / 4] |= (f0 & FUNC0_AE ? ST_AE : 0) |
                                      (f0 & FUNC0_AWB ? ST_AWB : 0) |
                                      (f0 & FUNC0_AF ? ST_AF : 0) |
                                      (f0 & FUNC0_HIST ? ST_HIST : 0);
        }
        s->in_frame = false;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            period / 4);
    }
    gm_isp_update(s);
}

static void gm_isp_reg_write(GmIspState *s, uint32_t off, uint32_t val)
{
    if (off >= GM_ISP_SIZE) {
        return;
    }
    switch (off) {
    case R_STATUS0:
    case R_STATUS1:
        s->regs[off / 4] &= ~val;
        break;
    case R_DMACTL:
        s->regs[off / 4] = val & ~0x10u;        /* the reset is done */
        break;
    case R_IDMA:
        s->regs[off / 4] = val;
        if (val & DMA_GO) {
            gm_isp_input_dma(s);
        }
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
}

static uint64_t gm_isp_read(void *opaque, hwaddr off, unsigned size)
{
    GmIspState *s = opaque;

    if (off == R_DMACTL) {
        return s->regs[off / 4] | 3;
    }
    return s->regs[off / 4];
}

static void gm_isp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmIspState *s = opaque;

    gm_isp_reg_write(s, off, val);
    gm_isp_update(s);
}

static const MemoryRegionOps gm_isp_ops = {
    .read = gm_isp_read,
    .write = gm_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_isp_reset(DeviceState *dev)
{
    GmIspState *s = GM_ISP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_MASK0 / 4] = s->regs[R_MASK1 / 4] = 0xFFFFFFFF;
    s->in_frame = false;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
    gm_isp_update(s);
}

static void gm_isp_init(Object *obj)
{
    GmIspState *s = GM_ISP(obj);

    /* settable after realize: the machine attaches the sensor later */
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static void gm_isp_realize(DeviceState *dev, Error **errp)
{
    GmIspState *s = GM_ISP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_isp_ops, s, TYPE_GM_ISP,
                          GM_ISP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_isp_frame, s);
}

static const Property gm_isp_props[] = {
    DEFINE_PROP_UINT32("fps", GmIspState, fps, 30),
    DEFINE_PROP_UINT32("luma", GmIspState, luma, 100),
    DEFINE_PROP_UINT32("gain-one", GmIspState, gain_one, 128),
    DEFINE_PROP_STRING("record", GmIspState, record),
    DEFINE_PROP_BOOL("stats-trace", GmIspState, stats_trace, false),
};

static const VMStateDescription vmstate_gm_isp = {
    .name = TYPE_GM_ISP,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmIspState),
        VMSTATE_UINT32_ARRAY(regs, GmIspState, GM_ISP_NREGS),
        VMSTATE_BOOL(in_frame, GmIspState),
        VMSTATE_UINT32_V(frames, GmIspState, 2),
        VMSTATE_UINT32_V(expo.lines, GmIspState, 2),
        VMSTATE_UINT32_V(expo.gain, GmIspState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_isp_realize;
    device_class_set_legacy_reset(dc, gm_isp_reset);
    device_class_set_props(dc, gm_isp_props);
    dc->vmsd = &vmstate_gm_isp;
}

static const TypeInfo gm_isp_info = {
    .name           = TYPE_GM_ISP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmIspState),
    .instance_init  = gm_isp_init,
    .class_init     = gm_isp_class_init,
};

static void gm_isp_register_types(void)
{
    type_register_static(&gm_isp_info);
}

type_init(gm_isp_register_types)
