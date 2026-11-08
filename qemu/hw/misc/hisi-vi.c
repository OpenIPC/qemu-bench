/*
 * HiSilicon V4 VI (Hi3516EV200 family): capture (VI_CAP) and the ISP
 * pipe (VI_PROC) as the frame clock of the online pipeline.
 *
 * The sensor's frames never exist here: VPSS (hisi-vpss) paints the
 * pictures.  What VI must do is what hi_vi.o waits for every frame:
 *
 *   VI_CAP (0x11000000):
 *     0x00f0  status: bit 8 pipe 0 (while its write channel has anything
 *             pending), bit 9 pipe 1 (VI_DRV_IsCapValidInt wants one of 0xf00)
 *     0x00f8  mask (the driver sets all of it when it loads)
 *     0x10000 + pipe * 0x1000 + 0xf0: the pipe's write-channel status,
 *             write 1s to clear: bit 0 frame start, bit 15 "early" (the
 *             online VPSS prepares its next frame, VPSS_COMM_VI_EarlyIrqProc),
 *             bit 1 frame end (the frame is sent on)
 *             (VI_COMM_CapOnLineFrameStartIrqRoute / ...EarlyEndIrqRoute)
 *   VI_CAP 0x10000 + pipe * 0x1000: the pipe's write channel, bit 31 on
 *   VI_CAP 0x1000 + dev * 0x100 + 0xf0: the port's status, write 1s to
 *             clear; never set (bits 1-2 are errors: ISP_ISR skips the
 *             frame on them)
 *   VI_CAP 0x200f0 / 0x200f8: the ISP front end's status (write 1s to
 *             clear: bit 0 frame start, bit 4 frame end) and mask, on the
 *             capture line too (the DTS's "ISP" interrupt is SPI 43): the
 *             ISP driver's per frame work, statistics and the 3A sync,
 *             runs from them (ISP_ISR)
 *   VI_PROC (0x11200000):
 *     0x0300  offline: writing 1 (or 3) starts the pipe on a captured frame
 *     0x0310  status, write 1s to clear: bit 10 frame start, bit 1 frame
 *             end; VI_COMM_ProcIrqRoute panics on a zero status
 *     0x0318  cleared with 0x310 (VI_HAL_ClearAnyProcInt)
 *
 * The ISP backend's registers sit in VI_PROC from 0x20000 (the SDK's
 * ISP_BE_REG_BASE, 0x11220000; offsets below are VI_PROC's).  At the end
 * of each frame the backend's statistics engines write what the 3A
 * algorithms measure (ISP_DRV_BE_STT_StatisticsRead /
 * ISP_DRV_BE_APB_StatisticsRead):
 *
 *   to DDR, at addresses the driver sets once (ISP_DRV_SetOnlineSttAddr):
 *     0x0668  AE histogram, 1024 u32 bins of the Bayer samples >> 6
 *     0x0670  AE zone means R << 16 | Gr, one u32 a zone, row by row
 *     0x0678  AE zone means Gb << 16 | B
 *     0x0680  AWB zones, two u32 each: G << 16 | R, count << 16 | B
 *   to registers:
 *     0x22018 / 0x2201c  AE pixels, total / counted
 *     0x22020..0x2202c   AE global R / Gr / Gb / B means
 *     0x22140..0x2214c   AWB global R / G / B means, count (a zone's,
 *                        every pixel: lib_hiawb holds its gains while the
 *                        zones' average is 1024 or less)
 *   read from registers: 0x22010 AE zones (hnum | vnum << 8, 17 x 15),
 *   0x22110 AWB zones (32 x 32)
 *
 * All values are 16-bit and the AWB means are taken before the white
 * balance gains (FE_MODULE_POS 0x20090 bits 3:2 not 2).  The front end
 * (VI_CAP 0x20000) measures each captured frame: a 256-bin AE histogram
 * (>> 8) at the STT address in VI_CAP 0x10208, which its driver swaps
 * every frame (ISP_DRV_FE_SwitchSttAddr), zone means at 0x10210 / 0x10218,
 * and AE1 globals at 0x22818..0x2282c laid out as the backend's.
 *
 * The statistics measure the scene in front of the camera ("scene" link,
 * hisi-scene: a picture or grey bars, and the light) as raw Bayer on a
 * grid of an eighth of the window.  How bright it comes out follows the
 * sensor's exposure and gain
 * (the image-sensor interface, "sensor" link) one frame late, as a sensor
 * applies what was written during the frame before; without a model of
 * them, white is half scale.  So auto exposure converges on an exposure
 * time, and auto white balance on gains that neutralise the light.  Each
 * frame VI hands the scene the level and the white balance gains, which
 * hisi-vpss paints the picture with.
 *
 * Offline the pipe's strips write their statistics themselves, one block
 * each at the address in their VIPROC register image (see
 * vi_be_stats_offline()).
 *
 * Every other register is storage.  Capture
 * interrupts go out on the first line, the pipe's on the second; the
 * "frame" GPIO pulses once a frame, for VPSS to write it.
 *
 * Frames run while the "run" GPIO is high (hisi-vpss raises it while its
 * online frame interrupt is unmasked, which the driver does only once the
 * whole pipeline exists) or pipe 0's write channel is enabled (the pipe's
 * start, in both modes).
 *
 * Online (VI_ONLINE_VPSS_ONLINE) the pipe follows capture: every frame
 * raises VI_PROC's start and end too, and pulses VPSS.  Offline
 * (VI_OFFLINE_VPSS_ONLINE, a sensor wider than 2304) capture only fills
 * DDR; the driver then starts the pipe on the frame (VI_PROC 0x300), VPSS
 * takes its output (the pulse goes out then) and the job's end comes on
 * the pipe's interrupt a quarter frame later.  The first start switches
 * the model to offline (until the write channel is disabled, the pipe's
 * stop); before it, the pipe's per-frame interrupt finds no job and is
 * harmless (VI_COMM_VIOffLineVPSSOffLineIrqRoute).  Nothing in VI's own
 * registers says so early enough or late enough: its interrupt mask is
 * set in full when the module loads, and an interrupt for a pipe that is
 * not there (any more) dereferences a NULL context in the driver.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/hisi-scene.h"
#include "qemu/error-report.h"
#include "system/dma.h"

#define TYPE_HISI_VI "hisi-vi"
OBJECT_DECLARE_SIMPLE_TYPE(HisiViState, HISI_VI)

#define VI_CAP_SIZE         0x200000
#define VI_PROC_SIZE        0x40000

#define CAP_INT             0x00f0
#define CAP_INT_MASK        0x00f8
#define CAP_INT_PIPE0       (1u << 8)
#define CAP_WCH_CTRL(p)     (0x10000 + (p) * 0x1000)    /* bit 31 enable */
#define CAP_WCH_INT(p)      (0x10000 + (p) * 0x1000 + 0xf0)
#define WCH_INT_FSTART      (1u << 0)
#define WCH_INT_FEND        (1u << 1)
#define WCH_INT_EARLY       (1u << 15)  /* lines in: VPSS prepares the next */
#define CAP_PT_INT(d)       (0x1000 + (d) * 0x100 + 0xf0)  /* VI port */
#define CAP_FE_INT(p)       (0x20000 + (p) * 0x20000 + 0xf0)    /* ISP FE */
#define CAP_FE_INT_MASK(p)  (0x20000 + (p) * 0x20000 + 0xf8)
#define FE_INT_FSTART       (1u << 0)
#define FE_INT_FEND         (1u << 4)

#define PROC_START          0x0300  /* offline: 1 / 3 starts the node list */
#define PROC_INT            0x0310
#define PROC_INT2           0x0318
#define PROC_INT_FEND       (1u << 1)
#define PROC_INT_FSTART     (1u << 10)

/* the ISP backend's statistics (see above) */
#define STT_AE_HIST         0x0668
#define STT_AE_RGR          0x0670
#define STT_AE_GBB          0x0678
#define STT_AWB             0x0680
#define BE_AE_ZONE          0x22010
#define BE_AE_TOTAL         0x22018
#define BE_AE_COUNT         0x2201c
#define BE_AE_AVER          0x22020     /* R, Gr, Gb, B */
#define BE_AWB_ZONE         0x22110
#define BE_AWB_AVG          0x22140     /* R, G, B, count */
#define PROC_PNEXT          0x02f8      /* offline: the pipe's node list */
#define PROC_STT_OFFLINE    0x05d0      /* offline: the strip's STT block */
#define PROC_BE_CFG         0x0240      /* offline: the strip's BE registers */
#define BE_WB_GAIN1         0x21210     /* R << 16 | Gr */
#define BE_WB_GAIN2         0x21214     /* B << 16 | Gb */
#define PROC_NODE_MAX       4           /* strips */
#define FE_STT_AE_HIST      0x10208     /* VI_CAP, pipe 0's channel */
#define FE_STT_AE_RGR       0x10210
#define FE_STT_AE_GBB       0x10218
#define FE_AE_ZONE          0x22810     /* VI_CAP: the front end's AE1 */
#define FE_AE_TOTAL         0x22818
#define FE_AE_COUNT         0x2281c
#define FE_AE_AVER          0x22820
#define FE_AE_BINS          256
#define AE_BINS             1024
#define AE_ZONES_MAX        (17 * 15)
#define AWB_ZONES_MAX       (32 * 32)

/* what a frame's statistics and record are made from */
typedef struct ViFrame {
    uint16_t *raw;
    int gw, gh;
    bool valid;
    double scene_t, light, tint[3], wb[2];
    ImageSensorExposure frame_expo;
} ViFrame;

struct HisiViState {
    SysBusDevice parent_obj;
    MemoryRegion cap_iomem;
    MemoryRegion proc_iomem;
    qemu_irq cap_irq;
    qemu_irq proc_irq;
    qemu_irq frame_out;
    QEMUTimer *timer;
    QEMUTimer *proc_timer;      /* an offline pipe job finishing */
    uint32_t *cap;
    uint32_t *proc;
    uint32_t fps;
    bool run;
    bool offline;               /* the driver starts the pipe per frame */
    bool trace;
    bool stats_trace;           /* a line of what AE / AWB see, each second */
    Object *sensor;             /* what the statistics measure */
    HisiSceneState *scene;      /* what is in front of the camera */
    ImageSensorExposure expo;   /* applied this frame (written the last) */
    ImageSensorExposure frame_expo;     /* what the current frame had */
    double level;               /* the current frame's white, 16-bit */
    uint16_t *raw;              /* the frame on the grid, R / G / B */
    double tint[3];             /* the light it was captured under */
    double light;               /* ...and its level */
    double scene_t;             /* the scene's timeline clock, s */
    double wb[2];               /* the WB gains the driver programmed */
    char *record;               /* a CSV of every measured frame */
    ViFrame job;                /* offline: the frame the pipe runs on */
    FILE *record_f;
    int gw, gh;
    uint32_t stat_frames;
};

/* the capture status: pipe 0's bit stands while its write channel has
 * anything pending, the driver clears only the latter */
static uint32_t vi_cap_status(HisiViState *s)
{
    return s->cap[CAP_INT / 4] |
           (s->cap[CAP_WCH_INT(0) / 4] ? CAP_INT_PIPE0 : 0);
}

static void vi_update(HisiViState *s)
{
    qemu_set_irq(s->cap_irq, (vi_cap_status(s) & s->cap[CAP_INT_MASK / 4] &
                              0xf00) ||
                             (s->cap[CAP_FE_INT(0) / 4] &
                              s->cap[CAP_FE_INT_MASK(0) / 4]));
    qemu_set_irq(s->proc_irq, s->proc[PROC_INT / 4] != 0);
}

/*
 * The next frame: as long as the sensor's registers make it (HTS x VTS /
 * pclk, so a driver's slow shutter or frame rate change shows), or the
 * "fps" property when the sensor does not model its timing.
 */
static void vi_arm(HisiViState *s)
{
    int64_t ns = image_sensor_frame_ns(s->sensor);

    if (ns < NANOSECONDS_PER_SECOND / 240 || ns > NANOSECONDS_PER_SECOND * 2) {
        ns = NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

static bool vi_running(HisiViState *s)
{
    return s->run || (s->cap[CAP_WCH_CTRL(0) / 4] & (1u << 31));
}


static void stt_write(uint32_t addr, const void *buf, size_t len)
{
    dma_memory_write(&address_space_memory, addr, buf, len,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * The frame as the sensor delivers it, measured on a grid of an eighth of
 * its window each way (hisi-scene's reflectance times the light, the
 * exposure and the gain, clipped at 16 bits).  A grid pixel stands for
 * 8 x 8 sensor pixels: 4 of the AE's samples (every 4th row and column),
 * each of them four Bayer samples, R, Gr, Gb, B.
 */
#define GRID_DIV            8
#define GRID_SAMPLES        4

/*
 * A captured frame: the exposure it had is what the sensor was told during
 * the frame before (a sensor applies its registers at the next frame start).
 */
static void vi_scene_frame(HisiViState *s)
{
    ImageSensorExposure now = { 0 };
    double tint[3], light = 600;
    const float *refl;
    int i, c;

    tint[0] = 0.55;
    tint[1] = 1.0;
    tint[2] = 0.7;
    if (s->scene) {
        s->scene_t = hisi_scene_frame_begin(s->scene);
        light = hisi_scene_light(s->scene, tint);
    }
    s->light = light;
    if (s->sensor && image_sensor_exposure(s->sensor, &now)) {
        if (image_sensor_frame_start(s->sensor)) {
            /*
             * The sensor keeps its own frame timing (sensor-sync.h): what
             * it applied for the frame ending now is this frame's, and
             * the frame start latches what was written for the next.
             */
            s->frame_expo = now;
        } else {
            /* writes apply at once: a sensor uses them a frame later */
            s->frame_expo = s->expo;
            s->expo = now;
        }
        s->level = (double)s->frame_expo.lines * s->frame_expo.gain / 1024 *
                   light;
    } else {
        memset(&s->frame_expo, 0, sizeof(s->frame_expo));
        s->level = 65535.0 / 2;     /* no sensor model: white at half */
    }

    s->gw = MAX((s->frame_expo.width ? s->frame_expo.width : 1920) /
                GRID_DIV, 1);
    s->gh = MAX((s->frame_expo.height ? s->frame_expo.height : 1080) /
                GRID_DIV, 1);
    s->raw = g_renew(uint16_t, s->raw, (size_t)s->gw * s->gh * 3);
    if (s->scene) {
        refl = hisi_scene_grid(s->scene, s->gw, s->gh);
        for (i = 0; i < s->gw * s->gh; i++) {
            for (c = 0; c < 3; c++) {
                s->raw[i * 3 + c] = MIN(refl[i * 3 + c] * tint[c] * s->level,
                                        65535.0);
            }
        }
    } else {
        for (i = 0; i < s->gw * s->gh * 3; i++) {
            s->raw[i] = MIN(s->level / 2 * tint[i % 3], 65535.0);
        }
    }
    memcpy(s->tint, tint, sizeof(s->tint));
    if (!s->offline) {
        /* the white balance the driver programmed: ISP_WB_GAIN1 / 2, R and
         * B in the high halves (offline: vi_offline_wb()) */
        s->wb[0] = (s->proc[BE_WB_GAIN1 / 4] >> 16) / 256.0;
        s->wb[1] = (s->proc[BE_WB_GAIN2 / 4] >> 16) / 256.0;
        if (s->scene) {
            hisi_scene_set_frame(s->scene, s->level, s->tint, s->wb[0],
                                 s->wb[1]);
        }
    }
}

/*
 * Offline, when the driver starts the pipe on a frame: its white balance
 * gains are in the strips' backend register images, the block at each
 * VIPROC node's 0x240 laid out from ISP_BE_REG_BASE (ISP_WB_GAIN1 / 2 at
 * +0x1210 / +0x1214), not in the MMIO registers.
 */
static void vi_offline_wb(HisiViState *s)
{
    uint32_t node[PROC_BE_CFG / 4 + 1], gain[2] = { 0, 0 };

    if (s->proc[PROC_PNEXT / 4]) {
        dma_memory_read(&address_space_memory, s->proc[PROC_PNEXT / 4], node,
                        sizeof(node), MEMTXATTRS_UNSPECIFIED);
        if (node[PROC_BE_CFG / 4]) {
            dma_memory_read(&address_space_memory,
                            node[PROC_BE_CFG / 4] + BE_WB_GAIN1 - 0x20000,
                            gain, sizeof(gain), MEMTXATTRS_UNSPECIFIED);
        }
    }
    s->wb[0] = (le32_to_cpu(gain[0]) >> 16) / 256.0;
    s->wb[1] = (le32_to_cpu(gain[1]) >> 16) / 256.0;
    if (s->scene) {
        hisi_scene_set_frame(s->scene, s->level, s->tint, s->wb[0],
                             s->wb[1]);
    }
}

/*
 * An offline job runs on the frame the driver started it on: what its
 * statistics and record use is kept from the start (vi_job_start()) and
 * swapped in at its end (vi_job_swap()), so a capture in between does not
 * mix two frames.
 */
static void vi_job_start(HisiViState *s)
{
    ViFrame *j = &s->job;

    j->raw = g_renew(uint16_t, j->raw, (size_t)MAX(s->gw * s->gh, 1) * 3);
    if (s->raw) {
        memcpy(j->raw, s->raw, (size_t)s->gw * s->gh * 3 * sizeof(*s->raw));
    }
    j->valid = s->raw != NULL;
    j->gw = s->gw;
    j->gh = s->gh;
    j->scene_t = s->scene_t;
    j->light = s->light;
    memcpy(j->tint, s->tint, sizeof(j->tint));
    memcpy(j->wb, s->wb, sizeof(j->wb));
    j->frame_expo = s->frame_expo;
}

static void vi_job_swap(HisiViState *s)
{
    ViFrame *j = &s->job;
    uint16_t *raw = s->raw;
    ViFrame live = {
        .raw = s->raw, .gw = s->gw, .gh = s->gh, .scene_t = s->scene_t,
        .light = s->light, .frame_expo = s->frame_expo,
    };

    memcpy(live.tint, s->tint, sizeof(live.tint));
    memcpy(live.wb, s->wb, sizeof(live.wb));
    s->raw = j->raw;
    s->gw = j->gw;
    s->gh = j->gh;
    s->scene_t = j->scene_t;
    s->light = j->light;
    memcpy(s->tint, j->tint, sizeof(s->tint));
    memcpy(s->wb, j->wb, sizeof(s->wb));
    s->frame_expo = j->frame_expo;
    live.valid = j->valid;
    *j = live;
    j->raw = raw;
}

/*
 * The record (docs/hisilicon-digital-twin.md, step 3): a CSV line a
 * measured frame, what the scene was, what the sensor applied, what the
 * statistics saw and what the 3A loop answered with.
 */
static void vi_record(HisiViState *s, uint32_t r, uint32_t g, uint32_t b)
{
    uint64_t violations = 0;

    if (!s->record || !*s->record) {
        return;
    }
    if (!s->record_f) {
        s->record_f = fopen(s->record, "w");
        if (!s->record_f) {
            warn_report("hisi-vi: cannot write the record '%s'", s->record);
            g_free(s->record);
            s->record = NULL;
            return;
        }
        fprintf(s->record_f, "t,frame,light,light_r,light_b,lines,gain,"
                "r,g,b,wb_r,wb_b,violations\n");
    }
    if (s->sensor && object_property_find(s->sensor, "violations")) {
        violations = object_property_get_uint(s->sensor, "violations", NULL);
    }
    fprintf(s->record_f, "%.4f,%u,%.0f,%.0f,%.0f,%u,%.3f,%u,%u,%u,%.3f,%.3f,"
            "%" PRIu64 "\n", s->scene_t, s->stat_frames, s->light,
            s->tint[0] * 1000, s->tint[2] * 1000, s->frame_expo.lines,
            s->frame_expo.gain / 1024.0, r, g, b, s->wb[0], s->wb[1],
            violations);
    fflush(s->record_f);
}

/* the mean of colour c over grid columns [x0, x1), rows [y0, y1) */
static uint32_t raw_mean(HisiViState *s, int c, int x0, int x1, int y0,
                         int y1)
{
    uint64_t sum = 0;
    int x, y;

    x1 = MAX(x1, x0 + 1);
    y1 = MAX(y1, y0 + 1);
    for (y = y0; y < y1; y++) {
        for (x = x0; x < x1; x++) {
            sum += s->raw[((size_t)y * s->gw + x) * 3 + c];
        }
    }
    return sum / ((uint64_t)(x1 - x0) * (y1 - y0));
}

/* zone z of n across d grid pixels: [*z0, return) */
static int zone_span(int z, int n, int d, int *z0)
{
    *z0 = MIN((int64_t)z * d / n, d - 1);
    return MAX((int64_t)(z + 1) * d / n, *z0 + 1);
}

/* the AE histogram of grid columns [x0, x1), bins of 65536 / nbins */
static void raw_hist(HisiViState *s, uint32_t *hist, int nbins, int x0,
                     int x1)
{
    const int shift = nbins == FE_AE_BINS ? 8 : 6;
    int x, y;

    for (y = 0; y < s->gh; y++) {
        for (x = x0; x < x1; x++) {
            const uint16_t *p = s->raw + ((size_t)y * s->gw + x) * 3;

            hist[p[0] >> shift] += GRID_SAMPLES;
            hist[p[1] >> shift] += 2 * GRID_SAMPLES;
            hist[p[2] >> shift] += GRID_SAMPLES;
        }
    }
}

/*
 * An AWB zone's count: every pixel of the zone (no skip).  lib_hiawb
 * averages the zones' counts and leaves its gains alone unless that
 * average passes 1024 (its ext register +0x30), and it ignores a zone
 * of 64 or fewer (AwbWbMatrixCalculate / AwbWbMatrixNormalize).
 */
static uint32_t scene_awb_count(HisiViState *s, int zones)
{
    return MIN((uint64_t)s->gw * s->gh * GRID_DIV * GRID_DIV /
               MAX(zones, 1), 0xffff);
}

/*
 * AE zone means, R << 16 | Gr and Gb << 16 | B, row by row: zone columns
 * [cx0, cx0 + width) of nx across the picture, into rows of width
 */
static void scene_ae_zones(HisiViState *s, int nx, int ny, int cx0,
                           int width, uint32_t *rgr, uint32_t *gbb)
{
    int x, y;

    for (y = 0; y < ny; y++) {
        int y0, y1 = zone_span(y, ny, s->gh, &y0);

        for (x = 0; x < width; x++) {
            int x0, x1 = zone_span(cx0 + x, nx, s->gw, &x0);
            const uint32_t r = raw_mean(s, 0, x0, x1, y0, y1);
            const uint32_t g = raw_mean(s, 1, x0, x1, y0, y1);
            const uint32_t b = raw_mean(s, 2, x0, x1, y0, y1);

            rgr[y * width + x] = cpu_to_le32(r << 16 | g);
            gbb[y * width + x] = cpu_to_le32(g << 16 | b);
        }
    }
}

/* AWB zones, G << 16 | R and count << 16 | B, as scene_ae_zones() */
static void scene_awb_zones(HisiViState *s, int nx, int ny, int cx0,
                            int width, uint32_t *awb)
{
    const uint32_t n = scene_awb_count(s, nx * ny);
    int x, y;

    for (y = 0; y < ny; y++) {
        int y0, y1 = zone_span(y, ny, s->gh, &y0);

        for (x = 0; x < width; x++) {
            int x0, x1 = zone_span(cx0 + x, nx, s->gw, &x0);
            const uint32_t r = raw_mean(s, 0, x0, x1, y0, y1);
            const uint32_t g = raw_mean(s, 1, x0, x1, y0, y1);
            const uint32_t b = raw_mean(s, 2, x0, x1, y0, y1);

            awb[(y * width + x) * 2] = cpu_to_le32(g << 16 | r);
            awb[(y * width + x) * 2 + 1] = cpu_to_le32(n << 16 | b);
        }
    }
}

/* AE global over grid columns [x0, x1): pixels total / counted, R, Gr,
 * Gb, B means */
static void scene_ae_global(HisiViState *s, uint32_t *regs, int x0, int x1)
{
    int c;

    regs[0] = MIN((uint64_t)(x1 - x0) * s->gh * GRID_SAMPLES * 4, 0x7ffffff);
    regs[1] = regs[0];
    for (c = 0; c < 4; c++) {
        regs[2 + c] = raw_mean(s, c == 0 ? 0 : c == 3 ? 2 : 1, x0, x1, 0,
                               s->gh);
    }
}

static void ae_grid(uint32_t zone, int *nx, int *ny)
{
    *nx = MAX(MIN(zone & 0x1f, 17), 1);
    *ny = MAX(MIN((zone >> 8) & 0x1f, 15), 1);
}

static void awb_grid(uint32_t zone, int *nx, int *ny)
{
    *nx = MAX(MIN(zone & 0x3f, 32), 1);
    *ny = MAX(MIN((zone >> 8) & 0x3f, 32), 1);
}

/* the front end's statistics of the frame captured now */
static void vi_fe_stats(HisiViState *s)
{
    uint32_t *p = s->cap;
    uint32_t rgr[AE_ZONES_MAX], gbb[AE_ZONES_MAX];
    int nx, ny;

    if (p[FE_STT_AE_HIST / 4]) {
        uint32_t hist[FE_AE_BINS] = { 0 };
        int b;

        raw_hist(s, hist, FE_AE_BINS, 0, s->gw);
        for (b = 0; b < FE_AE_BINS; b++) {
            hist[b] = cpu_to_le32(hist[b]);
        }
        stt_write(p[FE_STT_AE_HIST / 4], hist, sizeof(hist));
    }
    ae_grid(p[FE_AE_ZONE / 4], &nx, &ny);
    scene_ae_zones(s, nx, ny, 0, nx, rgr, gbb);
    if (p[FE_STT_AE_RGR / 4]) {
        stt_write(p[FE_STT_AE_RGR / 4], rgr, nx * ny * 4);
    }
    if (p[FE_STT_AE_GBB / 4]) {
        stt_write(p[FE_STT_AE_GBB / 4], gbb, nx * ny * 4);
    }
    scene_ae_global(s, p + FE_AE_TOTAL / 4, 0, s->gw);
}

/* the backend's statistics of the frame the pipe finishes now */
static void vi_be_stats(HisiViState *s)
{
    uint32_t *p = s->proc;
    uint32_t rgr[AE_ZONES_MAX], gbb[AE_ZONES_MAX];
    int nx, ny, awx, awy, c;

    if (!s->raw) {
        return;                     /* no frame captured yet */
    }

    if (p[STT_AE_HIST / 4]) {
        uint32_t hist[AE_BINS] = { 0 };
        int b;

        raw_hist(s, hist, AE_BINS, 0, s->gw);
        for (b = 0; b < AE_BINS; b++) {
            hist[b] = cpu_to_le32(hist[b]);
        }
        stt_write(p[STT_AE_HIST / 4], hist, sizeof(hist));
    }
    ae_grid(p[BE_AE_ZONE / 4], &nx, &ny);
    scene_ae_zones(s, nx, ny, 0, nx, rgr, gbb);
    if (p[STT_AE_RGR / 4]) {
        stt_write(p[STT_AE_RGR / 4], rgr, nx * ny * 4);
    }
    if (p[STT_AE_GBB / 4]) {
        stt_write(p[STT_AE_GBB / 4], gbb, nx * ny * 4);
    }
    scene_ae_global(s, p + BE_AE_TOTAL / 4, 0, s->gw);
    awb_grid(p[BE_AWB_ZONE / 4], &awx, &awy);
    if (p[STT_AWB / 4]) {
        uint32_t awb[AWB_ZONES_MAX * 2];

        scene_awb_zones(s, awx, awy, 0, awx, awb);
        stt_write(p[STT_AWB / 4], awb, awx * awy * 8);
    }
    for (c = 0; c < 3; c++) {
        p[BE_AWB_AVG / 4 + c] = raw_mean(s, c, 0, s->gw, 0, s->gh);
    }
    p[BE_AWB_AVG / 4 + 3] = scene_awb_count(s, awx * awy);

    if (s->stats_trace && s->stat_frames % MAX(s->fps, 1) == 0) {
        /* the WB gains the driver set (ISP_WB_GAIN1 / 2: R, B high) */
        fprintf(stderr, "isp stats: %u rows x%.2f gain, R %u G mean %u B %u, "
                "WB gains R %#x B %#x\n", s->frame_expo.lines,
                s->frame_expo.gain / 1024.0, p[BE_AWB_AVG / 4],
                p[BE_AWB_AVG / 4 + 1], p[BE_AWB_AVG / 4 + 2],
                p[BE_WB_GAIN1 / 4] >> 16, p[BE_WB_GAIN2 / 4] >> 16);
    }
    vi_record(s, p[BE_AWB_AVG / 4], p[BE_AWB_AVG / 4 + 1],
              p[BE_AWB_AVG / 4 + 2]);
    s->stat_frames++;
}

/*
 * Offline (VI_OFFLINE_VPSS_ONLINE) the pipe runs a list of VIPROC
 * register images in DDR, one a vertical strip, chained through their
 * PNEXT (0x2f8); each strip writes its statistics as one block
 * (S_ISP_STT_REGS_TYPE) to the address at its 0x5d0 (VI_HAL_SetIspSttAddr),
 * which VI copies to the ISP (VI_DRV_GetBeSttInfo).  Strip k holds the
 * zone columns n / strips (+ 1 for the first n % strips) of the AE and
 * AWB grids, row by row at that width; the ISP sums the histograms and
 * puts the columns side by side (ISP_DRV_BE_*_OfflineStatisticsRead).
 */
#define OFF_AE_GLOBAL       0x0000      /* total, count, R, Gr, Gb, B means */
#define OFF_AE_HIST         0x001c
#define OFF_AE_RGR          0x101c
#define OFF_AE_GBB          0x141c
#define OFF_AWB_GLOBAL      0x181c      /* R, G, B means, count */
#define OFF_AWB_ZONES       0x182c
#define OFF_STT_SIZE        0x382c      /* up to the AF statistics */

static int strip_cols(int cols, int strips, int k, int *first)
{
    int i, w = 0;

    *first = 0;
    for (i = 0; i <= k; i++) {
        *first += w;
        w = cols / strips + (i < cols % strips);
    }
    return w;
}

static void vi_be_stats_offline(HisiViState *s)
{
    const uint32_t aez = s->proc[BE_AE_ZONE / 4] ? s->proc[BE_AE_ZONE / 4]
                                                  : 0xf11;
    const uint32_t awz = s->proc[BE_AWB_ZONE / 4] ? s->proc[BE_AWB_ZONE / 4]
                                                   : 0x2020;
    uint32_t stt[PROC_NODE_MAX], node[PROC_STT_OFFLINE / 4 + 1];
    g_autofree uint32_t *blk = g_malloc(OFF_STT_SIZE);
    uint32_t a = s->proc[PROC_PNEXT / 4];
    int aex, aey, awx, awy, strips = 0, k, b, c;

    if (!s->raw) {
        return;                     /* the pipe ran before any capture */
    }
    ae_grid(aez, &aex, &aey);
    awb_grid(awz, &awx, &awy);
    for (k = 0; k < PROC_NODE_MAX && a; k++) {
        dma_memory_read(&address_space_memory, a, node, sizeof(node),
                        MEMTXATTRS_UNSPECIFIED);
        stt[k] = node[PROC_STT_OFFLINE / 4];
        a = node[PROC_PNEXT / 4];
        strips++;
    }
    for (k = 0; k < strips; k++) {
        int ae0, aw0, gx0, gx1;
        const int aew = strip_cols(aex, strips, k, &ae0);
        const int aww = strip_cols(awx, strips, k, &aw0);

        if (!stt[k]) {
            continue;
        }
        /* the strip's grid columns: its share of the AWB zones */
        gx0 = (int64_t)aw0 * s->gw / awx;
        gx1 = MAX((int64_t)(aw0 + aww) * s->gw / awx, gx0 + 1);
        memset(blk, 0, OFF_STT_SIZE);
        scene_ae_global(s, blk + OFF_AE_GLOBAL / 4, gx0, gx1);
        raw_hist(s, blk + OFF_AE_HIST / 4, AE_BINS, gx0, gx1);
        scene_ae_zones(s, aex, aey, ae0, aew, blk + OFF_AE_RGR / 4,
                       blk + OFF_AE_GBB / 4);
        for (c = 0; c < 3; c++) {
            blk[OFF_AWB_GLOBAL / 4 + c] = raw_mean(s, c, gx0, gx1, 0, s->gh);
        }
        blk[OFF_AWB_GLOBAL / 4 + 3] = scene_awb_count(s, awx * awy);
        scene_awb_zones(s, awx, awy, aw0, aww, blk + OFF_AWB_ZONES / 4);
        /* the zone writers stored little-endian already */
        for (b = 0; b < OFF_AE_RGR / 4; b++) {
            blk[b] = cpu_to_le32(blk[b]);
        }
        for (b = OFF_AWB_GLOBAL / 4; b < OFF_AWB_ZONES / 4; b++) {
            blk[b] = cpu_to_le32(blk[b]);
        }
        stt_write(stt[k], blk, OFF_STT_SIZE);
    }
    if (s->stats_trace && s->stat_frames % MAX(s->fps, 1) == 0) {
        /* the gains are in the strips' register images, not shown */
        fprintf(stderr, "isp stats: %u rows x%.2f gain, R %u G mean %u B %u, "
                "offline, %d strips\n", s->frame_expo.lines,
                s->frame_expo.gain / 1024.0,
                raw_mean(s, 0, 0, s->gw, 0, s->gh),
                raw_mean(s, 1, 0, s->gw, 0, s->gh),
                raw_mean(s, 2, 0, s->gw, 0, s->gh), strips);
    }
    vi_record(s, raw_mean(s, 0, 0, s->gw, 0, s->gh),
              raw_mean(s, 1, 0, s->gw, 0, s->gh),
              raw_mean(s, 2, 0, s->gw, 0, s->gh));
    s->stat_frames++;
}

static void vi_frame(void *opaque)
{
    HisiViState *s = opaque;

    if (!vi_running(s)) {
        return;                     /* the pipeline was stopped */
    }
    vi_scene_frame(s);
    vi_fe_stats(s);
    s->cap[CAP_WCH_INT(0) / 4] |= WCH_INT_FSTART | WCH_INT_EARLY | WCH_INT_FEND;
    /* the ISP front end's frame: its driver's per-frame work (ISP_ISR) */
    s->cap[CAP_FE_INT(0) / 4] |= FE_INT_FSTART | FE_INT_FEND;
    if (!s->offline && s->run) {
        /* online: the pipe runs with capture, VPSS behind it */
        vi_be_stats(s);
        s->proc[PROC_INT / 4] |= PROC_INT_FSTART | PROC_INT_FEND;
        qemu_irq_pulse(s->frame_out);
    }
    vi_update(s);
    vi_arm(s);
}

/* offline: the job the driver started on a captured frame is done */
static void vi_proc_done(void *opaque)
{
    HisiViState *s = opaque;

    if (!vi_running(s)) {
        return;                     /* the pipe was stopped meanwhile */
    }
    if (s->job.valid) {
        /* the statistics and record of the frame the job was started on */
        vi_job_swap(s);
        vi_be_stats_offline(s);
        vi_job_swap(s);
    }
    s->proc[PROC_INT / 4] |= PROC_INT_FSTART | PROC_INT_FEND;
    vi_update(s);
}

static void vi_run(void *opaque, int n, int level)
{
    HisiViState *s = opaque;

    s->run = level;
    if (vi_running(s) && !timer_pending(s->timer)) {
        vi_arm(s);
    }
}

static uint64_t vi_cap_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViState *s = HISI_VI(opaque);
    const uint32_t val = offset == CAP_INT ? vi_cap_status(s)
                                           : s->cap[offset / 4];

    if (s->trace) {
        fprintf(stderr, "vi-cap R %06x = %08x\n", (int)offset, val);
    }
    return val;
}

static void vi_cap_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace) {
        fprintf(stderr, "vi-cap W %06x : %08x\n", (int)offset, (uint32_t)val);
    }
    if (offset == CAP_INT || offset == CAP_WCH_INT(0) ||
        offset == CAP_WCH_INT(1) || offset == CAP_FE_INT(0) ||
        (offset >= CAP_PT_INT(0) && offset <= CAP_PT_INT(3) &&
         (offset & 0xff) == 0xf0)) {
        s->cap[offset / 4] &= ~(uint32_t)val;
    } else {
        s->cap[offset / 4] = val;
    }
    if (offset == CAP_WCH_CTRL(0)) {
        if (!(val & (1u << 31))) {
            /* the pipe stopped: whatever it is rebuilt as starts online */
            s->offline = false;
            timer_del(s->proc_timer);
        } else if (vi_running(s) && !timer_pending(s->timer)) {
            vi_arm(s);              /* capture into DDR enabled */
        }
    }
    vi_update(s);
}

static uint64_t vi_proc_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace && offset < 0x1000) {
        fprintf(stderr, "vi-proc R %05x = %08x\n", (int)offset,
                s->proc[offset / 4]);
    }
    return s->proc[offset / 4];
}

static void vi_proc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    HisiViState *s = HISI_VI(opaque);

    if (s->trace && offset < 0x1000) {
        fprintf(stderr, "vi-proc W %05x : %08x\n", (int)offset,
                (uint32_t)val);
    }
    if (offset == PROC_INT || offset == PROC_INT2) {
        s->proc[offset / 4] &= ~(uint32_t)val;
    } else {
        s->proc[offset / 4] = val;
    }
    if (offset == PROC_START && (val & 1)) {
        /*
         * Offline (a sensor too wide for online, VI_OFFLINE_VPSS_ONLINE):
         * the driver starts the pipe on each captured frame (VI_HAL_Start),
         * VPSS takes the pipe's output online, and the job's end comes on
         * the pipe's interrupt.
         */
        s->offline = true;
        vi_offline_wb(s);
        vi_job_start(s);
        qemu_irq_pulse(s->frame_out);
        timer_mod(s->proc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / 4 / MAX(s->fps, 1));
    }
    vi_update(s);
}

static const MemoryRegionOps vi_cap_ops = {
    .read = vi_cap_read,
    .write = vi_cap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps vi_proc_ops = {
    .read = vi_proc_read,
    .write = vi_proc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vi_init(Object *obj)
{
    HisiViState *s = HISI_VI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->cap = g_new0(uint32_t, VI_CAP_SIZE / 4);
    s->proc = g_new0(uint32_t, VI_PROC_SIZE / 4);
    memory_region_init_io(&s->cap_iomem, obj, &vi_cap_ops, s, "hisi-vi-cap",
                          VI_CAP_SIZE);
    memory_region_init_io(&s->proc_iomem, obj, &vi_proc_ops, s,
                          "hisi-vi-proc", VI_PROC_SIZE);
    sysbus_init_mmio(sbd, &s->cap_iomem);
    sysbus_init_mmio(sbd, &s->proc_iomem);
    sysbus_init_irq(sbd, &s->cap_irq);
    sysbus_init_irq(sbd, &s->proc_irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_out, "frame", 1);
    qdev_init_gpio_in_named(DEVICE(obj), vi_run, "run", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vi_frame, s);
    s->proc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vi_proc_done, s);
    /* settable after realize: the machine attaches the sensor later */
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static void hisi_vi_reset(DeviceState *dev)
{
    HisiViState *s = HISI_VI(dev);

    timer_del(s->timer);
    timer_del(s->proc_timer);
    s->run = false;
    s->offline = false;
    memset(&s->expo, 0, sizeof(s->expo));
    memset(&s->frame_expo, 0, sizeof(s->frame_expo));
    s->level = 0;
    s->stat_frames = 0;
    memset(s->cap, 0, VI_CAP_SIZE);
    memset(s->proc, 0, VI_PROC_SIZE);
    qemu_set_irq(s->cap_irq, 0);
    qemu_set_irq(s->proc_irq, 0);
}

static const Property hisi_vi_properties[] = {
    DEFINE_PROP_UINT32("fps", HisiViState, fps, 25),
    DEFINE_PROP_BOOL("trace", HisiViState, trace, false),
    DEFINE_PROP_BOOL("stats-trace", HisiViState, stats_trace, false),
    DEFINE_PROP_STRING("record", HisiViState, record),
};

static void hisi_vi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, hisi_vi_properties);
    device_class_set_legacy_reset(dc, hisi_vi_reset);
}

static const TypeInfo hisi_vi_info = {
    .name          = TYPE_HISI_VI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiViState),
    .instance_init = hisi_vi_init,
    .class_init    = hisi_vi_class_init,
};

static void hisi_vi_register_types(void)
{
    type_register_static(&hisi_vi_info);
}

type_init(hisi_vi_register_types)
