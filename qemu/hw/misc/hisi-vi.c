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
 *     0x22140..0x2214c   AWB global R / G / B means, zone count
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
 * The scene is the one hisi-vpss paints, eight grey bars, as raw Bayer
 * under a light of R 0.55, G 1, B 0.7 ("light-r" / "light-b", per mille
 * of G).  How bright it comes out follows the sensor's exposure and gain
 * (the image-sensor interface, "sensor" link) one frame late, as a sensor
 * applies what was written during the frame before; without a model of
 * them, white is half scale.  So auto exposure converges on an exposure
 * time, and auto white balance measures the light's colour temperature.
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
#define SCENE_BARS          8

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
    uint32_t light;             /* white, 16-bit units a row at 1x */
    uint32_t light_r, light_b;  /* its colour: R, B per mille of G */
    ImageSensorExposure expo;   /* applied this frame (written the last) */
    ImageSensorExposure frame_expo;     /* what the current frame had */
    double level;               /* the current frame's white, 16-bit */
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

static void vi_arm(HisiViState *s)
{
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / MAX(s->fps, 1));
}

static bool vi_running(HisiViState *s)
{
    return s->run || (s->cap[CAP_WCH_CTRL(0) / 4] & (1u << 31));
}


/*
 * The scene, in 16-bit raw: SCENE_BARS vertical grey bars (hisi-vpss's,
 * light to dark) under a light whose raw response is R 0.55, G 1, B 0.7
 * of white, times the exposure, clipped.
 */
static const uint8_t scene_y[SCENE_BARS] = { 235, 210, 170, 145, 106, 81, 41, 16 };
/* the light's raw response, R and B per mille of G */
static uint32_t scene_light_r = 550, scene_light_b = 700;

static double scene_level(int bar, int c, double expo)
{
    double refl = MAX(scene_y[bar] - 16, 6) / 219.0;
    double tint = c == 0 ? scene_light_r / 1000.0 :
                  c == 2 ? scene_light_b / 1000.0 : 1.0;

    return MIN(refl * tint * expo, 65535.0);
}

/* the mean of colour c over [x0, x1) of the width (0..1), bars clipped */
static double scene_mean(int c, double x0, double x1, double expo)
{
    double sum = 0;
    int b;

    for (b = 0; b < SCENE_BARS; b++) {
        double l = MAX(x0, (double)b / SCENE_BARS);
        double r = MIN(x1, (double)(b + 1) / SCENE_BARS);

        if (r > l) {
            sum += (r - l) * scene_level(b, c, expo);
        }
    }
    return x1 > x0 ? sum / (x1 - x0) : 0;
}

static void stt_write(uint32_t addr, const void *buf, size_t len)
{
    dma_memory_write(&address_space_memory, addr, buf, len,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * A captured frame: the exposure it had, which is what the sensor was
 * told during the frame before (a sensor applies its registers at the
 * next frame start).
 */
static void vi_scene_frame(HisiViState *s)
{
    ImageSensorExposure now = { 0 };

    scene_light_r = s->light_r;
    scene_light_b = s->light_b;

    if (s->sensor && image_sensor_exposure(s->sensor, &now)) {
        s->frame_expo = s->expo;
        s->level = (double)s->expo.lines * s->expo.gain / 1024 * s->light;
        s->expo = now;
    } else {
        memset(&s->frame_expo, 0, sizeof(s->frame_expo));
        s->level = 65535.0 / 2;     /* no sensor model: white at half */
    }
}

/* every 4th row and column (the AE skip), all four Bayer samples */
static uint32_t scene_samples(HisiViState *s)
{
    const uint32_t w = s->frame_expo.width ? s->frame_expo.width : 1920;
    const uint32_t h = s->frame_expo.height ? s->frame_expo.height : 1080;

    return (w / 4) * (h / 4);
}

/* AE zone means, R << 16 | Gr and Gb << 16 | B, row by row */
static void scene_ae_zones(HisiViState *s, uint32_t zone, uint32_t rgr_addr,
                           uint32_t gbb_addr)
{
    uint32_t rgr[AE_ZONES_MAX], gbb[AE_ZONES_MAX];
    int nx = MAX(MIN(zone & 0x1f, 17), 1);
    int ny = MAX(MIN((zone >> 8) & 0x1f, 15), 1);
    int x, y;

    for (x = 0; x < nx; x++) {
        const double x0 = (double)x / nx, x1 = (double)(x + 1) / nx;
        const uint32_t r = scene_mean(0, x0, x1, s->level);
        const uint32_t g = scene_mean(1, x0, x1, s->level);
        const uint32_t b = scene_mean(2, x0, x1, s->level);

        for (y = 0; y < ny; y++) {
            rgr[y * nx + x] = cpu_to_le32(r << 16 | g);
            gbb[y * nx + x] = cpu_to_le32(g << 16 | b);
        }
    }
    if (rgr_addr) {
        stt_write(rgr_addr, rgr, nx * ny * 4);
    }
    if (gbb_addr) {
        stt_write(gbb_addr, gbb, nx * ny * 4);
    }
}

/* an AE histogram of the Bayer samples, bins of 65536 / nbins */
static void scene_ae_hist(HisiViState *s, uint32_t addr, int nbins)
{
    uint32_t hist[AE_BINS] = { 0 };
    const uint32_t n = scene_samples(s) / SCENE_BARS;
    const int shift = nbins == FE_AE_BINS ? 8 : 6;
    int b;

    for (b = 0; b < SCENE_BARS; b++) {
        hist[(int)scene_level(b, 0, s->level) >> shift] += n;
        hist[(int)scene_level(b, 1, s->level) >> shift] += 2 * n;
        hist[(int)scene_level(b, 2, s->level) >> shift] += n;
    }
    for (b = 0; b < nbins; b++) {
        hist[b] = cpu_to_le32(hist[b]);
    }
    stt_write(addr, hist, nbins * 4);
}

/* AE global: pixels total / counted, then R, Gr, Gb, B means */
static void scene_ae_global(HisiViState *s, uint32_t *regs)
{
    int c;

    regs[0] = MIN(scene_samples(s) * 4, 0x7ffffff);
    regs[1] = MIN(scene_samples(s) * 4, 0x7fffffff);
    for (c = 0; c < 4; c++) {
        regs[2 + c] = scene_mean(c == 0 ? 0 : c == 3 ? 2 : 1, 0, 1, s->level);
    }
}

/* the front end's statistics of the frame captured now */
static void vi_fe_stats(HisiViState *s)
{
    uint32_t *p = s->cap;

    if (p[FE_STT_AE_HIST / 4]) {
        scene_ae_hist(s, p[FE_STT_AE_HIST / 4], FE_AE_BINS);
    }
    scene_ae_zones(s, p[FE_AE_ZONE / 4], p[FE_STT_AE_RGR / 4],
                   p[FE_STT_AE_GBB / 4]);
    scene_ae_global(s, p + FE_AE_TOTAL / 4);
}

/* the backend's statistics of the frame the pipe finishes now */
static void vi_be_stats(HisiViState *s)
{
    uint32_t *p = s->proc;
    int awx = MAX(MIN(p[BE_AWB_ZONE / 4] & 0x3f, 32), 1);
    int awy = MAX(MIN((p[BE_AWB_ZONE / 4] >> 8) & 0x3f, 32), 1);
    int c, x, y;

    if (p[STT_AE_HIST / 4]) {
        scene_ae_hist(s, p[STT_AE_HIST / 4], AE_BINS);
    }
    scene_ae_zones(s, p[BE_AE_ZONE / 4], p[STT_AE_RGR / 4],
                   p[STT_AE_GBB / 4]);
    scene_ae_global(s, p + BE_AE_TOTAL / 4);
    if (p[STT_AWB / 4]) {
        uint32_t awb[AWB_ZONES_MAX * 2];
        const uint32_t n = MIN(scene_samples(s) / (awx * awy), 0xffff);

        for (x = 0; x < awx; x++) {
            uint32_t m[3];

            for (c = 0; c < 3; c++) {
                m[c] = scene_mean(c, (double)x / awx, (double)(x + 1) / awx,
                                  s->level);
            }
            for (y = 0; y < awy; y++) {
                awb[(y * awx + x) * 2] = cpu_to_le32(m[1] << 16 | m[0]);
                awb[(y * awx + x) * 2 + 1] = cpu_to_le32(n << 16 | m[2]);
            }
        }
        stt_write(p[STT_AWB / 4], awb, awx * awy * 8);
    }
    for (c = 0; c < 3; c++) {
        p[BE_AWB_AVG / 4 + c] = scene_mean(c, 0, 1, s->level);
    }
    p[BE_AWB_AVG / 4 + 3] = MIN(scene_samples(s), 0xffff);

    if (s->stats_trace && s->stat_frames % MAX(s->fps, 1) == 0) {
        fprintf(stderr, "isp stats: %u rows x%.2f gain, G mean %u, "
                "WB gains R %#x B %#x\n", s->frame_expo.lines,
                s->frame_expo.gain / 1024.0, p[BE_AWB_AVG / 4 + 1],
                p[0x21210 / 4] >> 16, p[0x21214 / 4] >> 16);
    }
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
    vi_be_stats(s);
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
    DEFINE_PROP_UINT32("light", HisiViState, light, 600),
    DEFINE_PROP_UINT32("light-r", HisiViState, light_r, 550),
    DEFINE_PROP_UINT32("light-b", HisiViState, light_b, 700),
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
