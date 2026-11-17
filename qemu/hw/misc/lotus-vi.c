/*
 * XMedia "lotus" SPC030 VI (Goke GK7206 / xmorca): VICAP and VIPROC, as
 * the stock Xiongmai firmware's xm_vi.ko drives them in its camera mode
 * (VICAP -> VIPROC offline, VIPROC -> VPSS online, one pipe, MIPI).
 *
 * The sensor's frames never exist here: the model is the frame clock and
 * the interrupt / list handshakes that make the driver's state machines
 * run and its buffers flow.  What the pictures look like is lotus-vpss's
 * business (it paints its output channels).
 *
 * VICAP (0x11000000, SPI 51 "vi_cap0"; xm_vi ioremaps 0xa000).  Register
 * semantics from xm_vi.ko hal_vi_vicap_*, the SDK's open RISC-V MCU copy
 * of the same HAL (mcu/media/drivers/hal_vi.c) and a GK7206V11A camera
 * streaming 2560x1440@25 (devmem):
 *   0x0000  TOP_EN    bit d: port (PT) d, bit 5 + f: front end (FE) f
 *   0x0004            FE -> PT binding, 3 bits per FE
 *   0x0010  TOP_INT   bit 5 + f: FE f has an unmasked interrupt (read only)
 *   0x0014  TOP_MASK  1 = masked (the driver leaves 0x1c7 / 0x1fdf)
 *   0x0080            switch error status (read by the ISR, kept 0)
 *   0x1000 * (d + 1)  PT d: +0x00 bit 0 timing enable, +0x0c detected
 *                     size (height << 16 | width; the App polls it)
 *   0x6000 + 0x1000 * f  FE f:
 *     +0x000  bit 0 online (to VIPROC); the camera runs offline (2)
 *     +0x004  input size, +0x02c output size, +0x028 bit width
 *     +0x01c  early-interrupt line / location
 *     +0x030  raw frame write address (the driver swaps it every frame)
 *     +0x03c  INT_MASK  1 = masked (0x3e: start, early, end enabled)
 *     +0x040  INT       write 1 to clear: bit 0 early (the driver arms the
 *                       next frame's buffers), 6 frame start, 7 frame end
 *                       (the frame is complete in DDR: a VIPROC task)
 *     +0x048  "reg newer" (shadow update), +0x09c detected size
 *   Each frame raises start, early (at 4/5 of it, where the driver puts
 *   the early line) and end: the 3 interrupts a frame /proc/umap/vi
 *   counts in int_cnt.
 *
 * VIPROC (0x11200000, SPI 48 "vi_proc0"; ioremapped 0x2610): the ISP back
 * end.  The driver builds each task as a register image (a "node") in
 * DDR and starts it in list mode; the hardware loads the image and runs:
 *   0x0014  CTRL      bit 0 ctl_up_finish, 4 filter_up_finish (start;
 *                     they read back 0 when the task is done), 8 list mode
 *   0x002c  INT_MASK  1 = masked (0x18 by the driver)
 *   0x0048  INT       write 1 to clear: 0 early, 3 frame start, 4 frame
 *                     end, 5 list (task) done, others errors / timeouts
 *   0x00fc            reference interrupt status (kept 0)
 *   0x0120..0x013c    list control: stage-update enables, node id (0x128),
 *                     ctl / common / cnfg base addresses (node image
 *                     + 0x120, + 0x160, ISP BE config), ref addresses
 *   0x0140            the node id being worked on
 *   0x01e8            early mask (from the node image)
 * In the camera's mode xm_vi waits for both the task's list-done
 * interrupt and VPSS's frame-done (the VIPROC -> VPSS online rendezvous,
 * hal_vi_viproc_vpss_deal_vi_finish_int / _deal_vpss_finish_int), then
 * frees the raw frame and submits the next.  A task here takes about
 * 24 ms (the real one about 29 ms): start, early, done; VPSS is told at
 * each step ("vpss-start", "vpss-early", "vpss-done" GPIOs).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "system/dma.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/hisi-scene.h"

#define TYPE_LOTUS_VI "lotus-vi"
OBJECT_DECLARE_SIMPLE_TYPE(LotusViState, LOTUS_VI)

#define CAP_SIZE            0x200000
#define PROC_SIZE           0x40000

/* VICAP */
#define CAP_TOP_EN          0x0000
#define CAP_TOP_INT         0x0010
#define CAP_TOP_INT_MASK    0x0014
#define CAP_PT_BASE         0x1000
#define CAP_PT_END          0x6000
#define PT_SIZE_DET         0x00c
#define CAP_FE(f)           (0x6000 + 0x1000 * (f))
#define CAP_FE_END          0xa000
#define FE_IN_SIZE          0x004
#define FE_INT_MASK         0x03c
#define FE_INT              0x040
#define FE_SIZE_DET         0x09c
#define FE_STT_ADDR         0x064       /* the AE histogram's address */
#define FE_INT_EARLY        (1u << 0)
#define FE_INT_START        (1u << 6)
#define FE_INT_END          (1u << 7)
#define TOP_FE_EN(f)        (1u << (5 + (f)))
#define CAP_WAIT_NS         (1 * 1000 * 1000)   /* re-check a pending event */

/* VIPROC */
#define PROC_CTRL           0x0014
#define PROC_CTRL_START     0x11u
#define PROC_INT_MASK       0x002c
#define PROC_INT            0x0048
#define PROC_NODE_ID        0x0128
#define PROC_COMM_BASE      0x0130      /* the node's common block (+0x160) */
#define PROC_CNFG_BASE      0x0134      /* the ISP BE image, from reg 0x400 */
#define PROC_WORK_NODE_ID   0x0140
#define PROC_INT_EARLY      (1u << 0)
#define PROC_INT_START      (1u << 3)
#define PROC_INT_END        (1u << 4)
#define PROC_INT_LIST       (1u << 5)

/*
 * ISP statistics.  The front end (VICAP FE f) writes a 256-bin AE
 * histogram, one u32 a bin, to the address at FE +0x64 (xm_isp
 * hal_isp_stt_get_fe_ctrl_args sets it a frame ahead and puts 0xffffffff
 * in its first word; hal_isp_stt_format_fe_statistics skips a buffer
 * still holding that).  The back end (a VIPROC task) writes its "3A"
 * block to the address in its node's common register +0x120 (node image
 * +0x280, hal_isp_stt_get_be_ctrl_args; the open MCU copy hal_isp.c
 * isp_init_buffer has the layout):
 *   AWB zones   rows x cols x { count, R, G, B }      (32 x 32 here)
 *   AWB global  { R, G, B }
 *   AE zones    rows x cols x { R, Gr, B, Gb }        (17 x 15)
 *   AE global   { R, Gr, B, Gb }
 *   AE histogram 1024 u32
 * The zone grids are the AAC registers of the task's ISP BE image
 * (VIPROC 0x134, laid out from register 0x400): 0x4c0 AE cols / rows - 1
 * at [20:16] / [27:24], 0x4c4 start x / y, 0x4cc zone w / h - 1 at
 * [8:0] / [24:16], 0x544 AWB cols / rows - 1 at [20:16] / [28:24], 0x548
 * start, 0x54c min / max thresholds, 0x550 zone w / h - 1 at [7:0] /
 * [15:8], 0x560.. the AE zone weights (4 bits a zone, row by row of 17).
 * Means are 16-bit, taken after the ISP digital gain (DGN1 0x464, 256 =
 * 1x) and before white balance (DGN3 0x1464 R | Gr << 16, 0x1468 Gb | B
 * << 16, 256 = 1x), as a GK7206V11A streaming SC465SL shows them.
 */
#define FE_HIST_BINS        256
#define BE_HIST_BINS        1024
#define BE_CNFG_REG0        0x400
#define BE_AAC_AE_ZONES     0x4c0
#define BE_AAC_AE_START     0x4c4
#define BE_AAC_AE_SIZE      0x4cc
#define BE_AAC_AWB_ZONES    0x544
#define BE_AAC_AWB_START    0x548
#define BE_AAC_AWB_THRESH   0x54c
#define BE_AAC_AWB_SIZE     0x550
#define BE_AAC_AE_WEIGHT    0x560
#define BE_DGN1_GAIN        0x464
#define BE_DGN3_GAIN_RGR    0x1464
#define BE_DGN3_GAIN_GBB    0x1468
#define BE_COMM_STT         0x120       /* from the common block */
#define BE_CNFG_SIZE        (0x1470 - BE_CNFG_REG0)
#define ZONES_MAX           (32 * 32)
#define GRID_DIV            8           /* a grid cell: 8 x 8 pixels */

struct LotusViState {
    SysBusDevice parent_obj;

    MemoryRegion cap_mr, proc_mr;
    qemu_irq cap_irq, proc_irq;
    qemu_irq vpss_start, vpss_early, vpss_done;
    uint32_t *cap, *proc;

    /* the capture clock: phase 0 start, 1 early, 2 end */
    QEMUTimer *cap_timer;
    int64_t cap_next;
    int cap_phase;
    uint32_t cap_wait, cap_late;    /* events held back / clock late */
    bool running;

    /* the VIPROC task: phase 0 start, 1 early, 2 done */
    QEMUTimer *proc_timer;
    int proc_phase;

    uint32_t fps;
    bool trace;

    /* what the statistics measure (docs/hisilicon-digital-twin.md) */
    Object *sensor;
    HisiSceneState *scene;
    bool stats_trace;
    ImageSensorExposure expo;   /* the frame being captured */
    double light, tint[3];
    double level;               /* its white, raw, before the ISP's gain */
    int gw, gh;                 /* the grid the frames are measured on */
    uint16_t *raw;              /* the frame being captured, R G B a cell */
    uint16_t *done;             /* the last one captured: VIPROC's next */
    uint16_t *job;              /* the one the task in progress processes */
    double job_level;
    ImageSensorExposure job_expo;
    uint32_t fe_stt;            /* latched at the frame start */
    uint32_t be_stt, be_cnfg;   /* latched at the task start */
    uint32_t frames, be_frames;
    int64_t trace_next;
    /* what the last task's statistics said, for the trace */
    uint32_t last_g, last_wg, last_wb[2], last_luma;
    uint16_t gamma[4096];       /* 1 / 2.2, 12 bits */
};

static void lv_trace(LotusViState *s, const char *blk, char rw,
                     hwaddr off, uint64_t val)
{
    if (s->trace) {
        qemu_log("%" PRId64 " lotus-vi %s %c %05" HWADDR_PRIx " %08" PRIx64
                 "\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), blk, rw, off,
                 val);
    }
}

static int64_t lv_frame_ns(LotusViState *s)
{
    return NANOSECONDS_PER_SECOND / (s->fps ? s->fps : 25);
}

static void lv_cap_update(LotusViState *s)
{
    uint32_t top = 0;

    for (int f = 0; f < 4; f++) {
        uint32_t fe = CAP_FE(f) >> 2;
        if (s->cap[fe + (FE_INT >> 2)] & ~s->cap[fe + (FE_INT_MASK >> 2)] &
            0xffff) {
            top |= TOP_FE_EN(f);
        }
    }
    s->cap[CAP_TOP_INT >> 2] = top;
    qemu_set_irq(s->cap_irq, !!(top & ~s->cap[CAP_TOP_INT_MASK >> 2]));
}

static void lv_proc_update(LotusViState *s)
{
    qemu_set_irq(s->proc_irq, !!(s->proc[PROC_INT >> 2] &
                                 ~s->proc[PROC_INT_MASK >> 2]));
}

/* --- statistics ---------------------------------------------------------- */

static void lv_dma_write(uint32_t addr, const void *buf, size_t len)
{
    dma_memory_write(&address_space_memory, addr, buf, len,
                     MEMTXATTRS_UNSPECIFIED);
}

static uint32_t lv_ldl(uint32_t addr)
{
    uint32_t v = 0;

    dma_memory_read(&address_space_memory, addr, &v, 4,
                    MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* a usable DDR address the driver gave the hardware */
static bool lv_addr_ok(uint32_t a)
{
    return a && a != 0xffffffff && !(a & 3);
}

/*
 * A frame starts: the sensor takes what was written for it (image-sensor
 * frame_start), so this frame's exposure is what it reports now.  The
 * frame is measured on a grid of an eighth of its window each way: the
 * scene's reflectance times the light, the exposure and the gain, as the
 * sensor's black-level-corrected 16-bit raw (clipped at full well).
 */
static void lv_frame_start(LotusViState *s)
{
    ImageSensorExposure e = { 0 };
    double tint[3] = { 0.55, 1.0, 0.7 }, light = 600;
    const float *refl = NULL;
    int w, h, i, c;

    if (s->scene) {
        hisi_scene_frame_begin(s->scene);
        light = hisi_scene_light(s->scene, tint);
    }
    if (s->sensor && image_sensor_exposure(s->sensor, &e)) {
        image_sensor_frame_start(s->sensor);
        image_sensor_exposure(s->sensor, &e);
        s->level = (double)e.lines * e.gain / 1024 * light;
    } else {
        s->level = 65535.0 / 4;
    }
    s->expo = e;
    s->light = light;
    memcpy(s->tint, tint, sizeof(s->tint));

    /* the window: the sensor's, else what the front end was told */
    w = e.width;
    h = e.height;
    if (!w || !h) {
        uint32_t sz = s->cap[(CAP_FE(0) + FE_IN_SIZE) >> 2];
        w = sz & 0xffff;
        h = sz >> 16;
    }
    w = MAX(MIN(w ? w : 2560, 8192) / GRID_DIV, 1);
    h = MAX(MIN(h ? h : 1440, 8192) / GRID_DIV, 1);
    if (w != s->gw || h != s->gh) {
        size_t n = (size_t)w * h * 3;

        s->raw = g_renew(uint16_t, s->raw, n);
        s->done = g_renew(uint16_t, s->done, n);
        s->job = g_renew(uint16_t, s->job, n);
        memset(s->done, 0, n * sizeof(uint16_t));
        memset(s->job, 0, n * sizeof(uint16_t));
        s->gw = w;
        s->gh = h;
    }
    if (s->scene) {
        refl = hisi_scene_grid(s->scene, w, h);
    }
    for (i = 0; i < w * h; i++) {
        for (c = 0; c < 3; c++) {
            const double r = refl ? refl[i * 3 + c] : 0.18;

            s->raw[i * 3 + c] = MIN(MAX(r * tint[c] * s->level, 0), 65535.0);
        }
    }
    s->fe_stt = s->cap[(CAP_FE(0) + FE_STT_ADDR) >> 2];
}

/* The frame ends: the front end's AE histogram (G, a sample a 2 x 2) */
static void lv_frame_end(LotusViState *s)
{
    uint32_t hist[FE_HIST_BINS] = { 0 };
    int i;

    if (!s->raw) {
        return;
    }
    if (lv_addr_ok(s->fe_stt)) {
        for (i = 0; i < s->gw * s->gh; i++) {
            hist[s->raw[i * 3 + 1] >> 8] += GRID_DIV * GRID_DIV / 4;
        }
        for (i = 0; i < FE_HIST_BINS; i++) {
            hist[i] = cpu_to_le32(hist[i]);
        }
        lv_dma_write(s->fe_stt, hist, sizeof(hist));
    }
    /* the raw frame is in DDR: VIPROC's next task takes it */
    memcpy(s->done, s->raw, (size_t)s->gw * s->gh * 3 * sizeof(uint16_t));
    s->frames++;
}

typedef struct LvZones {
    int nx, ny;                 /* zones */
    int x0, y0, zw, zh;         /* first zone's corner and a zone's size */
} LvZones;

static void lv_zones(uint32_t num, uint32_t start, uint32_t size, bool awb,
                     int w, int h, LvZones *z)
{
    z->nx = ((num >> 16) & 0x1f) + 1;
    z->ny = ((num >> 24) & (awb ? 0x1f : 0xf)) + 1;
    z->x0 = start & 0xfff;
    z->y0 = (start >> 16) & 0xfff;
    z->zw = (awb ? (size & 0xff) : (size & 0x1ff)) + 1;
    z->zh = (awb ? ((size >> 8) & 0xff) : ((size >> 16) & 0x1ff)) + 1;
    if (!num || z->zw * z->nx > w + 64 || z->zh * z->ny > h + 64) {
        /* not programmed: the MCU driver's own grid, centred */
        z->nx = awb ? 32 : 17;
        z->ny = awb ? 32 : 15;
        z->zw = w / z->nx & ~1;
        z->zh = h / z->ny & ~1;
        z->x0 = (w - z->zw * z->nx) / 4 * 2;
        z->y0 = (h - z->zh * z->ny) / 4 * 2;
    }
}

/* the zone of grid column / row g (its centre pixel), -1 if outside */
static int lv_zone_of(int g, int start, int size, int n)
{
    const int p = g * GRID_DIV + GRID_DIV / 2 - start;
    const int z = p >= 0 ? p / MAX(size, 1) : -1;

    return z < n ? z : -1;
}

/*
 * Without a TNR reference (VIPROC's reference write and read addresses
 * 0x138 / 0x13c both the ISP driver's small "luma" buffer and its common
 * register bit 7 set, hal_isp_alg_get_be_ctrl_args), the task leaves the
 * picture's luma sum in the buffer's first word:
 * hal_isp_alg_get_luma_average reads it and the BE statistics carry it
 * divided by (width / 128) x (height / 128) x 8.  Modelled as the mean
 * 12-bit gamma-encoded luma of the white-balanced frame times that.
 */
#define PROC_REF_WADDR      0x0138
#define PROC_REF_RADDR      0x013c

static void lv_be_luma(LotusViState *s, double dg)
{
    const uint32_t a = s->proc[PROC_REF_WADDR >> 2];
    const uint32_t comm = s->proc[PROC_COMM_BASE >> 2];
    double k[3], sum = 0;
    uint32_t sz, v;
    int i, c;

    if (!lv_addr_ok(a) || a != s->proc[PROC_REF_RADDR >> 2] ||
        !lv_addr_ok(comm) || !(lv_ldl(comm) & 0x80) || !s->job) {
        return;
    }
    k[0] = dg * (s->last_wb[0] ? s->last_wb[0] : 256) / 256.0 / 65535.0;
    k[1] = dg / 65535.0;
    k[2] = dg * (s->last_wb[1] ? s->last_wb[1] : 256) / 256.0 / 65535.0;
    if (!s->gamma[4095]) {
        for (i = 0; i < 4096; i++) {
            s->gamma[i] = 4095 * pow(i / 4095.0, 1 / 2.2) + 0.5;
        }
    }
    for (i = 0; i < s->gw * s->gh; i++) {
        double e[3];

        for (c = 0; c < 3; c++) {
            e[c] = s->gamma[(int)(MIN(s->job[i * 3 + c] * k[c], 1.0) * 4095)];
        }
        sum += (0.299 * e[0] + 0.587 * e[1] + 0.114 * e[2]) / 4095;
    }
    sz = s->cap[(CAP_FE(0) + FE_IN_SIZE) >> 2];
    s->last_luma = 4095 * sum / MAX(s->gw * s->gh, 1);
    v = s->last_luma * MAX((sz & 0xffff) >> 7, 1) * MAX((sz >> 16) >> 7, 1) * 8;
    v = cpu_to_le32(v);
    lv_dma_write(a, &v, 4);
}

/*
 * The task is done: its 3A block, from the raw frame it processed times
 * the ISP digital gain in its register image.
 */
static void lv_be_stats(LotusViState *s)
{
    uint32_t cnfg[BE_CNFG_SIZE / 4];
    const int w = s->gw * GRID_DIV, h = s->gh * GRID_DIV;
    LvZones ae, awb;
    double dg = 1.0, awb_sum[ZONES_MAX][3] = { { 0 } };
    double ae_sum[ZONES_MAX][3] = { { 0 } }, glob[3] = { 0 };
    uint32_t awb_n[ZONES_MAX] = { 0 }, ae_n[ZONES_MAX] = { 0 }, glob_n = 0;
    uint32_t awb_min, awb_max, hist[BE_HIST_BINS] = { 0 };
    uint8_t wgt[256];
    uint32_t *buf;
    size_t len, o;
    int x, y, i;

#define CNFG(r) le32_to_cpu(cnfg[((r) - BE_CNFG_REG0) / 4])
    memset(cnfg, 0, sizeof(cnfg));
    if (lv_addr_ok(s->be_cnfg)) {
        dma_memory_read(&address_space_memory, s->be_cnfg, cnfg,
                        sizeof(cnfg), MEMTXATTRS_UNSPECIFIED);
    }
    if (CNFG(BE_DGN1_GAIN) & 0xffff) {
        dg = (CNFG(BE_DGN1_GAIN) & 0xffff) / 256.0;
    }
    lv_zones(CNFG(BE_AAC_AE_ZONES), CNFG(BE_AAC_AE_START),
             CNFG(BE_AAC_AE_SIZE), false, w, h, &ae);
    lv_zones(CNFG(BE_AAC_AWB_ZONES), CNFG(BE_AAC_AWB_START),
             CNFG(BE_AAC_AWB_SIZE), true, w, h, &awb);
    awb_min = CNFG(BE_AAC_AWB_THRESH) & 0xffff;
    awb_max = CNFG(BE_AAC_AWB_THRESH) >> 16;
    if (!awb_max) {
        awb_max = 0xffff;
    }
    {
        uint32_t any = 0;

        for (i = 0; i < 256; i++) {
            wgt[i] = (CNFG(BE_AAC_AE_WEIGHT + i / 8 * 4) >> (i % 8 * 4)) & 0xf;
            any |= wgt[i];
        }
        if (!any) {
            memset(wgt, 1, sizeof(wgt));     /* not programmed */
        }
    }
    s->last_wb[0] = CNFG(BE_DGN3_GAIN_RGR) & 0xffff;
    s->last_wb[1] = CNFG(BE_DGN3_GAIN_GBB) >> 16;
#undef CNFG

    for (y = 0; y < s->gh; y++) {
        const int aey = lv_zone_of(y, ae.y0, ae.zh, ae.ny);
        const int awby = lv_zone_of(y, awb.y0, awb.zh, awb.ny);

        for (x = 0; x < s->gw; x++) {
            const uint16_t *p = s->job + ((size_t)y * s->gw + x) * 3;
            const int aex = lv_zone_of(x, ae.x0, ae.zw, ae.nx);
            const int awbx = lv_zone_of(x, awb.x0, awb.zw, awb.nx);
            double v[3];
            int c;

            for (c = 0; c < 3; c++) {
                v[c] = MIN(p[c] * dg, 65535.0);
            }
            if (aex >= 0 && aey >= 0) {
                const int z = aey * ae.nx + aex;

                for (c = 0; c < 3; c++) {
                    ae_sum[z][c] += v[c];
                    glob[c] += v[c];
                }
                ae_n[z]++;
                glob_n++;
                hist[(uint32_t)v[1] >> 6] +=
                    GRID_DIV * GRID_DIV / 4 * (z < 256 ? wgt[z] : 1);
            }
            if (awbx >= 0 && awby >= 0 && v[1] >= awb_min &&
                v[1] <= awb_max) {
                const int z = awby * awb.nx + awbx;

                for (c = 0; c < 3; c++) {
                    awb_sum[z][c] += v[c];
                }
                awb_n[z]++;
            }
        }
    }

    len = (awb.nx * awb.ny * 4 + 3 + ae.nx * ae.ny * 4 + 4 + BE_HIST_BINS) * 4;
    buf = g_malloc0(len);
    o = 0;
    for (i = 0; i < awb.nx * awb.ny; i++) {
        const double n = MAX(awb_n[i], 1);

        buf[o++] = MIN((uint64_t)awb_n[i] * GRID_DIV * GRID_DIV,
                       (uint64_t)awb.zw * awb.zh);
        buf[o++] = awb_sum[i][0] / n;
        buf[o++] = awb_sum[i][1] / n;
        buf[o++] = awb_sum[i][2] / n;
    }
    for (i = 0; i < 3; i++) {
        buf[o++] = glob[i] / MAX(glob_n, 1);
    }
    for (i = 0; i < ae.nx * ae.ny; i++) {
        const double n = MAX(ae_n[i], 1);

        buf[o++] = ae_sum[i][0] / n;
        buf[o++] = ae_sum[i][1] / n;
        buf[o++] = ae_sum[i][2] / n;
        buf[o++] = ae_sum[i][1] / n;
    }
    /*
     * The AE global is the zones' means weighted by the AE weight table
     * and divided by the number of zones (tile_num_all = 32768 / zones):
     * the camera's stock table (rows 0-2 weight 0, the lower half 4) makes
     * it about 2.6 times the plain mean, as on the real one.
     */
    {
        double wsum[3] = { 0 };
        int c;

        for (i = 0; i < ae.nx * ae.ny; i++) {
            const double n = MAX(ae_n[i], 1);
            const int wz = i < 256 ? wgt[i] : 1;

            for (c = 0; c < 3; c++) {
                wsum[c] += ae_sum[i][c] / n * wz;
            }
        }
        for (c = 0; c < 3; c++) {
            wsum[c] = MIN(wsum[c] / MAX(ae.nx * ae.ny, 1), 0xffffff);
        }
        buf[o++] = wsum[0];
        buf[o++] = wsum[1];
        buf[o++] = wsum[2];
        buf[o++] = wsum[1];
        s->last_wg = wsum[1];
    }
    memcpy(buf + o, hist, sizeof(hist));
    o += BE_HIST_BINS;
    s->last_g = glob[1] / MAX(glob_n, 1);
    for (i = 0; i < o; i++) {
        buf[i] = cpu_to_le32(buf[i]);
    }
    if (lv_addr_ok(s->be_stt)) {
        lv_dma_write(s->be_stt, buf, o * 4);
    }
    g_free(buf);
    lv_be_luma(s, dg);
    s->be_frames++;

    if (s->stats_trace) {
        int64_t now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);

        if (now >= s->trace_next) {
            s->trace_next = now + 1000;
            qemu_log("lotus-vi stats: t %" PRId64 " frames %u/%u light %.0f "
                     "lines %u vts %u gain %.3f isp_dg %.3f level %.0f "
                     "G %u/%u luma %u wb %u/%u ae %dx%d awb %dx%d fe 0x%x be 0x%x\n",
                     now, s->frames, s->be_frames, s->light,
                     s->job_expo.lines, s->job_expo.vts,
                     s->job_expo.gain / 1024.0, dg, s->job_level, s->last_g,
                     s->last_wg, s->last_luma, s->last_wb[0], s->last_wb[1], ae.nx, ae.ny, awb.nx,
                     awb.ny, s->fe_stt, s->be_stt);
        }
    }
}

/*
 * A task starts on the last frame captured: where its statistics go and
 * its register image are latched, and the scene learns the frame VPSS
 * is about to paint (its level after the ISP gain, and the white
 * balance the image programs).
 */
static void lv_be_start(LotusViState *s)
{
    uint32_t comm = s->proc[PROC_COMM_BASE >> 2];
    uint32_t gains[2] = { 0, 0 };
    double dg = 1.0;

    s->be_stt = lv_addr_ok(comm) ? lv_ldl(comm + BE_COMM_STT) : 0;
    s->be_cnfg = s->proc[PROC_CNFG_BASE >> 2];
    if (s->job && s->done) {
        memcpy(s->job, s->done, (size_t)s->gw * s->gh * 3 * sizeof(uint16_t));
    }
    s->job_level = s->level;
    s->job_expo = s->expo;
    if (lv_addr_ok(s->be_cnfg)) {
        uint32_t g = lv_ldl(s->be_cnfg + BE_DGN1_GAIN - BE_CNFG_REG0) & 0xffff;

        dg = g ? g / 256.0 : 1.0;
        gains[0] = lv_ldl(s->be_cnfg + BE_DGN3_GAIN_RGR - BE_CNFG_REG0);
        gains[1] = lv_ldl(s->be_cnfg + BE_DGN3_GAIN_GBB - BE_CNFG_REG0);
    }
    if (s->scene) {
        const double g = (gains[0] >> 16) ? (gains[0] >> 16) : 256;

        hisi_scene_set_frame(s->scene, s->level * dg, s->tint,
                             (gains[0] & 0xffff) / g, (gains[1] >> 16) / g);
    }
}

/* FE 0's frame: start, early at 4/5 of it, end at 9/10 */
static void lv_cap_tick(void *opaque)
{
    LotusViState *s = opaque;
    static const uint32_t bits[3] = { FE_INT_START, FE_INT_EARLY,
                                      FE_INT_END };
    static const int tenths[3] = { 8, 1, 1 };   /* to the next phase */

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * The front end's interrupts reach the driver one at a time: on the
     * camera its ISR takes each within microseconds, long before the next
     * (4 ms apart at the shortest).  Here the guest can be tens of ms to
     * seconds late (host load, or a long device emulation holding the
     * CPU); raised on schedule regardless, start / early / end pile up in
     * the one status word, xm_vi sees "start interrupt lost" and restarts
     * the front end, leaving its interrupts masked for good: capture stops
     * (VIPROC, VPSS idle, both raw buffers held).  So an event waits while
     * the previous one is still pending (and unmasked), and a late clock
     * does not catch up in a burst: the frame just comes late.
     */
    if (s->cap[CAP_TOP_INT >> 2] & ~s->cap[CAP_TOP_INT_MASK >> 2] &
        TOP_FE_EN(0)) {
        s->cap_wait++;
        s->cap_next = now + CAP_WAIT_NS;
        timer_mod(s->cap_timer, s->cap_next);
        return;
    }
    if (s->cap_next < now - CAP_WAIT_NS) {
        s->cap_late++;
    }
    s->cap_next = MAX(s->cap_next, now);
    lv_trace(s, "frm", 'T', s->cap_phase, 0);
    if (bits[s->cap_phase] == FE_INT_START) {
        lv_frame_start(s);
    } else if (bits[s->cap_phase] == FE_INT_END) {
        lv_frame_end(s);
    }
    s->cap[(CAP_FE(0) + FE_INT) >> 2] |= bits[s->cap_phase];
    lv_cap_update(s);
    s->cap_next += lv_frame_ns(s) * tenths[s->cap_phase] / 10;
    timer_mod(s->cap_timer, s->cap_next);
    s->cap_phase = (s->cap_phase + 1) % 3;
}

static void lv_proc_tick(void *opaque)
{
    LotusViState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lv_trace(s, "prc", 'T', s->proc_phase, 0);
    switch (s->proc_phase) {
    case 0:     /* the task starts; online VPSS starts with it */
        s->proc[PROC_WORK_NODE_ID >> 2] = s->proc[PROC_NODE_ID >> 2];
        lv_be_start(s);
        s->proc[PROC_INT >> 2] |= PROC_INT_START;
        qemu_irq_pulse(s->vpss_start);
        s->proc_phase = 1;
        timer_mod(s->proc_timer, now + lv_frame_ns(s) * 4 / 10);
        break;
    case 1:     /* the early line */
        s->proc[PROC_INT >> 2] |= PROC_INT_EARLY;
        qemu_irq_pulse(s->vpss_early);
        s->proc_phase = 2;
        timer_mod(s->proc_timer, now + lv_frame_ns(s) / 10);
        break;
    default:    /* the task (list) is done, and VPSS's frame */
        lv_be_stats(s);
        s->proc[PROC_INT >> 2] |= PROC_INT_END | PROC_INT_LIST;
        s->proc[PROC_CTRL >> 2] &= ~PROC_CTRL_START;
        qemu_irq_pulse(s->vpss_done);
        break;
    }
    lv_proc_update(s);
}

static uint64_t cap_read(void *opaque, hwaddr off, unsigned size)
{
    LotusViState *s = opaque;
    uint32_t v = s->cap[off >> 2];

    /* what the port and the front end measure: the configured size */
    if (off >= CAP_PT_BASE && off < CAP_PT_END &&
        (off & 0xfff) == PT_SIZE_DET) {
        v = s->running ? s->cap[(CAP_FE(0) + FE_IN_SIZE) >> 2] : 0;
    } else if (off >= CAP_FE(0) && off < CAP_FE_END &&
               (off & 0xfff) == FE_SIZE_DET) {
        v = s->running ? s->cap[(off - FE_SIZE_DET + FE_IN_SIZE) >> 2] : 0;
    }
    lv_trace(s, "cap", 'R', off, v);
    return v;
}

static void cap_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    LotusViState *s = opaque;

    lv_trace(s, "cap", 'W', off, val);
    if (off >= CAP_FE(0) && off < CAP_FE_END && (off & 0xfff) == FE_INT) {
        s->cap[off >> 2] &= ~val;
        lv_cap_update(s);
        return;
    }
    if (off == CAP_TOP_INT) {
        return;
    }
    s->cap[off >> 2] = val;
    if (off == CAP_TOP_EN) {
        bool run = val & TOP_FE_EN(0);

        if (run && !s->running) {
            s->cap_phase = 0;
            s->cap_next = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          lv_frame_ns(s) / 4;
            timer_mod(s->cap_timer, s->cap_next);
        } else if (!run) {
            timer_del(s->cap_timer);
        }
        s->running = run;
    }
    if (off == CAP_TOP_INT_MASK ||
        (off >= CAP_FE(0) && off < CAP_FE_END &&
         (off & 0xfff) == FE_INT_MASK)) {
        lv_cap_update(s);
    }
}

static uint64_t proc_read(void *opaque, hwaddr off, unsigned size)
{
    LotusViState *s = opaque;
    uint32_t v = s->proc[off >> 2];

    lv_trace(s, "proc", 'R', off, v);
    return v;
}

static void proc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    LotusViState *s = opaque;

    lv_trace(s, "proc", 'W', off, val);
    if (off == PROC_INT) {
        s->proc[off >> 2] &= ~val;
        lv_proc_update(s);
        return;
    }
    if (off == PROC_CTRL && (val & PROC_CTRL_START) &&
        !(s->proc[off >> 2] & PROC_CTRL_START)) {
        s->proc_phase = 0;
        timer_mod(s->proc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  lv_frame_ns(s) / 20);
    }
    s->proc[off >> 2] = val;
    if (off == PROC_INT_MASK) {
        lv_proc_update(s);
    }
}

static const MemoryRegionOps cap_ops = {
    .read = cap_read,
    .write = cap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps proc_ops = {
    .read = proc_read,
    .write = proc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void lotus_vi_reset(DeviceState *dev)
{
    LotusViState *s = LOTUS_VI(dev);

    timer_del(s->cap_timer);
    timer_del(s->proc_timer);
    memset(s->cap, 0, CAP_SIZE);
    memset(s->proc, 0, PROC_SIZE);
    s->running = false;
    s->cap_phase = s->proc_phase = 0;
    s->cap_wait = s->cap_late = 0;
    qemu_irq_lower(s->cap_irq);
    qemu_irq_lower(s->proc_irq);
}

static void lotus_vi_realize(DeviceState *dev, Error **errp)
{
    LotusViState *s = LOTUS_VI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->cap = g_new0(uint32_t, CAP_SIZE / 4);
    s->proc = g_new0(uint32_t, PROC_SIZE / 4);
    memory_region_init_io(&s->cap_mr, OBJECT(s), &cap_ops, s,
                          "lotus-vi.cap", CAP_SIZE);
    memory_region_init_io(&s->proc_mr, OBJECT(s), &proc_ops, s,
                          "lotus-vi.proc", PROC_SIZE);
    sysbus_init_mmio(sbd, &s->cap_mr);
    sysbus_init_mmio(sbd, &s->proc_mr);
    sysbus_init_irq(sbd, &s->cap_irq);
    sysbus_init_irq(sbd, &s->proc_irq);
    qdev_init_gpio_out_named(dev, &s->vpss_start, "vpss-start", 1);
    qdev_init_gpio_out_named(dev, &s->vpss_early, "vpss-early", 1);
    qdev_init_gpio_out_named(dev, &s->vpss_done, "vpss-done", 1);
    s->cap_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lv_cap_tick, s);
    s->proc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lv_proc_tick, s);
}

static void lotus_vi_init(Object *obj)
{
    LotusViState *s = LOTUS_VI(obj);

    /* settable after realize: the machine attaches the sensor later */
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "scene", TYPE_HISI_SCENE,
                             (Object **)&s->scene,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    /* how often the guest's late ISR held the capture back (qom-get) */
    object_property_add_uint32_ptr(obj, "cap-held", &s->cap_wait,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "cap-late", &s->cap_late,
                                   OBJ_PROP_FLAG_READ);
}

static const Property lotus_vi_properties[] = {
    DEFINE_PROP_UINT32("fps", LotusViState, fps, 25),
    DEFINE_PROP_BOOL("trace", LotusViState, trace, false),
    /* a line a second of what the statistics saw (-d guest_errors log) */
    DEFINE_PROP_BOOL("stats-trace", LotusViState, stats_trace, false),
};

static void lotus_vi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = lotus_vi_realize;
    device_class_set_legacy_reset(dc, lotus_vi_reset);
    device_class_set_props(dc, lotus_vi_properties);
    dc->desc = "XMedia lotus SPC030 VI (VICAP + VIPROC)";
}

static const TypeInfo lotus_vi_info = {
    .name = TYPE_LOTUS_VI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LotusViState),
    .instance_init = lotus_vi_init,
    .class_init = lotus_vi_class_init,
};

static void lotus_vi_register(void)
{
    type_register_static(&lotus_vi_info);
}
type_init(lotus_vi_register)
