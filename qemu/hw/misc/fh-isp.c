/*
 * Fullhan FH8626V100 ISP and the VPU inside its register window, at
 * 0xE8400000, as OpenIPC's 4.19 drivers/media/platform/fh8626 drives them
 * (fh8626-isp.c, fh8626-vpu.c; registers in fh8626-isp-regs.h).  There is
 * no SDK or register manual for the part: that driver is the only
 * description, and this model gives it the frames, interrupts and
 * statistics it is written for.  What a register does inside the
 * pipeline -- noise filters, tone curves, colour matrix, NR3D -- is
 * storage here; the pixels are a synthetic scene, not the sensor's.
 *
 * Frames.  They flow while the ISP runs (0x18 CTRL bit 0), its input is
 * open (0x68 bit 0), the CSI receiver is out of reset ("csi-up") and the
 * sensor streams: its frame length (image_sensor_frame_ns(), non-zero
 * once the GC1054 has its MIPI output on) is the frame period, and the
 * frame ends after its active rows.  At each frame's start
 *   - the status gets PICTURE_START (13) and VPU_START (15);
 *   - each VPU channel whose write enable is set in 0x2C (bits 17-19)
 *     latches its Y / CbCr addresses (0x9AC + 12c, 0x9B0 + 12c), so the
 *     driver's writes at that interrupt apply to the next frame, as its
 *     vpu_chan_irq() expects; so does the statistics target, 0x1D8;
 * and at its end
 *   - each latched channel gets the frame, linear NV12 at its output size
 *     (0x62C / 0x634, 0x654 / 0x65C, 0x958) and Y stride (0x9B4 + 12c);
 *   - with 0x24 bit 13 set, the zone statistics go to the latched target:
 *     a 4 KiB bank per Bayer site in physical 2x2 order (0x24[2:1] the
 *     pattern), 16x16 cells of {sum, count, max, min} -- what
 *     isp_grid_snapshot() reads;
 *   - the nine AWB blocks' {R, G, B, count} sums at 0xAE0, before white
 *     balance; with 0x24 bit 16 set, the 24x16 motion bitmap at 0x1F4;
 *   - the status gets FRAME_END (0) and VPU_END (2).
 * 0x04 is write-one-to-clear, 0x08 enables, and IRQ 5 is their AND.
 *
 * The scene is lit as the sensor's exposure says: its level is the
 * reflectance times rows x analog gain x the ISP's digital gain (0x168
 * [12:3], Q6), under a light on the sensor's white locus (below),
 * clipped at 12 bits; the frame applies
 * the white-balance gains (0x224 / 0x228, Q9) and an approximate gamma.
 * The delays are those fh8626-3ad's README measured on a GC1054 board:
 * a sensor write first shows in the statistics published three frame
 * events later, the ISP gain two -- the frame integrated in the period
 * before it starts, the digital gain applied at its start.
 *
 * Not modelled: the VPU's block means, logo and mask overlays, register
 * load lists and scaler filters; NR3D's history DMA; overflow and pipe
 * conflict; the ISP's second statistics bank (0x1E8).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/i2c/image-sensor.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_FH_ISP "fh-isp"
OBJECT_DECLARE_SIMPLE_TYPE(FhIspState, FH_ISP)

#define FH_ISP_SIZE         0x4000

#define ISP_INT_STATUS      0x04
#define ISP_INT_ENABLE      0x08
#define ISP_CTRL            0x18
#define ISP_BAYER_FMT       0x24
#define ISP_VPU_ENABLE      0x2C
#define ISP_INPUT_SIZE      0x30
#define ISP_INPUT_CTRL      0x68
#define ISP_GAIN_0          0x168
#define ISP_STAT_ADDR0      0x1D8
#define ISP_MOTION_MAP      0x1F4
#define ISP_WB_GAIN_01      0x224
#define ISP_WB_GAIN_23      0x228
#define ISP_AWB_STATS       0xAE0

#define INT_FRAME_END       (1u << 0)
#define INT_VPU_END         (1u << 2)
#define INT_PICTURE_START   (1u << 13)
#define INT_VPU_START       (1u << 15)

#define BAYER_STATS_EN      (1u << 13)
#define BAYER_MOTION_STAT   (1u << 16)

#define VPU_CH0_OUT_WIDTH   0x62C
#define VPU_CH0_OUT_HEIGHT  0x634
#define VPU_CH1_OUT_WIDTH   0x654
#define VPU_CH1_OUT_HEIGHT  0x65C
#define VPU_CH2_OUT_SIZE    0x958
#define VPU_CHAN_ADDR_Y(c)  (0x9AC + 0xC * (c))
#define VPU_CHAN_ADDR_C(c)  (0x9B0 + 0xC * (c))
#define VPU_CHAN_STRIDE(c)  (0x9B4 + 0xC * (c))
#define VPU_CHANNELS        3

#define GRID                16              /* zone statistics cells */
#define MOTION_COLS         24
#define MOTION_ROWS         16
#define MOTION_WORDS        (MOTION_COLS * MOTION_ROWS / 32)
#define AWB_BLOCKS          9
#define LEVEL_MAX           4095
#define FRAME_MAX_W         1920
#define FRAME_MAX_H         1080
#define POLL_NS             (10 * SCALE_MS)  /* no sensor frames yet */

/* fh8626-isp.c's isp_bayer_slot: colour (R, G, B, second G) -> site */
static const uint8_t fh_isp_bayer_slot[4][4] = {
    { 0, 1, 3, 2 }, { 1, 0, 2, 3 }, { 3, 2, 0, 1 }, { 2, 3, 1, 0 },
};

typedef struct FhIspChan {
    bool on;
    uint32_t y, c;
    uint32_t w, h, stride;
} FhIspChan;

struct FhIspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    Object *sensor;

    uint32_t regs[FH_ISP_SIZE / 4];
    uint32_t status;
    bool csi_up;
    bool running;
    bool in_frame;              /* between a frame's start and its end */
    int64_t frame_start_ns;
    int64_t frame_ns;
    uint32_t active_ns;
    uint32_t seq;

    /* latched at the frame's start */
    FhIspChan chan[VPU_CHANNELS];
    uint32_t stat_addr;
    bool stats_on;
    uint32_t dgain;             /* Q6 */
    /* the sensor's exposure, sampled at each frame start: the frame
     * integrated before it is the one that ends next */
    uint32_t exp_lines, exp_gain;
    uint32_t prev_lines, prev_gain;

    uint16_t motion_prev[MOTION_COLS * MOTION_ROWS];

    double scale;               /* level per row at unit gain, grey 1.0 */
};

static uint32_t fh_isp_r(FhIspState *s, hwaddr off)
{
    return s->regs[off >> 2];
}

static void fh_isp_update_irq(FhIspState *s)
{
    qemu_set_irq(s->irq, !!(s->status & fh_isp_r(s, ISP_INT_ENABLE)));
}

/*
 * The scene, in reflectance per colour at a point of the frame (0..1 each
 * way): a grey gradient, six colour bars across the middle, a lamp that
 * clips, and a box that moves with the frame count.
 */
static void fh_isp_scene(uint32_t seq, double x, double y, double rgb[3])
{
    static const double bars[6][3] = {
        { 0.40, 0.06, 0.06 }, { 0.06, 0.40, 0.06 }, { 0.06, 0.06, 0.40 },
        { 0.06, 0.40, 0.40 }, { 0.40, 0.06, 0.40 }, { 0.40, 0.40, 0.06 },
    };
    double box = fmod(seq * 0.01, 0.9);
    double g = 0.06 + 0.30 * y;

    rgb[0] = rgb[1] = rgb[2] = g;
    if (y >= 0.40 && y < 0.60 && x >= 0.10 && x < 0.90) {
        memcpy(rgb, bars[(int)((x - 0.10) / (0.80 / 6))], sizeof(bars[0]));
    }
    if (x >= 0.75 && x < 0.85 && y >= 0.08 && y < 0.18) {
        rgb[0] = rgb[1] = rgb[2] = 4.0;         /* the lamp */
    }
    if (x >= box && x < box + 0.08 && y >= 0.70 && y < 0.84) {
        rgb[0] = rgb[1] = rgb[2] = 0.70;        /* the moving box */
    }
}

/*
 * The light, as the GC1054's raw greys see it (R/G, 1, B/G): the second of
 * the white points fh8626-3ad's AWB measured for that sensor (its
 * awb_locus_q12[1]), a warmer light than the one it starts from, so its
 * white balance has somewhere to go.
 */
static const double fh_isp_light[3] = { 3026 / 4096.0, 1.0, 1635 / 4096.0 };

/* 12-bit linear levels, before white balance, of the frame now ending */
static void fh_isp_levels(FhIspState *s, double x, double y,
                          double lvl[3])
{
    double rgb[3];
    double e = s->scale * s->prev_lines * (s->prev_gain / 1024.0) *
               (s->dgain / 64.0);
    int c;

    fh_isp_scene(s->seq, x, y, rgb);
    for (c = 0; c < 3; c++) {
        lvl[c] = MIN(rgb[c] * fh_isp_light[c] * e, LEVEL_MAX);
    }
}

/* white balance gains by colour (R, G, B), Q9, from the site registers */
static void fh_isp_wb(FhIspState *s, double wb[3])
{
    unsigned pattern = (fh_isp_r(s, ISP_BAYER_FMT) >> 1) & 3;
    const uint8_t *slot = fh_isp_bayer_slot[pattern];
    uint16_t site[4] = {
        fh_isp_r(s, ISP_WB_GAIN_01) & 0xFFFF,
        fh_isp_r(s, ISP_WB_GAIN_01) >> 16,
        fh_isp_r(s, ISP_WB_GAIN_23) & 0xFFFF,
        fh_isp_r(s, ISP_WB_GAIN_23) >> 16,
    };
    int c;

    for (c = 0; c < 3; c++) {
        wb[c] = (site[slot[c]] & 0x1FFF) ? (site[slot[c]] & 0x1FFF) / 512.0
                                          : 1.0;
    }
}

static uint8_t fh_isp_gamma(double v)
{
    return v <= 0 ? 0 : v >= LEVEL_MAX ? 255 : sqrt(v / LEVEL_MAX) * 255;
}

/*
 * One channel's frame, NV12, in 8x8 blocks of the scene (it is flat at
 * that scale), the last ones clipped to the frame.  BT.601, limited range,
 * as the driver reports it.
 */
static void fh_isp_render(FhIspState *s, FhIspChan *ch)
{
    uint32_t w = ch->w & ~1u, h = ch->h & ~1u, stride = ch->stride;
    uint32_t nbx = DIV_ROUND_UP(w, 8);
    g_autofree uint8_t *yb = NULL, *cb = NULL;
    g_autofree uint8_t *row = NULL;
    double wb[3];
    uint32_t bx, x, y;

    if (!w || !h || w > FRAME_MAX_W || h > FRAME_MAX_H || stride < w) {
        qemu_log_mask(LOG_GUEST_ERROR, "fh-isp: channel %ux%u stride %u\n",
                      ch->w, ch->h, stride);
        return;
    }
    fh_isp_wb(s, wb);
    yb = g_malloc0((size_t)stride * h);
    cb = g_malloc0((size_t)stride * h / 2);
    row = g_malloc(nbx * 3);                /* Y, U, V of a block row */
    for (y = 0; y < h; y++) {
        if (!(y % 8)) {
            for (bx = 0; bx < nbx; bx++) {
                double lvl[3], r, g, b;

                fh_isp_levels(s, (bx * 8 + 4.0) / w, (y + 4.0) / h, lvl);
                r = fh_isp_gamma(lvl[0] * wb[0]);
                g = fh_isp_gamma(lvl[1] * wb[1]);
                b = fh_isp_gamma(lvl[2] * wb[2]);
                row[bx * 3] = 16 + (65.481 * r + 128.553 * g +
                                    24.966 * b) / 255;
                row[bx * 3 + 1] = 128 + (-37.797 * r - 74.203 * g +
                                         112.0 * b) / 255;
                row[bx * 3 + 2] = 128 + (112.0 * r - 93.786 * g -
                                         18.214 * b) / 255;
            }
        }
        for (x = 0; x < w; x++) {
            yb[(size_t)y * stride + x] = row[(x / 8) * 3];
        }
        if (!(y & 1)) {
            for (x = 0; x < w; x += 2) {
                uint8_t *p = cb + (size_t)(y / 2) * stride + x;

                p[0] = row[(x / 8) * 3 + 1];
                p[1] = row[(x / 8) * 3 + 2];
            }
        }
    }
    dma_memory_write(&address_space_memory, ch->y, yb, (size_t)stride * h,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, ch->c, cb,
                     (size_t)stride * h / 2, MEMTXATTRS_UNSPECIFIED);
}

/* The zone statistics, the AWB blocks and the motion bitmap. */
static void fh_isp_statistics(FhIspState *s)
{
    uint32_t size = fh_isp_r(s, ISP_INPUT_SIZE);
    uint32_t w = (size & 0xFFFF) + 1, h = (size >> 16) + 1;
    unsigned pattern = (fh_isp_r(s, ISP_BAYER_FMT) >> 1) & 3;
    const uint8_t *slot = fh_isp_bayer_slot[pattern];
    /* colour of each physical site: R, G, B, G */
    static const uint8_t colour_of[4] = { 0, 1, 2, 1 };
    uint32_t site_colour[4];
    uint32_t n, c, i;
    uint64_t awb[AWB_BLOCKS][4] = { { 0 } };
    uint32_t motion[MOTION_WORDS] = { 0 };
    uint8_t bank[4][GRID * GRID * 16];
    uint32_t per_site = (w / GRID) * (h / GRID) / 4;

    for (c = 0; c < 4; c++) {
        site_colour[slot[c]] = colour_of[c];
    }
    for (n = 0; n < GRID * GRID; n++) {
        uint32_t cx = n % GRID, cy = n / GRID;
        double sum[3] = { 0 }, mx[3] = { 0 }, mn[3] = { LEVEL_MAX,
                                                      LEVEL_MAX, LEVEL_MAX };
        unsigned blk = (cy * 3 / GRID) * 3 + cx * 3 / GRID;

        /* 4x4 points of each cell */
        for (i = 0; i < 16; i++) {
            double lvl[3];

            fh_isp_levels(s, (cx + (i % 4 + 0.5) / 4) / GRID,
                          (cy + (i / 4 + 0.5) / 4) / GRID, lvl);
            for (c = 0; c < 3; c++) {
                sum[c] += lvl[c] / 16;
                mx[c] = MAX(mx[c], lvl[c]);
                mn[c] = MIN(mn[c], lvl[c]);
            }
        }
        for (c = 0; c < 4; c++) {
            unsigned col = site_colour[c];
            uint32_t cell[4] = {
                cpu_to_le32((uint32_t)(sum[col] * per_site)),
                cpu_to_le32(per_site),
                cpu_to_le32((uint32_t)mx[col]),
                cpu_to_le32((uint32_t)mn[col]),
            };

            memcpy(&bank[c][n * 16], cell, 16);
        }
        awb[blk][0] += sum[0] * per_site;
        awb[blk][1] += sum[1] * per_site * 2;
        awb[blk][2] += sum[2] * per_site;
        awb[blk][3] += per_site * 4;
    }
    if (s->stats_on && s->stat_addr) {
        for (c = 0; c < 4; c++) {
            dma_memory_write(&address_space_memory, s->stat_addr + c * 0x1000,
                             bank[c], sizeof(bank[c]),
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
    for (i = 0; i < AWB_BLOCKS; i++) {
        for (c = 0; c < 4; c++) {
            s->regs[(ISP_AWB_STATS >> 2) + i * 4 + c] =
                MIN(awb[i][c], UINT32_MAX);
        }
    }
    if (fh_isp_r(s, ISP_BAYER_FMT) & BAYER_MOTION_STAT) {
        for (n = 0; n < MOTION_COLS * MOTION_ROWS; n++) {
            double lvl[3];
            uint16_t y;

            fh_isp_levels(s, (n % MOTION_COLS + 0.5) / MOTION_COLS,
                          (n / MOTION_COLS + 0.5) / MOTION_ROWS, lvl);
            y = (lvl[0] + 2 * lvl[1] + lvl[2]) / 4;
            /* a change of more than 1/16 marks the block */
            if (abs(y - s->motion_prev[n]) * 16 > MAX(s->motion_prev[n], 64)) {
                motion[n / 32] |= 1u << (n % 32);
            }
            s->motion_prev[n] = y;
        }
        for (i = 0; i < MOTION_WORDS; i++) {
            s->regs[(ISP_MOTION_MAP >> 2) + i] = motion[i];
        }
    }
}

static bool fh_isp_should_run(FhIspState *s)
{
    return (fh_isp_r(s, ISP_CTRL) & 1) && (fh_isp_r(s, ISP_INPUT_CTRL) & 1) &&
           s->csi_up;
}

static void fh_isp_frame_start(FhIspState *s, int64_t now)
{
    ImageSensorExposure e = { 0 };
    uint32_t en = fh_isp_r(s, ISP_VPU_ENABLE);
    int c;

    image_sensor_frame_start(s->sensor);
    s->seq++;
    s->prev_lines = s->exp_lines;
    s->prev_gain = s->exp_gain;
    if (image_sensor_exposure(s->sensor, &e)) {
        s->exp_lines = e.lines;
        s->exp_gain = e.gain ? e.gain : 1024;
        if (!s->prev_lines) {
            /* the first frame since the start: exposed as programmed */
            s->prev_lines = s->exp_lines;
            s->prev_gain = s->exp_gain;
        }
        s->active_ns = e.vts ? muldiv64(s->frame_ns, e.height ? e.height
                                                      : e.vts, e.vts)
                             : s->frame_ns * 4 / 5;
    } else {
        s->active_ns = s->frame_ns * 4 / 5;
    }
    s->dgain = (fh_isp_r(s, ISP_GAIN_0) >> 3) & 0x3FF;
    s->stat_addr = fh_isp_r(s, ISP_STAT_ADDR0);
    s->stats_on = fh_isp_r(s, ISP_BAYER_FMT) & BAYER_STATS_EN;
    for (c = 0; c < VPU_CHANNELS; c++) {
        FhIspChan *ch = &s->chan[c];

        ch->on = en & (1u << (17 + c));
        ch->y = fh_isp_r(s, VPU_CHAN_ADDR_Y(c));
        ch->c = fh_isp_r(s, VPU_CHAN_ADDR_C(c));
        ch->stride = fh_isp_r(s, VPU_CHAN_STRIDE(c)) & 0xFFFF;
        if (c == 0) {
            ch->w = fh_isp_r(s, VPU_CH0_OUT_WIDTH) + 1;
            ch->h = fh_isp_r(s, VPU_CH0_OUT_HEIGHT) + 1;
        } else if (c == 1) {
            ch->w = fh_isp_r(s, VPU_CH1_OUT_WIDTH) + 1;
            ch->h = fh_isp_r(s, VPU_CH1_OUT_HEIGHT) + 1;
        } else {
            ch->w = (fh_isp_r(s, VPU_CH2_OUT_SIZE) & 0xFFFF) + 1;
            ch->h = (fh_isp_r(s, VPU_CH2_OUT_SIZE) >> 16) + 1;
        }
    }
    s->in_frame = true;
    s->frame_start_ns = now;
    s->status |= INT_PICTURE_START | INT_VPU_START;
    fh_isp_update_irq(s);
}

static void fh_isp_frame_end(FhIspState *s)
{
    int c;

    for (c = 0; c < VPU_CHANNELS; c++) {
        if (s->chan[c].on) {
            fh_isp_render(s, &s->chan[c]);
        }
    }
    fh_isp_statistics(s);
    s->in_frame = false;
    s->status |= INT_FRAME_END | INT_VPU_END;
    fh_isp_update_irq(s);
}

static void fh_isp_tick(void *opaque)
{
    FhIspState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!s->running) {
        return;
    }
    if (s->in_frame) {
        fh_isp_frame_end(s);
        timer_mod(s->timer, s->frame_start_ns + s->frame_ns);
        return;
    }
    s->frame_ns = image_sensor_frame_ns(s->sensor);
    if (!s->frame_ns) {
        timer_mod(s->timer, now + POLL_NS);     /* the sensor is not on */
        return;
    }
    fh_isp_frame_start(s, now);
    timer_mod(s->timer, now + s->active_ns);
}

static void fh_isp_update_run(FhIspState *s)
{
    bool run = fh_isp_should_run(s);

    if (run == s->running) {
        return;
    }
    s->running = run;
    s->in_frame = false;
    s->exp_lines = 0;
    if (run) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    } else {
        timer_del(s->timer);
    }
}

static uint64_t fh_isp_read(void *opaque, hwaddr off, unsigned size)
{
    FhIspState *s = opaque;

    if (off == ISP_INT_STATUS) {
        return s->status;
    }
    return s->regs[off >> 2];
}

static void fh_isp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    FhIspState *s = opaque;

    if (off == ISP_INT_STATUS) {
        s->status &= ~val;
        fh_isp_update_irq(s);
        return;
    }
    s->regs[off >> 2] = val;
    switch (off) {
    case ISP_INT_ENABLE:
        fh_isp_update_irq(s);
        break;
    case ISP_CTRL:
    case ISP_INPUT_CTRL:
        fh_isp_update_run(s);
        break;
    }
}

static const MemoryRegionOps fh_isp_ops = {
    .read = fh_isp_read,
    .write = fh_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void fh_isp_csi_up(void *opaque, int n, int level)
{
    FhIspState *s = opaque;

    s->csi_up = level;
    fh_isp_update_run(s);
}

static void fh_isp_reset(DeviceState *dev)
{
    FhIspState *s = FH_ISP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chan, 0, sizeof(s->chan));
    memset(s->motion_prev, 0, sizeof(s->motion_prev));
    s->status = 0;
    s->running = false;
    s->in_frame = false;
    s->seq = 0;
    s->exp_lines = s->prev_lines = 0;
    s->exp_gain = s->prev_gain = 1024;
    s->dgain = 64;
    timer_del(s->timer);
    fh_isp_update_irq(s);
}

static void fh_isp_init(Object *obj)
{
    FhIspState *s = FH_ISP(obj);

    memory_region_init_io(&s->iomem, obj, &fh_isp_ops, s, TYPE_FH_ISP,
                          FH_ISP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), fh_isp_csi_up, "csi-up", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fh_isp_tick, s);
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    /*
     * A grey card (1.0) at 891 rows and 1x reads about 2/3 of full
     * scale: a mid-grey scene wants some gain, as indoors.
     */
    s->scale = LEVEL_MAX * 2.0 / 3 / 891;
}

static const VMStateDescription vmstate_fh_isp_chan = {
    .name = "fh-isp-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(on, FhIspChan),
        VMSTATE_UINT32(y, FhIspChan),
        VMSTATE_UINT32(c, FhIspChan),
        VMSTATE_UINT32(w, FhIspChan),
        VMSTATE_UINT32(h, FhIspChan),
        VMSTATE_UINT32(stride, FhIspChan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_fh_isp = {
    .name = TYPE_FH_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, FhIspState, FH_ISP_SIZE / 4),
        VMSTATE_UINT32(status, FhIspState),
        VMSTATE_BOOL(csi_up, FhIspState),
        VMSTATE_BOOL(running, FhIspState),
        VMSTATE_BOOL(in_frame, FhIspState),
        VMSTATE_INT64(frame_start_ns, FhIspState),
        VMSTATE_INT64(frame_ns, FhIspState),
        VMSTATE_UINT32(active_ns, FhIspState),
        VMSTATE_UINT32(seq, FhIspState),
        VMSTATE_STRUCT_ARRAY(chan, FhIspState, VPU_CHANNELS, 1,
                             vmstate_fh_isp_chan, FhIspChan),
        VMSTATE_UINT32(stat_addr, FhIspState),
        VMSTATE_BOOL(stats_on, FhIspState),
        VMSTATE_UINT32(dgain, FhIspState),
        VMSTATE_UINT32(exp_lines, FhIspState),
        VMSTATE_UINT32(exp_gain, FhIspState),
        VMSTATE_UINT32(prev_lines, FhIspState),
        VMSTATE_UINT32(prev_gain, FhIspState),
        VMSTATE_UINT16_ARRAY(motion_prev, FhIspState,
                             MOTION_COLS * MOTION_ROWS),
        VMSTATE_TIMER_PTR(timer, FhIspState),
        VMSTATE_END_OF_LIST()
    },
};

static void fh_isp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fh_isp_reset);
    dc->vmsd = &vmstate_fh_isp;
}

static const TypeInfo fh_isp_info = {
    .name          = TYPE_FH_ISP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FhIspState),
    .instance_init = fh_isp_init,
    .class_init    = fh_isp_class_init,
};

static void fh_isp_register_types(void)
{
    type_register_static(&fh_isp_info);
}

type_init(fh_isp_register_types)
