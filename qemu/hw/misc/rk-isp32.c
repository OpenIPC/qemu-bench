/*
 * Rockchip ISP32 lite ("rockchip,rv1106-rkisp", 0xFFA00000), the RV1106's
 * image signal processor, as far as the vendor kernel's
 * drivers/media/platform/rockchip/isp (develop-5.10: hw.c, rkisp.c,
 * capture_v32.c, regs.h / regs_v3x.h) needs it to deliver frames in
 * online mode (VICAP -> ISP -> memory):
 *
 *   0x0400 ISP_CTRL0     bit 0 ISP enable
 *   0x059c / 0x05a0      ISP_OUT_H_SIZE / ISP_OUT_V_SIZE
 *   0x05bc IMSC  0x05c0 RIS  0x05c4 MIS (RIS & IMSC)  0x05c8 ICR  0x05cc ISR
 *   0x05d0..0x05dc       the 3A IMSC / RIS / MIS / ICR (never raised)
 *   0x1400 MI_WR_CTRL    bit 0 mainpath enable
 *   0x1408 MP_WR_Y_BASE  0x140c Y size  0x141c CB base  0x1420 CB size
 *   0x1474 MI_WR_CTRL_SHD  bits 0 / 16 MP in / out enabled
 *   0x1478 / 0x1488      MP Y / CB base shadows
 *   0x14f8 IMSC  0x14fc RIS  0x1500 MIS  0x1504 ICR  0x1508 ISR
 *   0x15e8 MP_WR_Y_LLENGTH
 *   0x1600 BP_WR_CTRL    bit 0 bypass path enable
 *   0x1604 BP Y base  0x1608 Y size  0x1614 Y llength  0x1624 CB base
 *   0x1628 CB size  0x1634 / 0x1640 BP Y / CB base shadows
 *   0x1800 MI_WR_CTRL2   0x1810 its shadow (bit 20 BP out enabled)
 *
 * While the ISP is enabled a frame clock runs each frame as the
 * hardware's sequence: the configuration goes to the shadow registers,
 * the sensor ("sensor" link, image-sensor.h) is told a frame starts and
 * V_START (bit 6) is raised; half a frame later the frame is "processed"
 * and written to each enabled path's buffers (physical addresses: the
 * RV1106 ISP has no IOMMU), and FRAME_IN (bit 5) and FRAME (bit 1) are
 * raised,
 * with the MI's MP_FRAME (bit 0) and BP_FRAME (bit 25).  The driver's
 * stop checks read the shadows, so a path disabled stops at the next
 * frame start.  MI_WR_CTRL2's self-update bits (4 MP, 6 BP) latch that
 * path's shadows at once: a paused path the driver restarts with a new
 * buffer is enabled, self-updated and disabled again (capture_v32.c
 * update_mi), so the one frame is written; latched only at frame start
 * the path stayed off and the stream stalled for good.  Three interrupt
 * lines: ISP, MI and MIPI (never raised), and "vicap-fs" / "vicap-fe"
 * pulsed at the frame's start and end for VICAP's to-ISP interrupts
 * (rk-vicap.c), from which the 3A library takes its frame sync.  With
 * "source-gated" (a VICAP feeds it) frames run only while the
 * "source-on" input is high (VICAP's to-ISP channel enabled): online, the
 * ISP's frame count must start with VICAP's, else every statistics
 * buffer is far ahead of the frame syncs and the library skips it.  Nor
 * are there frames before the sensor streams (image_sensor_streaming):
 * the CSI-2 receiver's frame-sync count, which the library uses, starts
 * with the sensor's stream, so the ISP's may not start earlier.  The frame lasts what the
 * sensor's registers make it (image_sensor_frame_ns), else 1 / "fps".
 *
 * The picture is a scene lit by the sensor's exposure: a grey gradient,
 * six colour bars, a lamp that clips and a box moving with the frame
 * count (rk-scene.c, shared with the ISPP), in reflectance, times
 * integration lines x gain (a grey card at RK_SCENE_NOMINAL_LINES and 1x is
 * two thirds of the 12-bit range) under a neutral light, in 8x8 blocks.  Gamma (square root) and BT.601 limited
 * range make the NV12, through the colour processing (CPROC, 0x800: CTRL
 * bit 0, CONTRAST and SATURATION Q1.7, BRIGHTNESS signed, HUE signed in
 * 90/128 degree steps) when the IQ turns it on -- majestic's image
 * settings land there.  So the vendor AE, which reads the statistics
 * below and writes the sensor's exposure over I2C, sees its own changes.
 *
 * The 3A statistics the lite ISP's driver reads from registers at FRAME
 * (isp_stats_v32.c, rkisp_stats_send_meas_lite): for each block enabled
 * (CTRL bit 0) the frame end sets MEAS_DONE (CTRL bit 31, write 1 to
 * clear: the driver writes back what it read) and its 3A_RIS bit, with
 * statistics of the frame's raw levels:
 *   0x4400 RAWAE_BIG1  (3A bit 0)  sums at +0x30/+0x40/+0x50, means
 *                      from the port +0x80 (G:12 | B:10 << 12 | R:10 << 22)
 *   0x4500 RAWAE_LITE  (bit 1)     25 means at +0x10
 *   0x4800 RAWHIST_BIG1 (bit 4), 0x4900 RAWHIST_LITE (bit 5): the G
 *                      levels' histograms (256 bins) from the port +0x80
 *   0x5000 RAWAWB      (bit 11)    white-point sums with R/G = B/G = 1
 * The ports return one value a read, from the first again each frame.
 * That is the lite ISP's register path; the RV1106's driver is ISP_V32's,
 * which takes the means and histograms from memory: the ISP writes them
 * (struct isp32_stat, rk-isp32-config.h) to MI_3A_WR_BASE (0x197c,
 * latched at frame start like the other shadows) at the frame end --
 * 225 big-window means at 0x000 / 0x390 / 0x720, 25 lite means at 0xab0,
 * 256-bin histograms at 0xc00 / 0x1000 / 0x1400 / 0x1800.  The AF and
 * AWB parts (0x1c00, 0x2b00) stay as they were.
 *
 * "readback" makes it the RV1126 / RV1109 ISP20 ("rockchip,rv1126-rkisp",
 * develop-4.19 isp/rkisp.c, capture_v20.c, bridge.c), whose SDK pipeline
 * runs it from memory: the CIF captures the raw frame, the 3A library
 * queues it to the raw read DMA, and the driver starts each frame by
 * writing CSI2RX_CTRL0 (0x1c00) with bit 0 set (rkisp_trigger_read_back);
 * bits 13:12 + 1 are the passes over it (2 with dehaze or tone mapping
 * on).  Each pass is V_START, then FRAME_IN and FRAME with the 3A
 * statistics, at the same registers as above (the v2x driver reads them
 * all from registers, at every FRAME, and clears the done flags of all
 * but the last pass); the last also ends the MI paths -- MP_FRAME, or
 * MPFBC_FRAME (MI bit 31) with the FBC encoder on (0x18c0 bit 0).
 * There is no frame clock: the driver's next trigger follows the frame
 * ends.  The MP buffers are the bridge's to the ISPP, which draws its own
 * pictures (rk-ispp.c), so nothing is written to them.
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "hw/i2c/image-sensor.h"
#include "hw/misc/rk-scene.h"

#define TYPE_RK_ISP32 "rk-isp32"
OBJECT_DECLARE_SIMPLE_TYPE(RkIsp32State, RK_ISP32)

#define REGS_SIZE       0x8000

#define R_ISP_CTRL0     0x0400
#define R_ISP_OUT_H     0x059c
#define R_ISP_OUT_V     0x05a0
#define R_ISP_IMSC      0x05bc
#define R_ISP_RIS       0x05c0
#define R_ISP_MIS       0x05c4
#define R_ISP_ICR       0x05c8
#define R_ISP_ISR       0x05cc
#define R_3A_IMSC       0x05d0
#define R_3A_RIS        0x05d4
#define R_3A_MIS        0x05d8
#define R_3A_ICR        0x05dc

#define R_MI_CTRL       0x1400
#define R_MP_Y_BASE     0x1408
#define R_MP_Y_SIZE     0x140c
#define R_MP_CB_BASE    0x141c
#define R_MP_CB_SIZE    0x1420
#define R_MI_CTRL_SHD   0x1474
#define R_MP_Y_SHD      0x1478
#define R_MP_CB_SHD     0x1488
#define R_MI_IMSC       0x14f8
#define R_MI_RIS        0x14fc
#define R_MI_MIS        0x1500
#define R_MI_ICR        0x1504
#define R_MI_ISR        0x1508
#define R_MP_LLENGTH    0x15e8
#define R_BP_CTRL       0x1600
#define R_BP_Y_BASE     0x1604
#define R_BP_Y_SIZE     0x1608
#define R_BP_LLENGTH    0x1614
#define R_BP_CB_BASE    0x1624
#define R_BP_CB_SIZE    0x1628
#define R_BP_Y_SHD      0x1634
#define R_BP_CB_SHD     0x1640
#define R_MPFBC_CTRL    0x18c0      /* ISP20: the MP's FBC encoder */
#define R_CSI2RX_CTRL0  0x1c00      /* ISP20: read-back trigger */
#define R_MI_CTRL2      0x1800
#define R_MI_CTRL2_SHD  0x1810

#define ISP_ENABLE      (1u << 0)
#define ISP_FRAME       (1u << 1)
#define ISP_FRAME_IN    (1u << 5)
#define ISP_V_START     (1u << 6)
#define MI_MP_FRAME     (1u << 0)
#define MI_BP_FRAME     (1u << 25)
#define MI_MPFBC_FRAME  (1u << 31)
#define READBACK_NS     4000000     /* a pass over the frame in memory */
#define MI_MP_ENABLE    (1u << 0)
#define MI_SHD_MP_IN    (1u << 0)
#define MI_SHD_MP_OUT   (1u << 16)
#define BP_ENABLE       (1u << 0)
#define BP_EN_OUT_SHD   (1u << 20)
#define MP_SELF_UPD     (1u << 4)
#define BP_SELF_UPD     (1u << 6)

#define R_CPROC_CTRL    0x0800      /* colour processing: bit 0 enable */
#define R_CPROC_CONTRAST 0x0804     /* Q1.7, 0x80 = 1 */
#define R_CPROC_BRIGHT  0x0808      /* signed, added to Y */
#define R_CPROC_SAT     0x080c      /* Q1.7 */
#define R_CPROC_HUE     0x0810      /* signed, 90 / 128 degree steps */
#define R_AE_BIG        0x4400
#define R_AE_LITE       0x4500
#define R_AE_BIG2       0x4600
#define R_AE_BIG3       0x4700
#define R_HIST_BIG      0x4800
#define R_HIST_BIG2     0x4a00
#define R_HIST_BIG3     0x4b00
#define R_HIST_LITE     0x4900
#define R_AWB           0x5000
#define PORT_OFF        0x80        /* AE / histogram read ports */
#define R_AWB_RAM       0x5700
#define R_MI_3A_BASE    0x197c      /* MI_BASE + 0x57c: struct isp32_stat in DDR */
#define MEAS_DONE       (1u << 31)
#define HIST_BINS       256
#define AWB_WP          4096        /* white points per illuminant */
#define LEVEL_MAX       RK_SCENE_LEVEL_MAX
#define BLK             8           /* the scene is flat at this scale */
#define AE_LITE_N       5           /* zones a side */
#define AE_BIG_N        15

struct RkIsp32State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq_isp, irq_mi, irq_mipi;
    qemu_irq vicap_fs, vicap_fe;    /* the frame's start / end, to VICAP */
    bool source_gated;      /* frames only while the source sends them */
    bool source_on;
    QEMUTimer *timer;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t fps;
    uint32_t frame;         /* frames processed: moves the picture */
    uint32_t port_idx[4];   /* AE big, hist big, hist lite, AWB RAM reads */
    bool in_frame;          /* between V_START and the frame end */
    bool readback;          /* ISP20: frames run by read-back triggers */
    uint32_t passes;        /* read-back passes left in this frame */
    Object *sensor;

    /* the frame now ending, per 8x8 block: NV12 and raw G level */
    uint32_t bw, bh;
    uint8_t *blk_yuv;       /* Y, Cb, Cr */
    uint16_t *blk_lvl;      /* R, G, B, 12-bit */
    uint32_t ae_big[AE_BIG_N * AE_BIG_N];
    uint32_t ae_lite[AE_LITE_N * AE_LITE_N];
    uint32_t stats_base;    /* MI_3A_WR_BASE latched at frame start */
    uint32_t hist[HIST_BINS];
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_isp32_update(RkIsp32State *s)
{
    REG(s, R_ISP_MIS) = REG(s, R_ISP_RIS) & REG(s, R_ISP_IMSC);
    REG(s, R_3A_MIS) = REG(s, R_3A_RIS) & REG(s, R_3A_IMSC);
    REG(s, R_MI_MIS) = REG(s, R_MI_RIS) & REG(s, R_MI_IMSC);
    qemu_set_irq(s->irq_isp, REG(s, R_ISP_MIS) || REG(s, R_3A_MIS));
    qemu_set_irq(s->irq_mi, !!REG(s, R_MI_MIS));
}

static int64_t rk_isp32_half_frame(RkIsp32State *s)
{
    int64_t ns = s->sensor ? image_sensor_frame_ns(s->sensor) : 0;

    if (ns <= 0 || ns > NANOSECONDS_PER_SECOND) {
        ns = NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
    }
    return ns / 2;
}

/*
 * Colour processing (CPROC), as the ISP applies it after the statistics:
 * Y scaled by the contrast and offset by the brightness, the chroma
 * rotated by the hue and scaled by the saturation.
 */
static void rk_isp32_cproc(RkIsp32State *s, uint8_t yuv[3])
{
    double con = (REG(s, R_CPROC_CONTRAST) & 0xff) / 128.0;
    double sat = (REG(s, R_CPROC_SAT) & 0xff) / 128.0;
    double bri = (int8_t)REG(s, R_CPROC_BRIGHT);
    double hue = (int8_t)REG(s, R_CPROC_HUE) * (M_PI / 2) / 128;
    double u = yuv[1] - 128.0, v = yuv[2] - 128.0;
    double y = yuv[0] * con + bri;
    double u2 = (u * cos(hue) + v * sin(hue)) * sat + 128;
    double v2 = (v * cos(hue) - u * sin(hue)) * sat + 128;

    yuv[0] = y < 0 ? 0 : y > 255 ? 255 : lrint(y);
    yuv[1] = u2 < 0 ? 0 : u2 > 255 ? 255 : lrint(u2);
    yuv[2] = v2 < 0 ? 0 : v2 > 255 ? 255 : lrint(v2);
}

/*
 * The frame now ending: each 8x8 block's raw levels at the sensor's
 * exposure, its NV12, and the statistics of those levels.
 */
static void rk_isp32_render(RkIsp32State *s)
{
    uint32_t w = REG(s, R_ISP_OUT_H) & 0x3fff, h = REG(s, R_ISP_OUT_V) & 0x3fff;
    uint32_t bw = DIV_ROUND_UP(MAX(w, 1), BLK), bh = DIV_ROUND_UP(MAX(h, 1), BLK);
    uint64_t lite[AE_LITE_N * AE_LITE_N][4] = { { 0 } };
    uint64_t big[AE_BIG_N * AE_BIG_N][4] = { { 0 } };
    double k;
    uint32_t bx, by, i;
    int c;

    k = rk_scene_scale(s->sensor);
    if (bw != s->bw || bh != s->bh) {
        g_free(s->blk_yuv);
        g_free(s->blk_lvl);
        s->blk_yuv = g_new(uint8_t, bw * bh * 3);
        s->blk_lvl = g_new(uint16_t, bw * bh * 3);
        s->bw = bw;
        s->bh = bh;
    }
    memset(s->hist, 0, sizeof(s->hist));
    for (by = 0; by < bh; by++) {
        for (bx = 0; bx < bw; bx++) {
            uint32_t b = by * bw + bx;
            double rgb[3], lvl[3];
            uint32_t zl = (by * AE_LITE_N / bh) * AE_LITE_N + bx * AE_LITE_N / bw;
            uint32_t zb = (by * AE_BIG_N / bh) * AE_BIG_N + bx * AE_BIG_N / bw;

            rk_scene_rgb(s->frame, (bx + 0.5) / bw, (by + 0.5) / bh, rgb);
            for (c = 0; c < 3; c++) {
                lvl[c] = MIN(rgb[c] * k, LEVEL_MAX);
                s->blk_lvl[b * 3 + c] = lvl[c];
                lite[zl][c] += lvl[c];
                big[zb][c] += lvl[c];
            }
            lite[zl][3]++;
            big[zb][3]++;
            s->hist[(uint32_t)lvl[1] >> 4] += BLK * BLK;
            rk_scene_yuv(lvl, true, &s->blk_yuv[b * 3]);
            if (REG(s, R_CPROC_CTRL) & 1) {
                rk_isp32_cproc(s, &s->blk_yuv[b * 3]);
            }
        }
    }
    /* zone means: G 12-bit, B and R 10-bit */
    for (i = 0; i < AE_LITE_N * AE_LITE_N; i++) {
        uint64_t n = MAX(lite[i][3], 1);

        s->ae_lite[i] = (lite[i][1] / n) | (lite[i][2] / n / 4) << 12 |
                        (lite[i][0] / n / 4) << 22;
        REG(s, R_AE_LITE + 0x10 + 4 * i) = s->ae_lite[i];
    }
    for (i = 0; i < AE_BIG_N * AE_BIG_N; i++) {
        uint64_t n = MAX(big[i][3], 1);

        s->ae_big[i] = (big[i][1] / n) | (big[i][2] / n / 4) << 12 |
                       (big[i][0] / n / 4) << 22;
    }
    for (c = 0; c < 3; c++) {                   /* window sums, R G B */
        uint64_t sum = 0;
        uint32_t v;

        for (i = 0; i < AE_BIG_N * AE_BIG_N; i++) {
            sum += big[i][c];
        }
        v = MIN(sum * BLK * BLK / (c == 1 ? 1 : 4), UINT32_MAX);
        REG(s, R_AE_BIG + 0x30 + 0x10 * c) = v;
        REG(s, R_AE_BIG2 + 0x30 + 0x10 * c) = v;
        REG(s, R_AE_BIG3 + 0x30 + 0x10 * c) = v;
    }
}

/* The frame's NV12 into a path's buffers: Y, then interleaved CbCr. */
static void rk_isp32_draw(RkIsp32State *s, uint32_t y_base, uint32_t y_size,
                          uint32_t c_base, uint32_t c_size, uint32_t stride)
{
    uint32_t w = REG(s, R_ISP_OUT_H) & 0x3fff;
    uint32_t h = REG(s, R_ISP_OUT_V) & 0x3fff;
    g_autofree uint8_t *yl = NULL, *cl = NULL;
    uint32_t x, y;

    if (!y_base || !w || !h || !s->blk_yuv) {
        return;
    }
    stride = MAX(stride, w);
    if (y_size) {
        h = MIN(h, y_size / stride);
    }
    yl = g_malloc0(stride);
    cl = g_malloc0(stride);
    for (y = 0; y < h; y++) {
        const uint8_t *row = s->blk_yuv + (y / BLK) * s->bw * 3;

        if (y % BLK == 0) {
            for (x = 0; x < w; x++) {
                yl[x] = row[(x / BLK) * 3];
            }
            for (x = 0; x + 1 < w; x += 2) {
                cl[x] = row[(x / BLK) * 3 + 1];
                cl[x + 1] = row[(x / BLK) * 3 + 2];
            }
        }
        dma_memory_write(&address_space_memory, y_base + y * stride, yl,
                         stride, MEMTXATTRS_UNSPECIFIED);
        if (c_base && !(y & 1) && (!c_size || (y / 2 + 1) * stride <= c_size)) {
            dma_memory_write(&address_space_memory,
                             c_base + (y / 2) * stride, cl, stride,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
}

/*
 * CTRL, its 3A_RIS bit: the big AE windows 1..3 (the driver's rawae3,
 * rawae1, rawae2), the lite one (rawae0), the histograms likewise, AWB.
 * The vendor AE reads the window its IQ file selects; a block it
 * enabled but that never reports done leaves the AE statistics invalid
 * and AE never runs.
 */
static const uint32_t rk_isp32_meas[][2] = {
    { R_AE_BIG, 1u << 0 }, { R_AE_LITE, 1u << 1 },
    { R_AE_BIG2, 1u << 2 }, { R_AE_BIG3, 1u << 3 },
    { R_HIST_BIG, 1u << 4 }, { R_HIST_LITE, 1u << 5 },
    { R_HIST_BIG2, 1u << 6 }, { R_HIST_BIG3, 1u << 7 },
    { R_AWB, 1u << 11 },
};

static bool rk_isp32_is_meas_ctrl(hwaddr off)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(rk_isp32_meas); i++) {
        if (rk_isp32_meas[i][0] == off) {
            return true;
        }
    }
    return false;
}

/*
 * The frame's 3A statistics: done flags and RIS bits; AE and histograms
 * were taken when it was rendered; AWB is neutral (so is the light).
 */
static void rk_isp32_stats(RkIsp32State *s)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(rk_isp32_meas); i++) {
        if (REG(s, rk_isp32_meas[i][0]) & 1) {
            REG(s, rk_isp32_meas[i][0]) |= MEAS_DONE;
            REG(s, R_3A_RIS) |= rk_isp32_meas[i][1];
        }
    }
    for (i = 0; i < 4; i++) {                   /* illuminants */
        uint32_t b = R_AWB + 0x340 + 0x30 * i;

        REG(s, b + 0x0) = AWB_WP << 8;          /* R/G sum, Q8 */
        REG(s, b + 0x8) = AWB_WP << 8;          /* B/G sum */
        REG(s, b + 0xc) = AWB_WP;
        REG(s, b + 0x10) = AWB_WP << 8;         /* the big window's */
        REG(s, b + 0x18) = AWB_WP << 8;
        REG(s, b + 0x1c) = AWB_WP;
        REG(s, R_AWB + 0x630 + 4 * i) = AWB_WP;
    }
    for (i = 0; i < 4; i++) {
        REG(s, R_AWB + 0x620 + 4 * i) = (AWB_WP / 8) | (AWB_WP / 8) << 16;
    }
    memset(s->port_idx, 0, sizeof(s->port_idx));
}

/* The frame's means and histograms where the driver reads them: DDR. */
static void rk_isp32_stats_ddr(RkIsp32State *s)
{
    static const uint32_t ae_big_at[] = { 0x000, 0x390, 0x720 };
    static const uint32_t hist_at[] = { 0xc00, 0x1000, 0x1400, 0x1800 };
    uint32_t le[HIST_BINS];
    int i;

    if (!s->stats_base) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(ae_big_at); i++) {
        uint32_t m[AE_BIG_N * AE_BIG_N];
        int k;

        for (k = 0; k < ARRAY_SIZE(m); k++) {
            m[k] = cpu_to_le32(s->ae_big[k]);
        }
        dma_memory_write(&address_space_memory, s->stats_base + ae_big_at[i],
                         m, sizeof(m), MEMTXATTRS_UNSPECIFIED);
    }
    for (i = 0; i < AE_LITE_N * AE_LITE_N; i++) {
        le[i] = cpu_to_le32(s->ae_lite[i]);
    }
    dma_memory_write(&address_space_memory, s->stats_base + 0xab0, le,
                     4 * AE_LITE_N * AE_LITE_N, MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < HIST_BINS; i++) {
        le[i] = cpu_to_le32(s->hist[i]);
    }
    for (i = 0; i < ARRAY_SIZE(hist_at); i++) {
        dma_memory_write(&address_space_memory, s->stats_base + hist_at[i],
                         le, sizeof(le), MEMTXATTRS_UNSPECIFIED);
    }
}

static void rk_isp32_latch_mp(RkIsp32State *s)
{
    REG(s, R_MI_CTRL_SHD) = (REG(s, R_MI_CTRL) & MI_MP_ENABLE) ?
                            (MI_SHD_MP_IN | MI_SHD_MP_OUT) : 0;
    REG(s, R_MP_Y_SHD) = REG(s, R_MP_Y_BASE);
    REG(s, R_MP_CB_SHD) = REG(s, R_MP_CB_BASE);
}

static void rk_isp32_latch_bp(RkIsp32State *s)
{
    REG(s, R_MI_CTRL2_SHD) = (REG(s, R_MI_CTRL2) & ~BP_EN_OUT_SHD) |
        ((REG(s, R_BP_CTRL) & BP_ENABLE) ? BP_EN_OUT_SHD : 0);
    REG(s, R_BP_Y_SHD) = REG(s, R_BP_Y_BASE);
    REG(s, R_BP_CB_SHD) = REG(s, R_BP_CB_BASE);
}

/* ISP20 read back: one pass's start or end */
static void rk_isp32_readback_tick(RkIsp32State *s)
{
    uint32_t mi = 0;

    if (!s->passes) {
        return;
    }
    if (!s->in_frame) {
        rk_isp32_latch_mp(s);
        REG(s, R_ISP_RIS) |= ISP_V_START;
        s->in_frame = true;
    } else {
        s->in_frame = false;
        REG(s, R_ISP_RIS) |= ISP_FRAME_IN | ISP_FRAME;
        rk_isp32_render(s);                 /* each pass measures */
        rk_isp32_stats(s);
        if (--s->passes == 0) {
            if (REG(s, R_MPFBC_CTRL) & 1) {
                mi |= MI_MPFBC_FRAME;
            } else if (REG(s, R_MI_CTRL_SHD) & MI_SHD_MP_OUT) {
                mi |= MI_MP_FRAME;
            }
            s->frame++;
            REG(s, R_MI_RIS) |= mi;
        }
    }
    rk_isp32_update(s);
    if (s->passes) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  READBACK_NS / 2);
    }
}

/* frame start: the configuration takes effect */
static void rk_isp32_frame_begin(RkIsp32State *s, bool online)
{
    if (online) {
        if (s->sensor) {
            image_sensor_frame_start(s->sensor);
        }
        qemu_irq_pulse(s->vicap_fs);
    }
    rk_isp32_latch_mp(s);
    rk_isp32_latch_bp(s);
    s->stats_base = REG(s, R_MI_3A_BASE);
    REG(s, R_ISP_RIS) |= ISP_V_START;
    s->in_frame = true;
}

/* frame end: the picture lands in the latched buffers */
static void rk_isp32_frame_end(RkIsp32State *s, bool online)
{
    uint32_t mi = 0;

    rk_isp32_render(s);
    rk_isp32_stats_ddr(s);
    if (online) {
        qemu_irq_pulse(s->vicap_fe);
    }
    if (REG(s, R_MI_CTRL_SHD) & MI_SHD_MP_OUT) {
        rk_isp32_draw(s, REG(s, R_MP_Y_SHD), REG(s, R_MP_Y_SIZE),
                      REG(s, R_MP_CB_SHD), REG(s, R_MP_CB_SIZE),
                      REG(s, R_MP_LLENGTH));
        mi |= MI_MP_FRAME;
    }
    if (REG(s, R_MI_CTRL2_SHD) & BP_EN_OUT_SHD) {
        rk_isp32_draw(s, REG(s, R_BP_Y_SHD), REG(s, R_BP_Y_SIZE),
                      REG(s, R_BP_CB_SHD), REG(s, R_BP_CB_SIZE),
                      REG(s, R_BP_LLENGTH));
        mi |= MI_BP_FRAME;
    }
    s->frame++;
    rk_isp32_stats(s);
    REG(s, R_ISP_RIS) |= ISP_FRAME_IN | ISP_FRAME;
    REG(s, R_MI_RIS) |= mi;
    s->in_frame = false;
}

static void rk_isp32_tick(void *opaque)
{
    RkIsp32State *s = opaque;

    if (s->readback) {
        rk_isp32_readback_tick(s);
        return;
    }

    if (s->passes) {
        /* a read-back pass over the frame VICAP left in memory */
        if (!s->in_frame) {
            rk_isp32_frame_begin(s, false);
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      READBACK_NS / 2);
        } else {
            rk_isp32_frame_end(s, false);
            s->passes = 0;
            /* back to watching the online source */
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      2 * rk_isp32_half_frame(s));
        }
        rk_isp32_update(s);
        return;
    }

    if (!(REG(s, R_ISP_CTRL0) & ISP_ENABLE)) {
        s->in_frame = false;
        return;
    }
    if ((s->source_gated && !s->source_on) ||
        (s->sensor && !image_sensor_streaming(s->sensor))) {
        /* no data in: look again in a frame's time */
        s->in_frame = false;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  2 * rk_isp32_half_frame(s));
        return;
    }
    if (!s->in_frame) {
        rk_isp32_frame_begin(s, true);
    } else {
        rk_isp32_frame_end(s, true);
    }
    rk_isp32_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              rk_isp32_half_frame(s));
}

static void rk_isp32_source_on(void *opaque, int n, int level)
{
    RkIsp32State *s = opaque;
    bool was = s->source_on;

    s->source_on = level;
    if (level && !was && (REG(s, R_ISP_CTRL0) & ISP_ENABLE) &&
        !timer_pending(s->timer)) {
        s->in_frame = false;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  rk_isp32_half_frame(s));
    }
}

static uint64_t rk_isp32_read(void *opaque, hwaddr off, unsigned size)
{
    RkIsp32State *s = opaque;

    switch (off) {
    case R_ISP_ICR:
    case R_ISP_ISR:
    case R_3A_ICR:
    case R_MI_ICR:
    case R_MI_ISR:
        return 0;
    case R_AE_BIG + PORT_OFF:                   /* 15 x 15 means */
        return s->ae_big[s->port_idx[0]++ % (AE_BIG_N * AE_BIG_N)];
    case R_AE_BIG2 + PORT_OFF:
    case R_AE_BIG3 + PORT_OFF:
        return s->ae_big[s->port_idx[0]++ % (AE_BIG_N * AE_BIG_N)];
    case R_HIST_BIG + PORT_OFF:
    case R_HIST_BIG2 + PORT_OFF:
    case R_HIST_BIG3 + PORT_OFF:
    case R_HIST_LITE + PORT_OFF: {
        uint32_t *k = &s->port_idx[off == R_HIST_LITE + PORT_OFF ? 2 : 1];

        return *k < HIST_BINS ? s->hist[(*k)++] : 0;
    }
    case R_AWB_RAM:                             /* per-block RGB: none */
        s->port_idx[3]++;
        return 0;
    }
    return s->regs[off / 4];
}

static void rk_isp32_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkIsp32State *s = opaque;
    bool was_on = REG(s, R_ISP_CTRL0) & ISP_ENABLE;

    switch (off) {
    case R_ISP_RIS:
    case R_ISP_MIS:
    case R_3A_RIS:
    case R_3A_MIS:
    case R_MI_RIS:
    case R_MI_MIS:
    case R_MI_CTRL_SHD:
    case R_MP_Y_SHD:
    case R_MP_CB_SHD:
    case R_BP_Y_SHD:
    case R_BP_CB_SHD:
    case R_MI_CTRL2_SHD:
        return;                         /* read-only */
    case R_ISP_ICR:
        REG(s, R_ISP_RIS) &= ~val;
        break;
    case R_ISP_ISR:
        REG(s, R_ISP_RIS) |= val;
        break;
    case R_3A_ICR:
        REG(s, R_3A_RIS) &= ~val;
        break;
    case R_MI_ICR:
        REG(s, R_MI_RIS) &= ~val;
        break;
    case R_MI_ISR:
        REG(s, R_MI_RIS) |= val;
        break;
    default:
        if (rk_isp32_is_meas_ctrl(off)) {
            /* MEAS_DONE is write-1-to-clear; the rest configuration */
            s->regs[off / 4] = (val & ~MEAS_DONE) |
                ((s->regs[off / 4] & MEAS_DONE) & ~(uint32_t)val);
            return;
        }
        if (off == R_MI_CTRL2) {
            /* the self-update bits act at once and do not stay set */
            s->regs[off / 4] = val & ~(MP_SELF_UPD | BP_SELF_UPD);
            if (val & MP_SELF_UPD) {
                rk_isp32_latch_mp(s);
            }
            if (val & BP_SELF_UPD) {
                rk_isp32_latch_bp(s);
            }
            return;
        }
        s->regs[off / 4] = val;
        if (s->readback) {
            if (off == R_CSI2RX_CTRL0 && (val & 1) && !s->passes) {
                s->passes = ((val >> 12) & 3) + 1;
                s->in_frame = false;
                timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          READBACK_NS / 2);
            }
            return;
        }
        if (off == R_CSI2RX_CTRL0 && (val & 1) && !s->passes &&
            !(s->source_gated && s->source_on)) {
            /*
             * Multi-sensor ISP32: VICAP wrote the raw frame to memory and
             * the driver starts one pass over it (rkisp_trigger_read_back).
             */
            s->passes = 1;
            s->in_frame = false;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      READBACK_NS / 2);
            return;
        }
        if (off == R_ISP_CTRL0 && (val & ISP_ENABLE) && !was_on) {
            s->in_frame = false;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      rk_isp32_half_frame(s));
        }
        return;
    }
    rk_isp32_update(s);
}

static const MemoryRegionOps rk_isp32_ops = {
    .read = rk_isp32_read,
    .write = rk_isp32_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_isp32_reset(DeviceState *dev)
{
    RkIsp32State *s = RK_ISP32(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->frame = 0;
    s->in_frame = false;
    s->passes = 0;
    timer_del(s->timer);
    rk_isp32_update(s);
}

static void rk_isp32_realize(DeviceState *dev, Error **errp)
{
    RkIsp32State *s = RK_ISP32(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_isp32_ops, s,
                          TYPE_RK_ISP32, REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq_isp);
    sysbus_init_irq(sbd, &s->irq_mi);
    sysbus_init_irq(sbd, &s->irq_mipi);
    qdev_init_gpio_out_named(dev, &s->vicap_fs, "vicap-fs", 1);
    qdev_init_gpio_out_named(dev, &s->vicap_fe, "vicap-fe", 1);
    qdev_init_gpio_in_named(dev, rk_isp32_source_on, "source-on", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_isp32_tick, s);
}

static const Property rk_isp32_properties[] = {
    DEFINE_PROP_UINT32("fps", RkIsp32State, fps, 30),
    DEFINE_PROP_BOOL("source-gated", RkIsp32State, source_gated, false),
    DEFINE_PROP_BOOL("readback", RkIsp32State, readback, false),
    DEFINE_PROP_LINK("sensor", RkIsp32State, sensor, TYPE_DEVICE, Object *),
};

static const VMStateDescription rk_isp32_vmstate = {
    .name = TYPE_RK_ISP32,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkIsp32State, REGS_SIZE / 4),
        VMSTATE_UINT32(frame, RkIsp32State),
        VMSTATE_UINT32_ARRAY(port_idx, RkIsp32State, 4),
        VMSTATE_UINT32(stats_base, RkIsp32State),
        VMSTATE_BOOL(source_on, RkIsp32State),
        VMSTATE_BOOL(in_frame, RkIsp32State),
        VMSTATE_UINT32(passes, RkIsp32State),
        VMSTATE_TIMER_PTR(timer, RkIsp32State),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_isp32_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_isp32_realize;
    device_class_set_legacy_reset(dc, rk_isp32_reset);
    device_class_set_props(dc, rk_isp32_properties);
    dc->vmsd = &rk_isp32_vmstate;
}

static const TypeInfo rk_isp32_info = {
    .name = TYPE_RK_ISP32,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkIsp32State),
    .class_init = rk_isp32_class_init,
};

static void rk_isp32_register_types(void)
{
    type_register_static(&rk_isp32_info);
}

type_init(rk_isp32_register_types)
