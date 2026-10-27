/*
 * MStar infinity3 ISP and MIPI CSI receiver: the frame clock, the pixel
 * pipeline and the 3A statistics.
 *
 * Read off the vendor kernel's drivers/mstar/isp/infinity3 (hal_isp.c,
 * drv_isp.c, infinity3e_reg_isp0.h..isp9.h; the I3 SDK
 * MSC3XX_SDK_V_1711291828).  The sensor feeds CSI, CSI the ISP and the ISP
 * the scaler in real time.  Three register windows are mapped here:
 *
 * ISP0 top, bank 0x1302 (16-bit registers on a 4-byte stride):
 *   0x28 mask (1 = masked), 0x29 force, 0x2A clear, 0x2B final status,
 *   0x2C raw status: interrupt group 1 (bit 14 ISPIF_VSYNC, the frame
 *   start; 15 AE_DONE; 8 AWB_DONE; 4 AF_DONE).  0x2D [3:0] mask2 and
 *   [11:8] force2, 0x2E [3:0] clear2, 0x2F [3:0] final2 and [11:8] raw2:
 *   group 2 (VDOS line events; never raised).  0x4A-0x4E mask3, force3,
 *   clear3, final3, raw3: group 3 (bit 0 AE_WIN0_DONE, 2 AE block row,
 *   4 SW_INT_INPUT_DONE -- the frame end, 5 SW_INT_OUTPUT_DONE, 8
 *   HIT_LINE_COUNT3).  final = raw & ~mask; the line is high while any
 *   final is non-zero.  The ISR clears what it handled with a 1 then a 0
 *   to clear, so clear reads back as written and only a 1 acts.
 *   0x7A frame counters: [6:0] frames started, [14:8] frames done (the
 *   scaler driver reads it too), bit 15 clears both.
 *
 * ISP4, bank 0x1306, the menuload (MIU -> ISP SRAM tables): 0x00 [3:0]
 * the SRAM, bit 8 write; 0x01 words - 1, 0x02 the first entry, 0x03/0x04
 * the MIU address >> 4.  0x07 bit 0 queues that load (the model copies
 * it at once), bit 1 starts the queue, after which the driver polls 0x00
 * bit 11 (load done) for up to 50 ms; 0x06 reads back SRAM, as zero.
 *
 * CSI MAC, bank 0x1204: 0x06/0x07/0x08 report-interrupt mask, force and
 * clear, 0x0E source (raw & ~mask), 0x0F raw; bit 6 is VC0, the frame end
 * with vc0_vs_mode 1, which the ISR turns into the ISP's VEND event.
 * 0x03-0x05 and 0x0C/0x0D are the same for the error interrupts (never
 * raised).  0x16 bit 0 reads idle.
 *
 * Each frame, while group 1's VSYNC is unmasked (HalISPEnableInt at the
 * end of probe): VSYNC; at 60 % of the frame the picture and statistics
 * (below), then AE row, AE window 0, AE and AWB done; at 90 % the frame
 * end, the output end, line count 3, the done count and CSI's VC0.  GPIO
 * out 0 pulses at the frame start and 1 at the frame end, for the
 * scaler, which takes the picture through sstar_i3_isp_frame().
 *
 * The picture.  The sensor (the "sensor" link, hisi-jxf22) looks at a
 * scene: sstar-pattern.h's, the OpenIPC logo and its sliding dot in an
 * 80 % white on the navy gradient, as linear reflectances, under a light
 * that is not white -- tungsten, the sensor's R and B at 0.82 and 0.40 of its G
 * ("light-r", "light-b", per mille, and "light", the level: 1/1000 LSB
 * per row of exposure for a white under G; all three may change while
 * running, qom-set /machine/isp).  From the exposure rows and analog gain
 * in its I2C registers (hisi_soi_exposure()) it makes 10-bit Bayer at its
 * window size with black at 64, in the colour order ISP0 0x11 [1:0] says
 * (0 RGGB, 1 GRBG, 2 BGGR -- the camera's, 3 GBRG), cropped by ISP0
 * 0x12-0x15.  The stages, as the camera application programs them (all
 * found by tracing it; disabled ones pass through):
 *
 *   OBC         ISP0 0x00 bit 2: (in - offset) x gain; ISP1 0x67/0x69 the
 *               offsets, 10-bit units (0x40), 0x6A-0x6D gains, 4.10
 *   AE dgain    ISP1 0x6E bit 0, 0x6F, 4.10 (the 3A sets ~1.0 a frame)
 *   statistics  taken here, before white balance (ISP3 0x75 = 0x20: AE
 *               from the WB pre-gain, AWB times the colour shading gain,
 *               which is off)
 *   WBG0, WBG1  ISP0 0x00 bits 3 / 4; ISP1 0x08-0x0F and ISP6 (bank 0x1308)
 *               0x48-0x4F, R Gr B Gb, 3.10.  The 3A drives WBG1.
 *   GM12to10    ISP1 0x06 bit 12 (on): the Bayer gamma, menuload SRAMs
 *               3-5, 256 entries a 16 apart, interpolated; [9:0] with
 *               bit 13 ends the top segment.  The application's is linear.
 *   demosaic    (CFAI, ISP0 0x70, always): each 2x2 quad gives one RGB
 *               pixel, R, the mean of the Gs, B: a half-size picture,
 *               960x540, which the scaler resamples
 *   GM10to10    ISP0 0x46 bit 0: the RGB gamma, SRAMs 10-12, a 4 apart;
 *               [11:2] with bit 1 the top.  The application's is a real
 *               display curve.
 *   R2Y         ISP0 0x3C-0x45: a 3x3 matrix, 13-bit signed / 1024, rows
 *               Cr, Y, Cb, with the optional -16 on the inputs and +16 /
 *               +128 on the outputs (0x3C bits 1-8).  The 3A library folds
 *               its colour correction in (HalISPSetYUVCCM: there is no
 *               separate CCM); with bit 0 off, BT.601 full range.
 *
 * Passed through: LSC and ALSC (off), DPC, spike, Bayer and YUV noise
 * reduction, false-colour suppression and the demosaic's own filters (a
 * synthetic noise-free scene has nothing for them), FPN (off), HSV and
 * saturation tables (off), RGB-IR, HDR and the other statistics (AF,
 * histograms: not written).  The VIP's YUV gamma (bank 0x1226 0x40) and
 * the scaler's RGB gammas and CCM (bank 0x1211 0x62, 0x76, 0x77) sit in
 * the scaler; the application leaves its gammas off and its CCM at the
 * identity, so the picture leaves the ISP as it goes into the encoder.
 *
 * The statistics: per block the mean R, G and B (and for AE a Y, 0.30 R +
 * 0.59 G + 0.11 B) of the OBC'd data as 8 bits (12-bit >> 4).  AE blocks
 * are 4 bytes and AWB blocks 3, at the bases ISP3 (bank 0x1305) gives:
 * AE base 0x06/0x07, max 0x10/0x11, block size - 1 0x01/0x02, blocks - 1
 * 0x03 ([6:0] x, [14:8] y), offset 0x04/0x05; AWB 0x08/0x09, 0x12/0x13,
 * 0x51/0x52, 0x53, 0x54/0x55.  Bases are MIU addresses >> 4, low 16 bits
 * then the rest; the maximum is exclusive.  The camera's 3A asks for
 * 128x80 AE blocks of 15x13 and 128x90 AWB blocks of 15x12, and runs its
 * exposure (sensor rows and gain, over I2C) and its WBG1 gains from them.
 * With no sensor, the ISP off (ISP0 0x00 bit 0) or "pixels" off, every
 * block reads mid grey and there is no picture.
 *
 * Cost: a quad row the same as the one above is copied rather than
 * worked out again, and so is a scene row; this scene, whose rows repeat,
 * costs ~0.75 ms a 1080p frame, against ~10.6 ms done in full.
 *
 * "trace" logs register writes but the per-frame ones, and a line a
 * frame: exposure, gains and the mean Y, Cb, Cr.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-riu.h"
#include "hw/misc/sstar-i3-isp.h"
#include "hw/i2c/hisi-soi.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define BANK_REGS           0x80
#define BANK_ISP1           0x1303
#define BANK_ISP3           0x1305
#define BANK_ISP6           0x1308
#define DRAM_BASE           0x20000000

/* ISP0 */
#define G1_MASK             0x28
#define G1_FORCE            0x29
#define G1_CLR              0x2A
#define G1_FINAL            0x2B
#define G1_RAW              0x2C
#define G2_MASK_FORCE       0x2D
#define G2_CLR              0x2E
#define G2_STATUS           0x2F
#define G3_MASK             0x4A
#define G3_FORCE            0x4B
#define G3_CLR              0x4C
#define G3_FINAL            0x4D
#define G3_RAW              0x4E
#define FRM_CNT             0x7A
#define TOP_EN              0x00
#define TOP_EN_ISP          (1u << 0)
#define TOP_EN_OBC          (1u << 2)
#define TOP_EN_WBG0         (1u << 3)
#define TOP_EN_WBG1         (1u << 4)
#define SEN_ARRAY           0x11
#define CROP_X              0x12        /* x, y, width - 1, height - 1 */
#define R2Y_CTRL            0x3C
#define R2Y_COEF            0x3D        /* 9: the Cr, Y, Cb rows */
#define GM10_CTRL           0x46

#define G1_VSYNC            (1u << 14)
#define G1_AE_DONE          (1u << 15)
#define G1_AWB_DONE         (1u << 8)
#define G3_AE_WIN0          (1u << 0)
#define G3_AE_ROW           (1u << 2)
#define G3_IN_DONE          (1u << 4)
#define G3_OUT_DONE         (1u << 5)
#define G3_HIT_LINE3        (1u << 8)

/* ISP4 */
#define ML_CTRL             0x00
#define ML_DONE             (1u << 11)
#define ML_SRAM_DATA        0x06
#define ML_TRIG             0x07
#define ML_TRIG_LOAD        (1u << 1)
#define ML_TRIG_REG         (1u << 0)
#define ML_RW               (1u << 8)
#define ML_AMOUNT           0x01
#define ML_SRAM_ADDR        0x02
#define ML_ADDR             0x03

/* ISP1 */
#define GM12_CTRL           0x06
#define WBG0_GAIN           0x08        /* R, Gr, B, Gb on every other */
#define OBC_OFST            0x67        /* R | Gr << 8, then Gb | B << 8 */
#define OBC_GAIN            0x6A        /* R, Gr, Gb, B */
#define AE_DGAIN_EN         0x6E
#define AE_DGAIN            0x6F
/* ISP3 */
#define STS_AE_SIZE         0x01        /* AE: block size - 1, x then y */
#define STS_AE_NUM          0x03        /* [6:0] x - 1, [14:8] y - 1 */
#define STS_AE_OFFSET       0x04        /* x, y */
#define STS_AE_BASE         0x06
#define STS_AWB_BASE        0x08
#define STS_AE_MAX          0x10
#define STS_AWB_MAX         0x12
#define STS_AWB_SIZE        0x51
#define STS_AWB_NUM         0x53
#define STS_AWB_OFFSET      0x54
/* ISP6 */
#define WBG1_GAIN           0x48

/* the menuload SRAMs modelled: ids 3-5 and 10-12 (hal_isp.h) */
#define ML_ID_GM12          3
#define ML_ID_GM10          10
#define GM_ENTRIES          256

#define MAX_BLK             128
#define MAX_W               4096
#define MAX_H               4096
#define BLACK               64          /* the JX-F22's, 10-bit */
#define RAW_MAX             1023

/* Bayer colours, in the order of the OBC registers */
enum { C_R, C_GR, C_GB, C_B };

/* CSI */
#define CSI_ERR_MASK        0x03
#define CSI_ERR_FORCE       0x04
#define CSI_ERR_CLR         0x05
#define CSI_RPT_MASK        0x06
#define CSI_RPT_FORCE       0x07
#define CSI_RPT_CLR         0x08
#define CSI_ERR_SRC         0x0C
#define CSI_ERR_RAW         0x0D
#define CSI_RPT_SRC         0x0E
#define CSI_RPT_RAW         0x0F
#define CSI_MAC_IDLE        0x16
#define CSI_RPT_VC0         (1u << 6)

enum { PHASE_START, PHASE_STATS, PHASE_END };

/*
 * The scene: sstar-pattern.h's picture as linear reflectances, 1.0 =
 * 4096.  The logo and the dot are an 80 % white; the navy, a band of the
 * gradient a row, is the pattern's colour taken back through BT.601 and a
 * 2.2 gamma, with that white at 235.
 */
#define SC_WHITE            3277
#define SC_BANDS            16

static void i3_scene_navy(int band, double *rgb)
{
    double y = (50 - 14.0 * band / SC_BANDS - 16) * 255 / 219;
    double cb = (146 - 128) * 255 / 224.0, cr = (122 - 128) * 255 / 224.0;
    double v[3] = { y + 1.402 * cr, y - 0.344 * cb - 0.714 * cr,
                    y + 1.772 * cb };
    int c;

    for (c = 0; c < 3; c++) {
        rgb[c] = SC_WHITE * pow(MAX(v[c], 0) / 255, 2.2);
    }
}

struct SstarI3IspState {
    SysBusDevice parent_obj;

    MemoryRegion isp0_mem, isp4_mem, csi_mem;
    qemu_irq isp_irq, csi_irq;
    qemu_irq frame_out[2];          /* frame start, frame end */
    QEMUTimer *timer;
    SstarRiuState *riu;

    uint16_t isp0[BANK_REGS], isp4[BANK_REGS], csi[BANK_REGS];
    uint16_t raw1, raw2, raw3;
    uint16_t csi_rpt_raw, csi_err_raw;
    uint8_t phase;
    int64_t frame_start;
    uint32_t fps;
    uint32_t light, light_r, light_b;
    bool pixels;
    bool trace;
    Object *sensor;

    uint16_t gamma[6][GM_ENTRIES];  /* menuload: 12to10 R G B, 10to10 */
    uint32_t frames;

    /* the frame, rebuilt every frame and not migrated */
    uint16_t *raw;                  /* 10-bit Bayer at the crop size */
    uint8_t *yuv;                   /* Y, Cb, Cr planes at half that */
    uint8_t *idx;                   /* a scene row, the last odd, even */
    uint32_t *ae, *awb;             /* block sums: R, G, B, count */
    int out_w, out_h;
    bool have_frame;
};

static void i3_isp_update(SstarI3IspState *s)
{
    uint16_t mask2 = s->isp0[G2_MASK_FORCE] & 0xF;
    bool isp = (s->raw1 & ~s->isp0[G1_MASK]) ||
               (s->raw2 & ~mask2 & 0xF) ||
               (s->raw3 & ~s->isp0[G3_MASK]);
    bool csi = (s->csi_rpt_raw & ~s->csi[CSI_RPT_MASK] & 0x7F) ||
               (s->csi_err_raw & ~s->csi[CSI_ERR_MASK] & 0x7FFF);

    qemu_set_irq(s->isp_irq, isp);
    qemu_set_irq(s->csi_irq, csi);
}

static bool i3_isp_streaming(SstarI3IspState *s)
{
    return !(s->isp0[G1_MASK] & G1_VSYNC);
}

static int64_t i3_isp_period(SstarI3IspState *s)
{
    return NANOSECONDS_PER_SECOND / (s->fps ? s->fps : 30);
}

static void i3_isp_schedule(SstarI3IspState *s)
{
    int64_t when, now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!i3_isp_streaming(s)) {
        timer_del(s->timer);
        return;
    }
    if (timer_pending(s->timer)) {
        return;
    }
    if (s->phase == PHASE_START) {
        when = now + i3_isp_period(s) / 10;
    } else {
        when = s->frame_start +
               i3_isp_period(s) * (s->phase == PHASE_STATS ? 6 : 9) / 10;
        when = MAX(when, now);
    }
    timer_mod(s->timer, when);
}

/* A DMA base or bound in ISP3: MIU >> 4, the low 16 bits then the rest. */
static hwaddr i3_isp_miu(SstarI3IspState *s, int reg)
{
    uint32_t v = sstar_riu_get(s->riu, BANK_ISP3, reg) |
                 (uint32_t)(sstar_riu_get(s->riu, BANK_ISP3, reg + 1) & 0x7FF)
                 << 16;

    return (hwaddr)v << 4;
}

static void i3_isp_dma(SstarI3IspState *s, int base_reg, int max_reg,
                       const uint8_t *buf, uint32_t len)
{
    hwaddr base = i3_isp_miu(s, base_reg);
    hwaddr max = i3_isp_miu(s, max_reg);

    if (!base || max <= base) {
        return;
    }
    address_space_write(&address_space_memory, DRAM_BASE + base,
                        MEMTXATTRS_UNSPECIFIED, buf, MIN(len, max - base));
}

static uint32_t i3_isp_blocks(SstarI3IspState *s, int num_reg)
{
    uint16_t n = sstar_riu_get(s->riu, BANK_ISP3, num_reg);

    return ((n & 0x7F) + 1) * (((n >> 8) & 0x7F) + 1);
}

/* Without a picture: every block mid grey. */
static void i3_isp_grey_stats(SstarI3IspState *s)
{
    uint32_t ae = i3_isp_blocks(s, STS_AE_NUM) * 4;
    uint32_t awb = i3_isp_blocks(s, STS_AWB_NUM) * 3;
    g_autofree uint8_t *buf = g_malloc(MAX(ae, awb));

    memset(buf, 0x80, MAX(ae, awb));
    i3_isp_dma(s, STS_AE_BASE, STS_AE_MAX, buf, ae);
    i3_isp_dma(s, STS_AWB_BASE, STS_AWB_MAX, buf, awb);
}

static uint16_t i3_isp1(SstarI3IspState *s, int reg)
{
    return sstar_riu_get(s->riu, BANK_ISP1, reg);
}

/* One step of a menuload: a table from DRAM into a gamma SRAM. */
static void i3_isp_mload(SstarI3IspState *s)
{
    int id = s->isp4[ML_CTRL] & 0xF;
    uint32_t n = s->isp4[ML_AMOUNT] + 1;
    uint32_t at = s->isp4[ML_SRAM_ADDR];
    hwaddr a = DRAM_BASE + ((hwaddr)(s->isp4[ML_ADDR] |
                                     (uint32_t)s->isp4[ML_ADDR + 1] << 16)
                            << 4);
    uint16_t *t;

    if (!(s->isp4[ML_CTRL] & ML_RW)) {
        return;
    }
    if (id >= ML_ID_GM12 && id < ML_ID_GM12 + 3) {
        t = s->gamma[id - ML_ID_GM12];
    } else if (id >= ML_ID_GM10 && id < ML_ID_GM10 + 3) {
        t = s->gamma[3 + id - ML_ID_GM10];
    } else {
        return;                 /* ALSC, DPC, FPN: those stages are off */
    }
    if (at >= GM_ENTRIES) {
        return;
    }
    n = MIN(n, GM_ENTRIES - at);
    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       t + at, n * 2);
    for (; n; n--, at++) {
        t[at] = le16_to_cpu(t[at]) & 0x3FF;
    }
}

/* A 256-entry curve at @in, interpolated; @last ends the top segment. */
static uint32_t i3_isp_curve(const uint16_t *t, uint32_t in, int shift,
                             uint32_t last)
{
    uint32_t i = in >> shift, f = in & ((1u << shift) - 1);
    int32_t a = t[i], b = i < GM_ENTRIES - 1 ? t[i + 1] : last;

    return a + (((b - a) * (int32_t)f) >> shift);
}

static int32_t i3_isp_s13(uint16_t v)
{
    return (int32_t)(v & 0x1FFF) - (v & 0x1000 ? 0x2000 : 0);
}

/*
 * The scene's row @y (sensor coordinates, @w x @h): each pixel's white
 * coverage, then the navy band in idx[@w].
 */
static void i3_isp_scene_row(SstarI3IspState *s, int w, int h, int y,
                             uint8_t *idx)
{
    int dot = sstar_pattern_dot_x(s->frames);
    int py = (int64_t)y * SSTAR_PAT_H / MAX(h, 1);
    int x;

    if ((py < SSTAR_PAT_LOGO_Y || py >= SSTAR_PAT_LOGO_Y + AW_VINC_LOGO_H) &&
        abs(py - SSTAR_PAT_DOT_Y) > SSTAR_PAT_DOT_R + 1) {
        memset(idx, 0, w);              /* the navy only */
    } else {
        for (x = 0; x < w; x++) {
            idx[x] = sstar_pattern_cover(dot, w, h, x, y);
        }
    }
    idx[w] = (int64_t)y * SC_BANDS / MAX(h, 1);
}

/*
 * The sensor: the scene under the light, at its exposure and gain, plus
 * the black level, as 10-bit Bayer in the colour order the ISP is set
 * for.  Returns false if there is no sensor to read.
 */
static bool i3_isp_sense(SstarI3IspState *s, int cx, int cy, int w, int h)
{
    HisiSoiExposure e;
    uint16_t pal[256][4];
    uint8_t col[4];
    int sw, sh, x, y, i, c, band = -1;
    double k, light[3];

    if (!s->sensor || !hisi_soi_exposure(s->sensor, &e)) {
        return false;
    }
    sw = e.width ? e.width : cx + w;
    sh = e.height ? e.height : cy + h;
    if (sw < cx + w || sh < cy + h || sw > MAX_W) {
        return false;
    }
    if (e.vts) {
        e.lines = MIN(e.lines, e.vts);
    }
    /* LSB per row of exposure at gain 1, reflectance 1, the G light */
    k = e.lines * (e.gain / 1024.0) * (s->light / 1000.0) / 4096;
    light[0] = k * s->light_r / 1000.0;
    light[1] = k;
    light[2] = k * s->light_b / 1000.0;
    /* the colour at each position of a 2x2 quad: [R G; G B] for array 0 */
    for (i = 0; i < 4; i++) {
        col[i] = i ^ (const uint8_t[]){ 0, 1, 3, 2 }[s->isp0[SEN_ARRAY] & 3];
    }
    for (y = 0; y < h; y++) {
        uint16_t *r = s->raw + (size_t)y * w;
        const uint8_t *c0 = col + 2 * (y & 1);
        uint8_t *last = s->idx + (1 + (y & 1)) * (MAX_W + 1);

        /* a row of the scene like the one two above: the same samples */
        i3_isp_scene_row(s, sw, sh, cy + y, s->idx);
        if (y >= 2 && !memcmp(s->idx, last, sw + 1)) {
            memcpy(r, r - 2 * w, w * sizeof(*r));
            continue;
        }
        memcpy(last, s->idx, sw + 1);
        if (s->idx[sw] != band) {
            /* the samples of each coverage over this band's navy */
            double navy[3];

            band = s->idx[sw];
            i3_scene_navy(band, navy);
            for (i = 0; i < 256; i++) {
                for (c = 0; c < 3; c++) {
                    double v = BLACK + light[c] *
                               (navy[c] + (SC_WHITE - navy[c]) * i / 255);

                    pal[i][c == 2 ? C_B : c] = MIN(v + 0.5, RAW_MAX);
                }
                pal[i][C_GB] = pal[i][C_GR];
            }
        }
        for (x = 0; x < w; x++) {
            r[x] = pal[s->idx[cx + x]][c0[x & 1]];
        }
    }
    return true;
}

/* raw 10-bit -> 12-bit after OBC and the AE digital gain, colour @c */
static uint32_t i3_isp_obc(SstarI3IspState *s, int c, uint32_t v12)
{
    static const uint8_t ofst_reg[4] = { 0, 0, 1, 1 };
    uint32_t ofst = i3_isp1(s, OBC_OFST + 2 * ofst_reg[c]) >> (8 * (c & 1));

    if (s->isp0[TOP_EN] & TOP_EN_OBC) {
        ofst = (ofst & 0xFF) << 2;
        v12 = v12 > ofst ? v12 - ofst : 0;
        v12 = (v12 * (i3_isp1(s, OBC_GAIN + c) & 0x3FFF)) >> 10;
    }
    if (i3_isp1(s, AE_DGAIN_EN) & 1) {
        v12 = (v12 * (i3_isp1(s, AE_DGAIN) & 0x3FFF)) >> 10;
    }
    return MIN(v12, 4095);
}

/*
 * The statistics, from the Bayer data after OBC and before white
 * balance (ISP3 0x75: AE from the WB pre-gain, AWB from that times the
 * colour shading gain, which is off): per block the mean R, G, B and
 * (AE) Y, 8 bits.  A 2x2 quad counts in the block of its first pixel.
 */
static void i3_isp_block_map(SstarI3IspState *s, int size_reg, int num_reg,
                             int off_reg, int w, int h, int16_t *bx,
                             int16_t *by, int *nx, int *ny)
{
    uint16_t n = sstar_riu_get(s->riu, BANK_ISP3, num_reg);
    int sx = (sstar_riu_get(s->riu, BANK_ISP3, size_reg) & 0x3FF) + 1;
    int sy = (sstar_riu_get(s->riu, BANK_ISP3, size_reg + 1) & 0x3FF) + 1;
    int ox = sstar_riu_get(s->riu, BANK_ISP3, off_reg) & 0xFFF;
    int oy = sstar_riu_get(s->riu, BANK_ISP3, off_reg + 1) & 0xFFF;
    int i, b;

    *nx = (n & 0x7F) + 1;
    *ny = ((n >> 8) & 0x7F) + 1;
    for (i = 0; i < w / 2; i++) {
        b = 2 * i >= ox ? (2 * i - ox) / sx : -1;
        bx[i] = b < *nx ? b : -1;
    }
    for (i = 0; i < h / 2; i++) {
        b = 2 * i >= oy ? (2 * i - oy) / sy : -1;
        by[i] = b < *ny ? b : -1;
    }
}

static void i3_isp_write_stats(SstarI3IspState *s, const uint32_t *sum,
                               int n, int bpb, int base_reg, int max_reg)
{
    g_autofree uint8_t *buf = g_malloc0(n * bpb);
    int i, c;

    for (i = 0; i < n; i++, sum += 4) {
        uint32_t cnt = sum[3], v[3];

        if (!cnt) {
            continue;
        }
        for (c = 0; c < 3; c++) {
            /* means of 10-bit samples, the G ones two a quad */
            uint32_t d = cnt << (c == 1);
            uint32_t m = (uint32_t)(((uint64_t)sum[c] * 4 + d / 2) / d);

            v[c] = i3_isp_obc(s, c == 2 ? C_B : c, m) >> 4;
            buf[i * bpb + c] = v[c];
        }
        if (bpb == 4) {
            buf[i * bpb + 3] = (77 * v[0] + 150 * v[1] + 29 * v[2]) >> 8;
        }
    }
    i3_isp_dma(s, base_reg, max_reg, buf, n * bpb);
}

/*
 * The pipeline, a 2x2 quad to an output pixel: OBC, AE digital gain,
 * WB pre-gain (WBG0) and post-gain (WBG1), the 12-to-10 Bayer gamma
 * (menuload SRAMs 3-5), the quad's R, mean G and B as its RGB, the
 * 10-to-10 RGB gamma (SRAMs 10-12), then RGB to YCbCr through the matrix
 * in ISP0 0x3D-0x45 (CCM folded in by the 3A library).
 */
static void i3_isp_process(SstarI3IspState *s)
{
    static const uint8_t wbg_reg[4] = { 0, 1, 3, 2 };  /* R Gr Gb B */
    int cx = s->isp0[CROP_X] & 0xFFF, cy = s->isp0[CROP_X + 1] & 0xFFF;
    int w = (s->isp0[CROP_X + 2] & 0xFFF) + 1;
    int h = (s->isp0[CROP_X + 3] & 0xFFF) + 1;
    uint16_t gm12 = i3_isp1(s, GM12_CTRL), gm10 = s->isp0[GM10_CTRL];
    uint16_t r2y = s->isp0[R2Y_CTRL];
    g_autofree uint16_t *lut = g_new(uint16_t, 4 * 1024 + 3 * 1024);
    uint16_t *glut = lut + 4 * 1024;
    int16_t aex[MAX_W / 2], aey[MAX_H / 2], awx[MAX_W / 2], awy[MAX_H / 2];
    int anx, any, wnx, wny, ow = w / 2, oh = h / 2;
    uint32_t rae[MAX_BLK * 4], rawb[MAX_BLK * 4];  /* one quad row's */
    int32_t m[9], pre[3], post[3];
    uint8_t col[4], *yp, *up, *vp;
    int c, i, x, y;

    s->have_frame = false;
    w &= ~1;
    h &= ~1;
    if (!s->pixels || !(s->isp0[TOP_EN] & TOP_EN_ISP) || w < 2 || h < 2 ||
        w > MAX_W || h > MAX_H) {
        i3_isp_grey_stats(s);
        return;
    }
    if (s->out_w != ow || s->out_h != oh) {
        s->raw = g_renew(uint16_t, s->raw, (size_t)w * h);
        s->yuv = g_renew(uint8_t, s->yuv, (size_t)ow * oh * 3);
        s->out_w = ow;
        s->out_h = oh;
    }
    if (!i3_isp_sense(s, cx, cy, w, h)) {
        i3_isp_grey_stats(s);
        return;
    }

    /* per colour: raw 10-bit to the 10-bit Bayer gamma output */
    for (c = 0; c < 4; c++) {
        uint32_t wb0 = i3_isp1(s, WBG0_GAIN + 2 * wbg_reg[c]) & 0x1FFF;
        uint32_t wb1 = sstar_riu_get(s->riu, BANK_ISP6,
                                     WBG1_GAIN + 2 * wbg_reg[c]) & 0x1FFF;
        int g = c == C_B ? 2 : c == C_GB ? 1 : c;

        for (i = 0; i < 1024; i++) {
            uint32_t v = i3_isp_obc(s, c, i << 2);

            if (s->isp0[TOP_EN] & TOP_EN_WBG0) {
                v = MIN((v * wb0) >> 10, 4095);
            }
            if (s->isp0[TOP_EN] & TOP_EN_WBG1) {
                v = MIN((v * wb1) >> 10, 4095);
            }
            lut[c * 1024 + i] = gm12 & 0x1000 ?
                i3_isp_curve(s->gamma[g], v, 4, gm12 & 0x2000 ?
                             gm12 & 0x3FF : s->gamma[g][GM_ENTRIES - 1]) :
                v >> 2;
        }
    }
    for (c = 0; c < 3; c++) {
        for (i = 0; i < 1024; i++) {
            glut[c * 1024 + i] = gm10 & 1 ?
                i3_isp_curve(s->gamma[3 + c], i, 2, gm10 & 2 ?
                             (gm10 >> 2) & 0x3FF :
                             s->gamma[3 + c][GM_ENTRIES - 1]) : i;
        }
    }
    /* the matrix's rows are Cr, Y, Cb; without it, BT.601 full range */
    if (r2y & 1) {
        for (i = 0; i < 9; i++) {
            m[i] = i3_isp_s13(s->isp0[R2Y_COEF + i]);
        }
    } else {
        static const int32_t bt601[9] = { 512, -429, -83, 306, 601, 117,
                                          -173, -339, 512 };

        memcpy(m, bt601, sizeof(m));
        r2y = 0x180;
    }
    pre[0] = r2y & 4 ? 64 : 0;                  /* R, G (the "Y"), B */
    pre[1] = r2y & 2 ? 64 : 0;
    pre[2] = r2y & 8 ? 64 : 0;
    post[0] = (r2y & 0x20 ? 64 : 0) + (r2y & 0x100 ? 512 : 0);     /* Cr */
    post[1] = r2y & 0x10 ? 64 : 0;                                  /* Y */
    post[2] = (r2y & 0x40 ? 64 : 0) + (r2y & 0x80 ? 512 : 0);      /* Cb */

    i3_isp_block_map(s, STS_AE_SIZE, STS_AE_NUM, STS_AE_OFFSET, w, h,
                     aex, aey, &anx, &any);
    i3_isp_block_map(s, STS_AWB_SIZE, STS_AWB_NUM, STS_AWB_OFFSET, w, h,
                     awx, awy, &wnx, &wny);
    memset(s->ae, 0, MAX_BLK * MAX_BLK * 4 * sizeof(uint32_t));
    memset(s->awb, 0, MAX_BLK * MAX_BLK * 4 * sizeof(uint32_t));

    for (i = 0; i < 4; i++) {
        /* where in the quad each colour sits */
        col[i ^ (const uint8_t[]){ 0, 1, 3, 2 }[s->isp0[SEN_ARRAY] & 3]] = i;
    }
    yp = s->yuv;
    up = yp + ow * oh;
    vp = up + ow * oh;
    for (y = 0; y < oh; y++, yp += ow, up += ow, vp += ow) {
        const uint16_t *q0 = s->raw + (size_t)2 * y * w, *q1 = q0 + w;

        /* a quad row like the one above comes out the same */
        if (y && !memcmp(q0, q0 - 2 * w, 2 * w * sizeof(*q0))) {
            memcpy(yp, yp - ow, ow);
            memcpy(up, up - ow, ow);
            memcpy(vp, vp - ow, ow);
        } else {
            memset(rae, 0, anx * 4 * sizeof(*rae));
            memset(rawb, 0, wnx * 4 * sizeof(*rawb));
            for (x = 0; x < ow; x++) {
                uint16_t q[4] = { q0[2 * x], q0[2 * x + 1], q1[2 * x],
                                  q1[2 * x + 1] };
                uint32_t r = q[col[C_R]], gr = q[col[C_GR]];
                uint32_t gb = q[col[C_GB]], b = q[col[C_B]];
                int32_t rgb[3], v[3];
                uint32_t *a;

                if (aex[x] >= 0) {
                    a = rae + aex[x] * 4;
                    a[0] += r;
                    a[1] += gr + gb;
                    a[2] += b;
                    a[3]++;
                }
                if (awx[x] >= 0) {
                    a = rawb + awx[x] * 4;
                    a[0] += r;
                    a[1] += gr + gb;
                    a[2] += b;
                    a[3]++;
                }
                rgb[0] = glut[lut[C_R * 1024 + r]] - pre[0];
                rgb[1] = glut[1024 + ((lut[C_GR * 1024 + gr] +
                                       lut[C_GB * 1024 + gb] + 1) >> 1)] -
                         pre[1];
                rgb[2] = glut[2048 + lut[C_B * 1024 + b]] - pre[2];
                for (c = 0; c < 3; c++) {
                    v[c] = ((m[3 * c] * rgb[0] + m[3 * c + 1] * rgb[1] +
                             m[3 * c + 2] * rgb[2] + 512) >> 10) + post[c];
                    v[c] = (v[c] + 2) >> 2;
                    v[c] = v[c] < 0 ? 0 : v[c] > 255 ? 255 : v[c];
                }
                vp[x] = v[0];
                yp[x] = v[1];
                up[x] = v[2];
            }
        }
        if (aey[y] >= 0) {
            uint32_t *a = s->ae + aey[y] * anx * 4;

            for (i = 0; i < anx * 4; i++) {
                a[i] += rae[i];
            }
        }
        if (awy[y] >= 0) {
            uint32_t *a = s->awb + awy[y] * wnx * 4;

            for (i = 0; i < wnx * 4; i++) {
                a[i] += rawb[i];
            }
        }
    }
    i3_isp_write_stats(s, s->ae, anx * any, 4, STS_AE_BASE, STS_AE_MAX);
    i3_isp_write_stats(s, s->awb, wnx * wny, 3, STS_AWB_BASE, STS_AWB_MAX);
    s->have_frame = true;
    if (s->trace) {
        HisiSoiExposure e;
        uint64_t sum[3] = { 0 };

        hisi_soi_exposure(s->sensor, &e);
        for (i = 0; i < ow * oh; i++) {
            sum[0] += s->yuv[i];
            sum[1] += s->yuv[ow * oh + i];
            sum[2] += s->yuv[2 * ow * oh + i];
        }
        qemu_log("sstar-i3-isp: frame %u: %u rows of %u, gain %u/1024, "
                 "dgain %u, WB %u %u %u %u, mean Y %u Cb %u Cr %u\n",
                 s->frames, e.lines, e.vts, e.gain, i3_isp1(s, AE_DGAIN),
                 sstar_riu_get(s->riu, BANK_ISP6, WBG1_GAIN),
                 sstar_riu_get(s->riu, BANK_ISP6, WBG1_GAIN + 2),
                 sstar_riu_get(s->riu, BANK_ISP6, WBG1_GAIN + 6),
                 sstar_riu_get(s->riu, BANK_ISP6, WBG1_GAIN + 4),
                 (unsigned)(sum[0] / (ow * oh)),
                 (unsigned)(sum[1] / (ow * oh)),
                 (unsigned)(sum[2] / (ow * oh)));
    }
}

bool sstar_i3_isp_frame(SstarI3IspState *s, const uint8_t **y,
                        const uint8_t **cb, const uint8_t **cr, int *w,
                        int *h)
{
    if (!s->have_frame) {
        return false;
    }
    *y = s->yuv;
    *cb = s->yuv + s->out_w * s->out_h;
    *cr = *cb + s->out_w * s->out_h;
    *w = s->out_w;
    *h = s->out_h;
    return true;
}

static void i3_isp_tick(void *opaque)
{
    SstarI3IspState *s = opaque;
    uint16_t cnt = s->isp0[FRM_CNT];

    switch (s->phase) {
    case PHASE_START:
        s->frame_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->raw1 |= G1_VSYNC;
        s->frames++;
        s->isp0[FRM_CNT] = (cnt & ~0x7F) | ((cnt + 1) & 0x7F);
        qemu_irq_pulse(s->frame_out[0]);
        s->phase = PHASE_STATS;
        break;
    case PHASE_STATS:
        i3_isp_process(s);
        s->raw1 |= G1_AE_DONE | G1_AWB_DONE;
        s->raw3 |= G3_AE_ROW | G3_AE_WIN0;
        s->phase = PHASE_END;
        break;
    case PHASE_END:
        s->raw3 |= G3_IN_DONE | G3_OUT_DONE | G3_HIT_LINE3;
        s->isp0[FRM_CNT] = (cnt & ~0x7F00) | ((cnt + 0x100) & 0x7F00);
        s->csi_rpt_raw |= CSI_RPT_VC0;
        qemu_irq_pulse(s->frame_out[1]);
        s->phase = PHASE_START;
        break;
    }
    i3_isp_update(s);
    i3_isp_schedule(s);
}

static uint64_t i3_reg_read(uint16_t *regs, hwaddr off, unsigned size,
                            uint16_t v)
{
    if ((off & 3) >= 2) {
        return 0;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static uint16_t i3_reg_merge(uint16_t old, hwaddr off, uint64_t val,
                             unsigned size)
{
    if (size == 1) {
        return off & 1 ? (old & 0x00FF) | ((val & 0xFF) << 8)
                       : (old & 0xFF00) | (val & 0xFF);
    }
    return val;
}

static uint64_t i3_isp0_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v = s->isp0[reg];

    switch (reg) {
    case G1_FINAL:
        v = s->raw1 & ~s->isp0[G1_MASK];
        break;
    case G1_RAW:
        v = s->raw1;
        break;
    case G2_STATUS:
        v = (s->raw2 & ~s->isp0[G2_MASK_FORCE] & 0xF) |
            ((s->raw2 & 0xF) << 8);
        break;
    case G3_FINAL:
        v = s->raw3 & ~s->isp0[G3_MASK];
        break;
    case G3_RAW:
        v = s->raw3;
        break;
    }
    return i3_reg_read(s->isp0, off, size, v);
}

static void i3_isp0_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    v = i3_reg_merge(s->isp0[reg], off, val, size);
    if (s->trace && reg != G1_CLR && reg != G3_CLR && reg != G2_CLR &&
        reg != TOP_EN && reg != 0x02) {
        qemu_log("sstar-i3-isp: W isp0:%02x = 0x%04x\n", reg, v);
    }
    switch (reg) {
    case G1_FINAL:
    case G1_RAW:
    case G3_FINAL:
    case G3_RAW:
    case G2_STATUS:
        return;                         /* read only */
    case G1_FORCE:
        s->raw1 |= v;
        break;
    case G1_CLR:
        s->raw1 &= ~v;
        break;
    case G2_MASK_FORCE:
        s->raw2 |= (v >> 8) & 0xF;
        break;
    case G2_CLR:
        s->raw2 &= ~(v & 0xF);
        break;
    case G3_FORCE:
        s->raw3 |= v;
        break;
    case G3_CLR:
        s->raw3 &= ~v;
        break;
    case FRM_CNT:
        if (v & 0x8000) {
            v = 0;
        } else {
            v = (s->isp0[reg] & 0x7F7F) | (v & 0x8080);
        }
        break;
    }
    s->isp0[reg] = v;
    i3_isp_update(s);
    i3_isp_schedule(s);
}

static uint64_t i3_isp4_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);

    return i3_reg_read(s->isp4, off, size,
                       reg == ML_SRAM_DATA ? 0 : s->isp4[reg]);
}

static void i3_isp4_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    v = i3_reg_merge(s->isp4[reg], off, val, size);
    if (s->trace) {
        qemu_log("sstar-i3-isp: W isp4:%02x = 0x%04x\n", reg, v);
    }
    if (reg == ML_TRIG) {
        /* the table lands in SRAM at once; the triggers do not stay set */
        if (v & ML_TRIG_REG) {
            i3_isp_mload(s);
        }
        if (v & ML_TRIG_LOAD) {
            s->isp4[ML_CTRL] |= ML_DONE;
        }
        return;
    }
    if (reg == ML_CTRL) {
        /* a new load configuration takes the done flag down */
        v = (v & ~ML_DONE) | (s->isp4[ML_CTRL] & ML_DONE);
        if ((v ^ s->isp4[ML_CTRL]) & ~ML_DONE) {
            v &= ~ML_DONE;
        }
    }
    s->isp4[reg] = v;
}

static uint64_t i3_csi_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v = s->csi[reg];

    switch (reg) {
    case CSI_RPT_SRC:
        v = s->csi_rpt_raw & ~s->csi[CSI_RPT_MASK] & 0x7F;
        break;
    case CSI_RPT_RAW:
        v = s->csi_rpt_raw;
        break;
    case CSI_ERR_SRC:
        v = s->csi_err_raw & ~s->csi[CSI_ERR_MASK] & 0x7FFF;
        break;
    case CSI_ERR_RAW:
        v = s->csi_err_raw;
        break;
    case CSI_MAC_IDLE:
        v |= 1;
        break;
    }
    return i3_reg_read(s->csi, off, size, v);
}

static void i3_csi_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    SstarI3IspState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    v = i3_reg_merge(s->csi[reg], off, val, size);
    switch (reg) {
    case CSI_RPT_SRC:
    case CSI_RPT_RAW:
    case CSI_ERR_SRC:
    case CSI_ERR_RAW:
        return;
    case CSI_RPT_FORCE:
        s->csi_rpt_raw |= v & 0x7F;
        break;
    case CSI_RPT_CLR:
        s->csi_rpt_raw &= ~v;
        break;
    case CSI_ERR_FORCE:
        s->csi_err_raw |= v & 0x7FFF;
        break;
    case CSI_ERR_CLR:
        s->csi_err_raw &= ~v;
        break;
    }
    s->csi[reg] = v;
    i3_isp_update(s);
}

#define I3_OPS(name)                                                    \
    static const MemoryRegionOps name##_ops = {                         \
        .read = name##_read,                                            \
        .write = name##_write,                                          \
        .endianness = DEVICE_LITTLE_ENDIAN,                             \
        .impl.min_access_size = 1,                                      \
        .impl.max_access_size = 2,                                      \
        .valid.min_access_size = 1,                                     \
        .valid.max_access_size = 4,                                     \
    }

I3_OPS(i3_isp0);
I3_OPS(i3_isp4);
I3_OPS(i3_csi);

static void sstar_i3_isp_reset(DeviceState *dev)
{
    SstarI3IspState *s = SSTAR_I3_ISP(dev);
    uint16_t ramp[GM_ENTRIES];
    int i;

    memset(s->isp0, 0, sizeof(s->isp0));
    memset(s->isp4, 0, sizeof(s->isp4));
    memset(s->csi, 0, sizeof(s->csi));
    s->isp0[G1_MASK] = 0xFFFF;
    s->isp0[G2_MASK_FORCE] = 0xF;
    s->isp0[G3_MASK] = 0xFFFF;
    s->csi[CSI_RPT_MASK] = 0x7F;
    s->csi[CSI_ERR_MASK] = 0x7FFF;
    s->raw1 = s->raw2 = s->raw3 = 0;
    s->csi_rpt_raw = s->csi_err_raw = 0;
    s->phase = PHASE_START;
    s->have_frame = false;
    for (i = 0; i < GM_ENTRIES; i++) {
        ramp[i] = i * 4;                /* linear until a menuload */
    }
    for (i = 0; i < 6; i++) {
        memcpy(s->gamma[i], ramp, sizeof(ramp));
    }
    timer_del(s->timer);
}

static void sstar_i3_isp_realize(DeviceState *dev, Error **errp)
{
    SstarI3IspState *s = SSTAR_I3_ISP(dev);

    if (!s->riu) {
        error_setg(errp, "sstar-i3-isp: the riu link is not set");
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, i3_isp_tick, s);
    s->idx = g_malloc(3 * (MAX_W + 1));
    s->ae = g_new(uint32_t, MAX_BLK * MAX_BLK * 4);
    s->awb = g_new(uint32_t, MAX_BLK * MAX_BLK * 4);
}

static void sstar_i3_isp_init(Object *obj)
{
    SstarI3IspState *s = SSTAR_I3_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->isp0_mem, obj, &i3_isp0_ops, s,
                          "sstar-i3-isp.isp0", 0x200);
    memory_region_init_io(&s->isp4_mem, obj, &i3_isp4_ops, s,
                          "sstar-i3-isp.isp4", 0x200);
    memory_region_init_io(&s->csi_mem, obj, &i3_csi_ops, s,
                          "sstar-i3-isp.csi", 0x200);
    sysbus_init_mmio(sbd, &s->isp0_mem);
    sysbus_init_mmio(sbd, &s->isp4_mem);
    sysbus_init_mmio(sbd, &s->csi_mem);
    sysbus_init_irq(sbd, &s->isp_irq);
    sysbus_init_irq(sbd, &s->csi_irq);
    qdev_init_gpio_out(DEVICE(obj), s->frame_out, 2);
    /* the scene's light, which may change while running (qom-set) */
    s->light = 1700;
    s->light_r = 820;
    s->light_b = 400;
    object_property_add_uint32_ptr(obj, "light", &s->light,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-r", &s->light_r,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-b", &s->light_b,
                                   OBJ_PROP_FLAG_READWRITE);
    /* set by the board once the sensor exists, after realize */
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
}

static const Property sstar_i3_isp_props[] = {
    DEFINE_PROP_UINT32("fps", SstarI3IspState, fps, 30),
    DEFINE_PROP_BOOL("pixels", SstarI3IspState, pixels, true),
    DEFINE_PROP_BOOL("trace", SstarI3IspState, trace, false),
    DEFINE_PROP_LINK("riu", SstarI3IspState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
};

static const VMStateDescription vmstate_sstar_i3_isp = {
    .name = TYPE_SSTAR_I3_ISP,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(isp0, SstarI3IspState, BANK_REGS),
        VMSTATE_UINT16_ARRAY(isp4, SstarI3IspState, BANK_REGS),
        VMSTATE_UINT16_ARRAY(csi, SstarI3IspState, BANK_REGS),
        VMSTATE_UINT16(raw1, SstarI3IspState),
        VMSTATE_UINT16(raw2, SstarI3IspState),
        VMSTATE_UINT16(raw3, SstarI3IspState),
        VMSTATE_UINT16(csi_rpt_raw, SstarI3IspState),
        VMSTATE_UINT16(csi_err_raw, SstarI3IspState),
        VMSTATE_UINT8(phase, SstarI3IspState),
        VMSTATE_INT64(frame_start, SstarI3IspState),
        VMSTATE_TIMER_PTR(timer, SstarI3IspState),
        VMSTATE_UINT16_2DARRAY(gamma, SstarI3IspState, 6, GM_ENTRIES),
        VMSTATE_UINT32(frames, SstarI3IspState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i3_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_i3_isp_realize;
    device_class_set_legacy_reset(dc, sstar_i3_isp_reset);
    device_class_set_props(dc, sstar_i3_isp_props);
    dc->vmsd = &vmstate_sstar_i3_isp;
}

static const TypeInfo sstar_i3_isp_info = {
    .name           = TYPE_SSTAR_I3_ISP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI3IspState),
    .instance_init  = sstar_i3_isp_init,
    .class_init     = sstar_i3_isp_class_init,
};

static void sstar_i3_isp_register_types(void)
{
    type_register_static(&sstar_i3_isp_info);
}

type_init(sstar_i3_isp_register_types)
