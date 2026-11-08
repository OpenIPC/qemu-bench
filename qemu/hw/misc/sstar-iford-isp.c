/*
 * SigmaStar Iford image pipe: sensor to the scaler's input, in one piece.
 *
 * The sensor (the "sensor" link, its exposure rows and total gain) looks
 * at sstar-pattern.h's scene with the wedge of neutral patches
 * ingenic-isp-paint.h adds, on a neutral band as wide as the picture: the
 * vendor's AWB wants many unclipped neutral blocks, and the navy and the logo
 * give it none (with the patches alone it never moves off its 5000 K gains).  Each frame, on the "frame" input
 * (sstar-isp's clock):
 *
 *   Bayer      12 bits, BGGR, at the sensor's window: reflectance x light x
 *              exposure, clipped.  The light is "light" (mLSB of full scale
 *              per row at gain 1 on a white, under G) with "light-r" and
 *              "light-b", per mille of G; their defaults, 490 and 580, are
 *              what the vendor's IQ file takes for 5000 K on this sensor
 *              (its AWB gains there are 2089/1024 and 1765/1024).  All
 *              three may change while running: qom-set /machine/isp.
 *   stats      below, from that Bayer data
 *   WBG        bank 0x1312 0x40 bit 0, gains 3.10: 0x41 R, 0x44 Gr, 0x47 Gb,
 *              0x4A B (from HalIspIqWbgCfg and a trace of the running AWB)
 *   demosaic   each 2x2 quad gives one RGB pixel: half the window
 *   gamma      STUB: a fixed 1/2.2.  The IQ file's curves are not read.
 *   R2Y        BT.601 limited range, fixed.  The IQ's CCM is not applied.
 *
 * Not modelled: black level, LSC, noise reduction, sharpening, and any
 * register of the image pipe but those above.  What leaves here goes to the
 * scaler through sstar_iford_isp_picture().
 *
 * The statistics.  The AE and AWB are the vendor's own, in userland
 * (libcus3a, libispalgo): each frame they read what the statistics DMA left
 * in ISP0_STATS.  From mi_isp.ko's HalIspHwStatistics*, HalIspAe*,
 * HalIspAwb*, HalIspHisto* the registers are bank 0x131B, and the buffer,
 * for a set of 0x14240 bytes (the driver alternates two):
 *
 *   +0x0000 AE, 4 bytes a block: mean R, G, B, Y   (base 0x08/0x09)
 *   +0x1000 AWB, 3 bytes a block: mean R, G, B     (base 0x0A/0x0B)
 *   +0x9700 histogram 0, +0x9900 histogram 1, 128 16-bit bins: blocks by
 *           their Y / 2                            (base 0x0C/0x0D)
 *
 * Bases are MIU addresses >> 4, low 16 bits then the rest.  0x02 enables:
 * bit 0 AE, bit 4 AWB, bit 8 the histograms.  0x30/0x31 the AE block size
 * - 1, 0x32/0x33 blocks - 1 (32 x 32 of 80 x 45 at 2560x1440); AWB 0x40-0x43
 * the same (128 x 90 of 20 x 16); the histograms 0x61-0x64, here always
 * AWB's grid, which is what the driver sets, so they count 128 x 90 blocks.
 * Their window registers are not read.  Means are of the WB-free data as 8
 * bits (12 bits >> 4), and Y is 0.30 R + 0.59 G + 0.11 B.
 *
 * VIF's write DMA, when the pipeline binds VIF to the ISP frame by frame
 * (mi_vif.ko HAL_VIF_IF_Wdma*, HAL_VIF_IspDma*).  Channel 0's scatter DMA is
 * bank 0x1358:
 *
 *   0x00 bit 0 enable, bit 4 push 0x06/0x07 on to the FIFO, bit 5 clear
 *   0x04 [14:0] pitch, 16-byte units   0x06/0x07 the buffer, MIU >> 4
 *   0x0C [13:8] selects what 0x0E/0x0F read: 0 the last buffer done, 1
 *        [2:0] the FIFO's count, 3 the buffer in hand, 4 and 6 the time,
 *        5 the write pointer
 *   0x10 low byte clears events, high byte reads them
 *
 * Bank 0x130C, SPI 56, four groups of mask, force, clear, status at 0x00,
 * 0x04, 0x10, 0x14: bit 0 of the last is the frame's start, of the second
 * its line-count hit, of the first its end.  Each frame with a buffer queued the Bayer data goes there, 10 bits a
 * pixel, four to five bytes, least significant first.  That packing is a
 * guess: the ISP here never reads it back, it works from the sensor's frame
 * directly, RDMA or no RDMA.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/image-sensor.h"
#include "hw/irq.h"
#include "hw/misc/sstar-iford-isp.h"
#include "hw/misc/sstar-riu.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include <math.h>
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "ingenic-isp-paint.h"

OBJECT_DECLARE_SIMPLE_TYPE(SstarIfordIspState, SSTAR_IFORD_ISP)

#define BANK_STATS          0x131B
#define STS_ENABLE          0x02
#define STS_AE_BASE         0x08
#define STS_AWB_BASE        0x0A
#define STS_HISTO_BASE      0x0C
#define STS_AE_SIZE_X       0x30
#define STS_AE_SIZE_Y       0x31
#define STS_AE_BLOCKS_X     0x32
#define STS_AE_BLOCKS_Y     0x33
#define STS_AWB_SIZE_X      0x40
#define STS_AWB_SIZE_Y      0x41
#define STS_AWB_BLOCKS_X    0x42
#define STS_AWB_BLOCKS_Y    0x43
#define EN_AE               (1u << 0)
#define EN_AWB              (1u << 4)
#define EN_HISTO            (1u << 8)

#define BANK_WBG            0x1312
#define WBG_ENABLE          0x40
#define WBG_R               0x41
#define WBG_GR              0x44
#define WBG_GB              0x47
#define WBG_B               0x4A

#define HISTO1_OFF          0x200
#define HISTO_BINS          128

/* before the driver has configured the grids */
#define ISP_AE_GRID         32
#define ISP_DEF_W           2560
#define ISP_DEF_H           1440
#define ISP_MAX_DIM         8192

#define ISP_DRAM_BASE       0x20000000
#define ISP_PAGE            4096
#define ISP_FULL            4095
#define ISP_NO_DOT          (-100000)

/* reflectance 1.0 = 1 << REFL_SHIFT; the logo and the dot are an 80 % white */
#define REFL_SHIFT          14
#define SC_WHITE            0.8
#define SC_FLOOR            0.02        /* the wedge's row, about the patches */

#define WD_BANK_REGS        0x80
#define WD_CTRL             0x00
#define  WD_EN              (1u << 0)
#define  WD_PUSH            (1u << 4)
#define WD_PITCH            0x04
#define WD_ADDR             0x06
#define WD_INFO_SEL         0x0C
#define WD_INFO_LO          0x0E
#define WD_INFO_HI          0x0F
#define WD_FIFO             5

#define DI_GROUPS           4
#define DI_MASK(g)          ((g) * 4 + ((g) >= 2 ? 8 : 0))
#define DI_CLEAR(g)         (DI_MASK(g) + 2)
#define DI_STATUS(g)        (DI_MASK(g) + 3)
#define DI_END              0           /* group: the frame's end */
#define DI_LINE             1           /* group: its line-count hit */
#define DI_START            3           /* group: its start */

enum { C_R, C_G, C_B };

struct SstarIfordIspState {
    SysBusDevice parent_obj;

    Object *sensor;
    SstarRiuState *riu;
    uint32_t light, light_r, light_b;
    bool trace;
    uint32_t frames;

    int w, h;                       /* the sensor's window, even */
    uint16_t *refl;                 /* w x h: its reflectance, no dot */
    uint16_t *bayer;                /* w x h: this frame */
    uint8_t *pic_y, *pic_uv;        /* w/2 x h/2 and w/2 x h/4 */
    uint8_t gamma[ISP_FULL + 1];

    MemoryRegion wd_mem, di_mem;
    qemu_irq di_irq;
    uint16_t wd[WD_BANK_REGS], di[WD_BANK_REGS];
    uint32_t fifo[WD_FIFO];
    int fifo_n;
    uint32_t done;                  /* the last buffer done, MIU >> 4 */
    bool sensor_window;
    bool ae_grid_32;
};

/* the site's colour: BGGR */
static inline int isp_site(int x, int y)
{
    return (y & 1) ? ((x & 1) ? C_R : C_G) : ((x & 1) ? C_G : C_B);
}

static void isp_write_mem(uint64_t miu, const void *buf, size_t len)
{
    const uint8_t *p = buf;

    while (len) {
        size_t n = MIN(len, ISP_PAGE - (miu & (ISP_PAGE - 1)));

        dma_memory_write(&address_space_memory,
                         ISP_DRAM_BASE + sstar_miu_xlate(miu), p, n,
                         MEMTXATTRS_UNSPECIFIED);
        miu += n;
        p += n;
        len -= n;
    }
}

/* the navy at row @y of @h, taken back through BT.601 and a 2.2 gamma */
static void isp_scene_navy(int y, int h, double *rgb)
{
    int py = (int64_t)y * SSTAR_PAT_H / MAX(h, 1);
    double yy = (50 - 14.0 * py / SSTAR_PAT_H - 16) * 255 / 219;
    double cb = (146 - 128) * 255 / 224.0, cr = (122 - 128) * 255 / 224.0;
    double v[3] = { yy + 1.402 * cr, yy - 0.344 * cb - 0.714 * cr,
                    yy + 1.772 * cb };
    int c;

    for (c = 0; c < 3; c++) {
        rgb[c] = SC_WHITE * pow(MAX(v[c], 0) / 255, 2.2);
    }
}

/* the reflectance of (x, y) in colour @c, the dot where we are told */
static uint16_t isp_scene_site(int dot, int w, int h, int x, int y, int c,
                               const double *navy)
{
    unsigned a = sstar_pattern_cover(dot, w, h, x, y);
    double v = navy[c] + (SC_WHITE - navy[c]) * a / 255.0;
    int i;

    if (isp_wedge_row(h, y)) {
        i = isp_wedge_patch(w, x);
        v = i >= 0 ? SC_WHITE * pow((60 + 30 * i - 16) / 219.0, 2.2) : SC_FLOOR;
    }
    return lrint(v * (1 << REFL_SHIFT));
}

static void isp_window(SstarIfordIspState *s, int w, int h)
{
    int x, y;

    w &= ~1;
    h &= ~1;
    if (s->w == w && s->h == h) {
        return;
    }
    g_free(s->refl);
    g_free(s->bayer);
    g_free(s->pic_y);
    g_free(s->pic_uv);
    s->w = w;
    s->h = h;
    s->refl = g_malloc(w * h * sizeof(*s->refl));
    s->bayer = g_malloc(w * h * sizeof(*s->bayer));
    s->pic_y = g_malloc0(w / 2 * (h / 2));
    s->pic_uv = g_malloc0(w / 2 * (h / 2));
    for (y = 0; y < h; y++) {
        double navy[3];

        isp_scene_navy(y, h, navy);
        for (x = 0; x < w; x++) {
            s->refl[y * w + x] = isp_scene_site(ISP_NO_DOT, w, h, x, y,
                                                isp_site(x, y), navy);
        }
    }
}

/* the sensor: this frame's Bayer data, the dot in it */
static void isp_sensor(SstarIfordIspState *s, const ImageSensorExposure *e)
{
    int w = s->w, h = s->h;
    double expo = (double)e->lines * e->gain / 1024 * s->light / 1e6;
    uint32_t mul[3];
    int dot = sstar_pattern_dot_x(s->frames);
    int dx0 = MAX((dot - SSTAR_PAT_DOT_R) * w / SSTAR_PAT_W - 2, 0);
    int dx1 = MIN((dot + SSTAR_PAT_DOT_R) * w / SSTAR_PAT_W + 2, w - 1);
    int dy0 = MAX((SSTAR_PAT_DOT_Y - SSTAR_PAT_DOT_R) * h / SSTAR_PAT_H - 2, 0);
    int dy1 = MIN((SSTAR_PAT_DOT_Y + SSTAR_PAT_DOT_R) * h / SSTAR_PAT_H + 2,
                  h - 1);
    int x, y;

    /* 12 bits of output per unit reflectance, as 16.16 of a 2.14 input */
    mul[C_R] = MIN(expo * s->light_r / 1000 * ISP_FULL * 4, UINT32_MAX);
    mul[C_G] = MIN(expo * ISP_FULL * 4, UINT32_MAX);
    mul[C_B] = MIN(expo * s->light_b / 1000 * ISP_FULL * 4, UINT32_MAX);
    for (y = 0; y < h; y++) {
        const uint16_t *in = s->refl + y * w;
        uint16_t *out = s->bayer + y * w;
        uint32_t m0 = mul[isp_site(0, y)], m1 = mul[isp_site(1, y)];

        for (x = 0; x + 1 < w; x += 2) {
            uint64_t a = ((uint64_t)in[x] * m0) >> 16;
            uint64_t b = ((uint64_t)in[x + 1] * m1) >> 16;

            out[x] = a > ISP_FULL ? ISP_FULL : a;
            out[x + 1] = b > ISP_FULL ? ISP_FULL : b;
        }
    }
    for (y = dy0; y <= dy1; y++) {
        double navy[3];

        isp_scene_navy(y, h, navy);
        for (x = dx0; x <= dx1; x++) {
            int c = isp_site(x, y);
            uint64_t v = isp_scene_site(dot, w, h, x, y, c, navy);

            s->bayer[y * w + x] = MIN((v * mul[c]) >> 16, ISP_FULL);
        }
    }
}

static uint64_t isp_base(SstarIfordIspState *s, int reg)
{
    return (((uint64_t)sstar_riu_get(s->riu, BANK_STATS, reg + 1) << 16) |
            sstar_riu_get(s->riu, BANK_STATS, reg)) << 4;
}

/* the means of an nx x ny grid of the quads, as 8 bits, R G B a block */
static void isp_means(SstarIfordIspState *s, int nx, int ny, uint8_t *out,
                      int stride)
{
    int qw = s->w / 2, qh = s->h / 2, x, y;
    g_autofree uint32_t *sum = g_malloc0(nx * ny * 3 * sizeof(*sum));
    g_autofree uint16_t *map = g_malloc(qw * sizeof(*map));

    for (x = 0; x < qw; x++) {
        map[x] = (uint64_t)x * nx / qw;
    }
    for (y = 0; y < qh; y++) {
        int by = (uint64_t)y * ny / qh;
        const uint16_t *r0 = s->bayer + 2 * y * s->w, *r1 = r0 + s->w;
        uint32_t *row = sum + by * nx * 3;

        for (x = 0; x < qw; x++) {
            uint32_t *b = row + map[x] * 3;

            b[C_B] += r0[2 * x];
            b[C_G] += r0[2 * x + 1] + r1[2 * x];
            b[C_R] += r1[2 * x + 1];
        }
    }
    for (y = 0; y < ny; y++) {
        for (x = 0; x < nx; x++) {
            /* quads in this block */
            uint32_t n = ((x + 1) * qw / nx - x * qw / nx) *
                         ((y + 1) * qh / ny - y * qh / ny);
            const uint32_t *b = sum + (y * nx + x) * 3;
            uint8_t *o = out + (y * nx + x) * stride;

            n = MAX(n, 1);
            o[0] = b[C_R] / n >> 4;
            o[1] = b[C_G] / (2 * n) >> 4;
            o[2] = b[C_B] / n >> 4;
        }
    }
}

static inline uint8_t isp_luma(const uint8_t *rgb)
{
    return (30 * rgb[0] + 59 * rgb[1] + 11 * rgb[2]) / 100;
}

/*
 * The infinity6 family has no block count: HalIspAeSetWin writes the size at
 * 0x30/0x31 and an offset at 0x34/0x35 of a fixed 32 x 32 grid, which is how
 * they add up to the sensor's window on all three dies.
 */
static int ae_blocks(SstarIfordIspState *s, int reg)
{
    if (s->ae_grid_32) {
        return ISP_AE_GRID;
    }
    return (sstar_riu_get(s->riu, BANK_STATS, reg) & 0x7F) + 1;
}

static void isp_stats(SstarIfordIspState *s, const ImageSensorExposure *e)
{
    uint16_t en = sstar_riu_get(s->riu, BANK_STATS, STS_ENABLE);
    int aex = ae_blocks(s, STS_AE_BLOCKS_X);
    int aey = ae_blocks(s, STS_AE_BLOCKS_Y);
    int awx = (sstar_riu_get(s->riu, BANK_STATS, STS_AWB_BLOCKS_X) & 0xFF) + 1;
    int awy = (sstar_riu_get(s->riu, BANK_STATS, STS_AWB_BLOCKS_Y) & 0x7F) + 1;
    int64_t sum = 0;
    int i;

    if ((en & EN_AE) && isp_base(s, STS_AE_BASE)) {
        g_autofree uint8_t *ae = g_malloc(aex * aey * 4);

        isp_means(s, aex, aey, ae, 4);
        for (i = 0; i < aex * aey; i++) {
            ae[i * 4 + 3] = isp_luma(ae + i * 4);
            sum += ae[i * 4 + 3];
        }
        isp_write_mem(isp_base(s, STS_AE_BASE), ae, aex * aey * 4);
    }
    if ((en & (EN_AWB | EN_HISTO))) {
        g_autofree uint8_t *awb = g_malloc(awx * awy * 3);

        isp_means(s, awx, awy, awb, 3);
        if ((en & EN_AWB) && isp_base(s, STS_AWB_BASE)) {
            isp_write_mem(isp_base(s, STS_AWB_BASE), awb, awx * awy * 3);
        }
        if ((en & EN_HISTO) && isp_base(s, STS_HISTO_BASE)) {
            uint16_t bins[HISTO_BINS] = { 0 };

            for (i = 0; i < awx * awy; i++) {
                bins[isp_luma(awb + i * 3) / 2]++;
            }
            isp_write_mem(isp_base(s, STS_HISTO_BASE), bins, sizeof(bins));
            isp_write_mem(isp_base(s, STS_HISTO_BASE) + HISTO1_OFF, bins,
                          sizeof(bins));
        }
    }
    if (s->trace) {
        qemu_log("sstar-iford-isp: frame %u en 0x%04x AE %dx%d @0x%"PRIx64
                 " light %u lines %u gain %u WB %u/%u mean Y %"PRId64"\n",
                 s->frames, en, aex, aey, isp_base(s, STS_AE_BASE), s->light,
                 e->lines, e->gain, sstar_riu_get(s->riu, BANK_WBG, WBG_R),
                 sstar_riu_get(s->riu, BANK_WBG, WBG_B),
                 sum / (aex * aey));
    }
}

/* WBG, the quads as RGB, gamma, R2Y */
static void isp_picture(SstarIfordIspState *s)
{
    int qw = s->w / 2, qh = s->h / 2, x, y;
    bool wb = sstar_riu_get(s->riu, BANK_WBG, WBG_ENABLE) & 1;
    uint32_t gr = wb ? sstar_riu_get(s->riu, BANK_WBG, WBG_R) : 1024;
    uint32_t gg = wb ? (sstar_riu_get(s->riu, BANK_WBG, WBG_GR) +
                        sstar_riu_get(s->riu, BANK_WBG, WBG_GB)) / 2 : 1024;
    uint32_t gb = wb ? sstar_riu_get(s->riu, BANK_WBG, WBG_B) : 1024;

    for (y = 0; y < qh; y++) {
        const uint16_t *r0 = s->bayer + 2 * y * s->w, *r1 = r0 + s->w;
        uint8_t *oy = s->pic_y + y * qw;
        uint8_t *ouv = s->pic_uv + (y / 2) * qw;

        for (x = 0; x < qw; x++) {
            uint32_t b = r0[2 * x] * gb >> 10;
            uint32_t g = (r0[2 * x + 1] + r1[2 * x]) * gg >> 11;
            uint32_t r = r1[2 * x + 1] * gr >> 10;
            int cr, cg, cb;

            cr = s->gamma[MIN(r, ISP_FULL)];
            cg = s->gamma[MIN(g, ISP_FULL)];
            cb = s->gamma[MIN(b, ISP_FULL)];
            oy[x] = 16 + ((66 * cr + 129 * cg + 25 * cb + 128) >> 8);
            if (!(y & 1) && !(x & 1) && x + 1 < qw) {
                ouv[x] = 128 + ((-38 * cr - 74 * cg + 112 * cb + 128) >> 8);
                ouv[x + 1] = 128 + ((112 * cr - 94 * cg - 18 * cb + 128) >> 8);
            }
        }
    }
}

static void isp_di_update(SstarIfordIspState *s)
{
    bool level = false;
    int g;

    for (g = 0; g < DI_GROUPS; g++) {
        level |= !!(s->di[DI_STATUS(g)] & ~s->di[DI_MASK(g)]);
    }
    qemu_set_irq(s->di_irq, level);
}

/* the Bayer frame through VIF's write DMA, if it has a buffer */
static void isp_vif_dma(SstarIfordIspState *s)
{
    uint32_t pitch = (s->wd[WD_PITCH] & 0x7FFF) * 16, base;
    size_t row = s->w / 4 * 5;
    g_autofree uint8_t *line = NULL;
    int x, y;

    if (!(s->wd[WD_CTRL] & WD_EN)) {
        return;
    }
    s->di[DI_STATUS(DI_START)] |= 1;
    s->di[DI_STATUS(DI_LINE)] |= 1;
    if (s->fifo_n && pitch >= row) {
        base = s->fifo[0];
        memmove(s->fifo, s->fifo + 1, --s->fifo_n * sizeof(s->fifo[0]));
        line = g_malloc(row);
        for (y = 0; y < s->h; y++) {
            const uint16_t *in = s->bayer + y * s->w;
            uint8_t *o = line;

            for (x = 0; x + 3 < s->w; x += 4, o += 5) {
                uint64_t v = (uint64_t)(in[x] >> 2) |
                             (uint64_t)(in[x + 1] >> 2) << 10 |
                             (uint64_t)(in[x + 2] >> 2) << 20 |
                             (uint64_t)(in[x + 3] >> 2) << 30;

                o[0] = v;
                o[1] = v >> 8;
                o[2] = v >> 16;
                o[3] = v >> 24;
                o[4] = v >> 32;
            }
            isp_write_mem(((uint64_t)base << 4) + y * (uint64_t)pitch, line,
                          row);
        }
        s->done = base;
        s->di[DI_STATUS(DI_END)] |= 1;
    }
    isp_di_update(s);
}

static uint32_t isp_wd_info(SstarIfordIspState *s)
{
    switch ((s->wd[WD_INFO_SEL] >> 8) & 0x3F) {
    case 0:
        return s->done;
    case 1:
        return s->fifo_n;
    case 3:
    case 5:
        return s->fifo_n ? s->fifo[0] : s->done;
    case 4:
    case 6:
        return qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    default:
        return 0;
    }
}

static uint64_t isp_wd_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIfordIspState *s = opaque;
    int r = (off >> 2) & (WD_BANK_REGS - 1);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    if (r == WD_INFO_LO) {
        v = isp_wd_info(s);
    } else if (r == WD_INFO_HI) {
        v = isp_wd_info(s) >> 16;
    } else {
        v = s->wd[r];
    }
    return off & 1 ? v >> 8 : (size == 1 ? (v & 0xFF) : v);
}

static void isp_wd_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarIfordIspState *s = opaque;
    int r = (off >> 2) & (WD_BANK_REGS - 1);
    uint16_t v = s->wd[r];

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (v & 0x00FF) | (val & 0xFF) << 8
                    : (v & 0xFF00) | (val & 0xFF);
    } else {
        v = val;
    }
    if (r == WD_CTRL && (v & WD_PUSH)) {
        v &= ~WD_PUSH;
        if (s->fifo_n < WD_FIFO) {
            s->fifo[s->fifo_n++] = s->wd[WD_ADDR] |
                                   (uint32_t)s->wd[WD_ADDR + 1] << 16;
        }
    }
    s->wd[r] = v;
}

static const MemoryRegionOps isp_wd_ops = {
    .read = isp_wd_read,
    .write = isp_wd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static uint64_t isp_di_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIfordIspState *s = opaque;
    uint16_t v = s->di[(off >> 2) & (WD_BANK_REGS - 1)];

    if ((off & 3) >= 2) {
        return 0;
    }
    return off & 1 ? v >> 8 : (size == 1 ? (v & 0xFF) : v);
}

static void isp_di_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarIfordIspState *s = opaque;
    int r = (off >> 2) & (WD_BANK_REGS - 1);
    uint16_t v = s->di[r];
    int g;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (v & 0x00FF) | (val & 0xFF) << 8
                    : (v & 0xFF00) | (val & 0xFF);
    } else {
        v = val;
    }
    for (g = 0; g < DI_GROUPS; g++) {
        if (r == DI_STATUS(g)) {
            return;
        }
        if (r == DI_CLEAR(g)) {
            s->di[DI_STATUS(g)] &= ~v;
            isp_di_update(s);
            return;
        }
    }
    s->di[r] = v;
    isp_di_update(s);
}

static const MemoryRegionOps isp_di_ops = {
    .read = isp_di_read,
    .write = isp_di_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void isp_frame(SstarIfordIspState *s)
{
    ImageSensorExposure e = { .lines = 1, .gain = 1024 };
    int w = ((sstar_riu_get(s->riu, BANK_STATS, STS_AE_SIZE_X) & 0x1FFF) + 1) *
            ae_blocks(s, STS_AE_BLOCKS_X);
    int h = ((sstar_riu_get(s->riu, BANK_STATS, STS_AE_SIZE_Y) & 0x1FFF) + 1) *
            ae_blocks(s, STS_AE_BLOCKS_Y);

    if (w < 64 || h < 64 || w > ISP_MAX_DIM || h > ISP_MAX_DIM) {
        w = ISP_DEF_W;
        h = ISP_DEF_H;
    }
    image_sensor_exposure(s->sensor, &e);
    /* infinity6e, infinity6c: the sensor gives the window */
    if (s->sensor_window && e.width && e.height) {
        w = MIN(e.width / 2, ISP_MAX_DIM) & ~1;
        h = MIN(e.height / 2, ISP_MAX_DIM) & ~1;
    }
    isp_window(s, w, h);
    isp_sensor(s, &e);
    isp_vif_dma(s);
    isp_stats(s, &e);
    isp_picture(s);
    s->frames++;
}

void sstar_iford_isp_frame(Object *isp)
{
    isp_frame(SSTAR_IFORD_ISP(isp));
}

void sstar_iford_isp_picture(Object *isp, const uint8_t **y,
                             const uint8_t **uv, int *w, int *h)
{
    SstarIfordIspState *s = SSTAR_IFORD_ISP(isp);

    if (!s->refl) {
        isp_frame(s);
    }
    *y = s->pic_y;
    *uv = s->pic_uv;
    *w = s->w / 2;
    *h = s->h / 2;
}

static void isp_frame_in(void *opaque, int n, int level)
{
    SstarIfordIspState *s = opaque;

    if (level) {
        isp_frame(s);
    }
}

static void isp_init(Object *obj)
{
    SstarIfordIspState *s = SSTAR_IFORD_ISP(obj);
    int i;

    qdev_init_gpio_in(DEVICE(obj), isp_frame_in, 1);
    memory_region_init_io(&s->wd_mem, obj, &isp_wd_ops, s, "sstar-vif-wdma",
                          WD_BANK_REGS * 4);
    memory_region_init_io(&s->di_mem, obj, &isp_di_ops, s, "sstar-vif-dmaint",
                          WD_BANK_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->wd_mem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->di_mem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->di_irq);
    s->light = 8000;
    s->light_r = 490;
    s->light_b = 580;
    for (i = 0; i <= ISP_FULL; i++) {
        s->gamma[i] = lrint(255 * pow(i / (double)ISP_FULL, 1 / 2.2));
    }
    object_property_add_uint32_ptr(obj, "light", &s->light,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-r", &s->light_r,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-b", &s->light_b,
                                   OBJ_PROP_FLAG_READWRITE);
}

static const Property isp_props[] = {
    DEFINE_PROP_LINK("sensor", SstarIfordIspState, sensor, TYPE_OBJECT,
                     Object *),
    DEFINE_PROP_LINK("riu", SstarIfordIspState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
    DEFINE_PROP_BOOL("trace", SstarIfordIspState, trace, false),
    DEFINE_PROP_BOOL("sensor-window", SstarIfordIspState, sensor_window, false),
    DEFINE_PROP_BOOL("ae-grid-32", SstarIfordIspState, ae_grid_32, false),
};

static void isp_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_props(DEVICE_CLASS(oc), isp_props);
}

static const TypeInfo isp_info = {
    .name = TYPE_SSTAR_IFORD_ISP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarIfordIspState),
    .instance_init = isp_init,
    .class_init = isp_class_init,
};

static void isp_register(void)
{
    type_register_static(&isp_info);
}

type_init(isp_register)
