/*
 * Rockchip VEPU540C ("rockchip,rkv-encoder-rv1106", 0xFFA50000), the
 * RV1106's H.264 / H.265 / JPEG encoder, as the RV1106 SDK's kernel MPP
 * drives it (kmpp: mpp/mpp_rkvenc_540c.c and vcodec/mpp/hal/rkenc/h265e/
 * hal_h265e_vepu540c.c).  Without "rockchip,task-capacity" in the
 * devicetree the driver runs one frame at a time: it writes the register
 * set, then ENC_START.
 *
 *   0x010 ENC_START   bits 9:8 command (1: encode one frame), 7:0 the
 *                     link table's task count
 *   0x020 INT_EN  0x024 INT_MSK  0x028 INT_CLR  0x02c INT_STA
 *                     bit 0 one frame finished
 *   0x280 / 0x284     source Y / CbCr       0x334 / 0x338 their strides
 *   0x28c / 0x294     reconstruction write / read (the reference)
 *   0x2b0 BSBT  0x2b4 BSBB  0x2b8 BSBS  0x2bc BSBR   the stream buffer:
 *                     top, bottom, where to write, where the reader is
 *   0x300 ENC_PIC     bits 1:0 standard (1 H.265), 13:8 picture QP
 *   0x310 ENC_RSL     width / 8 - 1, height / 8 - 1 << 16
 *   0x318 SRC_FMT     bits 5:2 format (6 NV12)
 *   0x3b0 SYNT_NAL    nal_unit_type    0x3bc SYNT_SLI0 bits 16:15 type
 *   0x3c0 SYNT_SLI1   bits 21:16 slice QP   0x3c4 SYNT_SLI2 POC LSB
 *   0x47c JPEG_BASE_CFG  bit 31 JPEG
 *   0x4000 ST_BS_LGTH (bytes written)  0x400c ST_QP_SUM
 *
 * The hardware writes the slice NAL units (start code, slice header from
 * the SYNT registers, slice data); the HAL writes the VPS / SPS / PPS in
 * software just before BSBS on an IDR.  The model parses those
 * (hevc_enc_parse_headers) and codes each picture with the shared
 * encoder (hevc-enc.c): one slice, the header those parameter sets call
 * for, intra DC I pictures and P pictures that skip what did not change.
 * The reference a P picture codes against is the reconstruction the
 * job before it wrote at the address this one reads (two channels keep
 * their own).  Without parameter sets seen, an IDR carries ours in front.
 *
 * JPEG (JPEG_BASE_CFG bit 31, alongside a video frame or alone) has a
 * register set of its own: 0x400 BSBT  0x404 BSBB  0x408 BSBR
 * 0x40c BSBS, 0x420 / 0x424 the NV12 source, 0x440 its size (as
 * ENC_RSL), 0x464 / 0x468 the strides.  The HAL writes SOI..SOS (its
 * DQT, Annex K's Huffman tables) before BSBS; the hardware the scan and
 * EOI (jpeg_pic_last_ecs), with the quantisers of that DQT, and its
 * length in bytes at 0x4064 (JPEG_HEAD_BITS).  A 2304x1296 scan is a
 * few hundred milliseconds of host time, so it runs on the thread pool
 * (the source is read when the job starts); the frame is done when both
 * halves are.
 *
 * QP_SUM is the QP times the 8x8 blocks: the rate control divides it
 * back.
 *
 * H.264 (ENC_PIC standard 0): the HAL's SPS / PPS (before BSBS, as for
 * H.265) are parsed (h264_enc_parse_headers) and each picture is one
 * slice from the shared H.264 encoder (hantro-h264.c,
 * h264_enc_frame_ref): an IDR when SYNT_NAL's nal_unit_type (bits 6:2) is
 * 5, an I slice when SYNT_SLI0's type (bits 1:0) is 2, else a P slice
 * with inter coding against the previous reconstruction.  The numbering
 * state and the reconstruction follow the chain of RFPW / RFPR addresses
 * as the H.265 references do.  The frame
 * finishes "job-ns" (1 ms) after the start.
 *
 * "vepu541" makes it the RV1126 / RV1109's VEPU541 ("rockchip,
 * rkv-encoder-rv1126", 0xFFBB0000; MPP hal_h265e_vepu541_reg.h,
 * hal_h264e_vepu541_reg.h; the kernel's mpp_rkvenc.c), the same encoder
 * with its registers elsewhere: 0x004 ENC_START, 0x010..0x01c INT_EN /
 * MSK / CLR / STA, 0x030 ENC_RSL, 0x034 ENC_PIC (bit 0 the standard),
 * 0x044 SRC_FMT, 0x114 the strides (Y 15:0, C 31:16), 0x118 / 0x11c the
 * source, 0x128 / 0x130 RFPW / RFPR, 0x14c BSBT 0x150 BSBB 0x154 BSBR
 * 0x158 BSBS (the write start), 0x19c SYNT_NAL, 0x1a8..0x1b0 SYNT_SLI0..2
 * (fields as above), 0x210 ST_BSL, 0x218 the QP sum.  It has no JPEG
 * (the RV1126's JPEG is the VEPU2's), and its buffers are IOVAs through
 * its IOMMU ("iommu" link).  Its OSD is drawn into the source before
 * coding: OSD_CFG (0x1c0) bits 7:0 enable regions 0..7, each at
 * OSD_POS (0x1d0 + 4n: left, top, right, bottom in 16x16 blocks, a byte
 * each) with one 8-bit palette index a pixel, row by row, at OSD_ADDR
 * (0x1f0 + 4n); the palette is 256 words at 0x400 (Y 7:0, U 15:8, V 23:16,
 * alpha 31:24 with 255 opaque: the HAL's OsdPlt, not the public
 * MppEncOSDPltVal's order -- mediaserver's #fff799 text came out
 * orange-red that way), blended over the picture by alpha.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "hevc-enc.h"
#include "jpeg-enc.h"
#include "hantro-h264.h"
#include "block/thread-pool.h"
#include "hw/misc/rk-iommu.h"
#include "system/cpu-timers.h"

#define TYPE_RK_VEPU540C "rk-vepu540c"
OBJECT_DECLARE_SIMPLE_TYPE(RkVepuState, RK_VEPU540C)

#define REGS_SIZE       0x6000

#define R_ENC_START     0x010
#define R_DVBM_CFG      0x060   /* bits 1:0: start on the ISP's frame */
#define DVBM_ARM        3u      /* the connect alone writes 0x15 */
#define R_INT_EN        0x020
#define R_INT_MSK       0x024
#define R_INT_CLR       0x028
#define R_INT_STA       0x02c
#define R_SRC0          0x280
#define R_SRC1          0x284
#define R_RFPW_H        0x28c
#define R_RFPR_H        0x294
#define R_BSBT          0x2b0
#define R_BSBB          0x2b4
#define R_BSBS          0x2b8
#define R_BSBR          0x2bc
#define R_ENC_PIC       0x300
#define R_ENC_RSL       0x310
#define R_SRC_FMT       0x318
#define R_SRC_STRD0     0x334
#define R_SRC_STRD1     0x338
#define R_SYNT_NAL      0x3b0
#define R_SYNT_SLI0     0x3bc
#define R_SYNT_SLI1     0x3c0
#define R_SYNT_SLI2     0x3c4
#define R_JPEG_CFG      0x47c
#define R_ST_BS_LGTH    0x4000
#define R_ST_QP_SUM     0x400c
#define R_JBSBT         0x400
#define R_JBSBB         0x404
#define R_JBSBR         0x408
#define R_JBSBS         0x40c
#define R_JSRC0         0x420
#define R_JSRC1         0x424
#define R_JRSL          0x440
#define R_JSTRD0        0x464
#define R_JSTRD1        0x468
#define R_ST_JPEG_LEN   0x4064

#define INT_FRAME_DONE  (1u << 0)
#define FMT_NV12        6
#define NREFS           8
#define HDR_SCAN        1024    /* bytes before BSBS searched for a VPS */

typedef struct RkVepuRef {
    uint32_t addr;              /* the reconstruction buffer it stands for */
    HevcEncRef *pic;
    HevcEncHdr hdr;
    bool has_hdr;
    H264Enc avc;                /* an H.264 stream's headers and numbering */
    uint8_t *ry, *ruv;          /* its reconstruction, NV12 at mbw * 16 */
    int rmbw, rmbh;
    uint64_t used;
} RkVepuRef;

/* Where a variant keeps the registers the model uses */
typedef struct RkVepuMap {
    hwaddr enc_start, int_en, int_msk, int_clr, int_sta;
    hwaddr src0, src1, rfpw, rfpr, bsbt, bsbb, bsbs, bsbr;
    hwaddr enc_pic, enc_rsl, src_fmt, strd0, strd1;   /* strd1 0: packed */
    hwaddr synt_nal, sli0, sli1, sli2, st_len, st_qp, st_lo, st_hi;
    uint32_t stnd_mask;         /* ENC_PIC's standard field */
    hwaddr osd_cfg, osd_pos, osd_addr, osd_plt;    /* osd_cfg 0: no OSD */
    bool jpeg;                  /* has the JPEG register set */
} RkVepuMap;

static const RkVepuMap rk_vepu540c_map = {
    .enc_start = R_ENC_START, .int_en = R_INT_EN, .int_msk = R_INT_MSK,
    .int_clr = R_INT_CLR, .int_sta = R_INT_STA,
    .src0 = R_SRC0, .src1 = R_SRC1, .rfpw = R_RFPW_H, .rfpr = R_RFPR_H,
    .bsbt = R_BSBT, .bsbb = R_BSBB, .bsbs = R_BSBS, .bsbr = R_BSBR,
    .enc_pic = R_ENC_PIC, .enc_rsl = R_ENC_RSL, .src_fmt = R_SRC_FMT,
    .strd0 = R_SRC_STRD0, .strd1 = R_SRC_STRD1,
    .synt_nal = R_SYNT_NAL, .sli0 = R_SYNT_SLI0, .sli1 = R_SYNT_SLI1,
    .sli2 = R_SYNT_SLI2, .st_len = R_ST_BS_LGTH, .st_qp = R_ST_QP_SUM,
    .st_lo = 0x4000, .st_hi = 0x4ffc,
    .stnd_mask = 3, .jpeg = true,
};

/* VEPU541 (RV1126 / RV1109 "rockchip,rkv-encoder-rv1126", MPP's
 * hal_h265e_vepu541_reg.h / hal_h264e_vepu541_reg.h) */
static const RkVepuMap rk_vepu541_map = {
    .enc_start = 0x004, .int_en = 0x010, .int_msk = 0x014,
    .int_clr = 0x018, .int_sta = 0x01c,
    .src0 = 0x118, .src1 = 0x11c, .rfpw = 0x128, .rfpr = 0x130,
    .bsbt = 0x14c, .bsbb = 0x150, .bsbr = 0x154, .bsbs = 0x158,
    .enc_pic = 0x034, .enc_rsl = 0x030, .src_fmt = 0x044,
    .strd0 = 0x114, .strd1 = 0,
    .synt_nal = 0x19c, .sli0 = 0x1a8, .sli1 = 0x1ac, .sli2 = 0x1b0,
    .st_len = 0x210, .st_qp = 0x218, .st_lo = 0x210, .st_hi = 0x2fc,
    .osd_cfg = 0x1c0, .osd_pos = 0x1d0, .osd_addr = 0x1f0, .osd_plt = 0x400,
    .stnd_mask = 1, .jpeg = false,
};

struct RkVepuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t job_ns;
    RkVepuRef refs[NREFS];
    uint64_t jobs;
    int pending;                /* halves of the job still running */
    uint32_t gen;               /* reset count: stale JPEG work is dropped */
    const RkVepuMap *m;
    bool vepu541;               /* the RV1126's register map, no JPEG */
    bool inter;                 /* property: H.265 P with motion search */
    Object *iommu;
    bool dvbm_armed;            /* DVBM_CFG enabled: the ISP's next frame */
};

typedef struct RkVepuJpegWork {
    RkVepuState *s;
    uint32_t gen;
    uint8_t *y, *cb, *cr;
    int ys, w, h;
    uint8_t ql[64], qc[64];
    uint32_t bs;
    size_t cap, n;
    uint8_t *out;
    bool overflow;
} RkVepuJpegWork;

#define REG(s, off) ((s)->regs[(off) / 4])

static int rk_vepu_ystride(RkVepuState *s)
{
    return s->m->strd1 ? REG(s, s->m->strd0) & 0x1ffff
                       : REG(s, s->m->strd0) & 0xffff;
}

static int rk_vepu_cstride(RkVepuState *s)
{
    return s->m->strd1 ? REG(s, s->m->strd1) & 0xffff
                       : REG(s, s->m->strd0) >> 16;
}

static uint32_t rk_vepu_stnd(RkVepuState *s)
{
    return REG(s, s->m->enc_pic) & s->m->stnd_mask;
}

/* Memory as the encoder sees it: through its IOMMU when it has one */
static void rk_vepu_dma_read(RkVepuState *s, uint32_t a, void *buf,
                             size_t len, MemTxAttrs attrs)
{
    if (s->iommu) {
        rk_iommu_dma_read(s->iommu, a, buf, len);
    } else {
        dma_memory_read(&address_space_memory, a, buf, len, attrs);
    }
}

static void rk_vepu_dma_write(RkVepuState *s, uint32_t a, const void *buf,
                              size_t len, MemTxAttrs attrs)
{
    if (s->iommu) {
        rk_iommu_dma_write(s->iommu, a, buf, len);
    } else {
        dma_memory_write(&address_space_memory, a, buf, len, attrs);
    }
}

/* The OSD regions over the NV12 source, blended by the palette's alpha */
static void rk_vepu_osd(RkVepuState *s, uint8_t *y, int ys, uint8_t *uv,
                        int cs, int w, int h)
{
    uint32_t en = s->m->osd_cfg ? REG(s, s->m->osd_cfg) & 0xff : 0;
    int n;

    for (n = 0; n < 8; n++) {
        uint32_t pos = REG(s, s->m->osd_pos + 4 * n);
        int x0 = (pos & 0xff) * 16, y0 = ((pos >> 8) & 0xff) * 16;
        int x1 = ((pos >> 16) & 0xff) * 16 + 16;
        int y1 = ((pos >> 24) & 0xff) * 16 + 16;
        int rw = x1 - x0, r, c;
        g_autofree uint8_t *row = NULL;

        if (!(en & (1u << n)) || x1 <= x0 || y1 <= y0) {
            continue;
        }
        row = g_malloc(rw);
        for (r = y0; r < y1 && r < h; r++) {
            rk_vepu_dma_read(s, REG(s, s->m->osd_addr + 4 * n) +
                             (uint32_t)(r - y0) * rw, row, rw,
                             MEMTXATTRS_UNSPECIFIED);
            for (c = x0; c < x1 && c < w; c++) {
                uint32_t p = REG(s, s->m->osd_plt + 4 * row[c - x0]);
                uint32_t a = p >> 24;
                uint8_t *py = &y[r * ys + c];

                if (!a) {
                    continue;
                }
                *py = ((p & 0xff) * a + *py * (255 - a)) / 255;
                if (!(r & 1) && !(c & 1)) {
                    uint8_t *pc = &uv[(r / 2) * cs + c];

                    pc[0] = (((p >> 8) & 0xff) * a + pc[0] * (255 - a)) / 255;
                    pc[1] = (((p >> 16) & 0xff) * a + pc[1] * (255 - a)) /
                            255;
                }
            }
        }
    }
}

/*
 * VEPU540C's OSD (kmpp vepu540c_common.h vepu540c_osd_reg, written by
 * vepu540c_set_osd): eight regions of nine words from 0x3000 for H.264 /
 * H.265 and again from 0x3138 for JPEG --
 *   +0x00 cfg0  bit 0 on, 11:8 format (VEPU541_FMT_: 0 BGRA8888, 1 BGR888,
 *               2 BGR565, 3 ARGB1555), 12 alpha swap, 13 R/B swap,
 *               21:14 background alpha, 29:22 foreground alpha
 *   +0x04 cfg1  left x 13:0, top y 27:14 (pixels)
 *   +0x08 cfg2  right x, bottom y, inclusive
 *   +0x0c       the picture's address (through the IOMMU)
 *   +0x14 cfg5  stride in bytes, 16:0
 * then 0x120 on (0x3120 / 0x3258) the RGB->YUV matrix (9-bit signed
 * Y/U/V rows of R, G, B weights, /256) and its offsets.  A 1555 pixel's
 * alpha bit picks
 * the foreground or background alpha (XM's text: 240 and 0); a BGRA8888
 * pixel's own alpha scales the foreground one.
 */
#define OSD540_BASE     0x3000      /* H.264 / H.265 (osd_comb_cfg) */
#define OSD540_JPEG     0x3138      /* JPEG (osd_jpeg_cfg) */
#define OSD540_CSC      0x120       /* the matrix, after the 8 regions */

static int rk_vepu_s9(uint32_t v)
{
    return (v & 0x100) ? (int)(v & 0x1ff) - 0x200 : (int)(v & 0x1ff);
}

static void rk_vepu_osd540c(RkVepuState *s, hwaddr base, uint8_t *y, int ys,
                            uint8_t *uv, int cs, int w, int h)
{
    uint32_t c0 = REG(s, base + OSD540_CSC), c1 = REG(s, base + OSD540_CSC + 4);
    uint32_t c2 = REG(s, base + OSD540_CSC + 8);
    uint32_t co = REG(s, base + OSD540_CSC + 12);
    int m[3][3] = {
        { rk_vepu_s9(c0), rk_vepu_s9(c0 >> 9), rk_vepu_s9(c0 >> 18) },
        { rk_vepu_s9(c1), rk_vepu_s9(c1 >> 9), rk_vepu_s9(c1 >> 18) },
        { rk_vepu_s9(c2), rk_vepu_s9(c2 >> 9), rk_vepu_s9(c2 >> 18) },
    };
    int ofs[3] = { co & 0xff, (co >> 8) & 0xff, (co >> 16) & 0xff };
    int n;

    if (!c0) {                          /* no matrix: BT.601 full range */
        static const int d[3][3] = {
            { 77, 150, 29 }, { -43, -85, 128 }, { 128, -107, -21 },
        };
        memcpy(m, d, sizeof(m));
        ofs[1] = ofs[2] = 128;
    }
    for (n = 0; n < 8; n++) {
        hwaddr b = base + 0x24 * n;
        uint32_t cfg0 = REG(s, b), cfg1 = REG(s, b + 4), cfg2 = REG(s, b + 8);
        uint32_t addr = REG(s, b + 0xc), stride = REG(s, b + 0x14) & 0x1ffff;
        int fmt = (cfg0 >> 8) & 0xf, bpp;
        bool aswap = cfg0 & (1u << 12), rbswap = cfg0 & (1u << 13);
        int bg = (cfg0 >> 14) & 0xff, fg = (cfg0 >> 22) & 0xff;
        int x0 = cfg1 & 0x3fff, y0 = (cfg1 >> 14) & 0x3fff;
        int x1 = (cfg2 & 0x3fff) + 1, y1 = ((cfg2 >> 14) & 0x3fff) + 1;
        g_autofree uint8_t *row = NULL;
        int r, c;

        if (!(cfg0 & 1) || !addr || x1 <= x0 || y1 <= y0) {
            continue;
        }
        bpp = fmt == 0 ? 4 : fmt == 1 ? 3 : (fmt == 2 || fmt == 3) ? 2 : 0;
        if (!bpp) {
            qemu_log_mask(LOG_UNIMP, "rk-vepu540c: OSD format %d\n", fmt);
            continue;
        }
        stride = MAX(stride, (uint32_t)(x1 - x0) * bpp);
        row = g_malloc(stride);
        for (r = y0; r < y1 && r < h; r++) {
            rk_vepu_dma_read(s, addr + (uint32_t)(r - y0) * stride, row,
                             stride, MEMTXATTRS_UNSPECIFIED);
            for (c = x0; c < x1 && c < w; c++) {
                const uint8_t *p = row + (c - x0) * bpp;
                int rgb[3], a, k, v;
                uint8_t *py = &y[r * ys + c];

                if (fmt == 3) {         /* ARGB1555, or RGBA5551 swapped */
                    uint32_t q = p[0] | p[1] << 8;
                    bool on = aswap ? q & 1 : q >> 15;

                    if (aswap) {
                        q >>= 1;
                    }
                    rgb[0] = ((q >> 10) & 0x1f) * 255 / 31;
                    rgb[1] = ((q >> 5) & 0x1f) * 255 / 31;
                    rgb[2] = (q & 0x1f) * 255 / 31;
                    a = on ? fg : bg;
                } else if (fmt == 2) {  /* BGR565 */
                    uint32_t q = p[0] | p[1] << 8;

                    rgb[0] = ((q >> 11) & 0x1f) * 255 / 31;
                    rgb[1] = ((q >> 5) & 0x3f) * 255 / 63;
                    rgb[2] = (q & 0x1f) * 255 / 31;
                    a = fg;
                } else {                /* B, G, R (, A) in memory */
                    const uint8_t *q = (fmt == 0 && aswap) ? p + 1 : p;

                    rgb[2] = q[0];
                    rgb[1] = q[1];
                    rgb[0] = q[2];
                    a = fmt == 0 ? (aswap ? p[0] : p[3]) * fg / 255 : fg;
                }
                if (rbswap) {
                    int t = rgb[0];

                    rgb[0] = rgb[2];
                    rgb[2] = t;
                }
                if (!a) {
                    continue;
                }
                for (k = 0; k < 3; k++) {
                    v = (m[k][0] * rgb[0] + m[k][1] * rgb[1] +
                         m[k][2] * rgb[2] + 128) / 256 + ofs[k];
                    v = MIN(MAX(v, 0), 255);
                    if (k == 0) {
                        *py = (v * a + *py * (255 - a)) / 255;
                    } else if (!(r & 1) && !(c & 1)) {
                        uint8_t *pc = &uv[(r / 2) * cs + c + k - 1];

                        *pc = (v * a + *pc * (255 - a)) / 255;
                    }
                }
            }
        }
    }
}

static void rk_vepu_update(RkVepuState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, s->m->int_sta) & REG(s, s->m->int_en) &
                            ~REG(s, s->m->int_msk)));
}

static RkVepuRef *rk_vepu_ref(RkVepuState *s, uint32_t addr, bool create)
{
    RkVepuRef *lru = &s->refs[0];
    int i;

    for (i = 0; i < NREFS; i++) {
        if (s->refs[i].pic && s->refs[i].addr == addr) {
            s->refs[i].used = s->jobs;
            return &s->refs[i];
        }
        if (s->refs[i].used < lru->used) {
            lru = &s->refs[i];
        }
    }
    if (!create) {
        return NULL;
    }
    if (!lru->pic) {
        lru->pic = hevc_enc_ref_new();
    }
    lru->addr = addr;
    lru->has_hdr = false;
    lru->used = s->jobs;
    return lru;
}

/* Bytes of the stream ring from @addr on, @len of them, wrapping at top. */
static void rk_vepu_ring_rw(RkVepuState *s, uint32_t addr, void *buf,
                            size_t len, bool write)
{
    uint32_t bot = REG(s, s->m->bsbb), top = REG(s, s->m->bsbt);
    uint8_t *p = buf;

    while (len) {
        size_t n = len;

        if (top > bot && addr <= top) {
            n = MIN(n, (size_t)top - addr + 1);
        }
        if (write) {
            rk_vepu_dma_write(s, addr, p, n,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            rk_vepu_dma_read(s, addr, p, n,
                            MEMTXATTRS_UNSPECIFIED);
        }
        p += n;
        len -= n;
        addr += n;
        if (len && top > bot && addr > top) {
            addr = bot;
        }
    }
}

/*
 * The VPS / SPS / PPS the HAL put in front of BSBS, if it did; *len: how
 * many bytes before BSBS they start.
 */
static bool rk_vepu_headers(RkVepuState *s, HevcEncHdr *hdr, uint32_t *len)
{
    uint32_t bs = REG(s, s->m->bsbs), bot = REG(s, s->m->bsbb);
    uint32_t back = MIN(HDR_SCAN, bs - MIN(bs, bot));
    g_autofree uint8_t *buf = NULL;
    uint32_t i;

    if (!back) {
        return false;
    }
    buf = g_malloc(back);
    rk_vepu_dma_read(s, bs - back, buf, back,
                    MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i + 5 < back; i++) {
        /* 00 00 01 40 01: a VPS's start code and NAL header */
        if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1 &&
            buf[i + 3] == 0x40 && buf[i + 4] == 0x01) {
            uint32_t from = (i && !buf[i - 1]) ? i - 1 : i;

            if (hevc_enc_parse_headers(hdr, buf + from, back - from)) {
                hdr->prm.intra_ac = true;   /* the residual, not just DC */
                if (s->inter) {
                    /*
                     * real inter coding, within what the slice header
                     * says: no temporal MVP, MaxNumMergeCand 4
                     */
                    hdr->prm.inter = true;
                    hdr->prm.tmvp = false;
                    hdr->prm.max_merge_cand = 4;
                    hdr->prm.scaling_list = false;
                    hdr->prm.sign_hiding = false;
                }
                *len = back - from;
                return true;
            }
        }
    }
    return false;
}

static size_t rk_vepu_hevc(RkVepuState *s, uint8_t *out, size_t cap,
                           uint32_t *qp_out)
{
    uint32_t rsl = REG(s, s->m->enc_rsl);
    int w = ((rsl & 0x7ff) + 1) * 8, h = (((rsl >> 16) & 0x7ff) + 1) * 8;
    int ys = rk_vepu_ystride(s), cs = rk_vepu_cstride(s);
    int type = (REG(s, s->m->sli0) >> 15) & 3;
    int nal = REG(s, s->m->synt_nal) & 0x3f;
    int qp = (REG(s, s->m->sli1) >> 16) & 0x3f;
    int poc = REG(s, s->m->sli2) & 0xffff;
    bool p_slice = type != 2 && !(nal >= 16 && nal <= 21);
    RkVepuRef *ref = NULL, *rec;
    HevcEncRef *recon;
    HevcEncHdr hdr;
    bool have_hdr = false;
    uint32_t hdr_len = 0;
    g_autofree uint8_t *y = NULL, *uv = NULL;
    size_t n;

    if (!qp) {
        qp = (REG(s, s->m->enc_pic) >> 8) & 0x3f;
    }
    *qp_out = qp;
    if (((REG(s, s->m->src_fmt) >> 2) & 0xf) != FMT_NV12) {
        qemu_log_mask(LOG_UNIMP, "rk-vepu540c: source format %u\n",
                      (REG(s, s->m->src_fmt) >> 2) & 0xf);
    }
    if (ys < w) {
        ys = w;
    }
    if (cs < w) {
        cs = ys;
    }
    y = g_malloc((size_t)ys * h);
    uv = g_malloc((size_t)cs * (h / 2));
    rk_vepu_dma_read(s, REG(s, s->m->src0), y,
                    (size_t)ys * h, MEMTXATTRS_UNSPECIFIED);
    rk_vepu_dma_read(s, REG(s, s->m->src1), uv,
                    (size_t)cs * (h / 2), MEMTXATTRS_UNSPECIFIED);
    if (s->vepu541) {
        rk_vepu_osd(s, y, ys, uv, cs, w, h);
    } else {
        rk_vepu_osd540c(s, OSD540_BASE, y, ys, uv, cs, w, h);
    }

    if (p_slice) {
        ref = rk_vepu_ref(s, REG(s, s->m->rfpr), false);
        if (ref && ref->has_hdr) {
            hdr = ref->hdr;
            have_hdr = true;
        }
    } else {
        have_hdr = rk_vepu_headers(s, &hdr, &hdr_len);
    }
    /* coded into a fresh picture: the reference may be the same buffer */
    recon = hevc_enc_ref_new();
    if (have_hdr) {
        n = hevc_enc_hdr_frame(&hdr, qp, p_slice, poc,
                               ref ? ref->pic : NULL, recon,
                               y, w, h, ys, uv, cs, out, cap);
    } else {
        HevcEncParams prm = hevc_params_ctb32_sao;

        prm.intra_ac = true;
        prm.inter = s->inter;           /* our own SPS / PPS say so */
        n = hevc_enc_frame(&prm, w, h, qp, p_slice, poc,
                           ref ? ref->pic : NULL, recon,
                           y, w, h, ys, uv, cs, out, cap);
    }
    rec = rk_vepu_ref(s, REG(s, s->m->rfpw), true);
    hevc_enc_ref_free(rec->pic);
    rec->pic = recon;
    rec->has_hdr = have_hdr;
    if (have_hdr) {
        rec->hdr = hdr;
    }
    return n;
}

static const uint8_t rk_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * The quantisers (natural order) of the DQT in the header the HAL wrote
 * before the JPEG BSBS; false when there is none.
 */
static bool rk_vepu_jpeg_dqt(RkVepuState *s, uint8_t ql[64], uint8_t qc[64])
{
    uint32_t bs = REG(s, R_JBSBS), bot = REG(s, R_JBSBB);
    uint32_t back = MIN(HDR_SCAN, bs - MIN(bs, bot));
    g_autofree uint8_t *b = NULL;
    uint32_t i, k, got = 0;

    if (back < 4) {
        return false;
    }
    b = g_malloc(back);
    rk_vepu_dma_read(s, bs - back, b, back,
                    MEMTXATTRS_UNSPECIFIED);
    for (i = back - 2; i > 0 && !(b[i] == 0xff && b[i + 1] == 0xd8); i--) {
        continue;
    }
    if (b[i] != 0xff || b[i + 1] != 0xd8) {
        return false;
    }
    for (i += 2; i + 4 <= back && b[i] == 0xff; ) {
        uint32_t len = b[i + 2] << 8 | b[i + 3], end = i + 2 + len;

        if (end > back) {
            break;
        }
        if (b[i + 1] == 0xdb) {                 /* DQT: one or more tables */
            for (k = i + 4; k + 65 <= end; k += 65) {
                uint8_t *q = (b[k] & 0xf) ? qc : ql;
                int j;

                if (b[k] >> 4) {
                    return false;               /* 16-bit: not ours */
                }
                for (j = 0; j < 64; j++) {
                    q[rk_zigzag[j]] = b[k + 1 + j];
                }
                got |= (b[k] & 0xf) ? 2 : 1;
            }
        }
        if (b[i + 1] == 0xda) {
            break;
        }
        i = end;
    }
    if (got == 1) {
        memcpy(qc, ql, 64);
    }
    return got != 0;
}

static void rk_vepu_finish_half(RkVepuState *s);

static int rk_vepu_jpeg_work(void *opaque)
{
    RkVepuJpegWork *wk = opaque;

    wk->out = g_malloc(wk->cap);
    wk->n = jpeg_enc_scan(wk->y, wk->ys, wk->cb, wk->cr, wk->w / 2,
                          wk->w, wk->h, wk->ql, wk->qc, true,
                          wk->out, wk->cap, &wk->overflow);
    return 0;
}

static void rk_vepu_jpeg_done(void *opaque, int ret)
{
    RkVepuJpegWork *wk = opaque;
    RkVepuState *s = wk->s;

    if (wk->gen == s->gen) {
        if (wk->overflow) {
            wk->n = 0;
        }
        if (wk->n) {
            rk_vepu_dma_write(wk->s, wk->bs, wk->out, wk->n,
                             MEMTXATTRS_UNSPECIFIED);
        }
        REG(s, R_ST_JPEG_LEN) = wk->n;
        rk_vepu_finish_half(s);
    }
    g_free(wk->y);
    g_free(wk->cb);
    g_free(wk->cr);
    g_free(wk->out);
    g_free(wk);
}

/* Read the JPEG half's input and hand it to the thread pool. */
static bool rk_vepu_jpeg_start(RkVepuState *s)
{
    uint32_t rsl = REG(s, R_JRSL);
    int w = ((rsl & 0x7ff) + 1) * 8, h = (((rsl >> 16) & 0x7ff) + 1) * 8;
    int ys = REG(s, R_JSTRD0) & 0x1ffff, cs = REG(s, R_JSTRD1) & 0xffff;
    uint32_t bs = REG(s, R_JBSBS), top = REG(s, R_JBSBT);
    RkVepuJpegWork *wk;
    g_autofree uint8_t *uv = NULL;
    int i;

    wk = g_new0(RkVepuJpegWork, 1);
    if (top < bs || !rk_vepu_jpeg_dqt(s, wk->ql, wk->qc)) {
        g_free(wk);
        return false;
    }
    ys = MAX(ys, w);
    cs = cs ? cs : ys;
    wk->s = s;
    wk->gen = s->gen;
    wk->w = w;
    wk->h = h;
    wk->ys = ys;
    wk->bs = bs;
    wk->cap = MIN((size_t)top - bs + 1, 16 << 20);
    wk->y = g_malloc((size_t)ys * h);
    wk->cb = g_malloc((size_t)(w / 2) * (h / 2));
    wk->cr = g_malloc((size_t)(w / 2) * (h / 2));
    uv = g_malloc((size_t)cs * (h / 2));
    rk_vepu_dma_read(s, REG(s, R_JSRC0), wk->y,
                    (size_t)ys * h, MEMTXATTRS_UNSPECIFIED);
    rk_vepu_dma_read(s, REG(s, R_JSRC1), uv,
                    (size_t)cs * (h / 2), MEMTXATTRS_UNSPECIFIED);
    if (!s->vepu541) {
        /* the snapshot's own OSD set */
        rk_vepu_osd540c(s, OSD540_JPEG, wk->y, ys, uv, cs, w, h);
    }
    for (i = 0; i < (w / 2) * (h / 2); i++) {
        const uint8_t *p = uv + (i / (w / 2)) * cs + (i % (w / 2)) * 2;

        wk->cb[i] = p[0];
        wk->cr[i] = p[1];
    }
    thread_pool_submit_aio(rk_vepu_jpeg_work, wk, rk_vepu_jpeg_done, wk);
    return true;
}

static size_t rk_vepu_h264(RkVepuState *s, uint8_t *out, size_t cap,
                           uint32_t *qp_out)
{
    uint32_t rsl = REG(s, s->m->enc_rsl);
    int w = ((rsl & 0x7ff) + 1) * 8, h = (((rsl >> 16) & 0x7ff) + 1) * 8;
    int mbw = (w + 15) / 16, mbh = (h + 15) / 16;
    int ys = rk_vepu_ystride(s), cs = rk_vepu_cstride(s);
    bool idr = ((REG(s, s->m->synt_nal) >> 2) & 0x1f) == 5;
    int qp = (REG(s, s->m->enc_pic) >> 8) & 0x3f;
    bool p_slice = !idr && (REG(s, s->m->sli0) & 3) != 2;
    RkVepuRef *prev = NULL, *rec;
    H264Enc avc = { 0 };
    g_autofree uint8_t *y = NULL, *uv = NULL;
    uint8_t *rec_y, *rec_uv;
    const uint8_t *ref_y = NULL, *ref_uv = NULL;
    size_t n;

    *qp_out = qp;
    ys = MAX(ys, w);
    cs = cs ? cs : ys;
    /* whole macroblocks: rows past the picture repeat nothing useful,
     * but stay inside our buffers */
    y = g_malloc0((size_t)ys * mbh * 16);
    uv = g_malloc0((size_t)cs * mbh * 8);
    rk_vepu_dma_read(s, REG(s, s->m->src0), y,
                    (size_t)ys * h, MEMTXATTRS_UNSPECIFIED);
    rk_vepu_dma_read(s, REG(s, s->m->src1), uv,
                    (size_t)cs * (h / 2), MEMTXATTRS_UNSPECIFIED);
    if (s->vepu541) {
        rk_vepu_osd(s, y, ys, uv, cs, w, h);
    } else {
        rk_vepu_osd540c(s, OSD540_BASE, y, ys, uv, cs, w, h);
    }
    if (!idr) {
        prev = rk_vepu_ref(s, REG(s, s->m->rfpr), false);
    }
    if (prev) {
        avc = prev->avc;
        if (prev->ry && prev->rmbw == mbw && prev->rmbh == mbh) {
            ref_y = prev->ry;
            ref_uv = prev->ruv;
        }
    }
    if (idr) {
        uint32_t bs = REG(s, s->m->bsbs), bot = REG(s, s->m->bsbb);
        uint32_t back = MIN(HDR_SCAN, bs - MIN(bs, bot));

        if (back) {
            g_autofree uint8_t *hb = g_malloc(back);

            rk_vepu_dma_read(s, bs - back, hb, back,
                            MEMTXATTRS_UNSPECIFIED);
            h264_enc_parse_headers(&avc, hb, back);
        }
    }
    /* coded into fresh planes: the reference may be the same slot */
    rec_y = g_malloc0((size_t)mbw * 16 * mbh * 16);
    rec_uv = g_malloc0((size_t)mbw * 16 * mbh * 8);
    n = (avc.have_sps && avc.have_pps) ?
        h264_enc_frame_ref(&avc, idr, p_slice, qp ? qp : 30, mbw, mbh,
                           y, ys, uv, cs, ref_y, ref_uv, rec_y, rec_uv,
                           out, cap) : 0;
    rec = rk_vepu_ref(s, REG(s, s->m->rfpw), true);
    rec->avc = avc;
    g_free(rec->ry);
    g_free(rec->ruv);
    rec->ry = rec_y;
    rec->ruv = rec_uv;
    rec->rmbw = mbw;
    rec->rmbh = mbh;
    return n;
}

static void rk_vepu_run(RkVepuState *s)
{
    uint32_t bs = REG(s, s->m->bsbs), bot = REG(s, s->m->bsbb), top = REG(s, s->m->bsbt);
    uint32_t rd = REG(s, s->m->bsbr);
    uint32_t rsl = REG(s, s->m->enc_rsl), qp = 0;
    uint32_t blocks = ((rsl & 0x7ff) + 1) * (((rsl >> 16) & 0x7ff) + 1);
    size_t cap, n = 0;
    g_autofree uint8_t *out = NULL;

    s->jobs++;
    /* room up to the reader, or to the top of a buffer that is not one */
    if (top > bot && rd > bs) {
        cap = rd - bs;
    } else if (top > bot && rd != bot) {
        cap = (size_t)top - bs + 1 + (rd - bot);
    } else {
        cap = top >= bs ? (size_t)top - bs + 1 : 0;
    }
    cap = MIN(cap, 16 << 20);
    if (rk_vepu_stnd(s) <= 1 && cap) {   /* 2: JPEG alone */
        out = g_malloc(cap);
        /*
         * The encode runs here, in the vCPU's register write; the guest's
         * clock stands still meanwhile, so the job costs it "job-ns" as on
         * the chip whatever the host takes.  Without that, a slow 4K
         * picture outlasted the driver's task timeout, the driver reset
         * the encoder mid-job and never got back in step.
         */
        cpu_disable_ticks();
        n = rk_vepu_stnd(s) == 1 ? rk_vepu_hevc(s, out, cap, &qp)
                                         : rk_vepu_h264(s, out, cap, &qp);
        cpu_enable_ticks();
        if (n > cap) {
            n = 0;              /* truncated: nothing usable */
        }
        if (n) {
            rk_vepu_ring_rw(s, bs, out, n, true);
        }
    }
    REG(s, s->m->st_len) = n;
    REG(s, s->m->st_qp) = qp * blocks;
    if (s->m->jpeg) {
        REG(s, R_ST_JPEG_LEN) = 0;
    }
    s->pending = 1;
    if (s->m->jpeg && (REG(s, R_JPEG_CFG) >> 31) &&
        rk_vepu_jpeg_start(s)) {
        s->pending++;
    }
    timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->job_ns);
}

static void rk_vepu_finish_half(RkVepuState *s)
{
    if (s->pending > 0 && --s->pending == 0) {
        REG(s, s->m->int_sta) |= INT_FRAME_DONE;
        rk_vepu_update(s);
    }
}

static void rk_vepu_done(void *opaque)
{
    rk_vepu_finish_half(opaque);
}

static uint64_t rk_vepu_read(void *opaque, hwaddr off, unsigned size)
{
    RkVepuState *s = opaque;

    return s->regs[off / 4];
}

static void rk_vepu_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkVepuState *s = opaque;

    if (off == s->m->enc_start) {
        REG(s, off) = val;
        if (((val >> 8) & 3) == 1) {
            rk_vepu_run(s);
        }
        return;
    } else if (off == R_DVBM_CFG && !s->vepu541) {
        /*
         * Hardware DVBM ("online mode 2"): the driver writes the frame's
         * registers and then DVBM_CFG with bits 1:0 set (0x4000b)
         * instead of ENC_START; the ISP's next mainpath frame end starts
         * the job.  Connecting the ISP alone writes 0x15.
         */
        REG(s, off) = val;
        s->dvbm_armed = (val & DVBM_ARM) == DVBM_ARM;
        return;
    } else if (off == s->m->int_clr) {
        REG(s, s->m->int_sta) &= ~val;
    } else if (off == s->m->int_sta) {
        REG(s, s->m->int_sta) &= val;   /* the driver writes 0 after CLR */
    } else if (off >= s->m->st_lo && off <= s->m->st_hi) {
        return;                         /* status: read-only */
    } else {
        s->regs[off / 4] = val;
        if (off != s->m->int_en && off != s->m->int_msk) {
            return;
        }
    }
    rk_vepu_update(s);
}

/* the ISP's mainpath frame end, through the DVBM link */
static void rk_vepu_dvbm_frame(void *opaque, int n, int level)
{
    RkVepuState *s = opaque;

    if (level && s->dvbm_armed && !s->pending) {
        s->dvbm_armed = false;
        rk_vepu_run(s);
    }
}

static const MemoryRegionOps rk_vepu_ops = {
    .read = rk_vepu_read,
    .write = rk_vepu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_vepu_reset(DeviceState *dev)
{
    RkVepuState *s = RK_VEPU540C(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->done);
    for (i = 0; i < NREFS; i++) {
        hevc_enc_ref_free(s->refs[i].pic);
        g_free(s->refs[i].ry);
        g_free(s->refs[i].ruv);
        s->refs[i] = (RkVepuRef) { 0 };
    }
    s->jobs = 0;
    s->pending = 0;
    s->dvbm_armed = false;
    s->gen++;
    rk_vepu_update(s);
}

static void rk_vepu_realize(DeviceState *dev, Error **errp)
{
    RkVepuState *s = RK_VEPU540C(dev);

    s->m = s->vepu541 ? &rk_vepu541_map : &rk_vepu540c_map;
    memory_region_init_io(&s->iomem, OBJECT(s), &rk_vepu_ops, s,
                          TYPE_RK_VEPU540C, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    qdev_init_gpio_in_named(dev, rk_vepu_dvbm_frame, "dvbm-frame", 1);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_vepu_done, s);
}

static const Property rk_vepu_properties[] = {
    DEFINE_PROP_UINT32("job-ns", RkVepuState, job_ns, 1000000),
    DEFINE_PROP_BOOL("vepu541", RkVepuState, vepu541, false),
    DEFINE_PROP_BOOL("inter", RkVepuState, inter, false),
    DEFINE_PROP_LINK("iommu", RkVepuState, iommu, "rk-iommu", Object *),
};

/* the references are host pictures: migrating drops them (next IDR) */
static const VMStateDescription rk_vepu_vmstate = {
    .name = TYPE_RK_VEPU540C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkVepuState, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(done, RkVepuState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_vepu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_vepu_realize;
    device_class_set_legacy_reset(dc, rk_vepu_reset);
    device_class_set_props(dc, rk_vepu_properties);
    dc->vmsd = &rk_vepu_vmstate;
}

static const TypeInfo rk_vepu_info = {
    .name = TYPE_RK_VEPU540C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkVepuState),
    .class_init = rk_vepu_class_init,
};

static void rk_vepu_register_types(void)
{
    type_register_static(&rk_vepu_info);
}

type_init(rk_vepu_register_types)
