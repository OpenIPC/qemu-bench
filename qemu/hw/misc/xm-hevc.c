/*
 * Xiongmai second-generation HEVC encoder engine (0x21000000, 64 KiB, and
 * 0x21010000, 256 bytes), as the vendor hevc.ko and the jc_v200 library
 * statically linked into Sofia drive it.
 *
 * hevc.ko is a shim: the library hands it register lists, it writes them,
 * starts the engine and sleeps until hevc_interrupt (SPI 45) wakes it.
 * What a job is, read from the registers the library programs (logged at
 * run time; there is no public source):
 *
 *   +0x000     bit 0: start (the kernel writes 0 when it is done)
 *   +0x018     interrupt acknowledge: write 1s to clear +0x074 bits
 *   +0x044/48  encoded width - 1 / height - 1
 *   +0x054     slice type: 0 I (an IDR), 1 P
 *   +0x058     table select, the table's words at +0x800..  (rate control)
 *   +0x05C     slice QP
 *   +0x088     bytes free at the output pointer
 *   +0x0A0/A4  source luma / chroma (NV12), physical
 *   +0x0A8/AC  the reference picture, luma / chroma: what the last job of
 *              the stream wrote to +0x0B0/B4, its reconstruction
 *   +0x0D0     output pointer (the stream buffer + 256)
 *   +0x0D8/DC  source luma / chroma strides
 *   +0x0F8     bytes the job wrote (hevc_interrupt's stream length)
 *   +0x0FC     1 main stream, 0 sub stream
 *   +0x100/104 source width - 1 / height - 1: the engine scales, which is
 *              how a 720p sensor gets a "1080P" stream
 *   +0x074     status: bit 0 frame done (3 takes the plain success path)
 *
 *   +0x11C     regions to overlay, a bit each (Sofia's main-stream clock and
 *              title; the sub stream's go through the OSD engine).  Region n
 *              at +0x120 + 0x20 * n: +0x00 x0, y0, x1, y1 (bytes, from the
 *              top, in 16-pixel units, the end exclusive), +0x04 enable,
 *              +0x08 bit 0 / bit 2: the bitmap doubled across / down,
 *              +0x0C..+0x18 the four colours (A Y U V, 0 clear), +0x1C the
 *              bitmap: 2 bits a pixel in 16x16 tiles of 64 bytes, row by
 *              row, a tile's rows 4 bytes each, most significant pixel
 *              first.  Blended into the source before it is encoded.
 *
 * +0x068 / +0x06C / +0x070 (errors, line mode) read 0.  Every other word is
 * RAM-backed.  A job starts a millisecond after the start bit and is
 * encoded on a worker thread; it completes when the encoder is done, so a
 * big picture takes (virtual) time as on the hardware.  The library
 * writes the parameter sets and the slice header itself, in the 256 bytes
 * before the output pointer, and the engine only the slice data after them:
 * so the job writes slice_segment_data() (hevc-enc.c), coded against the
 * library's slice header.  A P job skips the CUs whose source is the
 * reference's: the engine keeps each decoded picture (on the host, not in
 * the guest's reconstruction buffer, whose format is the hardware's) under
 * the reconstruction address, and the next P job of the stream names it as
 * its reference.  Without one (a reset, a picture of another size) the P
 * picture is coded all intra.
 *
 * layout "xm533" (XM530V300) is a different engine behind the same
 * library: hevc.ko writes registers 2..188 from the list and 1 to +0x004;
 *
 *   +0x034/38  encoded width - 1 / height - 1 (the source is that size too)
 *   +0x040     slice type: 0 I, 1 P          +0x044  slice QP
 *   +0x1D4     bytes free at the output pointer
 *   +0x1D8     bytes the job wrote
 *   +0x1E0/E4  source luma / chroma (NV12)   +0x224/228  their strides
 *   +0x1E8/EC  the reference picture          +0x1F0/F4  the reconstruction
 *   +0x210     output pointer (the stream buffer + 256)
 *   (layout "xm530v200", its earlier revision: the output pointer at
 *   +0x200, the source strides at +0x214 / +0x218; its library codes
 *   32x32 CTBs with SAO, so the slice data does too.  Layout
 *   "xm8521dv200": the XM8521DV200 DVR library, the same registers, its
 *   CTBs 32x32 without SAO; it writing its headers in front of the
 *   output only after the job, and with +0x1C4 set checks that the
 *   engine filled its per-CTB tables at +0x20C / +0x210, see
 *   xm_hevc_ctb_tables())
 *   +0x058     status, 3 frame done; 3 to +0x030 acknowledges.  +0x054
 *              (acknowledged at +0x02C) and +0x050 (errors, at +0x028)
 *              also raise the interrupt; neither happens here.
 *
 * Its parameter sets and slice headers are field for field those of the
 * XM535 library, so the slice data is the same.
 *
 * layout "xm232" (XM232, SPI 24) is a third engine: hevc.ko's handle_bgn
 * writes registers 2..401 from the library's list and 1 to +0x000, and
 * hevc_interrupt reads +0x0B8 (2: frame done, the stream length at
 * +0x174, the snapshot's JPEG length at +0x614), acknowledged by writing
 * it back to +0x028.  +0x0A0 / +0x0A4 / +0x0B0 / +0x0B4 (cleared at
 * +0x010 / +0x014 / +0x020 / +0x024) also raise the interrupt; none of
 * them happens here.  +0x098 (busy) reads 0.
 *
 *   +0x078/7C  encoded width - 1 / height - 1
 *   +0x088     0 I (an IDR), 1 P               +0x090  slice QP
 *   +0x0D0     bytes free at the output pointer
 *   +0x10C/110 source luma / chroma (NV12)     +0x154/158  their strides
 *   +0x114/118 the reference picture           +0x11C/120  the reconstruction
 *   +0x13C     output pointer (the stream buffer + 128 for an I picture,
 *              + 32 for a P one: the library writes its headers there
 *              after the job)
 *   +0x180     bit 0: scale from +0x184/188, source width - 1 / height - 1
 *   +0x5C8     bit 0: a JPEG snapshot of the same source, to +0x5FC
 *
 * Its library's SPS / PPS are XM535's (CTB 16, no SAO, beta / tc offsets
 * 5 / -2).  The same engine codes H.264 (Sofia's default on the X6S-WEQ-H,
 * Main profile, CAVLC): see xm_hevc_xm232_avc() for how a job's codec is
 * told and xm_hevc_work_h264() for the coding.  rgn.ko's overlays are the
 * engine's own regions (xm_hevc_osd232()).  The second window is hevc.ko's
 * line-mode block (0x21010000), RAM here; the encoder does not wait for
 * lines.  No JPEG is coded: its length reads 0, and a snapshot comes out
 * as the library's headers alone (Sofia starts its JPEG channel only on
 * demand).
 *
 * dump-dir (-global xm-hevc.dump-dir=DIR) keeps every job on the host as
 * frame-NNNNN.hevc, a whole access unit with headers equal to the
 * library's, main and sub stream jobs alike.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hevc-enc.h"
#include "hantro-h264.h"

#define TYPE_XM_HEVC "xm-hevc"
OBJECT_DECLARE_SIMPLE_TYPE(XmHevcState, XM_HEVC)

#define XM_HEVC_SIZE    0x10000
#define XM_HEVC2_SIZE   0x100

#define R_START         0x000
#define R_ACK           0x018
#define R_ENC_W         0x044
#define R_ENC_H         0x048
#define R_SLICE_TYPE    0x054
#define R_QP            0x05C
#define R_STATUS        0x074
#define R_OUT_FREE      0x088
#define R_SRC_Y         0x0A0
#define R_SRC_C         0x0A4
#define R_OUT           0x0D0
#define R_SRC_YS        0x0D8
#define R_SRC_CS        0x0DC
#define R_OUT_LEN       0x0F8
#define R_MAIN          0x0FC
#define R_SRC_W         0x100
#define R_SRC_H         0x104
#define R_OSD_MASK      0x11C
#define R_OSD(n)        (0x120 + 0x20 * (n))
#define OSD_REGIONS     4

#define ST_DONE         3

/* the widest source line a job may name: twice the widest picture */
#define XM_HEVC_MAX_STRIDE  16384

/* decoded pictures kept for P jobs: one a stream is enough (D2: six) */
#define XM_HEVC_REFS    8

/* XM530V300 (XM533): another engine.  hevc.ko's handle_bgn writes the
 * library's register list (registers 2..188) and then 1 to +0x004; its
 * hevc_interrupt reads +0x058 (3: frame done; the stream length at
 * +0x1D8), acknowledges with 3 to +0x030, and checks +0x054 (acknowledged
 * at +0x02C) and +0x050 (errors). */
#define V3_START        0x004
#define V3_ERR          0x050
#define V3_ST2          0x054
#define V3_STATUS       0x058
#define V3_ACK2         0x02C
#define V3_ACK          0x030
#define V3_ENC_W        0x034
#define V3_ENC_H        0x038
#define V3_SLICE_TYPE   0x040   /* 0 I (an IDR), 1 P */
#define V3_QP           0x044
#define V3_OUT_FREE     0x1D4
#define V3_OUT_LEN      0x1D8
#define V3_SRC_Y        0x1E0   /* NV12, at the encoded size */
#define V3_SRC_C        0x1E4

/* XM232: hevc.ko's handle_bgn writes registers 2..401 and then 1 to
 * +0x000; hevc_interrupt reads +0x0A0 / +0x0A4 / +0x0B0 / +0x0B4 / +0x0B8
 * and clears each by writing it to +0x010 / +0x014 / +0x020 / +0x024 /
 * +0x028. */
#define X2_START        0x000
#define X2_CLR_AUX      0x010
#define X2_CLR_ERR      0x014
#define X2_CLR_BGN      0x020
#define X2_CLR_HALT     0x024
#define X2_CLR_END      0x028
#define X2_ENC_W        0x078
#define X2_ENC_H        0x07C
#define X2_P_SLICE      0x088   /* 0 I (an IDR), else P */
#define X2_QP           0x090
#define X2_BUSY         0x098
#define X2_AUX          0x0A0   /* aux halt */
#define X2_ERR          0x0A4
#define X2_BGN          0x0B0
#define X2_HALT         0x0B4
#define X2_END          0x0B8   /* 2 frame done, 1 failed */
#define X2_OUT_FREE     0x0D0
#define X2_SRC_Y        0x10C   /* NV12 */
#define X2_SRC_C        0x110
#define X2_OUT          0x13C
#define X2_SRC_YS       0x154
#define X2_SRC_CS       0x158
#define X2_OUT_LEN      0x174
#define X2_SCALE        0x180
#define X2_SRC_W        0x184
#define X2_SRC_H        0x188
#define X2_JPEG_LEN     0x614
/* an H.264 job: the slice header bits the library's last whole byte did
 * not take, left-aligned in a byte, and how many */
#define X2_HDR_BITS     0x52C
#define X2_HDR_NBITS    0x530
#define X2_ST_DONE      2

struct XmHevcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem, iomem2;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[XM_HEVC_SIZE / 4];
    uint32_t regs2[XM_HEVC2_SIZE / 4];
    bool dbg_level;
    unsigned dbg_n;
    char *dump_dir;
    char *layout;
    bool v3;                    /* layout "xm533", "xm530v200" or
                                 * "xm8521dv200" */
    bool xm232;                 /* layout "xm232" */
    uint32_t v3_out, v3_ys, v3_cs;  /* where they differ */
    const HevcEncParams *prm;   /* what the library's headers say */
    uint32_t ref_off, recon_off;    /* the reference / reconstruction luma */
    uint32_t jobs;
    uint32_t poc[2];            /* of the dumps, per stream */
    /* the worker's: decoded pictures by the reconstruction buffer the
     * engine wrote them to, which a P job names as its reference */
    struct {
        uint32_t pa;
        HevcEncRef *pic;
        uint64_t used;
    } refs[XM_HEVC_REFS];
    HevcEncRef *spare;
    /* the same for H.264 (xm232): NV12 at the macroblocks' size */
    struct {
        uint32_t pa;
        uint8_t *y, *uv;
        int mbw, mbh;
        uint64_t used;
    } avc_refs[XM_HEVC_REFS];
    H264Enc avc;                /* the library's SPS / PPS, as last seen */
    struct {
        int w, h;
        bool h264;
        H264Deblock db;
        /* its last job's dump, finished at its next job: the headers go
         * into the gap before the output only after the job */
        char *dump_path;
        uint32_t dump_out;
        uint8_t *dump;
        size_t dump_n;
    } avc_streams[4];           /* by encoded size: main, sub, ... */
    uint64_t ref_clock;
    uint32_t ref_gen;
    /* encoding runs on a worker thread, the guest carries on meanwhile */
    GThreadPool *pool;
    GAsyncQueue *done;
    QEMUBH *done_bh;
    uint32_t gen;               /* bumped by reset: stale jobs are dropped */
};

typedef struct XmHevcJob {
    XmHevcState *s;
    uint32_t gen;
    int w, h, sw, sh, ys, cs, qp, poc;
    bool p_slice;
    uint8_t *y, *uv, *out;
    uint32_t out_pa, cap;
    uint32_t ref_pa, recon_pa;
    size_t n;
    uint32_t len_reg;
    char *dump_path;
    /* H.264 (xm232): the macroblocks, the header's last bits, its
     * deblocking, and the library's bytes before the output for the dump */
    bool h264;
    int stream_slot;
    int mbw, mbh, hdr_nbits;
    uint8_t hdr_bits;
    H264Deblock db;
} XmHevcJob;

#define R(s, off) ((s)->regs[(off) / 4])

static void xm_hevc_update(XmHevcState *s)
{
    if (s->xm232) {
        qemu_set_irq(s->irq, (R(s, X2_AUX) | R(s, X2_ERR) | R(s, X2_BGN) |
                              R(s, X2_HALT) | R(s, X2_END)) != 0);
        return;
    }
    if (s->v3) {
        bool lv = ((R(s, V3_STATUS) | R(s, V3_ST2) | R(s, V3_ERR)) & 3) != 0 ||
                  s->regs2[0x0C / 4] != 0;

        if (lv != s->dbg_level && s->dbg_n < 40) {
            s->dbg_n++;
            qemu_log_mask(LOG_UNIMP, "xm-hevc: irq %d (status %x st2 %x err %x)\n",
                          lv, R(s, V3_STATUS), R(s, V3_ST2), R(s, V3_ERR));
        }
        s->dbg_level = lv;
        qemu_set_irq(s->irq, lv);
        return;
    }
    qemu_set_irq(s->irq, R(s, R_STATUS) != 0);
}

static uint8_t osd_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/*
 * One overlay region into the source (w x h encoded from sw x sh: output
 * pixels mapped back to the source's): x0..x1 / y0..y1 in the encoded
 * picture, the bitmap at bmp_pa 2 bits a pixel in 16x16 tiles of 64
 * bytes, a row of tiles after another, each pixel 2 << sx2 / 2 << sy2 of
 * it wide / high, its value a palette entry (alpha, Y, Cb, Cr).
 */
static void xm_hevc_osd_region(uint8_t *y, uint8_t *uv, int w, int h, int sw,
                               int sh, int ys, int cs, int x0, int y0, int x1,
                               int y1, int sx2, int sy2, const uint32_t *pal,
                               uint32_t bmp_pa)
{
    x1 = MIN(x1, w);
    y1 = MIN(y1, h);
    if (!bmp_pa || x1 <= x0 || y1 <= y0 || sx2 > 4 || sy2 > 4) {
        return;
    }
    const int tpr = (((x1 - x0) >> sx2) + 15) / 16;
    const int bh = (((y1 - y0) >> sy2) + 15) / 16 * 16;
    const size_t len = (size_t)tpr * (bh / 16) * 64;
    g_autofree uint8_t *bmp = g_malloc(len);

    dma_memory_read(&address_space_memory, bmp_pa, bmp, len,
                    MEMTXATTRS_UNSPECIFIED);
    for (int oy = y0; oy < y1; oy++) {
        const int by = (oy - y0) >> sy2;
        const int sy = oy * sh / h;

        for (int ox = x0; ox < x1; ox++) {
            const int bx = (ox - x0) >> sx2;
            const uint8_t b = bmp[((by / 16) * tpr + bx / 16) * 64 +
                                  (by % 16) * 4 + (bx % 16) / 4];
            const uint32_t c = pal[(b >> (6 - 2 * (bx % 4))) & 3];
            const int sx = ox * sw / w;

            if (!(c >> 24)) {
                continue;
            }
            y[(size_t)sy * ys + sx] = osd_blend(y[(size_t)sy * ys + sx],
                                                c >> 16, c >> 24);
            if (!(sy & 1)) {
                uint8_t *p = &uv[(size_t)(sy / 2) * cs + (sx & ~1)];

                p[0] = osd_blend(p[0], c >> 8, c >> 24);
                p[1] = osd_blend(p[1], c, c >> 24);
            }
        }
    }
}

/* the engine's overlay regions (XM535) */
static void xm_hevc_osd(XmHevcState *s, uint8_t *y, uint8_t *uv, int w,
                        int h, int sw, int sh, int ys, int cs)
{
    for (int n = 0; n < OSD_REGIONS; n++) {
        const uint32_t *r = &s->regs[R_OSD(n) / 4];
        const uint32_t pos = r[0], fmt = r[2];

        if (!(R(s, R_OSD_MASK) & (1u << n)) || !r[1]) {
            continue;
        }
        xm_hevc_osd_region(y, uv, w, h, sw, sh, ys, cs, (pos >> 24) * 16,
                           ((pos >> 16) & 0xff) * 16,
                           ((pos >> 8) & 0xff) * 16, (pos & 0xff) * 16,
                           fmt & 1, (fmt >> 2) & 1, &r[3], r[7]);
    }
}

/*
 * XM232's (rgn.ko's comp_osd_process): +0x2A4 the regions on, region n at
 * +0x2A8 + 0x24 * n: +0x00 its first and +0x04 its last macroblock, column
 * << 10 | row; +0x08 the format; +0x0C the bitmap's pixel size, x << 3 |
 * y, as shifts; +0x10..+0x1C the palette; +0x20 the bitmap.
 */
#define X2_OSD_MASK     0x2A4
#define X2_OSD(n)       (0x2A8 + 0x24 * (n))
#define X2_OSD_REGIONS  8

static void xm_hevc_osd232(XmHevcState *s, uint8_t *y, uint8_t *uv, int w,
                           int h, int sw, int sh, int ys, int cs)
{
    for (int n = 0; n < X2_OSD_REGIONS; n++) {
        const uint32_t *r = &s->regs[X2_OSD(n) / 4];

        if (!(R(s, X2_OSD_MASK) & (1u << n))) {
            continue;
        }
        xm_hevc_osd_region(y, uv, w, h, sw, sh, ys, cs, (r[0] >> 10) * 16,
                           (r[0] & 0x3ff) * 16, ((r[1] >> 10) + 1) * 16,
                           ((r[1] & 0x3ff) + 1) * 16, (r[3] >> 3) & 7,
                           r[3] & 7, &r[4], r[8]);
    }
}

/* encode w x h from NV12 at y / uv (strides ys / cs) into out_pa, at most
 * cap bytes: the slice data under the library's own headers */
/* the worker: the slice data, and the dump if asked for */
/* RBSP into a NAL's payload: an emulation prevention byte wherever two
 * zeros (counting the zeros the stream ends with before it) precede a byte
 * of 3 or less */
static size_t xm_hevc_escape(const uint8_t *in, size_t n, int zeros,
                             uint8_t *out, size_t cap)
{
    size_t o = 0;

    for (size_t i = 0; i < n && o < cap; i++) {
        if (zeros >= 2 && in[i] <= 3) {
            out[o++] = 3;
            zeros = 0;
            if (o >= cap) {
                break;
            }
        }
        out[o++] = in[i];
        zeros = in[i] ? 0 : zeros + 1;
    }
    return o;
}

/*
 * An H.264 job (xm232, the main and sub streams Sofia sets to H.264): the
 * same engine codes CAVLC slice data under the library's SPS, PPS and
 * slice header.  The header's bits past its last whole byte come in
 * +0x52C / +0x530 and the slice data follows them.  The source is scaled
 * to the picture's macroblocks, P pictures are predicted from the decoded
 * picture the reference address names (hantro-h264's coder: skip, 16x16
 * inter with residual, or intra), and that picture's reconstruction is
 * kept under the job's reconstruction address.
 */
static void xm_hevc_work_h264(XmHevcJob *j)
{
    XmHevcState *s = j->s;
    const int pw = j->mbw * 16, ph = j->mbh * 16;
    g_autofree uint8_t *y = g_malloc((size_t)pw * ph);
    g_autofree uint8_t *uv = g_malloc((size_t)pw * ph / 2);
    g_autofree uint8_t *rbsp = g_malloc0(j->cap + 64);
    uint8_t *ry = NULL, *ruv = NULL;
    int i, slot = 0;

    /* nearest neighbour, sw x sh to the macroblocks */
    for (int py = 0; py < ph; py++) {
        const int sy = MIN(py * j->sh / ph, j->sh - 1);

        for (int px = 0; px < pw; px++) {
            y[(size_t)py * pw + px] =
                j->y[(size_t)sy * j->ys + MIN(px * j->sw / pw, j->sw - 1)];
        }
        if (!(py & 1)) {
            const int cy = MIN(sy / 2, j->sh / 2 - 1);

            for (int px = 0; px < pw; px += 2) {
                const int cx = MIN(px * j->sw / pw, j->sw - 1) & ~1;

                uv[(size_t)(py / 2) * pw + px] = j->uv[(size_t)cy * j->cs + cx];
                uv[(size_t)(py / 2) * pw + px + 1] =
                    j->uv[(size_t)cy * j->cs + cx + 1];
            }
        }
    }
    for (i = 0; i < XM_HEVC_REFS; i++) {
        if (j->p_slice && s->avc_refs[i].y && s->avc_refs[i].pa == j->ref_pa &&
            s->avc_refs[i].mbw == j->mbw && s->avc_refs[i].mbh == j->mbh) {
            ry = s->avc_refs[i].y;
            ruv = s->avc_refs[i].uv;
        }
    }
    H264Pic *pic = h264_pic_new(j->mbw, j->mbh, y, pw, uv, pw, ry, ruv,
                                s->avc.chroma_qp_offset, false);
    size_t bits = h264_pic_slice_cavlc(pic, j->p_slice, 0, j->mbh, j->qp,
                                       j->hdr_nbits, rbsp, j->cap, NULL);

    /* the coder leaves the bits before bit_offset as they were (zero):
     * they are the header's */
    rbsp[0] = (rbsp[0] & (0xff >> j->hdr_nbits)) | j->hdr_bits;
    const size_t nb = MIN(DIV_ROUND_UP(bits, 8), (size_t)j->cap);
    /* the header's last whole bytes end in at most one zero */
    j->n = xm_hevc_escape(rbsp, nb, 0, j->out, j->cap);
    if (!j->recon_pa) {
        h264_pic_free(pic);
        return;
    }
    for (i = 0; i < XM_HEVC_REFS; i++) {
        if (s->avc_refs[i].y && s->avc_refs[i].pa == j->recon_pa) {
            slot = i;
            break;
        }
        if (!s->avc_refs[i].y ||
            (s->avc_refs[slot].y && s->avc_refs[i].used < s->avc_refs[slot].used)) {
            slot = i;
        }
    }
    if (s->avc_refs[slot].mbw != j->mbw || s->avc_refs[slot].mbh != j->mbh) {
        g_free(s->avc_refs[slot].y);
        g_free(s->avc_refs[slot].uv);
        s->avc_refs[slot].y = g_malloc((size_t)pw * ph);
        s->avc_refs[slot].uv = g_malloc((size_t)pw * ph / 2);
        s->avc_refs[slot].mbw = j->mbw;
        s->avc_refs[slot].mbh = j->mbh;
    }
    h264_pic_finish(pic, &j->db, s->avc_refs[slot].y, s->avc_refs[slot].uv);
    h264_pic_free(pic);
    s->avc_refs[slot].pa = j->recon_pa;
    s->avc_refs[slot].used = ++s->ref_clock;
}

static void xm_hevc_work(gpointer data, gpointer user_data)
{
    XmHevcJob *j = data;
    XmHevcState *s = j->s;
    const HevcEncRef *ref = NULL;
    int i, slot = 0;

    if (j->gen != s->ref_gen) {             /* reset: forget them all */
        for (i = 0; i < XM_HEVC_REFS; i++) {
            hevc_enc_ref_free(s->refs[i].pic);
            s->refs[i].pic = NULL;
            s->refs[i].pa = 0;
            s->avc_refs[i].pa = 0;
        }
        s->ref_gen = j->gen;
    }
    if (j->h264) {
        xm_hevc_work_h264(j);
        g_async_queue_push(s->done, j);
        qemu_bh_schedule(s->done_bh);
        return;
    }
    for (i = 0; i < XM_HEVC_REFS; i++) {
        if (s->refs[i].pic && s->refs[i].pa == j->ref_pa && j->p_slice) {
            ref = s->refs[i].pic;
        }
    }
    if (!s->spare) {
        s->spare = hevc_enc_ref_new();
    }
    j->n = hevc_enc_slice_data(s->prm, j->w, j->h, j->qp, j->p_slice, ref,
                               j->recon_pa ? s->spare : NULL, j->y, j->sw,
                               j->sh, j->ys, j->uv, j->cs, j->out, j->cap);
    if (j->dump_path) {
        const size_t dcap = MIN(j->n, j->cap) + 256;
        g_autofree uint8_t *au = g_malloc(dcap);
        const size_t dn = hevc_enc_frame(s->prm, j->w, j->h, j->qp,
                                         j->p_slice, j->poc, ref, NULL,
                                         j->y, j->sw,
                                         j->sh, j->ys, j->uv, j->cs, au, dcap);
        g_file_set_contents(j->dump_path, (const char *)au, MIN(dn, dcap),
                            NULL);
    }
    if (j->recon_pa) {
        /* keep the picture under its buffer, in the slot that held that
         * buffer or the least recently used one */
        for (i = 0; i < XM_HEVC_REFS; i++) {
            if (s->refs[i].pic && s->refs[i].pa == j->recon_pa) {
                slot = i;
                break;
            }
            if (!s->refs[i].pic ||
                (s->refs[slot].pic && s->refs[i].used < s->refs[slot].used)) {
                slot = i;
            }
        }
        HevcEncRef *old = s->refs[slot].pic;
        s->refs[slot].pic = s->spare;
        s->refs[slot].pa = j->recon_pa;
        s->refs[slot].used = ++s->ref_clock;
        s->spare = old;
    }
    g_async_queue_push(s->done, j);
    qemu_bh_schedule(s->done_bh);
}

/* a job done: the status the layout's interrupt handler looks for */
static void xm_hevc_set_done(XmHevcState *s)
{
    if (s->xm232) {
        R(s, X2_END) = X2_ST_DONE;
        R(s, X2_JPEG_LEN) = 0;
    } else if (s->v3) {
        R(s, V3_STATUS) = ST_DONE;
    } else {
        R(s, R_STATUS) = ST_DONE;
    }
    xm_hevc_update(s);
}

static void xm_hevc_job_free(XmHevcJob *j)
{
    g_free(j->y);
    g_free(j->uv);
    g_free(j->out);
    g_free(j->dump_path);
    g_free(j);
}

/*
 * xm530v200 layout with +0x1C4 set (Sofia's DVR library on XM8521DV200):
 * the engine writes 16 bytes a 32x32 CTB of its own into the tables at
 * +0x20C (and +0x210), CTB columns outside, rows inside.  The library
 * marks them 0xff before the job and drops the frame (-10, silently) if a
 * marker is left; nothing reads the contents, so they are cleared here.
 */
/* the DVR library's parameter sets: 32x32 CTBs, no SAO, deblocking
 * offsets 5 / -5, a QP group a CTB */
static const HevcEncParams xm_hevc_params_dvr = {
    .log2_ctb = 5, .sao = false, .beta_offset_div2 = 5, .tc_offset_div2 = -5,
};

static void xm_hevc_ctb_tables(XmHevcState *s, int w, int h)
{
    static const uint8_t zero[4096];
    size_t len = (size_t)DIV_ROUND_UP(w, 32) * DIV_ROUND_UP(h, 32) * 16, k;
    hwaddr t[2];

    if (!s->v3 || s->v3_out != 0x200 || !R(s, 0x1C4)) {
        return;
    }
    t[0] = R(s, 0x20C);
    t[1] = R(s, 0x210);
    for (int i = 0; i < 2; i++) {
        for (k = 0; t[i] && k < len; k += sizeof(zero)) {
            dma_memory_write(&address_space_memory, t[i] + k, zero,
                             MIN(sizeof(zero), len - k),
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
}

/* the last H.264 job's dump: the library's headers, now in the gap before
 * its output (from the access unit's first start code), then the slice */
static void xm_hevc_avc_dump_flush(XmHevcState *s, int slot)
{
    enum { GAP = 256 };
    uint8_t hdr[GAP];
    size_t k;
    __typeof__(s->avc_streams[0]) *t = &s->avc_streams[slot];

    if (!t->dump_path) {
        return;
    }
    dma_memory_read(&address_space_memory, t->dump_out - GAP, hdr, GAP,
                    MEMTXATTRS_UNSPECIFIED);
    /* the picture's own NALs: its slice, the last NAL, and for an IDR the
     * SPS and PPS before it.  The gap can hold older pictures' NALs too (a
     * P picture's output is nearer the buffer's start than an IDR's). */
    size_t last = GAP, sps = GAP;
    for (k = 0; k + 3 < GAP; k++) {
        if (!hdr[k] && !hdr[k + 1] && hdr[k + 2] == 1) {
            last = k;
            if ((hdr[k + 3] & 0x1f) == 7) {
                sps = k;
            }
        }
    }
    if (last == GAP) {
        return;
    }
    k = (hdr[last + 3] & 0x1f) == 5 && sps < last ? sps : last;
    k = k && !hdr[k - 1] ? k - 1 : k;
    g_autofree uint8_t *au = g_malloc(GAP - k + t->dump_n);
    memcpy(au, hdr + k, GAP - k);
    memcpy(au + GAP - k, t->dump, t->dump_n);
    g_file_set_contents(t->dump_path, (const char *)au, GAP - k + t->dump_n,
                        NULL);
    g_clear_pointer(&t->dump_path, g_free);
    g_clear_pointer(&t->dump, g_free);
}

/* back in the main loop: the stream into guest memory, then done */
static void xm_hevc_done(void *opaque)
{
    XmHevcState *s = XM_HEVC(opaque);
    XmHevcJob *j;

    while ((j = g_async_queue_try_pop(s->done))) {
        if (j->gen == s->gen) {
            size_t n = j->n;

            if (n > j->cap) {
                qemu_log_mask(LOG_GUEST_ERROR, "xm-hevc: %zu bytes over the "
                              "%u-byte buffer\n", n, j->cap);
                n = j->cap;
            }
            dma_memory_write(&address_space_memory, j->out_pa, j->out, n,
                             MEMTXATTRS_UNSPECIFIED);
            if (j->h264 && j->dump_path) {
                xm_hevc_avc_dump_flush(s, j->stream_slot);
                s->avc_streams[j->stream_slot].dump_path =
                    g_steal_pointer(&j->dump_path);
                s->avc_streams[j->stream_slot].dump_out = j->out_pa;
                s->avc_streams[j->stream_slot].dump = g_memdup2(j->out, n);
                s->avc_streams[j->stream_slot].dump_n = n;
            }
            xm_hevc_ctb_tables(s, j->w, j->h);
            s->regs[j->len_reg / 4] = n;
            xm_hevc_set_done(s);
        }
        xm_hevc_job_free(j);
    }
}

/* a job: its source out of guest memory, then to the worker; a job the
 * engine cannot do completes at once with nothing written */
static void xm_hevc_encode_job(XmHevcState *s, int w, int h, int sw,
                               int sh, int ys, int cs, uint32_t y_pa,
                               uint32_t uv_pa, uint32_t out_pa, uint32_t cap,
                               bool p_slice, int qp, int stream,
                               uint32_t len_reg, const XmHevcJob *avc)
{
    if (w < 8 || h < 8 || w > 8192 || h > 8192 || (w & 7) || (h & 7) ||
        sw < 2 || sh < 2 || sw > 8192 || sh > 8192 || ys < sw || cs < sw ||
        ys > XM_HEVC_MAX_STRIDE || cs > XM_HEVC_MAX_STRIDE ||
        !y_pa || !uv_pa || !out_pa || !cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-hevc: job %dx%d from %dx%d "
                      "(strides %d/%d) to 0x%x+%u not done\n", w, h, sw, sh,
                      ys, cs, out_pa, cap);
        s->regs[len_reg / 4] = 0;
        xm_hevc_set_done(s);
        return;
    }
    XmHevcJob *j = g_new0(XmHevcJob, 1);
    uint32_t *poc = &s->poc[stream & 1];

    *poc = p_slice ? *poc + 1 : 0;
    *j = (XmHevcJob){ .s = s, .gen = s->gen, .w = w, .h = h, .sw = sw,
                      .sh = sh, .ys = ys, .cs = cs, .qp = qp, .poc = *poc,
                      .p_slice = p_slice, .out_pa = out_pa, .cap = cap,
                      .ref_pa = s->regs[s->ref_off / 4],
                      .recon_pa = s->regs[s->recon_off / 4],
                      .len_reg = len_reg };
    j->y = g_malloc((size_t)ys * sh);
    j->uv = g_malloc((size_t)cs * (sh / 2));
    if (avc) {
        j->h264 = true;
        j->mbw = avc->mbw;
        j->mbh = avc->mbh;
        j->hdr_bits = avc->hdr_bits;
        j->hdr_nbits = avc->hdr_nbits;
        j->db = avc->db;
        j->stream_slot = avc->stream_slot;
    }
    /* no more than the slice data can take: the guest's free count can be
     * anything (H.264: an I_PCM macroblock is 384 bytes) */
    j->cap = cap = MIN(cap, avc ? (uint32_t)(avc->mbw * avc->mbh * 400 + 4096)
                                : (uint32_t)((w / 8) * (h / 8) * 80 + 4096));
    j->out = g_malloc(cap);
    dma_memory_read(&address_space_memory, y_pa, j->y, (size_t)ys * sh,
                    MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, uv_pa, j->uv,
                    (size_t)cs * (sh / 2), MEMTXATTRS_UNSPECIFIED);
    if (s->xm232) {
        xm_hevc_osd232(s, j->y, j->uv, w, h, sw, sh, ys, cs);
    } else if (!s->v3) {
        xm_hevc_osd(s, j->y, j->uv, w, h, sw, sh, ys, cs);
    }
    if (s->dump_dir) {
        j->dump_path = avc ?
            g_strdup_printf("%s/frame-%05u-%dx%d.h264", s->dump_dir, s->jobs,
                            avc->mbw * 16, avc->mbh * 16) :
            g_strdup_printf("%s/frame-%05u.hevc", s->dump_dir, s->jobs);
    }
    s->jobs++;
    g_thread_pool_push(s->pool, j, NULL);
}

static void xm_hevc_encode(XmHevcState *s, int w, int h, int sw, int sh,
                           int ys, int cs, uint32_t y_pa, uint32_t uv_pa,
                           uint32_t out_pa, uint32_t cap, bool p_slice,
                           int qp, int stream, uint32_t len_reg)
{
    xm_hevc_encode_job(s, w, h, sw, sh, ys, cs, y_pa, uv_pa, out_pa, cap,
                       p_slice, qp, stream, len_reg, NULL);
}

/*
 * An xm232 job's codec.  The library writes its headers into the gap
 * before the output after the job, H.264 and H.265 alike, but an H.264
 * slice header seldom ends on a byte: the bits past its last whole byte
 * come before the job, in +0x52C / +0x530, and an H.265 one is byte
 * aligned.  So a job with bits there is H.264, and so is one whose gap
 * still holds an H.264 slice from the stream's last picture, or whose
 * stream (by its encoded size) was H.264 before.  The deblocking the
 * decoder runs comes from that last slice header (the engine's first byte
 * after it holds its last bits).
 */
typedef struct {
    const uint8_t *p;
    size_t n, pos;
} XmBits;

static int xm_bit(XmBits *b)
{
    int v;

    if (b->pos >= b->n * 8) {
        return 0;
    }
    v = (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
    b->pos++;
    return v;
}

static uint32_t xm_bits(XmBits *b, int n)
{
    uint32_t v = 0;

    while (n--) {
        v = v << 1 | xm_bit(b);
    }
    return v;
}

static uint32_t xm_ue(XmBits *b)
{
    int z = 0;

    while (!xm_bit(b) && z < 31) {
        z++;
    }
    return (1u << z) - 1 + xm_bits(b, z);
}

static int32_t xm_se(XmBits *b)
{
    const uint32_t k = xm_ue(b);

    return k & 1 ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

/* the slice header at buf[st] (its NAL header byte) for its deblocking */
static bool xm_hevc_avc_deblock(XmHevcState *s, const uint8_t *buf,
                                size_t len, size_t st, H264Deblock *db)
{
    g_autofree uint8_t *rbsp = g_malloc(len);
    size_t n = 0;
    const int nal = buf[st], type = nal & 0x1f;
    uint32_t slice_type;

    for (size_t k = st + 1; k < len; k++) {
        if (k >= st + 3 && buf[k] == 3 && !buf[k - 1] && !buf[k - 2]) {
            continue;
        }
        rbsp[n++] = buf[k];
    }
    XmBits b = { rbsp, n, 0 };

    *db = (H264Deblock){ 0 };
    xm_ue(&b);                                  /* first_mb_in_slice */
    slice_type = xm_ue(&b) % 5;
    xm_ue(&b);                                  /* pps id */
    xm_bits(&b, s->avc.have_sps ? s->avc.log2_max_frame_num : 4);
    if (type == 5) {
        xm_ue(&b);                              /* idr_pic_id */
    }
    if (s->avc.poc_type == 0) {
        xm_bits(&b, s->avc.log2_max_poc_lsb);
    }
    if (s->avc.redundant_pic_cnt_present) {
        xm_ue(&b);
    }
    if (slice_type == 0) {                      /* P */
        if (xm_bit(&b)) {                       /* num_ref_idx override */
            xm_ue(&b);
        }
        if (xm_bit(&b)) {                       /* ref_pic_list_modification */
            return false;
        }
    }
    if (nal & 0x60) {
        if (type == 5) {
            xm_bits(&b, 2);
        } else if (xm_bit(&b)) {                /* adaptive marking */
            return false;
        }
    }
    xm_se(&b);                                  /* slice_qp_delta */
    if (s->avc.deblocking_control) {
        db->idc = xm_ue(&b);
        if (db->idc != 1) {
            db->alpha_off = 2 * xm_se(&b);
            db->beta_off = 2 * xm_se(&b);
        }
    }
    return b.pos <= n * 8;
}

static bool xm_hevc_xm232_avc(XmHevcState *s, XmHevcJob *a, int ew, int eh)
{
    enum { GAP = 256 };
    uint8_t buf[GAP + 8];
    const uint32_t out = R(s, X2_OUT);
    size_t st = SIZE_MAX, k;
    int slot = -1, i;
    bool h264 = R(s, X2_HDR_NBITS) != 0;

    for (i = 0; i < ARRAY_SIZE(s->avc_streams); i++) {
        if (s->avc_streams[i].w == ew && s->avc_streams[i].h == eh) {
            slot = i;
        } else if (slot < 0 && !s->avc_streams[i].w) {
            slot = i;
        }
    }
    if (slot < 0) {
        slot = 0;
    }
    xm_hevc_avc_dump_flush(s, slot);
    a->stream_slot = slot;
    if (s->avc_streams[slot].w != ew || s->avc_streams[slot].h != eh) {
        s->avc_streams[slot].w = ew;
        s->avc_streams[slot].h = eh;
        s->avc_streams[slot].h264 = false;
        s->avc_streams[slot].db = (H264Deblock){ 0 };
    }
    if (out >= GAP) {
        dma_memory_read(&address_space_memory, out - GAP, buf, sizeof(buf),
                        MEMTXATTRS_UNSPECIFIED);
        for (k = GAP - 3; k > 0; k--) {         /* the gap's last NAL */
            if (!buf[k - 1] && !buf[k] && buf[k + 1] == 1) {
                st = k + 2;
                break;
            }
        }
    }
    if (st < GAP && !(buf[st] & 0x80) &&
        ((buf[st] & 0x1f) == 1 || (buf[st] & 0x1f) == 5)) {
        for (k = 0; k + 3 < st; k++) {          /* the SPS / PPS before */
            if (!buf[k] && !buf[k + 1] && buf[k + 2] == 1) {
                break;
            }
        }
        h264_enc_parse_headers(&s->avc, buf + k, st - k);
        xm_hevc_avc_deblock(s, buf, sizeof(buf), st,
                            &s->avc_streams[slot].db);
        h264 = true;
    }
    h264 |= s->avc_streams[slot].h264;
    s->avc_streams[slot].h264 = h264;
    if (!h264) {
        return false;
    }
    a->mbw = DIV_ROUND_UP(ew, 16);
    a->mbh = DIV_ROUND_UP(eh, 16);
    a->hdr_nbits = MIN(R(s, X2_HDR_NBITS), 7);
    a->hdr_bits = R(s, X2_HDR_BITS) & (0xff00 >> a->hdr_nbits);
    a->db = s->avc_streams[slot].db;
    return true;
}

static void xm_hevc_v3_job(XmHevcState *s)
{
    const int w = R(s, V3_ENC_W) + 1, h = R(s, V3_ENC_H) + 1;

    xm_hevc_encode(s, w, h, w, h, s->regs[s->v3_ys / 4],
                   s->regs[s->v3_cs / 4], R(s, V3_SRC_Y), R(s, V3_SRC_C),
                   s->regs[s->v3_out / 4], R(s, V3_OUT_FREE),
                   R(s, V3_SLICE_TYPE) == 1, MIN(R(s, V3_QP), 51), w >= 1280,
                   V3_OUT_LEN);
}

/* the picture is coded at the 8x8 minimum CU multiple the SPS gives; the
 * source size is +0x184 / +0x188 when the engine scales (+0x180) */
static void xm_hevc_xm232_job(XmHevcState *s)
{
    const int ew = R(s, X2_ENC_W) + 1, eh = R(s, X2_ENC_H) + 1;
    const int w = ROUND_UP(ew, 8), h = ROUND_UP(eh, 8);
    const bool scale = R(s, X2_SCALE) & 1;
    XmHevcJob avc;
    bool h264;

    h264 = xm_hevc_xm232_avc(s, &avc, ew, eh);

    xm_hevc_encode_job(s, w, h, scale ? R(s, X2_SRC_W) + 1 : ew,
                       scale ? R(s, X2_SRC_H) + 1 : eh, R(s, X2_SRC_YS),
                       R(s, X2_SRC_CS), R(s, X2_SRC_Y), R(s, X2_SRC_C),
                       R(s, X2_OUT), R(s, X2_OUT_FREE), R(s, X2_P_SLICE) != 0,
                       MIN(R(s, X2_QP), 51), w >= 1280, X2_OUT_LEN,
                       h264 ? &avc : NULL);
}

static void xm_hevc_job(void *opaque)
{
    XmHevcState *s = XM_HEVC(opaque);

    if (s->xm232) {
        xm_hevc_xm232_job(s);
        return;
    }
    if (s->v3) {
        xm_hevc_v3_job(s);
        return;
    }
    const int w = R(s, R_ENC_W) + 1, h = R(s, R_ENC_H) + 1;
    const int sw = R(s, R_SRC_W) + 1, sh = R(s, R_SRC_H) + 1;

    xm_hevc_encode(s, w, h, sw, sh, R(s, R_SRC_YS) ? R(s, R_SRC_YS) : sw,
                   R(s, R_SRC_CS) ? R(s, R_SRC_CS) : sw, R(s, R_SRC_Y),
                   R(s, R_SRC_C), R(s, R_OUT), R(s, R_OUT_FREE),
                   R(s, R_SLICE_TYPE) == 1, MIN(R(s, R_QP), 51),
                   R(s, R_MAIN), R_OUT_LEN);
}

static uint64_t xm_hevc_read(void *opaque, hwaddr off, unsigned size)
{
    XmHevcState *s = XM_HEVC(opaque);

    if (s->xm232) {
        return off == X2_BUSY ? 0 : s->regs[off / 4];
    }
    if (s->v3) {
        return s->regs[off / 4];
    }
    switch (off) {
    case 0x068:
    case 0x06C:
    case 0x070:
        return 0;
    default:
        return s->regs[off / 4];
    }
}

static void xm_hevc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    XmHevcState *s = XM_HEVC(opaque);

    if (s->xm232) {
        switch (off) {
        case X2_START:
            s->regs[off / 4] = val;
            if (val & 1) {
                R(s, X2_END) = 0;
                timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          SCALE_MS);
            }
            break;
        case X2_CLR_AUX:
            R(s, X2_AUX) &= ~(uint32_t)val;
            break;
        case X2_CLR_ERR:
            R(s, X2_ERR) &= ~(uint32_t)val;
            break;
        case X2_CLR_BGN:
            R(s, X2_BGN) &= ~(uint32_t)val;
            break;
        case X2_CLR_HALT:
            R(s, X2_HALT) &= ~(uint32_t)val;
            break;
        case X2_CLR_END:
            R(s, X2_END) &= ~(uint32_t)val;
            break;
        case X2_AUX:
        case X2_ERR:
        case X2_BGN:
        case X2_HALT:
        case X2_END:
        case X2_OUT_LEN:
        case X2_JPEG_LEN:
            break;
        default:
            s->regs[off / 4] = val;
            break;
        }
        xm_hevc_update(s);
        return;
    }
    if (s->v3) {
        switch (off) {
        case V3_START:
            s->regs[off / 4] = val;
            if (val & 1) {
                timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          SCALE_MS);
            }
            break;
        case V3_ACK:
            R(s, V3_STATUS) &= ~(uint32_t)val;
            break;
        case V3_ACK2:
            R(s, V3_ST2) &= ~(uint32_t)val;
            break;
        case 0x028:
            R(s, V3_ERR) &= ~(uint32_t)val;
            break;
        case V3_STATUS:
        case V3_ST2:
        case V3_ERR:
        case V3_OUT_LEN:
            break;
        default:
            s->regs[off / 4] = val;
            break;
        }
        xm_hevc_update(s);
        return;
    }
    switch (off) {
    case R_START:
        s->regs[off / 4] = val;
        if (val & 1) {
            R(s, R_STATUS) = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SCALE_MS);
        }
        break;
    case R_ACK:
        s->regs[off / 4] = val;
        R(s, R_STATUS) &= ~(uint32_t)val;
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
    xm_hevc_update(s);
}

static const MemoryRegionOps xm_hevc_ops = {
    .read = xm_hevc_read,
    .write = xm_hevc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t xm_hevc2_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_HEVC(opaque)->regs2[off / 4];
}

/*
 * The second window is the OSD engine rgn.ko's compressed-frame path
 * drives on the v3 engines (XM8521DV200 / XM8536E): +0x00 bit 0 starts a
 * job, and it reports done in +0x0C on the encoder's interrupt, which
 * hevc_interrupt checks first and clears by writing 0.  Jobs complete at
 * once; nothing is drawn.
 */
static void xm_hevc2_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    XmHevcState *s = XM_HEVC(opaque);

    s->regs2[off / 4] = val;
    if (s->v3 && off == 0 && (val & 1)) {
        s->regs2[0x0C / 4] = 1;
    }
    if (s->v3) {
        xm_hevc_update(s);
    }
}

static const MemoryRegionOps xm_hevc2_ops = {
    .read = xm_hevc2_read,
    .write = xm_hevc2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_hevc_reset(DeviceState *dev)
{
    XmHevcState *s = XM_HEVC(dev);

    timer_del(s->timer);
    s->gen++;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs2, 0, sizeof(s->regs2));
    /* the H.264 streams start over: codec, deblocking, pending dumps (the
     * worker drops its references itself when it sees the new gen) */
    for (int i = 0; i < ARRAY_SIZE(s->avc_streams); i++) {
        g_free(s->avc_streams[i].dump_path);
        g_free(s->avc_streams[i].dump);
    }
    memset(s->avc_streams, 0, sizeof(s->avc_streams));
    xm_hevc_update(s);
}

static void xm_hevc_init(Object *obj)
{
    XmHevcState *s = XM_HEVC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &xm_hevc_ops, s, TYPE_XM_HEVC,
                          XM_HEVC_SIZE);
    memory_region_init_io(&s->iomem2, obj, &xm_hevc2_ops, s,
                          TYPE_XM_HEVC ".2", XM_HEVC2_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->iomem2);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_hevc_job, s);
    s->done = g_async_queue_new();
    s->done_bh = qemu_bh_new(xm_hevc_done, s);
    /* one worker: jobs complete in order, as the engine's would */
    s->pool = g_thread_pool_new(xm_hevc_work, NULL, 1, FALSE, NULL);
}

static void xm_hevc_realize(DeviceState *dev, Error **errp)
{
    XmHevcState *s = XM_HEVC(dev);

    s->prm = &hevc_params_ctb16;
    s->ref_off = 0x1E8;
    s->recon_off = 0x1F0;
    if (!s->layout || !strcmp(s->layout, "xm535")) {
        s->v3 = false;
        s->ref_off = 0x0A8;
        s->recon_off = 0x0B0;
    } else if (!strcmp(s->layout, "xm533")) {
        s->v3 = true;
        s->v3_out = 0x210;
        s->v3_ys = 0x224;
        s->v3_cs = 0x228;
    } else if (!strcmp(s->layout, "xm530v200") ||
               !strcmp(s->layout, "xm8521dv200")) {
        s->v3 = true;
        s->v3_out = 0x200;
        s->v3_ys = 0x214;
        s->v3_cs = 0x218;
        s->prm = strcmp(s->layout, "xm530v200") ? &xm_hevc_params_dvr :
                 &hevc_params_ctb32_sao;
    } else if (!strcmp(s->layout, "xm232")) {
        s->xm232 = true;
        s->ref_off = 0x114;
        s->recon_off = 0x11C;
    } else {
        error_setg(errp, "xm-hevc: unknown layout '%s'", s->layout);
    }
}

static const Property xm_hevc_props[] = {
    DEFINE_PROP_STRING("dump-dir", XmHevcState, dump_dir),
    DEFINE_PROP_STRING("layout", XmHevcState, layout),
};

static void xm_hevc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_hevc_reset);
    dc->realize = xm_hevc_realize;
    device_class_set_props(dc, xm_hevc_props);
}

static const TypeInfo xm_hevc_info = {
    .name          = TYPE_XM_HEVC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmHevcState),
    .instance_init = xm_hevc_init,
    .class_init    = xm_hevc_class_init,
};

static void xm_hevc_register_types(void)
{
    type_register_static(&xm_hevc_info);
}

type_init(xm_hevc_register_types)
