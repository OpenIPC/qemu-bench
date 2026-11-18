/*
 * Grain Media FTMCP280 H.264 encoder.
 *
 * The encoder at 0x92C00000 (and its reference cache, VCACHE, at
 * 0x92D00000) on the GM813x, driven by the SDK's favc_enc.ko
 * (module/ftmcp280: enc_driver/mcp280_reg.h, encoder.c, slice.c, vlc.c,
 * params.c, slice_header.c; favc_enc_vg.c).
 *
 * The driver writes the SPS, PPS and every slice header itself, element
 * by element, through the header packer: up to 32 {control, value} pairs
 * at 0x400, control bits 5:0 the length of a fixed-length element, 6
 * emulation prevention on (off only for start codes and NAL header
 * bytes), 7 Exp-Golomb, 8 rbsp_trailing_bits after it, 9 the last pair;
 * CMD2 (0x88) bit 12 packs them and STS0 (0xC0) bit 1 says done.  The
 * bits go to the bitstream buffer (BS_ADDR 0x148, BS_BUF_SIZE 0x144) at
 * the byte count STS1[23:0], which CMD2 bit 11 clears; the bit position
 * and the emulation-prevention state carry on from one batch to the next
 * and into the slice data.  CMD0 (0x80) bit 0 then encodes a slice: the
 * hardware continues the same bitstream with the slice data -- after
 * cabac_alignment_one_bits in CABAC -- and the slice's trailing bits.
 *
 * The slice comes from the registers:
 *   PARM0 0x00  [31:24] last / [23:16] first MB row, [15:8] width in MBs
 *               - 1, [7:0] source stride / 16 - 1
 *   PARM1 0x04  [31:24] height in MBs - 1, [13:12] slice type (0 P, 1 B,
 *               2 I)
 *   PARM3 0x0C  [31:26] QP, [20:16] chroma QP offset
 *   PARM6 0x18  [2:1] cabac_init_idc, [0] CABAC
 *   DIDN_CUR_TOP 0x100: the slice's first source row, packed 4:2:2
 *   (UYVY; YUYV when SWAP_DMA 0x180 bit 0 is set), stride source width * 2
 * I slices are I_16x16 DC macroblocks; P slices are inter coded -- P_Skip,
 * P_L0_16x16 with a quarter-sample motion search and a residual, intra
 * where nothing matches (hw/misc/hantro-h264-inter.c.inc) -- against the
 * decoder's picture, deblocked as the slice header asks (the model reads
 * the SPS, PPS and slice header it packed); B slices skip everything.  A
 * picture's slices share one picture coder, the first reading the whole
 * source (CUR_TOP less the rows above: inferred contiguous); at the last
 * slice (PARM5 bit 31) the decoder's picture goes to the reconstruction
 * buffer (REC 0x120 / 0x124 at the slice's first row), which the next
 * picture reads back through ME_REF 0x108 / 0x10C.  A header the model
 * cannot follow falls back to the old coding (I_PCM where the picture
 * changed).
 *
 * Status: STS0 0xC0 bit 3 DMA idle and bit 1 packer done (always), STS1
 * 0xC4 [23:0] the byte count, [25:24] free input buffers; STS2 0xC8 [4:0]
 * slice done / bitstream full / DMA errors / MB timeout, write 1 to
 * clear, [20:16] their masks (1 = masked); STS3 0xCC a cost, STS4 0xD0
 * the slice's QP sum.  A slice finishes a millisecond after its kick on
 * interrupt 29.  "-global gm-h264e.dump=DIR" appends each picture's output
 * to DIR/gm-h264e-<width>x<height>.h264, one file per stream size.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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
#include "hantro-h264.h"

#define TYPE_GM_H264E "gm-h264e"
OBJECT_DECLARE_SIMPLE_TYPE(GmH264eState, GM_H264E)

#define GM_H264E_SIZE       0x1000
#define GM_H264E_NREGS      (GM_H264E_SIZE / 4)
#define GM_VCACHE_SIZE      0x1000
#define GM_VCACHE_NREGS     (GM_VCACHE_SIZE / 4)
#define GM_H264E_SLICE_NS   1000000

#define R_PARM0             0x00
#define R_PARM1             0x04
#define R_PARM3             0x0C
#define R_PARM6             0x18
#define R_CMD0              0x80
#define R_CMD2              0x88
#define R_STS0              0xC0
#define R_STS1              0xC4
#define R_STS2              0xC8
#define R_STS3              0xCC
#define R_STS4              0xD0
#define R_CUR_TOP           0x100
#define R_ME_REF_LUMA       0x108
#define R_ME_REF_CHROMA     0x10C
#define R_REC_LUMA          0x120
#define R_REC_CHROMA        0x124
#define R_BS_SIZE           0x144
#define R_BS_ADDR           0x148
#define R_SWAP_DMA          0x180
#define R_SLICE_HDR         0x400
#define SLICE_HDR_PAIRS     32

#define CMD0_GO             (1u << 0)
#define CMD0_DMA_OFF        (1u << 2)
#define CMD0_RESET          (1u << 31)
#define CMD2_CLOSE          (1u << 10)
#define CMD2_CLEAR          (1u << 11)
#define CMD2_PACK           (1u << 12)

#define STS0_PACKED         (1u << 1)
#define STS0_DMA_IDLE       (1u << 3)

#define IRQ_SLICE_DONE      (1u << 0)
#define IRQ_BS_FULL         (1u << 1)

#define HDR_EP              (1u << 6)
#define HDR_UE              (1u << 7)
#define HDR_TRAILING        (1u << 8)
#define HDR_LAST            (1u << 9)

#define VC_DECODE           0x04

/* REC address the driver gives a picture that is not a reference */
#define GM_H264E_NO_REC     0xC0000000

struct GmH264eState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion vcache_mem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[GM_H264E_NREGS];
    uint32_t vcache[GM_VCACHE_NREGS];
    uint32_t irq_status;
    uint32_t irq_mask;
    uint32_t pending;       /* status the slice in flight will report */

    /* the bitstream writer */
    uint32_t bs_count;      /* bytes written since the last clear */
    uint32_t bs_acc;        /* bits not yet a byte, MSB first */
    uint32_t bs_nbits;
    uint32_t bs_zeros;      /* zero bytes just written, for 00 00 03 */
    bool bs_full;

    /* inter coding: the parameter sets seen, the picture being coded */
    H264Enc hdr;
    H264Pic *pic;
    uint8_t *pic_y, *pic_uv;    /* its source, NV12 (the coder reads it) */
    int pic_mbw, pic_mbh;
    H264Deblock db;

    char *dump;
};

static void gm_h264e_update(GmH264eState *s)
{
    qemu_set_irq(s->irq, (s->irq_status & ~s->irq_mask & 0x1F) != 0);
}

/* ---- the bitstream writer ---------------------------------------------- */

static void gm_h264e_put_byte(GmH264eState *s, uint8_t b, bool ep)
{
    const uint32_t size = s->regs[R_BS_SIZE / 4];
    const hwaddr base = s->regs[R_BS_ADDR / 4];

    if (ep && s->bs_zeros >= 2 && b <= 3) {
        gm_h264e_put_byte(s, 3, false);
    }
    if (s->bs_count < size) {
        dma_memory_write(&address_space_memory, base + s->bs_count, &b, 1,
                         MEMTXATTRS_UNSPECIFIED);
    } else {
        s->bs_full = true;
    }
    s->bs_count++;
    /* zeros written outside emulation prevention (start codes) don't count */
    s->bs_zeros = (ep && !b) ? s->bs_zeros + 1 : 0;
}

static void gm_h264e_put_bits(GmH264eState *s, uint32_t v, int n, bool ep)
{
    while (n--) {
        s->bs_acc = (s->bs_acc << 1) | ((v >> n) & 1);
        if (++s->bs_nbits == 8) {
            gm_h264e_put_byte(s, s->bs_acc, ep);
            s->bs_acc = s->bs_nbits = 0;
        }
    }
}

static void gm_h264e_put_ue(GmH264eState *s, uint32_t v, bool ep)
{
    const uint64_t x = (uint64_t)v + 1;
    const int len = 64 - clz64(x);

    gm_h264e_put_bits(s, 0, len - 1, ep);
    if (len > 32) {
        gm_h264e_put_bits(s, x >> 32, len - 32, ep);
        gm_h264e_put_bits(s, x, 32, ep);
    } else {
        gm_h264e_put_bits(s, x, len, ep);
    }
}

static void gm_h264e_trailing(GmH264eState *s, bool ep)
{
    gm_h264e_put_bits(s, 1, 1, ep);
    while (s->bs_nbits) {
        gm_h264e_put_bits(s, 0, 1, ep);
    }
}

static void gm_h264e_clear(GmH264eState *s)
{
    s->bs_count = 0;
    s->bs_acc = s->bs_nbits = 0;
    s->bs_zeros = 0;
    s->bs_full = false;
}

/* CMD2 bit 12: the header packer */
static void gm_h264e_pack(GmH264eState *s)
{
    int i;

    for (i = 0; i < SLICE_HDR_PAIRS; i++) {
        const uint32_t ctl = s->regs[R_SLICE_HDR / 4 + 2 * i];
        const uint32_t val = s->regs[R_SLICE_HDR / 4 + 2 * i + 1];
        const bool ep = ctl & HDR_EP;

        if (ctl & HDR_UE) {
            gm_h264e_put_ue(s, val, ep);
        } else {
            int len = ctl & 0x3F;

            if (len > 32) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "gm-h264e: element of %d bits\n", len);
                len = 32;
            }
            if (len == 32) {
                gm_h264e_put_bits(s, val >> 16, 16, ep);
                gm_h264e_put_bits(s, val & 0xFFFF, 16, ep);
            } else {
                gm_h264e_put_bits(s, val, len, ep);
            }
        }
        if (ctl & HDR_TRAILING) {
            gm_h264e_trailing(s, ep);
        }
        if (ctl & HDR_LAST) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "gm-h264e: header batch without an end\n");
}

/* ---- slices -------------------------------------------------------------- */

static void gm_h264e_dump(GmH264eState *s)
{
    g_autofree char *path = NULL;
    g_autofree uint8_t *buf = NULL;
    const uint32_t n = MIN(s->bs_count, s->regs[R_BS_SIZE / 4]);
    FILE *f;

    if (!s->dump || !n) {
        return;
    }
    buf = g_malloc(n);
    dma_memory_read(&address_space_memory, s->regs[R_BS_ADDR / 4], buf, n,
                    MEMTXATTRS_UNSPECIFIED);
    /* one file per picture size: the main and sub streams interleave */
    path = g_strdup_printf("%s/gm-h264e-%ux%u.h264", s->dump,
                           (extract32(s->regs[R_PARM0 / 4], 8, 8) + 1) * 16,
                           (extract32(s->regs[R_PARM1 / 4], 24, 8) + 1) * 16);
    f = fopen(path, "ab");
    if (f) {
        fwrite(buf, 1, n, f);
        fclose(f);
    }
}

/* bits [from, to) of out, most significant first */
static void gm_h264e_emit(GmH264eState *s, const uint8_t *out, size_t from,
                          size_t to, size_t cap)
{
    size_t i;

    to = MIN(to, cap * 8);
    for (i = from; i < to; i++) {
        gm_h264e_put_bits(s, (out[i / 8] >> (7 - i % 8)) & 1, 1, true);
    }
}

/*
 * What the slice data must agree with is in the headers the driver packed:
 * the SPS / PPS (parsed as they pass, kept across pictures) and this
 * slice's header, which ends just before the slice data -- its
 * disable_deblocking_filter_idc and the filter offsets.
 */
typedef struct GmBits {
    const uint8_t *p;
    size_t n, pos;              /* bits */
} GmBits;

static uint32_t gm_u(GmBits *b, int n)
{
    uint32_t v = 0;

    while (n--) {
        v <<= 1;
        if (b->pos < b->n) {
            v |= (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
        }
        b->pos++;
    }
    return v;
}

static uint32_t gm_ue(GmBits *b)
{
    int z = 0;

    while (z < 32 && b->pos < b->n && !gm_u(b, 1)) {
        z++;
    }
    return z >= 32 ? 0 : ((1u << z) - 1) + gm_u(b, z);
}

static int gm_se(GmBits *b)
{
    uint32_t k = gm_ue(b);

    return k & 1 ? (int)((k + 1) / 2) : -(int)(k / 2);
}

/* The current slice's header: the deblocking it asks for; false if unread */
static bool gm_h264e_slice_header(GmH264eState *s, H264Deblock *db)
{
    const uint32_t size = s->regs[R_BS_SIZE / 4];
    const uint32_t len = MIN(s->bs_count, size);
    g_autofree uint8_t *buf = NULL, *rbsp = NULL;
    size_t i, start = 0, n = 0;
    int nal_type, nal_ref, type;
    GmBits b;
    bool found = false;

    if (!len) {
        return false;
    }
    buf = g_malloc(len + 1);
    dma_memory_read(&address_space_memory, s->regs[R_BS_ADDR / 4], buf, len,
                    MEMTXATTRS_UNSPECIFIED);
    h264_enc_parse_headers(&s->hdr, buf, len);
    if (!s->hdr.have_sps || !s->hdr.have_pps) {
        return false;
    }
    for (i = 0; i + 3 <= len; i++) {        /* the last start code */
        if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1) {
            start = i + 3;
            found = true;
        }
    }
    if (!found || start >= len) {
        return false;
    }
    /* unescape, then the bits not yet a byte */
    rbsp = g_malloc(len - start + 1);
    for (i = start; i < len; i++) {
        if (i >= start + 2 && buf[i] == 3 && !buf[i - 1] && !buf[i - 2]) {
            continue;
        }
        rbsp[n++] = buf[i];
    }
    nal_ref = (rbsp[0] >> 5) & 3;
    nal_type = rbsp[0] & 31;
    if (nal_type != 1 && nal_type != 5) {
        return false;
    }
    rbsp[n] = s->bs_acc << (8 - s->bs_nbits);
    b = (GmBits){ rbsp + 1, (n - 1) * 8 + s->bs_nbits, 0 };
    gm_ue(&b);                                  /* first_mb_in_slice */
    type = gm_ue(&b) % 5;                       /* 0 P, 1 B, 2 I */
    gm_ue(&b);                                  /* pic_parameter_set_id */
    gm_u(&b, s->hdr.log2_max_frame_num);
    if (nal_type == 5) {
        gm_ue(&b);                              /* idr_pic_id */
    }
    if (s->hdr.poc_type == 0) {
        gm_u(&b, s->hdr.log2_max_poc_lsb);
        if (s->hdr.bottom_field_poc_present) {
            gm_se(&b);
        }
    } else if (s->hdr.poc_type == 1) {
        return false;                           /* not seen from the driver */
    }
    if (s->hdr.redundant_pic_cnt_present) {
        gm_ue(&b);
    }
    if (type == 1) {
        gm_u(&b, 1);                            /* direct_spatial_mv_pred */
    }
    if (type != 2) {
        if (gm_u(&b, 1)) {                      /* num_ref_idx override */
            gm_ue(&b);
            if (type == 1) {
                gm_ue(&b);
            }
        }
        for (i = 0; i < (type == 1 ? 2 : 1); i++) {
            if (gm_u(&b, 1)) {                  /* ref_pic_list_modification */
                uint32_t op;

                do {
                    op = gm_ue(&b);
                    if (op != 3) {
                        gm_ue(&b);
                    }
                } while (op != 3 && b.pos < b.n);
            }
        }
    }
    if (nal_ref) {                              /* dec_ref_pic_marking */
        if (nal_type == 5) {
            gm_u(&b, 2);
        } else if (gm_u(&b, 1)) {
            uint32_t op;

            do {
                op = gm_ue(&b);
                if (op == 1 || op == 3) {
                    gm_ue(&b);
                }
                if (op == 2 || op == 3 || op == 6) {
                    gm_ue(&b);
                }
                if (op == 4) {
                    gm_ue(&b);
                }
            } while (op && b.pos < b.n);
        }
    }
    if (s->hdr.cabac && type != 2) {
        gm_ue(&b);                              /* cabac_init_idc */
    }
    gm_se(&b);                                  /* slice_qp_delta */
    db->idc = 0;
    db->alpha_off = db->beta_off = 0;
    if (s->hdr.deblocking_control) {
        db->idc = gm_ue(&b);
        if (db->idc != 1) {
            db->alpha_off = 2 * gm_se(&b);
            db->beta_off = 2 * gm_se(&b);
        }
    }
    return b.pos <= b.n && db->idc <= 2;
}

static void gm_h264e_drop_pic(GmH264eState *s)
{
    h264_pic_free(s->pic);
    s->pic = NULL;
    g_free(s->pic_y);
    g_free(s->pic_uv);
    s->pic_y = s->pic_uv = NULL;
}

/* 4:2:2 packed rows (stride bytes) to NV12 (w wide): chroma from even lines */
static void gm_h264e_to_nv12(const uint8_t *src, int stride, bool yuyv, int w,
                             int h, uint8_t *y, uint8_t *uv)
{
    int r, x;

    for (r = 0; r < h; r++) {
        const uint8_t *l = src + (size_t)r * stride;

        for (x = 0; x < w; x += 2) {
            const uint8_t *px = l + x * 2;

            y[r * w + x] = yuyv ? px[0] : px[1];
            y[r * w + x + 1] = yuyv ? px[2] : px[3];
            if (!(r & 1)) {
                uv[(r / 2) * w + x] = yuyv ? px[1] : px[0];
                uv[(r / 2) * w + x + 1] = yuyv ? px[3] : px[2];
            }
        }
    }
}

static void gm_h264e_encode_slice(GmH264eState *s)
{
    const uint32_t p0 = s->regs[R_PARM0 / 4], p1 = s->regs[R_PARM1 / 4];
    const uint32_t p3 = s->regs[R_PARM3 / 4], p6 = s->regs[R_PARM6 / 4];
    const int mbw = extract32(p0, 8, 8) + 1;
    const int first = extract32(p0, 16, 8), last = extract32(p0, 24, 8);
    const int stride = (extract32(p0, 0, 8) + 1) * 16 * 2;
    const int rows = last >= first ? last - first + 1 : 0;
    const int pic_mbh = extract32(p1, 24, 8) + 1;
    const int type = extract32(p1, 12, 2);
    const int qp = extract32(p3, 26, 6);
    const int cqp = sextract32(p3, 16, 5);
    const int idc = extract32(p6, 1, 2);
    const bool cabac = p6 & 1;
    const bool yuyv = s->regs[R_SWAP_DMA / 4] & 1;
    const bool last_slice = s->regs[0x14 / 4] & (1u << 31);
    const int nmb = mbw * rows, w = mbw * 16, h = rows * 16;
    const hwaddr rec_y = s->regs[R_REC_LUMA / 4];
    const hwaddr rec_uv = s->regs[R_REC_CHROMA / 4];
    const uint32_t size = s->regs[R_BS_SIZE / 4];
    g_autofree uint8_t *src = NULL, *y = NULL, *uv = NULL;
    g_autofree uint8_t *ry = NULL, *ruv = NULL, *out = NULL;
    const int off = s->bs_nbits;            /* where the header left off */
    H264PStats st = { .qp = qp };
    H264Deblock db;
    size_t cap, n, from = 0;
    bool inter = false;

    if (!nmb) {
        return;
    }
    if (stride < w * 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "gm-h264e: source stride %d below "
                      "the picture's %d bytes\n", stride, w * 2);
        return;
    }
    /*
     * I and P slices are coded against the decoder's picture when this
     * slice's header can be read (it always can for what favc_enc packs):
     * a picture of slices kept across the kicks, its first slice reading
     * the whole source (CUR_TOP less the rows above: the slices are one
     * contiguous frame -- inferred) and the reference at ME_REF.
     */
    if (type != 1 && gm_h264e_slice_header(s, &db)) {
        if (!s->pic || first == 0 || s->pic_mbw != mbw ||
            s->pic_mbh != pic_mbh) {
            const size_t pw = (size_t)mbw * 16, ph = (size_t)pic_mbh * 16;
            g_autofree uint8_t *whole = g_malloc(stride * ph);
            g_autofree uint8_t *py = NULL, *puv = NULL;

            gm_h264e_drop_pic(s);
            dma_memory_read(&address_space_memory,
                            s->regs[R_CUR_TOP / 4] -
                            (hwaddr)first * 16 * stride,
                            whole, stride * ph, MEMTXATTRS_UNSPECIFIED);
            s->pic_y = g_malloc(pw * ph);
            s->pic_uv = g_malloc(pw * ph / 2);
            gm_h264e_to_nv12(whole, stride, yuyv, pw, ph, s->pic_y,
                             s->pic_uv);
            if (type == 0) {
                py = g_malloc(pw * ph);
                puv = g_malloc(pw * ph / 2);
                dma_memory_read(&address_space_memory,
                                s->regs[R_ME_REF_LUMA / 4], py, pw * ph,
                                MEMTXATTRS_UNSPECIFIED);
                dma_memory_read(&address_space_memory,
                                s->regs[R_ME_REF_CHROMA / 4], puv,
                                pw * ph / 2, MEMTXATTRS_UNSPECIFIED);
            }
            s->pic = h264_pic_new(mbw, pic_mbh, s->pic_y, pw, s->pic_uv, pw,
                                  py, puv, cqp, false);
            s->pic_mbw = mbw;
            s->pic_mbh = pic_mbh;
        }
        s->db = db;
        inter = true;
    }
    if (cabac) {
        while (s->bs_nbits) {               /* cabac_alignment_one_bit */
            gm_h264e_put_bits(s, 1, 1, true);
        }
    }
    if (inter) {
        /* the room left; the coder raises the QP rather than overflow */
        cap = size > s->bs_count + 64 ? size - s->bs_count - 32 : 64;
        out = g_malloc0(cap + 64);
        if (cabac) {
            n = h264_pic_slice_cabac(s->pic, type == 0, first, rows, qp, idc,
                                     out, cap, &st);
        } else {
            n = h264_pic_slice_cavlc(s->pic, type == 0, first, rows, qp, off,
                                     out, cap, &st);
        }
        n = MIN(n, cabac ? cap + 64 : (cap + 64) * 8);
    } else {
        /* B, or a header the model could not follow: the plain coders */
        src = g_malloc((size_t)stride * h);
        y = g_malloc((size_t)w * h);
        uv = g_malloc((size_t)w * h / 2);
        dma_memory_read(&address_space_memory, s->regs[R_CUR_TOP / 4], src,
                        (size_t)stride * h, MEMTXATTRS_UNSPECIFIED);
        gm_h264e_to_nv12(src, stride, yuyv, w, h, y, uv);
        cap = (size_t)nmb * 400 + 64;       /* PCM can take every MB */
        out = g_malloc0(cap);
        if (type == 2) {
            if (cabac) {
                n = h264_slice_i16dc_cabac(qp, cqp, mbw, rows, y, w, uv, w,
                                           out, cap);
            } else {
                n = h264_slice_i16dc_cavlc(qp, cqp, mbw, rows, y, w, uv, w,
                                           off, out, cap);
            }
            ry = g_memdup2(y, (size_t)w * h);
            ruv = g_memdup2(uv, (size_t)w * h / 2);
        } else if (type == 0) {
            ry = g_malloc((size_t)w * h);
            ruv = g_malloc((size_t)w * h / 2);
            dma_memory_read(&address_space_memory,
                            s->regs[R_ME_REF_LUMA / 4] +
                            (hwaddr)first * w * 16,
                            ry, (size_t)w * h, MEMTXATTRS_UNSPECIFIED);
            dma_memory_read(&address_space_memory,
                            s->regs[R_ME_REF_CHROMA / 4] +
                            (hwaddr)first * w * 8,
                            ruv, (size_t)w * h / 2, MEMTXATTRS_UNSPECIFIED);
            if (cabac) {
                n = h264_slice_ppcm_cabac(qp, idc, mbw, rows, y, w, uv, w, ry,
                                          ruv, out, cap);
            } else {
                n = h264_slice_ppcm_cavlc(mbw, rows, y, w, uv, w, ry, ruv,
                                          off, out, cap);
            }
        } else if (cabac) {                 /* B: nothing changes */
            /* B's skip flags have contexts of their own: not modelled */
            qemu_log_mask(LOG_UNIMP, "gm-h264e: CABAC B slice\n");
            n = h264_slice_pskip_cabac(qp, idc, nmb, out, cap);
        } else {
            n = h264_slice_pskip_cavlc(nmb, off, out, cap);
        }
    }
    if (cabac) {
        n *= 8;                             /* bytes, from a byte boundary */
    } else {
        from = off;                         /* past the header's bits */
    }
    gm_h264e_emit(s, out, from, n, inter ? cap + 64 : cap);
    /*
     * the reconstruction buffer, unless the picture is not a reference;
     * the driver gives REC at the slice's first row, ME_REF at the picture
     */
    if (inter && (last_slice || first + rows >= s->pic_mbh)) {
        const size_t pw = (size_t)mbw * 16, ph = (size_t)s->pic_mbh * 16;
        g_autofree uint8_t *fy = g_malloc(pw * ph);
        g_autofree uint8_t *fuv = g_malloc(pw * ph / 2);

        h264_pic_finish(s->pic, &s->db, fy, fuv);
        if (rec_y && rec_y != GM_H264E_NO_REC) {
            dma_memory_write(&address_space_memory,
                             rec_y - (hwaddr)first * pw * 16, fy, pw * ph,
                             MEMTXATTRS_UNSPECIFIED);
            dma_memory_write(&address_space_memory,
                             rec_uv - (hwaddr)first * pw * 8, fuv,
                             pw * ph / 2, MEMTXATTRS_UNSPECIFIED);
        }
        gm_h264e_drop_pic(s);
    } else if (!inter && ry && rec_y && rec_y != GM_H264E_NO_REC) {
        dma_memory_write(&address_space_memory, rec_y, ry, (size_t)w * h,
                         MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, rec_uv, ruv,
                         (size_t)w * h / 2, MEMTXATTRS_UNSPECIFIED);
    }
    s->regs[R_STS3 / 4] = nmb * 64;
    s->regs[R_STS4 / 4] = nmb * st.qp;
    s->pending = IRQ_SLICE_DONE | (s->bs_full ? IRQ_BS_FULL : 0);
    /* PARM5 bit 31: the picture's last slice */
    if (last_slice) {
        gm_h264e_dump(s);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                        GM_H264E_SLICE_NS);
}

static void gm_h264e_done(void *opaque)
{
    GmH264eState *s = opaque;

    s->irq_status |= s->pending;
    s->pending = 0;
    gm_h264e_update(s);
}

/* ---- registers ----------------------------------------------------------- */

static uint64_t gm_h264e_read(void *opaque, hwaddr off, unsigned size)
{
    GmH264eState *s = opaque;

    switch (off) {
    case R_STS0:
        return STS0_PACKED | STS0_DMA_IDLE;
    case R_STS1:
        return (s->bs_count & 0xFFFFFF) | (2u << 24);
    case R_STS2:
        return s->irq_status | (s->irq_mask << 16);
    }
    return s->regs[off / 4];
}

static void gm_h264e_reset_engine(GmH264eState *s)
{
    timer_del(s->timer);
    s->pending = 0;
    s->irq_status = 0;
    gm_h264e_update(s);
}

static void gm_h264e_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    GmH264eState *s = opaque;

    switch (off) {
    case R_CMD0:
        s->regs[off / 4] = val & ~(CMD0_GO | CMD0_RESET);
        if (val & CMD0_RESET) {
            gm_h264e_reset_engine(s);
        }
        if (val & CMD0_GO) {
            gm_h264e_encode_slice(s);
        }
        return;
    case R_CMD2:
        s->regs[off / 4] = 0;
        if (val & CMD2_CLEAR) {
            gm_h264e_clear(s);
        }
        if (val & CMD2_PACK) {
            gm_h264e_pack(s);
        }
        if (val & CMD2_CLOSE) {
            gm_h264e_trailing(s, true);
        }
        return;
    case R_STS0:
    case R_STS1:
        return;
    case R_STS2:
        s->irq_status &= ~(val & 0x1F);
        s->irq_mask = extract32(val, 16, 5);
        gm_h264e_update(s);
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps gm_h264e_ops = {
    .read = gm_h264e_read,
    .write = gm_h264e_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* VCACHE: storage, the DMA it shuts down reads as stopped (DECODE bit 1) */
static uint64_t gm_vcache_read(void *opaque, hwaddr off, unsigned size)
{
    GmH264eState *s = opaque;

    return s->vcache[off / 4] | (off == VC_DECODE ? 2 : 0);
}

static void gm_vcache_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmH264eState *s = opaque;

    s->vcache[off / 4] = val;
}

static const MemoryRegionOps gm_vcache_ops = {
    .read = gm_vcache_read,
    .write = gm_vcache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_h264e_reset(DeviceState *dev)
{
    GmH264eState *s = GM_H264E(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->vcache, 0, sizeof(s->vcache));
    s->irq_mask = 0x1F;
    gm_h264e_clear(s);
    gm_h264e_reset_engine(s);
    gm_h264e_drop_pic(s);
    memset(&s->hdr, 0, sizeof(s->hdr));
}

static void gm_h264e_realize(DeviceState *dev, Error **errp)
{
    GmH264eState *s = GM_H264E(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_h264e_ops, s,
                          TYPE_GM_H264E, GM_H264E_SIZE);
    memory_region_init_io(&s->vcache_mem, OBJECT(s), &gm_vcache_ops, s,
                          "gm-h264e.vcache", GM_VCACHE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->vcache_mem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_h264e_done, s);
}

static const Property gm_h264e_props[] = {
    DEFINE_PROP_STRING("dump", GmH264eState, dump),
};

static const VMStateDescription vmstate_gm_h264e = {
    .name = TYPE_GM_H264E,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmH264eState),
        VMSTATE_UINT32_ARRAY(regs, GmH264eState, GM_H264E_NREGS),
        VMSTATE_UINT32_ARRAY(vcache, GmH264eState, GM_VCACHE_NREGS),
        VMSTATE_UINT32(irq_status, GmH264eState),
        VMSTATE_UINT32(irq_mask, GmH264eState),
        VMSTATE_UINT32(pending, GmH264eState),
        VMSTATE_UINT32(bs_count, GmH264eState),
        VMSTATE_UINT32(bs_acc, GmH264eState),
        VMSTATE_UINT32(bs_nbits, GmH264eState),
        VMSTATE_UINT32(bs_zeros, GmH264eState),
        VMSTATE_BOOL(bs_full, GmH264eState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_h264e_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_h264e_realize;
    device_class_set_legacy_reset(dc, gm_h264e_reset);
    device_class_set_props(dc, gm_h264e_props);
    dc->vmsd = &vmstate_gm_h264e;
}

static const TypeInfo gm_h264e_info = {
    .name           = TYPE_GM_H264E,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmH264eState),
    .class_init     = gm_h264e_class_init,
};

static void gm_h264e_register_types(void)
{
    type_register_static(&gm_h264e_info);
}

type_init(gm_h264e_register_types)
