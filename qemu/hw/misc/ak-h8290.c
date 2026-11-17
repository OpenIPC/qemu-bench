/*
 * Anyka AK3918EV200 video encoder: a Hantro/On2 8290 (0x20020000, IRQ 2).
 *
 * The contract is OpenIPC's in-kernel V4L2 mem2mem driver ak-venc
 * (drivers/media/platform/anyka/ak-venc*.c on the anyka-ak3918ev300
 * branch), whose register map, ak-venc-regs.h, is the vendor library's own
 * field table.  The 8290 is the hx280 register layout: the same swreg
 * numbers as the H1 that xm-h1.c models for the H.264 / JPEG fields this
 * uses, but this is written from that table, not from xm-h1.
 *
 *   swreg0   0x000  ASIC id; the probe wants exactly 0x82900760
 *   swreg1   0x004  [0] IRQ line, [1] IRQ disable, [2] frame ready,
 *                   [3] bus error / timeout, [4] SW reset, [5] buffer full;
 *                   the driver acks by writing back with bit 0 cleared
 *   swreg5   0x014  output stream base (8-byte aligned)
 *   swreg6   0x018  control base: the NAL size table
 *   swreg7-10       reference / reconstructed luma and chroma
 *   swreg11-13      input Y / Cb / Cr
 *   swreg14  0x038  [29] NAL sizes on, [27:19] width MB, [18:10] height MB,
 *                   [6] no reconstruction, [4:3] type (1 intra, 0 inter),
 *                   [2:1] mode (2 JPEG, 3 H.264), [0] enable: starts it
 *   swreg15  0x03c  input offsets, row length (pixels), right overfill / 4,
 *                   bottom overfill, input format (0 I420, 1 NV12)
 *   swreg16  0x040  PPS QP, alpha / beta / chroma QP offsets, deblocking
 *                   mode, idr_pic_id, constrained intra
 *   swreg17  0x044  pic_parameter_set_id
 *   swreg18  0x048  CABAC, transform 8x8, frame_num
 *   swreg20  0x050  JPEG 4:2:2, slice enable, restart interval
 *   swreg22/23      header remainder bits, MSB-aligned
 *   swreg24  0x060  buffer limit in 64-bit words in; out: bits written
 *   swreg25  0x064  MAD QP delta / threshold; out: QP sum / 2
 *   swreg27  0x06c  initial / max / min QP
 *   swreg37  0x094  [28:23] remainder bit count; out: RLC words / 4
 *   swreg38  0x098  out: MBs under the MAD threshold, MB count
 *   swreg63  0x0fc  synthesis configuration (max width 1280)
 *   swreg64-95      JPEG quantisers, luma then chroma, four per word MSB
 *                   first, four rows of a column at a time
 *   0x200-0x3fc     mirror swreg0..127 (the driver never writes there)
 *
 * Output: the core continues the stream from the last whole 64-bit word
 * software wrote (the SPS / PPS, or the JFIF header up to SOS), so the
 * remainder bytes go out first.  H.264: then the slice NAL with its own
 * four-byte start code, a slice header written from the registers -- the
 * 8290 codes frame_num in 16 bits, writes no picture order syntax, and
 * this is what the driver's SPS (log2_max_frame_num 16, POC type 2) is
 * built around -- and slice data from hantro-h264.c: I slices of
 * I_16x16, P slices of skip / inter / intra macroblocks predicted from
 * the reference store the driver points at.  The reconstruction goes to
 * the reconstructed store (NV12 at the macroblock-padded size, which is
 * what the driver allocates for each), so a wrong reference rotation in
 * the driver drifts here as on silicon.  JPEG: the scan of jpeg-enc.c,
 * Annex K Huffman tables, and EOI.  swreg24 reads back the bits written
 * from the stream base, remainder included.
 *
 * The job completes from a timer, AK_H8290_JOB_NS after the enable write:
 * the driver starts the next job from its interrupt handler, and an
 * interrupt raised inside that write would land in the middle of
 * device_run.  Writing swreg14 with the enable bit clear stops a job
 * that has not completed (the driver's abort and error paths).
 *
 * Approximated: rate control inside the core (MAD QP adjustment, the
 * checkpoints, ROI) does not move the QP -- every macroblock is coded at
 * the initial QP, or higher when the frame would not fit -- and the RLC
 * count is estimated from the size.  The NAL size table gets one entry,
 * the slice NAL's size, then a zero.  The clock gate and soft reset in
 * SYSCTRL are not looked at: the id reads back whether gated or not.
 *
 * dump-dir (-global ak-h8290.dump-dir=DIR) keeps every job: frame-NNNNN.h264
 * (an IDR with the SPS / PPS the driver wrote in front of it, so the files
 * concatenate into a playable stream), frame-NNNNN.jpg (the whole file), and
 * frame-NNNNN.regs.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hantro-h264.h"
#include "jpeg-enc.h"

#define TYPE_AK_H8290 "ak-h8290"
OBJECT_DECLARE_SIMPLE_TYPE(AkH8290State, AK_H8290)

#define AK_H8290_MMIO_SIZE  0x10000
#define AK_H8290_NREGS      128         /* 0x200 mirrors them */

#define AK_H8290_ID         0x82900760u
/* swreg63: tiled input no, RGB in, H.264, JPEG, stabilisation; AXI, 64-bit
 * bus; the maximum width in pixels in [11:0] (ak-venc.h: 1280) */
#define AK_H8290_CFG        ((1u << 28) | (1u << 27) | (1u << 25) | \
                             (1u << 24) | (1u << 20) | (1u << 12))

#define AK_H8290_JOB_NS     (1 * 1000 * 1000)

/* swreg indices */
#define R_ID            0
#define R_IRQ           1
#define   IRQ_LINE          (1u << 0)
#define   IRQ_DIS           (1u << 1)
#define   IRQ_FRAME_RDY     (1u << 2)
#define   IRQ_BUS_ERR       (1u << 3)
#define   IRQ_SW_RESET      (1u << 4)
#define   IRQ_BUF_FULL      (1u << 5)
#define   IRQ_STATUS        0x1fcu
#define R_OUT           5
#define R_CTRL_BASE     6
#define R_REF_Y         7
#define R_REF_C         8
#define R_REC_Y         9
#define R_REC_C         10
#define R_IN_Y          11
#define R_IN_CB         12
#define R_IN_CR         13
#define R_ENC           14
#define   ENC_EN            (1u << 0)
#define   ENC_MODE(v)       (((v) >> 1) & 3)
#define   MODE_JPEG         2
#define   MODE_H264         3
#define   ENC_TYPE(v)       (((v) >> 3) & 3)   /* 1 intra, 0 inter */
#define   ENC_NO_REC        (1u << 6)
#define   ENC_HEIGHT(v)     (((v) >> 10) & 0x1ff)
#define   ENC_WIDTH(v)      (((v) >> 19) & 0x1ff)
#define   ENC_NAL_SIZES     (1u << 29)
#define R_IN_IMG        15
#define   IN_ROT(v)         ((v) & 3)
#define   IN_FMT(v)         (((v) >> 2) & 0xf)
#define   IN_YFILL(v)       (((v) >> 6) & 0xf)
#define   IN_XFILL(v)       (((v) >> 10) & 3)
#define   IN_ROW(v)         (((v) >> 12) & 0x3fff)
#define   IN_YOFF(v)        (((v) >> 26) & 7)
#define   IN_COFF(v)        (((v) >> 29) & 7)
#define R_H264_0        16
#define R_H264_1        17
#define R_H264_2        18
#define R_JPEG          20
#define   JPEG_422          (1u << 25)
#define   JPEG_SLICES       (1u << 24)
#define   JPEG_RST(v)       (((v) >> 16) & 0xff)
#define R_REM_MSB       22
#define R_REM_LSB       23
#define R_LIMIT         24
#define R_MAD           25
#define R_QP            27
#define R_RLC           37
#define   RLC_REM_BITS(v)   (((v) >> 23) & 0x3f)
#define R_MBS           38
#define R_CFG           63
#define R_QUANT         64              /* 64..79 luma, 80..95 chroma */

/* a sign-extended field */
static int sfield(uint32_t v, int shift, int bits)
{
    return sextract32(v, shift, bits);
}

struct AkH8290State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *job;
    uint32_t regs[AK_H8290_NREGS];
    uint32_t max_width;
    char *dump_dir;
    uint32_t jobs;
};

/* ---- dumps ------------------------------------------------------------ */

static void ak_h8290_dump(AkH8290State *s, const char *ext,
                          const uint8_t *data, size_t len)
{
    g_autofree char *path = g_strdup_printf("%s/frame-%05u.%s", s->dump_dir,
                                            s->jobs, ext);
    g_autoptr(GError) err = NULL;

    if (!g_file_set_contents(path, (const char *)data, len, &err)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-h8290: cannot dump %s: %s\n",
                      path, err->message);
    }
}

static void ak_h8290_dump_regs(AkH8290State *s)
{
    g_autoptr(GString) out = g_string_new(NULL);
    int i;

    for (i = 0; i < AK_H8290_NREGS; i++) {
        g_string_append_printf(out, "%03x %08x\n", i * 4, s->regs[i]);
    }
    ak_h8290_dump(s, "regs", (const uint8_t *)out->str, out->len);
}

/* ---- input ------------------------------------------------------------ */

static void ak_h8290_rd(hwaddr a, void *buf, size_t n)
{
    dma_memory_read(&address_space_memory, a, buf, n, MEMTXATTRS_UNSPECIFIED);
}

static void ak_h8290_wr(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * The picture as planar 4:2:0 of its real size (the coded size less the
 * overfill), stride w and w / 2.  Only what the input holds is read: the
 * driver puts Cb straight after the real height's luma rows.
 */
typedef struct AkPicture {
    int mbw, mbh, w, h;
    uint8_t *y, *cb, *cr;
} AkPicture;

static bool ak_h8290_input(AkH8290State *s, const char *what, AkPicture *p)
{
    const uint32_t enc = s->regs[R_ENC], img = s->regs[R_IN_IMG];
    const int fmt = IN_FMT(img);
    int row, r, i;

    p->mbw = ENC_WIDTH(enc);
    p->mbh = ENC_HEIGHT(enc);
    p->w = p->mbw * 16 - IN_XFILL(img) * 4;
    p->h = p->mbh * 16 - IN_YFILL(img);
    row = IN_ROW(img) ? IN_ROW(img) : p->w;
    p->w &= ~1;
    p->h &= ~1;
    if (!p->mbw || !p->mbh || p->mbw * 16 > (int)s->max_width ||
        p->mbh > 511 || p->w <= 0 || p->h <= 0 || row < p->w) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-h8290: %s %dx%d MB (%dx%d), "
                      "row %d\n", what, p->mbw, p->mbh, p->w, p->h, row);
        return false;
    }
    if (fmt > 1 || IN_ROT(img)) {
        qemu_log_mask(LOG_UNIMP, "ak-h8290: input format %d rotation %d\n",
                      fmt, IN_ROT(img));
        return false;
    }

    const hwaddr ya = s->regs[R_IN_Y] + IN_YOFF(img);
    const hwaddr ca = s->regs[R_IN_CB] + IN_COFF(img);
    const int cw = p->w / 2, ch = p->h / 2;

    p->y = g_malloc((size_t)p->w * p->h);
    p->cb = g_malloc((size_t)cw * ch);
    p->cr = g_malloc((size_t)cw * ch);
    for (r = 0; r < p->h; r++) {
        ak_h8290_rd(ya + (hwaddr)r * row, p->y + (size_t)r * p->w, p->w);
    }
    if (fmt == 0) {                     /* I420 */
        const hwaddr cra = s->regs[R_IN_CR] + IN_COFF(img);

        for (r = 0; r < ch; r++) {
            ak_h8290_rd(ca + (hwaddr)r * (row / 2), p->cb + (size_t)r * cw,
                        cw);
            ak_h8290_rd(cra + (hwaddr)r * (row / 2), p->cr + (size_t)r * cw,
                        cw);
        }
    } else {                            /* NV12 */
        g_autofree uint8_t *line = g_malloc(p->w);

        for (r = 0; r < ch; r++) {
            ak_h8290_rd(ca + (hwaddr)r * row, line, p->w);
            for (i = 0; i < cw; i++) {
                p->cb[r * cw + i] = line[2 * i];
                p->cr[r * cw + i] = line[2 * i + 1];
            }
        }
    }
    return true;
}

static void ak_picture_free(AkPicture *p)
{
    g_free(p->y);
    g_free(p->cb);
    g_free(p->cr);
}

/* The header's last partial word, handed back through swreg22/23/37. */
static int ak_h8290_remainder(AkH8290State *s, uint8_t *rem)
{
    const int bits = RLC_REM_BITS(s->regs[R_RLC]);
    const uint64_t w = ((uint64_t)s->regs[R_REM_MSB] << 32) |
                       s->regs[R_REM_LSB];
    int i;

    if (bits % 8 || bits > 56) {
        qemu_log_mask(LOG_UNIMP, "ak-h8290: %d header remainder bits, not "
                      "whole bytes\n", bits);
    }
    for (i = 0; i < MIN(bits, 56) / 8; i++) {
        rem[i] = w >> (56 - 8 * i);
    }
    return MIN(bits, 56) / 8;
}

/* Host memory for the output: the programmed limit, but never more than a
 * picture of this size can need. */
static size_t ak_h8290_cap(AkH8290State *s, int mbw, int mbh)
{
    return MIN((uint64_t)s->regs[R_LIMIT] * 8,
               (uint64_t)mbw * mbh * 1024 + 4096);
}

/* ---- JPEG ------------------------------------------------------------- */

static uint32_t ak_h8290_jpeg(AkH8290State *s)
{
    const uint32_t jctl = s->regs[R_JPEG];
    const hwaddr base = s->regs[R_OUT];
    uint8_t ql[64], qc[64], rem[8];
    AkPicture p;
    bool overflow;
    int r, c;

    if (jctl & (JPEG_422 | JPEG_SLICES) || JPEG_RST(jctl)) {
        qemu_log_mask(LOG_UNIMP, "ak-h8290: JPEG control 0x%x (4:2:2, "
                      "slices or restart markers)\n", jctl);
        return IRQ_BUS_ERR;
    }
    if (!ak_h8290_input(s, "JPEG", &p)) {
        return IRQ_BUS_ERR;
    }
    /* akcam_h8290_quant_apply(): slot i holds row r, column c */
    for (r = 0; r < 8; r++) {
        for (c = 0; c < 8; c++) {
            const int i = 4 * (4 * (c >> 1) + 2 * (r >> 2) + (c & 1)) +
                          (r & 3);
            const int sh = 24 - 8 * (i % 4);

            ql[r * 8 + c] = MAX(1, (s->regs[R_QUANT + i / 4] >> sh) & 0xff);
            qc[r * 8 + c] = MAX(1, (s->regs[R_QUANT + 16 + i / 4] >> sh) &
                                   0xff);
        }
    }

    const int nrem = ak_h8290_remainder(s, rem);
    const size_t cap = ak_h8290_cap(s, p.mbw, p.mbh);
    g_autofree uint8_t *out = g_malloc(MAX(cap, 8));
    size_t n = 0;

    overflow = (size_t)nrem + 2 > cap;
    if (!overflow) {
        memcpy(out, rem, nrem);
        n = nrem + jpeg_enc_scan(p.y, p.w, p.cb, p.cr, p.w / 2, p.w, p.h,
                                 ql, qc, true, out + nrem, cap - nrem,
                                 &overflow);
    }
    ak_picture_free(&p);
    if (overflow) {
        qemu_log_mask(LOG_GUEST_ERROR, "ak-h8290: JPEG over the %zu-byte "
                      "buffer\n", cap);
        return IRQ_BUF_FULL;
    }
    ak_h8290_wr(base, out, n);
    s->regs[R_LIMIT] = n * 8;
    if (s->dump_dir) {
        /* the driver's header is in the buffer in front of the base */
        g_autoptr(GByteArray) f = g_byte_array_new();
        const int w = ENC_WIDTH(s->regs[R_ENC]) * 16 -
                      IN_XFILL(s->regs[R_IN_IMG]) * 4;
        const int h = ENC_HEIGHT(s->regs[R_ENC]) * 16 -
                      IN_YFILL(s->regs[R_IN_IMG]);

        jpeg_enc_header(f, w, h, ql, qc);
        g_byte_array_append(f, out + nrem, n - nrem);
        ak_h8290_dump(s, "jpg", f->data, f->len);
    }
    return IRQ_FRAME_RDY;
}

/* ---- H.264 ------------------------------------------------------------ */

typedef struct AkBits {
    uint8_t *p;
    size_t cap, bits;
} AkBits;

static void bits_put(AkBits *b, uint32_t v, int n)
{
    while (n--) {
        if (b->bits / 8 < b->cap) {
            if ((v >> n) & 1) {
                b->p[b->bits / 8] |= 0x80 >> (b->bits % 8);
            } else {
                b->p[b->bits / 8] &= ~(0x80 >> (b->bits % 8));
            }
        }
        b->bits++;
    }
}

static void bits_ue(AkBits *b, uint32_t v)
{
    const int len = 32 - clz32(v + 1);

    bits_put(b, 0, len - 1);
    bits_put(b, v + 1, len);
}

static void bits_se(AkBits *b, int32_t v)
{
    bits_ue(b, v <= 0 ? -2 * v : 2 * v - 1);
}

/* start code, NAL header, then the RBSP with emulation prevention */
static size_t nal_wrap(uint8_t head, const uint8_t *rbsp, size_t n,
                       uint8_t *out)
{
    size_t o = 0, i;
    int zeros = 0;

    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 1;
    out[o++] = head;
    for (i = 0; i < n; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            out[o++] = 3;
            zeros = 0;
        }
        out[o++] = rbsp[i];
        zeros = rbsp[i] ? 0 : zeros + 1;
    }
    return o;
}

/* MBs whose mean absolute difference from the reference (or, with none,
 * from their own mean) is under the threshold */
static uint32_t ak_h8290_mad_under(const uint8_t *y, const uint8_t *ry,
                                   int mbw, int mbh, uint32_t thr)
{
    const int st = mbw * 16;
    uint32_t under = 0;
    int mx, my, i, j;

    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            const uint8_t *a = y + (size_t)my * 16 * st + mx * 16;
            const uint8_t *b = ry ? ry + (size_t)my * 16 * st + mx * 16 : NULL;
            uint32_t sum = 0, sad = 0;

            if (!b) {
                for (j = 0; j < 16; j++) {
                    for (i = 0; i < 16; i++) {
                        sum += a[j * st + i];
                    }
                }
            }
            for (j = 0; j < 16; j++) {
                for (i = 0; i < 16; i++) {
                    const int ref = b ? b[j * st + i] : (int)(sum / 256);

                    sad += abs(a[j * st + i] - ref);
                }
            }
            under += sad / 256 < thr;
        }
    }
    return under;
}

static uint32_t ak_h8290_h264(AkH8290State *s)
{
    const uint32_t enc = s->regs[R_ENC];
    const uint32_t h0 = s->regs[R_H264_0], h2 = s->regs[R_H264_2];
    const bool intra = ENC_TYPE(enc) == 1;
    const bool cabac = h2 & (1u << 18);
    const bool t8x8 = h2 & (1u << 21);
    const int pps_qp = extract32(h0, 26, 6);
    const int qp = MIN(extract32(s->regs[R_QP], 26, 6), 51);
    const int cqp = sfield(h0, 13, 5);
    const H264Deblock db = {
        .idc = extract32(h0, 5, 2),
        .alpha_off = 2 * sfield(h0, 22, 4),
        .beta_off = 2 * sfield(h0, 18, 4),
    };
    const hwaddr base = s->regs[R_OUT];
    const bool have_ref = !intra && s->regs[R_REF_Y] && s->regs[R_REF_C];
    uint8_t rem[8];
    H264PStats st;
    AkPicture p;
    int r, x;

    if (ENC_TYPE(enc) > 1) {
        qemu_log_mask(LOG_UNIMP, "ak-h8290: picture type %d\n",
                      ENC_TYPE(enc));
        return IRQ_BUS_ERR;
    }
    if (!ak_h8290_input(s, "H.264", &p)) {
        return IRQ_BUS_ERR;
    }

    /* the picture padded out to whole macroblocks, NV12 at stride pw */
    const int mbw = p.mbw, mbh = p.mbh, pw = mbw * 16, ph = mbh * 16;
    g_autofree uint8_t *y = g_malloc((size_t)pw * ph);
    g_autofree uint8_t *uv = g_malloc((size_t)pw * ph / 2);

    for (r = 0; r < ph; r++) {
        const uint8_t *src = p.y + (size_t)MIN(r, p.h - 1) * p.w;

        memcpy(y + (size_t)r * pw, src, p.w);
        memset(y + (size_t)r * pw + p.w, src[p.w - 1], pw - p.w);
    }
    for (r = 0; r < ph / 2; r++) {
        const size_t sr = (size_t)MIN(r, p.h / 2 - 1) * (p.w / 2);

        for (x = 0; x < pw / 2; x++) {
            const int sx = MIN(x, p.w / 2 - 1);

            uv[(size_t)r * pw + 2 * x] = p.cb[sr + sx];
            uv[(size_t)r * pw + 2 * x + 1] = p.cr[sr + sx];
        }
    }
    ak_picture_free(&p);

    g_autofree uint8_t *ry = g_malloc0((size_t)pw * ph);
    g_autofree uint8_t *ruv = g_malloc0((size_t)pw * ph / 2);
    if (have_ref) {
        ak_h8290_rd(s->regs[R_REF_Y], ry, (size_t)pw * ph);
        ak_h8290_rd(s->regs[R_REF_C], ruv, (size_t)pw * ph / 2);
    }

    /* slice_header(), as the 8290 writes it */
    const size_t rcap = (size_t)mbw * mbh * 800 + 1024;
    g_autofree uint8_t *rbsp = g_malloc0(rcap);
    AkBits b = { .p = rbsp, .cap = rcap };

    bits_ue(&b, 0);                             /* first_mb_in_slice */
    bits_ue(&b, intra ? 7 : 5);                 /* I / P, all slices */
    bits_ue(&b, extract32(s->regs[R_H264_1], 24, 8));   /* pps id */
    bits_put(&b, extract32(h2, 0, 16), 16);     /* frame_num, 16 bits */
    if (intra) {
        bits_ue(&b, extract32(h0, 1, 4));       /* idr_pic_id */
    } else {
        bits_put(&b, 0, 1);         /* num_ref_idx_active_override_flag */
        bits_put(&b, 0, 1);         /* ref_pic_list_modification_flag_l0 */
    }
    if (intra) {
        bits_put(&b, 0, 1);         /* no_output_of_prior_pics_flag */
        bits_put(&b, 0, 1);         /* long_term_reference_flag */
    } else {
        bits_put(&b, 0, 1);         /* adaptive_ref_pic_marking_mode_flag */
    }
    if (cabac && !intra) {
        bits_ue(&b, extract32(h2, 19, 2));      /* cabac_init_idc */
    }
    bits_se(&b, qp - pps_qp);                   /* slice_qp_delta */
    bits_ue(&b, db.idc);            /* deblocking_filter_control_present */
    if (db.idc != 1) {
        bits_se(&b, db.alpha_off / 2);
        bits_se(&b, db.beta_off / 2);
    }

    H264Pic *pic = h264_pic_new(mbw, mbh, y, pw, uv, pw,
                                have_ref ? ry : NULL, have_ref ? ruv : NULL,
                                cqp, t8x8);
    const size_t hb = b.bits / 8;
    size_t n;

    if (cabac) {
        while (b.bits % 8) {
            bits_put(&b, 1, 1);                 /* cabac_alignment_one_bit */
        }
        n = b.bits / 8 + h264_pic_slice_cabac(pic, !intra, 0, mbh, qp,
                                              extract32(h2, 19, 2),
                                              rbsp + b.bits / 8,
                                              rcap - b.bits / 8, &st);
    } else {
        /* the slice coder starts mid-byte and zeroes the bits before */
        const uint8_t keep = 0xff00 >> (b.bits % 8);
        const uint8_t hdr = rbsp[hb] & keep;

        n = h264_pic_slice_cavlc(pic, !intra, 0, mbh, qp, b.bits % 8,
                                 rbsp + hb, rcap - hb, &st);
        rbsp[hb] = hdr | (rbsp[hb] & ~keep);
        n = (hb * 8 + n + 7) / 8;
    }
    const uint32_t mad_under =
        ak_h8290_mad_under(y, have_ref ? ry : NULL, mbw, mbh,
                           extract32(s->regs[R_MAD], 22, 6));
    h264_pic_finish(pic, &db, ry, ruv);
    h264_pic_free(pic);

    const int nrem = ak_h8290_remainder(s, rem);
    const size_t cap = ak_h8290_cap(s, mbw, mbh);
    g_autofree uint8_t *out = g_malloc(nrem + 5 + n * 3 / 2 + 16);

    memcpy(out, rem, nrem);
    const size_t nal = nal_wrap(intra ? 0x65 : 0x41, rbsp, n, out + nrem);
    const size_t total = nrem + nal;

    if (total > cap) {
        /* nothing is written; the driver forces the next frame intra */
        qemu_log_mask(LOG_GUEST_ERROR, "ak-h8290: H.264 picture of %zu bytes "
                      "over the %zu-byte buffer\n", total, cap);
        return IRQ_BUF_FULL;
    }
    ak_h8290_wr(base, out, total);
    if (!(enc & ENC_NO_REC) && s->regs[R_REC_Y] && s->regs[R_REC_C]) {
        ak_h8290_wr(s->regs[R_REC_Y], ry, (size_t)pw * ph);
        ak_h8290_wr(s->regs[R_REC_C], ruv, (size_t)pw * ph / 2);
    }
    if ((enc & ENC_NAL_SIZES) && s->regs[R_CTRL_BASE]) {
        const uint32_t sizes[2] = { cpu_to_le32(nal), 0 };

        ak_h8290_wr(s->regs[R_CTRL_BASE], sizes, sizeof(sizes));
    }

    /* the results the driver's rate control reads */
    const uint32_t mbs = mbw * mbh;
    s->regs[R_LIMIT] = total * 8;
    s->regs[R_MAD] = (s->regs[R_MAD] & ~0x1fffffu) |
                     ((mbs * MIN(st.qp ? st.qp : qp, 51) / 2) & 0x1fffff);
    /* about six bits a codeword, which is the driver's starting guess */
    s->regs[R_RLC] = (s->regs[R_RLC] & ~0x7fffffu) |
                     MIN(total * 8 / 24, 0x7fffffu);
    s->regs[R_MBS] = (MIN(mad_under, 0xffff) << 16) | (mbs & 0xffff);

    if (s->dump_dir) {
        g_autoptr(GByteArray) f = g_byte_array_new();

        if (intra) {
            /* the SPS / PPS the driver wrote in front of the base (their
             * last partial word is the remainder, in out) */
            uint8_t hdrs[128];
            const hwaddr from = base >= sizeof(hdrs) ? base - sizeof(hdrs) : 0;
            const size_t hn = base - from;
            int i;

            ak_h8290_rd(from, hdrs, hn);
            for (i = (int)hn - 5; i >= 0; i--) {
                if (!hdrs[i] && !hdrs[i + 1] && !hdrs[i + 2] &&
                    hdrs[i + 3] == 1 && (hdrs[i + 4] & 0x1f) == 7) {
                    g_byte_array_append(f, hdrs + i, hn - i);
                    break;
                }
            }
        }
        g_byte_array_append(f, out, total);
        ak_h8290_dump(s, "h264", f->data, f->len);
    }
    return IRQ_FRAME_RDY;
}

/* ---- the core --------------------------------------------------------- */

static void ak_h8290_update_irq(AkH8290State *s)
{
    const uint32_t v = s->regs[R_IRQ];

    qemu_set_irq(s->irq, (v & IRQ_LINE) && !(v & IRQ_DIS));
}

static void ak_h8290_job_done(void *opaque)
{
    AkH8290State *s = AK_H8290(opaque);
    const uint32_t enc = s->regs[R_ENC];
    uint32_t status = IRQ_BUS_ERR;

    if (!(enc & ENC_EN)) {
        return;
    }
    if (s->dump_dir) {
        ak_h8290_dump_regs(s);
    }
    if (ENC_MODE(enc) == MODE_JPEG) {
        status = ak_h8290_jpeg(s);
    } else if (ENC_MODE(enc) == MODE_H264) {
        status = ak_h8290_h264(s);
    } else {
        qemu_log_mask(LOG_UNIMP, "ak-h8290: encoding mode %u\n",
                      ENC_MODE(enc));
    }
    s->jobs++;
    s->regs[R_ENC] &= ~ENC_EN;
    s->regs[R_IRQ] = (s->regs[R_IRQ] & ~IRQ_STATUS) | status | IRQ_LINE;
    ak_h8290_update_irq(s);
}

static uint64_t ak_h8290_read(void *opaque, hwaddr off, unsigned size)
{
    AkH8290State *s = AK_H8290(opaque);
    unsigned r;

    if (off >= 0x400) {
        return 0;
    }
    r = (off & 0x1ff) / 4;
    if (r == R_ID) {
        return AK_H8290_ID;
    }
    if (r == R_CFG) {
        return AK_H8290_CFG | MIN(s->max_width, 0xfff);
    }
    return s->regs[r];
}

static void ak_h8290_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    AkH8290State *s = AK_H8290(opaque);
    const uint32_t was = s->regs[R_ENC];
    unsigned r;

    if (off >= 0x400) {
        return;
    }
    r = (off & 0x1ff) / 4;
    if (r == R_ID || r == R_CFG) {
        return;
    }
    s->regs[r] = val;
    if (r == R_IRQ) {
        ak_h8290_update_irq(s);
    } else if (r == R_ENC) {
        if ((val & ENC_EN) && !(was & ENC_EN)) {
            s->regs[R_IRQ] &= ~(IRQ_STATUS | IRQ_LINE);
            ak_h8290_update_irq(s);
            timer_mod(s->job, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              AK_H8290_JOB_NS);
        } else if (!(val & ENC_EN)) {
            timer_del(s->job);          /* stopped before it completed */
        }
    }
}

static const MemoryRegionOps ak_h8290_ops = {
    .read = ak_h8290_read,
    .write = ak_h8290_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ak_h8290_reset(DeviceState *dev)
{
    AkH8290State *s = AK_H8290(dev);

    timer_del(s->job);
    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void ak_h8290_init(Object *obj)
{
    AkH8290State *s = AK_H8290(obj);

    memory_region_init_io(&s->iomem, obj, &ak_h8290_ops, s, TYPE_AK_H8290,
                          AK_H8290_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->job = timer_new_ns(QEMU_CLOCK_VIRTUAL, ak_h8290_job_done, s);
}

static const VMStateDescription vmstate_ak_h8290 = {
    .name = TYPE_AK_H8290,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AkH8290State, AK_H8290_NREGS),
        VMSTATE_TIMER_PTR(job, AkH8290State),
        VMSTATE_UINT32(jobs, AkH8290State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ak_h8290_props[] = {
    DEFINE_PROP_STRING("dump-dir", AkH8290State, dump_dir),
    DEFINE_PROP_UINT32("max-width", AkH8290State, max_width, 1280),
};

static void ak_h8290_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->vmsd = &vmstate_ak_h8290;
    device_class_set_legacy_reset(DEVICE_CLASS(klass), ak_h8290_reset);
    device_class_set_props(DEVICE_CLASS(klass), ak_h8290_props);
}

static const TypeInfo ak_h8290_info = {
    .name          = TYPE_AK_H8290,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkH8290State),
    .instance_init = ak_h8290_init,
    .class_init    = ak_h8290_class_init,
};

static void ak_h8290_register_types(void)
{
    type_register_static(&ak_h8290_info);
}

type_init(ak_h8290_register_types)
