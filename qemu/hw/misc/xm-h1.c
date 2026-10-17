/*
 * Hantro H1 video encoder, as XM530 carries it (0x20070000, SPI 45).
 *
 * The vendor driver is xm530_h264.ko, Hantro's own hx280enc: at load it reads
 * swreg0 and wants a product ID of 0x6280/0x7280/0x8270/0x8290, and Hantro's
 * EWL in the SDK reads the synthesis configuration (swreg63) before any
 * encoder initialises.
 *
 * Encoding follows the register contract of the open-source H1 driver in
 * mainline Linux (drivers/media/platform/verisilicon: hantro_h1_regs.h,
 * hantro_h1_jpeg_enc.c, hantro_jpeg.c), not any vendor code:
 *
 *   ADDR_IN_PLANE_0/1/2   input planes;  IN_IMG_CTRL  row length, format
 *                         (0 YUV420 planar, 1 YUV420 semi-planar)
 *   ADDR_OUTPUT_STREAM    where the entropy-coded scan goes (the driver
 *                         writes the JFIF header in front of it itself)
 *   STR_BUF_LIMIT         bytes available; on completion, bits written
 *   JPEG_LUMA/CHROMA_QUAT the quantisers, packed big-endian four to a word
 *                         in the driver's hw_reorder order
 *   ENC_CTRL              MB width/height, mode, EN: writing EN starts it
 *   INTERRUPT             FRAME_RDY | IRQ on completion, the line raised
 *                         unless DIS
 *
 * JPEG: baseline, 4:2:0, the Annex K Huffman tables hantro_jpeg puts in the
 * header, ending with EOI.  The frame completes before the start write
 * returns.
 *
 * H.264 (mode 3): Hantro's library writes the SPS and PPS itself, up to an
 * 8-byte aligned ADDR_OUTPUT_STREAM, and hands the H1 the bits that did not
 * fit: STR_HDR_REM_MSB/LSB, RLC_CTRL[28:23] of them.  The model emits those
 * first, then a slice from hantro-h264.c fitted to the SPS/PPS it reads
 * back out of the stream buffer.  ENC_CTRL's intra bit makes it an IDR;
 * ENC_CTRL0[31:26] is the QP.
 *
 * dump-dir (-global xm-h1.dump-dir=DIR) keeps every job on the host, to see
 * what a guest's encoder actually produced: a JPEG job as a complete
 * frame-NNNNN.jpg (the model writes the JFIF header the driver would, from
 * the programmed quantisers), an H.264 job as frame-NNNNN.h264, the bytes
 * the H1 wrote; and each job's registers as frame-NNNNN.regs.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hw/qdev-properties.h"
#include "hantro-h264.h"
#include <math.h>

#define TYPE_XM_H1 "xm-h1"
OBJECT_DECLARE_SIMPLE_TYPE(XmH1State, XM_H1)

#define H1_REGS_SIZE    0x1000
#define H1_SWREG0_ID    0x000
/* 8290 is the H1 in the driver's list; low half is the build. */
#define H1_ID           0x82901000u
/* swreg63, the synthesis configuration Hantro's EWL reads before any
 * encoder will initialise (EWLReadAsicConfig): max width (bits 10:0,
 * pixels), and the codecs the silicon has -- VS 24, JPEG 25, VP8 26,
 * H.264 27, RGB 28 -- with an AXI bus (type 1, bits 23:20) 64 bits wide
 * (bits 13:12 = 1). */
#define H1_SWREG63_CFG  0x0FC
#define H1_CFG          ((1u << 28) | (1u << 27) | (1u << 26) | (1u << 25) | \
                         (1u << 24) | (1u << 20) | (1u << 12) | 1920)

/* hantro_h1_regs.h */
#define H1_REG_INTERRUPT            0x004
/* 0x08 and 0x20 as Hantro's own encoder names them (encasiccontroller.h):
 * ASIC_STATUS_ERROR and ASIC_STATUS_BUFF_FULL */
#define   H1_INT_BUF_FULL           (1u << 5)
#define   H1_INT_ERROR              (1u << 3)
#define   H1_INT_FRAME_RDY          (1u << 2)
#define   H1_INT_DIS                (1u << 1)
#define   H1_INT_BIT                (1u << 0)
#define H1_REG_ADDR_OUTPUT_STREAM   0x014
#define H1_REG_ADDR_IN_PLANE_0      0x02c
#define H1_REG_ADDR_IN_PLANE_1      0x030
#define H1_REG_ADDR_IN_PLANE_2      0x034
#define H1_REG_ENC_CTRL             0x038
#define   H1_ENC_EN                 (1u << 0)
#define   H1_ENC_MODE(v)            (((v) >> 1) & 3)
#define   H1_ENC_MODE_JPEG          2
#define   H1_ENC_WIDTH_MB(v)        (((v) >> 19) & 0x1ff)
#define   H1_ENC_HEIGHT_MB(v)       (((v) >> 10) & 0x1ff)
#define H1_REG_IN_IMG_CTRL          0x03c
#define   H1_IN_ROW_LEN(v)          (((v) >> 12) & 0x3fff)
#define   H1_IN_FMT(v)              (((v) >> 2) & 0xf)
#define H1_REG_ENC_CTRL0            0x040
#define   H1_CTRL0_INIT_QP(v)       (((v) >> 26) & 0x3f)
#define   H1_ENC_PIC_INTRA          (1u << 3)
#define   H1_ENC_MODE_H264          3
#define H1_REG_STR_HDR_REM_MSB      0x058
#define H1_REG_STR_HDR_REM_LSB      0x05c
#define H1_REG_STR_BUF_LIMIT        0x060
#define H1_REG_RLC_CTRL             0x094
#define   H1_RLC_STR_OFFS(v)        (((v) >> 23) & 0x3f)
#define H1_REG_JPEG_LUMA_QUAT       0x100
#define H1_REG_JPEG_CHROMA_QUAT     0x140

struct XmH1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[H1_REGS_SIZE / 4];
    char *dump_dir;
    uint32_t jobs;
    H264Enc h264;
};

/* ---- JPEG baseline encoder ------------------------------------------ */

/* hantro_jpeg.c: hw[i] = Q[hw_reorder[i]], Q in natural (row-major) order */
static const uint8_t hw_reorder[64] = {
     0,  8, 16, 24,  1,  9, 17, 25, 32, 40, 48, 56, 33, 41, 49, 57,
     2, 10, 18, 26,  3, 11, 19, 27, 34, 42, 50, 58, 35, 43, 51, 59,
     4, 12, 20, 28,  5, 13, 21, 29, 36, 44, 52, 60, 37, 45, 53, 61,
     6, 14, 22, 30,  7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63,
};

/* zigzag position -> natural index */
static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* ITU-T T.81 Annex K.3, the tables hantro_jpeg writes into its header */
static const uint8_t dc_luma_bits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_chroma_bits[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
static const uint8_t dc_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const uint8_t ac_luma_bits[16] = {
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
};
static const uint8_t ac_luma_vals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};
static const uint8_t ac_chroma_bits[16] = {
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
};
static const uint8_t ac_chroma_vals[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

typedef struct HuffTable {
    uint16_t code[256];
    uint8_t len[256];
} HuffTable;

static HuffTable huff_dc[2], huff_ac[2];     /* [0] luma, [1] chroma */

static void huff_build(HuffTable *t, const uint8_t *bits, const uint8_t *vals)
{
    unsigned code = 0, k = 0, l, i;

    for (l = 1; l <= 16; l++) {
        for (i = 0; i < bits[l - 1]; i++, k++) {
            t->code[vals[k]] = code++;
            t->len[vals[k]] = l;
        }
        code <<= 1;
    }
}

typedef struct BitWriter {
    uint8_t *buf;
    size_t cap, n;
    uint32_t acc;
    int nbits;
    bool overflow;
} BitWriter;

static void bw_byte(BitWriter *w, uint8_t b)
{
    if (w->n >= w->cap) {
        w->overflow = true;
        return;
    }
    w->buf[w->n++] = b;
}

static void bw_put(BitWriter *w, uint32_t bits, int len)
{
    w->acc = (w->acc << len) | (bits & ((1u << len) - 1));
    w->nbits += len;
    while (w->nbits >= 8) {
        const uint8_t b = w->acc >> (w->nbits - 8);
        bw_byte(w, b);
        if (b == 0xff) {
            bw_byte(w, 0x00);           /* byte stuffing */
        }
        w->nbits -= 8;
    }
}

static void bw_flush(BitWriter *w)
{
    if (w->nbits) {
        bw_put(w, 0x7f, 8 - w->nbits);  /* pad with 1s */
    }
}

static int bit_size(int v)
{
    int n = 0;

    v = v < 0 ? -v : v;
    while (v) {
        n++;
        v >>= 1;
    }
    return n;
}

static void put_value(BitWriter *w, int v, int size)
{
    if (size) {
        bw_put(w, v < 0 ? v - 1 : v, size);
    }
}

/* Forward 8x8 DCT, then quantise; out[] in zigzag order. */
static void fdct_quant(const float in[64], const uint8_t q[64], int out[64])
{
    static float c[8][8];
    static bool init;
    float rows[64], dct[64];
    int u, v, x, y;

    if (!init) {
        for (u = 0; u < 8; u++) {
            for (x = 0; x < 8; x++) {
                c[u][x] = (u ? 0.5f : 0.5f / sqrtf(2.0f)) *
                          cosf((2 * x + 1) * u * (float)M_PI / 16);
            }
        }
        init = true;
    }
    for (y = 0; y < 8; y++) {           /* rows: rows[y][u] */
        for (u = 0; u < 8; u++) {
            float acc = 0;
            for (x = 0; x < 8; x++) {
                acc += c[u][x] * in[y * 8 + x];
            }
            rows[y * 8 + u] = acc;
        }
    }
    for (u = 0; u < 8; u++) {           /* columns: dct[v][u] */
        for (v = 0; v < 8; v++) {
            float acc = 0;
            for (y = 0; y < 8; y++) {
                acc += c[v][y] * rows[y * 8 + u];
            }
            dct[v * 8 + u] = acc;
        }
    }
    for (x = 0; x < 64; x++) {
        const int nat = zigzag[x];
        out[x] = (int)lrintf(dct[nat] / q[nat]);
    }
}

static void encode_block(BitWriter *w, const float px[64], const uint8_t q[64],
                         int comp, int *dc_pred)
{
    const HuffTable *dc = &huff_dc[comp ? 1 : 0];
    const HuffTable *ac = &huff_ac[comp ? 1 : 0];
    int coef[64], k, run = 0, size;

    fdct_quant(px, q, coef);
    const int diff = coef[0] - *dc_pred;
    *dc_pred = coef[0];
    size = bit_size(diff);
    bw_put(w, dc->code[size], dc->len[size]);
    put_value(w, diff, size);
    for (k = 1; k < 64; k++) {
        if (!coef[k]) {
            run++;
            continue;
        }
        while (run > 15) {
            bw_put(w, ac->code[0xf0], ac->len[0xf0]);   /* ZRL */
            run -= 16;
        }
        size = bit_size(coef[k]);
        const int sym = (run << 4) | size;
        bw_put(w, ac->code[sym], ac->len[sym]);
        put_value(w, coef[k], size);
        run = 0;
    }
    if (run) {
        bw_put(w, ac->code[0], ac->len[0]);              /* EOB */
    }
}

static uint8_t px_at(const uint8_t *p, int stride, int w, int h, int x, int y)
{
    x = MIN(x, w - 1);
    y = MIN(y, h - 1);
    return p[y * stride + x];
}

static void xm_h1_dump(XmH1State *s, const char *ext, const uint8_t *data,
                       size_t len)
{
    g_autofree char *path = g_strdup_printf("%s/frame-%05u.%s", s->dump_dir,
                                            s->jobs, ext);
    g_autoptr(GError) err = NULL;

    if (!g_file_set_contents(path, (const char *)data, len, &err)) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: cannot dump %s: %s\n", path,
                      err->message);
    }
}

static void xm_h1_dump_regs(XmH1State *s)
{
    g_autoptr(GString) out = g_string_new(NULL);
    int i;

    for (i = 0; i < 0x200 / 4; i++) {
        g_string_append_printf(out, "%03x %08x\n", i * 4, s->regs[i]);
    }
    xm_h1_dump(s, "regs", (const uint8_t *)out->str, out->len);
}

/* The JFIF header hantro_jpeg writes ahead of the scan, so a dumped JPEG job
 * is a file any decoder opens. */
static void jfif_append(GByteArray *b, int w, int h, const uint8_t ql[64],
                        const uint8_t qc[64])
{
    static const uint8_t soi[] = { 0xff, 0xd8 };
    uint8_t seg[4 + 1 + 64 + 1 + 64];
    int i, t, n;

    g_byte_array_append(b, soi, 2);
    seg[0] = 0xff; seg[1] = 0xdb; seg[2] = 0; seg[3] = 2 + 2 * 65;
    for (t = 0; t < 2; t++) {
        seg[4 + t * 65] = t;
        for (i = 0; i < 64; i++) {
            seg[5 + t * 65 + i] = (t ? qc : ql)[zigzag[i]];
        }
    }
    g_byte_array_append(b, seg, 4 + 130);
    const uint8_t sof[] = { 0xff, 0xc0, 0, 17, 8, h >> 8, h, w >> 8, w, 3,
                            1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1 };
    g_byte_array_append(b, sof, sizeof(sof));
    for (t = 0; t < 4; t++) {
        const uint8_t *bits = (const uint8_t *[]){ dc_luma_bits, ac_luma_bits,
                                                   dc_chroma_bits,
                                                   ac_chroma_bits }[t];
        const uint8_t *vals = (const uint8_t *[]){ dc_vals, ac_luma_vals,
                                                   dc_vals,
                                                   ac_chroma_vals }[t];
        const uint8_t cls = ((t & 1) << 4) | (t >> 1);
        for (n = 0, i = 0; i < 16; i++) {
            n += bits[i];
        }
        const uint8_t hdr[] = { 0xff, 0xc4, (3 + 16 + n) >> 8, 3 + 16 + n, cls };
        g_byte_array_append(b, hdr, sizeof(hdr));
        g_byte_array_append(b, bits, 16);
        g_byte_array_append(b, vals, n);
    }
    const uint8_t sos[] = { 0xff, 0xda, 0, 12, 3, 1, 0x00, 2, 0x11, 3, 0x11,
                            0, 63, 0 };
    g_byte_array_append(b, sos, sizeof(sos));
}

/* swreg63 advertises 1920 pixels; the stride must cover the picture so
 * every read stays inside the input copy, and stay a row's worth */
#define H1_MAX_WIDTH_MB   (1920 / 16)
#define H1_MAX_HEIGHT_MB  (4096 / 16)

static bool xm_h1_geometry_ok(const char *what, int mbw, int mbh, int stride)
{
    if (!mbw || !mbh || mbw > H1_MAX_WIDTH_MB || mbh > H1_MAX_HEIGHT_MB ||
        stride < mbw * 16 || stride > 4096) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: %s %dx%d MB, stride %d\n",
                      what, mbw, mbh, stride);
        return false;
    }
    return true;
}

/* Host memory for the output: the programmed limit, but never more than a
 * frame of this size can take (6 bytes a pixel covers JPEG at quality 1). */
static uint32_t xm_h1_out_cap(XmH1State *s, int mbw, int mbh)
{
    return MIN(s->regs[H1_REG_STR_BUF_LIMIT / 4],
               (uint32_t)mbw * mbh * 256 * 6 + 4096);
}

static uint32_t xm_h1_jpeg(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    const uint32_t img = s->regs[H1_REG_IN_IMG_CTRL / 4];
    const int mbw = H1_ENC_WIDTH_MB(ctrl), mbh = H1_ENC_HEIGHT_MB(ctrl);
    const int w = mbw * 16, h = mbh * 16;
    const int stride = H1_IN_ROW_LEN(img) ? H1_IN_ROW_LEN(img) : w;
    const int fmt = H1_IN_FMT(img);
    uint8_t ql[64], qc[64];
    uint8_t *y, *cb, *cr;
    BitWriter bw = { 0 };
    int dc[3] = { 0, 0, 0 }, i, mx, my;

    if (fmt != 0 && fmt != 1) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: JPEG input format %d\n", fmt);
        return H1_INT_ERROR;
    }
    if (!xm_h1_geometry_ok("JPEG", mbw, mbh, stride)) {
        return H1_INT_ERROR;
    }
    for (i = 0; i < 64; i++) {
        const uint32_t lw = s->regs[H1_REG_JPEG_LUMA_QUAT / 4 + i / 4];
        const uint32_t cw = s->regs[H1_REG_JPEG_CHROMA_QUAT / 4 + i / 4];
        ql[hw_reorder[i]] = MAX(1, (lw >> (24 - 8 * (i % 4))) & 0xff);
        qc[hw_reorder[i]] = MAX(1, (cw >> (24 - 8 * (i % 4))) & 0xff);
    }

    y = g_malloc(stride * h);
    cb = g_malloc((stride / 2) * (h / 2));
    cr = g_malloc((stride / 2) * (h / 2));
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_0 / 4],
                    y, stride * h, MEMTXATTRS_UNSPECIFIED);
    if (fmt == 1) {                     /* semi-planar: CbCr interleaved */
        uint8_t *uv = g_malloc(stride * (h / 2));
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_1 / 4], uv,
                        stride * (h / 2), MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < (stride / 2) * (h / 2); i++) {
            cb[i] = uv[2 * i];
            cr[i] = uv[2 * i + 1];
        }
        g_free(uv);
    } else {
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_1 / 4], cb,
                        (stride / 2) * (h / 2), MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        s->regs[H1_REG_ADDR_IN_PLANE_2 / 4], cr,
                        (stride / 2) * (h / 2), MEMTXATTRS_UNSPECIFIED);
    }

    bw.cap = xm_h1_out_cap(s, mbw, mbh);
    bw.buf = g_malloc(MAX(bw.cap, 1));
    for (my = 0; my < mbh; my++) {
        for (mx = 0; mx < mbw; mx++) {
            float blk[64];
            int b, bx, by;

            for (b = 0; b < 4; b++) {               /* Y: 2x2 blocks */
                const int ox = mx * 16 + (b & 1) * 8, oy = my * 16 + (b >> 1) * 8;
                for (by = 0; by < 8; by++) {
                    for (bx = 0; bx < 8; bx++) {
                        blk[by * 8 + bx] = px_at(y, stride, w, h, ox + bx,
                                                 oy + by) - 128.0f;
                    }
                }
                encode_block(&bw, blk, ql, 0, &dc[0]);
            }
            for (b = 0; b < 2; b++) {               /* Cb, Cr */
                const uint8_t *p = b ? cr : cb;
                for (by = 0; by < 8; by++) {
                    for (bx = 0; bx < 8; bx++) {
                        blk[by * 8 + bx] = px_at(p, stride / 2, w / 2, h / 2,
                                                 mx * 8 + bx, my * 8 + by) -
                                           128.0f;
                    }
                }
                encode_block(&bw, blk, qc, 1, &dc[1 + b]);
            }
        }
    }
    bw_flush(&bw);
    bw_byte(&bw, 0xff);                             /* EOI */
    bw_byte(&bw, 0xd9);
    if (bw.overflow) {
        /* nothing is written: the driver sees BUFF_FULL, not a short scan */
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: JPEG output over the %u-byte "
                      "buffer\n", (unsigned)bw.cap);
    } else {
        dma_memory_write(&address_space_memory,
                         s->regs[H1_REG_ADDR_OUTPUT_STREAM / 4], bw.buf, bw.n,
                         MEMTXATTRS_UNSPECIFIED);
        /* hantro_h1_jpeg_enc_done(): bytesused = STR_BUF_LIMIT / 8 */
        s->regs[H1_REG_STR_BUF_LIMIT / 4] = bw.n * 8;
    }
    if (s->dump_dir && !bw.overflow) {
        g_autoptr(GByteArray) f = g_byte_array_new();
        jfif_append(f, w, h, ql, qc);
        g_byte_array_append(f, bw.buf, bw.n);
        xm_h1_dump(s, "jpg", f->data, f->len);
    }
    g_free(bw.buf);
    g_free(y);
    g_free(cb);
    g_free(cr);
    return bw.overflow ? H1_INT_BUF_FULL : H1_INT_FRAME_RDY;
}

static uint32_t xm_h1_h264(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    const uint32_t img = s->regs[H1_REG_IN_IMG_CTRL / 4];
    const int mbw = H1_ENC_WIDTH_MB(ctrl), mbh = H1_ENC_HEIGHT_MB(ctrl);
    const int stride = H1_IN_ROW_LEN(img) ? H1_IN_ROW_LEN(img) : mbw * 16;
    const bool idr = ctrl & H1_ENC_PIC_INTRA;
    const hwaddr out = s->regs[H1_REG_ADDR_OUTPUT_STREAM / 4];
    const uint32_t cap = xm_h1_out_cap(s, mbw, mbh);
    const int rem_bits = H1_RLC_STR_OFFS(s->regs[H1_REG_RLC_CTRL / 4]);
    const uint64_t rem = ((uint64_t)s->regs[H1_REG_STR_HDR_REM_MSB / 4] << 32) |
                         s->regs[H1_REG_STR_HDR_REM_LSB / 4];
    uint8_t rem_bytes[8];
    uint8_t hdr[512 + 8];
    size_t hdr_at = 512;                /* the headers' start in hdr, if any */
    int i, nrem = rem_bits / 8;

    if (H1_IN_FMT(img) != 1) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: H.264 input format %d\n",
                      H1_IN_FMT(img));
        return H1_INT_ERROR;
    }
    if (!xm_h1_geometry_ok("H.264", mbw, mbh, stride)) {
        return H1_INT_ERROR;
    }
    if (rem_bits % 8) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: %d header bits, not whole bytes\n",
                      rem_bits);
    }
    for (i = 0; i < nrem; i++) {
        rem_bytes[i] = rem >> (56 - 8 * i);
    }
    if (idr) {
        /* the SPS/PPS the library wrote, completed by the remainder */
        dma_memory_read(&address_space_memory, out - 512, hdr, 512,
                        MEMTXATTRS_UNSPECIFIED);
        memcpy(hdr + 512, rem_bytes, nrem);
        h264_enc_parse_headers(&s->h264, hdr, 512 + nrem);
        /* the last SPS's start code: the dump of an IDR job carries the
         * headers it was coded against, so the files concatenate into a
         * stream any decoder plays even when rate control rewrites the PPS */
        for (i = 512 - 5; i >= 0; i--) {
            if (!hdr[i] && !hdr[i + 1] && hdr[i + 2] == 1 &&
                (hdr[i + 3] & 0x1f) == 7) {
                hdr_at = i > 0 && !hdr[i - 1] ? i - 1 : i;
                break;
            }
        }
    }

    g_autofree uint8_t *y = g_malloc((size_t)stride * mbh * 16);
    g_autofree uint8_t *uv = g_malloc((size_t)stride * mbh * 8);
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_0 / 4],
                    y, (size_t)stride * mbh * 16, MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, s->regs[H1_REG_ADDR_IN_PLANE_1 / 4],
                    uv, (size_t)stride * mbh * 8, MEMTXATTRS_UNSPECIFIED);

    g_autofree uint8_t *buf = g_malloc(MAX(cap, 16));
    memcpy(buf, rem_bytes, MIN(nrem, (int)cap));
    size_t n = h264_enc_frame(&s->h264, idr,
                              H1_CTRL0_INIT_QP(s->regs[H1_REG_ENC_CTRL0 / 4]),
                              mbw, mbh, y, stride, uv, stride,
                              buf + nrem, cap > (uint32_t)nrem ? cap - nrem : 0);
    if (!n) {
        qemu_log_mask(LOG_UNIMP, "xm-h1: no SPS/PPS this encoder handles "
                      "(CABAC, POC type 0/2, progressive) before the stream\n");
        return H1_INT_ERROR;
    }
    n += nrem;
    if (n > cap) {
        /* Hantro's library answers BUFF_FULL with a bigger buffer or a
         * skipped frame; nothing is written */
        qemu_log_mask(LOG_GUEST_ERROR, "xm-h1: H.264 output over the %u-byte "
                      "buffer\n", cap);
        return H1_INT_BUF_FULL;
    }
    dma_memory_write(&address_space_memory, out, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
    if (s->dump_dir) {
        g_autofree uint8_t *file = g_malloc(512 - hdr_at + n);
        memcpy(file, hdr + hdr_at, 512 - hdr_at);
        memcpy(file + 512 - hdr_at, buf, n);
        xm_h1_dump(s, "h264", file, 512 - hdr_at + n);
    }
    s->regs[H1_REG_STR_BUF_LIMIT / 4] = n * 8;
    return H1_INT_FRAME_RDY;
}

static void xm_h1_update_irq(XmH1State *s)
{
    const uint32_t v = s->regs[H1_REG_INTERRUPT / 4];

    qemu_set_irq(s->irq, (v & H1_INT_BIT) && !(v & H1_INT_DIS));
}

static void xm_h1_start(XmH1State *s)
{
    const uint32_t ctrl = s->regs[H1_REG_ENC_CTRL / 4];
    uint32_t status = H1_INT_ERROR;

    if (s->dump_dir) {
        xm_h1_dump_regs(s);
    }
    if (H1_ENC_MODE(ctrl) == H1_ENC_MODE_JPEG) {
        status = xm_h1_jpeg(s);
    } else if (H1_ENC_MODE(ctrl) == H1_ENC_MODE_H264) {
        status = xm_h1_h264(s);
    } else {
        qemu_log_mask(LOG_UNIMP, "xm-h1: encode mode %u\n", H1_ENC_MODE(ctrl));
    }
    s->jobs++;
    s->regs[H1_REG_ENC_CTRL / 4] &= ~H1_ENC_EN;
    s->regs[H1_REG_INTERRUPT / 4] |= status | H1_INT_BIT;
    xm_h1_update_irq(s);
}

static uint64_t xm_h1_read(void *opaque, hwaddr off, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID) {
        return H1_ID;
    }
    if (off == H1_SWREG63_CFG) {
        return H1_CFG;
    }
    return s->regs[off / 4];
}

static void xm_h1_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    XmH1State *s = XM_H1(opaque);

    if (off == H1_SWREG0_ID || off == H1_SWREG63_CFG) {
        return;
    }
    s->regs[off / 4] = val;
    if (off == H1_REG_INTERRUPT) {
        xm_h1_update_irq(s);
    } else if (off == H1_REG_ENC_CTRL && (val & H1_ENC_EN)) {
        xm_h1_start(s);
    }
}

static const MemoryRegionOps xm_h1_ops = {
    .read = xm_h1_read,
    .write = xm_h1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void xm_h1_reset(DeviceState *dev)
{
    XmH1State *s = XM_H1(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void xm_h1_init(Object *obj)
{
    XmH1State *s = XM_H1(obj);

    memory_region_init_io(&s->iomem, obj, &xm_h1_ops, s, TYPE_XM_H1,
                          H1_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property xm_h1_props[] = {
    DEFINE_PROP_STRING("dump-dir", XmH1State, dump_dir),
};

static void xm_h1_class_init(ObjectClass *klass, const void *data)
{
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_h1_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_h1_props);
    huff_build(&huff_dc[0], dc_luma_bits, dc_vals);
    huff_build(&huff_dc[1], dc_chroma_bits, dc_vals);
    huff_build(&huff_ac[0], ac_luma_bits, ac_luma_vals);
    huff_build(&huff_ac[1], ac_chroma_bits, ac_chroma_vals);
}

static const TypeInfo xm_h1_info = {
    .name          = TYPE_XM_H1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmH1State),
    .instance_init = xm_h1_init,
    .class_init    = xm_h1_class_init,
};

static void xm_h1_register_types(void)
{
    type_register_static(&xm_h1_info);
}

type_init(xm_h1_register_types)
