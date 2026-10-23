/*
 * Novatek NA51089 AI engines: the work of a job.
 *
 * kdrv_ai (vendor/ai/drivers/k_driver/source/kdrv_ai; the SDK's rtos tree
 * has it as C source, the Linux one as objects) drives three engines,
 * named in the "ai" device-tree node's order: NUE (0xF0C60000), NUE2
 * (0xF0D50000) and CNN (0xF0CB0000).  nvt-media runs their jobs and
 * linked lists, and calls here on a worker thread with a copy of the
 * engine's registers.  docs/novatek.md has the register maps.
 *
 * NUE, fully connected (SVM mode 0, kernel 1 in dot mode;
 * kdrv_ai_tran_nue_fc_parm):
 *
 *   k1[o][j] = ((sum_k ((x'[o][k] * w'[j][k]) >> fshift)) * gain
 *               >> gshift) + offset
 *
 * The weights are 12-bit, an LSB-first bit stream (8-bit with MODE1's
 * quantisation bit).  8-bit operands enter the 12-bit multiplier
 * top-aligned, x 16 each: vendor_ai_op's 8-bit inputs and weights come
 * out x 256, the classifier's u8 inputs by 12-bit weights x 16.  Output
 * paths 1 / 3 write k1 as s32; path 0 (result less rho) writes
 * (k1 >> rho_fmt) - rho[j], a 12-bit rho a row, through the ReLU and its
 * shift, as 16 or 8 bits.  Permute (mode 4) turns a planar map into each
 * pixel's channels together, a pixel OFSO bytes apart; softmax (mode 6)
 * takes Q.8 logits to Q.15 probabilities over each pixel's channels, and
 * writes them the same way.
 *
 * NUE2, preprocessing (nue2_reg.h, the 528 layout NA51089 uses): YUV to
 * RGB, bilinear scaling corners to corners (the step kdrv programs is
 * (in - 1) / (out - 1)), the DC or planar mean subtraction, the
 * scale-shift (a scale of 0: none) and a clamp to the output's range.
 *
 * CNN (cnn_reg.h): a layer, or a stripe of one, a job.  The convolution
 * takes 12-bit weights, an LSB-first stream ordered [set][channel][ky][kx],
 * with zero padding of half the kernel on the sides the job pads.  Then
 * come a scale-shift; without BN the bias (12 bits, << CONV_SHIFT_B), with
 * it BN from an 8-byte record a channel -- alpha [23:0], mean [35:24],
 * beta [47:36]; the element-wise add or max of a second map; another
 * scale-shift, a clamp to 16 bits and the activation.  out0 takes act0
 * through its scale-shift and, if 8-bit, drops the 8.8 datapath's fraction
 * bits; out1 takes act1 through its scale-shift, the pooling (local or
 * global) and the pooling's shifts.  Deconvolution spreads a map out with
 * holes (its convolution is jobs of its own), scale-up repeats pixels.
 * Depthwise layers are a job a channel.  The layouts, the 8.8 datapath and
 * where the bias goes are not in the SDK: they were fitted to the SDK's
 * ImageNet classifier and the AI turnkey's detectors, which then name
 * and find what their pictures hold.
 *
 * Nothing in the SDK fixes the rounding (half up here), NUE2's colour
 * matrix (full-range BT.601, chroma replicated) or its output channel
 * order.  The engines' other modes log as unimplemented.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/nvt-ai.h"

#define R(off)      (regs[(off) / 4])

static bool rd(hwaddr a, void *buf, size_t n)
{
    if (dma_memory_read(&address_space_memory, a, buf, n,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: cannot read 0x%" HWADDR_PRIx
                      "\n", a);
        return false;
    }
    return true;
}

/*
 * A job's writes, kept until the main loop commits them: a job a reset or
 * a load voids leaves guest memory as it was, and device windows are
 * written on the main thread only.
 */
struct NvtAiWrites {
    GByteArray *log;            /* address (8), length (4), data, ... */
};

NvtAiWrites *nvt_ai_writes_new(void)
{
    NvtAiWrites *w = g_new(NvtAiWrites, 1);

    w->log = g_byte_array_new();
    return w;
}

static void wr(NvtAiWrites *w, hwaddr a, const void *buf, size_t n)
{
    uint64_t a64 = a;
    uint32_t n32 = n;

    g_byte_array_append(w->log, (const uint8_t *)&a64, 8);
    g_byte_array_append(w->log, (const uint8_t *)&n32, 4);
    g_byte_array_append(w->log, buf, n);
}

void nvt_ai_writes_commit(NvtAiWrites *w)
{
    size_t at = 0;

    while (at + 12 <= w->log->len) {
        uint64_t a;
        uint32_t n;

        memcpy(&a, w->log->data + at, 8);
        memcpy(&n, w->log->data + at + 8, 4);
        if (dma_memory_write(&address_space_memory, a, w->log->data + at + 12,
                             n, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: cannot write 0x%"
                          PRIx64 "\n", a);
        }
        at += 12 + n;
    }
}

void nvt_ai_writes_free(NvtAiWrites *w)
{
    if (w) {
        g_byte_array_free(w->log, true);
        g_free(w);
    }
}

/* ---- NUE ---- */

#define NUE_MODE0           0x004
#define NUE_MODE1           0x008
#define NUE_IN_ADDR         0x00C
#define NUE_SV_ADDR         0x014
#define NUE_OUT_ADDR        0x02C
#define NUE_SIZE0           0x040
#define NUE_SIZE2           0x048
#define NUE_KERNEL0         0x04C
#define NUE_KERNEL1         0x050
#define NUE_RELU            0x054
#define NUE_RHO             0x058
#define NUE_RHO_ADDR        0x01C

#define NUE_M1_RHO_REG      (1u << 11)      /* rho from 0x058, not DRAM */
#define NUE_M1_QUANT        (1u << 21)      /* 8-bit weights */
#define NUE_M1_RELU         (1u << 25)
/*
 * kernel 2, AR, LUT, interlace, FCD VLC / sparse / k-means, intermediate
 * input, scientific notation
 */
#define NUE_M1_UNSUPPORTED  ((1u << 2) | (1u << 10) | (1u << 12) | \
                             (1u << 16) | (1u << 18) | (1u << 20) | \
                             (3u << 22) | (3u << 26))

#define NUE_MAX_SVW         9216
#define NUE_MAX_SVH         4096
#define NUE_MAX_OBJ         64
/*
 * The products a job may take, done at once on the vCPU: the drivers
 * split layers into jobs of 256 rows, far below this.
 */
#define NUE_MAX_MACS        (1u << 26)

/* a 12-bit signed element of an LSB-first bit stream, at a bit */
static int16_t w12_at(const uint8_t *p, size_t bit)
{
    uint32_t v = p[bit / 8] | (p[bit / 8 + 1] << 8);

    return sextract32(v >> (bit % 8), 0, 12);
}

static int16_t w12(const uint8_t *p, size_t i)
{
    return w12_at(p, i * 12);
}

#define NUE_MODE_PERMUTE    4
#define NUE_MODE_SOFTMAX    6
#define NUE_OFSO            0x034
#define NUE_SIZE1           0x044
#define NUE_SOFTMAX         0x0EC
#define NUE_IN_SS           0x0F0
#define NUE_PERMUTE         0x1C4
#define NUE_MAX_MAP         (4 * MiB)

/* the input scale-shift (0x0F0): [27:12] scale (0: none), [8:4] shift */
static int64_t nue_in_ss(const uint32_t *regs, int64_t v)
{
    uint32_t r = R(NUE_IN_SS);
    unsigned s = extract32(r, 4, 5);

    if (!extract32(r, 12, 16)) {
        return v;
    }
    v *= extract32(r, 12, 16);
    return r & 1 ? v * ((int64_t)1 << s) : v >> s;
}

static int64_t nue_sat(int64_t v, bool sign, bool wide)
{
    int64_t lo = sign ? (wide ? INT16_MIN : INT8_MIN) : 0;
    int64_t hi = sign ? (wide ? INT16_MAX : INT8_MAX) : (wide ? 65535 : 255);

    return MIN(MAX(v, lo), hi);
}

static void nue_put(uint8_t *p, int64_t v, bool wide)
{
    if (wide) {
        stw_le_p(p, v);
    } else {
        *p = v;
    }
}

/*
 * A planar C x H x W map, read whole: W [13:0] and H [28:16] at 0x040,
 * C [27:16] at 0x044, the elements' type from MODE0 bits 4 / 12.
 */
static int64_t *nue_read_map(const uint32_t *regs, unsigned *w, unsigned *h,
                             unsigned *c)
{
    uint32_t mode0 = R(NUE_MODE0);
    bool sign = mode0 & (1u << 4), wide = mode0 & (1u << 12);
    unsigned es = wide ? 2 : 1;
    size_t i, n;
    g_autofree uint8_t *buf = NULL;
    int64_t *v;

    *w = extract32(R(NUE_SIZE0), 0, 14);
    *h = extract32(R(NUE_SIZE0), 16, 13);
    *c = extract32(R(NUE_SIZE1), 16, 12);
    n = (size_t)*w * *h * *c;
    if (!n || n > NUE_MAX_MAP) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: NUE map %ux%ux%u\n", *w, *h,
                      *c);
        return NULL;
    }
    buf = g_malloc(n * es);
    if (!rd(R(NUE_IN_ADDR), buf, n * es)) {
        return NULL;
    }
    v = g_new(int64_t, n);
    for (i = 0; i < n; i++) {
        v[i] = nue_in_ss(regs, wide ? (sign ? (int16_t)lduw_le_p(buf + 2 * i)
                                            : lduw_le_p(buf + 2 * i))
                                    : (sign ? (int8_t)buf[i] : buf[i]));
    }
    return v;
}

/*
 * Permute, channel first (MODE1 bit 3 clear): planar C x H x W in, each
 * pixel's C elements together out, pixels OFSO (0x034) bytes apart (the
 * converter stripes wide maps by channels), through the shift at 0x1C4.
 */
static void nue_permute(const uint32_t *regs, NvtAiWrites *wl)
{
    uint32_t mode0 = R(NUE_MODE0), ps = R(NUE_PERMUTE);
    bool sign = mode0 & (1u << 8), wide = mode0 & (1u << 16);
    unsigned es = wide ? 2 : 1, w, h, c, x, k;
    unsigned sh = extract32(ps, 4, 3);
    g_autofree int64_t *v = NULL;
    g_autofree uint8_t *px = NULL;
    uint32_t ofso;

    if (R(NUE_MODE1) & (1u << 3)) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: NUE permute, height first\n");
        return;
    }
    v = nue_read_map(regs, &w, &h, &c);
    if (!v) {
        return;
    }
    ofso = extract32(R(NUE_OFSO), 0, 20);
    ofso = ofso ? ofso : c * es;
    px = g_malloc((size_t)c * es);
    for (x = 0; x < w * h; x++) {
        for (k = 0; k < c; k++) {
            int64_t e = v[(size_t)k * w * h + x];

            e = ps & 1 ? e * ((int64_t)1 << sh) : e >> sh;
            nue_put(px + k * es, nue_sat(e, sign, wide), wide);
        }
        wr(wl, R(NUE_OUT_ADDR) + (hwaddr)x * ofso, px, (size_t)c * es);
    }
}

/*
 * Softmax over each pixel's channels, in groups of OBJECT_NUM (0x0EC
 * [23:16]): the logits brought to Q.8 by the input shift ([7:4], bit 0
 * left), at most 12 bits; the probabilities in Q.15 through the output
 * shift ([15:12], bit 8 left); written each pixel's channels together.
 */
static void nue_softmax(const uint32_t *regs, NvtAiWrites *wl)
{
    uint32_t mode0 = R(NUE_MODE0), sf = R(NUE_SOFTMAX);
    bool sign = mode0 & (1u << 8), wide = mode0 & (1u << 16);
    unsigned es = wide ? 2 : 1, w, h, c, x, k, g, grp;
    unsigned ish = extract32(sf, 4, 4), osh = extract32(sf, 12, 4);
    g_autofree int64_t *v = NULL;
    g_autofree uint8_t *out = NULL;
    g_autofree double *e = NULL;

    v = nue_read_map(regs, &w, &h, &c);
    if (!v) {
        return;
    }
    grp = extract32(sf, 16, 8);
    grp = grp && grp <= c ? grp : c;
    out = g_malloc0((size_t)w * h * c * es);
    e = g_new(double, grp);
    for (x = 0; x < w * h; x++) {
        for (g = 0; g + grp <= c; g += grp) {
            double sum = 0, top = -1e300;

            for (k = 0; k < grp; k++) {
                int64_t l = v[(size_t)(g + k) * w * h + x];

                l = sf & 1 ? l * ((int64_t)1 << ish) : l >> ish;
                e[k] = MIN(MAX(l, -2048), 2047) / 256.0;
                top = MAX(top, e[k]);
            }
            for (k = 0; k < grp; k++) {
                e[k] = exp(e[k] - top);
                sum += e[k];
            }
            for (k = 0; k < grp; k++) {
                int64_t o = llround(e[k] / sum * 32768.0);

                o = sf & (1u << 8) ? o * ((int64_t)1 << osh) : o >> osh;
                nue_put(out + ((size_t)x * c + g + k) * es,
                        nue_sat(o, sign, wide), wide);
            }
        }
    }
    wr(wl, R(NUE_OUT_ADDR), out, (size_t)w * h * c * es);
}

void nvt_ai_nue_run(const uint32_t *regs, NvtAiWrites *wl)
{
    uint32_t mode0 = R(NUE_MODE0), mode1 = R(NUE_MODE1);
    uint32_t svw = R(NUE_SIZE0) & 0x3FFF, svh = (R(NUE_SIZE0) >> 16) & 0x1FFF;
    uint32_t obj = R(NUE_SIZE2) & 0x7F, path = (mode1 >> 8) & 3;
    uint32_t gain = R(NUE_KERNEL0) & 0xFFF, relu = R(NUE_RELU);
    unsigned gshift = (R(NUE_KERNEL0) >> 16) & 0xF;
    unsigned fshift = (R(NUE_KERNEL1) >> 24) & 0xF;
    unsigned rho_fmt = extract32(R(NUE_RHO), 16, 4) + 3;
    int32_t offset = sextract32(R(NUE_KERNEL1), 0, 21);
    bool sign = mode0 & (1u << 4), wide = mode0 & (1u << 12);
    bool w8 = mode1 & NUE_M1_QUANT;
    bool out_sign = mode0 & (1u << 8), out16 = mode0 & (1u << 16);
    unsigned esz = wide ? 2 : 1, osz = path ? 4 : (out16 ? 2 : 1);
    hwaddr in = R(NUE_IN_ADDR), sv = R(NUE_SV_ADDR) & ~3u;
    hwaddr out = R(NUE_OUT_ADDR);
    size_t wrow = w8 ? svw : (size_t)svw * 12 / 8 + 2;
    g_autofree uint8_t *x = NULL, *w = NULL, *rho = NULL, *res = NULL;
    g_autofree int64_t *xs = NULL;
    uint32_t o, j, k;

    if ((mode0 & 7) == NUE_MODE_PERMUTE) {
        nue_permute(regs, wl);
        return;
    } else if ((mode0 & 7) == NUE_MODE_SOFTMAX) {
        nue_softmax(regs, wl);
        return;
    }
    /* path 0: the result less rho (result mode 1); 1 / 3: the kernel's */
    if ((mode0 & 7) || (mode1 & 3) || path == 2 ||
        (!path && extract32(mode1, 4, 2) != 1) ||
        (mode1 & NUE_M1_UNSUPPORTED)) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: NUE mode 0x%x / 0x%x\n", mode0,
                      mode1);
        return;
    }
    if (!svw || !svh || !obj || svw > NUE_MAX_SVW || svh > NUE_MAX_SVH ||
        obj > NUE_MAX_OBJ) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: NUE %u rows of %u, %u "
                      "vectors\n", svh, svw, obj);
        return;
    }
    if ((uint64_t)svw * svh * obj > NUE_MAX_MACS) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: NUE job of %u rows of %u, %u "
                      "vectors, too big to run at once\n", svh, svw, obj);
        return;
    }
    x = g_malloc((size_t)obj * svw * esz);
    xs = g_new(int64_t, (size_t)obj * svw);
    w = g_malloc(wrow + 2);
    res = g_malloc((size_t)obj * svh * osz);
    if (!rd(in, x, (size_t)obj * svw * esz)) {
        return;
    }
    /* 8-bit operands enter the 12-bit multiplier top-aligned, x 16 */
    for (k = 0; k < obj * svw; k++) {
        xs[k] = wide ? (sign ? (int16_t)lduw_le_p(x + 2 * k)
                             : lduw_le_p(x + 2 * k))
                     : (sign ? (int8_t)x[k] : x[k]) * 16;
    }
    if (path == 0 && !(mode1 & NUE_M1_RHO_REG)) {
        rho = g_malloc((size_t)svh * 12 / 8 + 2);
        if (!rd(R(NUE_RHO_ADDR) & ~3u, rho, ((size_t)svh * 12 + 7) / 8)) {
            return;
        }
    }
    for (j = 0; j < svh; j++) {
        /* 12-bit rows follow on bit by bit: one may start mid-byte */
        size_t bit = w8 ? 0 : (size_t)j * svw * 12 % 8;

        if (!rd(sv + (w8 ? (hwaddr)j * svw : (hwaddr)j * svw * 12 / 8), w,
                w8 ? svw : (bit + (size_t)svw * 12 + 7) / 8)) {
            return;
        }
        for (o = 0; o < obj; o++) {
            const int64_t *xo = xs + (size_t)o * svw;
            size_t at = (size_t)o * svh + j;
            int64_t acc = 0, r;

            for (k = 0; k < svw; k++) {
                int64_t wk = w8 ? (int8_t)w[k] * 16
                                : w12_at(w, bit + 12 * k);

                acc += (xo[k] * wk) >> fshift;
            }
            r = ((acc * gain) >> gshift) + offset;
            if (path) {                 /* the kernel's result, s32 */
                stl_le_p(res + 4 * at, r);
                continue;
            }
            /* less rho, the activation and its shift, 8 or 16 bits */
            r = (r >> rho_fmt) - (rho ? w12(rho, j)
                                      : sextract32(R(NUE_RHO), 0, 12));
            r = MIN(MAX(r, -(1ll << 40)), 1ll << 40);
            if ((mode1 & NUE_M1_RELU) && r < 0) {
                r = (r * sextract32(relu, 0, 11)) >>
                    (7 + extract32(relu, 12, 4));
            }
            r = relu & (1u << 16) ? r * ((int64_t)1 << extract32(relu, 20, 4))
                                  : r >> extract32(relu, 20, 4);
            if (out16) {
                stw_le_p(res + 2 * at, out_sign ? MIN(MAX(r, INT16_MIN),
                                                      INT16_MAX)
                                                : MIN(MAX(r, 0), 65535));
            } else {
                r >>= 8;
                res[at] = out_sign ? MIN(MAX(r, INT8_MIN), INT8_MAX)
                                   : MIN(MAX(r, 0), 255);
            }
        }
    }
    wr(wl, out, res, (size_t)obj * svh * osz);
}

/* ---- NUE2 ---- */

#define NUE2_FUNC           0x004
#define NUE2_IN0            0x008
#define NUE2_OUT0           0x018
#define NUE2_IN_LOFS0       0x024
#define NUE2_OUT_LOFS0      0x030
#define NUE2_IN_SIZE        0x044
#define NUE2_OUT_SIZE       0x058
#define NUE2_SUB_SIZE       0x05C
#define NUE2_SUB            0x060
#define NUE2_MEAN_SHIFT     0x07C

#define NUE2_YUV2RGB        (1u << 0)
#define NUE2_SUB_EN         (1u << 1)
#define NUE2_UNSUPPORTED    (7u << 2)       /* pad, HSV, rotate */
#define NUE2_OUT_SIGNED     (1u << 14)
#define NUE2_SUB_PLANAR     (1u << 15)

#define NUE2_MAX            2048

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

void nvt_ai_nue2_run(const uint32_t *regs, NvtAiWrites *wl)
{
    uint32_t func = R(NUE2_FUNC), fmt = (func >> 12) & 3;
    uint32_t iw = R(NUE2_IN_SIZE) & 0x7FF, ih = (R(NUE2_IN_SIZE) >> 16) & 0xFFF;
    uint32_t ow = R(NUE2_OUT_SIZE) & 0x7FF;
    uint32_t oh = (R(NUE2_OUT_SIZE) >> 16) & 0xFFF;
    uint32_t sub = R(NUE2_SUB), ms = R(NUE2_MEAN_SHIFT);
    uint32_t sw = R(NUE2_SUB_SIZE) & 0x7FF;
    uint32_t sh = (R(NUE2_SUB_SIZE) >> 16) & 0xFFF;
    unsigned dup = (sub >> 24) & 3, mshift = (ms >> 4) & 0x1F, c;
    int32_t mscale = (ms >> 12) & 0xFFFF;
    bool sign = func & NUE2_OUT_SIGNED;
    g_autofree uint8_t *rgb = NULL, *mean = NULL, *line = NULL;
    g_autofree uint8_t *uvl = NULL, *outl = NULL;
    uint32_t x, y;

    if ((func & NUE2_UNSUPPORTED) || (fmt != 0 && fmt != 3) ||
        (fmt == 0) != !!(func & NUE2_YUV2RGB)) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: NUE2 function 0x%x\n", func);
        return;
    }
    if (!iw || !ih || !ow || !oh || iw > NUE2_MAX || ih > NUE2_MAX ||
        ow > NUE2_MAX || oh > NUE2_MAX || (((iw | ih) & 1) && !fmt)) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: NUE2 %ux%u to %ux%u\n", iw,
                      ih, ow, oh);
        return;
    }
    /* the input as interleaved RGB */
    rgb = g_malloc((size_t)iw * ih * 3);
    line = g_malloc(iw);
    uvl = g_malloc(iw);
    for (y = 0; y < ih; y++) {
        uint8_t *dst = rgb + (size_t)y * iw * 3;

        if (fmt == 0) {
            uint32_t ylofs = R(NUE2_IN_LOFS0) & 0x1FFFF;
            uint32_t clofs = R(NUE2_IN_LOFS0 + 4) & 0x1FFFF;

            if (!rd(R(NUE2_IN0) + (hwaddr)y * ylofs, line, iw) ||
                !rd(R(NUE2_IN0 + 4) + (hwaddr)(y / 2) * clofs, uvl, iw)) {
                return;
            }
            for (x = 0; x < iw; x++) {
                int yy = line[x], u = uvl[x & ~1u] - 128;
                int v = uvl[x | 1] - 128;

                /* full-range BT.601 */
                dst[3 * x] = clamp8(yy + ((359 * v + 128) >> 8));
                dst[3 * x + 1] = clamp8(yy - ((88 * u + 183 * v + 128) >> 8));
                dst[3 * x + 2] = clamp8(yy + ((454 * u + 128) >> 8));
            }
        } else {
            for (c = 0; c < 3; c++) {
                uint32_t lofs = R(NUE2_IN_LOFS0 + 4 * c) & 0x1FFFF;

                if (!rd(R(NUE2_IN0 + 4 * c) + (hwaddr)y * lofs, line, iw)) {
                    return;
                }
                for (x = 0; x < iw; x++) {
                    dst[3 * x + c] = line[x];
                }
            }
        }
    }
    /* the planar mean, 3 bytes a pixel, before its replication */
    if ((func & NUE2_SUB_EN) && (func & NUE2_SUB_PLANAR)) {
        uint32_t lofs = R(NUE2_IN_LOFS0 + 8) & 0x1FFFF;

        if (!sw || !sh || sw > NUE2_MAX || sh > NUE2_MAX || lofs < sw * 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: NUE2 mean %ux%u\n", sw,
                          sh);
            return;
        }
        mean = g_malloc((size_t)sw * sh * 3);
        for (y = 0; y < sh; y++) {
            if (!rd(R(NUE2_IN0 + 8) + (hwaddr)y * lofs,
                    mean + (size_t)y * sw * 3, (size_t)sw * 3)) {
                return;
            }
        }
    }
    outl = g_malloc(ow);
    for (c = 0; c < 3; c++) {
        uint32_t lofs = R(NUE2_OUT_LOFS0 + 4 * c) & 0x1FFFF;
        hwaddr oa = R(NUE2_OUT0 + 4 * c) & 0x7FFFFFFF;

        if (lofs < ow) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: NUE2 output line offset "
                          "%u\n", lofs);
            return;
        }
        for (y = 0; y < oh; y++) {
            /* bilinear, corner to corner, 16.16 */
            uint64_t fy = oh > 1 ? (uint64_t)y * ((ih - 1) << 16) / (oh - 1)
                                 : 0;
            uint32_t y0 = fy >> 16, y1 = MIN(y0 + 1, ih - 1);
            uint32_t ay = fy & 0xFFFF;

            for (x = 0; x < ow; x++) {
                uint64_t fx = ow > 1 ? (uint64_t)x * ((iw - 1) << 16) /
                                       (ow - 1) : 0;
                uint32_t x0 = fx >> 16, x1 = MIN(x0 + 1, iw - 1);
                uint32_t ax = fx & 0xFFFF;
                const uint8_t *r0 = rgb + (size_t)y0 * iw * 3 + c;
                const uint8_t *r1 = rgb + (size_t)y1 * iw * 3 + c;
                uint64_t top = r0[3 * x0] * (0x10000 - ax) + r0[3 * x1] * ax;
                uint64_t bot = r1[3 * x0] * (0x10000 - ax) + r1[3 * x1] * ax;
                int64_t v = (top * (0x10000 - ay) + bot * ay + (1ull << 31))
                            >> 32;

                if (func & NUE2_SUB_EN) {
                    if (mean) {
                        uint32_t mx = MIN(x >> dup, sw - 1);
                        uint32_t my = MIN(y >> dup, sh - 1);

                        v -= mean[((size_t)my * sw + mx) * 3 + c];
                    } else {
                        v -= (sub >> (8 * c)) & 0xFF;
                    }
                    if (mscale) {           /* 0: no scale-shift */
                        v *= mscale;
                        v = ms & 1 ? v * ((int64_t)1 << mshift)
                                   : v >> mshift;
                    }
                }
                outl[x] = sign ? (uint8_t)MIN(MAX(v, -128), 127)
                               : clamp8(v);
            }
            wr(wl, oa + (hwaddr)y * lofs, outl, ow);
        }
    }
}

/* ---- CNN ---- */

#define CNN_MODE            0x004
#define CNN_IN_ADDR         0x010
#define CNN_IN_LOFS         0x020
#define CNN_IN_COFS         0x024
#define CNN_ELT_LOFS        0x028
#define CNN_ELT_COFS        0x02C
#define CNN_W_ADDR          0x030
#define CNN_BN_ADDR         0x038
#define CNN_ELT_ADDR        0x03C
#define CNN_OUT0_ADDR       0x044
#define CNN_OUT1_ADDR       0x048
#define CNN_OUT0_LOFS       0x04C
#define CNN_OUT0_COFS       0x050
#define CNN_OUT1_LOFS       0x054
#define CNN_OUT1_COFS       0x058
#define CNN_INSIZE0         0x05C
#define CNN_INSIZE1         0x060
#define CNN_IO              0x064
#define CNN_CONV0           0x068
#define CNN_CONV1           0x06C
#define CNN_BN              0x078
#define CNN_ELT0            0x084
#define CNN_ELT1            0x088
#define CNN_ACT0            0x090
#define CNN_ACT1            0x094
#define CNN_POOL0           0x098
#define CNN_POOL1           0x09C
#define CNN_DECONV          0x0A0
#define CNN_SCALEUP         0x0A4
#define CNN_SS_ELT_IN       0x0A8
#define CNN_SS_CONV         0x0AC
#define CNN_SS_BN           0x0B0
#define CNN_SS_OUT0         0x0B4
#define CNN_SS_ACT1         0x0B8
#define CNN_SS_POOL         0x0BC

#define CNN_M_ELT_MAX       (2u << 2)
#define CNN_M_BN            (1u << 8)
#define CNN_M_ACT           (1u << 9)
#define CNN_M_POOL          (1u << 10)
#define CNN_M_ELT           (1u << 11)
#define CNN_M_CONV          (1u << 12)
/* eltwise multiply, tanh / LUT, pre-act, FCD */
#define CNN_M_UNSUPPORTED   ((1u << 2) | (1u << 4) | (1u << 13) | \
                             (0x1Fu << 16))

#define CNN_IO_SIGNED       (1u << 4)
#define CNN_IO_WIDE         (1u << 5)
#define CNN_IO_OUT0_SIGNED  (1u << 6)
#define CNN_IO_OUT1_SIGNED  (1u << 7)
#define CNN_IO_OUT0_EN      (1u << 8)
#define CNN_IO_OUT0_WIDE    (1u << 9)
#define CNN_IO_OUT1_WIDE    (1u << 12)
#define CNN_IO_OUT1_EN      (1u << 13)
#define CNN_IO_POOL_SIGNED  (1u << 14)
/* channel stripes, out0's intermediate mode and row offset, flip, interlace */
#define CNN_IO_UNSUPPORTED  ((1u << 3) | (3u << 10) | (7u << 16) | \
                             (7u << 20))

/* the largest feature map, weight set and job a job may have */
#define CNN_MAX_ELEMS       (4 * MiB)
#define CNN_MAX_WEIGHTS     (16 * MiB)
#define CNN_MAX_MACS        (1ull << 32)
/* intermediate values saturate here, well inside 64-bit arithmetic */
#define CNN_LIM             (1ll << 40)

typedef struct {
    unsigned c, h, w;
    int64_t *v;
} CnnMap;

static int64_t cnn_shr(int64_t v, unsigned s)
{
    return s ? (v + (1ll << (s - 1))) >> s : v;
}

static int64_t cnn_clamp(int64_t v)
{
    return MIN(MAX(v, -CNN_LIM), CNN_LIM);
}

/* v << s, saturating at CNN_LIM */
static int64_t cnn_shl(int64_t v, unsigned s)
{
    v = cnn_clamp(v);
    if (v && (s > 40 || (v < 0 ? -v : v) > CNN_LIM >> s)) {
        return v < 0 ? -CNN_LIM : CNN_LIM;
    }
    return v * ((int64_t)1 << s);
}

/* a scale-shift stage: [27:12] scale, [8:4] shift, bit 0 left */
static int64_t cnn_ss(int64_t v, uint32_t reg)
{
    unsigned s = extract32(reg, 4, 5);

    v = cnn_clamp(v) * extract32(reg, 12, 16);
    return reg & 1 ? cnn_shl(v, s) : cnn_clamp(cnn_shr(v, s));
}

static int64_t cnn_sat(int64_t v, bool sign, bool wide)
{
    int64_t lo = sign ? (wide ? INT16_MIN : INT8_MIN) : 0;
    int64_t hi = sign ? (wide ? INT16_MAX : INT8_MAX) : (wide ? 65535 : 255);

    return MIN(MAX(v, lo), hi);
}

static bool cnn_read(hwaddr a, uint32_t lofs, uint32_t cofs, bool sign,
                     bool wide, CnnMap *m)
{
    unsigned es = wide ? 2 : 1, c, y, x;
    g_autofree uint8_t *line = g_malloc((size_t)m->w * es);

    for (c = 0; c < m->c; c++) {
        for (y = 0; y < m->h; y++) {
            int64_t *d = m->v + ((size_t)c * m->h + y) * m->w;

            if (!rd(a + (hwaddr)c * cofs + (hwaddr)y * lofs, line,
                    (size_t)m->w * es)) {
                return false;
            }
            for (x = 0; x < m->w; x++) {
                d[x] = wide ? (sign ? (int16_t)lduw_le_p(line + 2 * x)
                                    : lduw_le_p(line + 2 * x))
                            : (sign ? (int8_t)line[x] : line[x]);
            }
        }
    }
    return true;
}

static void cnn_write(NvtAiWrites *wl, hwaddr a, uint32_t lofs,
                      uint32_t cofs, bool sign, bool wide, const CnnMap *m)
{
    unsigned es = wide ? 2 : 1, c, y, x;
    g_autofree uint8_t *line = g_malloc((size_t)m->w * es);

    for (c = 0; c < m->c; c++) {
        for (y = 0; y < m->h; y++) {
            const int64_t *s = m->v + ((size_t)c * m->h + y) * m->w;

            for (x = 0; x < m->w; x++) {
                int64_t v = cnn_sat(s[x], sign, wide);

                if (wide) {
                    stw_le_p(line + 2 * x, v);
                } else {
                    line[x] = v;
                }
            }
            wr(wl, a + (hwaddr)c * cofs + (hwaddr)y * lofs, line,
               (size_t)m->w * es);
        }
    }
}

/* the 12-bit signed weights, an LSB-first bit stream */
static int16_t cnn_w12(const uint8_t *p, size_t i)
{
    size_t bit = i * 12;
    uint32_t v = p[bit / 8] | (p[bit / 8 + 1] << 8);

    return sextract32(v >> (bit % 8), 0, 12);
}

/* conv0 [3:0]: 11x11 7x7 5x5 3x3 1x1 7x1 1x7 5x1 1x5 3x1 1x3 9x9 (w x h) */
static const uint8_t cnn_ker[12][2] = {
    { 11, 11 }, { 7, 7 }, { 5, 5 }, { 3, 3 }, { 1, 1 }, { 7, 1 },
    { 1, 7 }, { 5, 1 }, { 1, 5 }, { 3, 1 }, { 1, 3 }, { 9, 9 },
};

static bool cnn_conv(const uint32_t *regs, const CnnMap *in, CnnMap *out)
{
    uint32_t conv0 = R(CNN_CONV0), pads = R(CNN_CONV1);
    unsigned sel = extract32(conv0, 0, 4), sets = extract32(conv0, 8, 12);
    unsigned st = 1u << extract32(conv0, 4, 2), kw, kh, hkw, hkh;
    unsigned pt, pb, pl, pr, ph, pw, s, c, ky, kx, oy, ox;
    size_t nw;
    g_autofree uint8_t *wb = NULL;
    g_autofree int32_t *xp = NULL;

    if (sel >= ARRAY_SIZE(cnn_ker) || st > 4 || !sets ||
        extract32(conv0, 20, 3)) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: CNN conv 0x%x / 0x%x\n", conv0,
                      pads);
        return false;
    }
    kw = cnn_ker[sel][0];
    kh = cnn_ker[sel][1];
    hkw = kw > 1 ? kw / 2 : 0;
    hkh = kh > 1 ? kh / 2 : 0;
    pt = pads & 1 ? hkh : 0;
    pb = pads & 2 ? hkh : 0;
    pl = pads & 4 ? hkw : 0;
    pr = pads & 8 ? hkw : 0;
    if (in->h + pt + pb < kh || in->w + pl + pr < kw) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: CNN %ux%u input for a "
                      "%ux%u kernel\n", in->w, in->h, kw, kh);
        return false;
    }
    out->c = sets;
    out->h = (in->h + pt + pb - kh) / st + 1;
    out->w = (in->w + pl + pr - kw) / st + 1;
    nw = (size_t)sets * in->c * kh * kw;
    if (nw > CNN_MAX_WEIGHTS || (uint64_t)nw * out->h * out->w > CNN_MAX_MACS ||
        (size_t)out->c * out->h * out->w > CNN_MAX_ELEMS) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: CNN job too big to run at once\n");
        return false;
    }
    wb = g_malloc0(nw * 12 / 8 + 2);
    if (!rd(R(CNN_W_ADDR) & ~3u, wb, (nw * 12 + 7) / 8)) {
        return false;
    }
    /* the input zero-padded, so the inner loop has no edges */
    ph = in->h + pt + pb;
    pw = in->w + pl + pr;
    xp = g_new0(int32_t, (size_t)in->c * ph * pw);
    for (c = 0; c < in->c; c++) {
        for (oy = 0; oy < in->h; oy++) {
            const int64_t *srow = in->v + ((size_t)c * in->h + oy) * in->w;
            int32_t *drow = xp + ((size_t)c * ph + oy + pt) * pw + pl;

            for (ox = 0; ox < in->w; ox++) {
                drow[ox] = srow[ox];
            }
        }
    }
    out->v = g_new0(int64_t, (size_t)out->c * out->h * out->w);
    for (s = 0; s < sets; s++) {
        int64_t *o = out->v + (size_t)s * out->h * out->w;

        for (c = 0; c < in->c; c++) {
            const int32_t *ip = xp + (size_t)c * ph * pw;

            for (ky = 0; ky < kh; ky++) {
                for (kx = 0; kx < kw; kx++) {
                    int32_t w = cnn_w12(wb, (((size_t)s * in->c + c) * kh +
                                             ky) * kw + kx);

                    if (!w) {
                        continue;
                    }
                    for (oy = 0; oy < out->h; oy++) {
                        const int32_t *row = ip + (size_t)(oy * st + ky) * pw +
                                             kx;
                        int64_t *orow = o + (size_t)oy * out->w;

                        if (st == 1) {
                            for (ox = 0; ox < out->w; ox++) {
                                orow[ox] += (int64_t)w * row[ox];
                            }
                        } else {
                            for (ox = 0; ox < out->w; ox++) {
                                orow[ox] += (int64_t)w * row[ox * st];
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

/* per output channel: alpha [23:0], mean [35:24] and beta [47:36] signed */
static bool cnn_bn(const uint32_t *regs, CnnMap *m)
{
    uint32_t bn = R(CNN_BN);
    unsigned shm = extract32(bn, 0, 5), shb = extract32(bn, 8, 5);
    unsigned sha = extract32(bn, 21, 5), c;
    size_t i, n = (size_t)m->h * m->w;
    g_autofree uint64_t *rec = g_new(uint64_t, m->c);

    if (!rd(R(CNN_BN_ADDR) & ~3u, rec, (size_t)m->c * 8)) {
        return false;
    }
    for (c = 0; c < m->c; c++) {
        uint64_t r = le64_to_cpu(rec[c]);
        int64_t alpha = extract64(r, 0, 24);
        int64_t mean = cnn_shl(sextract64(r, 24, 12), shm);
        int64_t beta = cnn_shl(sextract64(r, 36, 12), shb);
        int64_t *v = m->v + (size_t)c * n;

        for (i = 0; i < n; i++) {
            /* x alpha (24 bits) of a difference within 2^38 */
            int64_t d = MIN(MAX(v[i] - mean, -(1ll << 38)), 1ll << 38);

            v[i] = cnn_clamp(cnn_shr(d * alpha, sha) + beta);
        }
    }
    return true;
}

/* ReLU, leaky by an 11-bit slope >> (7 + [15:12]) */
static int64_t cnn_act(int64_t v, uint32_t reg)
{
    if (v >= 0) {
        return v;
    }
    return cnn_shr(v * sextract32(reg, 0, 11), 7 + extract32(reg, 12, 4));
}

static void cnn_pool(const uint32_t *regs, CnnMap *m)
{
    uint32_t p0 = R(CNN_POOL0), p1 = R(CNN_POOL1);
    bool avg = p0 & 1;
    size_t n = (size_t)m->h * m->w;
    unsigned c, y, x, ky, kx;

    if (R(CNN_MODE) & (1u << 5)) {          /* global */
        unsigned mul = extract32(p1, 8, 11), sh = extract32(p1, 20, 5) + 10;

        for (c = 0; c < m->c; c++) {
            const int64_t *v = m->v + (size_t)c * n;
            int64_t r = v[0];
            size_t i;

            if (avg) {
                for (r = 0, i = 0; i < n; i++) {
                    r += v[i];
                }
                r = cnn_shr(MIN(MAX(r, -(1ll << 48)), 1ll << 48) * mul, sh);
            } else {
                for (i = 1; i < n; i++) {
                    r = MAX(r, v[i]);
                }
            }
            m->v[c] = r;
        }
        m->h = m->w = 1;
    } else {                                 /* local, k x k, ceil or floor */
        unsigned k = 2 + extract32(p0, 8, 2), s = p0 & 0x10 ? 2 : 1;
        unsigned hk = k / 2, pt = p0 & 0x1000 ? hk : 0;
        unsigned pb = p0 & 0x2000 ? hk : 0, pl = p0 & 0x4000 ? hk : 0;
        unsigned pr = p0 & 0x8000 ? hk : 0, oh, ow;
        bool floor = p0 & 2, bound = p1 & 0x80;
        int64_t *o;

        if (m->h + pt + pb < k || m->w + pl + pr < k) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: CNN pooling a %ux%u "
                          "map with a %ux%u kernel\n", m->w, m->h, k, k);
            m->c = 0;
            return;
        }
        oh = (m->h + pt + pb - k + (floor ? 0 : s - 1)) / s + 1;
        ow = (m->w + pl + pr - k + (floor ? 0 : s - 1)) / s + 1;
        o = g_new(int64_t, (size_t)m->c * oh * ow);
        for (c = 0; c < m->c; c++) {
            const int64_t *v = m->v + (size_t)c * n;

            for (y = 0; y < oh; y++) {
                for (x = 0; x < ow; x++) {
                    int64_t r = avg ? 0 : INT64_MIN;
                    unsigned cnt = 0;

                    for (ky = 0; ky < k; ky++) {
                        int iy = (int)(y * s + ky) - (int)pt;

                        for (kx = 0; kx < k; kx++) {
                            int ix = (int)(x * s + kx) - (int)pl;
                            int64_t e = 0;

                            if (iy >= 0 && iy < (int)m->h && ix >= 0 &&
                                ix < (int)m->w) {
                                e = v[(size_t)iy * m->w + ix];
                                cnt++;
                            } else if (!avg) {
                                continue;
                            }
                            r = avg ? r + e : MAX(r, e);
                        }
                    }
                    if (avg) {
                        r /= bound ? MAX(cnt, 1u) : k * k;
                    }
                    o[((size_t)c * oh + y) * ow + x] = r == INT64_MIN ? 0 : r;
                }
            }
        }
        g_free(m->v);
        m->v = o;
        m->h = oh;
        m->w = ow;
    }
    if (p1 & 1) {
        for (n = 0; n < (size_t)m->c * m->h * m->w; n++) {
            m->v[n] = cnn_shl(m->v[n], extract32(p1, 4, 3));
        }
    } else {
        for (n = 0; n < (size_t)m->c * m->h * m->w; n++) {
            m->v[n] = cnn_shr(m->v[n], extract32(p1, 4, 3));
        }
    }
}

/*
 * Without BN, a convolution's bias: 12 bits in a 16-bit slot a set,
 * << CONV_SHIFT_B.  Where it goes is fitted to the AI turnkey's detectors:
 * after the convolution's scale-shift (0x0AC) for an 8-bit out0 -- before
 * it, the person and face detectors find nothing -- and before it for a
 * 16-bit out0 (the detectors' heads) -- after it, the person / vehicle
 * detector finds no people.
 */
static bool cnn_bias(const uint32_t *regs, CnnMap *m)
{
    g_autofree uint16_t *bias = g_new(uint16_t, m->c);
    unsigned bs = extract32(R(CNN_CONV1), 8, 5), c;
    size_t i, n = (size_t)m->h * m->w;

    if (!rd(R(CNN_BN_ADDR) & ~3u, bias, (size_t)m->c * 2)) {
        return false;
    }
    for (c = 0; c < m->c; c++) {
        int64_t b = cnn_shl(sextract32(le16_to_cpu(bias[c]), 0, 12), bs);
        int64_t *v = m->v + (size_t)c * n;

        for (i = 0; i < n; i++) {
            v[i] = cnn_clamp(v[i] + b);
        }
    }
    return true;
}

/*
 * Deconvolution (MODE 1) spreads the input out, stride - 1 holes between
 * pixels, with pad rows / columns (counts 0..3) -- the convolution after
 * it is a job of its own; scale-up (MODE 2) repeats each pixel rate x rate
 * times.  Holes and pads take the pad value.
 */
static bool cnn_upsample(const uint32_t *regs, const CnnMap *in, CnnMap *out)
{
    bool deconv = (R(CNN_MODE) & 3) == 1;
    uint32_t r = R(deconv ? CNN_DECONV : CNN_SCALEUP);
    unsigned k = 1u << extract32(r, deconv ? 24 : 0, 2);
    unsigned sh = deconv ? 0 : 2;
    unsigned pt = extract32(r, sh, 2), pb = extract32(r, sh + 2, 2);
    unsigned pl = extract32(r, sh + 4, 2), pr = extract32(r, sh + 6, 2);
    int64_t padv = deconv ? sextract32(r, 8, 16) : sextract32(r, 12, 16);
    unsigned c, y, x;
    size_t i, n;

    out->c = in->c;
    out->w = (deconv ? (in->w - 1) * k + 1 : in->w * k) + pl + pr;
    out->h = (deconv ? (in->h - 1) * k + 1 : in->h * k) + pt + pb;
    n = (size_t)out->c * out->h * out->w;
    if (n > CNN_MAX_ELEMS) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: CNN job too big to run at once\n");
        return false;
    }
    out->v = g_new(int64_t, n);
    for (i = 0; i < n; i++) {
        out->v[i] = padv;
    }
    for (c = 0; c < out->c; c++) {
        for (y = 0; y < out->h - pt - pb; y++) {
            for (x = 0; x < out->w - pl - pr; x++) {
                int64_t *o = out->v + ((size_t)c * out->h + y + pt) * out->w +
                             x + pl;

                if (!deconv) {
                    *o = in->v[((size_t)c * in->h + y / k) * in->w + x / k];
                } else if (!(y % k) && !(x % k)) {
                    *o = in->v[((size_t)c * in->h + y / k) * in->w + x / k];
                }
            }
        }
    }
    return true;
}

/*
 * Element-wise (MODE bit 11), after BN: the second input from SAI7 (its
 * type ELT0 [1:0]) through its scale-shift 0x0A8, then
 * ((a * coef0) >> shift0) + ((b * coef1) >> shift1) -- or the larger of
 * the two (MODE [3:2] = 2) -- >> the out shift.
 */
static bool cnn_eltwise(const uint32_t *regs, CnnMap *m)
{
    uint32_t e0 = R(CNN_ELT0), e1 = R(CNN_ELT1);
    bool wide = e0 & 2;
    unsigned es = wide ? 2 : 1;
    int64_t c0 = (int16_t)extract32(e1, 0, 16);
    int64_t c1 = (int16_t)extract32(e1, 16, 16);
    unsigned s0 = extract32(e0, 4, 5), s1 = extract32(e0, 12, 5);
    unsigned so = extract32(e0, 20, 5);
    CnnMap b = { .c = m->c, .h = m->h, .w = m->w };
    uint32_t lofs = R(CNN_ELT_LOFS) ? R(CNN_ELT_LOFS) : m->w * es;
    uint32_t cofs = R(CNN_ELT_COFS) ? R(CNN_ELT_COFS) : lofs * m->h;
    size_t i, n = (size_t)m->c * m->h * m->w;
    bool ok;

    b.v = g_new(int64_t, n);
    ok = cnn_read(R(CNN_ELT_ADDR), lofs, cofs, e0 & 1, wide, &b);
    for (i = 0; ok && i < n; i++) {
        int64_t va = cnn_clamp(cnn_shr(cnn_clamp(m->v[i]) * c0, s0));
        int64_t vb = cnn_clamp(cnn_shr(cnn_ss(b.v[i], R(CNN_SS_ELT_IN)) * c1,
                                       s1));

        m->v[i] = cnn_shr(R(CNN_MODE) & CNN_M_ELT_MAX ? MAX(va, vb) : va + vb,
                          so);
    }
    g_free(b.v);
    return ok;
}

void nvt_ai_cnn_run(const uint32_t *regs, NvtAiWrites *wl)
{
    uint32_t mode = R(CNN_MODE), io = R(CNN_IO);
    bool wide = io & CNN_IO_WIDE;
    unsigned es = wide ? 2 : 1;
    CnnMap in = {
        .w = extract32(R(CNN_INSIZE0), 0, 10),
        .h = extract32(R(CNN_INSIZE0), 12, 10),
        .c = extract32(R(CNN_INSIZE1), 0, 12),
    };
    CnnMap v = { 0 }, a1 = { 0 };
    uint32_t lofs, cofs;
    size_t i, n;
    bool bias;

    if ((mode & CNN_M_UNSUPPORTED) || (mode & 3) == 3 ||
        (io & CNN_IO_UNSUPPORTED) || extract32(R(CNN_INSIZE1), 12, 7) > 1 ||
        (R(CNN_ACT0) & (1u << 16)) || (R(CNN_ACT1) & (1u << 16))) {
        qemu_log_mask(LOG_UNIMP, "nvt-ai: CNN mode 0x%x io 0x%x\n", mode, io);
        return;
    }
    if (!in.w || !in.h || !in.c ||
        (size_t)in.c * in.h * in.w > CNN_MAX_ELEMS) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: CNN input %ux%ux%u\n", in.w,
                      in.h, in.c);
        return;
    }
    lofs = R(CNN_IN_LOFS) ? R(CNN_IN_LOFS) : in.w * es;
    cofs = R(CNN_IN_COFS) ? R(CNN_IN_COFS) : lofs * in.h;
    in.v = g_new(int64_t, (size_t)in.c * in.h * in.w);
    if (!cnn_read(R(CNN_IN_ADDR), lofs, cofs, io & CNN_IO_SIGNED, wide, &in)) {
        goto out;
    }
    if (mode & 3) {
        /* deconvolution or scale-up alone: out0 through its scale-shift */
        if (!(io & CNN_IO_OUT0_EN) || !cnn_upsample(regs, &in, &v)) {
            goto out;
        }
        for (i = 0; i < (size_t)v.c * v.h * v.w; i++) {
            v.v[i] = cnn_ss(v.v[i], R(CNN_SS_OUT0));
        }
        cnn_write(wl, R(CNN_OUT0_ADDR), R(CNN_OUT0_LOFS) ? R(CNN_OUT0_LOFS) :
                  v.w * (io & CNN_IO_OUT0_WIDE ? 2 : 1),
                  R(CNN_OUT0_COFS) ? R(CNN_OUT0_COFS) :
                  v.w * v.h * (io & CNN_IO_OUT0_WIDE ? 2 : 1),
                  io & CNN_IO_OUT0_SIGNED, io & CNN_IO_OUT0_WIDE, &v);
        goto out;
    }
    if (mode & CNN_M_CONV) {
        if (wide) {             /* 16-bit features lose 8 bits first */
            for (i = 0; i < (size_t)in.c * in.h * in.w; i++) {
                in.v[i] >>= 8;
            }
        }
        if (!cnn_conv(regs, &in, &v)) {
            goto out;
        }
    } else if (mode & (CNN_M_BN | CNN_M_ELT)) {
        /* BN or element-wise without a convolution: the input as it is */
        v = in;
        in.v = NULL;
    } else {
        /* pooling or activation alone: no scale-shift before them */
        v = in;
        in.v = NULL;
        goto stages_done;
    }
    n = (size_t)v.c * v.h * v.w;
    bias = (mode & CNN_M_CONV) && !(mode & CNN_M_BN);
    if (bias && (io & CNN_IO_OUT0_WIDE) && !cnn_bias(regs, &v)) {
        goto out;
    }
    for (i = 0; i < n; i++) {
        v.v[i] = cnn_ss(v.v[i], R(CNN_SS_CONV));
    }
    if (bias && !(io & CNN_IO_OUT0_WIDE) && !cnn_bias(regs, &v)) {
        goto out;
    }
    if ((mode & CNN_M_BN) && !cnn_bn(regs, &v)) {
        goto out;
    }
    if ((mode & CNN_M_ELT) && !cnn_eltwise(regs, &v)) {
        goto out;
    }
    for (i = 0; i < n; i++) {
        v.v[i] = cnn_ss(v.v[i], R(CNN_SS_BN));
    }
stages_done:
    n = (size_t)v.c * v.h * v.w;
    if (mode & CNN_M_ACT) {
        for (i = 0; i < n; i++) {
            v.v[i] = MIN(MAX(v.v[i], INT16_MIN), INT16_MAX);
        }
    }
    if (io & CNN_IO_OUT1_EN) {
        /* act1, its scale, the pooling, its scale: to 8 or 16 bits */
        a1 = v;
        a1.v = g_new(int64_t, n);
        for (i = 0; i < n; i++) {
            a1.v[i] = cnn_ss(mode & CNN_M_ACT ? cnn_act(v.v[i], R(CNN_ACT1))
                                              : v.v[i], R(CNN_SS_ACT1));
        }
        if (mode & CNN_M_POOL) {
            cnn_pool(regs, &a1);
            for (i = 0; i < (size_t)a1.c * a1.h * a1.w; i++) {
                a1.v[i] = cnn_ss(a1.v[i], R(CNN_SS_POOL));
            }
        }
        cnn_write(wl, R(CNN_OUT1_ADDR), R(CNN_OUT1_LOFS) ? R(CNN_OUT1_LOFS) :
                  a1.w * (io & CNN_IO_OUT1_WIDE ? 2 : 1),
                  R(CNN_OUT1_COFS) ? R(CNN_OUT1_COFS) :
                  a1.w * a1.h * (io & CNN_IO_OUT1_WIDE ? 2 : 1),
                  io & (CNN_IO_OUT1_SIGNED | CNN_IO_POOL_SIGNED),
                  io & CNN_IO_OUT1_WIDE, &a1);
    }
    if (io & CNN_IO_OUT0_EN) {
        /*
         * act0 and its scale; 8-bit outputs drop the 8.8 datapath's
         * fraction bits, 16-bit ones keep them
         */
        for (i = 0; i < n; i++) {
            v.v[i] = cnn_shr(cnn_ss(mode & CNN_M_ACT ?
                                    cnn_act(v.v[i], R(CNN_ACT0)) : v.v[i],
                                    R(CNN_SS_OUT0)),
                             io & CNN_IO_OUT0_WIDE ? 0 : 8);
        }
        cnn_write(wl, R(CNN_OUT0_ADDR), R(CNN_OUT0_LOFS) ? R(CNN_OUT0_LOFS) :
                  v.w * (io & CNN_IO_OUT0_WIDE ? 2 : 1),
                  R(CNN_OUT0_COFS) ? R(CNN_OUT0_COFS) :
                  v.w * v.h * (io & CNN_IO_OUT0_WIDE ? 2 : 1),
                  io & CNN_IO_OUT0_SIGNED, io & CNN_IO_OUT0_WIDE, &v);
    }
out:
    g_free(in.v);
    g_free(v.v);
    g_free(a1.v);
}
