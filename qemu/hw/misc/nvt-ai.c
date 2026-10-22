/*
 * Novatek NA51089 AI engines: the work of a job.
 *
 * kdrv_ai (vendor/ai/drivers/k_driver/source/kdrv_ai; the SDK's rtos tree
 * has it as C source, the Linux one as objects) drives three engines,
 * named in the "ai" device-tree node's order: NUE (0xF0C60000), NUE2
 * (0xF0D50000) and CNN (0xF0CB0000).  nvt-media runs their jobs and
 * linked lists; when a job is set up it calls here with the engine's
 * registers.
 *
 * NUE, fully connected (SVM, kernel 1 in dot mode; kdrv_ai_tran_nue_fc_parm):
 *
 *   0x004 MODE0    [2:0] mode (0 SVM), bit 4 input signed, bit 12 input
 *                  16-bit
 *   0x008 MODE1    [1:0] kernel 1 (0 dot), [9:8] output path (1 / 3: the
 *                  kernel's result, s32), bit 20 FCD VLC
 *   0x00C / 0x014 / 0x02C   input vectors, weight rows, output
 *   0x040          weight row length [13:0], rows [28:16]
 *   0x048          input vectors [6:0]
 *   0x04C          gain [11:0], its shift [19:16]
 *   0x050          offset [20:0] (signed), per-product shift [27:24]
 *
 *   out[o][j] = ((sum_k ((x[o][k] * 256 * w[j][k]) >> fshift)) * gain
 *                >> gshift) + offset
 *
 * fshift is the SDK's weight shift, applied to each product before the
 * sum, as the field's place among the weight parameters has it.
 *
 * The rows are packed, a row a weight-row length apart; the outputs s32,
 * a vector's rows together.  8-bit data carries 8 fraction bits through
 * the datapath, which the x 256 stands for: vendor_ai_op asks for a total
 * shift of in + weight - out fraction bits + 8, and kdrv_ai_fc_done
 * shifts 8-bit results by 8 more than 16-bit ones.
 *
 * NUE2, preprocessing (nue2_reg.h, the 528 layout NA51089 uses):
 *
 *   0x004 FUNC     bit 0 YUV to RGB, 1 mean subtraction, 2 pad; [13:12]
 *                  input (0 YUV 4:2:0 semi-planar, 3 RGB planes); bit 14
 *                  output signed; bit 15 planar mean (else DC)
 *   0x008..0x010   inputs: Y / UV / planar mean (or R / G / B)
 *   0x018..0x020   outputs R / G / B; line offsets at 0x024.. / 0x030..
 *   0x044 / 0x058  input / output size (w [10:0], h [27:16])
 *   0x05C          the planar mean's size; 0x060 DC means (bytes 0..2),
 *                  the mean replicated [25:24] (1 / 2 / 4 / 8 x)
 *   0x07C          mean scale [27:12], shift [8:4], left (bit 0)
 *
 * The pipeline is YUV to RGB, scaling (bilinear, corners to corners: the
 * step kdrv programs is (in - 1) / (out - 1)), then the mean subtraction
 * (planar means are 3 bytes a pixel, R G B), the scale-shift and a clamp
 * to the output's range.
 *
 * Nothing in the SDK fixes NUE2's colour matrix or rounding, so YUV to
 * RGB is full-range BT.601, chroma replicated; the NUE result's rounding
 * is a floor.  Other NUE modes (results with bias / ReLU, RBF and the
 * other kernels, ROI pooling, softmax...) and NUE2's pad, HSV and rotate
 * log as unimplemented.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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

static void wr(hwaddr a, const void *buf, size_t n)
{
    if (dma_memory_write(&address_space_memory, a, buf, n,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ai: cannot write 0x%"
                      HWADDR_PRIx "\n", a);
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

#define NUE_MAX_SVW         9216
#define NUE_MAX_SVH         4096
#define NUE_MAX_OBJ         64
/*
 * The products a job may take, done at once on the vCPU: the drivers
 * split layers into jobs of 256 rows, far below this.
 */
#define NUE_MAX_MACS        (1u << 26)

static int64_t nue_elem(const uint8_t *p, size_t i, bool wide, bool sign)
{
    if (wide) {
        return sign ? (int16_t)lduw_le_p(p + 2 * i) : lduw_le_p(p + 2 * i);
    }
    return sign ? (int8_t)p[i] : p[i];
}

void nvt_ai_nue_run(const uint32_t *regs)
{
    uint32_t mode0 = R(NUE_MODE0), mode1 = R(NUE_MODE1);
    uint32_t svw = R(NUE_SIZE0) & 0x3FFF, svh = (R(NUE_SIZE0) >> 16) & 0x1FFF;
    uint32_t obj = R(NUE_SIZE2) & 0x7F, path = (mode1 >> 8) & 3;
    uint32_t gain = R(NUE_KERNEL0) & 0xFFF;
    unsigned gshift = (R(NUE_KERNEL0) >> 16) & 0xF;
    unsigned fshift = (R(NUE_KERNEL1) >> 24) & 0xF;
    int32_t offset = sextract32(R(NUE_KERNEL1), 0, 21);
    bool sign = mode0 & (1u << 4), wide = mode0 & (1u << 12);
    unsigned esz = wide ? 2 : 1;
    hwaddr in = R(NUE_IN_ADDR), sv = R(NUE_SV_ADDR) & ~3u;
    hwaddr out = R(NUE_OUT_ADDR);
    g_autofree uint8_t *x = NULL, *w = NULL;
    g_autofree int32_t *res = NULL;
    uint32_t o, j, k;

    if ((mode0 & 7) || (mode1 & 3) || (path != 1 && path != 3) ||
        (mode1 & (1u << 20))) {
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
    w = g_malloc((size_t)svw * esz);
    res = g_new(int32_t, (size_t)obj * svh);
    if (!rd(in, x, (size_t)obj * svw * esz)) {
        return;
    }
    for (j = 0; j < svh; j++) {
        /* weights are signed as the input is (one signedness bit) */
        if (!rd(sv + (hwaddr)j * svw * esz, w, (size_t)svw * esz)) {
            return;
        }
        for (o = 0; o < obj; o++) {
            const uint8_t *xo = x + (size_t)o * svw * esz;
            int64_t acc = 0;

            for (k = 0; k < svw; k++) {
                int64_t p = nue_elem(xo, k, wide, sign) *
                            nue_elem(w, k, wide, sign);

                acc += (wide ? p : p * 256) >> fshift;
            }
            res[(size_t)o * svh + j] = ((acc * gain) >> gshift) + offset;
        }
    }
    for (o = 0; o < (size_t)obj * svh; o++) {
        res[o] = cpu_to_le32(res[o]);
    }
    wr(out, res, (size_t)obj * svh * 4);
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

void nvt_ai_nue2_run(const uint32_t *regs)
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
                    v *= mscale;
                    v = ms & 1 ? v * ((int64_t)1 << mshift) : v >> mshift;
                }
                outl[x] = sign ? (uint8_t)MIN(MAX(v, -128), 127)
                               : clamp8(v);
            }
            wr(oa + (hwaddr)y * lofs, outl, ow);
        }
    }
}
