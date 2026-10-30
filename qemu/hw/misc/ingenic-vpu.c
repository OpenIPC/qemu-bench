/*
 * Ingenic T-series video encoders before the AVPU: the T10/T20 VPU
 * ("jz-vpu"), HELIX (T21, T23, T30: H.264 and JPEG) and RADIX (T30:
 * H.265), all driven by the vendor libimp through /dev/soc_vpu.
 * tmp/soc-vpu-contract.md has the evidence for what follows.
 *
 * The kernel side (drivers/video/soc_vpu/{soc_vpu.c, jz_nvpu/, helix/,
 * radix/} in the OpenIPC 3.10 trees) only arbitrates the engine, resets it
 * through CPM SRBC, starts one task and waits for its interrupt.  The task
 * is libimp's: a VDMA "ACFG" chain in memory, pairs of words {value,
 * VLD (bit 31) | TERM (bit 30) | register offset [19:2]}, that the engine
 * writes into its own registers up to the TERM entry.  On HELIX/VPU the
 * chain programs every unit -- EFE (front end, 0x40000), MCE (0x50000),
 * DBLK (0x70000), VMAU (0x80000), SDE (0x90000) with its CABAC context
 * table (0x92000, initialised by software), JPGC (0xE0000) -- and ends
 * with EFE_CTRL with RUN, which starts an H.264 slice or (EFE_CTRL[15:14]
 * = 1) a JPEG picture.  HELIX (T21/T23) takes the chain's address at 0x84,
 * the VPU and T30's HELIX at VDMA 0x10008, RADIX at CFGC 0x08; RADIX's
 * units sit at 0x8000 apart (EFE 0x20000, SDE 0x68000, ...) and its chain
 * ends with VDMA 0x8008.
 *
 * What the engine produces and libimp keeps:
 *   H.264: CABAC slice_data(), end_of_slice and the arithmetic coder's
 *     flush with the stop bit, unescaped, at SDE_CFG4: libimp writes the
 *     SPS, PPS and slice header (after the engine has run), memcpy()s
 *     SDE_CFG9 bytes behind its cabac_alignment_one_bits and escapes the
 *     NAL (i264e_slice_write).  libimp's x264 port refuses CAVLC.  The
 *     slice: EFE_GEOM [31:24]/[15:8] first/last MB row, [7:0] width in MBs
 *     - 1; EFE_RAWY/RAWC, EFE_RAW_STRD (Y << 16 | C) an NV12 (or NV21)
 *     source; EFE_CTRL [13:8] QP, bit 4 P slice.
 *   H.265 (RADIX): slice_segment_data() with its trailing bits, unescaped,
 *     at SDE 0x68008; libimp writes the VPS/SPS/PPS and the header and
 *     escapes the substream (I265e::sliceEncap).  EFE 0x20004 h << 16 | w,
 *     0x2000C/0x20010 the source, 0x20014 strides, 0x20040 [15:8] QP,
 *     0x20000 bit 4 P slice.  The coding follows libimp's fixed choices:
 *     CTB 64, cu_qp_delta per 16x16, chroma QP offset -3, SAO luma only.
 *   JPEG: the entropy-coded scan, byte-stuffed, at JPGC_BSA; libimp writes
 *     SOI..SOS and EOI around it.  The quantisers are QMEM's (luma then
 *     chroma, zigzag, as reciprocals), the source the EFE's registers.
 * Coding: I slices are I_16x16 DC macroblocks; H.264 P slices send each
 * macroblock whose source changed since the last picture of that size as
 * I_PCM and skip the rest (hw/misc/hantro-h264.c), so the stream decodes
 * to the source at 4x4-block resolution and is exact where it moves.
 * H.265: 8x8 DC intra CUs, skipped where the reference still matches
 * (hw/misc/hevc-enc.c).
 *
 * Resets: CPM SRBC bit 31 (HELIX/VPU) or 28 (RADIX), which the kernel
 * pulses before every task, clears the status -- the kernel enables the
 * interrupt before it starts the task, so a status left over would
 * complete the next task at once.
 *
 * Completion: a virtual millisecond after the start, SCH_STAT bit 0
 * (ENDFLAG; bit 4 too for JPEG), the interrupt while SCH_GLBC bit 16 is
 * set; SDE_CFG9 / JPGC_STAT[23:0] hold the byte counts that the kernel
 * hands back as channel_node.output_len, EFE_SSAD a cost.  RADIX:
 * CFGC_STAT bit 8 (and 2), the interrupt while CFGC GLB_CTRL bit 24 is
 * set, the byte count at SDE 0x68024.  libimp reads some 40 statistics
 * registers after each H.264 task (EFE 0x40094.., MCE 0x500E8.., VMAU
 * 0x80034..0x80168) for its rate control: they read what the chain left,
 * which is enough for its CBR to settle.
 *
 * "-global ingenic-vpu.debug=on" logs every chain, register access from
 * the CPU and the job each chain started.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
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
#include "jpeg-enc.h"
#include "hevc-enc.h"

#define TYPE_INGENIC_VPU "ingenic-vpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicVpuState, INGENIC_VPU)

#define VPU_SIZE            0x100000
#define VPU_NREGS           (VPU_SIZE / 4)
#define VPU_JOB_NS          1000000
#define VPU_CHAIN_MAX       65536
#define VPU_MAX_DIM         4096
#define VPU_NREFS           4

/* the engine kinds */
enum {
    VPU_KIND_HELIX = 0,         /* T20 VPU, HELIX: the same register map */
    VPU_KIND_RADIX = 1,
};

/* VDMA chain words */
#define ACFG_VLD            (1u << 31)
#define ACFG_TERM           (1u << 30)
#define ACFG_IDX(c)         ((c) & 0xFFFFC)

/* HELIX / VPU */
#define R_SCH_GLBC          0x00000
#define  GLBC_INTE_ENDF     (1u << 16)
#define R_SCH_STAT          0x00034
#define  STAT_ENDF          (1u << 0)
#define  STAT_JPGEND        (1u << 4)
#define R_VDMA_TASKRG       0x10008
#define R_VDMA_TASKST       0x1000C
#define  TASKST_ACFG_END    (1u << 2)
#define  TASKST_BUSY        (1u << 0)
#define R_EFE_CTRL          0x40000
#define  EFE_RUN            (1u << 0)
#define  EFE_ID(v)          extract32(v, 14, 2)     /* 0 H.264, 1 JPEG */
#define  EFE_QP(v)          extract32(v, 8, 6)
#define  EFE_P_SLICE        (1u << 4)
#define  EFE_PLANE(v)       extract32(v, 2, 2)      /* 0 tile, 2 NV12 */
#define R_EFE_GEOM          0x40004
#define R_EFE_RAWY          0x40010
#define R_EFE_RAWC          0x40014
#define R_EFE_RAW_STRD      0x40038
#define R_EFE_SSAD          0x40108
#define R_EFE_CQP_OFST      0x40120
#define R_DBLK_GSTA         0x70070
#define R_SDE_STAT          0x90000
#define  SDE_STAT_BSEND     (1u << 1)
#define R_SDE_BS_ADDR       0x90024         /* SDE_CFG4 */
#define R_SDE_BS_SIZE       0x90028         /* SDE_CFG5 */
#define R_SDE_BSLEN         0x90038         /* SDE_CFG9 */
#define R_JPGC_TRIG         0xE0000
#define R_JPGC_STAT         0xE0008
#define  JPGC_STAT_ENDF     (1u << 31)
#define R_JPGC_BSA          0xE000C
#define R_JPGC_QMEM         0xE1400

/* RADIX */
#define R_RX_GLB_CTRL       0x00000
#define  RX_INTE_ENDF       (1u << 24)
#define R_RX_ACM_CTRL       0x00008
#define R_RX_STAT           0x00040
#define  RX_STAT_GLB_END    (1u << 2)
#define  RX_INTST_END       (1u << 8)
#define R_RX_EFE_CTRL       0x20000         /* bit 4: P slice */
#define R_RX_EFE_SIZE       0x20004         /* [31:16] height, [15:0] width */
#define R_RX_EFE_RAWY       0x2000C
#define R_RX_EFE_RAWC       0x20010
#define R_RX_EFE_STRD       0x20014         /* [31:16] Y, [15:0] C */
#define R_RX_EFE_QP         0x20040         /* [15:8] slice QP */
#define R_RX_SDE_BS_ADDR    0x68008
#define R_RX_SDE_SLBL       0x68024

typedef struct VpuRef {
    int w, h;
    uint8_t *y, *uv;            /* the last source of this size, NV12 */
    uint64_t used;
} VpuRef;

struct IngenicVpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t kind;
    uint32_t taskrg;            /* where the chain address goes */
    bool debug;

    uint32_t regs[VPU_NREGS];
    uint32_t chain;             /* the chain the next job runs */
    bool busy;
    /* what the chain started */
    bool efe_run, jpgc_run;
    uint64_t jobs;

    VpuRef refs[VPU_NREFS];
    /* RADIX: the H.265 coder's parameters and decoded pictures */
    HevcEncParams hevc_prm;
    HevcEncRef *hevc_ref, *hevc_recon;
};

static uint32_t vpu_reg(IngenicVpuState *s, uint32_t off)
{
    return s->regs[(off & (VPU_SIZE - 1)) >> 2];
}

static void vpu_set(IngenicVpuState *s, uint32_t off, uint32_t v)
{
    s->regs[(off & (VPU_SIZE - 1)) >> 2] = v;
}

static void vpu_update_irq(IngenicVpuState *s)
{
    bool level;

    if (s->kind == VPU_KIND_RADIX) {
        level = (vpu_reg(s, R_RX_STAT) & RX_INTST_END) &&
                (vpu_reg(s, R_RX_GLB_CTRL) & RX_INTE_ENDF);
    } else {
        level = (vpu_reg(s, R_SCH_STAT) & STAT_ENDF) &&
                (vpu_reg(s, R_SCH_GLBC) & GLBC_INTE_ENDF);
    }
    qemu_set_irq(s->irq, level);
}

/* ---- memory -------------------------------------------------------------- */

static void vpu_read_mem(hwaddr a, void *buf, size_t n)
{
    dma_memory_read(&address_space_memory, a, buf, n, MEMTXATTRS_UNSPECIFIED);
}

static void vpu_write_mem(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * The picture's planes, packed: Y at w, CbCr (interleaved, NV12) at w.
 * NV21 is turned into NV12.
 */
static void vpu_fetch_nv12(hwaddr ya, hwaddr ca, int ys, int cs, int w,
                           int h, bool nv21, uint8_t *y, uint8_t *uv)
{
    int r, x;

    for (r = 0; r < h; r++) {
        vpu_read_mem(ya + (hwaddr)r * ys, y + r * w, w);
    }
    for (r = 0; r < h / 2; r++) {
        uint8_t *p = uv + r * w;

        vpu_read_mem(ca + (hwaddr)r * cs, p, w);
        if (nv21) {
            for (x = 0; x < w; x += 2) {
                uint8_t t = p[x];
                p[x] = p[x + 1];
                p[x + 1] = t;
            }
        }
    }
}

static VpuRef *vpu_ref(IngenicVpuState *s, int w, int h, bool *fresh)
{
    VpuRef *r, *lru = &s->refs[0];
    int i;

    for (i = 0; i < VPU_NREFS; i++) {
        r = &s->refs[i];
        if (r->y && r->w == w && r->h == h) {
            r->used = ++s->jobs;
            *fresh = false;
            return r;
        }
        if (r->used < lru->used) {
            lru = r;
        }
    }
    g_free(lru->y);
    g_free(lru->uv);
    lru->w = w;
    lru->h = h;
    lru->y = g_malloc0((size_t)w * h);
    lru->uv = g_malloc0((size_t)w * h / 2);
    lru->used = ++s->jobs;
    *fresh = true;
    return lru;
}

/* ---- H.264 (EFE + SDE) --------------------------------------------------- */

static void vpu_h264(IngenicVpuState *s)
{
    const uint32_t ctrl = vpu_reg(s, R_EFE_CTRL);
    const uint32_t geom = vpu_reg(s, R_EFE_GEOM);
    const uint32_t strd = vpu_reg(s, R_EFE_RAW_STRD);
    const int mbw = extract32(geom, 0, 8) + 1;
    const int fst = extract32(geom, 24, 8), lst = extract32(geom, 8, 8);
    const int mbh = lst >= fst ? lst - fst + 1 : 1;
    const int ys = extract32(strd, 16, 16), cs = extract32(strd, 0, 16);
    const int qp = EFE_QP(ctrl);
    const bool p_slice = ctrl & EFE_P_SLICE;
    const int w = mbw * 16, h = mbh * 16;
    const hwaddr bs = vpu_reg(s, R_SDE_BS_ADDR);
    uint32_t cap = vpu_reg(s, R_SDE_BS_SIZE);
    int cqp = sextract32(vpu_reg(s, R_EFE_CQP_OFST), 0, 5);
    g_autofree uint8_t *y = NULL, *uv = NULL, *out = NULL;
    VpuRef *ref;
    bool fresh;
    size_t n;

    if (EFE_PLANE(ctrl) != 2 && EFE_PLANE(ctrl) != 3) {
        qemu_log_mask(LOG_UNIMP, "ingenic-vpu: source format %u\n",
                      EFE_PLANE(ctrl));
    }
    if (w > VPU_MAX_DIM || h > VPU_MAX_DIM || !ys || !cs) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: bad slice %dx%d "
                      "strides %d/%d\n", w, h, ys, cs);
        vpu_set(s, R_SDE_BSLEN, 0);
        return;
    }
    if (!cap || cap > 16 * MiB) {
        cap = 4 * MiB;              /* T20's chains leave SDE_CFG5 alone */
    }
    y = g_malloc((size_t)w * h);
    uv = g_malloc((size_t)w * h / 2);
    vpu_fetch_nv12(vpu_reg(s, R_EFE_RAWY) + (hwaddr)fst * 16 * ys,
                   vpu_reg(s, R_EFE_RAWC) + (hwaddr)fst * 8 * cs,
                   ys, cs, w, h, EFE_PLANE(ctrl) == 3, y, uv);
    out = g_malloc(cap);
    ref = vpu_ref(s, w, h, &fresh);
    if (p_slice && !fresh) {
        n = h264_slice_ppcm_cabac(qp, 0, mbw, mbh, y, w, uv, w,
                                  ref->y, ref->uv, out, cap);
    } else {
        n = h264_slice_i16dc_cabac(qp, cqp, mbw, mbh, y, w, uv, w, out,
                                   cap);
        memcpy(ref->y, y, (size_t)w * h);
        memcpy(ref->uv, uv, (size_t)w * h / 2);
    }
    if (n > cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: bitstream buffer full "
                      "(%zu > %u)\n", n, cap);
        n = cap;
    }
    vpu_write_mem(bs, out, n);
    vpu_set(s, R_SDE_BSLEN, n);
    /* a cost for the rate control: the source's activity, never 0 */
    vpu_set(s, R_EFE_SSAD, mbw * mbh * 256 + 1);
    vpu_set(s, R_SDE_STAT, vpu_reg(s, R_SDE_STAT) | SDE_STAT_BSEND);
    vpu_set(s, R_DBLK_GSTA, vpu_reg(s, R_DBLK_GSTA) | 1);
    if (s->debug) {
        qemu_log("ingenic-vpu: H.264 %c slice %dx%d rows %d-%d qp %d "
                 "src %08x/%08x stride %d/%d -> %08" HWADDR_PRIx
                 " %zu bytes\n", p_slice ? 'P' : 'I', w, h, fst, lst, qp,
                 vpu_reg(s, R_EFE_RAWY), vpu_reg(s, R_EFE_RAWC), ys, cs,
                 bs, n);
    }
}

/* ---- JPEG (JPGC) --------------------------------------------------------- */

static const uint8_t vpu_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * QMEM holds one word per coefficient, the luma table (0..63) then the
 * chroma one (64..127), both in zigzag order, as reciprocals: [13:11] a
 * shift s, [10:0] r = 2^(11+s) / q (libimp's T20_JPEGE_SliceInit copies
 * them from its qmem table).  Turn them back into quantisers, row-major.
 */
static void vpu_jpeg_qtab(IngenicVpuState *s, uint8_t ql[64], uint8_t qc[64])
{
    int i, t;

    for (t = 0; t < 2; t++) {
        uint8_t *q = t ? qc : ql;

        for (i = 0; i < 64; i++) {
            uint32_t v = vpu_reg(s, R_JPGC_QMEM + 4 * (64 * t + i));
            uint32_t r = extract32(v, 0, 11), sh = extract32(v, 11, 3);

            q[vpu_zigzag[i]] = r ? MIN(MAX(((2048u << sh) + r / 2) / r, 1),
                                       255) : 1;
        }
    }
}

static void vpu_jpeg(IngenicVpuState *s)
{
    const uint32_t geom = vpu_reg(s, R_EFE_GEOM);
    const uint32_t strd = vpu_reg(s, R_EFE_RAW_STRD);
    const int w = (extract32(geom, 0, 16) + 1) * 16;
    const int h = (extract32(geom, 16, 16) + 1) * 16;
    const int ys = extract32(strd, 16, 16), cs = extract32(strd, 0, 16);
    const hwaddr bs = vpu_reg(s, R_JPGC_BSA);
    const size_t cap = 4 * MiB;
    g_autofree uint8_t *y = NULL, *uv = NULL, *cb = NULL, *cr = NULL;
    g_autofree uint8_t *out = NULL;
    uint8_t ql[64], qc[64];
    bool overflow = false;
    size_t n;
    int i;

    if (w > VPU_MAX_DIM || h > VPU_MAX_DIM || !ys || !cs) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: bad JPEG %dx%d\n", w, h);
        vpu_set(s, R_JPGC_STAT, JPGC_STAT_ENDF);
        return;
    }
    y = g_malloc((size_t)w * h);
    uv = g_malloc((size_t)w * h / 2);
    cb = g_malloc((size_t)w * h / 4);
    cr = g_malloc((size_t)w * h / 4);
    vpu_fetch_nv12(vpu_reg(s, R_EFE_RAWY), vpu_reg(s, R_EFE_RAWC), ys, cs,
                   w, h, EFE_PLANE(vpu_reg(s, R_EFE_CTRL)) == 3, y, uv);
    for (i = 0; i < w * h / 4; i++) {
        cb[i] = uv[2 * i];
        cr[i] = uv[2 * i + 1];
    }
    vpu_jpeg_qtab(s, ql, qc);
    out = g_malloc(cap);
    n = jpeg_enc_scan(y, w, cb, cr, w / 2, w, h, ql, qc, false, out, cap,
                      &overflow);
    vpu_write_mem(bs, out, n);
    vpu_set(s, R_JPGC_STAT, JPGC_STAT_ENDF | (n & 0xFFFFFF));
    if (s->debug) {
        qemu_log("ingenic-vpu: JPEG %dx%d q0 %d/%d -> %08" HWADDR_PRIx
                 " %zu bytes\n", w, h, ql[0], qc[0], bs, n);
    }
}

/* ---- H.265 (RADIX) ------------------------------------------------------- */

/* hevc-enc escapes its slice data; RADIX leaves that to libimp */
static size_t vpu_unescape(uint8_t *p, size_t n)
{
    size_t i, o = 0;
    int zeros = 0;

    for (i = 0; i < n; i++) {
        if (zeros >= 2 && p[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = p[i] ? 0 : zeros + 1;
        p[o++] = p[i];
    }
    return o;
}

static void vpu_h265(IngenicVpuState *s)
{
    const uint32_t size = vpu_reg(s, R_RX_EFE_SIZE);
    const uint32_t strd = vpu_reg(s, R_RX_EFE_STRD);
    const int w = extract32(size, 0, 16), h = extract32(size, 16, 16);
    const int ys = extract32(strd, 16, 16), cs = extract32(strd, 0, 16);
    const int qp = extract32(vpu_reg(s, R_RX_EFE_QP), 8, 6);
    const bool p_slice = vpu_reg(s, R_RX_EFE_CTRL) & EFE_P_SLICE;
    const hwaddr bs = vpu_reg(s, R_RX_SDE_BS_ADDR);
    const int ew = ROUND_UP(w, 8), eh = ROUND_UP(h, 8);
    const size_t cap = 4 * MiB;
    g_autofree uint8_t *y = NULL, *uv = NULL, *out = NULL;
    size_t n;

    if (!w || !h || w > VPU_MAX_DIM || h > VPU_MAX_DIM || ys < w || cs < w) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: bad H.265 picture "
                      "%dx%d strides %d/%d\n", w, h, ys, cs);
        vpu_set(s, R_RX_SDE_SLBL, 0);
        return;
    }
    if (!s->hevc_ref) {
        s->hevc_ref = hevc_enc_ref_new();
        s->hevc_recon = hevc_enc_ref_new();
    }
    y = g_malloc((size_t)w * h);
    uv = g_malloc((size_t)w * h / 2);
    vpu_fetch_nv12(vpu_reg(s, R_RX_EFE_RAWY), vpu_reg(s, R_RX_EFE_RAWC), ys,
                   cs, w, h, false, y, uv);
    out = g_malloc(cap);
    n = hevc_enc_slice_data(&s->hevc_prm, ew, eh, qp, p_slice,
                            p_slice ? s->hevc_ref : NULL, s->hevc_recon,
                            y, w, h, w, uv, w, out, cap);
    if (n > cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: H.265 slice too big\n");
        n = cap;
    }
    n = vpu_unescape(out, n);
    /* the decoded picture is the next picture's reference */
    {
        HevcEncRef *t = s->hevc_ref;

        s->hevc_ref = s->hevc_recon;
        s->hevc_recon = t;
    }
    vpu_write_mem(bs, out, n);
    vpu_set(s, R_RX_SDE_SLBL, n);
    if (s->debug) {
        qemu_log("ingenic-vpu: H.265 %c slice %dx%d qp %d src %08x/%08x "
                 "stride %d/%d -> %08" HWADDR_PRIx " %zu bytes\n",
                 p_slice ? 'P' : 'I', w, h, qp, vpu_reg(s, R_RX_EFE_RAWY),
                 vpu_reg(s, R_RX_EFE_RAWC), ys, cs, bs, n);
    }
}

/* ---- the task ------------------------------------------------------------ */

static void vpu_reg_write(IngenicVpuState *s, uint32_t off, uint32_t v,
                          bool from_chain);

static void vpu_run_chain(IngenicVpuState *s, uint32_t addr)
{
    uint32_t w[2];
    int i;

    s->efe_run = s->jpgc_run = false;
    for (i = 0; i < VPU_CHAIN_MAX; i++) {
        vpu_read_mem(addr + 8 * i, w, 8);
        w[0] = le32_to_cpu(w[0]);
        w[1] = le32_to_cpu(w[1]);
        if (s->debug) {
            qemu_log("ingenic-vpu: chain %08x [%4d] %05x <- %08x%s\n", addr,
                     i, ACFG_IDX(w[1]), w[0],
                     (w[1] & ACFG_TERM) ? " TERM" : "");
        }
        if (w[1] & ACFG_VLD) {
            vpu_reg_write(s, ACFG_IDX(w[1]), w[0], true);
        }
        if (w[1] & ACFG_TERM) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: chain at %08x has no end\n",
                  addr);
}

static void vpu_done(void *opaque)
{
    IngenicVpuState *s = opaque;

    vpu_run_chain(s, s->chain);
    if (s->kind == VPU_KIND_RADIX) {
        vpu_h265(s);
        vpu_set(s, R_RX_STAT, vpu_reg(s, R_RX_STAT) | RX_INTST_END |
                RX_STAT_GLB_END);
    } else {
        uint32_t st = STAT_ENDF;

        if (s->efe_run && EFE_ID(vpu_reg(s, R_EFE_CTRL)) == 1) {
            vpu_jpeg(s);
            st |= STAT_JPGEND;
        } else if (s->jpgc_run && !s->efe_run) {
            /* libimp always feeds the JPGC through the EFE */
            qemu_log_mask(LOG_UNIMP, "ingenic-vpu: JPGC fed by its own "
                          "DMA\n");
            vpu_set(s, R_JPGC_STAT, JPGC_STAT_ENDF);
            st |= STAT_JPGEND;
        } else if (s->efe_run) {
            vpu_h264(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: chain started "
                          "nothing\n");
        }
        vpu_set(s, R_VDMA_TASKST, TASKST_ACFG_END);
        vpu_set(s, R_SCH_STAT, vpu_reg(s, R_SCH_STAT) | st);
    }
    s->busy = false;
    vpu_update_irq(s);
}

static void vpu_start(IngenicVpuState *s, uint32_t addr)
{
    if (s->busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: start while busy\n");
        return;
    }
    s->chain = addr;
    s->busy = true;
    /* the kernel resets the engine (CPM SRBC) before every task */
    if (s->kind == VPU_KIND_RADIX) {
        vpu_set(s, R_RX_STAT, 0);
    } else {
        vpu_set(s, R_SCH_STAT, 0);
        vpu_set(s, R_VDMA_TASKST, TASKST_BUSY);
        vpu_set(s, R_SDE_BSLEN, 0);
        vpu_set(s, R_JPGC_STAT, 0);
    }
    vpu_update_irq(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VPU_JOB_NS);
}

static void vpu_reg_write(IngenicVpuState *s, uint32_t off, uint32_t v,
                          bool from_chain)
{
    vpu_set(s, off, v);
    if (s->kind == VPU_KIND_RADIX) {
        if (!from_chain && off == R_RX_ACM_CTRL && (v & 1)) {
            vpu_start(s, v & ~0xFFu);
        }
        return;
    }
    if (!from_chain && off == s->taskrg && (v & 1)) {
        vpu_start(s, v & ~0x7Fu);
        return;
    }
    if (from_chain && off == R_EFE_CTRL && (v & EFE_RUN)) {
        s->efe_run = true;
    }
    if (from_chain && off == R_JPGC_TRIG && (v & 1)) {
        s->jpgc_run = true;
    }
}

/* ---- registers ----------------------------------------------------------- */

static uint64_t vpu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicVpuState *s = opaque;

    if (s->debug) {
        qemu_log("ingenic-vpu: read  %05" HWADDR_PRIx " = %08x\n", off,
                 vpu_reg(s, off));
    }
    return vpu_reg(s, off);
}

static void vpu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    IngenicVpuState *s = opaque;

    if (s->debug) {
        qemu_log("ingenic-vpu: write %05" HWADDR_PRIx " = %08" PRIx64 "\n",
                 off, val);
    }
    vpu_reg_write(s, off, val, false);
    if (off == R_SCH_GLBC || off == R_SCH_STAT ||
        (s->kind == VPU_KIND_RADIX && off == R_RX_STAT)) {
        vpu_update_irq(s);
    }
}

static const MemoryRegionOps vpu_ops = {
    .read = vpu_read,
    .write = vpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* CPM SRBC's soft-reset bit for this engine: stop and clear the status */
static void vpu_srbc_reset(void *opaque, int n, int level)
{
    IngenicVpuState *s = opaque;

    if (!level) {
        return;
    }
    timer_del(s->timer);
    s->busy = false;
    if (s->kind == VPU_KIND_RADIX) {
        vpu_set(s, R_RX_STAT, 0);
    } else {
        vpu_set(s, R_SCH_STAT, 0);
        vpu_set(s, R_VDMA_TASKST, 0);
        vpu_set(s, R_SDE_STAT, 0);
        vpu_set(s, R_DBLK_GSTA, 0);
        vpu_set(s, R_JPGC_STAT, 0);
    }
    vpu_update_irq(s);
}

static void vpu_reset(DeviceState *dev)
{
    IngenicVpuState *s = INGENIC_VPU(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->busy = false;
    vpu_update_irq(s);
}

static void vpu_realize(DeviceState *dev, Error **errp)
{
    IngenicVpuState *s = INGENIC_VPU(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->kind > VPU_KIND_RADIX) {
        error_setg(errp, "ingenic-vpu: kind is 0 (VPU/HELIX) or 1 (RADIX)");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &vpu_ops, s,
                          TYPE_INGENIC_VPU, VPU_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, vpu_srbc_reset, "reset", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vpu_done, s);
    /*
     * What T30's libimp (i265e) puts in its SPS/PPS/slice headers: CTB 64
     * (maxCUSize), cu_qp_delta per 16x16 (rc.qgSize), chroma QP offsets
     * -3 (cbQpOffset/crQpOffset), SAO for luma alone, deblocking at the
     * default offsets, VUI.
     */
    s->hevc_prm = (HevcEncParams){
        .log2_ctb = 6, .log2_qg = 4, .sao = true, .sao_luma_only = true,
        .chroma_qp_offset = -3, .beta_offset_div2 = 0, .tc_offset_div2 = 0,
        .vui = true,
    };
}

static void vpu_finalize(Object *obj)
{
    IngenicVpuState *s = INGENIC_VPU(obj);
    int i;

    for (i = 0; i < VPU_NREFS; i++) {
        g_free(s->refs[i].y);
        g_free(s->refs[i].uv);
    }
    if (s->hevc_ref) {
        hevc_enc_ref_free(s->hevc_ref);
        hevc_enc_ref_free(s->hevc_recon);
    }
}

static const Property vpu_props[] = {
    DEFINE_PROP_UINT32("kind", IngenicVpuState, kind, VPU_KIND_HELIX),
    DEFINE_PROP_UINT32("taskrg", IngenicVpuState, taskrg, R_VDMA_TASKRG),
    DEFINE_PROP_BOOL("debug", IngenicVpuState, debug, false),
};

static const VMStateDescription vmstate_vpu = {
    .name = TYPE_INGENIC_VPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, IngenicVpuState),
        VMSTATE_UINT32_ARRAY(regs, IngenicVpuState, VPU_NREGS),
        VMSTATE_UINT32(chain, IngenicVpuState),
        VMSTATE_BOOL(busy, IngenicVpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void vpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = vpu_realize;
    device_class_set_legacy_reset(dc, vpu_reset);
    device_class_set_props(dc, vpu_props);
    dc->vmsd = &vmstate_vpu;
}

static const TypeInfo vpu_info = {
    .name           = TYPE_INGENIC_VPU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(IngenicVpuState),
    .instance_finalize = vpu_finalize,
    .class_init     = vpu_class_init,
};

static void vpu_register_types(void)
{
    type_register_static(&vpu_info);
}

type_init(vpu_register_types)
