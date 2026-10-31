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
 *   JPEG decoding (T23, T30: libimp's IMP_Decoder): a chain that starts
 *     the JPGC alone (JPGC_TRIG = 7) decodes a baseline scan; see
 *     vpu_jpeg_decode() for the contract.
 * Coding (hw/misc/hantro-h264.c): I slices are I_16x16 DC macroblocks;
 * H.264 P slices are inter coded against the previous picture of that
 * size -- P_Skip, P_L0_16x16 with a full-pel motion search and the 4x4
 * transform's residual, I_16x16 DC where nothing matches -- the reference
 * being the decoder's picture, deblocked as libimp's slice header asks
 * (on, offsets 0), so nothing drifts.  libimp's x264 port writes a High
 * profile PPS with transform_8x8_mode_flag set whatever profile the
 * application asks for (T21), so inter macroblocks with luma residual
 * send transform_size_8x8_flag = 0 ("transform-8x8" turns that off).  A
 * picture that would not fit SDE_CFG5 is coded again at a higher QP
 * (mb_qp_delta).  H.265: 8x8 DC intra CUs, skipped where the reference
 * still matches (hw/misc/hevc-enc.c).
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
#include <math.h>
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

/*
 * T41's JPEG engine is HELIX's JPEG half with its units renumbered: SCH 0,
 * VDMA 1, EFE 2, JPGC 3 at 64 KiB each (the 4.4 kernel's helix.h, HID_*),
 * the rest (AUX, SDE, ...) folded on 7; its chains set bit 19 of every
 * address.  Registers are kept at HELIX's offsets (EFE 0x40000, JPGC
 * 0xE0000).
 */
#define VPU_LAYOUT_HELIX    0
#define VPU_LAYOUT_T41_JPEG 1

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
#define R_JPGC_GLBI         0xE0004
#define  GLBI_RSTER         (1u << 2)
#define  GLBI_NCOMP(v)      (extract32(v, 4, 2) + 1)
#define R_JPGC_BSA          0xE000C
#define R_JPGC_P0A          0xE0010         /* decoded luma, 16x16 tiles */
#define R_JPGC_P1A          0xE0014         /* decoded chroma, 16x8 tiles */
#define R_JPGC_NMCU         0xE0028
#define R_JPGC_NRSM         0xE002C
#define R_JPGC_P0C          0xE0030
#define R_JPGC_MCUS         0xE0064         /* MCUs decoded */
#define R_JPGC_HUFB         0xE1200
#define R_JPGC_HUFM         0xE1300
#define R_JPGC_QMEM         0xE1400
#define R_JPGC_HUFS         0xE1800
#define JPGC_HUFS_WORDS     336

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
    uint32_t layout;            /* VPU_LAYOUT_*: the units' addresses */
    bool debug;
    bool t8x8;                  /* libimp's PPS sets transform_8x8_mode */

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

static uint32_t vpu_map(IngenicVpuState *s, uint32_t off)
{
    static const uint8_t t41_unit[8] = { 0x0, 0x1, 0x4, 0xE, 0xF, 0xF, 0xF,
                                         0xF };

    if (s->layout != VPU_LAYOUT_T41_JPEG) {
        return off;
    }
    off &= 0x7FFFF;
    return (t41_unit[off >> 16] << 16) | (off & 0xFFFF);
}

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

/*
 * The engines' TLB: with a page table base set and a unit's enable on,
 * that unit's addresses are virtual, translated through the kernel's
 * DMMU table (libdmmu.c): a 4 KiB page directory indexed by bits 31:22,
 * each entry the physical address of a page table with bit 0 valid; a
 * page table of 1024 entries indexed by bits 21:12, each the physical
 * page with bit 0 valid (bits 11:1 the kernel's map count).  HELIX / VPU:
 * the base at SCH_TLBA (0x30), the enables in GLBC -- per unit bits 26..21
 * (JPGC, DBLK, SDE, EFE, VDMA, MCE) or bit 30 for all; RADIX: CFGC
 * TLB_TBA (0x10) and TLBE (0x1C) bits 16..21 (EFE, VDMA, ODMA, DBLK,
 * MCE, SDE).  An unmapped page is a TLB error: logged, its bytes read as
 * zero and writes dropped, and on HELIX / VPU SCH_STAT bit 10 set.
 */
enum { VPU_U_EFE, VPU_U_SDE, VPU_U_JPGC, VPU_U_VDMA };

#define R_SCH_TLBA          0x00030
#define R_RX_TLB_TBA        0x00010
#define R_RX_TLBE           0x0001C
#define  STAT_TLBERR        (1u << 10)

static bool vpu_tlb_on(IngenicVpuState *s, int unit)
{
    static const uint8_t helix[] = { 23, 24, 26, 22 };
    static const uint8_t radix[] = { 16, 21, 0, 17 };
    uint32_t g;

    if (s->kind == VPU_KIND_RADIX) {
        return radix[unit] && vpu_reg(s, R_RX_TLB_TBA) &&
               (vpu_reg(s, R_RX_TLBE) & (1u << radix[unit]));
    }
    g = vpu_reg(s, R_SCH_GLBC);
    return vpu_reg(s, R_SCH_TLBA) &&
           (g & ((1u << helix[unit]) | (1u << 30)));
}

static bool vpu_xlate(IngenicVpuState *s, uint32_t va, hwaddr *pa)
{
    uint32_t base = vpu_reg(s, s->kind == VPU_KIND_RADIX ? R_RX_TLB_TBA :
                                                            R_SCH_TLBA);
    uint32_t pgd, pte;

    pgd = ldl_le_phys(&address_space_memory, (base & ~0xfffu) + 4 * (va >> 22));
    if (pgd & 1) {
        pte = ldl_le_phys(&address_space_memory, (pgd & ~0xfffu) +
                          4 * ((va >> 12) & 0x3ff));
        if (pte & 1) {
            *pa = (pte & ~0xfffu) | (va & 0xfff);
            return true;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: TLB error at 0x%08x\n", va);
    if (s->kind != VPU_KIND_RADIX) {
        vpu_set(s, R_SCH_STAT, vpu_reg(s, R_SCH_STAT) | STAT_TLBERR);
    }
    return false;
}

/* a unit's access, a page at a time when its TLB is on */
static void vpu_mem(IngenicVpuState *s, int unit, uint32_t a, void *buf,
                    size_t n, bool write)
{
    uint8_t *p = buf;

    while (n) {
        size_t k = n;
        hwaddr pa = a;
        bool ok = true;

        if (vpu_tlb_on(s, unit)) {
            k = MIN(n, 0x1000 - (a & 0xfff));
            ok = vpu_xlate(s, a, &pa);
        }
        if (write && ok) {
            dma_memory_write(&address_space_memory, pa, p, k,
                             MEMTXATTRS_UNSPECIFIED);
        } else if (!write) {
            if (ok) {
                dma_memory_read(&address_space_memory, pa, p, k,
                                MEMTXATTRS_UNSPECIFIED);
            } else {
                memset(p, 0, k);
            }
        }
        a += k;
        p += k;
        n -= k;
    }
}

static void vpu_read_mem(IngenicVpuState *s, int unit, uint32_t a, void *buf,
                         size_t n)
{
    vpu_mem(s, unit, a, buf, n, false);
}

static void vpu_write_mem(IngenicVpuState *s, int unit, uint32_t a,
                          const void *buf, size_t n)
{
    vpu_mem(s, unit, a, (void *)buf, n, true);
}

/*
 * The picture's planes, packed: Y at w, CbCr (interleaved, NV12) at w.
 * NV21 is turned into NV12.
 */
static void vpu_fetch_nv12(IngenicVpuState *s, uint32_t ya, uint32_t ca,
                           int ys, int cs, int w, int h, bool nv21,
                           uint8_t *y, uint8_t *uv)
{
    int r, x;

    for (r = 0; r < h; r++) {
        vpu_read_mem(s, VPU_U_EFE, ya + r * ys, y + r * w, w);
    }
    for (r = 0; r < h / 2; r++) {
        uint8_t *p = uv + r * w;

        vpu_read_mem(s, VPU_U_EFE, ca + r * cs, p, w);
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
    /* libimp's slice header: deblocking on, offsets 0, cabac_init_idc 0 */
    const H264Deblock db = { 0, 0, 0 };
    H264PStats st = { 0 };
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
    vpu_fetch_nv12(s, vpu_reg(s, R_EFE_RAWY) + fst * 16 * ys,
                   vpu_reg(s, R_EFE_RAWC) + fst * 8 * cs,
                   ys, cs, w, h, EFE_PLANE(ctrl) == 3, y, uv);
    out = g_malloc(cap);
    ref = vpu_ref(s, w, h, &fresh);
    if (p_slice && !fresh) {
        n = h264_slice_p_cabac(qp, cqp, 0, s->t8x8, &db, mbw, mbh, y, w, uv,
                               w, ref->y, ref->uv, out, cap, &st);
    } else {
        n = h264_slice_i16dc_cabac_ref(qp, cqp, &db, mbw, mbh, y, w, uv, w,
                                       ref->y, ref->uv, out, cap);
    }
    if (n > cap) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: bitstream buffer full "
                      "(%zu > %u)\n", n, cap);
        n = cap;
    }
    vpu_write_mem(s, VPU_U_SDE, bs, out, n);
    vpu_set(s, R_SDE_BSLEN, n);
    /* a cost for the rate control: the source's activity, never 0 */
    vpu_set(s, R_EFE_SSAD, mbw * mbh * 256 + 1);
    vpu_set(s, R_SDE_STAT, vpu_reg(s, R_SDE_STAT) | SDE_STAT_BSEND);
    vpu_set(s, R_DBLK_GSTA, vpu_reg(s, R_DBLK_GSTA) | 1);
    if (s->debug) {
        qemu_log("ingenic-vpu: H.264 %c slice %dx%d rows %d-%d qp %d "
                 "src %08x/%08x stride %d/%d -> %08" HWADDR_PRIx
                 " %zu bytes (skip %d inter %d intra %d, qp %d)\n",
                 p_slice ? 'P' : 'I', w, h, fst, lst, qp,
                 vpu_reg(s, R_EFE_RAWY), vpu_reg(s, R_EFE_RAWC), ys, cs,
                 bs, n, st.skip, st.inter, st.intra, st.qp);
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
    vpu_fetch_nv12(s, vpu_reg(s, R_EFE_RAWY), vpu_reg(s, R_EFE_RAWC), ys, cs,
                   w, h, EFE_PLANE(vpu_reg(s, R_EFE_CTRL)) == 3, y, uv);
    for (i = 0; i < w * h / 4; i++) {
        cb[i] = uv[2 * i];
        cr[i] = uv[2 * i + 1];
    }
    vpu_jpeg_qtab(s, ql, qc);
    out = g_malloc(cap);
    n = jpeg_enc_scan(y, w, cb, cr, w / 2, w, h, ql, qc, false, out, cap,
                      &overflow);
    vpu_write_mem(s, VPU_U_JPGC, bs, out, n);
    vpu_set(s, R_JPGC_STAT, JPGC_STAT_ENDF | (n & 0xFFFFFF));
    if (s->debug) {
        qemu_log("ingenic-vpu: JPEG %dx%d q0 %d/%d -> %08" HWADDR_PRIx
                 " %zu bytes\n", w, h, ql[0], qc[0], bs, n);
    }
}

/*
 * JPEG decoding: the JPGC alone, started by a chain that ends with
 * JPGC_TRIG = 7 (libimp's T20_JPEGD_SliceInit, T23 and T30; libimp parses
 * the headers itself, ijpegd_decode_header).  What the chain programs:
 *   BSA      the scan as it is in the file: byte-stuffed (FF 00), with
 *            its restart markers (FF Dn) and the EOI behind it
 *   GLBI     [5:4] components - 1, [2] restart markers every NRSM + 1 MCUs
 *   NMCU     MCUs - 1; libimp counts 16x16 macroblocks (4:2:0's MCU)
 *   P0C..P3C per component: [7:4] blocks per MCU - 1, [3:2] quantiser
 *            table, [1] AC table, [0] DC table
 *   QMEM     four tables of 64 words, the quantisers in zigzag order
 *   HUFM     per Huffman table (16 words: AC0, DC0, AC1, DC1) the
 *            canonical first code of each length L, its low min(L, 8)
 *            bits: lengths 1..8 packed from the top of a 36-bit field
 *            (words 2-3), 9..12 and 13..16 a byte each (words 1 and 0)
 *   HUFB     per table and length, 9 bits: where the length's symbols
 *            start in HUFS minus its first code (modulo 512)
 *   HUFS     the symbols, one a word: AC0 from 0, AC1 from 174, both DC
 *            tables in the nibbles of the words from 162 (table 0 low)
 *   P0A/P1A  the picture, tiled ("T420", libimp's c_copy_frame_t420_to_
 *            nv12): macroblock k's 16x16 luma at P0A + 256 k, its chroma
 *            at P1A + 128 k, eight rows of 8 Cb then 8 Cr bytes
 * The JPGC does not know the picture's width, so its MCUs go out one after
 * the other in macroblock order -- exact for 4:2:0, libimp's own encoder's
 * format.  Other layouts (an MCU of 1 or 2 luma blocks) put an MCU's blocks
 * in its macroblock's quadrants in order: that is an inference.
 * Completion: JPGC_STAT ENDF and SCH_STAT bit 4, as for encoding, and
 * MCUS the number of MCUs decoded -- the kernel's helix.c takes it as the
 * task's output length and fails a decode that leaves it 0.
 */
typedef struct VpuHuff {
    int mins[17], cnt[17], base[17];
    bool nibble, high;
} VpuHuff;

typedef struct VpuBits {
    const uint8_t *p, *end;
    uint32_t acc;
    int n;
} VpuBits;

static int vpu_bit(VpuBits *b)
{
    if (!b->n) {
        /* FF 00 is a data byte FF; a marker ends the data (zeros) */
        if (b->p + 1 < b->end && b->p[0] == 0xFF && b->p[1]) {
            b->acc = 0;
        } else {
            b->acc = b->p < b->end ? *b->p++ : 0;
            if (b->acc == 0xFF && b->p < b->end) {
                b->p++;
            }
        }
        b->n = 8;
    }
    b->n--;
    return (b->acc >> b->n) & 1;
}

static int vpu_bits(VpuBits *b, int n)
{
    int v = 0;

    while (n--) {
        v = (v << 1) | vpu_bit(b);
    }
    return v;
}

/* HUFM/HUFB of table @g back into the canonical code's ranges */
static void vpu_huff_load(IngenicVpuState *s, int g, VpuHuff *h)
{
    uint32_t w[4];
    uint64_t lo;
    int a[16], l, m;

    for (l = 0; l < 4; l++) {
        w[l] = vpu_reg(s, R_JPGC_HUFM + 4 * (4 * g + l));
    }
    lo = ((uint64_t)(w[3] & 0xF) << 32) | w[2];
    for (l = 0, m = 36; l < 8; l++) {
        m -= l + 1;
        a[l] = (lo >> m) & ((1u << (l + 1)) - 1);
    }
    for (l = 0; l < 4; l++) {
        a[8 + l] = (w[1] >> (24 - 8 * l)) & 0xFF;
        a[12 + l] = (w[0] >> (24 - 8 * l)) & 0xFF;
    }
    /* first codes: exact up to 8 bits, then continued from the counts */
    m = 0;
    for (l = 0; l < 16; l++) {
        int c = 0, wd;

        h->mins[l + 1] = l < 8 ? a[l] : m;
        h->base[l + 1] = vpu_reg(s, R_JPGC_HUFB + 4 * (16 * g + l)) & 0x1FF;
        if (l == 15) {
            h->cnt[16] = 0x10000 - h->mins[16];     /* the rest */
            break;
        }
        wd = MIN(l + 2, 8);
        while ((((h->mins[l + 1] + c) << 1) & ((1u << wd) - 1)) !=
               (uint32_t)a[l + 1] && c < 256) {
            c++;
        }
        h->cnt[l + 1] = c;
        m = (h->mins[l + 1] + c) << 1;
    }
    h->nibble = g & 1;              /* DC tables share the nibbles */
    h->high = g == 3;
}

static int vpu_huff_sym(IngenicVpuState *s, const VpuHuff *h, VpuBits *b)
{
    int code = 0, l;

    for (l = 1; l <= 16; l++) {
        code = (code << 1) | vpu_bit(b);
        if (code - h->mins[l] >= 0 && code - h->mins[l] < h->cnt[l]) {
            uint32_t v = vpu_reg(s, R_JPGC_HUFS +
                                 4 * (((code + h->base[l]) & 0x1FF) %
                                      JPGC_HUFS_WORDS));

            if (h->nibble) {
                v = h->high ? v >> 4 : v;
            }
            return v & (h->nibble ? 0xF : 0xFF);
        }
    }
    return -1;
}

static int vpu_extend(int v, int n)
{
    return n && v < (1 << (n - 1)) ? v - (1 << n) + 1 : v;
}

/* the inverse DCT of a dequantised block, row-major, to 8-bit samples */
static void vpu_idct(const int *c, uint8_t *out, int stride)
{
    static double cs[8][8];
    static bool init;
    double t[64];
    int x, y, u;

    if (!init) {
        for (x = 0; x < 8; x++) {
            for (u = 0; u < 8; u++) {
                cs[x][u] = (u ? 1.0 : M_SQRT1_2) *
                           cos((2 * x + 1) * u * M_PI / 16) / 2;
            }
        }
        init = true;
    }
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
            double a = 0;

            for (u = 0; u < 8; u++) {
                a += cs[x][u] * c[y * 8 + u];
            }
            t[y * 8 + x] = a;
        }
    }
    for (x = 0; x < 8; x++) {
        for (y = 0; y < 8; y++) {
            double a = 0;
            int v;

            for (u = 0; u < 8; u++) {
                a += cs[y][u] * t[u * 8 + x];
            }
            v = (int)lrint(a) + 128;
            out[y * stride + x] = MIN(MAX(v, 0), 255);
        }
    }
}

static void vpu_jpeg_decode(IngenicVpuState *s)
{
    const uint32_t glbi = vpu_reg(s, R_JPGC_GLBI);
    const int ncomp = GLBI_NCOMP(glbi);
    const int nmcu = vpu_reg(s, R_JPGC_NMCU) + 1;
    const int rst = (glbi & GLBI_RSTER) ? vpu_reg(s, R_JPGC_NRSM) + 1 : 0;
    const uint32_t bsa = vpu_reg(s, R_JPGC_BSA);
    const uint32_t p0a = vpu_reg(s, R_JPGC_P0A), p1a = vpu_reg(s, R_JPGC_P1A);
    const size_t cap = 4 * MiB;
    g_autofree uint8_t *bs = g_malloc(cap);
    uint8_t mby[256], mbc[128], blk[64];
    VpuHuff huff[4];
    VpuBits b = { .p = bs, .end = bs + cap };
    int pred[4] = { 0 }, cfg[4], i, k, c, n;

    if (nmcu > (VPU_MAX_DIM / 16) * (VPU_MAX_DIM / 16)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: JPEG decode of %d "
                      "MCUs\n", nmcu);
        return;
    }
    vpu_read_mem(s, VPU_U_JPGC, bsa, bs, cap);
    for (i = 0; i < 4; i++) {
        vpu_huff_load(s, i, &huff[i]);
        cfg[i] = vpu_reg(s, R_JPGC_P0C + 4 * i);
    }
    if (ncomp == 3 && (extract32(cfg[0], 4, 4) != 3 ||
                       extract32(cfg[1], 4, 4) || extract32(cfg[2], 4, 4))) {
        qemu_log_mask(LOG_UNIMP, "ingenic-vpu: JPEG decode of a layout "
                      "other than 4:2:0 (%02x %02x %02x)\n", cfg[0], cfg[1],
                      cfg[2]);
    }
    for (k = 0; k < nmcu; k++) {
        if (rst && k && !(k % rst)) {
            /* byte-align and step over FF Dn; the predictors start over */
            b.n = 0;
            if (b.p + 1 < b.end && b.p[0] == 0xFF &&
                (b.p[1] & 0xF8) == 0xD0) {
                b.p += 2;
            }
            memset(pred, 0, sizeof(pred));
        }
        memset(mby, 0, sizeof(mby));
        memset(mbc, 0x80, sizeof(mbc));
        for (c = 0; c < ncomp && c < 4; c++) {
            const int nblk = extract32(cfg[c], 4, 4) + 1;
            const VpuHuff *dc = &huff[extract32(cfg[c], 0, 1) ? 3 : 1];
            const VpuHuff *ac = &huff[extract32(cfg[c], 1, 1) ? 2 : 0];
            const int qt = extract32(cfg[c], 2, 2);

            for (n = 0; n < nblk; n++) {
                int coef[64] = { 0 }, t, z;

                t = vpu_huff_sym(s, dc, &b);
                if (t < 0) {
                    goto bad;
                }
                pred[c] += vpu_extend(vpu_bits(&b, t), t);
                coef[0] = pred[c] * (int)vpu_reg(s, R_JPGC_QMEM + 4 * 64 * qt);
                for (z = 1; z < 64; z++) {
                    int rs = vpu_huff_sym(s, ac, &b), r, sz;

                    if (rs < 0) {
                        goto bad;
                    }
                    r = rs >> 4;
                    sz = rs & 15;
                    if (!sz) {
                        if (r != 15) {
                            break;              /* end of block */
                        }
                        z += 15;
                        continue;
                    }
                    z += r;
                    if (z > 63) {
                        break;
                    }
                    coef[vpu_zigzag[z]] = vpu_extend(vpu_bits(&b, sz), sz) *
                        (int)vpu_reg(s, R_JPGC_QMEM + 4 * (64 * qt + z));
                }
                vpu_idct(coef, blk, 8);
                if (c == 0) {
                    /* the n-th luma block: quadrant n of the macroblock */
                    uint8_t *q = mby + (n & 2 ? 128 : 0) + (n & 1 ? 8 : 0);
                    int r;

                    for (r = 0; r < 8 && n < 4; r++) {
                        memcpy(q + 16 * r, blk + 8 * r, 8);
                    }
                } else if (c < 3 && n == 0) {
                    int r;

                    for (r = 0; r < 8; r++) {
                        memcpy(mbc + 16 * r + (c == 2 ? 8 : 0), blk + 8 * r, 8);
                    }
                }
            }
        }
        vpu_write_mem(s, VPU_U_JPGC, p0a + 256 * k, mby, sizeof(mby));
        vpu_write_mem(s, VPU_U_JPGC, p1a + 128 * k, mbc, sizeof(mbc));
    }
    vpu_set(s, R_JPGC_MCUS, k);
    if (s->debug) {
        qemu_log("ingenic-vpu: JPEG decode %d MCUs, %d components, restart "
                 "%d -> %08x/%08x\n", nmcu, ncomp, rst, p0a, p1a);
    }
    return;
bad:
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-vpu: JPEG decode: bad Huffman "
                  "code in MCU %d of %d\n", k, nmcu);
    vpu_set(s, R_JPGC_MCUS, k);
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
    vpu_fetch_nv12(s, vpu_reg(s, R_RX_EFE_RAWY), vpu_reg(s, R_RX_EFE_RAWC), ys,
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
    vpu_write_mem(s, VPU_U_SDE, bs, out, n);
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
        vpu_read_mem(s, VPU_U_VDMA, addr + 8 * i, w, 8);
        w[0] = le32_to_cpu(w[0]);
        w[1] = le32_to_cpu(w[1]);
        if (s->debug) {
            qemu_log("ingenic-vpu: chain %08x [%4d] %05x <- %08x%s\n", addr,
                     i, ACFG_IDX(w[1]), w[0],
                     (w[1] & ACFG_TERM) ? " TERM" : "");
        }
        if (w[1] & ACFG_VLD) {
            vpu_reg_write(s, vpu_map(s, ACFG_IDX(w[1])), w[0], true);
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
            /* the JPGC on its own: a decode */
            vpu_jpeg_decode(s);
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
        vpu_set(s, R_JPGC_MCUS, 0);
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

    off = vpu_map(s, off);
    if (s->debug) {
        qemu_log("ingenic-vpu: read  %05" HWADDR_PRIx " = %08x\n", off,
                 vpu_reg(s, off));
    }
    return vpu_reg(s, off);
}

static void vpu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    IngenicVpuState *s = opaque;

    off = vpu_map(s, off);
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
        vpu_set(s, R_JPGC_MCUS, 0);
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
    DEFINE_PROP_UINT32("layout", IngenicVpuState, layout, VPU_LAYOUT_HELIX),
    DEFINE_PROP_BOOL("debug", IngenicVpuState, debug, false),
    DEFINE_PROP_BOOL("transform-8x8", IngenicVpuState, t8x8, true),
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
