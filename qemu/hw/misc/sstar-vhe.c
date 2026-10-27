/*
 * MStar infinity3 VHE: the H.265 encoder, a VeriSilicon Hantro H2 v3
 * (vendor driver drivers/mstar/vhe with vhe/h2v3, "mstar,mvhe" interrupt
 * "mvhe_isr", GIC SPI 53, INT_IRQ_VHE).
 *
 * Register-driven, no firmware.  Two windows (h2v3_ios.c):
 *
 *   - conf, bank 0x1328 at 0x1F265000, 16-bit registers on the RIU's
 *     4-byte stride: 0x01 bit 0 the soft reset (active low; _sw_rst()
 *     writes 0 then 1), 0x04 [11:8] interrupt clear (irq_clr() sets them
 *     read-modify-write, so they read back 0), 0x48 [3:0] interrupt mask
 *     (1 = masked).
 *   - the Hantro software registers ("swregs", banks 0x1329-0x132B from
 *     0x1F265200): swreg i is 32 bits split over two RIU words, the low
 *     half at 8 * i and the high half at 8 * i + 4 (REGL_SAVE/REGL_LOAD);
 *     H2V3_REG_SIZE = 165.
 *
 * Per frame _enc_fire() resets the core, clears and unmasks the interrupt,
 * reads swreg0 (product) and swreg80 (configuration) without checking
 * them, writes swregs 1..164 and then swreg5 with bit 0 (ENC_E): the
 * start.  The model encodes on a timer and completes as the core does:
 * the slice NAL(s) at BASE_OUTBS_ADDR (swreg8, an MIU byte address past
 * the VPS/SPS/PPS and SEI the driver has put in front, h2v3_ops.c
 * _put_data()), the bytes written in swreg9 (which held the buffer's
 * size), swreg1 = IRQ | FRAME_RDY (0x5) and ENC_E cleared.  The line is
 * swreg1 bit 0 while conf 0x48 bit 0 is clear; _isr_func() reads swreg1,
 * writes it to 0 and masks.  A stream larger than the buffer raises
 * BUF_FULL (bit 5) instead, which the driver logs and resets on.
 *
 * The core writes the whole slice NAL: start code, NAL header (type
 * swreg164 [31:26], temporal id [25:23]), a slice header generated from
 * the registers, CABAC slice data and the trailing bits.  The header here
 * follows the parameter sets h265_enc.c writes (h265_write_sps/_pps and
 * the defaults sps_def/pps_def, h2v3_ops.c _set_conf()/_enc_conf()):
 * PPS 0 / SPS 0, no dependent slices, no output flag, no extra slice
 * header bits, sign hiding off, cabac_init_present 1, one reference by
 * default, no weighted prediction, no tiles or WPP, loop filter across
 * slices on, deblocking control present, no lists modification, temporal
 * MVP off, PCM off, no scaling list data.  What varies comes from the
 * registers: POC (swreg11, 8 bits as h265_sh_parser() reads it),
 * num_short_term_ref_pic_sets (swreg6 [24:20]) and the set (swreg6 [6:2]),
 * SAO (swreg4 bit 6: slice_sao_luma/chroma_flag present, coded 0),
 * num_ref_idx_active_override (swreg4 bit 7, the count swreg17 [7:6]),
 * cabac_init_flag (swreg7 bit 25), slice QP (swreg7 [13:8]) against
 * init_qp (swreg7 [31:26]), the deblocking override (swreg5 bits 9/8,
 * swreg6 [15] disable, [14:11] tc and [10:7] beta offsets / 2), cu QP
 * delta (swreg6 bit 0, depth swreg7 [15:14]), chroma QP offset (swreg4
 * [17:13]), CTB, CB and TB sizes and transform depths (swreg4).  Slices
 * are swreg6 [31:25] CTB rows each (0: one), five_minus_max_num_merge_cand
 * is 1 as in the driver's own header writer (h265_sh_writer()).
 *
 * The slice data is the model's own: a small, standards-conforming encoder
 * with a CABAC coder (ITU-T H.265 9.3).  I slices are 16x16 intra CUs
 * (smaller at the picture's edges, larger if the minimum CB is), DC
 * prediction for luma and chroma (intra_chroma_pred_mode 4) and a coded
 * residual: the HEVC integer DCT, flat quantization at the slice QP.  P
 * slices code each CU as skip (merge candidate 0, the zero vector into
 * the first reference) when its residual quantizes to nothing, and
 * otherwise as a 2Nx2N merge CU with the zero vector and the residual.
 * The reconstruction runs the decoder's process including the deblocking
 * filter (SAO is switched off in every slice), is written to the
 * reconstruction buffer (swreg15/16) as NV12 and read back through the
 * reference (swreg18/19) for the next P frame, so the stream decodes to
 * what the model holds, without drift.  Scaling lists (swreg4 bit 8) are
 * not applied; the driver never enables them.
 *
 * The source is NV12 or YUYV (swreg38 [31:28] 1/2, row length [19:6] in
 * pixels, chroma swap swreg45 bit 27) read from DRAM; when its address is
 * outside DRAM (the scaler streaming through IMI) it is the test pattern
 * of sstar-pattern.h.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-cabac.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_SSTAR_VHE "sstar-vhe"
OBJECT_DECLARE_SIMPLE_TYPE(SstarVheState, SSTAR_VHE)

#define VHE_CONF_REGS       0x80
#define VHE_SWREGS          192         /* 0x600 bytes; 165 are used */
#define VHE_SWREGS_USED     165

#define CONF_RSTZ           0x01
#define CONF_IRQ_CLR        0x04
#define CONF_IRQ_MASK       0x48

/* swregs, and fields as register, msb, lsb (h2v3_regtab.h) */
typedef struct VheField {
    uint8_t reg, msb, lsb;
} VheField;

#define SW_PRODUCT          0
#define SW_STATUS           1
#define SW_ENC              5
#define SW_OUTBS_ADDR       8
#define SW_OUTBS_LIMIT      9
#define SW_SLICE_NALS       10
#define SW_POC              11
#define SW_INPUT_Y          12
#define SW_INPUT_C          13
#define SW_RECON_Y          15
#define SW_RECON_C          16
#define SW_L0RP0_Y          18
#define SW_L0RP0_C          19
#define SW_CONFIG           80
#define SW_CYCLES           82

#define F_MIN_CB            ((VheField){ 4, 26, 25 })
#define F_MAX_CB            ((VheField){ 4, 24, 23 })
#define F_MIN_TB            ((VheField){ 4, 22, 21 })
#define F_MAX_TB            ((VheField){ 4, 20, 19 })
#define F_CHROMA_QP_OFFSET  ((VheField){ 4, 17, 13 })
#define F_SCALING_LIST      ((VheField){ 4, 8, 8 })
#define F_ACTIVE_OVERRIDE   ((VheField){ 4, 7, 7 })
#define F_SAO               ((VheField){ 4, 6, 6 })
#define F_DEPTH_INTRA       ((VheField){ 4, 5, 3 })
#define F_DEPTH_INTER       ((VheField){ 4, 2, 0 })
#define F_PIC_WIDTH         ((VheField){ 5, 31, 22 })
#define F_PIC_HEIGHT        ((VheField){ 5, 21, 11 })
#define F_PPS_DBK_OVERRIDE  ((VheField){ 5, 9, 9 })
#define F_DBK_OVERRIDE      ((VheField){ 5, 8, 8 })
#define F_CODING_TYPE       ((VheField){ 5, 2, 1 })
#define F_SLICE_ROWS        ((VheField){ 6, 31, 25 })
#define F_NUM_RPS           ((VheField){ 6, 24, 20 })
#define F_DBK_DISABLE       ((VheField){ 6, 15, 15 })
#define F_DBK_TC            ((VheField){ 6, 14, 11 })
#define F_DBK_BETA          ((VheField){ 6, 10, 7 })
#define F_RPS_ID            ((VheField){ 6, 6, 2 })
#define F_NAL_SIZE_WRITE    ((VheField){ 6, 1, 1 })
#define F_CU_QP_DELTA       ((VheField){ 6, 0, 0 })
#define F_INIT_QP           ((VheField){ 7, 31, 26 })
#define F_CABAC_INIT        ((VheField){ 7, 25, 25 })
#define F_QP_DELTA_DEPTH    ((VheField){ 7, 15, 14 })
#define F_PIC_QP            ((VheField){ 7, 13, 8 })
#define F_ACTIVE_L0         ((VheField){ 17, 7, 6 })
#define F_INPUT_FORMAT      ((VheField){ 38, 31, 28 })
#define F_ROWLENGTH         ((VheField){ 38, 19, 6 })
#define F_CHROMA_SWAP       ((VheField){ 45, 27, 27 })
#define F_NAL_TYPE          ((VheField){ 164, 31, 26 })
#define F_TEMPORAL_ID       ((VheField){ 164, 25, 23 })
#define F_INTRACU8NUM       ((VheField){ 111, 31, 12 })
#define F_SKIPCU8NUM        ((VheField){ 112, 31, 12 })
#define F_CTBBITS_MIN       ((VheField){ 119, 31, 16 })
#define F_CTBBITS_MAX       ((VheField){ 119, 15, 0 })
#define F_QP_SUM            ((VheField){ 159, 31, 8 })
#define F_QP_NUM            ((VheField){ 160, 30, 14 })
#define F_PIC_COMPLEXITY    ((VheField){ 161, 31, 9 })

#define STATUS_IRQ          (1u << 0)
#define STATUS_FRAME_RDY    (1u << 2)
#define STATUS_BUF_FULL     (1u << 5)
#define STATUS_ALL          0x3FDu
#define ENC_E               (1u << 0)

#define H2_PRODUCT_ID       0x48320301u /* "H2", 3.1 */
#define H2_HW_CONFIG        (1u << 27)  /* HEVC */

#define DRAM_BASE           0x20000000
#define ENCODE_NS           (2 * SCALE_MS)
#define DIM_MAX             4096

struct SstarVheState {
    SysBusDevice parent_obj;

    MemoryRegion conf_mr;
    MemoryRegion swreg_mr;
    qemu_irq irq;
    QEMUTimer *timer;

    uint16_t conf[VHE_CONF_REGS];
    uint32_t sw[VHE_SWREGS];
    uint32_t frames;
    uint64_t dram_size;
    bool trace;
};

static uint32_t vhe_f(SstarVheState *s, VheField f)
{
    return (s->sw[f.reg] >> f.lsb) & (uint32_t)((2ull << (f.msb - f.lsb)) - 1);
}

static void vhe_set_f(SstarVheState *s, VheField f, uint32_t v)
{
    uint32_t m = (uint32_t)((2ull << (f.msb - f.lsb)) - 1) << f.lsb;

    s->sw[f.reg] = (s->sw[f.reg] & ~m) | ((v << f.lsb) & m);
}

static int sext(uint32_t v, int bits)
{
    return (int32_t)(v << (32 - bits)) >> (32 - bits);
}

/* ------------------------------------------------------------------ */
/* CABAC (H.265 9.3): the contexts, the engine is sstar-cabac.h's      */

enum {
    CTX_SPLIT_CU = 0,
    CTX_SKIP = CTX_SPLIT_CU + 3,
    CTX_CU_QP_DELTA = CTX_SKIP + 3,
    CTX_PRED_MODE = CTX_CU_QP_DELTA + 2,
    CTX_PART_MODE = CTX_PRED_MODE + 1,
    CTX_PREV_INTRA = CTX_PART_MODE + 1,
    CTX_CHROMA_MODE = CTX_PREV_INTRA + 1,
    CTX_MERGE_FLAG = CTX_CHROMA_MODE + 1,
    CTX_MERGE_IDX = CTX_MERGE_FLAG + 1,
    CTX_SPLIT_TU = CTX_MERGE_IDX + 1,
    CTX_CBF_LUMA = CTX_SPLIT_TU + 3,
    CTX_CBF_C = CTX_CBF_LUMA + 2,
    CTX_LAST_X = CTX_CBF_C + 4,
    CTX_LAST_Y = CTX_LAST_X + 18,
    CTX_CSBF = CTX_LAST_Y + 18,
    CTX_SIG = CTX_CSBF + 4,
    CTX_GT1 = CTX_SIG + 42,
    CTX_GT2 = CTX_GT1 + 24,
    CTX_NUM = CTX_GT2 + 6,
};

/* initValue per context (Tables 9-5 to 9-37), by initType */
static const uint8_t vhe_ctx_init[3][CTX_NUM] = {
    {   /* initType 0: I slices */
        /* split_coding_unit_flag */
        139, 141, 157,
        /* skip_flag */
        154, 154, 154,
        /* cu_qp_delta */
        154, 154,
        /* pred_mode */
        154,
        /* part_mode */
        184,
        /* prev_intra_luma_pred_mode */
        184,
        /* intra_chroma_pred_mode */
        63,
        /* merge_flag */
        154,
        /* merge_idx */
        154,
        /* split_transform_flag */
        153, 138, 138,
        /* cbf_luma */
        111, 141,
        /* cbf_cb, cbf_cr */
        94, 138, 182, 154,
        /* last_significant_coeff_x_prefix */
        110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143,
        127, 111, 79, 108, 123, 63,
        /* last_significant_coeff_y_prefix */
        110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143,
        127, 111, 79, 108, 123, 63,
        /* significant_coeff_group_flag */
        91, 171, 134, 141,
        /* significant_coeff_flag */
        111, 111, 125, 110, 110, 94, 124, 108, 124, 107, 125, 141,
        179, 153, 125, 107, 125, 141, 179, 153, 125, 107, 125, 141,
        179, 153, 125, 140, 139, 182, 182, 152, 136, 152, 136, 153,
        136, 139, 111, 136, 139, 111,
        /* coeff_abs_level_greater1_flag */
        140, 92, 137, 138, 140, 152, 138, 139, 153, 74, 149, 92,
        139, 107, 122, 152, 140, 179, 166, 182, 140, 227, 122, 197,
        /* coeff_abs_level_greater2_flag */
        138, 153, 136, 167, 152, 152,
    },
    {   /* initType 1: P slices, cabac_init_flag 0 */
        /* split_coding_unit_flag */
        107, 139, 126,
        /* skip_flag */
        197, 185, 201,
        /* cu_qp_delta */
        154, 154,
        /* pred_mode */
        149,
        /* part_mode */
        154,
        /* prev_intra_luma_pred_mode */
        154,
        /* intra_chroma_pred_mode */
        152,
        /* merge_flag */
        110,
        /* merge_idx */
        122,
        /* split_transform_flag */
        124, 138, 94,
        /* cbf_luma */
        153, 111,
        /* cbf_cb, cbf_cr */
        149, 107, 167, 154,
        /* last_significant_coeff_x_prefix */
        125, 110, 94, 110, 95, 79, 125, 111, 110, 78, 110, 111,
        111, 95, 94, 108, 123, 108,
        /* last_significant_coeff_y_prefix */
        125, 110, 94, 110, 95, 79, 125, 111, 110, 78, 110, 111,
        111, 95, 94, 108, 123, 108,
        /* significant_coeff_group_flag */
        121, 140, 61, 154,
        /* significant_coeff_flag */
        155, 154, 139, 153, 139, 123, 123, 63, 153, 166, 183, 140,
        136, 153, 154, 166, 183, 140, 136, 153, 154, 166, 183, 140,
        136, 153, 154, 170, 153, 123, 123, 107, 121, 107, 121, 167,
        151, 183, 140, 151, 183, 140,
        /* coeff_abs_level_greater1_flag */
        154, 196, 196, 167, 154, 152, 167, 182, 182, 134, 149, 136,
        153, 121, 136, 137, 169, 194, 166, 167, 154, 167, 137, 182,
        /* coeff_abs_level_greater2_flag */
        107, 167, 91, 122, 107, 167,
    },
    {   /* initType 2: P slices, cabac_init_flag 1 */
        /* split_coding_unit_flag */
        107, 139, 126,
        /* skip_flag */
        197, 185, 201,
        /* cu_qp_delta */
        154, 154,
        /* pred_mode */
        134,
        /* part_mode */
        154,
        /* prev_intra_luma_pred_mode */
        183,
        /* intra_chroma_pred_mode */
        152,
        /* merge_flag */
        154,
        /* merge_idx */
        137,
        /* split_transform_flag */
        224, 167, 122,
        /* cbf_luma */
        153, 111,
        /* cbf_cb, cbf_cr */
        149, 92, 167, 154,
        /* last_significant_coeff_x_prefix */
        125, 110, 124, 110, 95, 94, 125, 111, 111, 79, 125, 126,
        111, 111, 79, 108, 123, 93,
        /* last_significant_coeff_y_prefix */
        125, 110, 124, 110, 95, 94, 125, 111, 111, 79, 125, 126,
        111, 111, 79, 108, 123, 93,
        /* significant_coeff_group_flag */
        121, 140, 61, 154,
        /* significant_coeff_flag */
        170, 154, 139, 153, 139, 123, 123, 63, 124, 166, 183, 140,
        136, 153, 154, 166, 183, 140, 136, 153, 154, 166, 183, 140,
        136, 153, 154, 170, 153, 138, 138, 122, 121, 122, 121, 167,
        151, 183, 140, 151, 183, 140,
        /* coeff_abs_level_greater1_flag */
        154, 196, 167, 167, 154, 152, 167, 182, 182, 134, 149, 136,
        153, 121, 136, 122, 169, 208, 166, 167, 154, 152, 167, 182,
        /* coeff_abs_level_greater2_flag */
        107, 167, 91, 107, 107, 167,
    },
};

QEMU_BUILD_BUG_ON(CTX_NUM > SSTAR_CABAC_CTX_MAX);

static void cabac_start(SstarCabac *c, SstarBits *bw, int init_type, int qp)
{
    int i;

    sstar_cabac_start(c, bw);
    for (i = 0; i < CTX_NUM; i++) {         /* 9.3.2.2 */
        int v = vhe_ctx_init[init_type][i];

        sstar_cabac_init_ctx(c, i, (v >> 4) * 5 - 45, ((v & 15) << 3) - 16,
                             qp);
    }
}

/* ------------------------------------------------------------------ */
/* Transform, quantization (8.6), deblocking (8.7.2) tables            */

static int8_t vhe_dct[32][32];          /* [frequency][sample] */

static void vhe_dct_init(void)
{
    /* the 32-point matrix's distinct magnitudes, cos(j * pi / 64) */
    static const uint8_t c[32] = {
        64, 90, 90, 90, 89, 88, 87, 85, 83, 82, 80, 78, 75, 73, 70, 67,
        64, 61, 57, 54, 50, 46, 43, 38, 36, 31, 25, 22, 18, 13, 9, 4,
    };
    int k, n;

    for (k = 0; k < 32; k++) {
        for (n = 0; n < 32; n++) {
            int j = (2 * n + 1) * k % 128;

            vhe_dct[k][n] = !k ? 64 :
                            j < 32 ? c[j] :
                            j < 64 ? -c[64 - j] :
                            j < 96 ? -c[j - 64] : c[128 - j];
        }
    }
}

static const uint16_t vhe_qscale[6] = {
    26214, 23302, 20560, 18396, 16384, 14564,
};
static const uint8_t vhe_level_scale[6] = { 40, 45, 51, 57, 64, 72 };

static const uint8_t vhe_beta[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7, 8,
    9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24, 26, 28, 30, 32, 34,
    36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
};
static const uint8_t vhe_tc[54] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
    5, 5, 6, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 20, 22, 24,
};

/* QpC from qPi for 4:2:0 (Table 8-10) */
static int vhe_chroma_qp(int qpi)
{
    static const uint8_t t[14] = {
        29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37,
    };

    return qpi < 30 ? qpi : qpi > 43 ? qpi - 6 : t[qpi - 30];
}

/* up-right diagonal scans (6.5.3) of 1x1 .. 8x8 blocks, as (x, y) */
static uint8_t vhe_diag[4][64][2];

static void vhe_diag_init(void)
{
    int l, i, x, y;

    for (l = 0; l < 4; l++) {
        int size = 1 << l;

        i = x = y = 0;
        while (i < size * size) {
            while (y >= 0) {
                if (x < size && y < size) {
                    vhe_diag[l][i][0] = x;
                    vhe_diag[l][i][1] = y;
                    i++;
                }
                y--;
                x++;
            }
            y = x;
            x = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* The encoder                                                        */

typedef struct VhePic {
    int w[3], h[3];
    uint8_t *p[3];                      /* Y, Cb, Cr planes */
} VhePic;

typedef struct VheEnc {
    int w, h;
    int ctb_log2, min_cb_log2, min_tb_log2, max_tb_log2, cu_log2;
    int depth_intra, depth_inter;
    int ctb_w, ctb_h, nctb;
    bool islice;
    int nal_type, tid, poc;
    int qp, init_qp, qpc[3];
    int cb_offset;
    bool sao, cabac_init_flag, active_override;
    int active_l0, num_rps, rps_id;
    bool dbk_override_en, dbk_override, dbk_disable;
    int dbk_beta, dbk_tc;               /* offsets / 2 */
    bool cu_qp_delta, qg_coded;
    int qg_log2;
    int max_merge;
    int skip_thr;

    VhePic src, rec, ref;
    bool have_ref;

    /* per 8x8 block */
    int bw, bh;
    uint8_t *depth, *skip, *intra, *nz;
    uint32_t *tu;
    uint32_t tus;
    int *slice_of_ctb;

    SstarCabac cab;
    struct VheCu *cu;                   /* the CU being coded */
    /* statistics */
    uint32_t intra8, skip8;
} VheEnc;

static void vhe_pic_alloc(VhePic *p, int w, int h)
{
    int c;

    for (c = 0; c < 3; c++) {
        p->w[c] = c ? w / 2 : w;
        p->h[c] = c ? h / 2 : h;
        p->p[c] = g_malloc0((size_t)p->w[c] * p->h[c]);
    }
}

static void vhe_pic_free(VhePic *p)
{
    int c;

    for (c = 0; c < 3; c++) {
        g_free(p->p[c]);
        p->p[c] = NULL;
    }
}

static int blk(VheEnc *e, int x, int y)
{
    return (y >> 3) * e->bw + (x >> 3);
}

/* z-scan availability (6.4.1) of (xn, yn) for the block at (xc, yc) */
static bool vhe_avail(VheEnc *e, int xc, int yc, int xn, int yn)
{
    int mask = (1 << e->ctb_log2) - 1, ctbn, ctbc, i;
    uint32_t zn = 0, zc = 0;

    if (xn < 0 || yn < 0 || xn >= e->w || yn >= e->h) {
        return false;
    }
    ctbn = (yn >> e->ctb_log2) * e->ctb_w + (xn >> e->ctb_log2);
    ctbc = (yc >> e->ctb_log2) * e->ctb_w + (xc >> e->ctb_log2);
    if (e->slice_of_ctb[ctbn] != e->slice_of_ctb[ctbc]) {
        return false;
    }
    if (ctbn != ctbc) {
        return ctbn < ctbc;
    }
    for (i = 0; i < 4; i++) {           /* 4x4 units in a CTB up to 64 */
        zn |= (((xn & mask) >> (2 + i)) & 1) << (2 * i);
        zn |= (((yn & mask) >> (2 + i)) & 1) << (2 * i + 1);
        zc |= (((xc & mask) >> (2 + i)) & 1) << (2 * i);
        zc |= (((yc & mask) >> (2 + i)) & 1) << (2 * i + 1);
    }
    return zn < zc;
}

/* Intra DC prediction (8.4.4.2) of an n x n block of component c. */
static void vhe_pred_dc(VheEnc *e, int c, int x0, int y0, int log2,
                        uint8_t *pred)
{
    int n = 1 << log2, sub = c ? 1 : 0, stride = e->rec.w[c];
    const uint8_t *pl = e->rec.p[c];
    int ref[4 * 32 + 1];
    bool av[4 * 32 + 1], any = false;
    int k, x, y, sum = 0, dc;

    /* 0 .. 2n-1: p[-1][2n-1 .. 0], 2n: p[-1][-1], 2n+1 ..: p[0..][-1] */
    for (k = 0; k <= 4 * n; k++) {
        int xn = k < 2 * n ? x0 - 1 : k == 2 * n ? x0 - 1 : x0 + k - 2 * n - 1;
        int yn = k < 2 * n ? y0 + 2 * n - 1 - k : y0 - 1;

        av[k] = vhe_avail(e, x0 << sub, y0 << sub, xn << sub, yn << sub);
        ref[k] = av[k] ? pl[yn * stride + xn] : 0;
        any |= av[k];
    }
    if (!any) {
        for (k = 0; k <= 4 * n; k++) {
            ref[k] = 128;
        }
    } else {                            /* substitution, 8.4.4.2.2 */
        if (!av[0]) {
            for (k = 1; !av[k]; k++) {
                continue;
            }
            ref[0] = ref[k];
        }
        for (k = 1; k <= 4 * n; k++) {
            if (!av[k]) {
                ref[k] = ref[k - 1];
            }
        }
    }
    for (k = 0; k < n; k++) {
        sum += ref[2 * n - 1 - k] + ref[2 * n + 1 + k];
    }
    dc = (sum + n) >> (log2 + 1);
    for (y = 0; y < n; y++) {
        for (x = 0; x < n; x++) {
            pred[y * n + x] = dc;
        }
    }
    if (!c && n < 32) {                 /* DC edge filter, 8.4.4.2.5 */
        pred[0] = (ref[2 * n - 1] + 2 * dc + ref[2 * n + 1] + 2) >> 2;
        for (x = 1; x < n; x++) {
            pred[x] = (ref[2 * n + 1 + x] + 3 * dc + 2) >> 2;
        }
        for (y = 1; y < n; y++) {
            pred[y * n] = (ref[2 * n - 1 - y] + 3 * dc + 2) >> 2;
        }
    }
}

/* Forward transform (any rounding will do) and quantization. */
static bool vhe_fwd_quant(const int16_t *res, int log2, int qp, bool intra,
                          int16_t *lev, int lstride)
{
    int n = 1 << log2, step = 32 >> log2, s1 = log2 - 1, s2 = log2 + 6;
    int qbits = 21 + qp / 6 - log2;
    int64_t add = (int64_t)(intra ? 171 : 85) << (qbits - 9);
    int32_t tmp[32 * 32];
    bool nz = false;
    int x, y, u, v;

    for (y = 0; y < n; y++) {
        for (u = 0; u < n; u++) {
            int32_t a = 0;

            for (x = 0; x < n; x++) {
                a += vhe_dct[u * step][x] * res[y * n + x];
            }
            tmp[y * n + u] = (a + (1 << (s1 - 1))) >> s1;
        }
    }
    for (u = 0; u < n; u++) {
        for (v = 0; v < n; v++) {
            int64_t a = 0, l;

            for (y = 0; y < n; y++) {
                a += vhe_dct[v * step][y] * tmp[y * n + u];
            }
            a = (a + (1 << (s2 - 1))) >> s2;
            l = (llabs(a) * vhe_qscale[qp % 6] + add) >> qbits;
            l = MIN(l, 32767);
            lev[v * lstride + u] = a < 0 ? -l : l;
            nz |= l != 0;
        }
    }
    return nz;
}

/* Scaling (8.6.2/8.6.3) and the inverse transform (8.6.4.2). */
static void vhe_dequant_inv(const int16_t *lev, int lstride, int log2, int qp,
                            int16_t *res)
{
    int n = 1 << log2, step = 32 >> log2, bd = log2 + 3;
    int32_t d[32 * 32], g[32 * 32];
    int x, y, k;

    for (y = 0; y < n; y++) {
        for (x = 0; x < n; x++) {
            int64_t v = ((int64_t)lev[y * lstride + x] * 16 *
                         vhe_level_scale[qp % 6] << (qp / 6)) +
                        (1 << (bd - 1));

            d[y * n + x] = MIN(MAX(v >> bd, -32768), 32767);
        }
    }
    for (x = 0; x < n; x++) {           /* columns */
        for (y = 0; y < n; y++) {
            int64_t a = 0;

            for (k = 0; k < n; k++) {
                a += vhe_dct[k * step][y] * d[k * n + x];
            }
            g[y * n + x] = MIN(MAX((a + 64) >> 7, -32768), 32767);
        }
    }
    for (y = 0; y < n; y++) {           /* rows */
        for (x = 0; x < n; x++) {
            int64_t a = 0;

            for (k = 0; k < n; k++) {
                a += vhe_dct[k * step][x] * g[y * n + k];
            }
            res[y * n + x] = (a + 2048) >> 12;
        }
    }
}

/* A CU's transform blocks: levels per component, cbf per TU. */
typedef struct VheCu {
    int x0, y0, log2, tlog2, k;         /* k x k TUs of 1 << tlog2 */
    bool intra;
    int16_t lev[3][64 * 64];
    bool cbf[3][8][8];
} VheCu;

/* Predict, code and reconstruct one component of one TU. */
static void vhe_tu_comp(VheEnc *e, VheCu *cu, int c, int tx, int ty)
{
    int log2 = cu->tlog2 - (c ? 1 : 0), n = 1 << log2;
    int sub = c ? 1 : 0;
    int x0 = (cu->x0 >> sub) + (tx << log2);
    int y0 = (cu->y0 >> sub) + (ty << log2);
    int stride = e->rec.w[c], lstride = 1 << (cu->log2 - sub);
    int16_t *lev = cu->lev[c] + (ty << log2) * lstride + (tx << log2);
    int qp = e->qpc[c];
    uint8_t pred[32 * 32];
    int16_t res[32 * 32];
    int x, y, maxd = 0;
    bool nz = false;

    if (cu->intra) {
        vhe_pred_dc(e, c, x0, y0, log2, pred);
    } else {
        for (y = 0; y < n; y++) {
            memcpy(pred + y * n, e->ref.p[c] + (y0 + y) * stride + x0, n);
        }
    }
    for (y = 0; y < n; y++) {
        for (x = 0; x < n; x++) {
            res[y * n + x] = e->src.p[c][(y0 + y) * stride + x0 + x] -
                             pred[y * n + x];
            maxd = MAX(maxd, abs(res[y * n + x]));
        }
    }
    /* a residual this small is not worth coding in a P frame */
    if (cu->intra || maxd > e->skip_thr) {
        nz = vhe_fwd_quant(res, log2, qp, cu->intra, lev, lstride);
    }
    if (!nz) {
        for (y = 0; y < n; y++) {
            memset(lev + y * lstride, 0, n * sizeof(*lev));
        }
    }
    cu->cbf[c][ty][tx] = nz;
    if (nz) {
        vhe_dequant_inv(lev, lstride, log2, qp, res);
    }
    for (y = 0; y < n; y++) {
        for (x = 0; x < n; x++) {
            int v = pred[y * n + x] + (nz ? res[y * n + x] : 0);

            e->rec.p[c][(y0 + y) * stride + x0 + x] = MIN(MAX(v, 0), 255);
        }
    }
}

/* residual_coding() (7.3.8.11), scanIdx 0 (DC and inter), no hiding */
static void vhe_residual(VheEnc *e, const int16_t *coef, int stride,
                         int log2, int c)
{
    static const uint8_t ctx_map[16] = {
        0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8,
    };
    static const uint8_t group_idx[32] = {
        0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7,
        8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
    };
    static const uint8_t min_in_group[10] = {
        0, 1, 2, 3, 4, 6, 8, 12, 16, 24,
    };
    SstarCabac *cb = &e->cab;
    int lsb = log2 - 2, nsb = 1 << lsb;
    int last_sb = -1, last_p = -1, lx, ly, i, p, k;
    int off, shift, gx, gy, cmax = 2 * log2 - 1;
    uint8_t csbf[8][8] = { { 0 } };
    int c1 = 1;

    for (i = nsb * nsb - 1; i >= 0 && last_sb < 0; i--) {
        for (p = 15; p >= 0; p--) {
            int x = (vhe_diag[lsb][i][0] << 2) + vhe_diag[2][p][0];
            int y = (vhe_diag[lsb][i][1] << 2) + vhe_diag[2][p][1];

            if (coef[y * stride + x]) {
                last_sb = i;
                last_p = p;
                break;
            }
        }
    }
    assert(last_sb >= 0);
    lx = (vhe_diag[lsb][last_sb][0] << 2) + vhe_diag[2][last_p][0];
    ly = (vhe_diag[lsb][last_sb][1] << 2) + vhe_diag[2][last_p][1];

    /* last_sig_coeff_x/y_prefix, then the suffixes (9.3.4.2.3) */
    if (!c) {
        off = 3 * (log2 - 2) + ((log2 - 1) >> 2);
        shift = (log2 + 1) >> 2;
    } else {
        off = 15;
        shift = log2 - 2;
    }
    gx = group_idx[lx];
    gy = group_idx[ly];
    for (k = 0; k < gx; k++) {
        sstar_cabac_bin(cb, 1, CTX_LAST_X + off + (k >> shift));
    }
    if (gx < cmax) {
        sstar_cabac_bin(cb, 0, CTX_LAST_X + off + (gx >> shift));
    }
    for (k = 0; k < gy; k++) {
        sstar_cabac_bin(cb, 1, CTX_LAST_Y + off + (k >> shift));
    }
    if (gy < cmax) {
        sstar_cabac_bin(cb, 0, CTX_LAST_Y + off + (gy >> shift));
    }
    if (gx > 3) {
        sstar_cabac_bypass_bits(cb, lx - min_in_group[gx], (gx >> 1) - 1);
    }
    if (gy > 3) {
        sstar_cabac_bypass_bits(cb, ly - min_in_group[gy], (gy >> 1) - 1);
    }

    for (i = last_sb; i >= 0; i--) {
        int xs = vhe_diag[lsb][i][0], ys = vhe_diag[lsb][i][1];
        int prev = (xs + 1 < nsb ? csbf[xs + 1][ys] : 0) |
                   (ys + 1 < nsb ? csbf[xs][ys + 1] : 0) << 1;
        int abs_l[16], sign = 0, num = 0, first_c2 = -1, ctx_set, rice;
        bool infer_dc = false, any = false;

        for (p = 0; p < 16; p++) {
            int x = (xs << 2) + vhe_diag[2][p][0];
            int y = (ys << 2) + vhe_diag[2][p][1];

            any |= coef[y * stride + x] != 0;
        }
        if (i < last_sb && i > 0) {
            int r = (xs + 1 < nsb ? csbf[xs + 1][ys] : 0) +
                    (ys + 1 < nsb ? csbf[xs][ys + 1] : 0);

            sstar_cabac_bin(cb, any, CTX_CSBF + MIN(r, 1) + (c ? 2 : 0));
            infer_dc = true;
        } else {
            any = true;
        }
        csbf[xs][ys] = any;
        if (!any) {
            continue;
        }
        for (p = i == last_sb ? last_p : 15; p >= 0; p--) {
            int x = (xs << 2) + vhe_diag[2][p][0];
            int y = (ys << 2) + vhe_diag[2][p][1];
            int v = coef[y * stride + x];

            if (i == last_sb && p == last_p) {
                /* the last coefficient: significant by position */
            } else if (p > 0 || !infer_dc) {
                int sig;

                if (log2 == 2) {
                    sig = ctx_map[(y << 2) + x];
                } else if (x + y == 0) {
                    sig = 0;
                } else {
                    int xp = x & 3, yp = y & 3;

                    sig = prev == 0 ? (xp + yp == 0 ? 2 : xp + yp < 3 ? 1 : 0) :
                          prev == 1 ? (yp == 0 ? 2 : yp == 1 ? 1 : 0) :
                          prev == 2 ? (xp == 0 ? 2 : xp == 1 ? 1 : 0) : 2;
                    if (!c) {
                        if (xs || ys) {
                            sig += 3;
                        }
                        sig += log2 == 3 ? 9 : 21;
                    } else {
                        sig += log2 == 3 ? 9 : 12;
                    }
                }
                sstar_cabac_bin(cb, v != 0, CTX_SIG + (c ? 27 : 0) + sig);
                if (v) {
                    infer_dc = false;
                }
            } else {
                /* sub-block DC inferred significant */
                assert(v);
            }
            if (v) {
                abs_l[num] = abs(v);
                sign = sign << 1 | (v < 0);
                num++;
            }
        }

        /* greater1 / greater2 flags (9.3.4.2.6, 9.3.4.2.7) */
        ctx_set = (i == 0 || c) ? 0 : 2;
        if (c1 == 0) {
            ctx_set++;
        }
        c1 = 1;
        for (k = 0; k < MIN(num, 8); k++) {
            int g1 = abs_l[k] > 1;

            sstar_cabac_bin(cb, g1, CTX_GT1 + (c ? 16 : 0) + ctx_set * 4 + c1);
            if (g1) {
                c1 = 0;
                if (first_c2 < 0) {
                    first_c2 = k;
                }
            } else if (c1 > 0 && c1 < 3) {
                c1++;
            }
        }
        if (first_c2 >= 0) {
            sstar_cabac_bin(cb, abs_l[first_c2] > 2,
                      CTX_GT2 + (c ? 4 : 0) + ctx_set);
        }
        sstar_cabac_bypass_bits(cb, sign, num);

        /* coeff_abs_level_remaining (9.3.3.11) */
        rice = 0;
        if (c1 == 0 || num > 8) {
            int first2 = 1;

            for (k = 0; k < num; k++) {
                int base = k < 8 ? 2 + first2 : 1;

                if (abs_l[k] >= base) {
                    uint32_t rem = abs_l[k] - base;

                    if (rem < (3u << rice)) {
                        uint32_t len = rem >> rice;

                        sstar_cabac_bypass_bits(cb, (1u << (len + 1)) - 2,
                                                len + 1);
                        sstar_cabac_bypass_bits(cb, rem & ((1u << rice) - 1),
                                                rice);
                    } else {
                        int len = rice, pre;

                        rem -= 3u << rice;
                        while (rem >= (1u << len)) {
                            rem -= 1u << len;
                            len++;
                        }
                        pre = 3 + len + 1 - rice;
                        sstar_cabac_bypass_bits(cb, (1u << pre) - 2, pre);
                        sstar_cabac_bypass_bits(cb, rem, len);
                    }
                    if (abs_l[k] > 3 * (1 << rice)) {
                        rice = MIN(rice + 1, 4);
                    }
                }
                if (abs_l[k] >= 2) {
                    first2 = 0;
                }
            }
        }
    }
}

/* Is any TU under the node at (tx, ty), 1 << l TUs a side, coded? */
static bool vhe_node_cbf(VheCu *cu, int c, int tx, int ty, int l)
{
    int x, y;

    for (y = ty; y < ty + (1 << l); y++) {
        for (x = tx; x < tx + (1 << l); x++) {
            if (cu->cbf[c][y][x]) {
                return true;
            }
        }
    }
    return false;
}

/* transform_tree() (7.3.8.8) of a CU whose TUs are all cu->tlog2 */
static void vhe_tt(VheEnc *e, VheCu *cu, int tx, int ty, int log2, int depth,
                   bool pcb, bool pcr)
{
    SstarCabac *cb = &e->cab;
    int max_depth = cu->intra ? e->depth_intra : e->depth_inter;
    int l = log2 - cu->tlog2;
    bool split, ccb, ccr;

    if (log2 <= e->max_tb_log2 && log2 > e->min_tb_log2 &&
        depth < max_depth) {
        sstar_cabac_bin(cb, 0, CTX_SPLIT_TU + 5 - log2);
        split = false;
    } else {
        split = log2 > e->max_tb_log2;
    }
    ccb = vhe_node_cbf(cu, 1, tx, ty, l);
    ccr = vhe_node_cbf(cu, 2, tx, ty, l);
    if (log2 > 2) {
        if (depth == 0 || pcb) {
            sstar_cabac_bin(cb, ccb, CTX_CBF_C + depth);
        }
        if (depth == 0 || pcr) {
            sstar_cabac_bin(cb, ccr, CTX_CBF_C + depth);
        }
    }
    if (split) {
        int h = 1 << (l - 1);

        vhe_tt(e, cu, tx, ty, log2 - 1, depth + 1, ccb, ccr);
        vhe_tt(e, cu, tx + h, ty, log2 - 1, depth + 1, ccb, ccr);
        vhe_tt(e, cu, tx, ty + h, log2 - 1, depth + 1, ccb, ccr);
        vhe_tt(e, cu, tx + h, ty + h, log2 - 1, depth + 1, ccb, ccr);
        return;
    }
    {
        bool y = cu->cbf[0][ty][tx];
        int ls = 1 << cu->log2, cs = ls >> 1, n = 1 << log2;

        if (cu->intra || depth || ccb || ccr) {
            sstar_cabac_bin(cb, y, CTX_CBF_LUMA + (depth ? 0 : 1));
        } else {
            assert(y);                  /* inferred 1 */
        }
        if (!(y || ccb || ccr)) {
            return;
        }
        if (e->cu_qp_delta && !e->qg_coded) {
            sstar_cabac_bin(cb, 0, CTX_CU_QP_DELTA); /* cu_qp_delta_abs 0 */
            e->qg_coded = true;
        }
        if (y) {
            vhe_residual(e, cu->lev[0] + ty * n * ls + tx * n, ls, log2, 0);
        }
        if (ccb) {
            vhe_residual(e, cu->lev[1] + ty * (n / 2) * cs + tx * (n / 2), cs,
                         log2 - 1, 1);
        }
        if (ccr) {
            vhe_residual(e, cu->lev[2] + ty * (n / 2) * cs + tx * (n / 2), cs,
                         log2 - 1, 2);
        }
    }
}

/* coding_unit() (7.3.8.5) */
static void vhe_cu(VheEnc *e, int x0, int y0, int log2, int depth)
{
    VheCu *cu = e->cu;
    SstarCabac *cb = &e->cab;
    int n = 1 << log2, i, j, c, x, y;
    bool skip = false;

    /* every TU writes all its levels; only the flags need clearing */
    memset(cu->cbf, 0, sizeof(cu->cbf));
    cu->x0 = x0;
    cu->y0 = y0;
    cu->log2 = log2;
    cu->tlog2 = MIN(log2, e->max_tb_log2);
    cu->k = 1 << (log2 - cu->tlog2);
    cu->intra = e->islice || !e->have_ref;

    /* reconstruct the TUs in z order: intra prediction needs the last */
    for (i = 0; i < cu->k * cu->k; i++) {
        int tx = 0, ty = 0;

        for (j = 0; j < 3; j++) {
            tx |= ((i >> (2 * j)) & 1) << j;
            ty |= ((i >> (2 * j + 1)) & 1) << j;
        }
        for (c = 0; c < 3; c++) {
            vhe_tu_comp(e, cu, c, tx, ty);
        }
    }
    if (!cu->intra) {
        skip = true;
        for (c = 0; c < 3 && skip; c++) {
            skip = !vhe_node_cbf(cu, c, 0, 0, log2 - cu->tlog2);
        }
    }

    if (!e->islice) {
        int l = vhe_avail(e, x0, y0, x0 - 1, y0) &&
                e->skip[blk(e, x0 - 1, y0)];
        int a = vhe_avail(e, x0, y0, x0, y0 - 1) &&
                e->skip[blk(e, x0, y0 - 1)];

        sstar_cabac_bin(cb, skip, CTX_SKIP + l + a);
    }
    if (skip) {
        if (e->max_merge > 1) {
            sstar_cabac_bin(cb, 0, CTX_MERGE_IDX); /* merge_idx 0 */
        }
    } else if (cu->intra) {
        if (!e->islice) {
            sstar_cabac_bin(cb, 1, CTX_PRED_MODE); /* MODE_INTRA */
        }
        if (log2 == e->min_cb_log2) {
            sstar_cabac_bin(cb, 1, CTX_PART_MODE); /* PART_2Nx2N */
        }
        /*
         * Every intra CU here is DC, so are its neighbours' candidates
         * (and unavailable ones count as DC): the MPM list is planar, DC,
         * vertical and DC is mpm_idx 1.
         */
        sstar_cabac_bin(cb, 1, CTX_PREV_INTRA);
        sstar_cabac_bypass(cb, 1);
        sstar_cabac_bypass(cb, 0);
        sstar_cabac_bin(cb, 0, CTX_CHROMA_MODE); /* 4: as luma */
        vhe_tt(e, cu, 0, 0, log2, 0, false, false);
    } else {
        sstar_cabac_bin(cb, 0, CTX_PRED_MODE);  /* MODE_INTER */
        sstar_cabac_bin(cb, 1, CTX_PART_MODE);  /* PART_2Nx2N */
        sstar_cabac_bin(cb, 1, CTX_MERGE_FLAG);
        if (e->max_merge > 1) {
            sstar_cabac_bin(cb, 0, CTX_MERGE_IDX);
        }
        /* rqt_root_cbf is inferred 1 for a 2Nx2N merge CU */
        vhe_tt(e, cu, 0, 0, log2, 0, false, false);
    }

    /* what neighbours and the deblocking filter need to know */
    for (y = y0; y < y0 + n; y += 8) {
        for (x = x0; x < x0 + n; x += 8) {
            int b = blk(e, x, y);
            int tx = (x - x0) >> cu->tlog2, ty = (y - y0) >> cu->tlog2;

            e->depth[b] = depth;
            e->skip[b] = skip;
            e->intra[b] = cu->intra;
            e->nz[b] = cu->cbf[0][ty][tx];
            e->tu[b] = e->tus + ty * cu->k + tx;
            if (cu->intra) {
                e->intra8++;
            } else if (skip) {
                e->skip8++;
            }
        }
    }
    e->tus += cu->k * cu->k;
}

/* coding_quadtree() (7.3.8.4) */
static void vhe_cqt(VheEnc *e, int x0, int y0, int log2, int depth)
{
    int n = 1 << log2;
    bool split;

    if (x0 + n <= e->w && y0 + n <= e->h && log2 > e->min_cb_log2) {
        int l = vhe_avail(e, x0, y0, x0 - 1, y0) &&
                e->depth[blk(e, x0 - 1, y0)] > depth;
        int a = vhe_avail(e, x0, y0, x0, y0 - 1) &&
                e->depth[blk(e, x0, y0 - 1)] > depth;

        split = log2 > e->cu_log2;
        sstar_cabac_bin(&e->cab, split, CTX_SPLIT_CU + l + a);
    } else {
        split = log2 > e->min_cb_log2;
    }
    if (e->cu_qp_delta && log2 >= e->qg_log2) {
        e->qg_coded = false;
    }
    if (split) {
        int h = n / 2;

        vhe_cqt(e, x0, y0, log2 - 1, depth + 1);
        if (x0 + h < e->w) {
            vhe_cqt(e, x0 + h, y0, log2 - 1, depth + 1);
        }
        if (y0 + h < e->h) {
            vhe_cqt(e, x0, y0 + h, log2 - 1, depth + 1);
        }
        if (x0 + h < e->w && y0 + h < e->h) {
            vhe_cqt(e, x0 + h, y0 + h, log2 - 1, depth + 1);
        }
    } else {
        vhe_cu(e, x0, y0, log2, depth);
    }
}

static int ceil_log2(int v)
{
    int n = 0;

    while ((1 << n) < v) {
        n++;
    }
    return n;
}

/* One slice segment NAL: CTBs ctb0 .. ctb1, appended to out. */
static void vhe_slice(VheEnc *e, int ctb0, int ctb1, GByteArray *out)
{
    SstarBits b = { .buf = g_byte_array_new() };
    uint8_t hdr[6] = { 0, 0, 0, 1, e->nal_type << 1, e->tid + 1 };
    int ctb, i, zeros = 0;

    /* slice_segment_header() (7.3.6.1) */
    sstar_bits_put(&b, ctb0 == 0, 1);   /* first_slice_segment_in_pic */
    if (e->nal_type >= 16 && e->nal_type <= 23) {
        sstar_bits_put(&b, 0, 1);       /* no_output_of_prior_pics_flag */
    }
    sstar_bits_ue(&b, 0);               /* slice_pic_parameter_set_id */
    if (ctb0) {
        /* slice_segment_address */
        sstar_bits_put(&b, ctb0, ceil_log2(e->nctb));
    }
    sstar_bits_ue(&b, e->islice ? 2 : 1);   /* slice_type */
    if (e->nal_type != 19 && e->nal_type != 20) {
        sstar_bits_put(&b, e->poc & 0xFF, 8);   /* slice_pic_order_cnt_lsb */
        sstar_bits_put(&b, 1, 1);       /* short_term_ref_pic_set_sps_flag */
        if (e->num_rps > 1) {
            sstar_bits_put(&b, e->rps_id, ceil_log2(e->num_rps));
        }
    }
    if (e->sao) {
        sstar_bits_put(&b, 0, 1);       /* slice_sao_luma_flag */
        sstar_bits_put(&b, 0, 1);       /* slice_sao_chroma_flag */
    }
    if (!e->islice) {
        sstar_bits_put(&b, e->active_override, 1);
        if (e->active_override) {
            sstar_bits_ue(&b, MAX(e->active_l0, 1) - 1);
        }
        sstar_bits_put(&b, e->cabac_init_flag, 1);
        sstar_bits_ue(&b, 5 - e->max_merge);
    }
    sstar_bits_se(&b, e->qp - e->init_qp);  /* slice_qp_delta */
    if (e->dbk_override_en) {
        sstar_bits_put(&b, e->dbk_override, 1);
    }
    if (e->dbk_override) {
        sstar_bits_put(&b, e->dbk_disable, 1);
        if (!e->dbk_disable) {
            sstar_bits_se(&b, e->dbk_beta);
            sstar_bits_se(&b, e->dbk_tc);
        }
    }
    if (!e->dbk_disable) {
        /* slice_loop_filter_across_slices_enabled_flag */
        sstar_bits_put(&b, 1, 1);
    }
    sstar_bits_one_align(&b);           /* byte_alignment() */

    /* slice_segment_data() */
    cabac_start(&e->cab, &b, e->islice ? 0 : e->cabac_init_flag ? 2 : 1,
                e->qp);
    for (ctb = ctb0; ctb <= ctb1; ctb++) {
        vhe_cqt(e, (ctb % e->ctb_w) << e->ctb_log2,
                (ctb / e->ctb_w) << e->ctb_log2, e->ctb_log2, 0);
        sstar_cabac_terminate(&e->cab, ctb == ctb1); /* end_of_slice_segment */
    }
    sstar_cabac_finish(&e->cab);
    sstar_bits_one_align(&b);           /* rbsp_slice_segment_trailing_bits */

    /* the NAL: start code, header, emulation prevention */
    g_byte_array_append(out, hdr, sizeof(hdr));
    for (i = 0; i < b.buf->len; i++) {
        uint8_t v = b.buf->data[i];

        if (zeros >= 2 && v <= 3) {
            uint8_t epb = 3;

            g_byte_array_append(out, &epb, 1);
            zeros = 0;
        }
        g_byte_array_append(out, &v, 1);
        zeros = v ? 0 : zeros + 1;
    }
    g_byte_array_unref(b.buf);
}

/* The luma edge filter (8.7.2.5.3/.6/.7), four lines from q0 of line 0. */
static void vhe_filter_luma(uint8_t *q, int xs, int ys, int beta, int tc)
{
#define P(k, i) q[(i) * ys - ((k) + 1) * xs]
#define Q(k, i) q[(i) * ys + (k) * xs]
    int dp0 = abs(P(2, 0) - 2 * P(1, 0) + P(0, 0));
    int dp3 = abs(P(2, 3) - 2 * P(1, 3) + P(0, 3));
    int dq0 = abs(Q(2, 0) - 2 * Q(1, 0) + Q(0, 0));
    int dq3 = abs(Q(2, 3) - 2 * Q(1, 3) + Q(0, 3));
    int dpq0 = dp0 + dq0, dpq3 = dp3 + dq3, dp = dp0 + dp3, dq = dq0 + dq3;
    bool strong, dep, deq;
    int i;

    if (dpq0 + dpq3 >= beta) {
        return;
    }
    strong = 2 * dpq0 < (beta >> 2) &&
             abs(P(3, 0) - P(0, 0)) + abs(Q(0, 0) - Q(3, 0)) < (beta >> 3) &&
             abs(P(0, 0) - Q(0, 0)) < ((5 * tc + 1) >> 1) &&
             2 * dpq3 < (beta >> 2) &&
             abs(P(3, 3) - P(0, 3)) + abs(Q(0, 3) - Q(3, 3)) < (beta >> 3) &&
             abs(P(0, 3) - Q(0, 3)) < ((5 * tc + 1) >> 1);
    dep = dp < ((beta + (beta >> 1)) >> 3);
    deq = dq < ((beta + (beta >> 1)) >> 3);
    for (i = 0; i < 4; i++) {
        int p0 = P(0, i), p1 = P(1, i), p2 = P(2, i), p3 = P(3, i);
        int q0 = Q(0, i), q1 = Q(1, i), q2 = Q(2, i), q3 = Q(3, i);

        if (strong) {
            int t2 = 2 * tc;

            P(0, i) = MIN(MAX((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3,
                              p0 - t2), p0 + t2);
            P(1, i) = MIN(MAX((p2 + p1 + p0 + q0 + 2) >> 2, p1 - t2), p1 + t2);
            P(2, i) = MIN(MAX((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3,
                              p2 - t2), p2 + t2);
            Q(0, i) = MIN(MAX((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3,
                              q0 - t2), q0 + t2);
            Q(1, i) = MIN(MAX((p0 + q0 + q1 + q2 + 2) >> 2, q1 - t2), q1 + t2);
            Q(2, i) = MIN(MAX((p0 + q0 + q1 + 3 * q2 + 2 * q3 + 4) >> 3,
                              q2 - t2), q2 + t2);
        } else {
            int d = (9 * (q0 - p0) - 3 * (q1 - p1) + 8) >> 4;

            if (abs(d) >= tc * 10) {
                continue;
            }
            d = MIN(MAX(d, -tc), tc);
            P(0, i) = MIN(MAX(p0 + d, 0), 255);
            Q(0, i) = MIN(MAX(q0 - d, 0), 255);
            if (dep) {
                int dd = MIN(MAX((((p2 + p0 + 1) >> 1) - p1 + d) >> 1,
                                 -(tc >> 1)), tc >> 1);

                P(1, i) = MIN(MAX(p1 + dd, 0), 255);
            }
            if (deq) {
                int dd = MIN(MAX((((q2 + q0 + 1) >> 1) - q1 - d) >> 1,
                                 -(tc >> 1)), tc >> 1);

                Q(1, i) = MIN(MAX(q1 + dd, 0), 255);
            }
        }
    }
#undef P
#undef Q
}

/* The chroma edge filter (8.7.2.5.5), four lines. */
static void vhe_filter_chroma(uint8_t *q, int xs, int ys, int tc)
{
    int i;

    for (i = 0; i < 4; i++) {
        uint8_t *s = q + i * ys;
        int p0 = s[-xs], p1 = s[-2 * xs], q0 = s[0], q1 = s[xs];
        int d = MIN(MAX((((q0 - p0) << 2) + p1 - q1 + 4) >> 3, -tc), tc);

        s[-xs] = MIN(MAX(p0 + d, 0), 255);
        s[0] = MIN(MAX(q0 - d, 0), 255);
    }
}

/* bS (8.7.2.4) of the edge between the blocks of luma p and q, 0 if none */
static int vhe_bs(VheEnc *e, int xp, int yp, int xq, int yq)
{
    int bp = blk(e, xp, yp), bq = blk(e, xq, yq);

    if (e->tu[bp] == e->tu[bq]) {
        return 0;                       /* not a transform/prediction edge */
    }
    if (e->intra[bp] || e->intra[bq]) {
        return 2;
    }
    if (e->nz[bp] || e->nz[bq]) {
        return 1;
    }
    return 0;                           /* same reference, same vector */
}

/* The deblocking filter (8.7.2): vertical edges, then horizontal ones. */
static void vhe_deblock(VheEnc *e)
{
    int beta = vhe_beta[MIN(MAX(e->qp + 2 * e->dbk_beta, 0), 51)];
    int tcl[3], tcc[3], dir, x, y, c;

    if (e->dbk_disable) {
        return;
    }
    for (x = 1; x <= 2; x++) {
        tcl[x] = vhe_tc[MIN(MAX(e->qp + 2 * (x - 1) + 2 * e->dbk_tc, 0), 53)];
    }
    for (c = 1; c <= 2; c++) {
        int qpc = vhe_chroma_qp(MAX(e->qp + e->cb_offset, 0));

        tcc[c] = vhe_tc[MIN(MAX(qpc + 2 + 2 * e->dbk_tc, 0), 53)];
    }
    for (dir = 0; dir < 2; dir++) {
        int xs = dir ? e->w : 1, ys = dir ? 1 : e->w;
        int cxs = dir ? e->w / 2 : 1, cys = dir ? 1 : e->w / 2;

        /* dir 0: edges at x = 8k, segments of 4 rows; dir 1 transposed */
        for (y = 0; y < (dir ? e->w : e->h); y += 4) {
            for (x = 8; x < (dir ? e->h : e->w); x += 8) {
                int bs = dir ? vhe_bs(e, y, x - 1, y, x)
                             : vhe_bs(e, x - 1, y, x, y);
                int qx = dir ? y : x, qy = dir ? x : y;

                if (!bs) {
                    continue;
                }
                vhe_filter_luma(e->rec.p[0] + qy * e->w + qx, xs, ys, beta,
                                tcl[bs]);
                /* chroma: bS 2 on its 8x8 grid, 4 lines per 8 luma */
                if (bs == 2 && !(x & 15) && !(y & 7)) {
                    for (c = 1; c <= 2; c++) {
                        vhe_filter_chroma(e->rec.p[c] + (qy / 2) * (e->w / 2) +
                                          qx / 2, cxs, cys, tcc[c]);
                    }
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Pictures in DRAM                                                   */

static bool vhe_dram(SstarVheState *s, uint32_t a, uint64_t len, hwaddr *pa)
{
    uint64_t miu = a;

    /* MIU addresses; a CPU physical address in DRAM is taken as well */
    if (miu >= DRAM_BASE && miu - DRAM_BASE < s->dram_size) {
        miu -= DRAM_BASE;
    }
    if (miu + len > s->dram_size) {
        return false;
    }
    *pa = DRAM_BASE + miu;
    return true;
}

static void vhe_load_source(SstarVheState *s, VheEnc *e)
{
    int fmt = vhe_f(s, F_INPUT_FORMAT), pitch = vhe_f(s, F_ROWLENGTH);
    bool yuyv = fmt == 2, swap = vhe_f(s, F_CHROMA_SWAP);
    int w = e->w, h = e->h, x, y;
    hwaddr ya, ca = 0;
    uint64_t bpl;

    if (!pitch) {
        pitch = w;
    }
    pitch = MAX(pitch, 2);              /* a chroma pair at least */
    bpl = yuyv ? 2 * pitch : pitch;
    if (fmt != 1 && fmt != 2) {
        qemu_log_mask(LOG_UNIMP, "sstar-vhe: input format %d read as NV12\n",
                      fmt);
    }
    if (!vhe_dram(s, s->sw[SW_INPUT_Y], bpl * h, &ya) ||
        (!yuyv && !vhe_dram(s, s->sw[SW_INPUT_C], bpl * h / 2, &ca))) {
        uint8_t px[3];

        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                sstar_pattern_pixel(s->frames, w, h, x, y, px);
                e->src.p[0][y * w + x] = px[0];
                if (!(x & 1) && !(y & 1)) {
                    e->src.p[1][(y / 2) * (w / 2) + x / 2] = px[1];
                    e->src.p[2][(y / 2) * (w / 2) + x / 2] = px[2];
                }
            }
        }
        return;
    }
    {
        g_autofree uint8_t *line = g_malloc(bpl);

        for (y = 0; y < h; y++) {
            address_space_read(&address_space_memory, ya + bpl * y,
                               MEMTXATTRS_UNSPECIFIED, line, bpl);
            for (x = 0; x < w; x++) {
                int xi = MIN(x, pitch - 1);

                e->src.p[0][y * w + x] = yuyv ? line[2 * xi] : line[xi];
            }
            if (yuyv && !(y & 1)) {
                for (x = 0; x < w / 2; x++) {
                    int o = 4 * MIN(x, pitch / 2 - 1);
                    size_t d = (y / 2) * (w / 2) + x;

                    e->src.p[1][d] = line[o + (swap ? 3 : 1)];
                    e->src.p[2][d] = line[o + (swap ? 1 : 3)];
                }
            }
        }
        for (y = 0; !yuyv && y < h / 2; y++) {
            address_space_read(&address_space_memory, ca + bpl * y,
                               MEMTXATTRS_UNSPECIFIED, line, bpl);
            for (x = 0; x < w / 2; x++) {
                int o = 2 * MIN(x, pitch / 2 - 1);

                e->src.p[1][y * (w / 2) + x] = line[o + (swap ? 1 : 0)];
                e->src.p[2][y * (w / 2) + x] = line[o + (swap ? 0 : 1)];
            }
        }
    }
}

/* Reconstruction and reference: NV12, the picture's width as pitch. */
static bool vhe_pic_rw(SstarVheState *s, VhePic *p, int yreg, int creg,
                       bool write)
{
    size_t ylen = (size_t)p->w[0] * p->h[0];
    g_autofree uint8_t *c = g_malloc(ylen / 2);
    hwaddr ya, ca;
    size_t i;

    if (!vhe_dram(s, s->sw[yreg], ylen, &ya) ||
        !vhe_dram(s, s->sw[creg], ylen / 2, &ca)) {
        return false;
    }
    if (write) {
        for (i = 0; i < ylen / 4; i++) {
            c[2 * i] = p->p[1][i];
            c[2 * i + 1] = p->p[2][i];
        }
        address_space_write(&address_space_memory, ya,
                            MEMTXATTRS_UNSPECIFIED, p->p[0], ylen);
        address_space_write(&address_space_memory, ca,
                            MEMTXATTRS_UNSPECIFIED, c, ylen / 2);
    } else {
        address_space_read(&address_space_memory, ya,
                           MEMTXATTRS_UNSPECIFIED, p->p[0], ylen);
        address_space_read(&address_space_memory, ca,
                           MEMTXATTRS_UNSPECIFIED, c, ylen / 2);
        for (i = 0; i < ylen / 4; i++) {
            p->p[1][i] = c[2 * i];
            p->p[2][i] = c[2 * i + 1];
        }
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* The device                                                         */

static void vhe_update(SstarVheState *s)
{
    qemu_set_irq(s->irq, (s->sw[SW_STATUS] & STATUS_IRQ) &&
                         !(s->conf[CONF_IRQ_MASK] & 1));
}

static bool vhe_setup(SstarVheState *s, VheEnc *e)
{
    int qstep;

    e->w = vhe_f(s, F_PIC_WIDTH) * 8;
    e->h = vhe_f(s, F_PIC_HEIGHT) * 8;
    e->ctb_log2 = vhe_f(s, F_MAX_CB) + 3;
    e->min_cb_log2 = vhe_f(s, F_MIN_CB) + 3;
    e->min_tb_log2 = vhe_f(s, F_MIN_TB) + 2;
    e->max_tb_log2 = vhe_f(s, F_MAX_TB) + 2;
    e->depth_intra = vhe_f(s, F_DEPTH_INTRA);
    e->depth_inter = vhe_f(s, F_DEPTH_INTER);
    if (!e->w || !e->h || e->w > DIM_MAX || e->h > DIM_MAX ||
        e->ctb_log2 < 4 || e->min_cb_log2 > e->ctb_log2 ||
        e->max_tb_log2 < 3 || e->max_tb_log2 > 5 ||
        e->min_tb_log2 > e->max_tb_log2) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-vhe: cannot code %dx%d, CTB %d, "
                      "CB %d, TB %d..%d\n", e->w, e->h, 1 << e->ctb_log2,
                      1 << e->min_cb_log2, 1 << e->min_tb_log2,
                      1 << e->max_tb_log2);
        return false;
    }
    e->cu_log2 = MAX(e->min_cb_log2, MIN(4, e->ctb_log2));
    e->ctb_w = DIV_ROUND_UP(e->w, 1 << e->ctb_log2);
    e->ctb_h = DIV_ROUND_UP(e->h, 1 << e->ctb_log2);
    e->nctb = e->ctb_w * e->ctb_h;
    e->islice = vhe_f(s, F_CODING_TYPE) == 1;
    e->nal_type = vhe_f(s, F_NAL_TYPE);
    e->tid = vhe_f(s, F_TEMPORAL_ID);
    e->poc = s->sw[SW_POC];
    e->qp = MIN(vhe_f(s, F_PIC_QP), 51);
    e->init_qp = vhe_f(s, F_INIT_QP);
    e->cb_offset = sext(vhe_f(s, F_CHROMA_QP_OFFSET), 5);
    e->qpc[0] = e->qp;
    e->qpc[1] = e->qpc[2] =
        vhe_chroma_qp(MIN(MAX(e->qp + e->cb_offset, 0), 57));
    e->sao = vhe_f(s, F_SAO);
    e->cabac_init_flag = vhe_f(s, F_CABAC_INIT);
    e->active_override = vhe_f(s, F_ACTIVE_OVERRIDE);
    e->active_l0 = vhe_f(s, F_ACTIVE_L0);
    e->num_rps = vhe_f(s, F_NUM_RPS);
    e->rps_id = vhe_f(s, F_RPS_ID);
    e->dbk_override_en = vhe_f(s, F_PPS_DBK_OVERRIDE);
    e->dbk_override = e->dbk_override_en && vhe_f(s, F_DBK_OVERRIDE);
    e->dbk_disable = vhe_f(s, F_DBK_DISABLE);
    e->dbk_beta = sext(vhe_f(s, F_DBK_BETA), 4);
    e->dbk_tc = sext(vhe_f(s, F_DBK_TC), 4);
    e->cu_qp_delta = vhe_f(s, F_CU_QP_DELTA);
    e->qg_log2 = e->ctb_log2 - vhe_f(s, F_QP_DELTA_DEPTH);
    e->max_merge = 4;
    /* skip a P CU whose residual stays within half a quantizer step */
    qstep = (1 << (e->qp / 6)) * 5 / 8;
    e->skip_thr = MAX(2, qstep / 2);
    if (vhe_f(s, F_SCALING_LIST)) {
        qemu_log_mask(LOG_UNIMP, "sstar-vhe: scaling lists are not "
                      "applied; the reconstruction will drift\n");
    }
    return true;
}

static void vhe_encode(void *opaque)
{
    SstarVheState *s = opaque;
    g_autofree VheEnc *e = g_new0(VheEnc, 1);
    g_autoptr(GByteArray) out = g_byte_array_new();
    uint32_t sizes[DIM_MAX / 16 + 1], nsizes = 0, limit = s->sw[SW_OUTBS_LIMIT];
    uint32_t len = 0, status = STATUS_IRQ | STATUS_FRAME_RDY;
    hwaddr pa;
    int rows, ctb, i;

    if (vhe_setup(s, e)) {
        e->bw = DIV_ROUND_UP(e->w, 8);
        e->bh = DIV_ROUND_UP(e->h, 8);
        e->depth = g_malloc0(e->bw * e->bh);
        e->skip = g_malloc0(e->bw * e->bh);
        e->intra = g_malloc0(e->bw * e->bh);
        e->nz = g_malloc0(e->bw * e->bh);
        e->tu = g_new0(uint32_t, e->bw * e->bh);
        e->slice_of_ctb = g_new0(int, e->nctb);
        e->cu = g_new0(VheCu, 1);
        vhe_pic_alloc(&e->src, e->w, e->h);
        vhe_pic_alloc(&e->rec, e->w, e->h);
        vhe_load_source(s, e);
        if (!e->islice) {
            vhe_pic_alloc(&e->ref, e->w, e->h);
            e->have_ref = vhe_pic_rw(s, &e->ref, SW_L0RP0_Y, SW_L0RP0_C,
                                     false);
            if (!e->have_ref) {
                qemu_log_mask(LOG_GUEST_ERROR, "sstar-vhe: reference "
                              "0x%x outside DRAM, P frame coded intra\n",
                              s->sw[SW_L0RP0_Y]);
            }
        }

        rows = vhe_f(s, F_SLICE_ROWS);
        if (!rows) {
            rows = e->ctb_h;
        }
        for (ctb = 0; ctb < e->nctb; ctb++) {
            e->slice_of_ctb[ctb] = ctb / e->ctb_w / rows;
        }
        for (ctb = 0; ctb < e->nctb; ctb += rows * e->ctb_w) {
            uint32_t before = out->len;

            vhe_slice(e, ctb, MIN(ctb + rows * e->ctb_w, e->nctb) - 1, out);
            if (nsizes < ARRAY_SIZE(sizes) - 1) {
                sizes[nsizes++] = out->len - before;
            }
        }
        vhe_deblock(e);
        vhe_pic_rw(s, &e->rec, SW_RECON_Y, SW_RECON_C, true);

        vhe_pic_free(&e->src);
        vhe_pic_free(&e->rec);
        vhe_pic_free(&e->ref);
        g_free(e->depth);
        g_free(e->skip);
        g_free(e->intra);
        g_free(e->nz);
        g_free(e->tu);
        g_free(e->slice_of_ctb);
        g_free(e->cu);
    }

    len = out->len;
    if (len > limit) {
        len = limit;
        status = STATUS_IRQ | STATUS_BUF_FULL;
    }
    if (len && vhe_dram(s, s->sw[SW_OUTBS_ADDR], len, &pa)) {
        address_space_write(&address_space_memory, pa,
                            MEMTXATTRS_UNSPECIFIED, out->data, len);
    }
    if (vhe_f(s, F_NAL_SIZE_WRITE) &&
        vhe_dram(s, s->sw[SW_SLICE_NALS], 4 * (nsizes + 1), &pa)) {
        sizes[nsizes] = 0;
        for (i = 0; i <= nsizes; i++) {
            sizes[i] = cpu_to_le32(sizes[i]);
        }
        address_space_write(&address_space_memory, pa,
                            MEMTXATTRS_UNSPECIFIED, sizes, 4 * (nsizes + 1));
    }

    /* what _isr_func() and the rate controllers read back */
    s->sw[SW_OUTBS_LIMIT] = len;
    s->sw[SW_CYCLES] = e->nctb * 2000;
    vhe_set_f(s, F_INTRACU8NUM, e->intra8);
    vhe_set_f(s, F_SKIPCU8NUM, e->skip8);
    vhe_set_f(s, F_QP_SUM, e->qp * MAX(e->bw * e->bh, 1));
    vhe_set_f(s, F_QP_NUM, MAX(e->bw * e->bh, 1));
    vhe_set_f(s, F_PIC_COMPLEXITY, MAX(e->bw * e->bh, 1));
    vhe_set_f(s, F_CTBBITS_MIN, 0);
    vhe_set_f(s, F_CTBBITS_MAX, MIN(len * 8 / MAX(e->nctb, 1) * 2, 0xFFFF));
    s->sw[120] = len * 8;               /* TOTALLCUBITS */
    s->sw[SW_STATUS] = (s->sw[SW_STATUS] & ~STATUS_ALL) | status;
    s->sw[SW_ENC] &= ~ENC_E;
    if (s->trace) {
        qemu_log("sstar-vhe: %c POC %d %dx%d QP %d src 0x%x, %u bytes at "
                 "0x%x%s\n", e->islice ? 'I' : 'P', e->poc, e->w, e->h,
                 e->qp, s->sw[SW_INPUT_Y], len, s->sw[SW_OUTBS_ADDR],
                 status & STATUS_BUF_FULL ? " (buffer full)" : "");
    }
    s->frames++;
    vhe_update(s);
}

static uint64_t vhe_conf_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVheState *s = opaque;
    uint16_t v = s->conf[(off >> 2) & (VHE_CONF_REGS - 1)];

    if ((off & 3) >= 2) {
        return 0;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void vhe_conf_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    SstarVheState *s = opaque;
    int reg = (off >> 2) & (VHE_CONF_REGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->conf[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->conf[reg] & 0xFF00) | (val & 0xFF);
    }
    switch (reg) {
    case CONF_RSTZ:
        if (!(v & 1)) {
            /* soft reset: stop the core, drop its status */
            timer_del(s->timer);
            s->sw[SW_STATUS] &= ~STATUS_ALL;
            s->sw[SW_ENC] &= ~ENC_E;
        }
        break;
    case CONF_IRQ_CLR:
        v &= ~0x0F00;
        break;
    }
    s->conf[reg] = v;
    vhe_update(s);
}

static uint64_t vhe_sw_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVheState *s = opaque;
    int i = off >> 3;
    uint16_t v;

    if ((off & 3) >= 2 || i >= VHE_SWREGS) {
        return 0;
    }
    v = off & 4 ? s->sw[i] >> 16 : s->sw[i] & 0xFFFF;
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void vhe_sw_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarVheState *s = opaque;
    int i = off >> 3, sh = off & 4 ? 16 : 0;
    uint32_t mask, old;

    if ((off & 3) >= 2 || i >= VHE_SWREGS ||
        i == SW_PRODUCT || i == SW_CONFIG) {
        return;
    }
    if (size == 1) {
        sh += (off & 1) * 8;
        mask = 0xFFu << sh;
    } else {
        mask = 0xFFFFu << sh;
    }
    old = s->sw[i];
    s->sw[i] = (old & ~mask) | (((uint32_t)val << sh) & mask);
    if (i == SW_ENC) {
        if ((s->sw[i] & ENC_E) && !(old & ENC_E)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ENCODE_NS);
        } else if (!(s->sw[i] & ENC_E) && (old & ENC_E)) {
            timer_del(s->timer);        /* ENC_E cleared: abort */
        }
    }
    vhe_update(s);
}

static const MemoryRegionOps vhe_conf_ops = {
    .read = vhe_conf_read,
    .write = vhe_conf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps vhe_sw_ops = {
    .read = vhe_sw_read,
    .write = vhe_sw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_vhe_reset(DeviceState *dev)
{
    SstarVheState *s = SSTAR_VHE(dev);

    memset(s->conf, 0, sizeof(s->conf));
    memset(s->sw, 0, sizeof(s->sw));
    s->conf[CONF_RSTZ] = 1;
    s->conf[CONF_IRQ_MASK] = 0xF;
    s->sw[SW_PRODUCT] = H2_PRODUCT_ID;
    s->sw[SW_CONFIG] = H2_HW_CONFIG;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void sstar_vhe_init(Object *obj)
{
    SstarVheState *s = SSTAR_VHE(obj);

    memory_region_init_io(&s->conf_mr, obj, &vhe_conf_ops, s,
                          TYPE_SSTAR_VHE "-conf", VHE_CONF_REGS * 4);
    memory_region_init_io(&s->swreg_mr, obj, &vhe_sw_ops, s,
                          TYPE_SSTAR_VHE "-swreg", VHE_SWREGS * 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->conf_mr);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->swreg_mr);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vhe_encode, s);
    vhe_dct_init();
    vhe_diag_init();
}

static const Property sstar_vhe_props[] = {
    DEFINE_PROP_UINT64("dram-size", SstarVheState, dram_size, 64 * MiB),
    DEFINE_PROP_BOOL("trace", SstarVheState, trace, false),
};

static const VMStateDescription vmstate_sstar_vhe = {
    .name = TYPE_SSTAR_VHE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(conf, SstarVheState, VHE_CONF_REGS),
        VMSTATE_UINT32_ARRAY(sw, SstarVheState, VHE_SWREGS),
        VMSTATE_UINT32(frames, SstarVheState),
        VMSTATE_TIMER_PTR(timer, SstarVheState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_vhe_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_vhe_reset);
    device_class_set_props(dc, sstar_vhe_props);
    dc->vmsd = &vmstate_sstar_vhe;
}

static const TypeInfo sstar_vhe_info = {
    .name           = TYPE_SSTAR_VHE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarVheState),
    .instance_init  = sstar_vhe_init,
    .class_init     = sstar_vhe_class_init,
};

static void sstar_vhe_register_types(void)
{
    type_register_static(&sstar_vhe_info);
}

type_init(sstar_vhe_register_types)
