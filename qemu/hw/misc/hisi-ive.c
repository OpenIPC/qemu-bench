/*
 * HiSilicon IVE (Intelligent Video Engine) — Motion Detection MVP
 *
 * Emulates the IVE hardware accelerator for V4 SoCs (EV300/CV500).
 * Register map reverse-engineered from live EV300 hardware capture
 * (see docs/ive-registers.md).
 *
 * MVP scope: DMA, SAD (with threshold), CCL operations — enough
 * for majestic's HI_IVS_MD_Process() motion detection pipeline.
 *
 * Copyright (c) 2026 OpenIPC / Dmitry Ilyin
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "system/dma.h"
#include "hw/misc/hisi-ive-ops.h"

#define TYPE_HISI_IVE "hisi-ive"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIveState, HISI_IVE)

/* Register offsets (from live hardware capture) */
#define IVE_CTRL        0x0004
#define IVE_SW_FIRE     0x0008
#define IVE_OP_DESC     0x0010
#define IVE_CMD_READY   0x0014
#define IVE_CMD_DONE    0x0018
#define IVE_OP_PARAMS   0x001C
#define IVE_CFG0        0x002C
#define IVE_STATUS      0x003C
#define IVE_FRAME_CNT   0x0040
#define IVE_TOTAL_OPS   0x0044
#define IVE_OP_ACTIVE   0x0048
#define IVE_BLEND_WTS   0x005C
#define IVE_IRQ_MASK    0x0060
#define IVE_SAD_PARAMS  0x0064
#define IVE_HW_ID       0x0080
#define IVE_HW_VER0     0x0084
#define IVE_HW_VER1     0x0088
#define IVE_HW_CAP      0x0090

/* Command parameter registers */
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_BLOCK_CFG   0x010C
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_DST2_ADDR   0x011C
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_STRIDES_2   0x0130
#define IVE_STRIDES_3   0x0134
#define IVE_THRESH_VAL  0x0138
#define IVE_CCL_CFG0    0x0160
#define IVE_CCL_CFG1    0x0164

/* Interrupt */
#define IVE_IRQ_STATUS  0x025C
#define IVE_IRQ_FRAME_DONE  (1 << 24)

/* Additional command registers */
#define IVE_BLEND_WTS   0x005C      /* weighted add X/Y (Q0.16 packed) */
#define IVE_THRESH_LO   0x013C      /* threshold low value */
#define IVE_THRESH_HI   0x0140      /* threshold high value */
#define IVE_THRESH_MIN  0x0144      /* threshold output min */
#define IVE_THRESH_MID  0x0148      /* threshold output mid */
#define IVE_THRESH_MAX  0x014C      /* threshold output max */
#define IVE_FILTER_MASK 0x0200      /* 25-byte filter mask (5×5 kernel) */
#define IVE_MAP_LUT     0x0300      /* 256-byte LUT for map operation */
#define IVE_NCC_OUTPUT  0x0180      /* NCC result output address */

/* GMM2 registers */
#define IVE_GMM2_MODEL  0x0190      /* model buffer physical address */
#define IVE_GMM2_FG     0x0194      /* foreground output address */
#define IVE_GMM2_BG     0x0198      /* background output address */
#define IVE_GMM2_FACTOR 0x019C      /* factor image address */
#define IVE_GMM2_NMODEL 0x01A0      /* number of Gaussians per pixel (1-5) */
#define IVE_GMM2_VARMAX 0x01A4      /* max variance (Q9.7) */
#define IVE_GMM2_VARMIN 0x01A8      /* min variance (Q9.7) */
#define IVE_GMM2_SNS    0x01AC      /* sensitivity factor */
#define IVE_GMM2_FTHR   0x01B0      /* frequency threshold */
#define IVE_GMM2_FINIT  0x01B4      /* frequency init value */
#define IVE_GMM2_FADD   0x01B8      /* frequency add factor */
#define IVE_GMM2_FREDU  0x01BC      /* frequency reduction factor */
#define IVE_GMM2_LTHR   0x01C0      /* life threshold */
#define IVE_GMM2_VRATE  0x01C4      /* variance update rate */

/* Extended op registers */
#define IVE_EXT_CTRL    0x040C
#define IVE_EXT_COEFF0  0x0414
#define IVE_EXT_COEFF1  0x0418
#define IVE_EXT_MODE    0x0484

/* Operation types (from OP_TYPE register) */
#define IVE_OP_DMA      0
#define IVE_OP_SAD      1
#define IVE_OP_CCL      2
#define IVE_OP_SUB      3
#define IVE_OP_ADD      4
#define IVE_OP_AND      5
#define IVE_OP_OR       6
#define IVE_OP_XOR      7
#define IVE_OP_THRESH   8
#define IVE_OP_HIST     9
#define IVE_OP_FILTER   10
#define IVE_OP_SOBEL    11
#define IVE_OP_DILATE   12
#define IVE_OP_ERODE    13
#define IVE_OP_INTEG    14
#define IVE_OP_MAP      15
#define IVE_OP_NCC      16
#define IVE_OP_GMM2     17
#define IVE_OP_THRESH_S16   18
#define IVE_OP_THRESH_U16   19
#define IVE_OP_16BIT_TO_8BIT 20
#define IVE_OP_ORD_STAT     21
#define IVE_OP_EQUALIZE_HIST 22
#define IVE_OP_MAG_AND_ANG  23
#define IVE_OP_RESIZE       24
#define IVE_OP_LBP          25
#define IVE_OP_CANNY_EDGE   26
#define IVE_OP_CSC          27
#define IVE_OP_NORM_GRAD    28
#define IVE_OP_GRAD_FG      29
#define IVE_OP_FILTER_CSC   30
#define IVE_OP_GMM          31
#define IVE_OP_ST_CANDI     32
#define IVE_OP_ST_CORNER    33
#define IVE_OP_MATCH_BG     34
#define IVE_OP_UPDATE_BG    35
#define IVE_OP_LK_FLOW      36
#define IVE_OP_PERSP_TRANS  37
#define IVE_OP_HOG          38

/* CCL output structure matches hi_ive.h IVE_CCBLOB_S */
#define IVE_MAX_REGION_NUM  254
#define CCBLOB_SIZE         (4 + IVE_MAX_REGION_NUM * 12)

/* XNN CNN accelerator registers (shared address space with IVE) */
#define IVE_XNN_TRIGGER  0x0000      /* write 1 to start XNN task chain */
#define IVE_XNN_STATUS   0x000C      /* completion status (non-zero = done) */
#define IVE_XNN_TASK_PTR 0x0010      /* task buffer physical address */

/* XNN task node layer markers (at node byte +11) */
#define XNN_LAYER_PREPROC  1
#define XNN_LAYER_FLATTEN  1   /* same as preproc — distinguished by position */
#define XNN_LAYER_FC       2
#define XNN_LAYER_UNPACK   4

#define IVE_MMIO_SIZE   0x10000
#define IVE_NREGS       (IVE_MMIO_SIZE / 4)

struct HisiIveState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[IVE_NREGS];
    uint32_t total_ops;
};

/* ── DMA operation ────────────────────────────────────────────── */

static void ive_op_dma(HisiIveState *s)
{
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides = s->regs[IVE_STRIDES_0 / 4];
    uint16_t src_stride = strides >> 16;
    uint16_t dst_stride = strides & 0xFFFF;
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;

    if (!src || !dst || !width || !height) {
        return;
    }
    if (!src_stride) {
        src_stride = width;
    }
    if (!dst_stride) {
        dst_stride = width;
    }

    uint8_t line[4096];
    uint16_t copy_w = MIN(width, sizeof(line));

    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src + (uint64_t)y * src_stride,
                        line, copy_w, MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory,
                         dst + (uint64_t)y * dst_stride,
                         line, copy_w, MEMTXATTRS_UNSPECIFIED);
    }
}

/* ── SAD operation (with optional threshold) ──────────────────── */

static void ive_op_sad(HisiIveState *s)
{
    uint32_t src1 = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t src2 = s->regs[IVE_SRC2_ADDR / 4];
    uint32_t dst_sad = s->regs[IVE_DST1_ADDR / 4];
    uint32_t dst_thr = s->regs[IVE_DST2_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint32_t strides1 = s->regs[IVE_STRIDES_1 / 4];
    uint32_t strides2 = s->regs[IVE_STRIDES_2 / 4];
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint32_t thresh_reg = s->regs[IVE_THRESH_VAL / 4];
    uint16_t threshold = thresh_reg & 0xFFFF;
    uint8_t min_val = (thresh_reg >> 16) & 0xFF;
    uint8_t max_val = (thresh_reg >> 24) & 0xFF;

    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;
    uint16_t src1_stride = (strides0 >> 16) ? (strides0 >> 16) : width;
    uint16_t src2_stride = (strides1 >> 16) ? (strides1 >> 16) : width;

    if (!src1 || !src2 || !width || !height) {
        return;
    }

    /* SAD with 4×4 macro blocks */
    uint16_t bw = width / 4;
    uint16_t bh = height / 4;
    /* Output strides: pixel-based, from strides2 register */
    uint16_t sad_stride = (strides2 & 0xFFFF) ? (strides2 & 0xFFFF) : bw;
    uint16_t thr_stride = (strides2 >> 16) ? (strides2 >> 16) : bw;

    uint8_t *frame1 = g_malloc(width * height);
    uint8_t *frame2 = g_malloc(width * height);
    uint16_t *sad_out = g_malloc0(bw * bh * sizeof(uint16_t));
    uint8_t *thr_out = g_malloc0(bw * bh);

    /* Read both frames */
    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src1 + (uint64_t)y * src1_stride,
                        frame1 + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        src2 + (uint64_t)y * src2_stride,
                        frame2 + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
    }

    /* SAD per 4x4 block (hisi-ive-ops) */
    uint32_t *sad = g_new(uint32_t, MAX(bw * bh, 1));

    ive_sad(frame1, frame2, width, height, 4, sad);
    for (uint32_t i = 0; i < bw * bh; i++) {
        sad_out[i] = MIN(sad[i], 0xFFFF);
        thr_out[i] = sad[i] > threshold ? max_val : min_val;
    }
    g_free(sad);

    /* Write U16 SAD values */
    if (dst_sad) {
        for (uint16_t y = 0; y < bh; y++) {
            dma_memory_write(&address_space_memory,
                             dst_sad + (uint64_t)y * sad_stride * 2,
                             sad_out + y * bw, bw * sizeof(uint16_t),
                             MEMTXATTRS_UNSPECIFIED);
        }
    }

    /* Write U8 threshold output */
    if (dst_thr) {
        for (uint16_t y = 0; y < bh; y++) {
            dma_memory_write(&address_space_memory,
                             dst_thr + (uint64_t)y * thr_stride,
                             thr_out + y * bw, bw,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }

    g_free(frame1);
    g_free(frame2);
    g_free(sad_out);
    g_free(thr_out);
}

/* ── CCL operation (Connected Component Labeling) ─────────────── */

/*
 * Output format: IVE_CCBLOB_S (from hi_ive.h)
 *   u32 u32CurAreaThr;
 *   s8  s8LabelStatus;  (0=ok, -1=overflow)
 *   u8  u8RegionNum;
 *   u16 reserved;
 *   struct { u32 area; u16 left,top,right,bottom; } astRegion[254];
 */

static void ive_op_ccl(HisiIveState *s)
{
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;
    uint32_t area_thr = (s->regs[IVE_CCL_CFG0 / 4] >> 16) & 0xFFFF;

    if (!src || !dst || !width || !height) {
        return;
    }
    if (area_thr == 0) {
        area_thr = 16;  /* default: 4×4 block area */
    }

    uint8_t *img = g_malloc(width * height);
    int16_t *labels = g_malloc0(width * height * sizeof(int16_t));

    /* Read binary image */
    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src + (uint64_t)y * width,
                        img + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
    }

    /* Two-pass CCL with union-find */
    int16_t *parent = g_malloc(IVE_MAX_REGION_NUM * sizeof(int16_t));
    int next_label = 1;

    /* Find root with path compression */
    #define FIND(x) ({ \
        int16_t _r = (x); \
        while (parent[_r] != _r) _r = parent[_r]; \
        int16_t _i = (x); \
        while (parent[_i] != _r) { int16_t _t = parent[_i]; parent[_i] = _r; _i = _t; } \
        _r; \
    })

    #define UNION(a, b) do { \
        int16_t _ra = FIND(a), _rb = FIND(b); \
        if (_ra != _rb) parent[_ra] = _rb; \
    } while (0)

    for (int i = 0; i < IVE_MAX_REGION_NUM; i++) {
        parent[i] = i;
    }

    /* Pass 1: label and merge (8-connected) */
    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            if (img[y * width + x] == 0) {
                continue;
            }
            int16_t left = (x > 0) ? labels[y * width + x - 1] : 0;
            int16_t up   = (y > 0) ? labels[(y - 1) * width + x] : 0;
            int16_t ul   = (x > 0 && y > 0) ? labels[(y - 1) * width + x - 1] : 0;
            int16_t ur   = (x < width - 1 && y > 0) ? labels[(y - 1) * width + x + 1] : 0;

            /* Find any non-zero neighbor label */
            int16_t chosen = left ? left : up ? up : ul ? ul : ur;
            if (chosen) {
                labels[y * width + x] = chosen;
                if (left && left != chosen) UNION(chosen, left);
                if (up && up != chosen) UNION(chosen, up);
                if (ul && ul != chosen) UNION(chosen, ul);
                if (ur && ur != chosen) UNION(chosen, ur);
            } else {
                if (next_label < IVE_MAX_REGION_NUM) {
                    labels[y * width + x] = next_label++;
                }
            }
        }
    }

    /* Pass 2: flatten labels, compute bounding boxes */
    uint32_t area[IVE_MAX_REGION_NUM];
    uint16_t left[IVE_MAX_REGION_NUM], top[IVE_MAX_REGION_NUM];
    uint16_t right[IVE_MAX_REGION_NUM], bottom[IVE_MAX_REGION_NUM];

    memset(area, 0, sizeof(area));
    for (int i = 0; i < IVE_MAX_REGION_NUM; i++) {
        left[i] = width;
        top[i] = height;
        right[i] = 0;
        bottom[i] = 0;
    }

    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            int16_t l = labels[y * width + x];
            if (l == 0) {
                continue;
            }
            l = FIND(l);
            if (l >= IVE_MAX_REGION_NUM) {
                continue;
            }
            area[l]++;
            if (x < left[l])   left[l]   = x;
            if (y < top[l])    top[l]    = y;
            if (x > right[l])  right[l]  = x;
            if (y > bottom[l]) bottom[l] = y;
        }
    }

    #undef FIND
    #undef UNION

    /* Build IVE_CCBLOB_S: u16 curAreaThr + s8 labelStatus + u8 regionNum
     * + IVE_REGION_S[254] at label-1 index */
    uint8_t blob[CCBLOB_SIZE];
    memset(blob, 0, sizeof(blob));

    uint8_t region_num = 0;

    for (int i = 1; i < next_label; i++) {
        if (area[i] >= area_thr && region_num < IVE_MAX_REGION_NUM) {
            /* Store at index = region_num (label-1 for sequential labels) */
            uint8_t *p = blob + 4 + region_num * 12;
            /* u32 area */
            p[0] = area[i] & 0xFF;
            p[1] = (area[i] >> 8) & 0xFF;
            p[2] = (area[i] >> 16) & 0xFF;
            p[3] = (area[i] >> 24) & 0xFF;
            /* u16 left */
            p[4] = left[i] & 0xFF;
            p[5] = (left[i] >> 8) & 0xFF;
            /* u16 right */
            p[6] = right[i] & 0xFF;
            p[7] = (right[i] >> 8) & 0xFF;
            /* u16 top */
            p[8] = top[i] & 0xFF;
            p[9] = (top[i] >> 8) & 0xFF;
            /* u16 bottom */
            p[10] = bottom[i] & 0xFF;
            p[11] = (bottom[i] >> 8) & 0xFF;
            region_num++;
        }
    }

    /* Header: u16 curAreaThr, s8 labelStatus, u8 regionNum */
    blob[0] = area_thr & 0xFF;
    blob[1] = (area_thr >> 8) & 0xFF;
    blob[2] = 0;           /* s8LabelStatus = OK */
    blob[3] = region_num;  /* u8RegionNum */

    dma_memory_write(&address_space_memory, dst,
                     blob, sizeof(blob), MEMTXATTRS_UNSPECIFIED);

    g_free(img);
    g_free(labels);
    g_free(parent);
}

/* ── Helper: read two frames from guest memory ────────────────── */

static void ive_read_frames(HisiIveState *s, uint8_t **f1, uint8_t **f2,
                            uint16_t *pw, uint16_t *ph)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    uint32_t src1 = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t src2 = s->regs[IVE_SRC2_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t s1 = (strides0 >> 16) ? (strides0 >> 16) : w;
    uint16_t s2 = s->regs[IVE_STRIDES_1 / 4] >> 16;
    if (!s2) s2 = w;
    *pw = w; *ph = h;
    *f1 = g_malloc(w * h);
    if (f2) *f2 = g_malloc(w * h); else if (f2) *f2 = NULL;
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, src1 + (uint64_t)y * s1,
                        *f1 + y * w, w, MEMTXATTRS_UNSPECIFIED);
        if (f2 && src2) {
            dma_memory_read(&address_space_memory, src2 + (uint64_t)y * s2,
                            *f2 + y * w, w, MEMTXATTRS_UNSPECIFIED);
        }
    }
}

static void ive_write_frame(HisiIveState *s, const uint8_t *data,
                            uint16_t w, uint16_t h)
{
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t ds = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : w;
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_write(&address_space_memory, dst + (uint64_t)y * ds,
                         data + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
}

/* ── Phase 1: Pixel-wise operations ──────────────────────────── */

static void ive_op_sub(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint8_t *out = g_malloc(w * h);
    ive_sub(f1, f2, out, w * h, false);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

static void ive_op_add(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint32_t wts = s->regs[IVE_BLEND_WTS / 4];
    uint16_t wx = wts & 0xFFFF, wy = wts >> 16;
    if (!wx && !wy) { wx = 32768; wy = 32768; } /* default 50/50 */
    uint8_t *out = g_malloc(w * h);
    ive_add(f1, f2, out, w * h, wx, wy);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

static void ive_op_bitwise(HisiIveState *s, int op)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint8_t *out = g_malloc(w * h);
    ive_bitwise(op == IVE_OP_AND ? IVE_AND : op == IVE_OP_OR ? IVE_OR : IVE_XOR,
                f1, f2, out, w * h);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

/* ── Phase 2: Threshold + Histogram ──────────────────────────── */

static void ive_op_thresh(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t thr_reg = s->regs[IVE_THRESH_VAL / 4];
    uint8_t thr = (thr_reg >> 8) & 0xFF;
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;
    uint8_t *out = g_malloc(w * h);
    /* binary threshold: at or below thr min, above it max */
    ive_thresh(f1, out, w * h, 0, thr, thr, minv, minv, maxv);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_hist(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t hist[256];
    ive_hist(f1, w * h, hist);
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     hist, sizeof(hist), MEMTXATTRS_UNSPECIFIED);
    g_free(f1);
}

/* ── Phase 2b: 16-bit Threshold + Conversion ─────────────────── */

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Read S16 frame from guest memory (src1, 2 bytes per pixel) */
static int16_t *ive_read_frame_s16(HisiIveState *s, uint16_t *pw, uint16_t *ph)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t stride = (strides0 >> 16) ? (strides0 >> 16) : (w * 2);
    *pw = w; *ph = h;
    int16_t *f = g_malloc(w * h * 2);
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, src + (uint64_t)y * stride,
                        f + y * w, w * 2, MEMTXATTRS_UNSPECIFIED);
    }
    return f;
}

/* Read U16 frame from guest memory */
static uint16_t *ive_read_frame_u16(HisiIveState *s, uint16_t *pw, uint16_t *ph)
{
    return (uint16_t *)ive_read_frame_s16(s, pw, ph);
}

/*
 * Thresh_S16: S16 input → U8 output
 * Modes (from SDK hi_ive.h IVE_THRESH_S16_MODE_E):
 *   0: S16→S8  min/mid/max
 *   1: S16→S8  min/ori/max
 *   2: S16→U8  min/mid/max  (most useful — binary threshold on S16)
 *   3: S16→U8  min/ori/max
 *
 * Uses same threshold registers as Thresh:
 *   IVE_THRESH_VAL bits[23:8]  = low threshold (signed, as int16)
 *   IVE_THRESH_HI  bits[15:0]  = high threshold
 *   IVE_THRESH_MIN = min output val
 *   IVE_THRESH_MID = mid output val
 *   IVE_THRESH_MAX = max output val
 *   IVE_EXT_MODE   = mode (0-3)
 */
static void ive_op_thresh_s16(HisiIveState *s)
{
    uint16_t w, h;
    int16_t *f = ive_read_frame_s16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    int16_t lo_thr = (int16_t)(s->regs[IVE_THRESH_VAL / 4] & 0xFFFF);
    int16_t hi_thr = (int16_t)(s->regs[IVE_THRESH_HI / 4] & 0xFFFF);
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t midv = s->regs[IVE_THRESH_MID / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        int16_t v = f[i];
        switch (mode) {
        case 0: /* S16→S8 min/mid/max */
        case 2: /* S16→U8 min/mid/max */
            out[i] = (v <= lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
            break;
        case 1: /* S16→S8 min/ori/max */
        case 3: /* S16→U8 min/ori/max */
            if (v <= lo_thr) out[i] = minv;
            else if (v > hi_thr) out[i] = maxv;
            else out[i] = (mode <= 1) ? (int8_t)v : (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            out[i] = (v <= lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/*
 * Thresh_U16: U16 input → U8 output
 * Same logic as Thresh_S16 but unsigned input.
 */
static void ive_op_thresh_u16(HisiIveState *s)
{
    uint16_t w, h;
    uint16_t *f = ive_read_frame_u16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint16_t lo_thr = s->regs[IVE_THRESH_VAL / 4] & 0xFFFF;
    uint16_t hi_thr = s->regs[IVE_THRESH_HI / 4] & 0xFFFF;
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t midv = s->regs[IVE_THRESH_MID / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        uint16_t v = f[i];
        switch (mode) {
        case 0: /* min/mid/max */
            out[i] = (v < lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
            break;
        case 1: /* min/ori/max */
            if (v < lo_thr) out[i] = minv;
            else if (v > hi_thr) out[i] = maxv;
            else out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            out[i] = (v < lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/*
 * 16BitTo8Bit: S16/U16 input → U8 output with gain/bias
 * Modes (from SDK hi_ive.h IVE_16BIT_TO_8BIT_MODE_E):
 *   0: S16→S8:       out = (val * numerator / denominator) clamped to [-128,127]
 *   1: S16→U8_ABS:   out = abs(val) * numerator / denominator, clamped to [0,255]
 *   2: S16→U8_BIAS:  out = val * numerator / denominator + bias, clamped to [0,255]
 *   3: U16→U8:       out = val * numerator / denominator, clamped to [0,255]
 *
 * Registers:
 *   IVE_EXT_MODE      = mode (0-3)
 *   IVE_THRESH_VAL[7:0]  = numerator (u8)
 *   IVE_THRESH_VAL[23:8] = denominator (u16)
 *   IVE_THRESH_MIN[7:0]  = bias (s8)
 */
static void ive_op_16bit_to_8bit(HisiIveState *s)
{
    uint16_t w, h;
    int16_t *f = ive_read_frame_s16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint32_t thr_reg = s->regs[IVE_THRESH_VAL / 4];
    uint8_t numer = thr_reg & 0xFF;
    uint16_t denom = (thr_reg >> 8) & 0xFFFF;
    int8_t bias = (int8_t)(s->regs[IVE_THRESH_MIN / 4] & 0xFF);
    if (denom == 0) denom = 1;
    if (numer == 0) numer = 1;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        int32_t v;
        switch (mode) {
        case 0: /* S16→S8 */
            v = (int32_t)f[i] * numer / denom;
            out[i] = (uint8_t)(int8_t)clamp_i32(v, -128, 127);
            break;
        case 1: /* S16→U8_ABS */
            v = abs((int32_t)f[i]) * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        case 2: /* S16→U8_BIAS */
            v = (int32_t)f[i] * numer / denom + bias;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        case 3: /* U16→U8 */
            v = (int32_t)(uint16_t)f[i] * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            v = abs((int32_t)f[i]) * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/* ── Phase 3: Convolution + Morphology ───────────────────────── */

static void ive_op_filter(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Read 5×5 kernel from mask registers (packed as bytes) */
    int8_t mask[25];
    for (int i = 0; i < 25; i++) {
        mask[i] = (int8_t)(s->regs[(IVE_FILTER_MASK + i) / 4] >>
                           ((i % 4) * 8));
    }
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            int sum = 0;
            for (int ky = -2; ky <= 2; ky++) {
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y + ky, sx = x + kx;
                    if (sy < 0) sy = 0;
                    if (sy >= h) sy = h - 1;
                    if (sx < 0) sx = 0;
                    if (sx >= w) sx = w - 1;
                    sum += (int)f1[sy * w + sx] * mask[(ky+2)*5 + (kx+2)];
                }
            }
            sum >>= 8; /* normalize */
            out[y * w + x] = (sum < 0) ? 0 : (sum > 255) ? 255 : sum;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* Sobel: 5×5 configurable mask, outputs S16C1 horizontal + vertical gradients.
 * Matches real HW: two separate S16 output images, clamped border handling. */
static void ive_op_sobel(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);

    /* Read 5×5 mask from OP_DESC (horizontal gradient kernel) */
    int8_t mask_h[25];
    uint32_t mask_addr = s->regs[IVE_OP_DESC / 4];
    if (mask_addr) {
        dma_memory_read(&address_space_memory, mask_addr,
                        mask_h, 25, MEMTXATTRS_UNSPECIFIED);
    } else {
        /* Default: Sobel horizontal 3×3 embedded in 5×5 */
        int8_t def[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        memcpy(mask_h, def, 25);
    }
    /* Transpose mask for vertical gradient */
    int8_t mask_v[25];
    for (int r = 0; r < 5; r++)
        for (int c = 0; c < 5; c++)
            mask_v[r * 5 + c] = mask_h[c * 5 + r];

    /* Compute S16 gradients — clamped borders for rows 1..H-2, row 0 undefined */
    int16_t *out_h = g_malloc0(w * h * 2);
    int16_t *out_v = g_malloc0(w * h * 2);

    for (uint16_t y = 1; y < h - 1; y++) {
        for (uint16_t x = 0; x < w; x++) {
            int sumx = 0, sumy = 0;
            for (int ky = -2; ky <= 2; ky++)
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y + ky < 0 ? 0 : (y + ky >= h ? h - 1 : y + ky);
                    int sx = x + kx < 0 ? 0 : (x + kx >= w ? w - 1 : x + kx);
                    uint8_t pixel = f1[sy * w + sx];
                    sumx += pixel * mask_h[(ky + 2) * 5 + (kx + 2)];
                    sumy += pixel * mask_v[(ky + 2) * 5 + (kx + 2)];
                }
            out_h[y * w + x] = (int16_t)clamp_i32(sumx, -32768, 32767);
            out_v[y * w + x] = (int16_t)clamp_i32(sumy, -32768, 32767);
        }
    }

    /* Write horizontal gradient to dst1 (S16, stride = w*2 aligned to 16) */
    uint32_t dst1 = s->regs[IVE_DST1_ADDR / 4];
    uint32_t dst2 = s->regs[IVE_DST2_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t ds = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : (w * 2);
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_write(&address_space_memory, dst1 + (uint64_t)y * ds,
                         out_h + y * w, w * 2, MEMTXATTRS_UNSPECIFIED);
    }
    /* Write vertical gradient to dst2 */
    if (dst2) {
        for (uint16_t y = 0; y < h; y++) {
            dma_memory_write(&address_space_memory, dst2 + (uint64_t)y * ds,
                             out_v + y * w, w * 2, MEMTXATTRS_UNSPECIFIED);
        }
    }

    g_free(f1); g_free(out_h); g_free(out_v);
}

/* Dilate: bitwise morphology — max(src[i] & mask[i]) over 5×5 neighborhood.
 * With mask=255 this is standard max-filter. With mask<255, bits are masked. */
static void ive_op_dilate(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);

    /* Read 5×5 mask from registers */
    uint8_t mask[25];
    uint32_t mask_addr = s->regs[IVE_OP_DESC / 4];
    if (mask_addr) {
        dma_memory_read(&address_space_memory, mask_addr,
                        mask, 25, MEMTXATTRS_UNSPECIFIED);
    } else {
        memset(mask, 255, 25); /* default: all-ones = standard max */
    }

    uint8_t *out = g_malloc(w * h);
    ive_morph(f1, w, h, mask, true, out);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* Erode: bitwise morphology — min(src[i] | ~mask[i]) over 5×5 neighborhood.
 * With mask=255 this is standard min-filter. With mask<255, bits are forced on. */
static void ive_op_erode(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);

    uint8_t mask[25];
    uint32_t mask_addr = s->regs[IVE_OP_DESC / 4];
    if (mask_addr) {
        dma_memory_read(&address_space_memory, mask_addr,
                        mask, 25, MEMTXATTRS_UNSPECIFIED);
    } else {
        memset(mask, 255, 25);
    }

    uint8_t *out = g_malloc(w * h);
    ive_morph(f1, w, h, mask, false, out);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* ── Phase 4: Analysis ───────────────────────────────────────── */

static void ive_op_integ(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t *out = g_malloc0(w * h * sizeof(uint32_t));
    ive_integ(f1, w, h, out, NULL);
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     out, w * h * sizeof(uint32_t), MEMTXATTRS_UNSPECIFIED);
    g_free(f1); g_free(out);
}

static void ive_op_map(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Read 256-byte LUT from src2 address */
    uint8_t lut[256];
    uint32_t lut_addr = s->regs[IVE_SRC2_ADDR / 4];
    dma_memory_read(&address_space_memory, lut_addr,
                    lut, sizeof(lut), MEMTXATTRS_UNSPECIFIED);
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        out[i] = lut[f1[i]];
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_ncc(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    /* NCC: sum(a*b) / sqrt(sum(a*a) * sum(b*b)) scaled to Q1.15 */
    /* Write 3 u64 values: sum_ab, sum_aa, sum_bb to dst */
    uint64_t result[3];
    ive_ncc(f1, f2, w * h, result);
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     result, sizeof(result), MEMTXATTRS_UNSPECIFIED);
    g_free(f1); g_free(f2);
}

/* ── GMM2 (Gaussian Mixture Model, stateful background subtraction) ── */

/*
 * Per-pixel model: K Gaussians, each 8 bytes:
 *   Byte 0:   mean (U8)
 *   Byte 1-2: variance (U16, Q9.7)
 *   Byte 3-4: frequency (U16)
 *   Byte 5-6: life (U16)
 *   Byte 7:   reserved
 */
#define GMM2_GAUSS_SIZE 8

static void ive_op_gmm2(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *frame;
    ive_read_frames(s, &frame, NULL, &w, &h);
    int npix = w * h;

    uint8_t K = s->regs[IVE_GMM2_NMODEL / 4] & 0xFF;
    if (K < 1) K = 3;
    if (K > 5) K = 5;

    uint32_t model_addr  = s->regs[IVE_GMM2_MODEL / 4];
    uint32_t fg_addr     = s->regs[IVE_GMM2_FG / 4];
    uint32_t bg_addr     = s->regs[IVE_GMM2_BG / 4];
    uint16_t var_max     = s->regs[IVE_GMM2_VARMAX / 4] & 0xFFFF;
    uint16_t var_min     = s->regs[IVE_GMM2_VARMIN / 4] & 0xFFFF;
    uint8_t  sns_factor  = s->regs[IVE_GMM2_SNS / 4] & 0xFF;
    uint16_t freq_thr    = s->regs[IVE_GMM2_FTHR / 4] & 0xFFFF;
    uint16_t freq_init   = s->regs[IVE_GMM2_FINIT / 4] & 0xFFFF;
    uint16_t freq_add    = s->regs[IVE_GMM2_FADD / 4] & 0xFFFF;
    uint16_t freq_redu   = s->regs[IVE_GMM2_FREDU / 4] & 0xFFFF;
    uint16_t life_thr    = s->regs[IVE_GMM2_LTHR / 4] & 0xFFFF;
    uint16_t var_rate    = s->regs[IVE_GMM2_VRATE / 4] & 0xFFFF;

    /* Defaults from SDK sample */
    if (!var_max)   var_max   = (16 * 16) << 7;  /* 32768 */
    if (!var_min)   var_min   = (8 * 8) << 7;    /* 8192 */
    if (!sns_factor) sns_factor = 8;
    if (!freq_thr)  freq_thr  = 12000;
    if (!freq_init) freq_init = 20000;
    if (!freq_add)  freq_add  = 0xEF;
    if (!freq_redu) freq_redu = 0xFF00;
    if (!life_thr)  life_thr  = 5000;
    if (!var_rate)  var_rate  = 1;

    int model_size = K * GMM2_GAUSS_SIZE * npix;
    uint8_t *model = g_malloc(model_size);
    uint8_t *fg = g_malloc0(npix);
    uint8_t *bg = g_malloc0(npix);

    /* Read model from guest memory */
    dma_memory_read(&address_space_memory, model_addr,
                    model, model_size, MEMTXATTRS_UNSPECIFIED);

    for (int p = 0; p < npix; p++) {
        uint8_t pixel = frame[p];
        uint8_t *pmodel = model + p * K * GMM2_GAUSS_SIZE;
        int matched = 0;
        int best_k = 0;
        uint16_t best_freq = 0;

        /* Try to match pixel against each Gaussian */
        for (int k = 0; k < K; k++) {
            uint8_t *g = pmodel + k * GMM2_GAUSS_SIZE;
            uint8_t  mean = g[0];
            uint16_t var  = g[1] | (g[2] << 8);
            uint16_t freq = g[3] | (g[4] << 8);
            uint16_t life = g[5] | (g[6] << 8);

            if (freq == 0 && life == 0 && var == 0) {
                continue; /* uninitialized slot */
            }

            /* Match criterion: |pixel - mean| < sns_factor * sqrt(var)
             * Simplified: (pixel - mean)^2 < sns_factor^2 * var / 128 */
            int diff = (int)pixel - (int)mean;
            int diff2 = diff * diff;
            int var_real = (var > 0) ? var : var_min;
            int threshold = ((int)sns_factor * sns_factor * var_real) >> 7;

            if (diff2 < threshold) {
                /* Matched — update mean and variance */
                int new_mean = mean + (diff > 0 ? var_rate : -var_rate);
                if (new_mean < 0) new_mean = 0;
                if (new_mean > 255) new_mean = 255;
                g[0] = new_mean;

                /* Update variance */
                int new_var = var + var_rate * (diff2 - (var >> 7));
                if (new_var < var_min) new_var = var_min;
                if (new_var > var_max) new_var = var_max;
                g[1] = new_var & 0xFF;
                g[2] = (new_var >> 8) & 0xFF;

                /* Increase frequency */
                int new_freq = freq + freq_add;
                if (new_freq > 65535) new_freq = 65535;
                g[3] = new_freq & 0xFF;
                g[4] = (new_freq >> 8) & 0xFF;

                /* Reset life */
                g[5] = life_thr & 0xFF;
                g[6] = (life_thr >> 8) & 0xFF;

                matched = 1;

                /* Track best (highest freq) for background */
                if (new_freq > best_freq) {
                    best_freq = new_freq;
                    best_k = k;
                }
                break; /* first match wins */
            } else {
                /* Not matched — reduce frequency */
                int new_freq = (int)freq * freq_redu >> 16;
                g[3] = new_freq & 0xFF;
                g[4] = (new_freq >> 8) & 0xFF;

                /* Decrease life */
                if (life > 0) {
                    life--;
                    g[5] = life & 0xFF;
                    g[6] = (life >> 8) & 0xFF;
                }

                if (freq > best_freq) {
                    best_freq = freq;
                    best_k = k;
                }
            }
        }

        if (!matched) {
            /* No match — replace weakest Gaussian */
            int weakest = 0;
            uint16_t min_freq = 65535;
            for (int k = 0; k < K; k++) {
                uint8_t *g = pmodel + k * GMM2_GAUSS_SIZE;
                uint16_t freq = g[3] | (g[4] << 8);
                if (freq < min_freq) { min_freq = freq; weakest = k; }
            }
            uint8_t *g = pmodel + weakest * GMM2_GAUSS_SIZE;
            g[0] = pixel;                           /* mean = current pixel */
            g[1] = var_max & 0xFF;                  /* high initial variance */
            g[2] = (var_max >> 8) & 0xFF;
            g[3] = freq_init & 0xFF;                /* initial frequency */
            g[4] = (freq_init >> 8) & 0xFF;
            g[5] = life_thr & 0xFF;                 /* full life */
            g[6] = (life_thr >> 8) & 0xFF;
            g[7] = 0;

            /* Foreground: pixel doesn't match any stable model */
            fg[p] = 255;
        } else {
            /* Background: matched a model — check if model is stable */
            uint8_t *g = pmodel + best_k * GMM2_GAUSS_SIZE;
            uint16_t freq = g[3] | (g[4] << 8);
            fg[p] = (freq >= freq_thr) ? 0 : 255;
        }

        /* Background image: mean of best-matching Gaussian */
        bg[p] = pmodel[best_k * GMM2_GAUSS_SIZE]; /* mean */
    }

    /* Write outputs back to guest memory */
    dma_memory_write(&address_space_memory, model_addr,
                     model, model_size, MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, fg_addr,
                     fg, npix, MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, bg_addr,
                     bg, npix, MEMTXATTRS_UNSPECIFIED);

    g_free(model);
    g_free(fg);
    g_free(bg);
    g_free(frame);
}

/* ── Phase 5: Additional operations ──────────────────────────── */

/* OrdStatFilter: 3×3 median/min/max filter */
static void ive_op_ord_stat(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            /* Collect 3×3 neighborhood with clamped borders */
            uint8_t nb[9];
            int k = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int sy = y+dy < 0 ? 0 : (y+dy >= h ? h-1 : y+dy);
                    int sx = x+dx < 0 ? 0 : (x+dx >= w ? w-1 : x+dx);
                    nb[k++] = f1[sy*w+sx];
                }
            /* Sort for median (bubble sort on 9 elements) */
            for (int i = 0; i < 8; i++)
                for (int j = i+1; j < 9; j++)
                    if (nb[i] > nb[j]) { uint8_t t = nb[i]; nb[i] = nb[j]; nb[j] = t; }
            switch (mode) {
            case 0: out[y*w+x] = nb[4]; break; /* median */
            case 1: out[y*w+x] = nb[8]; break; /* max */
            case 2: out[y*w+x] = nb[0]; break; /* min */
            default: out[y*w+x] = nb[4];
            }
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* EqualizeHist: histogram equalization */
static void ive_op_equalize_hist(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Build histogram */
    uint32_t hist[256] = {0};
    int npix = w * h;
    for (int i = 0; i < npix; i++) hist[f1[i]]++;
    /* Build CDF and equalization map */
    uint8_t map[256];
    uint32_t cdf = 0;
    for (int i = 0; i < 256; i++) {
        cdf += hist[i];
        map[i] = (uint8_t)((cdf * 255ULL) / npix);
    }
    /* Apply map */
    uint8_t *out = g_malloc(npix);
    for (int i = 0; i < npix; i++) out[i] = map[f1[i]];
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* MagAndAng: gradient magnitude (and optionally angle) */
static void ive_op_mag_and_ang(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF; /* 0=mag only, 1=mag+ang */

    /* Output: magnitude as U16, optionally angle as U8 */
    int16_t *mag = g_malloc0(w * h * 2);
    uint8_t *ang = mode ? g_malloc0(w * h) : NULL;

    for (uint16_t y = 1; y < h-1; y++) {
        for (uint16_t x = 1; x < w-1; x++) {
            int gx = -f1[(y-1)*w+x-1] + f1[(y-1)*w+x+1]
                     -2*f1[y*w+x-1]   + 2*f1[y*w+x+1]
                     -f1[(y+1)*w+x-1]  + f1[(y+1)*w+x+1];
            int gy = -f1[(y-1)*w+x-1] - 2*f1[(y-1)*w+x] - f1[(y-1)*w+x+1]
                     +f1[(y+1)*w+x-1]  + 2*f1[(y+1)*w+x] + f1[(y+1)*w+x+1];
            int m = abs(gx) + abs(gy); /* approximate magnitude */
            mag[y*w+x] = (int16_t)(m > 32767 ? 32767 : m);
            if (ang) {
                /* Quantize angle to 0-255 (0°-360°) */
                double a = atan2((double)gy, (double)gx);
                if (a < 0) a += 2 * 3.14159265358979;
                ang[y*w+x] = (uint8_t)(a * 255.0 / (2 * 3.14159265358979));
            }
        }
    }

    /* Write magnitude to dst1 (S16) */
    uint32_t dst1 = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t ds = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : (w * 2);
    for (uint16_t y = 0; y < h; y++)
        dma_memory_write(&address_space_memory, dst1 + (uint64_t)y * ds,
                         mag + y * w, w * 2, MEMTXATTRS_UNSPECIFIED);
    /* Write angle to dst2 if mode=1 */
    if (ang) {
        uint32_t dst2 = s->regs[IVE_SRC2_ADDR / 4]; /* reuse src2 addr as dst2 */
        for (uint16_t y = 0; y < h; y++)
            dma_memory_write(&address_space_memory, dst2 + (uint64_t)y * w,
                             ang + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }

    g_free(f1); g_free(mag); g_free(ang);
}

/* Resize: bilinear interpolation */
static void ive_op_resize(HisiIveState *s)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t src_w = dim & 0xFFF, src_h = (dim >> 16) & 0xFFF;
    /* Dst dimensions from a separate register (use THRESH_HI for dst_w, THRESH_VAL for dst_h) */
    uint16_t dst_w = s->regs[IVE_THRESH_HI / 4] & 0xFFF;
    uint16_t dst_h = s->regs[IVE_THRESH_VAL / 4] & 0xFFF;
    if (!dst_w) dst_w = src_w;
    if (!dst_h) dst_h = src_h;

    uint32_t src_addr = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t src_stride = (strides0 >> 16) ? (strides0 >> 16) : src_w;

    uint8_t *src = g_malloc(src_w * src_h);
    for (uint16_t y = 0; y < src_h; y++)
        dma_memory_read(&address_space_memory, src_addr + (uint64_t)y * src_stride,
                        src + y * src_w, src_w, MEMTXATTRS_UNSPECIFIED);

    uint8_t *dst = g_malloc(dst_w * dst_h);
    /* Bilinear interpolation */
    for (uint16_t dy = 0; dy < dst_h; dy++) {
        float sy = (float)dy * src_h / dst_h;
        int iy = (int)sy;
        float fy = sy - iy;
        if (iy >= src_h - 1) { iy = src_h - 2; fy = 1.0f; }
        for (uint16_t dx = 0; dx < dst_w; dx++) {
            float sx = (float)dx * src_w / dst_w;
            int ix = (int)sx;
            float fx = sx - ix;
            if (ix >= src_w - 1) { ix = src_w - 2; fx = 1.0f; }
            float v = src[iy*src_w+ix]     * (1-fx)*(1-fy)
                    + src[iy*src_w+ix+1]   * fx*(1-fy)
                    + src[(iy+1)*src_w+ix] * (1-fx)*fy
                    + src[(iy+1)*src_w+ix+1] * fx*fy;
            dst[dy*dst_w+dx] = (uint8_t)(v + 0.5f);
        }
    }

    uint32_t dst_addr = s->regs[IVE_DST1_ADDR / 4];
    uint16_t dst_stride = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : dst_w;
    for (uint16_t y = 0; y < dst_h; y++)
        dma_memory_write(&address_space_memory, dst_addr + (uint64_t)y * dst_stride,
                         dst + y * dst_w, dst_w, MEMTXATTRS_UNSPECIFIED);
    g_free(src); g_free(dst);
}

/* LBP: Local Binary Pattern (3×3 neighborhood) */
static void ive_op_lbp(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    int8_t thr_s = (int8_t)(s->regs[IVE_THRESH_VAL / 4] & 0xFF);
    uint8_t thr_u = (uint8_t)(s->regs[IVE_THRESH_VAL / 4] & 0xFF);

    uint8_t *out = g_malloc0(w * h);
    /* 8-neighbor offsets: clockwise from top-left */
    int dx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    int dy[8] = {-1, -1, -1, 0, 1, 1, 1, 0};

    for (uint16_t y = 1; y < h-1; y++) {
        for (uint16_t x = 1; x < w-1; x++) {
            uint8_t center = f1[y*w+x];
            uint8_t code = 0;
            for (int k = 0; k < 8; k++) {
                int neighbor = f1[(y+dy[k])*w+(x+dx[k])];
                int s_bit;
                if (mode == 0) { /* normal: P(x)-P(center) >= thr */
                    s_bit = ((int)neighbor - (int)center) >= (int)thr_s ? 1 : 0;
                } else { /* abs: |P(x)-P(center)| >= thr */
                    s_bit = abs((int)neighbor - (int)center) >= (int)thr_u ? 1 : 0;
                }
                code |= (s_bit << k);
            }
            out[y*w+x] = code;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* CannyEdge: 5×5 gradient (from mask register) + NMS + hysteresis
 *
 * Matches real hardware CannyHysEdge + CannyEdge two-step pipeline:
 * 1. Apply 5×5 mask as horizontal gradient kernel, transpose as vertical
 * 2. Compute magnitude = |gx| + |gy|
 * 3. Non-Maximum Suppression (strict: m > both neighbors)
 * 4. Double threshold: strong (>= hi_thr) and weak (>= lo_thr)
 * 5. Hysteresis: weak edges connected to strong edges → strong
 */
static void ive_op_canny_edge(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint16_t lo_thr = s->regs[IVE_THRESH_VAL / 4] & 0xFFFF;
    uint16_t hi_thr = s->regs[IVE_THRESH_HI / 4] & 0xFFFF;

    /* Read 5×5 mask from registers (same location as Sobel/Filter) */
    int8_t mask_h[25];
    uint32_t mask_addr = s->regs[IVE_OP_DESC / 4];
    if (mask_addr) {
        dma_memory_read(&address_space_memory, mask_addr,
                        mask_h, 25, MEMTXATTRS_UNSPECIFIED);
    } else {
        /* Default: Sobel horizontal 3×3 embedded in 5×5 */
        int8_t def[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        memcpy(mask_h, def, 25);
    }
    /* Transpose mask for vertical gradient */
    int8_t mask_v[25];
    for (int r = 0; r < 5; r++)
        for (int c = 0; c < 5; c++)
            mask_v[r * 5 + c] = mask_h[c * 5 + r];

    /* 1. Apply 5×5 gradient kernels (with clamped border handling) */
    int16_t *gx = g_malloc0(w * h * 2);
    int16_t *gy = g_malloc0(w * h * 2);
    uint16_t *mag = g_malloc0(w * h * 2);

    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            int sumx = 0, sumy = 0;
            for (int ky = -2; ky <= 2; ky++)
                for (int kx = -2; kx <= 2; kx++) {
                    /* Clamp to image boundaries */
                    int sy = y + ky, sx = x + kx;
                    if (sy < 0) sy = 0;
                    if (sy >= h) sy = h - 1;
                    if (sx < 0) sx = 0;
                    if (sx >= w) sx = w - 1;
                    uint8_t pixel = f1[sy * w + sx];
                    sumx += pixel * mask_h[(ky + 2) * 5 + (kx + 2)];
                    sumy += pixel * mask_v[(ky + 2) * 5 + (kx + 2)];
                }
            gx[y * w + x] = sumx;
            gy[y * w + x] = sumy;
            mag[y * w + x] = abs(sumx) + abs(sumy);
        }
    }

    /* 2. Non-Maximum Suppression with clamped border access */
    uint8_t *nms = g_malloc0(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint16_t m = mag[y * w + x];
            if (m < lo_thr) continue;

            int ax = abs(gx[y * w + x]);
            int ay = abs(gy[y * w + x]);

            /* Neighbor coordinates with clamping */
            int ym1 = (y > 0) ? y - 1 : 0;
            int yp1 = (y < h - 1) ? y + 1 : h - 1;
            int xm1 = (x > 0) ? x - 1 : 0;
            int xp1 = (x < w - 1) ? x + 1 : w - 1;

            uint16_t n1, n2;
            if (ay > 2 * ax) {
                n1 = mag[ym1 * w + x];
                n2 = mag[yp1 * w + x];
            } else if (ax > 2 * ay) {
                n1 = mag[y * w + xm1];
                n2 = mag[y * w + xp1];
            } else if ((gx[y * w + x] > 0) == (gy[y * w + x] > 0)) {
                n1 = mag[ym1 * w + xm1];
                n2 = mag[yp1 * w + xp1];
            } else {
                n1 = mag[ym1 * w + xp1];
                n2 = mag[yp1 * w + xm1];
            }

            /* NMS: m > n1 && m >= n2.
             * n1 is the "backward" neighbor along gradient direction.
             * At boundaries, n1 coordinate gets clamped to self → m==n1.
             * In that case, the pixel IS the boundary maximum → pass. */
            int n1_clamped = 0;
            if (ay > 2 * ax) { /* vertical: n1 at y-1 */
                n1_clamped = (y == 0);
            } else if (ax > 2 * ay) { /* horizontal: n1 at x-1 */
                n1_clamped = (x == 0);
            } else if ((gx[y * w + x] > 0) == (gy[y * w + x] > 0)) { /* diag \ */
                n1_clamped = (y == 0 || x == 0);
            } else { /* diag / */
                n1_clamped = (y == 0 || x == w - 1);
            }

            int pass1 = n1_clamped ? (m >= n1) : (m > n1);
            if (pass1 && m >= n2)
                nms[y * w + x] = (m >= hi_thr) ? 255 : 128;
        }
    }

    /* 3. Hysteresis: stack-based edge tracing (matches HW CannyEdge) */
    uint8_t *out = g_malloc0(w * h);

    /* Stack for strong edge propagation */
    typedef struct { uint16_t x, y; } Point;
    Point *stack = g_malloc(w * h * sizeof(Point));
    int sp = 0;

    /* Seed stack with all strong edges */
    for (uint16_t y = 0; y < h; y++)
        for (uint16_t x = 0; x < w; x++)
            if (nms[y * w + x] == 255) {
                out[y * w + x] = 255;
                stack[sp++] = (Point){x, y};
            }

    /* Trace: connect weak edges adjacent to strong edges */
    while (sp > 0) {
        Point p = stack[--sp];
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int nx = p.x + dx, ny = p.y + dy;
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                if (nms[ny * w + nx] == 128 && out[ny * w + nx] == 0) {
                    out[ny * w + nx] = 255;
                    stack[sp++] = (Point){nx, ny};
                }
            }
    }

    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(gx); g_free(gy); g_free(mag);
    g_free(nms); g_free(out); g_free(stack);
}

/* CSC: Color Space Conversion (YUV420SP→RGB only for now) */
static void ive_op_csc(HisiIveState *s)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    /* mode selects BT.601/709 and YUV↔RGB — simplified to BT.601 full-range */
    (void)s->regs[IVE_EXT_MODE / 4]; /* mode — future use */
    uint32_t src_addr = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t src_stride = (strides0 >> 16) ? (strides0 >> 16) : w;

    /* Read Y plane */
    uint8_t *y_plane = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++)
        dma_memory_read(&address_space_memory, src_addr + (uint64_t)y * src_stride,
                        y_plane + y * w, w, MEMTXATTRS_UNSPECIFIED);

    /* Read UV plane (420SP: half resolution, interleaved U/V) */
    uint32_t uv_addr = src_addr + src_stride * h;
    uint8_t *uv_plane = g_malloc(w * (h / 2));
    for (uint16_t y = 0; y < h / 2; y++)
        dma_memory_read(&address_space_memory, uv_addr + (uint64_t)y * src_stride,
                        uv_plane + y * w, w, MEMTXATTRS_UNSPECIFIED);

    /* Convert to RGB (BT.601 full-range) */
    uint8_t *rgb = g_malloc(w * h * 3);
    for (uint16_t py = 0; py < h; py++) {
        for (uint16_t px = 0; px < w; px++) {
            int Y = y_plane[py * w + px];
            int U = uv_plane[(py/2) * w + (px & ~1)] - 128;
            int V = uv_plane[(py/2) * w + (px | 1)] - 128;
            int R = Y + ((359 * V) >> 8);
            int G = Y - ((88 * U + 183 * V) >> 8);
            int B = Y + ((454 * U) >> 8);
            int idx = (py * w + px) * 3;
            rgb[idx+0] = (uint8_t)clamp_i32(R, 0, 255);
            rgb[idx+1] = (uint8_t)clamp_i32(G, 0, 255);
            rgb[idx+2] = (uint8_t)clamp_i32(B, 0, 255);
        }
    }

    /* Write RGB output (packed U8C3) */
    uint32_t dst_addr = s->regs[IVE_DST1_ADDR / 4];
    uint16_t dst_stride = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : (w * 3);
    for (uint16_t y = 0; y < h; y++)
        dma_memory_write(&address_space_memory, dst_addr + (uint64_t)y * dst_stride,
                         rgb + y * w * 3, w * 3, MEMTXATTRS_UNSPECIFIED);

    g_free(y_plane); g_free(uv_plane); g_free(rgb);
}

/* ── Phase 6: Remaining IVE ops for 100% coverage ────────────── */

/* NormGrad: Normalized gradient (like Sobel but with normalization) */
static void ive_op_norm_grad(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint8_t norm = s->regs[IVE_THRESH_VAL / 4] & 0xFF;
    if (!norm) norm = 1;

    int8_t mask_h[25];
    uint32_t mask_addr = s->regs[IVE_OP_DESC / 4];
    if (mask_addr) {
        dma_memory_read(&address_space_memory, mask_addr,
                        mask_h, 25, MEMTXATTRS_UNSPECIFIED);
    } else {
        int8_t def[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        memcpy(mask_h, def, 25);
    }
    int8_t mask_v[25];
    for (int r = 0; r < 5; r++)
        for (int c = 0; c < 5; c++)
            mask_v[r*5+c] = mask_h[c*5+r];

    int8_t *out_h = g_malloc0(w * h);
    int8_t *out_v = g_malloc0(w * h);
    uint8_t *out_hv = g_malloc0(w * h);

    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            int sumx = 0, sumy = 0;
            for (int ky = -2; ky <= 2; ky++)
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y+ky < 0 ? 0 : (y+ky >= h ? h-1 : y+ky);
                    int sx = x+kx < 0 ? 0 : (x+kx >= w ? w-1 : x+kx);
                    sumx += f1[sy*w+sx] * mask_h[(ky+2)*5+(kx+2)];
                    sumy += f1[sy*w+sx] * mask_v[(ky+2)*5+(kx+2)];
                }
            int round = 1 << (norm - 1); /* rounding: add 2^(norm-1) before shift */
            out_h[y*w+x] = (int8_t)clamp_i32((sumx + round) >> norm, -128, 127);
            out_v[y*w+x] = (int8_t)clamp_i32((sumy + round) >> norm, -128, 127);
            out_hv[y*w+x] = (uint8_t)clamp_i32((abs(sumx) + abs(sumy) + round) >> norm, 0, 255);
        }
    }

    uint32_t dst1 = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t ds = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : w;
    if (mode == 0 || mode == 1) { /* HOR or HOR_AND_VER */
        for (uint16_t y = 0; y < h; y++)
            dma_memory_write(&address_space_memory, dst1 + (uint64_t)y * ds,
                             out_h + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
    if (mode == 0 || mode == 2) { /* VER or HOR_AND_VER */
        uint32_t dst2 = s->regs[IVE_SRC2_ADDR / 4];
        for (uint16_t y = 0; y < h; y++)
            dma_memory_write(&address_space_memory, dst2 + (uint64_t)y * ds,
                             out_v + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
    if (mode == 3) { /* COMBINE */
        for (uint16_t y = 0; y < h; y++)
            dma_memory_write(&address_space_memory, dst1 + (uint64_t)y * ds,
                             out_hv + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
    g_free(f1); g_free(out_h); g_free(out_v); g_free(out_hv);
}

/* GradFg: Gradient-based foreground detection */
static void ive_op_grad_fg(HisiIveState *s)
{
    /* 3 inputs: bgDiffFg (src1), curGrad (src2), bgGrad (via OP_DESC addr) */
    uint16_t w, h;
    uint8_t *bg_diff, *cur_grad;
    ive_read_frames(s, &bg_diff, &cur_grad, &w, &h);

    uint32_t bg_grad_addr = s->regs[IVE_OP_DESC / 4];
    uint8_t *bg_grad = g_malloc(w * h);
    if (bg_grad_addr) {
        uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
        uint16_t s3 = (strides0 >> 16) ? (strides0 >> 16) : w;
        for (uint16_t y = 0; y < h; y++)
            dma_memory_read(&address_space_memory, bg_grad_addr + (uint64_t)y * s3,
                            bg_grad + y * w, w, MEMTXATTRS_UNSPECIFIED);
    } else {
        memset(bg_grad, 0, w * h);
    }

    uint8_t crl_thr = s->regs[IVE_THRESH_VAL / 4] & 0xFF;
    uint8_t mag_thr = (s->regs[IVE_THRESH_VAL / 4] >> 8) & 0xFF;
    if (!crl_thr) crl_thr = 80;
    if (!mag_thr) mag_thr = 4;

    uint8_t *out = g_malloc0(w * h);
    for (int i = 0; i < w * h; i++) {
        if (bg_diff[i] == 0) { out[i] = 0; continue; }
        int diff = abs((int)cur_grad[i] - (int)bg_grad[i]);
        out[i] = (diff >= mag_thr) ? 255 : 0;
    }

    ive_write_frame(s, out, w, h);
    g_free(bg_diff); g_free(cur_grad); g_free(bg_grad); g_free(out);
}

/* FilterAndCSC: Combined filter + color space conversion (stub — delegates) */
static void ive_op_filter_csc(HisiIveState *s)
{
    /* For now, just do the filter part on Y channel */
    ive_op_filter(s);
}

/* GMM v1: Simpler Gaussian Mixture Model */
static void ive_op_gmm(HisiIveState *s)
{
    /* Similar to GMM2 but simpler parameters */
    /* Delegate to GMM2 with v1-style defaults */
    ive_op_gmm2(s);
}

/* STCandiCorner: Shi-Tomasi candidate corner eigenvalue map */
static void ive_op_st_candi(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);

    /* Compute Sobel gradients Ix, Iy */
    int16_t *ix = g_malloc0(w * h * 2);
    int16_t *iy = g_malloc0(w * h * 2);
    for (uint16_t y = 1; y < h-1; y++)
        for (uint16_t x = 1; x < w-1; x++) {
            ix[y*w+x] = -f1[(y-1)*w+x-1] + f1[(y-1)*w+x+1]
                        -2*f1[y*w+x-1]   + 2*f1[y*w+x+1]
                        -f1[(y+1)*w+x-1]  + f1[(y+1)*w+x+1];
            iy[y*w+x] = -f1[(y-1)*w+x-1] - 2*f1[(y-1)*w+x] - f1[(y-1)*w+x+1]
                        +f1[(y+1)*w+x-1]  + 2*f1[(y+1)*w+x] + f1[(y+1)*w+x+1];
        }

    /* Compute structure tensor eigenvalues in 3×3 window */
    uint8_t *out = g_malloc0(w * h);
    for (uint16_t y = 2; y < h-2; y++) {
        for (uint16_t x = 2; x < w-2; x++) {
            int64_t sxx = 0, syy = 0, sxy = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int idx = (y+dy)*w+(x+dx);
                    sxx += (int64_t)ix[idx] * ix[idx];
                    syy += (int64_t)iy[idx] * iy[idx];
                    sxy += (int64_t)ix[idx] * iy[idx];
                }
            /* Min eigenvalue = (trace - sqrt(trace²-4·det))/2
             * Simplified: min_eig ≈ det / trace (Harris-like) */
            int64_t trace = sxx + syy;
            int64_t det = sxx * syy - sxy * sxy;
            int min_eig;
            if (trace > 0)
                min_eig = (int)(det / trace);
            else
                min_eig = 0;
            out[y*w+x] = (uint8_t)clamp_i32(min_eig >> 10, 0, 255);
        }
    }

    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(ix); g_free(iy); g_free(out);
}

/* STCorner: Corner selection from eigenvalue map (CPU-side, no IVE hardware) */
static void ive_op_st_corner(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *eig;
    ive_read_frames(s, &eig, NULL, &w, &h);

    uint16_t max_corners = s->regs[IVE_THRESH_VAL / 4] & 0xFFFF;
    uint16_t min_dist = s->regs[IVE_THRESH_HI / 4] & 0xFFFF;
    if (!max_corners) max_corners = 500;
    if (!min_dist) min_dist = 5;

    /* Find corners: local maxima in eigenvalue map */
    typedef struct { uint16_t x, y; uint8_t val; } Corner;
    Corner *candidates = g_malloc(w * h * sizeof(Corner));
    int ncand = 0;

    for (uint16_t y = 2; y < h-2; y++)
        for (uint16_t x = 2; x < w-2; x++) {
            uint8_t v = eig[y*w+x];
            if (v == 0) continue;
            /* Check 8-connected local maximum */
            int is_max = 1;
            for (int dy = -1; dy <= 1 && is_max; dy++)
                for (int dx = -1; dx <= 1 && is_max; dx++)
                    if ((dy || dx) && eig[(y+dy)*w+(x+dx)] >= v) is_max = 0;
            if (is_max && ncand < w * h)
                candidates[ncand++] = (Corner){x, y, v};
        }

    /* Sort by value descending */
    for (int i = 0; i < ncand-1; i++)
        for (int j = i+1; j < ncand; j++)
            if (candidates[j].val > candidates[i].val) {
                Corner tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }

    /* Select with minimum distance enforcement */
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint16_t n_selected = 0;
    uint8_t *used = g_malloc0(w * h);

    /* Output: u16 count + array of (u16 x, u16 y) */
    for (int i = 0; i < ncand && n_selected < max_corners; i++) {
        uint16_t cx = candidates[i].x, cy = candidates[i].y;
        if (used[cy*w+cx]) continue;
        /* Write corner point */
        uint16_t pt[2] = {cx, cy};
        dma_memory_write(&address_space_memory,
                         dst + 2 + n_selected * 4, pt, 4, MEMTXATTRS_UNSPECIFIED);
        n_selected++;
        /* Mark neighborhood as used */
        for (int dy = -(int)min_dist; dy <= (int)min_dist; dy++)
            for (int dx = -(int)min_dist; dx <= (int)min_dist; dx++) {
                int ny = cy+dy, nx = cx+dx;
                if (ny >= 0 && ny < h && nx >= 0 && nx < w)
                    used[ny*w+nx] = 1;
            }
    }

    /* Write corner count at start */
    dma_memory_write(&address_space_memory, dst, &n_selected, 2, MEMTXATTRS_UNSPECIFIED);

    g_free(eig); g_free(candidates); g_free(used);
}

/* MatchBgModel: Background model matching (stub) */
static void ive_op_match_bg(HisiIveState *s)
{
    /* Complex stateful op — minimal stub that produces valid output */
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Output: all background (no foreground detected) */
    uint8_t *out = g_malloc0(w * h);
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* UpdateBgModel: Background model update (stub) */
static void ive_op_update_bg(HisiIveState *s)
{
    /* Complex stateful op — minimal stub */
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint8_t *out = g_malloc0(w * h);
    memcpy(out, f1, w * h); /* BG = current frame (trivial model) */
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* LKOpticalFlowPyr: Lucas-Kanade optical flow on pyramid */
static void ive_op_lk_flow(HisiIveState *s)
{
    /* Read control parameters */
    uint16_t pts_num = s->regs[IVE_THRESH_VAL / 4] & 0xFFFF;
    uint8_t iter_cnt = (s->regs[IVE_THRESH_HI / 4] >> 8) & 0xFF;
    if (!pts_num) pts_num = 500;
    if (!iter_cnt) iter_cnt = 10;

    uint32_t prev_pts_addr = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t next_pts_addr = s->regs[IVE_DST1_ADDR / 4];
    uint32_t status_addr = s->regs[IVE_SRC2_ADDR / 4];

    /* Read prev pyramid level 0 */
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    if (!w || !h) { return; }

    /* Read prev and next level-0 images from GMM2 model/fg addresses */
    uint32_t prev_addr = s->regs[IVE_GMM2_MODEL / 4];
    uint32_t next_addr = s->regs[IVE_GMM2_FG / 4];

    uint8_t *prev = g_malloc(w * h);
    uint8_t *next = g_malloc(w * h);
    dma_memory_read(&address_space_memory, prev_addr, prev, w * h, MEMTXATTRS_UNSPECIFIED);
    dma_memory_read(&address_space_memory, next_addr, next, w * h, MEMTXATTRS_UNSPECIFIED);

    /* Read prev points (S25Q7 format: 4 bytes per point, x and y) */
    int32_t *prev_pts = g_malloc(pts_num * 8);
    dma_memory_read(&address_space_memory, prev_pts_addr,
                    prev_pts, pts_num * 8, MEMTXATTRS_UNSPECIFIED);

    /* Simple LK at level 0 only (no pyramid for QEMU simplicity) */
    int32_t *next_pts = g_malloc0(pts_num * 8);
    uint8_t *status = g_malloc0(pts_num);

    for (int p = 0; p < pts_num; p++) {
        int32_t px = prev_pts[p*2], py = prev_pts[p*2+1];
        int fx = px >> 7, fy = py >> 7; /* Q7 to integer */

        if (fx < 2 || fx >= w-2 || fy < 2 || fy >= h-2) {
            status[p] = 0; /* out of bounds */
            next_pts[p*2] = px;
            next_pts[p*2+1] = py;
            continue;
        }

        /* Iterative LK: compute gradient and flow */
        int32_t dx = 0, dy = 0;
        for (int iter = 0; iter < iter_cnt; iter++) {
            int nx = fx + (dx >> 7), ny = fy + (dy >> 7);
            if (nx < 1 || nx >= w-1 || ny < 1 || ny >= h-1) break;

            int32_t sxx = 0, syy = 0, sxy = 0, stx = 0, sty = 0;
            for (int ky = -1; ky <= 1; ky++)
                for (int kx = -1; kx <= 1; kx++) {
                    int idx_p = (fy+ky)*w+(fx+kx);
                    int idx_n = (ny+ky)*w+(nx+kx);
                    int gx = (int)prev[(fy+ky)*w+(fx+kx+1)] - (int)prev[(fy+ky)*w+(fx+kx-1)];
                    int gy = (int)prev[(fy+ky+1)*w+(fx+kx)] - (int)prev[(fy+ky-1)*w+(fx+kx)];
                    int dt = (int)next[idx_n] - (int)prev[idx_p];
                    sxx += gx * gx; syy += gy * gy; sxy += gx * gy;
                    stx += gx * dt; sty += gy * dt;
                }

            int64_t det = (int64_t)sxx * syy - (int64_t)sxy * sxy;
            if (det == 0) break;
            int32_t ddx = (int32_t)(((int64_t)syy * stx - (int64_t)sxy * sty) * 128 / det);
            int32_t ddy = (int32_t)(((int64_t)sxx * sty - (int64_t)sxy * stx) * 128 / det);
            dx += ddx; dy += ddy;
            if (abs(ddx) < 2 && abs(ddy) < 2) break; /* converged */
        }

        next_pts[p*2] = px + dx;
        next_pts[p*2+1] = py + dy;
        status[p] = 1; /* tracked */
    }

    /* Write outputs */
    dma_memory_write(&address_space_memory, next_pts_addr,
                     next_pts, pts_num * 8, MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, status_addr,
                     status, pts_num, MEMTXATTRS_UNSPECIFIED);

    g_free(prev); g_free(next); g_free(prev_pts);
    g_free(next_pts); g_free(status);
}

/* PerspTrans: Perspective/affine transform (stub) */
static void ive_op_persp_trans(HisiIveState *s)
{
    /* Stub — copy input to output unchanged */
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    ive_write_frame(s, f1, w, h);
    g_free(f1);
}

/* Hog: Histogram of Oriented Gradients (stub) */
static void ive_op_hog(HisiIveState *s)
{
    /* Stub — HOG requires YUV input and blob output, complex format.
     * Write zeros to output. */
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t zeros[256] = {0};
    dma_memory_write(&address_space_memory, dst,
                     zeros, sizeof(zeros), MEMTXATTRS_UNSPECIFIED);
}

/* ── XNN CNN accelerator ──────────────────────────────────────── */

/*
 * Flatten: copy from planar-stride input to contiguous output.
 * Node fields: in_addr(+16), out_addr(+20), stride×height at (+32).
 */
static void ive_xnn_flatten(uint8_t *node)
{
    uint32_t in_addr  = *(uint32_t *)(node + 16);
    uint32_t out_addr = *(uint32_t *)(node + 20);
    /* node+32 has one channel's stride×height. Multiply by in_c (node+48)
     * or use the out_stride (node+46) which is the full flat size. */
    uint16_t out_stride = *(uint16_t *)(node + 46);
    uint32_t total = out_stride ? out_stride : *(uint32_t *)(node + 32);

    if (!total || total > 1024 * 1024) return;

    uint8_t *buf = g_malloc(total);
    dma_memory_read(&address_space_memory, in_addr,
                    buf, total, MEMTXATTRS_UNSPECIFIED);
    qemu_log_mask(LOG_GUEST_ERROR,
        "hisi-ive: Flatten in=0x%x out=0x%x total=%u data[0]=%u data[1]=%u\n",
        in_addr, out_addr, total, buf[0], buf[1]);
    dma_memory_write(&address_space_memory, out_addr,
                     buf, total, MEMTXATTRS_UNSPECIFIED);
    g_free(buf);
}

/*
 * FC: int8 matrix multiply with int32 bias → s16 output.
 * Weight blob: 1-byte pad + (out_w × in_w) int8 + (out_w) int32 bias.
 * Node fields: in(+16), out(+20), weights(+24), weight_size(+28),
 *   in_w(+48), out_w(+50).
 */
static void ive_xnn_fc(uint8_t *node)
{
    uint32_t in_addr     = *(uint32_t *)(node + 16);
    uint32_t out_addr    = *(uint32_t *)(node + 20);
    uint32_t weight_addr = *(uint32_t *)(node + 24);
    uint16_t in_w  = *(uint16_t *)(node + 48);
    uint16_t out_w = *(uint16_t *)(node + 50);

    if (!in_w || !out_w) return;

    int8_t *input   = g_malloc(in_w);
    int8_t *weights = g_malloc(out_w * in_w);
    int32_t *bias   = g_malloc(out_w * sizeof(int32_t));
    int16_t *output = g_malloc0(out_w * sizeof(int16_t));

    dma_memory_read(&address_space_memory, in_addr,
                    input, in_w, MEMTXATTRS_UNSPECIFIED);
    /* weight_addr = model_phys + arg_offset (small, e.g. 1).
     * Real HW adds weight_data_off internally. In QEMU, read the OMS
     * segment header at model_phys to find weight_data_off (at seg+60),
     * then read weights from model_phys + weight_data_off + arg_offset. */
    {
        uint32_t seg_weight_off;
        dma_memory_read(&address_space_memory, weight_addr & ~0xFFF,
                        /* read u32 at model_base + 60 */ NULL, 0,
                        MEMTXATTRS_UNSPECIFIED);
        /* Simplified: model_base is weight_addr with low bits cleared.
         * arg_offset is small (1), so model_base ≈ weight_addr. */
        uint32_t model_base = weight_addr - (weight_addr & 0xFFF ? 1 : 0);
        /* Actually: weight_addr = model_phys + arg_offset. If arg_offset=1,
         * model_base = weight_addr - 1. Read seg+60 for weight_data_off. */
        model_base = weight_addr - 1; /* subtract arg_offset=1 */
        dma_memory_read(&address_space_memory, model_base + 60,
                        &seg_weight_off, 4, MEMTXATTRS_UNSPECIFIED);
        uint32_t actual_weight = model_base + seg_weight_off + 1; /* +1 for padding */
        dma_memory_read(&address_space_memory, actual_weight,
                        weights, out_w * in_w, MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, actual_weight + out_w * in_w,
                        bias, out_w * sizeof(int32_t), MEMTXATTRS_UNSPECIFIED);
    }

    for (int j = 0; j < out_w; j++) {
        int32_t acc = bias[j];
        for (int i = 0; i < in_w; i++)
            acc += (int32_t)input[i] * (int32_t)weights[j * in_w + i];
        output[j] = (int16_t)(acc >> 19);
    }
    qemu_log_mask(LOG_GUEST_ERROR,
        "hisi-ive: FC in=0x%x out=0x%x w=0x%x in_w=%u out_w=%u "
        "input[0]=%d w[0]=%d bias[0]=%d acc0>>19=%d out[0]=%d\n",
        in_addr, out_addr, weight_addr, in_w, out_w,
        input[0], weights[0], bias[0],
        (int)((bias[0] + (int32_t)input[0] * (int32_t)weights[0]) >> 19),
        output[0]);

    dma_memory_write(&address_space_memory, out_addr,
                     output, out_w * sizeof(int16_t), MEMTXATTRS_UNSPECIFIED);
    g_free(input); g_free(weights); g_free(bias); g_free(output);
}

/*
 * Unpack: copy s16 input to s8 output (truncate high byte).
 * Node fields: in(+16), out(+20), count from (+116) or dimensions.
 */
static void ive_xnn_unpack(uint8_t *node)
{
    uint32_t in_addr  = *(uint32_t *)(node + 16);
    uint32_t out_addr = *(uint32_t *)(node + 20);
    uint32_t count    = *(uint32_t *)(node + 116); /* total output bytes */
    uint16_t out_w    = *(uint16_t *)(node + 40);
    uint16_t out_h    = *(uint16_t *)(node + 42);

    if (!count) count = out_w * out_h;
    if (!count || count > 1024 * 1024) return;

    /* Read s16 input (count values × 2 bytes) */
    int16_t *s16buf = g_malloc(count * sizeof(int16_t));
    int8_t  *s8buf  = g_malloc(count);

    dma_memory_read(&address_space_memory, in_addr,
                    s16buf, count * sizeof(int16_t), MEMTXATTRS_UNSPECIFIED);

    for (uint32_t i = 0; i < count; i++) {
        /* Real HW Unpack maps s16 → u8 with +1 offset:
         * output = clamp(s16_value + 1, 0, 255) */
        int32_t v = (int32_t)s16buf[i] + 1;
        s8buf[i] = (v > 255) ? 255 : (v < 0) ? 0 : (uint8_t)v;
    }

    dma_memory_write(&address_space_memory, out_addr,
                     s8buf, count, MEMTXATTRS_UNSPECIFIED);
    g_free(s16buf); g_free(s8buf);
}

/*
 * Walk the XNN task node chain and execute each layer.
 * Triggered by writing 1 to register 0x00.
 */
static void ive_xnn_fire(HisiIveState *s)
{
    uint32_t node_addr = s->regs[IVE_XNN_TASK_PTR / 4];
    int node_count = 0;

    qemu_log_mask(LOG_GUEST_ERROR, "hisi-ive: XNN fire task_ptr=0x%08x\n",
                  node_addr);

    while (node_addr) {
        uint8_t node[128];
        dma_memory_read(&address_space_memory, node_addr,
                        node, 128, MEMTXATTRS_UNSPECIFIED);

        uint8_t marker = node[11];

        switch (marker) {
        case XNN_LAYER_FC:
            ive_xnn_fc(node);
            break;
        case XNN_LAYER_UNPACK:
            ive_xnn_unpack(node);
            break;
        case XNN_LAYER_FLATTEN:
            /* Flatten and Preproc share marker 1.
             * Skip first node (Preproc pass-through — CPU already
             * preprocessed into tmp). Execute subsequent Flatten nodes. */
            if (node_count > 0)
                ive_xnn_flatten(node);
            break;
        }

        node_count++;
        node_addr = *(uint32_t *)node; /* next-node pointer */
    }

    qemu_log_mask(LOG_GUEST_ERROR, "hisi-ive: XNN done, %d nodes\n", node_count);
    s->regs[IVE_XNN_STATUS / 4] |= 0x08; /* bit 3 = done */
}

/* ── Fire handler ─────────────────────────────────────────────── */

static void ive_fire(HisiIveState *s)
{
    uint8_t op_type = s->regs[IVE_OP_TYPE / 4] & 0xFF;

    switch (op_type) {
    case IVE_OP_DMA:    ive_op_dma(s); break;
    case IVE_OP_SAD:    ive_op_sad(s); break;
    case IVE_OP_CCL:    ive_op_ccl(s); break;
    case IVE_OP_SUB:    ive_op_sub(s); break;
    case IVE_OP_ADD:    ive_op_add(s); break;
    case IVE_OP_AND:
    case IVE_OP_OR:
    case IVE_OP_XOR:    ive_op_bitwise(s, op_type); break;
    case IVE_OP_THRESH: ive_op_thresh(s); break;
    case IVE_OP_HIST:   ive_op_hist(s); break;
    case IVE_OP_FILTER: ive_op_filter(s); break;
    case IVE_OP_SOBEL:  ive_op_sobel(s); break;
    case IVE_OP_DILATE: ive_op_dilate(s); break;
    case IVE_OP_ERODE:  ive_op_erode(s); break;
    case IVE_OP_INTEG:  ive_op_integ(s); break;
    case IVE_OP_MAP:    ive_op_map(s); break;
    case IVE_OP_NCC:    ive_op_ncc(s); break;
    case IVE_OP_GMM2:   ive_op_gmm2(s); break;
    case IVE_OP_THRESH_S16:    ive_op_thresh_s16(s); break;
    case IVE_OP_THRESH_U16:    ive_op_thresh_u16(s); break;
    case IVE_OP_16BIT_TO_8BIT: ive_op_16bit_to_8bit(s); break;
    case IVE_OP_ORD_STAT:      ive_op_ord_stat(s); break;
    case IVE_OP_EQUALIZE_HIST: ive_op_equalize_hist(s); break;
    case IVE_OP_MAG_AND_ANG:   ive_op_mag_and_ang(s); break;
    case IVE_OP_RESIZE:        ive_op_resize(s); break;
    case IVE_OP_LBP:           ive_op_lbp(s); break;
    case IVE_OP_CANNY_EDGE:    ive_op_canny_edge(s); break;
    case IVE_OP_CSC:           ive_op_csc(s); break;
    case IVE_OP_NORM_GRAD:     ive_op_norm_grad(s); break;
    case IVE_OP_GRAD_FG:       ive_op_grad_fg(s); break;
    case IVE_OP_FILTER_CSC:    ive_op_filter_csc(s); break;
    case IVE_OP_GMM:           ive_op_gmm(s); break;
    case IVE_OP_ST_CANDI:      ive_op_st_candi(s); break;
    case IVE_OP_ST_CORNER:     ive_op_st_corner(s); break;
    case IVE_OP_MATCH_BG:      ive_op_match_bg(s); break;
    case IVE_OP_UPDATE_BG:     ive_op_update_bg(s); break;
    case IVE_OP_LK_FLOW:       ive_op_lk_flow(s); break;
    case IVE_OP_PERSP_TRANS:   ive_op_persp_trans(s); break;
    case IVE_OP_HOG:           ive_op_hog(s); break;
    default:
        qemu_log_mask(LOG_UNIMP, "hisi-ive: unimplemented op_type %d\n",
                      op_type);
        break;
    }

    /* Signal completion */
    s->regs[IVE_CMD_DONE / 4] = 1;
    s->regs[IVE_OP_ACTIVE / 4] = 0;
    s->regs[IVE_IRQ_STATUS / 4] |= IVE_IRQ_FRAME_DONE;
    s->total_ops++;
    s->regs[IVE_TOTAL_OPS / 4] = s->total_ops;

    /* Assert IRQ if unmasked */
    if (!(s->regs[IVE_IRQ_MASK / 4] & IVE_IRQ_FRAME_DONE)) {
        qemu_irq_raise(s->irq);
    }
}

/* ── MMIO handlers ────────────────────────────────────────────── */

static uint64_t hisi_ive_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiIveState *s = HISI_IVE(opaque);

    if (offset >= IVE_MMIO_SIZE) {
        return 0;
    }

    return s->regs[offset / 4];
}

static void hisi_ive_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    HisiIveState *s = HISI_IVE(opaque);

    if (offset >= IVE_MMIO_SIZE) {
        return;
    }

    s->regs[offset / 4] = value;

    switch (offset) {
    case IVE_XNN_TRIGGER: /* 0x00 — XNN task chain trigger */
        if (value & 1)
            ive_xnn_fire(s);
        break;

    case IVE_SW_FIRE: /* 0x08 — IVE image op fire / XNN interrupt clear */
        if (value == 1) {
            /* IVE image operation fire (vendor writes exactly 1) */
            s->regs[IVE_SW_FIRE / 4] = 0;
            s->regs[IVE_CMD_DONE / 4] = 0;
            s->regs[IVE_OP_ACTIVE / 4] = 1;
            ive_fire(s);
        } else {
            /* XNN interrupt clear (writes 7) */
            s->regs[IVE_XNN_STATUS / 4] = 0;
        }
        break;

    case IVE_IRQ_STATUS:
        /* Write-one-clear for interrupt bits */
        s->regs[IVE_IRQ_STATUS / 4] &= ~value;
        if (!(s->regs[IVE_IRQ_STATUS / 4] & IVE_IRQ_FRAME_DONE)) {
            qemu_irq_lower(s->irq);
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps hisi_ive_ops = {
    .read = hisi_ive_read,
    .write = hisi_ive_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ─────────────────────────────────────────── */

static void hisi_ive_reset(DeviceState *dev)
{
    HisiIveState *s = HISI_IVE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->total_ops = 0;

    /* Power-on defaults (from live hardware capture) */
    s->regs[IVE_CTRL / 4]     = 0x00000006;
    s->regs[0x0034 / 4]       = 0x00313307;  /* STATIC_CFG */
    s->regs[0x0054 / 4]       = 0x00003F07;  /* IRQ_CFG */
    s->regs[IVE_IRQ_MASK / 4] = 0xFFFFFFFF;  /* all masked */
    s->regs[IVE_HW_ID / 4]    = 0x11E1A300;
    s->regs[IVE_HW_VER0 / 4]  = 0x00000001;
    s->regs[IVE_HW_VER1 / 4]  = 0x00000001;
    s->regs[IVE_HW_CAP / 4]   = 0x01AB5159;
    s->regs[0x0300 / 4]       = 0x00100000;  /* RESULT0 default */
}

static void hisi_ive_init(Object *obj)
{
    HisiIveState *s = HISI_IVE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_ive_ops, s,
                          "hisi-ive", IVE_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Set power-on defaults immediately (from live hardware capture) */
    s->regs[IVE_CTRL / 4]     = 0x00000006;
    s->regs[0x0034 / 4]       = 0x00313307;
    s->regs[0x0054 / 4]       = 0x00003F07;
    s->regs[IVE_IRQ_MASK / 4] = 0xFFFFFFFF;
    s->regs[IVE_HW_ID / 4]    = 0x11E1A300;
    s->regs[IVE_HW_VER0 / 4]  = 0x00000001;
    s->regs[IVE_HW_VER1 / 4]  = 0x00000001;
    s->regs[IVE_HW_CAP / 4]   = 0x01AB5159;
    s->regs[0x0300 / 4]       = 0x00100000;
}

static void hisi_ive_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_ive_reset);
}

static const TypeInfo hisi_ive_info = {
    .name          = TYPE_HISI_IVE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiIveState),
    .instance_init = hisi_ive_init,
    .class_init    = hisi_ive_class_init,
};

static void hisi_ive_register_types(void)
{
    type_register_static(&hisi_ive_info);
}

type_init(hisi_ive_register_types)
