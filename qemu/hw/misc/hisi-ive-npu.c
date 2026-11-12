/*
 * HiSilicon V5 (Hi3516CV608 / CV610) IVE: image ops as SVP NPU tasks
 *
 * On V5 the IVE is not a fixed-function block like V4's (hisi-ive.c).
 * Each op is a 576-byte task the driver (mod_ive.o) builds in DDR and
 * hands to the NPU's hardware task scheduler (HWTS), which runs it on the
 * AI core with an NPU model compiled into ive.ko.  This model does the
 * scheduler's side: four task streams, the push and completion handshake,
 * the interrupts and resets the driver waits on, and computes each task's
 * op in C, by the op code the driver writes at +288 (the NPU models are
 * shared between ops, so they say nothing about which one runs).
 *
 * Registers (from 0x14000000; the driver maps 64 KiB, DT ive@0x14000000
 * SPI 76, sharing the window with npu SPI 75):
 *   0x0100 / 0x0104   AI core interrupt status / clear
 *   0x80a0 bit 10     AI core working
 *   0x8108 bit 0      AI core soft reset, acked at 0x8110 bit 0
 *   0x8200 / 0x82b0   interrupt mask, group 0 / 1 (a set bit masks)
 *   0x8204 / 0x82b4   interrupt status: bit 0, a stream finished tasks
 *   0x8210 / 0x82c0   finished streams, one bit each
 *   0x8220 / 0x82d0   stream stopped, one bit each
 *   0x8228 / 0x82d8   stream pre-stopped, one bit each
 *   0x8280            read: allocate a stream (0xffff: none left)
 *   0x8500 + 0x40 s   stream config: bit 5 picks the interrupt group
 *          + 0x08     task ring base | 1
 *          + 0x10     control: [11:8] tasks pushed, bit 2 pre-stop
 *                     (take no more), bit 1 stop, bit 0 (with the CQE
 *                     count at [15:12]) free
 *          + 0x14     write: clear the stream's finish
 *          + 0x18     SQE [11:0] / CQE [27:16] counts
 *          + 0x20     the last finished task's id [15:0]
 * Group 1 is IVE's (SPI 76); group 0 (SPI 75) the svp_npu driver's.
 *
 * Tasks (mod_ive.o ive_init, ive_fill_npu_*_task): the ring is a list --
 * +6 is the task's id (its slot), +8 the next task's address, the last
 * slot pointing back at the first.  Images are at +64 onwards with their
 * strides at +104.., the op code at +288, width / height at +292 / +296
 * and the op's own fields from +300.  21 ops are modelled -- dma, sad,
 * ccl, add, sub, hist, and, or, xor, the three thresholds, 16bit_to_8bit,
 * map, integ, ncc, dilate, erode, filter, sobel, ord_stat_filter -- their
 * pixel work in hisi-ive-ops.c, shared with V4's hisi-ive; any other op
 * completes and leaves its output alone (LOG_UNIMP).
 *
 * Copyright (c) 2026 OpenIPC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/dma.h"
#include "migration/vmstate.h"
#include "hw/misc/hisi-ive-ops.h"

#define TYPE_HISI_IVE_NPU "hisi-ive-npu"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIveNpuState, HISI_IVE_NPU)

#define NPU_MMIO_SIZE       0x10000

#define AIC_IRQ_STATUS      0x0100
#define AIC_IRQ_CLEAR       0x0104
#define HWTS_AIC_WORKING    0x80a0
#define HWTS_AIC_SRST       0x8108
#define HWTS_AIC_SRST_ACK   0x8110
#define HWTS_ALLOC          0x8280
#define HWTS_GROUP(g)       (0x8200 + (g) * 0xb0)
#define   GRP_MASK          0x00
#define   GRP_STATUS        0x04
#define   GRP_NODE          0x10
#define   GRP_STOPPED       0x20
#define   GRP_PRESTOPPED    0x28
#define HWTS_IRQ_FINISH     (1u << 0)

#define NPU_STREAMS         4
#define SQ(s)               (0x8500 + (s) * 0x40)
#define   SQ_CFG            0x00
#define     SQ_CFG_GROUP    (1u << 5)
#define   SQ_BASE           0x08
#define   SQ_CTRL           0x10
#define     SQ_CTRL_FREE    (1u << 0)
#define     SQ_CTRL_STOP    (1u << 1)
#define     SQ_CTRL_PRESTOP (1u << 2)
#define   SQ_CLEAR          0x14
#define   SQ_COUNT          0x18
#define   SQ_LAST_ID        0x20

#define TASK_SIZE           576
#define TASK_ID             6
#define TASK_NEXT           8
#define TASK_OP             288
#define TASK_W              292
#define TASK_H              296
#define TASK_MAX            512     /* the driver's ring is 20..512 */
#define TASK_NS             (1000 * 1000)   /* a push completes 1 ms on */

/* the op codes ive_fill_npu_*_task write at +288 */
enum {
    OP_DMA = 0, OP_FILTER = 1, OP_SOBEL = 3, OP_DILATE = 5, OP_ERODE = 6,
    OP_THRESH = 7, OP_AND = 8, OP_SUB = 9, OP_OR = 10, OP_INTEG = 11,
    OP_HIST = 12, OP_THRESH_S16 = 13, OP_THRESH_U16 = 14, OP_16TO8 = 15,
    OP_ORD_STAT = 16, OP_MAP = 17, OP_ADD = 18, OP_XOR = 19, OP_NCC = 20,
    OP_CCL = 21, OP_LBP = 23, OP_NORM_GRAD = 24, OP_SAD = 27, OP_RESIZE = 29,
    OP_CSC = 2, OP_MAG_AND_ANG = 4, OP_CANNY = 22, OP_GMM2 = 30,
    OP_ACC = 32,
    OP_IMAGE_DOT = 33,
    OP_ALPHA = 34,
};

#define PIC_MAX             4096
#define RGN_MAX             254     /* OT_IVE_MAX_RGN_NUM */

struct HisiIveNpuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[2];            /* group 0 (svp_npu), group 1 (IVE) */
    QEMUTimer *timer;

    uint32_t regs[NPU_MMIO_SIZE / 4];
    uint32_t alloc;             /* allocated streams, one bit each */
    uint32_t node[2];           /* finished streams per group */
    uint32_t cur[NPU_STREAMS];  /* the next task to run */
    uint32_t pending[NPU_STREAMS];
    uint32_t last_id[NPU_STREAMS];
    int64_t due[NPU_STREAMS];   /* when a stream's pushed tasks run */
    uint32_t ops;               /* tasks run */
    bool trace;
};

static uint32_t task_u32(const uint8_t *t, int off)
{
    return ldl_le_p(t + off);
}

/* An image's low 32 bits of address: DDR sits below 4 GiB */
static uint32_t task_addr(const uint8_t *t, int off)
{
    return ldl_le_p(t + off);
}

static void img_read(uint32_t a, uint32_t stride, uint8_t *buf,
                     uint32_t w, uint32_t h)
{
    for (uint32_t y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, a + (uint64_t)y * stride,
                        buf + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
}

static void img_write(uint32_t a, uint32_t stride, const uint8_t *buf,
                      uint32_t w, uint32_t h)
{
    for (uint32_t y = 0; y < h; y++) {
        dma_memory_write(&address_space_memory, a + (uint64_t)y * stride,
                         buf + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
}

/*
 * DMA (ot_ive_dma_ctrl): +80 source, +104 its stride, +96 / +112 the
 * destination; mode at +308.  Interval copy takes every ver_seg_rows'th
 * row and elem_size bytes of every hor_seg_size (+336 / +340 / +344);
 * the set modes fill the source itself with the value at +320.
 */
static void op_dma(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t src = task_addr(t, 80), sstride = task_u32(t, 104);
    uint32_t dst = task_addr(t, 96), dstride = task_u32(t, 112);
    uint32_t mode = task_u32(t, 308);
    g_autofree uint8_t *row = g_malloc(w);

    switch (mode) {
    case 0:                                     /* direct copy */
        for (uint32_t y = 0; y < h; y++) {
            img_read(src + y * sstride, sstride, row, w, 1);
            img_write(dst + y * dstride, dstride, row, w, 1);
        }
        break;
    case 1: {                                   /* interval copy */
        uint32_t seg = task_u32(t, 336), elem = task_u32(t, 340);
        uint32_t rows = task_u32(t, 344), dy = 0;
        g_autofree uint8_t *out = g_malloc(w);

        /* guest fields: every segment inside the row, no wrapping */
        if (!seg || !elem || elem >= seg || seg > w || !rows || rows > h) {
            break;
        }
        for (uint32_t y = 0; y < h; y += rows, dy++) {
            uint32_t n = 0;

            img_read(src + y * sstride, sstride, row, w, 1);
            for (uint32_t x = 0; x < w && w - x >= elem; x += seg) {
                memcpy(out + n, row + x, elem);
                n += elem;
            }
            img_write(dst + dy * dstride, dstride, out, n, 1);
        }
        break;
    }
    case 2:                                     /* set 3 bytes */
    case 3: {                                   /* set 8 bytes */
        uint64_t val = ldq_le_p(t + 320);
        int n = mode == 2 ? 3 : 8;

        for (uint32_t x = 0; x < w; x++) {
            row[x] = val >> (8 * (x % n));
        }
        for (uint32_t y = 0; y < h; y++) {
            img_write(dst + y * dstride, dstride, row, w, 1);
        }
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "hisi-ive-npu: dma mode %u\n", mode);
        break;
    }
}

/*
 * SAD (ot_ive_sad_ctrl) of two U8 images in w x h blocks: +64 / +72 the
 * sources, +104 / +108 their strides; +88 the first output, +96 the
 * threshold image when both are written, their strides at +116 / +120 in
 * bytes; block mode at +308 (4x4, 8x8, 16x16), output control at +312:
 * 0 / 1 16- / 8-bit SAD and threshold, 2 / 3 SAD alone, 4 threshold alone
 * (then at +88).  Threshold +316: SAD above it gives max_val (+324),
 * else min_val (+320).  8-bit SAD saturates at 255.
 */
static void op_sad(const uint8_t *t, uint32_t w, uint32_t h)
{
    static const int blk[] = { 4, 8, 16 };
    uint32_t mode = task_u32(t, 308), ctrl = task_u32(t, 312);
    uint32_t thr = task_u32(t, 316) & 0xffff;
    uint8_t lo = task_u32(t, 320), hi = task_u32(t, 324);
    int b = mode < ARRAY_SIZE(blk) ? blk[mode] : 4;
    uint32_t bw = w / b, bh = h / b, nb = MAX(bw * bh, 1);
    g_autofree uint8_t *f1 = g_malloc(w * h);
    g_autofree uint8_t *f2 = g_malloc(w * h);
    g_autofree uint32_t *sad = g_new(uint32_t, nb);
    g_autofree uint8_t *sad8 = g_malloc(nb);
    g_autofree uint16_t *sad16 = g_malloc(2 * nb);
    g_autofree uint8_t *thr8 = g_malloc(nb);
    uint32_t out1 = task_addr(t, 88), out2 = task_addr(t, 96);

    if (ctrl > 4 || !bw || !bh) {
        return;
    }
    img_read(task_addr(t, 64), task_u32(t, 104), f1, w, h);
    img_read(task_addr(t, 72), task_u32(t, 108), f2, w, h);
    ive_sad(f1, f2, w, h, b, sad);
    for (uint32_t i = 0; i < bw * bh; i++) {
        sad16[i] = cpu_to_le16(MIN(sad[i], 0xffff));
        sad8[i] = MIN(sad[i], 0xff);
        thr8[i] = sad[i] > thr ? hi : lo;
    }
    if (ctrl == 4) {
        img_write(out1, task_u32(t, 116), thr8, bw, bh);
        return;
    }
    if (ctrl == 0 || ctrl == 2) {
        img_write(out1, task_u32(t, 116), (uint8_t *)sad16, 2 * bw, bh);
    } else {
        img_write(out1, task_u32(t, 116), sad8, bw, bh);
    }
    if (ctrl <= 1) {
        img_write(out2, task_u32(t, 120), thr8, bw, bh);
    }
}

/*
 * CCL (ot_ive_ccl_ctrl) of a U8 binary image, labelled in place: +64 the
 * image, +88 its stride, +80 the ot_ive_ccblob; +300 4- / 8-connected,
 * +304 the initial area threshold, +308 its step.  Regions whose area is
 * below the threshold are dropped, and the threshold rises by step until
 * at most 254 remain.  The blob: a word of cur_area_threshold [22:0],
 * label_status [23], rgn_num [31:24], then 254 regions of area (u32),
 * left, right, top, bottom (u16); region i is label i + 1.
 */
typedef struct {
    uint32_t area;
    uint16_t left, right, top, bottom;
} Rgn;

static int cmp_desc(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;

    return x < y ? 1 : x > y ? -1 : 0;
}

static void op_ccl(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t a = task_addr(t, 64), stride = task_u32(t, 88);
    bool c8 = task_u32(t, 300) == 1;
    uint32_t thr = task_u32(t, 304) & 0xffff;
    uint32_t step = task_u32(t, 308) & 0xffff;
    g_autofree uint8_t *img = g_malloc(w * h);
    g_autofree uint32_t *lab = g_malloc0(sizeof(uint32_t) * w * h);
    g_autofree uint32_t *stack = g_malloc(sizeof(uint32_t) * w * h);
    g_autoptr(GArray) rgn = g_array_new(false, true, sizeof(Rgn));
    g_autofree uint8_t *blob = g_malloc0(4 + RGN_MAX * 12);
    g_autofree uint8_t *map = NULL;
    uint32_t n, kept, i;
    bool failed = false;

    img_read(a, stride, img, w, h);
    for (uint32_t p0 = 0; p0 < w * h; p0++) {
        Rgn r;
        uint32_t sp = 0;

        if (!img[p0] || lab[p0]) {
            continue;
        }
        r = (Rgn){ 0, p0 % w, p0 % w, p0 / w, p0 / w };
        lab[p0] = rgn->len + 1;
        stack[sp++] = p0;
        while (sp) {
            uint32_t p = stack[--sp], x = p % w, y = p / w;

            r.area++;
            r.left = MIN(r.left, x);
            r.right = MAX(r.right, x);
            r.top = MIN(r.top, y);
            r.bottom = MAX(r.bottom, y);
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = x + dx, ny = y + dy;
                    uint32_t q;

                    if ((!dx && !dy) || (!c8 && dx && dy) ||
                        nx < 0 || ny < 0 || nx >= w || ny >= h) {
                        continue;
                    }
                    q = ny * w + nx;
                    if (img[q] && !lab[q]) {
                        lab[q] = rgn->len + 1;
                        stack[sp++] = q;
                    }
                }
            }
        }
        g_array_append_val(rgn, r);
    }
    n = rgn->len;
    for (kept = 0, i = 0; i < n; i++) {
        kept += g_array_index(rgn, Rgn, i).area >= thr;
    }
    if (kept > RGN_MAX) {
        /*
         * The threshold rises by step until at most 254 regions are left:
         * straight to the first value above the 255th largest area
         * (stepping there could take ~2^32 rescans)
         */
        g_autofree uint32_t *area = g_new(uint32_t, n);
        uint64_t need, k;

        for (i = 0; i < n; i++) {
            area[i] = g_array_index(rgn, Rgn, i).area;
        }
        qsort(area, n, sizeof(*area), cmp_desc);
        need = (uint64_t)area[RGN_MAX] + 1;
        if (!step) {
            failed = true;
        } else {
            k = (need - thr + step - 1) / step;
            thr = MIN(thr + k * step, 0x7fffff);
            for (kept = 0, i = 0; i < n; i++) {
                kept += area[i] >= thr;
            }
            failed = kept > RGN_MAX;
        }
    }
    /* the regions that remain, in order, as labels 1..kept */
    map = g_malloc0(n + 1);
    for (kept = 0, i = 0; i < n && kept < RGN_MAX; i++) {
        Rgn *r = &g_array_index(rgn, Rgn, i);
        uint8_t *o = blob + 4 + kept * 12;

        if (r->area < thr) {
            continue;
        }
        stl_le_p(o, r->area);
        stw_le_p(o + 4, r->left);
        stw_le_p(o + 6, r->right);
        stw_le_p(o + 8, r->top);
        stw_le_p(o + 10, r->bottom);
        map[i + 1] = ++kept;
    }
    stl_le_p(blob, (thr & 0x7fffff) | (uint32_t)failed << 23 | kept << 24);
    dma_memory_write(&address_space_memory, task_addr(t, 80), blob,
                     4 + RGN_MAX * 12, MEMTXATTRS_UNSPECIFIED);
    for (i = 0; i < w * h; i++) {
        img[i] = map[lab[i]];
    }
    img_write(a, stride, img, w, h);
}

/* A picture of bpp-byte pixels from +addr with its stride at +stride */
static void *task_img(const uint8_t *t, int addr, int stride, uint32_t w,
                      uint32_t h, int bpp)
{
    uint8_t *buf = g_malloc(w * h * bpp);

    img_read(task_addr(t, addr), task_u32(t, stride), buf, w * bpp, h);
    return buf;
}

static void task_put(const uint8_t *t, int addr, int stride, const void *buf,
                     uint32_t w, uint32_t h, int bpp)
{
    img_write(task_addr(t, addr), task_u32(t, stride), buf, w * bpp, h);
}

/*
 * Two U8 sources into a U8 destination (add, sub, and, or, xor): +80 / +88
 * the sources, +104 the destination, strides at +112 / +116 / +124; add's
 * Q0.16 weights at +300 / +304, sub's mode (|a - b|, or the signed
 * difference halved) at +300
 */
static void op_two(const uint8_t *t, uint32_t op, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *a = task_img(t, 80, 112, w, h, 1);
    g_autofree uint8_t *b = task_img(t, 88, 116, w, h, 1);

    switch (op) {
    case OP_ADD:
        ive_add(a, b, a, w * h, task_u32(t, 300), task_u32(t, 304));
        break;
    case OP_SUB:
        ive_sub(a, b, a, w * h, task_u32(t, 300) == 1);
        break;
    default:
        ive_bitwise(op == OP_AND ? IVE_AND : op == OP_OR ? IVE_OR : IVE_XOR,
                    a, b, a, w * h);
        break;
    }
    task_put(t, 104, 124, a, w, h, 1);
}

/* HIST: the U8 picture at +80 (stride +104) into 256 u32 bins at +96 */
static void op_hist(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *f = task_img(t, 80, 104, w, h, 1);
    uint32_t bins[256];

    ive_hist(f, w * h, bins);
    for (int i = 0; i < 256; i++) {
        bins[i] = cpu_to_le32(bins[i]);
    }
    dma_memory_write(&address_space_memory, task_addr(t, 96), bins,
                     sizeof(bins), MEMTXATTRS_UNSPECIFIED);
}

/*
 * The thresholds (ot_ive_threshold_*_ctrl): +80 the source, +104 its
 * stride in bytes, +96 / +112 the U8 (or S8) destination; mode +300, low
 * / high +304 / +308, min / mid / max +312 / +316 / +320 (sign-extended
 * for the S16-to-S8 modes)
 */
static void op_thresh(const uint8_t *t, uint32_t op, uint32_t w, uint32_t h)
{
    uint32_t mode = task_u32(t, 300), n = w * h;
    int lo = task_u32(t, 304), hi = task_u32(t, 308);
    int min = task_u32(t, 312), mid = task_u32(t, 316), max = task_u32(t, 320);
    g_autofree uint8_t *out = g_malloc(n);

    if (op == OP_THRESH) {
        g_autofree uint8_t *f = task_img(t, 80, 104, w, h, 1);

        ive_thresh(f, out, n, mode, lo, hi, min, mid, max);
    } else {
        g_autofree uint16_t *f = task_img(t, 80, 104, w, h, 2);

        for (uint32_t i = 0; i < n; i++) {
            f[i] = le16_to_cpu(f[i]);
        }
        if (op == OP_THRESH_S16) {
            ive_thresh_s16((int16_t *)f, out, n, mode, lo, hi, min, mid, max);
        } else {
            ive_thresh_u16(f, out, n, mode, lo, hi, min, mid, max);
        }
    }
    task_put(t, 96, 112, out, w, h, 1);
}

/*
 * 16BIT_TO_8BIT (ot_ive_16bit_to_8bit_ctrl): +80 the 16-bit source (stride
 * +104, bytes), +96 / +112 the destination; mode +300, bias +304, and
 * num / den as the Q16 ratio the driver computes, +308
 */
static void op_16to8(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint16_t *f = task_img(t, 80, 104, w, h, 2);
    g_autofree uint8_t *out = g_malloc(w * h);

    for (uint32_t i = 0; i < w * h; i++) {
        f[i] = le16_to_cpu(f[i]);
    }
    ive_16bit_to_8bit(f, out, w * h, task_u32(t, 300), task_u32(t, 308),
                      (int8_t)task_u32(t, 304));
    task_put(t, 96, 112, out, w, h, 1);
}

/*
 * MAP (ot_ive_map_ctrl): +80 the U8 source (+112), +88 the 256-entry table,
 * +104 the destination (+124, bytes); mode +300: U8, S16 or U16 entries
 */
static void op_map(const uint8_t *t, uint32_t w, uint32_t h)
{
    int bpp = task_u32(t, 300) ? 2 : 1;
    g_autofree uint8_t *f = task_img(t, 80, 112, w, h, 1);
    g_autofree uint8_t *out = g_malloc(w * h * bpp);
    uint8_t lut[512];

    dma_memory_read(&address_space_memory, task_addr(t, 88), lut,
                    256 * bpp, MEMTXATTRS_UNSPECIFIED);
    for (uint32_t i = 0; i < w * h; i++) {
        memcpy(out + i * bpp, lut + f[i] * bpp, bpp);
    }
    task_put(t, 104, 124, out, w, h, bpp);
}

/*
 * INTEG (ot_ive_integ_ctrl): +80 the U8 source (+104), +96 the destination
 * (+112, bytes); out_ctrl +300: 1 sums (u32); 2 sums of squares (u64);
 * 0 both in one u64, the sum in [27:0] and the squares above
 */
static void op_integ(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t mode = task_u32(t, 300), n = w * h;
    g_autofree uint8_t *f = task_img(t, 80, 104, w, h, 1);
    g_autofree uint32_t *sum = g_new(uint32_t, n);
    g_autofree uint64_t *sq = g_new(uint64_t, n);

    ive_integ(f, w, h, sum, sq);
    if (mode == 1) {
        for (uint32_t i = 0; i < n; i++) {
            sum[i] = cpu_to_le32(sum[i]);
        }
        task_put(t, 96, 112, sum, w, h, 4);
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint64_t v = mode == 2 ? sq[i] : (sum[i] & 0xfffffff) | sq[i] << 28;

        sq[i] = cpu_to_le64(v);
    }
    task_put(t, 96, 112, sq, w, h, 8);
}

/*
 * NCC: +80 / +88 the U8 sources (+112 / +116), +104 ot_ive_ncc_dst_mem,
 * whose three sums are written; its reserved tail is left alone
 */
static void op_ncc(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *a = task_img(t, 80, 112, w, h, 1);
    g_autofree uint8_t *b = task_img(t, 88, 116, w, h, 1);
    uint64_t r[3];

    ive_ncc(a, b, w * h, r);
    for (int i = 0; i < 3; i++) {
        r[i] = cpu_to_le64(r[i]);
    }
    dma_memory_write(&address_space_memory, task_addr(t, 104), r, sizeof(r),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * DILATE / ERODE (ot_ive_dilate_ctrl): +64 the binary U8 source (+96),
 * +80 the destination (+104); the 5x5 mask as 25 bytes of 0 / 1 at +300
 */
static void op_morph(const uint8_t *t, bool dilate, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *f = task_img(t, 64, 96, w, h, 1);
    g_autofree uint8_t *out = g_malloc(w * h);
    uint8_t mask[25];

    for (int i = 0; i < 25; i++) {
        mask[i] = t[300 + i] & 1 ? 0xff : 0;
    }
    ive_morph(f, w, h, mask, dilate, out);
    task_put(t, 80, 104, out, w, h, 1);
}

/*
 * FILTER (ot_ive_filter_ctrl): +64 the source's Y (+104), +88 the
 * destination's (+116); for a semi-planar picture (+300 non-zero) the UV
 * planes at +72 (+108) and +96 (+120) pass through.  A kernel_w x
 * kernel_h mask (+308 / +312, 3 or 5) of S8 at +332, the sum shifted right
 * by norm (+304) and saturated; edges replicated (+316, constant padding,
 * is taken as replication too).
 */
static void op_filter(const uint8_t *t, uint32_t w, uint32_t h)
{
    int kw = task_u32(t, 308), kh = task_u32(t, 312);
    uint32_t norm = MIN(task_u32(t, 304), 31);   /* guest's: no wild shift */
    g_autofree uint8_t *f = task_img(t, 64, 104, w, h, 1);
    g_autofree int32_t *sum = g_new(int32_t, w * h);

    if (kw != 3 && kw != 5) {
        kw = 5;
    }
    if (kh != 3 && kh != 5) {
        kh = 5;
    }
    ive_correlate(f, w, h, (const int8_t *)t + 332, kw, kh, sum);
    for (uint32_t i = 0; i < w * h; i++) {
        f[i] = MIN(MAX(sum[i] >> norm, 0), 255);
    }
    task_put(t, 88, 116, f, w, h, 1);
    if (task_u32(t, 300)) {
        g_autofree uint8_t *uv = task_img(t, 72, 108, w, h / 2, 1);

        task_put(t, 96, 120, uv, w, h / 2, 1);
    }
}

/*
 * SOBEL (ot_ive_sobel_ctrl): +64 the U8 source (+96); the 5x5 S8 mask at
 * +300 gives the horizontal gradient, its transpose the vertical, as S16;
 * out_ctrl +328: 0 both (+80 / +88, strides +104 / +108), 1 horizontal or
 * 2 vertical alone at +80 (+104).  Edges replicated.
 */
static void op_sobel(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t ctrl = task_u32(t, 328), n = w * h;
    g_autofree uint8_t *f = task_img(t, 64, 96, w, h, 1);
    g_autofree int32_t *g = g_new(int32_t, n);
    g_autofree int16_t *out = g_new(int16_t, n);
    int8_t mask[2][25];

    memcpy(mask[0], t + 300, 25);
    for (int k = 0; k < 25; k++) {
        mask[1][k] = mask[0][(k % 5) * 5 + k / 5];
    }
    for (int dir = 0; dir < 2; dir++) {
        int slot;

        if ((ctrl == 1 && dir) || (ctrl == 2 && !dir) || ctrl > 2) {
            continue;
        }
        slot = ctrl == 0 && dir ? 1 : 0;
        ive_correlate(f, w, h, mask[dir], 5, 5, g);
        for (uint32_t i = 0; i < n; i++) {
            out[i] = cpu_to_le16(MIN(MAX(g[i], INT16_MIN), INT16_MAX));
        }
        task_put(t, 80 + 8 * slot, 104 + 4 * slot, out, w, h, 2);
    }
}

/*
 * ORD_STAT_FILTER: +80 the U8 source (+104), +96 the destination (+112);
 * mode +300: 3x3 median, max or min
 */
static void op_ord_stat(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *f = task_img(t, 80, 104, w, h, 1);
    g_autofree uint8_t *out = g_malloc(w * h);

    ive_ord_stat(f, w, h, task_u32(t, 300), out);
    task_put(t, 96, 112, out, w, h, 1);
}

/*
 * MAG_AND_ANG (ot_ive_mag_and_ang_ctrl): +64 the U8 source (+96); the
 * magnitude as U16 at +80 (+104, bytes) and, when out_ctrl (+328) is 1,
 * the angle at +88 (+108).  The 5x5 S8 mask is at +300, the threshold at
 * +332 and the mask's range code -- which picks the engine's internal
 * scale, and is 0 for the Sobel masks the SDK uses -- at +336.
 */
static void op_mag_and_ang(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t n = w * h, ctrl = task_u32(t, 328);
    g_autofree uint8_t *f = task_img(t, 64, 96, w, h, 1);
    g_autofree uint16_t *mag = g_new(uint16_t, n);
    g_autofree uint8_t *ang = ctrl == 1 ? g_malloc(n) : NULL;

    ive_mag_and_ang(f, w, h, (const int8_t *)t + 300, task_u32(t, 332),
                    mag, ang);
    for (uint32_t i = 0; i < n; i++) {
        mag[i] = cpu_to_le16(mag[i]);
    }
    task_put(t, 80, 104, mag, w, h, 2);
    if (ang) {
        task_put(t, 88, 108, ang, w, h, 1);
    }
}

/*
 * NORM_GRAD (ot_ive_norm_grad_ctrl): +64 the U8 source (+96), the 5x5 S8
 * mask at +300, norm at +336 and out_ctrl at +328: 0 both gradients as S8
 * (+80 / +104 and +88 / +108), 1 the horizontal or 2 the vertical alone
 * at +80, 3 both interleaved as S8C2 at +80 with a doubled stride
 */
static void op_norm_grad(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t n = w * h, ctrl = task_u32(t, 328);
    g_autofree uint8_t *f = task_img(t, 64, 96, w, h, 1);
    g_autofree int8_t *gh = g_new(int8_t, n);
    g_autofree int8_t *gv = g_new(int8_t, n);

    if (ctrl > 3) {
        return;
    }
    ive_norm_grad(f, w, h, (const int8_t *)t + 300, task_u32(t, 336),
                  ctrl == 2 ? NULL : gh, ctrl == 1 ? NULL : gv);
    if (ctrl == 3) {
        g_autofree int8_t *pack = g_new(int8_t, 2 * n);

        for (uint32_t i = 0; i < n; i++) {
            pack[2 * i] = gh[i];
            pack[2 * i + 1] = gv[i];
        }
        task_put(t, 80, 104, pack, w, h, 2);
        return;
    }
    task_put(t, 80, 104, ctrl == 2 ? gv : gh, w, h, 1);
    if (!ctrl) {
        task_put(t, 88, 108, gv, w, h, 1);
    }
}

/*
 * LBP (ot_ive_lbp_ctrl): +80 the U8 source (+104), +96 the destination
 * (+112); mode at +300 and the threshold at +304, signed in normal mode
 */
static void op_lbp(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t mode = task_u32(t, 300);
    g_autofree uint8_t *f = task_img(t, 80, 104, w, h, 1);
    g_autofree uint8_t *out = g_malloc(w * h);
    int thr = mode ? (uint8_t)task_u32(t, 304) : (int8_t)task_u32(t, 304);

    ive_lbp(f, w, h, mode, thr, out);
    task_put(t, 96, 112, out, w, h, 1);
}

/*
 * ACC (+80 / +88 the sources, +104 the destination, strides +112 / +116 /
 * +124, mode +300), IMAGE_DOT (the same with S32 pictures, so the strides
 * are in bytes already) and ALPHA_BLENDING (+80 / +88 / +96 the two
 * sources and the alpha, +112 the destination, strides +120 / +124 /
 * +128 / +136)
 */
static void op_acc(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *a = task_img(t, 80, 112, w, h, 1);
    g_autofree uint8_t *b = task_img(t, 88, 116, w, h, 1);

    ive_acc(a, b, a, w * h, task_u32(t, 300));
    task_put(t, 104, 124, a, w, h, 1);
}

static void op_image_dot(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t n = w * h;
    g_autofree int32_t *a = task_img(t, 80, 112, w, h, 4);
    g_autofree int32_t *b = task_img(t, 88, 116, w, h, 4);

    for (uint32_t i = 0; i < n; i++) {
        a[i] = le32_to_cpu(a[i]);
        b[i] = le32_to_cpu(b[i]);
    }
    ive_image_dot(a, b, a, n);
    for (uint32_t i = 0; i < n; i++) {
        a[i] = cpu_to_le32(a[i]);
    }
    task_put(t, 104, 124, a, w, h, 4);
}

static void op_alpha_blend(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *a = task_img(t, 80, 120, w, h, 1);
    g_autofree uint8_t *b = task_img(t, 88, 124, w, h, 1);
    g_autofree uint8_t *al = task_img(t, 96, 128, w, h, 1);

    ive_alpha_blend(a, b, al, a, w * h);
    task_put(t, 112, 136, a, w, h, 1);
}

/*
 * CSC (ot_ive_csc_ctrl): the source's format code at +300 and the
 * destination's at +304 (1 / 0 YUV420SP, 2 / 1 YUV422SP, 3 / 2 packed
 * RGB, 4 / 3 planar RGB), the mode at +308.  The source planes are at
 * +80 / +88 / +96 with their strides at +136 / +140 / +144, the
 * destination's at +112 / +120 / +128 and +152 / +156 / +160.
 */
static void op_csc(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t sfmt = task_u32(t, 300), dfmt = task_u32(t, 304);
    uint32_t mode = task_u32(t, 308), n = w * h;
    bool to_rgb = mode < 4;

    /*
     * The format codes are the source's 1 / 2 / 3 / 4 and the
     * destination's 0 / 1 / 2 / 3 for YUV420SP, YUV422SP, packed and
     * planar RGB.  Only the pairs the SDK's modes allow are computed:
     * semi-planar YUV to RGB, or RGB to YUV420SP.
     */
    if (!ive_csc_mode_valid(mode) || (w | h) & 1 ||
        (to_rgb ? (sfmt != 1 && sfmt != 2) || (dfmt != 2 && dfmt != 3)
                : (sfmt != 3 && sfmt != 4) || dfmt != 0)) {
        qemu_log_mask(LOG_UNIMP,
                      "hisi-ive-npu: csc mode %u, %u to %u, %ux%u\n",
                      mode, sfmt, dfmt, w, h);
        return;
    }
    if (to_rgb) {
        /* Y at +80, the chroma pairs at +88; 420 doubles each line */
        bool sp420 = sfmt == 1;
        g_autofree uint8_t *y = task_img(t, 80, 136, w, h, 1);
        g_autofree uint8_t *c = task_img(t, 88, 140, w, sp420 ? h / 2 : h, 1);
        g_autofree uint8_t *u = g_malloc(n);
        g_autofree uint8_t *v = g_malloc(n);
        g_autofree uint8_t *out = g_malloc(3 * n);
        uint8_t *ch[3];

        for (uint32_t i = 0; i < n; i++) {
            uint32_t row = sp420 ? (i / w) / 2 : i / w;
            const uint8_t *p = c + row * w + (i % w & ~1u);

            v[i] = p[0];
            u[i] = p[1];
        }
        for (int k = 0; k < 3; k++) {
            ch[k] = out + (dfmt == 3 ? k * n : k);
        }
        ive_csc_yuv_to_rgb(y, u, v, ch, dfmt == 3 ? 1 : 3, n, mode);
        if (dfmt == 3) {
            for (int k = 0; k < 3; k++) {
                task_put(t, 112 + 8 * k, 152 + 4 * k, out + k * n, w, h, 1);
            }
        } else {
            task_put(t, 112, 152, out, w, h, 3);
        }
        return;
    }
    {
        /* RGB in, YUV420SP out: the chroma plane is half the height */
        bool planar = sfmt == 4;
        g_autofree uint8_t *rgb = planar ? g_malloc(3 * n)
                                         : task_img(t, 80, 136, w, h, 3);
        g_autofree uint8_t *y = g_malloc(n);
        g_autofree uint8_t *c = g_malloc(n / 2);
        const uint8_t *p[3];
        int step = planar ? 1 : 3;

        if (planar) {
            for (int k = 0; k < 3; k++) {
                g_autofree uint8_t *pl = task_img(t, 80 + 8 * k, 136 + 4 * k,
                                                  w, h, 1);

                memcpy(rgb + k * n, pl, n);
            }
        }
        for (int k = 0; k < 3; k++) {
            p[k] = rgb + (planar ? k * n : k);
        }
        ive_csc_rgb_to_yuv(p[0], p[1], p[2], step, w, h, mode, y, c);
        task_put(t, 112, 152, y, w, h, 1);
        task_put(t, 120, 156, c, w, h / 2, 1);
    }
}

/*
 * RESIZE (op 29): the one op whose parameters are not in the task.  The
 * driver (ive_fill_npu_resize_task) builds a block in DDR -- its address
 * at task +64 -- holding the op code, the number of records, and then one
 * 64-byte record per picture (or per plane), with the fields from +16:
 * source and destination addresses, their strides, the source and
 * destination sizes, and a Q27 ratio and start fraction per axis.  The
 * task's own op code and size are left at zero, which is how a resize is
 * told apart from a DMA.
 */
#define RSZ_OP          0
#define RSZ_COUNT       4
#define RSZ_RECORD      64
#define RSZ_SRC         16
#define RSZ_DST         20
#define RSZ_SRC_STRIDE  24
#define RSZ_DST_STRIDE  28
#define RSZ_SRC_W       32
#define RSZ_SRC_H       36
#define RSZ_DST_W       40
#define RSZ_DST_H       44
#define RSZ_XRATIO      48
#define RSZ_XOFF        52
#define RSZ_YRATIO      56
#define RSZ_YOFF        60
#define RSZ_MAX         64      /* records per push: the SDK's num is <= 32 */

static bool op_resize(const uint8_t *t)
{
    uint32_t base = task_addr(t, 64);
    uint8_t hdr[8];
    uint32_t count;

    if (!base) {
        return false;
    }
    dma_memory_read(&address_space_memory, base, hdr, sizeof(hdr),
                    MEMTXATTRS_UNSPECIFIED);
    if (ldl_le_p(hdr + RSZ_OP) != OP_RESIZE) {
        return false;
    }
    count = ldl_le_p(hdr + RSZ_COUNT);
    for (uint32_t i = 0; i < MIN(count, RSZ_MAX); i++) {
        uint8_t r[RSZ_RECORD + 4];
        uint32_t sw, sh, dw, dh;

        dma_memory_read(&address_space_memory, base + (uint64_t)i * RSZ_RECORD,
                        r, sizeof(r), MEMTXATTRS_UNSPECIFIED);
        sw = ldl_le_p(r + RSZ_SRC_W);
        sh = ldl_le_p(r + RSZ_SRC_H);
        dw = ldl_le_p(r + RSZ_DST_W);
        dh = ldl_le_p(r + RSZ_DST_H);
        if (!sw || !sh || !dw || !dh ||
            sw > PIC_MAX || sh > PIC_MAX || dw > PIC_MAX || dh > PIC_MAX) {
            continue;
        }
        g_autofree uint8_t *src = g_malloc((size_t)sw * sh);
        g_autofree uint8_t *dst = g_malloc((size_t)dw * dh);

        img_read(ldl_le_p(r + RSZ_SRC), ldl_le_p(r + RSZ_SRC_STRIDE),
                 src, sw, sh);
        ive_resize(src, sw, sh, dst, dw, dh,
                   ldl_le_p(r + RSZ_XRATIO), (int32_t)ldl_le_p(r + RSZ_XOFF),
                   ldl_le_p(r + RSZ_YRATIO), (int32_t)ldl_le_p(r + RSZ_YOFF));
        img_write(ldl_le_p(r + RSZ_DST), ldl_le_p(r + RSZ_DST_STRIDE),
                  dst, dw, dh);
    }
    return true;
}

/*
 * CANNY_HYS_EDGE (ot_ive_canny_hys_edge_ctrl): the task has no source
 * picture.  The driver runs a mag_and_ang task first, into the ctrl's own
 * mem: the U16 magnitude, then the octant angle right after it.  This
 * task gets that buffer at +64 (the magnitude's byte stride at +96, the
 * angle's rows are w), the edge picture at +80 (+104) and the stack of
 * seeds at +88, with the thresholds at +300 / +304.  The stack is
 * ot_svp_point_u16 per seed and then, past the picture's worth of them,
 * the ot_ive_canny_stack_size the MPI's walk reads.
 */
static void op_canny_hys_edge(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t n = w * h, base = task_addr(t, 64), stride = task_u32(t, 96);
    g_autofree uint16_t *mag = g_new(uint16_t, n);
    g_autofree uint8_t *ang = g_malloc(n);
    g_autofree uint8_t *edge = g_malloc(n);
    g_autofree uint32_t *stack = g_new0(uint32_t, n);
    uint32_t count = 0;
    uint8_t tail[16] = { 0 };

    img_read(base, stride, (uint8_t *)mag, w * 2, h);
    /* the angle follows the magnitude, one byte per pixel */
    img_read(base + (uint64_t)stride * h, w, ang, w, h);
    for (uint32_t i = 0; i < n; i++) {
        mag[i] = le16_to_cpu(mag[i]);
    }
    ive_canny_hys_edge(mag, ang, w, h, task_u32(t, 300), task_u32(t, 304),
                       edge, stack, n, &count);
    task_put(t, 80, 104, edge, w, h, 1);
    for (uint32_t i = 0; i < count && i < n; i++) {
        stack[i] = cpu_to_le32(stack[i]);
    }
    dma_memory_write(&address_space_memory, task_addr(t, 88), stack,
                     (uint64_t)MIN(count, n) * 4, MEMTXATTRS_UNSPECIFIED);
    stl_le_p(tail, count);
    dma_memory_write(&address_space_memory,
                     task_addr(t, 88) + (uint64_t)n * 4, tail, sizeof(tail),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * GMM2 (ot_ive_gmm2_ctrl): +72 the U8 picture (+132), +64 the per-pixel
 * factor picture when either mode is per-pixel (+128, U16; otherwise the
 * driver puts the model's address there), +80 and +120 the model, +96 the
 * match info (+144), +104 the foreground (+148), +112 the background
 * (+152).  The control fields follow the struct's order from +312:
 * life factor, life threshold, initial / reduce / add / threshold
 * frequency, variation rate, max and min variation, then the global
 * sensitivity factor and the model count as bytes at +348 / +352.
 *
 * The model is model_num Gaussians of 8 bytes per pixel, pixel by pixel.
 */
static void op_gmm2(const uint8_t *t, uint32_t w, uint32_t h)
{
    IveGmm2Ctrl c = {
        .life_factor = task_u32(t, 312), .life_threshold = task_u32(t, 316),
        .freq_init = task_u32(t, 320), .freq_reduce = task_u32(t, 324),
        .freq_add = task_u32(t, 328), .freq_threshold = task_u32(t, 332),
        .var_rate = task_u32(t, 336), .max_var = task_u32(t, 340),
        .min_var = task_u32(t, 344), .sns_factor = task_u32(t, 348),
        .model_num = task_u32(t, 352),
    };
    uint32_t n = w * h, num = MIN(MAX(c.model_num, 1), 5);
    uint32_t model = task_addr(t, 80);
    g_autofree uint8_t *src = task_img(t, 72, 132, w, h, 1);
    g_autofree uint8_t *fg = g_malloc(n);
    g_autofree uint8_t *bg = g_malloc(n);
    g_autofree uint8_t *info = g_malloc(n);
    g_autofree IveGmm2Model *g = g_new(IveGmm2Model, (size_t)n * num);

    if (!model || num * 8 * (uint64_t)n > (1u << 28)) {
        return;
    }
    dma_memory_read(&address_space_memory, model, g,
                    (uint64_t)n * num * 8, MEMTXATTRS_UNSPECIFIED);
    for (uint32_t i = 0; i < n * num; i++) {
        g[i].mean = le16_to_cpu(g[i].mean);
        g[i].var = le16_to_cpu(g[i].var);
        g[i].freq = le16_to_cpu(g[i].freq);
        g[i].life = le16_to_cpu(g[i].life);
    }
    for (uint32_t i = 0; i < n; i++) {
        fg[i] = ive_gmm2_pixel(g + (size_t)i * num, src[i], &c,
                               &bg[i], &info[i]);
    }
    for (uint32_t i = 0; i < n * num; i++) {
        g[i].mean = cpu_to_le16(g[i].mean);
        g[i].var = cpu_to_le16(g[i].var);
        g[i].freq = cpu_to_le16(g[i].freq);
        g[i].life = cpu_to_le16(g[i].life);
    }
    dma_memory_write(&address_space_memory, task_addr(t, 120), g,
                     (uint64_t)n * num * 8, MEMTXATTRS_UNSPECIFIED);
    task_put(t, 104, 148, fg, w, h, 1);
    task_put(t, 112, 152, bg, w, h, 1);
    task_put(t, 96, 144, info, w, h, 1);
}

static void npu_run_task(HisiIveNpuState *s, const uint8_t *t)
{
    uint32_t op = task_u32(t, TASK_OP);
    uint32_t w = task_u32(t, TASK_W), h = task_u32(t, TASK_H);

    if (s->trace) {
        fprintf(stderr, "hisi-ive-npu: task %u op %u %ux%u\n",
                lduw_le_p(t + TASK_ID), op, w, h);
    }
    if (!op && !w && !h) {
        /* a resize carries its own parameter block and no size */
        if (op_resize(t)) {
            s->ops++;
        }
        return;
    }
    if (!w || !h || w > PIC_MAX || h > PIC_MAX) {
        return;
    }
    switch (op) {
    case OP_DMA:
        op_dma(t, w, h);
        break;
    case OP_SAD:
        op_sad(t, w, h);
        break;
    case OP_CCL:
        op_ccl(t, w, h);
        break;
    case OP_ADD:
    case OP_SUB:
    case OP_AND:
    case OP_OR:
    case OP_XOR:
        op_two(t, op, w, h);
        break;
    case OP_HIST:
        op_hist(t, w, h);
        break;
    case OP_THRESH:
    case OP_THRESH_S16:
    case OP_THRESH_U16:
        op_thresh(t, op, w, h);
        break;
    case OP_16TO8:
        op_16to8(t, w, h);
        break;
    case OP_MAP:
        op_map(t, w, h);
        break;
    case OP_INTEG:
        op_integ(t, w, h);
        break;
    case OP_NCC:
        op_ncc(t, w, h);
        break;
    case OP_DILATE:
    case OP_ERODE:
        op_morph(t, op == OP_DILATE, w, h);
        break;
    case OP_FILTER:
        op_filter(t, w, h);
        break;
    case OP_SOBEL:
        op_sobel(t, w, h);
        break;
    case OP_ORD_STAT:
        op_ord_stat(t, w, h);
        break;
    case OP_CSC:
        op_csc(t, w, h);
        break;
    case OP_MAG_AND_ANG:
        op_mag_and_ang(t, w, h);
        break;
    case OP_CANNY:
        op_canny_hys_edge(t, w, h);
        break;
    case OP_GMM2:
        op_gmm2(t, w, h);
        break;
    case OP_NORM_GRAD:
        op_norm_grad(t, w, h);
        break;
    case OP_LBP:
        op_lbp(t, w, h);
        break;
    case OP_ACC:
        op_acc(t, w, h);
        break;
    case OP_IMAGE_DOT:
        op_image_dot(t, w, h);
        break;
    case OP_ALPHA:
        op_alpha_blend(t, w, h);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "hisi-ive-npu: op %u\n", op);
        break;
    }
    s->ops++;
}

static int npu_group(HisiIveNpuState *s, int sq)
{
    return !!(s->regs[(SQ(sq) + SQ_CFG) / 4] & SQ_CFG_GROUP);
}

static void npu_update_irq(HisiIveNpuState *s)
{
    for (int g = 0; g < 2; g++) {
        uint32_t st = s->node[g] ? HWTS_IRQ_FINISH : 0;
        uint32_t mask = s->regs[(HWTS_GROUP(g) + GRP_MASK) / 4];

        qemu_set_irq(s->irq[g], (st & ~mask) != 0);
    }
}

/* Arm the timer for the earliest stream with tasks waiting */
static void npu_schedule(HisiIveNpuState *s)
{
    int64_t next = INT64_MAX;

    for (int sq = 0; sq < NPU_STREAMS; sq++) {
        if (s->pending[sq]) {
            next = MIN(next, s->due[sq]);
        }
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
}

/*
 * Each stream whose time has come runs its pushed tasks, in order along
 * the ring, up to a ring's worth per call (the rest go on 1 ms later),
 * and reports a finish
 */
static void npu_run(void *opaque)
{
    HisiIveNpuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint8_t t[TASK_SIZE];

    for (int sq = 0; sq < NPU_STREAMS; sq++) {
        bool ran = false;

        if (!s->pending[sq] || s->due[sq] > now) {
            continue;
        }
        for (int n = 0; s->pending[sq] && n < TASK_MAX; n++) {
            if (!s->cur[sq] ||
                dma_memory_read(&address_space_memory, s->cur[sq], t,
                                sizeof(t), MEMTXATTRS_UNSPECIFIED)) {
                s->pending[sq] = 0;
                break;
            }
            npu_run_task(s, t);
            s->last_id[sq] = lduw_le_p(t + TASK_ID);
            s->cur[sq] = task_u32(t, TASK_NEXT);
            s->pending[sq]--;
            ran = true;
        }
        if (ran) {
            s->node[npu_group(s, sq)] |= 1u << sq;
        }
        s->due[sq] = now + TASK_NS;
    }
    npu_schedule(s);
    npu_update_irq(s);
}

static uint64_t hisi_ive_npu_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiIveNpuState *s = opaque;
    uint32_t val = s->regs[offset / 4];

    switch (offset) {
    case AIC_IRQ_STATUS:
        val = 0;
        break;
    case HWTS_AIC_WORKING:
        val &= ~(1u << 10);
        break;
    case HWTS_AIC_SRST_ACK:
        val = 1;                    /* the soft reset is done at once */
        break;
    case HWTS_ALLOC:
        val = 0xffff;
        for (int sq = 0; sq < NPU_STREAMS; sq++) {
            if (!(s->alloc & 1u << sq)) {
                s->alloc |= 1u << sq;
                val = sq;
                break;
            }
        }
        break;
    case HWTS_GROUP(0) + GRP_STATUS:
    case HWTS_GROUP(1) + GRP_STATUS:
        val = s->node[offset >= HWTS_GROUP(1)] ? HWTS_IRQ_FINISH : 0;
        break;
    case HWTS_GROUP(0) + GRP_NODE:
    case HWTS_GROUP(1) + GRP_NODE:
        val = s->node[offset >= HWTS_GROUP(1)];
        break;
    case HWTS_GROUP(0) + GRP_STOPPED:
    case HWTS_GROUP(1) + GRP_STOPPED:
    case HWTS_GROUP(0) + GRP_PRESTOPPED:
    case HWTS_GROUP(1) + GRP_PRESTOPPED:
        /*
         * hal_ive_npu_reset_stream: pre-stop, then stop, each polled here;
         * tasks run whole within a push, so a stream stops at once
         */
        val = (1u << NPU_STREAMS) - 1;
        break;
    default:
        if (offset >= SQ(0) && offset < SQ(NPU_STREAMS)) {
            int sq = (offset - SQ(0)) / 0x40;

            switch ((offset - SQ(0)) % 0x40) {
            case SQ_CTRL:
                val &= ~0xf07u;     /* push, stop, pre-stop, free: actions */
                break;
            case SQ_COUNT:
                val = MIN(s->pending[sq], 0xfff);
                break;
            case SQ_LAST_ID:
                val = s->last_id[sq];
                break;
            }
        }
        break;
    }
    if (s->trace) {
        fprintf(stderr, "hisi-ive-npu R %04x = %08x\n", (int)offset, val);
    }
    return val;
}

static void hisi_ive_npu_write(void *opaque, hwaddr offset, uint64_t val64,
                               unsigned size)
{
    HisiIveNpuState *s = opaque;
    uint32_t val = val64;
    int sq = (offset - SQ(0)) / 0x40;

    if (s->trace) {
        fprintf(stderr, "hisi-ive-npu W %04x : %08x\n", (int)offset, val);
    }
    s->regs[offset / 4] = val;
    if (offset == HWTS_GROUP(0) + GRP_MASK ||
        offset == HWTS_GROUP(1) + GRP_MASK) {
        npu_update_irq(s);
        return;
    }
    if (offset < SQ(0) || offset >= SQ(NPU_STREAMS)) {
        return;
    }
    switch ((offset - SQ(0)) % 0x40) {
    case SQ_BASE:
        s->cur[sq] = val & ~1u;
        s->pending[sq] = 0;
        break;
    case SQ_CTRL:
        if (val & SQ_CTRL_PRESTOP) {
            s->pending[sq] = 0;
        }
        if (val & SQ_CTRL_STOP) {
            s->pending[sq] = 0;
            s->cur[sq] = s->regs[(SQ(sq) + SQ_BASE) / 4] & ~1u;
        }
        if ((val & SQ_CTRL_FREE) && !(val & 0xf00)) {
            s->alloc &= ~(1u << sq);
        }
        if (val & 0xf00) {
            if (!s->pending[sq]) {
                s->due[sq] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TASK_NS;
            }
            s->pending[sq] += (val >> 8) & 0xf;
            npu_schedule(s);
        }
        break;
    case SQ_CLEAR:
        s->node[0] &= ~(1u << sq);
        s->node[1] &= ~(1u << sq);
        npu_update_irq(s);
        break;
    }
}

static const MemoryRegionOps hisi_ive_npu_ops = {
    .read = hisi_ive_npu_read,
    .write = hisi_ive_npu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_ive_npu_reset(DeviceState *dev)
{
    HisiIveNpuState *s = HISI_IVE_NPU(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->alloc = 0;
    memset(s->node, 0, sizeof(s->node));
    memset(s->cur, 0, sizeof(s->cur));
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->last_id, 0, sizeof(s->last_id));
    memset(s->due, 0, sizeof(s->due));
    s->ops = 0;
}

static void hisi_ive_npu_init(Object *obj)
{
    HisiIveNpuState *s = HISI_IVE_NPU(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_ive_npu_ops, s,
                          TYPE_HISI_IVE_NPU, NPU_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[0]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[1]);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, npu_run, s);
    object_property_add_uint32_ptr(obj, "ops", &s->ops, OBJ_PROP_FLAG_READ);
}

static const Property hisi_ive_npu_props[] = {
    DEFINE_PROP_BOOL("trace", HisiIveNpuState, trace, false),
};

static const VMStateDescription vmstate_hisi_ive_npu = {
    .name = TYPE_HISI_IVE_NPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HisiIveNpuState, NPU_MMIO_SIZE / 4),
        VMSTATE_UINT32(alloc, HisiIveNpuState),
        VMSTATE_UINT32_ARRAY(node, HisiIveNpuState, 2),
        VMSTATE_UINT32_ARRAY(cur, HisiIveNpuState, NPU_STREAMS),
        VMSTATE_UINT32_ARRAY(pending, HisiIveNpuState, NPU_STREAMS),
        VMSTATE_UINT32_ARRAY(last_id, HisiIveNpuState, NPU_STREAMS),
        VMSTATE_INT64_ARRAY(due, HisiIveNpuState, NPU_STREAMS),
        VMSTATE_TIMER_PTR(timer, HisiIveNpuState),
        VMSTATE_END_OF_LIST()
    },
};

static void hisi_ive_npu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_ive_npu_reset);
    device_class_set_props(dc, hisi_ive_npu_props);
    dc->vmsd = &vmstate_hisi_ive_npu;
    dc->desc = "HiSilicon V5 IVE (SVP NPU task scheduler)";
}

static const TypeInfo hisi_ive_npu_info = {
    .name = TYPE_HISI_IVE_NPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiIveNpuState),
    .instance_init = hisi_ive_npu_init,
    .class_init = hisi_ive_npu_class_init,
};

static void hisi_ive_npu_register_types(void)
{
    type_register_static(&hisi_ive_npu_info);
}

type_init(hisi_ive_npu_register_types)
