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
 * and the op's own fields from +300.  dma, sad, ccl and add (majestic's
 * motion detection) and sub and hist (the SDK sample's memory test) are
 * modelled; any other op completes and leaves its output alone
 * (LOG_UNIMP).
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
    OP_DMA = 0, OP_SUB = 9, OP_HIST = 12, OP_ADD = 18, OP_CCL = 21,
    OP_SAD = 27,
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
    g_autofree uint8_t *sad8 = g_malloc(nb);
    g_autofree uint16_t *sad16 = g_malloc(2 * nb);
    g_autofree uint8_t *thr8 = g_malloc(nb);
    uint32_t out1 = task_addr(t, 88), out2 = task_addr(t, 96);

    if (ctrl > 4 || !bw || !bh) {
        return;
    }
    img_read(task_addr(t, 64), task_u32(t, 104), f1, w, h);
    img_read(task_addr(t, 72), task_u32(t, 108), f2, w, h);
    for (uint32_t by = 0; by < bh; by++) {
        for (uint32_t bx = 0; bx < bw; bx++) {
            uint32_t sad = 0, i = by * bw + bx;

            for (int dy = 0; dy < b; dy++) {
                for (int dx = 0; dx < b; dx++) {
                    uint32_t p = (by * b + dy) * w + bx * b + dx;

                    sad += abs((int)f1[p] - (int)f2[p]);
                }
            }
            sad16[i] = cpu_to_le16(MIN(sad, 0xffff));
            sad8[i] = MIN(sad, 0xff);
            thr8[i] = sad > thr ? hi : lo;
        }
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

/*
 * ADD (ot_ive_add_ctrl): +80 / +88 the U8 sources, +104 the destination,
 * their strides at +112 / +116 / +124; dst = (x * src1 + y * src2) >> 16
 * with the Q0.16 weights x / y at +300 / +304, saturated.
 */
static void op_add(const uint8_t *t, uint32_t w, uint32_t h)
{
    uint32_t x = task_u32(t, 300) & 0xffff, y = task_u32(t, 304) & 0xffff;
    g_autofree uint8_t *f1 = g_malloc(w * h);
    g_autofree uint8_t *f2 = g_malloc(w * h);

    img_read(task_addr(t, 80), task_u32(t, 112), f1, w, h);
    img_read(task_addr(t, 88), task_u32(t, 116), f2, w, h);
    for (uint32_t i = 0; i < w * h; i++) {
        f1[i] = MIN((f1[i] * x + f2[i] * y) >> 16, 255);
    }
    img_write(task_addr(t, 104), task_u32(t, 124), f1, w, h);
}

/*
 * SUB (ot_ive_sub_ctrl), add's layout: dst = |src1 - src2|, or with mode
 * 1 (+300) the signed difference halved, as S8.
 */
static void op_sub(const uint8_t *t, uint32_t w, uint32_t h)
{
    bool shift = task_u32(t, 300) == 1;
    g_autofree uint8_t *f1 = g_malloc(w * h);
    g_autofree uint8_t *f2 = g_malloc(w * h);

    img_read(task_addr(t, 80), task_u32(t, 112), f1, w, h);
    img_read(task_addr(t, 88), task_u32(t, 116), f2, w, h);
    for (uint32_t i = 0; i < w * h; i++) {
        int d = (int)f1[i] - (int)f2[i];

        f1[i] = shift ? (uint8_t)(int8_t)(d >> 1) : abs(d);
    }
    img_write(task_addr(t, 104), task_u32(t, 124), f1, w, h);
}

/* HIST: the U8 image at +80 (stride +104) into 256 u32 bins at +96 */
static void op_hist(const uint8_t *t, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *f = g_malloc(w * h);
    uint32_t bins[256] = { 0 };

    img_read(task_addr(t, 80), task_u32(t, 104), f, w, h);
    for (uint32_t i = 0; i < w * h; i++) {
        bins[f[i]]++;
    }
    for (int i = 0; i < 256; i++) {
        bins[i] = cpu_to_le32(bins[i]);
    }
    dma_memory_write(&address_space_memory, task_addr(t, 96), bins,
                     sizeof(bins), MEMTXATTRS_UNSPECIFIED);
}

static void npu_run_task(HisiIveNpuState *s, const uint8_t *t)
{
    uint32_t op = task_u32(t, TASK_OP);
    uint32_t w = task_u32(t, TASK_W), h = task_u32(t, TASK_H);

    if (s->trace) {
        fprintf(stderr, "hisi-ive-npu: task %u op %u %ux%u\n",
                lduw_le_p(t + TASK_ID), op, w, h);
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
        op_add(t, w, h);
        break;
    case OP_SUB:
        op_sub(t, w, h);
        break;
    case OP_HIST:
        op_hist(t, w, h);
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
