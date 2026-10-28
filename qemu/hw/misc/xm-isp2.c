/*
 * Xiongmai second-generation (XM530V200 / XM533 / XM535) VI + ISP, as the
 * vendor isp.ko drives it.  Capture is no longer a separate vi.ko block:
 * isp.ko runs VICU (capture) -> BPU (the Bayer pipeline, with its own
 * frame timing generator) -> NR3D online, one interrupt each per frame.
 *
 * Three windows, all RAM-backed so the driver's configuration reads back:
 *
 *   0x30000000  VICU0 at +0x40000, VICU1 at +0x50000; per VICU two
 *               interrupt status words (non-zero: pending; the driver
 *               writes 0 to clear): +0x805C at frame start, +0x804C at
 *               frame end, with +0x8008 the frame-done info (non-zero for
 *               a frame); +0x8080 the current line
 *   0x32000000  BPU: +0x44 interrupt status, likewise; +0x100008 written 3
 *               and read back by bpu_reg_init
 *   0x34000000  VPSS / NR3D: NR3D interrupt status at +0xF04C
 *
 * Interrupts (isp.ko request_irq), outputs 0..3 in this order:
 *   vicu_bayer_irq0 SPI 55   +0x804C of either VICU (vicu_irq_0 reads both)
 *   vicu_bayer_irq1 SPI 56   +0x805C of either VICU (vicu_irq_1)
 *   bpu_hvgen_irq   SPI 30
 *   nr3d_irq        SPI 37
 *
 * Each frame period: frame start on every VICU whose frame-start interrupt
 * is enabled (+0x50 of its interrupt block; frame end: +0x40), half a period
 * later frame end, then the BPU and NR3D.  A line drops when the driver
 * clears its status.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/xm-regtrace.h"
#include <math.h>
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "system/dma.h"
#include "aw-vinc-logo.h"

#define TYPE_XM_ISP2 "xm-isp2"
OBJECT_DECLARE_SIMPLE_TYPE(XmIsp2State, XM_ISP2)

#define VI_SIZE     0x100000
#define BPU_SIZE    0x200000
#define VPSS_SIZE   0x100000

#define VICU0       0x40000
#define VICU1       0x50000
/* each VICU's interrupt block, at +0x8000 (XM535) or +0x7000 (XM530V200):
 * +0x4C frame-end status, +0x5C frame-start status, +0x08 frame done */
#define VICU_INT(s) ((s)->l->vicu_irq ? (s)->l->vicu_irq : 0x8000)
#define VICU_ST(s)      (VICU_INT(s) + 0x4C)    /* frame end */
#define VICU_ST_FS(s)   (VICU_INT(s) + 0x5C)    /* frame start */
#define VICU_DONE(s)    (VICU_INT(s) + 0x08)
#define VICU_EN(s)      (VICU_INT(s) + 0x40)    /* frame end enable */
#define VICU_EN_FS(s)   (VICU_INT(s) + 0x50)    /* frame start enable */
/* the VICU's line count, holding the last frame's height until the next
 * frame ends.  isp.ko keeps a frame of a shared-phy sensor pair (XM535D2)
 * only if this is between half and all of (+0x54 lines + 15). */
#define VICU_LINE(s)    (VICU_INT(s) + 0x80)
/* MIPI receivers, one per sensor: what they measure of each frame.
 * +0x40/+0x44 total, +0x48/+0x4C active (MIPI words), +0x10C..+0x118 the
 * same in pixels -- read by isp.ko for /proc/umap/isp and by the isp
 * daemon's XM_MPI_MIPI_RefreshFV, which wants the active line count. */
/* MIPI-IO (mipi_io.ko): the MIPI pads 6..9 as GPIOs, inputs at +0x1E8,
 * a bit per pad.  Sofia's keys 1006..1009 are these pads -- XM533's reset
 * key is 1008 -- active low: idle inputs read high. */
#define MIPI_IO_IN  0x1E8
#define MIPI_IO_UP  0x3C0
#define MIPI0       0x4000
#define MIPI1       0x5000
#define BPU_ST      0x44
#define BPU_START   0x20        /* bpu_offline_deal_in writes 2: one job */
/* MemorySaver's read DMA, in the VI window: the offline BPU job's raw frame
 * fetch, kicked (+0x34 = 1) after BPU_START -- and after isp.ko marks the
 * BPU busy.  A job completes 2 ms after the kick: completing it between the
 * start and the busy mark (the host was slow to run the guest) makes
 * bpu_irq take it for a spurious interrupt and leaves the BPU busy for
 * good (XM535D1/D2). */
#define RDMA_KICK   0x11034
#define NR3D_ST     0xF04C
/*
 * XM530V200's vpss.ko runs the VPSS memory to memory, a job per frame:
 *   +0x1000C / +0x11020   input width (at most the stride) / height
 *   +0x11000 / +0x11010   input luma / chroma, (address - 0x80000000) >> 4,
 *   +0x11008 / +0x11018   their strides in 16 bytes
 *   +0x81004 / +0x8100C   scaler c's output width / height (+0x1000 * c)
 *   store window c (+0x90000 + 0x1000 * c): +0x34 enable, +0x00 / +0x10
 *                         luma / chroma, +0x20 / +0x28 their strides
 *   +0x10020              1: start; reads 1 until the job is done
 *   +0x10044              done (SPI 40); the driver writes 0
 * The input is the BPU's output; here the job draws the test picture into
 * it, then scales it to each enabled output, nearest neighbour.  Splices
 * (two lenses into one picture) need nothing more: the driver programs
 * each lens's half as an output address.
 */
#define VPSS_START      0x10020
#define VPSS_INT        0x10044
#define VPSS_IN_W       0x1000C
#define VPSS_IN_H       0x11020
#define VPSS_IN_Y       0x11000
#define VPSS_IN_C       0x11010
#define VPSS_IN_YS      0x11008
#define VPSS_IN_CS      0x11018
#define VPSS_OUT_W(c)   (0x81004 + (c) * 0x1000)
#define VPSS_OUT_H(c)   (0x8100C + (c) * 0x1000)
#define VPSS_STORE(c)   (0x90000 + (c) * 0x1000)
#define VPSS_OUTPUTS    3
#define VPSS_NR3D_REF   0xC000  /* the group's own 3DNR reference */
#define VPSS_LENSES     4
/* VPSS store (older SDKs, in vi.ko: vi_yuv_store_int), per output channel c
 * at +0x1000 * c: interrupt enable +0x87040 / +0x87050, status +0x8704C
 * (frame stored) / +0x8705C (frame start).  Lines SPI 25, 26, 35, 36. */
#define STORE_INT(c)    (s->l->store_int + (c) * s->l->store_int_stride)
/* ...and its store window at +0x90000 + 0x1000 * c: +0x00 / +0x10 the luma
 * and chroma buffers the driver programs, (address - 0x80000000) >> 4, and
 * +0x100 / +0x108 their shadows, which the hardware takes at frame start.
 * The frame-stored handler counts a frame as written only when the shadows
 * hold a buffer (luma and chroma differ). */
#define STORE_WIN(c)    (s->l->store_win + (c) * 0x1000)
#define STORE_Y         0x00
#define STORE_C         0x10
#define STORE_Y_STRIDE  0x20        /* in 16-byte units, chroma at +0x24 */
#define STORE_Y_SIZE    0x58        /* luma plane bytes >> 4 */
#define STORE_Y_SHADOW  0x100
#define STORE_C_SHADOW  0x108
#define STORE_CHANNELS  4

/*
 * Where the blocks sit differs between SDK generations (the `layout`
 * property):
 *   xm535  XM535: VICU blocks at 0x30040000 / 0x30050000 with
 *          their status words at +0x804C / +0x805C (XM530V200, layout
 *          xm530v200: at +0x704C / +0x705C), MIPI at 0x30004000 /
 *          0x30005000, the VPSS store in the 0x34000000 window
 *   xm533  XM533 (XM530V300): one VICU interrupt block at 0x300F0040
 *          (enable +0x40, status +0x4C, as XM530's VI), MIPI at 0x30000000,
 *          the store in the VI window: one interrupt block at +0x67000 for
 *          all channels, their windows at +0x70000
 */
typedef struct XmIsp2Layout {
    const char *name;
    uint32_t vicu[2];           /* +0x804C/+0x805C VICUs, 0 = none */
    uint32_t vicu_irq;          /* their interrupt block, 0 = +0x8000 */
    uint32_t vicu_int;          /* XM530-style interrupt block, 0 = none */
    uint32_t mipi[2];
    bool store_in_vi;           /* store blocks in the VI window, not VPSS */
    uint32_t store_int, store_win;
    uint32_t store_int_stride;  /* 0: one interrupt block for all channels */
    bool vpss_offline;          /* vpss.ko's memory-to-memory jobs */
    bool vpss_inline;           /* isp.ko's VPSS behind each offline BPU job */
} XmIsp2Layout;

static const XmIsp2Layout xm_isp2_layouts[] = {
    { .name = "xm535", .vicu = { 0x40000, 0x50000 },
      .mipi = { 0x4000, 0x5000 }, .store_int = 0x87000, .store_win = 0x90000,
      .store_int_stride = 0x1000, .vpss_inline = true },
    /* XM530V200's isp.ko: XM535's blocks, the VICU interrupt words at
     * +0x7000 (vicu_irq_0 reads +0x704C of both VICUs) */
    { .name = "xm530v200", .vicu = { 0x40000, 0x50000 }, .vicu_irq = 0x7000,
      .mipi = { 0x4000, 0x5000 }, .store_int = 0x87000, .store_win = 0x90000,
      .store_int_stride = 0x1000, .vpss_offline = true },
    /* XM533's vi.ko reads one status word, +0x6704C (stored) / +0x6705C
     * (start), and then walks every channel */
    { .name = "xm533", .vicu_int = 0xF0000, .mipi = { 0x0000, 0x1000 },
      .store_in_vi = true, .store_int = 0x67000, .store_win = 0x70000 },
};

enum { IRQ_VICU0, IRQ_VICU1, IRQ_BPU, IRQ_NR3D, IRQ_STORE0,
       IRQ_VPSS = IRQ_STORE0 + STORE_CHANNELS, NUM_IRQ };

struct XmIsp2State {
    SysBusDevice parent_obj;
    MemoryRegion vi_io, bpu_io, vpss_io;
    uint32_t vi[VI_SIZE / 4], bpu[BPU_SIZE / 4], vpss[VPSS_SIZE / 4];
    unsigned long vi_seen[BITS_TO_LONGS(VI_SIZE / 4)];
    unsigned long vpss_seen[BITS_TO_LONGS(VPSS_SIZE / 4)];
    bool trace_reads;
    qemu_irq irq[NUM_IRQ];
    QEMUTimer *timer;
    QEMUTimer *bpu_timer;
    QEMUTimer *vpss_timer;
    uint32_t vpss_jobs;
    uint32_t vpss_pending;      /* started, not completed yet */
    uint32_t vpss_group_ref[VPSS_LENSES];   /* which group is which lens */
    uint32_t fps;
    uint32_t vicus;             /* VICUs that capture: 1, or 2 (dual lens) */
    uint32_t sns_w, sns_h;      /* the sensor's active output */
    uint64_t frames;
    bool end_phase;
    uint32_t phase;             /* quarter frames, see xm_isp2_frame */
    uint32_t store_lines;       /* 0: one line per event, not per channel */
    char *layout;
    const XmIsp2Layout *l;
    uint32_t *store;            /* the window the store blocks are in */
    uint32_t store_frame[STORE_CHANNELS];
    bool bpu_offline;           /* seen a BPU_START: jobs, not a timing gen */
    bool bpu_started;           /* a BPU_START not yet followed by RDMA_KICK */
    uint32_t bpu_pending;       /* offline BPU starts not yet completed */
    bool trace;
};

static void xm_isp2_update(XmIsp2State *s)
{
    bool end = false, start = false;

    for (int v = 0; v < 2; v++) {
        if (s->l->vicu[v]) {
            end |= s->vi[(s->l->vicu[v] + VICU_ST(s)) / 4] != 0;
            start |= s->vi[(s->l->vicu[v] + VICU_ST_FS(s)) / 4] != 0;
        }
    }
    if (s->l->vicu_int) {
        end |= s->vi[(s->l->vicu_int + 0x4C) / 4] != 0;
    }
    qemu_set_irq(s->irq[IRQ_VICU0], end);
    qemu_set_irq(s->irq[IRQ_VICU1], start);
    qemu_set_irq(s->irq[IRQ_BPU], s->bpu[BPU_ST / 4] != 0);
    qemu_set_irq(s->irq[IRQ_NR3D], s->vpss[NR3D_ST / 4] != 0);
    qemu_set_irq(s->irq[IRQ_VPSS], s->vpss[VPSS_INT / 4] != 0);
    bool stored = false, started = false;
    for (int c = 0; c < STORE_CHANNELS; c++) {
        const bool e = s->store[(STORE_INT(c) + 0x4C) / 4] != 0;
        const bool f = s->store[(STORE_INT(c) + 0x5C) / 4] != 0;

        stored |= e;
        started |= f;
        if (s->store_lines) {
            qemu_set_irq(s->irq[IRQ_STORE0 + c], e || f);
        }
    }
    if (!s->store_lines) {
        /* XM533: a line per event, each walking every channel --
         * vi_yuv_store_int the stored words (+0x4C), vi_yuv_store_nrt_int
         * the frame-start ones (+0x5C) */
        qemu_set_irq(s->irq[IRQ_STORE0], stored);
        qemu_set_irq(s->irq[IRQ_STORE0 + 1], started);
    }
}

static void xm_isp2_store_pattern(XmIsp2State *s, int c);

static void xm_isp2_frame(void *opaque)
{
    XmIsp2State *s = XM_ISP2(opaque);

    const uint32_t *vicu = s->l->vicu;
    /* Two sensors are not phase-locked: their frame starts and ends come a
     * quarter frame apart (phases 0..3: VICU0 start, VICU1 start, VICU0
     * end, VICU1 end). isp.ko's frame-end handler sets the BPU's
     * new-parameters mask to its own channel alone, so ends arriving
     * together leave one channel's parameters never taken. */
    const bool two = s->vicus > 1 && vicu[1];
    const int only = two ? (s->phase & 1) : -1;
    uint32_t v;

    s->end_phase = s->phase >= 2;
    /* only what vicu_run enabled: +0x40 frame end, +0x50 frame start */
    for (v = 0; v < s->vicus && v < 2 && vicu[v]; v++) {
        if (only >= 0 && v != (uint32_t)only) {
            continue;
        }
        if (s->end_phase) {
            s->vi[(vicu[v] + VICU_LINE(s)) / 4] = s->sns_h;
        }
        if (!s->end_phase) {
            if (s->vi[(vicu[v] + VICU_EN_FS(s)) / 4]) {
                s->vi[(vicu[v] + VICU_ST_FS(s)) / 4] = 1;
            }
        } else if (s->vi[(vicu[v] + VICU_EN(s)) / 4]) {
            s->vi[(vicu[v] + VICU_DONE(s)) / 4] = 1;
            s->vi[(vicu[v] + VICU_ST(s)) / 4] = 1;
        }
    }
    if (only > 0) {
        goto next;
    }
    if (s->l->vicu_int && s->end_phase &&
        s->vi[(s->l->vicu_int + 0x40) / 4]) {
        s->vi[(s->l->vicu_int + 0x4C) / 4] = 1;
    }
    for (v = 0; v < s->vicus && v < 2; v++) {
        uint32_t m = s->l->mipi[v], w = s->sns_w, h = s->sns_h;

        s->vi[(m + 0x40) / 4] = w;
        s->vi[(m + 0x44) / 4] = h;
        s->vi[(m + 0x48) / 4] = w;
        s->vi[(m + 0x4C) / 4] = h;
        s->vi[(m + 0x10C) / 4] = w;
        s->vi[(m + 0x110) / 4] = h;
        s->vi[(m + 0x114) / 4] = w;
        s->vi[(m + 0x118) / 4] = h;
        /* The VICU's own measure, height << 16 | width: +0x90 total,
         * +0x94 active (isp.ko's VI_info). */
        if (vicu[v]) {
            s->vi[(vicu[v] + 0x90) / 4] = h << 16 | w;
            s->vi[(vicu[v] + 0x94) / 4] = h << 16 | w;
        }
    }
    if (s->end_phase) {
        s->frames++;
        s->vpss[NR3D_ST / 4] = 1;
        /* Online (older isp.ko): the BPU's timing generator ticks with
         * every frame.  Offline jobs are started through BPU_START. */
        if (!s->bpu_offline) {
            s->bpu[BPU_ST / 4] = 1;
        }
    }
    for (int c = 0; c < STORE_CHANNELS; c++) {
        uint32_t b = STORE_INT(c);

        if (s->end_phase && s->store[(b + 0x40) / 4]) {
            xm_isp2_store_pattern(s, c);
            s->store[(b + 0x4C) / 4] = 1;
        } else if (!s->end_phase && s->store[(b + 0x50) / 4]) {
            uint32_t w = STORE_WIN(c);

            s->store[(w + STORE_Y_SHADOW) / 4] = s->store[(w + STORE_Y) / 4];
            s->store[(w + STORE_C_SHADOW) / 4] = s->store[(w + STORE_C) / 4];
            s->store[(b + 0x5C) / 4] = 1;
        }
    }
next:
    s->phase = (s->phase + (two ? 1 : 2)) & 3;
    xm_isp2_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->fps / (two ? 4 : 2));
}

/*
 * The picture a stored frame holds: moving bars with the frame number in
 * binary blocks top left (as xm-vi draws), NV12 at the programmed luma and
 * chroma buffers.
 */
/*
 * The test picture, as aw-vinc.c's: the OpenIPC logo, white on a navy
 * gradient, with a round dot sliding to and fro beneath it, laid out on a
 * 1920x1080 canvas and scaled to each buffer.  Background and logo are
 * drawn once per size; each frame copies them and adds the dot (frame n).
 */
#define PAT_W               1920
#define PAT_H               1080
#define PAT_LOGO_X          ((PAT_W - AW_VINC_LOGO_W) / 2)
#define PAT_LOGO_Y          (PAT_H * 2 / 5 - AW_VINC_LOGO_H / 2)
#define PAT_DOT_R           64
#define PAT_DOT_Y           (PAT_H * 4 / 5)
#define PAT_DOT_X0          (PAT_LOGO_X + PAT_DOT_R)
#define PAT_DOT_RANGE       (AW_VINC_LOGO_W - 2 * PAT_DOT_R)

static uint8_t *xm_logo;        /* coverage 0..255, AW_VINC_LOGO_W x _H */

static void xm_logo_init(void)
{
    const unsigned sc = AW_VINC_LOGO_SCALE;
    const unsigned bw = AW_VINC_LOGO_W * sc;
    gsize n, i = 0;
    g_autofree guchar *runs = g_base64_decode(aw_vinc_logo_runs_b64, &n);
    g_autofree uint8_t *count = g_malloc0(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    uint64_t pos = 0, end = (uint64_t)bw * AW_VINC_LOGO_H * sc;
    bool set = false;

    while (i < n && pos < end) {
        uint64_t run = 0;
        unsigned shift = 0;

        do {
            run |= (uint64_t)(runs[i] & 0x7F) << shift;
            shift += 7;
        } while ((runs[i++] & 0x80) && i < n && shift < 35);
        for (; set && run && pos < end; run--, pos++) {
            count[(pos / bw / sc) * AW_VINC_LOGO_W + (pos % bw) / sc]++;
        }
        pos += run;
        set = !set;
    }
    xm_logo = g_malloc(AW_VINC_LOGO_W * AW_VINC_LOGO_H);
    for (i = 0; i < AW_VINC_LOGO_W * AW_VINC_LOGO_H; i++) {
        xm_logo[i] = count[i] * 255 / (sc * sc);
    }
}

static uint8_t xm_blend(uint8_t bg, uint8_t fg, unsigned a)
{
    return bg + ((int)fg - bg) * (int)a / 255;
}

/* Y, Cb, Cr of the canvas at (px, py), the dot centred at dot_x (or none) */
static void xm_pattern_pixel(int px, int py, int dot_x, uint8_t *yuv)
{
    unsigned a = 0;

    yuv[0] = 50 - 14 * py / PAT_H;      /* navy, lighter at the top */
    yuv[1] = 146;
    yuv[2] = 122;
    if (px >= PAT_LOGO_X && px < PAT_LOGO_X + AW_VINC_LOGO_W &&
        py >= PAT_LOGO_Y && py < PAT_LOGO_Y + AW_VINC_LOGO_H) {
        a = xm_logo[(py - PAT_LOGO_Y) * AW_VINC_LOGO_W + px - PAT_LOGO_X];
    } else if (dot_x >= 0) {
        const int dx = px - dot_x, dy = py - PAT_DOT_Y;

        if (abs(dx) <= PAT_DOT_R + 1 && abs(dy) <= PAT_DOT_R + 1) {
            const int d = (int)(sqrt(dx * dx + dy * dy) * 256);

            a = d <= PAT_DOT_R * 256 - 128 ? 255 :
                d >= PAT_DOT_R * 256 + 128 ? 0 :
                (PAT_DOT_R * 256 + 128 - d) * 255 / 256;
        }
    }
    if (a) {
        yuv[0] = xm_blend(yuv[0], 235, a);
        yuv[1] = xm_blend(yuv[1], 128, a);
        yuv[2] = xm_blend(yuv[2], 128, a);
    }
}

/* the picture without the dot, NV12, w x h (one size at a time) */
static struct {
    int w, h;
    uint8_t *y, *c;
} xm_pat;

static void xm_pattern_static(int w, int h)
{
    uint8_t yuv[3];

    if (xm_pat.w == w && xm_pat.h == h) {
        return;
    }
    if (!xm_logo) {
        xm_logo_init();
    }
    g_free(xm_pat.y);
    g_free(xm_pat.c);
    xm_pat.w = w;
    xm_pat.h = h;
    xm_pat.y = g_malloc((size_t)w * h);
    xm_pat.c = g_malloc((size_t)w * (h / 2));
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            xm_pattern_pixel(x * PAT_W / w, y * PAT_H / h, -1, yuv);
            xm_pat.y[(size_t)y * w + x] = yuv[0];
            if (!(y & 1) && !(x & 1)) {
                xm_pat.c[(size_t)(y / 2) * w + x] = yuv[1];
                xm_pat.c[(size_t)(y / 2) * w + x + 1] = yuv[2];
            }
        }
    }
}

static void xm_isp2_draw(hwaddr ya, hwaddr ca, int w, int h, int ys, int cs,
                         uint32_t n, int lens)
{
    /* each lens sees the dot a third of a sweep later than the last */
    const uint32_t t = (n * 8 + lens * 2 * PAT_DOT_RANGE / 3) %
                       (2 * PAT_DOT_RANGE);
    const int dot_x = PAT_DOT_X0 +
                      (t < PAT_DOT_RANGE ? t : 2 * PAT_DOT_RANGE - t);
    /* the rows and columns the dot can touch, in buffer pixels; columns
     * from an even one, where the CbCr pairs start */
    const int y0 = (PAT_DOT_Y - PAT_DOT_R - 2) * h / PAT_H;
    const int y1 = (PAT_DOT_Y + PAT_DOT_R + 2) * h / PAT_H + 1;
    const int x0 = MAX((dot_x - PAT_DOT_R - 2) * w / PAT_W, 0) & ~1;
    const int x1 = MIN((dot_x + PAT_DOT_R + 2) * w / PAT_W + 2, w) & ~1;
    uint8_t yuv[3];

    if (w <= 1 || h <= 1 || w > 4096 || h > 4096 || ys < w || cs < w) {
        return;
    }
    xm_pattern_static(w, h);
    g_autofree uint8_t *line = g_malloc(w);
    for (int y = 0; y < h; y++) {
        memcpy(line, xm_pat.y + (size_t)y * w, w);
        if (y >= y0 && y <= y1) {
            for (int x = x0; x < x1; x++) {
                xm_pattern_pixel(x * PAT_W / w, y * PAT_H / h, dot_x, yuv);
                line[x] = yuv[0];
            }
        }
        dma_memory_write(&address_space_memory, ya + (hwaddr)y * ys, line, w,
                         MEMTXATTRS_UNSPECIFIED);
    }
    for (int y = 0; y < h / 2; y++) {
        memcpy(line, xm_pat.c + (size_t)y * w, w & ~1);
        if (2 * y >= y0 && 2 * y <= y1) {
            for (int x = x0; x < x1; x += 2) {
                xm_pattern_pixel(x * PAT_W / w, 2 * y * PAT_H / h, dot_x, yuv);
                line[x] = yuv[1];
                line[x + 1] = yuv[2];
            }
        }
        dma_memory_write(&address_space_memory, ca + (hwaddr)y * cs, line,
                         w & ~1, MEMTXATTRS_UNSPECIFIED);
    }
}

static void xm_isp2_store_pattern(XmIsp2State *s, int c)
{
    const uint32_t win = STORE_WIN(c);
    /* the buffer the hardware took at frame start, not the next one the
     * driver may already have programmed */
    const uint32_t yr = s->store[(win + STORE_Y_SHADOW) / 4] ?
                        s->store[(win + STORE_Y_SHADOW) / 4] :
                        s->store[(win + STORE_Y) / 4];
    const uint32_t cr = s->store[(win + STORE_C_SHADOW) / 4] ?
                        s->store[(win + STORE_C_SHADOW) / 4] :
                        s->store[(win + STORE_C) / 4];
    const int ys = s->store[(win + STORE_Y_STRIDE) / 4] * 16;
    const int cs = s->store[(win + STORE_Y_STRIDE + 4) / 4] * 16;
    const int w = ys, h = ys ? (int)(s->store[(win + STORE_Y_SIZE) / 4] * 16 /
                                     ys) : 0;
    const uint32_t n = s->store_frame[c]++;

    if (!yr || !cr) {
        return;
    }
    xm_isp2_draw(0x80000000ULL + (hwaddr)yr * 16,
                 0x80000000ULL + (hwaddr)cr * 16, w, h, ys, cs, n, c);
}

/* which lens a VPSS job is for: each group (lens; two on the spliced
 * dual-lens boards) keeps its own 3DNR reference, numbered as they come */
static int xm_isp2_vpss_lens(XmIsp2State *s)
{
    const uint32_t ref = s->vpss[VPSS_NR3D_REF / 4];
    int lens = 0;

    while (lens < VPSS_LENSES && s->vpss_group_ref[lens] &&
           s->vpss_group_ref[lens] != ref) {
        lens++;
    }
    if (lens == VPSS_LENSES) {
        lens = 0;
    }
    s->vpss_group_ref[lens] = ref;
    return lens;
}

/*
 * The VPSS behind an offline BPU job (newer XM535 isp.ko, "VpssMode
 * Online"): no input registers, the BPU's output goes straight to the
 * scalers, so each enabled output gets the picture at its own size.
 */
static void xm_isp2_vpss_inline(XmIsp2State *s)
{
    const uint32_t *r = s->vpss;
    const int lens = xm_isp2_vpss_lens(s);

    for (int c = 0; c < VPSS_OUTPUTS; c++) {
        const uint32_t st = VPSS_STORE(c);
        const int ow = r[VPSS_OUT_W(c) / 4], oh = r[VPSS_OUT_H(c) / 4];

        if (r[(st + 0x34) / 4] && r[st / 4] && r[(st + 0x10) / 4]) {
            xm_isp2_draw(0x80000000ULL + (hwaddr)r[st / 4] * 16,
                         0x80000000ULL + (hwaddr)r[(st + 0x10) / 4] * 16,
                         ow, oh, r[(st + 0x20) / 4] * 16,
                         r[(st + 0x28) / 4] * 16, s->vpss_jobs, lens);
        }
    }
    s->vpss_jobs++;
}

/* a job's work, done when it starts (the registers are the next job's
 * soon); its completion comes later, xm_isp2_vpss_done */
static void xm_isp2_vpss_job(XmIsp2State *s)
{
    const uint32_t *r = s->vpss;
    /* strides in 16 bytes; anything past 16 KiB is no job (and must not
     * size a buffer) */
    const int ys = MIN(r[VPSS_IN_YS / 4], 1025u) * 16;
    const int cs = MIN(r[VPSS_IN_CS / 4], 1025u) * 16;
    /* +0x1000C is the width the job scales from; boards that sell a 2MP
     * sensor as "3M" set it above the input's (2304 on a 1920 stride) */
    const int w = MIN((int)r[VPSS_IN_W / 4], ys), h = r[VPSS_IN_H / 4];
    const hwaddr ya = 0x80000000ULL + (hwaddr)r[VPSS_IN_Y / 4] * 16;
    const hwaddr ca = 0x80000000ULL + (hwaddr)r[VPSS_IN_C / 4] * 16;

    if (w > 0 && h > 1 && w <= 4096 && h <= 4096 && ys >= w && cs >= w &&
        ys <= 16384 && cs <= 16384 && r[VPSS_IN_Y / 4] && r[VPSS_IN_C / 4]) {
        g_autofree uint8_t *iy = g_malloc((size_t)ys * h);
        g_autofree uint8_t *ic = g_malloc((size_t)cs * (h / 2));

        xm_isp2_draw(ya, ca, w, h, ys, cs, s->vpss_jobs,
                     xm_isp2_vpss_lens(s));
        dma_memory_read(&address_space_memory, ya, iy, (size_t)ys * h,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, ca, ic, (size_t)cs * (h / 2),
                        MEMTXATTRS_UNSPECIFIED);
        for (int c = 0; c < VPSS_OUTPUTS; c++) {
            const uint32_t st = VPSS_STORE(c);
            const int ow = r[VPSS_OUT_W(c) / 4], oh = r[VPSS_OUT_H(c) / 4];
            const int oys = r[(st + 0x20) / 4] * 16;
            const int ocs = r[(st + 0x28) / 4] * 16;

            if (!r[(st + 0x34) / 4] || !r[st / 4] || !r[(st + 0x10) / 4] ||
                ow <= 1 || oh <= 1 || ow > 4096 || oh > 4096 ||
                oys < ow || ocs < ow) {
                continue;
            }
            const hwaddr oya = 0x80000000ULL + (hwaddr)r[st / 4] * 16;
            const hwaddr oca = 0x80000000ULL + (hwaddr)r[(st + 0x10) / 4] * 16;
            g_autofree uint8_t *line = g_malloc(ow);
            for (int y = 0; y < oh; y++) {
                const uint8_t *src = iy + (size_t)(y * h / oh) * ys;
                for (int x = 0; x < ow; x++) {
                    line[x] = src[x * w / ow];
                }
                dma_memory_write(&address_space_memory, oya + (hwaddr)y * oys,
                                 line, ow, MEMTXATTRS_UNSPECIFIED);
            }
            for (int y = 0; y < oh / 2; y++) {
                const uint8_t *src = ic + (size_t)(y * (h / 2) / (oh / 2)) * cs;
                for (int x = 0; x < ow / 2; x++) {
                    const int sx = 2 * (x * (w / 2) / (ow / 2));
                    line[2 * x] = src[sx];
                    line[2 * x + 1] = src[sx + 1];
                }
                dma_memory_write(&address_space_memory, oca + (hwaddr)y * ocs,
                                 line, ow & ~1, MEMTXATTRS_UNSPECIFIED);
            }
        }
    }
    s->vpss_jobs++;
}

/*
 * One job completes.  vpss.ko can start a job while one is still running:
 * with two CPUs a start outside the interrupt handler overlaps the
 * handler's own, and each start wants its completion; a timer re-armed
 * for the second lost the first's, and the pipeline stalled for good.
 */
static void xm_isp2_vpss_done(void *opaque)
{
    XmIsp2State *s = XM_ISP2(opaque);

    if (s->vpss_pending) {
        s->vpss_pending--;
    }
    s->vpss[VPSS_START / 4] = s->vpss_pending != 0;    /* 1 while busy */
    s->vpss[VPSS_INT / 4] = 1;
    xm_isp2_update(s);
    if (s->vpss_pending) {
        timer_mod(s->vpss_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  2 * SCALE_MS);
    }
}

static void xm_isp2_vi_written(XmIsp2State *s, hwaddr off, uint32_t val)
{
    if (off == RDMA_KICK && (val & 1) && s->bpu_started) {
        s->bpu_started = false;
        timer_mod(s->bpu_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  2 * SCALE_MS);
    }
}

static void xm_isp2_vpss_written(XmIsp2State *s, hwaddr off, uint32_t val)
{
    if (s->l->vpss_offline && off == VPSS_START && (val & 1)) {
        xm_isp2_vpss_job(s);
        if (!s->vpss_pending++) {
            timer_mod(s->vpss_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      2 * SCALE_MS);
        }
    }
}

#define XM_ISP2_TRACE(s, win, off, val)                                     \
    do {                                                                    \
        if ((s)->trace) {                                                   \
            qemu_log("xm-isp2: %s+0x%05" HWADDR_PRIx " = 0x%08" PRIx64 "\n",\
                     win, (hwaddr)(off), (uint64_t)(val));                 \
        }                                                                   \
    } while (0)

/* trace-reads: each offset's first read, with the reader's pc and mode. */
static void xm_isp2_trace_read(XmIsp2State *s, const char *win,
                               unsigned long *seen, hwaddr off, uint32_t val)
{
    if (!s->trace_reads || test_and_set_bit(off / 4, seen)) {
        return;
    }
    fprintf(stderr, "xm-isp2: read %s+0x%05" HWADDR_PRIx " = 0x%08x\n",
            win, off, val);
}

#define XM_ISP2_OPS(name, arr)                                              \
static uint64_t name##_read(void *opaque, hwaddr off, unsigned size)        \
{                                                                           \
    XmIsp2State *s = XM_ISP2(opaque);                                       \
    xm_isp2_trace_read(s, #arr, s->arr##_seen, off, s->arr[off / 4]);       \
    if (s->arr == s->vi && off == MIPI_IO_IN) {                             \
        return s->arr[off / 4] | MIPI_IO_UP;                                \
    }                                                                       \
    return s->arr[off / 4];                                                 \
}                                                                           \
static void name##_write(void *opaque, hwaddr off, uint64_t val,            \
                         unsigned size)                                     \
{                                                                           \
    XmIsp2State *s = XM_ISP2(opaque);                                       \
    XM_ISP2_TRACE(s, #arr, off, val);                                       \
    xm_regtrace("isp2." #arr, off, val);                                    \
    s->arr[off / 4] = val;                                                  \
    if (s->arr == s->vpss) {                                                \
        xm_isp2_vpss_written(s, off, val);                                  \
    } else if (s->arr == s->vi) {                                           \
        xm_isp2_vi_written(s, off, val);                                    \
    }                                                                       \
    xm_isp2_update(s);                                                      \
}                                                                           \
static const MemoryRegionOps name##_ops = {                                 \
    .read = name##_read,                                                    \
    .write = name##_write,                                                  \
    .endianness = DEVICE_LITTLE_ENDIAN,                                     \
    .valid.min_access_size = 4,                                             \
    .valid.max_access_size = 4,                                             \
};

/* An offline BPU job reads the queued Bayer frame and is done well within
 * a frame period. */
static void xm_isp2_bpu_done(void *opaque)
{
    XmIsp2State *s = XM_ISP2(opaque);

    if (s->l->vpss_inline) {
        xm_isp2_vpss_inline(s);
    }
    s->bpu[BPU_ST / 4] = 1;
    /* isp.ko may start the next channel's job before this one is done
     * (two sensors' frames arriving together): one completion per start */
    if (s->bpu_pending && --s->bpu_pending) {
        timer_mod(s->bpu_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  2 * SCALE_MS);
    }
    xm_isp2_update(s);
}

static uint64_t xm_isp2_bpu_read(void *opaque, hwaddr off, unsigned size)
{
    return XM_ISP2(opaque)->bpu[off / 4];
}

static void xm_isp2_bpu_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    XmIsp2State *s = XM_ISP2(opaque);

    XM_ISP2_TRACE(s, "bpu", off, val);
    xm_regtrace("isp2.bpu", off, val);
    s->bpu[off / 4] = val;
    if (off == BPU_START && val) {
        s->bpu_started = true;
        s->bpu_offline = true;
        if (!s->bpu_pending++) {
            timer_mod(s->bpu_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      2 * SCALE_MS);
        }
    }
    xm_isp2_update(s);
}

static const MemoryRegionOps xm_isp2_bpu_ops = {
    .read = xm_isp2_bpu_read,
    .write = xm_isp2_bpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

XM_ISP2_OPS(xm_isp2_vi, vi)
XM_ISP2_OPS(xm_isp2_vpss, vpss)

static void xm_isp2_realize(DeviceState *dev, Error **errp)
{
    XmIsp2State *s = XM_ISP2(dev);

    for (int i = 0; i < ARRAY_SIZE(xm_isp2_layouts); i++) {
        if (!strcmp(s->layout ? s->layout : "xm535", xm_isp2_layouts[i].name)) {
            s->l = &xm_isp2_layouts[i];
        }
    }
    if (!s->l) {
        error_setg(errp, "xm-isp2: unknown layout '%s'", s->layout);
        return;
    }
    if (!s->fps) {
        error_setg(errp, "xm-isp2: fps must not be 0");
        return;
    }
    s->store = s->l->store_in_vi ? s->vi : s->vpss;
}

static void xm_isp2_reset(DeviceState *dev)
{
    XmIsp2State *s = XM_ISP2(dev);

    memset(s->vi, 0, sizeof(s->vi));
    memset(s->bpu, 0, sizeof(s->bpu));
    memset(s->vpss, 0, sizeof(s->vpss));
    memset(s->vpss_group_ref, 0, sizeof(s->vpss_group_ref));
    s->bpu_started = false;
    s->vpss_pending = 0;
    timer_del(s->vpss_timer);
    timer_del(s->bpu_timer);
    s->bpu_pending = 0;
    s->frames = 0;
    s->end_phase = false;
    s->phase = 0;
    s->bpu_offline = false;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->fps);
    xm_isp2_update(s);
}

static void xm_isp2_init(Object *obj)
{
    XmIsp2State *s = XM_ISP2(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    memory_region_init_io(&s->vi_io, obj, &xm_isp2_vi_ops, s,
                          "xm-isp2.vi", VI_SIZE);
    memory_region_init_io(&s->bpu_io, obj, &xm_isp2_bpu_ops, s,
                          "xm-isp2.bpu", BPU_SIZE);
    memory_region_init_io(&s->vpss_io, obj, &xm_isp2_vpss_ops, s,
                          "xm-isp2.vpss", VPSS_SIZE);
    sysbus_init_mmio(sbd, &s->vi_io);
    sysbus_init_mmio(sbd, &s->bpu_io);
    sysbus_init_mmio(sbd, &s->vpss_io);
    for (n = 0; n < NUM_IRQ; n++) {
        sysbus_init_irq(sbd, &s->irq[n]);
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_isp2_frame, s);
    s->bpu_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_isp2_bpu_done, s);
    s->vpss_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_isp2_vpss_done, s);
}

static const Property xm_isp2_props[] = {
    DEFINE_PROP_UINT32("fps", XmIsp2State, fps, 12),
    DEFINE_PROP_STRING("layout", XmIsp2State, layout),
    DEFINE_PROP_UINT32("store-lines", XmIsp2State, store_lines, 4),
    DEFINE_PROP_UINT32("vicus", XmIsp2State, vicus, 2),
    DEFINE_PROP_UINT32("sensor-width", XmIsp2State, sns_w, 1920),
    DEFINE_PROP_UINT32("sensor-lines", XmIsp2State, sns_h, 1088),
    DEFINE_PROP_BOOL("trace", XmIsp2State, trace, false),
    DEFINE_PROP_BOOL("trace-reads", XmIsp2State, trace_reads, false),
};

static void xm_isp2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, xm_isp2_reset);
    dc->realize = xm_isp2_realize;
    device_class_set_props(dc, xm_isp2_props);
}

static const TypeInfo xm_isp2_info = {
    .name          = TYPE_XM_ISP2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmIsp2State),
    .instance_init = xm_isp2_init,
    .class_init    = xm_isp2_class_init,
};

static void xm_isp2_register_types(void)
{
    type_register_static(&xm_isp2_info);
}

type_init(xm_isp2_register_types)
