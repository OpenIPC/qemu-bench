/*
 * Axera AX620E IVPS processing engines: VPP @ 0x04403000, GDC @ 0x04404000,
 * TDP @ 0x04405000, and the DPU @ 0x04407000
 *
 * The camera stack's IVPS module drives these through the closed
 * ax_vpp.ko/ax_gdc.ko drivers as "cpucmd" lists: tables of
 * (value, register offset) pairs, one batch per tile/stage. The driver
 * writes a batch whose last store kicks the engine, then waits for the
 * engine's done interrupt; the IRQ handler acknowledges it and writes the
 * next batch (vpp_cpucmd_execute_*() / gdc_tile_cpucmd_execute_v2()),
 * and only wakes the IVPS thread once the list is exhausted. Without a
 * done interrupt the very first job never finishes and IVPS output stays
 * empty forever -- the App's XmCap_Vpss_getChnFrame() then fails with
 * 0x800d0120 (AX_ID_IVPS, AX_ERR_BUF_EMPTY) for the whole boot.
 *
 * Protocol, from disassembling the real AArch64 ax_vpp.ko/ax_gdc.ko IRQ
 * handlers shipped in the AX620E firmware:
 *
 *   VPP: four channels, each with its own kick register and done bit in
 *        the status register 0x888; each IRQ handler acks its own bit by
 *        writing it to 0x8b4 (write-1-to-clear). The list builders queue
 *        "ack, enable, kick" as each batch's last three stores:
 *          v0   enable 0x890, kick 0x004, done bit 0
 *          v1   enable 0x894, kick 0x038, done bit 3
 *          isp  enable 0x898, kick 0x06c, done bit 6
 *          gdc  enable 0x89c, kick 0x07c, done bit 9 (the GDC-linked path)
 *   TDP: status 0x004, bit 0 = done (bits 1/2 are other events);
 *        tdpdev_hw_start() writes 0x010=3 (ack), 0x00c=2 (mask), then
 *        kicks with 0x01c=1, and tdp_irq() acks by writing the status
 *        bits to 0x010 (write-1-to-clear).
 *   DPU (vo@4407000, the OSD/overlay stage IVPS runs on every output
 *        frame): ax620e_draw_start() writes a (offset, value) list whose
 *        last two stores are 0x024=1 and the kick 0x028=1; dpu_irq() reads
 *        status 0x004 (bit 0 = draw done, routed to draw_irq_handler) and
 *        acks by writing the same value to 0x014. Without the done
 *        interrupt each draw only finishes on the driver's own timeout,
 *        so a frame or two trickles out and VENC starves.
 *   GDC: status 0xd8, bits [31:16], bit 16 = frame done; 0xd4 bit 16 is
 *        the interrupt enable (written 0x10000 at init and before each
 *        kick) and writing 0x10001 there acks. Kicked by writing 1 to
 *        0x06c (last store of the batch).
 *
 * Every register is plain storage (the drivers read back what they
 * wrote); only the kick, status and ack registers have behaviour. A kick
 * completes after a short delay rather than synchronously, so the done
 * interrupt never arrives before the driver has finished its own
 * bookkeeping for the kick.
 *
 * Pixel work ("pixel" property), from the job registers traced live and
 * read in tdpdev_start() / ax620e_draw_prepare() / ax620a_draw_start();
 * other engines and jobs complete without touching their output.
 *
 *   1  AX620U TDP: copy-scale an NV12 picture on each kick. Addresses
 *      are (physical - 0x40000000) >> 4, strides / 16:
 *        0x020 source size h << 16 | w, 0x028 source compressed,
 *        0x030 / 0x038 source Y / UV, 0x034 / 0x03c their strides,
 *        0x040 bit 0 scale enable, 0x044 output size,
 *        0x3d0 / 0x3d8 output Y / UV, 0x3d4 / 0x3dc their strides.
 *   2  AX620E GDC: its source is the ISP's picture, which it streams
 *      tile by tile into VPP (TDP is never used for frames there):
 *        0x074 / 0x078 source Y low / high, 0x07c / 0x080 UV,
 *        0x044 source size, 0x09c bits [15:0] source stride / 8.
 *      Each kick of the first tile has VPP scale the whole picture.
 *   3  AX620E VPP: up to four output ports, port k's block at
 *      0x76c + 0x30 k: +0x00 enable, +0x04 size, +0x08 format (0xa NV12),
 *      +0x0c / +0x10 Y low / high, +0x14 Y stride, +0x18 / +0x1c UV,
 *      +0x20 UV stride, +0x24 compression (nonzero: the frame-buffer
 *      format the encoder reads as its input format 6). The driver
 *      writes each port's whole-frame geometry while the port is
 *      disabled, then per tile enables it and overwrites size and
 *      address with the tile's; 0x000 is the tile number, 1 first.
 *      The whole-frame geometry is what is kept and written.
 *   4  AX620E DPU: IVPS draws each OSD region (AX_IVPS_RGN, the App's
 *      time stamp and title) with one kick: layer 1, an RGB bitmap,
 *      blended over layer 0, the picture, into the output rectangle,
 *      in place (layer 0 = output). Addresses are 37-bit hi/lo pairs:
 *        0x02c rectangle size h << 16 | w,
 *        layer 0: 0x034 format (0xa NV12), 0x040/0x044 Y, 0x048 stride,
 *          0x04c/0x050 UV, 0x054 stride, 0x058 compressed, 0x060 origin,
 *        layer 1: 0x178 bit 0 enable, bit 1 0x19c alpha used,
 *          0x0d0 size, 0x16c position in the rectangle, 0x0d4 format
 *          (vo_fmt2hw_fmt: 0 ARGB1555, 1 ARGB4444, 3 ARGB8888,
 *          4/5/7 RGBA5551/4444/8888, 8 RGB565, 0x1f bitmap),
 *          0x0dc/0x0e0 pixels, 0x0e4 stride in bytes,
 *        output: 0x240 format, 0x244/0x248 Y, 0x24c stride,
 *          0x250/0x254 UV, 0x258 stride, 0x25c compressed,
 *          0x264 the rectangle's origin in a compressed frame.
 *      Uncompressed, the addresses already point at the rectangle; a
 *      compressed frame (the main stream's) is drawn on in its
 *      axera-fbuf copy, at the origin.
 *   5  AX620U DPU: the same per-region job, ax620a_draw_start()'s
 *      layout; addresses (physical - 0x40000000) >> 4, strides / 16:
 *        0x024 output size, 0x18c / 0x194 output Y / UV,
 *        0x190 / 0x198 their strides,
 *        YUV layer 0: 0x02c enable, 0x034 format (0 NV12), 0x038 /
 *          0x044 Y / UV, 0x040 / 0x04c strides,
 *        RGB layers at 0x0bc and 0x0f8: +0x00 enable, +0x04 position,
 *          +0x08 size, +0x0c format (1 ARGB1555), +0x14 pixels,
 *          +0x1c stride.
 *      The overlay is converted to BT.601 limited range and alpha
 *      blended: luma per pixel, chroma per 2x2 block.
 *
 * Set AXERA_IVPS_DEBUG=1 to log every register write (the way the kick
 * registers above were found), or to engine names (axera-dpu) to log
 * only theirs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "block/aio.h"
#include "block/thread-pool.h"
#include "hw/misc/axera-fbuf.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_AXERA_IVPS_ENGINE "axera-ivps-engine"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraIvpsEngineState, AXERA_IVPS_ENGINE)

#define ENGINE_SIZE   0x1000
#define ENGINE_DELAY_NS (200 * SCALE_US)
#define ENGINE_KICKS    4

enum { PIXEL_NONE, PIXEL_U_TDP, PIXEL_E_GDC, PIXEL_E_VPP, PIXEL_E_DPU,
       PIXEL_U_DPU };
#define VPP_PORTS       4
#define VPP_PORT(k)     (0x76c + 0x30 * (k))
#define VPP_PORT_SIZE   0x30

enum {
    ACK_W1C = 0,        /* VPP: write status bits to the ack register */
    ACK_BIT0_ENABLE = 1 /* GDC: ack = bit 0, enable = bits [31:16] */
};

struct AxeraIvpsEngineState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[ENGINE_SIZE / 4];
    uint32_t status;
    uint32_t pending;
    bool debug;

    char *name;
    uint32_t kick_off[ENGINE_KICKS];
    uint32_t done_bits[ENGINE_KICKS];
    uint32_t status_off;
    uint32_t ack_off;
    uint32_t ack_mode;
    uint32_t pixel;
    uint32_t port[VPP_PORTS][VPP_PORT_SIZE / 4];   /* VPP whole frame */

    /*
     * AX620U TDP jobs (a whole-picture NV12 load, scale and store, up to
     * 4K) run in a worker thread on a snapshot of the registers, and the
     * done bit is only posted once the worker is finished. Run inside
     * the kick's MMIO write they held the BQL 40-60 ms each, hundreds of
     * ms on a loaded host. Meanwhile no other vCPU could take an
     * interrupt, so the SPI-NAND driver's DMA completion (GIC SPI 5, to
     * CPU0) missed its 200 ms jiffies deadline: "polling_xfer_completion:
     * timeout", squashfs read errors (the App's std::ios_failure abort),
     * UBI write errors (UBIFS read-only). AXERA_IVPS_SYNC=1 restores the
     * synchronous path.
     *
     * The AX620E GDC's job (a 2688x1520 load scaled into up to four VPP
     * ports, one of them 3200x1800: ~20-25 ms a kick, 25-30 kicks a
     * second) runs the same way. Inside the kick it ran on the vCPU
     * thread and took most of that thread's time: with the guest on
     * one CPU (before CPU1 came online) IVPS, VENC and the App fell
     * behind a 30 fps VIN, IVPS's output allocation failed
     * (/proc/ax_proc/ivps BlkMiss) and, with the host loaded, the main
     * stream stopped for minutes ("zero bitrate (chn:0 main:enc_stop)",
     * MotionDetect's AX_ERR_IVPS_BUF_EMPTY). The kick captures the
     * source and the port geometry; the done interrupt follows the
     * worker, so nothing downstream reads a port before it is written.
     */
    uint32_t job_regs[ENGINE_SIZE / 4];
    uint32_t job_bits;          /* done bits of the job in the worker */
    uint32_t next_bits;         /* kicked again while busy: run next */
    /*
     * AX620E GDC: done bits of a frame's later tile kicks (empty jobs,
     * see engine_gdc_capture()), posted with the done of the frame's job
     * in the worker (job_tail) or of the one queued after it (next_tail).
     */
    uint32_t job_tail, next_tail;
    uint32_t next_regs[ENGINE_SIZE / 4];
    bool busy;
    uint32_t gen;               /* bumped on reset: drop a stale result */
    uint32_t job_gen;

    /*
     * AX620E GDC jobs: the source and VPP port geometry captured at the
     * kick, for the worker (see engine_gdc_capture()).
     */
    struct AxeraGdcJob {
        AxeraNv12Buf src;
        AxeraNv12Buf dst[VPP_PORTS];
        int ndst;
    } gdc_job, gdc_next;
    /*
     * AX620E: the DPU whose kick waits for this GDC job (its regs and
     * done bits in the DPU's next_regs / next_bits, see
     * engine_dpu_defer()).
     */
    AxeraIvpsEngineState *waiter;
    /*
     * AX620E DPU: a kick held for the GDC (held) is given up after
     * dpu-hold-ms of guest time (hold_timer), see engine_dpu_defer().
     */
    bool held;
    uint32_t hold_ms;
    QEMUTimer *hold_timer;
};

/* The AX620E VPP, which the GDC streams into, and the GDC */
static AxeraIvpsEngineState *engine_vpp, *engine_gdc;

static void engine_update_irq(AxeraIvpsEngineState *s)
{
    uint32_t live = s->status;

    if (s->ack_mode == ACK_BIT0_ENABLE) {
        live &= s->regs[s->ack_off / 4] & 0xffff0000u;
    }
    qemu_set_irq(s->irq, live != 0);
}

static void engine_complete(void *opaque)
{
    AxeraIvpsEngineState *s = opaque;

    s->status |= s->pending;
    s->pending = 0;
    engine_update_irq(s);
}

static uint32_t engine_reg(AxeraIvpsEngineState *s, hwaddr off)
{
    return s->regs[off / 4];
}

static hwaddr engine_addr64(AxeraIvpsEngineState *s, hwaddr lo)
{
    return engine_reg(s, lo) | ((hwaddr)engine_reg(s, lo + 4) << 32);
}

static void engine_tdp_job_regs(const uint32_t *regs);

static void engine_tdp_job(AxeraIvpsEngineState *s)
{
    engine_tdp_job_regs(s->regs);
}

static void engine_gdc_run(struct AxeraGdcJob *job);

static void engine_dpu_run(const uint32_t *regs, uint32_t pixel);
static void engine_gdc_release(AxeraIvpsEngineState *g);
static bool engine_is_dpu(AxeraIvpsEngineState *s)
{
    return s->pixel == PIXEL_E_DPU || s->pixel == PIXEL_U_DPU;
}

/*
 * A DPU job drawing into a compressed frame's axera-fbuf copy runs in a
 * worker: the copy's lock is also held by the GDC worker while it
 * stores and scales whole frames, and waiting for it under the BQL
 * stalled the guest. Any other DPU job (one OSD region, at most a few
 * hundred KB of guest memory) runs inside the kick: done in a worker,
 * a late write (the IVPS driver giving up on a slow job under host
 * load) can land in a buffer already reused for another frame.
 */
static bool engine_dpu_on_fbuf(AxeraIvpsEngineState *s, const uint32_t *regs)
{
    return s->pixel == PIXEL_E_DPU && (regs[0x25c / 4] & 1);
}

static int engine_tdp_worker(void *opaque)
{
    AxeraIvpsEngineState *s = opaque;

    if (engine_is_dpu(s)) {
        engine_dpu_run(s->job_regs, s->pixel);
    } else if (s->pixel == PIXEL_E_GDC) {
        engine_gdc_run(&s->gdc_job);
    } else {
        engine_tdp_job_regs(s->job_regs);
    }
    return 0;
}

static void engine_tdp_submit(AxeraIvpsEngineState *s, const uint32_t *regs,
                              uint32_t bits);
static void engine_gdc_capture(AxeraIvpsEngineState *s,
                               struct AxeraGdcJob *job);

/* Main loop, BQL held */
static void engine_tdp_done(void *opaque, int ret)
{
    AxeraIvpsEngineState *s = opaque;

    s->busy = false;
    /*
     * A job from before a reset posts nothing (engine_reset() waits for
     * it, so this is only a safety net); the empty tiles behind it and
     * a kick queued meanwhile are newer and go ahead regardless.
     */
    if (s->job_gen == s->gen) {
        s->pending |= s->job_bits;
    }
    s->pending |= s->job_tail;
    s->job_tail = 0;
    if (s->pending) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ENGINE_DELAY_NS);
    }
    if (s->next_bits && !s->held) {    /* a held DPU kick waits on */
        uint32_t bits = s->next_bits;

        s->next_bits = 0;
        engine_tdp_submit(s, s->next_regs, bits);
        s->job_tail = s->next_tail;
        s->next_tail = 0;
    }
    if (s->waiter) {
        engine_gdc_release(s);
    }
}

/*
 * AX620E GDC: the driver kicks a frame's tiles back to back; only the
 * first carries the frame (the worker renders it whole), the others are
 * empty jobs. Their done must not overtake the frame's: run as their own
 * jobs behind a busy worker, they completed after the previous frame's
 * job and before the current one, so ax_gdc took a frame as done (and
 * IVPS handed it to the DPU, VENC) while the worker was still writing
 * it. Merged into the queued job's registers, a later empty tile also
 * replaced a queued frame. Now an empty tile's done joins the done of
 * the frame job ahead of it (or is posted at once if there is none).
 */
static bool engine_gdc_tail(AxeraIvpsEngineState *s, const uint32_t *regs,
                            uint32_t bits)
{
    struct AxeraGdcJob job;

    if (s->pixel != PIXEL_E_GDC || regs == s->next_regs) {
        return false;
    }
    engine_gdc_capture(s, &job);
    if (job.ndst) {
        return false;
    }
    if (s->next_bits) {
        s->next_tail |= bits;
    } else if (s->busy) {
        s->job_tail |= bits;
    } else {
        s->pending |= bits;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ENGINE_DELAY_NS);
    }
    return true;
}

static void engine_tdp_submit(AxeraIvpsEngineState *s, const uint32_t *regs,
                              uint32_t bits)
{
    if (engine_gdc_tail(s, regs, bits)) {
        return;
    }
    if (s->busy) {
        memcpy(s->next_regs, regs, sizeof(s->next_regs));
        if (s->pixel == PIXEL_E_GDC) {
            engine_gdc_capture(s, &s->gdc_next);
        }
        s->next_bits |= bits;
        return;
    }
    memcpy(s->job_regs, regs, sizeof(s->job_regs));
    if (s->pixel == PIXEL_E_GDC) {
        if (regs == s->next_regs) {
            s->gdc_job = s->gdc_next;
        } else {
            engine_gdc_capture(s, &s->gdc_job);
        }
    }
    s->job_bits = bits;
    s->job_gen = s->gen;
    s->busy = true;
    thread_pool_submit_aio(engine_tdp_worker, s, engine_tdp_done, s);
}

static bool engine_async(void)
{
    static int on = -1;

    if (on < 0) {
        on = !getenv("AXERA_IVPS_SYNC");
    }
    return on;
}

#define TDP_REG(off) (regs[(off) / 4])
/* AX620U TDP: (physical - 0x40000000) >> 4 */
#define TDP_ADDR(off) (0x40000000 + ((hwaddr)TDP_REG(off) << 4))

static void engine_tdp_job_regs(const uint32_t *regs)
{
    uint32_t ssz = TDP_REG(0x020);
    uint32_t dsz = (TDP_REG(0x040) & 1) ? TDP_REG(0x044) : ssz;
    AxeraNv12Buf src = {
        .y = TDP_ADDR(0x030), .uv = TDP_ADDR(0x038),
        .w = ssz & 0xffff, .h = ssz >> 16,
        .y_stride = TDP_REG(0x034) << 4,
        .uv_stride = TDP_REG(0x03c) << 4,
        .compressed = TDP_REG(0x028) != 0,
    };
    AxeraNv12Buf dst = {
        .y = TDP_ADDR(0x3d0), .uv = TDP_ADDR(0x3d8),
        .w = dsz & 0xffff, .h = dsz >> 16,
        .y_stride = TDP_REG(0x3d4) << 4,
        .uv_stride = TDP_REG(0x3dc) << 4,
    };
    AxeraPicture in = { 0 }, out = { 0 };

    if (!src.w || !src.h || !dst.w || !dst.h || src.w > 8192 ||
        src.h > 8192 || dst.w > 8192 || dst.h > 8192 ||
        !TDP_REG(0x030) || !TDP_REG(0x3d0)) {
        return;
    }
    if (axera_nv12_load(&src, &in)) {
        axera_picture_alloc(&out, dst.w, dst.h);
        axera_picture_scale(&in, &out);
        axera_fbuf_drop(dst.y);
        axera_nv12_store(&dst, &out);
    }
    axera_picture_free(&in);
    axera_picture_free(&out);
}

/*
 * AX620E: the GDC kicked with VPP on its first tile: capture what fills
 * VPP's ports -- the source picture and every enabled port's whole-frame
 * geometry. An empty job (other tiles, nothing enabled) has ndst 0.
 */
static void engine_gdc_capture(AxeraIvpsEngineState *s,
                               struct AxeraGdcJob *job)
{
    AxeraIvpsEngineState *v = engine_vpp;
    uint32_t ssz = engine_reg(s, 0x044);
    AxeraNv12Buf src = {
        .y = engine_addr64(s, 0x074), .uv = engine_addr64(s, 0x07c),
        .w = ssz & 0xffff, .h = ssz >> 16,
        .y_stride = (engine_reg(s, 0x09c) & 0xffff) << 3,
        .uv_stride = (engine_reg(s, 0x09c) & 0xffff) << 3,
    };
    int k;

    job->ndst = 0;
    if (!v || engine_reg(v, 0x000) != 1 || !src.y || !src.w || !src.h ||
        src.w > 8192 || src.h > 8192) {
        return;
    }
    job->src = src;
    for (k = 0; k < VPP_PORTS; k++) {
        const uint32_t *p = v->port[k];
        uint32_t dsz = p[0x04 / 4];
        AxeraNv12Buf dst = {
            .y = p[0x0c / 4] | ((hwaddr)p[0x10 / 4] << 32),
            .uv = p[0x18 / 4] | ((hwaddr)p[0x1c / 4] << 32),
            .w = dsz & 0xffff, .h = dsz >> 16,
            .y_stride = p[0x14 / 4], .uv_stride = p[0x20 / 4],
            .compressed = p[0x24 / 4] != 0,
        };

        if (!(engine_reg(v, VPP_PORT(k)) & 1) || !dst.y || !dst.w ||
            !dst.h || dst.w > 8192 || dst.h > 8192 ||
            (!dst.compressed && (!dst.uv || dst.y_stride < dst.w))) {
            continue;
        }
        job->dst[job->ndst++] = dst;
    }
}

/* Scale the source into each captured port: no device state touched */
static void engine_gdc_run(struct AxeraGdcJob *job)
{
    AxeraPicture in = { 0 }, out = { 0 };
    int k;

    if (!job->ndst) {
        return;
    }
    job->src.compressed = axera_fbuf_has(job->src.y);
    if (axera_nv12_load(&job->src, &in)) {
        for (k = 0; k < job->ndst; k++) {
            AxeraNv12Buf *dst = &job->dst[k];

            axera_picture_alloc(&out, dst->w, dst->h);
            axera_picture_scale(&in, &out);
            if (!dst->compressed) {
                axera_fbuf_drop(dst->y);
            }
            axera_nv12_store(dst, &out);
        }
    }
    axera_picture_free(&in);
    axera_picture_free(&out);
}

static void engine_gdc_job(AxeraIvpsEngineState *s)
{
    struct AxeraGdcJob job;

    engine_gdc_capture(s, &job);
    engine_gdc_run(&job);
}

/*
 * DPU draw jobs: blend up to two RGB overlays (the App's OSD regions)
 * onto a rectangle of an NV12 picture. See the header comment for the
 * register layouts.
 */
enum {
    OVL_NONE, OVL_ARGB1555, OVL_ARGB4444, OVL_ARGB8888, OVL_RGBA5551,
    OVL_RGBA4444, OVL_RGBA8888, OVL_RGB565,
};
#define DPU_OVERLAYS 2

typedef struct DpuOverlay {
    hwaddr addr;
    uint32_t stride, w, h;
    int32_t x, y;               /* position in the job's rectangle */
    int fmt;
    uint32_t galpha;            /* 0..255, scales every pixel's alpha */
    uint8_t *data;
} DpuOverlay;

typedef struct DpuJob {
    uint32_t w, h;              /* the rectangle drawn */
    AxeraNv12Buf bg, out;       /* rectangle, or whole compressed frame */
    uint32_t bg_x, bg_y, out_x, out_y;  /* its origin in a compressed one */
    bool have_bg;               /* else out is drawn on in place */
    DpuOverlay ov[DPU_OVERLAYS];
    int nov;
} DpuJob;

static int ovl_bpp(int fmt)
{
    return fmt == OVL_ARGB8888 || fmt == OVL_RGBA8888 ? 4 : 2;
}

static void ovl_pixel(const DpuOverlay *o, const uint8_t *p,
                      uint32_t *r, uint32_t *g, uint32_t *b, uint32_t *a)
{
    uint32_t v = o->fmt == OVL_ARGB8888 || o->fmt == OVL_RGBA8888 ?
                 ldl_le_p(p) : lduw_le_p(p);

    switch (o->fmt) {
    case OVL_ARGB1555:
        *a = v & 0x8000 ? 255 : 0;
        *r = (v >> 10) & 31; *g = (v >> 5) & 31; *b = v & 31;
        *r = *r << 3 | *r >> 2; *g = *g << 3 | *g >> 2; *b = *b << 3 | *b >> 2;
        break;
    case OVL_RGBA5551:
        *a = v & 1 ? 255 : 0;
        *r = (v >> 11) & 31; *g = (v >> 6) & 31; *b = (v >> 1) & 31;
        *r = *r << 3 | *r >> 2; *g = *g << 3 | *g >> 2; *b = *b << 3 | *b >> 2;
        break;
    case OVL_ARGB4444:
        *a = (v >> 12) * 17; *r = ((v >> 8) & 15) * 17;
        *g = ((v >> 4) & 15) * 17; *b = (v & 15) * 17;
        break;
    case OVL_RGBA4444:
        *r = (v >> 12) * 17; *g = ((v >> 8) & 15) * 17;
        *b = ((v >> 4) & 15) * 17; *a = (v & 15) * 17;
        break;
    case OVL_ARGB8888:
        *a = v >> 24; *r = (v >> 16) & 255; *g = (v >> 8) & 255; *b = v & 255;
        break;
    case OVL_RGBA8888:
        *r = v >> 24; *g = (v >> 16) & 255; *b = (v >> 8) & 255; *a = v & 255;
        break;
    default: /* OVL_RGB565 */
        *a = 255;
        *r = (v >> 11) & 31; *g = (v >> 5) & 63; *b = v & 31;
        *r = *r << 3 | *r >> 2; *g = *g << 2 | *g >> 4; *b = *b << 3 | *b >> 2;
        break;
    }
    *a = *a * o->galpha / 255;
}

/*
 * Blend overlay @o, at (@x0 + o->x, @y0 + o->y) in @pic, clipped to the
 * job's @w x @h rectangle at (@x0, @y0): RGB to BT.601 limited-range
 * YCbCr, luma per pixel, chroma per 2x2 block with the block's alphas.
 */
static void dpu_blend(AxeraPicture *pic, uint32_t x0, uint32_t y0,
                      uint32_t w, uint32_t h, const DpuOverlay *o)
{
    int bpp = ovl_bpp(o->fmt);
    int32_t cx0 = MAX(o->x, 0), cy0 = MAX(o->y, 0);
    int32_t cx1 = MIN(o->x + (int32_t)o->w, (int32_t)w);
    int32_t cy1 = MIN(o->y + (int32_t)o->h, (int32_t)h);

    cx1 = MIN(cx1, (int32_t)pic->w - (int32_t)x0);
    cy1 = MIN(cy1, (int32_t)pic->h - (int32_t)y0);
    for (int32_t y = cy0 & ~1; y < cy1; y += 2) {
        for (int32_t x = cx0 & ~1; x < cx1; x += 2) {
            int32_t su = 0, sv = 0;
            uint8_t *uv = pic->uv + (size_t)((y0 + y) / 2) * pic->w +
                          ((x0 + x) & ~1u);

            for (int k = 0; k < 4; k++) {
                int32_t px = x + (k & 1), py = y + (k >> 1);
                uint32_t r, g, b, a;
                uint8_t *yp;
                int32_t yy;

                if (px < cx0 || px >= cx1 || py < cy0 || py >= cy1) {
                    continue;
                }
                ovl_pixel(o, o->data + (size_t)(py - o->y) * o->stride +
                          (size_t)(px - o->x) * bpp, &r, &g, &b, &a);
                if (!a) {
                    continue;
                }
                yp = pic->y + (size_t)(y0 + py) * pic->w + x0 + px;
                yy = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
                *yp += (int32_t)(yy - *yp) * (int32_t)a / 255;
                su += ((((int32_t)(-38 * r - 74 * g + 112 * b) + 128) >> 8) +
                       128 - uv[0]) * (int32_t)a;
                sv += ((((int32_t)(112 * r - 94 * g - 18 * b) + 128) >> 8) +
                       128 - uv[1]) * (int32_t)a;
            }
            uv[0] += su / (4 * 255);
            uv[1] += sv / (4 * 255);
        }
    }
}

typedef struct DpuDraw {
    const DpuJob *job;
    uint32_t x0, y0;
} DpuDraw;

static void dpu_draw_cb(AxeraPicture *pic, void *opaque)
{
    DpuDraw *d = opaque;

    for (int i = 0; i < d->job->nov; i++) {
        dpu_blend(pic, d->x0, d->y0, d->job->w, d->job->h, &d->job->ov[i]);
    }
}

static void dpu_job_run(DpuJob *job)
{
    AxeraPicture pic = { 0 };
    DpuDraw d = { .job = job };
    int i;

    for (i = 0; i < job->nov; i++) {
        DpuOverlay *o = &job->ov[i];
        size_t len = (size_t)o->stride * (o->h - 1) + o->w * ovl_bpp(o->fmt);

        o->data = g_malloc(len);
        dma_memory_read(&address_space_memory, o->addr, o->data, len,
                        MEMTXATTRS_UNSPECIFIED);
    }
    if (job->out.compressed) {
        /* in place on the frame kept for the compressed buffer */
        d.x0 = job->out_x;
        d.y0 = job->out_y;
        if (job->have_bg && job->bg.y != job->out.y) {
            AxeraPicture bg = { 0 };

            if (axera_fbuf_copy(job->bg.y, &bg)) {
                axera_fbuf_put(job->out.y, &bg);
            }
            axera_picture_free(&bg);
        }
        axera_fbuf_edit(job->out.y, dpu_draw_cb, &d);
    } else {
        AxeraNv12Buf *src = job->have_bg ? &job->bg : &job->out;

        if (!src->compressed && axera_nv12_load(src, &pic)) {
            dpu_draw_cb(&pic, &d);
            axera_nv12_store(&job->out, &pic);
        }
        axera_picture_free(&pic);
    }
    for (i = 0; i < job->nov; i++) {
        g_free(job->ov[i].data);
    }
}

static bool dpu_ovl_valid(const DpuOverlay *o)
{
    return o->fmt != OVL_NONE && o->addr && o->w && o->h && o->w <= 8192 &&
           o->h <= 8192 && o->stride >= o->w * ovl_bpp(o->fmt) &&
           o->stride <= 32768;
}

#define DPU_REG(off) (regs[(off) / 4])
#define DPU_ADDR64(hi, lo) (((hwaddr)(DPU_REG(hi) & 0x1f) << 32) | DPU_REG(lo))
/* AX620U: (physical - 0x40000000) >> 4, like its TDP */
#define DPU_ADDR_U(off) (0x40000000 + ((hwaddr)DPU_REG(off) << 4))

static bool dpu_capture_e(const uint32_t *regs, DpuJob *job)
{
    static const int fmts[16] = {
        [0] = OVL_ARGB1555, [1] = OVL_ARGB4444, [3] = OVL_ARGB8888,
        [4] = OVL_RGBA5551, [5] = OVL_RGBA4444, [7] = OVL_RGBA8888,
        [8] = OVL_RGB565,
    };
    uint32_t sz = DPU_REG(0x02c);
    DpuOverlay *o = &job->ov[0];

    job->w = sz & 0xffff;
    job->h = sz >> 16;
    /* output and layer 0 (the picture under the overlay): NV12 only */
    if (DPU_REG(0x240) != 0xa || !(DPU_REG(0x178) & 1)) {
        return false;
    }
    job->out = (AxeraNv12Buf) {
        .y = DPU_ADDR64(0x244, 0x248), .uv = DPU_ADDR64(0x250, 0x254),
        .y_stride = DPU_REG(0x24c), .uv_stride = DPU_REG(0x258),
        .w = job->w, .h = job->h, .compressed = DPU_REG(0x25c) & 1,
    };
    job->out_x = DPU_REG(0x264) & 0xffff;
    job->out_y = DPU_REG(0x264) >> 16;
    job->bg = (AxeraNv12Buf) {
        .y = DPU_ADDR64(0x040, 0x044), .uv = DPU_ADDR64(0x04c, 0x050),
        .y_stride = DPU_REG(0x048), .uv_stride = DPU_REG(0x054),
        .w = job->w, .h = job->h, .compressed = DPU_REG(0x058) & 1,
    };
    job->bg_x = DPU_REG(0x060) & 0xffff;
    job->bg_y = DPU_REG(0x060) >> 16;
    job->have_bg = DPU_REG(0x034) == 0xa && job->bg.y &&
                   (job->bg.y != job->out.y || job->bg_x != job->out_x ||
                    job->bg_y != job->out_y);
    /* layer 1: the overlay */
    *o = (DpuOverlay) {
        .addr = DPU_ADDR64(0x0dc, 0x0e0), .stride = DPU_REG(0x0e4),
        .w = DPU_REG(0x0d0) & 0xffff, .h = DPU_REG(0x0d0) >> 16,
        .x = DPU_REG(0x16c) & 0xffff, .y = DPU_REG(0x16c) >> 16,
        .fmt = fmts[DPU_REG(0x0d4) & 15],
        .galpha = DPU_REG(0x178) & 2 ? DPU_REG(0x19c) & 0xff : 255,
    };
    job->nov = dpu_ovl_valid(o);
    if (job->w > 8192 || job->h > 8192) {
        qemu_log_mask(LOG_GUEST_ERROR, "axera-dpu: draw rectangle %ux%u "
                      "too large, not drawn\n", job->w, job->h);
        return false;
    }
    return job->nov && job->out.y && job->w && job->h;
}

static bool dpu_capture_u(const uint32_t *regs, DpuJob *job)
{
    static const hwaddr rgb[DPU_OVERLAYS] = { 0x0bc, 0x0f8 };
    uint32_t sz = DPU_REG(0x024);
    int k;

    job->w = sz & 0xffff;
    job->h = sz >> 16;
    job->out = (AxeraNv12Buf) {
        .y = DPU_ADDR_U(0x18c), .uv = DPU_ADDR_U(0x194),
        .y_stride = DPU_REG(0x190) << 4, .uv_stride = DPU_REG(0x198) << 4,
        .w = job->w, .h = job->h,
    };
    /* YUV layer 0, NV12 (format 0) */
    job->bg = (AxeraNv12Buf) {
        .y = DPU_ADDR_U(0x038), .uv = DPU_ADDR_U(0x044),
        .y_stride = DPU_REG(0x040) << 4, .uv_stride = DPU_REG(0x04c) << 4,
        .w = job->w, .h = job->h,
    };
    job->have_bg = (DPU_REG(0x02c) & 1) && DPU_REG(0x034) == 0 &&
                   job->bg.y != job->out.y;
    job->nov = 0;
    for (k = 0; k < DPU_OVERLAYS; k++) {
        hwaddr b = rgb[k];
        DpuOverlay *o = &job->ov[job->nov];

        if (!(DPU_REG(b) & 1)) {
            continue;
        }
        *o = (DpuOverlay) {
            .addr = DPU_ADDR_U(b + 0x14), .stride = DPU_REG(b + 0x1c) << 4,
            .w = DPU_REG(b + 0x08) & 0xffff, .h = DPU_REG(b + 0x08) >> 16,
            .x = DPU_REG(b + 0x04) & 0xffff, .y = DPU_REG(b + 0x04) >> 16,
            .fmt = DPU_REG(b + 0x0c) == 1 ? OVL_ARGB1555 : OVL_NONE,
            .galpha = 255,
        };
        job->nov += dpu_ovl_valid(o);
    }
    return job->nov && job->w && job->h && job->w <= 8192 && job->h <= 8192;
}

static void engine_dpu_run(const uint32_t *regs, uint32_t pixel)
{
    DpuJob job = { 0 };

    if (pixel == PIXEL_E_DPU ? dpu_capture_e(regs, &job)
                             : dpu_capture_u(regs, &job)) {
        dpu_job_run(&job);
    }
}

static void engine_kick(AxeraIvpsEngineState *s, const uint32_t *regs,
                        uint32_t bits)
{
    if ((s->pixel == PIXEL_U_TDP || s->pixel == PIXEL_E_GDC ||
         engine_dpu_on_fbuf(s, regs)) && engine_async()) {
        engine_tdp_submit(s, regs, bits);
        return;
    } else if (engine_is_dpu(s)) {
        engine_dpu_run(regs, s->pixel);
    } else if (s->pixel == PIXEL_U_TDP) {
        engine_tdp_job(s);
    } else if (s->pixel == PIXEL_E_GDC) {
        engine_gdc_job(s);
    }
    s->pending |= bits;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ENGINE_DELAY_NS);
}

/* Whether the AX620E GDC job @job is writing the picture at @addr */
static bool engine_gdc_writes(const struct AxeraGdcJob *job, hwaddr addr)
{
    for (int k = 0; k < job->ndst; k++) {
        const AxeraNv12Buf *d = &job->dst[k];
        hwaddr end = d->y + (hwaddr)MAX(d->y_stride, d->w) * d->h;

        if (addr == d->y || (!d->compressed && addr > d->y && addr < end) ||
            (!d->compressed && addr >= d->uv &&
             addr < d->uv + (hwaddr)MAX(d->uv_stride, d->w) * d->h / 2)) {
            return true;
        }
    }
    return false;
}

/*
 * AX620E: IVPS hands a VPP output to the DPU once VPP is done with it,
 * but the picture is only written by the GDC worker (VPP's done bits
 * do not wait for it), so a draw could land before the picture and be
 * overwritten: hold the kick until that GDC job is done.
 */
static bool engine_dpu_defer(AxeraIvpsEngineState *s, const uint32_t *regs,
                             uint32_t bits)
{
    AxeraIvpsEngineState *g = engine_gdc;
    hwaddr out = ((hwaddr)(regs[0x244 / 4] & 0x1f) << 32) | regs[0x248 / 4];

    if (s->pixel != PIXEL_E_DPU || !g || !g->busy ||
        !(engine_gdc_writes(&g->gdc_job, out) ||
          (g->next_bits && engine_gdc_writes(&g->gdc_next, out)))) {
        return false;
    }
    if (regs != s->next_regs) {
        memcpy(s->next_regs, regs, sizeof(s->next_regs));
    }
    s->next_bits = bits;
    g->waiter = s;
    if (!s->held) {
        s->held = true;
        timer_mod_ns(s->hold_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     (int64_t)s->hold_ms * SCALE_MS);
    }
    return true;
}

/*
 * A held DPU kick whose GDC job is still not done after dpu-hold-ms of
 * guest time: complete it without drawing. A starved GDC worker can
 * fall far behind the guest on a loaded host, and ax_vo waits 300-500
 * ms for a draw (draw_worker_thread / draw_finished_thread); draws kept
 * in flight that long met its error and channel-reset paths
 * (refcount_t underflow in ciw_put, NULL + 0x40 in
 * draw_finished_thread). Losing the OSD on a late frame is the lesser
 * evil.
 */
static void engine_dpu_hold_expired(void *opaque)
{
    AxeraIvpsEngineState *s = opaque;

    if (!s->held) {
        return;
    }
    s->held = false;
    if (engine_gdc && engine_gdc->waiter == s) {
        engine_gdc->waiter = NULL;
    }
    if (s->debug) {
        qemu_log("%s: kick held %u ms for the GDC, done without drawing\n",
                 s->name, s->hold_ms);
    }
    s->pending |= s->next_bits;
    s->next_bits = 0;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ENGINE_DELAY_NS);
}

/* Main loop, BQL held: the GDC job a DPU kick waited for is done */
static void engine_gdc_release(AxeraIvpsEngineState *g)
{
    AxeraIvpsEngineState *s = g->waiter;
    uint32_t bits;

    g->waiter = NULL;
    if (!s || !s->held || !s->next_bits) {
        return;
    }
    bits = s->next_bits;
    s->next_bits = 0;
    if (!engine_dpu_defer(s, s->next_regs, bits)) {
        s->held = false;
        timer_del(s->hold_timer);
        engine_kick(s, s->next_regs, bits);
    }
}

static uint64_t engine_read(void *opaque, hwaddr off, unsigned size)
{
    AxeraIvpsEngineState *s = opaque;

    if (off == s->status_off) {
        return s->status;
    }
    return s->regs[off / 4];
}

static void engine_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    AxeraIvpsEngineState *s = opaque;

    if (s->debug) {
        qemu_log("%s: [0x%03" HWADDR_PRIx "] = 0x%08" PRIx64 "\n",
                 s->name, off, val);
    }
    s->regs[off / 4] = val;

    if (s->pixel == PIXEL_E_VPP && off >= VPP_PORT(0) &&
        off < VPP_PORT(VPP_PORTS)) {
        int k = (off - VPP_PORT(0)) / VPP_PORT_SIZE;

        /* whole-frame geometry: written while the port is disabled */
        if (!(engine_reg(s, VPP_PORT(k)) & 1) && off != VPP_PORT(k)) {
            s->port[k][(off - VPP_PORT(k)) / 4] = val;
        }
    }
    if (off == s->ack_off) {
        if (s->ack_mode == ACK_W1C) {
            s->status &= ~(uint32_t)val;
        } else if (val & 1) {
            s->status = 0;
        }
        engine_update_irq(s);
    }
    for (int k = 0; k < ENGINE_KICKS; k++) {
        if (s->done_bits[k] && off == s->kick_off[k] && (val & 1)) {
            if (engine_is_dpu(s) &&
                engine_dpu_defer(s, s->regs, s->done_bits[k])) {
                continue;
            }
            engine_kick(s, s->regs, s->done_bits[k]);
        }
    }
}

static const MemoryRegionOps engine_ops = {
    .read = engine_read,
    .write = engine_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void engine_reset(DeviceState *dev)
{
    AxeraIvpsEngineState *s = AXERA_IVPS_ENGINE(dev);

    /*
     * A pixel job in the worker writes guest memory and axera-fbuf
     * pictures: wait for it, so none of it lands after the reset (on a
     * new boot's buffers). The worker needs no BQL; its completion,
     * engine_tdp_done(), runs here from aio_poll() in the main loop.
     * Drop what is queued first and bump gen, so that completion
     * neither posts done bits nor starts another job.
     */
    s->next_bits = 0;
    s->job_tail = 0;
    s->next_tail = 0;
    s->waiter = NULL;
    s->held = false;
    s->gen++;
    while (s->busy) {
        aio_poll(qemu_get_aio_context(), true);
    }
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->port, 0, sizeof(s->port));
    s->status = 0;
    s->pending = 0;
    timer_del(s->timer);
    timer_del(s->hold_timer);
    qemu_set_irq(s->irq, 0);
}

static void engine_realize(DeviceState *dev, Error **errp)
{
    AxeraIvpsEngineState *s = AXERA_IVPS_ENGINE(dev);
    const char *dbg = getenv("AXERA_IVPS_DEBUG");

    s->debug = dbg && (!strcmp(dbg, "1") ||
                       (s->name && strstr(dbg, s->name)));
    memory_region_init_io(&s->iomem, OBJECT(s), &engine_ops, s,
                          s->name ? s->name : TYPE_AXERA_IVPS_ENGINE,
                          ENGINE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, engine_complete, s);
    s->hold_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, engine_dpu_hold_expired,
                                 s);
    if (s->pixel == PIXEL_E_VPP) {
        engine_vpp = s;
    } else if (s->pixel == PIXEL_E_GDC) {
        engine_gdc = s;
    }
}

static const Property engine_props[] = {
    DEFINE_PROP_STRING("name", AxeraIvpsEngineState, name),
    DEFINE_PROP_UINT32("kick-off0", AxeraIvpsEngineState, kick_off[0], 0),
    DEFINE_PROP_UINT32("kick-off1", AxeraIvpsEngineState, kick_off[1], 0),
    DEFINE_PROP_UINT32("kick-off2", AxeraIvpsEngineState, kick_off[2], 0),
    DEFINE_PROP_UINT32("kick-off3", AxeraIvpsEngineState, kick_off[3], 0),
    DEFINE_PROP_UINT32("done-bits0", AxeraIvpsEngineState, done_bits[0], 0),
    DEFINE_PROP_UINT32("done-bits1", AxeraIvpsEngineState, done_bits[1], 0),
    DEFINE_PROP_UINT32("done-bits2", AxeraIvpsEngineState, done_bits[2], 0),
    DEFINE_PROP_UINT32("done-bits3", AxeraIvpsEngineState, done_bits[3], 0),
    DEFINE_PROP_UINT32("status-off", AxeraIvpsEngineState, status_off, 0),
    DEFINE_PROP_UINT32("ack-off", AxeraIvpsEngineState, ack_off, 0),
    DEFINE_PROP_UINT32("ack-mode", AxeraIvpsEngineState, ack_mode, 0),
    DEFINE_PROP_UINT32("pixel", AxeraIvpsEngineState, pixel, 0),
    DEFINE_PROP_UINT32("dpu-hold-ms", AxeraIvpsEngineState, hold_ms, 200),
};

static void engine_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = engine_realize;
    device_class_set_legacy_reset(dc, engine_reset);
    device_class_set_props(dc, engine_props);
}

static const TypeInfo engine_info = {
    .name          = TYPE_AXERA_IVPS_ENGINE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraIvpsEngineState),
    .class_init    = engine_class_init,
};

static void engine_register_types(void)
{
    type_register_static(&engine_info);
}

type_init(engine_register_types)
