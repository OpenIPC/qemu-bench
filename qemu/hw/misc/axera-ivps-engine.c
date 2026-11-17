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
 * read in tdpdev_start(); other engines and jobs complete without
 * touching their output (the DPU draws OSD in place: nothing to do).
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
 *
 * Set AXERA_IVPS_DEBUG=1 to log every register write (the way the kick
 * registers above were found).
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

#define TYPE_AXERA_IVPS_ENGINE "axera-ivps-engine"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraIvpsEngineState, AXERA_IVPS_ENGINE)

#define ENGINE_SIZE   0x1000
#define ENGINE_DELAY_NS (200 * SCALE_US)
#define ENGINE_KICKS    4

enum { PIXEL_NONE, PIXEL_U_TDP, PIXEL_E_GDC, PIXEL_E_VPP };
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
};

/* The AX620E VPP, which the GDC streams into */
static AxeraIvpsEngineState *engine_vpp;

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

static int engine_tdp_worker(void *opaque)
{
    AxeraIvpsEngineState *s = opaque;

    if (s->pixel == PIXEL_E_GDC) {
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
    if (s->job_gen != s->gen) {
        return;
    }
    s->pending |= s->job_bits;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ENGINE_DELAY_NS);
    if (s->next_bits) {
        uint32_t bits = s->next_bits;

        s->next_bits = 0;
        engine_tdp_submit(s, s->next_regs, bits);
    }
}

static void engine_tdp_submit(AxeraIvpsEngineState *s, const uint32_t *regs,
                              uint32_t bits)
{
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
            if ((s->pixel == PIXEL_U_TDP || s->pixel == PIXEL_E_GDC) &&
                engine_async()) {
                engine_tdp_submit(s, s->regs, s->done_bits[k]);
                continue;
            } else if (s->pixel == PIXEL_U_TDP) {
                engine_tdp_job(s);
            } else if (s->pixel == PIXEL_E_GDC) {
                engine_gdc_job(s);
            }
            s->pending |= s->done_bits[k];
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ENGINE_DELAY_NS);
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

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->port, 0, sizeof(s->port));
    s->status = 0;
    s->pending = 0;
    s->next_bits = 0;
    s->gen++;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void engine_realize(DeviceState *dev, Error **errp)
{
    AxeraIvpsEngineState *s = AXERA_IVPS_ENGINE(dev);
    const char *dbg = getenv("AXERA_IVPS_DEBUG");

    s->debug = dbg && dbg[0] == '1';
    memory_region_init_io(&s->iomem, OBJECT(s), &engine_ops, s,
                          s->name ? s->name : TYPE_AXERA_IVPS_ENGINE,
                          ENGINE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, engine_complete, s);
    if (s->pixel == PIXEL_E_VPP) {
        engine_vpp = s;
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
