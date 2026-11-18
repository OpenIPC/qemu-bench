/*
 * Axera AX620E/AX620U VIN/ISP frame-completion interrupt (the real
 * "axera,proton" node at 0x02400000, DTB `interrupts = <0 0x1b 4  0 0x1c
 * 4>`, `interrupt-names = "isp0-0","isp0-1"`, i.e. GIC SPI 27 and 28).
 *
 * Found by live-tracing the real deployed ax_proton.ko with QEMU's own
 * gdbstub (not the SDK reference build -- confirmed a different build,
 * same mismatch pattern as ax_venc.ko elsewhere in this project) through
 * four layers of indirection: the camera app's "wait for frame" ioctl
 * (ax_vin_itp_frame_done_irq_get) -> ax_wrapper_vin_pipe_action_ctrl
 * (action 3) -> vin_pipe_common_action_ctrl (action 3) ->
 * vin_ipc_pipe_interrupt_query, which just blocks (AX_OSAL_SYNC_
 * wait_interruptible[_timeout]) on an atomic counter in the per-pipe
 * context struct until something increments it. That something is the
 * real hardware interrupt: __vin_int_handle (resolved via the static
 * handler-pointer relocation vin_irq_create writes into the device
 * struct, then __vin_probe_irq_register reads back and passes to
 * AX_OSAL_DEV_request_threaded_irq for both SPI 27 and 28, thread_fn
 * NULL -- a plain hard-IRQ handler, not a kernel-threaded one).
 *
 * __vin_int_handle's own register accesses (traced through
 * ax_isp_int_mask_get -> _raw_get/_sts_get/_clear_set down to the raw
 * ISP_DRV_IO_READ32/WRITE32, a plain base+offset dereference with no
 * further indirection) resolve to per-channel 16-byte blocks, all inside
 * this one 0x100000 region (matches the real DTB reg property exactly,
 * so this replaces the old passive axera-vin regbank rather than adding
 * a new region):
 *
 *   blk+0x0   MASK     R/W, one bit enables interest in that event
 *   blk+0x4   CLEAR    W,   write-1-to-clear the matching RAW bit
 *   blk+0x8   RAW      R,   hardware-latched pending bits (unmasked)
 *   blk+0xC   STATUS   R,   RAW & MASK (what the real ISR actually acts on)
 *
 * MASK POLARITY -- a plausible-looking inversion was tried and FALSIFIED
 * live this session, worth recording so it isn't re-tried from scratch:
 * `ax_isp_int_mask_set` (0xac78) does `orr` (sets bits) and
 * `ax_isp_int_unmask_set` (0xad40) does `bic` (clears bits), which reads
 * like the standard "1 = suppressed" interrupt-mask convention -- but
 * `__vin_int_handle` (0x7ed90), the real ISR, only acts on a channel when
 * `ax_isp_int_mask_get() & ax_isp_int_sts_get()` is nonzero (disassembly:
 * `and w3, w3, w1` on the two register reads, `cbz w3` skips the
 * ack/log block otherwise). That AND can only ever be nonzero if MASK
 * and STATUS are positively correlated (both 1 = this bit matters) --
 * if STATUS were really `RAW & ~MASK` as the "standard convention"
 * reading would imply, `MASK & STATUS` is identically zero and the ISR's
 * own per-channel ack logic would be dead code, which can't be right.
 * Flipping this file's polarity accordingly (STATUS/level computed as
 * `RAW & ~MASK`, tick latching RAW unconditionally) was built and
 * live-tested: it booted into an immediate IRQ storm/RCU stall hang
 * (`rcu_preempt kthread starved`, CPU0 permanently stuck inside
 * `__vin_irq_control`/`enable_irq` right as `ax_vin_glb_create` first
 * registers the IRQ) -- because under that polarity every channel left
 * at the natural post-reset MASK=0 reads as "fully pending" every tick,
 * so dozens of channels assert simultaneously forever and the guest
 * never gets a quiet edge. Reverted. The original (this) polarity is
 * correct for how the real ISR consumes these registers; it is NOT the
 * (still open) reason BUF_EMPTY persists for channel 2/range-1 -- see
 * the kaeru episode for this session for where that stands.
 *
 *   range 0 (assumed = SPI 27 = irq[0]):             blk = 0x10    + ch*16
 *   range 1 (assumed = SPI 28 = irq[1]), ch 0-7:      blk = 0x80010 + ch*16
 *   range 1, ch 8-15:                                 blk = 0xc0010 + (ch-8)*16
 *
 * Independently cross-checked: the AX620U machine's own comment (real
 * driver traffic observed at regbank offsets 0x20/0x30/0x40/0x50, and
 * the same four again at +0x80000) matches range 0's formula exactly for
 * channels 1-4.
 *
 * Two things are explicit assumptions, not confirmed facts, because they
 * cannot be resolved without the device existing to trace against live:
 *
 *   - which physical SPI (27 vs 28) is range 0 vs range 1. The DTB lists
 *     "isp0-0" (index 0) before "isp0-1" (index 1) and
 *     __vin_probe_irq_register requests them in that order, so index 0
 *     -> SPI 27 -> range 0 is the natural reading, but nothing read so
 *     far pins it down harder than that. Cheap to flip (swap the two
 *     pic[] indices below) if a live test shows it backwards.
 *   - which exact bit within a channel's mask means "frame done" versus
 *     some other event (AF done, start-of-frame, ...) sharing the same
 *     block -- not resolved. This model does not try to guess a bit
 *     position: every tick it simply raises whatever bit(s) the guest
 *     driver itself has currently unmasked for that channel, on the
 *     assumption that whichever bit the real wait path cares about is
 *     the one the driver bothered to unmask. A real driver watching
 *     several event types on one channel at once would see all of them
 *     fire together where real hardware might stagger them -- acceptable
 *     for unblocking the wait, not a faithful event model.
 *
 * Channel counts (16 per range) are a generous over-allocation: the real
 * ISR loops "10 or 16" channels depending on pipe, per the live trace,
 * but which pipe gets which count wasn't pinned down either -- extra
 * unused channels simply never get touched by the real guest driver and
 * cost nothing.
 *
 * Timing: isp_ife_wdma_wrapper_trigger was observed live to fire exactly
 * once per capture-start across one clean ~20-30s test window while the
 * frame-wait ioctl itself was polled roughly once a second -- consistent
 * with "arm once, real hardware then raises one IRQ per completed frame
 * indefinitely", but that's a single unconfirmed data point, not
 * independently reverified (a recheck attempt hit an unrelated confound,
 * a watchdog auto-reboot mid-test). This model does not try to detect
 * that specific trigger write at all: it just raises the enabled bit(s)
 * on every tick of a free-running periodic timer, `fps` times a second
 * (default 30, matching the one frame-rate value seen in the real app's
 * own console log: "fDstFrameRate:30.000000"). A channel the guest never
 * unmasks never fires, so there is no cost to running continuously from
 * reset rather than only after a detected trigger.
 * On AX620E ("itp-jobs") the tick raises range 0 only: the ITP's range-1
 * events follow the jobs the driver pushes, see axera_vin_itp_next().
 *
 * CDMA at +0x1000 (AX620E). ax_base.ko's g_cdma_hw_info[0] is
 * 0x02401000 (consumers 0..3; [1] = 0x04400000 serves VPP/DPU):
 *
 *   +0x00 CMD_INFO0  [3:0] flags, [6:4] field, [31:7] request index
 *   +0x04 CMD_INFO1  command list physical address >> 3
 *   +0x08 CMD_CTRL0  bit0 push INFO0/INFO1, bit1 CPU-trigger request
 *                    INFO0[31:7] (ax_base_cdma_trigger: ctrl0=1, ctrl0=2)
 *   +0x18 [15:1] queue occupancy ("cmd queue is full")
 *   +0x2c bit0 CPU request busy (polled 200 x 5us)
 *
 * The list format is the one axera_vin_scan_cdma() walks (descriptor
 * w0 bit31 = last, [30:24] request index, bit4 = LUT leaf; regmap leaf =
 * count, then {value, register} pairs from +0x80). Besides the WDMA
 * set-up the lists carry interrupt arming: vin_irq_executor_add only
 * writes MASK over MMIO for executors not flagged CDMA-managed, and
 * __isp_yuv_wdma_setirq / __isp_itp_wdma_setirq emit CLEAR/MASK through
 * ax_isp_int_{clear,mask}_set_cdma instead. Live AX620E lists: CLEAR
 * 0xffffffff for range-1 ch0..14 every frame, MASK r1/ch6 = 0x2000
 * (__vin_itp_af_done_irq_exec_node) and r1/ch9 = 0x1
 * (__vin_yuv_wdma_done_irq_exec_node) in the last partition, 0 in the
 * first. Applying those writes arms exactly what the executors wait on
 * (r0/ch1 bit0 sensor frame start and r0/ch4 bit21 IFE WDMA done are
 * MMIO-armed). Range-0 ch2 has no executor; r1/ch12 bit25 (EIS post
 * frame end) has one but is never armed while EIS is off, so on
 * hardware it never fires either.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/axera-fbuf.h"
#include "axera-scene.h"
#include "axera-isp.h"
#include "hw/core/cpu.h"

#define TYPE_AXERA_VIN "axera-vin"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraVinState, AXERA_VIN)

#define AXERA_VIN_SIZE      0x100000
#define AXERA_VIN_RANGES    2
#define AXERA_VIN_CHANNELS  16

#define RANGE0_BASE   0x10
#define RANGE1A_BASE  0x80010
#define RANGE1A_CHANS 8
#define RANGE1B_BASE  0xc0010

/* AX620E CDMA block, see the header comment */
#define CDMA_INFO0  0x1000
#define CDMA_INFO1  0x1004

/*
 * The picture the ISP writes (see "Sensor picture" below), per SoC:
 * where its YUV write-DMA (WDMA) is programmed.
 */
enum { ISP_LAYOUT_NONE = 0, ISP_LAYOUT_AX620E = 1, ISP_LAYOUT_AX620U = 2 };
#define VIN_OUTS        2       /* ITP output channels painted */
#define VIN_PAINTED     8       /* buffers already holding the scene */
#define CDMA_TOP_MAX    256     /* command lists per push, at most */
#define ITP_YUV_Y       0x024d0118  /* AX620E ITP YUV WDMA Y address */
#define ITP_JOBS_MAX    16      /* ITP jobs queued in the CDMA, at most */
#define CDMA_SUB_MAX    8192    /* register writes per command list */

/* 3A statistics: see axera_vin_ae_stats() */
#define AE_PTN_MAX      8       /* ITP partitions per frame, at most */
#define AE_GRID_MAX     64      /* grid rows / columns, at most */
#define AE_SCENE_W      192     /* the scene sampled for AX620E's statistics */
#define AE_SCENE_H      108

typedef struct AxeraAePtn {
    hwaddr grid, hist;          /* hist 0: not programmed */
    uint32_t rows, cols;
    uint32_t rw, rh;            /* a grid region's size, pixels */
} AxeraAePtn;

typedef struct AxeraVinOut {
    AxeraNv12Buf buf;
    bool enabled;
    bool y_set, uv_set;     /* in the list being scanned */
} AxeraVinOut;

struct AxeraVinState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[AXERA_VIN_RANGES];
    uint32_t cdma_info0;
    uint32_t cdma_info1;
    uint32_t mask[AXERA_VIN_RANGES][AXERA_VIN_CHANNELS];
    uint32_t raw[AXERA_VIN_RANGES][AXERA_VIN_CHANNELS];
    QEMUTimer *timer;
    uint32_t fps;
    /* itp-jobs: see axera_vin_itp_next() */
    bool itp_jobs;
    uint32_t itp_us;        /* how long the ITP works a frame */
    hwaddr itp_q[ITP_JOBS_MAX];
    unsigned itp_qh, itp_qn;
    bool itp_busy;          /* a job is in the ITP */
    bool itp_unacked;       /* its done events not cleared yet */
    QEMUTimer *itp_timer;
    /* the driver taking the done events, see axera_vin_itp_settled() */
    uint32_t settle_us;
    bool settling;
    int64_t settle_ns;      /* when the done events were cleared */
    int64_t *settle_cpu_ns; /* each vCPU's run time then */
    QEMUTimer *settle_timer;
    bool frame_owed;        /* a sensor frame waits for the ITP */
    /* gate-on-sensor: no frame events until the image sensor wired to
     * the "sensor-streaming" input says it is streaming. */
    bool gate_on_sensor;
    bool sensor_streaming;
    /* Apply interrupt-register writes found in CDMA lists (AX620E) */
    bool cdma_irq_regs;
    /* Event bits never raised by the tick, at one register block
     * (quiet-off = its MASK offset): error/overflow events the driver
     * unmasks to count, which healthy hardware does not report. */
    uint32_t quiet_off;
    uint32_t quiet_bits;
    /* Event bits latched in RAW every tick even though masked: status
     * the driver polls rather than takes as an interrupt (poll-off =
     * that block's MASK offset). */
    uint32_t poll_off;
    uint32_t poll_bits;

    /* Sensor picture: see axera_vin_scan_cdma() */
    uint32_t isp_layout;
    uint32_t cdma_list_off;     /* the ITP CDMA list-address register */
    uint64_t cdma_list_base;    /* that register holds (list - base) >> 3 */
    uint32_t cdma_list;
    uint32_t cdma_list_off2;    /* a second CDMA engine's (AX620U IFE) */
    uint32_t cdma_list2;
    AxeraVinOut out[VIN_OUTS];
    AxeraPicture scene[VIN_OUTS];
    uint32_t frame;
    struct {
        hwaddr y;
        uint32_t w, h;
    } painted[VIN_PAINTED];
    unsigned painted_next;

    /* 3A statistics */
    DeviceState *sensor;        /* its "exposure" scales the scene */
    uint64_t ae_ref;            /* the exposure the scene is painted at */
    uint32_t ae_shift;          /* 8-bit luma to the statistics' scale */
    uint32_t ae_grid_cfg, ae_region;
    AxeraAePtn ae_ptn[AE_PTN_MAX];
    unsigned ae_nptn;
    uint32_t ae_black;          /* AX620U raw: the black level */
    AxeraNv12Buf ae_raw;        /* AX620U raw: the frame (y only) */
    bool ae_raw_set;            /* in the list being scanned */
    AxeraPicture ae_scene;      /* the scene, at the size last asked */

    /* ISP registers as the command lists left them, see axera_vin_isp_reg() */
    uint32_t *isp_reg;
    unsigned isp_dump_n;
    bool scan_ife;              /* scanning the AX620U IFE CDMA's list */
    /* the colour pipeline, see axera_vin_isp_params() */
    bool isp_model;
    uint32_t isp_ref_k;         /* AE's settled exposure / ae-ref, x 1000 */
    AxeraIsp isp;
    AxeraIspParams isp_cand;    /* the reference to be, and for how long */
    uint32_t isp_stable;
    AxeraPicture isp_pic[VIN_OUTS];
    uint32_t isp_pic_gen[VIN_OUTS];
    int64_t isp_ns;             /* time spent in the pipeline (debug) */
};

enum { FIELD_MASK = 0, FIELD_CLEAR = 1, FIELD_RAW = 2, FIELD_STATUS = 3 };

/* Resolve an MMIO offset to (range, channel, field), or false if it
 * falls outside the three known per-channel block windows. */
static bool axera_vin_decode(hwaddr off, int *range, int *ch, int *field)
{
    uint32_t blk_off;

    if (off >= RANGE0_BASE &&
        off < RANGE0_BASE + AXERA_VIN_CHANNELS * 16) {
        *range = 0;
        blk_off = off - RANGE0_BASE;
    } else if (off >= RANGE1A_BASE &&
               off < RANGE1A_BASE + RANGE1A_CHANS * 16) {
        *range = 1;
        blk_off = off - RANGE1A_BASE;
    } else if (off >= RANGE1B_BASE &&
               off < RANGE1B_BASE + (AXERA_VIN_CHANNELS - RANGE1A_CHANS) * 16) {
        *range = 1;
        blk_off = off - RANGE1B_BASE + RANGE1A_CHANS * 16;
    } else {
        return false;
    }
    *ch = blk_off / 16;
    *field = (blk_off % 16) / 4;
    return *field <= FIELD_STATUS;
}

static void axera_vin_update_irq(AxeraVinState *s, int range)
{
    int ch;
    bool level = false;

    for (ch = 0; ch < AXERA_VIN_CHANNELS; ch++) {
        if (s->mask[range][ch] & s->raw[range][ch]) {
            level = true;
            break;
        }
    }
    qemu_set_irq(s->irq[range], level);
}

/*
 * Sensor picture. There is no image sensor and no model of the closed
 * ISP silicon, so the ISP's output is made up here: the OpenIPC test
 * scene (axera-scene.h), written into the very buffers the ISP driver
 * points its YUV write-DMA at, so that everything downstream -- IVPS
 * scaling, OSD, the encoders -- works on real pictures.
 *
 * The driver never writes those addresses to registers directly. It
 * builds command lists in memory and hands them to the ISP's CDMA
 * engine: a write of the top list's address (absolute >> 3 on AX620E,
 * (address - 0x40000000) >> 3 on AX620U) to the list register, then a
 * "push" (1) to the trigger register next to it. A top list is a run
 * of {descriptor, sub-list address} words, bit 31 of the descriptor
 * marking the last; a sub-list is a count word, then from +0x80 that
 * many (value, register) pairs -- the register a physical address on
 * AX620E, a word index into the ISP block on AX620U. This walks every
 * list pushed and picks out the WDMA settings:
 *
 *   AX620E  0x024d0118 Y >> 3, 0x024d00cc UV >> 3, 0x024d0028 size
 *           (h << 16 | w), 0x024d0144 / 0x024d00f8 Y / UV stride >> 3.
 *           The buffer is compressed (half the size of NV12), so the
 *           picture is kept in axera-fbuf for GDC to find.
 *   AX620U  four 12-word WDMA blocks from word 0x7007: Y and UV of ITP
 *           output 0, then of output 1; block +0 bit 0 enable, +3
 *           stride >> 3, +8 (address - 0x40000000) >> 3, +10 size.
 *
 * and paints the next frame of the scene into each output programmed.
 */
static void axera_vin_wdma_set(AxeraVinState *s, uint32_t key, uint32_t val)
{
    if (s->isp_layout == ISP_LAYOUT_AX620E) {
        AxeraNv12Buf *b = &s->out[0].buf;

        switch (key) {
        case 0x024d0118:
            b->y = (hwaddr)val << 3;
            b->compressed = true;
            s->out[0].enabled = true;
            break;
        case 0x024d00cc:
            b->uv = (hwaddr)val << 3;
            break;
        case 0x024d0028:
            b->w = val & 0xffff;
            b->h = val >> 16;
            break;
        case 0x024d0144:
            b->y_stride = val << 3;
            break;
        case 0x024d00f8:
            b->uv_stride = val << 3;
            break;
        }
    } else if (s->isp_layout == ISP_LAYOUT_AX620U &&
               key >= 0x7007 && key < 0x7007 + 4 * 12) {
        int blk = (key - 0x7007) / 12, o = (key - 0x7007) % 12;
        AxeraVinOut *out = &s->out[blk / 2];
        bool uv = blk & 1;
        hwaddr a = s->cdma_list_base + ((hwaddr)val << 3);

        switch (o) {
        case 0:
            if (!uv) {
                out->enabled = val & 1;
            }
            break;
        case 3:
            if (uv) {
                out->buf.uv_stride = val << 3;
            } else {
                out->buf.y_stride = val << 3;
            }
            break;
        case 8: {
            /*
             * The ITP works a frame in vertical partitions, programming
             * each one's start (the frame's address plus its x offset) in
             * turn within one list: the frame starts at the lowest.
             */
            hwaddr *cur = uv ? &out->buf.uv : &out->buf.y;
            bool *set = uv ? &out->uv_set : &out->y_set;

            if (!*set || a < *cur) {
                *cur = a;
            }
            *set = true;
            break;
        }
        case 10:
            if (!uv) {
                out->buf.w = val & 0xffff;
                out->buf.h = val >> 16;
            }
            break;
        }
    }
}

/* Rows 0, h/4, h/2 (luma) and the first chroma row still hold @pic */
static bool axera_vin_still_painted(const AxeraVinOut *out,
                                    const AxeraPicture *pic)
{
    const uint32_t rows[] = { 0, pic->h / 4, pic->h / 2 };
    const AxeraNv12Buf *b = &out->buf;
    g_autofree uint8_t *line = g_malloc(pic->w);
    size_t i;

    for (i = 0; i < ARRAY_SIZE(rows); i++) {
        if (dma_memory_read(&address_space_memory,
                            b->y + (hwaddr)rows[i] * b->y_stride, line,
                            pic->w, MEMTXATTRS_UNSPECIFIED) ||
            memcmp(line, pic->y + (size_t)rows[i] * pic->w, pic->w)) {
            return false;
        }
    }
    return !dma_memory_read(&address_space_memory, b->uv, line, pic->w,
                            MEMTXATTRS_UNSPECIFIED) &&
           !memcmp(line, pic->uv, pic->w);
}

static void axera_vin_paint(AxeraVinState *s, int n)
{
    AxeraVinOut *out = &s->out[n];
    AxeraPicture *pic = &s->scene[n];
    uint32_t w = out->buf.w, h = out->buf.h, r0, r1;
    bool fresh;
    int i;

    /* Y and UV at one address: the driver's placeholder before it has
     * buffers (AX620U: 0x41000000, in use by the kernel) */
    if (!out->enabled || !out->buf.y || !out->buf.uv ||
        out->buf.y == out->buf.uv || w < 16 || h < 16 ||
        w > 8192 || h > 8192) {
        return;
    }
    if (getenv("AXERA_PIX_DEBUG") && s->frame < 400 && !(s->frame % 50)) {
        fprintf(stderr, "axera-vin: frame %u out%d %ux%u y 0x%" HWADDR_PRIx
                "/%u uv 0x%" HWADDR_PRIx "/%u%s\n", s->frame, n, w, h,
                out->buf.y, out->buf.y_stride, out->buf.uv,
                out->buf.uv_stride, out->buf.compressed ? " fbc" : "");
    }
    fresh = pic->w != ROUND_UP(w, 2) || pic->h != ROUND_UP(h, 2);
    axera_picture_alloc(pic, w, h);
    axera_scene_dot_rows(pic->h, &r0, &r1);
    if (fresh) {
        axera_scene_nv12(pic->y, pic->w, pic->uv, pic->w, pic->w, pic->h,
                         s->frame);
    } else {
        axera_scene_nv12_rows(pic->y, pic->w, pic->uv, pic->w, pic->w,
                              pic->h, s->frame, r0, r1);
    }
    if (s->isp_model && s->isp.ref_set) {
        AxeraPicture *ip = &s->isp_pic[n];
        bool all = fresh || ip->w != pic->w || ip->h != pic->h ||
                   s->isp_pic_gen[n] != s->isp.gen;

        int64_t t0 = get_clock();

        axera_picture_alloc(ip, w, h);
        axera_isp_rows(&s->isp, pic, ip, all ? 0 : r0, all ? pic->h : r1);
        s->isp_ns += get_clock() - t0;
        if (getenv("AXERA_ISP_DEBUG") && !(s->frame % 300)) {
            fprintf(stderr, "axera-vin: isp %.2f ms/frame (out%d %ux%u)\n",
                    s->isp_ns / 300e6, n, w, h);
            s->isp_ns = 0;
        }
        s->isp_pic_gen[n] = s->isp.gen;
        pic = ip;
    }
    if (out->buf.compressed) {
        axera_fbuf_put(out->buf.y, pic);
        return;
    }
    /*
     * A buffer that already holds the scene only needs the dot's rows --
     * unless the driver has since lent it to something else, which
     * would leave stale rows the encoder takes in once and then keeps
     * (it skips unchanged blocks): spot-check a few rows first.
     */
    for (i = 0; i < VIN_PAINTED; i++) {
        if (s->painted[i].y == out->buf.y && s->painted[i].w == w &&
            s->painted[i].h == h && axera_vin_still_painted(out, pic)) {
            axera_nv12_store_rows(&out->buf, pic, r0, r1);
            return;
        }
    }
    axera_fbuf_drop(out->buf.y);
    axera_nv12_store(&out->buf, pic);
    i = s->painted_next++ % VIN_PAINTED;
    s->painted[i].y = out->buf.y;
    s->painted[i].w = w;
    s->painted[i].h = h;
}

/*
 * 3A statistics. The AE meters what the ISP measured, not the picture
 * painted above: with nothing measured it sees a black scene, drives
 * exposure and gain to their maximum, and the camera App takes the
 * resulting exposure value (AX_ISP_IQ_GetAeStatus nExpVal, shutter x
 * gains) for night -- slow shutter, a few frames a second, an encoder
 * stop. So what the AE reads is written here, at the addresses the
 * command lists program, from the scene (axera-scene.h, sampled small)
 * as exposed: its luma times the exposure the AE last gave the sensor
 * (the sensor's "exposure" property, rows x gain) over ae-ref, the
 * exposure at which the scene reads as painted. The loop then settles
 * where that meets the AE's target, a short daylight shutter at unity
 * gain for the default ae-ref.
 *
 * AX620E: the ISP's own statistics (ax_proton.ko
 * isp_itp_ae_stat_wrapper_update -> itp_ae_mod_update; userspace
 * libax_proton ax_isp_fw_stat_ae_merging_process hands them to
 * libax_ae). The ITP AE block, 0x02497000, is programmed once per ITP
 * partition (vertical strip) in the lists:
 *   +0x18  grid0 (rows - 1) << 16 | (columns - 1) in this partition
 *   +0x1c  grid0 region height << 16 | width
 *   +0x8c  grid0 buffer >> 3: rows x columns cells, row-major
 *   +0x90  grid1 (flicker) buffer >> 3 -- not written
 *   +0x94  histogram buffer >> 3: 256 linear, then 64 log bins
 * The merge lays the partitions' cells side by side (adding the
 * boundary columns two partitions share) and adds their histograms.
 * A cell is AX_AE_GRID_STATS_T, u32 sum[4] then u16 count[4]; a bin
 * AX_AE_HIST_BIN_T, u32 count[4]; values U8.4 (ae-shift 4), the scale
 * of the AE's set point (nSetPoint 32768 = 32.0 in U8.10). Every cell
 * gets the exposed scene's mean, and the first partition's histogram
 * its whole histogram (the others none): the AE weighs levels, not
 * where in the picture they are.
 *
 * AX620U: the App's AE runs ax_3a_stat_process_by_cpu -- it does not
 * read the ISP's IFE statistics at all but measures a small raw frame
 * on the CPU (grid, histogram: the App's own code), the IFE raw
 * scaler's output, its write-DMA block at list word 0x6869:
 *   +4     (buffer - 0x40000000) >> 3, a new one each frame
 *   +8     width << 16 | height (168 x 96 for 2688 x 1520)
 *   +9     stride >> 3
 * 14-bit Bayer, packed nine pixels to each 128-bit word from bit 0 up
 * (its unpacker, format 0xe); the App takes (pixel - black level) / 64,
 * black level 1024 (ae-black), as the 8-bit luma it meters against its
 * set point (12288 = 12.0). The scene is written grey (R = G = B).
 */
static void axera_vin_ae_set(AxeraVinState *s, uint32_t key, uint32_t val)
{
    AxeraAePtn *p;

    if (s->isp_layout == ISP_LAYOUT_AX620E && (key >> 12) == 0x02497) {
        switch (key & 0xfff) {
        case 0x18:
            s->ae_grid_cfg = val;
            break;
        case 0x1c:
            s->ae_region = val;
            break;
        case 0x8c:
            if (s->ae_nptn == AE_PTN_MAX) {
                return;
            }
            p = &s->ae_ptn[s->ae_nptn++];
            p->grid = (hwaddr)val << 3;
            p->hist = 0;
            p->rows = MIN(((s->ae_grid_cfg >> 16) & 0x3ff) + 1, AE_GRID_MAX);
            p->cols = MIN((s->ae_grid_cfg & 0x7f) + 1, AE_GRID_MAX);
            p->rh = s->ae_region >> 16;
            p->rw = s->ae_region & 0xffff;
            break;
        case 0x94:
            if (s->ae_nptn) {
                s->ae_ptn[s->ae_nptn - 1].hist = (hwaddr)val << 3;
            }
            break;
        }
    } else if (s->isp_layout == ISP_LAYOUT_AX620U &&
               key >= 0x6869 && key < 0x6869 + 10) {
        switch (key - 0x6869) {
        case 4:
            s->ae_raw.y = s->cdma_list_base + ((hwaddr)val << 3);
            s->ae_raw_set = true;
            break;
        case 8:
            s->ae_raw.w = val >> 16;
            s->ae_raw.h = val & 0xffff;
            break;
        case 9:
            s->ae_raw.y_stride = val << 3;
            break;
        }
    }
}

/* The exposure the AE gave the sensor over ae-ref: the scene's gain */
static double axera_vin_ae_gain(AxeraVinState *s, uint64_t *e)
{
    *e = 0;
    if (s->sensor) {
        *e = object_property_get_uint(OBJECT(s->sensor), "exposure", NULL);
    }
    return *e && s->ae_ref ? (double)*e / s->ae_ref : 1.0;
}

/* The scene's luma at @w x @h, kept in s->ae_scene */
static const uint8_t *axera_vin_ae_scene(AxeraVinState *s, uint32_t w,
                                         uint32_t h)
{
    if (s->ae_scene.w != ROUND_UP(w, 2) || s->ae_scene.h != ROUND_UP(h, 2)) {
        axera_picture_alloc(&s->ae_scene, w, h);
        axera_scene_nv12(s->ae_scene.y, s->ae_scene.w, s->ae_scene.uv,
                         s->ae_scene.w, s->ae_scene.w, s->ae_scene.h, 0);
    }
    return s->ae_scene.y;
}

static void axera_vin_ae_stats(AxeraVinState *s)
{
    const uint8_t *y = axera_vin_ae_scene(s, AE_SCENE_W, AE_SCENE_H);
    uint32_t bins[320][4] = { 0 };
    uint32_t hist[256] = { 0 };
    uint64_t e, lsum = 0, total = AE_SCENE_W * AE_SCENE_H;
    double k = axera_vin_ae_gain(s, &e);
    uint8_t cell[24];
    unsigned i, r, c, b, bits = 8 + s->ae_shift;
    double mean;

    for (i = 0; i < total; i++) {
        hist[y[i]]++;
    }
    /* the scene as exposed: each luma scaled by k, clipped at white */
    for (b = 0; b < 256; b++) {
        uint32_t l = MIN(b * k + 0.5, 255.0);
        uint32_t lb = MIN(log2(((uint64_t)l << s->ae_shift) + 1) * 64 / bits,
                          63);

        lsum += (uint64_t)l * hist[b];
        for (c = 0; c < 4; c++) {
            bins[l][c] += hist[b];
            bins[256 + lb][c] += hist[b];
        }
    }
    mean = (double)lsum / total;
    if (getenv("AXERA_AE_DEBUG") && !(s->frame % 30)) {
        fprintf(stderr, "axera-vin: AE exposure %" PRIu64 " k %.3f mean "
                "%.1f, %u partitions, grid %ux%u\n", e, k, mean, s->ae_nptn,
                s->ae_ptn[0].rows, s->ae_ptn[0].cols);
    }
    for (b = 0; b < 320; b++) {
        for (c = 0; c < 4; c++) {
            bins[b][c] = cpu_to_le32(bins[b][c]);
        }
    }

    for (i = 0; i < s->ae_nptn; i++) {
        AxeraAePtn *p = &s->ae_ptn[i];
        uint32_t val = mean * (1u << s->ae_shift);
        uint32_t num = MAX(MIN(p->rw * p->rh / 4, 0xffff), 1);

        /* a region's sum is a u32 */
        num = MAX(MIN(num, 0xffffffffu / MAX(val, 1)), 1);
        for (c = 0; c < 4; c++) {
            stl_le_p(cell + 4 * c, num * val);
            stw_le_p(cell + 16 + 2 * c, num);
        }
        for (r = 0; r < p->rows; r++) {
            for (c = 0; c < p->cols; c++) {
                dma_memory_write(&address_space_memory,
                                 p->grid + 24 * (r * p->cols + c), cell,
                                 sizeof(cell), MEMTXATTRS_UNSPECIFIED);
            }
        }
        if (!p->hist) {
            continue;
        }
        if (i == 0) {
            dma_memory_write(&address_space_memory, p->hist, bins,
                             sizeof(bins), MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_set(&address_space_memory, p->hist, 0, sizeof(bins),
                           MEMTXATTRS_UNSPECIFIED);
        }
    }
}

#define AE_RAW_MAX_STRIDE   (64 * KiB)

/* AX620U: the IFE raw scaler's frame, see "3A statistics" */
static void axera_vin_ae_raw(AxeraVinState *s)
{
    AxeraNv12Buf *f = &s->ae_raw;
    uint32_t w = f->w, h = f->h, groups = DIV_ROUND_UP(w, 9), r, g, i;
    g_autofree uint8_t *row = NULL;
    const uint8_t *y;
    uint16_t lut[256];
    uint64_t e;
    double k;

    if (!w || !h || w > 1024 || h > 1024 || f->y_stride < groups * 16) {
        return;
    }
    /* a 1024-pixel row packs into 1824 bytes */
    if (f->y_stride > AE_RAW_MAX_STRIDE) {
        qemu_log_mask(LOG_GUEST_ERROR, "axera-vin: AE raw stride %u too "
                      "large, frame not written\n", f->y_stride);
        return;
    }
    k = axera_vin_ae_gain(s, &e);
    for (i = 0; i < 256; i++) {
        lut[i] = MIN(s->ae_black + i * k * 64 + 0.5, 16383.0);
    }
    if (getenv("AXERA_AE_DEBUG") && !(s->frame % 30)) {
        fprintf(stderr, "axera-vin: AE exposure %" PRIu64 " k %.3f, raw "
                "%ux%u at 0x%" HWADDR_PRIx "\n", e, k, w, h, f->y);
    }
    y = axera_vin_ae_scene(s, w, h);
    row = g_malloc0(f->y_stride);
    for (r = 0; r < h; r++) {
        const uint8_t *yl = y + (size_t)r * s->ae_scene.w;

        for (g = 0; g < groups; g++) {
            uint64_t lo = 0, hi = 0;

            for (i = 0; i < 9 && g * 9 + i < w; i++) {
                uint64_t v = lut[yl[g * 9 + i]];
                unsigned bit = 14 * i;

                if (bit < 64) {
                    lo |= v << bit;
                    if (bit + 14 > 64) {
                        hi |= v >> (64 - bit);
                    }
                } else {
                    hi |= v << (bit - 64);
                }
            }
            stq_le_p(row + 16 * g, lo);
            stq_le_p(row + 16 * g + 8, hi);
        }
        dma_memory_write(&address_space_memory,
                         f->y + (hwaddr)r * f->y_stride, row, f->y_stride,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void axera_vin_reg_write(AxeraVinState *s, hwaddr off, uint32_t val,
                                bool from_cdma);

/* Does the command list at top write register key? */
static bool axera_vin_cdma_writes(AxeraVinState *s, hwaddr top, uint32_t key)
{
    g_autofree uint32_t *pairs = NULL;
    int i, n;

    for (i = 0; i < CDMA_TOP_MAX; i++) {
        uint32_t ent[2], count;
        hwaddr sub;

        if (dma_memory_read(&address_space_memory, top + 8 * i, ent,
                            sizeof(ent), MEMTXATTRS_UNSPECIFIED)) {
            break;
        }
        sub = s->cdma_list_base + ((hwaddr)le32_to_cpu(ent[1]) << 3);
        if (!dma_memory_read(&address_space_memory, sub, &count,
                             sizeof(count), MEMTXATTRS_UNSPECIFIED)) {
            count = le32_to_cpu(count);
            if (count && count <= CDMA_SUB_MAX) {
                pairs = g_renew(uint32_t, pairs, 2 * count);
                if (!dma_memory_read(&address_space_memory, sub + 0x80,
                                     pairs, 8 * count,
                                     MEMTXATTRS_UNSPECIFIED)) {
                    for (n = 0; n < count; n++) {
                        if (le32_to_cpu(pairs[2 * n + 1]) == key) {
                            return true;
                        }
                    }
                }
            }
        }
        if (le32_to_cpu(ent[0]) & 0x80000000u) {
            break;
        }
    }
    return false;
}

static void axera_vin_itp_next(AxeraVinState *s);
static void axera_vin_itp_settle(AxeraVinState *s);

#define ISP_REGS        0x40000     /* words: AX620E 0x02400000 + 1 MiB */

/*
 * A register's place in isp_reg[]: its offset in the ISP block / 4.
 * AX620E lists carry physical addresses; AX620U ones word indices, into
 * the ITP (block + 0x80000) on the ITP's CDMA, into the IFE (block + 0)
 * on the IFE's (scan_ife).
 */
static int axera_vin_isp_idx(AxeraVinState *s, uint32_t key)
{
    if (s->isp_layout == ISP_LAYOUT_AX620E) {
        key = (key - 0x02400000u) >> 2;
    } else if (key < 0x20000 && !s->scan_ife) {
        key += 0x20000;
    }
    return key < ISP_REGS ? key : -1;
}

/* A register write the CDMA engine carries out */
static void axera_vin_isp_reg(AxeraVinState *s, uint32_t key, uint32_t val)
{
    int i = axera_vin_isp_idx(s, key);
    static int64_t trace_lo = -1, trace_hi;

    if (trace_lo == -1) {
        const char *t = getenv("AXERA_ISP_TRACE");

        trace_lo = t ? strtoll(t, NULL, 16) : 0;
        trace_hi = t && strchr(t, '-') ? strtoll(strchr(t, '-') + 1, NULL, 16)
                                       : trace_lo;
    }
    if (trace_lo && key >= trace_lo && key <= trace_hi) {
        fprintf(stderr, "axera-vin: isp reg f%u %08x = %08x\n", s->frame, key,
                val);
    }
    if (i >= 0) {
        s->isp_reg[i] = val;
    }
}

/* Debug: AXERA_ISP_DUMP=<prefix>; touch <prefix>.req for a register dump */
static void axera_vin_isp_dump(AxeraVinState *s)
{
    const char *pfx = getenv("AXERA_ISP_DUMP");
    g_autofree char *req = NULL, *out = NULL;
    FILE *f;
    int i;

    if (!pfx) {
        return;
    }
    req = g_strdup_printf("%s.req", pfx);
    if (access(req, F_OK)) {
        return;
    }
    unlink(req);
    out = g_strdup_printf("%s.%u", pfx, s->isp_dump_n++);
    f = fopen(out, "w");
    if (!f) {
        return;
    }
    for (i = 0; i < ISP_REGS; i++) {
        if (s->isp_reg[i]) {
            fprintf(f, "%08x %08x\n", s->isp_layout == ISP_LAYOUT_AX620E ?
                    0x02400000u + 4 * i : i, s->isp_reg[i]);
        }
    }
    fclose(f);
}

/*
 * A LUT leaf (descriptor bit 4): a 64-bit header, [15:0] words, [63:16]
 * the first register's address >> 2, then from +0x80 that many words for
 * consecutive registers (ax_proton.ko isp_cdma_io_write_lut).
 */
static void axera_vin_scan_lut(AxeraVinState *s, hwaddr sub)
{
    uint64_t hdr;
    uint32_t n, i, key;
    g_autofree uint32_t *w = NULL;

    if (dma_memory_read(&address_space_memory, sub, &hdr, sizeof(hdr),
                        MEMTXATTRS_UNSPECIFIED)) {
        return;
    }
    hdr = le64_to_cpu(hdr);
    n = hdr & 0xffff;
    key = s->isp_layout == ISP_LAYOUT_AX620E ? (uint32_t)(hdr >> 16) << 2
                                             : (uint32_t)(hdr >> 16);
    if (!n || n > CDMA_SUB_MAX) {
        return;
    }
    w = g_new(uint32_t, n);
    if (dma_memory_read(&address_space_memory, sub + 0x80, w, 4 * n,
                        MEMTXATTRS_UNSPECIFIED)) {
        return;
    }
    for (i = 0; i < n; i++) {
        axera_vin_isp_reg(s, key + (s->isp_layout == ISP_LAYOUT_AX620E ?
                                    4 * i : i), le32_to_cpu(w[i]));
    }
}

#define ISP_REF_FRAMES  60      /* frames the reference settings hold */

/* A register as programmed: AX620E by address, AX620U by ITP word */
static uint32_t axera_vin_isp_rd(AxeraVinState *s, uint32_t key)
{
    if (s->isp_layout == ISP_LAYOUT_AX620U) {
        key += 0x20000;
        return key < ISP_REGS ? s->isp_reg[key] : 0;
    }
    key = (key - 0x02400000u) >> 2;
    return key < ISP_REGS ? s->isp_reg[key] : 0;
}

/*
 * The colour pipeline's settings (axera-isp.h), from the registers the
 * lists programmed. AX620E, libax_proton's register maps (its DWARF:
 * ISP_WBC_REG_T, ISP_ITP_GAMMA_T, ISP_LCE_REG_T):
 *
 *   0x0248d000 ITP WBC  +0x18 d_gain [17:0]
 *                       +0x1c r_gain [11:0], gr_gain [23:12]
 *                       +0x20 b_gain [11:0], gb_gain [23:12]
 *   0x024c8000 RGB GAM  +0x08 bit 0 input spacing exponential
 *                       +0x18 64 x {low [11:0], high [27:16]}, +0x118
 *                       the 129th: U8.4 outputs
 *   0x024cc000 YUV LCE  +0x7c8 YCLIP_LUT 16 x {[9:0], [25:16]}, +0x808
 *                       the 33rd [9:0] and enable (bit 10): the luma
 *                       curve, 10-bit, input step 32
 *                       +0x91c CCLIP enable (bit 0), +0x920..0x92c
 *                       CCLIP_CMTX[2][2], signed 8-bit: the chroma
 *                       matrix
 *
 * The camera App's image settings (AVEnc.VideoColor, AX_ISP_IQ_
 * SetYcprocParam) land in YCLIP (brightness scales the curve, contrast
 * steepens it about the middle) and CCLIP (saturation scales the
 * matrix, hue rotates it, colour mode "black and white" zeroes it).
 * Its defaults (50 each) are the reference: the curve (i - 2) x 1024 /
 * 30, the matrix 34 x identity.
 *
 * AX620U, ITP list words (live diffs; the WBC base is ax_proton.ko's
 * isp_wbc_m0_regmap, ITP byte 0x8c000):
 *
 *   0x3004 d_gain [17:0], 0x3005 r [11:0] gr [23:12], 0x3006 b [11:0]
 *          gb [23:12]: the white balance, as AX620E's
 *   0x501d in [9:0], out [19:10]; 0x501e gain U4.8: the App's
 *          brightness and contrast, y' = (y - in) x gain + out on 10-bit
 *          luma (default in = out = 512, gain 1)
 *   0x5030 the chroma matrix, int8 m00 [7:0] m01 [15:8] m10 [23:16]
 *          m11 [31:24], unity 32: saturation, hue, black and white
 *
 * Its gamma was not found in the lists: a nominal 1/2.2 curve stands in.
 *
 * White balance and digital gain come from the ISP's own algorithms:
 * their reference is where the AWB settles on both SoCs (r 0x5d6, g
 * 0x400, b 0xb6d on AX620E; 0x175, 0x100, 0x2db on AX620U; digital gain
 * 0x100). The gamma's is what the camera programs once the settings
 * have held for ISP_REF_FRAMES (the first lists carry start-up values);
 * until then the scene goes out as painted.
 */
static void axera_vin_isp_params(AxeraVinState *s)
{
    AxeraIspParams p;
    uint64_t e;
    double k;
    int i;

    memset(&p, 0, sizeof(p));
    if (s->isp_layout == ISP_LAYOUT_AX620E) {
        uint32_t dg = axera_vin_isp_rd(s, 0x0248d018) & 0x3ffff;
        uint32_t g0 = axera_vin_isp_rd(s, 0x0248d01c);
        uint32_t g1 = axera_vin_isp_rd(s, 0x0248d020);
        uint32_t yr = axera_vin_isp_rd(s, 0x024cc808);

        if (dg && (g0 & 0xfff) && (g1 & 0xfff)) {
            p.wb_set = true;
            p.wb[0] = (double)(g0 & 0xfff) * dg;
            p.wb[1] = (double)(((g0 >> 12) & 0xfff) + ((g1 >> 12) & 0xfff)) /
                      2 * dg;
            p.wb[2] = (double)(g1 & 0xfff) * dg;
        }
        for (i = 0; i < 64; i++) {
            uint32_t v = axera_vin_isp_rd(s, 0x024c8018 + 4 * i);

            p.gam[2 * i] = v & 0xfff;
            p.gam[2 * i + 1] = (v >> 16) & 0xfff;
        }
        p.gam[128] = axera_vin_isp_rd(s, 0x024c8118) & 0xfff;
        p.gam_set = p.gam[128] != 0;
        p.gam_exp = axera_vin_isp_rd(s, 0x024c8008) & 1;
        for (i = 0; i < 16; i++) {
            uint32_t v = axera_vin_isp_rd(s, 0x024cc7c8 + 4 * i);

            p.ylut[2 * i] = v & 0x3ff;
            p.ylut[2 * i + 1] = (v >> 16) & 0x3ff;
        }
        p.ylut[32] = yr & 0x3ff;
        p.ylut_set = yr & 0x400;
        p.cm_set = axera_vin_isp_rd(s, 0x024cc91c) & 1;
        for (i = 0; i < 4; i++) {
            p.cm[i / 2][i % 2] =
                (int8_t)axera_vin_isp_rd(s, 0x024cc920 + 4 * i) / 32.0;
        }
    } else if (s->isp_layout == ISP_LAYOUT_AX620U) {
        uint32_t dg = axera_vin_isp_rd(s, 0x3004) & 0x3ffff;
        uint32_t g0 = axera_vin_isp_rd(s, 0x3005);
        uint32_t g1 = axera_vin_isp_rd(s, 0x3006);
        uint32_t yo = axera_vin_isp_rd(s, 0x501d);
        uint32_t yg = axera_vin_isp_rd(s, 0x501e) & 0xfff;
        uint32_t cm = axera_vin_isp_rd(s, 0x5030);

        if (dg && (g0 & 0xfff) && (g1 & 0xfff)) {
            p.wb_set = true;
            p.wb[0] = (double)(g0 & 0xfff) * dg;
            p.wb[1] = (double)(((g0 >> 12) & 0xfff) + ((g1 >> 12) & 0xfff)) /
                      2 * dg;
            p.wb[2] = (double)(g1 & 0xfff) * dg;
        }
        if (yg) {
            /* (y - in) x gain + out, 10-bit: as a 33-point curve */
            p.ylut_set = true;
            for (i = 0; i < AXERA_ISP_YLUT_N; i++) {
                double v = (32.0 * i - (yo & 0x3ff)) * yg / 256 +
                           ((yo >> 10) & 0x3ff);

                p.ylut[i] = MIN(MAX(lround(v), 0), 1023);
            }
        }
        p.cm_set = yg != 0;     /* 0 is black and white */
        for (i = 0; i < 4; i++) {
            p.cm[i / 2][i % 2] = (int8_t)(cm >> (8 * i)) / 32.0;
        }
    } else {
        return;
    }


    if (!s->isp.ref_set) {
        /* the reference: the settings once they have held a while */
        p.k = 0;
        if (!p.wb_set ||
            (s->isp_layout == ISP_LAYOUT_AX620E && !p.gam_set) ||
            memcmp(&p, &s->isp_cand, sizeof(p))) {
            s->isp_cand = p;
            s->isp_stable = 0;
            return;
        }
        if (++s->isp_stable < ISP_REF_FRAMES) {
            return;
        }
        s->isp.ref = p;
        s->isp.ref.k = 1;
        /*
         * White balance: what the AWB settles at on both (r 1.457, b 2.856
         * x g; it starts elsewhere), digital gain 1.
         */
        s->isp.ref.wb[1] = 0x100 * (s->isp_layout == ISP_LAYOUT_AX620E ?
                                    0x400 : 0x100);
        s->isp.ref.wb[0] = s->isp.ref.wb[1] * 1.457;
        s->isp.ref.wb[2] = s->isp.ref.wb[1] * 2.856;
        if (s->isp_layout == ISP_LAYOUT_AX620E) {
            s->isp.ref.ylut_set = s->isp.ref.cm_set = true;
            for (i = 0; i < AXERA_ISP_YLUT_N; i++) {
                s->isp.ref.ylut[i] = MIN(MAX(lround((i - 2) * 1024 / 30.0),
                                             0), 1023);
            }
            memset(s->isp.ref.cm, 0, sizeof(s->isp.ref.cm));
            s->isp.ref.cm[0][0] = s->isp.ref.cm[1][1] = 34 / 32.0;
        } else {
            s->isp.ref.ylut_set = s->isp.ref.cm_set = true;
            for (i = 0; i < AXERA_ISP_YLUT_N; i++) {
                s->isp.ref.ylut[i] = MIN(32 * i, 1023);
            }
            memset(s->isp.ref.cm, 0, sizeof(s->isp.ref.cm));
            s->isp.ref.cm[0][0] = s->isp.ref.cm[1][1] = 1;
        }
        s->isp.ref_set = true;
    }
    /* the exposure, in steps of 1/24 of a stop */
    k = axera_vin_ae_gain(s, &e) * 1000 / MAX(s->isp_ref_k, 1);
    p.k = k > 0 ? exp2(round(log2(k) * 24) / 24) : 0;
    if (axera_isp_update(&s->isp, &p)) {
        /* every buffer needs the whole new picture */
        memset(s->painted, 0, sizeof(s->painted));
        if (getenv("AXERA_ISP_DEBUG")) {
            fprintf(stderr, "axera-vin: isp gen %u k %.3f wb %.0f %.0f %.0f "
                    "gam %d/%u ylut %d/%u..%u cm %d/%.2f %.2f %.2f %.2f\n",
                    s->isp.gen, p.k, p.wb[0], p.wb[1], p.wb[2], p.gam_set,
                    p.gam[64], p.ylut_set, p.ylut[8], p.ylut[24], p.cm_set,
                    p.cm[0][0], p.cm[0][1], p.cm[1][0], p.cm[1][1]);
        }
    }
}

static void axera_vin_scan_cdma(AxeraVinState *s, hwaddr top)
{
    g_autofree uint32_t *pairs = NULL;
    bool seen = false;
    int i, n;

    for (n = 0; n < VIN_OUTS; n++) {
        s->out[n].y_set = s->out[n].uv_set = false;
    }
    s->ae_nptn = 0;
    s->ae_raw_set = false;

    for (i = 0; i < CDMA_TOP_MAX; i++) {
        uint32_t ent[2], count;
        hwaddr sub;

        if (dma_memory_read(&address_space_memory, top + 8 * i, ent,
                            sizeof(ent), MEMTXATTRS_UNSPECIFIED)) {
            break;
        }
        sub = s->cdma_list_base + ((hwaddr)le32_to_cpu(ent[1]) << 3);
        if (le32_to_cpu(ent[0]) & 0x10) {
            axera_vin_scan_lut(s, sub);
        } else if (!dma_memory_read(&address_space_memory, sub, &count,
                             sizeof(count), MEMTXATTRS_UNSPECIFIED)) {
            count = le32_to_cpu(count);
            if (count && count <= CDMA_SUB_MAX) {
                pairs = g_renew(uint32_t, pairs, 2 * count);
                if (!dma_memory_read(&address_space_memory, sub + 0x80,
                                     pairs, 8 * count,
                                     MEMTXATTRS_UNSPECIFIED)) {
                    for (n = 0; n < count; n++) {
                        uint32_t key = le32_to_cpu(pairs[2 * n + 1]);

                        axera_vin_isp_reg(s, key, le32_to_cpu(pairs[2 * n]));

                        /* What the CDMA engine does with every pair;
                         * here only the interrupt registers matter. */
                        if (s->cdma_irq_regs && (key >> 20) == 0x024) {
                            axera_vin_reg_write(s, key & 0xfffff,
                                                le32_to_cpu(pairs[2 * n]),
                                                true);
                        }
                        axera_vin_ae_set(s, key,
                                         le32_to_cpu(pairs[2 * n]));
                        if ((key >= 0x7007 && key < 0x7037) ||
                            (key >> 12) == 0x024d0) {
                            axera_vin_wdma_set(s, key,
                                               le32_to_cpu(pairs[2 * n]));
                            seen = true;
                        }
                    }
                }
            }
        }
        if (le32_to_cpu(ent[0]) & 0x80000000u) {
            break;
        }
    }
    axera_vin_isp_dump(s);
    if (s->ae_nptn) {
        axera_vin_ae_stats(s);
    }
    if (s->ae_raw_set) {
        axera_vin_ae_raw(s);
    }
    if (seen) {
        if (s->isp_model) {
            axera_vin_isp_params(s);
        }
        for (n = 0; n < VIN_OUTS; n++) {
            axera_vin_paint(s, n);
        }
        s->frame++;
    }
}

static uint64_t axera_vin_read(void *opaque, hwaddr off, unsigned size)
{
    AxeraVinState *s = AXERA_VIN(opaque);
    int range, ch, field;

    if (off == CDMA_INFO0) {
        return s->cdma_info0;
    }
    if (off == CDMA_INFO1) {
        return s->cdma_info1;
    }
    if (off >= CDMA_INFO0 + 8 && off < CDMA_INFO0 + 0x100) {
        /* CTRL0 self-clears; queue empty (+0x18), CPU request not busy
         * (+0x2c): a list completes as it is pushed here. */
        return 0;
    }

    if (!axera_vin_decode(off, &range, &ch, &field)) {
        qemu_log_mask(LOG_UNIMP, "axera-vin: read [0x%05" HWADDR_PRIx "]\n",
                      off);
        return 0;
    }
    switch (field) {
    case FIELD_MASK:
        return s->mask[range][ch];
    case FIELD_RAW:
        return s->raw[range][ch];
    case FIELD_STATUS:
        return s->mask[range][ch] & s->raw[range][ch];
    default: /* CLEAR is write-only */
        return 0;
    }
}

static void axera_vin_reg_write(AxeraVinState *s, hwaddr off, uint32_t val,
                                bool from_cdma)
{
    int range, ch, field;

    if (!axera_vin_decode(off, &range, &ch, &field)) {
        /* ISP configuration: not modelled. Lists carry thousands of
         * these per frame, so only CPU writes are logged. */
        if (!from_cdma) {
            qemu_log_mask(LOG_UNIMP, "axera-vin: write [0x%05" HWADDR_PRIx
                          "] = 0x%08" PRIx32 "\n", off, val);
        }
        return;
    }
    switch (field) {
    case FIELD_MASK:
        if (getenv("AXERA_VIN_DEBUG") && s->mask[range][ch] != val) {
            qemu_log("axera-vin: mask r%d ch%d [0x%05" HWADDR_PRIx
                     "] %08x -> %08x%s\n", range, ch, off,
                     s->mask[range][ch], val, from_cdma ? " (cdma)" : "");
        }
        s->mask[range][ch] = val;
        break;
    case FIELD_CLEAR:
        s->raw[range][ch] &= ~val;
        break;
    default: /* RAW and STATUS are read-only */
        return;
    }
    axera_vin_update_irq(s, range);
    if (range == 1 && s->itp_unacked && !from_cdma) {
        int c;

        for (c = 0; c < AXERA_VIN_CHANNELS; c++) {
            if (s->mask[1][c] & s->raw[1][c]) {
                return;
            }
        }
        /* the driver has taken the last job's done events */
        s->itp_unacked = false;
        timer_del(s->itp_timer);
        axera_vin_itp_settle(s);
        axera_vin_itp_next(s);
    }
}

static void axera_vin_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    AxeraVinState *s = AXERA_VIN(opaque);

    if (s->isp_layout && off == s->cdma_list_off) {
        s->cdma_list = val;
    }
    if (s->isp_layout && off == s->cdma_list_off + 4 && (val & 1)) {
        hwaddr top = s->cdma_list_base + ((hwaddr)s->cdma_list << 3);

        if (s->itp_jobs && s->itp_qn < ITP_JOBS_MAX &&
            axera_vin_cdma_writes(s, top, ITP_YUV_Y)) {
            /* an ITP job: queued, see axera_vin_itp_next() */
            s->itp_q[(s->itp_qh + s->itp_qn++) % ITP_JOBS_MAX] = top;
            axera_vin_itp_next(s);
        } else {
            axera_vin_scan_cdma(s, top);
        }
    }
    if (s->cdma_list_off2 && off == s->cdma_list_off2) {
        s->cdma_list2 = val;
    }
    if (s->cdma_list_off2 && off == s->cdma_list_off2 + 4 && (val & 1)) {
        s->scan_ife = true;
        axera_vin_scan_cdma(s, s->cdma_list_base +
                               ((hwaddr)s->cdma_list2 << 3));
        s->scan_ife = false;
    }
    if (s->isp_layout == ISP_LAYOUT_AX620E) {
        axera_vin_isp_reg(s, 0x02400000u + off, val);
    } else if (s->isp_layout && off >= 0x80000) {
        axera_vin_isp_reg(s, (off - 0x80000) >> 2, val);    /* ITP */
    }
    if (off == CDMA_INFO0) {
        s->cdma_info0 = val;
        return;
    }
    if (off == CDMA_INFO1) {
        s->cdma_info1 = val;
        return;
    }
    axera_vin_reg_write(s, off, val, false);
}

/*
 * .valid.max_access_size is 8, not 4: the real closed-source ax_proton.ko
 * backs up this whole bank with a plain memcpy() at probe time
 * (VIN_glb_create -> ax_regio_backup -> __ax_regio_default_create ->
 * memcpy, confirmed live -- the same real access pattern already
 * documented against this same region in hisi-regbank.c, which this
 * device replaces), so 8-byte loads/stores against two adjacent
 * channels' worth of registers are a genuine, harmless access real
 * hardware has no trouble with. .impl stays at 4: the memory core's own
 * generic splitting turns one 8-byte request into two correctly-adjacent
 * 4-byte offset/offset+4 calls into the same read/write above, so no
 * changes were needed there.
 */
static const MemoryRegionOps axera_vin_ops = {
    .read = axera_vin_read,
    .write = axera_vin_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* One sensor frame: latch every enabled event of its ranges */
static void axera_vin_frame(AxeraVinState *s)
{
    int range, ch;

    for (range = 0; range < AXERA_VIN_RANGES; range++) {
        if (range == 1 && s->itp_jobs) {
            continue;   /* raised by the ITP job, axera_vin_itp_done() */
        }
        for (ch = 0; ch < AXERA_VIN_CHANNELS; ch++) {
            uint32_t ev = s->mask[range][ch];
            int qr, qc, qf;

            if (s->quiet_bits &&
                axera_vin_decode(s->quiet_off, &qr, &qc, &qf) &&
                qr == range && qc == ch) {
                ev &= ~s->quiet_bits;
            }
            if (s->poll_bits &&
                axera_vin_decode(s->poll_off, &qr, &qc, &qf) &&
                qr == range && qc == ch) {
                ev |= s->poll_bits;
            }
            s->raw[range][ch] |= ev;
        }
        axera_vin_update_irq(s, range);
    }
}

static void axera_vin_tick(void *opaque)
{
    AxeraVinState *s = opaque;

    /*
     * Real hardware sees no frame start/end before the sensor streams:
     * the AX620U driver unmasks its IFE WDMA frame-start event while the
     * ISP is still being set up, and a frame-start delivered then sends
     * isp_irq_thread -> irq_process_ife_wdma_fsof into VREG command
     * buffers that do not exist yet (kernel paging fault in
     * DRV_VREG_IO_WRITE32_CMD_BUF_INIT, or the same fault in the App's
     * userspace copy of that path).
     */
    if (s->gate_on_sensor && !s->sensor_streaming) {
        /* nothing */
    } else if (s->itp_jobs && (s->itp_busy || s->itp_qn ||
                               s->itp_unacked || s->settling)) {
        /* the last frame is still in the ITP: this one ends after it */
        s->frame_owed = true;
    } else {
        axera_vin_frame(s);
    }
    timer_mod_ns(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 NANOSECONDS_PER_SECOND / s->fps);
}

/*
 * itp-jobs (AX620E). The ISP is two engines fed by the CDMA, and
 * ax_proton.ko moves each sensor frame through them with three kernel
 * threads (live trace, 1 and 2 vCPUs):
 *
 *   IFE  sensor -> raw frame in DDR. Its events ride the sensor: frame
 *        start r0/ch1 bit0 (isp_irq_task_0 __vin_pipe_sns_fsof_irq_exec_
 *        node pushes the IFE list, CDMA request 3, pages 0x02400-0x02414)
 *        and WDMA done r0/ch4 bit21 (__vin_ife_wdma_done_irq_exec_node ->
 *        vin_ife_node_comm_wdma_img_backward sends the raw block on).
 *   ITP  raw frame -> YUV. vin_nor_sch_0 runs the frame through the
 *        pipe's nodes (vin_ipc_pipe_process: ..._wdma_dehaze_forward
 *        looks the raw block up), then vin_nor_sch_1 (vin_ipc_pipe_
 *        trigger -> ax_regio_slave_trigger) pushes the frame's ITP job:
 *        CDMA request 29, one list of five vertical partitions (the
 *        ITP's register pages 0x02480-0x024d0, YUV WDMA Y at 0x024d0118
 *        in each), followed by three small lists (requests 22 and 23,
 *        page 0x024cf; request 9, page 0x02495). The first partition
 *        clears every range-1 event and masks YUV WDMA done (r1/ch9 bit0)
 *        and AF done (r1/ch6 bit13); the last unmasks both, so they fire
 *        once, when the ITP has finished the whole frame. isp_irq_task_1
 *        then takes them: __vin_yuv_wdma_done_irq_exec_node pops the
 *        oldest job and vin_itp_raw_pre_node_comm_rdma_img_backward
 *        gives its raw block back to the pool.
 *
 * The driver relies on two orderings the silicon's timing gives it:
 *
 *   1. a job's done events follow that job. Raised on the sensor tick
 *      instead, they fell on whatever frame was oldest.
 *   2. a job's done events have been taken (isp_irq_task_1 has run its
 *      backward chain) before the next frame's IFE WDMA done: the ITP
 *      works a frame in a few milliseconds, well inside a frame period.
 *      When the pipe is behind, vin_nor_sch_0 drops the oldest frame it
 *      holds; with a done still to come for that frame, isp_irq_task_1
 *      then gives back the next frame's raw block, the one being
 *      forwarded.
 *
 * Broken, either one oopses the forward (NULL + 0xc8 at
 * vin_itp_rgb_pst_node_comm_wdma_dehaze_forward+0x84; on one vCPU at
 * vin_ai3dnr_node_mode2_event_backward). With CPU1 online, or on a
 * loaded host, the guest lags behind QEMU's clock by more than a frame
 * often enough (11% of frames in a 2-vCPU trace saw their job finish
 * after the next frame end) for that to happen within minutes.
 *
 * So, here:
 *   - a push of a list writing the YUV WDMA is an ITP job. Jobs run one
 *     at a time, in order: a job starts (its list is applied and the
 *     picture painted) when the ITP is free and the last job's done
 *     events are taken, and finishes itp-us later, raising the range-1
 *     events its list unmasked. The sensor tick raises range 0 only.
 *   - a sensor frame that ends while a job is queued, in the ITP or not
 *     yet taken by the driver waits, and ends as soon as it is
 *     (axera_vin_itp_idle()). "Taken" is the guest's progress, not
 *     QEMU's clock: the driver has cleared the done events, and since
 *     then every vCPU has run settle-us of its own time or is idle
 *     (halted, nothing pending) -- what took the silicon's interrupt
 *     thread a fraction of the frame period. A frame owed that long is
 *     the sensor's one frame late; frames that fell due meanwhile are
 *     lost, as when the silicon's pipe cannot keep up.
 */
static void axera_vin_itp_idle(AxeraVinState *s);

static void axera_vin_itp_next(AxeraVinState *s)
{
    if (s->itp_busy || s->itp_unacked || !s->itp_qn) {
        return;
    }
    s->itp_busy = true;
    axera_vin_scan_cdma(s, s->itp_q[s->itp_qh]);
    s->itp_qh = (s->itp_qh + 1) % ITP_JOBS_MAX;
    s->itp_qn--;
    timer_mod_ns(s->itp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 (int64_t)s->itp_us * SCALE_US);
}

static void axera_vin_itp_done(void *opaque)
{
    AxeraVinState *s = opaque;
    int ch;

    if (s->itp_busy) {
        s->itp_busy = false;
        for (ch = 0; ch < AXERA_VIN_CHANNELS; ch++) {
            s->raw[1][ch] |= s->mask[1][ch];
            if (s->mask[1][ch]) {
                s->itp_unacked = true;
            }
        }
        axera_vin_update_irq(s, 1);
        if (s->itp_unacked) {
            /* a driver that never clears them: go on after a second */
            timer_mod_ns(s->itp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                         + NANOSECONDS_PER_SECOND);
            return;
        }
    } else if (s->itp_unacked) {
        s->itp_unacked = false;
        axera_vin_itp_settle(s);
    }
    axera_vin_itp_next(s);
    axera_vin_itp_idle(s);
}

/* A vCPU's own run time: what it has executed, whatever the host did */
static int64_t axera_vin_cpu_ns(CPUState *cs)
{
#ifdef __linux__
    clockid_t id;
    struct timespec ts;

    if (cs->thread && !pthread_getcpuclockid(cs->thread->thread, &id) &&
        !clock_gettime(id, &ts)) {
        return ts.tv_sec * NANOSECONDS_PER_SECOND + ts.tv_nsec;
    }
#endif
    return get_clock();
}

static void axera_vin_itp_settle(AxeraVinState *s)
{
    CPUState *cs;
    int n = 0;

    CPU_FOREACH(cs) {
        n = MAX(n, cs->cpu_index + 1);
    }
    s->settling = true;
    s->settle_ns = get_clock();
    s->settle_cpu_ns = g_renew(int64_t, s->settle_cpu_ns, n);
    CPU_FOREACH(cs) {
        s->settle_cpu_ns[cs->cpu_index] = axera_vin_cpu_ns(cs);
    }
    timer_mod_ns(s->settle_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * SCALE_US);
}

static bool axera_vin_itp_settled(AxeraVinState *s)
{
    int64_t need = (int64_t)s->settle_us * SCALE_US;
    CPUState *cs;

    if (get_clock() - s->settle_ns > NANOSECONDS_PER_SECOND) {
        return true;    /* a guest this stuck has bigger problems */
    }
    CPU_FOREACH(cs) {
        if (!(cs->halted && !cpu_has_work(cs)) &&
            axera_vin_cpu_ns(cs) - s->settle_cpu_ns[cs->cpu_index] < need) {
            return false;
        }
    }
    return true;
}

static void axera_vin_settle_poll(void *opaque)
{
    AxeraVinState *s = opaque;

    if (!s->settling) {
        return;
    }
    if (!axera_vin_itp_settled(s)) {
        timer_mod_ns(s->settle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200 * SCALE_US);
        return;
    }
    s->settling = false;
    axera_vin_itp_idle(s);
}

/* The ITP has nothing left: a sensor frame that waited for it ends */
static void axera_vin_itp_idle(AxeraVinState *s)
{
    if (s->frame_owed && !s->itp_busy && !s->itp_qn && !s->itp_unacked &&
        !s->settling) {
        s->frame_owed = false;
        axera_vin_frame(s);
    }
}

static void axera_vin_reset(DeviceState *dev)
{
    AxeraVinState *s = AXERA_VIN(dev);
    int range;

    memset(s->mask, 0, sizeof(s->mask));
    memset(s->raw, 0, sizeof(s->raw));
    timer_del(s->itp_timer);
    s->itp_qh = s->itp_qn = 0;
    s->itp_busy = s->itp_unacked = false;
    timer_del(s->settle_timer);
    s->settling = s->frame_owed = false;
    s->cdma_info0 = 0;
    s->cdma_info1 = 0;
    s->isp.ref_set = false;
    s->isp_stable = 0;
    memset(&s->isp_cand, 0, sizeof(s->isp_cand));
    memset(s->painted, 0, sizeof(s->painted));
    for (range = 0; range < AXERA_VIN_RANGES; range++) {
        qemu_irq_lower(s->irq[range]);
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void axera_vin_sensor_streaming(void *opaque, int n, int level)
{
    AxeraVinState *s = opaque;

    if (getenv("AXERA_VIN_DEBUG")) {
        fprintf(stderr, "axera-vin: sensor streaming %d\n", level);
    }
    s->sensor_streaming = level;
}

static void axera_vin_init(Object *obj)
{
    AxeraVinState *s = AXERA_VIN(obj);
    int range;

    memory_region_init_io(&s->iomem, obj, &axera_vin_ops, s, TYPE_AXERA_VIN,
                           AXERA_VIN_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (range = 0; range < AXERA_VIN_RANGES; range++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[range]);
    }
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, axera_vin_tick, s);
    s->itp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, axera_vin_itp_done, s);
    s->settle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, axera_vin_settle_poll,
                                   s);
    qdev_init_gpio_in_named(DEVICE(obj), axera_vin_sensor_streaming,
                            "sensor-streaming", 1);
    s->isp_reg = g_new0(uint32_t, ISP_REGS);
}

static const Property axera_vin_props[] = {
    DEFINE_PROP_UINT32("fps", AxeraVinState, fps, 30),
    DEFINE_PROP_BOOL("itp-jobs", AxeraVinState, itp_jobs, false),
    DEFINE_PROP_UINT32("itp-us", AxeraVinState, itp_us, 5000),
    DEFINE_PROP_UINT32("settle-us", AxeraVinState, settle_us, 2000),
    DEFINE_PROP_BOOL("gate-on-sensor", AxeraVinState, gate_on_sensor, false),
    DEFINE_PROP_BOOL("cdma-irq-regs", AxeraVinState, cdma_irq_regs, false),
    DEFINE_PROP_UINT32("quiet-off", AxeraVinState, quiet_off, 0),
    DEFINE_PROP_UINT32("quiet-bits", AxeraVinState, quiet_bits, 0),
    DEFINE_PROP_UINT32("poll-off", AxeraVinState, poll_off, 0),
    DEFINE_PROP_UINT32("poll-bits", AxeraVinState, poll_bits, 0),
    DEFINE_PROP_UINT32("isp-layout", AxeraVinState, isp_layout, 0),
    DEFINE_PROP_UINT32("cdma-list-off", AxeraVinState, cdma_list_off, 0),
    DEFINE_PROP_UINT32("cdma-list-off2", AxeraVinState, cdma_list_off2, 0),
    DEFINE_PROP_UINT64("cdma-list-base", AxeraVinState, cdma_list_base, 0),
    DEFINE_PROP_LINK("sensor", AxeraVinState, sensor, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_UINT64("ae-ref", AxeraVinState, ae_ref, 0),
    DEFINE_PROP_UINT32("ae-shift", AxeraVinState, ae_shift, 4),
    DEFINE_PROP_UINT32("ae-black", AxeraVinState, ae_black, 1024),
    DEFINE_PROP_BOOL("isp-model", AxeraVinState, isp_model, true),
    DEFINE_PROP_UINT32("isp-ref-k", AxeraVinState, isp_ref_k, 1000),
};

static void axera_vin_realize(DeviceState *dev, Error **errp)
{
    AxeraVinState *s = AXERA_VIN(dev);

    if (s->fps < 1 || s->fps > 240) {
        error_setg(errp, "axera-vin: fps must be 1..240, not %u", s->fps);
    }

}

static void axera_vin_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = axera_vin_realize;
    device_class_set_legacy_reset(DEVICE_CLASS(klass), axera_vin_reset);
    device_class_set_props(DEVICE_CLASS(klass), axera_vin_props);
}

static const TypeInfo axera_vin_info = {
    .name          = TYPE_AXERA_VIN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraVinState),
    .instance_init = axera_vin_init,
    .class_init    = axera_vin_class_init,
};

static void axera_vin_register_types(void)
{
    type_register_static(&axera_vin_info);
}

type_init(axera_vin_register_types)
