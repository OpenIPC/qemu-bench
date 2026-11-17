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
#define CDMA_SUB_MAX    8192    /* register writes per command list */

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
    if (out->buf.compressed) {
        axera_fbuf_put(out->buf.y, pic);
        return;
    }
    /* A buffer that already holds the scene only needs the dot's rows */
    for (i = 0; i < VIN_PAINTED; i++) {
        if (s->painted[i].y == out->buf.y && s->painted[i].w == w &&
            s->painted[i].h == h) {
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

static void axera_vin_reg_write(AxeraVinState *s, hwaddr off, uint32_t val,
                                bool from_cdma);

static void axera_vin_scan_cdma(AxeraVinState *s, hwaddr top)
{
    g_autofree uint32_t *pairs = NULL;
    bool seen = false;
    int i, n;

    for (n = 0; n < VIN_OUTS; n++) {
        s->out[n].y_set = s->out[n].uv_set = false;
    }

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
                        uint32_t key = le32_to_cpu(pairs[2 * n + 1]);

                        /* What the CDMA engine does with every pair;
                         * here only the interrupt registers matter. */
                        if (s->cdma_irq_regs && (key >> 20) == 0x024) {
                            axera_vin_reg_write(s, key & 0xfffff,
                                                le32_to_cpu(pairs[2 * n]),
                                                true);
                        }
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
    if (seen) {
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
}

static void axera_vin_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    AxeraVinState *s = AXERA_VIN(opaque);

    if (s->isp_layout && off == s->cdma_list_off) {
        s->cdma_list = val;
    }
    if (s->isp_layout && off == s->cdma_list_off + 4 && (val & 1)) {
        axera_vin_scan_cdma(s, s->cdma_list_base +
                               ((hwaddr)s->cdma_list << 3));
    }
    if (s->cdma_list_off2 && off == s->cdma_list_off2) {
        s->cdma_list2 = val;
    }
    if (s->cdma_list_off2 && off == s->cdma_list_off2 + 4 && (val & 1)) {
        axera_vin_scan_cdma(s, s->cdma_list_base +
                               ((hwaddr)s->cdma_list2 << 3));
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

static void axera_vin_tick(void *opaque)
{
    AxeraVinState *s = opaque;
    int range, ch;

    /*
     * Real hardware sees no frame start/end before the sensor streams:
     * the AX620U driver unmasks its IFE WDMA frame-start event while the
     * ISP is still being set up, and a frame-start delivered then sends
     * isp_irq_thread -> irq_process_ife_wdma_fsof into VREG command
     * buffers that do not exist yet (kernel paging fault in
     * DRV_VREG_IO_WRITE32_CMD_BUF_INIT, or the same fault in the App's
     * userspace copy of that path).
     */
    if (!s->gate_on_sensor || s->sensor_streaming) {
        for (range = 0; range < AXERA_VIN_RANGES; range++) {
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
    timer_mod_ns(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 NANOSECONDS_PER_SECOND / s->fps);
}

static void axera_vin_reset(DeviceState *dev)
{
    AxeraVinState *s = AXERA_VIN(dev);
    int range;

    memset(s->mask, 0, sizeof(s->mask));
    memset(s->raw, 0, sizeof(s->raw));
    s->cdma_info0 = 0;
    s->cdma_info1 = 0;
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
    qdev_init_gpio_in_named(DEVICE(obj), axera_vin_sensor_streaming,
                            "sensor-streaming", 1);
}

static const Property axera_vin_props[] = {
    DEFINE_PROP_UINT32("fps", AxeraVinState, fps, 30),
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
