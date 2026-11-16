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
 * PRAGMATIC ADDITION (explicitly user-authorized "fix whatever it takes",
 * NOT derived from real hardware behavior): offsets 0x1000/0x1004/0x1008,
 * found this session via live guest-kallsyms tracing to be a generic
 * "CDMA command descriptor" convention (info0=command/type word,
 * info1=target guest buffer address, ctrl0=trigger) implemented by a
 * separate shared software module (ax_base.ko's cmd_info0/1_cfg_set,
 * cmd_ctrl0_cfg_set), called from this driver's own
 * ax_regio_blk_io_write_within_cdma path -- confirmed landing inside
 * THIS device's own 0x100000 region, written continuously at frame
 * cadence with real guest buffer pointers, but with no determinable
 * real completion semantics (see this session's kaeru episodes for the
 * full derivation, including a retracted "separate hardware" theory).
 * On a nonzero write to ctrl0 (0x1008), treated as a trigger:
 *   1. a small synthetic byte pattern is DMA-written to the guest
 *      address found in info1 (0x1004) -- NOT a real decoded frame,
 *      just non-zero content, in case anything downstream ever reads
 *      it back;
 *   2. ctrl0 auto-clears to 0 (the common real-hardware "write-1-to-
 *      start, HW clears on completion" convention);
 *   3. MASK and RAW are force-set for VIN channel 2/range 0 (the
 *      channel this session confirmed, via a live kernel breakpoint on
 *      vin_irq_executor_add, the real guest driver never arms on its
 *      own -- used by AX_IVPS group 0) and for every channel of range 1
 *      (used by group 1, also never armed) -- a deliberate override of
 *      this model's normal masked-IRQ semantics, since the real driver
 *      never unmasks these slots itself and the normal path can
 *      therefore never fire for them. This is a guess-and-test hack,
 *      not a modeled register; see the header comment's own MASK
 *      POLARITY section for the (unsuccessful, reverted) attempt at a
 *      principled fix along similar lines.
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

#define TYPE_AXERA_VIN "axera-vin"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraVinState, AXERA_VIN)

#define AXERA_VIN_SIZE      0x100000
#define AXERA_VIN_RANGES    2
#define AXERA_VIN_CHANNELS  16

#define RANGE0_BASE   0x10
#define RANGE1A_BASE  0x80010
#define RANGE1A_CHANS 8
#define RANGE1B_BASE  0xc0010

/* Pragmatic CDMA command-descriptor block, see the file's own header
 * comment -- not part of the per-channel MASK/CLEAR/RAW/STATUS scheme. */
#define CDMA_INFO0  0x1000
#define CDMA_INFO1  0x1004
#define CDMA_CTRL0  0x1008

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
    /* Where this SoC's ISP puts its CDMA command trigger(s): AX620E's
     * single one at 0x1008; AX620U's IFE and ITP ones at 0x3008 and
     * 0x83008 (isp_ife/itp_cdma_cmd_cpu_trigger_* in its ax_proton.ko). */
    uint32_t cdma_ctrl;
    uint32_t cdma_ctrl2;
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

/* Pragmatic, explicitly user-authorized guess-and-test synthetic
 * "frame done" completion -- see the file's own header comment. Not a
 * modeled register behavior. */
static void axera_vin_cdma_trigger(AxeraVinState *s)
{
    uint8_t buf[4096];
    unsigned i;
    int ch;

    if (s->cdma_info1 != 0) {
        for (i = 0; i < sizeof(buf); i++) {
            buf[i] = (uint8_t)(i ^ (i >> 3));
        }
        address_space_write(&address_space_memory, s->cdma_info1,
                             MEMTXATTRS_UNSPECIFIED, buf, sizeof(buf));
    }

    /* VIN channel 2/range 0 (AX_IVPS group 0) and every channel of
     * range 1 (group 1): confirmed this session that the real guest
     * driver never arms these via vin_irq_executor_add, so the normal
     * masked path can never fire for them. Force both MASK and RAW so
     * this model's own status computation (MASK & RAW) is nonzero,
     * then assert the IRQ lines directly. */
    s->mask[0][2] |= 0xffffffffu;
    s->raw[0][2]  |= 0xffffffffu;
    for (ch = 0; ch < AXERA_VIN_CHANNELS; ch++) {
        s->mask[1][ch] |= 0xffffffffu;
        s->raw[1][ch]  |= 0xffffffffu;
    }
    axera_vin_update_irq(s, 0);
    axera_vin_update_irq(s, 1);
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
    if (off == CDMA_CTRL0) {
        return 0; /* auto-cleared */
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

static void axera_vin_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    AxeraVinState *s = AXERA_VIN(opaque);
    int range, ch, field;

    if (off == CDMA_INFO0) {
        s->cdma_info0 = val;
        return;
    }
    if (off == CDMA_INFO1) {
        s->cdma_info1 = val;
        return;
    }
    if (off == s->cdma_ctrl || (s->cdma_ctrl2 && off == s->cdma_ctrl2)) {
        if (getenv("AXERA_VIN_DEBUG") && val != 0) {
            static int n;
            if (n++ < 20) {
                qemu_log("axera-vin: cdma trigger [0x%05" HWADDR_PRIx "] = %08x\n",
                         off, (uint32_t)val);
            }
        }
        if (val != 0) {
            axera_vin_cdma_trigger(s);
        }
        return;
    }

    if (!axera_vin_decode(off, &range, &ch, &field)) {
        qemu_log_mask(LOG_UNIMP, "axera-vin: write [0x%05" HWADDR_PRIx
                      "] = 0x%08" PRIx64 "\n", off, val);
        return;
    }
    switch (field) {
    case FIELD_MASK:
        if (getenv("AXERA_VIN_DEBUG") && s->mask[range][ch] != val) {
            qemu_log("axera-vin: mask r%d ch%d [0x%05" HWADDR_PRIx "] %08x -> %08x\n",
                     range, ch, off, s->mask[range][ch], (uint32_t)val);
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
    DEFINE_PROP_UINT32("cdma-ctrl", AxeraVinState, cdma_ctrl, CDMA_CTRL0),
    DEFINE_PROP_UINT32("cdma-ctrl2", AxeraVinState, cdma_ctrl2, 0),
    DEFINE_PROP_UINT32("quiet-off", AxeraVinState, quiet_off, 0),
    DEFINE_PROP_UINT32("quiet-bits", AxeraVinState, quiet_bits, 0),
    DEFINE_PROP_UINT32("poll-off", AxeraVinState, poll_off, 0),
    DEFINE_PROP_UINT32("poll-bits", AxeraVinState, poll_bits, 0),
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
