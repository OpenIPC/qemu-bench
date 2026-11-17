/*
 * Axera AX_DMAPER: a generic LLI-descriptor memory-to-memory DMA engine
 * at 0x048a0000 (boot/bl1/driver/dmaper/dmaper.h, dmaper.c). Confirmed
 * from a real flash-file= boot of the AX620E SPI NOR dump spinning
 * forever (25M+ accesses in a 40s run) inside axi_dma_wait_xfer_done()'s
 * `while (!readl(AX_DMAPER_INT_STA(ch))) ;` -- a real hardware
 * completion poll with NO timeout of its own, same shape as every other
 * "write enable/start, poll a ready/done bit" gap this project has
 * already hit (axera-pllc.c, axera-ddr-pll.c, the AX620U clock regbank),
 * except this one genuinely needs to move real bytes: BL1 stages a real
 * SPI NOR read into a scratch buffer, then uses this DMA engine to copy
 * it into the image's final DRAM destination (confirmed empirically --
 * before this device existed, that final destination stayed all-zero
 * forever even though the real file's bytes checksum correctly at the
 * scratch location, and BL1's own axi_dma_xfer_start()/
 * axi_dma_wait_xfer_done() pair is the only thing between the two).
 *
 * Real register shape (dmaper.c's ax_dma_per_set_lli()): software
 * builds an "LLI" descriptor -- {u64 sar; u64 dar; a 64-bit ctrl
 * bitfield; a 64-bit data bitfield with block_ts:22 at bit 37} -- in
 * its OWN memory, then writes only that descriptor's address into
 * AX_DMAPER_CHN_LLI_H/L(ch). Writing AX_DMAPER_START's channel bit is
 * real hardware being told "go fetch that descriptor and do the
 * transfer it describes" -- so this model does exactly that: read the
 * 32-byte descriptor back out of guest memory, pull src/dst/length out
 * of it (xfer_len = block_ts << ctrl.src_tr_wridth, the exact inverse
 * of how ax_dma_per_set_lli() computed block_ts), copy that many bytes
 * guest-memory-to-guest-memory, then set the channel's completion bit
 * (AX_DMAPER_INT_STA(ch) bit 0, CHN_TRANSF_DONE) for the poll to see.
 * AX_DMAPER_INT_CLR(ch) clears it back, matching
 * axi_dma_wait_xfer_done()'s own cleanup write.
 *
 * Every other register (CHN_EN, CTRL, clock/reset helpers, priority,
 * the error registers) is plain RAM-backed readback -- BL1's own
 * driver only read-modify-writes them, never polls a hardware-driven
 * bit in any of them.
 *
 * INT_STA(ch) is NOT the raw completion flip-flop: real hardware
 * layers it as INT_STA(ch) = INT_RAW(ch) & INT_MASK(ch), and ALSO
 * ORs every channel's raw bit up into one combined
 * INT_GLB_STA = INT_GLB_RAW & INT_GLB_MASK (dmaper.h's GLB_CHN_INT =
 * GENMASK(15,0), one bit per channel) that the real external
 * interrupt line is actually wired from -- standard two-level
 * raw/mask/status hardware IRQ shape, but with the OPPOSITE polarity
 * from the usual "1 = masked" convention: dmaper.c's own
 * axi_dma_xfer_start() writes BIT(ch) into BOTH AX_DMAPER_INT_GLB_MASK
 * and AX_DMAPER_INT_MASK(ch) *before* starting the transfer it then
 * polls AX_DMAPER_INT_STA(ch) for, so on this IP a set mask bit
 * ENABLES that channel's bit at STA, a clear one blocks it (confirmed
 * the hard way: with the usual &~MASK formula, BL1's own self-enable
 * write permanently masked its own completion, hanging before even
 * the first console banner print).
 *
 * BL1 never needed the GLB layer at all (it just polls INT_STA(ch)
 * directly with interrupts never even unmasked at the GIC), but the
 * real kernel's closed-source "axera,dma-per" driver's ISR almost
 * certainly checks INT_GLB_STA first as a cheap combined check before
 * breaking out which channel -- confirmed by symptom: before this
 * model set INT_GLB_RAW/STA at all (they read back as whatever was
 * last written, i.e. 0 forever), the real transfer and per-channel
 * INT_STA/INT_CLR handshake was already provably correct (clean 1:1
 * do_transfer/INT_CLR pairs) yet the guest's own GIC ISENABLER for
 * this IRQ still toggled on/off ~300 times per real completion --
 * Linux's own spurious/unhandled level-IRQ backoff
 * (kernel/irq/spurious.c), which only makes sense if the real ISR
 * kept running, finding GLB_STA==0, and returning IRQ_NONE despite
 * the per-channel bit genuinely being set.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "hw/misc/axera-dmaper.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraDmaperState, AXERA_DMAPER)

#define AXERA_DMAPER_SIZE       0x400

#define DMAPER_CHN_EN           0x00
#define DMAPER_CHN_PAUSE        0x04
#define DMAPER_START            0x0c
#define DMAPER_PAUSE_STA        0x10
#define DMAPER_CHN_ABORT        0xa0
#define DMAPER_CHN_LLI_H(ch)    (0x20 + (ch) * 8)
#define DMAPER_CHN_LLI_L(ch)    (0x24 + (ch) * 8)
#define DMAPER_INT_GLB_MASK     0xd8
#define DMAPER_INT_GLB_RAW      0xdc
#define DMAPER_INT_GLB_STA      0xe0
#define DMAPER_INT_MASK(ch)     (0xf4 + (ch) * 0x10)
#define DMAPER_INT_CLR(ch)      (0xf8 + (ch) * 0x10)
#define DMAPER_INT_RAW(ch)      (0xfc + (ch) * 0x10)
#define DMAPER_INT_STA(ch)      (0x100 + (ch) * 0x10)

#define DMAPER_MAX_CHN          16
#define CHN_TRANSF_DONE         BIT(0)

/* The real engine takes real time to move bytes; completing (and
 * raising the IRQ) synchronously inside the DMAPER_START write let
 * the interrupt assert before the guest's own dmaengine submission
 * code had gotten back from that same MMIO store to arm its wait --
 * the real ISR then found nothing ready, left the level line
 * asserted, and the kernel re-entered it in a tight storm (observed:
 * ~7000 GIC enable/disable cycles for this IRQ per boot) instead of
 * ever satisfying wait_for_completion_timeout(). Same race the
 * project's other engines (axera-vcmd.c, sstar-vpu.c, nvt-h26xdec.c)
 * already avoid by deferring completion onto a timer. */
#define AXERA_DMAPER_LATENCY_NS (1 * SCALE_MS)

struct AxeraDmaperState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer completion_timer;
    uint32_t pending_chn;
    uint32_t mem[AXERA_DMAPER_SIZE / 4];

    /*
     * Peripheral flow control (audio). The real driver
     * (ax_dma_per_set_req_id()) routes a channel to a peripheral
     * request id through set/clear register pairs in the shared
     * clk-periph syscon at 0x04870000: +0x158-8k (set) / +0x15c-8k
     * (clear), k = ch / 5, a 6-bit request field at (ch % 5) * 6.
     * "reqmux" below models just that 0x30-byte window (mapped by the
     * machine over the passive syscon regbank) so the engine knows
     * which request line paces which channel.
     */
    MemoryRegion reqmux_mem;
    uint32_t reqmux_en[2];      /* +0x130 / +0x138 request enables */
    uint32_t reqmux[4];
    uint64_t paced_reqs;        /* property: request ids to honour */
    uint64_t block_reqs;        /* property: block-mode handshake ids */
    uint64_t req_level;
    bool in_service;
    QEMUBH *service_bh;
    struct {
        bool active;
        uint64_t sar, dar, len, done;
        unsigned unit;
        bool dev_to_mem;
    } pch[DMAPER_MAX_CHN];
};

/*
 * BL1's own real axi_dma_wait_xfer_done() (dmaper.c) never enables or
 * waits on an interrupt at all -- it just spins polling
 * AX_DMAPER_INT_STA(ch) directly, so this model never needed an IRQ
 * line for that consumer to work. But once the real kernel's own
 * dmaengine framework uses this same real block (confirmed: the real
 * devicetree's spi@1A00000 "dmas" property resolves, via its own
 * phandle, to this exact device, not a different one), its generic
 * wait_for_completion_timeout() needs a genuine interrupt to signal
 * the dmaengine callback that wakes it -- a polled status bit a
 * sleeping kernel thread never looks at is invisible to it, so
 * without this the real transfer completes correctly (the right
 * bytes really do move) but the kernel never finds out and times out
 * regardless ("DMA transaction timed out" -- confirmed live: the
 * real DMAPER_START write does happen, right before the timeout
 * fires). Harmless for BL1: real silicon only delivers this SPI
 * interrupt to a CPU that unmasked it at the GIC distributor, which
 * BL1 (running polled, with VBAR_EL3 never programmed) never does.
 */
/* Recompute channel ch's masked status from its raw bits, then the
 * combined GLB_STA the real external IRQ line is wired from, and
 * finally the line itself. Every raw-bit or mask write funnels
 * through this. */
static void axera_dmaper_update_irq(AxeraDmaperState *s, unsigned ch)
{
    uint32_t glb_raw = 0;
    unsigned i;

    s->mem[DMAPER_INT_STA(ch) / 4] =
        s->mem[DMAPER_INT_RAW(ch) / 4] & s->mem[DMAPER_INT_MASK(ch) / 4];

    for (i = 0; i < DMAPER_MAX_CHN; i++) {
        if (s->mem[DMAPER_INT_RAW(i) / 4]) {
            glb_raw |= BIT(i);
        }
    }
    s->mem[DMAPER_INT_GLB_RAW / 4] = glb_raw;
    s->mem[DMAPER_INT_GLB_STA / 4] = glb_raw & s->mem[DMAPER_INT_GLB_MASK / 4];

    qemu_set_irq(s->irq, s->mem[DMAPER_INT_GLB_STA / 4] != 0);
}

static void axera_dmaper_do_transfer(AxeraDmaperState *s, unsigned ch)
{
    uint8_t desc[32];
    uint64_t sar, dar, ctrl, data;
    uint32_t src_tr_wridth, block_ts;
    uint64_t xfer_len;
    bool sinc, dinc;
    uint8_t *buf;
    uint64_t i;
    uint64_t lli_addr;

    lli_addr = ((uint64_t)s->mem[DMAPER_CHN_LLI_H(ch) / 4] << 32) |
               s->mem[DMAPER_CHN_LLI_L(ch) / 4];

    if (address_space_read(&address_space_memory, lli_addr,
                           MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc)) !=
        MEMTX_OK) {
        return;
    }

    sar = ldq_le_p(desc + 0);
    dar = ldq_le_p(desc + 8);
    ctrl = ldq_le_p(desc + 16);
    data = ldq_le_p(desc + 24);

    src_tr_wridth = (ctrl >> 27) & 0x7;
    sinc = (ctrl >> 31) & 1;
    dinc = (ctrl >> 30) & 1;
    block_ts = (data >> 37) & 0x3fffff;
    xfer_len = (uint64_t)block_ts << src_tr_wridth;

    if (xfer_len == 0) {
        return;
    }

    /*
     * ax_dma_per_set_lli()'s DMA_DEV_TO_MEM case sets sinc=0: the real
     * source here is a device register's own FIFO (the SPI controller's
     * DR, for every real transfer this model has seen), which drains
     * one real byte per 4-byte-wide bus access -- not `xfer_len`
     * sequential source addresses -- so a non-incrementing side has to
     * be read (or written, for the symmetric MEM_TO_DEV case) one
     * fixed-address access at a time, same as real silicon stepping a
     * single peripheral register through the transfer instead of
     * walking memory.
     */
    buf = g_malloc(xfer_len);

    if (sinc) {
        if (address_space_read(&address_space_memory, sar,
                               MEMTXATTRS_UNSPECIFIED, buf, xfer_len) !=
            MEMTX_OK) {
            g_free(buf);
            return;
        }
    } else {
        for (i = 0; i < xfer_len; i++) {
            uint32_t word = 0;

            address_space_read(&address_space_memory, sar,
                               MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
            buf[i] = word & 0xff;
        }
    }

    if (dinc) {
        address_space_write(&address_space_memory, dar,
                            MEMTXATTRS_UNSPECIFIED, buf, xfer_len);
    } else {
        for (i = 0; i < xfer_len; i++) {
            uint32_t word = buf[i];

            address_space_write(&address_space_memory, dar,
                                MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
        }
    }

    g_free(buf);

    s->mem[DMAPER_INT_RAW(ch) / 4] |= CHN_TRANSF_DONE;
    axera_dmaper_update_irq(s, ch);
}

static unsigned axera_dmaper_chn_req(AxeraDmaperState *s, unsigned ch)
{
    return (s->reqmux[ch / 5] >> ((ch % 5) * 6)) & 0x3f;
}

static bool axera_dmaper_chn_paced(AxeraDmaperState *s, unsigned ch)
{
    return s->paced_reqs & (1ULL << axera_dmaper_chn_req(s, ch));
}

/*
 * Paced (peripheral-handshake) transfer: the real ax_dma_per driver
 * issues every LLI as its own one-block transfer -- for a cyclic
 * (audio) descriptor its tasklet reads the channel's LLI register
 * back, finds which period just finished, and restarts the channel on
 * that LLI's next pointer -- so one START here still means exactly one
 * descriptor. Unlike the memory/SPI case it may only move data while
 * the peripheral's request line is high, one peripheral-width word at
 * a time, which is what keeps ALSA's period interrupts at the I2S
 * serial clock's real pace.
 */
static void axera_dmaper_service(AxeraDmaperState *s)
{
    unsigned ch;
    bool progress = true;

    if (s->in_service) {
        return;
    }
    s->in_service = true;
    while (progress) {
        progress = false;
        for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
            unsigned req = axera_dmaper_chn_req(s, ch);
            uint32_t word = 0;
            MemTxResult r;

            if (!s->pch[ch].active || !(s->req_level & (1ULL << req))) {
                continue;
            }
            if (s->pch[ch].dev_to_mem) {
                r = address_space_read(&address_space_memory, s->pch[ch].sar,
                                       MEMTXATTRS_UNSPECIFIED, &word,
                                       s->pch[ch].unit);
                r |= address_space_write(&address_space_memory,
                                         s->pch[ch].dar + s->pch[ch].done,
                                         MEMTXATTRS_UNSPECIFIED, &word,
                                         s->pch[ch].unit);
            } else {
                r = address_space_read(&address_space_memory,
                                       s->pch[ch].sar + s->pch[ch].done,
                                       MEMTXATTRS_UNSPECIFIED, &word,
                                       s->pch[ch].unit);
                r |= address_space_write(&address_space_memory,
                                         s->pch[ch].dar,
                                         MEMTXATTRS_UNSPECIFIED, &word,
                                         s->pch[ch].unit);
            }
            if (r != MEMTX_OK) {
                /* A rejected peripheral access never drains the FIFO,
                 * so the request would stay high and this loop would
                 * spin forever; park the channel instead. */
                qemu_log_mask(LOG_GUEST_ERROR,
                              "axera-dmaper: ch%u bus error, stopping\n", ch);
                s->pch[ch].active = false;
                continue;
            }
            s->pch[ch].done += s->pch[ch].unit;
            progress = true;
            if (s->pch[ch].done >= s->pch[ch].len) {
                s->pch[ch].active = false;
                s->mem[DMAPER_INT_RAW(ch) / 4] |= CHN_TRANSF_DONE;
                axera_dmaper_update_irq(s, ch);
            }
        }
    }
    s->in_service = false;
}

static void axera_dmaper_service_bh(void *opaque)
{
    axera_dmaper_service(opaque);
}

static void axera_dmaper_start_paced(AxeraDmaperState *s, unsigned ch)
{
    uint8_t desc[32];
    uint64_t ctrl, data, lli_addr;
    unsigned src_w, dst_w;
    bool sinc, dinc;

    lli_addr = ((uint64_t)s->mem[DMAPER_CHN_LLI_H(ch) / 4] << 32) |
               s->mem[DMAPER_CHN_LLI_L(ch) / 4];
    if (address_space_read(&address_space_memory, lli_addr,
                           MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc)) !=
        MEMTX_OK) {
        return;
    }
    ctrl = ldq_le_p(desc + 16);
    data = ldq_le_p(desc + 24);
    src_w = (ctrl >> 27) & 0x7;
    dst_w = (ctrl >> 24) & 0x7;
    sinc = (ctrl >> 31) & 1;
    dinc = (ctrl >> 30) & 1;

    s->pch[ch].sar = ldq_le_p(desc + 0);
    s->pch[ch].dar = ldq_le_p(desc + 8);
    s->pch[ch].len = (uint64_t)((data >> 37) & 0x3fffff) << src_w;
    s->pch[ch].done = 0;
    s->pch[ch].dev_to_mem = !sinc && dinc;
    /* peripheral side's bus width; the I2S FIFO port is 32-bit */
    s->pch[ch].unit = MIN(1u << (s->pch[ch].dev_to_mem ? src_w : dst_w), 4u);
    s->pch[ch].active = s->pch[ch].len != 0;
    if (getenv("AXERA_DMAPER_DEBUG")) {
        qemu_log("axera-dmaper: paced ch%u req 0x%x lli 0x%" PRIx64
                 " sar 0x%" PRIx64 " dar 0x%" PRIx64 " len %" PRIu64
                 " unit %u %s\n", ch, axera_dmaper_chn_req(s, ch), lli_addr,
                 s->pch[ch].sar, s->pch[ch].dar, s->pch[ch].len,
                 s->pch[ch].unit, s->pch[ch].dev_to_mem ? "dev->mem" : "mem->dev");
    }
    axera_dmaper_service(s);
}

static void axera_dmaper_req_in(void *opaque, int req, int level)
{
    AxeraDmaperState *s = opaque;

    if (level) {
        unsigned ch;

        s->req_level |= 1ULL << req;
        /*
         * Not inline either: a request typically rises inside the
         * peripheral's own MMIO handler (the I2S raises its TX request
         * when the guest enables the transmitter or flushes its FIFO),
         * and the transfer then has to access that same peripheral --
         * a re-entrant access QEMU blocks ("Blocked re-entrant IO"),
         * which parked the TX channel before a single word moved.
         */
        qemu_bh_schedule(s->service_bh);
        /* Release a block-mode channel waiting on this request. Not
         * inline: the request is raised from inside the peripheral's
         * own MMIO handler, which the transfer then has to read. */
        for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
            if ((s->pending_chn & BIT(ch)) &&
                axera_dmaper_chn_req(s, ch) == req) {
                timer_mod_anticipate(&s->completion_timer,
                                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                     AXERA_DMAPER_LATENCY_NS);
                break;
            }
        }
    } else {
        s->req_level &= ~(1ULL << req);
    }
}

static bool axera_dmaper_chn_blocked(AxeraDmaperState *s, unsigned ch);

static void axera_dmaper_stop_paced(AxeraDmaperState *s, uint32_t mask)
{
    unsigned ch;

    for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
        if (mask & BIT(ch)) {
            s->pch[ch].active = false;
            /* an aborted channel still waiting for its request must
             * not fire on the next, unrelated, one */
            if (axera_dmaper_chn_blocked(s, ch)) {
                s->pending_chn &= ~BIT(ch);
            }
        }
    }
}

static uint64_t axera_dmaper_reqmux_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    AxeraDmaperState *s = opaque;

    if (offset == 0x00 || offset == 0x08) {
        return s->reqmux_en[offset / 8];
    }
    if (offset >= 0x10 && offset < 0x30) {
        return s->reqmux[(0x2c - offset) / 8];
    }
    return 0;
}

static void axera_dmaper_reqmux_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    AxeraDmaperState *s = opaque;

    if (offset == 0x00 || offset == 0x08) {
        s->reqmux_en[offset / 8] |= value;
    } else if (offset >= 0x10 && offset < 0x30) {
        unsigned k = (0x2c - offset) / 8;

        if (offset & 4) {
            s->reqmux[k] &= ~value;     /* +0x15c - 8k: clear */
        } else {
            s->reqmux[k] |= value;      /* +0x158 - 8k: set */
        }
    }
}

static const MemoryRegionOps axera_dmaper_reqmux_ops = {
    .read = axera_dmaper_reqmux_read,
    .write = axera_dmaper_reqmux_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Block-mode peripheral handshake (SPI NOR RX, request id 1): the
 * channel is armed by DMAPER_START but may only move its block once
 * the peripheral raises its request, i.e. once its FIFO really holds
 * the data. U-Boot's sf read starts the channel first and only then
 * enables the SPI controller for the RX phase; copying unconditionally
 * after the fixed latency raced that second MMIO write and, whenever
 * the vCPU was descheduled in between, drained an empty FIFO into the
 * destination (whole 32 KiB chunks of zeros in the squashfs U-Boot
 * loads the kernel from). Channels routed elsewhere are unaffected.
 */
static bool axera_dmaper_chn_blocked(AxeraDmaperState *s, unsigned ch)
{
    unsigned req = axera_dmaper_chn_req(s, ch);

    return (s->block_reqs & (1ULL << req)) && !(s->req_level & (1ULL << req));
}

static void axera_dmaper_timer_cb(void *opaque)
{
    AxeraDmaperState *s = opaque;
    unsigned ch;

    for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
        if ((s->pending_chn & BIT(ch)) && !axera_dmaper_chn_blocked(s, ch)) {
            s->pending_chn &= ~BIT(ch);
            axera_dmaper_do_transfer(s, ch);
        }
    }
}

static void axera_dmaper_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AxeraDmaperState *s = opaque;
    unsigned ch;

    if (offset == DMAPER_START) {
        uint32_t immediate = 0;

        for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
            if (!(value & BIT(ch))) {
                continue;
            }
            if (axera_dmaper_chn_paced(s, ch)) {
                axera_dmaper_start_paced(s, ch);
            } else {
                immediate |= BIT(ch);
            }
        }
        if (immediate) {
            s->pending_chn |= immediate;
            timer_mod(&s->completion_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     AXERA_DMAPER_LATENCY_NS);
        }
        return;
    }
    if (offset == DMAPER_CHN_ABORT) {
        /* dma_chan_terminate_all(): abort, then drop CHN_EN/PAUSE */
        axera_dmaper_stop_paced(s, value);
        s->mem[offset / 4] = value;
        return;
    }
    if (offset == DMAPER_CHN_EN) {
        axera_dmaper_stop_paced(s, s->mem[offset / 4] & ~value);
        s->mem[offset / 4] = value;
        return;
    }

    for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
        if (offset == DMAPER_INT_CLR(ch)) {
            /* Write-1-to-clear on the RAW flip-flop; STA/GLB_STA
             * recompute from it. */
            s->mem[DMAPER_INT_RAW(ch) / 4] &= ~value;
            axera_dmaper_update_irq(s, ch);
            return;
        }
        if (offset == DMAPER_INT_MASK(ch)) {
            s->mem[offset / 4] = value;
            axera_dmaper_update_irq(s, ch);
            return;
        }
    }

    if (offset == DMAPER_INT_GLB_MASK) {
        s->mem[offset / 4] = value;
        axera_dmaper_update_irq(s, 0);
        return;
    }

    if (offset < AXERA_DMAPER_SIZE) {
        s->mem[offset / 4] = value;
    }
}

static uint64_t axera_dmaper_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraDmaperState *s = opaque;

    if (offset == DMAPER_PAUSE_STA) {
        /* dma_chan_pause() polls bit (ch + 1) after setting PAUSE(ch);
         * nothing here runs mid-burst, so a pause is always immediate. */
        return s->mem[offset / 4] | (s->mem[DMAPER_CHN_PAUSE / 4] << 1);
    }
    if (offset < AXERA_DMAPER_SIZE) {
        return s->mem[offset / 4];
    }
    return 0;
}

static const MemoryRegionOps axera_dmaper_ops = {
    .read = axera_dmaper_read,
    .write = axera_dmaper_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_dmaper_reset(DeviceState *dev)
{
    AxeraDmaperState *s = AXERA_DMAPER(dev);

    timer_del(&s->completion_timer);
    s->pending_chn = 0;
    memset(s->pch, 0, sizeof(s->pch));
    memset(s->reqmux, 0, sizeof(s->reqmux));
    memset(s->reqmux_en, 0, sizeof(s->reqmux_en));
}

static void axera_dmaper_realize(DeviceState *dev, Error **errp)
{
    AxeraDmaperState *s = AXERA_DMAPER(dev);

    timer_init_ns(&s->completion_timer, QEMU_CLOCK_VIRTUAL,
                 axera_dmaper_timer_cb, s);
    s->service_bh = qemu_bh_new_guarded(axera_dmaper_service_bh, s,
                                        &dev->mem_reentrancy_guard);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_dmaper_ops, s,
                          TYPE_AXERA_DMAPER, AXERA_DMAPER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    /* MMIO 1: request-mux window, 0x30 bytes at clk-periph + 0x130 */
    memory_region_init_io(&s->reqmux_mem, OBJECT(s), &axera_dmaper_reqmux_ops,
                          s, TYPE_AXERA_DMAPER "-reqmux", 0x30);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->reqmux_mem);
    qdev_init_gpio_in_named(dev, axera_dmaper_req_in, AXERA_DMAPER_REQ,
                            AXERA_DMAPER_NR_REQ);
}

static const Property axera_dmaper_props[] = {
    DEFINE_PROP_UINT64("paced-reqs", AxeraDmaperState, paced_reqs, 0),
    DEFINE_PROP_UINT64("block-reqs", AxeraDmaperState, block_reqs, 0),
};

static void axera_dmaper_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_dmaper_realize;
    device_class_set_legacy_reset(dc, axera_dmaper_reset);
    device_class_set_props(dc, axera_dmaper_props);
}

static const TypeInfo axera_dmaper_info = {
    .name = TYPE_AXERA_DMAPER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraDmaperState),
    .class_init = axera_dmaper_class_init,
};

static void axera_dmaper_register_types(void)
{
    type_register_static(&axera_dmaper_info);
}

type_init(axera_dmaper_register_types)
