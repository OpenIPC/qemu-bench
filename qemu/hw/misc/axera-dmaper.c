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
#include "hw/irq.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AXERA_DMAPER "axera-dmaper"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraDmaperState, AXERA_DMAPER)

#define AXERA_DMAPER_SIZE       0x400

#define DMAPER_START            0x0c
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

static void axera_dmaper_timer_cb(void *opaque)
{
    AxeraDmaperState *s = opaque;
    unsigned ch;

    for (ch = 0; ch < DMAPER_MAX_CHN; ch++) {
        if (s->pending_chn & BIT(ch)) {
            axera_dmaper_do_transfer(s, ch);
        }
    }
    s->pending_chn = 0;
}

static void axera_dmaper_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AxeraDmaperState *s = opaque;
    unsigned ch;

    if (offset == DMAPER_START) {
        if (value) {
            s->pending_chn |= value;
            timer_mod(&s->completion_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     AXERA_DMAPER_LATENCY_NS);
        }
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
}

static void axera_dmaper_realize(DeviceState *dev, Error **errp)
{
    AxeraDmaperState *s = AXERA_DMAPER(dev);

    timer_init_ns(&s->completion_timer, QEMU_CLOCK_VIRTUAL,
                 axera_dmaper_timer_cb, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_dmaper_ops, s,
                          TYPE_AXERA_DMAPER, AXERA_DMAPER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_dmaper_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_dmaper_realize;
    device_class_set_legacy_reset(dc, axera_dmaper_reset);
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
