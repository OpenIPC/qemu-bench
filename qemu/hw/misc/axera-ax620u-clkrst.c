/*
 * Axera AX620U's per-instance clock/reset-control block @ 0x01920000:
 * a bank of identical 0x100-byte register windows, one per peripheral
 * instance (base + instance*0x100), discovered the same way as this
 * SoC's other undocumented blocks -- no real devicetree confirms this
 * address, gdb-traced from this real dump's own BL1-equivalent (no
 * source, no symbols).
 *
 * BL1 configures an instance by writing to +0x00/+0x08 (some value,
 * confirmed from the real dump) and +0x04/+0x0c (0, a clear/disable),
 * then spins reading back +0x88 and ANDing it against a sparse
 * multi-bit mask (0x3f7fe2 for instance 1, the only one confirmed
 * live) waiting for *any* bit in that mask to come up -- same
 * "disable/reconfigure/enable, then poll a status word elsewhere in
 * the same window" shape already modelled for axera-pllc.c,
 * axera-ddr-pll.c and axera-ax620u-ddr-pll.c. Against plain storage
 * (confirmed: a bare hisi-regbank here still hangs), +0x88 reads back
 * 0 forever and BL1 spins with no further console output, ~390% host
 * CPU.
 *
 * +0x88's mask is not a plain "any bit = ready" test everywhere it's
 * used, though: BL1's generic per-instance wait helper (gdb-traced at
 * guest 0x287c, called with the instance number as its only arg) ANDs
 * the raw +0x88 value against that same 0x3f7fe2 mask to decide when
 * to stop spinning, but then re-ANDs it against 0x3f7fe0 (the same
 * mask with bit 1 cleared) to decide pass/fail: any of THOSE bits set
 * is treated as a latched hardware error and makes the helper return
 * -1, while bit 1 alone means success. This helper is reused for
 * instance 1's SPI-NAND-controller clock/IRQ wait on every
 * READ_FROM_CACHE/GET_FEATURE/READID response phase (axera-spi-nand.c
 * wires that into AX620U's NAND bad-block-table scan), not just the
 * one-time init call -- so a status word with *every* bit set, while
 * harmless to the simpler "nonzero" pollers elsewhere, makes this
 * specific helper see every error bit as latched and fail -1 on every
 * single call, which upstream BL1 code reads back as "page/block is
 * bad" regardless of the actual NAND contents. Confirmed live via
 * gdb: forcing READ_FROM_CACHE's emulated byte data to all-0xff made
 * no difference to the "bad" verdict, and breakpointing 0x287c's own
 * two masked comparisons showed the raw +0x88 read (0xffffffff) satisfies
 * the first (mask1) and always trips the second (mask2) as well.
 *
 * A second, separate instance (confirmed live: instance 0, +0x58)
 * uses a different, more standard shape: BL1 writes bit 0 of +0x58
 * to 1 (a "start" strobe), then polls that exact same offset waiting
 * for bit 0 to clear again on its own -- the self-clearing "start"
 * bit idiom hisi-regbank's own autoclear-offset/autoclear-mask
 * properties already model elsewhere in this tree, just needed here
 * on a *per-instance* offset rather than a single fixed one.
 *
 * As with axera-ax620u-ddr-pll.c, there's no real hardware lock delay
 * worth modelling for an emulator -- this device just makes every
 * instance's own +0x88 status word always read back with *only* bit 1
 * set (the one bit every confirmed-live consumer treats as "done",
 * with none of the bits any consumer treats as an error), and clears
 * bit 0 of every instance's own +0x58 immediately after it's written,
 * which trivially satisfies every poll BL1 runs here, while leaving
 * the rest of each instance's window (and the whole bank) as plain
 * read/write storage, same as hisi-regbank.
 *
 * Fixing the +0x88 status word (above) was not enough on its own to
 * make AX620U's NAND bad-block-table scan see real data, though:
 * gdb-traced further, BL1's SPI-NAND driver's READ_FROM_CACHE path
 * (guest 0x5b06, once it decides the transfer is big enough to use
 * this "IRQ-style" wait instead of a tight poll) programs a *second*,
 * genuinely-needs-real-bytes transfer directly into this same
 * per-instance window at guest 0x252c/0x281e: {sar_lo,sar_hi,dar_lo,
 * dar_hi} written straight to +0x00/+0x04/+0x08/+0x0c, a (length-1)
 * word at +0x10, and a direction word (1 = memory-to-device, 2 =
 * device-to-memory) at +0x24, always the sequence's last write, which
 * is this model's trigger to actually perform it. Confirmed live: sar
 * was the SPI-NAND controller's own DW_SPI_DR register address
 * (0x08000060) and dar the caller's real destination buffer, with
 * every other register this device already handles (+0x58, +0x88)
 * untouched by this specific sequence -- so, like AX_DMAPER's own
 * DMA_DEV_TO_MEM/MEM_TO_DEV cases, the device side is one fixed
 * address stepped a word at a time (draining/filling its FIFO) while
 * the memory side walks linearly a byte at a time.
 *
 * U-Boot's own SPI-NAND driver (drivers/spi/axera_spi.c's real,
 * available source calling into AX620U's closed axi_dma_xfer_start())
 * does NOT use that flat +0x00/+0x08/+0x10 shape at all, even though
 * it shares this same per-instance window and the same +0x24
 * direction-word trigger: it always builds a small LLI-style
 * descriptor in guest RAM first (malloc'd fresh per transfer, even an
 * unchained one -- {sar_lo,sar_hi,dar_lo,dar_hi,len-1,...} at the
 * descriptor's own +0/+4/+8/+0xc/+0x10) and writes a *pointer* to that
 * descriptor into +0x28, leaving the flat +0x00/+0x08/+0x10 registers
 * holding whatever BL1 last wrote to them (confirmed live: +0x08 kept
 * showing a stale, totally unrelated address from BL1's own earlier
 * use of this same register, long after U-Boot took over -- it's
 * simply never touched again once the descriptor-pointer path is in
 * use). This had looked, from the flat registers alone, exactly like
 * "the real destination buffer is never written" -- it isn't; it's
 * just described in RAM instead of being in the obvious register.
 * Confirmed live by dumping the descriptor U-Boot built for its own
 * NAND bad-block OOB-marker read: sar_lo=0x08000060 (DW_SPI_DR, same
 * as BL1's own shape), dar_lo = the real, correct destination buffer,
 * len-1 = 63 (a real 64-byte OOB read) -- all correct, all simply
 * unread by this model before this fix, which is why the real
 * hardware has always handled this fine while the emulator didn't.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/misc/axera-ax620u-clkrst.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraAx620uClkrstState, AXERA_AX620U_CLKRST)

#define CLKRST_SIZE             0x1000
#define INSTANCE_STRIDE         0x100
#define STATUS_OFF              0x88
#define STATUS_READY            0x00000002U
#define START_OFF               0x58
#define START_BIT               BIT(0)

#define DMA_SAR_OFF             0x00
#define DMA_DAR_OFF             0x08
#define DMA_LEN_OFF             0x10
#define DMA_DESC_OFF            0x28
#define DMA_DIR_OFF             0x24
#define DMA_DIR_MEM_TO_DEV      1
#define DMA_DIR_DEV_TO_MEM      2

/* Real dw-axi-dmac common-block channel-enable register; see its use
 * in axera_ax620u_clkrst_write() below. */
#define DMAC_CHEN_OFF           0x18
#define DMAC_CHEN_CH0_EN        BIT(0)

struct AxeraAx620uClkrstState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[CLKRST_SIZE / 4];
    /* Set whenever DMA_DESC_OFF is written, cleared whenever a
     * transfer actually fires -- see DMAC_CHEN_OFF's write handling
     * below for why. */
    bool dma_desc_dirty;
    /* A transfer ran from the direction-word trigger (no IRQ) and the
     * kernel has not enabled the channel for it yet. */
    bool dma_done_unsignalled;
    /* GIC SPI 5 ("dw_dmac@1920000" in ax620u.dts); wired up in
     * hw/arm/axera.c. Only ever raised for the DMAC_CHEN_OFF-triggered
     * (real kernel dmaengine) path, never for BL1/U-Boot's own
     * DMA_DIR_OFF-triggered one -- see both sites' comments below. */
    qemu_irq irq;
};

/*
 * The real dw-axi-dmac kernel driver treats this same window as its
 * 2-channel common+per-channel register layout (confirmed via
 * DMAC_CHEN_OFF above: COMMON_REG_LEN/CHAN_REG_LEN=0x100, so channel 0
 * is this bank's instance 1 -- the one this file's own do_dma() uses --
 * and channel 1 is instance 2, at 0x200-0x2ff) and reads channel 1's
 * own CH_INTSTATUS at +0x88 relative to ITS base, i.e. absolute 0x288,
 * to check for a pending completion -- exactly STATUS_OFF, landing on
 * the generic "every instance's +0x88 always reads STATUS_READY"
 * shortcut below meant for the OTHER (clock-ready-poll) consumers.
 * STATUS_READY is 0x2, which is also literally DWAXIDMAC_IRQ_DMA_TRF:
 * channel 1 is never driven by anything in this file (only instance 1,
 * channel 0, ever gets a real transfer), so this made the kernel's ISR
 * see a permanent, spurious "channel 1 transfer complete" on every
 * single read -- confirmed live: "dma dma0chan1: BUG: ... the head of
 * vc list is NULL!" repeating hundreds of times within the first
 * second of boot, eventually panicking a watchdog kthread from CPU
 * starvation. Carving out just this one absolute offset (not a wider
 * "instance 2" exclusion, since no consumer of instance 2 for anything
 * else is confirmed either way) keeps instance 0's and instance 1's
 * own confirmed-live STATUS_OFF consumers (BL1's clock-ready polls,
 * the NAND wait helper) working unchanged.
 */
#define DMAC_CH1_INTSTATUS_OFF  (2 * INSTANCE_STRIDE + STATUS_OFF)

static uint64_t axera_ax620u_clkrst_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    AxeraAx620uClkrstState *s = opaque;

    if (offset == DMAC_CH1_INTSTATUS_OFF) {
        return 0;
    }
    if ((offset % INSTANCE_STRIDE) == STATUS_OFF) {
        return STATUS_READY;
    }
    if (offset == DMAC_CHEN_OFF) {
        /*
         * The real kernel's dw-axi-dmac ISR reads this register as
         * its very first action on taking the completion IRQ (see
         * do_dma()'s own comment), to check whether the channel it's
         * about to handle is idle yet -- lowering the IRQ (GIC SPI 5,
         * level-sensitive) right here, independent of any later
         * write, is what lets do_dma() also pre-clear the enable bit
         * before raising it (see there) without re-introducing the
         * interrupt storm a value-independent, write-only lowering
         * scheme hit before: the ISR now sees "idle" on its first
         * check and takes its normal completion path instead of the
         * warn-then-retry one, which (confirmed live) otherwise never
         * actually completes the real kernel's own tx descriptor --
         * only the warn-then-retry path's own side-effecting write
         * was ever lowering the IRQ before, and that path doesn't run
         * the normal completion callback, so the higher-level
         * squashfs/MTD read it was for kept timing out and retrying
         * the exact same descriptor from scratch (confirmed live: the
         * same desc_ptr/sar/dar/count observed four times running).
         */
        qemu_irq_lower(s->irq);
    }
    return s->regs[offset / 4];
}

static void axera_ax620u_clkrst_do_block(uint64_t sar, uint64_t dar,
                                         uint32_t count, uint32_t dir);

static void axera_ax620u_clkrst_do_block(uint64_t sar, uint64_t dar,
                                         uint32_t count, uint32_t dir)
{
    uint32_t i;

    if (dir == DMA_DIR_DEV_TO_MEM) {
        /*
         * The device (sar) side genuinely needs one real MMIO access
         * per byte -- axera-spi-nand.c's DW_SPI_DR pops exactly one
         * byte off its rxq per read, a real side effect, so that part
         * can't be batched. The RAM (dar) side has no such side
         * effect and nothing about it changes between bytes except
         * the address, so re-walking the whole address-space region
         * tree from scratch for every single byte there was pure
         * waste -- confirmed live this made a multi-thousand-byte
         * transfer (routine during a real NAND verify/format pass)
         * take a very long real time under TCG even with icount's
         * sleep=off. address_space_cache_init's fast path resolves
         * this guest-RAM range to a direct host pointer once, up
         * front, instead of on every byte.
         */
        MemoryRegionCache cache;
        int64_t cached_len = address_space_cache_init(
            &cache, &address_space_memory, dar, count, true);

        for (i = 0; i < count; i++) {
            uint32_t word = 0;
            uint8_t byte;

            address_space_read(&address_space_memory, sar,
                               MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
            byte = word & 0xff;
            if (cached_len >= count) {
                address_space_stb_cached(&cache, i, byte,
                                         MEMTXATTRS_UNSPECIFIED, NULL);
            } else {
                address_space_write(&address_space_memory, dar + i,
                                    MEMTXATTRS_UNSPECIFIED, &byte, 1);
            }
        }
        if (cached_len >= count) {
            address_space_cache_invalidate(&cache, 0, count);
        }
        address_space_cache_destroy(&cache);
    } else if (dir == DMA_DIR_MEM_TO_DEV) {
        MemoryRegionCache cache;
        int64_t cached_len = address_space_cache_init(
            &cache, &address_space_memory, sar, count, false);

        for (i = 0; i < count; i++) {
            uint8_t byte = 0;
            uint32_t word;

            if (cached_len >= count) {
                byte = address_space_ldub_cached(&cache, i,
                                                 MEMTXATTRS_UNSPECIFIED, NULL);
            } else {
                address_space_read(&address_space_memory, sar + i,
                                   MEMTXATTRS_UNSPECIFIED, &byte, 1);
            }
            word = byte;
            address_space_write(&address_space_memory, dar,
                                MEMTXATTRS_UNSPECIFIED, &word, sizeof(word));
        }
        address_space_cache_destroy(&cache);
    }

}

static void axera_ax620u_clkrst_do_dma(AxeraAx620uClkrstState *s,
                                       uint32_t base, uint32_t dir,
                                       bool raise_irq)
{
    uint64_t sar = s->regs[(base + DMA_SAR_OFF) / 4];
    uint64_t dar = s->regs[(base + DMA_DAR_OFF) / 4];
    uint32_t count = s->regs[(base + DMA_LEN_OFF) / 4] + 1;
    uint32_t desc_ptr = s->regs[(base + DMA_DESC_OFF) / 4];

    if (desc_ptr != 0) {
        /* LLI-descriptor mode: +0x28 holds a pointer to a software
         * descriptor in guest RAM (sar_lo @ +0, sar_hi @ +4, dar_lo @
         * +8, dar_hi @ +0xc, len-1 @ +0x10, ...) that U-Boot's own
         * axi_dma_xfer_start() builds for every transfer, even a
         * single, unchained one -- the flat +0x00/+0x08/+0x10
         * registers are only what BL1's own, simpler driver uses
         * directly; U-Boot always goes through a descriptor instead,
         * so the real sar/dar/len live in RAM, not in those flat
         * registers (confirmed live: the flat DAR register still held
         * a stale, unrelated value from BL1's own earlier, unrelated
         * use of this same register while the descriptor's own dar_lo
         * held the real, correct destination).
         */
        /*
         * dw-axi-dmac LLI layout: sar u64 @0, dar u64 @8, block_ts
         * @0x10, llp u64 @0x18 (next descriptor, low 6 bits reserved),
         * ctl_lo @0x20, ctl_hi @0x24 (bit 30 = LLI_LAST, bit 31 =
         * LLI_VALID). The Linux driver splits a transfer over a chain
         * of these (a 2112-byte SPI-NAND page program arrives as four
         * 528-byte blocks); U-Boot's descriptors are single blocks
         * whose llp is 0. Walk the whole chain.
         */
        uint32_t desc[10];
        int n;

        s->dma_desc_dirty = false;
        for (n = 0; n < 256 && desc_ptr; n++) {
            address_space_read(&address_space_memory, desc_ptr,
                               MEMTXATTRS_UNSPECIFIED, desc, sizeof(desc));
            if (getenv("AXERA_DMAC_DEBUG")) {
                qemu_log("ax620u-dma: desc %08x dir %u sar %08x dar %08x ts %u llp %08x ctl %08x:%08x\n",
                         desc_ptr, dir, desc[0], desc[2], desc[4], desc[6],
                         desc[8], desc[9]);
            }
            /*
             * block_ts counts items of the SOURCE width (ctl_lo[10:8],
             * log2 bytes): the kernel's page program reads memory 32
             * bits at a time (528 items = 2112 bytes) into the 8-bit
             * DR FIFO, so the byte count is (ts+1) << src_width.
             */
            axera_ax620u_clkrst_do_block(desc[0], desc[2],
                                         (desc[4] + 1) << ((desc[8] >> 8) & 7),
                                         dir);
            if ((desc[9] & BIT(30)) || desc[6] == 0 ||
                (desc[6] & ~0x3fU) == desc_ptr) {
                break;
            }
            desc_ptr = desc[6] & ~0x3fU;
        }
    } else {
        s->dma_desc_dirty = false;
        axera_ax620u_clkrst_do_block(sar, dar, count, dir);
    }

    if (raise_irq) {
        /*
         * Without this, "ch0: start 1 times, completed 0 times"
         * forever and "SPI transfer timed out" is the observed,
         * confirmed-live result even once the transfer above is
         * byte-correct: the real kernel's dw-axi-dmac interrupt
         * handler only ever learns of completion through this line
         * (GIC SPI 5).
         *
         * Confirmed live via disassembly that the ISR *reads*
         * DMAC_CHEN_OFF to check whether the channel is idle yet (it
         * shifts the read value right by the channel index and tests
         * bit 0 -- this register's per-channel enable bit doubles as
         * a live busy/idle flag, not just a write-only trigger).
         * Clear it here, before the ISR ever gets to read it, so it
         * sees "idle" immediately: leaving it set (tried first) made
         * the ISR take its own warn-then-retry path instead ("BUG:
         * ... but channel not idle!", cosmetic by itself), which
         * *doesn't* run the real completion callback the higher-level
         * squashfs/MTD read is actually waiting on -- confirmed live,
         * that read kept retrying the exact same descriptor from
         * scratch (same desc_ptr/sar/dar/count observed four times
         * running) because it never saw a real completion, even
         * though the byte-correct copy below ran every time. The
         * read handler above is what lowers the IRQ now (independent
         * of this register's value), which is what makes clearing it
         * here safe -- an earlier attempt that cleared it without
         * that independent lowering path caused an infinite interrupt
         * storm instead (see kaeru memory for that dead end).
         */
        qemu_irq_raise(s->irq);
        s->regs[DMAC_CHEN_OFF / 4] &= ~DMAC_CHEN_CH0_EN;
    }
}

static void axera_ax620u_clkrst_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    AxeraAx620uClkrstState *s = opaque;
    uint32_t sub_off = offset % INSTANCE_STRIDE;

    s->regs[offset / 4] = value;

    if (sub_off == START_OFF) {
        s->regs[offset / 4] &= ~START_BIT;
    } else if (offset == DMAC_CHEN_OFF && !(value & DMAC_CHEN_CH0_EN)) {
        /*
         * Disable write -- the guest always performs this shortly
         * after the enable write below, whether or not the transfer
         * it requested actually succeeded (confirmed live). Used as
         * the point to lower the completion IRQ raised there, since
         * there's no evidence of a dedicated ack/clear register for
         * it; qemu_irq_lower() on an already-lowered line is a no-op.
         */
        qemu_irq_lower(s->irq);
    } else if (offset == DMAC_CHEN_OFF && (value & DMAC_CHEN_CH0_EN)) {
        /*
         * Real dw-axi-dmac (per ax620u.dts's "dw_dmac@1920000", 2
         * channels) channel-enable register, common block, confirmed
         * live via PC-gated write trace: the kernel's own SPI-NAND
         * dmaengine driver writes instance 1's SAR/DAR/LEN (already
         * stale/unused by this point)/CFG(+0x20/+0x24)/LLP(+0x28/+0x2c)
         * registers, in that order, then writes *this* register last
         * to actually kick off channel 0 -- the opposite order from
         * BL1/U-Boot's own direction-word-last convention (the DMA_DIR_OFF
         * trigger below), which fires too early for the kernel's own
         * sequence: at the moment CFG_HI(+0x24) is written, LLP(+0x28)
         * still holds the previous transfer's stale descriptor pointer
         * (confirmed live: produced a bogus desc_ptr=(stale)
         * sar=0x0 dar=0x0 count=1 no-op transfer well before the real
         * LLP pointer was written a few instructions later).
         *
         * BL1/U-Boot *do* also write this same enable bit (confirmed
         * live, every transfer: val=0x301 right alongside their own
         * already-correct DMA_DIR_OFF-triggered one) -- always
         * re-describing the exact same, already-completed transfer.
         * Comparing the resolved sar/dar/count/desc_ptr against the
         * last-fired transfer to detect this was tried first and
         * broke U-Boot's own environment load: it scans several NAND
         * pages' OOB area into the *same* small fixed scratch buffer
         * one DMA at a time, each a legitimately new transfer that
         * happens to share identical sar/dar/count/desc_ptr with the
         * one before it, which that approach wrongly skipped. What
         * actually distinguishes BL1/U-Boot's redundant enable-write
         * from the kernel's real one is the *descriptor pointer
         * register being freshly written since the last fire*, not
         * any particular value in it -- the kernel writes LLP(+0x28)
         * after DMA_DIR_OFF, so it's still dirty here; BL1/U-Boot
         * write it (if at all) before DMA_DIR_OFF, which already
         * consumed and cleared it below.
         */
        if (s->dma_desc_dirty) {
            uint32_t dir_word = s->regs[(0x100 + DMA_DIR_OFF) / 4];

            if (dir_word & DMA_DIR_DEV_TO_MEM) {
                axera_ax620u_clkrst_do_dma(s, 0x100, DMA_DIR_DEV_TO_MEM, true);
            } else if (dir_word & DMA_DIR_MEM_TO_DEV) {
                axera_ax620u_clkrst_do_dma(s, 0x100, DMA_DIR_MEM_TO_DEV, true);
            }
        } else if (s->dma_done_unsignalled) {
            /*
             * The kernel wrote this transfer's descriptor pointer before
             * its direction word, so the transfer already ran from that
             * trigger, without the completion IRQ the kernel's
             * dw-axi-dmac waits on ("SPI transfer timed out" -> squashfs
             * read errors -> App abort, intermittently). Signal it now.
             */
            qemu_irq_raise(s->irq);
            s->regs[DMAC_CHEN_OFF / 4] &= ~DMAC_CHEN_CH0_EN;
        }
        s->dma_done_unsignalled = false;
    } else if (sub_off == DMA_DESC_OFF) {
        s->dma_desc_dirty = true;
        s->dma_done_unsignalled = false;
    } else if (sub_off == DMA_DIR_OFF &&
               (value & (DMA_DIR_DEV_TO_MEM | DMA_DIR_MEM_TO_DEV))) {
        /*
         * BL1 and U-Boot's own axi_dma_xfer_start() always write
         * exactly 1 or 2 here, but the real Linux kernel's own SPI/DMA
         * driver packs the direction into just bit 1 of a much larger
         * control word (confirmed live: 0x7f808012, which silently
         * never triggered a transfer at all against the old exact-
         * value check, hanging the kernel's own SPI-NAND driver in
         * "polling_xfer_completion: timeout" forever) -- check the
         * direction bit itself, not the whole word, so both shapes
         * trigger the same way. No IRQ here: this is also BL1/U-Boot's
         * own trigger, and they never expect or handle one.
         *
         * Only actually fire when desc_ptr is 0 (BL1's own flat-
         * register mode, which never touches DMA_DESC_OFF and so
         * never sets dma_desc_dirty -- this trigger is the *only* one
         * that ever runs its transfer, so it must stay unconditional)
         * or when dma_desc_dirty is set (U-Boot's own LLI mode, which
         * -- unlike the kernel's -- always writes a fresh descriptor
         * pointer *before* this trigger, so dirty is reliably true
         * here for a real new transfer). Confirmed live this was
         * missing: once the kernel's own retry leaves a stale-but-
         * still-*valid* descriptor pointer in this instance's
         * DMA_DESC_OFF register from the previous successful transfer
         * (never reset to 0, unlike BL1's leftover pointers earlier
         * in boot), every later retry's own DMA_DIR_OFF write -- which
         * the kernel always performs *before* writing its own fresh
         * descriptor pointer for that retry, see DMAC_CHEN_OFF above
         * -- silently resolved that stale-but-valid descriptor and
         * re-ran the transfer a second time, draining the first half
         * of the RX queue the real SPI-NAND read had just freshly
         * filled before the real, traced completion (DMAC_CHEN_OFF)
         * ever got to read it, leaving it only the second
         * (OOB-padding, all-0xff) half.
         */
        uint32_t base = offset - sub_off;
        uint32_t desc_ptr_reg = s->regs[(base + DMA_DESC_OFF) / 4];

        if (desc_ptr_reg == 0 || s->dma_desc_dirty) {
            uint32_t this_dir = (value & DMA_DIR_DEV_TO_MEM) ?
                                 DMA_DIR_DEV_TO_MEM : DMA_DIR_MEM_TO_DEV;

            axera_ax620u_clkrst_do_dma(s, base, this_dir, false);
            s->dma_done_unsignalled = true;
        }
    }
}

static const MemoryRegionOps axera_ax620u_clkrst_ops = {
    .read = axera_ax620u_clkrst_read,
    .write = axera_ax620u_clkrst_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_ax620u_clkrst_realize(DeviceState *dev, Error **errp)
{
    AxeraAx620uClkrstState *s = AXERA_AX620U_CLKRST(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_ax620u_clkrst_ops, s,
                          TYPE_AXERA_AX620U_CLKRST, CLKRST_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_ax620u_clkrst_class_init(ObjectClass *klass,
                                           const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_ax620u_clkrst_realize;
}

static const TypeInfo axera_ax620u_clkrst_info = {
    .name = TYPE_AXERA_AX620U_CLKRST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraAx620uClkrstState),
    .class_init = axera_ax620u_clkrst_class_init,
};

static void axera_ax620u_clkrst_register_types(void)
{
    type_register_static(&axera_ax620u_clkrst_info);
}

type_init(axera_ax620u_clkrst_register_types)
