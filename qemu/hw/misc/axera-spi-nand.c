/*
 * Axera BL1's boot SPI NAND flash, modelled behind the same
 * DesignWare APB SSI controller shape as hw/misc/axera-spi-nor.c (see
 * that file's own comment for the controller-register side of this,
 * shared verbatim here: SSIENR 0->1 transitions, CTRL0's TMOD field,
 * chip-select handled off-controller via a GPIO BL1 never asks this
 * device about), but implementing the standard SPI NAND command set
 * (JEDEC-ish, not Axera-specific -- the same opcodes practically every
 * SPI NAND part uses) instead of NOR's flat linear read.
 *
 * Confirmed from the real vendor BL1 source
 * (maix_ax620e_sdk/boot/bl1/driver/spi/spinand.c,
 * driver/include/spinand.h) for the AX620E/AX630C/AX620Q family, which
 * shares this exact protocol with AX620U per the real AX620U
 * devicetree extracted from a deployed camera firmware dump
 * (openipc-linux-ax620u's ax620u.dts: spi-master@0x08000000,
 * compatible = "snps,dw-ssi", a child spi_nand@0 node) -- no AX620U
 * BL1 *source* exists in this vendor SDK at all (a different, distinct
 * SoC, "axera,ax620x" not "axera,ax620e"), so this model's grounding
 * for AX620U specifically is the real devicetree's controller identity
 * plus the real NAND part number a saved U-Boot env string names
 * (mx35lf1ge4ab, a Macronix MX35LF1GE4AB -- used for a plausible JEDEC
 * ID below), not a byte-traced real boot the way axera-spi-nor.c's
 * AX620E support was.
 *
 * Real read sequence (drivers/spi/spinand.c's spinand_read_page()):
 * PAGE_READ (0x13, 3-byte row number -- already divided out of the
 * logical address by the caller, not a byte address) loads a whole
 * 2048-byte page into the chip's internal cache and sets the busy bit
 * while it does; BL1 polls GET_FEATURE(REG_STATUS=0xc0) until busy
 * clears (spinand_wait()); READ_FROM_CACHE (0x03, 2-byte column, 1
 * dummy byte) then shifts the requested bytes out of that cache. This
 * model never goes busy (no real program/erase timing worth
 * modelling for a read-only boot path) and serves real file content
 * for the data phase, backed by a "file" property the same way
 * axera-spi-nor.c is.
 *
 * Confirmed live (debug tracing of every DR/CTRL0/CTRL1/SSIENR write,
 * across two rounds -- the first fix attempt here was itself wrong
 * and had to be reverted, see below): BL1's own controller driver
 * writes a dummy byte (0xff) to DR right after GET_FEATURE's two
 * command bytes latch, apparently to clock the 1-byte status response
 * out over MISO (real DW_SSI full-duplex (TMOD=0) hardware needs a TX
 * write to produce each RX clock) -- but BL1 never actually issues a
 * *separate* DR read to collect that response at all, confirmed live.
 *
 * The real bug was simpler than either of two wrong fixes tried
 * first (both "special-case a DR write while cmd_len==0 and
 * rxq_len>0": one discarding it outright, the other draining one
 * queued byte) -- it's that axera_spi_nand_latch() unconditionally
 * called rxq_reset() for *every* opcode, including the no-data ones.
 * The dummy 0xff byte matches OP_RESET's own declared length (1) and
 * genuinely latches as that opcode, which is harmless in itself (it
 * pushes nothing) -- but rxq_reset() still ran first and wiped the
 * status byte GET_FEATURE had just queued, before BL1's own
 * subsequent dummy-write-is-how-I-read-it mechanism ever consulted
 * it. Discarding the dummy write (first fix attempt) avoided that
 * wipe but also permanently blocked every later command's own first
 * byte from ever being accumulated into cmd_buf (since cmd_len==0 and
 * rxq_len>0 stayed true forever once nothing ever drains the queue
 * via a real DR read) -- PAGE_READ and READ_FROM_CACHE's own opcodes
 * got silently eaten this way for every NAND operation after the
 * first, permanently wedging the loaded row/column and feeding every
 * later "read the bad-block byte" request stale data from the wrong
 * page (symptom: every single eraseblock reported bad). Draining one
 * byte instead of discarding (second fix attempt) fixed that, but
 * broke the *first* GET_FEATURE call of all (at NAND probe time,
 * before any PAGE_READ/READ_FROM_CACHE), since BL1 apparently expects
 * to see that first status byte via RXFLR/DR *without* writing a
 * dummy byte at all in that specific spot -- draining it left RXFLR
 * permanently at 0, hanging before the bad-block scan even started.
 *
 * Real fix: move rxq_reset() out of the unconditional top of
 * axera_spi_nand_latch() and into only the opcodes that actually push
 * data (GET_FEATURE, READID, READ_FROM_CACHE). OP_RESET's own
 * spurious latch is then a true no-op, so whichever of the two
 * mechanisms BL1 actually uses to consume a queued response -- a
 * dummy DR write, a real DR read, or simply never reading it at all
 * before moving on -- the queue is never disturbed by anything else
 * in between, and no special-casing of DR writes is needed at all.
 *
 * Fixing the above was enough to get BL1 itself all the way through
 * the NAND bad-block-table scan and into U-Boot -- but READ_FROM_CACHE
 * returning real byte data was *also* gated on a second, unrelated bug
 * one layer down, in the AX620U clkrst block's own per-instance DMA
 * trigger (see hw/misc/axera-ax620u-clkrst.c's own comment): BL1's
 * SPI-NAND driver moves this opcode's response bytes via a real
 * memory-mapped DMA-ish transfer rather than a per-byte DR read once a
 * transfer is long enough, and without that second fix the
 * destination buffer BL1's own bad-block check inspected was never
 * written at all, regardless of this device's own DR/rxq content.
 *
 * Once U-Boot itself took over (same controller, its own
 * drivers/spi/axera_spi.c driver, confirmed from this project's
 * maix_ax620e_sdk U-Boot-2020.04 source), a *third*, again unrelated
 * bug showed up: U-Boot's driver legitimately disables/reconfigures/
 * re-enables SSIENR *in the middle* of a single logical SPI-NAND
 * command -- e.g. between READID's own opcode byte and its trailing
 * dummy byte, to retune CTRL0/CTRL1 for the read phase without ever
 * touching chip-select (handled off-controller via GPIO, invisible to
 * this model either way). This model used to reset cmd_len on every
 * SSIENR 0->1 transition; against U-Boot's sequence that wiped the
 * already-latched opcode byte before its own trailing byte ever
 * arrived, so that trailing byte (0xff) got misread as a brand new,
 * bogus OP_RESET command, silently discarding the real READID and
 * leaving RXFLR at 0 forever -- U-Boot's own poll_transfer() has no
 * timeout on an RX FIFO that never fills, so this hung solid at the
 * "NAND:" banner. BL1's own minimal driver never happens to split a
 * command across an SSIENR toggle like this, so dropping that reset
 * entirely (cmd_len already resets to 0 on every successful latch, so
 * there was never any stale state the reset was actually needed to
 * clear) fixes U-Boot without affecting BL1. READID's own response
 * was also widened from 2 bytes to the full SPINAND_MAX_ID_LEN=4
 * U-Boot's spinand_read_id_op() always clocks out (confirmed from the
 * same U-Boot source), since only the first two bytes are ever matched
 * against a manufacturer table but RXFLR still needs to satisfy all 4.
 *
 * Past "NAND:", two more bugs surfaced before U-Boot's own NAND stack
 * worked correctly: first, drivers/mtd/nand/spi/macronix.c's own
 * detect() (confirmed from the same U-Boot source) reads manufacturer
 * ID from id[1] and device ID from id[2], not id[0]/id[1] like every
 * other supported manufacturer, with a comment explaining Macronix
 * parts need a genuine extra dummy byte before the real ID -- so this
 * model's READID response leads with a real dummy byte now
 * ({0x00,0xc2,0x12,0x00}), confirmed live via U-Boot logging "unknown
 * raw ID" against the un-shifted version beforehand. Second, and far
 * more consequential: axera_spi.c's own read_cache_variants table
 * (drivers/mtd/nand/spi/macronix.c) tries Quad Output Fast Read (0x6b)
 * before plain READ_FROM_CACHE (0x03), and nothing in this model (or,
 * apparently, the real controller driver's own capability checks)
 * ever rejects it -- so U-Boot settles on 0x6b for every single
 * cache-read from then on. This model didn't recognise that opcode at
 * all (falling through cmd_len()'s default case, length 1), so the
 * opcode byte alone immediately "completed" as a bogus zero-effect
 * command, and every trailing byte (column, dummy) was then misread
 * as its own fresh one-byte command too -- never actually reaching
 * the real READ_FROM_CACHE logic, so every cache-read after this point
 * returned nothing but stale/empty rxq content. Confirmed live via
 * gdb/trace: this is what made U-Boot's "is this block bad" checks
 * see garbage instead of a real OOB marker, which is also the real
 * explanation for the symptom that first exposed it -- several blocks
 * BL1 never had any trouble with came back "bad" under U-Boot, and a
 * later NAND read spanning enough blocks to hit this always ended
 * "Attempt to read outside the flash area" once too many real blocks
 * were wrongly treated as bad to fit the requested length before
 * running off the end of the device. SPINAND_PAGE_READ_FROM_CACHE_X4_OP
 * in include/linux/mtd/spinand.h (same U-Boot source) confirms 0x6b
 * needs the exact same column+dummy-byte shape as 0x03 -- only the
 * data phase's I/O width differs, which this byte-level model never
 * represented anyway -- so it's handled as a second case alongside
 * OP_READ_FROM_CACHE everywhere, not a separate code path.
 *
 * Boot-only (BL1/U-Boot) traffic never writes to the flash, so none of
 * the above needed a write path at all. Once the real kernel's own
 * UBIFS reaches real userspace (confirmed live: "UBIFS: recovery
 * needed"/"recovery completed" -- a journal replay that writes -- and
 * ordinary daemons like axklogd updating files under the mounted "mtd"
 * UBI volume), it genuinely issues PROGRAM_LOAD (0x02/0x32, cmd + 2-
 * byte column, then an open-ended run of page-cache data bytes with no
 * length known in advance) and PROGRAM_EXECUTE (0x10, cmd + 3-byte row,
 * commits the staged cache to the array) / BLOCK_ERASE (0xd8, cmd +
 * 3-byte row identifying the block). None of these were recognised at
 * all (falling through cmd_len()'s default case, length 1) -- so
 * PROGRAM_LOAD's own opcode byte "completed" as a bogus zero-effect
 * command the instant it landed, and every one of its column and page-
 * data bytes afterward was independently misread as its own fresh
 * one-byte command too, with whichever of them happened to equal an
 * already-recognised opcode (0x13 PAGE_READ was the one actually hit)
 * latching as if it were real, using more of that same data as fake
 * command arguments. Confirmed live via a guest-PC-gated trace on
 * exactly this: real page-data bytes decoding into wildly implausible
 * "row" numbers (0xdc0000, 0xe1a000, ...) -- each one well beyond this
 * chip's real ~0x10000-row range, yet dividing evenly by the real
 * 2048-byte page size into entirely ordinary small numbers, which is
 * exactly what "a byte-offset deep inside a data buffer, misread as if
 * it were itself a page-row argument" looks like, not a kernel-side
 * bug (the real geometry table this kernel's own macronix.c driver
 * selects for this exact device ID was independently confirmed correct
 * via disassembly). The real DW_SPI_DR write path has no way to tell
 * "this byte is new command opcode" from "this byte is page-cache data
 * belonging to an open PROGRAM_LOAD" without tracking where one series
 * of writes ends -- hence program_active below, cleared on the real
 * controller's own SSIENR falling edge exactly the way
 * axera-spi-nor.c's own Page Program already does for the same reason
 * (see its own comment), the one real hardware/software boundary this
 * model has access to that doesn't depend on knowing a length up
 * front. Data integrity of what's staged/committed this way doesn't
 * need to be perfect (nothing downstream of a successful boot
 * currently re-reads and verifies arbitrary written content byte for
 * byte), only that it no longer corrupts this device's own command
 * parser.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/core/cpu.h"

#define TYPE_AXERA_SPI_NAND "axera-spi-nand"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraSpiNandState, AXERA_SPI_NAND)

#define DW_SPI_CTRL0            0x00
#define DW_SPI_CTRL1            0x04
#define DW_SPI_SSIENR           0x08
#define DW_SPI_SER              0x10
#define DW_SPI_TXFLR            0x20
#define DW_SPI_RXFLR            0x24
#define DW_SPI_SR               0x28
#define DW_SPI_DR               0x60
#define DW_SPI_SIZE             0x100

#define SR_TF_NOT_FULL          BIT(1)
#define SR_TF_EMPT              BIT(2)
#define SR_RF_NOT_EMPT          BIT(3)

#define SPINAND_PAGE_SIZE       2048
#define SPINAND_OOB_SIZE        64
#define SPINAND_CACHE_SIZE      (SPINAND_PAGE_SIZE + SPINAND_OOB_SIZE)
#define SPINAND_PAGES_PER_BLOCK 64
#define SPINAND_BLOCK_SIZE      (SPINAND_PAGE_SIZE * SPINAND_PAGES_PER_BLOCK)

#define OP_RESET                0xff
#define OP_WREN                 0x06
#define OP_WRDI                 0x04
#define OP_READID               0x9f
#define OP_GET_FEATURE          0x0f
#define OP_SET_FEATURE          0x1f
#define OP_PAGE_READ            0x13
#define OP_READ_FROM_CACHE      0x03
#define OP_READ_FROM_CACHE_X4   0x6b
#define OP_PROGRAM_LOAD         0x02
#define OP_PROGRAM_LOAD_X4      0x32
#define OP_PROGRAM_LOAD_RDM     0x84
#define OP_PROGRAM_LOAD_X4_RDM  0x34
#define OP_PROGRAM_EXECUTE      0x10
#define OP_BLOCK_ERASE          0xd8

#define REG_STATUS              0xc0
#define STATUS_OIP              BIT(0)
#define STATUS_WEL              BIT(1)

#define AXERA_SPI_NAND_RXQ_MAX  4096

struct AxeraSpiNandState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    char *file;
    uint8_t *flash;
    gsize flash_size;

    uint32_t mem[DW_SPI_SIZE / 4];
    uint8_t cmd_buf[8];
    unsigned int cmd_len;
    int pending_set_feature_addr;      /* -1 when no SET_FEATURE pending */
    /*
     * Write Enable Latch: set by WRITE ENABLE (0x06), cleared by WRITE
     * DISABLE (0x04) and by every PROGRAM EXECUTE / BLOCK ERASE, which
     * real parts ignore while it is clear. Both U-Boot's and Linux's
     * spinand core issue WRITE ENABLE before every program/erase.
     * Reported as status-register bit 1 like on real parts. RESET is
     * deliberately not modelled as clearing it: controller dummy 0xff
     * bytes already latch here as OP_RESET (see the top comment).
     */
    bool write_enable_latch;
    uint8_t feature_regs[256];
    uint32_t loaded_row;

    /*
     * True from a latched PROGRAM_LOAD(_X4)(_RDM)'s own opcode+2-byte
     * column through the real controller's next SSIENR falling edge --
     * see this file's own top comment for why. While true, DW_SPI_DR
     * writes are page-cache data (program_cache[program_column+i]),
     * never command bytes.
     */
    bool program_active;
    bool ssienr;
    uint32_t program_column;
    uint32_t program_idx;
    bool debug;
    bool ser_seen;      /* guest drives chip select through SER */
    uint8_t program_cache[SPINAND_CACHE_SIZE];

    uint8_t rxq[AXERA_SPI_NAND_RXQ_MAX];
    unsigned int rxq_head;
    unsigned int rxq_len;
};

static void rxq_reset(AxeraSpiNandState *s)
{
    s->rxq_head = 0;
    s->rxq_len = 0;
}

static void rxq_push(AxeraSpiNandState *s, uint8_t byte)
{
    if (s->rxq_len < AXERA_SPI_NAND_RXQ_MAX) {
        s->rxq[(s->rxq_head + s->rxq_len) % AXERA_SPI_NAND_RXQ_MAX] = byte;
        s->rxq_len++;
    }
}

/*
 * How many total cmd+addr(+dummy) bytes each supported opcode needs
 * before it either completes (no-data ops) or hands off to a data
 * phase (handled by the caller once this many bytes have arrived).
 */
static int axera_spi_nand_cmd_len(uint8_t opcode)
{
    switch (opcode) {
    case OP_RESET:
    case OP_WREN:
    case OP_WRDI:
        return 1;
    case OP_READID:            /* cmd + 1 dummy byte */
    case OP_GET_FEATURE:       /* cmd + 1 reg addr byte */
    case OP_SET_FEATURE:       /* cmd + 1 reg addr byte */
        return 2;
    case OP_PAGE_READ:             /* cmd + 3-byte row */
    case OP_READ_FROM_CACHE:       /* cmd + 2-byte column + 1 dummy byte */
    case OP_READ_FROM_CACHE_X4:    /* same shape, x4 I/O lines for data */
    case OP_PROGRAM_EXECUTE:       /* cmd + 3-byte row */
    case OP_BLOCK_ERASE:           /* cmd + 3-byte row (any row in the block) */
        return 4;
    case OP_PROGRAM_LOAD:          /* cmd + 2-byte column, then open-ended
                                     * page-cache data -- see program_active */
    case OP_PROGRAM_LOAD_X4:
    case OP_PROGRAM_LOAD_RDM:
    case OP_PROGRAM_LOAD_X4_RDM:
        return 3;
    default:
        return 1;
    }
}

static void axera_spi_nand_latch(AxeraSpiNandState *s)
{
    uint8_t opcode = s->cmd_buf[0];
    uint32_t column, i;

    switch (opcode) {
    case OP_WREN:
        s->write_enable_latch = true;
        break;
    case OP_WRDI:
        s->write_enable_latch = false;
        break;
    case OP_PAGE_READ:
        s->loaded_row = (s->cmd_buf[1] << 16) | (s->cmd_buf[2] << 8) |
                        s->cmd_buf[3];
        break;
    case OP_PROGRAM_LOAD:
    case OP_PROGRAM_LOAD_X4:
        /*
         * Real parts clear the whole cache to 0xff first for these two
         * (unlike their _RDM siblings, which only overwrite the given
         * column range) -- harmless to do unconditionally here since
         * nothing downstream needs byte-perfect partial-cache fidelity,
         * only that this device's own command parser stops treating
         * open-ended page-cache data as a stream of new commands (see
         * this file's own top comment).
         */
        memset(s->program_cache, 0xff, sizeof(s->program_cache));
        /* fallthrough */
    case OP_PROGRAM_LOAD_RDM:
    case OP_PROGRAM_LOAD_X4_RDM:
        s->program_column = (s->cmd_buf[1] << 8) | s->cmd_buf[2];
        s->program_idx = 0;
        s->program_active = true;
        break;
    case OP_PROGRAM_EXECUTE:
        {
            uint32_t row = (s->cmd_buf[1] << 16) | (s->cmd_buf[2] << 8) |
                           s->cmd_buf[3];
            uint64_t off = (uint64_t)row * SPINAND_PAGE_SIZE;
            /*
             * Only the 2048 data bytes land in the flat image: it has no
             * room for OOB, and copying the whole 2112-byte cache would
             * spill the 64 OOB bytes over the start of the next page (and
             * over the next block's UBI EC header for the last page of a
             * block). Programming can only clear bits, as on real NAND.
             */
            uint32_t len = s->flash_size > off ?
                           (uint32_t)MIN((uint64_t)SPINAND_PAGE_SIZE,
                                         s->flash_size - off) : 0;

            if (!s->write_enable_latch) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "axera-spi-nand: PROGRAM EXECUTE row 0x%x "
                              "without WRITE ENABLE, ignored\n", row);
                len = 0;
            }
            s->write_enable_latch = false;
            for (i = 0; i < len; i++) {
                s->flash[off + i] &= s->program_cache[i];
            }
            if (s->debug) {
                qemu_log("spinand: PROG_EXEC row 0x%x col %u n %u\n", row,
                         s->program_column, s->program_idx);
            }
        }
        break;
    case OP_BLOCK_ERASE:
        {
            uint32_t row = (s->cmd_buf[1] << 16) | (s->cmd_buf[2] << 8) |
                           s->cmd_buf[3];
            uint64_t block_off = (uint64_t)(row / SPINAND_PAGES_PER_BLOCK) *
                                 SPINAND_BLOCK_SIZE;
            uint32_t len = s->flash_size > block_off ?
                           (uint32_t)MIN((uint64_t)SPINAND_BLOCK_SIZE,
                                         s->flash_size - block_off) : 0;

            if (!s->write_enable_latch) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "axera-spi-nand: BLOCK ERASE row 0x%x "
                              "without WRITE ENABLE, ignored\n", row);
                len = 0;
            }
            s->write_enable_latch = false;
            if (len) {
                memset(s->flash + block_off, 0xff, len);
            }
            if (s->debug) {
                qemu_log("spinand: ERASE row 0x%x\n", row);
            }
        }
        break;
    case OP_SET_FEATURE:
        s->pending_set_feature_addr = s->cmd_buf[1];
        break;
    case OP_GET_FEATURE:
        /*
         * Echo back whatever SET_FEATURE last wrote to this same
         * address (status/busy bit forced clear regardless, since
         * this model never goes busy) rather than a blanket 0x00:
         * U-Boot's own XmSpiNand driver (unlike BL1's minimal one)
         * writes a QE/ECC-enable bit via SET_FEATURE and then reads
         * it straight back via GET_FEATURE to confirm it took,
         * printing "Enable ECC Failed" when the readback didn't
         * reflect what it had just set.
         */
        rxq_reset(s);
        if (s->cmd_buf[1] == REG_STATUS) {
            rxq_push(s, (s->feature_regs[REG_STATUS] &
                         ~(STATUS_OIP | STATUS_WEL)) |
                        (s->write_enable_latch ? STATUS_WEL : 0));
        } else {
            rxq_push(s, s->feature_regs[s->cmd_buf[1]] & ~STATUS_OIP);
        }
        break;
    case OP_READID:
        {
            /* Macronix MX35LF1GE4AB (mx35lf1ge4ab, confirmed from a
             * real AX620U camera's own saved U-Boot env string) --
             * manufacturer 0xc2, a plausible device ID since BL1 only
             * logs this rather than gating boot on an exact match.
             * U-Boot's own real spinand core (drivers/mtd/nand/spi/
             * core.c's spinand_read_id_op(), confirmed from this same
             * vendor SDK's U-Boot source) always clocks out
             * SPINAND_MAX_ID_LEN=4 bytes regardless of how many a
             * given part's ID actually carries, and Macronix parts
             * specifically shift the real ID one byte later than
             * every other supported manufacturer: macronix.c's own
             * detect() reads manufacturer/device from id[1]/id[2], not
             * id[0]/id[1], with a comment explaining why verbatim --
             * "Macronix SPI NAND read ID needs a dummy byte, so the
             * first byte in raw_id is garbage." Leading with a real
             * dummy byte here (confirmed live: U-Boot logged "unknown
             * raw ID" against the un-shifted {0xc2,0x12,0x00,0x00}
             * before this fix) matches that exactly. */
            static const uint8_t id[] = { 0x00, 0xc2, 0x12, 0x00 };

            rxq_reset(s);
            for (i = 0; i < sizeof(id); i++) {
                rxq_push(s, id[i]);
            }
        }
        break;
    case OP_READ_FROM_CACHE:
    case OP_READ_FROM_CACHE_X4:
        rxq_reset(s);
        column = (s->cmd_buf[1] << 8) | s->cmd_buf[2];
        for (i = 0; i < 4096; i++) {
            uint32_t page_off = column + i;
            if (page_off >= SPINAND_PAGE_SIZE) {
                rxq_push(s, 0xff);     /* OOB: no bad-block marker */
                continue;
            }
            uint64_t off = (uint64_t)s->loaded_row * SPINAND_PAGE_SIZE +
                           page_off;
            rxq_push(s, off < s->flash_size ? s->flash[off] : 0xff);
        }
        break;
    default:
        break;
    }
}

static uint64_t axera_spi_nand_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    AxeraSpiNandState *s = opaque;
    uint8_t val;

    switch (offset) {
    case DW_SPI_TXFLR:
        return 0;
    case DW_SPI_RXFLR:
        return s->rxq_len;
    case DW_SPI_SR:
        return SR_TF_NOT_FULL | SR_TF_EMPT |
               (s->rxq_len ? SR_RF_NOT_EMPT : 0);
    case DW_SPI_DR:
        if (s->rxq_len == 0) {
            return 0;
        }
        val = s->rxq[s->rxq_head];
        s->rxq_head = (s->rxq_head + 1) % AXERA_SPI_NAND_RXQ_MAX;
        s->rxq_len--;
        return val;
    default:
        if (offset < DW_SPI_SIZE) {
            return s->mem[offset / 4];
        }
        return 0;
    }
}

static void axera_spi_nand_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    AxeraSpiNandState *s = opaque;

    if (s->debug && getenv("AXERA_NAND_DEBUG")[0] == '2') {
        qemu_log("spinand: w %02x=%08x size %u act %d\n", (unsigned)offset,
                 (uint32_t)value, size, s->program_active);
    }
    switch (offset) {
    case DW_SPI_TXFLR:
    case DW_SPI_RXFLR:
    case DW_SPI_SR:
        return;
    case DW_SPI_SSIENR:
        {
            bool enable = (value & 1) != 0;

            /*
             * Falling edge is the only general-purpose "this logical
             * transfer is over" signal this controller shape offers
             * (see this file's own top comment) -- PROGRAM_LOAD's own
             * data phase is open-ended, with no byte count known up
             * front to latch on instead. Gated on program_active so
             * this never affects any other, already-working command
             * shape (confirmed by this file's own earlier history:
             * resetting unconditionally on SSIENR transitions broke
             * U-Boot's own READID, which legitimately toggles SSIENR
             * mid-command).
             */
            if (!enable && s->ssienr && s->program_active && !s->ser_seen) {
                if (s->debug) {
                    qemu_log("spinand: LOAD end col %u n %u\n",
                             s->program_column, s->program_idx);
                }
                s->program_active = false;
                s->cmd_len = 0;
            }
            s->ssienr = enable;
        }
        s->mem[offset / 4] = value;
        return;
    case DW_SPI_SER:
        /*
         * Linux's spi-dw toggles SSIENR around every spi_transfer, so a
         * spi-mem PROGRAM_LOAD (cmd+column transfer, then the DMA data
         * transfer) spans an SSIENR falling edge. What brackets a whole
         * spi_message on this kernel is its SER (chip select) write: it
         * is written (always 1, never back to 0) once at the start of
         * every message, and never between that message's transfers.
         * Once the guest is seen doing that, SER alone ends an open
         * PROGRAM_LOAD -- the next message (PROGRAM_EXECUTE) begins.
         */
        s->ser_seen = true;
        if (s->program_active) {
            if (s->debug) {
                qemu_log("spinand: LOAD end(cs) col %u n %u\n",
                         s->program_column, s->program_idx);
            }
            s->program_active = false;
            s->cmd_len = 0;
        }
        s->mem[offset / 4] = value;
        return;
    case DW_SPI_DR:
        if (s->program_active) {
            uint32_t off = s->program_column + s->program_idx;

            if (off < sizeof(s->program_cache)) {
                s->program_cache[off] = value & 0xff;
            }
            s->program_idx++;
            return;
        }
        if (s->pending_set_feature_addr >= 0) {
            /* The feature value byte: store it so a later GET_FEATURE
             * to the same address reads back what was actually set. */
            s->feature_regs[s->pending_set_feature_addr] = value & 0xff;
            s->pending_set_feature_addr = -1;
            return;
        }
        if (s->cmd_len < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_len++] = value & 0xff;
        }
        if (s->cmd_len > 0 &&
            s->cmd_len == (unsigned)axera_spi_nand_cmd_len(s->cmd_buf[0])) {
            axera_spi_nand_latch(s);
            s->cmd_len = 0;
        }
        return;
    default:
        if (offset < DW_SPI_SIZE) {
            s->mem[offset / 4] = value;
        }
        return;
    }
}

static const MemoryRegionOps axera_spi_nand_ops = {
    .read = axera_spi_nand_read,
    .write = axera_spi_nand_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * The AX620U kernel's spi-dw-dma path feeds DR through the DW AXI
     * DMAC model, which moves one byte per access: accept byte/halfword
     * accesses (widened to one 32-bit DR access each) instead of
     * silently dropping them -- which lost every page-program payload.
     */
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_spi_nand_realize(DeviceState *dev, Error **errp)
{
    AxeraSpiNandState *s = AXERA_SPI_NAND(dev);

    s->pending_set_feature_addr = -1;
    s->debug = getenv("AXERA_NAND_DEBUG") && getenv("AXERA_NAND_DEBUG")[0];

    if (s->file && s->file[0]) {
        if (!g_file_get_contents(s->file, (gchar **)&s->flash,
                                 &s->flash_size, NULL)) {
            error_setg(errp, "axera-spi-nand: could not read file '%s'",
                      s->file);
            return;
        }
    } else {
        s->flash_size = 0;
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_spi_nand_ops, s,
                          TYPE_AXERA_SPI_NAND, DW_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

/*
 * A system reset (watchdog, reboot) must bring the chip back to its
 * power-on state: features set by Linux (OTP/ECC mode in 0xb0, block
 * lock) left in place made U-Boot read every block as bad
 * ("eba N is bad") after a reboot.
 */
static void axera_spi_nand_reset(DeviceState *dev)
{
    AxeraSpiNandState *s = AXERA_SPI_NAND(dev);

    memset(s->mem, 0, sizeof(s->mem));
    memset(s->cmd_buf, 0, sizeof(s->cmd_buf));
    memset(s->feature_regs, 0, sizeof(s->feature_regs));
    s->cmd_len = 0;
    s->pending_set_feature_addr = -1;
    s->write_enable_latch = false;
    s->loaded_row = 0;
    s->program_active = false;
    s->ssienr = false;
    s->program_column = 0;
    s->program_idx = 0;
    s->ser_seen = false;
    rxq_reset(s);
}

static const Property axera_spi_nand_properties[] = {
    DEFINE_PROP_STRING("file", AxeraSpiNandState, file),
};

static void axera_spi_nand_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_spi_nand_realize;
    device_class_set_props(dc, axera_spi_nand_properties);
    device_class_set_legacy_reset(dc, axera_spi_nand_reset);
}

static const TypeInfo axera_spi_nand_info = {
    .name = TYPE_AXERA_SPI_NAND,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraSpiNandState),
    .class_init = axera_spi_nand_class_init,
};

static void axera_spi_nand_register_types(void)
{
    type_register_static(&axera_spi_nand_info);
}

type_init(axera_spi_nand_register_types)
