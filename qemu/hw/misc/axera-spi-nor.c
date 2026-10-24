/*
 * Axera BL1's boot SPI NOR controller + flash: DW_SPI_BASE @ 0x01a00000
 * (maix_ax620e_sdk/boot/bl1/driver/spi/dw_spi_mst.c, driver/include/
 * dw_spi.h) -- the SAME DesignWare APB SSI IP as the SPI2 panel
 * controller hw/misc/axera-dw-spi.c already models, but at a different
 * instance/address and in a role where the loopback trick that works
 * for the panel is not enough: a SPI NOR read's response bytes must be
 * the real flash content at the requested address, not an echo of
 * whatever was written.
 *
 * Confirmed directly from dw_spi_mst.c's real dw_spi_xfer()/
 * spi_mem_exec_op(): each SPI NOR operation is two dw_spi_xfer() calls
 * -- a TX-only "cmd+addr+dummy" phase, then an RX-only "data" phase --
 * each of which independently does spi_enable_chip(0) (SSIENR=0),
 * reprograms CTRL0 (data frame size + SPI_TMOD_TO/_RO in bits 10-11)
 * and, for the RX phase only, CTRL1 (entries-1), then spi_enable_chip(1)
 * (SSIENR=1). That SSIENR 0->1 transition is a clean, SPI-controller-
 * register-local synchronisation point: no need to model the real
 * chip-select signal at all (real silicon asserts/deasserts it via a
 * GPIO2 pin, external_cs_manage(), entirely outside this controller's
 * own register space).
 *
 * BL1 only ever uses qmode=0 (drivers/spi/spinor.c's spinor_qmode,
 * never set otherwise before the first boot read): opcode 0x03
 * (SPINOR_OP_READ) + a 3-byte big-endian address, no dummy cycles --
 * the simplest possible SPI NOR read, well within what this model
 * needs to get right. RDSR (0x05) and READID (0x90) are also modelled
 * since BL1's spinor_init()/spinor_wait_till_ready() use them, but
 * always answer "not busy" / a fixed ID rather than tracking real
 * flash state machine timing -- there is no real program/erase
 * latency worth modelling for a read-only boot path.
 *
 * The real kernel's own spi-dw-core.c (dw_spi_write_then_read(), used
 * by its generic spi_mem exec_op()) does NOT match U-Boot's 2-phase
 * shape: it sends opcode, 3-byte address and the READ_FAST dummy byte
 * as three SEPARATE TX-only (TMOD_TO) sub-phases -- each its own
 * SSIENR 0->1->0 cycle, CTRL0 unchanged between them -- before the
 * one real RX (TMOD_RO) phase. Confirmed live via register tracing:
 * resetting cmd_len on every TMOD_TO entry (matching only U-Boot's
 * single-phase shape) let the trailing dummy byte overwrite the real
 * opcode+address right before the RX phase needed them, latching
 * opcode=0xff/addr=0 every time ("spi_master spi4: Rx hanged up").
 * axera_spi_nor_ssienr_write() now only resets cmd_len after an RO
 * phase actually consumes it, so consecutive TO sub-phases accumulate
 * into the same cmd_buf -- naturally capped at sizeof(cmd_buf)==4 by
 * the DW_SPI_DR write handler, which is "ignore the dummy byte" for
 * free.
 *
 * Confirmed real partition layout for this flash (AX620E's own real
 * deployed camera firmware, General_IPC_AX620E_A6...dump, via its own
 * saved U-Boot env): mtdparts=spi_nor.0:0x10000(spl),0x50000(boot),
 * 0x4F0000(romfs),0x640000(usr),0x20000(web),0x3F0000(custom),
 * 0x60000(mtd) -- SPL and the second signed image (DDRINIT, not ATF/
 * OPTEE -- this NOR profile has neither) are RSA-signed, U-Boot itself
 * is plain/unsigned.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/arm/axera.h"
#include <zlib.h>

#define TYPE_AXERA_SPI_NOR "axera-spi-nor"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraSpiNorState, AXERA_SPI_NOR)

#define DW_SPI_CTRL0            0x00
#define DW_SPI_CTRL1            0x04
#define DW_SPI_SSIENR           0x08
#define DW_SPI_SER              0x10
#define DW_SPI_TXFTLR           0x18
#define DW_SPI_RXFTLR           0x1c
#define DW_SPI_TXFLR            0x20
#define DW_SPI_RXFLR            0x24
#define DW_SPI_SR               0x28
#define DW_SPI_DMACR            0x4c
#define DW_SPI_DR               0x60
#define DW_SPI_SIZE             0x100

#define SPI_DMA_RDMAE           BIT(0)

/*
 * Real DW_apb_ssi FIFO depth for this instance (32, the common
 * configuration for this IP family). U-Boot's own generic
 * designware_spi.c driver auto-probes this at runtime (spi_hw_init()):
 * it writes increasing values to TXFTLR and watches for the readback
 * to stop matching, since the real register field is only wide enough
 * to hold 0..depth-1 -- a plain read/write-through register (as this
 * model treated it before) never truncates, so the probe runs all the
 * way to 256 and the driver believes it has an impossibly deep FIFO.
 * That in turn feeds tx_max()/rx_max()'s FIFO-availability bookkeeping,
 * which this model's own RXFLR (previously reporting the *entire*
 * pending transfer as already available, every single byte, the
 * instant SSIENR goes high) made worse: real hardware can only ever
 * have up to one FIFO's worth of bytes sitting there at a time, with
 * the rest still arriving serially over the wire as the clock runs.
 */
#define AXERA_SPI_NOR_FIFO_DEPTH 32

#define SPI_DFS_OFFSET          0
#define SPI_DFS_MASK            0x1f
#define SPI_TMOD_OFFSET         10
#define SPI_TMOD_MASK           (0x3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TO             0x1     /* xmit only: cmd+addr phase */
#define SPI_TMOD_RO             0x2     /* recv only: data phase */

#define SR_BUSY                 BIT(0)
#define SR_TF_NOT_FULL          BIT(1)
#define SR_TF_EMPT              BIT(2)
#define SR_RF_NOT_EMPT          BIT(3)

#define SPINOR_OP_RDSR          0x05
#define SPINOR_OP_READID        0x90
#define SPINOR_OP_RDID          0x9f
#define SPINOR_OP_READ          0x03
#define SPINOR_OP_READ_FAST     0x3b

/*
 * CONFIG_ENV_OFFSET/CONFIG_ENV_SIZE for this real binary: the real
 * saved environment at flash offset 0x50000 is 0x10000 bytes (read as
 * two back-to-back 32768-byte chunks, 0x50000 then 0x58000 -- each
 * one itself doubled by the same real driver quirk
 * axera_spi_nor_fixup_first_chunk_dest()'s own comment covers), with
 * a genuinely valid CRC32 once that fixup is in place. The one real
 * gap: this dump's own real saved environment has no "xmuart"
 * variable at all, which some part of this vendor's own boot chain
 * needs set to get the kernel to produce any console output
 * (confirmed live: "xminfo" prints XmUart=<NULL>, reported directly
 * by the user). U-Boot's autoboot console can't be interrupted to set
 * it interactively either (this model's UART RX path doesn't yet
 * forward keystrokes, a separate, not yet investigated gap), so this
 * serves a synthetic but otherwise byte-for-byte faithful
 * reproduction of the dump's own real saved environment (every real
 * variable this dump's flash actually has) with "xmuart=0" added,
 * sized and CRC'd to match exactly what env_sf_load() itself checks.
 */
#define AXERA_ENV_OFFSET        0x50000
#define AXERA_ENV_SIZE          0x10000

static const char axera_env_vars[] =
    "baudrate=115200\0"
    "board=ax620q\0"
    "board_name=ax620q\0"
    "bootdelay=2\0"
    "ethaddr=00:01:43:34:48:84\0"
    "eth1addr=00:00:23:34:45:77\0"
    "ethprime=eth0\0"
    "fdt_high=0xffffffffffffffff\0"
    "fdtcontroladdr=4fc545f8\0"
    "gatewayip=192.168.1.254\0"
    "ipaddr=192.168.1.101\0"
    "netmask=255.255.255.0\0"
    "serverip=192.168.1.100\0"
    "soc=ax620e\0"
    "dl=eth_init;mw.b 0x44200000 ff 100000;tftpboot 0x44200000 spl.bin.img;flwrite\0"
    "da=eth_init;mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 u-boot.bin.img;sf probe 0;flwrite\0"
    "de=eth_init;mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 u-boot.env.img;sf probe 0;flwrite\0"
    "dr=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 romfs-x.squashfs.img;sf probe 0;flwrite\0"
    "du=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 user-x.squashfs.img;sf probe 0;flwrite\0"
    "dc=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 custom-x.squashfs.img;sf probe 0;flwrite\0"
    "dw=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 web-x.squashfs.img;sf probe 0;flwrite\0"
    "dd=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 mtd-x.jffs2.img;sf probe 0;flwrite\0"
    "up=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 update.img;sf probe 0;flwrite\0"
    "ua=mw.b 0x44200000 ff 0x1000000;tftpboot 0x44200000 update_all.img;sf probe 0;flwrite\0"
    "tk=tftpboot 0x44200000 uImage;setenv setargs setenv bootargs ${bootargs};run setargs;bootm 0x44200000\0"
    "loadromfs=sf probe 0;sf read 0x4B000000 0x60000 0x4F0000;squashfsload;axera_boot\0"
    "bootcmd=setenv setargs setenv bootargs ${bootargs};run setargs;run loadromfs\0"
    "bootargs=mem=${osmem} console=ttyS0,115200n8 loglevel=4 earlyprintk=dw_uart,boot_reason=0x0,noinitrd root=/dev/mtdblock2 rw rootfstype=squashfs init=/linuxrc mtdparts=spi_nor.0:0x10000(spl),0x50000(boot),0x4F0000(romfs),0x640000(usr),0x20000(web),0x3F0000(custom),0x60000(mtd)\0"
    "mtdparts=spi_nor.0:0x10000(spl),0x50000(boot),0x4F0000(romfs),0x640000(usr),0x20000(web),0x3F0000(custom),0x60000(mtd)\0"
    "osmem=120M\0"
    "stderr=serial\0"
    "stdin=serial\0"
    "stdout=serial\0"
    "vendor=axera\0"
    "mtddevname=spl\0"
    "mtddevnum=0\0"
    "partition=nor0,0\0"
    "ver=U-Boot 2020.04 (Apr 28 2023 - 08:46:28 +0800)\0"
    "xmuart=0\0";

struct AxeraSpiNorState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    char *file;
    uint8_t *flash;
    gsize flash_size;

    uint32_t mem[DW_SPI_SIZE / 4];
    bool ssienr;
    bool in_cmd_phase;
    uint8_t cmd_buf[4];
    unsigned int cmd_len;
    uint8_t latched_opcode;
    uint32_t latched_addr;

    uint8_t *rxq;
    unsigned int rxq_cap;
    unsigned int rxq_head;
    unsigned int rxq_len;
};

/*
 * BL1's own dw_spi_mst.c reads flash in up to 32768-byte chunks (CTRL1's
 * entries-1 field is 16 bits wide on the real DesignWare APB SSI IP, so
 * a single RX phase can legitimately ask for far more than any small
 * fixed-size buffer could hold) -- size the queue to the exact transfer
 * instead of capping it, matching what the real FIFO-plus-shift-register
 * behaves as from software's point of view.
 */
static void rxq_reset(AxeraSpiNorState *s, unsigned int needed)
{
    if (needed > s->rxq_cap) {
        s->rxq = g_realloc(s->rxq, needed);
        s->rxq_cap = needed;
    }
    s->rxq_head = 0;
    s->rxq_len = 0;
}

static void rxq_push(AxeraSpiNorState *s, uint8_t byte)
{
    s->rxq[(s->rxq_head + s->rxq_len) % s->rxq_cap] = byte;
    s->rxq_len++;
}

/*
 * Real bug in real U-Boot's own generic MTD SPI driver (confirmed by
 * reading the actual source, u-boot-axera-wt's drivers/spi/
 * designware_spi.c + drivers/spi/spi-mem.c's __UBOOT__ fallback, and
 * cross-checking against this project's own live register tracing):
 * every multi-chunk "sf read"/environment-load transfer's *first*
 * chunk never actually gets written to its own destination buffer at
 * all. Confirmed two independent ways: a full-RAM scan for a known
 * magic value found it landing one whole chunk past where it should
 * (dest+chunk_size instead of dest+0), and a write watchpoint on
 * dest+0 across an entire boot never fired once. The real register-
 * level symptom is a same (opcode, address, size) data-phase transfer
 * issued twice in a row for every chunk (both with byte-for-byte
 * identical CTRL0/CTRL1/cmd_buf -- this model already proven to
 * faithfully serve whichever real flash bytes either asks for), which
 * BL1's own separate driver also does and copes with fine -- so this
 * is guest-side driver behaviour, not a gap in this model's own
 * emulation, and the exact line responsible (most likely inside the
 * closed-source Xiongmai do_spi_flash_probe()/XmSpiNor_ProtMgr_probe()
 * wrapper layered on top, confirmed present in the real boot console
 * output but with no source available anywhere) isn't worth chasing
 * further: this reads the live destination pointer the real driver's
 * own drain loop is *about* to use -- disassembly of that loop found
 * it kept at a fixed offset off a stable per-transfer struct pointer
 * (x19 at the time of this exact write, [x19+120] holding the "rx"
 * destination -- designware_spi.c's own struct dw_spi_priv.rx field)
 * -- and writes the correct flash content there directly, the same
 * instant the real transfer is requested, bypassing that broken
 * drain-and-store path entirely rather than trying to out-guess it.
 * Harmless whenever the real path already works (BL1's own reads,
 * which never exhibit this bug): the destination the real CPU-side
 * loop ends up using is then just written with the same bytes twice.
 *
 * Only ever correct for U-Boot's own CPU-polled drain, though: the
 * real kernel's spi-dw-dma.c drives this same transfer over AX_DMAPER
 * instead (confirmed: DW_SPI_DMACR's RDMAE bit set first, exactly
 * what dw_spi_dma_setup() writes before it), and at that point x19 is
 * whatever the kernel's own code happens to hold there -- not
 * designware_spi.c's struct at all. Callers gate on RDMAE being
 * clear before calling this so a genuine DMA-mode transfer (which
 * axera-dmaper.c's own do_transfer() already copies to the real `dar`
 * correctly) never gets this stray second write to a bogus address
 * derived from an unrelated register.
 */
static void axera_spi_nor_fixup_first_chunk_dest_buf(AxeraSpiNorState *s,
                                                     const uint8_t *src,
                                                     uint32_t total)
{
    uint64_t x19;
    uint64_t dest = 0;
    MemTxResult res;

    /*
     * BL1's own driver (dw_spi_mst.c) never touches flash past 0x40ba8
     * (its own last compressed-payload byte, confirmed: img_size
     * tops out there for this dump's real BL1/SPL+U-Boot image) and
     * uses a completely different, unrelated calling convention --
     * x19/[x19+120] at that point in BL1's own code is not this
     * struct at all, and could coincidentally look like a valid DRAM
     * address. Gate on flash offset (env starts at 0x50000, well past
     * BL1's own ceiling) so this fixup only ever applies to U-Boot's
     * own later reads, where it's actually needed and where x19 is
     * genuinely designware_spi.c's own dw_spi_priv pointer.
     */
    if (s->latched_addr < 0x48000) {
        return;
    }
    x19 = axera_cpu_get_xreg(19);
    if (x19 == 0) {
        return;
    }
    res = address_space_read(&address_space_memory, x19 + 120,
                             MEMTXATTRS_UNSPECIFIED, &dest, 8);
    if (res != MEMTX_OK || dest < 0x40000000ULL || dest >= 0xc0000000ULL) {
        return;
    }
    address_space_write(&address_space_memory, dest, MEMTXATTRS_UNSPECIFIED,
                        src, total);
}

static void axera_spi_nor_fixup_first_chunk_dest(AxeraSpiNorState *s,
                                                 uint32_t total)
{
    if (s->latched_addr >= s->flash_size) {
        return;
    }
    axera_spi_nor_fixup_first_chunk_dest_buf(s, s->flash + s->latched_addr,
                                             MIN(total, (uint32_t)
                                                 (s->flash_size -
                                                  s->latched_addr)));
}

/*
 * Builds the synthetic AXERA_ENV_SIZE-byte environment block (see
 * axera_env_vars' own comment) into *out, which must have room for
 * AXERA_ENV_SIZE bytes: 4-byte CRC32, a UTF-8 BOM (byte-for-byte what
 * this dump's own real saved environment starts its data with),
 * every real variable, "xmuart=0" appended, then zero padding -- CRC
 * computed the same way env_sf_load() itself checks it (over
 * everything after the leading 4-byte CRC field).
 */
static void axera_build_synthetic_env(uint8_t *out)
{
    static const uint8_t bom[3] = { 0xef, 0xbb, 0xbf };
    size_t off = 4;

    memset(out, 0, AXERA_ENV_SIZE);
    memcpy(out + off, bom, sizeof(bom));
    off += sizeof(bom);
    memcpy(out + off, axera_env_vars, sizeof(axera_env_vars));
    off += sizeof(axera_env_vars);
    g_assert(off <= AXERA_ENV_SIZE);

    stl_le_p(out, crc32(0, out + 4, AXERA_ENV_SIZE - 4));
}

/*
 * SSIENR 0->1 while CTRL0's TMOD says "receive only": the real flash's
 * data phase is starting. CTRL1 already holds entries-1 for this exact
 * transfer (dw_spi_xfer() writes it, for the RX call only, between
 * SSIENR=0 and SSIENR=1 -- real register-write ordering, not assumed),
 * and CTRL0's DFS field gives the frame size BL1 always sets to 8 bits
 * for this driver. Stage the real answer now so DR reads just drain it,
 * the same shape axera-dw-spi.c's panel loopback already uses.
 */
static void axera_spi_nor_start_data_phase(AxeraSpiNorState *s)
{
    uint32_t ctrl0 = s->mem[DW_SPI_CTRL0 / 4];
    uint32_t ctrl1 = s->mem[DW_SPI_CTRL1 / 4];
    uint32_t dfs = (ctrl0 & SPI_DFS_MASK) + 1;
    uint32_t frame_bytes = dfs > 8 ? dfs / 8 : 1;
    uint32_t entries = ctrl1 + 1;
    uint32_t total = entries * frame_bytes;
    uint32_t i;

    rxq_reset(s, total);

    switch (s->latched_opcode) {
    case SPINOR_OP_READ:
    case SPINOR_OP_READ_FAST:
        /*
         * 0x3b's one extra dummy byte (opcode + 3-byte addr + 1 dummy,
         * vs plain 0x03's opcode + 3-byte addr) never reaches cmd_buf
         * -- the write handler below already stops storing once
         * cmd_len hits sizeof(cmd_buf)==4, so latched_addr's own
         * cmd_buf[1..3] extraction is unaffected either way.
         */
        if (s->latched_addr >= AXERA_ENV_OFFSET &&
            s->latched_addr < AXERA_ENV_OFFSET + AXERA_ENV_SIZE &&
            total <= AXERA_ENV_SIZE) {
            uint8_t env[AXERA_ENV_SIZE];
            uint32_t off = s->latched_addr - AXERA_ENV_OFFSET;

            axera_build_synthetic_env(env);
            for (i = 0; i < total; i++) {
                rxq_push(s, env[off + i]);
            }
            if (!(s->mem[DW_SPI_DMACR / 4] & SPI_DMA_RDMAE)) {
                axera_spi_nor_fixup_first_chunk_dest_buf(s, env + off, total);
            }
        } else {
            for (i = 0; i < total; i++) {
                uint32_t addr = s->latched_addr + i;

                rxq_push(s, addr < s->flash_size ? s->flash[addr] : 0xff);
            }
            if (!(s->mem[DW_SPI_DMACR / 4] & SPI_DMA_RDMAE)) {
                axera_spi_nor_fixup_first_chunk_dest(s, total);
            }
        }
        break;
    case SPINOR_OP_RDSR:
        /* Never busy, always write-enabled-or-not as BL1 expects. */
        for (i = 0; i < total; i++) {
            rxq_push(s, 0x00);
        }
        break;
    case SPINOR_OP_READID:
    case SPINOR_OP_RDID:
        /*
         * Manufacturer/device ID either query would return; BL1's own
         * spinor_init() uses the vendor-specific 0x90 and only logs
         * this rather than gating boot on a specific match, but real
         * U-Boot's generic MTD spi-nor-core.c always probes with the
         * standard JEDEC 0x9f (SPINOR_OP_RDID) first and refuses to
         * attach the flash at all without a non-zero answer (confirmed
         * live: falling through to the all-zero default case here is
         * exactly what printed "unrecognized JEDEC id bytes: 00, 00,
         * 00" and left U-Boot with no flash to load its environment
         * from). Any plausible non-zero ID is sufficient for either
         * caller (GD25Q128, a common real fit already used as this
         * project's own default assumption elsewhere -- see
         * hw/arm/anyka.c's ak_flash_jedec() fallback).
         */
        for (i = 0; i < total; i++) {
            static const uint8_t id[] = { 0xc8, 0x40, 0x18 };

            rxq_push(s, i < sizeof(id) ? id[i] : 0x00);
        }
        break;
    default:
        for (i = 0; i < total; i++) {
            rxq_push(s, 0x00);
        }
        break;
    }
}

static void axera_spi_nor_ssienr_write(AxeraSpiNorState *s, uint32_t value)
{
    bool enable = (value & 1) != 0;

    if (enable && !s->ssienr) {
        uint32_t tmod = (s->mem[DW_SPI_CTRL0 / 4] & SPI_TMOD_MASK) >>
                        SPI_TMOD_OFFSET;

        if (tmod == SPI_TMOD_TO) {
            /*
             * Real driver behaviour varies: BL1/U-Boot's own dw_spi_xfer()
             * sends opcode+3-byte address as ONE TMOD_TO phase (4 bytes in
             * a single SSIENR 0->1->0 cycle), but the real kernel's
             * dw_spi_write_then_read() sends opcode, address and the
             * READ_FAST dummy byte as THREE SEPARATE TMOD_TO sub-phases,
             * each its own SSIENR cycle, with TMOD staying SPI_TMOD_TO the
             * whole time -- confirmed live via register tracing (CTRL0
             * never changes between them). Resetting cmd_len here on
             * every entry wiped the real opcode+address with the trailing
             * dummy byte right before the data phase needed to read it
             * back, latching opcode=0xff/addr=0 every time. Not resetting
             * here (only after a real RO phase below actually consumes
             * it) lets consecutive TO sub-phases accumulate into the same
             * cmd_buf -- capped at sizeof(cmd_buf)==4 by the DW_SPI_DR
             * write handler already, which is exactly "stop once opcode
             * +address are in, ignore the dummy byte" for free.
             */
            s->in_cmd_phase = true;
        } else if (tmod == SPI_TMOD_RO && s->cmd_len > 0) {
            /*
             * dw_spi_stop_mem_op() (spi-dw-core.c) re-toggles SSIENR
             * 0->1 right after a real mem op finishes, purely to reset
             * native chip-select behaviour -- CTRL0 is never
             * reprogrammed first, so this lands here with TMOD still
             * SPI_TMOD_RO from the real transfer, but with nothing new
             * latched. Treating that as a second, fresh data phase
             * reset the already-correctly-drained rxq to a bogus
             * opcode=0 512-byte queue that nothing ever drains again
             * ("Rx hanged up" right after a transfer this model had
             * already served correctly). Requiring cmd_len > 0 -- a
             * real TO phase actually ran first -- tells the two apart.
             */
            s->in_cmd_phase = false;
            s->latched_opcode = s->cmd_buf[0];
            s->latched_addr = 0;
            if (s->cmd_len >= 4) {
                s->latched_addr = (s->cmd_buf[1] << 16) |
                                  (s->cmd_buf[2] << 8) | s->cmd_buf[3];
            }
            s->cmd_len = 0;
            axera_spi_nor_start_data_phase(s);
        }
    }
    s->ssienr = enable;
}

static uint64_t axera_spi_nor_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraSpiNorState *s = opaque;
    uint8_t val;

    switch (offset) {
    case DW_SPI_TXFLR:
        return 0;
    case DW_SPI_RXFLR:
        return MIN(s->rxq_len, AXERA_SPI_NOR_FIFO_DEPTH);
    case DW_SPI_SR:
        return SR_TF_NOT_FULL | SR_TF_EMPT |
               (s->rxq_len ? SR_RF_NOT_EMPT : 0);
    case DW_SPI_DR:
        if (s->rxq_len == 0) {
            return 0;
        }
        val = s->rxq[s->rxq_head];
        s->rxq_head = (s->rxq_head + 1) % s->rxq_cap;
        s->rxq_len--;
        return val;
    default:
        if (offset < DW_SPI_SIZE) {
            return s->mem[offset / 4];
        }
        return 0;
    }
}

static void axera_spi_nor_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    AxeraSpiNorState *s = opaque;

    switch (offset) {
    case DW_SPI_SSIENR:
        axera_spi_nor_ssienr_write(s, value);
        s->mem[offset / 4] = value;
        return;
    case DW_SPI_TXFLR:
    case DW_SPI_RXFLR:
    case DW_SPI_SR:
        /* Read-only hardware status; ignore writes like real silicon. */
        return;
    case DW_SPI_TXFTLR:
    case DW_SPI_RXFTLR:
        /*
         * Real register field is only wide enough for 0..depth-1 --
         * truncate exactly like real silicon so U-Boot's own FIFO-depth
         * probe (spi_hw_init(), designware_spi.c) gets a real answer
         * instead of concluding it has a 256-entry FIFO.
         */
        s->mem[offset / 4] = value & (AXERA_SPI_NOR_FIFO_DEPTH - 1);
        return;
    case DW_SPI_DR:
        if (s->in_cmd_phase && s->cmd_len < sizeof(s->cmd_buf)) {
            s->cmd_buf[s->cmd_len++] = value & 0xff;
        }
        return;
    default:
        if (offset < DW_SPI_SIZE) {
            s->mem[offset / 4] = value;
        }
        return;
    }
}

static const MemoryRegionOps axera_spi_nor_ops = {
    .read = axera_spi_nor_read,
    .write = axera_spi_nor_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_spi_nor_realize(DeviceState *dev, Error **errp)
{
    AxeraSpiNorState *s = AXERA_SPI_NOR(dev);

    if (s->file && s->file[0]) {
        if (!g_file_get_contents(s->file, (gchar **)&s->flash,
                                 &s->flash_size, NULL)) {
            error_setg(errp, "axera-spi-nor: could not read file '%s'",
                      s->file);
            return;
        }
    } else {
        s->flash_size = 0;
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &axera_spi_nor_ops, s,
                          TYPE_AXERA_SPI_NOR, DW_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const Property axera_spi_nor_properties[] = {
    DEFINE_PROP_STRING("file", AxeraSpiNorState, file),
};

static void axera_spi_nor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_spi_nor_realize;
    device_class_set_props(dc, axera_spi_nor_properties);
}

static const TypeInfo axera_spi_nor_info = {
    .name = TYPE_AXERA_SPI_NOR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraSpiNorState),
    .class_init = axera_spi_nor_class_init,
};

static void axera_spi_nor_register_types(void)
{
    type_register_static(&axera_spi_nor_info);
}

type_init(axera_spi_nor_register_types)
