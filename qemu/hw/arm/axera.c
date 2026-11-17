/*
 * Axera camera-SoC machines: AX630C/AX620E (two Cortex-A53s, AArch64),
 * AX620Q (two Cortex-A7s, AArch32) and AX620U (four Cortex-A7s, AArch32,
 * a distinct SoC from AX620Q despite the shared core -- root compatible
 * "axera,ax620x", not "axera,ax620e"/"axera,ax620q"). All four share one
 * GIC-400 at 0x01850000 (distributor at +0x1000, CPU interface at
 * +0x2000) and the ARM generic timer at 24 MHz (the dtsi's "timer" node
 * clock-frequency); only the UART0 base differs per SoC (AxeraSoCConfig's
 * uart0_base/uart0_irq). AX620Q and AX620U bring their extra cores up
 * through their own vendor spin-tables ("axera,ax620e-smp" /
 * "axera,ax620x-smp": CPU0 writes the entry address to a board-specific
 * "secondary-boot-reg"), not PSCI, and neither is modelled yet -- only
 * CPU0 boots on either, the same "leave CPU1 offline" stance this tree
 * already takes for release methods it has not modelled (see the
 * Goke/HiSilicon MP2 parts).
 *
 * There is no machine-built device tree: boot with the kernel's own
 * (-dtb), as novatek's nt98332 does for the same reason -- the AX620E
 * family's clock, pinctrl and reset drivers read almost everything from
 * it, and nothing here tries to be a second source of truth for that map.
 *
 * Sources: the onekvm/linux-6.18-ax620e tree's
 * arch/arm64/boot/dts/axera/ax620e.dtsi (AX630C/AX620E, AArch64),
 * arch/arm/boot/dts/axera/ax620e.dtsi + ax620q-nanokvm-go.dts (AX620Q,
 * AArch32, DRAM at 0x40000000/256 MiB) and arch/arm/mach-axera/platsmp.c
 * (the spin-table SMP method); and, for AX620E specifically, the real
 * devicetree extracted from a deployed Xiongmai IVG-A6 (AX620E) camera firmware's
 * FIT-image kernel (DRAM at 0x40000000/1 GiB; GIC/timer/UART addresses
 * confirmed byte-for-byte against the onekvm tree's).
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/axera.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/axera-axi-dmac.h"
#include "hw/misc/axera-ax620u-clkrst.h"
#include "hw/misc/axera-ax620u-ddr-pll.h"
#include "hw/misc/axera-ax620u-smp-wake.h"
#include "hw/misc/axera-bl1-timer.h"
#include "hw/misc/axera-ddr-phy.h"
#include "hw/misc/axera-ddr-pll.h"
#include "hw/misc/axera-dma-checksum.h"
#include "hw/misc/axera-dmaper.h"
#include "hw/misc/axera-i2s.h"
#include "hw/misc/axera-gzipd.h"
#include "hw/misc/axera-dw-spi.h"
#include "hw/misc/axera-eip130.h"
#include "hw/misc/axera-pllc.h"
#include "hw/misc/axera-spi-nand.h"
#include "hw/misc/axera-spi-nor.h"
#include "hw/misc/axera-vcmd.h"
#include "hw/misc/axera-wdt.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/cadence_sdhci.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-dwc3.h"
#include "hw/usb/hcd-xhci-sysbus.h"
#include "net/net.h"
#include "qapi/visitor.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/gtimer.h"
#include "target/arm/multiprocessing.h"

#define TYPE_AXERA_MACHINE MACHINE_TYPE_NAME("axera")
OBJECT_DECLARE_SIMPLE_TYPE(AxeraMachineState, AXERA_MACHINE)

struct AxeraMachineState {
    MachineState parent_obj;
    char *flash_file;
    char *uboot_file;
};

/* CONFIG_SYS_TEXT_BASE from the vendor U-Boot's own AX630C defconfig --
 * within the normal DRAM window, no separate RAM carve-out needed. */
#define AXERA_UBOOT_BASE        0x5c000400ULL

/* CONFIG_SYS_TEXT_BASE from the vendor U-Boot's own AX620Q eMMC defconfig
 * (a lower address than AX630C's -- AX620Q's real DRAM window is 256 MiB,
 * not 2 GiB). No real AX620U U-Boot source exists in the vendor dump, so
 * only AX620Q gets a uboot-file= entry point for now. */
#define AXERA_AX620Q_UBOOT_BASE 0x44000400ULL

/*
 * BL1/SPL's real link address, read directly from the vendor SDK's own
 * linker script (boot/bl1/board/arch/arm64/bl1.lds): a 1 MiB internal
 * ROM/SRAM window, not DDR -- ENTRY(_start), MEMORY { iRom (rx) :
 * ORIGIN = 0x03000400, LENGTH = 0x100000 }.
 */
#define AXERA_BL1_BASE          0x03000400ULL
#define AXERA_BL1_SIZE          (1 * MiB)

/*
 * TOP_CHIPMODE_GLB_SW (TOP_CHIPMODE_GLB + 0xC, see the axera-chipmode
 * regbank comment above): the flash_type board.c's get_boot_mode()
 * decodes from bits [3:1] (chip_mode & 0xe) >> 1, so BL1 runs the real
 * driver matching the real dump's own flash -- "EMMC BOOT 8BIT_50M_768K"
 * (flash_type=1, chip_mode=0x2) for AX630C's real eMMC dev-board dump,
 * "NOR" (flash_type=7, chip_mode=0xe) for AX620E's real SPI NOR camera
 * dump. Confirmed from a real deployed AX620E NOR dump getting stuck
 * forever with the SPI NOR controller's own registers completely
 * untouched (SSIENR etc all still 0) while this was unconditionally
 * preseeding the eMMC value regardless of which real flash the dump
 * actually used -- BL1 was dutifully calling emmc_init()/trying to read
 * U-Boot's image off a storage type that was never there.
 */
#define AXERA_CHIPMODE_SW_ADDR  0x0239000cULL
#define AXERA_CHIPMODE_EMMC_BOOT_8BIT 0x2U
#define AXERA_CHIPMODE_NOR      0xeU

/*
 * PIN_MUX_G11_VDET_RO0 (board.c's get_emmc_voltage(), bit 12: 1=3.3V,
 * 0=1.8V). With this left at its catch-all/regbank default of 0, BL1
 * sends CMD1 (SEND_OP_COND) with EMMC_IO_1V8 (bit 7) as its OCR probe
 * argument -- bit 7 isn't covered by either the VDD_VOLTAGE_WIN_HI
 * field (bits 8-23) or ACCEPT_SWITCH_1V8 (bit 24) that QEMU's generic
 * eMMC model advertises in sd_set_ocr(), so sd_cmd_SEND_OP_COND()'s
 * `sd->ocr & req.arg` is always 0 and the card never leaves
 * sd_idle_state -- CMD2 (ALL_SEND_CID) right after then fails with
 * "invalid state for cmd", surfacing to BL1 as a plain CMD_RESP
 * error indistinguishable from a real timeout. Forcing bit 12 here
 * makes BL1 pick the 3.3V probe argument (EMMC_IO_3V3, bits 15-22),
 * which the same advertised OCR does cover -- real eMMC on real
 * boards is overwhelmingly 3.3V-strapped anyway, not 1.8V UHS-style.
 */
#define AXERA_VDET_RO0_ADDR     0x023090a0ULL
#define AXERA_VDET_3V3_BIT      (1U << 12)

/*
 * Lets hw/misc/axera-spi-nor.c (plain system_ss "common" code, which
 * cannot itself include target/arm/cpu.h without breaking the
 * multi-target build) peek at CPU0's current x-registers. Needed to
 * work around a real bug in real U-Boot's own generic MTD SPI driver
 * (designware_spi.c/spi-mem.c's __UBOOT__ fallback): every multi-chunk
 * "sf read"/env-load transfer's first chunk never actually gets
 * written to its own destination buffer at all (confirmed: a
 * full-RAM-scan plus a full-boot write-watchpoint on that exact
 * address both show it is simply never touched) -- see
 * axera_spi_nor_fixup_first_chunk_dest()'s own comment for the full
 * story and why this reads the live x19/[x19+120] destination pointer
 * directly off the CPU instead.
 */
uint64_t axera_cpu_get_xreg(unsigned int n)
{
    ARMCPU *cpu = ARM_CPU(first_cpu);

    if (n >= ARRAY_SIZE(cpu->env.xregs)) {
        return 0;
    }
    return cpu->env.xregs[n];
}

static void axera_bl1_chipmode_reset(void *opaque)
{
    bool flash_is_nor = (uintptr_t)opaque;

    address_space_stl_le(&address_space_memory, AXERA_CHIPMODE_SW_ADDR,
                         flash_is_nor ? AXERA_CHIPMODE_NOR
                                      : AXERA_CHIPMODE_EMMC_BOOT_8BIT,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    /* The 3.3V-probe strap only matters for the eMMC OCR dance below;
     * harmless to leave set either way, but only meaningful for eMMC. */
    address_space_stl_le(&address_space_memory, AXERA_VDET_RO0_ADDR,
                         AXERA_VDET_3V3_BIT,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * What the AX620Q's BL1/SPL leaves in iram0 for U-Boot and the kernel
 * (boot/bl1/spl/spl_main.c setup_boot_mode() and the misc_info block,
 * layouts from arch/arm/include/asm/arch-axera/boot_mode.h). uboot-file=
 * and -kernel both skip SPL, so this stands in for it, the same way
 * axera_bl1_chipmode_reset() stands in for the boot straps.
 *
 * boot_mode_info @ 0x700: magic, mode (SPL's BOOT_MODE_NORMALBOOT),
 * storage_sel/boot_type (the board's eMMC UDA strap), and is_sd_boot --
 * set when an SD card is attached at -drive if=sd,index=1, standing in
 * for the boot ROM's COMM_SYS_DUMMY_SW5 == DL_CHAN_SD report. U-Boot
 * turns is_sd_boot into its "sd_boot" bootcmd.
 *
 * misc_info @ 0x740: chip_type/phy_board_id select the board.
 * AX620QF_CHIP (9) + PHY_AX620QF_LP4_NANOAGENT_256M (1) is the 256 MiB
 * NanoKVM-Go this machine models; U-Boot's sd_boot only takes the
 * NanoKVM-Go path (/boot/kernel.img off the ext4 p2) for that pair.
 *
 * GPIO3 @ 0x06001000, ext-port register +0x8c bit 2: the board's boot
 * key, pulled up on the real board. Reading 0 means "pressed", which
 * sends U-Boot and the initramfs into USB-recovery mode.
 */
#define AXERA_BOOT_MODE_INFO_ADDR   0x700ULL
#define AXERA_BOOT_MODE_MAGIC       0x12345678U
#define AXERA_SPL_MODE_NORMALBOOT   0x2U
#define AXERA_STORAGE_TYPE_EMMC     0x0U
#define AXERA_EMMC_BOOT_UDA         0x1U
#define AXERA_MISC_INFO_ADDR        0x740ULL
#define AXERA_AX620QF_CHIP          0x9U
#define AXERA_PHY_NANOAGENT_256M    0x1U
#define AXERA_GPIO3_EXT_PORT_ADDR   0x0600108cULL
#define AXERA_BOOT_KEY_RELEASED     (1U << 2)

static void axera_ax620q_spl_handoff_reset(void *opaque)
{
    bool sd_boot = (uintptr_t)opaque;
    AddressSpace *as = &address_space_memory;
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;
    hwaddr b = AXERA_BOOT_MODE_INFO_ADDR;
    hwaddr m = AXERA_MISC_INFO_ADDR;
    int i;

    for (i = 0; i < 0x40; i += 4) {
        address_space_stl_le(as, b + i, 0, attrs, NULL);
    }
    address_space_stl_le(as, b + 0x00, AXERA_BOOT_MODE_MAGIC, attrs, NULL);
    address_space_stl_le(as, b + 0x04, AXERA_SPL_MODE_NORMALBOOT, attrs, NULL);
    address_space_stl_le(as, b + 0x0c, AXERA_STORAGE_TYPE_EMMC, attrs, NULL);
    address_space_stl_le(as, b + 0x10, AXERA_EMMC_BOOT_UDA, attrs, NULL);
    address_space_stb(as, b + 0x14, sd_boot, attrs, NULL);

    for (i = 0; i < 0x60; i += 4) {
        address_space_stl_le(as, m + i, 0, attrs, NULL);
    }
    address_space_stl_le(as, m + 0x44, AXERA_AX620QF_CHIP, attrs, NULL);
    address_space_stl_le(as, m + 0x5c, AXERA_PHY_NANOAGENT_256M, attrs, NULL);

    address_space_stl_le(as, AXERA_GPIO3_EXT_PORT_ADDR,
                         AXERA_BOOT_KEY_RELEASED, attrs, NULL);
}

typedef struct {
    ARMCPU *cpu;
    uint64_t entry;
} AxeraBootromReset;

static void axera_bootrom_cpu_reset(void *opaque)
{
    AxeraBootromReset *info = opaque;
    CPUState *cs = CPU(info->cpu);

    cpu_reset(cs);
    cpu_set_pc(cs, info->entry);
}

static void axera_cpu_reset_only(void *opaque)
{
    cpu_reset(CPU(opaque));
}

/* GICC includes DIR at +0x1000 past the CPU interface, as the dtsi notes. */
#define AXERA_GIC_DIST          0x01851000
#define AXERA_GIC_CPU           0x01852000
#define AXERA_NUM_SPI           160

/* serial@4880000, "snps,dw-apb-uart": reg-shift 2, clock-frequency 208 MHz. */
#define AXERA_UART0_BASE        0x04880000ULL
#define AXERA_UART0_IRQ         118
#define AXERA_UART0_FREQ        208000000

/* The dtsi's timer clock-frequency: the vendor 24 MHz CNTFRQ, on both the
 * armv7-timer (AX620Q) and armv8-timer (AX630C/AX620E) nodes. */
#define AXERA_CNTFRQ            24000000

/*
 * Everything from physical address 0 up through the eMMC/SD/USB window,
 * so an unmodelled block reads 0 / ignores writes instead of hanging the
 * bus. Real blocks below map over pieces of this with mmio_map_overlap-
 * style priority; for now nothing does, so it is one plain unimplemented
 * region. AX620Q's own block set is smaller (no VENC/ISP/crypto in its
 * dtsi), but mapping the wider AX630C/AX620E range there too is harmless.
 *
 * Starts at 0, not the GIC's 0x01800000ish window: real U-Boot's
 * chip_init() (store_chip_type(), via arch/arm/mach-axera/ax620e/
 * chip_config.c) unconditionally reads a vendor misc-info struct at the
 * real SoC's iRAM0 address 0x740, before console is up -- with nothing
 * mapped that low, QEMU raises a synchronous external abort there
 * (EC=0x25 data abort, DFSC=0x10) which U-Boot's own exception handler
 * turns into a silent panic()->hang(), with no UART output at all since
 * gd->have_console is still 0. A passive catch-all there is enough: the
 * driver only reads a handful of fields and tolerates them reading 0.
 */
#define AXERA_PERIPH_BASE       0x0ULL
#define AXERA_PERIPH_SIZE       0x10b00000ULL

#define AXERA_RAM_BASE          0x40000000ULL

/*
 * Simple syscon-style blocks shared by AX630C/AX620E (clock controllers,
 * reset regmaps aliasing the same syscon memory as their clock
 * controller so only one entry is needed per address, pinctrl, GPIO):
 * RAM-backed storage (hisi-regbank, reused from the HiSilicon family, not
 * Axera-specific) so a driver's write-then-read probe sees what it wrote,
 * unlike the unimplemented catch-all underneath. Addresses from the
 * dtsi; sizes match its reg properties.
 */
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} ax620e_regbanks[] = {
    /*
     * TOP_CHIPMODE_GLB (boot/bl1/common/include/chip_reg.h): BL1's own
     * main() reads the SW register here as a hardware boot-mode strap
     * (chip_mode) and feeds it straight into get_boot_mode() (board.c),
     * which maps chip_mode bits [3:1] to a flash_type enum -- 0 means
     * "EMMC UDA", the "already initialized" fast-boot profile BL1
     * assumes a real boot ROM already brought the card up for (true on
     * real silicon, since the boot ROM itself read BL1 off the card
     * before BL1 ever ran; false under flash-file= direct injection,
     * which skips that boot ROM step entirely). A plain zeroed regbank
     * here is indistinguishable from strap pins wired to 0, so BL1
     * takes that shortcut, finds the card still in idle state, and its
     * CMD6/SWITCH (bus-width select) fails over and over with no
     * timeout. See the flash-file= reset hook below, which preseeds
     * this to flash_type=1 ("EMMC BOOT 8BIT_50M_768K") so BL1 runs its
     * real emmc_init_card() sequence instead.
     */
    { "axera-chipmode",     0x02390000, 0x1000 },
    { "axera-tsensor",      0x02000000, 0x1000 },
    { "axera-clk-cpu",      0x01900000, 0x10000 },
    { "axera-clk-common",   0x02340000, 0x10000 },
    { "axera-dphytx",       0x023f1000, 0x1000 },
    { "axera-isp-syscon",   0x02500000, 0x10000 },
    { "axera-clk-flash",    0x10030000, 0x10000 },
    { "axera-clk-periph",   0x04870000, 0x10000 },
    { "axera-dispc-reset",  0x04600000, 0x100 },
    { "axera-clk-vpu",      0x04030000, 0x10000 },
    { "axera-clk-mm",       0x04430000, 0x10000 },
    { "axera-pinctrl",      0x02300000, 0xb000 },
    { "axera-pinctrl-aux",  0x104f0000, 0x3000 },
    { "axera-gpio0",        0x04800000, 0x400 },
    { "axera-gpio1",        0x04801000, 0x400 },
    { "axera-gpio2",        0x06000000, 0x400 },
    { "axera-gpio3",        0x06001000, 0x400 },
    /*
     * CSI2/TDP (VIN/ISP itself, 0x02400000, is now a real device --
     * axera-vin below, created where pic[] exists -- see its own header
     * comment): the real open driver (open_vin_capture.c,
     * open_vin_csi2.c) is a huge, bring-up-derived register protocol
     * (clock-mux "golden" settle values, per-channel WDMA trigger/seq/
     * addr/enable, multiple interrupt groups) -- faithfully modelling
     * frame DMA for these remaining blocks is future work (see
     * docs/axera.md), but a plain regbank at least lets the driver's
     * many write-then-poll-for-settle loops see what they wrote,
     * instead of always reading 0 and spinning to a timeout.
     */
    { "axera-mipi-csi",     0x02600000, 0x4000 },
    { "axera-mipi-dphy",    0x023f0000, 0x1000 },
    /*
     * DDRMC @ 0x00100000 (boot/bl1/driver/ddr/ddr/ddrmc_reg_addr_def.h,
     * D_DDRMC_*): ddr_init.c/ddr_scan.c read back plenty of these
     * mid-training (port config, retrain/eye-scan registers at the
     * +0xc00 sub-block) the same write-then-read way everything else
     * in this table needs a real readback for. No dynamic bit of its
     * own is known to matter yet, unlike the DDR PHY blocks below --
     * add one here if a future training stage needs it.
     */
    { "axera-ddrmc",        0x00100000, 0x10000 },
};

/*
 * The real IP blocks (codecs, crypto, capture, storage, network, USB):
 * named unimplemented stubs, so -d unimp names the actual block a boot
 * stage polls instead of one generic "axera-periph" blob. No protocol
 * behaviour yet -- that needs a real device model per block, future work
 * (see docs/axera.md).
 */
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} ax620e_unimp[] = {
    /*
     * axera-npu @ 0x03800000, size 0xb0000: real DT confirms "axera,npu"
     * reg <0x3800000 0xb0000>, <0x3000000 0x280000> plus 8 clock inputs
     * (cpll/epll/vpll/npll derived), extracted from a real deployed
     * AX620E camera's own FDT. NPU driver source (ax_npu.ko) is closed
     * -- unavailable in every public SDK generation -- so this can only
     * be characterised empirically, not from source.
     *
     * Investigated live (2026-11-15) via -d unimp plus full-boot traces
     * (one 150s run, one 400s/3-reboot run) against the real deployed
     * firmware dump. Findings:
     *
     * - This block IS touched every boot: a bounded ~116-access burst
     *   (clock/reset/IRQ-mask register pokes across four 0x10000-spaced
     *   sub-blocks at 0x00000/0x10000/0x20000/0x30000, plus small
     *   control regions at 0x40000/0x50000/0x80000/0x90000) right after
     *   kernel module probe, every single boot cycle. No access ever
     *   falls outside this 0xb0000 window.
     * - The only reads are short, bounded polls (3-6 reads of a status
     *   word at offset 0x90000, never retried beyond that) -- nothing
     *   observed treats "always reads back 0" as a failure: no hang, no
     *   WARN/error print, no retry storm attributable to this block in
     *   either trace. The bare "return 0 to everything" stub already
     *   satisfies whatever this init sequence checks.
     * - What looked like periodic re-polling during steady state turned
     *   out, on closer inspection, to be fresh per-boot re-probes: the
     *   400s trace reboots 3 times, driven by two already-documented,
     *   NPU-unrelated causes (an early Gpio2Env-triggered reboot, then
     *   the CEncode zero-bitrate watchdog -- see
     *   ax620e-nobitnoreboot-flag-workaround-CONFIRMED-working-live).
     * - The real App's only NPU-shaped consumer (xmcap_td.c's AI
     *   "Target Detection" task, alongside xmcap_motion_detect.c) never
     *   reaches a real compute submission: both fail upstream at
     *   XmCap_Vpss_getChnFrame (ret 0x800d0120, AX_ERR_IVPS_BUF_EMPTY)
     *   every cycle, because no real sensor/ISP pixel data ever flows
     *   into VPSS -- the same already out-of-scope root cause that
     *   blocks RTSP video streaming project-wide.
     *
     * Net: no doorbell/completion emulation is needed here. The bare
     * stub is a confirmed-sufficient, not merely assumed-sufficient,
     * fit for both boot and current App startup -- there is no reached
     * code path today that needs more from this block.
     */
    { "axera-npu",          0x03800000, 0xb0000 },
};

static void axera_map_regbanks(void)
{
    int n;

    /* clk_pllc @ 0x02210000: BL1 enables the CPU PLL and VPLL0 here and
     * spins waiting for their ready bits (see hw/misc/axera-pllc.c) --
     * a real device, not a passive regbank, is required for BL1's
     * flash-boot path to get past PLL bring-up at all. */
    sysbus_create_simple(TYPE_AXERA_PLLC, 0x02210000, NULL);

    /* spi2 @ 0x06072000, the SPI-attached display panel's bus: U-Boot's
     * poll_transfer() loops on the RX FIFO level with no timeout (see
     * hw/misc/axera-dw-spi.c) -- a passive regbank hangs the boot dead
     * silently right after console init. */
    sysbus_create_simple(TYPE_AXERA_DW_SPI, 0x06072000, NULL);

    /* TIMER_BASE @ 0x04830000 (BL1's delay timer, Linux's ax_hrtimer):
     * created in axera_ax620e_periphs(), where its SPI exists. */

    /* dpll_ctrl_glb @ 0x00265000, the DDR system DPLL (a separate PLL
     * instance/register layout from clk_pllc above, confirmed from the
     * real vendor source and a gdb trace of a silent flash-file= SPI
     * NOR boot): BL1 enables it and spins on two separate ready-bit
     * polls before DDR training can even start (see
     * hw/misc/axera-ddr-pll.c). */
    sysbus_create_simple(TYPE_AXERA_DDR_PLL, 0x00265000, NULL);

    /*
     * DDRPHY channel A @ 0x00110000, channel B @ 0x00120000 (A + the
     * real DDR_PHY_OFFSET of 0x10000, per ddrphy_reg_addr_def.h). Real
     * BL1's ddrphy_zqc_seq() (ddr_init.c) spins here forever (12M+
     * accesses traced on the real dump) in a bounded SAR calibration
     * search whose comparator bits a passive regbank can't supply --
     * see hw/misc/axera-ddr-phy.c's own comment for why this needs a
     * dynamic comparator rather than the plain readback the rest of
     * this training stage gets below.
     */
    sysbus_create_simple(TYPE_AXERA_DDR_PHY, 0x00110000, NULL);
    sysbus_create_simple(TYPE_AXERA_DDR_PHY, 0x00120000, NULL);

    /*
     * AX_DMAPER @ 0x048a0000: created down in axera_a64_init() instead
     * of here, since it needs its real IRQ (GIC SPI 0x70) wired at
     * creation time via pic[], which doesn't exist yet this early --
     * this function runs before the GIC is even set up. See that call
     * site's own comment for AX_DMAPER's own real purpose.
     */

    /*
     * DMADIM (hardware word-checksum DMA) @ 0x10460000 (boot/bl1/
     * driver/axdma/axdma_checksum.c): boot.c's calc_word_chksum() tries
     * this first before falling back to a plain CPU loop, and the
     * fallback only triggers on a reported hardware failure, not a
     * poll that never finishes -- see hw/misc/axera-dma-checksum.c's
     * own comment.
     */
    sysbus_create_simple(TYPE_AXERA_DMA_CHECKSUM, 0x10460000, NULL);

    /*
     * GZIPD @ 0x10410000 (boot/bl1/driver/gzipd/ax_gzipd_reg.h): BL1
     * routes every non-DDRINIT flash image through this engine to
     * decode Axera's own proprietary "AXGZIP" container format -- the
     * real AX620E NOR dump's own U-Boot image genuinely needs this
     * (confirmed: starts with the real container magic, and the
     * decoded output's own embedded link-address field matches this
     * project's real UBOOT_IMG_HEADER_BASE exactly) -- see
     * hw/misc/axera-gzipd.c's own comment for the full format and how
     * it was reverse-engineered (no source for the algorithm exists
     * anywhere, only a compiled host-side tool used as an oracle).
     */
    sysbus_create_simple(TYPE_AXERA_GZIPD, 0x10410000, NULL);

    for (n = 0; n < ARRAY_SIZE(ax620e_regbanks); n++) {
        DeviceState *dev = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(dev, "size", ax620e_regbanks[n].size);
        qdev_prop_set_string(dev, "name", ax620e_regbanks[n].name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                ax620e_regbanks[n].base, 1);
    }
    for (n = 0; n < ARRAY_SIZE(ax620e_unimp); n++) {
        create_unimplemented_device(ax620e_unimp[n].name,
                                    ax620e_unimp[n].base,
                                    ax620e_unimp[n].size);
    }
}

/*
 * emmc@1b40000/sd@104e0000/sdio@104d0000: create a cadence.sdhci
 * controller and, if the user passed a matching "-drive if=sd,index=N"
 * (or legacy "-sd"), plug a real SD card onto it so U-Boot's MMC probe
 * (env load/save, FAT/ext4 kernel+rootfs load) sees real media instead
 * of "Card did not respond to voltage select!". index 0 is the eMMC
 * ("mmc@1B40000: 0" per U-Boot's own device numbering at boot).
 *
 * The real "cdns,sd4hc" IP is UHS-I capable, and the real kernel's
 * generic sdhci_start_signal_voltage_switch() (drivers/mmc/host/
 * sdhci.c) gates its own SDHCI_HOST_CONTROL2 write on exactly that
 * (reading V18_ENA back afterward to confirm "the 1.8V regulator
 * output became stable" before returning success). QEMU's own
 * generic hw/sd/sdhci.c model gates writing that same register field
 * on its own "uhs" property, which defaults to UHS_NOT_SUPPORTED and
 * was never set here -- so every switch attempt's write was silently
 * dropped, the readback the kernel checks always came back 0, and
 * drivers/mmc/host/sdhci.c's own "did not become stable" retry fired
 * forever (confirmed live: a real, no-card-inserted slot otherwise
 * boots and idles fine -- U-Boot's own "mmc@104E0000: 1" probe never
 * needed this at all, since it never requests UHS speeds).
 */
static void axera_attach_sdhci(hwaddr base, qemu_irq irq, int unit,
                               bool emmc)
{
    DeviceState *dev = qdev_new(TYPE_CADENCE_SDHCI);
    CadenceSDHCIState *sdhci = CADENCE_SDHCI(dev);
    DriveInfo *di = drive_get(IF_SD, 0, unit);

    qdev_prop_set_uint8(DEVICE(&sdhci->sdhci), "uhs", UHS_I);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    if (di) {
        DeviceState *card = qdev_new(emmc ? TYPE_EMMC : TYPE_SD_CARD);

        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, sdhci->bus, &error_fatal);
    }
}

static const AxeraSoCConfig ax630c_soc = {
    .desc              = "Axera AX630C (2x Cortex-A53, AX620E family)",
    /*
     * The real Sipeed NanoKVM-Pro/MaixCam board this machine targets has
     * 2 GiB DRAM: U-Boot's own board/axera/ax620e_emmc/ax620e_emmc.c
     * dram_init() hardcodes gd->ram_size = 0x7FFFF000 regardless of what
     * QEMU actually backs, and board_f.c carves malloc/board-info/stack
     * down from that computed top -- with only 1 GiB of real RAM behind
     * it, those late reservations land past the end of backed memory and
     * U-Boot dies with a silent synchronous external abort in memset()
     * (called from reserve_board()) right after relocation.
     */
    .ram_size_default  = 2 * GiB,
    .uart0_base        = AXERA_UART0_BASE,
    .uart0_irq         = AXERA_UART0_IRQ,
};

/*
 * Same SoC family and the same GIC/timer/UART addresses as AX630C (the
 * dtsi's own compatible string is "axera,ax620e" either way); kept as a
 * separate machine because the real deployed profile differs -- a Bayer
 * sensor + ISP/NPU camera board (confirmed from a real Xiongmai IVG-A6 (AX620E)
 * firmware's own devicetree: 1 GiB DRAM at 0x40000000, isp@2400000
 * "axera,proton", ax_npu@0x3800000 "axera,npu") rather than AX630C's
 * HDMI-capture/KVM board shape.
 */
static const AxeraSoCConfig ax620e_soc = {
    .desc              = "Axera AX620E (2x Cortex-A53, IP-camera profile)",
    .ram_size_default  = 1 * GiB,
    .flash_is_nor      = true,
    .uart0_base        = AXERA_UART0_BASE,
    .uart0_irq         = AXERA_UART0_IRQ,
};

/*
 * CPU1's wake mailbox @ COMMON_SYS_GLB_BASE+0xDC/0xE0 ("COMM_SYS_DUMMY_
 * SW0/1_CORE1_LOW/HIGH" in the real ATF source) -- stands in for the
 * real hardware's own (software-invisible) power-gate release logic
 * that plat/axera/ax620e/ax620e_pwrc.c's ax620e_pwrc_write_pponr() (the
 * real ATF source under github/u-boot-axera) relies on: that function
 * does nothing more than write a target entry address into this exact
 * register pair and execute sev(), with no separate "release from
 * reset" register anywhere in this platform's own software-visible
 * driver tree. On the write that completes the handshake (the LOW
 * word, written second by the real code -- see the HIGH-only branch
 * below), this reconfigures CPU1's own "rvbar" to the address just
 * assembled and performs a real cpu_reset()-based release via
 * arm_set_cpu_on_and_reset(), landing it at EL3, freshly reset, same
 * as this SoC's own built-in PSCI-over-SMC emulation does for the
 * non-real-firmware boot paths elsewhere in this file. See the long
 * comment on CPU1's own "start-powered-off" property, in the CPU
 * creation loop below, for why this is needed at all.
 */
typedef struct AxeraCpu1Wake {
    ARMCPU *target;
    uint32_t low, high;
    bool parked;        /* CPU1 halted in BL1's wfe loop, see below */
    bool debug;         /* AXERA_SMP_DEBUG=1 */
} AxeraCpu1Wake;

/*
 * WFE for BL1's secondary wait loop (bl1/board/arch/arm64/start.S,
 * start_el1: zero the mailbox, then "2: wfe; ldr mailbox; cbz x4, 2b;
 * br x4" -- the kernel's spin-table enable-method writes
 * secondary_holding_pen's address here, cpu-release-addr). TCG runs
 * WFE as a plain yield, so CPU1 re-read this window ~3 million times a
 * second for the whole of U-Boot and the kernel's early boot, each read
 * an MMIO access under the BQL, starving CPU0 (one boot sat in U-Boot
 * for over six minutes). On silicon WFE sleeps until the SEV the
 * releasing CPU sends after writing the mailbox; model that: CPU1
 * reading an empty mailbox halts itself, and the write that fills the
 * mailbox wakes it. The loop's own code still runs unchanged -- woken,
 * CPU1 re-reads the address and takes its own "br x4" at EL1, exactly
 * as on hardware.
 */
static void axera_cpu1_park_work(CPUState *cs, run_on_cpu_data data)
{
    AxeraCpu1Wake *s = data.host_ptr;

    if (s->parked) {
        cs->halted = 1;
    }
}

static void axera_cpu1_unpark_work(CPUState *cs, run_on_cpu_data data)
{
    cs->halted = 0;
}

static uint64_t axera_cpu1_wake_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    AxeraCpu1Wake *s = opaque;
    CPUState *cs = CPU(s->target);

    if (current_cpu == cs && !s->low && !s->high && !s->parked) {
        s->parked = true;
        if (s->debug) {
            fprintf(stderr, "axera-smp: %.3f CPU1 parks in BL1 wfe loop\n",
                    g_get_monotonic_time() / 1e6);
        }
        async_run_on_cpu(cs, axera_cpu1_park_work, RUN_ON_CPU_HOST_PTR(s));
    }
    return offset == 0 ? s->low : s->high;
}

static void axera_cpu1_unpark(AxeraCpu1Wake *s)
{
    if (s->parked && (s->low || s->high)) {
        s->parked = false;
        async_run_on_cpu(CPU(s->target), axera_cpu1_unpark_work,
                         RUN_ON_CPU_NULL);
    }
}

static void axera_cpu1_wake_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    AxeraCpu1Wake *s = opaque;

    if (s->debug) {
        fprintf(stderr, "axera-smp: %.3f cpu%d writes %s=0x%" PRIx64
                " (CPU1 %s%s)\n", g_get_monotonic_time() / 1e6,
                current_cpu ? current_cpu->cpu_index : -1,
                offset ? "HIGH" : "LOW", value,
                s->target->power_state == PSCI_OFF ? "off" : "on",
                s->parked ? ", parked" : "");
    }
    if (offset == 4) {
        s->high = value;
        axera_cpu1_unpark(s);
        return;
    }

    s->low = value;
    axera_cpu1_unpark(s);
    if ((s->low || s->high) && s->target->power_state == PSCI_OFF) {
        uint64_t addr = ((uint64_t)s->high << 32) | s->low;

        /*
         * The real primary-path BL1 code writes this exact mailbox
         * pair very early for its own internal reasons unrelated to
         * actually needing CPU1 (confirmed live: a gdb watchpoint
         * caught CPU0 itself, from inside its own early init, writing
         * precisely AXERA_BL1_BASE here, long before the kernel's own
         * PSCI CPU_ON ever runs) -- but that release is NOT wasted:
         * skipping it (an earlier version of this code did, trying to
         * save one round trip) confirmed live that it is in fact
         * *necessary*. The kernel's own secondary_holding_pen/
         * secondary_startup/__enable_mmu sequence (what the real PSCI
         * CPU_ON target address, written later, actually points at)
         * assumes it is entered at EL1 (its own "bl el2_setup" call
         * checks CurrentEL and, finding anything other than EL2,
         * unconditionally assumes EL1 -- never EL3) and uses EL1-only
         * virtual addresses once "MMU enabled" without ever actually
         * enabling SCTLR_EL3's own MMU bit; skipping straight there
         * from a fresh EL3 reset faults immediately. This early write
         * is what gets CPU1 through BL1's own EL3->EL2->EL1 drop (the
         * same path confirmed correct earlier, single-stepped end to
         * end) *before* the real kernel address ever arrives, so that
         * by the time it does, CPU1 is already sitting at EL1 and can
         * reach it with a plain software branch out of BL1's own wfe
         * loop, exactly like a real spin-table release -- no second
         * arm_set_cpu_on_and_reset() needed or wanted for that part.
         */
        object_property_set_uint(OBJECT(s->target), "rvbar", addr,
                                 &error_abort);
        arm_set_cpu_on_and_reset(arm_cpu_mp_affinity(s->target));
    }
}

/*
 * System reset (the camera reboots itself once on first boot, and on
 * every watchdog/"reboot") has to put CPU1 back behind its power gate.
 * The flash-file= path never calls arm_load_kernel(), whose reset hook
 * is what normally resets every CPU; only CPU0 got
 * axera_bootrom_cpu_reset(). CPU1 thus kept running the previous
 * kernel (parked in its cpu_park_loop or idle) across the reboot,
 * power_state still ON, so the next BL1's release write was a no-op
 * and that boot's spin-table release found nobody in BL1's wfe loop.
 * cpu_reset() honours "start-powered-off": CPU1 is off and halted
 * again, ready for BL1 to release it as on a cold boot.
 */
static void axera_cpu1_wake_reset(void *opaque)
{
    AxeraCpu1Wake *s = opaque;

    cpu_reset(CPU(s->target));
    s->low = s->high = 0;
    s->parked = false;
}

static const MemoryRegionOps axera_cpu1_wake_ops = {
    .read = axera_cpu1_wake_read,
    .write = axera_cpu1_wake_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * The AX620E-die peripheral set behind the GIC: crypto, codecs, DMA,
 * watchdogs, RTC, SDHCI, I2C, VIN, EQoS and USB. AX630C, AX620E and
 * AX620Q are one die (all three boot the vendor's shared AX620E.dtsi,
 * with the same addresses and GIC SPIs -- confirmed against the
 * AX620Q NanoKVM-Go devicetree the Maix SDK's 4.19 kernel builds), so
 * the AArch64 and AArch32 machines share this one list. camera_board
 * adds the SC450AI sensor the real AX620E IP-camera board has on
 * i2c6; the NanoKVM boards have no Bayer sensor there.
 */
typedef struct {
    const char *name;
    hwaddr base;
    int irq;
    uint32_t status, ack, mode;
    uint32_t kick[4], done[4];
    uint32_t pixel;             /* pixel work, see axera-ivps-engine.c */
} AxeraIvpsEngineDesc;

/* IVPS processing engines, see hw/misc/axera-ivps-engine.c */
static void axera_create_ivps_engines(const AxeraIvpsEngineDesc *engines,
                                      int n, qemu_irq *pic)
{
    int e, k;

    for (e = 0; e < n; e++) {
        DeviceState *dev = qdev_new("axera-ivps-engine");

        qdev_prop_set_string(dev, "name", engines[e].name);
        qdev_prop_set_uint32(dev, "status-off", engines[e].status);
        qdev_prop_set_uint32(dev, "ack-off", engines[e].ack);
        qdev_prop_set_uint32(dev, "ack-mode", engines[e].mode);
        qdev_prop_set_uint32(dev, "pixel", engines[e].pixel);
        for (k = 0; k < 4; k++) {
            char prop[16];

            snprintf(prop, sizeof(prop), "kick-off%d", k);
            qdev_prop_set_uint32(dev, prop, engines[e].kick[k]);
            snprintf(prop, sizeof(prop), "done-bits%d", k);
            qdev_prop_set_uint32(dev, prop, engines[e].done[k]);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, engines[e].base);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[engines[e].irq]);
    }
}

/*
 * AX_DMAPER request id the SPI NOR controller's RX FIFO handshake is
 * routed to: BL1, U-Boot and the kernel all program reqmux field 0x01
 * for the SPI RX channel (ch0), matching the dts "dmas = <&dma_per 1
 * &dma_per 0>". Block-mode: see axera_dmaper_chn_blocked().
 */
#define AXERA_DMAPER_REQ_SPI_NOR_RX 1

/*
 * Returns AX_DMAPER, for the flash-file= SPI NOR to wire its request;
 * spi_nor_dma says that SPI NOR exists, so its request id is handshaked.
 */
static DeviceState *axera_ax620e_periphs(qemu_irq *pic, bool camera_board,
                                         bool vin_itp_lockstep,
                                         bool spi_nor_dma)
{
    /* the SC450AI, whose exposure scales axera-vin's 3A statistics */
    DeviceState *ax620e_sensor = NULL;
    DeviceState *dmaper_dev;

    /*
     * vpp@4403000 (SPI 0x67), gdc@4404000 (SPI 0x68), tdp@4405000
     * (SPI 0x63) and the vo@4407000 DPU (SPI 0x61, IVPS's OSD stage):
     * the IVPS engines. Their done interrupts are what move IVPS through its
     * per-tile command lists -- see hw/misc/axera-ivps-engine.c.
     */
    {
        static const AxeraIvpsEngineDesc engines[] = {
            /* v0, v1, isp, gdc-linked channels */
            { "axera-vpp", 0x04403000, 0x67, 0x888, 0x8b4, 0,
              { 0x004, 0x038, 0x06c, 0x07c },
              { 1u << 0, 1u << 3, 1u << 6, 1u << 9 }, 3 },
            { "axera-gdc", 0x04404000, 0x68, 0x0d8, 0x0d4, 1,
              { 0x06c }, { 1u << 16 }, 2 },
            { "axera-tdp", 0x04405000, 0x63, 0x004, 0x010, 0,
              { 0x01c }, { 1u << 0 } },
            /* vo@4407000 "dpu": the IVPS OSD/overlay stage */
            { "axera-dpu", 0x04407000, 0x61, 0x004, 0x014, 0,
              { 0x028 }, { 1u << 0 }, 4 },
        };

        axera_create_ivps_engines(engines, ARRAY_SIZE(engines), pic);
    }

    /* TIMER_BASE @ 0x04830000 (boot/bl1/driver/include/timer.h): BL1's
     * delay()/mdelay()/udelay() -- and a timer-ratio helper called from
     * a tight retry loop very early in boot -- read this free-running
     * down-counter twice and spin until it's advanced, with no timeout
     * of their own (see hw/misc/axera-bl1-timer.c). A passive regbank
     * can never satisfy that: two reads of inert storage are always
     * equal, so BL1 spins here forever before anything else runs.
     * Linux's ax_hrtimer@4830000 sleeps on timer 0's expiry interrupt,
     * GIC SPI 124 (see hw/misc/axera-bl1-timer.c).
     */
    sysbus_create_simple(TYPE_AXERA_BL1_TIMER, 0x04830000, pic[124]);

    /* crypto@4900000, "axera,ax630c-eip130", GIC_SPI 110. Real AES
     * ECB/CBC/CTR via qcrypto, not just a satisfied probe -- see
     * hw/misc/axera-eip130.c. */
    sysbus_create_simple(TYPE_AXERA_EIP130, 0x04900000, pic[110]);

    /* venc@4010000/jenc@4000000/vdec@4020000: the VeriSilicon VCMD
     * command-queue front-end (probe/submit/wait/complete handshake is
     * real). The command-buffer's own opcode stream is decoded for
     * real, and an actual encode job (H.264/HEVC/JPEG) is bridged to
     * the project's shared Hantro software encoder library to produce
     * a real bitstream -- see hw/misc/axera-vcmd.c. Decode (vdec) is
     * not bridged. */
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04010000, pic[93]);
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04000000, pic[91]);
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04020000, pic[92]);

    /*
     * AX_DMAPER @ 0x048a0000 (boot/bl1/driver/dmaper/dmaper.h), GIC
     * SPI 0x70: real BL1 stages a flash read into scratch RAM then
     * uses this LLI-descriptor DMA engine (axi_dma_xfer_start()/
     * axi_dma_wait_xfer_done()) to copy it into the image's final
     * destination. Also the real target of the real kernel's own
     * spi@1A00000 "dmas = <0x09 0x01 0x09 0x00>;" property (confirmed
     * by checking phandle 0x09's own definition in the decompiled
     * dtb, not just its proximity to the spi node in the source) --
     * BL1's own real axi_dma_wait_xfer_done() never waits on an
     * interrupt at all, just spins polling a status register
     * directly, so this device genuinely needed no IRQ line for that
     * consumer alone; but the real kernel's own dmaengine framework
     * needs a genuine interrupt to wake its wait_for_completion()
     * (confirmed live: the real DMAPER_START write does happen, the
     * transfer itself genuinely completes, but nothing ever signals
     * the sleeping kernel thread, so "DMA transaction timed out" fires
     * regardless) -- see hw/misc/axera-dmaper.c's own comment for the
     * full story and why raising this is harmless for BL1.
     */
    {
        /*
         * Audio: i2s_mst@6050000 ("axera,dwc-i2s-mst") and
         * i2s_slv@6051000 ("axera,dwc-i2s-slv"), Synopsys DW_apb_i2s
         * (see hw/misc/axera-i2s.c). The real devicetree gives them no
         * "interrupts" at all ("IRQ index 0 not found" is expected), so
         * dwc-i2s.c runs them through dmaengine on THIS controller:
         * dmas = <&dma_per 0x0f &dma_per 0x0e> (rx, tx) for i2s_mst and
         * <0x11 0x10> for i2s_slv. Those four request ids are the only
         * flow-controlled ones; the request-mux the driver programs
         * lives in the clk-periph syscon (DMAPER MMIO 1, mapped over
         * the passive "axera-clk-periph" regbank at a higher priority).
         * i2s_slv is linked with the inner codec as bit/frame master,
         * so it reports COMP1.MODE_EN=0 ("master"=false); i2s_mst needs
         * MODE_EN=1 for its CBS_CFS set_fmt to succeed.
         *
         * i2s_slv's frame clock comes from the actt codec, not from
         * hw_params, and runs at 16 kHz although ALSA is told 8000:
         * the camera's capture path (libax_audio, AI dev ChnCnt=2,
         * PeriodSize=160, /proc/ax_proc/ai) reads 320 frames per
         * 20 ms G.711 frame and decimates by two. At 8 kHz it got
         * 25 Readi/s, RTSP audio ran at half rate with a 400 Hz test
         * tone coming out at 800 Hz, and the A/V muxer starved video
         * (0 RTP video packets); at 16 kHz it is 50 Readi/s, 50 audio
         * packets/s with the tone at 400 Hz, and video streams.
         */
        static const struct {
            const char *name;
            hwaddr base;
            bool master;
            uint32_t rate;
            int rx_req, tx_req;
        } ax620e_i2s[] = {
            { "i2s_mst", 0x06050000, true,  8000,  0x0f, 0x0e },
            { "i2s_slv", 0x06051000, false, 16000, 0x11, 0x10 },
        };
        DeviceState *dmaper = qdev_new(TYPE_AXERA_DMAPER);
        uint64_t paced = 0;
        int i;

        for (i = 0; i < ARRAY_SIZE(ax620e_i2s); i++) {
            paced |= 1ULL << ax620e_i2s[i].rx_req;
            paced |= 1ULL << ax620e_i2s[i].tx_req;
        }
        qdev_prop_set_uint64(dmaper, "paced-reqs", paced);
        if (spi_nor_dma) {
            qdev_prop_set_uint64(dmaper, "block-reqs",
                                 1ULL << AXERA_DMAPER_REQ_SPI_NOR_RX);
        }
        dmaper_dev = dmaper;
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dmaper), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dmaper), 0, 0x048a0000);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dmaper), 1, 0x04870130, 2);
        sysbus_connect_irq(SYS_BUS_DEVICE(dmaper), 0, pic[0x70]);

        for (i = 0; i < ARRAY_SIZE(ax620e_i2s); i++) {
            DeviceState *i2s = qdev_new(TYPE_AXERA_I2S);

            qdev_prop_set_string(i2s, "name", ax620e_i2s[i].name);
            qdev_prop_set_bit(i2s, "master", ax620e_i2s[i].master);
            qdev_prop_set_uint32(i2s, "rate", ax620e_i2s[i].rate);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(i2s), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(i2s), 0,
                                    ax620e_i2s[i].base, 2);
            qdev_connect_gpio_out_named(i2s, AXERA_I2S_DMA_RX_REQ, 0,
                qdev_get_gpio_in_named(dmaper, AXERA_DMAPER_REQ,
                                       ax620e_i2s[i].rx_req));
            qdev_connect_gpio_out_named(i2s, AXERA_I2S_DMA_TX_REQ, 0,
                qdev_get_gpio_in_named(dmaper, AXERA_DMAPER_REQ,
                                       ax620e_i2s[i].tx_req));
        }
    }

    /*
     * dw_dmac@48b0000, "axera,axi-dma-1.01a", GIC SPI 0x71: a real
     * Synopsys DesignWare AXI DMA Controller, a separate physical IP
     * from AX_DMAPER above -- kept modelled since it's a real IP
     * block this SoC genuinely has (some other, not yet identified
     * consumer may legitimately need it), even though it turned out
     * NOT to be what the kernel's SPI-NOR driver actually uses -- see
     * hw/misc/axera-axi-dmac.c's own comment for that story.
     */
    sysbus_create_simple(TYPE_AXERA_AXI_DMAC, 0x048b0000, pic[0x71]);

    /*
     * watchdog@4840000, "axera,ax-wdt", GIC SPI 0x95: a real Synopsys
     * DW_APB_WDT, confirmed (via its own IRQ-less reset-mode kick
     * sequence, no PSCI/syscon-reboot node anywhere in the real
     * devicetree, and a live trace showing zero PSCI SMC calls across
     * an entire boot+reboot cycle) to be the real reboot mechanism on
     * this SoC -- see hw/misc/axera-wdt.c's own comment for the full
     * story and why a passive regbank here left reboot(RB_AUTOBOOT)
     * hanging forever in the kernel's own generic machine_restart()
     * fallback loop.
     */
    sysbus_create_simple(TYPE_AXERA_WDT, 0x04840000, pic[0x95]);

    /*
     * watchdog@6040000 (wdt2), the SAME "axera,ax-wdt" IP as wdt0
     * above, GIC SPI 150 (the vendor SDK's own AX620E.dtsi marks both
     * wdt0 and wdt2 "status = disabled" -- clearly overridden in this
     * real dump's own compiled-in DTB, since dmesg shows
     * "ax_wdt_drv_probe:347: done!" TWICE on every boot, not once).
     * Previously unwired, confirmed live via a fresh '-d unimp'
     * survey as this boot's single largest remaining offender (offset
     * +0x30 relative to this base, 32 hits in one run) -- some
     * runtime register this second instance's own keep-alive/refresh
     * path reads or writes repeatedly, not just a one-off probe-time
     * access.
     */
    sysbus_create_simple(TYPE_AXERA_WDT, 0x06040000, pic[0x96]);

    /*
     * rtc-top@23E0000, "axera,axi-top-rtc" (a DesignWare RTC core --
     * see hw/rtc/axera-rtc.c's own header comment, written from the
     * real, open drivers/rtc/rtc-axera.c vendor source), GIC SPI 0x6b.
     * reg-num=2: the second range (0x2340000) is just a clock-enable
     * bit (CLK_CONFIG_ADDR, offset 0x6C) the driver toggles through a
     * separate ioremap -- a hisi-regbank write-then-read is enough for
     * that one, so only the main register window gets the real model.
     */
    sysbus_create_simple("axera-rtc", 0x023e0000, pic[0x6b]);
    {
        DeviceState *rtc_clk = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(rtc_clk, "size", 0x1000);
        qdev_prop_set_string(rtc_clk, "name", "axera-rtc-clk");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rtc_clk), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(rtc_clk), 0, 0x02340000, 1);
    }

    /*
     * emmc@1b40000/sd@104e0000/sdio@104d0000, "axera,sdhc","cdns,sd4hc":
     * QEMU's own generic cadence.sdhci model, a real off-the-shelf
     * device (not Axera-specific) that already implements this exact
     * compatible string's register set. See axera_attach_sdhci() for
     * the real-media wiring ("-drive if=sd,index=0" is the eMMC).
     */
    axera_attach_sdhci(0x01b40000, pic[9], 0, true);
    axera_attach_sdhci(0x104e0000, pic[74], 1, false);
    axera_attach_sdhci(0x104d0000, pic[73], 2, false);

    /*
     * i2c@4852000 (i2c2) and i2c@4856000 (i2c6), "snps,designware-i2c",
     * GIC SPI 0x89/0x8d: of this SoC's eight i2c0-i2c7 controller
     * instances, the real devicetree has only these two actually
     * enabled (every other one is "status = disabled", confirmed by
     * reading each node directly) -- and both already printed
     * "Unknown Synopsys component type: 0x00000000" during normal
     * boot before this (a completely unmapped MMIO region reads back
     * all-zero, including the real driver's own IP-identification
     * register it checks). i2c6 specifically is what this dump's own
     * vendor userspace camera/ISP pipeline opens directly as
     * /dev/i2c-6 to talk to the real image sensor -- confirmed by
     * this same boot's own "Load_AX620E -i imx307" invocation (the
     * real devicetree has no sensor child node at all under this
     * i2c bus; the vendor's own userspace ISP library owns the
     * sensor probe/config entirely, not the kernel). With no device
     * node for /dev/i2c-6 ever created (this controller never having
     * probed successfully at all), the real vendor binary's own
     * open() failure cascaded into a guest-side SIGBUS ("Bus error")
     * in its video pipeline shortly after.
     *
     * An earlier attempt wired this up with hisi-i2c-dw, but that
     * model (despite its name) implements a different, HiSilicon-
     * specific "AUTO mode" single-shot-transaction register protocol
     * (see its own file header) for the drivers/i2c/busses/
     * i2c-hisilicon.c vendor driver on CV200-family SoCs -- it is NOT
     * register-compatible with the real mainline i2c-designware-*.c
     * driver this AX620E kernel actually uses, which still read back
     * "Unknown Synopsys component type: 0x00000000" from
     * DW_IC_COMP_TYPE (offset 0xfc) against hisi-i2c-dw and bailed
     * out of i2c_dw_set_reg_access() with -ENODEV before ever
     * creating /dev/i2c-N, confirmed live. dw-i2c is a new, generic
     * DW_IC_* register model (CON/TAR/DATA_CMD FIFO semantics,
     * COMP_TYPE/COMP_VERSION/COMP_PARAM_1 included) written
     * specifically to satisfy that real driver; hisi-imx307 (a real
     * IMX307 stub already written for this exact Xiongmai probe
     * sequence -- see its own comment) is a plain I2CSlave and works
     * unchanged on either master model. The sensor's own I2C address,
     * 0x1a, is the industry-standard Sony STARVIS address (confirmed
     * already assumed correct elsewhere in this project, for the
     * same sensor family, on a different Xiongmai-derived SoC).
     */
    sysbus_create_simple("dw-i2c", 0x04852000, pic[0x89]);

    /*
     * i2c@4850000 (i2c0) and i2c@4857000 (i2c7), GIC SPI 0x87/0x8e:
     * these two were plain create_unimplemented_device catch-all stubs
     * (ax620e_unimp[]) until now, exactly like i2c2/i2c6 used to be --
     * but unlike i2c2/i2c6, this exact dump's real devicetree node for
     * BOTH of these has "status = disabled" (confirmed by reading the
     * node directly, same extraction this file's own i2c2/i2c6 comment
     * above already relies on), so the guest's i2c-designware platform
     * driver never even calls probe() on either one -- no MMIO access
     * of any kind reaches these two addresses anywhere in a live boot,
     * confirmed empirically with a `-d unimp` run all the way through
     * kernel boot, rootfs mount and the real camera App's own start
     * (Load_AX620E): the unimp log for this exact dump has zero hits
     * on either address, before or after this fix. Neither U-Boot
     * (whose AX620E defconfigs don't even enable CONFIG_I2C) nor BL1
     * (no I2C driver exists anywhere in this platform's own boot/bl1
     * tree) touch them either. So this fix is "no longer a bare
     * catch-all stub, now a real DW_IC_* register model" for
     * correctness/consistency with every other real IP block in this
     * file, not because anything observed actually depends on it --
     * if a different dump's devicetree ever flips either node to
     * "status = okay" (or a future BL1/U-Boot image starts using
     * CONFIG_I2C), this is already the right model to answer it,
     * instead of silently returning zero from a generic stub the way
     * i2c2/i2c6 themselves used to (which caused a real guest-visible
     * "Unknown Synopsys component type" probe failure for those two).
     * No downstream slave is attached to either bus: nothing in this
     * dump's kernel, U-Boot, BL1, or App ever probes an address on
     * them, so there is nothing for a stand-in sensor/PMIC to answer.
     */
    sysbus_create_simple("dw-i2c", 0x04850000, pic[0x87]);
    sysbus_create_simple("dw-i2c", 0x04857000, pic[0x8e]);

    {
        DeviceState *i2c6 = sysbus_create_simple("dw-i2c", 0x04856000,
                                                 pic[0x8d]);
        I2CBus *i2c6_bus = I2C_BUS(qdev_get_child_bus(i2c6, "i2c"));

        /*
         * Confirmed live: the real vendor userspace ISP's own sensor
         * auto-probe never actually tries address 0x1a (where a
         * hisi-imx307 was first attached, based solely on this dump's
         * "-i imx307" launch argument turning out to just be a
         * generic/default argument, not a real indicator) -- it
         * cycles through 0x36/0x30/0x10 looking for a response and
         * never reaches 0x1a within many repeated passes. 0x30 is
         * this tree's own "Sofia (XiongMai) detects these sensors at
         * I2C addr 0x60 (8-bit) = 0x30 (7-bit)" SmartSens SC* family
         * address (see hisi-smartsens.c) -- and this exact dump's own
         * extracted ai_isp .axmodel filename independently confirms
         * the real deployed sensor is a SmartSens SC450AI, not a Sony
         * IMX307 at all. The real id_high/id_low pair (0xBD/0x2F, from
         * SC450AI_SENSOR_ID = 0xbd2f at registers 0x3107/0x3108,
         * maix_ax620e_sdk/msp/component/isp_proton/sensor/
         * smartsens_sc450ai/sc450ai_reg.h) replaces an earlier sc307h
         * stand-in (0xCB/0x1C): with the wrong ID, the app's own
         * sensor-name lookup misidentified the chip as "sc200ai" and
         * then failed outright trying to dlopen a /usr/lib/
         * libsns_sc200ai.so this rootfs never ships (only
         * libsns_sc450ai.so is present) -- confirmed live via
         * "xmcap_sensor.c ... open /usr/lib/libsns_sc200ai.so fail!"
         * aborting IspVi_create before it ever reaches real VIN/WDMA
         * setup. The correct ID lets the app's own lookup resolve to
         * "sc450ai" and successfully load the .so that's actually
         * there.
         */
        if (camera_board) {
            DeviceState *sensor = qdev_new("hisi-smartsens");

            ax620e_sensor = sensor;

            qdev_prop_set_uint8(sensor, "id_high", 0xBD);
            qdev_prop_set_uint8(sensor, "id_low", 0x2F);
            qdev_prop_set_uint8(sensor, "address", 0x30);
            qdev_realize_and_unref(sensor, BUS(i2c6_bus), &error_fatal);
        }
    }

    /*
     * isp@2400000, "axera,proton", GIC SPI 27/28 ("isp0-0"/"isp0-1"):
     * the real VIN/ISP frame-completion interrupt -- see
     * hw/misc/axera-vin.c's own header comment for the full discovery
     * (live gdbstub trace through four layers of the real ax_proton.ko
     * down to the exact register map) and for what's still an assumption
     * versus confirmed. Previously a passive axera-vin regbank (moved
     * here from axera_map_regbanks() since, like AX_DMAPER above, it
     * needs pic[] at creation time): that left the real kernel driver's
     * frame-done wait permanently timing out, which is why the camera
     * app never stopped logging AX_ERR_IVPS_BUF_EMPTY and, almost
     * certainly, why it never called accept() on its RTSP socket either.
     */
    {
        DeviceState *vin = qdev_new("axera-vin");

        /* the sensor picture: see "Sensor picture" in axera-vin.c */
        qdev_prop_set_uint32(vin, "isp-layout", 1);
        qdev_prop_set_uint32(vin, "cdma-list-off", 0x1004);
        /* CDMA lists arm the range-1 interrupts (YUV WDMA done, AF done) */
        qdev_prop_set_bit(vin, "cdma-irq-regs", true);
        /* ...and, with CPU1 online, the ITP's done events follow its
         * job and the next frame waits for them: see
         * axera_vin_itp_poll() */
        qdev_prop_set_bit(vin, "itp-lockstep", vin_itp_lockstep);
        /* 3A statistics: see axera_vin_ae_stats() */
        if (ax620e_sensor) {
            object_property_set_link(OBJECT(vin), "sensor",
                                     OBJECT(ax620e_sensor), &error_fatal);
            /*
             * The scene reads as painted at 118 rows (~2.5 ms) at unity
             * gain: the App's AE (set point 32) settles at ~69 rows,
             * well short of the night threshold (nightExp 12518971).
             */
            qdev_prop_set_uint64(vin, "ae-ref", 121000);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(vin), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(vin), 0, 0x02400000);
        sysbus_connect_irq(SYS_BUS_DEVICE(vin), 0, pic[0x1b]);
        sysbus_connect_irq(SYS_BUS_DEVICE(vin), 1, pic[0x1c]);
    }

    /*
     * ethernet@104c0000, "axera,dwmac-4.10a", GIC SPI 0x50 ("macirq"):
     * a real Synopsys DWC EQoS / dwmac-4.10a MAC+DMA core with an
     * external RMII PHY, confirmed by the real devicetree's own
     * clocks/resets/phy-handle properties -- see hw/net/axera-eqos.c's
     * own header comment for why its register map is identical to
     * hw/net/nvt-eqos.c's (same underlying Synopsys IP generation,
     * just a different vendor glue/PHY). Previously left as a plain
     * unimplemented register stub ("axera-eth0"), which let U-Boot's
     * own probe see *something* respond but gave the Linux guest no
     * way to ever bring an interface up -- confirmed live, no eth0
     * IP configuration of any kind ever appeared in a full boot's
     * console output against the stub.
     */
    {
        DeviceState *eth = qdev_new("axera-eqos");

        qemu_configure_nic_device(eth, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(eth), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(eth), 0, 0x104c0000);
        sysbus_connect_irq(SYS_BUS_DEVICE(eth), 0, pic[0x50]);
    }

    /*
     * usb2@8000000, "snps,dwc3": QEMU's generic usb_dwc3 model (also
     * used by Xilinx ZynqMP boards for the same Synopsys IP). Upstream
     * does not expose its internal XHCI IRQ as a sysbus output either
     * (confirmed: no sysbus_init_irq in hcd-dwc3.c's realize, and
     * xlnx-zynqmp.c's own instantiation connects none) -- register-level
     * access works, interrupt-driven transfers do not, a limitation of
     * the upstream model itself, not introduced here.
     */
    {
        DeviceState *dwc3 = qdev_new(TYPE_USB_DWC3);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(dwc3), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dwc3), 0, 0x08000000);
    }

    return dmaper_dev;
}

static void axera_a64_init(MachineState *machine, const AxeraSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    unsigned int smp_cpus = machine->smp.cpus;
    static struct arm_boot_info binfo;
    qemu_irq pic[AXERA_NUM_SPI];
    DeviceState *gic;
    DeviceState *dmaper;
    bool spi_nor;
    SysBusDevice *gicbus;
    ARMCPU *cpu[2];
    int n, i;

    if (smp_cpus > ARRAY_SIZE(cpu)) {
        error_report("axera: at most %zu CPUs", ARRAY_SIZE(cpu));
        exit(1);
    }

    /*
     * flash-file= boots the real BL1, which genuinely starts at EL3
     * (its own reset code touches scr_el3 as its first instructions,
     * confirmed from a real deployed camera's own SPI NOR dump) -- a
     * real EL3 has to exist on the CPU model for that, unlike the
     * uboot-file=/-kernel paths below, which deliberately disable it
     * so QEMU's own internal PSCI-over-SMC emulation answers CPU_ON
     * directly (no real ATF/BL31 modelled at all for those paths).
     */
    bool want_el3 = AXERA_MACHINE(machine)->flash_file &&
                    AXERA_MACHINE(machine)->flash_file[0] &&
                    !machine->kernel_filename;

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        /*
         * The vendor ATF hands the kernel EL1 and answers PSCI over SMC
         * (the dtsi's psci node: method = "smc"); CPU1 comes up by
         * CPU_ON, per the comment above that node about the AX630C
         * firmware's spin-table-less boot. plat/axera/ax620e/ax620e_pwrc.c's
         * ax620e_pwrc_write_pponr() (confirmed from the real ATF source
         * under github/u-boot-axera) does nothing more than write a
         * target entry address into a shared scratch register pair and
         * execute sev() -- no separate hardware "release from reset"
         * register exists anywhere in this platform's own software-
         * visible driver tree (pmu.h's MODULE_CPU state covers the
         * whole cluster, which is trivially already on). On real
         * silicon this only works because CPU1 genuinely sits power-
         * gated from chip reset and the hardware's own (software-
         * invisible) wake logic, watching this exact mailbox pair,
         * physically releases it straight into EL3 at the address just
         * written -- exactly like a fresh cold boot, just with a
         * dynamically supplied target instead of the real BL1's own
         * fixed entry point. An earlier attempt here instead let CPU1
         * free-run from reset and rely on its own "br" out of a
         * software wfe-loop to reach that same target; confirmed live
         * (gdb + a non-interactive "-d exec" trace) that this fails,
         * because by the time CPU1 takes that branch it has *already*
         * dropped to EL1 as part of its own BL1 cold-boot path, so the
         * jump lands on an EL3-only register access (the real entry
         * point's own first instruction) at the wrong privilege level
         * -- an instant trap, zero further instructions executed,
         * which is exactly the "CPU1: failed to come online" symptom.
         * Marking it QEMU's own "start-powered-off" here (matching
         * real hardware's own power-gated-until-woken reality) is
         * correct, with axera_cpu1_wake_write() (a small MMIO overlay
         * wired in below, right after axera_map_regbanks()) standing
         * in for the real hardware wake logic: it watches that exact
         * mailbox pair and, on the write that completes the handshake,
         * reconfigures CPU1's own "rvbar" to the address just
         * assembled and performs a real cpu_reset()-based release via
         * arm_set_cpu_on_and_reset() -- the same QEMU API this SoC's
         * own built-in PSCI-over-SMC emulation uses for the non-real-
         * firmware boot paths elsewhere in this file -- so CPU1 always
         * lands at EL3, freshly reset, at whatever address was
         * actually requested. (This has to live here, in per-target
         * hw/arm/ code that already pulls in target/arm/cpu.h, rather
         * than as a separate hw/misc device: those files build once
         * into the target-independent "common code" library, which is
         * structurally forbidden from including any target/arm header
         * at all -- confirmed by trying exactly that and hitting
         * cpu-defs.h's own "#error cpu.h included from common code".)
         */
        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                      &error_fatal);
        }
        object_property_set_bool(cpuobj, "has_el3", want_el3, &error_fatal);
        /*
         * The real BL1 also does its own genuine EL3->EL2 drop as part
         * of bring-up (confirmed live: its second instruction at EL2,
         * "mrs x0, cnthctl_el2", traps as undefined the moment EL2 is
         * disabled) before continuing on to DDR/flash init -- needs a
         * real EL2 for the same want_el3 cases that need a real EL3.
         */
        object_property_set_bool(cpuobj, "has_el2", want_el3, &error_fatal);
        if (!want_el3) {
            object_property_set_int(cpuobj, "psci-conduit",
                                     QEMU_PSCI_CONDUIT_SMC, &error_fatal);
        }
        object_property_set_uint(cpuobj, "cntfrq", AXERA_CNTFRQ,
                                  &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }

    memory_region_add_subregion(sysmem, AXERA_RAM_BASE, machine->ram);

    /*
     * axera-npu-sram @ 0x03000000, 0x280000: real on-chip SRAM, not an
     * NPU-only scratchpad -- BL1 itself both RUNS from inside this
     * window (its own linker script: ORIGIN=0x03000400 within this
     * exact range, see AXERA_BL1_BASE below) AND uses the rest of it
     * as general early-boot scratch, e.g. DDRINIT_PARAM_HEADER_BASE=
     * 0x03200000 for the DRAM-controller init parameters BL1 reads
     * off flash before DRAM itself exists to hold them anywhere else.
     * Modelling this as create_unimplemented_device (silently
     * discarding writes) broke that: the eMMC block-read into
     * 0x03200000 appeared to succeed but the destination stayed all
     * zero, and verify_img_header() correctly rejected it as garbage.
     * Real SRAM is real RAM; NPU compute itself using it is still
     * unmodelled (future work), but passive storage is what every
     * other consumer of this address range actually needs.
     */
    MemoryRegion *npu_sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(npu_sram, NULL, "axera.npu-sram",
                           0x280000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x03000000ULL, npu_sram);

    /*
     * axera-iram0 @ 0x0, 0x1000: a separate small on-chip SRAM from
     * axera-npu-sram above (confirmed: boot/bl1/common/include/boot.h's
     * BOOT_MODE_INFO_ADDR/MISC_INFO_ADDR/DDR_INFO_ADDR -- 0x700/0x740/
     * 0x800 -- are literal absolute addresses the real SoC's own
     * memory map keeps apart from SYS_OCM_BASE's 0x3000000). BL1's
     * setup_boot_mode() (spl_main.c) writes a boot_mode_info_t handoff
     * struct here (magic 0x12345678 + mode/channel/storage/type
     * fields) that U-Boot's own get_dl_and_boot_info() (arch/arm/
     * mach-axera/ax620e/ax620e.c) reads back by the same fixed address
     * -- the exact same "create_unimplemented_device silently discards
     * the write" trap npu_sram above already had to avoid, confirmed
     * live by boot_mode->magic reading back 0 instead of 0x12345678.
     */
    MemoryRegion *iram0 = g_new(MemoryRegion, 1);
    memory_region_init_ram(iram0, NULL, "axera.iram0", 0x1000, &error_fatal);
    memory_region_add_subregion(sysmem, 0x0ULL, iram0);

    create_unimplemented_device("axera-periph", AXERA_PERIPH_BASE,
                                 AXERA_PERIPH_SIZE);
    axera_map_regbanks();

    /*
     * axera_cpu1_wake_write()'s own MMIO window, standing in for the
     * real hardware wake logic for CPU1's power-gate -- see its own
     * comment above for the full story. Overlaid on top of the
     * "axera-clk-common" regbank axera_map_regbanks() just mapped at
     * priority 1, so this exact 8-byte window (COMM_SYS_DUMMY_SW0/1_
     * CORE1_LOW/HIGH, COMMON_SYS_GLB_BASE+0xDC/0xE0) takes precedence
     * while the rest of that regbank's address range still falls
     * through underneath unaffected.
     */
    if (want_el3 && smp_cpus > 1) {
        AxeraCpu1Wake *wake = g_new0(AxeraCpu1Wake, 1);
        MemoryRegion *wake_mr = g_new(MemoryRegion, 1);

        wake->target = cpu[1];
        wake->debug = getenv("AXERA_SMP_DEBUG") != NULL;
        qemu_register_reset(axera_cpu1_wake_reset, wake);
        memory_region_init_io(wake_mr, OBJECT(machine), &axera_cpu1_wake_ops,
                              wake, "axera.cpu1-wake", 8);
        memory_region_add_subregion_overlap(sysmem, 0x023400dcULL, wake_mr, 2);
    }

    /*
     * GIC-400, distributor and CPU interface only: nothing modelled yet
     * uses the virtualization frames at +0x4000/+0x6000. Security
     * extensions on, like novatek's nt98332 GIC-400 -- BL31 resets every
     * interrupt to group 1 on silicon, and a -kernel boot gets the same
     * through arm_load_kernel()'s secure_boot handling.
     */
    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(gic, "num-irq", AXERA_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    if (!machine->kernel_filename) {
        ARM_GIC_COMMON(gic)->irq_reset_nonsecure = true;
    }
    sysbus_mmio_map(gicbus, 0, AXERA_GIC_DIST);
    sysbus_mmio_map(gicbus, 1, AXERA_GIC_CPU);

    for (n = 0; n < smp_cpus; n++) {
        DeviceState *cpudev = DEVICE(cpu[n]);
        int ppibase = AXERA_NUM_SPI + n * GIC_INTERNAL + GIC_NR_SGIS;
        /*
         * The dtsi's timer PPIs, active-low on this GIC (13 secure, 14
         * non-secure, 11 virtual, 10 hypervisor) -- the usual INTIDs,
         * just not the usual polarity.
         */
        static const int timer_ppi[] = {
            [GTIMER_PHYS] = 14,
            [GTIMER_VIRT] = 11,
            [GTIMER_HYP]  = 10,
            [GTIMER_SEC]  = 13,
        };

        for (i = 0; i < ARRAY_SIZE(timer_ppi); i++) {
            qdev_connect_gpio_out(cpudev, i,
                                   qdev_get_gpio_in(gic, ppibase +
                                                     timer_ppi[i]));
        }
        sysbus_connect_irq(gicbus, n,
                            qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbus, n + smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbus, n + 2 * smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbus, n + 3 * smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    for (n = 0; n < AXERA_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    serial_mm_init(sysmem, c->uart0_base, 2, pic[c->uart0_irq],
                   AXERA_UART0_FREQ / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    spi_nor = AXERA_MACHINE(machine)->flash_file &&
              AXERA_MACHINE(machine)->flash_file[0] &&
              !machine->kernel_filename;
    dmaper = axera_ax620e_periphs(pic, true, c == &ax620e_soc && smp_cpus > 1,
                                  spi_nor);

    {
        AxeraMachineState *ams = AXERA_MACHINE(machine);

        /*
         * Back the real SPI NOR boot flash whenever a real dump is
         * given, whether or not BL1 itself is also being replayed:
         * uboot-file= + flash-file= together means "skip straight to
         * this real U-Boot binary (same trade-off as the uboot-file=
         * -only case below), but let it read the REST of this real
         * flash dump (kernel, rootfs) via its own real spinor_read()",
         * instead of needing BL1's own DDR training to run at all.
         */
        if (spi_nor) {
            DeviceState *nor = qdev_new(TYPE_AXERA_SPI_NOR);

            qdev_prop_set_string(nor, "file", ams->flash_file);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(nor), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(nor), 0, 0x01a00000);
            qdev_connect_gpio_out_named(nor, AXERA_SPI_NOR_DMA_RX_REQ, 0,
                qdev_get_gpio_in_named(dmaper, AXERA_DMAPER_REQ,
                                       AXERA_DMAPER_REQ_SPI_NOR_RX));
        }

        if (ams->uboot_file && ams->uboot_file[0] && !machine->kernel_filename) {
            /*
             * Start straight at U-Boot instead of replaying BL1/SPL's
             * own raw PLL/DDR bring-up: that sequence is real silicon
             * register bit-banging (see hw/misc/axera-pllc.c's own
             * comment) with more stages than the one PLL handshake
             * already modelled, and QEMU's RAM is "trained" by
             * construction -- there is nothing for a DDR controller
             * model to do here that isn't already done. This is the
             * same trade-off this tree's own HiSilicon V5 aarch64 path
             * already makes (locating u-boot-z.bin inside the image and
             * starting CPU0 there, not replaying the real boot ROM).
             * CONFIG_SYS_TEXT_BASE is within the normal DRAM window
             * mapped above, so no separate RAM region is needed here.
             */
            char *uboot_data = NULL;
            gsize uboot_len = 0;

            if (!g_file_get_contents(ams->uboot_file, &uboot_data,
                                     &uboot_len, NULL)) {
                error_report("axera: %s: could not read uboot-file '%s'",
                              MACHINE_GET_CLASS(machine)->name,
                              ams->uboot_file);
                exit(1);
            }

            rom_add_blob_fixed("axera.uboot", uboot_data, uboot_len,
                               AXERA_UBOOT_BASE);
            g_free(uboot_data);

            AxeraBootromReset *info = g_new0(AxeraBootromReset, 1);
            info->cpu = cpu[0];
            info->entry = AXERA_UBOOT_BASE;
            qemu_register_reset(axera_bootrom_cpu_reset, info);
            return;
        }

        if (ams->flash_file && ams->flash_file[0] && !machine->kernel_filename) {
            /*
             * Flash boot: BL1/SPL occupies offset 0 of a real dumped
             * flash/NAND image (confirmed: a leaked debug-info path
             * string "AX620E_SDK/boot/bl1/spl" and board/start.S sit
             * right at the start of a real Xiongmai-built AX620E
             * firmware dump). Inject it verbatim at its real link
             * address and start CPU0 there -- same technique this tree
             * already uses for HiSilicon's V1/V5 boot-ROM paths
             * (hisilicon_maskrom_cpu_reset), just a different address.
             * 0x03000400 is real RAM already -- it's inside the
             * axera-npu-sram region added above, not a dedicated
             * carve-out -- so this is just the blob write.
             *
             * Some real dumps (confirmed: a real deployed AX620E SPI
             * NOR camera firmware) have the real vendor sec_boot RSA
             * image_header (build/tools/imgsign/sec_boot_AX620E_sign.py)
             * in front of BL1 at offset 0 -- magic 0x55543322 at +4,
             * the real payload size at +0xc, a 1024-byte header. Detect
             * and skip it so the CPU starts on real BL1 code, not
             * header bytes; a dump with no such header (BL1 truly raw
             * at offset 0) is unaffected, same as before.
             */
            char *bl1_data = NULL;
            gsize bl1_len = 0;
            gsize bl1_off = 0;

            if (!g_file_get_contents(ams->flash_file, &bl1_data, &bl1_len,
                                     NULL)) {
                error_report("axera: %s: could not read flash-file '%s'",
                              MACHINE_GET_CLASS(machine)->name,
                              ams->flash_file);
                exit(1);
            }

            if (bl1_len >= 1024 &&
                ldl_le_p(bl1_data + 4) == 0x55543322) {
                bl1_off = 1024;
            }

            /*
             * spi0@0x01a00000 (BL1's own drivers/spi/dw_spi_mst.c reads
             * the rest of this same NOR flash through, real opcode 0x03
             * reads) is already backed by this same file -- see the
             * shared block above, which runs whether or not uboot-file=
             * is also given.
             */
            rom_add_blob_fixed("axera.bl1",
                               bl1_data + bl1_off,
                               MIN(bl1_len - bl1_off, (gsize)AXERA_BL1_SIZE),
                               AXERA_BL1_BASE);
            g_free(bl1_data);

            AxeraBootromReset *info = g_new0(AxeraBootromReset, 1);
            info->cpu = cpu[0];
            info->entry = AXERA_BL1_BASE;
            qemu_register_reset(axera_bootrom_cpu_reset, info);
            qemu_register_reset(axera_bl1_chipmode_reset,
                                (void *)(uintptr_t)c->flash_is_nor);
            return;
        }
    }

    if (machine->kernel_filename && !machine->dtb) {
        error_report("axera: %s needs the kernel's device tree (-dtb)",
                      MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AXERA_RAM_BASE;
    binfo.psci_conduit = QEMU_PSCI_CONDUIT_SMC;
    arm_load_kernel(cpu[0], machine, &binfo);
}

static const TypeInfo axera_machine_info = {
    .name           = TYPE_AXERA_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(AxeraMachineState),
    .interfaces     = arm_machine_interfaces,
};

static void axera_machine_register_types(void)
{
    type_register_static(&axera_machine_info);
}

type_init(axera_machine_register_types)

static char *axera_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(AXERA_MACHINE(obj)->flash_file);
}

static void axera_machine_set_flash_file(Object *obj, const char *value,
                                         Error **errp)
{
    AxeraMachineState *s = AXERA_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *axera_machine_get_uboot_file(Object *obj, Error **errp)
{
    return g_strdup(AXERA_MACHINE(obj)->uboot_file);
}

static void axera_machine_set_uboot_file(Object *obj, const char *value,
                                         Error **errp)
{
    AxeraMachineState *s = AXERA_MACHINE(obj);

    g_free(s->uboot_file);
    s->uboot_file = g_strdup(value);
}

#define DEFINE_AXERA_A64_MACHINE(namestr, tag, config)                \
    static void tag##_init(MachineState *machine)                     \
    {                                                                 \
        axera_a64_init(machine, &config);                             \
    }                                                                 \
    static void tag##_class_init(MachineClass *mc)                    \
    {                                                                 \
        ObjectClass *oc = OBJECT_CLASS(mc);                           \
        mc->desc = config.desc;                                       \
        mc->init = tag##_init;                                        \
        mc->default_ram_size = config.ram_size_default;                \
        mc->default_ram_id = "axera.ram";                              \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");        \
        mc->max_cpus = 2;                                              \
        mc->default_cpus = 2;                                         \
        object_class_property_add_str(oc, "flash-file",               \
            axera_machine_get_flash_file,                             \
            axera_machine_set_flash_file);                            \
        object_class_property_set_description(oc, "flash-file",       \
            "Raw flash/NAND dump; BL1/SPL at offset 0 is injected at " \
            "its real link address (0x03000400) and CPU0 starts "     \
            "there, with no -kernel.");                                \
        object_class_property_add_str(oc, "uboot-file",                \
            axera_machine_get_uboot_file,                              \
            axera_machine_set_uboot_file);                             \
        object_class_property_set_description(oc, "uboot-file",       \
            "A real U-Boot binary (CONFIG_SYS_TEXT_BASE link layout), "\
            "injected at 0x5c000400 with CPU0 starting there -- skips "\
            "BL1/SPL's own DDR/PLL bring-up, with no -kernel.");       \
    }                                                                 \
    DEFINE_MACHINE_EXTENDED(namestr, AXERA_MACHINE, AxeraMachineState, \
                            tag##_class_init, false,                    \
                            aarch64_machine_interfaces)

DEFINE_AXERA_A64_MACHINE("ax630c", ax630c, ax630c_soc)
DEFINE_AXERA_A64_MACHINE("ax620e", ax620e, ax620e_soc)

/*
 * AX620Q (AArch32, two Cortex-A7s): the Sipeed NanoKVM-Go / MaixStereo
 * NAND board profile. DRAM at 0x40000000, 256 MiB
 * (ax620q-nanokvm-go.dts). Only CPU0 boots -- see the file comment on the
 * vendor spin-table SMP method, not modelled here.
 */
static const AxeraSoCConfig ax620q_soc = {
    .desc              = "Axera AX620Q (Cortex-A7, AArch32, CPU1 not modelled)",
    .ram_size_default  = 256 * MiB,
    .uart0_base        = AXERA_UART0_BASE,
    .uart0_irq         = AXERA_UART0_IRQ,
};

/*
 * AX620U's secondary-CPU spin stub (hw/arm/boot.c's
 * default_write_secondary()) needs its own small RAM window, the same
 * way realview.c's SMP_BOOT_ADDR does -- it must not land inside the
 * real DRAM region arm_load_kernel() places the Image/dtb/initrd in,
 * and it must be real read/write RAM, not a create_unimplemented_device
 * catch-all (an unimplemented region ignores writes, so the stub itself
 * would never get written there). Placed just past the periph
 * catch-all's end.
 */
#define AXERA_AX620U_SMP_BOOT_ADDR 0x10c00000ULL
#define AXERA_AX620U_SMP_BOOT_SIZE 0x1000

/*
 * AX620U's flash-file= BL1-equivalent (no real BL1 source exists for
 * this SoC -- see axera-spi-nand.c's own comment): a real deployed
 * AX620U camera's own SPI NAND dump has the same sec_boot RSA
 * image_header magic (0x55543322 at +4) AX620E's does, but
 * *duplicated* -- a second identical header copy at +1024, then real
 * code starting at file offset 0x800. Disassembling that code (real
 * ARM32, not assumed) shows an 8-entry exception vector table loaded
 * 1:1 at this same address (confirmed from its own `ldr pc, [pc, #..]`
 * literal pool targets, all small absolute addresses like 0x820/0x8a8,
 * not high-SRAM-relative), followed by code that copies those 8
 * vectors down to real address 0x0 and then branches on MPIDR to a
 * secondary-CPU spin loop against 0x02220300 -- the exact same
 * smp_bootreg_addr already wired below from the real devicetree. So
 * this stage runs in place at its own file offset with no relocation:
 * load it at 0x800 and start CPU0 there, same technique as AX620E's
 * BL1_BASE below.
 */
#define AXERA_AX620U_BL1_BASE   0x00000800ULL
#define AXERA_AX620U_BL1_SIZE   (128 * KiB)

static void axera_32_init(MachineState *machine, const AxeraSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    static struct arm_boot_info binfo;
    qemu_irq pic[AXERA_NUM_SPI];
    DeviceState *gic;
    SysBusDevice *gicbus;
    ARMCPU *cpu[4];
    unsigned int smp_cpus = machine->smp.cpus;
    int n, i;
    /* Set below, inside the AX620U-only branch, where clkrst itself is
     * created -- the pic[]/gic wiring below isn't ready yet at that
     * point, so the actual IRQ connection happens later in this
     * function; see its own use. */
    DeviceState *ax620u_clkrst = NULL;
    /* Same deferred-IRQ reason as ax620u_clkrst above -- see its own
     * creation site (the ethernet@0x4970000 block) for why. */
    DeviceState *ax620u_eth = NULL;
    /* Same deferred-IRQ reason as ax620u_clkrst above -- see the ten
     * i2c@0x49... controllers' own creation site for why. */
    DeviceState *ax620u_i2c_dev[10] = { NULL };
    /* AX620U's OS04A10, kept to gate axera-vin on its stream state. */
    DeviceState *ax620u_sensor = NULL;

    if (smp_cpus > ARRAY_SIZE(cpu)) {
        error_report("axera: at most %zu CPUs", ARRAY_SIZE(cpu));
        exit(1);
    }

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        /*
         * No PSCI on AX620Q/AX620U -- real silicon's secondary cores
         * come up through the vendor's own spin-table protocol (see
         * ax620u_soc's comment), not CPU_ON. Leaving psci-conduit at
         * its default (disabled) is what makes arm_load_kernel() below
         * fall back to its generic secondary-CPU boot stub automatically
         * once more than one CPU is realized.
         */
        object_property_set_uint(cpuobj, "cntfrq", AXERA_CNTFRQ,
                                  &error_fatal);
        /*
         * QEMU's cortex-a7 model defaults has_el3=true (the CPU
         * supports TrustZone), and nothing here was ever overriding
         * that -- unlike AX630C/AX620E's real ATF/BL31, AX620U's BL1
         * is the simple, PSCI-less, spin-table-only stage described
         * just above, with no secure-monitor loop left running once
         * it hands off. Confirmed live this costs AX620U's kernel
         * boot *every* interrupt, not just a cosmetic one: with EL3
         * present but never properly configured by this BL1 (no real
         * secure-monitor code ever runs to set SCR/NSACR/CNTHCTL for
         * routing non-secure interrupts to EL1), the architected
         * timer's physical-PPI interrupt is requested and "enabled"
         * by the kernel's arch_timer driver without error (clocksource
         * and sched_clock both register and print correctly, since
         * those only need the free-running counter, not the compare
         * interrupt) but then never actually reaches the CPU: `-d
         * int` showed exactly 5 exception/mode-switch lines for an
         * entire 20+ second run (all early boot EL-mode-switch setup,
         * none after) and the boot CPU sat frozen at the exact same
         * idle-loop PC (confirmed via repeated live register samples)
         * forever. Explicitly disabling has_el3 here (matching the
         * real, PSCI-less, no-secure-monitor shape this file already
         * documents for AX620Q/AX620U) fixes this completely: the
         * same `-d int` trace went from 5 lines to over 11 million in
         * the same window, and boot proceeds for real -- past the
         * timer/scheduler, through driver probing, to the kernel's
         * own MTD/SPI-NAND driver actually attempting to read the
         * root filesystem.
         */
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }

    memory_region_add_subregion(sysmem, AXERA_RAM_BASE, machine->ram);

    if (c->smp_bootreg_addr) {
        MemoryRegion *smpboot_ram = g_new(MemoryRegion, 1);

        memory_region_init_ram(smpboot_ram, NULL, "axera.ax620u-smpboot",
                               AXERA_AX620U_SMP_BOOT_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, AXERA_AX620U_SMP_BOOT_ADDR,
                                    smpboot_ram);
    }

    create_unimplemented_device("axera-periph", AXERA_PERIPH_BASE,
                                 AXERA_PERIPH_SIZE);
    /*
     * AX620Q is the AX620E die (see axera_ax620e_periphs()): same
     * clock/pinctrl/GPIO syscons, PLL, timers and SPI panel bus, and the
     * same on-chip SRAMs, as the AArch64 machines. AX620U
     * ("axera,ax620x") is a different SoC design whose clock controller
     * addresses are not confirmed, so it gets none of these -- only the
     * ISP/NPU blocks a real AX620U firmware's own devicetree actually
     * showed (same addresses as AX630C/AX620E's family, smaller register
     * windows).
     */
    if (!c->is_ax620x) {
        MemoryRegion *npu_sram = g_new(MemoryRegion, 1);
        MemoryRegion *iram0 = g_new(MemoryRegion, 1);

        memory_region_init_ram(npu_sram, NULL, "axera.npu-sram",
                               0x280000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x03000000ULL, npu_sram);
        memory_region_init_ram(iram0, NULL, "axera.iram0", 0x1000,
                               &error_fatal);
        memory_region_add_subregion(sysmem, 0x0ULL, iram0);

        axera_map_regbanks();
    } else {
        /*
         * AX620U's real devicetree (from a deployed camera firmware,
         * same extraction method as AX620E's): isp@2400000
         * "axera,proton" reg <0x100000>; ax_npu@0x3800000 "axera,npu"
         * reg <0x3800000 0x7000>, <0x3000000 0x500000> -- smaller
         * windows than AX630C/AX620E's, confirmed from the real DT, not
         * assumed to match.
         *
         * isp@2400000 "axera,proton" is now the real axera-vin device
         * (see hw/misc/axera-vin.c), not a passive regbank -- created
         * down where pic[] exists, same reason as AX620E's own
         * instantiation (see that call site's comment). This devicetree
         * itself doesn't surface an "interrupts" property for this node
         * in anything extracted so far (the boot-stage DTB found in the
         * real dump lacks the isp@2400000 node entirely, and the guest
         * shell is too RT-starved under this app's retry-loop flood to
         * read /proc/interrupts live, the same way AX620E's own
         * previously-undocumented real number had to be found) -- so
         * the SPI number used there is a documented ASSUMPTION, not a
         * confirmed fact, flagged at that call site.
         */
        /*
         * axera-npu itself is created down where pic[] exists (see the
         * axera-vin call site); its on-chip memory (OCM, which ax_npu.ko
         * maps as "npu_ocm") is plain RAM.
         */
        {
            MemoryRegion *ocm = g_new(MemoryRegion, 1);

            memory_region_init_ram(ocm, NULL, "axera.npu-ocm", 0x500000,
                                   &error_fatal);
            memory_region_add_subregion(sysmem, 0x03000000ULL, ocm);
        }

        /*
         * secondary-boot-reg @ 0x02220300 (see
         * hw/misc/axera-ax620u-smp-wake.c's own comment): CPU0's own
         * wake-secondary-cpus handshake is fatal if it fails, not a
         * soft warning, and its three per-core ack words need to
         * default to -1, not 0 -- a plain hisi-regbank's own
         * zero-initialized storage reads back as "a problem was
         * reported" to BL1's own check, not "no problem yet", which
         * is backwards from what gets this past the check.
         */
        DeviceState *smpreg = qdev_new(TYPE_AXERA_AX620U_SMP_WAKE);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(smpreg), &error_fatal);
        /* Priority 2: the "axera-clk-common" regbank mapped later at
         * 0x02220000 (64 KiB) would otherwise shadow this window. */
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(smpreg), 0,
                                c->smp_bootreg_addr & ~0xfffULL, 2);
        /* The three per-core ack words themselves, 0x1329c (see
         * hw/misc/axera-ax620u-smp-wake.c's own comment) -- overlaid
         * on top of boot_sram's plain storage below. */
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(smpreg), 1, 0x1329cULL, 1);

        /*
         * axera-ax620u-clk @ 0x049a0000, 0x20000: no real devicetree
         * confirms this address (unlike smp_bootreg_addr above) -- it's
         * reverse-engineered from a real deployed AX620U camera's own
         * flash-file= boot replay (see axera_32_init()'s own BL1
         * injection below): disassembling the real BL1-equivalent and
         * tracing its actual register accesses (every one previously
         * landing on the create_unimplemented_device catch-all, logged
         * via '-d unimp') showed a clock-gate enable sequence
         * (0x049a0008 bit0 clear/write-full-config-at-+0x000/set, the
         * same disable-reconfigure-enable shape already modelled for
         * axera-pllc.c and axera-ddr-pll.c) plus several sibling
         * register clusters at 0x049a2xxx/0x049a8xxx/0x049ba000/
         * 0x049bc6xx all read-modify-written the same way. A passive
         * readback bank (not a bespoke ready-bit device, unlike the PLL
         * cases) was enough to get the real BL1 code past this stage --
         * confirmed empirically: the exact same real dump that spun
         * here forever (12M+ unimplemented-device accesses in a 20s
         * run) against the catch-all makes real forward progress once
         * writes here read back instead of discarding.
         *
         * That "clock-gate enable sequence" read of +0x000/+0x004/+0x008
         * was misidentified above: it's not a PLL-style gate at all --
         * it's the exact same DesignWare APB free-running down-counter
         * AX620E's BL1 drives at TIMER_BASE (LOADCOUNT/CURRENT_VALUE/
         * CONTROL_REG, see hw/misc/axera-bl1-timer.c's own comment),
         * just rebased to 0x049a0000 for AX620U. Confirmed live via
         * gdb (no symbols): BL1's own get_time(unit) helper reads raw
         * ticks from +0x004 and divides by an AXERA_CNTFRQ-scaled
         * constant (24000000/24000/24, for seconds/ms/us), and its
         * caller spins "while (duration >= start - current)" -- against
         * the passive regbank underneath, current never advances past
         * start, so this is an unconditional infinite loop on the very
         * first delay()/mdelay()/udelay() call BL1 makes (confirmed: zero
         * console output and ~390% host CPU even after 3 minutes).
         * Overlay just the timer's own 0x100-byte register window, at a
         * higher priority than the passive regbank below, which still
         * backs the real +0x2xxx/+0x8xxx/+0xa000/+0xc6xx clusters.
         */
        DeviceState *clk = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(clk, "size", 0x20000);
        qdev_prop_set_string(clk, "name", "axera-ax620u-clk");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(clk), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(clk), 0, 0x049a0000, 1);

        DeviceState *clktimer = qdev_new(TYPE_AXERA_BL1_TIMER);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(clktimer), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(clktimer), 0, 0x049a0000, 2);

        /*
         * watchdog@49aa000 ("snps,dw-wdt", bound to upstream dw_wdt) also
         * sits inside the passive regbank above. It is the kernel's only
         * restart handler on AX620U (no PSCI, no syscon-reboot): against
         * plain storage every reboot printed "Reboot failed -- System
         * halted". Its interrupt is not wired: this kernel's dw_wdt
         * never requests one.
         */
        DeviceState *wdt = qdev_new(TYPE_AXERA_WDT);

        qdev_prop_set_bit(wdt, "dw-apb", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(wdt), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(wdt), 0, 0x049aa000, 2);

        /*
         * ddr_pll @ 0x00210000 (see hw/misc/axera-ax620u-ddr-pll.c's own
         * comment): BL1's ddr-init routine spins on this block's own
         * ready bit next, right after the timer fix above lets it get
         * this far at all -- same "passive storage can never satisfy a
         * poll" shape, just a third real device discovered the same way
         * (gdb, no symbols, confirmed against the real dump).
         */
        DeviceState *ddrpll = qdev_new(TYPE_AXERA_AX620U_DDR_PLL);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(ddrpll), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(ddrpll), 0, 0x00210000, 1);

        /*
         * clkrst @ 0x01920000 (see hw/misc/axera-ax620u-clkrst.c's own
         * comment): BL1 configures a peripheral instance here next,
         * right after the ddr-pll fix above, then spins on that same
         * instance's own +0x88 status word -- a fourth real device
         * discovered the same way (gdb, no symbols, confirmed against
         * the real dump).
         */
        DeviceState *clkrst = qdev_new(TYPE_AXERA_AX620U_CLKRST);

        ax620u_clkrst = clkrst;
        sysbus_realize_and_unref(SYS_BUS_DEVICE(clkrst), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(clkrst), 0, 0x01920000, 1);

        /*
         * AX620U's real devicetree (ax620u.dts) defines eight plain
         * "syscon" clock-provider nodes the Linux kernel's own clk
         * framework registers against during early boot -- none of
         * these were ever wired up for ax620u before now, since
         * nothing before the kernel itself (BL1, U-Boot) ever touches
         * them. Confirmed live via '-d unimp': once the NAND/DMA fixes
         * above let a real kernel actually start executing, it hammers
         * flash_clk@0x10030000 in a tight, unbroken poll loop (hundreds
         * of thousands of reads in a few seconds, vs. a handful for
         * every other syscon here) -- almost certainly the SPI-NAND
         * clock's own clk_prepare_enable()/clk_is_enabled() path,
         * which needs to succeed before the root filesystem
         * (/dev/mtdblock4) can even be probed, let alone mounted --
         * and with nothing backing that address, the read always
         * comes back 0 and the wait never ends, so the kernel never
         * reaches its own console/printk output at all. A plain
         * readback regbank for each (same "good enough until a real
         * clk driver is modelled" shape already used for the ax620e/
         * ax620q tables above) was enough to let it past this point.
         * ap-clk@0x049ba000 is deliberately omitted: it's already
         * covered by the "axera-ax620u-clk" regbank mapped higher up
         * in this same block (0x049a0000, size 0x20000).
         */
        static const struct {
            const char *name;
            hwaddr base;
            uint32_t size;
        } ax620u_clk_regbanks[] = {
            { "axera-clk-flash",  0x10030000, 0x10000 },
            { "axera-clk-cpu",    0x01900000, 0x10000 },
            { "axera-clk-common", 0x02220000, 0x10000 },
            { "axera-clk-isp",    0x02500000, 0x80000 },
            { "axera-clk-vpu",    0x04030000, 0x10000 },
            { "axera-clk-mm",     0x04430000, 0x10000 },
            { "axera-clk-dispc",  0x044b0000, 0x10000 },
            /*
             * Two of the real devicetree's three "snps,dw-apb-gpio"
             * controllers (the third, @0x049a8000, already falls
             * inside the "axera-ax620u-clk" regbank mapped above and
             * needs nothing extra) -- confirmed live: once the
             * flash-clk fix above let the kernel get this far, it
             * immediately started hammering gpio@0x10010000 with a
             * write/read-back retry loop (toggling bit 5, 0x20) in
             * the exact same "spins forever against passive storage
             * that doesn't reflect real GPIO electrical state" shape
             * as everything else in this file -- plain storage is
             * enough to get a probe-only driver like this unstuck,
             * same as the clk syscons just above.
             */
            { "axera-gpio-a",     0x10010000, 0x1000 },
            { "axera-gpio-b",     0x04960000, 0x1000 },
            /*
             * rtc-top@0x10040000 ("axera,axi-top-rtc", two reg
             * ranges): once the gpio fix above let the kernel probe
             * further, its own RTC driver immediately became the new,
             * overwhelmingly dominant spin (hundreds of thousands of
             * reads of the second range's own +0x18 in a few seconds,
             * vs. a handful for everything else) -- same "probe-time
             * ready/sync-bit poll against passive storage that never
             * satisfies it" shape as everything else here.
             */
            { "axera-rtc-top-a",  0x10040000, 0x2000 },
            { "axera-rtc-top-b",  0x02210000, 0x100 },
            /*
             * 0x02200000: unlike every other entry in this table, no
             * node in the real devicetree (ax620u.dts) has a reg
             * range starting here -- after the rtc-top fix above this
             * became the new dominant spin (tens of thousands of
             * reads of its own +0x14 in a few seconds), with nearby
             * writes including a literal "mc20" ASCII magic value at
             * +0x2c, so it's likely probed via a fixed platform
             * address inside a closed, source-unavailable driver
             * rather than through the devicetree at all (same
             * closedness as this project's other Xiongmai-specific
             * layers) -- unconfirmed identity, but the same passive-
             * storage treatment unblocks it the same way.
             */
            { "axera-misc-a",     0x02200000, 0x1000 },
            /*
             * 0x04980000 and 0x02260000: same shape as 0x02200000
             * above -- no matching devicetree reg range, found live
             * by letting the kernel run further after each previous
             * fix and watching what it touches next.
             */
            { "axera-misc-b",     0x04980000, 0x1000 },
            { "axera-misc-c",     0x02260000, 0x1000 },
            /*
             * DDRMC @ 0x00100000: the same block, same address, and
             * same already-documented reason as ax620e_regbanks[]'s
             * own entry above ("ddr_init.c/ddr_scan.c read back
             * plenty of these mid-training") -- confirmed genuinely
             * touched here too via a fresh '-d unimp' survey (several
             * scattered offsets in the 0x00102000-0x00105600 range,
             * a handful of hits each, consistent with BL1's own DDR
             * training reading back retrain/eye-scan registers before
             * the kernel or this table's other AX620U-specific
             * entries ever run). Never added for AX620U before now.
             */
            { "axera-ddrmc",      0x00100000, 0x10000 },
            /*
             * tsensor @ 0x02000000: once the misc-a/b/c fixes above let
             * the kernel probe further, a closed-source vendor thermal
             * module (its own private "kthread-thm" polling thread --
             * NOT drivers/thermal/axera_thermal.c, whose
             * "axera,ax620e-tsensor" DT node this board's real,
             * dump-extracted devicetree does not even contain) starts
             * polling a raw 10-bit ADC register at +0x4c. Confirmed
             * live via gdb (no symbols: disassembled the actual read
             * site and its whole call chain by breakpointing the
             * generic thermal_zone_get_temp()-style ops->get_temp()
             * indirect call and walking the kernel stack's saved return
             * addresses back up, since the panic backtrace's own
             * "ax620_thermal_enable_sensor+0x0" symbol is the usual
             * misattribution -- the compiler omits the epilogue after a
             * noreturn panic() call, so the linker's next function
             * takes over that address) -- the register offset matches
             * exactly, confirmed separately via '-d unimp' showing
             * "unimplemented device read (size 4, offset 0x0200004c)".
             * Against the catch-all (or a zero-initialized regbank),
             * that read always comes back 0, which the module's own
             * piecewise-linear raw-ADC-to-temperature conversion
             * (reverse-engineered from the same disassembly) maps to
             * 192352 -- matching the real panic's own
             * "temp[0]:192352, temp[1]:192352, temp[2]:192352" printout
             * exactly. Three consecutive polls above threshold is this
             * module's own thermal-protection trip condition, so a
             * constant 0 raw reading guarantees "Kernel panic - not
             * syncing: thm reboot" a few seconds after boot, every
             * time. See the raw-value preseed below, right after this
             * table's regbanks are realized, for the fix.
             */
            { "axera-tsensor",    0x02000000, 0x1000 },
            /*
             * usb/usb@0x4800000, "cdns,usb3" (ax620u.dts): reg-names
             * "otg","xhci","dev" at 0x4800000/0x4810000/0x4820000,
             * dr_mode="host" (fixed, no OTG role-switching ever
             * happens) -- same "no interrupts property in this dump's
             * own devicetree" pattern already seen for both the
             * tsensor and the ethernet node above, so whatever real
             * IRQ this uses (if any) isn't recoverable from the DT
             * either. The "xhci" window gets QEMU's own real,
             * well-tested TYPE_XHCI_SYSBUS model below instead of a
             * passive regbank (its capability/operational registers
             * need to read back as a genuine, self-consistent XHCI
             * controller for the kernel's generic xhci-hcd probe to
             * succeed at all, not just "whatever was last written").
             * "otg" and "dev" are passive regbanks here: in a
             * dr_mode=host-fixed build the cdns3 glue driver still
             * reads a few registers in each (ID/capability probing)
             * before handing off to xhci-hcd, but never needs the
             * OTG role-switch or device-mode logic those two windows
             * would otherwise drive. No IRQ is connected for the
             * xhci instance either, matching this project's own
             * already-accepted precedent for AX620E's usb2@8000000
             * TYPE_USB_DWC3 instance (see its own comment: register-
             * level access works, interrupt-driven transfers do not,
             * a limitation of the upstream models themselves).
             */
            { "axera-usb-otg",    0x04800000, 0x10000 },
            { "axera-usb-dev",    0x04820000, 0x10000 },
        };
        int rb;

        for (rb = 0; rb < ARRAY_SIZE(ax620u_clk_regbanks); rb++) {
            DeviceState *dev = qdev_new("hisi-regbank");

            qdev_prop_set_uint32(dev, "size", ax620u_clk_regbanks[rb].size);
            qdev_prop_set_string(dev, "name", ax620u_clk_regbanks[rb].name);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                    ax620u_clk_regbanks[rb].base, 1);
        }

        /*
         * Preseed the tsensor regbank's raw ADC register (+0x4c, mapped
         * just above) with a value whose conversion comes out plausible
         * instead of catastrophic. The module's own conversion formula
         * (decoded from the disassembly described in the table comment
         * above) is piecewise: raw<=449 always maps to >=60000 (and
         * raw=0 lands on exactly 192352, the real panic's value), while
         * raw>449 maps to 100000*(778-raw)/242 - 40000, decreasing as
         * raw increases. raw=584 (0x248) lands that branch on 40165 --
         * a plausible ~40 degC-scale reading, comfortably under any
         * real trip threshold, so the module's own 3-consecutive-polls-
         * over-threshold safety panic never fires. A plain
         * zero-initialized regbank can't produce a nonzero default by
         * itself, so this one-time direct physical-memory write (same
         * mechanism as axera_bl1_chipmode_reset() above) stands in for
         * whatever real calibration/trim write real firmware would
         * normally have done to this register before the kernel ever
         * reads it.
         */
        address_space_stl_le(&address_space_memory, 0x0200004c, 584,
                             MEMTXATTRS_UNSPECIFIED, NULL);

        /*
         * ethernet@0x4970000, "axera,ax620-eqos" (ax620u.dts): once the
         * tsensor fix above let the kernel run long enough to reach
         * real userspace for the first time, its access pattern against
         * the previous plain axera-periph catch-all (confirmed live via
         * '-d unimp': MAC_CONFIGURATION writes at +0x000, MAC_PACKET_
         * FILTER at +0x008, MTL block at +0xc30/+0xd00/+0xd2c/+0xd30,
         * DMA_MODE/DMA_SYSBUS_MODE at +0x1000/+0x1004, DMA channel 0's
         * CONTROL/TX_CONTROL/RX_CONTROL at +0x1100/+0x1104/+0x1108)
         * lines up exactly with hw/net/axera-eqos.c's own already-
         * modelled Synopsys DWC EQoS register map (MAC at 0x0000, MTL
         * at 0x0C00, DMA at 0x1000, channel 0 at 0x1100 -- see that
         * file's own header comment) -- the same underlying IP, just
         * AX620U's own SoC-level glue name for it, exactly like that
         * file's header already describes for nvt-eqos.c's relationship
         * to it. Reused as-is rather than writing a second copy. No
         * "interrupts" property surfaced for this node in this dump's
         * own extracted devicetree (confirmed via a raw FDT byte walk,
         * not just dtc's decompile -- genuinely absent, not an
         * extraction artifact), so the real number isn't recoverable
         * from the DT at all. Originally wired to a guessed GIC SPI
         * 0x50 (AX620E's own, unrelated, macirq number for the same IP
         * family) -- confirmed live via `-d int` exception tracing
         * that this guess was never delivered to the CPU at all (zero
         * non-timer IRQ exceptions across repeated inbound connection
         * attempts). The REAL number was found directly, with no
         * disassembly needed: a live shell's own `cat /proc/interrupts`
         * lists three separate real registrations for eth0 -- GIC SPI
         * 132/133/134, matching mainline stmmac's "macirq"/
         * "eth_wake_irq"/"eth_lpi" triple (only the first, mandatory,
         * data-path one is modelled here, same as AX620E's own file
         * header comment already describes for its own eqos instance).
         *
         * FOUND AND FIXED: an earlier session tried wiring this to
         * 0x50 (AX620E's own unrelated macirq number, a guess) and then
         * to 0x84/0x85/0x86 (132/133/134 taken AS-IS from the
         * /proc/interrupts listing above) -- all four failed to ever
         * deliver, leading to a (wrong) conclusion that the bug was
         * shared/platform-independent, not a wrong-number problem.
         * The actual mistake: /proc/interrupts' "GIC-0 nnn" column is
         * the Linux-side annotation, which includes the GICv2 SGI+PPI
         * reservation (the first 32 INTIDs) -- it is SPI_INDEX + 32,
         * NOT the raw SPI index this file's own pic[] array is indexed
         * by. (Cross-checked directly against AX620E's own eth0, whose
         * SPI 0x50/80 -- independently confirmed correct straight from
         * its real devicetree -- shows up as exactly "GIC-0 112" in its
         * own /proc/interrupts, i.e. 80 + 32 = 112.) So 132/133/134
         * are GIC-0 annotations, and the real SPI indices are
         * 132-32=100=0x64, 133-32=101=0x65, 134-32=102=0x66 -- NONE of
         * which were ever actually tried; 0x84/0x85/0x86 (132/133/134
         * used directly as pic[] indices) are a completely different,
         * unrelated 32 interrupts higher. Wiring to pic[0x65] (the
         * corrected "macirq", first/mandatory per stmmac_platform.c's
         * request order) and confirming live: the guest's own ARP
         * reply to slirp's "who-has 192.168.1.10" now completes (the
         * next packet slirp sends arrives correctly addressed to the
         * guest's real unicast MAC, which only happens once slirp has
         * received and processed a genuine ARP reply -- proof the
         * guest's kernel actually took the RX interrupt and ran its
         * ARP handler), and the IRQ line now visibly asserts AND
         * clears (guest-acknowledged) on every single inbound frame,
         * unlike the previous four candidates where it never did either.
         * RX networking is genuinely fixed on AX620U now.
         *
         * The remaining, SEPARATE symptom -- the RTSP SYN itself still
         * gets no TCP-level response even with RX now fully working --
         * reproduces identically to AX620E's own already-diagnosed
         * app-level issue (see kaeru episode
         * ax620e-rtsp-syn-delivery-fully-confirmed-correct-root-cause-is-app-level-not-qemu):
         * the SYN is delivered and IRQ-acked perfectly, yet the guest
         * never replies, not even a RST. That is NOT a networking/IRQ
         * bug (confirmed, same evidence as there) -- it's the shared
         * camera app's own connection-acceptance behavior while its
         * video pipeline is unhealthy (the already-known missing real
         * ISP/VPSS pixel pipeline), not anything fixable here.
         */
        DeviceState *eth = qdev_new("axera-eqos");

        ax620u_eth = eth;
        qemu_configure_nic_device(eth, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(eth), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(eth), 0, 0x04970000, 1);

        {
            DeviceState *xhci = qdev_new(TYPE_XHCI_SYSBUS);

            qdev_prop_set_uint32(xhci, "slots", 4);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(xhci), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(xhci), 0, 0x04810000, 1);
        }

        /*
         * ax620u.dts's own aliases list ten i2c buses (i2c0-i2c9),
         * i2c@0x4980000 through i2c@0x49b6000, each with a real,
         * DT-confirmed GIC SPI (unlike the eth/usb/tsensor nodes
         * earlier in this branch, whose own "interrupts" property is
         * genuinely absent from this dump's own devicetree) --
         * interrupts = <0x00 0x5a 0x04>  through  <0x00 0x63 0x04>,
         * one consecutive SPI per bus in declaration order. None of
         * these were ever wired up before, leaving every access on
         * the plain axera-periph catch-all -- the same "no /dev/i2c-N
         * node ever created, kernel's i2c-designware driver can't
         * read a sane DW_IC_COMP_TYPE back" gap already fixed for
         * AX620E's own i2c2/i2c6 (see this function's own i2c6_bus
         * comment, in axera_a64_init() above) via hw/i2c/dw-i2c.c --
         * the SAME generic model, reused as-is, since both SoCs'
         * kernels bind the real mainline i2c-designware-master.c
         * driver against this register layout. No sensor child is
         * attached to any of these ten buses yet -- unlike AX620E,
         * which bus (if any) the real camera app's own sensor probe
         * actually walks on AX620U isn't known yet; wiring the bus
         * controllers alone already lets the kernel create real
         * /dev/i2c-N nodes for whichever ones it probes, which is
         * the same first concrete step AX620E's own fix took before
         * the specific bus/sensor-address pairing was identified.
         */
        {
            static const struct {
                hwaddr base;
                int irq;
            } ax620u_i2c[] = {
                { 0x04980000, 0x5a },
                { 0x04982000, 0x5b },
                { 0x04984000, 0x5c },
                { 0x04986000, 0x5d },
                { 0x04988000, 0x5e },
                { 0x0498a000, 0x5f },
                { 0x049b0000, 0x60 },
                { 0x049b2000, 0x61 },
                { 0x049b4000, 0x62 },
                { 0x049b6000, 0x63 },
            };
            int ic;

            /* pic[]/gic aren't ready yet this early in the function
             * (same reason ax620u_eth/ax620u_clkrst defer their own
             * irq connection) -- create+map each bus now, save its
             * DeviceState, and connect the real irq later below. */
            for (ic = 0; ic < ARRAY_SIZE(ax620u_i2c); ic++) {
                DeviceState *i2c = qdev_new("dw-i2c");

                sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
                sysbus_mmio_map_overlap(SYS_BUS_DEVICE(i2c), 0,
                                        ax620u_i2c[ic].base, 1);
                ax620u_i2c_dev[ic] = i2c;
            }

            /*
             * Confirmed live via '-d unimp'/dmesg: once the ten bus
             * controllers above are wired up, i2c-designware itself
             * binds cleanly on three of them (4982000/49b0000/
             * 49b4000.i2c: "running with gpio recovery mode!"), and
             * the real camera app's own repeated probe traffic lands
             * specifically on /dev/i2c-8 (= 0x49b4000.i2c, index 8
             * here, matching ax620u.dts's own i2c0-i2c9 alias
             * ordering) -- "Remote I/O error" (EREMOTEIO, no ACK)
             * against the bare controller with nothing attached.
             *
             * A first attempt here used hisi-smartsens at 0x30 (the
             * same stand-in AX620E's own sensor fix needed, since that
             * board's own "-i imx307" launch argument turned out to be
             * a generic default, not the real sensor) -- but got no
             * ACK, and docs/axera.md's own "confirmed sensors" note
             * already independently identified AX620U's real deployed
             * sensor as an OmniVision OS04A10, not a SmartSens part at
             * all. Confirmed directly from the vendor SDK's own
             * drivers/sensors/ov_os04a10/os04a10.c and os04a10_reg.c:
             * gOs04A10SlaveAddr[] = {0x36, 0x36} (7-bit I2C address),
             * and os04a10_get_chipid() reading exactly
             * reg(0x300A)<<16 | reg(0x300B)<<8 | reg(0x300C) against
             * OS04A10_SENSOR_CHIP_ID = 0x530441 -- i.e. id bytes
             * 0x53/0x04/0x41 at that base, the same 16-bit-register/
             * 8-bit-data shape hisi-smartsens.c already models for
             * SmartSens parts, just a different vendor's register
             * convention. hw/i2c/axera-ov-sensor.c is a new, equally
             * generic stub for this OmniVision/OS-series protocol
             * family, written specifically from that confirmed source
             * rather than guessed.
             */
            {
                DeviceState *i2c8 = ax620u_i2c_dev[8];
                I2CBus *i2c8_bus = I2C_BUS(qdev_get_child_bus(i2c8, "i2c"));
                DeviceState *sensor = qdev_new("axera-ov-sensor");

                qdev_prop_set_uint16(sensor, "id_reg_base", 0x300A);
                qdev_prop_set_uint8(sensor, "id0", 0x53);
                qdev_prop_set_uint8(sensor, "id1", 0x04);
                qdev_prop_set_uint8(sensor, "id2", 0x41);
                qdev_prop_set_uint8(sensor, "address", 0x36);
                qdev_realize_and_unref(sensor, BUS(i2c8_bus), &error_fatal);
                ax620u_sensor = sensor;

                /*
                 * expander_io@0x20 on this exact bus in the real
                 * devicetree ("axera,pcal6416a", a generic NXP
                 * PCAL6416A 16-bit I2C GPIO expander). Confirmed live
                 * on a real dump (JFtech_AX620U_xmuart0.dump, booted
                 * with xmuart=0 so the App's own console log is
                 * actually visible): right as the camera pipe's own
                 * "HAL_Reset_init pipe 0" step runs, the real deployed
                 * ax_pcal6416a.ko logs a burst of "i2c_read_byte"/
                 * "i2c_write_byte ... error: -121" (EREMOTEIO) against
                 * address 0x20 on this bus -- nothing was attached
                 * there before this fix -- immediately followed by
                 * AX_VIN_SetChnAttr[0] failing with "Error [80010180]:
                 * Close all chn is not allowed", which is this
                 * project's own long-chased "AX_VIN_GetYuvFrame fail
                 * ret[0x8001010a]" symptom's real upstream cause (see
                 * hw/i2c/axera-pcal6416a.c's own comment for the full
                 * trace and the real .ko's confirmed register usage).
                 */
                DeviceState *expander = qdev_new("axera-pcal6416a");

                qdev_prop_set_uint8(expander, "address", 0x20);
                qdev_realize_and_unref(expander, BUS(i2c8_bus), &error_fatal);
            }
        }
    }

    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(gic, "num-irq", AXERA_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    if (!machine->kernel_filename) {
        ARM_GIC_COMMON(gic)->irq_reset_nonsecure = true;
    }
    sysbus_mmio_map(gicbus, 0, AXERA_GIC_DIST);
    sysbus_mmio_map(gicbus, 1, AXERA_GIC_CPU);

    for (n = 0; n < smp_cpus; n++) {
        DeviceState *cpudev = DEVICE(cpu[n]);
        int ppibase = AXERA_NUM_SPI + n * GIC_INTERNAL + GIC_NR_SGIS;
        static const int timer_ppi[] = {
            [GTIMER_PHYS] = 14,
            [GTIMER_VIRT] = 11,
            [GTIMER_HYP]  = 10,
            [GTIMER_SEC]  = 13,
        };

        for (i = 0; i < ARRAY_SIZE(timer_ppi); i++) {
            qdev_connect_gpio_out(cpudev, i,
                                   qdev_get_gpio_in(gic, ppibase +
                                                     timer_ppi[i]));
        }
        sysbus_connect_irq(gicbus, n,
                            qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbus, n + smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbus, n + 2 * smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbus, n + 3 * smp_cpus,
                            qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    for (n = 0; n < AXERA_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    if (ax620u_clkrst) {
        /*
         * ax620u.dts's "dw_dmac@1920000" declares interrupts = <0x00
         * 0x05 0x04> (GIC SPI 5). Needed for the real kernel's
         * dw-axi-dmac dmaengine driver, whose generic submission path
         * (confirmed live via gdb, disassembling the code that writes
         * this device's own DMAC_CHEN-equivalent register at absolute
         * offset 0x18 -- see axera-ax620u-clkrst.c) is upstream Linux
         * dmaengine framework code operating on a per-channel
         * descriptor queue, which only an interrupt-driven completion
         * callback (this line, in real hardware) ever walks -- BL1
         * and U-Boot's own much simpler polled driver never expects
         * or needs it, so it's only connected here, never raised for
         * their own DMA_DIR_OFF-triggered transfers.
         */
        sysbus_connect_irq(SYS_BUS_DEVICE(ax620u_clkrst), 0, pic[5]);
    }

    if (ax620u_eth) {
        sysbus_connect_irq(SYS_BUS_DEVICE(ax620u_eth), 0, pic[0x65]);
    }

    if (c->is_ax620x) {
        /*
         * isp@2400000, "axera,proton": the same axera-vin device as
         * AX620E (see hw/misc/axera-vin.c's own header comment) --
         * register map independently cross-checked against AX620U's
         * own real driver traffic (a prior '-d unimp' survey hit
         * exactly 0x20/0x30/0x40/0x50 and the same four again at
         * +0x80000, matching the derived per-channel formula exactly),
         * so that part is solid. The SPI number is NOT: the real
         * ax_proton.ko here (ARM32, a completely different build from
         * AX620E's AArch64 one) calls AX_OSAL_DEV_platform_get_irq()
         * exactly ONCE for this device (confirmed via disassembly of
         * the real deployed module), unlike AX620E's two separate
         * requests -- so both of axera-vin's IRQ outputs are wired to
         * the SAME pic[] index here, on the working theory that one
         * physical line carries both register ranges on this smaller
         * variant. That index itself is an ASSUMPTION copied from
         * AX620E's first (confirmed, DTB-sourced) SPI number, 0x1b --
         * every attempt to find AX620U's own real number failed this
         * session (the dump's boot-stage DTB has no isp@2400000 node
         * at all, and the guest shell is too RT-starved under this
         * app's own retry-loop flood to read /proc/interrupts live).
         * Cheap to correct later if wrong: this is the one thing a
         * live trace against a less-starved boot, or a kernel-image
         * disassembly for the compiled-in board IRQ resource table,
         * would settle.
         */
        /*
         * venc@4010000 / jenc@4000000 / dec@4020000: the same VeriSilicon
         * VCMD front-ends as AX620E's, at the same addresses but on
         * AX620U's own SPIs (0x2a/0x28/0x29, read from the running
         * kernel's /proc/device-tree -- the boot-stage DTB in the dump
         * has no media nodes). Without them every one of the three
         * probes failed with -1 and nothing could ever be encoded.
         */
        {
            static const struct { hwaddr base; int irq; } vcmds[] = {
                { 0x04010000, 0x2a }, { 0x04000000, 0x28 },
                { 0x04020000, 0x29 },
            };
            int v;

            for (v = 0; v < ARRAY_SIZE(vcmds); v++) {
                DeviceState *d = qdev_new(TYPE_AXERA_VCMD);

                /* AX620U's driver programs DRAM-relative bus addresses */
                qdev_prop_set_uint64(d, "dma-offset", AXERA_RAM_BASE);
                sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, vcmds[v].base);
                sysbus_connect_irq(SYS_BUS_DEVICE(d), 0, pic[vcmds[v].irq]);
            }
        }

        /*
         * NPU job-completion pass-through (hw/misc/axera-npu.c):
         * npu_normal is GIC 68 = SPI 36 in the real AX620U's
         * /proc/interrupts.
         */
        sysbus_create_simple("axera-npu", 0x03800000, pic[36]);

        /*
         * AX620U IVPS engines (register protocol from the real ARM32
         * ax_ivps.ko/ax_gdc.ko/ax_vo.ko; IRQs from the guest's
         * /proc/interrupts): tdp@4410000 SPI 20 (tdpdev_start's last
         * store 0x018=1; tdp_irq acks status 0x004 via W1C 0x010),
         * gdc@2620000 SPI 21 (gdcdev_start kicks 0x058=1; gdc_irq
         * reads 0x048, acks 0x054), vo@4400000 SPI 19 (the IVPS draw
         * stage: ax620a_draw_start kicks 0x020=1; dpu_irq reads 0x004,
         * acks 0x010). Not modelled: TDP "inplace" mode's CDMA at
         * 0x04420000 and GDC tile mode's queue at GDC+0x100.
         */
        {
            static const AxeraIvpsEngineDesc engines[] = {
                { "axera-tdp", 0x04410000, 20, 0x004, 0x010, 0,
                  { 0x018 }, { 1u << 0 }, 1 },
                { "axera-gdc", 0x02620000, 21, 0x048, 0x054, 0,
                  { 0x058 }, { 1u << 0 } },
                { "axera-dpu", 0x04400000, 19, 0x004, 0x010, 0,
                  { 0x020 }, { 1u << 0 }, 5 },
            };

            axera_create_ivps_engines(engines, ARRAY_SIZE(engines), pic);
            create_unimplemented_device("axera-tdp-cdma", 0x04420000,
                                        0x1000);
        }

        DeviceState *vin = qdev_new("axera-vin");

        /* No frame events before the OS04A10 is told to stream -- see
         * axera_vin_tick(). */
        qdev_prop_set_bit(vin, "gate-on-sensor", ax620u_sensor != NULL);
        /*
         * IFE WDMA status (0x50): the driver unmasks bits 17-28 only
         * to count WDMA errors/overflows -- isp_irq_thread marks the
         * channel's next frame for dropping on bits 17-20 -- so a tick
         * raising them made IFE drop every frame (vin_status Loss ==
         * InSeq). isp_irq_thread's error-class mask for this register
         * is 0x1ffe0001 (bit 0 too): any of those also stops it
         * forwarding the frame to userspace (the poll()/read() user-IRQ
         * path that drives AI-ISP). Bits 29-30 are the per-channel
         * frame starts (irq_process_ife_wdma_fsof tests bit 30-n) and
         * bits 1/3/5 the frame ends. Bit 31 is the frame-loss event:
         * isp_irq_thread adds one to every channel's
         * isp_err_stats_proc_param.loss_frm_total (vin_status "Loss"),
         * and the driver first enables exactly 0x9ffe0001 -- its error
         * set -- before adding the frame events (0xdffe002b).
         */
        qdev_prop_set_uint32(vin, "quiet-off", 0x50);
        qdev_prop_set_uint32(vin, "quiet-bits", 0x9ffe0001);
        /*
         * RLTM histogram done: isp_itp_drv_thread polls RAW bit 19 of
         * the 0x80020 block (isp_irq_rltm_hist_done_get reads 0x80028,
         * W1C 0x80024) after each ITP frame end and only then hands the
         * frame's histogram to the type-4 stat source that AX_ISP_Run's
         * 3A loop waits on. The driver never unmasks it: RAW
         * (sta_int_raw) latches events whatever int_en says, and the
         * histogram is complete when the ITP frame ends, the same tick
         * that raises the 0x80040 bit-16 ITP frame done.
         */
        qdev_prop_set_uint32(vin, "poll-off", 0x80020);
        qdev_prop_set_uint32(vin, "poll-bits", 1u << 19);
        /* the sensor picture: see "Sensor picture" in axera-vin.c */
        qdev_prop_set_uint32(vin, "isp-layout", 2);
        qdev_prop_set_uint32(vin, "cdma-list-off", 0x83004);
        qdev_prop_set_uint32(vin, "cdma-list-off2", 0x3004);
        qdev_prop_set_uint64(vin, "cdma-list-base", 0x40000000);
        /* 3A statistics: see axera_vin_ae_stats() */
        if (ax620u_sensor) {
            object_property_set_link(OBJECT(vin), "sensor",
                                     OBJECT(ax620u_sensor), &error_fatal);
            /*
             * The scene reads as painted at 440 rows at unity gain: the
             * App's AE (set point 12) settles at ~143 rows, well short
             * of the night threshold (nightExp 8518971).
             */
            qdev_prop_set_uint64(vin, "ae-ref", 450000);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(vin), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(vin), 0, 0x02400000);
        sysbus_connect_irq(SYS_BUS_DEVICE(vin), 0, pic[0x1b]);
        sysbus_connect_irq(SYS_BUS_DEVICE(vin), 1, pic[0x1b]);
        if (ax620u_sensor) {
            qdev_connect_gpio_out_named(ax620u_sensor, "streaming", 0,
                qdev_get_gpio_in_named(vin, "sensor-streaming", 0));
        }
    }

    {
        static const int ax620u_i2c_irq[] = {
            0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
        };
        int ic;

        for (ic = 0; ic < ARRAY_SIZE(ax620u_i2c_irq); ic++) {
            if (ax620u_i2c_dev[ic]) {
                sysbus_connect_irq(SYS_BUS_DEVICE(ax620u_i2c_dev[ic]), 0,
                                   pic[ax620u_i2c_irq[ic]]);
            }
        }
    }

    if (c->is_ax620x) {
        /*
         * i2s_mst@49ac000 (Synopsys DW_apb_i2s, see hw/misc/axera-i2s.c),
         * GIC SPI 0x74 (Linux irq 41, "49ac000.i2s_mst" in
         * /proc/interrupts). Linked to the ES8311 codec on i2c1 0x19 by
         * ax_audio.ko's simple-card; with an interrupt present the 4.19
         * dwc-i2s.c runs PIO (dw_pcm_push_tx/pop_rx from its own IRQ
         * handler), no DMA. It sits inside the passive
         * "axera-ax620u-clk" regbank (0x049a0000-0x049c0000, priority 1)
         * and the clk timer overlay (priority 2), so map above both.
         */
        DeviceState *i2s = qdev_new(TYPE_AXERA_I2S);

        qdev_prop_set_string(i2s, "name", "i2s_mst");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(i2s), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(i2s), 0, 0x049ac000, 3);
        sysbus_connect_irq(SYS_BUS_DEVICE(i2s), 0, pic[0x74]);
    }

    if (c->is_ax620x) {
        /*
         * mmc@0x10000000 ("socionext,uniphier-sd4hc","cdns,sd4hc",
         * non-removable, status="okay") and mmc@0x4950000 (same
         * compatible, cd-gpios present -- a real card slot) in
         * ax620u.dts: the SAME real "cdns,sd4hc" IP axera_attach_sdhci()
         * (above, shared with AX630C/AX620E's own emmc/sd/sdio) already
         * models, at AX620U's own addresses and DT-confirmed GIC SPIs
         * (interrupts = <0x00 0x09 0x04> / <0x00 0x84 0x04>). Previously
         * unwired -- confirmed live via dmesg: "4950000.sdhc get flash
         * clk:200000000Hz" / "Got CD GPIO" / "sdhci-cdns: probe of
         * 4950000.sdhc failed with error -110" (ETIMEDOUT), the real
         * kernel driver genuinely tries to talk to this controller and
         * times out against the plain axera-periph catch-all. unit 0
         * is the eMMC (matches axera_attach_sdhci()'s own "index 0 is
         * the eMMC" convention above); unit 1 the removable slot.
         */
        axera_attach_sdhci(0x10000000, pic[9], 0, true);
        axera_attach_sdhci(0x04950000, pic[0x84], 1, false);
    }

    if (!c->is_ax620x) {
        axera_ax620e_periphs(pic, false, false, false);
        qemu_register_reset(axera_ax620q_spl_handoff_reset,
                            (void *)(uintptr_t)(drive_get(IF_SD, 0, 1)
                                                != NULL));
    }

    /*
     * AX620Q's console is ttyS3 @ 0x06080000 (serial3 in the shared
     * AX620E.dtsi, GIC SPI 0x79), so it gets serial_hd(0) and UART0
     * becomes the second serial port. AX620U's console is its own UART0.
     */
    serial_mm_init(sysmem, c->uart0_base, 2, pic[c->uart0_irq],
                   AXERA_UART0_FREQ / 16,
                   serial_hd(c->is_ax620x ? 0 : 1),
                   DEVICE_LITTLE_ENDIAN);

    /*
     * AX620Q's own real vendor U-Boot (CONFIG_CONS_INDEX=4,
     * CONFIG_SYS_NS16550_COM4 in include/configs/ax620e_common.h) puts
     * its console on a fourth, separate NS16550 instance at 0x06080000,
     * not on uart0_base -- confirmed by GDB-tracing a silent boot straight
     * into eserial4_init()/NS16550_init(com_port=0x6080000). The kernel's
     * own devicetree (ax_uart@6080000, serial3) gives it GIC SPI 0x79;
     * without it the 8250 driver's THRE-interrupt TX path stalls the
     * moment userspace starts writing to ttyS3.
     */
    if (!c->is_ax620x) {
        serial_mm_init(sysmem, 0x06080000, 2, pic[0x79],
                       AXERA_UART0_FREQ / 16, serial_hd(0),
                       DEVICE_LITTLE_ENDIAN);
    }

    {
        AxeraMachineState *ams = AXERA_MACHINE(machine);

        if (!c->is_ax620x && ams->flash_file && ams->flash_file[0]) {
            error_report("axera: %s: flash-file= is not supported yet "
                         "(no BL1 replay path for AX620Q)",
                          MACHINE_GET_CLASS(machine)->name);
            exit(1);
        }

        /*
         * spi_nand@0x08000000 (real AX620U devicetree: spi-master
         * compatible "snps,dw-ssi", a child spi_nand@0 node -- see
         * hw/misc/axera-spi-nand.c's own comment): back it whenever a
         * real dump is given, same shared-block trade-off as AX620E's
         * SPI NOR above.
         */
        if (c->is_ax620x && ams->flash_file && ams->flash_file[0] &&
            !machine->kernel_filename) {
            DeviceState *spi_nand = qdev_new(TYPE_AXERA_SPI_NAND);

            qdev_prop_set_string(spi_nand, "file", ams->flash_file);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(spi_nand), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(spi_nand), 0, 0x08000000);
        }

        if (ams->uboot_file && ams->uboot_file[0] && !machine->kernel_filename) {
            char *uboot_data = NULL;
            gsize uboot_len = 0;
            hwaddr uboot_base;

            if (c->is_ax620x) {
                error_report("axera: %s: uboot-file= is not supported yet "
                             "(no real AX620U U-Boot source available to "
                             "confirm its link address)",
                              MACHINE_GET_CLASS(machine)->name);
                exit(1);
            }
            uboot_base = AXERA_AX620Q_UBOOT_BASE;

            if (!g_file_get_contents(ams->uboot_file, &uboot_data,
                                     &uboot_len, NULL)) {
                error_report("axera: %s: could not read uboot-file '%s'",
                              MACHINE_GET_CLASS(machine)->name,
                              ams->uboot_file);
                exit(1);
            }

            rom_add_blob_fixed("axera.uboot", uboot_data, uboot_len,
                               uboot_base);
            g_free(uboot_data);

            AxeraBootromReset *info = g_new0(AxeraBootromReset, 1);
            info->cpu = cpu[0];
            info->entry = uboot_base;
            qemu_register_reset(axera_bootrom_cpu_reset, info);
            return;
        }

        if (c->is_ax620x && ams->flash_file && ams->flash_file[0] &&
            !machine->kernel_filename) {
            /*
             * See AXERA_AX620U_BL1_BASE's own comment above for how
             * this address and the double-header skip below were
             * derived from the real dump. The 0x0-0x20000 range this
             * stage actually runs in (vector table at 0x0 once it
             * relocates itself, a stack somewhere under the ~0x18000
             * high-water mark its own literal pool showed) isn't real
             * DRAM -- it's a small on-chip boot SRAM, same real-SoC
             * shape as AX620E's axera-npu-sram carve-out, just with no
             * confirmed real base/size for AX620U's version, so this
             * is sized generously (128 KiB) rather than to a confirmed
             * real figure.
             */
            MemoryRegion *boot_sram = g_new(MemoryRegion, 1);

            memory_region_init_ram(boot_sram, NULL, "axera.ax620u-boot-sram",
                                   AXERA_AX620U_BL1_SIZE, &error_fatal);
            memory_region_add_subregion(sysmem, 0, boot_sram);

            char *bl1_data = NULL;
            gsize bl1_len = 0;
            gsize bl1_off = 0;

            if (!g_file_get_contents(ams->flash_file, &bl1_data, &bl1_len,
                                     NULL)) {
                error_report("axera: %s: could not read flash-file '%s'",
                              MACHINE_GET_CLASS(machine)->name,
                              ams->flash_file);
                exit(1);
            }

            if (bl1_len >= 1024 &&
                ldl_le_p(bl1_data + 4) == 0x55543322) {
                bl1_off = 1024;
                if (bl1_len >= 2048 &&
                    ldl_le_p(bl1_data + 1024 + 4) == 0x55543322) {
                    bl1_off = 2048;
                }
            }

            rom_add_blob_fixed("axera.ax620u-bl1",
                               bl1_data + bl1_off,
                               MIN(bl1_len - bl1_off,
                                   (gsize)AXERA_AX620U_BL1_SIZE),
                               AXERA_AX620U_BL1_BASE);
            g_free(bl1_data);

            AxeraBootromReset *info = g_new0(AxeraBootromReset, 1);
            info->cpu = cpu[0];
            info->entry = AXERA_AX620U_BL1_BASE;
            qemu_register_reset(axera_bootrom_cpu_reset, info);
            /*
             * Without arm_load_kernel() nothing else resets the other
             * cores: on a guest reboot (watchdog) CPU1-3 kept running
             * the old kernel while BL1/U-Boot reloaded it, and the new
             * kernel died in its first SMP bring-up (garbage CPU number
             * in the Oops) before its console came up. Reset them to
             * their power-on state, as at cold boot: they fall through
             * to BL1's MPIDR check and park in its spin loop.
             */
            for (n = 1; n < smp_cpus; n++) {
                qemu_register_reset(axera_cpu_reset_only, cpu[n]);
            }
            return;
        }
    }

    if (machine->kernel_filename && !machine->dtb) {
        error_report("axera: %s needs the kernel's device tree (-dtb)",
                      MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AXERA_RAM_BASE;
    if (c->smp_bootreg_addr) {
        binfo.smp_bootreg_addr = c->smp_bootreg_addr;
        binfo.smp_loader_start = AXERA_AX620U_SMP_BOOT_ADDR;
        binfo.gic_cpu_if_addr = AXERA_GIC_CPU;
    }
    arm_load_kernel(cpu[0], machine, &binfo);
}

#define DEFINE_AXERA_MACHINE(namestr, tag, config)                    \
    static void tag##_init(MachineState *machine)                     \
    {                                                                 \
        axera_32_init(machine, &config);                               \
    }                                                                 \
    static void tag##_class_init(MachineClass *mc)                    \
    {                                                                 \
        ObjectClass *oc = OBJECT_CLASS(mc);                           \
        mc->desc = config.desc;                                       \
        mc->init = tag##_init;                                        \
        mc->default_ram_size = config.ram_size_default;                \
        mc->default_ram_id = "axera.ram";                              \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");         \
        mc->max_cpus = config.max_cpus ? config.max_cpus : 1;          \
        mc->default_cpus = mc->max_cpus;                               \
        object_class_property_add_str(oc, "uboot-file",                \
            axera_machine_get_uboot_file,                              \
            axera_machine_set_uboot_file);                             \
        object_class_property_set_description(oc, "uboot-file",       \
            "A real U-Boot binary (CONFIG_SYS_TEXT_BASE link layout), "\
            "injected at its real link address with CPU0 starting "   \
            "there, with no -kernel. AX620Q only -- no real AX620U "  \
            "U-Boot source exists to confirm its link address.");     \
        object_class_property_add_str(oc, "flash-file",                \
            axera_machine_get_flash_file,                              \
            axera_machine_set_flash_file);                             \
        object_class_property_set_description(oc, "flash-file",       \
            "A real dumped SPI NAND image. Backs spi_nand@0x08000000 "\
            "and injects its own BL1-equivalent at its real on-dump " \
            "link address, with no -kernel. AX620U only -- AX620Q "   \
            "has no BL1 replay path yet.");                             \
    }                                                                 \
    DEFINE_MACHINE_EXTENDED(namestr, AXERA_MACHINE, AxeraMachineState, \
                            tag##_class_init, false,                    \
                            arm_machine_interfaces)

DEFINE_AXERA_MACHINE("ax620q", ax620q, ax620q_soc)

/*
 * AX620U (AArch32, four Cortex-A7s): a distinct SoC from AX620Q despite
 * the shared core -- root compatible is "axera,ax620x", not
 * "axera,ax620e"/"axera,ax620q", and it has its own UART bank. Confirmed
 * from a real deployed AX620U camera firmware's own FIT-image
 * devicetree: GIC-400 at the *same* 0x01850000 family address as
 * AX620E/AX630C/AX620Q, "arm,armv7-timer" at the same 24 MHz with the
 * same PPI layout, but UART0 at 0x04990000 (not 0x04880000) and DRAM at
 * 0x40000000/~1 GiB. Its own vendor spin-table SMP ("axera,ax620x-smp",
 * secondary-boot-reg at 0x02220300, a different register than
 * AX620Q's) is modelled using QEMU's generic ARM secondary-CPU boot
 * code (hw/arm/boot.c's default_write_secondary(): a small poll-and-
 * branch stub written to smp_loader_start, woken by the primary CPU's
 * write to smp_bootreg_addr then an SEV/IPI) -- the exact mechanism
 * the real vendor platsmp.c's ax620x_smp_ops implements kernel-side,
 * just with QEMU providing the boot-ROM-level spin stub real silicon
 * would.
 */
static const AxeraSoCConfig ax620u_soc = {
    .desc              = "Axera AX620U (Cortex-A7 x4, AArch32)",
    .ram_size_default  = 1 * GiB,
    .uart0_base        = 0x04990000ULL,
    .uart0_irq         = 68,
    .is_ax620x         = true,
    .max_cpus          = 4,
    .smp_bootreg_addr  = 0x02220300ULL,
};

DEFINE_AXERA_MACHINE("ax620u", ax620u, ax620u_soc)
