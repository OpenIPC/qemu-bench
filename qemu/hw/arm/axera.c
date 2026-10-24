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
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/axera-bl1-timer.h"
#include "hw/misc/axera-ddr-pll.h"
#include "hw/misc/axera-dw-spi.h"
#include "hw/misc/axera-eip130.h"
#include "hw/misc/axera-pllc.h"
#include "hw/misc/axera-spi-nand.h"
#include "hw/misc/axera-spi-nor.h"
#include "hw/misc/axera-vcmd.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/cadence_sdhci.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-dwc3.h"
#include "qapi/visitor.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/gtimer.h"

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
 * regbank comment above) and the flash_type=1 ("EMMC BOOT
 * 8BIT_50M_768K") encoding board.c's get_boot_mode() expects in bits
 * [3:1] to make BL1 run its real card-init sequence.
 */
#define AXERA_CHIPMODE_SW_ADDR  0x0239000cULL
#define AXERA_CHIPMODE_EMMC_BOOT_8BIT 0x2U

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

static void axera_bl1_chipmode_reset(void *opaque)
{
    address_space_stl_le(&address_space_memory, AXERA_CHIPMODE_SW_ADDR,
                         AXERA_CHIPMODE_EMMC_BOOT_8BIT,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_le(&address_space_memory, AXERA_VDET_RO0_ADDR,
                         AXERA_VDET_3V3_BIT,
                         MEMTXATTRS_UNSPECIFIED, NULL);
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
     * VIN/CSI2/TDP: the real open driver (open_vin_capture.c,
     * open_vin_csi2.c) is a huge, bring-up-derived register protocol
     * (clock-mux "golden" settle values, per-channel WDMA trigger/seq/
     * addr/enable, multiple interrupt groups) -- faithfully modelling
     * frame DMA is future work (see docs/axera.md), but a plain
     * regbank at least lets the driver's many write-then-poll-for-
     * settle loops see what they wrote, instead of always reading 0
     * and spinning to a timeout.
     */
    { "axera-vin",          0x02400000, 0x100000 },
    { "axera-mipi-csi",     0x02600000, 0x4000 },
    { "axera-mipi-dphy",    0x023f0000, 0x1000 },
    { "axera-tdp",          0x04405000, 0x1000 },
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
    { "axera-i2c0",         0x04850000, 0x400 },
    { "axera-i2c7",         0x04857000, 0x400 },
    { "axera-wdt0",         0x04840000, 0x400 },
    { "axera-eth0",         0x104c0000, 0x10000 },
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

    /* TIMER_BASE @ 0x04830000 (boot/bl1/driver/include/timer.h): BL1's
     * delay()/mdelay()/udelay() -- and a timer-ratio helper called from
     * a tight retry loop very early in boot -- read this free-running
     * down-counter twice and spin until it's advanced, with no timeout
     * of their own (see hw/misc/axera-bl1-timer.c). A passive regbank
     * can never satisfy that: two reads of inert storage are always
     * equal, so BL1 spins here forever before anything else runs. */
    sysbus_create_simple(TYPE_AXERA_BL1_TIMER, 0x04830000, NULL);

    /* dpll_ctrl_glb @ 0x00265000, the DDR system DPLL (a separate PLL
     * instance/register layout from clk_pllc above, confirmed from the
     * real vendor source and a gdb trace of a silent flash-file= SPI
     * NOR boot): BL1 enables it and spins on two separate ready-bit
     * polls before DDR training can even start (see
     * hw/misc/axera-ddr-pll.c). */
    sysbus_create_simple(TYPE_AXERA_DDR_PLL, 0x00265000, NULL);

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
 */
static void axera_attach_sdhci(hwaddr base, qemu_irq irq, int unit,
                               bool emmc)
{
    DeviceState *dev = qdev_new(TYPE_CADENCE_SDHCI);
    DriveInfo *di = drive_get(IF_SD, 0, unit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    if (di) {
        CadenceSDHCIState *sdhci = CADENCE_SDHCI(dev);
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
    .uart0_base        = AXERA_UART0_BASE,
    .uart0_irq         = AXERA_UART0_IRQ,
};

static void axera_a64_init(MachineState *machine, const AxeraSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    unsigned int smp_cpus = machine->smp.cpus;
    static struct arm_boot_info binfo;
    qemu_irq pic[AXERA_NUM_SPI];
    DeviceState *gic;
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

        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                      &error_fatal);
        }
        /*
         * The vendor ATF hands the kernel EL1 and answers PSCI over SMC
         * (the dtsi's psci node: method = "smc"); CPU1 comes up by
         * CPU_ON, per the comment above that node about the AX630C
         * firmware's spin-table-less boot.
         */
        object_property_set_bool(cpuobj, "has_el3", want_el3, &error_fatal);
        object_property_set_bool(cpuobj, "has_el2", false, &error_fatal);
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

    create_unimplemented_device("axera-periph", AXERA_PERIPH_BASE,
                                 AXERA_PERIPH_SIZE);
    axera_map_regbanks();

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

    /* crypto@4900000, "axera,ax630c-eip130", GIC_SPI 110. Real AES
     * ECB/CBC/CTR via qcrypto, not just a satisfied probe -- see
     * hw/misc/axera-eip130.c. */
    sysbus_create_simple(TYPE_AXERA_EIP130, 0x04900000, pic[110]);

    /* venc@4010000/jenc@4000000/vdec@4020000: the VeriSilicon VCMD
     * command-queue front-end (probe/submit/wait/complete handshake is
     * real; the linked-list command-buffer content itself -- the part
     * that would drive a real libavcodec encode/decode -- is not
     * decoded, see hw/misc/axera-vcmd.c). */
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04010000, pic[93]);
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04000000, pic[91]);
    sysbus_create_simple(TYPE_AXERA_VCMD, 0x04020000, pic[92]);

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
        if (ams->flash_file && ams->flash_file[0] && !machine->kernel_filename) {
            DeviceState *spi_nor = qdev_new(TYPE_AXERA_SPI_NOR);

            qdev_prop_set_string(spi_nor, "file", ams->flash_file);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(spi_nor), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(spi_nor), 0, 0x01a00000);
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
            qemu_register_reset(axera_bl1_chipmode_reset, NULL);
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
     * AX620Q's own dtsi only documents these three blocks (no GPIO,
     * pinctrl or VPU/ISP nodes at all -- a much smaller feature set than
     * AX630C/AX620E's), at the same addresses as the aarch64 family.
     * AX620U ("axera,ax620x") is a different SoC design and its clock
     * controller addresses are not confirmed, so it gets none of these
     * -- only the ISP/NPU blocks a real AX620U firmware's own devicetree
     * actually showed (same addresses as AX630C/AX620E's family, smaller
     * register windows).
     */
    if (!c->is_ax620x) {
        static const struct {
            const char *name;
            hwaddr base;
            uint32_t size;
        } ax620q_regbanks[] = {
            { "axera-clk-cpu",    0x01900000, 0x1000 },
            { "axera-clk-pllc",   0x02210000, 0x1000 },
            { "axera-clk-common", 0x02340000, 0x1000 },
        };
        int r;

        for (r = 0; r < ARRAY_SIZE(ax620q_regbanks); r++) {
            DeviceState *dev = qdev_new("hisi-regbank");

            qdev_prop_set_uint32(dev, "size", ax620q_regbanks[r].size);
            qdev_prop_set_string(dev, "name", ax620q_regbanks[r].name);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                    ax620q_regbanks[r].base, 1);
        }

        /*
         * spi2 @ 0x06072000 (same address as AX630C/AX620E's, confirmed
         * from arch/arm/dts/ax620e.dtsi's shared spi2 node): AX620Q's own
         * boot logo also drives a SPI-attached panel through this same
         * poll_transfer() loop with no timeout (see hw/misc/axera-dw-spi.c)
         * -- a passive regbank hangs the boot dead right after relocation,
         * same shape as the aarch64 family's panel hang.
         */
        sysbus_create_simple(TYPE_AXERA_DW_SPI, 0x06072000, NULL);
    } else {
        /*
         * AX620U's real devicetree (from a deployed camera firmware,
         * same extraction method as AX620E's): isp@2400000
         * "axera,proton" reg <0x100000>; ax_npu@0x3800000 "axera,npu"
         * reg <0x3800000 0x7000>, <0x3000000 0x500000> -- smaller
         * windows than AX630C/AX620E's, confirmed from the real DT, not
         * assumed to match.
         */
        create_unimplemented_device("axera-vin", 0x02400000, 0x100000);
        create_unimplemented_device("axera-npu", 0x03800000, 0x7000);
        create_unimplemented_device("axera-npu-sram", 0x03000000, 0x500000);

        /*
         * secondary-boot-reg @ 0x02220300 (real devicetree value, see
         * ax620u_soc's comment): the primary CPU's write of the
         * secondary-CPU entry address has to actually stick so the
         * spin stub's poll loop can read it back, unlike the
         * create_unimplemented_device catch-all underneath, which
         * always discards writes.
         */
        DeviceState *smpreg = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(smpreg, "size", 0x1000);
        qdev_prop_set_string(smpreg, "name", "axera-ax620u-smp-bootreg");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(smpreg), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(smpreg), 0,
                                c->smp_bootreg_addr & ~0xfffULL, 1);

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
         */
        DeviceState *clk = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(clk, "size", 0x20000);
        qdev_prop_set_string(clk, "name", "axera-ax620u-clk");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(clk), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(clk), 0, 0x049a0000, 1);
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

    serial_mm_init(sysmem, c->uart0_base, 2, pic[c->uart0_irq],
                   AXERA_UART0_FREQ / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    /*
     * AX620Q's own real vendor U-Boot (CONFIG_CONS_INDEX=4,
     * CONFIG_SYS_NS16550_COM4 in include/configs/ax620e_common.h) puts
     * its console on a fourth, separate NS16550 instance at 0x06080000,
     * not on uart0_base -- confirmed by GDB-tracing a silent boot straight
     * into eserial4_init()/NS16550_init(com_port=0x6080000). Not in
     * AX620Q's own documented dtsi blocks (board bring-up only, no IRQ
     * confirmed from any real devicetree), so no interrupt line.
     */
    if (!c->is_ax620x) {
        serial_mm_init(sysmem, 0x06080000, 2, NULL,
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
