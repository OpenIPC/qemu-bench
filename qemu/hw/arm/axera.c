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
#include "hw/misc/axera-dw-spi.h"
#include "hw/misc/axera-eip130.h"
#include "hw/misc/axera-pllc.h"
#include "hw/misc/axera-vcmd.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/cadence_sdhci.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-dwc3.h"
#include "qapi/visitor.h"
#include "system/address-spaces.h"
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

/*
 * BL1/SPL's real link address, read directly from the vendor SDK's own
 * linker script (boot/bl1/board/arch/arm64/bl1.lds): a 1 MiB internal
 * ROM/SRAM window, not DDR -- ENTRY(_start), MEMORY { iRom (rx) :
 * ORIGIN = 0x03000400, LENGTH = 0x100000 }.
 */
#define AXERA_BL1_BASE          0x03000400ULL
#define AXERA_BL1_SIZE          (1 * MiB)

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
    { "axera-tsensor",      0x02000000, 0x1000 },
    { "axera-clk-cpu",      0x01900000, 0x10000 },
    { "axera-clk-common",   0x02340000, 0x10000 },
    { "axera-dphytx",       0x023f1000, 0x1000 },
    { "axera-isp-syscon",   0x02500000, 0x10000 },
    { "axera-clk-flash",    0x10030000, 0x10000 },
    { "axera-clk-periph",   0x04870000, 0x10000 },
    { "axera-dispc-reset",  0x04600000, 0x100 },
    /*
     * Not in any Linux-visible devicetree node -- found by tracing BL1's
     * own flash-boot execution (qemu -d unimp): BL1 writes 0xffffffff
     * here then polls/kicks +8 before anything else, the classic
     * "disable/kick the watchdog at reset" shape. Likely a secure-
     * world-only early watchdog distinct from the wdt0 Linux sees at
     * 0x04840000 (0x10000 away) -- not confirmed against vendor source,
     * just the observed access pattern.
     */
    { "axera-bl1-wdt",      0x04830000, 0x100 },
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
    { "axera-npu-sram",     0x03000000, 0x280000 },
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
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
        object_property_set_bool(cpuobj, "has_el2", false, &error_fatal);
        object_property_set_int(cpuobj, "psci-conduit",
                                 QEMU_PSCI_CONDUIT_SMC, &error_fatal);
        object_property_set_uint(cpuobj, "cntfrq", AXERA_CNTFRQ,
                                  &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }

    memory_region_add_subregion(sysmem, AXERA_RAM_BASE, machine->ram);

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
     * compatible string's register set.
     */
    sysbus_create_simple(TYPE_CADENCE_SDHCI, 0x01b40000, pic[9]);
    sysbus_create_simple(TYPE_CADENCE_SDHCI, 0x104e0000, pic[74]);
    sysbus_create_simple(TYPE_CADENCE_SDHCI, 0x104d0000, pic[73]);

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
             * A small RAM region backs 0x03000400 (inside the broader
             * axera-periph unimplemented catch-all) so the injected
             * code is actually executable, at normal priority so it
             * wins over that -1000-priority catch-all underneath.
             */
            MemoryRegion *bl1_ram = g_new(MemoryRegion, 1);
            char *bl1_data = NULL;
            gsize bl1_len = 0;

            if (!g_file_get_contents(ams->flash_file, &bl1_data, &bl1_len,
                                     NULL)) {
                error_report("axera: %s: could not read flash-file '%s'",
                              MACHINE_GET_CLASS(machine)->name,
                              ams->flash_file);
                exit(1);
            }

            memory_region_init_ram(bl1_ram, NULL, "axera.bl1ram",
                                   AXERA_BL1_SIZE, &error_fatal);
            memory_region_add_subregion(sysmem, AXERA_BL1_BASE, bl1_ram);

            rom_add_blob_fixed("axera.bl1",
                               bl1_data,
                               MIN(bl1_len, (gsize)AXERA_BL1_SIZE),
                               AXERA_BL1_BASE);
            g_free(bl1_data);

            AxeraBootromReset *info = g_new0(AxeraBootromReset, 1);
            info->cpu = cpu[0];
            info->entry = AXERA_BL1_BASE;
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

static void axera_32_init(MachineState *machine, const AxeraSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    static struct arm_boot_info binfo;
    qemu_irq pic[AXERA_NUM_SPI];
    DeviceState *gic;
    SysBusDevice *gicbus;
    ARMCPU *cpu0;
    Object *cpuobj;
    int n, i;

    cpuobj = object_new(machine->cpu_type);
    object_property_set_uint(cpuobj, "cntfrq", AXERA_CNTFRQ, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu0 = ARM_CPU(cpuobj);

    memory_region_add_subregion(sysmem, AXERA_RAM_BASE, machine->ram);

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
    }

    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", 1);
    qdev_prop_set_uint32(gic, "num-irq", AXERA_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    if (!machine->kernel_filename) {
        ARM_GIC_COMMON(gic)->irq_reset_nonsecure = true;
    }
    sysbus_mmio_map(gicbus, 0, AXERA_GIC_DIST);
    sysbus_mmio_map(gicbus, 1, AXERA_GIC_CPU);

    {
        DeviceState *cpudev = DEVICE(cpu0);
        int ppibase = AXERA_NUM_SPI + GIC_NR_SGIS;
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
        sysbus_connect_irq(gicbus, 0, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(gicbus, 1, qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(gicbus, 2, qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbus, 3, qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    for (n = 0; n < AXERA_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    serial_mm_init(sysmem, c->uart0_base, 2, pic[c->uart0_irq],
                   AXERA_UART0_FREQ / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    if (machine->kernel_filename && !machine->dtb) {
        error_report("axera: %s needs the kernel's device tree (-dtb)",
                      MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AXERA_RAM_BASE;
    arm_load_kernel(cpu0, machine, &binfo);
}

#define DEFINE_AXERA_MACHINE(namestr, tag, config)                    \
    static void tag##_init(MachineState *machine)                     \
    {                                                                 \
        axera_32_init(machine, &config);                               \
    }                                                                 \
    static void tag##_class_init(MachineClass *mc)                    \
    {                                                                 \
        mc->desc = config.desc;                                       \
        mc->init = tag##_init;                                        \
        mc->default_ram_size = config.ram_size_default;                \
        mc->default_ram_id = "axera.ram";                              \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");         \
        mc->max_cpus = 1;                                              \
        mc->default_cpus = 1;                                         \
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
 * secondary-boot-reg at 0x02220300) is a different register than
 * AX620Q's and is not modelled; only CPU0 boots, same stance as AX620Q.
 */
static const AxeraSoCConfig ax620u_soc = {
    .desc              = "Axera AX620U (Cortex-A7 x4, AArch32, CPU1-3 not modelled)",
    .ram_size_default  = 1 * GiB,
    .uart0_base        = 0x04990000ULL,
    .uart0_irq         = 68,
    .is_ax620x         = true,
};

DEFINE_AXERA_MACHINE("ax620u", ax620u, ax620u_soc)
