/*
 * HiSilicon IP camera SoC family definitions.
 *
 * Table-driven configuration for all supported SoC variants.
 * Add new SoCs by defining a HisiSoCConfig instance — the shared
 * hisilicon_common_init() handles the rest.
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_HISILICON_H
#define HW_ARM_HISILICON_H

#include "qemu/units.h"
#include "exec/hwaddr.h"

/* Maximum peripheral counts */
#define HISI_MAX_UARTS    5
#define HISI_MAX_TIMERS   4
#define HISI_MAX_SPIS     4
#define HISI_MAX_HIMCI    3
#define HISI_MAX_SDHCI    3
#define HISI_MAX_I2C      8
#define HISI_MAX_REGBANKS 20
#define HISI_MAX_CRG_DEFAULTS 8
#define HISI_MAX_GPIO_EXTRAS 4

typedef struct HisiRegbankEntry {
    const char *name;
    hwaddr      base;     /* 0 = skip */
    uint32_t    size;
} HisiRegbankEntry;

typedef struct HisiSoCConfig {
    const char     *name;
    const char     *desc;
    const char     *cpu_type;       /* full QOM type, set at runtime */
    uint32_t        soc_id;
    ram_addr_t      ram_size_default;
    int             max_cpus;       /* 0 = 1 (default) */
    int             default_cpus;   /* 0 = QEMU default (1); else cores at boot */

    /*
     * Default image sensor model name, applied when the user did not
     * pass -machine sensor=... on the command line.  Should match the
     * sensor wired to the typical OpenIPC reference board for this SoC,
     * so vanilla firmware boots out-of-the-box.  NULL = no default
     * (user must specify explicitly to attach any sensor).
     */
    const char     *default_sensor;

    /*
     * Size in MiB the Linux kernel is told to use via "mem=<N>M".
     * OpenIPC's /usr/bin/load_hisilicon defaults to osmem=32 (and rejects
     * boots where mem= exceeds the totalmem U-Boot env, which is unset on
     * QEMU and falls back to 64), so 32 is the safe value across chips.
     * Setting this to 0 leaves the kernel with the full ram_size_default.
     */
    uint32_t        kernel_mem_mb;

    /*
     * Extra kernel command-line arguments injected by machine init
     * — the HiSilicon-patched kernel reserves a CMA region from the
     * mmz= hint, and the vendor mmz.ko / open_osal.ko then claim it.
     * Pin MMZ at ram_base+32M with size = ram_size - 32M to match the
     * canonical OpenIPC layout (osmem=32, mmz=rest).  NULL = none.
     */
    const char     *extra_cmdline;

    /* Memory regions */
    hwaddr          ram_base;
    hwaddr          sram_base;
    size_t          sram_size;

    /* Interrupt controller — VIC or GIC, selected by use_gic */
    bool            use_gic;
    hwaddr          vic_base;       /* PL190, when !use_gic */
    hwaddr          gic_dist_base;  /* GICv2 dist / GICv3 GICD, when use_gic */
    hwaddr          gic_cpu_base;   /* GICv2 only */
    int             gic_num_spi;
    /* GIC architecture version: 0/2 = GICv2 (default), 3 = GICv3.  GICv3
     * (e.g. Hi3519DV500/Hi3516DV500) has a redistributor instead of a CPU
     * interface MMIO; gic_cpu_base is then ignored and gic_redist_base used. */
    int             gic_version;
    hwaddr          gic_redist_base; /* GICv3 GICR base, when gic_version == 3 */
    /* Cortex-A9 family: use combined a9mpcore_priv (SCU + GIC + gtimer +
     * mptimer + wdt at MPCORE_PERIPHBASE+0x000/0x100/0x200/0x600/0x1000).
     * When set, gic_dist_base / gic_cpu_base are ignored.  Plain GIC is
     * used otherwise (A7-family pattern). */
    hwaddr          gic_mpcore_base; /* 0 = plain GIC */

    /* System controller + CRG */
    hwaddr          sysctl_base;
    hwaddr          crg_base;

    /*
     * SCSYSID register layout selector.
     *   false — V4+: SCSYSID0 returns the full 32-bit soc_id as a word.
     *   true  — V1/V2/V2A/V3/V3A: SCSYSID0..3 each return one byte of
     *           soc_id (low byte first), matching real silicon and the
     *           layout expected by vendor V3 sys.ko / ipctool.
     */
    bool            chipid_byte_layout;

    /*
     * Chip sub-variant byte placed in SCSYSID0[31:24] when
     * chipid_byte_layout is set.  ipctool / vendor sys.ko use it to
     * disambiguate SoCs that share a family ID — e.g. 0x3518E200
     * is CV200 (variant 1), 18EV200 (2), or 18EV201 (3).  A value of
     * 0 covers CV300, AV100 and 19V101 whose default sub-variant maps
     * to the expected chip name.
     */
    uint8_t         chip_variant;

    /*
     * V1-family chip identification at non-standard sysctl offsets.
     * ipctool's get_chip_V1() reads SCSYSID0 (giving the family ID
     * 0x35180100) and then disambiguates the four V1 variants via
     * sysctl_base + 0x88 and sysctl_base + 0x8C — see hisi-sysctl.c
     * and ipctool's hal_hisi.c get_chip_V1().  Both default to 0
     * (= unmatched) for non-V1 SoCs.
     */
    uint32_t        v1_chip_id_88; /* 1=CV100, 2=18EV100, 3=18AV100 */
    uint32_t        v1_chip_id_8c; /* bits[14:8]: 0x10=18CV100, 0x57=18EV100 */

    /*
     * V1 flash boot runs U-Boot in place from the SPI NOR window, the way
     * the silicon does, instead of copying it to DDR first.  Its start.S
     * only walks the reg_info table at image offset 0x40 (PLL, DDR
     * controller, pin mux) when it finds itself outside DDR, so a copy
     * to DDR skipped that table and an image whose table was empty booted
     * here and bricked real boards (OpenIPC/firmware#2593).  It also
     * creates the hisi-ddrc-v1 controller: DDR reads as zero and drops
     * writes until the table has started the DDR PHY.
     */
    bool            v1_xip_boot;

    /*
     * CRG register that reports PLL lock, one bit per PLL, and the bits
     * that read as locked.  V1 reg_info tables poll CRG+0xe8 for 0xf
     * after programming the four PLLs.  0 = no such register.
     */
    uint32_t        crg_pll_lock_offset;
    uint32_t        crg_pll_lock_bits;

    /* UARTs (PL011) */
    int             num_uarts;
    hwaddr          uart_bases[HISI_MAX_UARTS];
    int             uart_irqs[HISI_MAX_UARTS];

    /* Timers (SP804) */
    int             num_timers;
    hwaddr          timer_bases[HISI_MAX_TIMERS];
    int             timer_irqs[HISI_MAX_TIMERS];
    uint32_t        timer_freq;     /* 0 = device default */
    /*
     * ARM generic timer (CNTFRQ) rate.  QEMU's Cortex-A7 counter runs at
     * 62.5 MHz unless told otherwise, but vendor DTS arm,armv7-timer nodes
     * pin clock-frequency (50 MHz V4/V4A, 24 MHz V3A), and the kernel then
     * converts counter ticks to time at the DT rate.  0 = QEMU default.
     */
    uint32_t        arch_timer_freq;
    /*
     * Goke V500 "xmsp804": one hisi-xmsp804 device (4 single-timer blocks at
     * 0x100 stride) at timer_bases[0] instead of the stock dual-timer sp804s.
     * When set, the sp804 loop above is skipped.  See hisi-xmsp804.c.
     */
    bool            xmsp804_timer;

    /* SPI (PL022) */
    int             num_spis;
    hwaddr          spi_bases[HISI_MAX_SPIS];
    int             spi_irqs[HISI_MAX_SPIS];

    /* Flash Memory Controller (HiFMC or HISFC350) */
    hwaddr          fmc_ctrl_base;  /* 0 = no flash controller */
    hwaddr          fmc_mem_base;
    const char     *fmc_type;       /* NULL = "hisi-fmc", or "hisi-sfc350" */
    /* hisi-fmc register-map variant: 0 = HiFMC V100 (default), 1 = fmc100
     * ("lotus,fmc" on Goke xmorca / GK7206 — same IP, shuffled offsets). */
    uint32_t        fmc_variant;
    /* FMC op/DMA-done IRQ (GIC SPI).  0 = unconnected (V100 users poll);
     * the fmc100 driver waits on an interrupt for DMA completion. */
    int             fmc_irq;

    /* GPIO (PL061) */
    hwaddr          gpio_base;
    int             gpio_count;
    int             gpio_stride;    /* address step between ports (0x1000 or 0x10000) */
    int             gpio_irq;       /* VIC: shared IRQ for all ports */
    int             gpio_irq_start; /* GIC: first SPI, one per port */
    uint32_t        gpio0_input_high; /* mask of GPIO0 input pins to drive high
                                       * (board pull-ups, e.g. update button) */

    /*
     * Extra GPIO ports at non-contiguous addresses or with
     * non-sequential IRQs (e.g. AV100 port 15, CV500 port 11).
     * An entry with .base == 0 terminates the list.
     */
    struct { hwaddr base; int irq; } gpio_extras[HISI_MAX_GPIO_EXTRAS];

    /* DMA (PL080 by default; "hisi-regbank" stubs the DVR/NVR HiSilicon DW
     * DMAC instead, since a PL080 at the same address responds with PL080
     * PrimeCell IDs that the vendor hisi-dmac driver rejects, then probes
     * forever).  NULL = "pl080". */
    hwaddr          dma_base;       /* 0 = no DMA controller */
    int             dma_irq;
    const char     *dma_type;       /* NULL = "pl080", or "hisi-regbank" */

    /* FEMAC (Fast Ethernet MAC) */
    hwaddr          femac_base;     /* 0 = no FEMAC */
    int             femac_irq;

    /* GMAC (Gigabit Ethernet MAC) — for AV100, 3519V101 */
    hwaddr          gmac_base;      /* 0 = no GMAC */
    int             gmac_irq;
    /* Descriptor stride in bytes (0 = device default of 32).  Set to 16
     * for SoCs whose vendor higmacv300 driver defines
     * CONFIG_HIGMAC_DESC_4_WORD — currently hi3536cv100. */
    uint32_t        gmac_desc_size;
    /* RX DMA offset (NET_IP_ALIGN) the MAC writes each frame at.  0 (default)
     * for hi_gmac_v200; 2 for the gmac-v5 driver (Hi3519DV500/Hi3516DV500). */
    int             gmac_rx_offset;

    /* SD/MMC — himciv200 (older SoCs) */
    int             num_himci;
    hwaddr          himci_bases[HISI_MAX_HIMCI];
    int             himci_irqs[HISI_MAX_HIMCI];

    /* SD/MMC — SDHCI (newer SoCs) */
    int             num_sdhci;
    hwaddr          sdhci_bases[HISI_MAX_SDHCI];
    int             sdhci_irqs[HISI_MAX_SDHCI];
    /* PRNSTS bit 19 the other way round from the SDHCI spec, set when the
     * kernel's driver carries SDHCI_QUIRK_INVERTED_WRITE_PROTECT: without it
     * a writable card reads back as write-protected and mounts read-only. */
    bool            sdhci_wp_inverted;

    /* I2C (HiBVT for V2+, register-poll "hisi-i2c-v1" for CV100 family) */
    int             num_i2c;
    hwaddr          i2c_bases[HISI_MAX_I2C];
    const char     *i2c_type;       /* NULL = "hisi-i2c", or "hisi-i2c-v1" */
    /* GIC SPI of each controller; 0 = not wired. A kernel whose device tree
     * gives the controllers interrupts runs i2c-hibvt in interrupt mode and
     * waits for them -- 1 s per message, then "rx/tx timeout", when the line
     * goes nowhere. */
    int             i2c_irqs[HISI_MAX_I2C];

    /* MIPI RX (CSI-2 / LVDS receiver) */
    hwaddr          mipi_rx_base;   /* 0 = no MIPI RX */
    int             mipi_rx_irq;

    /* RTC (SPI-bridge internal RTC) */
    hwaddr          rtc_base;       /* 0 = no RTC */
    int             rtc_irq;

    /* VEDU (Video Encoder) + JPGE */
    hwaddr          vedu_base;      /* 0 = no VEDU */
    hwaddr          jpge_base;
    int             vedu_irq;
    int             jpge_irq;

    /* Watchdog (SP805 / cmsdk-apb-watchdog) */
    hwaddr          wdt_base;       /* 0 = no watchdog */
    int             wdt_irq;        /* -1 = no IRQ */
    uint32_t        wdt_freq;       /* clock frequency in Hz */

    /* Hardware GZIP decompressor */
    hwaddr          gzip_base;      /* 0 = no GZIP engine */

    /* SATA AHCI controller (sysbus-ahci) — DVR/NVR family.
     * Vendor SATA module busy-loops if the controller is absent; even a
     * stub regbank is insufficient because libahci probes CAP/PI/HBAR. */
    hwaddr          sata_base;      /* 0 = no SATA */
    int             sata_irq;
    int             sata_num_ports; /* 0 = 1 port (default) */

    /* USB host (sysbus-EHCI + sysbus-OHCI) — DVR/NVR family.
     * Stub-OK: with no devices attached, the kernel cleanly probes and
     * reports an empty bus.  Required because vendor usb_storage / mass-
     * storage modules autoload and busy-loop on a missing controller. */
    hwaddr          usb_ehci_base;  /* 0 = no EHCI */
    int             usb_ehci_irq;
    hwaddr          usb_ohci_base;  /* 0 = no OHCI */
    int             usb_ohci_irq;

    /* USB 3.0 XHCI (sysbus-XHCI) — declared but currently unused.  Will be
     * populated in Phase 3 for Hi3531A / Hi3535 / Hi3536-flagship which
     * expose XHCI. */
    hwaddr          xhci_base;      /* 0 = no XHCI */
    int             xhci_irq;
    int             xhci_slots;     /* 0 = default 4 */
    int             xhci_intrs;     /* 0 = default 1 */

    /* HiSilicon HIL2V200 L2 cache controller — used by Hi3520D family
     * (CONFIG_CACHE_HIL2V200).  Vendor 3.0.x driver hangs in early init
     * polling REG_L2_RINT.AUTO_END after writing maintenance ops.  The
     * `hisi-l2cache` model latches AUTO_END on every maintenance write
     * so the spin-wait completes immediately. */
    hwaddr          l2cache_base;   /* 0 = no L2 cache controller */

    /* PL011 UARTEN pre-enable — set true when the SoC ships vendor Linux
     * 3.0/3.4/3.10 kernels that rely on U-Boot to set UARTCR before
     * jumping to the kernel.  Real boards' U-Boot writes UARTCR=0x301
     * (UARTEN | TXE | RXE).  Without this, those kernels write to UART
     * with UARTEN clear and we see "PL011 data written to disabled UART"
     * with no console output.  Linux 3.18+ and 4.9 don't need this — the
     * PL011 driver's startup path sets UARTCR itself. */
    bool            uart_pre_enable;

    /* PL011 MMIO window size — set when the vendor's mach amba_device
     * resource is sized larger than the PL011's actual 0x1000 register
     * block (Hi3520Dv200's HIL_AMBA_DEVICE macro hardcodes 0x10000).
     * The AMBA bus probe reads PrimeCell ID at `tmp + size - 0x20` and
     * `tmp + size - 0x10`, i.e. at the END of the resource — outside
     * the PL011's modeled 0x1000 region for these SoCs, so periphid
     * reads back as 0 and the AMBA driver fails to bind.  Real silicon
     * mirrors PL011 across the larger window; we model that with an
     * alias subregion at (base + window_size - 0x1000).
     * 0 = use PL011's native 0x1000 (default). */
    uint32_t        uart_window_size;

    /* HiSilicon SF (single-FIFO) Ethernet controller — used by Hi3520D
     * family and other vendor 3.0/3.4 kernels via drivers/net/hieth-sf/.
     * Register layout matches FEMAC byte-for-byte; the `hisi-hieth-sf`
     * model subclasses `hisi-femac` and only overrides the PHY MDIO
     * address (3 instead of 1, where vendor `drivers/net/hieth-sf/
     * mdio.c` scans the himii bus).  Full TX/RX rings and IRQ path
     * are inherited from the FEMAC model. */
    hwaddr          hieth_sf_base; /* 0 = no HiSilicon SF Ethernet */
    int             hieth_sf_irq;  /* GIC SPI for the hieth-sf controller */

    /* Hardware True Random Number Generator
     * (HISEC_TRNG_CTRL on V3+, RNG_GEN on V2) */
    hwaddr          hwrng_base;        /* 0 = no HWRNG on this SoC */
    uint32_t        hwrng_data_offset; /* 0x204 V3+, 0x004 V2 */

    /* VI frame producer — periodic VI/VPSS IRQ pulses to wake the
     * vendor MPP pipeline so VENC stops timing out.  MVP scaffolding,
     * wired only on hi3516ev300 for now. */
    hwaddr          vi_fp_base;        /* 0 = no frame producer */
    int             vi_fp_cap_irq;     /* GIC SPI for VI_CAP0 */
    int             vi_fp_proc_irq;    /* GIC SPI for VI_PROC0 */
    int             vi_fp_vpss_irq;    /* GIC SPI for VPSS */
    int             vi_fp_vedu_irq;    /* GIC SPI for VEDU */

    /* CPU soft-reset register offset in CRG (for SMP bringup) */
    uint32_t        cpu_srst_offset; /* 0 = disabled, e.g. 0x78 for CV500 */

    /* ATAGs board ID (machine_arch_type) for non-DT boots.  0 = unset; the
     * kernel uses DT compatible string instead (preferred for V3+ SoCs).
     * Required for older Linux 3.10 board-config kernels (e.g. Hi3536). */
    uint32_t        board_id;

    /* PSCI conduit: 0 = DISABLED (V1–V5 IPC + DVR/NVR), or set to a non-zero
     * value to indicate ARMv8 STB family (Hi3798CV200) which uses PSCI/SMC
     * for SMP bringup.  When set, the upstream "smpboot" stub at 0x0 is
     * suppressed (it would collide with the bootloader stub) and secondary
     * CPUs are released by PSCI calls into QEMU's SMC handler. */
    int             psci_conduit;

    /* ARMv8/aarch64 V5 SoCs (Cortex-A55, Hi3519DV500/Hi3516DV500): boot the
     * gzip self-extracting u-boot-z.bin directly from DDR instead of the
     * ARMv7/v5 flash->DDR copy stub.  uboot_load_addr is the image link/entry
     * address (CONFIG_SYS_TEXT_BASE_ORI, e.g. 0x48700000). */
    bool            aarch64;
    hwaddr          uboot_load_addr;

    /* CRG register defaults (mimics U-Boot clock init before kernel boot) */
    int             num_crg_defaults;
    struct { uint32_t offset; uint32_t value; } crg_defaults[HISI_MAX_CRG_DEFAULTS];

    /* Generic RAM-backed register banks (pin mux, DDR PHY, PWM, etc.) */
    int             num_regbanks;
    HisiRegbankEntry regbanks[HISI_MAX_REGBANKS];
} HisiSoCConfig;

/*
 * SoC identification values — written to SCSYSID register at
 * sysctl_base + 0xEE0.  Software (U-Boot, kernel, ipctool)
 * reads this to auto-detect the chip model.
 */
#define HISI_SOC_ID_CV300       0x3516C300
#define HISI_SOC_ID_EV200       0x3516E200
#define HISI_SOC_ID_EV300       0x3516E300
#define HISI_SOC_ID_18EV300     0x3518E300
#define HISI_SOC_ID_DV200       0x3516D200

/* V1 generation */
#define HISI_SOC_ID_CV100       0x35180100

/* V2 generation */
#define HISI_SOC_ID_CV200       0x3518E200

/* V2A generation (Cortex-A7 + GIC, V1-era 0x20xxxxxx address map) */
#define HISI_SOC_ID_AV100       0x3516A100

/* V3.5 generation (Cortex-A7 + GIC, but V3-era address map) */
#define HISI_SOC_ID_CV500       0x3516C500

/* V4A generation — same architecture as CV500 with 4K + bigger NPU */
#define HISI_SOC_ID_AV300       0x3516A300

/* V4A generation — DV300, sibling of AV300/CV500 (per ipctool's
 * hal_hisi.c chip-id table: case 0x3516D300 → "3516DV300", HISI_V4A). */
#define HISI_SOC_ID_DV300       0x3516D300

/* V3A generation (Cortex-A7/A17 big.LITTLE + GIC, V3-era address map) */
#define HISI_SOC_ID_19V101      0x35190101
/* AV200 shares 19V101's family ID; sub-variant byte 5/6/0x15/0x16
 * in SCSYSID0 distinguishes 3516AV200 from 3519V101 (0/1/2/0x11/0x12). */

/* DVR/NVR family — surveillance back-end SoCs (separate product line from IPC).
 * Word-layout SCSYSID0, same encoding as V4 IPC.  Hi3536DV100 / Hi3536CV100 /
 * Hi3521V100 confirmed against ipctool's hal_hisi.c chip-id table; Hi3531A
 * value is a placeholder (vendor SDK does not read SCSYSID for boot). */
#define HISI_SOC_ID_3536DV100   0x3536D100
#define HISI_SOC_ID_3536CV100   0x3536C100
#define HISI_SOC_ID_3520DV200   0x3520D100  /* yes, V200 reports 0x3520D100 */
#define HISI_SOC_ID_3521V100    0x35210100
#define HISI_SOC_ID_3531A       0x35310100  /* placeholder; verify on hardware */
#define HISI_SOC_ID_3536        0x35360100  /* Hi3536 flagship; placeholder */
#define HISI_SOC_ID_3521DV100   0x3521D100  /* placeholder; H.265 dual A7 sibling of Hi3521A */
#define HISI_SOC_ID_3520DV300   0x3520D300  /* placeholder; A7 sibling of Hi3521A */
#define HISI_SOC_ID_3520DV400   0x3520D400  /* placeholder; A7 1.3GHz H.265 single */
#define HISI_SOC_ID_3531DV100   0x3531D100  /* placeholder; A9 H.265 sibling of Hi3531A */
#define HISI_SOC_ID_3535        0x35350100  /* placeholder; A9 dual NVR-only */
#define HISI_SOC_ID_3798CV200   0x3798C200  /* Hi3798CV200 STB; placeholder */
#define HISI_SOC_ID_3796MV100   0x3796D100  /* Hi3796M V100 STB; placeholder */

/*
 * V5 generation (Cortex-A7 MP2 + GIC, new 0x11xxxxxx address map, ~2023)
 * Same die, different feature tiers.  Model suffixes:
 *   10B=CV610, 20S=CV613, 00S/20G/00G=unknown IDs (not yet in lab)
 * Also HISI_OT: Hi3516DV500 (0x3516D500), Hi3519DV500 (0x3519D500)
 */
#define HISI_SOC_ID_CV608       0x3516C608  /* consumer, 0.2 TOPS, 3M */
#define HISI_SOC_ID_CV610       0x3516C610  /* 10B: 0.5 TOPS, 5M */
#define HISI_SOC_ID_CV613       0x3516C613  /* 20S: 1 TOPS, 4K */

/* V5 HISI_OT aarch64 die (Cortex-A55, ~2024) — shares the V5 0x11xxxxxx
 * peripheral map with CV610 but is 64-bit and boots a gzip-compressed u-boot. */
#define HISI_SOC_ID_DV500       0x3519D500  /* Hi3519DV500 */
#define HISI_SOC_ID_3516DV500   0x3516D500  /* Hi3516DV500 */

/* Goke chips — die-identical V4 silicon with different SoC IDs */
#define GOKE_SOC_ID_7205V200    0x72050200  /* = hi3516ev200 */
#define GOKE_SOC_ID_7205V300    0x72050300  /* = hi3516ev300 */
#define GOKE_SOC_ID_7202V300    0x72020300  /* = hi3518ev300 */
#define GOKE_SOC_ID_7605V100    0x76050100  /* = hi3516dv200 */
#define GOKE_SOC_ID_7201V200    0x72010200  /* ~hi3516ev200 stripped (1M/2M IPC) */

/* Goke next-gen — own design, V4-compatible address map + NPU (2022+) */
#define GOKE_SOC_ID_7205V500    0x72050500  /* 5M, 0.5 TOPS, 512Mb DDR2 */
#define GOKE_SOC_ID_7205V510    0x72050510  /* 5M, 1.0 TOPS, 1Gb DDR3 */
#define GOKE_SOC_ID_7205V530    0x72050530  /* 5M, 1.0 TOPS, ext DDR */
#define GOKE_SOC_ID_7202V330    0x72020330  /* 5M, 0.5 TOPS, no FEPHY */

/*
 * Goke third generation (2025-2026) — Goke's own designs on the XMedia
 * "Lotus" SDK (ShiMetaPi shimetapi_pico_gx), split into two product lines
 * the way HiSilicon splits 3516 (mainstream) / 3519 (high-end):
 *   GK7206 — codename "xmorca",  Cortex-A7 MP2, embedded DDR, 5M (V10) / 4K (V11+).
 *   GK7606 — codename "xmfalcon", Cortex-A55, external DDR, 4K.
 * Both keep the gk7205v5xx-style 0x12xxxxxx control block (sysctl
 * 0x12020000, CRG 0x12010000, UART 0x12040000, xmedia,sp804 0x12000000),
 * RAM @0x40000000, an ATF BL1->BL31->BL33(u-boot) chain, and a RISC-V MCU
 * coprocessor.  The chip ID is read from REG_SC_CHIPID = sysctl_base + 0xEE0
 * (= 0x12020EE0, verified in the SDK u-boot arch-xmorca/platform.h).  The
 * ID *values* below are PLACEHOLDERS pending a live SCSYSID/CHIPID capture —
 * the xmorca values are not in the SDK source (u-boot and the kernel just
 * print whatever they read).  They follow the encoding the SDK kernel's
 * lotus/machine/xmfalcon/include/platform.h uses for its siblings
 * (CHIPID_XM7605V12 = 0x76050120, CHIPID_XM7606V13 = 0x76060130): part
 * number, then the Vxy version as 0xxy0.  The "A" suffix's encoding is
 * unknown.  The 5.10 kernel only prints the xmorca ID (/proc/soc_info);
 * no driver branches on it. */
#define GOKE_SOC_ID_7206V10     0x72060100  /* GK7206V10, dual A7 MP2, 1.0 TOPS; placeholder */
#define GOKE_SOC_ID_7206V12A    0x72060120  /* GK7206V12A, 2Gb DDR3L, 4K; placeholder */
#define GOKE_SOC_ID_7606V1      0x76060100  /* GK7606V1, A55 4K; placeholder */

#endif /* HW_ARM_HISILICON_H */
