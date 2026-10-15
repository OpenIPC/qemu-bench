/*
 * HiSilicon IP camera SoC emulation.
 *
 * Table-driven: each SoC variant is a HisiSoCConfig struct.
 * One shared init function handles VIC/GIC, peripherals, boot.
 *
 * 21 machine types across six HiSilicon IPC SoC generations (V1–V5)
 * plus Goke V4 rebrands and Goke next-gen NPU SoCs.
 * See roadmap table below for specs & timeline.
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/char/pl011.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gicv3_common.h"
#include "qobject/qlist.h"
#include "hw/arm/hisilicon.h"
#include "hw/misc/hisi-fastboot.h"
#include "hw/arm/machines-qom.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "elf.h"
#include "hw/sd/sdhci.h"
#include "hw/sd/sd.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "net/net.h"
#include "hw/i2c/i2c.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/gtimer.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include <libfdt.h>
#include "hw/loader.h"
#include <zlib.h>  /* crc32() for uImage header fixup */
#include <glib/gstdio.h>  /* g_stat() for flash-image size sniffing */


/* ── SoC configuration tables ──────────────────────────────────────── */

/*
 * HiSilicon IPC SoC product roadmap — release timeline & key specs
 * (source: vendor roadmap + OpenIPC firmware CI platform groupings)
 * ─────────────────────────────────────────────────────────────────────
 *
 *  Gen   SoC         Year    CPU                 Video            Tier
 *  ───── ─────────── ─────── ─────────────────── ──────────────── ─────
 *  V1    CV100       <2015   ARM926 @550MHz      H.264 1080P@30   2M
 *  V2    CV200       <2015   ARM926 @550MHz      H.264 960P@30    1M
 *  V2A   AV100       <2015   A7 @600MHz          H.265 5M@25      5M
 *        DV100*      <2015   A7 @600MHz          H.265 4M@25      4M
 *  V3    CV300       ~2017   ARM926 @800MHz      H.265 1080P@30   2M
 *        EV100*      ~2018   ARM9 @800MHz        H.265 1080P@20   1M
 *  V3A   3519V101    ~2017   A17@1.2G+A7@800M    H.265 4K@30+2M   4K
 *        AV200*      ~2017   A17@1.2G+A7@800M    H.265 5M@30      5M
 *  V3.5  CV500       2018    Dual A7 @900MHz      H.265/264 3M     3M
 *        AV300*      2019    Dual A7 @900MHz      H.265/264 4K     4K
 *        DV300*      2019    Dual A7 @900MHz      H.265/264 5M     5M
 *  V4    EV200       2018Q4  A7 @900MHz          H.265 3M@20      3M
 *        EV300       2018Q4  A7 @900MHz          H.265 5M@15      5M
 *        18EV300     2018Q4  A7 @900MHz          H.265 3M@20      3M
 *        DV200       2018Q4  A7 @900MHz          H.265 5M@20      5M
 *  V4g   GK7205V200  2021    (die-identical V4 — Goke rebrand)
 *  V4n   GK7205V500  2022    A7 @1GHz            H.265 5M@25      5M+NPU
 *  V5    CV608       ~2023   Dual A7 MP2         H.265 3M         3M
 *        CV610       ~2023   Dual A7 MP2         H.265 5M         5M
 *        CV613       ~2023   Dual A7 MP2         H.265 4K         4K
 *
 *  Entries marked * share a platform SDK with the line above but are
 *  not emulated here.  Hi3519A (4K Smart Vision) and Hi3559A (8K,
 *  2xA73+2xA53) also appear on the roadmap but are not emulated.
 *
 * Platform family groupings (same SDK, same address map):
 *   CV100 platform:  hi3516cv100, hi3518cv100, hi3518ev100
 *   CV200 platform:  hi3516cv200, hi3518ev200
 *   AV100 platform:  hi3516av100, hi3516dv100
 *   CV300 platform:  hi3516cv300, hi3516ev100
 *   3519V101 platf:  hi3519v101, hi3516av200
 *   CV500 platform:  hi3516cv500, hi3516av300, hi3516dv300
 *   EV200 platform:  hi3516ev200, hi3516ev300, hi3518ev300, hi3516dv200
 *   GK7205V200:      gk7205v200, gk7205v210, gk7205v300, gk7202v300, gk7605v100
 *   GK7205V500:      gk7205v500, gk7205v510, gk7205v530, gk7202v330 (NPU)
 *
 * Address map evolution:
 *   V1/V2/V2A: 0x20xxxxxx peripherals, RAM @ 0x80000000
 *   V3/V3A:    0x12xxxxxx peripherals, RAM @ 0x80000000
 *   V3.5:      0x12xxxxxx (variant), GIC @ 0x10301000, RAM @ 0x80000000
 *   V4:        0x12xxxxxx peripherals, RAM @ 0x40000000
 *   V5:        0x11xxxxxx peripherals, GIC @ 0x124xxxxx, RAM @ 0x40000000
 */

/* V1 DDR controller and DDR PHY (inside the misc block), see hisi-ddrc-v1 */
#define HISI_V1_DDRC_BASE       0x20110000
#define HISI_V1_DDRPHY_BASE     0x20120400

/*
 * Hi3516CV100 (V1): pre-2015, 2M mainstream.  ARM926EJ-S @550MHz.
 * Video: H.264, 1080P@30fps.  Memory: 64MB DDR2.
 * Platform family: hi3518cv100, hi3518ev100 (same SDK).
 * VIC at 0x10140000 (not 0x100D0000 like V2).  HISFC350 flash controller
 * (not HiFMC V100) — set fmc_ctrl_base=0 to skip FMC creation.
 * 12 GPIO banks (GPIO0-11).  Single himciv100 SD controller.
 * UART0 and UART1 share IRQ 5.  No SPI (PL022) controllers.
 * Kernel 3.0.8, MACHINE_START (no DT), computes bus clock from BPLL PLL.
 */
static const HisiSoCConfig hi3516cv100_soc = {
    .name               = "hi3516cv100",
    .desc               = "HiSilicon Hi3516CV100 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV100,
    .chipid_byte_layout = true,            /* V1: byte-wise SCSYSID0..3 */
    /* ipctool get_chip_V1(): writes 3 to 0x88, expects 1 read back. */
    .v1_chip_id_88      = 1,
    .default_sensor     = "imx122",        /* Sony 3-wire SPI on V1 ref boards */
    /* 64 MiB on-chip DDR2 (512Mb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims the upper 32 MiB at 0x82000000 — matches OpenIPC V1
     * firmware's load_hisilicon defaults (totalmem=64, osmem=32). */
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,32M",

    .ram_base           = 0x80000000,
    .v1_xip_boot        = true,         /* reg_info brings DDR up */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10140000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,
    .crg_pll_lock_offset = 0xe8,        /* 4 PLL lock bits */
    .crg_pll_lock_bits  = 0xf,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 5, 25 },    /* UART0 and UART1 share IRQ 5 */

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 110000000,    /* BPLL 220 MHz bus / prescale 2 */

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .gpio_base          = 0x20140000,
    .gpio_count         = 12,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x100D0000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 1,
    .himci_bases        = { 0x10020000 },
    .himci_irqs         = { 18 },

    /* V1 register-poll I2C controller (vendor hi_i2c.ko target). */
    .num_i2c            = 1,
    .i2c_bases          = { 0x200D0000 },
    .i2c_type           = "hisi-i2c-v1",

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    /*
     * PLL register defaults: what the OpenIPC U-Boot reg_info table writes,
     * read back from a Hi3516CV100 board (lab hi3516cv100-imx122, CRG0..
     * CRG5 and the CRG+0xe8 lock word).  A flash boot now runs that table
     * (v1_xip_boot), so these have to agree with it or the kernel would
     * derive one timer clock on a -kernel boot and another after U-Boot.
     * Kernel computes: clk = 24M * fbdiv / (2 * refdiv * pstdiv1 * pstdiv2)
     *
     * BPLL (CRG4/CRG5): pstdiv1=1, pstdiv2=2, refdiv=3, fbdiv=110:
     *   24*110/12 = 220 MHz bus.  Timer clock = busclk / prescale(2) =
     *   110 MHz; that board's kernel prints "sched_clock: 32 bits at
     *   110MHz".  APLL (CRG0/CRG1) carries the same words.
     */
    .num_crg_defaults   = 4,
    .crg_defaults       = {
        { 0x00, 0x11000000 },               /* CRG0: APLL post dividers */
        { 0x04, 0x0068306e },               /* CRG1: APLL refdiv/fbdiv  */
        { 0x10, 0x11000000 },               /* CRG4: BPLL post dividers */
        { 0x14, 0x0068306e },               /* CRG5: BPLL refdiv/fbdiv  */
    },

    /*
     * Register bank stubs — vendor .ko modules access video/peripheral
     * hardware during init.  Without mapped regions, reads return 0
     * from QEMU's "unimplemented" handler, causing poll loops to hang.
     */
    .num_regbanks       = 7,     /* DDRC: hisi-ddrc-v1 */
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-vedu",   0x20620000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3518AV100 (V1): die-identical to Hi3516CV100 — same ARM926EJ-S,
 * same V1-era 0x20xxxxxx address map, same HISFC350 flash, same
 * peripheral layout.  Per ipctool's hal_hisi.c get_chip_V1():
 * write 3 to sysctl_base + 0x88 and read back: 3 → "3518AV100".
 * Shipped under the same OpenIPC u-boot-hi3516cv100 tree.
 */
static const HisiSoCConfig hi3518av100_soc = {
    .name               = "hi3518av100",
    .desc               = "HiSilicon Hi3518AV100 (ARM926EJ-S, CV100 sibling)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV100,
    .chipid_byte_layout = true,
    .v1_chip_id_88      = 3,                /* 18AV100 marker */
    .default_sensor     = "imx122",
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,32M",

    .ram_base           = 0x80000000,
    .v1_xip_boot        = true,         /* reg_info brings DDR up */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10140000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,
    .crg_pll_lock_offset = 0xe8,        /* 4 PLL lock bits */
    .crg_pll_lock_bits  = 0xf,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 5, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 50000000,

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .gpio_base          = 0x20140000,
    .gpio_count         = 12,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,

    .dma_base           = 0x100D0000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 1,
    .himci_bases        = { 0x10020000 },
    .himci_irqs         = { 18 },

    .num_i2c            = 1,
    .i2c_bases          = { 0x200D0000 },
    .i2c_type           = "hisi-i2c-v1",

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 4,
    .crg_defaults       = {
        { 0x00, (1 << 24) | (1 << 27) },
        { 0x04, (1 << 12) | 46 },
        { 0x10, (1 << 24) | (1 << 27) },
        { 0x14, (3 << 12) | 25 },
    },

    .num_regbanks       = 7,     /* DDRC: hisi-ddrc-v1 */
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-vedu",   0x20620000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3518CV100 (V1): die-identical to Hi3516CV100; ipctool identifies
 * it via sysctl_base + 0x8C bits[14:8] == 0x10.
 */
static const HisiSoCConfig hi3518cv100_soc = {
    .name               = "hi3518cv100",
    .desc               = "HiSilicon Hi3518CV100 (ARM926EJ-S, CV100 sibling)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV100,
    .chipid_byte_layout = true,
    .v1_chip_id_8c      = 0x10 << 8,        /* bits[14:8] = 0x10 → 3518CV100 */
    .default_sensor     = "imx122",
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,32M",

    .ram_base           = 0x80000000,
    .v1_xip_boot        = true,         /* reg_info brings DDR up */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10140000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,
    .crg_pll_lock_offset = 0xe8,        /* 4 PLL lock bits */
    .crg_pll_lock_bits  = 0xf,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 5, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 50000000,

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .gpio_base          = 0x20140000,
    .gpio_count         = 12,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,

    .dma_base           = 0x100D0000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 1,
    .himci_bases        = { 0x10020000 },
    .himci_irqs         = { 18 },

    .num_i2c            = 1,
    .i2c_bases          = { 0x200D0000 },
    .i2c_type           = "hisi-i2c-v1",

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 4,
    .crg_defaults       = {
        { 0x00, (1 << 24) | (1 << 27) },
        { 0x04, (1 << 12) | 46 },
        { 0x10, (1 << 24) | (1 << 27) },
        { 0x14, (3 << 12) | 25 },
    },

    .num_regbanks       = 7,     /* DDRC: hisi-ddrc-v1 */
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-vedu",   0x20620000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3518EV100 (V1): die-identical to Hi3516CV100; ipctool identifies
 * it via either sysctl_base + 0x8C bits[14:8] == 0x57 OR
 * sysctl_base + 0x88 == 2.  We populate both for robustness.
 */
static const HisiSoCConfig hi3518ev100_soc = {
    .name               = "hi3518ev100",
    .desc               = "HiSilicon Hi3518EV100 (ARM926EJ-S, CV100 sibling)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV100,
    .chipid_byte_layout = true,
    .v1_chip_id_88      = 2,                /* 18EV100 marker */
    .v1_chip_id_8c      = 0x57 << 8,        /* bits[14:8] = 0x57 → 3518EV100 */
    .default_sensor     = "imx122",
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,32M",

    .ram_base           = 0x80000000,
    .v1_xip_boot        = true,         /* reg_info brings DDR up */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10140000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,
    .crg_pll_lock_offset = 0xe8,        /* 4 PLL lock bits */
    .crg_pll_lock_bits  = 0xf,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 5, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 50000000,

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .gpio_base          = 0x20140000,
    .gpio_count         = 12,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,

    .dma_base           = 0x100D0000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 1,
    .himci_bases        = { 0x10020000 },
    .himci_irqs         = { 18 },

    .num_i2c            = 1,
    .i2c_bases          = { 0x200D0000 },
    .i2c_type           = "hisi-i2c-v1",

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 4,
    .crg_defaults       = {
        { 0x00, (1 << 24) | (1 << 27) },
        { 0x04, (1 << 12) | 46 },
        { 0x10, (1 << 24) | (1 << 27) },
        { 0x14, (3 << 12) | 25 },
    },

    .num_regbanks       = 7,     /* DDRC: hisi-ddrc-v1 */
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-vedu",   0x20620000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3516CV200 (V2): pre-2015, 1M economy.  ARM926EJ-S @550MHz.
 * Video: H.264, 960P@30fps.  Memory: 32/64MB DDR2 integrated.
 * Platform family: hi3518ev200 (same die, =hi3516cv200).
 *
 * 0x20xxxxxx peripheral space.  Uses hieth-sf in vendor kernel but FEMAC
 * in OpenIPC's 4.9+ kernel.  FMC memory window at 0x58000000 (not 0x14000000).
 */
static const HisiSoCConfig hi3516cv200_soc = {
    .name               = "hi3516cv200",
    .desc               = "HiSilicon Hi3516CV200 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV200,
    .chipid_byte_layout = true,            /* V2: byte-wise SCSYSID0..3 */
    .chip_variant       = 1,               /* 1 = 3516CV200 (vs 2=18EV200, 3=18EV201) */
    /* V2 RNG_GEN block — different layout from V3+ HISEC_TRNG_CTRL,
     * data register at +0x004 instead of +0x204. */
    .hwrng_base         = 0x20280000,
    .hwrng_data_offset  = 0x004,
    /* 64 MiB on-chip DDR2 (512Mb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims the upper 32 MiB at 0x82000000. */
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,32M",
    /* CV200 kernel uses the DesignWare-derived i2c-hisilicon driver
     * (CON/TAR/DATA_CMD/AUTO/TX_RX layout) rather than HiBVT. */
    .i2c_type           = "hisi-i2c-dw",

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x100D0000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 30, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,      /* 3 MHz (APB bus / 4) */

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,

    .gpio_base          = 0x20140000,
    .gpio_count         = 9,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x10060000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 2,
    .himci_bases        = { 0x10020000, 0x10030000 },
    .himci_irqs         = { 18, 8 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x200D0000, 0x20240000, 0x20250000 },

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    /*
     * CRG register defaults — mimic what U-Boot sets before booting Linux.
     * The 4.9 kernel uses DT-based clocks; gate bits must be pre-enabled
     * or the PL011 UART driver can't get its clock and the console is dead.
     */
    .num_crg_defaults   = 4,
    .crg_defaults       = {
        /* UART0/1/2 + SPI0/1 clk enable, UART mux=24MHz */
        { 0xe4, (1 << 13) | (1 << 14) | (1 << 15) |
                (1 << 16) | (1 << 17) | (1 << 19) },
        { 0xc0, (1 << 1) },            /* FMC clk enable */
        { 0xec, (1 << 1) },            /* ETH clk enable */
        { 0xc4, (1 << 1) | (1 << 9) }, /* MMC0 + MMC1 clk enable */
    },

    /*
     * Register bank stubs — vendor .ko modules access video/peripheral
     * hardware during init.  Without mapped regions, reads return 0
     * from QEMU's "unimplemented" handler, causing poll loops to hang.
     */
    .num_regbanks       = 7,
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-ddr",    0x20110000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3516AV100 (V2A): pre-2015, 5M professional.  Cortex-A7 @600MHz.
 * Video: H.265, WDR, 5M@25fps.  First HiSilicon IPC SoC with H.265.
 * Platform family: hi3516dv100 (4M@25fps, same SDK).
 *
 * Cortex-A7 + GICv2 but with V1/V2-era 0x20xxxxxx peripheral addresses.
 * Uses HISFC350 flash controller (like CV100) and GMAC (not FEMAC).
 * No ARM arch timer — SP804 at 50 MHz is primary clocksource.
 */
static const HisiSoCConfig hi3516av100_soc = {
    .name               = "hi3516av100",
    .desc               = "HiSilicon Hi3516AV100 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_AV100,
    .chipid_byte_layout = true,            /* V2A: byte-wise SCSYSID0..3 */
    /* AV100 has no on-chip DDR (external DDR3/3L up to 512 MiB).
     * Typical OpenIPC AV100 cameras ship 128 MiB.  Kernel gets 32 MiB,
     * vendor mmz.ko claims the upper 96 MiB at 0x82000000. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,96M",

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x20301000,       /* V1-era address space! */
    .gic_cpu_base       = 0x20302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 4,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000, 0x20230000 },
    .uart_irqs          = { 8, 9, 10, 11 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,          /* 3 MHz — matches DT clocksource;
                                             * clockevent 16.7x slow but avoids
                                             * timer storm from clocksource speedup */

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 12, 13 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",    /* HISFC350, same as CV100 */

    .gpio_base          = 0x20140000,
    .gpio_count         = 15,               /* ports 0-14 contiguous */
    .gpio_stride        = 0x10000,
    .gpio_irq           = 47,               /* shared IRQ for all ports (GIC) */
    .gpio_extras        = {
        { 0x20260000, 49 },                 /* port 15: non-contiguous, IRQ 49 */
    },

    .gmac_base          = 0x10090000,
    .gmac_irq           = 25,

    .num_himci          = 2,
    .himci_bases        = { 0x206E0000, 0x206F0000 },
    .himci_irqs         = { 19, 20 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x200D0000, 0x20240000, 0x20250000 },
    /* AV100 kernel uses the same DesignWare-derived i2c-hisilicon
     * driver as CV200 (vendor #ifdef CONFIG_ARCH_HI3516A). */
    .i2c_type           = "hisi-i2c-dw",

    .mipi_rx_base       = 0x20680000,
    .mipi_rx_irq        = 34,

    .rtc_base           = 0x20060000,
    .rtc_irq            = 7,

    .vedu_base          = 0x20640000,
    .jpge_base          = 0x20660000,
    .vedu_irq           = 43,
    .jpge_irq           = 41,

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_regbanks       = 8,
    .regbanks           = {
        { "hisi-misc",       0x20120000, 0x10000 },
        { "hisi-ddr",        0x20110000, 0x10000 },
        { "hisi-pwm",        0x20130000, 0x10000 },
        { "hisi-nandc",      0x10000000, 0x1000  },
        /* USB/SNAND left unmapped — regbanks store poll-bit writes
         * causing infinite loops in EHCI handshake and SPI NAND OP */
        { "hisi-regulator",  0x20270000, 0x1000  },
        { "hisi-viu",        0x20580000, 0x40000 },
        { "hisi-vpss",       0x20600000, 0x10000 },
        { "hisi-aiao",       0x20650000, 0x10000 },
    },
};

/*
 * Hi3516DV100 (V2A): die-identical to Hi3516AV100 — same Cortex-A7,
 * same V1-era 0x20xxxxxx address map, same HISFC350 flash, same
 * peripheral layout.  ipctool's hal_hisi.c distinguishes them only by
 * the chip-variant byte in SCSYSID0 (AV100 = 0/1, DV100 = 2).
 */
static const HisiSoCConfig hi3516dv100_soc = {
    .name               = "hi3516dv100",
    .desc               = "HiSilicon Hi3516DV100 (Cortex-A7, AV100 sibling)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_AV100,    /* shares AV100 family ID */
    .chipid_byte_layout = true,                 /* V2A: byte-wise SCSYSID0..3 */
    .chip_variant       = 2,                    /* SCSYSID0[31:24] = 2 → "3516DV100" */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,96M",

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x20301000,
    .gic_cpu_base       = 0x20302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 4,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000, 0x20230000 },
    .uart_irqs          = { 8, 9, 10, 11 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 12, 13 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .gpio_base          = 0x20140000,
    .gpio_count         = 15,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 47,
    .gpio_extras        = {
        { 0x20260000, 49 },
    },

    .gmac_base          = 0x10090000,
    .gmac_irq           = 25,

    .num_himci          = 2,
    .himci_bases        = { 0x206E0000, 0x206F0000 },
    .himci_irqs         = { 19, 20 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x200D0000, 0x20240000, 0x20250000 },
    .i2c_type           = "hisi-i2c-dw",

    .mipi_rx_base       = 0x20680000,
    .mipi_rx_irq        = 34,

    .rtc_base           = 0x20060000,
    .rtc_irq            = 7,

    .vedu_base          = 0x20640000,
    .jpge_base          = 0x20660000,
    .vedu_irq           = 43,
    .jpge_irq           = 41,

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_regbanks       = 8,
    .regbanks           = {
        { "hisi-misc",       0x20120000, 0x10000 },
        { "hisi-ddr",        0x20110000, 0x10000 },
        { "hisi-pwm",        0x20130000, 0x10000 },
        { "hisi-nandc",      0x10000000, 0x1000  },
        { "hisi-regulator",  0x20270000, 0x1000  },
        { "hisi-viu",        0x20580000, 0x40000 },
        { "hisi-vpss",       0x20600000, 0x10000 },
        { "hisi-aiao",       0x20650000, 0x10000 },
    },
};

/*
 * Hi3516CV300 (V3): ~2017, 2M mainstream.  ARM926EJ-S @800MHz.
 * Video: H.265, WDR, 1080P@30fps, 2K fisheye VI.  First ARM9 with H.265.
 * Platform family: hi3516ev100 (1M, H.265 1080P@20fps, 64MB DRAM, LiteOS).
 *
 * New 0x12xxxxxx address map (breaks from V1/V2).
 * VIC at 0x10040000.  Timer at 24 MHz.
 */
static const HisiSoCConfig hi3516cv300_soc = {
    .name               = "hi3516cv300",
    .desc               = "HiSilicon Hi3516CV300 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV300,
    .chipid_byte_layout = true,            /* V3: byte-wise SCSYSID0..3 */
    .default_sensor     = "imx291",        /* Sony 1080p Starvis on V3 ref boards */
    .hwrng_base         = 0x120C0000,      /* HISEC_TRNG_CTRL (V3) */
    .hwrng_data_offset  = 0x204,
    /* CV300 has no on-chip DDR (external DDR3/3L up to 512 MiB).
     * Stock CV300 cameras ship 128 MiB; kernel gets 32 MiB and the
     * vendor mmz.ko claims the upper 96 MiB at 0x82000000.
     *
     * Do NOT inject mmz= here.  Real V3 u-boot does not set it, and
     * OpenIPC's load_hisilicon treats the presence of mmz= as the
     * "use CMA allocator" flag — but the cv300_lite kernel shipped
     * in openipc.hi3516cv300-nor-lite.tgz is built without
     * CONFIG_CMA, so the CMA branch silently fails and
     * S99lsmodprobe never runs.  See issue #101. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10040000,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12100000, 0x12101000, 0x12102000 },
    .uart_irqs          = { 5, 30, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,      /* kernel: "sched_clock: 32 bits at
                                         * 3000kHz", -kernel and U-Boot alike */

    .num_spis           = 2,
    .spi_bases          = { 0x12120000, 0x12121000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x12140000,
    .gpio_count         = 9,
    .gpio_stride        = 0x1000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x10030000,
    .dma_irq            = 14,

    .femac_base         = 0x10050000,
    .femac_irq          = 12,

    .num_i2c            = 2,
    .i2c_bases          = { 0x12110000, 0x12112000 },

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
    .himci_irqs         = { 18, 27, 27 },

    .wdt_base           = 0x12080000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,
};

/*
 * Timer clocks of the Hi3516EV200/EV300 SDK family (EV200, EV300, 18EV300,
 * DV200 and their Goke rebrands) and of the CV500/AV300/DV300 SDK.  Their
 * DTS clock the SP804s (arm,sp804 / hisilicon,hisp804) from the fixed 3 MHz
 * clock (clk_3m, or CRG HI35xx_FIXED_3M) and declare the arm,armv7-timer at
 * clock-frequency = 50 MHz.  Left at QEMU's defaults (SP804 1 MHz, generic
 * timer 62.5 MHz) the guest clock ran at 1/3 speed where hisp804 is the
 * clocksource (CV500/AV300/DV300) and 1.25x where arch_sys_counter is
 * (EV200/EV300).
 */
#define HISI_V4_TIMER_CLOCKS                                \
    .timer_freq         = 3000000,                          \
    .arch_timer_freq    = 50000000

/*
 * Hi3516CV500 (V3.5): 2018, 3M smart-vision.  Dual Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps / 1080P@30fps.  NPU: 0.5 TOPS.
 * Memory: ext DDR3(L)/DDR4 up to 8Gbit.  280-pin TFBGA 12x12mm.
 * Platform family: hi3516av300 (4K, 1.0 TOPS), hi3516dv300 (5M, 1.0 TOPS).
 *
 * Cortex-A7 + GICv2 (like V4), but unique peripheral address map distinct
 * from both V3 and V4.  RAM at 0x80000000 (like V1-V3), himciv200 MMC, 40 KB SRAM.
 */
static const HisiSoCConfig hi3516cv500_soc = {
    .name               = "hi3516cv500",
    .desc               = "HiSilicon Hi3516CV500 (Cortex-A7, dual-core)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV500,
    /* HISEC_TRNG_CTRL block at 0x10090000 (RSA at 0x10080000 displaces
     * TRNG up by 0x10000 vs V4); data register at +0x204. */
    .hwrng_base         = 0x10090000,
    .hwrng_data_offset  = 0x204,
    /* CV500 has no on-chip DDR (external DDR3(L)/DDR4 up to 1 GiB).
     * Stock CV500 cameras ship 128 MiB.  Kernel gets 32 MiB, vendor
     * mmz.ko claims the upper 96 MiB at 0x82000000. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,96M",
    .max_cpus           = 2,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 40 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x120A0000, 0x120A1000, 0x120A2000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 1, 2 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 3,
    .spi_bases          = { 0x120C0000, 0x120C1000, 0x120C2000 },
    .spi_irqs           = { 68, 69, 70 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120D0000,
    .gpio_count         = 11,           /* ports 0-10, sequential IRQs 16..26 */
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,           /* per-port: SPI 16..26 (GIC) */
    .gpio_extras        = {
        { 0x120DB000, 80 },             /* port 11: IRQ 80 (non-sequential) */
    },

    .femac_base         = 0x10010000,
    .femac_irq          = 32,

    .num_himci          = 3,
    .himci_bases        = { 0x10100000, 0x100F0000, 0x10020000 },
    .himci_irqs         = { 64, 30, 31 },

    .num_i2c            = 7,
    .i2c_bases          = { 0x120B0000, 0x120B1000, 0x120B2000, 0x120B3000,
                            0x120B5000, 0x120B6000, 0x120B7000 },

    .mipi_rx_base       = 0x113A0000,
    .mipi_rx_irq        = 57,

    .rtc_base           = 0x12080000,
    .rtc_irq            = 5,

    .vedu_base          = 0x11500000,
    .jpge_base          = 0x11220000,
    .vedu_irq           = 40,
    .jpge_irq           = 36,

    .wdt_base           = 0x12051000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 3,
    .crg_defaults       = {
        { 0x1B8, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 18)
               | (1 << 11) | (1 << 12) | (1 << 13)
               | (1 << 14) | (1 << 15) | (1 << 16)
               | (1 << 17) | (1 << 18) },
                                        /* UART0/1/2 clk enable + I2C0-7 clk enable
                                         * + UART0 mux 24MHz */
        { 0x144, 0x02 },               /* FMC clock enable */
        { 0x16C, 0x02 },               /* ETH clock enable */
    },

    .gzip_base          = 0x11200000,

    .cpu_srst_offset    = 0x78,         /* REG_CPU_SRST_CRG for SMP bringup */

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x8000  },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12040000, 0x10000 },
        { "hisi-iocfg2",     0x10FF0000, 0x10000 },
        { "hisi-pwm",        0x12070000, 0x10000 },
        { "hisi-usb3",       0x100E0000, 0x10000 },
        { "hisi-vi-cap",     0x11300000, 0x100000 },
        { "hisi-vi-proc",    0x11000000, 0x40000 },
        { "hisi-vpss",       0x11040000, 0x10000 },
        { "hisi-aiao",       0x113B0000, 0x20000 },
        { "hisi-ive",        0x11230000, 0x10000 },   /* CV500 IVE: DT @0x11230000, SPI 37 */
        { "hisi-nnie",       0x11100000, 0x10000 },   /* CV500 NNIE: DT @0x11100000, SPI 45 */
    },
};

/*
 * Hi3516AV300 (V4A): 2019, 4K smart-vision.  Dual Cortex-A7 @900MHz.
 * Video: H.265/H.264, 4K(3840x2160)@30fps.  NPU: 1.0 TOPS.
 * Same die family as CV500 (identical peripheral map, MMC, SRAM, IRQs);
 * differs only in feature tier (4K + bigger NPU) and the SCSYSID family
 * ID 0x3516A300 (vs CV500's 0x3516C500).
 */
static const HisiSoCConfig hi3516av300_soc = {
    .name               = "hi3516av300",
    .desc               = "HiSilicon Hi3516AV300 (Cortex-A7, dual-core, 4K)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_AV300,
    .default_sensor     = "imx415",        /* Sony 4K Starvis on AV300 ref boards */
    /* Inherits CV500 family TRNG layout (no separate AV300 datasheet). */
    .hwrng_base         = 0x10090000,
    .hwrng_data_offset  = 0x204,
    /* Same memory layout as CV500: external DDR, 128 MiB typical. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,96M",
    .max_cpus           = 2,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 40 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x120A0000, 0x120A1000, 0x120A2000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 1, 2 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 3,
    .spi_bases          = { 0x120C0000, 0x120C1000, 0x120C2000 },
    .spi_irqs           = { 68, 69, 70 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120D0000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,
    .gpio_extras        = {
        { 0x120DB000, 80 },
    },

    .femac_base         = 0x10010000,
    .femac_irq          = 32,

    .num_himci          = 3,
    .himci_bases        = { 0x10100000, 0x100F0000, 0x10020000 },
    .himci_irqs         = { 64, 30, 31 },

    .num_i2c            = 7,
    .i2c_bases          = { 0x120B0000, 0x120B1000, 0x120B2000, 0x120B3000,
                            0x120B5000, 0x120B6000, 0x120B7000 },

    .mipi_rx_base       = 0x113A0000,
    .mipi_rx_irq        = 57,

    .rtc_base           = 0x12080000,
    .rtc_irq            = 5,

    .vedu_base          = 0x11500000,
    .jpge_base          = 0x11220000,
    .vedu_irq           = 40,
    .jpge_irq           = 36,

    .wdt_base           = 0x12051000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 3,
    .crg_defaults       = {
        { 0x1B8, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 18)
               | (1 << 11) | (1 << 12) | (1 << 13)
               | (1 << 14) | (1 << 15) | (1 << 16)
               | (1 << 17) | (1 << 18) },
        { 0x144, 0x02 },
        { 0x16C, 0x02 },
    },

    .gzip_base          = 0x11200000,

    .cpu_srst_offset    = 0x78,

    .num_regbanks       = 18,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x8000  },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12040000, 0x10000 },
        { "hisi-iocfg2",     0x10FF0000, 0x10000 },
        { "hisi-pwm",        0x12070000, 0x10000 },
        { "hisi-usb3",       0x100E0000, 0x10000 },
        { "hisi-vi-cap",     0x11300000, 0x100000 },
        { "hisi-vi-proc",    0x11000000, 0x40000 },
        { "hisi-vpss",       0x11040000, 0x10000 },
        { "hisi-aiao",       0x113B0000, 0x20000 },
        { "hisi-npu",        0x11700000, 0x100000 },  /* 1.0 TOPS NPU */
        { "hisi-ive",        0x11230000, 0x10000 },   /* AV300 IVE: DT @0x11230000, SPI 37 */
        { "hisi-nnie",       0x11100000, 0x10000 },   /* AV300 NNIE: DT @0x11100000, SPI 45 */
        /* Mask-ROM security peripherals — reads return 0, writes are
         * dropped.  Sufficient to keep the bootrom from faulting when
         * it touches RSA0/SPACC/KLAD/OTP during signature verification;
         * functional verification of the crypto path requires real
         * device models, not stubs. */
        { "hisi-klad",       0x10070000, 0x10000 },  /* key ladder */
        { "hisi-rsa0",       0x10080000, 0x10000 },  /* RSA0 engine */
        { "hisi-otp-mirror", 0x100A0000, 0x10000 },  /* OTP slot mirror */
        { "hisi-otpuser",    0x100B0000, 0x10000 },  /* OTP user controller */
        { "hisi-spacc",      0x100C0000, 0x10000 },  /* Synopsys SHA/AES */
    },
};

/*
 * Hi3516DV300 (V4A): die-identical to Hi3516AV300 / Hi3516CV500 — same
 * dual Cortex-A7 @900MHz, same 0x12xxxxxx peripheral map, same MMC,
 * same SRAM, same DDR layout.  Per ipctool's hal_hisi.c the chip is
 * distinguished only by the SCSYSID0 family ID 0x3516D300 (vs CV500's
 * 0x3516C500 and AV300's 0x3516A300); shipped in the Hi3516CV500 SDK
 * as a sibling target (`hi3516dv300_spi_smp_image_glibc`).
 */
static const HisiSoCConfig hi3516dv300_soc = {
    .name               = "hi3516dv300",
    .desc               = "HiSilicon Hi3516DV300 (Cortex-A7, dual-core, CV500 sibling)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_DV300,
    .hwrng_base         = 0x10090000,
    .hwrng_data_offset  = 0x204,
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,96M",
    .max_cpus           = 2,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 40 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x120A0000, 0x120A1000, 0x120A2000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 1, 2 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 3,
    .spi_bases          = { 0x120C0000, 0x120C1000, 0x120C2000 },
    .spi_irqs           = { 68, 69, 70 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120D0000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,
    .gpio_extras        = {
        { 0x120DB000, 80 },
    },

    .femac_base         = 0x10010000,
    .femac_irq          = 32,

    .num_himci          = 3,
    .himci_bases        = { 0x10100000, 0x100F0000, 0x10020000 },
    .himci_irqs         = { 64, 30, 31 },

    .num_i2c            = 7,
    .i2c_bases          = { 0x120B0000, 0x120B1000, 0x120B2000, 0x120B3000,
                            0x120B5000, 0x120B6000, 0x120B7000 },

    .mipi_rx_base       = 0x113A0000,
    .mipi_rx_irq        = 57,

    .rtc_base           = 0x12080000,
    .rtc_irq            = 5,

    .vedu_base          = 0x11500000,
    .jpge_base          = 0x11220000,
    .vedu_irq           = 40,
    .jpge_irq           = 36,

    .wdt_base           = 0x12051000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 3,
    .crg_defaults       = {
        { 0x1B8, (1 << 0) | (1 << 1) | (1 << 2) | (1 << 18)
               | (1 << 11) | (1 << 12) | (1 << 13)
               | (1 << 14) | (1 << 15) | (1 << 16)
               | (1 << 17) | (1 << 18) },
        { 0x144, 0x02 },
        { 0x16C, 0x02 },
    },

    .gzip_base          = 0x11200000,

    .cpu_srst_offset    = 0x78,

    .num_regbanks       = 11,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x8000  },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12040000, 0x10000 },
        { "hisi-iocfg2",     0x10FF0000, 0x10000 },
        { "hisi-pwm",        0x12070000, 0x10000 },
        { "hisi-usb3",       0x100E0000, 0x10000 },
        { "hisi-vi-cap",     0x11300000, 0x100000 },
        { "hisi-vi-proc",    0x11000000, 0x40000 },
        { "hisi-vpss",       0x11040000, 0x10000 },
        { "hisi-aiao",       0x113B0000, 0x20000 },
        { "hisi-npu",        0x11700000, 0x100000 },
    },
};

/*
 * Hi3519V101 (V3A): ~2017, 4K professional.
 * CPU: Cortex-A17 @1.25GHz + Cortex-A7 @800MHz (big.LITTLE).
 * Video: H.265, 4-frame WDR, 4K@30fps + 2M@30fps simultaneous.
 * Sensor: dual input, max 16M pixels combined.
 * Platform family: hi3516av200 (5M@30fps+720P@30fps, same big.LITTLE).
 *
 * V3A generation — big.LITTLE + GICv2, V3-era peripheral addresses
 * (0x121xxxxx UARTs/SPI/GPIO like CV300).
 * Uses GMAC (HiGMAC V200, hi_gmac_v200 driver) for Ethernet.
 */
static const HisiSoCConfig hi3519v101_soc = {
    .name               = "hi3519v101",
    .desc               = "HiSilicon Hi3519V101 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_19V101,
    .chipid_byte_layout = true,            /* V3A: byte-wise SCSYSID0..3 */
    .hwrng_base         = 0x120C0000,      /* HISEC_TRNG_CTRL (V3A) */
    .hwrng_data_offset  = 0x204,
    /* 3519V101 has no on-chip DDR (external DDR4/3/3L up to 2 GiB dual).
     * 4K/WDR boards typically ship 256 MiB.  Kernel gets 32 MiB, vendor
     * mmz.ko claims the upper 224 MiB at 0x82000000. */
    .ram_size_default   = 256 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,224M",

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 5,
    .uart_bases         = { 0x12100000, 0x12101000, 0x12102000,
                            0x12103000, 0x12104000 },
    .uart_irqs          = { 4, 5, 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 64, 66 },
    .timer_freq         = 3000000,          /* 3 MHz */
    .arch_timer_freq    = 24000000,         /* DTS armv7-timer clock-frequency */

    .num_spis           = 4,
    .spi_bases          = { 0x12120000, 0x12121000, 0x12122000, 0x12123000 },
    .spi_irqs           = { 9, 10, 11, 12 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x12140000,
    .gpio_count         = 17,               /* ports 0-14, phantom 15, port 16 */
    .gpio_stride        = 0x1000,
    .gpio_irq           = 43,               /* shared IRQ for all ports (GIC) */

    /* Gigabit Ethernet — HiGMAC V200, DT compatible "hisilicon,higmac-v3".
     * Vendor DTB: reg = <0x10050000 0x1000 0x120100ec 0x04>, the 2nd cell
     * is the macif phy-mode select inside the CRG, which our hisi-regbank
     * already covers.  IRQ confirmed from the embedded board.dtb. */
    .gmac_base          = 0x10050000,
    .gmac_irq           = 25,

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
    .himci_irqs         = { 23, 24, 13 },

    .num_i2c            = 4,
    .i2c_bases          = { 0x12110000, 0x12111000, 0x12112000, 0x12113000 },

    .mipi_rx_base       = 0x11300000,
    .mipi_rx_irq        = 28,

    .rtc_base           = 0x12090000,
    .rtc_irq            = 1,

    .vedu_base          = 0x11280000,
    .jpge_base          = 0x11200000,
    .vedu_irq           = 37,
    .jpge_irq           = 38,

    .wdt_base           = 0x12080000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 1,
    .crg_defaults       = {
        /* APLL ctrl_reg2: fbdiv=792, refdiv=24 → 792 MHz (prevents div-by-zero) */
        { 0x04, (24 << 12) | 792 },
    },

    .num_regbanks       = 10,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x10000 },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12160000, 0x10000 },
        { "hisi-pwm",        0x12130000, 0x10000 },
        { "hisi-usb-ehci",   0x10120000, 0x10000 },
        { "hisi-usb-ohci",   0x10110000, 0x10000 },
        { "hisi-vi-cap",     0x11380000, 0x100000 },
        { "hisi-vou",        0x11000000, 0x20000 },
        { "hisi-vpss",       0x11180000, 0x10000 },
        { "hisi-aiao",       0x11080000, 0x10000 },
    },
};

/*
 * Hi3516AV200 (V3A): same die as 3519V101, distinguished by SCSYSID0
 * sub-variant byte (5/6/0x15/0x16 per ipctool's get_chip_V3A).  Targets
 * 5M@30fps + 720P@30fps cameras with the same big.LITTLE A17+A7 layout.
 *
 * Per the joint hi3519v101/hi3516av200 hardware guide ("未有特殊说明，
 * Hi3516AV200 与Hi3519V101 完全一致"), every peripheral address, IRQ,
 * GPIO count, and clock matches 3519V101.  Inherit everything via the
 * 3519V101 layout; just swap SoC ID variant + default sensor.
 */
static const HisiSoCConfig hi3516av200_soc = {
    .name               = "hi3516av200",
    .desc               = "HiSilicon Hi3516AV200 (Cortex-A7, big.LITTLE die)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_19V101,    /* shares family with 3519V101 */
    .chipid_byte_layout = true,
    .chip_variant       = 5,                     /* SCSYSID0 byte 5 = 3516AV200 */
    .default_sensor     = "imx385",              /* Sony 1080p Starvis on AV200 ref boards */
    .hwrng_base         = 0x120C0000,            /* same as 3519V101 family */
    .hwrng_data_offset  = 0x204,
    .ram_size_default   = 256 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x82000000,224M",

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 5,
    .uart_bases         = { 0x12100000, 0x12101000, 0x12102000,
                            0x12103000, 0x12104000 },
    .uart_irqs          = { 4, 5, 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 64, 66 },
    .timer_freq         = 3000000,
    .arch_timer_freq    = 24000000,         /* same SDK DTS as Hi3519V101 */

    .num_spis           = 4,
    .spi_bases          = { 0x12120000, 0x12121000, 0x12122000, 0x12123000 },
    .spi_irqs           = { 9, 10, 11, 12 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x12140000,
    .gpio_count         = 17,
    .gpio_stride        = 0x1000,
    .gpio_irq           = 43,

    /* GMAC: inherits the 3519V101 wiring (same die per vendor guide). */
    .gmac_base          = 0x10050000,
    .gmac_irq           = 25,

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
    .himci_irqs         = { 23, 24, 13 },

    .num_i2c            = 4,
    .i2c_bases          = { 0x12110000, 0x12111000, 0x12112000, 0x12113000 },

    .mipi_rx_base       = 0x11300000,
    .mipi_rx_irq        = 28,

    .rtc_base           = 0x12090000,
    .rtc_irq            = 1,

    .vedu_base          = 0x11280000,
    .jpge_base          = 0x11200000,
    .vedu_irq           = 37,
    .jpge_irq           = 38,

    .wdt_base           = 0x12080000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 1,
    .crg_defaults       = {
        { 0x04, (24 << 12) | 792 },
    },

    .num_regbanks       = 10,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x10000 },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12160000, 0x10000 },
        { "hisi-pwm",        0x12130000, 0x10000 },
        { "hisi-usb-ehci",   0x10120000, 0x10000 },
        { "hisi-usb-ohci",   0x10110000, 0x10000 },
        { "hisi-vi-cap",     0x11380000, 0x100000 },
        { "hisi-vou",        0x11000000, 0x20000 },
        { "hisi-vpss",       0x11180000, 0x10000 },
        { "hisi-aiao",       0x11080000, 0x10000 },
    },
};

/*
 * Hi3516EV300 (V4): 2019, 5M professional.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 5M(2592x1944)@15fps / 4M(2688x1520)@25fps.
 * Memory: 1Gbit DDR3L integrated (128MB).  279-pin TFBGA.
 * V4 generation — new RAM base at 0x40000000.  SDHCI replaces himciv200.
 * 10 GPIO groups.  Per-port GIC IRQs (SPI 16+).
 */
static const HisiSoCConfig hi3516ev300_soc = {
    .name               = "hi3516ev300",
    .desc               = "HiSilicon Hi3516EV300 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_EV300,
    .default_sensor     = "imx335",        /* Sony 5MP on EV300 ref boards */
    .hwrng_base         = 0x10080000,      /* HISEC_TRNG_CTRL (V4) */
    .hwrng_data_offset  = 0x204,
    /* MVP video pipeline IRQ heartbeat — pulses VI_CAP0/VI_PROC0/VPSS
     * at 25 Hz so the vendor MPP modules' IRQ handlers run and Majestic
     * gets past its "Timeout from venc channel 0" loop.
     * SPI numbers are GIC absolute IRQ - 32 (per /proc/interrupts the
     * kernel shows GIC-75/76/78 for VI_CAP0/VI_PROC0/VPSS). */
    .vi_fp_base         = 0x11FE0000,      /* unused gap above vedu/jpge */
    .vi_fp_cap_irq      = 43,              /* VI_CAP0  = SPI 43 (GIC 75) */
    .vi_fp_proc_irq     = 44,              /* VI_PROC0 = SPI 44 (GIC 76) */
    .vi_fp_vpss_irq     = 46,              /* VPSS     = SPI 46 (GIC 78) */
    .vi_fp_vedu_irq     = 47,              /* VEDU     = SPI 47 (GIC 79) */
    /* 128 MiB on-chip DDR3L (1Gb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims 96 MiB at 0x42000000 — matches the canonical EV300 layout
     * documented in OpenIPC's /usr/bin/load_hisilicon. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,96M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 10,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,           /* per-port: SPI 16..25 (GIC) */

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,

    .gzip_base          = 0x11310000,

    /*
     * CRG register defaults — mimic what U-Boot sets before booting Linux.
     * Register map from drivers/clk/hisilicon/clk-hi3516ev300.c.
     */
    .num_crg_defaults   = 4,
    .crg_defaults       = {
        { 0x144, (1 << 1) },            /* FMC clock enable */
        { 0x16c, (1 << 1) },            /* ETH clock enable (resets deasserted=0) */
        { 0x1b8, (1 << 0) | (1 << 1) | (1 << 2)
               | (1 << 11) | (1 << 12) | (1 << 13) },
                                         /* UART0/1/2 + I2C0/1/2 clock enable */
        { 0x1bc, (1 << 12) | (1 << 13) },/* SPI0/1 clock enable */
    },

    /* Match real EV300 silicon — vendor open_vi.ko / open_vpss.ko /
     * open_vedu.ko reach for these MMIO blocks; without backing
     * regbank RAM their writes vanish and reads return 0, which
     * the vendor IRQ handlers then mistake for a spurious IRQ and
     * panic ("VI_COMM_ProcIrqRoute line 1993").  Layout from the
     * V4 family map (also used by the Goke variants via the
     * HISI_V4_COMMON_PERIPH macro) plus EV300-specific IVE. */
    .num_regbanks       = 5,
    .regbanks           = {
        { "hisi-vi-cap",     0x11000000, 0x200000 },
        { "hisi-vi-proc",    0x11200000, 0x40000  },
        { "hisi-vgs",        0x11300000, 0x10000  },
        { "hisi-ive",        0x11320000, 0x10000  },
        { "hisi-vpss",       0x11400000, 0x10000  },
    },
};

/*
 * Hi3516EV200 (V4): 2019, 3M economy.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps.  Memory: 512Mb DDR2 (64MB).
 * Economy variant — 8 GPIO groups.  Same peripheral addresses as EV300.
 */
static const HisiSoCConfig hi3516ev200_soc = {
    .name               = "hi3516ev200",
    .desc               = "HiSilicon Hi3516EV200 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_EV200,
    .default_sensor     = "imx307",        /* Sony 1080p Starvis on EV200 ref boards */
    .hwrng_base         = 0x10080000,      /* HISEC_TRNG_CTRL (V4) */
    .hwrng_data_offset  = 0x204,
    /* 64 MiB on-chip DDR2 (512Mb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims the upper 32 MiB at 0x42000000 — matches the canonical
     * EV200 layout documented in OpenIPC's /usr/bin/load_hisilicon. */
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,32M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 8,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,

    /* HiSilicon HW gzip decompressor — same V4 IP / address as ev300.
     * U-Boot's u-boot-z.bin uses it during early boot; without it the
     * vendor lib/hw_dec/hw_decompress_hi3516ev200.c times out with
     * "Uncompress hardware decompress overtime!" before reaching the
     * U-Boot banner.  See openhisilicon#60. */
    .gzip_base          = 0x11310000,
};

/*
 * Hi3518EV300 (V4): 2019, 3M consumer/IoT.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps.  Memory: 512Mb DDR2 (64MB).
 * No integrated FE PHY — boards need external PHY or go Ethernet-less.
 * 8 GPIO groups.
 * FEMAC controller exists but no on-chip PHY; we still instantiate FEMAC
 * so the driver probe succeeds (it just won't link up without a real PHY).
 */
static const HisiSoCConfig hi3518ev300_soc = {
    .name               = "hi3518ev300",
    .desc               = "HiSilicon Hi3518EV300 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_18EV300,
    .hwrng_base         = 0x10080000,      /* HISEC_TRNG_CTRL (V4) */
    .hwrng_data_offset  = 0x204,
    /* 64 MiB on-chip DDR2 (512Mb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims the upper 32 MiB at 0x42000000 — matches the canonical
     * 18EV300 layout documented in OpenIPC's /usr/bin/load_hisilicon. */
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,32M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 8,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,

    /* HiSilicon HW gzip decompressor — same V4 IP / address as ev300.
     * U-Boot's u-boot-z.bin uses it during early boot; without it the
     * vendor lib/hw_dec/hw_decompress_hi3518ev300.c times out with
     * "Uncompress hardware decompress overtime!" before reaching the
     * U-Boot banner.  See openhisilicon#60. */
    .gzip_base          = 0x11310000,
};

/*
 * Hi3516DV200 (V4): 2019, 5M professional.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 5M(3072x1728)@20fps.  Memory: ext DDR3/DDR3L up to 4Gbit.
 * Professional variant — 10 GPIO groups.  Same peripheral layout as EV300.
 */
static const HisiSoCConfig hi3516dv200_soc = {
    .name               = "hi3516dv200",
    .desc               = "HiSilicon Hi3516DV200 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_DV200,
    .hwrng_base         = 0x10080000,      /* HISEC_TRNG_CTRL (V4) */
    .hwrng_data_offset  = 0x204,
    /* DV200 has no on-chip DDR (external DDR3/3L up to 512 MiB).
     * Stock OpenIPC DV200 cameras ship 128 MiB.  Kernel gets 32 MiB,
     * vendor mmz.ko claims the upper 96 MiB at 0x42000000. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,96M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },
    HISI_V4_TIMER_CLOCKS,

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 10,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,

    /* HiSilicon HW gzip decompressor — same V4 IP / address as ev300.
     * U-Boot's u-boot-z.bin uses it during early boot; without it the
     * vendor lib/hw_dec/hw_decompress_hi3516dv200.c times out with
     * "Uncompress hardware decompress overtime!" before reaching the
     * U-Boot banner.  See openhisilicon#60. */
    .gzip_base          = 0x11310000,
};

/*
 * Goke (国科微) variants — die-identical V4 HiSilicon silicon rebranded.
 * Hardware addresses, IRQs, GPIO counts all match the HiSilicon original.
 *
 * Goke V4 (2021, emulated here):
 *   GK7205V200 = Hi3516EV200  3M@25fps,  512Mb DDR2
 *   GK7205V210 (not emulated) 4M@20fps,  512Mb DDR
 *   GK7205V300 = Hi3516EV300  5M@25fps,  1Gb DDR
 *   GK7202V300 = Hi3518EV300  4M@20fps,  512Mb DDR
 *   GK7605V100 = Hi3516DV200  5M@25fps
 *
 * Goke next-gen (2022+, NOT emulated, separate Goke designs with NPU):
 *   GK7205V500  5M@25fps  0.5 TOPS  512Mb DDR   (2022)
 *   GK7205V510  5M@30fps  1 TOPS    1Gb DDR     (2022)
 *   GK7205V530  5M@30fps  1 TOPS    ext DDR     (2022)
 *   GK7202V330  5M@25fps  0.5 TOPS  512Mb DDR   (2022)
 *   GK7606V100  4K@60fps  2 TOPS                (2022)
 *   GK7609V100  8K@30fps  4 TOPS                (2022)
 *   GK7608V100  4K@60fps  4+4 TOPS              (2021)
 *   GK7605V200  4K@30fps  2 TOPS                (2023)
 *   GK7205V600  4K@20fps  1 TOPS                (2024)
 *   GK7606V200  4K@60fps  4 TOPS                (2024)
 *   GK7609V200  8K@30fps  8 TOPS    H.266       (2024)
 *   GK7608V200  4K@60fps  8 TOPS    H.266       (2025)
 */

/*
 * Stub register banks shared by every Goke V4 die.  Kept as a bare list so the
 * NPU-equipped next-gen parts can append to it: `.regbanks` is an array and
 * cannot be partially overridden after the common macro.
 */
#define HISI_V4_REGBANK_LIST                                \
        { "hisi-misc",       0x12028000, 0x8000  },         \
        { "hisi-ddr",        0x120d0000, 0x10000 },         \
        { "hisi-iocfg-vio",  0x112c0000, 0x10000 },         \
        { "hisi-iocfg-core", 0x120c0000, 0x10000 },         \
        { "hisi-iocfg-ahb",  0x100c0000, 0x10000 },         \
        { "hisi-pwm",        0x12080000, 0x10000 },         \
        { "hisi-usb3",       0x10030000, 0x10000 },         \
        { "hisi-aiao",       0x100e0000, 0x10000 },         \
        { "hisi-acodec",     0x100f0000, 0x10000 },         \
        { "hisi-vi-cap",     0x11000000, 0x200000 },        \
        { "hisi-vi-proc",    0x11200000, 0x40000 },         \
        { "hisi-vgs",        0x11300000, 0x10000 },         \
        { "hisi-ive",        0x11320000, 0x10000 },         \
        { "hisi-vpss",       0x11400000, 0x10000 }

/*
 * Goke next-gen V500 family (gk7205v500/v510/v530, gk7202v330): V4 banks plus
 * the NPU at 0x11340000.  Must follow HISI_V4_COMMON_PERIPH.  The EV200/EV300
 * class dies (gk7205v200/v300, gk7202v300, gk7201v200, gk7605v100) have no
 * NPU — they do detection on IVE — so they keep the plain V4 list.
 */
#define HISI_V4_NPU_REGBANKS                                \
    .num_regbanks       = 15,                               \
    .regbanks           = {                                 \
        HISI_V4_REGBANK_LIST,                               \
        { "hisi-npu",        0x11340000, 0x10000 },         \
    }

/* Common V4 peripheral block — shared by all V4 & Goke configs.
 *
 * Memory layout (ram_size_default / kernel_mem_mb / extra_cmdline) is
 * intentionally NOT included: each Goke variant ships a different DDR
 * size (64 MiB on EV200/18EV300 dies, 128 MiB on EV300/DV200 dies, etc.)
 * and must declare its own.  Other peripheral addresses are identical.
 */
#define HISI_V4_COMMON_PERIPH                               \
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),   \
    .ram_base           = 0x40000000,                       \
    .sram_base          = 0x04010000,                       \
    .sram_size          = 64 * KiB,                         \
    .use_gic            = true,                             \
    .gic_dist_base      = 0x10301000,                       \
    .gic_cpu_base       = 0x10302000,                       \
    .gic_num_spi        = 128,                              \
    .sysctl_base        = 0x12020000,                       \
    .crg_base           = 0x12010000,                       \
    .num_uarts          = 3,                                \
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 }, \
    .uart_irqs          = { 7, 8, 9 },                     \
    .num_timers         = 2,                                \
    .timer_bases        = { 0x12000000, 0x12001000 },       \
    .timer_irqs         = { 5, 6 },                        \
    .num_spis           = 2,                                \
    .spi_bases          = { 0x12070000, 0x12071000 },       \
    .spi_irqs           = { 14, 15 },                      \
    .fmc_ctrl_base      = 0x10000000,                       \
    .fmc_mem_base       = 0x14000000,                       \
    .gpio_base          = 0x120b0000,                       \
    .gpio_stride        = 0x1000,                           \
    .gpio_irq_start     = 16,                              \
    .femac_base         = 0x10040000,                       \
    .femac_irq          = 33,                              \
    .mipi_rx_base       = 0x11240000,                      \
    .mipi_rx_irq        = 45,                              \
    .rtc_base           = 0x120e0000,                      \
    .rtc_irq            = 0,                               \
    .num_sdhci          = 2,                                \
    .sdhci_bases        = { 0x10010000, 0x10020000 },       \
    .sdhci_irqs         = { 30, 31 },                       \
    .num_i2c            = 3,                                \
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 }, \
    .vedu_base          = 0x11410000,                       \
    .jpge_base          = 0x11420000,                       \
    .vedu_irq           = 47,                              \
    .jpge_irq           = 48,                              \
    .wdt_base           = 0x12030000,                       \
    .wdt_irq            = 2,                                \
    .wdt_freq           = 3000000,                          \
    /* Same V4 HW gzip decompressor as Hi3516EV200/EV300/  \
     * 18EV300/DV200 — needed by u-boot-z.bin's            \
     * hw_decompress_*.c during early boot.  Same IP &     \
     * register block on Goke rebrands (verified against   \
     * GKIPC SDK gk7205v[23]00/hw_decompress.c).  See      \
     * openhisilicon#60. */                                 \
    .gzip_base          = 0x11310000,                       \
    .hwrng_base         = 0x10080000,                       \
    .hwrng_data_offset  = 0x204,                            \
    .num_regbanks       = 14,                               \
    .regbanks           = { HISI_V4_REGBANK_LIST }

/* Per-die DDR layouts shared by V4 and Goke variants.
 * V4_DDR_64M:  on-chip 512Mb DDR2 (EV200-class dies)
 * V4_DDR_128M: on-chip 1Gb DDR3L or external 128 MiB (EV300/DV200-class)
 */
#define HISI_V4_DDR_64M                                     \
    .ram_size_default   = 64 * MiB,                         \
    .kernel_mem_mb      = 32,                               \
    .extra_cmdline      = "mmz_allocator=hisi "             \
                          "mmz=anonymous,0,0x42000000,32M"

#define HISI_V4_DDR_128M                                    \
    .ram_size_default   = 128 * MiB,                        \
    .kernel_mem_mb      = 32,                               \
    .extra_cmdline      = "mmz_allocator=hisi "             \
                          "mmz=anonymous,0,0x42000000,96M"

static const HisiSoCConfig gk7205v200_soc = {
    .name               = "gk7205v200",
    .desc               = "Goke GK7205V200 (Cortex-A7, ~Hi3516EV200)",
    .soc_id             = GOKE_SOC_ID_7205V200,
    .gpio_count         = 8,
    .default_sensor     = "imx307",     /* same die as EV200 */
    HISI_V4_DDR_64M,                /* EV200 die: 512Mb DDR2 */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_TIMER_CLOCKS,
};

/*
 * GK7201V200 — the low-end binning of the Hi3516EV200-class die that
 * GK7205V200 is built on: same 0x12xxxxxx V4 control block and the same
 * package (QFN 9x9 mm, 88 pins), with feature tiers cut.  Per the Goke
 * GK7205V200 datasheet v1.1 and the vendor GK7201V200 spec sheet:
 *
 *                     GK7205V200                 GK7201V200
 *   CPU               Cortex-A7 @ 900 MHz        Cortex-A7 @ 600 MHz
 *   Encoder max       2304x1296                  2304x1296 @ 15 fps
 *                                                (or 2x1080p + 2x360p @ 15)
 *   Sensor input      MIPI / LVDS / HiSPI        MIPI 2-lane (1.5 Gbps) + DVP
 *   Video output      LCD, BT656/BT1120          none
 *   Common            512 Mb embedded DDR2, FE PHY, USB 2.0, SDIO + SD,
 *                     audio codec, IVE; no NPU on either
 *
 * The cuts are rated limits and unbonded interfaces, not a different memory
 * map, so everything comes from the shared V4 macros; only name/desc/soc_id
 * differ from gk7205v200.  In practice the 600 MHz rating is soft: the stock
 * Xiongmai firmware on IPC_GK7201V200_G3H_S38 boards programs the A7 PLL to
 * 900 MHz (ipctool).  Those boards carry 8 MB NOR, the 64 MB DDR2, a MIS2008
 * sensor and kernel 4.9.37; chip id 0x72010200 was verified live via ipctool
 * and SCSYSID0.  The vendor XMedia kernel runs it on the EV200-class platform
 * (CONFIG_ARCH_XM72050200), while OpenIPC ships the userland MPP drivers from
 * its goke-osdrv-gk7205v500 (xm_*) package.  No default sensor: the MIS2008
 * has no i2c model yet.
 */
static const HisiSoCConfig gk7201v200_soc = {
    .name               = "gk7201v200",
    .desc               = "Goke GK7201V200 (Cortex-A7, ~Hi3516EV200 stripped)",
    .soc_id             = GOKE_SOC_ID_7201V200,
    .gpio_count         = 8,
    HISI_V4_DDR_64M,                /* 512Mb DDR2 */
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7205v300_soc = {
    .name               = "gk7205v300",
    .desc               = "Goke GK7205V300 (Cortex-A7, ~Hi3516EV300)",
    .soc_id             = GOKE_SOC_ID_7205V300,
    .gpio_count         = 10,
    .default_sensor     = "imx335",     /* same die as EV300 */
    HISI_V4_DDR_128M,               /* EV300 die: 1Gb DDR3L */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_TIMER_CLOCKS,
};

static const HisiSoCConfig gk7202v300_soc = {
    .name               = "gk7202v300",
    .desc               = "Goke GK7202V300 (Cortex-A7, ~Hi3518EV300)",
    .soc_id             = GOKE_SOC_ID_7202V300,
    .gpio_count         = 8,
    HISI_V4_DDR_64M,                /* 18EV300 die: 512Mb DDR2 */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_TIMER_CLOCKS,
};

static const HisiSoCConfig gk7605v100_soc = {
    .name               = "gk7605v100",
    .desc               = "Goke GK7605V100 (Cortex-A7, ~Hi3516DV200)",
    .soc_id             = GOKE_SOC_ID_7605V100,
    .gpio_count         = 10,
    HISI_V4_DDR_128M,               /* DV200 die: external 128 MiB typical */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_TIMER_CLOCKS,
};

/*
 * Goke next-gen (2022+) — Goke's own designs with NPU, V4-compatible address map.
 * CPU: Cortex-A7 @1GHz.  Video: H.265/H.264, 5M@25-30fps.
 * NPU: 0.5 TOPS (V500, V330) or 1.0 TOPS (V510, V530).
 * Same peripheral addresses as V4; NPU added at 0x11340000.
 * SDK: XMediaIPCLinuxV100R002C00SPC020, kernel 5.10.
 *
 * GK7205V500:  5M@25fps, 0.5 TOPS, 512Mb DDR2 MCP, FEPHY
 * GK7205V510:  5M@30fps, 1.0 TOPS, 1Gb DDR3 MCP, FEPHY
 * GK7205V530:  5M@30fps, 1.0 TOPS, ext DDR, FEPHY
 * GK7202V330:  5M@25fps, 0.5 TOPS, 512Mb DDR2 MCP, no FEPHY
 */

static const HisiSoCConfig gk7205v500_soc = {
    .name               = "gk7205v500",
    .desc               = "Goke GK7205V500 (Cortex-A7, 0.5 TOPS NPU)",
    .soc_id             = GOKE_SOC_ID_7205V500,
    .gpio_count         = 8,
    HISI_V4_DDR_64M,                /* 512Mb DDR2 MCP */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_NPU_REGBANKS,
    .xmsp804_timer      = true,     /* goke V500 kernel uses xmedia,sp804 */
};

static const HisiSoCConfig gk7205v510_soc = {
    .name               = "gk7205v510",
    .desc               = "Goke GK7205V510 (Cortex-A7, 1.0 TOPS NPU)",
    .soc_id             = GOKE_SOC_ID_7205V510,
    .gpio_count         = 8,
    HISI_V4_DDR_128M,               /* 1Gb DDR3 MCP */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_NPU_REGBANKS,
    .xmsp804_timer      = true,     /* goke V500 kernel uses xmedia,sp804 */
};

static const HisiSoCConfig gk7205v530_soc = {
    .name               = "gk7205v530",
    .desc               = "Goke GK7205V530 (Cortex-A7, 1.0 TOPS NPU, ext DDR)",
    .soc_id             = GOKE_SOC_ID_7205V530,
    .gpio_count         = 8,
    HISI_V4_DDR_128M,               /* external DDR, 128 MiB typical */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_NPU_REGBANKS,
    .xmsp804_timer      = true,     /* goke V500 kernel uses xmedia,sp804 */
};

static const HisiSoCConfig gk7202v330_soc = {
    .name               = "gk7202v330",
    .desc               = "Goke GK7202V330 (Cortex-A7, 0.5 TOPS NPU, no FEPHY)",
    .soc_id             = GOKE_SOC_ID_7202V330,
    .gpio_count         = 8,
    HISI_V4_DDR_64M,                /* 512Mb DDR2 MCP */
    HISI_V4_COMMON_PERIPH,
    HISI_V4_NPU_REGBANKS,
    .xmsp804_timer      = true,     /* same GK7205V500 register family / goke V500 kernel */
};

/*
 * Goke third generation (2025-2026) — Goke's own designs on the XMedia
 * "Lotus" SDK (ShiMetaPi shimetapi_pico_gx), split into two product lines
 * like HiSilicon's 3516/3519:
 *   GK7206 (codename "xmorca")  — A7 mainstream, embedded DDR, 5M/4K.
 *   GK7606 (codename "xmfalcon")— A55 high-end, external DDR, 4K.
 * Both keep the gk7205v5xx-style 0x12xxxxxx control block (UART 0x12040000,
 * sysctl 0x12020000, CRG 0x12010000, xmedia,sp804 timers 0x12000000, GPIO
 * 0x120b0000), RAM @0x40000000, an ATF BL1->BL31->BL33(u-boot-2020.01)
 * chain, and a RISC-V MCU coprocessor.  Chip ID at REG_SC_CHIPID 0x12020EE0.
 *
 * ---- GK7206V1 series (codename xmorca) ----
 * Goke's product page (gokemicro.com/znjk/info.aspx?itemid=444, 2026-02)
 * lists five variants, all dual Cortex-A7 MP2 + RISC-V MCU, 1.0 TOPS NPU
 * (INT16/8/4) + IVE, "圆鸮" AI ISP, AOV/fast boot, embedded DDR KGD:
 *
 *   variant  package  DDR                    encode  FE PHY  VI/VO extras
 *   V10      QFN92    512Mb DDR2  (64 MiB)   5M@30   yes     -
 *   V10B     QFN92    512Mb DDR2  (64 MiB)   5M@30   yes     DVP, RGB/BT.656
 *   V11      QFN92    1Gb DDR3L  (128 MiB)   4K@25   none    -
 *   V11A     QFN128   1Gb DDR3L  (128 MiB)   4K@25   yes     DVP, I80/BT.1120
 *   V12A     QFN128   2Gb DDR3L  (256 MiB)   4K@25   yes     DVP, I80/BT.1120
 *
 * The ShiMetaPi SDK ships board configs only for xm7206v11a and xm7206v12a
 * (configs/<chip>/reg/ DDR sheets: DDR3 128M and 256M).  The variants share
 * one die map, so it lives in GK7206_XMORCA_PERIPH below and each machine
 * adds only its name, chip ID and DDR layout.  Modelled: gk7206v10 and
 * gk7206v12a (the SDK target run-gk7206.sh builds its kernel for).
 *
 * DDR layout is the vendor one: Linux gets mem=32M and the rest is MMZ at
 * 0x42000000 under the *xmedia* allocator (not the HiSilicon "hisi" one the
 * V4 parts use) — 32M on 64 MiB parts, 96M on 128 MiB, 224M on 256 MiB.
 * That is what the SDK's GMP "load" script passes to xm_osal.ko, and what
 * the *_bootargs_buildin.txt profiles (media drivers built in) put on the
 * kernel command line.
 *
 * Map from the SDK u-boot arch/arm/include/asm/arch-xmorca/platform.h and
 * the vendor xmorca.dts.  The gk7205v5xx V4-common block is exact for the
 * control plane (UART0/sysctl/CRG/timers/GPIO bases); the I/O side differs
 * in many places — FE MAC 0x10040000 -> 0x100d0000, UART/I2C/SPI counts and
 * IRQs, SDHCI, FMC, VEDU, IVE — all re-specified in the macro below.  Uses
 * the xmedia,sp804 timer like gk7205v500.  (DDRC is at 0x11330000 vs the
 * V4-common 0x120d0000 hisi-ddr stub; left as-is for now — a RAM-backed
 * stub either way, not touched on the -kernel boot path.)
 *
 * Declares 2 CPUs (dual A7 MP2): the GICv2 makes ITARGETSR RAZ for a
 * uniprocessor GIC, so a 1-CPU model makes the kernel's gic_get_cpumask()
 * read 0 ("GIC CPU mask not found — kernel will fail to boot") and wedges
 * every interrupt-driven probe.  With 2 CPU interfaces ITARGETSR reports the
 * real mask and interrupts work.  CPU1 itself never onlines — the vendor
 * "lotus,xmorca-smp" release method isn't modeled — but the kernel times out
 * on it gracefully and continues single-core (same as the DTS expects).
 *
 * Boots the vendor SDK kernel (linux-5.10.y, xmorca_defconfig) via -kernel
 * with the appended xmorca.dtb, to busybox userspace from an initramfs or
 * from a jffs2 root on SPI-NOR (root=/dev/mtdblock4) — see run-gk7206.sh.
 */
#define GK7206_XMORCA_PERIPH                                                   \
    .max_cpus           = 2,            /* dual Cortex-A7 MP2 */               \
    .default_cpus       = 2,            /* 2 GIC CPU interfaces (ITARGETSR !RAZ) */ \
    .gpio_count         = 9,            /* GPIO0..GPIO8 @ 0x120b0000..0x120b8000 (DTS) */ \
    HISI_V4_COMMON_PERIPH,                                                     \
    /* 9 GPIO banks on GIC SPI 22..30 (DTS gpio_chip irqs 0x16..0x1e).  Must   \
     * follow the macro, which sets the wrong V4 gpio_irq_start=16 — a 16-base \
     * would put GPIO3..5 on 19..21 and clobber the SPIs below. */             \
    .gpio_irq_start     = 22,                                                  \
    .xmsp804_timer      = true,         /* XMedia kernel uses xmedia,sp804 */  \
    .femac_base         = 0x100d0000,   /* GMAC_REG_BASE — orca relocated FE MAC */ \
    /* DTS ethernet@100d0000 irq 0x25.  V4 common's femac_irq=33 is both wrong \
     * for orca and collides with sdhci mmc1 (also 33) below. */               \
    .femac_irq          = 37,                                                  \
    /* Flash: "lotus,fmc" (fmc100) — same IP as HiFMC V100 but a shuffled      \
     * register map, and the DT "control" reg sits at 0x10000010 (not          \
     * 0x10000000).  Memory window stays at 0x14000000 (from V4 common). */    \
    .fmc_ctrl_base      = 0x10000010,                                          \
    .fmc_variant        = 1,            /* FMC_VARIANT_FMC100 */               \
    .fmc_irq            = 36,           /* DTS: flash-memory-controller irq SPI 36 */ \
    /* SD/MMC — three "lotus,sdhci" controllers.  The V4-common defaults       \
     * (2 ports @0x10010000/0x10020000, IRQ 30/31) don't match orca, so        \
     * override with the DTS map: mmc0 eMMC @0x10010000 irq32,                 \
     * mmc1 @0x10040000 irq33, mmc2 SDIO @0x10020000 irq34.  A wrong IRQ       \
     * leaves the eMMC probe waiting forever for command-complete (which is    \
     * why the sdhci node used to be DTB-disabled). */                         \
    .num_sdhci          = 3,                                                   \
    .sdhci_bases        = { 0x10010000, 0x10040000, 0x10020000 },              \
    .sdhci_irqs         = { 32, 33, 34 },                                      \
    /* Pre-mark the SD/MMC DLL (delay-locked-loop) status registers as         \
     * locked/ready in the CRG.  The lotus,sdhci driver reads these via its    \
     * crg_regmap phandle (drivers/mmc/host/sdhci-xmorca.c) to pick a data     \
     * sampling phase; if they never report ready it settles on a bad phase    \
     * and card data init loops ("Skipping voltage switch").  DRV_DLL_STATUS   \
     * = LOCK|READY (bit15|bit14), SAMPL/DS/DS180 = SLAVE_READY (bit0). */     \
    .num_crg_defaults   = 8,                                                   \
    .crg_defaults       = {                                                    \
        { 0x208, 0x00000001 },  /* eMMC  SAMPL_DLL_STATUS: SLAVE_READY */      \
        { 0x210, 0x0000C000 },  /* eMMC  DRV_DLL_STATUS:   LOCK|READY  */      \
        { 0x214, 0x00000001 },  /* eMMC  DS_DLL_STATUS:    READY       */      \
        { 0x218, 0x00000001 },  /* eMMC  DS180_DLL_STATUS: READY       */      \
        { 0x224, 0x00000001 },  /* SDIO0 SAMPL_DLL_STATUS: SLAVE_READY */      \
        { 0x228, 0x0000C000 },  /* SDIO0 DRV_DLL_STATUS:   LOCK|READY  */      \
        { 0x238, 0x00000001 },  /* SDIO1 SAMPL_DLL_STATUS: SLAVE_READY */      \
        { 0x23c, 0x0000C000 },  /* SDIO1 DRV_DLL_STATUS:   LOCK|READY  */      \
    },                                                                         \
    /*                                                                         \
     * ---- xmorca-specific overrides (from qemu-boot/gk7206/xmorca.dts.tmp) ---- \
     * HISI_V4_COMMON_PERIPH carries the Hi3516EV/DV V4-die counts, bases and  \
     * IRQs; several don't match the GK7206 xmorca die.  Everything below is   \
     * re-specified from the vendor DTS so kernel probes land on real MMIO and \
     * wire the right GIC lines instead of the inherited V4 numbers.           \
     */                                                                        \
                                                                               \
    /* 4 PL011 UARTs @ 0x1204{0..3}000, GIC SPI 11..14 (V4 common had 3 @7..9). */ \
    .num_uarts          = 4,                                                   \
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000, 0x12043000 },  \
    .uart_irqs          = { 11, 12, 13, 14 },                                  \
                                                                               \
    /* 4 I2C @ 0x1206{0..3}000 (V4 common had 3; I2C wires no IRQ here). */    \
    .num_i2c            = 4,                                                   \
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000, 0x12063000 },  \
                                                                               \
    /* 3 PL022 SPI @ 0x1207{0..2}000, GIC SPI 19..21 (V4 common had 2 @14..15). */ \
    .num_spis           = 3,                                                   \
    .spi_bases          = { 0x12070000, 0x12071000, 0x12072000 },              \
    .spi_irqs           = { 19, 20, 21 },                                      \
                                                                               \
    /*                                                                         \
     * VEDU relocated on orca: DTS vedu@0x11800000 with a 2nd reg window at    \
     * 0x11420000 (V4 common put the primary block at 0x11410000).  jpge_base  \
     * stays 0x11420000 — that is exactly VEDU's 2nd window on this die, mapped \
     * as the hisi-vedu device's MMIO region 1.  DTS shares GIC SPI 45 between \
     * VEDU and MIPI RX; QEMU's GIC has one level per input, so we keep the    \
     * inherited vedu_irq (47, otherwise unused in this model) rather than     \
     * clobber MIPI's pic[45].  The encoder IRQ isn't exercised on -kernel boot. \
     */                                                                        \
    .vedu_base          = 0x11800000,                                          \
                                                                               \
    /*                                                                         \
     * Full regbank re-list for xmorca (V4 common's array can't be             \
     * partially overridden).  Deltas vs HISI_V4_COMMON_PERIPH:                \
     *   - hisi-ive relocated 0x11320000 -> 0x11490000 (DTS ive@11490000).     \
     *     "hisi-ive" instantiates the real functional IVE device, so this     \
     *     moves it to orca's address, not just a stub.                        \
     *   - added hisi-lsadc, hisi-cipher, hisi-vo, hisi-gdc, hisi-iocfg-vo     \
     *     (adc@120a0000, cipher@10050000, vo/gfbg@11280000, gdc@11480000,     \
     *     iocfg-controller3@11980000) — present in the DTS, previously        \
     *     unmapped so the kernel drivers faulted on probe.                    \
     * Note: 2nd watchdog@12030020 (DTS) sits inside wdt@12030000's window and \
     * is left unmodelled.  hisi-jpge stays folded into hisi-vedu (see above). \
     */                                                                        \
    .num_regbanks       = 20,                                                  \
    .regbanks           = {                                                    \
        { "hisi-misc",       0x12028000, 0x8000  },                            \
        { "hisi-ddr",        0x120d0000, 0x10000 },                            \
        { "hisi-iocfg-vio",  0x112c0000, 0x10000 },                            \
        { "hisi-iocfg-core", 0x120c0000, 0x10000 },                            \
        { "hisi-iocfg-ahb",  0x100c0000, 0x10000 },                            \
        { "hisi-pwm",        0x12080000, 0x10000 },                            \
        { "hisi-usb3",       0x10030000, 0x10000 },                            \
        { "hisi-aiao",       0x100e0000, 0x10000 },                            \
        { "hisi-acodec",     0x100f0000, 0x10000 },                            \
        { "hisi-vi-cap",     0x11000000, 0x200000 },                           \
        { "hisi-vi-proc",    0x11200000, 0x40000 },                            \
        { "hisi-vgs",        0x11300000, 0x10000 },                            \
        { "hisi-ive",        0x11490000, 0x10000 },  /* orca: was 0x11320000 */ \
        { "hisi-npu",        0x11340000, 0x10000 },                            \
        { "hisi-vpss",       0x11400000, 0x10000 },                            \
        { "hisi-lsadc",      0x120a0000, 0x1000  },  /* adc@120a0000 lotus,lsadc */ \
        { "hisi-cipher",     0x10050000, 0x10000 },  /* cipher@10050000 AES/SHA/RSA */ \
        { "hisi-vo",         0x11280000, 0x40000 },  /* vo/gfbg@11280000 display */ \
        { "hisi-gdc",        0x11480000, 0x10000 },  /* gdc@11480000 lens distort */ \
        { "hisi-iocfg-vo",   0x11980000, 0x10000 },  /* iocfg-controller3@11980000 */ \
    }

static const HisiSoCConfig gk7206v10_soc = {
    .name               = "gk7206v10",
    .desc               = "Goke GK7206V10 (Cortex-A7 MP2 + RISC-V MCU, 1.0 TOPS NPU)",
    .soc_id             = GOKE_SOC_ID_7206V10,
    GK7206_XMORCA_PERIPH,
    .ram_size_default   = 64 * MiB,     /* embedded 512Mb DDR2 */
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=xmedia "
                          "mmz=anonymous,0,0x42000000,32M",
};

/* GK7206V12A — the QFN128 top bin with 2Gb DDR3L (256 MiB). */
static const HisiSoCConfig gk7206v12a_soc = {
    .name               = "gk7206v12a",
    .desc               = "Goke GK7206V12A (Cortex-A7 MP2 + RISC-V MCU, 1.0 TOPS NPU, 4K)",
    .soc_id             = GOKE_SOC_ID_7206V12A,
    GK7206_XMORCA_PERIPH,
    .ram_size_default   = 256 * MiB,    /* embedded 2Gb DDR3L */
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=xmedia "
                          "mmz=anonymous,0,0x42000000,224M",
};

/*
 * ---- GK7606V1 series (codename xmfalcon) — PLACEHOLDER ----
 * Goke's A55 high-end 4K line, sibling of GK7206.  Cortex-A55 (aarch64),
 * external DDR3/DDR4/LPDDR4, adds USB3.  The SDK ships NO xmfalcon board
 * config (only xmorca/GK7206), so the map below — from the SDK u-boot
 * arch/arm/include/asm/arch-xmfalcon/platform.h — is UNVERIFIED against
 * silicon or a boot.  Registered so the machine name exists and future work
 * has a scaffold; NOT expected to boot yet.  UNKNOWNS flagged inline:
 *   - GIC physical bases (u-boot uses virtual 0xf6801000/0xf6802000);
 *   - all IRQ numbers (borrowed from hi3519dv500, likely wrong);
 *   - SoC ID and external DDR size.
 * Deltas from GK7206/xmorca: A55 aarch64 + GICv3 + ATF/PSCI SMP; FE/GMAC
 * 0x100d0000 -> gigabit GMAC 0x11900000; adds USB3 @0x10040000; ext DDR.
 * Shared 0x12xx control block (UART/sysctl/CRG/sp804/GPIO) is authoritative.
 */
static const HisiSoCConfig gk7606v1_soc = {
    .name               = "gk7606v1",
    .desc               = "Goke GK7606V1 (Cortex-A55, 4K — PLACEHOLDER, does not boot yet)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a55"),
    .soc_id             = GOKE_SOC_ID_7606V1,
    .aarch64            = true,
    .uboot_load_addr    = 0x40700000,   /* CONFIG_SYS_TEXT_BASE_ORI (xmfalcon.h) */
    .psci_conduit       = 1,

    .ram_size_default   = 256 * MiB,    /* external DDR; size UNVERIFIED */
    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 128 * KiB,

    .use_gic            = true,
    .gic_version        = 3,
    .gic_dist_base      = 0x12400000,   /* PLACEHOLDER — u-boot virt 0xf6801000 */
    .gic_redist_base    = 0x12440000,   /* PLACEHOLDER — u-boot virt 0xf6802000 */
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,   /* SYS_CTRL_REG_BASE (authoritative) */
    .crg_base           = 0x12010000,   /* CRG_REG_BASE (authoritative) */

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 }, /* authoritative */
    .uart_irqs          = { 5, 6, 7 },  /* PLACEHOLDER irqs */

    .xmsp804_timer      = true,         /* xmedia,sp804 @ 0x12000000 */
    .num_timers         = 1,
    .timer_bases        = { 0x12000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 19, 20 },   /* PLACEHOLDER irqs */

    .fmc_ctrl_base      = 0x10000000,   /* FMC_REG_BASE (authoritative) */
    .fmc_mem_base       = 0x14000000,   /* FMC_MEM_BASE (authoritative) */

    .gpio_base          = 0x120b0000,   /* GPIO0_REG_BASE (authoritative) */
    .gpio_count         = 10,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,           /* PLACEHOLDER */

    /* Gigabit GMAC (GMAC_REG_BASE 0x11900000).  Model like hi3519dv500's
     * gmac-v5 (16-byte descriptors); IRQ is a PLACEHOLDER. */
    .gmac_base          = 0x11900000,
    .gmac_irq           = 54,           /* PLACEHOLDER */
    .gmac_desc_size     = 16,
    .gmac_rx_offset     = 2,
};

/*
 * V5 family (~2023): Hi3516CV608 / CV610 / CV613 — Dual Cortex-A7 MP2 + NPU.
 * Video: H.265/H.264, up to 4K.  New 0x11xxxxxx address map, GIC @ 0x124xxxxx.
 * NPU: 0.2 TOPS (CV608), 0.5 TOPS (CV610), 1 TOPS (CV613).
 * Same die, different feature tiers.
 * All share identical peripheral addresses; only SoC ID differs.
 *
 * Datasheet model suffixes → chip IDs (from Section 1.2.14):
 *   10B  → Hi3516CV610  0x3516C610  0.5 TOPS, 5M,  DDR2, QFN
 *   20S  → Hi3516CV613  0x3516C613  1 TOPS,   4K,  DDR3, QFN
 *   00S  → unknown ID                1 TOPS,   4K,  DDR3, QFN
 *   20G  → unknown ID                1 TOPS,   4K,  DDR3, QFN, GB35114
 *   00G  → unknown ID                1 TOPS,   4K,  ext DDR3, BGA, GB35114
 *   (separate chip) Hi3516CV608  0x3516C608  0.2 TOPS, 3M, DDR2, QFN
 *
 * ipctool also lists Hi3516DV500 (0x3516D500) and Hi3519DV500 (0x3519D500)
 * as HISI_OT generation — likely same V5 address map, awaiting SDK/lab.
 */
static const HisiSoCConfig hi3516cv608_soc = {
    .name               = "hi3516cv608",
    .desc               = "HiSilicon Hi3516CV608 (Cortex-A7 MP2, ~CV610)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV608,
    .gzip_base          = 0x170F0000,    /* V5 emar HW gzip (vendor platform.h) */
    /* 64 MiB on-chip DDR2 (512Mb).  Kernel gets 32 MiB, vendor mmz.ko
     * claims the upper 32 MiB at 0x42000000. */
    .ram_size_default   = 64 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,32M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    /* Same SP804-style udelay timer as cv610 (cv608 is the same A7 MP2 die):
     * without it U-Boot's udelay() spins forever and hangs in the SPI-NOR
     * probe on a flash boot (-machine hi3516cv608,flash-file=...). */
    .num_timers         = 1,
    .timer_bases        = { 0x11000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

/*
 * Hi3516CV610: V5 generation (~2023) — Dual Cortex-A7 MP2, GICv2.
 * Completely new address map (0x11xxxxxx peripherals, GIC @ 0x124xxxxx).
 * No SP804 timer — uses ARM arch timer exclusively.
 * FEMAC-v2 with integrated FEPHY, "nebula,sdhci" for SD/MMC.
 * FMC memory window below registers at 0x0F000000.
 */
static const HisiSoCConfig hi3516cv610_soc = {
    .name               = "hi3516cv610",
    .desc               = "HiSilicon Hi3516CV610 (Cortex-A7 MP2)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV610,
    .gzip_base          = 0x170F0000,    /* V5 emar HW gzip (vendor platform.h) */
    /* 128 MiB on-chip DDR3/3L (1Gb) on CV610-20S/20G.  Kernel gets 32 MiB,
     * vendor mmz.ko claims the upper 96 MiB at 0x42000000. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,96M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    /* Linux uses the Cortex-A7 generic timer (24 MHz, via DTB).  But the
     * cv610 U-Boot's udelay() polls an SP804-style countdown timer at
     * 0x11000000 (vendor platform.h: CFG_TIMERBASE, VALUE@0x4,
     * CFG_TIMER_CLK = 3 MHz).  Without it udelay() spins forever and U-Boot
     * hangs in the SPI-NOR probe during a flash boot (`-machine
     * hi3516cv610,flash-file=...` with no -kernel).  Wire one SP804 so the
     * flash-boot path works; the IRQ is unused (U-Boot polls, Linux uses the
     * arch timer and has no DT node here), so any free SPI suffices. */
    .num_timers         = 1,
    .timer_bases        = { 0x11000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,           /* per-port: SPI 23..33 */

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

static const HisiSoCConfig hi3516cv613_soc = {
    .name               = "hi3516cv613",
    .desc               = "HiSilicon Hi3516CV613 (Cortex-A7 MP2, ~CV610)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV613,
    .gzip_base          = 0x170F0000,    /* V5 emar HW gzip (vendor platform.h) */
    /* CV613 is the 4K-capable V5 die (no public datasheet); follow CV610-20S.
     * Kernel gets 32 MiB, vendor mmz.ko claims upper 96 MiB at 0x42000000. */
    .ram_size_default   = 128 * MiB,
    .kernel_mem_mb      = 32,
    .extra_cmdline      = "mmz_allocator=hisi "
                          "mmz=anonymous,0,0x42000000,96M",

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    /* Same SP804-style udelay timer as cv610/cv608 (cv613 is the same A7 MP2
     * die): without it U-Boot's udelay() spins forever and hangs in the
     * SPI-NOR probe on a flash boot (-machine hi3516cv613,flash-file=...). */
    .num_timers         = 1,
    .timer_bases        = { 0x11000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

/*
 * Hi3519DV500 / Hi3516DV500 — V5 "HISI_OT" aarch64 die (Cortex-A55, ~2024).
 * Shares the V5/SS626 0x11xxxxxx peripheral map with CV610 (PL011 UART0 @
 * 0x11040000, sysctl 0x11020000, crg 0x11010000, GIC @ 0x124xxxxx, HiFMC @
 * 0x10000000, emar HW gzip @ 0x170F0000) but is 64-bit and boots a gzip
 * self-extracting u-boot-z.bin: the boot ROM/GSL loads the image at its link
 * address CONFIG_SYS_TEXT_BASE_ORI=0x48700000, the on-die HW gzip (hisi-gzip)
 * decompresses it to CONFIG_SYS_TEXT_BASE=0x48800000, and U-Boot runs from DDR.
 */
static const HisiSoCConfig hi3519dv500_soc = {
    .name               = "hi3519dv500",
    .desc               = "HiSilicon Hi3519DV500 (Cortex-A55, V5/aarch64)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a55"),
    .soc_id             = HISI_SOC_ID_DV500,
    .aarch64            = true,
    .max_cpus           = 2,            /* dual Cortex-A55 */
    .default_cpus       = 2,            /* boot both cores by default */
    .uboot_load_addr    = 0x48700000,   /* CONFIG_SYS_TEXT_BASE_ORI (u-boot.lds) */
    .psci_conduit       = 1,            /* QEMU_PSCI_CONDUIT_SMC (kernel SMP) */
    .gzip_base          = 0x170F0000,   /* V5 emar HW gzip (shared with CV610) */

    /* DDR @ 0x40000000.  Vendor U-Boot probes 512 MiB and relocates itself to
     * the top of RAM (~0x5ff20000), so map the full 512 MiB. */
    .ram_size_default   = 512 * MiB,
    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 128 * KiB,

    .use_gic            = true,
    .gic_version        = 3,            /* vendor dtsi: compatible "arm,gic-v3" */
    .gic_dist_base      = 0x12400000,   /* GICD (dtsi reg[0]) */
    .gic_redist_base    = 0x12440000,   /* GICR redistributor (dtsi reg[1]) */
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    /* vendor dtsi: uart0=GIC_SPI 5, uart1=6, uart2=7.  Polled u-boot ignored
     * these, but Linux userspace TTY output is interrupt-driven, so a wrong
     * SPI = silent console after the kernel hands off to init. */
    .uart_irqs          = { 5, 6, 7 },
    /* Gigabit Ethernet: vendor "gmac-v5" (drivers/net/ethernet/vendor/gmac),
     * same HiGMAC ring + MDIO layout as the existing hisi-gmac model, with
     * 4-word (16-byte) descriptors.  MAC core + integrated MDIO @0x10290000,
     * GIC_SPI 54 (queue 0). */
    .gmac_base          = 0x10290000,
    .gmac_irq           = 54,
    .gmac_desc_size     = 16,
    .gmac_rx_offset     = 2,            /* gmac-v5 driver reserves NET_IP_ALIGN */

    /* U-Boot udelay() polls an SP804-style countdown timer at 0x11000000
     * (vendor platform.h CFG_TIMERBASE, 3 MHz) — wire one so the flash boot
     * path's SPI-NOR probe doesn't spin forever (same as CV610). */
    .num_timers         = 1,
    .timer_bases        = { 0x11000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,
    .gpio0_input_high   = (1u << 0),    /* GPIO0_0 update-button pull-up (high) */

    .num_regbanks       = 7,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        /* Pinmux (IOCFG) controllers — vendor sys_config: iocfg=0x10260000,
         * iocfg2=0x179F0000, iocfg3=0x0EFF0000; plus the AON pad-mux in the
         * PMC at 0x11120000.  RAM-backed regbanks so the kernel's pin_mux.c
         * writes persist and /dev/mem pinmux tools (ipctool reginfo) read the
         * configured mux selections instead of all-zero. */
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },   /* CORE_IOCFG window, kept as a stub */
        { "hisi-iocfg2",     0x179F0000, 0x10000 },
        { "hisi-iocfg3",     0x0EFF0000, 0x10000 },
        { "hisi-pmc",        0x11120000, 0x10000 },   /* AON pad-mux (GPIO9) + PMC regs */
    },
};

static const HisiSoCConfig hi3516dv500_soc = {
    .name               = "hi3516dv500",
    .desc               = "HiSilicon Hi3516DV500 (Cortex-A55, V5/aarch64)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a55"),
    .soc_id             = HISI_SOC_ID_3516DV500,
    .aarch64            = true,
    .max_cpus           = 2,            /* dual Cortex-A55 */
    .default_cpus       = 2,            /* boot both cores by default */
    .uboot_load_addr    = 0x48700000,
    .psci_conduit       = 1,
    .gzip_base          = 0x170F0000,

    .ram_size_default   = 512 * MiB,  /* U-Boot relocates to top of 512 MiB */
    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 128 * KiB,

    .use_gic            = true,
    .gic_version        = 3,            /* vendor dtsi: compatible "arm,gic-v3" */
    .gic_dist_base      = 0x12400000,   /* GICD (dtsi reg[0]) */
    .gic_redist_base    = 0x12440000,   /* GICR redistributor (dtsi reg[1]) */
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    /* vendor dtsi: uart0=GIC_SPI 5, uart1=6, uart2=7.  Polled u-boot ignored
     * these, but Linux userspace TTY output is interrupt-driven, so a wrong
     * SPI = silent console after the kernel hands off to init. */
    .uart_irqs          = { 5, 6, 7 },
    /* Gigabit Ethernet: vendor "gmac-v5" (drivers/net/ethernet/vendor/gmac),
     * same HiGMAC ring + MDIO layout as the existing hisi-gmac model, with
     * 4-word (16-byte) descriptors.  MAC core + integrated MDIO @0x10290000,
     * GIC_SPI 54 (queue 0). */
    .gmac_base          = 0x10290000,
    .gmac_irq           = 54,
    .gmac_desc_size     = 16,
    .gmac_rx_offset     = 2,            /* gmac-v5 driver reserves NET_IP_ALIGN */

    .num_timers         = 1,
    .timer_bases        = { 0x11000000 },
    .timer_irqs         = { 4 },
    .timer_freq         = 3000000,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,
    .gpio0_input_high   = (1u << 0),    /* GPIO0_0 update-button pull-up (high) */

    .num_regbanks       = 7,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        /* Pinmux (IOCFG) controllers — vendor sys_config: iocfg=0x10260000,
         * iocfg2=0x179F0000, iocfg3=0x0EFF0000; plus the AON pad-mux in the
         * PMC at 0x11120000.  RAM-backed regbanks so the kernel's pin_mux.c
         * writes persist and /dev/mem pinmux tools (ipctool reginfo) read the
         * configured mux selections instead of all-zero. */
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },   /* CORE_IOCFG window, kept as a stub */
        { "hisi-iocfg2",     0x179F0000, 0x10000 },
        { "hisi-iocfg3",     0x0EFF0000, 0x10000 },
        { "hisi-pmc",        0x11120000, 0x10000 },   /* AON pad-mux (GPIO9) + PMC regs */
    },
};

/*
 * Hi3536DV100 (DVR/NVR family): 2017, single Cortex-A7 @850MHz, decode-only NVR.
 * H.265/H.264 dec, 4×1080p20, FE Ethernet, 1×SATA 2.0, USB 2.0 host.
 * First DVR/NVR-class machine in this fork — see plan
 * /home/dima/.claude/plans/do-exhaustive-research-about-luminous-cat.md
 *
 * Memory map and IRQs from vendor kernel
 * arch/arm/boot/dts/hi3536dv100.dtsi (RichStrong/Hi3536DV100 4.9.37 SDK).
 * SoC ID confirmed against ipctool hal_hisi.c (0x3536D100, V4+ word layout).
 * DDR base 0x80000000 / 512 MiB confirmed in hi3536dv100-demb.dts.
 */
static const HisiSoCConfig hi3536dv100_soc = {
    .name               = "hi3536dv100",
    .desc               = "HiSilicon Hi3536DV100 (Cortex-A7, NVR decode-only)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3536DV100,
    .default_sensor     = NULL,             /* NVR has no image sensor */
    /* DDR3L 16-bit @ 800-933 MHz, max 2 GB.  512 MiB matches the vendor
     * DEMB reference board (memory@80000000 size=0x20000000).  For boot-
     * to-shell we let Linux own all 512 MiB — vendor MPP mmz.ko isn't
     * loaded so no CMA reservation is needed.  When the media pipeline
     * gets emulated later, switch to kernel_mem_mb=64 + the canonical
     * "mmz=anonymous,0,0x84000000,448M" reservation. */
    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    /* 4× SP804 dual-timer blocks → 8 timers total.  Only timer0 is enabled
     * in the DEMB DTS, but instantiate all 4 so vendor module-loaded
     * timers (timer3/4/5 in vendor SDK examples) work. */
    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 0,                /* SPI bus exposed via FMC only */

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    /* 6× PL061 GPIO ports, 64KB stride, per-port IRQs starting at SPI 55 */
    .gpio_base          = 0x12150000,
    .gpio_count         = 6,
    .gpio_stride        = 0x10000,
    .gpio_irq_start     = 55,

    /* HiSilicon DW DMAC (NOT PL080) — stub via regbanks[] below */
    .dma_base           = 0x11020000,
    .dma_type           = "hisi-regbank",

    /* FEMAC v2: port at 0x10010000, MDIO + reg-glb at +0x1100/+0x1300 */
    .femac_base         = 0x10010000,
    .femac_irq          = 11,

    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c",        /* HiBVT IP */

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    /* SATA AHCI v1.x, 1 port */
    .sata_base          = 0x10030000,
    .sata_irq           = 17,
    .sata_num_ports     = 1,

    /* USB 2.0 host: EHCI + companion OHCI */
    .usb_ehci_base      = 0x11010000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x11000000,
    .usb_ohci_irq       = 18,

    /* CRG default register state — mimic U-Boot clock init.  Hi3536DV100
     * register map differs from EV300; offsets verified against vendor
     * drivers/clk/hisilicon/clk-hi3536dv100.c.  Set conservative defaults
     * (FMC + ETH + UART clocks ungated) so the kernel finds working
     * peripherals.  Expand if smoke-test reveals more gates needed. */
    .num_crg_defaults   = 0,

    /* Stubs for blocks the kernel touches before module-load that aren't
     * full peripherals: pinmux, USB PHY / misc CRG, DDR controller config,
     * cipher engine, IR receiver, DMAC (regbank-substituted PL080), and
     * the media subsystem (VOU/VGS/AUDIO/VDEC/TDE/JPGE/JPGD).  Vendor MPP
     * modules are loaded later from rootfs and reach for these MMIO
     * windows; without backing storage their writes vanish and they busy-
     * loop on the next read. */
    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-dmac",       0x11020000, 0x1000  },
        { "hisi-cipher",     0x11030000, 0x10000 },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
        { "hisi-pinmux",     0x120F0000, 0x1000  },
        { "hisi-vou",        0x13020000, 0x10000 },
        { "hisi-aiao",       0x13040000, 0x10000 },
        { "hisi-vgs",        0x13100000, 0x10000 },
        { "hisi-jpgd",       0x13110000, 0x10000 },
        { "hisi-jpge",       0x13120000, 0x10000 },
        { "hisi-vdec",       0x13200000, 0x10000 },
    },
};

/*
 * Hi3521A (DVR/NVR family): 2014-2015, single Cortex-A7, hybrid DVR/NVR.
 * H.264 enc+dec, 4×1080p, GbE, 2× SATA 2.0, USB 2.0 host, AHD/TVI/CVI
 * analog support via external NVP6124B 4-channel decoder over I2C.
 *
 * BDS prose mistakenly calls it "Cortex-A9" — vendor `hi3521a.dtsi`
 * declares `arm,cortex-a7`.  DTS is authoritative.
 *
 * SoC ID 0x35210100 confirmed in ipctool hal_hisi.c.  DDR base
 * 0x80000000 / 512 MiB confirmed in hi3521a-demb.dts.
 */
static const HisiSoCConfig hi3521a_soc = {
    .name               = "hi3521a",
    .desc               = "HiSilicon Hi3521A (Cortex-A7, hybrid DVR/NVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3521V100,
    /* External NVP6124B AHD decoder on I2C0 @ 7-bit addr 0x60. */
    .default_sensor     = "nvp6124b",

    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    /* PL022 SPI controller — DEMB DTS attaches two `rohm,dh2228fv` slaves */
    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    /* DMA: HiSilicon DW DMAC at 0x10060000 (NOT 0x11020000 like Hi3536DV100).
     * Stub via regbanks[] below; the dma_type selector skips PL080. */
    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    /* Gigabit Ethernet — `higmac` driver, MDIO at +0x3c0 */
    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    /* I2C0 — DesignWare register layout (vendor i2c-hisilicon.c driver),
     * matches the fork's hisi-i2c-dw model. */
    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    /* SATA AHCI v1.x, 2 ports per BDS (1 used in DEMB) */
    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 2,

    /* USB 2.0 host: EHCI + companion OHCI (different addresses from Hi3536DV100) */
    .usb_ehci_base      = 0x10040000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x10030000,
    .usb_ohci_irq       = 18,

    .num_crg_defaults   = 0,

    /* Stubs for blocks the kernel touches before module-load:
     * USB-PHY/misc, DDR controller, DMAC (regbank-substituted), cipher,
     * VDEC, IR.  Media subsystem (VIU/VOU/VPSS/JPEG/VEDU/AIO/HDMI) is
     * loaded via vendor MPP modules from rootfs and added later if needed. */
    .num_regbanks       = 6,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-vdec",       0x10080000, 0x4000  },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3531A (DVR/NVR family): 2016, dual Cortex-A9 SMP @ 1.1 GHz.
 * H.264 8×1080p enc + 4×1080p dec, 1× GbE, 4× muxed lanes (SATA 3.0 /
 * PCIe 2.0 / USB 3.0).  Memory map and IRQs from vendor kernel
 * arch/arm/boot/dts/hi3531a.dtsi (RichStrong Linux 4.9.37).
 *
 * Phase-3 boot-to-shell scope: single-CPU boot (max_cpus=1).  Vendor SMP
 * bringup uses CRG bit 4 (REG_CRG32 / 0x80) which the fork's hisi-crg
 * model doesn't yet detect — that's a Phase-3.5 follow-up if vendor MPP
 * modules need both CPUs online.  PCIe controllers stubbed via regbank
 * (the kernel pcie driver fails enumeration cleanly).
 */
static const HisiSoCConfig hi3531a_soc = {
    .name               = "hi3531a",
    .desc               = "HiSilicon Hi3531A (Cortex-A9, dual-core, DVR/NVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .soc_id             = HISI_SOC_ID_3531A,
    .max_cpus           = 1,                /* SMP deferred to Phase 3.5 */
    .default_sensor     = NULL,             /* multi-channel — no single sensor */

    /* DDR3 32-bit, max 3 GiB.  DEMB board ships 3 GiB at 0x40000000. */
    .ram_size_default   = 1 * GiB,          /* keep modest for QEMU */
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x40000000,       /* NOT 0x80000000 like Hi3521A/3536DV100 */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    /* A9 MP-core peripheral block at 0x10300000 (size 0x2000) covers
     * SCU @+0x000, GIC cpu @+0x100, gtimer @+0x200, mptimer @+0x600,
     * wdt @+0x620, GIC dist @+0x1000.  Use a9mpcore_priv to keep these
     * in one container (they overlap as separate sysbus regions). */
    .gic_mpcore_base    = 0x10300000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 4,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000, 0x12130000 },
    .uart_irqs          = { 6, 7, 8, 20 },

    /* 4× SP804 dual-timer.  Vendor kernel binds the first three under the
     * "hisilicon,hisp804" wrapper (clocksource + per-CPU local timer);
     * the underlying register layout is plain SP804.  Only timer 3 (the
     * "dual_timer3@12030000" sp804 instance) is active in DEMB DTS, but
     * we instantiate all four so the wrapper finds both clocksource and
     * local-timer regions when compatible matches. */
    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    /* PL022 SPI controller — DEMB attaches 4× rohm,dh2228fv slaves */
    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    /* DMA: HiSilicon DW DMAC at 0x10060000 (same address as Hi3521A) */
    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    /* Gigabit Ethernet — `higmac` driver, MDIO at +0x3c0 */
    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    /* 2× I2C — DesignWare register layout */
    .num_i2c            = 2,
    .i2c_bases          = { 0x120c0000, 0x122e0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    /* SATA AHCI v1.x, 4 ports (1 used in DEMB) */
    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 4,

    /* USB 2.0 host: EHCI + companion OHCI in IOCH1 range */
    .usb_ehci_base      = 0x100c0000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x100b0000,
    .usb_ohci_irq       = 18,

    /* USB 3.0 XHCI — first DVR/NVR SoC to expose it */
    .xhci_base          = 0x11000000,
    .xhci_irq           = 22,
    .xhci_slots         = 4,
    .xhci_intrs         = 1,

    .num_crg_defaults   = 0,

    /* Stub regbanks for blocks the kernel touches before module-load.
     * PCIe controllers, parallel NAND, L2 cache (PL310 at 0x10700000),
     * cipher, VDEC, IR, DDR/misc — all stubbed via regbank.  PL310
     * enumeration may hang on a regbank stub; if so, patch the DT to
     * disable the L2 node via hisilicon_patch_appended_dtb. */
    .num_regbanks       = 9,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-vdec",       0x10080000, 0x4000  },
        { "hisi-nandc",      0x10010000, 0x10000 },
        { "hisi-pcie0",      0x11020000, 0x10000 },
        { "hisi-pcie1",      0x11030000, 0x10000 },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3521D V100 (DVR/NVR family): 2017, dual Cortex-A7 SMP @ 1.3 GHz, H.265.
 * Direct H.265 follow-up to Hi3521A — same peripheral layout per BDS, same
 * DDR base, same address scheme.  Vendor SDK ships Linux 3.18.20.
 *
 * Single-CPU boot for Phase 5 (max_cpus=1) — vendor SMP would need the same
 * CRG bit-handling extension as Hi3531A's deferred Phase 3.5.
 */
static const HisiSoCConfig hi3521dv100_soc = {
    .name               = "hi3521dv100",
    .desc               = "HiSilicon Hi3521DV100 (Cortex-A7, dual-core, H.265 DVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3521DV100,
    .max_cpus           = 1,
    .default_sensor     = "nvp6124b",       /* analog AHD/TVI/CVI decoder */

    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,       /* per uImage load addr 0x80008000 */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 2,

    .usb_ehci_base      = 0x10040000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x10030000,
    .usb_ohci_irq       = 18,

    .num_regbanks       = 6,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-vdec",       0x10080000, 0x4000  },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3520DV200 (DVR/NVR family, legacy): 2013, single Cortex-A9 @ ~660 MHz,
 * H.264 only.  V1-era 0x20xxxxxx peripheral address scheme.  Vendor SDK
 * ships Linux 3.0.8.  OpenIPC firmware
 * (https://github.com/OpenIPC/firmware) ships a Linux 3.0.8-built uImage
 * for this SoC — we boot that directly via `-M hi3520dv200`.
 *
 * Memory map confirmed against OpenIPC linux fork
 * (`hisilicon-hi3520dv200` branch, `arch/arm/mach-hi3520d/include/mach/
 * platform.h`).  A9 mpcore at ARM_INTNL_BASE = 0x20300000 (SCU @+0x000,
 * GIC cpu @+0x100, GIC dist @+0x1000) — same overlap pattern as Hi3531A,
 * a9mpcore_priv handles it.
 */
static const HisiSoCConfig hi3520dv200_soc = {
    .name               = "hi3520dv200",
    .desc               = "HiSilicon Hi3520DV200 (Cortex-A9, single, H.264 DVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .soc_id             = HISI_SOC_ID_3520DV200,
    .max_cpus           = 1,
    .default_sensor     = NULL,
    /* MACH_TYPE_HI3520D = 8000 — vendor 3.0.8 + OpenIPC kernel use ATAGs
     * MACHINE_START "hi3520d" without DT, so the ATAGs board ID matters. */
    .board_id           = 8000,

    .ram_size_default   = 256 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 16 * KiB,

    .use_gic            = true,
    /* A9 mpcore peripheral block at 0x20300000 (size 0x2000) covers SCU
     * (+0x000), GIC cpu (+0x100), GIC dist (+0x1000).  Use a9mpcore_priv
     * to keep these in one container (they overlap as separate sysbus
     * regions). */
    .gic_mpcore_base    = 0x20300000,
    .gic_num_spi        = 96,               /* HI3520D_IRQ_START + 96 = NR_IRQS */

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    /* PL011 UART0 at 0x20080000 GIC SPI 8.  Vendor mach registers UART0
     * + UART1; OpenIPC inittab uses /dev/console which Linux maps to the
     * cmdline-specified console=ttyAMA0. */
    .num_uarts          = 2,
    .uart_bases         = { 0x20080000, 0x20090000 },
    .uart_irqs          = { 8, 9 },

    /* SP804 dual-timer at 0x20000000 / 0x20010000 IRQs 3/4 (TIMER01/23
     * per vendor irqs.h).  Vendor formula: timer_clk_hz = busclk / 4 with
     * busclk = 396 MHz from CRG defaults below → 99 MHz timer rate. */
    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 99000000,

    .num_spis           = 0,

    /* HISFC350 SPI flash controller — vendor U-Boot platform.h hardcodes
     * SFC_REG_BASE = 0x10010000 and SFC_MEM_BASE = 0x58000000 (same V1-era
     * layout as Hi3516CV100 / Hi3518EV100).  Lets `-machine hi3520dv200,
     * flash-file=...` boot vendor / OpenIPC U-Boot directly from the
     * NOR image (openhisilicon#89). */
    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

    .num_i2c            = 1,
    .i2c_bases          = { 0x200d0000 },
    .i2c_type           = "hisi-i2c",       /* CV100-class HiBVT */

    .rtc_base           = 0x20060000,
    .rtc_irq            = 0,

    /* HiSilicon HIL2V200 L2 cache controller — vendor 3.0.x kernel hangs
     * in l2cache_driver_init without this (openhisilicon#66). */
    .l2cache_base       = 0x20700000,

    /* PL011 UARTEN pre-enable — vendor 3.0.8 kernel writes to UARTDR
     * before initialising UARTCR (relies on U-Boot to set CR=0x301). */
    .uart_pre_enable    = true,

    /* HIL_AMBA_DEVICE() in vendor mach-hi3520d/core.c hardcodes a
     * 0x10000 resource size for uart:0/uart:1, so the AMBA bus probes
     * PrimeCell ID at the END of that window (offset 0xffe0/0xfff0).
     * Mirror PL011 at base + 0xf000 so the IDs read back correctly and
     * the AMBA driver binds — otherwise /dev/ttyAMA0 never appears and
     * userspace can't open /dev/console (openhisilicon#70). */
    .uart_window_size   = 0x10000,

    /* HiSilicon SF Ethernet (drivers/net/hieth-sf/) — vendor 3.0.x
     * kernel hangs in hieth_init (sys-hi3520d.c set_phy_valtage busy
     * waits on MDIO_RWCTRL bit 15) without this (openhisilicon#68).
     * Vendor `drivers/net/hieth-sf/Kconfig` sets HIETH_IRQNUM = 56 for
     * ARCH_HI3520D (Linux IRQ); GIC SPI = 56 - HI3520D_IRQ_START(32). */
    .hieth_sf_base      = 0x10090000,
    .hieth_sf_irq       = 24,

    /* CRG default register state — vendor get_bus_clk() reads CRG0 + CRG1
     * + A9_AXI_SCALE_REG to compute busclk = (24 MHz / refdiv * fbdiv) / 2.
     * With our regbank reads returning 0 the kernel divides by zero and
     * sets timer_clk_hz to garbage → "Timer with delta zero, disabling".
     * Seed: refdiv=1, fbdiv=33, pstdiv1=1, pstdiv2=1, A9 scale bits=0xc
     * → foutvco = 24*33 = 792 MHz; busclk = 396 MHz. */
    .num_crg_defaults   = 3,
    .crg_defaults       = {
        { 0x00, 0x09000000 },               /* CRG0: pstdiv1=1 / pstdiv2=1 */
        { 0x04, 0x00001021 },               /* CRG1: refdiv=1 / fbdiv=33 */
        { 0x28, 0x0000000c },               /* A9_AXI_SCALE_REG */
    },
};

/*
 * Hi3520DV300 (DVR/NVR family): 2016, single Cortex-A7 @ 800 MHz, H.264.
 * Per LKML hint, shares ARCH_HI3521A in the kernel — peripheral layout is
 * the same as Hi3521A, just lower clock.  Treat as Hi3521A clone with
 * different SoC ID and FE Ethernet (vs Hi3521A's GbE per Brief Data Sheet).
 */
static const HisiSoCConfig hi3520dv300_soc = {
    .name               = "hi3520dv300",
    .desc               = "HiSilicon Hi3520DV300 (Cortex-A7, single, H.264 DVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3520DV300,
    .default_sensor     = "nvp6124b",

    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 16 * KiB,         /* BDS says 16 KiB SRAM */

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 2,

    .usb_ehci_base      = 0x10040000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x10030000,
    .usb_ohci_irq       = 18,

    .num_regbanks       = 5,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3520DV400 (DVR/NVR family): 2017+, single Cortex-A7 @ 1.3 GHz, H.265.
 * Co-distributed in the same SDK as Hi3521D (Linux 3.18.20).  Mobile DVR
 * and NDI live-encoder boards.  Same peripheral set as Hi3521D, single CPU.
 */
static const HisiSoCConfig hi3520dv400_soc = {
    .name               = "hi3520dv400",
    .desc               = "HiSilicon Hi3520DV400 (Cortex-A7, single, H.265 DVR/encoder)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3520DV400,
    .default_sensor     = NULL,

    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 1,                /* Dv400 BDS lists 1× SATA only */

    .usb_ehci_base      = 0x10040000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x10030000,
    .usb_ohci_irq       = 18,

    .num_regbanks       = 5,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3531DV100 (DVR/NVR family): 2017, dual Cortex-A9 SMP @ 1.4 GHz, H.265.
 * Direct H.265 follow-up to Hi3531A — vendor BDS says identical peripheral
 * pin/layout.  Reuses Hi3531A's a9mpcore_priv path.
 *
 * Single-CPU boot for Phase 5 (same SMP caveat as Hi3531A).
 */
static const HisiSoCConfig hi3531dv100_soc = {
    .name               = "hi3531dv100",
    .desc               = "HiSilicon Hi3531DV100 (Cortex-A9, dual-core, H.265 DVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .soc_id             = HISI_SOC_ID_3531DV100,
    .max_cpus           = 1,
    .default_sensor     = NULL,

    .ram_size_default   = 1 * GiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x40000000,       /* same as Hi3531A */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_mpcore_base    = 0x10300000,       /* a9mpcore_priv layout */
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 4,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000, 0x12130000 },
    .uart_irqs          = { 6, 7, 8, 20 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    .num_i2c            = 2,
    .i2c_bases          = { 0x120c0000, 0x122e0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 4,

    .usb_ehci_base      = 0x100c0000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x100b0000,
    .usb_ohci_irq       = 18,

    .xhci_base          = 0x11000000,
    .xhci_irq           = 22,
    .xhci_slots         = 4,
    .xhci_intrs         = 1,

    .num_regbanks       = 9,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-vdec",       0x10080000, 0x4000  },
        { "hisi-nandc",      0x10010000, 0x10000 },
        { "hisi-pcie0",      0x11020000, 0x10000 },
        { "hisi-pcie1",      0x11030000, 0x10000 },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3535 (DVR/NVR family): 2013, dual Cortex-A9 SMP @ 1 GHz, H.264.
 * Pure NVR (no analog VI), 2× GbE TOE, USB 3.0 + integrated audio codec,
 * 1× PCIe 2.0/SATA 3.0 mux + 2× SATA 3.0.  Linux 3.4 vendor SDK (older
 * than Hi3531A).  Same Hi3531A-class peripheral scheme.
 *
 * Phase 5 boot-to-shell likely needs Linux 3.4 → 3.18 backport effort
 * similar to Hi3536 flagship; mark architecture-only for now.
 */
static const HisiSoCConfig hi3535_soc = {
    .name               = "hi3535",
    .desc               = "HiSilicon Hi3535 (Cortex-A9, dual-core, H.264 NVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .soc_id             = HISI_SOC_ID_3535,
    .max_cpus           = 1,
    .default_sensor     = NULL,             /* NVR-only, no analog */
    .board_id           = 0,                /* Linux 3.4 boot may need ATAGs id */

    .ram_size_default   = 1 * GiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    /* Vendor 3.4 uImage loads at 0x80008000 (DDR base 0x80000000), but
     * that kernel hangs in QEMU.  Phase 4.5 uses a Hi3531A-built Linux
     * 4.9 kernel (same A9 + a9mpcore_priv + identical peripherals) which
     * was compiled with PHYS_OFFSET = 0x40000000.  Match the kernel. */
    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 16 * KiB,         /* BDS says 10 KiB SRAM */

    .use_gic            = true,
    .gic_mpcore_base    = 0x10300000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x10060000,
    .dma_type           = "hisi-regbank",

    /* 2× GbE — fork only models one GMAC; stub the second as regbank */
    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,

    .num_i2c            = 2,
    .i2c_bases          = { 0x120c0000, 0x122e0000 },
    .i2c_type           = "hisi-i2c-dw",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x11010000,
    .sata_irq           = 17,
    .sata_num_ports     = 2,

    .usb_ehci_base      = 0x100c0000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x100b0000,
    .usb_ohci_irq       = 18,

    .xhci_base          = 0x11000000,
    .xhci_irq           = 22,
    .xhci_slots         = 4,
    .xhci_intrs         = 1,

    .num_regbanks       = 7,
    .regbanks           = {
        { "hisi-dmac",       0x10060000, 0x1000  },
        { "hisi-cipher",     0x10070000, 0x2000  },
        { "hisi-gmac1",      0x100b1000, 0x1000  },  /* 2nd GbE stub */
        { "hisi-pcie0",      0x11020000, 0x10000 },
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-ddrc",       0x12110000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3536C / Hi3536CV100 (DVR/NVR family): 2017, dual Cortex-A7 @ 1.3 GHz,
 * H.265.  Entry-level 4K NVR — the "Hi3536-lite".  Distinct die from the
 * Hi3536 flagship (different CPU IP block — A7 dual vs A17 quad+A7) and
 * from Hi3536DV100 (A7 single decode-only).  2× GbE, 2× SATA 3.0, no XHCI.
 */
static const HisiSoCConfig hi3536cv100_soc = {
    .name               = "hi3536cv100",
    .desc               = "HiSilicon Hi3536CV100 (Cortex-A7, dual-core, 4K NVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3536CV100,
    .max_cpus           = 1,
    .default_sensor     = NULL,

    .ram_size_default   = 512 * MiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x80000000,       /* same as Hi3536DV100 */
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12080000, 0x12090000, 0x120a0000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 3000000,

    .num_spis           = 1,
    .spi_bases          = { 0x120d0000 },
    .spi_irqs           = { 14 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .dma_base           = 0x11020000,
    .dma_type           = "hisi-regbank",

    /* Vendor higmacv300 — DT `ethernet@100a0000` in OpenIPC kernel
     * arch/arm/boot/dts/hi3536c.dtsi, second port `ethernet@100a1000`.
     * U-Boot `arch/arm/include/asm/arch-hi3536c/platform.h` defines
     * GSF_REG_BASE = 0x100a0000, HIGMAC1_IOBASE = +0x1000.  Shared GIC
     * SPI 16 between both ports.  Earlier 0x10010000/IRQ 11 was a stale
     * copy from hi3536dv100's femac slot (different controller).
     *
     * Vendor `drivers/net/higmacv300/higmac.h` sets
     * CONFIG_HIGMAC_DESC_4_WORD on hi3536cv100 — 16-byte descriptors. */
    .gmac_base          = 0x100a0000,
    .gmac_irq           = 16,
    .gmac_desc_size     = 16,

    .num_i2c            = 1,
    .i2c_bases          = { 0x120c0000 },
    .i2c_type           = "hisi-i2c",

    .rtc_base           = 0x120b0000,
    .rtc_irq            = 5,

    .sata_base          = 0x10030000,       /* same as Hi3536DV100 */
    .sata_irq           = 17,
    .sata_num_ports     = 2,                /* C-variant has 2 ports */

    .usb_ehci_base      = 0x11010000,
    .usb_ehci_irq       = 19,
    .usb_ohci_base      = 0x11000000,
    .usb_ohci_irq       = 18,

    .num_regbanks       = 5,
    .regbanks           = {
        { "hisi-dmac",       0x11020000, 0x1000  },
        { "hisi-cipher",     0x11030000, 0x10000 },
        { "hisi-gmac1",      0x100a1000, 0x1000  },  /* HIGMAC1 stub
                                                       — DT ethernet@100a1000,
                                                       responds 0 so the
                                                       vendor U-Boot probe
                                                       loop falls through
                                                       to HIGMAC0 (where our
                                                       PHY at addr 1 links
                                                       UP). */
        { "hisi-ir",         0x12140000, 0x10000 },
        { "hisi-misc",       0x12120000, 0x10000 },
    },
};

/*
 * Hi3798CV200 (STB family — first 64-bit ARM SoC in this fork): 2015,
 * Cortex-A53 quad @ ~1.5 GHz, ARMv8-a.  HiSTB platform — TV-box / DVB STB
 * SoC, Linaro Poplar 96Boards reference, **mainline Linux DT supported**
 * (`arch/arm64/boot/dts/hisilicon/hi3798cv200.dtsi`).  This SoC is a
 * different product line from the IPC and DVR/NVR families — no analog
 * VI, no surveillance ISP, no NVP6124B; it has TS demux, HDMI 2.0,
 * Mali-T720 GPU, Dolby Vision support.
 *
 * Memory map from upstream Linux DT.  PSCI/SMC for SMP bringup (no
 * pen-release pattern, no SCU writes — kernel uses `arm,psci-0.2`).
 * Sysctl/CRG layout differs from V1–V5 hisi-sysctl model — leave
 * sysctl_base = crg_base = 0 to skip those instances.
 *
 * Boot-to-shell scope: minimal — only the GICv2-400 (dist + cpu) and
 * a single PL011 UART are wired.  Storage / USB / Mali / TS demux all
 * deferred — guest kernel boots to a usable shell prompt over UART0.
 */
static const HisiSoCConfig hi3798cv200_soc = {
    .name               = "hi3798cv200",
    .desc               = "HiSilicon Hi3798CV200 (Cortex-A53 quad, HiSTB)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a53"),
    .soc_id             = HISI_SOC_ID_3798CV200,
    .max_cpus           = 4,
    .default_sensor     = NULL,
    .board_id           = 0,                /* DT-based boot, ATAGs unused */
    /* QEMU_PSCI_CONDUIT_SMC = 1 in upstream "hw/arm/boot.h"; enables PSCI
     * for SMP bringup so the upstream smpboot stub at 0x0 is suppressed. */
    .psci_conduit       = 1,                /* QEMU_PSCI_CONDUIT_SMC */

    /* DDR base 0x0 from poplar.dts memory@0; default 2 GiB matches Poplar. */
    .ram_size_default   = 2 * GiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x0,
    .sram_base          = 0,                /* skip SRAM region */
    .sram_size          = 0,

    .use_gic            = true,
    /* GIC-400, GICv2 mode for now (skip GICH/GICV virt extensions).
     * dist 0xf1001000/0x1000, cpu 0xf1002000/0x2000 — non-overlapping. */
    .gic_dist_base      = 0xf1001000,
    .gic_cpu_base       = 0xf1002000,
    .gic_num_spi        = 224,              /* GIC-400 supports up to 224 SPIs */

    .sysctl_base        = 0,                /* HiSTB sysctl uses different layout */
    .crg_base           = 0,                /* HiSTB CRG uses different layout */

    /* PL011 UART0 at 0xf8b00000 (soc base 0xf0000000 + 0x8b00000) IRQ 49 */
    .num_uarts          = 1,
    .uart_bases         = { 0xf8b00000 },
    .uart_irqs          = { 49 },

    /* ARM generic timer used for Cortex-A53; no SP804 in DT */
    .num_timers         = 0,

    .num_spis           = 0,
    .fmc_ctrl_base      = 0,                /* skip HiFMC — not present on STB */
    .num_i2c            = 0,
    .num_himci          = 0,
    .num_sdhci          = 0,
};

/*
 * Hi3796M V100 (HiSTB family): 2015, Cortex-A7 quad @ 1.5 GHz, ARMv7-a.
 * DVB-S2/-C/-T2 STB, Mali-450, H.265 dec to 4Kp30.  Phase 4.6 backport:
 * minimal Linux 4.9 board port to RichStrong tree (mach-hi3796mv100,
 * hi3796mv100.dtsi with fixed-rate clocks for UART/timer — no HiSTB
 * clock controller driver yet).
 *
 * Memory map confirmed against vendor `arch/arm/mach-hifone/include/mach/
 * platform.h` (Hi3796M shares the "hifone" mach with Hi3798M and other
 * HiSTB SoCs).  Single-CPU boot; quad SMP requires HiSTB power-mgmt
 * code (mach-hifone/hipm.c) which is not ported.
 */
static const HisiSoCConfig hi3796mv100_soc = {
    .name               = "hi3796mv100",
    .desc               = "HiSilicon Hi3796M V100 (Cortex-A7 quad, HiSTB)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_3796MV100,
    .max_cpus           = 1,
    .default_sensor     = NULL,

    .ram_size_default   = 1 * GiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    /* HiSTB family: DDR base 0x0; uImage load addr 0x00008000 */
    .ram_base           = 0x0,
    .sram_base          = 0,
    .sram_size          = 0,

    .use_gic            = true,
    /* GIC-400 at 0xf1000000 per vendor REG_BASE_GIC_REG; dist @+0x1000,
     * cpu @+0x2000 (8KB window).  Non-overlapping → plain GIC works. */
    .gic_dist_base      = 0xf1001000,
    .gic_cpu_base       = 0xf1002000,
    .gic_num_spi        = 128,

    /* HiSTB sysctrl/CRG layout differs from V1–V5 model — leave 0 to skip */
    .sysctl_base        = 0,
    .crg_base           = 0,

    /* PL011 UART0 at 0xf8b00000 (HiSTB AMBA cluster), GIC SPI 6 per DT */
    .num_uarts          = 1,
    .uart_bases         = { 0xf8b00000 },
    .uart_irqs          = { 6 },

    /* SP804 dual-timer at 0xf8002000 (HiSTB TIMER01), GIC SPI 1.
     * Vendor REG_BASE_TIMER01 = 0xf8002000.  At 50 MHz fixed clock the
     * kernel calibrates BogoMIPS in seconds. */
    .num_timers         = 1,
    .timer_bases        = { 0xf8002000 },
    .timer_irqs         = { 1 },
    .timer_freq         = 50000000,

    .num_spis           = 0,
    .fmc_ctrl_base      = 0,
    .num_i2c            = 0,
};

/*
 * Hi3536 (DVR/NVR family flagship): 2014, **Cortex-A17 quad** @ 1.6 GHz
 * + Cortex-A7 single @ 900 MHz video coprocessor (asymmetric, separate
 * kernel).  H.265 dec / H.264 enc, 2× GbE TOE, 2× SATA 3.0 + 1× PCIe 2.0/
 * SATA 3.0 mux, USB 3.0, 2× SDIO, dual HDMI/VGA, Mali-T720 GPU, IVE 2.0.
 * Uses Linux 3.10 board-config style (no DTB).
 *
 * Memory map and IRQs from vendor kernel arch/arm/mach-hi3536/include/mach/
 * (Hi3536 SDK V2.0.2.0, Linux 3.10.0_hi3536).  Phase-4 boot-to-shell scope:
 * single-CPU on the A17 cluster (max_cpus=1 — quad A17 SMP via the pen-
 * release pattern is a Phase-4.5 follow-up that needs a writable trampoline
 * page at 0x0 in place of the fork's read-only NULL trap).  A7 video
 * coprocessor is a separate machine class entirely — not modelled.
 *
 * QEMU has no cortex-a17 CPU type; cortex-a15 is the closest analogue
 * (same armv7-a ISA, NEON+VFPv4, generic timer, big.LITTLE-class).
 */
static const HisiSoCConfig hi3536_soc = {
    .name               = "hi3536",
    .desc               = "HiSilicon Hi3536 flagship (Cortex-A17 quad, NVR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a15"),
    .soc_id             = HISI_SOC_ID_3536,
    .max_cpus           = 1,                /* A17 SMP deferred */
    .default_sensor     = NULL,
    /* MACH_TYPE_HI3536 from arch/arm/tools/mach-types — required because
     * Hi3536's vendor 3.10 kernel uses MACHINE_START (ATAGs board ID),
     * not DT_MACHINE_START.  Without this, lookup_machine_type fails. */
    .board_id           = 8000,

    /* DDR3/DDR4 32-bit, max 3 GiB.  Vendor U-Boot defaults to mem=96M. */
    .ram_size_default   = 1 * GiB,
    .kernel_mem_mb      = 0,
    .extra_cmdline      = NULL,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    /* A17 mpcore peripheral block at 0x1fff0000 (size 0x10000):
     *   +0x0000 SCU (we leave a regbank stub there)
     *   +0x1000 GIC dist
     *   +0x2000 GIC cpu interface
     * Unlike the A9 layout, dist and cpu are separated by 0x1000 each so
     * plain GIC mapping doesn't overlap.  No a9mpcore_priv path needed. */
    .gic_dist_base      = 0x1fff1000,
    .gic_cpu_base       = 0x1fff2000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12050000,
    .crg_base           = 0x12040000,

    /* Vendor mach-hi3536 only registers UART0 + UART1 as platform devices;
     * UARTs 2/3 exist on silicon but aren't part of the boot path. */
    .num_uarts          = 2,
    .uart_bases         = { 0x12080000, 0x12090000 },
    .uart_irqs          = { 6, 7 },

    /* 4× SP804 dual-timer + 1× extra block at 0x12060000 (timer 8/9).
     * INTNR_TIMER_0=1, _2=2, _4=3, _6=4 per irqs.h.  The 5th block
     * (timer 8/9 @ 0x12060000) shares CRG slot but has its own irq pair
     * (77/78) outside the standard sp804 driver scope. */
    .num_timers         = 4,
    .timer_bases        = { 0x12000000, 0x12010000, 0x12020000, 0x12030000 },
    .timer_irqs         = { 1, 2, 3, 4 },
    .timer_freq         = 12500000,         /* get_bus_clk()/2 ≈ 12.5 MHz */

    .num_spis           = 0,                /* no PL022 in mach amba_devs */

    /* No FMC in mach-hi3536 platform list — kernel doesn't probe SPI NOR
     * during boot.  Set to 0 to skip instantiating hisi-fmc. */
    .fmc_ctrl_base      = 0,

    .num_i2c            = 0,                /* I2C loaded later as module */

    /* Stub regbanks for blocks the kernel touches before module-load.
     * The A17 MPCore SCU at 0x1fff0000 (size 0x1000) sits just below
     * the GIC dist; a regbank lets vendor SMP power-up writes succeed. */
    .num_regbanks       = 1,
    .regbanks           = {
        { "hisi-a17-scu",    0x1fff0000, 0x1000  },
    },
};

/* ── Machine state with sensor + flash-file properties ─────────────── */

/* Extra state appended to MachineState for machine-level properties */
typedef struct {
    MachineState parent_obj;
    char *sensor;
    char *flash_file;
} HisiMachineState;

/* ── Appended DTB fixup ────────────────────────────────────────────── */

/*
 * Fixup appended DTB in the loaded kernel payload.
 *
 * Many OpenIPC uImages have a device tree appended with zero padding
 * and no /chosen node.  The kernel's atags_to_fdt() tries to patch
 * the DTB in-place with bootargs and initrd info; without padding it
 * silently corrupts the DTB and boot fails.  Some DTBs also lack a
 * /chosen { stdout-path } node needed for console output.
 *
 * This runs as a reset handler (after ROM blobs are loaded into RAM)
 * so the kernel payload is already in guest memory.
 */
/*
 * Read the kernel file, find the appended DTB, patch it (add /chosen
 * with stdout-path, add padding for atags_to_fdt), and write a temp
 * file with the patched kernel.  Returns the temp file path (caller
 * frees), or NULL if no patching was needed.
 */
static char *hisilicon_patch_appended_dtb(const char *kernel_filename,
                                           hwaddr load_addr,
                                           const HisiSoCConfig *c)
{
    gsize file_size;
    gchar *buf;
    (void)load_addr; /* reserved for future use */

    if (!kernel_filename ||
        !g_file_get_contents(kernel_filename, &buf, &file_size, NULL)) {
        return NULL;
    }

    /* Skip uImage header if present */
    size_t hdr_off = 0;
    if (file_size > 64 &&
        (uint8_t)buf[0] == 0x27 && (uint8_t)buf[1] == 0x05 &&
        (uint8_t)buf[2] == 0x19 && (uint8_t)buf[3] == 0x56) {
        hdr_off = 64;
    }

    /* Scan payload for FDT magic (take the last match = appended DTB) */
    size_t dtb_off = 0;
    uint32_t dtb_size = 0;
    for (size_t i = hdr_off + 256 * 1024; i + 8 < file_size; i += 4) {
        if ((uint8_t)buf[i]   == 0xd0 && (uint8_t)buf[i+1] == 0x0d &&
            (uint8_t)buf[i+2] == 0xfe && (uint8_t)buf[i+3] == 0xed) {
            uint32_t sz = ((uint8_t)buf[i+4] << 24) |
                          ((uint8_t)buf[i+5] << 16) |
                          ((uint8_t)buf[i+6] << 8)  |
                          (uint8_t)buf[i+7];
            if (sz > 1024 && sz < 256 * 1024 && i + sz <= file_size) {
                dtb_off = i;
                dtb_size = sz;
            }
        }
    }

    if (!dtb_off) {
        g_free(buf);
        return NULL;
    }

    /* Patch the DTB */
    uint32_t new_dtb_size = dtb_size + 4096;
    void *dtb = g_malloc0(new_dtb_size);
    memcpy(dtb, buf + dtb_off, dtb_size);

    if (fdt_open_into(dtb, dtb, new_dtb_size) != 0) {
        g_free(dtb);
        g_free(buf);
        return NULL;
    }

    /* Ensure /chosen with stdout-path exists */
    if (fdt_path_offset(dtb, "/chosen") < 0) {
        fdt_add_subnode(dtb, 0, "chosen");
    }
    int chosen = fdt_path_offset(dtb, "/chosen");
    if (chosen >= 0 && !fdt_getprop(dtb, chosen, "stdout-path", NULL)) {
        fdt_setprop_string(dtb, chosen, "stdout-path",
                           "serial0:115200n8");
    }

    /*
     * If timer_freq is set, patch fixed-clock "clk_apb" to match.
     * Some SoCs (hi3516av100) have SP804 with split clocks (50 MHz + 3 MHz)
     * that QEMU can't model — unify them to avoid timer storms.
     */
    if (c->timer_freq) {
        int soc_node = fdt_path_offset(dtb, "/soc");
        if (soc_node >= 0) {
            int node;
            fdt_for_each_subnode(node, dtb, soc_node) {
                const char *compat = fdt_getprop(dtb, node, "compatible", NULL);
                const char *name = fdt_get_name(dtb, node, NULL);
                if (compat && name &&
                    !strcmp(compat, "fixed-clock") &&
                    !strcmp(name, "clk_apb")) {
                    uint32_t freq = cpu_to_be32(c->timer_freq);
                    fdt_setprop_inplace(dtb, node, "clock-frequency",
                                        &freq, sizeof(freq));
                }
            }
        }
    }

    /*
     * Disable NAND controllers that we don't emulate — their drivers
     * poll hardware status registers that cause hangs on regbank stubs.
     */
    {
        const char *disable_paths[] = {
            "/soc/spi_nand_controller",
            "/soc/nand_controller",
            NULL
        };
        for (int i = 0; disable_paths[i]; i++) {
            int node = fdt_path_offset(dtb, disable_paths[i]);
            if (node >= 0) {
                fdt_setprop_string(dtb, node, "status", "disabled");
            }
        }
    }

    /*
     * Widen an under-sized GICv2 CPU-interface reg window.
     *
     * Some vendor DTBs declare the GIC CPU interface as only 0x100 bytes
     * (e.g. Goke xmorca / GK7206: reg = <0x10301000 0x1000 0x10302000
     * 0x100>).  The Linux GICv2 driver requires the CPU-interface region to
     * be >= SZ_8K (0x2000) or it refuses to probe ("GIC: GICv2 detected, but
     * range too small ... will fail to boot", drivers/irqchip/irq-gic.c),
     * which then wedges every interrupt-driven device (SDHCI, etc.).  Real
     * GIC-400 silicon has a 0x2000 CPU interface; the DTS just undersized it
     * (the physical board's U-Boot env compensates with
     * irqchip.gicv2_force_probe=1).  Rather than require an extra bootarg,
     * bump the size to 0x2000 here so the stock driver probes normally.
     * QEMU only maps 0x100 of CPU-interface MMIO, but the machine sets
     * ignore_memory_transaction_failures, so reads of the unmapped tail
     * return 0 harmlessly — exactly like the known-good Hi3516CV610 DTB,
     * which already declares 0x2000.  Only fires when the window is smaller
     * than 0x2000, so correctly-sized DTBs are left untouched.
     */
    {
        static const char * const gicv2_compat[] = {
            "arm,gic-400", "arm,cortex-a7-gic", "arm,cortex-a15-gic",
            "arm,cortex-a9-gic", "arm,gic-v2", NULL
        };
        for (int i = 0; gicv2_compat[i]; i++) {
            int gic = fdt_node_offset_by_compatible(dtb, -1, gicv2_compat[i]);
            if (gic < 0) {
                continue;
            }
            int len = 0;
            const uint32_t *reg = fdt_getprop(dtb, gic, "reg", &len);
            /* 1/1 address/size cells: <dist_base dist_sz cpu_base cpu_sz> */
            if (reg && len >= 4 * (int)sizeof(uint32_t) &&
                be32_to_cpu(reg[3]) < 0x2000) {
                g_autofree uint32_t *newreg = g_memdup2(reg, len);
                newreg[3] = cpu_to_be32(0x2000);
                fdt_setprop_inplace(dtb, gic, "reg", newreg, len);
            }
        }
    }

    /* Pack then re-expand with padding for kernel's atags_to_fdt() */
    fdt_pack(dtb);
    uint32_t packed = fdt_totalsize(dtb);
    fdt_open_into(dtb, dtb, packed + 4096);
    uint32_t final_dtb_size = fdt_totalsize(dtb);

    /*
     * Build new uImage: original_header + kernel_before_dtb + patched_dtb.
     * For raw (non-uImage) kernels, hdr_off=0 so the header is absent.
     * We must preserve the uImage header so QEMU's load_uimage_as()
     * recognizes the file format and uses the correct load address.
     * The uImage data-size and CRC fields are updated to match.
     */
    size_t new_payload_size = (dtb_off - hdr_off) + final_dtb_size;
    size_t new_file_size = hdr_off + new_payload_size;
    gchar *new_buf = g_malloc(new_file_size);

    /* Copy header (if any) */
    if (hdr_off) {
        memcpy(new_buf, buf, hdr_off);
    }
    /* Copy kernel before DTB */
    memcpy(new_buf + hdr_off, buf + hdr_off, dtb_off - hdr_off);
    /* Copy patched DTB */
    memcpy(new_buf + dtb_off, dtb, final_dtb_size);

    /* Update uImage header if present */
    if (hdr_off == 64) {
        /* data size at offset 12 (big-endian) */
        uint32_t data_size = new_payload_size;
        new_buf[12] = (data_size >> 24) & 0xff;
        new_buf[13] = (data_size >> 16) & 0xff;
        new_buf[14] = (data_size >> 8) & 0xff;
        new_buf[15] = data_size & 0xff;
        /* Zero header CRC for recalculation */
        memset(new_buf + 4, 0, 4);
        /* Recalculate data CRC at offset 24 */
        uint32_t dcrc = crc32(0, (const uint8_t *)new_buf + hdr_off, data_size);
        new_buf[24] = (dcrc >> 24) & 0xff;
        new_buf[25] = (dcrc >> 16) & 0xff;
        new_buf[26] = (dcrc >> 8) & 0xff;
        new_buf[27] = dcrc & 0xff;
        /* Recalculate header CRC */
        uint32_t hcrc = crc32(0, (uint8_t *)new_buf, 64);
        new_buf[4] = (hcrc >> 24) & 0xff;
        new_buf[5] = (hcrc >> 16) & 0xff;
        new_buf[6] = (hcrc >> 8) & 0xff;
        new_buf[7] = hcrc & 0xff;
    }

    /* Write temp file */
    GError *err = NULL;
    char *tmppath = NULL;
    int fd = g_file_open_tmp("hisi-kernel-XXXXXX", &tmppath, &err);
    if (fd < 0) {
        g_free(new_buf);
        g_free(dtb);
        g_free(buf);
        return NULL;
    }
    if (write(fd, new_buf, new_file_size) != (ssize_t)new_file_size) {
        close(fd);
        unlink(tmppath);
        g_free(tmppath);
        g_free(new_buf);
        g_free(dtb);
        g_free(buf);
        return NULL;
    }
    close(fd);

    g_free(new_buf);
    g_free(dtb);
    g_free(buf);
    return tmppath;
}


/* ── Shared machine init ───────────────────────────────────────────── */

/* PPI numbers — same for all GICv2 HiSilicon SoCs */
#define HISI_PPI_HYPTIMER   10
#define HISI_PPI_VIRTTIMER  11
#define HISI_PPI_SECTIMER   13
#define HISI_PPI_PHYSTIMER  14

static struct arm_boot_info hisilicon_binfo;

/*
 * No machine here uses boot.c's vexpress-style secondary pen.  Left at
 * NULL, arm_load_kernel() would install it at smp_loader_start (0x0, as
 * we never set it), clobbering the NULL trap page and the low page the
 * vendor kernel writes its own trampoline into, and park the secondaries
 * on it.  Secondaries instead stay start-powered-off until the SoC's
 * release mechanism (CRG soft-reset bit, PSCI) turns them on.
 */
static void hisilicon_write_secondary_boot(ARMCPU *cpu,
                                           const struct arm_boot_info *info)
{
}

static void hisilicon_reset_secondary(ARMCPU *cpu,
                                      const struct arm_boot_info *info)
{
}

/*
 * CV500/AV300/DV300 CPU1 release, driven by the CRG "cpu1-reset" line
 * (REG_CPU_SRST_CRG bit 2).  Before clearing the bit the kernel has
 * written "ldr pc, [pc, #-4]; .word secondary_startup" at physical 0,
 * so CPU1 just has to leave reset at PC 0.
 *
 * After a firmware boot that is literally a reset: CPU0 runs U-Boot and
 * the kernel in Secure SVC, and so does a freshly reset CPU1.  With
 * -kernel, boot.c enters CPU0 in NS Hyp (or NS SVC without EL2), so
 * start CPU1 at 0 in that same state; a Secure SVC CPU1 next to a Hyp
 * CPU0 makes the kernel report "CPUs started in inconsistent modes" and
 * drop the Hyp stub.
 */
typedef struct HisiCpuRelease {
    ARMCPU *cpu;
    bool kernel_boot;
} HisiCpuRelease;

static void hisilicon_cpu1_reset(void *opaque, int n, int level)
{
    HisiCpuRelease *r = opaque;
    uint64_t mpidr = arm_cpu_mp_affinity(r->cpu);

    if (level) {
        arm_set_cpu_off(mpidr);
    } else if (r->kernel_boot) {
        int el = arm_feature(&r->cpu->env, ARM_FEATURE_EL2) ? 2 : 1;

        arm_set_cpu_on(mpidr, 0, 0, el, false);
    } else {
        arm_set_cpu_on_and_reset(mpidr);
    }
}

/* ARM instruction encoding helpers for boot ROM generation */
static inline uint32_t arm_movw(int rd, uint16_t imm16)
{
    /* MOVW Rd, #imm16 (ARMv7): Rd[15:0] = imm16, Rd[31:16] = 0 */
    return 0xe3000000 | ((imm16 >> 12) << 16) | (rd << 12) | (imm16 & 0xfff);
}

static inline uint32_t arm_movt(int rd, uint16_t imm16)
{
    /* MOVT Rd, #imm16 (ARMv7): Rd[31:16] = imm16, Rd[15:0] unchanged */
    return 0xe3400000 | ((imm16 >> 12) << 16) | (rd << 12) | (imm16 & 0xfff);
}

/*
 * Write a boot ROM at address 0 that emulates the HiSilicon boot ROM:
 * copies U-Boot from the SPI NOR flash memory window to DDR and jumps to it.
 *
 * Real HiSilicon boot ROMs (e.g. Hi3516CV500) do a two-stage load:
 *   1) 24 KB from flash to SRAM (contains DDR init + BSBM header)
 *   2) 512 KB from flash+0x6000 to DDR
 * Since QEMU DDR is already available, we simplify to a single copy
 * of the boot partition (256 KB) from flash window to DDR base.
 */
static void hisilicon_write_bootrom(MemoryRegion *sysmem,
                                     const HisiSoCConfig *c,
                                     MachineState *machine)
{
    /*
     * aarch64 V5 SoCs (Hi3519DV500/Hi3516DV500) don't use the ARMv7/v5
     * flash->DDR copy stub: the gzip self-extracting u-boot-z.bin is placed
     * at its DDR link address and CPU0's reset PC is set there, after the
     * CPUs are created (see the flash_boot branch in hisilicon_common_init).
     */
    if (c->aarch64) {
        return;
    }

    /*
     * V1: hand off to the flash window and let U-Boot run in place.  On
     * silicon the SPI NOR is remapped at 0 and the reset code jumps into
     * the window through _clr_remap_spi_entry; starting at the window
     * skips only that jump.  From there start.S walks its reg_info table,
     * which is what brings the PLLs and DDR up (see hisi-ddrc-v1), and
     * only then copies itself to DDR.  Address 0 stays RAM: relocate
     * copies U-Boot's exception table there.
     */
    if (c->v1_xip_boot) {
        uint32_t v1rom[] = {
            cpu_to_le32(0xea000006),    /* b reset_handler (word 8)    */
            cpu_to_le32(0xe1b0f00e),    /* movs pc, lr  (undef: skip)  */
            cpu_to_le32(0xeafffffe),    /* b .  (SWI)                  */
            cpu_to_le32(0xeafffffe),    /* b .  (prefetch abort)       */
            cpu_to_le32(0xe25ef004),    /* subs pc, lr, #4 (dabort)    */
            cpu_to_le32(0xeafffffe),    /* b .  (reserved)             */
            cpu_to_le32(0xeafffffe),    /* b .  (IRQ)                  */
            cpu_to_le32(0xeafffffe),    /* b .  (FIQ)                  */
            cpu_to_le32(0xe321f0d3),    /* msr cpsr_c, #0xD3           */
            cpu_to_le32(0xe51ff004),    /* ldr pc, [pc, #-4]           */
            cpu_to_le32(c->fmc_mem_base),
        };
        MemoryRegion *v1bootrom = g_new(MemoryRegion, 1);

        memory_region_init_ram(v1bootrom, NULL, "hisilicon.bootrom",
                               0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, v1bootrom);
        rom_add_blob_fixed("hisilicon.bootrom", v1rom, sizeof(v1rom), 0);
        return;
    }

    hwaddr flash_src = c->fmc_mem_base;         /* e.g. 0x14000000 */
    hwaddr ram_dst   = c->ram_base;             /* e.g. 0x40000000 */
    hwaddr uboot_entry = 0;   /* jump target; defaults to ram_dst (set below) */
    /*
     * Boot partition copy size.  Vendor U-Boot binaries vary widely:
     * Hi3516CV100 ≈ 130 KB, Hi3516EV200 ≈ 230 KB, Goke V4 ≈ 515 KB.
     * Real silicon's boot ROM parses a header to learn the exact size;
     * we don't model that — copy 1 MiB which is enough to cover every
     * vendor image we've seen, padded with whatever the surrounding
     * flash holds.  Goke V4 silently failed when this was 256 KiB:
     * relocation tables sit past 256 KiB in the 515 KiB binary, so
     * post-relocation U-Boot jumped into garbage and produced no
     * UART output.
     */
    uint32_t copy_sz = 0x100000;                /* 1 MiB boot partition */
    bool armv7 = c->use_gic;  /* ARM926 SoCs use VIC, Cortex-A7+ use GIC */

    /*
     * Detect U-Boot TEXT_BASE from the exception vector table in flash.
     * If vector[1] (undefined instruction handler) is "ldr pc, [pc, #off]",
     * the literal it loads is an absolute address revealing where U-Boot
     * expects to be loaded.  Position-independent U-Boots (e.g. EV300) use
     * infinite loops instead, in which case we fall back to ram_base.
     */
    uint32_t vec1_insn = address_space_ldl(&address_space_memory,
                            flash_src + 4, MEMTXATTRS_UNSPECIFIED, NULL);
    if ((vec1_insn & 0xFFFFF000) == 0xe59FF000) {
        /* ldr pc, [pc, #imm12] */
        uint32_t pool_off = vec1_insn & 0xFFF;
        uint32_t abs_handler = address_space_ldl(&address_space_memory,
                                flash_src + 4 + 8 + pool_off,
                                MEMTXATTRS_UNSPECIFIED, NULL);
        /* TEXT_BASE = handler rounded down to 64K boundary */
        hwaddr text_base = abs_handler & ~0xFFFF;
        if (text_base >= c->ram_base) {
            ram_dst = text_base;
        }
    }

    /*
     * xm720 / Goke-V500 (gk7205v500/v510/v530) multi-section vendor images
     * carry no vector table at flash offset 0 — the first 64 bytes are zero
     * and the first-stage loader sits deeper in the image, prefixed by its
     * own ARM vector table and a run of 8 0xdeadbeef marker words that is
     * immediately followed by the loader's absolute load address (e.g.
     * 0x40707000).  Real silicon's boot ROM parses this layout; mirror it
     * here.  When flash offset 0 is not a reset branch (so the naive
     * copy-to-ram_base / jump-to-0 path would just execute zeros), scan for
     * the marker run, take the load address that follows it, and copy/jump
     * from the loader's vector table 8 words (32 bytes) before the markers.
     *
     * Read the flash image from the file directly: at machine-init time the
     * FMC memory window is not yet readable through the address space (the
     * copy loop only reads the real flash at guest runtime), so we inspect the
     * backing file to learn the layout.
     */
    HisiMachineState *hms = (HisiMachineState *)machine;
    if (hms->flash_file && hms->flash_file[0]) {
        char *data = NULL;
        gsize len = 0;
        if (g_file_get_contents(hms->flash_file, &data, &len, NULL)) {
            const uint32_t *w = (const uint32_t *)data;
            size_t nwords = len / 4;
            /*
             * XM/Xiongmai V4 "reg-config" boot header (e.g. gk7201v200):
             * flash offset 0 is three `b .` reset stubs (0xeafffffe) + an
             * 8-word 0xdeadbeef magic run + a CRG (0x1201xxxx) register-init
             * table — not a reset branch and not a load descriptor.  The real
             * u-boot load descriptor sits *after* the table as a short
             * `…<entry> … 0xdeadbeef 0xdeadbeef <load_base> …` block, with the
             * DRAM entry word two slots ahead of the marker run and the u-boot
             * image (a 0x40-byte header then code) following it.  The generic
             * scan below misses this: the leading 8-word run is followed by a
             * CRG register (not a DRAM addr), and this descriptor's own run is
             * only 2 words.  Detect the signature and resolve it explicitly.
             */
            bool xm_handled = false;
            if (nwords > 16 &&
                le32_to_cpu(w[0]) == 0xeafffffe &&
                le32_to_cpu(w[3]) == 0xdeadbeef &&
                (le32_to_cpu(w[11]) & 0xffff0000) == 0x12010000) {
                size_t xscan = MIN(nwords, (size_t)(0x8000 / 4));
                for (size_t r = 12; r + 3 < xscan; r++) {
                    if (le32_to_cpu(w[r]) != 0xdeadbeef) {
                        continue;
                    }
                    size_t run = 0;
                    while (r + run < xscan &&
                           le32_to_cpu(w[r + run]) == 0xdeadbeef) {
                        run++;
                    }
                    /*
                     * The load descriptor is the z-stage start.S literal pool:
                     * a `.balignl 16,0xdeadbeef` gap followed by _TEXT_BASE,
                     * _clr_remap_fmc_entry (a pointer into the FMC NOR window)
                     * and _start_armboot.  Require that exact shape so we don't
                     * latch onto an incidental 0xdeadbeef word in the reg table.
                     */
                    if (r + run + 3 >= xscan) {
                        break;      /* descriptor + code would run off the image */
                    }
                    uint32_t text_base = le32_to_cpu(w[r + run]);
                    uint32_t fmc_entry = le32_to_cpu(w[r + run + 1]);
                    uint32_t armboot   = le32_to_cpu(w[r + run + 2]);
                    hwaddr reset_off   = (hwaddr)(r + run + 3) * 4;
                    /* The whole copy (and so the entry inside it) must land in
                     * the RAM this machine actually maps. */
                    const hwaddr xm_copy_sz = 0x40000;   /* boot partition */
                    if ((text_base & 0xffff) == 0 &&
                        text_base >= c->ram_base &&
                        machine->ram_size >= xm_copy_sz &&
                        text_base - c->ram_base <=
                            machine->ram_size - xm_copy_sz &&
                        reset_off < xm_copy_sz &&
                        fmc_entry >= c->fmc_mem_base &&
                        (fmc_entry - c->fmc_mem_base) < 0x1000000 &&
                        armboot >= c->ram_base &&
                        armboot - c->ram_base < machine->ram_size) {
                        /*
                         * QEMU's FMC NOR window is MMIO (not executable), so the
                         * z-stage cannot run execute-in-place from flash as it
                         * does under the mask ROM.  Copy the boot partition from
                         * flash 0 to _TEXT_BASE (the u-boot.lds link base,
                         * TEXT_BASE_ORI) and enter the `reset` handler, which
                         * follows the 3-word descriptor pool (_TEXT_BASE,
                         * _clr_remap_fmc_entry, _start_armboot); the factory
                         * `_start` word at flash 0 is a dead `b .`.  fmc_entry is
                         * only used as part of the signature.
                         */
                        flash_src   = c->fmc_mem_base;        /* flash 0       */
                        ram_dst     = text_base;              /* TEXT_BASE_ORI */
                        uboot_entry = text_base + reset_off;  /* reset handler */
                        copy_sz     = xm_copy_sz;
                        xm_handled  = true;
                        break;
                    }
                    r += run;
                }
            }
            /*
             * The loader self-descriptor is a run of >= 4 0xdeadbeef marker
             * words immediately followed by the loader's absolute load
             * address.  The loader block (its ARM reset-vector word) begins
             * 0x40 bytes — 16 words of vector table + markers — before that
             * address word, however the run is split:
             *   xm720/V500  : 8 vectors + 8 markers, block @0x7000 -> 0x40707000
             *   cv6xx raw   : 8 vectors + 8 markers, block @0x0000 -> 0x41700000
             *   cv6xx signed: same descriptor, but the u-boot block sits behind
             *                 the GSL/DDR header so the block opens @0x9400; the
             *                 header also contains a spurious 0xdeadbeef run, so
             *                 the scan must keep going until a run passes sanity.
             * The run length (>= 4) distinguishes these from older V4 images,
             * which use a 2-marker descriptor in a different layout and boot
             * correctly from ram_base via a position-independent reset branch —
             * those are left untouched.
             */
            const size_t MIN_RUN = 4;
            size_t scan = MIN(nwords, (size_t)(0x40000 / 4));  /* first 256 KiB */
            for (size_t i = 0; !xm_handled && i + MIN_RUN <= scan; i++) {
                if (le32_to_cpu(w[i]) != 0xdeadbeef) {
                    continue;
                }
                size_t run = 0;
                while (i + run < scan && le32_to_cpu(w[i + run]) == 0xdeadbeef) {
                    run++;
                }
                if (run < MIN_RUN) {
                    i += run;             /* skip this short run */
                    continue;
                }
                size_t la_idx = i + run;  /* load-address word follows the run */
                if (la_idx < 16 || la_idx >= scan) {
                    i += run;             /* run too near the edges — keep scanning */
                    continue;
                }
                uint32_t load_addr = le32_to_cpu(w[la_idx]);
                size_t blk_off = (la_idx - 16) * 4; /* block opens 0x40 earlier */
                uint32_t blk_vec = le32_to_cpu(w[la_idx - 16]);
                /* Sanity: load address lands in DRAM and the block opens with
                 * a reset branch (b <imm24>). */
                if ((load_addr & ~0xFFFFu) >= c->ram_base &&
                    (load_addr - c->ram_base) < 0x20000000 &&
                    (blk_vec & 0xFF000000) == 0xEA000000) {
                    flash_src += blk_off;
                    ram_dst = load_addr;
                    break;                /* real loader descriptor found */
                }
                /*
                 * A 0xdeadbeef run that fails the sanity check is not the
                 * loader descriptor — e.g. the GSL-signed cv6xx boot image
                 * carries a spurious 6-word run inside its DDR-init blob (~0xc00)
                 * ahead of the real u-boot descriptor at 0x9420.  Skip it and
                 * keep scanning instead of giving up (which left the stub copying
                 * the GSL header at offset 0 to DRAM and hanging).
                 */
                i += run;
            }
            g_free(data);
        }
    }

    /* Jump target defaults to the copy destination unless a descriptor set a
     * distinct entry (e.g. the XM reg-config header, whose image carries a
     * 0x40-byte header ahead of its entry). */
    if (!uboot_entry) {
        uboot_entry = ram_dst;
    }

    /*
     * Build a small boot ROM that copies U-Boot from the SPI NOR flash
     * memory window to DDR and jumps to it.
     *
     * ARMv7 (Cortex-A7) uses movw/movt for 32-bit immediates.
     * ARMv5 (ARM926) uses PC-relative ldr from a literal pool appended
     * after the code (it lacks movw/movt instructions).
     */
    uint32_t rom[32];
    int n = 0;

    /* Set SVC mode, disable IRQ/FIQ */
    rom[n++] = cpu_to_le32(0xe321f0d3);        /* msr cpsr_c, #0xD3      */

    if (armv7) {
        /* ARMv7: use movw/movt for 32-bit immediates */
        rom[n++] = cpu_to_le32(arm_movw(0, flash_src & 0xffff));
        rom[n++] = cpu_to_le32(arm_movt(0, flash_src >> 16));
        rom[n++] = cpu_to_le32(arm_movw(1, ram_dst & 0xffff));
        rom[n++] = cpu_to_le32(arm_movt(1, ram_dst >> 16));
        rom[n++] = cpu_to_le32(arm_movw(2, copy_sz & 0xffff));
        rom[n++] = cpu_to_le32(arm_movt(2, copy_sz >> 16));

        /* r3 = end pointer */
        rom[n++] = cpu_to_le32(0xe0813002);     /* add r3, r1, r2         */

        /* copy_loop: */
        rom[n++] = cpu_to_le32(0xe4904004);     /* ldr r4, [r0], #4       */
        rom[n++] = cpu_to_le32(0xe4814004);     /* str r4, [r1], #4       */
        rom[n++] = cpu_to_le32(0xe1510003);     /* cmp r1, r3             */
        rom[n++] = cpu_to_le32(0x1afffffb);     /* bne copy_loop          */

        /* Jump to U-Boot entry (== ram_dst unless a descriptor set one) */
        rom[n++] = cpu_to_le32(arm_movw(1, uboot_entry & 0xffff));
        rom[n++] = cpu_to_le32(arm_movt(1, uboot_entry >> 16));
        rom[n++] = cpu_to_le32(0xe12fff11);     /* bx r1                  */
    } else {
        /*
         * ARMv5 (ARM926): no movw/movt, no VBAR.  Exception vectors are
         * always at address 0, so the boot ROM must provide a proper
         * vector table.  QEMU's ARM926 model raises undefined-instruction
         * exceptions for some cp15 registers that real silicon silently
         * ignores (e.g. L2 cache aux control), so the undefined handler
         * must skip the faulting instruction instead of looping.
         *
         * Layout:
         *   [0]  b reset_handler      ; reset vector  -> word 8
         *   [1]  movs pc, lr          ; undefined insn -> skip & continue
         *   [2]  b .                  ; SWI            -> hang
         *   [3]  b .                  ; prefetch abort -> hang
         *   [4]  subs pc, lr, #4      ; data abort     -> retry
         *   [5]  b .                  ; reserved
         *   [6]  b .                  ; IRQ            -> hang
         *   [7]  b .                  ; FIQ            -> hang
         *   [8]  msr cpsr_c, #0xD3   ; reset_handler
         *   [9]  ldr r0, [pc, #32]   -> pool[0] = flash_src
         *  [10]  ldr r1, [pc, #32]   -> pool[1] = ram_dst
         *  [11]  ldr r2, [pc, #32]   -> pool[2] = copy_sz
         *  [12]  add r3, r1, r2
         *  [13]  ldr r4, [r0], #4    (copy_loop)
         *  [14]  str r4, [r1], #4
         *  [15]  cmp r1, r3
         *  [16]  bne copy_loop
         *  [17]  ldr r1, [pc, #4]    -> pool[1] = ram_dst
         *  [18]  bx r1
         *  [19]  pool[0] = flash_src
         *  [20]  pool[1] = ram_dst
         *  [21]  pool[2] = copy_sz
         */
        n = 0;
        rom[n++] = cpu_to_le32(0xea000006);     /* b reset_handler (word 8) */
        rom[n++] = cpu_to_le32(0xe1b0f00e);     /* movs pc, lr              */
        rom[n++] = cpu_to_le32(0xeafffffe);     /* b .  (SWI)               */
        rom[n++] = cpu_to_le32(0xeafffffe);     /* b .  (prefetch abort)    */
        rom[n++] = cpu_to_le32(0xe25ef004);     /* subs pc, lr, #4 (dabort) */
        rom[n++] = cpu_to_le32(0xeafffffe);     /* b .  (reserved)          */
        rom[n++] = cpu_to_le32(0xeafffffe);     /* b .  (IRQ)               */
        rom[n++] = cpu_to_le32(0xeafffffe);     /* b .  (FIQ)               */

        /* reset_handler at word 8 */
        assert(n == 8);
        rom[n++] = cpu_to_le32(0xe321f0d3);     /* msr cpsr_c, #0xD3       */

        /* PC-relative offset = pool_addr - (insn_addr + 8)
         * For insn[i] -> pool at word p: offset = (p - i - 2) * 4 */
        rom[n++] = cpu_to_le32(0xe59f0000 | 32); /* ldr r0, [pc, #32]      */
        rom[n++] = cpu_to_le32(0xe59f1000 | 32); /* ldr r1, [pc, #32]      */
        rom[n++] = cpu_to_le32(0xe59f2000 | 32); /* ldr r2, [pc, #32]      */

        rom[n++] = cpu_to_le32(0xe0813002);     /* add r3, r1, r2           */
        rom[n++] = cpu_to_le32(0xe4904004);     /* ldr r4, [r0], #4         */
        rom[n++] = cpu_to_le32(0xe4814004);     /* str r4, [r1], #4         */
        rom[n++] = cpu_to_le32(0xe1510003);     /* cmp r1, r3               */
        rom[n++] = cpu_to_le32(0x1afffffb);     /* bne copy_loop            */

        rom[n++] = cpu_to_le32(0xe59f1000 | 4); /* ldr r1, [pc, #4]        */
        rom[n++] = cpu_to_le32(0xe12fff11);     /* bx r1                    */

        /* Literal pool at word 19 */
        assert(n == 19);
        rom[n++] = cpu_to_le32(flash_src);
        rom[n++] = cpu_to_le32(ram_dst);
        rom[n++] = cpu_to_le32(copy_sz);
    }

    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    if (armv7 && !c->cpu_srst_offset) {
        /* Cortex-A7+ uses VBAR for exception vectors, so address 0 can be
         * ROM.  Firmware with NULL pointer bugs that write to address 0
         * will have writes silently dropped — same as real silicon.
         * SoCs that release CPU1 through the CRG (cpu_srst_offset) need
         * it writable: the kernel puts CPU1's reset trampoline there. */
        memory_region_init_rom(bootrom, NULL, "hisilicon.bootrom",
                               0x1000, &error_fatal);
    } else {
        /* ARM926 has no VBAR — exception vectors are always at address 0.
         * U-Boot must overwrite the boot ROM vectors with its own handlers
         * (e.g. for undefined instruction).  Use RAM so the vectors are
         * writable, matching real hardware where address 0 is internal
         * SRAM that becomes writable after boot ROM handoff. */
        memory_region_init_ram(bootrom, NULL, "hisilicon.bootrom",
                               0x1000, &error_fatal);
    }
    memory_region_add_subregion(sysmem, 0, bootrom);
    /* A ROM blob, so every system reset reinstalls the boot ROM.  On
     * ARM926 it is RAM and U-Boot has overwritten it with its vectors;
     * the reboot must start from the boot ROM again, not from those. */
    rom_add_blob_fixed("hisilicon.bootrom", rom, sizeof(rom), 0);
}

/*
 * Mask-ROM emulation path.  `-bios <file.elf>` loads a reverse-engineered
 * (or otherwise reconstructed) bootrom image at the SoC's mask-ROM base
 * and starts the CPU there — replacing both the synthetic flash→DDR
 * stub built by hisilicon_write_bootrom() and the hisi-fastboot
 * substitute that bypasses the bootrom on -kernel-less runs.
 *
 * The window is fixed at 0x04000000 / 64 KiB to match every V4-family
 * mask-ROM dumped to date (av300, cv500, ev200, ev300, …).  Earlier
 * generations (V1–V3) use different mask-ROM bases and aren't covered
 * here; the function gates on c->sram_base == 0x04010000 so it's a
 * no-op for those.
 *
 * An external host (defib, or a python test harness) drives UART0
 * directly to feed the bootrom's CRC-framed download protocol — the
 * bootrom does its own framing, so no UART-side glue is needed in
 * QEMU.
 */
#define HISI_MASKROM_BASE  0x04000000
#define HISI_MASKROM_SIZE  (64 * KiB)

typedef struct {
    ARMCPU  *cpu;
    uint64_t entry;
    MemoryRegion *alias0;       /* reset-time mask-ROM alias at 0, or NULL */
} HisiMaskromReset;

/*
 * Restart CPU0 at @entry on every system reset.  Shared by the -bios
 * mask-ROM path (which also re-enables its alias at 0) and the flash-boot
 * paths, which have no mask-ROM alias.
 */
static void hisilicon_maskrom_cpu_reset(void *opaque)
{
    HisiMaskromReset *info = opaque;
    CPUState *cs = CPU(info->cpu);

    if (info->alias0) {
        memory_region_set_enabled(info->alias0, true);
    }
    cpu_reset(cs);
    cpu_set_pc(cs, info->entry);
}

/*
 * SC_CTRL bit 8 ("clear boot remap"), driven by hisi-sysctl.  The
 * mask-ROM sets it in its last step before jumping to the loaded image;
 * from then on address 0 is memory again.  The SoCs that release CPU1
 * through the CRG depend on that: the kernel writes CPU1's reset
 * trampoline at physical 0.  One-way until the next system reset, so a
 * later SC_CTRL write that happens to leave bit 8 clear cannot bring the
 * ROM back over the trampoline.
 */
static void hisilicon_maskrom_remap(void *opaque, int n, int level)
{
    HisiMaskromReset *info = opaque;

    if (level) {
        memory_region_set_enabled(info->alias0, false);
    }
}

static HisiMaskromReset *hisilicon_load_maskrom(MemoryRegion *sysmem,
                                    const HisiSoCConfig *c,
                                    MachineState *machine,
                                    ARMCPU *cpu0)
{
    MemoryRegion *rom;
    uint64_t entry, low, high;
    ssize_t loaded;
    HisiMaskromReset *info;
    MemoryRegion *alias;

    if (c->sram_base != 0x04010000) {
        error_report("hisilicon: -bios mask-ROM path is only wired up for "
                     "V4-family SoCs (sram_base=0x04010000); this machine "
                     "uses sram_base=0x%" HWADDR_PRIx, c->sram_base);
        exit(1);
    }

    rom = g_new(MemoryRegion, 1);
    memory_region_init_rom(rom, NULL, "hisilicon.maskrom",
                           HISI_MASKROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, HISI_MASKROM_BASE, rom);

    /*
     * Real silicon aliases the mask-ROM at low address 0 on reset.
     * The bootrom relies on this: media_program_a stores
     * `0x00004680` as the SD-read function pointer and later calls it
     * via `bx`, expecting to hit the bootrom's read-block routine.
     * Without the alias, qemu jumps to whatever is mapped at 0x4680
     * (the trapnull stub or unmapped memory) and the bootrom crashes
     * before ever issuing CMD17.
     *
     * The alias overrides the `hisilicon.trapnull` ROM that the
     * non-bios paths install at 0 (which we still install for
     * compatibility, then this alias takes precedence in the
     * memory region priority).
     */
    alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(alias, NULL, "hisilicon.maskrom-alias-0",
                             rom, 0, HISI_MASKROM_SIZE);
    memory_region_add_subregion_overlap(sysmem, 0, alias, 1);

    loaded = load_elf(machine->firmware, NULL, NULL, NULL,
                      &entry, &low, &high, NULL,
                      0 /* little-endian */, EM_ARM,
                      1 /* clear_lsb */, 0 /* data_swab */);
    if (loaded < 0) {
        error_report("hisilicon: failed to load -bios '%s': %s",
                     machine->firmware, load_elf_strerror(loaded));
        exit(1);
    }
    if (low < HISI_MASKROM_BASE ||
        high > HISI_MASKROM_BASE + HISI_MASKROM_SIZE) {
        error_report("hisilicon: -bios '%s' load range "
                     "[0x%" PRIx64 "..0x%" PRIx64 "] outside mask-ROM "
                     "window [0x%08x..0x%08x]",
                     machine->firmware, low, high,
                     (unsigned)HISI_MASKROM_BASE,
                     (unsigned)(HISI_MASKROM_BASE + HISI_MASKROM_SIZE));
        exit(1);
    }

    info = g_new0(HisiMaskromReset, 1);
    info->cpu = cpu0;
    info->entry = entry;
    info->alias0 = alias;
    qemu_register_reset(hisilicon_maskrom_cpu_reset, info);
    return info;
}

/*
 * Instantiate a PL061 GPIO bank at @base.  When stride >= 0x10000 the DTB
 * declares a 64 KiB reg window and Linux's AMBA bus probe reads PrimeCell
 * IDs at resource_end - 0x20 (offset 0xFFE0).  Alias the 0x1000 register
 * page at offset 0xF000 so IDs at 0xFE0..0xFFF also appear at 0xFFE0..0xFFFF.
 */
static DeviceState *hisilicon_create_pl061(MemoryRegion *sysmem, hwaddr base,
                                           int stride, qemu_irq irq)
{
    DeviceState *pl061 = qdev_new("pl061");
    SysBusDevice *sbd = SYS_BUS_DEVICE(pl061);

    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, irq);

    if (stride >= 0x10000) {
        MemoryRegion *pl061_mr = sysbus_mmio_get_region(sbd, 0);
        MemoryRegion *alias = g_new0(MemoryRegion, 1);

        memory_region_init_alias(alias, OBJECT(pl061),
                                 "pl061-primecell-id-alias",
                                 pl061_mr, 0, 0x1000);
        memory_region_add_subregion(sysmem, base + 0xF000, alias);
    }

    return pl061;
}

/* Re-assert "pulled-up" GPIO inputs after every system reset (pl061_reset
 * zeroes the data register, so a one-time qemu_set_irq at init wouldn't
 * survive).  Models board pull-ups the vendor firmware samples at boot. */
typedef struct {
    DeviceState *dev;
    uint32_t     mask;
} HisiGpioPull;

static void hisilicon_gpio_pull_reset(void *opaque)
{
    HisiGpioPull *p = opaque;
    int b;
    for (b = 0; b < 8; b++) {
        if (p->mask & (1u << b)) {
            qemu_set_irq(qdev_get_gpio_in(p->dev, b), 1);
        }
    }
}

/* PL011 UARTCR = UARTEN | TXE | RXE: stamps every PL011 in the SoC config
 * after device-level reset, which would otherwise zero UARTEN.  Lets vendor
 * Linux 3.0/3.4/3.10 kernels print to console without a U-Boot stage that
 * normally programs UARTCR before kernel handoff. */
typedef struct {
    Notifier notifier;
    const HisiSoCConfig *c;
} HisiUartPreEnableNotifier;

static void hisi_uart_pre_enable_done(Notifier *n, void *opaque)
{
    HisiUartPreEnableNotifier *nf = container_of(n, HisiUartPreEnableNotifier,
                                                 notifier);
    const HisiSoCConfig *c = nf->c;
    int i;

    for (i = 0; i < c->num_uarts; i++) {
        address_space_stl(&address_space_memory,
                          c->uart_bases[i] + 0x30,
                          0x301,    /* UARTCR = UARTEN | TXE | RXE */
                          MEMTXATTRS_UNSPECIFIED, NULL);
    }
}

static void hisi_register_uart_pre_enable_reset(const HisiSoCConfig *c)
{
    HisiUartPreEnableNotifier *nf = g_new0(HisiUartPreEnableNotifier, 1);
    nf->notifier.notify = hisi_uart_pre_enable_done;
    nf->c = c;
    qemu_add_machine_init_done_notifier(&nf->notifier);
}

static void hisilicon_common_init(MachineState *machine,
                                  const HisiSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    int smp_cpus = machine->smp.cpus;
#define HISI_MAX_SMP 4
    Object *cpuobj[HISI_MAX_SMP];
    ARMCPU *cpu[HISI_MAX_SMP];
    qemu_irq pic[256];
    int num_pic = 0;
    int n;
    SysBusDevice *fmc_irq_dev = NULL; /* FMC, IRQ-connected after pic[] is built */
    bool flash_boot = false;  /* true when booting from SPI NOR flash dump */
    bool fmc_nand_boot = false; /* true when the FMC boot flash is SPI NAND */
    bool bios_boot = machine->firmware && machine->firmware[0];
    HisiMaskromReset *maskrom = NULL;
                                /* true when -bios loads a mask-ROM ELF */

    /* SRAM (skipped on STB family which has no on-chip SRAM in DT) */
    if (c->sram_size) {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "hisilicon.sram",
                               c->sram_size, &error_fatal);
        memory_region_add_subregion(sysmem, c->sram_base, sram);
    }

    /* Flash controller (HiFMC V100 or HISFC350) */
    if (c->fmc_ctrl_base) {
        const char *fmc_type = c->fmc_type ? c->fmc_type : "hisi-fmc";
        DeviceState *fmc = qdev_new(fmc_type);
        SysBusDevice *fmcbus = SYS_BUS_DEVICE(fmc);

        /* Select the register-map variant (0 = HiFMC V100, 1 = fmc100 for
         * Goke xmorca / GK7206).  Only the hisi-fmc model has this property. */
        if (c->fmc_variant && (!c->fmc_type || !strcmp(fmc_type, "hisi-fmc"))) {
            qdev_prop_set_uint32(fmc, "variant", c->fmc_variant);
        }

        /*
         * Forward the machine-level "flash-file" property to whichever
         * flash controller the SoC instantiates (hisi-fmc or hisi-sfc350).
         * Lets consumers say `-machine $M,flash-file=...` (or `-global
         * hisi-arm.flash-file=...`) without knowing which controller the
         * machine uses.  See openhisilicon#80.
         *
         * The device-level globals `-global hisi-fmc.flash-file=...` and
         * `-global hisi-sfc350.flash-file=...` still work too — they're
         * applied by the qdev realize machinery and only match when the
         * SoC actually instantiates the named device, so they don't
         * conflict with this forwarding.
         */
        HisiMachineState *hms = (HisiMachineState *)machine;
        if (hms->flash_file && hms->flash_file[0]) {
            qdev_prop_set_string(fmc, "flash-file", hms->flash_file);

            /*
             * Auto-select SPI-NAND mode by image size.  The hisi-fmc model
             * backs SPI NOR up to 16 MiB; anything larger is a SPI-NAND dump
             * (e.g. the 128 MiB GD5F1GM7 on Goke-V500 NAND boards).  Setting
             * flash-type=1 before realize makes the FMC allocate the NAND
             * array + OOB and report the GigaDevice ID, so U-Boot/Linux can
             * attach UBI and flash-boot straight from the dump.  Only the
             * hisi-fmc controller has a flash-type property (hisi-sfc350 is
             * NOR-only), so gate on the device type.  See widgetii#115.
             */
            if (!c->fmc_type || !strcmp(fmc_type, "hisi-fmc")) {
                GStatBuf st;
                if (g_stat(hms->flash_file, &st) == 0 &&
                    st.st_size > 16 * 1024 * 1024) {
                    qdev_prop_set_uint32(fmc, "flash-type", 1 /* SPI NAND */);
                    fmc_nand_boot = true;
                }
            }
        }

        sysbus_realize_and_unref(fmcbus, &error_fatal);
        sysbus_mmio_map(fmcbus, 0, c->fmc_ctrl_base);
        sysbus_mmio_map(fmcbus, 1, c->fmc_mem_base);
        /* The fmc100 (lotus,fmc) driver waits on an interrupt for DMA-read
         * completion.  The IRQ can only be connected once the GIC has built
         * pic[] (below), so stash the device and wire it up after that. */
        fmc_irq_dev = fmcbus;

        /*
         * Dual NOR+NAND boards (e.g. OpenIPC hi3516ev300 NAND: u-boot+env in
         * SPI-NOR, rootfs in SPI-NAND) pass `-global hisi-fmc.nand-file=...`
         * (applied during realize, so this is queried afterwards).  Their boot
         * flash device type is SPI-NAND, so report that in SYSSTAT too — after
         * the NOR scan U-Boot's dev_type_switch(DEFAULT) returns the FMC to
         * NAND mode, letting hifmc100_host_init succeed.
         */
        if (!c->fmc_type || !strcmp(fmc_type, "hisi-fmc")) {
            char *nf = object_property_get_str(OBJECT(fmc), "nand-file", NULL);
            if (nf && nf[0]) {
                fmc_nand_boot = true;
            }
            g_free(nf);
        }

        /* Check if flash-file was set (via either the machine-level
         * property or a device-level -global). */
        if (!machine->kernel_filename) {
            char *ff = object_property_get_str(OBJECT(fmc), "flash-file", NULL);
            if (ff && ff[0]) {
                flash_boot = true;
            }
            g_free(ff);
        }
    }

    /*
     * Address 0 page — either a boot ROM (for flash boot) or a trap page
     * (for normal -kernel boot where NULL function pointers must be safe).
     */
    if (flash_boot) {
        hisilicon_write_bootrom(sysmem, c, machine);
    } else if (c->ram_base != 0) {
        /* NULL trap page: makes NULL function pointer calls return cleanly
         * for V1–V5 IPC kernels.  Skip when RAM lives at 0 (e.g. STB family
         * Hi3798CV200) — the trap would conflict with the system memory. */
        MemoryRegion *trap = g_new(MemoryRegion, 1);
        if (c->cpu_srst_offset || !c->use_gic) {
            /* RAM, not ROM: the kernel's SMP bringup writes CPU1's reset
             * trampoline here (see hisilicon_cpu1_reset), and the V1
             * 3.0.8 kernel's arch_reset() copies its tcm_slow_reset stub
             * to physical 0x100 and jumps to it (mach-hi3518/tcm_reset.S);
             * with ROM the copy is dropped and the restart prefetch-aborts
             * forever instead of reaching SC_SYSRES. */
            memory_region_init_ram(trap, NULL, "hisilicon.trapnull",
                                   0x1000, &error_fatal);
        } else {
            memory_region_init_rom(trap, NULL, "hisilicon.trapnull",
                                   0x1000, &error_fatal);
        }
        memory_region_add_subregion(sysmem, 0, trap);
        uint32_t insn[2] = { cpu_to_le32(0xe3a00000),   /* mov r0, #0 */
                             cpu_to_le32(0xe12fff1e) };  /* bx lr      */
        /* A ROM blob, so every system reset rewrites it over whatever the
         * previous kernel left in the RAM variant. */
        rom_add_blob_fixed("hisilicon.trapnull", insn, sizeof(insn), 0);
    }

    /* RAM */
    memory_region_add_subregion(sysmem, c->ram_base, machine->ram);

    /*
     * V1 DDR controller + PHY.  On a flash boot DDR stays unusable until
     * U-Boot's reg_info table starts the PHY, so an image built without
     * the table fails here as it does on a board (OpenIPC/firmware#2593).
     * The PHY window sits inside the hisi-misc regbank, hence priority 1.
     */
    if (c->v1_xip_boot) {
        DeviceState *ddrc = qdev_new("hisi-ddrc-v1");
        SysBusDevice *ddrcbus = SYS_BUS_DEVICE(ddrc);

        if (flash_boot) {
            qdev_prop_set_uint64(ddrc, "ram-size", machine->ram_size);
        }
        sysbus_realize_and_unref(ddrcbus, &error_fatal);
        sysbus_mmio_map(ddrcbus, 0, HISI_V1_DDRC_BASE);
        sysbus_mmio_map_overlap(ddrcbus, 1, HISI_V1_DDRPHY_BASE, 1);
        if (flash_boot) {
            sysbus_mmio_map_overlap(ddrcbus, 2, c->ram_base, 1);
        }
    }

    /* CPUs — secondary CPUs start halted (held in reset until kernel brings them up) */
    for (n = 0; n < smp_cpus; n++) {
        cpuobj[n] = object_new(machine->cpu_type);
        if (n > 0) {
            object_property_set_bool(cpuobj[n], "start-powered-off", true,
                                     &error_fatal);
        }
        if (c->aarch64) {
            /* Reset straight into EL1 (no EL2, no EL3).  Two reasons:
             *  - qemu -kernel: entering at EL2 makes the kernel turn on VHE and
             *    use the EL2 (hyp) physical timer (PPI 10), which the vendor dtb's
             *    arm,armv8-timer does not declare (only sec-EL1 PPI 13 / NS-EL1
             *    PPI 14) — no tick, CPU idles in WFI forever, userspace never
             *    starts.
             *  - Flash boot: U-Boot would otherwise reset at EL3 and its bootm
             *    EL-drop ("exception return") to the (now absent) EL2 faults
             *    with "Illegal exception return at EL3".  At EL1 U-Boot just
             *    jumps to the kernel directly. */
            object_property_set_bool(cpuobj[n], "has_el2", false, &error_fatal);
            object_property_set_bool(cpuobj[n], "has_el3", false, &error_fatal);
            /* Match the vendor dtb's MPIDR scheme: each core is its own
             * cluster (cpu@0 reg=0x0, cpu@1 reg=0x100, ... i.e. Aff1=index).
             * QEMU would otherwise default to Aff0=index (0,1,...), so PSCI
             * CPU_ON(0x100) for the secondary would find no CPU (-EINVAL) and
             * SMP would fail. */
            object_property_set_uint(cpuobj[n], "mp-affinity",
                                     (uint64_t)n << 8, &error_fatal);
            /* Enable QEMU's built-in PSCI-over-SMC emulation on the CPU
             * itself.  arm_load_kernel() only applies psci_conduit on the
             * -kernel path; when flash-booting (U-Boot → kernel, no
             * -kernel) it is never reached, so the guest's PSCI SMC would
             * trap as an Undefined instruction at EL1 (no EL3 to service
             * it) and the kernel panics in psci_get_version().  Setting it
             * here covers both boot paths. */
            if (c->psci_conduit) {
                object_property_set_int(cpuobj[n], "psci-conduit",
                                        c->psci_conduit, &error_fatal);
            }
        }
        if (c->arch_timer_freq) {
            object_property_set_uint(cpuobj[n], "cntfrq", c->arch_timer_freq,
                                     &error_fatal);
        }
        qdev_realize(DEVICE(cpuobj[n]), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj[n]);
    }

    /* Mask-ROM ELF (-bios) is loaded once CPU 0 exists; the registered
     * reset hook fires after cpu_reset() to redirect PC to the ELF entry. */
    if (bios_boot) {
        maskrom = hisilicon_load_maskrom(sysmem, c, machine, cpu[0]);
    }

    /* Interrupt controller */
    if (c->use_gic) {
        int num_irq = c->gic_num_spi + GIC_INTERNAL;
        DeviceState *gicdev;
        SysBusDevice *gicbus;

        if (c->gic_mpcore_base) {
            /* Cortex-A9 family — use combined a9mpcore_priv that exposes
             * SCU + GIC cpu + gtimer + mptimer + wdt + GIC dist within
             * one 0x2000 region.  Required because A9's GIC cpu interface
             * sits at MPCORE+0x100 and overlaps the dist at MPCORE+0x1000
             * if mapped as separate sysbus regions. */
            gicdev = qdev_new("a9mpcore_priv");
            qdev_prop_set_uint32(gicdev, "num-cpu", smp_cpus);
            qdev_prop_set_uint32(gicdev, "num-irq", num_irq);
            gicbus = SYS_BUS_DEVICE(gicdev);
            sysbus_realize_and_unref(gicbus, &error_fatal);
            sysbus_mmio_map(gicbus, 0, c->gic_mpcore_base);
        } else if (c->gic_version == 3) {
            /* GICv3 (Hi3519DV500/Hi3516DV500): distributor + redistributor,
             * no CPU-interface MMIO -- the CPU interface is accessed via system
             * registers.  One redistributor region with one frame per CPU. */
            QList *redist_region_count = qlist_new();
            gicdev = qdev_new(gicv3_class_name());
            qdev_prop_set_uint32(gicdev, "revision", 3);
            qdev_prop_set_uint32(gicdev, "num-cpu", smp_cpus);
            qdev_prop_set_uint32(gicdev, "num-irq", num_irq);
            qlist_append_int(redist_region_count, smp_cpus);
            qdev_prop_set_array(gicdev, "redist-region-count",
                                redist_region_count);
            gicbus = SYS_BUS_DEVICE(gicdev);
            sysbus_realize_and_unref(gicbus, &error_fatal);
            sysbus_mmio_map(gicbus, 0, c->gic_dist_base);   /* GICD */
            sysbus_mmio_map(gicbus, 1, c->gic_redist_base); /* GICR */
        } else {
            gicdev = qdev_new(gic_class_name());
            qdev_prop_set_uint32(gicdev, "revision", 2);
            qdev_prop_set_uint32(gicdev, "num-cpu", smp_cpus);
            qdev_prop_set_uint32(gicdev, "num-irq", num_irq);
            qdev_prop_set_bit(gicdev, "has-security-extensions", false);
            gicbus = SYS_BUS_DEVICE(gicdev);
            sysbus_realize_and_unref(gicbus, &error_fatal);
            sysbus_mmio_map(gicbus, 0, c->gic_dist_base);
            sysbus_mmio_map(gicbus, 1, c->gic_cpu_base);
        }

        /* GIC outputs → each CPU.  Both plain GIC and a9mpcore_priv use the
         * grouped pattern: outputs 0..N-1 = IRQ for CPU 0..N-1; N..2N-1 =
         * FIQ; 2N..3N-1 = VIRQ; 3N..4N-1 = VFIQ.  (Per upstream
         * arm_gic_common.c:148–159 and a9mpcore_priv pass-through.)
         * The earlier interleaved pattern (irq_ofs = n * 4) happened to
         * work for V1–V5 IPC because N = 1 makes both layouts equivalent —
         * but breaks SMP when N > 1 (CPU0 takes wrong FIQs etc). */
        for (n = 0; n < smp_cpus; n++) {
            sysbus_connect_irq(gicbus, n + 0 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpu[n]), ARM_CPU_IRQ));
            sysbus_connect_irq(gicbus, n + 1 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpu[n]), ARM_CPU_FIQ));
            sysbus_connect_irq(gicbus, n + 2 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpu[n]), ARM_CPU_VIRQ));
            sysbus_connect_irq(gicbus, n + 3 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpu[n]), ARM_CPU_VFIQ));
        }

        if (!c->gic_mpcore_base) {
            /* CPU timer PPIs → GIC, per-CPU.  A9 doesn't have an ARM
             * generic timer; a9mpcore_priv has its own gtimer that wires
             * itself, so skip this for A9.  PPI input index for CPU C
             * is: gic_num_spi + C*GIC_INTERNAL + GIC_NR_SGIS + ppi (per
             * arm_gic_common.c gpio-in layout). */
            const int timer_ppi[] = {
                [GTIMER_PHYS] = HISI_PPI_PHYSTIMER,
                [GTIMER_VIRT] = HISI_PPI_VIRTTIMER,
                [GTIMER_HYP]  = HISI_PPI_HYPTIMER,
                [GTIMER_SEC]  = HISI_PPI_SECTIMER,
            };
            for (n = 0; n < smp_cpus; n++) {
                DeviceState *cpudev = DEVICE(cpu[n]);
                int ppibase = c->gic_num_spi + n * GIC_INTERNAL + GIC_NR_SGIS;
                for (int i = 0; i < ARRAY_SIZE(timer_ppi); i++) {
                    qdev_connect_gpio_out(cpudev, i,
                        qdev_get_gpio_in(gicdev, ppibase + timer_ppi[i]));
                }
                /* PMU */
                qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
                    qdev_get_gpio_in(gicdev, ppibase + 7));
            }
        }

        /* SPI IRQ array */
        num_pic = c->gic_num_spi;
        for (n = 0; n < num_pic; n++) {
            pic[n] = qdev_get_gpio_in(gicdev, n);
        }
    } else {
        /* PL190 VIC */
        DeviceState *vic = sysbus_create_varargs("pl190", c->vic_base,
                qdev_get_gpio_in(DEVICE(cpu[0]), ARM_CPU_IRQ),
                qdev_get_gpio_in(DEVICE(cpu[0]), ARM_CPU_FIQ),
                NULL);
        num_pic = 32;
        for (n = 0; n < num_pic; n++) {
            pic[n] = qdev_get_gpio_in(vic, n);
        }
    }

    /* Now that pic[] exists, wire the FMC op/DMA-done IRQ (fmc100 only). */
    if (fmc_irq_dev && c->fmc_irq && c->fmc_irq < num_pic) {
        sysbus_connect_irq(fmc_irq_dev, 0, pic[c->fmc_irq]);
    }

    /* SysCtrl — V1–V5 IPC and DVR/NVR scheme; STB family (HiSTB) uses
     * a different sysctl register layout that the hisi-sysctl model
     * doesn't implement, so leave sysctl_base = 0 to skip. */
    if (c->sysctl_base) {
        DeviceState *sysctl = qdev_new("hisi-sysctl");
        qdev_prop_set_uint32(sysctl, "soc-id", c->soc_id);
        qdev_prop_set_bit(sysctl, "byte-layout-id", c->chipid_byte_layout);
        qdev_prop_set_uint8(sysctl, "chip-variant", c->chip_variant);
        qdev_prop_set_uint32(sysctl, "v1-chip-id-88", c->v1_chip_id_88);
        qdev_prop_set_uint32(sysctl, "v1-chip-id-8c", c->v1_chip_id_8c);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sysctl), 0, c->sysctl_base);
        if (maskrom) {
            qdev_connect_gpio_out_named(sysctl, "remap-clear", 0,
                qemu_allocate_irq(hisilicon_maskrom_remap, maskrom, 0));
        }

        /*
         * Pinstrap injection for -bios mask-ROM runs.  Real silicon
         * drives SYSSTAT bit 5 from a BOOT_MODE pin; the av300 bootrom
         * reads SYSCTRL+0x8c once at entry and dispatches fastboot when
         * bit 5 is set.  Without this the bootrom falls through every
         * load attempt and soft-resets in a loop.  Must run AFTER
         * sysctl mmio map so the store lands in the device's regbank.
         */
        /*
         * SYSSTAT (0x8c) boot-strap bits, OR-ed together:
         *   bit5 (0x20) — BOOT_MODE fastboot pin, for -bios mask-ROM runs.
         *   SPI device type = SPI NAND.  U-Boot's get_boot_media() reads
         *     GET_SPI_DEVICE_TYPE(SYSSTAT); if it isn't NAND, spi_flash_probe()
         *     drives the SPI-NOR path (env-in-NOR, NOR scan) instead of NAND,
         *     so NAND-boot OpenIPC images loop on the missing NOR env and never
         *     attach UBI.  The strap BIT IS SOC-SPECIFIC: bit10 (0x400) on
         *     ev200/ev300/dv200/3518ev300, bit6 (0x40) on cv500/dv300.  Set
         *     both so whichever bit the running SoC samples reads NAND; the
         *     unused one is harmless.  See widgetii#115.
         */
        uint32_t sysstat = 0;
        if (bios_boot) {
            sysstat |= 0x20;
        }
        if (fmc_nand_boot) {
            sysstat |= 0x40 | 0x400;
        }
        if (sysstat) {
            address_space_stl(&address_space_memory,
                              c->sysctl_base + 0x8c, sysstat,
                              MEMTXATTRS_UNSPECIFIED, NULL);
        }
    }

    /* CRG — same caveat as sysctl: STB family uses a different layout. */
    if (c->crg_base) {
        DeviceState *crg = qdev_new("hisi-crg");
        if (c->cpu_srst_offset) {
            qdev_prop_set_uint32(crg, "cpu-srst-offset", c->cpu_srst_offset);
        }
        if (c->crg_pll_lock_offset) {
            qdev_prop_set_uint32(crg, "pll-lock-offset", c->crg_pll_lock_offset);
            qdev_prop_set_uint32(crg, "pll-lock-bits", c->crg_pll_lock_bits);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(crg), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(crg), 0, c->crg_base);
        if (c->cpu_srst_offset && smp_cpus > 1) {
            HisiCpuRelease *r = g_new0(HisiCpuRelease, 1);

            r->cpu = cpu[1];
            r->kernel_boot = machine->kernel_filename != NULL;
            qdev_connect_gpio_out_named(crg, "cpu1-reset", 0,
                qemu_allocate_irq(hisilicon_cpu1_reset, r, 0));
        }

        /* Pre-enable clocks (mimics U-Boot init before kernel boot) */
        for (n = 0; n < c->num_crg_defaults; n++) {
            address_space_stl(&address_space_memory,
                              c->crg_base + c->crg_defaults[n].offset,
                              c->crg_defaults[n].value,
                              MEMTXATTRS_UNSPECIFIED, NULL);
        }
    }

    /* Fastboot mode: when no -kernel is given, no flash-file, and a serial
     * backend exists, emulate the boot ROM download protocol on UART0
     * instead of creating PL011 immediately.  The hisi-fastboot device will
     * hand the chardev off to a newly-created PL011 after firmware upload
     * completes.  Flash boot skips fastboot — the boot ROM runs U-Boot
     * from flash directly. */
    bool fastboot_mode = !machine->kernel_filename && !flash_boot
                         && !bios_boot && serial_hd(0);
    qemu_irq uart0_irq = NULL;

    /* UARTs */
    for (n = 0; n < c->num_uarts; n++) {
        DeviceState *uart;
        if (n == 0 && fastboot_mode) {
            uart0_irq = pic[c->uart_irqs[0]];
            continue;   /* UART0 PL011 created later by fastboot device */
        }
        uart = pl011_create(c->uart_bases[n], pic[c->uart_irqs[n]],
                            serial_hd(n));

        /* If the SoC's vendor mach hardcodes a larger PL011 resource
         * size, the AMBA bus probe reads PrimeCell ID at the END of
         * that window — outside our 0x1000 PL011.  Alias only those
         * 0x20 bytes (PERIPHID0..3 + PCELLID0..3, offsets 0xfe0..0xfff)
         * to `base + window_size - 0x20`, so the AMBA driver sees the
         * right IDs and binds, without leaking the full PL011 register
         * block into the upper part of the window (which would let
         * stray accesses there read RX bytes from UARTDR).  See
         * openhisilicon#70. */
        if (c->uart_window_size > 0x1000) {
            MemoryRegion *pl011_mr =
                sysbus_mmio_get_region(SYS_BUS_DEVICE(uart), 0);
            MemoryRegion *alias = g_new0(MemoryRegion, 1);
            char *name = g_strdup_printf("pl011-id-mirror-%d", n);
            memory_region_init_alias(alias, OBJECT(uart), name,
                                     pl011_mr, 0xfe0, 0x20);
            memory_region_add_subregion(sysmem,
                c->uart_bases[n] + c->uart_window_size - 0x20,
                alias);
            g_free(name);
        }
    }
    /* Post-reset PL011 enable: writes UARTCR = UARTEN|TXE|RXE on the same
     * path U-Boot would on real silicon.  Required by SoCs whose vendor
     * Linux 3.0/3.4/3.10 PL011 driver writes to UARTDR before
     * initialising UARTCR (Hi3520DV200, Hi3535, Hi3536 flagship);
     * harmless elsewhere because newer drivers re-write CR during their
     * startup path.  Registered as a qemu reset handler so the write
     * lands AFTER PL011's own reset (which zeros UARTEN). */
    if (c->uart_pre_enable) {
        hisi_register_uart_pre_enable_reset(c);
    }

    /* Timers */
    if (c->xmsp804_timer) {
        /*
         * Goke V500 (gk7205v500/v510/v530, gk7202v330) xmsp804: four
         * single-timer blocks at 0x100 stride at timer_bases[0] (0x12000000).
         * Per xm720xxx.dtsi the per-block IRQs are GIC SPI 27, 6, 28 for
         * blocks 1/2/3 (block0 is the free-running clocksource, no IRQ).
         */
        DeviceState *t = qdev_new("hisi-xmsp804");
        qdev_prop_set_uint32(t, "freq", 24000000);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(t), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(t), 0, c->timer_bases[0]);
        sysbus_connect_irq(SYS_BUS_DEVICE(t), 1, pic[27]);
        sysbus_connect_irq(SYS_BUS_DEVICE(t), 2, pic[6]);
        sysbus_connect_irq(SYS_BUS_DEVICE(t), 3, pic[28]);
    } else {
        for (n = 0; n < c->num_timers; n++) {
            if (c->timer_freq) {
                DeviceState *t = qdev_new("sp804");
                qdev_prop_set_uint32(t, "freq0", c->timer_freq);
                qdev_prop_set_uint32(t, "freq1", c->timer_freq);
                sysbus_realize_and_unref(SYS_BUS_DEVICE(t), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(t), 0, c->timer_bases[n]);
                sysbus_connect_irq(SYS_BUS_DEVICE(t), 0, pic[c->timer_irqs[n]]);
            } else {
                sysbus_create_simple("sp804", c->timer_bases[n],
                                     pic[c->timer_irqs[n]]);
            }
        }
    }

    /* SPI (HiSilicon variant of PL022) */
    DeviceState *spi_devs[HISI_MAX_SPIS] = { NULL };
    for (n = 0; n < c->num_spis; n++) {
        spi_devs[n] = qdev_new("hisi-spi");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(spi_devs[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(spi_devs[n]), 0, c->spi_bases[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(spi_devs[n]), 0,
                           pic[c->spi_irqs[n]]);
    }

    /* DMA — PL080 by default, or "hisi-regbank" to stub the DVR/NVR
     * HiSilicon DW DMAC (a PL080 at that address would respond with
     * PL080 PrimeCell IDs that the vendor hisi-dmac driver rejects).
     * Regbank-stubbed DMA is later populated via regbanks[] in the
     * SoC config so the kernel sees a non-zero MMIO. */
    if (c->dma_base && (!c->dma_type || !strcmp(c->dma_type, "pl080"))) {
        DeviceState *dma = qdev_new("pl080");
        object_property_set_link(OBJECT(dma), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        SysBusDevice *busdev = SYS_BUS_DEVICE(dma);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->dma_base);
        sysbus_connect_irq(busdev, 0, pic[c->dma_irq]);
    }

    /* GPIOs (PL061) */
    for (n = 0; n < c->gpio_count; n++) {
        qemu_irq irq;
        hwaddr gpio_base = c->gpio_base + n * c->gpio_stride;

        if (c->gpio_irq_start) {
            irq = pic[c->gpio_irq_start + n]; /* per-port IRQs (GIC) */
        } else {
            irq = pic[c->gpio_irq];           /* shared IRQ (VIC or GIC) */
        }
        DeviceState *gp = hisilicon_create_pl061(sysmem, gpio_base,
                                                 c->gpio_stride, irq);

        /* Drive selected GPIO0 inputs high to model board pull-ups.  On
         * Hi3519DV500 the vendor U-Boot reads GPIO0_0 as an "USB/SD update"
         * request and treats LOW as "pressed"; with PL061 inputs defaulting
         * to 0 the board would wrongly enter the download loop instead of
         * booting.  GPIO0_0 has a pull-up on real boards (high = not pressed). */
        if (n == 0 && c->gpio0_input_high) {
            HisiGpioPull *p = g_new0(HisiGpioPull, 1);
            p->dev = gp;
            p->mask = c->gpio0_input_high;
            qemu_register_reset(hisilicon_gpio_pull_reset, p);
            hisilicon_gpio_pull_reset(p);
        }
    }

    /* Extra GPIO ports at non-contiguous addresses / IRQs */
    for (n = 0; n < HISI_MAX_GPIO_EXTRAS; n++) {
        if (!c->gpio_extras[n].base) {
            break;
        }
        hisilicon_create_pl061(sysmem, c->gpio_extras[n].base,
                               c->gpio_stride,
                               pic[c->gpio_extras[n].irq]);
    }

    /* FEMAC */
    if (c->femac_base) {
        DeviceState *femac = qdev_new("hisi-femac");
        qemu_configure_nic_device(femac, true, NULL);
        SysBusDevice *busdev = SYS_BUS_DEVICE(femac);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->femac_base);
        sysbus_connect_irq(busdev, 0, pic[c->femac_irq]);
    }

    /* GMAC (Gigabit Ethernet MAC) — for AV100, 3519V101, hi3536cv100… */
    if (c->gmac_base) {
        DeviceState *gmac = qdev_new("hisi-gmac");
        qemu_configure_nic_device(gmac, true, NULL);
        if (c->gmac_desc_size) {
            qdev_prop_set_uint32(gmac, "desc-size", c->gmac_desc_size);
        }
        if (c->gmac_rx_offset) {
            qdev_prop_set_uint32(gmac, "rx-pkt-offset", c->gmac_rx_offset);
        }
        SysBusDevice *busdev = SYS_BUS_DEVICE(gmac);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->gmac_base);
        sysbus_connect_irq(busdev, 0, pic[c->gmac_irq]);
    }

    /* SD/MMC — himciv200 */
    for (n = 0; n < c->num_himci; n++) {
        DeviceState *mmc = qdev_new("hisi-himci");
        SysBusDevice *busdev = SYS_BUS_DEVICE(mmc);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->himci_bases[n]);
        if (c->himci_irqs[n]) {
            sysbus_connect_irq(busdev, 0, pic[c->himci_irqs[n]]);
        }

        DriveInfo *di = drive_get(IF_SD, 0, n);
        if (di) {
            BusState *bus = qdev_get_child_bus(DEVICE(mmc), "sd-bus");
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
    }

    /* SD/MMC — SDHCI */
    for (n = 0; n < c->num_sdhci; n++) {
        DeviceState *sdhci = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(sdhci, "sd-spec-version", 3);
        qdev_prop_set_uint8(sdhci, "uhs", UHS_I);
        qdev_prop_set_uint64(sdhci, "capareg", 0x017834b4);
        SysBusDevice *busdev = SYS_BUS_DEVICE(sdhci);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->sdhci_bases[n]);
        if (c->sdhci_irqs[n]) {
            sysbus_connect_irq(busdev, 0, pic[c->sdhci_irqs[n]]);
        }

        DriveInfo *di = drive_get(IF_SD, 0, n);
        if (di) {
            BusState *bus = qdev_get_child_bus(sdhci, "sd-bus");
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
    }

    /* I2C — HiBVT for V2+, register-poll "hisi-i2c-v1" for CV100 family. */
    const char *i2c_type = c->i2c_type ? c->i2c_type : "hisi-i2c";
    DeviceState *i2c_devs[HISI_MAX_I2C] = { NULL };
    for (n = 0; n < c->num_i2c; n++) {
        i2c_devs[n] = qdev_new(i2c_type);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c_devs[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(i2c_devs[n]), 0, c->i2c_bases[n]);
    }

    /* MIPI RX */
    if (c->mipi_rx_base) {
        DeviceState *mipi = qdev_new("hisi-mipi-rx");
        SysBusDevice *busdev = SYS_BUS_DEVICE(mipi);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->mipi_rx_base);
        sysbus_connect_irq(busdev, 0, pic[c->mipi_rx_irq]);
    }

    /* RTC (SPI-bridge) */
    if (c->rtc_base) {
        DeviceState *rtc = qdev_new("hisi-rtc");
        SysBusDevice *busdev = SYS_BUS_DEVICE(rtc);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->rtc_base);
        sysbus_connect_irq(busdev, 0, pic[c->rtc_irq]);
    }

    /* VEDU + JPGE */
    if (c->vedu_base) {
        DeviceState *vedu = qdev_new("hisi-vedu");
        SysBusDevice *busdev = SYS_BUS_DEVICE(vedu);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->vedu_base);
        sysbus_mmio_map(busdev, 1, c->jpge_base);
        /* Skip the VEDU IRQ wire when hisi-vi-fp also drives it —
         * QEMU's GIC has one input level per SPI, so two devices
         * sharing pic[N] clobber each other (last-writer-wins). */
        if (!c->vi_fp_vedu_irq || c->vi_fp_vedu_irq != c->vedu_irq) {
            sysbus_connect_irq(busdev, 0, pic[c->vedu_irq]);
        }
        sysbus_connect_irq(busdev, 1, pic[c->jpge_irq]);
    }

    /* Hardware True Random Number Generator
     * (HISEC_TRNG_CTRL on V3+, RNG_GEN on V2) */
    if (c->hwrng_base) {
        DeviceState *rng = qdev_new("hisi-hwrng");
        qdev_prop_set_uint32(rng, "data-offset", c->hwrng_data_offset);
        SysBusDevice *busdev = SYS_BUS_DEVICE(rng);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->hwrng_base);
    }

    /* VI frame producer — periodic VI_CAP/VI_PROC/VPSS IRQ heartbeat
     * to wake the vendor MPP pipeline.  MVP, EV300 only for now. */
    if (c->vi_fp_base) {
        DeviceState *vifp = qdev_new("hisi-vi-fp");
        SysBusDevice *busdev = SYS_BUS_DEVICE(vifp);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->vi_fp_base);
        sysbus_connect_irq(busdev, 0, pic[c->vi_fp_cap_irq]);
        sysbus_connect_irq(busdev, 1, pic[c->vi_fp_proc_irq]);
        sysbus_connect_irq(busdev, 2, pic[c->vi_fp_vpss_irq]);
        if (c->vi_fp_vedu_irq) {
            sysbus_connect_irq(busdev, 3, pic[c->vi_fp_vedu_irq]);
        }
    }

    /* Watchdog (SP805-compatible, reuse cmsdk-apb-watchdog) */
    if (c->wdt_base) {
        Clock *wdt_clk = clock_new(OBJECT(machine), "wdt-clk");
        clock_set_hz(wdt_clk, c->wdt_freq);
        DeviceState *wdt = qdev_new("cmsdk-apb-watchdog");
        qdev_connect_clock_in(wdt, "WDOGCLK", wdt_clk);
        SysBusDevice *busdev = SYS_BUS_DEVICE(wdt);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->wdt_base);
        if (c->wdt_irq >= 0) {
            sysbus_connect_irq(busdev, 0, pic[c->wdt_irq]);
        }
    }

    /* Hardware GZIP decompressor (used by U-Boot hw_compressed first stage) */
    if (c->gzip_base) {
        DeviceState *gzip = qdev_new("hisi-gzip");
        SysBusDevice *busdev = SYS_BUS_DEVICE(gzip);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->gzip_base);
    }

    /* SATA AHCI controller (sysbus-AHCI) — DVR/NVR family.  Boot-to-shell
     * stub: no devices attached.  Vendor SATA/AHCI driver probes CAP/PI
     * and reports an empty bus, then returns cleanly. */
    if (c->sata_base) {
        DeviceState *sata = qdev_new("sysbus-ahci");
        int ports = c->sata_num_ports ? c->sata_num_ports : 1;
        qdev_prop_set_uint32(sata, "num-ports", ports);
        SysBusDevice *busdev = SYS_BUS_DEVICE(sata);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->sata_base);
        sysbus_connect_irq(busdev, 0, pic[c->sata_irq]);
    }

    /* USB host — sysbus EHCI 2.0 + sysbus OHCI 1.1.  Boot-to-shell stub:
     * no devices attached, generic-ehci / generic-ohci kernel drivers
     * probe and report an empty bus. */
    if (c->usb_ehci_base) {
        DeviceState *ehci = qdev_new("platform-ehci-usb");
        SysBusDevice *busdev = SYS_BUS_DEVICE(ehci);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->usb_ehci_base);
        sysbus_connect_irq(busdev, 0, pic[c->usb_ehci_irq]);
    }
    if (c->usb_ohci_base) {
        DeviceState *ohci = qdev_new("sysbus-ohci");
        SysBusDevice *busdev = SYS_BUS_DEVICE(ohci);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->usb_ohci_base);
        sysbus_connect_irq(busdev, 0, pic[c->usb_ohci_irq]);
    }

    /* USB 3.0 XHCI (sysbus) — DVR/NVR family Hi3531A / Hi3535 /
     * Hi3536-flagship.  Hi3536DV100 / Hi3521A leave xhci_base = 0. */
    if (c->xhci_base) {
        DeviceState *xhci = qdev_new("sysbus-xhci");
        if (c->xhci_slots) {
            qdev_prop_set_uint32(xhci, "slots", c->xhci_slots);
        }
        if (c->xhci_intrs) {
            qdev_prop_set_uint32(xhci, "intrs", c->xhci_intrs);
        }
        SysBusDevice *busdev = SYS_BUS_DEVICE(xhci);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->xhci_base);
        sysbus_connect_irq(busdev, 0, pic[c->xhci_irq]);
    }

    /* HiSilicon HIL2V200 L2 cache controller — Hi3520D family.
     * Required by vendor 3.0.x kernel's drivers/arm/mm/cache-hil2v200.c
     * which polls REG_L2_RINT.AUTO_END after every maintenance op; a
     * regbank stub returns 0 forever and the kernel hangs in
     * l2cache_driver_init.  See openhisilicon#66. */
    if (c->l2cache_base) {
        DeviceState *l2 = qdev_new("hisi-l2cache");
        SysBusDevice *busdev = SYS_BUS_DEVICE(l2);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->l2cache_base);
    }

    /* HiSilicon SF (single-FIFO) Ethernet controller — Hi3520D family.
     * Subclass of `hisi-femac` (same register layout) with the integrated
     * PHY responding at MDIO address 3 instead of 1 (vendor drivers/net/
     * hieth-sf/mdio.c scans the himii bus and binds at himii:03).  Full
     * TX/RX/IRQ inherited from FEMAC. */
    if (c->hieth_sf_base) {
        DeviceState *eth = qdev_new("hisi-hieth-sf");
        qemu_configure_nic_device(eth, true, NULL);
        SysBusDevice *busdev = SYS_BUS_DEVICE(eth);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->hieth_sf_base);
        sysbus_connect_irq(busdev, 0, pic[c->hieth_sf_irq]);
    }

    /* Generic register banks (pin mux, DDR PHY, PWM, etc.) */
    for (n = 0; n < c->num_regbanks; n++) {
        if (c->regbanks[n].base) {
            /* IVE gets a proper functional device instead of regbank */
            if (!strcmp(c->regbanks[n].name, "hisi-ive")) {
                DeviceState *ive = qdev_new("hisi-ive");
                SysBusDevice *busdev = SYS_BUS_DEVICE(ive);
                sysbus_realize_and_unref(busdev, &error_fatal);
                sysbus_mmio_map(busdev, 0, c->regbanks[n].base);
                /* IVE IRQ: EV300=SPI51, CV500=SPI37 — extract from DTS */
                continue;
            }

            /* NNIE: cv500-family CNN inference engine. Same pattern as
             * IVE above — functional device, IRQ wiring TODO (the
             * userspace test polls IRQ_STATUS like test-ive-ops.c). */
            if (!strcmp(c->regbanks[n].name, "hisi-nnie")) {
                DeviceState *nnie = qdev_new("hisi-nnie");
                SysBusDevice *busdev = SYS_BUS_DEVICE(nnie);
                sysbus_realize_and_unref(busdev, &error_fatal);
                sysbus_mmio_map(busdev, 0, c->regbanks[n].base);
                continue;
            }

            DeviceState *rb = qdev_new("hisi-regbank");
            qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
            qdev_prop_set_string(rb, "name", c->regbanks[n].name);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);

            if (!strcmp(c->regbanks[n].name, "hisi-nandc")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x20, 0x01,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
            }
            /*
             * VICAP/ISP: pre-set PT_INTF_MOD (offset 0x4000) so the
             * ISP driver's init doesn't poll a zero-valued register.
             * Also set ISP interrupt status at 0x41F8 to indicate
             * "frame done" so ISP init completes immediately.
             */
            if (!strcmp(c->regbanks[n].name, "hisi-viu")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x41F8, 0xFFFFFFFF,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
            }
        }
    }

    /* Sensor auto-attach via -machine sensor=<name>
     *
     * If the user did not pass -machine sensor=..., fall back to the
     * SoC's default_sensor (set per-config to mirror the typical
     * OpenIPC reference board).  Use sensor=none to disable.
     *
     * I2C addresses below are 7-bit slave addresses (vendor docs use 8-bit
     * which is shifted left by one).  All SmartSens sensors share addr 0x30
     * so only one can be attached at a time — the user picks which one
     * via the machine arg.
     */
    {
        HisiMachineState *hms = (HisiMachineState *)machine;
        const char *sensor_name = hms->sensor;
        if (!sensor_name && c->default_sensor) {
            sensor_name = c->default_sensor;
        } else if (sensor_name && !strcmp(sensor_name, "none")) {
            sensor_name = NULL;
        }
        /* SPI sensors (Sony 3-wire, e.g. CV100 + ssp_sony.ko) take
         * precedence — IMX122/IMX222 share one device that ipctool
         * always reports as "IMX122". */
        if (sensor_name &&
            (!strcmp(sensor_name, "imx122") ||
             !strcmp(sensor_name, "imx222"))) {
            if (c->num_spis == 0 || !spi_devs[0]) {
                error_report("sensor '%s' requires an SPI controller on "
                             "this SoC", sensor_name);
                exit(1);
            }
            BusState *ssi_bus = qdev_get_child_bus(spi_devs[0], "ssi");
            DeviceState *sensor = qdev_new("hisi-imx122");
            qdev_realize_and_unref(sensor, ssi_bus, &error_fatal);
        } else if (sensor_name && c->num_i2c > 0 && i2c_devs[0]) {
            BusState *i2c_bus = qdev_get_child_bus(i2c_devs[0], "i2c");
            DeviceState *sensor = NULL;
            uint8_t i2c_addr = 0;

            if (!strcmp(sensor_name, "imx335")) {
                sensor = qdev_new("hisi-imx335");
                i2c_addr = 0x1A;
            } else if (!strcmp(sensor_name, "imx307")) {
                sensor = qdev_new("hisi-imx307");
                i2c_addr = 0x1A;
            } else if (!strcmp(sensor_name, "imx291")) {
                sensor = qdev_new("hisi-imx291");
                i2c_addr = 0x1A;
            } else if (!strcmp(sensor_name, "imx415")) {
                sensor = qdev_new("hisi-imx415");
                i2c_addr = 0x1A;
            } else if (!strcmp(sensor_name, "imx385")) {
                sensor = qdev_new("hisi-imx385");
                i2c_addr = 0x1A;
            } else if (!strcmp(sensor_name, "f37")) {
                sensor = qdev_new("hisi-f37");
                i2c_addr = 0x40;
            } else if (!strcmp(sensor_name, "gc2053")) {
                sensor = qdev_new("hisi-gc2053");
                i2c_addr = 0x37;
            } else if (!strcmp(sensor_name, "sp2305")) {
                sensor = qdev_new("hisi-sp2305");
                i2c_addr = 0x3C;
            } else if (!strcmp(sensor_name, "mis2006")) {
                sensor = qdev_new("hisi-mis2006");
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "nvp6124b")) {
                sensor = qdev_new("hisi-nvp6124b");
                i2c_addr = 0x60;          /* 7-bit; vendor uses 0x60..0x66 */
            } else if (!strcmp(sensor_name, "sc2315e")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0x22);
                qdev_prop_set_uint8(sensor, "id_low",  0x38);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc2315")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0x23);
                qdev_prop_set_uint8(sensor, "id_low",  0x11);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc2235p")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0x22);
                qdev_prop_set_uint8(sensor, "id_low",  0x32);
                qdev_prop_set_uint8(sensor, "disc",    0x01);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc2235e")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0x22);
                qdev_prop_set_uint8(sensor, "id_low",  0x32);
                qdev_prop_set_uint8(sensor, "disc",    0x20);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc2335")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0xCB);
                qdev_prop_set_uint8(sensor, "id_low",  0x14);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc2239")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0xCB);
                qdev_prop_set_uint8(sensor, "id_low",  0x10);
                i2c_addr = 0x30;
            } else if (!strcmp(sensor_name, "sc307h")) {
                sensor = qdev_new("hisi-smartsens");
                qdev_prop_set_uint8(sensor, "id_high", 0xCB);
                qdev_prop_set_uint8(sensor, "id_low",  0x1C);
                i2c_addr = 0x30;
            } else {
                error_report("Unknown sensor '%s' (supported: imx291, "
                             "imx307, imx335, imx385, imx415, f37, "
                             "gc2053, sp2305, mis2006, nvp6124b, "
                             "sc2235p, sc2235e, sc2315, sc2315e, sc2335, "
                             "sc2239, sc307h, imx122, imx222; or 'none' "
                             "to disable default)",
                             sensor_name);
                exit(1);
            }

            qdev_prop_set_uint8(sensor, "address", i2c_addr);
            qdev_realize_and_unref(sensor, i2c_bus, &error_fatal);
        } else if (sensor_name) {
            error_report("sensor '%s' requested but this SoC has no I2C "
                         "or matching SPI controller", sensor_name);
            exit(1);
        }
    }

    if (flash_boot) {
        if (c->aarch64) {
            /* aarch64 V5 (Hi3519DV500/Hi3516DV500): place the gzip
             * self-extracting u-boot-z.bin at its DDR link address and start
             * CPU0 there.  The on-die HW gzip (hisi-gzip @ gzip_base)
             * decompresses it to CONFIG_SYS_TEXT_BASE at runtime. */
            HisiMachineState *hms = (HisiMachineState *)machine;
            char *udata = NULL;
            gsize ulen = 0;
            if (!hms->flash_file ||
                !g_file_get_contents(hms->flash_file, &udata, &ulen, NULL)) {
                error_report("hisilicon: %s needs -machine %s,flash-file="
                             "<u-boot-z.bin>", c->name, c->name);
                exit(1);
            }
            /*
             * The flash may hold either the raw self-extracting u-boot-z at
             * offset 0, or a full NOR image whose first partition is the
             * GSL-signed boot image (boot-<soc>-<binning>-nor.bin) — there the
             * GSL/DDR header sits at offset 0 and the u-boot-z payload is
             * embedded deeper (e.g. 0x11200). Both open with an aarch64 reset
             * branch (b, opcode 0x14......) immediately followed by a run of
             * 0xdeadbeef self-descriptor markers, so locate that block and load
             * it at the DDR link address. Loading the GSL header verbatim would
             * land non-executable bytes at the entry and hang at "Starting".
             */
            size_t uoff = 0;
            {
                const uint32_t *w = (const uint32_t *)udata;
                size_t nw = ulen / 4;
                size_t scan = MIN(nw, (size_t)(0x40000 / 4)); /* first 256 KiB */
                for (size_t i = 0; i + 5 <= scan; i++) {
                    if ((le32_to_cpu(w[i]) & 0xFC000000) == 0x14000000 &&
                        le32_to_cpu(w[i + 1]) == 0xdeadbeef &&
                        le32_to_cpu(w[i + 2]) == 0xdeadbeef &&
                        le32_to_cpu(w[i + 3]) == 0xdeadbeef &&
                        le32_to_cpu(w[i + 4]) == 0xdeadbeef) {
                        uoff = (size_t)i * 4;
                        break;
                    }
                }
            }
            rom_add_blob_fixed("hisilicon.uboot", udata + uoff, ulen - uoff,
                               c->uboot_load_addr);
            g_free(udata);

            HisiMaskromReset *info = g_new0(HisiMaskromReset, 1);
            info->cpu = cpu[0];
            info->entry = c->uboot_load_addr;
            qemu_register_reset(hisilicon_maskrom_cpu_reset, info);
        } else {
            /* ARMv7/v5: boot ROM at 0x0 (built by hisilicon_write_bootrom)
             * copies U-Boot from the flash window to DDR; CPU starts at
             * reset 0.  With no -kernel nothing else resets CPU0, so
             * without this a guest SC_SYSRES reset every device but left
             * U-Boot spinning in its reset loop. */
            HisiMaskromReset *info = g_new0(HisiMaskromReset, 1);
            info->cpu = cpu[0];
            info->entry = 0;
            qemu_register_reset(hisilicon_maskrom_cpu_reset, info);
        }
    } else if (bios_boot) {
        /* Mask-ROM emulation: the bootrom ELF is already in ROM at
         * 0x04000000, and a reset hook redirects PC there.  Nothing else
         * to do — the bootrom drives the boot itself, including UART0. */
    } else if (fastboot_mode) {
        /* Boot ROM fastboot: halt CPU and wait for serial firmware upload */
        DeviceState *fb = qdev_new(TYPE_HISI_FASTBOOT);
        qdev_prop_set_chr(fb, "chardev", serial_hd(0));
        hisi_fastboot_setup(fb, CPU(cpu[0]), serial_hd(0),
                            c->uart_bases[0], uart0_irq);
        qdev_realize(fb, NULL, &error_fatal);

        CPUState *cs = CPU(cpu[0]);
        cs->halted = 1;
    } else {
        /* Normal boot path: patch DTB and load kernel */
        char *patched_kernel = NULL;
        if (machine->kernel_filename) {
            patched_kernel = hisilicon_patch_appended_dtb(
                machine->kernel_filename, c->ram_base + 0x8000, c);
            if (patched_kernel) {
                machine->kernel_filename = patched_kernel;
            }
        }

        /*
         * Inject per-SoC kernel cmdline defaults so run scripts and
         * CI don't have to hardcode them.  User-supplied -append args
         * take precedence if they already contain the same keys.
         */
        const char *user_cmdline = machine->kernel_cmdline ?: "";
        bool has_mem = strstr(user_cmdline, "mem=") != NULL;
        bool has_mmz = strstr(user_cmdline, "mmz=") != NULL ||
                       strstr(user_cmdline, "mmz_allocator=") != NULL;
        /* If the user shrank -m below the SoC default, our SoC-default
         * extra_cmdline (e.g. "mmz=anonymous,0,0x82000000,224M") would
         * pin a CMA region past the actual end of memory and the kernel
         * would panic before producing any console output.  Treat the
         * smaller -m as opt-out from the default MMZ pin. */
        bool ram_overridden = machine->ram_size < c->ram_size_default;
        bool has_extra = c->extra_cmdline &&
                         strstr(user_cmdline, c->extra_cmdline) != NULL;
        bool want_mem = c->kernel_mem_mb && !has_mem;
        bool want_extra = c->extra_cmdline && !has_extra &&
                          !has_mmz && !ram_overridden;

        if (want_mem || want_extra) {
            GString *cl = g_string_new(user_cmdline);
            if (cl->len && cl->str[cl->len - 1] != ' ') {
                g_string_append_c(cl, ' ');
            }
            if (want_mem) {
                g_string_append_printf(cl, "mem=%uM ", c->kernel_mem_mb);
            }
            if (want_extra) {
                g_string_append(cl, c->extra_cmdline);
            }
            machine->kernel_cmdline = g_string_free(cl, false);
        }

        /*
         * The guest is told the whole DDR, as U-Boot tells it: ATAG_MEM,
         * which an ATAG_DTB_COMPAT kernel copies into its memory node, is
         * what OpenIPC's load_hisilicon reads as total RAM, and it carves
         * MMZ out of what lies above mem=.  Reporting kernel_mem_mb there
         * instead (32M) made the firmware see total == os_mem and load no
         * modules.
         *
         * kernel_mem_mb keeps its other job, placing the initrd and DTB:
         * at its first half, exactly as before.  That is below any MMZ the
         * command line carves -- the hisi allocator's above mem=, and the
         * CMA one inside it (hi3516ev300 CI: mem=128M with CMA from
         * 0x42000000; an initrd placed by mem= at +64M landed in that
         * region and the kernel died before its console came up).
         */
        hisilicon_binfo.ram_size = machine->ram_size;
        hisilicon_binfo.kernel_ram_size = (uint64_t)c->kernel_mem_mb * MiB;
        hisilicon_binfo.loader_start = c->ram_base;
        hisilicon_binfo.board_id = c->board_id; /* ATAGs machine_arch_type */
        hisilicon_binfo.write_secondary_boot = hisilicon_write_secondary_boot;
        hisilicon_binfo.secondary_cpu_reset_hook = hisilicon_reset_secondary;
        if (c->psci_conduit) {
            /* ARMv8 STB family — PSCI/SMC for SMP, suppresses upstream
             * "smpboot" stub at 0x0 that collides with the bootloader. */
            hisilicon_binfo.psci_conduit = c->psci_conduit;
        }
        arm_load_kernel(cpu[0], machine, &hisilicon_binfo);
    }
}

/* ── Machine-level property accessors ──────────────────────────────── */

static char *hisi_machine_get_sensor(Object *obj, Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    return g_strdup(s->sensor);
}

static void hisi_machine_set_sensor(Object *obj, const char *value,
                                     Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

static char *hisi_machine_get_flash_file(Object *obj, Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    return g_strdup(s->flash_file);
}

static void hisi_machine_set_flash_file(Object *obj, const char *value,
                                        Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

/* ── Per-machine wrappers ──────────────────────────────────────────── */

/*
 * All HiSilicon machines use HisiMachineState (for the sensor property)
 * and the ARM interface array (for arm/aarch64 dual-build).
 */
#define DEFINE_HISI_MACHINE(namestr, tag, config)                    \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        hisilicon_common_init(machine, &config);                     \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        ObjectClass *oc = OBJECT_CLASS(mc);                          \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu_type;                      \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "hisilicon.ram";                        \
        mc->block_default_type = IF_MTD;                             \
        mc->ignore_memory_transaction_failures = true;               \
        object_class_property_add_str(oc, "sensor",                  \
            hisi_machine_get_sensor, hisi_machine_set_sensor);       \
        object_class_property_set_description(oc, "sensor",          \
            "Image sensor to attach (e.g. imx335)");                 \
        object_class_property_add_str(oc, "flash-file",              \
            hisi_machine_get_flash_file,                             \
            hisi_machine_set_flash_file);                            \
        object_class_property_set_description(oc, "flash-file",      \
            "SPI NOR flash image to attach to the SoC's flash "      \
            "controller (forwards to whichever device the machine "  \
            "instantiates: hisi-fmc or hisi-sfc350).");              \
        if (config.max_cpus > 0) {                                   \
            mc->max_cpus = config.max_cpus;                          \
        }                                                            \
        if (config.default_cpus > 0) {                               \
            mc->default_cpus = config.default_cpus;                  \
        }                                                            \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, MACHINE, HisiMachineState,      \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_HISI_MACHINE("hi3516cv100", hi3516cv100, hi3516cv100_soc)
DEFINE_HISI_MACHINE("hi3518av100", hi3518av100, hi3518av100_soc)
DEFINE_HISI_MACHINE("hi3518cv100", hi3518cv100, hi3518cv100_soc)
DEFINE_HISI_MACHINE("hi3518ev100", hi3518ev100, hi3518ev100_soc)
DEFINE_HISI_MACHINE("hi3516cv200", hi3516cv200, hi3516cv200_soc)
DEFINE_HISI_MACHINE("hi3516av100", hi3516av100, hi3516av100_soc)
DEFINE_HISI_MACHINE("hi3516dv100", hi3516dv100, hi3516dv100_soc)
DEFINE_HISI_MACHINE("hi3516cv300", hi3516cv300, hi3516cv300_soc)
DEFINE_HISI_MACHINE("hi3516cv500", hi3516cv500, hi3516cv500_soc)
DEFINE_HISI_MACHINE("hi3516av300", hi3516av300, hi3516av300_soc)
DEFINE_HISI_MACHINE("hi3516dv300", hi3516dv300, hi3516dv300_soc)
DEFINE_HISI_MACHINE("hi3519v101", hi3519v101, hi3519v101_soc)
DEFINE_HISI_MACHINE("hi3516av200", hi3516av200, hi3516av200_soc)
DEFINE_HISI_MACHINE("hi3516ev300", hi3516ev300, hi3516ev300_soc)
DEFINE_HISI_MACHINE("hi3516ev200", hi3516ev200, hi3516ev200_soc)
DEFINE_HISI_MACHINE("hi3518ev300", hi3518ev300, hi3518ev300_soc)
DEFINE_HISI_MACHINE("hi3516dv200", hi3516dv200, hi3516dv200_soc)
DEFINE_HISI_MACHINE("gk7205v200", gk7205v200, gk7205v200_soc)
DEFINE_HISI_MACHINE("gk7201v200", gk7201v200, gk7201v200_soc)
DEFINE_HISI_MACHINE("gk7205v300", gk7205v300, gk7205v300_soc)
DEFINE_HISI_MACHINE("gk7202v300", gk7202v300, gk7202v300_soc)
DEFINE_HISI_MACHINE("gk7605v100", gk7605v100, gk7605v100_soc)
DEFINE_HISI_MACHINE("gk7205v500", gk7205v500, gk7205v500_soc)
DEFINE_HISI_MACHINE("gk7205v510", gk7205v510, gk7205v510_soc)
DEFINE_HISI_MACHINE("gk7205v530", gk7205v530, gk7205v530_soc)
DEFINE_HISI_MACHINE("gk7202v330", gk7202v330, gk7202v330_soc)
DEFINE_HISI_MACHINE("gk7206v10",  gk7206v10,  gk7206v10_soc)
DEFINE_HISI_MACHINE("gk7206v12a", gk7206v12a, gk7206v12a_soc)
DEFINE_HISI_MACHINE("gk7606v1",   gk7606v1,   gk7606v1_soc)
DEFINE_HISI_MACHINE("hi3516cv608", hi3516cv608, hi3516cv608_soc)
DEFINE_HISI_MACHINE("hi3516cv610", hi3516cv610, hi3516cv610_soc)
DEFINE_HISI_MACHINE("hi3516cv613", hi3516cv613, hi3516cv613_soc)
DEFINE_HISI_MACHINE("hi3519dv500", hi3519dv500, hi3519dv500_soc)
DEFINE_HISI_MACHINE("hi3516dv500", hi3516dv500, hi3516dv500_soc)
DEFINE_HISI_MACHINE("hi3536dv100", hi3536dv100, hi3536dv100_soc)
DEFINE_HISI_MACHINE("hi3521a",     hi3521a,     hi3521a_soc)
DEFINE_HISI_MACHINE("hi3531a",     hi3531a,     hi3531a_soc)
DEFINE_HISI_MACHINE("hi3536",      hi3536,      hi3536_soc)
DEFINE_HISI_MACHINE("hi3521dv100", hi3521dv100, hi3521dv100_soc)
DEFINE_HISI_MACHINE("hi3520dv200", hi3520dv200, hi3520dv200_soc)
DEFINE_HISI_MACHINE("hi3520dv300", hi3520dv300, hi3520dv300_soc)
DEFINE_HISI_MACHINE("hi3520dv400", hi3520dv400, hi3520dv400_soc)
DEFINE_HISI_MACHINE("hi3531dv100", hi3531dv100, hi3531dv100_soc)
DEFINE_HISI_MACHINE("hi3535",      hi3535,      hi3535_soc)
DEFINE_HISI_MACHINE("hi3536cv100", hi3536cv100, hi3536cv100_soc)
DEFINE_HISI_MACHINE("hi3798cv200", hi3798cv200, hi3798cv200_soc)
DEFINE_HISI_MACHINE("hi3796mv100", hi3796mv100, hi3796mv100_soc)
