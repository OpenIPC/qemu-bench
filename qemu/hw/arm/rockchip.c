/*
 * Rockchip RV1103 / RV1106 / RV1103B / RV1126 / RV1109 IP-camera SoC
 * machines.  The RV1106 family is described here; RV1103B and RV1126 are
 * further configs below (their own maps and boot stand-ins).
 *
 * One Cortex-A7, a GIC-400 and the ARM generic timer (24 MHz), DDR at 0
 * and the peripherals from 0xFF000000.  Addresses and interrupts are the
 * vendor SDK's arch/arm/boot/dts/rv1106.dtsi (rockchip-linux/kernel
 * develop-5.10; Luckfox Pico SDK 1.4 has the same map):
 *
 *   0xFF000000 GRF (PERI_GRF, reboot mode at +0x20200)  rk-syscon
 *   0xFF1C0000 RTC                                      hw/rtc/rk-rtc.c
 *   0xFF1F0000 GIC-400 (dist +0x1000, cpu +0x2000)      a15mpcore_priv
 *   0xFF300000 PMU                                      rk-syscon
 *   0xFF310000, 0xFF320000, 0xFF450000, 0xFF460000, 0xFF470000
 *              I2C0..4 (the sensor on I2C4)             hw/i2c/rk-i2c.c
 *   0xFF380000, 0xFF530000 + n * 0x10000  GPIO0..4      hw/gpio/rk-gpio.c
 *   0xFF388000 PMUIOC   0xFF538000 IOC                  rk-syscon
 *   0xFF3A0000 CRU (TOPCRU +0x10000: PLLs, MODE_CON)    rk-syscon
 *   0xFF4A0000 + n * 0x10000 UART0..5 (DW APB 16550)    serial-mm
 *   0xFF3C0000 SARADC v2 (keys read released)           rk-syscon
 *   0xFF3C8000 TSADC (reads about 40 C)                 rk-syscon
 *   0xFF3D0000 OTP, secure OTP at +0x8000               rk-syscon
 *   0xFF420000 DMAC (PL330)                             pl330
 *   0xFF440000 crypto v3 (hash engine)                  hw/misc/rk-crypto.c
 *   0xFF448000 TRNG v1                                  hw/misc/rk-trng.c
 *   0xFF520000 hardware decompressor (gzip, zlib, LZ4)  hw/misc/rk-decom.c
 *   0xFF5A0000 watchdog (DW APB)                        fh-wdt
 *   0xFF5C0000 mailbox to the MCU ("done" stand-in)     hw/misc/rk-mailbox.c
 *   0xFF6C0000 system SRAM, 256 KiB
 *   0xFF9A0000 SDIO, 0xFFA90000 eMMC, 0xFFAA0000 SD (DW MSHC 2.70a)
 *                                                       hisi-himci
 *   0xFFA80000 GMAC (dwmac-4.20a, integrated PHY at 2)  axera-eqos
 *   0xFFAC0000 SFC (SPI NOR / SPI NAND)                 hw/ssi/rk-sfc.c
 *   0xFFFF0000 boot ROM
 *
 * Boot: flash-file= is a camera's SPI NOR image.  The mask ROM stand-in
 * finds the RKNS ("header v2") ID block, skips its first image (the
 * closed DDR-init blob: DRAM needs no training here) and loads the next
 * runnable one, the vendor SPL, at DRAM 0, where it is linked.  What the ROM and
 * the DDR blob leave behind for it is recreated: the boot medium at
 * SRAM + 0x10 and the preloader ATAGs (DRAM size, console, SoC info) at
 * 2M - 8K.
 * The SPL then reads U-Boot from the flash through the SFC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/rk-scene.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qobject/qlist.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "net/net.h"
#include "hw/cpu/a15mpcore.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/loader.h"
#include "qemu/cutils.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "hw/usb/hcd-dwc3.h"
#include <libfdt.h>
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "block/block-common.h"
#include "target/arm/cpu.h"
#include "target/arm/arm-powerctl.h"
#include "qemu/main-loop.h"
#include "hw/i2c/bitbang_i2c.h"
#include "hw/nvram/eeprom_at24c.h"

#define RK_DDR_BASE         0x00000000
#define RK_BOOTROM_BASE     0xFFFF0000
#define RK_BOOTROM_SIZE     0x10000
#define RK_NUM_SPI          128

/* What the vendor SPL expects the boot ROM and DDR blob to leave. */
#define RK_ATAGS_BASE       (0x200000 - 0x2000)
#define ATAG_CORE           0x54410001
#define ATAG_NONE           0x00000000
#define ATAG_SERIAL         0x54410050
#define ATAG_DDR_MEM        0x54410052
#define ATAG_SOC_INFO       0x54410057
#define SOC_FLAGS_TDBT      (1u << 0)
#define BROM_BOOT_SPINOR    3
#define BROM_BOOT_SPINAND   4

/*
 * TOPCRU PLL registers, from clk_rv1106.c: APLL CON(0), CPLL CON(8),
 * DPLL CON(16), GPLL CON(24), all pll_rk3328 (LOCK = CON1 bit 10).  They
 * come out of reset at the rates the vendor loader leaves (APLL 1104,
 * CPLL 1000, DPLL 924, GPLL 1188 MHz) and in normal mode, so a kernel
 * that only reads them gets real rates instead of a divide by zero.
 */
static const uint32_t rv1106_cru_lock[] = {
    0x10004, 0x10024, 0x10044, 0x10064,
};

static const uint32_t rv1106_cru_init[] = {
    0x10000, 0x205c, 0x10004, 0x1041,       /* APLL 24 * 92 / 2 */
    0x10020, 0x107d, 0x10024, 0x1043,       /* CPLL 24 * 125 / 3 */
    0x10040, 0x204d, 0x10044, 0x1041,       /* DPLL 24 * 77 / 2 */
    0x10060, 0x2063, 0x10064, 0x1041,       /* GPLL 24 * 99 / 2 */
    0x10280, 0x0015,                        /* MODE_CON: all normal */
    0x1f280, 0x0001,                        /* SUBDDRMODE_CON */
};

/*
 * SARADC v2 (0xFF3C0000): conversion always done (END_INT_ST, 0x110) and
 * every channel (DATA0..15 from 0x120) at full scale, 10 bits: the
 * recovery / download key on channel 0 reads released.  U-Boot's
 * rockchip_dnl_key_pressed() takes a reading of 0 as "held" and stops in
 * rockusb.
 */
static const uint32_t rv1106_saradc_const[] = {
    0x110, 1,
    0x120, 0x3ff, 0x124, 0x3ff, 0x128, 0x3ff, 0x12c, 0x3ff,
    0x130, 0x3ff, 0x134, 0x3ff, 0x138, 0x3ff, 0x13c, 0x3ff,
};

/*
 * TSADC (0xFF3C8000), the kernel's rk_tsadcv4_get_temp: channel 0's code
 * at DATA (0x2c).  rv1106_code_table puts 504 at 25 C and 605 at 85 C;
 * 530 is about 40 C, a camera at rest.  Control writes are hiword-masked
 * (AUTO_SRC_EN_MASK).
 */
static const uint32_t rv1106_tsadc_const[] = {
    0x2c, 530,
};

static const uint32_t rv1106_otp_status[] = {
    0x0304, 0xffff, 0x8304, 0xffff,
};

/*
 * RV1103B CRU (clk-rv1103b.c): TOPCRU at +0x60000, and only DPLL CON(16)
 * and GPLL CON(24) -- the core runs from GPLL or its PVTPLL.  Same
 * pll_rk3328 layout and rates as the RV1106's.
 */
/*
 * RV1126 (clk-rv1126.c, develop-4.19): the PMUCRU at 0xFF480000 holds
 * GPLL (PMU_PLL_CON(0) at 0x10, PMU_MODE at 0); the CRU at +0x10000
 * APLL CON(0), DPLL CON(8), CPLL CON(16), HPLL CON(24) and MODE_CON at
 * 0x90 (shifts 0, 2, 4, 6).  The same pll_rk3328 layout as the RV1106.
 */
static const uint32_t rv1126_cru_lock[] = {
    0x00014, 0x10004, 0x10024, 0x10044, 0x10064,
};

static const uint32_t rv1126_cru_init[] = {
    0x00010, 0x2063, 0x00014, 0x1041,       /* GPLL 24 * 99 / 2 */
    0x00000, 0x0001,                        /* PMU_MODE: GPLL normal */
    0x10000, 0x205c, 0x10004, 0x1041,       /* APLL 24 * 92 / 2 */
    0x10020, 0x204d, 0x10024, 0x1041,       /* DPLL 24 * 77 / 2 */
    0x10040, 0x107d, 0x10044, 0x1043,       /* CPLL 24 * 125 / 3 */
    0x10060, 0x2063, 0x10064, 0x1041,       /* HPLL 24 * 99 / 2 */
    0x10090, 0x0055,                        /* MODE_CON: all normal */
};

/* The fractional dividers (COMPOSITE_FRACMUX): whole-word registers */
static const uint32_t rv1126_cru_frac[] = {
    0x00100 + 5 * 4, 0x00100 + 13 * 4,                  /* PMU_CLKSEL */
    0x10100 + 11 * 4, 0x10100 + 13 * 4, 0x10100 + 15 * 4, 0x10100 + 17 * 4,
    0x10100 + 19 * 4, 0x10100 + 28 * 4, 0x10100 + 29 * 4, 0x10100 + 32 * 4,
    0x10100 + 34 * 4, 0x10100 + 37 * 4, 0x10100 + 48 * 4, 0x10100 + 52 * 4,
    0x10100 + 74 * 4,
};

static const uint32_t rv1106_cru_frac[] = {
    0x10300 + 6 * 4, 0x10300 + 8 * 4, 0x10300 + 10 * 4, 0x10300 + 12 * 4,
    0x10300 + 14 * 4, 0x10300 + 16 * 4, 0x10300 + 18 * 4, 0x10300 + 20 * 4,
    0x10300 + 26 * 4, 0x10300 + 28 * 4, 0x10300 + 30 * 4, 0x10300 + 32 * 4,
};

static const uint32_t rv1103b_cru_frac[] = {
    0x60300 + 10 * 4, 0x60300 + 11 * 4, 0x60300 + 12 * 4, 0x60300 + 21 * 4,
    0x70300 + 1 * 4,
};

/*
 * RV1126 PMU (pm_domains.c rv1126_pmu): power requests at PWR 0x110, their
 * status at 0x108 (1 = off); bus idle requests at 0xc0 (and 0xc4 for the
 * NPU and crypto domains, whose ack / idle bits sit 16 higher), their ack
 * at 0xd0 and idle status at 0xd8.  Every handshake completes at once.
 */
static const uint32_t rv1126_pmu_mirror[] = {
    0x108, 0x110, 0,
    0x0d0, 0x0c0, 0, 0x0d0, 0x0c4, 16,
    0x0d8, 0x0c0, 0, 0x0d8, 0x0c4, 16,
};

/*
 * The RV1126's parallel NAND controller (0xFFC80000, the kernel's
 * rkflash nandc.c) on a board without a parallel NAND: FMCTL reads
 * ready with an empty FIFO (bits 9, 10), and chip 0's data port reads
 * FF -- no chip answers.  Without it every command waits out 100000
 * polls of the ready bit.
 */
static const uint32_t rv1126_nandc_const[] = {
    0x000, 0x600, 0x800, 0xff,
};

/*
 * The RV1126's naneng USB2 PHYs (0xFF4C0000, 0xFF4C8000;
 * phy-rockchip-naneng-usb2.c): the Rterm self-calibration result at
 * +0x34, bit 4 done, a mid-scale code below it.
 */
static const uint32_t rv1126_usbphy_const[] = {
    0x0034, 0x18, 0x8034, 0x18,
};

static const uint32_t rv1103b_cru_lock[] = {
    0x60044, 0x60064,
};

static const uint32_t rv1103b_cru_init[] = {
    0x60040, 0x204d, 0x60044, 0x1041,       /* DPLL 24 * 77 / 2 */
    0x60060, 0x2063, 0x60064, 0x1041,       /* GPLL 24 * 99 / 2 */
    0x60280, 0x0005,                        /* MODE_CON: normal */
    0x00a28, 0x00f0,                        /* SOFTRST(10): HPMCU held */
};

typedef struct RkSoCConfig {
    const char *desc;
    uint64_t ram_size_default;
    hwaddr periph_base;         /* catch-all for what is not modelled */
    uint64_t periph_size;
    hwaddr gic_base;
    hwaddr sram_base;
    uint32_t sram_size;
    hwaddr grf_base;
    uint32_t grf_size;
    hwaddr pmu_base;            /* 0: none (as for every base below) */
    uint32_t cru_srst_fst;       /* GLB_SRST_FST in the CRU (0: none) */
    hwaddr cru_base;
    uint32_t cru_size;
    const uint32_t *cru_lock;
    int n_cru_lock;
    const uint32_t *cru_init;
    int n_cru_init;
    const uint32_t *cru_frac;
    int n_cru_frac;
    hwaddr pmuioc_base;
    hwaddr ioc_base;
    uint32_t ioc_size;
    hwaddr uart_base[6];
    int uart_irq[6];
    int console_uart;
    uint32_t console_baud;
    hwaddr sfc_base;
    int sfc_irq;
    hwaddr otp_base;
    bool otp_v1126;             /* the RV1126's byte-wide OTP, + secure */
    hwaddr saradc_base;
    hwaddr tsadc_base;
    hwaddr crypto_base;
    int crypto_irq;
    uint32_t crypto_version;
    hwaddr decom_base;
    int decom_irq;
    uint32_t soc_id;            /* tag_soc_info.name: 0x1106 ... */
    hwaddr i2c_base[5];
    int i2c_irq[5];
    int sensor_bus;             /* the camera's I2C */
    const char *sensor_default; /* NULL: sc2336p at 0x30 */
    int i2c_rtc_bus;            /* a PCF8563 / BM8563 at 0x51, 0: none */
    int codec_bus;              /* an ES8311 at 0x18 (bus index + 1) */
    int dsi_bridge_bus;         /* an SN65DSI8x at 0x2c (bus index + 1) */
    hwaddr i2s_base;            /* I2S/TDM, paced through the PL330 */
    int i2s_irq;
    int i2s_drq[2];             /* PL330 peripheral requests: TX, RX */
    bool i2s_clk_rv1126;        /* its MCLK from the RV1126 CRU layout */
    bool i2s_clk_rv1106;        /* its MCLK from the RV1106 CRU layout */
    bool isp_v33;               /* the ISP is V33 (RV1103B) */
    hwaddr isp_base;            /* ISP32: frames in online mode */
    bool isp_readback;          /* ISP20: frames read back from memory */
    hwaddr vicap_base;          /* VICAP: the to-ISP frame interrupts */
    int vicap_irq;
    int isp_irq[3];             /* ISP, MI, MIPI */
    hwaddr cif_base;            /* RV1126 CIF: the MIPI capture frames */
    hwaddr ispp_base;           /* RV1126 ISPP: the camera's pictures */
    hwaddr rga_base;            /* RGA2 2D engine */
    hwaddr pwm_base[3];         /* PWM v2 blocks, four channels each */
    int pwm_irq[3];             /* 0: none (RV1126) */
    int rga_irq;
    int ispp_irq[2];            /* ISPP, FEC */
    int cif_irq;
    hwaddr npu_base;            /* RV1126 NPU (Vivante, galcore) */
    hwaddr rknpu_base;          /* RV1106 / RV1103B RKNPU */
    hwaddr vepu_pp_base;        /* RV1106 RKVENC-PP */
    hwaddr saradc_v1_base;      /* RV1126 SARADC v1 (rk3399's) */
    int saradc_v1_irq;
    int vepu_pp_irq;
    int rknpu_irq;
    int npu_irq;
    hwaddr vepu_base;           /* VEPU540C H.265 encoder */
    hwaddr hpmcu_base;          /* RV1103B HPMCU mailbox (wrap encoding) */
    hwaddr ddrc_base;           /* DW uMCTL2: the suspend code's handshake */
    uint32_t pmu_wake_mode;     /* PMU syscon: PWR_MODE_CON (bit 0 on) */
    uint32_t pmu_wake_timeout;  /* ... and its 32 kHz wake-up time-out */
    const uint32_t *pmu_plain;  /* PMU registers written as whole words */
    int n_pmu_plain;
    bool vepu541;               /* ... the RV1126's VEPU541 */
    hwaddr vepu2_base;          /* VEPU2: JPEG (RV1126) */
    int vepu2_irq;
    int vdpu2_irq;              /* VDPU2: JPEG decode, at vepu2_base + 0x400 */
    int vepu_irq;
    hwaddr gpio_base[5];
    int gpio_irq[5];
    hwaddr trng_base;
    int trng_irq;
    bool rkrng;                 /* the RV1103B's RNG register file */
    bool rng_v2;                /* the RV1126's, inside the crypto block */
    hwaddr rtc_base;
    int rtc_irq;
    hwaddr wdt_base;
    int wdt_irq;
    hwaddr dmac_base;
    int dmac_irq[8];            /* events 0..7 */
    int dmac_abort_irq;
    bool dmac_events_or;        /* all event IRQs share dmac_irq[0] */
    hwaddr gmac_base;
    int gmac_irq;
    uint8_t gmac_phy;           /* its PHY's MDIO address */
    uint8_t ws73_irq;           /* wifi=ws73: XM's board's interrupt pin,
                                 * a Linux GPIO number (syschannel.ko) */
    hwaddr mmc_base[3];         /* SD, SDIO, eMMC: -drive if=sd,index=n */
    int mmc_irq[3];
    hwaddr mbox_base;
    int mbox_irq;
    hwaddr rkdma_base;          /* the newer dies' DMA, in the PL330's place */
    int rkdma_irq;
    hwaddr sai_base;            /* the newer dies' audio interface */
    int num_cpus;               /* 0: one */
    uint32_t mpidr_base;        /* the devicetree's cpu reg: 0xf00 ... */
    bool psci;                  /* QEMU's PSCI stands in for OP-TEE's */
    bool spi_nand;              /* the SFC's flash is SPI NAND */
    bool boot_fit;              /* no ID block: load U-Boot from the FIT */
    hwaddr syscon_base[4];      /* more register files: PMUGRF, PMU ... */
    uint32_t syscon_size[4];
    hwaddr pmu_ctl_base;        /* the PMU's power domain handshakes */
    hwaddr nandc_base;          /* parallel NAND controller, no chip */
    hwaddr hotboot;             /* where the BootROM sends a core powered up */
    hwaddr ehci_base;           /* USB host: EHCI + companion OHCI */
    int ehci_irq;
    hwaddr ohci_base;
    int ohci_irq;
    hwaddr usbphy_base;         /* USB2 PHYs (naneng), calibration only */
    hwaddr dwc3_base;           /* USB OTG DWC3, host mode: its xHCI */
    int dwc3_irq;
    hwaddr vop_base;            /* display controller (VOP lite) */
    int vop_irq;
    hwaddr iommu_base[16];      /* IOMMU windows, 0x40 each */
    int iommu_irq[16];          /* 0: shares the previous window's line */
    hwaddr timer_base;          /* rk3288-type timer, the kernel's clockevent */
    int timer_irq[6];
    const uint32_t *pmu_mirror;
    int n_pmu_mirror;
} RkSoCConfig;

#define RV1106_COMMON                                                   \
    .soc_id         = 0x1106,                                           \
    .periph_base    = 0xFF000000,                                       \
    .periph_size    = 0xFFFF0000 - 0xFF000000,                          \
    .gic_base       = 0xFF1F0000,                                       \
    .sram_base      = 0xFF6C0000,                                       \
    .sram_size      = 256 * KiB,                                        \
    .grf_base       = 0xFF000000,                                       \
    .grf_size       = 0x68000,                                          \
    .pmu_base       = 0xFF300000,                                       \
    .cru_base       = 0xFF3A0000,                                       \
    .cru_srst_fst   = 0x10C08,      /* TOPCRU + 0xc08 */                \
    .cru_size       = 0x20000,                                          \
    .cru_lock       = rv1106_cru_lock,                                  \
    .n_cru_lock     = ARRAY_SIZE(rv1106_cru_lock),                      \
    .cru_init       = rv1106_cru_init,                                  \
    .n_cru_init     = ARRAY_SIZE(rv1106_cru_init),                      \
    .cru_frac       = rv1106_cru_frac,                                  \
    .n_cru_frac     = ARRAY_SIZE(rv1106_cru_frac),                      \
    .pmuioc_base    = 0xFF388000,                                       \
    .ioc_base       = 0xFF538000,                                       \
    .ioc_size       = 0x40000,                                          \
    .uart_base      = { 0xFF4A0000, 0xFF4B0000, 0xFF4C0000,             \
                        0xFF4D0000, 0xFF4E0000, 0xFF4F0000 },           \
    .uart_irq       = { 25, 26, 27, 28, 29, 30 },                       \
    .console_uart   = 2,                                                \
    .console_baud   = 115200,                                           \
    .sfc_base       = 0xFFAC0000,                                       \
    .sfc_irq        = 53,                                               \
    .otp_base       = 0xFF3D0000,                                       \
    .saradc_base    = 0xFF3C0000,                                       \
    .tsadc_base     = 0xFF3C8000,                                       \
    .crypto_base    = 0xFF440000,                                       \
    .pwm_base       = { 0xFF350000, 0xFF360000, 0xFF490000 },           \
    .dwc3_base      = 0xFFB00000,                                       \
    .dwc3_irq       = 54,                                               \
    .pwm_irq        = { 31, 33, 35 },                                   \
    .crypto_irq     = 15,                                               \
    .crypto_version = 0x03000000,                                       \
    .decom_base     = 0xFF520000,                                       \
    .decom_irq      = 112,                                              \
    .i2c_base       = { 0xFF310000, 0xFF320000, 0xFF450000,             \
                        0xFF460000, 0xFF470000 },                       \
    .i2c_irq        = { 18, 19, 20, 21, 22 },                           \
    .sensor_bus     = 4,                                                \
    .gpio_base      = { 0xFF380000, 0xFF530000, 0xFF540000,             \
                        0xFF550000, 0xFF560000 },                       \
    .gpio_irq       = { 5, 7, 9, 11, 13 },                              \
    .trng_base      = 0xFF448000,                                       \
    .trng_irq       = 17,                                               \
    .rtc_base       = 0xFF1C0000,                                       \
    .rtc_irq        = 98,                                               \
    .wdt_base       = 0xFF5A0000,                                       \
    .wdt_irq        = 46,                                               \
    .dmac_base      = 0xFF420000,                                       \
    .dmac_irq       = { 63, 64, 65, 66, 67, 68, 69, 70 },               \
    .dmac_abort_irq = 79,                                               \
    .gmac_base      = 0xFFA80000,                                       \
    .gmac_irq       = 93,                                               \
    .gmac_phy       = 2,            /* the integrated 100M PHY */       \
    .mmc_base       = { 0xFFAA0000, 0xFF9A0000, 0xFFA90000 },           \
    .mmc_irq        = { 52, 49, 48 },                                   \
    .mbox_base      = 0xFF5C0000,                                       \
    .mbox_irq       = 1,                                                \
    .isp_base       = 0xFFA00000,                                       \
    .isp_irq        = { 105, 104, 103 },                                \
    .vicap_base     = 0xFFA10000,                                       \
    .vicap_irq      = 106,                                              \
    .vepu_base      = 0xFFA50000,                                       \
    .vepu_irq       = 107,                                              \
    .i2s_base       = 0xFFAE0000,                                       \
    .i2s_clk_rv1106 = true,                                             \
    .i2s_irq        = 80,                                               \
    .i2s_drq        = { 22, 21 },                                       \
    .rga_base       = 0xFF980000,   /* "rockchip,rga2_core0" */          \
    .rga_irq        = 87,                                               \
    .rknpu_base     = 0xFF660000,   /* "rockchip,rv1106-rknpu" */        \
    .rknpu_irq      = 109,                                              \
    .vepu_pp_base   = 0xFFA60000,   /* "rockchip,rkvenc-pp-rv1106" */    \
    .vepu_pp_irq    = 108,                                              \
    .ws73_irq       = 70,           /* G2: GPIO2_A6 */                  \
    .ddrc_base      = 0xFF800000,                                       \
    .pmu_wake_mode  = 0x04,         /* the kernel's suspend code */     \
    .pmu_wake_timeout = 0x48        /* XM's AOV suspend time x 32.768 */

static const RkSoCConfig rv1103_soc = {
    .desc = "Rockchip RV1103 (Cortex-A7, 64 MiB DDR2)",
    .ram_size_default = 64 * MiB,
    RV1106_COMMON,
};

static const RkSoCConfig rv1106_soc = {
    .desc = "Rockchip RV1106 / RV1106G2 (Cortex-A7, 128 MiB DDR3)",
    .ram_size_default = 128 * MiB,
    RV1106_COMMON,
};

/*
 * RV1103B / RV1106B: a newer die with the peripherals at 0x20000000
 * (the vendor kernel's rv1103b.dtsi, develop-5.10, and U-Boot's own
 * devicetree in the XM "RV1103" dump).  The same IP blocks as the
 * RV1106 elsewhere, except crypto v4, the "rkrng" RNG, a Rockchip DMA
 * controller in the PL330's place (hw/dma/rk-dmac.c) and the FSPI, which is
 * the SFC by another name.  The boot ROM contract is the RV1106's:
 * IRAM at 0x210F6000 (boot source at +0x10), ATAGs at 2M - 8K.
 */
/* the RV1103B PMU's time-out and the counters around it: whole words */
static const uint32_t rv1103b_pmu_plain[] = {
    0x4080, 0x4084, 0x4088, 0x408c, 0x4090, 0x4094, 0x4098, 0x409c,
};

static const RkSoCConfig rv1103b_soc = {
    .desc = "Rockchip RV1103B / RV1106B (Cortex-A7, 64 MiB DDR2)",
    .ram_size_default = 64 * MiB,
    .soc_id         = 0x1103,
    .ws73_irq       = 68,           /* XM's board: GPIO2_A4 */
    .periph_base    = 0x20000000,
    .periph_size    = 0x01200000,
    .gic_base       = 0x20410000,
    .sram_base      = 0x210F6000,
    .sram_size      = 0xA000,
    .grf_base       = 0x20100000,
    .grf_size       = 0x61000,
    .cru_base       = 0x20000000,
    .cru_size       = 0x81000,
    .cru_srst_fst   = 0x60C08,      /* TOPCRU + 0xc08 */
    .cru_lock       = rv1103b_cru_lock,
    .n_cru_lock     = ARRAY_SIZE(rv1103b_cru_lock),
    .cru_init       = rv1103b_cru_init,
    .n_cru_init     = ARRAY_SIZE(rv1103b_cru_init),
    .cru_frac       = rv1103b_cru_frac,
    .n_cru_frac     = ARRAY_SIZE(rv1103b_cru_frac),
    .ioc_base       = 0x20170000,
    .ioc_size       = 0x60000,
    .uart_base      = { 0x20540000, 0x20870000, 0x20880000 },
    .uart_irq       = { 69, 70, 71 },
    .console_uart   = 0,
    .console_baud   = 1500000,
    .sfc_base       = 0x20D40000,
    .sfc_irq        = 52,
    .otp_base       = 0x20820000,           /* the rk3568 OTPC flow */
    .saradc_base    = 0x209A0000,
    .tsadc_base     = 0x209B0000,
    .crypto_base    = 0x20680000,
    .crypto_irq     = 47,
    .crypto_version = 0x04000000,
    .decom_base     = 0x209F0000,
    .decom_irq      = 104,
    .i2c_base       = { 0x20530000, 0x20910000, 0x20920000,
                        0x20930000, 0x20940000 },
    .i2c_irq        = { 63, 64, 65, 66, 67 },
    .sensor_bus     = 4,
    /* the devicetree's gpio1 is 0x20D80000 and gpio2 0x20840000 */
    .gpio_base      = { 0x20520000, 0x20D80000, 0x20840000 },
    .gpio_irq       = { 75, 79, 83 },
    .trng_base      = 0x20690000,
    .trng_irq       = 46,
    .rkrng          = true,
    .rtc_base       = 0x20750000,
    .rtc_irq        = 49,
    .wdt_base       = 0x208D0000,
    .wdt_irq        = 29,
    .gmac_base      = 0x20800000,
    .gmac_irq       = 22,
    .gmac_phy       = 2,
    .mmc_base       = { 0x20D20000, 0x20650000, 0x20D30000 },
    .mmc_irq        = { 42, 44, 51 },
    .rkdma_base     = 0x20740000,
    .rkdma_irq      = 0,
    .sai_base       = 0x20660000,
    /* "rockchip,rv1103b-rkisp" (ISP V33), "rv1103b-cif", "rga2" */
    .isp_base       = 0x20D00000,
    .isp_irq        = { 60, 59, 58 },
    .isp_v33        = true,
    .vicap_base     = 0x20D10000,
    .vicap_irq      = 57,
    .rga_base       = 0x20640000,
    .rga_irq        = 48,
    .rknpu_base     = 0x20F00000,           /* "rockchip,rv1103b-rknpu" */
    .rknpu_irq      = 50,
    /* "rockchip,rkv-encoder-rv1103b" (VEPU500): the VEPU540C's layout */
    .vepu_base      = 0x20E80000,
    .vepu_irq       = 62,
    .hpmcu_base     = 0x20A10000,           /* rockit's vmcu, mailbox */
    .ddrc_base      = 0x20C00000,
    .pmu_base       = 0x20510000,
    .pmu_wake_mode  = 0x4004,       /* PMU1 PWR_MODE_CON */
    .pmu_wake_timeout = 0x4094,
    .pmu_plain      = rv1103b_pmu_plain,
    .n_pmu_plain    = ARRAY_SIZE(rv1103b_pmu_plain),
};

/*
 * RV1126 / RV1109: four (two) Cortex-A7s, the peripherals from
 * 0xFE000000 (the vendor kernel's rv1126.dtsi, develop-4.19).  The boot
 * chain is DDR blob / TPL, SPL, OP-TEE, U-Boot; the one dump at hand
 * (a SPI NAND camera) lost its first 3 MiB, the ID block with every
 * loader before U-Boot, so the stand-in starts where the SPL would hand
 * over (see rk_fit_setup): OP-TEE in the secure world, which returns to
 * U-Boot non-secure.  The second core stays off: OP-TEE's PSCI powers
 * cores up through the PMU, which is not modelled.
 */
#define RV1126_COMMON                                                   \
    .soc_id         = 0x1126,                                           \
    .mpidr_base     = 0xf00,                                            \
    .spi_nand       = true,                                             \
    .boot_fit       = true,                                             \
    .periph_base    = 0xFE000000,                                       \
    .periph_size    = 0xFFFF0000 - 0xFE000000,                          \
    .gic_base       = 0xFEFF0000,                                       \
    .sram_base      = 0xFF700000,                                       \
    .sram_size      = 0x20000,                                          \
    .grf_base       = 0xFE000000,                                       \
    .grf_size       = 0x20000,                                          \
    .syscon_base    = { 0xFE020000 },                   /* PMUGRF */    \
    .syscon_size    = { 0x10000 },                                      \
    .pmu_ctl_base   = 0xFF3E0000,                                       \
    .nandc_base     = 0xFFC80000,                                       \
    .hotboot        = 0xFF710000,                                       \
    .ehci_base      = 0xFFE00000,                                       \
    .ehci_irq       = 82,                                               \
    .ohci_base      = 0xFFE10000,                                       \
    .ohci_irq       = 83,                                               \
    .usbphy_base    = 0xFF4C0000,                                       \
    .dwc3_base      = 0xFFD00000,   /* the OTG port */                  \
    .dwc3_irq       = 85,                                               \
    .iommu_base     = { 0xFFAE0800, 0xFFAE8800, 0xFFB00F00, 0xFFB20800, \
                        0xFFB51A00, 0xFFB60E00, 0xFFB60E40, 0xFFB60F00, \
                        0xFFB80480, 0xFFB804C0, 0xFFB90800, 0xFFBB0F00, \
                        0xFFBB0F40 },                                   \
    /* the CIF's and the VOP's IOMMUs share SPIs 58 / 59 with their     \
     * masters, and raise nothing */                                    \
    .iommu_irq      = { 0, 53, 0, 114, 51, 65, 66, 67, 72, 0, 75,      \
                        69, 70 },                                       \
    .cif_base       = 0xFFAE0000,                                       \
    .isp_base       = 0xFFB50000,   /* ISP20, its IOMMU at +0x1a00 */   \
    .isp_irq        = { 52, 54, 55 },                                   \
    .isp_readback   = true,                                             \
    .ispp_base      = 0xFFB60000,   /* its IOMMUs at +0xe00.. */        \
    .ispp_irq       = { 63, 64 },                                       \
    .rga_base       = 0xFFAF0000,                                       \
    .pwm_base       = { 0xFF430000, 0xFF440000, 0xFF550000 },           \
    .rga_irq        = 62,                                               \
    .cif_irq        = 58,                                               \
    .vepu_base      = 0xFFBB0000,   /* VEPU541, its IOMMU at +0xf00 */  \
    .vepu_irq       = 68,                                               \
    .vepu541        = true,                                             \
    .vepu2_base     = 0xFFB90000,   /* its IOMMU at +0x800 */           \
    .vepu2_irq      = 74,                                               \
    .vdpu2_irq      = 73,                                               \
    .npu_base       = 0xFFBC0000,                                       \
    .saradc_v1_base = 0xFF5E0000,                                       \
    .saradc_v1_irq  = 40,                                               \
    .npu_irq        = 107,                                              \
    .vop_base       = 0xFFB00000,                                       \
    .vop_irq        = 59,                                               \
    .timer_base     = 0xFF660000,                                       \
    .dmac_base      = 0xFF4E0000,   /* PL330, SPI 1 and 2 */            \
    .dmac_irq       = { 1 },                                            \
    .dmac_events_or = true,                                             \
    .dmac_abort_irq = 2,                                                \
    .timer_irq      = { 24 },   /* the one channel the dtsi names */   \
    .pmu_mirror     = rv1126_pmu_mirror,                                \
    .n_pmu_mirror   = ARRAY_SIZE(rv1126_pmu_mirror),                    \
    .cru_base       = 0xFF480000,                                       \
    .cru_size       = 0x20000,                                          \
    .cru_srst_fst   = 0x10408,      /* CRU (0xff490000) + 0x408 */      \
    .cru_lock       = rv1126_cru_lock,                                  \
    .n_cru_lock     = ARRAY_SIZE(rv1126_cru_lock),                      \
    .cru_init       = rv1126_cru_init,                                  \
    .n_cru_init     = ARRAY_SIZE(rv1126_cru_init),                      \
    .cru_frac       = rv1126_cru_frac,                                  \
    .n_cru_frac     = ARRAY_SIZE(rv1126_cru_frac),                      \
    .uart_base      = { 0xFF560000, 0xFF410000, 0xFF570000,             \
                        0xFF580000, 0xFF590000, 0xFF5A0000 },           \
    .uart_irq       = { 12, 13, 14, 15, 16, 17 },                       \
    .console_uart   = 2,                                                \
    .console_baud   = 1500000,                                          \
    .sfc_base       = 0xFFC90000,                                       \
    .sfc_irq        = 80,                                               \
    .otp_base       = 0xFF5C0000,                                       \
    .otp_v1126      = true,                                             \
    .saradc_base    = 0xFF5E0000,                                       \
    .tsadc_base     = 0xFF5F0000,                                       \
    .crypto_base    = 0xFF500000,                                       \
    .crypto_irq     = 3,                                                \
    .crypto_version = 0x02000000,                                       \
    .trng_base      = 0xFF500400,                                       \
    .rng_v2         = true,                                             \
    .decom_base     = 0xFF6C0000,                                       \
    .decom_irq      = 81,                                               \
    .i2c_base       = { 0xFF3F0000, 0xFF510000, 0xFF400000,             \
                        0xFF520000, 0xFF530000 },                       \
    .i2c_irq        = { 4, 5, 6, 7, 8 },                                \
    .sensor_bus     = 1,                                                \
    .sensor_default = "imx415",     /* the RV1109 board's, at 0x1a */   \
    .i2c_rtc_bus    = 0 + 1,        /* BM8563 on I2C0 (bus index + 1) */\
    .codec_bus      = 4 + 1,        /* ES8311 on I2C4 */                \
    .dsi_bridge_bus = 3 + 1,        /* SN65DSI8x on I2C3 */             \
    .i2s_base       = 0xFF800000,                                       \
    .i2s_irq        = 46,                                               \
    .i2s_drq        = { 20, 19 },   /* the devicetree's dmas */         \
    .i2s_clk_rv1126 = true,                                             \
    .gpio_base      = { 0xFF460000, 0xFF620000, 0xFF630000,             \
                        0xFF640000, 0xFF650000 },                       \
    .gpio_irq       = { 34, 35, 36, 37, 38 },                           \
    .wdt_base       = 0xFF680000,                                       \
    .wdt_irq        = 32,                                               \
    .gmac_base      = 0xFFC40000,                                       \
    .gmac_irq       = 95,                                               \
    .gmac_phy       = 0,            /* the board's RMII PHY */          \
    .mmc_base       = { 0xFFC60000, 0xFFC70000, 0xFFC50000 },           \
    .mmc_irq        = { 76, 77, 78 }

static const RkSoCConfig rv1126_soc = {
    .desc = "Rockchip RV1126 (4x Cortex-A7, 1 GiB DDR3)",
    .ram_size_default = 1 * GiB,
    .num_cpus = 4,
    RV1126_COMMON,
};

static const RkSoCConfig rv1109_soc = {
    .desc = "Rockchip RV1109 (2x Cortex-A7, 1 GiB DDR3)",
    .ram_size_default = 1 * GiB,     /* the SDK's 4K camera profile: 512 MiB
                                     * leaves mediaserver to the OOM killer */
    .num_cpus = 2,
    RV1126_COMMON,
};

static const RkSoCConfig rv1106g3_soc = {
    .desc = "Rockchip RV1106G3 (Cortex-A7, 256 MiB DDR3)",
    .ram_size_default = 256 * MiB,
    RV1106_COMMON,
};


#define TYPE_RK_MACHINE MACHINE_TYPE_NAME("rockchip")
OBJECT_DECLARE_SIMPLE_TYPE(RkMachineState, RK_MACHINE)

struct RkMachineState {
    MachineState parent_obj;

    char *flash_file;
    char *flash_type;
    char *sensor;
    char *sensor2;
    char *usb_otg;              /* "host" (default) or "device" */
    char *gpio_high;            /* pins that idle high, "4C0/0A2" */
    uint32_t gpio_levels[5];    /* parsed gpio_high, per bank */
    char *tb_isp;               /* "ADDR:CAMS[:STRIDE]": the MCU's ISP heads */
    char *gc6160;               /* "CLK:DATA": the motor driver's GPIOs */
    char *wifi;                 /* the SDIO slot's Wi-Fi: "ws73" */
    int gc_clk, gc_data;        /* parsed, Linux GPIO numbers; -1 none */
    uint64_t tb_addr;
    uint32_t tb_cams, tb_stride;
    bool tb_off;                /* tb-isp=off: not even from the meta */
    uint32_t meta_cams;         /* rk_cam_num in the flash's meta */
    DeviceState *gic;
    DeviceState *pl330;
    DeviceState *cam_sensor;    /* the camera I2C bus's first sensor */
    DeviceState *cam_sensor2;   /* and the second, at 0x32, if any */
    DeviceState *iommu[16];     /* per RkSoCConfig.iommu_base */

    /* the PMU's core power control (rk_cpupwr_*) */
    MemoryRegion cpupwr;
    ARMCPU *cpu[4];
    int ncpu;
    hwaddr hotboot;
    uint32_t pwr_mode;          /* PMU 0x90: a 4-bit mode per core */
    uint32_t pwr_req;           /* PMU 0x94: power-down request per core */
    bool wfi_off[4];            /* off since a WFI in power-down mode */
    bool line[4][2];            /* GIC IRQ / FIQ to each core */
    qemu_irq cpu_line[4][2];
    qemu_irq *spi_in;           /* peripheral interrupts, watched for wake */
    DECLARE_BITMAP(spi_level, RK_NUM_SPI);
};

static char *rk_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->flash_file);
}

static void rk_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *rk_machine_get_flash_type(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->flash_type);
}

static char *rk_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->sensor);
}

static void rk_machine_set_sensor(Object *obj, const char *value,
                                  Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

static char *rk_machine_get_usb_otg(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->usb_otg ?: "host");
}

static void rk_machine_set_usb_otg(Object *obj, const char *value,
                                   Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    if (strcmp(value, "host") && strcmp(value, "device")) {
        error_setg(errp, "usb-otg: host or device, not '%s'", value);
        return;
    }
    g_free(s->usb_otg);
    s->usb_otg = g_strdup(value);
}

static char *rk_machine_get_gpio_high(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->gpio_high ?: "");
}

/*
 * "4C0/0A2": bank, port letter A..D, pin 0..7.  An input nothing drives
 * reads 0 in the model; a board pull-up (a reset button's, say) is listed
 * here instead.
 */
static void rk_machine_set_gpio_high(Object *obj, const char *value,
                                     Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);
    g_auto(GStrv) pins = g_strsplit(value, "/", -1);
    uint32_t lv[5] = { 0 };
    int i;

    for (i = 0; pins[i]; i++) {
        const char *p = pins[i];
        int c;

        if (!*p) {
            continue;
        }
        c = g_ascii_toupper(p[1]);
        if (strlen(p) != 3 || p[0] < '0' || p[0] > '4' ||
            c < 'A' || c > 'D' || p[2] < '0' || p[2] > '7') {
            error_setg(errp, "gpio-high: '%s' is not a pin like 4C0", p);
            return;
        }
        lv[p[0] - '0'] |= 1u << ((c - 'A') * 8 + p[2] - '0');
    }
    memcpy(s->gpio_levels, lv, sizeof(lv));
    g_free(s->gpio_high);
    s->gpio_high = g_strdup(value);
}

static char *rk_machine_get_wifi(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->wifi ?: "");
}

static void rk_machine_set_wifi(Object *obj, const char *value, Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    if (*value && strcmp(value, "ws73")) {
        error_setg(errp, "wifi: ws73 or nothing, not '%s'", value);
        return;
    }
    g_free(s->wifi);
    s->wifi = *value ? g_strdup(value) : NULL;
}

static char *rk_machine_get_gc6160(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->gc6160 ?: "");
}

/* "70:75": the clock and data GPIOs, as Linux numbers (bank * 32 + pin) */
static void rk_machine_set_gc6160(Object *obj, const char *value,
                                  Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);
    g_auto(GStrv) f = g_strsplit(value, ":", 2);
    uint64_t clk, data;

    if (!f[0] || !f[1] || qemu_strtou64(f[0], NULL, 0, &clk) ||
        qemu_strtou64(f[1], NULL, 0, &data) || clk >= 160 || data >= 160 ||
        clk == data) {
        error_setg(errp, "gc6160: CLK:DATA (Linux GPIO numbers), not '%s'",
                   value);
        return;
    }
    s->gc_clk = clk;
    s->gc_data = data;
    g_free(s->gc6160);
    s->gc6160 = g_strdup(value);
}

/*
 * The GC6160 pan / tilt motor and IR-cut driver of XM's PTZ boards
 * (XmMotor.ko, g_useI2c=3, gpio_pin=CLK,DATA): an I2C device bit-banged
 * on two GPIOs, the main chip at 0x26 and two sub chips at 0x20 / 0x22
 * (8-bit addresses).  Each is a 256-byte register file: the driver
 * reads back what it wrote ("ircut close fail, retry to close" without
 * it).  The lines idle high, the board's pull-ups.
 */
static void rk_machine_gc6160(RkMachineState *s, DeviceState **gpio,
                              int nbanks)
{
    DeviceState *br;
    I2CBus *bus;
    int cb = s->gc_clk / 32, cp = s->gc_clk % 32;
    int db = s->gc_data / 32, dp = s->gc_data % 32;

    if (!s->gc6160 || cb >= nbanks || db >= nbanks) {
        return;
    }
    br = qdev_new(TYPE_GPIO_I2C);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(br), &error_fatal);
    qdev_connect_gpio_out(gpio[cb], cp, qdev_get_gpio_in(br, 1));   /* SCL */
    qdev_connect_gpio_out(gpio[db], dp, qdev_get_gpio_in(br, 0));   /* SDA */
    qdev_connect_gpio_out(br, 0, qdev_get_gpio_in(gpio[db], dp));
    bus = I2C_BUS(qdev_get_child_bus(br, "i2c"));
    at24c_eeprom_init(bus, 0x26 >> 1, 256);
    at24c_eeprom_init(bus, 0x20 >> 1, 256);
    at24c_eeprom_init(bus, 0x22 >> 1, 256);
}

static void rk_machine_set_scene(Object *obj, const char *value,
                                 Error **errp)
{
    rk_scene_set_source(value, errp);
}

static char *rk_machine_get_tb_isp(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->tb_isp ?: "");
}

/*
 * "0x8b0000:2[:11245]": the ISP's reserved memory (its devicetree
 * memory-region), the cameras, and each camera's block size there --
 * sizeof(struct rkisp32_thunderboot_resmem_head) in the kernel's
 * layout, 11245 in XM's RV1106G2 kernel.
 */
static void rk_machine_set_tb_isp(Object *obj, const char *value,
                                  Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);
    g_auto(GStrv) f = g_strsplit(value, ":", 3);
    uint64_t addr, cams, stride = 11245;

    if (!strcmp(value, "off")) {
        s->tb_off = true;
        s->tb_cams = 0;
        g_free(s->tb_isp);
        s->tb_isp = g_strdup(value);
        return;
    }
    if (!f[0] || !f[1] || qemu_strtou64(f[0], NULL, 0, &addr) ||
        qemu_strtou64(f[1], NULL, 0, &cams) || !cams || cams > 4 ||
        (f[2] && (qemu_strtou64(f[2], NULL, 0, &stride) || stride < 100))) {
        error_setg(errp, "tb-isp: ADDR:CAMERAS[:STRIDE], not '%s'", value);
        return;
    }
    s->tb_addr = addr;
    s->tb_cams = cams;
    s->tb_stride = stride;
    g_free(s->tb_isp);
    s->tb_isp = g_strdup(value);
}

/*
 * The MCU's thunder-boot result for the ISP driver (rkisp_chk_tb_over):
 * per camera, at its block, struct rkisp_thunderboot_resmem_head's
 * enable, complete = RKISP_TB_OK (1), one frame, the size, the cameras
 * and its index.  The ISP parameters after the 100-byte head stay zero.
 * Copied in at every reset, as the MCU writes it before Linux runs.
 */
/*
 * Without tb-isp, the head's place comes from the flash's thunder-boot
 * meta partition (Rockchip's "RKMT", which SPL loads for the kernel): its
 * CMDL item's meta_load_addr + meta_part_size is where the ISP's reserved
 * memory follows it, and rk_cam_num the cameras.  The XM G2's says
 * 0x800000 + 0xb0000 and 2: tb-isp=0x8b0000:2.  Only a board with two
 * cameras or more needs it (one camera runs online), so only those get it.
 */
static void rk_machine_tb_from_meta(RkMachineState *s, const uint8_t *img,
                                    size_t len)
{
    uint64_t load = 0, size = 0, cams = 0;
    size_t o, i;

    if (!img) {
        return;
    }
    for (o = 0; o + 0x2000 <= MIN(len, 4 << 20); o += 0x1000) {
        if (memcmp(img + o, "RKMT", 4)) {
            continue;
        }
        for (i = o; i + 5 < MIN(len, o + 0x20000); i += 4) {
            g_autofree char *c = NULL;
            g_auto(GStrv) kv = NULL;
            int k;

            if (memcmp(img + i, "CMDL ", 5)) {
                continue;
            }
            c = g_strndup((const char *)img + i + 5,
                          MIN(1024, len - i - 5));
            kv = g_strsplit(c, " ", 0);
            for (k = 0; kv[k]; k++) {
                if (g_str_has_prefix(kv[k], "meta_load_addr=")) {
                    qemu_strtou64(kv[k] + 15, NULL, 0, &load);
                } else if (g_str_has_prefix(kv[k], "meta_part_size=")) {
                    qemu_strtou64(kv[k] + 15, NULL, 0, &size);
                } else if (g_str_has_prefix(kv[k], "rk_cam_num=")) {
                    qemu_strtou64(kv[k] + 11, NULL, 0, &cams);
                }
            }
            break;
        }
        break;
    }
    s->meta_cams = cams;
    if (!s->tb_cams && !s->tb_off && load && size && cams >= 2 && cams <= 4) {
        s->tb_addr = load + size;
        s->tb_cams = cams;
        s->tb_stride = 11245;
    }
}

#define TB_AWB_GAIN     (100 + 4645)
#define TB_MODULES      0x7fc0020ull   /* ISP32_MODULE_RAWAE0..AWB_GAIN */

static void rk_machine_tb_isp(RkMachineState *s)
{
    g_autofree uint8_t *b = NULL;
    uint32_t i;

    if (!s->tb_cams) {
        return;
    }
    b = g_malloc0(s->tb_stride * s->tb_cams);
    for (i = 0; i < s->tb_cams; i++) {
        uint8_t *h = b + i * s->tb_stride;

        stw_le_p(h + 0, 1);                     /* enable */
        stw_le_p(h + 2, 1);                     /* complete: OK */
        stw_le_p(h + 4, 1);                     /* frm_total */
        stw_le_p(h + 10, 1920);
        stw_le_p(h + 12, 1080);
        stw_le_p(h + 14, s->tb_cams);           /* camera_num */
        stw_le_p(h + 16, i);                    /* camera_index */
        /* the MCU's last exposure, Q16: 10 ms at unity gain */
        stl_le_p(h + 20, 655);                  /* exp_time[0], s */
        stl_le_p(h + 32, 65536);                /* exp_gain[0] */
        stl_le_p(h + 68, 65536);                /* exp_isp_dgain[0] */
        if (s->tb_stride >= TB_AWB_GAIN + 32) {
            /*
             * The modules the MCU ran: the 3A statistics (RAWAE 0..3,
             * RAWHIST 0..3, RAWAWB) and the AWB gains, enabled -- rkaiq
             * takes no frame's statistics unless every block reports.
             */
            stq_le_p(h + 100, TB_MODULES);      /* module_en_update */
            stq_le_p(h + 108, TB_MODULES);      /* module_ens */
            /*
             * The ISP parameters' white balance (isp32_awb_gain_cfg at
             * 4645 into isp32_isp_params_cfg, after the head): rkaiq's
             * fast AWB starts from it.  AWB1 a daylight white (R 1.52,
             * B 1.71, Q8), the three AWB0 gains unity.
             */
            static const uint16_t g[16] = {
                0x100, 0x100, 0x1b7, 0x185,
                0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100, 0x100,
                0x100, 0x100, 0x100, 0x100,
            };
            int k;

            for (k = 0; k < 16; k++) {
                stw_le_p(h + TB_AWB_GAIN + 2 * k, g[k]);
            }
        }
    }
    rom_add_blob_fixed("rockchip.tb-isp", b, s->tb_stride * s->tb_cams,
                       s->tb_addr);
}

static char *rk_machine_get_sensor2(Object *obj, Error **errp)
{
    return g_strdup(RK_MACHINE(obj)->sensor2);
}

static void rk_machine_set_sensor2(Object *obj, const char *value,
                                   Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    g_free(s->sensor2);
    s->sensor2 = g_strdup(value);
}

static void rk_machine_set_flash_type(Object *obj, const char *value,
                                      Error **errp)
{
    RkMachineState *s = RK_MACHINE(obj);

    g_free(s->flash_type);
    s->flash_type = g_strdup(value);
}

static qemu_irq rk_irq(RkMachineState *s, int spi)
{
    if (s->spi_in) {
        return s->spi_in[spi];      /* through the PMU's wake filter */
    }
    return qdev_get_gpio_in(s->gic, spi);
}

static void rk_prop_list(DeviceState *dev, const char *name,
                         const uint32_t *v, int n)
{
    QList *l;
    int i;

    if (!n) {
        return;
    }
    l = qlist_new();
    for (i = 0; i < n; i++) {
        qlist_append_int(l, v[i]);
    }
    qdev_prop_set_array(dev, name, l);
}

/* An unrealized rk-syscon: PLL LOCK registers and reset values set */
static DeviceState *rk_syscon_new(uint32_t size,
                                  const uint32_t *lock, int nlock,
                                  const uint32_t *init, int ninit)
{
    DeviceState *dev = qdev_new("rk-syscon");

    qdev_prop_set_uint32(dev, "size", size);
    rk_prop_list(dev, "lock-regs", lock, nlock);
    rk_prop_list(dev, "init", init, ninit);
    return dev;
}

static DeviceState *rk_syscon(hwaddr base, uint32_t size,
                              const uint32_t *lock, int nlock,
                              const uint32_t *init, int ninit)
{
    DeviceState *dev = rk_syscon_new(size, lock, nlock, init, ninit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return dev;
}

static const char *rk_flash_type(gsize len)
{
    if (len > 16 * MiB) {
        return "w25q256";
    }
    if (len > 8 * MiB) {
        return "mx25l12805d";
    }
    return "w25q64";
}

static DeviceState *rk_create_flash(RkMachineState *s, BusState *bus,
                                    gsize img_len, bool spi_nand)
{
    DeviceState *dev;
    BlockBackend *blk = NULL;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    const char *type = s->flash_type;

    if (s->flash_file) {
        QDict *opts = qdict_new();
        Error *err = NULL;

        qdict_put_str(opts, "driver", "raw");
        blk = blk_new_open(s->flash_file, NULL, opts, BDRV_O_RDWR, &err);
        if (!blk) {
            error_reportf_err(err, "rockchip: %s: ", s->flash_file);
            exit(1);
        }
    } else if (di) {
        blk = blk_by_legacy_dinfo(di);
        img_len = blk_getlength(blk);
    }
    if (!type) {
        type = spi_nand ? "gd5f2gq4ub" : rk_flash_type(img_len);
    }
    dev = qdev_new(type);
    if (spi_nand) {
        /* a part of the generic SPI NAND model */
        assert(object_dynamic_cast(OBJECT(dev), "ingenic-spinand"));
        qdev_prop_set_uint32(dev, "dummy-clocks", 1);
    }
    if (blk) {
        qdev_prop_set_drive_err(dev, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(dev, bus, &error_fatal);
    if (s->flash_file) {
        blk_unref(blk);     /* the device holds its own (see fullhan.c) */
    }
    return dev;
}

/*
 * The DesignWare registers above the 16550 window (0x20..0xff).  The
 * vendor SPL and fiq-debugger poll USR (0x7c) for "TX FIFO not full /
 * empty" before every character; Linux's 8250_dw reads it on a busy
 * detect.  The FIFOs drain at once here, so TX always reads empty.
 */
#define DW_USR      0x7c
#define DW_TFL      0x80
#define DW_RFL      0x84

static uint64_t rk_uart_dw_read(void *opaque, hwaddr off, unsigned size)
{
    SerialMM *smm = opaque;
    bool rx = smm->serial.lsr & 0x01;       /* UART_LSR_DR */

    switch (off + 0x20) {
    case DW_USR:
        return 0x6 | (rx ? 0x8 : 0);        /* TFNF | TFE | RFNE */
    case DW_RFL:
        return rx ? 1 : 0;
    default:
        return 0;
    }
}

static void rk_uart_dw_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
}

static const MemoryRegionOps rk_uart_dw_ops = {
    .read = rk_uart_dw_read,
    .write = rk_uart_dw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void rk_create_uart(MemoryRegion *sysmem, hwaddr base, qemu_irq irq,
                           Chardev *chr)
{
    SerialMM *smm = serial_mm_init(sysmem, base, 2, irq, 24000000 / 16,
                                   chr, DEVICE_LITTLE_ENDIAN);
    MemoryRegion *dw = g_new(MemoryRegion, 1);

    memory_region_init_io(dw, OBJECT(smm), &rk_uart_dw_ops, smm,
                          "rockchip.uart-dw", 0xe0);
    memory_region_add_subregion(sysmem, base + 0x20, dw);
}

/* ---- mask ROM stand-in ---------------------------------------------- */

typedef struct RkAtags {
    uint32_t buf[256];
    int n;
} RkAtags;

static void rk_atag(RkAtags *a, uint32_t magic, const uint32_t *body,
                    int words)
{
    int i;

    /* header (size, magic), body, then a zero hash: "no hash, accept" */
    a->buf[a->n++] = cpu_to_le32(2 + words + (magic ? 1 : 0));
    a->buf[a->n++] = cpu_to_le32(magic);
    for (i = 0; i < words; i++) {
        a->buf[a->n++] = cpu_to_le32(body[i]);
    }
    if (magic) {
        a->buf[a->n++] = 0;
    }
}

static void rk_write_atags(const RkSoCConfig *c, uint64_t ram_size)
{
    RkAtags a = { 0 };
    uint32_t core[3] = { 0 };
    /* tag_serial: version, enable, addr (u64), baud, m_mode, id, rsv[2] */
    uint32_t serial[9] = {
        0, 1, c->uart_base[c->console_uart], 0, c->console_baud, 0,
        c->console_uart, 0, 0,
    };
    /*
     * tag_ddr_mem: count, version, bank[20] (u64), flags, data[2]; the
     * sizes follow the bases: bank[i] base, bank[count + i] size.
     */
    uint32_t ddr[2 + 40 + 3] = { 0 };
    /*
     * tag_soc_info: version, name, flags, rsv[6].  The RV1106 DDR blobs
     * (stock and thunder-boot builds alike) set SOC_FLAGS_TDBT.  A
     * thunder-boot SPL needs it to boot the kernel it preloads: without
     * it, it jumps to U-Boot instead, which the preloaded kernel (at
     * 0x208000) and the SPL's own buffers have already overwritten.
     */
    uint32_t soc[9] = { 1, c->soc_id, SOC_FLAGS_TDBT };

    ddr[0] = 1;
    ddr[2] = RK_DDR_BASE;
    ddr[2 + 2 * 1] = ram_size;
    rk_atag(&a, ATAG_CORE, core, 3);
    rk_atag(&a, ATAG_SERIAL, serial, 9);
    rk_atag(&a, ATAG_DDR_MEM, ddr, ARRAY_SIZE(ddr));
    rk_atag(&a, ATAG_SOC_INFO, soc, ARRAY_SIZE(soc));
    a.buf[a.n++] = 0;               /* ATAG_NONE: size 0 */
    a.buf[a.n++] = cpu_to_le32(ATAG_NONE);
    rom_add_blob_fixed("rockchip.atags", a.buf, a.n * 4, RK_ATAGS_BASE);
}

/*
 * The ID block: "RKNS", then size_and_nimage (image count in bits 23:16),
 * boot_flag, and from 0x78 up to four 0x58-byte image entries { size_and_off
 * (size << 16 | offset, 512-byte sectors from the header), address, flag,
 * counter } (U-Boot tools/rkcommon.c, struct header0_info_v2).
 *
 * Images with address 0xffffffff run, in order: the first is the DDR init
 * blob (skipped: DRAM needs no training), the next the SPL, run at DRAM
 * 0.  An image with an address is data the ROM copies there: thunder-boot
 * IDBs carry the ISP's fast-AE tables this way (flag 0x10007, at 0x40000).
 */
static const gsize rk_idb_probe[] = { 0x0, 0x10000, 0x8000, 0x20000, 0x40000 };

/*
 * The older ID block (header v1, the RV1126's and RK3399's, U-Boot
 * tools/rkcommon.c struct header0_info): sector 0 is RC4-scrambled with a
 * fixed key and holds magic 0x0ff0aa55, init_offset (0x0c, sectors),
 * init_size (0x1fa) and init_boot_size (0x1fc, both sectors from
 * init_offset).  The DDR blob is [init_offset, + init_size), the SPL the
 * rest up to init_boot_size; their data is plain when disable_rc4 (0x08)
 * is set, as mkimage's rksd writes it.
 */
static void rk_rc4(uint8_t *buf, size_t len)
{
    static const uint8_t key[16] = {
        124, 78, 3, 4, 85, 5, 9, 7, 45, 44, 123, 56, 23, 13, 23, 17
    };
    uint8_t st[256], t;
    unsigned i, j = 0, k = 0;

    for (i = 0; i < 256; i++) {
        st[i] = i;
    }
    for (i = 0; i < 256; i++) {
        j = (j + st[i] + key[i % 16]) & 0xff;
        t = st[i]; st[i] = st[j]; st[j] = t;
    }
    for (i = j = 0; k < len; k++) {
        i = (i + 1) & 0xff;
        j = (j + st[i]) & 0xff;
        t = st[i]; st[i] = st[j]; st[j] = t;
        buf[k] ^= st[(st[i] + st[j]) & 0xff];
    }
}

static bool rk_boot_rom_setup_v1(const uint8_t *img, gsize len,
                                 const RkSoCConfig *c, uint64_t ram_size)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(rk_idb_probe); i++) {
        gsize hdr = rk_idb_probe[i];
        uint8_t h[512];
        uint32_t off, init, total, src, size;

        if (hdr + 512 > len) {
            continue;
        }
        memcpy(h, img + hdr, 512);
        rk_rc4(h, 512);
        if (ldl_le_p(h) != 0x0ff0aa55) {
            continue;
        }
        if (!ldl_le_p(h + 8)) {
            error_report("rockchip: v1 ID block at 0x%zx has RC4-scrambled "
                         "images", hdr);
            return false;
        }
        off = lduw_le_p(h + 0x0c);
        init = lduw_le_p(h + 0x1fa);
        total = lduw_le_p(h + 0x1fc);
        src = hdr + (off + init) * 512;
        size = total > init ? (total - init) * 512 : 0;
        if (!size || src >= len) {
            error_report("rockchip: v1 ID block at 0x%zx has no SPL", hdr);
            return false;
        }
        size = MIN(size, len - src);
        rom_add_blob_fixed("rockchip.idb1", img + src, size, RK_DDR_BASE);
        rk_write_atags(c, ram_size);
        {
            uint32_t id = cpu_to_le32(BROM_BOOT_SPINOR);

            rom_add_blob_fixed("rockchip.bootsource", &id, 4,
                               c->sram_base + 0x10);
        }
        return true;
    }
    return false;
}

/* An ID block, either kind, at one of the places the ROM looks */
static bool rk_has_idb(const uint8_t *img, gsize len)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(rk_idb_probe); i++) {
        gsize hdr = rk_idb_probe[i];
        uint8_t h[4];

        if (hdr + 512 > len) {
            continue;
        }
        if (!memcmp(img + hdr, "RKNS", 4)) {
            return true;
        }
        memcpy(h, img + hdr, 4);
        rk_rc4(h, 4);
        if (ldl_le_p(h) == 0x0ff0aa55) {
            return true;
        }
    }
    return false;
}

static bool rk_boot_rom_setup(const uint8_t *img, gsize len,
                              const RkSoCConfig *c, uint64_t ram_size)
{
    const gsize *probe = rk_idb_probe;
    gsize hdr = 0;
    uint32_t nimg, n, runs = 0;
    bool spl = false;
    int i;

    for (i = 0; i < ARRAY_SIZE(rk_idb_probe); i++) {
        if (probe[i] + 0x800 <= len && !memcmp(img + probe[i], "RKNS", 4)) {
            hdr = probe[i];
            break;
        }
    }
    if (i == ARRAY_SIZE(rk_idb_probe)) {
        if (rk_boot_rom_setup_v1(img, len, c, ram_size)) {
            return true;
        }
        error_report("rockchip: no ID block in the flash image");
        return false;
    }
    nimg = MIN((ldl_le_p(img + hdr + 8) >> 16) & 0xff, 4);
    for (n = 0; n < nimg && !spl; n++) {
        const uint8_t *e = img + hdr + 0x78 + 0x58 * n;
        uint32_t so = ldl_le_p(e), addr = ldl_le_p(e + 4);
        uint32_t src = hdr + (so & 0xffff) * 512, size = (so >> 16) * 512;
        g_autofree char *name = g_strdup_printf("rockchip.idb%u", n);

        if (!size || src + size > len) {
            error_report("rockchip: ID block image %u (0x%x bytes at 0x%x) "
                         "is outside the flash image", n, size, src);
            return false;
        }
        if (addr != 0xffffffff) {
            rom_add_blob_fixed(name, img + src, size, addr);
        } else if (runs++) {
            rom_add_blob_fixed(name, img + src, size, RK_DDR_BASE);
            spl = true;
        }
    }
    if (!spl) {
        error_report("rockchip: ID block at 0x%zx has no SPL after the DDR "
                     "init image", hdr);
        return false;
    }
    rk_write_atags(c, ram_size);
    {
        uint32_t id = cpu_to_le32(BROM_BOOT_SPINOR);

        rom_add_blob_fixed("rockchip.bootsource", &id, 4, c->sram_base + 0x10);
    }
    return true;
}

/*
 * The RV1126 PMU's core power control, as OP-TEE's PSCI drives it
 * (plat-rockchip psci: cpu_power_domain_ctr, check_cpu_wfie):
 *
 *   0x90  a 4-bit power mode per core (hiword); bit 0 of a core's field
 *         set: the core loses power when it executes WFI -- CPU_SUSPEND
 *         and CPU_OFF end in "dsb; wfi; b ." on that promise
 *   0x94  power-down request, a bit per core (hiword): 1 off, 0 on
 *   0x9c  status: bits n and n + 4 while core n is off
 *
 * A core powered up -- its request cleared, or an interrupt for a core
 * that went down at WFI -- comes out of reset where the BootROM sends a
 * warm or secondary core: OP-TEE's trampoline at "hotboot" (0xFF710000,
 * in the SRAM; OP-TEE copies it there with its parameters at +0x1fe0).
 * The PMU wakes a core that went down at WFI on an interrupt at the
 * distributor's inputs (OP-TEE turns the core's GIC CPU interface off
 * before it), so the peripheral interrupts pass a filter on their way to
 * the GIC, and so do the IRQ / FIQ lines to each core.
 */
static bool rk_cpu_is_off(ARMCPU *cpu)
{
    return cpu->power_state == PSCI_OFF;
}

static void rk_cpupwr_on(RkMachineState *s, int n)
{
    if (rk_cpu_is_off(s->cpu[n])) {
        s->wfi_off[n] = false;
        arm_set_cpu_on(s->cpu[n]->mp_affinity, s->hotboot, 0, 3, false);
    }
}

static void rk_cpupwr_wake_bh(void *opaque)
{
    RkMachineState *s = opaque;
    bool spi = !bitmap_empty(s->spi_level, RK_NUM_SPI);
    int n;

    for (n = 0; n < s->ncpu; n++) {
        if (s->wfi_off[n] && !(s->pwr_req & (1u << n)) &&
            (s->line[n][0] || s->line[n][1] || spi)) {
            rk_cpupwr_on(s, n);
        }
    }
}

/* A peripheral interrupt: every core down at WFI wakes */
static void rk_cpupwr_any_wake_bh(void *opaque)
{
    RkMachineState *s = opaque;
    int n;

    for (n = 0; n < s->ncpu; n++) {
        if (s->wfi_off[n] && !(s->pwr_req & (1u << n))) {
            rk_cpupwr_on(s, n);
        }
    }
}

static void rk_cpupwr_wfi(ARMCPU *cpu, void *opaque)
{
    RkMachineState *s = opaque;
    int n;

    for (n = 0; n < s->ncpu && s->cpu[n] != cpu; n++) {
    }
    if (n == s->ncpu ||
        !((s->pwr_mode >> (4 * n)) & 1 || (s->pwr_req >> n) & 1)) {
        return;                     /* an ordinary WFI */
    }
    /* power is cut: the WFI halts the core for good */
    cpu->power_state = PSCI_OFF;
    s->wfi_off[n] = true;
    if (s->line[n][0] || s->line[n][1] ||
        !bitmap_empty(s->spi_level, RK_NUM_SPI)) {
        /* an interrupt already waiting: the core comes straight back */
        aio_bh_schedule_oneshot(qemu_get_aio_context(), rk_cpupwr_wake_bh, s);
    }
}

static void rk_cpupwr_line(void *opaque, int irq, int level)
{
    RkMachineState *s = opaque;
    int n = irq / 2, fiq = irq & 1;

    s->line[n][fiq] = level;
    qemu_set_irq(s->cpu_line[n][fiq], level);
    if (level && s->wfi_off[n]) {
        rk_cpupwr_wake_bh(s);
    }
}

static void rk_cpupwr_spi(void *opaque, int spi, int level)
{
    RkMachineState *s = opaque;
    int n;

    qemu_set_irq(qdev_get_gpio_in(s->gic, spi), level);
    if (level) {
        set_bit(spi, s->spi_level);
    } else {
        clear_bit(spi, s->spi_level);
    }
    if (level) {
        for (n = 0; n < s->ncpu; n++) {
            if (s->wfi_off[n] && !(s->pwr_req & (1u << n))) {
                aio_bh_schedule_oneshot(qemu_get_aio_context(),
                                        rk_cpupwr_any_wake_bh, s);
                break;
            }
        }
    }
}

static uint64_t rk_cpupwr_read(void *opaque, hwaddr off, unsigned size)
{
    RkMachineState *s = opaque;
    uint32_t st = 0;
    int n;

    switch (off) {
    case 0x0:
        return s->pwr_mode;
    case 0x4:
        return s->pwr_req;
    case 0xc:
        for (n = 0; n < 4; n++) {
            if (n >= s->ncpu || rk_cpu_is_off(s->cpu[n])) {
                st |= (1u << n) | (1u << (n + 4));
            }
        }
        return st;
    }
    return 0;
}

static void rk_cpupwr_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    RkMachineState *s = opaque;
    uint32_t mask = val >> 16;
    int n;

    switch (off) {
    case 0x0:
        s->pwr_mode = (s->pwr_mode & ~mask) | (val & mask);
        break;
    case 0x4: {
        uint32_t old = s->pwr_req;

        s->pwr_req = (s->pwr_req & ~mask) | (val & mask);
        (void)old;
        for (n = 0; n < s->ncpu; n++) {
            /* every write acts, whatever was stored before */
            if ((mask >> n) & 1) {
                if (s->pwr_req & (1u << n)) {
                    /* another core going off now; the caller at its WFI */
                    if (s->cpu[n] != ARM_CPU(current_cpu) &&
                        !rk_cpu_is_off(s->cpu[n])) {
                        arm_set_cpu_off(s->cpu[n]->mp_affinity);
                    }
                } else {
                    rk_cpupwr_on(s, n);
                }
            }
        }
        break;
    }
    }
}

static const MemoryRegionOps rk_cpupwr_ops = {
    .read = rk_cpupwr_read,
    .write = rk_cpupwr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * The SPL stand-in for a flash whose ID block is gone (the RV1109 dump):
 * the SPL's job from U-Boot on.  It finds Rockchip's uboot.img -- a FIT
 * on a 128 KiB boundary in the first 16 MiB with an "os = U-Boot" image
 * -- and loads U-Boot at its load address with the "flat_dt" image right
 * after it, where U-Boot's OF_SEPARATE looks (spl_fit_append_fdt does
 * the same).  With an "os = op-tee" image it loads that too and a stub
 * at DRAM 0 enters it as the SPL's spl_optee_entry does -- secure SVC,
 * r1 = lr = U-Boot, r2 = U-Boot's devicetree -- and OP-TEE returns to U-Boot
 * non-secure; without one the stub jumps to U-Boot.  The ATAGs and the
 * boot source (SPI NAND) are what the SPL leaves.
 */
static bool rk_fit_setup(const uint8_t *img, gsize len,
                         const RkSoCConfig *c, uint64_t ram_size)
{
    gsize off;

    for (off = 0; off + 0x1000 <= MIN(len, 16 * MiB); off += 0x20000) {
        const void *fit = img + off;
        uint32_t ext, load = 0, size = 0, dt_size = 0;
        uint32_t tee_load = 0, tee_entry = 0, tee_size = 0;
        const uint8_t *data = NULL, *dt = NULL, *tee = NULL;
        int images, node;

        if (fdt_magic(fit) != FDT_MAGIC ||
            fdt_totalsize(fit) > len - off ||
            (images = fdt_path_offset(fit, "/images")) < 0) {
            continue;
        }
        ext = off + ((fdt_totalsize(fit) + 3) & ~3);
        fdt_for_each_subnode(node, fit, images) {
            const char *os = fdt_getprop(fit, node, "os", NULL);
            const char *type = fdt_getprop(fit, node, "type", NULL);
            const fdt32_t *pos = fdt_getprop(fit, node, "data-position",
                                             NULL);
            const fdt32_t *rel = fdt_getprop(fit, node, "data-offset", NULL);
            const fdt32_t *sz = fdt_getprop(fit, node, "data-size", NULL);
            const fdt32_t *ld = fdt_getprop(fit, node, "load", NULL);
            uint64_t at;

            if (!sz || (!pos && !rel)) {
                continue;
            }
            at = pos ? off + fdt32_to_cpu(*pos) : ext + fdt32_to_cpu(*rel);
            if (at + fdt32_to_cpu(*sz) > len) {
                continue;
            }
            if (os && !strcmp(os, "U-Boot") && ld) {
                data = img + at;
                size = fdt32_to_cpu(*sz);
                load = fdt32_to_cpu(*ld);
            } else if (os && !strcmp(os, "op-tee") && ld) {
                const fdt32_t *ep = fdt_getprop(fit, node, "entry", NULL);

                tee = img + at;
                tee_size = fdt32_to_cpu(*sz);
                tee_load = fdt32_to_cpu(*ld);
                tee_entry = ep ? fdt32_to_cpu(*ep) : tee_load;
            } else if (type && !strcmp(type, "flat_dt")) {
                dt = img + at;
                dt_size = fdt32_to_cpu(*sz);
            }
        }
        if (!data) {
            continue;
        }
        {
            /* U-Boot's spl_optee_entry: r1 = lr = U-Boot, r2 = dtb */
            uint32_t stub[9] = {
                cpu_to_le32(0xe59f1010),    /* ldr r1, [pc, #16]: U-Boot */
                cpu_to_le32(0xe1a0e001),    /* mov lr, r1 */
                cpu_to_le32(0xe3a00000),    /* mov r0, #0 */
                cpu_to_le32(0xe59f2008),    /* ldr r2, [pc, #8]: its dtb */
                cpu_to_le32(0xe59ff008),    /* ldr pc, [pc, #8] */
                cpu_to_le32(0xe1a00000),    /* nop */
                cpu_to_le32(load),
                cpu_to_le32(dt ? load + size : 0),
                cpu_to_le32(tee ? tee_entry : load),
            };
            uint32_t id = cpu_to_le32(BROM_BOOT_SPINAND);

            rom_add_blob_fixed("rockchip.u-boot", data, size, load);
            if (dt) {
                rom_add_blob_fixed("rockchip.u-boot-dtb", dt, dt_size,
                                   load + size);
            }
            if (tee) {
                rom_add_blob_fixed("rockchip.optee", tee, tee_size,
                                   tee_load);
            }
            rom_add_blob_fixed("rockchip.stub", stub, sizeof(stub),
                               RK_DDR_BASE);
            rk_write_atags(c, ram_size);
            rom_add_blob_fixed("rockchip.bootsource", &id, 4,
                               c->sram_base + 0x10);
        }
        return true;
    }
    error_report("rockchip: no U-Boot FIT (uboot.img) in the flash image");
    return false;
}

/*
 * The devices past the boot path: what the vendor kernel and the camera
 * application drive once Linux is up.
 */
static void rk_create_pl330(RkMachineState *s, const RkSoCConfig *c);
static void rk_create_tail(RkMachineState *s, const RkSoCConfig *c);

static void rk_create_more(RkMachineState *s, const RkSoCConfig *c)
{
    DeviceState *dev, *gpio[5] = { NULL }, *sai;
    SysBusDevice *sbd;
    I2CBus *sensor_bus = NULL, *bus[5] = { NULL };
    int i;

    for (i = 0; i < 5; i++) {
        g_autofree char *name = g_strdup_printf("i2c%d", i);

        dev = qdev_new("rk-i2c");
        object_property_add_child(OBJECT(s), name, OBJECT(dev));
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->i2c_base[i]);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->i2c_irq[i]));
        bus[i] = I2C_BUS(qdev_get_child_bus(dev, "i2c"));
        if (i == c->sensor_bus) {
            sensor_bus = bus[i];
        }
    }
    if (c->i2c_rtc_bus) {
        i2c_slave_create_simple(bus[c->i2c_rtc_bus - 1], "pcf8563", 0x51);
    }
    if (c->codec_bus) {
        i2c_slave_create_simple(bus[c->codec_bus - 1], "es8311", 0x18);
    }
    if (c->dsi_bridge_bus) {
        /*
         * The RV1109 board's TI SN65DSI8x MIPI DSI -> LVDS bridge for its
         * 800x480 panel, which /app/mipi_4lane_800x480.sh programs with
         * i2cset at boot (23 "Error: Write failed" with nothing there):
         * a register file, its ID at 0x00..0x08 ("DSI85" reversed, then
         * the revision).  No LVDS is produced.
         */
        static const uint8_t sn65dsi_id[] = {
            0x35, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20, 0x01,
        };

        at24c_eeprom_init_rom(bus[c->dsi_bridge_bus - 1], 0x2c, 256,
                              sn65dsi_id, sizeof(sn65dsi_id));
    }
    /*
     * The camera boards' sensors answer at 0x30 and, on two-sensor
     * boards, 0x32 (the vendor devicetrees list sc2336p and sc231hai
     * at both; the driver whose ID matches binds).
     */
    if (c->sensor_default) {
        /* at the catalogue's address for it */
        s->cam_sensor = image_sensor_create_at(sensor_bus,
                               s->sensor ? s->sensor : c->sensor_default, 0);
    } else {
        s->cam_sensor = image_sensor_create_at(sensor_bus,
                                    s->sensor ? s->sensor : "sc2336p", 0x30);
    }
    /* a second camera the meta speaks of: the first one's model */
    s->cam_sensor2 = image_sensor_create_at(sensor_bus,
                         s->sensor2 ? s->sensor2 :
                         s->meta_cams >= 2 ? (s->sensor ? s->sensor : "sc2336p")
                                           : "none", 0x32);

    rk_machine_tb_isp(s);

    /* /machine/gpioN: a test drives a pin with set_irq_in */
    if (s->gc6160) {                    /* its lines idle high */
        s->gpio_levels[s->gc_clk / 32] |= 1u << (s->gc_clk % 32);
        s->gpio_levels[s->gc_data / 32] |= 1u << (s->gc_data % 32);
    }
    for (i = 0; i < 5 && c->gpio_base[i]; i++) {
        g_autofree char *name = g_strdup_printf("gpio%d", i);

        dev = qdev_new("rk-gpio");
        gpio[i] = dev;
        qdev_prop_set_uint32(dev, "input-levels", s->gpio_levels[i]);
        object_property_add_child(OBJECT(s), name, OBJECT(dev));
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->gpio_base[i]);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->gpio_irq[i]));
    }
    rk_machine_gc6160(s, gpio, i);

    if (c->trng_base) {
        dev = qdev_new("rk-trng");
        qdev_prop_set_bit(dev, "rkrng", c->rkrng);
        qdev_prop_set_bit(dev, "crypto-v2", c->rng_v2);
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        /* over the crypto model where it sits inside the crypto block */
        sysbus_mmio_map_overlap(sbd, 0, c->trng_base, 1);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->trng_irq));
    }
    if (c->rtc_base) {
        sysbus_create_simple("rk-rtc", c->rtc_base, rk_irq(s, c->rtc_irq));
    }
    if (c->ddrc_base) {
        sysbus_create_simple("rk-ddrc", c->ddrc_base, NULL);
    }


    /* DW APB watchdog; the kernel's dw_wdt reads its clock as 24 MHz */
    dev = qdev_new("fh-wdt");
    qdev_prop_set_uint32(dev, "freq", 24000000);
    qdev_prop_set_uint32(dev, "comp-params-1", 0x1042);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->wdt_base);
    sysbus_connect_irq(sbd, 0, rk_irq(s, c->wdt_irq));

    if (c->dmac_base) {
        rk_create_pl330(s, c);
    }
    sai = NULL;
    if (c->sai_base) {
        /* its samples move through the RKDMA, at its own rate */
        sai = qdev_new("rk-sai");
        qdev_prop_set_uint64(sai, "cru-base", c->cru_base);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sai), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sai), 0, c->sai_base);
    }
    if (c->rkdma_base) {
        dev = qdev_new("rk-dmac");
        if (sai) {
            object_property_set_link(OBJECT(dev), "pace", OBJECT(sai),
                                     &error_fatal);
            qdev_prop_set_uint64(dev, "pace-base", c->sai_base);
            qdev_prop_set_uint64(dev, "pace-size", 0x1000);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->rkdma_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, rk_irq(s, c->rkdma_irq));
    }
    if (c->mbox_base) {
        /*
         * The mailbox to the RISC-V MCU, with the stand-in for its
         * firmware's "done" on channel 1, which thunder-boot kernels
         * wait for.
         */
        dev = qdev_new("rk-mailbox");
        qdev_prop_set_int32(dev, "mcu-done-chan", 1);
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->mbox_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->mbox_irq));
    }
    rk_create_tail(s, c);
}

/* PL330: 8 channels, 32 peripheral requests, an IRQ per event */
static void rk_create_pl330(RkMachineState *s, const RkSoCConfig *c)
{
    DeviceState *dev;
    SysBusDevice *sbd;
    int i;

    dev = s->pl330 = qdev_new("pl330");
    object_property_set_link(OBJECT(dev), "memory",
                             OBJECT(get_system_memory()), &error_fatal);
    qdev_prop_set_uint32(dev, "num_chnls", 8);
    qdev_prop_set_uint8(dev, "num_periph_req", 32);
    qdev_prop_set_uint8(dev, "num_events", 8);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->dmac_base);
    sysbus_connect_irq(sbd, 0, rk_irq(s, c->dmac_abort_irq));
    if (c->dmac_events_or) {
        /* two lines on the chip: the events ORed, and the abort */
        DeviceState *orgate = qdev_new(TYPE_OR_IRQ);

        qdev_prop_set_uint16(orgate, "num-lines", 8);
        qdev_realize_and_unref(orgate, NULL, &error_fatal);
        qdev_connect_gpio_out(orgate, 0, rk_irq(s, c->dmac_irq[0]));
        for (i = 0; i < 8; i++) {
            sysbus_connect_irq(sbd, i + 1, qdev_get_gpio_in(orgate, i));
        }
    } else {
        for (i = 0; i < 8; i++) {
            sysbus_connect_irq(sbd, i + 1, rk_irq(s, c->dmac_irq[i]));
        }
    }
    {
        /*
         * The devicetree gives it 16 KiB, and the AMBA bus reads the
         * PrimeCell ID from the top of that (0x3fe0): the 4 KiB register
         * file repeats there.
         */
        MemoryRegion *alias = g_new(MemoryRegion, 1);

        memory_region_init_alias(alias, OBJECT(dev), "rockchip.dmac-id",
                                 sysbus_mmio_get_region(sbd, 0), 0, 0x1000);
        memory_region_add_subregion(get_system_memory(),
                                    c->dmac_base + 0x3000, alias);
    }
}

/* The IOMMU created at @base, if any */
static DeviceState *rk_iommu_at(RkMachineState *s, const RkSoCConfig *c,
                                hwaddr base)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(c->iommu_base) && c->iommu_base[i]; i++) {
        if (c->iommu_base[i] == base) {
            return s->iommu[i];
        }
    }
    return NULL;
}

/* Ethernet, the MMC hosts and audio */
static void rk_create_tail(RkMachineState *s, const RkSoCConfig *c)
{
    DeviceState *dev, *isp = NULL;
    SysBusDevice *sbd;
    int i;

    if (c->isp_base) {
        dev = qdev_new("rk-isp32");
        /* online behind VICAP: frames only while it sends them */
        qdev_prop_set_bit(dev, "source-gated", c->vicap_base != 0);
        qdev_prop_set_bit(dev, "readback", c->isp_readback);
        qdev_prop_set_bit(dev, "v33", c->isp_v33);
        if (s->cam_sensor) {
            /* the scene is lit by its exposure */
            object_property_set_link(OBJECT(dev), "sensor",
                                     OBJECT(s->cam_sensor), &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        isp = dev;
        /* under the IOMMU mapped inside its window */
        sysbus_mmio_map_overlap(sbd, 0, c->isp_base, -1);
        for (i = 0; i < 3; i++) {
            sysbus_connect_irq(sbd, i, rk_irq(s, c->isp_irq[i]));
        }
        if (c->vicap_base) {
            DeviceState *vicap = qdev_new("rk-vicap");

            /* each MIPI host's frames are its sensor's scene */
            if (s->cam_sensor) {
                object_property_set_link(OBJECT(vicap), "sensor0",
                                         OBJECT(s->cam_sensor), &error_fatal);
            }
            if (s->cam_sensor2) {
                object_property_set_link(OBJECT(vicap), "sensor1",
                                         OBJECT(s->cam_sensor2), &error_fatal);
            }
            sysbus_realize_and_unref(SYS_BUS_DEVICE(vicap), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(vicap), 0, c->vicap_base);
            sysbus_connect_irq(SYS_BUS_DEVICE(vicap), 0,
                               rk_irq(s, c->vicap_irq));
            qdev_connect_gpio_out_named(dev, "vicap-fs", 0,
                qdev_get_gpio_in_named(vicap, "frame-start", 0));
            qdev_connect_gpio_out_named(dev, "vicap-fe", 0,
                qdev_get_gpio_in_named(vicap, "frame-end", 0));
            qdev_connect_gpio_out_named(vicap, "toisp-on", 0,
                qdev_get_gpio_in_named(dev, "source-on", 0));
        }
    }
    if (c->cif_base) {
        dev = qdev_new("rk-cif");
        if (s->cam_sensor) {
            /* frames while it streams, at its rate */
            object_property_set_link(OBJECT(dev), "sensor",
                                     OBJECT(s->cam_sensor), &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->cif_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->cif_irq));
    }
    if (c->ispp_base) {
        DeviceState *ispp_mmu = rk_iommu_at(s, c, c->ispp_base + 0xe00);

        dev = qdev_new("rk-ispp");
        if (ispp_mmu) {
            object_property_set_link(OBJECT(dev), "iommu", OBJECT(ispp_mmu),
                                     &error_fatal);
        }
        if (s->cam_sensor) {
            object_property_set_link(OBJECT(dev), "sensor",
                                     OBJECT(s->cam_sensor), &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map_overlap(sbd, 0, c->ispp_base, -1);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->ispp_irq[0]));
        sysbus_connect_irq(sbd, 1, rk_irq(s, c->ispp_irq[1]));
    }
    if (c->vepu2_base) {
        DeviceState *mmu = rk_iommu_at(s, c, c->vepu2_base + 0x800);

        dev = qdev_new("rk-vepu2");
        if (mmu) {
            object_property_set_link(OBJECT(dev), "iommu", OBJECT(mmu),
                                     &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->vepu2_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->vepu2_irq));

        /* the VDPU2 (JPEG decode) beside it, behind the same IOMMU */
        dev = qdev_new("rk-vdpu2");
        if (mmu) {
            object_property_set_link(OBJECT(dev), "iommu", OBJECT(mmu),
                                     &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->vepu2_base + 0x400);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->vdpu2_irq));
    }
    if (c->rga_base) {
        sysbus_create_simple("rk-rga2", c->rga_base, rk_irq(s, c->rga_irq));
    }
    /* /machine/pwmN: its outputN properties read the pins' levels */
    for (i = 0; i < ARRAY_SIZE(c->pwm_base) && c->pwm_base[i]; i++) {
        g_autofree char *name = g_strdup_printf("pwm%d", i);

        dev = qdev_new("rk-pwm");
        object_property_add_child(OBJECT(s), name, OBJECT(dev));
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->pwm_base[i]);
        if (c->pwm_irq[i]) {
            sysbus_connect_irq(sbd, 0, rk_irq(s, c->pwm_irq[i]));
        }
    }
    if (c->saradc_v1_base) {
        sysbus_create_simple("rk-saradc-v1", c->saradc_v1_base,
                             rk_irq(s, c->saradc_v1_irq));
    }
    if (c->vepu_pp_base) {
        sysbus_create_simple("rk-vepu-pp", c->vepu_pp_base,
                             rk_irq(s, c->vepu_pp_irq));
    }
    if (c->rknpu_base) {
        sysbus_create_simple("rk-rknpu", c->rknpu_base,
                             rk_irq(s, c->rknpu_irq));
    }
    if (c->npu_base) {
        dev = qdev_new("rk-npu");
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->npu_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->npu_irq));
    }
    if (c->vepu_base) {
        DeviceState *mmu = rk_iommu_at(s, c, c->vepu_base + 0xf00);

        dev = qdev_new("rk-vepu540c");
        qdev_prop_set_bit(dev, "vepu541", c->vepu541);
        if (mmu) {
            object_property_set_link(OBJECT(dev), "iommu", OBJECT(mmu),
                                     &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        /* under an IOMMU mapped inside its window */
        sysbus_mmio_map_overlap(sbd, 0, c->vepu_base, -1);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->vepu_irq));
        if (isp && c->hpmcu_base) {
            /* the HPMCU starts online jobs for the ISP's current pipe */
            DeviceState *mcu = qdev_new("rk-hpmcu");

            qdev_prop_set_uint64(mcu, "vepu-base", c->vepu_base);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(mcu), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(mcu), 0, c->hpmcu_base);
            qdev_connect_gpio_out_named(isp, "dvbm", 0,
                qdev_get_gpio_in_named(mcu, "isp-frame", 0));
            qdev_connect_gpio_out_named(mcu, "vepu-start", 0,
                qdev_get_gpio_in_named(dev, "dvbm-frame", 0));
        } else if (isp) {
            /* DVBM: the ISP's mainpath frames start online jobs */
            qdev_connect_gpio_out_named(isp, "dvbm", 0,
                qdev_get_gpio_in_named(dev, "dvbm-frame", 0));
        }
    }
    if (c->i2s_base && s->pl330) {
        /* its DMA requests stall the PL330's waits until the FIFO moves */
        dev = qdev_new("rk-i2s-tdm");
        if (c->i2s_clk_rv1106) {
            /* its MCLK from the RV1106 TOPCRU (CLKSEL at +0x10300) */
            qdev_prop_set_uint64(dev, "cru-base", c->cru_base + 0x10200);
            qdev_prop_set_bit(dev, "clk-rv1106", true);
            qdev_prop_set_uint64(dev, "cpll-addr", c->cru_base + 0x10020);
            qdev_prop_set_uint64(dev, "gpll-addr", c->cru_base + 0x10060);
        }
        if (c->i2s_clk_rv1126) {
            /* its sample clock from the CRU's MCLK (the RV1126 layout) */
            qdev_prop_set_uint64(dev, "cru-base", c->cru_base + 0x10000);
            qdev_prop_set_uint64(dev, "cpll-addr", c->cru_base + 0x10040);
            qdev_prop_set_uint64(dev, "gpll-addr", c->cru_base + 0x10);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->i2s_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->i2s_irq));
        for (i = 0; i < 2; i++) {
            qdev_connect_gpio_out_named(dev, "drq-busy", i,
                qdev_get_gpio_in(s->pl330, c->i2s_drq[i]));
        }
    }

    /* GMAC: dwmac-4.20a with a PHY at the SoC's / board's MDIO address */
    dev = qdev_new("axera-eqos");
    /* with the Wi-Fi the default -nic is the chip's: wlan0 is the net */
    qemu_configure_nic_device(dev, !s->wifi || !*s->wifi, NULL);
    qdev_prop_set_uint8(dev, "phy-addr", c->gmac_phy);
    qdev_prop_set_uint8(dev, "snps-ver", 0x42);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->gmac_base);
    sysbus_connect_irq(sbd, 0, rk_irq(s, c->gmac_irq));

    /* DW MSHC 2.70a: SD card (index 0), SDIO (1), eMMC (2) */
    for (i = 0; i < 3 && c->mmc_base[i]; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);

        dev = qdev_new("hisi-himci");
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->mmc_base[i]);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->mmc_irq[i]));
        if (i == 1 && s->wifi && !strcmp(s->wifi, "ws73")) {
            /* XM's battery boards: a WS73 on the SDIO slot */
            DeviceState *card = qdev_new("ws73-sdio");
            g_autofree char *bank = g_strdup_printf("gpio%d",
                                                     c->ws73_irq / 32);
            DeviceState *gpio = DEVICE(object_resolve_path_component(
                                           OBJECT(s), bank));

            qemu_configure_nic_device(card, true, NULL);
            object_property_add_child(OBJECT(s), "ws73", OBJECT(card));
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
            /* its interrupt: the GPIO XM's syschannel asks for */
            qdev_connect_gpio_out_named(card, "wake", 0,
                qdev_get_gpio_in(gpio, c->ws73_irq % 32));
            /*
             * Its host wake-up in XM's AOV: the power key's line
             * (GPIO0_A2, gpio-keys, a wake-up source) on both boards
             */
            qdev_connect_gpio_out_named(card, "host-wake", 0,
                qdev_get_gpio_in(DEVICE(object_resolve_path_component(
                                     OBJECT(s), "gpio0")), 2));
        } else if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }
}

static void rockchip_common_init(MachineState *machine, const RkSoCConfig *c)
{
    RkMachineState *s = RK_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *dev, *flash, *vop_mmu = NULL;
    SysBusDevice *sbd;
    Object *cpuobj, *cpus[4];
    int i, ncpu;
    bool spi_nand;

    if (s->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(s->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("rockchip: cannot read %s: %s", s->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }
    rk_machine_tb_from_meta(s, img, img_len);

    ncpu = machine->smp.cpus;
    for (i = 0; i < ncpu; i++) {
        cpus[i] = object_new(machine->cpu_type);
        object_property_set_int(cpus[i], "reset-cbar", c->gic_base,
                                &error_abort);
        object_property_set_int(cpus[i], "cntfrq", 24 * 1000 * 1000,
                                &error_abort);
        if (c->mpidr_base) {
            object_property_set_int(cpus[i], "mp-affinity",
                                    c->mpidr_base + i, &error_abort);
        }
        if (c->psci) {
            /*
             * No secure world to run: PSCI calls (SMC) go to QEMU, as
             * they would go to OP-TEE, and Linux boots non-secure.
             */
            object_property_set_bool(cpus[i], "has_el3", false,
                                     &error_abort);
            object_property_set_bool(cpus[i], "has_el2", false,
                                     &error_abort);
        }
        if (i && !c->psci) {
            /* powered up by the firmware's PSCI, not from reset */
            object_property_set_bool(cpus[i], "start-powered-off", true,
                                     &error_abort);
        }
        qdev_realize(DEVICE(cpus[i]), NULL, &error_fatal);
    }
    cpuobj = cpus[0];

    memory_region_add_subregion(sysmem, RK_DDR_BASE, machine->ram);
    memory_region_init_ram(sram, NULL, "rockchip.sram", c->sram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, c->sram_base, sram);
    memory_region_init_rom(bootrom, NULL, "rockchip.bootrom",
                           RK_BOOTROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, RK_BOOTROM_BASE, bootrom);

    /* Everything unmodelled in the peripheral space reads as zero. */
    create_unimplemented_device("rockchip.periph", c->periph_base,
                                c->periph_size);

    /* GIC-400 with the A7's private layout: dist +0x1000, cpu +0x2000. */
    s->gic = qdev_new(TYPE_A15MPCORE_PRIV);
    qdev_prop_set_uint32(s->gic, "num-cpu", ncpu);
    qdev_prop_set_uint32(s->gic, "num-irq", RK_NUM_SPI + 32);
    sbd = SYS_BUS_DEVICE(s->gic);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->gic_base);
    for (i = 0; i < ncpu; i++) {
        DeviceState *cd = DEVICE(cpus[i]);

        if (c->hotboot) {
            /* IRQ / FIQ through the PMU's wake filter */
            qemu_irq *f = qemu_allocate_irqs(rk_cpupwr_line, s, 8);

            s->cpu_line[i][0] = qdev_get_gpio_in(cd, ARM_CPU_IRQ);
            s->cpu_line[i][1] = qdev_get_gpio_in(cd, ARM_CPU_FIQ);
            sysbus_connect_irq(sbd, i, f[2 * i]);
            sysbus_connect_irq(sbd, i + ncpu, f[2 * i + 1]);
        } else {
            sysbus_connect_irq(sbd, i, qdev_get_gpio_in(cd, ARM_CPU_IRQ));
            sysbus_connect_irq(sbd, i + ncpu,
                               qdev_get_gpio_in(cd, ARM_CPU_FIQ));
        }
        sysbus_connect_irq(sbd, i + 2 * ncpu,
                           qdev_get_gpio_in(cd, ARM_CPU_VIRQ));
        sysbus_connect_irq(sbd, i + 3 * ncpu,
                           qdev_get_gpio_in(cd, ARM_CPU_VFIQ));
    }

    if (c->hotboot) {
        s->spi_in = qemu_allocate_irqs(rk_cpupwr_spi, s, RK_NUM_SPI);
    }
    rk_syscon(c->grf_base, c->grf_size, NULL, 0, NULL, 0);
    if (c->pmu_base) {
        DeviceState *pmu = rk_syscon_new(0x10000, NULL, 0, NULL, 0);

        rk_prop_list(pmu, "plain-regs", c->pmu_plain, c->n_pmu_plain);
        if (c->pmu_wake_timeout) {
            /* a timed wake-up from the kernel's suspend (XM's AOV) */
            qdev_prop_set_uint32(pmu, "wake-mode", c->pmu_wake_mode);
            qdev_prop_set_uint32(pmu, "wake-mode-bit", 1);
            qdev_prop_set_uint32(pmu, "wake-timeout", c->pmu_wake_timeout);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pmu), 0, c->pmu_base);
    }
    if (c->pmuioc_base) {
        rk_syscon(c->pmuioc_base, 0x8000, NULL, 0, NULL, 0);
    }
    if (c->ioc_base) {
        rk_syscon(c->ioc_base, c->ioc_size, NULL, 0, NULL, 0);
    }
    for (i = 0; i < ARRAY_SIZE(c->syscon_base) && c->syscon_base[i]; i++) {
        rk_syscon(c->syscon_base[i], c->syscon_size[i], NULL, 0, NULL, 0);
    }
    if (c->ehci_base) {
        /* EHCI with the OHCI as its companion for full / low speed */
        DeviceState *ehci = qdev_new(TYPE_PLATFORM_EHCI);
        DeviceState *ohci = qdev_new(TYPE_SYSBUS_OHCI);

        qdev_prop_set_bit(ehci, "companion-enable", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, c->ehci_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0, rk_irq(s, c->ehci_irq));
        qdev_prop_set_string(ohci, "masterbus",
                             QLIST_FIRST(&ehci->child_bus)->name);
        qdev_prop_set_uint32(ohci, "num-ports", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ohci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ohci), 0, c->ohci_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(ohci), 0, rk_irq(s, c->ohci_irq));
    }
    if (c->dwc3_base && s->usb_otg && !strcmp(s->usb_otg, "device")) {
        dev = qdev_new("rk-dwc3");
        qemu_configure_nic_device(dev, false, "usb-gadget");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->dwc3_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, rk_irq(s, c->dwc3_irq));
    } else if (c->dwc3_base) {
        /*
         * QEMU's DWC3 reports a host-only core (GHWPARAMS0 mode 1): the
         * driver then runs it as a host whatever dr_mode the DT asks
         * for.  Its xHCI's bus comes after the EHCI's (usb-bus.0 on the
         * RV1126; usb-bus.0 itself on the RV1106).
         */
        USBDWC3 *dwc3 = USB_DWC3(qdev_new(TYPE_USB_DWC3));

        qdev_prop_set_uint32(DEVICE(&dwc3->sysbus_xhci), "p2", 2);
        qdev_prop_set_uint32(DEVICE(&dwc3->sysbus_xhci), "p3", 1);
        qdev_prop_set_uint32(DEVICE(&dwc3->sysbus_xhci), "slots", 16);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dwc3), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dwc3), 0, c->dwc3_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(&dwc3->sysbus_xhci), 0,
                           rk_irq(s, c->dwc3_irq));
    }
    if (c->usbphy_base) {
        dev = qdev_new("rk-syscon");
        qdev_prop_set_uint32(dev, "size", 0x10000);
        qdev_prop_set_bit(dev, "hiword", false);
        rk_prop_list(dev, "const", rv1126_usbphy_const,
                     ARRAY_SIZE(rv1126_usbphy_const));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->usbphy_base);
    }
    /* IOMMUs; the VOP's (at its base + 0xf00) translates its scan-out */
    for (i = 0; i < ARRAY_SIZE(c->iommu_base) && c->iommu_base[i]; i++) {
        dev = qdev_new("rk-iommu");
        s->iommu[i] = dev;
        if (c->vop_base && c->iommu_base[i] == c->vop_base + 0xf00) {
            vop_mmu = dev;
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->iommu_base[i]);
        if (c->iommu_irq[i]) {
            sysbus_connect_irq(sbd, 0, rk_irq(s, c->iommu_irq[i]));
        }
    }
    if (c->vop_base) {
        dev = qdev_new("rk-vop");
        if (vop_mmu) {
            object_property_set_link(OBJECT(dev), "iommu", OBJECT(vop_mmu),
                                     &error_fatal);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->vop_base);
        sysbus_connect_irq(sbd, 0, rk_irq(s, c->vop_irq));
    }
    if (c->timer_base) {
        dev = qdev_new("rk-timer");
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, c->timer_base);
        for (i = 0; i < 6; i++) {
            if (c->timer_irq[i]) {
                sysbus_connect_irq(sbd, i, rk_irq(s, c->timer_irq[i]));
            }
        }
    }
    if (c->nandc_base) {
        dev = qdev_new("rk-syscon");
        qdev_prop_set_uint32(dev, "size", 0x4000);
        qdev_prop_set_bit(dev, "hiword", false);
        rk_prop_list(dev, "const", rv1126_nandc_const,
                     ARRAY_SIZE(rv1126_nandc_const));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->nandc_base);
    }
    if (c->pmu_ctl_base) {
        dev = rk_syscon_new(0x10000, NULL, 0, NULL, 0);
        rk_prop_list(dev, "mirror", c->pmu_mirror, c->n_pmu_mirror);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->pmu_ctl_base);
    }
    if (c->hotboot) {
        s->ncpu = ncpu;
        s->hotboot = c->hotboot;
        for (i = 0; i < ncpu; i++) {
            s->cpu[i] = ARM_CPU(cpus[i]);
            s->cpu[i]->wfi_hook = rk_cpupwr_wfi;
            s->cpu[i]->wfi_hook_opaque = s;
        }
        memory_region_init_io(&s->cpupwr, OBJECT(s), &rk_cpupwr_ops, s,
                              "rockchip.pmu-cores", 0x10);
        memory_region_add_subregion_overlap(sysmem, c->pmu_ctl_base + 0x90,
                                            &s->cpupwr, 1);
    }
    /*
     * OTP (0xFF3D0000) and secure OTP (+0x8000): the drivers poll the
     * interrupt status at +0x304 for "done", then clear it.  Reading
     * every done bit set gives an unprogrammed (all-zero) OTP.
     */
    if (c->otp_base && c->otp_v1126) {
        /* the OTP Linux reads, and the secure one OP-TEE reads */
        sysbus_create_simple("rk-otp", c->otp_base, NULL);
        sysbus_create_simple("rk-otp", c->otp_base + 0x10000, NULL);
    } else if (c->otp_base) {
        dev = qdev_new("rk-syscon");
        qdev_prop_set_uint32(dev, "size", 0x10000);
        rk_prop_list(dev, "const", rv1106_otp_status,
                     ARRAY_SIZE(rv1106_otp_status));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->otp_base);
    }
    if (!c->saradc_v1_base) {
        /* SARADC v2: constant readings (a v1 is its own model) */
        dev = qdev_new("rk-syscon");
        qdev_prop_set_uint32(dev, "size", 0x1000);
        qdev_prop_set_bit(dev, "hiword", false);
        rk_prop_list(dev, "const", rv1106_saradc_const,
                     ARRAY_SIZE(rv1106_saradc_const));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->saradc_base);
    }

    dev = qdev_new("rk-syscon");
    qdev_prop_set_uint32(dev, "size", 0x1000);
    rk_prop_list(dev, "const", rv1106_tsadc_const,
                 ARRAY_SIZE(rv1106_tsadc_const));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->tsadc_base);

    dev = rk_syscon_new(c->cru_size, c->cru_lock, c->n_cru_lock,
                        c->cru_init, c->n_cru_init);
    rk_prop_list(dev, "plain-regs", c->cru_frac, c->n_cru_frac);
    qdev_prop_set_uint32(dev, "srst-fst", c->cru_srst_fst);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->cru_base);

    /* DW APB UARTs: 16550 at reg-shift 2, 24 MHz; USR (0x7c) reads idle. */
    for (i = 0; i < 6 && c->uart_base[i]; i++) {
        rk_create_uart(sysmem, c->uart_base[i], rk_irq(s, c->uart_irq[i]),
                       i == c->console_uart ? serial_hd(0) : NULL);
    }

    dev = qdev_new("rk-crypto");
    qdev_prop_set_uint32(dev, "version", c->crypto_version);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->crypto_base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, rk_irq(s, c->crypto_irq));

    dev = qdev_new("rk-decom");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->decom_base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, rk_irq(s, c->decom_irq));

    /*
     * A part named on the command line decides: SPI NAND or NOR (an
     * RV1126 board with SPI NOR, a NAND RV1106); else the machine's own
     */
    spi_nand = s->flash_type ?
        !!object_class_dynamic_cast(object_class_by_name(s->flash_type),
                                    "ingenic-spinand") :
        c->spi_nand;
    dev = qdev_new("rk-sfc");
    qdev_prop_set_bit(dev, "dummy-bytes", spi_nand);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->sfc_base);
    sysbus_connect_irq(sbd, 0, rk_irq(s, c->sfc_irq));
    flash = rk_create_flash(s, qdev_get_child_bus(dev, "ssi"), img_len,
                            spi_nand);
    qdev_connect_gpio_out_named(dev, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));

    rk_create_more(s, c);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = RK_DDR_BASE;
    if (c->psci) {
        binfo.psci_conduit = QEMU_PSCI_CONDUIT_SMC;
    } else {
        /*
         * -kernel: start Linux in the secure world, as the vendor chain
         * leaves it (no secure firmware; CONFIG_FIQ_GLUE kernels run there
         * and put the interrupts in group 1 themselves).  Booted
         * non-secure, every interrupt stays in group 0 and the first
         * timer tick never comes.
         */
        binfo.secure_boot = true;
    }
    arm_load_kernel(ARM_CPU(cpuobj), machine, &binfo);

    if (!machine->kernel_filename && img) {
        /*
         * The RV1126 dump at hand lost its ID block, so that machine
         * starts U-Boot from the FIT -- unless the image has one
         */
        bool fit = c->boot_fit && !rk_has_idb(img, img_len);
        bool ok = fit ? rk_fit_setup(img, img_len, c, machine->ram_size)
                      : rk_boot_rom_setup(img, img_len, c, machine->ram_size);
        if (!ok) {
            exit(1);
        }
    }
}

static void rk_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "rockchip.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        rk_machine_get_flash_file, rk_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (RKNS ID block at 0 or "
        "0x10000); erases and programs are written back to it");
    object_class_property_add_str(oc, "flash-type",
        rk_machine_get_flash_type, rk_machine_set_flash_type);
    object_class_property_set_description(oc, "flash-type",
        "flash part on the SFC: an m25p80 SPI NOR (e.g. mx25l12805d; default "
        "by image size) or a SPI NAND (w25n01gv, gd5f1gq4ub, ...), which "
        "puts the SFC in NAND mode");
    object_class_property_add_str(oc, "sensor",
        rk_machine_get_sensor, rk_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor at 0x30 on the camera I2C (none, sc2336p, sc231hai, "
        "sc3336, ...); default sc2336p");
    object_class_property_add_str(oc, "sensor2",
        rk_machine_get_sensor2, rk_machine_set_sensor2);
    object_class_property_set_description(oc, "sensor2",
        "Second sensor, at 0x32 (two-sensor boards); default none");
    object_class_property_add_str(oc, "usb-otg",
        rk_machine_get_usb_otg, rk_machine_set_usb_otg);
    object_class_property_set_description(oc, "usb-otg",
        "The OTG port's role: host (QEMU's DWC3 + xHCI; default) or device "
        "(rk-dwc3: the gadget enumerated by a built-in host, a CDC ECM "
        "function bridged to -nic ...,model=usb-gadget)");
    object_class_property_add_str(oc, "wifi",
        rk_machine_get_wifi, rk_machine_set_wifi);
    object_class_property_set_description(oc, "wifi",
        "A Wi-Fi chip on the SDIO slot: ws73 (XM's battery boards, the G2 "
        "and the RV1103B)");
    object_class_property_add_str(oc, "gc6160",
        rk_machine_get_gc6160, rk_machine_set_gc6160);
    object_class_property_set_description(oc, "gc6160",
        "XM's GC6160 motor / IR-cut driver, bit-banged I2C on two GPIOs: "
        "CLK:DATA as Linux GPIO numbers (XmMotor.ko's gpio_pin; 70:75 on "
        "the XM RV1103B)");
    object_class_property_add_str(oc, "tb-isp",
        rk_machine_get_tb_isp, rk_machine_set_tb_isp);
    object_class_property_set_description(oc, "tb-isp",
        "Stand in for the MCU's thunder-boot result to the ISP driver: "
        "ADDR:CAMERAS[:STRIDE] (the ISP's reserved memory, e.g. "
        "0x8b0000:2 on the XM G2; by default from the flash's thunder-boot "
        "meta when it names two cameras; off: none)");
    object_class_property_add_str(oc, "scene", NULL, rk_machine_set_scene);
    object_class_property_set_description(oc, "scene",
        "A YUV4MPEG2 (4:2:0) recording the cameras see, looped, in place "
        "of the test pattern");
    object_class_property_add_str(oc, "gpio-high",
        rk_machine_get_gpio_high, rk_machine_set_gpio_high);
    object_class_property_set_description(oc, "gpio-high",
        "Input pins that idle high, as a board pull-up holds them: "
        "'/'-separated bank, port and pin (4C0/0A2); others read 0");
}

static const TypeInfo rk_machine_info = {
    .name           = TYPE_RK_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(RkMachineState),
    .class_init     = rk_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void rk_machine_register_types(void)
{
    type_register_static(&rk_machine_info);
}

type_init(rk_machine_register_types)

#define DEFINE_RK_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        rockchip_common_init(machine, &config);                      \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_cpus = MAX(config.num_cpus, 1);                  \
        mc->min_cpus = mc->default_cpus;                             \
        mc->max_cpus = mc->default_cpus;                             \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, RK_MACHINE, RkMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_RK_MACHINE("rv1103", rv1103, rv1103_soc)
DEFINE_RK_MACHINE("rv1106", rv1106, rv1106_soc)
DEFINE_RK_MACHINE("rv1106g3", rv1106g3, rv1106g3_soc)
DEFINE_RK_MACHINE("rv1103b", rv1103b, rv1103b_soc)
DEFINE_RK_MACHINE("rv1126", rv1126, rv1126_soc)
DEFINE_RK_MACHINE("rv1109", rv1109, rv1109_soc)
