/*
 * Fullhan IP-camera SoC machines.
 *
 * Table-driven like the other families: one FhSoCConfig per die.  Every
 * Fullhan camera die so far keeps one memory map -- DDR at 0xA0000000 and
 * mostly DesignWare peripherals -- and the generations differ in the CPU,
 * the DMA engine, the interrupt numbers and a few blocks:
 *
 *   0xE0200000 INTC (DW APB ICTL, 64 level IRQs)    hw/intc/fh-intc.c
 *   0xE0300000 DMAC: DW_axi_dmac on FH885xV200 /    hw/dma/fh-axi-dmac.c
 *              FH865x, DW_ahb_dmac before           hw/dma/fh-ahb-dmac.c
 *   0xE0600000 GMAC (DWMAC, normal descriptors)     hw/net/xm-dwmac.c
 *   0xE0700000 USB 2.0 OTG (DWC2, host)             QEMU's dwc2-usb
 *   0xE2000000 / 0xE2200000 SD0 / SD1 (DW MMC, IDMAC) hw/misc/hisi-himci.c
 *   0xF0000000 PMU: clocks, resets, pads, chip ID   hw/misc/fh-pmu.c
 *   0xF0200000 / 0xF0B00000 / 0xF0100000 I2C0-2 (DW) hw/i2c/fh-i2c.c
 *   0xF0300000 / 0xF4000000 GPIO0 / GPIO1 (DW)      hw/gpio/fh-gpio.c
 *   0xF0500000 / 0xF0600000 / 0xF0640000 SPI0-2     hw/ssi/fh-spi.c
 *   0xF0700000 / 0xF0800000 / 0xF1300000 UART0-2    hw/char/fh-uart.c
 *   0xF0C00000 timers (DW APB)                      hw/timer/fh-timer.c
 *   0xF0D00000 watchdog (DW)                        hw/watchdog/fh-wdt.c
 *   0xF1500000 RTC                                  hw/rtc/fh-rtc.c
 *   0xF1600000 eFuse                                hw/misc/fh-efuse.c
 *   0xE8200000 AES / DES engine                     hw/misc/fh-aes.c
 *   0xF1200000 SADC                                 hw/misc/fh-sadc.c
 *   0xE7000040 PTS (FH8632's clocksource)           hw/timer/fh-pts.c
 *
 * (mach-fh/<soc>/chip.h and the board files of the SDKs; docs/fullhan.md
 * has the tables.)  FH865x puts a Cortex-A7 around the FH885xV200 map and
 * swaps the DW ICTL for a GIC-400 at 0x30000000 (its kernel's init_irq),
 * the same interrupt numbers arriving as SPIs.  The kernels the SDKs and
 * the cameras ship are board-file ones, machine number 9999 for every part
 * (arch/arm/tools/mach-types), with no device tree.
 *
 * Boot: flash-file= boots a camera's SPI NOR image the way the mask ROM
 * does (fh_boot_rom_setup() below), or -kernel boots a uImage / zImage
 * with ATAGs.  The flash is a JEDEC part on SPI0's chip select 0, a GPIO.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/cpu/a15mpcore.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "block/block-common.h"
#include "target/arm/cpu.h"

#define FH_SRAM_BASE        0x10000000
#define FH_DDR_BASE         0xA0000000
#define FH_INTC_BASE        0xE0200000
#define FH_DMAC_BASE        0xE0300000
#define FH_GMAC_BASE        0xE0600000
#define FH_USB_BASE         0xE0700000
#define FH_PMU_BASE         0xF0000000
#define FH_AES_BASE         0xE8200000
#define FH_SADC_BASE        0xF1200000
#define FH_TIMER_BASE       0xF0C00000
#define FH_WDT_BASE         0xF0D00000
#define FH_RTC_BASE         0xF1500000
#define FH_EFUSE_BASE       0xF1600000

#define FH_BOOTROM_SIZE     0x10000
#define FH_NUM_IRQ          64

/* arch/arm/tools/mach-types: every Fullhan board file. */
#define FH_MACH             9999

typedef struct FhSoCConfig {
    const char *desc;
    const char *cpu;
    uint64_t ram_size_default;
    uint32_t sram_size;
    hwaddr gic_base;            /* 0: the DW ICTL at 0xE0200000 */

    uint32_t chip_id;           /* PMU 0x000, platform ID */
    uint32_t ip_ver;            /* PMU 0x004, the V100 package ID */
    uint32_t chip_info;         /* PMU 0x33C, the V200 package ID */
    uint32_t ddr_mbit;          /* PMU 0x334 */
    uint32_t pll_scheme;        /* fh-pmu: where the system PLL is */
    uint32_t tmr_prediv;        /* fh-pmu: scheme 2's PLL1 prediv, 0: 10 */
    const char *sensor;         /* default sensor= */
    uint32_t swrst_base;        /* fh-pmu: 0 for 0x040 */
    uint32_t spc_io_status;     /* PMU 0x054, FH8632's variant */
    bool rtc_v1;
    bool pts;                   /* the PAE PTS counter at 0xE7000040 */

    int timer_irq;
    int wdt_irq;
    int rtc_irq;
    int aes_irq;
    int sadc_irq;
    uint32_t sadc_version;      /* fh-sadc: 1 V100 / FH8632, 2, 3 V200 */
    int gmac_irq;
    int usb_irq;
    bool usb_fh_otg;            /* 3.0.8 fh_otg: check GHWCFG, DDMA */
    bool ahb_dmac;
    int dmac_irq;
    int num_uart;
    hwaddr uart_base[3];
    int uart_irq[3];
    int num_spi;
    hwaddr spi_base[3];
    int spi_irq[3];
    int spi_dma_rx[3];          /* DMAC handshake numbers */
    int spi_dma_tx[3];
    int flash_cs_gpio;
    int num_i2c;
    hwaddr i2c_base[3];
    int i2c_irq[3];
    int sdc_irq[2];
    int gpio_irq[2];
} FhSoCConfig;

/*
 * FH885xV200 (mach-fh/fh8856v200/chip.h, the SDK's fh8856v200.dts).  The
 * FH865x dies keep it (the FH8652 dump's kernel has the same static I/O
 * map and resources).
 */
#define FH_V200_MAP                                                     \
    .sram_size          = 136 * KiB,                                    \
    .timer_irq          = 3,                                            \
    .wdt_irq            = 2,                                            \
    .rtc_irq            = 33,                                           \
    .aes_irq            = 16,                                           \
    .sadc_irq           = 20,                                           \
    .sadc_version       = 3,                                            \
    .gmac_irq           = 44,                                           \
    .usb_irq            = 39,                                           \
    .dmac_irq           = 23,                                           \
    .num_uart           = 3,                                            \
    .uart_base          = { 0xF0700000, 0xF0800000, 0xF1300000 },       \
    .uart_irq           = { 30, 31, 41 },                               \
    .num_spi            = 3,                                            \
    .spi_base           = { 0xF0500000, 0xF0600000, 0xF0640000 },       \
    .spi_irq            = { 28, 21, 38 },                               \
    .spi_dma_rx         = { 4, 2, 14 },                                 \
    .spi_dma_tx         = { 5, 3, 15 },                                 \
    .flash_cs_gpio      = 6,                                            \
    .num_i2c            = 3,                                            \
    .i2c_base           = { 0xF0200000, 0xF0B00000, 0xF0100000 },       \
    .i2c_irq            = { 11, 12, 46 },                               \
    .sdc_irq            = { 42, 43 },                                   \
    .gpio_irq           = { 26, 40 }

#define FH885XV200_COMMON                                               \
    FH_V200_MAP,                                                        \
    .cpu                = ARM_CPU_TYPE_NAME("arm1176"),                 \
    .sensor             = "gc4653",                                     \
    .chip_id            = 0x19112201

/* mach-fh/fh_chipid.c: the package IDs, and the in-package DDR. */
static const FhSoCConfig fh8852v200_soc = {
    FH885XV200_COMMON,
    .desc               = "Fullhan FH8852V200 (ARM1176JZF-S, 64 MB DDR)",
    .ram_size_default   = 64 * MiB,
    .chip_info          = 0x00000001,
    .ddr_mbit           = 512,
};

static const FhSoCConfig fh8856v200_soc = {
    FH885XV200_COMMON,
    .desc               = "Fullhan FH8856V200 (ARM1176JZF-S, 128 MB DDR)",
    .ram_size_default   = 128 * MiB,
    .chip_info          = 0x00100001,
    .ddr_mbit           = 1024,
};

static const FhSoCConfig fh8858v200_soc = {
    FH885XV200_COMMON,
    .desc               = "Fullhan FH8858V200 (ARM1176JZF-S, 128 MB DDR)",
    .ram_size_default   = 128 * MiB,
    .chip_info          = 0x00410001,
    .ddr_mbit           = 1024,
};

/*
 * FH8652 / FH8656 / FH8658: package IDs from mach-fh/fh_chipid.c (masked
 * with 0xC); the FH8652 dump's U-Boot is ARMv7 code (MOVW) and its kernel
 * a Cortex-A7 build.  DDR sizes are the FH885xV200 siblings', not seen on
 * a dump.
 */
#define FH865X_COMMON                                                   \
    FH_V200_MAP,                                                        \
    .cpu                = ARM_CPU_TYPE_NAME("cortex-a7"),               \
    .gic_base           = 0x30000000,                                   \
    .sensor             = "gc2063",                                     \
    .chip_id            = 0x20031601

static const FhSoCConfig fh8652_soc = {
    FH865X_COMMON,
    .desc               = "Fullhan FH8652 (Cortex-A7, 64 MB DDR)",
    .ram_size_default   = 64 * MiB,
    .chip_info          = 0xC,
    .ddr_mbit           = 512,
};

static const FhSoCConfig fh8656_soc = {
    FH865X_COMMON,
    .desc               = "Fullhan FH8656 (Cortex-A7, 128 MB DDR)",
    .ram_size_default   = 128 * MiB,
    .chip_info          = 0x8,
    .ddr_mbit           = 1024,
};

static const FhSoCConfig fh8658_soc = {
    FH865X_COMMON,
    .desc               = "Fullhan FH8658 (Cortex-A7, 128 MB DDR)",
    .ram_size_default   = 128 * MiB,
    .chip_info          = 0x4,
    .ddr_mbit           = 1024,
};

/*
 * FH8852 / FH8856 (V100, SDK FH8856_IPC_V1.2.0, Linux 3.0.8): the same map
 * with the AHB DMAC, two UARTs and I2Cs, the flash select on GPIO 54 and
 * their own interrupt numbers (mach-fh/include/mach/fh8856.h, the board
 * file's resources and DMA handshakes).
 */
#define FH885X_V100_COMMON                                              \
    .cpu                = ARM_CPU_TYPE_NAME("arm1176"),                 \
    .sram_size          = 32 * KiB,                                     \
    .chip_id            = 0x17092901,                                   \
    .sensor             = "gc2053",                                     \
    .pll_scheme         = 1,                                            \
    .timer_irq          = 3,                                            \
    .wdt_irq            = 2,                                            \
    .rtc_irq            = 33,                                           \
    .aes_irq            = 16,                                           \
    .sadc_irq           = 20,                                           \
    .sadc_version       = 1,                                            \
    .gmac_irq           = 15,                                           \
    .usb_irq            = 27,                                           \
    .usb_fh_otg         = true,                                         \
    .ahb_dmac           = true,                                         \
    .dmac_irq           = 23,                                           \
    .num_uart           = 2,                                            \
    .uart_base          = { 0xF0700000, 0xF0800000 },                   \
    .uart_irq           = { 30, 31 },                                   \
    .num_spi            = 3,                                            \
    .spi_base           = { 0xF0500000, 0xF0600000, 0xF0640000 },       \
    .spi_irq            = { 28, 21, 22 },                               \
    .spi_dma_rx         = { 2, 4, 12 },                                 \
    .spi_dma_tx         = { 3, 5, 13 },                                 \
    .flash_cs_gpio      = 54,                                           \
    .num_i2c            = 2,                                            \
    .i2c_base           = { 0xF0200000, 0xF0B00000 },                   \
    .i2c_irq            = { 11, 12 },                                   \
    .sdc_irq            = { 17, 18 },                                   \
    .gpio_irq           = { 26, 40 }

static const FhSoCConfig fh8852v100_soc = {
    FH885X_V100_COMMON,
    .desc               = "Fullhan FH8852 (ARM1176JZF-S, 64 MB DDR)",
    .ram_size_default   = 64 * MiB,
    .ip_ver             = 0xC,
    .ddr_mbit           = 512,
};

static const FhSoCConfig fh8856v100_soc = {
    FH885X_V100_COMMON,
    .desc               = "Fullhan FH8856 (ARM1176JZF-S, 128 MB DDR)",
    .ram_size_default   = 128 * MiB,
    .ip_ver             = 0xD,
    .ddr_mbit           = 1024,
};

/*
 * FH8626V100: no SDK; the camera dump's kernel (its kallsyms, IKCONFIG and
 * resource tables) gives a V100-family die with its own interrupt
 * numbers, SPI0 moved to 0xF0E00000, and the block resets at 0x04C.
 */
static const FhSoCConfig fh8626v100_soc = {
    .desc               = "Fullhan FH8626V100 (ARM1176JZF-S, 64 MB DDR)",
    .cpu                = ARM_CPU_TYPE_NAME("arm1176"),
    .ram_size_default   = 64 * MiB,
    .sram_size          = 32 * KiB,
    .chip_id            = 0x18112301,
    .ddr_mbit           = 512,
    .pll_scheme         = 2,
    .tmr_prediv         = 1,
    .sensor             = "gc2053",
    .swrst_base         = 0x04C,
    .timer_irq          = 3,
    .wdt_irq            = 2,
    .rtc_irq            = 33,
    .aes_irq            = 24,
    .sadc_irq           = 25,
    .sadc_version       = 2,
    .gmac_irq           = 13,
    .usb_irq            = 27,
    .usb_fh_otg         = true,
    .ahb_dmac           = true,
    .dmac_irq           = 21,
    .num_uart           = 3,
    .uart_base          = { 0xF0700000, 0xF0800000, 0xF1300000 },
    .uart_irq           = { 18, 19, 20 },
    .num_spi            = 2,
    .spi_base           = { 0xF0E00000, 0xF0500000 },
    .spi_irq            = { 17, 16 },
    .spi_dma_rx         = { 4, 2 },
    .spi_dma_tx         = { 5, 3 },
    .flash_cs_gpio      = 54,
    .num_i2c            = 3,
    .i2c_base           = { 0xF0200000, 0xF0B00000, 0xF0100000 },
    .i2c_irq            = { 9, 10, 11 },
    .sdc_irq            = { 14, 15 },
    .gpio_irq           = { 26, 40 },
};

/*
 * FH8632 (SDK FH8632_IPC_V1.0 and its FP62 patch, mach-fh board-fh8833.c,
 * Linux 3.0.8): the V100 interrupt numbers, the older RTC, and the PTS
 * as its clocksource.  The variant is PMU 0x054 bits 5:0 (0x37; 0x07 the
 * FH8632v2).
 */
static const FhSoCConfig fh8632_soc = {
    .desc               = "Fullhan FH8632 (ARM1176JZF-S, 64 MB DDR)",
    .cpu                = ARM_CPU_TYPE_NAME("arm1176"),
    .ram_size_default   = 64 * MiB,
    .sram_size          = 32 * KiB,
    .chip_id            = 0x46488302,
    .spc_io_status      = 0x37,
    .ddr_mbit           = 512,
    .pll_scheme         = 2,
    .rtc_v1             = true,
    .pts                = true,
    .timer_irq          = 3,
    .wdt_irq            = 2,
    .rtc_irq            = 33,
    .aes_irq            = 16,
    .sadc_irq           = 20,
    .sadc_version       = 1,
    .gmac_irq           = 15,
    .usb_irq            = 27,
    .usb_fh_otg         = true,
    .ahb_dmac           = true,
    .dmac_irq           = 23,
    .num_uart           = 2,
    .uart_base          = { 0xF0700000, 0xF0800000 },
    .uart_irq           = { 30, 31 },
    .num_spi            = 3,
    .spi_base           = { 0xF0500000, 0xF0600000, 0xF0640000 },
    .spi_irq            = { 28, 21, 22 },
    .spi_dma_rx         = { 2, 4, 12 },
    .spi_dma_tx         = { 3, 5, 13 },
    .flash_cs_gpio      = 54,
    .num_i2c            = 2,
    .i2c_base           = { 0xF0200000, 0xF0B00000 },
    .i2c_irq            = { 11, 12 },
    .sdc_irq            = { 17, 18 },
    .gpio_irq           = { 26, 40 },
};

#define TYPE_FH_MACHINE MACHINE_TYPE_NAME("fullhan")
OBJECT_DECLARE_SIMPLE_TYPE(FhMachineState, FH_MACHINE)

struct FhMachineState {
    MachineState parent_obj;
    char *flash_file;
    char *flash_type;
    char *sensor;
    DeviceState *irqc;
};

static char *fh_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(FH_MACHINE(obj)->flash_file);
}

static void fh_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    FhMachineState *s = FH_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *fh_machine_get_flash_type(Object *obj, Error **errp)
{
    return g_strdup(FH_MACHINE(obj)->flash_type);
}

static void fh_machine_set_flash_type(Object *obj, const char *value,
                                      Error **errp)
{
    FhMachineState *s = FH_MACHINE(obj);

    g_free(s->flash_type);
    s->flash_type = g_strdup(value);
}

static char *fh_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(FH_MACHINE(obj)->sensor);
}

static void fh_machine_set_sensor(Object *obj, const char *value,
                                  Error **errp)
{
    FhMachineState *s = FH_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

/*
 * Image sensors for sensor=, on I2C 0 where the vendor apps (SensorDevice_
 * Init on /dev/i2c-0), OpenIPC's sensor libraries and ipctool's Fullhan
 * HAL probe them.  The apps link their sensor drivers in and try them in
 * turn: the FH885xV200 ones GC2063 (0x37), GC4653 (0x29), MIS2006 (0x30);
 * the V100 / FH8626 ones GC2053, GC2093, GC1054, JX-F37, MIS2006.
 */
static const struct {
    const char *name;
    const char *type;           /* NULL: galaxycore-sensor */
    uint8_t addr;
    bool reg16;
    uint16_t id_reg, id;
} fh_sensors[] = {
    { "gc4653",  NULL,           0x29, true,  0x03F0, 0x4653 },
    { "gc2053",  NULL,           0x37, false, 0xF0,   0x2053 },
    { "gc2063",  NULL,           0x37, false, 0xF0,   0x2063 },
    { "gc2093",  NULL,           0x37, true,  0x03F0, 0x2093 },
    { "gc1054",  NULL,           0x21, false, 0xF0,   0x1054 },
    { "mis2006", "hisi-mis2006", 0x30 },
    { "f37",     "hisi-f37",     0x40 },
};

static void fh_create_sensor(const char *name, I2CBus *bus)
{
    DeviceState *dev;
    int n;

    if (!name || !strcmp(name, "none")) {
        return;
    }
    for (n = 0; n < ARRAY_SIZE(fh_sensors); n++) {
        if (!strcmp(name, fh_sensors[n].name)) {
            break;
        }
    }
    if (n == ARRAY_SIZE(fh_sensors)) {
        error_report("fullhan: unknown sensor '%s' (none, gc4653, gc2053, "
                     "gc2063, gc2093, gc1054, mis2006, f37)", name);
        exit(1);
    }
    if (fh_sensors[n].type) {
        i2c_slave_create_simple(bus, fh_sensors[n].type, fh_sensors[n].addr);
        return;
    }
    dev = DEVICE(i2c_slave_new("galaxycore-sensor", fh_sensors[n].addr));
    qdev_prop_set_bit(dev, "reg16", fh_sensors[n].reg16);
    qdev_prop_set_uint16(dev, "id-reg", fh_sensors[n].id_reg);
    qdev_prop_set_uint16(dev, "id", fh_sensors[n].id);
    i2c_slave_realize_and_unref(I2C_SLAVE(dev), bus, &error_fatal);
}

/* Interrupt n of the SoC's numbering: DW ICTL input n, or GIC SPI n. */
static qemu_irq fh_irq(FhMachineState *fms, int n)
{
    return qdev_get_gpio_in(fms->irqc, n);
}

/*
 * The mask ROM's boot header, the first 0x200-0x300 bytes of the SPI NOR
 * (the flash dumps; the SDK's flash_upgrade tools write it):
 *
 *   +0x00  "2BL*OABD", then version, image count, parameter count and
 *          header size
 *   +0x100 64-byte records: a name, and at record +0x20 size, +0x24
 *          flash offset, +0x28 load address, +0x2C entry, +0x30 type
 *
 * The parameter records ("DDR0".., or the DDR part's name; type 0x23 or
 * 0x43) point at register scripts the ROM runs for the fitted DDR --
 * 16-byte {address, value, mask, op} entries for the PMU PLLs and the DDR
 * controller at 0xED000000.  QEMU's DDR needs no training, so they are
 * skipped.  The image the ROM loads and enters is the one of type 0x10:
 * U-Boot itself on most cameras ("uboot"), FH8632's SPL in DDR, and on
 * FH865x and the V100 SDK's images a ramboot in SRAM, which trains and
 * sizes the DDR (by probing for address aliases, which QEMU's RAM does not
 * have) and then loads the U-Boot record (type 0x11 or 0x12).  For those
 * this stand-in loads that U-Boot itself.  Headers without types fall back
 * to the record named "uboot".
 */
#define FH_HDR_MAGIC        "2BL*OABD"
#define FH_HDR_RECORDS      0x100
#define FH_HDR_END          0x300
#define FH_HDR_REC          0x40
#define FH_HDR_TYPE_ROM     0x10

/*
 *     ldr  pc, [pc, #-4]
 *     .word entry
 */
static const uint32_t fh_bootrom_stub[] = { 0xE51FF004 };

static bool fh_record_fits(uint32_t load, uint32_t size, uint64_t base,
                           uint64_t len)
{
    return load >= base && load - base + (uint64_t)size <= len;
}

static void fh_boot_rom_setup(const uint8_t *img, gsize len,
                              MachineState *machine,
                              const FhSoCConfig *c, const char *path)
{
    uint32_t rec, size, off, load, entry, type;
    uint32_t pick = 0, named = 0, uboot = 0;
    g_autofree uint32_t *rom = g_malloc(FH_BOOTROM_SIZE);

    if (len < FH_HDR_END || memcmp(img, FH_HDR_MAGIC, 8)) {
        error_report("fullhan: %s has no Fullhan boot header", path);
        exit(1);
    }
    for (rec = FH_HDR_RECORDS; rec < FH_HDR_END; rec += FH_HDR_REC) {
        type = ldl_le_p(img + rec + 0x30) & 0xFF;
        if (!pick && type == FH_HDR_TYPE_ROM) {
            pick = rec;
        }
        if (!uboot && (type == 0x11 || type == 0x12) &&
            ldl_le_p(img + rec + 0x28) >= FH_DDR_BASE) {
            uboot = rec;
        }
        if (!named && !g_ascii_strncasecmp((const char *)img + rec,
                                           "uboot", 6)) {
            named = rec;
        }
    }
    if (pick && ldl_le_p(img + pick + 0x28) < FH_DDR_BASE && uboot) {
        pick = uboot;           /* past the ramboot */
    }
    rec = pick ? pick : named;
    if (!rec) {
        error_report("fullhan: %s: the boot header names no image to load",
                     path);
        exit(1);
    }
    size = ldl_le_p(img + rec + 0x20);
    off = ldl_le_p(img + rec + 0x24);
    load = ldl_le_p(img + rec + 0x28);
    entry = ldl_le_p(img + rec + 0x2C);
    if (!size || (uint64_t)off + size > len ||
        !(fh_record_fits(load, size, FH_DDR_BASE, machine->ram_size) ||
          fh_record_fits(load, size, FH_SRAM_BASE, c->sram_size))) {
        error_report("fullhan: %s: the boot image (0x%x bytes from flash "
                     "0x%x to 0x%08x) does not fit", path, size, off, load);
        exit(1);
    }
    rom_add_blob_fixed("fullhan.boot", img + off, size, load);
    /*
     * The rest of the ROM reads as ones, not zeros: the FH8632 U-Boot
     * prints before serial_init() has set its UART pointer, so through a
     * NULL one, polling "USR" at 0x7C for TX-not-full -- which the real
     * ROM's contents evidently satisfy.
     */
    memset(rom, 0xFF, FH_BOOTROM_SIZE);
    rom[0] = cpu_to_le32(fh_bootrom_stub[0]);
    rom[1] = cpu_to_le32(entry);
    rom_add_blob_fixed("fullhan.bootrom", rom, FH_BOOTROM_SIZE, 0);
}

/* A part of the image's size the vendor drivers know. */
static const char *fh_flash_type(gsize len)
{
    switch (len) {
    case 4 * MiB:   return "w25q32";
    case 16 * MiB:  return "mx25l12805d";
    case 32 * MiB:  return "w25q256";
    default:        return "w25q64";
    }
}

static DeviceState *fh_create_flash(FhMachineState *fms, BusState *bus,
                                    gsize img_len)
{
    DeviceState *dev;
    BlockBackend *blk = NULL;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    const char *type = fms->flash_type;

    if (fms->flash_file) {
        QDict *opts = qdict_new();
        Error *err = NULL;

        qdict_put_str(opts, "driver", "raw");
        blk = blk_new_open(fms->flash_file, NULL, opts, BDRV_O_RDWR, &err);
        if (!blk) {
            error_reportf_err(err, "fullhan: %s: ", fms->flash_file);
            exit(1);
        }
    } else if (di) {
        blk = blk_by_legacy_dinfo(di);
        img_len = blk_getlength(blk);
    }
    if (!type) {
        type = fh_flash_type(img_len);
    }
    dev = qdev_new(type);
    if (blk) {
        qdev_prop_set_drive_err(dev, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(dev, bus, &error_fatal);
    if (fms->flash_file) {
        /*
         * The drive property names blk's node, and the device gets a
         * BlockBackend of its own on it.  Ours, writable and attached to
         * no device, would stay a parent of the node, and migration could
         * never inactivate it ("bdrv_inactivate_all() failed").
         */
        blk_unref(blk);
    }
    return dev;
}

static void fh_create_irqc(FhMachineState *fms, const FhSoCConfig *c,
                           DeviceState *cpu)
{
    SysBusDevice *sbd;

    if (!c->gic_base) {
        fms->irqc = qdev_new("fh-intc");
        sbd = SYS_BUS_DEVICE(fms->irqc);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, FH_INTC_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(cpu, ARM_CPU_IRQ));
        sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(cpu, ARM_CPU_FIQ));
        return;
    }
    /* The Cortex-A7 MPCore's private region: distributor at +0x1000. */
    fms->irqc = qdev_new(TYPE_A15MPCORE_PRIV);
    qdev_prop_set_uint32(fms->irqc, "num-cpu", 1);
    qdev_prop_set_uint32(fms->irqc, "num-irq", FH_NUM_IRQ + 32);
    sbd = SYS_BUS_DEVICE(fms->irqc);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, c->gic_base);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(cpu, ARM_CPU_IRQ));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(cpu, ARM_CPU_FIQ));
    sysbus_connect_irq(sbd, 2, qdev_get_gpio_in(cpu, ARM_CPU_VIRQ));
    sysbus_connect_irq(sbd, 3, qdev_get_gpio_in(cpu, ARM_CPU_VFIQ));
}

static void fullhan_common_init(MachineState *machine, const FhSoCConfig *c)
{
    FhMachineState *fms = FH_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *pmu, *dmac, *dev, *flash;
    DeviceState *gpio[2], *spi[3];
    Object *cpuobj;
    int n;

    if (fms->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(fms->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("fullhan: cannot read %s: %s", fms->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    cpuobj = object_new(machine->cpu_type);
    if (c->gic_base) {
        object_property_set_int(cpuobj, "reset-cbar", c->gic_base,
                                &error_abort);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    memory_region_add_subregion(sysmem, FH_DDR_BASE, machine->ram);
    memory_region_init_ram(sram, NULL, "fullhan.sram", c->sram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, FH_SRAM_BASE, sram);
    memory_region_init_rom(bootrom, NULL, "fullhan.bootrom",
                           FH_BOOTROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, 0, bootrom);

    /* Everything unmodelled in the peripheral space reads as zero. */
    create_unimplemented_device("fullhan.periph", 0xE0000000, 0x18000000);

    fh_create_irqc(fms, c, DEVICE(cpuobj));

    pmu = qdev_new("fh-pmu");
    qdev_prop_set_uint32(pmu, "chip-id", c->chip_id);
    qdev_prop_set_uint32(pmu, "ip-ver", c->ip_ver);
    qdev_prop_set_uint32(pmu, "chip-info", c->chip_info);
    qdev_prop_set_uint32(pmu, "ddr-mbit", c->ddr_mbit);
    qdev_prop_set_uint32(pmu, "pll-scheme", c->pll_scheme);
    if (c->tmr_prediv) {
        qdev_prop_set_uint32(pmu, "tmr-prediv", c->tmr_prediv);
    }
    if (c->swrst_base) {
        qdev_prop_set_uint32(pmu, "swrst-base", c->swrst_base);
    }
    qdev_prop_set_uint32(pmu, "spc-io-status", c->spc_io_status);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmu), 0, FH_PMU_BASE);

    dev = qdev_new("fh-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_WDT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->wdt_irq));
    qdev_connect_gpio_out_named(pmu, "wdt-pause", 0,
                                qdev_get_gpio_in_named(dev, "pause", 0));

    dev = qdev_new("fh-rtc");
    qdev_prop_set_bit(dev, "v1", c->rtc_v1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_RTC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->rtc_irq));
    if (c->pts) {
        sysbus_create_simple("fh-pts", 0xE7000000, NULL);
    }
    sysbus_create_simple("fh-efuse", FH_EFUSE_BASE, NULL);
    sysbus_create_simple("fh-aes", FH_AES_BASE, fh_irq(fms, c->aes_irq));
    /* /machine/sadc: qom-set its chN to move what the inputs read */
    dev = qdev_new("fh-sadc");
    object_property_add_child(OBJECT(machine), "sadc", OBJECT(dev));
    qdev_prop_set_uint32(dev, "version", c->sadc_version);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_SADC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->sadc_irq));

    /* Both channels on one line; the clock is the PMU's tmr0 divider. */
    dev = qdev_new("fh-timer");
    qdev_prop_set_uint32(dev, "num-timers", 2);
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(pmu, "tmr-clk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_TIMER_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->timer_irq));
    qdev_connect_gpio_out_named(pmu, "tmr-reset", 0,
                                qdev_get_gpio_in_named(dev, "reset", 0));

    for (n = 0; n < c->num_uart; n++) {
        dev = qdev_new("fh-uart");
        qdev_prop_set_chr(dev, "chardev", serial_hd(n));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->uart_base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           fh_irq(fms, c->uart_irq[n]));
    }

    for (n = 0; n < 2; n++) {
        static const hwaddr base[2] = { 0xF0300000, 0xF4000000 };

        gpio[n] = qdev_new("fh-gpio");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio[n]), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio[n]), 0,
                           fh_irq(fms, c->gpio_irq[n]));
    }

    dmac = qdev_new(c->ahb_dmac ? "fh-ahb-dmac" : "fh-axi-dmac");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, FH_DMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dmac), 0, fh_irq(fms, c->dmac_irq));

    for (n = 0; n < c->num_spi; n++) {
        spi[n] = qdev_new("fh-spi");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(spi[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(spi[n]), 0, c->spi_base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(spi[n]), 0,
                           fh_irq(fms, c->spi_irq[n]));
        qdev_connect_gpio_out_named(spi[n], "dma-rx-req", 0,
            qdev_get_gpio_in_named(dmac, "dma-req", c->spi_dma_rx[n]));
        qdev_connect_gpio_out_named(spi[n], "dma-tx-req", 0,
            qdev_get_gpio_in_named(dmac, "dma-req", c->spi_dma_tx[n]));
    }

    /*
     * SD0 / SD1: DesignWare MMC hosts, which the vendor fhmci driver runs
     * in IDMAC mode -- the himci model's.  -drive if=sd,index=N is a card
     * in slot N; an empty slot reads "no card" from CDETECT.
     */
    for (n = 0; n < 2; n++) {
        DriveInfo *di = drive_get(IF_SD, 0, n);

        dev = qdev_new("hisi-himci");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0xE2000000 + n * 0x200000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           fh_irq(fms, c->sdc_irq[n]));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    for (n = 0; n < c->num_i2c; n++) {
        dev = qdev_new("fh-i2c");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->i2c_base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           fh_irq(fms, c->i2c_irq[n]));
        if (n == 0) {
            fh_create_sensor(fms->sensor ? fms->sensor : c->sensor,
                             (I2CBus *)qdev_get_child_bus(dev, "i2c"));
        }
    }

    /*
     * Ethernet: a Synopsys GMAC with normal descriptors under the vendor
     * fh_gmac driver (drivers/net/ethernet/fullhan), MAC at 0, DMA at
     * 0x1000.  The driver scans MDIO for a PHY on its list, then attaches
     * the generic PHY code to address 0: an IP101G there suits it.  It
     * takes 4 off every received length (GMAC_DMA_RxStatus() never says
     * llc_snap), so the core keeps the FCS.
     */
    dev = qdev_new("xm-dwmac");
    qemu_configure_nic_device(dev, true, NULL);
    qdev_prop_set_uint32(dev, "regs-size", 0x2000);
    qdev_prop_set_uint32(dev, "phy-id", 0x02430C54);
    qdev_prop_set_uint32(dev, "phy-addr", 0);
    qdev_prop_set_bit(dev, "rx-fcs", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_GMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->gmac_irq));

    /*
     * USB: a Synopsys DWC2 OTG core in host mode, which the 4.9 kernels
     * run under mainline dwc2 ("fh_usb") and the 3.0.8 ones under the
     * vendor's dwc_otg fork (fh_otg): QEMU's model.  -device usb-storage
     * and friends plug into its root port.
     */
    dev = qdev_new("dwc2-usb");
    object_property_add_const_link(OBJECT(dev), "dma-mr", OBJECT(sysmem));
    if (c->usb_fh_otg) {
        /*
         * fh_otg refuses its module parameters (-22) unless the synthesis
         * registers allow them: an HNP/SRP OTG core with a UTMI PHY, 5 device
         * endpoints, 64 KiB transfer and 511 packet counters, a 512-word
         * periodic Tx FIFO, dedicated Tx FIFOs -- and descriptor DMA, which
         * it always uses.
         */
        qdev_prop_set_uint32(dev, "ghwcfg2", 0x228DD550);
        qdev_prop_set_uint32(dev, "ghwcfg3", 0x100000E8);
        qdev_prop_set_uint32(dev, "ghwcfg4", 0x56000000);
        qdev_prop_set_uint32(dev, "hptxfsiz", 0x02000600);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, FH_USB_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, fh_irq(fms, c->usb_irq));

    /* The boot NOR, its select a GPIO through SPI0's cs-gpio. */
    flash = fh_create_flash(fms, qdev_get_child_bus(spi[0], "ssi"), img_len);
    qdev_connect_gpio_out_named(spi[0], SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));
    qdev_connect_gpio_out(gpio[c->flash_cs_gpio / 32], c->flash_cs_gpio % 32,
                          qdev_get_gpio_in_named(spi[0], "cs-gpio", 0));

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = FH_DDR_BASE;
    binfo.board_id = FH_MACH;
    arm_load_kernel(ARM_CPU(cpuobj), machine, &binfo);

    if (!machine->kernel_filename && img) {
        fh_boot_rom_setup(img, img_len, machine, c, fms->flash_file);
    }
}

static void fh_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "fullhan.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        fh_machine_get_flash_file, fh_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (boot header at 0); "
        "erases and programs are written back to it");
    object_class_property_add_str(oc, "flash-type",
        fh_machine_get_flash_type, fh_machine_set_flash_type);
    object_class_property_set_description(oc, "flash-type",
        "m25p80 part the SPI NOR is modelled as (e.g. w25q64); default: "
        "one the vendor drivers know, by image size");
    object_class_property_add_str(oc, "sensor",
        fh_machine_get_sensor, fh_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor on I2C 0 (none, gc4653, gc2053, gc2063, gc2093, "
        "gc1054, mis2006, f37); default: gc4653 on FH885xV200, gc2063 on "
        "FH865x, gc2053 on the V100 dies and FH8626V100, none on FH8632");
}

static const TypeInfo fh_machine_info = {
    .name           = TYPE_FH_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(FhMachineState),
    .class_init     = fh_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void fh_machine_register_types(void)
{
    type_register_static(&fh_machine_info);
}

type_init(fh_machine_register_types)

#define DEFINE_FH_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        fullhan_common_init(machine, &config);                       \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu;                           \
        mc->default_ram_size = config.ram_size_default;              \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, FH_MACHINE, FhMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_FH_MACHINE("fh8852v200", fh8852v200, fh8852v200_soc)
DEFINE_FH_MACHINE("fh8856v200", fh8856v200, fh8856v200_soc)
DEFINE_FH_MACHINE("fh8858v200", fh8858v200, fh8858v200_soc)
DEFINE_FH_MACHINE("fh8652", fh8652, fh8652_soc)
DEFINE_FH_MACHINE("fh8656", fh8656, fh8656_soc)
DEFINE_FH_MACHINE("fh8658", fh8658, fh8658_soc)
DEFINE_FH_MACHINE("fh8852v100", fh8852v100, fh8852v100_soc)
DEFINE_FH_MACHINE("fh8856v100", fh8856v100, fh8856v100_soc)
DEFINE_FH_MACHINE("fh8626v100", fh8626v100, fh8626v100_soc)
DEFINE_FH_MACHINE("fh8632", fh8632, fh8632_soc)
