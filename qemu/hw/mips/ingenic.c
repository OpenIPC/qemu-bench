/*
 * Ingenic T-series IP-camera SoC machines: T10, T20, T21, T23, T30, T31
 * (XBurst1, Linux 3.10) and T40, T41 (XBurst2, Linux 4.4).
 *
 * One little-endian MIPS32 core (two on XBurst2) with DDR from physical
 * 0 and the peripherals from 0x10000000, nearly at the same addresses
 * and interrupt numbers on every part (arch/mips/xburst/soc-<soc>/include/
 * soc/{base,irq}.h of the OpenIPC 3.10 kernels; the T40/T41 device trees
 * of the 4.4 ones):
 *
 *   0x10000000 CPM: clocks, PLLs, gates          hw/misc/ingenic-cpm.c
 *   0x10001000 INTC (64 sources -> IP2)          hw/intc/ingenic-intc.c
 *   0x10002000 TCU, watchdog, boot loader OST    hw/timer/ingenic-tcu.c
 *   0x10003000 RTC / hibernation                 hw/rtc/ingenic-rtc.c
 *   0x10010000 GPIO                              hw/gpio/ingenic-gpio.c
 *   0x10030000 + n * 0x1000  UART0..            16550, 4-byte stride
 *   0x12000000 system OST (-> IP4), the kernel's clock
 *                                                hw/timer/ingenic-ost.c
 *   0x13010000 DDR PHY and APB, 0x134F0000 DDRC  hw/misc/ingenic-ddr.c
 *   0x13440000 SFC + SPI NOR (m25p80) or SPI NAND hw/ssi/ingenic-sfc.c,
 *                                                hw/block/ingenic-spinand.c
 *   0x13450000 T41's second SFC (IRQ 19)
 *   0x13450000 / 0x13460000 MSC0/1 (SD)          hw/sd/ingenic-msc.c
 *   0x134B0000 Synopsys GMAC                     hw/net/ingenic-gmac.c
 *   0x10050000 + n * 0x1000  I2C0..              hw/i2c/ingenic-i2c.c
 *   0x13500000 DWC2 USB OTG, host                QEMU's dwc2-usb
 *   0x13540000 EFUSE (blank)                     hw/misc/ingenic-efuse.c
 *   0x13200000 AVPU video encoder (T31, T40; T41 hw/misc/ingenic-avpu.c
 *              at 0x13100000), interrupt 62
 *   0x13300000 ISP core, 0x133E0000 VIC, 0x10023000 CSI, 0x10022000 D-PHY
 *              (T31, T23; T21: VIC at 0x13310000, no CSI); the sensor=
 *              image sensor on I2C0               hw/misc/ingenic-isp.c
 *   0x13380000 T10's, T20's and T30's Apical ISP core, 0x13300000 VIC,
 *              0x13320000 their interrupt block, 0x10040000 CSI (and T30's
 *              scaler at 0x130B0000, an ingenic-isp)
 *                                                hw/misc/ingenic-isp-apical.c
 *   0x13080000 IPU, interrupt 6: libimp's OSD    hw/misc/ingenic-ipu.c
 *   0x130B0000 I2D, 45, rotation (T40, T41)     hw/misc/ingenic-i2d.c
 *   0x130D0000 DRAWBOX, 42, OSD boxes (T40, T41) hw/misc/ingenic-drawbox.c
 *
 * XBurst2 moves the interrupt controller and timers next to the cores:
 * a global OST at 0x12000000 (the clock source), a per-core OST at
 * 0x12100000 + 0x100 n (IP4), the CCU at 0x12200000 (core reset and
 * entry, mailboxes on IP3, per-core enables) and a per-core INTC at
 * 0x12300000 + 0x100 n (IP2); hw/intc/ingenic-ccu.c has those.
 * Everything else in 0x10000000-0x13FFFFFF reads as zero.
 *
 * Boot, two ways:
 *
 * -kernel boots a uImage (LZMA, gzip or none) or an ELF the way U-Boot's
 *   bootm does: a0 = argc, a1 = argv (argv[1] the command line), a2 =
 *   envp, from a stub in the boot ROM that also leaves UART1's LCR set --
 *   the kernels' early console picks the first UART whose LCR is not
 *   zero.  The kernels size memory from mem= alone (no DT memory node, no
 *   DDR registers), so a command line without one gets the U-Boot
 *   default for the RAM size, mem= plus the rmem= the ISP driver takes.
 *   -initrd is placed at 16 MiB and passed as rd_start/rd_size.
 *
 * flash-file= boots a camera's SPI NOR image as the mask ROM does: it
 *   checks the SPL header at offset 0 (06 05 04 03 02 55 aa 55 aa, CRC7,
 *   length at 0xC), copies the SPL to 0x80001000 and enters it at
 *   0x80001800 with sp at 0x80008000, which the SPL never sets.  That is
 *   cache-as-RAM on silicon, and here too: the CPU's car-size window
 *   (qemu/patches/0016) keeps KSEG0 0x80000000-0x80007FFF out of DDR
 *   until the SPL jumps on, because the T41 SPL tests all of DDR through
 *   KSEG1 underneath itself.  The SPL brings up clocks and
 *   DDR, loads U-Boot from flash 0x6800 and U-Boot boots the kernel from
 *   flash.  Erases and programs are written back to the file.
 *
 *   An image of SPI NAND size (128/256 MiB of page data, or the pages
 *   with their OOB) is a SPI NAND on the same SFC, and boots the mask
 *   ROM's NAND way: the same header with byte 8 = 00 and the geometry in
 *   bytes 10-11, the SPL from the data of the first pages
 *   (ingenic_nand_boot); the SFC NAND SPL loads U-Boot from 0x6C00.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qobject/qdict.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/bitbang_i2c.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/mips/bootloader.h"
#include "hw/mips/ingenic.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/sd/sdhci.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "hw/arm/nvt-lzma.h"
#include "elf.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/mips/cpu.h"

#define TYPE_INGENIC_MACHINE MACHINE_TYPE_NAME("ingenic")
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMachineState, INGENIC_MACHINE)

#define BOOTROM_BASE    0x1FC00000
#define BOOTROM_SIZE    0x10000
#define CAR_BASE        0x1F000000  /* the cache lines the ROM locks */
#define CAR_SIZE        0x8000
#define ORAM_BASE       0x12660000  /* XBurst2 on-chip RAM */
#define IVDC_STORE_BASE 0x80000000  /* T41 direct_mode: the IVDC's frame */
#define IVDC_STORE_SIZE 0x02000000
#define ORAM_SIZE       0x10000
#define IO_BASE         0x10000000
#define IO_SIZE         0x04000000

#define CPM_BASE        0x10000000
#define INTC_BASE       0x10001000
#define TCU_BASE        0x10002000
#define RTC_BASE        0x10003000
#define GPIO_BASE       0x10010000
#define UART_BASE       0x10030000
#define I2C_BASE        0x10050000
#define OTG_BASE        0x13500000
#define EFUSE_BASE      0x13540000
#define HARB0_BASE      0x13000000
#define AIC_BASE        0x10020000
#define CODEC_BASE      0x10021000
#define PDMA_BASE       0x13420000
#define IRQ_AIC         1
#define IRQ_PDMA        10
#define DMIC_BASE       0x10034000  /* T31, T40 */
#define IRQ_DMIC        12
#define GMAC_BASE       0x134B0000
#define OST_BASE        0x12000000
#define DDR_PHY_BASE    0x13010000
#define DDRC_BASE       0x134F0000
#define SFC_BASE        0x13440000
#define SFC1_BASE       0x13450000  /* T41's second SFC (NAND) */
#define CCU_BASE        0x12200000
#define CORE_OST_BASE   0x12100000
#define CORE_INTC_BASE  0x12300000
#define MSC0_BASE       0x13450000
#define MSC1_BASE       0x13460000
#define SDHCI0_BASE     0x13060000
#define SDHCI1_BASE     0x13070000
#define AVPU_BASE       0x13200000
#define AVPU_BASE_T41   0x13100000
#define ISP_BASE        0x13300000
#define VIC_BASE        0x133E0000
#define CSI_BASE        0x10023000
#define DPHY_BASE       0x10022000

#define IRQ_SFC         7
#define IRQ_SFC1        19
#define IRQ_GPIO0       17
#define IRQ_TCU0        27
#define IRQ_TCU1        26
#define IRQ_TCU2        25
#define IRQ_UART0       51
#define IRQ_I2C0        60
#define IRQ_OTG         21
#define IRQ_GMAC        55
#define IRQ_MSC0        37
#define IRQ_MSC1        36
#define IRQ_AVPU        62      /* HELIX0: Linux IRQ 70 */
#define IRQ_ISP         29      /* Linux 37, "isp-m0" */
#define IRQ_VIC         30      /* Linux 38, "isp-w02" */
#define IRQ_SCALER_T30  19      /* Linux 27, "isp-m3" */
#define IPU_BASE        0x13080000
#define SADC_BASE       0x10070000
#define IRQ_SADC        18
#define USBPHY_BASE     0x10060000  /* the OTG PHY's own registers */
#define DTRNG_BASE      0x10072000
#define AES_BASE        0x13430000
#define HASH_BASE       0x13480000
#define RSA_BASE        0x134C0000
#define SSI0_BASE       0x10043000
#define SSI1_BASE       0x10044000
#define IRQ_SSI0        9
#define IRQ_SSI1        8
#define IRQ_HASH        22
#define IRQ_AES         23
#define IRQ_DTRNG       34
#define PWM_BASE        0x13460000  /* T41; the others use TCU channels */
#define IRQ_IPU         6       /* Linux 14, "ipu" */
#define I2D_BASE        0x130B0000  /* T40, T41 */
#define DRAWBOX_BASE    0x130D0000  /* T40, T41 */
#define IRQ_DRAWBOX     42
#define IRQ_I2D         45

/* the SPL's place in the boot ROM's world (U-Boot isvp_<soc>.h) */
#define SPL_LOAD        0x80001000
#define SPL_ENTRY       0x80001800
#define SPL_SP          0x80008000
#define SPL_MAX         0x6800
#define SPL_MAX_NAND    0x7000      /* to the top of the cache-as-RAM */

#define KERNEL_ARGS_RESERVE (64 * KiB)
#define INITRD_OFFSET   (16 * MiB)

typedef struct IngenicSocInfo {
    const char *name;
    const char *desc;
    IngenicSoc soc;
    const char *cpu;
    uint64_t ram_default;
    int num_uarts;
    int gpio_ports;
    uint32_t gpio_stride, gpio_shadow;
    uint32_t sfc_version;
    const char *mtd_id;     /* the kernel's SFC mtd name */
    const char *nand_mtd_id; /* ... with a SPI NAND */
    bool has_sfc1;          /* a second SFC at 0x13450000 */
    hwaddr rtc_base;
    int max_cpus;
    int num_i2c;
    int msc0_cd;            /* MSC0 card-detect GPIO, port * 32 + pin */
    uint32_t soc_id;        /* HARB0 0x2C */
    uint32_t prid;          /* 0: the CPU model's */
    uint32_t l1_kib;        /* 0: the CPU model's */
    const char *isp;        /* ingenic-isp layout, NULL: none */
    hwaddr isp_base, vic_base, csi_base, dphy_base;  /* csi 0: none */
    hwaddr scaler_base;     /* T30's MSCA scaler (ingenic-isp "t30") */
    int irq_isp, irq_vic;   /* INTC sources, 0: IRQ_ISP / IRQ_VIC */
    int sensor_bus;         /* the I2C bus sinfo / tx-isp probe */
    int sensor1_bus;        /* > 0: sensor1= (a second sensor) is here */
    int sensor_gpio_sda;    /* > 0: the sensor bus is bit-banged on GPIOs
                               sda / sda + 1 (T10's i2c-gpio) */
} IngenicSocInfo;

/*
 * The grades within a family, as ipctool (src/hal/ingenic.c) tells them
 * apart: CPPSR's low byte, and the code in the upper half of the EFUSE's
 * SUBSOCTYPE1 (0x238; SUBSOCTYPE2, 0x250, on XBurst2).  The first entry
 * of each SoC is its default.
 */
typedef struct IngenicVariant {
    const char *name;
    IngenicSoc soc;
    uint8_t cppsr;
    uint16_t code;
} IngenicVariant;

static const IngenicVariant ingenic_variants[] = {
    { "t10",   INGENIC_T10, 0x01, 0 },
    { "t10l",  INGENIC_T10, 0x10, 0 },
    { "t20",   INGENIC_T20, 0x01, 0x1111 },
    { "t20l",  INGENIC_T20, 0x10, 0x3333 },
    { "t20x",  INGENIC_T20, 0x01, 0x2222 },
    { "t21n",  INGENIC_T21, 0x01, 0x1111 },
    { "t21l",  INGENIC_T21, 0x01, 0x3333 },
    { "t21x",  INGENIC_T21, 0x01, 0x2222 },
    { "t21z",  INGENIC_T21, 0x01, 0x5555 },
    { "t23n",  INGENIC_T23, 0x01, 0x1111 },
    { "t23x",  INGENIC_T23, 0x01, 0x2222 },
    { "t30n",  INGENIC_T30, 0x01, 0x1111 },
    { "t30l",  INGENIC_T30, 0x10, 0 },
    { "t30x",  INGENIC_T30, 0x01, 0x2222 },
    { "t30a",  INGENIC_T30, 0x01, 0x4444 },
    { "t30z",  INGENIC_T30, 0x01, 0x5555 },
    { "t31n",  INGENIC_T31, 0x01, 0x1111 },
    { "t31l",  INGENIC_T31, 0x01, 0x3333 },
    { "t31x",  INGENIC_T31, 0x01, 0x2222 },
    { "t31a",  INGENIC_T31, 0x01, 0x4444 },
    { "t31zl", INGENIC_T31, 0x01, 0x5555 },
    { "t31zx", INGENIC_T31, 0x01, 0x6666 },
    { "t31al", INGENIC_T31, 0x01, 0xcccc },
    { "t31zc", INGENIC_T31, 0x01, 0xdddd },
    { "t31lc", INGENIC_T31, 0x01, 0xeeee },
    { "t40n",  INGENIC_T40, 0x01, 0x1111 },
    { "t40nn", INGENIC_T40, 0x01, 0x8888 },
    { "t40xp", INGENIC_T40, 0x01, 0x7777 },
    { "t40a",  INGENIC_T40, 0x01, 0x4444 },
    { "t41nq", INGENIC_T41, 0x01, 0xaaaa },
    { "t41n",  INGENIC_T41, 0x01, 0x1111 },
    { "t41l",  INGENIC_T41, 0x01, 0x3333 },
    { "t41lq", INGENIC_T41, 0x01, 0x9999 },
    { "t41lc", INGENIC_T41, 0x01, 0x8888 },
    { "t41zl", INGENIC_T41, 0x01, 0x5555 },
    { "t41zn", INGENIC_T41, 0x01, 0x7777 },
    { "t41x",  INGENIC_T41, 0x01, 0x6666 },
};

static const IngenicVariant *ingenic_variant(IngenicSoc soc, const char *name)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ingenic_variants); i++) {
        if (ingenic_variants[i].soc == soc &&
            (!name || !strcmp(name, ingenic_variants[i].name))) {
            return &ingenic_variants[i];
        }
    }
    return NULL;
}

struct IngenicMachineState {
    MachineState parent_obj;

    char *flash_file;
    char *flash_type;
    char *variant;
    char *sensor;
    DeviceState *sensor_dev;        /* the sensor= model, for the ISP */
    char *sensor1;
    DeviceState *sensor1_dev;       /* sensor1=, T40's second pipe */
    const IngenicSocInfo *info;
    MIPSCPU *cpu[2];
    Clock *cpuclk;
    DeviceState *intc, *sfc, *sfc1;
    DeviceState *nand;      /* the boot flash is a SPI NAND */
};

typedef struct IngenicMachineClass {
    MachineClass parent_class;
    const IngenicSocInfo *info;
} IngenicMachineClass;

DECLARE_CLASS_CHECKERS(IngenicMachineClass, INGENIC_MACHINE,
                       TYPE_INGENIC_MACHINE)

/* --- interrupt lines ---------------------------------------------------- */

static qemu_irq ingenic_irq(IngenicMachineState *ims, int n)
{
    return qdev_get_gpio_in(ims->intc, n);
}

/* --- the video encoders before the AVPU ----------------------------------- */

/*
 * T10/T20: the VPU ("jz-vpu.0"); T21/T23: HELIX, which takes its chain at
 * 0x84; T30: HELIX and RADIX (H.265).  Bases and interrupts are the
 * kernels' platform.c / soc/irq.h (Linux IRQ = INTC source + 8).
 */
static void ingenic_vpu_one(IngenicMachineState *ims, DeviceState *cpm,
                            hwaddr base, int irq, uint32_t kind,
                            uint32_t taskrg, int srbc_sr)
{
    DeviceState *dev = qdev_new("ingenic-vpu");
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_prop_set_uint32(dev, "kind", kind);
    qdev_prop_set_uint32(dev, "taskrg", taskrg);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, irq));
    /* the kernel resets the engine through CPM SRBC before every task */
    qdev_connect_gpio_out_named(cpm, "srbc", srbc_sr,
                                qdev_get_gpio_in_named(dev, "reset", 0));
}

static void ingenic_vpu_create(IngenicMachineState *ims, DeviceState *cpm,
                               IngenicSoc soc)
{
    switch (soc) {
    case INGENIC_T10:
    case INGENIC_T20:
        ingenic_vpu_one(ims, cpm, 0x13200000, 62, 0, 0x10008, 31);
        break;
    case INGENIC_T21:
        ingenic_vpu_one(ims, cpm, 0x13200000, 62, 0, 0x84, 31);
        break;
    case INGENIC_T23:
        ingenic_vpu_one(ims, cpm, 0x13100000, 40, 0, 0x84, 31);
        break;
    case INGENIC_T30:
        ingenic_vpu_one(ims, cpm, 0x13200000, 62, 0, 0x10008, 31);
        ingenic_vpu_one(ims, cpm, 0x13600000, 45, 1, 0, 28);
        break;
    default:
        break;                      /* T31 and later: the AVPU */
    }
}

/* --- flash -------------------------------------------------------------- */

#define TYPE_SPINAND    "ingenic-spinand"

static const char *ingenic_flash_part(int64_t size)
{
    if (size <= 8 * MiB) {
        return "w25q64";            /* 0xef4017 */
    } else if (size <= 16 * MiB) {
        return "mx25l12805d";       /* 0xc22018, in every vendor table */
    }
    return "w25q256";               /* 0xef4019, 4-byte addresses */
}

/*
 * A SPI NAND image is the data of every page (128 or 256 MiB), or every
 * page with its OOB after it; the size picks the part.  Parts with the
 * same geometry (w25n01gv and gd5f1gq4ub without OOB) need flash-type=.
 */
static const struct {
    int64_t size;
    const char *part;
} ingenic_nand_sizes[] = {
    { 128 * MiB, "w25n01gv" },              /* 2048 + 64, 1 Gbit */
    { 65536 * (2048 + 64), "w25n01gv" },    /* 132 MiB */
    { 65536 * (2048 + 128), "gd5f1gq4ub" }, /* 136 MiB */
    { 256 * MiB, "gd5f2gq4ub" },            /* 2048 + 128, 2 Gbit */
    { 131072 * (2048 + 128), "gd5f2gq4ub" },/* 272 MiB */
    { 131072 * (2048 + 64), "mx35lf2ge4ab" },/* 264 MiB, two planes */
};

static const char *ingenic_nand_part(int64_t size)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ingenic_nand_sizes); i++) {
        if (ingenic_nand_sizes[i].size == size) {
            return ingenic_nand_sizes[i].part;
        }
    }
    return NULL;
}

static BlockBackend *ingenic_open_flash(const char *file)
{
    QDict *opts = qdict_new();
    Error *err = NULL;
    BlockBackend *blk;

    qdict_put_str(opts, "driver", "raw");
    blk = blk_new_open(file, NULL, opts, BDRV_O_RDWR, &err);
    if (!blk) {
        error_reportf_err(err, "ingenic: %s: ", file);
        exit(1);
    }
    return blk;
}

/* A NOR (m25p80) or SPI NAND (ingenic-spinand) chip on an SFC's bus. */
static DeviceState *ingenic_attach_flash(DeviceState *sfc, BlockBackend *blk,
                                         const char *type)
{
    int64_t size = blk ? blk_getlength(blk) : 16 * MiB;
    DeviceState *flash;

    if (!type) {
        type = size > 32 * MiB ? ingenic_nand_part(size) :
               ingenic_flash_part(size);
    }
    if (!type) {
        error_report("ingenic: a %" PRId64 "-byte flash image is neither a "
                     "NOR (up to 32 MiB) nor a SPI NAND image (128/256 MiB "
                     "of data, or with the OOB: 132/136/264/272 MiB); "
                     "flash-type= names the part", size);
        exit(1);
    }
    if (!object_class_by_name(type)) {
        error_report("ingenic: flash-type %s is not a flash part", type);
        exit(1);
    }
    flash = qdev_new(type);
    if (blk) {
        qdev_prop_set_drive_err(flash, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(flash, qdev_get_child_bus(sfc, "spi"),
                           &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(sfc), 1,
                       qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));
    return flash;
}

static void ingenic_create_flash(IngenicMachineState *ims)
{
    BlockBackend *blk = NULL;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    DeviceState *flash;

    if (ims->flash_file) {
        blk = ingenic_open_flash(ims->flash_file);
    } else if (di) {
        blk = blk_by_legacy_dinfo(di);
    }
    flash = ingenic_attach_flash(ims->sfc, blk, ims->flash_type);
    if (object_dynamic_cast(OBJECT(flash), TYPE_SPINAND)) {
        ims->nand = flash;
    }
    if (ims->flash_file) {
        /* the device holds its own reference on the node (cf. grainmedia) */
        blk_unref(blk);
    }

    /* T41's second SFC: -drive if=mtd,index=1, NOR or NAND by size */
    di = drive_get(IF_MTD, 0, 1);
    if (ims->sfc1 && di) {
        ingenic_attach_flash(ims->sfc1, blk_by_legacy_dinfo(di), NULL);
    }
}

/* --- boot: mask ROM stand-in -------------------------------------------- */

static const uint8_t spl_magic[9] = {
    0x06, 0x05, 0x04, 0x03, 0x02, 0x55, 0xaa, 0x55, 0xaa,
};

static uint8_t crc7(const uint8_t *p, size_t len)
{
    uint8_t crc = 0;
    size_t i;
    int b;

    for (i = 0; i < len; i++) {
        crc ^= p[i];
        for (b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ (0x09 << 1) : crc << 1;
        }
    }
    return crc >> 1;
}

static void ingenic_flash_boot(IngenicMachineState *ims)
{
    g_autofree uint8_t *spl = NULL;
    g_autoptr(GError) gerr = NULL;
    uint8_t rom[64];
    void *p = rom;
    gsize len;
    uint32_t spl_len;

    if (!g_file_get_contents(ims->flash_file, (gchar **)&spl, &len, &gerr)) {
        error_report("ingenic: cannot read %s: %s", ims->flash_file,
                     gerr->message);
        exit(1);
    }
    if (len < 0x800 + 16 || memcmp(spl, spl_magic, sizeof(spl_magic))) {
        error_report("ingenic: %s has no SPL header at offset 0 "
                     "(06 05 04 03 02 55 aa 55 aa)", ims->flash_file);
        exit(1);
    }
    spl_len = ldl_le_p(spl + 12);
    if (spl_len <= 0x800 || spl_len > SPL_MAX || spl_len > len) {
        error_report("ingenic: SPL length 0x%x in %s is out of range",
                     spl_len, ims->flash_file);
        exit(1);
    }
    if (crc7(spl + 0x800, spl_len - 0x800) != spl[9]) {
        warn_report("ingenic: SPL CRC7 mismatch in %s; the mask ROM "
                    "would refuse it", ims->flash_file);
    }

    rom_add_blob_fixed("ingenic.spl", spl, spl_len,
                       CAR_BASE + (SPL_LOAD & (CAR_SIZE - 1)));
    bl_gen_jump_kernel(&p, true, SPL_SP, false, 0, false, 0, false, 0,
                       false, 0, SPL_ENTRY);
    rom_add_blob_fixed("ingenic.bootrom", rom, (uint8_t *)p - rom,
                       BOOTROM_BASE);
}

/*
 * The mask ROM's SFC NAND path.  The SPL header is the NOR one with byte
 * 8 = 00 (NOR: aa) and the geometry in bytes 10-11 (start.S):
 * PPB / 32 and BPP / 1024, BPP being page + OOB in bytes (2112 -> 2).
 * The SPL is in the data area of the first pages, the length at 0xC and
 * the CRC7 at 9 as on NOR; the ROM reads it with the on-die ECC on and
 * never looks at the OOB.  The U-Boot SFC NAND SPL then loads U-Boot from
 * data offset 0x6C00 (27 KiB) with page reads of its own.
 */
static void ingenic_nand_boot(IngenicMachineState *ims)
{
    g_autofree uint8_t *img = NULL, *spl = NULL;
    g_autoptr(GError) gerr = NULL;
    uint32_t page = object_property_get_uint(OBJECT(ims->nand), "page-size",
                                             &error_abort);
    uint32_t oob = object_property_get_uint(OBJECT(ims->nand), "oob-size",
                                            &error_abort);
    uint32_t ppb = object_property_get_uint(OBJECT(ims->nand),
                                            "pages-per-block", &error_abort);
    uint32_t pages = object_property_get_uint(OBJECT(ims->nand), "pages",
                                              &error_abort);
    uint32_t stride, spl_len, hdr_ppb, hdr_bpp, i;
    uint8_t rom[64];
    void *p = rom;
    gsize len;

    if (!g_file_get_contents(ims->flash_file, (gchar **)&img, &len, &gerr)) {
        error_report("ingenic: cannot read %s: %s", ims->flash_file,
                     gerr->message);
        exit(1);
    }
    stride = len == (gsize)pages * (page + oob) ? page + oob : page;

    /* the data area of the pages the SPL can fill */
    spl = g_malloc(SPL_MAX_NAND);
    for (i = 0; i * page < SPL_MAX_NAND; i++) {
        memcpy(spl + i * page, img + (gsize)i * stride,
               MIN(page, SPL_MAX_NAND - i * page));
    }
    if (memcmp(spl, spl_magic, 8) || spl[8] == 0xaa) {
        error_report("ingenic: %s has no SPI NAND SPL header in page 0 "
                     "(06 05 04 03 02 55 aa 55 00)%s", ims->flash_file,
                     spl[8] == 0xaa ? ": it is a NOR SPL" : "");
        exit(1);
    }
    hdr_ppb = spl[10] * 32;
    hdr_bpp = spl[11] * 1024;
    if (hdr_ppb != ppb || hdr_bpp / 1024 != (page + oob) / 1024) {
        warn_report("ingenic: the SPL header of %s says %u pages per block "
                    "and %u-byte pages, the %s has %u and %u + %u",
                    ims->flash_file, hdr_ppb, hdr_bpp,
                    object_get_typename(OBJECT(ims->nand)), ppb, page, oob);
    }
    spl_len = ldl_le_p(spl + 12);
    if (spl_len <= 0x800 || spl_len > SPL_MAX_NAND) {
        error_report("ingenic: SPL length 0x%x in %s is out of range",
                     spl_len, ims->flash_file);
        exit(1);
    }
    if (crc7(spl + 0x800, spl_len - 0x800) != spl[9]) {
        warn_report("ingenic: SPL CRC7 mismatch in %s; the mask ROM "
                    "would refuse it", ims->flash_file);
    }

    rom_add_blob_fixed("ingenic.spl", spl, spl_len,
                       CAR_BASE + (SPL_LOAD & (CAR_SIZE - 1)));
    bl_gen_jump_kernel(&p, true, SPL_SP, false, 0, false, 0, false, 0,
                       false, 0, SPL_ENTRY);
    rom_add_blob_fixed("ingenic.bootrom", rom, (uint8_t *)p - rom,
                       BOOTROM_BASE);
}

/* --- boot: -kernel ------------------------------------------------------ */

#define UIMAGE_MAGIC    0x27051956
#define UIMAGE_HDR      64

/* Load a uImage or ELF; returns the entry point, sets *end (physical). */
static uint64_t ingenic_load_kernel(const char *file, hwaddr *end)
{
    g_autofree uint8_t *img = NULL;
    g_autoptr(GError) gerr = NULL;
    g_autofree uint8_t *data = NULL;
    uint64_t entry, high;
    uint32_t load, ep, dlen;
    ssize_t n;
    gsize len;

    if (load_elf(file, NULL, cpu_mips_kseg0_to_phys, NULL, &entry, NULL,
                 &high, NULL, ELFDATA2LSB, EM_MIPS, 1, 0) > 0) {
        *end = cpu_mips_kseg0_to_phys(NULL, high);
        return entry;
    }
    if (!g_file_get_contents(file, (gchar **)&img, &len, &gerr)) {
        error_report("ingenic: cannot read kernel %s: %s", file,
                     gerr->message);
        exit(1);
    }
    if (len < UIMAGE_HDR || ldl_be_p(img) != UIMAGE_MAGIC) {
        error_report("ingenic: %s is neither an ELF nor a uImage", file);
        exit(1);
    }
    dlen = ldl_be_p(img + 12);
    load = ldl_be_p(img + 16);
    ep = ldl_be_p(img + 20);
    if (dlen > len - UIMAGE_HDR) {
        error_report("ingenic: %s is truncated", file);
        exit(1);
    }
    switch (img[31]) {
    case 0:     /* none */
        data = g_memdup2(img + UIMAGE_HDR, dlen);
        n = dlen;
        break;
    case 1:     /* gzip */
        data = g_malloc(64 * MiB);
        n = gunzip(data, 64 * MiB, img + UIMAGE_HDR, dlen);
        break;
    case 3:     /* lzma */
        data = g_malloc(64 * MiB);
        n = nvt_lzma_decode(img + UIMAGE_HDR, dlen, data, 64 * MiB);
        break;
    default:
        error_report("ingenic: %s: unsupported uImage compression %d",
                     file, img[31]);
        exit(1);
    }
    if (n <= 0) {
        error_report("ingenic: %s: cannot decompress the kernel", file);
        exit(1);
    }
    rom_add_blob_fixed("ingenic.kernel", data, n,
                       cpu_mips_kseg0_to_phys(NULL, load));
    *end = cpu_mips_kseg0_to_phys(NULL, load) + n;
    return (int32_t)ep;
}

/* mem=/rmem= as the OpenIPC U-Boot environment has them (isvp_common.h) */
static char *ingenic_mem_args(uint64_t ram)
{
    uint64_t os = ram >= 128 * MiB ? ram / 2 : ram * 5 / 8;

    return g_strdup_printf("mem=%" PRIu64 "M rmem=%" PRIu64 "M@0x%" PRIx64,
                           os / MiB, (ram - os) / MiB, os);
}

static void ingenic_kernel_boot(IngenicMachineState *ims)
{
    MachineState *ms = MACHINE(ims);
    const IngenicSocInfo *info = ims->info;
    g_autofree char *cmdline = NULL, *mem = NULL, *rd = NULL;
    const char *append = ms->kernel_cmdline;
    hwaddr kend, args, initrd = 0;
    int64_t initrd_size = 0;
    uint64_t entry;
    uint32_t *argv;
    char *s;
    uint8_t rom[256];
    void *p = rom;
    size_t pos;
    uint8_t *buf;

    entry = ingenic_load_kernel(ms->kernel_filename, &kend);

    if (ms->initrd_filename) {
        initrd = INITRD_OFFSET;
        if (initrd < kend) {
            initrd = ROUND_UP(kend, 64 * KiB);
        }
        initrd_size = load_image_targphys(ms->initrd_filename, initrd,
                                          ms->ram_size - initrd -
                                          KERNEL_ARGS_RESERVE, NULL);
        if (initrd_size < 0) {
            error_report("ingenic: cannot load initrd %s",
                         ms->initrd_filename);
            exit(1);
        }
        rd = g_strdup_printf(" rd_start=0x%08" PRIx32 " rd_size=%" PRId64,
                             (uint32_t)cpu_mips_phys_to_kseg0(NULL, initrd),
                             initrd_size);
    }
    if (!append || !*append) {
        append = NULL;
    }
    mem = (append && strstr(append, "mem=")) ? g_strdup("") :
          ingenic_mem_args(ms->ram_size);
    cmdline = g_strdup_printf("%s%s%s%s%s",
        append ? append : "console=ttyS1,115200n8 panic=20 init=/init",
        append ? "" : (initrd ? " root=/dev/ram0 rootfstype=squashfs" :
                       ims->nand ? " ubi.mtd=2 ubi.block=0,0 "
                                   "root=/dev/ubiblock0_0 "
                                   "rootfstype=squashfs" :
                       " root=/dev/mtdblock3 rootfstype=squashfs"),
        *mem ? " " : "", mem, rd ? rd : "");
    if (!append) {
        s = cmdline;
        if (ims->nand) {
            /* qemu-boot/ingenic-nand-image.sh's layout */
            cmdline = g_strdup_printf("%s mtdparts=%s:1M(boot),8M(kernel),"
                                      "40M(rootfs),-(data)", s,
                                      info->nand_mtd_id);
        } else {
            cmdline = g_strdup_printf("%s mtdparts=%s:256k(boot),64k(env),"
                                      "2048k(kernel),5120k(rootfs),"
                                      "-(rootfs_data)", s, info->mtd_id);
        }
        g_free(s);
    }
    if (initrd && initrd + initrd_size > ms->ram_size * 5 / 8 &&
        !(append && strstr(append, "mem="))) {
        warn_report("ingenic: the initrd ends past mem=; pass a larger "
                    "-m or a mem= that covers 0x%" PRIx64,
                    (uint64_t)(initrd + initrd_size));
    }

    /* argv[0] = "", argv[1] = command line, envp = { memsize, NULL } */
    args = ms->ram_size - KERNEL_ARGS_RESERVE;
    buf = g_malloc0(KERNEL_ARGS_RESERVE);
    argv = (uint32_t *)buf;
    pos = 32;
    argv[0] = cpu_to_le32(cpu_mips_phys_to_kseg0(NULL, args + pos));
    pos += 1;
    argv[1] = cpu_to_le32(cpu_mips_phys_to_kseg0(NULL, args + pos));
    pstrcpy((char *)buf + pos, KERNEL_ARGS_RESERVE - pos - 64, cmdline);
    pos += strlen(cmdline) + 1;
    argv[2] = 0;
    /* envp at +16 */
    argv[4] = cpu_to_le32(cpu_mips_phys_to_kseg0(NULL, args + pos));
    argv[5] = 0;
    snprintf((char *)buf + pos, 32, "memsize=%" PRIu64, ms->ram_size / MiB);
    rom_add_blob_fixed("ingenic.args", buf, KERNEL_ARGS_RESERVE, args);
    g_free(buf);

    /* what U-Boot leaves: UART1 programmed (LCR 8N1), then bootm */
    bl_gen_write_u32(&p, 0xb0000000 | (UART_BASE + 0x1000 + 0x0c), 0x03);
    bl_gen_jump_kernel(&p, true, cpu_mips_phys_to_kseg0(NULL, args),
                       true, 2,
                       true, cpu_mips_phys_to_kseg0(NULL, args),
                       true, cpu_mips_phys_to_kseg0(NULL, args + 16),
                       true, 0, entry);
    rom_add_blob_fixed("ingenic.bootrom", rom, (uint8_t *)p - rom,
                       BOOTROM_BASE);
}

/* --- image sensors ------------------------------------------------------ */

/*
 * sensor=: the parts the OpenIPC rootfs has drivers for that a repo model
 * answers, at the address and with the ID sinfo.ko's table probes
 * (g_sinfo; the sensor drivers' _detect reads the same registers).  The
 * sensor sits on I2C0 (I2C1 on T40); MCLK and the PA18 reset pulse need
 * nothing here -- the models answer whether or not they are clocked.
 */
static const struct {
    const char *name;
    const char *type;           /* NULL: galaxycore-sensor */
    uint8_t addr;
    bool reg16;                 /* galaxycore: 16-bit register addresses */
    uint16_t id_reg, id;        /* galaxycore / hisi-smartsens; ov2735 rev */
} ingenic_sensors[] = {
    { "gc2053", NULL, 0x37, false, 0xF0, 0x2053 },
    { "gc2083", NULL, 0x37, true, 0x03F0, 0x2083 },
    { "sc2336", "hisi-smartsens", 0x30, true, 0x3107, 0xCB3A },
    { "sc2335", "hisi-smartsens", 0x30, true, 0x3107, 0xCB14 },
    { "jxf37", "hisi-f37", 0x40 },
    { "jxf22", "hisi-jxf22", 0x40 },
    { "jxh62", "hisi-jxh62", 0x30 },
    { "gc4653", NULL, 0x29, true, 0x03F0, 0x4653 },
    { "gc4663", NULL, 0x29, true, 0x03F0, 0x4653 },
    { "ov2735b", "ingenic-ov2735", 0x3C, false, 0, 0x07 },
    { "ov2735", "ingenic-ov2735", 0x3C, false, 0, 0x05 },
};

static DeviceState *ingenic_create_sensor(const char *name, I2CBus *bus)
{
    DeviceState *dev;
    int n;

    for (n = 0; n < ARRAY_SIZE(ingenic_sensors); n++) {
        if (!strcmp(name, ingenic_sensors[n].name)) {
            break;
        }
    }
    if (n == ARRAY_SIZE(ingenic_sensors)) {
        error_report("ingenic: unknown sensor '%s'; one of: none", name);
        for (n = 0; n < ARRAY_SIZE(ingenic_sensors); n++) {
            error_printf("  %s\n", ingenic_sensors[n].name);
        }
        exit(1);
    }
    if (!ingenic_sensors[n].type) {
        dev = DEVICE(i2c_slave_new("galaxycore-sensor",
                                   ingenic_sensors[n].addr));
        qdev_prop_set_bit(dev, "reg16", ingenic_sensors[n].reg16);
        qdev_prop_set_uint16(dev, "id-reg", ingenic_sensors[n].id_reg);
        qdev_prop_set_uint16(dev, "id", ingenic_sensors[n].id);
    } else {
        dev = DEVICE(i2c_slave_new(ingenic_sensors[n].type,
                                   ingenic_sensors[n].addr));
        if (!strcmp(ingenic_sensors[n].type, "hisi-smartsens")) {
            qdev_prop_set_uint8(dev, "id_high", ingenic_sensors[n].id >> 8);
            qdev_prop_set_uint8(dev, "id_low", ingenic_sensors[n].id & 0xFF);
        } else if (!strcmp(ingenic_sensors[n].type, "ingenic-ov2735")) {
            qdev_prop_set_uint8(dev, "rev", ingenic_sensors[n].id);
        }
    }
    i2c_slave_realize_and_unref(I2C_SLAVE(dev), bus, &error_fatal);
    return dev;
}

/* --- machine ------------------------------------------------------------ */

static void ingenic_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void ingenic_init(MachineState *ms)
{
    IngenicMachineState *ims = INGENIC_MACHINE(ms);
    const IngenicSocInfo *info = INGENIC_MACHINE_GET_CLASS(ms)->info;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *rom = g_new(MemoryRegion, 1);
    MemoryRegion *car = g_new(MemoryRegion, 1);
    MemoryRegion *usbphy;
    const IngenicVariant *var;
    CPUMIPSState *env;
    DeviceState *dev, *gpio, *cpm, *pdma;
    SysBusDevice *sbd;
    int i;

    ims->info = info;
    var = ingenic_variant(info->soc, ims->variant);
    if (!var) {
        error_report("ingenic: %s is not a %s grade; one of:", ims->variant,
                     info->name);
        for (i = 0; i < ARRAY_SIZE(ingenic_variants); i++) {
            if (ingenic_variants[i].soc == info->soc) {
                error_printf("  %s\n", ingenic_variants[i].name);
            }
        }
        exit(1);
    }
    if (ms->ram_size > 256 * MiB) {
        error_report("ingenic: at most 256 MiB of RAM fit below the I/O "
                     "space");
        exit(1);
    }

    ims->cpuclk = clock_new(OBJECT(ms), "cpu-refclk");
    clock_set_hz(ims->cpuclk, 24000000);
    for (i = 0; i < ms->smp.cpus; i++) {
        dev = qdev_new(ms->cpu_type);
        qdev_connect_clock_in(dev, "clk-in", ims->cpuclk);
        object_property_set_bool(OBJECT(dev), "big-endian", false,
                                 &error_abort);
        object_property_set_uint(OBJECT(dev), "car-size", CAR_SIZE,
                                 &error_abort);
        object_property_set_uint(OBJECT(dev), "car-phys", CAR_BASE,
                                 &error_abort);
        object_property_set_uint(OBJECT(dev), "prid", info->prid,
                                 &error_abort);
        object_property_set_uint(OBJECT(dev), "l1-kib", info->l1_kib,
                                 &error_abort);
        /*
         * No hidden tlbwr shadow entries: on two cores the T40/T41 4.4
         * kernel otherwise leaves a user page translated to stale data
         */
        object_property_set_bool(OBJECT(dev), "tlb-shadow", false,
                                 &error_abort);
        /* the CCU releases the second core */
        object_property_set_bool(OBJECT(dev), "start-powered-off", i > 0,
                                 &error_abort);
        qdev_realize(dev, NULL, &error_fatal);
        ims->cpu[i] = MIPS_CPU(dev);
        cpu_mips_irq_init_cpu(ims->cpu[i]);
        cpu_mips_clock_init(ims->cpu[i]);
        qemu_register_reset(ingenic_cpu_reset, ims->cpu[i]);
    }
    env = &ims->cpu[0]->env;

    memory_region_add_subregion(sysmem, 0, ms->ram);
    memory_region_init_rom(rom, NULL, "ingenic.bootrom", BOOTROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, BOOTROM_BASE, rom);
    memory_region_init_ram(car, NULL, "ingenic.car", CAR_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CAR_BASE, car);
    create_unimplemented_device("ingenic.io", IO_BASE, IO_SIZE);
    if (ingenic_is_xburst2(info->soc)) {
        /*
         * The T41 SPL (epc_debug.c) copies a stub here and starts core 1
         * on it through the CCU; without RAM core 1 runs zeros off the
         * end of the I/O space into the boot ROM's exception vector and
         * back into the SPL, which then hangs at random.
         */
        MemoryRegion *oram = g_new(MemoryRegion, 1);

        memory_region_init_ram(oram, NULL, "ingenic.oram", ORAM_SIZE,
                               &error_fatal);
        memory_region_add_subregion(sysmem, ORAM_BASE, oram);
    }
    if (info->soc == INGENIC_T41) {
        /*
         * IVDC (direct_mode): tx-isp queues channel 0 the pair 0x80000000 /
         * 0x81000000, which the scaler writes into the IVDC and the encoder
         * reads back; RAM there stands in for its store
         */
        MemoryRegion *ivdc = g_new(MemoryRegion, 1);

        memory_region_init_ram(ivdc, NULL, "ingenic.ivdc", IVDC_STORE_SIZE,
                               &error_fatal);
        memory_region_add_subregion(sysmem, IVDC_STORE_BASE, ivdc);
    }
    /* CPM */
    dev = cpm = qdev_new("ingenic-cpm");
    qdev_prop_set_uint32(dev, "soc", info->soc);
    qdev_prop_set_uint32(dev, "cppsr", var->cppsr);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, CPM_BASE);

    if (ingenic_is_xburst2(info->soc)) {
        /* CCU + core INTC -> IP2/IP3/IP4 of each core */
        ims->intc = qdev_new("ingenic-ccu");
        qdev_prop_set_uint32(ims->intc, "num-cores", ms->smp.cpus);
        sbd = SYS_BUS_DEVICE(ims->intc);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, CCU_BASE);
        sysbus_mmio_map(sbd, 1, CORE_INTC_BASE);
        for (i = 0; i < ms->smp.cpus; i++) {
            CPUMIPSState *e = &ims->cpu[i]->env;

            sysbus_connect_irq(sbd, 3 * i, e->irq[2]);
            sysbus_connect_irq(sbd, 3 * i + 1, e->irq[3]);
            sysbus_connect_irq(sbd, 3 * i + 2, e->irq[4]);
        }
        /* global OST: the clock source; core OSTs: the ticks */
        dev = qdev_new("ingenic-ost");
        qdev_prop_set_uint32(dev, "layout", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, OST_BASE);
        for (i = 0; i < ms->smp.cpus; i++) {
            dev = qdev_new("ingenic-ost");
            qdev_prop_set_uint32(dev, "layout", 2);
            sbd = SYS_BUS_DEVICE(dev);
            sysbus_realize_and_unref(sbd, &error_fatal);
            sysbus_mmio_map(sbd, 0, CORE_OST_BASE + i * 0x100);
            sysbus_connect_irq(sbd, 0,
                               qdev_get_gpio_in_named(ims->intc, "ost", i));
        }
    } else {
        /* INTC -> IP2 */
        ims->intc = qdev_new("ingenic-intc");
        sbd = SYS_BUS_DEVICE(ims->intc);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, INTC_BASE);
        sysbus_connect_irq(sbd, 0, env->irq[2]);

        /* system OST -> IP4 */
        dev = qdev_new("ingenic-ost");
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, OST_BASE);
        sysbus_connect_irq(sbd, 0, env->irq[4]);
    }

    /* TCU + watchdog */
    dev = qdev_new("ingenic-tcu");
    qdev_prop_set_bit(dev, "xburst2", ingenic_is_xburst2(info->soc));
    object_property_add_child(OBJECT(ims), "tcu", OBJECT(dev));
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, TCU_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_TCU0));
    sysbus_connect_irq(sbd, 1, ingenic_irq(ims, IRQ_TCU1));
    sysbus_connect_irq(sbd, 2, ingenic_irq(ims, IRQ_TCU2));

    /* RTC */
    sysbus_create_simple("ingenic-rtc", info->rtc_base, NULL);

    /* GPIO */
    dev = gpio = qdev_new("ingenic-gpio");
    qdev_prop_set_uint32(dev, "num-ports", info->gpio_ports);
    qdev_prop_set_uint32(dev, "stride", info->gpio_stride);
    qdev_prop_set_uint32(dev, "shadow", info->gpio_shadow);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, GPIO_BASE);
    for (i = 0; i < info->gpio_ports; i++) {
        sysbus_connect_irq(sbd, i, ingenic_irq(ims, IRQ_GPIO0 - i));
    }

    /*
     * UARTs.  The console is UART1 on every board, so it takes the first
     * -serial; UART0 the second, then UART2 and up.
     */
    for (i = 0; i < info->num_uarts; i++) {
        int hd = i == 1 ? 0 : (i == 0 ? 1 : i);

        serial_mm_init(sysmem, UART_BASE + i * 0x1000, 2,
                       ingenic_irq(ims, IRQ_UART0 - i), 115200,
                       serial_hd(hd), DEVICE_LITTLE_ENDIAN);
    }

    /* I2C, the sensor's bus on I2C0 */
    for (i = 0; i < info->num_i2c; i++) {
        dev = sysbus_create_simple("ingenic-i2c", I2C_BASE + i * 0x1000,
                                   ingenic_irq(ims, IRQ_I2C0 - i));
        if (i == info->sensor_bus && !info->sensor_gpio_sda && ims->sensor &&
            strcmp(ims->sensor, "none")) {
            ims->sensor_dev = ingenic_create_sensor(ims->sensor,
                I2C_BUS(qdev_get_child_bus(dev, "i2c")));
        }
        /* T40's second sensor, on I2C3 through CSI1 (the SDK's samples) */
        if (info->sensor1_bus && i == info->sensor1_bus && ims->sensor1 &&
            strcmp(ims->sensor1, "none")) {
            ims->sensor1_dev = ingenic_create_sensor(ims->sensor1,
                I2C_BUS(qdev_get_child_bus(dev, "i2c")));
        }
    }

    /*
     * T10's kernel bit-bangs the sensor bus (i2c-gpio, SDA PA12, SCL
     * PA13): QEMU's gpio_i2c on the pins, open-drain through the GPIO
     * model's "drive" outputs.
     */
    if (info->sensor_gpio_sda && ims->sensor && strcmp(ims->sensor, "none")) {
        int sda = info->sensor_gpio_sda;

        dev = sysbus_create_simple(TYPE_GPIO_I2C, -1, NULL);
        qdev_connect_gpio_out_named(gpio, "drive", sda,
                                    qdev_get_gpio_in(dev, 0));
        qdev_connect_gpio_out_named(gpio, "drive", sda + 1,
                                    qdev_get_gpio_in(dev, 1));
        qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(gpio, sda));
        ims->sensor_dev = ingenic_create_sensor(ims->sensor,
            I2C_BUS(qdev_get_child_bus(dev, "i2c")));
    }

    /* the image pipeline: ISP core, VIC, MIPI CSI host and D-PHY */
    if (info->isp && !strcmp(info->isp, "apical")) {
        /* T10, T20, T30: the Apical core, the VIC, their interrupt block */
        dev = qdev_new("ingenic-isp-apical");
        /* T10's VIC has its control at 0x20 */
        qdev_prop_set_uint32(dev, "vic-ctrl",
                             info->soc == INGENIC_T10 ? 0x20 : 0);
        object_property_add_child(OBJECT(ims), "isp", OBJECT(dev));
        if (ims->sensor_dev) {
            /* the exposure the AE programs lights the picture */
            object_property_set_link(OBJECT(dev), "sensor",
                                     OBJECT(ims->sensor_dev), &error_abort);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, info->isp_base);
        sysbus_mmio_map(sbd, 1, info->vic_base);
        sysbus_mmio_map(sbd, 2, info->vic_base + 0x20000);
        sysbus_mmio_map(sbd, 3, info->csi_base);
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_ISP));
        if (info->scaler_base) {
            /* T30: the frames come out of the scaler, on its own IRQ */
            dev = qdev_new("ingenic-isp");
            qdev_prop_set_string(dev, "soc", "t30");
            sbd = SYS_BUS_DEVICE(dev);
            sysbus_realize_and_unref(sbd, &error_fatal);
            sysbus_mmio_map(sbd, 0, info->scaler_base);
            sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_SCALER_T30));
            /* the NCU (noise) and LDC (lens) units: storage */
            for (i = 0; i < 2; i++) {
                MemoryRegion *mr = g_new(MemoryRegion, 1);

                memory_region_init_ram(mr, NULL, i ? "ingenic.isp-ldc" :
                                       "ingenic.isp-ncu", 0x10000,
                                       &error_fatal);
                memory_region_add_subregion(sysmem, i ? 0x130A0000 :
                                            0x13090000, mr);
            }
        }
    } else if (info->isp) {
        dev = qdev_new("ingenic-isp");
        qdev_prop_set_string(dev, "soc", info->isp);
        object_property_add_child(OBJECT(ims), "isp", OBJECT(dev));
        if (ims->sensor_dev) {
            /* the exposure the AE programs lights the picture */
            object_property_set_link(OBJECT(dev), "sensor",
                                     OBJECT(ims->sensor_dev), &error_abort);
        }
        if (ims->sensor1_dev) {
            /* T40's second pipe: its own sensor and exposure */
            object_property_set_link(OBJECT(dev), "sensor1",
                                     OBJECT(ims->sensor1_dev), &error_abort);
        }
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, info->isp_base);
        sysbus_mmio_map(sbd, 1, info->vic_base);
        if (info->csi_base) {
            sysbus_mmio_map(sbd, 2, info->csi_base);
            sysbus_mmio_map(sbd, 3, info->dphy_base);
        }
        if (info->soc == INGENIC_T41) {
            /* T41's scaler sits past the VIC, at core + 0xF0000 */
            sysbus_mmio_map(sbd, 4, info->isp_base + 0xF0000);
        }
        if (info->soc == INGENIC_T40) {
            /* VIC1 and VIC2, 64 KiB apart past VIC0, INTC 29 and 28 */
            sysbus_mmio_map(sbd, 5, info->vic_base + 0x10000);
            sysbus_mmio_map(sbd, 6, info->vic_base + 0x20000);
            sysbus_connect_irq(sbd, 2, ingenic_irq(ims, info->irq_vic - 1));
            sysbus_connect_irq(sbd, 3, ingenic_irq(ims, info->irq_vic - 2));
        }
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, info->irq_isp ?
                                               info->irq_isp : IRQ_ISP));
        sysbus_connect_irq(sbd, 1, ingenic_irq(ims, info->irq_vic ?
                                               info->irq_vic : IRQ_VIC));
    }

    /* the IPU: libimp's OSD blender, the same block on every part */
    sysbus_create_simple("ingenic-ipu", IPU_BASE, ingenic_irq(ims, IRQ_IPU));

    /* XBurst2's rotator (frame channel rotation) and OSD box drawer */
    if (ingenic_is_xburst2(info->soc)) {
        sysbus_create_simple("ingenic-i2d", I2D_BASE,
                             ingenic_irq(ims, IRQ_I2D));
        sysbus_create_simple("ingenic-drawbox", DRAWBOX_BASE,
                             ingenic_irq(ims, IRQ_DRAWBOX));
    }

    /*
     * Sound: the PDMA, and the AIC with the internal codec, which request
     * it on lines 6 (play), 7 (record) and 5 (loop record).
     */
    pdma = qdev_new("ingenic-pdma");
    sbd = SYS_BUS_DEVICE(pdma);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, PDMA_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_PDMA));
    dev = qdev_new("ingenic-aic");
    switch (info->soc) {
    case INGENIC_T10: case INGENIC_T20: case INGENIC_T21: case INGENIC_T30:
        qdev_prop_set_string(dev, "codec", "t10");
        break;
    case INGENIC_T23: case INGENIC_T31:
        qdev_prop_set_string(dev, "codec", "t31");
        break;
    default:
        break;
    }
    object_property_set_link(OBJECT(dev), "cpm", OBJECT(cpm), &error_fatal);
    if (ms->audiodev) {
        qdev_prop_set_string(dev, "audiodev", ms->audiodev);
    }
    object_property_add_child(OBJECT(ims), "aic", OBJECT(dev));
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, AIC_BASE);
    sysbus_mmio_map(sbd, 1, CODEC_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_AIC));
    qdev_connect_gpio_out_named(dev, "drq-tx", 0,
                                qdev_get_gpio_in_named(pdma, "drq", 6));
    qdev_connect_gpio_out_named(dev, "drq-rx", 0,
                                qdev_get_gpio_in_named(pdma, "drq", 7));
    qdev_connect_gpio_out_named(dev, "drq-loop", 0,
                                qdev_get_gpio_in_named(pdma, "drq", 5));

    /*
     * The SPI masters, on PDMA lines 0x16 / 0x17 (SSI0) and 0x18 / 0x19
     * (SSI1); their buses "ssi0" / "ssi1" take -device SPI peripherals.
     */
    for (i = 0; i < 2; i++) {
        if (i && (info->soc == INGENIC_T10 || info->soc == INGENIC_T20 ||
                  info->soc == INGENIC_T23)) {
            break;
        }
        dev = qdev_new("ingenic-ssi");
        qdev_prop_set_string(dev, "bus-name", i ? "ssi1" : "ssi0");
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, i ? SSI1_BASE : SSI0_BASE);
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, i ? IRQ_SSI1 : IRQ_SSI0));
        qdev_connect_gpio_out_named(dev, "drq-tx", 0,
                                    qdev_get_gpio_in_named(pdma, "drq",
                                                           0x16 + 2 * i));
        qdev_connect_gpio_out_named(dev, "drq-rx", 0,
                                    qdev_get_gpio_in_named(pdma, "drq",
                                                           0x17 + 2 * i));
    }

    /* the digital microphones of T31 and T40, on request line 13 */
    if (info->soc == INGENIC_T31 || info->soc == INGENIC_T40) {
        dev = qdev_new("ingenic-dmic");
        object_property_add_child(OBJECT(ims), "dmic", OBJECT(dev));
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, DMIC_BASE);
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_DMIC));
        qdev_connect_gpio_out_named(dev, "drq", 0,
                                    qdev_get_gpio_in_named(pdma, "drq", 13));
    }

    /* the auxiliary ADC (jz_adc_v13 / ingenic_adc_v13): /machine/sadc */
    dev = qdev_new("ingenic-sadc");
    object_property_add_child(OBJECT(ims), "sadc", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SADC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ingenic_irq(ims, IRQ_SADC));

    /* T41's PWM controller: /machine/pwm */
    if (info->soc == INGENIC_T41) {
        dev = qdev_new("ingenic-pwm");
        object_property_add_child(OBJECT(ims), "pwm", OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, PWM_BASE);
    }

    /*
     * The security blocks: AES on every part; HASH and the DTRNG from
     * T21 on; RSA (polled) on T23, T31, T40 and T41.
     */
    sysbus_create_simple("ingenic-aes", AES_BASE, ingenic_irq(ims, IRQ_AES));
    if (info->soc != INGENIC_T10 && info->soc != INGENIC_T20 &&
        info->soc != INGENIC_T30) {
        sysbus_create_simple("ingenic-hash", HASH_BASE,
                             ingenic_irq(ims, IRQ_HASH));
        sysbus_create_simple("ingenic-dtrng", DTRNG_BASE,
                             ingenic_irq(ims, IRQ_DTRNG));
        if (info->soc != INGENIC_T21) {
            sysbus_create_simple("ingenic-rsa", RSA_BASE, NULL);
        }
    }

    /*
     * The OTG PHY's registers (byte-wide tuning bits the kernels set:
     * cpm_usb.c, phy-ingenic-inno.c); nothing is polled, so storage.
     */
    usbphy = g_new(MemoryRegion, 1);
    memory_region_init_ram(usbphy, NULL, "ingenic.usbphy", 0x1000,
                           &error_fatal);
    memory_region_add_subregion(sysmem, USBPHY_BASE, usbphy);

    dev = qdev_new("ingenic-efuse");
    qdev_prop_set_uint32(dev, ingenic_is_xburst2(info->soc) ?
                         "subsoctype2" : "subsoctype1",
                         (uint32_t)var->code << 16);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, EFUSE_BASE);

    /* the AHB0 arbiter, with the SoC ID */
    dev = qdev_new("ingenic-harb");
    qdev_prop_set_uint32(dev, "soc-id", info->soc_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, HARB0_BASE);

    /*
     * The AVPU (Allegro) encoder of T31, T40 and T41: avpu.ko's own
     * platform device, registers from 0x8000 in the block (from 0 on T41)
     */
    if (info->soc == INGENIC_T31 || info->soc == INGENIC_T40 ||
        info->soc == INGENIC_T41) {
        const bool t41 = info->soc == INGENIC_T41;

        dev = qdev_new("ingenic-avpu");
        qdev_prop_set_uint32(dev, "reg-base", t41 ? 0 : 0x8000);
        /* T41's core is the next generation: core pages, 4 KiB entries */
        qdev_prop_set_uint8(dev, "gen", t41 ? 2 : 1);
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, t41 ? AVPU_BASE_T41 : AVPU_BASE);
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_AVPU));
    }

    /* USB: the DWC2 OTG core, host mode (QEMU's Raspberry Pi model) */
    dev = qdev_new("dwc2-usb");
    object_property_add_const_link(OBJECT(dev), "dma-mr", OBJECT(sysmem));
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, OTG_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_OTG));

    /* Ethernet: Synopsys GMAC, an IP101G PHY */
    dev = qdev_new("ingenic-gmac");
    qemu_configure_nic_device(dev, true, NULL);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, GMAC_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_GMAC));

    ingenic_vpu_create(ims, cpm, info->soc);

    /* DDR controller and PHY */
    dev = qdev_new("ingenic-ddr");
    qdev_prop_set_uint32(dev, "soc", info->soc);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, DDR_PHY_BASE);
    sysbus_mmio_map(sbd, 1, DDRC_BASE);

    /* SFC + NOR */
    ims->sfc = qdev_new("ingenic-sfc");
    qdev_prop_set_uint32(ims->sfc, "version", info->sfc_version);
    sbd = SYS_BUS_DEVICE(ims->sfc);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, SFC_BASE);
    sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_SFC));
    if (info->has_sfc1) {
        ims->sfc1 = qdev_new("ingenic-sfc");
        qdev_prop_set_uint32(ims->sfc1, "version", info->sfc_version);
        sbd = SYS_BUS_DEVICE(ims->sfc1);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, SFC1_BASE);
        sysbus_connect_irq(sbd, 0, ingenic_irq(ims, IRQ_SFC1));
    }
    ingenic_create_flash(ims);

    if (ingenic_is_xburst2(info->soc)) {
        /* MSC0/1 are SDHCI 3.0 hosts on XBurst2 (sdhci-ingenic) */
        for (i = 0; i < 2; i++) {
            DriveInfo *di = drive_get(IF_SD, 0, i);

            dev = qdev_new(TYPE_SYSBUS_SDHCI);
            qdev_prop_set_uint8(dev, "sd-spec-version", 3);
            qdev_prop_set_uint64(dev, "capareg", 0x057834b4);
            sbd = SYS_BUS_DEVICE(dev);
            sysbus_realize_and_unref(sbd, &error_fatal);
            sysbus_mmio_map(sbd, 0, i ? SDHCI1_BASE : SDHCI0_BASE);
            sysbus_connect_irq(sbd, 0, ingenic_irq(ims, i ? IRQ_MSC1 :
                                                   IRQ_MSC0));
            if (di) {
                DeviceState *card = qdev_new(TYPE_SD_CARD);

                qdev_prop_set_drive_err(card, "drive",
                                        blk_by_legacy_dinfo(di),
                                        &error_fatal);
                qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                       &error_fatal);
            }
        }
    } else {
        /*
         * MSC0/1, the jzmmc hosts; MSC0's card detect is a GPIO the host
         * drives as QEMU inserts or ejects the card (-drive if=sd,index=0)
         */
        for (i = 0; i < 2; i++) {
            DriveInfo *di = drive_get(IF_SD, 0, i);

            dev = qdev_new("ingenic-msc");
            sbd = SYS_BUS_DEVICE(dev);
            sysbus_realize_and_unref(sbd, &error_fatal);
            sysbus_mmio_map(sbd, 0, i ? MSC1_BASE : MSC0_BASE);
            sysbus_connect_irq(sbd, 0, ingenic_irq(ims, i ? IRQ_MSC1 :
                                                   IRQ_MSC0));
            if (i == 0) {
                qdev_connect_gpio_out_named(dev, "cd", 0,
                        qdev_get_gpio_in(gpio, info->msc0_cd));
            }
            if (di) {
                DeviceState *card = qdev_new(TYPE_SD_CARD);

                qdev_prop_set_drive_err(card, "drive",
                                        blk_by_legacy_dinfo(di), &error_fatal);
                qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                       &error_fatal);
            }
        }
    }

    if (ms->kernel_filename) {
        ingenic_kernel_boot(ims);
    } else if (ims->flash_file && ims->nand) {
        ingenic_nand_boot(ims);
    } else if (ims->flash_file) {
        ingenic_flash_boot(ims);
    } else if (!qtest_enabled()) {
        error_report("ingenic: nothing to boot: give -kernel or "
                     "-machine %s,flash-file=<image>", info->name);
        exit(1);
    }
}

static char *ingenic_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_MACHINE(obj)->flash_file);
}

static void ingenic_set_flash_file(Object *obj, const char *value,
                                   Error **errp)
{
    IngenicMachineState *ims = INGENIC_MACHINE(obj);

    g_free(ims->flash_file);
    ims->flash_file = g_strdup(value);
}

static char *ingenic_get_flash_type(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_MACHINE(obj)->flash_type);
}

static void ingenic_set_flash_type(Object *obj, const char *value,
                                   Error **errp)
{
    IngenicMachineState *ims = INGENIC_MACHINE(obj);

    g_free(ims->flash_type);
    ims->flash_type = g_strdup(value);
}

static char *ingenic_get_variant(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_MACHINE(obj)->variant);
}

static void ingenic_set_variant(Object *obj, const char *value, Error **errp)
{
    IngenicMachineState *ims = INGENIC_MACHINE(obj);

    g_free(ims->variant);
    ims->variant = g_strdup(value);
}

static char *ingenic_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_MACHINE(obj)->sensor);
}

static void ingenic_set_sensor(Object *obj, const char *value, Error **errp)
{
    IngenicMachineState *ims = INGENIC_MACHINE(obj);

    g_free(ims->sensor);
    ims->sensor = g_strdup(value);
}

static char *ingenic_get_sensor1(Object *obj, Error **errp)
{
    return g_strdup(INGENIC_MACHINE(obj)->sensor1);
}

static void ingenic_set_sensor1(Object *obj, const char *value, Error **errp)
{
    IngenicMachineState *ims = INGENIC_MACHINE(obj);

    g_free(ims->sensor1);
    ims->sensor1 = g_strdup(value);
}

static void ingenic_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "ingenic.ram";
    mc->ignore_memory_transaction_failures = true;
    machine_add_audiodev_property(mc);
    object_class_property_add_str(oc, "flash-file",
        ingenic_get_flash_file, ingenic_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image (SPL header at 0, U-Boot at 0x6800) or SPI NAND "
        "image (128/256 MiB of page data, or pages with their OOB); booted "
        "without -kernel, and erases and programs are written back");
    object_class_property_add_str(oc, "flash-type",
        ingenic_get_flash_type, ingenic_set_flash_type);
    object_class_property_set_description(oc, "flash-type",
        "flash part: an m25p80 NOR, default by size w25q64 (8 MiB), "
        "mx25l12805d (16 MiB), w25q256 (32 MiB); or a SPI NAND, w25n01gv "
        "(128/132 MiB), gd5f1gq4ub (136 MiB), gd5f2gq4ub (256/272 MiB), "
        "mx35lf2ge4ab (264 MiB)");
    object_class_property_add_str(oc, "variant",
        ingenic_get_variant, ingenic_set_variant);
    object_class_property_set_description(oc, "variant",
        "the grade within the family the SoC reports in CPPSR and the "
        "EFUSE (e.g. t31x, t40xp, t41lq); DRAM is still -m");
    object_class_property_add_str(oc, "sensor",
        ingenic_get_sensor, ingenic_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "image sensor on the sensor I2C bus: none (default), gc2053, "
        "gc2083, gc4653, gc4663, sc2336, sc2335, jxf37, jxf22, ov2735b, ov2735");
    object_class_property_add_str(oc, "sensor1",
        ingenic_get_sensor1, ingenic_set_sensor1);
    object_class_property_set_description(oc, "sensor1",
        "T40: a second image sensor, on I2C3 (CSI1, the second pipe); the "
        "same names as sensor=");
}

static void ingenic_soc_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    const IngenicSocInfo *info = data;

    INGENIC_MACHINE_CLASS(oc)->info = info;
    mc->desc = info->desc;
    mc->init = ingenic_init;
    mc->default_cpu_type = info->cpu;
    mc->default_ram_size = info->ram_default;
    mc->max_cpus = info->max_cpus;
    mc->default_cpus = info->max_cpus;
}

static const IngenicSocInfo ingenic_socs[] = {
    {
        .name = "t10", .desc = "Ingenic T10 (XBurst1)",
        .soc = INGENIC_T10, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 6, .gpio_stride = 0x100, .gpio_shadow = 0xF00,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 2,
        .soc_id = 0x10005003, .prid = 0, .l1_kib = 0,
        .sensor_gpio_sda = 12,
        .msc0_cd = 32 + 27, .isp = "apical", .isp_base = 0x13380000,
        .vic_base = 0x13300000, .csi_base = 0x10040000,
    }, {
        .name = "t20", .desc = "Ingenic T20 (XBurst1)",
        .soc = INGENIC_T20, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 6, .gpio_stride = 0x100, .gpio_shadow = 0xF00,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 2,
        .soc_id = 0x12000002, .prid = 0x00d00101, .l1_kib = 0,
        .msc0_cd = 32 + 27, .isp = "apical", .isp_base = 0x13380000,
        .vic_base = 0x13300000, .csi_base = 0x10040000,
    }, {
        .name = "t21", .desc = "Ingenic T21 (XBurst1)",
        .soc = INGENIC_T21, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 2,
        .soc_id = 0x10021003, .prid = 0, .l1_kib = 16,
        .msc0_cd = 32 + 20, .isp = "t21", .isp_base = ISP_BASE,
        .vic_base = 0x13310000,
    }, {
        .name = "t23", .desc = "Ingenic T23 (XBurst1)",
        .soc = INGENIC_T23, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 3,
        .soc_id = 0x10023003, .prid = 0, .l1_kib = 16,
        .msc0_cd = 32 + 27, .isp = "t23", .isp_base = ISP_BASE,
        .vic_base = VIC_BASE, .csi_base = CSI_BASE, .dphy_base = DPHY_BASE,
    }, {
        .name = "t30", .desc = "Ingenic T30 (XBurst1)",
        .soc = INGENIC_T30, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 3,
        .soc_id = 0x10030005, .prid = 0, .l1_kib = 0,
        .msc0_cd = 32 + 27, .isp = "apical", .isp_base = 0x13380000,
        .vic_base = 0x13300000, .csi_base = 0x10040000,
        .scaler_base = 0x130B0000,
    }, {
        .name = "t31", .desc = "Ingenic T31 (XBurst1)",
        .soc = INGENIC_T31, .cpu = MIPS_CPU_TYPE_NAME("XBurst1"),
        .ram_default = 64 * MiB, .num_uarts = 3,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 1, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = RTC_BASE, .max_cpus = 1, .num_i2c = 2,
        .soc_id = 0x10031003, .prid = 0, .l1_kib = 0,
        .msc0_cd = 32 + 27, .isp = "t31", .isp_base = ISP_BASE,
        .vic_base = VIC_BASE, .csi_base = CSI_BASE, .dphy_base = DPHY_BASE,
    }, {
        .name = "t40", .desc = "Ingenic T40 (XBurst2, 2 cores)",
        .soc = INGENIC_T40, .cpu = MIPS_CPU_TYPE_NAME("XBurst2"),
        .ram_default = 128 * MiB, .num_uarts = 4,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 2, .mtd_id = "jz_sfc", .nand_mtd_id = "sfc_nand",
        .rtc_base = 0x132A0000, .max_cpus = 2, .num_i2c = 4,
        .soc_id = 0x10040003, .prid = 0, .l1_kib = 0,
        .sensor_bus = 1, .sensor1_bus = 3,
        /* VIC0 of three; CSI0 (the one a single sensor uses) */
        .isp = "t40", .isp_base = ISP_BASE, .vic_base = 0x13380000,
        .csi_base = 0x10054000, .dphy_base = DPHY_BASE,
        .irq_isp = 31, .irq_vic = 30,
    }, {
        .name = "t41", .desc = "Ingenic T41 (XBurst2, 2 cores)",
        .soc = INGENIC_T41, .cpu = MIPS_CPU_TYPE_NAME("XBurst2"),
        .ram_default = 128 * MiB, .num_uarts = 4,
        .gpio_ports = 4, .gpio_stride = 0x1000, .gpio_shadow = 0x7000,
        .sfc_version = 2, .mtd_id = "sfc0_nor",
        .nand_mtd_id = "sfc0_nand", .has_sfc1 = true,
        .rtc_base = RTC_BASE, .max_cpus = 2, .num_i2c = 3,
        .soc_id = 0x10041003, .prid = 0, .l1_kib = 0,
        /* one VIC, one CSI host; the scaler at 0x133F0000 */
        .isp = "t41", .isp_base = ISP_BASE, .vic_base = 0x13380000,
        .csi_base = 0x10023000, .dphy_base = DPHY_BASE,
        .irq_isp = 31, .irq_vic = 30,
    },
};

static void ingenic_machine_register_types(void)
{
    static const TypeInfo base = {
        .name = TYPE_INGENIC_MACHINE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(IngenicMachineState),
        .class_size = sizeof(IngenicMachineClass),
        .class_init = ingenic_machine_class_init,
    };
    int i;

    type_register_static(&base);
    for (i = 0; i < ARRAY_SIZE(ingenic_socs); i++) {
        TypeInfo ti = {
            .name = g_strdup_printf(MACHINE_TYPE_NAME("%s"),
                                    ingenic_socs[i].name),
            .parent = TYPE_INGENIC_MACHINE,
            .class_init = ingenic_soc_class_init,
            .class_data = &ingenic_socs[i],
        };

        type_register_static(&ti);
    }
}

type_init(ingenic_machine_register_types)
