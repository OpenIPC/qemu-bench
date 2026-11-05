/*
 * Novatek NT9856x (NA51089) and NT9852x (NA51055) IP-camera SoC machines,
 * the NT98533 (NS02501) and the NT98332 (NA51103) NVR die at the end of
 * the file.
 *
 * Table-driven like hisilicon.c and the other families; include/hw/arm/
 * novatek.h has the SoC.  Boots a kernel with -kernel / -initrd, with a
 * device tree built here from the SDK's board files: the OpenIPC
 * nt98562/nt98566 uImage carries none, and on silicon the loader hands
 * U-Boot one from flash.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "hw/usb/hcd-xhci-sysbus.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/novatek.h"
#include "hw/arm/nvt-bootrom.h"
#include "hw/boards.h"
#include "hw/cpu/cluster.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/ide/ahci.h"
#include "hw/ide/ahci-sysbus.h"
#include "hw/intc/arm_gic.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/irq.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/nvt-flash.h"
#include "hw/arm/nvt-lzma.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/device_tree.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/gtimer.h"
#include <libfdt.h>

#define NVT_PERIPH_BASE     0xF0000000
#define NVT_PERIPH_SIZE     0x02000000
#define NVT_DDRC_BASE       0xF0000000
#define NVT_TOP_BASE        0xF0010000
#define NVT_CG_BASE         0xF0020000
#define NVT_PAD_BASE        0xF0030000
#define NVT_TIMER_BASE      0xF0040000
#define NVT_WDT_BASE        0xF0050000
#define NVT_RTC_BASE        0xF0060000
#define NVT_GPIO_BASE       0xF0070000
#define NVT_FLASH_BASE      0xF0400000
#define NVT_ETH_BASE        0xF02B0000
#define NVT_USB_BASE        0xF0600000
#define NVT_OTP_BASE        0xF0660000
#define NVT_PWM_BASE        0xF0210000
#define NVT_DAI_BASE        0xF0630000
#define NVT_IDE_BASE        0xF0800000
/*
 * -bios: ms a full UART 0 goes without an RBR read before its input is
 * lost.  Silicon loses it after one character time, 87 us at 115200; a
 * starved vCPU on a busy host can go far longer than that mid-transfer
 * (250 ms dropped bytes of the U-Boot pack), and the ROM leaves its UART
 * alone for a minute, so seconds separate the two.
 */
#define NVT_ROM_UART_IDLE_MS 5000
#define NVT_DSI_BASE        0xF0840000
#define NVT_EAC_BASE        0xF0640000
#define NVT_USB_ALIAS       0xFF600000  /* where NA51089's driver looks */
#define NVT_MPCORE_BASE     0xFFD00000
#define NVT_L2_BASE         0xFFE00000

/* GIC SPI numbers (nvt-peri.dtsi) */
#define NVT_IRQ_GPIO        24
#define NVT_IRQ_USB         27
#define NVT_IRQ_PWM         26
#define NVT_IRQ_TIMER       0
#define NVT_IRQ_DAI         15
#define NVT_IRQ_IDE         48
#define NVT_IRQ_DSI         50
#define NVT_D_GPIO(n)       (0x80 + (n))    /* nvt-gpio.h D_GPIO() */
#define NVT_IRQ_ARB         33
#define NVT_ARB2_BASE       0xF0FE0000
#define NVT_IRQ_FLASH       29
#define NVT_IRQ_ETH         34
#define NVT_IRQ_RTC         56
#define NVT_IRQ_WDT         57
#define NVT_IRQ_L2          96

#define NVT_NUM_IRQ         256     /* irqs.h: SPIs 32..255 */

/* The security blocks: cipher, hash, RSA (nvt-peri.dtsi). */
static const struct {
    const char *type;
    const char *node;
    hwaddr base;
    int irq;
} nvt_crypto_blocks[] = {
    { "nvt-crypto", "crypto", 0xF0620000, 38 },
    { "nvt-hash",   "hash",   0xF0670000, 28 },
    { "nvt-rsa",    "rsa",    0xF06A0000, 20 },
};

/* NA51055 has the hash engine at 0xF0700000 (nvt-na51055-peri.dtsi). */
#define NA51055_HASH_BASE   0xF0700000

static const struct {
    hwaddr base;
    int irq;
    uint32_t clk;
} nvt_uarts[] = {
    { 0xF0290000, 43, 24000000 },
    { 0xF0300000, 44, 48000000 },
    { 0xF0310000, 45, 48000000 },
};

/* NA51055's UART4..6 (nvt-na51055-peri.dtsi), after NA51089's three. */
static const struct {
    hwaddr base;
    int irq;
} na51055_uarts[] = {
    { 0xF0380000, 63 },
    { 0xF03E0000, 64 },
    { 0xF03F0000, 65 },
};

/*
 * The I2C controllers: i2c-nvt.c names its clock after the device
 * ("f0350000.i2c2"), so the nodes keep the vendor's names; id n makes the
 * adapter /dev/i2c-n.
 */
static const struct {
    const char *node;
    hwaddr base;
    int irq;
} nvt_i2c[] = {
    { "i2c",  0xF0220000, 41 },
    { "i2c2", 0xF0350000, 42 },
    { "i2c3", 0xF03A0000, 60 },
};

/* NA51055's I2C 4 / 5 (cfg_528_IPCAM1_NOR_EVB nvt-na51055-i2c.dtsi). */
static const struct {
    const char *node;
    hwaddr base;
    int irq;
} na51055_i2c[] = {
    { "i2c4", 0xF03B0000, 68 },
    { "i2c5", 0xF03C0000, 69 },
};

/*
 * sensor= on I2C 0, where ipcinfo's Novatek HAL probes: the catalogue's
 * parts, of which OpenIPC's firmware has a driver (nvt_sen_<name>.ko) for
 * sc401ai and sc501ai.
 */
/*
 * The SD hosts: card n from -drive if=sd,index=n.  reset is the host's
 * nvt-cg "sys-reset" line (CG_SYS_RESET_REG1 bit 2 / 3 / 14).  The EVB
 * device trees take SDIO1's card detect from C_GPIO 9, low with a card
 * in.  SDIO3 is the eMMC host (cfg_562_EMMC_EVB: mmc@f0510000,
 * "nvt,nvt_mmc3", SPI 32): its "card" is an eMMC, 8 bits wide here.
 */
#define NVT_SD_CD_GPIO      9

static const struct {
    hwaddr base;
    int irq;
    int reset;
    const char *compat;
    bool emmc;
} nvt_sdio[] = {
    { 0xF0420000, 30, 32 + 2,  "nvt,nvt_mmc",  false },
    { 0xF0500000, 31, 32 + 3,  "nvt,nvt_mmc2", false },
    { 0xF0510000, 32, 32 + 14, "nvt,nvt_mmc3", true },
};

/*
 * The HDAL's media engines (cfg_566_IPCAM1_EVB nvt-peri.dtsi), those the
 * OpenIPC release's kdrv_* / kflow_* modules bind by compatible: each
 * register window becomes an nvt-media block named for the engine
 * ("sie", "sie2", ...), its interrupt the node's interrupt of the same
 * index.  An interrupt two nodes share (CSI and LVDS, RHE and AI) is
 * driven by the first only.  irq -1: none.
 */
#define NVT_MEDIA_REGS  5

typedef struct {
    hwaddr base;
    uint32_t size;
    int irq;
} NvtMediaWin;

static const struct {
    const char *node;
    const char *compat;
    NvtMediaWin win[NVT_MEDIA_REGS];
    uint32_t clock;             /* "clock-frequency", 0: none */
} nvt_media[] = {
    { "sie@f0c00000",     "nvt,drv_sie",
      { { 0xF0C00000, 0x900, 1 }, { 0xF0D20000, 0x900, 2 },
        { 0xF0D30000, 0x900, 3 } } },
    { "tge@f0cc0000",     "nvt,kdrv_tge",  { { 0xF0CC0000, 0x150, 22 } } },
    { "ise@f0c90000",     "nvt,kdrv_ise",  { { 0xF0C90000, 0x100, 21 } } },
    { "ipe@f0c30000",     "nvt,kdrv_ipe",  { { 0xF0C30000, 0x900, 5 } } },
    /* IFE before IME: IME finishes the frames IFE is pointed at */
    { "ife@f0c70000",     "nvt,kdrv_ife",  { { 0xF0C70000, 0x800, 8 } } },
    { "ime@f0c40000",     "nvt,kdrv_ime",  { { 0xF0C40000, 0x1000, 6 } } },
    { "dce@f0c20000",     "nvt,kdrv_dce",  { { 0xF0C20000, 0x650, 7 } } },
    /*
     * NUE, NUE2, CNN, in kdrv_ai's order.  The tree's CNN size, 0x22C,
     * stops one register short of the checksum counter the driver reads.
     */
    /*
     * IVE; probe takes the clock by the device name.  The tree says 0x6C,
     * but the driver writes 0x6C on every job.
     */
    { "ive@f0d70000",     "nvt,kdrv_ive",  { { 0xF0D70000, 0x100, 53 } } },
    /* MDBC; probe takes the clock by the device name, so the node name */
    { "md@f0c10000",      "nvt,kdrv_md",   { { 0xF0C10000, 0x150, 46 } },
      240000000 },
    { "ai@f0c60000",      "nvt,kdrv_ai",
      { { 0xF0C60000, 0x23C, 14 }, { 0xF0D50000, 0x114, 13 },
        { 0xF0CB0000, 0x230, 11 } }, 600000000 },
    { "csi@f0280000",     "nvt,nvt_csi",
      { { 0xF0280000, 0x100, 54 }, { 0xF0330000, 0x100, 55 } } },
    { "lvds@f0270000",    "nvt,nvt_lvds",
      { { 0xF0270000, 0x200, -1 }, { 0xF0370000, 0x200, -1 } } },
    { "senphy@f06b0000",  "nvt,nvt_senphy", { { 0xF06B0000, 0x100, -1 } } },
    { "sif@f0240000",     "nvt,nvt_sif",   { { 0xF0240000, 0x200, 40 } },
      1000000 },
    { "graphic@f0c80000", "nvt,nvt_graphic",
      { { 0xF0C80000, 0x300, 18 }, { 0xF0D10000, 0x100, 19 } } },
    { "h26x@f0a10000",    "nvt,nvt_h26x",  { { 0xF0A10000, 0xA00, 16 } } },
    { "jpg@f0a00000",     "nvt,nvt_jpg",   { { 0xF0A00000, 0x100, 17 } } },
    /* the drivers reach past the tree's reg sizes: EAC 0x200, DAI 0xD0 */
    { "eac@f0640000",     "nvt,nvt_eac",   { { 0xF0640000, 0x300, -1 } } },
    { "dai@f0630000",     "nvt,nvt_dai",   { { 0xF0630000, 0x100, 15 } } },
};

/*
 * NA51055's "ai" node (nvt-na51055-peri.dtsi): NUE, NUE2, then two CNNs --
 * kdrv_ai's ai_drv.c sets CNN 1 from the third window and CNN 0 from the
 * fourth, which is NA51089's one CNN.  The tree's sizes (NUE 0x1CC, the
 * CNNs 0x200) stop short of registers the drivers read, as on NA51089, so
 * the windows keep NA51089's.
 */
static const NvtMediaWin na51055_ai_win[NVT_MEDIA_REGS] = {
    { 0xF0C60000, 0x23C, 14 }, { 0xF0D50000, 0x114, 13 },
    { 0xF0D60000, 0x230, 49 }, { 0xF0CB0000, 0x230, 11 },
};

/* NA51055's five SIEs: kdrv_builtin's drv_sie wants every window. */
static const NvtMediaWin na51055_sie_win[NVT_MEDIA_REGS] = {
    { 0xF0C00000, 0x900, 1 }, { 0xF0D20000, 0x900, 2 },
    { 0xF0D30000, 0x900, 3 }, { 0xF0D40000, 0x900, 59 },
    { 0xF0D80000, 0x900, 61 },
};

static const struct {
    const char *compat;
    const NvtMediaWin *win;
} na51055_media_override[] = {
    { "nvt,kdrv_ai", na51055_ai_win },
    { "nvt,drv_sie", na51055_sie_win },
};

/*
 * NA51055's DDR arbiter node covers both DDR controllers (the EVBs' second
 * DDR at 0x40000000): DDR1 and DDR2 protection, then their ARB2 windows,
 * and interrupts 33 and 51.  DDR2's registers are plain storage here.
 */
#define NA51055_DDRC2_BASE  0xF0100000
#define NA51055_ARB2B_BASE  0xF0FD0000
#define NA51055_IRQ_ARB2    51
/* ISE has a second interrupt on NA51055 (nvt-na51055-peri.dtsi) */
#define NA51055_IRQ_ISE2    85

/* The HDAL's software units (nvt-media.dtsi): nodes with only a name. */
static const char *const nvt_hdal_units[] = {
    "nvtmpp", "isf_stream", "isf_flow", "isf_vdocap", "isf_vdoprc",
    "isf_dummy", "isf_vdoenc", "isf_vdodec", "isf_vdoout", "isf_audenc",
    "isf_auddec", "isf_audcap", "isf_audout",
};

/* cfg_566_IPCAM1_EVB nvt-peri.dtsi pll_preset@0: <pll rate enable> */
static const uint32_t nvt_pll_preset[][3] = {
    { 3, 0, 0 }, { 4, 0, 0 }, { 5, 297000000, 1 }, { 6, 0, 0 },
    { 7, 0, 0 }, { 8, 0, 0 }, { 9, 0, 0 }, { 11, 0, 0 }, { 12, 0, 1 },
};

static const struct {
    const char *prop;
    int n;
    uint32_t v[8];
} nvt_hdal_limits[] = {
    { "vdocap_active_list", 8, { 1, 1, 1, 0, 0, 0, 0, 0 } },
    { "vdoprc_maxdevice", 1, { 2 } },
    { "vdoenc_maxpath", 1, { 8 } },
    { "vdodec_maxpath", 1, { 2 } },
    { "vdoout_maxdevice", 1, { 1 } },
    { "adocap_maxdevice", 1, { 1 } },
    { "adoout_maxdevice", 1, { 1 } },
    { "adoenc_maxpath", 1, { 1 } },
    { "adodec_maxpath", 1, { 1 } },
    { "gfx_maxjob", 1, { 2 } },
    { "stamp_maximg", 1, { 16 } },
    { "vdoprc_maxstamp", 2, { 4, 16 } },
    { "vdoprc_maxmask", 2, { 8, 64 } },
    { "vdoenc_maxstamp", 2, { 32, 16 } },
    { "vdoenc_maxmask", 2, { 0, 64 } },
    { "vdoout_maxstamp", 2, { 0, 16 } },
    { "vdoout_maxmask", 2, { 0, 64 } },
};

/*
 * Blocks with no model yet: plain storage, so drivers' read-modify-writes
 * keep what they wrote.  Everything else in the peripheral window reads 0
 * and is logged under -d unimp.
 */
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} nvt_regbanks[] = {
    { "nvt-ddrc",   NVT_DDRC_BASE,  0x10000 },
    { "nvt-pad",    NVT_PAD_BASE,   0x10000 },
};

/*
 * NT98562 has 64 MB of DDR2 in the package, NT98566 128 MB of DDR3; the
 * SDK's memory tables (nvt-mem-tbl.dtsi) give Linux the first 30 / 32 MB
 * and the HDAL media pool the rest.  That leaves Linux too little to run
 * the HDAL samples and their files beside the media modules, so the
 * machines are given more than the package holds: Linux the first
 * 128 MB, the media pool (/hdal-memory) the 96 MB after it -- the
 * NT98566 board's pool.  -m sizes the pool.  With less RAM than that
 * (-m 64 / 128, the packages' sizes) the machine falls back to the SDK's
 * split.  The vendor firmware's own device tree keeps its own.
 */
#define NVT_LINUX_MEM       (128 * MiB)
#define NVT_RAM_DEFAULT     (NVT_LINUX_MEM + 96 * MiB)

static const NvtSoCConfig nt98562_soc = {
    .desc               = "Novatek NT98562 (Cortex-A9, NA51089, 64 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = NVT_RAM_DEFAULT,
    .linux_mem          = NVT_LINUX_MEM,
    .sdk_linux_mem      = 0x01E00000,       /* cfg_562_IPCAM1_NOR_EVB */
};

static const NvtSoCConfig nt98566_soc = {
    .desc               = "Novatek NT98566 (Cortex-A9, NA51089, 128 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = NVT_RAM_DEFAULT,
    .linux_mem          = NVT_LINUX_MEM,
    .sdk_linux_mem      = 0x02000000,       /* FW98566A.bin, IPCAM1_EVB */
};

/*
 * The other NA51089 parts the SDK has boards for: the die and chip ID are
 * the same (the part name is only U-Boot's build-time _CHIP_NAME_), the
 * package's DDR differs.  NT98560: 512 MB, Linux the first 256
 * (cfg_560_IPCAM1_EVB); NT98561: 64 MB like NT98562, Linux 30
 * (cfg_561_IPCAM1_EVB), given the same emulated room as NT98562.
 */
static const NvtSoCConfig nt98560_soc = {
    .desc               = "Novatek NT98560 (Cortex-A9, NA51089, 512 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = 512 * MiB,
    .linux_mem          = 0x10000000,       /* cfg_560_IPCAM1_EVB */
    .sdk_linux_mem      = 0x10000000,
};

static const NvtSoCConfig nt98561_soc = {
    .desc               = "Novatek NT98561 (Cortex-A9, NA51089, 64 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = NVT_RAM_DEFAULT,
    .linux_mem          = NVT_LINUX_MEM,
    .sdk_linux_mem      = 0x01E00000,       /* cfg_561_IPCAM1_EVB */
};

/*
 * NT98520 / NT98523 / NT98525 (CHIP_NA51055, 0x4821) and NT98528 / NT98529
 * (CHIP_NA51084, 0x5021): the NT9852x SDK's one die and kernel (4.19.91,
 * CONFIG_NVT_IVOT_PLAT_NA51055), two Cortex-A9s at 1.2 GHz.  Its EVBs
 * carry 512 MB at 0 (cfg_IPCAM1_NOR_EVB nvt-na51055-mem-tbl.dtsi: Linux
 * the first 0x12800000, the HDAL the rest) and a second 512 MB at
 * 0x40000000 that only the HDAL's pool uses, which is not modelled; the
 * NT98523 board has 128 MB, Linux the first 64.
 */
#define NA51055_RAM_DEFAULT     (512 * MiB)

static const NvtSoCConfig nt98525_soc = {
    .desc               = "Novatek NT98525 (2x Cortex-A9, NA51055, 512 MB)",
    .model              = "Novatek NA51055",
    .chip_id            = 0x4821,
    .ram_size_default   = NA51055_RAM_DEFAULT,
    .linux_mem          = 0x12800000,   /* cfg_IPCAM1_NOR_EVB */
    .sdk_linux_mem      = 0x04000000,   /* cfg_IPCAM1_523_NOR_EVB */
    .na51055            = true,
    .num_cpus           = 2,
};

static const NvtSoCConfig nt98528_soc = {
    .desc               = "Novatek NT98528 (2x Cortex-A9, NA51084, 512 MB)",
    .model              = "Novatek NA51055",    /* the SDK's, for both */
    .chip_id            = 0x5021,
    .ram_size_default   = NA51055_RAM_DEFAULT,
    .linux_mem          = 0x12800000,   /* cfg_528_IPCAM1_NOR_EVB */
    .sdk_linux_mem      = 0x04000000,
    .na51055            = true,
    .num_cpus           = 2,
};

/* NT98523: 128 MB, Linux the first 64 (cfg_IPCAM1_523_NOR_EVB) */
static const NvtSoCConfig nt98523_soc = {
    .desc               = "Novatek NT98523 (2x Cortex-A9, NA51055, 128 MB)",
    .model              = "Novatek NA51055",
    .chip_id            = 0x4821,
    .ram_size_default   = 128 * MiB,
    .linux_mem          = 0x04000000,
    .sdk_linux_mem      = 0x04000000,
    .na51055            = true,
    .num_cpus           = 2,
};

/* NT98529: NA51084, 512 MB, Linux 256 (cfg_529_IPCAM1_RAMDISK_EVB) */
static const NvtSoCConfig nt98529_soc = {
    .desc               = "Novatek NT98529 (2x Cortex-A9, NA51084, 512 MB)",
    .model              = "Novatek NA51055",
    .chip_id            = 0x5021,
    .ram_size_default   = NA51055_RAM_DEFAULT,
    .linux_mem          = 0x10000000,
    .sdk_linux_mem      = 0x04000000,
    .na51055            = true,
    .num_cpus           = 2,
};

/* "nvt,ca9" is what mach-nvt-ivot/board-dt.c matches. */
static const char nvt_compat[] = "novatek,na51089\0nvt,ca9";
static const char na51055_compat[] = "novatek,na51055\0nvt,ca9";

/*
 * cfg_562_IPCAM1_NOR_EVB/nvt-top.dtsi: the pinmux the board boots with.
 * The pinctrl driver refuses to probe without these children, and takes
 * each one's index as its PIN_FUNC id, so the order is the driver's.
 */
static const struct {
    const char *name;
    uint32_t pinmux;
} nvt_pinmux[] = {
    { "sdio", 0x5 }, { "sdio2", 0x5 }, { "sdio3", 0x0 }, { "nand", 0x5 },
    { "sensor", 0x0 }, { "sensor2", 0x0 }, { "mipi_lvds", 0x0 },
    { "i2c", 0x0 }, { "sif", 0x0 }, { "uart", 0x1 }, { "spi", 0x0 },
    { "sdp", 0x0 }, { "remote", 0x0 }, { "pwm", 0x0 }, { "pwm2", 0x0 },
    { "ccnt", 0x0 }, { "audio", 0x0 }, { "lcd", 0x10000000 }, { "tv", 0x0 },
    { "eth", 0x22 }, { "misc", 0x0 },
};

#define TYPE_NVT_MACHINE MACHINE_TYPE_NAME("novatek")
OBJECT_DECLARE_SIMPLE_TYPE(NvtMachineState, NVT_MACHINE)

struct NvtMachineState {
    MachineState parent_obj;

    char *flash_file;
    bool real_loader;
    bool rom_fallback;
    bool rom_fast;
    char *boot_file;
    uint32_t boot_strap;
    uint32_t flash_jedec;
    char *sensor;
    NvtFlashState *flash;
};

static const NvtSoCConfig *nvt_config;

/* The ADC and its thermal sensor (nvt-adc), NA51089 / NA51055 */
#define NVT_ADC_BASE        0xF0260000
#define NVT_IRQ_ADC         47
#define NS02501_ADC_BASE    0xF01D0000
#define NS02501_IRQ_ADC     73
#define NS02501_THERMAL     0xF0260000  /* nvtivot_thermal.c's block */

/* thermal_get_trimdata() / thermal_get_temp() per die (nvt-adc.c) */
static void nvt_adc_create(hwaddr base, qemu_irq irq, uint32_t chip_id)
{
    DeviceState *dev = qdev_new("nvt-adc");

    if (chip_id == 0x5521) {
        qdev_prop_set_uint8(dev, "layout", 1);      /* NS02501's */
    }

    object_property_add_child(qdev_get_machine(), "adc", OBJECT(dev));
    qdev_prop_set_uint32(dev, "trim", chip_id == 0x4821 ? 259 : 231);
    qdev_prop_set_uint32(dev, "step", chip_id == 0x4821 ? 1300 : 1319);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
}
static uint64_t nvt_linux_mem;

static unsigned nvt_num_cpus(const NvtSoCConfig *c)
{
    return c->num_cpus ? c->num_cpus : 1;
}

static hwaddr nvt_crypto_base(int n)
{
    if (nvt_config->na51055 && !strcmp(nvt_crypto_blocks[n].node, "hash")) {
        return NA51055_HASH_BASE;
    }
    return nvt_crypto_blocks[n].base;
}

/* UARTs and I2C controllers by index: NA51089's, then NA51055's extras. */
static int nvt_num_uarts(const NvtSoCConfig *c)
{
    return ARRAY_SIZE(nvt_uarts) + (c->na51055 ? ARRAY_SIZE(na51055_uarts) : 0);
}

static hwaddr nvt_uart_base(int n)
{
    return n < ARRAY_SIZE(nvt_uarts) ? nvt_uarts[n].base :
           na51055_uarts[n - ARRAY_SIZE(nvt_uarts)].base;
}

static int nvt_uart_irq(int n)
{
    return n < ARRAY_SIZE(nvt_uarts) ? nvt_uarts[n].irq :
           na51055_uarts[n - ARRAY_SIZE(nvt_uarts)].irq;
}

static uint32_t nvt_uart_clk(int n)
{
    return n < ARRAY_SIZE(nvt_uarts) ? nvt_uarts[n].clk : 48000000;
}

static int nvt_num_i2c(const NvtSoCConfig *c)
{
    return ARRAY_SIZE(nvt_i2c) + (c->na51055 ? ARRAY_SIZE(na51055_i2c) : 0);
}

static const char *nvt_i2c_node(int n)
{
    return n < ARRAY_SIZE(nvt_i2c) ? nvt_i2c[n].node :
           na51055_i2c[n - ARRAY_SIZE(nvt_i2c)].node;
}

static hwaddr nvt_i2c_base(int n)
{
    return n < ARRAY_SIZE(nvt_i2c) ? nvt_i2c[n].base :
           na51055_i2c[n - ARRAY_SIZE(nvt_i2c)].base;
}

static int nvt_i2c_irq(int n)
{
    return n < ARRAY_SIZE(nvt_i2c) ? nvt_i2c[n].irq :
           na51055_i2c[n - ARRAY_SIZE(nvt_i2c)].irq;
}

static const NvtMediaWin *nvt_media_wins(int n)
{
    int i;

    for (i = 0; nvt_config->na51055 && i < ARRAY_SIZE(na51055_media_override);
         i++) {
        if (!strcmp(nvt_media[n].compat, na51055_media_override[i].compat)) {
            return na51055_media_override[i].win;
        }
    }
    return nvt_media[n].win;
}

/*
 * NA51055's second core: the kernel writes its entry to the word the
 * loader parks it on, CC_CPU2_CPU1_CMDBUF_REG1 at the top of the boot
 * SRAM (na51055_ldr Memory.h; the tree's cpu-release-addr), from
 * nvt_ca9_smp_prepare_cpus(), then sends it a wake-up IPI and releases it
 * from nvt_secondary_startup's pen.  Here the core stays off until that
 * write, which starts it at the address; the rest of the page is storage.
 */
#define NA51055_RELEASE_BASE    0xF07F8000
#define NA51055_RELEASE_SIZE    0x1000

/* any access size: the loader's stack (from 0xF07F8FFC) is in the page */
static uint64_t na51055_release_read(void *opaque, hwaddr addr, unsigned size)
{
    return ldn_le_p((uint8_t *)opaque + addr, size);
}

static void na51055_release_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    CPUState *cs = qemu_get_cpu(1);

    stn_le_p((uint8_t *)opaque + addr, size, val);
    if (addr == 0 && size == 4 && val && cs &&
        ARM_CPU(cs)->power_state == PSCI_OFF) {
        arm_set_cpu_on(ARM_CPU(cs)->mp_affinity, (uint32_t)val, 0, 1, false);
    }
}

/*
 * The CPU block's core 2 reset control (0xF0E400F0): the loader's
 * bl_core2_reset() sets bits 2 / 10 / 17 / 21 to take core 2 out of
 * reset, which starts it at its reset vector, 0 -- where the loader has
 * put a jump to its core-2 program (the "core2entry1" region).
 */
#define NA51055_CPU_RESET       0xF0E400F0
#define NA51055_CPU_RESET_CORE2 (1u << 2)

static uint64_t na51055_cpu_reset_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    return *(uint32_t *)opaque;
}

static void na51055_cpu_reset_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    CPUState *cs = qemu_get_cpu(1);
    uint32_t old = *(uint32_t *)opaque;

    *(uint32_t *)opaque = val;
    if ((val & ~old & NA51055_CPU_RESET_CORE2) && cs &&
        ARM_CPU(cs)->power_state == PSCI_OFF) {
        /* out of reset: the highest exception level (EL3 if it has it) */
        int el = arm_feature(&ARM_CPU(cs)->env, ARM_FEATURE_EL3) ? 3 : 1;

        arm_set_cpu_on(ARM_CPU(cs)->mp_affinity, 0, 0, el, false);
    }
}

static const MemoryRegionOps na51055_cpu_reset_ops = {
    .read = na51055_cpu_reset_read,
    .write = na51055_cpu_reset_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps na51055_release_ops = {
    .read = na51055_release_read,
    .write = na51055_release_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.unaligned = true,
    .impl.unaligned = true,
};

/* -kernel: the second core waits for that write (on NS02501 the kernel's
 * write to CMDBUF_REG1), not on a boot stub. */
static void nvt_write_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
}

static void nvt_reset_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
}

static void nvt_fdt_irq(void *fdt, const char *path, int spi)
{
    qemu_fdt_setprop_cells(fdt, path, "interrupts", 0, spi, 4);
}

/*
 * The SoC part of cfg_562_IPCAM1_NOR_EVB (nvt-basic.dtsi, nvt-peri.dtsi,
 * nvt-mem-tbl.dtsi), cut to the blocks modelled here.  Node names are the
 * vendor's: the clock driver finds each gate by device name, e.g.
 * "f0290000.uart".
 */
static void *nvt_build_fdt(const struct arm_boot_info *info, int *size)
{
    const NvtSoCConfig *c = nvt_config;
    MachineState *ms = MACHINE(qdev_get_machine());
    bool nand = NVT_MACHINE(ms)->flash->nand;
    uint32_t gic, periph_clk, l2;
    const char *path;
    char *upath;
    void *fdt;
    int n;

    fdt = create_device_tree(size);
    if (!fdt) {
        return NULL;
    }
    gic = qemu_fdt_alloc_phandle(fdt);
    periph_clk = qemu_fdt_alloc_phandle(fdt);
    l2 = qemu_fdt_alloc_phandle(fdt);

    qemu_fdt_setprop_string(fdt, "/", "model", c->model);
    if (c->na51055) {
        qemu_fdt_setprop(fdt, "/", "compatible", na51055_compat,
                         sizeof(na51055_compat));
    } else {
        qemu_fdt_setprop(fdt, "/", "compatible", nvt_compat,
                         sizeof(nvt_compat));
    }
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "interrupt-parent", gic);

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path",
                            "/uart@f0290000");
    qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
                            "console=ttyS0,115200");

    qemu_fdt_add_subnode(fdt, "/memory");
    qemu_fdt_setprop_string(fdt, "/memory", "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, "/memory", "reg", 0, nvt_linux_mem);

    /* What the vendor U-Boot leaves for the HDAL modules (nvtmem.ko). */
    if (ms->ram_size > nvt_linux_mem) {
        qemu_fdt_add_subnode(fdt, "/hdal-memory");
        qemu_fdt_setprop_cell(fdt, "/hdal-memory", "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, "/hdal-memory", "#size-cells", 1);
        qemu_fdt_add_subnode(fdt, "/hdal-memory/media");
        qemu_fdt_setprop_cells(fdt, "/hdal-memory/media", "reg",
                               nvt_linux_mem, ms->ram_size - nvt_linux_mem);
    }

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    /* platsmp.c brings up the cores whose node has a cpu-release-addr. */
    for (n = ms->smp.cpus - 1; n >= 0; n--) {
        g_autofree char *cpath = g_strdup_printf("/cpus/cpu@%d", n);

        qemu_fdt_add_subnode(fdt, cpath);
        qemu_fdt_setprop_string(fdt, cpath, "device_type", "cpu");
        qemu_fdt_setprop_string(fdt, cpath, "compatible", "arm,cortex-a9");
        qemu_fdt_setprop_cell(fdt, cpath, "reg", n);
        if (n) {
            qemu_fdt_setprop_cell(fdt, cpath, "cpu-release-addr",
                                  NA51055_RELEASE_BASE);
        }
        qemu_fdt_setprop_cell(fdt, cpath, "next-level-cache", l2);
    }

    qemu_fdt_add_subnode(fdt, "/interrupt-controller@ffd00000");
    path = "/interrupt-controller@ffd00000";
    qemu_fdt_setprop_string(fdt, path, "compatible", "arm,cortex-a9-gic");
    qemu_fdt_setprop_cell(fdt, path, "#interrupt-cells", 3);
    qemu_fdt_setprop(fdt, path, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cells(fdt, path, "reg",
                           NVT_MPCORE_BASE + 0x1000, 0x1000,
                           NVT_MPCORE_BASE + 0x100, 0x100);
    qemu_fdt_setprop_cell(fdt, path, "phandle", gic);

    qemu_fdt_add_subnode(fdt, "/snoop-control-unit@ffd00000");
    qemu_fdt_setprop_string(fdt, "/snoop-control-unit@ffd00000",
                            "compatible", "arm,cortex-a9-scu");
    qemu_fdt_setprop_cells(fdt, "/snoop-control-unit@ffd00000", "reg",
                           NVT_MPCORE_BASE, 0x100);

    /*
     * Both timers tick at periph_clk, which the clock driver derives from
     * CG (cpu_clk / 8): see nvt-cg for why that is 100 MHz here.  Its
     * init reads CG through the base the "nvt,core_clk" init maps, and
     * of_clk_init() runs the two in tree order -- which is the reverse of
     * creation order, since fdt_add_subnode() puts a new node first.
     */
    qemu_fdt_add_subnode(fdt, "/periph_clk");
    qemu_fdt_setprop_string(fdt, "/periph_clk", "compatible",
                            "nvt,periph_clk");
    qemu_fdt_setprop_cell(fdt, "/periph_clk", "#clock-cells", 0);
    qemu_fdt_setprop_string(fdt, "/periph_clk", "clock-output-names",
                            "periph_clk");
    qemu_fdt_setprop_cell(fdt, "/periph_clk", "phandle", periph_clk);

    qemu_fdt_add_subnode(fdt, "/cg@f0020000");
    qemu_fdt_setprop_string(fdt, "/cg@f0020000", "compatible",
                            "nvt,core_clk");
    qemu_fdt_setprop_cells(fdt, "/cg@f0020000", "reg", NVT_CG_BASE, 0x1000);

    qemu_fdt_add_subnode(fdt, "/global_timer@ffd00200");
    path = "/global_timer@ffd00200";
    qemu_fdt_setprop_string(fdt, path, "compatible",
                            "arm,cortex-a9-global-timer");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_MPCORE_BASE + 0x200, 0x20);
    qemu_fdt_setprop_cells(fdt, path, "interrupts", 1, 11, 0xf01);
    qemu_fdt_setprop_cell(fdt, path, "clocks", periph_clk);

    qemu_fdt_add_subnode(fdt, "/private_timer@ffd00600");
    path = "/private_timer@ffd00600";
    qemu_fdt_setprop_string(fdt, path, "compatible",
                            "arm,cortex-a9-twd-timer");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_MPCORE_BASE + 0x600, 0x20);
    qemu_fdt_setprop_cells(fdt, path, "interrupts", 1, 13, 0xf01);
    qemu_fdt_setprop_cell(fdt, path, "clocks", periph_clk);

    qemu_fdt_add_subnode(fdt, "/cache-controller@ffe00000");
    path = "/cache-controller@ffe00000";
    qemu_fdt_setprop_string(fdt, path, "compatible", "arm,pl310-cache");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_L2_BASE, 0x1000);
    nvt_fdt_irq(fdt, path, NVT_IRQ_L2);
    qemu_fdt_setprop(fdt, path, "cache-unified", NULL, 0);
    qemu_fdt_setprop_cell(fdt, path, "cache-level", 2);
    qemu_fdt_setprop_cell(fdt, path, "phandle", l2);

    /* reg: TOP, PAD, GPIO -- nvt_get_chip_id() wants all six cells. */
    qemu_fdt_add_subnode(fdt, "/top@f0010000");
    qemu_fdt_setprop_string(fdt, "/top@f0010000", "compatible",
                            "nvt,nvt_top");
    qemu_fdt_setprop_cells(fdt, "/top@f0010000", "reg",
                           NVT_TOP_BASE, 0x2000, NVT_PAD_BASE, 0x2000,
                           NVT_GPIO_BASE, 0x10000);
    for (n = ARRAY_SIZE(nvt_pinmux) - 1; n >= 0; n--) {
        upath = g_strdup_printf("/top@f0010000/%s", nvt_pinmux[n].name);
        qemu_fdt_add_subnode(fdt, upath);
        qemu_fdt_setprop_cell(fdt, upath, "pinmux", nvt_pinmux[n].pinmux);
        g_free(upath);
    }

    for (n = 0; n < nvt_num_uarts(c); n++) {
        hwaddr base = nvt_uart_base(n);

        upath = g_strdup_printf("/uart@%" HWADDR_PRIx, base);
        path = upath;
        qemu_fdt_add_subnode(fdt, path);
        qemu_fdt_setprop_string(fdt, path, "compatible", "ns16550a");
        qemu_fdt_setprop_cells(fdt, path, "reg", base, 0x1000);
        nvt_fdt_irq(fdt, path, nvt_uart_irq(n));
        qemu_fdt_setprop_cell(fdt, path, "baud", 115200);
        qemu_fdt_setprop_cell(fdt, path, "reg-shift", 2);
        qemu_fdt_setprop_cell(fdt, path, "reg-io-width", 4);
        qemu_fdt_setprop_cell(fdt, path, "no-loopback-test", 1);
        qemu_fdt_setprop_cell(fdt, path, "clock-frequency", nvt_uart_clk(n));
        qemu_fdt_setprop_cell(fdt, path, "fifo-size", 64);
        qemu_fdt_setprop_cell(fdt, path, "uart_id", n);
        g_free(upath);
    }

    /* Built in: the reboot notifier powers off through it. */
    qemu_fdt_add_subnode(fdt, "/adc@f0260000");
    qemu_fdt_setprop_string(fdt, "/adc@f0260000", "compatible",
                            "nvt,nvt_adc");
    qemu_fdt_setprop_cells(fdt, "/adc@f0260000", "reg", NVT_ADC_BASE,
                           0x1000);
    nvt_fdt_irq(fdt, "/adc@f0260000", NVT_IRQ_ADC);
    qemu_fdt_setprop_cell(fdt, "/adc@f0260000", "#io-channel-cells", 1);

    qemu_fdt_add_subnode(fdt, "/rtc@f0060000");
    qemu_fdt_setprop_string(fdt, "/rtc@f0060000", "compatible",
                            "nvt,nvt_rtc");
    qemu_fdt_setprop_cells(fdt, "/rtc@f0060000", "reg", NVT_RTC_BASE, 0x100);
    nvt_fdt_irq(fdt, "/rtc@f0060000", NVT_IRQ_RTC);

    qemu_fdt_add_subnode(fdt, "/gpio@f0070000");
    qemu_fdt_setprop_string(fdt, "/gpio@f0070000", "compatible",
                            "nvt,nvt_gpio");
    qemu_fdt_setprop_cells(fdt, "/gpio@f0070000", "reg",
                           NVT_GPIO_BASE, 0x10000);
    nvt_fdt_irq(fdt, "/gpio@f0070000", NVT_IRQ_GPIO);
    qemu_fdt_setprop_cell(fdt, "/gpio@f0070000", "#gpio-cells", 2);

    /*
     * spinor.c / spinand.c name the MTD after nvt-devname:
     * mtdparts=spi_nor.0:... or spi_nand.0:..., as cfg_566_IPCAM1_EVB has.
     */
    path = nand ? "/nand@f0400000" : "/nor@f0400000";
    qemu_fdt_add_subnode(fdt, path);
    qemu_fdt_setprop_string(fdt, path, "compatible",
                            nand ? "nvt,nvt_spinand" : "nvt,nvt_spinor");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_FLASH_BASE, 0x1000);
    nvt_fdt_irq(fdt, path, NVT_IRQ_FLASH);
    qemu_fdt_setprop_cell(fdt, path, "clock-frequency", 48000000);
    qemu_fdt_setprop_string(fdt, path, "nvt-devname",
                            nand ? "spi_nand.0" : "spi_nor.0");
    if (!nand) {
        qemu_fdt_setprop_cell(fdt, path, "trace-stdtable", 1);
    }

    /*
     * The SD hosts (na51055_mmc.c, which names its clock after the device,
     * "f0420000.mmc").  No cd_gpio: the driver then takes the slot as
     * always full, and an empty one just times out.
     */
    for (n = ARRAY_SIZE(nvt_sdio) - 1; n >= 0; n--) {
        upath = g_strdup_printf("/mmc@%" HWADDR_PRIx, nvt_sdio[n].base);
        qemu_fdt_add_subnode(fdt, upath);
        qemu_fdt_setprop_string(fdt, upath, "compatible", nvt_sdio[n].compat);
        qemu_fdt_setprop_cells(fdt, upath, "reg", nvt_sdio[n].base, 0x1000);
        nvt_fdt_irq(fdt, upath, nvt_sdio[n].irq);
        qemu_fdt_setprop_cell(fdt, upath, "max-frequency", 48000000);
        qemu_fdt_setprop_cell(fdt, upath, "voltage-switch", 0);
        qemu_fdt_setprop_cell(fdt, upath, "max-voltage", 3300);
        qemu_fdt_setprop_cell(fdt, upath, "bus-width",
                              nvt_sdio[n].emmc ? 8 : 4);
        /*
         * NA51055: na51055_mmc.c re-applies the pads' drive strengths at
         * every clock change, and with the slot taken as full (no
         * cd_gpio) it re-initialises an empty one without end, logging
         * "No Valid DS value" each time for SDIO2's defaults.  So, as the
         * vendor EVB trees: SDIO1 polls C_GPIO 9 (the machine drives it),
         * SDIO2 has no card-detect pin and is scanned once.  The drive
         * strengths are left to the driver's per-chip defaults.
         */
        if (c->na51055 && n == 0) {
            qemu_fdt_setprop_cells(fdt, upath, "cd_gpio", NVT_SD_CD_GPIO, 0, 0);
        } else if (c->na51055) {
            qemu_fdt_setprop_cells(fdt, upath, "cd_gpio", 0, 0, 1);
        }
        g_free(upath);
    }

    /* The HDAL's kdrv_comm.ko times with these (timer_open_sys_timer()). */
    qemu_fdt_add_subnode(fdt, "/timer@f0040000");
    qemu_fdt_setprop_string(fdt, "/timer@f0040000", "compatible",
                            "nvt,nvt_timer");
    qemu_fdt_setprop_cells(fdt, "/timer@f0040000", "reg", NVT_TIMER_BASE,
                           0x300);
    nvt_fdt_irq(fdt, "/timer@f0040000", NVT_IRQ_TIMER);

    /* The vendor trees' node, which the kernel's pwm-na51055.c binds where
     * it is built; the HDAL's kdrv_comm.ko binds only nvt,nvt_kdrv_pwm. */
    qemu_fdt_add_subnode(fdt, "/pwm@f0210000");
    qemu_fdt_setprop_string(fdt, "/pwm@f0210000", "compatible",
                            "nvt,nvt_pwm");
    qemu_fdt_setprop_cells(fdt, "/pwm@f0210000", "reg", NVT_PWM_BASE, 0x2000);
    nvt_fdt_irq(fdt, "/pwm@f0210000", NVT_IRQ_PWM);

    for (n = nvt_num_i2c(c) - 1; n >= 0; n--) {
        upath = g_strdup_printf("/%s@%" HWADDR_PRIx, nvt_i2c_node(n),
                                nvt_i2c_base(n));
        qemu_fdt_add_subnode(fdt, upath);
        qemu_fdt_setprop_string(fdt, upath, "compatible", "nvt,nvt_i2c");
        qemu_fdt_setprop_cells(fdt, upath, "reg", nvt_i2c_base(n), 0x100);
        nvt_fdt_irq(fdt, upath, nvt_i2c_irq(n));
        qemu_fdt_setprop_cell(fdt, upath, "clock-frequency", 400000);
        qemu_fdt_setprop_cell(fdt, upath, "id", n);
        qemu_fdt_setprop_cell(fdt, upath, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, upath, "#size-cells", 0);
        g_free(upath);
    }

    /* The media engines, and the HDAL units and limits. */
    for (n = 0; n < ARRAY_SIZE(nvt_media); n++) {
        g_autofree char *mpath = g_strdup_printf("/%s", nvt_media[n].node);
        const NvtMediaWin *win = nvt_media_wins(n);
        uint32_t regs[2 * NVT_MEDIA_REGS], irqs[3 * NVT_MEDIA_REGS];
        int nr = 0, ni = 0, w;

        qemu_fdt_add_subnode(fdt, mpath);
        qemu_fdt_setprop_string(fdt, mpath, "compatible",
                                nvt_media[n].compat);
        for (w = 0; w < NVT_MEDIA_REGS && win[w].size; w++) {
            regs[nr++] = cpu_to_be32(win[w].base);
            regs[nr++] = cpu_to_be32(win[w].size);
            if (win[w].irq >= 0 || !strcmp(nvt_media[n].compat,
                                           "nvt,nvt_lvds")) {
                /* LVDS names CSI's interrupts, 54 and 55 */
                int spi = win[w].irq >= 0 ? win[w].irq : 54 + w;

                irqs[ni++] = cpu_to_be32(0);
                irqs[ni++] = cpu_to_be32(spi);
                irqs[ni++] = cpu_to_be32(4);
            }
        }
        qemu_fdt_setprop(fdt, mpath, "reg", regs, nr * 4);
        if (c->na51055 && !strcmp(nvt_media[n].compat, "nvt,kdrv_ise")) {
            irqs[ni++] = cpu_to_be32(0);
            irqs[ni++] = cpu_to_be32(NA51055_IRQ_ISE2);
            irqs[ni++] = cpu_to_be32(4);
        }
        if (ni) {
            qemu_fdt_setprop(fdt, mpath, "interrupts", irqs, ni * 4);
        }
        if (!strcmp(nvt_media[n].compat, "nvt,kdrv_ai")) {
            /* one rate an engine, as kdrv_ai reads it: NUE, NUE2, CNN(s) */
            if (c->na51055) {
                qemu_fdt_setprop_cells(fdt, mpath, "clock-frequency",
                                       600000000, 480000000, 600000000,
                                       600000000);
            } else {
                qemu_fdt_setprop_cells(fdt, mpath, "clock-frequency",
                                       600000000, 480000000, 600000000);
            }
        } else if (nvt_media[n].clock) {
            qemu_fdt_setprop_cell(fdt, mpath, "clock-frequency",
                                  nvt_media[n].clock);
        }
    }
    for (n = 0; n < ARRAY_SIZE(nvt_hdal_units); n++) {
        g_autofree char *unit = g_strdup_printf("/%s", nvt_hdal_units[n]);
        g_autofree char *compat = g_strdup_printf("nvt,%s",
                                                  nvt_hdal_units[n]);

        qemu_fdt_add_subnode(fdt, unit);
        qemu_fdt_setprop_string(fdt, unit, "compatible", compat);
    }
    qemu_fdt_add_subnode(fdt, "/audio");
    qemu_fdt_setprop_string(fdt, "/audio", "compatible", "nvt,nvt_audio");
    /* the embedded codec on I2S controller 4, as the EVB has it */
    qemu_fdt_add_subnode(fdt, "/audio@1");
    qemu_fdt_setprop_string(fdt, "/audio@1", "type", "none");
    qemu_fdt_add_subnode(fdt, "/audio@2");
    qemu_fdt_setprop_string(fdt, "/audio@2", "type", "embedded");
    qemu_fdt_setprop_cell(fdt, "/audio@2", "i2s_ctrl", 4);
    /* the HDAL units' path limits (nvt-mem-tbl.dtsi), which they size by */
    qemu_fdt_add_subnode(fdt, "/hdal-maxpath-cfg");
    for (n = 0; n < ARRAY_SIZE(nvt_hdal_limits); n++) {
        uint32_t v[8];
        int i;

        for (i = 0; i < nvt_hdal_limits[n].n; i++) {
            v[i] = cpu_to_be32(nvt_hdal_limits[n].v[i]);
        }
        qemu_fdt_setprop(fdt, "/hdal-maxpath-cfg", nvt_hdal_limits[n].prop,
                         v, nvt_hdal_limits[n].n * 4);
    }
    /*
     * nvt_preset_pll() (plat-na51089/pll_preset.c) sets each PLL's rate
     * from here at boot: PLL5 to 297 MHz, which the sensor master clock
     * divides to 27 MHz (ctl_sen fails "cannot find pll for MCLK"
     * without it).  <pll rate enable>, as the vendor tree has them.
     */
    qemu_fdt_add_subnode(fdt, "/pll_preset@0");
    for (n = 0; n < ARRAY_SIZE(nvt_pll_preset); n++) {
        g_autofree char *ppath = g_strdup_printf("/pll_preset@0/pll%d",
                                                 nvt_pll_preset[n][0]);

        qemu_fdt_add_subnode(fdt, ppath);
        qemu_fdt_setprop_cells(fdt, ppath, "pll_config", nvt_pll_preset[n][0],
                               nvt_pll_preset[n][1], nvt_pll_preset[n][2]);
    }
    qemu_fdt_add_subnode(fdt, "/kdrv_ipp");
    qemu_fdt_setprop_cell(fdt, "/kdrv_ipp", "clock-frequency", 240000000);
    /* the DDR arbiter: its protection window, past the DDRC registers */
    qemu_fdt_add_subnode(fdt, "/nvt_arb@f0000000");
    qemu_fdt_setprop_string(fdt, "/nvt_arb@f0000000", "compatible",
                            "nvt,nvt_arb");
    if (c->na51055) {
        qemu_fdt_setprop_cells(fdt, "/nvt_arb@f0000000", "reg",
                               0xF0000000, 0xA000, NA51055_DDRC2_BASE, 0xA000,
                               NVT_ARB2_BASE, 0x300, NA51055_ARB2B_BASE, 0x300);
        qemu_fdt_setprop_cells(fdt, "/nvt_arb@f0000000", "interrupts",
                               0, NVT_IRQ_ARB, 4, 0, NA51055_IRQ_ARB2, 4);
    } else {
        qemu_fdt_setprop_cells(fdt, "/nvt_arb@f0000000", "reg",
                               0xF0000000, 0xA000, NVT_ARB2_BASE, 0x300);
        nvt_fdt_irq(fdt, "/nvt_arb@f0000000", NVT_IRQ_ARB);
    }

    /*
     * The drivers bind these, and take "mclk" (1: 320 MHz) for the clock
     * they parent the block on.
     */
    for (n = 0; n < ARRAY_SIZE(nvt_crypto_blocks); n++) {
        g_autofree char *cpath = g_strdup_printf("/%s@%" HWADDR_PRIx,
                                                 nvt_crypto_blocks[n].node,
                                                 nvt_crypto_base(n));
        g_autofree char *compat = g_strdup_printf("nvt,nvt_%s",
                                                  nvt_crypto_blocks[n].node);

        qemu_fdt_add_subnode(fdt, cpath);
        qemu_fdt_setprop_string(fdt, cpath, "compatible", compat);
        qemu_fdt_setprop_cells(fdt, cpath, "reg", nvt_crypto_base(n), 0x100);
        nvt_fdt_irq(fdt, cpath, nvt_crypto_blocks[n].irq);
        qemu_fdt_setprop_cell(fdt, cpath, "mclk", 1);
    }

    /* The OTP / eFuse block: drivers (the USB PHY trim among them) call
     * efuse_readParamOps(), which needs this driver bound. */
    qemu_fdt_add_subnode(fdt, "/nvt_otp@f0660000");
    qemu_fdt_setprop_string(fdt, "/nvt_otp@f0660000", "compatible",
                            "nvt,nvt_otp");
    qemu_fdt_setprop_cells(fdt, "/nvt_otp@f0660000", "reg", NVT_OTP_BASE,
                           0x70);

    /* ehci-na51089.c (CONFIG_USB_EHCI_HCD_NVTIVOT); FOTG200's device
     * side, nvt,fotg200_udc, is not modelled. */
    path = "/nvt_usb2host@f0600000";
    qemu_fdt_add_subnode(fdt, path);
    qemu_fdt_setprop_string(fdt, path, "compatible", "nvt,ehci-nvtivot");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_USB_BASE, 0x10000);
    nvt_fdt_irq(fdt, path, NVT_IRQ_USB);

    /* The MAC, its Novatek glue and the embedded PHY in one 16 KiB block;
     * the vendor node maps the first 0x3800 and reaches past it. */
    qemu_fdt_add_subnode(fdt, "/eth@f02b0000");
    qemu_fdt_setprop_string(fdt, "/eth@f02b0000", "compatible",
                            "nvt,synopsys_eth");
    if (c->na51055) {
        /* NA51055's tree splits it around the MDIO pair at +0x200 */
        qemu_fdt_setprop_cells(fdt, "/eth@f02b0000", "reg", NVT_ETH_BASE,
                               0x200, NVT_ETH_BASE + 0x208, 0x35F8);
    } else {
        qemu_fdt_setprop_cells(fdt, "/eth@f02b0000", "reg", NVT_ETH_BASE,
                               0x3800);
    }
    nvt_fdt_irq(fdt, "/eth@f02b0000", NVT_IRQ_ETH);
    qemu_fdt_setprop_cell(fdt, "/eth@f02b0000", "sp-clk", 0);
    qemu_fdt_setprop_cell(fdt, "/eth@f02b0000", "ref-clk-out", 0);

    qemu_fdt_add_subnode(fdt, "/wdt@f0050000");
    qemu_fdt_setprop_string(fdt, "/wdt@f0050000", "compatible",
                            "nvt,nvt_wdt");
    qemu_fdt_setprop_cells(fdt, "/wdt@f0050000", "reg",
                           NVT_WDT_BASE, 0x10000);
    nvt_fdt_irq(fdt, "/wdt@f0050000", NVT_IRQ_WDT);

    /*
     * The display path (nvt-peri.dtsi, nvt-media.dtsi, nvt-display.dtsi):
     * the IDE, which nvt_ide.ko binds and finds its clocks by
     * ("f0800000.ide"), the HDAL's display object and device units, and
     * the EVB's LCD -- an HX8238D on 8-bit serial RGB whose panel driver
     * bit-bangs its registers on D_GPIO 3 / 4 / 5 (lcd_ctrl 1).  The
     * modules are not in the OpenIPC release (docs/novatek.md).
     */
    qemu_fdt_add_subnode(fdt, "/ide@f0800000");
    qemu_fdt_setprop_string(fdt, "/ide@f0800000", "compatible",
                            "nvt,nvt_ide");
    qemu_fdt_setprop_cells(fdt, "/ide@f0800000", "reg", NVT_IDE_BASE,
                           0x1000);
    nvt_fdt_irq(fdt, "/ide@f0800000", NVT_IRQ_IDE);
    /* the MIPI DSI host, for the DSI panels (nvt_dsi.ko) */
    qemu_fdt_add_subnode(fdt, "/dsi@f0840000");
    qemu_fdt_setprop_string(fdt, "/dsi@f0840000", "compatible",
                            "nvt,nvt_dsi");
    qemu_fdt_setprop_cells(fdt, "/dsi@f0840000", "reg", NVT_DSI_BASE,
                           0x1000);
    nvt_fdt_irq(fdt, "/dsi@f0840000", NVT_IRQ_DSI);
    qemu_fdt_add_subnode(fdt, "/dispobj");
    qemu_fdt_setprop_string(fdt, "/dispobj", "compatible", "nvt,nvt_dispobj");
    qemu_fdt_add_subnode(fdt, "/dispdev");
    qemu_fdt_setprop_string(fdt, "/dispdev", "compatible", "nvt,nvt_dispdev");
    qemu_fdt_add_subnode(fdt, "/display");
    qemu_fdt_setprop_string(fdt, "/display", "type", "lcd");
    qemu_fdt_setprop_cell(fdt, "/display", "lcd_ctrl", 1);
    qemu_fdt_setprop_cell(fdt, "/display", "sif_channel", 2);
    qemu_fdt_setprop_cell(fdt, "/display", "gpio_cs", NVT_D_GPIO(3));
    qemu_fdt_setprop_cell(fdt, "/display", "gpio_clk", NVT_D_GPIO(4));
    qemu_fdt_setprop_cell(fdt, "/display", "gpio_data", NVT_D_GPIO(5));

    return fdt;
}

/*
 * Flash boot.  On silicon the mask ROM loads the 32 KiB loader
 * (LD9856xA.bin) into SRAM; it trains DDR with closed libraries
 * (Reset_560.a) and then, in bl_func.c: copies the FDT from the fixed fdt
 * partition (0x10000 on NOR, block 2 -- 0x40000 -- on NAND) to
 * /nvt_memory_cfg/fdt, loads the uboot partition to
 * /nvt_memory_cfg/uboot -- unpacking a BCL1 one -- fills the SHMINFO block
 * at /nvt_memory_cfg/shmem that U-Boot's nvt_shminfo_init() insists on
 * ("LD_NVT", the FDT address), and calls U-Boot's entry.  DDR needs no
 * training here, so the machine does those steps itself and leaves a
 * two-word branch at 0 for the CPU to start on.
 */
#define NVT_NOR_FDT_OFF     0x10000
#define NVT_NAND_FDT_OFF    0x40000
#define NVT_SHMINFO_SIZE    328
#define NVT_BCL1            0x314C4342      /* "BCL1" */
#define NVT_BCL1_HDR        16
#define NVT_UBOOT_MAX       (8 * MiB)       /* U-Boot's region is 6.75 MiB */

static int nvt_fdt_reg(const void *fdt, const char *path, int cells,
                       uint64_t *base, uint64_t *size)
{
    int node = fdt_path_offset(fdt, path), len;
    const fdt32_t *reg;

    if (node < 0) {
        return -1;
    }
    reg = fdt_getprop(fdt, node, "reg", &len);
    if (!reg || len < 8 * cells) {
        return -1;
    }
    *base = cells == 2 ? ((uint64_t)fdt32_to_cpu(reg[0]) << 32) |
                         fdt32_to_cpu(reg[1]) : fdt32_to_cpu(reg[0]);
    *size = cells == 2 ? ((uint64_t)fdt32_to_cpu(reg[2]) << 32) |
                         fdt32_to_cpu(reg[3]) : fdt32_to_cpu(reg[1]);
    return 0;
}

/* The loader's LZ_Un_compress (U-Boot's lz_uncompress): LZ77 with a
 * marker byte and 7-bit variable-length length / offset. */
static bool nvt_lz_varsize(const uint8_t *in, uint32_t inlen, uint32_t *pos,
                           uint32_t *v)
{
    uint32_t b;

    *v = 0;
    do {
        if (*pos >= inlen) {
            return false;
        }
        b = in[(*pos)++];
        *v = (*v << 7) | (b & 0x7F);
    } while (b & 0x80);
    return true;
}

static bool nvt_lz_uncompress(const uint8_t *in, uint32_t inlen,
                              uint8_t *out, uint32_t outlen)
{
    uint32_t inpos = 1, outpos = 0, len, off, i;
    uint8_t marker, sym;

    if (inlen < 1) {
        return false;
    }
    marker = in[0];
    while (inpos < inlen) {
        sym = in[inpos++];
        if (sym != marker) {
            if (outpos >= outlen) {
                return false;
            }
            out[outpos++] = sym;
        } else if (inpos < inlen && in[inpos] == 0) {
            if (outpos >= outlen) {
                return false;
            }
            out[outpos++] = marker;
            inpos++;
        } else {
            if (!nvt_lz_varsize(in, inlen, &inpos, &len) ||
                !nvt_lz_varsize(in, inlen, &inpos, &off) ||
                off == 0 || off > outpos || len > outlen - outpos) {
                return false;
            }
            for (i = 0; i < len; i++, outpos++) {
                out[outpos] = out[outpos - off];
            }
        }
    }
    return outpos == outlen;
}

/*
 * Xiongmai's NA51103 images keep both LZMA-packed: the FDT behind an
 * 8-byte "XMFDT32G" tag at 0x10000 and U-Boot as the whole of
 * partition_boot.  Their loader (FW98332A.BIN) unpacks them itself.
 */
#define NVT_XMFDT_TAG       "XMFDT32G"
#define NVT_FDT_MAX         (1 * MiB)

static uint8_t *nvt_flash_fdt(NvtFlashState *flash, uint32_t off,
                              uint32_t *len)
{
    uint8_t hdr[16];
    uint8_t *fdt;

    nvt_flash_read(flash, off, hdr, sizeof(hdr));
    if (!memcmp(hdr, NVT_XMFDT_TAG, 8)) {
        uint32_t inlen = MIN(NVT_FDT_MAX, flash->size - off - 8);
        g_autofree uint8_t *in = g_malloc(inlen);
        ssize_t n;

        nvt_flash_read(flash, off + 8, in, inlen);
        fdt = g_malloc(NVT_FDT_MAX);
        n = nvt_lzma_decode(in, inlen, fdt, NVT_FDT_MAX);
        if (n < (ssize_t)sizeof(hdr) || ldl_be_p(fdt) != FDT_MAGIC ||
            ldl_be_p(fdt + 4) > n) {
            error_report("novatek: the XMFDT32G device tree at flash 0x%x "
                         "does not unpack", off);
            exit(1);
        }
        *len = ldl_be_p(fdt + 4);
        return fdt;
    }
    if (ldl_be_p(hdr) != FDT_MAGIC) {
        return NULL;
    }
    *len = ldl_be_p(hdr + 4);
    if (*len < sizeof(hdr) || *len > NVT_FDT_MAX) {
        error_report("novatek: device tree size 0x%x in flash", *len);
        exit(1);
    }
    fdt = g_malloc(*len);
    nvt_flash_read(flash, off, fdt, *len);
    return fdt;
}

/*
 * Xiongmai's NA51089 cameras have no fdt partition: their loader carries
 * the board's device tree itself, plain, behind an "XMFDT562" tag (at
 * loader +0x8000 in the 80N40 / 80N50 boards' loaders), and U-Boot follows
 * the loader at 0x10000.  So with no tree at the SDK's offset, look for one
 * in the loader: the first FDT there with an nvt_memory_cfg node.
 */
static uint8_t *nvt_loader_fdt(NvtFlashState *flash, uint32_t end,
                               uint32_t *len)
{
    g_autofree uint8_t *ldr = g_malloc(end);
    uint32_t off, size;

    nvt_flash_read(flash, 0, ldr, end);
    for (off = 0; off + 0x40 <= end; off += 4) {
        if (ldl_be_p(ldr + off) != FDT_MAGIC) {
            continue;
        }
        size = ldl_be_p(ldr + off + 4);
        if (size < 0x40 || size > end - off ||
            fdt_check_header(ldr + off) ||
            fdt_path_offset(ldr + off, "/nvt_memory_cfg") < 0) {
            continue;
        }
        *len = size;
        return g_memdup2(ldr + off, size);
    }
    return NULL;
}

/* The storage node: NA51089's names, or NA51068's SPI020 ones. */
static int nvt_flash_node(const void *fdt, bool nand)
{
    int node = fdt_node_offset_by_compatible(fdt, -1, nand ?
                                             "nvt,nvt_spinand" :
                                             "nvt,nvt_spinor");

    if (node < 0) {
        node = fdt_node_offset_by_compatible(fdt, -1, nand ?
                                             "nvt,nvt_spi020_nand" :
                                             "nvt,nvt_spi020_nor");
    }
    return node;
}

void nvt_flash_boot(NvtFlashState *flash, uint64_t ram_size, bool a64)
{
    g_autofree uint8_t *fdt = NULL;
    g_autofree uint8_t *part = NULL;
    g_autofree uint8_t *uboot = NULL;
    uint8_t shm[NVT_SHMINFO_SIZE] = { 0 };
    uint64_t fdt_addr, fdt_max, ub_addr, ub_max, shm_addr, shm_size;
    uint64_t ub_off, ub_len, ub_size;
    uint32_t stub[4], fdt_len;
    uint32_t fdt_off = flash->nand ? NVT_NAND_FDT_OFF : NVT_NOR_FDT_OFF;
    char path[96];
    size_t n;
    int nor;

    fdt = nvt_flash_fdt(flash, fdt_off, &fdt_len);
    if (!fdt && !flash->nand) {
        fdt = nvt_loader_fdt(flash, MIN(fdt_off, flash->size), &fdt_len);
    }
    if (!fdt) {
        error_report("novatek: no device tree at flash 0x%x%s: not a Novatek "
                     "flash image (qemu-boot/nvtpack2flash.py makes one from "
                     "an FW*.bin)", fdt_off,
                     flash->nand ? "" : " or in the loader before it");
        exit(1);
    }
    nor = nvt_flash_node(fdt, flash->nand);
    if (fdt_check_header(fdt) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/fdt", 1, &fdt_addr, &fdt_max) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/uboot", 1, &ub_addr, &ub_max) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/shmem", 1, &shm_addr, &shm_size) ||
        nor < 0 || fdt_get_path(fdt, nor, path, sizeof(path) - 20)) {
        error_report("novatek: the flash's device tree lacks nvt_memory_cfg "
                     "or the %s node", flash->nand ? "nand" : "nor");
        exit(1);
    }
    /* The SDK's name for it, or Xiongmai's. */
    n = strlen(path);
    strcpy(path + n, "/partition_uboot");
    if (fdt_path_offset(fdt, path) < 0) {
        strcpy(path + n, "/partition_boot");
    }
    if (nvt_fdt_reg(fdt, path, 2, &ub_off, &ub_size) ||
        ub_off + ub_size > flash->size || ub_size < 4 ||
        ub_size > NVT_UBOOT_MAX) {
        error_report("novatek: no usable partition_uboot in the flash's "
                     "device tree");
        exit(1);
    }

    part = g_malloc(ub_size);
    nvt_flash_read(flash, ub_off, part, ub_size);
    if (ldl_le_p(part) == NVT_BCL1) {
        uint32_t alg, raw, comp;

        if (ub_size < NVT_BCL1_HDR) {
            error_report("novatek: BCL1 U-Boot partition shorter than its "
                         "header");
            exit(1);
        }
        /* NVTPACK_BFC_HDR: FourCC, algorithm, sizes big-endian. */
        alg = ldl_be_p(part + 4) & 0xFF;
        raw = ldl_be_p(part + 8);
        comp = ldl_be_p(part + 12);

        if (alg != 9 && alg != 11) {
            error_report("novatek: U-Boot is BCL1 algorithm %u; only LZ (9) "
                         "and LZMA (11) are unpacked here", alg);
            exit(1);
        }
        if (comp > ub_size - NVT_BCL1_HDR || raw > ub_max ||
            raw > NVT_UBOOT_MAX) {
            error_report("novatek: BCL1 U-Boot sizes 0x%x/0x%x do not fit",
                         comp, raw);
            exit(1);
        }
        uboot = g_malloc(raw);
        /*
         * Algorithm 11 is what the SDK's "bfc c lzma" writes: an LZMA
         * "alone" stream (5 bytes of properties, the 64-bit unpacked size,
         * then the range-coded data), which the loader hands to the mask
         * ROM's decoder with the header's sizes.
         */
        if (alg == 11 ?
            nvt_lzma_decode(part + NVT_BCL1_HDR, comp, uboot, raw) != raw :
            !nvt_lz_uncompress(part + NVT_BCL1_HDR, comp, uboot, raw)) {
            error_report("novatek: BCL1 U-Boot (algorithm %u) does not "
                         "unpack to 0x%x bytes", alg, raw);
            exit(1);
        }
        ub_len = raw;
    } else {
        /*
         * Xiongmai's U-Boot is LZMA ("alone" format, its size often left
         * unknown), and their NA51089 trees say so with an "lzma" property
         * on the partition, which the loader honours.  Without it, anything
         * that does not unpack as LZMA is taken as a plain image.
         */
        if (nvt_lzma_probe(part, ub_size)) {
            ssize_t raw;

            uboot = g_malloc(MIN(ub_max, NVT_UBOOT_MAX));
            raw = nvt_lzma_decode(part, ub_size, uboot,
                                  MIN(ub_max, NVT_UBOOT_MAX));
            if (raw >= 4) {
                ub_len = raw;
            } else {
                g_clear_pointer(&uboot, g_free);
            }
        }
        if (!uboot && fdt_getprop(fdt, fdt_path_offset(fdt, path), "lzma",
                                  NULL)) {
            error_report("novatek: %s is marked lzma but does not unpack "
                         "as LZMA", path);
            exit(1);
        }
        if (!uboot) {
            uboot = g_steal_pointer(&part);
            ub_len = MIN(ub_size, ub_max);
        }
    }

    /* SHMINFO (arch/arm/include/asm/nvt-common/shm_info.h), as the loader
     * leaves it: name, boot storage (2 SPI NAND, 3 SPI NOR), FDT address,
     * normal boot. */
    memcpy(shm, "LD_NVT", 6);
    stw_le_p(shm + 42, flash->nand ? 2 : 3);    /* boot.LdStorage */
    stl_le_p(shm + 44, fdt_addr);           /* boot.fdt_addr */
    memcpy(shm + 112, "COMMINFO", 8);       /* comm.CommInfo_1 */
    if (shm_size < sizeof(shm) || fdt_len > fdt_max) {
        error_report("novatek: nvt_memory_cfg regions too small");
        exit(1);
    }
    if (fdt_addr + fdt_len > ram_size || shm_addr + sizeof(shm) > ram_size ||
        ub_addr + ub_len > ram_size) {
        error_report("novatek: nvt_memory_cfg places the boot images past "
                     "the end of RAM (0x%" PRIx64 ")", ram_size);
        exit(1);
    }

    rom_add_blob_fixed("novatek.fdt", fdt, fdt_len, fdt_addr);
    rom_add_blob_fixed("novatek.shminfo", shm, sizeof(shm), shm_addr);
    rom_add_blob_fixed("novatek.uboot", uboot, ub_len, ub_addr);
    if (a64) {
        stub[0] = cpu_to_le32(0x58000041);  /* ldr x1, 1f */
        stub[1] = cpu_to_le32(0xD61F0020);  /* br x1 */
        stq_le_p(&stub[2], ub_addr);        /* 1: .quad U-Boot */
    } else {
        stub[0] = cpu_to_le32(0xE51FF004);  /* ldr pc, [pc, #-4] */
        stub[1] = cpu_to_le32(ub_addr);
    }
    rom_add_blob_fixed("novatek.bootrom", stub, a64 ? 16 : 8, 0);
}

/*
 * The flash's own loader: a header whose first words are the entry and the
 * load address in SRAM (LD9856xA.bin: 0xF07C0028, 0xF07C0000).
 */
#define NVT_SRAM_BASE       0xF07C0000
#define NVT_SRAM_SIZE       0x40000

static const char *nvt_rom_loader_file;

void nvt_rom_set_loader_file(const char *path)
{
    nvt_rom_loader_file = path;
}

static bool nvt_rom_fallback, nvt_rom_fast;
static uint32_t nvt_rom_strap;
static DeviceState *nvt_rom_eth;        /* the MAC its Ethernet boot drives */

void nvt_rom_set_fallback(bool on, uint32_t strap, bool fast)
{
    nvt_rom_fallback = on;
    nvt_rom_strap = strap;
    nvt_rom_fast = fast;
}

/*
 * What the ROM model starts from flash (nvt-bootrom.c, rom_first_source):
 * all but SD (1), Ethernet (4), USB (5, 0xB) and UART (0xC).
 */
bool nvt_rom_strap_is_flash(uint32_t strap)
{
    switch (strap & 0xF) {
    case 1: case 4: case 5: case 0xB: case 0xC:
        return false;
    default:
        return true;
    }
}

bool nvt_flash_has_loader(NvtFlashState *flash, uint64_t sram_base,
                          uint64_t sram_size)
{
    uint32_t hdr[2];

    if (nvt_rom_loader_file) {
        return true;
    }
    nvt_flash_read(flash, 0, hdr, sizeof(hdr));
    return le32_to_cpu(hdr[1]) == sram_base &&
           le32_to_cpu(hdr[0]) - sram_base < sram_size;
}

/*
 * Boot as the silicon does: the mask ROM (nvt-bootrom) copies the loader
 * to SRAM and runs it; the loader brings DDR up and loads U-Boot.
 */
void nvt_rom_boot(NvtFlashState *flash, const NvtRomConfig *rc)
{
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    DeviceState *rom;

    memory_region_init_ram(sram, NULL, "nvt.sram", rc->sram_size,
                           &error_fatal);
    /* beneath the pages devices keep in it (the release word) */
    memory_region_add_subregion_overlap(get_system_memory(), rc->sram_base,
                                        sram, 0);
    rom = qdev_new(TYPE_NVT_BOOTROM);
    object_property_add_child(qdev_get_machine(), "bootrom", OBJECT(rom));
    object_property_set_link(OBJECT(rom), "flash", OBJECT(flash),
                             &error_fatal);
    qdev_prop_set_uint64(rom, "sram-base", rc->sram_base);
    qdev_prop_set_uint64(rom, "sram-size", rc->sram_size);
    qdev_prop_set_uint64(rom, "sp", rc->sp);
    qdev_prop_set_uint64(rom, "uart", rc->uart);
    qdev_prop_set_uint8(rom, "layout", rc->layout);
    if (nvt_rom_loader_file) {
        qdev_prop_set_string(rom, "loader-file", nvt_rom_loader_file);
    }
    if (nvt_rom_fallback && rc->layout == 0) {
        qdev_prop_set_bit(rom, "fallback", true);
        qdev_prop_set_bit(rom, "fast", nvt_rom_fast);
        qdev_prop_set_uint8(rom, "strap", nvt_rom_strap & 0xF);
        if (nvt_rom_eth) {
            object_property_set_link(OBJECT(rom), "eth", OBJECT(nvt_rom_eth),
                                     &error_fatal);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(rom), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(rom), 0,
                            object_property_get_uint(OBJECT(rom), "svc-base",
                                                     &error_fatal), 10);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(rom), 1,
                            object_property_get_uint(OBJECT(rom), "rom-base",
                                                     &error_fatal), 10);
}

/*
 * -bios: a mask ROM image of one's own -- OpenIPC/opennovatek's re.elf,
 * the NA51089 ROM reconstructed as source -- runs in place of the model.
 * It is loaded twice: read-only at its home (0x80000000, which the
 * loaders' DDR bring-up copies from) and into DRAM at 0, where it runs,
 * as the copy leaves it.  The SRAM it keeps its data and stack in is
 * mapped; nothing else of nvt-bootrom is used.  The CPU starts at 0.
 */
#define NVT_ROM_HOME        0x80000000
#define NVT_ROM_SIZE        0x8000

static uint64_t nvt_rom_home(void *opaque, uint64_t addr)
{
    return addr + NVT_ROM_HOME;
}

static void nvt_maskrom_load(const char *file)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *home = g_new(MemoryRegion, 1);
    uint64_t entry, low, high;
    ssize_t n;

    memory_region_init_ram(sram, NULL, "nvt.sram", NVT_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion_overlap(sysmem, NVT_SRAM_BASE, sram, 0);
    memory_region_init_rom(home, NULL, "nvt.maskrom", NVT_ROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion_overlap(sysmem, NVT_ROM_HOME, home, 10);

    n = load_elf(file, NULL, nvt_rom_home, NULL, &entry, &low, &high, NULL,
                 ELFDATA2LSB, EM_ARM, 1, 0);
    if (n < 0) {
        error_report("novatek: cannot load -bios '%s': %s", file,
                     load_elf_strerror(n));
        exit(1);
    }
    if (low != NVT_ROM_HOME || high > NVT_ROM_HOME + NVT_ROM_SIZE) {
        error_report("novatek: -bios '%s' spans 0x%" PRIx64 "..0x%" PRIx64
                     ", not a mask ROM linked at 0 (32 KiB)", file,
                     low - NVT_ROM_HOME, high - NVT_ROM_HOME);
        exit(1);
    }
    load_elf(file, NULL, NULL, NULL, NULL, NULL, NULL, NULL, ELFDATA2LSB,
             EM_ARM, 1, 0);
}

/* the ROMs of NA51089 / NA51055 (layout 0 / 1) and NS02501 (2) */
static const NvtRomConfig nvt_rom_cfg[] = {
    { NVT_SRAM_BASE, NVT_SRAM_SIZE, 0xF07F8FFC, 0xF0290000, 0 },
    { NVT_SRAM_BASE, NVT_SRAM_SIZE, 0xF07F8FFC, 0xF0290000, 1 },
    { NVT_SRAM_BASE, NVT_SRAM_SIZE, 0xF07F8FFC, 0xF0290000, 2 },
};

static void novatek_common_init(MachineState *machine, const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq pic[NVT_NUM_IRQ - 32];
    DeviceState *mp, *dev, *flash, *cg, *gpio, *ife = NULL;
    I2CBus *i2c0 = NULL;
    SysBusDevice *mpbus;
    unsigned int smp_cpus = machine->smp.cpus;
    ARMCPU *cpu[2];
    int n;

    nvt_config = c;
    nvt_linux_mem = machine->ram_size >= c->ram_size_default ? c->linux_mem :
                    MIN(c->sdk_linux_mem, machine->ram_size);

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        /* Entered in SVC by the loader and U-Boot; there is no TEE here. */
        if (object_property_find(cpuobj, "has_el3")) {
            object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
        }
        object_property_set_int(cpuobj, "reset-cbar", NVT_MPCORE_BASE,
                                &error_fatal);
        /*
         * QEMU's A9 is r0p0, and Linux's arm_global_timer will not run on
         * anything before r2p0 (an erratum in the comparator), leaving
         * sched_clock on jiffies.  The NT9856x part is a late A9: say r4p1.
         */
        object_property_set_uint(cpuobj, "midr", 0x414fc091, &error_fatal);
        /* NA51055's second core waits for its release word, below. */
        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);

    /* Everything not modelled below reads as zero and is logged. */
    create_unimplemented_device("nvt-periph", NVT_PERIPH_BASE,
                                NVT_PERIPH_SIZE);

    mp = qdev_new("a9mpcore_priv");
    mpbus = SYS_BUS_DEVICE(mp);
    qdev_prop_set_uint32(mp, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(mp, "num-irq", NVT_NUM_IRQ);
    sysbus_realize_and_unref(mpbus, &error_fatal);
    sysbus_mmio_map(mpbus, 0, NVT_MPCORE_BASE);
    for (n = 0; n < smp_cpus; n++) {
        DeviceState *cpudev = DEVICE(cpu[n]);

        sysbus_connect_irq(mpbus, n, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
        sysbus_connect_irq(mpbus, n + smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
        sysbus_connect_irq(mpbus, n + 2 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
        sysbus_connect_irq(mpbus, n + 3 * smp_cpus,
                           qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }
    for (n = 0; n < NVT_NUM_IRQ - 32; n++) {
        pic[n] = qdev_get_gpio_in(mp, n);
    }

    if (smp_cpus > 1) {
        MemoryRegion *rel = g_new(MemoryRegion, 1);

        memory_region_init_io(rel, OBJECT(machine), &na51055_release_ops,
                              g_malloc0(NA51055_RELEASE_SIZE),
                              "na51055-release", NA51055_RELEASE_SIZE);
        memory_region_add_subregion_overlap(sysmem, NA51055_RELEASE_BASE,
                                            rel, 1);
        {
            MemoryRegion *rst = g_new(MemoryRegion, 1);

            memory_region_init_io(rst, OBJECT(machine),
                                  &na51055_cpu_reset_ops, g_new0(uint32_t, 1),
                                  "na51055.cpu-reset", 4);
            memory_region_add_subregion_overlap(sysmem, NA51055_CPU_RESET,
                                                rst, 1);
        }
    }

    sysbus_create_simple("l2x0", NVT_L2_BASE, NULL);

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
    qdev_prop_set_uint32(dev, "strap", nms->boot_strap);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_TOP_BASE, 1);

    cg = qdev_new("nvt-cg");
    /* nvt-cg "layout": where the die keeps its PLL ratio registers */
    if (c->na51055) {
        qdev_prop_set_uint8(cg, "layout", c->chip_id == 0x4821 ? 1 : 2);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cg), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(cg), 0, NVT_CG_BASE, 1);

    dev = qdev_new("nvt-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_WDT_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_WDT]);

    dev = qdev_new("nvt-rtc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_RTC_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_RTC]);

    nvt_adc_create(NVT_ADC_BASE, pic[NVT_IRQ_ADC], c->chip_id);

    /* A named child, so qtest and a board can reach its pins at
     * /machine/nvt-gpio. */
    gpio = qdev_new("nvt-gpio");
    object_property_add_child(OBJECT(machine), "nvt-gpio", OBJECT(gpio));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(gpio), 0, NVT_GPIO_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(gpio), 0, pic[NVT_IRQ_GPIO]);

    sysbus_create_simple("nvt-otp", NVT_OTP_BASE, NULL);
    sysbus_create_simple("nvt-timer", NVT_TIMER_BASE, pic[NVT_IRQ_TIMER]);
    for (n = 0; n < ARRAY_SIZE(nvt_media); n++) {
        const NvtMediaWin *win = nvt_media_wins(n);
        int w;

        for (w = 0; w < NVT_MEDIA_REGS && win[w].size; w++) {
            g_autofree char *engine = g_strndup(nvt_media[n].node,
                                strcspn(nvt_media[n].node, "@"));

            if (w) {
                char *e = g_strdup_printf("%s%d", engine, w + 1);

                g_free(engine);
                engine = e;
            }
            if (!strcmp(engine, "eac") || !strcmp(engine, "dai")) {
                continue;               /* nvt-audio, below */
            }
            dev = qdev_new("nvt-media");
            qdev_prop_set_string(dev, "engine", engine);
            qdev_prop_set_uint32(dev, "size", win[w].size);
            if (!strcmp(engine, "ife")) {
                ife = dev;
            } else if (!strcmp(engine, "ime")) {
                object_property_set_link(OBJECT(dev), "ipp-input",
                                         OBJECT(ife), &error_fatal);
            }
            object_property_add_child(OBJECT(machine), engine, OBJECT(dev));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, win[w].base);
            if (win[w].irq >= 0) {
                sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[win[w].irq]);
            }
        }
    }
    /*
     * Audio: the DAI and the EAC, one device; its rate comes from the CG,
     * and with -audiodev / -audio (the machine's audiodev) it plays and,
     * if asked to, captures through the host.
     */
    dev = qdev_new("nvt-audio");
    object_property_set_link(OBJECT(dev), "cg", OBJECT(cg), &error_fatal);
    if (machine->audiodev) {
        qdev_prop_set_string(dev, "audiodev", machine->audiodev);
    }
    object_property_add_child(OBJECT(machine), "audio", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NVT_DAI_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, NVT_EAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_DAI]);

    /* the display engine, shown on QEMU's console */
    sysbus_create_simple("nvt-ide", NVT_IDE_BASE, pic[NVT_IRQ_IDE]);
    sysbus_create_simple("nvt-dsi", NVT_DSI_BASE, pic[NVT_IRQ_DSI]);

    dev = qdev_new("nvt-media");
    qdev_prop_set_string(dev, "engine", "arb2");
    qdev_prop_set_uint32(dev, "size", 0x300);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NVT_ARB2_BASE);
    if (c->na51055) {
        dev = qdev_new("nvt-media");
        qdev_prop_set_string(dev, "engine", "arb2b");
        qdev_prop_set_uint32(dev, "size", 0x300);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NA51055_ARB2B_BASE);

        /* the second DDR controller / PHY: as the first */
        dev = qdev_new("nvt-ddrc");
        qdev_prop_set_bit(dev, "phy-pages", c->chip_id == 0x5021);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51055_DDRC2_BASE, 1);
    }

    for (n = 0; n < ARRAY_SIZE(nvt_crypto_blocks); n++) {
        dev = qdev_new(nvt_crypto_blocks[n].type);
        if (nvt_config->na51055 && !strcmp(nvt_crypto_blocks[n].node,
                                           "hash")) {
            /* NA51055's engine: the IV byte-reversed (nvt-hash.c) */
            qdev_prop_set_bit(dev, "iv-reversed", true);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, nvt_crypto_base(n));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           pic[nvt_crypto_blocks[n].irq]);
    }

    /* Named, so qtest and a board can watch its "out" lines. */
    dev = qdev_new("nvt-pwm");
    object_property_add_child(OBJECT(machine), "nvt-pwm", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NVT_PWM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_PWM]);

    for (n = 0; n < nvt_num_i2c(c); n++) {
        dev = sysbus_create_simple("nvt-i2c", nvt_i2c_base(n),
                                   pic[nvt_i2c_irq(n)]);
        if (n == 0) {
            i2c0 = (I2CBus *)qdev_get_child_bus(dev, "i2c");
        }
    }
    image_sensor_create(i2c0, nms->sensor);

    /*
     * USB: the EHCI (region 0) over the FOTG200 glue (region 1), at the
     * block's address and at the alias ehci-na51089.c uses on NA51089.
     */
    dev = qdev_new("nvt-usb");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_USB]);
    for (n = 0; n < 2; n++) {
        MemoryRegion *ehci = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
        MemoryRegion *glue = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
        hwaddr base = n ? NVT_USB_ALIAS : NVT_USB_BASE;

        if (n) {
            MemoryRegion *a = g_new(MemoryRegion, 2);

            memory_region_init_alias(&a[0], OBJECT(dev), "nvt-usb.ehci-alias",
                                     ehci, 0, memory_region_size(ehci));
            memory_region_init_alias(&a[1], OBJECT(dev), "nvt-usb.glue-alias",
                                     glue, 0, memory_region_size(glue));
            ehci = &a[0];
            glue = &a[1];
        }
        memory_region_add_subregion_overlap(sysmem, base, glue, 1);
        memory_region_add_subregion_overlap(sysmem, base, ehci, 2);
    }

    dev = qdev_new("nvt-eqos");
    /* NA51055's DMA skips in units of 4 bytes (eth_na51055.c), not 8 */
    qdev_prop_set_uint32(dev, "dsl-unit", c->chip_id == 0x4821 ? 4 : 8);
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    nvt_rom_eth = dev;
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_ETH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_ETH]);

    for (n = 0; n < ARRAY_SIZE(nvt_sdio); n++) {
        DriveInfo *di = drive_get(IF_SD, 0, n);

        dev = qdev_new("nvt-sdio");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, nvt_sdio[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[nvt_sdio[n].irq]);
        qdev_connect_gpio_out_named(cg, "sys-reset", nvt_sdio[n].reset,
                                    qdev_get_gpio_in_named(dev, "reset", 0));
        if (n == 0) {
            qemu_set_irq(qdev_get_gpio_in_named(gpio, "in", NVT_SD_CD_GPIO),
                         !di);
        }
        if (di) {
            DeviceState *card = qdev_new(nvt_sdio[n].emmc ? TYPE_EMMC
                                                           : TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    flash = qdev_new(TYPE_NVT_FLASH);
    dev = flash;
    if (nms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", nms->flash_file);
    }
    qdev_prop_set_uint32(dev, "jedec", nms->flash_jedec);
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    nms->flash = NVT_FLASH(dev);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_FLASH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_FLASH]);

    for (n = 0; n < nvt_num_uarts(c); n++) {
        /*
         * Under a mask ROM image (-bios), UART 0 loses its input while
         * nobody reads it, as silicon's does: the ROM leaves it alone for
         * a minute while it tries SD, Ethernet and USB, then resets the
         * FIFO and frames 16-byte commands from what comes after.
         */
        if (n == 0 && machine->firmware && !machine->kernel_filename) {
            dev = qdev_new(TYPE_SERIAL_MM);
            qdev_prop_set_uint8(dev, "regshift", 2);
            qdev_prop_set_uint32(dev, "baudbase", nvt_uart_clk(n) / 16);
            qdev_prop_set_chr(dev, "chardev", serial_hd(n));
            qdev_set_legacy_instance_id(dev, nvt_uart_base(n), 2);
            qdev_prop_set_uint8(dev, "endianness", DEVICE_LITTLE_ENDIAN);
            qdev_prop_set_uint32(dev, "rx-idle-drop", NVT_ROM_UART_IDLE_MS);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[nvt_uart_irq(n)]);
            memory_region_add_subregion(sysmem, nvt_uart_base(n),
                    sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
            continue;
        }
        serial_mm_init(sysmem, nvt_uart_base(n), 2, pic[nvt_uart_irq(n)],
                       nvt_uart_clk(n) / 16, serial_hd(n),
                       DEVICE_LITTLE_ENDIAN);
    }

    for (n = 0; n < ARRAY_SIZE(nvt_regbanks); n++) {
        /* the DDR controller / PHY: storage with the loader's done bits */
        if (!strcmp(nvt_regbanks[n].name, "nvt-ddrc")) {
            dev = qdev_new("nvt-ddrc");
            /* NA51084's DDR3-2133 PHY is paged */
            qdev_prop_set_bit(dev, "phy-pages", c->chip_id == 0x5021);
        } else {
            dev = qdev_new("hisi-regbank");
            qdev_prop_set_uint32(dev, "size", nvt_regbanks[n].size);
            qdev_prop_set_string(dev, "name", nvt_regbanks[n].name);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                nvt_regbanks[n].base, 1);
    }

    binfo.ram_size = nvt_linux_mem;
    binfo.loader_start = 0;
    binfo.board_id = -1;
    /* Without -kernel the flash's own device tree is the one that counts,
     * and boot.c would place ours at 0, over the boot stub. */
    if (machine->kernel_filename) {
        binfo.get_dtb = nvt_build_fdt;
    }
    binfo.write_secondary_boot = nvt_write_secondary;
    binfo.secondary_cpu_reset_hook = nvt_reset_secondary;
    arm_load_kernel(cpu[0], machine, &binfo);

    if (machine->firmware && !machine->kernel_filename) {
        if (c->na51055) {
            error_report("novatek: -bios takes an NA51089 mask ROM; this "
                         "machine's ROM is not reconstructed");
            exit(1);
        }
        nvt_maskrom_load(machine->firmware);
    } else if (!machine->kernel_filename && nms->flash_file) {
        /* NA51089's ROM: the fallback, asked for or implied by the straps */
        bool fallback = nms->real_loader && !c->na51055 &&
                        (nms->rom_fallback ||
                         !nvt_rom_strap_is_flash(nms->boot_strap));

        nvt_rom_set_loader_file(nms->boot_file);
        nvt_rom_set_fallback(fallback, nms->boot_strap, nms->rom_fast);
        if (nms->real_loader &&
            (fallback || nvt_flash_has_loader(NVT_FLASH(flash), NVT_SRAM_BASE,
                                              NVT_SRAM_SIZE))) {
            /* NA51055's ROM has its routines at fixed addresses */
            nvt_rom_boot(NVT_FLASH(flash), &nvt_rom_cfg[c->na51055 ? 1 : 0]);
        } else {
            nvt_flash_boot(NVT_FLASH(flash), machine->ram_size, false);
        }
    }
}

static char *nvt_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(NVT_MACHINE(obj)->flash_file);
}

static void nvt_machine_set_flash_file(Object *obj, const char *value,
                                       Error **errp)
{
    NvtMachineState *s = NVT_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *nvt_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(NVT_MACHINE(obj)->sensor);
}

static void nvt_machine_set_sensor(Object *obj, const char *value,
                                   Error **errp)
{
    NvtMachineState *s = NVT_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

static void nvt_machine_get_flash_jedec(Object *obj, Visitor *v,
                                        const char *name, void *opaque,
                                        Error **errp)
{
    visit_type_uint32(v, name, &NVT_MACHINE(obj)->flash_jedec, errp);
}

static void nvt_machine_set_flash_jedec(Object *obj, Visitor *v,
                                        const char *name, void *opaque,
                                        Error **errp)
{
    visit_type_uint32(v, name, &NVT_MACHINE(obj)->flash_jedec, errp);
}

static bool nvt_machine_get_loader(Object *obj, Error **errp)
{
    return NVT_MACHINE(obj)->real_loader;
}

static void nvt_machine_set_loader(Object *obj, bool value, Error **errp)
{
    NVT_MACHINE(obj)->real_loader = value;
}

static char *nvt_machine_get_boot_file(Object *obj, Error **errp)
{
    return g_strdup(NVT_MACHINE(obj)->boot_file);
}

static void nvt_machine_set_boot_file(Object *obj, const char *value,
                                      Error **errp)
{
    g_free(NVT_MACHINE(obj)->boot_file);
    NVT_MACHINE(obj)->boot_file = g_strdup(value);
}

static void nvt_machine_get_boot_strap(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    visit_type_uint32(v, name, &NVT_MACHINE(obj)->boot_strap, errp);
}

static void nvt_machine_set_boot_strap(Object *obj, Visitor *v,
                                       const char *name, void *opaque,
                                       Error **errp)
{
    visit_type_uint32(v, name, &NVT_MACHINE(obj)->boot_strap, errp);
}

static bool nvt_machine_get_rom_fallback(Object *obj, Error **errp)
{
    return NVT_MACHINE(obj)->rom_fallback;
}

static void nvt_machine_set_rom_fallback(Object *obj, bool value, Error **errp)
{
    NVT_MACHINE(obj)->rom_fallback = value;
}

static bool nvt_machine_get_rom_fast(Object *obj, Error **errp)
{
    return NVT_MACHINE(obj)->rom_fast;
}

static void nvt_machine_set_rom_fast(Object *obj, bool value, Error **errp)
{
    NVT_MACHINE(obj)->rom_fast = value;
}

static void nvt_machine_instance_init(Object *obj)
{
    NVT_MACHINE(obj)->real_loader = true;
}

static void nvt_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    object_class_property_add_bool(oc, "loader", nvt_machine_get_loader,
                                   nvt_machine_set_loader);
    object_class_property_set_description(oc, "loader",
        "with flash-file: run the flash's own loader from the mask ROM "
        "model when the image has one (default on); off, or no loader in "
        "the image, loads U-Boot with the built-in stand-in");

    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");
    mc->default_ram_id = "novatek.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "boot-file",
        nvt_machine_get_boot_file, nvt_machine_set_boot_file);
    object_class_property_set_description(oc, "boot-file",
        "loader image the mask ROM got by TFTP or UART, in place of the "
        "flash's: the ROM's own fetch is not modelled");
    object_class_property_add(oc, "boot-strap", "uint32",
        nvt_machine_get_boot_strap, nvt_machine_set_boot_strap, NULL, NULL);
    object_class_property_set_description(oc, "boot-strap",
        "boot strap pins as TOP+0 reads them (BS[3:0]: NT9856x 4 Ethernet, "
        "0xC UART)");
    object_class_property_add_bool(oc, "rom-fallback",
        nvt_machine_get_rom_fallback, nvt_machine_set_rom_fallback);
    object_class_property_set_description(oc, "rom-fallback",
        "NA51089: with no valid loader in flash, run the mask ROM's own "
        "fallback (SD, Ethernet, USB, UART) as the silicon does, instead of "
        "the stand-in; implied by boot-strap other than flash");
    object_class_property_add_bool(oc, "rom-fast",
        nvt_machine_get_rom_fast, nvt_machine_set_rom_fast);
    object_class_property_set_description(oc, "rom-fast",
        "the ROM fallback's waits before UART (DHCP 5 s, USB 30 s) / 10");
    object_class_property_add_str(oc, "flash-file",
        nvt_machine_get_flash_file, nvt_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image (power-of-two size, 1..32 MiB) or SPI NAND image "
        "(128 or 256 MiB, or 2112 bytes a page with the OOB); erases and "
        "programs are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        nvt_machine_get_flash_jedec, nvt_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the flash answers with; default a Winbond W25Q (NOR) or "
        "W25N (NAND) of the image's size");
    object_class_property_add_str(oc, "sensor",
        nvt_machine_get_sensor, nvt_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor on I2C 0 (none, sc401ai, sc501ai); default none");
    machine_add_audiodev_property(mc);
}

static const TypeInfo nvt_machine_info = {
    .name           = TYPE_NVT_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(NvtMachineState),
    .class_init     = nvt_machine_class_init,
    .instance_init  = nvt_machine_instance_init,
    .interfaces     = arm_machine_interfaces,
};

static void nvt_machine_register_types(void)
{
    type_register_static(&nvt_machine_info);
}

type_init(nvt_machine_register_types)

#define DEFINE_NVT_MACHINE(namestr, tag, config)                     \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        novatek_common_init(machine, &config);                       \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->max_cpus = nvt_num_cpus(&config);                        \
        mc->default_cpus = nvt_num_cpus(&config);                    \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, NVT_MACHINE, NvtMachineState,   \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_NVT_MACHINE("nt98562", nt98562, nt98562_soc)
DEFINE_NVT_MACHINE("nt98566", nt98566, nt98566_soc)
DEFINE_NVT_MACHINE("nt98525", nt98525, nt98525_soc)
DEFINE_NVT_MACHINE("nt98528", nt98528, nt98528_soc)
DEFINE_NVT_MACHINE("nt98560", nt98560, nt98560_soc)
DEFINE_NVT_MACHINE("nt98561", nt98561, nt98561_soc)
DEFINE_NVT_MACHINE("nt98523", nt98523, nt98523_soc)
DEFINE_NVT_MACHINE("nt98529", nt98529, nt98529_soc)

/*
 * NT98532 / NT98533 (NS02501): NA51089's peripherals at NA51089's
 * addresses (U-Boot arch-nvt-ns02501_a32/IOAddress.h, the SDK's
 * cfg_53x board trees, NT98533_DataSheet_Register_Map.pdf), around two
 * Cortex-A32s -- ARMv8 cores running a 32-bit ARMv7 kernel -- with a
 * GIC-400 at 0xFFF01000 and the ARMv8 generic timer, at 12 MHz per the
 * tree.  QEMU has no A32; a Cortex-A7 runs the same code.
 *
 * Source: the ns02501 Linux SDK (U-Boot 2021.10, kernel 5.10.168) and
 * the ns02501 loader source.  The interrupts that moved from NA51089:
 * WDT 75, RTC 76, I2C 1 SPI 10, I2C 3 at 0xF0360000 SPI 42 (I2C 2 41).
 */
#define NS02501_GICD_BASE       0xFFF01000
#define NS02501_GICC_BASE       0xFFF02000
#define NS02501_CC_BASE         0xF0090000
#define NS02501_CPU_RESET       0xFFE40000
#define NS02501_GZIP_BASE       0xF0100000
#define NS02501_CC_SIZE         0x300
#define NS02501_CC_RELEASE      0x100   /* CC_CPU2_CPU1_CMDBUF_REG1 */
#define NS02501_CNTFRQ          12000000
#define NS02501_NUM_SPI         128
#define NS02501_IRQ_WDT         75
#define NS02501_IRQ_RTC         76
#define NS02501_IRQ_CC          59

/* The HR timers, nvt-peri.dtsi's nvt_hr_hwtimer 0..7 and the clocksource */
static const struct {
    hwaddr base;
    int irq;
} ns02501_hrtimers[] = {
    { 0xF00C0000, 61 }, { 0xF0150000, 62 }, { 0xF0160000, 63 },
    { 0xF0170000, 64 }, { 0xF0180000, 65 }, { 0xF0190000, 66 },
    { 0xF01A0000, 67 }, { 0xF01C0000, 68 }, { 0xF01E0000, 72 },
};

static const struct {
    hwaddr base;
    int irq;
} ns02501_i2c[] = {
    { 0xF0220000, 10 },
    { 0xF0350000, 41 },
    { 0xF0360000, 42 },
};

/*
 * The core communicator's command buffers.  The second core waits in the
 * loader until CMDBUF_REG1 (+0x100) is non-zero and jumps there, in
 * secure SVC (ns02501_ldr bl_lowlevel_core2_jump()); the kernel's
 * cpu-release-addr is that register (mach-nvt-ivot/platsmp.c).  Here the
 * second core stays off until the write, which starts it at the address.
 */
typedef struct {
    uint32_t regs[NS02501_CC_SIZE / 4];
} NvtCCState;

static uint64_t ns02501_cc_read(void *opaque, hwaddr addr, unsigned size)
{
    NvtCCState *s = opaque;

    return s->regs[addr / 4];
}

static void ns02501_cc_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    NvtCCState *s = opaque;
    CPUState *cs = qemu_get_cpu(1);

    s->regs[addr / 4] = val;
    if (addr == NS02501_CC_RELEASE && val && cs &&
        ARM_CPU(cs)->power_state == PSCI_OFF) {
        arm_set_cpu_on(ARM_CPU(cs)->mp_affinity, (uint32_t)val, 0, 3, false);
    }
}

static const MemoryRegionOps ns02501_cc_ops = {
    .read = ns02501_cc_read,
    .write = ns02501_cc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * 256 MB in the package on the EVBs (nvt_memory_cfg/dram); the SDK's
 * tables give Linux the first 36 MB and the HDAL the rest.
 */
static const NvtSoCConfig nt98533_soc = {
    .desc               = "Novatek NT98533 (2x Cortex-A32, NS02501, 256 MB)",
    .model              = "Novatek NS02501",
    .chip_id            = 0x5521,
    .ram_size_default   = 256 * MiB,
    .linux_mem          = 0x02400000,
};

/* NT98532: the same die and package DDR (cfg_532_*_BD_EVB: Linux 36 MB) */
static const NvtSoCConfig nt98532_soc = {
    .desc               = "Novatek NT98532 (2x Cortex-A32, NS02501, 256 MB)",
    .model              = "Novatek NS02501",
    .chip_id            = 0x5521,
    .ram_size_default   = 256 * MiB,
    .linux_mem          = 0x02400000,
};

static const ARMCPRegInfo ns02501_cpuectlr = {
    .name = "CPUECTLR", .cp = 15, .opc1 = 1, .crm = 15,
    .type = ARM_CP_64BIT | ARM_CP_CONST | ARM_CP_NO_RAW,
    .access = PL1_RW, .resetvalue = 0,
};

static void novatek_ns02501_init(MachineState *machine,
                                 const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    unsigned int smp_cpus = machine->smp.cpus;
    qemu_irq pic[NS02501_NUM_SPI];
    DeviceState *gic, *dev, *cg, *gpio;
    SysBusDevice *gicbus;
    MemoryRegion *cc = g_new(MemoryRegion, 1);
    ARMCPU *cpu[2];
    int n, i;

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        /*
         * The loader and U-Boot run in secure SVC, and so does the kernel
         * they start.  U-Boot's lowlevel_init goes to monitor mode, sets
         * SCR.NS and clears CNTVOFF, which QEMU allows only with EL2
         * there: the A7's virtualization extensions stay.  The second core
         * is started by its release register, below.
         */
        object_property_set_bool(cpuobj, "has_el3", true, &error_fatal);
        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        object_property_set_uint(cpuobj, "cntfrq", NS02501_CNTFRQ,
                                 &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
        /*
         * The A32's CPUECTLR (MRRC p15, 1, c15), which the loader's start
         * code reads to set SMPEN: the A7 standing in here has none.
         */
        define_one_arm_cp_reg(cpu[n], &ns02501_cpuectlr);
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);
    create_unimplemented_device("nvt-periph", NVT_PERIPH_BASE,
                                NVT_PERIPH_SIZE);

    /*
     * GIC-400 ("arm,cortex-a7-gic"): everything runs secure, so its
     * security extensions would change nothing the software sees.
     */
    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(gic, "num-irq", NS02501_NUM_SPI + GIC_INTERNAL);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    sysbus_mmio_map(gicbus, 0, NS02501_GICD_BASE);
    sysbus_mmio_map(gicbus, 1, NS02501_GICC_BASE);

    for (n = 0; n < smp_cpus; n++) {
        DeviceState *cpudev = DEVICE(cpu[n]);
        int ppibase = NS02501_NUM_SPI + n * GIC_INTERNAL + GIC_NR_SGIS;
        /* nvt-basic.dtsi: 13 secure, 14 non-secure, 11 virtual, 10 hyp */
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
    for (n = 0; n < NS02501_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    memory_region_init_io(cc, OBJECT(machine), &ns02501_cc_ops,
                          g_new0(NvtCCState, 1), "nvt-cc", NS02501_CC_SIZE);
    memory_region_add_subregion_overlap(sysmem, NS02501_CC_BASE, cc, 1);
    {
        /*
         * The CPU block's core reset (0xFFE40000): the loader's
         * cc_startCore(1) sets bits 2 / 10 to take core 1 out of reset,
         * at its reset vector -- the ROM, which re-enters the loader, where
         * core 1 configures DDR and parks on CMDBUF_REG1.  NA51055's model.
         */
        MemoryRegion *rst = g_new(MemoryRegion, 1);

        memory_region_init_io(rst, OBJECT(machine), &na51055_cpu_reset_ops,
                              g_new0(uint32_t, 1), "ns02501.cpu-reset", 4);
        memory_region_add_subregion_overlap(sysmem, NS02501_CPU_RESET, rst,
                                            1);
    }

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
    qdev_prop_set_uint32(dev, "strap", nms->boot_strap);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_TOP_BASE, 1);

    cg = qdev_new("nvt-cg");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cg), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(cg), 0, NVT_CG_BASE, 1);

    dev = qdev_new("nvt-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_WDT_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NS02501_IRQ_WDT]);

    dev = qdev_new("nvt-rtc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_RTC_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NS02501_IRQ_RTC]);

    /* The GZIP engine the loader unpacks its main part with */
    sysbus_create_simple("nvt-gzip", NS02501_GZIP_BASE, NULL);

    for (n = 0; n < ARRAY_SIZE(ns02501_hrtimers); n++) {
        sysbus_create_simple("nvt-hrtimer", ns02501_hrtimers[n].base,
                             pic[ns02501_hrtimers[n].irq]);
    }

    /* The ADC (NA51089's block, its thermal register unused here) and the
     * stand-alone thermal sensor nvtivot_thermal.c reads: 0.5 C a count
     * (TEMP_MULTI 500), the tree's default_trim 191. */
    nvt_adc_create(NS02501_ADC_BASE, pic[NS02501_IRQ_ADC], c->chip_id);
    dev = qdev_new("nvt-thermal");
    object_property_add_child(OBJECT(machine), "thermal", OBJECT(dev));
    qdev_prop_set_uint32(dev, "trim", 191);
    qdev_prop_set_uint32(dev, "step", 500);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NS02501_THERMAL, 1);

    gpio = qdev_new("nvt-gpio");
    object_property_add_child(OBJECT(machine), "nvt-gpio", OBJECT(gpio));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(gpio), 0, NVT_GPIO_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(gpio), 0, pic[NVT_IRQ_GPIO]);

    sysbus_create_simple("nvt-otp", NVT_OTP_BASE, NULL);
    /* 3 MHz over CLKDIV0 / CLKDIV1 + 1, both 1 MHz at reset: as NA51089's */
    sysbus_create_simple("nvt-timer", NVT_TIMER_BASE, pic[NVT_IRQ_TIMER]);

    for (n = 0; n < ARRAY_SIZE(nvt_crypto_blocks); n++) {
        sysbus_create_simple(nvt_crypto_blocks[n].type,
                             nvt_crypto_blocks[n].base,
                             pic[nvt_crypto_blocks[n].irq]);
    }

    dev = qdev_new("nvt-pwm");
    object_property_add_child(OBJECT(machine), "nvt-pwm", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NVT_PWM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_PWM]);

    for (n = 0; n < ARRAY_SIZE(ns02501_i2c); n++) {
        sysbus_create_simple("nvt-i2c", ns02501_i2c[n].base,
                             pic[ns02501_i2c[n].irq]);
    }

    /* USB 2.0: the tree puts the controller at 0xFF600000 itself. */
    dev = qdev_new("nvt-usb");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, NVT_USB_ALIAS, 1);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_USB_ALIAS, 2);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_USB]);

    dev = qdev_new("nvt-eqos");
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_ETH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_ETH]);

    /* SDIO1 / 2 / 3 as on NA51089, reset by the same CG bits. */
    for (n = 0; n < ARRAY_SIZE(nvt_sdio); n++) {
        DriveInfo *di = drive_get(IF_SD, 0, n);

        dev = qdev_new("nvt-sdio");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, nvt_sdio[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[nvt_sdio[n].irq]);
        qdev_connect_gpio_out_named(cg, "sys-reset", nvt_sdio[n].reset,
                                    qdev_get_gpio_in_named(dev, "reset", 0));
        if (di) {
            DeviceState *card = qdev_new(nvt_sdio[n].emmc ? TYPE_EMMC
                                                           : TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    dev = qdev_new(TYPE_NVT_FLASH);
    if (nms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", nms->flash_file);
    }
    qdev_prop_set_uint32(dev, "jedec", nms->flash_jedec);
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    nms->flash = NVT_FLASH(dev);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_FLASH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_FLASH]);

    /* 16550s, reg-shift 2, all at 48 MHz (nvt-peri.dtsi) */
    for (n = 0; n < ARRAY_SIZE(nvt_uarts); n++) {
        serial_mm_init(sysmem, nvt_uarts[n].base, 2, pic[nvt_uarts[n].irq],
                       48000000 / 16, serial_hd(n), DEVICE_LITTLE_ENDIAN);
    }

    for (n = 0; n < ARRAY_SIZE(nvt_regbanks); n++) {
        /* the DDR controller / PHY: storage with the loader's done bits */
        if (!strcmp(nvt_regbanks[n].name, "nvt-ddrc")) {
            dev = qdev_new("nvt-ddrc");
            qdev_prop_set_bit(dev, "ns02501", true);
        } else {
            dev = qdev_new("hisi-regbank");
            qdev_prop_set_uint32(dev, "size", nvt_regbanks[n].size);
            qdev_prop_set_string(dev, "name", nvt_regbanks[n].name);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                nvt_regbanks[n].base, 1);
    }

    if (machine->kernel_filename && !machine->dtb) {
        error_report("novatek: %s needs a board device tree (-dtb), such "
                     "as the SDK's cfg_53x nvt-evb.dts built",
                     MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    binfo.ram_size = MIN(c->linux_mem, machine->ram_size);
    binfo.loader_start = 0;
    binfo.secure_boot = true;
    binfo.write_secondary_boot = nvt_write_secondary;
    binfo.secondary_cpu_reset_hook = nvt_reset_secondary;
    arm_load_kernel(cpu[0], machine, &binfo);

    if (!machine->kernel_filename && nms->flash_file) {
        nvt_rom_set_loader_file(nms->boot_file);
        if (nms->real_loader &&
            nvt_flash_has_loader(nms->flash, NVT_SRAM_BASE, NVT_SRAM_SIZE)) {
            nvt_rom_boot(nms->flash, &nvt_rom_cfg[2]);
        } else {
            nvt_flash_boot(nms->flash, machine->ram_size, false);
        }
    }
}

#define DEFINE_NVT_NS_MACHINE(namestr, tag, config)                  \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        novatek_ns02501_init(machine, &config);                      \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");       \
        mc->max_cpus = 2;                                            \
        mc->default_cpus = 2;                                        \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, NVT_MACHINE, NvtMachineState,   \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_NVT_NS_MACHINE("nt98533", nt98533, nt98533_soc)
DEFINE_NVT_NS_MACHINE("nt98532", nt98532, nt98532_soc)

/*
 * NT98331 / NT98332 / NT98631 / NT98632 (NA51103): two Cortex-A53s running
 * an aarch64 kernel, a GIC-400 and the ARMv8 generic timer, with the
 * Novatek blocks moved above 4 GiB at 0x2_F0000000.  Most of them keep
 * NA51089's register layout at new offsets -- the same offsets as
 * NA51090's (U-Boot arch-nvt-na51090_a64/IOAddress.h) -- so the models
 * above carry over.  The map is the shipped device tree's: a Xiongmai
 * NBD80N36RA-KL NT98332G NOR dump, kernel 4.19.148, U-Boot 2019.04.
 *
 * There is no machine-built device tree yet: boot with the firmware's own
 * (-dtb), which the vendor drivers need all of anyway.
 */
#define NA51103_IO(off)         (0x2F0000000ULL + (off))
#define NA51103_PERIPH_SIZE     0x10000000
#define NA51103_TOP_BASE        NA51103_IO(0x00010000)
#define NA51103_CG_BASE         NA51103_IO(0x00020000)
#define NA51103_PAD_BASE        NA51103_IO(0x00030000)
#define NA51103_GPIO_BASE       NA51103_IO(0x00040000)
#define NA51103_WDT_BASE        NA51103_IO(0x00060000)
#define NA51103_TIMER_BASE      NA51103_IO(0x00100000)
#define NA51103_FLASH_BASE      NA51103_IO(0x00180000)
#define NA51103_ETH_BASE        NA51103_IO(0x00440000)
#define NA51103_RTC_BASE        NA51103_IO(0x00620000)
#define NA51103_OTP_BASE        NA51103_IO(0x00630000)
#define NA51103_GICD_BASE       NA51103_IO(0x0FF01000)
#define NA51103_GICC_BASE       NA51103_IO(0x0FF02000)

#define NA51103_IRQ_ETH         12
#define NA51103_IRQ_GPIO        24
#define NA51103_IRQ_FLASH       29
#define NA51103_IRQ_RTC         56
#define NA51103_IRQ_WDT         57
#define NA51103_NUM_SPI         256     /* the PMU is on SPI 212/213 */

/* The DT's arm,armv8-timer clock-frequency: a 12 MHz counter. */
#define NA51103_CNTFRQ          12000000

/*
 * What moves between NA51103 (nt98332) and NA51090 (nt98633): the rest of
 * the two dies' maps is the same (TOP, CG, PAD, GPIO, the timer, the flash
 * controller, I2C, PWM, the GIC, the display and media blocks).
 */
typedef struct {
    hwaddr base;
    int irq;
} NvtA64Blk;

typedef struct {
    const char *type;
    hwaddr base;
    int irq;
} NvtA64Crypto;

typedef struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} NvtA64Bank;

typedef struct {
    hwaddr ahci, glue, phy;
    int irq;
} NvtA64Sata;

/* An HDAL engine as nvt-engine: its kdrv module's view of it */
typedef struct {
    const char *name;
    hwaddr base;
    uint32_t size;
    int irq;
    uint32_t fire_reg, fire_mask, sts_reg, en_reg, done_bits;
    uint32_t ll_reg, ll_fire_mask;
    bool scaler;
    uint32_t ll_hi_reg, ll_off_mask;
    uint32_t ll_hi_shift, ll_format, ll_active_reg;
    bool sts_read_clear;
} NvtA64Engine;

struct NvtA64Map {
    unsigned a53s;
    /* the default -smp has the boot core, so the real loader runs */
    bool loader_default;
    hwaddr wdt, rtc, otp, thermal;
    int irq_wdt, irq_rtc;
    const NvtA64Crypto *crypto;         /* crypto, hash, RSA */
    /* each list's count follows it, for NVT_A64_LIST() */
    const NvtA64Blk *eth;
    unsigned n_eth;
    const NvtA64Blk *uarts;
    unsigned n_uarts;
    const NvtA64Blk *usb;
    unsigned n_usb;
    const NvtA64Blk *hrtimers;
    unsigned n_hrtimers;
    const NvtA64Sata *sata;
    unsigned n_sata;
    const hwaddr *testers;
    unsigned n_testers;
    const hwaddr *ddrc;                 /* the controllers, PHY at +0x8000 */
    unsigned n_ddrc;
    const hwaddr *sdio;                 /* SD hosts (nvt-sdio), polled */
    unsigned n_sdio;
    hwaddr cc;                          /* the core communicator, or 0 */
    uint8_t cg_layout;                  /* nvt-cg's PLL table */
    NvtA64Blk xhci;                     /* the USB 3 host, or base 0 */
    NvtA64Blk venc2;                    /* a second H.26x encoder, or 0 */
    bool media_na51090;                 /* the decoder, VPE: NA51090 layouts */
    const NvtA64Bank *banks;
    unsigned n_banks;
    const NvtA64Engine *engines;        /* the die's own, past the shared */
    unsigned n_engines;
};

/* a list and its count, as two consecutive initializers */
#define NVT_A64_LIST(a)     (a), ARRAY_SIZE(a)

/* 16550s with reg-shift 2; 48 MHz (the DT's clock-frequency). */
static const NvtA64Blk na51103_uarts[] = {
    { NA51103_IO(0x00280000), 43 },
    { NA51103_IO(0x00290000), 44 },
    { NA51103_IO(0x002A0000), 45 },
    { NA51103_IO(0x002B0000), 38 },
};

/* The audio interfaces, in the "nvt,nvt_dai" node's reg / interrupts order. */
static const struct {
    hwaddr base;
    int irq;
} na51103_dais[] = {
    { NA51103_IO(0x00850000), 15 },
    { NA51103_IO(0x00860000), 34 },
    { NA51103_IO(0x00220000), 35 },
    { NA51103_IO(0x00230000), 36 },
    { NA51103_IO(0x00870000), 37 },
};

/*
 * The HDAL media engines, as nvt-engine: fire a job, report it done.  Each
 * line is the engine's kdrv module's view of it (fire register and bits,
 * status and enable registers, the status bits its ISR takes as done).
 */
static const NvtA64Engine na51103_engines[] = {
    /* kdrv_vpe: CTRL bit 31 fires the link list at 0x300, bit 30 a single
     * job; 0x400/0x404 status/enable, bit 0 frame end, bit 2 list end. */
    { "nvt-vpe", NA51103_IO(0x00410000), 0x1040, 3,
      0x000, 0xC0000000, 0x400, 0x404, 0x5, 0x300, 0x80000000, true, 0,
      0x1FFC },
    /* kdrv_osg (osg_drv_eng_ll_fire / _isr_all): writing the list's
     * address to 0x30 fires it, bits 35:32 at 0x340; 0x70 the status,
     * bit 0 done, written back to clear.  Its lists are the VPE's command
     * format, but the byte enables in hi[15:12] (osg_drv_ll_write_reg:
     * 0x8000F000 | offset). */
    { "nvt-osg", NA51103_IO(0x00300000), 0x1000, 10,
      0x030, 0xFFFFFFF8, 0x070, 0x000, 0x1, 0x030, 0xFFFFFFF8, false,
      0x340, 0xFFC },
    /* kdrv_dei (dei_eng_trig_ll_hw_reg / _isr_hw_reg): the list's address
     * at 0x180, bits 35:32 at 0x200, then CTRL bit 31 (bit 30 a single
     * job); 0x1C the status, bit 0 frame done, bit 2 list done, written
     * back to clear.  The OSG's command format.  No pixel work. */
    { "nvt-dei", NA51103_IO(0x00610000), 0x1000, 6,
      0x000, 0xC0000000, 0x01C, 0x000, 0x5, 0x180, 0x80000000, false,
      0x200, 0xFFC },
    /* kdrv_ssca (ssca_eng_trig_ll_hw_reg / _isr_hw_reg): the list's
     * address at 0xAC, bits 35:32 in 0xA8 [31:28], CTRL bit 8 fires it
     * (bit 1 a single job); 0x0C the status, bits 31 / 30 the list and
     * the frame done, written back.  Its writes: 0x8F000000 | offset. */
    { "nvt-ssca", NA51103_IO(0x00310000), 0x1000, 74,
      0x000, 0x102, 0x00C, 0x000, 0xC0000000, 0x0AC, 0x100, false,
      0x0A8, 0x1FFC, 28 },
    /* kdrv_age (age_eng_trig_ll_hw_reg / _isr_hw_reg): the list at 0x418,
     * bits 35:32 at 0x420, fired by 0x30101 to 0x41C (0x1000 before it);
     * its own list format (age_drv_ll_*); 0x410 set marks list mode, and
     * 0x404 bits 1:0 report it done -- the handler never clears them, so
     * they clear when read. */
    { "nvt-age", NA51103_IO(0x003A0000), 0x1000, 16,
      0x41C, 0x1, 0x404, 0x000, 0x3, 0x418, 0x1, false,
      0x420, 0x1FFC, 0, 1, 0x410, true },
};

/*
 * NA51090's own engines: the second deinterlacer (the tree's dei2), and
 * the VPE's second instance ("nvt,vpel"), which kdrv_vpe drives as the
 * VPE, its lists sharing out the preview's jobs; a list left unfinished
 * there holds its display frames and stalls the whole preview.
 */
static const NvtA64Engine na51090_engines[] = {
    { "nvt-vpel", NA51103_IO(0x00420000), 0x1040, 1,
      0x000, 0xC0000000, 0x400, 0x404, 0x5, 0x300, 0x80000000, true, 0,
      0x1FFC },
    { "nvt-dei2", NA51103_IO(0x00620000), 0x1000, 21,
      0x000, 0xC0000000, 0x01C, 0x000, 0x5, 0x180, 0x80000000, false,
      0x200, 0xFFC },
};

static const hwaddr na51103_testers[] = {
    NA51103_IO(0x000A0000), NA51103_IO(0x00110000), NA51103_IO(0x00510000),
};

static const NvtA64Bank na51103_regbanks[] = {
    { "nvt-ddrc",   NA51103_IO(0x00000000), 0x10000 },
    { "nvt-pad",    NA51103_PAD_BASE,       0x10000 },
    { "nvt-lcd200", NA51103_IO(0x00820000), 0x10000 },
    { "nvt-usb-phy", NA51103_IO(0x004E0000), 0x1000 },
    { "nvt-usb-phy2", NA51103_IO(0x004F0000), 0x1000 },
};

/*
 * The two SATA hosts ("novatech,nvt_sata100", the kernel's built-in
 * NVT_SATA100_AHCI): a standard one-port AHCI, with a Novatek glue block
 * and the PHY beside it (storage).  The disks are -drive if=ide,index=0/1.
 */
static const NvtA64Sata na51103_sata[] = {
    { NA51103_IO(0x00680000), NA51103_IO(0x006C0000), NA51103_IO(0x006E0000),
      13 },
    { NA51103_IO(0x00690000), NA51103_IO(0x006D0000), NA51103_IO(0x006F0000),
      14 },
};

/*
 * GHC on the Novatek AHCI.  The driver sets AE | IE (0x80000002) and then,
 * after a write to CAP, writes GHC = 0 (nvt_ahci_save_initial_config()):
 * on QEMU's AHCI that drops IE for good and no command ever completes
 * ("qc timeout (cmd 0xec)").  The board runs on it, so the bits do not
 * clear there; keep them set and pass the rest through.
 */
#define NA51103_AHCI_GHC        0x04
#define NA51103_AHCI_GHC_KEEP   0x80000002u

static uint64_t na51103_ghc_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = 0;

    memory_region_dispatch_read(opaque, NA51103_AHCI_GHC, &val, MO_32,
                                MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void na51103_ghc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    memory_region_dispatch_write(opaque, NA51103_AHCI_GHC,
                                 val | NA51103_AHCI_GHC_KEEP, MO_32,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps na51103_ghc_ops = {
    .read = na51103_ghc_read,
    .write = na51103_ghc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Blocks the NVR's tree has whose NA51089 models carry over: I2C 1..4
 * (nvt,nvt_i2c; the driver probes all four), the PWM, and the crypto,
 * hash and RSA engines, which the vendor modules drive.
 */
static const struct {
    hwaddr base;
    int irq;
} na51103_i2c[] = {
    { NA51103_IO(0x001C0000), 41 },
    { NA51103_IO(0x001D0000), 42 },
    { NA51103_IO(0x001E0000), 60 },
    { NA51103_IO(0x001F0000), 50 },
};

static const NvtA64Crypto na51103_crypto[] = {
    { "nvt-crypto", NA51103_IO(0x00640000), 79 },
    { "nvt-hash",   NA51103_IO(0x00650000), 46 },
    { "nvt-rsa",    NA51103_IO(0x00660000), 48 },
};

/*
 * The HR timers (nvt-hrtimer, CNT at 0x14): 0x140000 is the tree's
 * nvt_hwtimer 0 and its clocksource, 0x170000 nvt_hwtimer 1 and ptp_nvt.
 */
static const NvtA64Blk na51103_hrtimers[] = {
    { NA51103_IO(0x00140000), 83 },
    { NA51103_IO(0x00170000), 84 },
};

#define NA51103_PWM_BASE        NA51103_IO(0x00120000)
#define NA51103_IRQ_PWM         26
#define NA51103_THERMAL_BASE    NA51103_IO(0x00890000)

/* The two USB 2.0 hosts (nvt-usb: EHCI over the FOTG200 glue). */
static const NvtA64Blk na51103_usb[] = {
    { NA51103_IO(0x00480000), 27 },
    { NA51103_IO(0x00490000), 78 },
};

static const hwaddr na51103_ddrc[] = {
    NA51103_IO(0x000E0000), NA51103_IO(0x000F0000),
};

static const NvtA64Blk na51103_eth[] = {
    { NA51103_ETH_BASE, NA51103_IRQ_ETH },
};

static const NvtA64Map na51103_map = {
    .a53s = 2,
    .loader_default = true,
    .wdt = NA51103_WDT_BASE, .irq_wdt = NA51103_IRQ_WDT,
    .rtc = NA51103_RTC_BASE, .irq_rtc = NA51103_IRQ_RTC,
    .otp = NA51103_OTP_BASE, .thermal = NA51103_THERMAL_BASE,
    .crypto = na51103_crypto,
    .eth = NVT_A64_LIST(na51103_eth), .uarts = NVT_A64_LIST(na51103_uarts),
    .usb = NVT_A64_LIST(na51103_usb),
    .hrtimers = NVT_A64_LIST(na51103_hrtimers),
    .sata = NVT_A64_LIST(na51103_sata),
    .testers = NVT_A64_LIST(na51103_testers),
    .ddrc = NVT_A64_LIST(na51103_ddrc),
    .banks = NVT_A64_LIST(na51103_regbanks),
};

/*
 * 1 GB DDR: U-Boot reports it, the tree's nvt_memory_cfg/dram says
 * 0x40000000 and the HDAL pool runs to its end (the boot logo's
 * framebuffer is at 0x3DD00000).  The tree gives Linux the first 150 MB.
 */
static const NvtSoCConfig nt98332_soc = {
    .desc               = "Novatek NT98332 (2x Cortex-A53, NA51103, 1 GB)",
    .model              = "Novatek NA51103",
    .chip_id            = 0x8B20,   /* U-Boot arch-nvt-na51103: CHIP_NA51103 */
    .ram_size_default   = 1 * GiB,
    .linux_mem          = 0x09600000,
    .a64                = &na51103_map,
};

/*
 * NT98633 (NA51090): four A53s and NA51103's map with these blocks moved
 * (the tree of an NT98633 board's SPI NAND, "novatek,na51090"): the WDT to
 * 0x110000 and the OTP to 0xA90000 -- both of which the loader uses --,
 * the RTC to 0xA50000, the thermal sensor to 0x510000, the crypto engines
 * to 0x950000 / 0x960000 / 0xA80000, two Ethernet MACs at 0x210000 /
 * 0x200000, USB 2.0 at 0x380000 / 0x390000, three SATA hosts and a fifth
 * UART.  Its DRAM testers are the arbiters' at 0x80000 and 0xA0000.
 *
 * By default (-smp 5) its loader (FW98633A) runs on the boot core, as on
 * NA51103: DDR training against nvt-ddrc's NA51090 models, then BL31, then
 * U-Boot, whose trampoline's mailboxes are in the core communicator below.
 * -smp 4 or loader=off: the stand-in.
 */
static const NvtA64Blk na51090_uarts[] = {
    { NA51103_IO(0x00280000), 43 },
    { NA51103_IO(0x00290000), 44 },
    { NA51103_IO(0x002A0000), 45 },
    { NA51103_IO(0x002B0000), 38 },
    { NA51103_IO(0x002C0000), 47 },
};

static const NvtA64Blk na51090_eth[] = {
    { NA51103_IO(0x00210000), 11 },
    { NA51103_IO(0x00200000), 12 },
};

static const NvtA64Blk na51090_usb[] = {
    { NA51103_IO(0x00380000), 27 },
    { NA51103_IO(0x00390000), 78 },
};

static const NvtA64Crypto na51090_crypto[] = {
    { "nvt-crypto", NA51103_IO(0x00950000), 79 },
    { "nvt-hash",   NA51103_IO(0x00960000), 46 },
    { "nvt-rsa",    NA51103_IO(0x00A80000), 48 },
};

static const NvtA64Sata na51090_sata[] = {
    { NA51103_IO(0x00B80000), NA51103_IO(0x00BC0000), NA51103_IO(0x00BE0000),
      13 },
    { NA51103_IO(0x00B90000), NA51103_IO(0x00BD0000), NA51103_IO(0x00BF0000),
      14 },
    { NA51103_IO(0x00480000), NA51103_IO(0x004E0000), NA51103_IO(0x008B0000),
      69 },
};

static const hwaddr na51090_testers[] = {
    NA51103_IO(0x00080000), NA51103_IO(0x000A0000),
};

/*
 * SDIO 1 / 2 (U-Boot's IOAddress.h, its EVB tree's mmc@2,f0190000; no
 * interrupt there): the loader looks for an update card on SDIO 1.
 */
static const hwaddr na51090_sdio[] = {
    NA51103_IO(0x00190000), NA51103_IO(0x001A0000),
};

/* the loader's PHY at 0xF06E8000 (page select 0x3F8), and its twin */
static const hwaddr na51090_ddrc[] = {
    NA51103_IO(0x006E0000), NA51103_IO(0x006F0000),
};

static const NvtA64Bank na51090_regbanks[] = {
    { "nvt-ddrc",     NA51103_IO(0x00000000), 0x10000 },
    { "nvt-pad",      NA51103_PAD_BASE,       0x10000 },
    { "nvt-lcd200",   NA51103_IO(0x00820000), 0x10000 },
    { "nvt-usb-phy",  NA51103_IO(0x003E1000), 0x1000 },
    { "nvt-usb-phy2", NA51103_IO(0x003F1000), 0x1000 },
    { "nvt-usb3-phy", NA51103_IO(0x005F0000), 0x4000 },
};

/*
 * NA51090's core communicator ("kdrv_rpc", 0x2_F0160000): the loader's
 * AArch64 trampoline keeps its mailboxes there -- BL31's entry at 0x100
 * (read as one 64-bit word by core 0) and 0x10C (the others'), the
 * cores' arrival flags at 0x114..0x11C, a state word at 0x108.  Storage.
 */
#define NA51090_CC_SIZE     0x200

static uint64_t na51090_cc_read(void *opaque, hwaddr addr, unsigned size)
{
    return ((uint32_t *)opaque)[addr / 4];
}

static void na51090_cc_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    ((uint32_t *)opaque)[addr / 4] = val;
}

static const MemoryRegionOps na51090_cc_ops = {
    .read = na51090_cc_read,
    .write = na51090_cc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const NvtA64Map na51090_map = {
    .a53s = 4,
    .loader_default = true,
    .wdt = NA51103_IO(0x00110000), .irq_wdt = 57,
    .rtc = NA51103_IO(0x00A50000), .irq_rtc = 56,
    .otp = NA51103_IO(0x00A90000), .thermal = NA51103_IO(0x00510000),
    .crypto = na51090_crypto,
    .eth = NVT_A64_LIST(na51090_eth), .uarts = NVT_A64_LIST(na51090_uarts),
    .usb = NVT_A64_LIST(na51090_usb),
    .sata = NVT_A64_LIST(na51090_sata),
    .testers = NVT_A64_LIST(na51090_testers),
    .ddrc = NVT_A64_LIST(na51090_ddrc),
    .sdio = NVT_A64_LIST(na51090_sdio),
    .cc = NA51103_IO(0x00160000),
    .cg_layout = 4,                     /* NA51090's PLLs */
    /*
     * The USB 3 host ("nvt,nvt_usb3xhci", xhci-plat with Novatek's PHY
     * set-up): a standard xHCI, its PHYs (0x5F0000) storage.
     */
    .xhci = { NA51103_IO(0x00580000), 28 },
    .venc2 = { NA51103_IO(0x00A00000), 8 },     /* the tree's venc2 */
    .media_na51090 = true,
    .banks = NVT_A64_LIST(na51090_regbanks),
    .engines = NVT_A64_LIST(na51090_engines),
};

/*
 * 2 GB DDR (the tree's nvt_memory_cfg/dram); Linux gets the first
 * 0x57600000.
 */
static const NvtSoCConfig nt98633_soc = {
    .desc               = "Novatek NT98633 (4x Cortex-A53, NA51090, 2 GB)",
    .model              = "Novatek NA51090",
    .chip_id            = 0xBC21,   /* U-Boot hardware.h: CHIP_NA51090 */
    .ram_size_default   = 2 * GiB,
    .linux_mem          = 0x57600000,
    .a64                = &na51090_map,
};

/*
 * NA51103's boot: the mask ROM starts core 0 in AArch32 at EL3 and runs the
 * loader (FW98332A.BIN: a stub, then its LZ-packed main part at
 * 0xFE020000) from the SRAM; the loader brings DDR up, loads BL31 and
 * U-Boot, and warm-resets core 0 into AArch64 at BL31.
 *
 * QEMU cannot start an AArch64 CPU in AArch32, so the AArch32 part runs on
 * a boot core of its own -- a Cortex-A7 (the loader is ARMv7 code) in a
 * cluster of its own -- that sees the low 4 GiB, with the peripherals
 * (0x2_F0000000) at 0xF0000000.  The warm reset (RMR) stops it and starts
 * the A53s; nothing but memory carries over.
 */
#define NA51103_SRAM_A32        0xFE000000
#define NA51103_SRAM_SIZE       0x40000
#define NA51103_A32_IO          0xF0000000
/* the A53s the warm reset starts: the SoC's, set by novatek_a64_init() */
static unsigned nvt_a64_cores;

/*
 * The A53's 64-bit IMPLEMENTATION DEFINED registers the loader touches
 * (CPUECTLR's SMPEN around the cache set-up), which the Cortex-A7 lacks:
 * zero, writes ignored.
 */
static const ARMCPRegInfo na51103_a53_cp_reginfo[] = {
    { .name = "CPUACTLR", .cp = 15, .opc1 = 0, .crm = 15,
      .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_64BIT },
    { .name = "CPUECTLR", .cp = 15, .opc1 = 1, .crm = 15,
      .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_64BIT },
    { .name = "CPUMERRSR", .cp = 15, .opc1 = 2, .crm = 15,
      .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_64BIT },
    { .name = "L2MERRSR", .cp = 15, .opc1 = 3, .crm = 15,
      .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_64BIT },
};

/*
 * L2CTLR [25:24]: the cores the loader starts at the warm reset -- the
 * A53s, not QEMU's count of every CPU in the machine.
 */
static const ARMCPRegInfo na51103_l2ctlr_reginfo = {
    .name = "L2CTLR", .cp = 15, .opc1 = 1, .crn = 9, .crm = 0, .opc2 = 2,
    .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_OVERRIDE,
};

/*
 * RMR: the loader's warm reset (AA64 | RR) ends the AArch32 part.  The
 * boot core stops, and the A53s come out of reset in AArch64 at EL3 at
 * RVBAR 0, where the loader has put the trampoline that takes each of
 * them to BL31.
 */
static void na51103_rmr_write(CPUARMState *env, const ARMCPRegInfo *ri,
                              uint64_t value)
{
    ARMCPU *cpu = env_archcpu(env);
    int n;

    if (!(value & 2)) {
        return;
    }
    for (n = 0; n < nvt_a64_cores; n++) {
        arm_set_cpu_on(n, 0, 0, 3, true);
    }
    arm_set_cpu_off(cpu->mp_affinity);
}

static const ARMCPRegInfo na51103_rmr_reginfo = {
    .name = "RMR", .cp = 15, .opc1 = 0, .crn = 12, .crm = 0, .opc2 = 2,
    .access = PL3_RW, .type = ARM_CP_IO | ARM_CP_NO_RAW | ARM_CP_OVERRIDE,
    .readfn = arm_cp_read_zero, .writefn = na51103_rmr_write,
};

static void na51103_boot_core(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *view = g_new(MemoryRegion, 1);
    MemoryRegion *low = g_new(MemoryRegion, 1);
    MemoryRegion *io = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    Object *cluster = object_new(TYPE_CPU_CLUSTER);
    Object *cpuobj;
    DeviceState *rom;

    memory_region_init(view, OBJECT(machine), "na51103.a32", 4 * GiB);
    memory_region_init_alias(low, OBJECT(machine), "na51103.a32-low", sysmem,
                             0, 4 * GiB);
    memory_region_add_subregion(view, 0, low);
    /*
     * The peripherals at 0xF0000000 too, as the loader's AArch64
     * trampoline uses them (the UART, the cores' mailboxes at 0xF0011200);
     * after it, BL31 and the rest use 0x2_F0000000.
     */
    memory_region_init_alias(io, OBJECT(machine), "na51103.a32-io", sysmem,
                             NA51103_IO(0), NA51103_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, NA51103_A32_IO, io);

    memory_region_init_ram(sram, NULL, "nvt.sram", NA51103_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem,
                                NA51103_IO(NA51103_SRAM_A32 - NA51103_A32_IO),
                                sram);

    object_property_add_child(OBJECT(machine), "boot-cluster", cluster);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 1);
    cpuobj = object_new(ARM_CPU_TYPE_NAME("cortex-a7"));
    object_property_add_child(cluster, "boot-cpu", cpuobj);
    /* apart from the A53s', which PSCI looks up */
    object_property_set_int(cpuobj, "mp-affinity", 0x100, &error_fatal);
    object_property_set_uint(cpuobj, "cntfrq", NA51103_CNTFRQ, &error_fatal);
    object_property_set_link(cpuobj, "memory", OBJECT(view), &error_fatal);
    define_arm_cp_regs(ARM_CPU(cpuobj), na51103_a53_cp_reginfo);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    {
        ARMCPRegInfo l2ctlr = na51103_l2ctlr_reginfo;

        l2ctlr.resetvalue = (nvt_a64_cores - 1) << 24;
        define_one_arm_cp_reg(ARM_CPU(cpuobj), &l2ctlr);
    }
    define_one_arm_cp_reg(ARM_CPU(cpuobj), &na51103_rmr_reginfo);
    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    rom = qdev_new(TYPE_NVT_BOOTROM);
    object_property_add_child(OBJECT(machine), "bootrom", OBJECT(rom));
    object_property_set_link(OBJECT(rom), "flash",
                             OBJECT(NVT_MACHINE(machine)->flash),
                             &error_fatal);
    object_property_set_link(OBJECT(rom), "memory", OBJECT(view),
                             &error_fatal);
    qdev_prop_set_uint64(rom, "sram-base", NA51103_SRAM_A32);
    qdev_prop_set_uint64(rom, "sram-size", NA51103_SRAM_SIZE);
    qdev_prop_set_uint64(rom, "sp", 0xFE038FFC);
    qdev_prop_set_uint64(rom, "uart", 0xF0280000);
    qdev_prop_set_uint8(rom, "layout", 3);
    qdev_prop_set_uint64(rom, "rom-base", NA51103_A32_IO);
    if (nvt_rom_loader_file) {
        qdev_prop_set_string(rom, "loader-file", nvt_rom_loader_file);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(rom), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(rom), 0,
                            NA51103_IO(object_property_get_uint(OBJECT(rom),
                                       "svc-base", &error_fatal) -
                                       NA51103_A32_IO), 10);
    /* the ROM at 0xF0000000 (0x2_F0000000), 32 KiB */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(rom), 1, NA51103_IO(0), 10);
}

static void novatek_a64_init(MachineState *machine, const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    const NvtA64Map *m = c->a64;
    /* the A53s; one more CPU slot is the real loader's boot core */
    unsigned int smp_cpus = MIN(machine->smp.cpus, m->a53s);
    qemu_irq pic[NA51103_NUM_SPI];
    DeviceState *gic, *dev;
    SysBusDevice *gicbus;
    ARMCPU *cpu[4];
    Object *clusters[1];
    bool rom;
    int n, i;

    nvt_a64_cores = m->a53s;

    dev = qdev_new(TYPE_NVT_FLASH);
    if (nms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", nms->flash_file);
    }
    qdev_prop_set_uint32(dev, "jedec", nms->flash_jedec);
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    nms->flash = NVT_FLASH(dev);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_FLASH_BASE, 1);

    /*
     * The real loader (and its BL31) when the flash has one and there is
     * a CPU slot for the boot core; loader=off, -smp 2 or -kernel: the
     * stand-in.
     */
    nvt_rom_set_loader_file(nms->boot_file);
    rom = nms->real_loader && nms->flash_file &&
          !machine->kernel_filename && machine->smp.cpus > smp_cpus &&
          nvt_flash_has_loader(nms->flash, NA51103_SRAM_A32,
                               NA51103_SRAM_SIZE);
    if (rom) {
        clusters[0] = object_new(TYPE_CPU_CLUSTER);
        object_property_add_child(OBJECT(machine), "a53-cluster",
                                  clusters[0]);
        qdev_prop_set_uint32(DEVICE(clusters[0]), "cluster-id", 0);
    }

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        if (n > 0 || rom) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        if (rom) {
            /*
             * The A53s come up in AArch64 at EL3 when the loader warm-resets
             * into its BL31, which answers PSCI and enters U-Boot at EL2.
             */
            object_property_add_child(clusters[0], "cpu[*]", cpuobj);
        } else {
            /*
             * The kernel is entered at EL1, with QEMU answering PSCI over
             * SMC in place of the firmware's BL31: the tree's psci node
             * says method = "smc", and the secondary comes up by CPU_ON.
             */
            object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
            object_property_set_bool(cpuobj, "has_el2", false, &error_fatal);
            object_property_set_int(cpuobj, "psci-conduit",
                                    QEMU_PSCI_CONDUIT_SMC, &error_fatal);
        }
        object_property_set_uint(cpuobj, "cntfrq", NA51103_CNTFRQ,
                                 &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }
    if (rom) {
        qdev_realize(DEVICE(clusters[0]), NULL, &error_fatal);
        na51103_boot_core(machine);
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);
    create_unimplemented_device("nvt-periph", NA51103_IO(0),
                                NA51103_PERIPH_SIZE);

    /*
     * GIC-400 ("arm,cortex-a7-gic" in the tree), distributor and CPU
     * interface only: nothing here uses its virtualization frames.  It has
     * the security extensions, and everything after BL31 runs non-secure:
     * U-Boot's own GIC setup (every IGROUPR to group 1) is then ignored, as
     * on silicon.  Without them it would land, and Linux, which enables
     * only its group, would never see CPU 0's IPIs or timer.  BL31's part
     * -- every interrupt in group 1 -- is the GIC's non-secure reset,
     * which a -kernel boot asks for through arm_load_kernel() and a flash
     * boot below.
     */
    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(gic, "num-irq", NA51103_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    if (!machine->kernel_filename && !rom) {
        ARM_GIC_COMMON(gic)->irq_reset_nonsecure = true;
    }
    sysbus_mmio_map(gicbus, 0, NA51103_GICD_BASE);
    sysbus_mmio_map(gicbus, 1, NA51103_GICC_BASE);

    for (n = 0; n < smp_cpus; n++) {
        DeviceState *cpudev = DEVICE(cpu[n]);
        int ppibase = NA51103_NUM_SPI + n * GIC_INTERNAL + GIC_NR_SGIS;
        /* The tree's timer PPIs: 13 secure, 14 non-secure, 11 virtual,
         * 10 hypervisor -- the usual ones. */
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
    for (n = 0; n < NA51103_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
    qdev_prop_set_uint32(dev, "strap", nms->boot_strap);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_TOP_BASE, 1);

    dev = qdev_new("nvt-cg");
    qdev_prop_set_uint32(dev, "sleep-reg", 0xB0);
    qdev_prop_set_uint8(dev, "layout", m->cg_layout);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_CG_BASE, 1);

    dev = qdev_new("nvt-gpio");
    object_property_add_child(OBJECT(machine), "nvt-gpio", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_GPIO_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_GPIO]);

    dev = qdev_new("nvt-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->wdt, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->irq_wdt]);

    dev = qdev_new("nvt-rtc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->rtc, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->irq_rtc]);

    sysbus_create_simple("nvt-otp", m->otp, NULL);
    sysbus_create_simple("nvt-timer", NA51103_TIMER_BASE, pic[0]);

    /* nvtivot_thermal.c: TEMP_MULTI 1319, the tree's default_trim 231 */
    dev = qdev_new("nvt-thermal");
    object_property_add_child(OBJECT(machine), "thermal", OBJECT(dev));
    qdev_prop_set_uint32(dev, "trim", 231);
    qdev_prop_set_uint32(dev, "step", 1319);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->thermal, 1);

    dev = qdev_new("nvt-pwm");
    object_property_add_child(OBJECT(machine), "nvt-pwm", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_PWM_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_PWM]);

    for (n = 0; n < m->n_hrtimers; n++) {
        dev = qdev_new("nvt-hrtimer");
        qdev_prop_set_uint32(dev, "count-at", 0x14);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                m->hrtimers[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->hrtimers[n].irq]);
    }
    for (n = 0; n < ARRAY_SIZE(na51103_i2c); n++) {
        sysbus_create_simple("nvt-i2c", na51103_i2c[n].base,
                             pic[na51103_i2c[n].irq]);
    }
    for (n = 0; n < 3; n++) {
        sysbus_create_simple(m->crypto[n].type, m->crypto[n].base,
                             pic[m->crypto[n].irq]);
    }

    /*
     * The two DDR controllers, each with its paged PHY at +0x8000 (the
     * loader's _phy_write bases 0xF00E8000 / 0xF00F8000), which the real
     * loader trains; NA51068's PHY.
     */
    for (n = 0; n < m->n_ddrc; n++) {
        dev = qdev_new("nvt-ddrc");
        qdev_prop_set_bit(dev, "phy-pages", true);
        qdev_prop_set_uint32(dev, "phy-offset", 0x8000);
        qdev_prop_set_bit(dev, "qsen-match", true);
        qdev_prop_set_bit(dev, "na51090", m->a53s == 4);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->ddrc[n], 1);
    }

    /*
     * The DRAM arbiters' built-in testers (the loader's "hvyload" check):
     * start bits at +0x805C, status at +0x80A0, one per arbiter.
     */
    for (n = 0; n < m->n_testers; n++) {
        dev = qdev_new("nvt-ddrc");
        qdev_prop_set_bit(dev, "tester", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->testers[n], 1);
    }

    if (m->cc) {
        MemoryRegion *cc = g_new(MemoryRegion, 1);

        memory_region_init_io(cc, OBJECT(machine), &na51090_cc_ops,
                              g_new0(uint32_t, NA51090_CC_SIZE / 4),
                              "nvt-cc", NA51090_CC_SIZE);
        memory_region_add_subregion_overlap(sysmem, m->cc, cc, 1);
    }

    for (n = 0; n < m->n_sdio; n++) {
        DriveInfo *di = drive_get(IF_SD, 0, n);

        dev = qdev_new("nvt-sdio");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->sdio[n], 1);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    /* -nic: the first MAC; a second -nic (NA51090) the second */
    for (n = 0; n < m->n_eth; n++) {
        dev = qdev_new("nvt-eqos");
        qemu_configure_nic_device(dev, n == 0, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->eth[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->eth[n].irq]);
    }

    sysbus_connect_irq(SYS_BUS_DEVICE(nms->flash), 0,
                       pic[NA51103_IRQ_FLASH]);

    for (n = 0; n < ARRAY_SIZE(na51103_engines) + m->n_engines; n++) {
        const NvtA64Engine *e = n < ARRAY_SIZE(na51103_engines)
                                ? &na51103_engines[n]
                                : &m->engines[n - ARRAY_SIZE(na51103_engines)];

        dev = qdev_new("nvt-engine");
        qdev_prop_set_string(dev, "name", e->name);
        qdev_prop_set_uint32(dev, "size", e->size);
        qdev_prop_set_uint32(dev, "fire-reg", e->fire_reg);
        qdev_prop_set_uint32(dev, "fire-mask", e->fire_mask);
        qdev_prop_set_uint32(dev, "sts-reg", e->sts_reg);
        qdev_prop_set_uint32(dev, "en-reg", e->en_reg);
        qdev_prop_set_uint32(dev, "done-bits", e->done_bits);
        qdev_prop_set_uint32(dev, "ll-reg", e->ll_reg);
        qdev_prop_set_uint32(dev, "ll-fire-mask", e->ll_fire_mask);
        qdev_prop_set_bit(dev, "scaler", e->scaler);
        qdev_prop_set_bit(dev, "na51090", e->scaler && m->media_na51090);
        qdev_prop_set_uint32(dev, "ll-hi-reg", e->ll_hi_reg);
        qdev_prop_set_uint32(dev, "ll-off-mask", e->ll_off_mask);
        qdev_prop_set_uint32(dev, "ll-hi-shift", e->ll_hi_shift);
        qdev_prop_set_uint32(dev, "ll-format", e->ll_format);
        qdev_prop_set_uint32(dev, "ll-active-reg", e->ll_active_reg);
        qdev_prop_set_bit(dev, "sts-read-clear", e->sts_read_clear);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, e->base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[e->irq]);
    }

    /*
     * Both dies' JPEG codec ("nvt,kdrv_jpg"): NA51089's register layout
     * (kdrv_jpg.ko's jpeg_set_*), plus address bits 35:32 at 0xC8 / 0xCC
     * that DRAM below 4 GiB leaves 0 -- NA51089's model.  The hardware
     * copy engine beside it.
     */
    dev = qdev_new("nvt-media");
    qdev_prop_set_string(dev, "engine", "jpg");
    qdev_prop_set_uint32(dev, "size", 0x3000);
    object_property_add_child(OBJECT(machine), "jpg", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_IO(0x00320000), 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[17]);
    sysbus_create_simple("nvt-hwcopy", NA51103_IO(0x00330000), pic[70]);

    /* The IR remote receiver (nvt_ir_fr.c): frames by qom-set
     * /machine/ir frame=<64-bit word> */
    dev = qdev_new("nvt-ir");
    object_property_add_child(OBJECT(machine), "ir", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_IO(0x00130000), 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[25]);

    /*
     * The H.26x encoder ("nvt,venc", kdrv_venc): nvt-media's "venc", a
     * later revision of NA51089's engine that runs link lists (nvt-media.c)
     * and codes with nvt-h26x.c.  NA51090 has a second ("nvt,venc2").
     */
    for (n = 0; n < 2; n++) {
        NvtA64Blk e = n ? m->venc2
                        : (NvtA64Blk){ NA51103_IO(0x00500000), 7 };

        if (!e.base) {
            continue;
        }
        dev = qdev_new("nvt-media");
        qdev_prop_set_string(dev, "engine", "venc");
        qdev_prop_set_uint32(dev, "size", 0x8000);
        object_property_add_child(OBJECT(machine), n ? "venc2" : "venc",
                                  OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, e.base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[e.irq]);
    }

    dev = sysbus_create_simple("nvt-lcd300", NA51103_IO(0x00800000), pic[18]);
    sysbus_create_simple("nvt-hdmi", NA51103_IO(0x00880000), pic[52]);
    dev = qdev_new("nvt-h26xdec");
    qdev_prop_set_bit(dev, "na51090", m->media_na51090);
    object_property_add_child(OBJECT(machine), "h26xdec", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NA51103_IO(0x00600000));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[9]);

    for (n = 0; n < m->n_usb; n++) {
        dev = qdev_new("nvt-usb");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, m->usb[n].base, 1);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->usb[n].base, 2);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->usb[n].irq]);
    }

    if (m->xhci.base) {
        /* the third USB bus, usb-bus.2: -device usb-storage,bus=usb-bus.2 */
        dev = qdev_new(TYPE_XHCI_SYSBUS);
        qdev_prop_set_uint32(dev, "slots", 64);
        object_property_add_child(OBJECT(machine), "xhci", OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->xhci.base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->xhci.irq]);
    }

    for (n = 0; n < m->n_sata; n++) {
        DriveInfo *hd[1] = { drive_get(IF_IDE, 0, n) };
        static const char *const names[][2] = {
            { "nvt-sata-glue", "nvt-sata-phy" },
            { "nvt-sata2-glue", "nvt-sata2-phy" },
            { "nvt-sata3-glue", "nvt-sata3-phy" },
        };

        dev = qdev_new(TYPE_SYSBUS_AHCI);
        qdev_prop_set_uint32(dev, "num-ports", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->sata[n].ahci,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[m->sata[n].irq]);
        ahci_ide_create_devs(&SYSBUS_AHCI(dev)->ahci, hd);
        {
            MemoryRegion *ghc = g_new(MemoryRegion, 1);

            /* Not owned by the AHCI: its re-entrancy guard would block
             * the forwarded access. */
            memory_region_init_io(ghc, OBJECT(machine), &na51103_ghc_ops,
                                  sysbus_mmio_get_region(SYS_BUS_DEVICE(dev),
                                                         0),
                                  "nvt-sata.ghc", 4);
            memory_region_add_subregion_overlap(sysmem,
                                                m->sata[n].ahci +
                                                NA51103_AHCI_GHC, ghc, 2);
        }
        for (i = 0; i < 2; i++) {
            DeviceState *bank = qdev_new("hisi-regbank");

            qdev_prop_set_uint32(bank, "size", i ? 0x8000 : 0x1000);
            qdev_prop_set_string(bank, "name", names[n][i]);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(bank), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(bank), 0,
                                    i ? m->sata[n].phy
                                      : m->sata[n].glue, 1);
        }
    }

    for (n = 0; n < ARRAY_SIZE(na51103_dais); n++) {
        sysbus_create_simple("nvt-dai", na51103_dais[n].base,
                             pic[na51103_dais[n].irq]);
    }

    for (n = 0; n < m->n_uarts; n++) {
        serial_mm_init(sysmem, m->uarts[n].base, 2,
                       pic[m->uarts[n].irq], 48000000 / 16,
                       serial_hd(n), DEVICE_LITTLE_ENDIAN);
    }

    for (n = 0; n < m->n_banks; n++) {
        dev = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(dev, "size", m->banks[n].size);
        qdev_prop_set_string(dev, "name", m->banks[n].name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, m->banks[n].base, 1);
    }

    if (machine->kernel_filename && !machine->dtb) {
        error_report("novatek: %s needs the firmware's device tree (-dtb)",
                     MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    binfo.ram_size = MIN(c->linux_mem, machine->ram_size);
    binfo.loader_start = 0;
    binfo.psci_conduit = QEMU_PSCI_CONDUIT_SMC;
    arm_load_kernel(cpu[0], machine, &binfo);

    /* As on NA51089: the loader's work, then U-Boot at EL1. */
    if (!machine->kernel_filename && nms->flash_file && !rom) {
        nvt_flash_boot(nms->flash, machine->ram_size, true);
    }
}

#define DEFINE_NVT_A64_MACHINE(namestr, tag, config)                 \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        novatek_a64_init(machine, &config);                          \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a53");      \
        /* the A53s and the loader's AArch32 boot core */            \
        mc->max_cpus = config.a64->a53s + 1;                         \
        mc->default_cpus = config.a64->a53s +                        \
                           config.a64->loader_default;               \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, NVT_MACHINE, NvtMachineState,   \
                            tag##_class_init, false,                  \
                            aarch64_machine_interfaces)

DEFINE_NVT_A64_MACHINE("nt98332", nt98332, nt98332_soc)
DEFINE_NVT_A64_MACHINE("nt98633", nt98633, nt98633_soc)
