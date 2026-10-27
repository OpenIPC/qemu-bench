/*
 * Novatek NT9856x (NA51089) IP-camera SoC machines, and the NT98332
 * (NA51103) NVR die at the end of the file.
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
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/novatek.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
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

static const struct {
    hwaddr base;
    int irq;
    uint32_t clk;
} nvt_uarts[] = {
    { 0xF0290000, 43, 24000000 },
    { 0xF0300000, 44, 48000000 },
    { 0xF0310000, 45, 48000000 },
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

/*
 * Image sensors for sensor=: SmartSens parts the OpenIPC release has a
 * driver for (nvt_sen_<name>.ko), at 0x30 on I2C 0 where ipcinfo's
 * Novatek HAL probes (ssens_addrs 0x60, 8-bit), chip ID at 0x3107/0x3108.
 */
static const struct {
    const char *name;
    uint8_t id_high, id_low;
} nvt_sensors[] = {
    { "sc401ai", 0xCD, 0x2E },
    { "sc501ai", 0xCE, 0x1F },
};

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
#define NVT_MEDIA_REGS  3

static const struct {
    const char *node;
    const char *compat;
    struct {
        hwaddr base;
        uint32_t size;
        int irq;
    } win[NVT_MEDIA_REGS];
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

/* "nvt,ca9" is what mach-nvt-ivot/board-dt.c matches. */
static const char nvt_compat[] = "novatek,na51089\0nvt,ca9";

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
    uint32_t flash_jedec;
    char *sensor;
    NvtFlashState *flash;
};

static const NvtSoCConfig *nvt_config;
static uint64_t nvt_linux_mem;

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
    qemu_fdt_setprop(fdt, "/", "compatible", nvt_compat, sizeof(nvt_compat));
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
    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "compatible",
                            "arm,cortex-a9");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "next-level-cache", l2);

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

    for (n = 0; n < ARRAY_SIZE(nvt_uarts); n++) {
        upath = g_strdup_printf("/uart@%" HWADDR_PRIx, nvt_uarts[n].base);
        path = upath;
        qemu_fdt_add_subnode(fdt, path);
        qemu_fdt_setprop_string(fdt, path, "compatible", "ns16550a");
        qemu_fdt_setprop_cells(fdt, path, "reg", nvt_uarts[n].base, 0x1000);
        nvt_fdt_irq(fdt, path, nvt_uarts[n].irq);
        qemu_fdt_setprop_cell(fdt, path, "baud", 115200);
        qemu_fdt_setprop_cell(fdt, path, "reg-shift", 2);
        qemu_fdt_setprop_cell(fdt, path, "reg-io-width", 4);
        qemu_fdt_setprop_cell(fdt, path, "no-loopback-test", 1);
        qemu_fdt_setprop_cell(fdt, path, "clock-frequency", nvt_uarts[n].clk);
        qemu_fdt_setprop_cell(fdt, path, "fifo-size", 64);
        qemu_fdt_setprop_cell(fdt, path, "uart_id", n);
        g_free(upath);
    }

    /* Built in: the reboot notifier powers off through it. */
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

    for (n = ARRAY_SIZE(nvt_i2c) - 1; n >= 0; n--) {
        upath = g_strdup_printf("/%s@%" HWADDR_PRIx, nvt_i2c[n].node,
                                nvt_i2c[n].base);
        qemu_fdt_add_subnode(fdt, upath);
        qemu_fdt_setprop_string(fdt, upath, "compatible", "nvt,nvt_i2c");
        qemu_fdt_setprop_cells(fdt, upath, "reg", nvt_i2c[n].base, 0x100);
        nvt_fdt_irq(fdt, upath, nvt_i2c[n].irq);
        qemu_fdt_setprop_cell(fdt, upath, "clock-frequency", 400000);
        qemu_fdt_setprop_cell(fdt, upath, "id", n);
        qemu_fdt_setprop_cell(fdt, upath, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, upath, "#size-cells", 0);
        g_free(upath);
    }

    /* The media engines, and the HDAL units and limits. */
    for (n = 0; n < ARRAY_SIZE(nvt_media); n++) {
        g_autofree char *mpath = g_strdup_printf("/%s", nvt_media[n].node);
        uint32_t regs[2 * NVT_MEDIA_REGS], irqs[3 * NVT_MEDIA_REGS];
        int nr = 0, ni = 0, w;

        qemu_fdt_add_subnode(fdt, mpath);
        qemu_fdt_setprop_string(fdt, mpath, "compatible",
                                nvt_media[n].compat);
        for (w = 0; w < NVT_MEDIA_REGS && nvt_media[n].win[w].size; w++) {
            regs[nr++] = cpu_to_be32(nvt_media[n].win[w].base);
            regs[nr++] = cpu_to_be32(nvt_media[n].win[w].size);
            if (nvt_media[n].win[w].irq >= 0 || !strcmp(nvt_media[n].compat,
                                                        "nvt,nvt_lvds")) {
                /* LVDS names CSI's interrupts, 54 and 55 */
                int spi = nvt_media[n].win[w].irq >= 0 ?
                          nvt_media[n].win[w].irq : 54 + w;

                irqs[ni++] = cpu_to_be32(0);
                irqs[ni++] = cpu_to_be32(spi);
                irqs[ni++] = cpu_to_be32(4);
            }
        }
        qemu_fdt_setprop(fdt, mpath, "reg", regs, nr * 4);
        if (ni) {
            qemu_fdt_setprop(fdt, mpath, "interrupts", irqs, ni * 4);
        }
        if (!strcmp(nvt_media[n].compat, "nvt,kdrv_ai")) {
            /* one rate an engine, as kdrv_ai reads it: NUE, NUE2, CNN */
            qemu_fdt_setprop_cells(fdt, mpath, "clock-frequency",
                                   600000000, 480000000, 600000000);
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
    qemu_fdt_setprop_cells(fdt, "/nvt_arb@f0000000", "reg",
                           0xF0000000, 0xA000, NVT_ARB2_BASE, 0x300);
    nvt_fdt_irq(fdt, "/nvt_arb@f0000000", NVT_IRQ_ARB);

    /*
     * The drivers bind these, and take "mclk" (1: 320 MHz) for the clock
     * they parent the block on.
     */
    for (n = 0; n < ARRAY_SIZE(nvt_crypto_blocks); n++) {
        g_autofree char *cpath = g_strdup_printf("/%s@%" HWADDR_PRIx,
                                                 nvt_crypto_blocks[n].node,
                                                 nvt_crypto_blocks[n].base);
        g_autofree char *compat = g_strdup_printf("nvt,nvt_%s",
                                                  nvt_crypto_blocks[n].node);

        qemu_fdt_add_subnode(fdt, cpath);
        qemu_fdt_setprop_string(fdt, cpath, "compatible", compat);
        qemu_fdt_setprop_cells(fdt, cpath, "reg", nvt_crypto_blocks[n].base,
                               0x100);
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
    qemu_fdt_setprop_cells(fdt, "/eth@f02b0000", "reg", NVT_ETH_BASE, 0x3800);
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

static void nvt_flash_boot(NvtFlashState *flash, uint64_t ram_size, bool a64)
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
    nor = fdt_node_offset_by_compatible(fdt, -1, flash->nand ?
                                        "nvt,nvt_spinand" : "nvt,nvt_spinor");
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

static void novatek_common_init(MachineState *machine, const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq pic[NVT_NUM_IRQ - 32];
    DeviceState *mp, *dev, *flash, *cg, *gpio, *ife = NULL;
    I2CBus *i2c0 = NULL;
    SysBusDevice *mpbus;
    Object *cpuobj;
    ARMCPU *cpu;
    int n;

    nvt_config = c;
    nvt_linux_mem = machine->ram_size >= NVT_RAM_DEFAULT ? c->linux_mem :
                    MIN(c->sdk_linux_mem, machine->ram_size);

    cpuobj = object_new(machine->cpu_type);
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
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    memory_region_add_subregion(sysmem, 0, machine->ram);

    /* Everything not modelled below reads as zero and is logged. */
    create_unimplemented_device("nvt-periph", NVT_PERIPH_BASE,
                                NVT_PERIPH_SIZE);

    mp = qdev_new("a9mpcore_priv");
    mpbus = SYS_BUS_DEVICE(mp);
    qdev_prop_set_uint32(mp, "num-cpu", 1);
    qdev_prop_set_uint32(mp, "num-irq", NVT_NUM_IRQ);
    sysbus_realize_and_unref(mpbus, &error_fatal);
    sysbus_mmio_map(mpbus, 0, NVT_MPCORE_BASE);
    sysbus_connect_irq(mpbus, 0, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(mpbus, 1, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));
    sysbus_connect_irq(mpbus, 2, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VIRQ));
    sysbus_connect_irq(mpbus, 3, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VFIQ));
    for (n = 0; n < NVT_NUM_IRQ - 32; n++) {
        pic[n] = qdev_get_gpio_in(mp, n);
    }

    sysbus_create_simple("l2x0", NVT_L2_BASE, NULL);

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_TOP_BASE, 1);

    cg = qdev_new("nvt-cg");
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
        int w;

        for (w = 0; w < NVT_MEDIA_REGS && nvt_media[n].win[w].size; w++) {
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
            qdev_prop_set_uint32(dev, "size", nvt_media[n].win[w].size);
            if (!strcmp(engine, "ife")) {
                ife = dev;
            } else if (!strcmp(engine, "ime")) {
                object_property_set_link(OBJECT(dev), "ipp-input",
                                         OBJECT(ife), &error_fatal);
            }
            object_property_add_child(OBJECT(machine), engine, OBJECT(dev));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, nvt_media[n].win[w].base);
            if (nvt_media[n].win[w].irq >= 0) {
                sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                                   pic[nvt_media[n].win[w].irq]);
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

    for (n = 0; n < ARRAY_SIZE(nvt_crypto_blocks); n++) {
        sysbus_create_simple(nvt_crypto_blocks[n].type,
                             nvt_crypto_blocks[n].base,
                             pic[nvt_crypto_blocks[n].irq]);
    }

    /* Named, so qtest and a board can watch its "out" lines. */
    dev = qdev_new("nvt-pwm");
    object_property_add_child(OBJECT(machine), "nvt-pwm", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NVT_PWM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_PWM]);

    for (n = 0; n < ARRAY_SIZE(nvt_i2c); n++) {
        dev = sysbus_create_simple("nvt-i2c", nvt_i2c[n].base,
                                   pic[nvt_i2c[n].irq]);
        if (n == 0) {
            i2c0 = (I2CBus *)qdev_get_child_bus(dev, "i2c");
        }
    }
    if (nms->sensor && strcmp(nms->sensor, "none")) {
        for (n = 0; n < ARRAY_SIZE(nvt_sensors); n++) {
            if (!strcmp(nms->sensor, nvt_sensors[n].name)) {
                break;
            }
        }
        if (n == ARRAY_SIZE(nvt_sensors)) {
            error_report("novatek: unknown sensor '%s' (none, sc401ai, "
                         "sc501ai)", nms->sensor);
            exit(1);
        }
        dev = DEVICE(i2c_slave_new("hisi-smartsens", 0x30));
        qdev_prop_set_uint8(dev, "id_high", nvt_sensors[n].id_high);
        qdev_prop_set_uint8(dev, "id_low", nvt_sensors[n].id_low);
        i2c_slave_realize_and_unref(I2C_SLAVE(dev), i2c0, &error_fatal);
    }

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
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
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

    for (n = 0; n < ARRAY_SIZE(nvt_uarts); n++) {
        serial_mm_init(sysmem, nvt_uarts[n].base, 2, pic[nvt_uarts[n].irq],
                       nvt_uarts[n].clk / 16, serial_hd(n),
                       DEVICE_LITTLE_ENDIAN);
    }

    for (n = 0; n < ARRAY_SIZE(nvt_regbanks); n++) {
        dev = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(dev, "size", nvt_regbanks[n].size);
        qdev_prop_set_string(dev, "name", nvt_regbanks[n].name);
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
    arm_load_kernel(cpu, machine, &binfo);

    if (!machine->kernel_filename && nms->flash_file) {
        nvt_flash_boot(NVT_FLASH(flash), machine->ram_size, false);
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

static void nvt_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");
    mc->default_ram_id = "novatek.ram";
    mc->ignore_memory_transaction_failures = true;
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
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, NVT_MACHINE, NvtMachineState,   \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_NVT_MACHINE("nt98562", nt98562, nt98562_soc)
DEFINE_NVT_MACHINE("nt98566", nt98566, nt98566_soc)

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
#define NS02501_CC_SIZE         0x300
#define NS02501_CC_RELEASE      0x100   /* CC_CPU2_CPU1_CMDBUF_REG1 */
#define NS02501_CNTFRQ          12000000
#define NS02501_NUM_SPI         128
#define NS02501_IRQ_WDT         75
#define NS02501_IRQ_RTC         76
#define NS02501_IRQ_CC          59

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

/* -kernel: no boot stub for the second core, which waits for the kernel's
 * write to CMDBUF_REG1 as above. */
static void ns02501_write_secondary(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void ns02501_reset_secondary(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

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

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
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
        dev = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(dev, "size", nvt_regbanks[n].size);
        qdev_prop_set_string(dev, "name", nvt_regbanks[n].name);
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
    binfo.write_secondary_boot = ns02501_write_secondary;
    binfo.secondary_cpu_reset_hook = ns02501_reset_secondary;
    arm_load_kernel(cpu[0], machine, &binfo);

    if (!machine->kernel_filename && nms->flash_file) {
        nvt_flash_boot(nms->flash, machine->ram_size, false);
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

/* 16550s with reg-shift 2; 48 MHz (the DT's clock-frequency). */
static const struct {
    hwaddr base;
    int irq;
} na51103_uarts[] = {
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
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
    int irq;
    uint32_t fire_reg, fire_mask, sts_reg, en_reg, done_bits;
    uint32_t ll_reg, ll_fire_mask;
    bool scaler;
} na51103_engines[] = {
    /* kdrv_vpe: CTRL bit 31 fires the link list at 0x300, bit 30 a single
     * job; 0x400/0x404 status/enable, bit 0 frame end, bit 2 list end. */
    { "nvt-vpe", NA51103_IO(0x00410000), 0x1040, 3,
      0x000, 0xC0000000, 0x400, 0x404, 0x5, 0x300, 0x80000000, true },
};

static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} na51103_regbanks[] = {
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
static const struct {
    hwaddr ahci, glue, phy;
    int irq;
} na51103_sata[] = {
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

/* The two USB 2.0 hosts (nvt-usb: EHCI over the FOTG200 glue). */
static const struct {
    hwaddr base;
    int irq;
} na51103_usb[] = {
    { NA51103_IO(0x00480000), 27 },
    { NA51103_IO(0x00490000), 78 },
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
};

static void novatek_a64_init(MachineState *machine, const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    unsigned int smp_cpus = machine->smp.cpus;
    qemu_irq pic[NA51103_NUM_SPI];
    DeviceState *gic, *dev;
    SysBusDevice *gicbus;
    ARMCPU *cpu[4];
    int n, i;

    if (smp_cpus > ARRAY_SIZE(cpu)) {
        error_report("novatek: at most %zu CPUs", ARRAY_SIZE(cpu));
        exit(1);
    }

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        /*
         * The kernel is entered at EL1, with QEMU answering PSCI over SMC
         * in place of the firmware's BL31: the tree's psci node says
         * method = "smc", and the secondary comes up by CPU_ON.
         */
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
        object_property_set_bool(cpuobj, "has_el2", false, &error_fatal);
        object_property_set_int(cpuobj, "psci-conduit",
                                QEMU_PSCI_CONDUIT_SMC, &error_fatal);
        object_property_set_uint(cpuobj, "cntfrq", NA51103_CNTFRQ,
                                 &error_fatal);
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
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
    if (!machine->kernel_filename) {
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
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_TOP_BASE, 1);

    dev = qdev_new("nvt-cg");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_CG_BASE, 1);

    dev = qdev_new("nvt-gpio");
    object_property_add_child(OBJECT(machine), "nvt-gpio", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_GPIO_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_GPIO]);

    dev = qdev_new("nvt-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_WDT_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_WDT]);

    dev = qdev_new("nvt-rtc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_RTC_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_RTC]);

    sysbus_create_simple("nvt-otp", NA51103_OTP_BASE, NULL);
    sysbus_create_simple("nvt-timer", NA51103_TIMER_BASE, pic[0]);

    dev = qdev_new("nvt-eqos");
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_ETH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_ETH]);

    dev = qdev_new(TYPE_NVT_FLASH);
    if (nms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", nms->flash_file);
    }
    qdev_prop_set_uint32(dev, "jedec", nms->flash_jedec);
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    nms->flash = NVT_FLASH(dev);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51103_FLASH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51103_IRQ_FLASH]);

    for (n = 0; n < ARRAY_SIZE(na51103_engines); n++) {
        dev = qdev_new("nvt-engine");
        qdev_prop_set_string(dev, "name", na51103_engines[n].name);
        qdev_prop_set_uint32(dev, "size", na51103_engines[n].size);
        qdev_prop_set_uint32(dev, "fire-reg", na51103_engines[n].fire_reg);
        qdev_prop_set_uint32(dev, "fire-mask", na51103_engines[n].fire_mask);
        qdev_prop_set_uint32(dev, "sts-reg", na51103_engines[n].sts_reg);
        qdev_prop_set_uint32(dev, "en-reg", na51103_engines[n].en_reg);
        qdev_prop_set_uint32(dev, "done-bits", na51103_engines[n].done_bits);
        qdev_prop_set_uint32(dev, "ll-reg", na51103_engines[n].ll_reg);
        qdev_prop_set_uint32(dev, "ll-fire-mask",
                             na51103_engines[n].ll_fire_mask);
        qdev_prop_set_bit(dev, "scaler", na51103_engines[n].scaler);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                na51103_engines[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           pic[na51103_engines[n].irq]);
    }

    dev = sysbus_create_simple("nvt-lcd300", NA51103_IO(0x00800000), pic[18]);
    sysbus_create_simple("nvt-hdmi", NA51103_IO(0x00880000), pic[52]);
    sysbus_create_simple("nvt-h26xdec", NA51103_IO(0x00600000), pic[9]);

    for (n = 0; n < ARRAY_SIZE(na51103_usb); n++) {
        dev = qdev_new("nvt-usb");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, na51103_usb[n].base,
                                1);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51103_usb[n].base,
                                2);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51103_usb[n].irq]);
    }

    for (n = 0; n < ARRAY_SIZE(na51103_sata); n++) {
        DriveInfo *hd[1] = { drive_get(IF_IDE, 0, n) };
        static const char *const names[][2] = {
            { "nvt-sata-glue", "nvt-sata-phy" },
            { "nvt-sata2-glue", "nvt-sata2-phy" },
        };

        dev = qdev_new(TYPE_SYSBUS_AHCI);
        qdev_prop_set_uint32(dev, "num-ports", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51103_sata[n].ahci,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51103_sata[n].irq]);
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
                                                na51103_sata[n].ahci +
                                                NA51103_AHCI_GHC, ghc, 2);
        }
        for (i = 0; i < 2; i++) {
            DeviceState *bank = qdev_new("hisi-regbank");

            qdev_prop_set_uint32(bank, "size", i ? 0x8000 : 0x1000);
            qdev_prop_set_string(bank, "name", names[n][i]);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(bank), &error_fatal);
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(bank), 0,
                                    i ? na51103_sata[n].phy
                                      : na51103_sata[n].glue, 1);
        }
    }

    for (n = 0; n < ARRAY_SIZE(na51103_dais); n++) {
        sysbus_create_simple("nvt-dai", na51103_dais[n].base,
                             pic[na51103_dais[n].irq]);
    }

    for (n = 0; n < ARRAY_SIZE(na51103_uarts); n++) {
        serial_mm_init(sysmem, na51103_uarts[n].base, 2,
                       pic[na51103_uarts[n].irq], 48000000 / 16,
                       serial_hd(n), DEVICE_LITTLE_ENDIAN);
    }

    for (n = 0; n < ARRAY_SIZE(na51103_regbanks); n++) {
        dev = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(dev, "size", na51103_regbanks[n].size);
        qdev_prop_set_string(dev, "name", na51103_regbanks[n].name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                na51103_regbanks[n].base, 1);
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
    if (!machine->kernel_filename && nms->flash_file) {
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
        mc->max_cpus = 2;                                            \
        mc->default_cpus = 2;                                        \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, NVT_MACHINE, NvtMachineState,   \
                            tag##_class_init, false,                  \
                            aarch64_machine_interfaces)

DEFINE_NVT_A64_MACHINE("nt98332", nt98332, nt98332_soc)
