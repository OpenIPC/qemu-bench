/*
 * Novatek NT9856x (NA51089) IP-camera SoC machines.
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
#include "hw/irq.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/nvt-flash.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/device_tree.h"
#include "system/system.h"
#include "target/arm/cpu.h"
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
#define NVT_USB_ALIAS       0xFF600000  /* where NA51089's driver looks */
#define NVT_MPCORE_BASE     0xFFD00000
#define NVT_L2_BASE         0xFFE00000

/* GIC SPI numbers (nvt-peri.dtsi) */
#define NVT_IRQ_GPIO        24
#define NVT_IRQ_USB         27
#define NVT_IRQ_PWM         26
#define NVT_IRQ_TIMER       0
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
    { "ime@f0c40000",     "nvt,kdrv_ime",  { { 0xF0C40000, 0x1000, 6 } } },
    { "ise@f0c90000",     "nvt,kdrv_ise",  { { 0xF0C90000, 0x100, 21 } } },
    { "ipe@f0c30000",     "nvt,kdrv_ipe",  { { 0xF0C30000, 0x900, 5 } } },
    { "ife@f0c70000",     "nvt,kdrv_ife",  { { 0xF0C70000, 0x800, 8 } } },
    { "dce@f0c20000",     "nvt,kdrv_dce",  { { 0xF0C20000, 0x650, 7 } } },
    { "ai@f0c60000",      "nvt,kdrv_ai",
      { { 0xF0C60000, 0x23C, 14 }, { 0xF0D50000, 0x114, 13 },
        { 0xF0CB0000, 0x22C, 11 } }, 600000000 },
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
        if (nvt_media[n].clock) {
            qemu_fdt_setprop_cell(fdt, mpath, "clock-frequency",
                                  nvt_media[n].clock);
        }
    }
    for (n = 0; n < ARRAY_SIZE(nvt_hdal_units); n++) {
        g_autofree char *upath = g_strdup_printf("/%s", nvt_hdal_units[n]);
        g_autofree char *compat = g_strdup_printf("nvt,%s",
                                                  nvt_hdal_units[n]);

        qemu_fdt_add_subnode(fdt, upath);
        qemu_fdt_setprop_string(fdt, upath, "compatible", compat);
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

static void nvt_flash_boot(NvtFlashState *flash, uint64_t ram_size)
{
    uint8_t hdr[16];
    g_autofree uint8_t *fdt = NULL;
    g_autofree uint8_t *part = NULL;
    g_autofree uint8_t *uboot = NULL;
    uint8_t shm[NVT_SHMINFO_SIZE] = { 0 };
    uint64_t fdt_addr, fdt_max, ub_addr, ub_max, shm_addr, shm_size;
    uint64_t ub_off, ub_len, ub_size;
    uint32_t stub[2], fdt_len;
    uint32_t fdt_off = flash->nand ? NVT_NAND_FDT_OFF : NVT_NOR_FDT_OFF;
    char path[96];
    int nor;

    nvt_flash_read(flash, fdt_off, hdr, sizeof(hdr));
    if (ldl_be_p(hdr) != FDT_MAGIC) {
        error_report("novatek: no device tree at flash 0x%x: not a Novatek "
                     "flash image (qemu-boot/nvtpack2flash.py makes one from "
                     "an FW*.bin)", fdt_off);
        exit(1);
    }
    fdt_len = ldl_be_p(hdr + 4);
    if (fdt_len < sizeof(hdr) || fdt_len > 1 * MiB) {
        error_report("novatek: device tree size 0x%x in flash", fdt_len);
        exit(1);
    }
    fdt = g_malloc(fdt_len);
    nvt_flash_read(flash, fdt_off, fdt, fdt_len);
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
    strcat(path, "/partition_uboot");
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

        if (alg != 9) {
            error_report("novatek: U-Boot is BCL1 algorithm %u; only LZ (9) "
                         "is unpacked here (11, LZMA, needs the mask ROM)",
                         alg);
            exit(1);
        }
        if (comp > ub_size - NVT_BCL1_HDR || raw > ub_max ||
            raw > NVT_UBOOT_MAX) {
            error_report("novatek: BCL1 U-Boot sizes 0x%x/0x%x do not fit",
                         comp, raw);
            exit(1);
        }
        uboot = g_malloc(raw);
        if (!nvt_lz_uncompress(part + NVT_BCL1_HDR, comp, uboot, raw)) {
            error_report("novatek: BCL1 U-Boot does not unpack");
            exit(1);
        }
        ub_len = raw;
    } else {
        uboot = g_steal_pointer(&part);
        ub_len = MIN(ub_size, ub_max);
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
    stub[0] = cpu_to_le32(0xE51FF004);      /* ldr pc, [pc, #-4] */
    stub[1] = cpu_to_le32(ub_addr);
    rom_add_blob_fixed("novatek.bootrom", stub, sizeof(stub), 0);
}

static void novatek_common_init(MachineState *machine, const NvtSoCConfig *c)
{
    NvtMachineState *nms = NVT_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq pic[NVT_NUM_IRQ - 32];
    DeviceState *mp, *dev, *flash, *cg, *gpio;
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
            dev = qdev_new("nvt-media");
            qdev_prop_set_string(dev, "engine", engine);
            qdev_prop_set_uint32(dev, "size", nvt_media[n].win[w].size);
            object_property_add_child(OBJECT(machine), engine, OBJECT(dev));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, nvt_media[n].win[w].base);
            if (nvt_media[n].win[w].irq >= 0) {
                sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                                   pic[nvt_media[n].win[w].irq]);
            }
        }
    }
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
        nvt_flash_boot(NVT_FLASH(flash), machine->ram_size);
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
