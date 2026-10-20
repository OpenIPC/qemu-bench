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
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/nvt-flash.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
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
#define NVT_MPCORE_BASE     0xFFD00000
#define NVT_L2_BASE         0xFFE00000

/* GIC SPI numbers (nvt-peri.dtsi) */
#define NVT_IRQ_GPIO        24
#define NVT_IRQ_FLASH       29
#define NVT_IRQ_ETH         34
#define NVT_IRQ_RTC         56
#define NVT_IRQ_WDT         57
#define NVT_IRQ_L2          96

#define NVT_NUM_IRQ         256     /* irqs.h: SPIs 32..255 */

static const struct {
    hwaddr base;
    int irq;
    uint32_t clk;
} nvt_uarts[] = {
    { 0xF0290000, 43, 24000000 },
    { 0xF0300000, 44, 48000000 },
    { 0xF0310000, 45, 48000000 },
};

static const struct {
    hwaddr base;
    int irq;
} nvt_sdio[] = {
    { 0xF0420000, 30 },
    { 0xF0500000, 31 },
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
    { "nvt-timer",  NVT_TIMER_BASE, 0x10000 },
};

/*
 * NT98562: 64 MB DDR2 in the package.  cfg_562_IPCAM1_NOR_EVB's
 * nvt-mem-tbl.dtsi gives Linux the first 30 MB and the HDAL media pool
 * (vendor modules) the rest.
 */
static const NvtSoCConfig nt98562_soc = {
    .desc               = "Novatek NT98562 (Cortex-A9, NA51089, 64 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = 64 * MiB,
    .linux_mem          = 0x01E00000,
};

/* NT98566: the same die with 128 MB DDR3.  The SDK's IPCAM1_EVB image
 * (FW98566A.bin, v1.00.000) keeps Linux to 32 MB and gives the HDAL media
 * pool the other 96. */
static const NvtSoCConfig nt98566_soc = {
    .desc               = "Novatek NT98566 (Cortex-A9, NA51089, 128 MB)",
    .model              = "Novatek NA51089",
    .chip_id            = 0x7021,
    .ram_size_default   = 128 * MiB,
    .linux_mem          = 0x02000000,
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

    /* spinor.c names the MTD after nvt-devname: mtdparts=spi_nor.0:... */
    qemu_fdt_add_subnode(fdt, "/nor@f0400000");
    path = "/nor@f0400000";
    qemu_fdt_setprop_string(fdt, path, "compatible", "nvt,nvt_spinor");
    qemu_fdt_setprop_cells(fdt, path, "reg", NVT_FLASH_BASE, 0x1000);
    nvt_fdt_irq(fdt, path, NVT_IRQ_FLASH);
    qemu_fdt_setprop_cell(fdt, path, "clock-frequency", 48000000);
    qemu_fdt_setprop_string(fdt, path, "nvt-devname", "spi_nor.0");
    qemu_fdt_setprop_cell(fdt, path, "trace-stdtable", 1);

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
 * partition (0x10000) to /nvt_memory_cfg/fdt, loads the uboot partition to
 * /nvt_memory_cfg/uboot -- unpacking a BCL1 one -- fills the SHMINFO block
 * at /nvt_memory_cfg/shmem that U-Boot's nvt_shminfo_init() insists on
 * ("LD_NVT", the FDT address), and calls U-Boot's entry.  DDR needs no
 * training here, so the machine does those steps itself and leaves a
 * two-word branch at 0 for the CPU to start on.
 */
#define NVT_FLASH_FDT_OFF   0x10000
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
    char path[96];
    int nor;

    nvt_flash_read(flash, NVT_FLASH_FDT_OFF, hdr, sizeof(hdr));
    if (ldl_be_p(hdr) != FDT_MAGIC) {
        error_report("novatek: no device tree at flash 0x%x: not a Novatek "
                     "flash image (qemu-boot/nvtpack2flash.py makes one from "
                     "an FW*.bin)", NVT_FLASH_FDT_OFF);
        exit(1);
    }
    fdt_len = ldl_be_p(hdr + 4);
    if (fdt_len < sizeof(hdr) || fdt_len > 1 * MiB) {
        error_report("novatek: device tree size 0x%x in flash", fdt_len);
        exit(1);
    }
    fdt = g_malloc(fdt_len);
    nvt_flash_read(flash, NVT_FLASH_FDT_OFF, fdt, fdt_len);
    nor = fdt_node_offset_by_compatible(fdt, -1, "nvt,nvt_spinor");
    if (fdt_check_header(fdt) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/fdt", 1, &fdt_addr, &fdt_max) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/uboot", 1, &ub_addr, &ub_max) ||
        nvt_fdt_reg(fdt, "/nvt_memory_cfg/shmem", 1, &shm_addr, &shm_size) ||
        nor < 0 || fdt_get_path(fdt, nor, path, sizeof(path) - 20)) {
        error_report("novatek: the flash's device tree lacks nvt_memory_cfg "
                     "or the nor node");
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
     * leaves it: name, boot storage SPI NOR, FDT address, normal boot. */
    memcpy(shm, "LD_NVT", 6);
    stw_le_p(shm + 42, 3);                  /* boot.LdStorage */
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
    DeviceState *mp, *dev, *flash;
    SysBusDevice *mpbus;
    Object *cpuobj;
    ARMCPU *cpu;
    int n;

    nvt_config = c;
    nvt_linux_mem = MIN(c->linux_mem, machine->ram_size);

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

    dev = qdev_new("nvt-cg");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_CG_BASE, 1);

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
    dev = qdev_new("nvt-gpio");
    object_property_add_child(OBJECT(machine), "nvt-gpio", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_GPIO_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_GPIO]);

    dev = qdev_new("nvt-eqos");
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NVT_ETH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NVT_IRQ_ETH]);

    /* Empty SD slots: U-Boot looks for an update image on the card. */
    for (n = 0; n < ARRAY_SIZE(nvt_sdio); n++) {
        sysbus_create_simple("nvt-sdio", nvt_sdio[n].base,
                             pic[nvt_sdio[n].irq]);
    }

    flash = qdev_new(TYPE_NVT_FLASH);
    dev = flash;
    if (nms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", nms->flash_file);
    }
    qdev_prop_set_uint32(dev, "jedec", nms->flash_jedec);
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
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
        "SPI NOR image (power-of-two size, 1..32 MiB); erases and programs "
        "are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        nvt_machine_get_flash_jedec, nvt_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with; default a Winbond W25Q of the "
        "image's size");
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
