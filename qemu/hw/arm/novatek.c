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
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/novatek.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
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
#define NVT_MPCORE_BASE     0xFFD00000
#define NVT_L2_BASE         0xFFE00000

/* GIC SPI numbers (nvt-peri.dtsi) */
#define NVT_IRQ_GPIO        24
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

    qemu_fdt_add_subnode(fdt, "/wdt@f0050000");
    qemu_fdt_setprop_string(fdt, "/wdt@f0050000", "compatible",
                            "nvt,nvt_wdt");
    qemu_fdt_setprop_cells(fdt, "/wdt@f0050000", "reg",
                           NVT_WDT_BASE, 0x10000);
    nvt_fdt_irq(fdt, "/wdt@f0050000", NVT_IRQ_WDT);

    return fdt;
}

static void novatek_common_init(MachineState *machine, const NvtSoCConfig *c)
{
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq pic[NVT_NUM_IRQ - 32];
    DeviceState *mp, *dev;
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
    binfo.get_dtb = nvt_build_fdt;
    arm_load_kernel(cpu, machine, &binfo);
}

#define DEFINE_NVT_MACHINE(namestr, tag, config)                     \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        novatek_common_init(machine, &config);                       \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "novatek.ram";                          \
        mc->ignore_memory_transaction_failures = true;               \
    }                                                                \
    DEFINE_MACHINE_ARM(namestr, tag##_class_init)

DEFINE_NVT_MACHINE("nt98562", nt98562, nt98562_soc)
DEFINE_NVT_MACHINE("nt98566", nt98566, nt98566_soc)
