/*
 * Ambarella S3L: a single Cortex-A9 among Ambarella's own peripherals, as in
 * the Amcrest IP4M-1026E / Dahua IPC-HFW1430SN (docs/ambarella.md).
 *
 * The map follows arch/arm/boot/dts/ambarella-s3l.dtsi in the vendor kernel
 * (OpenIPC/linux, branch ambarella-s3l).  With -kernel the machine generates
 * the device tree the boot loader would pass, including the /iavmem node
 * that says how much of DRAM belongs to the DSP: Linux gets what is below
 * it.  What software needs from the IAV blocks stops at loading its modules;
 * there is no video here.  Blocks with no model read as zero.
 *
 * flash-file= attaches NAND, as on the Hikvision DS-2CD2423G0-I.  There are no
 * partitions in the generated tree: that layout is the board's, so a kernel
 * takes it from mtdparts= on its command line.
 *
 * As on the camera, the sensor stays off I2C until GPIO 107 releases its
 * XSHUTDN, so software that probes too early finds nothing.  sensor-reset=off
 * is a board with no such line.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/irq.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sd/sd.h"
#include "net/net.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "system/device_tree.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include <libfdt.h>

#define TYPE_AMB_MACHINE MACHINE_TYPE_NAME("s3l")
typedef struct AmbMachineState AmbMachineState;
DECLARE_INSTANCE_CHECKER(AmbMachineState, AMB_MACHINE, TYPE_AMB_MACHINE)

#define AMB_RAM_DEFAULT     (512 * MiB)
#define AMB_LINUX_BASE      0x00200000  /* the PPM below it is the DSP's */
#define AMB_IAVMEM_BASE     0x07800000  /* mem=118M, as on the IP4M-1026E */

#define AMB_AHB_BASE        0xe0000000
#define AMB_BOOT_BASE       0xffff0000  /* the high vectors */
#define AMB_BOOT_SIZE       0x1000
#define AMB_BST_SIZE        2048
#define AMB_FIO_FIFO_BASE   0xe0000000
#define AMB_FIO_BASE        0xe0001000
#define AMB_FDMA_BASE       0xe0012000
#define AMB_VIC0_BASE       0xe0003000
#define AMB_VIC1_BASE       0xe0010000
#define AMB_SCRATCHPAD_BASE 0xe001b000
#define AMB_VIC2_BASE       0xe001c000
#define AMB_APB_BASE        0xe8000000
#define AMB_IDC1_BASE       0xe8001000
#define AMB_IDC0_BASE       0xe8003000
#define AMB_UART0_BASE      0xe8005000
#define AMB_GPIO0_BASE      0xe8009000
#define AMB_GPIO1_BASE      0xe800a000
#define AMB_GPIO2_BASE      0xe800e000
#define AMB_GPIO3_BASE      0xe8010000
#define AMB_IOMUX_BASE      0xe8016000
#define AMB_IDC2_BASE       0xe8007000
#define AMB_TIMER_BASE      0xe800b000
#define AMB_DBGBUS_BASE     0xec000000
#define AMB_RCT_BASE        0xec170000
#define AMB_DRAMC_BASE      0xdffe0000
#define AMB_AXI_BASE        0xf0000000
#define AMB_L2_BASE         0xf0002000
#define AMB_SD0_BASE        0xe0002000
#define AMB_ETH_BASE        0xe000e000
#define AMB_SD2_BASE        0xe001f000
#define AMB_ETH_SIZE        0x2000
#define AMB_PHY_ID          0x001cc816  /* RTL8201F, as on the IP4M-1026E */
#define AMB_NAND_PHY_ID     0x02430c54  /* as on the DS-2CD2423G0-I */

#define AMB_GPIO_XSHUTDN    107

#define AMB_IRQ_TIMER7      6
#define AMB_IRQ_GPIO3       49
#define AMB_IRQ_GPIO2       50
#define AMB_IRQ_GPIO1       51
#define AMB_IRQ_GPIO0       52
#define AMB_IRQ_UART0       61
#define AMB_IRQ_ETH         65
#define AMB_IRQ_SD2         74
#define AMB_IRQ_SD0         75
#define AMB_IRQ_TIMER8      63
#define AMB_IRQ_FDMA        70
#define AMB_IRQ_FIO_CMD     72
#define AMB_IRQ_FIO_DMA     73
#define AMB_IRQ_IDC2        81
#define AMB_IRQ_IDC1        82
#define AMB_IRQ_IDC0        83
#define AMB_IRQ_VDSP        45   /* CODE_VDSP_0 "vdsp", per /proc/interrupts */
#define AMB_IRQ_VCAP        46   /* CODE_VDSP_1 "vcap" */
#define AMB_IRQ_VENC        47   /* CODE_VDSP_2 "venc" */

#define AMB_DT_EDGE_RISING  1
#define AMB_DT_LEVEL_HIGH   4

/* SYS_CONFIG straps for 2K pages, read-confirm and BCH on a 6-bit spare */
#define AMB_SYS_CONFIG_NAND 0x2007005b  /* as strapped on the DS-2CD2423G0-I */
#define AMB_SYS_CONFIG_ETH  0x00000001  /* the driver refuses without it */

#define AMB_UART_US         0x7c    /* past what the 16550 model covers */
#define AMB_UART_US_TFE     0x04
#define AMB_UART_US_TFNF    0x02

struct AmbMachineState {
    MachineState parent_obj;

    char *sensor;
    bool no_sensor_reset;
    char *flash_file;
    char *bad_blocks;
    bool dsp;
};

static const struct {
    const char *name;
    hwaddr base;
    uint64_t size;
} amb_unimp[] = {
    { "amb-ahb", AMB_AHB_BASE, 16 * MiB },
    { "amb-apb", AMB_APB_BASE, 16 * MiB },
    { "amb-dbgbus", AMB_DBGBUS_BASE, 2 * MiB },
    { "amb-dramc", AMB_DRAMC_BASE, 128 * KiB },
    { "amb-axi", AMB_AXI_BASE, 192 * KiB },
};

static const struct {
    hwaddr base;
    int irq;
} amb_idc[] = {
    { AMB_IDC0_BASE, AMB_IRQ_IDC0 },
    { AMB_IDC1_BASE, AMB_IRQ_IDC1 },
    { AMB_IDC2_BASE, AMB_IRQ_IDC2 },
};

/* The driver only reads this to see whether its transmit FIFO has room. */
static uint64_t amb_uart_ext_read(void *opaque, hwaddr off, unsigned size)
{
    return off == 0 ? AMB_UART_US_TFE | AMB_UART_US_TFNF : 0;
}

static void amb_uart_ext_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
}

static const MemoryRegionOps amb_uart_ext_ops = {
    .read = amb_uart_ext_read,
    .write = amb_uart_ext_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void amb_fdt_irq(void *fdt, const char *path, int irq, int type)
{
    qemu_fdt_setprop_cells(fdt, path, "interrupts", irq, type);
}

static void amb_fdt_timer(void *fdt, const char *compat, hwaddr base, int irq,
                          int ctrl_offset)
{
    g_autofree char *path = g_strdup_printf("/apb@e8000000/timer@%" HWADDR_PRIx,
                                            base);

    qemu_fdt_add_subnode(fdt, path);
    qemu_fdt_setprop_string(fdt, path, "compatible", compat);
    qemu_fdt_setprop_cells(fdt, path, "reg", base, 0x10,
                           AMB_TIMER_BASE + 0x30, 4);
    amb_fdt_irq(fdt, path, irq, AMB_DT_EDGE_RISING);
    qemu_fdt_setprop_cell(fdt, path, "ctrl-offset", ctrl_offset);
}

static void amb_fdt_sd(void *fdt, hwaddr base, int irq, const char *clock,
                       int id, hwaddr timing)
{
    g_autofree char *p = g_strdup_printf("/ahb@e0000000/sdmmc%d@%" HWADDR_PRIx,
                                         id ? 2 : 0, base);
    g_autofree char *slot = g_strdup_printf("%s/slot@0", p);

    qemu_fdt_add_subnode(fdt, p);
    qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,sdmmc");
    qemu_fdt_setprop_cell(fdt, p, "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, p, "#size-cells", 0);
    qemu_fdt_setprop_cells(fdt, p, "reg", base, 0x1000, AMB_FIO_BASE, 0x80,
                           timing, 8);
    amb_fdt_irq(fdt, p, irq, AMB_DT_LEVEL_HIGH);
    qemu_fdt_setprop_string(fdt, p, "amb,clk-name", clock);
    qemu_fdt_setprop_cell(fdt, p, "amb,wait-tmo", 10000);
    qemu_fdt_setprop_cell(fdt, p, "amb,switch-vol-tmo", 100);
    qemu_fdt_setprop_cell(fdt, p, "amb,max-blk-size", 131072);
    qemu_fdt_setprop_cell(fdt, p, "amb,phy-type", 0);
    qemu_fdt_setprop_cells(fdt, p, "amb,phy-timing",
                           0x1f, 0x04070000, 0,
                           0x60, 0x10410400, 0x00d04104,
                           0x80, 0x10410400, 0x02504104);
    qemu_fdt_setprop_string(fdt, p, "pinctrl-names", "default");
    qemu_fdt_setprop(fdt, p, "pinctrl-0", NULL, 0);

    qemu_fdt_add_subnode(fdt, slot);
    qemu_fdt_setprop_cell(fdt, slot, "reg", 0);
    qemu_fdt_setprop_cell(fdt, slot, "global-id", id);
    qemu_fdt_setprop_cell(fdt, slot, "bus-width", 4);
    qemu_fdt_setprop_cell(fdt, slot, "max-frequency", 50000000);
    qemu_fdt_setprop(fdt, slot, "no-1-8-v", NULL, 0);
    qemu_fdt_setprop_cell(fdt, slot, "amb,fixed-cd", 1);
    qemu_fdt_setprop_cell(fdt, slot, "amb,fixed-wp", 0);
}

static void *amb_build_fdt(const struct arm_boot_info *info, int *size)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    void *fdt = create_device_tree(size);
    AmbMachineState *ams = AMB_MACHINE(ms);
    uint32_t intc, pinctrl, gpio;

    intc = qemu_fdt_alloc_phandle(fdt);
    pinctrl = qemu_fdt_alloc_phandle(fdt);
    gpio = qemu_fdt_alloc_phandle(fdt);
    qemu_fdt_setprop_string(fdt, "/", "model", "Ambarella S3L Olive Board");
    qemu_fdt_setprop_string(fdt, "/", "compatible", "ambarella,s3l");
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/", "interrupt-parent", intc);

    qemu_fdt_add_subnode(fdt, "/aliases");
    qemu_fdt_setprop_string(fdt, "/aliases", "serial0",
                            "/apb@e8000000/uart@e8005000");
    qemu_fdt_setprop_string(fdt, "/aliases", "i2c0",
                            "/apb@e8000000/i2c@e8003000");
    qemu_fdt_setprop_string(fdt, "/aliases", "i2c1",
                            "/apb@e8000000/i2c@e8001000");
    qemu_fdt_setprop_string(fdt, "/aliases", "i2c2",
                            "/apb@e8000000/i2c@e8007000");

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "linux,stdout-path",
                            "/apb@e8000000/uart@e8005000");

    qemu_fdt_add_subnode(fdt, "/memory");
    qemu_fdt_setprop_string(fdt, "/memory", "device_type", "memory");
    qemu_fdt_setprop_cells(fdt, "/memory", "reg", AMB_LINUX_BASE,
                           AMB_IAVMEM_BASE - AMB_LINUX_BASE);

    /* What the boot loader adds.  The kernel takes it from here, and so
     * does anything reading /proc/device-tree/iavmem/reg. */
    qemu_fdt_add_subnode(fdt, "/iavmem");
    qemu_fdt_setprop_string(fdt, "/iavmem", "device_type", "iavmem");
    qemu_fdt_setprop_cells(fdt, "/iavmem", "reg", AMB_IAVMEM_BASE,
                           ms->ram_size - AMB_IAVMEM_BASE);

    /* iav.ko dereferences /iav/vinc0 without checking.  It takes the
     * VIN's reset from here, and a sensor driver releases it on loading:
     * the flag is the reset level, so 0 holds it low. */
    qemu_fdt_add_subnode(fdt, "/iav");
    qemu_fdt_setprop_string(fdt, "/iav", "compatible", "ambarella,iav");
    qemu_fdt_add_subnode(fdt, "/iav/vinc0");
    if (!ams->no_sensor_reset) {
        qemu_fdt_setprop_cells(fdt, "/iav/vinc0", "vinrst-gpios", gpio,
                               AMB_GPIO_XSHUTDN, 0);
    }

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0);
    qemu_fdt_add_subnode(fdt, "/cpus/cpu@0");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "device_type", "cpu");
    qemu_fdt_setprop_string(fdt, "/cpus/cpu@0", "compatible",
                            "arm,cortex-a9");
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "reg", 0);

    qemu_fdt_add_subnode(fdt, "/ahb@e0000000");
    qemu_fdt_setprop_string(fdt, "/ahb@e0000000", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(fdt, "/ahb@e0000000", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/ahb@e0000000", "#size-cells", 1);
    qemu_fdt_setprop(fdt, "/ahb@e0000000", "ranges", NULL, 0);
    qemu_fdt_add_subnode(fdt, "/ahb@e0000000/interrupt-controller@e0003000");
    {
        const char *p = "/ahb@e0000000/interrupt-controller@e0003000";

        qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,vic");
        qemu_fdt_setprop(fdt, p, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, p, "#interrupt-cells", 2);
        qemu_fdt_setprop_cells(fdt, p, "reg", AMB_VIC0_BASE, 0x1000,
                               AMB_VIC1_BASE, 0x1000, AMB_VIC2_BASE, 0x1000);
        qemu_fdt_setprop_cell(fdt, p, "phandle", intc);
    }

    /*
     * No nand-on-flash-bbt, which the real tree has: with none found the
     * kernel would write a table into blocks a dump may not have read.
     * Without it, it reads the markers of every block at boot and writes
     * nothing.
     */
    if (ams->flash_file) {
        const char *p = "/ahb@e0000000/nand@e0001000";

        qemu_fdt_add_subnode(fdt, p);
        qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,nand");
        qemu_fdt_setprop_cell(fdt, p, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, p, "#size-cells", 1);
        qemu_fdt_setprop_cells(fdt, p, "reg", AMB_FIO_BASE, 0x1000,
                               AMB_FDMA_BASE, 0x1000, AMB_FIO_FIFO_BASE, 4);
        qemu_fdt_setprop_cells(fdt, p, "interrupts",
                               AMB_IRQ_FIO_CMD, AMB_DT_LEVEL_HIGH,
                               AMB_IRQ_FIO_DMA, AMB_DT_LEVEL_HIGH,
                               AMB_IRQ_FDMA, AMB_DT_LEVEL_HIGH);
    }

    qemu_fdt_add_subnode(fdt, "/apb@e8000000");
    qemu_fdt_setprop_string(fdt, "/apb@e8000000", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(fdt, "/apb@e8000000", "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, "/apb@e8000000", "#size-cells", 1);
    qemu_fdt_setprop(fdt, "/apb@e8000000", "ranges", NULL, 0);

    amb_fdt_timer(fdt, "ambarella,clock-source", AMB_TIMER_BASE + 0x64,
                  AMB_IRQ_TIMER7, 24);
    amb_fdt_timer(fdt, "ambarella,clock-event", AMB_TIMER_BASE + 0x74,
                  AMB_IRQ_TIMER8, 28);

    qemu_fdt_add_subnode(fdt, "/ahb@e0000000/ethernet@e000e000");
    {
        const char *p = "/ahb@e0000000/ethernet@e000e000";
        const char *phy = "/ahb@e0000000/ethernet@e000e000/phy@0";

        qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,eth");
        qemu_fdt_setprop_cell(fdt, p, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, p, "#size-cells", 0);
        qemu_fdt_setprop_cells(fdt, p, "reg", AMB_ETH_BASE, AMB_ETH_SIZE);
        amb_fdt_irq(fdt, p, AMB_IRQ_ETH, AMB_DT_LEVEL_HIGH);
        qemu_fdt_setprop_cell(fdt, p, "amb,tx-ring-size", 32);
        qemu_fdt_setprop_cell(fdt, p, "amb,rx-ring-size", 64);
        qemu_fdt_setprop_string(fdt, p, "pinctrl-names", "default");
        qemu_fdt_setprop(fdt, p, "pinctrl-0", NULL, 0);
        qemu_fdt_add_subnode(fdt, phy);
        /* amb_ethernet() puts a NAND board's PHY at MDIO address 1 */
        qemu_fdt_setprop_cell(fdt, phy, "reg", ams->flash_file ? 1 : 0);
    }

    amb_fdt_sd(fdt, AMB_SD0_BASE, AMB_IRQ_SD0, "gclk_sd", 0, 0xec1704c0);
    amb_fdt_sd(fdt, AMB_SD2_BASE, AMB_IRQ_SD2, "gclk_sdxc", 1, 0xec1704c8);

    /* The driver walks its children for pin groups, and there are none. */
    qemu_fdt_add_subnode(fdt, "/apb@e8000000/pinctrl@e8009000");
    {
        const char *p = "/apb@e8000000/pinctrl@e8009000";
        const char *g = "/apb@e8000000/pinctrl@e8009000/gpio@0";
        static const char compat[] = "ambarella,pinctrl\0simple-bus";
        static const char names[] = "gpio0\0gpio1\0gpio2\0gpio3\0iomux";

        qemu_fdt_setprop(fdt, p, "compatible", compat, sizeof(compat));
        qemu_fdt_setprop_cell(fdt, p, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, p, "#size-cells", 0);
        qemu_fdt_setprop_cells(fdt, p, "reg", AMB_GPIO0_BASE, 0x1000,
                               AMB_GPIO1_BASE, 0x1000, AMB_GPIO2_BASE, 0x1000,
                               AMB_GPIO3_BASE, 0x1000, AMB_IOMUX_BASE, 0x1000);
        qemu_fdt_setprop(fdt, p, "reg-names", names, sizeof(names));
        qemu_fdt_setprop_cell(fdt, p, "#gpio-range-cells", 3);
        qemu_fdt_setprop_cell(fdt, p, "phandle", pinctrl);

        qemu_fdt_add_subnode(fdt, g);
        qemu_fdt_setprop_string(fdt, g, "compatible", "ambarella,gpio");
        qemu_fdt_setprop_cells(fdt, g, "interrupts",
                               AMB_IRQ_GPIO0, AMB_DT_LEVEL_HIGH,
                               AMB_IRQ_GPIO1, AMB_DT_LEVEL_HIGH,
                               AMB_IRQ_GPIO2, AMB_DT_LEVEL_HIGH,
                               AMB_IRQ_GPIO3, AMB_DT_LEVEL_HIGH);
        qemu_fdt_setprop(fdt, g, "gpio-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, g, "#gpio-cells", 2);
        qemu_fdt_setprop_cells(fdt, g, "gpio-ranges", pinctrl, 0, 0, 128);
        qemu_fdt_setprop(fdt, g, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, g, "#interrupt-cells", 2);
        qemu_fdt_setprop_cell(fdt, g, "phandle", gpio);
    }

    qemu_fdt_add_subnode(fdt, "/apb@e8000000/uart@e8005000");
    {
        const char *p = "/apb@e8000000/uart@e8005000";

        qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,uart");
        qemu_fdt_setprop_cells(fdt, p, "reg", AMB_UART0_BASE, 0x1000);
        amb_fdt_irq(fdt, p, AMB_IRQ_UART0, AMB_DT_LEVEL_HIGH);
        /* The driver refuses to probe with no default pin state.  An empty
         * one is a dummy to Linux, and there is no pad mux to set up here. */
        qemu_fdt_setprop_string(fdt, p, "pinctrl-names", "default");
        qemu_fdt_setprop(fdt, p, "pinctrl-0", NULL, 0);
    }

    for (int i = 0; i < ARRAY_SIZE(amb_idc); i++) {
        g_autofree char *p = g_strdup_printf("/apb@e8000000/i2c@%" HWADDR_PRIx,
                                             amb_idc[i].base);

        qemu_fdt_add_subnode(fdt, p);
        qemu_fdt_setprop_string(fdt, p, "compatible", "ambarella,i2c");
        qemu_fdt_setprop_cell(fdt, p, "#address-cells", 1);
        qemu_fdt_setprop_cell(fdt, p, "#size-cells", 0);
        qemu_fdt_setprop_cells(fdt, p, "reg", amb_idc[i].base, 0x1000);
        amb_fdt_irq(fdt, p, amb_idc[i].irq, AMB_DT_LEVEL_HIGH);
        qemu_fdt_setprop_cell(fdt, p, "clock-frequency", 100000);
        qemu_fdt_setprop_cell(fdt, p, "amb,i2c-class", 0x81);
        if (i == 0) {
            /* What a sensor driver binds to, which is what releases its
             * reset.  The address is overwritten at run time. */
            g_autofree char *vin = g_strdup_printf("%s/ambvin0@01", p);

            qemu_fdt_add_subnode(fdt, vin);
            qemu_fdt_setprop_string(fdt, vin, "compatible",
                                    "ambarella,ambvin");
            qemu_fdt_setprop_cell(fdt, vin, "reg", 1);
        }
    }

    qemu_fdt_add_subnode(fdt, "/l2-cache@f0002000");
    qemu_fdt_setprop_string(fdt, "/l2-cache@f0002000", "compatible",
                            "arm,pl310-cache");
    qemu_fdt_setprop_cells(fdt, "/l2-cache@f0002000", "reg", AMB_L2_BASE,
                           0x1000);
    qemu_fdt_setprop(fdt, "/l2-cache@f0002000", "cache-unified", NULL, 0);
    qemu_fdt_setprop_cell(fdt, "/l2-cache@f0002000", "cache-level", 2);

    return fdt;
}

/* The Ethernet is a Synopsys core with normal descriptors, the layout Goke's
 * has, and the driver's own MDIO at the MAC. */
static void amb_ethernet(DeviceState *vic, bool nand_board)
{
    DeviceState *dev = qdev_new("xm-dwmac");

    qemu_configure_nic_device(dev, true, NULL);
    qdev_prop_set_uint32(dev, "regs-size", AMB_ETH_SIZE);
    /* The driver takes 4 off every received length */
    qdev_prop_set_bit(dev, "rx-fcs", true);
    qdev_prop_set_uint32(dev, "phy-id",
                         nand_board ? AMB_NAND_PHY_ID : AMB_PHY_ID);
    qdev_prop_set_uint32(dev, "phy-addr", nand_board ? 1 : 0);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AMB_ETH_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(vic, AMB_IRQ_ETH));
}

static void amb_sd(DeviceState *vic, hwaddr base, int irq, int unit)
{
    DeviceState *dev = qdev_new("generic-sdhci");
    DriveInfo *di = drive_get(IF_SD, 0, unit);

    /* QEMU's default capabilities and bit 27, which the driver refuses a
     * controller without.  It reads the write-protect pin inverted. */
    qdev_prop_set_uint64(dev, "capareg", 0x057834b4 | (1u << 27));
    qdev_prop_set_bit(dev, "wp-inverted", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, qdev_get_gpio_in(vic, irq));
    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);

        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }
}

/*
 * What the chip does at power-on from NAND: its boot logic reads the first
 * page into a buffer at the high vectors, and the CPU starts there.  That page
 * is the BST, which copies parts of itself to DRAM from those addresses, so
 * they are not incidental.  The stand-in for the CPU starting there sits at the
 * reset vector in DRAM, where the BST will put the BLD over it.
 */
static void amb_boot_from_flash(MemoryRegion *sysmem, const char *path)
{
    static const uint32_t jump[] = {
        0xe51ff004,                 /* ldr pc, [pc, #-4] */
        AMB_BOOT_BASE,
    };
    MemoryRegion *buf = g_new(MemoryRegion, 1);
    uint8_t image[AMB_BST_SIZE];
    int fd = open(path, O_RDONLY | O_BINARY);

    if (fd < 0 || read(fd, image, AMB_BST_SIZE) != AMB_BST_SIZE) {
        error_report("s3l: cannot read the BST from %s", path);
        exit(1);
    }
    close(fd);
    memory_region_init_ram(buf, NULL, "amb-boot", AMB_BOOT_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, AMB_BOOT_BASE, buf);
    rom_add_blob_fixed("amb-bst", image, AMB_BST_SIZE, AMB_BOOT_BASE);
    rom_add_blob_fixed("amb-reset", jump, sizeof(jump), 0);
}

static void amb_init(MachineState *ms)
{
    static struct arm_boot_info binfo;
    AmbMachineState *ams = AMB_MACHINE(ms);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *uart_ext = g_new(MemoryRegion, 1);
    Object *cpuobj;
    DeviceState *cpu, *vic, *timer, *pads, *rct;
    qemu_irq not_held;
    SysBusDevice *sbd;

    if (ms->ram_size <= AMB_IAVMEM_BASE) {
        error_report("s3l: -m must leave room for the DSP above %d MiB",
                     AMB_IAVMEM_BASE / (int)MiB);
        exit(1);
    }
    if (ms->ram_size > AMB_DRAMC_BASE) {
        error_report("s3l: -m must end below the peripherals at 0x%x",
                     AMB_DRAMC_BASE);
        exit(1);
    }

    memory_region_add_subregion(sysmem, 0, ms->ram);

    for (int i = 0; i < ARRAY_SIZE(amb_unimp); i++) {
        create_unimplemented_device(amb_unimp[i].name, amb_unimp[i].base,
                                    amb_unimp[i].size);
    }

    cpuobj = object_new(ms->cpu_type);
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    /* r4p1, as the camera reports: /proc/cpuinfo prints it */
    object_property_set_uint(cpuobj, "midr", 0x414fc091, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = DEVICE(cpuobj);
    object_unref(cpuobj);

    vic = qdev_new("amb-vic");
    sbd = SYS_BUS_DEVICE(vic);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, AMB_VIC0_BASE);
    sysbus_mmio_map(sbd, 1, AMB_VIC1_BASE);
    sysbus_mmio_map(sbd, 2, AMB_VIC2_BASE);
    sysbus_mmio_map(sbd, 3, AMB_SCRATCHPAD_BASE);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(cpu, ARM_CPU_IRQ));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(cpu, ARM_CPU_FIQ));

    timer = qdev_new("amb-timer");
    sbd = SYS_BUS_DEVICE(timer);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, AMB_TIMER_BASE);
    sysbus_connect_irq(sbd, 6, qdev_get_gpio_in(vic, AMB_IRQ_TIMER7));
    sysbus_connect_irq(sbd, 7, qdev_get_gpio_in(vic, AMB_IRQ_TIMER8));

    pads = qdev_new("amb-gpio");
    sbd = SYS_BUS_DEVICE(pads);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, AMB_GPIO0_BASE);
    sysbus_mmio_map(sbd, 1, AMB_GPIO1_BASE);
    sysbus_mmio_map(sbd, 2, AMB_GPIO2_BASE);
    sysbus_mmio_map(sbd, 3, AMB_GPIO3_BASE);

    rct = qdev_new("amb-rct");
    qdev_prop_set_uint32(rct, "sys-config",
                         ams->flash_file ? AMB_SYS_CONFIG_NAND
                                         : AMB_SYS_CONFIG_ETH);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(rct), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(rct), 0, AMB_RCT_BASE);

    if (ams->flash_file) {
        DeviceState *nand = qdev_new("amb-nand");

        sbd = SYS_BUS_DEVICE(nand);
        qdev_prop_set_string(nand, "flash-file", ams->flash_file);
        if (ams->bad_blocks) {
            qdev_prop_set_string(nand, "bad-blocks", ams->bad_blocks);
        }
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, AMB_FIO_BASE);
        sysbus_mmio_map(sbd, 1, AMB_FDMA_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(vic, AMB_IRQ_FIO_CMD));
        sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(vic, AMB_IRQ_FIO_DMA));
        sysbus_connect_irq(sbd, 2, qdev_get_gpio_in(vic, AMB_IRQ_FDMA));
    }

    sysbus_create_simple("l2x0", AMB_L2_BASE, NULL);

    serial_mm_init(sysmem, AMB_UART0_BASE, 2,
                   qdev_get_gpio_in(vic, AMB_IRQ_UART0), 115200,
                   serial_hd(0), DEVICE_LITTLE_ENDIAN);
    memory_region_init_io(uart_ext, OBJECT(ms), &amb_uart_ext_ops, NULL,
                          "amb-uart-ext", 0x10);
    memory_region_add_subregion(sysmem, AMB_UART0_BASE + AMB_UART_US,
                                uart_ext);

    for (int i = 0; i < ARRAY_SIZE(amb_idc); i++) {
        DeviceState *idc = qdev_new("amb-idc");

        sbd = SYS_BUS_DEVICE(idc);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_mmio_map(sbd, 0, amb_idc[i].base);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(vic, amb_idc[i].irq));
        if (i == 0) {
            image_sensor_create(I2C_BUS(qdev_get_child_bus(idc, "i2c")),
                                ams->sensor ? ams->sensor : "ov4689");
            if (!ams->no_sensor_reset) {
                /* XSHUTDN is active low: a low pad holds the sensor */
                not_held = qemu_irq_invert(
                    qdev_get_gpio_in_named(idc, "bus-held", 0));
                qdev_connect_gpio_out(pads, AMB_GPIO_XSHUTDN, not_held);
                qemu_irq_lower(not_held);
            }
        }
    }

    binfo.ram_size = AMB_IAVMEM_BASE - AMB_LINUX_BASE;
    binfo.loader_start = AMB_LINUX_BASE;
    binfo.board_id = -1;
    binfo.get_dtb = amb_build_fdt;
    amb_ethernet(vic, ams->flash_file);
    amb_sd(vic, AMB_SD0_BASE, AMB_IRQ_SD0, 0);
    amb_sd(vic, AMB_SD2_BASE, AMB_IRQ_SD2, 1);

    if (ams->dsp) {
        /* A stand-in for the ORC DSP, so iav sees a coprocessor to drive. */
        DeviceState *dsp = qdev_new("amb-dsp");
        qdev_prop_set_uint32(dsp, "iavmem-base", AMB_IAVMEM_BASE);
        qdev_prop_set_uint32(dsp, "iavmem-end", ms->ram_size);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dsp), &error_fatal);
        sysbus_connect_irq(SYS_BUS_DEVICE(dsp), 0,
                           qdev_get_gpio_in(vic, AMB_IRQ_VDSP));
        sysbus_connect_irq(SYS_BUS_DEVICE(dsp), 1,
                           qdev_get_gpio_in(vic, AMB_IRQ_VCAP));
        sysbus_connect_irq(SYS_BUS_DEVICE(dsp), 2,
                           qdev_get_gpio_in(vic, AMB_IRQ_VENC));
    }

    if (ms->kernel_filename || !ams->flash_file) {
        arm_load_kernel(ARM_CPU(cpu), ms, &binfo);
    } else {
        amb_boot_from_flash(sysmem, ams->flash_file);
    }
}

static char *amb_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(AMB_MACHINE(obj)->sensor);
}

static void amb_set_sensor(Object *obj, const char *val, Error **errp)
{
    AmbMachineState *ams = AMB_MACHINE(obj);

    g_free(ams->sensor);
    ams->sensor = g_strdup(val);
}

static bool amb_get_sensor_reset(Object *obj, Error **errp)
{
    return !AMB_MACHINE(obj)->no_sensor_reset;
}

static void amb_set_sensor_reset(Object *obj, bool val, Error **errp)
{
    AMB_MACHINE(obj)->no_sensor_reset = !val;
}

static bool amb_get_dsp(Object *obj, Error **errp)
{
    return AMB_MACHINE(obj)->dsp;
}

static void amb_set_dsp(Object *obj, bool val, Error **errp)
{
    AMB_MACHINE(obj)->dsp = val;
}

static char *amb_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(AMB_MACHINE(obj)->flash_file);
}

static void amb_set_flash_file(Object *obj, const char *val, Error **errp)
{
    AmbMachineState *ams = AMB_MACHINE(obj);

    g_free(ams->flash_file);
    ams->flash_file = g_strdup(val);
}

static char *amb_get_bad_blocks(Object *obj, Error **errp)
{
    return g_strdup(AMB_MACHINE(obj)->bad_blocks);
}

static void amb_set_bad_blocks(Object *obj, const char *val, Error **errp)
{
    AmbMachineState *ams = AMB_MACHINE(obj);

    g_free(ams->bad_blocks);
    ams->bad_blocks = g_strdup(val);
}

static void amb_machine_class_init(MachineClass *mc)
{
    ObjectClass *oc = OBJECT_CLASS(mc);

    mc->desc = "Ambarella S3L (Cortex-A9)";
    mc->init = amb_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");
    mc->default_ram_size = AMB_RAM_DEFAULT;
    mc->default_ram_id = "ambarella.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "sensor", amb_get_sensor,
                                  amb_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "image sensor on I2C0, from the sensor catalogue (default ov4689); "
        "none for no sensor");
    object_class_property_add_bool(oc, "sensor-reset", amb_get_sensor_reset,
                                   amb_set_sensor_reset);
    object_class_property_set_description(oc, "sensor-reset",
        "hold the sensor off I2C until GPIO 107 is released, as on the "
        "IP4M-1026E (default on)");
    object_class_property_add_str(oc, "flash-file", amb_get_flash_file,
                                  amb_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "128 MiB of NAND page data, or 132 MiB with spare bytes; written "
        "through unless the file is read-only");
    object_class_property_add_str(oc, "bad-blocks", amb_get_bad_blocks,
                                  amb_set_bad_blocks);
    object_class_property_set_description(oc, "bad-blocks",
        "NAND blocks to mark bad, as byte offsets separated by colons");

    object_class_property_add_bool(oc, "dsp", amb_get_dsp, amb_set_dsp);
    object_class_property_set_description(oc, "dsp",
        "model a stand-in for the ORC DSP (experimental, for the encoder path)");
}

DEFINE_MACHINE_EXTENDED("s3l", MACHINE, AmbMachineState,
                        amb_machine_class_init, false, arm_machine_interfaces)
