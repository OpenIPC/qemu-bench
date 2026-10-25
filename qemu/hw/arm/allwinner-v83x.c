/*
 * Allwinner V831 / V833 (sun8iw19p1) IP-camera SoC machines.
 *
 * One Cortex-A7 and a GIC-400 in the "ncat" memory map the H6 introduced
 * (CCU 0x03001000, UART 0x05000000).  Boots a camera's SPI NOR image
 * (flash-file=) the way the mask ROM does: boot0 (eGON.BT0) from flash
 * offset 0 into SRAM A1, which brings up DRAM and loads the boot package
 * (OP-TEE, U-Boot, DTB) through SPI0; OP-TEE drops to U-Boot, U-Boot
 * loads the kernel from the GPT "boot" partition.
 *
 * Addresses: the vendor U-Boot's arch-sunxi/cpu_ncat.h and the board
 * device tree in the boot package.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/allwinner-i2c.h"
#include "hw/i2c/i2c.h"
#include "hw/intc/arm_gic.h"
#include "hw/net/allwinner-sun8i-emac.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/loader.h"
#include "hw/misc/aw-iommu.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/allwinner-sdhost.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/gtimer.h"

#define AW_VE_BASE          0x01C0E000
#define AW_G2D_BASE         0x01480000
#define AW_BROM_BASE        0x00000000
#define AW_BROM_SIZE        0x00010000
#define AW_SRAM_A1_BASE     0x00010000      /* A1 + C + A2: boot0's SRAM */
#define AW_SRAM_A1_SIZE     0x00050000
#define AW_SRAM_A2_BASE     0x00100000      /* boot0's CONFIG_SYS_SRAMA2 */
#define AW_SRAM_A2_SIZE     0x0001C000

#define AW_SRAMC_BASE       0x03000000
#define AW_CCU_BASE         0x03001000
#define AW_DMA_BASE         0x03002000
#define AW_SID_BASE         0x03006000
#define AW_TIMER_BASE       0x03009000
#define AW_PIO_BASE         0x0300B000
#define AW_IOMMU_BASE       0x030F0000
#define AW_GIC_BASE         0x03020000
#define AW_GICD_BASE        (AW_GIC_BASE + 0x1000)
#define AW_GICC_BASE        (AW_GIC_BASE + 0x2000)
#define AW_DRAM_COM_BASE    0x04002000
#define AW_DRAM_CTL_BASE    0x04003000
#define AW_DRAM_PHY_BASE    0x04005000
#define AW_MMC0_BASE        0x04020000
#define AW_UART0_BASE       0x05000000
#define AW_TWI0_BASE        0x05002000
#define AW_SPI0_BASE        0x05010000
#define AW_EMAC_BASE        0x05020000
#define AW_RTC_BASE         0x07000000
#define AW_PRCM_BASE        0x07010000
#define AW_R_PIO_BASE       0x07022000
#define AW_R_TWI_BASE       0x07081400
#define AW_CPUCFG_BASE      0x09010000
#define AW_DRAM_BASE        0x40000000
#define AW_DRAM_WINDOW      0x80000000  /* 2 GiB, 0x40000000..0xBFFFFFFF */

/*
 * 192 interrupt lines (GICD_TYPER ITLinesNumber 5).  The DTS stops at SPI
 * 0x72, but OP-TEE puts the interrupts in group 1 (non-secure) only on a
 * GIC with more than 179 lines; with fewer it leaves them all secure and
 * Linux, in the normal world, never gets one.
 */
#define AW_NUM_SPI          160
#define AW_CNTFRQ           24000000    /* the 24 MHz oscillator */

#define AW_IRQ_DMA          10
#define AW_IRQ_WDT          14
#define AW_IRQ_TIMER0       60
#define AW_IRQ_VINC0        74
#define AW_IRQ_IOMMU        95
#define AW_IRQ_TWI0         41
#define AW_IRQ_VE           25
#define AW_IRQ_G2D          21
#define AW_IRQ_EMAC         27
#define AW_IRQ_MMC0         46
#define AW_IRQ_UART0        49
#define AW_IRQ_SPI0         54

#define AW_DRQ_SPI0         22          /* dma-sun8iw19.h, RX and TX */

#define AW_BOOT0_LOAD       0x00020000  /* SRAM A1, boot0.lds */
#define AW_BOOT0_MAX        0x00018000
/* where the boot ROM looks for eGON.BT0 on an SD card: sector 16 */
#define AW_SD_BOOT0_OFFSET  0x2000
/* eGON header: the boot ROM writes where it loaded the image from */
#define AW_EGON_BOOT_MEDIA  0x28
#define AW_BOOT_MEDIA_MMC0  0
#define AW_BOOT_MEDIA_SPI   3

/*
 * Blocks with no model yet: plain storage, so read-modify-writes keep
 * what was written.
 */
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} aw_regbanks[] = {
    { "aw-sramc",       AW_SRAMC_BASE,      0x1000 },
    { "aw-sid",         AW_SID_BASE,        0x1000 },
    { "aw-dram-phy",    AW_DRAM_PHY_BASE,   0x1000 },
    { "aw-rtc",         AW_RTC_BASE,        0x0400 },
    { "aw-prcm",        AW_PRCM_BASE,       0x0400 },
    { "aw-cpucfg",      AW_CPUCFG_BASE,     0x1000 },
    /* video input: ISP + VIPP scalers, CSI top/parsers, MIPI */
    { "aw-isp",         0x02100000,         0x10000 },
    { "aw-csi",         0x06600000,         0x9000 },
    { "aw-csi-mipi",    0x0660A000,         0x6000 },
};

typedef struct AwSoCConfig {
    const char *desc;
    uint64_t ram_size_default;
    /* the DDR part's geometry (aw-dramcom) */
    uint32_t dram_row_bits, dram_bank_bits, dram_page;
} AwSoCConfig;

/* V831: 64 MiB DDR2 in the package (the boot0 header's dram_type 2). */
static const AwSoCConfig v831_soc = {
    .desc               = "Allwinner V831 (Cortex-A7, 64 MB DDR2)",
    .ram_size_default   = 64 * MiB,
    /* 512 Mbit x16 DDR2: 8K rows, 4 banks, 1K columns */
    .dram_row_bits      = 13,
    .dram_bank_bits     = 2,
    .dram_page          = 2048,
};

/* V833: 128 MiB DDR3 in the package. */
static const AwSoCConfig v833_soc = {
    .desc               = "Allwinner V833 (Cortex-A7, 128 MB DDR3)",
    .ram_size_default   = 128 * MiB,
    /* 1 Gbit x16 DDR3: 8K rows, 8 banks, 1K columns */
    .dram_row_bits      = 13,
    .dram_bank_bits     = 3,
    .dram_page          = 2048,
};

#define TYPE_AW_MACHINE MACHINE_TYPE_NAME("allwinner-v83x")
OBJECT_DECLARE_SIMPLE_TYPE(AwMachineState, AW_MACHINE)

struct AwMachineState {
    MachineState parent_obj;
    char *flash_file;
    uint32_t flash_jedec;
};

static char *aw_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(AW_MACHINE(obj)->flash_file);
}

static void aw_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    AwMachineState *s = AW_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void aw_machine_get_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &AW_MACHINE(obj)->flash_jedec, errp);
}

static void aw_machine_set_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &AW_MACHINE(obj)->flash_jedec, errp);
}

/*
 * The UART is a Synopsys DW APB 16550: the eight 16550 registers, then
 * the DW extras.  USR (0x7C) is what drivers poll before touching LCR;
 * it reads not busy, TX FIFO empty and not full.
 */
static uint64_t aw_uart_ext_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr + 0x20) {
    case 0x7C:                  /* USR */
        return 0x6;
    case 0xF8:                  /* UCV, the DW component version */
        return 0x3430322A;
    case 0xFC:                  /* CTR, "DW" */
        return 0x44570110;
    }
    return 0;
}

static void aw_uart_ext_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
}

static const MemoryRegionOps aw_uart_ext_ops = {
    .read = aw_uart_ext_read,
    .write = aw_uart_ext_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * eGON.BT0 header (boot0_head.c): branch, "eGON.BT0", checksum, length
 * at +0x10.  The mask ROM copies `length` bytes from flash offset 0 into
 * SRAM A1 and enters it.
 */
static bool aw_boot0_ok(const uint8_t *img, gsize len, uint32_t *plen)
{
    if (len < 0x200 || memcmp(img + 4, "eGON.BT0", 8)) {
        return false;
    }
    *plen = ldl_le_p(img + 0x10);
    return *plen >= 0x200 && *plen <= AW_BOOT0_MAX && *plen <= len;
}

static void aw_v83x_common_init(MachineState *machine, const AwSoCConfig *c)
{
    AwMachineState *ams = AW_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *brom = g_new(MemoryRegion, 1);
    MemoryRegion *sram_a1 = g_new(MemoryRegion, 1);
    MemoryRegion *sram_a2 = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    qemu_irq pic[AW_NUM_SPI];
    static struct arm_boot_info binfo;
    DeviceState *gic, *cpudev, *dev;
    DeviceState *spi[3], *dma, *iommu;
    SysBusDevice *gicbus;
    Object *cpuobj;
    ARMCPU *cpu;
    int n;

    if (ams->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(ams->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("allwinner: cannot read %s: %s", ams->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    /*
     * boot0 runs secure SVC and enters OP-TEE in monitor mode, which
     * leaves for U-Boot in non-secure SVC.
     */
    cpuobj = object_new(machine->cpu_type);
    object_property_set_bool(cpuobj, "has_el3", true, &error_fatal);
    object_property_set_uint(cpuobj, "cntfrq", AW_CNTFRQ, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);
    cpudev = DEVICE(cpu);

    memory_region_init_rom(brom, NULL, "aw.brom", AW_BROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, AW_BROM_BASE, brom);
    memory_region_init_ram(sram_a1, NULL, "aw.sram-a1", AW_SRAM_A1_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, AW_SRAM_A1_BASE, sram_a1);
    memory_region_init_ram(sram_a2, NULL, "aw.sram-a2", AW_SRAM_A2_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, AW_SRAM_A2_BASE, sram_a2);
    memory_region_add_subregion(sysmem, AW_DRAM_BASE, machine->ram);
    /*
     * The controller decodes only the address bits the part has, so the
     * DRAM repeats across its window: the boot package is linked for
     * 0x4A000000 (U-Boot) whatever the size, and lands 32 MiB in on a
     * 64 MiB V831.
     */
    for (n = 1; (uint64_t)n * machine->ram_size < AW_DRAM_WINDOW; n++) {
        MemoryRegion *mirror = g_new(MemoryRegion, 1);
        g_autofree char *name = g_strdup_printf("aw.dram-mirror%d", n);

        memory_region_init_alias(mirror, NULL, name, machine->ram, 0,
                                 machine->ram_size);
        memory_region_add_subregion(sysmem,
                                    AW_DRAM_BASE + n * machine->ram_size,
                                    mirror);
    }

    /* GIC-400, with the security extensions OP-TEE programs. */
    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", 1);
    qdev_prop_set_uint32(gic, "num-irq", AW_NUM_SPI + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    sysbus_mmio_map(gicbus, 0, AW_GICD_BASE);
    sysbus_mmio_map(gicbus, 1, AW_GICC_BASE);
    {
        int ppibase = AW_NUM_SPI + GIC_NR_SGIS;
        /* sun8iw19p1.dtsi timer: 13 secure, 14 non-secure, 11 virt, 10 hyp */
        static const int timer_ppi[] = {
            [GTIMER_PHYS] = 14,
            [GTIMER_VIRT] = 11,
            [GTIMER_HYP]  = 10,
            [GTIMER_SEC]  = 13,
        };

        for (n = 0; n < ARRAY_SIZE(timer_ppi); n++) {
            qdev_connect_gpio_out(cpudev, n,
                                  qdev_get_gpio_in(gic, ppibase +
                                                   timer_ppi[n]));
        }
    }
    sysbus_connect_irq(gicbus, 0, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
    sysbus_connect_irq(gicbus, 1, qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
    sysbus_connect_irq(gicbus, 2, qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
    sysbus_connect_irq(gicbus, 3, qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    for (n = 0; n < AW_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    for (n = 0; n < 4; n++) {
        hwaddr base = AW_UART0_BASE + n * 0x400;
        MemoryRegion *ext = g_new(MemoryRegion, 1);

        serial_mm_init(sysmem, base, 2, pic[AW_IRQ_UART0 + n],
                       AW_CNTFRQ / 16, serial_hd(n), DEVICE_LITTLE_ENDIAN);
        memory_region_init_io(ext, NULL, &aw_uart_ext_ops, NULL,
                              "aw-uart-ext", 0x400 - 0x20);
        memory_region_add_subregion(sysmem, base + 0x20, ext);
    }

    for (n = 0; n < ARRAY_SIZE(aw_regbanks); n++) {
        DeviceState *rb = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(rb, "size", aw_regbanks[n].size);
        qdev_prop_set_string(rb, "name", aw_regbanks[n].name);
        qdev_prop_set_bit(rb, "sub-word", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, aw_regbanks[n].base);
    }
    /*
     * SPI0..2; the boot NOR on SPI0's CS0.  ak-spinor is the byte-wise
     * NOR the Anyka machines use: the Allwinner drivers too send fast-read
     * dummies as TX bytes, and it answers to any JEDEC ID.
     */
    dma = sysbus_create_simple("aw-dma", AW_DMA_BASE, pic[AW_IRQ_DMA]);
    for (n = 0; n < 3; n++) {
        spi[n] = qdev_new("aw-spi");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(spi[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(spi[n]), 0, AW_SPI0_BASE + n * 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(spi[n]), 0, pic[AW_IRQ_SPI0 + n]);
        qdev_connect_gpio_out_named(spi[n], "drq", 0,
            qdev_get_gpio_in_named(dma, "drq-src", AW_DRQ_SPI0 + n));
        qdev_connect_gpio_out_named(spi[n], "drq", 1,
            qdev_get_gpio_in_named(dma, "drq-dst", AW_DRQ_SPI0 + n));
    }
    if (ams->flash_file) {
        dev = qdev_new("ak-spinor");
        qdev_prop_set_string(dev, "flash-file", ams->flash_file);
        qdev_prop_set_uint32(dev, "size", img_len);
        qdev_prop_set_uint32(dev, "jedec", ams->flash_jedec);
        qdev_realize_and_unref(dev, qdev_get_child_bus(spi[0], "ssi"),
                               &error_fatal);
        qdev_connect_gpio_out_named(spi[0], SSI_GPIO_CS, 0,
                                    qdev_get_gpio_in_named(dev, SSI_GPIO_CS,
                                                           0));
    }

    /*
     * Timers: the A10 block, as the DTS says ("sun4i-a10-timer"); Linux
     * takes timer 1 as its clocksource.  Its 0xA0.. counter registers
     * are where this SoC has the watchdog instead.
     */
    dev = qdev_new("allwinner-A10-timer");
    qdev_prop_set_uint32(dev, "clk0-freq", 32768);
    qdev_prop_set_uint32(dev, "clk1-freq", AW_CNTFRQ);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_TIMER_BASE);
    for (n = 0; n < 2; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), n, pic[AW_IRQ_TIMER0 + n]);
    }
    dev = qdev_new("aw-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, AW_TIMER_BASE + 0xA0, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_WDT]);

    /*
     * SD/MMC 0 (the card slot, -drive if=sd) and 1 (SDIO Wi-Fi, empty):
     * the A64's SMHC, FIFO at 0x200, which "sunxi-mmc-v4p1x" is.  The
     * vendor U-Boot looks on the card for /debug_tty or /Factory and
     * leaves the kernel console on ttyS0 only if it finds one.
     */
    for (n = 0; n < 2; n++) {
        DriveInfo *di = n ? NULL : drive_get(IF_SD, 0, 0);

        dev = qdev_new(TYPE_AW_SDHOST_SUN50I_A64);
        object_property_set_link(OBJECT(dev), "dma-memory",
                                 OBJECT(sysmem), &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_MMC0_BASE + n * 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_MMC0 + n]);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    /*
     * TWI0..3: upstream's sun6i TWI for the legacy registers, and at
     * +0x200 the packet engine (aw-twi) the DTS offers with twi_drv_used;
     * this firmware's kernel still drives the legacy side.  Both share the
     * block's interrupt.  R_TWI has only the legacy part: boot0 polls it
     * for a PMU, and finds none.
     */
    for (n = 0; n < 4; n++) {
        hwaddr base = AW_TWI0_BASE + n * 0x400;
        DeviceState *orgate = qdev_new(TYPE_OR_IRQ);
        DeviceState *twi;

        qdev_prop_set_uint16(orgate, "num-lines", 2);
        qdev_realize_and_unref(orgate, NULL, &error_fatal);
        qdev_connect_gpio_out(orgate, 0, pic[AW_IRQ_TWI0 + n]);
        twi = sysbus_create_simple(TYPE_AW_I2C_SUN6I, base,
                                   qdev_get_gpio_in(orgate, 0));
        if (n == 1) {
            /*
             * The camera sensor, a GC2053 at 0x37 on TWI1 (the vendor
             * vind DT and gc2053_mipi.c): ID 0x2053 at 0xF0/0xF1.
             */
            i2c_slave_create_simple(I2C_BUS(qdev_get_child_bus(twi, "i2c")),
                                    "hisi-gc2053", 0x37);
        }
        sysbus_create_simple("aw-twi", base + 0x200,
                             qdev_get_gpio_in(orgate, 1));
    }
    sysbus_create_simple(TYPE_AW_I2C_SUN6I, AW_R_TWI_BASE, NULL);

    /*
     * Ethernet: the vendor "sunxi-gmac" is the H3's EMAC register for
     * register (sunxi_gmac_ops.c), so upstream's model, with its PHY.
     * Cameras that ship Wi-Fi only still have the block and the DT node;
     * -nic gives them a cable.
     */
    dev = qdev_new(TYPE_AW_SUN8I_EMAC);
    qemu_configure_nic_device(dev, true, NULL);
    object_property_set_link(OBJECT(dev), "dma-memory", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_EMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_EMAC]);

    /*
     * The IOMMU in front of the media masters (VE, CSI, ISP, G2D, ...), the
     * video engine, the 2D engine, and the video input's four write-back
     * DMA channels, with test frames.
     */
    iommu = sysbus_create_simple(TYPE_AW_IOMMU, AW_IOMMU_BASE,
                                 pic[AW_IRQ_IOMMU]);
    dev = qdev_new("aw-ve");
    object_property_set_link(OBJECT(dev), "iommu", OBJECT(iommu),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_VE_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_VE]);
    dev = qdev_new("aw-g2d");
    object_property_set_link(OBJECT(dev), "iommu", OBJECT(iommu),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_G2D_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_G2D]);
    for (n = 0; n < 4; n++) {
        dev = qdev_new("aw-vinc");
        object_property_set_link(OBJECT(dev), "iommu", OBJECT(iommu),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x06609000 + n * 0x200);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[AW_IRQ_VINC0 + n]);
    }

    /*
     * Audio capture: the codec's ADC (DRQ 6) and DAUDIO0/1 (I2S, DRQ 3/4),
     * recording silence at the programmed rate (aw-audio.c).
     */
    {
        static const struct {
            hwaddr base;
            bool daudio;
            int drq;
        } aw_audio[] = {
            { 0x05096000, false, 6 },
            { 0x05090000, true, 3 },
            { 0x05091000, true, 4 },
        };

        for (n = 0; n < ARRAY_SIZE(aw_audio); n++) {
            dev = qdev_new("aw-audio");
            qdev_prop_set_bit(dev, "daudio", aw_audio[n].daudio);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, aw_audio[n].base);
            qdev_connect_gpio_out_named(dev, "drq", 0,
                qdev_get_gpio_in_named(dma, "drq-src", aw_audio[n].drq));
        }
    }

    sysbus_create_simple("aw-pio", AW_PIO_BASE, NULL);
    sysbus_create_simple("aw-pio", AW_R_PIO_BASE, NULL);

    sysbus_create_simple("aw-ccu", AW_CCU_BASE, NULL);
    sysbus_create_simple("aw-dramc", AW_DRAM_CTL_BASE, NULL);
    dev = qdev_new("aw-dramcom");
    object_property_set_link(OBJECT(dev), "ram", OBJECT(machine->ram),
                             &error_fatal);
    qdev_prop_set_uint32(dev, "row-bits", c->dram_row_bits);
    qdev_prop_set_uint32(dev, "bank-bits", c->dram_bank_bits);
    qdev_prop_set_uint32(dev, "page", c->dram_page);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AW_DRAM_COM_BASE);
    create_unimplemented_device("aw-periph", 0x01000000, 0x09000000);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AW_DRAM_BASE;
    arm_load_kernel(cpu, machine, &binfo);

    if (!machine->kernel_filename) {
        /* ldr pc, [pc, #-4]; .word boot0 */
        uint32_t stub[2] = { cpu_to_le32(0xE51FF004),
                             cpu_to_le32(AW_BOOT0_LOAD) };
        DriveInfo *sd = drive_get(IF_SD, 0, 0);
        g_autofree uint8_t *sdimg = NULL;
        uint8_t *boot0 = NULL;
        uint8_t media = AW_BOOT_MEDIA_SPI;
        uint32_t plen;

        /*
         * The boot ROM tries the SD card (SMHC0) before the SPI NOR: an
         * eGON.BT0 at sector 16 of the card wins.  Like the ROM, record
         * the medium in the header's boot_media word, which mainline
         * U-Boot's SPL reads to know where to load U-Boot from.
         */
        if (sd) {
            sdimg = g_malloc(AW_BOOT0_MAX);
            if (blk_pread(blk_by_legacy_dinfo(sd), AW_SD_BOOT0_OFFSET,
                          AW_BOOT0_MAX, sdimg, 0) >= 0 &&
                aw_boot0_ok(sdimg, AW_BOOT0_MAX, &plen)) {
                boot0 = sdimg;
                media = AW_BOOT_MEDIA_MMC0;
            }
        }
        if (!boot0 && img) {
            if (!aw_boot0_ok(img, img_len, &plen)) {
                error_report("allwinner: %s has no eGON.BT0 header at 0",
                             ams->flash_file);
                exit(1);
            }
            boot0 = g_memdup2(img, plen);
            g_free(sdimg);
            sdimg = boot0;
        }
        if (boot0) {
            boot0[AW_EGON_BOOT_MEDIA] = media;
            rom_add_blob_fixed("aw.brom-stub", stub, sizeof(stub),
                               AW_BROM_BASE);
            rom_add_blob_fixed("aw.boot0", boot0, plen, AW_BOOT0_LOAD);
        }
    }
}

static void aw_machine_instance_init(Object *obj)
{
    AW_MACHINE(obj)->flash_jedec = 0xC84017;
}

static void aw_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "aw.dram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        aw_machine_get_flash_file, aw_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (eGON.BT0 at 0); "
        "erases and programs are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        aw_machine_get_flash_jedec, aw_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with (default 0xC84017, GD25Q64)");
}

static const TypeInfo aw_machine_info = {
    .name           = TYPE_AW_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(AwMachineState),
    .instance_init  = aw_machine_instance_init,
    .class_init     = aw_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void aw_machine_register_types(void)
{
    type_register_static(&aw_machine_info);
}

type_init(aw_machine_register_types)

#define DEFINE_AW_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        aw_v83x_common_init(machine, &config);                       \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");       \
        mc->default_ram_size = config.ram_size_default;              \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, AW_MACHINE, AwMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_AW_MACHINE("v831", v831, v831_soc)
DEFINE_AW_MACHINE("v833", v833, v833_soc)
