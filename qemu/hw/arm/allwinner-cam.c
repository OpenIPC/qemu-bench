/*
 * Allwinner V3s (sun8iw8) and V851s (sun8iw21) camera SoCs: one Cortex-A7,
 * DDR in the package, SPI NOR on SPI0 and an SD card slot.
 *
 * Enough of each SoC to run OpenIPC's mainline U-Boot (openipc-allwinner)
 * the way the mask ROM starts it: the SPL (eGON.BT0) from SD card sector
 * 16 or, without one there, from SPI NOR offset 0, into SRAM; it brings up
 * the DDR, reads U-Boot from the same medium, and U-Boot boots the kernel
 * from the NOR.  The devices are the V831 machine's (allwinner-v83x.c) at
 * these SoCs' addresses:
 *
 *   V3s    the H3 generation: CCU 0x01C20000, UART0 0x01C28000, SPI0
 *          0x01C68000, the H3's DesignWare DRAM controller at 0x01C63000,
 *          SRAM A1 at 0 (the SPL runs where the ROM puts it), the EMAC
 *          with its PHY inside the SoC.
 *   V851s  the D1/T113 ("NCAT2") map: CCU 0x02001000, UART0 0x02500000,
 *          SPI0 0x04025000, the D1's DRAM controller at 0x03103000, SRAM
 *          A1 at 0x20000 with the ROM at 0, the EMAC at 0x04500000, the
 *          sun8iw19 DMA controller at 0x03002000 (the vendor kernel's SPI
 *          driver reads the root file system through it).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
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
#include "hw/intc/arm_gic.h"
#include "hw/net/allwinner-sun8i-emac.h"
#include "hw/loader.h"
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

#define AWC_DRAM_BASE       0x40000000
#define AWC_DRAM_WINDOW     0x80000000  /* 2 GiB, 0x40000000..0xBFFFFFFF */
#define AWC_CNTFRQ          24000000    /* the 24 MHz oscillator */
#define AWC_BOOT0_MAX       0x00018000
/* where the boot ROM looks for eGON.BT0 on an SD card: sector 16 */
#define AWC_SD_BOOT0_OFFSET 0x2000
/* eGON header: the boot ROM writes where it loaded the image from */
#define AWC_EGON_BOOT_MEDIA 0x28
#define AWC_BOOT_MEDIA_MMC0 0
#define AWC_BOOT_MEDIA_SPI  3

typedef struct AwcRegbank {
    const char *name;
    hwaddr base;
    uint32_t size;
} AwcRegbank;

typedef struct AwcSoC {
    const char *desc;
    uint64_t ram_size;
    hwaddr sram_base;           /* where the boot ROM puts the SPL */
    uint32_t sram_size;
    /*
     * ROM at 0 and the SRAM elsewhere: a stub at 0 jumps to the SPL. With
     * the SRAM at 0 (V3s, whose ROM is at 0xFFFF0000) the core starts
     * in the SPL, where the ROM would enter it.
     */
    bool brom_at_0;
    unsigned num_spi;           /* GIC shared interrupt lines */
    hwaddr gic, ccu, pio, r_pio, timer, uart0, spi0, mmc0, emac, dramc;
    hwaddr dramcom;
    /* aw-dma (0: none, SPI by PIO only) and SPI0's DRQ on it */
    hwaddr dma;
    unsigned irq_dma, drq_spi0;
    /* the DDR part's geometry (aw-dramcom) */
    uint32_t dram_row_bits, dram_bank_bits, dram_page;
    unsigned irq_timer0, irq_wdt, irq_uart0, irq_spi0, irq_mmc0, irq_emac;
    unsigned num_uarts;
    const char *sdhost;
    uint8_t phy_addr;
    uint8_t ccu_gen;            /* aw-ccu: 0 the H6 layout, 1 the sun6i one */
    AwcRegbank regbanks[8];
} AwcSoC;

/* V3s, sun8i-v3s.dtsi and the V3s datasheet; 64 MiB DDR2 in the package */
static const AwcSoC v3s_soc = {
    .desc       = "Allwinner V3s (Cortex-A7, 64 MB DDR2)",
    .ram_size   = 64 * MiB,
    .sram_base  = 0x00000000,
    .sram_size  = 0x00010000,   /* SRAM A1 (and what follows, for SRAM C) */
    .brom_at_0  = false,
    .num_spi    = 128,
    .gic        = 0x01C80000,
    .ccu        = 0x01C20000,
    .pio        = 0x01C20800,
    .r_pio      = 0,
    .timer      = 0x01C20C00,
    .uart0      = 0x01C28000,
    .spi0       = 0x01C68000,
    .mmc0       = 0x01C0F000,
    .emac       = 0x01C30000,
    .dramc      = 0x01C63000,
    .dramcom    = 0x01C62000,
    /* 512 Mbit x16 DDR2: 8K rows, 4 banks, 1K columns */
    .dram_row_bits  = 13,
    .dram_bank_bits = 2,
    .dram_page      = 2048,
    .irq_timer0 = 18,
    .irq_wdt    = 25,
    .irq_uart0  = 0,
    .irq_spi0   = 65,
    .irq_mmc0   = 60,
    .irq_emac   = 82,
    .num_uarts  = 3,
    .sdhost     = TYPE_AW_SDHOST_SUN5I,
    .phy_addr   = 1,            /* the internal PHY, int_mii_phy reg 1 */
    .ccu_gen    = 1,
    .regbanks   = {
        { "awc-syscon",     0x01C00000, 0x1000 },
        { "awc-sid",        0x01C14000, 0x0400 },
        { "awc-rtc",        0x01C20400, 0x0400 },
        { "awc-dram-phy",   0x01C65000, 0x1000 },
    },
};

/*
 * V851s, the D1/T113 map (sunxi-d1s-t113.dtsi, interrupts there numbered
 * from the first shared line); 64 MiB DDR2 in the package
 */
static const AwcSoC v851s_soc = {
    .desc       = "Allwinner V851s (Cortex-A7, 64 MB DDR2)",
    .ram_size   = 64 * MiB,
    .sram_base  = 0x00020000,
    .sram_size  = 0x00040000,   /* SRAM A1 and on, the SPL stack at 0x40000 */
    .brom_at_0  = true,
    .num_spi    = 224,
    .gic        = 0x03020000,
    .ccu        = 0x02001000,
    .pio        = 0x02000000,
    .r_pio      = 0x07022000,
    .timer      = 0x02050000,
    .uart0      = 0x02500000,
    .spi0       = 0x04025000,
    .mmc0       = 0x04020000,
    .emac       = 0x04500000,
    .dramc      = 0x03103000,
    .dramcom    = 0x03102000,
    .dram_row_bits  = 13,
    .dram_bank_bits = 2,
    .dram_page      = 2048,
    /*
     * The sun8iw19 DMA controller: the vendor SPI driver moves anything
     * past the FIFO by DMA. SPI0's DRQ is 22, as on the V831 and the D1.
     */
    .dma        = 0x03002000,
    .irq_dma    = 50,
    .drq_spi0   = 22,
    .irq_timer0 = 59,
    .irq_wdt    = 63,
    .irq_uart0  = 2,
    .irq_spi0   = 15,
    .irq_mmc0   = 40,
    .irq_emac   = 46,
    .num_uarts  = 4,
    .sdhost     = TYPE_AW_SDHOST_SUN50I_A64,
    .phy_addr   = 0,
    .regbanks   = {
        { "awc-sramc",      0x03000000, 0x1000 },
        { "awc-sid",        0x03006000, 0x1000 },
        { "awc-rtc",        0x07090000, 0x0400 },
        { "awc-prcm",       0x07010000, 0x0400 },
        { "awc-r-cpucfg",   0x07000400, 0x0400 },
    },
};

#define TYPE_AWC_MACHINE MACHINE_TYPE_NAME("allwinner-cam")
OBJECT_DECLARE_SIMPLE_TYPE(AwcMachineState, AWC_MACHINE)

struct AwcMachineState {
    MachineState parent_obj;
    char *flash_file;
    uint32_t flash_jedec;
};

static char *awc_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(AWC_MACHINE(obj)->flash_file);
}

static void awc_set_flash_file(Object *obj, const char *value, Error **errp)
{
    AwcMachineState *s = AWC_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void awc_get_flash_jedec(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &AWC_MACHINE(obj)->flash_jedec, errp);
}

static void awc_set_flash_jedec(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &AWC_MACHINE(obj)->flash_jedec, errp);
}

/* The DW APB 16550's extras past the eight registers (as on the V831) */
static uint64_t awc_uart_ext_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr + 0x20) {
    case 0x7C:                  /* USR: not busy, TX FIFO empty, not full */
        return 0x6;
    case 0xF8:                  /* UCV */
        return 0x3430322A;
    case 0xFC:                  /* CTR, "DW" */
        return 0x44570110;
    }
    return 0;
}

static void awc_uart_ext_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
}

static const MemoryRegionOps awc_uart_ext_ops = {
    .read = awc_uart_ext_read,
    .write = awc_uart_ext_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static bool awc_boot0_ok(const uint8_t *img, gsize len, uint32_t *plen)
{
    if (len < 0x200 || memcmp(img + 4, "eGON.BT0", 8)) {
        return false;
    }
    *plen = ldl_le_p(img + 0x10);
    return *plen >= 0x200 && *plen <= AWC_BOOT0_MAX && *plen <= len;
}

static void awc_common_init(MachineState *machine, const AwcSoC *c)
{
    AwcMachineState *ams = AWC_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    g_autofree uint8_t *sdimg = NULL;
    gsize img_len = 0;
    qemu_irq *pic = g_new0(qemu_irq, c->num_spi);
    static struct arm_boot_info binfo;
    DeviceState *gic, *cpudev, *dev, *spi;
    SysBusDevice *gicbus;
    DriveInfo *sd = drive_get(IF_SD, 0, 0);
    Object *cpuobj;
    ARMCPU *cpu;
    uint8_t *boot0 = NULL;
    uint8_t media = AWC_BOOT_MEDIA_SPI;
    uint32_t plen = 0;
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

    cpuobj = object_new(machine->cpu_type);
    object_property_set_bool(cpuobj, "has_el3", true, &error_fatal);
    object_property_set_uint(cpuobj, "cntfrq", AWC_CNTFRQ, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);
    cpudev = DEVICE(cpu);

    memory_region_init_ram(sram, NULL, "awc.sram", c->sram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, c->sram_base, sram);
    if (c->brom_at_0) {
        MemoryRegion *brom = g_new(MemoryRegion, 1);

        memory_region_init_rom(brom, NULL, "awc.brom", 0x10000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, brom);
    }

    /* The controller decodes the part's address bits: the DDR repeats. */
    memory_region_add_subregion(sysmem, AWC_DRAM_BASE, machine->ram);
    for (n = 1; (uint64_t)n * machine->ram_size < AWC_DRAM_WINDOW; n++) {
        MemoryRegion *mirror = g_new(MemoryRegion, 1);
        g_autofree char *name = g_strdup_printf("awc.dram-mirror%d", n);

        memory_region_init_alias(mirror, NULL, name, machine->ram, 0,
                                 machine->ram_size);
        memory_region_add_subregion(sysmem,
                                    AWC_DRAM_BASE + n * machine->ram_size,
                                    mirror);
    }

    gic = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gic, "revision", 2);
    qdev_prop_set_uint32(gic, "num-cpu", 1);
    qdev_prop_set_uint32(gic, "num-irq", c->num_spi + GIC_INTERNAL);
    qdev_prop_set_bit(gic, "has-security-extensions", true);
    gicbus = SYS_BUS_DEVICE(gic);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    sysbus_mmio_map(gicbus, 0, c->gic + 0x1000);
    sysbus_mmio_map(gicbus, 1, c->gic + 0x2000);
    {
        int ppibase = c->num_spi + GIC_NR_SGIS;
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
    for (n = 0; n < c->num_spi; n++) {
        pic[n] = qdev_get_gpio_in(gic, n);
    }

    for (n = 0; n < c->num_uarts; n++) {
        hwaddr base = c->uart0 + n * 0x400;
        MemoryRegion *ext = g_new(MemoryRegion, 1);

        serial_mm_init(sysmem, base, 2, pic[c->irq_uart0 + n],
                       AWC_CNTFRQ / 16, serial_hd(n), DEVICE_LITTLE_ENDIAN);
        memory_region_init_io(ext, NULL, &awc_uart_ext_ops, NULL,
                              "awc-uart-ext", 0x400 - 0x20);
        memory_region_add_subregion(sysmem, base + 0x20, ext);
    }

    for (n = 0; n < ARRAY_SIZE(c->regbanks) && c->regbanks[n].name; n++) {
        DeviceState *rb = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
        qdev_prop_set_string(rb, "name", c->regbanks[n].name);
        qdev_prop_set_bit(rb, "sub-word", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);
    }

    /* SPI0 and the boot NOR on its CS0; its DRQs on the DMA controller */
    spi = qdev_new("aw-spi");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(spi), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(spi), 0, c->spi0);
    sysbus_connect_irq(SYS_BUS_DEVICE(spi), 0, pic[c->irq_spi0]);
    if (c->dma) {
        DeviceState *dma = sysbus_create_simple("aw-dma", c->dma,
                                                pic[c->irq_dma]);

        qdev_connect_gpio_out_named(spi, "drq", 0,
            qdev_get_gpio_in_named(dma, "drq-src", c->drq_spi0));
        qdev_connect_gpio_out_named(spi, "drq", 1,
            qdev_get_gpio_in_named(dma, "drq-dst", c->drq_spi0));
    }
    if (ams->flash_file) {
        dev = qdev_new("ak-spinor");
        qdev_prop_set_string(dev, "flash-file", ams->flash_file);
        qdev_prop_set_uint32(dev, "size", img_len);
        qdev_prop_set_uint32(dev, "jedec", ams->flash_jedec);
        qdev_realize_and_unref(dev, qdev_get_child_bus(spi, "ssi"),
                               &error_fatal);
        qdev_connect_gpio_out_named(spi, SSI_GPIO_CS, 0,
                                    qdev_get_gpio_in_named(dev, SSI_GPIO_CS,
                                                           0));
    }

    /* Timers: the A10 block; the watchdog is at its 0xA0 */
    dev = qdev_new("allwinner-A10-timer");
    qdev_prop_set_uint32(dev, "clk0-freq", 32768);
    qdev_prop_set_uint32(dev, "clk1-freq", AWC_CNTFRQ);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->timer);
    for (n = 0; n < 2; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), n, pic[c->irq_timer0 + n]);
    }
    dev = qdev_new("aw-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, c->timer + 0xA0, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[c->irq_wdt]);

    /* SD/MMC 0, the card slot (-drive if=sd) */
    dev = qdev_new(c->sdhost);
    object_property_set_link(OBJECT(dev), "dma-memory", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->mmc0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[c->irq_mmc0]);
    if (sd) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);

        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(sd),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                               &error_fatal);
    }

    /* Ethernet: upstream's H3 EMAC, with its PHY */
    dev = qdev_new(TYPE_AW_SUN8I_EMAC);
    qemu_configure_nic_device(dev, true, NULL);
    qdev_prop_set_uint8(dev, "phy-addr", c->phy_addr);
    object_property_set_link(OBJECT(dev), "dma-memory", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->emac);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[c->irq_emac]);

    sysbus_create_simple("aw-pio", c->pio, NULL);
    if (c->r_pio) {
        sysbus_create_simple("aw-pio", c->r_pio, NULL);
    }
    /*
     * aw-ccu is the V831's 4 KiB; the V3s's CCU is 1 KiB, with the RTC,
     * PIO and timers in the rest of that page: under them
     */
    dev = qdev_new("aw-ccu");
    qdev_prop_set_uint8(dev, "gen", c->ccu_gen);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, c->ccu, -1);
    /* PIR, PGSR0, STATR: the same offsets on the DW and the D1 PHY */
    sysbus_create_simple("aw-dramc", c->dramc, NULL);
    dev = qdev_new("aw-dramcom");
    object_property_set_link(OBJECT(dev), "ram", OBJECT(machine->ram),
                             &error_fatal);
    qdev_prop_set_uint32(dev, "row-bits", c->dram_row_bits);
    qdev_prop_set_uint32(dev, "bank-bits", c->dram_bank_bits);
    qdev_prop_set_uint32(dev, "page", c->dram_page);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->dramcom);
    create_unimplemented_device("awc-periph", 0x01000000, 0x09000000);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AWC_DRAM_BASE;
    arm_load_kernel(cpu, machine, &binfo);
    if (machine->kernel_filename) {
        return;
    }

    /*
     * The boot ROM: the SD card first (eGON.BT0 at sector 16), then the
     * SPI NOR; the medium goes into the header's boot_media word.
     */
    if (sd) {
        sdimg = g_malloc(AWC_BOOT0_MAX);
        if (blk_pread(blk_by_legacy_dinfo(sd), AWC_SD_BOOT0_OFFSET,
                      AWC_BOOT0_MAX, sdimg, 0) >= 0 &&
            awc_boot0_ok(sdimg, AWC_BOOT0_MAX, &plen)) {
            boot0 = sdimg;
            media = AWC_BOOT_MEDIA_MMC0;
        }
    }
    if (!boot0 && img) {
        if (!awc_boot0_ok(img, img_len, &plen)) {
            error_report("allwinner: %s has no eGON.BT0 header at 0",
                         ams->flash_file);
            exit(1);
        }
        boot0 = img;
    }
    if (!boot0) {
        return;
    }
    if (plen > c->sram_size) {
        error_report("allwinner: the eGON.BT0 image is %u bytes, over the "
                     "%u bytes of SRAM it is loaded into", plen, c->sram_size);
        exit(1);
    }
    boot0[AWC_EGON_BOOT_MEDIA] = media;
    rom_add_blob_fixed("awc.boot0", boot0, plen, c->sram_base);
    if (c->brom_at_0) {
        /* ldr pc, [pc, #-4]; .word sram */
        uint32_t stub[2] = { cpu_to_le32(0xE51FF004),
                             cpu_to_le32(c->sram_base) };

        rom_add_blob_fixed("awc.brom-stub", stub, sizeof(stub), 0);
    }
}

static void awc_machine_instance_init(Object *obj)
{
    AWC_MACHINE(obj)->flash_jedec = 0xC84017;
}

static void awc_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "awc.dram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        awc_get_flash_file, awc_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image (eGON.BT0 at 0); erases and programs are written "
        "back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        awc_get_flash_jedec, awc_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with (default 0xC84017, GD25Q64)");
}

static const TypeInfo awc_machine_info = {
    .name           = TYPE_AWC_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(AwcMachineState),
    .instance_init  = awc_machine_instance_init,
    .class_init     = awc_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void awc_machine_register_types(void)
{
    type_register_static(&awc_machine_info);
}

type_init(awc_machine_register_types)

#define DEFINE_AWC_MACHINE(namestr, tag, config)                     \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        awc_common_init(machine, &config);                           \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");       \
        mc->default_ram_size = config.ram_size;                      \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, AWC_MACHINE, AwcMachineState,   \
                            tag##_class_init, false,                 \
                            arm_machine_interfaces)

DEFINE_AWC_MACHINE("v3s", v3s, v3s_soc)
DEFINE_AWC_MACHINE("v851s", v851s, v851s_soc)
