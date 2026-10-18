/*
 * Anyka AK39xx IP-camera SoC machines.
 *
 * Table-driven like hisilicon.c, xiongmai.c and goke.c; include/hw/arm/
 * anyka.h has the family.  Boots a camera's SPI-NOR image (flash-file=)
 * the way the mask ROM does, or a kernel with -kernel.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/anyka.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/ak-l2.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "target/arm/cpu.h"

#define AK_SYSCTRL_BASE     0x08000000
#define AK_SPI0_BASE        0x20120000
#define AK_SPI1_BASE        0x20128000
#define AK_UART0_BASE       0x20130000
#define AK_UART1_BASE       0x20138000
#define AK_L2_CTRL_BASE     0x20140000
#define AK_GPIO_BASE        0x20170000
#define AK_USB_BASE         0x20200000
#define AK_DDRC_BASE        0x21000000
#define AK_MAC_BASE         0x20300000
#define AK_L2_MEM_BASE      0x48000000
#define AK_DDR_BASE         0x80000000

#define AK_IRQ_SPI0         8
#define AK_IRQ_SPI1         9
#define AK_IRQ_UART0        10
#define AK_IRQ_UART1        11
#define AK_IRQ_L2           12
#define AK_IRQ_GPIO         15
#define AK_IRQ_MAC          16

/* arch/arm/tools/mach-types: anyka_ak39xx, every board file uses it. */
#define AK_MACH_AK39XX      3892

/*
 * Blocks with no model: plain storage, so the vendor drivers' read-modify-
 * writes and probes find their registers (mach-ak39 map.h, the EV330 DTS).
 */
static const struct {
    const char *name;
    hwaddr base;
    uint32_t size;
} ak_regbanks[] = {
    { "ak-isp",     0x20000000, 0x20000 },
    { "ak-venc",    0x20020000, 0x10000 },
    { "ak-dac",     0x20110000, 0x8000 },
    { "ak-adc",     0x20118000, 0x8000 },
    { "ak-irda",    0x20160000, 0x10000 },
    { "ak-encrypt", 0x20180000, 0x10000 },
    { "ak-mipi",    0x20400000, 0x100000 },
};

/*
 * AK3918EV200: 64 MiB DDR2.  Its boards report one of two IDs (the 4.4
 * port's mach-ak39ev330.c accepts both); 0x20150200 is the one the
 * EV200 camera dumps carry.
 */
static const AkSoCConfig ak3918ev200_soc = {
    .desc               = "Anyka AK3918EV200 (ARM926EJ-S, 64 MB DDR2)",
    .chip_id            = 0x20150200,
    .ram_size_default   = 64 * MiB,
    .num_timers         = 5,
    .flash_cs_gpio      = -1,
    .i2c_layout         = 1,
};

/* AnyCloud39EV300: mach-ak39 cpu.c AKCPU_VALUE for all three parts. */
static const AkSoCConfig ak3916ev300_soc = {
    .desc               = "Anyka AK3916EV300 (ARM926EJ-S, 64 MB DDR2)",
    .chip_id            = 0x20160100,
    .ram_size_default   = 64 * MiB,
    .num_timers         = 5,
    .flash_cs_gpio      = -1,
    .i2c_layout         = 1,
};

static const AkSoCConfig ak3918ev300_soc = {
    .desc               = "Anyka AK3918EV300 (ARM926EJ-S, 64 MB DDR2)",
    .chip_id            = 0x20160100,
    .ram_size_default   = 64 * MiB,
    .num_timers         = 5,
    .flash_cs_gpio      = -1,
    .i2c_layout         = 1,
};

static const AkSoCConfig ak3919ev300_soc = {
    .desc               = "Anyka AK3919EV300 (ARM926EJ-S, 128 MB DDR2)",
    .chip_id            = 0x20160100,
    .ram_size_default   = 128 * MiB,
    .num_timers         = 5,
    .flash_cs_gpio      = -1,
    .i2c_layout         = 1,
};

/*
 * AnyCloud V500: mach-ak39ev330.c AKCPU_VALUE, ten timers, and the boot
 * flash's chip select on GPIO 25 (the dumps' DTS: spi0 cs-gpios).
 */
static const AkSoCConfig ak3918ev330_soc = {
    .desc               = "Anyka AK3918EV330 (ARM926EJ-S, 64 MB DDR2)",
    .chip_id            = 0x20160101,
    .ram_size_default   = 64 * MiB,
    .num_timers         = 10,
    .has_efuse          = true,
    .flash_cs_gpio      = 25,
    .i2c_layout         = 2,
};

static const AkSoCConfig ak3919ev330_soc = {
    .desc               = "Anyka AK3919EV330 (ARM926EJ-S, 128 MB DDR2)",
    .chip_id            = 0x20160101,
    .ram_size_default   = 128 * MiB,
    .num_timers         = 10,
    .has_efuse          = true,
    .flash_cs_gpio      = 25,
    .i2c_layout         = 2,
};

#define TYPE_AK_MACHINE MACHINE_TYPE_NAME("anyka")
OBJECT_DECLARE_SIMPLE_TYPE(AkMachineState, AK_MACHINE)

struct AkMachineState {
    MachineState parent_obj;
    char *flash_file;
    uint32_t flash_jedec;
};

static char *ak_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(AK_MACHINE(obj)->flash_file);
}

static void ak_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    AkMachineState *s = AK_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void ak_machine_get_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &AK_MACHINE(obj)->flash_jedec, errp);
}

static void ak_machine_set_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &AK_MACHINE(obj)->flash_jedec, errp);
}

/*
 * The mask ROM's boot header, the first 512 bytes of the SPI NOR (the
 * U-Boot port's mkaksh.py, and the dumps):
 *
 *   +0x00  branch, for a ROM that enters at the header
 *   +0x04  magic: "AKSH_240" (EV200), "ANYKA_H3" (EV300), "ANYKAH3B" (EV330)
 *   +0x0C  length of what follows the header      +0x20  load address
 *   +0x2C  register script, {address, value} pairs, with markers in the
 *          address word: 0x66668888 = wait <value> (delay units),
 *          0x66668881 = poll <value> for <next word> (three words),
 *          0x88888888 = end
 *   +0x18C "SPIP" + the SPI NOR descriptor the vendor drivers use
 *
 * The ROM replays the script -- DDR controller and PLLs -- then copies
 * the payload from flash offset 0x200 to the load address and enters it.
 * EV200 / EV300 load U-Boot itself into DDR (0x80E00000, U-Boot's
 * TEXT_BASE 0x80DFFE00 being the header's address); the EV330 loads a
 * 4 KiB first stage into L2 memory, which repeats the exercise from the
 * second header at 0x200 and loads U-Boot the same way.
 */
#define AK_HDR_LEN          0x200
#define AK_HDR_SCRIPT       0x2C
#define AK_HDR_SPIP         0x18C
#define AK_SCRIPT_DELAY     0x66668888
#define AK_SCRIPT_POLL      0x66668881
#define AK_SCRIPT_END       0x88888888

#define AK_BOOTROM_SIZE     0x8000

/*
 * What stands in for the mask ROM at 0, run by the CPU out of reset like
 * the real one: store each {address, value} of the table that follows it,
 * up to a {0, 0}, then jump to the word after that.  The register script
 * has to be replayed by the CPU rather than at machine reset: the device
 * resets run after reset handlers and would undo it -- and the EV330's
 * script leaves the first stage its load parameters in timer registers.
 *
 *     add   r2, pc, #16        @ r2 = table
 *  1: ldmia r2!, {r0, r1}
 *     cmp   r0, #0
 *     strne r1, [r0]
 *     bne   1b
 *     ldr   pc, [r2]
 */
static const uint32_t ak_bootrom_stub[] = {
    0xE28F2010, 0xE8B20003, 0xE3500000, 0x15801000, 0x1AFFFFFB, 0xE592F000,
};

static bool ak_boot_header_ok(const uint8_t *img, gsize len)
{
    static const char *const magic[] = { "AKSH_240", "ANYKA_H3", "ANYKAH3B" };
    int i;

    if (len < AK_HDR_LEN) {
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(magic); i++) {
        if (!memcmp(img + 4, magic[i], 8)) {
            return true;
        }
    }
    return false;
}

static void ak_boot_rom_setup(const uint8_t *img, gsize len,
                              MachineState *machine, const char *path)
{
    uint32_t plen = ldl_le_p(img + 0x0C);
    uint32_t load = ldl_le_p(img + 0x20);
    GArray *rom = g_array_new(false, false, sizeof(uint32_t));
    bool in_ddr = load >= AK_DDR_BASE &&
                  load - AK_DDR_BASE + plen <= machine->ram_size;
    bool in_l2 = load >= AK_L2_MEM_BASE &&
                 load - AK_L2_MEM_BASE + plen <= AK_L2_MEM_SIZE;
    const uint32_t end = 0;
    uint32_t off;

    if (!plen || AK_HDR_LEN + (uint64_t)plen > len || (!in_ddr && !in_l2)) {
        error_report("anyka: %s: boot header does not fit (0x%x bytes to "
                     "0x%08x)", path, plen, load);
        exit(1);
    }
    g_array_append_vals(rom, ak_bootrom_stub, ARRAY_SIZE(ak_bootrom_stub));
    for (off = AK_HDR_SCRIPT; off + 8 <= AK_HDR_SPIP; ) {
        uint32_t a = ldl_le_p(img + off);
        uint32_t v = ldl_le_p(img + off + 4);

        if (a == AK_SCRIPT_END) {
            break;
        }
        if (a == AK_SCRIPT_POLL) {
            /* Waits for the DDR controller: done at once here. */
            off += 12;
            continue;
        }
        off += 8;
        if (a == AK_SCRIPT_DELAY || !a) {
            continue;
        }
        g_array_append_val(rom, a);
        g_array_append_val(rom, v);
    }
    g_array_append_val(rom, end);
    g_array_append_val(rom, end);
    g_array_append_val(rom, load);
    for (off = 0; off < rom->len; off++) {
        g_array_index(rom, uint32_t, off) =
            cpu_to_le32(g_array_index(rom, uint32_t, off));
    }
    rom_add_blob_fixed("anyka.bootrom", rom->data, rom->len * 4, 0);
    g_array_free(rom, true);

    rom_add_blob_fixed("anyka.boot", img + AK_HDR_LEN, plen, load);
    if (in_ddr) {
        /*
         * The ROM reads the header into L2 memory at +0x200 and leaves it
         * there: spiboot takes its SPI NOR parameters from that copy
         * (configs.h SPIBOOT_PARAM_ADDR_BASE).  U-Boot is linked with the
         * header in front of it (TEXT_BASE = load - 0x200).
         */
        rom_add_blob_fixed("anyka.boot-header-l2", img, AK_HDR_LEN,
                           AK_L2_MEM_BASE + AK_HDR_LEN);
        if (load - AK_DDR_BASE >= AK_HDR_LEN) {
            rom_add_blob_fixed("anyka.boot-header", img, AK_HDR_LEN,
                               load - AK_HDR_LEN);
        }
    }
}

/*
 * The JEDEC ID the flash answers with: flash-jedec=, else the part the boot
 * header's SPIP descriptor names -- the vendor drivers take the geometry
 * from that descriptor and expect the flash to match it.
 */
static uint32_t ak_flash_jedec(const uint8_t *img, gsize len)
{
    if (len >= AK_HDR_SPIP + 8 && !memcmp(img + AK_HDR_SPIP, "SPIP", 4)) {
        uint32_t id = ldl_le_p(img + AK_HDR_SPIP + 4);

        /* Stored as read: manufacturer in the low byte. */
        return ((id & 0xFF) << 16) | (id & 0xFF00) | ((id >> 16) & 0xFF);
    }
    return 0xC84017;            /* GD25Q64, the commonest fit */
}

static void anyka_common_init(MachineState *machine, const AkSoCConfig *c)
{
    AkMachineState *ams = AK_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *sysctrl, *l2, *dev;
    DeviceState *spi[2];
    Object *cpuobj;
    ARMCPU *cpu;
    int n;

    if (ams->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(ams->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("anyka: cannot read %s: %s", ams->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    cpuobj = object_new(machine->cpu_type);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    memory_region_add_subregion(sysmem, AK_DDR_BASE, machine->ram);

    sysctrl = qdev_new("ak-sysctrl");
    qdev_prop_set_uint32(sysctrl, "chip-id", c->chip_id);
    qdev_prop_set_uint32(sysctrl, "num-timers", c->num_timers);
    qdev_prop_set_bit(sysctrl, "has-efuse", c->has_efuse);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctrl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sysctrl), 0, AK_SYSCTRL_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(sysctrl), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(sysctrl), 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    l2 = qdev_new(TYPE_AK_L2);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(l2), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(l2), 0, AK_L2_CTRL_BASE);
    /* The SRAM and its seven mirrors fill the 64 KiB window. */
    for (n = 0; n < 8; n++) {
        sysbus_mmio_map(SYS_BUS_DEVICE(l2), 1 + n,
                        AK_L2_MEM_BASE + n * AK_L2_MEM_SIZE);
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(l2), 0,
                       qdev_get_gpio_in(sysctrl, AK_IRQ_L2));

    for (n = 0; n < 2; n++) {
        static const hwaddr base[2] = { AK_UART0_BASE, AK_UART1_BASE };
        static const int irq[2] = { AK_IRQ_UART0, AK_IRQ_UART1 };

        dev = qdev_new("ak-uart");
        qdev_prop_set_chr(dev, "chardev", serial_hd(n));
        qdev_prop_set_uint32(dev, "index", n);
        object_property_set_link(OBJECT(dev), "l2", OBJECT(l2),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(sysctrl, irq[n]));
    }

    for (n = 0; n < 2; n++) {
        static const hwaddr base[2] = { AK_SPI0_BASE, AK_SPI1_BASE };
        static const int irq[2] = { AK_IRQ_SPI0, AK_IRQ_SPI1 };
        static const uint32_t rx_dev[2] = { AK_L2_DEV_SPI0_RX,
                                            AK_L2_DEV_SPI1_RX };
        static const uint32_t tx_dev[2] = { AK_L2_DEV_SPI0_TX,
                                            AK_L2_DEV_SPI1_TX };

        spi[n] = qdev_new("ak-spi");
        qdev_prop_set_uint32(spi[n], "l2-rx-dev", rx_dev[n]);
        qdev_prop_set_uint32(spi[n], "l2-tx-dev", tx_dev[n]);
        object_property_set_link(OBJECT(spi[n]), "l2", OBJECT(l2),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(spi[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(spi[n]), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(spi[n]), 0,
                           qdev_get_gpio_in(sysctrl, irq[n]));
    }

    /* The boot SPI NOR on SPI0's chip select. */
    {
        BusState *bus = qdev_get_child_bus(spi[0], "ssi");

        dev = qdev_new("ak-spinor");
        if (ams->flash_file) {
            qdev_prop_set_string(dev, "flash-file", ams->flash_file);
        }
        qdev_prop_set_uint32(dev, "jedec", ams->flash_jedec
                             ? ams->flash_jedec
                             : ak_flash_jedec(img, img_len));
        qdev_realize_and_unref(dev, bus, &error_fatal);
        qdev_connect_gpio_out_named(spi[0], SSI_GPIO_CS, 0,
                                    qdev_get_gpio_in_named(dev, SSI_GPIO_CS,
                                                           0));
    }

    /* I2C 0, and the EV330's extra two (its DTS; harmless on the rest). */
    for (n = 0; n < 3; n++) {
        static const hwaddr base[3] = { 0x20150000, 0x20190000, 0x201A0000 };
        static const int irq[3] = { 13, 20, 29 };

        dev = qdev_new("ak-i2c");
        qdev_prop_set_uint32(dev, "layout", c->i2c_layout);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(sysctrl, irq[n]));
    }
    /* MCI0 (the SD slot, -drive if=sd) and MCI1 (SDIO Wi-Fi, empty). */
    for (n = 0; n < 2; n++) {
        DriveInfo *di = n ? NULL : drive_get(IF_SD, 0, 0);

        dev = sysbus_create_simple("ak-mci", 0x20100000 + 0x8000 * n,
                                   qdev_get_gpio_in(sysctrl, 4 + n));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }
    sysbus_create_simple("ak-usb", AK_USB_BASE, NULL);
    sysbus_create_simple("ak-ddrc", AK_DDRC_BASE, NULL);
    dev = sysbus_create_simple("ak-gpio", AK_GPIO_BASE,
                               qdev_get_gpio_in(sysctrl, AK_IRQ_GPIO));
    if (c->flash_cs_gpio >= 0) {
        qdev_connect_gpio_out(dev, c->flash_cs_gpio,
                              qdev_get_gpio_in_named(spi[0], "cs-gpio", 0));
    }

    for (n = 0; n < ARRAY_SIZE(ak_regbanks); n++) {
        DeviceState *rb = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(rb, "size", ak_regbanks[n].size);
        qdev_prop_set_string(rb, "name", ak_regbanks[n].name);
        qdev_prop_set_bit(rb, "sub-word", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, ak_regbanks[n].base);
    }
    /*
     * Ethernet: a Synopsys GMAC driven by a synopGMAC-derived driver
     * (SDK drivers/net/ethernet/ak-ethernet), normal descriptors two
     * software words apart (DmaDescriptorSkip2), an RMII PHY at MDIO
     * address 1 (the DTS phy-address).  The driver takes 4 off every
     * received length, so the core keeps the FCS.
     */
    dev = qdev_new("xm-dwmac");
    qemu_configure_nic_device(dev, true, NULL);
    qdev_prop_set_uint32(dev, "regs-size", 0x2000);
    qdev_prop_set_bit(dev, "rx-fcs", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, AK_MAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(sysctrl, AK_IRQ_MAC));
    /* The mask ROM.  Without a flash image to boot it stays empty. */
    memory_region_init_rom(bootrom, NULL, "anyka.bootrom", AK_BOOTROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, bootrom);
    create_unimplemented_device("ak-apb", 0x20000000, 0x02000000);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = AK_DDR_BASE;
    binfo.board_id = AK_MACH_AK39XX;
    arm_load_kernel(cpu, machine, &binfo);

    if (!machine->kernel_filename && img) {
        if (!ak_boot_header_ok(img, img_len)) {
            error_report("anyka: %s has no Anyka boot header",
                         ams->flash_file);
            exit(1);
        }
        ak_boot_rom_setup(img, img_len, machine, ams->flash_file);
    }
}

static void ak_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "anyka.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        ak_machine_get_flash_file, ak_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (boot header at 0); "
        "erases and programs are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        ak_machine_get_flash_jedec, ak_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with (e.g. 0xEF4017); default: the "
        "part the boot header's SPIP descriptor names");
}

static const TypeInfo ak_machine_info = {
    .name           = TYPE_AK_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(AkMachineState),
    .class_init     = ak_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void ak_machine_register_types(void)
{
    type_register_static(&ak_machine_info);
}

type_init(ak_machine_register_types)

#define DEFINE_AK_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        anyka_common_init(machine, &config);                         \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");          \
        mc->default_ram_size = config.ram_size_default;              \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, AK_MACHINE, AkMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_AK_MACHINE("ak3918ev200", ak3918ev200, ak3918ev200_soc)
DEFINE_AK_MACHINE("ak3916ev300", ak3916ev300, ak3916ev300_soc)
DEFINE_AK_MACHINE("ak3918ev300", ak3918ev300, ak3918ev300_soc)
DEFINE_AK_MACHINE("ak3919ev300", ak3919ev300, ak3919ev300_soc)
DEFINE_AK_MACHINE("ak3918ev330", ak3918ev330, ak3918ev330_soc)
DEFINE_AK_MACHINE("ak3919ev330", ak3919ev330, ak3919ev330_soc)
