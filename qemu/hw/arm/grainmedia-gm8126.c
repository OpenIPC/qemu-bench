/*
 * Grain Media GM8126 machine ("gm8126"): the generation before the GM813x
 * of grainmedia.c, with the same FA626TE and Faraday peripherals at other
 * addresses, around an FTINTC010.
 *
 * Bases and interrupts from the public SDK trees' platform-GM8126 headers
 * (arch/arm/mach-GM), checked against a camera's kernel:
 *
 *   0x90000000 AHBC / 0x99000000 PMU                hw/misc/gm-sys8126.c
 *   0x90B00000 / 0x90C00000 FTCAP020 capture        hw/misc/gm-cap020.c
 *   0x92200000 USB OTG                              hw/usb/gm-otg.c
 *   0x92400000 FTSCAL010 scaler                     hw/misc/gm-scal010.c
 *   0x92500000 FTMAC110 Ethernet                    hw/net/gm-ftmac110.c
 *   0x92600000 FTDMAC020                            hw/dma/gm-dmac.c
 *   0x92700000 FTAES020                             hw/misc/fh-aes.c
 *   0x92800000 FTSDC010, no card                    hw/sd/gm-sdc010.c
 *   0x94000000 MCP200 H.264 + 0x94100000 wrapper    hw/misc/gm-h264e.c
 *   0x94300000 MCP100's local memory (nsboot runs here), its mailbox at
 *              +0x8000, registers at +0x20000, 0x94200000 wrapper
 *                                                   hw/misc/gm-mcp100.c
 *   0x98200000 FTSSP010, the NOR flash (select: GPIO0 pin 14)
 *                                                   hw/ssi/gm-ssp-spi.c
 *   0x98300000 ... UART0-4
 *   0x98D00000 SAR ADC                              hw/misc/gm-saradc.c
 *   0x99100000 FTTMR010   0x99200000 FTWDT010   0x99300000 FTDDRC010
 *   0x99400000 / 0x99700000 / 0x99800000 FTGPIO010
 *   0x99500000 FTINTC010                            hw/intc/gm-intc010.c
 *   0x99600000 FTI2C010   0x99900000 RTC
 *
 * ISP210, LCD, TV encoder, NAND, the deinterlacer and audio are not
 * modelled: that space is storage that reads as zero.
 *
 * Boot: the NOR holds every 32-bit word byte-swapped -- the SSP shifts
 * 32-bit frames most significant byte first, which is how the loaders
 * read it.  Swapped back it starts with a system header ("GM8126", the
 * addresses of the three images, 0x55AA at 0xFE); each image has a
 * 256-byte header (magic 0x805A474D, "MGZ"; the size at +8; a name).  The
 * mask ROM copies nsboot (0x1000) to 0x94300000 and runs it; nsboot sets
 * up the DDR, remaps it to 0 and loads the next stage.  A dump read
 * through U-Boot is the swapped-back view: qemu-boot/gm8126-flash.py
 * turns one into the other.
 *
 * -kernel boots a uImage / zImage with ATAGs at 0x100, machine 758.  The
 * FA626TE's TCM region registers (CP15 c9, c1), which the 2.6.28 kernel
 * reads and QEMU's arm926 lacks, read as "no TCM".
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "block/block-common.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/image-sensor.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"

#define GM8126_MACH         758
#define GM8126_MIDR         0x66056261
#define GM8126_SRAM         0x94300000
#define GM8126_SRAM_SIZE    0x20000
#define GM8126_DDR_BOOT     0x10000000  /* DDR before the remap */
#define GM8126_CS_PIN       14
#define GM8126_IMG_MAGIC    0x805A474D
#define GM8126_IMG_HDR      0x100
#define GM8126_NSBOOT       0x1000
#define GM8126_NSBOOT_MAX   0x8000      /* the mailbox's offset */

#define TYPE_GM8126_MACHINE MACHINE_TYPE_NAME("gm8126")
OBJECT_DECLARE_SIMPLE_TYPE(Gm8126MachineState, GM8126_MACHINE)

struct Gm8126MachineState {
    MachineState parent_obj;
    char *flash_file;
    char *flash_type;
    char *sensor;
};

static const struct {
    hwaddr base;
    int irq;
} gm8126_uarts[] = {
    { 0x98300000, 9 }, { 0x98400000, 10 }, { 0x98500000, 20 },
    { 0x98800000, 21 }, { 0x98900000, 22 },
}, gm8126_gpios[] = {
    { 0x99400000, 13 }, { 0x99700000, 17 }, { 0x99800000, 25 },
}, gm8126_caps[] = {
    { 0x90B00000, 32 }, { 0x90C00000, 33 },
};

#define GM8126_STR_PROP(field)                                              \
static char *gm8126_get_##field(Object *obj, Error **errp)                  \
{                                                                           \
    return g_strdup(GM8126_MACHINE(obj)->field);                            \
}                                                                           \
static void gm8126_set_##field(Object *obj, const char *value, Error **errp)\
{                                                                           \
    Gm8126MachineState *s = GM8126_MACHINE(obj);                            \
                                                                            \
    g_free(s->field);                                                       \
    s->field = g_strdup(value);                                             \
}

GM8126_STR_PROP(flash_file)
GM8126_STR_PROP(flash_type)
GM8126_STR_PROP(sensor)

/* the mask ROM: nsboot to the SRAM, a jump to it at 0 */
static void gm8126_boot_rom(const uint8_t *img, gsize len, const char *path)
{
    const uint32_t stub[2] = { cpu_to_le32(0xE51FF004),
                               cpu_to_le32(GM8126_SRAM) };
    g_autofree uint8_t *ns = NULL;
    uint32_t size, i;

    if (len < GM8126_NSBOOT + GM8126_IMG_HDR) {
        error_report("gm8126: %s is too small for a NOR image", path);
        exit(1);
    }
    if (!memcmp(img, "GM81", 4)) {
        error_report("gm8126: %s is U-Boot's view of the flash; convert it "
                     "with qemu-boot/gm8126-flash.py", path);
        exit(1);
    }
    if (ldl_be_p(img + GM8126_NSBOOT) != GM8126_IMG_MAGIC) {
        error_report("gm8126: %s has no nsboot at 0x%x", path,
                     GM8126_NSBOOT);
        exit(1);
    }
    size = ROUND_UP(ldl_be_p(img + GM8126_NSBOOT + 8), 4);
    if (!size || size > GM8126_NSBOOT_MAX ||
        size > len - GM8126_NSBOOT - GM8126_IMG_HDR) {
        error_report("gm8126: %s: bad nsboot size 0x%x", path, size);
        exit(1);
    }
    ns = g_malloc(size);
    for (i = 0; i < size; i += 4) {
        stl_le_p(ns + i, ldl_be_p(img + GM8126_NSBOOT + GM8126_IMG_HDR + i));
    }
    rom_add_blob_fixed("gm8126.nsboot", ns, size, GM8126_SRAM);
    rom_add_blob_fixed("gm8126.bootrom", stub, sizeof(stub), 0);
}

static const char *gm8126_flash_type(gsize len)
{
    switch (len) {
    case 8 * MiB:   return "mx25l6405d";
    case 32 * MiB:  return "mx25l25635e";
    default:        return "mx25l12805d";
    }
}

static void gm8126_flash(Gm8126MachineState *gms, DeviceState *spi,
                         DeviceState *gpio, gsize len)
{
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    BlockBackend *blk = NULL;
    DeviceState *dev;

    if (gms->flash_file) {
        QDict *opts = qdict_new();
        Error *err = NULL;

        qdict_put_str(opts, "driver", "raw");
        blk = blk_new_open(gms->flash_file, NULL, opts, BDRV_O_RDWR, &err);
        if (!blk) {
            error_reportf_err(err, "gm8126: %s: ", gms->flash_file);
            exit(1);
        }
    } else if (di) {
        blk = blk_by_legacy_dinfo(di);
        len = blk_getlength(blk);
    }
    dev = qdev_new(gms->flash_type ? gms->flash_type
                                   : gm8126_flash_type(len));
    if (blk) {
        qdev_prop_set_drive_err(dev, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(dev, qdev_get_child_bus(spi, "ssi"), &error_fatal);
    if (gms->flash_file) {
        blk_unref(blk);     /* see gm_create_flash() in grainmedia.c */
    }
    qdev_connect_gpio_out(gpio, GM8126_CS_PIN,
                          qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0));
}

/* the SDK's uImages run in place: 64 bytes past the load address */
static char *gm8126_fixup_uimage(const char *path)
{
    g_autofree uint8_t *buf = NULL;
    gsize len;
    char *tmp;
    int fd;

    if (!g_file_get_contents(path, (gchar **)&buf, &len, NULL) || len < 64 ||
        ldl_be_p(buf) != 0x27051956 ||
        ldl_be_p(buf + 20) != ldl_be_p(buf + 16) + 64) {
        return g_strdup(path);
    }
    stl_be_p(buf + 16, ldl_be_p(buf + 20));
    fd = g_file_open_tmp("gm8126-uimage-XXXXXX", &tmp, NULL);
    if (fd < 0 || !g_file_set_contents(tmp, (gchar *)buf, len, NULL)) {
        error_report("gm8126: cannot write a copy of %s", path);
        exit(1);
    }
    close(fd);
    return tmp;
}

static DeviceState *gm8126_dev(const char *type, hwaddr base, qemu_irq irq)
{
    DeviceState *dev = qdev_new(type);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    if (irq) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }
    return dev;
}

static void gm8126_init(MachineState *machine)
{
    static const ARMCPRegInfo tcm = {
        .name = "TCMREGION", .cp = 15, .crn = 9, .crm = 1, .opc1 = 0,
        .opc2 = CP_ANY, .access = PL1_RW,
        .type = ARM_CP_CONST | ARM_CP_OVERRIDE, .resetvalue = 0,
    };
    /* the FA626's own control register, which the burn-in image sets up */
    static const ARMCPRegInfo ecr = {
        .name = "FA626_ECR", .cp = 15, .crn = 1, .crm = 0, .opc1 = 0,
        .opc2 = 1, .access = PL1_RW, .type = ARM_CP_CONST | ARM_CP_OVERRIDE,
        .resetvalue = 0,
    };
    static struct arm_boot_info binfo;
    Gm8126MachineState *gms = GM8126_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *low = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *intc, *sys, *dev, *spi, *gpio0 = NULL;
    Object *cpu;
    int n;

    if (machine->ram_size != 64 * MiB && machine->ram_size != 128 * MiB &&
        machine->ram_size != 256 * MiB) {
        error_report("gm8126: DDR must be 64, 128 or 256 MiB");
        exit(1);
    }
    if (gms->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(gms->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("gm8126: cannot read %s: %s", gms->flash_file,
                         gerr->message);
            exit(1);
        }
    }

    cpu = object_new(machine->cpu_type);
    object_property_set_uint(cpu, "midr", GM8126_MIDR, &error_fatal);
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);
    define_one_arm_cp_reg(ARM_CPU(cpu), &tcm);
    define_one_arm_cp_reg(ARM_CPU(cpu), &ecr);

    memory_region_add_subregion(sysmem, 0, machine->ram);
    memory_region_init_alias(low, NULL, "gm8126.ddr-boot", machine->ram, 0,
                             machine->ram_size);
    memory_region_add_subregion_overlap(sysmem, GM8126_DDR_BOOT, low, -1);
    memory_region_init_ram(sram, NULL, "gm8126.sram", GM8126_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, GM8126_SRAM, sram);
    create_unimplemented_device("gm8126.periph", 0x90000000, 0x10000000);

    intc = gm8126_dev("gm-intc010", 0x99500000, NULL);
    sysbus_connect_irq(SYS_BUS_DEVICE(intc), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(intc), 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));
#define IRQ(i) qdev_get_gpio_in(intc, i)

    sys = qdev_new("gm-sys8126");
    qdev_prop_set_uint64(sys, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sys), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sys), 0, 0x99000000);
    sysbus_mmio_map(SYS_BUS_DEVICE(sys), 1, 0x90000000);
    sysbus_connect_irq(SYS_BUS_DEVICE(sys), 0, IRQ(8));

    dev = qdev_new("gm-ddrc");
    qdev_prop_set_uint64(dev, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x99300000);

    dev = qdev_new("gm-timer");
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(sys, "pclk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x99100000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(14));
    dev = qdev_new("gm-wdt");
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(sys, "pclk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x99200000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(16));

    for (n = 0; n < ARRAY_SIZE(gm8126_uarts); n++) {
        serial_mm_init(sysmem, gm8126_uarts[n].base, 2,
                       IRQ(gm8126_uarts[n].irq), 30000000 / 16 / 16,
                       serial_hd(n),
                       DEVICE_LITTLE_ENDIAN);
    }
    for (n = 0; n < ARRAY_SIZE(gm8126_gpios); n++) {
        dev = gm8126_dev("gm-gpio", gm8126_gpios[n].base,
                         IRQ(gm8126_gpios[n].irq));
        gpio0 = gpio0 ? gpio0 : dev;
    }
    spi = gm8126_dev("gm-ssp-spi", 0x98200000, IRQ(6));
    gm8126_flash(gms, spi, gpio0, img_len);

    dev = gm8126_dev("gm-dmac", 0x92600000, IRQ(1));
    qdev_connect_gpio_out_named(spi, "dma-rx", 0,
                                qdev_get_gpio_in_named(dev, "dma-req", 0));
    qdev_connect_gpio_out_named(spi, "dma-tx", 0,
                                qdev_get_gpio_in_named(dev, "dma-req", 1));
    gm8126_dev("gm-otg", 0x92200000, IRQ(4));
    gm8126_dev("gm-sdc010", 0x92800000, IRQ(15));
    gm8126_dev("gm-rtc", 0x99900000, IRQ(5));
    dev = qdev_new("gm-saradc");
    object_property_add_child(OBJECT(machine), "saradc", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x98D00000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(40));
    dev = qdev_new("fh-aes");
    qdev_prop_set_bit(dev, "advance", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x92700000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(19));

    dev = qdev_new("gm-ftmac110");
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x92500000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(3));

    for (n = 0; n < ARRAY_SIZE(gm8126_caps); n++) {
        gm8126_dev("gm-cap020", gm8126_caps[n].base, IRQ(gm8126_caps[n].irq));
    }
    gm8126_dev("gm-scal010", 0x92400000, IRQ(12));
    dev = qdev_new("gm-h264e");
    qdev_prop_set_bit(dev, "mcp200", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x94000000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x94100000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(29));
    dev = qdev_new("gm-mcp100");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GM8126_SRAM + 0x20000);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, GM8126_SRAM + 0x8000, 1);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, 0x94200000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, IRQ(31));

    dev = gm8126_dev("gm-i2c", 0x99600000, IRQ(18));
    image_sensor_create(I2C_BUS(qdev_get_child_bus(dev, "i2c")),
                        gms->sensor);

    binfo.ram_size = machine->ram_size;
    binfo.board_id = GM8126_MACH;
    if (machine->kernel_filename) {
        char *orig = machine->kernel_filename;
        char *path = gm8126_fixup_uimage(orig);

        machine->kernel_filename = path;
        arm_load_kernel(ARM_CPU(cpu), machine, &binfo);
        machine->kernel_filename = orig;
        if (strcmp(path, orig)) {
            unlink(path);
        }
        g_free(path);
    } else {
        arm_load_kernel(ARM_CPU(cpu), machine, &binfo);
        if (img) {
            gm8126_boot_rom(img, img_len, gms->flash_file);
        }
    }
}

static void gm8126_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Grain Media GM8126 (FA626TE)";
    mc->init = gm8126_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "gm8126.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file", gm8126_get_flash_file,
                                  gm8126_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as on the chip (32-bit words byte-swapped); erases "
        "and programs are written back to it");
    object_class_property_add_str(oc, "flash-type", gm8126_get_flash_type,
                                  gm8126_set_flash_type);
    object_class_property_set_description(oc, "flash-type",
        "m25p80 part the NOR is modelled as; default by the image's size");
    object_class_property_add_str(oc, "sensor", gm8126_get_sensor,
                                  gm8126_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor on the I2C bus (none, ov7725); default: none");
}

static const TypeInfo gm8126_machine_info = {
    .name = TYPE_GM8126_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Gm8126MachineState),
    .class_init = gm8126_class_init,
    .interfaces = arm_machine_interfaces,
};

static void gm8126_machine_register_types(void)
{
    type_register_static(&gm8126_machine_info);
}

type_init(gm8126_machine_register_types)
