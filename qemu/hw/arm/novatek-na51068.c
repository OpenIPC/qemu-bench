/*
 * Novatek NT98321 / NT98323 (NA51068) DVR / NVR SoC machine.
 *
 * Unlike the NA51089 family in novatek.c, NA51068 is built from
 * Faraday-derived blocks (FTTMR010, FTGPIO010, FTDMAC030, SPI020, FTI2C010)
 * around a dual Cortex-A9 MPCore at 0xF9C00000.  The kernel maps three
 * windows statically (plat-na51068/hardware.h): 0xFA000000 (10 MiB),
 * 0xFC000000 and 0xFE000000 (13 MiB).  Sources: the NT9832x SDK
 * v1.02.024 (kernel 4.9.118, board trees cfg_9832{1,3}_*), its
 * NT9832x_DataSheet_Register_Map.pdf.
 *
 * -kernel needs the SDK's board tree (-dtb): cfg_98323_DDR4_4GX1_NOR_EVB's
 * nvt-na51068-evb.dts built.  flash-file= alone boots the flash through
 * novatek.c's loader stand-in and the SDK's U-Boot.  Modelled: the MPCore,
 * PL310, the clock generator, TOP and PAD, the four UARTs, the FTTMR010,
 * the watchdog, the OTP, the RTC and the hash / crypto / RSA engines
 * (NA51055's blocks, models shared with novatek.c), four FTGPIO010 banks,
 * six FTIIC010s (a 24C02 on the first), the FTDMAC030, the SDHCI, both
 * USB 2.0 hosts (nvt-usb), both SATA hosts (QEMU's AHCI), both MACs
 * (nvt-eqos), the SPI020 flash controller (nvt-flash's engine, NOR or
 * NAND), the PWM, IR receiver and thermal sensor
 * (hw/misc/nvt-na51068-misc.c), the bus glue as storage, and the second
 * core's release word.  What is left is the media blocks (VCAP, VPE, DEI,
 * OSG, codecs, displays, audio, DSP, CNN, GM2D, TSE): their windows read
 * as zero and are logged under -d unimp.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/novatek.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/ide/ahci.h"
#include "hw/ide/ahci-sysbus.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/sd/sdhci.h"
#include "hw/sysbus.h"
#include "hw/ssi/nvt-flash.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/arm-powerctl.h"

#define NA51068_MPCORE_BASE     0xF9C00000
#define NA51068_L2_BASE         0xF9C20000
#define NA51068_CG_BASE         0xFE000000
#define NA51068_TOP_BASE        0xFE030000
#define NA51068_TIMER_BASE      0xFE8C0000
#define NA51068_WDT_BASE        0xFE8A0000
#define NA51068_OTP_BASE        0xFECE0000
#define NA51068_FLASH_BASE      0xFA900000  /* SPI020 */
#define NA51068_IRQ_FLASH       32
#define NA51068_RELEASE_BASE    0xFE0B9000  /* nvt-na51068-evb.dts cpu1 */
#define NA51068_RELEASE_SIZE    0x1000
#define NA51068_SRAM_BASE       0xFE080000  /* the loader's (Memory.h) */
#define NA51068_SRAM_SIZE       0x80000

#define NA51068_IRQ_TIMER       35
#define NA51068_NUM_IRQ         256

/* The kernel's static windows, read as zero where nothing is modelled. */
static const struct {
    const char *name;
    hwaddr base;
    uint64_t size;
} na51068_windows[] = {
    { "na51068-periph1", 0xFA000000, 0x00A00000 },
    { "na51068-periph2", 0xFC000000, 0x01000000 },
    { "na51068-periph",  0xFE000000, 0x00D00000 },
    { "na51068-media",   0xF8000000, 0x01C00000 },  /* DSP, VCAP, SATA, USB */
};

/* nvt-na51068-peri.dtsi: 16550s, reg-shift 2, all at 48 MHz */
static const struct {
    hwaddr base;
    int irq;
} na51068_uarts[] = {
    { 0xFE200000, 29 },
    { 0xFE220000, 30 },
    { 0xFE240000, 56 },
    { 0xFE260000, 67 },
};

/*
 * The two SATA hosts ("novatech,nvt_sata100", NVT_SATA100_AHCI, the driver
 * NA51103 has): a one-port AHCI and its PHY window (storage).  The disks
 * are -drive if=ide,index=0/1.
 */
static const struct {
    hwaddr ahci, phy;
    int irq;
} na51068_sata[] = {
    { 0xF9E00000, 0xFDC00000, 15 },
    { 0xF9F00000, 0xFDD00000, 16 },
};

/*
 * GHC: the driver sets AE | IE and later writes GHC = 0
 * (nvt_ahci_save_initial_config()), which on QEMU's AHCI drops IE for good.
 * The silicon keeps them, as novatek.c's NA51103 notes; so does this.
 */
#define NA51068_AHCI_GHC        0x04
#define NA51068_AHCI_GHC_KEEP   0x80000002u

static uint64_t na51068_ghc_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = 0;

    memory_region_dispatch_read(opaque, NA51068_AHCI_GHC, &val, MO_32,
                                MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void na51068_ghc_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    memory_region_dispatch_write(opaque, NA51068_AHCI_GHC,
                                 val | NA51068_AHCI_GHC_KEEP, MO_32,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps na51068_ghc_ops = {
    .read = na51068_ghc_read,
    .write = na51068_ghc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* FTGPIO010 banks of 32 (pin_base 0 / 32 / 64 / 96) */
static const struct {
    hwaddr base;
    int irq;
} na51068_gpio[] = {
    { 0xFE400000, 31 },
    { 0xFE420000, 54 },
    { 0xFE440000, 55 },
    { 0xFE640000, 87 },
};

/* FTIIC010 0..5 (nvt-na51068-i2c.dtsi) */
static const struct {
    hwaddr base;
    int irq;
} na51068_i2c[] = {
    { 0xFE300000, 21 }, { 0xFE320000, 36 }, { 0xFE340000, 63 },
    { 0xFE360000, 66 }, { 0xFE380000, 86 }, { 0xFE3A0000, 19 },
};

/*
 * The two USB 2.0 hosts ("nvt,ehci-nvtivot", ehci-na51068.c): NA51089's
 * FOTG200 (nvt-usb), its UTMI PHY in a separate window (phy_reg).
 */
static const struct {
    hwaddr base, phy;
    int irq;
} na51068_usb[] = {
    { 0xF9100000, 0xFCF00000, 61 },
    { 0xF9200000, 0xFD000000, 62 },
};

/* The security engines: NA51055's blocks (na51068_{crypto,hash,rsa}.c) */
static const struct {
    const char *type;
    hwaddr base;
    int irq;
} na51068_crypto[] = {
    { "nvt-crypto", 0xFEC60000, 44 },
    { "nvt-hash",   0xFEC80000, 45 },
    { "nvt-rsa",    0xFECA0000, 46 },
};

/* Bus glue the drivers program and never read back: storage. */
static const struct {
    const char *name;
    hwaddr base;
    uint64_t size;
} na51068_storage[] = {
    { "na51068-pad",    0xFE040000, 0x1000 },  /* TOP's second window */
    { "na51068-ahbc",   0xFE840000, 0x1000 },  /* nvt_ahbc_drv.c */
};

#define NA51068_SDHCI_BASE      0xFA600000
#define NA51068_IRQ_SDHCI       64
#define NA51068_SDHCI_VENDOR    0x100       /* SDC_VENDOR0..3 */
#define NA51068_DMAC_BASE       0xFCA00000
#define NA51068_IRQ_DMAC        14
#define NA51068_RTC_BASE        0xFE880000
#define NA51068_IRQ_RTC         95
#define NA51068_PWM_BASE        0xFE6C0000
#define NA51068_IRQ_PWM         43
#define NA51068_IR_BASE         0xFE540000
#define NA51068_IRQ_IR          65
#define NA51068_THERMAL_BASE    0xFE2C0000
#define NA51068_EEPROM_ADDR     0x50        /* a 24C02 on I2C 0 */

/*
 * The two MACs ("nvt,synopsys_eth", drivers/net/ethernet/novatek/na51068):
 * NA51089's DWC Ethernet QoS core with its Novatek glue at +0x3000 and the
 * embedded PHY's analog block at +0x3800, so nvt-eqos serves both.  MAC 0
 * runs RGMII to an external PHY, MAC 1 the embedded one (nvt_eth_env_probe
 * by the ETH pinmux); either way the PHY is nvt-eqos's MII PHY.  -nic
 * fills MAC 0 (eth0) first.
 */
static const struct {
    hwaddr base;
    int irq;
} na51068_eth[] = {
    { 0xFCC00000, 59 },
    { 0xFCD00000, 60 },
};

/* -M nt98323,flash-file=: the boot flash image (SPI NOR, or SPI NAND) */
static char *na51068_flash_file;

static char *na51068_boot_file;
static uint32_t na51068_boot_strap;

static void na51068_get_boot_strap(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &na51068_boot_strap, errp);
}

static void na51068_set_boot_strap(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    visit_type_uint32(v, name, &na51068_boot_strap, errp);
}

static char *na51068_get_boot_file(Object *obj, Error **errp)
{
    return g_strdup(na51068_boot_file);
}

static void na51068_set_boot_file(Object *obj, const char *value,
                                  Error **errp)
{
    g_free(na51068_boot_file);
    na51068_boot_file = g_strdup(value);
}

static char *na51068_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(na51068_flash_file);
}

static void na51068_set_flash_file(Object *obj, const char *value,
                                   Error **errp)
{
    g_free(na51068_flash_file);
    na51068_flash_file = g_strdup(value);
}

typedef struct {
    const char *desc;
    uint32_t chip_id;
    uint64_t ram_size_default;
    uint64_t linux_mem;         /* the tree's /memory; the rest is HDAL's */
} Na51068SoCConfig;

/*
 * The EVBs: NT98323 with DDR4 4 Gb x 1 (cfg_98323_DDR4_4GX1_NOR_EVB:
 * nvt_memory_cfg/dram 512 MB, Linux 192 MB).  The chip ID is not in the
 * SDK's sources and nothing there compares it; 0 until it is read off a
 * part.
 */
static const Na51068SoCConfig nt98323_soc = {
    .desc               = "Novatek NT98323 (2x Cortex-A9, NA51068, 512 MB)",
    .chip_id            = 0,
    .ram_size_default   = 512 * MiB,
    .linux_mem          = 0x0C000000,   /* nvt-na51068-mem-tbl.dtsi */
};

/*
 * NT98321: the same die (U-Boot's _CHIP_NAME_ is the only difference),
 * cfg_98321_DDR3_4GX1_NAND_EVB: DDR3 512 MB, Linux the first 160 MB.
 */
static const Na51068SoCConfig nt98321_soc = {
    .desc               = "Novatek NT98321 (2x Cortex-A9, NA51068, 512 MB)",
    .chip_id            = 0,
    .ram_size_default   = 512 * MiB,
    .linux_mem          = 0x0A000000,   /* nvt-na51068-mem-tbl.dtsi */
};

/*
 * The second core: as on NA51055, mach-nvt-ivot/platsmp.c writes the
 * entry to cpu-release-addr and releases the core from its pen.  The core
 * stays off until the first non-zero write there.
 */
static uint64_t na51068_release_read(void *opaque, hwaddr addr, unsigned size)
{
    return ((uint32_t *)opaque)[addr / 4];
}

static void na51068_release_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned size)
{
    CPUState *cs = qemu_get_cpu(1);

    ((uint32_t *)opaque)[addr / 4] = val;
    if (addr == 0 && val && cs && ARM_CPU(cs)->power_state == PSCI_OFF) {
        arm_set_cpu_on(ARM_CPU(cs)->mp_affinity, (uint32_t)val, 0, 1, false);
    }
}

static const MemoryRegionOps na51068_release_ops = {
    .read = na51068_release_read,
    .write = na51068_release_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void na51068_write_secondary(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void na51068_reset_secondary(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void na51068_init(MachineState *machine, const Na51068SoCConfig *c)
{
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *rel = g_new(MemoryRegion, 1);
    unsigned int smp_cpus = machine->smp.cpus;
    qemu_irq pic[NA51068_NUM_IRQ - 32];
    DeviceState *mp, *dev, *flash;
    SysBusDevice *mpbus;
    ARMCPU *cpu[2];
    int n;

    for (n = 0; n < smp_cpus; n++) {
        Object *cpuobj = object_new(machine->cpu_type);

        /* Entered in SVC by the loader and U-Boot; no TEE. */
        if (object_property_find(cpuobj, "has_el3")) {
            object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
        }
        object_property_set_int(cpuobj, "reset-cbar", NA51068_MPCORE_BASE,
                                &error_fatal);
        /* r4p1, as novatek.c: arm_global_timer wants r2p0 or later */
        object_property_set_uint(cpuobj, "midr", 0x414fc091, &error_fatal);
        if (n > 0) {
            object_property_set_bool(cpuobj, "start-powered-off", true,
                                     &error_fatal);
        }
        qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
        cpu[n] = ARM_CPU(cpuobj);
    }

    memory_region_add_subregion(sysmem, 0, machine->ram);
    for (n = 0; n < ARRAY_SIZE(na51068_windows); n++) {
        create_unimplemented_device(na51068_windows[n].name,
                                    na51068_windows[n].base,
                                    na51068_windows[n].size);
    }

    mp = qdev_new("a9mpcore_priv");
    mpbus = SYS_BUS_DEVICE(mp);
    qdev_prop_set_uint32(mp, "num-cpu", smp_cpus);
    qdev_prop_set_uint32(mp, "num-irq", NA51068_NUM_IRQ);
    sysbus_realize_and_unref(mpbus, &error_fatal);
    sysbus_mmio_map(mpbus, 0, NA51068_MPCORE_BASE);
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
    for (n = 0; n < NA51068_NUM_IRQ - 32; n++) {
        pic[n] = qdev_get_gpio_in(mp, n);
    }

    sysbus_create_simple("l2x0", NA51068_L2_BASE, NULL);

    memory_region_init_io(rel, OBJECT(machine), &na51068_release_ops,
                          g_malloc0(NA51068_RELEASE_SIZE), "na51068-release",
                          NA51068_RELEASE_SIZE);
    memory_region_add_subregion_overlap(sysmem, NA51068_RELEASE_BASE, rel, 1);

    dev = qdev_new("nvt-cg");
    qdev_prop_set_uint8(dev, "layout", 3);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_CG_BASE, 1);

    dev = qdev_new("nvt-top");
    qdev_prop_set_uint32(dev, "chip-id", c->chip_id);
    qdev_prop_set_uint32(dev, "strap", na51068_boot_strap);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_TOP_BASE, 1);

    /* NA51089's OTP controller: efuse_open() (shipped) polls its status. */
    sysbus_create_simple("nvt-otp", NA51068_OTP_BASE, NULL);

    /* The keyed watchdog of NA51089; the restart hook's MANUAL reset. */
    dev = qdev_new("nvt-wdt");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_WDT_BASE, 1);

    dev = qdev_new("nvt-fttmr010");
    qdev_prop_set_uint32(dev, "freq", 12000000);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_TIMER_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_TIMER]);

    /* The boot flash on the SPI020: nvt-flash's engine, Faraday registers */
    flash = qdev_new(TYPE_NVT_FLASH);
    qdev_prop_set_bit(flash, "spi020", true);
    if (na51068_flash_file) {
        qdev_prop_set_string(flash, "flash-file", na51068_flash_file);
    }
    object_property_add_child(OBJECT(machine), "nvt-flash", OBJECT(flash));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(flash), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(flash), 0, NA51068_FLASH_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(flash), 0, pic[NA51068_IRQ_FLASH]);

    for (n = 0; n < ARRAY_SIZE(na51068_sata); n++) {
        DriveInfo *hd[1] = { drive_get(IF_IDE, 0, n) };
        MemoryRegion *ghc = g_new(MemoryRegion, 1);
        DeviceState *bank;

        dev = qdev_new(TYPE_SYSBUS_AHCI);
        qdev_prop_set_uint32(dev, "num-ports", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51068_sata[n].ahci,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51068_sata[n].irq]);
        ahci_ide_create_devs(&SYSBUS_AHCI(dev)->ahci, hd);
        /* Not owned by the AHCI: its re-entrancy guard would block the
         * forwarded access. */
        memory_region_init_io(ghc, OBJECT(machine), &na51068_ghc_ops,
                              sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0),
                              "na51068-sata.ghc", 4);
        memory_region_add_subregion_overlap(sysmem, na51068_sata[n].ahci +
                                            NA51068_AHCI_GHC, ghc, 2);

        bank = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(bank, "size", 0x1000);
        qdev_prop_set_string(bank, "name", n ? "na51068-sata2-phy" :
                                              "na51068-sata-phy");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bank), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(bank), 0, na51068_sata[n].phy,
                                1);
    }

    for (n = 0; n < ARRAY_SIZE(na51068_eth); n++) {
        dev = qdev_new("nvt-eqos");
        qemu_configure_nic_device(dev, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51068_eth[n].base,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51068_eth[n].irq]);
    }

    for (n = 0; n < ARRAY_SIZE(na51068_gpio); n++) {
        char name[16];

        dev = qdev_new("nvt-ftgpio010");
        snprintf(name, sizeof(name), "gpio%d", n);
        object_property_add_child(OBJECT(machine), name, OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51068_gpio[n].base,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51068_gpio[n].irq]);
    }

    for (n = 0; n < ARRAY_SIZE(na51068_i2c); n++) {
        char name[16];

        dev = qdev_new("nvt-ftiic010");
        snprintf(name, sizeof(name), "i2c%d", n);
        object_property_add_child(OBJECT(machine), name, OBJECT(dev));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51068_i2c[n].base,
                                1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51068_i2c[n].irq]);
        if (n == 0) {
            /* A 24C02 to talk to: the EVB's buses are empty. */
            I2CSlave *ee = i2c_slave_new("at24c-eeprom",
                                         NA51068_EEPROM_ADDR);

            qdev_prop_set_uint32(DEVICE(ee), "rom-size", 256);
            i2c_slave_realize_and_unref(ee, (I2CBus *)qdev_get_child_bus(
                                        dev, "i2c"), &error_fatal);
        }
    }

    dev = qdev_new("nvt-ftdmac030");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_DMAC_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_DMAC]);
    /* the SPI020's DMA request is handshake 0 (the loader's NAND driver) */
    qdev_connect_gpio_out_named(flash, "drq", 0,
                                qdev_get_gpio_in_named(dev, "drq", 0));

    /* The SD host: a standard SDHCI 3.0, its vendor registers storage. */
    {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        DeviceState *bank = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(bank, "size", 0x1000 - NA51068_SDHCI_VENDOR);
        qdev_prop_set_string(bank, "name", "na51068-sdc-vendor");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bank), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(bank), 0, NA51068_SDHCI_BASE +
                                NA51068_SDHCI_VENDOR, 1);

        dev = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(dev, "sd-spec-version", 3);
        qdev_prop_set_uint64(dev, "capareg", 0x057834b4);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_SDHCI_BASE, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_SDHCI]);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    for (n = 0; n < ARRAY_SIZE(na51068_usb); n++) {
        DeviceState *bank;

        dev = qdev_new("nvt-usb");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        /* the EHCI over the glue it leaves unclaimed (nvt-usb.c) */
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, na51068_usb[n].base,
                                1);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, na51068_usb[n].base,
                                2);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[na51068_usb[n].irq]);

        bank = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(bank, "size", 0x1000);
        qdev_prop_set_string(bank, "name", n ? "na51068-u2phy1" :
                                              "na51068-u2phy0");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(bank), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(bank), 0, na51068_usb[n].phy,
                                1);
    }

    dev = qdev_new("nvt-rtc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_RTC_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_RTC]);

    for (n = 0; n < ARRAY_SIZE(na51068_crypto); n++) {
        dev = qdev_new(na51068_crypto[n].type);
        if (!strcmp(na51068_crypto[n].type, "nvt-hash")) {
            qdev_prop_set_bit(dev, "iv-reversed", true);    /* NA51055's */
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                na51068_crypto[n].base, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           pic[na51068_crypto[n].irq]);
    }

    dev = qdev_new("nvt-na51068-pwm");
    object_property_add_child(OBJECT(machine), "pwm", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_PWM_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_PWM]);

    dev = qdev_new("nvt-na51068-ir");
    object_property_add_child(OBJECT(machine), "ir", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_IR_BASE, 1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, pic[NA51068_IRQ_IR]);

    dev = qdev_new("nvt-thermal");
    object_property_add_child(OBJECT(machine), "thermal", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, NA51068_THERMAL_BASE, 1);

    /*
     * The two DDR controllers with their arbiters (the tree's nvt_arb
     * windows) and their paged DDR3 / DDR4 PHYs at +0x8000, which the
     * loader trains.
     */
    for (n = 0; n < 2; n++) {
        dev = qdev_new("nvt-ddrc");
        qdev_prop_set_bit(dev, "phy-pages", true);
        qdev_prop_set_uint32(dev, "phy-offset", 0x8000);
        qdev_prop_set_bit(dev, "vref-strobe", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                n ? 0xFD600000 : 0xFC400000, 1);
    }

    for (n = 0; n < ARRAY_SIZE(na51068_storage); n++) {
        dev = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(dev, "size", na51068_storage[n].size);
        qdev_prop_set_string(dev, "name", na51068_storage[n].name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0,
                                na51068_storage[n].base, 1);
    }

    for (n = 0; n < ARRAY_SIZE(na51068_uarts); n++) {
        serial_mm_init(sysmem, na51068_uarts[n].base, 2,
                       pic[na51068_uarts[n].irq], 48000000 / 16,
                       serial_hd(n), DEVICE_LITTLE_ENDIAN);
    }

    if (machine->kernel_filename && !machine->dtb) {
        error_report("novatek: %s needs a board device tree (-dtb), such "
                     "as the SDK's cfg_98323_DDR4_4GX1_NOR_EVB "
                     "nvt-na51068-evb.dts built",
                     MACHINE_GET_CLASS(machine)->name);
        exit(1);
    }
    /* arm_load_kernel() rewrites the tree's /memory with this */
    binfo.ram_size = MIN(c->linux_mem, machine->ram_size);
    binfo.loader_start = 0;
    binfo.write_secondary_boot = na51068_write_secondary;
    binfo.secondary_cpu_reset_hook = na51068_reset_secondary;
    arm_load_kernel(cpu[0], machine, &binfo);

    /*
     * No -kernel: boot the flash the way the SDK's loader does (the same
     * fdt / uboot partitions and SHMINFO handoff as NA51089).
     */
    if (!machine->kernel_filename && na51068_flash_file) {
        nvt_rom_set_loader_file(na51068_boot_file);
        /* the loader in its SRAM (LDS_LZ.lds 0xFE080000), or the stand-in */
        static const NvtRomConfig rc = {
            NA51068_SRAM_BASE, NA51068_SRAM_SIZE, 0xFE0F8000, 0xFE200000, 2,
        };

        if (nvt_flash_has_loader(NVT_FLASH(flash), rc.sram_base,
                                 rc.sram_size)) {
            nvt_rom_boot(NVT_FLASH(flash), &rc);
        } else {
            nvt_flash_boot(NVT_FLASH(flash), machine->ram_size, false);
        }
    }
}

#define DEFINE_NA51068_MACHINE(namestr, tag, config)                 \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        na51068_init(machine, &config);                              \
    }                                                                \
    static void tag##_machine_init(MachineClass *mc)                 \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "na51068.ram";                          \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a9");       \
        mc->max_cpus = 2;                                            \
        mc->default_cpus = 2;                                        \
        mc->ignore_memory_transaction_failures = true;               \
        object_class_property_add(OBJECT_CLASS(mc), "boot-strap",    \
            "uint32", na51068_get_boot_strap, na51068_set_boot_strap,\
            NULL, NULL);                                             \
        object_class_property_add_str(OBJECT_CLASS(mc), "boot-file", \
            na51068_get_boot_file, na51068_set_boot_file);           \
        object_class_property_set_description(OBJECT_CLASS(mc),      \
            "boot-file", "loader image the mask ROM got by TFTP or " \
            "UART, in place of the flash's");                        \
        object_class_property_add_str(OBJECT_CLASS(mc), "flash-file",\
            na51068_get_flash_file, na51068_set_flash_file);         \
        object_class_property_set_description(OBJECT_CLASS(mc),      \
            "flash-file", "SPI NOR image on the SPI020 (or a 128 / "  \
            "256 MiB SPI NAND image); writes go back to it");        \
    }                                                                \
    DEFINE_MACHINE_ARM(namestr, tag##_machine_init)

DEFINE_NA51068_MACHINE("nt98323", nt98323, nt98323_soc)
DEFINE_NA51068_MACHINE("nt98321", nt98321, nt98321_soc)
