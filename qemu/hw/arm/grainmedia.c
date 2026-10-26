/*
 * Grain Media (GM) IP-camera SoC machines: GM8135S, GM8136, GM8136S.
 *
 * The GM813x parts put a Faraday FA626TE (ARMv5TE) in front of Faraday
 * peripherals, DDR at 0 (after the boot loader's AXI remap) and
 * everything else from 0x90000000 (arch/arm/mach-GM/include/mach/
 * platform-GM8136/{board.h,platform_io.h} of the GM 3.3 kernel, U-Boot
 * 2013.01's arch-gm8136/gm8136.h):
 *
 *   0x90700000 / 0x90800000 / 0x90900000 UART0-2 (16550, 4-byte stride)
 *   0x90C00000 PMU: chip ID, strap, PLLs, gating  hw/misc/gm-pmu.c
 *   0x90D00000 FTTMR010 timers                     hw/timer/gm-timer.c
 *   0x90E00000 FTWDT010 watchdog                   hw/watchdog/gm-wdt.c
 *   0x91000000 / 0x91900000 FTGPIO010 GPIO0/1      hw/gpio/gm-gpio.c
 *   0x91200000 FTI2C010 I2C (the sensor's bus)     hw/i2c/gm-i2c.c
 *   0x92000000 SRAM (the boot loader runs here)
 *   0x92300000 FTSPI020 flash controller           hw/ssi/gm-spi020.c
 *   0x92500000 FTGMAC100 Ethernet                  QEMU's ftgmac100
 *   0x92600000 FTDMAC020 AHB DMA                   hw/dma/gm-dmac.c
 *   0x92800000 / 0x93800000 FTSDC021 SD0/1 (SDHCI) QEMU's generic-sdhci
 *   0x93000000 FOTG210 USB OTG, host role          hw/usb/gm-otg.c
 *   0x96000000 FTINTC030 interrupt controller      hw/intc/gm-intc.c
 *   0x9A100000 FTDDRC010 DDR controller            hw/misc/gm-ddrc.c
 *
 * and the video pipeline the vendor graph runs:
 *
 *   0x93930000 ISP328 (frames, AE/AWB/AF stats)    hw/misc/gm-isp.c
 *   0x96100000 VCAP300 capture (OpenIPC logo)      hw/misc/gm-vcap.c
 *   0x9B700000 FT3DNR200 de-noiser                 hw/misc/gm-3dnr.c
 *   0x96200000 scaler300                           hw/misc/gm-scaler.c
 *   0x92C00000 MCP280 H.264 encoder (+ VCACHE at 0x92D00000)
 *                                                  hw/misc/gm-h264e.c
 *   0x9BF00000 LCD200, the graph's frame clock     hw/display/gm-lcd.c
 *   0x92200000 GM2D 2D engine                      hw/display/gm-2d.c
 *   0x92020000 MCP100 MJPEG (+ mailbox at 0x92008000, DMA wrapper at
 *              0x92400000)                         hw/misc/gm-mcp100.c
 *
 * audio and the small peripherals:
 *
 *   0x91300000 / 0x91400000 FTSSP010 I2S, + ADDA308 codec at 0x90B00000
 *                                                  hw/audio/gm-ssp.c
 *   0x90A00000 SAR ADC (CVBS detect, keys)         hw/misc/gm-saradc.c
 *   0x91C00000 FTRTC011 RTC                        hw/rtc/gm-rtc.c
 *   0x91B00000 FTPWM010 PWM                        hw/misc/gm-pwm.c
 *   0x91600000 FTIRDET IR receiver                 hw/misc/gm-irdet.c
 *   0x92700000 FTAES020 AES/DES                    hw/misc/fh-aes.c
 *   0x91D00000 pMonitor bandwidth monitor          hw/misc/gm-pmonitor.c
 *
 * The packages differ in the PMU's ID (bits 6:2) and the DRAM: the
 * GM8135S and GM8136S carry theirs in the package, the GM8136 takes it
 * outside.  The rest of the peripheral space reads as zero.
 *
 * Boot: flash-file= boots a camera's SPI NOR image as the mask ROM does.
 * The NOR starts with a 256-byte system header ("GM8136", partitions,
 * 0x55AA) and the nsboot loader at 0x1000; the ROM copies nsboot into
 * SRAM and runs it, nsboot sets up DDR and clocks, then loads U-Boot from
 * the header's first image (0x10000) to address 0.  -kernel boots a
 * uImage / zImage with ATAGs at 0x100, machine number 758 ("GM" in
 * arch/arm/tools/mach-types), as the vendor U-Boot does; the kernels
 * have no device tree.  The SDK's uImages are made to run in place: the
 * load address (0x2000000) is the header's, the entry (0x2000040) the
 * zImage's, so they are loaded 64 bytes up (gm_fixup_xip_uimage()).
 * The kernel's memory fixup (fmem_fixup_memory()) needs "mem=" on the
 * command line -- the vendor U-Boot's cmd1..cmd4 always carry it, with
 * "gmmem=" for the media pool -- and hangs before the console without
 * it, so a command line that lacks it gets the machine's RAM size.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/host-utils.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "hw/net/ftgmac100.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/sd/sdhci.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/system.h"
#include "block/block-common.h"
#include "target/arm/cpu.h"

#define GM_SRAM_BASE        0x92000000
#define GM_SRAM_SIZE        0x10000
#define GM_PERIPH_BASE      0x90000000
#define GM_PERIPH_SIZE      0x10000000

#define GM_PMU_BASE         0x90C00000
#define GM_TIMER_BASE       0x90D00000
#define GM_WDT_BASE         0x90E00000
#define GM_SPI020_BASE      0x92300000
#define GM_GMAC_BASE        0x92500000
#define GM_DMAC_BASE        0x92600000
#define GM_INTC_BASE        0x96000000
#define GM_I2C_BASE         0x91200000
#define GM_OTG_BASE         0x93000000
#define GM_DDRC_BASE        0x9A100000

#define GM_NUM_IRQ          64
#define GM_MACH             758     /* MACH_TYPE_GM */
#define GM_FA626TE_MIDR     0x66056261

#define GM_IRQ_DMAC         1
#define GM_IRQ_2D           4
#define GM_IRQ_SCALER       12
#define GM_IRQ_LCD          24
#define GM_IRQ_H264E        29
#define GM_IRQ_MCP100       31
#define GM_IRQ_VCAP         32
#define GM_IRQ_3DNR         42
#define GM_IRQ_ISP          43
#define GM_IRQ_GMAC         3
#define GM_IRQ_OTG          9
#define GM_IRQ_PMU_FCS      8
#define GM_IRQ_GPIO0        13
#define GM_IRQ_TIMER        14
#define GM_IRQ_WDT          16
#define GM_IRQ_I2C          18
#define GM_IRQ_SPI020       54
#define GM_IRQ_GPIO1        63

#define GM_IRQ_SSP0         6
#define GM_IRQ_SSP1         11
#define GM_IRQ_AES          19
#define GM_IRQ_SARADC       35
#define GM_IRQ_PMONITOR     44
#define GM_IRQ_IRDET        48
#define GM_IRQ_PWM          49
#define GM_IRQ_RTC          50

#define GM_DMA_REQ_SPI020   8

typedef struct GmSoCConfig {
    const char *desc;
    uint32_t chip_id;           /* PMU 0x00 */
    uint64_t ram_size_default;
} GmSoCConfig;

/*
 * PMU 0x00: 0x0813 6 1 xx -- revision B, the package in bits 6:2, which
 * the kernel's pmu_get_chip() and U-Boot's map to 8136 (0), 8136S (0x14,
 * 0x1C) or 8135 (0x4, 0x8, 0x10, 0x18).
 */
static const GmSoCConfig gm8135s_soc = {
    .desc = "Grain Media GM8135S (FA626TE, 64 MiB in package)",
    .chip_id = 0x08136100 | (0x04 << 2),
    .ram_size_default = 64 * MiB,
};

static const GmSoCConfig gm8136_soc = {
    .desc = "Grain Media GM8136 (FA626TE)",
    .chip_id = 0x08136100,
    .ram_size_default = 128 * MiB,
};

static const GmSoCConfig gm8136s_soc = {
    .desc = "Grain Media GM8136S (FA626TE, 128 MiB in package)",
    .chip_id = 0x08136100 | (0x14 << 2),
    .ram_size_default = 128 * MiB,
};

static const struct {
    hwaddr base;
    int irq;
} gm_uarts[] = {
    { 0x90700000, 21 },
    { 0x90800000, 22 },
    { 0x90900000, 25 },
};

/* the FTSSP010s: DMAC handshake lines from the kernel's platform data */
static const struct {
    hwaddr base;
    int irq;
    int req_rx, req_tx;
    bool adda;
} gm_ssps[] = {
    { 0x91300000, GM_IRQ_SSP0, 10, 9, true },
    { 0x91400000, GM_IRQ_SSP1, 12, 11, false },
};

#define TYPE_GM_MACHINE MACHINE_TYPE_NAME("grainmedia")
OBJECT_DECLARE_SIMPLE_TYPE(GmMachineState, GM_MACHINE)

static const struct {
    hwaddr base;
    int irq;
} gm_sdcs[] = {
    { 0x92800000, 15 },
    { 0x93800000, 17 },
};

/*
 * Image sensors for sensor=, on the I2C bus where the vendor ISP modules
 * (fisp_<sensor>.ko) probe them: 16-bit register addresses, the ID in
 * two registers.
 */
static const struct {
    const char *name;
    uint8_t addr;
    uint16_t id_reg;
    uint16_t id;
} gm_sensors[] = {
    { "ov2715", 0x36, 0x300A, 0x2715 },     /* the Longse cameras' */
    { "ov9715", 0x30, 0x300A, 0x9711 },
};

struct GmMachineState {
    MachineState parent_obj;
    char *flash_file;
    char *flash_type;
    char *sensor;
    DeviceState *intc;
};

static char *gm_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(GM_MACHINE(obj)->flash_file);
}

static void gm_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    GmMachineState *s = GM_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *gm_machine_get_flash_type(Object *obj, Error **errp)
{
    return g_strdup(GM_MACHINE(obj)->flash_type);
}

static void gm_machine_set_flash_type(Object *obj, const char *value,
                                      Error **errp)
{
    GmMachineState *s = GM_MACHINE(obj);

    g_free(s->flash_type);
    s->flash_type = g_strdup(value);
}

static char *gm_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(GM_MACHINE(obj)->sensor);
}

static void gm_machine_set_sensor(Object *obj, const char *value,
                                  Error **errp)
{
    GmMachineState *s = GM_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

static void gm_create_sensor(const char *name, I2CBus *bus)
{
    DeviceState *dev;
    int i;

    if (!name || !strcmp(name, "none")) {
        return;
    }
    for (i = 0; i < ARRAY_SIZE(gm_sensors); i++) {
        if (!strcmp(name, gm_sensors[i].name)) {
            break;
        }
    }
    if (i == ARRAY_SIZE(gm_sensors)) {
        error_report("grainmedia: unknown sensor '%s'", name);
        exit(1);
    }
    dev = qdev_new("galaxycore-sensor");
    qdev_prop_set_bit(dev, "reg16", true);
    qdev_prop_set_uint16(dev, "id-reg", gm_sensors[i].id_reg);
    qdev_prop_set_uint16(dev, "id", gm_sensors[i].id);
    qdev_prop_set_uint8(dev, "address", gm_sensors[i].addr);
    qdev_realize_and_unref(dev, BUS(bus), &error_fatal);
}

static qemu_irq gm_irq(GmMachineState *gms, int n)
{
    return qdev_get_gpio_in(gms->intc, n);
}

/*
 * The mask ROM's NOR boot: the system header's signature and 0x55AA at
 * 0xFE, then nsboot at 0x1000 for 24 KiB (SRAM0; SRAM1 above it is
 * nsboot's stack).  The stand-in copies nsboot to SRAM and puts a jump to
 * it at address 0, in DDR, which nsboot overwrites with U-Boot.
 */
#define GM_HDR_SIG          "GM81"
#define GM_NSBOOT_OFFSET    0x1000
#define GM_NSBOOT_SIZE      0x6000

/*
 *     ldr  pc, [pc, #-4]
 *     .word entry
 */
static const uint32_t gm_bootrom_stub[] = { 0xE51FF004, GM_SRAM_BASE };

static void gm_boot_rom_setup(const uint8_t *img, gsize len,
                              const char *path)
{
    uint32_t stub[ARRAY_SIZE(gm_bootrom_stub)];
    int i;

    if (len < GM_NSBOOT_OFFSET + GM_NSBOOT_SIZE) {
        error_report("grainmedia: %s is too small for a NOR image", path);
        exit(1);
    }
    if (memcmp(img, GM_HDR_SIG, 4) || img[0xFE] != 0x55 ||
        img[0xFF] != 0xAA) {
        warn_report("grainmedia: %s has no GM system header", path);
    }
    /* nsboot starts with its vector table: "b reset" */
    if (img[GM_NSBOOT_OFFSET + 3] != 0xEA) {
        error_report("grainmedia: %s has no nsboot at 0x%x", path,
                     GM_NSBOOT_OFFSET);
        exit(1);
    }
    rom_add_blob_fixed("grainmedia.nsboot", img + GM_NSBOOT_OFFSET,
                       GM_NSBOOT_SIZE, GM_SRAM_BASE);
    for (i = 0; i < ARRAY_SIZE(stub); i++) {
        stub[i] = cpu_to_le32(gm_bootrom_stub[i]);
    }
    rom_add_blob_fixed("grainmedia.bootrom", stub, sizeof(stub), 0);
}

/* A Macronix part of the image's size, as the cameras carry. */
static const char *gm_flash_type(gsize len)
{
    switch (len) {
    case 4 * MiB:   return "mx25l3205d";
    case 8 * MiB:   return "mx25l6405d";
    case 32 * MiB:  return "mx25l25635e";
    default:        return "mx25l12805d";
    }
}

static DeviceState *gm_create_flash(GmMachineState *gms, BusState *bus,
                                    gsize img_len)
{
    DeviceState *dev;
    BlockBackend *blk = NULL;
    DriveInfo *di = drive_get(IF_MTD, 0, 0);
    const char *type = gms->flash_type;

    if (gms->flash_file) {
        QDict *opts = qdict_new();
        Error *err = NULL;

        qdict_put_str(opts, "driver", "raw");
        blk = blk_new_open(gms->flash_file, NULL, opts, BDRV_O_RDWR, &err);
        if (!blk) {
            error_reportf_err(err, "grainmedia: %s: ", gms->flash_file);
            exit(1);
        }
    } else if (di) {
        blk = blk_by_legacy_dinfo(di);
        img_len = blk_getlength(blk);
    }
    if (!type) {
        type = gm_flash_type(img_len);
    }
    dev = qdev_new(type);
    if (blk) {
        qdev_prop_set_drive_err(dev, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(dev, bus, &error_fatal);
    return dev;
}

/*
 * U-Boot's bootm runs these uImages in place ("XIP Kernel Image"): the
 * header sits at the load address and the payload starts at the entry,
 * 64 bytes in.  QEMU puts the payload itself at the load address, so hand
 * it a copy whose load address is the entry.  Returns the file to load.
 */
#define GM_UIMAGE_MAGIC     0x27051956
#define GM_UIMAGE_HDR       64

static char *gm_fixup_xip_uimage(const char *path)
{
    g_autofree uint8_t *buf = NULL;
    g_autoptr(GError) gerr = NULL;
    gsize len;
    uint32_t load, ep;
    char *tmp;
    int fd;

    if (!g_file_get_contents(path, (gchar **)&buf, &len, NULL) ||
        len < GM_UIMAGE_HDR || ldl_be_p(buf) != GM_UIMAGE_MAGIC) {
        return g_strdup(path);
    }
    load = ldl_be_p(buf + 16);
    ep = ldl_be_p(buf + 20);
    if (ep != load + GM_UIMAGE_HDR) {
        return g_strdup(path);
    }
    stl_be_p(buf + 16, ep);
    fd = g_file_open_tmp("grainmedia-uimage-XXXXXX", &tmp, &gerr);
    if (fd < 0 || !g_file_set_contents(tmp, (gchar *)buf, len, &gerr)) {
        error_report("grainmedia: cannot write a copy of %s: %s", path,
                     gerr ? gerr->message : "?");
        exit(1);
    }
    close(fd);
    return tmp;
}

static void grainmedia_common_init(MachineState *machine,
                                   const GmSoCConfig *c)
{
    GmMachineState *gms = GM_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *pmu, *dmac, *spi, *dev, *flash;
    SysBusDevice *sbd;
    Object *cpuobj;
    int n;

    /* what the DDR controller's size codes describe */
    if (machine->ram_size < 64 * MiB || machine->ram_size > 512 * MiB ||
        !is_power_of_2(machine->ram_size)) {
        error_report("grainmedia: DDR must be 64, 128, 256 or 512 MiB");
        exit(1);
    }
    if (gms->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(gms->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("grainmedia: cannot read %s: %s", gms->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    /* The FA626TE: an ARMv5TE that the kernel's proc-fa626te knows by ID */
    cpuobj = object_new(machine->cpu_type);
    object_property_set_uint(cpuobj, "midr", GM_FA626TE_MIDR, &error_fatal);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    memory_region_add_subregion(sysmem, 0, machine->ram);
    memory_region_init_ram(sram, NULL, "grainmedia.sram", GM_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, GM_SRAM_BASE, sram);

    /* Everything unmodelled in the peripheral space reads as zero. */
    create_unimplemented_device("grainmedia.periph", GM_PERIPH_BASE,
                                GM_PERIPH_SIZE);

    gms->intc = qdev_new("gm-intc");
    qdev_prop_set_uint32(gms->intc, "num-irq", GM_NUM_IRQ);
    sbd = SYS_BUS_DEVICE(gms->intc);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, GM_INTC_BASE);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_IRQ));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_FIQ));

    pmu = qdev_new("gm-pmu");
    qdev_prop_set_uint32(pmu, "chip-id", c->chip_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmu), 0, GM_PMU_BASE);
    qdev_connect_gpio_out_named(pmu, "fcs-irq", 0,
                                gm_irq(gms, GM_IRQ_PMU_FCS));

    dev = qdev_new("gm-ddrc");
    qdev_prop_set_uint64(dev, "ram-size", machine->ram_size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GM_DDRC_BASE);

    dev = qdev_new("gm-timer");
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(pmu, "pclk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GM_TIMER_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_TIMER));

    dev = qdev_new("gm-wdt");
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(pmu, "pclk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GM_WDT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_WDT));

    /* FTUART010: a 16550 with 4-byte register stride, 25 MHz UCLK */
    for (n = 0; n < ARRAY_SIZE(gm_uarts); n++) {
        serial_mm_init(sysmem, gm_uarts[n].base, 2,
                       gm_irq(gms, gm_uarts[n].irq), 25000000 / 16,
                       serial_hd(n), DEVICE_LITTLE_ENDIAN);
    }

    sysbus_create_simple("gm-gpio", 0x91000000, gm_irq(gms, GM_IRQ_GPIO0));
    sysbus_create_simple("gm-gpio", 0x91900000, gm_irq(gms, GM_IRQ_GPIO1));

    dmac = qdev_new("gm-dmac");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, GM_DMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dmac), 0, gm_irq(gms, GM_IRQ_DMAC));

    spi = qdev_new("gm-spi020");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(spi), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(spi), 0, GM_SPI020_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(spi), 0, gm_irq(gms, GM_IRQ_SPI020));
    qdev_connect_gpio_out_named(spi, "dma-req", 0,
        qdev_get_gpio_in_named(dmac, "dma-req", GM_DMA_REQ_SPI020));
    flash = gm_create_flash(gms, qdev_get_child_bus(spi, "ssi"), img_len);
    qdev_connect_gpio_out_named(spi, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0));

    /*
     * Ethernet: Faraday's FTGMAC100 (not Aspeed's variant), which both the
     * vendor U-Boot and the kernel's ftgmac100 drive, the PHY found by an
     * MDIO scan through PHYCR.
     */
    dev = qdev_new(TYPE_FTGMAC100);
    qemu_configure_nic_device(dev, true, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GM_GMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_GMAC));

    /*
     * SD0 / SD1: FTSDC021, an SD 3.0 SDHCI with ADMA2, which the kernel's
     * sdhci-ftsdc021 runs with its own base clock.  -drive if=sd,index=N
     * is a card in slot N.
     */
    for (n = 0; n < ARRAY_SIZE(gm_sdcs); n++) {
        DriveInfo *di = drive_get(IF_SD, 0, n);

        dev = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(dev, "sd-spec-version", 3);
        qdev_prop_set_uint64(dev, "capareg", 0x017834b4);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, gm_sdcs[n].base);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           gm_irq(gms, gm_sdcs[n].irq));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    sysbus_create_simple("gm-otg", GM_OTG_BASE, gm_irq(gms, GM_IRQ_OTG));

    /* the video pipeline: ISP -> capture -> 3DNR / scaler -> H.264 */
    sysbus_create_simple("gm-isp", 0x93930000, gm_irq(gms, GM_IRQ_ISP));
    sysbus_create_simple("gm-vcap", 0x96100000, gm_irq(gms, GM_IRQ_VCAP));
    sysbus_create_simple("gm-3dnr", 0x9B700000, gm_irq(gms, GM_IRQ_3DNR));
    sysbus_create_simple("gm-scaler", 0x96200000,
                         gm_irq(gms, GM_IRQ_SCALER));
    dev = qdev_new("gm-h264e");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x92C00000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x92D00000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_H264E));
    sysbus_create_simple("gm-lcd", 0x9BF00000, gm_irq(gms, GM_IRQ_LCD));
    sysbus_create_simple("gm-2d", 0x92200000, gm_irq(gms, GM_IRQ_2D));
    /*
     * MCP100 (MJPEG): its registers, the firmware's mailbox in the codec's
     * local memory -- over the SRAM -- and the DMA wrapper
     */
    dev = qdev_new("gm-mcp100");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x92020000);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, GM_SRAM_BASE + 0x8000, 1);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, 0x92400000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_MCP100));

    /*
     * Audio: SSP0 is the I2S link to the ADDA308 codec (its registers at
     * 0x90B00000), the record / playback path of the camera's application;
     * SSP1 the second I2S port.  Both are DMAC handshake peripherals.
     */
    for (n = 0; n < ARRAY_SIZE(gm_ssps); n++) {
        dev = qdev_new("gm-ssp");
        qdev_prop_set_bit(dev, "adda", gm_ssps[n].adda);
        if (gm_ssps[n].adda && machine->audiodev) {
            qdev_prop_set_string(dev, "audiodev", machine->audiodev);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, gm_ssps[n].base);
        if (gm_ssps[n].adda) {
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x90B00000);
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           gm_irq(gms, gm_ssps[n].irq));
        qdev_connect_gpio_out_named(dev, "dma-rx", 0,
            qdev_get_gpio_in_named(dmac, "dma-req", gm_ssps[n].req_rx));
        qdev_connect_gpio_out_named(dev, "dma-tx", 0,
            qdev_get_gpio_in_named(dmac, "dma-req", gm_ssps[n].req_tx));
    }

    /*
     * the small peripherals; the SAR ADC's inputs and the IR remote are
     * driven from the monitor (qom-set /machine/saradc ch2 0x20,
     * qom-set /machine/irdet key 0x45)
     */
    dev = qdev_new("gm-saradc");
    object_property_add_child(OBJECT(machine), "saradc", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x90A00000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_SARADC));
    dev = qdev_new("gm-irdet");
    object_property_add_child(OBJECT(machine), "irdet", OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x91600000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_IRDET));
    sysbus_create_simple("gm-rtc", 0x91C00000, gm_irq(gms, GM_IRQ_RTC));
    sysbus_create_simple("gm-pwm", 0x91B00000, gm_irq(gms, GM_IRQ_PWM));
    dev = qdev_new("fh-aes");
    qdev_prop_set_bit(dev, "advance", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x92700000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_AES));
    dev = qdev_new("gm-pmonitor");
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(pmu, "pclk"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x91D00000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gm_irq(gms, GM_IRQ_PMONITOR));

    dev = sysbus_create_simple("gm-i2c", GM_I2C_BASE,
                               gm_irq(gms, GM_IRQ_I2C));
    gm_create_sensor(gms->sensor, (I2CBus *)qdev_get_child_bus(dev, "i2c"));

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = 0;
    binfo.board_id = GM_MACH;
    if (machine->kernel_filename) {
        char *orig = machine->kernel_filename;
        char *path = gm_fixup_xip_uimage(orig);
        const char *cmdline = machine->kernel_cmdline ?
                              machine->kernel_cmdline : "";

        if (!strstr(cmdline, "mem=")) {
            char *full = g_strdup_printf("%s%smem=%" PRIu64 "M", cmdline,
                                         *cmdline ? " " : "",
                                         machine->ram_size / MiB);

            g_free(machine->kernel_cmdline);
            machine->kernel_cmdline = full;
        }

        machine->kernel_filename = path;
        arm_load_kernel(ARM_CPU(cpuobj), machine, &binfo);
        machine->kernel_filename = orig;
        if (strcmp(path, orig)) {
            unlink(path);
        }
        g_free(path);
    } else {
        arm_load_kernel(ARM_CPU(cpuobj), machine, &binfo);
    }

    if (!machine->kernel_filename && img) {
        gm_boot_rom_setup(img, img_len, gms->flash_file);
    }
}

static void gm_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "grainmedia.ram";
    mc->ignore_memory_transaction_failures = true;
    machine_add_audiodev_property(mc);
    object_class_property_add_str(oc, "flash-file",
        gm_machine_get_flash_file, gm_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (system header at 0, "
        "nsboot at 0x1000); erases and programs are written back to it");
    object_class_property_add_str(oc, "flash-type",
        gm_machine_get_flash_type, gm_machine_set_flash_type);
    object_class_property_set_description(oc, "flash-type",
        "m25p80 part the SPI NOR is modelled as (e.g. mx25l6405d); "
        "default: a Macronix part of the image's size");
    object_class_property_add_str(oc, "sensor",
        gm_machine_get_sensor, gm_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "Image sensor on the I2C bus (none, ov2715, ov9715); "
        "default: none");
}

static const TypeInfo gm_machine_info = {
    .name           = TYPE_GM_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(GmMachineState),
    .class_init     = gm_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void gm_machine_register_types(void)
{
    type_register_static(&gm_machine_info);
}

type_init(gm_machine_register_types)

#define DEFINE_GM_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        grainmedia_common_init(machine, &config);                    \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");          \
        mc->default_ram_size = config.ram_size_default;              \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, GM_MACHINE, GmMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_GM_MACHINE("gm8135s", gm8135s, gm8135s_soc)
DEFINE_GM_MACHINE("gm8136", gm8136, gm8136_soc)
DEFINE_GM_MACHINE("gm8136s", gm8136s, gm8136s_soc)
