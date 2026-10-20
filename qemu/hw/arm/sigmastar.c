/*
 * SigmaStar (MStar) ARMv7 SoC machines.
 *
 * Table-driven like hisilicon.c, xiongmai.c, goke.c and anyka.c;
 * include/hw/arm/sigmastar.h has the family.  Boots a board's SPI NOR
 * image (flash-file=) through its own IPL and U-Boot, the way the mask
 * ROM does, or a kernel with -kernel.
 *
 * Copyright (c) 2026 Daniel Palmer <daniel@thingy.jp>
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/sigmastar.h"
#include "hw/boards.h"
#include "hw/cpu/a15mpcore.h"
#include "hw/ide/ahci.h"
#include "hw/ide/ahci-sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/sstar-riu.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/sstar-fsp.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"

#define SS_BOOTROM_BASE     0x00000000
#define SS_BOOTROM_SIZE     (16 * KiB)
#define SS_XIU_BASE         0x14000000
#define SS_PERIPHBASE       0x16000000
#define SS_DRAM_BASE        0x20000000
#define SS_DRAM_WINDOW      (512 * MiB)
#define SS_IMI_BASE         0xA0000000
#define SS_IMI_SIZE         (64 * KiB)

#define SS_MAX_CPUS         2
#define SS_GIC_NUM_SPI      128
#define SS_CNTFRQ_HZ        6000000     /* arm,armv7-timer clock-frequency */

/* RIU blocks with models; bank numbers as the vendor code names them. */
#define SS_ISP_BASE         SSTAR_RIU_ADDR(0x0008, 0)
#define SS_FSP_BASE         SSTAR_RIU_ADDR(0x0016, 0)
#define SS_QSPI_BASE        SSTAR_RIU_ADDR(0x0017, 0)
#define SS_TIMER_BASE       SSTAR_RIU_ADDR(0x0030, 0)   /* wdt + 3 timers */
#define SS_BDMA_BASE        SSTAR_RIU_ADDR(0x1002, 0)
#define SS_INTC_BASE        SSTAR_RIU_ADDR(0x1009, 0)
#define SS_SMPCTRL_BASE     SSTAR_RIU_ADDR(0x1020, 0)

/*
 * The smpctrl mailbox (bank 0x1020): CPU1 runs the mask ROM and parks
 * until the kernel writes its entry address to 0x13 (high half) and
 * 0x14 (low half) and the magic 0xBABE to 0x16 (smp_platform.c:
 * SECOND_START_ADDR_HI/LO, SECOND_MAGIC_NUMBER_ADDR).  Here CPU1 is
 * powered off instead and the magic powers it on at that address, as
 * Daniel Palmer's model does.  The bank otherwise reads back.
 */
#define SMPCTRL_BOOT_HI     0x13
#define SMPCTRL_BOOT_LO     0x14
#define SMPCTRL_UNLOCK      0x16
#define SMPCTRL_MAGIC       0xBABE

/* GIC SPIs, which are also the vendor DTS interrupt numbers. */
#define SS_SPI_FSP          37
#define SS_SPI_GOP_VSYNC    52
#define SS_SPI_SATA         50
#define SS_SPI_GE           57
#define SS_SPI_EMAC         58
#define SS_SPI_UART0        66
#define SS_SPI_UART1        67
#define SS_SPI_UART2        71
#define SS_SPI_FUART        79
#define SS_SPI_BDMA0        72
#define SS_SPI_BDMA1        73
#define SS_SPI_BDMA2        93
#define SS_SPI_BDMA3        94
#define SS_SPI_DISP_TOP     82
#define SS_SPI_UHC2         63
#define SS_SPI_UHC1         65
#define SS_SPI_UHC0         87
#define SS_SPI_TIMER0       96
#define SS_SPI_TIMER1       97
#define SS_SPI_WDT          98
#define SS_SPI_TIMER2       108

/*
 * Reset values the boot chain reads back without writing first, captured
 * on an SSD202D with contrib/mstarpoker by Daniel Palmer (mstar-clean,
 * hw/arm/mstarv7.c).  Same die family as the SSR621Q.
 */
static const SstarRegDefault ss_i2m_defaults[] = {
    /* clkgen (0x1F207000) */
    { 0x1F207008, 0x0001 }, { 0x1F20700C, 0x0001 }, { 0x1F2070CC, 0x1101 },
    { 0x1F2070D0, 0x0211 }, { 0x1F2070D4, 0x45A9 }, { 0x1F2070DC, 0x0101 },
    { 0x1F207108, 0x0001 }, { 0x1F207114, 0x0001 }, { 0x1F207118, 0x0101 },
    { 0x1F207144, 0x0001 }, { 0x1F207148, 0x0001 }, { 0x1F20714C, 0x0101 },
    { 0x1F207150, 0x0001 }, { 0x1F207154, 0x0101 }, { 0x1F207158, 0x0101 },
    { 0x1F20715C, 0x0001 }, { 0x1F207180, 0x0001 }, { 0x1F207184, 0x0001 },
    { 0x1F20718C, 0x0009 }, { 0x1F2071A8, 0x0001 }, { 0x1F2071B0, 0x0100 },
    { 0x1F2071B4, 0x003D }, { 0x1F2071B8, 0x0001 }, { 0x1F2071BC, 0x0001 },
    { 0x1F2071C4, 0xFFFF }, { 0x1F2071CC, 0x0A40 }, { 0x1F2071F8, 0x0101 },
    { 0x1F2071FC, 0xFFFF },
    /* PM clkgen (0x1F001C00) */
    { 0x1F001C00, 0x0003 }, { 0x1F001C20, 0x00FF }, { 0x1F001C50, 0x000F },
    { 0x1F001C58, 0x003F }, { 0x1F001C70, 0x0300 }, { 0x1F001C7C, 0x1800 },
    { 0x1F001CB8, 0x00FF }, { 0x1F001CC0, 0x0003 }, { 0x1F001CD4, 0x0008 },
    { 0x1F001CDC, 0xFFFF }, { 0x1F001CEC, 0xFFFF }, { 0x1F001CF0, 0xFFFF },
    { 0x1F001CF4, 0x003F }, { 0x1F001D24, 0x0020 }, { 0x1F001D28, 0x000F },
    { 0x1F001D30, 0x000F }, { 0x1F001D58, 0x003F }, { 0x1F001D84, 0x3C3F },
    { 0x1F001D88, 0x4000 }, { 0x1F001D8C, 0x0600 }, { 0x1F001D90, 0x2000 },
    { 0x1F001D94, 0x01FF }, { 0x1F001D98, 0x0006 }, { 0x1F001DBC, 0x0003 },
    /* chiptop (0x1F203C00), the bond strap at +0x120 aside */
    { 0x1F203C04, 0x0200 }, { 0x1F203C74, 0x7FF2 }, { 0x1F203C78, 0x800D },
    { 0x1F203C80, 0xFFFF }, { 0x1F203C88, 0xFFFF }, { 0x1F203CC4, 0x7FFF },
    { 0x1F203CC8, 0x7F00 }, { 0x1F203CE0, 0xFFFF }, { 0x1F203CE4, 0x0FFF },
    { 0x1F203CE8, 0xFFFF }, { 0x1F203CEC, 0x0FFF }, { 0x1F203CF0, 0xFFFF },
    { 0x1F203CF4, 0x0FFF }, { 0x1F203D00, 0x0003 }, { 0x1F203D10, 0xFFFF },
    { 0x1F203D14, 0xFFFF }, { 0x1F203D18, 0xFFFF }, { 0x1F203D50, 0x0054 },
    { 0x1F203D94, 0x1006 }, { 0x1F203DCC, 0xFFFF }, { 0x1F203DF0, 0xFFFF },
    { 0x1F203DF4, 0x0003 }, { 0x1F203DF8, 0x000F },
    /* chipver (0x1F003C00): +0x19C revision, read by the GOP code */
    { 0x1F003D9C, 0x0100 },
    /*
     * efuse (0x1F004000): the DDR ZQ / drive-strength calibration and the
     * USB/ETH/MIPI/HDMI analog trims the IPL reads (read_config_word)
     */
    { 0x1F004014, 0x0809 }, { 0x1F004018, 0xE148 }, { 0x1F00401C, 0x000D },
    { 0x1F004020, 0x0010 }, { 0x1F004024, 0x0591 }, { 0x1F004028, 0x4210 },
    { 0x1F00402C, 0x0F70 }, { 0x1F004058, 0x10BD }, { 0x1F00405C, 0x741A },
    { 0x1F004060, 0x84D1 }, { 0x1F004068, 0x5B65 }, { 0x1F00406C, 0x254A },
    { 0x1F004070, 0x4385 }, { 0x1F004074, 0x85CC },
    /* USB PLL, clock gate, UTMI PHYs (powered down) and USBC */
    { 0x1F283E00, 0x11B2 }, { 0x1F284000, 0x11B2 },
    { 0x1F284200, 0xFF05 }, { 0x1F284204, 0x9080 }, { 0x1F284210, 0x04AF },
    { 0x1F284600, 0x0200 },
    { 0x1F284A00, 0xFF05 }, { 0x1F284A04, 0x9080 }, { 0x1F284A10, 0x04AF },
    { 0x1F284E00, 0x0200 },
    { 0x1F285200, 0xFF05 }, { 0x1F285204, 0x9080 }, { 0x1F285210, 0x04AF },
    { 0x1F286200, 0x0200 },
    /* MIPI/DSI analog */
    { 0x1F2A4A00, 0x1000 }, { 0x1F2A4F40, 0x8002 },
    /* DID key: the boot-media strap, 0x20 = SPI NOR */
    { 0x1F0071C0, 0x0020 },
    /*
     * MIU DDR PLL frequency set: what the IPL programs, for a -kernel
     * boot that skips it (the kernel's MIU clock driver divides by it)
     */
    { 0x1F202060, 0x8000 }, { 0x1F202064, 0x0029 },
    { 0 }
};

/*
 * SSR621Q: the infinity2m die in Xiongmai's 4/8/16-channel NVRs, with
 * SATA, HDMI and the video decoder bonded out and DDR3 in the package.
 * The vendor DTS (model "INFINITY2M SSC010A-S01A-S") has memory at
 * 0x20000000 and the U-Boot bootargs LX_MEM=0xFF00000: 256 MiB.
 */
static const SsSoCConfig ssr621q_soc = {
    .desc               = "SigmaStar SSR621Q (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 256 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1E,
    .flash_jedec        = 0xC22018,         /* MX25L12835F, 16 MiB */
};

/* SSD202D: 128 MiB DDR3 in the package (bond 0x1E). */
static const SsSoCConfig ssd202d_soc = {
    .desc               = "SigmaStar SSD202D (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 128 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1E,
    .flash_jedec        = 0xEF4018,         /* W25Q128 */
};

/* SSD201: 64 MiB DDR2 in the package (bond 0x1D). */
static const SsSoCConfig ssd201_soc = {
    .desc               = "SigmaStar SSD201 (infinity2m, 2x Cortex-A7)",
    .num_cpus           = 2,
    .ram_size_default   = 64 * MiB,
    .chip_id            = 0xF0,
    .bond               = 0x1D,
    .flash_jedec        = 0xEF4018,
};

#define TYPE_SS_MACHINE MACHINE_TYPE_NAME("sigmastar")
OBJECT_DECLARE_SIMPLE_TYPE(SsMachineState, SS_MACHINE)

struct SsMachineState {
    MachineState parent_obj;

    char *flash_file;
    uint32_t flash_jedec;
    ARMCPU *cpus[SS_MAX_CPUS];
    MemoryRegion smpctrl;
    uint16_t smpctrl_regs[0x80];
    /* what bus masters see: MIU bus addresses, DRAM from 0 */
    MemoryRegion miu_root;
    MemoryRegion miu_sys;
    MemoryRegion miu_low;
    MemoryRegion miu_hi;
    AddressSpace miu_as;
    MemoryRegion sata_win;
};

static char *ss_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(SS_MACHINE(obj)->flash_file);
}

static void ss_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    SsMachineState *s = SS_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void ss_machine_get_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &SS_MACHINE(obj)->flash_jedec, errp);
}

static void ss_machine_set_flash_jedec(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    visit_type_uint32(v, name, &SS_MACHINE(obj)->flash_jedec, errp);
}

static uint64_t ss_smpctrl_read(void *opaque, hwaddr off, unsigned size)
{
    SsMachineState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->smpctrl_regs[(off >> 2) & 0x7F];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void ss_smpctrl_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SsMachineState *s = opaque;
    int reg = (off >> 2) & 0x7F;
    uint16_t *r = &s->smpctrl_regs[reg];

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        *r = (off & 1) ? (*r & 0x00FF) | ((val & 0xFF) << 8)
                       : (*r & 0xFF00) | (val & 0xFF);
    } else {
        *r = val;
    }
    if (reg == SMPCTRL_UNLOCK && *r == SMPCTRL_MAGIC && s->cpus[1]) {
        uint32_t entry = s->smpctrl_regs[SMPCTRL_BOOT_LO] |
                         ((uint32_t)s->smpctrl_regs[SMPCTRL_BOOT_HI] << 16);
        /* Released from the mask ROM, so in the Secure state. */
        int el = arm_feature(&s->cpus[1]->env, ARM_FEATURE_EL3) ? 3 : 1;
        int ret = arm_set_cpu_on(s->cpus[1]->mp_affinity, entry, 0, el,
                                 false);

        if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS &&
            ret != QEMU_ARM_POWERCTL_ALREADY_ON) {
            qemu_log_mask(LOG_GUEST_ERROR, "sigmastar: cannot start CPU1 "
                          "at 0x%08x (%d)\n", entry, ret);
        }
    }
}

static const MemoryRegionOps ss_smpctrl_ops = {
    .read = ss_smpctrl_read,
    .write = ss_smpctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * The IPL header at the start of the SPI NOR (and of the IPL image the
 * ROM copies to IMI):
 *
 *   +0x00  branch over the header      +0x04  "IPL_"
 *   +0x08  image size (16 bits)        +0x0C  checksum
 *
 * The mask ROM checks the magic through the XIU window, copies the
 * image into IMI at 0xA0000000 with BDMA and enters it there, the stack
 * in IMI at 0xA000D000 (Daniel Palmer's ROM notes, docs/system/arm/
 * mstarv7/bootrom.rst).  The IPL then brings up DDR, loads IPL_CUST,
 * which unpacks U-Boot.
 */
#define SS_IPL_MAGIC        0x5F4C5049      /* "IPL_" */
#define SS_IPL_STACK        0xA000D000

/*
 * What stands in for the mask ROM: the writes the ROM makes before it
 * enters the IPL -- watchdog off, timer 0 free-running from 0 to
 * 0xFFFFFFFF, the UART0 pad mux and clock -- then sp and a jump to the
 * IPL.  The same {address, value} table runner as anyka.c:
 *
 *     ldr   sp, [pc, #20]
 *     add   r2, pc, #20        @ r2 = table
 *  1: ldmia r2!, {r0, r1}
 *     cmp   r0, #0
 *     strne r1, [r0]
 *     bne   1b
 *     ldr   pc, [r2]
 *     .word SS_IPL_STACK
 */
static const uint32_t ss_bootrom_stub[] = {
    0xE59FD014, 0xE28F2014, 0xE8B20003, 0xE3500000, 0x15801000, 0x1AFFFFFB,
    0xE592F000, SS_IPL_STACK,
};

static const uint32_t ss_bootrom_script[] = {
    0x1F006010, 0x0000, 0x1F006014, 0x0000,     /* WDT MAX_PRD = 0: off */
    0x1F006040, 0x0000,                         /* timer 0 stopped */
    0x1F006048, 0xFFFF, 0x1F00604C, 0xFFFF,     /* MAX = 0xFFFFFFFF */
    0x1F006040, 0x0001,                         /* EN */
    0x1F2070C4, 0x0009,                         /* clk_uart */
    0x1F203D4C, 0x3210,                         /* UART pad mux */
};

static bool ss_boot_rom_setup(const uint8_t *img, gsize len, const char *path)
{
    GArray *rom = g_array_new(false, false, sizeof(uint32_t));
    const uint32_t end[3] = { 0, 0, SS_IMI_BASE };
    uint32_t size;
    guint i;

    if (len < 0x10 || ldl_le_p(img + 4) != SS_IPL_MAGIC) {
        error_report("sigmastar: %s has no IPL header (\"IPL_\" at 4)",
                     path);
        return false;
    }
    size = lduw_le_p(img + 8);
    if (size < 0x10 || size > len) {
        error_report("sigmastar: %s: IPL size 0x%x does not fit IMI",
                     path, size);
        return false;
    }
    g_array_append_vals(rom, ss_bootrom_stub, ARRAY_SIZE(ss_bootrom_stub));
    g_array_append_vals(rom, ss_bootrom_script,
                        ARRAY_SIZE(ss_bootrom_script));
    g_array_append_vals(rom, end, ARRAY_SIZE(end));
    for (i = 0; i < rom->len; i++) {
        g_array_index(rom, uint32_t, i) =
            cpu_to_le32(g_array_index(rom, uint32_t, i));
    }
    rom_add_blob_fixed("sigmastar.bootrom", rom->data, rom->len * 4,
                       SS_BOOTROM_BASE);
    g_array_free(rom, true);
    rom_add_blob_fixed("sigmastar.ipl", img, size, SS_IMI_BASE);
    return true;
}

/* A reset parks CPU1 again: the mailbox forgets its magic and address. */
static void ss_smpctrl_reset(void *opaque)
{
    SsMachineState *s = opaque;

    memset(s->smpctrl_regs, 0, sizeof(s->smpctrl_regs));
}

/* Secondaries wait on smpctrl, never on boot.c's pen. */
static void ss_write_secondary_boot(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void ss_reset_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
}

static qemu_irq ss_spi(DeviceState *gic, DeviceState *intc, int spi)
{
    /* SPI 32-127 are the main-intc's MS_IRQ and MS_FIQ groups. */
    if (spi >= 32 && spi < 128) {
        return qdev_get_gpio_in(intc, spi - 32);
    }
    return qdev_get_gpio_in(gic, spi);
}

static void sigmastar_common_init(MachineState *machine, const SsSoCConfig *c)
{
    SsMachineState *sms = SS_MACHINE(machine);
    static struct arm_boot_info binfo;
    static SstarRegDefault chip_id[2];
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *imi = g_new(MemoryRegion, 1);
    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    g_autofree uint8_t *img = NULL;
    gsize img_len = 0;
    DeviceState *gic, *intc, *riu, *fsp, *dev;
    const int ncpus = MIN(machine->smp.cpus, c->num_cpus);

    assert(c->num_cpus <= SS_MAX_CPUS);
    uint64_t off;
    int n;

    /* The MIU decodes a power of two; the window repeats it. */
    if (machine->ram_size > SS_DRAM_WINDOW ||
        !is_power_of_2(machine->ram_size)) {
        error_report("sigmastar: DRAM must be a power of two up to %" PRIu64
                     " MiB", (uint64_t)SS_DRAM_WINDOW / MiB);
        exit(1);
    }
    if (sms->flash_file) {
        GError *gerr = NULL;

        if (!g_file_get_contents(sms->flash_file, (gchar **)&img, &img_len,
                                 &gerr)) {
            error_report("sigmastar: cannot read %s: %s", sms->flash_file,
                         gerr->message);
            g_error_free(gerr);
            exit(1);
        }
    }

    for (n = 0; n < ncpus; n++) {
        Object *o = object_new(machine->cpu_type);

        object_property_set_int(o, "reset-cbar", SS_PERIPHBASE,
                                &error_fatal);
        /*
         * 6 MHz, as the vendor DTS pins arm,armv7-timer: the kernel
         * scales the counter by it.
         */
        object_property_set_int(o, "cntfrq", SS_CNTFRQ_HZ, &error_fatal);
        /* CPU1 parks in the mask ROM until smpctrl releases it. */
        object_property_set_bool(o, "start-powered-off", n > 0,
                                 &error_fatal);
        qdev_realize(DEVICE(o), NULL, &error_fatal);
        sms->cpus[n] = ARM_CPU(o);
    }

    /*
     * DRAM, mirrored through the whole 512 MiB MIU0 window as the address
     * decode wraps: the IPL sizes DRAM by looking for the wrap.
     */
    memory_region_add_subregion(sysmem, SS_DRAM_BASE, machine->ram);
    for (off = machine->ram_size; off + machine->ram_size <= SS_DRAM_WINDOW;
         off += machine->ram_size) {
        MemoryRegion *alias = g_new(MemoryRegion, 1);

        memory_region_init_alias(alias, NULL, "sigmastar.dram-mirror",
                                 machine->ram, 0, machine->ram_size);
        memory_region_add_subregion(sysmem, SS_DRAM_BASE + off, alias);
    }

    memory_region_init_ram(imi, NULL, "sigmastar.imi", SS_IMI_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SS_IMI_BASE, imi);
    memory_region_init_rom(bootrom, NULL, "sigmastar.bootrom",
                           SS_BOOTROM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SS_BOOTROM_BASE, bootrom);

    /* GIC-400 in the A7 private region: distributor +0x1000, CPU +0x2000 */
    gic = qdev_new(TYPE_A15MPCORE_PRIV);
    qdev_prop_set_uint32(gic, "num-cpu", ncpus);
    qdev_prop_set_uint32(gic, "num-irq", SS_GIC_NUM_SPI + 32);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(gic), 0, SS_PERIPHBASE);
    for (n = 0; n < ncpus; n++) {
        DeviceState *cpu = DEVICE(sms->cpus[n]);
        SysBusDevice *sbd = SYS_BUS_DEVICE(gic);

        sysbus_connect_irq(sbd, n, qdev_get_gpio_in(cpu, ARM_CPU_IRQ));
        sysbus_connect_irq(sbd, n + ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_FIQ));
        sysbus_connect_irq(sbd, n + 2 * ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_VIRQ));
        sysbus_connect_irq(sbd, n + 3 * ncpus,
                           qdev_get_gpio_in(cpu, ARM_CPU_VFIQ));
    }
    create_unimplemented_device("sigmastar.periphbase", SS_PERIPHBASE,
                                0x8000);

    /* Storage under every RIU bank, and the boot chain's status bits. */
    riu = qdev_new(TYPE_SSTAR_RIU);
    qdev_prop_set_uint16(riu, "bond", c->bond);
    sstar_riu_add_defaults(SSTAR_RIU(riu), ss_i2m_defaults);
    /* chipver +0x00: the chip ID byte U-Boot and the kernel identify by */
    chip_id[0].addr = SSTAR_RIU_ADDR(0x001E, 0x00);
    chip_id[0].val = c->chip_id;
    sstar_riu_add_defaults(SSTAR_RIU(riu), chip_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(riu), &error_fatal);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(riu), 0, SSTAR_RIU_BASE, -1);

    intc = qdev_new("sstar-intc");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(intc), 0, SS_INTC_BASE);
    for (n = 0; n < 96; n++) {
        qdev_connect_gpio_out(intc, n, qdev_get_gpio_in(gic, 32 + n));
    }

    dev = qdev_new("sstar-timer");
    object_property_set_link(OBJECT(dev), "riu", OBJECT(riu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SS_TIMER_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       ss_spi(gic, intc, SS_SPI_TIMER0));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       ss_spi(gic, intc, SS_SPI_TIMER1));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2,
                       ss_spi(gic, intc, SS_SPI_WDT));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 3,
                       ss_spi(gic, intc, SS_SPI_TIMER2));

    /* UART0 is the console (DTS alias serial0), then uart1, fuart, uart2 */
    for (n = 0; n < 4; n++) {
        static const hwaddr base[4] = {
            0x1F221000, 0x1F221200, 0x1F220400, 0x1F221400,
        };
        static const int spi[4] = {
            SS_SPI_UART0, SS_SPI_UART1, SS_SPI_FUART, SS_SPI_UART2,
        };

        dev = qdev_new("sstar-uart");
        qdev_prop_set_chr(dev, "chardev", serial_hd(n));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ss_spi(gic, intc, spi[n]));
    }

    fsp = qdev_new(TYPE_SSTAR_FSP);
    if (sms->flash_file) {
        qdev_prop_set_string(fsp, "flash-file", sms->flash_file);
    }
    qdev_prop_set_uint32(fsp, "jedec", sms->flash_jedec ? sms->flash_jedec
                                                        : c->flash_jedec);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(fsp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 0, SS_ISP_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 1, SS_FSP_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 2, SS_QSPI_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fsp), 3, SS_XIU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(fsp), 0, ss_spi(gic, intc, SS_SPI_FSP));

    dev = qdev_new("sstar-bdma");
    object_property_set_link(OBJECT(dev), "fsp", OBJECT(fsp), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SS_BDMA_BASE);
    {
        static const int spi[4] = {
            SS_SPI_BDMA0, SS_SPI_BDMA1, SS_SPI_BDMA2, SS_SPI_BDMA3,
        };

        for (n = 0; n < 4; n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), n,
                               ss_spi(gic, intc, spi[n]));
        }
    }

    /*
     * EMAC: RIU view, X32 view (vendor DTS emac0); its internal PHY's
     * registers are in the RIU storage at 0x1F006200.
     */
    dev = qdev_new("sstar-emac");
    qemu_configure_nic_device(dev, true, NULL);
    object_property_set_link(OBJECT(dev), "riu", OBJECT(riu), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1F2A2000);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, 0x1F343C00);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ss_spi(gic, intc, SS_SPI_EMAC));

    /*
     * The bus masters' view of memory (sstar_ahci_sata_bus_address()):
     * MIU bus addresses below 512 MiB are DRAM; above, physical.
     */
    memory_region_init(&sms->miu_root, OBJECT(sms), "sigmastar.miu",
                       UINT64_MAX);
    memory_region_init_alias(&sms->miu_sys, OBJECT(sms), "sigmastar.miu-sys",
                             sysmem, 0, UINT64_MAX);
    memory_region_init_alias(&sms->miu_low, OBJECT(sms), "sigmastar.miu-dram",
                             sysmem, SS_DRAM_BASE, SS_DRAM_WINDOW);
    memory_region_add_subregion_overlap(&sms->miu_root, 0, &sms->miu_sys, 0);
    memory_region_add_subregion_overlap(&sms->miu_root, 0, &sms->miu_low, 1);
    /*
     * The SATA driver puts its PRD high words through the same
     * phys-to-bus subtraction: 0 becomes 0xE0000000.  The 32-bit MIU
     * ignores them; QEMU's AHCI does not, so the bus repeats there.
     */
    memory_region_init_alias(&sms->miu_hi, OBJECT(sms), "sigmastar.miu-hi",
                             &sms->miu_root, 0, 4 * GiB);
    memory_region_add_subregion(&sms->miu_root, 0xE0000000ULL << 32,
                                &sms->miu_hi);
    address_space_init(&sms->miu_as, &sms->miu_root, "sigmastar.miu");

    /*
     * SATA: a one-port AHCI at 0x1F345000 (HBA, port 0 at +0x100), its
     * PHY and misc banks (0x1525-0x1527) RIU storage.  The vendor driver
     * hands it MIU bus addresses.  The disk is -drive if=ide,index=0.
     */
    dev = qdev_new(TYPE_SYSBUS_AHCI);
    qdev_prop_set_uint32(dev, "num-ports", 1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    SYSBUS_AHCI(dev)->ahci.as = &sms->miu_as;
    memory_region_init_alias(&sms->sata_win, OBJECT(sms), "sigmastar.sata",
                             sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0),
                             0, 0x200);
    memory_region_add_subregion(sysmem, SSTAR_RIU_ADDR(0x1A28, 0),
                                &sms->sata_win);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ss_spi(gic, intc, SS_SPI_SATA));
    {
        DriveInfo *hd[1] = { drive_get(IF_IDE, 0, 0) };

        ahci_ide_create_devs(&SYSBUS_AHCI(dev)->ahci, hd);
    }

    /*
     * The three EHCI hosts (UHC0-2, DTS Sstar-ehci-1..3), each beside its
     * UTMI PHY and USBC wrapper banks (ehci-mstar-chip.h).  -device
     * usb-tablet / usb-kbd / usb-storage attach as USB 2.0 devices.
     */
    for (n = 0; n < 3; n++) {
        static const uint32_t bank[3] = { 0x1424, 0x1428, 0x1432 };
        static const int spi[3] = { SS_SPI_UHC0, SS_SPI_UHC1, SS_SPI_UHC2 };

        dev = qdev_new("sstar-uhc");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(bank[n], 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ss_spi(gic, intc, spi[n]));
    }

    /*
     * The display: GOP 1 (banks 0x1234/0x1235, the plane fbdev uses), the
     * display top (frame timing, vsync) and the MOP video plane, scanned
     * out to a graphic console.
     */
    dev = qdev_new("mstar-disp");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1234, 0));
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, SSTAR_RIU_ADDR(0x1128, 0));
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, SSTAR_RIU_ADDR(0x1405, 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       ss_spi(gic, intc, SS_SPI_DISP_TOP));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       ss_spi(gic, intc, SS_SPI_GOP_VSYNC));

    /* GE 2D engine: MI_GFX composites the GUI through it */
    dev = qdev_new("mstar-ge");
    qdev_prop_set_uint32(dev, "dram-base", SS_DRAM_BASE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SSTAR_RIU_ADDR(0x1409, 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ss_spi(gic, intc, SS_SPI_GE));

    memory_region_init_io(&sms->smpctrl, OBJECT(sms), &ss_smpctrl_ops, sms,
                          "sigmastar.smpctrl", 0x200);
    memory_region_add_subregion(sysmem, SS_SMPCTRL_BASE, &sms->smpctrl);
    qemu_register_reset(ss_smpctrl_reset, sms);

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = SS_DRAM_BASE;
    /*
     * The vendor kernel is entered by U-Boot in the Secure state and
     * does Secure-only setup (ACTLR.SMP, the L2 controls).
     */
    binfo.secure_boot = true;
    binfo.write_secondary_boot = ss_write_secondary_boot;
    binfo.secondary_cpu_reset_hook = ss_reset_secondary;
    arm_load_kernel(sms->cpus[0], machine, &binfo);

    if (machine->kernel_filename) {
        return;
    }
    if (machine->firmware) {
        /* A real mask ROM dump: it loads the IPL itself, through BDMA. */
        if (load_image_targphys(machine->firmware, SS_BOOTROM_BASE,
                                SS_BOOTROM_SIZE, NULL) < 0) {
            error_report("sigmastar: cannot load ROM %s", machine->firmware);
            exit(1);
        }
    } else if (img && !ss_boot_rom_setup(img, img_len, sms->flash_file)) {
        exit(1);
    }
}

static void ss_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "sigmastar.ram";
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    /*
     * 4 KiB target pages, as on virt.  With QEMU's default 1 KiB pages a
     * region starting inside one 1 KiB page and covering the next (the
     * MOP at 0x1F280A00, the EMAC PHY bank at 0x1F006200) is mapped with a
     * misaligned offset that corrupts its TLB entry ("iotlb_to_section:
     * Assertion `section_index < d->map.sections_nb'").  The RIU's blocks
     * are 512-byte aligned; none spans a whole 4 KiB page from inside one.
     */
    mc->minimum_page_bits = 12;
    object_class_property_add_str(oc, "flash-file",
        ss_machine_get_flash_file, ss_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the board (IPL at 0); erases and "
        "programs are written back to it");
    object_class_property_add(oc, "flash-jedec", "uint32",
        ss_machine_get_flash_jedec, ss_machine_set_flash_jedec, NULL, NULL);
    object_class_property_set_description(oc, "flash-jedec",
        "JEDEC ID the SPI NOR answers with (e.g. 0xEF4018)");
}

static const TypeInfo ss_machine_info = {
    .name           = TYPE_SS_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(SsMachineState),
    .class_init     = ss_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void ss_machine_register_types(void)
{
    type_register_static(&ss_machine_info);
}

type_init(ss_machine_register_types)

#define DEFINE_SS_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        sigmastar_common_init(machine, &config);                     \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_ram_size = config.ram_size_default;              \
        mc->max_cpus = config.num_cpus;                              \
        mc->default_cpus = config.num_cpus;                          \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, SS_MACHINE, SsMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_SS_MACHINE("ssr621q", ssr621q, ssr621q_soc)
DEFINE_SS_MACHINE("ssd202d", ssd202d, ssd202d_soc)
DEFINE_SS_MACHINE("ssd201", ssd201, ssd201_soc)
