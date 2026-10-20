/*
 * Goke GK7101 / GK7102 IP-camera SoC machines.
 *
 * Table-driven like hisilicon.c and xiongmai.c; see include/hw/arm/goke.h
 * for the vendor HAL these models are seen through.
 *
 * Boots a camera's own SPI-NOR image (flash-file=) the way the mask ROM
 * does: the boot header at offset 0 is a list of register writes (DDR
 * bring-up) and a U-Boot to copy into DDR and enter.  That vendor U-Boot
 * then loads the vendor 3.4.43 kernel, which needs the HAL U-Boot leaves
 * in DDR -- so the flash path is the whole boot, not just a bootloader.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/goke.h"
#include "hw/boards.h"
#include "hw/char/serial-mm.h"
#include "hw/i2c/i2c.h"
#include "hw/or-irq.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpregs.h"

/*
 * The HAL's VIC bit permutation (identical for both VICs and for every
 * VIC register): kernel IRQ bit n is silicon input irq_line[n].
 */
#define GK710X_IRQ_LINES {                              \
    26, 27, 28, 29, 30, 31,  0,  1,                     \
    18, 19, 20, 21, 22, 23, 24, 25,                     \
    10, 11, 12, 13, 14, 15, 16, 17,                     \
     2,  3,  4,  5,  6,  7,  8,  9 }

/*
 * GK7101: mach-gk710x/include/mach/{hardware,irqs}.h for the kernel IRQ
 * numbers, the HAL for where each block really is.  The fictitious ->
 * real block moves (AHB 0x9000xxxx -> 0x600xxxxx, APB 0xA00xxxxx ->
 * 0x700xxxxx) are noted beside each address.
 */
static const GkSoCConfig gk7101_soc = {
    .desc               = "Goke GK7101 (ARM1176, 128 MB DDR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm1176"),
    .ram_base           = 0xC0000000,
    .ram_size_default   = 128 * MiB,
    .board_id           = 3988,                 /* MACH_GOKE_IPC */
    .vic_base           = { 0x60003000,         /* VIC1, HAL 0x90008000 */
                            0x60010000 },       /* VIC2, HAL 0x90009000 */
    .irq_line           = GK710X_IRQ_LINES,
    /* IR_IRQ (32 + 30): the IR receiver's line is active low. */
    .num_idle_high_irqs = 1,
    .idle_high_irqs     = { 62 },
    .num_uarts          = 3,
    .uart_bases         = { 0x70005000,         /* HAL 0xA0005000 */
                            0x7001F000,         /* HAL 0xA001F000 */
                            0x7001E000 },       /* HAL 0xA001E000 */
    .uart_irqs          = { 31, 15, 27 },       /* UART0/1/2_IRQ */
    /* U-Boot: UART_BASE_FREQ = 48 MHz / 2 with PLL_SCALER_UART = 1. */
    .uart_clk           = 24000000,
    .timer_base         = 0x7000B000,           /* HAL 0xA0000000 */
    .timer_irqs         = { 18, 19, 20 },       /* TIMER1/2/3_IRQ */
    .apb_freq           = 69000000,
    .wdt_base           = 0x7000C000,           /* HAL 0xA0006000 */
    .wdt_irq            = 11,
    .sflash_base        = 0x60016000,           /* HAL 0x90016000 */
    /* A standard SDHCI underneath (U-Boot's goke_mmc pokes CLOCK_CONTROL
     * at +0x2C directly); the HAL shows it as 0x90000000. */
    .sdhci_base         = 0x60002000,
    .sdhci_irq          = 8,                    /* SD_IRQ */
    .usb_base           = 0x60006000,           /* HAL 0x90006000 */
    /* Synopsys GMAC: MAC at +0, DMA at +0x1000 (HAL 0x9000E000/F000,
     * which swaps a few registers of each). */
    .mac_base           = 0x6000E000,
    .mac_irq            = 1,                    /* ETH_IRQ */
    .ephy_base          = 0x60022000,           /* HAL 0x90022000 */
    /* Ambarella IDC masters (HAL 0xA0003000, 0xA0004000): the sensor's
     * bus, and the HDMI one. */
    .idc_bases          = { 0x70003000, 0x70007000 },
    .idc_irqs           = { 9, 58 },            /* IDC_IRQ, IDC_HDMI_IRQ */
    .num_regbanks       = 16,
    .regbanks           = {
        { "gk-ddrc",     0x60004000, 0x1000 },  /* HAL 0x90002000 */
        { "gk-dmac",     0x60005000, 0x1000 },  /* HAL 0x90003000 */
        { "gk-i2s",      0x6000A000, 0x1000 },  /* HAL 0x9000A000 */
        { "gk-efuse",    0x6000B000, 0x1000 },  /* HAL 0x90001000 */
        { "gk-crypt",    0x60014000, 0x1000 },  /* HAL 0x90005000 */
        /* AHB general regs, USB PHY, audio codec, DDR PHY. */
        { "gk-ahb-greg", 0x60020000, 0x2000 },
        { "gk-pmu",      0x60080000, 0x50000 }, /* PMU, RTC, 8051 code */
        { "gk-tssi",     0x70001000, 0x1000 },
        { "gk-ssi2",     0x70002000, 0x1000 },
        { "gk-gpio",     0x70009000, 0x1000 },
        { "gk-adc2",     0x7000D000, 0x1000 },  /* HAL 0xA0007000 */
        { "gk-ssi1",     0x7000F000, 0x1000 },  /* HAL 0xA0008000 */
        { "gk-pwm",      0x70010000, 0x4000 },  /* HAL 0xA000A000.. */
        { "gk-apb-110",  0x70110000, 0x20000 },
        { "gk-mem",      0x70150000, 0x20000 }, /* MEM, chip ID page */
        { "gk-rct",      0x70170000, 0x1000 },
    },
    .hal_addr           = 0xC0012000,
    /* U-Boot include/configs/gk7101.h, 128 MiB: mem=50M bsbsize=4M. */
    .kernel_mem_mb      = 50,
    .bsb_mb             = 4,
};

/* GK7102: the same die with 64 MiB of DDR in the package. */
static const GkSoCConfig *gk7102_config(void)
{
    static GkSoCConfig c;

    if (!c.desc) {
        c = gk7101_soc;
        c.desc = "Goke GK7102 (ARM1176, 64 MB DDR)";
        c.ram_size_default = 64 * MiB;
        /* What the GK7102 cameras' U-Boot passes: mem=47M, bsbsize=1M
         * (the SDK's 64 MiB default is 36M / 2M; the other dump, 37M). */
        c.kernel_mem_mb = 47;
        c.bsb_mb = 1;
    }
    return &c;
}

/*
 * GK7101S / GK7102S: the S die (the GK7102C cameras are this family too;
 * their U-Boot carries the SDK's gk7101s HAL byte for byte).  The blocks
 * are where mach-gk710xs/include/mach/hardware.h puts them; its kernel
 * drives all but the UARTs and SSI directly, in the drivers' register
 * layout.  The VIC, timers, watchdog, IDC and GMAC are the GK710x models
 * behind gk-remap, which applies the GK710x HAL's register moves and bit
 * permutations in front of them -- for the VIC that is the line wiring,
 * so the GK710x irq_line table holds here too.  The IRQ numbers
 * (mach-gk710xs irqs.h) are the GK710x ones.
 */
static const GkSoCConfig gk7101s_soc = {
    .desc               = "Goke GK7101S (ARM1176, 128 MB DDR)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm1176"),
    .ram_base           = 0xC0000000,
    .ram_size_default   = 128 * MiB,
    .board_id           = 3988,                 /* MACH_GOKE_IPC */
    .vic_base           = { 0x90008000, 0x90009000 },
    .irq_line           = GK710X_IRQ_LINES,
    .num_idle_high_irqs = 1,
    .idle_high_irqs     = { 62 },
    .num_uarts          = 3,
    .uart_bases         = { 0xA0005000, 0xA001F000, 0xA001E000 },
    .uart_irqs          = { 31, 15, 27 },       /* UART0/1/2_IRQ */
    /* U-Boot arch-gk7101s/serial.h: UART_BASE_FREQ = 48 MHz / 2; the boot
     * ROM programs DLL 13 for 115200 from it. */
    .uart_clk           = 24000000,
    .timer_base         = 0xA0000000,
    .timer_irqs         = { 18, 19, 20 },
    .apb_freq           = 69000000,             /* gk7101s/timer.c APB_CLK */
    .wdt_base           = 0xA0006000,
    .wdt_irq            = 11,
    .sflash_base        = 0x90016000,
    /* SD controllers at 0x90000000 / 0x90010000 are left as storage: the
     * S HAL passes the vendor's halfword register layout through, which is
     * not the standard SDHCI one the GK710x HAL makes of it. */
    .usb_base           = 0x90006000,
    .dma_base           = 0x90003000,
    .dma_irq            = 21,                   /* DMA_IRQ */
    .mac_base           = 0x9000E000,
    .mac_irq            = 1,
    .ephy_base          = 0x90022000,
    .idc_bases          = { 0xA0003000, 0xA0004000 },
    .idc_irqs           = { 9, 58 },
    .num_regbanks       = 18,
    .regbanks           = {
        { "gk-sdc",      0x90000000, 0x1000 },
        { "gk-efuse",    0x90001000, 0x1000 },
        { "gk-ddrc",     0x90002000, 0x1000 },
        { "gk-vo",       0x90004000, 0x1000 },
        { "gk-crypt",    0x90005000, 0x1000 },
        { "gk-i2s",      0x9000A000, 0x1000 },
        { "gk-sdc1",     0x90010000, 0x1000 },
        /* AHB general regs, ADC, audio codec, USB PHY, DDR PHY. */
        { "gk-ahb-greg", 0x90020000, 0x2000 },
        { "gk-pmu",      0x90080000, 0x50000 }, /* PMU, RTC, 8051 code */
        { "gk-tssi",     0xA0001000, 0x1000 },
        { "gk-ssi2",     0xA0002000, 0x1000 },
        { "gk-adc2",     0xA0007000, 0x1000 },
        { "gk-ssi1",     0xA0008000, 0x1000 },
        { "gk-gpio",     0xA0009000, 0x1000 },
        { "gk-pwm",      0xA000A000, 0x4000 },
        { "gk-apb-110",  0xA0110000, 0x20000 },
        { "gk-mem",      0xA0150000, 0x20000 },
        { "gk-rct",      0xA0170000, 0x1000 },
    },
    .hal_addr           = 0xC0012000,
    /* U-Boot include/configs/gk7101s.h, 128 MiB: mem=50M bsbsize=4M. */
    .kernel_mem_mb      = 50,
    .bsb_mb             = 4,
    .s_layout           = true,
};

/* GK7102S: 64 MiB in the package; gk7101s.h gives it mem=36M bsbsize=2M. */
static const GkSoCConfig *gk7102s_config(void)
{
    static GkSoCConfig c;

    if (!c.desc) {
        c = gk7101s_soc;
        c.desc = "Goke GK7102S / GK7102C (ARM1176, 64 MB DDR)";
        c.ram_size_default = 64 * MiB;
        c.kernel_mem_mb = 36;
        c.bsb_mb = 2;
    }
    return &c;
}

/* An abstract parent for both machines: the state and properties they
 * share, and a checked cast. */
#define TYPE_GK_MACHINE MACHINE_TYPE_NAME("gk")
OBJECT_DECLARE_SIMPLE_TYPE(GkMachineState, GK_MACHINE)

struct GkMachineState {
    MachineState parent_obj;
    char *flash_file;
    char *hal_file;
    char *sensor;
};

static char *gk_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(GK_MACHINE(obj)->flash_file);
}

static void gk_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    GkMachineState *s = GK_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static char *gk_machine_get_hal_file(Object *obj, Error **errp)
{
    return g_strdup(GK_MACHINE(obj)->hal_file);
}

static void gk_machine_set_hal_file(Object *obj, const char *value,
                                    Error **errp)
{
    GkMachineState *s = GK_MACHINE(obj);

    g_free(s->hal_file);
    s->hal_file = g_strdup(value);
}

static char *gk_machine_get_sensor(Object *obj, Error **errp)
{
    return g_strdup(GK_MACHINE(obj)->sensor);
}

static void gk_machine_set_sensor(Object *obj, const char *value,
                                  Error **errp)
{
    GkMachineState *s = GK_MACHINE(obj);

    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

/* Sensors a GK7102 board may carry that have a model: I2C type, 7-bit
 * address on IDC 0. */
static const struct {
    const char *name;
    const char *type;
    uint8_t addr;
} gk_sensors[] = {
    { "jxh42", "hisi-jxh42", 0x30 },
    { "jxh62", "hisi-jxh62", 0x30 },
    { "f37",   "hisi-f37",   0x40 },
};

/*
 * The mask ROM's boot header, in CPU words (the flash holds each word
 * byte-swapped, see gk-sflash.c):
 *
 *   w0..w1  "GK7101" + version     w2  U-Boot length in words
 *   w3      load and entry address w4, w5  (not used here)
 *   w6      0x5A5A5A5A             then (address, value) register writes,
 *   0xFFFFFFFE, address, value     = poll address until it reads value,
 *   0xFFFFFFFF, count              = a delay (the GK7102C ROM table waits
 *                                    like this between the bytes it sends
 *                                    out of UART0),
 *   0                              = end; U-Boot follows the 0.
 *
 * w1 is 0x30310801 on the GK710x, 0x30360801 on the SDK's GK710xS images,
 * 0x304308EF on the GK7102C cameras; the table itself names the registers,
 * so it is not checked.
 */
#define GK_BOOT_MAGIC       0x474B3731      /* "GK71" */
#define GK_BOOT_MARK        0x5A5A5A5A
#define GK_BOOT_POLL        0xFFFFFFFE
#define GK_BOOT_DELAY       0xFFFFFFFF

typedef struct GkBootRom {
    ARMCPU *cpu;
    uint32_t entry;
    uint32_t *writes;       /* address, value pairs */
    int num_writes;
} GkBootRom;

static GkBootRom gk_boot_rom;

static void gk_boot_rom_reset(void *opaque)
{
    GkBootRom *b = opaque;
    CPUState *cs = CPU(b->cpu);
    int n;

    cpu_reset(cs);
    /*
     * The DDR controller and PHY settings: the stubs keep them, which is
     * all any later code asks of them.  The ROM's polls wait for the DDR
     * controller and the eFuse loader to finish; both are instant here.
     */
    for (n = 0; n < b->num_writes; n++) {
        address_space_stl_le(&address_space_memory, b->writes[2 * n],
                             b->writes[2 * n + 1], MEMTXATTRS_UNSPECIFIED,
                             NULL);
    }
    cpu_set_pc(cs, b->entry);
}

/*
 * Parse the boot header of the flash image and stage its U-Boot in DDR,
 * where the mask ROM would copy it.  Returns false if the image has no
 * Goke boot header.
 */
static bool gk_boot_rom_setup(const char *path, const GkSoCConfig *c,
                              MachineState *machine, ARMCPU *cpu)
{
    g_autofree uint8_t *img = NULL;
    g_autofree uint32_t *code = NULL;
    gsize len;
    uint32_t nwords, load, i, end, w;
    GError *err = NULL;
    GArray *writes;

    if (!g_file_get_contents(path, (gchar **)&img, &len, &err)) {
        error_report("goke: cannot read %s: %s", path, err->message);
        g_error_free(err);
        exit(1);
    }
#define GKW(i) ldl_be_p(img + 4 * (i))
    if (len < 64 || GKW(0) != GK_BOOT_MAGIC || GKW(6) != GK_BOOT_MARK) {
        if (len >= 64 && ldl_le_p(img) == GK_BOOT_MAGIC) {
            error_report("goke: %s is in CPU byte order, as `sf write` takes "
                         "it or a software dump reads; the flash holds every "
                         "word byte-swapped, so swap them for flash-file=",
                         path);
            exit(1);
        }
        return false;
    }
    nwords = GKW(2);
    load = GKW(3);
    writes = g_array_new(false, false, sizeof(uint32_t));
    for (i = 7; ; ) {
        if (4 * (uint64_t)(i + 1) > len) {
            error_report("goke: %s: boot header write list runs past the "
                         "image", path);
            exit(1);
        }
        w = GKW(i);
        if (w == 0) {
            break;
        }
        /* A poll is three words, a write two; both must be whole. */
        if (4 * (uint64_t)(i + (w == GK_BOOT_POLL ? 3 : 2)) > len) {
            error_report("goke: %s: boot header write list runs past the "
                         "image", path);
            exit(1);
        }
        if (w == GK_BOOT_POLL) {
            i += 3;
            continue;
        }
        if (w == GK_BOOT_DELAY) {
            i += 2;
            continue;
        }
        g_array_append_val(writes, w);
        w = GKW(i + 1);
        g_array_append_val(writes, w);
        i += 2;
    }
    end = i + 1;
    if (4 * (uint64_t)(end + nwords) > len || load < c->ram_base ||
        load - c->ram_base + 4ull * nwords > machine->ram_size) {
        error_report("goke: %s: boot header does not fit (U-Boot of %u "
                     "words at 0x%08x)", path, nwords, load);
        exit(1);
    }
    code = g_new(uint32_t, nwords);
    for (i = 0; i < nwords; i++) {
        code[i] = cpu_to_le32(GKW(end + i));
    }
#undef GKW
    rom_add_blob_fixed("goke.u-boot", code, 4 * nwords, load);

    gk_boot_rom.cpu = cpu;
    gk_boot_rom.entry = load;
    gk_boot_rom.num_writes = writes->len / 2;
    gk_boot_rom.writes = (uint32_t *)g_array_free(writes, false);
    return true;
}

/*
 * -kernel: do what the vendor U-Boot leaves behind for the kernel.
 *
 * The kernel calls into the HAL from map_io on (gk710x.c, gk_map_io), at
 * the address U-Boot stores at 0xC0000008, and sizes its DSP / BSB / user
 * carve-outs from the other U2K words (U-Boot common/cmd_bootm.c,
 * CONFIG_UBOOT_TO_KERNEL).  The HAL is vendor code and not ours to ship:
 * it is taken from hal-file=, or else flash-file=, which may be any image
 * holding it -- a U-Boot binary, or a flash dump in its byte-swapped
 * order.  It starts with the hal_init prologue below and is at most the
 * 16 KiB below CONFIG_GK_HAL_ADDR + 0x4000 that U-Boot leaves free.
 */
#define GK_HAL_SIZE         0x4000
#define GK_U2K_PHY_TYPE     0x00
#define GK_U2K_ARM_FREQ     0x04
#define GK_U2K_HAL_ADDR     0x08
#define GK_U2K_SOC          0x10
#define GK_U2K_MEM          0x14
#define GK_U2K_BSB          0x18
#define GK_U2K_ENABLE       0x20
#define GK_U2K_USR          0x24
#define GK_U2K_TOTAL_MEM    0x28
#define GK_U2K_SIZE         0x2C

static const uint32_t gk_hal_prologue[2][4] = {
    {
        0xE92D45F8,     /* GK710x: push {r3-r8, sl, lr} */
        0xE1A04000,     /* mov r4, r0 */
        0xE1A05001,     /* mov r5, r1 */
        0xE1A06002,     /* mov r6, r2 */
    }, {
        0xE59F0048,     /* GK710xS: ldr r0, [pc, #0x48] */
        0xE59F3048,     /* ldr r3, [pc, #0x48] */
        0xE5830000,     /* str r0, [r3] */
        0xE59F3044,     /* ldr r3, [pc, #0x44] */
    },
};

static bool gk_find_hal(const char *path, uint8_t *hal, bool s_layout)
{
    g_autofree uint8_t *img = NULL;
    gsize len, off;
    int swap;

    if (!g_file_get_contents(path, (gchar **)&img, &len, NULL)) {
        return false;
    }
    for (swap = 0; swap < 2; swap++) {
        for (off = 0; off + GK_HAL_SIZE <= len; off += 4) {
            uint32_t w[4];
            int i;

            for (i = 0; i < 4; i++) {
                w[i] = swap ? ldl_be_p(img + off + 4 * i)
                            : ldl_le_p(img + off + 4 * i);
            }
            if (memcmp(w, gk_hal_prologue[s_layout], sizeof(w))) {
                continue;
            }
            for (i = 0; i < GK_HAL_SIZE; i += 4) {
                stl_le_p(hal + i, swap ? ldl_be_p(img + off + i)
                                       : ldl_le_p(img + off + i));
            }
            return true;
        }
    }
    return false;
}

/*
 * mem=, which these kernels cannot do without: before arm_load_kernel().
 * Linux runs from PHYS_OFFSET, 2 MiB into DDR, and the BSB and the 64 KiB
 * user page come out of what lies above it (gk_map_io), so mem= must leave
 * room for them in the DDR there is.
 */
#define GK_PPM_SIZE         (2 * MiB)
#define GK_USR_SIZE         (64 * KiB)

static uint32_t gk_kernel_mem(MachineState *machine, const GkSoCConfig *c)
{
    const char *cmdline = machine->kernel_cmdline ?: "";
    const char *mem = strstr(cmdline, "mem=");
    const uint64_t room = machine->ram_size > GK_PPM_SIZE + c->bsb_mb * MiB +
                          GK_USR_SIZE + MiB
                          ? machine->ram_size - GK_PPM_SIZE -
                            c->bsb_mb * MiB - GK_USR_SIZE
                          : 0;
    uint32_t mem_mb = MIN(c->kernel_mem_mb, room / MiB);

    if (!room) {
        error_report("goke: -m %" PRIu64 "M leaves no room for Linux",
                     machine->ram_size / MiB);
        exit(1);
    }
    if (mem) {
        const char *end;
        uint64_t bytes;

        if (qemu_strtosz_MiB(mem + 4, &end, &bytes) || bytes < MiB) {
            error_report("goke: cannot parse '%s'", mem);
            exit(1);
        }
        if (bytes > room) {
            error_report("goke: mem=%" PRIu64 "M does not fit: %" PRIu64
                         " MiB of DDR leave %" PRIu64 "M above the kernel's "
                         "2 MiB and the BSB", bytes / MiB,
                         machine->ram_size / MiB, room / MiB);
            exit(1);
        }
        return bytes / MiB;
    }
    machine->kernel_cmdline = g_strdup_printf("%s%smem=%uM", cmdline,
                                              *cmdline ? " " : "", mem_mb);
    return mem_mb;
}

static void gk_kernel_handoff(MachineState *machine, const GkSoCConfig *c,
                              GkMachineState *gms, uint32_t mem_mb)
{
    const char *src = gms->hal_file ? gms->hal_file : gms->flash_file;
    uint8_t u2k[GK_U2K_SIZE] = { 0 };
    g_autofree uint8_t *hal = g_malloc(GK_HAL_SIZE);

    if (!src) {
        error_report("goke: -kernel needs the vendor HAL the kernel calls "
                     "into: pass hal-file=<U-Boot or flash image>");
        exit(1);
    }
    if (!gk_find_hal(src, hal, c->s_layout)) {
        error_report("goke: no GK710x HAL found in %s", src);
        exit(1);
    }
    rom_add_blob_fixed("goke.hal", hal, GK_HAL_SIZE, c->hal_addr);

    stl_le_p(u2k + GK_U2K_PHY_TYPE, 0);             /* internal EPHY */
    stl_le_p(u2k + GK_U2K_ARM_FREQ, 0x00112032);    /* U-Boot's default */
    stl_le_p(u2k + GK_U2K_HAL_ADDR, c->hal_addr);
    stl_le_p(u2k + GK_U2K_SOC, 0);                  /* as U-Boot writes */
    stl_le_p(u2k + GK_U2K_MEM, mem_mb * MiB);
    stl_le_p(u2k + GK_U2K_BSB, c->bsb_mb * MiB);
    stl_le_p(u2k + GK_U2K_ENABLE, 0x7101);
    stl_le_p(u2k + GK_U2K_USR, 0);
    stl_le_p(u2k + GK_U2K_TOTAL_MEM, machine->ram_size / MiB);
    rom_add_blob_fixed("goke.u2k", u2k, sizeof(u2k), c->ram_base);
}

/*
 * ARM1176 TCM registers, absent from QEMU's arm1176: U-Boot's start code
 * selects each TCM bank (TCMSR) and reads its data and instruction TCM
 * region registers.  The GK710x core has no TCM, so the regions read as
 * size 0, disabled.
 */
static const ARMCPRegInfo gk_arm1176_tcm_cp_reginfo[] = {
    { .name = "DTCMRR", .cp = 15, .crn = 9, .crm = 1, .opc1 = 0, .opc2 = 0,
      .access = PL1_RW, .type = ARM_CP_CONST, .resetvalue = 0 },
    { .name = "ITCMRR", .cp = 15, .crn = 9, .crm = 1, .opc1 = 0, .opc2 = 1,
      .access = PL1_RW, .type = ARM_CP_CONST, .resetvalue = 0 },
    { .name = "TCMSR", .cp = 15, .crn = 9, .crm = 2, .opc1 = 0, .opc2 = 0,
      .access = PL1_RW, .type = ARM_CP_CONST, .resetvalue = 0 },
};

static qemu_irq gk_irq(DeviceState *vic[2], const GkSoCConfig *c, int n)
{
    return qdev_get_gpio_in(vic[n / GK_VIC_LINES],
                            c->irq_line[n % GK_VIC_LINES]);
}

/*
 * Map a model at base: as it is, or on the S die through gk-remap in the
 * named S register layout.
 */
static void gk_map_model(const GkSoCConfig *c, DeviceState *dev, hwaddr base,
                         const char *layout, uint32_t size)
{
    DeviceState *remap;

    if (!c->s_layout || !layout) {
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
        return;
    }
    remap = qdev_new("gk-remap");
    qdev_prop_set_string(remap, "layout", layout);
    qdev_prop_set_uint32(remap, "size", size);
    object_property_set_link(OBJECT(remap), "target", OBJECT(dev),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(remap), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(remap), 0, base);
}

static void goke_common_init(MachineState *machine, const GkSoCConfig *c)
{
    GkMachineState *gms = GK_MACHINE(machine);
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    DeviceState *vic[2];
    DeviceState *dev;
    uint32_t kernel_mem_mb = 0;
    Object *cpuobj;
    ARMCPU *cpu;
    int n;

    cpuobj = object_new(machine->cpu_type);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);
    define_arm_cp_regs(cpu, gk_arm1176_tcm_cp_reginfo);

    memory_region_add_subregion(sysmem, c->ram_base, machine->ram);

    for (n = 0; n < 2; n++) {
        uint32_t idle = 0;
        int i;

        for (i = 0; i < c->num_idle_high_irqs; i++) {
            if (c->idle_high_irqs[i] / GK_VIC_LINES == n) {
                int line = c->idle_high_irqs[i] % GK_VIC_LINES;

                idle |= 1u << c->irq_line[line];
            }
        }
        vic[n] = qdev_new("gk-vic");
        qdev_prop_set_uint32(vic[n], "idle-high", idle);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(vic[n]), &error_fatal);
        gk_map_model(c, vic[n], c->vic_base[n], "vic", 0x1000);
    }
    /* Both VICs drive the core's nIRQ / nFIQ (they are not daisy-chained:
     * the kernel scans VIC1, then VIC2, mach-gk710x/irq.c). */
    for (n = 0; n < 2; n++) {
        static const int cpu_line[2] = { ARM_CPU_IRQ, ARM_CPU_FIQ };
        DeviceState *or = qdev_new(TYPE_OR_IRQ);
        int v;

        qdev_prop_set_uint16(or, "num-lines", 2);
        qdev_realize_and_unref(or, NULL, &error_fatal);
        qdev_connect_gpio_out(or, 0,
                              qdev_get_gpio_in(DEVICE(cpu), cpu_line[n]));
        for (v = 0; v < 2; v++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(vic[v]), n,
                               qdev_get_gpio_in(or, v));
        }
    }

    for (n = 0; n < c->num_uarts; n++) {
        DeviceState *rb = qdev_new("hisi-regbank");

        serial_mm_init(sysmem, c->uart_bases[n], 2,
                       gk_irq(vic, c, c->uart_irqs[n]), c->uart_clk / 16,
                       serial_hd(n), DEVICE_LITTLE_ENDIAN);
        /* The DesignWare extras past the 16550 registers (the kernel
         * clears the software-reset register at +0x88). */
        qdev_prop_set_uint32(rb, "size", 0x1000 - 0x20);
        qdev_prop_set_string(rb, "name", "gk-uart-dw");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->uart_bases[n] + 0x20);
    }

    dev = qdev_new("gk-timer");
    qdev_prop_set_uint32(dev, "freq", c->apb_freq);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    gk_map_model(c, dev, c->timer_base, "timer", 0x1000);
    for (n = 0; n < 3; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), n,
                           gk_irq(vic, c, c->timer_irqs[n]));
    }

    dev = qdev_new("gk-wdt");
    qdev_prop_set_uint32(dev, "freq", c->apb_freq);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    gk_map_model(c, dev, c->wdt_base, "wdt", 0x1000);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, gk_irq(vic, c, c->wdt_irq));

    dev = qdev_new("gk-sflash");
    if (gms->flash_file) {
        qdev_prop_set_string(dev, "flash-file", gms->flash_file);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->sflash_base);

    if (c->sdhci_base) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);

        dev = qdev_new("generic-sdhci");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, c->sdhci_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           gk_irq(vic, c, c->sdhci_irq));
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(dev, "sd-bus"),
                                   &error_fatal);
        }
    }

    for (n = 0; n < 2; n++) {
        if (!c->idc_bases[n]) {
            continue;
        }
        dev = qdev_new("gk-idc");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        gk_map_model(c, dev, c->idc_bases[n], "idc", 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           gk_irq(vic, c, c->idc_irqs[n]));
        if (n == 0 && gms->sensor && strcmp(gms->sensor, "none")) {
            int i;

            for (i = 0; i < ARRAY_SIZE(gk_sensors); i++) {
                if (!strcmp(gms->sensor, gk_sensors[i].name)) {
                    break;
                }
            }
            if (i == ARRAY_SIZE(gk_sensors)) {
                error_report("goke: unknown sensor '%s' (none, jxh42, "
                             "jxh62, f37)", gms->sensor);
                exit(1);
            }
            i2c_slave_create_simple(I2C_BUS(qdev_get_child_bus(dev, "i2c")),
                                    gk_sensors[i].type, gk_sensors[i].addr);
        }
    }

    if (c->mac_base) {
        dev = qdev_new("xm-dwmac");
        qemu_configure_nic_device(dev, true, NULL);
        qdev_prop_set_uint32(dev, "regs-size", 0x2000);
        /* The GK710x core keeps the FCS: the kernel driver takes 4 off
         * every received length (gk_eth_v1_00.c), U-Boot's does not. */
        qdev_prop_set_bit(dev, "rx-fcs", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        gk_map_model(c, dev, c->mac_base, "gmac", 0x2000);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           gk_irq(vic, c, c->mac_irq));
    }
    if (c->ephy_base) {
        sysbus_create_simple("gk-ephy", c->ephy_base, NULL);
    }

    if (c->dma_base) {
        sysbus_create_simple("gk-dma", c->dma_base,
                             gk_irq(vic, c, c->dma_irq));
    }

    if (c->usb_base) {
        /*
         * Mentor MUSB OTG core.  Nothing is ever plugged in, so plain
         * memory does, as long as CONFIGDATA (+0x1F with INDEX 0) reports
         * dynamic FIFO sizing and soft connect: with 0 there, the vendor
         * driver sizes FIFOs from the per-endpoint FIFOSIZE registers,
         * finds no bulk endpoint, fails the probe -- and its error path
         * oopses (gk7101_dma_controller_stop on a NULL controller).
         */
        MemoryRegion *usb = g_new(MemoryRegion, 1);
        uint8_t *regs;

        memory_region_init_ram(usb, NULL, "gk.usb", 0x2000, &error_fatal);
        memory_region_add_subregion(sysmem, c->usb_base, usb);
        regs = memory_region_get_ram_ptr(usb);
        regs[0x1F] = 0x06;              /* CONFIGDATA: DYNFIFO | SOFTCONE */
        stw_le_p(regs + 0x6C, 0x0800);  /* HWVERS 2.0 */
    }

    for (n = 0; n < c->num_regbanks; n++) {
        DeviceState *rb = qdev_new("hisi-regbank");

        qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
        qdev_prop_set_string(rb, "name", c->regbanks[n].name);
        /* The HAL has byte and halfword accessors for every block. */
        qdev_prop_set_bit(rb, "sub-word", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);
    }

    /*
     * Everything else in the real AHB / APB windows, and the fictitious
     * ones the HAL passes through untranslated for blocks it does not
     * know: -d unimp shows what the vendor code still reaches for.
     */
    create_unimplemented_device("gk.ahb", 0x60000000, 0x01000000);
    create_unimplemented_device("gk.apb", 0x70000000, 0x01000000);
    create_unimplemented_device("gk.hal-ahb", 0x90000000, 0x01000000);
    create_unimplemented_device("gk.hal-apb", 0xA0000000, 0x01000000);

    if (machine->kernel_filename) {
        kernel_mem_mb = gk_kernel_mem(machine, c);
    }
    /*
     * QEMU's boot stub and ATAGs go 1 MiB in, where U-Boot's would: still
     * in the kernel's private 2 MiB (PPM) below PHYS_OFFSET 0xC0200000,
     * but clear of the U2K words at 0xC0000000 and the HAL at 0xC0012000.
     * ATAG_MEM is moot: the kernel needs mem= and takes that instead.
     */
    binfo.ram_size = machine->ram_size - 1 * MiB;
    binfo.loader_start = c->ram_base + 1 * MiB;
    binfo.board_id = c->board_id;
    /* Secure SVC, as the vendor U-Boot leaves it: nothing on these parts
     * ever leaves the secure world. */
    binfo.secure_boot = true;
    arm_load_kernel(cpu, machine, &binfo);

    if (machine->kernel_filename) {
        gk_kernel_handoff(machine, c, gms, kernel_mem_mb);
    }

    if (!machine->kernel_filename && gms->flash_file) {
        if (!gk_boot_rom_setup(gms->flash_file, c, machine, cpu)) {
            error_report("goke: %s has no GK7101 boot header",
                         gms->flash_file);
            exit(1);
        }
        /* After arm_load_kernel(): its per-CPU reset hook would put the
         * PC back to 0. */
        qemu_register_reset(gk_boot_rom_reset, &gk_boot_rom);
    }
}

static void gk_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->default_ram_id = "goke.ram";
    mc->ignore_memory_transaction_failures = true;
    object_class_property_add_str(oc, "flash-file",
        gk_machine_get_flash_file, gk_machine_set_flash_file);
    object_class_property_set_description(oc, "flash-file",
        "SPI NOR image as dumped from the camera (boot header at 0); "
        "erases and programs are written back to it");
    object_class_property_add_str(oc, "sensor",
        gk_machine_get_sensor, gk_machine_set_sensor);
    object_class_property_set_description(oc, "sensor",
        "image sensor on I2C 0: none (default), jxh42, jxh62, f37");
    object_class_property_add_str(oc, "hal-file",
        gk_machine_get_hal_file, gk_machine_set_hal_file);
    object_class_property_set_description(oc, "hal-file",
        "image holding the vendor HAL for -kernel boots (a U-Boot "
        "binary or flash dump; default: flash-file)");
}

static const TypeInfo gk_machine_info = {
    .name           = TYPE_GK_MACHINE,
    .parent         = TYPE_MACHINE,
    .abstract       = true,
    .instance_size  = sizeof(GkMachineState),
    .class_init     = gk_machine_class_init,
    .interfaces     = arm_machine_interfaces,
};

static void gk_machine_register_types(void)
{
    type_register_static(&gk_machine_info);
}

type_init(gk_machine_register_types)

#define DEFINE_GK_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        goke_common_init(machine, &config);                          \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu_type;                      \
        mc->default_ram_size = config.ram_size_default;              \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, GK_MACHINE, GkMachineState,     \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_GK_MACHINE("gk7101", gk7101, gk7101_soc)
DEFINE_GK_MACHINE("gk7102", gk7102, (*gk7102_config()))
DEFINE_GK_MACHINE("gk7101s", gk7101s, gk7101s_soc)
DEFINE_GK_MACHINE("gk7102s", gk7102s, (*gk7102s_config()))
