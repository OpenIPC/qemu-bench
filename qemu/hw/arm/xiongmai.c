/*
 * Xiongmai XM510 / XM530 / XM550 IP-camera SoC machines.
 *
 * Table-driven like hisilicon.c: one XmSoCConfig per SoC and one shared
 * init.  See include/hw/arm/xiongmai.h for why this is a family of its own.
 *
 * Boots the OpenIPC xm510_lite kernel (vendor 3.0.101, ATAGs, MACH 8888)
 * and the xm530_lite one (vendor 3.10.103, same machine number) with
 * -kernel/-initrd to a shell, and a camera's own flash through its U-Boot.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/xiongmai.h"
#include "hw/boards.h"
#include "hw/char/pl011.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"
#include "qobject/qlist.h"
#include "target/arm/arm-powerctl.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"

/*
 * XM510: one Cortex-A5 (MIDR 410fc051), no L2, 32 MB DDR at 0x80000000.
 * QEMU has no Cortex-A5 model; the 3.0 kernel is generic ARMv7 (proc-v7)
 * and runs on the Cortex-A9 stand-in, which only differs in MIDR here.
 *
 * mach-xm510/include/mach/platform.h, irqs.h, time.h.
 */
static const XmSoCConfig xm510_soc = {
    .desc               = "Xiongmai XM510 (Cortex-A5 stand-in A9, PL190)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .ram_base           = 0x80000000,
    .ram_size_default   = 32 * MiB,
    .board_id           = 8888,
    .sys_base           = 0x20000000,           /* SYS_CTRL_BASE */
    .sys_rsten          = 0x30,
    .sys_rst            = 0x34,
    /* PLL status words polled by U-Boot 2014.04's lowlevel init. */
    .sys_pll_lock_mask  = (1u << (0x08 / 4)) | (1u << (0x1c / 4)) |
                          (1u << (0x28 / 4)),
    /* U-Boot sets sp = 0x04001000 before DDR training. */
    .sram_base          = 0x04000000,
    .sram_size          = 64 * KiB,
    .vic_base           = 0x20010000,           /* INTC_BASE */
    .num_uarts          = 3,
    .uart_bases         = { 0x10030000, 0x10040000, 0x10050000 },
    .uart_irqs          = { 0, 1, 2 },
    .num_timers         = 2,
    .timer_bases        = { 0x100C0000, 0x100D0000 },   /* TIMER01, TIMER23 */
    .timer_irqs         = { 4, 5 },
    .timer_freq         = 24000000,             /* CFG_TIMER_CLK */
    .sfc_base           = 0x100F0000,           /* XMSFC_REG_BASE_ADDR */
    .sfc_window         = 0x00000000,           /* XMSF_BASE_ADDR, 16 MiB */
    .mac_base           = 0x20040000,           /* GMAC_BASE */
    .mac_irq            = 3,
    .num_sdhci          = 2,
    .sdhci_bases        = { 0x50000000, 0x50010000 },
    .sdhci_irqs         = { 24, 25 },
    .wdt_base           = 0x20060000,
    /* xm_wdt.ko loads 93750 * t / 2 for a t-second timeout, and an SP805
     * resets on its second expiry: a 24 MHz / 256 clock. */
    .wdt_freq           = 93750,
    .ddrc_base          = 0x100B0000,           /* from U-Boot, see xm-ddrc */
    .num_ddrc_done      = 2,
    .ddrc_done          = { 0x0048, 0x1038 },
    /* Banked, eight pins a port: xm510-gpio, not XM530's xm-gpio. */
    .gpio_base          = 0x10020000,
    .gpio_type          = "xm510-gpio",
    .num_regbanks       = 3,
    /* The XM530 I2C controller; XM510 boards carry an SOI JX-H62, which
     * ipcinfo and the vendor libsns find at 0x30. */
    .i2c_base           = 0x10000000,
    .sensor_type        = "hisi-jxh62",
    .sensor_addr        = 0x30,
    /* xm510_vi.ko and the ISP share 0x40000000: vi_back_v is IRQ 18, and
     * vi_front_v with isp_irq IRQ 16. */
    .vi_base            = 0x40000000,
    .vi_layout          = "xm510",
    .vi_irq             = 18,
    .vi_irqs            = 1,
    .isp_irq            = 16,
    .isp_irqs           = 1,
    /* hx280enc on XM510: base_port=0x20070000 irq=26. */
    .h1_base            = 0x20070000,
    .h1_irq             = 26,
    .regbanks           = {
        { "xm-dma",      0x20020000, 0x10000 },  /* PL330, until modelled */
        { "xm-rtc",      0x100A0000, 0x10000 },
        { "xm-i2s",      0x10060000, 0x10000 },
    },
};

/*
 * XM530: two Cortex-A5 in an MPCore with the A9's private-region layout
 * (SCU, GIC CPU interface, TWD, GIC distributor), a PL310, 64 MB DDR.
 * The device lines keep their XM510 numbers, now as GIC SPIs.  The flash
 * window moved to 0x60000000 with the v2 controller; address 0 aliases it
 * at reset and becomes RAM once the kernel sets REG_SC_REMAP.
 *
 * mach-xm530/include/mach/platform.h, irqs.h; core.c (clocks, TWD rate
 * from the PLL words); platsmp.c, hotplug.c (CPU1 release).
 */
static const XmSoCConfig xm530_soc = {
    .desc               = "Xiongmai XM530 (2x Cortex-A5 stand-in A9, GIC)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a9"),
    .max_cpus           = 2,
    .ram_base           = 0x80000000,
    .ram_size_default   = 64 * MiB,
    .board_id           = 8888,
    .sys_base           = 0x20000000,
    .sys_rsten          = 0x80,
    .sys_rst            = 0x84,
    .sys_remap          = 0x204,                /* REG_SC_REMAP */
    .sys_cpu1_rst       = 0x8C,                 /* REG_CPU1_SOFT_RST */
    /*
     * core.c: pll = 12 MHz / refdiv * fbdiv / (postdiv + 1) from PLLA_CTRL,
     * twd = pll / (div + 1) from CPUCLK_CTRL.  QEMU's TWD ticks at 100 MHz
     * (10 ns per prescaler step), so: refdiv 1, fbdiv 100 -> 1.2 GHz, /12.
     */
    .num_sys_init       = 2,
    .sys_init_offsets   = { 0x08, 0x28 },
    .sys_init_values    = { (100 << 6) | 1, 11 },
    /* U-Boot 2014.04 ("Mar 15 2019") programs four PLLs and spins until
     * each control word reads negative: bit 31 is LOCK. */
    .sys_pll_lock31_mask = (1u << (0x08 / 4)) | (1u << (0x10 / 4)) |
                           (1u << (0x18 / 4)) | (1u << (0x20 / 4)),
    /* U-Boot's first stack, sp = 0x04001000 as on XM510. */
    .sram_base          = 0x04000000,
    .sram_size          = 64 * KiB,
    .mpcore_base        = 0x20200000,           /* ARM_INTNL_BASE */
    .num_spi            = 64,
    .l2x0_base          = 0x20210000,           /* REG_BASE_L2CACHE */
    .num_uarts          = 3,
    .uart_bases         = { 0x10030000, 0x10040000, 0x10050000 },
    .uart_irqs          = { 0, 1, 2 },
    .num_timers         = 2,
    .timer_bases        = { 0x100C0000, 0x100D0000 },
    .timer_irqs         = { 4, 5 },
    .timer_freq         = 12000000,             /* sp804_clk */
    .sfc_base           = 0x100F4000,           /* XM_SFC_V2_CTRL */
    .sfc_window         = 0x60000000,           /* XMSF_BASE_ADDR */
    .sfc_version        = 2,
    /*
     * The XM550 U-Boot 2014.04 ("Mar 15 2019") is linked at 0x08000000, and
     * a flash boot starts CPU0 at this alias (xm_flash_boot_reset): before
     * it relocates, U-Boot runs here and reads init_sequence (0x08022cbc)
     * and its board tables through it.  Without the alias that walk reads
     * 0, every init step is skipped -- dram_init included -- and U-Boot
     * relocates below 0x80000000.
     */
    .sfc_text_alias     = 0x08000000,
    .mac_base           = 0x10010000,           /* GMAC_BASE */
    .mac_irq            = 3,
    .num_sdhci          = 2,
    .sdhci_bases        = { 0x50000000, 0x50100000 },
    .sdhci_irqs         = { 41, 42 },
    /* xm_i2c.ko's bus 0, with a Sony IMX335 at 0x1A (ipctool probes 0x34
     * >> 1) -- the sensor of the lab XM550 and OpenIPC's X50 profile. */
    .i2c_base           = 0x10000000,
    .sensor_type        = "hisi-imx335",
    .sensor_addr        = 0x1A,
    /* xm530_vi.ko: ioremap(0x30000000, 1 MiB), IRQ 65..68 for chn 0..3. */
    .vi_base            = 0x30000000,
    .vi_layout          = "xm530",
    .vi_irq             = 33,
    .vi_irqs            = 4,
    .isp_irq            = 25,
    .isp_irqs           = 2,
    /* hx280enc's module defaults: base_port=0x20070000 irq=77 (SPI 45). */
    .h1_base            = 0x20070000,
    .h1_irq             = 45,
    .wdt_base           = 0x20060000,
    /* xm_wdt.ko: 46875 * t / 2, so 12 MHz / 256.  Unfed, it resets the
     * camera 60 s after insmod (the default timeout), as in the lab. */
    .wdt_freq           = 46875,
    .ddrc_base          = 0x100B0000,           /* same uPCTL as XM510 */
    /* XM550 U-Boot: spins on +0xF4, then +0xF8, reading 1. */
    .num_ddrc_done      = 2,
    .ddrc_done          = { 0x00F4, 0x00F8 },
    /* ...then programs PHY +0xA8 and waits for +0xB8 to echo it. */
    .ddrc_mirror        = 0x10B8,
    .ddrc_mirror_of     = 0x10A8,
    .gpio_base          = 0x10020000,
    .num_regbanks       = 5,
    .regbanks           = {
        { "xm-i2c1",     0x10001000, 0x0F000 },  /* xm_i2c1, unmodelled */
        { "xm-dma",      0x20020000, 0x10000 },  /* DMAC_BASE */
        { "xm-rtc",      0x100A0000, 0x10000 },
        { "xm-i2s",      0x10060000, 0x10000 },
        { "xm-isp",      0x40000000, 0x400000 }, /* ISP / VI, 0x4004..0x4020 */
    },
};

/* XM550: the XM530 die with 128 MB DDR. */
static const XmSoCConfig *xm550_config(void)
{
    static XmSoCConfig c;

    if (!c.desc) {
        c = xm530_soc;
        c.desc = "Xiongmai XM550 (2x Cortex-A5 stand-in A9, GIC, 128 MB)";
        c.ram_size_default = 128 * MiB;
    }
    return &c;
}

typedef struct XmMachineState {
    MachineState parent_obj;
    char *flash_file;
} XmMachineState;

#define XM_MACHINE(obj) ((XmMachineState *)(obj))

static char *xm_machine_get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(XM_MACHINE(obj)->flash_file);
}

static void xm_machine_set_flash_file(Object *obj, const char *value,
                                      Error **errp)
{
    XmMachineState *s = XM_MACHINE(obj);
    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

/*
 * REG_SC_REMAP: address 0 is the boot flash until the guest asks for RAM
 * there, which the kernel does to plant CPU1's trampoline.
 */
static void xm_remap(void *opaque, int n, int level)
{
    memory_region_set_enabled(opaque, level);
}

/*
 * REG_CPU1_SOFT_RST: CPU1 leaves reset at PC 0, in the state CPU0 runs in
 * (non-secure SVC: these CPUs have no EL3 here).
 */
static void xm_cpu1_on(CPUState *cs, run_on_cpu_data data)
{
    const int ret = arm_set_cpu_on(arm_cpu_mp_affinity(ARM_CPU(cs)),
                                   0, 0, 1, false);

    if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm: CPU1 release failed (%d)\n", ret);
    }
}

static void xm_cpu1_reset(void *opaque, int n, int level)
{
    ARMCPU *cpu1 = ARM_CPU(opaque);
    const uint64_t mpidr = arm_cpu_mp_affinity(cpu1);
    int ret;

    if (level) {
        ret = arm_set_cpu_off(mpidr);
        if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS &&
            ret != QEMU_ARM_POWERCTL_IS_OFF) {
            qemu_log_mask(LOG_GUEST_ERROR, "xm: CPU1 hold failed (%d)\n", ret);
        }
        return;
    }
    ret = arm_set_cpu_on(mpidr, 0, 0, 1, false);
    if (ret == QEMU_ARM_POWERCTL_ALREADY_ON) {
        /*
         * Hold and release are two back-to-back register writes, and the
         * power-off a hold asks for runs later, on CPU1 itself.  A release
         * that finds CPU1 still on queues behind that power-off on the same
         * CPU, so it lands once CPU1 really is off.
         */
        async_run_on_cpu(CPU(cpu1), xm_cpu1_on, RUN_ON_CPU_NULL);
    } else if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm: CPU1 release failed (%d)\n", ret);
    }
}

/*
 * Flash boot on XM530: U-Boot is linked at the text alias (0x08000000) and
 * is not position-independent across its relocation -- it returns into the
 * relocated copy at "here + reloc_off", which only lands if it was already
 * running at its link address.  So on silicon the CPU starts there, which
 * points at a mask ROM jumping to flash; nothing documents it, and the
 * U-Boot is the evidence.  Without -kernel, reset starts CPU0 there.
 */
static hwaddr xm_flash_boot_pc;

static void xm_flash_boot_reset(void *opaque)
{
    CPUState *cs = CPU(opaque);

    cpu_reset(cs);
    cpu_set_pc(cs, xm_flash_boot_pc);
}

/* Secondaries wait for REG_CPU1_SOFT_RST, never for boot.c's pen: at 0 it
 * would overwrite the trampoline the kernel writes there itself. */
static void xm_write_secondary_boot(ARMCPU *cpu,
                                    const struct arm_boot_info *info)
{
}

static void xm_reset_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
}

static void xiongmai_common_init(MachineState *machine, const XmSoCConfig *c)
{
    DeviceState *sys = NULL;
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    const int smp_cpus = machine->smp.cpus;
    qemu_irq pic[XM_MAX_IRQ];
    ARMCPU *cpus[2] = { NULL };
    ARMCPU *cpu;
    int n;

    for (n = 0; n < smp_cpus; n++) {
        Object *o = object_new(machine->cpu_type);
        /* The vendor kernel is entered in SVC by U-Boot; no secure world. */
        if (object_property_find(o, "has_el3")) {
            object_property_set_bool(o, "has_el3", false, &error_fatal);
        }
        if (object_property_find(o, "reset-cbar")) {
            object_property_set_int(o, "reset-cbar", c->mpcore_base,
                                    &error_fatal);
        }
        /* CPU1 waits for REG_CPU1_SOFT_RST, as on silicon. */
        if (n > 0) {
            object_property_set_bool(o, "start-powered-off", true,
                                     &error_fatal);
        }
        qdev_realize(DEVICE(o), NULL, &error_fatal);
        cpus[n] = ARM_CPU(o);
    }
    cpu = cpus[0];

    memory_region_add_subregion(sysmem, c->ram_base, machine->ram);

    if (c->sram_size) {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "xiongmai.sram", c->sram_size,
                               &error_fatal);
        memory_region_add_subregion(sysmem, c->sram_base, sram);
    }

    if (c->mpcore_base) {
        DeviceState *mp = qdev_new("a9mpcore_priv");
        SysBusDevice *mpbus = SYS_BUS_DEVICE(mp);

        qdev_prop_set_uint32(mp, "num-cpu", smp_cpus);
        qdev_prop_set_uint32(mp, "num-irq", c->num_spi + 32);
        sysbus_realize_and_unref(mpbus, &error_fatal);
        sysbus_mmio_map(mpbus, 0, c->mpcore_base);
        /* Outputs grouped per kind: IRQ 0..N-1, FIQ N..2N-1, ... */
        for (n = 0; n < smp_cpus; n++) {
            sysbus_connect_irq(mpbus, n,
                qdev_get_gpio_in(DEVICE(cpus[n]), ARM_CPU_IRQ));
            sysbus_connect_irq(mpbus, n + smp_cpus,
                qdev_get_gpio_in(DEVICE(cpus[n]), ARM_CPU_FIQ));
            sysbus_connect_irq(mpbus, n + 2 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpus[n]), ARM_CPU_VIRQ));
            sysbus_connect_irq(mpbus, n + 3 * smp_cpus,
                qdev_get_gpio_in(DEVICE(cpus[n]), ARM_CPU_VFIQ));
        }
        for (n = 0; n < c->num_spi; n++) {
            pic[n] = qdev_get_gpio_in(mp, n);
        }
    } else {
        /* PL190-layout VIC: the vendor entry macro reads IRQSTATUS at +0
         * and nothing else, and init writes INTENCLEAR/INTSELECT/
         * SOFTINTCLEAR at the PL190 offsets (REG_INTC_* in platform.h). */
        DeviceState *vic = sysbus_create_varargs("pl190", c->vic_base,
                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ),
                NULL);
        for (n = 0; n < 32; n++) {
            pic[n] = qdev_get_gpio_in(vic, n);
        }
    }

    if (c->l2x0_base) {
        sysbus_create_simple("l2x0", c->l2x0_base, NULL);
    }

    if (c->sys_base) {
        sys = qdev_new("xm-sysctl");
        qdev_prop_set_uint32(sys, "rsten-offset", c->sys_rsten);
        qdev_prop_set_uint32(sys, "rst-offset", c->sys_rst);
        qdev_prop_set_uint32(sys, "pll-lock-mask", c->sys_pll_lock_mask);
        qdev_prop_set_uint32(sys, "pll-lock31-mask", c->sys_pll_lock31_mask);
        qdev_prop_set_uint32(sys, "remap-offset", c->sys_remap);
        qdev_prop_set_uint32(sys, "cpu1-offset", c->sys_cpu1_rst);
        if (c->num_sys_init) {
            QList *offs = qlist_new(), *vals = qlist_new();
            for (n = 0; n < c->num_sys_init; n++) {
                qlist_append_int(offs, c->sys_init_offsets[n]);
                qlist_append_int(vals, c->sys_init_values[n]);
            }
            qdev_prop_set_array(sys, "init-offsets", offs);
            qdev_prop_set_array(sys, "init-values", vals);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sys), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sys), 0, c->sys_base);
    }

    if (c->ddrc_base) {
        DeviceState *ddrc = qdev_new("xm-ddrc");
        QList *done = qlist_new();

        for (n = 0; n < c->num_ddrc_done; n++) {
            qlist_append_int(done, c->ddrc_done[n]);
        }
        qdev_prop_set_array(ddrc, "done-offsets", done);
        if (c->ddrc_mirror) {
            QList *m = qlist_new(), *mo = qlist_new();

            qlist_append_int(m, c->ddrc_mirror);
            qlist_append_int(mo, c->ddrc_mirror_of);
            qdev_prop_set_array(ddrc, "mirror-reads", m);
            qdev_prop_set_array(ddrc, "mirror-of", mo);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ddrc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ddrc), 0, c->ddrc_base);
    }

    for (n = 0; n < c->num_uarts; n++) {
        pl011_create(c->uart_bases[n], pic[c->uart_irqs[n]], serial_hd(n));
    }

    for (n = 0; n < c->num_timers; n++) {
        DeviceState *t = qdev_new("sp804");
        qdev_prop_set_uint32(t, "freq0", c->timer_freq);
        qdev_prop_set_uint32(t, "freq1", c->timer_freq);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(t), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(t), 0, c->timer_bases[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(t), 0, pic[c->timer_irqs[n]]);
    }

    if (c->sfc_base) {
        XmMachineState *ms = XM_MACHINE(machine);
        DeviceState *sfc = qdev_new("xm-sfc");
        if (ms->flash_file) {
            qdev_prop_set_string(sfc, "flash-file", ms->flash_file);
        }
        qdev_prop_set_uint32(sfc, "version", c->sfc_version ? c->sfc_version : 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sfc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sfc), 0, c->sfc_base);
        sysbus_mmio_map(SYS_BUS_DEVICE(sfc), 1, c->sfc_window);

        if (c->sfc_window && c->sys_remap && sys) {
            MemoryRegion *win = sysbus_mmio_get_region(SYS_BUS_DEVICE(sfc), 1);
            MemoryRegion *boot = g_new(MemoryRegion, 1);
            MemoryRegion *low = g_new(MemoryRegion, 1);
            const uint64_t span = MIN(memory_region_size(win),
                                      memory_region_size(machine->ram));

            memory_region_init_alias(boot, NULL, "xiongmai.boot-alias", win, 0,
                                     memory_region_size(win));
            memory_region_add_subregion(sysmem, 0, boot);
            /* Remapped, address 0 is DDR itself, as REG_SC_REMAP is on the
             * HiSilicon parts these were templated on: what the kernel
             * writes there is what CPU1 fetches, and is in DDR too. */
            memory_region_init_alias(low, NULL, "xiongmai.remap-ddr",
                                     machine->ram, 0, span);
            memory_region_add_subregion_overlap(sysmem, 0, low, 1);
            memory_region_set_enabled(low, false);
            qdev_connect_gpio_out_named(sys, "remap", 0,
                                        qemu_allocate_irq(xm_remap, low, 0));
        }
        if (c->sfc_text_alias) {
            MemoryRegion *win = sysbus_mmio_get_region(SYS_BUS_DEVICE(sfc), 1);
            MemoryRegion *text = g_new(MemoryRegion, 1);

            memory_region_init_alias(text, NULL, "xiongmai.text-alias", win, 0,
                                     memory_region_size(win));
            memory_region_add_subregion(sysmem, c->sfc_text_alias, text);
            if (!machine->kernel_filename) {
                xm_flash_boot_pc = c->sfc_text_alias;
            }
        }
    }

    if (sys && c->sys_cpu1_rst && smp_cpus > 1) {
        qdev_connect_gpio_out_named(sys, "cpu1-reset", 0,
                                    qemu_allocate_irq(xm_cpu1_reset, cpus[1], 0));
    }

    if (c->mac_base) {
        DeviceState *mac = qdev_new("xm-dwmac");
        qemu_configure_nic_device(mac, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(mac), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(mac), 0, c->mac_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(mac), 0, pic[c->mac_irq]);
    }

    for (n = 0; n < c->num_sdhci; n++) {
        DeviceState *sd = qdev_new("generic-sdhci");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sd), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sd), 0, c->sdhci_bases[n]);
        sysbus_connect_irq(SYS_BUS_DEVICE(sd), 0, pic[c->sdhci_irqs[n]]);

        DriveInfo *di = drive_get(IF_SD, 0, n);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(sd, "sd-bus"),
                                   &error_fatal);
        }
    }

    if (c->i2c_base) {
        DeviceState *i2c = sysbus_create_simple("xm-i2c", c->i2c_base, NULL);

        if (c->sensor_type) {
            i2c_slave_create_simple(I2C_BUS(qdev_get_child_bus(i2c, "i2c")),
                                    c->sensor_type, c->sensor_addr);
        }
    }

    if (c->vi_base) {
        DeviceState *vi = qdev_new("xm-vi");

        qdev_prop_set_string(vi, "soc", c->vi_layout);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(vi), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(vi), 0, c->vi_base);
        for (n = 0; n < c->vi_irqs; n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(vi), n, pic[c->vi_irq + n]);
        }
        for (n = 0; n < c->isp_irqs; n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(vi), 4 + n, pic[c->isp_irq + n]);
        }
    }

    if (c->h1_base) {
        sysbus_create_simple("xm-h1", c->h1_base, pic[c->h1_irq]);
    }

    if (c->gpio_base) {
        /* A named child, so qtest and a board can reach its input lines at
         * /machine/xm-gpio, whichever of the two layouts the part has. */
        DeviceState *gpio = qdev_new(c->gpio_type ? c->gpio_type : "xm-gpio");

        object_property_add_child(OBJECT(machine), "xm-gpio", OBJECT(gpio));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 0, c->gpio_base);
    }

    if (c->wdt_base) {
        Clock *wdt_clk = clock_new(OBJECT(machine), "wdt-clk");
        clock_set_hz(wdt_clk, c->wdt_freq);
        DeviceState *wdt = qdev_new("cmsdk-apb-watchdog");
        qdev_connect_clock_in(wdt, "WDOGCLK", wdt_clk);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(wdt), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(wdt), 0, c->wdt_base);
    }

    for (n = 0; n < c->num_regbanks; n++) {
        DeviceState *rb = qdev_new("hisi-regbank");
        qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
        qdev_prop_set_string(rb, "name", c->regbanks[n].name);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);
    }

    binfo.ram_size = machine->ram_size;
    binfo.loader_start = c->ram_base;
    binfo.board_id = c->board_id;
    binfo.write_secondary_boot = xm_write_secondary_boot;
    binfo.secondary_cpu_reset_hook = xm_reset_secondary;
    arm_load_kernel(cpu, machine, &binfo);
    /* After arm_load_kernel(): its own per-CPU reset hook runs in
     * registration order and would put the PC back to 0. */
    if (xm_flash_boot_pc) {
        qemu_register_reset(xm_flash_boot_reset, cpu);
    }
}

#define DEFINE_XM_MACHINE(namestr, tag, config)                      \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        xiongmai_common_init(machine, &config);                      \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu_type;                      \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "xiongmai.ram";                         \
        mc->max_cpus = config.max_cpus ? config.max_cpus : 1;        \
        mc->ignore_memory_transaction_failures = true;               \
        object_class_property_add_str(OBJECT_CLASS(mc), "flash-file", \
            xm_machine_get_flash_file, xm_machine_set_flash_file);   \
        object_class_property_set_description(OBJECT_CLASS(mc),      \
            "flash-file", "SPI NOR image behind the xmsfc controller; " \
            "erases and programs are written back to it");            \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, MACHINE, XmMachineState,        \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_XM_MACHINE("xm510", xm510, xm510_soc)
DEFINE_XM_MACHINE("xm530", xm530, xm530_soc)
DEFINE_XM_MACHINE("xm550", xm550, (*xm550_config()))
