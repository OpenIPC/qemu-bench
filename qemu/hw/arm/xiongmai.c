/*
 * Xiongmai XM510 / XM530 / XM550 IP-camera SoC machines.
 *
 * Table-driven like hisilicon.c: one XmSoCConfig per SoC and one shared
 * init.  See include/hw/arm/xiongmai.h for why this is a family of its own.
 *
 * Boots the OpenIPC xm510_lite kernel (vendor 3.0.101, ATAGs, MACH 8888)
 * with -kernel/-initrd to a shell.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/xiongmai.h"
#include "hw/boards.h"
#include "hw/char/pl011.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "net/net.h"
#include "system/address-spaces.h"
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
    .wdt_freq           = 24000000,
    .num_regbanks       = 7,
    .regbanks           = {
        { "xm-gpio",     0x10020000, 0x10000 },  /* GPIO + pin mux */
        { "xm-i2c",      0x10000000, 0x10000 },
        { "xm-dma",      0x20020000, 0x10000 },  /* PL330, until modelled */
        { "xm-h264",     0x20070000, 0x10000 },  /* Hantro H1 */
        { "xm-rtc",      0x100A0000, 0x10000 },
        { "xm-i2s",      0x10060000, 0x10000 },
        { "xm-isp",      0x40000000, 0x100000 }, /* ISP / VI */
    },
};

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

static void xiongmai_common_init(MachineState *machine, const XmSoCConfig *c)
{
    static struct arm_boot_info binfo;
    MemoryRegion *sysmem = get_system_memory();
    qemu_irq pic[32];
    ARMCPU *cpu;
    int n;

    cpu = ARM_CPU(object_new(machine->cpu_type));
    /* The vendor kernel is entered in SVC by U-Boot; no secure world. */
    if (object_property_find(OBJECT(cpu), "has_el3")) {
        object_property_set_bool(OBJECT(cpu), "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    memory_region_add_subregion(sysmem, c->ram_base, machine->ram);

    /* PL190-layout VIC: the vendor entry macro reads IRQSTATUS at +0 and
     * nothing else, and init writes INTENCLEAR/INTSELECT/SOFTINTCLEAR at the
     * PL190 offsets (REG_INTC_* in platform.h). */
    DeviceState *vic = sysbus_create_varargs("pl190", c->vic_base,
            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ),
            NULL);
    for (n = 0; n < 32; n++) {
        pic[n] = qdev_get_gpio_in(vic, n);
    }

    if (c->sys_base) {
        DeviceState *sys = qdev_new("xm-sysctl");
        qdev_prop_set_uint32(sys, "rsten-offset", c->sys_rsten);
        qdev_prop_set_uint32(sys, "rst-offset", c->sys_rst);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sys), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sys), 0, c->sys_base);
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
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sfc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sfc), 0, c->sfc_base);
        sysbus_mmio_map(SYS_BUS_DEVICE(sfc), 1, c->sfc_window);
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
    arm_load_kernel(cpu, machine, &binfo);
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
