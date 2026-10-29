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
#include "qemu/cutils.h"
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
#include "hw/misc/xm-regtrace.h"
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

/*
 * The second generation: XM530V200 (its U-Boot reports XM550V200WX2), XM533
 * and XM535, read from camera dumps (#154).  The XM530 map with:
 *   - the boot ROM entering flash at the window, 0x60000000, where their
 *     U-Boot is linked -- there is no 0x08000000 text alias to start from;
 *   - DDR behind a DesignWare uMCTL2 and its PHY at 0x20050000 (xm-umctl2)
 *     instead of the uPCTL at 0x100B0000;
 *   - an SP804 that latches its count (xm_sp804_latch_ops);
 *   - the SDIO (Wi-Fi) controller moved from 0x50100000, to 0x50300000 on
 *     XM530V200 and XM535, to 0x50200000 on XM533 (the kernels' platform
 *     resources: IRQ 76 / 74).
 * The kernels' static I/O map (0x10000000 and 0x20000000) is the same on
 * all three.  Only the XM530V200 kernel is built SMP; -smp 2 is allowed on
 * the others too, as nothing says their die has one core.
 */
static XmSoCConfig xm53x_v2_base(const char *desc)
{
    XmSoCConfig c = xm530_soc;

    c.desc = desc;
    c.sfc_text_alias = 0;
    c.flash_entry = 0x60000000;
    c.ddrc_base = 0;
    c.num_ddrc_done = 0;
    c.ddrc_mirror = 0;
    c.ddrc_mirror_of = 0;
    c.umctl2_base = 0x20050000;
    /* xm_i2c.ko's bus 2 (bus n at 0x10000000 + n * 0x1000): XM535D2's two
     * phy-0 sensors (Isp.json I2cInfo dev 2, 0x30 / 0x32) */
    c.i2c2_base = 0x10002000;
    c.timer_latch = true;
    /* xm_usb0.ko's "dwc2" platform device: the DWC2 OTG core at
     * 0x50300000, IRQ 76 (SPI 44), where XM530 had the SDIO controller;
     * the Wi-Fi of these boards is a USB dongle (atbm, xm713g.ko).  Only
     * XM533 keeps an SDIO slot, at 0x50200000. */
    c.num_sdhci = 1;
    c.usb_base = 0x50300000;
    c.usb_irq = 44;
    /* Isp.json probes I2C 0x30 and 0x32, one 2 MP SmartSens part per lens
     * (isp.ko insists on 1080p timing: 1125 lines).  The "6M" in Sofia's
     * config is the VPSS upscaling the pair.  SC2336, ID 0xCB3A. */
    c.sensor_type = "hisi-smartsens";
    c.sensor_addr = 0x30;
    c.sensor_addr2 = 0x32;
    c.sensor_id = 0xCB3A;
    /* hx280enc as on XM530, moved: base_port 0x21100000, irq 78. */
    c.h1_base = 0x21100000;
    c.h1_irq = 46;
    c.h1_max_width = 4080;  /* the main stream is 2304 wide */
    /* VI / ISP moved too (0x32000000, 0x34000000): not modelled yet. */
    c.vi_base = 0;
    /* XM530's ISP regbank at 0x40000000 is last; the I2S one before it
     * is xm-audio here */
    c.num_regbanks = 3;
    /* i2s.ko's AI and AO blocks, IRQ 48 / 49; XM535's and XM533's move 20
     * ms (160 samples) an interrupt */
    c.audio_base = 0x10060000;
    c.audio_irqs[0] = 16;
    c.audio_irqs[1] = 17;
    c.audio_chunk = 160;
    /* VI + ISP as isp.ko drives them (xm-isp2): VICU end / start, BPU,
     * NR3D, VPSS store 0..3 (isp.ko's and vi.ko's request_irq) */
    c.isp2 = true;
    {
        static const int irqs[XM_ISP2_IRQS] = { 55, 56, 30, 37, 25, 26, 35, 36, -1 };
        memcpy(c.isp2_irqs, irqs, sizeof(irqs));
    }
    /* Pads up to 166 (XM533's AoEn 138, WifiPdn 166); adc.ko's ADC */
    c.gpio_pads = 200;
    c.gpio_adc_off = 0xCC4;
    /* hevc.ko's HEVC engine (xm-hevc): 0x21000000 and 0x21010000, SPI 45 */
    c.hevc_base = 0x21000000;
    c.hevc_irq = 45;
    /* rgn.ko's OSD engine (xm-osd), osd_int SPI 62 */
    c.osd_base = 0x21200000;
    c.osd_irq = 62;
    /* isp.ko's timer45 (SPI 6) and timer7 (SPI 54): two more SP804s. */
    c.num_timers = 4;
    c.timer_bases[2] = 0x100D1000;
    c.timer_irqs[2] = 6;
    c.timer_bases[3] = 0x100D2000;
    c.timer_irqs[3] = 54;
    return c;
}

static const XmSoCConfig *xm530v200_config(void)
{
    static XmSoCConfig c;

    if (!c.desc) {
        c = xm53x_v2_base("Xiongmai XM530V200 / XM550V200 "
                          "(2x Cortex-A5 stand-in A9, GIC, uMCTL2)");
        /* 128 MiB: the loader script's MEM_ALL, Linux mem=63M and the MMZ
         * from 0x83F00000 to the top */
        c.ram_size_default = 128 * MiB;
        /* its isp.ko: XM535's VI blocks, the VICU interrupt words
         * moved; vicu_bayer_irq0/1 SPI 55 / 56, bpu_hvgen_irq SPI 31;
         * and vpss.ko's memory-to-memory VPSS, vpss_int SPI 40 */
        c.isp2_layout = "xm530v200";
        /* its HEVC engine: an earlier revision of XM533's */
        c.hevc_layout = "xm530v200";
        /* rgn.ko's OSD engine at 0x21300000 (osd_int SPI 62 as XM535) */
        c.osd_base = 0x21300000;
        /* The boards' reset keys (Sofia's Device json Key: 80 on X2-WQ2,
         * 74 on X6C-WEQ), active low: low makes Sofia restore defaults and
         * reboot (type 10). */
        c.num_gpio_high = 2;
        c.gpio_high[0] = 80;
        c.gpio_high[1] = 74;
        /* its i2s.ko moves 256 samples an interrupt */
        c.audio_chunk = 256;
        /* nn.ko's NN accelerator (xm-nn), where XM535 has the OSD */
        c.nn_base = 0x21200000;
        c.nn_irq = 43;
        /* no hx280enc here: 0x21100000 / SPI 46 is jpeg.ko's JPEG engine */
        c.h1_base = 0;
        c.jpeg_base = 0x21100000;
        c.jpeg_irq = 46;
        {
            static const int irqs[XM_ISP2_IRQS] = { 55, 56, 31, -1, -1, -1, -1, -1, 40 };
            memcpy(c.isp2_irqs, irqs, sizeof(irqs));
        }
    }
    return &c;
}

static const XmSoCConfig *xm535_config(void)
{
    static XmSoCConfig c;

    if (!c.desc) {
        c = xm53x_v2_base("Xiongmai XM535 "
                          "(Cortex-A5 stand-in A9, GIC, uMCTL2)");
        /* The X6E-WEQ board's reset key (Sofia's Device json: Key=95),
         * active low: low at boot makes Sofia reboot to factory reset. */
        c.num_gpio_high = 1;
        c.gpio_high[0] = 95;
        /* nn.ko's NN accelerator (xm-nn) */
        c.nn_base = 0x21500000;
        c.nn_irq = 43;
        /* rgn.ko writes 2-bit colours 1 and 2 the other way round */
        c.osd_swap12 = true;
    }
    return &c;
}

static const XmSoCConfig *xm533_config(void)
{
    static XmSoCConfig c;

    if (!c.desc) {
        c = xm53x_v2_base("Xiongmai XM533 (XM530V300, \"qiandaohu\"; "
                          "Cortex-A5 stand-in A9, GIC, uMCTL2)");
        c.num_sdhci = 2;
        c.sdhci_bases[1] = 0x50200000;
        c.sdhci_irqs[1] = 42;
        /* ...and it is the card slot (sdio1_sd.ko), Wi-Fi the first */
        c.sd_slot = 1;
        c.usb_base = 0;
        /* XM530V300's own layout and lines: vicu_bayer_irq0 SPI 25,
         * vi_yuv_store SPI 27 / 28; no BPU, NR3D or second VICU line */
        c.isp2_layout = "xm533";
        c.hevc_layout = "xm533";
        c.isp2_store_lines_by_event = true;
        /* nn.ko's defaults: cnn_base 0x21500000, int_irq 75 (SPI 43), as
         * XM535's */
        c.nn_base = 0x21500000;
        c.nn_irq = 43;
        {
            static const int irqs[XM_ISP2_IRQS] = { 25, -1, -1, -1, 27, 28, -1, -1, -1 };
            memcpy(c.isp2_irqs, irqs, sizeof(irqs));
        }
    }
    return &c;
}

typedef struct XmMachineState {
    MachineState parent_obj;
    char *flash_file;
    uint32_t sensor_id;         /* 0: the SoC config's */
    uint32_t sensor_ext;        /* 0x801e, 0x1ff: the model's default */
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

static char *xm_machine_get_sensor_id(Object *obj, Error **errp)
{
    return g_strdup_printf("%04x", XM_MACHINE(obj)->sensor_id);
}

static void xm_machine_set_sensor_id(Object *obj, const char *value,
                                     Error **errp)
{
    uint64_t v, ext = 0x1ff;
    const char *end;

    /* "cb3a", or "9b3a/00": the chip ID and the value of 0x801e, which
     * tells SmartSens variants apart (SC2336P / SC2337P) */
    if (qemu_strtou64(value, &end, 16, &v) || v > 0xffff ||
        (*end == '/' && (qemu_strtou64(end + 1, NULL, 16, &ext) || ext > 0xff)) ||
        (*end && *end != '/')) {
        error_setg(errp, "sensor-id: a 16-bit hex chip ID, e.g. cb3a or 9b3a/00");
        return;
    }
    XM_MACHINE(obj)->sensor_id = v;
    XM_MACHINE(obj)->sensor_ext = ext;
}


char xm_regtrace_mode(void)
{
    if (!current_cpu) {
        return '-';
    }
    return (ARM_CPU(current_cpu)->env.uncached_cpsr & 0x1f) == 0x10 ? 'U' : 'K';
}

/* the board's sensor (and its identical twin, if any) on an I2C bus */
static void xm_attach_sensors(const XmSoCConfig *c, XmMachineState *ms,
                              I2CBus *bus, uint8_t busno)
{
    uint16_t id = ms->sensor_id ? ms->sensor_id : c->sensor_id;
    const uint8_t addrs[2] = { c->sensor_addr, c->sensor_addr2 };

    if (!c->sensor_type) {
        return;
    }
    for (int i = 0; i < 2 && addrs[i]; i++) {
        I2CSlave *sns = i2c_slave_new(c->sensor_type, addrs[i]);

        if (id) {               /* hisi-smartsens: which SC part */
            qdev_prop_set_uint8(DEVICE(sns), "id_high", id >> 8);
            qdev_prop_set_uint8(DEVICE(sns), "id_low", id & 0xff);
            if (ms->sensor_id && ms->sensor_ext <= 0xff) {
                qdev_prop_set_uint8(DEVICE(sns), "reg_801e", ms->sensor_ext);
            }
        }
        if (!strcmp(c->sensor_type, "hisi-smartsens")) {
            qdev_prop_set_uint8(DEVICE(sns), "bus", busno);
        }
        i2c_slave_realize_and_unref(sns, bus, &error_fatal);
    }
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
 * A flash boot runs with EL3 (the 2023+ U-Boots write NSACR), and U-Boot
 * leaves CPU0 in secure SVC for the kernel.  arm_set_cpu_on() at EL1 would
 * start CPU1 non-secure instead: its GIC bank and CPU0's then disagree on
 * interrupt groups, the IPIs between them are lost, and the kernel stalls
 * once both CPUs are up.  CPU1 starts in the security state CPU0 is in.
 */
static ARMCPU *xm_cpu0;

static int xm_cpu1_target_el(void)
{
    if (xm_cpu0 && arm_feature(&xm_cpu0->env, ARM_FEATURE_EL3) &&
        !(xm_cpu0->env.cp15.scr_el3 & SCR_NS)) {
        return 3;   /* secure SVC: what a reset leaves an AArch32 core in */
    }
    return 1;
}

/*
 * REG_CPU1_SOFT_RST: CPU1 leaves reset at PC 0, in the state CPU0 runs in
 * (SVC; secure or not as CPU0 is, see above).
 */
static void xm_cpu1_on(CPUState *cs, run_on_cpu_data data)
{
    const int ret = arm_set_cpu_on(arm_cpu_mp_affinity(ARM_CPU(cs)),
                                   0, 0, xm_cpu1_target_el(), false);

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
    ret = arm_set_cpu_on(mpidr, 0, 0, xm_cpu1_target_el(), false);
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

/*
 * The SP804 on the later XM53x parts latches its count.  The XM530V200
 * kernel's clocksource writes TimerControl with bit 8 set (0x1C2), waits for
 * bit 8 to read back clear, then reads the snapshot at +0x1C and inverts
 * it.  A stock SP804 has no such bit or register: bit 8 stays set, the
 * kernel gives up after six polls, and +0x1C reads 0 -- the clock never
 * moves and the kernel prints nothing.  This window sits over each timer's
 * control and snapshot words (timer 1's at +0x28 / +0x3C likewise) and
 * forwards the rest to the stock model.
 */
#define XM_SP804_LATCH  (1u << 8)

typedef struct XmSp804Latch {
    MemoryRegion iomem;
    MemoryRegion *timer;
    uint32_t snap[2];
} XmSp804Latch;

static uint64_t xm_sp804_latch_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    XmSp804Latch *l = opaque;
    int n = offset >= 0x20;
    hwaddr reg = offset & 0x1f;
    uint64_t val = 0;

    if (reg == 0x1c) {
        return l->snap[n];
    }
    memory_region_dispatch_read(l->timer, offset, &val, MO_32,
                                MEMTXATTRS_UNSPECIFIED);
    return val;
}

static void xm_sp804_latch_write(void *opaque, hwaddr offset, uint64_t val,
                                 unsigned size)
{
    XmSp804Latch *l = opaque;
    int n = offset >= 0x20;
    hwaddr reg = offset & 0x1f;

    if (reg == 0x1c) {
        return;
    }
    if (reg == 0x08 && (val & XM_SP804_LATCH)) {
        uint64_t count = 0, ctrl = 0;

        memory_region_dispatch_read(l->timer, offset - 0x08 + 0x04, &count,
                                    MO_32, MEMTXATTRS_UNSPECIFIED);
        l->snap[n] = count;
        val &= ~XM_SP804_LATCH;
        /* The stock model reloads on every control write; a latch request
         * that leaves the control bits as they are must not restart it. */
        memory_region_dispatch_read(l->timer, offset, &ctrl, MO_32,
                                    MEMTXATTRS_UNSPECIFIED);
        if (ctrl == val) {
            return;
        }
    }
    memory_region_dispatch_write(l->timer, offset, val, MO_32,
                                 MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps xm_sp804_latch_ops = {
    .read = xm_sp804_latch_read,
    .write = xm_sp804_latch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

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
        /*
         * -kernel enters the vendor kernel in SVC with no secure world.  A
         * flash boot keeps EL3, as the A5 has it: the 2023+ U-Boots write
         * NSACR in start.S to open the FPU, which is UNDEF without it, and
         * die on the exception before printing a byte.
         */
        if (object_property_find(o, "has_el3")) {
            object_property_set_bool(o, "has_el3",
                                     XM_MACHINE(machine)->flash_file != NULL,
                                     &error_fatal);
        }
        /*
         * The stand-in A9 lacks the A5's VFPv4: MVFR1.SIMDFMAC is 0 on the
         * A9 (0x01111111) and 1 on the A5 (0x11111111).  The vendor Sofia
         * on the second-generation parts uses VFMA, an Illegal instruction
         * on a plain A9.  Only the -cpu default (an A9) is touched.
         */
        if (!strcmp(machine->cpu_type, ARM_CPU_TYPE_NAME("cortex-a9"))) {
            ARM_CPU(o)->isar.mvfr1 = 0x11111111;
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
    xm_cpu0 = cpu;

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
    if (c->umctl2_base) {
        sysbus_create_simple("xm-umctl2", c->umctl2_base, NULL);
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
        if (c->timer_latch) {
            XmSp804Latch *l = g_new0(XmSp804Latch, 1);

            l->timer = sysbus_mmio_get_region(SYS_BUS_DEVICE(t), 0);
            memory_region_init_io(&l->iomem, NULL, &xm_sp804_latch_ops,
                                  l, "xm-sp804-latch", 0x40);
            memory_region_add_subregion_overlap(sysmem, c->timer_bases[n],
                                                &l->iomem, 1);
        }
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
        if (c->flash_entry && !machine->kernel_filename) {
            xm_flash_boot_pc = c->flash_entry;
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

        /* the first SD drive goes in the board's card slot */
        DriveInfo *di = drive_get(IF_SD, 0, n == c->sd_slot ? 0 :
                                  n == 0 ? c->sd_slot : n);
        if (di) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(sd, "sd-bus"),
                                   &error_fatal);
        }
    }

    if (c->usb_base) {
        DeviceState *usb = qdev_new("dwc2-usb");

        object_property_add_const_link(OBJECT(usb), "dma-mr",
                                       OBJECT(sysmem));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(usb), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(usb), 0, c->usb_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(usb), 0, pic[c->usb_irq]);
    }

    if (c->i2c_base) {
        DeviceState *i2c = sysbus_create_simple("xm-i2c", c->i2c_base, NULL);

        xm_attach_sensors(c, XM_MACHINE(machine),
                          I2C_BUS(qdev_get_child_bus(i2c, "i2c")), 0);
    }
    if (c->i2c2_base) {
        /* over the bus-1.. placeholder regbank */
        DeviceState *i2c = qdev_new("xm-i2c");

        sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(i2c), 0, c->i2c2_base, 1);
        xm_attach_sensors(c, XM_MACHINE(machine),
                          I2C_BUS(qdev_get_child_bus(i2c, "i2c")),
                          (c->i2c2_base >> 12) & 0xf);
    }

    if (c->audio_base) {
        DeviceState *au = qdev_new("xm-audio");

        if (c->audio_chunk) {
            qdev_prop_set_uint32(au, "chunk", c->audio_chunk);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(au), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(au), 0, c->audio_base);
        for (n = 0; n < 2; n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(au), n, pic[c->audio_irqs[n]]);
        }
    }
    if (c->nn_base) {
        sysbus_create_simple("xm-nn", c->nn_base, pic[c->nn_irq]);
    }
    if (c->jpeg_base) {
        sysbus_create_simple("xm-jpeg", c->jpeg_base, pic[c->jpeg_irq]);
    }

    if (c->hevc_base) {
        DeviceState *hevc = qdev_new("xm-hevc");
        if (c->hevc_layout) {
            qdev_prop_set_string(hevc, "layout", c->hevc_layout);
        }

        sysbus_realize_and_unref(SYS_BUS_DEVICE(hevc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(hevc), 0, c->hevc_base);
        sysbus_mmio_map(SYS_BUS_DEVICE(hevc), 1, c->hevc_base + 0x10000);
        sysbus_connect_irq(SYS_BUS_DEVICE(hevc), 0, pic[c->hevc_irq]);
    }
    if (c->osd_base) {
        DeviceState *osd = qdev_new("xm-osd");

        qdev_prop_set_bit(osd, "swap12", c->osd_swap12);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(osd), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(osd), 0, c->osd_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(osd), 0, pic[c->osd_irq]);
    }
    if (c->isp2) {
        DeviceState *isp = qdev_new("xm-isp2");
        const int *isp2_irqs = c->isp2_irqs;

        if (c->isp2_layout) {
            qdev_prop_set_string(isp, "layout", c->isp2_layout);
        }
        if (c->isp2_store_lines_by_event) {
            qdev_prop_set_uint32(isp, "store-lines", 0);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(isp), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(isp), 0, 0x30000000);
        sysbus_mmio_map(SYS_BUS_DEVICE(isp), 1, 0x32000000);
        sysbus_mmio_map(SYS_BUS_DEVICE(isp), 2, 0x34000000);
        for (n = 0; n < XM_ISP2_IRQS; n++) {
            if (isp2_irqs[n] >= 0) {
                sysbus_connect_irq(SYS_BUS_DEVICE(isp), n, pic[isp2_irqs[n]]);
            }
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
        DeviceState *h1 = qdev_new("xm-h1");

        if (c->h1_max_width) {
            qdev_prop_set_uint32(h1, "max-width", c->h1_max_width);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(h1), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(h1), 0, c->h1_base);
        sysbus_connect_irq(SYS_BUS_DEVICE(h1), 0, pic[c->h1_irq]);
    }

    if (c->gpio_base) {
        /* A named child, so qtest and a board can reach its input lines at
         * /machine/xm-gpio, whichever of the two layouts the part has. */
        DeviceState *gpio = qdev_new(c->gpio_type ? c->gpio_type : "xm-gpio");

        object_property_add_child(OBJECT(machine), "xm-gpio", OBJECT(gpio));
        if (c->gpio_pads) {
            qdev_prop_set_uint32(gpio, "pads", c->gpio_pads);
        }
        if (c->gpio_adc_off) {
            qdev_prop_set_uint32(gpio, "adc-offset", c->gpio_adc_off);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 0, c->gpio_base);
        /* Pulled-up inputs: an idle active-low button reads high. */
        for (n = 0; n < c->num_gpio_high; n++) {
            qemu_set_irq(qdev_get_gpio_in(gpio, c->gpio_high[n]), 1);
        }
    }

    if (c->wdt_base) {
        Clock *wdt_clk = clock_new(OBJECT(machine), "wdt-clk");
        clock_set_hz(wdt_clk, c->wdt_freq);
        /* The SP805 identity, not the CMSDK one: Linux's sp805_wdt binds
         * on the AMBA ID, and that driver is what gives /dev/watchdog. */
        DeviceState *wdt = qdev_new("arm-sp805");
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
        object_class_property_add_str(OBJECT_CLASS(mc), "sensor-id", \
            xm_machine_get_sensor_id, xm_machine_set_sensor_id);     \
        object_class_property_set_description(OBJECT_CLASS(mc),      \
            "sensor-id", "the SmartSens sensor's chip ID, hex (later " \
            "XM53x boards; e.g. da4d for SC1346)");                  \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, MACHINE, XmMachineState,        \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_XM_MACHINE("xm510", xm510, xm510_soc)
DEFINE_XM_MACHINE("xm530", xm530, xm530_soc)
DEFINE_XM_MACHINE("xm550", xm550, (*xm550_config()))
DEFINE_XM_MACHINE("xm530v200", xm530v200, (*xm530v200_config()))
DEFINE_XM_MACHINE("xm533", xm533, (*xm533_config()))
DEFINE_XM_MACHINE("xm535", xm535, (*xm535_config()))
