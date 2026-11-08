/*
 * Xiongmai XM510 / XM530 / XM550 IP-camera SoCs.
 *
 * Not HiSilicon parts, although their BSP kernels were started from
 * HiSilicon templates (the 3.0/3.10 mach-xm5x0 headers still carry
 * __HI_CHIP_REGS_H__ guards).  The memory map is Xiongmai's own, and so are
 * the flash controller (xmsfc), the per-pin GPIO and the I2C block; the rest
 * is licensed IP -- ARM PrimeCell (PL011, SP804, SP805, PL190, PL330, GIC,
 * PL310), Synopsys DWMAC, Arasan SDHCI, Hantro H1 -- so this family has its
 * own config struct instead of borrowing HisiSoCConfig's HiSilicon slots.
 *
 * Every address below is from the vendor kernels OpenIPC builds:
 * arch/arm/mach-xm510/include/mach/{platform,irqs,io,time}.h (3.0.101) and
 * arch/arm/mach-xm530/include/mach/ (3.10.103).
 */
#ifndef HW_ARM_XIONGMAI_H
#define HW_ARM_XIONGMAI_H

#include "exec/hwaddr.h"

#define XM_MAX_UART     3
#define XM_MAX_TIMER    4
#define XM_MAX_REGBANK  16
#define XM_MAX_SDHCI    2
#define XM_MAX_SYS_INIT 4
#define XM_MAX_IRQ      160     /* 32 private + up to 128 SPIs (XM8520D) */

typedef struct XmRegbank {
    const char *name;
    hwaddr      base;
    uint32_t    size;
} XmRegbank;

typedef struct XmSoCConfig {
    const char *desc;
    const char *cpu_type;
    int         max_cpus;       /* XM530/XM550: dual Cortex-A5 MPCore */

    hwaddr      ram_base;
    uint64_t    ram_size_default;
    /* ATAGs machine_arch_type: both vendor kernels register MACH_XM510 /
     * MACH_XM530 as 8888 (arch/arm/tools/mach-types). */
    uint32_t    board_id;

    /* SYS_CTRL and its soft-reset pair (REG_SYS_SOFT_RSTEN/_RST). */
    hwaddr      sys_base;
    uint32_t    sys_rsten;
    uint32_t    sys_rst;
    uint32_t    sys_pll_lock_mask;  /* xm-sysctl pll-lock-mask */
    uint32_t    sys_pll_lock31_mask; /* xm-sysctl pll-lock31-mask */
    uint32_t    sys_remap;          /* REG_SC_REMAP, 0 = none */
    uint32_t    sys_cpu1_rst;       /* REG_CPU1_SOFT_RST, 0 = none */
    int         num_sys_init;
    uint32_t    sys_init_offsets[XM_MAX_SYS_INIT];
    uint32_t    sys_init_values[XM_MAX_SYS_INIT];

    /* On-chip SRAM: U-Boot's first stack, before DDR is up. */
    hwaddr      sram_base;
    uint32_t    sram_size;

    /* DDR controller: only U-Boot's training touches it (xm-ddrc). */
    hwaddr      ddrc_base;
    int         num_ddrc_done;
    uint32_t    ddrc_done[4];   /* xm-ddrc done-offsets */
    hwaddr      ddrc_mirror;    /* xm-ddrc: reads of this offset ... */
    hwaddr      ddrc_mirror_of; /* ... return this one; 0 = none */
    /* Second generation: a uMCTL2 + PHY (xm-umctl2) instead; 0 = none. */
    hwaddr      umctl2_base;

    /* Interrupt controller: a PL190-layout VIC (XM510), or the A5 MPCore
     * private region (A9 layout: SCU +0, GIC CPU +0x100, TWD +0x600, GIC
     * dist +0x1000) with the device lines as GIC SPIs (XM530). */
    hwaddr      vic_base;
    hwaddr      mpcore_base;
    int         num_spi;
    hwaddr      l2x0_base;      /* PL310, 0 = none */

    int         num_uarts;
    hwaddr      uart_bases[XM_MAX_UART];
    int         uart_irqs[XM_MAX_UART];

    /* SP804 dual timers; the kernel takes its clock rate from a constant
     * (CFG_TIMER_CLK), not from a PLL register. */
    int         num_timers;
    hwaddr      timer_bases[XM_MAX_TIMER];
    int         timer_irqs[XM_MAX_TIMER];
    uint32_t    timer_freq;
    bool        timer_latch;    /* bit 8 snapshots the count to +0x1C */

    /* xmsfc SPI-NOR controller and the window its flash reads through.
     * When the window is not at 0, address 0 aliases it until the guest
     * remaps RAM there (SYS sys_remap), which is how the CPU boots. */
    hwaddr      sfc_base;
    hwaddr      sfc_window;
    int         sfc_version;
    /* A second alias of the flash window, where U-Boot is linked
     * (CONFIG_SYS_TEXT_BASE): it reads its own tables there before it
     * relocates.  0 = none. */
    hwaddr      sfc_text_alias;
    /* Where a flash boot starts CPU0 when U-Boot is linked at the window
     * itself (the second-generation parts); 0 = the text alias. */
    hwaddr      flash_entry;

    /* Synopsys DWMAC (10/100 core on XM510). */
    hwaddr      mac_base;
    int         mac_irq;

    /* Arasan SD/SDIO hosts: standard SDHCI registers (drivers/mmc/host/
     * arasan.h: PRESENT 0x24, CLOCK 0x2C, CAPS 0x40, VERSION 0xFE). */
    int         num_sdhci;
    hwaddr      sdhci_bases[XM_MAX_SDHCI];
    int         sdhci_irqs[XM_MAX_SDHCI];
    int         sd_slot;        /* the SDHCI with the card slot: -drive if=sd,index=0 */

    /* XM I2C controller (xm-i2c) and the sensor on it: a qdev type of an
     * I2C slave (the hisi-* sensor models) at a 7-bit address. */
    hwaddr      i2c_base;
    hwaddr      i2c2_base;      /* a second xm-i2c with the sensors; 0 = none */
    hwaddr      i2c1_base;      /* xm_i2c1: a DVR's AHD receivers; 0 = none */
    int         ahd_bus;        /* xm-i2c bus (1, 2) with NVP6158C AHD receivers */
    int         ahd_chips;      /* how many (4 cameras each) */
    const char *ahd_type;       /* "tp2830" (at 0x44..); NULL: NVP6158C at 0x30.. */
    const char *sensor_type;
    uint8_t     sensor_addr;
    uint8_t     sensor_addr2;   /* a second sensor of the same part; 0 = none */
    uint16_t    sensor_id;      /* hisi-smartsens: chip ID (0x3107/0x3108) */

    /* Video input (xm-vi) in vi_layout's register layout: vi_irqs lines
     * from vi_irq (XM530: one per channel; XM510: one for all) and the
     * ISP's isp_irqs frame interrupts from isp_irq. */
    hwaddr      vi_base;
    bool        isp2;           /* second generation: xm-isp2 instead */
    const char *isp2_layout;    /* xm-isp2 layout; NULL = xm535 */
    bool        isp2_store_lines_by_event;  /* store IRQs per event */
#define XM_ISP2_IRQS 9
    int         isp2_irqs[XM_ISP2_IRQS];    /* SPI per output, -1 = none */
    hwaddr      osd_base;       /* xm-osd; 0 = none */
    hwaddr      hevc_base;      /* xm-hevc; 0 = none */
    int         hevc_irq;
    const char *hevc_layout;    /* xm-hevc "layout"; NULL = its default */
    hwaddr      nn_base;        /* xm-nn; 0 = none */
    hwaddr      usb_base;       /* DWC2 OTG core in host mode; 0 = none */
    int         usb_irq;
    hwaddr      codec_i2c_base; /* an xm-i2c bus with an audio codec on it */
    uint8_t     codec_addr;
    const char *codec_regs;     /* xm-sensor register preset */
    int         eeprom_scl;     /* at24c.ko: AT24C16 bit-banged on pads; */
    int         eeprom_sda;     /* 0 = none */
    hwaddr      usb2_base;      /* a second DWC2 (XM8536E); 0 = none */
    int         usb2_irq;
    int         nn_irq;
    hwaddr      jpeg_base;      /* xm-jpeg (jpeg.ko's engine); 0 = none */
    const char *jpeg_layout;    /* its register layout; NULL: XM530V200's */
    int         jpeg_irq;
    hwaddr      audio_base;     /* xm-audio (I2S, codec); 0 = none */
    int         audio_irqs[5];  /* AI, AO; the DVRs' blocks in order */
    const char *audio_layout;   /* xm-audio "layout"; NULL: AI, AO */
    uint32_t    audio_chunk;    /* xm-audio "chunk"; 0 = its default */
    int         osd_irq;
    int         num_gpio_high;  /* pads whose input reads high (pull-ups) */
    uint32_t    gpio_pads;      /* 0 = the model's 100 */
    uint32_t    gpio_adc_off;   /* xm-gpio adc-offset; 0 = none */
    int         gpio_high[4];
    const char *vi_layout;
    int         vi_irq, vi_irqs;
    int         isp_irq, isp_irqs;

    /* Hantro H1 encoder (xm-h1), XM530: hx280enc base_port / irq. */
    /* Pad controller, mux and GPIO: xm-gpio (XM530, a register per pad)
     * unless gpio_type names another, as XM510's banked xm510-gpio. */
    hwaddr      gpio_base;
    const char *gpio_type;

    hwaddr      h1_base;
    int         h1_irq;
    uint32_t    h1_max_width;   /* swreg63 max width; 0 = the model's 1920 */

    /* SP805 watchdog (xm_wdt.ko unlocks it with 0x1ACCE551 at +0xC00). */
    hwaddr      wdt_base;
    uint32_t    wdt_freq;

    /* RAM-backed stubs: blocks with no model yet, so their drivers and the
     * vendor modules can map and poke them without a data abort. */
    int         num_regbanks;
    XmRegbank   regbanks[XM_MAX_REGBANK];
    hwaddr rtc2_base;           /* xm-rtc2 */
    hwaddr ahci_base;           /* sysbus-ahci */
    int ahci_irq;
    hwaddr dvr_vi_base;         /* xm-dvr-vi (xm_vi.ko of the DVRs) */
    int dvr_vi_irqs[2];
    hwaddr dvr_osd_base;        /* xm-dvr-osd (xm_rgn.ko's OSD engine) */
    int dvr_osd_irq;
    hwaddr dvr_vpss_base;       /* xm-dvr-vpss (xm_vpss.ko) */
    int dvr_vpss_irq;
    uint32_t dvr_layout;        /* 1: XM8520D's older VI / VPSS */
    uint32_t vo2_layout;        /* xm-vo2 "layout": 1 = XM8520D */
    hwaddr h2_base;             /* xm-h2, a Hantro H2 HEVC encoder */
    int h2_irq;
    hwaddr g2_base;             /* a Hantro G2 decoder (xm-vc8000d, id 0x8001) */
    int g2_irq;
    hwaddr ahci2_base;          /* a second sysbus-ahci (-drive if=ide,bus=1) */
    int ahci2_irq;
    hwaddr vc8000d_base;        /* xm-vc8000d */
    hwaddr pl330_base;          /* QEMU's pl330, xm_dma.ko */
    int pl330_irq;
    hwaddr vo2_base;            /* xm-vo2 */
    int vo2_irqs[4];            /* display 0, display 1, write-back, HDMI */
    uint32_t sys_read_or_offset;
    uint32_t sys_read_or;
    uint32_t dvr_vi_vports;     /* layout 1: the AHD port on VI lane A
                                 * [3:0] and B [7:4], 0xFF none */
} XmSoCConfig;

/*
 * An AHD receiver as the DVR VI sees it: which camera (0..3, -1 none) its
 * video port vport sends in time slot slot of its multiplex, as the
 * driver programmed it.
 */
#define TYPE_XM_AHD "xm-ahd"
typedef struct XmAhdClass {
    InterfaceClass parent;
    int (*camera)(Object *obj, int vport, int slot);
} XmAhdClass;
DECLARE_CLASS_CHECKERS(XmAhdClass, XM_AHD, TYPE_XM_AHD)

#endif /* HW_ARM_XIONGMAI_H */
