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
#define XM_MAX_TIMER    2
#define XM_MAX_REGBANK  16
#define XM_MAX_SDHCI    2

typedef struct XmRegbank {
    const char *name;
    hwaddr      base;
    uint32_t    size;
} XmRegbank;

typedef struct XmSoCConfig {
    const char *desc;
    const char *cpu_type;

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

    /* On-chip SRAM: U-Boot's first stack, before DDR is up. */
    hwaddr      sram_base;
    uint32_t    sram_size;

    /* DDR controller: only U-Boot's training touches it (xm-ddrc). */
    hwaddr      ddrc_base;

    /* Interrupt controller: a PL190-layout VIC (XM510). */
    hwaddr      vic_base;

    int         num_uarts;
    hwaddr      uart_bases[XM_MAX_UART];
    int         uart_irqs[XM_MAX_UART];

    /* SP804 dual timers; the kernel takes its clock rate from a constant
     * (CFG_TIMER_CLK), not from a PLL register. */
    int         num_timers;
    hwaddr      timer_bases[XM_MAX_TIMER];
    int         timer_irqs[XM_MAX_TIMER];
    uint32_t    timer_freq;

    /* xmsfc SPI-NOR controller and the window its flash reads through. */
    hwaddr      sfc_base;
    hwaddr      sfc_window;

    /* Synopsys DWMAC (10/100 core on XM510). */
    hwaddr      mac_base;
    int         mac_irq;

    /* Arasan SD/SDIO hosts: standard SDHCI registers (drivers/mmc/host/
     * arasan.h: PRESENT 0x24, CLOCK 0x2C, CAPS 0x40, VERSION 0xFE). */
    int         num_sdhci;
    hwaddr      sdhci_bases[XM_MAX_SDHCI];
    int         sdhci_irqs[XM_MAX_SDHCI];

    /* SP805 watchdog (xm_wdt.ko unlocks it with 0x1ACCE551 at +0xC00). */
    hwaddr      wdt_base;
    uint32_t    wdt_freq;

    /* RAM-backed stubs: blocks with no model yet, so their drivers and the
     * vendor modules can map and poke them without a data abort. */
    int         num_regbanks;
    XmRegbank   regbanks[XM_MAX_REGBANK];
} XmSoCConfig;

#endif /* HW_ARM_XIONGMAI_H */
