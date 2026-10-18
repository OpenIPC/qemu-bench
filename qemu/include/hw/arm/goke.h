/*
 * Goke GK7101 / GK7102 IP-camera SoCs.
 *
 * Goke's own first design, not HiSilicon-derived like the later GK720x /
 * GK7205 parts: one ARM1176 and peripherals on Ambarella A5s IP (VIC,
 * timers, watchdog, 16550 UARTs, Synopsys GMAC, SDHCI), DDR at 0xC0000000.
 * GK7102 is the 64 MiB in-package-DDR version of the GK7101 die.
 *
 * The vendor software never touches the hardware directly.  U-Boot carries
 * a "HAL" blob (board/goke/gk7101/gk7101_hal_data.h, linked at 0xC0012000)
 * and hands its address to the kernel at 0xC0000008; every gk_*_readl /
 * gk_*_writel in both goes through its hw_readl / hw_writel.  The drivers
 * use a fictitious map -- AHB at 0x90000000, APB at 0xA0000000, in U-Boot
 * and the kernel's mach-gk710x headers alike -- and the HAL moves each
 * block to its real place (AHB 0x60000000, APB 0x70000000), shuffles the
 * register offsets inside it and, for some registers, permutes the bits.
 * Undone, what is left is plain IP: the UART is an ordinary 16550 and the
 * VIC is PL190-shaped.  The models here are of the silicon as the HAL
 * drives it, recovered by executing the HAL for every fictitious address
 * (version 0x20160913, the SDK v2.0.0 blob; both camera dumps carry it).
 *
 * The bit permutation that matters for wiring is the VIC's: the kernel's
 * IRQ n (mach-gk710x/include/mach/irqs.h) is VIC input irq_line[n].
 */
#ifndef HW_ARM_GOKE_H
#define HW_ARM_GOKE_H

#include "exec/hwaddr.h"

#define GK_MAX_UART     3
#define GK_MAX_REGBANK  24
#define GK_VIC_LINES    32

typedef struct GkRegbank {
    const char *name;
    hwaddr      base;
    uint32_t    size;
} GkRegbank;

typedef struct GkSoCConfig {
    const char *desc;
    const char *cpu_type;

    hwaddr      ram_base;
    uint64_t    ram_size_default;
    /* ATAGs machine_arch_type: MACH_GOKE_IPC (arch/arm/tools/mach-types). */
    uint32_t    board_id;

    /* Two VICs; kernel IRQ n is input irq_line[n % 32] of VIC n / 32. */
    hwaddr      vic_base[2];
    uint8_t     irq_line[GK_VIC_LINES];
    /* Kernel IRQs whose (unmodelled) source is active low. */
    int         num_idle_high_irqs;
    int         idle_high_irqs[4];

    int         num_uarts;
    hwaddr      uart_bases[GK_MAX_UART];
    int         uart_irqs[GK_MAX_UART];     /* kernel IRQ numbers */
    uint32_t    uart_clk;

    hwaddr      timer_base;
    int         timer_irqs[3];
    uint32_t    apb_freq;

    hwaddr      wdt_base;
    int         wdt_irq;

    hwaddr      sflash_base;

    hwaddr      idc_bases[2];
    int         idc_irqs[2];

    hwaddr      sdhci_base;
    int         sdhci_irq;

    hwaddr      usb_base;       /* Mentor MUSB core, see goke.c */

    /* Synopsys GMAC (xm-dwmac) and the internal PHY (gk-ephy). */
    hwaddr      mac_base;
    int         mac_irq;
    hwaddr      ephy_base;

    /* RAM-backed stubs: blocks with no model, so the HAL's read-modify-
     * writes and the vendor modules' pokes land somewhere. */
    int         num_regbanks;
    GkRegbank   regbanks[GK_MAX_REGBANK];

    /* Where U-Boot puts the HAL and its hand-off words for the kernel
     * (CONFIG_GK_HAL_ADDR, CONFIG_U2K_*). */
    hwaddr      hal_addr;
    /* mem= for -kernel boots that do not set one: Linux gets this much
     * from PHYS_OFFSET (0xC0200000); the rest is the DSP's. */
    uint32_t    kernel_mem_mb;
    uint32_t    bsb_mb;         /* bitstream buffer carve-out, U2K word */
} GkSoCConfig;

#endif /* HW_ARM_GOKE_H */
