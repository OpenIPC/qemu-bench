/*
 * Axera AX620E family SoCs.
 *
 * Cortex-A53 parts with a GIC-400 and the ARMv8 generic timer -- ordinary
 * ARM infrastructure QEMU already models, unlike the RIU/HAL-indirected
 * families elsewhere in this tree. What is Axera-specific is the register
 * map: a GIC-400 at 0x01850000 (distributor at +0x1000, CPU interface at
 * +0x2000), a DesignWare 8250-compatible UART0 at 0x04880000 (reg-shift 2,
 * reg-io-width 4), and DRAM at 0x40000000.
 *
 * Source: the onekvm/linux-6.18-ax620e tree's
 * arch/arm64/boot/dts/axera/ax620e.dtsi (the SoC) and
 * arch/arm64/boot/dts/axera/ax630c-nanokvm-pro.dts (the board, memory map
 * and chosen/bootargs).
 */
#ifndef HW_ARM_AXERA_H
#define HW_ARM_AXERA_H

#include "exec/hwaddr.h"

typedef struct AxeraSoCConfig {
    const char *desc;
    uint64_t    ram_size_default;
    hwaddr      uart0_base;     /* snps,dw-apb-uart (or axera,ax-apb-uart) */
    int         uart0_irq;      /* the DT's raw SPI number, not the INTID */
    bool        is_ax620x;      /* AX620U: "axera,ax620x", not "axera,ax620e" */
} AxeraSoCConfig;

#endif /* HW_ARM_AXERA_H */
