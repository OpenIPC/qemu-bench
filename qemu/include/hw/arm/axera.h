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
    int         max_cpus;       /* 0 means 1 -- only AX620U's spin-table SMP
                                  * is modelled (see smp_bootreg_addr below) */
    hwaddr      smp_bootreg_addr; /* 0 if this SoC's secondary-boot-reg isn't
                                    * modelled (AX620Q: register not
                                    * confirmed; only CPU0 boots there) */
    bool        flash_is_nor;   /* flash-file='s real dump's own flash type
                                  * (board.c's get_boot_mode() chip_mode
                                  * strap, which BL1 itself reads to decide
                                  * spinor_init() vs emmc_init() -- real
                                  * hardware's boot ROM sets this from strap
                                  * pins before BL1 runs, so flash-file=
                                  * must preseed the same value): true for
                                  * AX620E's real SPI NOR camera dump, false
                                  * (eMMC) for AX630C's real dev-board dump.
                                  */
    const char *default_chip;   /* the part chip= defaults to: what the
                                  * eFuse/bond registers report (see
                                  * axera_chip_id_init()) */
} AxeraSoCConfig;

/*
 * Defined in hw/arm/axera.c (ARM-specific code, free to see into
 * target/arm/cpu.h); declared here so plain "common" devices like
 * hw/misc/axera-spi-nor.c -- which cannot include that header without
 * breaking the multi-target build -- can still call it.
 */
uint64_t axera_cpu_get_xreg(unsigned int n);

#endif /* HW_ARM_AXERA_H */
