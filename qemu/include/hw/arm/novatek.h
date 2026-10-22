/*
 * Novatek NT9856x (NA51089) IP-camera SoCs.
 *
 * One Cortex-A9 with its MPCore private region (SCU, GIC, global timer,
 * TWD) at 0xFFD00000 and a PL310 at 0xFFE00000, DDR in the package at 0,
 * and Novatek's own blocks from 0xF0000000: TOP (pinmux, chip ID) at
 * 0xF0010000, the clock generator at 0xF0020000, watchdog, RTC and GPIO
 * below 0xF0100000, 16550-compatible UARTs at 0xF0290000/0xF0300000/
 * 0xF0310000, the SPI NOR/NAND controller at 0xF0400000.  The kernel's
 * clocksource is the A9 global timer and TWD, not the Novatek timer.
 *
 * The same register layout, shifted to 0xC0000000, is NT96672's (NA51023,
 * MIPS) and, at 0xF0000000, NA51055's: the blocks here are not 51089-only.
 * NT9832x (NA51068) is a DVR part with Faraday IP and a different map.
 *
 * Sources: the NT9856x Linux SDK v1.01.002 (kernel 4.19.91, U-Boot
 * 2019.04, board configs cfg_562_IPCAM1_{,NOR_}EVB/nvt-*.dtsi) and the
 * OpenIPC novatek-nt9856x kernel, which is that SDK's.
 */
#ifndef HW_ARM_NOVATEK_H
#define HW_ARM_NOVATEK_H

#include "exec/hwaddr.h"

typedef struct NvtSoCConfig {
    const char *desc;
    const char *model;          /* the DT model string */
    uint32_t    chip_id;        /* TOP_VERSION_REG [31:16] */
    uint64_t    ram_size_default;
    uint64_t    linux_mem;      /* the DT memory node; the rest is HDAL's */
    uint64_t    sdk_linux_mem;  /* the SDK's share, when RAM is smaller */
} NvtSoCConfig;

#endif /* HW_ARM_NOVATEK_H */
