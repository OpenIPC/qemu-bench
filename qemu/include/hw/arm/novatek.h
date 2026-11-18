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

/* The A64 dies' block map (novatek.c): NA51103's or NA51090's */
typedef struct NvtA64Map NvtA64Map;

typedef struct NvtSoCConfig {
    const char *desc;
    const char *model;          /* the DT model string */
    uint32_t    chip_id;        /* TOP_VERSION_REG [31:16] */
    uint64_t    ram_size_default;
    uint64_t    linux_mem;      /* the DT memory node; the rest is HDAL's */
    uint64_t    sdk_linux_mem;  /* the SDK's share, when RAM is smaller */
    /*
     * novatek_common_init() only: NA51055 (NT9852x) is NA51089's die with
     * two cores and a few blocks added or moved; num_cpus 0 means one.
     */
    bool        na51055;
    unsigned    num_cpus;
    const NvtA64Map *a64;       /* novatek_a64_init() only */
} NvtSoCConfig;

typedef struct NvtFlashState NvtFlashState;

/*
 * Stand in for the closed loader: from a Novatek flash image, put the FDT,
 * U-Boot and the SHMINFO block where the flash's /nvt_memory_cfg says, and
 * a branch to U-Boot at 0 (novatek.c).
 */
void nvt_flash_boot(NvtFlashState *flash, uint64_t ram_size, bool a64);

/* The mask ROM model's per-chip setting (nvt-bootrom.c) */
typedef struct NvtRomConfig {
    uint64_t sram_base;         /* where the loader runs */
    uint64_t sram_size;
    uint64_t sp;                /* the ROM's stack: BOOT_ROM_SP_LIMIT_ADDR */
    uint64_t uart;              /* UART 0, for the ROM's messages */
    uint8_t  layout;            /* the ROM's routines (nvt-bootrom.c) */
} NvtRomConfig;

/* Whether the flash starts with a loader for this SRAM; run it. */
bool nvt_flash_has_loader(NvtFlashState *flash, uint64_t sram_base,
                          uint64_t sram_size);
void nvt_rom_boot(NvtFlashState *flash, const NvtRomConfig *rc);
/* The ROM's loader from a host file, not the flash (nvt-bootrom.c) */
void nvt_rom_set_loader_file(const char *path);
/*
 * The ROM's own fallback (NA51089, nvt-bootrom.c): with no loader in
 * flash, or the straps not at flash, it looks on SD, Ethernet, USB and UART.
 */
void nvt_rom_set_fallback(bool on, uint32_t strap, bool fast);
bool nvt_rom_strap_is_flash(uint32_t strap);

#endif /* HW_ARM_NOVATEK_H */
