/*
 * SigmaStar (MStar) ARMv7 SoCs.
 *
 * Cortex-A7 parts that share one memory map whatever the family: DRAM
 * (the MIU0 window) at 0x20000000, the SPI NOR read window at
 * 0x14000000, the A7 private region with the GIC at 0x16000000, every
 * peripheral behind the RIU at 0x1F000000 and IMI SRAM at 0xA0000000.
 * What differs between dies is the chip ID, the package bond strap the
 * IPL picks its DRAM profile from, the number of cores and how much
 * DRAM is in the package.
 *
 * Sources: the vendor infinity2m SDK (U-Boot 2015.01 and kernel 4.9.84,
 * "boot_621_release" / "kernel_621_release"), the device tree built into
 * an SSR621Q NVR's kernel, and Daniel Palmer's MStar QEMU work
 * (github.com/fifteenhex/qemu, branches mstar-clean and
 * mstar-miyoo-working), whose register findings were made on SSD202D
 * silicon.
 */
#ifndef HW_ARM_SIGMASTAR_H
#define HW_ARM_SIGMASTAR_H

#include "hw/misc/sstar-riu.h"

typedef enum SsDie {
    SS_DIE_INFINITY2M = 0,
    SS_DIE_IFADO,
    SS_DIE_INFINITY3,
    SS_DIE_IFORD,
    SS_DIE_INFINITY6,
    SS_DIE_INFINITY6B0,
    SS_DIE_INFINITY6E,
    SS_DIE_INFINITY6C,
    SS_DIE_IFLIEGEN,
} SsDie;

typedef struct SsSoCConfig {
    const char *desc;
    SsDie       die;
    uint32_t    num_cpus;
    uint64_t    ram_size_default;
    /* the evidence there is, for a part with no part-level size */
    const char *ram_unknown;
    /* why a part has no bond default: the machine warns until bond= */
    const char *bond_unknown;
    uint16_t    chip_id;        /* chipver bank 0x001E reg 0x00 */
    uint16_t    bond;           /* chiptop bank 0x101E reg 0x48 */
    uint32_t    flash_jedec;    /* the board's SPI NOR */
    uint32_t    imi_size;       /* IMI SRAM; 0 = 64 KiB */
    uint32_t    ipl_offset;     /* where the ROM looks for the IPL in NOR */
    uint16_t    fsp_bank;       /* boot FSP's RIU bank; 0 = 0x0016 */
    uint16_t    qspi_bank;      /* boot QSPI's RIU bank; 0 = 0x0017 */
    bool        fsp_sw_cs;      /* FSP-QSPI with the software chip select */
    bool        smf_ack;        /* the IPL's SMF stub handshake (bank 0x1121) */
    bool        urdma;          /* UART DMA, Ifado layout; SPIs 93/94 its */
    uint8_t     miic;           /* how many MIIC I2C masters (0-3) */
    uint8_t     miic_sensor_bus;    /* the one the image sensor is on */
    const char *default_sensor; /* sensor= when not given; NULL = none */
    bool        ipc;            /* a camera die: no SATA/display/VPU/GE */
    uint16_t    gpio_bank;      /* PADTOP GPIO bank, Ifado layout; 0 none */
    const SstarRegDefault *defaults;    /* over the infinity2m ones */
    bool        no_i2m_defaults;    /* not the SSD202D capture: another die's fuses */
    uint16_t    sd_bank;        /* the SD host's RIU bank, for -drive if=sd */
    uint16_t    sd_spi;
    uint16_t    sd2_bank;       /* a second SD host, on -drive if=sd,index=1 */
    int         sd2_spi;
    uint32_t    sd_cd_addr;     /* card detect pad, low with a card in */
    uint16_t    sd_cd_bit;
    uint32_t    sd_cd2_addr;    /* the second slot's, bit 0 */
    uint16_t    phy_bank;       /* EMAC ALBANY0's bank; 0 = 0x0031 */
    uint16_t    rtcpwc_spi;     /* the RTCPWC, bank 0x0034; 0 = none */
} SsSoCConfig;

#endif /* HW_ARM_SIGMASTAR_H */
