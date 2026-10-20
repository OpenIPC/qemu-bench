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

typedef struct SsSoCConfig {
    const char *desc;
    uint32_t    num_cpus;
    uint64_t    ram_size_default;
    uint16_t    chip_id;        /* chipver bank 0x001E reg 0x00 */
    uint16_t    bond;           /* chiptop bank 0x101E reg 0x48 */
    uint32_t    flash_jedec;    /* the board's SPI NOR */
} SsSoCConfig;

#endif /* HW_ARM_SIGMASTAR_H */
