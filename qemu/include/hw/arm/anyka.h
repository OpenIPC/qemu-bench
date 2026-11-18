/*
 * Anyka AK39xx IP-camera SoCs.
 *
 * One ARM926EJ-S with DDR2 in the package, and a memory map that has not
 * moved from the AK3918EV200 (2015) through the EV300 (AnyCloud39EV300)
 * to the EV330 (AnyCloud V500): the system control block at 0x08000000
 * (chip ID, clocks, the interrupt controller, timers, watchdog, RTC link),
 * peripherals from 0x20000000, the L2 buffer SRAM at 0x48000000 and DDR
 * at 0x80000000.  What differs between generations is the chip ID, the
 * number of timers, the DDR size and the boot header the mask ROM reads.
 *
 * The AK3918EV300L and AK3918AV100 ("ASPEN", the AnyCloud39AV130 SDK's
 * unified U-Boot 2019.10 / kernel 4.4.302 tree) keep that map but move SPI0
 * to 0x21100000 and change the clock block; see anyka.c.
 *
 * Sources: the AnyCloud39EV300 SDK (vendor U-Boot 2013.10, kernel 3.4.35
 * with board files), the AK3918EV200 4.4 kernel port, which shares the
 * EV330's device tree and drivers, and flash dumps of EV200, EV300, EV301
 * and EV330 cameras.
 */
#ifndef HW_ARM_ANYKA_H
#define HW_ARM_ANYKA_H

#include "exec/hwaddr.h"

typedef struct AkSoCConfig {
    const char *desc;
    uint32_t    chip_id;        /* SYSCTRL +0x00 */
    uint64_t    ram_size_default;
    uint32_t    num_timers;     /* 5, or 10 from the EV330 on */
    bool        has_efuse;      /* the EV330's eFuse reader at SYSCTRL +0x8C */
    uint32_t    i2c_layout;     /* 1: EV200 / EV300 controller, 2: EV330 */
    int         flash_cs_gpio;  /* GPIO the kernel drives flash CS on, or -1 */
    /*
     * EV300L / AV100 ("ASPEN"): SPI0 moved to 0x21100000, the clock block
     * has a new PLL layout with self-clearing "apply" bits, and I2C 2 sits
     * at 0x201C0000 (0x201A0000 is UART 2 there).
     */
    bool        aspen;
    /* AV100: the second MIPI / CSI input, the NPU (anycloud_ak3918av100) */
    bool        av100;
    /*
     * AV130: the AV100's map, but its own interrupt controller and timers
     * at 0x21100000 (ak-av130-intc), a FIFO UART, the SFC v2 at
     * 0x20120000, the GMAC at 0x20240000, I2C on the controller's TWI bank
     */
    bool        av130;
    uint32_t    chip_id_av;     /* the AV130's chip ID, SYSCTRL +0x3FC */
    /*
     * EV200, AK3918EV300: the ISP2 at 0x20000000 (ak-isp) writes frames,
     * and a sensor from the image-sensor catalogue answers on I2C 0;
     * sensor= overrides it, "none" leaves the bus empty.
     */
    bool        isp2;
    bool        isp2_ev300;     /* the EV300 core's buffer enables and table */
    uint32_t    isp_fps;        /* 0: ak-isp's default */
    const char *default_sensor;
    /*
     * The GPIO wired to the sensor's active-low reset, or -1; the board's,
     * not the SoC's, so sensor-reset= overrides it.
     */
    int         sensor_reset_gpio;
    /* EV200: the Hantro 8290 video encoder at 0x20020000 (ak-h8290) */
    bool        venc_h8290;
} AkSoCConfig;

#endif /* HW_ARM_ANYKA_H */
