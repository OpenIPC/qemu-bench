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
} AkSoCConfig;

#endif /* HW_ARM_ANYKA_H */
