/*
 * ARM PrimeCell SP805 watchdog, as the Xiongmai XM510/XM530 carry it at
 * 0x20060000.
 *
 * QEMU's cmsdk-apb-watchdog is the SP805 programmer's model already -- the
 * same LOAD/VALUE/CONTROL/INTCLR registers, the 0x1ACCE551 lock and the
 * two-stage interrupt-then-reset countdown. What it does not have is the
 * SP805's identity: its peripheral ID reads 0x001BB824 (the CMSDK APB
 * watchdog), so Linux's sp805_wdt, which binds on the AMBA ID 0x00141805,
 * never probes it. This subtype only swaps the ID registers for the ones a
 * lab XM550 and XM510 read back (PID0..3 05 18 14 00, PID4..7 zero, the
 * usual B105F00D cell ID).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "qom/object.h"

#define TYPE_ARM_SP805 "arm-sp805"

static const uint32_t arm_sp805_id[] = {
    0x00, 0x00, 0x00, 0x00, /* PID4..PID7 */
    0x05, 0x18, 0x14, 0x00, /* PID0..PID3: part 0x805, ARM, revision 1 */
    0x0d, 0xf0, 0x05, 0xb1, /* CID0..CID3 */
};

/* Runs after cmsdk_apb_watchdog_init(), which set the CMSDK table. */
static void arm_sp805_init(Object *obj)
{
    CMSDK_APB_WATCHDOG(obj)->id = arm_sp805_id;
}

static const TypeInfo arm_sp805_info = {
    .name          = TYPE_ARM_SP805,
    .parent        = TYPE_CMSDK_APB_WATCHDOG,
    .instance_init = arm_sp805_init,
};

static void arm_sp805_register_types(void)
{
    type_register_static(&arm_sp805_info);
}

type_init(arm_sp805_register_types)
