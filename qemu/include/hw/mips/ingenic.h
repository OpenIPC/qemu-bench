/*
 * Ingenic T-series camera SoCs: the identifiers shared by the machine
 * (hw/mips/ingenic.c) and the device models that differ per SoC.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MIPS_INGENIC_H
#define HW_MIPS_INGENIC_H

#include "hw/qdev-core.h"

/* The "soc" property of the per-SoC device models. */
typedef enum IngenicSoc {
    INGENIC_T10,
    INGENIC_T20,
    INGENIC_T21,
    INGENIC_T23,
    INGENIC_T30,
    INGENIC_T31,
    INGENIC_T40,
    INGENIC_T41,
} IngenicSoc;

/* XBurst2 parts: CCU, per-core INTC/OST, SFC v2, 4.4 kernels. */
static inline bool ingenic_is_xburst2(IngenicSoc soc)
{
    return soc == INGENIC_T40 || soc == INGENIC_T41;
}

/*
 * The CPM's I2S master clock in Hz for the AIC's play (tx) or record
 * side, 0 where the AIC takes its rate from the codec (hw/misc/
 * ingenic-cpm.c).
 */
uint64_t ingenic_cpm_i2s_hz(DeviceState *cpm, bool tx);

#endif /* HW_MIPS_INGENIC_H */
