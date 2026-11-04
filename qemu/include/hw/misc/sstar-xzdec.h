/*
 * SigmaStar XZ decoder (Iford RIU banks 0x11D7-0x11DC).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_XZDEC_H
#define HW_MISC_SSTAR_XZDEC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_SSTAR_XZDEC "sstar-xzdec"
OBJECT_DECLARE_SIMPLE_TYPE(SstarXzdecState, SSTAR_XZDEC)

/* BDMA destination port 6: the compressed stream. */
void sstar_xzdec_push(SstarXzdecState *s, const uint8_t *buf, uint32_t len);

#endif /* HW_MISC_SSTAR_XZDEC_H */
