/*
 * SigmaStar / MStar RIU register file (hw/misc/sstar-riu.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SSTAR_RIU_H
#define HW_MISC_SSTAR_RIU_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_SSTAR_RIU "sstar-riu"
OBJECT_DECLARE_SIMPLE_TYPE(SstarRiuState, SSTAR_RIU)

#define SSTAR_RIU_BASE      0x1F000000
#define SSTAR_RIU_SIZE      0x00400000

/* Byte address of register @reg of RIU bank @bank. */
#define SSTAR_RIU_ADDR(bank, reg) \
    (SSTAR_RIU_BASE + (hwaddr)(bank) * 0x200 + (reg) * 4)

/* A reset value: absolute address of a 16-bit RIU register, value. */
typedef struct SstarRegDefault {
    hwaddr addr;
    uint16_t val;
} SstarRegDefault;

#define SSTAR_RIU_MAX_DEFAULTS  16

struct SstarRiuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint16_t *regs;
    unsigned long *seen;
    char *trace;
    unsigned long *trace_banks;
    uint32_t poll_idx;
    uint32_t poll_count;
    uint16_t bond;
    /* tables of reset values, each ended by a zero address */
    const SstarRegDefault *defaults[SSTAR_RIU_MAX_DEFAULTS];
    uint32_t num_defaults;
};

uint16_t sstar_riu_get(SstarRiuState *s, uint32_t bank, uint32_t reg);
/* What a guest read sees, status bits included. */
uint16_t sstar_riu_read16(SstarRiuState *s, uint32_t bank, uint32_t reg);
void sstar_riu_set(SstarRiuState *s, uint32_t bank, uint32_t reg,
                   uint16_t val);

/* Before realize: reset values for registers the RIU stores. */
void sstar_riu_add_defaults(SstarRiuState *s, const SstarRegDefault *table);

#endif /* HW_MISC_SSTAR_RIU_H */
