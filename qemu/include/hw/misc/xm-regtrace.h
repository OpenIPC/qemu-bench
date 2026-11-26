/*
 * XM_REGTRACE=<file>: log guest register and sensor writes, one line each,
 * with the virtual time and whether the CPU was in user or kernel mode.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_XM_REGTRACE_H
#define HW_MISC_XM_REGTRACE_H

#include "qemu/timer.h"

static inline FILE *xm_regtrace_file(void)
{
    static FILE *f;
    static bool tried;

    if (!tried) {
        const char *p = getenv("XM_REGTRACE");

        tried = true;
        if (p && *p) {
            f = fopen(p, "a");
            if (f) {
                setvbuf(f, NULL, _IOLBF, 0);
            }
        }
    }
    return f;
}

/* 'U' or 'K': the CPU mode of the access (hw/arm/xiongmai.c) */
char xm_regtrace_mode(void);

static inline void xm_regtrace(const char *dev, uint64_t addr, uint64_t val)
{
    FILE *f = xm_regtrace_file();

    if (f) {
        fprintf(f, "%.6f %c %s %08" PRIx64 " %08" PRIx64 "\n",
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1e9,
                xm_regtrace_mode(), dev, addr, val);
    }
}

#endif
