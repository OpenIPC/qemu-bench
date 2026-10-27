/*
 * Novatek NA51089 IVE work (hw/misc/nvt-ive.c), for the nvt-media "ive"
 * engine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_IVE_H
#define HW_MISC_NVT_IVE_H

/*
 * Run the job the IVE's registers (regs[off / 4]) describe, reading and
 * writing guest memory; the IRV counters come back updated.
 */
void nvt_ive_run(uint32_t *regs);

#endif
