/*
 * Novatek NA51089 2D graphics and ISE scaling work (hw/misc/nvt-gfx.c),
 * for the nvt-media engines.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_GFX_H
#define HW_MISC_NVT_GFX_H

/*
 * Run the job the graphics engine's registers (regs[off / 4]) describe,
 * reading and writing guest memory.
 */
void nvt_gfx_grph_run(const uint32_t *regs);

/* Likewise for the ISE scaler. */
void nvt_gfx_ise_run(const uint32_t *regs);

#endif
