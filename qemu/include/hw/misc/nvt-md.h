/*
 * Novatek NA51089 MDBC motion detection work (hw/misc/nvt-md.c), for the
 * nvt-media "md" engine.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_MD_H
#define HW_MISC_NVT_MD_H

/*
 * Run the job the MDBC's registers (regs[off / 4]) describe, reading and
 * writing guest memory; the frame counter, random state and luminance
 * difference registers come back updated.
 */
void nvt_md_run(uint32_t *regs);

#endif
