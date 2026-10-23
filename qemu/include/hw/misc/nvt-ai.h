/*
 * Novatek NA51089 AI engines' work (hw/misc/nvt-ai.c), for the nvt-media
 * NUE / NUE2 / CNN engines.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_AI_H
#define HW_MISC_NVT_AI_H

/*
 * Run the job each engine's registers (regs[off / 4]) describe, reading
 * and writing guest memory.
 */
void nvt_ai_nue_run(const uint32_t *regs);
void nvt_ai_nue2_run(const uint32_t *regs);
void nvt_ai_cnn_run(const uint32_t *regs);

#endif
