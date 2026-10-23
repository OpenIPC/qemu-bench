/*
 * Novatek NA51089 AI engines' work (hw/misc/nvt-ai.c), for the nvt-media
 * NUE / NUE2 / CNN engines.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_NVT_AI_H
#define HW_MISC_NVT_AI_H

/*
 * A job's writes to guest memory, logged as the job runs (on a worker
 * thread) and done when the main loop commits them.
 */
typedef struct NvtAiWrites NvtAiWrites;

NvtAiWrites *nvt_ai_writes_new(void);
void nvt_ai_writes_commit(NvtAiWrites *w);
void nvt_ai_writes_free(NvtAiWrites *w);

/*
 * Run the job each engine's registers (regs[off / 4]) describe, reading
 * guest memory and logging its writes in w.
 */
void nvt_ai_nue_run(const uint32_t *regs, NvtAiWrites *w);
void nvt_ai_nue2_run(const uint32_t *regs, NvtAiWrites *w);
void nvt_ai_cnn_run(const uint32_t *regs, NvtAiWrites *w);

#endif
