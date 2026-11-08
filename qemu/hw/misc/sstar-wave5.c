/*
 * SigmaStar Chips&Media Wave5 video encoder, at its host interface.
 *
 * infinity6e and infinity6b0 carry a Wave5 (product 0x5210, firmware
 * chagall.bin, "VENC200"), 32-bit registers at 0x1F345200, interrupt
 * INT_IRQ_MHE; infinity6c another at 0x1F34D000 (INT_IRQ_MHE_1).  The firmware runs on the core's own CPU, so what is
 * modelled is what it does for the host (mhal.ko Wave5Vpu*, codec3_*):
 *
 *   0x004  program counter, read as "running" (Wave5VpuIsInit)
 *   0x020  FIO control: address; bit 31 reads "ready".  0x024 FIO data.
 *   0x034  interrupt reason clear, 0x03C VINT clear, 0x044 VINT status,
 *   0x048  interrupt enable, 0x04C interrupt reason
 *   0x038  host interrupt request: bit 0 runs the command in 0x100
 *   0x06C  core start: runs INIT_VPU the same way
 *   0x070  busy
 *   0x100  command, 0x104 option, 0x108 success, 0x10C fail reason
 *   0x110  instance
 *   0x1E0  queue status
 *   0x1E8  instances with a picture done, a bit each; the host clears them
 *          (MHAL_VENC_Wrapper_GetEngStatus)
 *
 * Command 0x4000 fetches results: option 0x40 the sequence's
 * (Wave5VpuEncGetSeqInfo), 0x02 the last picture's (Wave5VpuEncGetResult).
 * They are published then: the registers overlap the commands' inputs.
 *
 * Commands set a bit in the interrupt reason, the bit being the command's
 * own (1 << log2(command)), if 0x048 enables it: the host polls the others.  Results go in the registers
 * Wave5VpuEncGetResult reads (0x1DC success, 0x1D8 fail reason, 0x124
 * picture type, 0x128 POC, 0x12C recon index, 0x130 slices, 0x14C bytes,
 * 0x150 GOP index, 0x154 source index, 0x168 bitstream address, 0x18C
 * "release source" flags).  Wave5VpuEncGetHeader's and Wave5VpuEncode's
 * inputs: 0x118 the buffer, 0x11C its size, 0x148/0x14C/0x150 the source
 * planes, 0x154 the strides (luma << 16 | chroma).
 *
 * infinity6c's firmware takes pictures from a queue instead (mi.ko
 * DrvVcodecVdiWriteCmd): sixteen 256-byte pages in memory, each an image of
 * registers 0x100-0x1FF.  0x16C at INIT_VPU holds their address, 0x1F8 is
 * the host's write index and 0x1F4 the firmware's read index.  Its addresses
 * are relative to "mem-base" (DrvVcodecVdiGetBaseAddr), the sources' are
 * not, and the version INIT_VPU reports is "fw-version".  A picture's results
 * go to memory too: CREATE names a buffer in 0x14C, where the firmware leaves
 * an image of registers 0x100-0x1FF for each picture (the driver's
 * DrvVcodecWave5VpuReadRegV2): 0x100 the picture's id from 0x1C0, 0x104 its
 * error bits.
 *
 * A source on the scaler's hardware ring comes straight from the scaler: it
 * holds a few lines, never a frame (sstar_i6e_scl_ring()).
 *
 * The bitstream comes from hevc-enc.c: VPS, SPS and PPS for headers, IDR
 * and TRAIL_R pictures for the rest, against the stream's own reconstructed
 * picture.  The instance register's high half is the codec, 3 for AVC, seen
 * from the driver: such an instance gets a baseline CAVLC SPS and PPS of the
 * model's own, and intra pictures from hantro-h264.c, so its stream has no
 * P slices whatever the result registers say.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-i6e-scl.h"
#include "hw/misc/sstar-riu.h"
#include "hevc-enc.h"
#include "hantro-h264.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_SSTAR_WAVE5 "sstar-wave5"
OBJECT_DECLARE_SIMPLE_TYPE(SstarWave5State, SSTAR_WAVE5)

#define WAVE5_SIZE          0x800
#define WAVE5_DRAM          0x20000000u
#define WAVE5_MMU_PAGE      0x1000
#define WAVE5_INSTANCES     8
#define WAVE5_MAX_DIM       8192
#define WAVE5_MAX_STRIDE    (2 * WAVE5_MAX_DIM)
#define WAVE5_MAX_BS        (64 * MiB)
#define WAVE5_QP            30

#define R_PC                0x004
#define R_FIO_CTRL          0x020
#define R_FIO_DATA          0x024
#define R_REASON_CLEAR      0x034
#define R_HOST_INT          0x038
#define R_VINT_CLEAR        0x03C
#define R_VINT_STS          0x044
#define R_INT_ENABLE        0x048
#define R_INT_REASON        0x04C
#define R_RESET_REQ         0x050
#define R_RESET_STATUS      0x054
#define R_CORE_START        0x06C
#define R_BUSY              0x070
#define R_COMMAND           0x100
#define R_OPTION            0x104
#define R_SUCCESS           0x108
#define R_FAIL              0x10C
#define R_INSTANCE          0x110
#define R_RD_PTR            0x114
#define R_BS_START          0x118       /* also the write pointer's read */
#define R_BS_SIZE           0x11C
#define R_SRC_Y             0x148
#define R_SRC_CB            0x14C
#define R_SRC_STRIDE        0x154
#define R_CQ_BASE           0x16C
#define R_CQ_RD             0x1F4
#define R_CQ_WR             0x1F8
#define R_CQ_DUMP           0x14C
#define R_CQ_FIRE_ID        0x1C0
#define R_ENC_SUCCESS       0x1DC
#define R_ENC_FAIL          0x1D8
#define R_ENC_WARN          0x1D4
#define R_QUEUE             0x1E0
#define R_DONE_INST         0x1E8

#define CMD_INIT_VPU        0x0001
#define CMD_CREATE          0x0008
#define CMD_DESTROY         0x0020
#define CMD_INIT_SEQ        0x0040
#define CMD_SET_FB          0x0080
#define CMD_ENC_PIC         0x0100
#define CMD_SET_PARAM       0x0200
#define CMD_QUERY           0x4000

#define WAVE5_CODEC_AVC     3

#define QUERY_PIC           0x02
#define QUERY_SEQ           0x40

#define WAVE5_FW_VERSION    0x22050121u /* the firmware's own build */
#define WAVE5_CQ_PAGES      16
#define WAVE5_CQ_PAGE       0x100
#define WAVE5_CQ_REGS_END   0x1E0       /* status lives above: not an image */

typedef struct Wave5Inst {
    uint32_t width, height;
    uint32_t poc;
    uint32_t pics;
    uint32_t gop;
    uint32_t last;          /* the command a result query reports */
    uint32_t bytes;
    uint32_t bs;
    bool idr;
    uint32_t dump;          /* the queue's results, per instance */
    HevcEncRef *ref;
    bool avc;
    H264Enc h264;
} Wave5Inst;

struct SstarWave5State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[WAVE5_SIZE / 4];
    uint32_t pending_reason;
    uint32_t pending_inst;
    Wave5Inst inst[WAVE5_INSTANCES];
    uint32_t mem_base;
    uint32_t fw_version;
    uint32_t cq_base;
    uint32_t cq_rd;
    uint32_t cq_wr;
    uint32_t fire_id;
    bool cq_busy;
    bool trace;
};

static void wave5_mem(SstarWave5State *s, uint64_t miu, void *buf, size_t len,
                      bool write)
{
    uint8_t *p = buf;

    miu += s->mem_base;

    while (len) {
        size_t n = MIN(len, WAVE5_MMU_PAGE - (miu & (WAVE5_MMU_PAGE - 1)));

        /* a page with no MMU entry: nothing is behind it on the bus */
        if (!sstar_miu_missed(miu)) {
            dma_memory_rw(&address_space_memory,
                          WAVE5_DRAM + sstar_miu_xlate(miu), p, n,
                          write ? DMA_DIRECTION_FROM_DEVICE
                                : DMA_DIRECTION_TO_DEVICE,
                          MEMTXATTRS_UNSPECIFIED);
        }
        miu += n;
        p += n;
        len -= n;
    }
}

static inline uint32_t *wave5_reg(SstarWave5State *s, uint32_t off)
{
    return &s->regs[(off & (WAVE5_SIZE - 1)) / 4];
}

static void wave5_update_irq(SstarWave5State *s)
{
    qemu_set_irq(s->irq, *wave5_reg(s, R_VINT_STS) &&
                 (*wave5_reg(s, R_INT_REASON) & *wave5_reg(s, R_INT_ENABLE)));
}

static void wave5_finish(SstarWave5State *s, uint32_t cmd)
{
    /* a command the host polls for raises nothing */
    *wave5_reg(s, R_INT_REASON) |= cmd & *wave5_reg(s, R_INT_ENABLE);
    *wave5_reg(s, R_VINT_STS) = !!*wave5_reg(s, R_INT_REASON);
    *wave5_reg(s, R_BUSY) = 0;
    wave5_update_irq(s);
}

static void wave5_cq_run(SstarWave5State *s);
static void wave5_result(SstarWave5State *s, Wave5Inst *in, uint32_t opt);

/* A queued picture's results: the registers, as memory the host reads */
static void wave5_dump(SstarWave5State *s)
{
    Wave5Inst *in = &s->inst[s->pending_inst];
    uint32_t page[WAVE5_CQ_PAGE / 4];
    uint32_t off;

    wave5_result(s, in, QUERY_PIC);
    *wave5_reg(s, R_SUCCESS) = 1;
    *wave5_reg(s, R_FAIL) = 0;
    for (off = 0; off < WAVE5_CQ_PAGE; off += 4) {
        page[off / 4] = cpu_to_le32(*wave5_reg(s, R_COMMAND + off));
    }
    page[0] = cpu_to_le32(s->fire_id);
    page[1] = 0;
    wave5_mem(s, in->dump, page, sizeof(page), true);
}

static void wave5_done(void *opaque)
{
    SstarWave5State *s = opaque;

    if (s->cq_busy) {
        wave5_dump(s);
        /* bit 24: the low byte is the instance (DrvVcodecIrqHandler) */
        *wave5_reg(s, R_DONE_INST) = 1u << 24 | s->pending_inst;
    } else {
        *wave5_reg(s, R_DONE_INST) |= 1u << s->pending_inst;
    }
    wave5_finish(s, s->pending_reason);
    if (s->cq_busy) {
        s->cq_busy = false;
        s->cq_rd = (s->cq_rd + 1) % WAVE5_CQ_PAGES;
        *wave5_reg(s, R_CQ_RD) = s->cq_rd;
        wave5_cq_run(s);
    }
}

/* NV12 from the planes the command names; the encoder's input */
static void wave5_source(SstarWave5State *s, Wave5Inst *in, uint8_t **y,
                         uint8_t **uv, uint32_t *ys, uint32_t *cs)
{
    uint32_t stride = *wave5_reg(s, R_SRC_STRIDE);
    size_t ylen, clen;

    *ys = (stride >> 16) ? stride >> 16 : in->width;
    *cs = (stride & 0xFFFF) ? stride & 0xFFFF : *ys;
    /* every plane holds a row of the picture, at a size the host can spare */
    *ys = MIN(MAX(*ys, in->width), WAVE5_MAX_STRIDE);
    *cs = MIN(MAX(*cs, in->width), WAVE5_MAX_STRIDE);
    ylen = (size_t)*ys * in->height;
    /* the scaler's ring writes chroma rows as wide as luma's */
    clen = (size_t)MAX(*cs, *ys) * (in->height / 2);
    *y = g_malloc(ylen);
    *uv = g_malloc(clen);
    if (sstar_i6e_scl_ring(*wave5_reg(s, R_SRC_Y), *ys, in->height, *y,
                           *uv)) {
        *cs = *ys;
        return;
    }
    if (*ys >= in->width && sstar_iford_scl_ring(*wave5_reg(s, R_SRC_Y),
                                                 in->width, in->height, *y,
                                                 *uv)) {
        *ys = *cs = in->width;
        return;
    }
    wave5_mem(s, *wave5_reg(s, R_SRC_Y) - s->mem_base, *y, ylen, false);
    wave5_mem(s, *wave5_reg(s, R_SRC_CB) - s->mem_base, *uv, clen, false);
}

/* One access unit into the instance's buffer; returns its bytes */
/* ---- AVC ------------------------------------------------------------ */

/* One picture, the plane padded out to whole macroblocks. */
static size_t wave5_avc_frame(SstarWave5State *s, Wave5Inst *in, bool idr,
                              uint8_t *out, size_t cap)
{
    uint32_t mbw = DIV_ROUND_UP(in->width, 16);
    uint32_t mbh = DIV_ROUND_UP(in->height, 16);
    g_autofree uint8_t *y = NULL, *uv = NULL;
    g_autofree uint8_t *py = g_malloc(mbw * mbh * 256);
    g_autofree uint8_t *pc = g_malloc(mbw * mbh * 128);
    uint32_t ys, cs, l, ch = in->height / 2;
    size_t hdr = 0;

    wave5_source(s, in, &y, &uv, &ys, &cs);
    for (l = 0; l < mbh * 16; l++) {
        const uint8_t *src = y + MIN(l, in->height - 1) * ys;

        memcpy(py + l * mbw * 16, src, in->width);
        memset(py + l * mbw * 16 + in->width, src[in->width - 1],
               mbw * 16 - in->width);
    }
    for (l = 0; l < mbh * 8; l++) {
        const uint8_t *src = uv + MIN(l, ch - 1) * cs;

        memcpy(pc + l * mbw * 16, src, in->width & ~1);
        memset(pc + l * mbw * 16 + (in->width & ~1), 0x80,
               mbw * 16 - (in->width & ~1));
    }
    /* implicit headers, in front of an IDR */
    if (idr && cap > 128) {
        hdr = h264_enc_baseline_headers(in->width, in->height, out);
    }
    return hdr + h264_enc_frame(&in->h264, idr, WAVE5_QP, mbw, mbh,
                                py, mbw * 16, pc, mbw * 16,
                                out + hdr, cap - hdr);
}

static size_t wave5_encode(SstarWave5State *s, Wave5Inst *in, bool headers)
{
    uint32_t bs = *wave5_reg(s, R_BS_START);
    uint32_t cap = *wave5_reg(s, R_BS_SIZE);
    bool idr = headers || in->pics % MAX(in->gop, 1) == 0;
    uint32_t w = QEMU_ALIGN_DOWN(in->width, 8);
    uint32_t h = QEMU_ALIGN_DOWN(in->height, 8);
    g_autofree uint8_t *y = NULL, *uv = NULL, *out = NULL;
    uint32_t ys, cs;
    size_t n;

    if (!w || !h || !cap) {
        return 0;
    }
    /* A header request is no picture: the driver sends them mid-GOP. */
    if (idr && !headers) {
        in->poc = 0;
    }
    if (!in->ref) {
        in->ref = hevc_enc_ref_new();
    }
    cap = MIN(cap, WAVE5_MAX_BS);
    out = g_malloc(MAX(cap, 128));
    if (in->avc) {
        if (headers) {
            n = h264_enc_baseline_headers(in->width, in->height, out);
            h264_enc_parse_headers(&in->h264, out, n);
        } else {
            if (!in->h264.have_sps) {
                uint8_t sets[128];

                h264_enc_parse_headers(&in->h264, sets,
                                       h264_enc_baseline_headers(in->width,
                                                         in->height, sets));
            }
            n = wave5_avc_frame(s, in, idr, out, cap);
        }
    } else if (headers) {
        /* parameter sets: an IDR of grey, cut before its slice */
        ys = cs = w;
        y = g_malloc(w * h);
        uv = g_malloc(w * h / 2);
        memset(y, 0x80, w * h);
        memset(uv, 0x80, w * h / 2);
        n = hevc_enc_frame(&hevc_params_ctb32_sao, w, h, WAVE5_QP, false, 0,
                           NULL, NULL, y, w, h, ys, uv, cs, out, cap);
    } else {
        wave5_source(s, in, &y, &uv, &ys, &cs);
        n = hevc_enc_frame(&hevc_params_ctb32_sao, w, h, WAVE5_QP, !idr,
                           in->poc, in->ref, in->ref, y, in->width,
                           in->height, ys, uv, cs, out, cap);
    }
    n = MIN(n, cap);
    /*
     * The parameter sets alone for headers.  A picture's code option (0x164)
     * has bit 0 set by Wave5VpuEncode, implicit headers: the firmware puts
     * them in front of an IDR, as hevc_enc_frame() does.
     */
    if (headers && !in->avc) {
        size_t i;

        for (i = 0; i + 4 < n; i++) {
            if (!out[i] && !out[i + 1] && !out[i + 2] && out[i + 3] == 1 &&
                ((out[i + 4] >> 1) & 0x3F) < 32) {
                n = i;
                break;
            }
        }
    }
    wave5_mem(s, bs, out, n, true);
    if (!headers) {
        in->pics++;
        in->poc++;
    }
    return n;
}

static void wave5_result(SstarWave5State *s, Wave5Inst *in, uint32_t opt)
{
    *wave5_reg(s, R_ENC_SUCCESS) = 1;
    *wave5_reg(s, R_ENC_FAIL) = 0;
    *wave5_reg(s, R_ENC_WARN) = 0;
    if (opt == QUERY_SEQ) {
        *wave5_reg(s, 0x11C) = 1;               /* required framebuffers */
        *wave5_reg(s, 0x120) = 1;               /* minimum sources */
        *wave5_reg(s, 0x16C) = 0;
        *wave5_reg(s, 0x1B0) = 0x10000;         /* VLC buffer */
        *wave5_reg(s, 0x1B4) = 0x10000;         /* parameter buffer */
        return;
    }
    *wave5_reg(s, 0x124) = in->idr ? 0 : 1;     /* picture type: I, P */
    *wave5_reg(s, 0x128) = in->poc;
    *wave5_reg(s, 0x12C) = 0;                   /* recon index */
    *wave5_reg(s, 0x130) = 1;                   /* slices */
    *wave5_reg(s, 0x134) = 0;                   /* skipped */
    *wave5_reg(s, 0x148) = WAVE5_QP;
    *wave5_reg(s, 0x14C) = in->bytes;
    *wave5_reg(s, 0x150) = 0;
    *wave5_reg(s, 0x154) = 0;
    *wave5_reg(s, 0x158) = in->pics;
    *wave5_reg(s, 0x168) = in->bs;
    *wave5_reg(s, 0x18C) = 1;
    *wave5_reg(s, 0x190) = 0;
    *wave5_reg(s, R_RD_PTR) = in->bs;
    *wave5_reg(s, R_BS_START) = in->bs + in->bytes; /* the write pointer */
}

static void wave5_command(SstarWave5State *s)
{
    uint32_t cmd = *wave5_reg(s, R_COMMAND);
    uint32_t id = *wave5_reg(s, R_INSTANCE) & 0xFFFF;
    Wave5Inst *in = &s->inst[id % WAVE5_INSTANCES];
    uint32_t reason = cmd;
    uint32_t opt = *wave5_reg(s, R_OPTION);

    if (s->trace) {
        qemu_log("sstar-wave5: cmd 0x%04x opt 0x%x inst 0x%x\n", cmd, opt,
                 *wave5_reg(s, R_INSTANCE));
    }
    *wave5_reg(s, R_SUCCESS) = 1;
    *wave5_reg(s, R_FAIL) = 0;
    *wave5_reg(s, R_QUEUE) = 0;

    switch (cmd) {
    case CMD_INIT_VPU:
        *wave5_reg(s, R_PC) = 0x1000;
        s->cq_base = *wave5_reg(s, R_CQ_BASE);
        s->cq_rd = s->cq_wr = 0;
        break;
    case CMD_CREATE:
        hevc_enc_ref_free(in->ref);
        memset(in, 0, sizeof(*in));
        in->dump = *wave5_reg(s, R_CQ_DUMP);
        in->avc = *wave5_reg(s, R_INSTANCE) >> 16 == WAVE5_CODEC_AVC;
        break;
    case CMD_DESTROY:
        hevc_enc_ref_free(in->ref);
        memset(in, 0, sizeof(*in));
        break;
    case CMD_SET_PARAM:
        /* W5_CMD_ENC_SEQ_SRC_SIZE, height << 16 | width (option 0) */
        if (opt == 0) {
            uint32_t size = *wave5_reg(s, 0x11C);
            uint32_t intra = *wave5_reg(s, 0x130) >> 16;

            in->width = MIN(size & 0xFFFF, WAVE5_MAX_DIM);
            in->height = MIN(size >> 16, WAVE5_MAX_DIM);
            in->gop = intra ? intra : 30;
        }
        in->last = cmd;
        break;
    case CMD_INIT_SEQ:
    case CMD_SET_FB:
        in->last = cmd;
        break;
    case CMD_ENC_PIC:
        /* option: 0x10 Wave5VpuEncGetHeader, 0x02 Wave5VpuEncode */
        in->last = cmd;
        in->bs = *wave5_reg(s, R_BS_START);
        if (opt == 0x10) {
            in->idr = true;
            in->bytes = wave5_encode(s, in, true);
        } else {
            in->idr = in->pics % MAX(in->gop, 1) == 0;
            in->bytes = wave5_encode(s, in, false);
        }
        if (s->trace) {
            qemu_log("sstar-wave5: pic %u bytes at 0x%x idr %d %ux%u "
                     "src 0x%x 0x%x stride 0x%x fmt 0x%x\n", in->bytes,
                     in->bs, in->idr, in->width, in->height,
                     *wave5_reg(s, R_SRC_Y), *wave5_reg(s, R_SRC_CB),
                     *wave5_reg(s, R_SRC_STRIDE), *wave5_reg(s, 0x158));
        }
        break;
    case CMD_QUERY:
        if (opt == QUERY_PIC || opt == QUERY_SEQ) {
            wave5_result(s, in, opt);
        } else {
            *wave5_reg(s, R_BS_START) = s->fw_version;
        }
        break;
    default:
        break;
    }

    /* a picture takes a frame's time; everything else is at once */
    if (cmd == CMD_ENC_PIC) {
        s->pending_reason = reason;
        s->pending_inst = id % WAVE5_INSTANCES;
        timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  SCALE_MS * 5);
    } else {
        wave5_finish(s, reason);
    }
}

/* The pictures in the queue, one at a time: each takes a frame's time */
static void wave5_cq_run(SstarWave5State *s)
{
    uint32_t page[WAVE5_CQ_PAGE / 4];
    uint32_t off;

    while (!s->cq_busy && s->cq_rd != s->cq_wr) {
        wave5_mem(s, s->cq_base + s->cq_rd * WAVE5_CQ_PAGE, page, sizeof(page),
                  false);
        if (s->trace) {
            qemu_log("sstar-wave5: queue %u/%u at 0x%x (0x%" PRIx64
                     "): %08x %08x\n", s->cq_rd, s->cq_wr, s->cq_base,
                     sstar_miu_xlate(s->mem_base + s->cq_base +
                                     s->cq_rd * WAVE5_CQ_PAGE),
                     page[0], page[1]);
        }
        for (off = R_COMMAND; off < WAVE5_CQ_REGS_END; off += 4) {
            *wave5_reg(s, off) = le32_to_cpu(page[(off - R_COMMAND) / 4]);
        }
        s->cq_busy = true;
        s->fire_id = *wave5_reg(s, R_CQ_FIRE_ID);
        if (*wave5_reg(s, R_COMMAND) == CMD_ENC_PIC) {
            wave5_command(s);
            return;
        }
        wave5_command(s);
        s->cq_busy = false;
        s->cq_rd = (s->cq_rd + 1) % WAVE5_CQ_PAGES;
        *wave5_reg(s, R_CQ_RD) = s->cq_rd;
    }
}

static uint64_t sstar_wave5_read(void *opaque, hwaddr off, unsigned size)
{
    SstarWave5State *s = opaque;

    switch (off) {
    case R_FIO_CTRL:
        return *wave5_reg(s, off) | 0x80000000u;
    case R_FIO_DATA:
        return 0x3F;        /* the bus idle: codec3_wait_bus_busy */
    case R_RESET_STATUS:
        return 0;
    default:
        return *wave5_reg(s, off);
    }
}

static void sstar_wave5_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    SstarWave5State *s = opaque;

    switch (off) {
    case R_REASON_CLEAR:
        if (s->trace) {
            qemu_log("sstar-wave5: clear reason 0x%x of 0x%x sts %u en 0x%x\n",
                     (uint32_t)val, *wave5_reg(s, R_INT_REASON),
                     *wave5_reg(s, R_VINT_STS), *wave5_reg(s, R_INT_ENABLE));
        }
        *wave5_reg(s, R_INT_REASON) &= ~(uint32_t)val;
        wave5_update_irq(s);
        break;
    case R_VINT_CLEAR:
        *wave5_reg(s, R_VINT_STS) = 0;
        wave5_update_irq(s);
        break;
    case R_HOST_INT:
    case R_CORE_START:
        *wave5_reg(s, off) = val;
        if (val & 1) {
            wave5_command(s);
        }
        break;
    case R_CQ_WR:
        *wave5_reg(s, off) = val;
        s->cq_wr = val % WAVE5_CQ_PAGES;
        wave5_cq_run(s);
        break;
    default:
        *wave5_reg(s, off) = val;
        if (off == R_INT_ENABLE) {
            wave5_update_irq(s);
        }
        break;
    }
}

static const MemoryRegionOps sstar_wave5_ops = {
    .read = sstar_wave5_read,
    .write = sstar_wave5_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sstar_wave5_reset(DeviceState *dev)
{
    SstarWave5State *s = SSTAR_WAVE5(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < WAVE5_INSTANCES; i++) {
        hevc_enc_ref_free(s->inst[i].ref);
        memset(&s->inst[i], 0, sizeof(s->inst[i]));
    }
    s->cq_base = s->cq_rd = s->cq_wr = 0;
    s->cq_busy = false;
    timer_del(s->done);
    qemu_set_irq(s->irq, 0);
}

static void sstar_wave5_realize(DeviceState *dev, Error **errp)
{
    SstarWave5State *s = SSTAR_WAVE5(dev);

    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, wave5_done, s);
}

static void sstar_wave5_init(Object *obj)
{
    SstarWave5State *s = SSTAR_WAVE5(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_wave5_ops, s,
                          TYPE_SSTAR_WAVE5, WAVE5_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_wave5_props[] = {
    DEFINE_PROP_UINT32("mem-base", SstarWave5State, mem_base, 0),
    DEFINE_PROP_UINT32("fw-version", SstarWave5State, fw_version,
                       WAVE5_FW_VERSION),
    DEFINE_PROP_BOOL("trace", SstarWave5State, trace, false),
};

/* The instances' queues and pictures are host state: refuse, do not lose them */
static const VMStateDescription vmstate_sstar_wave5 = {
    .name = TYPE_SSTAR_WAVE5,
    .unmigratable = 1,
};

static void sstar_wave5_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &vmstate_sstar_wave5;
    dc->realize = sstar_wave5_realize;
    device_class_set_legacy_reset(dc, sstar_wave5_reset);
    device_class_set_props(dc, sstar_wave5_props);
}

static const TypeInfo sstar_wave5_info = {
    .name           = TYPE_SSTAR_WAVE5,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarWave5State),
    .instance_init  = sstar_wave5_init,
    .class_init     = sstar_wave5_class_init,
};

static void sstar_wave5_register_types(void)
{
    type_register_static(&sstar_wave5_info);
}

type_init(sstar_wave5_register_types)
