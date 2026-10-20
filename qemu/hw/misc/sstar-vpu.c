/*
 * SigmaStar SSR621Q / infinity2m video decoder: a Chips&Media WAVE5-family
 * VPU (WAVE511, product code 0x5110), modelled at its firmware's host
 * interface, with libavcodec doing the decoding.
 *
 * The host loads a firmware image (chagall.bin) and drives the VCPU through
 * a register mailbox at 0x1F344800 (mainline Linux's
 * drivers/media/platform/chips-media/wave5 drives the same interface).
 * Here no firmware runs; the device answers the mailbox itself:
 *
 *  - boot: the FIO indirect bus (backbone status reads idle), RESET_REQ,
 *    REMAP_CORE_START with INIT_VPU / WAKEUP_VPU (CUR_PC goes non-zero),
 *    QUERY GET_VPU_INFO (H.264 and HEVC decoders, no encoders);
 *  - per instance: CREATE / DESTROY / FLUSH_INSTANCE, SET_FB, UPDATE_BS,
 *    the QUERY options GET_RESULT, GET/SET_BS_RD_PTR and UPDATE_DISP_FLAG;
 *  - INIT_SEQ and DEC_PIC are queued (two deep, per instance) and finish
 *    from a timer a few milliseconds later: each leaves a result report
 *    for GET_RESULT, sets the instance in QUEUE_CMD_DONE_INST and raises
 *    the interrupt.  INIT_SEQ feeds the bitstream ring to libavcodec until
 *    the stream's size is known; DEC_PIC decodes one access unit and
 *    writes the picture it displays to memory.
 *
 * What the SigmaStar driver (mi_vdec.ko, CNM API 5.5.51, read off its
 * disassembly) does differently from mainline:
 *
 *  - INIT_SEQ completes through QUEUE_CMD_DONE_INST (0x1E8), as on
 *    mainline's WAVE515, and a completed DEC_PIC also leaves the
 *    instance's index in 0x1FC.  The queue status is read from 0x1F4.
 *  - The picture goes to an output buffer named with each DEC_PIC
 *    (0x150 flags, 0x154 size, 0x158/0x15C luma/chroma, 0x164 stride),
 *    NV12 and scaled to that size by the firmware: the "linear" frame
 *    buffer group the host registers carries no addresses.  The result
 *    must give that buffer back (0x188/0x18C) and the per-picture VLC
 *    buffer DEC_PIC passed in 0x168 (in 0x1A0): the host finds the output
 *    task by them.
 *  - Interrupt bit 11 / 0x1F0 asks for a bigger VLC buffer (the host
 *    answers with command 0x800); an emulation never needs one.
 *  - The host arms its wait for a completion after issuing the command:
 *    finishing inside the doorbell write loses it ("latency-us").
 *  - 0xF0/0xF4 are a console mailbox from the firmware, polled by a host
 *    thread; it stays idle.
 *
 * Without libavcodec at build time the device still boots and runs the
 * protocol, but every INIT_SEQ fails ("no sequence found").
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/blocker.h"
#include "migration/vmstate.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#ifdef SSTAR_VPU_LIBAVCODEC
#include <libavcodec/avcodec.h>
#endif

#define TYPE_SSTAR_VPU "sstar-vpu"
OBJECT_DECLARE_SIMPLE_TYPE(SstarVpuState, SSTAR_VPU)

#define VPU_WINDOW          0x10000

/* registers (mainline wave5-regdefine.h names) */
#define W5_VCPU_CUR_PC          0x0004
#define W5_VPU_FIO_CTRL_ADDR    0x0020
#define W5_VPU_FIO_DATA         0x0024
#define W5_VPU_VINT_REASON_USR  0x0030
#define W5_VPU_VINT_REASON_CLR  0x0034
#define W5_VPU_HOST_INT_REQ     0x0038
#define W5_VPU_VINT_CLEAR       0x003C
#define W5_VPU_VPU_INT_STS      0x0044
#define W5_VPU_VINT_ENABLE      0x0048
#define W5_VPU_VINT_REASON      0x004C
#define W5_VPU_RESET_REQ        0x0050
#define W5_VPU_RESET_STATUS     0x0054
#define W5_VPU_REMAP_CORE_START 0x006C
#define W5_VPU_BUSY_STATUS      0x0070
#define W5_VPU_DBG_REG0         0x00F0
#define W5_COMMAND              0x0100
#define W5_COMMAND_OPTION       0x0104
#define W5_RET_SUCCESS          0x0108
#define W5_RET_FAIL_REASON      0x010C
#define W5_CMD_INSTANCE_INFO    0x0110
#define W5_RET_QUEUE_STATUS     0x01E0
#define W5_RET_QUEUE_STATUS_SS  0x01F4
#define W5_PRODUCT_NUMBER       0x1044

/* QUERY GET_VPU_INFO results */
#define W5_RET_FW_VERSION       0x0118
#define W5_RET_PRODUCT_NAME     0x011C
#define W5_RET_PRODUCT_VERSION  0x0120
#define W5_RET_STD_DEF0         0x0124
#define W5_RET_STD_DEF1         0x0128
#define W5_RET_CONF_FEATURE     0x012C
#define W5_RET_CONF_DATE        0x0130
#define W5_RET_CONF_REVISION    0x0134
#define W5_RET_CONF_TYPE        0x0138

#define W5_INIT_VPU             0x0001
#define W5_WAKEUP_VPU           0x0002
#define W5_SLEEP_VPU            0x0004
#define W5_CREATE_INSTANCE      0x0008
#define W5_FLUSH_INSTANCE       0x0010
#define W5_DESTROY_INSTANCE     0x0020
#define W5_INIT_SEQ             0x0040
#define W5_SET_FB               0x0080
#define W5_DEC_PIC              0x0100
#define W5_UPDATE_VLC           0x0800  /* SigmaStar firmware */
#define W5_QUERY                0x4000
#define W5_UPDATE_BS            0x8000

#define GET_VPU_INFO            0

#define SYSERR_UNKNOWN_CMD      0x00002000
#define SYSERR_UNKNOWN_QUERY    0x00008000

/* the PC once the (emulated) firmware runs: anything but 0 */
#define VPU_FW_PC               0x00001000

#define FIO_CTRL_READY          (1u << 31)
#define FIO_CTRL_WRITE          (1u << 16)

#define WAVE511_CODE            0x5110
/* backbone GDI bus status, read through FIO: idle */
#define GDI_BUS_IDLE            0x3F

#define VPU_MAX_INST        32
#define VPU_MAX_FB          32
#define VPU_QUEUE_DEPTH     2
#define VPU_REPORT_WORDS    64          /* the result area 0x100..0x1FC */

/* more registers: the per-command parameter / result words */
#define W5_BS_RD_PTR                0x0118
#define W5_BS_WR_PTR                0x011C
#define W5_BS_OPTION                0x0120
#define W5_CMD_DEC_BS_START_ADDR    0x011C
#define W5_CMD_DEC_BS_SIZE          0x0120
#define W5_COMMON_PIC_INFO          0x0118
#define W5_PIC_SIZE                 0x011C
#define W5_SET_FB_NUM               0x0120
#define W5_ADDR_LUMA_BASE0          0x0134
#define W5_RET_DEC_COLOR_SAMPLE     0x0124
#define W5_RET_DEC_SEQ_PARAM        0x0120
#define W5_RET_DEC_NUM_REQUIRED_FB  0x0138
#define W5_RET_DEC_NUM_REORDER      0x013C
#define W5_RET_DEC_SUB_LAYER_INFO   0x0140
#define W5_RET_DEC_NOTIFICATION     0x0144
#define W5_RET_DEC_USERDATA_IDC     0x0148
#define W5_RET_DEC_PIC_SIZE         0x014C
#define W5_RET_DEC_CROP_TOP_BOTTOM  0x0150
#define W5_RET_DEC_CROP_LEFT_RIGHT  0x0154
#define W5_RET_DEC_AU_START_POS     0x0158
#define W5_RET_DEC_AU_END_POS       0x015C
#define W5_RET_DEC_PIC_TYPE         0x0160
#define W5_RET_DEC_PIC_POC          0x0164
#define W5_RET_DEC_DECODED_INDEX    0x0170
#define W5_RET_DEC_DISPLAY_INDEX    0x0174
#define W5_RET_DEC_DISP_IDC         0x017C
#define W5_RET_DEC_ERR_CTB_NUM      0x0180
#define W5_RET_VLC_BUF_SIZE         0x01B0
#define W5_RET_PARAM_BUF_SIZE       0x01B4
#define W5_RET_DEC_HOST_CMD_TICK    0x01B8
#define W5_RET_DEC_ENC_TICK         0x01D0
#define W5_RET_DEC_WARN_INFO        0x01D4
#define W5_RET_DEC_ERR_INFO         0x01D8
#define W5_RET_DEC_DECODING_SUCCESS 0x01DC
#define W5_RET_QUEUE_CMD_DONE_INST  0x01E8
#define W5_RET_DONE_INST_IDX        0x01FC  /* SigmaStar firmware */
#define W5_RET_QUERY_DEC_BS_RD_PTR  0x011C
#define W5_CMD_DEC_SET_DISP_IDC     0x0118
#define W5_CMD_DEC_CLR_DISP_IDC     0x011C

/* SigmaStar firmware: DEC_PIC's output picture and VLC buffer */
#define W5_CMD_DEC_OUT_FLAGS        0x0150
#define W5_CMD_DEC_OUT_SIZE         0x0154
#define W5_CMD_DEC_OUT_LUMA         0x0158
#define W5_CMD_DEC_OUT_CHROMA       0x015C
#define W5_CMD_DEC_OUT_STRIDE       0x0164
#define W5_CMD_DEC_VLC_ADDR         0x0168
#define W5_RET_DEC_OUT_LUMA         0x0188
#define W5_RET_DEC_OUT_CHROMA       0x018C
#define W5_RET_DEC_VLC_ADDR         0x01A0
#define W5_RET_DEC_VLC_USED         0x019C

#define BSOPT_EXPLICIT_END          (1u << 0)
#define BSOPT_STREAM_END            (1u << 1)
#define PIC_INFO_LINEAR             (1u << 28)
#define PIC_INFO_NV21               (1u << 17)
#define PIC_INFO_INTERLEAVE         (1u << 16)
#define SFB_FIRST                   (1u << 3)

#define QUERY_SET_WRITE_PROT        1
#define QUERY_GET_RESULT            2
#define QUERY_UPDATE_DISP_FLAG      3
#define QUERY_GET_BW_REPORT         4
#define QUERY_GET_BS_RD_PTR         5
#define QUERY_SET_BS_RD_PTR         8
#define QUERY_GET_DEBUG_INFO        0x61

#define INT_INIT_SEQ                6
#define INT_DEC_PIC                 8

#define STD_HEVC                    0
#define STD_AVC                     2

#define SYSERR_QUEUEING_FAIL        0x00000001
#define SYSERR_RESULT_NOT_READY     0x00000800
#define SYSERR_UNKNOWN_CODEC_STD    0x00004000
#define ERR_SEQ_NOT_FOUND           0x00005000

/* the decoded / display index "nothing" codes */
#define IDX_NO_FB                   (-1)
#define IDX_NONE                    (-2)
#define IDX_NOT_YET                 (-3)

/* DRAM, as the VPU's bus addresses reach it */
#define VPU_DRAM_BASE               0x20000000
#define VPU_DRAM_MASK               0x1FFFFFFF

typedef struct VpuReport {
    uint32_t w[VPU_REPORT_WORDS];
    uint64_t set;                       /* which words the result holds */
} VpuReport;

/*
 * The SigmaStar firmware takes the destination of each displayed picture
 * with the DEC_PIC command: its scaler writes it there, NV12, at the size
 * asked for.  The host's "linear" frame buffer group carries no addresses.
 */
typedef struct VpuOut {
    uint32_t flags;             /* 0x150 */
    uint32_t w, h;              /* 0x154: target size */
    uint32_t y, c;              /* 0x158, 0x15C */
    uint32_t stride;            /* 0x164 */
} VpuOut;

typedef struct VpuCmd {
    uint32_t cmd;
    uint32_t bs_wr;
    uint32_t bs_opt;
    uint32_t opt;
    VpuOut out;
    uint32_t vlc_addr;          /* DEC_PIC 0x168 */
} VpuCmd;

typedef struct VpuFb {
    uint32_t y, cb, cr;
} VpuFb;

typedef struct VpuInst {
    bool used;
    uint32_t std;
    uint32_t bs_start, bs_size;
    uint32_t rd, wr;
    bool stream_end;

    VpuCmd queue[VPU_QUEUE_DEPTH];
    int nqueue;
    VpuReport report[VPU_QUEUE_DEPTH + 1];
    int nreport;

    /* sequence */
    uint32_t pic_w, pic_h, crop_l, crop_r, crop_t, crop_b;
    uint32_t num_fb;

    /* the linear (WTL) output buffers */
    VpuFb lin[VPU_MAX_FB];
    uint32_t lin_info;          /* COMMON_PIC_INFO of the linear group */
    uint32_t lin_size;          /* PIC_SIZE of the linear group */
    uint32_t nlin;
    uint32_t nfbc;              /* compressed (reference) buffers */
    uint32_t disp_flags;        /* the host owns these buffers */
    uint32_t slot_busy;         /* assigned to a picture not yet shown */
    uint32_t pics;

#ifdef SSTAR_VPU_LIBAVCODEC
    AVCodecContext *ctx;
    AVCodecParserContext *parser;
    AVPacket *pkt;
    AVFrame *out[VPU_MAX_FB];   /* decoded, waiting for a DEC_PIC */
    int nout;
    AVFrame *last;              /* the last picture shown, to show again */
    int last_slot;
    bool drained;
#endif
} VpuInst;

struct SstarVpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;           /* the firmware finishing queued commands */
    uint32_t latency_us;
    Error *migration_blocker;
    uint32_t regs[VPU_WINDOW / 4];
    uint32_t fio[VPU_WINDOW / 4];   /* the indirect (FIO) register space */
    VpuInst inst[VPU_MAX_INST];
    uint32_t tick;
    bool trace;
};

static void vpu_update_irq(SstarVpuState *s)
{
    bool pending = s->regs[W5_VPU_VINT_REASON >> 2] &
                   s->regs[W5_VPU_VINT_ENABLE >> 2];

    s->regs[W5_VPU_VPU_INT_STS >> 2] = pending;
    qemu_set_irq(s->irq, pending);
}

static void vpu_set_reg(SstarVpuState *s, hwaddr off, uint32_t v)
{
    s->regs[off >> 2] = v;
}

static uint32_t vpu_reg(SstarVpuState *s, hwaddr off)
{
    return s->regs[off >> 2];
}

static void vpu_cmd_done(SstarVpuState *s, bool ok, uint32_t fail)
{
    vpu_set_reg(s, W5_RET_SUCCESS, ok);
    vpu_set_reg(s, W5_RET_FAIL_REASON, ok ? 0 : fail);
    vpu_set_reg(s, W5_VPU_BUSY_STATUS, 0);
}

static void vpu_queue_status(SstarVpuState *s, VpuInst *in)
{
    uint32_t v = in ? (in->nqueue << 16) | in->nreport : 0;

    vpu_set_reg(s, W5_RET_QUEUE_STATUS, v);
    /* where the SigmaStar driver reads it */
    vpu_set_reg(s, W5_RET_QUEUE_STATUS_SS, v);
}

static void rep_set(VpuReport *r, hwaddr off, uint32_t v)
{
    int i = (off - W5_COMMAND) >> 2;

    r->w[i] = v;
    r->set |= 1ull << i;
}

/* Bytes in the bitstream ring from rd up to wr. */
static uint32_t bs_avail(VpuInst *in, uint32_t rd, uint32_t wr)
{
    uint32_t end = in->bs_start + in->bs_size;

    if (rd < in->bs_start || rd >= end || wr < in->bs_start || wr > end) {
        return 0;
    }
    return wr >= rd ? wr - rd : end - rd + wr - in->bs_start;
}

/* ---- decoding ---- */

#ifdef SSTAR_VPU_LIBAVCODEC

/* guest memory, at the VPU's bus addresses */

static void vpu_dram_read(uint32_t bus, void *buf, uint32_t len)
{
    dma_memory_read(&address_space_memory,
                    VPU_DRAM_BASE + (bus & VPU_DRAM_MASK), buf, len,
                    MEMTXATTRS_UNSPECIFIED);
}

static void vpu_dram_write(uint32_t bus, const void *buf, uint32_t len)
{
    dma_memory_write(&address_space_memory,
                     VPU_DRAM_BASE + (bus & VPU_DRAM_MASK), buf, len,
                     MEMTXATTRS_UNSPECIFIED);
}

static uint8_t *bs_fetch(VpuInst *in, uint32_t rd, uint32_t len)
{
    uint32_t end = in->bs_start + in->bs_size;
    uint32_t first = MIN(len, end - rd);
    uint8_t *buf = g_malloc(len + 64);   /* + AV_INPUT_BUFFER_PADDING_SIZE */

    vpu_dram_read(rd, buf, first);
    if (len > first) {
        vpu_dram_read(in->bs_start, buf + first, len - first);
    }
    memset(buf + len, 0, 64);
    return buf;
}

static uint32_t bs_advance(VpuInst *in, uint32_t rd, uint32_t n)
{
    rd += n;
    if (rd >= in->bs_start + in->bs_size) {
        rd -= in->bs_size;
    }
    return rd;
}

static void inst_drop_frames(VpuInst *in)
{
    while (in->nout) {
        av_frame_free(&in->out[--in->nout]);
    }
    av_frame_free(&in->last);
    in->slot_busy = 0;
}

static void inst_free_codec(VpuInst *in)
{
    inst_drop_frames(in);
    av_parser_close(in->parser);
    in->parser = NULL;
    av_packet_free(&in->pkt);
    avcodec_free_context(&in->ctx);
}

static bool inst_open_codec(VpuInst *in)
{
    enum AVCodecID id = in->std == STD_AVC ? AV_CODEC_ID_H264
                                           : AV_CODEC_ID_HEVC;
    const AVCodec *codec = avcodec_find_decoder(id);

    if (!codec) {
        return false;
    }
    in->ctx = avcodec_alloc_context3(codec);
    in->ctx->apply_cropping = 0;    /* the firmware reports the crop */
    in->ctx->thread_count = 1;
    /*
     * Every DEC_PIC must give its picture back at once (the driver forces
     * a display latency of 0: FORCE_FB_LATENCY_PLUS1 = 1).
     */
    in->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
    in->ctx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
    if (avcodec_open2(in->ctx, codec, NULL) < 0) {
        avcodec_free_context(&in->ctx);
        return false;
    }
    in->parser = av_parser_init(id);
    in->pkt = av_packet_alloc();
    return true;
}

/* Collect what the decoder has output, in display order. */
static void inst_receive(VpuInst *in)
{
    while (in->nout < VPU_MAX_FB) {
        AVFrame *f = av_frame_alloc();

        if (avcodec_receive_frame(in->ctx, f) < 0) {
            av_frame_free(&f);
            return;
        }
        in->out[in->nout++] = f;
    }
}

static int inst_send(VpuInst *in, const uint8_t *data, int len, int64_t tag)
{
    int ret;

    in->pkt->data = (uint8_t *)data;
    in->pkt->size = len;
    in->pkt->pts = tag;
    in->pkt->dts = AV_NOPTS_VALUE;
    ret = avcodec_send_packet(in->ctx, len ? in->pkt : NULL);
    if (ret == AVERROR(EAGAIN)) {
        inst_receive(in);
        ret = avcodec_send_packet(in->ctx, len ? in->pkt : NULL);
    }
    inst_receive(in);
    return ret;
}

/* Copy a decoded picture into linear frame buffer @slot. */
static void inst_write_fb(VpuInst *in, int slot, AVFrame *f)
{
    VpuFb *fb = &in->lin[slot];
    uint32_t stride = in->lin_info & 0xFFFF;
    bool il = in->lin_info & PIC_INFO_INTERLEAVE;
    bool nv21 = in->lin_info & PIC_INFO_NV21;
    int w = MIN(f->width, (int)(in->lin_size >> 16));
    int h = MIN(f->height, (int)(in->lin_size & 0xFFFF));
    g_autofree uint8_t *row = g_malloc(stride + 2);
    int x, y;

    if (!stride || (f->format != AV_PIX_FMT_YUV420P &&
                    f->format != AV_PIX_FMT_YUVJ420P)) {
        return;
    }
    w = MIN(w, (int)stride);
    for (y = 0; y < h; y++) {
        vpu_dram_write(fb->y + y * stride, f->data[0] + y * f->linesize[0],
                       w);
    }
    for (y = 0; y < h / 2; y++) {
        const uint8_t *u = f->data[1] + y * f->linesize[1];
        const uint8_t *v = f->data[2] + y * f->linesize[2];

        if (il) {
            for (x = 0; x < w / 2; x++) {
                row[2 * x] = nv21 ? v[x] : u[x];
                row[2 * x + 1] = nv21 ? u[x] : v[x];
            }
            vpu_dram_write(fb->cb + y * stride, row, w & ~1);
        } else {
            vpu_dram_write(fb->cb + y * (stride / 2), u, w / 2);
            vpu_dram_write(fb->cr + y * (stride / 2), v, w / 2);
        }
    }
}

/* Scale (nearest) a decoded picture into a DEC_PIC output buffer, NV12. */
static void inst_write_out(VpuOut *o, AVFrame *f)
{
    g_autofree uint8_t *row = NULL;
    int ow = o->w, oh = o->h, x, y;
    int fw = f->width - f->crop_left - f->crop_right;
    int fh = f->height - f->crop_top - f->crop_bottom;

    if (!o->y || !o->stride || !ow || !oh || fw <= 0 || fh <= 0 ||
        (f->format != AV_PIX_FMT_YUV420P && f->format != AV_PIX_FMT_YUVJ420P)) {
        return;
    }
    ow = MIN(ow, (int)o->stride);
    row = g_malloc(ow + 2);
    for (y = 0; y < oh; y++) {
        const uint8_t *src = f->data[0] +
            (f->crop_top + y * fh / oh) * f->linesize[0] + f->crop_left;

        for (x = 0; x < ow; x++) {
            row[x] = src[x * fw / ow];
        }
        vpu_dram_write(o->y + y * o->stride, row, ow);
    }
    for (y = 0; y < oh / 2; y++) {
        int sy = (f->crop_top / 2) + y * (fh / 2) / (oh / 2);
        const uint8_t *u = f->data[1] + sy * f->linesize[1] + f->crop_left / 2;
        const uint8_t *v = f->data[2] + sy * f->linesize[2] + f->crop_left / 2;

        for (x = 0; x < ow / 2; x++) {
            int sx = x * (fw / 2) / (ow / 2);

            row[2 * x] = u[sx];
            row[2 * x + 1] = v[sx];
        }
        vpu_dram_write(o->c + y * o->stride, row, ow & ~1);
    }
}

/* Black, for a DEC_PIC that comes before any picture. */
static void inst_blank_out(VpuOut *o)
{
    uint32_t w = MIN(o->w, o->stride), y;
    g_autofree uint8_t *row = g_malloc(w);

    memset(row, 16, w);
    for (y = 0; y < o->h; y++) {
        vpu_dram_write(o->y + y * o->stride, row, w);
    }
    memset(row, 128, w);
    for (y = 0; y < o->h / 2; y++) {
        vpu_dram_write(o->c + y * o->stride, row, w);
    }
}

static uint32_t pic_type(AVFrame *f, uint32_t std)
{
    uint32_t t;

    switch (f->pict_type) {
    case AV_PICTURE_TYPE_B:
        t = 1u << 2;
        break;
    case AV_PICTURE_TYPE_P:
        t = 1u << 1;
        break;
    default:
        t = 1u << 0;
        break;
    }
    if (f->flags & AV_FRAME_FLAG_KEY) {
        t |= (std == STD_AVC ? 5 : 19) << 4;    /* IDR nal_unit_type */
    }
    return t;
}

/* Pick the oldest decoded picture to display, or IDX_NOT_YET. */
static int inst_display(VpuInst *in, VpuReport *r, VpuOut *o)
{
    AVFrame *f;
    int slot;

    if (!in->nout) {
        /*
         * Nothing new: show the last picture again.  A DEC_PIC that gives
         * no picture back leaves the driver's output task waiting until
         * it times out (10 s) and resets the decoder.
         */
        if (!o || !o->y) {
            return IDX_NOT_YET;
        }
        if (!in->last) {
            inst_blank_out(o);          /* nothing decoded yet */
            return 0;
        }
        inst_write_out(o, in->last);
        return in->last_slot;
    }
    f = in->out[0];
    memmove(in->out, in->out + 1, --in->nout * sizeof(in->out[0]));
    slot = f->pts;
    if (slot < 0 || slot >= VPU_MAX_FB) {
        av_frame_free(&f);
        return IDX_NOT_YET;
    }
    if (o && o->y) {
        inst_write_out(o, f);
    } else {
        inst_write_fb(in, slot, f);
        in->disp_flags |= 1u << slot;   /* the host owns it until cleared */
    }
    rep_set(r, W5_RET_DEC_PIC_TYPE, pic_type(f, in->std));
    rep_set(r, W5_RET_DEC_PIC_POC, in->pics++);
    av_frame_free(&in->last);
    in->last = f;
    in->last_slot = slot;
    in->slot_busy &= ~(1u << slot);
    return slot;
}

/*
 * Frame buffer indices: the linear buffers when the host registered them
 * with addresses, otherwise (outputs given with each DEC_PIC) the
 * compressed ones.
 */
static int inst_free_slot(VpuInst *in)
{
    uint32_t used = in->disp_flags | in->slot_busy;
    int n = in->nlin && in->lin[0].y ? in->nlin : in->nfbc;
    int i;

    for (i = 0; i < n; i++) {
        if (!(used & (1u << i))) {
            return i;
        }
    }
    return -1;
}

/*
 * Decode one access unit into a free frame buffer index (the tag the
 * picture comes back with).  Pictures beyond what the DEC_PICs can show
 * are dropped, oldest first: the host shows one per command.
 */
static void inst_decode_au(VpuInst *in, const uint8_t *au, int len,
                           int *decoded)
{
    int slot = inst_free_slot(in);
    int before = in->nout;

    while (slot < 0 && in->nout) {
        AVFrame *f = in->out[0];

        if (f->pts >= 0 && f->pts < VPU_MAX_FB) {
            in->slot_busy &= ~(1u << f->pts);
        }
        av_frame_free(&f);
        memmove(in->out, in->out + 1, --in->nout * sizeof(in->out[0]));
        before = in->nout;
        slot = inst_free_slot(in);
    }
    if (slot < 0) {
        return;
    }
    in->slot_busy |= 1u << slot;
    inst_send(in, au, len, slot);
    if (in->nout == before) {
        /* no picture (yet, or an error): the index is free again */
        in->slot_busy &= ~(1u << slot);
    } else {
        *decoded = slot;
    }
}

/*
 * INIT_SEQ: decode what the ring holds until the stream parameters are
 * known.  The bitstream is not consumed: DEC_PIC decodes it again.
 */
static void inst_init_seq(SstarVpuState *s, VpuInst *in, VpuCmd *c,
                          VpuReport *r)
{
    uint32_t len = bs_avail(in, in->rd, c->bs_wr);
    g_autofree uint8_t *buf = len ? bs_fetch(in, in->rd, len) : NULL;
    AVCodecContext *ctx;
    int nref;

    if (!in->ctx && !inst_open_codec(in)) {
        goto fail;
    }
    ctx = in->ctx;
    if (len) {
        inst_send(in, buf, len, -1);
    }
    if (!ctx->coded_width || !ctx->coded_height) {
        in->rd = c->bs_wr;              /* all consumed: wait for more */
        goto fail;
    }
    avcodec_flush_buffers(ctx);
    inst_drop_frames(in);
    av_parser_close(in->parser);
    in->parser = av_parser_init(ctx->codec_id);
    in->pic_w = ctx->coded_width;
    in->pic_h = ctx->coded_height;
    in->crop_l = in->crop_t = 0;
    in->crop_r = ctx->coded_width - MIN(ctx->width, ctx->coded_width);
    in->crop_b = ctx->coded_height - MIN(ctx->height, ctx->coded_height);
    nref = MAX(ctx->refs, 1) + ctx->has_b_frames + 2;
    in->num_fb = MIN(MAX(nref, 3), VPU_MAX_FB - 1);

    rep_set(r, W5_RET_DEC_DECODING_SUCCESS, 1);
    rep_set(r, W5_RET_DEC_ERR_INFO, 0);
    rep_set(r, W5_RET_DEC_WARN_INFO, 0);
    rep_set(r, W5_RET_DEC_PIC_SIZE, (in->pic_w << 16) | in->pic_h);
    rep_set(r, W5_RET_DEC_CROP_LEFT_RIGHT, (in->crop_l << 16) | in->crop_r);
    rep_set(r, W5_RET_DEC_CROP_TOP_BOTTOM, (in->crop_t << 16) | in->crop_b);
    rep_set(r, W5_RET_DEC_NUM_REQUIRED_FB, in->num_fb);
    rep_set(r, W5_RET_DEC_NUM_REORDER, ctx->has_b_frames);
    rep_set(r, W5_RET_DEC_COLOR_SAMPLE, (1u << 8) | (8 << 4) | 8);
    rep_set(r, W5_RET_DEC_SEQ_PARAM,
            ((ctx->profile & 0x7F) << 24) | (ctx->level & 0xFF));
    rep_set(r, W5_RET_DEC_SUB_LAYER_INFO, 0);
    rep_set(r, W5_RET_DEC_NOTIFICATION, 0);
    rep_set(r, W5_RET_DEC_USERDATA_IDC, 0);
    rep_set(r, W5_RET_VLC_BUF_SIZE, 0x100000);
    rep_set(r, W5_RET_PARAM_BUF_SIZE, 0x10000);
    rep_set(r, W5_RET_DEC_DISP_IDC, in->disp_flags);
    if (s->trace) {
        qemu_log("sstar-vpu: seq %ux%u crop r%u b%u fb %u\n", in->pic_w,
                 in->pic_h, in->crop_r, in->crop_b, in->num_fb);
    }
    return;
fail:
    rep_set(r, W5_RET_DEC_DECODING_SUCCESS, 0);
    rep_set(r, W5_RET_DEC_ERR_INFO, ERR_SEQ_NOT_FOUND);
}

/* DEC_PIC: decode one access unit, display at most one picture. */
static void inst_dec_pic(SstarVpuState *s, VpuInst *in, VpuCmd *c,
                         VpuReport *r)
{
    uint32_t len = bs_avail(in, in->rd, c->bs_wr);
    uint32_t au_start = in->rd;
    int decoded = IDX_NONE, display;
    hwaddr off;

    if (!in->ctx) {
        decoded = IDX_NONE;
    } else if (len) {
        g_autofree uint8_t *buf = bs_fetch(in, in->rd, len);
        uint32_t pos = 0;

        /*
         * The host's chunks are not pictures: one may end inside a picture
         * or hold the end of one and the start of the next, explicit end
         * flag or not.  Find the access units the way the firmware does,
         * by parsing (a picture is complete once the next one starts), and
         * decode each.
         */
        while (pos < len) {
            uint8_t *au;
            int aulen;
            int used = av_parser_parse2(in->parser, in->ctx, &au, &aulen,
                                        buf + pos, len - pos, AV_NOPTS_VALUE,
                                        AV_NOPTS_VALUE, 0);

            pos += used;
            if (aulen > 0) {
                inst_decode_au(in, au, aulen, &decoded);
            } else if (!used) {
                break;
            }
        }
        if (c->bs_opt & BSOPT_STREAM_END) {
            uint8_t *au;
            int aulen;

            av_parser_parse2(in->parser, in->ctx, &au, &aulen, NULL, 0,
                             AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (aulen > 0) {
                inst_decode_au(in, au, aulen, &decoded);
            }
        }
        in->rd = bs_advance(in, in->rd, len);
    } else if ((c->bs_opt & BSOPT_STREAM_END) && !in->drained) {
        inst_send(in, NULL, 0, -1);     /* drain */
        in->drained = true;
    }
    display = inst_display(in, r, &c->out);
    if (display == IDX_NOT_YET && in->drained && !in->nout) {
        display = IDX_NO_FB;            /* -1: the end of the sequence */
    }
    rep_set(r, W5_RET_DEC_DECODED_INDEX, decoded);
    rep_set(r, W5_RET_DEC_DISPLAY_INDEX, display);
    if (!(r->set & (1ull << ((W5_RET_DEC_PIC_TYPE - W5_COMMAND) >> 2)))) {
        rep_set(r, W5_RET_DEC_PIC_TYPE, 0);
        rep_set(r, W5_RET_DEC_PIC_POC, 0);
    }
    rep_set(r, W5_RET_DEC_DECODING_SUCCESS, 1);
    rep_set(r, W5_RET_DEC_ERR_INFO, 0);
    rep_set(r, W5_RET_DEC_WARN_INFO, 0);
    rep_set(r, W5_RET_DEC_NOTIFICATION, 0);
    rep_set(r, W5_RET_DEC_SUB_LAYER_INFO, 0);
    rep_set(r, W5_RET_DEC_USERDATA_IDC, 0);
    rep_set(r, W5_RET_DEC_PIC_SIZE, (in->pic_w << 16) | in->pic_h);
    rep_set(r, W5_RET_DEC_CROP_LEFT_RIGHT, (in->crop_l << 16) | in->crop_r);
    rep_set(r, W5_RET_DEC_CROP_TOP_BOTTOM, (in->crop_t << 16) | in->crop_b);
    rep_set(r, W5_RET_DEC_AU_START_POS, au_start);
    rep_set(r, W5_RET_DEC_AU_END_POS, in->rd);
    rep_set(r, W5_RET_DEC_ERR_CTB_NUM, 0);
    /*
     * Also read by the SigmaStar driver: the recovery point (0x168) and
     * cycle / statistics words (0x188..0x1A0, 0x1BC..0x1CC); none apply.
     */
    rep_set(r, 0x168, 0);
    for (off = 0x188; off <= 0x198; off += 4) {
        rep_set(r, off, 0);
    }
    /* where the picture was put: the host matches it to the output task */
    if (display >= 0) {
        rep_set(r, W5_RET_DEC_OUT_LUMA, c->out.y);
        rep_set(r, W5_RET_DEC_OUT_CHROMA, c->out.c);
    }
    /*
     * The VLC buffer the picture went through: the host finds the output
     * task by it (its VLC buffer table holds what DEC_PIC passed in 0x168).
     */
    rep_set(r, W5_RET_DEC_VLC_ADDR, c->vlc_addr);
    rep_set(r, W5_RET_DEC_VLC_USED, 0);
    for (off = 0x1BC; off <= 0x1CC; off += 4) {
        rep_set(r, off, 0);
    }
    rep_set(r, W5_RET_DEC_HOST_CMD_TICK, s->tick);
    rep_set(r, W5_RET_DEC_ENC_TICK, s->tick + 1);
    rep_set(r, W5_RET_DEC_DISP_IDC, in->disp_flags);
    s->tick += 2;
    if (s->trace) {
        qemu_log("sstar-vpu: dec len %u decoded %d display %d\n", len,
                 decoded, display);
    }
}

#else

static void inst_init_seq(SstarVpuState *s, VpuInst *in, VpuCmd *c,
                          VpuReport *r)
{
    in->rd = c->bs_wr;
    rep_set(r, W5_RET_DEC_DECODING_SUCCESS, 0);
    rep_set(r, W5_RET_DEC_ERR_INFO, ERR_SEQ_NOT_FOUND);
}

static void inst_dec_pic(SstarVpuState *s, VpuInst *in, VpuCmd *c,
                         VpuReport *r)
{
    in->rd = c->bs_wr;
    rep_set(r, W5_RET_DEC_DECODED_INDEX, IDX_NONE);
    rep_set(r, W5_RET_DEC_DISPLAY_INDEX, IDX_NOT_YET);
    rep_set(r, W5_RET_DEC_DECODING_SUCCESS, 1);
}

static void inst_free_codec(VpuInst *in)
{
}

#endif /* SSTAR_VPU_LIBAVCODEC */

/* ---- the firmware's command processing ---- */

static VpuInst *vpu_cur_inst(SstarVpuState *s)
{
    return &s->inst[(vpu_reg(s, W5_CMD_INSTANCE_INFO) & 0xFFFF) %
                    VPU_MAX_INST];
}

static void inst_destroy(VpuInst *in)
{
    inst_free_codec(in);
    memset(in, 0, sizeof(*in));
}

static void vpu_create_instance(SstarVpuState *s)
{
    uint32_t info = vpu_reg(s, W5_CMD_INSTANCE_INFO);
    VpuInst *in = vpu_cur_inst(s);
    uint32_t std = info >> 16;

    if (std != STD_HEVC && std != STD_AVC) {
        vpu_cmd_done(s, false, SYSERR_UNKNOWN_CODEC_STD);
        return;
    }
    inst_destroy(in);
    in->used = true;
    in->std = std;
    in->bs_start = vpu_reg(s, W5_CMD_DEC_BS_START_ADDR);
    in->bs_size = vpu_reg(s, W5_CMD_DEC_BS_SIZE);
    in->rd = in->wr = in->bs_start;
    if (!s->migration_blocker) {
        error_setg(&s->migration_blocker, "sstar-vpu: decoder instances "
                   "hold host decoder state");
        migrate_add_blocker(&s->migration_blocker, NULL);
    }
    vpu_queue_status(s, in);
    vpu_cmd_done(s, true, 0);
}

/* INIT_SEQ / DEC_PIC: queued, run from the timer, completed by IRQ. */
static void vpu_queue_cmd(SstarVpuState *s, uint32_t cmd)
{
    VpuInst *in = vpu_cur_inst(s);
    VpuCmd *c;

    if (!in->used || in->nqueue == VPU_QUEUE_DEPTH ||
        in->nqueue + in->nreport > VPU_QUEUE_DEPTH) {
        vpu_queue_status(s, in);
        vpu_cmd_done(s, false, SYSERR_QUEUEING_FAIL);
        vpu_set_reg(s, W5_CMD_INSTANCE_INFO, SYSERR_QUEUEING_FAIL);
        return;
    }
    if (cmd == W5_INIT_SEQ) {
        in->rd = vpu_reg(s, W5_BS_RD_PTR);
    }
    c = &in->queue[in->nqueue++];
    c->cmd = cmd;
    c->bs_wr = vpu_reg(s, W5_BS_WR_PTR);
    c->bs_opt = vpu_reg(s, W5_BS_OPTION);
    c->opt = vpu_reg(s, W5_COMMAND_OPTION);
    if (cmd == W5_DEC_PIC) {
        c->out.flags = vpu_reg(s, W5_CMD_DEC_OUT_FLAGS);
        c->out.w = vpu_reg(s, W5_CMD_DEC_OUT_SIZE) >> 16;
        c->out.h = vpu_reg(s, W5_CMD_DEC_OUT_SIZE) & 0xFFFF;
        c->out.y = vpu_reg(s, W5_CMD_DEC_OUT_LUMA);
        c->out.c = vpu_reg(s, W5_CMD_DEC_OUT_CHROMA);
        c->out.stride = vpu_reg(s, W5_CMD_DEC_OUT_STRIDE);
        c->vlc_addr = vpu_reg(s, W5_CMD_DEC_VLC_ADDR);
    }
    in->wr = c->bs_wr;
    in->stream_end = c->bs_opt & BSOPT_STREAM_END;
    vpu_queue_status(s, in);
    vpu_cmd_done(s, true, 0);
    /*
     * Not at once: the SigmaStar driver arms its wait for the completion
     * after issuing the command, and a picture takes the silicon
     * milliseconds anyway.
     */
    if (!timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) +
                  s->latency_us);
    }
}

static void vpu_run_queue(void *opaque)
{
    SstarVpuState *s = opaque;
    int i;

    for (i = 0; i < VPU_MAX_INST; i++) {
        VpuInst *in = &s->inst[i];

        while (in->used && in->nqueue) {
            VpuCmd c = in->queue[0];
            VpuReport *r = &in->report[in->nreport++];

            memmove(in->queue, in->queue + 1,
                    --in->nqueue * sizeof(in->queue[0]));
            memset(r, 0, sizeof(*r));
            /* a later UPDATE_BS may have added data */
            if (bs_avail(in, in->rd, in->wr) > bs_avail(in, in->rd, c.bs_wr)) {
                c.bs_wr = in->wr;
            }
            if (in->stream_end) {
                c.bs_opt |= BSOPT_STREAM_END;
            }
            if (c.cmd == W5_INIT_SEQ) {
                inst_init_seq(s, in, &c, r);
                /*
                 * The SigmaStar driver's ISR takes INIT_SEQ completions
                 * from QUEUE_CMD_DONE_INST too (mainline: WAVE515 only).
                 */
                s->regs[W5_RET_QUEUE_CMD_DONE_INST >> 2] |= 1u << i;
                s->regs[W5_VPU_VINT_REASON >> 2] |= 1u << INT_INIT_SEQ;
                s->regs[W5_VPU_VINT_REASON_USR >> 2] |= 1u << INT_INIT_SEQ;
            } else {
                inst_dec_pic(s, in, &c, r);
                /* the SigmaStar ISR also takes the instance index in 0x1FC */
                s->regs[W5_RET_QUEUE_CMD_DONE_INST >> 2] |= 1u << i;
                s->regs[W5_RET_DONE_INST_IDX >> 2] = i;
                s->regs[W5_VPU_VINT_REASON >> 2] |= 1u << INT_DEC_PIC;
                s->regs[W5_VPU_VINT_REASON_USR >> 2] |= 1u << INT_DEC_PIC;
            }
        }
    }
    vpu_update_irq(s);
}

static void vpu_get_result(SstarVpuState *s, VpuInst *in)
{
    VpuReport *r = &in->report[0];
    int i;

    if (!in->nreport) {
        vpu_queue_status(s, in);
        vpu_cmd_done(s, false, SYSERR_RESULT_NOT_READY);
        return;
    }
    for (i = 0; i < VPU_REPORT_WORDS; i++) {
        if (r->set & (1ull << i)) {
            s->regs[(W5_COMMAND >> 2) + i] = r->w[i];
        }
    }
    memmove(in->report, in->report + 1, --in->nreport * sizeof(*r));
    vpu_queue_status(s, in);
    vpu_cmd_done(s, true, 0);
}

/* SET_FB: one chunk of up to eight buffers of a group. */
static void vpu_set_fb(SstarVpuState *s)
{
    VpuInst *in = vpu_cur_inst(s);
    uint32_t info = vpu_reg(s, W5_COMMON_PIC_INFO);
    uint32_t num = vpu_reg(s, W5_SET_FB_NUM);
    uint32_t first = (num >> 8) & 0xFF, last = num & 0xFF, k;

    if (info & PIC_INFO_LINEAR) {
        if (vpu_reg(s, W5_COMMAND_OPTION) & SFB_FIRST) {
            memset(in->lin, 0, sizeof(in->lin));
            in->nlin = 0;
            in->lin_info = info;
            in->lin_size = vpu_reg(s, W5_PIC_SIZE);
        }
        for (k = first; k <= last && k < VPU_MAX_FB; k++) {
            hwaddr base = W5_ADDR_LUMA_BASE0 + (k - first) * 16;

            in->lin[k].y = vpu_reg(s, base);
            in->lin[k].cb = vpu_reg(s, base + 4);
            in->lin[k].cr = vpu_reg(s, base + 8);
            in->nlin = MAX(in->nlin, k + 1);
        }
        if (s->trace) {
            qemu_log("sstar-vpu: linear fb %u..%u info %08x size %08x\n",
                     first, last, info, in->lin_size);
        }
    } else {
        if (vpu_reg(s, W5_COMMAND_OPTION) & SFB_FIRST) {
            in->nfbc = 0;
        }
        in->nfbc = MIN(MAX(in->nfbc, last + 1), VPU_MAX_FB);
    }
    vpu_cmd_done(s, true, 0);
}

static void vpu_flush_instance(SstarVpuState *s, VpuInst *in)
{
    in->nqueue = in->nreport = 0;
    in->slot_busy = 0;
#ifdef SSTAR_VPU_LIBAVCODEC
    inst_drop_frames(in);
    if (in->ctx) {
        avcodec_flush_buffers(in->ctx);
        av_parser_close(in->parser);
        in->parser = av_parser_init(in->ctx->codec_id);
    }
    in->drained = false;
#endif
    in->wr = in->rd;
    vpu_queue_status(s, in);
    vpu_cmd_done(s, true, 0);
}

static void vpu_query(SstarVpuState *s)
{
    VpuInst *in = vpu_cur_inst(s);

    switch (vpu_reg(s, W5_COMMAND_OPTION)) {
    case GET_VPU_INFO:
        vpu_set_reg(s, W5_RET_FW_VERSION, 0x00030000);
        vpu_set_reg(s, W5_RET_PRODUCT_NAME, 0x57353131);    /* "W511" */
        vpu_set_reg(s, W5_RET_PRODUCT_VERSION, 0);
        vpu_set_reg(s, W5_RET_STD_DEF0, 0);
        vpu_set_reg(s, W5_RET_STD_DEF1, (1u << 3) | (1u << 2)); /* AVC|HEVC */
        vpu_set_reg(s, W5_RET_CONF_FEATURE, 0);
        vpu_set_reg(s, W5_RET_CONF_DATE, 20190101);
        vpu_set_reg(s, W5_RET_CONF_REVISION, 200000);
        vpu_set_reg(s, W5_RET_CONF_TYPE, 0);
        vpu_cmd_done(s, true, 0);
        return;
    case QUERY_GET_RESULT:
        vpu_get_result(s, in);
        return;
    case QUERY_UPDATE_DISP_FLAG:
        in->disp_flags = (in->disp_flags & ~vpu_reg(s, W5_CMD_DEC_CLR_DISP_IDC))
                         | vpu_reg(s, W5_CMD_DEC_SET_DISP_IDC);
        vpu_set_reg(s, W5_RET_DEC_DISP_IDC, in->disp_flags);
        break;
    case QUERY_GET_BS_RD_PTR:
        vpu_set_reg(s, W5_RET_QUERY_DEC_BS_RD_PTR, in->rd);
        break;
    case QUERY_SET_BS_RD_PTR:
        in->rd = vpu_reg(s, W5_RET_QUERY_DEC_BS_RD_PTR);
        break;
    case QUERY_SET_WRITE_PROT:
    case QUERY_GET_BW_REPORT:
    case QUERY_GET_DEBUG_INFO:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sstar-vpu: query option %u\n",
                      vpu_reg(s, W5_COMMAND_OPTION));
        vpu_cmd_done(s, false, SYSERR_UNKNOWN_QUERY);
        return;
    }
    vpu_cmd_done(s, true, 0);
}

/* HOST_INT_REQ: the firmware takes the command in W5_COMMAND. */
static void vpu_command(SstarVpuState *s)
{
    uint32_t cmd = vpu_reg(s, W5_COMMAND);
    VpuInst *in = vpu_cur_inst(s);

    if (s->trace) {
        qemu_log("sstar-vpu: command %04x opt %x inst %08x\n", cmd,
                 vpu_reg(s, W5_COMMAND_OPTION),
                 vpu_reg(s, W5_CMD_INSTANCE_INFO));
    }
    switch (cmd) {
    case W5_QUERY:
        vpu_query(s);
        break;
    case W5_CREATE_INSTANCE:
        vpu_create_instance(s);
        break;
    case W5_DESTROY_INSTANCE:
        inst_destroy(in);
        vpu_cmd_done(s, true, 0);
        break;
    case W5_FLUSH_INSTANCE:
        vpu_flush_instance(s, in);
        break;
    case W5_INIT_SEQ:
    case W5_DEC_PIC:
        vpu_queue_cmd(s, cmd);
        break;
    case W5_SET_FB:
        vpu_set_fb(s);
        break;
    case W5_UPDATE_VLC:
        /* a new VLC buffer (0x11C, 0x120): the emulation needs none */
        vpu_cmd_done(s, true, 0);
        break;
    case W5_UPDATE_BS:
        in->wr = vpu_reg(s, W5_BS_WR_PTR);
        in->stream_end = vpu_reg(s, W5_BS_OPTION) & BSOPT_STREAM_END;
        vpu_cmd_done(s, true, 0);
        break;
    case W5_SLEEP_VPU:
        vpu_cmd_done(s, true, 0);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sstar-vpu: command 0x%x\n", cmd);
        vpu_cmd_done(s, false, SYSERR_UNKNOWN_CMD);
        break;
    }
}

/* REMAP_CORE_START: the VCPU boots (INIT_VPU) or resumes (WAKEUP_VPU). */
static void vpu_core_start(SstarVpuState *s)
{
    uint32_t cmd = vpu_reg(s, W5_COMMAND);

    if (cmd == W5_INIT_VPU || cmd == W5_WAKEUP_VPU) {
        vpu_set_reg(s, W5_VCPU_CUR_PC, VPU_FW_PC);
        vpu_cmd_done(s, true, 0);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-vpu: core start, command "
                      "0x%x\n", cmd);
    }
}

static uint32_t vpu_fio_read(SstarVpuState *s, uint32_t addr)
{
    switch (addr) {
    case 0x8E14:                /* BUS_STATUS_VCORE0 */
    case 0x9E14:                /* BUS_STATUS_VCORE1 */
    case 0xFE14:                /* BUS_STATUS_VCPU / combined */
        return GDI_BUS_IDLE;
    default:
        return s->fio[(addr & 0xFFFF) >> 2];
    }
}

static void vpu_fio_ctrl(SstarVpuState *s, uint32_t ctrl)
{
    uint32_t addr = ctrl & 0xFFFF;

    if (ctrl & FIO_CTRL_WRITE) {
        s->fio[addr >> 2] = s->regs[W5_VPU_FIO_DATA >> 2];
    } else {
        s->regs[W5_VPU_FIO_DATA >> 2] = vpu_fio_read(s, addr);
    }
    s->regs[W5_VPU_FIO_CTRL_ADDR >> 2] = ctrl | FIO_CTRL_READY;
}

static uint64_t sstar_vpu_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVpuState *s = opaque;
    uint32_t v = s->regs[off >> 2];

    if (off == W5_PRODUCT_NUMBER) {
        v = WAVE511_CODE;
    }

    /* 0xF0/0xF4: the firmware console mailbox, which a host thread polls */
    if (s->trace && off != W5_VPU_DBG_REG0) {
        qemu_log("sstar-vpu: R %04" HWADDR_PRIx " = %08x\n", off, v);
    }
    return v;
}

static void sstar_vpu_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    SstarVpuState *s = opaque;

    if (s->trace && off != W5_VPU_DBG_REG0) {
        qemu_log("sstar-vpu: W %04" HWADDR_PRIx " = %08" PRIx64 "\n", off,
                 val);
    }
    switch (off) {
    case W5_VPU_FIO_CTRL_ADDR:
        vpu_fio_ctrl(s, val);
        return;
    case W5_VPU_HOST_INT_REQ:
        if (val & 1) {
            vpu_command(s);
        }
        return;
    case W5_VPU_REMAP_CORE_START:
        s->regs[off >> 2] = val;
        if (val & 1) {
            vpu_core_start(s);
        }
        return;
    case W5_VPU_VINT_REASON_CLR:
        s->regs[W5_VPU_VINT_REASON >> 2] &= ~val;
        vpu_update_irq(s);
        return;
    case W5_VPU_VINT_CLEAR:
        if (val & 1) {
            /* re-asserted below if enabled reasons are still pending */
            vpu_update_irq(s);
        }
        return;
    case W5_VPU_VINT_ENABLE:
        s->regs[off >> 2] = val;
        vpu_update_irq(s);
        return;
    case W5_VPU_RESET_REQ:
        s->regs[off >> 2] = val;
        if (val & 0x3F000000) {     /* the VCPU: the firmware stops */
            int i;

            for (i = 0; i < VPU_MAX_INST; i++) {
                inst_destroy(&s->inst[i]);
            }
            s->regs[W5_VCPU_CUR_PC >> 2] = 0;
            s->regs[W5_VPU_VINT_REASON >> 2] = 0;
            vpu_update_irq(s);
        }
        return;
    case W5_VPU_RESET_STATUS:
    case W5_VPU_VPU_INT_STS:
        return;                     /* read-only */
    default:
        s->regs[off >> 2] = val;
        return;
    }
}

static const MemoryRegionOps sstar_vpu_ops = {
    .read = sstar_vpu_read,
    .write = sstar_vpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sstar_vpu_reset(DeviceState *dev)
{
    SstarVpuState *s = SSTAR_VPU(dev);
    int i;

    timer_del(s->timer);
    for (i = 0; i < VPU_MAX_INST; i++) {
        inst_destroy(&s->inst[i]);
    }
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->fio, 0, sizeof(s->fio));
    qemu_set_irq(s->irq, 0);
}

static void sstar_vpu_init(Object *obj)
{
    SstarVpuState *s = SSTAR_VPU(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_vpu_ops, s, "sstar-vpu",
                          VPU_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_us(QEMU_CLOCK_VIRTUAL, vpu_run_queue, s);
}

/*
 * The registers only: open decoder instances hold host decoder state and
 * block migration (vpu_create_instance).
 */
static const VMStateDescription vmstate_sstar_vpu = {
    .name = TYPE_SSTAR_VPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SstarVpuState, VPU_WINDOW / 4),
        VMSTATE_UINT32_ARRAY(fio, SstarVpuState, VPU_WINDOW / 4),
        VMSTATE_UINT32(tick, SstarVpuState),
        VMSTATE_TIMER_PTR(timer, SstarVpuState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sstar_vpu_props[] = {
    DEFINE_PROP_BOOL("trace", SstarVpuState, trace, false),
    DEFINE_PROP_UINT32("latency-us", SstarVpuState, latency_us, 4000),
};

static void sstar_vpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_vpu_reset);
    device_class_set_props(dc, sstar_vpu_props);
    dc->vmsd = &vmstate_sstar_vpu;
}

static const TypeInfo sstar_vpu_info = {
    .name           = TYPE_SSTAR_VPU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarVpuState),
    .instance_init  = sstar_vpu_init,
    .class_init     = sstar_vpu_class_init,
};

static void sstar_vpu_register_types(void)
{
    type_register_static(&sstar_vpu_info);
}

type_init(sstar_vpu_register_types)
