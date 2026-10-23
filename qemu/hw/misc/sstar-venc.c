/*
 * SigmaStar Ifado video encoder (VENC), at its firmware interface.
 *
 * The encoder is a RISC-V core running firmware that sstar_mi.ko carries
 * and loads, in front of the encoding hardware.  The driver reaches it
 * through 32-bit registers in two blocks (sstar_mi.ko HalVencIos*,
 * HalVencIsrHandlerPartA; the DTB's "vpu-bit" at 0x1F34D000 is the third
 * 1 KiB of this window):
 *
 *   0x1F34C800  RISC control ("ctrl"): +0x80 control, +0x78 firmware
 *               state [3:0], +0x88 status, +0x134 boot handshake, the
 *               firmware's boot addresses at +0xB0..+0xDC
 *   0x1F34CC00  encoder ("enc"): +0x60 interrupt status, +0x64 its clear,
 *               +0x80..+0x94 the firmware image's base and size
 *
 * Boot (HalVencIosFirmWareInit): the driver waits for ctrl+0x134 bit 4,
 * sets its bit 0, then waits for ctrl+0x80 bit 1 with ctrl+0x78 [3:0] = 2,
 * the firmware running.  Here bit 4 is set from reset and the firmware
 * runs as soon as it is started.
 *
 * Jobs (HalVencIosFire): eight mailbox slots at ctrl+0x00, +0x0C ..
 * +0x54, three words each.  The driver picks the next slot whose word 0
 * bit 0 (busy) is clear, writes word 2 (the job's register program, an
 * MIU address in 16-byte units: (value, register offset) pairs for the
 * encoder hardware), word 1, then word 0 with bit 0 set.  The firmware
 * takes the job and, when the frame is encoded, writes the result block
 * (the program's register 0x1088, same units) and raises an interrupt
 * (HalVencIsrHandlerPartA, _VencWrapperIsrProcess) in enc+0x60, which
 * enc+0x64 clears (the bits are below); ctrl+0x88 [13:8] is the
 * channel, ctrl+0x210 the bitstream's size in bits, enc+0x258 and
 * ctrl+0x208 its start and end (16-byte units).
 * Slot word 0 [29:24] is the job's encoder context (one per stream),
 * which the completion reports in ctrl+0x88 [13:8].  The status block
 * (HalMfeOpsEncDone, checked by HalVencDpbDoneFrame) holds the last
 * 16-byte units written, each of which must lie in the buffer the
 * program gives: words 0x48 and 0x4C the reconstructed luma and chroma
 * (registers 0x170..0x17C and 0x18C..0x198), 0x34 and 0x38 two side
 * planes (0x150..0x15C, 0x160..0x16C), 0x2C the bitstream (0x1118 ..
 * 0x111C).  The program's end registers are inclusive.  Here the planes
 * are reported full.  Word 0x44 is taken for the frame's size in bits.
 *
 * To reconfigure (HalVencIosFirmWareDeInit) the driver writes enc+0x248
 * bit 0, waits for the firmware state ctrl+0x78 [3:0] = 8 (stopped),
 * resets the core through ctrl+0x80 and boots the firmware again.
 *
 * The driver then polls the status block (the program's register
 * 0x10BC; HalVencOpsCheckStatus): byte 0x23 set means done, with byte
 * 0x18 [3:0] the outcome: 7 an error (HalVencOpsEncDone warns "src" and
 * the driver resets the encoder), 5 a slice, others a frame.
 *
 * What a job encodes, from its program: register 0xA0 the picture's
 * height << 16 | width, 0xBC bit 8 an I frame (the driver keeps the GOP),
 * 0x1080 bit 1 H.265 (the MHE engine) rather than H.264.  The bitstream
 * goes into the ring of registers 0x1118..0x111C, from the write pointer
 * 0x1128; the hardware writes whole access units, headers included (the
 * driver writes nothing into the ring itself).  The source picture
 * (0x1258/0x125C) is a ring the scaler fills a few lines at a time, so
 * it is never whole in memory.
 *
 * The stream's destination is the output buffer the driver hands the
 * firmware after each frame (HalVencIosAddOutBuffer: ctrl+0x208 its
 * start and ctrl+0x20C its size, both in 16-byte units, the size less
 * one, then ctrl+0x204 = 1), for the context it was just told of
 * (ctrl+0x88): a record of the channel's MI ring pool.  The frame goes
 * there too, and the completion reports it in enc+0x258 and ctrl+0x208
 * (start and end) and ctrl+0x210 (bits).  The frame done is enc+0x60 bit
 * 0, MI's FRM_DONE; the ISR turns bit 5 into a buffer ready under run,
 * bit 6 into a slice done and bit 1 into a ring drop (the Ifliegen SDK's
 * E_MI_VENC_IMPL_* names), none of which hands the stream to MI_VENC_
 * GetStream.  The ISR clears the status and then uses the shared
 * registers (the done's end, the next output buffer), so the next done
 * is reported a little later.
 *
 * Here a job completes "done-us" (by default 30 ms, under a 25 fps
 * frame) after it is fired, each on its own deadline.  Built with
 * libavcodec (and its libx264 and libx265 encoders), the model encodes a
 * test pattern of its own (colour bars, a moving box and the frame number
 * in binary) at the job's size and codec, an IDR where the driver asks
 * for an I frame.  Each frame is encoded on a host thread of its own from
 * the moment the job is fired, while the guest runs, at a lower priority
 * than the vCPU (with the x265 pool it opens), so that a host the
 * encoders saturate slows the streams rather than the guest: a starved
 * vCPU, the guest clock running on, met the 60 s boot watchdog U-Boot
 * arms on a 2.4 GHz Xeon.  A frame that is not
 * ready at its deadline is waited for with the virtual clock held: a
 * guest that ran on past it would queue the channel's next frame onto
 * the ring-pool record of the one in flight.  libx265 takes 20-35 ms for
 * a 2560x1440 frame on its thread pool; encoded with the clock held
 * throughout, as before, the guest ran at half the wall clock's speed,
 * too slow for its RTSP and web clients.  "dump" names a host file prefix
 * that each stream's elementary stream is appended to (PREFIX-ctxN.h264
 * or .h265).  Without libavcodec, or when the encoder fails, the frame is
 * an access unit delimiter.
 *
 * "trace" logs every access.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/cpu-timers.h"
#include "system/dma.h"
#include "system/runstate.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#ifdef SSTAR_VENC_LIBAVCODEC
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#ifdef CONFIG_LINUX
#include <sys/resource.h>
#endif
#endif

#define TYPE_SSTAR_VENC "sstar-venc"
OBJECT_DECLARE_SIMPLE_TYPE(SstarVencState, SSTAR_VENC)

#define VENC_SIZE           0x800
#define CTRL                0x000       /* 0x1F34C800 */
#define ENC                 0x400       /* 0x1F34CC00 */

#define CTRL_CONTROL        (CTRL + 0x80)
#define CTRL_FW_STATE       (CTRL + 0x78)
#define CTRL_BOOT           (CTRL + 0x134)

#define SLOTS               8
#define SLOT_STRIDE         0x0C
#define SLOT_BUSY           (1u << 0)
#define CTRL_DONE_INFO      (CTRL + 0x88)
#define ENC_INT_STATUS      (ENC + 0x60)
#define ENC_INT_CLEAR       (ENC + 0x64)
/*
 * enc+0x60 bits as HalVencIsrHandlerPartA turns them into MI events (the
 * Ifliegen SDK's E_MI_VENC_IMPL_*): bit 0 a frame done (FRM_DONE), bit 5
 * a buffer ready under run, bit 6 a slice done, bit 1 a ring drop.
 */
#define INT_FRAME_DONE      (1u << 0)
#define INT_DONE_ANY        0x63u
#define CTRL_OUT_ADD        (CTRL + 0x204)  /* 1: an output buffer */
#define CTRL_BS_END         (CTRL + 0x208)  /* and its start; the end */
#define CTRL_OUT_LIMIT      (CTRL + 0x20C)  /* its size in units, less 1 */
#define CTRL_BS_BITS        (CTRL + 0x210)
#define CONTEXTS            64
#define REPORT_GAP_NS       2000000     /* between an ISR and the next done */
#define ENC_BS_START        (ENC + 0x258)
#define PROG_SIZE           0x00A0      /* height << 16 | width */
#define PROG_FRAME          0x00BC      /* bit 8: an I frame */
#define FRAME_I             (1u << 8)
#define PROG_CODEC          0x1080      /* bit 1: H.265, else H.264 */
#define CODEC_H265          (1u << 1)
#define PROG_RESULT         0x1088      /* program register: result block */
#define PROG_BS             0x1118      /* the bitstream ring, inclusive */
#define PROG_BS_LIMIT       0x111C
#define PROG_BS_WPTR        0x1128      /* where this frame starts */
#define PROG_STATUS         0x10BC      /* program register: status block */
#define STATUS_CODE         0x18
#define STATUS_VALID        0x23
#define STATUS_FRAME        1
#define RESULT_BYTES        0x190
#define STATUS_BS_END       0x2C        /* last bitstream unit written */
#define PROG_AUX_END        0x15C
#define STATUS_SUB_Y        0x34
#define STATUS_SUB_C        0x38
#define PROG_SUB_C_END      0x16C
#define STATUS_BITS         0x44
#define STATUS_REC_Y        0x48
#define STATUS_REC_C        0x4C
#define PROG_REC_Y_END      0x17C
#define PROG_REC_C_END      0x198
#define BS_BYTES            64          /* the placeholder frame */
#define BS_MAX              (4 * 1024 * 1024)
#define ENCODERS            4           /* streams encoded at once */
#define PROG_SCAN           0x2000      /* bytes of program searched */
#define SS_DRAM_BASE        0x20000000

#define BOOT_START          (1u << 0)
#define BOOT_READY          (1u << 4)
#define CONTROL_RUNNING     (1u << 1)
#define FW_STATE_RUN        2
#define FW_STATE_HALTED     8
#define ENC_HALT            (ENC + 0x248)   /* 1: stop the firmware */

/*
 * A fired job's frame, encoded on a host thread of its own while the
 * guest runs, waited for at its completion.
 */
typedef struct SstarVencWork {
    int n;                      /* its slot */
    int ctx;
    bool h265, intra;
    int width, height;
    hwaddr prog;
    uint8_t *out;
    size_t len;
    struct SstarVencEncoder *enc;   /* NULL: nothing to encode with */
    QemuThread thread;
    bool started;               /* the thread is to be joined */
    bool trace;
    const char *dump;
} SstarVencWork;

typedef struct SstarVencReport {
    uint8_t n;
    uint32_t irq;
} SstarVencReport;

struct SstarVencState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[VENC_SIZE / 4];
    bool running;
    bool halted;
    uint8_t pending;            /* slots taken, not yet done */
    uint8_t done;               /* slots with reports outstanding */
    SstarVencReport rq[2 * SLOTS];  /* the reports, in order */
    uint32_t rq_head, rq_len;
    uint32_t out_base[CONTEXTS];    /* each context's output buffer */
    uint32_t out_units[CONTEXTS];
    uint64_t out_valid;
    uint32_t bs[SLOTS];         /* each done slot's bitstream start */
    uint32_t bytes[SLOTS];      /* and size */
    uint8_t ctx[SLOTS];         /* and its encoder context */
    int64_t due[SLOTS];         /* each pending slot's completion time */
    QEMUTimer *job;
    QEMUTimer *next;            /* reports the next done, after an ISR */
#ifdef SSTAR_VENC_LIBAVCODEC
    struct SstarVencEncoder {
        int ctx;                /* -1: free */
        bool h265;
        int width, height;
        int64_t frames;
        uint64_t used;          /* for replacement */
        AVCodecContext *c;
        AVFrame *pic;
        AVPacket *pkt;
        FILE *dump;
        SstarVencWork *busy;    /* the frame its thread is encoding */
    } enc[ENCODERS];
    uint64_t uses;
    SstarVencWork *ready[SLOTS];    /* each submitted slot's frame */
#endif

    uint32_t done_us;
    char *dump;
    bool trace;
};

static uint32_t *venc_reg(SstarVencState *s, hwaddr off)
{
    return &s->regs[(off & (VENC_SIZE - 1)) >> 2];
}

static void sstar_venc_update_irq(SstarVencState *s)
{
    qemu_set_irq(s->irq, !!s->regs[ENC_INT_STATUS >> 2]);
}

static hwaddr venc_dram(uint32_t miu16)
{
    hwaddr a = (hwaddr)miu16 << 4;

    return a < SS_DRAM_BASE ? SS_DRAM_BASE + a : a;
}

/* The register a job's program sets, or 0. */
static uint32_t venc_prog_reg(hwaddr prog, uint32_t reg)
{
    uint32_t pair[2];
    hwaddr i;

    for (i = 0; i < PROG_SCAN; i += 8) {
        dma_memory_read(&address_space_memory, prog + i, pair, sizeof(pair),
                        MEMTXATTRS_UNSPECIFIED);
        if (le32_to_cpu(pair[1]) == reg) {
            return le32_to_cpu(pair[0]);
        }
    }
    return 0;
}

/* Report the next finished job, if the last one has been taken. */
static void sstar_venc_report(SstarVencState *s)
{
    int n;
    uint32_t irq;

    if (!s->rq_len || (s->regs[ENC_INT_STATUS >> 2] & INT_DONE_ANY) ||
        timer_pending(s->next)) {
        return;
    }
    n = s->rq[s->rq_head].n;
    irq = s->rq[s->rq_head].irq;
    s->rq_head = (s->rq_head + 1) % ARRAY_SIZE(s->rq);
    s->rq_len--;
    s->regs[CTRL_DONE_INFO >> 2] = (s->regs[CTRL_DONE_INFO >> 2] &
                                    ~(0x3Fu << 8)) | (s->ctx[n] << 8);
    if (irq == INT_FRAME_DONE) {
        s->regs[CTRL_BS_BITS >> 2] = s->bytes[n] * 8;
        s->regs[ENC_BS_START >> 2] = s->bs[n];
        s->regs[CTRL_BS_END >> 2] = s->bs[n] + DIV_ROUND_UP(s->bytes[n], 16);
    }
    s->regs[ENC_INT_STATUS >> 2] |= irq;
    sstar_venc_update_irq(s);
}

/* Queue a done report; the ISR takes them one at a time. */
static void venc_queue_report(SstarVencState *s, int n, uint32_t irq)
{
    if (s->rq_len < ARRAY_SIZE(s->rq)) {
        s->rq[(s->rq_head + s->rq_len) % ARRAY_SIZE(s->rq)].n = n;
        s->rq[(s->rq_head + s->rq_len) % ARRAY_SIZE(s->rq)].irq = irq;
        s->rq_len++;
    }
}

#ifdef SSTAR_VENC_LIBAVCODEC
/*
 * Wait for a frame's thread.  A running guest has its virtual clock held
 * meanwhile, so the frame is never late in guest time; a stopped one
 * (a load, a reset) has nothing to hold.
 */
static void venc_wait(SstarVencWork *w)
{
    int64_t t0;
    bool hold;

    if (!w || !w->started) {
        return;
    }
    hold = runstate_is_running();
    if (hold) {
        cpu_disable_ticks();
    }
    t0 = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
    qemu_thread_join(&w->thread);
    if (hold) {
        cpu_enable_ticks();
    }
    if (w->trace) {
        qemu_log("sstar-venc: ctx %d waited %" PRId64 " us\n", w->ctx,
                 qemu_clock_get_us(QEMU_CLOCK_REALTIME) - t0);
    }
    w->started = false;
    if (w->enc && w->enc->busy == w) {
        w->enc->busy = NULL;
    }
}

static void venc_encoder_close(struct SstarVencEncoder *e)
{
    venc_wait(e->busy);
    avcodec_free_context(&e->c);
    av_frame_free(&e->pic);
    av_packet_free(&e->pkt);
    if (e->dump) {
        fclose(e->dump);
        e->dump = NULL;
    }
    e->ctx = -1;
}

/*
 * The encoder of a stream, for its codec and size: a slot of the table,
 * closed when it was another stream's or another codec or size.  The
 * frame's thread opens it (venc_encoder_open), so the vCPU never does.
 * Only the main thread picks and closes, and closing waits for the slot's
 * frame.
 */
static struct SstarVencEncoder *venc_encoder(SstarVencState *s, int ctx,
                                             bool h265, int w, int h)
{
    struct SstarVencEncoder *e = NULL;
    int i;

    for (i = 0; i < ENCODERS; i++) {
        if (s->enc[i].ctx == ctx) {
            e = &s->enc[i];
            break;
        }
    }
    if (e && (e->h265 != h265 || e->width != w || e->height != h)) {
        venc_encoder_close(e);
    }
    if (!e) {
        e = &s->enc[0];
        for (i = 0; i < ENCODERS; i++) {
            if (s->enc[i].ctx < 0) {
                e = &s->enc[i];
                break;
            }
            if (s->enc[i].used < e->used) {
                e = &s->enc[i];
            }
        }
        if (e->ctx >= 0) {
            venc_encoder_close(e);
        }
    }
    e->used = ++s->uses;
    if (e->ctx != ctx) {
        e->ctx = ctx;
        e->h265 = h265;
        e->width = w;
        e->height = h;
        e->frames = 0;
    }
    return e;
}

/*
 * Open a slot's encoder, on the frame's thread; false if it cannot be.
 * A slot that failed stays closed, and its next frame tries again.
 */
static bool venc_encoder_open(struct SstarVencEncoder *e, const char *dump)
{
    const AVCodec *codec;
    int w = e->width, h = e->height;

    codec = avcodec_find_encoder_by_name(e->h265 ? "libx265" : "libx264");
    if (!codec || w < 16 || h < 16 || w > 8192 || h > 8192) {
        return false;
    }
    e->c = avcodec_alloc_context3(codec);
    e->pic = av_frame_alloc();
    e->pkt = av_packet_alloc();
    if (!e->c || !e->pic || !e->pkt) {
        goto fail;
    }
    e->c->width = w;
    e->c->height = h;
    e->c->pix_fmt = AV_PIX_FMT_YUV420P;
    e->c->time_base = (AVRational){ 1, 25 };
    e->c->framerate = (AVRational){ 25, 1 };
    e->c->gop_size = 1000;      /* the driver asks for the I frames */
    e->c->max_b_frames = 0;
    e->c->thread_count = 0;     /* x264's sliced threads; x265 pools anyway */
    av_opt_set(e->c->priv_data, "preset", "ultrafast", 0);
    av_opt_set(e->c->priv_data, "tune", "zerolatency", 0);
    av_opt_set(e->c->priv_data, "crf", "30", 0);
    av_opt_set(e->c->priv_data, "forced-idr", "1", 0);
    av_opt_set(e->c->priv_data, e->h265 ? "x265-params" : "x264-params",
               e->h265 ? "repeat-headers=1:log-level=error"
                       : "repeat-headers=1", 0);
    if (avcodec_open2(e->c, codec, NULL) < 0) {
        goto fail;
    }
    e->pic->format = AV_PIX_FMT_YUV420P;
    e->pic->width = w;
    e->pic->height = h;
    if (av_frame_get_buffer(e->pic, 0) < 0) {
        goto fail;
    }
    if (dump && *dump && !e->dump) {
        g_autofree char *name = g_strdup_printf("%s-ctx%d.%s", dump, e->ctx,
                                                e->h265 ? "h265" : "h264");

        e->dump = fopen(name, "ab");
    }
    return true;

fail:
    avcodec_free_context(&e->c);
    av_frame_free(&e->pic);
    av_packet_free(&e->pkt);
    return false;
}

/* Colour bars, a box moving across them, the frame number in binary. */
static void venc_pattern(AVFrame *f, int64_t frame)
{
    static const uint8_t bars[8][3] = {     /* 75% bars, BT.601 Y Cb Cr */
        { 180, 128, 128 }, { 162, 44, 142 }, { 131, 156, 44 },
        { 112, 72, 58 }, { 84, 184, 198 }, { 65, 100, 212 },
        { 35, 212, 114 }, { 16, 128, 128 },
    };
    int w = f->width, h = f->height, x, y, bit;
    /* both clipped to the frame, which may be as small as 16x16 */
    int box = MIN(MAX(h / 6, 16), MIN(w, h));
    int bx = (int)((frame * 8) % (w - box + 1)), by = h / 2 - box / 2;
    int cell = MIN(MAX(w / 40, 8), h);

    av_frame_make_writable(f);
    for (y = 0; y < h; y++) {
        uint8_t *py = f->data[0] + y * f->linesize[0];

        for (x = 0; x < w; x++) {
            py[x] = bars[x * 8 / w][0];
        }
        if (y >= by && y < by + box) {
            memset(py + bx, 235, box);
        }
        if (y < cell) {
            for (bit = 0; bit < 16; bit++) {
                int cx = (15 - bit) * cell;

                if (cx < w) {
                    memset(py + cx, (frame >> bit) & 1 ? 235 : 16,
                           MIN(cell - 2, w - cx));
                }
            }
        }
    }
    for (y = 0; y < h / 2; y++) {
        uint8_t *pu = f->data[1] + y * f->linesize[1];
        uint8_t *pv = f->data[2] + y * f->linesize[2];

        for (x = 0; x < w / 2; x++) {
            int b = x * 16 / w;
            bool white = (y * 2 >= by && y * 2 < by + box &&
                          x * 2 >= bx && x * 2 < bx + box) || y * 2 < cell;

            pu[x] = white ? 128 : bars[b][1];
            pv[x] = white ? 128 : bars[b][2];
        }
    }
}

/* The frame's thread: encode it into w->out. */
static void *venc_encode_thread(void *opaque)
{
    SstarVencWork *w = opaque;
    struct SstarVencEncoder *e = w->enc;
    int64_t t0 = qemu_clock_get_us(QEMU_CLOCK_REALTIME), t1;
    GByteArray *au;

#ifdef CONFIG_LINUX
    /*
     * Below the vCPU, and so are the pool threads x265 starts when this
     * thread opens it.  On a host the encoders could saturate, a starved
     * vCPU lets the guest clock run past the boot watchdog (60 s, armed
     * by U-Boot); a frame that is late instead holds the clock.
     */
    setpriority(PRIO_PROCESS, qemu_get_thread_id(), 10);
#endif
    if (!e->c && !venc_encoder_open(e, w->dump)) {
        return NULL;
    }
    t1 = qemu_clock_get_us(QEMU_CLOCK_REALTIME);
    venc_pattern(e->pic, e->frames);
    e->pic->pts = e->frames++;
    e->pic->pict_type = w->intra ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    if (avcodec_send_frame(e->c, e->pic) < 0) {
        return NULL;
    }
    au = g_byte_array_new();
    while (avcodec_receive_packet(e->c, e->pkt) == 0) {
        if (au->len + e->pkt->size <= BS_MAX) {
            g_byte_array_append(au, e->pkt->data, e->pkt->size);
        }
        av_packet_unref(e->pkt);
    }
    if (e->dump && au->len) {
        fwrite(au->data, 1, au->len, e->dump);
        fflush(e->dump);
    }
    w->len = au->len;
    w->out = g_byte_array_free(au, false);
    if (w->trace) {
        qemu_log("sstar-venc: ctx %d %s %dx%d%s: %zu bytes, open %" PRId64
                 " us, encode %" PRId64 " us\n", w->ctx,
                 w->h265 ? "H.265" : "H.264", w->width, w->height,
                 w->intra ? " I" : "", w->len, t1 - t0,
                 qemu_clock_get_us(QEMU_CLOCK_REALTIME) - t1);
    }
    return NULL;
}

/*
 * Start a job's frame: its stream's encoder, once that has finished the
 * stream's previous frame, then a thread for the encode itself.
 */
static void venc_encode(SstarVencState *s, SstarVencWork *w)
{
    struct SstarVencEncoder *e;

    e = venc_encoder(s, w->ctx, w->h265, w->width, w->height);
    venc_wait(e->busy);
    w->enc = e;
    w->trace = s->trace;
    w->dump = s->dump;
    e->busy = w;
    qemu_thread_create(&w->thread, "sstar-venc", venc_encode_thread, w,
                       QEMU_THREAD_JOINABLE);
    w->started = true;
}

#endif

/*
 * Write an access unit into the bitstream ring from its write pointer;
 * the last 16-byte unit written, and the start in s->bs[n].
 */
static uint32_t venc_ring_write(SstarVencState *s, int n, hwaddr prog,
                                const uint8_t *out, size_t len)
{
    uint32_t base = venc_prog_reg(prog, PROG_BS);
    uint32_t limit = venc_prog_reg(prog, PROG_BS_LIMIT);
    uint32_t wptr = venc_prog_reg(prog, PROG_BS_WPTR);
    uint64_t start, end, ring, first;

    if (!wptr || wptr < base || wptr > limit) {
        wptr = base;
    }
    s->bs[n] = wptr;
    if (!base || limit < base) {
        return wptr + DIV_ROUND_UP(len, 16) - 1;
    }
    start = (uint64_t)wptr << 4;
    end = ((uint64_t)limit + 1) << 4;
    ring = end - ((uint64_t)base << 4);
    len = MIN(len, ring);
    first = MIN(len, end - start);
    dma_memory_write(&address_space_memory, venc_dram(wptr), out, first,
                     MEMTXATTRS_UNSPECIFIED);
    if (first < len) {
        dma_memory_write(&address_space_memory, venc_dram(base),
                         out + first, len - first, MEMTXATTRS_UNSPECIFIED);
        return base + DIV_ROUND_UP(len - first, 16) - 1;
    }
    return (start + len - 1) >> 4;
}

/* A job's frame is encoded: write it out and report the slot done. */
static void venc_finish(SstarVencState *s, int n, hwaddr prog,
                        const uint8_t *out, size_t len)
{
    uint32_t *w0 = &s->regs[(CTRL + n * SLOT_STRIDE) >> 2];
    uint8_t placeholder[BS_BYTES];
    uint32_t res, st, last;

    if (!len) {
        /* an access unit delimiter, then filler */
        static const uint8_t aud264[] = { 0, 0, 0, 1, 0x09, 0xF0 };
        static const uint8_t aud265[] = { 0, 0, 0, 1, 0x46, 0x01, 0x50 };
        bool h265 = venc_prog_reg(prog, PROG_CODEC) & CODEC_H265;

        memset(placeholder, 0, sizeof(placeholder));
        memcpy(placeholder, h265 ? aud265 : aud264,
               h265 ? sizeof(aud265) : sizeof(aud264));
        out = placeholder;
        len = sizeof(placeholder);
    }
    s->bytes[n] = len;
    last = venc_ring_write(s, n, prog, out, len);
    /*
     * The frame's stream goes to the output buffer the driver gave its
     * context (HalVencIosAddOutBuffer, a record of the channel's MI ring
     * pool), which is what the completion reports; the status block keeps
     * the program's own ring, which HalVencDpbDoneFrame checks.  A
     * context's first frame, before any, is reported in the ring.
     */
    if (s->out_valid & (1ull << s->ctx[n])) {
        int c = s->ctx[n];
        size_t room = (size_t)s->out_units[c] * 16;

        s->out_valid &= ~(1ull << c);
        len = MIN(len, room);
        s->bytes[n] = len;
        s->bs[n] = s->out_base[c];
        dma_memory_write(&address_space_memory, venc_dram(s->out_base[c]),
                         out, len, MEMTXATTRS_UNSPECIFIED);
    }

    res = venc_prog_reg(prog, PROG_RESULT);
    st = venc_prog_reg(prog, PROG_STATUS);
    if (st) {
        uint8_t code = STATUS_FRAME, valid = 1;
        uint32_t w[3] = {
            cpu_to_le32(len * 8),
            cpu_to_le32(venc_prog_reg(prog, PROG_REC_Y_END) - 1),
            cpu_to_le32(venc_prog_reg(prog, PROG_REC_C_END) - 1),
        };
        uint32_t sub[2] = {
            cpu_to_le32(venc_prog_reg(prog, PROG_AUX_END) - 1),
            cpu_to_le32(venc_prog_reg(prog, PROG_SUB_C_END) - 1),
        };
        uint32_t bs_end = cpu_to_le32(last);

        dma_memory_write(&address_space_memory, venc_dram(st) + STATUS_BITS,
                         w, sizeof(w), MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory,
                         venc_dram(st) + STATUS_BS_END, &bs_end, 4,
                         MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, venc_dram(st) + STATUS_SUB_Y,
                         sub, sizeof(sub), MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, venc_dram(st) + STATUS_CODE,
                         &code, 1, MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory, venc_dram(st) + STATUS_VALID,
                         &valid, 1, MEMTXATTRS_UNSPECIFIED);
    }
    if (res) {
        uint32_t bytes = cpu_to_le32(len);

        dma_memory_write(&address_space_memory, venc_dram(res) + RESULT_BYTES,
                         &bytes, 4, MEMTXATTRS_UNSPECIFIED);
    }
    if (s->trace) {
        qemu_log("sstar-venc: %" PRId64 " ms slot %d done, ctx %d, %zu bytes"
                 " at 0x%08" PRIx64 ", program 0x%08" PRIx64 "\n",
                 qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), n, s->ctx[n], len,
                 (uint64_t)venc_dram(s->bs[n]), (uint64_t)prog);
    }
    *w0 &= ~SLOT_BUSY;
    venc_queue_report(s, n, INT_FRAME_DONE);
    sstar_venc_report(s);
}



#ifdef SSTAR_VENC_LIBAVCODEC
/*
 * Encode a job's frame as it is fired, with the virtual clock stopped:
 * the hardware takes a few milliseconds where libx265 takes tens, and a
 * guest that runs on meanwhile queues the channel's next frame onto the
 * ring-pool record of one still in flight (the driver keeps one per
 * channel), whose tail reservation then fails ([550] in the ISR).
 */
/* Start encoding slot n's job. */
static void venc_submit(SstarVencState *s, int n, uint32_t w0)
{
    hwaddr prog = venc_dram(s->regs[(CTRL + n * SLOT_STRIDE + 8) >> 2]);
    uint32_t size = venc_prog_reg(prog, PROG_SIZE);
    SstarVencWork *w = g_new0(SstarVencWork, 1);

    w->n = n;
    w->ctx = (w0 >> 24) & 0x3F;
    w->h265 = venc_prog_reg(prog, PROG_CODEC) & CODEC_H265;
    w->intra = venc_prog_reg(prog, PROG_FRAME) & FRAME_I;
    w->width = size & 0xFFFF;
    w->height = size >> 16;
    w->prog = prog;
    if (s->ready[n]) {
        venc_wait(s->ready[n]);
        g_free(s->ready[n]->out);
        g_free(s->ready[n]);
    }
    venc_encode(s, w);
    s->ready[n] = w;
}

static void venc_drop_ready(SstarVencState *s)
{
    int n;

    for (n = 0; n < SLOTS; n++) {
        if (s->ready[n]) {
            venc_wait(s->ready[n]);
            g_free(s->ready[n]->out);
            g_free(s->ready[n]);
            s->ready[n] = NULL;
        }
    }
}
#endif

/* The job timer, for the earliest pending slot. */
static void venc_arm(SstarVencState *s)
{
    int64_t when = INT64_MAX;
    int n;

    for (n = 0; n < SLOTS; n++) {
        if ((s->pending & (1u << n)) && s->due[n] < when) {
            when = s->due[n];
        }
    }
    if (when == INT64_MAX) {
        timer_del(s->job);
    } else {
        timer_mod(s->job, when);
    }
}

static void sstar_venc_next(void *opaque)
{
    sstar_venc_report(opaque);
}

static void sstar_venc_job(void *opaque)
{
    SstarVencState *s = opaque;
    int n;

    for (n = 0; n < SLOTS; n++) {
        uint32_t *w0 = &s->regs[(CTRL + n * SLOT_STRIDE) >> 2];
        hwaddr prog;

        if (!(s->pending & (1u << n)) ||
            s->due[n] > qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) {
            continue;
        }
        s->pending &= ~(1u << n);
        prog = venc_dram(s->regs[(CTRL + n * SLOT_STRIDE + 8) >> 2]);
        s->ctx[n] = (*w0 >> 24) & 0x3F;
#ifdef SSTAR_VENC_LIBAVCODEC
        if (s->ready[n]) {
            SstarVencWork *w = s->ready[n];

            venc_wait(w);
            s->ready[n] = NULL;
            venc_finish(s, n, w->prog, w->out, w->len);
            g_free(w->out);
            g_free(w);
            continue;
        }
#endif
        venc_finish(s, n, prog, NULL, 0);
    }
    venc_arm(s);
}

static uint32_t sstar_venc_reg_read(SstarVencState *s, hwaddr off)
{
    uint32_t v = *venc_reg(s, off);

    switch (off) {
    case CTRL_BOOT:
        v |= BOOT_READY;
        break;
    case CTRL_CONTROL:
        if (s->running) {
            v |= CONTROL_RUNNING;
        }
        break;
    case CTRL_FW_STATE:
        if (s->halted) {
            v = (v & ~0xF) | FW_STATE_HALTED;
        } else if (s->running) {
            v = (v & ~0xF) | FW_STATE_RUN;
        }
        break;
    }
    return v;
}

static void sstar_venc_reg_write(SstarVencState *s, hwaddr off, uint32_t v)
{
    if (off == ENC_INT_CLEAR) {
        s->regs[ENC_INT_STATUS >> 2] &= ~v;
        sstar_venc_update_irq(s);
        /*
         * The ISR clears first and goes on with the shared registers (the
         * done's ctrl+0x208, then the next output buffer in 0x204-0x20C,
         * for the context it was told of), so the next done waits.
         */
        timer_mod(s->next, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  REPORT_GAP_NS);
        return;
    }
    *venc_reg(s, off) = v;
    switch (off) {
    case CTRL_BOOT:
        if (v & BOOT_START) {
            s->running = true;
            s->halted = false;
        }
        break;
    case CTRL_OUT_ADD:
        if (v & 1) {
            int c = (s->regs[CTRL_DONE_INFO >> 2] >> 8) & 0x3F;

            s->out_base[c] = s->regs[CTRL_BS_END >> 2];
            s->out_units[c] = s->regs[CTRL_OUT_LIMIT >> 2] + 1;
            s->out_valid |= 1ull << c;
            if (s->trace) {
                qemu_log("sstar-venc: ctx %d output buffer 0x%08x, %u units\n",
                         c, s->out_base[c], s->out_units[c]);
            }
        }
        break;
    case ENC_HALT:
        /* HalVencIosFirmWareDeInit: the firmware stops, its jobs with it */
        if ((v & 1) && s->running) {
            s->running = false;
            s->halted = true;
#ifdef SSTAR_VENC_LIBAVCODEC
            venc_drop_ready(s);
#endif
            s->out_valid = 0;
            s->pending = 0;
            s->done = 0;
            s->rq_len = 0;
            timer_del(s->job);
            s->regs[ENC_INT_STATUS >> 2] = 0;
            sstar_venc_update_irq(s);
        }
        break;
    }
    if (off < CTRL + SLOTS * SLOT_STRIDE && off % SLOT_STRIDE == 0 &&
        (v & SLOT_BUSY) && s->running) {
#ifdef SSTAR_VENC_LIBAVCODEC
        venc_submit(s, off / SLOT_STRIDE, v);
#endif
        s->pending |= 1u << (off / SLOT_STRIDE);
        s->due[off / SLOT_STRIDE] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    (int64_t)s->done_us * 1000;
        venc_arm(s);
    }
}

static uint64_t sstar_venc_read(void *opaque, hwaddr off, unsigned size)
{
    SstarVencState *s = opaque;
    uint32_t v = sstar_venc_reg_read(s, off & ~3);


    v >>= (off & 3) * 8;
    if (size < 4) {
        v &= (1u << (size * 8)) - 1;
    }
    if (s->trace) {
        qemu_log("sstar-venc: R %s+0x%03x = 0x%08x\n",
                 off >= ENC ? "enc" : "ctrl", (unsigned)(off & 0x3FF), v);
    }
    return v;
}

static void sstar_venc_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarVencState *s = opaque;
    uint32_t v = val;

    if (size < 4) {
        int sh = (off & 3) * 8;
        uint32_t mask = ((1u << (size * 8)) - 1) << sh;

        v = (*venc_reg(s, off & ~3) & ~mask) | ((v << sh) & mask);
    }
    if (s->trace) {
        qemu_log("sstar-venc: W %s+0x%03x = 0x%08x\n",
                 off >= ENC ? "enc" : "ctrl", (unsigned)(off & 0x3FC), v);
    }
    sstar_venc_reg_write(s, off & ~3, v);
}

static const MemoryRegionOps sstar_venc_ops = {
    .read = sstar_venc_read,
    .write = sstar_venc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_venc_reset(DeviceState *dev)
{
    SstarVencState *s = SSTAR_VENC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->running = false;
    s->halted = false;
#ifdef SSTAR_VENC_LIBAVCODEC
    venc_drop_ready(s);
#endif
    s->out_valid = 0;
    s->pending = s->done = 0;
    s->rq_len = 0;
    timer_del(s->job);
    timer_del(s->next);
    qemu_set_irq(s->irq, 0);
}

static void sstar_venc_realize(DeviceState *dev, Error **errp)
{
    SstarVencState *s = SSTAR_VENC(dev);

    s->job = timer_new_ns(QEMU_CLOCK_VIRTUAL, sstar_venc_job, s);
    s->next = timer_new_ns(QEMU_CLOCK_VIRTUAL, sstar_venc_next, s);
#ifdef SSTAR_VENC_LIBAVCODEC
    {
        int i;

        for (i = 0; i < ENCODERS; i++) {
            s->enc[i].ctx = -1;
        }
    }
#endif
}

static void sstar_venc_init(Object *obj)
{
    SstarVencState *s = SSTAR_VENC(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_venc_ops, s,
                          TYPE_SSTAR_VENC, VENC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_venc_props[] = {
    DEFINE_PROP_UINT32("done-us", SstarVencState, done_us, 30000),
    DEFINE_PROP_STRING("dump", SstarVencState, dump),
    DEFINE_PROP_BOOL("trace", SstarVencState, trace, false),
};

static const VMStateDescription vmstate_sstar_venc_report = {
    .name = TYPE_SSTAR_VENC "/report",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(n, SstarVencReport),
        VMSTATE_UINT32(irq, SstarVencReport),
        VMSTATE_END_OF_LIST()
    }
};

/*
 * The encoded frames of fired jobs are host memory, not state: they are
 * encoded again from the guest's program once loaded.
 */
static int sstar_venc_post_load(void *opaque, int version_id)
{
#ifdef SSTAR_VENC_LIBAVCODEC
    SstarVencState *s = opaque;
    int n;

    venc_drop_ready(s);
    for (n = 0; n < SLOTS; n++) {
        if (s->pending & (1u << n)) {
            venc_submit(s, n, s->regs[(CTRL + n * SLOT_STRIDE) >> 2]);
        }
    }
#endif
    return 0;
}

static const VMStateDescription vmstate_sstar_venc = {
    .name = TYPE_SSTAR_VENC,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = sstar_venc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SstarVencState, VENC_SIZE / 4),
        VMSTATE_BOOL(running, SstarVencState),
        VMSTATE_BOOL(halted, SstarVencState),
        VMSTATE_UINT8(pending, SstarVencState),
        VMSTATE_UINT8(done, SstarVencState),
        VMSTATE_UINT32_ARRAY(bs, SstarVencState, SLOTS),
        VMSTATE_UINT8_ARRAY(ctx, SstarVencState, SLOTS),
        VMSTATE_TIMER_PTR(job, SstarVencState),
        VMSTATE_STRUCT_ARRAY(rq, SstarVencState, 2 * SLOTS, 1,
                             vmstate_sstar_venc_report, SstarVencReport),
        VMSTATE_UINT32(rq_head, SstarVencState),
        VMSTATE_UINT32(rq_len, SstarVencState),
        VMSTATE_UINT32_ARRAY(out_base, SstarVencState, CONTEXTS),
        VMSTATE_UINT32_ARRAY(out_units, SstarVencState, CONTEXTS),
        VMSTATE_UINT64(out_valid, SstarVencState),
        VMSTATE_UINT32_ARRAY(bytes, SstarVencState, SLOTS),
        VMSTATE_INT64_ARRAY(due, SstarVencState, SLOTS),
        VMSTATE_TIMER_PTR(next, SstarVencState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_venc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_venc_realize;
    device_class_set_legacy_reset(dc, sstar_venc_reset);
    device_class_set_props(dc, sstar_venc_props);
    dc->vmsd = &vmstate_sstar_venc;
}

static const TypeInfo sstar_venc_info = {
    .name           = TYPE_SSTAR_VENC,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarVencState),
    .instance_init  = sstar_venc_init,
    .class_init     = sstar_venc_class_init,
};

static void sstar_venc_register_types(void)
{
    type_register_static(&sstar_venc_info);
}

type_init(sstar_venc_register_types)
