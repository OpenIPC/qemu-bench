/*
 * Ambarella S3L DSP (ORC) stand-in, enough of the iav<->DSP interface for a
 * software encoder to take the place of the microcode the real part runs.
 *
 * The ORC is a polling coprocessor, not a register block: iav.ko and the DSP
 * talk through buffers at the top of the /iavmem region, synchronised with
 * sequence numbers, and the DSP drives everything from a frame-rate interrupt
 * it raises itself.  So this models a heartbeat: a timer that services the
 * command buffer and answers in the message buffer, which is what makes iav
 * believe a DSP booted.  On top of that boot handshake (chip id + the DSP
 * status message) it also synthesises the encode pipeline: once the driver
 * sets a stream up and starts it, every heartbeat reports a captured frame
 * (VCAP_STATUS) and encodes one -- MJPEG with the shared jpeg-enc.c, or
 * Baseline H.264 with hantro-h264.c -- into the bitstream buffer the driver
 * allocated, described by an enc_bits_info_t, so iav hands whole frames to a
 * client exactly as the real microcode would.  dump-dir
 * (-global amb-dsp.dump-dir=DIR) keeps every frame on the host for inspection.
 *
 * The command and message buffers are top-anchored in the iavmem region the
 * machine models (the microcode window is the last 5 MiB, the log, bitstream-
 * header fifo, message and command buffers sit just below it), so their
 * physical addresses follow from the region's end alone.  The bitstream and
 * bits-info fifos are not: the driver allocates them and names them in the
 * encode-setup command, which this reads.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/dma.h"
#include "jpeg-enc.h"
#include "hantro-h264.h"

#define TYPE_AMB_DSP "amb-dsp"
OBJECT_DECLARE_SIMPLE_TYPE(AmbDspState, AMB_DSP)

/* DSP area, counted down from the top of /iavmem (see amba_arch_mem.h). */
#define DSP_UCODE_SIZE      (5 * MiB)
#define DSP_LOG_SIZE        (128 * KiB)
#define DSP_BSH_SIZE        (64 * 256)      /* DSP_BS_UNIT_SIZE * DSP_BS_NUM */
#define DSP_PORT_SIZE       (4 * KiB)       /* one command/message port */
#define DSP_CMD_BUF_SIZE    (DSP_PORT_SIZE * 2)
#define DSP_MSG_BUF_SIZE    (DSP_PORT_SIZE * 2)

/* Offsets inside the microcode window. */
#define DSP_CHIP_ID_OFFSET   0x424000
#define DSP_INIT_DATA_OFFSET 0x424408      /* chip-id + 4 + 1 KiB + 4 */

/* DSP_INIT_DATA field offsets iav fills (daddr = DSP address). */
#define INIT_DEFBIN_ADDR     0
#define INIT_CMD_GEN_DADDR   8
#define INIT_MSG_GEN_DADDR   16
#define INIT_CMD_VCAP_DADDR  24
#define INIT_MSG_VCAP_DADDR  32
#define INIT_DSPBUF_DADDR    80     /* dsp_buffer_daddr: the DSP's own scratch */
#define INIT_DSPBUF_SIZE     84
#define INIT_CHIP_ID_DADDR   92

/* The S3Lm part id the loader also selects by (RCT 0x300 == 0x4a000000). */
#define AMBA_CHIP_ID_S3L_66  4

/* enum dsp_op_mode */
#define DSP_OP_MODE_IDLE          0
#define DSP_OP_MODE_CAMERA_RECORD 1
#define DSP_OP_MODE_INIT          255

/* command/message category and the codes we care about */
#define CMD_DSP_SET_OPERATION_MODE 0x01000001
#define CMD_DSP_SUSPEND_OPERATION_MODE 0x01000003
#define CMD_VCAP_SETUP             0x08000001
#define CMD_VCAP_TMR_MODE          0x08000003
#define VCAP_SETUP_MAIN_WH_OFF     12    /* main_w:16 | main_h:16 */
#define CMD_H264ENC_START          0x02000002
#define CMD_H264ENC_STOP           0x02000003
#define CMD_H264ENC_SETUP          0x02000001
#define CMD_H264ENC_JPEG_SETUP     0x02000004

/* cmd_encoder_jpeg_setup_t field byte offsets (iav hands us its buffers). */
#define JS_WH_OFF        16    /* encode_w:16 | encode_h:16 */
#define JS_BITS_BASE_OFF 20    /* bits_fifo_base (the BSB) */
#define JS_BITS_SIZE_OFF 24
#define JS_INFO_BASE_OFF 28    /* info_fifo_base (the BSH, enc_bits_info ring) */
#define JS_INFO_SIZE_OFF 32

/* cmd_encoder_setup_t (H.264) field byte offsets. */
#define HS_PROFILE_OFF   6     /* profile_idc (byte) */
#define HS_LEVEL_OFF     7     /* level_idc (byte) */
#define HS_WH_OFF        12    /* encode_w_sz:16 | encode_h_sz:16 */
#define HS_BITS_BASE_OFF 44    /* bits_fifo_base */
#define HS_BITS_SIZE_OFF 48
#define HS_INFO_BASE_OFF 52    /* info_fifo_base */
#define HS_INFO_SIZE_OFF 56

/* estr[].codec */
#define AMB_CODEC_JPEG   1
#define AMB_CODEC_H264   2
#define MSG_DSP_STATUS             0x81000001
#define MSG_VCAP_STATUS            0x88000001
#define MSG_VCAP_FRAME_DONE        0x88000003
#define MSG_H264ENC_STATUS         0x82000001

/* enc_status_msg_t: the one field iav keys the encode-start gate on. */
#define ENC_STATUS_ENCODE          1   /* enc_status_t STATUS_ENCODE */
/* enc_status_msg_t per-codec frame counters; iav queues frames on the delta */
#define ENC_STATUS_H264_CTR_OFF    8    /* total_bits_info_ctr_h264 (W2) */
#define ENC_STATUS_JPEG_CTR_OFF    16   /* total_bits_info_ctr_jpeg (W4) */

/* Fallback MJPEG resolution when the setup command leaves it to the DSP. */
#define ENC_W                      1280
#define ENC_H                      720

/*
 * enc_bits_info_t (BIT_STREAM_HDR), 64 bytes, one per encoded frame in the
 * BSH fifo.  iav reads stream_id (byte 0), pic_type (W1 low nibble),
 * bits_start_addr (W6) and bits_size (W7, 24-bit) to build a frame descriptor.
 */
#define EBI_STEP                   64
#define EBI_W_ID                   0
#define EBI_W_PICTYPE              1
#define EBI_W_FRAMENUM             3
#define EBI_W_PTS                  4
#define EBI_W_START                6
#define EBI_W_SIZE                 7
#define FRAME_TYPE_IDR             1   /* frame_type_t IDR_PICT */
#define FRAME_TYPE_JPG             5   /* frame_type_t JPG_T */
#define JPEG_QUALITY               50

/* Standard JPEG Annex K quantisers, natural (row-major) order. */
static const uint8_t amb_dsp_ql[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
};
static const uint8_t amb_dsp_qc[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
};

/* VCAP_STATUS_MSG: the fields do_process_msg keys on (byte offsets). */
#define VCAP_VIDEO_MODE            1   /* vcap_setup_mode value for recording */
#define VCAP_MSG_MODE_OFF          4   /* vcap_setup_mode (low 3 bits) + flags */
#define VCAP_MSG_LUMA_OFF          16  /* main_pic_luma_addr; nonzero => captured */
/*
 * The message carries a buffer-report block at byte 8 that handle_vcap_msg
 * memcpys wholesale into iav's buffer-descriptor global (.LANCHOR1+4); from
 * there irq_update_bufcap feeds the main source buffer's luma/chroma/pitch into
 * what iav_query_yuvdesc returns.  The main buffer's fields sit at block+8/+12/
 * +18 => message bytes 16/20/26.  Without chroma and pitch a raw-YUV readback
 * comes back pitch 0 and gets rejected by the client's validity check.
 */
#define VCAP_MSG_CHROMA_OFF        20  /* main_pic_chroma_addr */
#define VCAP_MSG_WIDTH_OFF         24  /* main buffer width (u16) */
#define VCAP_MSG_PITCH_OFF         26  /* main buffer line pitch (u16) */

/* DSP_HEADER_CMD: the first command in each iteration (128 bytes). */
#define CMD_HDR_CODE_OFF   0
#define CMD_HDR_SEQ_OFF    4
#define CMD_HDR_NUM_OFF    8
#define DSP_CMD_SIZE       128
#define DSP_MSG_SIZE       128      /* one message slot after the status */

/* DSP_STATUS_MSG: the first, implicit message in a port (28 bytes). */
struct dsp_status_msg {
    uint32_t msg_code;
    uint32_t dsp_mode;
    uint32_t time_code;
    uint32_t prev_cmd_seq;
    uint32_t prev_cmd_echo;
    uint32_t prev_num_cmds;
    uint32_t num_msgs;
};

#define AMB_DSP_HZ 30

/* Largest encode this stand-in will synthesise; a guest setup past it is
 * rejected rather than turned into a multi-gigabyte host allocation. */
#define ENC_MAX_DIM 4096

/* Per-stream encode state, exactly as the setup command gave it.  Scalars
 * only (the encoder scratch is local to amb_dsp_emit_frame), so it migrates. */
typedef struct AmbDspStream {
    uint32_t bits_base, bits_size;   /* the BSB: where the bitstream goes */
    uint32_t info_base, info_size;   /* the BSH: enc_bits_info_t ring */
    uint32_t bits_off;               /* rolling write offset within the BSB */
    uint32_t frames;                 /* frames emitted (this stream's counter) */
    uint16_t w, h;
    uint8_t codec;                   /* AMB_CODEC_JPEG / _H264 */
    uint8_t profile, level;          /* H.264 only */
} AmbDspStream;

struct AmbDspState {
    SysBusDevice parent_obj;

    uint32_t iavmem_base;
    uint32_t iavmem_end;        /* one past the last DRAM byte */

    QEMUTimer heartbeat;
    qemu_irq vdsp_irq;
    qemu_irq vcap_irq;
    qemu_irq venc_irq;

    bool chip_written;
    uint32_t op_mode;           /* what we report back to the driver */
    uint32_t boot_ticks;        /* heartbeats since reset */
    bool mode_commanded;        /* set once the driver asks for a mode */
    bool vcap_paused;           /* capture timer stopped: report vcap setup
                                 * mode IDLE so iav's enc-sub-mode wait wakes */
    uint16_t main_w, main_h;    /* main capture buffer size (CMD_VCAP_SETUP) */
    uint32_t capture_frames;    /* captured pictures: animates the raw YUV */
    uint32_t enc_streams;       /* bitmask of streams the driver has started */
    char *dump_dir;             /* if set, each frame is kept as frame-N.jpg/.h264 */

    AmbDspStream estr[8];

    uint32_t last_seq[2];       /* last command seq seen per port (debug) */
    bool debug;
    bool init_dumped;
};

static hwaddr amb_dsp_ucode(AmbDspState *s)
{
    return s->iavmem_end - DSP_UCODE_SIZE;
}

/* Command buffer (port 0) physical address. */
static hwaddr amb_dsp_cmd_buf(AmbDspState *s)
{
    return amb_dsp_ucode(s) - DSP_LOG_SIZE - DSP_BSH_SIZE
           - DSP_MSG_BUF_SIZE - DSP_CMD_BUF_SIZE;
}

/* Message buffer (port 0) physical address. */
static hwaddr amb_dsp_msg_buf(AmbDspState *s)
{
    return amb_dsp_cmd_buf(s) + DSP_CMD_BUF_SIZE;
}

static uint32_t amb_dsp_ld(hwaddr a)
{
    uint32_t v = 0;
    dma_memory_read(&address_space_memory, a, &v, sizeof(v),
                    MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void amb_dsp_st(hwaddr a, uint32_t v)
{
    uint32_t le = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, a, &le, sizeof(le),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * Read the sequence number the driver last wrote into a command port, and
 * note any operation-mode request so we report the mode it asked for.
 */
static uint32_t amb_dsp_service_port(AmbDspState *s, hwaddr cmd_port, int port)
{
    uint32_t seq = amb_dsp_ld(cmd_port + CMD_HDR_SEQ_OFF);
    uint32_t num = amb_dsp_ld(cmd_port + CMD_HDR_NUM_OFF);
    bool fresh = s->debug && seq != s->last_seq[port];

    if (fresh) {
        fprintf(stderr, "amb-dsp: port%d seq %u->%u num=%u codes:", port,
                s->last_seq[port], seq, num);
    }

    /*
     * Walk the iteration's commands.  Command i sits at slot i + 1 (slot 0 is
     * the port header), so i + 1 must stay below the port's slot count or the
     * last read would spill into the next port.
     */
    for (uint32_t i = 0; i < num && i + 1 < DSP_PORT_SIZE / DSP_CMD_SIZE; i++) {
        hwaddr c = cmd_port + (i + 1) * DSP_CMD_SIZE;
        uint32_t code = amb_dsp_ld(c + CMD_HDR_CODE_OFF);
        if (fresh) {
            fprintf(stderr, " %08x", code);
        }
        if (code == CMD_DSP_SET_OPERATION_MODE) {
            uint32_t m = amb_dsp_ld(c + 4) & 0xff;
            if (m == DSP_OP_MODE_CAMERA_RECORD || m == DSP_OP_MODE_IDLE) {
                s->op_mode = m;
                s->mode_commanded = true;
                if (m == DSP_OP_MODE_CAMERA_RECORD) {
                    s->vcap_paused = false;   /* capture resumes with the mode */
                }
            }
        } else if (code == CMD_VCAP_SETUP) {
            /* The main capture buffer's dimensions: the raw-YUV readback a
             * client meters 3A from reports this geometry with a matching
             * pitch (main_w:16 | main_h:16 at byte 12). */
            uint32_t wh = amb_dsp_ld(c + VCAP_SETUP_MAIN_WH_OFF);
            s->main_w = wh & 0xffff;
            s->main_h = wh >> 16;
        } else if (code == CMD_VCAP_TMR_MODE) {
            /*
             * iav moves the VCAP "setup mode" -- the enc sub-mode it polls --
             * by starting/stopping the capture timer.  A zero timer scaler
             * stops it; iav then waits (dsp_set_enc_sub_mode) for the DSP to
             * report setup mode IDLE before suspending the operation mode.
             * Track the pause so the next VCAP_STATUS carries setup mode 0 and
             * that wait completes instead of timing out.
             */
            s->vcap_paused = (amb_dsp_ld(c + 4) == 0);
        } else if (code == CMD_DSP_SUSPEND_OPERATION_MODE) {
            /*
             * iav leaves PREVIEW/ENCODE by suspending the operation mode back
             * to IDLE -- not with SET_OPERATION_MODE(IDLE) but with this
             * SUSPEND command, which carries no mode byte.  iav_goto_idle then
             * polls the DSP-reported op_mode to reach IDLE (dsp_set_op_mode's
             * "Enter mode event").  Real iav.ko on silicon sees the part return
             * to idle and the transition succeeds; report IDLE here so the same
             * PREVIEW->IDLE goto_idle completes instead of timing out (the
             * emulated DSP otherwise stayed in CAMERA_RECORD and iav failed the
             * whole pipeline with EPERM).
             */
            s->op_mode = DSP_OP_MODE_IDLE;
            s->mode_commanded = true;
        }
        /*
         * Track per-stream encode start/stop (stream_id in byte 4).  Only a
         * stream the driver has actually started may be reported encoding --
         * reporting it earlier trips start_encode's "already encoding" check.
         */
        if (code == CMD_H264ENC_START) {
            s->enc_streams |= 1u << (amb_dsp_ld(c + 4) & 0x1f);
        } else if (code == CMD_H264ENC_STOP) {
            s->enc_streams &= ~(1u << (amb_dsp_ld(c + 4) & 0x1f));
        } else if (code == CMD_H264ENC_JPEG_SETUP) {
            uint32_t sid = amb_dsp_ld(c + 4) & 0x7;
            uint32_t wh = amb_dsp_ld(c + JS_WH_OFF);
            uint32_t base = amb_dsp_ld(c + JS_BITS_BASE_OFF);
            s->estr[sid].codec = AMB_CODEC_JPEG;
            s->estr[sid].w = wh & 0xffff;
            s->estr[sid].h = wh >> 16;
            /* The driver re-sends this every frame; only a new buffer resets
             * the write cursor, otherwise frames would all land on one slot. */
            if (base != s->estr[sid].bits_base) {
                s->estr[sid].bits_off = 0;
            }
            s->estr[sid].bits_base = base;
            s->estr[sid].bits_size = amb_dsp_ld(c + JS_BITS_SIZE_OFF);
            s->estr[sid].info_base = amb_dsp_ld(c + JS_INFO_BASE_OFF);
            s->estr[sid].info_size = amb_dsp_ld(c + JS_INFO_SIZE_OFF);
            if (s->debug) {
                fprintf(stderr, "amb-dsp: jpeg-setup s%u %ux%u bits=%08x/%x "
                        "info=%08x/%x\n", sid, s->estr[sid].w, s->estr[sid].h,
                        s->estr[sid].bits_base, s->estr[sid].bits_size,
                        s->estr[sid].info_base, s->estr[sid].info_size);
            }
        } else if (code == CMD_H264ENC_SETUP) {
            uint32_t sid = amb_dsp_ld(c + 4) & 0x7;
            uint32_t idc = amb_dsp_ld(c + 4);
            uint32_t wh = amb_dsp_ld(c + HS_WH_OFF);
            uint32_t base = amb_dsp_ld(c + HS_BITS_BASE_OFF);
            s->estr[sid].codec = AMB_CODEC_H264;
            s->estr[sid].profile = (idc >> 16) & 0xff;
            s->estr[sid].level = (idc >> 24) & 0xff;
            s->estr[sid].w = wh & 0xffff;
            s->estr[sid].h = wh >> 16;
            if (base != s->estr[sid].bits_base) {
                s->estr[sid].bits_off = 0;
            }
            s->estr[sid].bits_base = base;
            s->estr[sid].bits_size = amb_dsp_ld(c + HS_BITS_SIZE_OFF);
            s->estr[sid].info_base = amb_dsp_ld(c + HS_INFO_BASE_OFF);
            s->estr[sid].info_size = amb_dsp_ld(c + HS_INFO_SIZE_OFF);
            if (s->debug) {
                fprintf(stderr, "amb-dsp: h264-setup s%u %ux%u prof=%u lvl=%u "
                        "bits=%08x/%x info=%08x/%x\n", sid, s->estr[sid].w,
                        s->estr[sid].h, s->estr[sid].profile, s->estr[sid].level,
                        s->estr[sid].bits_base, s->estr[sid].bits_size,
                        s->estr[sid].info_base, s->estr[sid].info_size);
            }
        }
    }
    if (fresh) {
        fprintf(stderr, " (op_mode=%u)\n", s->op_mode);
    }
    s->last_seq[port] = seq;
    return seq;
}

/* Write the port's status message (slot 0); num_msgs event slots follow it. */
static void amb_dsp_reply(AmbDspState *s, hwaddr msg_port, uint32_t seq,
                          uint32_t num_msgs)
{
    struct dsp_status_msg m = {
        .msg_code = MSG_DSP_STATUS,
        .dsp_mode = s->op_mode,
        .prev_cmd_seq = seq,
        .num_msgs = num_msgs,
    };
    uint32_t w[7] = {
        cpu_to_le32(m.msg_code), cpu_to_le32(m.dsp_mode),
        cpu_to_le32(m.time_code), cpu_to_le32(m.prev_cmd_seq),
        cpu_to_le32(m.prev_cmd_echo), cpu_to_le32(m.prev_num_cmds),
        cpu_to_le32(m.num_msgs),
    };
    dma_memory_write(&address_space_memory, msg_port, w, sizeof(w),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * A VCAP video-capture report in slot 1 of a message port.  do_process_msg
 * advances iav's vcap frame counter (what start_encode's wait blocks on) only
 * when it sees this message with vcap_setup_mode == VIDEO and a non-zero
 * main_pic_luma_addr while the DSP is in CAMERA_RECORD -- so a captured frame
 * is exactly "a VCAP_STATUS whose luma address points at the picture".
 *
 * The low 3 bits of that setup-mode field are also the DSP "enc sub mode" iav
 * polls (dsp_set_enc_sub_mode) across a PREVIEW<->IDLE restart: VIDEO(1) while
 * capturing, IDLE(0) once iav has stopped the capture timer.  Report IDLE when
 * paused so that wait wakes -- and with it the frame counter stops advancing,
 * which is what a stopped timer means.
 */
static void amb_dsp_post_vcap(hwaddr msg_port, uint32_t luma, uint32_t chroma,
                              uint32_t w, uint32_t pitch, bool active)
{
    uint32_t v[DSP_MSG_SIZE / 4] = { 0 };

    v[0] = cpu_to_le32(MSG_VCAP_STATUS);
    v[VCAP_MSG_MODE_OFF / 4] = active ? cpu_to_le32(VCAP_VIDEO_MODE | (1u << 3))
                                      : 0;
    v[VCAP_MSG_LUMA_OFF / 4] = cpu_to_le32(luma);
    v[VCAP_MSG_CHROMA_OFF / 4] = cpu_to_le32(chroma);
    /* width (u16) and pitch (u16) share the word at byte 24. */
    v[VCAP_MSG_WIDTH_OFF / 4] = cpu_to_le32((w & 0xffff) |
                                            ((pitch & 0xffff) << 16));
    dma_memory_write(&address_space_memory, msg_port + DSP_MSG_SIZE, v,
                     sizeof(v), MEMTXATTRS_UNSPECIFIED);
}

/*
 * A "captured picture is ready in the YUV buffer" report, in the slot after
 * VCAP_STATUS.  handle_vcap_msg calls irq_update_bufcap -- which sets the
 * per-buffer YUV-ready flag and wakes the waitqueue iav_query_yuvdesc blocks
 * on -- only for VCAP code 0x88000003, and only when the preceding
 * 0x88000001 recorded setup mode VIDEO.  Without it a client's raw-YUV
 * readback (divinus' software AE/AWB) blocks the full 2 s then gets -EAGAIN.
 */
static void amb_dsp_post_vcap_done(hwaddr msg_port, uint32_t slot, uint32_t luma)
{
    uint32_t v[DSP_MSG_SIZE / 4] = { 0 };

    v[0] = cpu_to_le32(MSG_VCAP_FRAME_DONE);
    v[VCAP_MSG_LUMA_OFF / 4] = cpu_to_le32(luma);
    dma_memory_write(&address_space_memory, msg_port + (slot + 1) * DSP_MSG_SIZE,
                     v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
}

/*
 * An encoder-status report in slot 1 of a message port.  iav's start_encode
 * blocks (5 s) on a per-stream "is this stream encoding yet" flag that only
 * this message flips: handle_vdsp_msg takes stream_id from byte 4 and
 * enc_status from byte 7, and on STATUS_ENCODE wakes the start_encode waiter.
 */
static void amb_dsp_post_enc(hwaddr msg_port, uint32_t slot, uint32_t stream_id,
                             uint8_t codec, uint32_t ctr)
{
    uint32_t v[DSP_MSG_SIZE / 4] = { 0 };
    uint32_t stype = codec == AMB_CODEC_H264 ? 1 : 2;   /* iav_stream_type */

    v[0] = cpu_to_le32(MSG_H264ENC_STATUS);
    /* word1: stream_id:8 | stream_type:8 | stream_report_no:8 | enc_status:8 */
    v[1] = cpu_to_le32((stream_id & 0xff) | (stype << 8) |
                       (ENC_STATUS_ENCODE << 24));
    /* iav queues frames on the delta of the per-codec counter since the last
     * report: total_bits_info_ctr_h264 (W2) or _jpeg (W4). */
    v[(codec == AMB_CODEC_H264 ? ENC_STATUS_H264_CTR_OFF
                               : ENC_STATUS_JPEG_CTR_OFF) / 4] = cpu_to_le32(ctr);
    dma_memory_write(&address_space_memory,
                     msg_port + (slot + 1) * DSP_MSG_SIZE, v, sizeof(v),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * Produce one encoded frame for a stream: synthesise a YUV 4:2:0 picture,
 * JPEG-encode it with the shared encoder, drop the bitstream into the next
 * BSB slot and describe it with an enc_bits_info_t in the BSH fifo.  This is
 * the frame the DSP's microcode would have made; iav and its clients cannot
 * tell the difference, and with a dump dir it is also a real .jpg on the host.
 */
/* SMPTE-ish colour bars; a marker band marches down one step a frame. */
static const uint8_t amb_bar_y[8] = { 180, 162, 131, 112, 84, 65, 35, 16 };
static const uint8_t amb_bar_u[8] = { 128, 44, 156, 72, 184, 100, 212, 128 };
static const uint8_t amb_bar_v[8] = { 128, 142, 44, 58, 198, 212, 114, 128 };

static void amb_dsp_fill_luma(uint8_t *y, int stride, int w, int h,
                              uint32_t frame)
{
    uint32_t band = h ? (frame * 8) % h : 0;

    for (int r = 0; r < h; r++) {
        for (int c = 0; c < w; c++) {
            uint8_t l = amb_bar_y[c * 8 / w];
            if ((uint32_t)r >= band && (uint32_t)r < band + 8) {
                l = 235;                        /* the moving marker */
            }
            y[(size_t)r * stride + c] = l;
        }
    }
}

/*
 * Write the main capture buffer as an NV12 (4:2:0, interleaved chroma) picture
 * at the DSP address the VCAP report points iav's raw-YUV readback at.  The
 * same colour-bar pattern the encoder synthesises, so a client's software 3A
 * meters a real, moving image.  luma plane is pitch*h, chroma plane pitch*h/2.
 */
static void amb_dsp_write_yuv(hwaddr luma, uint32_t pitch, uint32_t w,
                              uint32_t h, uint32_t frame)
{
    g_autofree uint8_t *y = g_malloc((size_t)pitch * h);
    g_autofree uint8_t *uv = g_malloc((size_t)pitch * (h / 2));

    amb_dsp_fill_luma(y, pitch, w, h, frame);
    for (uint32_t r = 0; r < h / 2; r++) {
        for (uint32_t c = 0; c + 1 < pitch; c += 2) {
            int b = c < w ? (int)(c * 8 / w) : 0;
            uv[(size_t)r * pitch + c]     = amb_bar_u[b];
            uv[(size_t)r * pitch + c + 1] = amb_bar_v[b];
        }
    }
    dma_memory_write(&address_space_memory, luma, y, (size_t)pitch * h,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, luma + (hwaddr)pitch * h, uv,
                     (size_t)pitch * (h / 2), MEMTXATTRS_UNSPECIFIED);
}

static void amb_dsp_emit_frame(AmbDspState *s, uint32_t sid)
{
    uint32_t bits_base = s->estr[sid].bits_base, bits_size = s->estr[sid].bits_size;
    uint32_t info_base = s->estr[sid].info_base, info_size = s->estr[sid].info_size;
    int w = s->estr[sid].w ? s->estr[sid].w : ENC_W;
    int h = s->estr[sid].h ? s->estr[sid].h : ENC_H;
    uint32_t fn = s->estr[sid].frames;
    int pic_type;
    const char *ext;

    if (!s->estr[sid].codec || !bits_base || !info_base ||
        info_size < EBI_STEP || bits_size == 0) {
        return;     /* the driver has not told us where its buffers are yet */
    }
    if (w <= 0 || h <= 0 || w > ENC_MAX_DIM || h > ENC_MAX_DIM) {
        return;     /* a resolution we will not allocate a host buffer for */
    }
    /* 4:2:0 subsampling and the SPS's 2-sample crop units both need even
     * dimensions; the driver aligns to the encoder grid, so this only rejects
     * a malformed setup rather than silently coding the wrong size. */
    if (s->estr[sid].codec == AMB_CODEC_H264 && ((w | h) & 1)) {
        return;
    }

    g_autoptr(GByteArray) bs = g_byte_array_new();

    if (s->estr[sid].codec == AMB_CODEC_H264) {
        int mbw = (w + 15) / 16, mbh = (h + 15) / 16;
        int pw = mbw * 16, ph = mbh * 16;        /* padded to whole MBs */
        g_autofree uint8_t *y = g_malloc((size_t)pw * ph);
        g_autofree uint8_t *uv = g_malloc((size_t)pw * (ph / 2));
        H264Enc e;
        int qp = 30;

        amb_dsp_fill_luma(y, pw, pw, ph, fn);
        for (int r = 0; r < ph / 2; r++) {        /* NV12 interleaved chroma */
            for (int c = 0; c < pw / 2; c++) {
                int b = (2 * c) * 8 / pw;
                uv[(size_t)r * pw + 2 * c]     = amb_bar_u[b];
                uv[(size_t)r * pw + 2 * c + 1] = amb_bar_v[b];
            }
        }
        /* Fresh SPS+PPS+IDR every frame: each one decodes on its own.  Write
         * the Baseline parameter sets with the shared generator, then parse
         * them back so h264_enc_frame()'s slice is consistent with them. */
        uint8_t sets[128];
        size_t sn = h264_enc_baseline_headers(w, h, sets);
        memset(&e, 0, sizeof e);
        h264_enc_parse_headers(&e, sets, sn);
        g_byte_array_append(bs, sets, sn);
        size_t cap = (size_t)mbw * mbh * 512 + 256;
        g_autofree uint8_t *slice = g_malloc(cap);
        size_t n = h264_enc_frame(&e, true, qp, mbw, mbh, y, pw, uv, pw,
                                  slice, cap);
        if (n > cap) {
            return;     /* truncated slice: drop the frame rather than ship it */
        }
        g_byte_array_append(bs, slice, n);
        pic_type = FRAME_TYPE_IDR;
        ext = "h264";
    } else {
        int cw = (w + 1) / 2, ch = (h + 1) / 2;
        g_autofree uint8_t *y = g_malloc((size_t)w * h);
        g_autofree uint8_t *cb = g_malloc((size_t)cw * ch);
        g_autofree uint8_t *cr = g_malloc((size_t)cw * ch);
        bool overflow;

        amb_dsp_fill_luma(y, w, w, h, fn);
        for (int r = 0; r < ch; r++) {
            for (int c = 0; c < cw; c++) {
                int b = (2 * c) * 8 / w;
                cb[(size_t)r * cw + c] = amb_bar_u[b];
                cr[(size_t)r * cw + c] = amb_bar_v[b];
            }
        }
        jpeg_enc_header(bs, w, h, amb_dsp_ql, amb_dsp_qc);
        /* Baseline entropy coding can expand a pathological block past the
         * pixel count; size for that and bail if the encoder still overflows,
         * rather than publishing a truncated scan with no EOI. */
        size_t cap = (size_t)w * h * 2 + 1024;
        g_autofree uint8_t *scan = g_malloc(cap);
        size_t n = jpeg_enc_scan(y, w, cb, cr, cw, w, h, amb_dsp_ql, amb_dsp_qc,
                                 true, scan, cap, &overflow);
        if (overflow) {
            return;
        }
        g_byte_array_append(bs, scan, n);
        pic_type = FRAME_TYPE_JPG;
        ext = "jpg";
    }

    uint32_t len = bs->len;
    uint32_t info_ents = info_size / EBI_STEP;

    /* A frame that cannot fit the BSB at all would have its DMA write run off
     * the end of the driver's buffer; drop it instead. */
    if (len > bits_size) {
        return;
    }

    /* Place the bitstream in the BSB ring the driver gave us, 32B-aligned. */
    if (s->estr[sid].bits_off + len > bits_size) {
        s->estr[sid].bits_off = 0;
    }
    uint32_t off = s->estr[sid].bits_off;
    hwaddr addr = bits_base + off;
    dma_memory_write(&address_space_memory, addr, bs->data, len,
                     MEMTXATTRS_UNSPECIFIED);
    s->estr[sid].bits_off = ROUND_UP(off + len, 32);

    uint32_t e[EBI_STEP / 4] = { 0 };
    e[EBI_W_ID]       = cpu_to_le32((sid & 0xff) | (JPEG_QUALITY << 8));
    e[EBI_W_PICTYPE]  = cpu_to_le32(pic_type);
    e[EBI_W_FRAMENUM] = cpu_to_le32(fn);
    e[EBI_W_PTS]      = cpu_to_le32(fn);   /* 1 tick/frame is fine */
    e[EBI_W_START]    = cpu_to_le32((uint32_t)addr);
    e[EBI_W_SIZE]     = cpu_to_le32(len & 0xffffff);
    /* 64-bit arithmetic so a ring index near the 4 GiB mark cannot wrap the
     * descriptor write down into low guest RAM; info_base + info_size was the
     * driver's own allocation, and the index stays inside it. */
    dma_memory_write(&address_space_memory,
                     (hwaddr)info_base + (uint64_t)(fn % info_ents) * EBI_STEP,
                     e, sizeof(e), MEMTXATTRS_UNSPECIFIED);

    if (s->dump_dir) {
        g_autofree char *path = g_strdup_printf("%s/frame-s%u-%05u.%s",
                                                s->dump_dir, sid, fn, ext);
        g_file_set_contents(path, (const char *)bs->data, bs->len, NULL);
    }

    s->estr[sid].frames++;
}

static void amb_dsp_tick(void *opaque)
{
    AmbDspState *s = opaque;
    hwaddr cmd = amb_dsp_cmd_buf(s);
    hwaddr msg = amb_dsp_msg_buf(s);
    uint32_t seq_gen, seq_vcap;

    if (!s->chip_written) {
        amb_dsp_st(amb_dsp_ucode(s) + DSP_CHIP_ID_OFFSET, AMBA_CHIP_ID_S3L_66);
        s->chip_written = true;
    }

    /*
     * The real DSP reports INIT (255) once while the microcode boots, then
     * drops to IDLE.  Mirror that: hold INIT for the first few heartbeats so
     * the driver sees the part come up, then report IDLE unless it has asked
     * for a specific mode -- otherwise iav never leaves its own INIT state and
     * never allocates the default resources a client queries.
     */
    if (!s->mode_commanded && s->op_mode == DSP_OP_MODE_INIT &&
        ++s->boot_ticks >= 3) {
        s->op_mode = DSP_OP_MODE_IDLE;
    }

    if (s->debug && !s->init_dumped) {
        hwaddr id = amb_dsp_ucode(s) + DSP_INIT_DATA_OFFSET;
        if (amb_dsp_ld(id + INIT_DEFBIN_ADDR)) {
            fprintf(stderr, "amb-dsp: init_data@%08x cmd_gen=%08x msg_gen=%08x "
                    "cmd_vcap=%08x msg_vcap=%08x chip_id=%08x\n", (uint32_t)id,
                    amb_dsp_ld(id + INIT_CMD_GEN_DADDR),
                    amb_dsp_ld(id + INIT_MSG_GEN_DADDR),
                    amb_dsp_ld(id + INIT_CMD_VCAP_DADDR),
                    amb_dsp_ld(id + INIT_MSG_VCAP_DADDR),
                    amb_dsp_ld(id + INIT_CHIP_ID_DADDR));
            s->init_dumped = true;
        }
    }

    /*
     * General and vcap ports are the two 4 KiB halves of each buffer.  The
     * driver flushes its queued commands into the command buffer and reads
     * our replies only from inside the vcap/vdsp interrupt handler, so the
     * interrupt has to keep ticking for the command handshake to run at all --
     * it is the DSP's heartbeat, not an event we can gate on a command we have
     * not been handed yet.
     */
    seq_gen = amb_dsp_service_port(s, cmd, 0);
    seq_vcap = amb_dsp_service_port(s, cmd + DSP_PORT_SIZE, 1);

    /*
     * Once the driver has us recording, report a captured frame every
     * heartbeat on the vcap port.  The luma address is the DSP's own scratch
     * buffer from the init data (a synthesised picture lands there in the
     * encode step); any non-zero address is enough to advance the counter.
     */
    bool capturing = (s->op_mode == DSP_OP_MODE_CAMERA_RECORD);
    bool vcap_frame_done = false;
    if (capturing) {
        hwaddr id = amb_dsp_ucode(s) + DSP_INIT_DATA_OFFSET;
        uint32_t luma = amb_dsp_ld(id + INIT_DSPBUF_DADDR);
        uint32_t scratch = amb_dsp_ld(id + INIT_DSPBUF_SIZE);
        if (!luma) {
            luma = s->iavmem_base;
            scratch = 0;            /* fallback address: no advertised size */
        }
        /*
         * Publish a raw-YUV picture -- so a client's software 3A can meter it
         * -- only once the main buffer geometry is known from CMD_VCAP_SETUP
         * and sane.  The encoder's dimension limit caps the host allocation
         * here too; an unset/oversized/odd size just leaves the readback
         * unarmed (the query times out, as before) rather than fabricating a
         * geometry or allocating gigabytes.  NV12 pitch is the width rounded to
         * the DSP's 32-pixel grid; the chroma plane follows the luma plane and
         * both must fit the scratch buffer and iavmem, or the DMA would overrun.
         */
        uint32_t mw = s->main_w, mh = s->main_h, mpitch = 0, mchroma = 0;
        bool have_yuv = !s->vcap_paused && mw && mh &&
                        mw <= ENC_MAX_DIM && mh <= ENC_MAX_DIM &&
                        !((mw | mh) & 1);
        if (have_yuv) {
            mpitch = ROUND_UP(mw, 32);
            uint64_t need = (uint64_t)mpitch * mh + (uint64_t)mpitch * (mh / 2);
            if (luma >= s->iavmem_base &&
                (uint64_t)luma + need <= s->iavmem_end &&
                (!scratch || need <= scratch)) {
                mchroma = luma + mpitch * mh;
                amb_dsp_write_yuv(luma, mpitch, mw, mh, s->capture_frames);
            } else {
                have_yuv = false;   /* would run off the scratch buffer */
            }
        }
        amb_dsp_post_vcap(msg + DSP_PORT_SIZE, luma, mchroma,
                          have_yuv ? mw : 0, have_yuv ? mpitch : 0,
                          !s->vcap_paused);
        if (have_yuv) {
            /* Follow the status with a frame-done report so iav's raw-YUV
             * readback (QUERY_DESC yuv) wakes instead of timing out, in vcap
             * slot 2. */
            amb_dsp_post_vcap_done(msg + DSP_PORT_SIZE, 1, luma);
            vcap_frame_done = true;
        }
        if (!s->vcap_paused) {
            s->capture_frames++;    /* animates the picture per captured frame */
        }
    }

    /*
     * Report every started stream as encoding on the general port.  One event
     * slot per stream; the first stream follows the status at slot 1.
     */
    uint32_t enc_msgs = 0;
    for (uint32_t sid = 0; sid < 8; sid++) {
        if (s->enc_streams & (1u << sid)) {
            /* A stopped capture timer means no new captured picture to encode,
             * so hold frame production (and the per-stream counter) while the
             * vcap is paused -- matching the IDLE setup mode VCAP_STATUS then
             * reports.  The encode-status handshake still goes out with the
             * unchanged counter (delta 0), so iav keeps the stream marked
             * encoding and simply sees no new frames until capture resumes. */
            if (!s->vcap_paused) {
                amb_dsp_emit_frame(s, sid);
            }
            /* The per-codec counter iav diffs must count only this stream's
             * frames; emit_frame advanced estr[sid].frames iff it shipped one. */
            amb_dsp_post_enc(msg, enc_msgs, sid, s->estr[sid].codec,
                             s->estr[sid].frames);
            enc_msgs++;
        }
    }

    amb_dsp_reply(s, msg, seq_gen, enc_msgs);
    /* vcap port: the status (slot 1), plus a frame-done (slot 2) when we
     * published a captured picture this heartbeat. */
    uint32_t vcap_msgs = capturing ? (vcap_frame_done ? 2 : 1) : 0;
    amb_dsp_reply(s, msg + DSP_PORT_SIZE, seq_vcap, vcap_msgs);

    qemu_irq_pulse(s->vdsp_irq);
    qemu_irq_pulse(s->vcap_irq);
    if (enc_msgs) {
        qemu_irq_pulse(s->venc_irq);    /* drives iav's encoder frame callback */
    }

    timer_mod(&s->heartbeat,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / AMB_DSP_HZ);
}

static void amb_dsp_reset(DeviceState *dev)
{
    AmbDspState *s = AMB_DSP(dev);

    s->chip_written = false;
    s->op_mode = DSP_OP_MODE_INIT;
    s->boot_ticks = 0;
    s->mode_commanded = false;
    s->vcap_paused = false;
    s->main_w = s->main_h = 0;
    s->capture_frames = 0;
    s->enc_streams = 0;
    memset(s->estr, 0, sizeof(s->estr));
    s->last_seq[0] = s->last_seq[1] = 0;
    timer_mod(&s->heartbeat,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / AMB_DSP_HZ);
}

static void amb_dsp_realize(DeviceState *dev, Error **errp)
{
    AmbDspState *s = AMB_DSP(dev);

    if (s->iavmem_end <= s->iavmem_base + DSP_UCODE_SIZE) {
        error_setg(errp, "amb-dsp: iavmem region too small for the DSP area");
        return;
    }
    s->debug = getenv("AMB_DSP_DEBUG") != NULL;
    if (s->debug) {
        fprintf(stderr, "amb-dsp: iavmem [%08x,%08x) ucode=%08x cmd=%08x msg=%08x\n",
                s->iavmem_base, s->iavmem_end, (uint32_t)amb_dsp_ucode(s),
                (uint32_t)amb_dsp_cmd_buf(s), (uint32_t)amb_dsp_msg_buf(s));
    }
    timer_init_ns(&s->heartbeat, QEMU_CLOCK_VIRTUAL, amb_dsp_tick, s);
}

static void amb_dsp_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    AmbDspState *s = AMB_DSP(obj);

    sysbus_init_irq(sbd, &s->vdsp_irq);
    sysbus_init_irq(sbd, &s->vcap_irq);
    sysbus_init_irq(sbd, &s->venc_irq);
}

static const Property amb_dsp_props[] = {
    DEFINE_PROP_UINT32("iavmem-base", AmbDspState, iavmem_base, 0),
    DEFINE_PROP_UINT32("iavmem-end", AmbDspState, iavmem_end, 0),
    DEFINE_PROP_STRING("dump-dir", AmbDspState, dump_dir),
};

static const VMStateDescription vmstate_amb_dsp_stream = {
    .name = "amb-dsp/stream",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bits_base, AmbDspStream),
        VMSTATE_UINT32(bits_size, AmbDspStream),
        VMSTATE_UINT32(info_base, AmbDspStream),
        VMSTATE_UINT32(info_size, AmbDspStream),
        VMSTATE_UINT32(bits_off, AmbDspStream),
        VMSTATE_UINT32(frames, AmbDspStream),
        VMSTATE_UINT16(w, AmbDspStream),
        VMSTATE_UINT16(h, AmbDspStream),
        VMSTATE_UINT8(codec, AmbDspStream),
        VMSTATE_UINT8(profile, AmbDspStream),
        VMSTATE_UINT8(level, AmbDspStream),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_amb_dsp = {
    .name = "amb-dsp",
    .version_id = 3,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(heartbeat, AmbDspState),
        VMSTATE_BOOL(chip_written, AmbDspState),
        VMSTATE_UINT32(op_mode, AmbDspState),
        VMSTATE_UINT32(boot_ticks, AmbDspState),
        VMSTATE_BOOL(mode_commanded, AmbDspState),
        VMSTATE_BOOL_V(vcap_paused, AmbDspState, 2),
        VMSTATE_UINT16_V(main_w, AmbDspState, 3),
        VMSTATE_UINT16_V(main_h, AmbDspState, 3),
        VMSTATE_UINT32_V(capture_frames, AmbDspState, 3),
        VMSTATE_UINT32(enc_streams, AmbDspState),
        VMSTATE_UINT32_ARRAY(last_seq, AmbDspState, 2),
        VMSTATE_STRUCT_ARRAY(estr, AmbDspState, 8, 1, vmstate_amb_dsp_stream,
                             AmbDspStream),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_dsp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = amb_dsp_realize;
    device_class_set_legacy_reset(dc, amb_dsp_reset);
    dc->vmsd = &vmstate_amb_dsp;
    device_class_set_props(dc, amb_dsp_props);
}

static const TypeInfo amb_dsp_info = {
    .name = TYPE_AMB_DSP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbDspState),
    .instance_init = amb_dsp_init,
    .class_init = amb_dsp_class_init,
};

static void amb_dsp_register(void)
{
    type_register_static(&amb_dsp_info);
}
type_init(amb_dsp_register)
