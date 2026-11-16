/*
 * Axera AX630C VeriSilicon VCMD command-queue engines: VENC (VC8000E),
 * JENC and VDEC all share the same VCMD front-end protocol (confirmed
 * from the open VC8000E driver: eswin/vc8000_vcmd_driver.c, dual
 * MIT/GPL VeriSilicon lineage, and its register field table
 * eswin/vcmdregistertable.h):
 *
 *   - HW_ID, byte offset 0x0, bits[31:16], read-only (VCMDREG
 *     HWIF_VCMD_HW_ID in vcmdregistertable.h): the real guest kernel
 *     module's venc_probe() hardware-detection scan (confirmed live via
 *     gdbstub register trace against the real deployed ax_venc.ko,
 *     not the mismatched SDK reference build) maps this exact MMIO
 *     block and requires HW_ID == 0x4342 before it will create
 *     /dev/ax_venc at all -- without it, venc_probe logs "hantrovcmd:
 *     NO ANY HW found!!" and AX_VENC_Init's later open("/dev/ax_venc")
 *     fails outright, so this single missing read-only field was
 *     blocking the entire VENC init chain, not anything deeper in the
 *     command-queue handshake below. Bits[15:0] are HWIF_VCMD_HW_VERSION
 *     ("[15:12]-major [11:8]-minor [7:0]-build", RO) -- not checked by
 *     this probe gate (which only cares about bits[31:16]), but IS a
 *     real, named silicon revision value elsewhere: the open driver's
 *     own hardware-detection probe (vc8000_vcmd_driver.c, ~line 4193)
 *     additionally rejects any hwid < HW_ID_1_2_1 (0x43421201) as "HW
 *     not found", and that same value is read back by userspace via an
 *     ioctl (HANTRO_IOCH_GET_VCMD_PARAMETER / vcmd_hw_version_id) that
 *     a closed-source profile/capability table could plausibly key on.
 *     An earlier placeholder here (0x43421000, the bare example value
 *     from vcmdregistertable.h's own comment) does not correspond to
 *     any real chip revision; using the real named HW_ID_1_2_1 constant
 *     instead is being tried as a fix for AX_VENC_Init's still-open
 *     NOT_INIT (0x80070212) failure, not just cosmetic.
 *   - WORK_STATE, byte offset 0x3C, bits[2:0], read-only:
 *     0 IDLE, 1 WORK, 2 STALL, 3 PEND, 4 ABORT.
 *   - CONTROL, byte offset 0x40 (VCMD_REGISTER_CONTROL_OFFSET), bit 0 is
 *     HWIF_VCMD_START_TRIGGER: write 1 to fetch and execute the queued
 *     command buffer, write 0 to abort.
 *   - INT_STATUS, byte offset 0x44 (VCMD_REGISTER_INT_STATUS_OFFSET),
 *     bit 0 is HWIF_VCMD_IRQ_ENDCMD: set by hardware on completion, and
 *     the guest ISR (hantrovcmd_isr) acks by writing the SAME value it
 *     just read back -- a plain write-1-to-clear, not a distinct ack
 *     register.
 *   - INT_CTL, byte offset 0x48 (VCMD_REGISTER_INT_CTL_OFFSET), bit 0 is
 *     HWIF_VCMD_IRQ_ENDCMD_EN: the interrupt mask.
 *
 * The real engine fetches a guest-built linked list of command-buffer
 * nodes (bi_list_node / cmdbuf_obj in the VCMD core) describing the
 * actual encode/decode job -- SPS/PPS, rate control, frame buffer
 * addresses. The cmdbuf's own opcode stream (WREG/RREG/JMP/STALL) is
 * decoded for real (axera_vcmd_exec_cmdbuf below); the per-job register
 * fields it writes (core_reg[], the VC8000E core's own register file --
 * see the ENC_REG_* map further down) are in turn bridged to the
 * project's shared Hantro software encoder library to produce a REAL
 * H.264/HEVC/JPEG bitstream (axera_vcmd_run_encode_job), not just a
 * completed handshake. As of this writing the real closed driver has
 * only been observed submitting the short capability-probe cmdbufs
 * (RREG-only, no WREG, no HWIF_ENC_E) -- an actual encode job never
 * reaches this device model during a normal boot because the app-level
 * ISP/VPSS pixel pipeline never hands CEncode a real frame to submit
 * (a separate, already-documented limitation -- see kaeru
 * axera-core-goal-achieved-both-platforms-streaming-blocked-on-isp-silicon).
 * The bridge itself is validated independently via a synthetic but
 * realistic cmdbuf+MMIO sequence driven over gdbstub, producing bytes a
 * real decoder (ffprobe/ffmpeg, PIL) accepts for all three codecs.
 *
 * Completion fires from a timer, not synchronously inside the CONTROL
 * write: the guest's wait_cmdbuf_ready() (and the VCMD core generally)
 * is built around being woken by a real interrupt after the doorbell
 * write returns, the same race the project's other codec models
 * (sstar-vpu.c, nvt-h26xdec.c) avoid the same way.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/misc/axera-vcmd.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hantro-h264.h"
#include "hevc-enc.h"
#include "jpeg-enc.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraVcmdState, AXERA_VCMD)

#define VCMD_MMIO_SIZE     0x10000
#define HW_ID_OFF          0x00
/*
 * VCMD front-end version 1.5.0, the value real AX630C silicon reports
 * (same AX620E die). It matters: the driver picks its completion protocol
 * from it. At <= 1.0.C (0x43421001) it expects an INT opcode in every
 * command buffer and completes through IRQ_INTCMD; above that it
 * programs CMDBUF_EXECUTING_ID, enables IRQ_JMPP/JMPD and reads the
 * finished buffer's ID back from the status memory the buffer's own
 * trailing RREG of the VCMD registers fills. The closed userspace encoder
 * builds the latter kind of buffer (no INT opcode, JMP-terminated, RREG of
 * VCMD regs 0-26 last), so with 1.0.0 no encode job was ever reported
 * complete and every VENC worker blocked in hantrovcmd_ioctl forever.
 */
#define HW_ID_VALUE        0x43421500u
#define WORK_STATE_OFF     0x3c
#define CONTROL_OFF        0x40
#define INT_STATUS_OFF     0x44
#define INT_CTL_OFF        0x48
/* HWIF_VCMD_EXECUTING_CMD_ADDR / _MSB / HWIF_VCMD_EXE_CMDBUF_LENGTH
 * (vcmdregistertable.h offsets 80/84/88): the bus address (64-bit,
 * split lo/hi) and length (64-bit units) of the command buffer the
 * driver wants fetched and executed next. Read-write: the driver
 * programs these before every CONTROL/START_TRIGGER doorbell. */
#define CMD_ADDR_LO_OFF    0x50
#define CMD_ADDR_HI_OFF    0x54
#define CMD_LENGTH_OFF     0x58
#define EXE_CMDBUF_COUNT_OFF 0x0c
#define RDY_CMDBUF_COUNT_OFF 0x60
#define EXECUTING_ID_OFF   0x68
#define VCMD_REG_WINDOW    0x100   /* front-end registers kept as storage */
#define START_TRIGGER      BIT(0)
#define IRQ_ENDCMD         BIT(0)
/* vcmdregistertable.h: JMPD "triggered by JMP command directly" (a JMP
 * with IE set), JMPP "triggered by JMP command when hw goes to PEND
 * state" (a JMP with RDY clear -- the end of the linked chain, with or
 * without IE). The encoder's buffers end in a plain JMP, so JMPP is the
 * only interrupt that ever reports them done. */
#define IRQ_JMPD           BIT(6)
#define IRQ_JMPP           BIT(7)
#define JMP_IE             BIT(25)
#define JMP_RDY            BIT(26)
#define VCMD_MAX_CHAIN     64
#define WORK_STATE_IDLE    0
#define WORK_STATE_WORK    1
#define WORK_STATE_PEND    3

/*
 * VCMD opcode stream (vcmd_abi.h, a from-scratch reverse-engineered ABI
 * doc for this exact driver family, cross-checked live against the
 * REAL closed Axera driver's own cmdbuf traffic: captured buffers are
 * byte-for-byte RREG_HDR(512, SUBMODULE_MAIN_ADDR) + dst-lo/hi/pad +
 * JMP_TAIL_WORD + target-lo/hi/pad, matching this decode exactly).
 * Opcode is bits [31:27].
 */
#define VCMD_OP_WREG        0x01u
#define VCMD_OP_END         0x02u
#define VCMD_OP_NOP         0x03u
#define VCMD_OP_RREG        0x16u
#define VCMD_OP_JMP         0x19u
#define VCMD_OP_STALL       0x09u
#define VCMD_OP_CLRINT      0x1au
#define VCMD_WREG_FIX       BIT(26)

/* The VC8000E core's own register file (swreg0..swregN), ASIC byte
 * address SUBMODULE_MAIN_ADDR (0x1000) + 4*n inside this same MMIO
 * window (vcmd_abi.h) -- a totally separate register space from the
 * VCMD front-end's own 5 registers above, written/read only through
 * WREG/RREG opcodes inside an executed command buffer, never directly
 * over MMIO. This is where the real encode job parameters (and
 * whatever capability data userspace probes before any job has run)
 * live. 512 is the real driver's own query width (captured RREG(512,
 * ...) above). */
#define CORE_REG_BASE       0x1000u
#define CORE_REG_COUNT      512

/* core_reg[0] (ASIC byte addr CORE_REG_BASE+0, i.e. the VC8000E core's
 * own swreg0) is the encoder *core's* product/config ID -- a different
 * identity from the VCMD front-end's own HW_ID_OFF register above.
 * Confirmed live (gdbstub trace into the real App's capability-probe
 * path, functions at 0x8247b8/0x82e3a0/0x82e438 in the stripped App
 * binary): the App reads this register via its mmap() of this same
 * MMIO window, compares it against named thresholds from Axera's own
 * vendor header (vc8000_driver.h: ENC_HW_ID1=0x48320100,
 * ENC_HW_ID2=0x80006000) to decide how many extra swreg fields encode
 * real capability data, and ALSO uses this same raw value as a lookup
 * key into a small (4-slot) runtime cache of already-probed core
 * profiles. Leaving this register at 0 (the memset default below) is
 * actively wrong, not just incomplete: slot 0 of that cache is ALSO
 * zero-initialized .bss, so a swreg0 of 0 spuriously collides with an
 * empty, never-populated cache slot. The lookup reports a false "cache
 * hit" and returns a pointer to that still-all-zero slot instead of
 * falling through to the code path that actually derives capability
 * fields from the core registers and fills the slot in -- which is
 * exactly why the final capability descriptor the App consults
 * (confirmed elsewhere to live at a static .bss address) was observed
 * completely zeroed even though every ioctl/mmap in the probe chain
 * reports success. Any nonzero value avoids that collision; using one
 * above ENC_HW_ID2 (and above the App's second, unnamed threshold
 * ENC_HW_ID2+0xf) additionally selects the fully-populated capability
 * path instead of the reduced/basic one. */
#define ENC_CORE_HW_ID      0x80006010u

/* core_reg[80] (ASIC byte addr CORE_REG_BASE+320, swreg80) is the VC8000E
 * core's own hardware-feature-support word -- documented bit-for-bit in
 * Axera's own vendor header (vc8000_driver.h: `struct swreg80_s`,
 * DUMP_HARDWARE_FEATURES union): bits[12:0]=sw_enc_HWMaxVideoWidth,
 * [14:13]=HWBusWidth, [15]=HWJPEGSupport, [16]=HWTU32Support,
 * [17]=HWRFCSupport, [18]=HWProgRdoSupport, [19]=HWLineBufSupport,
 * [20]=HWCavlcSupport, [23:21]=HWBus, [24]=HWMain10Support,
 * [25]=HWDenoiseSupport, [26]=HWVp9Support, [27]=HWHevcSupport,
 * [28]=HWRgbSupport, [29]=HWBFrameSupport, [30]=HWScalingSupport,
 * [31]=HWH264Support.
 *
 * Confirmed live (gdbstub trace into the real App's capability-probe
 * path): 0x82e3a0 copies this exact register (read through the mmap'd
 * core-register window) into byte offset 12 of the same software
 * descriptor struct whose offset-0 word swreg0 feeds into -- i.e. this
 * is the SAME descriptor, a different field of it -- and 0x82e438's
 * populate block extracts bit 15 of it (HWJPEGSupport) into the
 * descriptor field the App's channel-creation path requires to be
 * nonzero (0x829720: `cbz w1, <fail>`) before AX_VENC_Init will
 * succeed. Leaving this register at 0 (the memset default) means every
 * capability bit reports "unsupported", including JPEG, which is
 * exactly the one this specific check needs -- confirmed to be the
 * actual, direct cause of AX_VENC_Init's long-standing NOT_INIT
 * (0x80070212) failure, not a proprietary/undocumented bit (unlike the
 * earlier "missing register, not decodable" conclusion this supersedes
 * -- the vendor header documents this one by name). Value chosen:
 * HWMaxVideoWidth=4096, HWJPEGSupport=1, HWCavlcSupport=1,
 * HWHevcSupport=1, HWH264Support=1 (the codecs/features the real App
 * actually exercises: xmcap_video_encode.c for H.264/HEVC,
 * xmcap_jpeg_encode.c for JPEG), everything else left 0 pending
 * evidence either way. */
#define ENC_SWREG80_FEATURES 0x88109000u

/*
 * Per-job encode registers inside core_reg[] (core_reg[N] == swregN),
 * from VeriSilicon's own public SW/HW interface tables
 * (encswhwregisters.h / registertable.h for this exact VC8000E family --
 * the same vendor-documented register file this model already hosts at
 * core_reg[], just the per-frame-job fields rather than the
 * capability-probe ones above). Confirmed structurally live: a
 * AXERA_VCMD_DEBUG_WREG cmdbuf trace against the real closed driver shows
 * its only two START_TRIGGER doorbells so far are the short (8-word)
 * RREG capability probes already modelled; no real encode job has yet
 * been observed because the app-level capture pipeline never hands
 * CEncode a real frame to submit (the separate, already-documented
 * "blocked on ISP silicon" limitation -- see kaeru
 * axera-core-goal-achieved-both-platforms-streaming-blocked-on-isp-silicon).
 * This bridge is therefore grounded in the vendor's own published
 * register semantics (not a live-observed real job, which does not
 * currently occur) and validated by directly driving the VCMD MMIO
 * window the same way a real job's cmdbuf would, via gdbstub.
 *
 * swreg4 bits[31:29]: encoding mode, 1=hevc 2=h264 4=jpeg.
 * swreg5: bits[31:22] width/[21:11] height (unit 8px, h264/hevc), or
 *   bits[31:20] width/[19:8] height (unit 8px, jpeg); bits[2:1] frame
 *   coding type (1=I 0=P 2=B); bit0 HWIF_ENC_E, the real "start
 *   encoding" trigger -- HW clears it itself when the picture finishes.
 * swreg7 bits[31:26]: initial QP.
 * swreg8/9: output stream base address / buffer limit (HW overwrites
 *   swreg9 with the actual bytes produced on completion).
 * swreg12/13/14: input luma / Cb / Cr (semi-planar: Cb only) base
 *   addresses.
 * swreg38 bits[31:28]: input pixel format, 0=planar 1=semi-planar(NV12).
 * swreg210/211 bits[31:12]: input luma/chroma byte stride.
 * swreg1 bits: HWIF_ENC_IRQ(0) / FRAME_RDY_STATUS(2), the core's own
 *   completion status (distinct from the VCMD front-end's ENDCMD above).
 */
#define ENC_REG_IRQ             1
#define   ENC_IRQ_BIT           BIT(0)
#define   ENC_FRAME_RDY_BIT     BIT(2)
#define ENC_REG_MODE            4
#define   ENC_MODE_SHIFT        29
#define   ENC_MODE_MASK         0x7u
#define   ENC_MODE_HEVC         1
#define   ENC_MODE_H264         2
#define   ENC_MODE_JPEG         4
#define ENC_REG_GEOM            5
#define   ENC_E_BIT             BIT(0)
#define   ENC_CODING_TYPE_SHIFT 1
#define   ENC_CODING_TYPE_MASK  0x3u
#define   ENC_CODING_TYPE_I     1
#define   ENC_PIC_W_SHIFT       22
#define   ENC_PIC_W_MASK        0x3ffu
#define   ENC_PIC_H_SHIFT       11
#define   ENC_PIC_H_MASK        0x3ffu
#define   ENC_JPEG_W_SHIFT      20
#define   ENC_JPEG_W_MASK       0xfffu
#define   ENC_JPEG_H_SHIFT      8
#define   ENC_JPEG_H_MASK       0xfffu
#define ENC_REG_QP              7
#define   ENC_PIC_INIT_QP_SHIFT 26
#define   ENC_PIC_INIT_QP_MASK  0x3fu
#define ENC_REG_OUT_BASE        8
#define ENC_REG_OUT_LIMIT       9
#define ENC_REG_IN_Y            12
#define ENC_REG_IN_CB           13
#define ENC_REG_IN_CR           14
#define ENC_REG_IN_FORMAT       38
#define   ENC_IN_FMT_SHIFT      28
#define   ENC_IN_FMT_MASK       0xfu
#define ENC_REG_IN_LU_STRIDE    210
#define ENC_REG_IN_CH_STRIDE    211
#define   ENC_STRIDE_SHIFT      12

/* Standard IJG baseline quantisation tables (ITU-T T.81 Annex K.1),
 * row-major/natural order as jpeg_enc_header/jpeg_enc_scan want. There is
 * no per-coefficient quant-table register in this ASIC generation (unlike
 * the older Hantro H1 modelled in xm-h1.c) -- VC8000E derives its table
 * internally from a quality-like parameter, so this model approximates
 * that with the same public-domain base tables libjpeg scales. */
static const uint8_t jpeg_base_luma[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
};
static const uint8_t jpeg_base_chroma[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
};

static void jpeg_quant_table(int quality, const uint8_t base[64],
                              uint8_t out[64])
{
    int scale = quality < 50 ? 5000 / MAX(quality, 1) : 200 - quality * 2;
    int i;

    for (i = 0; i < 64; i++) {
        int v = (base[i] * scale + 50) / 100;
        out[i] = CLAMP(v, 1, 255);
    }
}

/* The real engine takes real time to fetch/execute a command list; a
 * short but nonzero delay keeps this observably async without slowing
 * boots down. */
#define AXERA_VCMD_LATENCY_NS (1 * SCALE_MS)

#define AXERA_VCMD_STREAMS 4

struct AxeraVcmdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer completion_timer;

    uint32_t int_status;
    uint32_t int_ctl;
    uint32_t work_state;
    uint32_t cmd_addr_lo;
    uint32_t cmd_addr_hi;
    uint32_t cmd_length;
    uint32_t vreg[VCMD_REG_WINDOW / 4]; /* other front-end registers */
    uint32_t exe_count;
    uint32_t pending_status;            /* raised when the run completes */
    hwaddr pend_jmp;                    /* parked JMP (RDY=0), or 0 */
    uint32_t core_reg[CORE_REG_COUNT];

    /* Encode-job bridge state, one context per stream: persists across
     * jobs so inter (P-frame) coding chains correctly from one job of a
     * stream to its next. One VCMD core serves several streams at once
     * (the camera's 3200x1800 main and 800x448 sub channel interleave
     * job by job), so a single shared context made every P-frame
     * reference the other stream's picture. */
    struct AxeraVcmdStream {
        bool used;
        uint32_t mode, w, h;
        uint64_t last_job;
        H264Enc h264;
        HevcEncRef *hevc_ref;
        uint32_t hevc_poc;
        uint32_t frames;
    } streams[AXERA_VCMD_STREAMS];
    uint32_t enc_jobs;
};

typedef struct AxeraVcmdStream AxeraVcmdStream;

static void axera_vcmd_stream_clear(AxeraVcmdStream *st)
{
    if (st->hevc_ref) {
        hevc_enc_ref_free(st->hevc_ref);
    }
    memset(st, 0, sizeof(*st));
}

/* The context for (mode, w, h), creating one -- or recycling the least
 * recently used -- for a new stream. *fresh says the stream has no
 * reference yet, so its first job must be coded as an IDR. */
static AxeraVcmdStream *axera_vcmd_stream(AxeraVcmdState *s, uint32_t mode,
                                          uint32_t w, uint32_t h,
                                          bool *fresh)
{
    AxeraVcmdStream *lru = &s->streams[0];
    int i;

    for (i = 0; i < AXERA_VCMD_STREAMS; i++) {
        AxeraVcmdStream *st = &s->streams[i];

        if (st->used && st->mode == mode && st->w == w && st->h == h) {
            st->last_job = s->enc_jobs;
            *fresh = false;
            return st;
        }
        if (!st->used || (lru->used && st->last_job < lru->last_job)) {
            lru = st;
        }
    }
    axera_vcmd_stream_clear(lru);
    lru->used = true;
    lru->mode = mode;
    lru->w = w;
    lru->h = h;
    lru->last_job = s->enc_jobs;
    *fresh = true;
    return lru;
}

static void axera_vcmd_update_irq(AxeraVcmdState *s)
{
    qemu_set_irq(s->irq, (s->int_status & s->int_ctl) != 0);
}

/* AXERA_VCMD_TRACE=1: log every MMIO access, with the instance base. */
static bool axera_vcmd_trace(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("AXERA_VCMD_TRACE");
        on = e && e[0] == '1';
    }
    return on;
}

/*
 * swreg1 (HWIF_ENC_IRQ etc.) mixes write-1-to-clear status bits (IRQ,
 * FRAME_RDY, BUS_ERROR, SW_RESET, BUFFER_FULL, TIMEOUT, LINE_BUFFER,
 * SLICE_RDY, FUSE_ERROR, STRM_SEGMENT_RDY) with two control bits
 * (IRQ_DIS bit 1, TIMEOUT_INT enable bit 11). Every encode job's buffer
 * starts by writing 0x9fd there to ack stale status; storing that as
 * plain data made the job's read-back status claim a bus error, reset,
 * buffer-full and timeout alongside FRAME_RDY, and the encoder API
 * discarded every frame.
 */
#define ENC_SWREG1_STATUS_MASK  0x000013fdu

static void axera_vcmd_core_write(AxeraVcmdState *s, uint32_t creg,
                                  uint32_t value)
{
    if (creg == 1) {
        uint32_t st = s->core_reg[1] & ENC_SWREG1_STATUS_MASK & ~value;

        s->core_reg[1] = st | (value & ~ENC_SWREG1_STATUS_MASK);
        return;
    }
    s->core_reg[creg] = value;
}

static void axera_vcmd_complete(void *opaque)
{
    AxeraVcmdState *s = opaque;

    s->work_state = s->pend_jmp ? WORK_STATE_PEND : WORK_STATE_IDLE;
    s->int_status |= s->pending_status;
    s->pending_status = 0;
    axera_vcmd_update_irq(s);
}

static uint64_t axera_vcmd_read_reg(AxeraVcmdState *s, hwaddr offset);

static uint64_t axera_vcmd_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraVcmdState *s = opaque;
    uint64_t v = axera_vcmd_read_reg(s, offset);

    if (axera_vcmd_trace()) {
        fprintf(stderr, "AXERA_VCMD_TRACE: %p R [0x%04" HWADDR_PRIx
                "] = 0x%08" PRIx64 "\n", (void *)s, offset, v);
    }
    return v;
}

static uint64_t axera_vcmd_read_reg(AxeraVcmdState *s, hwaddr offset)
{
    switch (offset) {
    case HW_ID_OFF:
        return HW_ID_VALUE;
    case WORK_STATE_OFF:
        return s->work_state;
    case CONTROL_OFF:
        return s->work_state == WORK_STATE_WORK ? START_TRIGGER : 0;
    case INT_STATUS_OFF:
        return s->int_status;
    case INT_CTL_OFF:
        return s->int_ctl;
    case CMD_ADDR_LO_OFF:
        return s->cmd_addr_lo;
    case CMD_ADDR_HI_OFF:
        return s->cmd_addr_hi;
    case CMD_LENGTH_OFF:
        return s->cmd_length;
    case EXE_CMDBUF_COUNT_OFF:
        return s->exe_count;
    default:
        if (offset < VCMD_REG_WINDOW) {
            return s->vreg[offset / 4];
        }
        /* The VC8000E core's own register file (see core_reg[] below) is
         * one physical bank reachable two ways: queued WREG/RREG opcodes
         * inside an executed command buffer (axera_vcmd_exec_cmdbuf), or
         * a direct peek/poke over this same MMIO window -- confirmed
         * live that the real App does the latter too (mmap()s this
         * region directly via HANTRO_IOCG_HWOFFSET/HWIOSIZE before any
         * cmdbuf ever runs). Keep both paths reading/writing the same
         * storage so neither sees a stale or inconsistent view. */
        if (offset >= CORE_REG_BASE) {
            uint32_t creg = (offset - CORE_REG_BASE) / 4;

            if (creg < CORE_REG_COUNT) {
                return s->core_reg[creg];
            }
        }
        return 0;
    }
}

/*
 * Run one real encode job: triggered when a WREG (or, defensively, a
 * direct MMIO poke) sets HWIF_ENC_E (core_reg[ENC_REG_GEOM] bit0). Reads
 * the job's parameters straight out of core_reg[] (see the field map
 * above), reads the source picture out of guest memory, calls the
 * project's shared Hantro encoder library (hantro-h264.c/hevc-enc.c/
 * jpeg-enc.c -- already proven against hisi-vedu and xm-h1), and writes
 * a real bitstream back to the output buffer the job named. Unlike the
 * older Hantro H1 (xm-h1.c), whose driver pre-writes SPS/PPS bytes
 * itself and the ASIC contributes only slice data, VC8000E's register
 * file carries full SPS/PPS/slice semantic fields (scaling lists,
 * short-term RPS counts, deblocking offsets, etc.) -- this generation's
 * ASIC assembles whole NAL units, headers included. This model mirrors
 * that by synthesising its own headers from the real width/height/QP
 * register values rather than expecting guest-written header bytes
 * (which this architecture has no register pointing at).
 */
/*
 * Nothing in this machine produces camera pixels: there is no image
 * sensor, and the ISP that would turn its raw Bayer data into YUV is
 * closed silicon with no model here, so every picture that reaches the
 * encoder is all zeros. An unchanging empty picture is not only blank
 * (solid green once decoded), it codes to almost nothing, and the camera
 * App treats that as a dead encoder: it logs "zero bitrate" and reboots.
 * So when a source picture is entirely empty -- and only then; anything
 * real that ever reaches it is encoded untouched -- this paints a test
 * card in its place: SMPTE-style colour bars scrolling sideways with a
 * white box sweeping across them, advancing one step per frame of that
 * stream -- the whole picture changes every frame, as a real sensor's
 * noise would make it, so P-frames carry real bits.
 */
static bool axera_vcmd_plane_empty(const uint8_t *p, size_t n)
{
    size_t i;

    for (i = 0; i < n; i += 64) {
        if (p[i]) {
            return false;
        }
    }
    return true;
}

static void axera_vcmd_test_card(uint8_t *y, uint32_t stride_y,
                                 uint8_t *uv, uint32_t stride_c,
                                 uint32_t w, uint32_t h, uint32_t frame)
{
    /* 75% bars: white, yellow, cyan, green, magenta, red, blue, black */
    static const uint8_t bars[8][3] = {
        { 180, 128, 128 }, { 162,  44, 142 }, { 131, 156,  44 },
        { 112,  72,  58 }, {  84, 184, 198 }, {  65, 100, 212 },
        {  35, 212, 114 }, {  16, 128, 128 },
    };
    uint32_t bw = MAX(w / 8, 1), box = MAX(h / 6, 16) & ~1u;
    uint32_t bx = (frame * 8) % MAX(w - box, 1) & ~1u;
    uint32_t by = (h - box) / 2 & ~1u;
    uint32_t r, c;

    for (r = 0; r < h; r++) {
        for (c = 0; c < w; c++) {
            const uint8_t *b = bars[((c + frame * 16) / bw) % 8];
            bool in = r >= by && r < by + box && c >= bx && c < bx + box;

            y[(size_t)r * stride_y + c] = in ? 235 : b[0];
            if (!(r & 1) && !(c & 1)) {
                uint8_t *p = &uv[(size_t)(r / 2) * stride_c + c];

                p[0] = in ? 128 : b[1];
                p[1] = in ? 128 : b[2];
            }
        }
    }
}

static void axera_vcmd_run_encode_job(AxeraVcmdState *s)
{
    uint32_t mode = (s->core_reg[ENC_REG_MODE] >> ENC_MODE_SHIFT) &
                     ENC_MODE_MASK;
    uint32_t geom = s->core_reg[ENC_REG_GEOM];
    uint32_t coding_type = (geom >> ENC_CODING_TYPE_SHIFT) &
                            ENC_CODING_TYPE_MASK;
    uint32_t fmt = (s->core_reg[ENC_REG_IN_FORMAT] >> ENC_IN_FMT_SHIFT) &
                   ENC_IN_FMT_MASK;
    hwaddr out_base = s->core_reg[ENC_REG_OUT_BASE];
    hwaddr y_base = s->core_reg[ENC_REG_IN_Y];
    hwaddr cb_base = s->core_reg[ENC_REG_IN_CB];
    hwaddr cr_base = s->core_reg[ENC_REG_IN_CR];
    uint32_t out_cap = s->core_reg[ENC_REG_OUT_LIMIT];
    uint32_t w, h, mbw, mbh;
    int qp = (s->core_reg[ENC_REG_QP] >> ENC_PIC_INIT_QP_SHIFT) &
             ENC_PIC_INIT_QP_MASK;
    uint32_t stride_y = s->core_reg[ENC_REG_IN_LU_STRIDE] >> ENC_STRIDE_SHIFT;
    uint32_t stride_c = s->core_reg[ENC_REG_IN_CH_STRIDE] >> ENC_STRIDE_SHIFT;
    g_autofree uint8_t *y = NULL;
    g_autofree uint8_t *uv = NULL;
    g_autofree uint8_t *out = NULL;
    size_t n = 0, cap;
    bool idr, fresh;
    AxeraVcmdStream *st;

    if (mode == ENC_MODE_JPEG) {
        w = ((geom >> ENC_JPEG_W_SHIFT) & ENC_JPEG_W_MASK) * 8;
        h = ((geom >> ENC_JPEG_H_SHIFT) & ENC_JPEG_H_MASK) * 8;
    } else if (mode == ENC_MODE_H264 || mode == ENC_MODE_HEVC) {
        w = ((geom >> ENC_PIC_W_SHIFT) & ENC_PIC_W_MASK) * 8;
        h = ((geom >> ENC_PIC_H_SHIFT) & ENC_PIC_H_MASK) * 8;
    } else {
        return; /* not a codec mode this bridge knows, or a stray bit */
    }
    if (!w || !h || w > 4096 || h > 4096 || !y_base || !out_base) {
        qemu_log_mask(LOG_UNIMP, "axera-vcmd: encode job with implausible "
                      "geometry/addresses (mode=%u %ux%u y=0x%" PRIx64
                      " out=0x%" PRIx64 "), skipped\n", mode, w, h, y_base,
                      out_base);
        return;
    }
    mbw = DIV_ROUND_UP(w, 16);
    mbh = DIV_ROUND_UP(h, 16);
    if (!stride_y) {
        stride_y = w;
    }
    if (!stride_c) {
        stride_c = (fmt == 0) ? w / 2 : w;
    }
    st = axera_vcmd_stream(s, mode, w, h, &fresh);
    idr = (coding_type == ENC_CODING_TYPE_I) || fresh;

    y = g_malloc0((size_t)stride_y * mbh * 16);
    dma_memory_read(&address_space_memory, y_base, y,
                    (size_t)stride_y * mbh * 16, MEMTXATTRS_UNSPECIFIED);

    if (fmt == 0 && cb_base && cr_base) {
        /* Planar input: de-interleave Cb/Cr into one NV12-style plane,
         * the shared library's own interface (both engines underneath
         * this model were themselves integrated against semi-planar
         * input -- see xm-h1.c). */
        size_t cw = mbw * 8, ch = mbh * 8, i;
        g_autofree uint8_t *cb = g_malloc0((size_t)stride_c * ch);
        g_autofree uint8_t *cr = g_malloc0((size_t)stride_c * ch);

        dma_memory_read(&address_space_memory, cb_base, cb,
                        (size_t)stride_c * ch, MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, cr_base, cr,
                        (size_t)stride_c * ch, MEMTXATTRS_UNSPECIFIED);
        uv = g_malloc((size_t)cw * 2 * ch);
        for (i = 0; i < ch; i++) {
            size_t j;

            for (j = 0; j < cw; j++) {
                uv[i * cw * 2 + 2 * j] = cb[i * stride_c + j];
                uv[i * cw * 2 + 2 * j + 1] = cr[i * stride_c + j];
            }
        }
        stride_c = cw * 2;
    } else {
        hwaddr uv_base = cb_base ? cb_base : cr_base;

        uv = g_malloc0((size_t)stride_c * mbh * 8);
        if (uv_base) {
            dma_memory_read(&address_space_memory, uv_base, uv,
                            (size_t)stride_c * mbh * 8,
                            MEMTXATTRS_UNSPECIFIED);
        }
    }

    if (fmt != 0 && stride_c >= w &&
        axera_vcmd_plane_empty(y, (size_t)stride_y * h) &&
        axera_vcmd_plane_empty(uv, (size_t)stride_c * (h / 2))) {
        axera_vcmd_test_card(y, stride_y, uv, stride_c, w, h, st->frames);
    }
    st->frames++;

    cap = out_cap ? out_cap : 256 * 1024;
    out = g_malloc0(cap);

    if (mode == ENC_MODE_H264) {
        if (idr) {
            uint8_t hdr[128];
            size_t hdr_len = h264_enc_baseline_headers(w, h, hdr);

            memset(&st->h264, 0, sizeof(st->h264));
            h264_enc_parse_headers(&st->h264, hdr, hdr_len);
            if (hdr_len <= cap) {
                memcpy(out, hdr, hdr_len);
                n = h264_enc_frame(&st->h264, true, qp ? qp : 26, mbw, mbh,
                                   y, stride_y, uv, stride_c,
                                   out + hdr_len, cap - hdr_len);
                n = n ? n + hdr_len : 0;
            }
        } else if (st->h264.have_sps && st->h264.have_pps) {
            n = h264_enc_frame(&st->h264, false, qp ? qp : 26, mbw, mbh,
                               y, stride_y, uv, stride_c, out, cap);
        }
    } else if (mode == ENC_MODE_HEVC) {
        bool p_slice = !idr && st->hevc_ref;

        if (idr && st->hevc_ref) {
            hevc_enc_ref_free(st->hevc_ref);
            st->hevc_ref = NULL;
        }
        if (!st->hevc_ref) {
            st->hevc_ref = hevc_enc_ref_new();
        }
        n = hevc_enc_frame(&hevc_params_ctb32_sao, w, h, qp ? qp : 26,
                           p_slice, p_slice ? st->hevc_poc + 1 : 0,
                           p_slice ? st->hevc_ref : NULL, st->hevc_ref,
                           y, w, h, stride_y, uv, stride_c, out, cap);
        if (n) {
            st->hevc_poc = p_slice ? st->hevc_poc + 1 : 0;
        }
    } else if (mode == ENC_MODE_JPEG) {
        size_t cw = mbw * 8, ch = mbh * 8, i;
        g_autoptr(GByteArray) hdrb = g_byte_array_new();
        uint8_t ql[64], qc[64];
        int quality = CLAMP(100 - qp * 2, 5, 95);
        g_autofree uint8_t *cb = g_malloc(cw * ch);
        g_autofree uint8_t *cr = g_malloc(cw * ch);
        bool overflow = false;

        jpeg_quant_table(quality, jpeg_base_luma, ql);
        jpeg_quant_table(quality, jpeg_base_chroma, qc);
        for (i = 0; i < cw * ch; i++) {
            cb[i] = uv[2 * i];
            cr[i] = uv[2 * i + 1];
        }
        jpeg_enc_header(hdrb, w, h, ql, qc);
        if (hdrb->len <= cap) {
            memcpy(out, hdrb->data, hdrb->len);
            n = jpeg_enc_scan(y, stride_y, cb, cr, cw, w, h, ql, qc, true,
                              out + hdrb->len, cap - hdrb->len, &overflow);
            n = (n && !overflow) ? n + hdrb->len : 0;
        }
    }

    /*
     * NAL size table (swreg10, HWIF_ENC_SIZE_TBL_BASE): one u32 per NAL
     * unit the core wrote, start code included -- confirmed live, the
     * encoder API pre-fills the entries for the VPS/SPS/PPS it writes in
     * software (27/50/11 bytes, exactly the 0x58 it leaves in front of
     * swreg8) and points swreg10 just past them. It sums this table to
     * get each frame's size, so without the core's own entries every
     * encoded frame looked empty and was dropped (FrmEnc stayed 0).
     */
    if (n && (mode == ENC_MODE_H264 || mode == ENC_MODE_HEVC) &&
        s->core_reg[10]) {
        g_autoptr(GArray) sizes = g_array_new(false, false, sizeof(uint32_t));
        size_t p, start = 0;
        uint32_t zero = 0, sz;

        for (p = 4; p + 4 <= n; p++) {
            if (out[p] == 0 && out[p + 1] == 0 && out[p + 2] == 0 &&
                out[p + 3] == 1) {
                sz = p - start;
                g_array_append_val(sizes, sz);
                start = p;
                p += 3;
            }
        }
        sz = n - start;
        g_array_append_val(sizes, sz);
        g_array_append_val(sizes, zero);
        dma_memory_write(&address_space_memory, s->core_reg[10],
                         sizes->data, sizes->len * sizeof(uint32_t),
                         MEMTXATTRS_UNSPECIFIED);
    }
    if (n && getenv("AXERA_VCMD_DEBUG_JOB") &&
        s->enc_jobs < (uint32_t)atoi(getenv("AXERA_VCMD_DEBUG_JOB"))) {
        uint8_t pre[0x60];
        uint32_t k;

        dma_memory_read(&address_space_memory, out_base - 0x58, pre,
                        sizeof(pre), MEMTXATTRS_UNSPECIFIED);
        fprintf(stderr, "AXERA_VCMD_JOB: mode=%u %ux%u idr=%d out=0x%"
                PRIx64 " n=%zu sizetbl=0x%x swreg4=0x%x pre:", mode, w, h,
                idr, out_base, (size_t)n, s->core_reg[10], s->core_reg[4]);
        fprintf(stderr, " swreg5=0x%x swreg6=0x%x swreg7=0x%x swreg8=0x%x"
                " swreg9cap=0x%x", s->core_reg[5], s->core_reg[6],
                s->core_reg[7], s->core_reg[8], out_cap);
        for (k = 0; k < sizeof(pre); k++) {
            fprintf(stderr, "%02x", pre[k]);
        }
        {
            uint32_t tbl[4];

            dma_memory_read(&address_space_memory, s->core_reg[10] - 12,
                            tbl, sizeof(tbl), MEMTXATTRS_UNSPECIFIED);
            fprintf(stderr, " tbl[-3..0]=%u,%u,%u,%u", tbl[0], tbl[1],
                    tbl[2], tbl[3]);
        }
        fprintf(stderr, " ours:");
        for (k = 0; k < 24 && k < n; k++) {
            fprintf(stderr, "%02x", out[k]);
        }
        fprintf(stderr, "\n");
    }
    if (n) {
        dma_memory_write(&address_space_memory, out_base, out, n,
                         MEMTXATTRS_UNSPECIFIED);
        /*
         * On completion the core overwrites the buffer limit with the
         * number of bytes it wrote (HWIF_ENC_OUTPUT_STRM_BUFFER_LIMIT,
         * "Output stream size (bytes)"). Confirmed over RTSP: reporting
         * bits made the camera App send each slice 8x its real length.
         */
        s->core_reg[ENC_REG_OUT_LIMIT] = n;
        s->core_reg[ENC_REG_IRQ] |= ENC_IRQ_BIT | ENC_FRAME_RDY_BIT;
        /*
         * swreg7[24:17] HWIF_ENC_NUM_SLICES_READY: completed slices. The
         * bridge always emits the picture as one slice; the API compares
         * this with the slice count it asked for and treats a short count
         * as an unfinished frame.
         */
        s->core_reg[7] = (s->core_reg[7] & ~(0xffu << 17)) | (1u << 17);
    } else {
        qemu_log_mask(LOG_UNIMP, "axera-vcmd: encode job produced no "
                      "output (mode=%u %ux%u idr=%d)\n", mode, w, h, idr);
    }
    s->core_reg[ENC_REG_GEOM] &= ~ENC_E_BIT;
    s->enc_jobs++;
}

/*
 * Execute the queued command buffer: a stream of WREG/RREG/JMP/STALL/
 * CLRINT/NOP opcodes (vcmd_abi.h; cross-checked live against the REAL
 * closed Axera driver's own cmdbuf traffic, which decodes byte-for-byte
 * as this exact opcode set -- RREG_HDR(512, SUBMODULE_MAIN_ADDR) +
 * dst-lo/hi/pad + JMP_TAIL_WORD + target-lo/hi/pad). WREG/RREG address
 * and move data to/from this device's own virtual core-register file
 * (core_reg[], ASIC byte addresses CORE_REG_BASE and up) -- the real
 * VC8000E core's own register bank, a separate space from the 5 VCMD
 * front-end registers in axera_vcmd_read/write above. Addresses outside
 * that window, and STALL/CLRINT (real inter-engine synchronisation this
 * model has no need for, since completion is driven by the outer
 * CONTROL/timer handshake below, not real JMP chaining) are no-ops.
 */
/*
 * Run one command buffer. Returns true and sets *next_addr/*next_len when
 * it ends in a JMP with RDY set, i.e. the driver has linked another
 * buffer behind it and the hardware carries straight on.
 */
static bool axera_vcmd_exec_one(AxeraVcmdState *s, hwaddr addr,
                                uint32_t length, hwaddr *next_addr,
                                uint32_t *next_len)
{
    uint32_t bytes = length * 8;
    g_autofree uint32_t *buf = NULL;
    uint32_t nwords, i;

    if (!addr || !bytes) {
        return false;
    }
    bytes = MIN(bytes, 0x2000); /* vcmd_abi.h CMDBUF_MAX_SIZE */
    nwords = bytes / 4;
    buf = g_new(uint32_t, nwords);
    dma_memory_read(&address_space_memory, addr, buf, bytes,
                    MEMTXATTRS_UNSPECIFIED);

    for (i = 0; i < nwords; ) {
        uint32_t word = buf[i];
        uint32_t op = word >> 27;
        uint32_t n = (word >> 16) & 0x3ff;
        uint32_t a = word & 0xffff;

        if (axera_vcmd_trace()) {
            fprintf(stderr, "AXERA_VCMD_TRACE: %p op 0x%02x n=%u a=0x%04x "
                    "@%u word=0x%08x\n", (void *)s, op, n, a, i, word);
        }
        if (op == VCMD_OP_WREG) {
            bool fix = word & VCMD_WREG_FIX;
            uint32_t k;

            i++;
            for (k = 0; k < n && i < nwords; k++, i++) {
                uint32_t creg = (a - CORE_REG_BASE) / 4 + (fix ? 0 : k);

                if (a >= CORE_REG_BASE && creg < CORE_REG_COUNT) {
                    if (getenv("AXERA_VCMD_DEBUG_WREG")) {
                        fprintf(stderr, "AXERA_VCMD_DBG: WREG creg=%u val=0x%x\n",
                                creg, buf[i]);
                    }
                    axera_vcmd_core_write(s, creg, buf[i]);
                    if (creg == ENC_REG_GEOM && (buf[i] & ENC_E_BIT)) {
                        axera_vcmd_run_encode_job(s);
                    }
                }
            }
        } else if (op == VCMD_OP_RREG) {
            hwaddr dst;

            if (i + 3 >= nwords) {
                break;
            }
            dst = buf[i + 1] | ((hwaddr)buf[i + 2] << 32);
            if (a < VCMD_REG_WINDOW && dst) {
                /* The front-end's own registers: the encoder's buffers
                 * end by copying VCMD regs 0..26 into status memory,
                 * where the driver's ISR reads CMDBUF_EXECUTING_ID. */
                g_autofree uint32_t *out = g_new0(uint32_t, n);
                uint32_t k;

                for (k = 0; k < n; k++) {
                    hwaddr off = a + k * 4;

                    out[k] = off < VCMD_REG_WINDOW ?
                             axera_vcmd_read_reg(s, off) : 0;
                }
                dma_memory_write(&address_space_memory, dst, out,
                                 (size_t)n * 4, MEMTXATTRS_UNSPECIFIED);
            } else if (a >= CORE_REG_BASE && dst) {
                uint32_t creg = (a - CORE_REG_BASE) / 4;
                uint32_t k, ndst = MIN(n, CORE_REG_COUNT - creg);
                g_autofree uint32_t *out = g_new0(uint32_t, n);

                for (k = 0; k < ndst; k++) {
                    out[k] = s->core_reg[creg + k];
                }
                dma_memory_write(&address_space_memory, dst, out,
                                 (size_t)n * 4, MEMTXATTRS_UNSPECIFIED);
            }
            i += 4;
        } else if (op == VCMD_OP_STALL || op == VCMD_OP_CLRINT) {
            i += 2;
        } else if (op == VCMD_OP_JMP) {
            /* JMP: [op|RDY|IE|next length], next addr lo, hi, next ID */
            bool rdy = word & JMP_RDY;

            s->exe_count++;
            if (word & JMP_IE) {
                s->pending_status |= IRQ_JMPD;
            }
            if (rdy && i + 3 < nwords) {
                *next_addr = buf[i + 1] | ((hwaddr)buf[i + 2] << 32);
                *next_len = word & 0xffff;
                s->vreg[EXECUTING_ID_OFF / 4] = buf[i + 3];
                return true;
            }
            /* End of the linked chain: the core parks on this JMP until
             * software links another buffer behind it (see the
             * RDY_CMDBUF_COUNT write handler). */
            s->pend_jmp = addr + (hwaddr)i * 4;
            s->pending_status |= IRQ_JMPP;
            return false;
        } else if (op == VCMD_OP_END) {
            s->exe_count++;
            s->pending_status |= IRQ_ENDCMD;
            return false;
        } else if (op == VCMD_OP_NOP) {
            i++;
        } else {
            qemu_log_mask(LOG_UNIMP, "axera-vcmd: unknown cmdbuf opcode "
                          "0x%x @word %u\n", op, i);
            break;
        }
    }
}

static void axera_vcmd_exec_chain(AxeraVcmdState *s, hwaddr addr,
                                  uint32_t len)
{
    int n;

    s->pend_jmp = 0;
    for (n = 0; n < VCMD_MAX_CHAIN; n++) {
        hwaddr next_addr = 0;
        uint32_t next_len = 0;

        if (!axera_vcmd_exec_one(s, addr, len, &next_addr, &next_len)) {
            break;
        }
        addr = next_addr;
        len = next_len;
    }
}

static void axera_vcmd_exec_cmdbuf(AxeraVcmdState *s)
{
    axera_vcmd_exec_chain(s, ((hwaddr)s->cmd_addr_hi << 32) | s->cmd_addr_lo,
                          s->cmd_length);
}

/*
 * Software linked a new buffer behind the parked JMP (rewriting it with
 * RDY set and the next buffer's address/length/ID) and bumped
 * RDY_CMDBUF_COUNT: re-fetch the JMP and carry on. Its interrupt was
 * already raised when the core first parked on it.
 */
static void axera_vcmd_resume(AxeraVcmdState *s)
{
    uint32_t jmp[4];

    if (!s->pend_jmp || s->work_state != WORK_STATE_PEND) {
        return;
    }
    dma_memory_read(&address_space_memory, s->pend_jmp, jmp, sizeof(jmp),
                    MEMTXATTRS_UNSPECIFIED);
    if ((jmp[0] >> 27) != VCMD_OP_JMP || !(jmp[0] & JMP_RDY)) {
        return;
    }
    s->vreg[EXECUTING_ID_OFF / 4] = jmp[3];
    s->work_state = WORK_STATE_WORK;
    axera_vcmd_exec_chain(s, jmp[1] | ((hwaddr)jmp[2] << 32),
                          jmp[0] & 0xffff);
    timer_mod(&s->completion_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + AXERA_VCMD_LATENCY_NS);
}

static void axera_vcmd_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AxeraVcmdState *s = opaque;

    if (axera_vcmd_trace()) {
        fprintf(stderr, "AXERA_VCMD_TRACE: %p W [0x%04" HWADDR_PRIx
                "] = 0x%08" PRIx64 "\n", (void *)s, offset, value);
    }
    switch (offset) {
    case CONTROL_OFF:
        if (value & START_TRIGGER) {
            if (getenv("AXERA_VCMD_DEBUG_WREG")) {
                fprintf(stderr, "AXERA_VCMD_DBG: START_TRIGGER addr=0x%x%08x "
                        "len=%u\n", s->cmd_addr_hi, s->cmd_addr_lo,
                        s->cmd_length);
            }
            axera_vcmd_exec_cmdbuf(s);
            s->work_state = WORK_STATE_WORK;
            timer_mod(&s->completion_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     AXERA_VCMD_LATENCY_NS);
        } else {
            /* Abort: stop immediately, no completion interrupt. */
            timer_del(&s->completion_timer);
            s->work_state = WORK_STATE_IDLE;
            s->pend_jmp = 0;
        }
        break;
    case INT_STATUS_OFF:
        /* Write-1-to-clear: the guest ISR writes back exactly what it
         * read, acking every bit that was set. */
        s->int_status &= ~value;
        axera_vcmd_update_irq(s);
        break;
    case INT_CTL_OFF:
        s->int_ctl = value;
        axera_vcmd_update_irq(s);
        break;
    case CMD_ADDR_LO_OFF:
        s->cmd_addr_lo = value;
        break;
    case CMD_ADDR_HI_OFF:
        s->cmd_addr_hi = value;
        break;
    case CMD_LENGTH_OFF:
        s->cmd_length = value;
        break;
    case EXE_CMDBUF_COUNT_OFF:
        s->exe_count = value;
        break;
    case RDY_CMDBUF_COUNT_OFF:
        s->vreg[offset / 4] = value;
        axera_vcmd_resume(s);
        break;
    default:
        if (offset < VCMD_REG_WINDOW) {
            s->vreg[offset / 4] = value;
            break;
        }
        if (offset >= CORE_REG_BASE) {
            uint32_t creg = (offset - CORE_REG_BASE) / 4;

            if (creg < CORE_REG_COUNT) {
                if (getenv("AXERA_VCMD_DEBUG_WREG")) {
                    fprintf(stderr, "AXERA_VCMD_DBG: MMIO-WRITE creg=%u val=0x%x\n",
                            creg, (uint32_t)value);
                }
                axera_vcmd_core_write(s, creg, value);
                if (creg == ENC_REG_GEOM && (value & ENC_E_BIT)) {
                    axera_vcmd_run_encode_job(s);
                }
            }
        }
        break;
    }
}

static const MemoryRegionOps axera_vcmd_ops = {
    .read = axera_vcmd_read,
    .write = axera_vcmd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_vcmd_reset(DeviceState *dev)
{
    AxeraVcmdState *s = AXERA_VCMD(dev);

    timer_del(&s->completion_timer);
    s->int_status = 0;
    s->int_ctl = 0;
    s->work_state = WORK_STATE_IDLE;
    s->cmd_addr_lo = 0;
    s->cmd_addr_hi = 0;
    s->cmd_length = 0;
    memset(s->vreg, 0, sizeof(s->vreg));
    s->exe_count = 0;
    s->pending_status = 0;
    s->pend_jmp = 0;
    memset(s->core_reg, 0, sizeof(s->core_reg));
    s->core_reg[0] = ENC_CORE_HW_ID;
    s->core_reg[80] = ENC_SWREG80_FEATURES;
    for (int i = 0; i < AXERA_VCMD_STREAMS; i++) {
        axera_vcmd_stream_clear(&s->streams[i]);
    }
    s->enc_jobs = 0;
    qemu_set_irq(s->irq, 0);
}

static void axera_vcmd_realize(DeviceState *dev, Error **errp)
{
    AxeraVcmdState *s = AXERA_VCMD(dev);

    timer_init_ns(&s->completion_timer, QEMU_CLOCK_VIRTUAL,
                 axera_vcmd_complete, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_vcmd_ops, s,
                          TYPE_AXERA_VCMD, VCMD_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_vcmd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_vcmd_realize;
    device_class_set_legacy_reset(dc, axera_vcmd_reset);
}

static const TypeInfo axera_vcmd_info = {
    .name = TYPE_AXERA_VCMD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraVcmdState),
    .class_init = axera_vcmd_class_init,
};

static void axera_vcmd_register_types(void)
{
    type_register_static(&axera_vcmd_info);
}

type_init(axera_vcmd_register_types)
