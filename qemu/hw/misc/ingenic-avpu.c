/*
 * Ingenic T31 / T40 / T41 AVPU video encoder.
 *
 * An Allegro DVT encoder core (the ZynqMP VCU's lineage) without its MCU:
 * avpu.ko (SDK 1.1.5 opensource/drivers/avpu/avpu_{main,ip}.c) only
 * proxies register reads and writes and the interrupt, and libimp runs
 * Allegro's control software on the CPU (alcodec/lib_scheduler_enc,
 * lib_encode/EncSchedulerCpu.c, CoreManager.c, EncSliceParam.c,
 * SliceHdrWriter.c).  The registers, from reg-base (0x8000 on T31 and
 * T40, 0 on T41: AVPU_BASE_OFFSET in avpu_ip.h; T41's core is the next
 * generation, gen = 2, with core pages and 4 KiB list entries: CL_ENTRY2):
 *
 *   0x000-0x00C IDs                     0x010 written 4096, 0x054 0x80
 *   0x014 interrupt mask (bit 0 Enc1 / JPEG done, bit 2 Enc2 done)
 *   0x018 interrupt status, write 1 to clear; the line is status & mask
 *   0x028 bit 2 latches the timer, 0x030 the timer (libimp spins until it
 *         is not zero: Timer.c getTime)
 *   core c at + 0x200 c (CoreManager.c; libimp on T31 runs the video
 *   encoder on core 0 and JPEG on core 1, interrupt bits 4 c + 0 / 2):
 *   0x200-0x228 JPEG R0-R10, 0x230 / 0x234 / 0x238 JPEG status / bytes
 *         written / bit 1 overflow
 *   0x3E0 command list address, 0x3E4 start: 2 Enc1 from the list, 8 Enc2
 *         from the list, 1 JPEG from R0-R10
 *   0x3F0 reset, 0x3F4 clock gates, 0x3F8 busy (bit 1 Enc1, 4 Enc2; reads
 *         idle: libimp asserts when it is set at a start)
 *
 * A command list is a run of 0x200-byte entries, one per slice, the last
 * with word 0 bit 31 set; the hardware writes each slice's status back into
 * its entry (EncCmdRegs.c EncodingStatusRegsToSliceStatus,
 * EntropyStatusRegsToSliceStatus).  Enc1 (motion estimation, transform)
 * reads:
 *   W0  [31] last slice, [29:28] codec (0 AVC, 1 HEVC), [26:24] log2 LCU - 4,
 *       [6:4] the intra transform-tree depth (HEVC; T31 libimp: 1)
 *   W4  [10:0] width / 8 - 1, [22:12] height / 8 - 1
 *   W8  [9:8] cabac_init_idc (HEVC: cabac_init_flag), [10] CABAC,
 *       [11] Enc1 codes the slice too (no Enc2 pass)
 *   WC  [4:0] / [12:8] Cb / Cr QP offset, [23:16] slice QP, [29:28] slice
 *       type (0 B, 1 P, 2 I)
 *   W10 [4:0] / [13:8] deblocking beta / tc offsets / 2
 *   W14 [9:0] / [21:12] the slice's first LCU x / y
 *   W80 / W84 source luma / NV12 chroma, W88 [15:0] source pitch,
 *   W90 reconstruction, WA0 reference,
 *   WC0 / WC4 / WC8 / WCC the stream buffer base / size / write offset /
 *       room when W8 bit 11 has Enc1 code the slice itself (what T31
 *       libimp does, H.264 and HEVC alike), with the Enc2 words W6C-W7C
 *       below in the same entry,
 *   WD0 / WD4 its output to Enc2 otherwise (the "intermediate" map and data)
 * and reports 0x104 [29:0] the intermediate bytes (against WCC's room), or
 * the stream bytes when it coded the slice, and rate-control counters
 * 0x10C-0x164, which libimp's rate control divides by: they are made up,
 * non-zero and consistent (QP min / max / sums, the bytes the slice will
 * take).  Enc2 (entropy coding) reads:
 *   W70 [9:8] cabac_init_idc, [21:16] QP, [29:28] slice type
 *   W78 [19:0] LCUs in the slice - 1, [28:24] n: header bits left to it
 *   W7C the last n slice-header bits, right-aligned, the byte before them
 *       above them
 *   WE8 / WEC Enc1's intermediate map / data
 *   WF0 / WF4 / WF8 / WFC stream buffer base / size / write offset / room
 * libimp writes the parameter sets and the slice header itself (Sections.c,
 * SliceHdrWriter.c GenerateAvcSliceHeader: FlushNAL escapes the header into
 * the stream ending at base + offset, plus the n tail bits after it) --
 * after the picture is done, so nothing of the current picture's headers
 * is in the buffer while it is coded; the hardware continues the bitstream
 * from base + offset with the tail bits, the slice data and the trailing
 * bits, emulation prevention throughout, and reports the bytes in 0x1E4
 * [29:0] (0x104 when Enc1 coded it), bits 30 / 31 overflow / error.  T31
 * libimp has n = 0 and offset 0x220: it writes CABAC's alignment bits too.
 *
 * Here Enc1 copies the source picture and Enc2 codes it, the two matched by
 * the intermediate buffer address (WD0 = WE8), else in order.  H.264 slices
 * come from hw/misc/hantro-h264.c: in CABAC (libimp's choice) or CAVLC,
 * I slices as I_16x16 DC macroblocks and P slices inter coded (P_Skip,
 * P_L0_16x16 at quarter-pel with a residual, intra where nothing matches),
 * a picture's slices on one reconstruction (h264_pic_*) filtered when the
 * last is done; B slices all P_Skip-like (nothing changes).  The rate
 * control's estimate of a P slice still counts I_PCM macroblocks
 * (pcm-budget).  The reference is the picture each P picture leaves the decoder
 * with, kept by reconstruction address (W90) and looked up by the
 * reference address (WA0), so each channel follows its own.  HEVC slices
 * (W0 codec 1) come from hw/misc/hevc-enc.c, P slices inter coded too
 * (hevc-enc-inter.c.inc), with the CTB size, depth and deblocking offsets
 * of the command words, cabac_init_flag from W8 [9:8], and SAO, transform
 * skip, the scaling lists, TMVP and the inter transform depth as the
 * channel's SPS / PPS say once a stream buffer has them in front of the
 * offset (before that: T31 libimp's).  JPEG writes the whole
 * file, SOI to EOI (libimp has no marker code: JPEG_Encoder.c generateNals
 * is empty), from R4 / R5 (source Y / C), R6 (tables: luma and chroma
 * quantisers in zigzag order at bytes 0 and 64), R7-R10 (stream base,
 * size, offset, room), R1 [15:0] height - 1, [31:16] width - 1, R3
 * [15:0] the source pitch; the size goes to 0x234.
 *
 * Every job completes 200 us after its start (the WAIT_IRQ thread is
 * waiting by then; avpu_ip.c queues each status bit).  Addresses are
 * masked to the 29 bits the bus sees, so KSEG0/1 forms work too.
 * "-global ingenic-avpu.dump=DIR" appends each channel's H.264 / HEVC
 * output to DIR/avpu-<w>x<h>.h264 / .h265 and writes JPEGs to
 * DIR/avpu-<n>.jpg; "-global ingenic-avpu.trace=1" logs every start and
 * command entry's words, trace=2 every register write too (LOG_UNIMP,
 * -d unimp).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include "hantro-h264.h"
#include "hevc-enc.h"
#include "jpeg-enc.h"

#define TYPE_INGENIC_AVPU "ingenic-avpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAvpuState, INGENIC_AVPU)

/*
 * avpu.ko claims 1 MiB, but the block decodes only up to the registers:
 * T40's RTC sits at 0x132A0000, inside that MiB
 */
#define AVPU_MMIO_SIZE      0x100000
#define AVPU_REGS_SIZE      0x1000
/* gen 2 (T41): the common page, then a page per core (G2_CORE) */
#define AVPU_REGS_SIZE2     0x3000
#define AVPU_NREGS          (AVPU_REGS_SIZE2 / 4)
#define AVPU_JOB_NS         200000
#define AVPU_DMA_MASK       0x1fffffff

#define R_ID0               0x000
#define R_ID1               0x004
#define R_ID2               0x008
#define R_ID3               0x00C
#define R_IRQ_MASK          0x014
#define R_IRQ_STATUS        0x018
#define R_TIMER_CTL         0x028
#define R_TIMER             0x030
/* core c's registers: these + 0x200 c (JPEG runs on core 1 on T31) */
#define CORE_STRIDE         0x200
#define AVPU_NCORES         7
#define R_JPEG(n)           (0x200 + 4 * (n))
#define R_JPEG_STATUS       0x230
#define R_JPEG_BYTES        0x234
#define R_JPEG_FLAGS        0x238
#define R_CL_ADDR           0x3E0
#define R_START             0x3E4
#define R_RESET             0x3F0
#define R_CLOCK             0x3F4
#define R_BUSY              0x3F8

#define START_JPEG          (1u << 0)
#define START_ENC1          (1u << 1)
#define START_ENC2          (1u << 3)

#define IRQ_ENC1            (1u << 0)   /* and JPEG; << 4 c for core c */
#define IRQ_ENC2            (1u << 2)

#define CL_ENTRY            0x200
#define CL_MAX_ENTRIES      256

/*
 * Gen 2, the T41's core (libimp 1.2's CoreManager.c, EncCmdRegs.c): core
 * c's registers in the page at 0x1000 (c + 1), Enc1's at +0, Enc2's at
 * +0x800 -- the command list's address (low, high), its status area's
 * (low, high), a control word, and the start (2; 1: JPEG) at +0x18, busy
 * at +0x1C bit 1.  Done is interrupt bit 2c (Enc1, JPEG) / 2c + 1 (Enc2).
 * A list entry is 4 KiB: Enc1's command words at 0, its status at 0x5C0
 * (0xB0 bytes), Enc2's words at 0x800 and status at 0xDC0; the last entry
 * has word 0 bit 31 set.  The words are another arrangement of gen 1's;
 * avpu_g2_entry() puts them back into gen 1's places for the code below.
 */
#define CL_ENTRY2           0x1000
#define G2_CORE(c)          (0x1000 * ((c) + 1))
#define G2_ENC2             0x800
#define G2_START            0x018
#define G2_BUSY             0x01C
#define G2_STATUS1          0x5C0
#define G2_STATUS2          0xDC0

#define AVPU_MAX_JOBS       16
#define AVPU_MAX_FRAMES     4
#define AVPU_MAX_SLICES     68          /* 1080p, one per MB row */
#define AVPU_MAX_REFS       8
#define AVPU_MAX_STREAMS    8

/* one picture as Enc1 saw it, waiting for Enc2 */
typedef struct AvpuSlice {
    uint32_t key;                       /* WD0: what Enc2 gives in WE8 */
    int first, nlcu;
    bool coded;
} AvpuSlice;

typedef struct AvpuFrame {
    bool used;
    uint64_t seq;
    int codec, log2_lcu;
    int w, h;                           /* coded: H.264 16s, HEVC 8s */
    int pw, ph;                         /* the picture's own */
    int qp, type, idc, cqp;
    bool cabac;                         /* W8 [10] */
    int depth_intra;                    /* W0 [6:4] */
    uint32_t w10;                       /* deblocking */
    uint32_t ref_key, rec_key;
    uint8_t *y, *uv;                    /* NV12 at stride w, edges replicated */
    int nslices;
    AvpuSlice slices[AVPU_MAX_SLICES];
    /* the reference the picture's slices build, H.264 */
    uint8_t *ry, *ruv;
    bool p_ref_missing;
    int cursor;                         /* the next MB a P picture sends */
    H264Pic *pic;                       /* H.264 CABAC: the picture coder */
} AvpuFrame;

/* the picture a decoder holds after a coded picture */
typedef struct AvpuRef {
    bool used;
    uint64_t seq;
    uint32_t key;
    int w, h;
    uint8_t *y, *uv;                    /* H.264: NV12 at stride w */
    HevcEncRef *hevc;
    int cursor;
} AvpuRef;

/* what a channel's parameter sets said (in its I pictures' stream buffers) */
typedef struct AvpuStream {
    bool used;
    uint32_t key;                       /* the channel: codec, size */
    uint64_t seq;
    bool hevc_sps, hevc_pps;            /* seen */
    HevcEncParams hevc;
    bool h264_pps;                      /* seen */
    bool h264_t8x8;                     /* PPS transform_8x8_mode_flag */
} AvpuStream;

struct IngenicAvpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t reg_base;
    uint8_t gen;                        /* 1: T31, T40; 2: T41 */
    uint32_t hw_id;                     /* ID1: the part's encoder HW ID */
    uint32_t regs[AVPU_NREGS];
    uint32_t irq_status;
    int64_t timer_latch;

    /* started, not yet done: {kind, list address} */
    uint32_t njobs;
    uint32_t job_kind[AVPU_MAX_JOBS];
    uint32_t job_addr[AVPU_MAX_JOBS];
    uint32_t job_core[AVPU_MAX_JOBS];
    uint32_t pending;                   /* the done bits they raise */

    uint64_t seq;
    AvpuFrame frames[AVPU_MAX_FRAMES];
    AvpuRef refs[AVPU_MAX_REFS];
    AvpuStream streams[AVPU_MAX_STREAMS];
    uint32_t jpeg_count;

    char *dump;
    uint8_t trace;          /* 1: command entries, 2: register writes too */
    uint32_t pcm_budget;
};

static void avpu_update(IngenicAvpuState *s)
{
    qemu_set_irq(s->irq, (s->irq_status & s->regs[R_IRQ_MASK / 4]) != 0);
}

/* ---- guest memory ------------------------------------------------------- */

/*
 * T41's direct_mode hands the encoder its source as 0x80000000 /
 * 0x81000000: the IVDC's store (RAM there, hw/mips/ingenic.c), not DRAM
 */
#define AVPU_IVDC_BASE      0x80000000u
#define AVPU_IVDC_SIZE      0x02000000u

static hwaddr avpu_addr(hwaddr a)
{
    a &= 0xffffffff;
    return a - AVPU_IVDC_BASE < AVPU_IVDC_SIZE ? a : a & AVPU_DMA_MASK;
}

static void avpu_read(hwaddr a, void *buf, size_t n)
{
    dma_memory_read(&address_space_memory, avpu_addr(a), buf, n,
                    MEMTXATTRS_UNSPECIFIED);
}

static void avpu_write_mem(hwaddr a, const void *buf, size_t n)
{
    dma_memory_write(&address_space_memory, a & AVPU_DMA_MASK, buf, n,
                     MEMTXATTRS_UNSPECIFIED);
}

static void avpu_stl(hwaddr a, uint32_t v)
{
    v = cpu_to_le32(v);
    avpu_write_mem(a, &v, 4);
}

/*
 * A gen 2 entry's words into gen 1's places (only what the coder reads),
 * read off libimp 1.2's entries: Enc1's control words W0-W14 at 0, 4, 0xC,
 * 0x60, 0x64, 0x68 (W0's and W4's fields moved); the source luma / chroma
 * at 0xA0 / 0xA8 with the pitch at 0xD0; the reconstruction at 0x1C0 and
 * the reference at 0x108; the stream buffer base / size / offset / room
 * at 0x220 / 0x22C / 0x230 / 0x234; the intermediate map at 0x268.
 * Enc2's W6C-W7C are at 0x800-0x814 in another order.
 */
static void avpu_g2_entry(const uint32_t *r, uint32_t *e)
{
    const uint32_t *r2 = r + G2_ENC2 / 4;

    memset(e, 0, CL_ENTRY);
    /*
     * W0: last, codec, log2 LCU - 4; the intra transform depth is 1, what
     * libimp 1.2's SPS says (it codes before the SPS is in a buffer)
     */
    e[0] = (r[0] & (1u << 31)) | (extract32(r[0], 0, 2) << 28) |
           (extract32(r[0], 24, 3) << 24) | (1u << 4);
    /* W4: width / 8 - 1, height / 8 - 1 */
    e[1] = extract32(r[1], 0, 11) | (extract32(r[1], 16, 11) << 12);
    e[2] = r[0x0C / 4];                 /* W8: cabac_init, CABAC, Enc1 codes */
    e[3] = r[0x60 / 4];                 /* WC: QPs, slice type */
    e[4] = r[0x64 / 4];                 /* W10: deblocking */
    e[5] = r[0x68 / 4];                 /* W14: the first LCU */
    e[0x80 / 4] = r[0xA0 / 4];
    e[0x84 / 4] = r[0xA8 / 4];
    e[0x88 / 4] = r[0xD0 / 4];
    e[0x90 / 4] = r[0x1C0 / 4];
    e[0xA0 / 4] = r[0x108 / 4];
    e[0xC0 / 4] = r[0x220 / 4];
    e[0xC4 / 4] = r[0x22C / 4];
    e[0xC8 / 4] = r[0x230 / 4];
    e[0xCC / 4] = r[0x234 / 4];
    e[0xD0 / 4] = r[0x268 / 4];
    e[0xD4 / 4] = r[0x270 / 4];
    e[0x6C / 4] = r2[0];
    e[0x70 / 4] = r2[1];
    e[0x74 / 4] = r2[3];
    e[0x78 / 4] = extract32(r2[4], 0, 20) | (extract32(r2[5], 24, 5) << 24);
    e[0x7C / 4] = r2[2];
}

static hwaddr avpu_entry_size(const IngenicAvpuState *s)
{
    return s->gen == 2 ? CL_ENTRY2 : CL_ENTRY;
}

/* entry ea of a list, in gen 1's form */
static void avpu_load_entry(IngenicAvpuState *s, hwaddr ea, uint32_t *e)
{
    int i;

    if (s->gen == 2) {
        g_autofree uint32_t *r = g_malloc(CL_ENTRY2);

        avpu_read(ea, r, CL_ENTRY2);
        for (i = 0; i < CL_ENTRY2 / 4; i++) {
            r[i] = le32_to_cpu(r[i]);
        }
        if (s->trace) {
            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: gen 2 entry @0x%"
                          HWADDR_PRIx ":", ea);
            for (i = 0; i < CL_ENTRY2 / 4; i++) {
                if (r[i] && (i < G2_STATUS1 / 4 ||
                             (i >= G2_ENC2 / 4 && i < G2_STATUS2 / 4))) {
                    qemu_log_mask(LOG_UNIMP, " %03x=%08x", i * 4, r[i]);
                }
            }
            qemu_log_mask(LOG_UNIMP, "\n");
        }
        avpu_g2_entry(r, e);
        return;
    }
    avpu_read(ea, e, CL_ENTRY);
    for (i = 0; i < CL_ENTRY / 4; i++) {
        e[i] = le32_to_cpu(e[i]);
    }
}

/* a status word, given at its gen 1 offset in the entry */
static void avpu_status(IngenicAvpuState *s, hwaddr ea, uint32_t off,
                        uint32_t v)
{
    if (s->gen == 2) {
        if (off >= 0x1E4) {
            off = G2_STATUS2 + off - 0x1E4;
        } else if (off < 0x118) {
            off = G2_STATUS1 + off - 0x104;
        } else if (off > 0x118) {
            off = G2_STATUS1 + 0x14 + off - 0x11C;
        } else {
            return;                     /* not in gen 2's status */
        }
    }
    avpu_stl(ea + off, v);
}

/*
 * An NV12 picture pw x ph at pitch from Y / C into a w x h one (w, h the
 * multiples of 16 it is coded at) at stride w, edges replicated.
 */
static void avpu_load_nv12(hwaddr ya, hwaddr ca, int pitch, int pw, int ph,
                           int w, int h, uint8_t *y, uint8_t *uv)
{
    int r;

    for (r = 0; r < h; r++) {
        uint8_t *l = y + (size_t)r * w;

        if (r < ph) {
            avpu_read(ya + (hwaddr)r * pitch, l, pw);
            memset(l + pw, l[pw - 1], w - pw);
        } else {
            memcpy(l, y + (size_t)(ph - 1) * w, w);
        }
    }
    for (r = 0; r < h / 2; r++) {
        uint8_t *l = uv + (size_t)r * w;
        int x;

        if (r < (ph + 1) / 2) {
            avpu_read(ca + (hwaddr)r * pitch, l, pw & ~1);
            for (x = pw & ~1; x < w; x += 2) {
                l[x] = l[(pw & ~1) - 2];
                l[x + 1] = l[(pw & ~1) - 1];
            }
        } else {
            memcpy(l, uv + (size_t)((ph + 1) / 2 - 1) * w, w);
        }
    }
}

/* ---- the stream writer: emulation prevention, a circular buffer ---------- */

typedef struct AvpuWriter {
    hwaddr base;
    uint32_t size, start, room;
    uint32_t count;                     /* bytes from start */
    uint32_t acc, nbits;
    int zeros;
    bool full;
} AvpuWriter;

static void avpu_wr_init(AvpuWriter *w, uint32_t base, uint32_t size,
                         uint32_t start, uint32_t room)
{
    uint8_t prev[2] = { 1, 1 };

    *w = (AvpuWriter){ .base = base, .size = size ? size : UINT32_MAX,
                       .start = start, .room = room ? room : UINT32_MAX };
    /* the emulation-prevention state from the header bytes before */
    if (start >= 2 || size) {
        avpu_read(base + (start + w->size - 2) % w->size, &prev[0], 1);
        avpu_read(base + (start + w->size - 1) % w->size, &prev[1], 1);
    }
    w->zeros = prev[1] ? 0 : prev[0] ? 1 : 2;
}

static void avpu_wr_raw(AvpuWriter *w, uint8_t b)
{
    if (w->count < w->room) {
        avpu_write_mem(w->base + ((uint64_t)w->start + w->count) % w->size,
                       &b, 1);
    } else {
        w->full = true;
    }
    w->count++;
}

static void avpu_wr_byte(AvpuWriter *w, uint8_t b)
{
    if (w->zeros >= 2 && b <= 3) {
        avpu_wr_raw(w, 3);
        w->zeros = 0;
    }
    avpu_wr_raw(w, b);
    w->zeros = b ? 0 : w->zeros + 1;
}

static void avpu_wr_bits(AvpuWriter *w, uint32_t v, int n)
{
    while (n--) {
        w->acc = (w->acc << 1) | ((v >> n) & 1);
        if (++w->nbits == 8) {
            avpu_wr_byte(w, w->acc);
            w->acc = w->nbits = 0;
        }
    }
}

/* bits [from, to) of buf, most significant first */
static void avpu_wr_buf_bits(AvpuWriter *w, const uint8_t *buf, size_t from,
                             size_t to)
{
    size_t i = from;

    while (i < to && (i & 7)) {
        avpu_wr_bits(w, buf[i / 8] >> (7 - i % 8), 1);
        i++;
    }
    while (i + 8 <= to) {
        if (w->nbits) {
            avpu_wr_bits(w, buf[i / 8], 8);
        } else {
            avpu_wr_byte(w, buf[i / 8]);
        }
        i += 8;
    }
    while (i < to) {
        avpu_wr_bits(w, buf[i / 8] >> (7 - i % 8), 1);
        i++;
    }
}

/* ---- parameter sets in the stream buffers ------------------------------- */

static AvpuStream *avpu_stream(IngenicAvpuState *s, uint32_t key)
{
    AvpuStream *st = NULL;
    int i;

    for (i = 0; i < AVPU_MAX_STREAMS; i++) {
        if (s->streams[i].used && s->streams[i].key == key) {
            st = &s->streams[i];
            break;
        }
    }
    if (!st) {
        for (i = 0; i < AVPU_MAX_STREAMS; i++) {
            if (!st || !s->streams[i].used ||
                (st->used && s->streams[i].seq < st->seq)) {
                st = &s->streams[i];
            }
        }
        *st = (AvpuStream){ .used = true, .key = key };
    }
    st->seq = ++s->seq;
    return st;
}

typedef struct AvpuBits {
    const uint8_t *p;
    size_t n, pos;
} AvpuBits;

static uint32_t avpu_bits(AvpuBits *b, int n)
{
    uint32_t v = 0;

    while (n--) {
        v <<= 1;
        if (b->pos < b->n * 8) {
            v |= (b->p[b->pos / 8] >> (7 - b->pos % 8)) & 1;
        }
        b->pos++;
    }
    return v;
}

static uint32_t avpu_ue(AvpuBits *b)
{
    int z = 0;

    while (!avpu_bits(b, 1) && z < 32) {
        z++;
    }
    return ((1u << z) - 1) + avpu_bits(b, z);
}

static int32_t avpu_se(AvpuBits *b)
{
    const uint32_t k = avpu_ue(b);

    return k & 1 ? (int32_t)((k + 1) / 2) : -(int32_t)(k / 2);
}

/* the NAL's payload without emulation-prevention bytes */
static size_t avpu_unescape(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t i, o = 0;
    int zeros = 0;

    for (i = 0; i < n; i++) {
        if (zeros >= 2 && in[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = in[i] ? 0 : zeros + 1;
        out[o++] = in[i];
    }
    return o;
}

/* profile_tier_level(1, max_sub_layers_minus1) */
static void avpu_hevc_ptl(AvpuBits *b, int subs)
{
    bool prof[8], lvl[8];
    int i;

    avpu_bits(b, 8 + 32 + 48 + 8);         /* general_* and level_idc */
    for (i = 0; i < subs; i++) {
        prof[i] = avpu_bits(b, 1);
        lvl[i] = avpu_bits(b, 1);
    }
    if (subs > 0) {
        for (i = subs; i < 8; i++) {
            avpu_bits(b, 2);
        }
    }
    for (i = 0; i < subs; i++) {
        if (prof[i]) {
            avpu_bits(b, 88);
        }
        if (lvl[i]) {
            avpu_bits(b, 8);
        }
    }
}

/* what hevc-enc has to follow from the SPS and PPS (H.265 7.3.2.2-3) */
static bool avpu_hevc_sps(AvpuStream *st, const uint8_t *r, size_t n)
{
    AvpuBits b = { r, n, 0 };
    int subs, i;

    avpu_bits(&b, 4);                       /* vps id */
    subs = avpu_bits(&b, 3);
    avpu_bits(&b, 1);
    avpu_hevc_ptl(&b, subs);
    avpu_ue(&b);                            /* sps id */
    if (avpu_ue(&b) == 3) {                 /* chroma_format_idc */
        avpu_bits(&b, 1);
    }
    avpu_ue(&b);                            /* width */
    avpu_ue(&b);                            /* height */
    if (avpu_bits(&b, 1)) {                 /* conformance window */
        avpu_ue(&b); avpu_ue(&b); avpu_ue(&b); avpu_ue(&b);
    }
    avpu_ue(&b);                            /* bit depths */
    avpu_ue(&b);
    avpu_ue(&b);                            /* log2_max_poc_lsb - 4 */
    i = avpu_bits(&b, 1) ? 0 : subs;        /* sub_layer_ordering_info */
    for (; i <= subs; i++) {
        avpu_ue(&b); avpu_ue(&b); avpu_ue(&b);
    }
    const int log2_min_cb = avpu_ue(&b) + 3;
    const int log2_ctb = log2_min_cb + avpu_ue(&b);
    avpu_ue(&b);                            /* TB sizes */
    avpu_ue(&b);
    st->hevc.trafo_depth_inter = avpu_ue(&b);
    st->hevc.trafo_depth_intra = avpu_ue(&b);
    /* scaling lists: the defaults (the inter coder applies them; a DC
     * scales as with flat ones) */
    st->hevc.scaling_list = avpu_bits(&b, 1);
    if (st->hevc.scaling_list && avpu_bits(&b, 1)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: HEVC scaling list data\n");
        return false;
    }
    avpu_bits(&b, 1);                       /* amp */
    st->hevc.sao = avpu_bits(&b, 1);
    st->hevc.log2_ctb = log2_ctb;
    st->hevc.tmvp = true;
    st->hevc_sps = true;
    if (avpu_bits(&b, 1)) {                 /* pcm */
        avpu_bits(&b, 8);
        avpu_ue(&b);
        avpu_ue(&b);
        avpu_bits(&b, 1);
    }
    if (avpu_ue(&b)) {                      /* num_short_term_ref_pic_sets */
        return true;                        /* (libimp's: 0) TMVP assumed */
    }
    if (avpu_bits(&b, 1)) {                 /* long-term references */
        return true;
    }
    st->hevc.tmvp = avpu_bits(&b, 1);       /* sps_temporal_mvp_enabled */
    return b.pos <= b.n * 8;
}

/*
 * The PPS, parsed to its end: true only when its rbsp_trailing_bits() come
 * where the NAL ends -- what lies in front of a slice's offset in a stream
 * buffer is what an earlier picture left there, and a P picture's header
 * written over the middle of an I picture's PPS leaves one that parses,
 * wrongly
 */
static bool avpu_hevc_pps(AvpuStream *st, const uint8_t *r, size_t n)
{
    AvpuBits b = { r, n, 0 };

    avpu_ue(&b);                            /* pps id */
    avpu_ue(&b);                            /* sps id */
    avpu_bits(&b, 1 + 1 + 3);               /* to num_extra_slice_header */
    st->hevc.sign_hiding = avpu_bits(&b, 1);
    avpu_bits(&b, 1);                       /* cabac_init_present */
    avpu_ue(&b);                            /* num_ref_idx defaults */
    avpu_ue(&b);
    avpu_se(&b);                            /* init_qp */
    avpu_bits(&b, 1);                       /* constrained intra */
    st->hevc.transform_skip = avpu_bits(&b, 1);
    if (avpu_bits(&b, 1)) {                 /* cu_qp_delta_enabled */
        avpu_ue(&b);
    }
    avpu_se(&b);                            /* cb / cr qp offsets */
    avpu_se(&b);
    avpu_bits(&b, 1 + 1 + 1 + 1);           /* to transquant bypass */
    const bool tiles = avpu_bits(&b, 1);
    avpu_bits(&b, 1);                       /* entropy sync */
    if (tiles) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: HEVC tiles\n");
        return false;
    }
    avpu_bits(&b, 1);                       /* across slices */
    if (avpu_bits(&b, 1)) {                 /* deblocking control */
        avpu_bits(&b, 1);                   /* override enabled */
        if (!avpu_bits(&b, 1)) {            /* pps disable */
            st->hevc.beta_offset_div2 = avpu_se(&b);
            st->hevc.tc_offset_div2 = avpu_se(&b);
        }
    } else {
        st->hevc.beta_offset_div2 = 0;
        st->hevc.tc_offset_div2 = 0;
    }
    if (avpu_bits(&b, 1)) {                 /* pps_scaling_list_data */
        return false;
    }
    avpu_bits(&b, 1);                       /* lists_modification */
    avpu_ue(&b);                            /* log2_parallel_merge_level */
    avpu_bits(&b, 1);                       /* slice header extension */
    if (avpu_bits(&b, 1)) {                 /* pps extensions */
        return false;
    }
    /* rbsp_trailing_bits(): a 1, then zeros to the NAL's end */
    if (!avpu_bits(&b, 1) || b.pos > b.n * 8) {
        return false;
    }
    while (b.pos < b.n * 8) {
        if (avpu_bits(&b, 1)) {
            return false;
        }
    }
    return true;
}

/* is there more than rbsp_trailing_bits() from bit pos on (7.2) */
static bool avpu_more_rbsp(const AvpuBits *b)
{
    size_t last = b->n * 8;

    while (last > 0 &&
           !((b->p[(last - 1) / 8] >> (7 - (last - 1) % 8)) & 1)) {
        last--;
    }
    return last && b->pos < last - 1;
}

/* the H.264 PPS's transform_8x8_mode_flag (7.3.2.2); true when it ends in
 * its trailing bits where the NAL does */
static bool avpu_h264_pps(AvpuStream *st, const uint8_t *r, size_t n)
{
    AvpuBits b = { r, n, 0 };

    avpu_ue(&b);                            /* pps id */
    avpu_ue(&b);                            /* sps id */
    avpu_bits(&b, 2);                       /* entropy, bottom field poc */
    if (avpu_ue(&b)) {                      /* num_slice_groups_minus1 */
        return false;
    }
    avpu_ue(&b);                            /* num_ref_idx defaults */
    avpu_ue(&b);
    avpu_bits(&b, 3);                       /* weighted pred / bipred */
    avpu_se(&b);                            /* pic_init_qp / qs */
    avpu_se(&b);
    avpu_se(&b);                            /* chroma_qp_index_offset */
    avpu_bits(&b, 3);                       /* deblocking, constrained, rpc */
    st->h264_t8x8 = false;
    if (avpu_more_rbsp(&b)) {
        st->h264_t8x8 = avpu_bits(&b, 1);
        if (avpu_bits(&b, 1)) {             /* pic_scaling_matrix_present */
            return false;
        }
        avpu_se(&b);                        /* second_chroma_qp_index_offset */
    }
    if (!avpu_bits(&b, 1) || b.pos > b.n * 8) {
        return false;
    }
    while (b.pos < b.n * 8) {
        if (avpu_bits(&b, 1)) {
            return false;
        }
    }
    return true;
}

/* SPS / PPS / VPS in the bytes libimp wrote before the slice */
static void avpu_parse_headers(IngenicAvpuState *s, AvpuStream *st,
                               int codec, uint32_t base, uint32_t size,
                               uint32_t off)
{
    const uint32_t n = MIN(off, 2048u);
    g_autofree uint8_t *buf = NULL;
    size_t i = 0;

    if (!n) {
        return;
    }
    /*
     * The channel's parameter sets are taken from an SPS and a PPS that
     * follow each other whole (each ended by the next start code, the PPS
     * parsed to its trailing bits): the bytes in front of the offset are
     * an earlier picture's -- the parameter sets a channel's I pictures
     * carry, or the remains of them under a P picture's header, which are
     * passed over
     */
    AvpuStream scratch = *st;
    bool sps_ok = false, pps_ok = false;

    buf = g_malloc(n);
    avpu_read(base + off - n, buf, n);
    while (i + 5 < n) {
        if (buf[i] || buf[i + 1] || buf[i + 2] != 1) {
            i++;
            continue;
        }
        size_t a = i + 3, j = a;
        while (j + 2 < n && !(buf[j] == 0 && buf[j + 1] == 0 &&
                              buf[j + 2] <= 1)) {
            j++;
        }
        if (j + 2 >= n) {
            j = n;
        }
        if (codec == 0 && j > a + 1 && j < n && (buf[a] & 0x1f) == 8) {
            g_autofree uint8_t *rbsp = g_malloc(j - a);
            const size_t m = avpu_unescape(buf + a + 1, j - a - 1, rbsp);

            if (avpu_h264_pps(&scratch, rbsp, m)) {
                st->h264_t8x8 = scratch.h264_t8x8;
                st->h264_pps = true;
            }
        } else if (codec == 1 && j > a + 2 && j < n) {
            g_autofree uint8_t *rbsp = g_malloc(j - a);
            const size_t m = avpu_unescape(buf + a + 2, j - a - 2, rbsp);

            switch ((buf[a] >> 1) & 0x3f) {
            case 33:
                sps_ok = avpu_hevc_sps(&scratch, rbsp, m);
                break;
            case 34:
                pps_ok = sps_ok && avpu_hevc_pps(&scratch, rbsp, m);
                break;
            }
        }
        i = j;
    }
    if (pps_ok) {
        st->hevc = scratch.hevc;
        st->hevc_sps = st->hevc_pps = true;
    }
}

/* ---- references -------------------------------------------------------- */

static void avpu_ref_clear(AvpuRef *r)
{
    g_free(r->y);
    g_free(r->uv);
    hevc_enc_ref_free(r->hevc);
    *r = (AvpuRef){ 0 };
}

static AvpuRef *avpu_ref_find(IngenicAvpuState *s, uint32_t key, int w,
                              int h)
{
    AvpuRef *best = NULL;
    int i;

    for (i = 0; i < AVPU_MAX_REFS; i++) {
        AvpuRef *r = &s->refs[i];

        if (r->used && r->w == w && r->h == h) {
            if (key && r->key == key) {
                return r;
            }
            if (!best || r->seq > best->seq) {
                best = r;
            }
        }
    }
    /* no reconstruction at that address: the channel's latest picture */
    return best;
}

/* a new reference under key (replacing one there), the oldest evicted */
static AvpuRef *avpu_ref_new(IngenicAvpuState *s, uint32_t key, int w, int h)
{
    AvpuRef *r = NULL;
    int i;

    for (i = 0; i < AVPU_MAX_REFS; i++) {
        if (s->refs[i].used && key && s->refs[i].key == key) {
            r = &s->refs[i];
            break;
        }
    }
    if (!r) {
        for (i = 0; i < AVPU_MAX_REFS; i++) {
            if (!r || !s->refs[i].used ||
                (r->used && s->refs[i].seq < r->seq)) {
                r = &s->refs[i];
            }
        }
    }
    avpu_ref_clear(r);
    *r = (AvpuRef){ .used = true, .seq = ++s->seq, .key = key, .w = w,
                    .h = h };
    return r;
}

/* ---- Enc1 ------------------------------------------------------------- */

static void avpu_frame_free(AvpuFrame *f)
{
    h264_pic_free(f->pic);
    g_free(f->y);
    g_free(f->uv);
    g_free(f->ry);
    g_free(f->ruv);
    *f = (AvpuFrame){ 0 };
}

static AvpuFrame *avpu_frame_new(IngenicAvpuState *s)
{
    AvpuFrame *f = NULL;
    int i;

    for (i = 0; i < AVPU_MAX_FRAMES; i++) {
        if (!f || !s->frames[i].used ||
            (f->used && s->frames[i].seq < f->seq)) {
            f = &s->frames[i];
        }
    }
    if (f->used) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: a picture from Enc1 "
                      "never went through Enc2\n");
    }
    avpu_frame_free(f);
    f->used = true;
    f->seq = ++s->seq;
    return f;
}

static void avpu_trace_entry(IngenicAvpuState *s, const char *what,
                             hwaddr a, const uint32_t *e)
{
    int i;

    if (!s->trace) {
        return;
    }
    qemu_log_mask(LOG_UNIMP, "ingenic-avpu: %s entry @0x%" HWADDR_PRIx ":",
                  what, a);
    for (i = 0; i < 0x100 / 4; i++) {
        if (e[i]) {
            qemu_log_mask(LOG_UNIMP, " %02x=%08x", i * 4, e[i]);
        }
    }
    qemu_log_mask(LOG_UNIMP, "\n");
}

static bool avpu_mb_differs(const AvpuFrame *f, int mx, int my,
                            const uint8_t *ry, const uint8_t *ruv)
{
    int r;

    for (r = 0; r < 16; r++) {
        const size_t o = (size_t)(my * 16 + r) * f->w + mx * 16;
        if (memcmp(f->y + o, ry + o, 16)) {
            return true;
        }
    }
    for (r = 0; r < 8; r++) {
        const size_t o = (size_t)(my * 8 + r) * f->w + mx * 16;
        if (memcmp(f->uv + o, ruv + o, 16)) {
            return true;
        }
    }
    return false;
}

/* macroblocks of rows [r0, r0 + rows) that differ from the reference */
static int avpu_changed_mbs(const AvpuFrame *f, int r0, int rows,
                            const uint8_t *ry, const uint8_t *ruv)
{
    const int mbw = f->w / 16;
    int n = 0, mx, my;

    for (my = r0; my < r0 + rows; my++) {
        for (mx = 0; mx < mbw; mx++) {
            n += avpu_mb_differs(f, mx, my, ry, ruv);
        }
    }
    return n;
}

/*
 * The I_PCM macroblocks a P slice may send: as many as fit the stream
 * buffer's room and the pcm-budget property.  The others that changed
 * wait for the next P picture (they still differ from the reference), so
 * a picture that changes everywhere is refreshed over a few pictures.
 */
static int avpu_pcm_mbs(IngenicAvpuState *s, uint32_t room)
{
    uint32_t b = s->pcm_budget;

    if (room) {
        b = MIN(b, room > 8192 ? room - 8192 : room / 2);
    }
    return MAX(b / 390, 1);
}

/* the bytes a slice is going to take, for the rate control */
static uint32_t avpu_estimate(IngenicAvpuState *s, AvpuFrame *f,
                              const AvpuSlice *sl)
{
    const int mbw = f->w / 16;
    const int r0 = sl->first / mbw, rows = MAX(sl->nlcu / mbw, 1);
    AvpuRef *ref;

    if (f->codec != 0) {
        return sl->nlcu * (f->type == 2 ? 24 : 2) + 8;
    }
    if (f->type == 2) {
        const size_t cap = (size_t)mbw * rows * 16 + 64;
        g_autofree uint8_t *out = g_malloc(cap);

        return h264_slice_i16dc_cavlc(f->qp, f->cqp, mbw, rows,
                                      f->y + (size_t)r0 * 16 * f->w, f->w,
                                      f->uv + (size_t)r0 * 8 * f->w, f->w, 0,
                                      out, cap) / 8 + 1;
    }
    ref = avpu_ref_find(s, f->ref_key, f->w, f->h);
    if (f->type == 1 && ref && ref->y) {
        return MIN(avpu_changed_mbs(f, r0, rows, ref->y, ref->uv),
                   avpu_pcm_mbs(s, 0)) * 385 + sl->nlcu / 8 + 2;
    }
    return f->type == 1 ? MIN(sl->nlcu, avpu_pcm_mbs(s, 0)) * 385 + 2
                        : sl->nlcu / 8 + 2;
}

/* the made-up encoding statistics of one slice */
static void avpu_enc1_status(IngenicAvpuState *s, hwaddr e,
                             const AvpuFrame *f, int nlcu,
                             uint32_t bytes, uint32_t interm)
{
    const int intra = f->type == 2 ? nlcu : nlcu / 4;
    const uint64_t qpsum = (uint64_t)f->qp * nlcu;
    const uint64_t cplx = (uint64_t)nlcu * 256;
    int i;

    for (i = 0x104; i <= 0x174; i += 4) {
        avpu_status(s, e, i, 0);
    }
    avpu_status(s, e, 0x104, interm);
    avpu_status(s, e, 0x10C, nlcu);
    avpu_status(s, e, 0x110, intra);                 /* intra LCUs */
    avpu_status(s, e, 0x114, nlcu - intra);          /* inter */
    avpu_status(s, e, 0x11C, nlcu);
    avpu_status(s, e, 0x120, nlcu);
    avpu_status(s, e, 0x124, f->type == 2 ? 0 : nlcu - intra);
    avpu_status(s, e, 0x128, nlcu);
    avpu_status(s, e, 0x12C, nlcu | (nlcu << 16));
    avpu_status(s, e, 0x130, nlcu);
    avpu_status(s, e, 0x134, bytes * 8);
    avpu_status(s, e, 0x138, f->qp | (f->qp << 8) | ((uint32_t)nlcu << 16));
    avpu_status(s, e, 0x13C, bytes);
    for (i = 0x140; i <= 0x154; i += 4) {
        avpu_status(s, e, i, nlcu);
    }
    avpu_status(s, e, 0x158, qpsum);
    avpu_status(s, e, 0x15C, qpsum >> 32);
    avpu_status(s, e, 0x160, cplx);
    avpu_status(s, e, 0x164, cplx >> 32);
}

static void avpu_code_slice(IngenicAvpuState *s, AvpuFrame *f, AvpuSlice *sl,
                            const uint32_t *e, hwaddr ea, bool by_enc1);
static void avpu_frame_retire(AvpuFrame *f);

static void avpu_enc1(IngenicAvpuState *s, hwaddr list)
{
    AvpuFrame *f = NULL;
    uint32_t e[CL_ENTRY / 4];
    int k;

    for (k = 0; k < CL_MAX_ENTRIES; k++) {
        const hwaddr ea = list + (hwaddr)k * avpu_entry_size(s);

        avpu_load_entry(s, ea, e);
        avpu_trace_entry(s, "Enc1", ea, e);
        const int lcu = 1 << (extract32(e[0], 24, 3) + 4);

        if (!f) {
            const int log2 = extract32(e[0], 24, 3) + 4;
            const int pw = (extract32(e[1], 0, 11) + 1) * 8;
            const int ph = (extract32(e[1], 12, 11) + 1) * 8;
            int pitch = extract32(e[0x88 / 4], 0, 16);

            f = avpu_frame_new(s);
            f->codec = extract32(e[0], 28, 2);
            f->log2_lcu = log2;
            f->pw = pw;
            f->ph = ph;
            /* HEVC codes the SPS's size (multiples of 8), H.264 whole MBs */
            f->w = f->codec == 1 ? pw : ROUND_UP(pw, 16);
            f->h = f->codec == 1 ? ph : ROUND_UP(ph, 16);
            f->qp = extract32(e[3], 16, 8);
            f->type = extract32(e[3], 28, 2);
            f->idc = extract32(e[2], 8, 2);
            f->cqp = sextract32(e[3], 0, 5);
            f->cabac = e[2] & (1u << 10);
            f->depth_intra = extract32(e[0], 4, 3);
            f->w10 = e[4];
            f->rec_key = e[0x90 / 4] & AVPU_DMA_MASK;
            f->ref_key = e[0xA0 / 4] & AVPU_DMA_MASK;
            if (pitch < pw) {
                pitch = ROUND_UP(pw, 16);
            }
            f->y = g_malloc((size_t)f->w * f->h);
            f->uv = g_malloc((size_t)f->w * f->h / 2);
            avpu_load_nv12(e[0x80 / 4], e[0x84 / 4], pitch, pw, ph, f->w,
                           f->h, f->y, f->uv);
        }
        if (f->nslices < AVPU_MAX_SLICES) {
            AvpuSlice *sl = &f->slices[f->nslices++];
            const int lcuw = DIV_ROUND_UP(f->w, f->codec == 1 ? lcu : 16);
            const int lcuh = DIV_ROUND_UP(f->h, f->codec == 1 ? lcu : 16);
            const bool last = e[0] & (1u << 31);
            uint32_t next = lcuw * lcuh;

            sl->key = e[0xD0 / 4];
            sl->first = extract32(e[5], 12, 10) * lcuw + extract32(e[5], 0, 10);
            if (!last) {
                uint32_t n0, en[CL_ENTRY / 4];

                avpu_load_entry(s, ea + avpu_entry_size(s), en);
                n0 = en[5];
                next = extract32(n0, 12, 10) * lcuw + extract32(n0, 0, 10);
            }
            sl->nlcu = next > sl->first ? next - sl->first : 1;
            if (e[2] & (1u << 11)) {
                /* no Enc2 pass: Enc1 writes the stream itself */
                avpu_code_slice(s, f, sl, e, ea, true);
            } else {
                const uint32_t bytes = avpu_estimate(s, f, sl);
                const uint32_t room = e[0xCC / 4];
                uint32_t interm = MIN(sl->nlcu * 16u, 0x3FFFFFFFu);

                if (room > 64 && interm + 32 >= room - 32) {
                    interm = room / 2;
                }
                avpu_enc1_status(s, ea, f, sl->nlcu, bytes, interm);
            }
        }
        if (e[0] & (1u << 31)) {
            /* coded here (W8 bit 11): nothing left for Enc2 */
            avpu_frame_retire(f);
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: Enc1 list without a last "
                  "entry\n");
}

/* ---- slice coding ----------------------------------------------------- */

/*
 * The dump: each slice NAL from its start code (the last one before the
 * offset) to its end, after an I slice's parameter sets (from the last
 * SPS / VPS before it).  What lies further back in the buffer is stale.
 */
static void avpu_dump(IngenicAvpuState *s, const AvpuFrame *f, uint32_t base,
                      uint32_t off, uint32_t count)
{
    const uint32_t win = MIN(off, 2048u);
    g_autofree char *path = NULL;
    g_autofree uint8_t *buf = NULL;
    uint32_t i, from = win, ps = win;
    FILE *fp;

    if (!s->dump || !win) {
        return;
    }
    buf = g_malloc(win + count);
    avpu_read(base + off - win, buf, win + count);
    for (i = 0; i + 3 < win; i++) {
        if (buf[i] || buf[i + 1] || buf[i + 2] != 1) {
            continue;
        }
        const int t = f->codec == 1 ? (buf[i + 3] >> 1) & 0x3f
                                    : buf[i + 3] & 0x1f;
        const uint32_t sc = i && !buf[i - 1] ? i - 1 : i;

        from = sc;
        if (f->codec == 1 ? t == 32 : t == 7) {
            ps = sc;
        }
    }
    if (f->type == 2 && ps < from) {
        from = ps;
    }
    path = g_strdup_printf("%s/avpu-%dx%d.%s", s->dump, f->pw, f->ph,
                           f->codec == 1 ? "h265" : "h264");
    fp = fopen(path, "ab");
    if (fp) {
        fwrite(buf + from, 1, win + count - from, fp);
        fclose(fp);
    }
}

/*
 * With dump: the picture the decoder holds after each coded picture (the
 * reference kept), cropped to the picture's size, appended to
 * DIR/avpu-<w>x<h>.<ext>.rec as NV12 -- for comparing with a decode of the
 * dumped stream -- and the source picture to DIR/avpu-<w>x<h>.<ext>.src.
 */
static void avpu_dump_rec(IngenicAvpuState *s, const AvpuFrame *f,
                          const uint8_t *y, const uint8_t *cb,
                          const uint8_t *cr, const uint8_t *uv)
{
    g_autofree char *path = NULL;
    FILE *fp;
    int r, x;

    if (!s->dump) {
        return;
    }
    path = g_strdup_printf("%s/avpu-%dx%d.%s.src", s->dump, f->pw, f->ph,
                           f->codec == 1 ? "h265" : "h264");
    fp = fopen(path, "ab");
    if (fp) {
        /* and the source it was coded from, alike */
        for (r = 0; r < f->ph; r++) {
            fwrite(f->y + (size_t)r * f->w, 1, f->pw, fp);
        }
        for (r = 0; r < f->ph / 2; r++) {
            fwrite(f->uv + (size_t)r * f->w, 1, f->pw, fp);
        }
        fclose(fp);
    }
    g_free(path);
    path = g_strdup_printf("%s/avpu-%dx%d.%s.rec", s->dump, f->pw, f->ph,
                           f->codec == 1 ? "h265" : "h264");
    fp = fopen(path, "ab");
    if (!fp) {
        return;
    }
    for (r = 0; r < f->ph; r++) {
        fwrite(y + (size_t)r * f->w, 1, f->pw, fp);
    }
    for (r = 0; r < f->ph / 2; r++) {
        if (uv) {
            fwrite(uv + (size_t)r * f->w, 1, f->pw, fp);
            continue;
        }
        for (x = 0; x < f->pw / 2; x++) {
            fputc(cb[(size_t)r * (f->w / 2) + x], fp);
            fputc(cr[(size_t)r * (f->w / 2) + x], fp);
        }
    }
    fclose(fp);
}

static void avpu_code_slice(IngenicAvpuState *s, AvpuFrame *f, AvpuSlice *sl,
                            const uint32_t *e, hwaddr ea, bool by_enc1)
{
    /* the stream buffer: Enc2's, or WC0-WCC when Enc1 codes the slice */
    const int sw = by_enc1 ? 0xC0 / 4 : 0xF0 / 4;
    const uint32_t base = e[sw] & AVPU_DMA_MASK;
    const uint32_t size = e[sw + 1], off = e[sw + 2];
    const uint32_t room = e[sw + 3];
    const int n = MIN(extract32(e[0x78 / 4], 24, 5), 24);
    const uint32_t tail = e[0x7C / 4];
    /*
     * The picture's last slice: when Enc1 codes the slices it does so as
     * it reads their entries, so the entry's own last bit says it; for
     * Enc2 the picture's slices are all known by then
     */
    const bool last_slice = by_enc1 ? !!(e[0] & (1u << 31)) :
                            (sl == &f->slices[f->nslices - 1] ||
                             sl->first + sl->nlcu >= (f->w >> 4) * (f->h >> 4));
    AvpuStream *st = avpu_stream(s, (f->codec << 30) ^ (f->pw << 15) ^ f->ph);
    AvpuWriter w;
    g_autofree uint8_t *out = NULL;
    size_t cap, bits, from = 0;

    avpu_parse_headers(s, st, f->codec, base, size, off);
    avpu_wr_init(&w, base, size, off, room);
    avpu_wr_bits(&w, tail & ((1u << n) - 1), n);

    if (f->codec == 1) {
        /* HEVC: the header ends in byte_alignment() */
        /*
         * libimp writes the parameter sets when the picture is done, so
         * a channel's first picture goes by the command words and what
         * T31 libimp's own SPS / PPS say (no SAO, transform skip); the
         * ones later find the sets left in their buffers by an earlier
         * picture.  The deblocking offsets are the slice's (W10).
         */
        HevcEncParams prm = {
            .log2_ctb = f->log2_lcu,
            .sao = st->hevc_sps && st->hevc.sao,
            .beta_offset_div2 = sextract32(f->w10, 0, 5),
            .tc_offset_div2 = sextract32(f->w10, 8, 6),
            .chroma_qp_offset = f->cqp,
            .trafo_depth_intra = st->hevc_sps ? st->hevc.trafo_depth_intra
                                              : f->depth_intra,
            /* T31 libimp's PPS enables transform skip, T41's (gen 2) not */
            .transform_skip = st->hevc_pps ? st->hevc.transform_skip :
                              s->gen != 2,
            /* the slice header's cabac_init_flag: W8 [9:8] as for AVC */
            .cabac_init_flag = f->type != 2 && f->idc,
            /*
             * P pictures with real inter coding, as the parameter sets say
             * (before they are seen: libimp's -- default scaling lists,
             * TMVP, one level of inter transform split, no sign hiding);
             * MaxNumMergeCand is libimp's slice header's, 5
             */
            .inter = true,
            .scaling_list = st->hevc_sps ? st->hevc.scaling_list : true,
            .tmvp = st->hevc_sps ? st->hevc.tmvp : true,
            .trafo_depth_inter = st->hevc_sps ? st->hevc.trafo_depth_inter
                                              : 1,
            .max_merge_cand = 5,
            .sign_hiding = st->hevc_pps && st->hevc.sign_hiding,
        };
        HevcEncRef *recon = hevc_enc_ref_new();
        AvpuRef *ref = NULL, *rec;

        if (prm.log2_ctb < 4 || prm.log2_ctb > 6) {
            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: HEVC CTB %d\n",
                          1 << prm.log2_ctb);
            prm.log2_ctb = MIN(MAX(prm.log2_ctb, 4), 6);
        }
        if (f->nslices > 1) {
            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: HEVC picture of %d "
                          "slices\n", f->nslices);
        }
        while (w.nbits) {
            avpu_wr_bits(&w, 0, 1);
        }
        if (f->type != 2) {
            ref = avpu_ref_find(s, f->ref_key, f->w, f->h);
        }
        cap = (size_t)f->w * f->h * 2 + 1024;
        out = g_malloc(cap);
        /* the bytes must fit the room: again at a higher QP if not */
        for (;;) {
            bits = hevc_enc_slice_data(&prm, f->w, f->h, f->qp, f->type != 2,
                                       ref ? ref->hevc : NULL, recon,
                                       f->y, f->w, f->h, f->w, f->uv,
                                       f->w, out, cap);
            if (f->type == 2 || !room || bits + 256 <= room ||
                prm.all_skip) {
                break;
            }
            if (prm.cu_qp >= 51) {
                /* not even at QP 51: every CU skipped */
                prm.all_skip = true;
                continue;
            }
            prm.cu_qp = MIN((prm.cu_qp ? prm.cu_qp : MIN(f->qp, 22)) + 6,
                            51);
        }
        rec = avpu_ref_new(s, f->rec_key, f->w, f->h);
        rec->hevc = recon;
        {
            const uint8_t *ry, *rcb, *rcr;

            hevc_enc_ref_planes(recon, &ry, &rcb, &rcr);
            if (ry) {
                avpu_dump_rec(s, f, ry, rcb, rcr, NULL);
            }
        }
        /* already escaped, and its first byte follows a non-zero one */
        for (size_t i = 0; i < MIN(bits, cap); i++) {
            avpu_wr_raw(&w, out[i]);
        }
        sl->coded = true;
    } else {
        const int mbw = f->w / 16;
        const int r0 = sl->first / mbw;
        const int rows = MIN(MAX((sl->nlcu + mbw - 1) / mbw, 1),
                             f->h / 16 - r0);
        const bool cabac = f->cabac;
        const int cqp = f->cqp;
        const uint8_t *y = f->y + (size_t)r0 * 16 * f->w;
        const uint8_t *uv = f->uv + (size_t)r0 * 8 * f->w;
        const int nmb = mbw * rows;
        const int idc = extract32(e[0x70 / 4], 8, 2) ? : f->idc;

        if (sl->first % mbw || (!last_slice && sl->nlcu % mbw)) {
            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: slice of %d MBs from "
                          "MB %d: coded as whole rows\n", sl->nlcu, sl->first);
        }
        if (cabac) {
            while (w.nbits) {
                avpu_wr_bits(&w, 1, 1);     /* cabac_alignment_one_bit */
            }
        }
        if (f->type != 0) {
            /*
             * I and P slices with real coding, CABAC or CAVLC: the
             * picture's slices on one reconstruction (h264_pic_*), filtered
             * when the last is done and kept as the next picture's
             * reference.  The bytes have to fit the stream buffer's room
             * (the coder raises the QP).
             */
            size_t capb = (size_t)nmb * 800 + 4096;
            H264PStats pst;

            if (room) {
                capb = MIN(capb, room > 512 ? room - 256 : room / 2);
            }
            if (!f->pic) {
                AvpuRef *ref = f->type == 1 ?
                    avpu_ref_find(s, f->ref_key, f->w, f->h) : NULL;

                if (ref && !ref->y) {
                    ref = NULL;
                }
                f->p_ref_missing = f->type == 1 && !ref;
                f->pic = h264_pic_new(mbw, f->h / 16, f->y, f->w, f->uv, f->w,
                                      ref ? ref->y : NULL,
                                      ref ? ref->uv : NULL, cqp,
                                      st->h264_t8x8);
            }
            out = g_malloc0(capb);
            if (cabac) {
                bits = h264_pic_slice_cabac(f->pic, f->type == 1, r0, rows,
                                            f->qp, idc, out, capb, &pst) * 8;
                from = 0;
            } else {
                /* from the slice header's last bit in its byte */
                from = w.nbits;
                bits = h264_pic_slice_cavlc(f->pic, f->type == 1, r0, rows,
                                            f->qp, from, out, capb, &pst);
            }
            avpu_wr_buf_bits(&w, out, from, MIN(bits, capb * 8));
            sl->coded = true;
            if (last_slice) {
                /*
                 * disable_deblocking_filter_idc 0 and the slice header's
                 * offsets: W10 [13:8] alpha, [4:0] beta, halved -- what
                 * every slice header libimp writes says (the command
                 * words' bit that would turn the filter off is not known)
                 */
                const H264Deblock db = {
                    .idc = 0,
                    .alpha_off = 2 * sextract32(f->w10, 8, 6),
                    .beta_off = 2 * sextract32(f->w10, 0, 5),
                };
                AvpuRef *rec = avpu_ref_new(s, f->rec_key, f->w, f->h);

                rec->y = g_malloc((size_t)f->w * f->h);
                rec->uv = g_malloc((size_t)f->w * f->h / 2);
                h264_pic_finish(f->pic, &db, rec->y, rec->uv);
                h264_pic_free(f->pic);
                f->pic = NULL;
                avpu_dump_rec(s, f, rec->y, NULL, NULL, rec->uv);
            }
        } else {
            /* B: nothing changes (CABAC B skip contexts not modelled) */
            cap = (size_t)nmb + 64;
            out = g_malloc0(cap);
            if (cabac) {
                qemu_log_mask(LOG_UNIMP, "ingenic-avpu: CABAC B slice\n");
                bits = h264_slice_pskip_cabac(f->qp, idc, nmb, out, cap) * 8;
            } else {
                from = w.nbits;
                bits = h264_slice_pskip_cavlc(nmb, from, out, cap);
            }
            avpu_wr_buf_bits(&w, out, from, MIN(bits, cap * 8));
            sl->coded = true;
        }
    }
    while (w.nbits) {                           /* aligned already */
        avpu_wr_bits(&w, 0, 1);
    }

    /* a slice cut short at the room's end: the overflow bit (30) */
    if (by_enc1) {
        avpu_enc1_status(s, ea, f, sl->nlcu, w.count, w.count);
        if (w.full) {
            avpu_status(s, ea, 0x104, MIN(w.count, 0x3FFFFFFFu) | (1u << 30));
        }
        avpu_status(s, ea, 0x108, 0);
    } else {
        avpu_status(s, ea, 0x1E4, MIN(w.count, 0x3FFFFFFFu) |
                    (w.full ? 1u << 30 : 0));
        avpu_status(s, ea, 0x1E8, 0);
    }
    if (w.full) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: slice of %u bytes "
                      "in a room of %u\n", w.count, room);
    }
    if (off + w.count <= (size ? size : UINT32_MAX)) {
        avpu_dump(s, f, base, off, MIN(w.count, room));
    }
}

/* ---- Enc2 --------------------------------------------------------------- */

/* the picture and slice an Enc2 entry codes: by WE8, else the oldest */
static AvpuFrame *avpu_find_slice(IngenicAvpuState *s, uint32_t key,
                                  AvpuSlice **slp)
{
    AvpuFrame *oldest = NULL;
    int i, j;

    for (i = 0; i < AVPU_MAX_FRAMES; i++) {
        AvpuFrame *f = &s->frames[i];

        if (!f->used) {
            continue;
        }
        for (j = 0; j < f->nslices; j++) {
            if (!f->slices[j].coded && key && f->slices[j].key == key) {
                *slp = &f->slices[j];
                return f;
            }
        }
        if (!oldest || f->seq < oldest->seq) {
            oldest = f;
        }
    }
    if (oldest) {
        for (j = 0; j < oldest->nslices; j++) {
            if (!oldest->slices[j].coded) {
                *slp = &oldest->slices[j];
                return oldest;
            }
        }
    }
    return NULL;
}

static void avpu_frame_retire(AvpuFrame *f)
{
    int j;

    for (j = 0; j < f->nslices; j++) {
        if (!f->slices[j].coded) {
            return;
        }
    }
    avpu_frame_free(f);
}

static void avpu_enc2(IngenicAvpuState *s, hwaddr list)
{
    uint32_t e[CL_ENTRY / 4];
    int k;

    for (k = 0; k < CL_MAX_ENTRIES; k++) {
        const hwaddr ea = list + (hwaddr)k * avpu_entry_size(s);
        AvpuSlice *sl = NULL;
        AvpuFrame *f;

        avpu_load_entry(s, ea, e);
        avpu_trace_entry(s, "Enc2", ea, e);
        f = avpu_find_slice(s, e[0xE8 / 4], &sl);
        if (!f) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: Enc2 without a "
                          "picture from Enc1\n");
            avpu_status(s, ea, 0x1E4, 0);
            avpu_status(s, ea, 0x1E8, 0);
        } else {
            avpu_code_slice(s, f, sl, e, ea, false);
            avpu_frame_retire(f);
        }
        if (e[0] & (1u << 31)) {
            return;
        }
        if (!e[0] && !e[0x78 / 4] && !e[0xF0 / 4]) {
            return;                     /* past the slices: not a list */
        }
    }
}

/* ---- JPEG -------------------------------------------------------------- */

/* zigzag position -> natural index */
static const uint8_t avpu_zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * Gen 2 (T41) has the JPEG registers in the core's page, from 0x40, in
 * another order: R0 0x40, R1 0x44, R2 0x48, the pitch 0x74, the source
 * 0x80 / 0x88, the tables 0x58, the stream buffer base / size / offset /
 * room 0x60 / 0x68 / 0x6C / 0x70 (the odd words above each address are
 * its high half); status 0x600 and bytes 0x604 (CoreManager.c
 * AL_EncCore_ReadStatusRegsJpeg).
 */
static const uint16_t avpu_g2_jpeg[11] = {
    0x40, 0x44, 0x48, 0x74, 0x80, 0x88, 0x58, 0x60, 0x68, 0x6C, 0x70,
};

static void avpu_jpeg(IngenicAvpuState *s, int core)
{
    uint32_t *cregs = &s->regs[core * CORE_STRIDE / 4];
    uint32_t *page = &s->regs[G2_CORE(core) / 4];
    uint32_t r[11];
    int h, w, pitch, w2, h2;
    uint32_t base, size, off, room;
    uint8_t zz[128], ql[64], qc[64];
    g_autofree uint8_t *y = NULL, *uv = NULL, *cb = NULL, *cr = NULL;
    g_autofree uint8_t *scan = NULL;
    g_autoptr(GByteArray) hdr = g_byte_array_new();
    size_t n, cap, i;
    bool overflow = false;

    for (i = 0; i < ARRAY_SIZE(r); i++) {
        r[i] = s->gen == 2 ? page[avpu_g2_jpeg[i] / 4]
                           : cregs[R_JPEG(i) / 4];
    }
    h = extract32(r[1], 0, 16) + 1;
    w = extract32(r[1], 16, 16) + 1;
    base = r[7] & AVPU_DMA_MASK;
    size = r[8];
    off = r[9];
    room = r[10] ? r[10] : (size > off ? size - off : 0);
    pitch = extract32(r[3], 0, 16);
    w2 = ROUND_UP(w, 16);
    h2 = ROUND_UP(h, 16);
    if (pitch < w) {
        pitch = ROUND_UP(w, 16);
    }
    avpu_read(r[6], zz, sizeof(zz));
    for (i = 0; i < 64; i++) {
        ql[avpu_zigzag[i]] = zz[i] ? zz[i] : 1;
        qc[avpu_zigzag[i]] = zz[64 + i] ? zz[64 + i] : 1;
    }
    y = g_malloc((size_t)w2 * h2);
    uv = g_malloc((size_t)w2 * h2 / 2);
    cb = g_malloc((size_t)w2 * h2 / 4);
    cr = g_malloc((size_t)w2 * h2 / 4);
    avpu_load_nv12(r[4], r[5], pitch, w, h, w2, h2, y, uv);
    for (i = 0; i < (size_t)w2 * h2 / 4; i++) {
        cb[i] = uv[2 * i];
        cr[i] = uv[2 * i + 1];
    }
    jpeg_enc_header(hdr, w, h, ql, qc);
    cap = (size_t)w2 * h2 * 2 + 1024;
    scan = g_malloc(cap);
    n = jpeg_enc_scan(y, w2, cb, cr, w2 / 2, w, h, ql, qc, true, scan, cap,
                      &overflow);
    if (hdr->len + n > room) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: JPEG of %zu bytes in "
                      "a room of %u\n", hdr->len + n, room);
        overflow = true;
    }
    /* the stream buffer as written, wrapping at its size */
    {
        const uint32_t sz = size ? size : UINT32_MAX;
        const size_t total = MIN(hdr->len + n, (size_t)room);
        size_t done = 0;

        while (done < total) {
            const uint32_t pos = (off + done) % sz;
            const size_t chunk = MIN(total - done, (size_t)(sz - pos));
            const size_t h1 = done < hdr->len ?
                              MIN(chunk, hdr->len - done) : 0;

            if (h1) {
                avpu_write_mem(base + pos, hdr->data + done, h1);
            }
            if (chunk > h1) {
                avpu_write_mem(base + pos + h1,
                               scan + (done + h1 - hdr->len), chunk - h1);
            }
            done += chunk;
        }
        if (s->gen == 2) {
            page[0x604 / 4] = total;
        } else {
            cregs[R_JPEG_BYTES / 4] = total;
        }
    }
    if (s->gen == 2) {
        page[0x600 / 4] = 0;
    } else {
        cregs[R_JPEG_STATUS / 4] = 0;
        cregs[R_JPEG_FLAGS / 4] = overflow ? 2 : 0;
    }
    if (s->dump) {
        g_autofree char *path = g_strdup_printf("%s/avpu-%u.jpg", s->dump,
                                                s->jpeg_count);
        FILE *fp = fopen(path, "wb");

        if (fp) {
            fwrite(hdr->data, 1, hdr->len, fp);
            fwrite(scan, 1, n, fp);
            fclose(fp);
        }
    }
    s->jpeg_count++;
}

/* ---- jobs ---------------------------------------------------------------- */

static void avpu_done(void *opaque)
{
    IngenicAvpuState *s = opaque;
    uint32_t i;

    for (i = 0; i < s->njobs; i++) {
        switch (s->job_kind[i]) {
        case START_ENC1:
            avpu_enc1(s, s->job_addr[i]);
            break;
        case START_ENC2:
            avpu_enc2(s, s->job_addr[i]);
            break;
        case START_JPEG:
            avpu_jpeg(s, s->job_core[i]);
            break;
        }
    }
    s->njobs = 0;
    s->irq_status |= s->pending;
    s->pending = 0;
    avpu_update(s);
}

static void avpu_start(IngenicAvpuState *s, int core, uint32_t val)
{
    static const uint32_t kinds[] = { START_ENC1, START_ENC2, START_JPEG };
    const uint32_t *cr = &s->regs[core * CORE_STRIDE / 4];
    int i;

    if (s->trace) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: core %d start 0x%x, list "
                      "0x%x\n", core, val, cr[R_CL_ADDR / 4]);
        if (val & START_JPEG) {
            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: JPEG");
            for (i = 0; i < 11; i++) {
                qemu_log_mask(LOG_UNIMP, " R%d=%08x", i, cr[R_JPEG(i) / 4]);
            }
            qemu_log_mask(LOG_UNIMP, "\n");
        }
    }

    for (i = 0; i < ARRAY_SIZE(kinds); i++) {
        if (!(val & kinds[i])) {
            continue;
        }
        if (s->njobs == AVPU_MAX_JOBS) {
            /* run what is queued now: the guest outran the timer */
            avpu_done(s);
        }
        s->job_kind[s->njobs] = kinds[i];
        s->job_addr[s->njobs] = cr[R_CL_ADDR / 4] & AVPU_DMA_MASK;
        s->job_core[s->njobs] = core;
        s->njobs++;
        s->pending |= (kinds[i] == START_ENC2 ? IRQ_ENC2 : IRQ_ENC1) <<
                      (4 * core);
    }
    if (val & ~(START_ENC1 | START_ENC2 | START_JPEG)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: start 0x%x\n", val);
    }
    if (s->njobs && !timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            AVPU_JOB_NS);
    }
}

/* gen 2: a start in core c's page, Enc1's (or JPEG's) or Enc2's half */
static void avpu_start2(IngenicAvpuState *s, int core, bool enc2,
                        uint32_t val)
{
    const uint32_t list = s->regs[(G2_CORE(core) + (enc2 ? G2_ENC2 : 0)) / 4]
                          & AVPU_DMA_MASK;
    const uint32_t kind = enc2 ? START_ENC2 : (val & 2) ? START_ENC1 :
                          START_JPEG;

    if (s->trace) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: core %d %s start 0x%x, list "
                      "0x%x\n", core, enc2 ? "Enc2" : "Enc1", val, list);
        if (kind == START_JPEG) {
            int i;

            qemu_log_mask(LOG_UNIMP, "ingenic-avpu: JPEG");
            for (i = 0; i < 0x80; i += 4) {
                qemu_log_mask(LOG_UNIMP, " %02x=%08x", 0x40 + i,
                              s->regs[(G2_CORE(core) + 0x40 + i) / 4]);
            }
            qemu_log_mask(LOG_UNIMP, "\n");
        }
    }
    if (!(val & 3)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: start 0x%x\n", val);
        return;
    }
    if (s->njobs == AVPU_MAX_JOBS) {
        avpu_done(s);
    }
    s->job_kind[s->njobs] = kind;
    /* the entries begin Enc2's words before */
    s->job_addr[s->njobs] = enc2 ? list - G2_ENC2 : list;
    s->job_core[s->njobs] = core;
    s->njobs++;
    s->pending |= 1u << (2 * core + enc2);
    if (!timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            AVPU_JOB_NS);
    }
}

/* ---- registers ---------------------------------------------------------- */

static uint64_t avpu_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAvpuState *s = opaque;
    hwaddr off = addr - s->reg_base;

    if (addr < s->reg_base) {
        return 0;
    }
    if (s->gen == 2 && off >= G2_CORE(0)) {
        switch (off & (G2_ENC2 - 1)) {
        case G2_START:
        case G2_BUSY:
            return 0;
        }
        return s->regs[off / 4];
    }
    if (off >= R_JPEG(0)) {
        switch ((off - R_JPEG(0)) % CORE_STRIDE + R_JPEG(0)) {
        case R_BUSY:
        case R_START:
            return 0;
        }
        return s->regs[off / 4];
    }
    switch (off) {
    case R_IRQ_STATUS:
        return s->irq_status;
    case R_TIMER: {
        /* an 800 MHz count; never zero */
        const int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        return (uint32_t)(((ns - s->timer_latch) * 4) / 5) | 1;
    }
    }
    return s->regs[off / 4];
}

static void avpu_reg_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    IngenicAvpuState *s = opaque;
    hwaddr off = addr - s->reg_base;

    if (addr < s->reg_base) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: write 0x%" PRIx64
                      " below the registers at 0x%" HWADDR_PRIx "\n",
                      val, addr);
        return;
    }
    if (s->trace > 1 && off != R_TIMER_CTL) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: [0x%03" HWADDR_PRIx
                      "] = 0x%" PRIx64 "\n", off, val);
    }
    if (s->gen == 2 && off >= G2_CORE(0)) {
        const int core = off / G2_CORE(0) - 1;
        const bool enc2 = off & G2_ENC2;

        switch (off & (G2_ENC2 - 1)) {
        case G2_START:
            avpu_start2(s, core, enc2, val);
            return;
        case G2_BUSY:
            return;
        }
        s->regs[off / 4] = val;
        return;
    }
    if (off >= R_JPEG(0)) {
        const int core = (off - R_JPEG(0)) / CORE_STRIDE;

        switch ((off - R_JPEG(0)) % CORE_STRIDE + R_JPEG(0)) {
        case R_START:
            avpu_start(s, core, val);
            return;
        case R_BUSY:
        case R_JPEG_BYTES:
            return;
        }
        s->regs[off / 4] = val;
        return;
    }
    switch (off) {
    case R_IRQ_STATUS:
        s->irq_status &= ~val;
        avpu_update(s);
        return;
    case R_IRQ_MASK:
        s->regs[off / 4] = val;
        avpu_update(s);
        return;
    case R_TIMER:
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps avpu_ops = {
    .read = avpu_reg_read,
    .write = avpu_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void avpu_reset(DeviceState *dev)
{
    IngenicAvpuState *s = INGENIC_AVPU(dev);
    int i;

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    /* board UID, HW ID, revision, date (Board.c ShowBoardInformation) */
    s->regs[R_ID0 / 4] = 0x414c4c45;            /* "ELLA" */
    /*
     * the HW ID libimp's encoder library checks against the part's
     * ("Board ID mismatch: 5E20 instead of 72000460" on T31, 72000760 on
     * T40, 71000260 on T41): "hw-id"
     */
    s->regs[R_ID1 / 4] = s->hw_id;
    s->regs[R_ID2 / 4] = 0x00010000;
    s->regs[R_ID3 / 4] = 0x20220825;
    s->irq_status = 0;
    s->njobs = 0;
    s->pending = 0;
    s->timer_latch = 0;
    for (i = 0; i < AVPU_MAX_FRAMES; i++) {
        avpu_frame_free(&s->frames[i]);
    }
    for (i = 0; i < AVPU_MAX_REFS; i++) {
        avpu_ref_clear(&s->refs[i]);
    }
    memset(s->streams, 0, sizeof(s->streams));
    avpu_update(s);
}

static void avpu_realize(DeviceState *dev, Error **errp)
{
    IngenicAvpuState *s = INGENIC_AVPU(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    const uint32_t regs = s->gen == 2 ? AVPU_REGS_SIZE2 : AVPU_REGS_SIZE;

    if (s->gen < 1 || s->gen > 2) {
        error_setg(errp, "ingenic-avpu: gen %u is not 1 or 2", s->gen);
        return;
    }
    if (s->reg_base + regs > AVPU_MMIO_SIZE || s->reg_base & 3) {
        error_setg(errp, "ingenic-avpu: reg-base 0x%x out of the block",
                   s->reg_base);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &avpu_ops, s,
                          TYPE_INGENIC_AVPU, s->reg_base + regs);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, avpu_done, s);
}

static const Property avpu_props[] = {
    DEFINE_PROP_UINT32("reg-base", IngenicAvpuState, reg_base, 0x8000),
    DEFINE_PROP_UINT8("gen", IngenicAvpuState, gen, 1),
    DEFINE_PROP_UINT32("hw-id", IngenicAvpuState, hw_id, 0x72000460),
    DEFINE_PROP_STRING("dump", IngenicAvpuState, dump),
    DEFINE_PROP_UINT8("trace", IngenicAvpuState, trace, 0),
    DEFINE_PROP_UINT32("pcm-budget", IngenicAvpuState, pcm_budget, 256 * 1024),
};

/*
 * The model's pictures (Enc1's copies, the references) are not migrated:
 * after a migration P pictures code against no reference, i.e. as I_PCM
 * throughout, until the next IDR.
 */
static const VMStateDescription vmstate_avpu = {
    .name = TYPE_INGENIC_AVPU,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, IngenicAvpuState),
        VMSTATE_UINT32_ARRAY(regs, IngenicAvpuState, AVPU_NREGS),
        VMSTATE_UINT32(irq_status, IngenicAvpuState),
        VMSTATE_UINT32(njobs, IngenicAvpuState),
        VMSTATE_UINT32_ARRAY(job_kind, IngenicAvpuState, AVPU_MAX_JOBS),
        VMSTATE_UINT32_ARRAY(job_addr, IngenicAvpuState, AVPU_MAX_JOBS),
        VMSTATE_UINT32_ARRAY(job_core, IngenicAvpuState, AVPU_MAX_JOBS),
        VMSTATE_UINT32(pending, IngenicAvpuState),
        VMSTATE_UINT32(jpeg_count, IngenicAvpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void avpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = avpu_realize;
    device_class_set_legacy_reset(dc, avpu_reset);
    device_class_set_props(dc, avpu_props);
    dc->vmsd = &vmstate_avpu;
}

static const TypeInfo avpu_info = {
    .name           = TYPE_INGENIC_AVPU,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(IngenicAvpuState),
    .class_init     = avpu_class_init,
};

static void avpu_register_types(void)
{
    type_register_static(&avpu_info);
}

type_init(avpu_register_types)
