/*
 * Novatek NA51089 HDAL media engines: the register blocks the HDAL's
 * kdrv_* modules drive -- video capture (SIE, TGE, CSI, LVDS, sensor PHY),
 * the IPP chain (IFE, DCE, IPE, IME), 2D graphics and ISE, the H.26x and
 * JPEG codecs, audio (EAC, DAI), the serial interface SIF and the DDR
 * arbiter's protection window.
 *
 * One device type serves them all: an "engine" property picks the
 * engine's entry in the table below, which says how its registers behave
 * beyond plain storage -- reset bits that clear themselves, status the
 * driver polls, the start bit whose work is done at once and raises the
 * engine's interrupt.  No pixels, samples or bitstreams move: the engines
 * accept their setup and report each job finished, which is what the
 * drivers and the HDAL flows wait for (docs/novatek.md).
 *
 * "trace" logs every access, for working out what an engine's driver
 * expects: -global nvt-media.trace=on, or per instance.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "system/dma.h"
#include "hw/misc/nvt-pixel.h"

#define TYPE_NVT_MEDIA "nvt-media"
OBJECT_DECLARE_SIMPLE_TYPE(NvtMediaState, NVT_MEDIA)

#define NVT_MEDIA_MAX_IRQS  3

typedef struct NvtMediaEngine NvtMediaEngine;

struct NvtMediaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[NVT_MEDIA_MAX_IRQS];
    uint32_t *regs;
    char *engine_name;
    uint32_t size;
    bool trace;
    const NvtMediaEngine *engine;
    QEMUTimer *timer;           /* frame / job timing, for engines that run */
    uint32_t phase;             /* where in its frame or job the engine is */
    uint32_t ll_addr;           /* IPP: the linked list being worked through */
    uint32_t frames;            /* SIE: frames captured, for the pattern */
    NvtMediaState *ipp_src;     /* IME: the IFE whose input it finishes */
    uint8_t jpg_q[128];         /* JPEG: luma / chroma quantisation, zig-zag */
    uint32_t jpg_lut;           /* JPEG: the LUT port's select / address */
};

/*
 * What an engine does past storage.  'write' sees the new value before it
 * is stored and returns what to store; 'read' may compute a register.
 */
struct NvtMediaEngine {
    const char *name;
    uint64_t (*write)(NvtMediaState *s, hwaddr off, uint64_t val);
    bool (*read)(NvtMediaState *s, hwaddr off, uint64_t *val);
    void (*reset)(NvtMediaState *s);
    void (*timer)(NvtMediaState *s);
    void (*update_irq)(NvtMediaState *s);
    const void *opaque;         /* the engine's own constants */
};

static uint32_t *reg(NvtMediaState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_media_update_irq(NvtMediaState *s)
{
    if (s->engine->update_irq) {
        s->engine->update_irq(s);
    }
}

/*
 * SIE, the sensor interface (kdrv_builtin/sie/sie_reg.h): R0 control --
 * bit 0 reset, bit 1 load (shadow registers take effect; clears itself),
 * bit 2 ACT_EN, the engine capturing, bit 24 DMA channel disable, bit 25
 * DMA channel idle; R8 interrupt enables, RC status (write 1 to clear):
 * VD bit 0, line break points 1..3 bits 1..3, active / crop window start
 * bits 4 / 5, DRAM output 0..5 end bits 7..12, active / crop end bits
 * 16 / 17.  While capturing, a frame comes every 1/30 s in three steps, as
 * the sensor's lines would arrive: VD and the window starts, the break
 * points half way, then the window ends and the DRAM outputs' ends.
 * An output in single mode (RA4 bits 28..30) writes one frame, and its
 * single-enable bit then clears: that is how ctl_sie knows the buffer is
 * complete and hands it on.  Nothing reaches DRAM.
 */
#define SIE_CTRL            0x00
#define SIE_INTE            0x08
#define SIE_INTS            0x0C
#define SIE_CROP_SIZE       0x2C
#define SIE_OUT0_ADDR       0x5C
#define SIE_OUT0_CFG        0x60
#define SIE_DRAM            0xA4
#define SIE_DRAM_SINGLE     (7u << 28)
#define SIE_CTRL_RST        (1u << 0)
#define SIE_CTRL_LOAD       (1u << 1)
#define SIE_CTRL_ACT_EN     (1u << 2)
#define SIE_CTRL_DMA_DIS    (1u << 24)
#define SIE_CTRL_DMA_IDLE   (1u << 25)
#define SIE_INT_START       (0x1u | 0x30)               /* VD, ACT/CRP start */
#define SIE_INT_BP          0xEu                        /* BP1..3 */
#define SIE_INT_END         (0x1F80u | 0x30000)         /* DRAM out, ends */
#define SIE_FRAME_NS        (NANOSECONDS_PER_SECOND / 30)

static void sie_update_irq(NvtMediaState *s)
{
    qemu_set_irq(s->irq[0], !!(*reg(s, SIE_INTS) & *reg(s, SIE_INTE)));
}

static uint64_t sie_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    uint32_t old = *reg(s, off);

    switch (off) {
    case SIE_CTRL:
        if ((val & SIE_CTRL_ACT_EN) && !(old & SIE_CTRL_ACT_EN)) {
            s->phase = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      SIE_FRAME_NS / 3);
        } else if (!(val & SIE_CTRL_ACT_EN)) {
            timer_del(s->timer);
        }
        return val & ~(SIE_CTRL_RST | SIE_CTRL_LOAD);
    case SIE_INTS:
        val = old & ~val;
        *reg(s, off) = val;
        sie_update_irq(s);
        return val;
    }
    return val;
}

static bool sie_read(NvtMediaState *s, hwaddr off, uint64_t *val)
{
    if (off == SIE_CTRL) {
        *val |= SIE_CTRL_DMA_IDLE;
        return true;
    }
    return false;
}

static const unsigned raw_bits[4] = { 8, 10, 12, 16 };

/*
 * The sensor's frame, as the SIE writes it to DRAM output 0: the test
 * pattern (nvt-pixel.c) at the crop size, packed as OUT0_CFG [17:16]
 * says (8 / 10 / 12 / 16 bits), OUT0_CFG [15:2] bytes a line apart.
 */
static void sie_write_raw(NvtMediaState *s)
{
    uint32_t size = *reg(s, SIE_CROP_SIZE), cfg = *reg(s, SIE_OUT0_CFG);
    uint32_t w = size & 0xFFFF, h = size >> 16, lofs = cfg & 0xFFFC;
    unsigned bits = raw_bits[(cfg >> 16) & 3];
    hwaddr addr = *reg(s, SIE_OUT0_ADDR) & ~3u;
    size_t line = nvt_pixel_packed_size(w, bits);
    g_autofree uint16_t *px = NULL;
    g_autofree uint8_t *packed = NULL;
    uint32_t y;

    if (!addr || !w || !h || w > 8192 || h > 8192 || lofs < line) {
        return;
    }
    px = g_new(uint16_t, w);
    packed = g_malloc(line);
    for (y = 0; y < h; y++) {
        nvt_pixel_pattern_line(px, w, h, y, s->frames);
        nvt_pixel_pack(packed, px, w, bits);
        dma_memory_write(&address_space_memory, addr + (hwaddr)y * lofs,
                         packed, line, MEMTXATTRS_UNSPECIFIED);
    }
}

static void sie_frame(NvtMediaState *s)
{
    static const uint32_t step[3] = {
        SIE_INT_START, SIE_INT_BP, SIE_INT_END,
    };

    if (s->phase == 2) {
        sie_write_raw(s);
        s->frames++;
        *reg(s, SIE_DRAM) &= ~SIE_DRAM_SINGLE;
    }
    *reg(s, SIE_INTS) |= step[s->phase];
    s->phase = (s->phase + 1) % 3;
    sie_update_irq(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              SIE_FRAME_NS / 3);
}

/*
 * The IPP engines -- IFE, DCE, IPE, IME (kdrv_videoprocess, each
 * engine's eng/include/<engine>_eng_int_reg.h) -- as kdrv_ipp drives
 * them, in linked-list mode.  It writes a list of commands to DRAM, its
 * address to the engine, and fires it (control bit 28).  A command is 64
 * bits, its type in [63:61] (kdrv_ipp_int_util.h KDRV_IPP_LL_CMD):
 *
 *   4 UPD     write [31:0] to register [43:32], bytes [47:44]
 *   2 NXTUPD  go on reading at [39:8]
 *   1 NXTLL   this job is set up; the next list is at [39:8]
 *   0 NULL    the last job is set up; the list ends
 *
 * The updates go through the engine's own registers -- they include its
 * interrupt enables.  Each job is a frame: some 1 ms after the fire (and
 * after each job) the engine has run it and raises frame and job end,
 * and list end with the last.  No pixels move.  Status is write 1 to
 * clear; the interrupt is status & enable.
 */
typedef struct {
    hwaddr inte;
    hwaddr ints;
    hwaddr ll_addr;             /* the list's address register */
    uint32_t job_end;           /* status bits a finished job raises */
    uint32_t ll_end;            /* and the list's end */
    uint32_t self_clear;        /* control bits done as soon as written */
    uint32_t trig;              /* control bit of a single, list-less job */
    uint32_t trig_end;          /* and its status */
} NvtIppParams;

#define IPP_CTRL            0x00
#define IPP_LL_FIRE         (1u << 28)
#define IPP_JOB_NS          (1000 * 1000)

#define IPP_LL_MAX_CMDS     4096
#define IPP_CMD_UPD         4
#define IPP_CMD_NXTUPD      2
#define IPP_CMD_NXTLL       1
#define IPP_CMD_NULL        0

/* frame start / end and job end; list end; ... */
static const NvtIppParams ipp_ife = { 0x08, 0x0C, 0x50, 0x41, 0x08 };
static const NvtIppParams ipp_dce = { 0x08, 0x0C, 0x1C, 0x803, 0x100 };
static const NvtIppParams ipp_ipe = { 0x50, 0x54, 0x1C, 0x20A, 0x100 };
static const NvtIppParams ipp_ime = { 0x18, 0x1C, 0x14, 0xA0000008, 0x1 };
/*
 * ISE, the scaling engine (kdrv_gfx2d/kdrv_ise/eng/include/
 * ise_eng_int_reg.h): list address at 0x2C, list end status bit 0, frame
 * end bit 31; control bit 1 runs a single job.
 */
static const NvtIppParams ipp_ise = {
    0x08, 0x0C, 0x2C, 1u << 31, 0x1, 0, 1u << 1, 1u << 31,
};

/*
 * The 2D graphics engine (kdrv_gfx2d/kdrv_grph: graphic_reg.h, grph_ll.h)
 * takes the same lists: list address at 0xD0; a finished list raises list
 * end (status bit 1).  Control bit 0 runs a single job instead, done in
 * status bit 0.  DMA idle (bit 4) is what an abort ends with -- graph_isr
 * reports it as one -- so neither raises it.  Software reset (bit 1) and
 * list terminate (bit 29) take effect at once.
 */
static const NvtIppParams ipp_grph = {
    0x08, 0x0C, 0xD0, 0x00, 0x02, (1u << 1) | (1u << 29), 1u << 0, 0x01,
};

static void ipp_update_irq(NvtMediaState *s)
{
    const NvtIppParams *p = s->engine->opaque;

    qemu_set_irq(s->irq[0], !!(*reg(s, p->ints) & *reg(s, p->inte)));
}

static uint64_t ipp_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    const NvtIppParams *p = s->engine->opaque;

    if (off == IPP_CTRL) {
        if (val & IPP_LL_FIRE) {
            s->ll_addr = *reg(s, p->ll_addr);
            s->phase = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      IPP_JOB_NS);
        } else if (val & p->trig) {
            s->phase = 1;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      IPP_JOB_NS);
        }
        return val & ~(IPP_LL_FIRE | p->self_clear);
    }
    if (off == p->ints) {
        return *reg(s, off) & ~val;
    }
    return val;
}

static void nvt_media_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size);

/* Apply one list's updates up to the end of its job; false at its end. */
static bool ipp_run_job(NvtMediaState *s)
{
    hwaddr addr = s->ll_addr;
    uint64_t cmd;
    uint32_t off, mask, i;
    int n;

    for (n = 0; n < IPP_LL_MAX_CMDS; n++, addr += 8) {
        if (dma_memory_read(&address_space_memory, addr, &cmd, 8,
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-media %s: linked list at "
                          "0x%" HWADDR_PRIx " unreadable\n",
                          s->engine_name, addr);
            return false;
        }
        cmd = le64_to_cpu(cmd);
        switch (cmd >> 61) {
        case IPP_CMD_UPD:
            off = (cmd >> 32) & 0xFFC;
            if (off >= s->size) {
                break;
            }
            for (mask = 0, i = 0; i < 4; i++) {
                if (cmd & (1ull << (44 + i))) {
                    mask |= 0xFFu << (8 * i);
                }
            }
            nvt_media_write(s, off, (*reg(s, off) & ~mask) |
                            ((uint32_t)cmd & mask), 4);
            break;
        case IPP_CMD_NXTUPD:
            addr = ((cmd >> 8) & 0xFFFFFFFF) - 8;
            break;
        case IPP_CMD_NXTLL:
            s->ll_addr = (cmd >> 8) & 0xFFFFFFFF;
            return true;
        case IPP_CMD_NULL:
            return false;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-media %s: linked list "
                          "command 0x%016" PRIx64 "\n", s->engine_name, cmd);
            return false;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "nvt-media %s: linked list without an "
                  "end\n", s->engine_name);
    return false;
}

/*
 * The IPP chain's pixels, done when IME finishes a job: the RAW frame the
 * IFE was pointed at (input address 0x30, line offset 0x34, size 0x20,
 * sample depth CTRL1 [6:5]) becomes YUV 4:2:0 semi-planar at IME output
 * path 1 (enable 0x04 bit 2; size 0x84, Y / UV addresses 0x94 / 0x98,
 * line offsets 0x8C / 0x90): demosaiced 2x2, BT.601, scaled to the
 * output size.  DCE's and IPE's corrections are not applied.
 */
static void ime_write_yuv(NvtMediaState *s)
{
    NvtMediaState *ife = s->ipp_src;
    uint32_t isz, osz, iw, ih, ow, oh, ilofs, ylofs, uvlofs, y;
    unsigned bits;
    hwaddr in, ya, uva;
    g_autofree uint16_t *bayer = NULL;
    g_autofree uint8_t *packed = NULL, *yp = NULL, *uvp = NULL;
    size_t line;

    if (!ife || !(*reg(s, 0x04) & (1u << 2))) {
        return;
    }
    isz = *reg(ife, 0x20);
    osz = *reg(s, 0x84);
    iw = isz & 0xFFFF;
    ih = isz >> 16;
    ow = osz & 0xFFFF;
    oh = osz >> 16;
    ilofs = *reg(ife, 0x34) & 0xFFFC;
    ylofs = *reg(s, 0x8C) & 0xFFFC;
    uvlofs = *reg(s, 0x90) & 0xFFFC;
    bits = raw_bits[(*reg(ife, 0x04) >> 5) & 3];
    in = *reg(ife, 0x30) & ~3u;
    ya = *reg(s, 0x94) & ~3u;
    uva = *reg(s, 0x98) & ~3u;
    line = nvt_pixel_packed_size(iw, bits);
    if (!in || !ya || !uva || iw < 2 || ih < 2 || iw > 8192 || ih > 8192 ||
        ow < 2 || oh < 2 || ow > 8192 || oh > 8192 || ilofs < line ||
        ylofs < ow || uvlofs < ow) {
        return;
    }
    bayer = g_new(uint16_t, (size_t)iw * ih);
    packed = g_malloc(line);
    for (y = 0; y < ih; y++) {
        dma_memory_read(&address_space_memory, in + (hwaddr)y * ilofs,
                        packed, line, MEMTXATTRS_UNSPECIFIED);
        nvt_pixel_unpack(bayer + (size_t)y * iw, packed, iw, bits);
    }
    yp = g_malloc((size_t)ow * oh);
    uvp = g_malloc((size_t)ow * (oh / 2));
    nvt_pixel_bayer_to_nv12(bayer, iw & ~1u, ih & ~1u, yp, ow, uvp, ow,
                            ow & ~1u, oh & ~1u);
    for (y = 0; y < oh; y++) {
        dma_memory_write(&address_space_memory, ya + (hwaddr)y * ylofs,
                         yp + (size_t)y * ow, ow, MEMTXATTRS_UNSPECIFIED);
    }
    for (y = 0; y < oh / 2; y++) {
        dma_memory_write(&address_space_memory, uva + (hwaddr)y * uvlofs,
                         uvp + (size_t)y * ow, ow, MEMTXATTRS_UNSPECIFIED);
    }
}

static void ipp_job_done(NvtMediaState *s)
{
    const NvtIppParams *p = s->engine->opaque;
    bool more;

    if (s->phase) {
        /* a single job: its trigger bit drops when it is done */
        *reg(s, IPP_CTRL) &= ~p->trig;
        *reg(s, p->ints) |= p->trig_end;
        ipp_update_irq(s);
        return;
    }
    more = ipp_run_job(s);
    if (s->engine->opaque == &ipp_ime) {
        ime_write_yuv(s);
    }

    *reg(s, p->ints) |= p->job_end | (more ? 0 : p->ll_end);
    ipp_update_irq(s);
    if (more) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  IPP_JOB_NS);
    }
}

/*
 * The H.26x codec (kdrv_videocodec/vpu/inc/h26x_reg.h, h26x.h): CTRL at
 * 0x000 -- bit 4 software reset, bit 8 start, bit 12 busy, bit 27 DMA
 * channel disable -- INT_FLAG at 0x004 (write 1 to clear), INT_EN at
 * 0x01C.  Flags: picture finish bit 0, bitstream DMA bit 4, error bit 8,
 * bitstream buffer full bit 12, reset finish bit 28, DMA channel disable
 * finish bit 30.  The interrupt is INT_FLAG & INT_EN.
 */
#define H26X_CTRL           0x000
#define H26X_INT_FLAG       0x004
#define H26X_INT_EN         0x01C
#define H26X_CTRL_SWRST     (1u << 4)
#define H26X_CTRL_START     (1u << 8)
#define H26X_CTRL_DMA_DIS   (1u << 27)
#define H26X_INT_FINISH     (1u << 0)
#define H26X_INT_SWRST      (1u << 28)
#define H26X_INT_DMA_DIS    (1u << 30)

static void h26x_update_irq(NvtMediaState *s)
{
    qemu_set_irq(s->irq[0],
                 !!(*reg(s, H26X_INT_FLAG) & *reg(s, H26X_INT_EN)));
}

static uint64_t h26x_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    switch (off) {
    case H26X_CTRL:
        if (val & H26X_CTRL_SWRST) {
            *reg(s, H26X_INT_FLAG) |= H26X_INT_SWRST;
        }
        if ((val & H26X_CTRL_DMA_DIS) &&
            !(*reg(s, off) & H26X_CTRL_DMA_DIS)) {
            *reg(s, H26X_INT_FLAG) |= H26X_INT_DMA_DIS;
        }
        if (val & H26X_CTRL_START) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      IPP_JOB_NS);
        }
        return val & ~(H26X_CTRL_SWRST | H26X_CTRL_START);
    case H26X_INT_FLAG:
        return *reg(s, off) & ~val;
    }
    return val;
}

#define H26X_NAL_LEN_ADDR   0x068
#define H26X_BSOUT_ADDR     0x070
#define H26X_BSOUT_SIZE     0x07C
#define H26X_REPORT         0x250   /* CHK_REPORT[], H26XREPORT order */
#define H26X_RPT_BS_LEN     6
#define H26X_RPT_RRC_BITS   10
#define H26X_FILLER_LEN     64

/*
 * A picture "encoded": a filler NAL unit (H.265 FD_NUT, which decoders
 * skip) stands for the slice, its length in the report and the NAL
 * length list.
 */
static void h26x_done(NvtMediaState *s)
{
    uint8_t nal[H26X_FILLER_LEN];
    uint32_t len = MIN(H26X_FILLER_LEN, *reg(s, H26X_BSOUT_SIZE));
    uint32_t le;

    if (len < 8) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-media h26x: a %u-byte "
                      "bitstream buffer\n", *reg(s, H26X_BSOUT_SIZE));
        len = 0;
    } else {
        memset(nal, 0xFF, sizeof(nal));
        nal[0] = 0;
        nal[1] = 0;
        nal[2] = 0;
        nal[3] = 1;
        nal[4] = 38 << 1;           /* FD_NUT */
        nal[5] = 1;
        nal[len - 1] = 0x80;        /* rbsp trailing bits */
        le = cpu_to_le32(len);
        dma_memory_write(&address_space_memory, *reg(s, H26X_BSOUT_ADDR),
                         nal, len, MEMTXATTRS_UNSPECIFIED);
        if (*reg(s, H26X_NAL_LEN_ADDR)) {
            dma_memory_write(&address_space_memory,
                             *reg(s, H26X_NAL_LEN_ADDR), &le, 4,
                             MEMTXATTRS_UNSPECIFIED);
        }
    }
    *reg(s, H26X_REPORT + 4 * H26X_RPT_BS_LEN) = len;
    *reg(s, H26X_REPORT + 4 * H26X_RPT_RRC_BITS) = len * 8;
    *reg(s, H26X_INT_FLAG) |= H26X_INT_FINISH;
    h26x_update_irq(s);
}

/*
 * The JPEG codec (kdrv_videocodec/jpeg/include/jpeg_reg.h): CTRL at 0x00 --
 * bit 0 act (start a frame), bit 3 software reset (clears itself), bit 4
 * DMA abort, bit 5 DMA idle -- INT_EN / INT_STA at 0x18 / 0x1C (frame end
 * bit 0; write 1 to clear), the bitstream buffer at 0x40 / 0x44 and the
 * length coded at 0x4C, the MCU count at 0x2C.  A frame encodes as a flat
 * grey picture: every block "DC difference 0, end of block" in the
 * standard (Annex K) Huffman codes, four luma and two chroma blocks per
 * 4:2:0 MCU, then EOI after the software's header -- a valid JPEG of the
 * frame's size, if not of its pixels.
 */
#define JPG_CTRL            0x00
#define JPG_INT_EN          0x18
#define JPG_INT_STA         0x1C
#define JPG_LUT             0x10
#define JPG_LUT_DATA        0x14
#define JPG_DATA_SIZE       0x28
#define JPG_MCU             0x2C
#define JPG_Y_ADDR          0x54
#define JPG_UV_ADDR         0x58
#define JPG_Y_LOFS          0x60
#define JPG_UV_LOFS         0x64
#define JPG_BS_STADR        0x40
#define JPG_BS_BUFLEN       0x44
#define JPG_BS_CURADR       0x48
#define JPG_BS_CODECLEN     0x4C
#define JPG_CTRL_ACT        (1u << 0)
#define JPG_CTRL_SRST       (1u << 3)
#define JPG_CTRL_ABORT      (1u << 4)
#define JPG_CTRL_DMA_IDLE   (1u << 5)
#define JPG_INT_FRAME_END   (1u << 0)
#define JPG_INT_ABORT_DONE  (1u << 5)

static void jpg_update_irq(NvtMediaState *s)
{
    qemu_set_irq(s->irq[0],
                 !!(*reg(s, JPG_INT_STA) & *reg(s, JPG_INT_EN)));
}

static uint64_t jpg_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    switch (off) {
    case JPG_CTRL:
        if (val & JPG_CTRL_ABORT) {
            timer_del(s->timer);
            *reg(s, JPG_INT_STA) |= JPG_INT_ABORT_DONE;
            val &= ~JPG_CTRL_ACT;
        } else if ((val & JPG_CTRL_ACT) && !(*reg(s, off) & JPG_CTRL_ACT)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      IPP_JOB_NS);
        }
        return val & ~(JPG_CTRL_SRST | JPG_CTRL_ABORT);
    case JPG_INT_STA:
        return *reg(s, off) & ~val;
    case JPG_LUT:
        s->jpg_lut = val;
        return val;
    case JPG_LUT_DATA:
        /* table 1: the quantisation, luma then chroma, 4 bytes a word */
        if (((s->jpg_lut >> 12) & 0xF) == 1 && (s->jpg_lut & 0x3FF) < 32) {
            stl_le_p(s->jpg_q + 4 * (s->jpg_lut & 0x3FF), val);
        }
        if (s->jpg_lut & (1u << 16)) {      /* auto-increment */
            s->jpg_lut = (s->jpg_lut & ~0x3FFu) | ((s->jpg_lut + 1) & 0x3FF);
        }
        return val;
    }
    return val;
}

static bool jpg_read(NvtMediaState *s, hwaddr off, uint64_t *val)
{
    if (off == JPG_CTRL) {
        *val |= JPG_CTRL_DMA_IDLE;
        return true;
    }
    return false;
}

/*
 * A frame: MCU columns / rows in DATA_SIZE [12:0] / [28:16] and the MCU
 * count, each stored minus one; the 4:2:0 picture at Y / UV (line
 * offsets minus one).  It is encoded with the quantisation the driver
 * loaded and the standard Huffman tables; with no picture to read, every
 * block is coded flat grey ("DC difference 0, end of block": 4 bytes an
 * MCU).  Either way the scan ends in EOI after the software's header.
 */
static size_t jpg_encode(NvtMediaState *s, uint8_t *buf, size_t room)
{
    uint32_t sz = *reg(s, JPG_DATA_SIZE);
    uint32_t mw = (sz & 0x1FFF) + 1, mh = ((sz >> 16) & 0x1FFF) + 1;
    uint32_t ylofs = (*reg(s, JPG_Y_LOFS) & 0xFFFF) + 1;
    uint32_t uvlofs = (*reg(s, JPG_UV_LOFS) & 0xFFFF) + 1;
    hwaddr ya = *reg(s, JPG_Y_ADDR), uva = *reg(s, JPG_UV_ADDR);
    uint32_t w = mw * 16, h = mh * 16, r;
    g_autofree uint8_t *yp = NULL, *uvp = NULL;

    if (!ya || !uva || mw > 512 || mh > 512 || ylofs < w || uvlofs < w) {
        static const uint8_t grey[4] = { 0x28, 0xA2, 0x8A, 0x00 };
        uint32_t mcus = (*reg(s, JPG_MCU) & 0x3FFFFFF) + 1, i;

        if ((uint64_t)mcus * 4 + 2 > room) {
            return 0;
        }
        for (i = 0; i < mcus; i++) {
            memcpy(buf + 4 * i, grey, 4);
        }
        buf[4 * mcus] = 0xFF;
        buf[4 * mcus + 1] = 0xD9;
        return 4 * mcus + 2;
    }
    yp = g_malloc((size_t)w * h);
    uvp = g_malloc((size_t)w * h / 2);
    for (r = 0; r < h; r++) {
        dma_memory_read(&address_space_memory, ya + (hwaddr)r * ylofs,
                        yp + (size_t)r * w, w, MEMTXATTRS_UNSPECIFIED);
    }
    for (r = 0; r < h / 2; r++) {
        dma_memory_read(&address_space_memory, uva + (hwaddr)r * uvlofs,
                        uvp + (size_t)r * w, w, MEMTXATTRS_UNSPECIFIED);
    }
    return nvt_pixel_jpeg_scan(buf, room, yp, w, uvp, w, mw, mh,
                               s->jpg_q, s->jpg_q + 64);
}

static void jpg_done(NvtMediaState *s)
{
    uint32_t room = *reg(s, JPG_BS_BUFLEN) & 0x1FFFFFF;
    hwaddr bs = *reg(s, JPG_BS_STADR) & 0x7FFFFFFF;
    g_autofree uint8_t *buf = g_malloc(MAX(room, 1u));
    size_t len = jpg_encode(s, buf, room);

    if (!len) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-media jpg: the frame does not "
                      "fit the %u-byte buffer\n", room);
    } else {
        dma_memory_write(&address_space_memory, bs, buf, len,
                         MEMTXATTRS_UNSPECIFIED);
    }
    *reg(s, JPG_BS_CODECLEN) = len;
    *reg(s, JPG_BS_CURADR) = bs + len;
    *reg(s, JPG_CTRL) &= ~JPG_CTRL_ACT;
    *reg(s, JPG_INT_STA) |= JPG_INT_FRAME_END;
    jpg_update_irq(s);
}

/*
 * EAC, the embedded audio codec (kdrv_builtin/audio/eac/include/
 * eac_reg.h): two strobes the driver waits on to clear -- CTRL1 bit 0,
 * LOAD (the settings take effect), and EADC_CTRL0 bit 13,
 * ADC_DCCAN_INIT_LOAD (the DC cancellation's initial value is loaded).
 */
#define EAC_CTRL1           0x04
#define EAC_CTRL1_LOAD      (1u << 0)
#define EAC_EADC_CTRL0      0x10
#define EAC_DCCAN_INIT_LOAD (1u << 13)

static uint64_t eac_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    switch (off) {
    case EAC_CTRL1:
        return val & ~EAC_CTRL1_LOAD;
    case EAC_EADC_CTRL0:
        return val & ~EAC_DCCAN_INIT_LOAD;
    }
    return val;
}

/*
 * DAI, the audio interface's DMA (kdrv_builtin/audio/dai/include/
 * dai_reg.h): CTRL at 0x00 -- bit 0 DAIEN, bit 4 / 6 TX1 / RX DMA enable;
 * INTEN / INTSTATUS at 0x20 / 0x28 (write 1 to clear), TXLB's at 0x24 /
 * 0x2C; RX1 buffer address / size at 0x48 / 0x68, TX1's at 0x30 / 0x50.
 * The DMA is double-buffered: enabling it latches the buffer address
 * (DMALOAD: TX1 bit 4, RX bit 6), the driver queues the next, and each
 * buffer done raises buffer-full (TX1 bit 0, RX1 bit 2) and the next
 * load; disabling it raises stop (TX1 bit 8, RX bit 10).  Buffers last
 * as long as 16-bit mono at 16 kHz takes; capture fills them with
 * silence.
 */
#define DAI_CTRL            0x00
#define DAI_INTEN           0x20
#define DAI_INTEN2          0x24
#define DAI_INTSTS          0x28
#define DAI_INTSTS2         0x2C
#define DAI_TX1_ADDR        0x30
#define DAI_RX1_ADDR        0x48
#define DAI_TX1_SIZE        0x50
#define DAI_RX1_SIZE        0x68
#define DAI_CTRL_TX1_DMA    (1u << 4)
#define DAI_CTRL_RX_DMA     (1u << 6)
#define DAI_INT_TX1_BFI     (1u << 0)
#define DAI_INT_RX1_BFI     (1u << 2)
#define DAI_INT_TX1_LOAD    (1u << 4)
#define DAI_INT_RX_LOAD     (1u << 6)
#define DAI_INT_TX1_STOP    (1u << 8)
#define DAI_INT_RX_STOP     (1u << 10)
#define DAI_BYTES_PER_SEC   32000

static void dai_update_irq(NvtMediaState *s)
{
    qemu_set_irq(s->irq[0],
                 (*reg(s, DAI_INTSTS) & *reg(s, DAI_INTEN)) ||
                 (*reg(s, DAI_INTSTS2) & *reg(s, DAI_INTEN2)));
}

/* A buffer's length in bytes: the size registers count words. */
static uint32_t dai_buf_bytes(NvtMediaState *s, hwaddr size_reg)
{
    return MAX((*reg(s, size_reg) & 0x7FFFF) * 4, 4);
}

static void dai_schedule(NvtMediaState *s)
{
    uint32_t ctrl = *reg(s, DAI_CTRL), bytes = UINT32_MAX;

    if (ctrl & DAI_CTRL_RX_DMA) {
        bytes = dai_buf_bytes(s, DAI_RX1_SIZE);
    }
    if (ctrl & DAI_CTRL_TX1_DMA) {
        bytes = MIN(bytes, dai_buf_bytes(s, DAI_TX1_SIZE));
    }
    if (bytes == UINT32_MAX) {
        timer_del(s->timer);
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64(bytes, NANOSECONDS_PER_SECOND, DAI_BYTES_PER_SEC));
}

static uint64_t dai_write(NvtMediaState *s, hwaddr off, uint64_t val)
{
    uint32_t old = *reg(s, off), on, off_bits;

    switch (off) {
    case DAI_CTRL:
        on = val & ~old;
        off_bits = old & ~val;
        if (on & DAI_CTRL_RX_DMA) {
            *reg(s, DAI_INTSTS) |= DAI_INT_RX_LOAD;
        }
        if (on & DAI_CTRL_TX1_DMA) {
            *reg(s, DAI_INTSTS) |= DAI_INT_TX1_LOAD;
        }
        if (off_bits & DAI_CTRL_RX_DMA) {
            *reg(s, DAI_INTSTS) |= DAI_INT_RX_STOP;
        }
        if (off_bits & DAI_CTRL_TX1_DMA) {
            *reg(s, DAI_INTSTS) |= DAI_INT_TX1_STOP;
        }
        *reg(s, off) = val;
        if ((on | off_bits) & (DAI_CTRL_RX_DMA | DAI_CTRL_TX1_DMA)) {
            dai_schedule(s);
        }
        return val;
    case DAI_INTSTS:
    case DAI_INTSTS2:
        return old & ~val;
    }
    return val;
}

static void dai_period(NvtMediaState *s)
{
    uint32_t ctrl = *reg(s, DAI_CTRL);

    if (ctrl & DAI_CTRL_RX_DMA) {
        uint32_t n = dai_buf_bytes(s, DAI_RX1_SIZE);
        g_autofree uint8_t *silence = g_malloc0(n);

        dma_memory_write(&address_space_memory, *reg(s, DAI_RX1_ADDR),
                         silence, n, MEMTXATTRS_UNSPECIFIED);
        *reg(s, DAI_INTSTS) |= DAI_INT_RX1_BFI | DAI_INT_RX_LOAD;
    }
    if (ctrl & DAI_CTRL_TX1_DMA) {
        *reg(s, DAI_INTSTS) |= DAI_INT_TX1_BFI | DAI_INT_TX1_LOAD;
    }
    dai_update_irq(s);
    dai_schedule(s);
}

static const NvtMediaEngine nvt_media_engines[] = {
    { "generic" },
    { "sie", sie_write, sie_read, NULL, sie_frame, sie_update_irq },
    { "sie2", sie_write, sie_read, NULL, sie_frame, sie_update_irq },
    { "sie3", sie_write, sie_read, NULL, sie_frame, sie_update_irq },
    { "ife", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq, &ipp_ife },
    { "dce", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq, &ipp_dce },
    { "ipe", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq, &ipp_ipe },
    { "ime", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq, &ipp_ime },
    { "h26x", h26x_write, NULL, NULL, h26x_done, h26x_update_irq },
    { "eac", eac_write },
    { "dai", dai_write, NULL, NULL, dai_period, dai_update_irq },
    { "jpg", jpg_write, jpg_read, NULL, jpg_done, jpg_update_irq },
    { "ise", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq, &ipp_ise },
    { "graphic", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq,
      &ipp_grph },
    { "graphic2", ipp_write, NULL, NULL, ipp_job_done, ipp_update_irq,
      &ipp_grph },
};

static uint64_t nvt_media_read(void *opaque, hwaddr off, unsigned size)
{
    NvtMediaState *s = opaque;
    uint64_t val = *reg(s, off);

    if (s->engine->read) {
        s->engine->read(s, off, &val);
    }
    if (s->trace) {
        qemu_log("nvt-media %s: read  0x%03" HWADDR_PRIx " = 0x%08" PRIx64
                 "\n", s->engine_name, off, val);
    }
    return val;
}

static void nvt_media_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    NvtMediaState *s = opaque;

    if (s->trace) {
        qemu_log("nvt-media %s: write 0x%03" HWADDR_PRIx " = 0x%08" PRIx64
                 "\n", s->engine_name, off, val);
    }
    if (s->engine->write) {
        val = s->engine->write(s, off, val);
    }
    *reg(s, off) = val;
    nvt_media_update_irq(s);
}

static void nvt_media_timer(void *opaque)
{
    NvtMediaState *s = opaque;

    if (s->engine->timer) {
        s->engine->timer(s);
    }
}

static const MemoryRegionOps nvt_media_ops = {
    .read = nvt_media_read,
    .write = nvt_media_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_media_reset(DeviceState *dev)
{
    NvtMediaState *s = NVT_MEDIA(dev);
    int i;

    timer_del(s->timer);
    memset(s->regs, 0, s->size);
    if (s->engine->reset) {
        s->engine->reset(s);
    }
    for (i = 0; i < NVT_MEDIA_MAX_IRQS; i++) {
        qemu_irq_lower(s->irq[i]);
    }
}

static void nvt_media_realize(DeviceState *dev, Error **errp)
{
    NvtMediaState *s = NVT_MEDIA(dev);
    const char *name = s->engine_name ? s->engine_name : "generic";
    int i;

    if (!s->size || (s->size & 3)) {
        error_setg(errp, "nvt-media: size must be a nonzero multiple of 4");
        return;
    }
    s->engine = &nvt_media_engines[0];
    for (i = 0; i < ARRAY_SIZE(nvt_media_engines); i++) {
        if (!strcmp(nvt_media_engines[i].name, name)) {
            s->engine = &nvt_media_engines[i];
        }
    }
    s->regs = g_malloc0(s->size);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_media_timer, s);
    memory_region_init_io(&s->iomem, OBJECT(dev), &nvt_media_ops, s,
                          TYPE_NVT_MEDIA, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    for (i = 0; i < NVT_MEDIA_MAX_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }
}

static const Property nvt_media_properties[] = {
    DEFINE_PROP_STRING("engine", NvtMediaState, engine_name),
    DEFINE_PROP_UINT32("size", NvtMediaState, size, 0x1000),
    DEFINE_PROP_BOOL("trace", NvtMediaState, trace, false),
    DEFINE_PROP_LINK("ipp-input", NvtMediaState, ipp_src, TYPE_NVT_MEDIA,
                     NvtMediaState *),
};

static const VMStateDescription vmstate_nvt_media = {
    .name = TYPE_NVT_MEDIA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VBUFFER_UINT32(regs, NvtMediaState, 1, NULL, size),
        VMSTATE_TIMER_PTR(timer, NvtMediaState),
        VMSTATE_UINT32(phase, NvtMediaState),
        VMSTATE_UINT32(ll_addr, NvtMediaState),
        VMSTATE_UINT32(frames, NvtMediaState),
        VMSTATE_UINT8_ARRAY(jpg_q, NvtMediaState, 128),
        VMSTATE_UINT32(jpg_lut, NvtMediaState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_media_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_media_realize;
    device_class_set_legacy_reset(dc, nvt_media_reset);
    device_class_set_props(dc, nvt_media_properties);
    dc->vmsd = &vmstate_nvt_media;
}

static const TypeInfo nvt_media_info = {
    .name = TYPE_NVT_MEDIA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtMediaState),
    .class_init = nvt_media_class_init,
};

static void nvt_media_register_types(void)
{
    type_register_static(&nvt_media_info);
}

type_init(nvt_media_register_types)
