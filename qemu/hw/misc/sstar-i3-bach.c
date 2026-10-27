/*
 * MStar infinity3 audio ("BACH"): the playback reader and capture writer
 * DMAs of the ALSA driver "mstar,infinity3-audio".
 *
 * Read off the vendor kernel's drivers/mstar/sound (infinity.c,
 * infinity_pcm.c, infinity_reg.h; the I3 SDK MSC3XX_SDK_V_1711291828).
 * Two RIU banks, 16-bit registers on a 4-byte stride; the driver names
 * them by byte offset (offset o is register o / 2):
 *
 * BANK1, bank 0x1502:
 *   0x02  sample rates: [7:4] SRC1_SEL, the reader's (0-8 = 8, 11.025,
 *         12, 16, 22.05, 24, 32, 44.1, 48 kHz); [9:8] WRITER_SEL, the
 *         writer's (0-3 = 8, 16, 32, 48 kHz) (InfinityDmaSetRate)
 *   0x80  bit 15 WR_UNDERRUN, 14 WR_OVERRUN, 13 RD_UNDERRUN, 12
 *         RD_OVERRUN, 11 WR_FULL, 10 RD_EMPTY interrupt enables; 9
 *         WR_FULL and 8 RD_EMPTY flag clears (a 1 then a 0); 1 the
 *         reader's prefetch enable; 0 the DMA's software reset
 *   0x82  reader: bit 15 RD_ENABLE, 14 INIT, 13 TRIG, 12 level latch,
 *         [11:0] the buffer's address, low (8-byte units)
 *   0x84  [14:0] its address, high: MIU byte address ((hi << 12) | lo) * 8,
 *         DRAM from 0x20000000 (MIU0_OFFSET)
 *   0x86  its size; 0x88 RD_SIZE, what the CPU has added; 0x8A overrun
 *         and 0x8C underrun thresholds; 0x8E the level count
 *   0x90  flags: bit 7 RD_LOCALBUF_EMPTY, 6 WR_LOCALBUF_FULL, 5 WR_FULL,
 *         4 RD_EMPTY, 3 RD_OVERRUN, 2 RD_UNDERRUN, 1 WR_OVERRUN, 0
 *         WR_UNDERRUN
 *   0x92-0x9E the writer, laid out as 0x82-0x8E (0x98 WR_SIZE, what the
 *         CPU has taken)
 *   0xEE  bit 15 RD_MONO, 14 WR_MONO (else two channels), 16-bit samples
 * BANK2, bank 0x1503: 0x0E bit 1 REG_DMA_INT_EN, the DMA's interrupt
 *   (InfinityDmaReset); 0x2C the codec rate, storage.
 *
 * Sizes, levels and thresholds are 8-byte units.  The CPU tells the DMA
 * what it wrote or read by putting the amount in RD_SIZE / WR_SIZE and
 * inverting TRIG (InfinityDmaTrigLevelCnt): every change of TRIG adds
 * RD_SIZE to the reader's level and takes WR_SIZE from the writer's.
 * INIT resets a channel's pointer and level (InfinityDmaReInit).
 *
 * The interrupt (INT_IRQ_BACH, GIC SPI 74) is level: high while a flag
 * and its enable are both set and BANK2 0x0E bit 1 is.  The ISR
 * (infinity_pcm_dma_irq) acts by masking: a playback underrun (the level
 * below its threshold, buffer - period) is the period tick, which it
 * masks until the copy path has refilled; RD_EMPTY, sticky until its
 * clear, stops the stream with an XRUN.  For capture WR_OVERRUN (the level
 * above one period) is the tick, after which it waits for WR_UNDERRUN
 * (below 256 bytes) to re-arm it; WR_FULL, sticky, is the XRUN.  So the
 * underrun and overrun flags are live comparisons, the empty and full
 * ones latched.
 *
 * Timing is the virtual clock at the programmed rate.  The reader plays
 * continuously, 8 bytes at a time, and goes to the machine's audiodev if
 * there is one.  The writer's microphone hears a 1 kHz tone and arrives a
 * threshold (one period) at a time: the driver re-arms its period
 * interrupt only once the level has dropped under 256 bytes, which on the
 * chip a reader that answers the interrupt in time always sees, and which
 * samples trickling in between the interrupt and the read would defeat.
 * The writer's level count includes the 64-byte local buffer, as the
 * driver expects (InfinityDmaGetLevelCnt takes 8 units off).
 *
 * BANK3 (0x1034, the analog codec) is RIU storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/audio.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_I3_BACH "sstar-i3-bach"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI3BachState, SSTAR_I3_BACH)

#define BANK_REGS           0x80
#define R(off)              ((off) >> 1)

/* BANK1 */
#define SR0_SEL             R(0x02)
#define DMA_CTRL            R(0x80)
#define WR_UNDERRUN_EN      (1u << 15)
#define WR_OVERRUN_EN       (1u << 14)
#define RD_UNDERRUN_EN      (1u << 13)
#define RD_OVERRUN_EN       (1u << 12)
#define WR_FULL_EN          (1u << 11)
#define RD_EMPTY_EN         (1u << 10)
#define WR_FULL_CLR         (1u << 9)
#define RD_EMPTY_CLR        (1u << 8)
#define SW_RST              (1u << 0)
#define DMA_FLAGS           R(0x90)
#define RD_LOCALBUF_EMPTY   (1u << 7)
#define WR_LOCALBUF_FULL    (1u << 6)
#define WR_FULL             (1u << 5)
#define RD_EMPTY            (1u << 4)
#define RD_OVERRUN          (1u << 3)
#define RD_UNDERRUN         (1u << 2)
#define WR_OVERRUN          (1u << 1)
#define WR_UNDERRUN         (1u << 0)
#define TEST_CTRL7          R(0xEE)
#define RD_MONO             (1u << 15)
#define WR_MONO             (1u << 14)

/* a channel's registers, from 0x82 (reader) or 0x92 (writer) */
#define CH_CTRL             0
#define CH_ENABLE           (1u << 15)
#define CH_INIT             (1u << 14)
#define CH_TRIG             (1u << 13)
#define CH_ADDR_HI          1
#define CH_BUF_SIZE         2
#define CH_SIZE             3
#define CH_OVERRUN_TH       4
#define CH_UNDERRUN_TH      5
#define CH_LEVEL            6

/* BANK2 */
#define INT_EN              R(0x0E)
#define DMA_INT_EN          (1u << 1)

#define MIU0_OFFSET         0x20000000
#define LOCAL_UNITS         8           /* the writer's 64-byte local buffer */
#define TICK_NS             (10 * SCALE_MS)
#define OUT_RING            (64 * KiB)

enum { RD, WR };

typedef struct BachChan {
    uint32_t level;             /* bytes in DRAM between DMA and CPU */
    uint32_t ptr;               /* the DMA's offset in the buffer */
    int64_t t;                  /* virtual ns accounted for */
    bool local;                 /* writer: the local buffer holds data */
} BachChan;

struct SstarI3BachState {
    SysBusDevice parent_obj;

    MemoryRegion bank1;
    MemoryRegion bank2;
    qemu_irq irq;
    QEMUTimer *timer;
    uint16_t regs1[BANK_REGS];
    uint16_t regs2[BANK_REGS];
    BachChan ch[2];
    uint16_t sticky;            /* RD_EMPTY and WR_FULL */
    uint64_t sample;            /* tone frames written */

    AudioBackend *audio_be;
    SWVoiceOut *vout;
    uint32_t voice_rate;
    uint8_t voice_chans;
    uint8_t out[OUT_RING];
    uint32_t out_head, out_len;

    uint32_t tone_hz;
    bool trace;
};

static const uint32_t rd_rates[9] = {
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
};
static const uint32_t wr_rates[4] = { 8000, 16000, 32000, 48000 };

static uint16_t *ch_reg(SstarI3BachState *s, int c, int r)
{
    return &s->regs1[R(c == RD ? 0x82 : 0x92) + r];
}

static uint32_t ch_rate(SstarI3BachState *s, int c)
{
    if (c == RD) {
        return rd_rates[MIN((s->regs1[SR0_SEL] >> 4) & 0xF, 8)];
    }
    return wr_rates[(s->regs1[SR0_SEL] >> 8) & 3];
}

static uint32_t ch_chans(SstarI3BachState *s, int c)
{
    return s->regs1[TEST_CTRL7] & (c == RD ? RD_MONO : WR_MONO) ? 1 : 2;
}

/* bytes a second: 16-bit samples */
static uint32_t ch_byte_rate(SstarI3BachState *s, int c)
{
    return ch_rate(s, c) * ch_chans(s, c) * 2;
}

static uint32_t ch_size(SstarI3BachState *s, int c)
{
    return (uint32_t)*ch_reg(s, c, CH_BUF_SIZE) * 8;
}

static hwaddr ch_base(SstarI3BachState *s, int c)
{
    return ((((hwaddr)*ch_reg(s, c, CH_ADDR_HI) & 0x7FFF) << 12) |
            (*ch_reg(s, c, CH_CTRL) & 0xFFF)) * 8 + MIU0_OFFSET;
}

static bool ch_running(SstarI3BachState *s, int c)
{
    return (*ch_reg(s, c, CH_CTRL) & CH_ENABLE) && ch_size(s, c);
}

/* the level count as the registers hold it, in 8-byte units */
static uint32_t ch_level_units(SstarI3BachState *s, int c)
{
    uint32_t u = s->ch[c].level / 8;

    if (c == WR && s->ch[c].local) {
        u += LOCAL_UNITS;
    }
    return MIN(u, 0xFFFF);
}

/* the writer's chunk: one overrun threshold (a period), else 10 ms */
static uint32_t wr_chunk(SstarI3BachState *s)
{
    uint32_t th = (uint32_t)*ch_reg(s, WR, CH_OVERRUN_TH) * 8;

    if (th && th < ch_size(s, WR)) {
        return th;
    }
    return MAX(ch_byte_rate(s, WR) / 100 & ~7u, 8);
}

static void ch_init(SstarI3BachState *s, int c)
{
    s->ch[c].level = 0;
    s->ch[c].ptr = 0;
    s->ch[c].local = false;
    s->ch[c].t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void bach_out_cb(void *opaque, int avail)
{
    SstarI3BachState *s = opaque;

    while (avail > 0 && s->out_len) {
        uint32_t n = MIN(MIN((uint32_t)avail, s->out_len),
                         OUT_RING - s->out_head);
        size_t done = AUD_write(s->vout, s->out + s->out_head, n);

        if (!done) {
            break;
        }
        s->out_head = (s->out_head + done) % OUT_RING;
        s->out_len -= done;
        avail -= done;
    }
}

/* the playback voice, at the reader's rate, while the reader runs */
static void bach_voice(SstarI3BachState *s)
{
    bool on = ch_running(s, RD);
    uint32_t rate = ch_rate(s, RD);
    uint8_t chans = ch_chans(s, RD);

    if (!s->audio_be) {
        return;
    }
    if (s->vout && (s->voice_rate != rate || s->voice_chans != chans)) {
        AUD_close_out(s->audio_be, s->vout);
        s->vout = NULL;
    }
    if (on && !s->vout) {
        struct audsettings as = {
            .freq = rate, .nchannels = chans, .fmt = AUDIO_FORMAT_S16,
            .endianness = 0,
        };

        s->vout = AUD_open_out(s->audio_be, NULL, "sstar-i3-bach.out", s,
                               bach_out_cb, &as);
        s->voice_rate = rate;
        s->voice_chans = chans;
        s->out_head = s->out_len = 0;
    }
    if (s->vout) {
        AUD_set_active_out(s->vout, on);
    }
}

/* the reader plays n bytes from its pointer */
static void rd_play(SstarI3BachState *s, uint32_t n)
{
    BachChan *ch = &s->ch[RD];
    uint32_t size = ch_size(s, RD);
    uint8_t buf[1024];

    while (n) {
        uint32_t k = MIN(MIN(n, sizeof(buf)), size - ch->ptr);

        if (s->vout) {
            uint32_t i;

            dma_memory_read(&address_space_memory, ch_base(s, RD) + ch->ptr,
                            buf, k, MEMTXATTRS_UNSPECIFIED);
            for (i = 0; i < k && s->out_len < OUT_RING; i++) {
                s->out[(s->out_head + s->out_len++) % OUT_RING] = buf[i];
            }
        }
        ch->ptr = (ch->ptr + k) % size;
        n -= k;
    }
}

/* the microphone: n bytes of tone at the writer's pointer */
static void wr_record(SstarI3BachState *s, uint32_t n)
{
    BachChan *ch = &s->ch[WR];
    uint32_t size = ch_size(s, WR);
    uint32_t chans = ch_chans(s, WR);
    uint32_t rate = ch_rate(s, WR);
    uint8_t buf[1024];

    while (n) {
        uint32_t k = MIN(MIN(n, sizeof(buf)), size - ch->ptr) & ~1u;
        uint32_t i;

        if (!k) {
            break;
        }
        for (i = 0; i < k; i += 2) {
            uint64_t frame = s->sample;
            int16_t v = (int16_t)(8000 * sin(2 * M_PI * s->tone_hz *
                                             (double)(frame % rate) / rate));

            if ((i / 2 + 1) % chans == 0) {
                s->sample++;
            }
            buf[i] = v & 0xFF;
            buf[i + 1] = (v >> 8) & 0xFF;
        }
        dma_memory_write(&address_space_memory, ch_base(s, WR) + ch->ptr,
                         buf, k, MEMTXATTRS_UNSPECIFIED);
        ch->ptr = (ch->ptr + k) % size;
        n -= k;
    }
}

/* bring both channels up to now */
static void bach_sync(SstarI3BachState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    BachChan *rd = &s->ch[RD], *wr = &s->ch[WR];

    if (ch_running(s, RD)) {
        uint32_t br = ch_byte_rate(s, RD);
        uint64_t n = (uint64_t)MAX(now - rd->t, 0) * br /
                     NANOSECONDS_PER_SECOND & ~7ull;

        if (!rd->level) {
            rd->t = now;
        } else if (n) {
            n = MIN(n, rd->level);
            rd_play(s, n);
            rd->level -= n;
            rd->t += n * NANOSECONDS_PER_SECOND / br;
            if (!rd->level) {
                rd->t = now;
            }
        }
        if (!rd->level && !(s->regs1[DMA_CTRL] & RD_EMPTY_CLR)) {
            s->sticky |= RD_EMPTY;
        }
    }
    if (ch_running(s, WR)) {
        uint32_t br = ch_byte_rate(s, WR);
        uint32_t chunk = wr_chunk(s);
        uint64_t n = (uint64_t)MAX(now - wr->t, 0) * br /
                     NANOSECONDS_PER_SECOND;

        n -= n % chunk;
        if (n) {
            uint32_t size = ch_size(s, WR);
            uint32_t fit = MIN(n, size - wr->level);

            wr_record(s, fit);
            wr->level += fit;
            wr->local = true;
            wr->t += n * NANOSECONDS_PER_SECOND / br;
            if (wr->level >= size && !(s->regs1[DMA_CTRL] & WR_FULL_CLR)) {
                s->sticky |= WR_FULL;
            }
        }
    }
}

static uint16_t bach_flags(SstarI3BachState *s)
{
    uint16_t f = s->sticky;
    uint32_t rl = ch_level_units(s, RD), wl = ch_level_units(s, WR);
    uint16_t th;

    th = *ch_reg(s, RD, CH_UNDERRUN_TH);
    if (th && rl < th) {
        f |= RD_UNDERRUN;
    }
    th = *ch_reg(s, RD, CH_OVERRUN_TH);
    if (th && rl > th) {
        f |= RD_OVERRUN;
    }
    if (!s->ch[RD].level) {
        f |= RD_LOCALBUF_EMPTY;
    }
    th = *ch_reg(s, WR, CH_UNDERRUN_TH);
    if (th && wl < th) {
        f |= WR_UNDERRUN;
    }
    th = *ch_reg(s, WR, CH_OVERRUN_TH);
    if (th && wl > th) {
        f |= WR_OVERRUN;
    }
    if (s->ch[WR].local) {
        f |= WR_LOCALBUF_FULL;
    }
    return f;
}

/* the line, and the timer for the next thing that will change it */
static void bach_update(SstarI3BachState *s)
{
    static const struct {
        uint16_t flag, en;
    } map[] = {
        { WR_UNDERRUN, WR_UNDERRUN_EN }, { WR_OVERRUN, WR_OVERRUN_EN },
        { RD_UNDERRUN, RD_UNDERRUN_EN }, { RD_OVERRUN, RD_OVERRUN_EN },
        { WR_FULL, WR_FULL_EN }, { RD_EMPTY, RD_EMPTY_EN },
    };
    uint16_t f = bach_flags(s);
    uint16_t en = s->regs1[DMA_CTRL];
    int64_t next = INT64_MAX;
    bool on = false;
    int i;

    s->regs1[DMA_FLAGS] = f;
    for (i = 0; i < ARRAY_SIZE(map); i++) {
        if ((f & map[i].flag) && (en & map[i].en)) {
            on = true;
        }
    }
    qemu_set_irq(s->irq, on && (s->regs2[INT_EN] & DMA_INT_EN));

    if (ch_running(s, RD) && s->ch[RD].level) {
        /* every 10 ms for the voice, and when the level crosses under */
        uint32_t br = ch_byte_rate(s, RD);
        uint32_t th = (uint32_t)*ch_reg(s, RD, CH_UNDERRUN_TH) * 8;
        uint32_t lv = s->ch[RD].level;
        uint32_t n = th && lv >= th ? lv - th + 8 : lv;

        next = s->ch[RD].t + (int64_t)((uint64_t)n * NANOSECONDS_PER_SECOND /
                                       br) + 1;
        next = MIN(next, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
    }
    if (ch_running(s, WR)) {
        uint32_t br = ch_byte_rate(s, WR);

        next = MIN(next, s->ch[WR].t +
                   (int64_t)((uint64_t)wr_chunk(s) * NANOSECONDS_PER_SECOND /
                             br) + 1);
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
}

static void bach_tick(void *opaque)
{
    SstarI3BachState *s = opaque;

    bach_sync(s);
    bach_update(s);
}

static uint64_t bach_read(SstarI3BachState *s, uint16_t *regs, hwaddr off,
                          unsigned size)
{
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    bach_sync(s);
    bach_update(s);
    if (regs == s->regs1) {
        if ((off >> 2) == R(0x8E)) {
            regs[R(0x8E)] = ch_level_units(s, RD);
        } else if ((off >> 2) == R(0x9E)) {
            regs[R(0x9E)] = ch_level_units(s, WR);
        }
    }
    v = regs[(off >> 2) & (BANK_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static uint16_t bach_merge(uint16_t old, hwaddr off, uint64_t val,
                           unsigned size)
{
    if (size == 1) {
        return off & 1 ? (old & 0x00FF) | ((val & 0xFF) << 8)
                       : (old & 0xFF00) | (val & 0xFF);
    }
    return val;
}

static uint64_t bach_bank1_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3BachState *s = opaque;

    return bach_read(s, s->regs1, off, size);
}

static uint64_t bach_bank2_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3BachState *s = opaque;

    return bach_read(s, s->regs2, off, size);
}

static void bach_bank1_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarI3BachState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);
    uint16_t old, v;
    int c;

    if ((off & 3) >= 2) {
        return;
    }
    bach_sync(s);
    if (reg == DMA_FLAGS || reg == R(0x8E) || reg == R(0x9E)) {
        return;
    }
    old = s->regs1[reg];
    v = bach_merge(old, off, val, size);
    s->regs1[reg] = v;
    if (s->trace) {
        qemu_log("sstar-i3-bach: W 0x%02x = 0x%04x\n", reg << 1, v);
    }
    switch (reg) {
    case DMA_CTRL:
        if (v & RD_EMPTY_CLR) {
            s->sticky &= ~RD_EMPTY;
        }
        if (v & WR_FULL_CLR) {
            s->sticky &= ~WR_FULL;
        }
        if (v & SW_RST) {
            ch_init(s, RD);
            ch_init(s, WR);
            s->sticky = 0;
        }
        break;
    case R(0x82):
    case R(0x92):
        c = reg == R(0x82) ? RD : WR;
        if ((v & CH_INIT) && !(old & CH_INIT)) {
            ch_init(s, c);
        }
        if ((v & CH_ENABLE) && !(old & CH_ENABLE)) {
            /* the stream starts now; nothing has played or arrived yet */
            s->ch[c].t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        if ((v ^ old) & CH_TRIG) {
            uint32_t n = (uint32_t)*ch_reg(s, c, CH_SIZE) * 8;

            if (c == RD) {
                if (!s->ch[RD].level) {
                    s->ch[RD].t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                }
                s->ch[RD].level = MIN(s->ch[RD].level + n, 0xFFFF * 8);
            } else {
                s->ch[WR].level -= MIN(n, s->ch[WR].level);
            }
        }
        if (c == RD) {
            bach_voice(s);
        }
        break;
    case SR0_SEL:
    case TEST_CTRL7:
        bach_voice(s);
        break;
    }
    bach_update(s);
}

static void bach_bank2_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarI3BachState *s = opaque;
    int reg = (off >> 2) & (BANK_REGS - 1);

    if ((off & 3) >= 2) {
        return;
    }
    bach_sync(s);
    s->regs2[reg] = bach_merge(s->regs2[reg], off, val, size);
    bach_update(s);
}

static const MemoryRegionOps bach_bank1_ops = {
    .read = bach_bank1_read,
    .write = bach_bank1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps bach_bank2_ops = {
    .read = bach_bank2_read,
    .write = bach_bank2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_i3_bach_reset(DeviceState *dev)
{
    SstarI3BachState *s = SSTAR_I3_BACH(dev);

    memset(s->regs1, 0, sizeof(s->regs1));
    memset(s->regs2, 0, sizeof(s->regs2));
    ch_init(s, RD);
    ch_init(s, WR);
    s->sticky = 0;
    s->sample = 0;
    s->out_head = s->out_len = 0;
    timer_del(s->timer);
    bach_voice(s);
    qemu_set_irq(s->irq, 0);
}

static void sstar_i3_bach_realize(DeviceState *dev, Error **errp)
{
    SstarI3BachState *s = SSTAR_I3_BACH(dev);

    /* the host's audio only when the machine was given an audiodev */
    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bach_tick, s);
}

static void sstar_i3_bach_unrealize(DeviceState *dev)
{
    SstarI3BachState *s = SSTAR_I3_BACH(dev);

    if (s->vout) {
        AUD_close_out(s->audio_be, s->vout);
    }
    timer_free(s->timer);
}

static void sstar_i3_bach_init(Object *obj)
{
    SstarI3BachState *s = SSTAR_I3_BACH(obj);

    memory_region_init_io(&s->bank1, obj, &bach_bank1_ops, s,
                          "sstar-i3-bach.bank1", BANK_REGS * 4);
    memory_region_init_io(&s->bank2, obj, &bach_bank2_ops, s,
                          "sstar-i3-bach.bank2", BANK_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank1);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->bank2);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static int sstar_i3_bach_post_load(void *opaque, int version_id)
{
    SstarI3BachState *s = opaque;

    s->out_head = s->out_len = 0;
    bach_voice(s);
    return 0;
}

static const Property sstar_i3_bach_props[] = {
    DEFINE_AUDIO_PROPERTIES(SstarI3BachState, audio_be),
    DEFINE_PROP_UINT32("tone-hz", SstarI3BachState, tone_hz, 1000),
    DEFINE_PROP_BOOL("trace", SstarI3BachState, trace, false),
};

static const VMStateDescription vmstate_bach_chan = {
    .name = "sstar-i3-bach/chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(level, BachChan),
        VMSTATE_UINT32(ptr, BachChan),
        VMSTATE_INT64(t, BachChan),
        VMSTATE_BOOL(local, BachChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_sstar_i3_bach = {
    .name = TYPE_SSTAR_I3_BACH,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sstar_i3_bach_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs1, SstarI3BachState, BANK_REGS),
        VMSTATE_UINT16_ARRAY(regs2, SstarI3BachState, BANK_REGS),
        VMSTATE_STRUCT_ARRAY(ch, SstarI3BachState, 2, 1, vmstate_bach_chan,
                             BachChan),
        VMSTATE_UINT16(sticky, SstarI3BachState),
        VMSTATE_UINT64(sample, SstarI3BachState),
        VMSTATE_TIMER_PTR(timer, SstarI3BachState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i3_bach_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_i3_bach_realize;
    dc->unrealize = sstar_i3_bach_unrealize;
    device_class_set_legacy_reset(dc, sstar_i3_bach_reset);
    device_class_set_props(dc, sstar_i3_bach_props);
    dc->vmsd = &vmstate_sstar_i3_bach;
}

static const TypeInfo sstar_i3_bach_info = {
    .name           = TYPE_SSTAR_I3_BACH,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI3BachState),
    .instance_init  = sstar_i3_bach_init,
    .class_init     = sstar_i3_bach_class_init,
};

static void sstar_i3_bach_register_types(void)
{
    type_register_static(&sstar_i3_bach_info);
}

type_init(sstar_i3_bach_register_types)
