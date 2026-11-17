/*
 * SigmaStar Ifado audio ("BACH"), its capture DMA.
 *
 * sstar_mi.ko reaches BACH through HalBachReadReg/HalBachWriteReg with a
 * bank index and a byte offset: index 0 is RIU bank 0x1502, 1 is 0x1503,
 * 2 is 0x1504 and 3 is 0x1034, 16-bit registers at even offsets (offset
 * o is register o / 2 of the bank).  The DMAs (HalAudDma*) are in bank
 * 0x1502; the AI writer, DMA 0 to the driver (DMA 7, at 0x80-0x8E, is
 * the AO reader), is (byte offsets):
 *
 *   0x80  bit 11 enables the threshold interrupt, bit 14 the overrun
 *         one (HalAudDmaIntEnable); writing bit 9 clears the overrun
 *         (HalAudDmaClearInt)
 *   0x92  [11:0] the buffer's address [14:3]; bit 12 latches the level
 *         count into 0x9E; a change of bit 13 takes the count in 0x98 as
 *         read (HalAudDmaTrigLevelCnt); bit 15 runs the DMA
 *   0x94  [14:0] the buffer's address [29:15]
 *   0x96  the buffer's size in 8-byte units
 *   0x98  bytes read, in 8-byte units
 *   0x9A  the threshold, 8-byte units (HalAudDmaAiSetThreshold)
 *   0x9E  the level count, 8-byte units (HalAudDmaGetLevelCnt)
 *   0x90  flags (HalAudDmaGetFlags): bit 1 overrun, bit 5 the level at
 *         or above the threshold, bit 6 full
 *
 * infinity6c's registers count 16-byte units, not 8 (its HalAudDmaSetPhyAddr
 * writes [15:4] and [30:16]): "unit-shift" 4, or the ring lands at twice its
 * address, in the kernel.
 *
 * The AO reader, DMA 1, is the infinity3 BACH's reader (sstar-i3-bach.c),
 * as mhal.ko's HalAudDma* functions program it for index 1 on an
 * infinity6b0 (ssc337), and as a camera's registers read while it plays:
 *
 *   0x02  [7:4] its rate, 0-8 = 8, 11.025, 12, 16, 22.05, 24, 32, 44.1,
 *         48 kHz (HalAudDmaSetRate)
 *   0x80  bit 13 enables RD_UNDERRUN's interrupt, bit 10 RD_EMPTY's
 *         (HalAudDmaIntEnable); bit 8 clears RD_EMPTY (HalAudDmaClearInt);
 *         bit 1 is set before the reader runs (HalAudDmaEnable)
 *   0x82  bit 15 runs it, a 1 in bit 14 resets its pointer and level
 *         (HalAudDmaReset), a change of bit 13 adds 0x88 to the level
 *         (HalAudDmaTrigLevelCnt), bit 12 latches the level into 0x8E;
 *         [11:0] the buffer's address, low
 *   0x84  the address, high; 0x86 the buffer's size; 0x88 what the CPU
 *         has added; 0x8A the overrun, 0x8C the underrun threshold
 *   0x90  bit 2 RD_UNDERRUN (the level under 0x8C), bit 3 RD_OVERRUN
 *         (at or over 0x8A), both live; bit 4 RD_EMPTY, latched when the
 *         level runs out (HalAudDmaGetFlags; the bit 7 it also reads
 *         never sets on the camera)
 *   0xEE  bit 15 the reader in mono, else two channels interleaved
 *
 * Units as the writer's.  An ssc337 playing 8 kHz mono reads 0x1E in
 * 0x90 after a clip and 0x0E while one plays, with a 64 KiB buffer (0x86
 * 0x2000), 0x8A 0 and 0x8C 0x1FD8: RD_UNDERRUN for any level a frame
 * under full, RD_OVERRUN at any level against a zero threshold (so "at or
 * over", unlike infinity3's model), RD_EMPTY latched once a clip has
 * drained and cleared by the driver when the next is queued.  The reader plays at the
 * rate in 0x02 on the virtual clock, from the moment a level arrives,
 * and what it reads goes to the machine's audiodev if there is one
 * ("playback"; off where the AO driver has not been run against it).
 *
 * The model's microphone hears a 1 kHz tone: while the DMA runs the
 * ring fills at "rate" bytes a second of 16-bit samples ("sample-rate"
 * Hz), the level count growing until the buffer is full; samples
 * that arrive then are dropped, which latches the overrun until 0x80
 * bit 9 clears it.  _AudDmaIrq acts only on a flag whose enable is set (bit 5
 * with bit 11, bit 1 with bit 14) and masks it, so the interrupt is the
 * OR of exactly those pairs; a line raised on any other condition is
 * never masked and storms the CPU.  Everything else in the banks is
 * storage.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/audio.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/irq.h"
#include "hw/misc/sstar-riu.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SSTAR_BACH "sstar-bach"
OBJECT_DECLARE_SIMPLE_TYPE(SstarBachState, SSTAR_BACH)

#define BACH_REGS           0x80        /* one RIU bank */
#define R(off)              ((off) >> 1)
#define AI_IRQEN            R(0x80)
#define EN_THRESH           (1u << 11)
#define EN_OVERRUN          (1u << 14)
#define CLR_OVERRUN         (1u << 9)
#define AI_ADDR_LO          R(0x92)
#define AI_LATCH            (1u << 12)
#define AI_TRIG             (1u << 13)
#define AI_RUN              (1u << 15)
#define AI_ADDR_HI          R(0x94)
#define AI_SIZE             R(0x96)
#define AI_READ             R(0x98)
#define AI_THRESH           R(0x9A)
#define AI_LEVEL            R(0x9E)
#define AI_FLAGS            R(0x90)
#define FLAG_OVERRUN        (1u << 1)
#define FLAG_HIGH           (1u << 5)
#define FLAG_FULL           (1u << 6)
#define TICK_NS             (10 * SCALE_MS)

/* the AO reader, DMA 1 */
#define SR0_SEL             R(0x02)
#define RD_UNDERRUN_EN      (1u << 13)
#define RD_EMPTY_EN         (1u << 10)
#define RD_EMPTY_CLR        (1u << 8)
#define RD_CTRL             R(0x82)
#define RD_ENABLE           (1u << 15)
#define RD_INIT             (1u << 14)
#define RD_TRIG             (1u << 13)
#define RD_LATCH            (1u << 12)
#define RD_ADDR_HI          R(0x84)
#define RD_BUF_SIZE         R(0x86)
#define RD_SIZE             R(0x88)
#define RD_OVERRUN_TH       R(0x8A)
#define RD_UNDERRUN_TH      R(0x8C)
#define RD_LEVEL            R(0x8E)
#define RD_EMPTY            (1u << 4)
#define RD_OVERRUN          (1u << 3)
#define RD_UNDERRUN         (1u << 2)
#define RD_FLAGS            (RD_EMPTY | RD_OVERRUN | RD_UNDERRUN)
#define TEST_CTRL7          R(0xEE)
#define RD_MONO             (1u << 15)
#define OUT_RING            (64 * KiB)
#define SS_DRAM_BASE        0x20000000
#define SS_PAGE             4096

struct SstarBachState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *tick;
    uint16_t regs[BACH_REGS];
    uint32_t level;             /* bytes in the ring not yet read */
    uint32_t wptr;              /* where the next byte goes */
    uint64_t sample;            /* samples written so far */
    int64_t last;

    /* the AO reader */
    uint32_t rd_level;          /* bytes queued and not yet played */
    uint32_t rd_ptr;            /* its offset in the buffer */
    int64_t rd_t;               /* virtual ns played up to */
    bool rd_empty;              /* RD_EMPTY, latched */

    AudioBackend *audio_be;
    SWVoiceOut *vout;
    uint32_t voice_rate;
    uint8_t voice_chans;
    uint8_t out[OUT_RING];
    uint32_t out_head, out_len;

    uint32_t rate;
    uint32_t sample_rate;
    uint32_t unit_shift;        /* log2 of the DMA's address and size unit */
    uint32_t channels;          /* interleaved in the ring: mi_ai takes one */
    bool playback;
    bool trace;
};

static const uint32_t rd_rates[9] = {
    8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
};

static uint32_t rd_rate(SstarBachState *s)
{
    return rd_rates[MIN((s->regs[SR0_SEL] >> 4) & 0xF, 8)];
}

static uint8_t rd_chans(SstarBachState *s)
{
    return s->regs[TEST_CTRL7] & RD_MONO ? 1 : 2;
}

/* bytes a second: 16-bit samples */
static uint32_t rd_byte_rate(SstarBachState *s)
{
    return rd_rate(s) * rd_chans(s) * 2;
}

static uint32_t rd_size(SstarBachState *s)
{
    return (uint32_t)s->regs[RD_BUF_SIZE] << s->unit_shift;
}

static uint64_t rd_base(SstarBachState *s)
{
    return ((uint64_t)(s->regs[RD_ADDR_HI] & 0x7FFF) << (12 + s->unit_shift)) |
           ((uint64_t)(s->regs[RD_CTRL] & 0xFFF) << s->unit_shift);
}

static bool rd_running(SstarBachState *s)
{
    return s->playback && (s->regs[RD_CTRL] & RD_ENABLE) && rd_size(s);
}

static uint32_t rd_level_units(SstarBachState *s)
{
    return MIN(s->rd_level >> s->unit_shift, 0xFFFF);
}

static void rd_init(SstarBachState *s)
{
    s->rd_level = s->rd_ptr = 0;
    s->rd_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void bach_out_cb(void *opaque, int avail)
{
    SstarBachState *s = opaque;

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
static void bach_voice(SstarBachState *s)
{
    bool on = rd_running(s);
    uint32_t rate = rd_rate(s);
    uint8_t chans = rd_chans(s);

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

        /*
         * What is queued stays queued: it has left the DMA ring, so
         * dropping it here would lose it.  A rate change mid-clip plays
         * its last few milliseconds at the new rate.
         */
        s->vout = AUD_open_out(s->audio_be, NULL, "sstar-bach.out", s,
                               bach_out_cb, &as);
        s->voice_rate = rate;
        s->voice_chans = chans;
    }
    if (s->vout) {
        AUD_set_active_out(s->vout, on);
    }
}

/* the reader plays n bytes from its pointer */
static void rd_play(SstarBachState *s, uint32_t n)
{
    uint32_t size = rd_size(s);
    uint64_t base = rd_base(s);
    uint8_t buf[1024];

    if (s->rd_ptr >= size) {
        /* the buffer shrank under a running reader */
        s->rd_ptr = 0;
    }
    while (n) {
        uint32_t k = MIN(MIN(n, sizeof(buf)), size - s->rd_ptr);
        uint64_t a = base + s->rd_ptr;
        uint32_t i;

        if (s->vout) {
            /* through the MIU's MMU where a die has one */
            for (i = 0; i < k; ) {
                uint32_t run = MIN(k - i,
                                   SS_PAGE - ((a + i) & (SS_PAGE - 1)));

                dma_memory_read(&address_space_memory,
                                SS_DRAM_BASE + sstar_miu_xlate(a + i),
                                buf + i, run, MEMTXATTRS_UNSPECIFIED);
                i += run;
            }
            for (i = 0; i < k && s->out_len < OUT_RING; i++) {
                s->out[(s->out_head + s->out_len++) % OUT_RING] = buf[i];
            }
        }
        s->rd_ptr = (s->rd_ptr + k) % size;
        n -= k;
    }
}

/* play what the virtual clock says has gone out since the last look */
static void rd_sync(SstarBachState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t br, unit = 1u << s->unit_shift;
    uint64_t n;

    if (!rd_running(s)) {
        return;
    }
    br = rd_byte_rate(s);
    n = (uint64_t)MAX(now - s->rd_t, 0) * br / NANOSECONDS_PER_SECOND;
    n &= ~(uint64_t)(unit - 1);
    if (!s->rd_level) {
        s->rd_t = now;
    } else if (n) {
        n = MIN(n, s->rd_level);
        rd_play(s, n);
        s->rd_level -= n;
        s->rd_t += n * NANOSECONDS_PER_SECOND / br;
        if (!s->rd_level) {
            s->rd_t = now;
        }
    }
    if (!s->rd_level && !(s->regs[AI_IRQEN] & RD_EMPTY_CLR)) {
        s->rd_empty = true;
    }
}

static uint16_t rd_flags(SstarBachState *s)
{
    uint32_t lv = rd_level_units(s);
    uint16_t f = s->rd_empty ? RD_EMPTY : 0;

    if (s->regs[RD_UNDERRUN_TH] && lv < s->regs[RD_UNDERRUN_TH]) {
        f |= RD_UNDERRUN;
    }
    if (lv >= s->regs[RD_OVERRUN_TH]) {
        f |= RD_OVERRUN;
    }
    return f;
}

static uint64_t bach_base(SstarBachState *s)
{
    return ((uint64_t)(s->regs[AI_ADDR_HI] & 0x7FFF) << (12 + s->unit_shift)) |
           ((uint64_t)(s->regs[AI_ADDR_LO] & 0xFFF) << s->unit_shift);
}

static uint32_t bach_size(SstarBachState *s)
{
    return (uint32_t)s->regs[AI_SIZE] << s->unit_shift;
}

static void bach_update(SstarBachState *s)
{
    uint32_t thresh = (uint32_t)s->regs[AI_THRESH] << s->unit_shift;
    uint16_t f = s->regs[AI_FLAGS] & ~(FLAG_HIGH | FLAG_FULL);
    uint16_t en = s->regs[AI_IRQEN];
    bool on = s->regs[AI_ADDR_LO] & AI_RUN;
    bool rd = false;

    if (s->level >= bach_size(s) && bach_size(s)) {
        f |= FLAG_FULL;
    }
    if (thresh && s->level >= thresh) {
        f |= FLAG_HIGH;
    }
    if (s->playback) {
        f = (f & ~RD_FLAGS) | rd_flags(s);
        rd = ((f & RD_UNDERRUN) && (en & RD_UNDERRUN_EN)) ||
             ((f & RD_EMPTY) && (en & RD_EMPTY_EN));
    }
    s->regs[AI_FLAGS] = f;
    qemu_set_irq(s->irq, rd ||
                 (on && (((f & FLAG_HIGH) && (en & EN_THRESH)) ||
                         ((f & FLAG_OVERRUN) && (en & EN_OVERRUN)))));
}

static void bach_fill(SstarBachState *s, uint32_t n)
{
    uint32_t size = bach_size(s);
    uint64_t base = bach_base(s);
    uint8_t buf[512];

    while (n >= 2) {
        uint32_t chunk = MIN(MIN(n, sizeof(buf)), size - s->wptr) & ~1u;
        uint32_t i;
        hwaddr a = base + s->wptr;

        if (!chunk) {
            break;
        }
        for (i = 0; i < chunk; i += 2) {
            int16_t v = (int16_t)(8000 * sin(2 * M_PI * 1000 *
                                             (s->sample / MAX(s->channels, 1)) /
                                             MAX(s->sample_rate, 1)));

            s->sample++;
            buf[i] = v & 0xFF;
            buf[i + 1] = (v >> 8) & 0xFF;
        }
        /* DMA goes through the MIU's MMU where a die has one */
        for (i = 0; i < chunk; ) {
            uint32_t run = MIN(chunk - i, SS_PAGE - ((a + i) & (SS_PAGE - 1)));

            dma_memory_write(&address_space_memory,
                             SS_DRAM_BASE + sstar_miu_xlate(a + i), buf + i,
                             run, MEMTXATTRS_UNSPECIFIED);
            i += run;
        }
        s->wptr = (s->wptr + chunk) % size;
        n -= chunk;
    }
}

/* the microphone's side of a tick */
static void ai_tick(SstarBachState *s, int64_t now)
{
    uint32_t size = bach_size(s);

    if (!(s->regs[AI_ADDR_LO] & AI_RUN) || !size) {
        return;
    }
    if (s->last) {
        uint64_t want = (uint64_t)(now - s->last) * s->rate /
                        NANOSECONDS_PER_SECOND;
        uint64_t n = MIN(want, (uint64_t)(size - s->level)) & ~7ull;

        if (n) {
            bach_fill(s, n);
            s->level += n;
            s->last += (int64_t)(n * NANOSECONDS_PER_SECOND /
                                 MAX(s->rate, 1));
        }
        if (s->level >= size && want > n) {
            /* full: the microphone drops samples, which is the overrun */
            s->regs[AI_FLAGS] |= FLAG_OVERRUN;
            s->last = now;
        }
    } else {
        s->last = now;
    }
}

static void bach_tick(void *opaque)
{
    SstarBachState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    ai_tick(s, now);
    rd_sync(s);
    bach_update(s);
    if (((s->regs[AI_ADDR_LO] & AI_RUN) && bach_size(s)) ||
        (rd_running(s) && s->rd_level)) {
        timer_mod(s->tick, now + TICK_NS);
    }
}

/* the tick runs while a channel can move data, whatever order it was set up */
static void bach_kick(SstarBachState *s)
{
    if ((((s->regs[AI_ADDR_LO] & AI_RUN) && bach_size(s)) ||
         (rd_running(s) && s->rd_level)) && !timer_pending(s->tick)) {
        timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TICK_NS);
    }
}

static uint64_t sstar_bach_read(void *opaque, hwaddr off, unsigned size)
{
    SstarBachState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    if (s->playback) {
        rd_sync(s);
        bach_update(s);
    }
    v = s->regs[(off >> 2) & (BACH_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_bach_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarBachState *s = opaque;
    int reg = (off >> 2) & (BACH_REGS - 1);
    uint16_t mask, v, old;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }
    if (reg == AI_LEVEL || reg == AI_FLAGS ||
        (s->playback && reg == RD_LEVEL)) {
        return;
    }
    if (s->playback) {
        rd_sync(s);
    }
    old = s->regs[reg];
    s->regs[reg] = (old & ~mask) | (v & mask);
    if (s->trace) {
        qemu_log("sstar-bach: W 0x%02x = 0x%04x\n", reg << 1, s->regs[reg]);
    }
    switch (reg) {
    case AI_IRQEN:
        if (s->regs[reg] & CLR_OVERRUN) {
            s->regs[AI_FLAGS] &= ~FLAG_OVERRUN;
        }
        if (s->regs[reg] & RD_EMPTY_CLR) {
            s->rd_empty = false;
        }
        break;
    case RD_CTRL:
        if (!s->playback) {
            break;
        }
        if ((s->regs[reg] & RD_INIT) && !(old & RD_INIT)) {
            rd_init(s);
        }
        if ((s->regs[reg] & RD_ENABLE) && !(old & RD_ENABLE)) {
            /* the stream starts now; nothing has played yet */
            s->rd_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        if ((s->regs[reg] ^ old) & RD_TRIG) {
            uint32_t n = (uint32_t)s->regs[RD_SIZE] << s->unit_shift;

            if (!s->rd_level) {
                s->rd_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            }
            s->rd_level = MIN(s->rd_level + n, rd_size(s));
        }
        if (s->regs[reg] & RD_LATCH) {
            s->regs[RD_LEVEL] = rd_level_units(s);
        }
        bach_voice(s);
        break;
    case SR0_SEL:
    case TEST_CTRL7:
    case RD_BUF_SIZE:
        if (s->playback) {
            bach_voice(s);
        }
        break;
    case AI_ADDR_LO:
        if ((s->regs[reg] & AI_RUN) && !(old & AI_RUN)) {
            s->regs[AI_FLAGS] &= ~FLAG_OVERRUN;
            s->level = s->wptr = 0;
            s->last = 0;
            bach_tick(s);
        }
        if (s->regs[reg] & AI_LATCH) {
            s->regs[AI_LEVEL] = MIN(s->level >> s->unit_shift, 0xFFFF);
        }
        if ((s->regs[reg] ^ old) & AI_TRIG) {
            uint32_t n = (uint32_t)s->regs[AI_READ] << s->unit_shift;

            s->level -= MIN(n, s->level);
        }
        break;
    }
    if (s->playback) {
        /* RD_EMPTY latches again as soon as its clear falls */
        rd_sync(s);
    }
    bach_update(s);
    bach_kick(s);
}

static const MemoryRegionOps sstar_bach_ops = {
    .read = sstar_bach_read,
    .write = sstar_bach_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_bach_reset(DeviceState *dev)
{
    SstarBachState *s = SSTAR_BACH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->level = s->wptr = 0;
    s->sample = 0;
    s->last = 0;
    rd_init(s);
    s->rd_empty = false;
    s->out_head = s->out_len = 0;
    timer_del(s->tick);
    if (s->vout) {
        AUD_set_active_out(s->vout, false);
    }
    qemu_set_irq(s->irq, 0);
}

static void sstar_bach_realize(DeviceState *dev, Error **errp)
{
    SstarBachState *s = SSTAR_BACH(dev);

    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, bach_tick, s);
    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
}

static void sstar_bach_unrealize(DeviceState *dev)
{
    SstarBachState *s = SSTAR_BACH(dev);

    if (s->vout) {
        AUD_close_out(s->audio_be, s->vout);
        s->vout = NULL;
    }
    timer_free(s->tick);
}

static void sstar_bach_init(Object *obj)
{
    SstarBachState *s = SSTAR_BACH(obj);

    memory_region_init_io(&s->iomem, obj, &sstar_bach_ops, s,
                          TYPE_SSTAR_BACH, BACH_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_bach_props[] = {
    DEFINE_PROP_UINT32("rate", SstarBachState, rate, 32000),
    DEFINE_PROP_UINT32("sample-rate", SstarBachState, sample_rate, 16000),
    DEFINE_PROP_UINT32("unit-shift", SstarBachState, unit_shift, 3),
    DEFINE_PROP_UINT32("channels", SstarBachState, channels, 1),
    DEFINE_PROP_BOOL("playback", SstarBachState, playback, false),
    DEFINE_PROP_BOOL("trace", SstarBachState, trace, false),
    DEFINE_AUDIO_PROPERTIES(SstarBachState, audio_be),
};

static int sstar_bach_post_load(void *opaque, int version_id)
{
    SstarBachState *s = opaque;

    if (version_id < 2) {
        /* from before the reader: nothing queued, nothing played */
        rd_init(s);
        s->rd_empty = false;
        s->out_head = s->out_len = 0;
    }
    if (s->out_head >= OUT_RING || s->out_len > OUT_RING ||
        (rd_size(s) && s->rd_ptr >= rd_size(s))) {
        return -EINVAL;
    }
    /* the host voice is not state: open it for a running reader */
    bach_voice(s);
    bach_kick(s);
    return 0;
}

static const VMStateDescription vmstate_sstar_bach = {
    .name = TYPE_SSTAR_BACH,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = sstar_bach_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarBachState, BACH_REGS),
        VMSTATE_UINT32(level, SstarBachState),
        VMSTATE_UINT32(wptr, SstarBachState),
        VMSTATE_UINT64(sample, SstarBachState),
        VMSTATE_INT64(last, SstarBachState),
        VMSTATE_TIMER_PTR(tick, SstarBachState),
        VMSTATE_UINT32_V(rd_level, SstarBachState, 2),
        VMSTATE_UINT32_V(rd_ptr, SstarBachState, 2),
        VMSTATE_INT64_V(rd_t, SstarBachState, 2),
        VMSTATE_BOOL_V(rd_empty, SstarBachState, 2),
        VMSTATE_UINT8_ARRAY_V(out, SstarBachState, OUT_RING, 2),
        VMSTATE_UINT32_V(out_head, SstarBachState, 2),
        VMSTATE_UINT32_V(out_len, SstarBachState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_bach_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_bach_realize;
    dc->unrealize = sstar_bach_unrealize;
    device_class_set_legacy_reset(dc, sstar_bach_reset);
    device_class_set_props(dc, sstar_bach_props);
    dc->vmsd = &vmstate_sstar_bach;
}

static const TypeInfo sstar_bach_info = {
    .name           = TYPE_SSTAR_BACH,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarBachState),
    .instance_init  = sstar_bach_init,
    .class_init     = sstar_bach_class_init,
};

static void sstar_bach_register_types(void)
{
    type_register_static(&sstar_bach_info);
}

type_init(sstar_bach_register_types)
