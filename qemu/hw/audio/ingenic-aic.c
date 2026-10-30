/*
 * Ingenic AIC (I2S controller) at 0x10020000 and its internal codec at
 * 0x10021000, as the vendor audio.ko runs them (SDK drivers/audio/oss2
 * devices/xb47xx_i2s_v12.[ch] on T10..T31, oss3 host/T23/audio_aic.[ch]
 * on T23, the same register set on T40/T41).
 *
 *   0x00 AICFR   0 ENB, 3 RST (pulses: empties the FIFOs), 13/14 FIFO
 *                reset status (0), 20:16 TFTH, 27:24 RFTH
 *   0x04 AICCR   0 EREC, 1 ERPL, 2 ENLBF, 3..6 interrupt enables (TFS,
 *                RFS, TUR, ROR), 7/8 RFLUSH/TFLUSH (pulse), 11 M2S (a
 *                mono stream played on both channels), 12 MONOCTR_RIGHT
 *                / 13 MONOCTR (record one channel), 14 TDMS / 15 RDMS
 *                (DMA), 18:16 ISS / 21:19 OSS (0 8, 1 16, 2 18, 3 20,
 *                4 24 bits), 22 TLDMS (loop DMA), 23 ETFL (the loop
 *                FIFO), 26:24 CHANNEL (channels played - 1), 28 PACK16
 *   0x14 AICSR   3 TFS, 4 RFS, 5 TUR, 6 ROR (both write 0 to clear),
 *                13:8 TFL, 29:24 RFL
 *   0x1C I2SSR   busy bits, 0
 *   0x34 AICDR   the FIFOs: a write plays a sample (with PACK16 a word
 *                is two 16-bit ones), a read takes a recorded one
 *   0x38 AICLR   the loop FIFO: what was played, for echo cancelling
 *   others       storage (AICCR1/2, I2SCR, I2SDIV, CKCFG, ...)
 *
 * The codec's registers are bytes, one per word; the rest hold what is
 * written: the drivers read back each one they write.
 *
 * The sample rates: on T23, T31, T40 and T41 the AIC is the I2S master,
 * clocked by the CPM's speaker clock to play and its microphone clock to
 * record, which the drivers set to 256 fs for each direction ("cpm" link,
 * ingenic_cpm_i2s_hz; a rate within 3 % of a standard one is taken as
 * that).  On T10..T30, or while that clock is off, the codec clocks the
 * AIC: its CSRR (0x110, bits 2:0: 7 8 kHz, 6 12, 5 16, 4 24, 3 32, 2
 * 44.1, 1 48, 0 96) is the rate both ways.
 *
 * Frames move at those rates while ENB is set: with ERPL one frame's
 * samples leave the TX FIFO (1 with M2S, CHANNEL + 1 otherwise; the last
 * sample again on underrun) for the machine's audiodev (16-bit stereo),
 * with ETFL the loop FIFO gets the frame's first sample, and with EREC
 * the RX FIFO gets one sample per frame (MONOCTR*) or two: a sine of
 * "tone-hz" (1 kHz; 0 for silence) at a quarter of full scale, or with
 * rx-source=audiodev the host's input.  The FIFOs are the hardware's: 64
 * entries to play, 32 to record and loop.
 *
 * The PDMA moves the samples on request lines: "drq-tx" (request type 6)
 * while TDMS is set and the TX FIFO has room, "drq-rx" (7) while RDMS is
 * set and the RX FIFO holds a sample, "drq-loop" (5) while TLDMS is set
 * and the loop FIFO does.  The interrupt is AICSR's TFS/RFS/TUR/ROR under
 * their AICCR enables.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "hw/mips/ingenic.h"
#include "migration/vmstate.h"
#include "qemu/audio.h"
#include "qom/object.h"

#define TYPE_INGENIC_AIC "ingenic-aic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAicState, INGENIC_AIC)

#define AIC_REGS        0x100
#define CODEC_REGS      0x100       /* bytes, at 4 n */

#define R_AICFR         0x00
#define R_AICCR         0x04
#define R_AICSR         0x14
#define R_I2SSR         0x1C
#define R_AICDR         0x34
#define R_AICLR         0x38
#define CODEC_CSRR      (0x110 / 4)

#define FR_ENB          (1u << 0)
#define FR_RST          (1u << 3)
#define FR_FIFO_RST     (3u << 13)

#define CR_EREC         (1u << 0)
#define CR_ERPL         (1u << 1)
#define CR_ETFS         (1u << 3)
#define CR_ERFS         (1u << 4)
#define CR_ETUR         (1u << 5)
#define CR_EROR         (1u << 6)
#define CR_RFLUSH       (1u << 7)
#define CR_TFLUSH       (1u << 8)
#define CR_M2S          (1u << 11)
#define CR_MONO_RIGHT   (1u << 12)
#define CR_MONO         (1u << 13)
#define CR_TDMS         (1u << 14)
#define CR_RDMS         (1u << 15)
#define CR_TLDMS        (1u << 22)
#define CR_ETFL         (1u << 23)
#define CR_PACK16       (1u << 28)

#define SR_TFS          (1u << 3)
#define SR_RFS          (1u << 4)
#define SR_TUR          (1u << 5)
#define SR_ROR          (1u << 6)

#define TX_DEPTH        64
#define RX_DEPTH        32
#define RING            64
#define OUT_RING        8192        /* int16, stereo frames to the host */
#define IN_RING         8192        /* int16, the host's input */
#define TICK_NS         (1 * SCALE_MS)

typedef struct AicFifo {
    uint32_t data[RING];
    uint32_t head, count;
} AicFifo;

struct IngenicAicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion codec_mem;
    qemu_irq irq;
    qemu_irq drq_tx, drq_rx, drq_loop;
    QEMUTimer *timer;
    AudioBackend *audio_be;
    SWVoiceOut *vout;
    SWVoiceIn *vin;
    uint32_t voice_rate, voice_rate_in;
    DeviceState *cpm;

    uint32_t regs[AIC_REGS / 4];
    uint8_t codec[CODEC_REGS];
    AicFifo tx, rx, loop;
    uint32_t last_tx[8];        /* the frame last played */
    int64_t last_ns;
    uint64_t frac;              /* ns * rate carried over: play */
    uint64_t frac_rx;           /* record */
    uint64_t phase;             /* tone frames made */

    int16_t out[OUT_RING];
    uint32_t out_head, out_count;
    int16_t in[IN_RING];
    uint32_t in_head, in_count;

    uint32_t tone_hz;
    char *rx_source;
};

static uint32_t *aic_reg(IngenicAicState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static bool aic_rx_audiodev(IngenicAicState *s)
{
    return s->rx_source && !strcmp(s->rx_source, "audiodev");
}

static const uint32_t aic_rates[8] = {
    96000, 48000, 44100, 32000, 24000, 16000, 12000, 8000
};

/* the play (tx) or record sample rate */
static uint32_t aic_rate(IngenicAicState *s, bool tx)
{
    uint64_t hz = s->cpm ? ingenic_cpm_i2s_hz(s->cpm, tx) : 0;
    uint32_t fs = (hz + 128) / 256;
    int i;

    if (fs < 4000 || fs > 192000) {
        return aic_rates[s->codec[CODEC_CSRR] & 7];
    }
    for (i = 0; i < 8; i++) {
        if (fs * 100 >= aic_rates[i] * 97 && fs * 100 <= aic_rates[i] * 103) {
            return aic_rates[i];
        }
    }
    return fs;
}

/* ---- FIFOs and request lines ----------------------------------------- */

static bool fifo_push(AicFifo *f, uint32_t v, uint32_t depth)
{
    if (f->count >= depth) {
        return false;
    }
    f->data[(f->head + f->count) % RING] = v;
    f->count++;
    return true;
}

static bool fifo_pop(AicFifo *f, uint32_t *v)
{
    if (!f->count) {
        return false;
    }
    *v = f->data[f->head];
    f->head = (f->head + 1) % RING;
    f->count--;
    return true;
}

static void aic_update(IngenicAicState *s)
{
    uint32_t cr = *aic_reg(s, R_AICCR);
    uint32_t *sr = aic_reg(s, R_AICSR);
    uint32_t tfth = extract32(*aic_reg(s, R_AICFR), 16, 5);
    uint32_t rfth = extract32(*aic_reg(s, R_AICFR), 24, 4);

    *sr &= ~(SR_TFS | SR_RFS);
    *sr |= (s->tx.count <= tfth * 2 ? SR_TFS : 0) |
           (s->rx.count > rfth * 2 ? SR_RFS : 0);
    *sr = deposit32(*sr, 8, 6, MIN(s->tx.count, 63));
    *sr = deposit32(*sr, 24, 6, MIN(s->rx.count, 63));
    qemu_set_irq(s->irq, ((*sr & SR_TFS) && (cr & CR_ETFS)) ||
                         ((*sr & SR_RFS) && (cr & CR_ERFS)) ||
                         ((*sr & SR_TUR) && (cr & CR_ETUR)) ||
                         ((*sr & SR_ROR) && (cr & CR_EROR)));
    qemu_set_irq(s->drq_tx, (cr & CR_TDMS) && s->tx.count < TX_DEPTH);
    qemu_set_irq(s->drq_rx, (cr & CR_RDMS) && s->rx.count);
    qemu_set_irq(s->drq_loop, (cr & CR_TLDMS) && s->loop.count);
}

/* ---- samples --------------------------------------------------------- */

/* a 16-bit sample in a FIFO entry of "code" bits (ISS / OSS) */
static uint32_t aic_to_width(int16_t v, uint32_t code)
{
    static const int shift[8] = { -8, 0, 2, 4, 8, 8, 8, 8 };
    int32_t x = v;

    x = shift[code & 7] < 0 ? x >> 8 : x * (1 << shift[code & 7]);
    return x & 0xffffff;
}

static int16_t aic_from_width(uint32_t e, uint32_t code)
{
    static const int bits[8] = { 8, 16, 18, 20, 24, 24, 24, 24 };
    int b = bits[code & 7];
    int32_t x = sextract32(e, 0, b);

    return b >= 16 ? x >> (b - 16) : x << 8;
}

static int16_t aic_capture(IngenicAicState *s)
{
    int16_t v = 0;

    if (aic_rx_audiodev(s)) {
        if (s->in_count) {
            v = s->in[s->in_head];
            s->in_head = (s->in_head + 1) % IN_RING;
            s->in_count--;
        }
    } else if (s->tone_hz) {
        double t = (double)s->phase / aic_rate(s, false);

        v = (int16_t)lrint(8192 * sin(2 * M_PI * s->tone_hz * t));
    }
    return v;
}

/* one frame played: the TX FIFO to the host, and the loop FIFO */
static void aic_play_frame(IngenicAicState *s)
{
    uint32_t cr = *aic_reg(s, R_AICCR);
    uint32_t *sr = aic_reg(s, R_AICSR);
    uint32_t oss = extract32(cr, 19, 3);
    int i, n;

    if (cr & CR_ERPL) {
        n = (cr & CR_M2S) ? 1 : extract32(cr, 24, 3) + 1;
        for (i = 0; i < n; i++) {
            if (!fifo_pop(&s->tx, &s->last_tx[i])) {
                *sr |= SR_TUR;      /* the last sample again */
            }
        }
        if (s->vout && s->out_count + 2 <= OUT_RING) {
            int16_t l = aic_from_width(s->last_tx[0], oss);
            int16_t r = n > 1 ? aic_from_width(s->last_tx[1], oss) : l;
            uint32_t at = (s->out_head + s->out_count) % OUT_RING;

            s->out[at] = l;
            s->out[(at + 1) % OUT_RING] = r;
            s->out_count += 2;
        }
    }
    if (cr & CR_ETFL) {
        fifo_push(&s->loop, (cr & CR_ERPL) ? s->last_tx[0] : 0, RX_DEPTH);
    }
    aic_update(s);              /* the PDMA moves what it can now */
}

/* one frame recorded into the RX FIFO */
static void aic_rec_frame(IngenicAicState *s)
{
    uint32_t cr = *aic_reg(s, R_AICCR);
    uint32_t *sr = aic_reg(s, R_AICSR);
    uint32_t iss = extract32(cr, 16, 3);
    int i, n;

    if (cr & CR_EREC) {
        uint32_t v = aic_to_width(aic_capture(s), iss);

        n = (cr & (CR_MONO | CR_MONO_RIGHT)) ? 1 : 2;
        for (i = 0; i < n; i++) {
            if (!fifo_push(&s->rx, v, RX_DEPTH)) {
                *sr |= SR_ROR;
            }
        }
        s->phase++;
    }
    aic_update(s);              /* the PDMA moves what it can now */
}

static bool aic_running(IngenicAicState *s)
{
    return (*aic_reg(s, R_AICFR) & FR_ENB) &&
           (*aic_reg(s, R_AICCR) & (CR_EREC | CR_ERPL | CR_ETFL));
}

/* the frames due in dt ns at rate, the remainder kept in *frac */
static uint64_t aic_frames(uint64_t *frac, int64_t dt, uint32_t rate)
{
    uint64_t n;

    *frac += (uint64_t)dt * rate;
    n = *frac / NANOSECONDS_PER_SECOND;
    *frac %= NANOSECONDS_PER_SECOND;
    return n;
}

static void aic_advance(IngenicAicState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t dt = MIN(now - s->last_ns, 100 * TICK_NS);
    uint32_t cr = *aic_reg(s, R_AICCR);
    uint64_t ntx = 0, nrx = 0;

    s->last_ns = now;
    if (dt <= 0 || !aic_running(s)) {
        return;
    }
    if (cr & (CR_ERPL | CR_ETFL)) {
        ntx = aic_frames(&s->frac, dt, aic_rate(s, true));
    }
    if (cr & CR_EREC) {
        nrx = aic_frames(&s->frac_rx, dt, aic_rate(s, false));
    }
    /* the two sides interleaved, so that neither runs a tick ahead */
    while (ntx || nrx) {
        if (ntx && (!nrx || ntx >= nrx)) {
            aic_play_frame(s);
            ntx--;
        } else {
            aic_rec_frame(s);
            nrx--;
        }
    }
}

static void aic_schedule(IngenicAicState *s)
{
    if (aic_running(s)) {
        if (!timer_pending(s->timer)) {
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                TICK_NS);
        }
    } else {
        timer_del(s->timer);
    }
}

/* ---- the host's audio ------------------------------------------------ */

static void aic_out_cb(void *opaque, int avail)
{
    IngenicAicState *s = opaque;

    while (avail >= 4 && s->out_count >= 2) {
        uint32_t n = MIN(MIN((uint32_t)avail / 2, s->out_count),
                         OUT_RING - s->out_head) & ~1u;
        size_t done = n ? AUD_write(s->vout, &s->out[s->out_head], n * 2) : 0;

        if (!done) {
            break;
        }
        s->out_head = (s->out_head + done / 2) % OUT_RING;
        s->out_count -= done / 2;
        avail -= done;
    }
}

static void aic_in_cb(void *opaque, int avail)
{
    IngenicAicState *s = opaque;
    int16_t buf[256];

    while (avail > 0) {
        size_t n = AUD_read(s->vin, buf, MIN(sizeof(buf), (size_t)avail));
        size_t i;

        if (!n) {
            break;
        }
        for (i = 0; i < n / 2; i++) {
            if (s->in_count < IN_RING) {
                s->in[(s->in_head + s->in_count) % IN_RING] = buf[i];
                s->in_count++;
            }
        }
        avail -= n;
    }
}

static void aic_voices(IngenicAicState *s)
{
    uint32_t cr = *aic_reg(s, R_AICCR);
    bool on = *aic_reg(s, R_AICFR) & FR_ENB;
    uint32_t rate = aic_rate(s, true), rate_in = aic_rate(s, false);

    if (!s->audio_be) {
        return;
    }
    if (s->voice_rate != rate && s->vout) {
        AUD_close_out(s->audio_be, s->vout);
        s->vout = NULL;
    }
    if (s->voice_rate_in != rate_in && s->vin) {
        AUD_close_in(s->audio_be, s->vin);
        s->vin = NULL;
    }
    s->voice_rate = rate;
    s->voice_rate_in = rate_in;
    if (!s->vout) {
        struct audsettings as = {
            .freq = rate, .nchannels = 2, .fmt = AUDIO_FORMAT_S16,
        };

        s->vout = AUD_open_out(s->audio_be, NULL, "ingenic-aic.out", s,
                               aic_out_cb, &as);
    }
    if (!s->vin && aic_rx_audiodev(s)) {
        struct audsettings as = {
            .freq = rate_in, .nchannels = 1, .fmt = AUDIO_FORMAT_S16,
        };

        s->vin = AUD_open_in(s->audio_be, NULL, "ingenic-aic.in", s,
                             aic_in_cb, &as);
    }
    if (s->vout) {
        AUD_set_active_out(s->vout, on && (cr & CR_ERPL));
    }
    if (s->vin) {
        AUD_set_active_in(s->vin, on && (cr & CR_EREC));
    }
}

static void aic_tick(void *opaque)
{
    IngenicAicState *s = opaque;

    aic_advance(s);
    if (s->voice_rate != aic_rate(s, true) ||
        s->voice_rate_in != aic_rate(s, false)) {
        aic_voices(s);          /* the CPM's clocks changed */
    }
    aic_schedule(s);
}

/* ---- registers ------------------------------------------------------- */

static uint64_t ingenic_aic_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicAicState *s = opaque;
    uint32_t v = 0;

    if (off >= AIC_REGS) {
        return 0;
    }
    switch (off & ~3) {
    case R_AICDR:
        aic_advance(s);
        fifo_pop(&s->rx, &v);
        aic_update(s);
        return v;
    case R_AICLR:
        aic_advance(s);
        fifo_pop(&s->loop, &v);
        aic_update(s);
        return v;
    case R_I2SSR:
        return 0;
    case R_AICSR:
        aic_advance(s);
        aic_update(s);
        break;
    }
    return extract32(*aic_reg(s, off & ~3), (off & 3) * 8, size * 8);
}

static void ingenic_aic_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicAicState *s = opaque;
    uint32_t *reg, v;

    if (off >= AIC_REGS) {
        return;
    }
    aic_advance(s);
    reg = aic_reg(s, off & ~3);
    if (size < 4) {
        uint32_t mask = MAKE_64BIT_MASK((off & 3) * 8, size * 8);

        val = (*reg & ~mask) | ((val << (off & 3) * 8) & mask);
    }
    v = val;
    switch (off & ~3) {
    case R_AICDR:
        if (*aic_reg(s, R_AICCR) & CR_PACK16) {
            fifo_push(&s->tx, v & 0xffff, TX_DEPTH);
            fifo_push(&s->tx, v >> 16, TX_DEPTH);
        } else {
            fifo_push(&s->tx, v & 0xffffff, TX_DEPTH);
        }
        break;
    case R_AICLR:
    case R_I2SSR:
        break;
    case R_AICFR:
        if (v & FR_RST) {
            s->tx.count = s->rx.count = s->loop.count = 0;
            memset(s->last_tx, 0, sizeof(s->last_tx));
        }
        *reg = v & ~(FR_RST | FR_FIFO_RST);
        break;
    case R_AICCR:
        if (v & CR_RFLUSH) {
            s->rx.count = 0;
        }
        if (v & CR_TFLUSH) {
            s->tx.count = 0;
        }
        *reg = v & ~(CR_RFLUSH | CR_TFLUSH);
        break;
    case R_AICSR:
        /* TUR and ROR clear by writing 0; the levels are read only */
        *reg &= v | ~(SR_TUR | SR_ROR);
        break;
    default:
        *reg = v;
        break;
    }
    if ((off & ~3) == R_AICFR || (off & ~3) == R_AICCR) {
        s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        aic_voices(s);
        aic_schedule(s);
    }
    aic_update(s);
}

static const MemoryRegionOps ingenic_aic_ops = {
    .read = ingenic_aic_read,
    .write = ingenic_aic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static uint64_t ingenic_codec_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicAicState *s = opaque;

    return s->codec[(off >> 2) % CODEC_REGS];
}

static void ingenic_codec_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    IngenicAicState *s = opaque;

    aic_advance(s);
    s->codec[(off >> 2) % CODEC_REGS] = val;
    if ((off >> 2) == CODEC_CSRR) {
        aic_voices(s);              /* a new rate */
    }
}

static const MemoryRegionOps ingenic_codec_ops = {
    .read = ingenic_codec_read,
    .write = ingenic_codec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ---- the device ------------------------------------------------------ */

static void ingenic_aic_reset(DeviceState *dev)
{
    IngenicAicState *s = INGENIC_AIC(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->codec, 0, sizeof(s->codec));
    memset(s->last_tx, 0, sizeof(s->last_tx));
    s->tx.head = s->tx.count = 0;
    s->rx.head = s->rx.count = 0;
    s->loop.head = s->loop.count = 0;
    s->out_head = s->out_count = s->in_head = s->in_count = 0;
    s->frac = s->frac_rx = s->phase = 0;
    s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    aic_voices(s);
    aic_update(s);
}

static void ingenic_aic_realize(DeviceState *dev, Error **errp)
{
    IngenicAicState *s = INGENIC_AIC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &ingenic_aic_ops, s,
                          TYPE_INGENIC_AIC, 0x1000);
    /*
     * A register write that raises a request has the PDMA access the
     * FIFOs at once, from inside this region's own handler.
     */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->codec_mem, OBJECT(s), &ingenic_codec_ops, s,
                          "ingenic-codec", 0x1000);
    sysbus_init_mmio(sbd, &s->codec_mem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->drq_tx, "drq-tx", 1);
    qdev_init_gpio_out_named(dev, &s->drq_rx, "drq-rx", 1);
    qdev_init_gpio_out_named(dev, &s->drq_loop, "drq-loop", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic_tick, s);
}

static void ingenic_aic_unrealize(DeviceState *dev)
{
    IngenicAicState *s = INGENIC_AIC(dev);

    if (s->vout) {
        AUD_close_out(s->audio_be, s->vout);
    }
    if (s->vin) {
        AUD_close_in(s->audio_be, s->vin);
    }
    timer_free(s->timer);
}

static int ingenic_aic_post_load(void *opaque, int version_id)
{
    IngenicAicState *s = opaque;

    s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->voice_rate = s->voice_rate_in = 0;
    aic_voices(s);
    aic_schedule(s);
    return 0;
}

static const Property ingenic_aic_props[] = {
    DEFINE_AUDIO_PROPERTIES(IngenicAicState, audio_be),
    DEFINE_PROP_UINT32("tone-hz", IngenicAicState, tone_hz, 1000),
    DEFINE_PROP_STRING("rx-source", IngenicAicState, rx_source),
    DEFINE_PROP_LINK("cpm", IngenicAicState, cpm, TYPE_DEVICE, DeviceState *),
};

static const VMStateDescription vmstate_aic_fifo = {
    .name = "ingenic-aic-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(data, AicFifo, RING),
        VMSTATE_UINT32(head, AicFifo),
        VMSTATE_UINT32(count, AicFifo),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_aic = {
    .name = TYPE_INGENIC_AIC,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = ingenic_aic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicAicState, AIC_REGS / 4),
        VMSTATE_UINT8_ARRAY(codec, IngenicAicState, CODEC_REGS),
        VMSTATE_STRUCT(tx, IngenicAicState, 1, vmstate_aic_fifo, AicFifo),
        VMSTATE_STRUCT(rx, IngenicAicState, 1, vmstate_aic_fifo, AicFifo),
        VMSTATE_STRUCT(loop, IngenicAicState, 1, vmstate_aic_fifo, AicFifo),
        VMSTATE_UINT32_ARRAY(last_tx, IngenicAicState, 8),
        VMSTATE_UINT64(frac, IngenicAicState),
        VMSTATE_UINT64(phase, IngenicAicState),
        VMSTATE_UINT64_V(frac_rx, IngenicAicState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_aic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_aic_realize;
    dc->unrealize = ingenic_aic_unrealize;
    device_class_set_legacy_reset(dc, ingenic_aic_reset);
    device_class_set_props(dc, ingenic_aic_props);
    dc->vmsd = &vmstate_ingenic_aic;
}

static const TypeInfo ingenic_aic_info = {
    .name = TYPE_INGENIC_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAicState),
    .class_init = ingenic_aic_class_init,
};

static void ingenic_aic_register_types(void)
{
    type_register_static(&ingenic_aic_info);
}

type_init(ingenic_aic_register_types)
