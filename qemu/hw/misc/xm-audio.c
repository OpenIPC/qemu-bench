/*
 * Xiongmai second-generation audio: the I2S / audio codec window at
 * 0x10060000 (64 KiB), as i2s.ko and acodec.ko drive it.
 *
 * i2s.ko moves samples by programmed I/O through two I2S blocks, 16-bit
 * mono at its "rate" parameter (8000 by default):
 *   +0x6000  AI, capture: i2s1_interrupt on Linux IRQ 48 (SPI 16)
 *   +0x7000  AO, playback: IRQ 49 (SPI 17)
 * Each block (XM530V200's i2s.ko):
 *   +0x00  control: bit 0 on, bit 1 transmit (ao_open sets 3), bit 2
 *          receive (ai_open sets 0x25)
 *   +0x10  transmit status: bit 2 the FIFO wants a chunk (the handler
 *          writes one), bit 1 almost empty, bit 0 underrun
 *   +0x20  receive status: bit 4 a chunk is in (the handler reads one),
 *          bit 3 almost full, bit 0 overrun
 *   +0x38  the FIFO, 16-bit accesses
 * A chunk is 256 samples on XM530V200, 160 (20 ms) in XM535's and
 * XM533's i2s.ko, the "chunk" property; their registers are the same.
 * The handler writes the status words back when done; any write
 * acknowledges.  Everything else in the window (the codec, the third
 * block at +0x5000, clocks and dividers) reads back what was written.
 *
 * Capture is a built-in test signal, a sine at the "tone" property's
 * frequency (1 kHz); playback samples are counted and dropped.  With an
 * audiodev (-global xm-audio.audiodev=ID) playback goes to it, and with
 * mic=on so does capture come from it.  A chunk moves every chunk / rate
 * seconds of virtual time while its direction is on.
 *
 * layout "dvr" / "dvr2": the DVRs' xm_i2s.ko, the same blocks (control,
 * statuses, FIFO) at other places and more of them, each with its own
 * interrupt and chunk:
 *   "dvr"  (XM8520D, 2020): +0x5000 the AHD receivers' capture, eight
 *          channels time-multiplexed (sample i is channel i & 7), 960 a
 *          chunk, IRQ 47; +0x6000 HDMI playback, 480, IRQ 48; +0x7000
 *          the codec's capture and playback, 160, IRQ 49
 *   "dvr2" (XM8521DV200 / XM8536E, 2022): +0x3000 the eight-channel
 *          capture, 960, IRQ 94; +0x4000 / +0x5000 / +0x6000 / +0x7000
 *          playback and talk (the codec's at +0x6000), 160, IRQs 47, 48,
 *          49, 95
 * every channel 8000 samples a second.  The multiplexed capture carries a
 * tone a camera, "tone" + 250 Hz x channel; the codec's block is the one
 * an audiodev plays and records.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "qemu/audio.h"
#include <math.h>

#define TYPE_XM_AUDIO "xm-audio"
OBJECT_DECLARE_SIMPLE_TYPE(XmAudioState, XM_AUDIO)

#define XM_AUDIO_SIZE   0x10000
#define AI_BLOCK        0x6000
#define AO_BLOCK        0x7000

#define I2S_CTRL        0x00
#define I2S_TX_ST       0x10
#define I2S_RX_ST       0x20
#define I2S_FIFO        0x38

#define CTRL_ON         (1 << 0)
#define CTRL_TX         (1 << 1)
#define CTRL_RX         (1 << 2)
#define TX_ST_REQ       (1 << 2)
#define RX_ST_FULL      (1 << 4)

#define CHUNK_MAX       1024        /* samples the handler moves at a time */
#define XM_AUDIO_BLOCKS 5
#define RING            8192        /* audiodev buffering, a power of 2 */

typedef struct XmAudioState XmAudioState;

typedef struct XmI2sBlock {
    XmAudioState *s;
    hwaddr base;
    int chunk;                  /* samples an interrupt */
    int chans;                  /* time-multiplexed channels */
    bool codec;                 /* the audiodev's */
    qemu_irq irq;
    QEMUTimer *timer;
    int16_t rx[CHUNK_MAX];
    int rx_pos, rx_len;
    uint64_t tx_samples;
    int16_t tx[CHUNK_MAX * 2];
    int tx_len;
} XmI2sBlock;

struct XmAudioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[XM_AUDIO_SIZE / 4];
    XmI2sBlock blk[XM_AUDIO_BLOCKS];
    int nblk;
    char *layout;
    uint64_t tdm_phase;         /* multiplexed capture frames generated */
    uint32_t rate;
    uint32_t tone;
    uint32_t chunk;
    uint64_t phase;             /* capture samples generated */
    bool trace;
    bool mic;
    AudioBackend *audio_be;
    SWVoiceOut *out;
    SWVoiceIn *in;
    /* between the backend's callbacks and the chunk ticks, in samples */
    int16_t play[RING], rec[RING];
    unsigned play_rd, play_wr, rec_rd, rec_wr;
};

#define REG(s, b, off)  ((s)->regs[((b)->base + (off)) / 4])

static void xm_audio_update(XmAudioState *s, XmI2sBlock *b)
{
    qemu_set_irq(b->irq, (REG(s, b, I2S_TX_ST) | REG(s, b, I2S_RX_ST)) != 0);
}

static int16_t xm_audio_sample(XmAudioState *s)
{
    const double t = (double)s->phase++ / s->rate;

    return (int16_t)lrint(16000.0 * sin(2 * M_PI * s->tone * t));
}

static bool xm_audio_active(XmAudioState *s, XmI2sBlock *b)
{
    const uint32_t c = REG(s, b, I2S_CTRL);

    return (c & CTRL_ON) && (c & (CTRL_TX | CTRL_RX));
}

static void xm_audio_arm(XmAudioState *s, XmI2sBlock *b)
{
    if (!xm_audio_active(s, b)) {
        timer_del(b->timer);
    } else if (!timer_pending(b->timer)) {
        timer_mod(b->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND * b->chunk /
                  ((uint64_t)MAX(s->rate, 1) * b->chans));
    }
}

/* one chunk period of block n */
static void xm_audio_chunk(XmAudioState *s, int n)
{
    XmI2sBlock *b = &s->blk[n];
    const uint32_t c = REG(s, b, I2S_CTRL);

    if (!(c & CTRL_ON)) {
        return;
    }
    if ((c & CTRL_RX) && b->chans > 1) {
        /* a tone a channel, interleaved */
        for (int i = 0; i < b->chunk; i++) {
            const int ch = i % b->chans;
            const double t = (double)(s->tdm_phase + i / b->chans) / s->rate;

            b->rx[i] = (int16_t)lrint(12000.0 *
                                      sin(2 * M_PI * (s->tone + 250 * ch) * t));
        }
        s->tdm_phase += b->chunk / b->chans;
        b->rx_pos = 0;
        b->rx_len = b->chunk;
        REG(s, b, I2S_RX_ST) |= RX_ST_FULL;
    } else if (c & CTRL_RX) {
        for (int i = 0; i < b->chunk; i++) {
            if (!s->in || !b->codec) {
                b->rx[i] = xm_audio_sample(s);
            } else if (s->rec_rd != s->rec_wr) {
                b->rx[i] = s->rec[s->rec_rd++ % RING];
            } else {
                b->rx[i] = 0;
            }
        }
        b->rx_pos = 0;
        b->rx_len = b->chunk;
        REG(s, b, I2S_RX_ST) |= RX_ST_FULL;
    }
    if (c & CTRL_TX) {
        for (int i = 0; s->out && b->codec && i < b->tx_len; i++) {
            if (s->play_wr - s->play_rd < RING) {
                s->play[s->play_wr++ % RING] = b->tx[i];
            }
        }
        b->tx_len = 0;
        REG(s, b, I2S_TX_ST) |= TX_ST_REQ;
    }
    xm_audio_update(s, b);
}

static void xm_audio_tick(void *opaque)
{
    XmI2sBlock *b = opaque;
    XmAudioState *s = b->s;

    xm_audio_chunk(s, b - s->blk);
    xm_audio_arm(s, b);
}

static XmI2sBlock *xm_audio_block(XmAudioState *s, hwaddr off)
{
    for (int n = 0; n < s->nblk; n++) {
        if (off >= s->blk[n].base && off < s->blk[n].base + 0x1000) {
            return &s->blk[n];
        }
    }
    return NULL;
}

static uint64_t xm_audio_read(void *opaque, hwaddr off, unsigned size)
{
    XmAudioState *s = XM_AUDIO(opaque);
    XmI2sBlock *b = xm_audio_block(s, off);

    if (b && off - b->base == I2S_FIFO) {
        if (b->rx_pos < b->rx_len) {
            return (uint16_t)b->rx[b->rx_pos++];
        }
        return 0;
    }
    return extract32(s->regs[off / 4], (off & 3) * 8, size * 8);
}

static void xm_audio_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    XmAudioState *s = XM_AUDIO(opaque);
    XmI2sBlock *b = xm_audio_block(s, off);

    if (s->trace && !(b && off - b->base == I2S_FIFO)) {
        qemu_log("xm-audio: +0x%04" HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                 off, val);
    }
    if (b) {
        switch (off - b->base) {
        case I2S_FIFO:
            b->tx_samples++;
            if (b->tx_len < ARRAY_SIZE(b->tx)) {
                b->tx[b->tx_len++] = val;
            }
            return;
        case I2S_TX_ST:
        case I2S_RX_ST:
            REG(s, b, off - b->base) = 0;
            xm_audio_update(s, b);
            return;
        case I2S_CTRL:
            REG(s, b, I2S_CTRL) = val;
            xm_audio_arm(s, b);
            return;
        }
    }
    s->regs[off / 4] = deposit32(s->regs[off / 4], (off & 3) * 8, size * 8,
                                 val);
}

static const MemoryRegionOps xm_audio_ops = {
    .read = xm_audio_read,
    .write = xm_audio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void xm_audio_reset(DeviceState *dev)
{
    XmAudioState *s = XM_AUDIO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->play_rd = s->play_wr = s->rec_rd = s->rec_wr = 0;
    s->phase = 0;
    s->tdm_phase = 0;
    for (int n = 0; n < s->nblk; n++) {
        timer_del(s->blk[n].timer);
        s->blk[n].rx_pos = s->blk[n].rx_len = 0;
        s->blk[n].tx_len = 0;
        xm_audio_update(s, &s->blk[n]);
    }
}

static void xm_audio_init(Object *obj)
{
    XmAudioState *s = XM_AUDIO(obj);

    memory_region_init_io(&s->iomem, obj, &xm_audio_ops, s, TYPE_XM_AUDIO,
                          XM_AUDIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (int n = 0; n < XM_AUDIO_BLOCKS; n++) {
        s->blk[n].s = s;
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->blk[n].irq);
        s->blk[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xm_audio_tick,
                                       &s->blk[n]);
    }
}

/* the blocks of a layout: base, chunk, channels, the audiodev's */
static bool xm_audio_layout(XmAudioState *s, Error **errp)
{
    static const struct {
        const char *name;
        struct { hwaddr base; int chunk, chans; bool codec; } b[5];
    } layouts[] = {
        { "dvr", { { 0x5000, 960, 8 }, { 0x6000, 480, 1 },
                   { 0x7000, 160, 1, true } } },
        { "dvr2", { { 0x3000, 960, 8 }, { 0x4000, 160, 1 },
                    { 0x5000, 160, 1 }, { 0x6000, 160, 1, true },
                    { 0x7000, 160, 1 } } },
    };

    if (!s->layout || !*s->layout) {
        /* the cameras' i2s.ko: AI, AO */
        s->nblk = 2;
        s->blk[0] = (XmI2sBlock){ .s = s, .base = AI_BLOCK, .chunk = s->chunk,
                                  .chans = 1, .codec = true,
                                  .irq = s->blk[0].irq,
                                  .timer = s->blk[0].timer };
        s->blk[1] = (XmI2sBlock){ .s = s, .base = AO_BLOCK, .chunk = s->chunk,
                                  .chans = 1, .codec = true,
                                  .irq = s->blk[1].irq,
                                  .timer = s->blk[1].timer };
        return true;
    }
    for (int l = 0; l < ARRAY_SIZE(layouts); l++) {
        if (strcmp(s->layout, layouts[l].name)) {
            continue;
        }
        s->nblk = 0;
        for (int n = 0; n < XM_AUDIO_BLOCKS && layouts[l].b[n].base; n++) {
            s->blk[n].base = layouts[l].b[n].base;
            s->blk[n].chunk = layouts[l].b[n].chunk;
            s->blk[n].chans = layouts[l].b[n].chans;
            s->blk[n].codec = layouts[l].b[n].codec;
            s->nblk++;
        }
        return true;
    }
    error_setg(errp, "xm-audio: unknown layout '%s'", s->layout);
    return false;
}

/* the backend pulls as it plays: what AO has written, else silence */
static void xm_audio_out_cb(void *opaque, int avail)
{
    XmAudioState *s = opaque;
    int16_t buf[256];

    while (avail >= 2) {
        const int n = MIN(avail / 2, ARRAY_SIZE(buf));
        size_t done;

        unsigned rd = s->play_rd;

        for (int i = 0; i < n; i++) {
            buf[i] = rd != s->play_wr ? s->play[rd++ % RING] : 0;
        }
        done = AUD_write(s->out, buf, n * 2);
        /* only what the backend took leaves the ring */
        s->play_rd += MIN(done / 2, s->play_wr - s->play_rd);
        if (done < n * 2) {
            break;
        }
        avail -= done;
    }
}

/* ...and pushes what it records, for AI to take */
static void xm_audio_in_cb(void *opaque, int avail)
{
    XmAudioState *s = opaque;
    int16_t buf[256];

    while (avail >= 2) {
        const size_t got = AUD_read(s->in, buf, MIN(avail, sizeof(buf)));

        if (!got) {
            break;
        }
        for (size_t i = 0; i < got / 2; i++) {
            s->rec[s->rec_wr++ % RING] = buf[i];
            if (s->rec_wr - s->rec_rd > RING) {
                s->rec_rd = s->rec_wr - RING;
            }
        }
        avail -= got;
    }
}

static void xm_audio_realize(DeviceState *dev, Error **errp)
{
    XmAudioState *s = XM_AUDIO(dev);
    struct audsettings as = {
        .freq = s->rate, .nchannels = 1, .fmt = AUDIO_FORMAT_S16,
    };

    if (!s->chunk || s->chunk > CHUNK_MAX || !s->rate) {
        error_setg(errp, "xm-audio: chunk must be 1..%d, rate nonzero",
                   CHUNK_MAX);
        return;
    }
    if (!xm_audio_layout(s, errp)) {
        return;
    }
    if (!s->audio_be) {
        return;
    }
    if (!AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    s->out = AUD_open_out(s->audio_be, NULL, "xm-audio.out", s,
                          xm_audio_out_cb, &as);
    AUD_set_active_out(s->out, true);
    if (s->mic) {
        s->in = AUD_open_in(s->audio_be, NULL, "xm-audio.in", s,
                            xm_audio_in_cb, &as);
        AUD_set_active_in(s->in, true);
    }
}

static const Property xm_audio_props[] = {
    DEFINE_AUDIO_PROPERTIES(XmAudioState, audio_be),
    DEFINE_PROP_BOOL("mic", XmAudioState, mic, false),
    DEFINE_PROP_UINT32("rate", XmAudioState, rate, 8000),
    DEFINE_PROP_UINT32("tone", XmAudioState, tone, 1000),
    DEFINE_PROP_UINT32("chunk", XmAudioState, chunk, 256),
    DEFINE_PROP_BOOL("trace", XmAudioState, trace, false),
    DEFINE_PROP_STRING("layout", XmAudioState, layout),
};

static void xm_audio_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = xm_audio_realize;
    device_class_set_legacy_reset(DEVICE_CLASS(klass), xm_audio_reset);
    device_class_set_props(DEVICE_CLASS(klass), xm_audio_props);
}

static const TypeInfo xm_audio_info = {
    .name          = TYPE_XM_AUDIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmAudioState),
    .instance_init = xm_audio_init,
    .class_init    = xm_audio_class_init,
};

static void xm_audio_register_types(void)
{
    type_register_static(&xm_audio_info);
}

type_init(xm_audio_register_types)
