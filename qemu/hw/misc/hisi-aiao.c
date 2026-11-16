/*
 * HiSilicon V4 AIAO: the audio interface's DMA engines on the EV200-class
 * dies (Hi3516EV200 / EV300, Hi3518EV300, Hi3516DV200 and the Goke
 * GK7205V200 / V300, GK7202V300, GK7201V200 rebrands), 64 KiB at
 * 0x100e0000, one interrupt (SPI 42, "AIO Interrupt").  aio.ko's HAL
 * (AIP_HAL_* for capture, AOP_HAL_* for playback) programs it; ai.ko and
 * ao.ko take the interrupts.  The codec next door (acodec.ko, 0x100f0000)
 * only has to read back, and stays a register bank.
 *
 *   +0x0000  interrupt enable: bit n capture channel n, bit 16 + n
 *            playback channel n
 *   +0x0004  interrupt status, the same bits: a channel whose own status
 *            has an unmasked bit
 *   +0x0100 + 8n, +0x0104 + 8n   capture channel n's clocks
 *   +0x0140 + 8n, +0x0144 + 8n   playback channel n's clocks
 *            first word: bits 0-26 the master clock's NCO increment over
 *            1800 MHz / 2^27; second: bits 0-3 the bit clock's divider
 *            (code 5 / 7 / 9 is 8 / 16 / 32), bits 4-6 bit clocks a frame
 *            (code 1 is 32)
 *   +0x1000 + 0x100n  capture channel n, +0x2000 + 0x100n  playback:
 *     +0x00  interface: bits 2-3 sample width (0 8-bit, 1 16-bit, 2 24-bit
 *            in 32), bits 4-5 and 7 the channel count (bit 7 clear: 1, or
 *            2 when bits 4-5 are 1; set: 2 << bits 4-5)
 *     +0x04  bit 28 start, bit 29 stopped (read only)
 *     +0x7c  buffer address, high word; +0x80 low word
 *     +0x84  buffer size, +0x88 write pointer, +0x8c read pointer,
 *            +0x94 bytes an interrupt (24 bits each)
 *     +0xa0  interrupt mask; +0xa4 raw status; +0xa8 masked status;
 *            +0xac write 1 to clear
 *            capture: bit 0 a transfer is in, bit 1 the buffer is full
 *            (nothing was written); playback: bit 0 a transfer went out,
 *            bit 1 the buffer ran empty
 *
 * Measured on a GK7201V200 running majestic (8 kHz mono, 160 samples a
 * frame): NCO 0x95218 with dividers 0xC119, a 1024-byte buffer, 320 bytes
 * a transfer, an interrupt every 20 ms.  The hardware moves its pointer
 * in 32-byte steps; this model moves it a transfer at a time, every
 * transfer / byte rate of virtual time, which is what the drivers count.
 *
 * Capture is a sine at the "tone" property's frequency (1 kHz) in every
 * channel.  With an audiodev (-global hisi-aiao.audiodev=ID) playback
 * channel 0 goes to it, and with mic=on capture channel 0 comes from it.
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
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qemu/audio.h"
#include <math.h>

#define TYPE_HISI_AIAO "hisi-aiao"
OBJECT_DECLARE_SIMPLE_TYPE(HisiAiaoState, HISI_AIAO)

#define AIAO_SIZE       0x10000
#define AIAO_INT_ENA    0x0000
#define AIAO_INT_STA    0x0004
#define AIAO_RX_BASE    0x1000
#define AIAO_TX_BASE    0x2000
#define AIAO_RX_CRG     0x0100
#define AIAO_TX_CRG     0x0140
#define AIAO_RX_CHNS    2
#define AIAO_TX_CHNS    3
#define AIAO_CHNS       (AIAO_RX_CHNS + AIAO_TX_CHNS)

#define CH_ATTR         0x00
#define CH_CTRL         0x04
#define CH_ADDR_HI      0x7c
#define CH_ADDR         0x80
#define CH_SIZE         0x84
#define CH_WPTR         0x88
#define CH_RPTR         0x8c
#define CH_TRANS        0x94
#define CH_INT_MASK     0xa0
#define CH_INT_RAW      0xa4
#define CH_INT_STA      0xa8
#define CH_INT_CLR      0xac

#define CTRL_START      (1u << 28)
#define CTRL_STOPPED    (1u << 29)
#define INT_TRANS       (1u << 0)
#define INT_FULL        (1u << 1)     /* capture: full; playback: empty */

#define PTR_MASK        0xffffff
#define RING            16384         /* audiodev buffering, a power of 2 */
#define XFER_MAX        65536

typedef struct HisiAiaoState HisiAiaoState;

typedef struct HisiAiaoChn {
    HisiAiaoState *s;
    hwaddr base;                /* +0x1000 / +0x2000 block */
    hwaddr crg;                 /* its clock words */
    int index;                  /* 0..1 capture, 0..2 playback */
    bool tx;
    uint32_t int_bit;           /* in AIAO_INT_ENA / AIAO_INT_STA */
    QEMUTimer *timer;
    uint32_t rate;              /* frames a second, at start */
    uint32_t frame_bytes;
    uint64_t frames;            /* capture frames generated */
    int64_t due;                /* virtual time of the next transfer */
    bool stalled;               /* due, but no room in the buffer */
} HisiAiaoChn;

struct HisiAiaoState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[AIAO_SIZE / 4];
    HisiAiaoChn chn[AIAO_CHNS];
    uint32_t tone;
    bool trace;
    bool mic;
    AudioBackend *audio_be;
    SWVoiceOut *out;
    SWVoiceIn *in;
    uint32_t voice_rate;
    /* between the backend's callbacks and the transfers, in samples */
    int16_t play[RING], rec[RING];
    unsigned play_rd, play_wr, rec_rd, rec_wr;
};

#define REG(s, off)         ((s)->regs[(off) / 4])
#define CREG(c, off)        REG((c)->s, (c)->base + (off))

static void hisi_aiao_update(HisiAiaoState *s)
{
    uint32_t sta = 0;

    for (int n = 0; n < AIAO_CHNS; n++) {
        HisiAiaoChn *c = &s->chn[n];

        if (CREG(c, CH_INT_RAW) & CREG(c, CH_INT_MASK)) {
            sta |= c->int_bit;
        }
    }
    qemu_set_irq(s->irq, (sta & REG(s, AIAO_INT_ENA)) != 0);
}

/* sample bytes and channels of a frame, from the interface word */
static uint32_t hisi_aiao_frame_bytes(HisiAiaoChn *c, int *chans, int *width)
{
    const uint32_t a = CREG(c, CH_ATTR);
    static const int widths[4] = { 1, 2, 4, 2 };
    const int f = extract32(a, 4, 2);
    const int ch = (a & 0x80) ? 2 << f : (f == 1 ? 2 : 1);
    const int w = widths[extract32(a, 2, 2)];

    if (chans) {
        *chans = ch;
    }
    if (width) {
        *width = w;
    }
    return ch * w;
}

/* frames a second, from the channel's clock words */
static uint32_t hisi_aiao_rate(HisiAiaoChn *c)
{
    static const uint32_t bclk_div[16] = {
        1, 3, 2, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256,
    };
    static const uint32_t fs_bits[8] = { 16, 32, 48, 64, 128, 256, 8, 8 };
    const uint32_t nco = extract32(REG(c->s, c->crg), 0, 27);
    const uint32_t div = REG(c->s, c->crg + 4);
    const uint64_t mclk = (uint64_t)nco * 1800000000ull >> 27;
    const uint64_t fs = mclk / bclk_div[extract32(div, 0, 4)] /
                        fs_bits[extract32(div, 4, 3)];

    if (fs < 1000 || fs > 192000) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-aiao: %s %d clocks give %" PRIu64 " Hz, using "
                      "8000\n", c->tx ? "playback" : "capture", c->index, fs);
        return 8000;
    }
    /* the NCO's rounding: 7999.99 is 8000 */
    return (uint32_t)((fs + 50) / 100 * 100);
}

static void hisi_aiao_voice(HisiAiaoState *s, uint32_t rate);

static int64_t hisi_aiao_period(HisiAiaoChn *c)
{
    const uint32_t trans = CREG(c, CH_TRANS) & PTR_MASK;

    return MAX(NANOSECONDS_PER_SECOND * trans /
               ((uint64_t)c->rate * c->frame_bytes), 1000);
}

static void hisi_aiao_arm(HisiAiaoChn *c)
{
    if (!(CREG(c, CH_CTRL) & CTRL_START) ||
        !(CREG(c, CH_TRANS) & PTR_MASK) || !c->rate) {
        timer_del(c->timer);
        return;
    }
    if (c->stalled) {
        /* until the driver makes room, which also brings us here */
        timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  hisi_aiao_period(c) / 4);
    } else {
        timer_mod(c->timer, c->due);
    }
}

static hwaddr hisi_aiao_buf(HisiAiaoChn *c)
{
    return ((hwaddr)CREG(c, CH_ADDR_HI) << 32) | CREG(c, CH_ADDR);
}

static int16_t hisi_aiao_sample(HisiAiaoChn *c, uint64_t frame, bool *live)
{
    HisiAiaoState *s = c->s;

    if (s->in && c->index == 0) {
        *live = true;
        if (s->rec_rd != s->rec_wr) {
            return s->rec[s->rec_rd++ % RING];
        }
        return 0;
    }
    return (int16_t)lrint(16000.0 *
                          sin(2 * M_PI * s->tone * (double)frame / c->rate));
}

/*
 * One transfer into a capture channel's buffer: false when there is no
 * room for it.  The read pointer is the driver's, so no room means the
 * driver has not taken what came before.
 */
static bool hisi_aiao_capture(HisiAiaoChn *c)
{
    const uint32_t size = CREG(c, CH_SIZE) & PTR_MASK;
    const uint32_t trans = CREG(c, CH_TRANS) & PTR_MASK;
    const uint32_t wptr = CREG(c, CH_WPTR) & PTR_MASK;
    const uint32_t rptr = CREG(c, CH_RPTR) & PTR_MASK;
    g_autofree uint8_t *buf = NULL;
    int chans, width;
    uint32_t fb, room;

    if (!size || trans > size || trans > XFER_MAX || wptr >= size) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-aiao: capture %d: size 0x%x "
                      "transfer 0x%x write pointer 0x%x\n", c->index, size,
                      trans, wptr);
        return true;
    }
    /* the write pointer never catches up with the read pointer */
    room = (rptr + size - wptr - 1) % size;
    if (room < trans) {
        return false;
    }
    fb = hisi_aiao_frame_bytes(c, &chans, &width);
    buf = g_malloc0(trans);
    for (uint32_t off = 0; off + fb <= trans; off += fb) {
        bool live = false;
        const int16_t v = hisi_aiao_sample(c, c->frames++, &live);

        for (int ch = 0; ch < chans; ch++) {
            uint8_t *p = buf + off + ch * width;

            switch (width) {
            case 1:
                p[0] = (uint8_t)(v >> 8);
                break;
            case 2:
                stw_le_p(p, v);
                break;
            default:
                stl_le_p(p, (uint32_t)(int32_t)v << 16);
                break;
            }
        }
    }
    for (uint32_t done = 0; done < trans;) {
        const uint32_t pos = (wptr + done) % size;
        const uint32_t n = MIN(trans - done, size - pos);

        dma_memory_write(&address_space_memory, hisi_aiao_buf(c) + pos,
                         buf + done, n, MEMTXATTRS_UNSPECIFIED);
        done += n;
    }
    CREG(c, CH_WPTR) = (CREG(c, CH_WPTR) & ~PTR_MASK) | (wptr + trans) % size;
    CREG(c, CH_INT_RAW) |= INT_TRANS;
    return true;
}

/* one transfer out of a playback channel's buffer */
static bool hisi_aiao_playback(HisiAiaoChn *c)
{
    HisiAiaoState *s = c->s;
    const uint32_t size = CREG(c, CH_SIZE) & PTR_MASK;
    const uint32_t trans = CREG(c, CH_TRANS) & PTR_MASK;
    const uint32_t wptr = CREG(c, CH_WPTR) & PTR_MASK;
    const uint32_t rptr = CREG(c, CH_RPTR) & PTR_MASK;
    g_autofree uint8_t *buf = NULL;
    uint32_t avail, n, bits = 0;
    int chans, width;
    uint32_t fb;

    if (!size || trans > size || trans > XFER_MAX || rptr >= size) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-aiao: playback %d: size 0x%x "
                      "transfer 0x%x read pointer 0x%x\n", c->index, size,
                      trans, rptr);
        return true;
    }
    avail = (wptr + size - rptr) % size;
    n = MIN(avail, trans);
    fb = hisi_aiao_frame_bytes(c, &chans, &width);
    n -= n % fb;
    if (n) {
        buf = g_malloc(n);
        for (uint32_t done = 0; done < n;) {
            const uint32_t pos = (rptr + done) % size;
            const uint32_t k = MIN(n - done, size - pos);

            dma_memory_read(&address_space_memory, hisi_aiao_buf(c) + pos,
                            buf + done, k, MEMTXATTRS_UNSPECIFIED);
            done += k;
        }
        /* the first channel of each frame, as 16-bit */
        for (uint32_t off = 0; s->out && c->index == 0 && off < n;
             off += fb) {
            int16_t v = width == 1 ? (int16_t)(buf[off] << 8) :
                        width == 2 ? (int16_t)lduw_le_p(buf + off) :
                        (int16_t)(ldl_le_p(buf + off) >> 16);

            if (s->play_wr - s->play_rd < RING) {
                s->play[s->play_wr++ % RING] = v;
            }
        }
        CREG(c, CH_RPTR) = (CREG(c, CH_RPTR) & ~PTR_MASK) |
                           (rptr + n) % size;
        bits |= INT_TRANS;
    }
    if (avail < trans) {
        bits |= INT_FULL;
    }
    CREG(c, CH_INT_RAW) |= bits;
    return true;
}

/*
 * Every transfer that is due.  The transfers keep to their own schedule
 * (due), not to when the timer got to run: the main loop is also where the
 * video encoders work, and a timer can run tens of milliseconds late on a
 * busy guest.  The guest's clock kept running all that time, and a real
 * AIAO would have delivered those transfers on time, so they are made
 * whenever the device next gets a look in -- the timer, or the driver
 * reading a pointer or a status, or moving its pointer -- as far as the
 * buffer holds; the rest follow as the driver makes room.  A second
 * behind, a capture channel gives up on the backlog and says its buffer
 * was full, as the hardware does when nobody reads.
 */
static void hisi_aiao_sync(HisiAiaoChn *c)
{
    const int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const int64_t period = hisi_aiao_period(c);

    if (!(CREG(c, CH_CTRL) & CTRL_START) || !c->rate) {
        return;
    }
    c->stalled = false;
    while (c->due <= now) {
        if (!(c->tx ? hisi_aiao_playback(c) : hisi_aiao_capture(c))) {
            if (now - c->due > NANOSECONDS_PER_SECOND) {
                CREG(c, CH_INT_RAW) |= INT_FULL;
                c->due = now + period;
            } else {
                c->stalled = true;
            }
            break;
        }
        if (c->s->trace) {
            qemu_log("hisi-aiao: %s %d transfer %" PRId64 " us late\n",
                     c->tx ? "playback" : "capture", c->index,
                     (now - c->due) / 1000);
        }
        c->due += period;
    }
    hisi_aiao_update(c->s);
}

static void hisi_aiao_tick(void *opaque)
{
    HisiAiaoChn *c = opaque;

    hisi_aiao_sync(c);
    hisi_aiao_arm(c);
}

static void hisi_aiao_start(HisiAiaoChn *c, bool on)
{
    const bool was = CREG(c, CH_CTRL) & CTRL_START;

    if (on && !was) {
        c->rate = hisi_aiao_rate(c);
        c->frame_bytes = hisi_aiao_frame_bytes(c, NULL, NULL);
        c->frames = 0;
        c->due = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 hisi_aiao_period(c);
        c->stalled = false;
        if (c->index == 0) {
            hisi_aiao_voice(c->s, c->rate);
        }
        if (c->s->trace) {
            qemu_log("hisi-aiao: %s %d starts at %u Hz, %u bytes a frame, "
                     "%u a transfer, buffer %u\n",
                     c->tx ? "playback" : "capture", c->index, c->rate,
                     c->frame_bytes, CREG(c, CH_TRANS) & PTR_MASK,
                     CREG(c, CH_SIZE) & PTR_MASK);
        }
    }
}

static HisiAiaoChn *hisi_aiao_chn(HisiAiaoState *s, hwaddr off)
{
    for (int n = 0; n < AIAO_CHNS; n++) {
        if (off >= s->chn[n].base && off < s->chn[n].base + 0x100) {
            return &s->chn[n];
        }
    }
    return NULL;
}

static uint64_t hisi_aiao_read(void *opaque, hwaddr off, unsigned size)
{
    HisiAiaoState *s = HISI_AIAO(opaque);
    HisiAiaoChn *c = hisi_aiao_chn(s, off & ~3);
    uint32_t v;

    if (c) {
        switch ((off & ~3) - c->base) {
        case CH_WPTR:
        case CH_RPTR:
        case CH_INT_RAW:
        case CH_INT_STA:
            hisi_aiao_sync(c);
            hisi_aiao_arm(c);
            break;
        }
    } else if ((off & ~3) == AIAO_INT_STA) {
        for (int n = 0; n < AIAO_CHNS; n++) {
            hisi_aiao_sync(&s->chn[n]);
            hisi_aiao_arm(&s->chn[n]);
        }
    }
    v = s->regs[off / 4];
    if (c) {
        switch ((off & ~3) - c->base) {
        case CH_CTRL:
            v &= ~CTRL_STOPPED;
            if (!(v & CTRL_START)) {
                v |= CTRL_STOPPED;
            }
            break;
        case CH_INT_STA:
            v = CREG(c, CH_INT_RAW) & CREG(c, CH_INT_MASK);
            break;
        case CH_INT_CLR:
            v = 0;
            break;
        }
    } else if ((off & ~3) == AIAO_INT_STA) {
        v = 0;
        for (int n = 0; n < AIAO_CHNS; n++) {
            if (CREG(&s->chn[n], CH_INT_RAW) &
                CREG(&s->chn[n], CH_INT_MASK)) {
                v |= s->chn[n].int_bit;
            }
        }
    }
    return extract32(v, (off & 3) * 8, size * 8);
}

static void hisi_aiao_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    HisiAiaoState *s = HISI_AIAO(opaque);
    const hwaddr word = off & ~3;
    HisiAiaoChn *c = hisi_aiao_chn(s, word);
    const uint32_t v = deposit32(s->regs[word / 4], (off & 3) * 8, size * 8,
                                 val);

    if (s->trace && !(c && (word - c->base == CH_INT_CLR ||
                            word - c->base == CH_RPTR ||
                            word - c->base == CH_WPTR))) {
        qemu_log("hisi-aiao: +0x%04" HWADDR_PRIx " = 0x%08x\n", word, v);
    }
    if (c) {
        switch (word - c->base) {
        case CH_INT_CLR:
            CREG(c, CH_INT_RAW) &= ~v;
            hisi_aiao_update(s);
            return;
        case CH_INT_STA:
            return;
        case CH_CTRL:
            hisi_aiao_start(c, v & CTRL_START);
            CREG(c, CH_CTRL) = v & ~CTRL_STOPPED;
            if (!(v & CTRL_START)) {
                timer_del(c->timer);
            } else if (!timer_pending(c->timer)) {
                hisi_aiao_arm(c);
            }
            return;
        case CH_INT_MASK:
            CREG(c, CH_INT_MASK) = v;
            hisi_aiao_update(s);
            return;
        }
    }
    s->regs[word / 4] = v;
    if (word == AIAO_INT_ENA) {
        hisi_aiao_update(s);
    }
    /* the driver took some capture, or gave some playback */
    if (c && word - c->base == (c->tx ? CH_WPTR : CH_RPTR)) {
        hisi_aiao_sync(c);
        hisi_aiao_arm(c);
    }
}

static const MemoryRegionOps hisi_aiao_ops = {
    .read = hisi_aiao_read,
    .write = hisi_aiao_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void hisi_aiao_reset(DeviceState *dev)
{
    HisiAiaoState *s = HISI_AIAO(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->play_rd = s->play_wr = s->rec_rd = s->rec_wr = 0;
    for (int n = 0; n < AIAO_CHNS; n++) {
        timer_del(s->chn[n].timer);
        s->chn[n].rate = 0;
        s->chn[n].frames = 0;
    }
    hisi_aiao_update(s);
}

static void hisi_aiao_init(Object *obj)
{
    HisiAiaoState *s = HISI_AIAO(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_aiao_ops, s, TYPE_HISI_AIAO,
                          AIAO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    for (int n = 0; n < AIAO_CHNS; n++) {
        HisiAiaoChn *c = &s->chn[n];

        c->s = s;
        c->tx = n >= AIAO_RX_CHNS;
        c->index = c->tx ? n - AIAO_RX_CHNS : n;
        c->base = (c->tx ? AIAO_TX_BASE : AIAO_RX_BASE) + 0x100 * c->index;
        c->crg = (c->tx ? AIAO_TX_CRG : AIAO_RX_CRG) + 8 * c->index;
        c->int_bit = c->tx ? 1u << (16 + c->index) : 1u << c->index;
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hisi_aiao_tick, c);
    }
}

/* the backend pulls as it plays: what playback 0 sent, else silence */
static void hisi_aiao_out_cb(void *opaque, int avail)
{
    HisiAiaoState *s = opaque;
    int16_t buf[256];

    while (avail >= 2) {
        const int n = MIN(avail / 2, ARRAY_SIZE(buf));
        unsigned rd = s->play_rd;
        size_t done;

        for (int i = 0; i < n; i++) {
            buf[i] = rd != s->play_wr ? s->play[rd++ % RING] : 0;
        }
        done = AUD_write(s->out, buf, n * 2);
        s->play_rd += MIN(done / 2, s->play_wr - s->play_rd);
        if (done < n * 2) {
            break;
        }
        avail -= done;
    }
}

/* ...and pushes what it records, for capture 0 to take */
static void hisi_aiao_in_cb(void *opaque, int avail)
{
    HisiAiaoState *s = opaque;
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

/* the audiodev follows the rate channel 0 last started at */
static void hisi_aiao_voice(HisiAiaoState *s, uint32_t rate)
{
    struct audsettings as = {
        .freq = rate, .nchannels = 1, .fmt = AUDIO_FORMAT_S16,
    };

    if (!s->audio_be || rate == s->voice_rate) {
        return;
    }
    s->voice_rate = rate;
    s->out = AUD_open_out(s->audio_be, s->out, "hisi-aiao.out", s,
                          hisi_aiao_out_cb, &as);
    AUD_set_active_out(s->out, true);
    if (s->mic) {
        s->in = AUD_open_in(s->audio_be, s->in, "hisi-aiao.in", s,
                            hisi_aiao_in_cb, &as);
        AUD_set_active_in(s->in, true);
    }
}

static void hisi_aiao_realize(DeviceState *dev, Error **errp)
{
    HisiAiaoState *s = HISI_AIAO(dev);

    if (!s->audio_be) {
        return;
    }
    if (!AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    hisi_aiao_voice(s, 8000);
}

static const Property hisi_aiao_props[] = {
    DEFINE_AUDIO_PROPERTIES(HisiAiaoState, audio_be),
    DEFINE_PROP_BOOL("mic", HisiAiaoState, mic, false),
    DEFINE_PROP_UINT32("tone", HisiAiaoState, tone, 1000),
    DEFINE_PROP_BOOL("trace", HisiAiaoState, trace, false),
};

static void hisi_aiao_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->realize = hisi_aiao_realize;
    device_class_set_legacy_reset(DEVICE_CLASS(klass), hisi_aiao_reset);
    device_class_set_props(DEVICE_CLASS(klass), hisi_aiao_props);
}

static const TypeInfo hisi_aiao_info = {
    .name          = TYPE_HISI_AIAO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiAiaoState),
    .instance_init = hisi_aiao_init,
    .class_init    = hisi_aiao_class_init,
};

static void hisi_aiao_register_types(void)
{
    type_register_static(&hisi_aiao_info);
}

type_init(hisi_aiao_register_types)
