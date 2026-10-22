/*
 * Novatek NA51089 (NT9856x) audio: the DAI (digital audio interface,
 * the DMA) and the EAC (embedded audio codec).
 *
 * DAI (kdrv_builtin/audio/dai/include/dai_reg.h, as the closed dai.o and
 * Audio.o drive it):
 *
 *   0x00 CTRL      bit 0 DAIEN; DMA enables bit 4 TX1, 5 TX2, 6 RX, 7 TXLB;
 *                  channel enables bit 12 TX1, 13 TX2, 14 RX, 15 TXLB
 *   0x10 / 0x14 / 0x18  formats TX1 (TX2 at +16) / RX / TXLB: [1:0] mode
 *                  (0 dual mono, 1 stereo, 2 left, 3 right), [9:8] sample
 *                  (0 8-bit, 1 16-bit, 2 32-bit), [11] mono in DRAM
 *   0x20 / 0x28    interrupt enable / status (write 1 to clear): buffer
 *                  done bit 0 TX1, 1 TX2, 2 RX1, 3 RX2; DMA loaded bit 4
 *                  TX1, 5 TX2, 6 RX; stopped bit 8 TX1, 9 TX2, 10 RX
 *   0x24 / 0x2C    the same for TXLB, at bits 0 / 4 / 8
 *   0x40..0x50     buffer addresses TX1, TX2, RX1, RX2, TXLB
 *   0x60..0x70     their sizes, in 32-bit words [18:0]
 *   0x80..0x90     where each DMA is now (read only)
 *
 * The address and size registers queue the next buffer: enabling a DMA
 * takes them as its buffer (DMA loaded), and at each buffer's end the DMA
 * raises buffer done and takes whatever they hold then -- the driver has
 * queued the following buffer from its interrupt, or the same buffer goes
 * round again.  A channel's enable falling stops it and raises stopped,
 * which the driver waits on before it turns the DMA off.
 *
 * The sample rate is in neither block: the clock tree gives the DAI 256 x
 * fs (nvt_cg_audio_rate).  The DMAs move samples at that rate on the
 * virtual clock, whatever the host audio does:
 *
 *   - RX captures a test tone (1 kHz left, 1.5 kHz right, half scale;
 *     "tone-hz", 0 for silence), or with rx-source=audiodev the host's
 *     input through the audiodev, silence where it runs short;
 *   - TX1 plays to the audiodev's output, if there is one.  TX2 and TXLB
 *     move their buffers but are not heard.
 *
 * EAC (kdrv_builtin/audio/eac/include/eac_reg.h): register storage, with
 * the two strobes the driver waits on clearing themselves -- CTRL1 bit 0
 * LOAD, EADC_CTRL0 bit 13 DC cancel load -- and what shapes the signal:
 * CTRL0 bit 0 / 1 ADC / DAC enable, the ADC digital gain (0x20 [23:16] /
 * [31:24], 0 = mute), the DAC's L / R averaging (0xA4 bit 2) and digital
 * gain (0xAC [7:0] / [15:8], 0xFF 0 dB in 0.5 dB steps, below 0x4B mute).
 * The analog gains, ALC and filters are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/misc/nvt-cg.h"
#include "migration/vmstate.h"
#include "qemu/audio.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"
#include <math.h>

#define TYPE_NVT_AUDIO "nvt-audio"
OBJECT_DECLARE_SIMPLE_TYPE(NvtAudioState, NVT_AUDIO)

#define DAI_SIZE            0x100
#define EAC_SIZE            0x300

#define DAI_CTRL            0x00
#define DAI_CTRL_EN         (1u << 0)
#define DAI_FMT_TX          0x10
#define DAI_FMT_RX          0x14
#define DAI_FMT_TXLB        0x18
#define DAI_INTEN           0x20
#define DAI_INTEN2          0x24
#define DAI_INTSTS          0x28
#define DAI_INTSTS2         0x2C
#define DAI_ADDR            0x40
#define DAI_SIZE_REG        0x60
#define DAI_CUR             0x80

#define EAC_CTRL0           0x00
#define EAC_CTRL1           0x04
#define EAC_EADC_CTRL0      0x10
#define EAC_ADC_GAIN        0x20
#define EAC_DAC_CTRL0       0xA4
#define EAC_DAC_GAIN        0xAC
#define EAC_AD_EN           (1u << 0)
#define EAC_DA_EN           (1u << 1)
#define EAC_LOAD            (1u << 0)
#define EAC_DCCAN_LOAD      (1u << 13)
#define EAC_DATA_MIX        (1u << 2)

/* the DMA channels, in register order */
enum { CH_TX1, CH_TX2, CH_RX1, CH_RX2, CH_TXLB, CH_NUM };

#define TICK_NS             (5 * SCALE_MS)
#define RING_BYTES          (48000 * 4 / 4)     /* 250 ms of 48k stereo */

typedef struct {
    uint32_t addr;              /* the buffer it is in */
    uint32_t bytes;
    uint32_t pos;               /* bytes moved in it */
    bool loaded;                /* DMA on, a buffer taken */
} NvtDaiChan;

/* a byte ring for audio between the DMA and a host voice */
typedef struct {
    uint8_t data[RING_BYTES];
    uint32_t head, len;
} NvtAudioRing;

struct NvtAudioState {
    SysBusDevice parent_obj;

    MemoryRegion dai_mr;
    MemoryRegion eac_mr;
    qemu_irq irq;
    uint32_t dai[DAI_SIZE / 4];
    uint32_t eac[EAC_SIZE / 4];
    NvtDaiChan ch[CH_NUM];
    QEMUTimer *timer;
    int64_t last_ns;            /* the DMAs are up to here */
    uint64_t frac;              /* frames x 1e9 not yet moved */
    uint64_t tone_n;            /* the tone's sample count */
    unsigned rate;

    DeviceState *cg;
    AudioBackend *audio_be;
    uint32_t tone_hz;
    char *rx_source;
    SWVoiceIn *vin;
    SWVoiceOut *vout;
    unsigned voice_rate;
    NvtAudioRing in;            /* host input, for RX */
    NvtAudioRing out;           /* TX1, for the host */
};

/* the channel enable bit, and where its DMA enable is */
static const uint32_t ch_en[CH_NUM] = {
    1u << 12, 1u << 13, 1u << 14, 1u << 14, 1u << 15,
};
static const uint32_t ch_dma[CH_NUM] = {
    1u << 4, 1u << 5, 1u << 6, 1u << 6, 1u << 7,
};

static uint32_t *dai(NvtAudioState *s, hwaddr off)
{
    return &s->dai[off / 4];
}

static uint32_t *eac(NvtAudioState *s, hwaddr off)
{
    return &s->eac[off / 4];
}

static void nvt_audio_update_irq(NvtAudioState *s)
{
    qemu_set_irq(s->irq,
                 (*dai(s, DAI_INTSTS) & *dai(s, DAI_INTEN)) ||
                 (*dai(s, DAI_INTSTS2) & *dai(s, DAI_INTEN2)));
}

/* raise a channel's event: buffer done 0, loaded 4, stopped 8 */
static void ch_event(NvtAudioState *s, int c, unsigned what)
{
    static const unsigned bit[CH_NUM] = { 0, 1, 2, 3, 0 };
    unsigned b = bit[c];

    if (what && (c == CH_RX1 || c == CH_RX2)) {
        b = 2;                          /* RX has one load / stop bit */
    }
    if (c == CH_TXLB) {
        *dai(s, DAI_INTSTS2) |= 1u << what;
    } else {
        *dai(s, DAI_INTSTS) |= 1u << (b + what);
    }
}

static uint32_t ch_fmt(NvtAudioState *s, int c)
{
    switch (c) {
    case CH_TX1:
        return *dai(s, DAI_FMT_TX) & 0xFFFF;
    case CH_TX2:
        return *dai(s, DAI_FMT_TX) >> 16;
    case CH_TXLB:
        return *dai(s, DAI_FMT_TXLB);
    default:
        return *dai(s, DAI_FMT_RX);
    }
}

static bool rx_dual_mono(NvtAudioState *s)
{
    return (*dai(s, DAI_FMT_RX) & 3) == 0;
}

/* bytes of one sample, and of a frame in DRAM */
static unsigned ch_sample_bytes(NvtAudioState *s, int c)
{
    static const unsigned sz[4] = { 1, 2, 4, 2 };

    return sz[(ch_fmt(s, c) >> 8) & 3];
}

static unsigned ch_frame_bytes(NvtAudioState *s, int c)
{
    bool mono = (ch_fmt(s, c) & (1u << 11)) ||
                ((c == CH_RX1 || c == CH_RX2) && rx_dual_mono(s));

    return ch_sample_bytes(s, c) * (mono ? 1 : 2);
}

static bool ch_running(NvtAudioState *s, int c)
{
    uint32_t ctrl = *dai(s, DAI_CTRL);

    if (c == CH_RX2 && !rx_dual_mono(s)) {
        return false;
    }
    return s->ch[c].loaded && (ctrl & DAI_CTRL_EN) && (ctrl & ch_en[c]) &&
           (ctrl & ch_dma[c]);
}

/* take the queued buffer */
static void ch_load(NvtAudioState *s, int c)
{
    NvtDaiChan *ch = &s->ch[c];

    ch->addr = *dai(s, DAI_ADDR + 4 * c);
    ch->bytes = (*dai(s, DAI_SIZE_REG + 4 * c) & 0x7FFFF) * 4;
    ch->pos = 0;
    ch->loaded = true;
}

/* ---- the signal ---- */

/* the next stereo frame captured, as s16 */
static void rx_frame(NvtAudioState *s, int16_t lr[2])
{
    uint32_t gain = *eac(s, EAC_ADC_GAIN);

    lr[0] = lr[1] = 0;
    if (s->rx_source && !strcmp(s->rx_source, "audiodev")) {
        if (s->in.len >= 4) {
            uint8_t b[4];
            unsigned i;

            for (i = 0; i < 4; i++) {
                b[i] = s->in.data[(s->in.head + i) % RING_BYTES];
            }
            s->in.head = (s->in.head + 4) % RING_BYTES;
            s->in.len -= 4;
            lr[0] = lduw_le_p(b);
            lr[1] = lduw_le_p(b + 2);
        }
    } else if (s->tone_hz) {
        double t = (double)s->tone_n / s->rate;

        lr[0] = 16384 * sin(2 * G_PI * s->tone_hz * t);
        lr[1] = 16384 * sin(2 * G_PI * s->tone_hz * 1.5 * t);
    }
    s->tone_n++;
    if (!(*eac(s, EAC_CTRL0) & EAC_AD_EN)) {
        lr[0] = lr[1] = 0;
    }
    if (!((gain >> 16) & 0xFF)) {
        lr[0] = 0;
    }
    if (!(gain >> 24)) {
        lr[1] = 0;
    }
}

static void put_sample(uint8_t *p, int16_t v, unsigned bytes)
{
    if (bytes == 1) {
        *p = v >> 8;
    } else if (bytes == 2) {
        stw_le_p(p, v);
    } else {
        stl_le_p(p, (uint32_t)(int32_t)v << 16);
    }
}

static int16_t get_sample(const uint8_t *p, unsigned bytes)
{
    if (bytes == 1) {
        return (int8_t)*p << 8;
    } else if (bytes == 2) {
        return lduw_le_p(p);
    }
    return ldl_le_p(p) >> 16;
}

/* one frame of a TX1 buffer to the host, through the EAC's DAC path */
static void tx_frame(NvtAudioState *s, int16_t l, int16_t r)
{
    uint32_t gain = *eac(s, EAC_DAC_GAIN);
    int32_t v[2] = { l, r };
    unsigned i, j;

    if (!s->vout) {
        return;
    }
    if (*eac(s, EAC_DAC_CTRL0) & EAC_DATA_MIX) {
        v[0] = v[1] = (v[0] + v[1]) / 2;
    }
    for (i = 0; i < 2; i++) {
        unsigned g = (gain >> (8 * i)) & 0xFF;

        if (!(*eac(s, EAC_CTRL0) & EAC_DA_EN) || g < 0x4B) {
            v[i] = 0;
        } else {
            v[i] = v[i] * pow(10, (g - 255.0) * 0.5 / 20);
        }
    }
    if (s->out.len + 4 > RING_BYTES) {
        /* the host is slower: drop the oldest */
        s->out.head = (s->out.head + 4) % RING_BYTES;
        s->out.len -= 4;
    }
    for (i = 0; i < 2; i++) {
        uint16_t u = v[i];

        for (j = 0; j < 2; j++) {
            s->out.data[(s->out.head + s->out.len) % RING_BYTES] = u >> (8 * j);
            s->out.len++;
        }
    }
}

/*
 * Move n frames on channel c, crossing into the next buffer as it fills
 * one.  RX writes captured frames (both RX buffers in dual mono get
 * theirs from the same frames: c == CH_RX1 makes them); TX reads.
 */
static void ch_move(NvtAudioState *s, int c, const int16_t (*rx)[2],
                    uint64_t n)
{
    NvtDaiChan *ch = &s->ch[c];
    unsigned fb = ch_frame_bytes(s, c), sb = ch_sample_bytes(s, c);
    uint32_t mode = ch_fmt(s, c) & 3;
    uint8_t buf[8];
    uint64_t i;

    if (!ch->bytes || ch->bytes < fb) {
        return;
    }
    for (i = 0; i < n; i++) {
        hwaddr at = ch->addr + ch->pos;

        if (c == CH_RX1 || c == CH_RX2) {
            int16_t l = rx[i][0], r = rx[i][1];

            if (fb == sb) {             /* mono: one channel's sample */
                bool right = mode == 3 || (mode == 0 && c == CH_RX2);

                put_sample(buf, right ? r : l, sb);
            } else {
                put_sample(buf, mode == 3 ? r : l, sb);
                put_sample(buf + sb, mode == 2 ? l : r, sb);
            }
            dma_memory_write(&address_space_memory, at, buf, fb,
                             MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_read(&address_space_memory, at, buf, fb,
                            MEMTXATTRS_UNSPECIFIED);
            if (c == CH_TX1) {
                int16_t l = get_sample(buf, sb);
                int16_t r = fb == sb ? l : get_sample(buf + sb, sb);

                tx_frame(s, mode == 3 ? r : l, mode == 2 ? l : r);
            }
        }
        ch->pos += fb;
        if (ch->pos + fb > ch->bytes) {
            ch_event(s, c, 0);          /* buffer done */
            ch_load(s, c);              /* and the next, if queued */
            ch_event(s, c, 4);
            if (!ch->bytes || ch->bytes < fb) {
                return;
            }
        }
    }
}

/* bring the DMAs up to now */
static void nvt_audio_advance(NvtAudioState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t n, i;
    bool any = false;
    int c;

    for (c = 0; c < CH_NUM; c++) {
        any |= ch_running(s, c);
    }
    if (!any || now <= s->last_ns) {
        s->last_ns = now;
        return;
    }
    s->frac += (uint64_t)(now - s->last_ns) * s->rate;
    s->last_ns = now;
    n = s->frac / NANOSECONDS_PER_SECOND;
    s->frac %= NANOSECONDS_PER_SECOND;
    while (n) {
        uint64_t k = MIN(n, 256);
        int16_t rx[256][2];

        if (ch_running(s, CH_RX1) || ch_running(s, CH_RX2)) {
            for (i = 0; i < k; i++) {
                rx_frame(s, rx[i]);
            }
        }
        for (c = 0; c < CH_NUM; c++) {
            if (ch_running(s, c)) {
                ch_move(s, c, (const int16_t (*)[2])rx, k);
            }
        }
        n -= k;
    }
    nvt_audio_update_irq(s);
}

/*
 * The next tick: the first buffer end, so its interrupt comes on time,
 * or a tick's worth of samples later.
 */
static void nvt_audio_schedule(NvtAudioState *s)
{
    int64_t next = INT64_MAX;
    int c;

    for (c = 0; c < CH_NUM; c++) {
        /* a buffer too small for a frame never ends: nothing to time */
        if (ch_running(s, c) &&
            s->ch[c].bytes >= ch_frame_bytes(s, c)) {
            unsigned fb = ch_frame_bytes(s, c);
            uint64_t left = s->ch[c].bytes > s->ch[c].pos ?
                            (s->ch[c].bytes - s->ch[c].pos) / fb : 1;
            int64_t ns = DIV_ROUND_UP(MAX(left, 1) * NANOSECONDS_PER_SECOND -
                                      MIN(s->frac, left *
                                          NANOSECONDS_PER_SECOND), s->rate);

            next = MIN(next, s->last_ns + MIN(ns, TICK_NS));
        }
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
}

static void nvt_audio_voices(NvtAudioState *s);

/*
 * The clock tree changed under a running stream: the samples so far went
 * at the old rate, the rest go at the new one.
 */
static void nvt_audio_rate(NvtAudioState *s)
{
    unsigned rate = s->cg ? nvt_cg_audio_rate(s->cg) : 48000;

    if (rate != s->rate) {
        s->rate = rate;
        s->frac = 0;
        nvt_audio_voices(s);
    }
}

static void nvt_audio_tick(void *opaque)
{
    NvtAudioState *s = opaque;

    nvt_audio_advance(s);
    nvt_audio_rate(s);
    nvt_audio_schedule(s);
}

/* ---- the host's voices ---- */

static void nvt_audio_in_cb(void *opaque, int avail)
{
    NvtAudioState *s = opaque;
    uint8_t buf[1024];

    while (avail > 0) {
        size_t n = AUD_read(s->vin, buf, MIN(sizeof(buf), (size_t)avail));
        size_t i;

        if (!n) {
            break;
        }
        avail -= n;
        for (i = 0; i < n; i++) {
            if (s->in.len == RING_BYTES) {
                s->in.head = (s->in.head + 1) % RING_BYTES;
                s->in.len--;
            }
            s->in.data[(s->in.head + s->in.len) % RING_BYTES] = buf[i];
            s->in.len++;
        }
    }
}

static void nvt_audio_out_cb(void *opaque, int avail)
{
    NvtAudioState *s = opaque;

    while (avail > 0 && s->out.len) {
        uint32_t n = MIN(MIN((uint32_t)avail, s->out.len),
                         RING_BYTES - s->out.head);
        size_t done = AUD_write(s->vout, s->out.data + s->out.head, n);

        if (!done) {
            break;
        }
        s->out.head = (s->out.head + done) % RING_BYTES;
        s->out.len -= done;
        avail -= done;
    }
}

/* open (or close) the voices for what is running, at the current rate */
static void nvt_audio_voices(NvtAudioState *s)
{
    struct audsettings as = {
        .freq = s->rate, .nchannels = 2, .fmt = AUDIO_FORMAT_S16,
        .endianness = 0,
    };
    bool rx = (*dai(s, DAI_CTRL) & ch_en[CH_RX1]) && s->rx_source &&
              !strcmp(s->rx_source, "audiodev");
    bool tx = *dai(s, DAI_CTRL) & ch_en[CH_TX1];

    if (!s->audio_be) {
        return;
    }
    if (s->voice_rate != s->rate) {
        if (s->vin) {
            AUD_close_in(s->audio_be, s->vin);
            s->vin = NULL;
        }
        if (s->vout) {
            AUD_close_out(s->audio_be, s->vout);
            s->vout = NULL;
        }
        s->voice_rate = s->rate;
    }
    if (rx && !s->vin) {
        s->vin = AUD_open_in(s->audio_be, NULL, "nvt-audio.in", s,
                             nvt_audio_in_cb, &as);
    }
    if (tx && !s->vout) {
        s->vout = AUD_open_out(s->audio_be, NULL, "nvt-audio.out", s,
                               nvt_audio_out_cb, &as);
    }
    if (s->vin) {
        AUD_set_active_in(s->vin, rx);
    }
    if (s->vout) {
        AUD_set_active_out(s->vout, tx);
    }
}

/* ---- registers ---- */

static uint64_t nvt_dai_read(void *opaque, hwaddr off, unsigned size)
{
    NvtAudioState *s = opaque;

    nvt_audio_advance(s);
    if (off >= DAI_CUR && off < DAI_CUR + 4 * CH_NUM) {
        int c = (off - DAI_CUR) / 4;

        return s->ch[c].loaded ? s->ch[c].addr + s->ch[c].pos :
               *dai(s, DAI_ADDR + 4 * c);
    }
    return *dai(s, off);
}

static void nvt_dai_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtAudioState *s = opaque;
    uint32_t old, on, gone;
    int c;

    nvt_audio_advance(s);
    switch (off) {
    case DAI_CTRL:
        old = *dai(s, off);
        *dai(s, off) = val;
        on = val & ~old;
        gone = old & ~val;
        if (on & ch_en[CH_RX1]) {
            s->tone_n = 0;
        }
        /* a stream's queued samples go with it */
        if ((gone | on) & ch_en[CH_RX1]) {
            s->in.head = s->in.len = 0;
        }
        if ((gone | on) & ch_en[CH_TX1]) {
            s->out.head = s->out.len = 0;
        }
        if (on & (ch_en[CH_TX1] | ch_en[CH_TX2] | ch_en[CH_RX1] |
                  ch_en[CH_TXLB] | DAI_CTRL_EN)) {
            s->rate = s->cg ? nvt_cg_audio_rate(s->cg) : 48000;
            s->frac = 0;
        }
        for (c = 0; c < CH_NUM; c++) {
            if (c == CH_RX2 && !rx_dual_mono(s)) {
                continue;
            }
            if (on & ch_dma[c]) {
                ch_load(s, c);
                ch_event(s, c, 4);      /* loaded */
            }
            if ((gone & ch_en[c]) || (gone & ch_dma[c])) {
                if (c != CH_RX2) {
                    ch_event(s, c, 8);  /* stopped */
                }
            }
            if (gone & ch_dma[c]) {
                s->ch[c].loaded = false;
            }
        }
        nvt_audio_voices(s);
        break;
    case DAI_INTSTS:
    case DAI_INTSTS2:
        *dai(s, off) &= ~val;
        break;
    default:
        if (off >= DAI_CUR && off < DAI_CUR + 4 * CH_NUM) {
            break;                      /* read only */
        }
        *dai(s, off) = val;
        break;
    }
    nvt_audio_update_irq(s);
    nvt_audio_schedule(s);
}

static uint64_t nvt_eac_read(void *opaque, hwaddr off, unsigned size)
{
    NvtAudioState *s = opaque;

    return *eac(s, off);
}

static void nvt_eac_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtAudioState *s = opaque;

    nvt_audio_advance(s);
    if (off == EAC_CTRL1) {
        val &= ~EAC_LOAD;
    } else if (off == EAC_EADC_CTRL0) {
        val &= ~EAC_DCCAN_LOAD;
    }
    *eac(s, off) = val;
}

static const MemoryRegionOps nvt_dai_ops = {
    .read = nvt_dai_read,
    .write = nvt_dai_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps nvt_eac_ops = {
    .read = nvt_eac_read,
    .write = nvt_eac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_audio_reset(DeviceState *dev)
{
    NvtAudioState *s = NVT_AUDIO(dev);

    timer_del(s->timer);
    memset(s->dai, 0, sizeof(s->dai));
    memset(s->eac, 0, sizeof(s->eac));
    memset(s->ch, 0, sizeof(s->ch));
    s->frac = 0;
    s->tone_n = 0;
    s->rate = 48000;
    s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->in.head = s->in.len = 0;
    s->out.head = s->out.len = 0;
    nvt_audio_voices(s);
    qemu_irq_lower(s->irq);
}

static void nvt_audio_realize(DeviceState *dev, Error **errp)
{
    NvtAudioState *s = NVT_AUDIO(dev);

    if (s->rx_source && strcmp(s->rx_source, "tone") &&
        strcmp(s->rx_source, "audiodev")) {
        error_setg(errp, "nvt-audio: rx-source is tone or audiodev");
        return;
    }
    /* the host's audio only when the machine was given an audiodev */
    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    memory_region_init_io(&s->dai_mr, OBJECT(dev), &nvt_dai_ops, s,
                          "nvt-dai", DAI_SIZE);
    memory_region_init_io(&s->eac_mr, OBJECT(dev), &nvt_eac_ops, s,
                          "nvt-eac", EAC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->dai_mr);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->eac_mr);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_audio_tick, s);
    s->rate = 48000;
}

static void nvt_audio_unrealize(DeviceState *dev)
{
    NvtAudioState *s = NVT_AUDIO(dev);

    if (s->vin) {
        AUD_close_in(s->audio_be, s->vin);
    }
    if (s->vout) {
        AUD_close_out(s->audio_be, s->vout);
    }
    timer_free(s->timer);
}

static int nvt_audio_post_load(void *opaque, int version_id)
{
    NvtAudioState *s = opaque;

    nvt_audio_voices(s);
    return 0;
}

static const VMStateDescription vmstate_nvt_dai_chan = {
    .name = "nvt-dai-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(addr, NvtDaiChan),
        VMSTATE_UINT32(bytes, NvtDaiChan),
        VMSTATE_UINT32(pos, NvtDaiChan),
        VMSTATE_BOOL(loaded, NvtDaiChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_nvt_audio_ring = {
    .name = "nvt-audio-ring",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, NvtAudioRing, RING_BYTES),
        VMSTATE_UINT32(head, NvtAudioRing),
        VMSTATE_UINT32(len, NvtAudioRing),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_nvt_audio = {
    .name = TYPE_NVT_AUDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = nvt_audio_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(dai, NvtAudioState, DAI_SIZE / 4),
        VMSTATE_UINT32_ARRAY(eac, NvtAudioState, EAC_SIZE / 4),
        VMSTATE_STRUCT_ARRAY(ch, NvtAudioState, CH_NUM, 1,
                             vmstate_nvt_dai_chan, NvtDaiChan),
        VMSTATE_TIMER_PTR(timer, NvtAudioState),
        VMSTATE_INT64(last_ns, NvtAudioState),
        VMSTATE_UINT64(frac, NvtAudioState),
        VMSTATE_UINT64(tone_n, NvtAudioState),
        VMSTATE_UINT32(rate, NvtAudioState),
        VMSTATE_STRUCT(in, NvtAudioState, 1, vmstate_nvt_audio_ring,
                       NvtAudioRing),
        VMSTATE_STRUCT(out, NvtAudioState, 1, vmstate_nvt_audio_ring,
                       NvtAudioRing),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_audio_properties[] = {
    DEFINE_AUDIO_PROPERTIES(NvtAudioState, audio_be),
    DEFINE_PROP_LINK("cg", NvtAudioState, cg, TYPE_DEVICE, DeviceState *),
    DEFINE_PROP_UINT32("tone-hz", NvtAudioState, tone_hz, 1000),
    DEFINE_PROP_STRING("rx-source", NvtAudioState, rx_source),
};

static void nvt_audio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_audio_realize;
    dc->unrealize = nvt_audio_unrealize;
    device_class_set_legacy_reset(dc, nvt_audio_reset);
    device_class_set_props(dc, nvt_audio_properties);
    dc->vmsd = &vmstate_nvt_audio;
}

static const TypeInfo nvt_audio_info = {
    .name = TYPE_NVT_AUDIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtAudioState),
    .class_init = nvt_audio_class_init,
};

static void nvt_audio_register_types(void)
{
    type_register_static(&nvt_audio_info);
}

type_init(nvt_audio_register_types)
