/*
 * Rockchip SAI ("rockchip,sai-v1", version 0x24031103 on the RV1103B,
 * 0x20660000), as the vendor kernel's sound/soc/rockchip/rockchip_sai.c
 * drives it:
 *
 *   0x00 TXCR / 0x08 RXCR   bits 4:0 VDW - 1 (valid data width), 9:5 SBW
 *                           - 1 (slot width), 17:11 SNB - 1 (slots a
 *                           lane), 21:20 CSR - 1 (lanes)
 *   0x04 FSCR    bits 11:0 FW - 1 (bit clocks a frame)
 *   0x10 XFER    bit 1 FSS (frame sync), 2 TXS, 3 RXS (the streams)
 *   0x14 CLR     write 1s to clear; they self-clear
 *   0x18 CKR     bits 14:3 MDIV - 1 (mclk / bclk), 2 slave
 *   0x24 DMACR   bit 8 TDE, 24 RDE
 *   0x30 TXDR / 0x34 RXDR   the FIFOs, one 32-bit word a sample
 *   0x6c STATUS  bits 1..3 FS / TX / RX idle   0x70 VERSION
 *
 * Its samples move through the RKDMA (rk-dmac), which paces the blocks
 * to and from TXDR / RXDR at this SAI's byte rate (the "tx-byte-rate" /
 * "rx-byte-rate" properties): the frame rate mclk / MDIV / FW, times the
 * channels (SNB x CSR), times a sample's bytes in memory (VDW 8, 16 or
 * more: 1, 2, 4).  The mclk is the TOPCRU's mclk_sai_src
 * (clk-rv1103b.c): CLKSEL_CON(35) bits 11:10 pick clk_sai_src (GPLL /
 * (CLKSEL_CON(20) bits 4:0 + 1)), clk_sai_frac (that times CLKSEL_CON(21)
 * numerator 31:16 / denominator 15:0) or xin_osc0_half (12 MHz).
 *
 * Capture reads "rx-file" (a 16-bit PCM WAV, looped; a mono file feeds
 * every channel), else silence; playback is saved to "tx-file" as 16-bit
 * PCM at the stream's rate and channels.  Each sample is VDW bits,
 * right-justified; up to 16 bits, two share a FIFO word (low half
 * first), as the 32-bit DMA moves them to and from an S16 buffer -- the
 * RV1103B XM App captures 8 kHz stereo S16 at 8000 words a second.  Without them the sound card's streams run on time
 * and carry silence.  With "audiodev" playback goes to a QEMU audio
 * backend as well and capture comes from it (before the file), through a
 * ring that takes up the difference between the guest's virtual pace and
 * the backend's real one.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qemu/notify.h"
#include "system/system.h"
#include "qom/object.h"
#include "qemu/audio.h"

#define TYPE_RK_SAI "rk-sai"
OBJECT_DECLARE_SIMPLE_TYPE(RkSaiState, RK_SAI)

#define REGS_SIZE   0x1000
#define NREGS       (0x110 / 4)

#define R_TXCR      0x00
#define R_FSCR      0x04
#define R_RXCR      0x08
#define R_XFER      0x10
#define R_CLR       0x14
#define R_CKR       0x18
#define R_TXDR      0x30
#define R_RXDR      0x34
#define R_STATUS    0x6c
#define R_VERSION   0x70

#define SAI_VERSION_V   0x24031103
#define STATUS_IDLE     0xf         /* FS, TX, RX idle */
#define XFER_TXS        (1u << 2)

#define OSC_HALF_HZ     12000000
#define SAI_RING        16384   /* samples between guest and backend */

struct RkSaiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[NREGS];
    uint64_t cru_base;          /* property: the CRU (TOPCRU at +0x60000) */
    uint64_t rx_words;          /* words read since the stream started */

    char *rx_path, *tx_path;
    int16_t *rx_pcm;
    uint32_t rx_frames, rx_ch;
    FILE *tx_fp;
    uint32_t tx_bytes;
    int64_t tx_hdr_ns;
    Notifier exit;

    AudioBackend *audio_be;     /* property "audiodev" */
    SWVoiceOut *vout;
    SWVoiceIn *vin;
    uint32_t out_fs, out_ch, in_fs, in_ch;
    int16_t aring[2][SAI_RING];     /* 0 playback, 1 capture */
    uint32_t ahead[2], acount[2];
};

static void rk_sai_ring_put(RkSaiState *s, int r, int16_t v)
{
    if (s->acount[r] == SAI_RING) {             /* drop the oldest */
        s->ahead[r] = (s->ahead[r] + 1) % SAI_RING;
        s->acount[r]--;
    }
    s->aring[r][(s->ahead[r] + s->acount[r]) % SAI_RING] = v;
    s->acount[r]++;
}

static void rk_sai_out_cb(void *opaque, int avail)
{
    RkSaiState *s = opaque;

    while (avail >= 2 && s->acount[0]) {
        uint32_t n = MIN(s->acount[0], SAI_RING - s->ahead[0]);
        size_t done;

        n = MIN(n, (uint32_t)avail / 2);
        done = AUD_write(s->vout, &s->aring[0][s->ahead[0]], n * 2);
        if (!done) {
            break;
        }
        s->ahead[0] = (s->ahead[0] + done / 2) % SAI_RING;
        s->acount[0] -= done / 2;
        avail -= done;
    }
}

static void rk_sai_in_cb(void *opaque, int avail)
{
    RkSaiState *s = opaque;
    int16_t buf[512];

    while (avail >= 2) {
        size_t got = AUD_read(s->vin, buf, MIN((size_t)avail, sizeof(buf)));
        size_t i;

        if (!got) {
            break;
        }
        for (i = 0; i < got / 2; i++) {
            rk_sai_ring_put(s, 1, buf[i]);
        }
        avail -= got;
    }
}

static uint32_t rk_sai_fs(RkSaiState *s);
static uint32_t rk_sai_channels(RkSaiState *s, bool tx);

/* the voice for the stream's rate and channels, (re)opened to match */
static void rk_sai_voice(RkSaiState *s, bool in)
{
    uint32_t fs = rk_sai_fs(s), ch = rk_sai_channels(s, !in);
    struct audsettings as = {
        .freq = fs, .nchannels = ch, .fmt = AUDIO_FORMAT_S16, .endianness = 0,
    };

    if (!s->audio_be || !fs || !ch) {
        return;
    }
    if (in) {
        if (s->in_fs == fs && s->in_ch == ch) {
            return;                 /* open, or tried and failed */
        }
        if (s->vin) {
            AUD_close_in(s->audio_be, s->vin);
        }
        s->vin = AUD_open_in(s->audio_be, NULL, "rk-sai.in", s,
                             rk_sai_in_cb, &as);
        s->in_fs = fs;
        s->in_ch = ch;
        s->acount[1] = 0;
        if (s->vin) {
            AUD_set_active_in(s->vin, true);
        }
    } else {
        if (s->out_fs == fs && s->out_ch == ch) {
            return;
        }
        if (s->vout) {
            AUD_close_out(s->audio_be, s->vout);
        }
        s->vout = AUD_open_out(s->audio_be, NULL, "rk-sai.out", s,
                               rk_sai_out_cb, &as);
        s->out_fs = fs;
        s->out_ch = ch;
        s->acount[0] = 0;
        if (s->vout) {
            AUD_set_active_out(s->vout, true);
        }
    }
}

#define REG(s, off) ((s)->regs[(off) / 4])

static uint32_t rk_sai_cru(RkSaiState *s, hwaddr off)
{
    return address_space_ldl_le(&address_space_memory, s->cru_base + off,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/* GPLL (pll_rk3328 at TOPCRU 0x60, the 24 MHz oscillator in) */
static uint64_t rk_sai_gpll_hz(RkSaiState *s)
{
    uint32_t c0 = rk_sai_cru(s, 0x60060), c1 = rk_sai_cru(s, 0x60064);
    uint64_t fb = c0 & 0xfff, p1 = (c0 >> 12) & 7, ref = c1 & 0x3f;
    uint64_t p2 = (c1 >> 6) & 7;

    if (!fb || !p1 || !ref || !p2) {
        return 1188000000;
    }
    return 24000000ull * fb / ref / p1 / p2;
}

/* mclk_sai_src, in Hz */
static uint64_t rk_sai_mclk_hz(RkSaiState *s)
{
    uint32_t sel = (rk_sai_cru(s, 0x60300 + 35 * 4) >> 10) & 3;
    uint64_t src = rk_sai_gpll_hz(s) /
                   ((rk_sai_cru(s, 0x60300 + 20 * 4) & 0x1f) + 1);
    uint32_t frac = rk_sai_cru(s, 0x60300 + 21 * 4);

    switch (sel) {
    case 0:
        return src;
    case 1:
        return (frac & 0xffff) ? src * (frac >> 16) / (frac & 0xffff) : src;
    default:
        return OSC_HALF_HZ;     /* from the pin: taken as the oscillator */
    }
}

static uint32_t rk_sai_channels(RkSaiState *s, bool tx)
{
    uint32_t cr = REG(s, tx ? R_TXCR : R_RXCR);

    return (((cr >> 11) & 0x7f) + 1) * (((cr >> 20) & 3) + 1);
}

/* frames a second */
static uint32_t rk_sai_fs(RkSaiState *s)
{
    uint64_t mdiv = ((REG(s, R_CKR) >> 3) & 0xfff) + 1;
    uint64_t fw = (REG(s, R_FSCR) & 0xfff) + 1;

    return rk_sai_mclk_hz(s) / mdiv / fw;
}

/*
 * The DMA's bytes a second on the memory side: a sample takes 1, 2 or 4
 * bytes there (S8, S16, S24 / S32), whatever the FIFO word
 */
static uint32_t rk_sai_byte_rate(RkSaiState *s, bool tx)
{
    int vdw = (REG(s, tx ? R_TXCR : R_RXCR) & 0x1f) + 1;
    uint32_t bytes = vdw <= 8 ? 1 : vdw <= 16 ? 2 : 4;

    return rk_sai_fs(s) * rk_sai_channels(s, tx) * bytes;
}

static void rk_sai_get_rate(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    uint32_t val = rk_sai_byte_rate(RK_SAI(obj), opaque != NULL);

    visit_type_uint32(v, name, &val, errp);
}

/* sample @k of the capture: the file's, for its channel, VDW bits */
static int32_t rk_sai_rx_sample(RkSaiState *s, uint64_t k)
{
    uint32_t ch = rk_sai_channels(s, false);
    int vdw = (REG(s, R_RXCR) & 0x1f) + 1;
    int32_t v;

    if (s->audio_be) {
        rk_sai_voice(s, true);
    }
    if (s->vin) {
        v = 0;                      /* the backend's, silence while none */
        if (s->acount[1]) {
            v = s->aring[1][s->ahead[1]];
            s->ahead[1] = (s->ahead[1] + 1) % SAI_RING;
            s->acount[1]--;
        }
        return vdw > 16 ? v * (1 << (vdw - 16)) : v >> (16 - vdw);
    }
    if (!s->rx_frames) {
        return 0;
    }
    v = s->rx_pcm[((k / ch) % s->rx_frames) * s->rx_ch + (k % ch) % s->rx_ch];
    if (vdw > 16) {
        v *= 1 << (vdw - 16);
    } else {
        v >>= 16 - vdw;
    }
    return v;
}

/*
 * The next captured FIFO word.  Up to 16 bits a sample (S8 / S16) two
 * share a word, the earlier in the low half: the DMA moves 32-bit words
 * into the S16 buffer as they are.  Wider samples take a word each.
 */
static uint32_t rk_sai_rx_word(RkSaiState *s)
{
    int vdw = (REG(s, R_RXCR) & 0x1f) + 1;
    uint32_t lo, hi;

    if (vdw > 16) {
        return (uint32_t)rk_sai_rx_sample(s, s->rx_words++) &
               (vdw >= 32 ? ~0u : (1u << vdw) - 1);
    }
    lo = (uint16_t)rk_sai_rx_sample(s, s->rx_words++);
    hi = (uint16_t)rk_sai_rx_sample(s, s->rx_words++);
    return lo | hi << 16;
}

static void rk_sai_wav_header(RkSaiState *s)
{
    uint32_t ch = rk_sai_channels(s, true), fs = MAX(rk_sai_fs(s), 1);
    uint8_t h[44];

    memcpy(h, "RIFF\0\0\0\0WAVEfmt ", 16);
    stl_le_p(h + 16, 16);
    stw_le_p(h + 20, 1);
    stw_le_p(h + 22, ch);
    stl_le_p(h + 24, fs);
    stl_le_p(h + 28, fs * ch * 2);
    stw_le_p(h + 32, ch * 2);
    stw_le_p(h + 34, 16);
    memcpy(h + 36, "data", 4);
    stl_le_p(h + 40, s->tx_bytes);
    stl_le_p(h + 4, 36 + s->tx_bytes);
    fseek(s->tx_fp, 0, SEEK_SET);
    fwrite(h, 1, sizeof(h), s->tx_fp);
    fseek(s->tx_fp, 0, SEEK_END);
}

static void rk_sai_tx_sync(RkSaiState *s)
{
    if (s->tx_fp) {
        rk_sai_wav_header(s);
        fflush(s->tx_fp);
        s->tx_hdr_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

/* a played word: its VDW-bit sample, as 16 bits, into the WAV */
static void rk_sai_tx_word(RkSaiState *s, uint32_t w)
{
    int vdw = (REG(s, R_TXCR) & 0x1f) + 1;
    int16_t smp;

    if (s->audio_be) {
        rk_sai_voice(s, false);
        if (s->vout) {
            if (vdw <= 16) {
                rk_sai_ring_put(s, 0, (int16_t)(w << (16 - vdw)));
                rk_sai_ring_put(s, 0, (int16_t)((w >> 16) << (16 - vdw)));
            } else {
                rk_sai_ring_put(s, 0,
                                (int32_t)(w << (32 - vdw)) >> 16);
            }
        }
    }
    if (!s->tx_path) {
        return;
    }
    if (!s->tx_fp) {
        s->tx_fp = fopen(s->tx_path, "wb+");
        s->tx_bytes = 0;
        if (!s->tx_fp) {
            return;
        }
        rk_sai_wav_header(s);
    }
    if (vdw <= 16) {                /* two samples a word, low first */
        int16_t two[2] = { cpu_to_le16((int16_t)w),
                           cpu_to_le16((int16_t)(w >> 16)) };

        if (vdw < 16) {
            two[0] = cpu_to_le16((int16_t)(w << (16 - vdw)));
            two[1] = cpu_to_le16((int16_t)((w >> 16) << (16 - vdw)));
        }
        fwrite(two, 2, 2, s->tx_fp);
        s->tx_bytes += 4;
    } else {
        w <<= 32 - vdw;             /* the sign at bit 31 */
        smp = cpu_to_le16((int32_t)w >> 16);
        fwrite(&smp, 2, 1, s->tx_fp);
        s->tx_bytes += 2;
    }
    if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->tx_hdr_ns >=
        NANOSECONDS_PER_SECOND / 10) {
        rk_sai_tx_sync(s);
    }
}

static uint64_t rk_sai_read(void *opaque, hwaddr off, unsigned size)
{
    RkSaiState *s = opaque;

    switch (off) {
    case R_RXDR:
        return rk_sai_rx_word(s);
    case R_CLR:
        return 0;                   /* self-clearing */
    case R_STATUS:
        return STATUS_IDLE;
    case R_VERSION:
        return SAI_VERSION_V;
    }
    return off < NREGS * 4 ? s->regs[off / 4] : 0;
}

static void rk_sai_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkSaiState *s = opaque;
    uint32_t was;

    switch (off) {
    case R_TXDR:
        rk_sai_tx_word(s, val);
        return;
    case R_RXDR:
    case R_STATUS:
    case R_VERSION:
        return;
    case R_XFER:
        was = REG(s, R_XFER);
        if ((was & XFER_TXS) && !(val & XFER_TXS)) {
            rk_sai_tx_sync(s);      /* playback stopped: the file whole */
        }
        if ((val ^ was) & (1u << 3)) {
            s->rx_words = 0;        /* a capture starts from frame 0 */
        }
        break;
    }
    if (off < NREGS * 4) {
        s->regs[off / 4] = val;
    }
}

static const MemoryRegionOps rk_sai_ops = {
    .read = rk_sai_read,
    .write = rk_sai_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_sai_load_rx(RkSaiState *s, Error **errp)
{
    g_autofree gchar *buf = NULL;
    gsize len, off = 12;
    uint32_t ch = 0, bits = 0;

    if (!g_file_get_contents(s->rx_path, &buf, &len, NULL) || len < 12 ||
        memcmp(buf, "RIFF", 4) || memcmp(buf + 8, "WAVE", 4)) {
        error_setg(errp, "rx-file: '%s' is not a WAV file", s->rx_path);
        return;
    }
    while (off + 8 <= len) {
        uint32_t sz = ldl_le_p(buf + off + 4);

        if (!memcmp(buf + off, "fmt ", 4) && sz >= 16) {
            ch = lduw_le_p(buf + off + 10);
            bits = lduw_le_p(buf + off + 22);
        } else if (!memcmp(buf + off, "data", 4) && ch) {
            uint32_t i, n;

            if (bits != 16) {
                error_setg(errp, "rx-file: 16-bit PCM only, not %u", bits);
                return;
            }
            sz = MIN(sz, len - off - 8);
            n = sz / 2;
            s->rx_pcm = g_new(int16_t, MAX(n, 1));
            for (i = 0; i < n; i++) {
                s->rx_pcm[i] = lduw_le_p(buf + off + 8 + 2 * i);
            }
            s->rx_ch = ch;
            s->rx_frames = n / ch;
            return;
        }
        off += 8 + sz + (sz & 1);
    }
    error_setg(errp, "rx-file: '%s' has no 16-bit PCM data", s->rx_path);
}

static void rk_sai_exit(Notifier *n, void *data)
{
    RkSaiState *s = container_of(n, RkSaiState, exit);

    rk_sai_tx_sync(s);
}

static void rk_sai_realize(DeviceState *dev, Error **errp)
{
    RkSaiState *s = RK_SAI(dev);

    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    if (s->rx_path && *s->rx_path) {
        rk_sai_load_rx(s, errp);
    }
    s->exit.notify = rk_sai_exit;
    qemu_add_exit_notifier(&s->exit);
}

static void rk_sai_reset(DeviceState *dev)
{
    RkSaiState *s = RK_SAI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->rx_words = 0;
}

static void rk_sai_init(Object *obj)
{
    RkSaiState *s = RK_SAI(obj);

    memory_region_init_io(&s->iomem, obj, &rk_sai_ops, s, TYPE_RK_SAI,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    object_property_add(obj, "tx-byte-rate", "uint32", rk_sai_get_rate,
                        NULL, NULL, (void *)1);
    object_property_add(obj, "rx-byte-rate", "uint32", rk_sai_get_rate,
                        NULL, NULL, NULL);
}

static const Property rk_sai_properties[] = {
    DEFINE_PROP_UINT64("cru-base", RkSaiState, cru_base, 0x20000000),
    DEFINE_PROP_STRING("rx-file", RkSaiState, rx_path),
    DEFINE_PROP_STRING("tx-file", RkSaiState, tx_path),
    DEFINE_AUDIO_PROPERTIES(RkSaiState, audio_be),
};

static const VMStateDescription rk_sai_vmstate = {
    .name = TYPE_RK_SAI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkSaiState, NREGS),
        VMSTATE_UINT64(rx_words, RkSaiState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_sai_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_sai_realize;
    device_class_set_legacy_reset(dc, rk_sai_reset);
    device_class_set_props(dc, rk_sai_properties);
    dc->vmsd = &rk_sai_vmstate;
}

static const TypeInfo rk_sai_info = {
    .name = TYPE_RK_SAI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkSaiState),
    .instance_init = rk_sai_init,
    .class_init = rk_sai_class_init,
};

static void rk_sai_register_types(void)
{
    type_register_static(&rk_sai_info);
}

type_init(rk_sai_register_types)
