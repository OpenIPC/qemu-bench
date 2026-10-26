/*
 * Grain Media FTSSP010 in I2S mode, with the ADDA308 audio codec.
 *
 * The GM813x has two FTSSP010s, 0x91300000 and 0x91400000, which the
 * vendor audio_drv.ko (module/audio_drv/ssp_dma/ssp_dma.c) runs as I2S
 * slaves; SSP0's clocks come from the in-package ADDA308 codec, whose
 * wrapper sits at 0x90B00000 (module/adda308/adda308_drv.c).  SSP0's
 * instance maps both.
 *
 * SSP registers:
 *   0x00 CTRL0     13:12 I2S, bit 3 master, bit 2 stereo
 *   0x04 CTRL1     padding and sample width (I2S framing)
 *   0x08 CTRL2     0 enable, 1 TX output, 2 / 3 clear RX / TX FIFO, 6 reset
 *                  (those three pulse), 7 RX enable, 8 TX enable
 *   0x0C STAT      0 RX FIFO full, 1 TX FIFO not full, [8:4] RX level,
 *                  [16:12] TX level
 *   0x10 INTRCTRL  0 / 1 overrun / underrun interrupts, 4 / 5 RX / TX DMA,
 *                  [11:7] / [16:12] RX / TX thresholds
 *   0x14 INTRSTAT  clears when read
 *   0x18 DATA      the 16-entry FIFOs, one sample per entry
 *
 * The AHB DMA moves the samples on handshake lines (SSP0: RX 10, TX 9;
 * SSP1: RX 12, TX 11): "dma-rx" is high while RX DMA is on and the RX
 * FIFO holds its threshold, "dma-tx" while TX DMA is on and the TX FIFO
 * is at or below its threshold.  The driver fills the TX FIFO with zeros
 * until "not full" drops -- at init, with the SSP off -- so writes land in
 * it whatever is enabled.
 *
 * Samples run at the codec's rate: ADDA reg0 [7:4] codes the capture
 * rate, [11:8] the playback rate (3 8 kHz, 2 16 kHz, 8 22.05, 7 32,
 * 0xD 44.1, 0xF 48; line-out codes 4 8 kHz, 1 16 kHz, 6 48 kHz); SSP1
 * uses "rate".  Frames are stereo with CTRL0 bit 2, mono otherwise.
 * Capture is a tone ("tone-hz", 0 for silence) fed into the RX FIFO;
 * playback drains the TX FIFO to the machine's audiodev, if it has one.
 * The model keeps more than 16 samples behind the FIFO, so emulation
 * jitter never overruns it: overrun and underrun -- which make the driver
 * tear the stream down -- are never raised.  The codec's other registers
 * (gains, power, mutes) hold what is written.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/audio.h"
#include "qom/object.h"

#define TYPE_GM_SSP "gm-ssp"
OBJECT_DECLARE_SIMPLE_TYPE(GmSspState, GM_SSP)

#define GM_SSP_SIZE         0x1000
#define GM_ADDA_SIZE        0x1000
#define GM_ADDA_NREGS       (0x40 / 4)
#define GM_SSP_FIFO         16
#define GM_SSP_RING         2048        /* samples kept behind each FIFO */
#define GM_SSP_TICK_NS      1000000
#define GM_SSP_OUT_RING     8192

#define R_CTRL0             0x00
#define R_CTRL1             0x04
#define R_CTRL2             0x08
#define R_STAT              0x0C
#define R_INTRCTRL          0x10
#define R_INTRSTAT          0x14
#define R_DATA              0x18

#define CTRL0_STEREO        (1u << 2)
#define CTRL2_RXFCLR        (1u << 2)
#define CTRL2_TXFCLR        (1u << 3)
#define CTRL2_RESET         (1u << 6)
#define CTRL2_RXEN          (1u << 7)
#define CTRL2_TXEN          (1u << 8)
#define CTRL2_PULSES        (CTRL2_RXFCLR | CTRL2_TXFCLR | CTRL2_RESET)
#define ICR_RXDMA           (1u << 4)
#define ICR_TXDMA           (1u << 5)

typedef struct GmSspFifo {
    int16_t data[GM_SSP_RING];
    uint32_t head, count;
} GmSspFifo;

struct GmSspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion adda_mem;
    qemu_irq irq;
    qemu_irq dma_rx;
    qemu_irq dma_tx;
    QEMUTimer *timer;
    AudioBackend *audio_be;
    SWVoiceOut *vout;
    uint32_t voice_rate;
    int voice_ch;

    uint32_t ctrl0, ctrl1, ctrl2, intrctrl;
    uint32_t adda[GM_ADDA_NREGS];
    GmSspFifo rx, tx;
    int64_t last_ns;
    uint64_t rx_frac, tx_frac;  /* sample time carried over, ns * rate */
    uint64_t phase;             /* tone samples produced */

    int16_t out[GM_SSP_OUT_RING];
    uint32_t out_head, out_count;

    bool has_adda;
    uint32_t rate;
    uint32_t tone_hz;
};

/* ---- the sample clock ------------------------------------------------ */

static uint32_t gm_ssp_code_rate(uint32_t code)
{
    switch (code) {
    case 3: case 4:
        return 8000;
    case 2: case 1:
        return 16000;
    case 8:
        return 22050;
    case 7:
        return 32000;
    case 0xD:
        return 44100;
    case 0xF: case 6:
        return 48000;
    }
    return 8000;
}

static uint32_t gm_ssp_rx_rate(GmSspState *s)
{
    return s->has_adda ? gm_ssp_code_rate(extract32(s->adda[0], 4, 4))
                       : s->rate;
}

static uint32_t gm_ssp_tx_rate(GmSspState *s)
{
    return s->has_adda ? gm_ssp_code_rate(extract32(s->adda[0], 8, 4))
                       : s->rate;
}

static int gm_ssp_channels(GmSspState *s)
{
    return (s->ctrl0 & CTRL0_STEREO) ? 2 : 1;
}

static void gm_ssp_update(GmSspState *s)
{
    const uint32_t rx_th = MAX(extract32(s->intrctrl, 7, 5), 1);
    const uint32_t tx_th = extract32(s->intrctrl, 12, 5);
    const bool rx = (s->ctrl2 & CTRL2_RXEN) && (s->intrctrl & ICR_RXDMA) &&
                    s->rx.count >= rx_th;
    const bool tx = (s->ctrl2 & CTRL2_TXEN) && (s->intrctrl & ICR_TXDMA) &&
                    s->tx.count <= tx_th;

    qemu_set_irq(s->dma_rx, rx);
    qemu_set_irq(s->dma_tx, tx);
}

static void gm_ssp_push(GmSspFifo *f, int16_t v, uint32_t cap)
{
    if (f->count < cap) {
        f->data[(f->head + f->count) % GM_SSP_RING] = v;
        f->count++;
    }
}

static int16_t gm_ssp_pop(GmSspFifo *f)
{
    int16_t v = 0;

    if (f->count) {
        v = f->data[f->head];
        f->head = (f->head + 1) % GM_SSP_RING;
        f->count--;
    }
    return v;
}

/* ---- the host's output ----------------------------------------------- */

static void gm_ssp_out_cb(void *opaque, int avail)
{
    GmSspState *s = opaque;

    while (avail >= 2 && s->out_count) {
        const uint32_t n = MIN(MIN((uint32_t)avail / 2, s->out_count),
                               GM_SSP_OUT_RING - s->out_head);
        const size_t done = AUD_write(s->vout, &s->out[s->out_head], n * 2);

        if (!done) {
            break;
        }
        s->out_head = (s->out_head + done / 2) % GM_SSP_OUT_RING;
        s->out_count -= done / 2;
        avail -= done;
    }
}

static void gm_ssp_voice(GmSspState *s)
{
    const bool tx = s->ctrl2 & CTRL2_TXEN;
    const uint32_t rate = gm_ssp_tx_rate(s);
    const int ch = gm_ssp_channels(s);

    if (!s->audio_be) {
        return;
    }
    if (s->vout && (s->voice_rate != rate || s->voice_ch != ch)) {
        AUD_close_out(s->audio_be, s->vout);
        s->vout = NULL;
    }
    if (tx && !s->vout) {
        struct audsettings as = {
            .freq = rate, .nchannels = ch, .fmt = AUDIO_FORMAT_S16,
            .endianness = 0,
        };

        s->vout = AUD_open_out(s->audio_be, NULL, "gm-ssp.out", s,
                               gm_ssp_out_cb, &as);
        s->voice_rate = rate;
        s->voice_ch = ch;
    }
    if (s->vout) {
        AUD_set_active_out(s->vout, tx);
    }
}

/* ---- time passing ---------------------------------------------------- */

/* samples of the elapsed time at rate, keeping the remainder */
static uint32_t gm_ssp_samples(uint64_t *frac, int64_t dt, uint32_t rate,
                               int ch)
{
    uint64_t n;

    *frac += (uint64_t)dt * rate;
    n = *frac / NANOSECONDS_PER_SECOND;
    *frac %= NANOSECONDS_PER_SECOND;
    return n * ch;
}

static void gm_ssp_advance(GmSspState *s)
{
    const int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const int64_t dt = MIN(now - s->last_ns, 100 * GM_SSP_TICK_NS);
    const int ch = gm_ssp_channels(s);
    uint32_t n;

    s->last_ns = now;
    if (dt <= 0) {
        return;
    }
    if (s->ctrl2 & CTRL2_RXEN) {
        const uint32_t rate = gm_ssp_rx_rate(s);

        n = gm_ssp_samples(&s->rx_frac, dt, rate, ch);
        while (n--) {
            const double t = (double)(s->phase++ / ch) / rate;
            const int16_t v = s->tone_hz ?
                (int16_t)(8000 * sin(2 * M_PI * s->tone_hz * t)) : 0;

            gm_ssp_push(&s->rx, v, GM_SSP_RING);
        }
    }
    if (s->ctrl2 & CTRL2_TXEN) {
        n = gm_ssp_samples(&s->tx_frac, dt, gm_ssp_tx_rate(s), ch);
        while (n-- && s->tx.count) {
            const int16_t v = gm_ssp_pop(&s->tx);

            if (s->vout && s->out_count < GM_SSP_OUT_RING) {
                s->out[(s->out_head + s->out_count) % GM_SSP_OUT_RING] = v;
                s->out_count++;
            }
        }
    }
    gm_ssp_update(s);
}

static void gm_ssp_tick(void *opaque)
{
    GmSspState *s = opaque;

    gm_ssp_advance(s);
    if (s->ctrl2 & (CTRL2_RXEN | CTRL2_TXEN)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            GM_SSP_TICK_NS);
    }
}

/* ---- SSP registers --------------------------------------------------- */

static uint64_t gm_ssp_read(void *opaque, hwaddr off, unsigned size)
{
    GmSspState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_CTRL0:
        return s->ctrl0;
    case R_CTRL1:
        return s->ctrl1;
    case R_CTRL2:
        return s->ctrl2;
    case R_STAT:
        v = (s->rx.count >= GM_SSP_FIFO ? 1 : 0) |
            (s->tx.count < GM_SSP_FIFO ? 2 : 0) |
            (MIN(s->rx.count, GM_SSP_FIFO) << 4) |
            (MIN(s->tx.count, GM_SSP_FIFO) << 12);
        return v;
    case R_INTRCTRL:
        return s->intrctrl;
    case R_INTRSTAT:
        return 0;
    case R_DATA:
        v = (uint16_t)gm_ssp_pop(&s->rx);
        gm_ssp_update(s);
        return v;
    }
    return 0;
}

static void gm_ssp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    GmSspState *s = opaque;
    uint32_t was;

    switch (off) {
    case R_CTRL0:
        s->ctrl0 = val;
        break;
    case R_CTRL1:
        s->ctrl1 = val;
        break;
    case R_CTRL2:
        gm_ssp_advance(s);
        was = s->ctrl2;
        if (val & (CTRL2_RXFCLR | CTRL2_RESET)) {
            s->rx.head = s->rx.count = 0;
        }
        if (val & (CTRL2_TXFCLR | CTRL2_RESET)) {
            s->tx.head = s->tx.count = 0;
        }
        s->ctrl2 = val & ~CTRL2_PULSES;
        if ((s->ctrl2 & ~was) & (CTRL2_RXEN | CTRL2_TXEN)) {
            s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            timer_mod(s->timer, s->last_ns + GM_SSP_TICK_NS);
        }
        gm_ssp_voice(s);
        break;
    case R_INTRCTRL:
        s->intrctrl = val;
        break;
    case R_DATA:
        gm_ssp_push(&s->tx, val, GM_SSP_FIFO);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: write 0x%" HWADDR_PRIx "\n", __func__,
                      off);
        return;
    }
    gm_ssp_update(s);
}

static const MemoryRegionOps gm_ssp_ops = {
    .read = gm_ssp_read,
    .write = gm_ssp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- the ADDA308 wrapper --------------------------------------------- */

static uint64_t gm_adda_read(void *opaque, hwaddr off, unsigned size)
{
    GmSspState *s = opaque;

    return off < GM_ADDA_NREGS * 4 ? s->adda[off / 4] : 0;
}

static void gm_adda_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    GmSspState *s = opaque;

    if (off < GM_ADDA_NREGS * 4) {
        gm_ssp_advance(s);
        s->adda[off / 4] = val;
        gm_ssp_voice(s);                /* the rate may have changed */
    }
}

static const MemoryRegionOps gm_adda_ops = {
    .read = gm_adda_read,
    .write = gm_adda_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- the device ------------------------------------------------------ */

static void gm_ssp_reset(DeviceState *dev)
{
    GmSspState *s = GM_SSP(dev);

    timer_del(s->timer);
    s->ctrl0 = s->ctrl1 = s->ctrl2 = s->intrctrl = 0;
    memset(s->adda, 0, sizeof(s->adda));
    s->rx.head = s->rx.count = 0;
    s->tx.head = s->tx.count = 0;
    s->rx_frac = s->tx_frac = s->phase = 0;
    s->out_head = s->out_count = 0;
    s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    gm_ssp_voice(s);
    gm_ssp_update(s);
}

static void gm_ssp_realize(DeviceState *dev, Error **errp)
{
    GmSspState *s = GM_SSP(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->audio_be && !AUD_backend_check(&s->audio_be, errp)) {
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &gm_ssp_ops, s, TYPE_GM_SSP,
                          GM_SSP_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    if (s->has_adda) {
        memory_region_init_io(&s->adda_mem, OBJECT(s), &gm_adda_ops, s,
                              "gm-adda308", GM_ADDA_SIZE);
        sysbus_init_mmio(sbd, &s->adda_mem);
    }
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_rx, "dma-rx", 1);
    qdev_init_gpio_out_named(dev, &s->dma_tx, "dma-tx", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_ssp_tick, s);
}

static void gm_ssp_unrealize(DeviceState *dev)
{
    GmSspState *s = GM_SSP(dev);

    if (s->vout) {
        AUD_close_out(s->audio_be, s->vout);
    }
    timer_free(s->timer);
}

static int gm_ssp_post_load(void *opaque, int version_id)
{
    GmSspState *s = opaque;

    s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    gm_ssp_voice(s);
    if (s->ctrl2 & (CTRL2_RXEN | CTRL2_TXEN)) {
        timer_mod(s->timer, s->last_ns + GM_SSP_TICK_NS);
    }
    return 0;
}

static const Property gm_ssp_props[] = {
    DEFINE_AUDIO_PROPERTIES(GmSspState, audio_be),
    DEFINE_PROP_BOOL("adda", GmSspState, has_adda, false),
    DEFINE_PROP_UINT32("rate", GmSspState, rate, 8000),
    DEFINE_PROP_UINT32("tone-hz", GmSspState, tone_hz, 1000),
};

static const VMStateDescription vmstate_gm_ssp_fifo = {
    .name = "gm-ssp-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT16_ARRAY(data, GmSspFifo, GM_SSP_RING),
        VMSTATE_UINT32(head, GmSspFifo),
        VMSTATE_UINT32(count, GmSspFifo),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gm_ssp = {
    .name = TYPE_GM_SSP,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = gm_ssp_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl0, GmSspState),
        VMSTATE_UINT32(ctrl1, GmSspState),
        VMSTATE_UINT32(ctrl2, GmSspState),
        VMSTATE_UINT32(intrctrl, GmSspState),
        VMSTATE_UINT32_ARRAY(adda, GmSspState, GM_ADDA_NREGS),
        VMSTATE_STRUCT(rx, GmSspState, 1, vmstate_gm_ssp_fifo, GmSspFifo),
        VMSTATE_STRUCT(tx, GmSspState, 1, vmstate_gm_ssp_fifo, GmSspFifo),
        VMSTATE_UINT64(rx_frac, GmSspState),
        VMSTATE_UINT64(tx_frac, GmSspState),
        VMSTATE_UINT64(phase, GmSspState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_ssp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_ssp_realize;
    dc->unrealize = gm_ssp_unrealize;
    device_class_set_legacy_reset(dc, gm_ssp_reset);
    device_class_set_props(dc, gm_ssp_props);
    dc->vmsd = &vmstate_gm_ssp;
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo gm_ssp_info = {
    .name           = TYPE_GM_SSP,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmSspState),
    .class_init     = gm_ssp_class_init,
};

static void gm_ssp_register_types(void)
{
    type_register_static(&gm_ssp_info);
}

type_init(gm_ssp_register_types)
