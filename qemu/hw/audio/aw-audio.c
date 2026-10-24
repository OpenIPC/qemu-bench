/*
 * Allwinner audio capture FIFOs, V831/V833: the internal codec's ADC
 * (0x05096000, DRQ 6) and the digital audio interface DAUDIO0 (I2S,
 * 0x05090000, DRQ 3), as the vendor sound/soc/sunxi drivers program them.
 *
 * Nothing is connected to the microphones or the I2S lines, so capture
 * records silence -- but at the programmed rate: while the receiver is
 * on, a 1 ms tick puts the samples that interval holds into the FIFO, and
 * the DRQ is up while any are waiting and DRQ is enabled, so the DMA
 * moves them at the sample rate and ALSA gets its period interrupts.
 * The media application's AAC encoder and its recorder wait for audio.
 * Playback is accepted and dropped.
 *
 * codec:  0x30 ADC_FIFOC  bits 31:29 rate (48, 32, 24, 16, 12, 8 kHz),
 *                         28 ADC enable, 15:12 microphone mask,
 *                         3 DRQ enable, 0 flush
 *         0x38 ADC_FIFOS  bit 23 data available, 15:8 count
 *         0x40 ADC_RXDATA
 * daudio: 0x00 CTL        bit 0 global enable, 1 RX enable
 *         0x10 RXFIFO     0x14 FIFOCTL  bit 24 RX flush
 *         0x18 FIFOSTA    bit 8 data available, 6:0 count
 *         0x1C INTCTL     bit 3 RX DRQ enable
 *         0x30 CHCFG      bits 7:4 RX slots - 1
 *         (its rate is set by the clocks, not here: the "rate" property)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AW_AUDIO "aw-audio"
OBJECT_DECLARE_SIMPLE_TYPE(AwAudioState, AW_AUDIO)

#define AW_AUDIO_SIZE       0x400
#define AW_AUDIO_FIFO       64

#define CODEC_ADC_FIFOC     0x30
#define CODEC_ADC_FIFOS     0x38
#define CODEC_ADC_RXDATA    0x40
#define DAUDIO_CTL          0x00
#define DAUDIO_RXFIFO       0x10
#define DAUDIO_FIFOCTL      0x14
#define DAUDIO_FIFOSTA      0x18
#define DAUDIO_INTCTL       0x1C
#define DAUDIO_CHCFG        0x30

struct AwAudioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq drq;
    QEMUTimer *timer;
    bool daudio;
    uint32_t rate;
    uint32_t regs[AW_AUDIO_SIZE / 4];
    uint32_t count;             /* samples in the RX FIFO */
    uint64_t frac;              /* sample remainder, in 1/1000 */
};

static bool aw_audio_rx_on(AwAudioState *s)
{
    if (s->daudio) {
        return (s->regs[DAUDIO_CTL >> 2] & 3) == 3;
    }
    return s->regs[CODEC_ADC_FIFOC >> 2] & (1u << 28);
}

static bool aw_audio_drq_on(AwAudioState *s)
{
    if (s->daudio) {
        return s->regs[DAUDIO_INTCTL >> 2] & (1u << 3);
    }
    return s->regs[CODEC_ADC_FIFOC >> 2] & (1u << 3);
}

static void aw_audio_update(AwAudioState *s)
{
    qemu_set_irq(s->drq, s->count && aw_audio_rx_on(s) && aw_audio_drq_on(s));
}

/* Samples a second, all channels. */
static uint32_t aw_audio_samples_per_sec(AwAudioState *s)
{
    static const uint32_t fs[8] = {
        48000, 32000, 24000, 16000, 12000, 8000, 48000, 48000,
    };

    if (s->daudio) {
        return s->rate * (((s->regs[DAUDIO_CHCFG >> 2] >> 4) & 0xF) + 1);
    }
    return fs[s->regs[CODEC_ADC_FIFOC >> 2] >> 29] *
           MAX(ctpop32((s->regs[CODEC_ADC_FIFOC >> 2] >> 12) & 0xF), 1);
}

static void aw_audio_tick(void *opaque)
{
    AwAudioState *s = opaque;

    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    if (!aw_audio_rx_on(s)) {
        return;
    }
    s->frac += aw_audio_samples_per_sec(s);
    s->count = MIN(s->count + s->frac / 1000, AW_AUDIO_FIFO);
    s->frac %= 1000;
    aw_audio_update(s);
}

static uint64_t aw_audio_read(void *opaque, hwaddr addr, unsigned size)
{
    AwAudioState *s = opaque;
    uint32_t v = s->regs[addr >> 2];

    if (s->daudio) {
        switch (addr) {
        case DAUDIO_RXFIFO:
            if (s->count) {
                s->count--;
                aw_audio_update(s);
            }
            return 0;
        case DAUDIO_FIFOSTA:
            return (1u << 28) | (s->count ? 1u << 8 : 0) | s->count;
        }
        return v;
    }
    switch (addr) {
    case CODEC_ADC_RXDATA:
        if (s->count) {
            s->count--;
            aw_audio_update(s);
        }
        return 0;
    case CODEC_ADC_FIFOS:
        return (s->count ? 1u << 23 : 0) | (s->count << 8);
    }
    return v;
}

static void aw_audio_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    AwAudioState *s = opaque;

    if (s->daudio && addr == DAUDIO_FIFOCTL && (val & (1u << 24))) {
        s->count = 0;
        val &= ~(3u << 24);
    } else if (!s->daudio && addr == CODEC_ADC_FIFOC && (val & 1)) {
        s->count = 0;
        val &= ~1u;
    }
    s->regs[addr >> 2] = val;
    aw_audio_update(s);
}

static const MemoryRegionOps aw_audio_ops = {
    .read = aw_audio_read,
    .write = aw_audio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void aw_audio_reset_hold(Object *obj, ResetType type)
{
    AwAudioState *s = AW_AUDIO(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->count = 0;
    s->frac = 0;
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void aw_audio_reset_exit(Object *obj, ResetType type)
{
    aw_audio_update(AW_AUDIO(obj));
}

static void aw_audio_init(Object *obj)
{
    AwAudioState *s = AW_AUDIO(obj);

    memory_region_init_io(&s->iomem, obj, &aw_audio_ops, s, TYPE_AW_AUDIO,
                          AW_AUDIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->drq, "drq", 1);
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, aw_audio_tick, s);
}

static const Property aw_audio_properties[] = {
    DEFINE_PROP_BOOL("daudio", AwAudioState, daudio, false),
    DEFINE_PROP_UINT32("rate", AwAudioState, rate, 16000),
};

static void aw_audio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_audio_reset_hold;
    rc->phases.exit = aw_audio_reset_exit;
    device_class_set_props(dc, aw_audio_properties);
}

static const TypeInfo aw_audio_info = {
    .name           = TYPE_AW_AUDIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwAudioState),
    .instance_init  = aw_audio_init,
    .class_init     = aw_audio_class_init,
};

static void aw_audio_register_types(void)
{
    type_register_static(&aw_audio_info);
}

type_init(aw_audio_register_types)
