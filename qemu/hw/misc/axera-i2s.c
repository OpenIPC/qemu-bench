/*
 * Synopsys DesignWare APB I2S (DW_apb_i2s) as used by Axera AX620E
 * (i2s_mst@0x06050000 "axera,dwc-i2s-mst", i2s_slv@0x06051000
 * "axera,dwc-i2s-slv") and AX620U (i2s_mst@0x049ac000).
 *
 * Both guests bind the mainline-derived sound/soc/dwc/dwc-i2s.c
 * driver ("designware-i2s"; AX620E built-in, AX620U inside
 * ax_audio.ko). Before this model existed the block was unbacked:
 * COMP_PARAM_1/2 (0x1f4/0x1f0) read as zero, so dw_configure_dai()
 * advertised no TX/RX stream and no master/slave capability, set_fmt
 * returned -EINVAL, no PCM device node was ever created and the
 * camera app's AI/AO open failed (AX620E AX_AI_EnableDev 0x80200215,
 * AX620U "failed to open PCM").
 *
 * Register map (DW_apb_i2s databook / linux sound/soc/dwc/local.h):
 *   0x000 IER  0x004 IRER  0x008 ITER  0x00c CER  0x010 CCR
 *   0x014 RXFFR  0x018 TXFFR
 *   per channel n (0x40 * n +): 0x20 LRBR/LTHR 0x24 RRBR/RTHR
 *     0x28 RER 0x2c TER 0x30 RCR 0x34 TCR 0x38 ISR 0x3c IMR
 *     0x40 ROR 0x44 TOR 0x48 RFCR 0x4c TFCR 0x50 RFF 0x54 TFF
 *   0x1c0 RXDMA 0x1c4 RRXDMA 0x1c8 TXDMA 0x1cc RTXDMA
 *   0x1f0 COMP_PARAM_2 0x1f4 COMP_PARAM_1 0x1f8 COMP_VERSION
 *   0x1fc COMP_TYPE
 *
 * Capability decoding differs between the two guest kernels and is
 * why "master" is a property: AX620E's 5.15 dw_configure_dai() sets
 * DW_I2S_MASTER when COMP1.MODE_EN (bit 4) is set and DW_I2S_SLAVE
 * otherwise (i2s_slv is linked with the codec as bit/frame master, so
 * it must report MODE_EN=0); AX620U's 4.19 copy sets both on MODE_EN.
 *
 * Data path: only stereo channel 0 carries data. A virtual-clock
 * timer advances the serial clock at "rate" frames/s: every frame
 * pushes one L/R entry into the RX FIFO (a low sine tone, or silence
 * with tone-hz=0) and drains one entry from the TX FIFO (playback is
 * discarded). The FIFO level drives the real status bits (RXDA when
 * level > RFCR, TXFE when level <= TFCR, RXFO/TXFO on overrun), the
 * interrupt line (AX620U: PIO mode, the driver's i2s_irq_handler
 * moves fifo_th frames per RXDA/TXFE interrupt) and two DMA request
 * outputs (AX620E: no "interrupts" in the devicetree, so the driver
 * uses dmaengine through AX_DMAPER, which pulls/pushes words through
 * RXDMA/TXDMA while the request is high -- see axera-dmaper.c). Either
 * way ALSA's period interrupts arrive at real-time pace.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/misc/axera-i2s.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraI2SState, AXERA_I2S)

#define I2S_SIZE        0x400

#define I2S_IER         0x000
#define I2S_IRER        0x004
#define I2S_ITER        0x008
#define I2S_CER         0x00c
#define I2S_CCR         0x010
#define I2S_RXFFR       0x014
#define I2S_TXFFR       0x018
#define I2S_CH_BASE     0x020
#define I2S_CH_STRIDE   0x040
#define I2S_NR_CH       4
#define I2S_RXDMA       0x1c0
#define I2S_RRXDMA      0x1c4
#define I2S_TXDMA       0x1c8
#define I2S_RTXDMA      0x1cc
#define I2S_COMP_PARAM_2 0x1f0
#define I2S_COMP_PARAM_1 0x1f4
#define I2S_COMP_VERSION 0x1f8
#define I2S_COMP_TYPE   0x1fc

/* per-channel register offsets relative to I2S_CH_BASE + n * stride */
#define CH_LRBR         0x00
#define CH_RRBR         0x04
#define CH_RER          0x08
#define CH_TER          0x0c
#define CH_RCR          0x10
#define CH_TCR          0x14
#define CH_ISR          0x18
#define CH_IMR          0x1c
#define CH_ROR          0x20
#define CH_TOR          0x24
#define CH_RFCR         0x28
#define CH_TFCR         0x2c
#define CH_RFF          0x30
#define CH_TFF          0x34
#define CH_NREGS        (I2S_CH_STRIDE / 4)

#define ISR_RXDA        BIT(0)
#define ISR_RXFO        BIT(1)
#define ISR_TXFE        BIT(4)
#define ISR_TXFO        BIT(5)

#define FIFO_DEPTH      16      /* COMP1.FIFO_DEPTH_GLOBAL = 3 */

/*
 * The RX FIFO is the advertised 16 entries. Real silicon gives the CPU
 * 1 ms (8 frames at 8 kHz) to drain it after RXDA, which an emulated
 * guest under TCG load often misses, so some capture frames are lost
 * to RXFO (an AX620U run lost ~37%). Buffering more was tried (1024
 * frames) and is worse: the level-triggered RXDA then stays asserted
 * while the guest works through the backlog, an interrupt storm on
 * CPU0 that starves the SPI-NAND DMA completion past the driver's
 * deadline (SPI transfer timed out, SQUASHFS errors, App bus error).
 */
#define RX_CAPACITY     FIFO_DEPTH

/*
 * COMP_PARAM_1: APB_DATA_WIDTH=2 (32 bit), FIFO_DEPTH_GLOBAL=3 (16),
 * MODE_EN (bit 4, per "master"), TX_ENABLED, RX_ENABLED, one stereo
 * channel each way (RX/TX_CHANNELS=0), TX_WORDSIZE_0=4 (32 bit, so
 * S16/S24/S32 are all offered). COMP_PARAM_2: RX_WORDSIZE_0=4.
 */
#define COMP1_BASE      (0x2 | (0x3 << 2) | BIT(5) | BIT(6) | (0x4 << 16))
#define COMP1_MODE_EN   BIT(4)
#define COMP2_VALUE     0x4
#define COMP_VERSION    0x3130302a      /* "1.00*" */
#define COMP_TYPE       0x445701a0

/* Advance the serial clock at least this often while a stream runs. */
#define TICK_FRAMES     4

struct AxeraI2SState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dma_rx_req;
    qemu_irq dma_tx_req;
    QEMUTimer timer;

    /* properties */
    bool master;
    uint32_t rate;
    uint32_t tone_hz;

    uint32_t ier, irer, iter, cer, ccr;
    uint32_t ch[I2S_NR_CH][CH_NREGS];

    /* channel 0 FIFOs, in stereo entries */
    uint32_t rx_fifo[RX_CAPACITY][2];
    uint32_t rx_head, rx_level;
    bool rx_half;               /* RXDMA: left word already returned */
    uint32_t tx_level;
    uint32_t tx_left;           /* LTHR / first TXDMA word pending */
    bool tx_half;

    /* serial clock */
    bool running;
    int64_t t0_ns;
    uint64_t frames_done;
    uint64_t phase;             /* sample counter for the tone */
    bool rx_req_level, tx_req_level;
};

static bool axera_i2s_rx_on(AxeraI2SState *s)
{
    return (s->ier & 1) && (s->irer & 1) && (s->ch[0][CH_RER / 4] & 1);
}

static bool axera_i2s_tx_on(AxeraI2SState *s)
{
    return (s->ier & 1) && (s->iter & 1) && (s->ch[0][CH_TER / 4] & 1);
}

static void axera_i2s_update(AxeraI2SState *s)
{
    uint32_t *c = s->ch[0];
    uint32_t isr = c[CH_ISR / 4] & (ISR_RXFO | ISR_TXFO);
    bool rx_req, tx_req;
    int n;

    if (axera_i2s_rx_on(s) && s->rx_level > (c[CH_RFCR / 4] & 0xf)) {
        isr |= ISR_RXDA;
    }
    if (axera_i2s_tx_on(s) && s->tx_level <= (c[CH_TFCR / 4] & 0xf)) {
        isr |= ISR_TXFE;
    }
    c[CH_ISR / 4] = isr;
    for (n = 1; n < I2S_NR_CH; n++) {
        s->ch[n][CH_ISR / 4] = 0;
    }

    qemu_set_irq(s->irq, (s->ier & 1) && (isr & ~c[CH_IMR / 4] & 0x33));

    rx_req = axera_i2s_rx_on(s) && (s->rx_level > 0 || s->rx_half);
    tx_req = axera_i2s_tx_on(s) && s->tx_level < FIFO_DEPTH;
    if (rx_req != s->rx_req_level) {
        s->rx_req_level = rx_req;
        qemu_set_irq(s->dma_rx_req, rx_req);
    }
    if (tx_req != s->tx_req_level) {
        s->tx_req_level = tx_req;
        qemu_set_irq(s->dma_tx_req, tx_req);
    }
}

static int32_t axera_i2s_sample(AxeraI2SState *s)
{
    double v;

    if (!s->tone_hz || !s->rate) {
        return 0;
    }
    /* ~ -24 dBFS sine, 16-bit scale, sign-extended to the 32-bit word */
    v = sin(2.0 * M_PI * (double)s->tone_hz * (double)s->phase /
            (double)s->rate);
    return (int32_t)(v * 2048.0);
}

static void axera_i2s_clock_frames(AxeraI2SState *s, uint64_t n)
{
    bool rx = axera_i2s_rx_on(s), tx = axera_i2s_tx_on(s);

    while (n--) {
        if (rx) {
            if (s->rx_level < RX_CAPACITY) {
                uint32_t slot = (s->rx_head + s->rx_level) % RX_CAPACITY;
                int32_t v = axera_i2s_sample(s);

                s->rx_fifo[slot][0] = v;
                s->rx_fifo[slot][1] = v;
                s->rx_level++;
            } else {
                s->ch[0][CH_ISR / 4] |= ISR_RXFO;
            }
        }
        if (tx && s->tx_level) {
            s->tx_level--;      /* playback: consumed and discarded */
        }
        s->phase++;
    }
}

static void axera_i2s_schedule(AxeraI2SState *s)
{
    bool want = (s->ier & 1) && s->rate &&
                (axera_i2s_rx_on(s) || axera_i2s_tx_on(s));
    int64_t next;

    if (!want) {
        if (s->running) {
            timer_del(&s->timer);
            s->running = false;
        }
        return;
    }
    if (!s->running) {
        s->running = true;
        s->t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->frames_done = 0;
    }
    next = s->t0_ns + muldiv64(s->frames_done + TICK_FRAMES,
                               NANOSECONDS_PER_SECOND, s->rate);
    timer_mod(&s->timer, next);
}

static void axera_i2s_tick(void *opaque)
{
    AxeraI2SState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t due;

    if (!s->running) {
        return;
    }
    due = muldiv64(now - s->t0_ns, s->rate, NANOSECONDS_PER_SECOND);
    if (due > s->frames_done) {
        uint64_t n = due - s->frames_done;

        /* After a long host stall don't replay seconds of backlog:
         * the RX buffer saturates after RX_CAPACITY frames anyway. */
        axera_i2s_clock_frames(s, MIN(n, (uint64_t)RX_CAPACITY));
        s->frames_done = due;
    }
    axera_i2s_update(s);
    axera_i2s_schedule(s);
}

static uint32_t axera_i2s_pop_rx(AxeraI2SState *s, bool right)
{
    uint32_t v;

    if (!s->rx_level) {
        return 0;
    }
    v = s->rx_fifo[s->rx_head][right ? 1 : 0];
    if (right) {
        s->rx_head = (s->rx_head + 1) % RX_CAPACITY;
        s->rx_level--;
    }
    return v;
}

static void axera_i2s_push_tx(AxeraI2SState *s)
{
    if (s->tx_level < FIFO_DEPTH) {
        s->tx_level++;
    } else {
        s->ch[0][CH_ISR / 4] |= ISR_TXFO;
    }
}

static uint64_t axera_i2s_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraI2SState *s = opaque;
    uint32_t v = 0;

    switch (offset) {
    case I2S_IER:
        return s->ier;
    case I2S_IRER:
        return s->irer;
    case I2S_ITER:
        return s->iter;
    case I2S_CER:
        return s->cer;
    case I2S_CCR:
        return s->ccr;
    case I2S_RXDMA:
    case I2S_RRXDMA:
        /* one sample per access, left then right (DW databook) */
        v = axera_i2s_pop_rx(s, s->rx_half);
        s->rx_half = !s->rx_half;
        axera_i2s_update(s);
        return v;
    case I2S_COMP_PARAM_1:
        return COMP1_BASE | (s->master ? COMP1_MODE_EN : 0);
    case I2S_COMP_PARAM_2:
        return COMP2_VALUE;
    case I2S_COMP_VERSION:
        return COMP_VERSION;
    case I2S_COMP_TYPE:
        return COMP_TYPE;
    }

    if (offset >= I2S_CH_BASE &&
        offset < I2S_CH_BASE + I2S_NR_CH * I2S_CH_STRIDE) {
        unsigned n = (offset - I2S_CH_BASE) / I2S_CH_STRIDE;
        unsigned r = (offset - I2S_CH_BASE) % I2S_CH_STRIDE;

        switch (r) {
        case CH_LRBR:
            return n == 0 ? axera_i2s_pop_rx(s, false) : 0;
        case CH_RRBR:
            if (n == 0) {
                v = axera_i2s_pop_rx(s, true);
                s->rx_half = false;
                axera_i2s_update(s);
            }
            return v;
        case CH_ROR:            /* read clears RX overrun */
            s->ch[n][CH_ISR / 4] &= ~ISR_RXFO;
            axera_i2s_update(s);
            return 0;
        case CH_TOR:            /* read clears TX overrun */
            s->ch[n][CH_ISR / 4] &= ~ISR_TXFO;
            axera_i2s_update(s);
            return 0;
        default:
            return s->ch[n][r / 4];
        }
    }
    return 0;
}

static void axera_i2s_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AxeraI2SState *s = opaque;

    switch (offset) {
    case I2S_IER:
        s->ier = value & 1;
        break;
    case I2S_IRER:
        s->irer = value & 1;
        break;
    case I2S_ITER:
        s->iter = value & 1;
        break;
    case I2S_CER:
        s->cer = value & 1;
        break;
    case I2S_CCR:
        s->ccr = value & 0x1f;
        break;
    case I2S_RXFFR:
        if (value & 1) {
            s->rx_level = 0;
            s->rx_head = 0;
            s->rx_half = false;
        }
        break;
    case I2S_TXFFR:
        if (value & 1) {
            s->tx_level = 0;
            s->tx_half = false;
        }
        break;
    case I2S_TXDMA:
    case I2S_RTXDMA:
        if (s->tx_half) {
            axera_i2s_push_tx(s);
        } else {
            s->tx_left = value;
        }
        s->tx_half = !s->tx_half;
        break;
    default:
        if (offset >= I2S_CH_BASE &&
            offset < I2S_CH_BASE + I2S_NR_CH * I2S_CH_STRIDE) {
            unsigned n = (offset - I2S_CH_BASE) / I2S_CH_STRIDE;
            unsigned r = (offset - I2S_CH_BASE) % I2S_CH_STRIDE;

            switch (r) {
            case CH_LRBR:       /* LTHR */
                if (n == 0) {
                    s->tx_left = value;
                }
                break;
            case CH_RRBR:       /* RTHR: completes the stereo entry */
                if (n == 0) {
                    axera_i2s_push_tx(s);
                    s->tx_half = false;
                }
                break;
            case CH_ISR:
            case CH_ROR:
            case CH_TOR:
                break;          /* read-only */
            case CH_RFF:
                if ((value & 1) && n == 0) {
                    s->rx_level = 0;
                    s->rx_head = 0;
                    s->rx_half = false;
                }
                break;
            case CH_TFF:
                if ((value & 1) && n == 0) {
                    s->tx_level = 0;
                    s->tx_half = false;
                }
                break;
            default:
                s->ch[n][r / 4] = value;
                break;
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "axera-i2s: write to unknown reg 0x%" HWADDR_PRIx
                          "\n", offset);
        }
        break;
    }
    axera_i2s_update(s);
    axera_i2s_schedule(s);
}

static const MemoryRegionOps axera_i2s_ops = {
    .read = axera_i2s_read,
    .write = axera_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * AX620E's dmaengine slave config moves S16 audio with a 2-byte
     * peripheral width (one 16-bit sample per RXDMA/TXDMA access, two
     * per frame -- confirmed from the live AX_DMAPER descriptors), so
     * the DMA ports must accept 16-bit accesses; everything else is
     * 32-bit in practice.
     */
    .valid = { .min_access_size = 2, .max_access_size = 4 },
};

static void axera_i2s_reset(DeviceState *dev)
{
    AxeraI2SState *s = AXERA_I2S(dev);
    int n;

    timer_del(&s->timer);
    s->running = false;
    s->ier = s->irer = s->iter = s->cer = s->ccr = 0;
    memset(s->ch, 0, sizeof(s->ch));
    for (n = 0; n < I2S_NR_CH; n++) {
        s->ch[n][CH_IMR / 4] = 0x33;    /* all sources masked */
    }
    s->rx_head = s->rx_level = s->tx_level = 0;
    s->rx_half = s->tx_half = false;
    s->phase = 0;
    s->rx_req_level = s->tx_req_level = false;
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->dma_rx_req, 0);
    qemu_set_irq(s->dma_tx_req, 0);
}

static void axera_i2s_realize(DeviceState *dev, Error **errp)
{
    AxeraI2SState *s = AXERA_I2S(dev);

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, axera_i2s_tick, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_i2s_ops, s,
                          TYPE_AXERA_I2S, I2S_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_rx_req, AXERA_I2S_DMA_RX_REQ, 1);
    qdev_init_gpio_out_named(dev, &s->dma_tx_req, AXERA_I2S_DMA_TX_REQ, 1);
}

static const Property axera_i2s_props[] = {
    DEFINE_PROP_BOOL("master", AxeraI2SState, master, true),
    DEFINE_PROP_UINT32("rate", AxeraI2SState, rate, 8000),
    DEFINE_PROP_UINT32("tone-hz", AxeraI2SState, tone_hz, 400),
};

static void axera_i2s_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_i2s_realize;
    device_class_set_legacy_reset(dc, axera_i2s_reset);
    device_class_set_props(dc, axera_i2s_props);
}

static const TypeInfo axera_i2s_info = {
    .name = TYPE_AXERA_I2S,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraI2SState),
    .class_init = axera_i2s_class_init,
};

static void axera_i2s_register_types(void)
{
    type_register_static(&axera_i2s_info);
}

type_init(axera_i2s_register_types)
