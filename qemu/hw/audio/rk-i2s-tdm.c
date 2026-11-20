/*
 * Rockchip I2S/TDM controller ("rockchip,rv1126-i2s-tdm": RV1126 I2S0 at
 * 0xFF800000), the sound card's CPU side, from the kernel's
 * sound/soc/rockchip/rockchip_i2s_tdm.h (develop-4.19):
 *
 *   0x00 TXCR  0x04 RXCR  0x08 CKR    0x30 TDM_TXCR  0x34 TDM_RXCR
 *   0x0c TXFIFOLR / 0x2c RXFIFOLR  FIFO levels, bits 5:0
 *   0x10 DMACR  bit 8 TDE, 4:0 TDL (TX request while level <= TDL);
 *               bit 24 RDE, 20:16 RDL (RX request at level >= RDL + 1)
 *   0x14 INTCR  0x18 INTSR
 *   0x1c XFER   bit 0 TX start, bit 1 RX start
 *   0x20 CLR    bit 0 TX, bit 1 RX: FIFO and logic clear (self-clearing)
 *   0x24 TXDR   0x28 RXDR  the FIFOs
 *
 * The FIFOs move at the sample clock.  With "cru-base" set (RV1126) it
 * is the one the registers give: the MCLK the CRU makes for this I2S --
 * CLKSEL_CON(27) parent (CPLL / GPLL, from their own registers) and
 * integer divider, CON(28) / CON(29) fraction, CON(30) mux -- over
 * CLKDIV's TXM / RXM (MCLK to BCLK) and CKR's TSD / RSD (BCLK to LRCK),
 * a FIFO word per channel, or per two channels when 16-bit samples are
 * packed (VDW <= 16 without HWT).  Otherwise, and whenever that comes
 * out as nothing, "word-rate" words a second (default 96000), updated
 * every few words' time by the virtual time elapsed, so a late tick
 * catches up instead of slowing the stream.  The DMA requests are the PL330's per-peripheral
 * "busy" inputs, high to stall its DMAWFP: busy while there is nothing
 * to do -- TX FIFO above the threshold, RX FIFO below it -- so the
 * controller paces playback and capture as on the chip.  Captured
 * samples are silence, or with "rx-tone-hz" a sine at that frequency
 * (a quarter of full scale, every channel alike) at the frame rate the
 * clock gives, in RXCR's width (VDW, bits 4:0), two 16-bit samples a word
 * when packed; played ones go nowhere.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"

#define TYPE_RK_I2S_TDM "rk-i2s-tdm"
OBJECT_DECLARE_SIMPLE_TYPE(RkI2sTdmState, RK_I2S_TDM)

#define R_TXFIFOLR  0x0c
#define R_DMACR     0x10
#define R_INTSR     0x18
#define R_XFER      0x1c
#define R_CLR       0x20
#define R_TXDR      0x24
#define R_RXDR      0x28
#define R_RXFIFOLR  0x2c
#define REGS_SIZE   0x1000
#define NREGS       (0x40 / 4)

#define FIFO_DEPTH  32
#define BATCH       4

#define DMACR_TDE   (1u << 8)
#define DMACR_RDE   (1u << 24)
#define XFER_TXS    (1u << 0)
#define XFER_RXS    (1u << 1)

struct RkI2sTdmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq drq_busy[2];       /* 0 TX, 1 RX: the PL330's busy inputs */
    QEMUTimer *clock;

    uint32_t regs[NREGS];
    uint32_t tx_level, rx_level;
    uint32_t word_rate;
    uint64_t cru_base;          /* RV1126 CRU: CLKSEL_CON(x) at 0x100 + 4x */
    uint64_t pll_addr[2];       /* CPLL, GPLL CON0 (CON1 follows) */
    uint32_t rate[2];           /* words a second now: TX, RX */
    int64_t last_ns;            /* the sample clock's last update */
    uint64_t acc[2];            /* words owed, in ns * rate units */
    uint32_t tone_hz;           /* captured test tone, 0: silence */
    uint64_t rx_words;          /* words read from RXDR since RX start */
};

/*
 * Requests follow the FIFO levels: TX while at or below TDL, RX while at
 * or above RDL + 1.  (QEMU's PL330 queues the loads it executes and does
 * the reads later, so a per-burst handshake cannot be kept exactly.)
 */
static void rk_i2s_update_drq(RkI2sTdmState *s)
{
    uint32_t dmacr = s->regs[R_DMACR / 4];
    uint32_t xfer = s->regs[R_XFER / 4];
    bool tx = (dmacr & DMACR_TDE) && (xfer & XFER_TXS) &&
              s->tx_level <= (dmacr & 0x1f);
    bool rx = (dmacr & DMACR_RDE) && (xfer & XFER_RXS) &&
              s->rx_level >= ((dmacr >> 16) & 0x1f) + 1;

    qemu_set_irq(s->drq_busy[0], !tx);
    qemu_set_irq(s->drq_busy[1], !rx);
}

static uint32_t rk_i2s_rd(hwaddr a)
{
    return address_space_ldl_le(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/* pll_rk3328: 24 MHz * FBDIV / REFDIV / POSTDIV1 / POSTDIV2 (integer) */
static uint64_t rk_i2s_pll_hz(hwaddr con0)
{
    uint32_t c0 = rk_i2s_rd(con0), c1 = rk_i2s_rd(con0 + 4);
    uint32_t fb = c0 & 0xfff, pd1 = (c0 >> 12) & 7;
    uint32_t ref = c1 & 0x3f, pd2 = (c1 >> 6) & 7;

    if (!fb || !ref || !pd1 || !pd2) {
        return 0;
    }
    return 24000000ull * fb / ref / pd1 / pd2;
}

/* The FIFO words a second direction @rx moves, 0 when unknown. */
static uint32_t rk_i2s_word_rate(RkI2sTdmState *s, int rx)
{
    hwaddr sel = s->cru_base + 0x100;
    uint32_t con27, mux, div, sd, ctl, vdw, ch;
    uint64_t mclk, num, den;

    if (!s->cru_base) {
        return 0;
    }
    con27 = rk_i2s_rd(sel + 27 * 4);
    mux = (rk_i2s_rd(sel + 30 * 4) >> (rx ? 2 : 0)) & 3;
    mclk = rk_i2s_pll_hz(s->pll_addr[(con27 >> (rx ? 15 : 7)) & 1]);
    mclk /= ((con27 >> (rx ? 8 : 0)) & 0x7f) + 1;
    if (mux == 1) {                             /* the fraction */
        uint32_t frac = rk_i2s_rd(sel + (rx ? 29 : 28) * 4);

        num = frac >> 16;
        den = frac & 0xffff;
        mclk = den ? mclk * num / den : 0;
    } else if (mux == 3) {
        mclk = 12000000;                        /* xin12m */
    } else if (mux == 2) {
        return 0;                               /* from the pin: unknown */
    }
    div = ((s->regs[0x38 / 4] >> (rx ? 8 : 0)) & 0xff) + 1;
    sd = ((s->regs[0x08 / 4] >> (rx ? 8 : 0)) & 0xff) + 1;
    ctl = s->regs[(rx ? 0x04 : 0x00) / 4];
    vdw = (ctl & 0x1f) + 1;
    ch = 2 * (((ctl >> 15) & 3) + 1);
    if (vdw <= 16 && !(ctl & (1u << 14))) {
        ch = MAX(ch / 2, 1);                    /* two samples a word */
    }
    return mclk / div / sd * ch;
}

static void rk_i2s_arm(RkI2sTdmState *s)
{
    if (s->regs[R_XFER / 4] & (XFER_TXS | XFER_RXS)) {
        if (!timer_pending(s->clock)) {
            int d;

            s->last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            for (d = 0; d < 2; d++) {
                uint32_t r = rk_i2s_word_rate(s, d);

                s->rate[d] = r ? r : s->word_rate;
            }
        }
        timer_mod(s->clock, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND * BATCH /
                  MAX(MAX(s->rate[0], s->rate[1]), 1));
    } else {
        timer_del(s->clock);
    }
}

static void rk_i2s_tick(void *opaque)
{
    RkI2sTdmState *s = opaque;
    uint32_t xfer = s->regs[R_XFER / 4];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t n;

    /* the words the elapsed time is worth: late ticks catch up */
    for (int d = 0; d < 2; d++) {
        s->acc[d] += (uint64_t)(now - s->last_ns) * s->rate[d];
        n = s->acc[d] / NANOSECONDS_PER_SECOND;
        s->acc[d] %= NANOSECONDS_PER_SECOND;
        if (d == 0 && (xfer & XFER_TXS)) {
            s->tx_level -= MIN(s->tx_level, n);
        } else if (d == 1 && (xfer & XFER_RXS)) {
            s->rx_level = MIN(s->rx_level + n, FIFO_DEPTH);
        }
    }
    s->last_ns = now;
    rk_i2s_update_drq(s);
    rk_i2s_arm(s);
}

/* The next captured word: silence, or the test tone */
static uint32_t rk_i2s_rx_word(RkI2sTdmState *s)
{
    uint32_t ctl = s->regs[0x04 / 4];
    int vdw = (ctl & 0x1f) + 1, ch = 2 * (((ctl >> 15) & 3) + 1);
    bool packed = vdw <= 16 && !(ctl & (1u << 14));
    int per_frame = packed ? MAX(ch / 2, 1) : ch;
    uint64_t frame = s->rx_words++ / per_frame;
    double fs = (double)s->rate[1] / per_frame;
    int32_t v;

    if (!s->tone_hz || fs < 1) {
        return 0;
    }
    v = lrint(sin(2 * M_PI * s->tone_hz * (double)(frame % (uint64_t)fs) /
                  fs) * 8191);                  /* a quarter of 16 bits */
    if (packed) {
        return (uint16_t)v | (uint32_t)(uint16_t)v << 16;
    }
    if (vdw > 16) {
        v *= 1 << (vdw - 16);
    }
    return (uint32_t)v & (vdw >= 32 ? ~0u : (1u << vdw) - 1);
}

static uint64_t rk_i2s_read(void *opaque, hwaddr off, unsigned size)
{
    RkI2sTdmState *s = opaque;

    switch (off) {
    case R_TXFIFOLR:
        return s->tx_level;
    case R_RXFIFOLR:
        return s->rx_level;
    case R_INTSR:
    case R_CLR:
        return 0;
    case R_RXDR:
        if (s->rx_level) {
            s->rx_level--;
        }
        rk_i2s_update_drq(s);
        return rk_i2s_rx_word(s);
    case R_TXDR:
        return 0;
    }
    return off < NREGS * 4 ? s->regs[off / 4] : 0;
}

static void rk_i2s_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkI2sTdmState *s = opaque;

    switch (off) {
    case R_TXDR:
        if (s->tx_level < FIFO_DEPTH) {
            s->tx_level++;
        }
        break;
    case R_CLR:
        if (val & 1) {
            s->tx_level = 0;
        }
        if (val & 2) {
            s->rx_level = 0;
            s->rx_words = 0;
        }
        break;
    case R_TXFIFOLR:
    case R_RXFIFOLR:
    case R_INTSR:
    case R_RXDR:
        return;
    default:
        if (off < NREGS * 4) {
            s->regs[off / 4] = val;
        }
        if (off == R_XFER) {
            rk_i2s_arm(s);
        }
        break;
    }
    rk_i2s_update_drq(s);
}

static const MemoryRegionOps rk_i2s_ops = {
    .read = rk_i2s_read,
    .write = rk_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_i2s_reset(DeviceState *dev)
{
    RkI2sTdmState *s = RK_I2S_TDM(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->tx_level = s->rx_level = 0;
    timer_del(s->clock);
    rk_i2s_update_drq(s);
}

static void rk_i2s_init(Object *obj)
{
    RkI2sTdmState *s = RK_I2S_TDM(obj);

    memory_region_init_io(&s->iomem, obj, &rk_i2s_ops, s, TYPE_RK_I2S_TDM,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->drq_busy, "drq-busy", 2);
    s->clock = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_i2s_tick, s);
}

static const Property rk_i2s_properties[] = {
    DEFINE_PROP_UINT32("word-rate", RkI2sTdmState, word_rate, 96000),
    DEFINE_PROP_UINT32("rx-tone-hz", RkI2sTdmState, tone_hz, 0),
    DEFINE_PROP_UINT64("cru-base", RkI2sTdmState, cru_base, 0),
    DEFINE_PROP_UINT64("cpll-addr", RkI2sTdmState, pll_addr[0], 0),
    DEFINE_PROP_UINT64("gpll-addr", RkI2sTdmState, pll_addr[1], 0),
};

static const VMStateDescription rk_i2s_vmstate = {
    .name = TYPE_RK_I2S_TDM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkI2sTdmState, NREGS),
        VMSTATE_UINT32(tx_level, RkI2sTdmState),
        VMSTATE_UINT32(rx_level, RkI2sTdmState),
        VMSTATE_TIMER_PTR(clock, RkI2sTdmState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_i2s_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_i2s_reset);
    device_class_set_props(dc, rk_i2s_properties);
    dc->vmsd = &rk_i2s_vmstate;
}

static const TypeInfo rk_i2s_info = {
    .name = TYPE_RK_I2S_TDM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkI2sTdmState),
    .instance_init = rk_i2s_init,
    .class_init = rk_i2s_class_init,
};

static void rk_i2s_register_types(void)
{
    type_register_static(&rk_i2s_info);
}

type_init(rk_i2s_register_types)
