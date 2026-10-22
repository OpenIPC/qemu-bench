/*
 * Novatek NA51103 audio interface (DAI): I2S/TDM with its own DMA.
 *
 * Five instances on NA51103 (the tree's "nvt,nvt_dai" node: 0x2_F0850000,
 * 0x2_F0860000, 0x2_F0220000, 0x2_F0230000, 0x2_F0870000; SPI 15, 34..37),
 * driven by the HDAL's kdrv_audioio/nvt_audio.ko.  Registers, from that
 * module's dai_* functions (it is not stripped):
 *
 *   0x00  CTRL     bit 4/5 TX DMA ch 0/1, bit 6 RX DMA, bit 12/13 TX ch 0/1,
 *                  bit 14 RX
 *   0x20/0x28      interrupt enable / status 0: bit 0 TX ch 0 buffer done,
 *                  bit 1 TX ch 1, bit 2 RX buffer done
 *   0x24/0x2C      enable / status 1
 *   0x30/0xC0      per-channel enable / status: RX channel k buffer done
 *   0x34/0xC4      per-channel enable / status: DMA "load done" -- the
 *                  queued buffer was taken.  aud_record_rx() and the
 *                  playback path poll [19:0] after enabling the DMA, for
 *                  up to a second; that poll timing out is what took the
 *                  whole HDAL down before this model
 *   0x38/0xC8      enable / status (storage)
 *   0x100/0x110    TX ch 0 buffer address / size in words; 0x108 reads the
 *                  address in use.  TX ch 1 at 0x120/0x130/0x128
 *   0x140/0x150    RX buffer (dai_set_rx_dma_para); 0x148 in use.  A second
 *                  at 0x160/0x170/0x168
 *   0x200 + 0x20k  RX multi-channel buffer k, k = 0..20: address, and its
 *   0x210 + 0x20k  size in words (dai_set_rx_muti_ch_dma_para), which ids
 *                  0/1 use
 *
 * Every status register is write-1-to-clear (dai_isr() writes back what it
 * read); the line is up while any status bit is enabled.  Buffers are
 * double-buffered: the driver queues the next one while the current one
 * runs, and each completion takes what is queued.  Here a DMA enable takes
 * the queued buffers at once (load done), and the buffers complete on a
 * timer at a nominal 8 kHz 16-bit mono per channel, the rate the vendor
 * boards record at; RX buffers fill with silence.  There is no codec
 * behind it and no host audio.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_NVT_DAI "nvt-dai"
OBJECT_DECLARE_SIMPLE_TYPE(NvtDaiState, NVT_DAI)

#define DAI_REGS_SIZE       0x1000

#define DAI_CTRL            0x00
#define DAI_INT_EN0         0x20
#define DAI_INT_EN1         0x24
#define DAI_CH_EN_DONE      0x30
#define DAI_CH_EN_LOAD      0x34
#define DAI_CH_EN_C8        0x38
#define DAI_STS0            0x28
#define DAI_STS1            0x2C
#define DAI_CH_STS_DONE     0xC0
#define DAI_CH_STS_LOAD     0xC4
#define DAI_CH_STS_C8       0xC8

#define DAI_TX_ADDR(n)      (0x100 + 0x20 * (n))
#define DAI_TX_CUR(n)       (0x108 + 0x20 * (n))
#define DAI_TX_SIZE(n)      (0x110 + 0x20 * (n))
#define DAI_RX_ADDR(n)      (0x140 + 0x20 * (n))
#define DAI_RX_CUR(n)       (0x148 + 0x20 * (n))
#define DAI_RX_SIZE(n)      (0x150 + 0x20 * (n))
#define DAI_RXM_ADDR(k)     (0x200 + 0x20 * (k))
#define DAI_RXM_SIZE(k)     (0x210 + 0x20 * (k))
#define DAI_RXM_CHANNELS    21

#define CTRL_TX_DMA(n)      (1u << (4 + (n)))
#define CTRL_RX_DMA         (1u << 6)
#define CTRL_TX(n)          (1u << (12 + (n)))
#define CTRL_RX             (1u << 14)

#define STS0_TX_DONE(n)     (1u << (n))
#define STS0_RX_DONE        (1u << 2)

#define DAI_SIZE_MASK       0x7FFFF     /* words */
#define DAI_BYTES_PER_SEC   16000       /* 8 kHz, 16-bit, one channel */
#define DAI_MIN_PERIOD_NS   (1 * SCALE_MS)

struct NvtDaiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *tx_timer;
    QEMUTimer *rx_timer;
    uint32_t regs[DAI_REGS_SIZE / 4];
};

static uint32_t *reg(NvtDaiState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_dai_update_irq(NvtDaiState *s)
{
    bool level = (*reg(s, DAI_STS0) & *reg(s, DAI_INT_EN0)) ||
                 (*reg(s, DAI_STS1) & *reg(s, DAI_INT_EN1)) ||
                 (*reg(s, DAI_CH_STS_DONE) & *reg(s, DAI_CH_EN_DONE)) ||
                 (*reg(s, DAI_CH_STS_LOAD) & *reg(s, DAI_CH_EN_LOAD)) ||
                 (*reg(s, DAI_CH_STS_C8) & *reg(s, DAI_CH_EN_C8));

    qemu_set_irq(s->irq, level);
}

static uint32_t buf_bytes(NvtDaiState *s, hwaddr size_reg)
{
    return (*reg(s, size_reg) & DAI_SIZE_MASK) * 4;
}

/* The time one buffer of len bytes takes at the nominal rate. */
static int64_t buf_period_ns(uint32_t len)
{
    return MAX((int64_t)len * NANOSECONDS_PER_SECOND / DAI_BYTES_PER_SEC,
               DAI_MIN_PERIOD_NS);
}

/*
 * The RX multi-channel buffers that are set up: bit k for channel k.  The
 * single RX buffer, when it is the one in use, counts as channel 0.
 */
static uint32_t rx_channels(NvtDaiState *s)
{
    uint32_t mask = 0;
    int k;

    for (k = 0; k < DAI_RXM_CHANNELS; k++) {
        if (buf_bytes(s, DAI_RXM_SIZE(k))) {
            mask |= 1u << k;
        }
    }
    if (!mask && buf_bytes(s, DAI_RX_SIZE(0))) {
        mask = 1;
    }
    return mask;
}

static uint32_t tx_channels(NvtDaiState *s)
{
    uint32_t ctrl = *reg(s, DAI_CTRL), mask = 0;
    int n;

    for (n = 0; n < 2; n++) {
        if ((ctrl & CTRL_TX_DMA(n)) && buf_bytes(s, DAI_TX_SIZE(n))) {
            mask |= 1u << n;
        }
    }
    return mask;
}

/* Take the queued buffers: the "in use" address follows, load done rises. */
static void nvt_dai_rx_load(NvtDaiState *s)
{
    uint32_t mask = rx_channels(s);

    *reg(s, DAI_RX_CUR(0)) = *reg(s, DAI_RX_ADDR(0));
    *reg(s, DAI_RX_CUR(1)) = *reg(s, DAI_RX_ADDR(1));
    *reg(s, DAI_CH_STS_LOAD) |= mask ? mask : 1;
}

static void nvt_dai_tx_load(NvtDaiState *s)
{
    uint32_t mask = tx_channels(s);

    *reg(s, DAI_TX_CUR(0)) = *reg(s, DAI_TX_ADDR(0));
    *reg(s, DAI_TX_CUR(1)) = *reg(s, DAI_TX_ADDR(1));
    *reg(s, DAI_CH_STS_LOAD) |= mask << 1 ? mask << 1 : 2;
}

static void silence(hwaddr addr, uint32_t len)
{
    static const uint8_t zero[4096];

    while (len) {
        uint32_t n = MIN(len, sizeof(zero));

        dma_memory_write(&address_space_memory, addr, zero, n,
                         MEMTXATTRS_UNSPECIFIED);
        addr += n;
        len -= n;
    }
}

/*
 * The longest RX buffer set up: every active channel completes on the same
 * tick, so the tick is as long as the longest of them (the vendor queues
 * the same size on every channel).  0 when there is none.
 */
static uint32_t rx_len(NvtDaiState *s, uint32_t mask)
{
    uint32_t len = 0;
    int k;

    for (k = 0; k < DAI_RXM_CHANNELS; k++) {
        if (mask & (1u << k)) {
            len = MAX(len, buf_bytes(s, DAI_RXM_SIZE(k)));
        }
    }
    return len ? len : buf_bytes(s, DAI_RX_SIZE(0));
}

static bool rx_running(NvtDaiState *s)
{
    uint32_t ctrl = *reg(s, DAI_CTRL);

    return (ctrl & CTRL_RX_DMA) && (ctrl & CTRL_RX);
}

/* Start the RX clock: a running receiver with a buffer to fill. */
static void nvt_dai_rx_arm(NvtDaiState *s)
{
    uint32_t len = rx_len(s, rx_channels(s));

    if (rx_running(s) && len && !timer_pending(s->rx_timer)) {
        timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  buf_period_ns(len));
    }
}

static void nvt_dai_rx_tick(void *opaque)
{
    NvtDaiState *s = opaque;
    uint32_t mask = rx_channels(s), len = rx_len(s, mask);
    int k;

    /* No buffer, no completion: a later one restarts the clock. */
    if (!rx_running(s) || !len) {
        return;
    }
    for (k = 0; k < DAI_RXM_CHANNELS; k++) {
        if (mask & (1u << k)) {
            silence(*reg(s, DAI_RXM_ADDR(k)),
                    buf_bytes(s, DAI_RXM_SIZE(k)));
        }
    }
    if (mask == 1 && !buf_bytes(s, DAI_RXM_SIZE(0))) {
        silence(*reg(s, DAI_RX_ADDR(0)), buf_bytes(s, DAI_RX_SIZE(0)));
    }
    *reg(s, DAI_STS0) |= STS0_RX_DONE;
    *reg(s, DAI_CH_STS_DONE) |= mask;
    nvt_dai_rx_load(s);
    nvt_dai_update_irq(s);
    timer_mod(s->rx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              buf_period_ns(len));
}

static void nvt_dai_tx_tick(void *opaque)
{
    NvtDaiState *s = opaque;
    uint32_t mask = 0, len = 0;
    int n;

    for (n = 0; n < 2; n++) {
        if ((*reg(s, DAI_CTRL) & CTRL_TX(n)) &&
            (tx_channels(s) & (1u << n))) {
            mask |= STS0_TX_DONE(n);
            len = MAX(len, buf_bytes(s, DAI_TX_SIZE(n)));
        }
    }
    if (!mask) {
        return;
    }
    *reg(s, DAI_STS0) |= mask;
    nvt_dai_tx_load(s);
    nvt_dai_update_irq(s);
    timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              buf_period_ns(len));
}

/* Start the TX clock: a running transmitter with a buffer to play. */
static void nvt_dai_tx_arm(NvtDaiState *s)
{
    uint32_t len = 0;
    int n;

    for (n = 0; n < 2; n++) {
        if ((*reg(s, DAI_CTRL) & CTRL_TX(n)) &&
            (tx_channels(s) & (1u << n))) {
            len = MAX(len, buf_bytes(s, DAI_TX_SIZE(n)));
        }
    }
    if (len && !timer_pending(s->tx_timer)) {
        timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  buf_period_ns(len));
    }
}

static void nvt_dai_ctrl(NvtDaiState *s, uint32_t old, uint32_t val)
{
    uint32_t rise = val & ~old;
    int n;

    if (rise & CTRL_RX_DMA) {
        nvt_dai_rx_load(s);
    }
    for (n = 0; n < 2; n++) {
        if (rise & CTRL_TX_DMA(n)) {
            nvt_dai_tx_load(s);
        }
    }
    if (rx_running(s)) {
        nvt_dai_rx_arm(s);
    } else {
        timer_del(s->rx_timer);
    }
    if (val & (CTRL_TX(0) | CTRL_TX(1))) {
        nvt_dai_tx_arm(s);
    } else {
        timer_del(s->tx_timer);
    }
    nvt_dai_update_irq(s);
}

static uint64_t nvt_dai_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtDaiState *s = opaque;

    return *reg(s, offset & ~3);
}

static void nvt_dai_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    NvtDaiState *s = opaque;
    uint32_t val = value, old;

    offset &= ~3;
    old = *reg(s, offset);
    switch (offset) {
    case DAI_STS0:
    case DAI_STS1:
    case DAI_CH_STS_DONE:
    case DAI_CH_STS_LOAD:
    case DAI_CH_STS_C8:
        *reg(s, offset) = old & ~val;
        nvt_dai_update_irq(s);
        return;
    case DAI_TX_CUR(0):
    case DAI_TX_CUR(1):
    case DAI_RX_CUR(0):
    case DAI_RX_CUR(1):
        return;                             /* read-only */
    }
    *reg(s, offset) = val;
    switch (offset) {
    case DAI_CTRL:
        nvt_dai_ctrl(s, old, val);
        break;
    case DAI_TX_SIZE(0):
    case DAI_TX_SIZE(1):
        nvt_dai_tx_arm(s);
        break;
    case DAI_RX_SIZE(0):
    case DAI_RX_SIZE(1):
        nvt_dai_rx_arm(s);
        break;
    case DAI_INT_EN0:
    case DAI_INT_EN1:
    case DAI_CH_EN_DONE:
    case DAI_CH_EN_LOAD:
    case DAI_CH_EN_C8:
        nvt_dai_update_irq(s);
        break;
    default:
        if (offset >= DAI_RXM_SIZE(0) &&
            offset <= DAI_RXM_SIZE(DAI_RXM_CHANNELS - 1) &&
            (offset - DAI_RXM_SIZE(0)) % 0x20 == 0) {
            nvt_dai_rx_arm(s);
        }
        break;
    }
}

static const MemoryRegionOps nvt_dai_ops = {
    .read = nvt_dai_read,
    .write = nvt_dai_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_dai_reset(DeviceState *dev)
{
    NvtDaiState *s = NVT_DAI(dev);

    timer_del(s->rx_timer);
    timer_del(s->tx_timer);
    memset(s->regs, 0, sizeof(s->regs));
    qemu_irq_lower(s->irq);
}

static void nvt_dai_init(Object *obj)
{
    NvtDaiState *s = NVT_DAI(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_dai_ops, s, TYPE_NVT_DAI,
                          DAI_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_dai_rx_tick, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_dai_tx_tick, s);
}

static const VMStateDescription vmstate_nvt_dai = {
    .name = TYPE_NVT_DAI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtDaiState, DAI_REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(rx_timer, NvtDaiState),
        VMSTATE_TIMER_PTR(tx_timer, NvtDaiState),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_dai_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_dai_reset);
    dc->vmsd = &vmstate_nvt_dai;
}

static const TypeInfo nvt_dai_info = {
    .name           = TYPE_NVT_DAI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(NvtDaiState),
    .instance_init  = nvt_dai_init,
    .class_init     = nvt_dai_class_init,
};

static void nvt_dai_register_types(void)
{
    type_register_static(&nvt_dai_info);
}

type_init(nvt_dai_register_types)
