/*
 * Faraday FTDMAC030 AXI DMA controller, as on Novatek's NA51068
 * (NT98321 / NT98323) at 0xFCA00000 (SPI 14): drivers/dma/ftdmac030.c,
 * plat-na51068/ftdmac030.h.  Eight channels; one interrupt.
 *
 *   0x00 ISR       TCISR | EAISR, per channel
 *   0x04 TCISR     terminal count, unmasked channels (CFG bit 0 masks)
 *   0x08 TCICR     write 1 to clear TC
 *   0x0C EAISR     error / abort ([7:0] error, [23:16] abort)
 *   0x10 EAICR     write 1 to clear
 *   0x14 TCRAW     0x18 EARAW      before the channels' masks
 *   0x1C CH_ENABLED   the channels running
 *   0x34 REVISION  0x38 FEATURE    0x50 WO_VALUE  the memset word
 *   0x100 + 0x20 n  channel n: CTRL, CFG, SRC, DST, LLP, CYC, STRIDE
 *
 * CTRL bit 16 starts the channel.  [24:22] / [27:25] are the destination
 * / source widths (1 << n bytes), [19:18] / [21:20] 2 = fixed address,
 * bit 28 suppresses TC for this block, bit 14 2D mode (CYC [15:0] x
 * [31:16] lines, STRIDE source [15:0] / destination [31:16]).  CYC [21:0]
 * is the count of source-width units.  CFG bit 30 (WO) writes WO_VALUE
 * instead of reading the source.  LLP, when non-zero, points at the next
 * block's 24-byte descriptor: SRC, DST, LLP, CTRL, CYC, STRIDE.  The
 * driver chains a memcpy / memset / scatter-gather this way and masks TC
 * on every block but the last.
 *
 * A channel runs to the end of its chain as soon as it is started, unless
 * its CFG enables a handshake (bit 7 source, number [6:3]; bit 13
 * destination, number [12:9]): then it waits for that "drq" input to rise
 * -- the peripheral's request, e.g. the SPI020's once its command has data
 * -- and moves the whole chain then.  A bus error ends the chain with the
 * channel's error bit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_NVT_FTDMAC030 "nvt-ftdmac030"
OBJECT_DECLARE_SIMPLE_TYPE(NvtFtdmacState, NVT_FTDMAC030)

#define FTDMAC_CHANNELS     8
#define FTDMAC_ISR          0x00
#define FTDMAC_TCISR        0x04
#define FTDMAC_TCICR        0x08
#define FTDMAC_EAISR        0x0C
#define FTDMAC_EAICR        0x10
#define FTDMAC_TCRAW        0x14
#define FTDMAC_EARAW        0x18
#define FTDMAC_CH_ENABLED   0x1C
#define FTDMAC_REVISION     0x34
#define FTDMAC_FEATURE      0x38
#define FTDMAC_WO_VALUE     0x50
#define FTDMAC_GLOBAL_REGS  (0x100 / 4)
#define FTDMAC_CH_BASE      0x100
#define FTDMAC_CH_STRIDE    0x20

enum { CH_CTRL, CH_CFG, CH_SRC, CH_DST, CH_LLP, CH_CYC, CH_STRIDE, CH_REGS };

#define CTRL_2D             (1u << 14)
#define CTRL_ENABLE         (1u << 16)
#define CTRL_DST_FIXED(c)   ((((c) >> 18) & 3) == 2)
#define CTRL_SRC_FIXED(c)   ((((c) >> 20) & 3) == 2)
#define CTRL_DST_WIDTH(c)   (1u << MIN(((c) >> 22) & 7, 4))
#define CTRL_SRC_WIDTH(c)   (1u << MIN(((c) >> 25) & 7, 4))
#define CTRL_MASK_TC        (1u << 28)

#define CFG_MASK_TCI        (1u << 0)
#define CFG_MASK_EI         (1u << 1)
#define CFG_MASK_AI         (1u << 2)
#define CFG_WO_EN           (1u << 30)
#define CFG_SRC_HS_EN       (1u << 7)
#define CFG_DST_HS_EN       (1u << 13)
#define FTDMAC_DRQS         16

#define CYC_TOTAL(x)        ((x) & 0x3FFFFF)
#define EA_ERR(n)           (1u << (n))
#define EA_ABT(n)           (1u << ((n) + 16))

#define FTDMAC_REV          0x00010300
#define FTDMAC_FEAT         0x00008108  /* 8 channels */
#define FTDMAC_MAX_BLOCKS   65536       /* a looping chain stops here */

struct NvtFtdmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    AddressSpace *as;
    uint32_t regs[FTDMAC_GLOBAL_REGS];
    uint32_t ch[FTDMAC_CHANNELS][CH_REGS];
    uint32_t tc_raw;
    uint32_t ea_raw;
};

static uint32_t nvt_ftdmac_tcisr(NvtFtdmacState *s)
{
    uint32_t v = 0;
    int n;

    for (n = 0; n < FTDMAC_CHANNELS; n++) {
        if ((s->tc_raw & (1u << n)) && !(s->ch[n][CH_CFG] & CFG_MASK_TCI)) {
            v |= 1u << n;
        }
    }
    return v;
}

static uint32_t nvt_ftdmac_eaisr(NvtFtdmacState *s)
{
    uint32_t v = 0;
    int n;

    for (n = 0; n < FTDMAC_CHANNELS; n++) {
        if ((s->ea_raw & EA_ERR(n)) && !(s->ch[n][CH_CFG] & CFG_MASK_EI)) {
            v |= EA_ERR(n);
        }
        if ((s->ea_raw & EA_ABT(n)) && !(s->ch[n][CH_CFG] & CFG_MASK_AI)) {
            v |= EA_ABT(n);
        }
    }
    return v;
}

static void nvt_ftdmac_update(NvtFtdmacState *s)
{
    qemu_set_irq(s->irq, nvt_ftdmac_tcisr(s) || nvt_ftdmac_eaisr(s));
}

/* One block of channel n: false on a bus error. */
static bool nvt_ftdmac_block(NvtFtdmacState *s, int n)
{
    uint32_t *c = s->ch[n];
    uint32_t ctrl = c[CH_CTRL];
    uint32_t sw = CTRL_SRC_WIDTH(ctrl), dw = CTRL_DST_WIDTH(ctrl);
    uint32_t cyc = c[CH_CYC], lines = 1, units = CYC_TOTAL(cyc);
    uint32_t sstride = 0, dstride = 0;
    bool wo = c[CH_CFG] & CFG_WO_EN;
    uint64_t total, done, chunk;
    dma_addr_t src = c[CH_SRC], dst = c[CH_DST];
    uint8_t buf[4096];
    uint32_t l, i;

    if (ctrl & CTRL_2D) {
        units = cyc & 0xFFFF;
        lines = cyc >> 16;
        sstride = c[CH_STRIDE] & 0xFFFF;
        dstride = c[CH_STRIDE] >> 16;
    }
    for (l = 0; l < lines; l++) {
        dma_addr_t sp = src + (CTRL_SRC_FIXED(ctrl) ? 0 : l * sstride);
        dma_addr_t dp = dst + (CTRL_DST_FIXED(ctrl) ? 0 : l * dstride);

        total = (uint64_t)units * sw;
        for (done = 0; done < total; done += chunk) {
            chunk = MIN(total - done, sizeof(buf));
            if (CTRL_SRC_FIXED(ctrl) || CTRL_DST_FIXED(ctrl)) {
                chunk = MIN(chunk, MAX(sw, dw));
            }
            if (wo) {
                uint32_t v = s->regs[FTDMAC_WO_VALUE / 4];

                for (i = 0; i < chunk; i++) {
                    buf[i] = v >> (8 * (i & 3));
                }
            } else if (dma_memory_read(s->as, sp, buf, chunk,
                                       MEMTXATTRS_UNSPECIFIED)) {
                return false;
            }
            if (dma_memory_write(s->as, dp, buf, chunk,
                                 MEMTXATTRS_UNSPECIFIED)) {
                return false;
            }
            if (!CTRL_SRC_FIXED(ctrl)) {
                sp += chunk;
            }
            if (!CTRL_DST_FIXED(ctrl)) {
                dp += chunk;
            }
        }
    }
    return true;
}

static void nvt_ftdmac_run(NvtFtdmacState *s, int n)
{
    uint32_t *c = s->ch[n];
    uint32_t lld[6];
    int blocks;

    for (blocks = 0; blocks < FTDMAC_MAX_BLOCKS; blocks++) {
        if (!nvt_ftdmac_block(s, n)) {
            s->ea_raw |= EA_ERR(n);
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftdmac030: channel %d bus "
                          "error (src 0x%x dst 0x%x)\n", n, c[CH_SRC],
                          c[CH_DST]);
            break;
        }
        if (!(c[CH_CTRL] & CTRL_MASK_TC)) {
            s->tc_raw |= 1u << n;
        }
        if (!c[CH_LLP]) {
            break;
        }
        if (dma_memory_read(s->as, c[CH_LLP], lld, sizeof(lld),
                            MEMTXATTRS_UNSPECIFIED)) {
            s->ea_raw |= EA_ERR(n);
            break;
        }
        c[CH_SRC] = le32_to_cpu(lld[0]);
        c[CH_DST] = le32_to_cpu(lld[1]);
        c[CH_LLP] = le32_to_cpu(lld[2]);
        c[CH_CTRL] = le32_to_cpu(lld[3]) | CTRL_ENABLE;
        c[CH_CYC] = le32_to_cpu(lld[4]);
        c[CH_STRIDE] = le32_to_cpu(lld[5]);
    }
    c[CH_CTRL] &= ~CTRL_ENABLE;
}

/* The handshake a channel waits on, or -1 */
static int nvt_ftdmac_drq_of(NvtFtdmacState *s, int n)
{
    uint32_t cfg = s->ch[n][CH_CFG];

    if (cfg & CFG_SRC_HS_EN) {
        return (cfg >> 3) & 0xF;
    }
    if (cfg & CFG_DST_HS_EN) {
        return (cfg >> 9) & 0xF;
    }
    return -1;
}

static void nvt_ftdmac_drq(void *opaque, int line, int level)
{
    NvtFtdmacState *s = opaque;
    int n;

    if (!level) {
        return;
    }
    for (n = 0; n < FTDMAC_CHANNELS; n++) {
        if ((s->ch[n][CH_CTRL] & CTRL_ENABLE) &&
            nvt_ftdmac_drq_of(s, n) == line) {
            nvt_ftdmac_run(s, n);
        }
    }
    nvt_ftdmac_update(s);
}

static uint64_t nvt_ftdmac_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtFtdmacState *s = opaque;
    uint32_t en = 0;
    int n;

    if (offset >= FTDMAC_CH_BASE &&
        offset < FTDMAC_CH_BASE + FTDMAC_CHANNELS * FTDMAC_CH_STRIDE) {
        hwaddr o = offset - FTDMAC_CH_BASE;
        unsigned r = (o % FTDMAC_CH_STRIDE) / 4;

        return r < CH_REGS ? s->ch[o / FTDMAC_CH_STRIDE][r] : 0;
    }
    switch (offset) {
    case FTDMAC_ISR:
        return nvt_ftdmac_tcisr(s) | (nvt_ftdmac_eaisr(s) & 0xFF) |
               ((nvt_ftdmac_eaisr(s) >> 16) & 0xFF);
    case FTDMAC_TCISR:
        return nvt_ftdmac_tcisr(s);
    case FTDMAC_EAISR:
        return nvt_ftdmac_eaisr(s);
    case FTDMAC_TCRAW:
        return s->tc_raw;
    case FTDMAC_EARAW:
        return s->ea_raw;
    case FTDMAC_CH_ENABLED:
        for (n = 0; n < FTDMAC_CHANNELS; n++) {
            if (s->ch[n][CH_CTRL] & CTRL_ENABLE) {
                en |= 1u << n;
            }
        }
        return en;
    case FTDMAC_TCICR:
    case FTDMAC_EAICR:
        return 0;
    case FTDMAC_REVISION:
        return FTDMAC_REV;
    case FTDMAC_FEATURE:
        return FTDMAC_FEAT;
    default:
        if (offset < FTDMAC_CH_BASE) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftdmac030: read 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void nvt_ftdmac_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    NvtFtdmacState *s = opaque;

    if (offset >= FTDMAC_CH_BASE &&
        offset < FTDMAC_CH_BASE + FTDMAC_CHANNELS * FTDMAC_CH_STRIDE) {
        hwaddr o = offset - FTDMAC_CH_BASE;
        int n = o / FTDMAC_CH_STRIDE;
        unsigned r = (o % FTDMAC_CH_STRIDE) / 4;
        bool was = s->ch[n][CH_CTRL] & CTRL_ENABLE;

        if (r >= CH_REGS) {
            return;
        }
        s->ch[n][r] = val;
        if (r == CH_CTRL && (val & CTRL_ENABLE) && !was &&
            nvt_ftdmac_drq_of(s, n) < 0) {
            nvt_ftdmac_run(s, n);   /* else it waits for its request */
        } else if (r == CH_CTRL && was && !(val & CTRL_ENABLE)) {
            s->ea_raw |= EA_ABT(n);     /* stopped while running */
        }
        nvt_ftdmac_update(s);
        return;
    }
    switch (offset) {
    case FTDMAC_TCICR:
        s->tc_raw &= ~val;
        break;
    case FTDMAC_EAICR:
        s->ea_raw &= ~val;
        break;
    case FTDMAC_ISR:
    case FTDMAC_TCISR:
    case FTDMAC_EAISR:
    case FTDMAC_TCRAW:
    case FTDMAC_EARAW:
    case FTDMAC_CH_ENABLED:
    case FTDMAC_REVISION:
    case FTDMAC_FEATURE:
        return;
    default:
        if (offset < FTDMAC_CH_BASE) {
            s->regs[offset / 4] = val;
            break;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftdmac030: write 0x%"
                      HWADDR_PRIx "\n", offset);
        return;
    }
    nvt_ftdmac_update(s);
}

static const MemoryRegionOps nvt_ftdmac_ops = {
    .read = nvt_ftdmac_read,
    .write = nvt_ftdmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_ftdmac_reset(DeviceState *dev)
{
    NvtFtdmacState *s = NVT_FTDMAC030(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ch, 0, sizeof(s->ch));
    s->tc_raw = s->ea_raw = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_ftdmac_init(Object *obj)
{
    NvtFtdmacState *s = NVT_FTDMAC030(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ftdmac_ops, s,
                          TYPE_NVT_FTDMAC030, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->as = &address_space_memory;
    qdev_init_gpio_in_named(DEVICE(obj), nvt_ftdmac_drq, "drq", FTDMAC_DRQS);
}

static const VMStateDescription vmstate_nvt_ftdmac = {
    .name = TYPE_NVT_FTDMAC030,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtFtdmacState, FTDMAC_GLOBAL_REGS),
        VMSTATE_UINT32_2DARRAY(ch, NvtFtdmacState, FTDMAC_CHANNELS, CH_REGS),
        VMSTATE_UINT32(tc_raw, NvtFtdmacState),
        VMSTATE_UINT32(ea_raw, NvtFtdmacState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_ftdmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_ftdmac_reset);
    dc->vmsd = &vmstate_nvt_ftdmac;
}

static const TypeInfo nvt_ftdmac_info = {
    .name          = TYPE_NVT_FTDMAC030,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtFtdmacState),
    .instance_init = nvt_ftdmac_init,
    .class_init    = nvt_ftdmac_class_init,
};

static void nvt_ftdmac_register_types(void)
{
    type_register_static(&nvt_ftdmac_info);
}

type_init(nvt_ftdmac_register_types)
