/*
 * MStar infinity3 scaler (SCL): its frame interrupts and the DMA clients
 * that write pictures to DRAM.
 *
 * Read off the vendor kernel's drivers/mstar/scl/infinity3 (src/mxlib/
 * drv/sclirq/drvsclirq.c, drv/scldma/drvscldma.c, hal/scldma/
 * halscldma.c; the I3 SDK MSC3XX_SDK_V_1711291828).  In silicon the ISP
 * streams into the scaler, which crops, scales and writes YUV through
 * up to five DMA clients; here the ISP model's frame start and frame end
 * (GPIO in 0 and 1) clock it, and each client gets the ISP's last picture
 * (the "isp" link, sstar_i3_isp_frame(): Y, Cb and Cr at half the ISP's
 * crop) resampled to the client's size, nearest pixel, in its format.
 * The scaler's own crop, filters and VIP stages are not modelled: the
 * camera application leaves them neutral (see sstar-i3-isp.c).  While
 * the ISP makes no picture (no sensor, the ISP off: a bare kernel, qtest)
 * the clients get the test pattern of sstar-pattern.h instead, the
 * other targets' picture: the OpenIPC logo, white on a navy gradient,
 * with a dot sliding beneath it.
 *
 * SCL0, bank 0x1218 (mapped here; the rest of the bank is storage):
 *   0x08-0x0A interrupt mask, bits 0-47 (1 = masked); 0x10-0x12 clear,
 *   held: while a bit is 1 its status stays 0 (the ISR writes 1, then 0);
 *   0x13-0x15 status.  Events latch whatever the mask (the ISR acts on
 *   every status bit it reads); the line is status & ~mask, GIC SPI 52.
 *   Bits used: 4 VSYNC_IDCLK (input frame start), 0 SC_IN_FRM_END
 *   (input frame end), 2 SC1_ENG_FRM_END (the scaler's frame end, on
 *   which the driver hands buffers out), and each client's write ACTIVE /
 *   ACTIVE_N pair (below).
 *
 * The clients' registers stay in the RIU's storage (DMA0 bank 0x1213,
 * DMA1 0x1214, SCL3 0x1225): Y and C bases per ring buffer (MIU address
 * >> 3, two registers each), the ring size - 1 in [4:3], width, height,
 * the 4:2:0 bit (bit 3), the enable (bit 15) and 4:2:2 packed (bit 14)
 * bits, and the data mode in SCL3.  DMA2 (bank 0x1215) reg 0x03 holds
 * each client's ring index, which the hardware advances at each frame a
 * client is enabled for and the ISR reads at ACTIVE.
 *
 * Per frame, at the frame start: bit 4 and bit 35 (VSYNC_FCLK_LDC, the
 * scaler side's vsync: masked, but the ISR shifts its frame-in history
 * on it, and with the LDC path on, SCL_DELAYFRAME 1, a frame counts as
 * in only after that shift), and for each enabled client its index
 * steps and ACTIVE rises; at the frame end the picture is written
 * into the buffer at that index, then bits 0, the clients' ACTIVE_N and
 * 2 rise together.  The driver turns a client on during blanking (after
 * bit 2) or from the ISR, so the frame events run whether or not any
 * client is enabled.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/sstar-riu.h"
#include "aw-vinc-logo.h"
#include "hw/misc/sstar-pattern.h"
#include "hw/misc/sstar-i3-isp.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_SSTAR_I3_SCL "sstar-i3-scl"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI3SclState, SSTAR_I3_SCL)

#define SCL0_REGS           0x80
#define R_MASK              0x08
#define R_CLEAR             0x10
#define R_STATUS            0x13

#define BANK_DMA0           0x1213
#define BANK_DMA1           0x1214
#define BANK_DMA2           0x1215
#define BANK_SCL3           0x1225
#define DMA2_RW_IDX         0x03

#define IRQ_IN_FRM_END      0
#define IRQ_ENG_FRM_END     2
#define IRQ_VSYNC_IDCLK     4
#define IRQ_VSYNC_FCLK_LDC  35

#define DRAM_BASE           0x20000000
#define MAX_W               2688
#define MAX_H               2048

/*
 * One DMA write client: its bank, the register of its first Y base
 * (buffers at +0, +2, +4, +6, C bases at +8..), ring size (+0x10), width
 * and height (+0x12, +0x13), the 4:2:0 register (+0x16), its enable
 * register, its data mode in SCL3, its ACTIVE and ACTIVE_N interrupt
 * bits (drvsclirq_st.h SCLIRQ_MSK_*: FRM2's pair is the other way round,
 * 37 then 36) and its index field in DMA2 0x03.
 */
typedef struct I3SclClient {
    const char *name;
    uint16_t bank;
    uint8_t base;
    uint8_t en_reg;
    uint8_t mode_reg;
    uint8_t active_bit;
    uint8_t done_bit;
    uint8_t idx_shift;
} I3SclClient;

static const I3SclClient i3_scl_clients[] = {
    { "sc1-frm",  BANK_DMA0, 0x08, 0x01, 0x20, 7,  8,  0 },
    { "sc1-snp",  BANK_DMA0, 0x38, 0x03, 0x22, 9,  10, 4 },
    { "sc2-frm",  BANK_DMA0, 0x50, 0x04, 0x24, 23, 24, 6 },
    { "sc2-frm2", BANK_DMA1, 0x50, 0x04, 0x2E, 37, 36, 14 },
    { "sc3-w",    BANK_DMA1, 0x08, 0x01, 0x28, 33, 34, 12 },
};

#define N_CLIENTS           ARRAY_SIZE(i3_scl_clients)

struct SstarI3SclState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    SstarRiuState *riu;
    SstarI3IspState *isp;

    uint16_t regs[SCL0_REGS];
    uint64_t status;
    uint32_t frames;
    uint8_t active;                 /* clients written this frame */
    uint8_t enabled;                /* clients enabled last frame */
    bool trace;
};

static uint64_t i3_scl_get48(SstarI3SclState *s, int reg)
{
    return s->regs[reg] | (uint64_t)s->regs[reg + 1] << 16 |
           (uint64_t)s->regs[reg + 2] << 32;
}

static void i3_scl_update(SstarI3SclState *s)
{
    s->status &= ~i3_scl_get48(s, R_CLEAR);
    qemu_set_irq(s->irq, !!(s->status & ~i3_scl_get48(s, R_MASK)));
}

static void i3_scl_raise(SstarI3SclState *s, uint64_t bits)
{
    s->status |= bits;
    i3_scl_update(s);
}

static uint16_t i3_scl_dma(SstarI3SclState *s, const I3SclClient *c,
                           int off)
{
    return sstar_riu_get(s->riu, c->bank, c->base + off);
}

static hwaddr i3_scl_addr(SstarI3SclState *s, const I3SclClient *c, int off)
{
    uint32_t v = i3_scl_dma(s, c, off) |
                 (uint32_t)i3_scl_dma(s, c, off + 1) << 16;

    return DRAM_BASE + ((hwaddr)v << 3);
}

/*
 * A client's picture: the ISP's frame resampled to the client's size
 * (nearest pixel), or without one the test pattern.  @px gets Y, Cb, Cr.
 */
typedef struct I3SclSrc {
    SstarI3SclState *s;
    const uint8_t *y, *cb, *cr;
    int w, h;
    uint16_t *xmap;
    int row;                        /* source row offset for this line */
} I3SclSrc;

static inline void i3_scl_px(I3SclSrc *src, int w, int h, int x, int y,
                             uint8_t *px)
{
    int o;

    if (!src->y) {
        sstar_pattern_pixel(src->s->frames, w, h, x, y, px);
        return;
    }
    o = src->row + src->xmap[x];
    px[0] = src->y[o];
    px[1] = src->cb[o];
    px[2] = src->cr[o];
}

static void i3_scl_draw(SstarI3SclState *s, const I3SclClient *c, int idx)
{
    uint16_t w = i3_scl_dma(s, c, 0x12) & 0x1FFF;
    uint16_t h = i3_scl_dma(s, c, 0x13) & 0x1FFF;
    bool yuv420 = i3_scl_dma(s, c, 0x16) & 0x8;
    bool packed = sstar_riu_get(s->riu, c->bank, c->en_reg) & 0x4000;
    uint16_t mode = sstar_riu_get(s->riu, BANK_SCL3, c->mode_reg) & 7;
    hwaddr ya = i3_scl_addr(s, c, 2 * idx);
    hwaddr ca = i3_scl_addr(s, c, 8 + 2 * idx);
    g_autofree uint8_t *yl = NULL, *cl = NULL;
    g_autofree uint16_t *xmap = NULL;
    I3SclSrc src = { .s = s };
    uint8_t px[3];
    int x, y;

    if (!w || !h || w > MAX_W || h > MAX_H) {
        return;
    }
    if (s->isp && sstar_i3_isp_frame(s->isp, &src.y, &src.cb, &src.cr,
                                     &src.w, &src.h)) {
        xmap = g_new(uint16_t, w + 1);
        for (x = 0; x <= w; x++) {
            xmap[x] = MIN((2 * x + 1) * src.w / (2 * w), src.w - 1);
        }
        src.xmap = xmap;
    }
    yl = g_malloc(2 * w + 2);
    cl = g_malloc(w + 1);
    for (y = 0; y < h; y++) {
        if (src.y) {
            src.row = MIN((2 * y + 1) * src.h / (2 * h), src.h - 1) * src.w;
        }
        if (packed) {
            /* YUYV, one plane */
            for (x = 0; x < w; x += 2) {
                i3_scl_px(&src, w, h, x, y, px);
                yl[2 * x] = px[0];
                yl[2 * x + 1] = px[1];
                i3_scl_px(&src, w, h, x + 1, y, px);
                yl[2 * x + 2] = px[0];
                yl[2 * x + 3] = px[2];
            }
            address_space_write(&address_space_memory, ya + 2 * w * y,
                                MEMTXATTRS_UNSPECIFIED, yl, 2 * w);
            continue;
        }
        for (x = 0; x < w; x++) {
            i3_scl_px(&src, w, h, x, y, px);
            yl[x] = px[0];
            if (!(x & 1)) {
                cl[x] = px[1];
                cl[x + 1 < w ? x + 1 : x] = px[2];
            }
        }
        address_space_write(&address_space_memory, ya + w * y,
                            MEMTXATTRS_UNSPECIFIED, yl, w);
        /* Y + interleaved CbCr: 4:2:0 every other line, else every line */
        if (yuv420 && !(y & 1)) {
            address_space_write(&address_space_memory, ca + w * (y / 2),
                                MEMTXATTRS_UNSPECIFIED, cl, w);
        } else if (!yuv420 && (mode == 1 || mode == 0)) {
            address_space_write(&address_space_memory, ca + w * y,
                                MEMTXATTRS_UNSPECIFIED, cl, w);
        }
    }
    if (s->trace) {
        qemu_log("sstar-i3-scl: %s frame %u buf %d %ux%u %s at 0x%"
                 HWADDR_PRIx "/0x%" HWADDR_PRIx " from %s\n", c->name,
                 s->frames, idx, w, h,
                 packed ? "yuyv" : yuv420 ? "420" : "422", ya, ca,
                 src.y ? "the ISP" : "the pattern");
    }
}

static void i3_scl_frame_start(SstarI3SclState *s)
{
    uint16_t rw = sstar_riu_get(s->riu, BANK_DMA2, DMA2_RW_IDX);
    uint64_t bits = 1ull << IRQ_VSYNC_IDCLK | 1ull << IRQ_VSYNC_FCLK_LDC;
    unsigned i;

    s->active = 0;
    for (i = 0; i < N_CLIENTS; i++) {
        const I3SclClient *c = &i3_scl_clients[i];
        int max = (i3_scl_dma(s, c, 0x10) >> 3) & 3;
        int idx = (rw >> c->idx_shift) & 3;

        if (!(sstar_riu_get(s->riu, c->bank, c->en_reg) & 0x8000)) {
            continue;
        }
        /* a client just switched on starts its ring at buffer 0 */
        idx = !(s->enabled & (1u << i)) || idx >= max ? 0 : idx + 1;
        rw = (rw & ~(3u << c->idx_shift)) | idx << c->idx_shift;
        s->active |= 1u << i;
        bits |= 1ull << c->active_bit;
    }
    s->enabled = s->active;
    sstar_riu_set(s->riu, BANK_DMA2, DMA2_RW_IDX, rw);
    i3_scl_raise(s, bits);
}

static void i3_scl_frame_end(SstarI3SclState *s)
{
    uint16_t rw = sstar_riu_get(s->riu, BANK_DMA2, DMA2_RW_IDX);
    uint64_t bits = 1ull << IRQ_IN_FRM_END | 1ull << IRQ_ENG_FRM_END;
    unsigned i;

    for (i = 0; i < N_CLIENTS; i++) {
        const I3SclClient *c = &i3_scl_clients[i];

        if (!(s->active & (1u << i))) {
            continue;
        }
        i3_scl_draw(s, c, (rw >> c->idx_shift) & 3);
        bits |= 1ull << c->done_bit;
    }
    s->active = 0;
    s->frames++;
    i3_scl_raise(s, bits);
}

static void i3_scl_frame_in(void *opaque, int n, int level)
{
    SstarI3SclState *s = opaque;

    if (!level) {
        return;
    }
    if (n == 0) {
        i3_scl_frame_start(s);
    } else {
        i3_scl_frame_end(s);
    }
}

static uint64_t i3_scl_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI3SclState *s = opaque;
    int reg = (off >> 2) & (SCL0_REGS - 1);
    uint16_t v = s->regs[reg];

    if ((off & 3) >= 2) {
        return 0;
    }
    if (reg >= R_STATUS && reg < R_STATUS + 3) {
        v = s->status >> (16 * (reg - R_STATUS));
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void i3_scl_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    SstarI3SclState *s = opaque;
    int reg = (off >> 2) & (SCL0_REGS - 1);
    uint16_t v = val;

    if ((off & 3) >= 2 || (reg >= R_STATUS && reg < R_STATUS + 3)) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    s->regs[reg] = v;
    i3_scl_update(s);
}

static const MemoryRegionOps i3_scl_ops = {
    .read = i3_scl_read,
    .write = i3_scl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_i3_scl_reset(DeviceState *dev)
{
    SstarI3SclState *s = SSTAR_I3_SCL(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_MASK] = s->regs[R_MASK + 1] = s->regs[R_MASK + 2] = 0xFFFF;
    s->status = 0;
    s->active = 0;
    s->enabled = 0;
    qemu_set_irq(s->irq, 0);
}

static void sstar_i3_scl_realize(DeviceState *dev, Error **errp)
{
    SstarI3SclState *s = SSTAR_I3_SCL(dev);

    if (!s->riu) {
        error_setg(errp, "sstar-i3-scl: the riu link is not set");
    }
}

static void sstar_i3_scl_init(Object *obj)
{
    SstarI3SclState *s = SSTAR_I3_SCL(obj);

    memory_region_init_io(&s->iomem, obj, &i3_scl_ops, s,
                          TYPE_SSTAR_I3_SCL, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in(DEVICE(obj), i3_scl_frame_in, 2);
}

static const Property sstar_i3_scl_props[] = {
    DEFINE_PROP_LINK("riu", SstarI3SclState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
    DEFINE_PROP_LINK("isp", SstarI3SclState, isp, TYPE_SSTAR_I3_ISP,
                     SstarI3IspState *),
    DEFINE_PROP_BOOL("trace", SstarI3SclState, trace, false),
};

static const VMStateDescription vmstate_sstar_i3_scl = {
    .name = TYPE_SSTAR_I3_SCL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarI3SclState, SCL0_REGS),
        VMSTATE_UINT64(status, SstarI3SclState),
        VMSTATE_UINT32(frames, SstarI3SclState),
        VMSTATE_UINT8(active, SstarI3SclState),
        VMSTATE_UINT8(enabled, SstarI3SclState),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_i3_scl_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sstar_i3_scl_realize;
    device_class_set_legacy_reset(dc, sstar_i3_scl_reset);
    device_class_set_props(dc, sstar_i3_scl_props);
    dc->vmsd = &vmstate_sstar_i3_scl;
}

static const TypeInfo sstar_i3_scl_info = {
    .name           = TYPE_SSTAR_I3_SCL,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarI3SclState),
    .instance_init  = sstar_i3_scl_init,
    .class_init     = sstar_i3_scl_class_init,
};

static void sstar_i3_scl_register_types(void)
{
    type_register_static(&sstar_i3_scl_info);
}

type_init(sstar_i3_scl_register_types)
