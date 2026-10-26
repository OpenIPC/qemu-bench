/*
 * Grain Media FTMCP100 MPEG-4 / JPEG codec: control and the MJPEG encoder.
 *
 * The codec's registers sit at 0x92020000 on the GM813x, above its local
 * memory at 0x92000000 (the SRAM nsboot also runs in).  fmcp_drv.ko
 * (module/ftmcp100/fmcp.c fmcp100_resetrun()) resets the block through
 * 0x2040C bit 27 and polls it clear, stops the codec's embedded CPU
 * (INCTL 0x2007C = 1 << 17), copies the CPU's firmware into the local
 * memory, starts it (3 << 18) and polls INCTL bit 23 for the CPU running.
 * The model has no embedded CPU: INCTL reads it running, the reset
 * completes at once, and it does the firmware's part of the jobs itself.
 *
 * MJPEG (fmjpeg_drv.ko, the "embedded" build: jpeg_enc.c, jccoefct.c,
 * jcmarker.c) goes through the firmware's mailbox, the Share_Node_JE at
 * local 0x8000, which the model maps over the SRAM:
 *
 *   +0x00 command (byte): the host writes 8 to encode, the firmware 0x80
 *         when done (and 0 when it starts)
 *   +0x04 bs_phy, +0x08 image_width, +0x0C restart_interval (MCUs)
 *   +0x20 u82D: 4 packed UYVY through the DMA wrapper, 1 planar I420
 *   +0x24-0x30 the ROI's left / top / right / bottom, in MCUs
 *   +0x38-0x40 the I420 planes, +0x44 max_bs_thd, +0x48 / 0x4C stop x / y
 *   +0xD4 bitslen, +0xD8 jpgenc_done
 *
 * The host writes the headers itself (SOI, APP0, DQT, SOF0, DHT, DRI,
 * SOS) through the bit packer: BADR 0x2004C the value, BALR 0x20050 the
 * bit count, appended MSB-first at ABADR 0x20078 (= bs_phy | 1); it polls
 * VLDSTS 0x20074 bit 10 between segments.  On the command the model
 * encodes the picture -- 4:2:0, 16x16 MCUs, the quantisation tables of
 * the DQTs it was given, the standard Huffman tables, RSTn every
 * restart_interval MCUs -- right after the headers, ends it with EOI,
 * fills in the node and raises INT_FLAG 0x20098 bit 31 on interrupt 31
 * (the host writes the value back to clear it).  A scan that would pass
 * max_bs_thd is cut short with EOI and stop_x / stop_y, as the firmware
 * does, so the host retries at a lower quality.
 *
 * Packed input comes through the DMA wrapper at 0x92400000 (the second
 * region): +0x08 / +0x0C the ROI's width / height in MCUs, +0x18 the word
 * address of its first pixel, rows image_width * 2 bytes apart; chroma
 * line pairs are averaged to 4:2:0.  MPEG-4 is not built into the
 * cameras' kernels and its commands are not executed.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/nvt-pixel.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_MCP100 "gm-mcp100"
OBJECT_DECLARE_SIMPLE_TYPE(GmMcp100State, GM_MCP100)

#define GM_MCP100_SIZE      0x1000
#define GM_MCP100_NREGS     (GM_MCP100_SIZE / 4)
#define GM_MCP100_NODE_SIZE 0x1000
#define GM_MCP100_WRP_SIZE  0x1000
#define GM_MCP100_WRP_NREGS (0x40 / 4)
#define GM_MCP100_HDR_MAX   2048
#define GM_MCP100_JOB_NS    5000000
#define GM_MCP100_MAX_MCUS  256         /* per side: 4096 pixels */

#define R_BADR              0x04C
#define R_BALR              0x050
#define R_VLDSTS            0x074
#define R_ABADR             0x078
#define R_INCTL             0x07C
#define R_INT_FLAG          0x098
#define R_RESET             0x40C

#define INCTL_CPU_STOP      (1u << 17)
#define INCTL_CPU_START     (3u << 18)
#define INCTL_CPU_IDLE      (1u << 23)
#define VLDSTS_READY        (1u << 10)
#define INT_JPEG_DONE       (1u << 31)
#define RESET_GO            (1u << 27)

/* the Share_Node_JE */
#define N_COMMAND           0x00
#define N_BS_PHY            0x04
#define N_IMAGE_WIDTH       0x08
#define N_RESTART           0x0C
#define N_U82D              0x20
#define N_ROI_LEFT          0x24
#define N_ROI_TOP           0x28
#define N_ROI_RIGHT         0x2C
#define N_ROI_BOTTOM        0x30
#define N_YUV_ADDR          0x38
#define N_MAX_BS_THD        0x44
#define N_STOP_X            0x48
#define N_STOP_Y            0x4C
#define N_BITSLEN           0xD4
#define N_DONE              0xD8

#define CMD_ENCODE_JPEG     8

/* the DMA wrapper */
#define W_WIDTH             0x08
#define W_HEIGHT            0x0C
#define W_VSMSA             0x18

struct GmMcp100State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion node_mem;
    MemoryRegion wrp_mem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_MCP100_NREGS];
    uint8_t node[GM_MCP100_NODE_SIZE];
    uint32_t wrp[GM_MCP100_WRP_NREGS];

    /* the header bit packer */
    uint32_t bs_phy;
    uint32_t hdr_len;
    uint32_t acc;
    int acc_bits;
    uint8_t hdr[GM_MCP100_HDR_MAX];
};

static void gm_mcp100_update(GmMcp100State *s)
{
    qemu_set_irq(s->irq, (s->regs[R_INT_FLAG / 4] & INT_JPEG_DONE) != 0);
}

/* ---- the header bit packer ------------------------------------------- */

static void gm_mcp100_put_bits(GmMcp100State *s, uint32_t val, int n)
{
    int i;

    for (i = n - 1; i >= 0; i--) {
        s->acc = s->acc << 1 | ((val >> i) & 1);
        if (++s->acc_bits == 8) {
            uint8_t b = s->acc;

            if (s->hdr_len < GM_MCP100_HDR_MAX) {
                s->hdr[s->hdr_len] = b;
            }
            dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len,
                             &b, 1, MEMTXATTRS_UNSPECIFIED);
            s->hdr_len++;
            s->acc = 0;
            s->acc_bits = 0;
        }
    }
}

/* the quantisation tables of the DQT segments the host wrote (zig-zag) */
static void gm_mcp100_parse_dqt(GmMcp100State *s, uint8_t q[4][64])
{
    const uint32_t n = MIN(s->hdr_len, GM_MCP100_HDR_MAX);
    uint32_t i = 2;

    while (i + 4 <= n) {
        uint32_t seg, end;

        if (s->hdr[i] != 0xFF) {
            i++;
            continue;
        }
        if (s->hdr[i + 1] == 0xDA) {    /* SOS: the headers end */
            break;
        }
        seg = s->hdr[i + 2] << 8 | s->hdr[i + 3];
        end = MIN(i + 2 + seg, n);
        if (s->hdr[i + 1] == 0xDB) {
            uint32_t p = i + 4;

            while (p + 65 <= end) {
                const unsigned id = s->hdr[p] & 3;

                if (s->hdr[p] >> 4) {   /* 16-bit tables: not baseline */
                    break;
                }
                memcpy(q[id], &s->hdr[p + 1], 64);
                p += 65;
            }
        }
        i += 2 + seg;
    }
}

/* ---- the encoder ----------------------------------------------------- */

static uint32_t gm_mcp100_node32(GmMcp100State *s, uint32_t off)
{
    return ldl_le_p(&s->node[off]);
}

/* fetch the picture into Y / Cb / Cr planes of w x h and w/2 x h/2 */
static bool gm_mcp100_fetch(GmMcp100State *s, uint32_t w, uint32_t h,
                            uint8_t *y, uint8_t *cb, uint8_t *cr)
{
    const uint32_t width = gm_mcp100_node32(s, N_IMAGE_WIDTH);
    const unsigned u82d = s->node[N_U82D];
    const uint32_t left = gm_mcp100_node32(s, N_ROI_LEFT) * 16;
    const uint32_t top = gm_mcp100_node32(s, N_ROI_TOP) * 16;
    uint32_t r, x;

    if (u82d == 4) {
        const uint32_t stride = width * 2;
        const hwaddr base = (hwaddr)s->wrp[W_VSMSA / 4] << 2;
        g_autofree uint8_t *row = g_malloc(stride ? stride : 1);

        if (!stride || w * 2 > stride) {
            return false;
        }
        for (r = 0; r < h; r++) {
            dma_memory_read(&address_space_memory, base + (hwaddr)r * stride,
                            row, w * 2, MEMTXATTRS_UNSPECIFIED);
            for (x = 0; x < w; x++) {
                y[r * w + x] = row[x * 2 + 1];
            }
            for (x = 0; x < w / 2; x++) {
                const size_t c = (size_t)(r / 2) * (w / 2) + x;

                if (r & 1) {        /* average the line pair */
                    cb[c] = (cb[c] + row[x * 4] + 1) / 2;
                    cr[c] = (cr[c] + row[x * 4 + 2] + 1) / 2;
                } else {
                    cb[c] = row[x * 4];
                    cr[c] = row[x * 4 + 2];
                }
            }
        }
        return true;
    }
    if (u82d == 1) {
        const hwaddr py = gm_mcp100_node32(s, N_YUV_ADDR);
        const hwaddr pu = gm_mcp100_node32(s, N_YUV_ADDR + 4);
        const hwaddr pv = gm_mcp100_node32(s, N_YUV_ADDR + 8);

        if (!width || left + w > width) {
            return false;
        }
        for (r = 0; r < h; r++) {
            dma_memory_read(&address_space_memory,
                            py + (hwaddr)(top + r) * width + left,
                            y + (size_t)r * w, w, MEMTXATTRS_UNSPECIFIED);
        }
        for (r = 0; r < h / 2; r++) {
            const hwaddr off = (hwaddr)(top / 2 + r) * (width / 2) + left / 2;

            dma_memory_read(&address_space_memory, pu + off,
                            cb + (size_t)r * (w / 2), w / 2,
                            MEMTXATTRS_UNSPECIFIED);
            dma_memory_read(&address_space_memory, pv + off,
                            cr + (size_t)r * (w / 2), w / 2,
                            MEMTXATTRS_UNSPECIFIED);
        }
        return true;
    }
    qemu_log_mask(LOG_UNIMP, "gm-mcp100: JPEG input format %u\n", u82d);
    return false;
}

static void gm_mcp100_encode(GmMcp100State *s)
{
    uint8_t q[4][64];
    uint32_t mw, mh, w, h, room, thd, restart, len = 0;
    g_autofree uint8_t *y = NULL, *cb = NULL, *cr = NULL, *out = NULL;
    bool ok;

    if (s->node[N_U82D] == 4) {
        mw = s->wrp[W_WIDTH / 4];
        mh = s->wrp[W_HEIGHT / 4];
    } else {
        mw = gm_mcp100_node32(s, N_ROI_RIGHT) - gm_mcp100_node32(s, N_ROI_LEFT);
        mh = gm_mcp100_node32(s, N_ROI_BOTTOM) - gm_mcp100_node32(s, N_ROI_TOP);
    }
    memset(q, 1, sizeof(q));
    gm_mcp100_parse_dqt(s, q);
    thd = gm_mcp100_node32(s, N_MAX_BS_THD);
    room = thd > s->hdr_len && thd < 64 * MiB ? thd - s->hdr_len : 16 * MiB;
    restart = gm_mcp100_node32(s, N_RESTART);

    if (mw && mh && mw <= GM_MCP100_MAX_MCUS && mh <= GM_MCP100_MAX_MCUS) {
        w = mw * 16;
        h = mh * 16;
        y = g_malloc0((size_t)w * h);
        cb = g_malloc0((size_t)w * h / 4);
        cr = g_malloc0((size_t)w * h / 4);
        out = g_malloc(room);
        ok = gm_mcp100_fetch(s, w, h, y, cb, cr);
        if (ok) {
            len = nvt_pixel_jpeg_scan_rst(out, room, y, w, cb, cr, w / 2, 1,
                                          mw, mh, q[0], q[1], restart);
        }
    }
    if (len) {
        dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len, out,
                         len, MEMTXATTRS_UNSPECIFIED);
    } else {
        /* would not fit (or no picture): stop at the start, as the firmware */
        static const uint8_t eoi[2] = { 0xFF, 0xD9 };

        dma_memory_write(&address_space_memory, s->bs_phy + s->hdr_len, eoi,
                         2, MEMTXATTRS_UNSPECIFIED);
        len = 2;
        stl_le_p(&s->node[N_STOP_X], 0);
        stl_le_p(&s->node[N_STOP_Y], 0);
        stl_le_p(&s->node[N_MAX_BS_THD], 0x0FFFFFFF);
    }
    stl_le_p(&s->node[N_BITSLEN], s->hdr_len + len);
    stl_le_p(&s->node[N_DONE], 1);
    s->node[N_COMMAND] = CMD_ENCODE_JPEG << 4;
    s->regs[R_INT_FLAG / 4] |= INT_JPEG_DONE;
    gm_mcp100_update(s);
}

static void gm_mcp100_job(void *opaque)
{
    GmMcp100State *s = opaque;

    if ((s->node[N_COMMAND] & 0xF) == CMD_ENCODE_JPEG) {
        gm_mcp100_encode(s);
    }
}

/* ---- registers ------------------------------------------------------- */

static uint64_t gm_mcp100_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    switch (off) {
    case R_INCTL:
        return s->regs[off / 4] | INCTL_CPU_IDLE;
    case R_VLDSTS:
        return s->regs[off / 4] | VLDSTS_READY;
    }
    return s->regs[off / 4];
}

static void gm_mcp100_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    GmMcp100State *s = opaque;

    switch (off) {
    case R_RESET:
        val &= ~RESET_GO;
        break;
    case R_BALR:
        gm_mcp100_put_bits(s, s->regs[R_BADR / 4], MIN(val & 0x3F, 32));
        break;
    case R_ABADR:
        s->bs_phy = val & ~3u;
        s->hdr_len = 0;
        s->acc = 0;
        s->acc_bits = 0;
        break;
    case R_INCTL:
        if ((val & INCTL_CPU_START) == INCTL_CPU_START) {
            s->node[N_COMMAND] = 0;     /* the firmware's first act */
        }
        break;
    case R_INT_FLAG:
        s->regs[off / 4] &= ~val;
        gm_mcp100_update(s);
        return;
    }
    s->regs[off / 4] = val;
}

static const MemoryRegionOps gm_mcp100_ops = {
    .read = gm_mcp100_read,
    .write = gm_mcp100_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* the mailbox: plain memory, but for the command byte */
static uint64_t gm_mcp100_node_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    return ldn_le_p(&s->node[off], size);
}

static void gm_mcp100_node_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    GmMcp100State *s = opaque;

    stn_le_p(&s->node[off], size, val);
    if (off == N_COMMAND && (val & 0xF) == CMD_ENCODE_JPEG) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            GM_MCP100_JOB_NS);
    } else if (off == N_COMMAND && (val & 0xF)) {
        qemu_log_mask(LOG_UNIMP, "gm-mcp100: firmware command %u\n",
                      (unsigned)(val & 0xF));
    }
}

static const MemoryRegionOps gm_mcp100_node_ops = {
    .read = gm_mcp100_node_read,
    .write = gm_mcp100_node_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t gm_mcp100_wrp_read(void *opaque, hwaddr off, unsigned size)
{
    GmMcp100State *s = opaque;

    return off < GM_MCP100_WRP_NREGS * 4 ? s->wrp[off / 4] : 0;
}

static void gm_mcp100_wrp_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    GmMcp100State *s = opaque;

    if (off < GM_MCP100_WRP_NREGS * 4) {
        s->wrp[off / 4] = val;
    }
}

static const MemoryRegionOps gm_mcp100_wrp_ops = {
    .read = gm_mcp100_wrp_read,
    .write = gm_mcp100_wrp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_mcp100_reset(DeviceState *dev)
{
    GmMcp100State *s = GM_MCP100(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->node, 0, sizeof(s->node));
    memset(s->wrp, 0, sizeof(s->wrp));
    s->bs_phy = s->hdr_len = s->acc = 0;
    s->acc_bits = 0;
    gm_mcp100_update(s);
}

static void gm_mcp100_realize(DeviceState *dev, Error **errp)
{
    GmMcp100State *s = GM_MCP100(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_mcp100_ops, s,
                          TYPE_GM_MCP100, GM_MCP100_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    memory_region_init_io(&s->node_mem, OBJECT(s), &gm_mcp100_node_ops, s,
                          TYPE_GM_MCP100 ".node", GM_MCP100_NODE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->node_mem);
    memory_region_init_io(&s->wrp_mem, OBJECT(s), &gm_mcp100_wrp_ops, s,
                          TYPE_GM_MCP100 ".dmawrp", GM_MCP100_WRP_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->wrp_mem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_mcp100_job, s);
}

static const VMStateDescription vmstate_gm_mcp100 = {
    .name = TYPE_GM_MCP100,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, GmMcp100State),
        VMSTATE_UINT32_ARRAY(regs, GmMcp100State, GM_MCP100_NREGS),
        VMSTATE_UINT8_ARRAY(node, GmMcp100State, GM_MCP100_NODE_SIZE),
        VMSTATE_UINT32_ARRAY(wrp, GmMcp100State, GM_MCP100_WRP_NREGS),
        VMSTATE_UINT32(bs_phy, GmMcp100State),
        VMSTATE_UINT32(hdr_len, GmMcp100State),
        VMSTATE_UINT32(acc, GmMcp100State),
        VMSTATE_INT32(acc_bits, GmMcp100State),
        VMSTATE_UINT8_ARRAY(hdr, GmMcp100State, GM_MCP100_HDR_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_mcp100_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_mcp100_realize;
    device_class_set_legacy_reset(dc, gm_mcp100_reset);
    dc->vmsd = &vmstate_gm_mcp100;
}

static const TypeInfo gm_mcp100_info = {
    .name           = TYPE_GM_MCP100,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(GmMcp100State),
    .class_init     = gm_mcp100_class_init,
};

static void gm_mcp100_register_types(void)
{
    type_register_static(&gm_mcp100_info);
}

type_init(gm_mcp100_register_types)
