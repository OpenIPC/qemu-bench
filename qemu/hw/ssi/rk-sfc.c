/*
 * Rockchip SFC (serial flash controller), as on RV1103 / RV1106 / RV1126.
 *
 * One command register describes a whole SPI transaction: opcode, address
 * width, dummy clocks, direction, and (V4+) a length in LEN_EXT.  The
 * transaction starts at the CMD write when it has no address and at the
 * ADDR write otherwise.  Data then moves through the 32-bit DATA FIFO
 * register or by DMA (DMA_ADDR + DMA_TRIGGER).
 *
 * Drivers (U-Boot rockchip_sfc.c, Linux spi-rockchip-sfc.c, Linux
 * rkflash/sfc.c and the vendor SPL) all agree on the register layout:
 *
 *   0x00 CTRL    0x04 IMR    0x08 ICLR   0x0c FTLR   0x10 RCVR
 *   0x14 AX      0x18 ABIT   0x1c ISR    0x20 FSR    0x24 SR
 *   0x28 RISR    0x2c VER    0x34 EXT_CTRL           0x3c DLL_CTRL0
 *   0x80 DMA_TRIGGER  0x84 DMA_ADDR  0x88 LEN_CTRL  0x8c LEN_EXT
 *   0x100 CMD    0x104 ADDR  0x108 DATA   (+0x200: CS1 copies of CTRL etc.)
 *
 * A read runs on the bus at once and its data waits in a buffer; a write
 * collects its data and runs on the bus when the last byte arrives.  The
 * flash never stretches a transfer, so SR always reads idle.  m25p80
 * counts dummy clocks one ssi_transfer() each, as the command's dummy
 * field does.  With "dummy-bytes" the dummy clocks go out as bytes
 * instead, cycles x the address lines' width / 8: the convention of a
 * SPI NAND that also sees its dummy folded into an address byte (U-Boot
 * spi-mem on the RV1126 sends read-from-cache as a 24-bit address, the
 * kernel's rkflash as 16 bits and 8 dummy cycles; both are 24 clocks).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_RK_SFC "rk-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(RkSfcState, RK_SFC)

#define R_CTRL          0x00
#define R_IMR           0x04
#define R_ICLR          0x08
#define R_FTLR          0x0c
#define R_RCVR          0x10
#define R_AX            0x14
#define R_ABIT          0x18
#define R_ISR           0x1c
#define R_FSR           0x20
#define R_SR            0x24
#define R_RISR          0x28
#define R_VER           0x2c
#define R_EXT_CTRL      0x34
#define R_DLL_CTRL0     0x3c
#define R_DMA_TRIGGER   0x80
#define R_DMA_ADDR      0x84
#define R_LEN_CTRL      0x88
#define R_LEN_EXT       0x8c
#define R_CMD           0x100
#define R_ADDR          0x104
#define R_DATA          0x108

#define INT_TRAN_FINISH (1u << 4)
#define INT_DMA         (1u << 7)

#define FSR_TX_EMPTY    (1u << 1)
#define FSR_RX_EMPTY    (1u << 2)

#define FIFO_WORDS      16      /* what FSR reports at most */

#define SFC_REGS        0x80    /* per-CS register copy, 32 words */

struct RkSfcState {
    SysBusDevice parent_obj;
    bool dummy_bytes;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs[2];
    SSIBus *spi;
    AddressSpace *as;

    uint32_t version;

    uint32_t regs[2][SFC_REGS / 4];     /* CTRL..DLL per chip select */
    uint32_t imr, risr, dma_addr, len_ctrl, len_ext, cmd, addr;

    /* the transaction in flight */
    bool pending;           /* CMD written, waiting for ADDR */
    bool active;            /* data stage under way */
    bool write;
    int cs_num;
    uint32_t len, pos;
    uint8_t *buf;
    uint32_t buf_size;
};

static void rk_sfc_update_irq(RkSfcState *s)
{
    qemu_set_irq(s->irq, (s->risr & ~s->imr) != 0);
}

static uint32_t rk_sfc_xfer_len(RkSfcState *s)
{
    if (s->version >= 4 && (s->len_ctrl & 1)) {
        return s->len_ext;
    }
    return extract32(s->cmd, 16, 14);
}

static void rk_sfc_buf_reserve(RkSfcState *s, uint32_t len)
{
    if (len > s->buf_size) {
        s->buf = g_realloc(s->buf, len);
        s->buf_size = len;
    }
}

/* Chip select on, then opcode, address and dummy clocks. */
static void rk_sfc_head(RkSfcState *s)
{
    uint32_t mode = extract32(s->cmd, 14, 2);
    uint32_t dummy = extract32(s->cmd, 8, 4);
    uint32_t awidth = 1 << extract32(s->regs[s->cs_num][R_CTRL / 4], 10, 2);

    if (s->dummy_bytes) {
        dummy = DIV_ROUND_UP(dummy * awidth, 8);
    }
    int abytes = 0, i;

    switch (mode) {
    case 1:
        abytes = 3;
        break;
    case 2:
        abytes = 4;
        break;
    case 3:
        abytes = (s->regs[s->cs_num][R_ABIT / 4] + 1 + 7) / 8;
        break;
    }
    qemu_set_irq(s->cs[s->cs_num], 0);
    ssi_transfer(s->spi, s->cmd & 0xff);
    for (i = abytes - 1; i >= 0; i--) {
        ssi_transfer(s->spi, i < 4 ? (s->addr >> (8 * i)) & 0xff : 0xff);
    }
    for (i = 0; i < dummy; i++) {
        ssi_transfer(s->spi, 0);
    }
}

static void rk_sfc_finish(RkSfcState *s)
{
    qemu_set_irq(s->cs[s->cs_num], 1);
    s->active = false;
    s->risr |= INT_TRAN_FINISH;
    rk_sfc_update_irq(s);
}

static void rk_sfc_start(RkSfcState *s)
{
    uint32_t i;

    s->pending = false;
    s->write = extract32(s->cmd, 12, 1);
    s->cs_num = extract32(s->cmd, 30, 1);
    s->len = rk_sfc_xfer_len(s);
    s->pos = 0;

    if (!s->len) {
        rk_sfc_head(s);
        rk_sfc_finish(s);
        return;
    }
    rk_sfc_buf_reserve(s, s->len);
    s->active = true;
    if (!s->write) {
        rk_sfc_head(s);
        for (i = 0; i < s->len; i++) {
            s->buf[i] = ssi_transfer(s->spi, 0);
        }
        qemu_set_irq(s->cs[s->cs_num], 1);
    }
}

/* A write's data is all in: run it on the bus. */
static void rk_sfc_flush_write(RkSfcState *s)
{
    uint32_t i;

    rk_sfc_head(s);
    for (i = 0; i < s->len; i++) {
        ssi_transfer(s->spi, s->buf[i]);
    }
    rk_sfc_finish(s);
}

static void rk_sfc_dma(RkSfcState *s)
{
    uint32_t n;

    if (!s->active) {
        qemu_log_mask(LOG_GUEST_ERROR, "rk-sfc: DMA with no transfer\n");
        return;
    }
    n = s->len - s->pos;
    if (s->write) {
        dma_memory_read(s->as, s->dma_addr, s->buf + s->pos, n,
                        MEMTXATTRS_UNSPECIFIED);
        s->pos = s->len;
        rk_sfc_flush_write(s);
    } else {
        dma_memory_write(s->as, s->dma_addr, s->buf + s->pos, n,
                         MEMTXATTRS_UNSPECIFIED);
        s->pos = s->len;
        rk_sfc_finish(s);
    }
    s->risr |= INT_DMA;
    rk_sfc_update_irq(s);
}

static uint32_t rk_sfc_fsr(RkSfcState *s)
{
    uint32_t v = FSR_TX_EMPTY | (FIFO_WORDS << 8);
    uint32_t rx = 0;

    if (s->active && !s->write) {
        rx = (s->len - s->pos + 3) / 4;
    }
    if (!rx) {
        v |= FSR_RX_EMPTY;
    }
    v |= MIN(rx, 31) << 16;
    return v;
}

static uint64_t rk_sfc_read(void *opaque, hwaddr off, unsigned size)
{
    RkSfcState *s = opaque;
    int cs = off >= 0x200 ? 1 : 0;
    uint32_t v = 0, i;

    switch (off) {
    case R_IMR:
        return s->imr;
    case R_ISR:
        return s->risr & ~s->imr;
    case R_RISR:
        return s->risr;
    case R_FSR:
        return rk_sfc_fsr(s);
    case R_SR:
    case R_RCVR:
        return 0;
    case R_VER:
        return s->version;
    case R_DMA_ADDR:
        return s->dma_addr;
    case R_LEN_CTRL:
        return s->len_ctrl;
    case R_LEN_EXT:
        return s->len_ext;
    case R_CMD:
        return s->cmd;
    case R_ADDR:
        return s->addr;
    case R_DATA:
        if (!s->active || s->write) {
            qemu_log_mask(LOG_GUEST_ERROR, "rk-sfc: DATA read, no data\n");
            return 0;
        }
        for (i = 0; i < 4 && s->pos < s->len; i++) {
            v |= (uint32_t)s->buf[s->pos++] << (8 * i);
        }
        if (s->pos >= s->len) {
            rk_sfc_finish(s);
        }
        return v;
    default:
        off &= 0x1ff;
        if (off < SFC_REGS) {
            return s->regs[cs][off / 4];
        }
        qemu_log_mask(LOG_UNIMP, "rk-sfc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void rk_sfc_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkSfcState *s = opaque;
    int cs = off >= 0x200 ? 1 : 0;
    int i;

    switch (off) {
    case R_IMR:
        s->imr = val;
        rk_sfc_update_irq(s);
        return;
    case R_ICLR:
        s->risr &= ~val;
        rk_sfc_update_irq(s);
        return;
    case R_RCVR:
        if (val & 1) {
            if (s->active || s->pending) {
                qemu_set_irq(s->cs[s->cs_num], 1);
            }
            s->active = s->pending = false;
        }
        return;
    case R_DMA_TRIGGER:
        if (val & 1) {
            rk_sfc_dma(s);
        }
        return;
    case R_DMA_ADDR:
        s->dma_addr = val;
        return;
    case R_LEN_CTRL:
        s->len_ctrl = val;
        return;
    case R_LEN_EXT:
        s->len_ext = val;
        return;
    case R_CMD:
        if (s->active) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "rk-sfc: CMD with %u of %u bytes left over\n",
                          s->len - s->pos, s->len);
            qemu_set_irq(s->cs[s->cs_num], 1);
            s->active = false;
        }
        s->cmd = val;
        if (extract32(val, 14, 2)) {
            s->pending = true;      /* starts at the ADDR write */
        } else {
            rk_sfc_start(s);
        }
        return;
    case R_ADDR:
        s->addr = val;
        if (s->pending) {
            rk_sfc_start(s);
        }
        return;
    case R_DATA:
        if (!s->active || !s->write) {
            qemu_log_mask(LOG_GUEST_ERROR, "rk-sfc: DATA write, no data\n");
            return;
        }
        for (i = 0; i < 4 && s->pos < s->len; i++) {
            s->buf[s->pos++] = val >> (8 * i);
        }
        if (s->pos >= s->len) {
            rk_sfc_flush_write(s);
        }
        return;
    default:
        off &= 0x1ff;
        if (off < SFC_REGS) {
            s->regs[cs][off / 4] = val;
            return;
        }
        qemu_log_mask(LOG_UNIMP, "rk-sfc: write 0x%" HWADDR_PRIx " = 0x%"
                      PRIx64 "\n", off, val);
    }
}

static const MemoryRegionOps rk_sfc_ops = {
    .read = rk_sfc_read,
    .write = rk_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_sfc_reset(DeviceState *dev)
{
    RkSfcState *s = RK_SFC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->imr = 0xffffffff;
    s->risr = s->dma_addr = s->len_ctrl = s->len_ext = 0;
    s->cmd = s->addr = 0;
    s->active = s->pending = false;
    qemu_set_irq(s->cs[0], 1);
    qemu_set_irq(s->cs[1], 1);
    rk_sfc_update_irq(s);
}

static void rk_sfc_realize(DeviceState *dev, Error **errp)
{
    RkSfcState *s = RK_SFC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->spi = ssi_create_bus(dev, "ssi");
    s->as = &address_space_memory;
    memory_region_init_io(&s->iomem, OBJECT(s), &rk_sfc_ops, s,
                          TYPE_RK_SFC, 0x4000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, s->cs, SSI_GPIO_CS, 2);
}

static void rk_sfc_finalize(Object *obj)
{
    g_free(RK_SFC(obj)->buf);
}

static const Property rk_sfc_props[] = {
    DEFINE_PROP_UINT32("version", RkSfcState, version, 8),
    DEFINE_PROP_BOOL("dummy-bytes", RkSfcState, dummy_bytes, false),
};

static void rk_sfc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_sfc_realize;
    device_class_set_legacy_reset(dc, rk_sfc_reset);
    device_class_set_props(dc, rk_sfc_props);
    dc->desc = "Rockchip serial flash controller";
}

static const TypeInfo rk_sfc_info = {
    .name = TYPE_RK_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkSfcState),
    .instance_finalize = rk_sfc_finalize,
    .class_init = rk_sfc_class_init,
};

static void rk_sfc_register_types(void)
{
    type_register_static(&rk_sfc_info);
}

type_init(rk_sfc_register_types)
