/*
 * Novatek NA51055 / NA51089 I2C controller.
 *
 * drivers/i2c/busses/i2c-nvt.c and plat-na51089/i2c_reg.h.  The NT9856x
 * has three, at 0xF0220000 / 0xF0350000 / 0xF03A0000 (SPI 41 / 42 / 60).
 * The driver moves a message in PIO "chunks" of up to four bytes:
 *
 *   0x00 CTRL    bit 3 TB_EN starts a chunk; interrupt enables in
 *                [12:4] (DT 4, DR 5, BERR 6, ..., SCL timeout 12);
 *                bit 16 BUS_CLR (sends nine clocks, clears when done);
 *                bits 31.. DMA (the VD-synchronised writes, not modelled)
 *   0x04 STS     bit 1 NACK, 2 BUSY, 3 BUSBUSY, 4 DT (a chunk sent),
 *                5 DR (a chunk received), 6 BERR (no ACK), 28 BUSFREE;
 *                write 1 to clear
 *   0x08 CONFIG  [18:16] the chunk's byte count, and per byte i:
 *                bit 4+i NACK after it (reads), 8+i START before it,
 *                12+i STOP after it
 *   0x10 DATA    the chunk, byte i in bits [8i+7:8i]
 *   0x0C BUSCLK, 0x18 TIMING, 0x1C BUS, 0x28 BUSFREE: storage
 *
 * A byte with START before it is the address byte (addr << 1 | R/W),
 * which begins (or restarts) a transfer on the bus; the bytes after it
 * go the way that address said: a read chunk gets its data in DATA, from
 * byte 1 on in the first chunk (nvtim_get_rxdata() skips the address).
 * A chunk ends with DT for a write, DR for a read, or BERR when the
 * target did not ACK its address or a byte -- the driver then sends a
 * lone STOP (i2c_nvtim_handle_nack()).  The bus is otherwise idle:
 * BUSFREE set, BUSBUSY clear.  SPI n = STS & CTRL & 0x1FF0.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_I2C "nvt-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(NvtI2cState, NVT_I2C)

#define I2C_REGS_SIZE   0x100
#define I2C_CTRL        0x00
#define I2C_STS         0x04
#define I2C_CONFIG      0x08
#define I2C_DATA        0x10

#define CTRL_TB_EN      (1u << 3)
#define CTRL_INT_MASK   0x1FF0
#define CTRL_BUS_CLR    (1u << 16)
#define CTRL_DMA_EN     (1u << 31)
#define CONFIG_DMA      (1u << 0)
#define STS_NACK        (1u << 1)
#define STS_DT          (1u << 4)
#define STS_DR          (1u << 5)
#define STS_BERR        (1u << 6)
#define STS_BUSFREE     (1u << 28)
#define STS_W1C         0x1FF2

struct NvtI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    uint32_t regs[I2C_REGS_SIZE / 4];
    bool active;                /* a transfer is open on the bus */
    bool recv;                  /* and it is a read */
};

static uint32_t *reg(NvtI2cState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_i2c_update_irq(NvtI2cState *s)
{
    qemu_set_irq(s->irq, *reg(s, I2C_STS) & *reg(s, I2C_CTRL) &
                 CTRL_INT_MASK);
}

static void nvt_i2c_stop(NvtI2cState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
}

/* Run one PIO chunk as CONFIG and DATA describe it. */
static void nvt_i2c_chunk(NvtI2cState *s)
{
    uint32_t cfg = *reg(s, I2C_CONFIG);
    uint32_t data = *reg(s, I2C_DATA);
    int n = (cfg >> 16) & 7, i;
    uint32_t sts = 0;
    uint8_t byte;
    bool any_recv = false;

    if (cfg & CONFIG_DMA) {
        qemu_log_mask(LOG_UNIMP, "nvt-i2c: DMA (VD sync) transfers\n");
        return;
    }
    n = MIN(n, 4);
    for (i = 0; i < n; i++) {
        if (cfg & (1u << (8 + i))) {
            /* START (or repeated START) and the address byte */
            byte = data >> (8 * i);
            s->recv = byte & 1;
            if (i2c_start_transfer(s->bus, byte >> 1, s->recv)) {
                s->active = false;
                sts |= STS_BERR | STS_NACK;
                break;
            }
            s->active = true;
        } else if (!s->active) {
            /* No transfer open: the lone STOP the driver sends after a
             * NACK (i2c_nvtim_handle_nack()) has nothing left to end. */
        } else if (s->recv) {
            byte = i2c_recv(s->bus);
            data = deposit32(data, 8 * i, 8, byte);
            any_recv = true;
            if (cfg & (1u << (4 + i))) {
                i2c_nack(s->bus);
            }
        } else if (i2c_send(s->bus, data >> (8 * i))) {
            sts |= STS_BERR | STS_NACK;
            break;
        }
        if (cfg & (1u << (12 + i))) {
            nvt_i2c_stop(s);
        }
    }
    if (!(sts & STS_BERR)) {
        sts |= (any_recv || (s->active && s->recv)) ? STS_DR : STS_DT;
    }
    *reg(s, I2C_DATA) = data;
    *reg(s, I2C_STS) |= sts;
}

static uint64_t nvt_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    NvtI2cState *s = opaque;

    if (off == I2C_STS) {
        /* never busy: every chunk completes when it is started */
        return *reg(s, I2C_STS) | STS_BUSFREE;
    }
    return *reg(s, off);
}

static void nvt_i2c_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtI2cState *s = opaque;

    switch (off) {
    case I2C_CTRL:
        /* Bus clear and a chunk both finish at once. */
        *reg(s, off) = val & ~(CTRL_BUS_CLR | CTRL_TB_EN);
        if (val & CTRL_DMA_EN) {
            qemu_log_mask(LOG_UNIMP, "nvt-i2c: DMA (VD sync) transfers\n");
        } else if (val & CTRL_TB_EN) {
            nvt_i2c_chunk(s);
        }
        nvt_i2c_update_irq(s);
        break;
    case I2C_STS:
        *reg(s, off) &= ~(val & STS_W1C);
        nvt_i2c_update_irq(s);
        break;
    default:
        *reg(s, off) = val;
        break;
    }
}

static const MemoryRegionOps nvt_i2c_ops = {
    .read = nvt_i2c_read,
    .write = nvt_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_i2c_reset(DeviceState *dev)
{
    NvtI2cState *s = NVT_I2C(dev);

    nvt_i2c_stop(s);
    memset(s->regs, 0, sizeof(s->regs));
    /* plat-na51089/i2c_reg.h *_DEFAULT */
    *reg(s, I2C_CTRL) = 0x00000002;
    *reg(s, 0x18) = 0x03FF1001;
    *reg(s, 0x1C) = 0x00000003;
    *reg(s, 0x28) = 0x000000E2;
    qemu_irq_lower(s->irq);
}

static void nvt_i2c_init(Object *obj)
{
    NvtI2cState *s = NVT_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_i2c_ops, s, TYPE_NVT_I2C,
                          I2C_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_nvt_i2c = {
    .name = TYPE_NVT_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtI2cState, I2C_REGS_SIZE / 4),
        VMSTATE_BOOL(active, NvtI2cState),
        VMSTATE_BOOL(recv, NvtI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_i2c_reset);
    dc->vmsd = &vmstate_nvt_i2c;
}

static const TypeInfo nvt_i2c_info = {
    .name = TYPE_NVT_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtI2cState),
    .instance_init = nvt_i2c_init,
    .class_init = nvt_i2c_class_init,
};

static void nvt_i2c_register_types(void)
{
    type_register_static(&nvt_i2c_info);
}

type_init(nvt_i2c_register_types)
