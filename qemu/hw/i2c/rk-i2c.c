/*
 * Rockchip RK3x I2C controller ("rockchip,rk3399-i2c" and its RV1106
 * copies: I2C0..4 at 0xFF310000, 0xFF320000, 0xFF450000, 0xFF460000,
 * 0xFF470000).  The camera sensor sits on I2C4.
 *
 * Register file, from drivers/i2c/busses/i2c-rk3x.c:
 *
 *   0x00 CON      bit 0 EN, bits 2:1 mode (0 TX, 1 REGISTER_TX,
 *                 2 RX, 3 REGISTER_RX), bit 3 START, bit 4 STOP,
 *                 bit 5 LASTACK (NACK the last byte read), bit 6 ACTACK
 *                 (stop on NACK); bits 24:16 version (read only)
 *   0x08 MRXADDR  slave address for the register modes, bits 26:24 valid
 *   0x0c MRXRADDR register address bytes for the register modes, valid
 *   0x10 MTXCNT   writing it sends that many TXBUF bytes
 *   0x14 MRXCNT   writing it receives that many bytes into RXBUF
 *   0x18 IEN / 0x1c IPD (write 1 to clear): bit 0 BTF, 1 BRF, 2 MBTF,
 *                 3 MBRF, 4 START, 5 STOP, 6 NAKRCV
 *   0x20 FCNT     bytes moved by the last transfer
 *   0x228 CON1    bit 0 AUTO_STOP, bit 1 TRANSFER_AUTO_STOP, bit 2
 *                 NACK_AUTO_STOP (version 5 and up)
 *   0x100 TXBUF0..7, 0x200 RXBUF0..7: 32 bytes each, little-endian
 *
 * The controller has no repeated start of its own: START with the
 * address as the first TXBUF byte in TX mode, or with MRXADDR (and
 * MRXRADDR's register bytes, then a restart to read) in the register
 * modes; a further chunk of a long read is plain RX on the open
 * transfer.  Every transfer completes at the count write.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_I2C "rk-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(RkI2cState, RK_I2C)

#define R_CON       0x00
#define R_CLKDIV    0x04
#define R_MRXADDR   0x08
#define R_MRXRADDR  0x0c
#define R_MTXCNT    0x10
#define R_MRXCNT    0x14
#define R_IEN       0x18
#define R_IPD       0x1c
#define R_FCNT      0x20
#define R_SCL_OE_DB 0x24
#define R_CON1      0x228
#define R_TXBUF     0x100
#define R_RXBUF     0x200
#define REGS_SIZE   0x1000

#define CON_EN          (1u << 0)
#define CON_MOD(c)      (((c) >> 1) & 3)
#define CON_START       (1u << 3)
#define CON_STOP        (1u << 4)
#define CON_LASTACK     (1u << 5)
#define CON_ACTACK      (1u << 6)
#define CON_VERSION_SHIFT 16

#define MOD_TX          0
#define MOD_REG_TX      1
#define MOD_RX          2
#define MOD_REG_RX      3

#define INT_BTF         (1u << 0)
#define INT_BRF         (1u << 1)
#define INT_MBTF        (1u << 2)
#define INT_MBRF        (1u << 3)
#define INT_START       (1u << 4)
#define INT_STOP        (1u << 5)
#define INT_NAKRCV      (1u << 6)

#define CON1_AUTO_STOP          (1u << 0)
#define CON1_TRANSFER_AUTO_STOP (1u << 1)
#define CON1_NACK_AUTO_STOP     (1u << 2)

struct RkI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    uint32_t con, clkdiv, mrxaddr, mrxraddr, mtxcnt, mrxcnt;
    uint32_t ien, ipd, fcnt, scl_oe_db, con1;
    uint8_t txbuf[32];
    uint8_t rxbuf[32];
    bool started;       /* a START is pending: the next byte addresses */
    bool active;        /* a transfer is open on the bus */
    bool reading;       /* ... and it is a read */
    uint8_t version;
};

static void rk_i2c_update_irq(RkI2cState *s)
{
    qemu_set_irq(s->irq, !!(s->ipd & s->ien));
}

static void rk_i2c_stop(RkI2cState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
    s->started = false;
    s->ipd |= INT_STOP;
}

/* A NACK: report it, and end the transfer if the controller is told to. */
static void rk_i2c_nack(RkI2cState *s)
{
    s->ipd |= INT_NAKRCV;
    if (s->con1 & CON1_NACK_AUTO_STOP) {
        rk_i2c_stop(s);
    } else if (s->active) {
        /* the bus is released at the STOP the driver sends next */
        i2c_end_transfer(s->bus);
        s->active = false;
    }
}

/* Address @addr8 (8-bit form, bit 0 = read); false on NACK. */
static bool rk_i2c_address(RkI2cState *s, uint8_t addr8)
{
    bool rd = addr8 & 1;

    /* on an open transfer this is a repeated start */
    if (i2c_start_transfer(s->bus, addr8 >> 1, rd)) {
        if (s->active) {
            i2c_end_transfer(s->bus);
        }
        s->active = false;
        return false;
    }
    s->active = true;
    s->reading = rd;
    return true;
}

static void rk_i2c_done(RkI2cState *s, uint32_t flag)
{
    s->ipd |= flag;
    if ((s->con1 & CON1_AUTO_STOP) && (s->con1 & CON1_TRANSFER_AUTO_STOP)) {
        rk_i2c_stop(s);
    }
}

static void rk_i2c_do_tx(RkI2cState *s)
{
    int n = MIN(s->mtxcnt & 0x3f, 32);
    int i = 0;

    if (!(s->con & CON_EN)) {
        return;
    }
    s->fcnt = 0;
    if (s->started) {
        s->started = false;
        if (n == 0) {
            return;
        }
        if (!rk_i2c_address(s, s->txbuf[0])) {
            rk_i2c_nack(s);
            rk_i2c_update_irq(s);
            return;
        }
        i = 1;
        s->fcnt = 1;
    }
    for (; i < n; i++) {
        if (!s->active || i2c_send(s->bus, s->txbuf[i])) {
            rk_i2c_nack(s);
            rk_i2c_update_irq(s);
            return;
        }
        s->fcnt++;
    }
    rk_i2c_done(s, INT_MBTF);
}

static void rk_i2c_do_rx(RkI2cState *s)
{
    int n = MIN(s->mrxcnt & 0x3f, 32);
    int mod = CON_MOD(s->con);
    int i;

    if (!(s->con & CON_EN)) {
        return;
    }
    s->fcnt = 0;
    if (s->started && (mod == MOD_REG_TX || mod == MOD_REG_RX)) {
        uint8_t addr8 = s->mrxaddr & 0xff;
        int nreg = 0;

        s->started = false;
        for (i = 0; i < 3; i++) {
            if (s->mrxraddr & (1u << (24 + i))) {
                nreg = i + 1;
            }
        }
        if (nreg || !(addr8 & 1)) {
            /* write the register address, then restart to read */
            if (!rk_i2c_address(s, addr8 & ~1)) {
                rk_i2c_nack(s);
                rk_i2c_update_irq(s);
                return;
            }
            for (i = 0; i < nreg; i++) {
                if (i2c_send(s->bus, s->mrxraddr >> (i * 8))) {
                    rk_i2c_nack(s);
                    rk_i2c_update_irq(s);
                    return;
                }
            }
        }
        if (!rk_i2c_address(s, addr8 | 1)) {
            rk_i2c_nack(s);
            rk_i2c_update_irq(s);
            return;
        }
    } else if (s->started) {
        /* plain RX straight after START: the address is MRXADDR's */
        s->started = false;
        if (!rk_i2c_address(s, (s->mrxaddr & 0xff) | 1)) {
            rk_i2c_nack(s);
            rk_i2c_update_irq(s);
            return;
        }
    }
    memset(s->rxbuf, 0, sizeof(s->rxbuf));
    for (i = 0; i < n; i++) {
        s->rxbuf[i] = s->active && s->reading ? i2c_recv(s->bus) : 0xff;
    }
    if (n && s->active && (s->con & CON_LASTACK)) {
        i2c_nack(s->bus);
    }
    s->fcnt = n;
    rk_i2c_done(s, INT_MBRF);
}

static uint64_t rk_i2c_read(void *opaque, hwaddr off, unsigned size)
{
    RkI2cState *s = opaque;

    if (off >= R_TXBUF && off < R_TXBUF + 32) {
        return ldl_le_p(&s->txbuf[off - R_TXBUF]);
    }
    if (off >= R_RXBUF && off < R_RXBUF + 32) {
        return ldl_le_p(&s->rxbuf[off - R_RXBUF]);
    }
    switch (off) {
    case R_CON:
        return (s->con & 0xffff) | ((uint32_t)s->version << CON_VERSION_SHIFT);
    case R_CLKDIV:
        return s->clkdiv;
    case R_MRXADDR:
        return s->mrxaddr;
    case R_MRXRADDR:
        return s->mrxraddr;
    case R_MTXCNT:
        return s->mtxcnt;
    case R_MRXCNT:
        return s->mrxcnt;
    case R_IEN:
        return s->ien;
    case R_IPD:
        return s->ipd;
    case R_FCNT:
        return s->fcnt;
    case R_SCL_OE_DB:
        return s->scl_oe_db;
    case R_CON1:
        return s->con1;
    }
    return 0;
}

static void rk_i2c_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkI2cState *s = opaque;

    if (off >= R_TXBUF && off < R_TXBUF + 32) {
        stl_le_p(&s->txbuf[off - R_TXBUF], val);
        return;
    }
    switch (off) {
    case R_CON: {
        uint32_t old = s->con;

        s->con = val & 0xffff;
        if (!(s->con & CON_EN)) {
            /* disabling resets the state machine; the bus is let go */
            if (s->active) {
                i2c_end_transfer(s->bus);
                s->active = false;
            }
            s->started = false;
            break;
        }
        if ((s->con & CON_START) && !(old & CON_START)) {
            s->started = true;
            s->ipd |= INT_START;
        }
        if ((s->con & CON_STOP) && !(old & CON_STOP)) {
            rk_i2c_stop(s);
        }
        break;
    }
    case R_CLKDIV:
        s->clkdiv = val;
        break;
    case R_MRXADDR:
        s->mrxaddr = val;
        break;
    case R_MRXRADDR:
        s->mrxraddr = val;
        break;
    case R_MTXCNT:
        s->mtxcnt = val & 0x3f;
        rk_i2c_do_tx(s);
        break;
    case R_MRXCNT:
        s->mrxcnt = val & 0x3f;
        rk_i2c_do_rx(s);
        break;
    case R_IEN:
        s->ien = val & 0xff;
        break;
    case R_IPD:
        s->ipd &= ~val;
        break;
    case R_SCL_OE_DB:
        s->scl_oe_db = val;
        break;
    case R_CON1:
        s->con1 = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rk-i2c: write 0x%" HWADDR_PRIx "\n",
                      off);
        return;
    }
    rk_i2c_update_irq(s);
}

static const MemoryRegionOps rk_i2c_ops = {
    .read = rk_i2c_read,
    .write = rk_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_i2c_reset(DeviceState *dev)
{
    RkI2cState *s = RK_I2C(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->con = s->clkdiv = s->mrxaddr = s->mrxraddr = 0;
    s->mtxcnt = s->mrxcnt = s->ien = s->ipd = s->fcnt = 0;
    s->scl_oe_db = s->con1 = 0;
    memset(s->txbuf, 0, sizeof(s->txbuf));
    memset(s->rxbuf, 0, sizeof(s->rxbuf));
    s->started = s->active = s->reading = false;
    rk_i2c_update_irq(s);
}

static void rk_i2c_init(Object *obj)
{
    RkI2cState *s = RK_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &rk_i2c_ops, s, TYPE_RK_I2C,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const Property rk_i2c_properties[] = {
    /* CON[24:16]; 5 and up give the driver CON1's auto stop */
    DEFINE_PROP_UINT8("version", RkI2cState, version, 0),
};

static const VMStateDescription rk_i2c_vmstate = {
    .name = TYPE_RK_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(con, RkI2cState),
        VMSTATE_UINT32(clkdiv, RkI2cState),
        VMSTATE_UINT32(mrxaddr, RkI2cState),
        VMSTATE_UINT32(mrxraddr, RkI2cState),
        VMSTATE_UINT32(mtxcnt, RkI2cState),
        VMSTATE_UINT32(mrxcnt, RkI2cState),
        VMSTATE_UINT32(ien, RkI2cState),
        VMSTATE_UINT32(ipd, RkI2cState),
        VMSTATE_UINT32(fcnt, RkI2cState),
        VMSTATE_UINT32(scl_oe_db, RkI2cState),
        VMSTATE_UINT32(con1, RkI2cState),
        VMSTATE_UINT8_ARRAY(txbuf, RkI2cState, 32),
        VMSTATE_UINT8_ARRAY(rxbuf, RkI2cState, 32),
        VMSTATE_BOOL(started, RkI2cState),
        VMSTATE_BOOL(active, RkI2cState),
        VMSTATE_BOOL(reading, RkI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_i2c_reset);
    device_class_set_props(dc, rk_i2c_properties);
    dc->vmsd = &rk_i2c_vmstate;
}

static const TypeInfo rk_i2c_info = {
    .name = TYPE_RK_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkI2cState),
    .instance_init = rk_i2c_init,
    .class_init = rk_i2c_class_init,
};

static void rk_i2c_register_types(void)
{
    type_register_static(&rk_i2c_info);
}

type_init(rk_i2c_register_types)
