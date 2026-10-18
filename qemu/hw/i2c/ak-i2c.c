/*
 * Anyka AK39xx I2C master.
 *
 * Registers (kernel mach-ak39 include/mach/i2c.h, drivers/i2c/busses/
 * i2c-ak39.c):
 *
 *   0x00 CTRL   17 NACK enable, 16 no STOP, 15 ACK enable, 14 START (go),
 *               13 1 = read, 12:9 data bytes - 1, 8:7 delays, 6 divider,
 *               5 interrupt enable, 4 transfer done, 3:0 divider
 *   0x10..0x1C  CMD1..CMD4: 18 enable, 17 START condition, 7:0 the byte;
 *               the driver puts the address byte (addr << 1 | R/W) in CMD1
 *   0x20..0x2C  DATA0..DATA3: up to 16 data bytes, little endian
 *
 * Writing CTRL with START runs the whole transaction: the address from
 * CMD1, then the data bytes to or from DATA.  The vendor driver polls the
 * done flag with no timeout (ak39_poll_status()), so it must always come;
 * a missing device does not acknowledge, and reads from it see 0xFF.
 * Nothing in these drivers looks at the acknowledge.
 *
 * The EV330 has a later controller ("anyka,ak3790d-i2c" in its vendor
 * ak_i2c.ko), selected with layout=2: the address byte (| 0x300) at 0x08,
 * DATA at 0x10..0x1C, and in CTRL the count in 21:17, bit 23 starting a
 * write, bit 26 a read, bit 13 done (the driver clears it before each
 * start) and bit 27 "not acknowledged", which the driver checks after a
 * read -- so a sensor that is not there fails its probe cleanly.
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

#define TYPE_AK_I2C "ak-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(AkI2cState, AK_I2C)

#define R_CTRL          0x00
#define R_CMD1          0x10
#define R_DATA0         0x20

#define CTRL_NOSTOP     (1u << 16)
#define CTRL_START      (1u << 14)
#define CTRL_READ       (1u << 13)
#define CTRL_COUNT(v)   ((((v) >> 9) & 0xF) + 1)
#define CTRL_INTEN      (1u << 5)
#define CTRL_DONE       (1u << 4)

#define AK_I2C_DATA     16

#define V2_ADDR         0x08
#define V2_DATA0        0x10
#define V2_COUNT(v)     (((v) >> 17) & 0x1F)
#define V2_WRITE_GO     (1u << 23)
#define V2_READ_GO      (1u << 26)
#define V2_DONE         (1u << 13)
#define V2_NAK          (1u << 27)

struct AkI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t cmd[4];
    uint8_t data[AK_I2C_DATA];
    bool in_transfer;
    uint32_t layout;
};

static void ak_i2c_update_irq(AkI2cState *s)
{
    if (s->layout == 2) {
        return;                 /* polled by its driver */
    }
    qemu_set_irq(s->irq, (s->ctrl & CTRL_INTEN) && (s->ctrl & CTRL_DONE));
}

/* One transaction: address byte, n data bytes.  False if not acked. */
static bool ak_i2c_xfer(AkI2cState *s, uint8_t ab, unsigned n, bool stop)
{
    bool read = ab & 1;
    unsigned i;
    bool nak;

    /*
     * A new address phase every time, as the drivers issue the address
     * per message; a held bus (no STOP) gets a repeated start.
     */
    nak = i2c_start_transfer(s->bus, ab >> 1, read) != 0;
    s->in_transfer = !nak;
    for (i = 0; i < MIN(n, AK_I2C_DATA); i++) {
        if (read) {
            s->data[i] = nak ? 0xFF : i2c_recv(s->bus);
        } else if (!nak) {
            i2c_send(s->bus, s->data[i]);
        }
    }
    if (stop && s->in_transfer) {
        i2c_end_transfer(s->bus);
        s->in_transfer = false;
    }
    return !nak;
}

static void ak_i2c_run(AkI2cState *s)
{
    ak_i2c_xfer(s, s->cmd[0] & 0xFF, CTRL_COUNT(s->ctrl),
                !(s->ctrl & CTRL_NOSTOP));
    s->ctrl = (s->ctrl & ~CTRL_START) | CTRL_DONE;
}

static void ak_i2c_v2_run(AkI2cState *s)
{
    unsigned n = V2_COUNT(s->ctrl) ? V2_COUNT(s->ctrl) : AK_I2C_DATA;
    bool ack = ak_i2c_xfer(s, s->cmd[0] & 0xFF, n, true);

    s->ctrl = (s->ctrl & ~(V2_WRITE_GO | V2_READ_GO | V2_NAK)) | V2_DONE |
              (ack ? 0 : V2_NAK);
}

static uint64_t ak_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    AkI2cState *s = opaque;

    if (s->layout == 2) {
        if (offset == V2_ADDR) {
            return s->cmd[0];
        }
        if (offset >= V2_DATA0 && offset < V2_DATA0 + AK_I2C_DATA) {
            return ldl_le_p(s->data + (offset - V2_DATA0));
        }
        if (offset != R_CTRL) {
            return s->cmd[1 + (offset & 4) / 4];
        }
    }
    if (offset == R_CTRL) {
        return s->ctrl;
    }
    if (offset >= R_CMD1 && offset < R_CMD1 + 16) {
        return s->cmd[(offset - R_CMD1) / 4];
    }
    if (offset >= R_DATA0 && offset < R_DATA0 + AK_I2C_DATA) {
        return ldl_le_p(s->data + (offset - R_DATA0));
    }
    qemu_log_mask(LOG_UNIMP, "ak-i2c: read of 0x%02" HWADDR_PRIx "\n",
                  offset);
    return 0;
}

static void ak_i2c_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AkI2cState *s = opaque;

    if (s->layout == 2) {
        if (offset == R_CTRL) {
            s->ctrl = val;
            if ((val & (V2_WRITE_GO | V2_READ_GO)) && !(val & V2_DONE)) {
                ak_i2c_v2_run(s);
            }
        } else if (offset == V2_ADDR) {
            s->cmd[0] = val;
        } else if (offset >= V2_DATA0 && offset < V2_DATA0 + AK_I2C_DATA) {
            stl_le_p(s->data + (offset - V2_DATA0), val);
        } else {
            s->cmd[1 + (offset & 4) / 4] = val;
        }
        return;
    }
    if (offset == R_CTRL) {
        s->ctrl = val;
        if (val & CTRL_START) {
            ak_i2c_run(s);
        }
        ak_i2c_update_irq(s);
        return;
    }
    if (offset >= R_CMD1 && offset < R_CMD1 + 16) {
        s->cmd[(offset - R_CMD1) / 4] = val;
        return;
    }
    if (offset >= R_DATA0 && offset < R_DATA0 + AK_I2C_DATA) {
        stl_le_p(s->data + (offset - R_DATA0), val);
        return;
    }
    qemu_log_mask(LOG_UNIMP, "ak-i2c: write 0x%08" PRIx64 " to 0x%02"
                  HWADDR_PRIx "\n", val, offset);
}

static const MemoryRegionOps ak_i2c_ops = {
    .read = ak_i2c_read,
    .write = ak_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ak_i2c_reset(DeviceState *dev)
{
    AkI2cState *s = AK_I2C(dev);

    if (s->in_transfer) {
        i2c_end_transfer(s->bus);
    }
    s->in_transfer = false;
    s->ctrl = 0;
    memset(s->cmd, 0, sizeof(s->cmd));
    memset(s->data, 0, sizeof(s->data));
    ak_i2c_update_irq(s);
}

static const Property ak_i2c_properties[] = {
    DEFINE_PROP_UINT32("layout", AkI2cState, layout, 1),
};

static void ak_i2c_init(Object *obj)
{
    AkI2cState *s = AK_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ak_i2c_ops, s, TYPE_AK_I2C,
                          0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_ak_i2c = {
    .name = TYPE_AK_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, AkI2cState),
        VMSTATE_UINT32_ARRAY(cmd, AkI2cState, 4),
        VMSTATE_UINT8_ARRAY(data, AkI2cState, AK_I2C_DATA),
        VMSTATE_BOOL(in_transfer, AkI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void ak_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ak_i2c;
    device_class_set_legacy_reset(dc, ak_i2c_reset);
    device_class_set_props(dc, ak_i2c_properties);
}

static const TypeInfo ak_i2c_info = {
    .name          = TYPE_AK_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AkI2cState),
    .instance_init = ak_i2c_init,
    .class_init    = ak_i2c_class_init,
};

static void ak_i2c_register_types(void)
{
    type_register_static(&ak_i2c_info);
}

type_init(ak_i2c_register_types)
