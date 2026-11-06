/*
 * Ambarella IDC, the I2C master, one byte at a time.
 *
 * A transfer is a conversation through CTRL.  Software puts the address in
 * DATA and writes START; the controller raises IF, with CTRL's ACK bit set
 * if the address was not acknowledged.  Each CTRL write that follows clears
 * IF and moves one byte: out of DATA when writing, into DATA when reading
 * (ACK here means "do not acknowledge this one", for the last byte), until
 * one carries STOP, which raises nothing.
 *
 * The named input "bus-held" keeps every address from being acknowledged
 * while it is high.  The machine uses it for a sensor in reset: nothing else
 * is on that bus here, so the whole bus goes quiet rather than one address.
 *
 * The FIFO path (FMCTRL / FMDATA) is the driver's turbo mode, which only
 * the amb,turbo-mode device tree property turns on, and is not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_AMB_IDC "amb-idc"
OBJECT_DECLARE_SIMPLE_TYPE(AmbIdcState, AMB_IDC)

#define R_ENR           0x00
#define R_CTRL          0x04
#define R_DATA          0x08
#define R_STS           0x0c
#define R_PSLL          0x10
#define R_PSLH          0x14
#define R_FMCTRL        0x18
#define R_FMDATA        0x1c
#define R_DUTY          0x24
#define R_STRETCH       0x28

#define CTRL_STOP       0x08
#define CTRL_START      0x04
#define CTRL_IF         0x02
#define CTRL_ACK        0x01

#define STS_FIFO_EMP    0x04

struct AmbIdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    uint32_t enr;
    uint32_t ctrl;
    uint32_t data;
    uint32_t psll;
    uint32_t pslh;
    uint32_t duty;
    uint32_t stretch;
    bool active;
    bool reading;
    bool held;
};

static void amb_idc_done(AmbIdcState *s, bool nack)
{
    s->ctrl = CTRL_IF | (nack ? CTRL_ACK : 0);
    qemu_irq_raise(s->irq);
}

static void amb_idc_control(AmbIdcState *s, uint32_t val)
{
    qemu_irq_lower(s->irq);
    s->ctrl = 0;

    if (val & CTRL_STOP) {
        if (s->active) {
            i2c_end_transfer(s->bus);
        }
        s->active = false;
        return;
    }
    if (val & CTRL_START) {
        s->reading = s->data & 1;
        s->active = !s->held &&
                    !i2c_start_transfer(s->bus, (s->data & 0xff) >> 1,
                                        s->reading);
        amb_idc_done(s, !s->active);
        return;
    }
    if (!s->active) {
        amb_idc_done(s, true);
    } else if (s->reading) {
        s->data = i2c_recv(s->bus);
        if (val & CTRL_ACK) {
            i2c_nack(s->bus);
        }
        amb_idc_done(s, false);
    } else {
        amb_idc_done(s, i2c_send(s->bus, s->data & 0xff) != 0);
    }
}

static void amb_idc_set_held(void *opaque, int line, int level)
{
    AMB_IDC(opaque)->held = level;
}

static uint64_t amb_idc_read(void *opaque, hwaddr off, unsigned size)
{
    AmbIdcState *s = opaque;

    switch (off) {
    case R_ENR:
        return s->enr;
    case R_CTRL:
        return s->ctrl;
    case R_DATA:
        return s->data;
    case R_STS:
        return STS_FIFO_EMP;
    case R_PSLL:
        return s->psll;
    case R_PSLH:
        return s->pslh;
    case R_DUTY:
        return s->duty;
    case R_STRETCH:
        return s->stretch;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-idc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void amb_idc_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    AmbIdcState *s = opaque;

    switch (off) {
    case R_ENR:
        s->enr = val;
        break;
    case R_CTRL:
        amb_idc_control(s, val);
        break;
    case R_DATA:
        s->data = val;
        break;
    case R_PSLL:
        s->psll = val;
        break;
    case R_PSLH:
        s->pslh = val;
        break;
    case R_DUTY:
        s->duty = val;
        break;
    case R_STRETCH:
        s->stretch = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "amb-idc: write 0x%" HWADDR_PRIx "\n", off);
        break;
    }
}

static const MemoryRegionOps amb_idc_ops = {
    .read = amb_idc_read,
    .write = amb_idc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void amb_idc_reset(DeviceState *dev)
{
    AmbIdcState *s = AMB_IDC(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->active = s->reading = false;
    s->enr = s->ctrl = s->data = 0;
    s->psll = s->pslh = s->duty = s->stretch = 0;
    qemu_irq_lower(s->irq);
}

static void amb_idc_init(Object *obj)
{
    AmbIdcState *s = AMB_IDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &amb_idc_ops, s, TYPE_AMB_IDC,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    qdev_init_gpio_in_named(DEVICE(obj), amb_idc_set_held, "bus-held", 1);
}

static const VMStateDescription amb_idc_vmstate = {
    .name = TYPE_AMB_IDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(enr, AmbIdcState),
        VMSTATE_UINT32(ctrl, AmbIdcState),
        VMSTATE_UINT32(data, AmbIdcState),
        VMSTATE_UINT32(psll, AmbIdcState),
        VMSTATE_UINT32(pslh, AmbIdcState),
        VMSTATE_UINT32(duty, AmbIdcState),
        VMSTATE_UINT32(stretch, AmbIdcState),
        VMSTATE_BOOL(active, AmbIdcState),
        VMSTATE_BOOL(reading, AmbIdcState),
        VMSTATE_BOOL(held, AmbIdcState),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_idc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, amb_idc_reset);
    dc->vmsd = &amb_idc_vmstate;
}

static const TypeInfo amb_idc_info = {
    .name = TYPE_AMB_IDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbIdcState),
    .instance_init = amb_idc_init,
    .class_init = amb_idc_class_init,
};

static void amb_idc_register(void)
{
    type_register_static(&amb_idc_info);
}

type_init(amb_idc_register)
