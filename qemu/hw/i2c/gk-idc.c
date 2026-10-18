/*
 * Goke GK710x I2C master ("IDC").
 *
 * Ambarella's IDC, at the offsets and bit positions the silicon has (the
 * vendor HAL shuffles both; include/hw/arm/goke.h):
 *
 *   0x00 ENR     bit 0 enable
 *   0x04 CTRL    bit 0 ACK, bit 1 IF, bit 2 START, bit 3 STOP
 *   0x08 DATA
 *   0x0C STS     bit 0 MODE: 1 = master receiver
 *   0x10 PSLL, 0x14 PSLH  SCL prescaler
 *   0x18 FMCTRL  bit 1 IF, bit 2 START, bit 3 STOP   (bulk-write FIFO)
 *   0x1C FMDATA  FIFO input
 *
 * Byte at a time, driven from the interrupt (i2c-goke_v1_00.c): DATA gets
 * the address byte and CTRL = START puts start + address on the bus.  Every
 * later CTRL write without START or STOP clocks one byte -- out of DATA
 * when transmitting, into DATA when receiving, the ACK bit then meaning
 * "answer this byte with NACK".  Each byte ends with IF set and the
 * interrupt raised; read back, ACK tells whether the slave acknowledged.
 * STOP ends the transfer and raises nothing; nor does a CTRL write with
 * no transfer open (the handler's IF clear after an unanswered address).
 *
 * Writes longer than the driver's bulk_write_num go through the FIFO:
 * FMCTRL = START, the address and data bytes into FMDATA, then FMCTRL = IF
 * (| STOP) sends them all, ACK and IF in CTRL reporting as above.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_GK_IDC "gk-idc"
OBJECT_DECLARE_SIMPLE_TYPE(GkIdcState, GK_IDC)

#define IDC_ENR         0x00
#define IDC_CTRL        0x04
#define IDC_DATA        0x08
#define IDC_STS         0x0C
#define IDC_PSLL        0x10
#define IDC_PSLH        0x14
#define IDC_FMCTRL      0x18
#define IDC_FMDATA      0x1C

#define CTRL_ACK        (1u << 0)
#define CTRL_IF         (1u << 1)
#define CTRL_START      (1u << 2)
#define CTRL_STOP       (1u << 3)
#define STS_MODE_RX     (1u << 0)

#define IDC_FIFO_SIZE   64

struct GkIdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t enr;
    uint32_t ctrl;      /* ACK and IF as last reported */
    uint32_t data;
    uint32_t sts;
    uint32_t psll, pslh;
    bool busy;          /* between START and STOP */

    uint8_t fifo[IDC_FIFO_SIZE];
    uint32_t fifo_len;
    bool fifo_start;
};

static void gk_idc_done(GkIdcState *s, bool acked)
{
    s->ctrl = CTRL_IF | (acked ? CTRL_ACK : 0);
    qemu_irq_raise(s->irq);
}

static void gk_idc_stop(GkIdcState *s)
{
    if (s->busy) {
        i2c_end_transfer(s->bus);
        s->busy = false;
    }
}

/* Start (or repeated start) with the address byte; true if acknowledged. */
static bool gk_idc_address(GkIdcState *s, uint8_t addr)
{
    const bool rx = addr & 1;

    s->sts = rx ? STS_MODE_RX : 0;
    if (i2c_start_transfer(s->bus, addr >> 1, rx)) {
        /* No one there: the bus is left idle (i2c.c ended it). */
        s->busy = false;
        return false;
    }
    s->busy = true;
    return true;
}

static void gk_idc_ctrl(GkIdcState *s, uint32_t val)
{
    qemu_irq_lower(s->irq);
    s->ctrl &= ~CTRL_IF;

    if (!(s->enr & 1)) {
        return;
    }
    if (val & CTRL_STOP) {
        gk_idc_stop(s);
        return;
    }
    if (val & CTRL_START) {
        gk_idc_done(s, gk_idc_address(s, s->data));
        return;
    }
    if (!s->busy) {
        /* Clearing IF after an address nobody answered: the driver writes
         * CTRL = 0 from its interrupt handler before the STOP. */
        return;
    }
    if (s->sts & STS_MODE_RX) {
        s->data = i2c_recv(s->bus);
        if (val & CTRL_ACK) {
            i2c_nack(s->bus);
        }
        gk_idc_done(s, true);
    } else {
        gk_idc_done(s, i2c_send(s->bus, s->data) == 0);
    }
}

static void gk_idc_fmctrl(GkIdcState *s, uint32_t val)
{
    uint32_t i = 0;
    bool acked = true;

    if (val & CTRL_START) {
        s->fifo_len = 0;
        s->fifo_start = true;
    }
    if (!(val & CTRL_IF)) {
        return;
    }
    qemu_irq_lower(s->irq);
    if (s->fifo_start && s->fifo_len) {
        acked = gk_idc_address(s, s->fifo[0]);
        s->fifo_start = false;
        i = 1;
    }
    for (; acked && i < s->fifo_len; i++) {
        acked = s->busy && i2c_send(s->bus, s->fifo[i]) == 0;
    }
    s->fifo_len = 0;
    if (val & CTRL_STOP) {
        gk_idc_stop(s);
    }
    gk_idc_done(s, acked);
}

static uint64_t gk_idc_read(void *opaque, hwaddr offset, unsigned size)
{
    GkIdcState *s = opaque;

    switch (offset) {
    case IDC_ENR:
        return s->enr;
    case IDC_CTRL:
        return s->ctrl;
    case IDC_DATA:
        return s->data;
    case IDC_STS:
        return s->sts;
    case IDC_PSLL:
        return s->psll;
    case IDC_PSLH:
        return s->pslh;
    default:
        return 0;
    }
}

static void gk_idc_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    GkIdcState *s = opaque;

    switch (offset) {
    case IDC_ENR:
        s->enr = val & 1;
        if (!s->enr) {
            gk_idc_stop(s);
        }
        break;
    case IDC_CTRL:
        gk_idc_ctrl(s, val);
        break;
    case IDC_DATA:
        s->data = val & 0xFF;
        break;
    case IDC_PSLL:
        s->psll = val & 0xFF;
        break;
    case IDC_PSLH:
        s->pslh = val & 0xFF;
        break;
    case IDC_FMCTRL:
        gk_idc_fmctrl(s, val);
        break;
    case IDC_FMDATA:
        if (s->fifo_len < IDC_FIFO_SIZE) {
            s->fifo[s->fifo_len++] = val;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "gk-idc: FIFO overflow\n");
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps gk_idc_ops = {
    .read = gk_idc_read,
    .write = gk_idc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void gk_idc_reset(DeviceState *dev)
{
    GkIdcState *s = GK_IDC(dev);

    gk_idc_stop(s);
    s->enr = s->ctrl = s->data = s->sts = 0;
    s->psll = s->pslh = 0;
    s->fifo_len = 0;
    s->fifo_start = false;
    qemu_irq_lower(s->irq);
}

static void gk_idc_init(Object *obj)
{
    GkIdcState *s = GK_IDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gk_idc_ops, s, TYPE_GK_IDC,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_gk_idc = {
    .name = TYPE_GK_IDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(enr, GkIdcState),
        VMSTATE_UINT32(ctrl, GkIdcState),
        VMSTATE_UINT32(data, GkIdcState),
        VMSTATE_UINT32(sts, GkIdcState),
        VMSTATE_UINT32(psll, GkIdcState),
        VMSTATE_UINT32(pslh, GkIdcState),
        VMSTATE_BOOL(busy, GkIdcState),
        VMSTATE_UINT8_ARRAY(fifo, GkIdcState, IDC_FIFO_SIZE),
        VMSTATE_UINT32(fifo_len, GkIdcState),
        VMSTATE_BOOL(fifo_start, GkIdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void gk_idc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, gk_idc_reset);
    dc->vmsd = &vmstate_gk_idc;
}

static const TypeInfo gk_idc_info = {
    .name          = TYPE_GK_IDC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GkIdcState),
    .instance_init = gk_idc_init,
    .class_init    = gk_idc_class_init,
};

static void gk_idc_register_types(void)
{
    type_register_static(&gk_idc_info);
}

type_init(gk_idc_register_types)
