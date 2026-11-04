/*
 * Faraday FTIIC010 I2C master, as on Novatek's NA51068 (NT98321 / NT98323):
 * six at 0xFE300000 + 0x20000 n (SPI 21, 36, 63, 66, 86, 19),
 * drivers/i2c/busses/i2c-ftiic010.{c,h}:
 *
 *   0x00 CR    bit 0 RST (self-clearing), 1 I2C_EN, 2 SCL_EN, 3 GC_EN,
 *              4 START, 5 STOP, 6 NAK (send NACK after this byte),
 *              7 TB_EN (transfer one byte, self-clearing), 9..14 the
 *              interrupt enables (DR, BERR, STOP, SAM, AL, START),
 *              15 / 16 drive SCL / SDA low, 27 BC (bus clear: nine clocks,
 *              self-clearing)
 *   0x04 SR    bit 2 I2C busy, 3 bus busy, 5 DR (byte done), 6 BERR (the
 *              byte was not acknowledged), 7 STOP sent, 13 bus free;
 *              write 1 to clear
 *   0x08 CDR   0x0C DR  the byte to send, or the byte received
 *   0x10 SAR   0x14 TGSR  0x18 BMR  bit 0 SDA in, 1 SCL in (both high)
 *   0x1C..0x2C SMBus timing (storage)  0x30 REV  0x34 FEAT
 *
 * The driver does one byte per TB_EN and sleeps on the DR (or BERR)
 * interrupt.  A byte with START is an address: the slave either answers
 * (DR) or not (BERR).  Bytes after it go to or come from that slave in
 * the direction the address said; STOP ends the transfer once the byte is
 * done.  Every transfer completes at once.
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

#define TYPE_NVT_FTIIC010 "nvt-ftiic010"
OBJECT_DECLARE_SIMPLE_TYPE(NvtFtiicState, NVT_FTIIC010)

#define FTIIC_CR            0x00
#define FTIIC_SR            0x04
#define FTIIC_CDR           0x08
#define FTIIC_DR            0x0C
#define FTIIC_SAR           0x10
#define FTIIC_TGSR          0x14
#define FTIIC_BMR           0x18
#define FTIIC_REV           0x30
#define FTIIC_FEAT          0x34
#define FTIIC_NREGS         (0x40 / 4)

#define CR_RST              (1u << 0)
#define CR_START            (1u << 4)
#define CR_STOP             (1u << 5)
#define CR_NAK              (1u << 6)
#define CR_TB_EN            (1u << 7)
#define CR_INTR_SHIFT       9       /* DR, BERR, STOP, SAM, AL, START */
#define CR_BC               (1u << 27)
#define CR_SELF_CLEAR       (CR_RST | CR_TB_EN | CR_BC)

#define SR_I2CBUSY          (1u << 2)
#define SR_BUSBUSY          (1u << 3)
#define SR_DR               (1u << 5)
#define SR_BERR             (1u << 6)
#define SR_STOP             (1u << 7)
#define SR_BUSFREE          (1u << 13)
#define SR_W1C              0x1FE0  /* DR .. SCL timeout */
/* SR bits 5..10 against their enables at CR bits 9..14 */
#define SR_INTR_SHIFT       5
#define SR_INTR_MASK        (0x3Fu << SR_INTR_SHIFT)

#define BMR_LINES_HIGH      3
#define FTIIC_REVISION      0x00011200

struct NvtFtiicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    uint32_t regs[FTIIC_NREGS];
    bool active;            /* a transfer has been started */
    bool reading;
};

static void nvt_ftiic_update(NvtFtiicState *s)
{
    uint32_t cr = s->regs[FTIIC_CR / 4], sr = s->regs[FTIIC_SR / 4];
    uint32_t en = (cr >> CR_INTR_SHIFT) << SR_INTR_SHIFT;

    qemu_set_irq(s->irq, !!(sr & en & SR_INTR_MASK));
}

static void nvt_ftiic_stop(NvtFtiicState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
    s->regs[FTIIC_SR / 4] |= SR_STOP;
}

/* One TB_EN: the byte in DR, or one into it. */
static void nvt_ftiic_byte(NvtFtiicState *s, uint32_t cr)
{
    uint32_t *sr = &s->regs[FTIIC_SR / 4];
    uint8_t data = s->regs[FTIIC_DR / 4];
    bool nack = false;

    if (cr & CR_START) {
        /* a repeated START ends nothing first: i2c_start_transfer copes */
        s->reading = data & 1;
        nack = i2c_start_transfer(s->bus, data >> 1, s->reading) != 0;
        s->active = !nack;
        if (nack) {
            i2c_end_transfer(s->bus);
        }
    } else if (!s->active) {
        nack = true;
    } else if (s->reading) {
        s->regs[FTIIC_DR / 4] = i2c_recv(s->bus);
        if (cr & CR_NAK) {
            i2c_nack(s->bus);
        }
    } else {
        nack = i2c_send(s->bus, data) != 0;
    }
    *sr |= nack ? SR_BERR : SR_DR;
    if (cr & CR_STOP) {
        nvt_ftiic_stop(s);
    }
}

static uint64_t nvt_ftiic_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtFtiicState *s = opaque;

    switch (offset) {
    case FTIIC_SR:
        return (s->regs[FTIIC_SR / 4] & SR_W1C) |
               (s->active ? SR_BUSBUSY : SR_BUSFREE);
    case FTIIC_BMR:
        return BMR_LINES_HIGH;
    case FTIIC_REV:
        return FTIIC_REVISION;
    case FTIIC_FEAT:
        return 0;
    default:
        if (offset < FTIIC_NREGS * 4) {
            return s->regs[offset / 4];
        }
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftiic010: read 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
}

static void nvt_ftiic_reset_regs(NvtFtiicState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
    memset(s->regs, 0, sizeof(s->regs));
}

static void nvt_ftiic_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    NvtFtiicState *s = opaque;

    switch (offset) {
    case FTIIC_CR:
        if (val & CR_RST) {
            nvt_ftiic_reset_regs(s);
            break;
        }
        s->regs[FTIIC_CR / 4] = val & ~CR_SELF_CLEAR;
        if (val & CR_TB_EN) {
            nvt_ftiic_byte(s, val);
        } else if ((val & CR_STOP) && s->active) {
            nvt_ftiic_stop(s);
        }
        break;
    case FTIIC_SR:
        s->regs[FTIIC_SR / 4] &= ~(val & SR_W1C);
        break;
    case FTIIC_DR:
        s->regs[FTIIC_DR / 4] = val & 0xFF;
        break;
    case FTIIC_BMR:
    case FTIIC_REV:
    case FTIIC_FEAT:
        return;
    default:
        if (offset >= FTIIC_NREGS * 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "nvt-ftiic010: write 0x%"
                          HWADDR_PRIx "\n", offset);
            return;
        }
        s->regs[offset / 4] = val;
        break;
    }
    nvt_ftiic_update(s);
}

static const MemoryRegionOps nvt_ftiic_ops = {
    .read = nvt_ftiic_read,
    .write = nvt_ftiic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_ftiic_reset(DeviceState *dev)
{
    NvtFtiicState *s = NVT_FTIIC010(dev);

    nvt_ftiic_reset_regs(s);
    qemu_irq_lower(s->irq);
}

static void nvt_ftiic_init(Object *obj)
{
    NvtFtiicState *s = NVT_FTIIC010(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ftiic_ops, s,
                          TYPE_NVT_FTIIC010, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const VMStateDescription vmstate_nvt_ftiic = {
    .name = TYPE_NVT_FTIIC010,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtFtiicState, FTIIC_NREGS),
        VMSTATE_BOOL(active, NvtFtiicState),
        VMSTATE_BOOL(reading, NvtFtiicState),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_ftiic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_ftiic_reset);
    dc->vmsd = &vmstate_nvt_ftiic;
}

static const TypeInfo nvt_ftiic_info = {
    .name          = TYPE_NVT_FTIIC010,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtFtiicState),
    .instance_init = nvt_ftiic_init,
    .class_init    = nvt_ftiic_class_init,
};

static void nvt_ftiic_register_types(void)
{
    type_register_static(&nvt_ftiic_info);
}

type_init(nvt_ftiic_register_types)
