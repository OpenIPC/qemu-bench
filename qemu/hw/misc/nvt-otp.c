/*
 * Novatek NA51089 OTP / eFuse controller at 0xF0660000.
 *
 * The kernel reaches it only through a shipped object
 * (drivers/soc/nvt/otp/na51089/otp_protected.o_shipped); what it does, as
 * disassembled (efuse_open / efuse_read / efuse_readData / efuse_close):
 *
 *   0x04  word index of a read, [4:0]
 *   0x08  the word read
 *   0x20  control: bit 0 powers the array up (efuse_open), and then waits
 *         for STATUS bit 0; bit 4 starts a read and clears when it is done
 *   0x30  status: bit 0 the array is up, bit 1 a read finished (write 1
 *         to clear)
 *   0x50, 0x54  words the blob reads directly (indexes 8 and 4)
 *
 * The array reads as never programmed, all zero: efuse_readParamOps()
 * then reports no trim data and its callers (the USB PHY's
 * nvtim_init_usbhc(), Ethernet, DDR) keep their defaults.  Without the
 * status bits the blob spins forever, which is why the device tree's
 * nvt_otp node needs this model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_OTP "nvt-otp"
OBJECT_DECLARE_SIMPLE_TYPE(NvtOtpState, NVT_OTP)

#define OTP_REGS_SIZE   0x100
#define OTP_ADDR        0x04
#define OTP_DATA        0x08
#define OTP_CTRL        0x20
#define OTP_STATUS      0x30

#define CTRL_ENABLE     (1u << 0)
#define CTRL_READ       (1u << 4)
#define STS_READY       (1u << 0)
#define STS_READ_DONE   (1u << 1)

struct NvtOtpState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[OTP_REGS_SIZE / 4];
};

static uint64_t nvt_otp_read(void *opaque, hwaddr off, unsigned size)
{
    NvtOtpState *s = opaque;

    if (off == OTP_STATUS) {
        return (s->regs[OTP_STATUS / 4] & STS_READ_DONE) |
               (s->regs[OTP_CTRL / 4] & CTRL_ENABLE ? STS_READY : 0);
    }
    return s->regs[off / 4];
}

static void nvt_otp_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    NvtOtpState *s = opaque;

    switch (off) {
    case OTP_CTRL:
        if (val & CTRL_READ) {
            /* A blank array: every word reads 0, at once. */
            s->regs[OTP_DATA / 4] = 0;
            s->regs[OTP_STATUS / 4] |= STS_READ_DONE;
            val &= ~CTRL_READ;
        }
        s->regs[off / 4] = val;
        break;
    case OTP_STATUS:
        s->regs[off / 4] &= ~(val & STS_READ_DONE);
        break;
    case OTP_DATA:
        break;
    default:
        s->regs[off / 4] = val;
        break;
    }
}

static const MemoryRegionOps nvt_otp_ops = {
    .read = nvt_otp_read,
    .write = nvt_otp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_otp_reset(DeviceState *dev)
{
    NvtOtpState *s = NVT_OTP(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void nvt_otp_init(Object *obj)
{
    NvtOtpState *s = NVT_OTP(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_otp_ops, s, TYPE_NVT_OTP,
                          OTP_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_nvt_otp = {
    .name = TYPE_NVT_OTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtOtpState, OTP_REGS_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void nvt_otp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_otp_reset);
    dc->vmsd = &vmstate_nvt_otp;
}

static const TypeInfo nvt_otp_info = {
    .name = TYPE_NVT_OTP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtOtpState),
    .instance_init = nvt_otp_init,
    .class_init = nvt_otp_class_init,
};

static void nvt_otp_register_types(void)
{
    type_register_static(&nvt_otp_info);
}

type_init(nvt_otp_register_types)
