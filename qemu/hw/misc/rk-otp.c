/*
 * Rockchip RV1126 OTP ("rockchip,rv1126-otp": 0xFF5C0000, and the secure
 * one OP-TEE drives at 0xFF5D0000), from the kernel's
 * drivers/nvmem/rockchip-otp.c (develop-4.19, rv1126_otp_*):
 *
 *   0x00 NVM_CEB    write 0: power the array up   -> ST bit 0
 *   0x04 NVM_RSTB   write 1: out of reset          -> ST bits 2, 3
 *                   (OP-TEE waits for (ST & 3) == 1 after CEB and for
 *                   (ST & 0xf) == 0xd after RSTB; Linux tests bits 0, 2)
 *   0x18 NVM_ST
 *   0x1c NVM_RADDR  byte address;  0x20 NVM_RSTART  1 starts a read
 *   0x24 NVM_RDATA  the byte;      0x30 READ_ST     0: read done
 *   0x34 NVM_PRADDR bit address;   0x38 NVM_PRLEN   bits - 1
 *   0x3c NVM_PRDATA bits to set;   0x44 NVM_PRSTART 1 starts a program
 *   0x48 NVM_PRSTATE 0: program done
 *
 * The array ("size" bytes, 512 by default: 4 Kbit) comes up
 * unprogrammed, all zero, and programming only sets bits, as on the
 * part.  Operations complete at once.  Without the model both OP-TEE
 * ("rk_otp_base_init timeout! OTP_NVM_ST=0x0") and Linux ("timeout
 * during set ceb", and the cpuinfo driver deferring on it) wait out
 * their polls.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_OTP "rk-otp"
OBJECT_DECLARE_SIMPLE_TYPE(RkOtpState, RK_OTP)

#define R_CEB       0x00
#define R_RSTB      0x04
#define R_ST        0x18
#define R_RADDR     0x1c
#define R_RSTART    0x20
#define R_RDATA     0x24
#define R_TRWH      0x28
#define R_READ_ST   0x30
#define R_PRADDR    0x34
#define R_PRLEN     0x38
#define R_PRDATA    0x3c
#define R_FAILTIME  0x40
#define R_PRSTART   0x44
#define R_PRSTATE   0x48
#define REGS_SIZE   0x1000

#define ST_CEB      (1u << 0)
#define ST_RSTB     (1u << 2)
#define ST_READY    (1u << 3)

struct RkOtpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t size;
    uint8_t *array;
    uint32_t st, raddr, rdata, praddr, prlen, prdata, trwh, failtime;
};

static uint64_t rk_otp_read(void *opaque, hwaddr off, unsigned size)
{
    RkOtpState *s = opaque;

    switch (off) {
    case R_ST:
        return s->st;
    case R_RADDR:
        return s->raddr;
    case R_RDATA:
        return s->rdata;
    case R_TRWH:
        return s->trwh;
    case R_READ_ST:
    case R_PRSTATE:
        return 0;                   /* done */
    case R_PRADDR:
        return s->praddr;
    case R_PRLEN:
        return s->prlen;
    case R_PRDATA:
        return s->prdata;
    case R_FAILTIME:
        return s->failtime;
    }
    return 0;
}

static void rk_otp_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkOtpState *s = opaque;

    switch (off) {
    case R_CEB:
        s->st = (val & 1) ? s->st & ~ST_CEB : s->st | ST_CEB;
        break;
    case R_RSTB:
        s->st = (val & 1) ? s->st | ST_RSTB | ST_READY
                          : s->st & ~(ST_RSTB | ST_READY);
        break;
    case R_RADDR:
        s->raddr = val;
        break;
    case R_RSTART:
        if (val & 1) {
            s->rdata = s->raddr < s->size ? s->array[s->raddr] : 0;
        }
        break;
    case R_TRWH:
        s->trwh = val;
        break;
    case R_PRADDR:
        s->praddr = val;
        break;
    case R_PRLEN:
        s->prlen = val;
        break;
    case R_PRDATA:
        s->prdata = val;
        break;
    case R_FAILTIME:
        s->failtime = val;
        break;
    case R_PRSTART:
        if (val & 1) {
            uint32_t i, n = (s->prlen & 0x1f) + 1;

            for (i = 0; i < n; i++) {
                uint32_t bit = s->praddr + i;

                if ((s->prdata >> i) & 1 && bit / 8 < s->size) {
                    s->array[bit / 8] |= 1u << (bit % 8);
                }
            }
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "rk-otp: write 0x%" HWADDR_PRIx "\n", off);
    }
}

static const MemoryRegionOps rk_otp_ops = {
    .read = rk_otp_read,
    .write = rk_otp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void rk_otp_reset(DeviceState *dev)
{
    RkOtpState *s = RK_OTP(dev);

    /* the array keeps what was programmed; the controller does not */
    s->st = s->raddr = s->rdata = 0;
    s->praddr = s->prlen = s->prdata = s->trwh = s->failtime = 0;
}

static void rk_otp_realize(DeviceState *dev, Error **errp)
{
    RkOtpState *s = RK_OTP(dev);

    s->array = g_malloc0(s->size);
    memory_region_init_io(&s->iomem, OBJECT(s), &rk_otp_ops, s, TYPE_RK_OTP,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property rk_otp_properties[] = {
    DEFINE_PROP_UINT32("size", RkOtpState, size, 512),
};

static const VMStateDescription rk_otp_vmstate = {
    .name = TYPE_RK_OTP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_VBUFFER_UINT32(array, RkOtpState, 1, NULL, size),
        VMSTATE_UINT32(st, RkOtpState),
        VMSTATE_UINT32(raddr, RkOtpState),
        VMSTATE_UINT32(rdata, RkOtpState),
        VMSTATE_UINT32(praddr, RkOtpState),
        VMSTATE_UINT32(prlen, RkOtpState),
        VMSTATE_UINT32(prdata, RkOtpState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_otp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rk_otp_realize;
    device_class_set_legacy_reset(dc, rk_otp_reset);
    device_class_set_props(dc, rk_otp_properties);
    dc->vmsd = &rk_otp_vmstate;
}

static const TypeInfo rk_otp_info = {
    .name = TYPE_RK_OTP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkOtpState),
    .class_init = rk_otp_class_init,
};

static void rk_otp_register_types(void)
{
    type_register_static(&rk_otp_info);
}

type_init(rk_otp_register_types)
