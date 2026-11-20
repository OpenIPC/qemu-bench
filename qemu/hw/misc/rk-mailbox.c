/*
 * Rockchip mailbox ("rockchip,rk3368-mailbox"; RV1106: 0xFF5C0000, one
 * interrupt for its four channels), between the Cortex-A7 ("A") and the
 * RISC-V MCU ("B").
 *
 * Register file, from the kernel's drivers/mailbox/rockchip-mailbox.c:
 *
 *   0x00 A2B_INTEN   0x04 A2B_STATUS (write 1 to clear)
 *   0x08 + n * 8 A2B_CMD(n)   0x0c + n * 8 A2B_DAT(n): a DAT write posts
 *   0x28 B2A_INTEN   0x2c B2A_STATUS (write 1 to clear)
 *   0x30 + n * 8 B2A_CMD(n)   0x34 + n * 8 B2A_DAT(n)
 *
 * The A7 sees B2A; its interrupt is B2A_STATUS & B2A_INTEN.
 *
 * The MCU's firmware is not run.  On thunder-boot boards it brings the
 * sensor up and then reports to Linux on channel 1, cmd 0xf00d ("MCU
 * status") data 0xdeadbeef ("done"), which the kernel's thunder-boot
 * service (drivers/soc/rockchip/rockchip_thunderboot_service.c) waits
 * for: until then the I2C, ISP and CIF drivers it shares with the MCU
 * keep their interrupts off, and every sensor access times out.  The
 * stand-in posts that message at reset on the channel in "mcu-done-chan"
 * (-1: none), so it is waiting when Linux enables the channel.  What the
 * A7 posts to the MCU is kept and never answered.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_MAILBOX "rk-mailbox"
OBJECT_DECLARE_SIMPLE_TYPE(RkMailboxState, RK_MAILBOX)

#define NCHAN           4
#define R_A2B_INTEN     0x00
#define R_A2B_STATUS    0x04
#define R_A2B_MSG       0x08
#define R_B2A_INTEN     0x28
#define R_B2A_STATUS    0x2c
#define R_B2A_MSG       0x30
#define REGS_SIZE       0x200

#define CMD_MCU_STATUS  0x0000f00d
#define MCU_STATUS_DONE 0xdeadbeef

struct RkMailboxState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t a2b_inten, a2b_status;
    uint32_t b2a_inten, b2a_status;
    uint32_t a2b[NCHAN * 2];        /* cmd, data per channel */
    uint32_t b2a[NCHAN * 2];
    int32_t mcu_done_chan;
};

static void rk_mailbox_update_irq(RkMailboxState *s)
{
    qemu_set_irq(s->irq, !!(s->b2a_status & s->b2a_inten));
}

static uint64_t rk_mailbox_read(void *opaque, hwaddr off, unsigned size)
{
    RkMailboxState *s = opaque;

    switch (off) {
    case R_A2B_INTEN:
        return s->a2b_inten;
    case R_A2B_STATUS:
        return s->a2b_status;
    case R_A2B_MSG ... R_A2B_MSG + NCHAN * 8 - 4:
        return s->a2b[(off - R_A2B_MSG) / 4];
    case R_B2A_INTEN:
        return s->b2a_inten;
    case R_B2A_STATUS:
        return s->b2a_status;
    case R_B2A_MSG ... R_B2A_MSG + NCHAN * 8 - 4:
        return s->b2a[(off - R_B2A_MSG) / 4];
    }
    return 0;
}

static void rk_mailbox_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    RkMailboxState *s = opaque;

    switch (off) {
    case R_A2B_INTEN:
        s->a2b_inten = val & 0xf;
        break;
    case R_A2B_STATUS:
        s->a2b_status &= ~val;
        break;
    case R_A2B_MSG ... R_A2B_MSG + NCHAN * 8 - 4:
        s->a2b[(off - R_A2B_MSG) / 4] = val;
        if (off & 4) {
            /* the data word posts the message to the (absent) MCU */
            s->a2b_status |= 1u << ((off - R_A2B_MSG) / 8);
            qemu_log_mask(LOG_UNIMP, "rk-mailbox: message to the MCU on "
                          "channel %d: cmd 0x%x data 0x%x\n",
                          (int)(off - R_A2B_MSG) / 8,
                          s->a2b[(off - R_A2B_MSG) / 4 - 1], (uint32_t)val);
        }
        break;
    case R_B2A_INTEN:
        s->b2a_inten = val & 0xf;
        break;
    case R_B2A_STATUS:
        s->b2a_status &= ~val;
        break;
    case R_B2A_MSG ... R_B2A_MSG + NCHAN * 8 - 4:
        s->b2a[(off - R_B2A_MSG) / 4] = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rk-mailbox: write 0x%" HWADDR_PRIx
                      "\n", off);
        return;
    }
    rk_mailbox_update_irq(s);
}

static const MemoryRegionOps rk_mailbox_ops = {
    .read = rk_mailbox_read,
    .write = rk_mailbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_mailbox_reset(DeviceState *dev)
{
    RkMailboxState *s = RK_MAILBOX(dev);
    int c = s->mcu_done_chan;

    s->a2b_inten = s->a2b_status = s->b2a_inten = s->b2a_status = 0;
    memset(s->a2b, 0, sizeof(s->a2b));
    memset(s->b2a, 0, sizeof(s->b2a));
    if (c >= 0 && c < NCHAN) {
        s->b2a[c * 2] = CMD_MCU_STATUS;
        s->b2a[c * 2 + 1] = MCU_STATUS_DONE;
        s->b2a_status = 1u << c;
    }
}

static void rk_mailbox_init(Object *obj)
{
    RkMailboxState *s = RK_MAILBOX(obj);

    memory_region_init_io(&s->iomem, obj, &rk_mailbox_ops, s,
                          TYPE_RK_MAILBOX, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property rk_mailbox_properties[] = {
    DEFINE_PROP_INT32("mcu-done-chan", RkMailboxState, mcu_done_chan, -1),
};

static const VMStateDescription rk_mailbox_vmstate = {
    .name = TYPE_RK_MAILBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(a2b_inten, RkMailboxState),
        VMSTATE_UINT32(a2b_status, RkMailboxState),
        VMSTATE_UINT32(b2a_inten, RkMailboxState),
        VMSTATE_UINT32(b2a_status, RkMailboxState),
        VMSTATE_UINT32_ARRAY(a2b, RkMailboxState, NCHAN * 2),
        VMSTATE_UINT32_ARRAY(b2a, RkMailboxState, NCHAN * 2),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_mailbox_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_mailbox_reset);
    device_class_set_props(dc, rk_mailbox_properties);
    dc->vmsd = &rk_mailbox_vmstate;
}

static const TypeInfo rk_mailbox_info = {
    .name = TYPE_RK_MAILBOX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkMailboxState),
    .instance_init = rk_mailbox_init,
    .class_init = rk_mailbox_class_init,
};

static void rk_mailbox_register_types(void)
{
    type_register_static(&rk_mailbox_info);
}

type_init(rk_mailbox_register_types)
