/*
 * Axera AX630C VeriSilicon VCMD command-queue engines: VENC (VC8000E),
 * JENC and VDEC all share the same VCMD front-end protocol (confirmed
 * from the open VC8000E driver: eswin/vc8000_vcmd_driver.c, dual
 * MIT/GPL VeriSilicon lineage, and its register field table
 * eswin/vcmdregistertable.h):
 *
 *   - WORK_STATE, byte offset 0x3C, bits[2:0], read-only:
 *     0 IDLE, 1 WORK, 2 STALL, 3 PEND, 4 ABORT.
 *   - CONTROL, byte offset 0x40 (VCMD_REGISTER_CONTROL_OFFSET), bit 0 is
 *     HWIF_VCMD_START_TRIGGER: write 1 to fetch and execute the queued
 *     command buffer, write 0 to abort.
 *   - INT_STATUS, byte offset 0x44 (VCMD_REGISTER_INT_STATUS_OFFSET),
 *     bit 0 is HWIF_VCMD_IRQ_ENDCMD: set by hardware on completion, and
 *     the guest ISR (hantrovcmd_isr) acks by writing the SAME value it
 *     just read back -- a plain write-1-to-clear, not a distinct ack
 *     register.
 *   - INT_CTL, byte offset 0x48 (VCMD_REGISTER_INT_CTL_OFFSET), bit 0 is
 *     HWIF_VCMD_IRQ_ENDCMD_EN: the interrupt mask.
 *
 * The real engine fetches a guest-built linked list of command-buffer
 * nodes (bi_list_node / cmdbuf_obj in the VCMD core) describing the
 * actual encode/decode job -- SPS/PPS, rate control, frame buffer
 * addresses. Decoding that format well enough to drive a real
 * libavcodec/libx264 encode was not attempted here (it is undocumented
 * VeriSilicon microcode, not a register map one can read off a
 * devicetree or a platform glue driver the way EIP-130's token
 * protocol could be); this model completes the handshake correctly
 * (probe, submit, wait, complete -- no hang, no timeout) without
 * producing real bitstream content. A good next step for real pixels
 * is cmdbuf_obj/bi_list_node's exact DMA layout, not this file.
 *
 * Completion fires from a timer, not synchronously inside the CONTROL
 * write: the guest's wait_cmdbuf_ready() (and the VCMD core generally)
 * is built around being woken by a real interrupt after the doorbell
 * write returns, the same race the project's other codec models
 * (sstar-vpu.c, nvt-h26xdec.c) avoid the same way.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/misc/axera-vcmd.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(AxeraVcmdState, AXERA_VCMD)

#define VCMD_MMIO_SIZE     0x10000
#define WORK_STATE_OFF     0x3c
#define CONTROL_OFF        0x40
#define INT_STATUS_OFF     0x44
#define INT_CTL_OFF        0x48
#define START_TRIGGER      BIT(0)
#define IRQ_ENDCMD         BIT(0)
#define IRQ_ENDCMD_EN      BIT(0)
#define WORK_STATE_IDLE    0
#define WORK_STATE_WORK    1

/* The real engine takes real time to fetch/execute a command list; a
 * short but nonzero delay keeps this observably async without slowing
 * boots down. */
#define AXERA_VCMD_LATENCY_NS (1 * SCALE_MS)

struct AxeraVcmdState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer completion_timer;

    uint32_t int_status;
    uint32_t int_ctl;
    uint32_t work_state;
};

static void axera_vcmd_update_irq(AxeraVcmdState *s)
{
    qemu_set_irq(s->irq, (s->int_status & s->int_ctl & IRQ_ENDCMD_EN) != 0);
}

static void axera_vcmd_complete(void *opaque)
{
    AxeraVcmdState *s = opaque;

    s->work_state = WORK_STATE_IDLE;
    s->int_status |= IRQ_ENDCMD;
    axera_vcmd_update_irq(s);
}

static uint64_t axera_vcmd_read(void *opaque, hwaddr offset, unsigned size)
{
    AxeraVcmdState *s = opaque;

    switch (offset) {
    case WORK_STATE_OFF:
        return s->work_state;
    case CONTROL_OFF:
        return s->work_state == WORK_STATE_WORK ? START_TRIGGER : 0;
    case INT_STATUS_OFF:
        return s->int_status;
    case INT_CTL_OFF:
        return s->int_ctl;
    default:
        return 0;
    }
}

static void axera_vcmd_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AxeraVcmdState *s = opaque;

    switch (offset) {
    case CONTROL_OFF:
        if (value & START_TRIGGER) {
            s->work_state = WORK_STATE_WORK;
            timer_mod(&s->completion_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     AXERA_VCMD_LATENCY_NS);
        } else {
            /* Abort: stop immediately, no completion interrupt. */
            timer_del(&s->completion_timer);
            s->work_state = WORK_STATE_IDLE;
        }
        break;
    case INT_STATUS_OFF:
        /* Write-1-to-clear: the guest ISR writes back exactly what it
         * read, acking every bit that was set. */
        s->int_status &= ~value;
        axera_vcmd_update_irq(s);
        break;
    case INT_CTL_OFF:
        s->int_ctl = value;
        axera_vcmd_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps axera_vcmd_ops = {
    .read = axera_vcmd_read,
    .write = axera_vcmd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void axera_vcmd_reset(DeviceState *dev)
{
    AxeraVcmdState *s = AXERA_VCMD(dev);

    timer_del(&s->completion_timer);
    s->int_status = 0;
    s->int_ctl = 0;
    s->work_state = WORK_STATE_IDLE;
    qemu_set_irq(s->irq, 0);
}

static void axera_vcmd_realize(DeviceState *dev, Error **errp)
{
    AxeraVcmdState *s = AXERA_VCMD(dev);

    timer_init_ns(&s->completion_timer, QEMU_CLOCK_VIRTUAL,
                 axera_vcmd_complete, s);
    memory_region_init_io(&s->iomem, OBJECT(s), &axera_vcmd_ops, s,
                          TYPE_AXERA_VCMD, VCMD_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void axera_vcmd_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = axera_vcmd_realize;
    device_class_set_legacy_reset(dc, axera_vcmd_reset);
}

static const TypeInfo axera_vcmd_info = {
    .name = TYPE_AXERA_VCMD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraVcmdState),
    .class_init = axera_vcmd_class_init,
};

static void axera_vcmd_register_types(void)
{
    type_register_static(&axera_vcmd_info);
}

type_init(axera_vcmd_register_types)
