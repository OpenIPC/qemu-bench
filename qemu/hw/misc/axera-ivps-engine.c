/*
 * Axera AX620E IVPS processing engines: VPP @ 0x04403000, GDC @ 0x04404000,
 * TDP @ 0x04405000, and the DPU @ 0x04407000
 *
 * The camera stack's IVPS module drives these through the closed
 * ax_vpp.ko/ax_gdc.ko drivers as "cpucmd" lists: tables of
 * (value, register offset) pairs, one batch per tile/stage. The driver
 * writes a batch whose last store kicks the engine, then waits for the
 * engine's done interrupt; the IRQ handler acknowledges it and writes the
 * next batch (vpp_cpucmd_execute_*() / gdc_tile_cpucmd_execute_v2()),
 * and only wakes the IVPS thread once the list is exhausted. Without a
 * done interrupt the very first job never finishes and IVPS output stays
 * empty forever -- the App's XmCap_Vpss_getChnFrame() then fails with
 * 0x800d0120 (AX_ID_IVPS, AX_ERR_BUF_EMPTY) for the whole boot.
 *
 * Protocol, from disassembling the real AArch64 ax_vpp.ko/ax_gdc.ko IRQ
 * handlers shipped in the AX620E firmware:
 *
 *   VPP: four channels, each with its own kick register and done bit in
 *        the status register 0x888; each IRQ handler acks its own bit by
 *        writing it to 0x8b4 (write-1-to-clear). The list builders queue
 *        "ack, enable, kick" as each batch's last three stores:
 *          v0   enable 0x890, kick 0x004, done bit 0
 *          v1   enable 0x894, kick 0x038, done bit 3
 *          isp  enable 0x898, kick 0x06c, done bit 6
 *          gdc  enable 0x89c, kick 0x07c, done bit 9 (the GDC-linked path)
 *   TDP: status 0x004, bit 0 = done (bits 1/2 are other events);
 *        tdpdev_hw_start() writes 0x010=3 (ack), 0x00c=2 (mask), then
 *        kicks with 0x01c=1, and tdp_irq() acks by writing the status
 *        bits to 0x010 (write-1-to-clear).
 *   DPU (vo@4407000, the OSD/overlay stage IVPS runs on every output
 *        frame): ax620e_draw_start() writes a (offset, value) list whose
 *        last two stores are 0x024=1 and the kick 0x028=1; dpu_irq() reads
 *        status 0x004 (bit 0 = draw done, routed to draw_irq_handler) and
 *        acks by writing the same value to 0x014. Without the done
 *        interrupt each draw only finishes on the driver's own timeout,
 *        so a frame or two trickles out and VENC starves.
 *   GDC: status 0xd8, bits [31:16], bit 16 = frame done; 0xd4 bit 16 is
 *        the interrupt enable (written 0x10000 at init and before each
 *        kick) and writing 0x10001 there acks. Kicked by writing 1 to
 *        0x06c (last store of the batch).
 *
 * Every register is plain storage (the drivers read back what they
 * wrote); only the kick, status and ack registers have behaviour. A kick
 * completes after a short delay rather than synchronously, so the done
 * interrupt never arrives before the driver has finished its own
 * bookkeeping for the kick.
 *
 * Pixel work is not modelled yet: a job completes without touching its
 * destination buffer.
 *
 * Set AXERA_IVPS_DEBUG=1 to log every register write (the way the kick
 * registers above were found).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_AXERA_IVPS_ENGINE "axera-ivps-engine"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraIvpsEngineState, AXERA_IVPS_ENGINE)

#define ENGINE_SIZE   0x1000
#define ENGINE_DELAY_NS (200 * SCALE_US)
#define ENGINE_KICKS    4

enum {
    ACK_W1C = 0,        /* VPP: write status bits to the ack register */
    ACK_BIT0_ENABLE = 1 /* GDC: ack = bit 0, enable = bits [31:16] */
};

struct AxeraIvpsEngineState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[ENGINE_SIZE / 4];
    uint32_t status;
    uint32_t pending;
    bool debug;

    char *name;
    uint32_t kick_off[ENGINE_KICKS];
    uint32_t done_bits[ENGINE_KICKS];
    uint32_t status_off;
    uint32_t ack_off;
    uint32_t ack_mode;
};

static void engine_update_irq(AxeraIvpsEngineState *s)
{
    uint32_t live = s->status;

    if (s->ack_mode == ACK_BIT0_ENABLE) {
        live &= s->regs[s->ack_off / 4] & 0xffff0000u;
    }
    qemu_set_irq(s->irq, live != 0);
}

static void engine_complete(void *opaque)
{
    AxeraIvpsEngineState *s = opaque;

    s->status |= s->pending;
    s->pending = 0;
    engine_update_irq(s);
}

static uint64_t engine_read(void *opaque, hwaddr off, unsigned size)
{
    AxeraIvpsEngineState *s = opaque;

    if (off == s->status_off) {
        return s->status;
    }
    return s->regs[off / 4];
}

static void engine_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    AxeraIvpsEngineState *s = opaque;

    if (s->debug) {
        qemu_log("%s: [0x%03" HWADDR_PRIx "] = 0x%08" PRIx64 "\n",
                 s->name, off, val);
    }
    s->regs[off / 4] = val;

    if (off == s->ack_off) {
        if (s->ack_mode == ACK_W1C) {
            s->status &= ~(uint32_t)val;
        } else if (val & 1) {
            s->status = 0;
        }
        engine_update_irq(s);
    }
    for (int k = 0; k < ENGINE_KICKS; k++) {
        if (s->done_bits[k] && off == s->kick_off[k] && (val & 1)) {
            s->pending |= s->done_bits[k];
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      ENGINE_DELAY_NS);
        }
    }
}

static const MemoryRegionOps engine_ops = {
    .read = engine_read,
    .write = engine_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void engine_reset(DeviceState *dev)
{
    AxeraIvpsEngineState *s = AXERA_IVPS_ENGINE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    s->pending = 0;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void engine_realize(DeviceState *dev, Error **errp)
{
    AxeraIvpsEngineState *s = AXERA_IVPS_ENGINE(dev);
    const char *dbg = getenv("AXERA_IVPS_DEBUG");

    s->debug = dbg && dbg[0] == '1';
    memory_region_init_io(&s->iomem, OBJECT(s), &engine_ops, s,
                          s->name ? s->name : TYPE_AXERA_IVPS_ENGINE,
                          ENGINE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, engine_complete, s);
}

static const Property engine_props[] = {
    DEFINE_PROP_STRING("name", AxeraIvpsEngineState, name),
    DEFINE_PROP_UINT32("kick-off0", AxeraIvpsEngineState, kick_off[0], 0),
    DEFINE_PROP_UINT32("kick-off1", AxeraIvpsEngineState, kick_off[1], 0),
    DEFINE_PROP_UINT32("kick-off2", AxeraIvpsEngineState, kick_off[2], 0),
    DEFINE_PROP_UINT32("kick-off3", AxeraIvpsEngineState, kick_off[3], 0),
    DEFINE_PROP_UINT32("done-bits0", AxeraIvpsEngineState, done_bits[0], 0),
    DEFINE_PROP_UINT32("done-bits1", AxeraIvpsEngineState, done_bits[1], 0),
    DEFINE_PROP_UINT32("done-bits2", AxeraIvpsEngineState, done_bits[2], 0),
    DEFINE_PROP_UINT32("done-bits3", AxeraIvpsEngineState, done_bits[3], 0),
    DEFINE_PROP_UINT32("status-off", AxeraIvpsEngineState, status_off, 0),
    DEFINE_PROP_UINT32("ack-off", AxeraIvpsEngineState, ack_off, 0),
    DEFINE_PROP_UINT32("ack-mode", AxeraIvpsEngineState, ack_mode, 0),
};

static void engine_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = engine_realize;
    device_class_set_legacy_reset(dc, engine_reset);
    device_class_set_props(dc, engine_props);
}

static const TypeInfo engine_info = {
    .name          = TYPE_AXERA_IVPS_ENGINE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraIvpsEngineState),
    .class_init    = engine_class_init,
};

static void engine_register_types(void)
{
    type_register_static(&engine_info);
}

type_init(engine_register_types)
