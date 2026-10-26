/*
 * Grain Media FT3DNR200 3D noise reduction engine.
 *
 * The de-noiser at 0x9B700000 on the GM813x (module/ft3dnr200:
 * ft3dnr200.h, ft3dnr200_hw.c, ft3dnr200_drv.c), which the IP camera
 * graph puts between capture and the H.264 encoder.  It runs link lists
 * of 64-bit commands -- low word data, high word command -- held in its
 * own memory at 0x400-0xBFF, addressed absolutely: job blocks at 0x400 +
 * n * 0xC0, spatial and edge parameter blocks at 0x700 and 0xA00:
 *
 *   0x20000000           status slot, the first command of a job's block
 *   0x5E000000 | reg     write the data word to the register
 *   0x46000000 | reg     write its low halfword
 *   0x60000000 | addr    continue at addr
 *   0x80000000 | addr    end of the job: process the frame, then addr
 *   0                    stop
 *
 * The first job is fired through register 0 bit 24, with the engine at
 * 0x400; after that the driver queues each job by turning the stop
 * command the engine is parked on into the job's status slot, so the
 * engine resumes when that command's high word becomes non-zero.  At a
 * job's end the model copies the source frame (0x1E4) to the destination
 * (0x1F0) -- packed 4:2:2, the source size from 0x004 -- sets bit 0 of
 * the job's status slot, and raises frame done and list done (0x208
 * bits 3 and 7; the driver writes 0xC8 to clear) on interrupt 42.  The
 * reference, variance and motion buffers are the driver's own; the
 * temporal filter is a copy.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_GM_3DNR "gm-3dnr"
OBJECT_DECLARE_SIMPLE_TYPE(Gm3dnrState, GM_3DNR)

#define GM_3DNR_SIZE        0x1000
#define GM_3DNR_NREGS       (GM_3DNR_SIZE / 4)
#define GM_3DNR_LL_BASE     0x400
#define GM_3DNR_LL_END      0xC00
#define GM_3DNR_JOB_NS      2000000
#define GM_3DNR_MAX_CMDS    4096

#define R_CTRL              0x000
#define R_SRC_SIZE          0x004
#define R_SRC_ADDR          0x1E4
#define R_DST_ADDR          0x1F0
#define R_STATUS            0x208

#define CTRL_FIRE           (1u << 24)
#define STATUS_DMA_ACK      (1u << 0)
#define STATUS_DMA_OFF      (1u << 1)
#define STATUS_RESET        (1u << 2)
#define STATUS_FRAME_DONE   (1u << 3)
#define STATUS_BAD_CMD      (1u << 6)
#define STATUS_LIST_DONE    (1u << 7)

struct Gm3dnrState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[GM_3DNR_NREGS];
    bool running;           /* fired once: follows the lists */
    bool parked;            /* on the stop command at pc */
    uint32_t pc;
};

static void gm_3dnr_update(Gm3dnrState *s)
{
    qemu_set_irq(s->irq, (s->regs[R_STATUS / 4] &
                          (STATUS_FRAME_DONE | STATUS_LIST_DONE |
                           STATUS_BAD_CMD)) != 0);
}

static void gm_3dnr_frame(Gm3dnrState *s)
{
    const uint32_t size = s->regs[R_SRC_SIZE / 4];
    const size_t len = (size_t)extract32(size, 0, 16) *
                       extract32(size, 16, 16) * 2;
    g_autofree uint8_t *buf = NULL;

    if (!len || len > 64 * MiB) {
        return;
    }
    buf = g_malloc(len);
    dma_memory_read(&address_space_memory, s->regs[R_SRC_ADDR / 4], buf, len,
                    MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, s->regs[R_DST_ADDR / 4], buf, len,
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * Run the list from pc to the end of one job (scheduling the next step)
 * or to a stop command (parking).
 */
static void gm_3dnr_run(Gm3dnrState *s)
{
    uint32_t status_slot = 0;
    int n;

    for (n = 0; n < GM_3DNR_MAX_CMDS; n++) {
        uint32_t lo, hi, op, arg;

        if (s->pc < GM_3DNR_LL_BASE || s->pc + 8 > GM_3DNR_LL_END) {
            break;
        }
        lo = s->regs[s->pc / 4];
        hi = s->regs[s->pc / 4 + 1];
        op = hi >> 24;
        arg = hi & 0xFFFFFF;
        if (!hi) {
            s->parked = true;
            return;
        }
        switch (op & 0xE0) {
        case 0x20:                      /* status slot */
            status_slot = s->pc;
            s->pc += 8;
            break;
        case 0x40:                      /* register write */
            arg &= 0xFFC;
            if (arg >= GM_3DNR_LL_BASE) {
                break;
            }
            if (op == 0x46) {
                s->regs[arg / 4] = deposit32(s->regs[arg / 4], 0, 16, lo);
            } else {
                s->regs[arg / 4] = lo;
            }
            s->pc += 8;
            break;
        case 0x60:                      /* jump */
            s->pc = arg & 0xFFF;
            break;
        case 0x80:                      /* end of job */
            gm_3dnr_frame(s);
            if (status_slot) {
                s->regs[status_slot / 4 + 1] |= 1;
            }
            s->regs[R_STATUS / 4] |= STATUS_FRAME_DONE | STATUS_LIST_DONE;
            gm_3dnr_update(s);
            s->pc = arg & 0xFFF;
            s->parked = false;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                GM_3DNR_JOB_NS);
            return;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "gm-3dnr: bad command 0x%08x at 0x%x\n", hi, s->pc);
            s->regs[R_STATUS / 4] |= STATUS_BAD_CMD;
            gm_3dnr_update(s);
            s->running = false;
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "gm-3dnr: runaway list at 0x%x\n", s->pc);
    s->running = false;
}

static void gm_3dnr_step(void *opaque)
{
    Gm3dnrState *s = opaque;

    if (s->running) {
        gm_3dnr_run(s);
    }
}

static uint64_t gm_3dnr_read(void *opaque, hwaddr off, unsigned size)
{
    Gm3dnrState *s = opaque;

    return s->regs[off / 4];
}

static void gm_3dnr_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    Gm3dnrState *s = opaque;

    switch (off) {
    case R_CTRL:
        s->regs[0] = val & ~CTRL_FIRE;
        if (val & CTRL_FIRE) {
            s->running = true;
            s->parked = false;
            s->pc = GM_3DNR_LL_BASE;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                GM_3DNR_JOB_NS);
        }
        return;
    case R_STATUS:
        /* write 1 clears the events; reset completes, DMA stops at once */
        s->regs[off / 4] &= ~(val & (STATUS_FRAME_DONE | STATUS_LIST_DONE |
                                     STATUS_BAD_CMD | STATUS_DMA_ACK));
        if (val & STATUS_DMA_OFF) {
            s->regs[off / 4] |= STATUS_DMA_ACK;
        }
        if (val & STATUS_RESET) {
            timer_del(s->timer);
            s->running = s->parked = false;
        }
        gm_3dnr_update(s);
        return;
    }
    s->regs[off / 4] = val;
    /* a queued job makes the stop command the engine waits on its status */
    if (s->running && s->parked && (off & ~7) == s->pc &&
        s->regs[s->pc / 4 + 1]) {
        s->parked = false;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            GM_3DNR_JOB_NS);
    }
}

static const MemoryRegionOps gm_3dnr_ops = {
    .read = gm_3dnr_read,
    .write = gm_3dnr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void gm_3dnr_reset(DeviceState *dev)
{
    Gm3dnrState *s = GM_3DNR(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->running = s->parked = false;
    s->pc = GM_3DNR_LL_BASE;
    gm_3dnr_update(s);
}

static void gm_3dnr_realize(DeviceState *dev, Error **errp)
{
    Gm3dnrState *s = GM_3DNR(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &gm_3dnr_ops, s,
                          TYPE_GM_3DNR, GM_3DNR_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gm_3dnr_step, s);
}

static const VMStateDescription vmstate_gm_3dnr = {
    .name = TYPE_GM_3DNR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, Gm3dnrState),
        VMSTATE_UINT32_ARRAY(regs, Gm3dnrState, GM_3DNR_NREGS),
        VMSTATE_BOOL(running, Gm3dnrState),
        VMSTATE_BOOL(parked, Gm3dnrState),
        VMSTATE_UINT32(pc, Gm3dnrState),
        VMSTATE_END_OF_LIST()
    }
};

static void gm_3dnr_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = gm_3dnr_realize;
    device_class_set_legacy_reset(dc, gm_3dnr_reset);
    dc->vmsd = &vmstate_gm_3dnr;
}

static const TypeInfo gm_3dnr_info = {
    .name           = TYPE_GM_3DNR,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Gm3dnrState),
    .class_init     = gm_3dnr_class_init,
};

static void gm_3dnr_register_types(void)
{
    type_register_static(&gm_3dnr_info);
}

type_init(gm_3dnr_register_types)
