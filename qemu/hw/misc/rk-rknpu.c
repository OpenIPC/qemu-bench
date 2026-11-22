/*
 * Rockchip RKNPU ("rockchip,rv1106-rknpu", RV1106 0xFF660000; RV1103B
 * 0x20F00000), as the vendor rknpu driver (develop-5.10 drivers/rknpu:
 * rknpu_job.c, include/rknpu_ioctl.h) submits work to it:
 *
 *   0x000 VERSION      0x004 VERSION_NUM   read for the driver's banner
 *   0x008 PC_OP_EN     1 then 0: start the program counter on the job
 *   0x010 PC_DATA_ADDR the first task's register-command stream
 *   0x014 PC_DATA_AMOUNT
 *   0x020 INT_MASK     the last task's interrupt bits
 *   0x024 INT_CLEAR    write 1 to clear INT_STATUS / INT_RAW_STATUS bits
 *   0x028 INT_STATUS   0x02c INT_RAW_STATUS
 *   0x030 PC_TASK_CONTROL  tasks in bits 15:0 (RV1106: 16 bits)
 *   0x034 PC_DMA_BASE_ADDR the task array
 *   0x03c PC_TASK_STATUS   tasks done, 15:0
 *
 * A job "runs" for "task-ns" per task, then its tasks count as done and
 * INT_STATUS takes INT_MASK -- what the driver's rknpu_fuzz_status()
 * compares with the job's mask -- and the interrupt rises.  No convolution
 * is computed.  Everything else is storage.
 *
 * Each task is a stream of 64-bit register commands, target << 48 | value
 * << 16 | register (target 0x0201 CNA, 0x0801 CORE, 0x1001 DPU, 0x2001
 * DPU RDMA, 0x0101 PC), ending with the PC's op enable (target 0x0081,
 * register 0x0008).  Addresses in it are offsets from PC_DMA_BASE_ADDR:
 * the PC's 0x0010 is the next task's stream, the DPU's 0x4020 its output.
 * A task writes only the registers that change; the others keep the
 * last task's values.  At the job's end every DPU output cube (0x4030 width - 1, 0x4034
 * height - 1, 0x403c channels - 1; 16-channel surfaces, 0x4024 the
 * surface stride in 16-byte units, 0 when they are packed) is filled with
 * 0x80, INT8's minimum:
 * a detector's scores come out as low as they go, so it finds nothing.
 * Left as they were, stale outputs decoded as 128 objects a frame on the
 * XM R5, whose post-processing then took seconds per frame and starved
 * the AI's VPSS channel.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/memory.h"
#include "system/address-spaces.h"

#define TYPE_RK_RKNPU "rk-rknpu"
OBJECT_DECLARE_SIMPLE_TYPE(RkRknpuState, RK_RKNPU)

#define REGS_SIZE       0x10000
#define NREGS           (0x100 / 4)
#define R_VERSION       0x000
#define R_VERSION_NUM   0x004
#define R_PC_OP_EN      0x008
#define R_INT_MASK      0x020
#define R_INT_CLEAR     0x024
#define R_INT_STATUS    0x028
#define R_INT_RAW       0x02c
#define R_PC_DATA_ADDR  0x010
#define R_TASK_CTRL     0x030
#define R_PC_DMA_BASE   0x034
#define R_TASK_STATUS   0x03c

struct RkRknpuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;

    uint32_t regs[NREGS];
    uint32_t task_ns;           /* property: a task's time */
    bool trace;                 /* property: log each task's commands */
    uint64_t jobs;
    /* the DPU's output registers: tasks write only what changes */
    uint32_t dpu_dst, dpu_surf, dpu_w, dpu_h, dpu_ch;
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_rknpu_update(RkRknpuState *s)
{
    qemu_set_irq(s->irq, !!REG(s, R_INT_STATUS));
}

/* fill [addr, addr + len) with INT8's minimum, if it is all RAM */
static void rk_rknpu_fill(hwaddr addr, uint64_t len)
{
    MemoryRegionSection sec;
    bool ram;
    g_autofree uint8_t *buf = NULL;

    if (!len || len > (64 << 20)) {
        return;
    }
    sec = memory_region_find(get_system_memory(), addr, len);
    ram = sec.mr && memory_region_is_ram(sec.mr) && int128_get64(sec.size) == len;
    if (sec.mr) {
        memory_region_unref(sec.mr);
    }
    if (!ram) {
        return;
    }
    buf = g_malloc(len);
    memset(buf, 0x80, len);
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

#define MAX_TASK_CMDS   4096

/* the job's DPU outputs read "nothing" */
static void rk_rknpu_blank(RkRknpuState *s)
{
    uint32_t base = REG(s, R_PC_DMA_BASE);
    hwaddr pc = REG(s, R_PC_DATA_ADDR);
    int tasks = MAX(REG(s, R_TASK_CTRL) & 0xffff, 1);
    int t, i;

    s->jobs++;
    if (s->trace) {
        qemu_log("rk-rknpu: job %" PRIu64 ", %d tasks, base 0x%x\n",
                 s->jobs, tasks, base);
    }
    for (t = 0; t < tasks && pc; t++) {
        uint32_t next = 0;
        bool dpu = false;

        if (s->trace) {
            qemu_log("rk-rknpu: task %d at 0x%" HWADDR_PRIx ":", t, pc);
            for (i = 0; i < MAX_TASK_CMDS; i++) {
                uint64_t q = address_space_ldq_le(&address_space_memory,
                                                  pc + i * 8,
                                                  MEMTXATTRS_UNSPECIFIED,
                                                  NULL);

                qemu_log(" %04x:%04x=%08x", (uint32_t)(q >> 48),
                         (uint32_t)(q & 0xffff), (uint32_t)(q >> 16));
                if ((q >> 48) == 0x0081 && (q & 0xffff) == 0x0008) {
                    break;
                }
            }
            qemu_log("\n");
        }

        for (i = 0; i < MAX_TASK_CMDS; i++) {
            uint64_t q = address_space_ldq_le(&address_space_memory,
                                              pc + i * 8,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
            uint32_t tgt = q >> 48, reg = q & 0xffff, val = q >> 16;

            if (tgt == 0x1001) {
                dpu = true;
                switch (reg) {
                case 0x4020: s->dpu_dst = val; break;
                case 0x4024: s->dpu_surf = val >> 4; break;
                case 0x4030: s->dpu_w = (val & 0x1fff) + 1; break;
                case 0x4034: s->dpu_h = (val & 0x1fff) + 1; break;
                case 0x403c: s->dpu_ch = (val & 0x1fff) + 1; break;
                }
            } else if (tgt == 0x0101 && reg == 0x0010) {
                next = val;
            } else if (tgt == 0x0081 && reg == 0x0008) {
                break;                  /* the task's op enable: its end */
            }
        }
        if (dpu && s->dpu_dst && s->dpu_w && s->dpu_h && s->dpu_ch) {
            uint64_t plane = (uint64_t)s->dpu_w * s->dpu_h;   /* 16-B units */
            uint64_t stride = s->dpu_surf ? s->dpu_surf : plane;
            int surfaces = DIV_ROUND_UP(s->dpu_ch, 16), k;
            hwaddr a = (uint32_t)(base + s->dpu_dst);

            if (stride == plane) {          /* packed: one fill */
                rk_rknpu_fill(a, plane * surfaces * 16);
            } else {
                for (k = 0; k < surfaces; k++) {
                    rk_rknpu_fill(a + k * stride * 16, plane * 16);
                }
            }
        }
        pc = next ? (hwaddr)(uint32_t)(base + next) : 0;
    }
}

static void rk_rknpu_done(void *opaque)
{
    RkRknpuState *s = opaque;

    rk_rknpu_blank(s);
    REG(s, R_TASK_STATUS) = REG(s, R_TASK_CTRL) & 0xffff;
    REG(s, R_INT_RAW) |= REG(s, R_INT_MASK);
    REG(s, R_INT_STATUS) |= REG(s, R_INT_MASK);
    rk_rknpu_update(s);
}

static uint64_t rk_rknpu_read(void *opaque, hwaddr off, unsigned size)
{
    RkRknpuState *s = opaque;

    switch (off) {
    case R_VERSION:
        return 0x00010000;          /* the driver prints VERSION + NUM */
    case R_VERSION_NUM:
        return 0;
    }
    return off < NREGS * 4 ? s->regs[off / 4] : 0;
}

static void rk_rknpu_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkRknpuState *s = opaque;
    uint32_t tasks;

    switch (off) {
    case R_INT_CLEAR:
        REG(s, R_INT_STATUS) &= ~val;
        REG(s, R_INT_RAW) &= ~val;
        rk_rknpu_update(s);
        return;
    case R_INT_STATUS:
    case R_INT_RAW:
    case R_TASK_STATUS:
        return;                     /* read-only */
    case R_PC_OP_EN:
        if (val & 1) {
            tasks = MAX(REG(s, R_TASK_CTRL) & 0xffff, 1);
            REG(s, R_TASK_STATUS) = 0;
            timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (int64_t)tasks * s->task_ns);
        }
        break;
    }
    if (off < NREGS * 4) {
        s->regs[off / 4] = val;
    }
}

static const MemoryRegionOps rk_rknpu_ops = {
    .read = rk_rknpu_read,
    .write = rk_rknpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_rknpu_reset(DeviceState *dev)
{
    RkRknpuState *s = RK_RKNPU(dev);

    timer_del(s->done);
    memset(s->regs, 0, sizeof(s->regs));
    s->dpu_dst = s->dpu_surf = s->dpu_w = s->dpu_h = s->dpu_ch = 0;
    rk_rknpu_update(s);
}

static void rk_rknpu_init(Object *obj)
{
    RkRknpuState *s = RK_RKNPU(obj);

    memory_region_init_io(&s->iomem, obj, &rk_rknpu_ops, s, TYPE_RK_RKNPU,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_rknpu_done, s);
}

static const Property rk_rknpu_properties[] = {
    DEFINE_PROP_UINT32("task-ns", RkRknpuState, task_ns, 200000),
    DEFINE_PROP_BOOL("trace", RkRknpuState, trace, false),
};

static const VMStateDescription rk_rknpu_vmstate = {
    .name = TYPE_RK_RKNPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkRknpuState, NREGS),
        VMSTATE_TIMER_PTR(done, RkRknpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_rknpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_rknpu_reset);
    device_class_set_props(dc, rk_rknpu_properties);
    dc->vmsd = &rk_rknpu_vmstate;
}

static const TypeInfo rk_rknpu_info = {
    .name = TYPE_RK_RKNPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkRknpuState),
    .instance_init = rk_rknpu_init,
    .class_init = rk_rknpu_class_init,
};

static void rk_rknpu_register_types(void)
{
    type_register_static(&rk_rknpu_info);
}

type_init(rk_rknpu_register_types)
