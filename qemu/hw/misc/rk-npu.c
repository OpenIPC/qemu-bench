/*
 * The RV1109 / RV1126 NPU ("rockchip,npu", 0xFFBC0000): a Vivante VIP core
 * that the out-of-tree galcore driver (galcore.ko, 6.4.3) runs.  Its probe
 * reads the chip's identity and looks it up in the feature database built
 * into the module ("Feature database is not found, chipModel=0x0, ..."
 * otherwise, and no /dev/galcore: the SDK's mediaserver then dies in
 * _OpenDevice).  galcore.ko's gChipInfo holds two entries; the RV1126's
 * is the second:
 *
 *   0x020 chipModel 0x8000    0x024 chipRevision 0x8003
 *   0x030 customerID 0xb5     0x0a8 productID 0x05080009
 *   0x0e8 ecoID 0x04000000
 *
 *   0x000 AQHiClockControl    bits 16 / 17 (IDLE_3D / IDLE_2D) read set:
 *                             galcore's soft reset retries until they are
 *   0x004 AQHiIdle            every unit idle
 *   0x010 AQIntrAcknowledge   pending event bits, cleared by the read
 *   0x014 AQIntrEnbl
 *
 * The command front end: writing AQCmdBufferCtrl (0x658) with bit 16 set
 * starts fetching at AQCmdBufferAddr (0x654); AQFEDebugCurCmdAdr (0x664)
 * reads where it is.  galcore keeps it in a WAIT / LINK loop and patches
 * the WAIT into a LINK to submit work, so a timer walks the stream: LOAD
 * STATE (one to AQ_EVENT, 0x3804 -- state 0x0e01 -- raises that event's
 * interrupt bit), END (bit 8: an event too), NOP, WAIT (stop until the next
 * tick, then read the place again), LINK, CALL / RETURN, STALL,
 * CHIP_SELECT.  LOAD_STATE also stores the states below 0x1000 (state
 * n at register 4n), as the hardware's register file.  Once galcore turns
 * the MMU on (MMUv2: 0x388 bit 0; the master TLB at 0x184 bits 31:10,
 * loaded through state 0x0061) the stream's addresses are GPU virtual:
 * master entry va[31:22] -> slave table (bit 0 present), slave entry
 * va[21:12] -> 4 KiB page (bit 0 present).  A fetch that does not land in
 * RAM stops the front end.  What the commands compute (the NN / TP
 * engines, drawing) is not done: the jobs complete with their outputs
 * unwritten.
 *
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/rcu.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_NPU "rk-npu"
OBJECT_DECLARE_SIMPLE_TYPE(RkNpuState, RK_NPU)

#define REGS_SIZE       0x4000
#define R_CLOCK_CTRL    0x000
#define R_IDLE          0x004
#define R_INTR_ACK      0x010
#define R_INTR_ENBL     0x014
#define R_CHIP_ID       0x020
#define R_CHIP_REV      0x024
#define R_CUSTOMER_ID   0x030
#define R_PRODUCT_ID    0x0a8
#define R_ECO_ID        0x0e8
#define R_MMU_CONFIG    0x184       /* MMUv2: master TLB, bit 0 1K mode */
#define R_MMU_CONTROL   0x388       /* MMUv2: bit 0 enable */
#define R_CMD_ADDR      0x654
#define R_CMD_CTRL      0x658
#define R_FE_STATE      0x660
#define R_FE_CUR        0x664

#define CMD_ENABLE      (1u << 16)
#define STATE_EVENT     0x0e01      /* AQ_EVENT, 0x3804 >> 2 */
#define FE_TICK_NS      20000
#define FE_MAX_CMDS     4096        /* a tick's walk, past which it rests */

struct RkNpuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t pending;           /* event bits not yet acknowledged */
    QEMUTimer *fe;
    bool fe_on;
    uint32_t pc;                /* the command the front end is at */
    uint32_t ret;               /* CALL's return address */
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_npu_update(RkNpuState *s)
{
    qemu_set_irq(s->irq, !!(s->pending & REG(s, R_INTR_ENBL)));
}

static bool rk_npu_is_ram(hwaddr a)
{
    MemoryRegion *mr;
    hwaddr xlat, len = 4;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(&address_space_memory, a, &xlat, &len,
                                 false, MEMTXATTRS_UNSPECIFIED);
    return memory_region_is_ram(mr);
}

static bool rk_npu_rd(hwaddr a, uint32_t *v)
{
    if (!rk_npu_is_ram(a)) {
        return false;
    }
    dma_memory_read(&address_space_memory, a, v, 4, MEMTXATTRS_UNSPECIFIED);
    *v = le32_to_cpu(*v);
    return true;
}

/* A command word at GPU address @a, through the MMU when it is on */
static bool rk_npu_ld(RkNpuState *s, uint32_t a, uint32_t *v)
{
    hwaddr pa = a;

    if (REG(s, R_MMU_CONTROL) & 1) {
        uint32_t m, t;

        if (!rk_npu_rd((REG(s, R_MMU_CONFIG) & ~0x3ffu) + (a >> 22) * 4, &m) ||
            !(m & 1) ||
            !rk_npu_rd((m & ~0x3fu) + ((a >> 12) & 0x3ff) * 4, &t) ||
            !(t & 1)) {
            return false;
        }
        pa = (t & ~0xfffu) | (a & 0xfff);
    }
    return rk_npu_rd(pa, v);
}

static void rk_npu_event(RkNpuState *s, uint32_t id)
{
    s->pending |= 1u << (id & 31);
    rk_npu_update(s);
}

/* Walk the command stream from s->pc until a WAIT, an END or the budget. */
static void rk_npu_fe_tick(void *opaque)
{
    RkNpuState *s = opaque;
    int n;

    if (!s->fe_on) {
        return;
    }
    for (n = 0; n < FE_MAX_CMDS; n++) {
        uint32_t w, op, arg = 0;

        if (!rk_npu_ld(s, s->pc, &w) ||
            (((w >> 27) == 8 || (w >> 27) == 10) &&
             !rk_npu_ld(s, s->pc + 4, &arg))) {
            qemu_log_mask(LOG_GUEST_ERROR, "rk-npu: no memory at 0x%x\n",
                          s->pc);
            s->fe_on = false;
            return;
        }
        op = w >> 27;

        REG(s, R_FE_CUR) = s->pc;
        switch (op) {
        case 1: {                                   /* LOAD_STATE */
            uint32_t cnt = (w >> 16) & 0x3ff, st = w & 0xffff, i;

            cnt = cnt ? cnt : 1024;
            for (i = 0; i < cnt; i++) {
                uint32_t v;

                if (!rk_npu_ld(s, s->pc + 4 + 4 * i, &v)) {
                    break;
                }
                if (st + i == STATE_EVENT) {
                    rk_npu_event(s, v);
                } else if (st + i < REGS_SIZE / 4) {
                    s->regs[st + i] = v;        /* the register file */
                }
            }
            s->pc += 4 * ((1 + cnt + 1) & ~1u);     /* 64-bit aligned */
            break;
        }
        case 2:                                     /* END */
            if (w & (1u << 8)) {
                rk_npu_event(s, w & 0x1f);
            }
            s->fe_on = false;
            return;
        case 3:                                     /* NOP */
        case 9:                                     /* STALL */
        case 13:                                    /* CHIP_SELECT */
            s->pc += 8;
            break;
        case 5:                                     /* DRAW_PRIMITIVES */
        case 6:                                     /* DRAW_INDEXED */
            s->pc += 16;
            break;
        case 7:                                     /* WAIT: rest, then */
            goto rest;                              /* read it again */
        case 8:                                     /* LINK */
            s->pc = arg;
            break;
        case 10:                                    /* CALL */
            s->ret = s->pc + 8;
            s->pc = arg;
            break;
        case 11:                                    /* RETURN */
            s->pc = s->ret;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "rk-npu: command 0x%08x at 0x%x\n",
                          w, s->pc);
            s->pc += 8;
            break;
        }
    }
rest:
    timer_mod(s->fe, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FE_TICK_NS);
}

static uint64_t rk_npu_read(void *opaque, hwaddr off, unsigned size)
{
    RkNpuState *s = opaque;
    uint32_t v;

    switch (off) {
    case R_CLOCK_CTRL:              /* + IDLE_3D / IDLE_2D: reset done */
        return REG(s, R_CLOCK_CTRL) | (3u << 16);
    case R_IDLE:
        return 0x7fffffff;
    case R_INTR_ACK:
        v = s->pending;
        s->pending = 0;
        rk_npu_update(s);
        return v;
    case R_CHIP_ID:
        return 0x8000;
    case R_CHIP_REV:
        return 0x8003;
    case R_CUSTOMER_ID:
        return 0xb5;
    case R_PRODUCT_ID:
        return 0x05080009;
    case R_ECO_ID:
        return 0x04000000;
    }
    return s->regs[off / 4];
}

static void rk_npu_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkNpuState *s = opaque;

    s->regs[off / 4] = val;
    if (off == R_INTR_ENBL) {
        rk_npu_update(s);
    }
    if (off == R_CMD_CTRL && (val & CMD_ENABLE)) {
        s->pc = REG(s, R_CMD_ADDR);
        s->fe_on = true;
        timer_mod(s->fe, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FE_TICK_NS);
    }
}

static const MemoryRegionOps rk_npu_ops = {
    .read = rk_npu_read,
    .write = rk_npu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_npu_reset(DeviceState *dev)
{
    RkNpuState *s = RK_NPU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pending = 0;
    s->fe_on = false;
    timer_del(s->fe);
    rk_npu_update(s);
}

static void rk_npu_init(Object *obj)
{
    RkNpuState *s = RK_NPU(obj);

    memory_region_init_io(&s->iomem, obj, &rk_npu_ops, s, TYPE_RK_NPU,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->fe = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_npu_fe_tick, s);
}

static const VMStateDescription rk_npu_vmstate = {
    .name = TYPE_RK_NPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkNpuState, REGS_SIZE / 4),
        VMSTATE_UINT32(pending, RkNpuState),
        VMSTATE_BOOL(fe_on, RkNpuState),
        VMSTATE_UINT32(pc, RkNpuState),
        VMSTATE_UINT32(ret, RkNpuState),
        VMSTATE_TIMER_PTR(fe, RkNpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_npu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_npu_reset);
    dc->vmsd = &rk_npu_vmstate;
}

static const TypeInfo rk_npu_info = {
    .name = TYPE_RK_NPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkNpuState),
    .instance_init = rk_npu_init,
    .class_init = rk_npu_class_init,
};

static void rk_npu_register_types(void)
{
    type_register_static(&rk_npu_info);
}

type_init(rk_npu_register_types)
