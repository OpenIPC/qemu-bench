/*
 * SigmaStar Ifado CMDQ: the command queues that write registers for the
 * ISP, scaler and encoder drivers.
 *
 * Four engines, one RIU bank each (sstar_mi.ko g_halRegCmdCtlBase: banks
 * 0x1120, 0x1123, 0x1124, 0x1125), 16-bit registers on a 4-byte stride
 * (Hal_cmdq_*; byte offsets):
 *
 *   +0x04 bit 0     enable                 +0x0C bit 1  start, bit 8
 *   +0x10/+0x14     buffer start               latch the read pointer
 *   +0x18/+0x1C     buffer end              +0x28/+0x2C  read pointer
 *   +0x20/+0x24     write pointer           +0x30/+0x34  buffer read ptr
 *   +0x110/+0x114   interrupt status; bits 27 and 28 of the 32-bit
 *                   word are idle (Hal_cmdq_ReadIdleDone)
 *   +0x118/+0x11C   interrupt mask          +0x120/+0x124 clear
 *   +0xF0/+0xF4     the failing command's type, [23:16] of +0xF4
 *
 * The pointers are MIU addresses in 16-byte units; the driver writes the
 * write pointer's high half first, so the low half's write kicks.
 *
 * The queue holds 8-byte commands (MHal_cmdq_InsertOneCommand): the low
 * word is data << 16 | the bits to keep (0xFFFF for a plain write, which
 * keeps none), the high word type << 24 | the register's RIU word index.
 * Types: 0x10 masked write, 0x11-0x13 the same raising software interrupt
 * 1-3 (MHal_cmdq_WriteRegCmdqMaskWithInt), 0x20 wait for a trigger event,
 * 0x30 poll equal, 0xB0 poll not equal, 0x90/0x80 a 32-bit write's mask
 * and data, 0 nothing.  A moved write pointer runs the commands up to it
 * at once; waits and polls do not block.  "trace" logs each command.
 *
 * Copyright (c) 2026 OpenIPC.
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
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_SSTAR_CMDQ "sstar-cmdq"
OBJECT_DECLARE_SIMPLE_TYPE(SstarCmdqState, SSTAR_CMDQ)

#define CMDQ_ENGINES        4
#define CMDQ_REGS           0x80

/* register indices (byte offset / 4) */
#define R_CTRL_HI           0x01        /* bit 0 enable */
#define R_TRIG_HI           0x03        /* bit 1 start, bit 8 latch */
#define R_START_LO          0x04
#define R_START_HI          0x05
#define R_END_LO            0x06
#define R_END_HI            0x07
#define R_WPTR_LO           0x08
#define R_WPTR_HI           0x09
#define R_RPTR_LO           0x0A
#define R_RPTR_HI           0x0B
#define R_BRPTR_LO          0x0C
#define R_BRPTR_HI          0x0D
#define R_STATUS_LO         0x44
#define R_STATUS_HI         0x45
#define R_MASK_LO           0x46
#define R_CLEAR_LO          0x48

#define STATUS_HI_IDLE      (3u << 11)  /* bits 27, 28 of the word */
#define SW_INT(n)           (1u << (n))     /* a guess; see the ISR */

#define SS_RIU_BASE         0x1F000000
#define SS_DRAM_BASE        0x20000000
#define CMDQ_MAX_RUN        0x40000     /* commands per kick */

typedef struct SstarCmdqEngine {
    uint16_t regs[CMDQ_REGS];
    uint32_t rptr;
    uint32_t x32_mask;
} SstarCmdqEngine;

typedef struct SstarCmdqBank {
    SstarCmdqState *s;
    int n;
} SstarCmdqBank;

struct SstarCmdqState {
    SysBusDevice parent_obj;

    MemoryRegion iomem[CMDQ_ENGINES];
    SstarCmdqBank bank[CMDQ_ENGINES];
    qemu_irq irq;
    SstarCmdqEngine eng[CMDQ_ENGINES];
    bool trace;
};

static uint32_t eng32(SstarCmdqEngine *e, int lo)
{
    return e->regs[lo] | ((uint32_t)e->regs[lo + 1] << 16);
}

static hwaddr cmdq_dram(uint32_t miu)
{
    return miu < SS_DRAM_BASE ? SS_DRAM_BASE + miu : miu;
}

static void sstar_cmdq_update_irq(SstarCmdqState *s)
{
    bool level = false;
    int n;

    for (n = 0; n < CMDQ_ENGINES; n++) {
        SstarCmdqEngine *e = &s->eng[n];

        level |= !!(e->regs[R_STATUS_LO] & ~e->regs[R_MASK_LO]);
    }
    qemu_set_irq(s->irq, level);
}

static uint64_t sstar_cmdq_read(void *opaque, hwaddr off, unsigned size);
static void sstar_cmdq_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size);

/* A command naming one of the engines' own banks (their dummy registers) */
static SstarCmdqBank *cmdq_own_bank(SstarCmdqState *s, hwaddr a, hwaddr *off)
{
    int n;

    for (n = 0; n < CMDQ_ENGINES; n++) {
        MemoryRegion *mr = &s->iomem[n];

        if (mr->container && a >= mr->addr && a < mr->addr + CMDQ_REGS * 4) {
            *off = a - mr->addr;
            return &s->bank[n];
        }
    }
    return NULL;
}

static uint16_t riu_read16(SstarCmdqState *s, hwaddr a)
{
    uint16_t v = 0;
    SstarCmdqBank *b;
    hwaddr off;

    b = cmdq_own_bank(s, a, &off);
    if (b) {
        return sstar_cmdq_read(b, off, 2);
    }
    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       &v, 2);
    return le16_to_cpu(v);
}

static void riu_write16(SstarCmdqState *s, hwaddr a, uint16_t v)
{
    SstarCmdqBank *b;
    hwaddr off;

    b = cmdq_own_bank(s, a, &off);
    if (b) {
        sstar_cmdq_write(b, off, v, 2);
        return;
    }
    v = cpu_to_le16(v);
    address_space_write(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                        &v, 2);
}

/* The register a command names: a 16-bit RIU register's word index. */
static hwaddr cmdq_reg(uint32_t hi)
{
    return SS_RIU_BASE + (hwaddr)(hi & 0xFFFFFF) * 4;
}

static void sstar_cmdq_exec(SstarCmdqState *s, int n, uint32_t lo,
                            uint32_t hi)
{
    SstarCmdqEngine *e = &s->eng[n];
    uint8_t type = hi >> 24;
    uint16_t data = lo >> 16;
    /* the bits written; the driver writes 0xFFFF for all of them */
    uint16_t mask = (lo & 0xFFFF) == 0xFFFF ? 0xFFFF : ~lo & 0xFFFF;
    hwaddr a = cmdq_reg(hi);
    uint16_t cur;

    if (s->trace && type) {
        qemu_log("sstar-cmdq%d: %02x reg 0x%06x (0x%08" PRIx64 ") data "
                 "0x%04x mask 0x%04x\n", n, type, hi & 0xFFFFFF,
                 (uint64_t)a, data, mask);
    }
    switch (type) {
    case 0x00:
        break;
    case 0x10: case 0x11: case 0x12: case 0x13:
        cur = mask == 0xFFFF ? 0 : riu_read16(s, a);
        riu_write16(s, a, (cur & ~mask) | (data & mask));
        if (type & 3) {
            e->regs[R_STATUS_LO] |= SW_INT(type & 3);
            sstar_cmdq_update_irq(s);
        }
        break;
    case 0x20:              /* wait for an event: the events are now */
        break;
    case 0x30:
    case 0xB0:
        cur = riu_read16(s, a);
        if (((cur & mask) == (data & mask)) != (type == 0x30)) {
            qemu_log_mask(LOG_UNIMP, "sstar-cmdq%d: poll %s 0x%04x/0x%04x "
                          "at 0x%08" PRIx64 " reads 0x%04x; going on\n", n,
                          type == 0x30 ? "for" : "past", data, mask,
                          (uint64_t)a, cur);
        }
        break;
    case 0x90:
        e->x32_mask = ~lo;
        break;
    case 0x80: {
        uint32_t v = 0;

        address_space_read(&address_space_memory, a,
                           MEMTXATTRS_UNSPECIFIED, &v, 4);
        v = (le32_to_cpu(v) & ~e->x32_mask) | (lo & e->x32_mask);
        v = cpu_to_le32(v);
        address_space_write(&address_space_memory, a,
                            MEMTXATTRS_UNSPECIFIED, &v, 4);
        e->x32_mask = UINT32_MAX;
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "sstar-cmdq%d: command type 0x%02x\n", n,
                      type);
        break;
    }
}

/* Run the queue from the read pointer up to the write pointer. */
static void sstar_cmdq_run(SstarCmdqState *s, int n)
{
    SstarCmdqEngine *e = &s->eng[n];
    uint32_t start = eng32(e, R_START_LO) << 4, end = eng32(e, R_END_LO) << 4;
    uint32_t wptr = eng32(e, R_WPTR_LO) << 4;
    uint32_t shown;
    int count = 0;

    if (!(e->regs[R_CTRL_HI] & 1) || end <= start ||
        wptr < start || wptr > end) {
        return;
    }
    if (e->rptr < start || e->rptr >= end) {
        e->rptr = start;
    }
    if (wptr == end) {
        wptr = start;       /* a program that fills the ring to its end */
    }
    while (e->rptr != wptr && count++ < CMDQ_MAX_RUN) {
        uint32_t cmd[2];

        dma_memory_read(&address_space_memory, cmdq_dram(e->rptr), cmd,
                        sizeof(cmd), MEMTXATTRS_UNSPECIFIED);
        sstar_cmdq_exec(s, n, le32_to_cpu(cmd[0]), le32_to_cpu(cmd[1]));
        e->rptr += 8;
        if (e->rptr >= end) {
            e->rptr = start;
        }
    }
    /*
     * Caught up, the read pointer is the write pointer as the driver
     * wrote it, the ring's end included: read == write is an empty ring
     * to MHal_cmdq_CheckBufferAvailTimeout, and its start a full one.
     */
    shown = e->rptr == wptr ? eng32(e, R_WPTR_LO) << 4 : e->rptr;
    e->regs[R_RPTR_LO] = (shown >> 4) & 0xFFFF;
    e->regs[R_RPTR_HI] = shown >> 20;
    e->regs[R_BRPTR_LO] = (shown >> 4) & 0xFFFF;
    e->regs[R_BRPTR_HI] = shown >> 20;
    sstar_cmdq_update_irq(s);
}

static uint64_t sstar_cmdq_read(void *opaque, hwaddr off, unsigned size)
{
    SstarCmdqBank *b = opaque;
    SstarCmdqEngine *e = &b->s->eng[b->n];
    uint16_t v;
    int reg = (off >> 2) & (CMDQ_REGS - 1);

    if ((off & 3) >= 2) {
        return 0;
    }
    v = e->regs[reg];
    if (reg == R_STATUS_HI) {
        v |= STATUS_HI_IDLE;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void sstar_cmdq_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    SstarCmdqBank *b = opaque;
    SstarCmdqState *s = b->s;
    SstarCmdqEngine *e = &s->eng[b->n];
    int reg = (off >> 2) & (CMDQ_REGS - 1);
    uint16_t mask, v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        mask = off & 1 ? 0xFF00 : 0x00FF;
        v = off & 1 ? (val & 0xFF) << 8 : val & 0xFF;
    } else {
        mask = 0xFFFF;
        v = val;
    }
    switch (reg) {
    case R_RPTR_LO: case R_RPTR_HI:
    case R_BRPTR_LO: case R_BRPTR_HI:
    case R_STATUS_LO: case R_STATUS_HI:
        return;
    case R_CLEAR_LO:
        e->regs[R_STATUS_LO] &= ~(v & mask);
        sstar_cmdq_update_irq(s);
        return;
    case R_START_LO: case R_START_HI:
        e->regs[reg] = (e->regs[reg] & ~mask) | (v & mask);
        e->rptr = eng32(e, R_START_LO) << 4;
        e->regs[R_RPTR_LO] = (e->rptr >> 4) & 0xFFFF;
        e->regs[R_RPTR_HI] = e->rptr >> 20;
        return;
    }
    e->regs[reg] = (e->regs[reg] & ~mask) | (v & mask);
    switch (reg) {
    case R_WPTR_LO:
    case R_CTRL_HI:
    case R_TRIG_HI:
        sstar_cmdq_run(s, b->n);
        break;
    case R_MASK_LO:
        sstar_cmdq_update_irq(s);
        break;
    }
}

static const MemoryRegionOps sstar_cmdq_ops = {
    .read = sstar_cmdq_read,
    .write = sstar_cmdq_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void sstar_cmdq_reset(DeviceState *dev)
{
    SstarCmdqState *s = SSTAR_CMDQ(dev);

    int n;

    memset(s->eng, 0, sizeof(s->eng));
    for (n = 0; n < CMDQ_ENGINES; n++) {
        s->eng[n].x32_mask = UINT32_MAX;
    }
    sstar_cmdq_update_irq(s);
}

static void sstar_cmdq_init(Object *obj)
{
    SstarCmdqState *s = SSTAR_CMDQ(obj);
    int n;

    for (n = 0; n < CMDQ_ENGINES; n++) {
        s->bank[n].s = s;
        s->bank[n].n = n;
        s->eng[n].x32_mask = UINT32_MAX;
        memory_region_init_io(&s->iomem[n], obj, &sstar_cmdq_ops, &s->bank[n],
                              TYPE_SSTAR_CMDQ, CMDQ_REGS * 4);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem[n]);
    }
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sstar_cmdq_props[] = {
    DEFINE_PROP_BOOL("trace", SstarCmdqState, trace, false),
};

static const VMStateDescription vmstate_sstar_cmdq_engine = {
    .name = TYPE_SSTAR_CMDQ "-engine",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarCmdqEngine, CMDQ_REGS),
        VMSTATE_UINT32(rptr, SstarCmdqEngine),
        VMSTATE_UINT32(x32_mask, SstarCmdqEngine),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_sstar_cmdq = {
    .name = TYPE_SSTAR_CMDQ,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(eng, SstarCmdqState, CMDQ_ENGINES, 1,
                             vmstate_sstar_cmdq_engine, SstarCmdqEngine),
        VMSTATE_END_OF_LIST()
    }
};

static void sstar_cmdq_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_cmdq_reset);
    device_class_set_props(dc, sstar_cmdq_props);
    dc->vmsd = &vmstate_sstar_cmdq;
}

static const TypeInfo sstar_cmdq_info = {
    .name           = TYPE_SSTAR_CMDQ,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(SstarCmdqState),
    .instance_init  = sstar_cmdq_init,
    .class_init     = sstar_cmdq_class_init,
};

static void sstar_cmdq_register_types(void)
{
    type_register_static(&sstar_cmdq_info);
}

type_init(sstar_cmdq_register_types)
