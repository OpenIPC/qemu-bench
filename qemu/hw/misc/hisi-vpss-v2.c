/*
 * HiSilicon VPSS of the Hi3516CV200 / Hi3518EV200 family (0x20600000,
 * VIC 17), behind VI offline: a node engine.  Per frame hi3518e_vpss.o
 * builds register images in DDR, chains them and starts the chain; the
 * engine reads the frame VIU wrote and writes it, scaled, into each
 * enabled channel's buffers.
 *
 * MMIO (VPSS_HAL_Start / GetIntStatus / ClrInt / GetNodeID):
 *   0x01f4  ID of the last node done (the driver checks it against the
 *           node it expects, VPSS_COMM_IrqProc)
 *   0x02fc  the first node; 0x0300 START (1, or 3)
 *   0x0304  status, 0x0308 write 1s to clear, 0x030c raw: bit 0 a node
 *           done (VPSS_DRV_IsWrongInt: exactly 2 is a bus error it panics
 *           on, 4 a timeout, bit 7 an error)
 * A node (VPSS_HAL_Set*), offsets in its image:
 *   0x020  source size, width - 1 | (height - 1) << 16 (13 bits each)
 *   0x10c / 0x110 / 0x114  source Y, C, stride (Y | C << 16)
 *   channel 0 / 1 / 2 / 3 (the HAL's jump tables, not in order):
 *     on     0x008 bit 0 / 3 / 2 / 1
 *     size   0x070 / 0x064 / 0x068 / 0x06c, as the source's
 *     Y, C, stride:
 *     0x198 0x19c 0x1a0 / 0x150 0x154 0x158 / 0x168 0x16c 0x170 /
 *     0x180 0x184 0x188 (two compression header words before each)
 *   0x1f4  the node's ID, 0x1fc the interrupts it raises, 0x2fc the next
 * Pictures are semi-planar 4:2:0 and scaled nearest-neighbour.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_HISI_VPSS_V2 "hisi-vpss-v2"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVpssV2State, HISI_VPSS_V2)

#define V2_SIZE             0x10000
#define V2_NODE_ID          0x01f4
#define V2_NODE_MASK        0x01fc
#define V2_NEXT             0x02fc
#define V2_START            0x0300
#define V2_INT_STATUS       0x0304
#define V2_INT_CLR          0x0308
#define V2_INT_RAW          0x030c
#define V2_NODE_SIZE        0x400
#define V2_NODE_MAX         16
#define V2_PIC_MAX          4096
#define V2_INT_ERR          (1u << 7)
#define V2_JOB_NS           2000000     /* a 1080p node, about 2 ms */

/* the HAL's jump tables do not take the channels in order */
static const struct {
    hwaddr size, y, c, stride;
    int en;
} v2_chn[4] = {
    { 0x070, 0x198, 0x19c, 0x1a0, 0 },
    { 0x064, 0x150, 0x154, 0x158, 3 },
    { 0x068, 0x168, 0x16c, 0x170, 2 },
    { 0x06c, 0x180, 0x184, 0x188, 1 },
};

struct HisiVpssV2State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[V2_SIZE / 4];
    uint32_t pending;               /* raw bits the job raises */
    uint32_t queued;                /* a chain started while one ran */
    bool has_queued;
    uint32_t done_id;
    uint32_t nodes;
    bool trace;
};

static void v2_update_irq(HisiVpssV2State *s)
{
    s->regs[V2_INT_STATUS / 4] = s->regs[V2_INT_RAW / 4];
    qemu_set_irq(s->irq, s->regs[V2_INT_STATUS / 4] != 0);
}

static void v2_size(uint32_t v, int *w, int *h)
{
    *w = (v & 0x1fff) + 1;
    *h = ((v >> 16) & 0x1fff) + 1;
}

/* One plane, nearest neighbour; px is 2 for interleaved chroma pairs */
static void v2_scale(uint32_t src, int ss, int sw, int sh, uint32_t dst,
                     int ds, int dw, int dh, int px)
{
    g_autofree uint8_t *in = g_malloc((size_t)sw * px);
    g_autofree uint8_t *out = g_malloc((size_t)dw * px);
    int last = -1;

    for (int y = 0; y < dh; y++) {
        int sy = (int)((int64_t)y * sh / dh);

        if (sy != last) {
            dma_memory_read(&address_space_memory, src + (uint64_t)sy * ss,
                            in, (size_t)sw * px, MEMTXATTRS_UNSPECIFIED);
            for (int x = 0; x < dw; x++) {
                int sx = (int)((int64_t)x * sw / dw);

                memcpy(out + x * px, in + sx * px, px);
            }
            last = sy;
        }
        dma_memory_write(&address_space_memory, dst + (uint64_t)y * ds, out,
                         (size_t)dw * px, MEMTXATTRS_UNSPECIFIED);
    }
}

static void v2_run_node(HisiVpssV2State *s, const uint32_t *n)
{
    int sw, sh, sys, scs;
    uint32_t sy = n[0x10c / 4], sc = n[0x110 / 4];

    v2_size(n[0x020 / 4], &sw, &sh);
    sys = n[0x114 / 4] & 0xffff;
    scs = n[0x114 / 4] >> 16;
    if (!sy || sw < 2 || sh < 2 || sw > V2_PIC_MAX || sh > V2_PIC_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-vpss-v2: node source %dx%d at "
                      "0x%08x not processed\n", sw, sh, sy);
        return;
    }
    for (int c = 0; c < 4; c++) {
        int dw, dh;
        uint32_t dy = n[v2_chn[c].y / 4], dc = n[v2_chn[c].c / 4];
        uint32_t st = n[v2_chn[c].stride / 4];

        if (!(n[0x008 / 4] & (1u << v2_chn[c].en)) || !dy) {
            continue;
        }
        v2_size(n[v2_chn[c].size / 4], &dw, &dh);
        if (dw < 2 || dh < 2 || dw > V2_PIC_MAX || dh > V2_PIC_MAX) {
            continue;
        }
        if (s->trace) {
            fprintf(stderr, "hisi-vpss-v2: chn %d %dx%d -> %dx%d y %08x "
                    "c %08x stride %u/%u\n", c, sw, sh, dw, dh, dy, dc,
                    st & 0xffff, st >> 16);
        }
        v2_scale(sy, sys ? sys : sw, sw, sh, dy, (st & 0xffff) ?: dw,
                 dw, dh, 1);
        if (sc && dc) {
            v2_scale(sc, scs ? scs : sw, sw / 2, sh / 2, dc,
                     (st >> 16) ?: dw, dw / 2, dh / 2, 2);
        }
    }
}

/*
 * Runs the chain at 0x2fc: its last node's ID and the interrupts it
 * raises are the job's.  A node that cannot be read ends the chain with
 * the error bit (7), which the driver reports without panicking.
 */
static void v2_run(HisiVpssV2State *s, uint32_t a)
{
    uint32_t node[V2_NODE_SIZE / 4];
    uint32_t bits = 0;
    int k;

    for (k = 0; k < V2_NODE_MAX && a; k++) {
        if (dma_memory_read(&address_space_memory, a, node, sizeof(node),
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-vpss-v2: node at 0x%08x unreadable\n", a);
            bits = V2_INT_ERR;
            break;
        }
        v2_run_node(s, node);
        bits |= node[V2_NODE_MASK / 4] & 1;
        s->done_id = node[V2_NODE_ID / 4];
        s->nodes++;
        a = node[V2_NEXT / 4];
    }
    s->pending = bits ? bits : 1;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + V2_JOB_NS);
}

/* A START while a chain runs waits for it: each has its own completion */
static void v2_start(HisiVpssV2State *s)
{
    if (timer_pending(s->timer)) {
        s->queued = s->regs[V2_NEXT / 4];
        s->has_queued = true;
        return;
    }
    v2_run(s, s->regs[V2_NEXT / 4]);
}

static void v2_done(void *opaque)
{
    HisiVpssV2State *s = opaque;

    s->regs[V2_NODE_ID / 4] = s->done_id;
    s->regs[V2_INT_RAW / 4] |= s->pending;
    s->pending = 0;
    v2_update_irq(s);
    if (s->has_queued) {
        s->has_queued = false;
        v2_run(s, s->queued);
    }
}

static uint64_t hisi_vpss_v2_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVpssV2State *s = HISI_VPSS_V2(opaque);

    return offset < V2_SIZE ? s->regs[offset / 4] : 0;
}

static void hisi_vpss_v2_write(void *opaque, hwaddr offset, uint64_t val64,
                               unsigned size)
{
    HisiVpssV2State *s = HISI_VPSS_V2(opaque);
    uint32_t val = val64;

    if (offset >= V2_SIZE) {
        return;
    }
    switch (offset) {
    case V2_START:
        if (val & 1) {
            v2_start(s);
        }
        return;
    case V2_INT_CLR:
        s->regs[V2_INT_RAW / 4] &= ~val;
        v2_update_irq(s);
        return;
    case V2_INT_STATUS:
    case V2_INT_RAW:
        return;
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_vpss_v2_ops = {
    .read = hisi_vpss_v2_read,
    .write = hisi_vpss_v2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vpss_v2_reset(DeviceState *dev)
{
    HisiVpssV2State *s = HISI_VPSS_V2(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pending = 0;
    s->done_id = 0;
    s->has_queued = false;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void hisi_vpss_v2_realize(DeviceState *dev, Error **errp)
{
    HisiVpssV2State *s = HISI_VPSS_V2(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, v2_done, s);
}

static void hisi_vpss_v2_init(Object *obj)
{
    HisiVpssV2State *s = HISI_VPSS_V2(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_vpss_v2_ops, s,
                          TYPE_HISI_VPSS_V2, V2_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    object_property_add_uint32_ptr(obj, "nodes", &s->nodes,
                                   OBJ_PROP_FLAG_READ);
}

static const VMStateDescription vmstate_hisi_vpss_v2 = {
    .name = TYPE_HISI_VPSS_V2,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HisiVpssV2State, V2_SIZE / 4),
        VMSTATE_UINT32(pending, HisiVpssV2State),
        VMSTATE_UINT32(done_id, HisiVpssV2State),
        VMSTATE_UINT32(queued, HisiVpssV2State),
        VMSTATE_BOOL(has_queued, HisiVpssV2State),
        VMSTATE_TIMER_PTR(timer, HisiVpssV2State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property hisi_vpss_v2_properties[] = {
    DEFINE_PROP_BOOL("trace", HisiVpssV2State, trace, false),
};

static void hisi_vpss_v2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_vpss_v2_reset);
    device_class_set_props(dc, hisi_vpss_v2_properties);
    dc->realize = hisi_vpss_v2_realize;
    dc->vmsd = &vmstate_hisi_vpss_v2;
}

static const TypeInfo hisi_vpss_v2_info = {
    .name          = TYPE_HISI_VPSS_V2,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiVpssV2State),
    .instance_init = hisi_vpss_v2_init,
    .class_init    = hisi_vpss_v2_class_init,
};

static void hisi_vpss_v2_register_types(void)
{
    type_register_static(&hisi_vpss_v2_info);
}

type_init(hisi_vpss_v2_register_types)
