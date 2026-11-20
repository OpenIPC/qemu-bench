/*
 * HiSilicon V4 / V5 VGS (graphics / scaling engine): Hi3516CV608 / CV610 /
 * CV613, and XMedia xm720 (GK7205V510) whose xm_vgs.ko VGS_HAL_* has the
 * same front end
 *
 * VGS is what produces a VPSS *extended* channel's picture: VPSS paints
 * only its physical channels, and mod_vpp.o's vpp_submit_ext_chn_frame_to_vgs
 * hands that frame to vpp_add_online_task -> vgs_add_online_task, which
 * scales it into the extended channel's buffer (#273).  majestic's motion
 * detection reads extended channel 3, and the SDK's sample_ive case 7
 * ("VI->VPSS->IVE->VGS->VENC") takes the same path.
 *
 * A job is a list of register images in DDR, as on VPSS and VEDU.  From
 * mod_vgs.o:
 *
 *   - the "HAL" setters write into a *node image*, not the device:
 *     vgs_hal_set_mask -> image + 0xf0, vgs_hal_set_node_id -> + 0xf4,
 *     vgs_hal_set_next -> + 0xf8.  vgs_drv_comm_working chains the list by
 *     writing each node's physical address into the previous node's + 0xf8
 *     and 0 into the last one's;
 *   - only vgs_hal_start, _get_int_status, _clr_int, _get_node_id and
 *     _set_miscellaneous touch the device.  Start writes the first node's
 *     physical address to + 0xf8, *reads it back* and complains ("not
 *     match") if it did not stick, then writes 1 to + 0x300;
 *   - vgs_drv_comm_irq_proc takes any non-zero bit of + 0x314 [9:0] as a
 *     completion, reads + 0xf4 for the node that finished -- comparing it
 *     against the id it recorded, and logging if they differ -- and clears
 *     + 0x310 with the status it read.
 *
 * So the device answers with the node's own mask as the bits it raises, and
 * reports the last node of the chain in + 0xf4.
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

#define TYPE_HISI_VGS "hisi-vgs"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVgsState, HISI_VGS)

#define VGS_REG_SIZE        0x1000
#define VGS_NODE_SIZE       0x400    /* the image's registers the driver sets */
#define VGS_NODE_MAX        64

#define VGS_MASK            0x0f0
#define VGS_NODE_ID         0x0f4
#define VGS_NEXT            0x0f8
#define VGS_START           0x300
#define VGS_INT_CLR         0x310
#define VGS_INT_STATUS      0x314
#define VGS_MISC            0x330

/* a job takes about this long; completion is never raised inside START */
#define VGS_JOB_NS          200000

struct HisiVgsState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[VGS_REG_SIZE / 4];
    uint32_t status;                /* raised when the job's timer fires */
    uint32_t done_id;
    bool busy;

    char *dump_dir;
    uint32_t dumped;
    bool trace;
};

/*
 * Each node is dumped as its own file, so the layout can be read off a
 * real run (-global hisi-vgs.dump-dir=DIR).  The fields below 0xf0 are not
 * decoded yet (#273); until they are, a job completes without touching the
 * destination, which is what the driver needs to make progress.
 */
static void vgs_dump_node(HisiVgsState *s, uint32_t addr,
                          const uint32_t *node)
{
    g_autofree char *path = NULL;

    if (!s->dump_dir) {
        return;
    }
    path = g_strdup_printf("%s/vgs-node-%04u-%08x.bin", s->dump_dir,
                           s->dumped++, addr);
    g_file_set_contents(path, (const char *)node, VGS_NODE_SIZE, NULL);
}

static void vgs_run_job(HisiVgsState *s)
{
    uint32_t node[VGS_NODE_SIZE / 4];
    uint32_t a = s->regs[VGS_NEXT / 4];
    uint32_t bits = 0;
    int k;

    for (k = 0; k < VGS_NODE_MAX && a; k++) {
        if (dma_memory_read(&address_space_memory, a, node, sizeof(node),
                            MEMTXATTRS_UNSPECIFIED)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-vgs: node %d at 0x%08x unreadable\n", k, a);
            break;
        }
        if (s->trace) {
            fprintf(stderr, "hisi-vgs: node %d at %08x id %u mask %03x "
                    "next %08x\n", k, a, node[VGS_NODE_ID / 4],
                    node[VGS_MASK / 4] & 0x3ff, node[VGS_NEXT / 4]);
        }
        vgs_dump_node(s, a, node);
        /*
         * The node's mask is the set of interrupts it is allowed to raise
         * (the driver writes 0x34a); any of them is a completion to
         * vgs_drv_comm_irq_proc, so a node with none still has to report
         * one or the job is never collected.
         */
        bits |= node[VGS_MASK / 4] & 0x3ff ? node[VGS_MASK / 4] & 0x3ff : 2;
        s->done_id = node[VGS_NODE_ID / 4];
        a = node[VGS_NEXT / 4];
    }
    if (!k) {
        return;
    }
    s->status = bits;
    s->busy = true;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VGS_JOB_NS);
}

static void vgs_job_done(void *opaque)
{
    HisiVgsState *s = opaque;

    s->busy = false;
    s->regs[VGS_NODE_ID / 4] = s->done_id;
    s->regs[VGS_INT_STATUS / 4] = s->status;
    qemu_set_irq(s->irq, 1);
}

static uint64_t hisi_vgs_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVgsState *s = HISI_VGS(opaque);
    uint32_t val = offset < VGS_REG_SIZE ? s->regs[offset / 4] : 0;

    if (s->trace) {
        fprintf(stderr, "hisi-vgs R %04x = %08x\n", (int)offset, val);
    }
    return val;
}

static void hisi_vgs_write(void *opaque, hwaddr offset, uint64_t val64,
                           unsigned size)
{
    HisiVgsState *s = HISI_VGS(opaque);
    uint32_t val = val64;

    if (offset >= VGS_REG_SIZE) {
        return;
    }
    if (s->trace) {
        fprintf(stderr, "hisi-vgs W %04x : %08x\n", (int)offset, val);
    }
    switch (offset) {
    case VGS_START:
        s->regs[offset / 4] = 0;    /* an action, not a bit that stays */
        if (val & 1) {
            vgs_run_job(s);
        }
        break;
    case VGS_INT_CLR:
        s->regs[VGS_INT_STATUS / 4] &= ~(val & 0x3ff);
        if (!s->regs[VGS_INT_STATUS / 4]) {
            qemu_set_irq(s->irq, 0);
        }
        break;
    case VGS_INT_STATUS:
        break;                      /* raised by the engine only */
    default:
        s->regs[offset / 4] = val;
        break;
    }
}

static const MemoryRegionOps hisi_vgs_ops = {
    .read = hisi_vgs_read,
    .write = hisi_vgs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vgs_reset(DeviceState *dev)
{
    HisiVgsState *s = HISI_VGS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    s->done_id = 0;
    s->busy = false;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void hisi_vgs_realize(DeviceState *dev, Error **errp)
{
    HisiVgsState *s = HISI_VGS(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vgs_job_done, s);
}

static void hisi_vgs_init(Object *obj)
{
    HisiVgsState *s = HISI_VGS(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_vgs_ops, s, TYPE_HISI_VGS,
                          VGS_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_hisi_vgs = {
    .name = TYPE_HISI_VGS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, HisiVgsState, VGS_REG_SIZE / 4),
        VMSTATE_UINT32(status, HisiVgsState),
        VMSTATE_UINT32(done_id, HisiVgsState),
        VMSTATE_BOOL(busy, HisiVgsState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property hisi_vgs_properties[] = {
    DEFINE_PROP_STRING("dump-dir", HisiVgsState, dump_dir),
    DEFINE_PROP_BOOL("trace", HisiVgsState, trace, false),
};

static void hisi_vgs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_vgs_realize;
    device_class_set_legacy_reset(dc, hisi_vgs_reset);
    dc->vmsd = &vmstate_hisi_vgs;
    device_class_set_props(dc, hisi_vgs_properties);
}

static const TypeInfo hisi_vgs_info = {
    .name = TYPE_HISI_VGS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiVgsState),
    .instance_init = hisi_vgs_init,
    .class_init = hisi_vgs_class_init,
};

static void hisi_vgs_register_types(void)
{
    type_register_static(&hisi_vgs_info);
}

type_init(hisi_vgs_register_types)
