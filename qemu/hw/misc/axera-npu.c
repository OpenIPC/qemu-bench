/*
 * Axera AX620U NPU (ax_npu.ko, "axera,npu" @ 0x03800000, 0x7000):
 * job-completion pass-through
 *
 * No compute is modelled. A job submitted to an execution unit (EU)
 * completes after a short delay without touching its output buffers,
 * which is enough for the closed driver to treat it as run: the App's
 * detection models and the ISP's AI-ISP (raw denoise between IFE and
 * ITP) stop timing out and the pipeline keeps moving.
 *
 * Register map, from disassembling the real ARM32 ax_npu.ko shipped in
 * the AX620U firmware (unstripped):
 *
 *   EU n  @ n * 0x1000 (n = 0..4)
 *     +0x14  command-queue address: (paddr >> 5) | (paddr[34:32] << 27)
 *     +0x18  written 0x23 before each kick
 *     +0x10  kick: command count (npu_run_at_cmdq() writes it last)
 *     +0x7fc idle = 1 (npu_wait_used_eu_idle() polls it before a job)
 *   sync manager @ 0x5000 (sync_manager_*())
 *     +0x00  interrupt status, bit n = EU n finished
 *     +0x04  write-1-to-clear for +0x00
 *     +0x1c  software clear (0xfffff = everything)
 *     +0x28.. per-EU wait-cycle limits
 *     +0x48 + 8n / +0x4c + 8n  per-EU job ids (only shown by
 *            npu_show_status(), never checked on completion)
 *   fabric @ 0x6000
 *
 * Interrupts (AX620U /proc/interrupts): npu_normal GIC 68 = SPI 36
 * (level), npu_top 37, npu_fab 38, npu_err 39. npu_irq_normal_handle()
 * reads +0x00, clears it through +0x04 and hands the value to the IRQ
 * thread; npu_irq_wait_handle() turns it into {0, value & 0x1f} and the
 * waiting job (irq_get_err_code()) succeeds when that first word is 0.
 *
 * Set AXERA_NPU_DEBUG=1 to log kicks.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_AXERA_NPU "axera-npu"
OBJECT_DECLARE_SIMPLE_TYPE(AxeraNpuState, AXERA_NPU)

#define NPU_SIZE        0x7000
#define NPU_EUS         5
#define EU_STRIDE       0x1000
#define EU_KICK         0x10
#define EU_IDLE         0x7fc
#define SM_BASE         0x5000
#define SM_STATUS       (SM_BASE + 0x00)
#define SM_CLEAR        (SM_BASE + 0x04)
#define SM_SW_CLEAR     (SM_BASE + 0x1c)
#define NPU_JOB_NS      (2 * SCALE_MS)

struct AxeraNpuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[NPU_SIZE / 4];
    uint32_t status;
    uint32_t pending;
    bool debug;
};

static void npu_update_irq(AxeraNpuState *s)
{
    qemu_set_irq(s->irq, s->status != 0);
}

static void npu_complete(void *opaque)
{
    AxeraNpuState *s = opaque;

    s->status |= s->pending;
    s->pending = 0;
    npu_update_irq(s);
}

static uint64_t npu_read(void *opaque, hwaddr off, unsigned size)
{
    AxeraNpuState *s = opaque;

    if (off == SM_STATUS) {
        return s->status;
    }
    if (off < NPU_EUS * EU_STRIDE && (off % EU_STRIDE) == EU_IDLE) {
        return !(s->pending & BIT(off / EU_STRIDE));
    }
    return s->regs[off / 4];
}

static void npu_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    AxeraNpuState *s = opaque;

    s->regs[off / 4] = val;

    if (off < NPU_EUS * EU_STRIDE && (off % EU_STRIDE) == EU_KICK && val) {
        int eu = off / EU_STRIDE;
        uint32_t q = s->regs[(eu * EU_STRIDE + 0x14) / 4];

        if (s->debug) {
            qemu_log("axera-npu: EU%d kick count 0x%x cmdq 0x%" PRIx64 "\n",
                     eu, (uint32_t)val,
                     ((uint64_t)(q & 0x07ffffff) << 5) |
                     ((uint64_t)(q >> 27) << 32));
        }
        s->pending |= BIT(eu);
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NPU_JOB_NS);
        return;
    }
    switch (off) {
    case SM_CLEAR:
        s->status &= ~(uint32_t)val;
        npu_update_irq(s);
        break;
    case SM_SW_CLEAR:
        s->status &= ~(uint32_t)val;
        npu_update_irq(s);
        break;
    }
}

static const MemoryRegionOps npu_ops = {
    .read = npu_read,
    .write = npu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void npu_reset(DeviceState *dev)
{
    AxeraNpuState *s = AXERA_NPU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    s->pending = 0;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void npu_realize(DeviceState *dev, Error **errp)
{
    AxeraNpuState *s = AXERA_NPU(dev);
    const char *dbg = getenv("AXERA_NPU_DEBUG");

    s->debug = dbg && dbg[0] == '1';
    memory_region_init_io(&s->iomem, OBJECT(s), &npu_ops, s, TYPE_AXERA_NPU,
                          NPU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, npu_complete, s);
}

static void npu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = npu_realize;
    device_class_set_legacy_reset(dc, npu_reset);
}

static const TypeInfo npu_info = {
    .name          = TYPE_AXERA_NPU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AxeraNpuState),
    .class_init    = npu_class_init,
};

static void npu_register_types(void)
{
    type_register_static(&npu_info);
}

type_init(npu_register_types)
