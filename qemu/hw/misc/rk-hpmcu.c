/*
 * Rockchip RV1103B HPMCU, the small RISC-V core that sequences the
 * ISP -> VEPU "wrap" (online) encoding, modelled at the level of what its
 * firmware does (rockit.ko's hpmcu_wrap.bin, "mcu version[2.0.3]rc2",
 * 5280 bytes, loaded at 0x210F6000), and the mailbox it talks through
 * (0x20A10000, the rockchip-mailbox layout).
 *
 * rockit.ko (vmcu_probe) checks the core is held in reset (CRU
 * SOFTRST_CON(10) at 0x20000A28 bit 4, set by the machine's reset
 * values), copies the firmware, sets its boot address (GRF 0x20250080)
 * and releases it.  It then writes commands as A2B_DAT (+0x0c) then
 * A2B_CMD (+0x08):
 *
 *   1  wrap config (a table in DDR)    2  the log buffer
 *   3  wrap start, data the pipe       4  wrap end
 *   7, 8, 9 (rotating so each write is new): the frame the ISP starts,
 *      data pipe | frame id << 2
 *   10 AVS config  11 flush the log    6 halt
 *
 * The firmware polls the ISP's line counter (ISP_DEBUG2 0x64c) and RIS,
 * feeds the VEPU the lines written so far (0x018), writes its ENC_START
 * itself when the encoder is armed (DVBM_CFG 0x060 bit 0) for the pipe
 * the ISP is on (VEPU 0x308 bits 13:12), and posts events back as B2A_CMD
 * (+0x18, a sequence 12, 13, 14, 12 ...) and B2A_DAT (+0x1c, pipe << 16 |
 * event: 1..3 a quarter of the frame further, 4 the frame end).  rockit's
 * vmcu thread polls B2A_CMD every millisecond; each new value completes
 * the waiter for that pipe in vvi_mb_get, which hands the wrap buffer to
 * the encoder for the next frame.  Without those events only the first
 * online frame was ever encoded.
 *
 * The model's ISP writes the whole frame at its end, so this posts the
 * frame end (event 4) then, and starts an armed encoder for the current
 * pipe there ("isp-frame" in, "vepu-start" out).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_RK_HPMCU "rk-hpmcu"
OBJECT_DECLARE_SIMPLE_TYPE(RkHpmcuState, RK_HPMCU)

#define R_A2B_CMD       0x08
#define R_A2B_DAT       0x0c
#define R_B2A_CMD       0x18
#define R_B2A_DAT       0x1c
#define MBOX_SIZE       0x1000

#define VEPU_DVBM_CFG   0x060
#define VEPU_PIPE       0x308   /* bits 13:12 the ISP pipe it encodes */

#define EV_FRAME_END    4

struct RkHpmcuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq vepu_start;

    uint32_t regs[MBOX_SIZE / 4];
    uint64_t vepu_base;         /* property: the VEPU's registers */
    uint32_t running;           /* between wrap start and end */
    uint32_t pipe;              /* the pipe the ISP is on */
    uint32_t seq;               /* the next B2A_CMD, 12..14 */
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_hpmcu_post(RkHpmcuState *s, uint32_t pipe, uint32_t ev)
{
    REG(s, R_B2A_DAT) = pipe << 16 | ev;
    REG(s, R_B2A_CMD) = s->seq;
    s->seq = s->seq >= 14 ? 12 : s->seq + 1;
}

static void rk_hpmcu_command(RkHpmcuState *s, uint32_t cmd, uint32_t dat)
{
    switch (cmd) {
    case 3:                             /* wrap start */
        s->running = 1;
        s->pipe = dat & 3;
        break;
    case 4:                             /* wrap end */
        s->running = 0;
        break;
    case 7: case 8: case 9:             /* the frame the ISP starts */
        s->pipe = dat & 3;
        break;
    }
}

/* the ISP wrote a mainpath frame */
static void rk_hpmcu_isp_frame(void *opaque, int n, int level)
{
    RkHpmcuState *s = opaque;
    uint32_t cfg, pipe;

    if (!level || !s->running) {
        return;
    }
    cfg = address_space_ldl_le(&address_space_memory,
                               s->vepu_base + VEPU_DVBM_CFG,
                               MEMTXATTRS_UNSPECIFIED, NULL);
    pipe = address_space_ldl_le(&address_space_memory,
                                s->vepu_base + VEPU_PIPE,
                                MEMTXATTRS_UNSPECIFIED, NULL) >> 12 & 3;
    if ((cfg & 1) && pipe == s->pipe) {
        qemu_irq_pulse(s->vepu_start);
    }
    rk_hpmcu_post(s, s->pipe, EV_FRAME_END);
}

static uint64_t rk_hpmcu_read(void *opaque, hwaddr off, unsigned size)
{
    RkHpmcuState *s = opaque;

    return s->regs[off / 4];
}

static void rk_hpmcu_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    RkHpmcuState *s = opaque;

    s->regs[off / 4] = val;
    if (off == R_A2B_CMD) {
        rk_hpmcu_command(s, val, REG(s, R_A2B_DAT));
    }
}

static const MemoryRegionOps rk_hpmcu_ops = {
    .read = rk_hpmcu_read,
    .write = rk_hpmcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_hpmcu_reset(DeviceState *dev)
{
    RkHpmcuState *s = RK_HPMCU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->running = 0;
    s->pipe = 0;
    s->seq = 12;
}

static void rk_hpmcu_init(Object *obj)
{
    RkHpmcuState *s = RK_HPMCU(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rk_hpmcu_ops, s,
                          TYPE_RK_HPMCU, MBOX_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(dev, rk_hpmcu_isp_frame, "isp-frame", 1);
    qdev_init_gpio_out_named(dev, &s->vepu_start, "vepu-start", 1);
}

static const Property rk_hpmcu_properties[] = {
    DEFINE_PROP_UINT64("vepu-base", RkHpmcuState, vepu_base, 0),
};

static const VMStateDescription rk_hpmcu_vmstate = {
    .name = TYPE_RK_HPMCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkHpmcuState, MBOX_SIZE / 4),
        VMSTATE_UINT32(running, RkHpmcuState),
        VMSTATE_UINT32(pipe, RkHpmcuState),
        VMSTATE_UINT32(seq, RkHpmcuState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_hpmcu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_hpmcu_reset);
    device_class_set_props(dc, rk_hpmcu_properties);
    dc->vmsd = &rk_hpmcu_vmstate;
}

static const TypeInfo rk_hpmcu_info = {
    .name = TYPE_RK_HPMCU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkHpmcuState),
    .instance_init = rk_hpmcu_init,
    .class_init = rk_hpmcu_class_init,
};

static void rk_hpmcu_register_types(void)
{
    type_register_static(&rk_hpmcu_info);
}

type_init(rk_hpmcu_register_types)
