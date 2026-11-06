/*
 * Ambarella S3L DSP (ORC) stand-in, enough of the iav<->DSP interface for a
 * software encoder to take the place of the microcode the real part runs.
 *
 * The ORC is a polling coprocessor, not a register block: iav.ko and the DSP
 * talk through buffers at the top of the /iavmem region, synchronised with
 * sequence numbers, and the DSP drives everything from a frame-rate interrupt
 * it raises itself.  So this models a heartbeat: a timer that services the
 * command buffer and answers in the message buffer, which is what makes iav
 * believe a DSP booted.  Video encoding is layered on top of this later; for
 * now it carries the boot handshake (chip id + the DSP status message) so the
 * driver gets past DSP init.
 *
 * All addresses are derived from the iavmem region the machine already models.
 * The DSP area is top-anchored in it: the microcode window is the last 5 MiB,
 * and the log, bitstream-header fifo, message and command buffers sit just
 * below it, so their physical addresses follow from the region's end alone,
 * whatever the kernel reserved lower down.  The chip-id word and the init-data
 * block live at fixed offsets inside the microcode window.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/dma.h"

#define TYPE_AMB_DSP "amb-dsp"
OBJECT_DECLARE_SIMPLE_TYPE(AmbDspState, AMB_DSP)

/* DSP area, counted down from the top of /iavmem (see amba_arch_mem.h). */
#define DSP_UCODE_SIZE      (5 * MiB)
#define DSP_LOG_SIZE        (128 * KiB)
#define DSP_BSH_SIZE        (64 * 256)      /* DSP_BS_UNIT_SIZE * DSP_BS_NUM */
#define DSP_PORT_SIZE       (4 * KiB)       /* one command/message port */
#define DSP_CMD_BUF_SIZE    (DSP_PORT_SIZE * 2)
#define DSP_MSG_BUF_SIZE    (DSP_PORT_SIZE * 2)

/* Offsets inside the microcode window. */
#define DSP_CHIP_ID_OFFSET   0x424000
#define DSP_INIT_DATA_OFFSET 0x424408      /* chip-id + 4 + 1 KiB + 4 */

/* The S3Lm part id the loader also selects by (RCT 0x300 == 0x4a000000). */
#define AMBA_CHIP_ID_S3L_66  4

/* enum dsp_op_mode */
#define DSP_OP_MODE_IDLE          0
#define DSP_OP_MODE_CAMERA_RECORD 1
#define DSP_OP_MODE_INIT          255

/* command/message category and the codes we care about */
#define CMD_DSP_SET_OPERATION_MODE 0x01000001
#define MSG_DSP_STATUS             0x81000001

/* DSP_HEADER_CMD: the first command in each iteration (128 bytes). */
#define CMD_HDR_CODE_OFF   0
#define CMD_HDR_SEQ_OFF    4
#define CMD_HDR_NUM_OFF    8
#define DSP_CMD_SIZE       128

/* DSP_STATUS_MSG: the first, implicit message in a port (28 bytes). */
struct dsp_status_msg {
    uint32_t msg_code;
    uint32_t dsp_mode;
    uint32_t time_code;
    uint32_t prev_cmd_seq;
    uint32_t prev_cmd_echo;
    uint32_t prev_num_cmds;
    uint32_t num_msgs;
};

#define AMB_DSP_HZ 30

struct AmbDspState {
    SysBusDevice parent_obj;

    uint32_t iavmem_base;
    uint32_t iavmem_end;        /* one past the last DRAM byte */

    QEMUTimer heartbeat;
    qemu_irq vdsp_irq;
    qemu_irq vcap_irq;
    qemu_irq venc_irq;

    bool chip_written;
    uint32_t op_mode;           /* what we report back to the driver */
};

static hwaddr amb_dsp_ucode(AmbDspState *s)
{
    return s->iavmem_end - DSP_UCODE_SIZE;
}

/* Command buffer (port 0) physical address. */
static hwaddr amb_dsp_cmd_buf(AmbDspState *s)
{
    return amb_dsp_ucode(s) - DSP_LOG_SIZE - DSP_BSH_SIZE
           - DSP_MSG_BUF_SIZE - DSP_CMD_BUF_SIZE;
}

/* Message buffer (port 0) physical address. */
static hwaddr amb_dsp_msg_buf(AmbDspState *s)
{
    return amb_dsp_cmd_buf(s) + DSP_CMD_BUF_SIZE;
}

static uint32_t amb_dsp_ld(hwaddr a)
{
    uint32_t v = 0;
    dma_memory_read(&address_space_memory, a, &v, sizeof(v),
                    MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

static void amb_dsp_st(hwaddr a, uint32_t v)
{
    uint32_t le = cpu_to_le32(v);
    dma_memory_write(&address_space_memory, a, &le, sizeof(le),
                     MEMTXATTRS_UNSPECIFIED);
}

/*
 * Read the sequence number the driver last wrote into a command port, and
 * note any operation-mode request so we report the mode it asked for.
 */
static uint32_t amb_dsp_service_port(AmbDspState *s, hwaddr cmd_port)
{
    uint32_t seq = amb_dsp_ld(cmd_port + CMD_HDR_SEQ_OFF);
    uint32_t num = amb_dsp_ld(cmd_port + CMD_HDR_NUM_OFF);

    /* Walk the iteration's commands for a set-operation-mode. */
    for (uint32_t i = 0; i < num && i < DSP_PORT_SIZE / DSP_CMD_SIZE; i++) {
        hwaddr c = cmd_port + (i + 1) * DSP_CMD_SIZE;
        if (amb_dsp_ld(c + CMD_HDR_CODE_OFF) == CMD_DSP_SET_OPERATION_MODE) {
            uint32_t m = amb_dsp_ld(c + 4) & 0xff;
            if (m == DSP_OP_MODE_CAMERA_RECORD || m == DSP_OP_MODE_IDLE) {
                s->op_mode = m;
            }
        }
    }
    return seq;
}

static void amb_dsp_reply(AmbDspState *s, hwaddr msg_port, uint32_t seq)
{
    struct dsp_status_msg m = {
        .msg_code = MSG_DSP_STATUS,
        .dsp_mode = s->op_mode,
        .prev_cmd_seq = seq,
        .num_msgs = 0,
    };
    uint32_t w[7] = {
        cpu_to_le32(m.msg_code), cpu_to_le32(m.dsp_mode),
        cpu_to_le32(m.time_code), cpu_to_le32(m.prev_cmd_seq),
        cpu_to_le32(m.prev_cmd_echo), cpu_to_le32(m.prev_num_cmds),
        cpu_to_le32(m.num_msgs),
    };
    dma_memory_write(&address_space_memory, msg_port, w, sizeof(w),
                     MEMTXATTRS_UNSPECIFIED);
}

static void amb_dsp_tick(void *opaque)
{
    AmbDspState *s = opaque;
    hwaddr cmd = amb_dsp_cmd_buf(s);
    hwaddr msg = amb_dsp_msg_buf(s);
    uint32_t seq_gen, seq_vcap;

    if (!s->chip_written) {
        amb_dsp_st(amb_dsp_ucode(s) + DSP_CHIP_ID_OFFSET, AMBA_CHIP_ID_S3L_66);
        s->chip_written = true;
    }

    /* General and vcap ports are the two 4 KiB halves of each buffer. */
    seq_gen = amb_dsp_service_port(s, cmd);
    seq_vcap = amb_dsp_service_port(s, cmd + DSP_PORT_SIZE);
    amb_dsp_reply(s, msg, seq_gen);
    amb_dsp_reply(s, msg + DSP_PORT_SIZE, seq_vcap);

    /* The driver's vdsp/vcap handlers drain the message ports. */
    qemu_irq_pulse(s->vdsp_irq);
    qemu_irq_pulse(s->vcap_irq);

    timer_mod(&s->heartbeat,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / AMB_DSP_HZ);
}

static void amb_dsp_reset(DeviceState *dev)
{
    AmbDspState *s = AMB_DSP(dev);

    s->chip_written = false;
    s->op_mode = DSP_OP_MODE_INIT;
    timer_mod(&s->heartbeat,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / AMB_DSP_HZ);
}

static void amb_dsp_realize(DeviceState *dev, Error **errp)
{
    AmbDspState *s = AMB_DSP(dev);

    if (s->iavmem_end <= s->iavmem_base + DSP_UCODE_SIZE) {
        error_setg(errp, "amb-dsp: iavmem region too small for the DSP area");
        return;
    }
    timer_init_ns(&s->heartbeat, QEMU_CLOCK_VIRTUAL, amb_dsp_tick, s);
}

static void amb_dsp_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    AmbDspState *s = AMB_DSP(obj);

    sysbus_init_irq(sbd, &s->vdsp_irq);
    sysbus_init_irq(sbd, &s->vcap_irq);
    sysbus_init_irq(sbd, &s->venc_irq);
}

static const Property amb_dsp_props[] = {
    DEFINE_PROP_UINT32("iavmem-base", AmbDspState, iavmem_base, 0),
    DEFINE_PROP_UINT32("iavmem-end", AmbDspState, iavmem_end, 0),
};

static const VMStateDescription vmstate_amb_dsp = {
    .name = "amb-dsp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(heartbeat, AmbDspState),
        VMSTATE_BOOL(chip_written, AmbDspState),
        VMSTATE_UINT32(op_mode, AmbDspState),
        VMSTATE_END_OF_LIST()
    },
};

static void amb_dsp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = amb_dsp_realize;
    device_class_set_legacy_reset(dc, amb_dsp_reset);
    dc->vmsd = &vmstate_amb_dsp;
    device_class_set_props(dc, amb_dsp_props);
}

static const TypeInfo amb_dsp_info = {
    .name = TYPE_AMB_DSP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AmbDspState),
    .instance_init = amb_dsp_init,
    .class_init = amb_dsp_class_init,
};

static void amb_dsp_register(void)
{
    type_register_static(&amb_dsp_info);
}
type_init(amb_dsp_register)
