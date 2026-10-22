/*
 * Novatek NA51089 (NT9856x) MIPI DSI host (0xF0840000, SPI 50), for the
 * DSI panels: it carries the panel's set-up commands, and then the IDE's
 * picture (nvt-ide, which QEMU shows) to the panel.
 *
 * Registers from kdrv_videoout/dsi/include/dsi_reg.h, as the closed
 * dsi.o drives them (the vendor U-Boot's nvt-na51055/dsi is the same
 * driver as source):
 *
 *   0x00 EN        bit 0 TX on, 1 PHY on, 4..7 lane 0..3 escape trigger,
 *                  9 clock lane ULPS trigger
 *   0x04 CTRL0     [2:0] mode: 0 manual (send CMD0..), 1..3 automatic
 *                  (video / command frames)
 *   0x24 ESCCTRL0  bits 0..3 the next trigger on lane n is an escape
 *                  entry command; [5:4] + 2n stop after it
 *   0x28 ESCCTRL1  byte n: lane n's escape command / LP data byte
 *   0x2C CMDCTRL   [2:0] commands - 1, bit 8 / 9 bus turnaround (a read),
 *                  [28:20] bytes read back (read only)
 *   0x30..0x4C     CMD0..7
 *   0x80 / 0x84    interrupt enable / status, write 1 to clear: bit 0 TX
 *                  done, 1 frame end, 8 read data ready, 24 clock ULPS
 *                  done, 25..28 lane 0..3 escape done
 *
 * The driver waits on each of these with no timeout.  Every shipped panel
 * (nt35510, tb093, ili9488 / ut35067a0) sends its commands as low-power
 * data, a byte and an escape-done interrupt at a time; then the IDE is
 * started as a DSI device and the host goes automatic, frames streaming
 * until the driver closes, which waits for a frame end.  So:
 *
 *   - escape and ULPS triggers clear themselves and their done bit rises;
 *   - TX on in manual mode sends the commands, clears itself and raises
 *     TX done (a read gets no bytes back -- no panel reads);
 *   - TX on in an automatic mode raises frame end at 60 Hz until TX goes
 *     off, which raises TX done.
 *
 * Events come a few microseconds after the write, as the interrupt the
 * driver waits for would.  "trace" logs the low-power packets the panel
 * driver sends, as bytes.
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

#define TYPE_NVT_DSI "nvt-dsi"
OBJECT_DECLARE_SIMPLE_TYPE(NvtDsiState, NVT_DSI)

#define DSI_SIZE            0x1000
#define DSI_REGS            0x200

#define DSI_EN              0x00
#define DSI_CTRL0           0x04
#define DSI_ESCCTRL0        0x24
#define DSI_ESCCTRL1        0x28
#define DSI_CMDCTRL         0x2C
#define DSI_INTEN           0x80
#define DSI_INTSTS          0x84

#define EN_TX               (1u << 0)
#define EN_ESC_TRIG(n)      (1u << (4 + (n)))
#define EN_CLK_ULP_TRIG     (1u << 9)

#define CMDCTRL_BTA         (3u << 8)
#define CMDCTRL_RD_CNT      (0x1FFu << 20)

#define STS_TX_DIS          (1u << 0)
#define STS_FRM_END         (1u << 1)
#define STS_ULPS_DONE       (1u << 24)
#define STS_ESC_DONE(n)     (1u << (25 + (n)))

#define DSI_EVENT_NS        2000
#define DSI_FRAME_NS        (NANOSECONDS_PER_SECOND / 60)
#define DSI_LP_MAX          64

struct NvtDsiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *done;            /* raises the events below */
    QEMUTimer *frame;           /* frame end, in an automatic mode */
    uint32_t regs[DSI_REGS / 4];
    uint32_t pending;           /* status bits the done timer raises */
    uint8_t lp[DSI_LP_MAX];     /* the low-power packet so far (trace) */
    uint32_t lp_len;
    bool trace;
};

static uint32_t *reg(NvtDsiState *s, hwaddr off)
{
    return &s->regs[off / 4];
}

static void nvt_dsi_update_irq(NvtDsiState *s)
{
    qemu_set_irq(s->irq, (*reg(s, DSI_INTSTS) & *reg(s, DSI_INTEN)) != 0);
}

static void nvt_dsi_done(void *opaque)
{
    NvtDsiState *s = opaque;

    *reg(s, DSI_INTSTS) |= s->pending;
    s->pending = 0;
    nvt_dsi_update_irq(s);
}

static void nvt_dsi_post(NvtDsiState *s, uint32_t sts)
{
    s->pending |= sts;
    timer_mod(s->done, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DSI_EVENT_NS);
}

static void nvt_dsi_frame(void *opaque)
{
    NvtDsiState *s = opaque;

    if (!(*reg(s, DSI_EN) & EN_TX)) {
        return;
    }
    *reg(s, DSI_INTSTS) |= STS_FRM_END;
    nvt_dsi_update_irq(s);
    timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DSI_FRAME_NS);
}

static void nvt_dsi_lp_flush(NvtDsiState *s)
{
    g_autoptr(GString) str = g_string_new(NULL);
    uint32_t i;

    if (s->trace && s->lp_len) {
        for (i = 0; i < s->lp_len; i++) {
            g_string_append_printf(str, " %02x", s->lp[i]);
        }
        qemu_log_mask(LOG_UNIMP, "nvt-dsi: LP packet%s\n", str->str);
    }
    s->lp_len = 0;
}

/* lane n's escape trigger: an entry command, or a byte of LP data */
static void nvt_dsi_escape(NvtDsiState *s, unsigned n)
{
    uint32_t esc0 = *reg(s, DSI_ESCCTRL0);
    uint8_t byte = *reg(s, DSI_ESCCTRL1) >> (8 * n);

    if (esc0 & (1u << n)) {
        s->lp_len = 0;                  /* entry: a new transmission */
    } else if (s->lp_len < DSI_LP_MAX) {
        s->lp[s->lp_len++] = byte;
    }
    if ((esc0 >> (4 + 2 * n)) & 3) {
        nvt_dsi_lp_flush(s);            /* stop: the packet is out */
    }
}

static uint64_t nvt_dsi_read(void *opaque, hwaddr off, unsigned size)
{
    NvtDsiState *s = opaque;

    off &= ~3;
    return off < DSI_REGS ? *reg(s, off) : 0;
}

static void nvt_dsi_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    NvtDsiState *s = opaque;
    uint32_t val = value, old, ev = 0;
    unsigned n;

    off &= ~3;
    if (off >= DSI_REGS) {
        return;
    }
    switch (off) {
    case DSI_INTSTS:
        *reg(s, off) &= ~val;
        nvt_dsi_update_irq(s);
        return;
    case DSI_INTEN:
        *reg(s, off) = val;
        nvt_dsi_update_irq(s);
        return;
    case DSI_CMDCTRL:
        *reg(s, off) = (val & ~CMDCTRL_RD_CNT) |
                       (*reg(s, off) & CMDCTRL_RD_CNT);
        return;
    case DSI_EN:
        old = *reg(s, off);
        *reg(s, off) = val;
        for (n = 0; n < 4; n++) {
            if (val & EN_ESC_TRIG(n)) {
                nvt_dsi_escape(s, n);
                *reg(s, off) &= ~EN_ESC_TRIG(n);
                ev |= STS_ESC_DONE(n);
            }
        }
        if (val & EN_CLK_ULP_TRIG) {
            *reg(s, off) &= ~EN_CLK_ULP_TRIG;
            ev |= STS_ULPS_DONE;
        }
        if ((val & EN_TX) && !(old & EN_TX)) {
            if (!(*reg(s, DSI_CTRL0) & 7)) {
                /* manual: the commands go out; a read gets nothing */
                if (*reg(s, DSI_CMDCTRL) & CMDCTRL_BTA) {
                    *reg(s, DSI_CMDCTRL) &= ~CMDCTRL_RD_CNT;
                }
                *reg(s, off) &= ~EN_TX;
                ev |= STS_TX_DIS;
            } else {
                timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          DSI_FRAME_NS);
            }
        } else if (!(val & EN_TX) && (old & EN_TX)) {
            timer_del(s->frame);
            ev |= STS_TX_DIS;
        }
        if (ev) {
            nvt_dsi_post(s, ev);
        }
        return;
    }
    *reg(s, off) = val;
}

static const MemoryRegionOps nvt_dsi_ops = {
    .read = nvt_dsi_read,
    .write = nvt_dsi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nvt_dsi_reset(DeviceState *dev)
{
    NvtDsiState *s = NVT_DSI(dev);

    timer_del(s->done);
    timer_del(s->frame);
    memset(s->regs, 0, sizeof(s->regs));
    s->pending = 0;
    s->lp_len = 0;
    qemu_irq_lower(s->irq);
}

static void nvt_dsi_init(Object *obj)
{
    NvtDsiState *s = NVT_DSI(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_dsi_ops, s, TYPE_NVT_DSI,
                          DSI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->done = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_dsi_done, s);
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvt_dsi_frame, s);
}

static void nvt_dsi_finalize(Object *obj)
{
    NvtDsiState *s = NVT_DSI(obj);

    timer_free(s->done);
    timer_free(s->frame);
}

static const Property nvt_dsi_properties[] = {
    DEFINE_PROP_BOOL("trace", NvtDsiState, trace, false),
};

static const VMStateDescription vmstate_nvt_dsi = {
    .name = TYPE_NVT_DSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtDsiState, DSI_REGS / 4),
        VMSTATE_UINT32(pending, NvtDsiState),
        VMSTATE_TIMER_PTR(done, NvtDsiState),
        VMSTATE_TIMER_PTR(frame, NvtDsiState),
        VMSTATE_END_OF_LIST()
    },
};

static void nvt_dsi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, nvt_dsi_reset);
    device_class_set_props(dc, nvt_dsi_properties);
    dc->vmsd = &vmstate_nvt_dsi;
}

static const TypeInfo nvt_dsi_info = {
    .name = TYPE_NVT_DSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtDsiState),
    .instance_init = nvt_dsi_init,
    .instance_finalize = nvt_dsi_finalize,
    .class_init = nvt_dsi_class_init,
};

static void nvt_dsi_register_types(void)
{
    type_register_static(&nvt_dsi_info);
}

type_init(nvt_dsi_register_types)
