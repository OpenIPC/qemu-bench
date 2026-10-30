/*
 * Ingenic T20 / T30 image pipeline: the Apical ISP core, the VIC, the
 * interrupt block that combines them, and the MIPI CSI-2 host -- what the
 * vendor tx-isp-t20.ko (software "H201709151519", the Apical firmware
 * "H01-380") and tx-isp-t30.ko (H20190725b) drive.  (T21, T23 and T31 are
 * the later design, ingenic-isp.c.)  Read off the stripped modules of the
 * OpenIPC rootfs; the function addresses are in tx-isp-t20.ko .text.
 * There is no sensor data: each frame the model paints a test picture
 * (ingenic-isp-paint.h) into the buffers the driver gave the core's DMA
 * writers.
 *
 *   core      0x13380000, 512 KiB: storage (libimp also maps 0x1B000 of
 *             it through /dev/mem and tunes it directly), except
 *     0x0A0   status of the 16 interrupt slots (the ISR at 0x10eb4)
 *     0x0A4   [15:0] write 1 to clear a slot's status
 *     0x134   [2:0] the state 0x130 [2:0] asked for (the run state; the
 *             stop path writes 0 there and spins until this reads 0:
 *             apical_wdr_fs_isp_setup 0x31554)
 *     0x080 + 4 (slot / 2)  each slot's event, 6 bits, even slots in
 *             [5:0], odd in [21:16] (system_program_interrupt_event
 *             0x1193c); an event sets the status of every slot that names
 *             it, except event 0, which the unprogrammed slots name.
 *             Events: 24 frame start, 23 frame end, 14 / 15 / 1 the DMA
 *             writers of channels 0 / 1 / 2 (FR, DS1, DS2) done, 50 / 53
 *             DS1 / DS2 frame end (apical_fw_interrupts_init 0x15fd4)
 *     W = 0xB00 + 0x100 n, n = 0..2: channel n's DMA writer, Y at W,
 *             CbCr at W + 0x80 with the same layout:
 *     +0x00   [3:0] format, bit 9 enable (set at STREAMON, 0xfcc4)
 *     +0x04   width | height << 16
 *     +0x08 + 4 b   bank b's address, b = 0..4
 *     +0x1C   [2:0] banks - 1, bit 3 restart at bank 0
 *     +0x20   line offset, signed (negative for a vertical flip)
 *     +0x24   write 2: the next bank holds a buffer, 3: it does not (the
 *             frame after this one is dropped); read [10:8] the current
 *             bank, [13:11] the last completed one, bit 16 busy (0: the
 *             stream-off path spins on it with no timeout)
 *   VIC       0x13300000, 128 KiB: storage, except 0x00: 2 (reset) reads
 *             back 0 at once (the MIPI start spins on it), 1 starts the
 *             input, 8 stops it.  T10's VIC has that control at 0x20
 *             (its 0x00 is configuration, 0x0C the size): "vic-ctrl"
 *   interrupt block  0x13320000: +0x0C status, +0x1C mask (1 = masked),
 *             +0x04 write the pending bits to acknowledge them, +0x14
 *             enable; the one line (IRQ 37, INTC 29) is high while
 *             status & enable & ~mask.  Status bits 15:0 are the core's
 *             slot status (the handler at 0x4bcc does nothing unless one
 *             is set), bit 16 VIC frame done (the counter in
 *             /proc/jz/vic/isp_vic_frd that libimp watches)
 *   CSI host  0x10040000: DesignWare CSI-2 storage; PHY_STATE (0x14) reads
 *             0x630, lanes and clock in stop state, which the init polls
 *             for (0x42c8, "csi init failure!")
 *
 * A bank's buffer is given by the driver from its ISR at frame start and
 * frame end (0x10b28): it fills the empty banks from the current one on,
 * and at each buffer-done writes 2 or 3 to +0x24 according to whether the
 * bank after the current one is full.  So at a frame start the writer
 * moves on to the next bank if the last control written was 2 (the first
 * frame after the enable uses bank 0) and otherwise drops the frame.
 *
 * Each frame, while the VIC runs and the block's enable is not zero:
 * event 24; half a frame later, for each enabled writer that took a
 * bank, the picture at its size and strides into the bank's buffer,
 * [10:8] = that bank and its done event; then events 23, 50, 53 and VIC
 * frame done.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "ingenic-isp-paint.h"
#include "qom/object.h"

#define TYPE_INGENIC_ISP_APICAL "ingenic-isp-apical"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIspApicalState, INGENIC_ISP_APICAL)

#define CORE_SIZE       0x80000
#define VIC_SIZE        0x20000
#define IRQB_SIZE       0x1000
#define CSI_SIZE        0x1000

#define CORE_EVENT      0x080
#define CORE_STA        0x0A0
#define CORE_CLR        0x0A4
#define CORE_STATE      0x130
#define CORE_STATE_NOW  0x134
#define WR(n)           (0xB00 + 0x100 * (n))
#define WR_FMT          0x00
#define WR_FMT_EN       (1u << 9)
#define WR_SIZE         0x04
#define WR_BANK         0x08
#define WR_BANKS        0x1C
#define WR_BANKS_RESTART (1u << 3)
#define WR_STRIDE       0x20
#define WR_CTRL         0x24
#define WR_UV           0x80

#define IRQB_ACK        0x04
#define IRQB_STA        0x0C
#define IRQB_EN         0x14
#define IRQB_MASK       0x1C
#define IRQB_VIC_FD     (1u << 16)

#define CSI_PHY_STATE   0x14

#define EV_FRAME_START  24
#define EV_FRAME_END    23
#define EV_DS1_END      50
#define EV_DS2_END      53

#define NUM_CH          3

static const uint8_t ch_done_event[NUM_CH] = { 14, 15, 1 };

typedef struct IngenicIspApicalWriter {
    uint32_t cur;               /* the bank being written */
    uint32_t last;              /* the last completed bank */
    bool first;                 /* no frame since the enable / restart */
    bool active;                /* took a bank this frame */
    uint32_t ctrl;              /* the last control written to +0x24 */
} IngenicIspApicalWriter;

struct IngenicIspApicalState {
    SysBusDevice parent_obj;

    MemoryRegion core_mr, vic_mr, irqb_mr, csi_mr;
    qemu_irq irq;
    QEMUTimer *start_timer, *end_timer;

    uint32_t fps;
    uint32_t vic_ctrl;          /* the VIC's control register */
    bool trace;

    uint32_t core[CORE_SIZE / 4];
    uint32_t vic[VIC_SIZE / 4];
    uint32_t irqb[IRQB_SIZE / 4];
    uint32_t csi[CSI_SIZE / 4];
    uint32_t irqb_latch;        /* the block's own status bits (16 up) */
    bool vic_running;
    uint32_t frame;
    IngenicIspApicalWriter wr[NUM_CH];
    IngenicIspPainter paint;
};

#define CORE(o)         s->core[(o) >> 2]

/* --- interrupts ------------------------------------------------------------ */

static uint32_t apical_irqb_status(IngenicIspApicalState *s)
{
    return (CORE(CORE_STA) & 0xFFFF) | s->irqb_latch;
}

static void apical_update(IngenicIspApicalState *s)
{
    uint32_t p = apical_irqb_status(s) & s->irqb[IRQB_EN >> 2] &
                 ~s->irqb[IRQB_MASK >> 2];

    qemu_set_irq(s->irq, !!p);
}

/* Event @ev: every slot programmed with it. */
static void apical_event(IngenicIspApicalState *s, unsigned ev)
{
    int slot;

    for (slot = 0; slot < 16; slot++) {
        uint32_t r = CORE(CORE_EVENT + 4 * (slot / 2));
        unsigned e = (slot & 1 ? r >> 16 : r) & 0x3F;

        if (ev && e == ev) {
            CORE(CORE_STA) |= 1u << slot;
        }
    }
}

/* --- the frame clock --------------------------------------------------------- */

static int64_t apical_period(IngenicIspApicalState *s)
{
    return NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
}

static void apical_schedule(IngenicIspApicalState *s)
{
    bool run = s->vic_running && s->irqb[IRQB_EN >> 2];

    if (run && !timer_pending(s->start_timer)) {
        timer_mod(s->start_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  apical_period(s));
    } else if (!run) {
        timer_del(s->start_timer);
    }
}

static void apical_frame_start(void *opaque)
{
    IngenicIspApicalState *s = opaque;
    int n;

    for (n = 0; n < NUM_CH; n++) {
        IngenicIspApicalWriter *w = &s->wr[n];
        uint32_t banks = (CORE(WR(n) + WR_BANKS) & 7) + 1;
        uint32_t next;

        w->active = false;
        if (!(CORE(WR(n) + WR_FMT) & WR_FMT_EN)) {
            continue;
        }
        if (w->first) {
            next = w->cur % banks;
        } else if (w->ctrl & 1) {
            continue;           /* the next bank is empty: drop */
        } else {
            next = (w->cur + 1) % banks;
        }
        if (!CORE(WR(n) + WR_BANK + 4 * next)) {
            continue;
        }
        w->cur = next;
        w->first = false;
        w->active = true;
    }
    apical_event(s, EV_FRAME_START);
    apical_update(s);
    timer_mod(s->end_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              apical_period(s) / 2);
    timer_mod(s->start_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              apical_period(s));
}

static void apical_frame_end(void *opaque)
{
    IngenicIspApicalState *s = opaque;
    int n;

    for (n = 0; n < NUM_CH; n++) {
        IngenicIspApicalWriter *w = &s->wr[n];
        uint32_t size = CORE(WR(n) + WR_SIZE);
        uint32_t wd = size & 0xFFFF, ht = size >> 16;
        uint32_t fmt = CORE(WR(n) + WR_UV + WR_FMT);
        hwaddr ya, uva;

        if (!w->active || !wd || !ht) {
            continue;
        }
        ya = CORE(WR(n) + WR_BANK + 4 * w->cur);
        uva = CORE(WR(n) + WR_UV + WR_BANK + 4 * w->cur);
        /* NV21 is format code 141 (0x8D) in the CbCr writer */
        ingenic_isp_paint(&s->paint, s->frame, wd, ht,
                          (int32_t)CORE(WR(n) + WR_STRIDE),
                          (int32_t)CORE(WR(n) + WR_UV + WR_STRIDE), ya, uva,
                          (fmt & 0xFF) == 0x8D, 256);
        w->last = w->cur;
        w->active = false;
        apical_event(s, ch_done_event[n]);
        if (s->trace) {
            qemu_log("ingenic-isp-apical: frame %u ch%d %ux%u bank %u -> "
                     "0x%08x/0x%08x\n", s->frame, n, wd, ht, w->cur,
                     (uint32_t)ya, (uint32_t)uva);
        }
    }
    apical_event(s, EV_FRAME_END);
    apical_event(s, EV_DS1_END);
    apical_event(s, EV_DS2_END);
    s->irqb_latch |= IRQB_VIC_FD;
    s->frame++;
    apical_update(s);
}

/* --- core ------------------------------------------------------------------ */

static int apical_writer(hwaddr off, hwaddr *r)
{
    if (off < WR(0) || off >= WR(NUM_CH)) {
        return -1;
    }
    *r = (off - WR(0)) & 0xFF;
    return (off - WR(0)) >> 8;
}

static uint64_t apical_core_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspApicalState *s = opaque;
    hwaddr r;
    int n = apical_writer(off, &r);

    if (n >= 0 && (r == WR_CTRL || r == WR_UV + WR_CTRL)) {
        return (s->wr[n].cur & 7) << 8 | (s->wr[n].last & 7) << 11 |
               (CORE(off) & 0xFF);
    }
    if (off == CORE_STATE_NOW) {
        return (CORE(off) & ~7u) | (CORE(CORE_STATE) & 7);
    }
    return s->core[off >> 2];
}

static void apical_core_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicIspApicalState *s = opaque;
    hwaddr r;
    int n = apical_writer(off, &r);

    if (s->trace && n >= 0) {
        qemu_log("ingenic-isp-apical: [0x%04x] = 0x%08x\n", (unsigned)off,
                 (uint32_t)val);
    }
    if (off == CORE_STA) {
        return;
    }
    if (off == CORE_CLR) {
        CORE(CORE_STA) &= ~(val & 0xFFFF);
        s->core[off >> 2] = val;
        apical_update(s);
        return;
    }
    if (n >= 0) {
        IngenicIspApicalWriter *w = &s->wr[n];

        if (r == WR_FMT && (val & WR_FMT_EN) &&
            !(CORE(off) & WR_FMT_EN)) {
            w->cur = w->last = 0;
            w->first = true;
            w->ctrl = 2;
        } else if (r == WR_BANKS && (val & WR_BANKS_RESTART)) {
            w->cur = 0;
            w->first = true;
        } else if (r == WR_CTRL) {
            w->ctrl = val;
        }
    }
    s->core[off >> 2] = val;
}

/* --- VIC, interrupt block, CSI ------------------------------------------------ */

static uint64_t apical_vic_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspApicalState *s = opaque;

    return s->vic[off >> 2];
}

static void apical_vic_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    IngenicIspApicalState *s = opaque;

    if (off == s->vic_ctrl) {
        /* 2: reset, done at once; 1: start; 8: stop; 0x10: error reset */
        if (val == 1) {
            s->vic_running = true;
        } else if (val == 2 || val == 8) {
            s->vic_running = false;
        }
        s->vic[off >> 2] = 0;
        apical_schedule(s);
        return;
    }
    s->vic[off >> 2] = val;
}

static uint64_t apical_irqb_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspApicalState *s = opaque;

    if (off == IRQB_STA) {
        return apical_irqb_status(s);
    }
    return s->irqb[off >> 2];
}

static void apical_irqb_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicIspApicalState *s = opaque;

    if (off == IRQB_STA) {
        return;
    }
    if (off == IRQB_ACK) {
        s->irqb_latch &= ~val;
    }
    s->irqb[off >> 2] = val;
    apical_update(s);
    if (off == IRQB_EN) {
        apical_schedule(s);
    }
}

static uint64_t apical_csi_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspApicalState *s = opaque;

    if (off == CSI_PHY_STATE) {
        return 0x630;
    }
    return s->csi[off >> 2];
}

static void apical_csi_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    IngenicIspApicalState *s = opaque;

    s->csi[off >> 2] = val;
}

#define APICAL_OPS(rd, wr) {                        \
    .read = rd,                                     \
    .write = wr,                                    \
    .endianness = DEVICE_LITTLE_ENDIAN,             \
    .impl.min_access_size = 4,                      \
    .impl.max_access_size = 4,                      \
    .valid.min_access_size = 1,                     \
    .valid.max_access_size = 4,                     \
}

static const MemoryRegionOps apical_core_ops =
    APICAL_OPS(apical_core_read, apical_core_write);
static const MemoryRegionOps apical_vic_ops =
    APICAL_OPS(apical_vic_read, apical_vic_write);
static const MemoryRegionOps apical_irqb_ops =
    APICAL_OPS(apical_irqb_read, apical_irqb_write);
static const MemoryRegionOps apical_csi_ops =
    APICAL_OPS(apical_csi_read, apical_csi_write);

/* --- device ------------------------------------------------------------------ */

static void apical_reset(DeviceState *dev)
{
    IngenicIspApicalState *s = INGENIC_ISP_APICAL(dev);

    timer_del(s->start_timer);
    timer_del(s->end_timer);
    memset(s->core, 0, sizeof(s->core));
    memset(s->vic, 0, sizeof(s->vic));
    memset(s->irqb, 0, sizeof(s->irqb));
    memset(s->csi, 0, sizeof(s->csi));
    memset(s->wr, 0, sizeof(s->wr));
    s->irqb_latch = 0;
    s->vic_running = false;
    s->frame = 0;
    apical_update(s);
}

static void apical_init(Object *obj)
{
    IngenicIspApicalState *s = INGENIC_ISP_APICAL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->core_mr, obj, &apical_core_ops, s,
                          "ingenic-isp-apical.core", CORE_SIZE);
    memory_region_init_io(&s->vic_mr, obj, &apical_vic_ops, s,
                          "ingenic-isp-apical.vic", VIC_SIZE);
    memory_region_init_io(&s->irqb_mr, obj, &apical_irqb_ops, s,
                          "ingenic-isp-apical.irq", IRQB_SIZE);
    memory_region_init_io(&s->csi_mr, obj, &apical_csi_ops, s,
                          "ingenic-isp-apical.csi", CSI_SIZE);
    sysbus_init_mmio(sbd, &s->core_mr);
    sysbus_init_mmio(sbd, &s->vic_mr);
    sysbus_init_mmio(sbd, &s->irqb_mr);
    sysbus_init_mmio(sbd, &s->csi_mr);
    sysbus_init_irq(sbd, &s->irq);
    s->start_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, apical_frame_start, s);
    s->end_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, apical_frame_end, s);
}

static const VMStateDescription vmstate_apical_writer = {
    .name = "ingenic-isp-apical-writer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cur, IngenicIspApicalWriter),
        VMSTATE_UINT32(last, IngenicIspApicalWriter),
        VMSTATE_BOOL(first, IngenicIspApicalWriter),
        VMSTATE_BOOL(active, IngenicIspApicalWriter),
        VMSTATE_UINT32(ctrl, IngenicIspApicalWriter),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_apical = {
    .name = TYPE_INGENIC_ISP_APICAL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(core, IngenicIspApicalState, CORE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(vic, IngenicIspApicalState, VIC_SIZE / 4),
        VMSTATE_UINT32_ARRAY(irqb, IngenicIspApicalState, IRQB_SIZE / 4),
        VMSTATE_UINT32_ARRAY(csi, IngenicIspApicalState, CSI_SIZE / 4),
        VMSTATE_UINT32(irqb_latch, IngenicIspApicalState),
        VMSTATE_BOOL(vic_running, IngenicIspApicalState),
        VMSTATE_UINT32(frame, IngenicIspApicalState),
        VMSTATE_STRUCT_ARRAY(wr, IngenicIspApicalState, NUM_CH, 1,
                             vmstate_apical_writer, IngenicIspApicalWriter),
        VMSTATE_TIMER_PTR(start_timer, IngenicIspApicalState),
        VMSTATE_TIMER_PTR(end_timer, IngenicIspApicalState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property apical_properties[] = {
    DEFINE_PROP_UINT32("fps", IngenicIspApicalState, fps, 25),
    DEFINE_PROP_UINT32("vic-ctrl", IngenicIspApicalState, vic_ctrl, 0),
    DEFINE_PROP_BOOL("trace", IngenicIspApicalState, trace, false),
};

static void apical_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, apical_reset);
    device_class_set_props(dc, apical_properties);
    dc->vmsd = &vmstate_apical;
}

static const TypeInfo apical_info = {
    .name          = TYPE_INGENIC_ISP_APICAL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIspApicalState),
    .instance_init = apical_init,
    .class_init    = apical_class_init,
};

static void apical_register_types(void)
{
    type_register_static(&apical_info);
}

type_init(apical_register_types)
