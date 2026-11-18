/*
 * Ingenic DRAWBOX ("ingenic,t41-drawbox", 0x130D0000, core INTC source
 * 42 on T40 and T41): draws rectangles, corner marks and lines into an
 * NV12 (or BGRA) picture in place.  libimp's OSD draws its RECT, LINE
 * and corner regions with it (OSD_Draw_Layer_Cover_Pic_RectLine ->
 * dbox_osd -> /dev/dbox IOCTL_DBOX_START); the kernel driver is
 * drivers/video/ingenic_ipu/ingenic_drawbox.c (the same on both parts).
 *
 *   0x00 CTRL   bit 0 START (clears when done), 1 RESET (clears at once,
 *               empties the box RAM), 2 COLOR_MASK (draw luma only),
 *               3 IRQ_MASK, 4 done (the handler clears it by writing 0),
 *               5 clock gate, 6 BGRA picture, 7/8 timeout status/mask;
 *               [16:9] the box RAM write pointer, [24:17] read pointer
 *   0x04 YB     the luma (or BGRA) base   0x08 CB  the CbCr base
 *   0x0C STRIDE Y stride [15:0] | CbCr stride [31:16], in pixels
 *   0x10 IMG_WH width [15:0] | height [31:16]
 *   0x14/18/1C COLOR_Y/U/V   one byte per palette entry 0..3 (the driver
 *               loads red, black, green, yellow)
 *   0x20 RAM    each box is two writes, low word then high word:
 *               x [11:0], y [23:12], w [35:24], h [47:36], colour [49:48],
 *               mode [51:50] (0 rectangle, 1 corner marks, 2 horizontal
 *               line, 3 vertical line), line width [54:52] (pixels),
 *               line length [63:55] (the corner arms)
 *   0x24 TIMEOUT storage
 *
 * The boxes are drawn when START is set and the done bit rises a virtual
 * 50 us later, interrupting unless IRQ_MASK.  A line's length is its box
 * width (horizontal) or height (vertical), or the line length field when
 * that is 0; the corner marks are L shapes of line length at each corner
 * -- inferred from the driver's fields, as no documentation was found.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "qom/object.h"

#define TYPE_INGENIC_DRAWBOX "ingenic-drawbox"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDrawboxState, INGENIC_DRAWBOX)

#define R_CTRL          0x00
#define R_YB            0x04
#define R_CB            0x08
#define R_STRIDE        0x0C
#define R_IMG_WH        0x10
#define R_COLOR_Y       0x14
#define R_COLOR_U       0x18
#define R_COLOR_V       0x1C
#define R_RAM           0x20
#define R_TIMEOUT       0x24
#define NREGS           (0x28 / 4)

#define CTRL_START      (1u << 0)
#define CTRL_RESET      (1u << 1)
#define CTRL_COLOR_MASK (1u << 2)
#define CTRL_IRQ_MASK   (1u << 3)
#define CTRL_DONE       (1u << 4)
#define CTRL_BGRA       (1u << 6)
#define CTRL_WP_SHIFT   9
#define CTRL_RP_SHIFT   17
#define CTRL_PTRS       (0xffffu << CTRL_WP_SHIFT)

#define MAX_BOXES       128
#define DONE_NS         (50 * SCALE_US)

enum { MODE_RECT, MODE_CORNERS, MODE_HLINE, MODE_VLINE };

struct IngenicDrawboxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[NREGS];
    uint32_t ram[MAX_BOXES * 2];
    uint32_t nwords;
};

static void dbox_update(IngenicDrawboxState *s)
{
    uint32_t c = s->regs[R_CTRL / 4];

    qemu_set_irq(s->irq, (c & CTRL_DONE) && !(c & CTRL_IRQ_MASK));
}

static void dbox_set_ptrs(IngenicDrawboxState *s, unsigned wp, unsigned rp)
{
    uint32_t *c = &s->regs[R_CTRL / 4];

    *c = (*c & ~CTRL_PTRS) | ((wp & 0xff) << CTRL_WP_SHIFT) |
         ((rp & 0xff) << CTRL_RP_SHIFT);
}

/* One pixel of palette entry col at (x, y), clipped to the picture */
static void dbox_pixel(IngenicDrawboxState *s, int x, int y, int col)
{
    uint32_t wh = s->regs[R_IMG_WH / 4], st = s->regs[R_STRIDE / 4];
    int w = wh & 0xffff, h = wh >> 16;
    uint8_t py = s->regs[R_COLOR_Y / 4] >> (8 * col);
    uint8_t pu = s->regs[R_COLOR_U / 4] >> (8 * col);
    uint8_t pv = s->regs[R_COLOR_V / 4] >> (8 * col);
    hwaddr a;

    if (x < 0 || y < 0 || x >= w || y >= h) {
        return;
    }
    if (s->regs[R_CTRL / 4] & CTRL_BGRA) {
        /* BT.601 limited range back to RGB */
        int c = py - 16, d = pu - 128, e = pv - 128;
        uint8_t px[4] = {
            MIN(MAX((298 * c + 516 * d + 128) >> 8, 0), 255),
            MIN(MAX((298 * c - 100 * d - 208 * e + 128) >> 8, 0), 255),
            MIN(MAX((298 * c + 409 * e + 128) >> 8, 0), 255),
            0xff,
        };

        a = s->regs[R_YB / 4] + ((hwaddr)y * (st & 0xffff) + x) * 4;
        dma_memory_write(&address_space_memory, a, px, 4,
                         MEMTXATTRS_UNSPECIFIED);
        return;
    }
    a = s->regs[R_YB / 4] + (hwaddr)y * (st & 0xffff) + x;
    dma_memory_write(&address_space_memory, a, &py, 1, MEMTXATTRS_UNSPECIFIED);
    if (!(s->regs[R_CTRL / 4] & CTRL_COLOR_MASK) && !(x & 1) && !(y & 1)) {
        uint8_t uv[2] = { pu, pv };

        a = s->regs[R_CB / 4] + (hwaddr)(y / 2) * (st >> 16) + x;
        dma_memory_write(&address_space_memory, a, uv, 2,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

static void dbox_fill(IngenicDrawboxState *s, int x, int y, int w, int h,
                      int col)
{
    int i, j;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            dbox_pixel(s, x + i, y + j, col);
        }
    }
}

static void dbox_draw(IngenicDrawboxState *s, uint32_t lo, uint32_t hi)
{
    uint64_t e = (uint64_t)hi << 32 | lo;
    int x = extract64(e, 0, 12), y = extract64(e, 12, 12);
    int w = extract64(e, 24, 12), h = extract64(e, 36, 12);
    int col = extract64(e, 48, 2), mode = extract64(e, 50, 2);
    int lw = MAX(extract64(e, 52, 3), 1), ll = extract64(e, 55, 9);

    switch (mode) {
    case MODE_RECT:
        dbox_fill(s, x, y, w, lw, col);
        dbox_fill(s, x, y + h - lw, w, lw, col);
        dbox_fill(s, x, y, lw, h, col);
        dbox_fill(s, x + w - lw, y, lw, h, col);
        break;
    case MODE_CORNERS:
        ll = MIN(ll, MIN(w, h) / 2);
        dbox_fill(s, x, y, ll, lw, col);
        dbox_fill(s, x, y, lw, ll, col);
        dbox_fill(s, x + w - ll, y, ll, lw, col);
        dbox_fill(s, x + w - lw, y, lw, ll, col);
        dbox_fill(s, x, y + h - lw, ll, lw, col);
        dbox_fill(s, x, y + h - ll, lw, ll, col);
        dbox_fill(s, x + w - ll, y + h - lw, ll, lw, col);
        dbox_fill(s, x + w - lw, y + h - ll, lw, ll, col);
        break;
    case MODE_HLINE:
        dbox_fill(s, x, y, w ? w : ll, lw, col);
        break;
    case MODE_VLINE:
        dbox_fill(s, x, y, lw, h ? h : ll, col);
        break;
    }
}

static void dbox_done(void *opaque)
{
    IngenicDrawboxState *s = opaque;
    uint32_t *c = &s->regs[R_CTRL / 4];

    *c = (*c & ~CTRL_START) | CTRL_DONE;
    dbox_update(s);
}

static void dbox_start(IngenicDrawboxState *s)
{
    unsigned n;

    for (n = 0; n + 1 < s->nwords; n += 2) {
        dbox_draw(s, s->ram[n], s->ram[n + 1]);
    }
    dbox_set_ptrs(s, s->nwords / 2, s->nwords / 2);
    s->regs[R_CTRL / 4] &= ~CTRL_DONE;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DONE_NS);
}

static uint64_t ingenic_dbox_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDrawboxState *s = opaque;

    return off < NREGS * 4 && off != R_RAM ? s->regs[off / 4] : 0;
}

static void ingenic_dbox_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicDrawboxState *s = opaque;
    uint32_t old;

    switch (off) {
    case R_CTRL:
        old = s->regs[R_CTRL / 4];
        /* the pointers are read only; the done bit is cleared, not set */
        s->regs[R_CTRL / 4] = (val & ~(CTRL_PTRS | CTRL_DONE)) |
                              (old & CTRL_PTRS) | (old & val & CTRL_DONE);
        if (val & CTRL_RESET) {
            timer_del(s->timer);
            s->nwords = 0;
            s->regs[R_CTRL / 4] &= ~(CTRL_RESET | CTRL_START | CTRL_DONE);
            dbox_set_ptrs(s, 0, 0);
        } else if ((val & CTRL_START) && !(old & CTRL_START)) {
            dbox_start(s);
        }
        break;
    case R_RAM:
        if (s->nwords < ARRAY_SIZE(s->ram)) {
            s->ram[s->nwords++] = val;
            dbox_set_ptrs(s, s->nwords / 2,
                          extract32(s->regs[R_CTRL / 4], CTRL_RP_SHIFT, 8));
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: box RAM full\n");
        }
        break;
    default:
        if (off < NREGS * 4) {
            s->regs[off / 4] = val;
        }
        break;
    }
    dbox_update(s);
}

static const MemoryRegionOps ingenic_dbox_ops = {
    .read = ingenic_dbox_read,
    .write = ingenic_dbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_dbox_reset(DeviceState *dev)
{
    IngenicDrawboxState *s = INGENIC_DRAWBOX(dev);

    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->nwords = 0;
    dbox_update(s);
}

static void ingenic_dbox_init(Object *obj)
{
    IngenicDrawboxState *s = INGENIC_DRAWBOX(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_dbox_ops, s,
                          TYPE_INGENIC_DRAWBOX, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dbox_done, s);
}

static const VMStateDescription vmstate_ingenic_dbox = {
    .name = TYPE_INGENIC_DRAWBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicDrawboxState, NREGS),
        VMSTATE_UINT32_ARRAY(ram, IngenicDrawboxState, MAX_BOXES * 2),
        VMSTATE_UINT32(nwords, IngenicDrawboxState),
        VMSTATE_TIMER_PTR(timer, IngenicDrawboxState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_dbox_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_dbox_reset);
    dc->vmsd = &vmstate_ingenic_dbox;
}

static const TypeInfo ingenic_dbox_info = {
    .name = TYPE_INGENIC_DRAWBOX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDrawboxState),
    .instance_init = ingenic_dbox_init,
    .class_init = ingenic_dbox_class_init,
};

static void ingenic_dbox_register_types(void)
{
    type_register_static(&ingenic_dbox_info);
}

type_init(ingenic_dbox_register_types)
