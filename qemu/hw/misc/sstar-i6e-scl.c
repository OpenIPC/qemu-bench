/*
 * SigmaStar infinity6e scaler output: the frames SCL writes to memory.
 *
 * mhal.ko programs the output's write DMA through the CMDQ
 * (HalSclHwSetDmaBufConfig), in the RIU's storage, so the registers are read
 * from there each frame.  Port 0, the one the encoder is bound to:
 *
 *   bank 0x1213 reg 0x08/0x09  the Y plane, four slots two registers apart
 *   bank 0x1213 reg 0x10/0x11  the C plane
 *   bank 0x1215 reg 0x54       the Y stride in bytes
 *
 * Addresses are MIU >> 3.  What is modelled is the frame reaching whoever
 * takes it, on the "frame" input (sstar-isp's clock): the ISP's picture
 * (sstar_iford_isp_picture()) as NV12 at the output's size by nearest
 * neighbour.  No crop, no filters.  The stride is the width, and the planes'
 * distance gives the height, to the buffer's alignment.
 *
 * Bank 0x1218 holds the interrupts (mhal.ko HalSclIrq*), a 48-bit group: mask
 * at regs 0x08-0x0A (1 masked), clear at 0x10-0x12, flags at 0x13-0x15.  Each
 * frame raises whatever is unmasked: the driver takes the frame's end from
 * these for a whole buffer (DrvSclIrqIsrHandler).  INT_IRQ_SC_TOP.
 *
 * Bound to the encoder the port is a hardware ring: slot 0's buffers hold a
 * few lines (the MIU MMU maps nothing past them, 138 lines of a 1280x720
 * port) that the scaler fills as the encoder drains them, so no frame is ever
 * in memory.  A ring therefore goes to the encoder directly
 * (sstar_i6e_scl_ring()) and memory is written only for a whole buffer.
 * That goes for the JPEG channel too, which waybeam binds to the same port:
 * the JPE is given the ring's address and reads the same frame.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/misc/sstar-i6e-scl.h"
#include "hw/misc/sstar-iford-isp.h"
#include "hw/misc/sstar-riu.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_SSTAR_I6E_SCL "sstar-i6e-scl"
OBJECT_DECLARE_SIMPLE_TYPE(SstarI6eSclState, SSTAR_I6E_SCL)

#define BANK_DMA            0x1213
#define BANK_STRIDE         0x1215
#define R_Y_LO              0x08
#define R_C_LO              0x10
#define R_Y_STRIDE          0x54

#define IRQ_REGS            0x80
#define R_IRQ_MASK          0x08
#define R_IRQ_CLEAR         0x10
#define R_IRQ_FLAG          0x13
#define IRQ_WORDS           3

#define SCL_DRAM_BASE       0x20000000
#define SCL_PAGE            4096
#define SCL_MAX_DIM         4096
#define SCL_MMU_PAGE        (128 * KiB)
#define SCL_RING_PAGES      256

struct SstarI6eSclState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint16_t regs[IRQ_REGS];
    Object *isp;
    SstarRiuState *riu;
    uint32_t frames;
    bool ring;
    bool trace;
    int row_w;
    uint8_t *row;
};

static SstarI6eSclState *scl_state;

static void scl_put(uint64_t miu, const uint8_t *p, size_t len)
{
    while (len) {
        size_t n = MIN(len, SCL_PAGE - (miu & (SCL_PAGE - 1)));

        if (!sstar_miu_missed(miu)) {
            dma_memory_write(&address_space_memory,
                             SCL_DRAM_BASE + sstar_miu_xlate(miu), p, n,
                             MEMTXATTRS_UNSPECIFIED);
        }
        miu += n;
        p += n;
        len -= n;
    }
}

static uint64_t scl_addr(SstarI6eSclState *s, int reg)
{
    return ((uint64_t)sstar_riu_get(s->riu, BANK_DMA, reg + 1) << 16 |
            sstar_riu_get(s->riu, BANK_DMA, reg)) << 3;
}

/* The ISP's picture scaled to @w x @h: into @dy and @duv, or to memory */
static void scl_draw(SstarI6eSclState *s, int w, int h, uint8_t *dy,
                     uint8_t *duv, uint64_t ya, uint64_t ca)
{
    const uint8_t *py, *puv;
    int pw, ph, x, y;

    if (s->row_w != w) {
        g_free(s->row);
        s->row = g_malloc(w);
        s->row_w = w;
    }
    sstar_iford_isp_picture(s->isp, &py, &puv, &pw, &ph);
    for (y = 0; y < h; y++) {
        const uint8_t *in = py + (int64_t)y * ph / h * pw;
        uint8_t *row = dy ? dy + (size_t)y * w : s->row;

        for (x = 0; x < w; x++) {
            row[x] = in[(int64_t)x * pw / w];
        }
        if (!dy) {
            scl_put(ya + y * w, row, w);
        }
    }
    for (y = 0; y < h / 2; y++) {
        const uint8_t *in = puv + (int64_t)y * (ph / 2) / (h / 2) * pw;
        uint8_t *row = duv ? duv + (size_t)y * w : s->row;

        for (x = 0; x + 1 < w; x += 2) {
            int px = MIN((int64_t)x * pw / w, pw - 2) & ~1;

            row[x] = in[px];
            row[x + 1] = in[px + 1];
        }
        if (!duv) {
            scl_put(ca + y * w, row, w);
        }
    }
}

bool sstar_i6e_scl_ring(uint64_t ya, int w, int h, uint8_t *y, uint8_t *uv)
{
    SstarI6eSclState *s = scl_state;

    /*
     * Not s->ring: the table can look whole between two frames with the
     * reader still on the ring, and for a real buffer this is the same frame.
     */
    if (!s || ya != scl_addr(s, R_Y_LO)) {
        return false;
    }
    scl_draw(s, w, h, y, uv, 0, 0);
    return true;
}

static void scl_update_irq(SstarI6eSclState *s)
{
    bool level = false;
    int i;

    for (i = 0; i < IRQ_WORDS; i++) {
        level |= !!(s->regs[R_IRQ_FLAG + i] & ~s->regs[R_IRQ_MASK + i]);
    }
    qemu_set_irq(s->irq, level);
}

/*
 * A ring: part of the buffer has no page behind it, or wraps onto a page it
 * already used.  No register says so (0x1218 0x65 stays 3, its "off", and
 * bank 0x1215 0x53 stays 0 on this path): only the MIU MMU's table does.
 */
static bool scl_is_ring(uint64_t ya, uint64_t end)
{
    uint64_t seen[SCL_RING_PAGES];
    uint64_t a;
    int n = 0, i;

    for (a = ya & ~(SCL_MMU_PAGE - 1); a < end && n < SCL_RING_PAGES;
         a += SCL_MMU_PAGE) {
        uint64_t pa = sstar_miu_xlate(a);

        if (sstar_miu_missed(a)) {
            return true;
        }
        for (i = 0; i < n; i++) {
            if (seen[i] == pa) {
                return true;
            }
        }
        seen[n++] = pa;
    }
    return false;
}

static void scl_frame(SstarI6eSclState *s)
{
    uint64_t ya = scl_addr(s, R_Y_LO), ca = scl_addr(s, R_C_LO);
    int w = sstar_riu_get(s->riu, BANK_STRIDE, R_Y_STRIDE);
    int h, i;

    if (!ya || !ca || ca <= ya || !w || w > SCL_MAX_DIM) {
        s->ring = false;
    } else {
        h = MIN((ca - ya) / w, SCL_MAX_DIM);
        sstar_iford_isp_frame(s->isp);
        s->ring = scl_is_ring(ya, ca + (uint64_t)(h / 2) * w);
        if (!s->ring) {
            scl_draw(s, w, h, NULL, NULL, ya, ca);
        }
        if (s->trace) {
            qemu_log("sstar-i6e-scl: frame %u %dx%d at 0x%" PRIx64 " 0x%" PRIx64
                     "%s\n", s->frames, w, h, ya, ca,
                     s->ring ? " ring" : "");
        }
    }
    /*
     * The scaler's interrupts follow its input, buffer or no buffer: infinity6
     * programs its first buffer only after one (mi_vpe's _MI_VPE_Isr is what
     * wakes every stage).
     */
    s->frames++;
    for (i = 0; i < IRQ_WORDS; i++) {
        s->regs[R_IRQ_FLAG + i] |= ~s->regs[R_IRQ_MASK + i];
    }
    scl_update_irq(s);
}

static uint64_t scl_read(void *opaque, hwaddr off, unsigned size)
{
    SstarI6eSclState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[(off >> 2) & (IRQ_REGS - 1)];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void scl_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarI6eSclState *s = opaque;
    int r = (off >> 2) & (IRQ_REGS - 1);
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
    s->regs[r] = (s->regs[r] & ~mask) | (v & mask);
    if (r >= R_IRQ_CLEAR && r < R_IRQ_CLEAR + IRQ_WORDS) {
        s->regs[R_IRQ_FLAG + r - R_IRQ_CLEAR] &= ~(v & mask);
    }
    scl_update_irq(s);
}

static const MemoryRegionOps scl_ops = {
    .read = scl_read,
    .write = scl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void scl_frame_in(void *opaque, int n, int level)
{
    if (level) {
        scl_frame(opaque);
    }
}

static void scl_init(Object *obj)
{
    SstarI6eSclState *s = SSTAR_I6E_SCL(obj);

    qdev_init_gpio_in(DEVICE(obj), scl_frame_in, 1);
    memory_region_init_io(&s->iomem, obj, &scl_ops, s, TYPE_SSTAR_I6E_SCL,
                          IRQ_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    memset(s->regs + R_IRQ_MASK, 0xFF, IRQ_WORDS * sizeof(s->regs[0]));
    scl_state = s;
}

static const Property scl_props[] = {
    DEFINE_PROP_LINK("isp", SstarI6eSclState, isp, TYPE_OBJECT, Object *),
    DEFINE_PROP_LINK("riu", SstarI6eSclState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
    DEFINE_PROP_BOOL("trace", SstarI6eSclState, trace, false),
};

static void scl_reset(DeviceState *dev)
{
    SstarI6eSclState *s = SSTAR_I6E_SCL(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs + R_IRQ_MASK, 0xFF, IRQ_WORDS * sizeof(s->regs[0]));
    s->ring = false;
    s->frames = 0;
    qemu_set_irq(s->irq, 0);
}

static const VMStateDescription vmstate_scl = {
    .name = TYPE_SSTAR_I6E_SCL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarI6eSclState, IRQ_REGS),
        VMSTATE_UINT32(frames, SstarI6eSclState),
        VMSTATE_BOOL(ring, SstarI6eSclState),
        VMSTATE_END_OF_LIST()
    }
};

static void scl_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, scl_reset);
    device_class_set_props(dc, scl_props);
    dc->vmsd = &vmstate_scl;
}

static const TypeInfo scl_info = {
    .name = TYPE_SSTAR_I6E_SCL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarI6eSclState),
    .instance_init = scl_init,
    .class_init = scl_class_init,
};

static void scl_register(void)
{
    type_register_static(&scl_info);
}

type_init(scl_register)
