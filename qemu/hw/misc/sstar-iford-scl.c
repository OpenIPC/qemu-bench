/*
 * SigmaStar Iford scaler output: the frames SCL writes to memory.
 *
 * mi_scl.ko programs each output's write DMA from the CMDQ.  From its
 * _HAL_SCL_CHIP_SetBufCfg and the commands of a running pipeline, bank
 * 0x1253 is port 0, plane by plane, 0x20 registers apart (Y at 0, C at
 * 0x20).  Ports 1 to 3 are the next banks, 0x1254 to 0x1256, laid out the
 * same: OpenIPC's majestic runs its second stream from port 2 (bank 0x1255,
 * 1280x720 into four buffers in turn, which mi_scl hands to the encoder in
 * frame mode):
 *
 *   0x00 bit 0 enable, bit 4 start of a frame, bit 15 a hardware ring
 *        (set on the port bound to the encoder as a ring, clear on frame
 *        ports: both firmwares' commands, and /proc/mi_modules/mi_scl's
 *        "Ring" and "Frame" on a camera)
 *   0x02 the width in 16-pixel units   0x03 the height in rows
 *   0x05 the pitch in bytes
 *   0x06/0x07 the buffer, a MIU address in 16-byte units, which the MIU
 *   MMU may translate (0x08/0x09 repeat it: the second buffer slot)
 *
 * What is modelled is the frame reaching memory: on the Y plane's start
 * the ISP's last picture (sstar_iford_isp_picture()) is written to those
 * addresses as NV12, where the encoder takes it, at the output's size by
 * nearest neighbour.  On infinity6c ("ring", which the machine sets for that
 * die) a ring port writes nothing: its buffer holds a few lines, never a
 * frame, and its encoders (sstar-wave5, and the JPE through i6_irq) take the
 * picture from here (sstar_iford_scl_ring()); a whole frame written there ran
 * past the ring and cost more than any other step of a frame (23 ms of
 * 2592x1944).  Iford's encoder, sstar-venc, never asks for it, so there every
 * port's frame is written as before.  The
 * scaler's own registers beyond these are not read: no crop, no filters.
 * infinity6c's mi_scl starts an output before it has given it a buffer: a
 * plane whose address is still 0 is not written [assumed: MIU 0 is the
 * kernel's, so the hardware cannot be writing a frame there].
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/sstar-i6e-scl.h"
#include "hw/misc/sstar-iford-isp.h"
#include "hw/misc/sstar-riu.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"

#define TYPE_SSTAR_IFORD_SCL "sstar-iford-scl"
OBJECT_DECLARE_SIMPLE_TYPE(SstarIfordSclState, SSTAR_IFORD_SCL)

#define SCL_REGS            0x80        /* one port: a bank */
#define SCL_PORTS           4
#define PLANE_C             0x20
#define R_CTRL              0x00
#define R_WIDTH16           0x02
#define R_HEIGHT            0x03
#define R_PITCH             0x05
#define R_ADDR_LO           0x06
#define R_ADDR_HI           0x07
#define CTRL_START          (1u << 4)
#define CTRL_RING           (1u << 15)

struct SstarIfordSclState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Object *isp;
    uint16_t regs[SCL_PORTS * SCL_REGS];
    uint32_t frame;
    bool trace;
    bool ring;          /* infinity6c: ring ports feed the encoder directly */
    int w;
    uint8_t *row;
};

static SstarIfordSclState *scl_state;

static void scl_write_mem(uint64_t miu, const uint8_t *p, size_t len)
{
    sstar_miu_rw(miu, (void *)p, len, true);
}

static void scl_frame(SstarIfordSclState *s, int port)
{
    const uint16_t *pr = s->regs + port * SCL_REGS;
    int w = pr[R_WIDTH16] * 16, h = pr[R_HEIGHT];
    uint32_t pitch = pr[R_PITCH];
    uint64_t ya = ((uint64_t)pr[R_ADDR_HI] << 16 | pr[R_ADDR_LO]) << 4;
    uint64_t ca = ((uint64_t)pr[PLANE_C + R_ADDR_HI] << 16 |
                   pr[PLANE_C + R_ADDR_LO]) << 4;
    const uint8_t *py, *puv;
    int pw, ph, x, y;

    if (!w || !h || w > 4096 || h > 4096 || pitch < w ||
        (s->ring && (pr[R_CTRL] & CTRL_RING))) {
        return;
    }
    if (s->w != w) {
        g_free(s->row);
        s->row = g_malloc(w);
        s->w = w;
    }
    sstar_iford_isp_picture(s->isp, &py, &puv, &pw, &ph);
    for (y = 0; ya && y < h; y++) {
        const uint8_t *in = py + (int64_t)y * ph / h * pw;

        if (w == pw) {
            scl_write_mem(ya + y * pitch, in, w);
            continue;
        }
        for (x = 0; x < w; x++) {
            s->row[x] = in[(int64_t)x * pw / w];
        }
        scl_write_mem(ya + y * pitch, s->row, w);
    }
    for (y = 0; ca && y < h / 2; y++) {
        const uint8_t *in = puv + (int64_t)y * (ph / 2) / (h / 2) * pw;

        if (w == pw) {
            scl_write_mem(ca + y * pitch, in, w);
            continue;
        }
        for (x = 0; x < w; x += 2) {
            int px = MIN((int64_t)x * pw / w, pw - 2) & ~1;

            s->row[x] = in[px];
            s->row[x + 1] = in[px + 1];
        }
        scl_write_mem(ca + y * pitch, s->row, w);
    }
    if (s->trace) {
        qemu_log("sstar-iford-scl: port %d frame %u %dx%d from %dx%d\n", port,
                 s->frame, w, h, pw, ph);
    }
    s->frame++;
}

/*
 * infinity6c binds an output to the encoder as a ring: lines at a pitch of
 * the sensor's width, fewer of them than the frame has, which the encoder
 * drains as they come.  The picture goes to it from here, as NV12 at the
 * encoder's size.  Both firmwares seen bind port 0 so; with "ring", any port
 * with the ring bit is taken too, since scl_frame() writes no frame for it.
 */
static bool scl_ring_port(SstarIfordSclState *s, uint64_t ya)
{
    int port;

    for (port = 0; port < SCL_PORTS; port++) {
        const uint16_t *pr = s->regs + port * SCL_REGS;

        /* port 0 as before; ports 1 to 3 when bound as a ring */
        if ((port == 0 || (s->ring && (pr[R_CTRL] & CTRL_RING))) &&
            ya == ((uint64_t)pr[R_ADDR_HI] << 16 | pr[R_ADDR_LO]) << 4) {
            return true;
        }
    }
    return false;
}

bool sstar_iford_scl_ring(uint64_t ya, int w, int h, uint8_t *dy, uint8_t *duv)
{
    SstarIfordSclState *s = scl_state;
    const uint8_t *py, *puv;
    int pw, ph, x, y;

    if (!s || !scl_ring_port(s, ya)) {
        return false;
    }
    sstar_iford_isp_picture(s->isp, &py, &puv, &pw, &ph);
    for (y = 0; y < h; y++) {
        const uint8_t *in = py + (int64_t)y * ph / h * pw;

        for (x = 0; x < w; x++) {
            dy[(size_t)y * w + x] = in[(int64_t)x * pw / w];
        }
    }
    for (y = 0; y < h / 2; y++) {
        const uint8_t *in = puv + (int64_t)y * (ph / 2) / (h / 2) * pw;

        for (x = 0; x + 1 < w; x += 2) {
            int px = MIN((int64_t)x * pw / w, pw - 2) & ~1;

            duv[(size_t)y * w + x] = in[px];
            duv[(size_t)y * w + x + 1] = in[px + 1];
        }
    }
    return true;
}

static uint64_t scl_read(void *opaque, hwaddr off, unsigned size)
{
    SstarIfordSclState *s = opaque;
    int r = (off >> 2) % (SCL_PORTS * SCL_REGS);
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = s->regs[r];
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void scl_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarIfordSclState *s = opaque;
    int r = (off >> 2) % (SCL_PORTS * SCL_REGS);
    uint16_t v = s->regs[r];

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = (off & 1) ? (v & 0xFF) | ((val & 0xFF) << 8)
                      : (v & 0xFF00) | (val & 0xFF);
    } else {
        v = val;
    }
    /*
     * The start bit triggers one frame and does not stay set: mi_scl writes
     * the control register again with masked read-modify-writes (port 2's
     * disable, mask 0x8001, while its buffers are being freed), and a start
     * bit left standing would write a frame to a buffer it no longer owns.
     */
    s->regs[r] = v & (r % SCL_REGS == R_CTRL ? ~CTRL_START : 0xFFFF);
    if (r % SCL_REGS == R_CTRL && (v & CTRL_START)) {
        scl_frame(s, r / SCL_REGS);
    }
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

static void scl_init(Object *obj)
{
    SstarIfordSclState *s = SSTAR_IFORD_SCL(obj);

    memory_region_init_io(&s->iomem, obj, &scl_ops, s, TYPE_SSTAR_IFORD_SCL,
                          SCL_PORTS * SCL_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    scl_state = s;
}

static const Property scl_props[] = {
    DEFINE_PROP_LINK("isp", SstarIfordSclState, isp, TYPE_OBJECT, Object *),
    DEFINE_PROP_BOOL("trace", SstarIfordSclState, trace, false),
    DEFINE_PROP_BOOL("ring", SstarIfordSclState, ring, false),
};

static void scl_class_init(ObjectClass *oc, const void *data)
{
    device_class_set_props(DEVICE_CLASS(oc), scl_props);
}

static const TypeInfo scl_info = {
    .name = TYPE_SSTAR_IFORD_SCL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarIfordSclState),
    .instance_init = scl_init,
    .class_init = scl_class_init,
};

static void scl_register(void)
{
    type_register_static(&scl_info);
}

type_init(scl_register)
