/*
 * Rockchip RV1126 / RV1109 CIF ("rockchip,rv1126-cif", 0xFFAE0000), the
 * capture front end the camera's MIPI CSI-2 host feeds, as far as its
 * MIPI / LVDS channels go (the rkcif "rkcif_mipi_lvds" device).
 *
 * The SDK's pipeline captures RAW frames here into DDR; its 3A library
 * then queues each to the ISP's raw read DMA ("read back"), and the ISP
 * and ISPP turn it into pictures.  What reaches the pictures is the ISP /
 * ISPP models' scene, so no raw data is written: the capture is the frame
 * timing and its interrupts, which is what the drivers and the 3A
 * library run on.
 *
 *   0x080 + 8n  IDn_CTRL0     bit 0 capture enable (channel / virtual
 *                             channel n, 0..3)
 *   0x0a4 ...   FRMx_ADDR_*   ping-pong buffer addresses: storage
 *   0x124 INTEN               0x128 INTSTAT (write 1 to clear):
 *     bit 2n / 2n+1   FRAME0 / FRAME1 start of channel n
 *     bit 8+2n / 9+2n FRAME0 / FRAME1 end (DMA done) of channel n
 *   0x134 LINE_CNT_ID0_1      lines of the last frame of channels 0 / 1
 *   0x14c + 4n FRAME_NUM_VCn  frame number, counted per frame start
 *
 * Frames alternate between the FRAME0 and FRAME1 buffers, starting with
 * FRAME0 when capture is enabled.  A channel whose last frame end is
 * still pending skips the next frame.  They come at the sensor's frame rate
 * ("sensor" link: its frame period, or "fps") while it streams; the end
 * is half a frame after the start.
 *
 * Everything else is storage.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/image-sensor.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_CIF "rk-cif"
OBJECT_DECLARE_SIMPLE_TYPE(RkCifState, RK_CIF)

#define REGS_SIZE       0x800
#define R_ID_CTRL0(n)   (0x080 + 8 * (n))
#define R_ID_CTRL1(n)   (0x084 + 8 * (n))
#define R_INTEN         0x124
#define R_INTSTAT       0x128
#define R_LINE_CNT01    0x134
#define R_LINE_CNT23    0x138
#define R_FRAME_NUM(n)  (0x14c + 4 * (n))
#define NCH             4

#define CAPTURE_EN      (1u << 0)
#define FS(n, f)        (1u << (2 * (n) + (f)))
#define FE(n, f)        (1u << (8 + 2 * (n) + (f)))

struct RkCifState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[REGS_SIZE / 4];
    QEMUTimer *timer;
    bool in_frame;              /* between a frame's start and its end */
    uint8_t phase[NCH];         /* the buffer (0 / 1) the next frame fills */
    uint8_t active;             /* channels the current frame started on */

    uint32_t fps;
    Object *sensor;
};

#define REG(s, off) ((s)->regs[(off) / 4])

static void rk_cif_update(RkCifState *s)
{
    qemu_set_irq(s->irq, !!(REG(s, R_INTSTAT) & REG(s, R_INTEN)));
}

static int64_t rk_cif_half_frame(RkCifState *s)
{
    int64_t ns = image_sensor_frame_ns(s->sensor);

    if (ns <= 0) {
        ns = NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
    }
    return ns / 2;
}

static uint8_t rk_cif_enabled(RkCifState *s)
{
    uint8_t m = 0;
    int n;

    for (n = 0; n < NCH; n++) {
        if (REG(s, R_ID_CTRL0(n)) & CAPTURE_EN) {
            m |= 1u << n;
        }
    }
    return m;
}

static void rk_cif_tick(void *opaque)
{
    RkCifState *s = opaque;
    int n;

    if (!s->in_frame) {
        s->active = 0;
        if (!s->sensor || image_sensor_streaming(s->sensor)) {
            s->active = rk_cif_enabled(s);
        }
        for (n = 0; n < NCH; n++) {
            /*
             * The channel's last frame end isn't handled yet: this frame
             * is dropped, as when the DMA has nowhere to go.  Both
             * buffers' ends pending at once -- the guest a whole frame
             * late, which a slow emulation is -- made the rkcif driver's
             * rkcif_assign_new_buffer_pingpong dereference NULL (2 boots
             * in 4, four RV1109s under gdb with -d unimp).
             */
            if (REG(s, R_INTSTAT) & (FE(n, 0) | FE(n, 1))) {
                s->active &= ~(1u << n);
            }
        }
        for (n = 0; n < NCH; n++) {
            if (s->active & (1u << n)) {
                REG(s, R_INTSTAT) |= FS(n, s->phase[n]);
                REG(s, R_FRAME_NUM(n)) += 1;
            }
        }
        s->in_frame = true;
    } else {
        for (n = 0; n < NCH; n++) {
            if (s->active & (1u << n)) {
                uint32_t h = (REG(s, R_ID_CTRL1(n)) >> 16) & 0x3fff;
                hwaddr lc = n < 2 ? R_LINE_CNT01 : R_LINE_CNT23;
                int sh = (n & 1) * 16;

                REG(s, lc) = (REG(s, lc) & ~(0x3fffu << sh)) | h << sh;
                REG(s, R_INTSTAT) |= FE(n, s->phase[n]);
                s->phase[n] ^= 1;
            }
        }
        s->in_frame = false;
    }
    rk_cif_update(s);
    if (rk_cif_enabled(s) || s->in_frame) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  rk_cif_half_frame(s));
    }
}

static uint64_t rk_cif_read(void *opaque, hwaddr off, unsigned size)
{
    RkCifState *s = opaque;

    return s->regs[off / 4];
}

static void rk_cif_write(void *opaque, hwaddr off, uint64_t val,
                         unsigned size)
{
    RkCifState *s = opaque;
    int n;

    if (off == R_INTSTAT) {
        REG(s, R_INTSTAT) &= ~val;
        rk_cif_update(s);
        return;
    }
    for (n = 0; n < NCH; n++) {
        if (off == R_ID_CTRL0(n) && (val & CAPTURE_EN) &&
            !(REG(s, off) & CAPTURE_EN)) {
            s->phase[n] = 0;
        }
    }
    s->regs[off / 4] = val;
    if (off == R_INTEN) {
        rk_cif_update(s);
    }
    if (rk_cif_enabled(s) && !timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  rk_cif_half_frame(s));
    }
}

static const MemoryRegionOps rk_cif_ops = {
    .read = rk_cif_read,
    .write = rk_cif_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_cif_reset(DeviceState *dev)
{
    RkCifState *s = RK_CIF(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->phase, 0, sizeof(s->phase));
    s->in_frame = false;
    s->active = 0;
    timer_del(s->timer);
    rk_cif_update(s);
}

static void rk_cif_init(Object *obj)
{
    RkCifState *s = RK_CIF(obj);

    memory_region_init_io(&s->iomem, obj, &rk_cif_ops, s, TYPE_RK_CIF,
                          REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rk_cif_tick, s);
}

static const Property rk_cif_properties[] = {
    DEFINE_PROP_UINT32("fps", RkCifState, fps, 25),
    DEFINE_PROP_LINK("sensor", RkCifState, sensor, TYPE_DEVICE, Object *),
};

static const VMStateDescription rk_cif_vmstate = {
    .name = TYPE_RK_CIF,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, RkCifState, REGS_SIZE / 4),
        VMSTATE_TIMER_PTR(timer, RkCifState),
        VMSTATE_BOOL(in_frame, RkCifState),
        VMSTATE_UINT8_ARRAY(phase, RkCifState, NCH),
        VMSTATE_UINT8(active, RkCifState),
        VMSTATE_END_OF_LIST()
    }
};

static void rk_cif_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, rk_cif_reset);
    device_class_set_props(dc, rk_cif_properties);
    dc->vmsd = &rk_cif_vmstate;
}

static const TypeInfo rk_cif_info = {
    .name = TYPE_RK_CIF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkCifState),
    .instance_init = rk_cif_init,
    .class_init = rk_cif_class_init,
};

static void rk_cif_register_types(void)
{
    type_register_static(&rk_cif_info);
}

type_init(rk_cif_register_types)
