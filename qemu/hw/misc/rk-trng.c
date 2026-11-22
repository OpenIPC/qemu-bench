/*
 * Rockchip TRNG v1 ("rockchip,trngv1": RV1106, RK3588 ...), the random
 * number generator beside the crypto engine (RV1106: 0xFF448000).
 *
 * The register file, from the vendor kernel's
 * drivers/char/hw_random/rockchip-rng.c (develop-5.10):
 *
 *   0x00 CTRL    1 = generate (RAND), 2 = reseed (SEED)
 *   0x04 STAT    bit 9 SEEDED, bit 30 GENERATING, bit 31 RESEEDING
 *   0x08 MODE    bit 3: 256-bit output, else 128
 *   0x10 IE      bit 31 global, bit 1 seed done, bit 0 rand ready
 *   0x14 ISTAT   bit 1 seed done, bit 0 rand ready (write 1 to clear)
 *   0x20..0x3C   RAND0..7
 *   0x60 AUTO_RQSTS
 *   0xF0 VERSION 0x46bc; the driver refuses anything else, and on the
 *                failing path it spends ten seconds in hwrng retries
 *
 * The RV1103B's "rockchip,rkrng" is the same generator behind another
 * register file (property "rkrng"):
 *
 *   0x10 CTRL    hiword-masked: bit 3 SW_DRNG_REQ asks for 256 bits
 *   0x14 STATE   bit 3 SW_DRNG_ACK when they are ready (write 1 to clear)
 *   0x70..0x8C   DRNG_DATA_0..7
 *
 * The RV1126's "rockchip,cryptov2-rng", inside the crypto block at
 * +0x400 (property "crypto-v2"; a 0x80-byte window over the crypto
 * model):
 *
 *   0x00 CTL     hiword-masked: bit 0 START (clears when the sample is
 *                ready), bit 1 enable, bits 5:4 length (3 = 256 bits)
 *   0x04 SAMPLE_CNT   0x10..0x2c DOUT_0..7
 *
 * Output comes from the host's entropy, so the guest's crng gets seeded
 * ("random: crng init done") instead of warning about urandom reads.
 * Generation is instant: the driver waits 10 us before it looks anyway.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/guest-random.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_RK_TRNG "rk-trng"
OBJECT_DECLARE_SIMPLE_TYPE(RkTrngState, RK_TRNG)

#define R_CTRL          0x00
#define R_STAT          0x04
#define R_MODE          0x08
#define R_IE            0x10
#define R_ISTAT         0x14
#define R_RAND0         0x20
#define R_RAND7         0x3c
#define R_AUTO_RQSTS    0x60
#define R_VERSION       0xf0
#define REGS_SIZE       0x200

#define CTRL_RAND       1
#define CTRL_SEED       2
#define STAT_SEEDED     (1u << 9)
#define MODE_256        (1u << 3)
#define IE_GLBL         (1u << 31)
#define IST_RAND_RDY    (1u << 0)
#define IST_SEED_DONE   (1u << 1)
#define VERSION_CODE    0x46bc

struct RkTrngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t mode;
    uint32_t ie;
    uint32_t istat;
    uint32_t auto_rqsts;
    uint32_t rand[8];
    bool rkrng;             /* property: the RV1103B's register file */
    bool crypto_v2;         /* property: the RV1126's, in the crypto block */
    uint32_t rk_ctrl, rk_state;
};

#define RK_CTRL         0x10
#define RK_STATE        0x14
#define RK_DATA0        0x70
#define RK_DATA7        0x8c
#define RK_SW_DRNG      (1u << 3)

static uint64_t rk_v2_read(RkTrngState *s, hwaddr off)
{
    if (off == 0x00) {
        return s->rk_ctrl;
    }
    if (off == 0x04) {
        return s->rk_state;
    }
    if (off >= 0x10 && off < 0x30) {
        return s->rand[(off - 0x10) / 4];
    }
    return 0;
}

static void rk_v2_write(RkTrngState *s, hwaddr off, uint32_t val)
{
    uint32_t mask = val >> 16;

    if (off == 0x00) {
        s->rk_ctrl = (s->rk_ctrl & ~mask) | (val & mask);
        if (s->rk_ctrl & 1) {
            qemu_guest_getrandom_nofail(s->rand, sizeof(s->rand));
            s->rk_ctrl &= ~1u;          /* START clears: sample ready */
        }
    } else if (off == 0x04) {
        s->rk_state = val;              /* SAMPLE_CNT */
    }
}

static uint64_t rk_rkrng_read(RkTrngState *s, hwaddr off)
{
    switch (off) {
    case RK_CTRL:
        return s->rk_ctrl;
    case RK_STATE:
        return s->rk_state;
    case RK_DATA0 ... RK_DATA7:
        return s->rand[(off - RK_DATA0) / 4];
    }
    return 0;
}

static void rk_rkrng_write(RkTrngState *s, hwaddr off, uint32_t val)
{
    switch (off) {
    case RK_CTRL: {
        uint32_t mask = val >> 16;
        uint32_t req = val & mask & ~s->rk_ctrl;

        s->rk_ctrl = (s->rk_ctrl & ~mask) | (val & mask);
        if (req & RK_SW_DRNG) {
            qemu_guest_getrandom_nofail(s->rand, sizeof(s->rand));
            s->rk_state |= RK_SW_DRNG;
        }
        if (req & 0x17) {                       /* the other requests */
            s->rk_state |= req & 0x17;
        }
        break;
    }
    case RK_STATE:
        s->rk_state &= ~val;
        break;
    }
}

static void rk_trng_update_irq(RkTrngState *s)
{
    qemu_set_irq(s->irq, (s->ie & IE_GLBL) && (s->ie & s->istat & 3));
}

static uint64_t rk_trng_read(void *opaque, hwaddr off, unsigned size)
{
    RkTrngState *s = opaque;

    if (s->rkrng) {
        return rk_rkrng_read(s, off);
    }
    if (s->crypto_v2) {
        return rk_v2_read(s, off);
    }
    switch (off) {
    case R_CTRL:
        return 0;
    case R_STAT:
        return STAT_SEEDED;
    case R_MODE:
        return s->mode;
    case R_IE:
        return s->ie;
    case R_ISTAT:
        return s->istat;
    case R_RAND0 ... R_RAND7:
        return s->rand[(off - R_RAND0) / 4];
    case R_AUTO_RQSTS:
        return s->auto_rqsts;
    case R_VERSION:
        return VERSION_CODE;
    }
    return 0;
}

static void rk_trng_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    RkTrngState *s = opaque;

    if (s->rkrng) {
        rk_rkrng_write(s, off, val);
        return;
    }
    if (s->crypto_v2) {
        rk_v2_write(s, off, val);
        return;
    }
    switch (off) {
    case R_CTRL:
        if (val & CTRL_RAND) {
            int n = (s->mode & MODE_256) ? 8 : 4;

            memset(s->rand, 0, sizeof(s->rand));
            qemu_guest_getrandom_nofail(s->rand, n * 4);
            s->istat |= IST_RAND_RDY;
        }
        if (val & CTRL_SEED) {
            s->istat |= IST_SEED_DONE;
        }
        break;
    case R_MODE:
        s->mode = val;
        break;
    case R_IE:
        s->ie = val;
        break;
    case R_ISTAT:
        s->istat &= ~val;
        break;
    case R_AUTO_RQSTS:
        s->auto_rqsts = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "rk-trng: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        return;
    }
    rk_trng_update_irq(s);
}

static const MemoryRegionOps rk_trng_ops = {
    .read = rk_trng_read,
    .write = rk_trng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rk_trng_reset(DeviceState *dev)
{
    RkTrngState *s = RK_TRNG(dev);

    s->mode = s->ie = s->istat = s->auto_rqsts = 0;
    s->rk_ctrl = s->rk_state = 0;
    memset(s->rand, 0, sizeof(s->rand));
}

static void rk_trng_realize(DeviceState *dev, Error **errp)
{
    RkTrngState *s = RK_TRNG(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &rk_trng_ops, s,
                          TYPE_RK_TRNG, s->crypto_v2 ? 0x80 : REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void rk_trng_init(Object *obj)
{
    RkTrngState *s = RK_TRNG(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription rk_trng_vmstate = {
    .name = TYPE_RK_TRNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mode, RkTrngState),
        VMSTATE_UINT32(ie, RkTrngState),
        VMSTATE_UINT32(istat, RkTrngState),
        VMSTATE_UINT32(auto_rqsts, RkTrngState),
        VMSTATE_UINT32_ARRAY(rand, RkTrngState, 8),
        VMSTATE_UINT32(rk_ctrl, RkTrngState),
        VMSTATE_UINT32(rk_state, RkTrngState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property rk_trng_properties[] = {
    DEFINE_PROP_BOOL("rkrng", RkTrngState, rkrng, false),
    DEFINE_PROP_BOOL("crypto-v2", RkTrngState, crypto_v2, false),
};

static void rk_trng_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, rk_trng_properties);
    dc->realize = rk_trng_realize;
    device_class_set_legacy_reset(dc, rk_trng_reset);
    dc->vmsd = &rk_trng_vmstate;
}

static const TypeInfo rk_trng_info = {
    .name = TYPE_RK_TRNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RkTrngState),
    .instance_init = rk_trng_init,
    .class_init = rk_trng_class_init,
};

static void rk_trng_register_types(void)
{
    type_register_static(&rk_trng_info);
}

type_init(rk_trng_register_types)
