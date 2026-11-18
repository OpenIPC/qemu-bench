/*
 * SigmaStar infinity6 MHE: the H.265 encoder (mhal.ko, DTS "mhe-0" to "mhe-2"
 * at 0x1F264800, 0x1F264A00, 0x1F264C00, "mhe-irq", INT_IRQ_MHE).
 *
 * No vendor source; this comes from mhal.ko (_EncFire, _IsrFunc, _EncDone,
 * _OpsCheckStatus) and traces of the running driver.  16-bit registers on the
 * RIU's 4-byte stride, banks 0x1324-0x1326 here, and the "ven" banks it shares
 * with the MFE, which the RIU's storage holds:
 *
 *   0x1324 0x1C  [7:0] mask, [15:8] force; 0x1D [7:0] clear; 0x1E [7:0] the
 *                status past the mask, bit 0 frame done (as the MFE)
 *   0x1324 0x48  the slice QP
 *   0x1324 0x49  the header packer: a word, MSB first; 0x4A [14:5] the number
 *                of words - 1, [4:0] the last one's bits; 0x4B bit 3 takes it
 *   0x1325 0x73/0x74  the bits written
 *   0x1326 0x65/0x66  the record, MIU bytes; 0x67 bit 0 on
 *   0x1329  0x10 bit 0 start, 0x13/0x14 the last column and row, 0x15 pitch,
 *           0x20-0x23 the source Y and C
 *   0x132B  0x03-0x06 the output buffer's start and last 16-byte unit
 *
 * The driver feeds VPS, SPS, PPS and the slice header through the packer, and
 * the hardware appends the slice data: hevc_enc_slice_data() here, with what
 * the driver's parameter sets choose (CTB 32, TBs 4-32 at transform depth 2,
 * no SAO, no scaling lists, cu_qp_delta on at the CTB).  The slice type is the
 * header's NAL type.
 *
 * It finishes a frame from a 216-byte record: +8/+0xA the bit count, +0x10
 * bit 0 done, +0xD0-+0xD6 where the reconstruction ended.  As with the MFE, CMDQ1 carries the job,
 * waits and then forces frame done, and the CMDQ model does not hold on a
 * wait, so the frame is encoded inline at the last packer word.  The
 * reconstruction stays here, per stream, not in the driver's rings.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/misc/sstar-riu.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hevc-enc.h"

#define TYPE_SSTAR_MHE "sstar-mhe"
OBJECT_DECLARE_SIMPLE_TYPE(SstarMheState, SSTAR_MHE)

#define MHE_REGS            0x180       /* three banks */
#define BANK1(r)            (0x80 + (r))
#define BANK2(r)            (0x100 + (r))

#define R_IRQ_MASK          0x1C
#define R_IRQ_CLR           0x1D
#define R_IRQ_STATUS        0x1E
#define R_QP                0x48
#define R_FDC_WORD          0x49
#define R_FDC_LEN           0x4A
#define R_FDC_CTRL          0x4B
#define R_BITS_LO           BANK1(0x73)
#define R_BITS_HI           BANK1(0x74)
#define R_REC_LO            BANK2(0x65)
#define R_REC_HI            BANK2(0x66)
#define R_REC_EN            BANK2(0x67)

#define FDC_PUSH            (1u << 3)
#define IRQ_DONE            (1u << 0)

#define BANK_SRC            0x1329
#define BANK_OUT            0x132B
#define SRC_CTRL            0x10
#define SRC_XEND            0x13
#define SRC_YEND            0x14
#define SRC_PITCH           0x15
#define SRC_Y               0x20
#define SRC_C               0x22
#define REC_Y               0x58
#define REC_C               0x5A
#define OUT_START           0x03
#define OUT_END             0x05

#define DRAM_BASE           0x20000000
#define HDR_MAX             1024
#define REC_SIZE            216
#define MAX_DIM             4096
#define STREAMS             4

struct SstarMheState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint16_t regs[MHE_REGS];
    uint8_t raw;
    uint32_t hdr_words;
    uint32_t hdr_len;
    uint8_t hdr[HDR_MAX];
    uint32_t frames;
    uint64_t dram_size;
    SstarRiuState *riu;
    bool trace;
    struct {
        uint32_t key;
        HevcEncRef *ref;
    } streams[STREAMS];
    int stream_next;
};

static const HevcEncParams mhe_params = {
    .log2_ctb = 5,
    .trafo_depth_intra = 2,
    .trafo_depth_inter = 2,
};

static uint32_t mhe_reg32(SstarMheState *s, uint32_t bank, int reg)
{
    return sstar_riu_get(s->riu, bank, reg) |
           (uint32_t)sstar_riu_get(s->riu, bank, reg + 1) << 16;
}

static uint32_t mhe_rec_addr(SstarMheState *s)
{
    return s->regs[R_REC_LO] | (uint32_t)s->regs[R_REC_HI] << 16;
}

static bool mhe_in_dram(SstarMheState *s, uint32_t miu, uint64_t len)
{
    return miu + len <= s->dram_size;
}

static void mhe_update(SstarMheState *s)
{
    qemu_set_irq(s->irq, !!(s->raw & ~s->regs[R_IRQ_MASK] & 0xFF));
}

static HevcEncRef *mhe_stream_ref(SstarMheState *s, uint32_t key)
{
    int i;

    for (i = 0; i < STREAMS; i++) {
        if (s->streams[i].ref && s->streams[i].key == key) {
            return s->streams[i].ref;
        }
    }
    i = s->stream_next;
    s->stream_next = (i + 1) % STREAMS;
    if (s->streams[i].ref) {
        hevc_enc_ref_free(s->streams[i].ref);
    }
    s->streams[i].key = key;
    s->streams[i].ref = hevc_enc_ref_new();
    return s->streams[i].ref;
}

/* The NAL type of the last unit the packer carried: the slice's */
static int mhe_nal_type(SstarMheState *s)
{
    int i;

    for (i = s->hdr_len - 4; i >= 0; i--) {
        if (!s->hdr[i] && !s->hdr[i + 1] && s->hdr[i + 2] == 1) {
            return (s->hdr[i + 3] >> 1) & 0x3F;
        }
    }
    return -1;
}

static void mhe_record(SstarMheState *s, uint32_t bits, uint32_t ylen)
{
    uint32_t miu = mhe_rec_addr(s);
    uint32_t yend = (mhe_reg32(s, BANK_SRC, REC_Y) + ylen) >> 1;
    uint32_t cend = (mhe_reg32(s, BANK_SRC, REC_C) + ylen / 2) >> 1;
    uint16_t rec[REC_SIZE / 2] = { 0 };

    if (!(s->regs[R_REC_EN] & 1) || !mhe_in_dram(s, miu, sizeof(rec))) {
        return;
    }
    rec[0x08 / 2] = cpu_to_le16(bits & 0xFFFF);
    rec[0x0A / 2] = cpu_to_le16(bits >> 16);
    rec[0x10 / 2] = cpu_to_le16(IRQ_DONE);
    rec[0xD0 / 2] = cpu_to_le16(yend & 0xFFF0);
    rec[0xD2 / 2] = cpu_to_le16(yend >> 16);
    rec[0xD4 / 2] = cpu_to_le16(cend & 0xFFF0);
    rec[0xD6 / 2] = cpu_to_le16(cend >> 16);
    address_space_write(&address_space_memory, DRAM_BASE + miu,
                        MEMTXATTRS_UNSPECIFIED, rec, sizeof(rec));
}

static void mhe_encode(SstarMheState *s)
{
    int w = sstar_riu_get(s->riu, BANK_SRC, SRC_XEND) + 1;
    int h = sstar_riu_get(s->riu, BANK_SRC, SRC_YEND) + 1;
    int pitch = sstar_riu_get(s->riu, BANK_SRC, SRC_PITCH);
    uint32_t ya = mhe_reg32(s, BANK_SRC, SRC_Y);
    uint32_t ca = mhe_reg32(s, BANK_SRC, SRC_C);
    uint32_t o0 = mhe_reg32(s, BANK_OUT, OUT_START);
    uint32_t o1 = mhe_reg32(s, BANK_OUT, OUT_END) | 15;
    int nal = mhe_nal_type(s);
    bool p_slice = nal >= 0 && nal < 16;
    int qp = s->regs[R_QP] & 0x3F;
    size_t cap = o1 >= o0 ? (size_t)(o1 - o0) + 1 : 0;
    size_t len = 0;
    HevcEncRef *ref;

    if (w < 8 || h < 8 || w > MAX_DIM || h > MAX_DIM || (w & 7) || (h & 7) ||
        pitch < w || !mhe_in_dram(s, ya, (uint64_t)pitch * h) ||
        !mhe_in_dram(s, ca, (uint64_t)pitch * h / 2) ||
        !mhe_in_dram(s, o0, cap) || cap < s->hdr_len) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-mhe: picture %dx%d pitch %d, "
                      "source 0x%x 0x%x, output 0x%x-0x%x\n", w, h, pitch, ya,
                      ca, o0, o1);
    } else {
        g_autofree uint8_t *y = g_malloc((size_t)pitch * h);
        g_autofree uint8_t *uv = g_malloc((size_t)pitch * (h / 2));
        g_autofree uint8_t *out = g_malloc(cap);

        address_space_read(&address_space_memory, DRAM_BASE + ya,
                           MEMTXATTRS_UNSPECIFIED, y, (size_t)pitch * h);
        address_space_read(&address_space_memory, DRAM_BASE + ca,
                           MEMTXATTRS_UNSPECIFIED, uv, (size_t)pitch * (h / 2));
        memcpy(out, s->hdr, s->hdr_len);
        ref = mhe_stream_ref(s, mhe_rec_addr(s));
        len = hevc_enc_slice_data(&mhe_params, w, h, qp, p_slice,
                                  p_slice ? ref : NULL, ref, y, w, h, pitch,
                                  uv, pitch, out + s->hdr_len,
                                  cap - s->hdr_len);
        len = MIN(len, cap - s->hdr_len) + s->hdr_len;
        address_space_write(&address_space_memory, DRAM_BASE + o0,
                            MEMTXATTRS_UNSPECIFIED, out, len);
    }
    s->regs[R_BITS_LO] = (len * 8) & 0xFFFF;
    s->regs[R_BITS_HI] = (len * 8) >> 16;
    mhe_record(s, len * 8, w * h);
    if (s->trace) {
        qemu_log("sstar-mhe: %c %dx%d nal %d qp %d src 0x%x hdr %u, %zu bytes "
                 "at 0x%x\n", p_slice ? 'P' : 'I', w, h, nal, qp, ya,
                 s->hdr_len, len, o0);
    }
    s->frames++;
    s->hdr_len = s->hdr_words = 0;
    s->raw |= IRQ_DONE;
    mhe_update(s);
}

static void mhe_fdc_push(SstarMheState *s)
{
    uint32_t count = (s->regs[R_FDC_LEN] >> 5) & 0x3FF;
    uint32_t bits = s->hdr_words < count ? 16 : s->regs[R_FDC_LEN] & 0x1F;
    uint16_t word = s->regs[R_FDC_WORD];

    if (s->hdr_len + 2 <= HDR_MAX) {
        s->hdr[s->hdr_len++] = word >> 8;
        if (bits > 8) {
            s->hdr[s->hdr_len++] = word;
        }
    }
    if (++s->hdr_words != count + 1) {
        return;
    }
    if (sstar_riu_get(s->riu, BANK_SRC, SRC_CTRL) & 1) {
        mhe_encode(s);
    } else {
        s->hdr_len = s->hdr_words = 0;
    }
}

static uint64_t mhe_read(void *opaque, hwaddr off, unsigned size)
{
    SstarMheState *s = opaque;
    int reg = (off >> 2) % MHE_REGS;
    uint16_t v = s->regs[reg];

    if ((off & 3) >= 2) {
        return 0;
    }
    if (reg == R_IRQ_STATUS) {
        v = (s->raw & ~s->regs[R_IRQ_MASK] & 0xFF) | s->raw << 8;
    }
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void mhe_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarMheState *s = opaque;
    int reg = (off >> 2) % MHE_REGS;
    uint16_t v = val;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        v = off & 1 ? (s->regs[reg] & 0x00FF) | ((val & 0xFF) << 8)
                    : (s->regs[reg] & 0xFF00) | (val & 0xFF);
    }
    if (s->trace) {
        qemu_log("sstar-mhe: W %03x = %04x\n", reg, v);
    }
    switch (reg) {
    case R_IRQ_MASK:
        s->raw |= v >> 8;
        v &= 0xFF;
        break;
    case R_IRQ_CLR:
        s->raw &= ~v;
        v = 0;
        break;
    case R_IRQ_STATUS:
    case R_BITS_LO:
    case R_BITS_HI:
        return;
    case R_FDC_CTRL:
        s->regs[reg] = v;
        if (v & FDC_PUSH) {
            mhe_fdc_push(s);
        }
        break;
    }
    s->regs[reg] = v;
    mhe_update(s);
}

static const MemoryRegionOps mhe_ops = {
    .read = mhe_read,
    .write = mhe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void sstar_mhe_reset(DeviceState *dev)
{
    SstarMheState *s = SSTAR_MHE(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_IRQ_MASK] = 0xFF;
    s->raw = 0;
    s->hdr_len = s->hdr_words = 0;
    for (i = 0; i < STREAMS; i++) {
        if (s->streams[i].ref) {
            hevc_enc_ref_free(s->streams[i].ref);
            s->streams[i].ref = NULL;
        }
    }
    s->stream_next = 0;
    qemu_set_irq(s->irq, 0);
}

static void sstar_mhe_init(Object *obj)
{
    SstarMheState *s = SSTAR_MHE(obj);

    memory_region_init_io(&s->iomem, obj, &mhe_ops, s, TYPE_SSTAR_MHE,
                          MHE_REGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_sstar_mhe = {
    .name = TYPE_SSTAR_MHE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarMheState, MHE_REGS),
        VMSTATE_UINT8(raw, SstarMheState),
        VMSTATE_UINT32(hdr_words, SstarMheState),
        VMSTATE_UINT32(hdr_len, SstarMheState),
        VMSTATE_UINT8_ARRAY(hdr, SstarMheState, HDR_MAX),
        VMSTATE_UINT32(frames, SstarMheState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sstar_mhe_props[] = {
    DEFINE_PROP_UINT64("dram-size", SstarMheState, dram_size, 0),
    DEFINE_PROP_LINK("riu", SstarMheState, riu, TYPE_SSTAR_RIU,
                     SstarRiuState *),
    DEFINE_PROP_BOOL("trace", SstarMheState, trace, false),
};

static void sstar_mhe_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, sstar_mhe_reset);
    device_class_set_props(dc, sstar_mhe_props);
    dc->vmsd = &vmstate_sstar_mhe;
}

static const TypeInfo sstar_mhe_info = {
    .name = TYPE_SSTAR_MHE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarMheState),
    .instance_init = sstar_mhe_init,
    .class_init = sstar_mhe_class_init,
};

static void sstar_mhe_register_types(void)
{
    type_register_static(&sstar_mhe_info);
}

type_init(sstar_mhe_register_types)
