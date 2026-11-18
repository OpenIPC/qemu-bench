/*
 * Novatek DDR controller, PHY and arbiter (NA51089 / NA51055 at
 * 0xF0000000, 64 KiB: controller 0x0000, PHY 0x1000, arbiter 0x8000).
 *
 * The loader's closed DDR bring-up (Reset_560.a configDDR.o) programs the
 * controller and the PHY, starts the PHY's calibrations and trainings and
 * polls each for completion; U-Boot and Linux leave the block alone except
 * for the arbiter.  DRAM needs no training here, so the block is storage
 * whose status bits read as done -- the table below, each entry what the
 * loader waits for, found by running the vendor loaders:
 *
 *   0x004             the controller's command register: bits 0..3 start
 *                     a command and clear themselves, bits 8..10 their
 *                     done (initialisation, mode-register set, ...)
 *   0x020 bits 0..1   DDR3 training run (clears itself)
 *   PHY 0x2DC bit 7   the calibration started by PHY 0x2C0 bit 7
 *
 * NS02501 ("ns02501"): its PHY registers are at PHY + 4n, unpaged; the
 * loader's Delayline_K waits for registers 0x105 and 0x106 [4:0] to read
 * non-zero (then for 0x107 and 0x108 [4:0] to be zero, as stored), and
 * after starting a run at 0xCF for 0xBB bit 5 (done), then checks 0x6E
 * bit 5 (passed).
 *
 * Everything else reads back what was written.
 *
 * NA51084's PHY (the DDR3-2133 one, "phy-pages"), and NA51068's at
 * 0xFC400000 + 0x8000 ("phy-offset"): its registers are paged.  PHY
 * 0x3F8 selects the page, and register n of that page is at
 * PHY + 4n (n < 0xFE); the loader writes and checks per-page values
 * (its "checkphy"), so each page has its own storage.  Its done flags:
 * page 0 register 0x3D bits 6 / 7 (the calibrations "checkphy" waits
 * for), page 0 register 0x43 bit 6 (a training step; bits 7 and 5 its
 * errors), page 2 register 0x4A bit 5 (the training started at 0x240).
 * Page 2 register 0x8E is NA51068's DDR4 write Vref: the loader writes
 * the value with bit 7 set (load) and reads [6:0] back, so bit 7 reads 0
 * ("vref-strobe"; NA51084's loader uses that bit otherwise).
 *
 * NA51103: two controllers at 0xF00E0000 / 0xF00F0000, NA51068's paged PHY
 * at +0x8000, and QSEN's results below ("qsen-match").  Its DRAM arbiters
 * at 0xF00A0000 / 0xF0110000 / 0xF0510000 have a built-in tester (the
 * loader's "hvyload" check, "tester"): the start bits at +0x805C clear
 * themselves, and the status at +0x80A0 reads bit 0, passed (bit 1 is
 * failed).
 *
 * NA51090 ("na51090"): its controllers at 0xF06E0000 / 0xF06F0000 with the
 * same PHY, and its loader's own per-lane sweep (psdk), burst read-back
 * and QSEN gate (below); the testers are its arbiters' at 0xF0080000 /
 * 0xF00A0000.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_NVT_DDRC "nvt-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(NvtDdrcState, NVT_DDRC)

#define DDRC_SIZE       0x10000
#define DDRC_PHY        0x1000
#define DDRC_PHY_PAGE   0x13F8
#define DDRC_PHY_REGS   0xFE
#define DDRC_PAGES      16
#define DDRC_TEST_START  0x805C     /* the tester's start bits */
#define DDRC_TEST_STATUS 0x80A0     /* bit 0 passed, bit 1 failed */
#define DDRC_TEST_DONE   0x1

/* the paged PHY's done flags: page, register, bits */
static const struct {
    uint8_t page, reg;
    uint32_t done;
} nvt_ddrc_phy_done[] = {
    { 0, 0x3D, 0xC0 },
    { 0, 0x43, 0x40 },          /* training step done (bits 7 / 5: errors) */
    { 2, 0x4A, 0x20 },
};

/* the paged PHY's bits that clear themselves: page, register, bits */
static const struct {
    uint8_t page, reg;
    uint32_t start;
} nvt_ddrc_phy_cmd[] = {
    { 2, 0x8E, 0x80 },          /* DDR4 write Vref: bit 7 loads [6:0] */
};

struct NvtDdrcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[DDRC_SIZE / 4];
    uint32_t phy[DDRC_PAGES][DDRC_PHY_REGS];
    bool phy_pages;
    bool ns02501;
    bool vref_strobe;       /* NA51068: page 2 0x8E bit 7 clears itself */
    bool qsen_match;        /* NA51103: QSEN's compare reads as matched */
    bool tester;            /* NA51103: the arbiter's DRAM tester */
    bool na51090;           /* NA51090: the PHY's per-lane sweep (psdk) */
    uint8_t pattern;        /* NA51090: the last command 11's data byte */
    uint32_t phy_off;       /* the PHY within the block: 0x1000, 0x8000 */
};

/* NS02501's PHY (registers at PHY + 4n, not paged): its done flags */
static const struct {
    hwaddr offset;
    uint32_t done;
} nvt_ddrc_ns02501_done[] = {
    { DDRC_PHY + 0x105 * 4, 0x01 },     /* Delayline_K: calibration done */
    { DDRC_PHY + 0x106 * 4, 0x01 },     /* and its delay lines locked */
    { DDRC_PHY + 0x0BB * 4, 0x20 },     /* the run started at 0xCF: done */
    { DDRC_PHY + 0x06E * 4, 0x20 },     /* and passed */
};

/*
 * NA51103 ("qsen-match"): QSEN, the read-gate training, per byte lane l.
 * The lane's gate delay is a coarse step (page 2 register 0xBB + l / 2,
 * nibble l % 2) and a fine one (page 0 register 0xD2 + l / 2, the same
 * nibble), one position in the loader's order, where the fine step wraps
 * at 13 and the coarse one then moves.  After each step the loader reads
 * the PHY's compare of the DDR3 MPR pattern -- page 3 registers
 * 0x70 + 8l / 0x72 + 8l the mismatched bits, 0x71 + 8l / 0x73 + 8l the
 * matched ones -- and the gate's sampled level, page 0 register 0xB6 bit
 * 2l.  It looks for the window where the pattern matches, then inside it
 * for the level's 1-to-0 edge, and fails without one.  So: matched across
 * NVT_QSEN_LO..NVT_QSEN_HI, the level 1 below NVT_QSEN_EDGE.
 */
#define NVT_QSEN_LO     0x48
#define NVT_QSEN_EDGE   0x80
#define NVT_QSEN_HI     0xB8

static uint32_t nvt_ddrc_qsen_pos(NvtDdrcState *s, int lane)
{
    uint32_t coarse = s->phy[2][0xBB + lane / 2] >> (lane % 2 * 4);
    uint32_t fine = s->phy[0][0xD2 + lane / 2] >> (lane % 2 * 4);

    return (coarse & 0xF) * 16 + ((fine - 13) & 0xF);
}

/*
 * NA51090's QSEN.  The coarse / fine delays are NA51103's, position
 * coarse * 16 + ((fine - 13) & 15) (fine is two halves of eight steps,
 * 13..4 and 5..12, as the loader's fine stepper has it).  The loader
 * reads the lane's eight compare bytes (page 3 0x70 + 8 lane ..) as one
 * word: the burst read back, byte n 0xFF where bit 7 - n of the pattern
 * it last wrote with PHY command 11 is set (page 0 0x2C [3:0] 5, 0x2B
 * bit 7, the data in 0x29, started by 0x2C bit 7), 0 elsewhere -- its
 * first pattern is 0xAA, and it inverts it between passes.  The gate
 * level, page 0 0xB5 bit 4 + lane, is 1 below NA51090_QSEN_EDGE: the
 * loader finds the coarse step where it falls.  Bit 0 of the lane's
 * field in page 0 0xB6 ([2 lane + 1 : 2 lane]) then says which way to
 * step the fine delay: clear, down, until it reads set with the level
 * still 0 -- set below NA51090_QSEN_GATE.
 */
#define NA51090_QSEN_EDGE   0x14
#define NA51090_QSEN_GATE   0x16

static void nvt_ddrc_qsen_read_51090(NvtDdrcState *s, uint32_t *phy,
                                     uint32_t *v)
{
    int lane;

    if (phy >= &s->phy[3][0x70] && phy < &s->phy[3][0x90]) {
        int n = (phy - &s->phy[3][0x70]) % 8;

        *v = s->pattern >> (7 - n) & 1 ? 0xFF : 0;
        return;
    }
    if (phy == &s->phy[0][0xB6]) {
        for (lane = 0; lane < 4; lane++) {
            if (nvt_ddrc_qsen_pos(s, lane) < NA51090_QSEN_GATE) {
                *v |= 1u << (2 * lane);
            } else {
                *v &= ~(1u << (2 * lane));
            }
        }
        return;
    }
    if (phy != &s->phy[0][0xB5]) {
        return;
    }
    for (lane = 0; lane < 4; lane++) {
        if (nvt_ddrc_qsen_pos(s, lane) < NA51090_QSEN_EDGE) {
            *v |= 1u << (4 + lane);
        } else {
            *v &= ~(1u << (4 + lane));
        }
    }
}

/* QSEN's compare and level bits in the PHY register *phy reads, into *v */
static void nvt_ddrc_qsen_read(NvtDdrcState *s, uint32_t *phy, uint32_t *v)
{
    int lane;

    if (s->na51090) {
        nvt_ddrc_qsen_read_51090(s, phy, v);
        return;
    }

    for (lane = 0; lane < 4; lane++) {
        uint32_t pos = nvt_ddrc_qsen_pos(s, lane);

        if ((phy == &s->phy[3][0x71 + 8 * lane] ||
             phy == &s->phy[3][0x73 + 8 * lane]) &&
            pos >= NVT_QSEN_LO && pos <= NVT_QSEN_HI) {
            *v |= 0xFF;
            return;
        }
    }
    if (phy != &s->phy[0][0xB6]) {
        return;
    }
    for (lane = 0; lane < 4; lane++) {
        uint32_t pos = nvt_ddrc_qsen_pos(s, lane);

        if (pos >= NVT_QSEN_LO && pos < NVT_QSEN_EDGE) {
            *v |= 1u << (2 * lane);
        }
    }
}

/* The paged PHY's register for an offset, or NULL */
static uint32_t *nvt_ddrc_phy_reg(NvtDdrcState *s, hwaddr offset)
{
    hwaddr n = (offset - s->phy_off) / 4;
    hwaddr page = s->phy_off + (DDRC_PHY_PAGE - DDRC_PHY);

    if (!s->phy_pages || offset < s->phy_off || n >= DDRC_PHY_REGS) {
        return NULL;
    }
    return &s->phy[s->regs[page / 4] % DDRC_PAGES][n];
}

/* status bits that read as set: offset, mask */
static const struct {
    hwaddr offset;
    uint32_t done;
} nvt_ddrc_done[] = {
    { 0x0004, 7u << 8 },        /* controller: the commands' done */
    { 0x12DC, 1u << 7 },        /* PHY calibration done */
};

/* command bits that clear themselves (the command completes at once) */
static const struct {
    hwaddr offset;
    uint32_t start;
} nvt_ddrc_cmd[] = {
    { 0x0004, 0xF },            /* controller commands */
    { 0x0020, 0x3 },            /* DDR3 write-levelling / training run */
};

/*
 * NA51090's per-lane sweep ("psdk", run once per byte lane): the loader
 * starts it with page 3 0x00 [1:0], waits for bit 7 there, and takes the
 * lane as good when page 3 0x01 reads 0x78, 0x02..0x05 read 0 and 0x06
 * bit 7 is clear -- the result and its error counts, as a clean run
 * leaves them.
 */
static void nvt_ddrc_psdk_read(NvtDdrcState *s, uint32_t *phy, uint32_t *v)
{
    if (phy == &s->phy[3][0x00]) {
        *v |= 0x80;
    } else if (phy == &s->phy[3][0x01]) {
        *v = 0x78;
    } else if (phy >= &s->phy[3][0x02] && phy <= &s->phy[3][0x05]) {
        *v = 0;
    } else if (phy == &s->phy[3][0x06]) {
        *v &= ~0x80;
    }
}

static uint64_t nvt_ddrc_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtDdrcState *s = opaque;
    uint32_t *phy = nvt_ddrc_phy_reg(s, offset);
    uint32_t v = phy ? *phy : s->regs[offset / 4];
    int i;

    if (phy) {
        for (i = 0; i < ARRAY_SIZE(nvt_ddrc_phy_done); i++) {
            if (phy == &s->phy[nvt_ddrc_phy_done[i].page]
                              [nvt_ddrc_phy_done[i].reg]) {
                v |= nvt_ddrc_phy_done[i].done;
            }
        }
        if (s->qsen_match) {
            nvt_ddrc_qsen_read(s, phy, &v);
        }
        if (s->na51090) {
            nvt_ddrc_psdk_read(s, phy, &v);
        }
        for (i = 0; s->vref_strobe && i < ARRAY_SIZE(nvt_ddrc_phy_cmd); i++) {
            if (phy == &s->phy[nvt_ddrc_phy_cmd[i].page]
                              [nvt_ddrc_phy_cmd[i].reg]) {
                v &= ~nvt_ddrc_phy_cmd[i].start;
            }
        }
        return v;
    }

    if (s->tester && (offset & ~3) == DDRC_TEST_START) {
        return 0;
    }
    if (s->tester && (offset & ~3) == DDRC_TEST_STATUS) {
        return DDRC_TEST_DONE;
    }
    for (i = 0; s->ns02501 && i < ARRAY_SIZE(nvt_ddrc_ns02501_done); i++) {
        if (nvt_ddrc_ns02501_done[i].offset == (offset & ~3)) {
            v |= nvt_ddrc_ns02501_done[i].done;
        }
    }
    for (i = 0; i < ARRAY_SIZE(nvt_ddrc_cmd); i++) {
        if (nvt_ddrc_cmd[i].offset == (offset & ~3)) {
            v &= ~nvt_ddrc_cmd[i].start;
        }
    }

    for (i = 0; i < ARRAY_SIZE(nvt_ddrc_done); i++) {
        if (nvt_ddrc_done[i].offset == (offset & ~3)) {
            v |= nvt_ddrc_done[i].done;
        }
    }
    return v;
}

static void nvt_ddrc_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    NvtDdrcState *s = opaque;
    uint32_t *phy = nvt_ddrc_phy_reg(s, offset);

    if (phy) {
        *phy = val;
        if (s->na51090 && phy == &s->phy[0][0x2C] && (val & 0x80) &&
            (val & 0xF) == 5 && (s->phy[0][0x2B] & 0x80)) {
            s->pattern = s->phy[0][0x29];
        }
        return;
    }
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps nvt_ddrc_ops = {
    .read = nvt_ddrc_read,
    .write = nvt_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void nvt_ddrc_reset(DeviceState *dev)
{
    NvtDdrcState *s = NVT_DDRC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->phy, 0, sizeof(s->phy));
    s->pattern = 0xAA;
}

static void nvt_ddrc_init(Object *obj)
{
    NvtDdrcState *s = NVT_DDRC(obj);

    memory_region_init_io(&s->iomem, obj, &nvt_ddrc_ops, s, TYPE_NVT_DDRC,
                          DDRC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_nvt_ddrc = {
    .name = TYPE_NVT_DDRC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, NvtDdrcState, DDRC_SIZE / 4),
        VMSTATE_UINT32_2DARRAY(phy, NvtDdrcState, DDRC_PAGES, DDRC_PHY_REGS),
        VMSTATE_UINT8(pattern, NvtDdrcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_ddrc_properties[] = {
    DEFINE_PROP_BOOL("phy-pages", NvtDdrcState, phy_pages, false),
    DEFINE_PROP_BOOL("ns02501", NvtDdrcState, ns02501, false),
    DEFINE_PROP_BOOL("vref-strobe", NvtDdrcState, vref_strobe, false),
    DEFINE_PROP_BOOL("qsen-match", NvtDdrcState, qsen_match, false),
    DEFINE_PROP_BOOL("tester", NvtDdrcState, tester, false),
    DEFINE_PROP_BOOL("na51090", NvtDdrcState, na51090, false),
    DEFINE_PROP_UINT32("phy-offset", NvtDdrcState, phy_off, DDRC_PHY),
};

static void nvt_ddrc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, nvt_ddrc_properties);
    device_class_set_legacy_reset(dc, nvt_ddrc_reset);
    dc->vmsd = &vmstate_nvt_ddrc;
}

static const TypeInfo nvt_ddrc_info = {
    .name          = TYPE_NVT_DDRC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtDdrcState),
    .instance_init = nvt_ddrc_init,
    .class_init    = nvt_ddrc_class_init,
};

static void nvt_ddrc_register_types(void)
{
    type_register_static(&nvt_ddrc_info);
}

type_init(nvt_ddrc_register_types)
