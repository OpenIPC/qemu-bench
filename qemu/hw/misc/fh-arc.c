/*
 * Fullhan FH8626V100 ARC coprocessor: an ARC600 (ARCompact) core that runs
 * from system memory beside the ARM, as OpenIPC's 4.19 kernel drives it
 * (drivers/remoteproc/fh8626_arc.c) with its own firmware
 * (OpenIPC/firmware general/package/fh8626-arc-firmware: an RPMsg echo).
 *
 * The ARM loads the firmware into DDR, writes four boot words at PMU
 * 0x2000 (sr entry, [INT_VECTOR_BASE]; j entry), releases the ARC's reset
 * (PMU 0x1C4 bit 18) and sets its run bit (PMU 0x2010 bit 4).  The core
 * then executes from 0xF0002000, the boot words' physical address, and
 * sees the ARM's physical map: DDR, and the PMU's doorbells at 0x1E4 (to
 * the ARC) and 0x1F0 (to the ARM, which interrupts it).  The PMU turns
 * those registers into this device's "run" and "kick" inputs.
 *
 * What is modelled is the instruction set that firmware is built from,
 * decoded as the ARC600 does it: the 32-bit formats of major opcodes
 * 0x00-0x05 -- Bcc and B, BL and BRcc, LD and ST (word, half, byte, with
 * address write-back and sign extension), the general ALU operations
 * (ADD, ADC, SUB, SBC, RSUB, AND, OR, BIC, XOR, MOV, TST, CMP, MIN, MAX,
 * J, JL, LR, SR, SYNC) in all four operand formats with a long immediate,
 * conditional execution and the flags, and the barrel shifter's ASL, LSR,
 * ASR and ROR.  No 16-bit instructions, delay slots, zero-overhead loops
 * or interrupts into the ARC: an instruction outside the set takes the
 * instruction-error exception (vector 0x10), as a reserved one does on the
 * core, and a failed memory access the memory-error one (0x08).  Both are
 * logged; the echo firmware reports either to the ARM as a crash.
 *
 * Timing: "mips" million instructions per second of virtual time, in
 * 100 us slices, and a slice at once on the ARM's doorbell.  The echo
 * firmware polls its rings without a pause, so a slice that stores nothing
 * is taken as polling and the next comes twice as late, up to 10 ms; a
 * store, the doorbell or a start goes back to 100 us.  Linux rings the
 * doorbell after it posts to a ring, so only a core with nothing to do
 * slows down.  A core spinning on a branch to itself ends its slice there.
 * (The ARM is the bigger cost: OpenIPC's FH8626 kernel never issues WFI, so
 * the guest takes a host core whether the ARC runs or not.)
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define TYPE_FH_ARC "fh-arc"
OBJECT_DECLARE_SIMPLE_TYPE(FhArcState, FH_ARC)

#define FH_ARC_SLICE_NS     100000              /* 100 us */
#define FH_ARC_IDLE_MAX_NS  10000000            /* 10 ms */
#define FH_ARC_VEC_MEMERR   0x08
#define FH_ARC_VEC_INSNERR  0x10

#define R_BLINK             31
#define R_LIMM              62
#define R_PCL               63

#define AUX_STATUS32        0x0A
#define AUX_IDENTITY        0x04
#define AUX_VECBASE         0x25
#define ARC600_IDENTITY     0x00000022          /* ARCVER 0x22: ARC600 */

struct FhArcState {
    SysBusDevice parent_obj;

    QEMUTimer *timer;
    uint32_t r[64];
    uint32_t pc;
    uint32_t vecbase;
    bool z, n, c, v;
    bool run;
    uint32_t period_ns;         /* to the next slice */
    uint32_t stores;

    uint32_t boot_addr;
    uint32_t mips;
};

/* Executing one instruction: what it found to go on with. */
typedef struct {
    uint32_t insn;
    uint32_t limm;
    unsigned len;
    bool fault;
} FhArcInsn;

static uint32_t sext(uint32_t v, unsigned bits)
{
    return (uint32_t)((int32_t)(v << (32 - bits)) >> (32 - bits));
}

/*
 * The PMU, where the boot words are, takes only words, and ARCompact
 * stores an instruction as halfwords, the most significant first.
 */
static bool fh_arc_fetch16(uint32_t addr, uint16_t *hw)
{
    MemTxResult res;
    uint32_t w = address_space_ldl_le(&address_space_memory, addr & ~3u,
                                      MEMTXATTRS_UNSPECIFIED, &res);

    *hw = (addr & 2) ? w >> 16 : w & 0xFFFF;
    return res == MEMTX_OK;
}

static bool fh_arc_fetch32(uint32_t addr, uint32_t *w)
{
    uint16_t hi, lo;

    if (!(addr & 2)) {
        MemTxResult res;
        uint32_t v = address_space_ldl_le(&address_space_memory, addr,
                                          MEMTXATTRS_UNSPECIFIED, &res);

        *w = v << 16 | v >> 16;
        return res == MEMTX_OK;
    }
    if (!fh_arc_fetch16(addr, &hi) || !fh_arc_fetch16(addr + 2, &lo)) {
        return false;
    }
    *w = (uint32_t)hi << 16 | lo;
    return true;
}

static uint32_t fh_arc_reg(FhArcState *s, FhArcInsn *in, unsigned n)
{
    if (n == R_LIMM) {
        return in->limm;
    }
    if (n == R_PCL) {
        return s->pc & ~3u;
    }
    return s->r[n];
}

static void fh_arc_set(FhArcState *s, unsigned n, uint32_t val)
{
    if (n < 61) {
        s->r[n] = val;
    }
}

static bool fh_arc_cond(FhArcState *s, unsigned cc)
{
    switch (cc) {
    case 0x00: return true;
    case 0x01: return s->z;
    case 0x02: return !s->z;
    case 0x03: return !s->n;
    case 0x04: return s->n;
    case 0x05: return s->c;
    case 0x06: return !s->c;
    case 0x07: return s->v;
    case 0x08: return !s->v;
    case 0x09: return s->n == s->v && !s->z;
    case 0x0A: return s->n == s->v;
    case 0x0B: return s->n != s->v;
    case 0x0C: return s->z || s->n != s->v;
    case 0x0D: return !s->c && !s->z;
    case 0x0E: return s->c || s->z;
    case 0x0F: return !s->n && !s->z;
    }
    return false;
}

static void fh_arc_flags_zn(FhArcState *s, uint32_t res)
{
    s->z = res == 0;
    s->n = res >> 31;
}

static uint32_t fh_arc_add(FhArcState *s, uint32_t a, uint32_t b, bool cin,
                           bool f)
{
    uint64_t sum = (uint64_t)a + b + cin;
    uint32_t res = sum;

    if (f) {
        fh_arc_flags_zn(s, res);
        s->c = sum >> 32;
        s->v = ((a ^ res) & (b ^ res)) >> 31;
    }
    return res;
}

static uint32_t fh_arc_sub(FhArcState *s, uint32_t a, uint32_t b, bool bin,
                           bool f)
{
    uint32_t res = a - b - bin;

    if (f) {
        fh_arc_flags_zn(s, res);
        s->c = (uint64_t)a < (uint64_t)b + bin;
        s->v = ((a ^ b) & (a ^ res)) >> 31;
    }
    return res;
}

static bool fh_arc_ld(uint32_t addr, unsigned zz, bool x, uint32_t *val)
{
    MemTxResult res;

    switch (zz) {
    case 0:
        *val = address_space_ldl_le(&address_space_memory, addr,
                                    MEMTXATTRS_UNSPECIFIED, &res);
        break;
    case 1:
        *val = address_space_ldub(&address_space_memory, addr,
                                  MEMTXATTRS_UNSPECIFIED, &res);
        if (x) {
            *val = sext(*val, 8);
        }
        break;
    case 2:
        *val = address_space_lduw_le(&address_space_memory, addr,
                                     MEMTXATTRS_UNSPECIFIED, &res);
        if (x) {
            *val = sext(*val, 16);
        }
        break;
    default:
        return false;
    }
    return res == MEMTX_OK;
}

static bool fh_arc_st(uint32_t addr, unsigned zz, uint32_t val)
{
    MemTxResult res;

    switch (zz) {
    case 0:
        address_space_stl_le(&address_space_memory, addr, val,
                             MEMTXATTRS_UNSPECIFIED, &res);
        break;
    case 1:
        address_space_stb(&address_space_memory, addr, val,
                          MEMTXATTRS_UNSPECIFIED, &res);
        break;
    case 2:
        address_space_stw_le(&address_space_memory, addr, val,
                             MEMTXATTRS_UNSPECIFIED, &res);
        break;
    default:
        return false;
    }
    return res == MEMTX_OK;
}

/*
 * The address an LD / ST uses, and whether the base register is to be
 * written back (with *wb) -- only once the access has succeeded, so a
 * faulting one leaves it as it was.
 */
static uint32_t fh_arc_ea(uint32_t base, uint32_t off, unsigned aa,
                          unsigned zz, bool *update, uint32_t *wb)
{
    static const uint8_t scale[3] = { 2, 0, 1 };    /* word, byte, half */

    *update = aa == 1 || aa == 2;
    *wb = base + off;
    switch (aa) {
    case 2:                                     /* .ab: post-update */
        return base;
    case 3:                                     /* .as: scaled */
        return base + (off << scale[zz]);
    }
    return base + off;                          /* also .a: pre-update */
}

static uint32_t fh_arc_lr(FhArcState *s, uint32_t aux)
{
    switch (aux) {
    case AUX_IDENTITY:
        return ARC600_IDENTITY;
    case AUX_STATUS32:
        return (uint32_t)s->z << 11 | (uint32_t)s->n << 10 |
               (uint32_t)s->c << 9 | (uint32_t)s->v << 8;
    case AUX_VECBASE:
        return s->vecbase;
    }
    qemu_log_mask(LOG_UNIMP, "fh-arc: LR of aux register 0x%x\n", aux);
    return 0;
}

static void fh_arc_sr(FhArcState *s, uint32_t aux, uint32_t val)
{
    if (aux == AUX_VECBASE) {
        s->vecbase = val & ~0x3FFu;
        return;
    }
    qemu_log_mask(LOG_UNIMP, "fh-arc: SR 0x%x to aux register 0x%x\n",
                  val, aux);
}

/* Major opcodes 0x04 and 0x05: the general and the shift operations. */
static bool fh_arc_alu(FhArcState *s, FhArcInsn *in, uint32_t *next)
{
    uint32_t insn = in->insn;
    unsigned major = insn >> 27;
    unsigned b = ((insn >> 24) & 7) | ((insn >> 12) & 7) << 3;
    unsigned p = (insn >> 22) & 3;
    unsigned op = (insn >> 16) & 0x3F;
    bool f = (insn >> 15) & 1;
    unsigned cf = (insn >> 6) & 0x3F;
    unsigned af = insn & 0x3F;
    unsigned dest = af;
    uint32_t src1, src2, res;

    switch (p) {
    case 0:
        src2 = fh_arc_reg(s, in, cf);
        break;
    case 1:
        src2 = cf;
        break;
    case 2:
        src2 = sext(af << 6 | cf, 12);
        dest = b;
        break;
    default:
        src2 = (insn & 0x20) ? cf : fh_arc_reg(s, in, cf);
        dest = b;
        if (insn & 0x10) {
            return false;               /* extension condition codes */
        }
        if (!fh_arc_cond(s, insn & 0xF)) {
            return true;
        }
        break;
    }
    src1 = fh_arc_reg(s, in, b);

    if (major == 0x05) {
        unsigned sh = src2 & 31;

        switch (op) {
        case 0x00: res = src1 << sh; break;
        case 0x01: res = src1 >> sh; break;
        case 0x02: res = (uint32_t)((int32_t)src1 >> sh); break;
        case 0x03: res = sh ? src1 >> sh | src1 << (32 - sh) : src1; break;
        default:
            return false;
        }
        if (f) {
            fh_arc_flags_zn(s, res);
            if (sh) {
                s->c = op == 0x00 ? (src1 >> (32 - sh)) & 1
                                  : (src1 >> (sh - 1)) & 1;
            }
        }
        fh_arc_set(s, dest, res);
        return true;
    }

    switch (op) {
    case 0x00: res = fh_arc_add(s, src1, src2, false, f); break;
    case 0x01: res = fh_arc_add(s, src1, src2, s->c, f); break;
    case 0x02: res = fh_arc_sub(s, src1, src2, false, f); break;
    case 0x03: res = fh_arc_sub(s, src1, src2, s->c, f); break;
    case 0x0E: res = fh_arc_sub(s, src2, src1, false, f); break;
    case 0x04: res = src1 & src2; goto logic;
    case 0x05: res = src1 | src2; goto logic;
    case 0x06: res = src1 & ~src2; goto logic;
    case 0x07: res = src1 ^ src2; goto logic;
    case 0x08:
        res = (int32_t)src1 > (int32_t)src2 ? src1 : src2;
        goto minmax;
    case 0x09:
        res = (int32_t)src1 < (int32_t)src2 ? src1 : src2;
        goto minmax;
    case 0x0A:                                  /* MOV b, c */
        if (f) {
            fh_arc_flags_zn(s, src2);
        }
        fh_arc_set(s, b, src2);
        return true;
    case 0x0B:                                  /* TST */
        fh_arc_flags_zn(s, src1 & src2);
        return true;
    case 0x0C:                                  /* CMP */
        fh_arc_sub(s, src1, src2, false, true);
        return true;
    case 0x20:                                  /* J */
    case 0x22:                                  /* JL */
        if (op == 0x22) {
            s->r[R_BLINK] = s->pc + in->len;
        }
        *next = src2 & ~1u;
        return true;
    case 0x2A:                                  /* LR b, [c] */
        fh_arc_set(s, b, fh_arc_lr(s, src2));
        return true;
    case 0x2B:                                  /* SR b, [c] */
        fh_arc_sr(s, src2, src1);
        return true;
    case 0x2F:                                  /* SYNC; the rest not */
        return af == 0x3F && b == 3;
    default:
        return false;
    }
    fh_arc_set(s, dest, res);
    return true;

logic:
    if (f) {
        fh_arc_flags_zn(s, res);
    }
    fh_arc_set(s, dest, res);
    return true;

minmax:
    if (f) {
        fh_arc_sub(s, src1, src2, false, true);
    }
    fh_arc_set(s, dest, res);
    return true;
}

/* Whether a register field of this instruction names the long immediate. */
static bool fh_arc_wants_limm(uint32_t insn)
{
    unsigned major = insn >> 27;
    unsigned b = ((insn >> 24) & 7) | ((insn >> 12) & 7) << 3;
    unsigned cf = (insn >> 6) & 0x3F;

    switch (major) {
    case 0x01:
        return (insn & 0x10000) &&
               (b == R_LIMM || (!(insn & 0x10) && cf == R_LIMM));
    case 0x02:
        return b == R_LIMM;
    case 0x03:
        return b == R_LIMM || cf == R_LIMM;
    case 0x04:
    case 0x05: {
        unsigned p = (insn >> 22) & 3;
        bool creg = p == 0 || (p == 3 && !(insn & 0x20));

        return b == R_LIMM || (creg && cf == R_LIMM);
    }
    }
    return false;
}

/* One instruction, or the exception it takes. */
static void fh_arc_step(FhArcState *s)
{
    FhArcInsn in = { .len = 4 };
    uint32_t insn, next;
    unsigned major;
    bool ok = false;

    if (!fh_arc_fetch32(s->pc, &insn)) {
        goto memerr;
    }
    in.insn = insn;
    major = insn >> 27;
    if (major > 0x05) {
        goto insnerr;           /* 16-bit, or a 32-bit extension opcode */
    }
    if (fh_arc_wants_limm(insn)) {
        if (!fh_arc_fetch32(s->pc + 4, &in.limm)) {
            goto memerr;
        }
        in.len = 8;
    }
    next = s->pc + in.len;

    switch (major) {
    case 0x00: {                                /* Bcc, B */
        uint32_t disp = ((insn >> 17) & 0x3FF) << 1 |
                        ((insn >> 6) & 0x3FF) << 11;

        if (insn & 0x20) {
            goto insnerr;                       /* .d: no delay slots */
        }
        if (insn & 0x10000) {
            disp = sext(disp | (insn & 0xF) << 21, 25);
            next = (s->pc & ~3u) + disp;
        } else if (fh_arc_cond(s, insn & 0x1F)) {
            next = (s->pc & ~3u) + sext(disp, 21);
        }
        ok = true;
        break;
    }
    case 0x01:
        if (insn & 0x20) {
            goto insnerr;
        }
        if (!(insn & 0x10000)) {                /* BLcc, BL */
            uint32_t disp = ((insn >> 18) & 0x1FF) << 2 |
                            ((insn >> 6) & 0x3FF) << 11;
            bool take = true;

            if (insn & 0x20000) {
                disp = sext(disp | (insn & 0xF) << 21, 25);
            } else {
                disp = sext(disp, 21);
                take = fh_arc_cond(s, insn & 0x1F);
            }
            if (take) {
                s->r[R_BLINK] = s->pc + in.len;
                next = (s->pc & ~3u) + disp;
            }
            ok = true;
        } else {                                /* BRcc */
            unsigned b = ((insn >> 24) & 7) | ((insn >> 12) & 7) << 3;
            unsigned cf = (insn >> 6) & 0x3F;
            uint32_t disp = sext(((insn >> 15) & 1) << 8 |
                                 ((insn >> 17) & 0x7F) << 1, 9);
            uint32_t x = fh_arc_reg(s, &in, b);
            uint32_t y = (insn & 0x10) ? cf : fh_arc_reg(s, &in, cf);
            bool take;

            switch (insn & 0xF) {
            case 0x0: take = x == y; break;
            case 0x1: take = x != y; break;
            case 0x2: take = (int32_t)x < (int32_t)y; break;
            case 0x3: take = (int32_t)x >= (int32_t)y; break;
            case 0x4: take = x < y; break;
            case 0x5: take = x >= y; break;
            default:
                goto insnerr;                   /* BBIT0 / BBIT1 */
            }
            if (take) {
                next = (s->pc & ~3u) + disp;
            }
            ok = true;
        }
        break;
    case 0x02: {                                /* LD a, [b, s9] */
        unsigned b = ((insn >> 24) & 7) | ((insn >> 12) & 7) << 3;
        uint32_t off = sext(((insn >> 15) & 1) << 8 | ((insn >> 16) & 0xFF),
                            9);
        unsigned zz = (insn >> 7) & 3;
        uint32_t ea, wb, val;
        bool update;

        if (zz == 3) {
            goto insnerr;
        }
        ea = fh_arc_ea(fh_arc_reg(s, &in, b), off, (insn >> 9) & 3, zz,
                       &update, &wb);
        if (!fh_arc_ld(ea, zz, (insn >> 6) & 1, &val)) {
            goto memerr;
        }
        if (update) {
            fh_arc_set(s, b, wb);
        }
        fh_arc_set(s, insn & 0x3F, val);
        ok = true;
        break;
    }
    case 0x03: {                                /* ST c, [b, s9] */
        unsigned b = ((insn >> 24) & 7) | ((insn >> 12) & 7) << 3;
        uint32_t off = sext(((insn >> 15) & 1) << 8 | ((insn >> 16) & 0xFF),
                            9);
        unsigned zz = (insn >> 1) & 3;
        uint32_t val = fh_arc_reg(s, &in, (insn >> 6) & 0x3F);
        uint32_t ea, wb;
        bool update;

        if (zz == 3) {
            goto insnerr;
        }
        ea = fh_arc_ea(fh_arc_reg(s, &in, b), off, (insn >> 3) & 3, zz,
                       &update, &wb);
        if (!fh_arc_st(ea, zz, val)) {
            goto memerr;
        }
        if (update) {
            fh_arc_set(s, b, wb);
        }
        s->stores++;
        ok = true;
        break;
    }
    default:                                    /* 0x04, 0x05 */
        ok = fh_arc_alu(s, &in, &next);
        break;
    }
    if (!ok) {
        goto insnerr;
    }
    s->pc = next;
    return;

insnerr:
    qemu_log_mask(LOG_UNIMP, "fh-arc: instruction 0x%08x at 0x%08x not "
                  "modelled: instruction error\n", in.insn, s->pc);
    s->r[30] = s->pc;                           /* ILINK2 */
    s->pc = s->vecbase + FH_ARC_VEC_INSNERR;
    return;
memerr:
    qemu_log_mask(LOG_GUEST_ERROR, "fh-arc: memory error at 0x%08x\n",
                  s->pc);
    s->r[30] = s->pc;
    s->pc = s->vecbase + FH_ARC_VEC_MEMERR;
}

static void fh_arc_tick(void *opaque)
{
    FhArcState *s = opaque;
    unsigned n = s->mips * (FH_ARC_SLICE_NS / 1000);
    uint32_t stores = s->stores;

    if (!s->run) {
        return;
    }
    while (n--) {
        uint32_t pc = s->pc;

        fh_arc_step(s);
        if (s->pc == pc) {
            break;              /* spinning on itself until a reset */
        }
    }
    if (s->stores != stores) {
        s->period_ns = FH_ARC_SLICE_NS;
    } else {
        s->period_ns = MIN(s->period_ns * 2, FH_ARC_IDLE_MAX_NS);
    }
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              s->period_ns);
}

static void fh_arc_core_reset(FhArcState *s)
{
    memset(s->r, 0, sizeof(s->r));
    s->pc = s->boot_addr;
    s->vecbase = 0;
    s->z = s->n = s->c = s->v = false;
}

static void fh_arc_set_run(void *opaque, int n, int level)
{
    FhArcState *s = opaque;

    if (level && !s->run) {
        fh_arc_core_reset(s);
        s->run = true;
        s->period_ns = FH_ARC_SLICE_NS;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    } else if (!level && s->run) {
        s->run = false;
        timer_del(s->timer);
    }
}

static void fh_arc_kick(void *opaque, int n, int level)
{
    FhArcState *s = opaque;

    if (level && s->run) {
        s->period_ns = FH_ARC_SLICE_NS;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void fh_arc_reset(DeviceState *dev)
{
    FhArcState *s = FH_ARC(dev);

    s->run = false;
    timer_del(s->timer);
    fh_arc_core_reset(s);
}

static void fh_arc_init(Object *obj)
{
    FhArcState *s = FH_ARC(obj);

    qdev_init_gpio_in_named(DEVICE(obj), fh_arc_set_run, "run", 1);
    qdev_init_gpio_in_named(DEVICE(obj), fh_arc_kick, "kick", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, fh_arc_tick, s);
}

static const Property fh_arc_props[] = {
    DEFINE_PROP_UINT32("boot-addr", FhArcState, boot_addr, 0xF0002000),
    DEFINE_PROP_UINT32("mips", FhArcState, mips, 2),
};

static const VMStateDescription vmstate_fh_arc = {
    .name = TYPE_FH_ARC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(r, FhArcState, 64),
        VMSTATE_UINT32(pc, FhArcState),
        VMSTATE_UINT32(vecbase, FhArcState),
        VMSTATE_BOOL(z, FhArcState),
        VMSTATE_BOOL(n, FhArcState),
        VMSTATE_BOOL(c, FhArcState),
        VMSTATE_BOOL(v, FhArcState),
        VMSTATE_BOOL(run, FhArcState),
        VMSTATE_UINT32(period_ns, FhArcState),
        VMSTATE_UINT32(stores, FhArcState),
        VMSTATE_TIMER_PTR(timer, FhArcState),
        VMSTATE_END_OF_LIST()
    },
};

static void fh_arc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, fh_arc_reset);
    device_class_set_props(dc, fh_arc_props);
    dc->vmsd = &vmstate_fh_arc;
}

static const TypeInfo fh_arc_info = {
    .name          = TYPE_FH_ARC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FhArcState),
    .instance_init = fh_arc_init,
    .class_init    = fh_arc_class_init,
};

static void fh_arc_register_types(void)
{
    type_register_static(&fh_arc_info);
}

type_init(fh_arc_register_types)
