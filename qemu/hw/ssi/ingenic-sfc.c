/*
 * Ingenic SFC: the T-series SPI flash controller at 0x13440000.
 *
 * A transfer is a list of phases, each a command byte, 0..7 address
 * bytes, a number of dummy clocks and an optional data stage, run on
 * TRIG.START.  Two generations of the block exist and this model is both
 * (version property):
 *
 * v1 (T10..T31; U-Boot arch-t31/sfc.h, kernel drivers/mtd/devices/jz_sfc.h)
 *
 *   GLB 0x00        13 TRAN_DIR (1 = to flash), 12:7 THRESHOLD, 6 OP_MODE
 *                   (1 = DMA), 5:3 PHASE_NUM
 *   TRAN_CONF(n) 0x14+4n  28:26 ADDR_WIDTH, 25 POLL_EN, 24 CMD_EN,
 *                   22:17 DMYBITS, 16 DATA_EN, 15:0 CMD (31:29 line mode)
 *   TRAN_LEN 0x2C, DEV_ADDR(n) 0x30+4n, DEV_ADDR_PLUS(n) 0x48+4n,
 *   MEM_ADDR 0x60, TRIG 0x64 (2 FLUSH, 1 STOP, 0 START; reads 0),
 *   SR 0x68 (22:16 FIFO words, 4 END, 3 TREQ, 2 RREQ, 1 OVER, 0 UNDER),
 *   SCR 0x6C (write 1 to clear), INTC 0x70 (mask, 1 = masked),
 *   DEV_STA_EXP 0x08 / DEV_STA_MSK 0x10 for POLL_EN phases, the last
 *   status read back in DEV_STA_RT 0x0C,
 *   DR 0x1000 the FIFO.
 *
 *   The phases run in order with chip select released between them; the
 *   data stage belongs to the phase with DATA_EN.  The SPL and U-Boot move
 *   data through DR by polling RREQ/TREQ and END; the kernel takes the
 *   same flags as a level interrupt and uses DMA (MEM_ADDR, the whole
 *   TRAN_LEN) for everything but the JEDEC ID.
 *
 * v2 (T40, T41; kernel drivers/mtd/devices/ingenic_sfc_v2) keeps that
 * single-command path for the SPL and U-Boot (line mode moves to
 * TRAN_CONF1 0x9C+4n) and adds a command table:
 *
 *   GLB0 15 DES_EN, 14 CDT_EN.  CMD_IDX 0x7C: 5:0 first entry, 31 DATA_EN,
 *   30 DATA_DIR.  COL_ADDR 0x80, ROW_ADDR 0x84, STA_ADDR0/1 0x88/0x8C,
 *   DES_ADDR 0x90, GLB1 0x94, CDT 0x800: 64 entries of {link (31 LINK to
 *   the next entry, 1:0 which address register), xfer (TRAN_CONF0
 *   layout), staExp, staMsk}.  DMA goes through descriptors at DES_ADDR:
 *   {next, mem_addr, len, link}.
 *
 * DMA addresses are taken modulo 512 MiB: the T41 kernel hands the
 * controller virt_to_phys() of its uncached (KSEG1) descriptor buffer,
 * 0x2xxxxxxx for RAM at 0x0xxxxxxx, and the silicon evidently decodes
 * only the low 29 bits.
 *
 * Flash chips are m25p80 NORs or ingenic-spinand SPI NANDs on the SSI
 * bus; both count dummies in clocks, one transfer each, which is what
 * DMYBITS holds.  A POLL_EN phase (the NAND drivers' wait on OIP, as a
 * v1 phase or a CDT entry) reads the status byte until (status & MSK) ==
 * EXP and leaves the last one in DEV_STA_RT, where the drivers look for
 * P_FAIL, E_FAIL and the ECC bits; the chips complete their operations
 * at once, unless a NAND is told to stay busy for a few reads.
 * Everything happens before the TRIG write returns, except PIO data,
 * which moves as the guest reads or writes DR.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/dma.h"

#define TYPE_INGENIC_SFC "ingenic-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSfcState, INGENIC_SFC)

#define R_GLB           0x00
#define R_DEV_STA_EXP   0x08
#define R_DEV_STA_RT    0x0C
#define R_DEV_STA_MSK   0x10
#define R_TRAN_CONF0    0x14
#define R_TRAN_LEN      0x2C
#define R_DEV_ADDR0     0x30
#define R_DEV_ADDR_PLUS0 0x48
#define R_MEM_ADDR      0x60
#define R_TRIG          0x64
#define R_SR            0x68
#define R_SCR           0x6C
#define R_INTC          0x70
#define R_CMD_IDX       0x7C
#define R_COL_ADDR      0x80
#define R_DES_ADDR      0x90
#define R_CDT           0x800
#define R_DR            0x1000

#define REGS_SIZE       0x100
#define CDT_ENTRIES     64
#define NPHASE          6

#define GLB_TRAN_DIR    (1u << 13)
#define GLB_OP_DMA      (1u << 6)
#define GLB_PHASE_NUM(g) extract32(g, 3, 3)
#define GLB0_CDT_EN     (1u << 14)
#define GLB0_DES_EN     (1u << 15)

#define TC_ADDR_WIDTH(t) extract32(t, 26, 3)
#define TC_POLL_EN      (1u << 25)
#define TC_CMD_EN       (1u << 24)
#define TC_DMYBITS(t)   extract32(t, 17, 6)
#define TC_DATA_EN      (1u << 16)
#define TC_CMD(t)       extract32(t, 0, 8)

#define TRIG_START      (1u << 0)
#define TRIG_STOP       (1u << 1)
#define TRIG_FLUSH      (1u << 2)

#define SR_UNDER        (1u << 0)
#define SR_OVER         (1u << 1)
#define SR_RREQ         (1u << 2)
#define SR_TREQ         (1u << 3)
#define SR_END          (1u << 4)
#define SR_FIFO_SHIFT   16

#define IDX_DATA_EN     (1u << 31)
#define IDX_DATA_DIR    (1u << 30)
#define CDT_LINK        (1u << 31)

#define FIFO_WORDS      63
#define DMA_MASK        0x1fffffff  /* the bus sees 29 address bits */
#define POLL_MAX        100000

typedef struct SfcPhase {
    uint32_t conf;          /* TRAN_CONF0 layout */
    uint64_t addr;
    uint32_t sta_exp, sta_msk;
} SfcPhase;

struct IngenicSfcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *spi;
    qemu_irq irq;
    qemu_irq cs;
    AddressSpace *dma_as;
    MemoryRegion *dma_mr;
    uint32_t version;

    uint32_t regs[REGS_SIZE / 4];
    uint32_t cdt[CDT_ENTRIES * 4];
    uint32_t sr;            /* latched END/OVER/UNDER */

    /* the transfer in progress */
    SfcPhase phase[CDT_ENTRIES];
    int nphase, cur;
    bool data_pending;      /* PIO data stage waiting on DR */
    bool data_write;
    uint32_t remaining;
};

static uint32_t *sfc_reg(IngenicSfcState *s, hwaddr off)
{
    return &s->regs[off >> 2];
}

static void sfc_update_irq(IngenicSfcState *s)
{
    uint32_t sr = s->sr;

    if (s->data_pending && s->remaining) {
        sr |= s->data_write ? SR_TREQ : SR_RREQ;
    }
    qemu_set_irq(s->irq, (sr & ~*sfc_reg(s, R_INTC) & 0x1f) != 0);
}

static void sfc_cs(IngenicSfcState *s, bool active)
{
    qemu_set_irq(s->cs, !active);
}

/* Command, address and dummy clocks of a phase, chip select held. */
static void sfc_phase_head(IngenicSfcState *s, const SfcPhase *p)
{
    int i, aw = TC_ADDR_WIDTH(p->conf);

    sfc_cs(s, true);
    if (p->conf & TC_CMD_EN) {
        ssi_transfer(s->spi, TC_CMD(p->conf));
    }
    for (i = aw - 1; i >= 0; i--) {
        ssi_transfer(s->spi, (p->addr >> (8 * i)) & 0xff);
    }
    for (i = 0; i < TC_DMYBITS(p->conf); i++) {
        ssi_transfer(s->spi, 0);
    }
}

static void sfc_poll_status(IngenicSfcState *s, const SfcPhase *p)
{
    int n;

    for (n = 0; n < POLL_MAX; n++) {
        uint32_t st = ssi_transfer(s->spi, 0);

        /* the drivers read the last status back for the fail/ECC bits */
        *sfc_reg(s, R_DEV_STA_RT) = st & 0xff;
        if ((st & p->sta_msk) == (p->sta_exp & p->sta_msk)) {
            return;
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-sfc: status poll of cmd 0x%x "
                  "never matched\n", TC_CMD(p->conf));
}

static void sfc_dma_move(IngenicSfcState *s, hwaddr addr, uint32_t len,
                         bool to_flash)
{
    uint8_t buf[256];

    while (len) {
        uint32_t n = MIN(len, sizeof(buf)), i;

        /* the bus is 29 bits: an access past its top wraps to 0 */
        addr &= DMA_MASK;
        n = MIN(n, DMA_MASK + 1 - addr);
        if (to_flash) {
            dma_memory_read(s->dma_as, addr, buf, n, MEMTXATTRS_UNSPECIFIED);
            for (i = 0; i < n; i++) {
                ssi_transfer(s->spi, buf[i]);
            }
        } else {
            for (i = 0; i < n; i++) {
                buf[i] = ssi_transfer(s->spi, 0);
            }
            dma_memory_write(s->dma_as, addr, buf, n, MEMTXATTRS_UNSPECIFIED);
        }
        addr += n;
        len -= n;
    }
}

/* DMA data stage: one buffer (v1, v2 without DES_EN) or a descriptor chain. */
static void sfc_dma_data(IngenicSfcState *s, uint32_t len, bool to_flash)
{
    uint32_t glb = *sfc_reg(s, R_GLB);
    hwaddr des = *sfc_reg(s, R_DES_ADDR) & DMA_MASK;
    int n;

    if (s->version < 2 || !(glb & GLB0_DES_EN)) {
        sfc_dma_move(s, *sfc_reg(s, R_MEM_ADDR) & DMA_MASK, len, to_flash);
        return;
    }
    for (n = 0; n < 1024 && len; n++) {
        uint32_t d[4];

        dma_memory_read(s->dma_as, des, d, sizeof(d), MEMTXATTRS_UNSPECIFIED);
        d[2] = MIN(le32_to_cpu(d[2]), len);
        sfc_dma_move(s, le32_to_cpu(d[1]) & DMA_MASK, d[2], to_flash);
        len -= d[2];
        if (!le32_to_cpu(d[3])) {
            break;
        }
        des = le32_to_cpu(d[0]) & DMA_MASK;
    }
}

static void sfc_finish(IngenicSfcState *s)
{
    s->data_pending = false;
    s->remaining = 0;
    s->nphase = 0;
    s->sr |= SR_END;
}

/*
 * A read has ended on the wire once the rest is in the FIFO: END does
 * not wait for the CPU to drain it, and the vendor U-Boots read the words
 * they want and then wait for END (a two-byte status read leaves part of
 * a word behind).  But not before the rest is one RREQ's worth (GLB
 * THRESHOLD words): the T41 U-Boot's jz_sfc_v1 loop takes one SR
 * snapshot, reads THRESHOLD words on RREQ and stops on END in the same
 * snapshot, so an earlier END loses the words past the threshold (the
 * last 64 bytes of every 2 KiB NAND page).
 */
static void sfc_rx_end(IngenicSfcState *s)
{
    uint32_t thr = extract32(*sfc_reg(s, R_GLB), 7, 6);

    thr = MAX(MIN(thr, FIFO_WORDS), 1);
    if (s->data_pending && !s->data_write && s->cur == s->nphase - 1 &&
        s->remaining <= thr * 4) {
        s->sr |= SR_END;
    }
}

/* Run phases from s->cur until one waits on DR, or the transfer ends. */
static void sfc_run(IngenicSfcState *s)
{
    uint32_t len = *sfc_reg(s, R_TRAN_LEN);

    for (; s->cur < s->nphase; s->cur++) {
        const SfcPhase *p = &s->phase[s->cur];

        sfc_phase_head(s, p);
        if (p->conf & TC_POLL_EN) {
            sfc_poll_status(s, p);
        } else if ((p->conf & TC_DATA_EN) && len) {
            if (*sfc_reg(s, R_GLB) & GLB_OP_DMA) {
                sfc_dma_data(s, len, s->data_write);
            } else {
                s->data_pending = true;
                s->remaining = len;
                sfc_rx_end(s);
                return;         /* chip select stays on for DR */
            }
        }
        sfc_cs(s, false);
    }
    sfc_finish(s);
}

static void sfc_abort(IngenicSfcState *s);

static void sfc_start(IngenicSfcState *s)
{
    uint32_t glb = *sfc_reg(s, R_GLB);
    int i;

    sfc_abort(s);           /* whatever the last read left in the FIFO */
    s->nphase = 0;
    s->cur = 0;
    s->data_pending = false;
    s->remaining = 0;
    s->data_write = (glb & GLB_TRAN_DIR) != 0;

    if (s->version >= 2 && (glb & GLB0_CDT_EN)) {
        uint32_t idx = *sfc_reg(s, R_CMD_IDX);
        int e = idx & 0x3f;

        s->data_write = (idx & IDX_DATA_DIR) != 0;
        do {
            uint32_t *c = &s->cdt[e * 4];
            SfcPhase *p = &s->phase[s->nphase++];

            p->conf = c[1];
            if (!(idx & IDX_DATA_EN)) {
                p->conf &= ~TC_DATA_EN;
            }
            p->addr = *sfc_reg(s, R_COL_ADDR + 4 * (c[0] & 3));
            p->sta_exp = c[2];
            p->sta_msk = c[3];
            if (!(c[0] & CDT_LINK)) {
                break;
            }
            e++;
        } while (e < CDT_ENTRIES && s->nphase < CDT_ENTRIES);
    } else {
        int n = GLB_PHASE_NUM(glb);

        for (i = 0; i < MAX(n, 1) && i < NPHASE; i++) {
            SfcPhase *p = &s->phase[s->nphase++];

            p->conf = *sfc_reg(s, R_TRAN_CONF0 + 4 * i);
            p->addr = *sfc_reg(s, R_DEV_ADDR0 + 4 * i) |
                      (uint64_t)*sfc_reg(s, R_DEV_ADDR_PLUS0 + 4 * i) << 32;
            p->sta_exp = *sfc_reg(s, R_DEV_STA_EXP);
            p->sta_msk = *sfc_reg(s, R_DEV_STA_MSK);
        }
    }
    sfc_run(s);
}

static void sfc_abort(IngenicSfcState *s)
{
    if (s->data_pending) {
        sfc_cs(s, false);
    }
    s->data_pending = false;
    s->remaining = 0;
    s->nphase = 0;
}

/* The PIO data stage ran out: release the chip and go on with the list. */
static void sfc_data_done(IngenicSfcState *s)
{
    s->data_pending = false;
    sfc_cs(s, false);
    s->cur++;
    sfc_run(s);
}

static uint32_t sfc_dr_read(IngenicSfcState *s)
{
    uint32_t v = 0;
    int i;

    if (!s->data_pending || s->data_write || !s->remaining) {
        s->sr |= SR_UNDER;
        return 0;
    }
    for (i = 0; i < 4 && s->remaining; i++, s->remaining--) {
        v |= (uint32_t)(ssi_transfer(s->spi, 0) & 0xff) << (8 * i);
    }
    if (!s->remaining) {
        sfc_data_done(s);
    } else {
        sfc_rx_end(s);
    }
    return v;
}

static void sfc_dr_write(IngenicSfcState *s, uint32_t v)
{
    int i;

    if (!s->data_pending || !s->data_write || !s->remaining) {
        return;     /* the kernel's init writes one word into an idle FIFO */
    }
    for (i = 0; i < 4 && s->remaining; i++, s->remaining--) {
        ssi_transfer(s->spi, (v >> (8 * i)) & 0xff);
    }
    if (!s->remaining) {
        sfc_data_done(s);
    }
}

static uint64_t ingenic_sfc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicSfcState *s = opaque;
    uint32_t v = 0, words;

    if (off >= R_DR) {
        v = sfc_dr_read(s);
        sfc_update_irq(s);
        return v;
    }
    if (off >= R_CDT && off < R_CDT + sizeof(s->cdt)) {
        return s->cdt[(off - R_CDT) >> 2];
    }
    if (off >= REGS_SIZE) {
        return 0;
    }
    switch (off) {
    case R_TRIG:
        return 0;
    case R_SR:
        v = s->sr;
        if (s->data_pending && s->remaining) {
            v |= s->data_write ? SR_TREQ : SR_RREQ;
            words = s->data_write ? FIFO_WORDS - 1 :
                    MIN(DIV_ROUND_UP(s->remaining, 4), FIFO_WORDS);
            v |= words << SR_FIFO_SHIFT;
        }
        return v;
    default:
        return *sfc_reg(s, off);
    }
}

static void ingenic_sfc_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicSfcState *s = opaque;

    if (off >= R_DR) {
        sfc_dr_write(s, val);
    } else if (off >= R_CDT && off < R_CDT + sizeof(s->cdt)) {
        s->cdt[(off - R_CDT) >> 2] = val;
        return;
    } else if (off < REGS_SIZE) {
        switch (off) {
        case R_TRIG:
            if (val & (TRIG_STOP | TRIG_FLUSH)) {
                sfc_abort(s);
            }
            if (val & TRIG_START) {
                sfc_start(s);
            }
            break;
        case R_SR:
            break;
        case R_SCR:
            s->sr &= ~val;
            break;
        default:
            *sfc_reg(s, off) = val;
            break;
        }
    }
    sfc_update_irq(s);
}

static const MemoryRegionOps ingenic_sfc_ops = {
    .read = ingenic_sfc_read,
    .write = ingenic_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ingenic_sfc_reset(DeviceState *dev)
{
    IngenicSfcState *s = INGENIC_SFC(dev);

    sfc_abort(s);
    sfc_cs(s, false);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->cdt, 0, sizeof(s->cdt));
    *sfc_reg(s, R_INTC) = 0x1f;
    s->sr = 0;
    sfc_update_irq(s);
}

static void ingenic_sfc_realize(DeviceState *dev, Error **errp)
{
    IngenicSfcState *s = INGENIC_SFC(dev);

    s->dma_as = g_new0(AddressSpace, 1);
    address_space_init(s->dma_as, s->dma_mr ? s->dma_mr : get_system_memory(),
                       "ingenic-sfc-dma");
}

static void ingenic_sfc_init(Object *obj)
{
    IngenicSfcState *s = INGENIC_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_sfc_ops, s,
                          TYPE_INGENIC_SFC, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->cs);
    s->spi = ssi_create_bus(DEVICE(obj), "spi");
}

static const Property ingenic_sfc_props[] = {
    DEFINE_PROP_UINT32("version", IngenicSfcState, version, 1),
    DEFINE_PROP_LINK("dma-memory", IngenicSfcState, dma_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static const VMStateDescription vmstate_sfc_phase = {
    .name = TYPE_INGENIC_SFC "/phase",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf, SfcPhase),
        VMSTATE_UINT64(addr, SfcPhase),
        VMSTATE_UINT32(sta_exp, SfcPhase),
        VMSTATE_UINT32(sta_msk, SfcPhase),
        VMSTATE_END_OF_LIST()
    }
};

/* version 2: the transfer in progress too (a PIO data stage on DR) */
static const VMStateDescription vmstate_ingenic_sfc = {
    .name = TYPE_INGENIC_SFC,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicSfcState, REGS_SIZE / 4),
        VMSTATE_UINT32_ARRAY(cdt, IngenicSfcState, CDT_ENTRIES * 4),
        VMSTATE_UINT32(sr, IngenicSfcState),
        VMSTATE_STRUCT_ARRAY(phase, IngenicSfcState, CDT_ENTRIES, 2,
                             vmstate_sfc_phase, SfcPhase),
        VMSTATE_INT32_V(nphase, IngenicSfcState, 2),
        VMSTATE_INT32_V(cur, IngenicSfcState, 2),
        VMSTATE_BOOL_V(data_pending, IngenicSfcState, 2),
        VMSTATE_BOOL_V(data_write, IngenicSfcState, 2),
        VMSTATE_UINT32_V(remaining, IngenicSfcState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_sfc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ingenic_sfc_realize;
    device_class_set_legacy_reset(dc, ingenic_sfc_reset);
    device_class_set_props(dc, ingenic_sfc_props);
    dc->vmsd = &vmstate_ingenic_sfc;
}

static const TypeInfo ingenic_sfc_info = {
    .name = TYPE_INGENIC_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSfcState),
    .instance_init = ingenic_sfc_init,
    .class_init = ingenic_sfc_class_init,
};

static void ingenic_sfc_register_types(void)
{
    type_register_static(&ingenic_sfc_info);
}

type_init(ingenic_sfc_register_types)
