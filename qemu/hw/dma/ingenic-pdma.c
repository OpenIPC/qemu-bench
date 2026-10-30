/*
 * Ingenic PDMA: the T-series peripheral DMA controller at 0x13420000.
 *
 * 32 channels, channel n's registers at 0x20 n (mach/jzdma.h of the 3.10
 * kernels, drivers/dma/jzdma/jz_dma.c; drivers/dma/ingenic/ingenic_dma.h
 * of the 4.4 ones):
 *
 *   0x00 DSA  source address      0x04 DTA  target address
 *   0x08 DTC  transfer count      0x0C DRT  request type
 *   0x10 DCS  31 NDES (no descriptor), 30 DES8, 15:8 CDOA, 4 AR, 3 TT,
 *             2 HLT, 0 CTE (enable)
 *   0x14 DCM  23 SAI, 22 DAI, 19:16 RDIL, 15:14 SP, 13:12 DP (0 32-bit,
 *             1 8-bit, 2 16-bit), 10:8 TSZ, 1 TIE, 0 LINK
 *   0x18 DDA  descriptor address (31:12 page, 11:4 offset)
 *   0x1C DSD  stride
 *
 * and the controller's: 0x1000 DMAC (0 DMAE, 2 AR, 3 HLT), 0x1004 DIRQP
 * (channel n's interrupt pending; a write keeps only the bits written
 * as 1, so both the 3.10 drivers' 0 and the 4.4 ones' ~pending clear),
 * 0x1008 DDR / 0x100C DDRS (descriptor doorbells, DDRS sets), 0x1010 DIP /
 * 0x1014 DIC (descriptor interrupts pending / clear), 0x101C DMACP and
 * the MCU's 0x1030..0x103C, storage, as is TCSM (0x2000, the MCU's RAM).
 *
 * A channel runs when DMAC.DMAE and its DCS.CTE are set and AR / HLT
 * are not.  Without NDES it first fetches a descriptor at DDA (its
 * doorbell rung): dcm, dsa, dta, dtc (bits 31:24 the next descriptor's
 * offset in DDA's page), then with DES8 sd and drt.  TSZ codes the unit
 * DTC counts (0 4 bytes, 1 1, 2 2, 3 16, 4 32, 5 64, 6 128; 7 "auto",
 * the 4.4 drivers' choice, counts bytes).  At DTC 0 a descriptor with
 * LINK chains to the next one -- the drivers' cyclic transfers (audio)
 * are a ring of linked descriptors, which never end and never
 * interrupt; the hardware raises nothing for a linked descriptor (the
 * drivers' interrupt handlers stop the channel) -- and one without ends
 * the transfer: TT, and with TIE the channel's DIRQP bit and the
 * interrupt line.
 *
 * Request type 8 (AUTO) runs at once: a memory to memory copy.  Any
 * other is paced by the peripheral: the "drq" GPIO input of that number
 * (0..63) is high while the peripheral wants service -- the AIC's 5
 * (loop record), 6 (play) and 7 (record) -- and every access to the
 * peripheral's side waits for it.  An access is the port width of that
 * side (SP for a source without SAI, DP for a target without DAI).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_INGENIC_PDMA "ingenic-pdma"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicPdmaState, INGENIC_PDMA)

#define NCHAN           32
#define NREQ            64
#define PDMA_SIZE       0x10000

#define CH_DSA          0x00
#define CH_DTA          0x04
#define CH_DTC          0x08
#define CH_DRT          0x0C
#define CH_DCS          0x10
#define CH_DCM          0x14
#define CH_DDA          0x18
#define CH_DSD          0x1C

#define R_DMAC          0x1000
#define R_DIRQP         0x1004
#define R_DDR           0x1008
#define R_DDRS          0x100C
#define R_DIP           0x1010
#define R_DIC           0x1014
#define GLOBAL_REGS     0x40        /* 0x1000..0x103F */
#define TCSM_BASE       0x2000
#define TCSM_SIZE       0x2000

#define DCS_NDES        (1u << 31)
#define DCS_DES8        (1u << 30)
#define DCS_AR          (1u << 4)
#define DCS_TT          (1u << 3)
#define DCS_HLT         (1u << 2)
#define DCS_CTE         (1u << 0)

#define DCM_SAI         (1u << 23)
#define DCM_DAI         (1u << 22)
#define DCM_TIE         (1u << 1)
#define DCM_LINK        (1u << 0)

#define DMAC_DMAE       (1u << 0)
#define DMAC_AR         (1u << 2)
#define DMAC_HLT        (1u << 3)

#define DRT_AUTO        8

typedef struct IngenicPdmaChan {
    uint32_t dsa, dta, dtc, drt, dcs, dcm, dda, dsd;
    uint32_t unit_done;     /* bytes of the current unit moved */
    bool loaded;            /* a descriptor (or NDES registers) is live */
} IngenicPdmaChan;

struct IngenicPdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    IngenicPdmaChan ch[NCHAN];
    uint32_t global[GLOBAL_REGS / 4];
    uint8_t tcsm[TCSM_SIZE];
    uint32_t drq[NREQ / 32];
    bool busy;              /* inside pdma_run: no re-entry */
    bool again;             /* a request changed meanwhile */
};

static uint32_t *pdma_global(IngenicPdmaState *s, hwaddr off)
{
    return &s->global[(off - R_DMAC) >> 2];
}

static void pdma_update_irq(IngenicPdmaState *s)
{
    qemu_set_irq(s->irq, *pdma_global(s, R_DIRQP) != 0);
}

static unsigned pdma_port(uint32_t code)
{
    return code == 1 ? 1 : code == 2 ? 2 : 4;
}

/* the bytes DTC counts in; 0 for "auto" (DTC in bytes) */
static unsigned pdma_unit(uint32_t dcm)
{
    static const unsigned tsz[8] = { 4, 1, 2, 16, 32, 64, 128, 0 };

    return tsz[extract32(dcm, 8, 3)];
}

static bool pdma_drq(IngenicPdmaState *s, uint32_t drt)
{
    drt &= 0x3f;
    return drt == DRT_AUTO || (s->drq[drt / 32] >> (drt % 32)) & 1;
}

static bool pdma_fetch(IngenicPdmaState *s, IngenicPdmaChan *c)
{
    uint32_t d[8] = { 0 };
    int n = (c->dcs & DCS_DES8) ? 8 : 4;

    if (dma_memory_read(&address_space_memory, c->dda, d, n * 4,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        c->dcs |= DCS_AR;
        return false;
    }
    c->dcm = le32_to_cpu(d[0]);
    c->dsa = le32_to_cpu(d[1]);
    c->dta = le32_to_cpu(d[2]);
    c->dtc = le32_to_cpu(d[3]);
    if (n == 8) {
        c->dsd = le32_to_cpu(d[4]);
        c->drt = le32_to_cpu(d[5]);
    }
    c->dcs = deposit32(c->dcs, 8, 8, extract32(c->dtc, 24, 8));
    c->unit_done = 0;
    c->loaded = true;
    return true;
}

static bool pdma_runnable(IngenicPdmaState *s, IngenicPdmaChan *c)
{
    return (*pdma_global(s, R_DMAC) & DMAC_DMAE) &&
           !(*pdma_global(s, R_DMAC) & (DMAC_AR | DMAC_HLT)) &&
           (c->dcs & DCS_CTE) && !(c->dcs & (DCS_AR | DCS_HLT | DCS_TT));
}

/* the end of a descriptor: chain on, or end the transfer */
static bool pdma_desc_done(IngenicPdmaState *s, int n)
{
    IngenicPdmaChan *c = &s->ch[n];
    uint32_t doa = extract32(c->dtc, 24, 8);

    c->loaded = false;
    if ((c->dcm & DCM_LINK) && !(c->dcs & DCS_NDES)) {
        c->dda = (c->dda & 0xfffff000) | (doa << 4);
        if (c->dcm & DCM_TIE) {
            *pdma_global(s, R_DIP) |= 1u << n;
        }
        return pdma_fetch(s, c);
    }
    c->dcs |= DCS_TT;
    if (c->dcm & DCM_TIE) {
        *pdma_global(s, R_DIRQP) |= 1u << n;
        pdma_update_irq(s);
    }
    return false;
}

/*
 * Move what channel n can: the whole transfer for AUTO, otherwise as
 * long as the peripheral requests.  Returns whether anything moved.
 */
static bool pdma_run_chan(IngenicPdmaState *s, int n)
{
    IngenicPdmaChan *c = &s->ch[n];
    bool moved = false;
    int guard = 1 << 16;    /* peripheral accesses per call: a stuck ring */

    while (pdma_runnable(s, c) && guard > 0) {
        bool sai, dai, paced;
        unsigned sw, dw, unit, count;
        uint64_t left;
        uint32_t w;
        uint8_t buf[4096];

        if (!c->loaded) {
            if (c->dcs & DCS_NDES) {
                c->loaded = true;
            } else if (!pdma_fetch(s, c)) {
                break;
            }
        }
        count = extract32(c->dtc, 0, 24);
        if (!count) {
            if (!pdma_desc_done(s, n)) {
                break;
            }
            continue;
        }
        sai = c->dcm & DCM_SAI;
        dai = c->dcm & DCM_DAI;
        paced = (c->drt & 0x3f) != DRT_AUTO && !(sai && dai);
        if (paced && !pdma_drq(s, c->drt)) {
            break;
        }
        sw = pdma_port(extract32(c->dcm, 14, 2));
        dw = pdma_port(extract32(c->dcm, 12, 2));
        unit = pdma_unit(c->dcm);
        left = unit ? (uint64_t)count * unit - c->unit_done : count;
        if (paced) {
            /* one access of the peripheral side's width */
            w = !sai ? sw : dw;
            guard--;
        } else {
            /* memory to memory, or a fixed address at its port width */
            w = (sai && dai) ? sizeof(buf) : MIN(sw, dw);
        }
        w = MIN(w, left);
        if (dma_memory_read(&address_space_memory, c->dsa, buf, w,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK ||
            dma_memory_write(&address_space_memory, c->dta, buf, w,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            c->dcs |= DCS_AR;
            *pdma_global(s, R_DMAC) |= DMAC_AR;
            break;
        }
        moved = true;
        if (sai) {
            c->dsa += w;
        }
        if (dai) {
            c->dta += w;
        }
        if (unit) {
            c->unit_done += w;
            count -= c->unit_done / unit;
            c->unit_done %= unit;
        } else {
            count -= w;
        }
        c->dtc = deposit32(c->dtc, 0, 24, count);
    }
    return moved;
}

static void pdma_run(IngenicPdmaState *s)
{
    int n;

    if (s->busy) {
        s->again = true;
        return;
    }
    s->busy = true;
    do {
        s->again = false;
        for (n = 0; n < NCHAN; n++) {
            pdma_run_chan(s, n);
        }
    } while (s->again);
    s->busy = false;
}

static void pdma_set_drq(void *opaque, int line, int level)
{
    IngenicPdmaState *s = opaque;
    uint32_t bit = 1u << (line % 32);

    if (level) {
        s->drq[line / 32] |= bit;
        pdma_run(s);
    } else {
        s->drq[line / 32] &= ~bit;
    }
}

static uint64_t ingenic_pdma_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicPdmaState *s = opaque;

    if (off < NCHAN * 0x20) {
        IngenicPdmaChan *c = &s->ch[off / 0x20];

        switch (off % 0x20) {
        case CH_DSA: return c->dsa;
        case CH_DTA: return c->dta;
        case CH_DTC: return extract32(c->dtc, 0, 24);
        case CH_DRT: return c->drt;
        case CH_DCS: return c->dcs;
        case CH_DCM: return c->dcm;
        case CH_DDA: return c->dda;
        case CH_DSD: return c->dsd;
        }
    } else if (off >= R_DMAC && off < R_DMAC + GLOBAL_REGS) {
        return *pdma_global(s, off & ~3);
    } else if (off >= TCSM_BASE && off < TCSM_BASE + TCSM_SIZE) {
        return ldl_le_p(&s->tcsm[(off - TCSM_BASE) & ~3]);
    }
    return 0;
}

static void ingenic_pdma_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicPdmaState *s = opaque;

    if (off < NCHAN * 0x20) {
        int n = off / 0x20;
        IngenicPdmaChan *c = &s->ch[n];

        switch (off % 0x20) {
        case CH_DSA: c->dsa = val; break;
        case CH_DTA: c->dta = val; break;
        case CH_DTC: c->dtc = val & 0xffffff; break;
        case CH_DRT: c->drt = val; break;
        case CH_DCM: c->dcm = val; break;
        case CH_DDA: c->dda = val; break;
        case CH_DSD: c->dsd = val; break;
        case CH_DCS:
            if (!(val & DCS_CTE) || ((val ^ c->dcs) & DCS_NDES)) {
                c->loaded = false;      /* stopped: the next start fetches */
            }
            if ((val & DCS_CTE) && !(c->dcs & DCS_CTE) &&
                !(val & DCS_NDES)) {
                /* a start in descriptor mode needs its doorbell */
                if (!(*pdma_global(s, R_DDR) & (1u << n))) {
                    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-pdma: channel %d"
                                  " started without its doorbell\n", n);
                }
                *pdma_global(s, R_DDR) &= ~(1u << n);
                c->loaded = false;
            }
            c->dcs = val;
            pdma_run(s);
            break;
        }
        return;
    }
    if (off >= TCSM_BASE && off < TCSM_BASE + TCSM_SIZE) {
        stl_le_p(&s->tcsm[(off - TCSM_BASE) & ~3], val);
        return;
    }
    if (off < R_DMAC || off >= R_DMAC + GLOBAL_REGS) {
        return;
    }
    switch (off & ~3) {
    case R_DMAC:
        *pdma_global(s, R_DMAC) = val;
        pdma_run(s);
        break;
    case R_DIRQP:
        *pdma_global(s, R_DIRQP) &= val;
        pdma_update_irq(s);
        break;
    case R_DDR:
        break;
    case R_DDRS:
        *pdma_global(s, R_DDR) |= val;
        break;
    case R_DIC:
        *pdma_global(s, R_DIP) &= ~val;
        break;
    default:
        *pdma_global(s, off & ~3) = val;
        break;
    }
}

static const MemoryRegionOps ingenic_pdma_ops = {
    .read = ingenic_pdma_read,
    .write = ingenic_pdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ingenic_pdma_reset(DeviceState *dev)
{
    IngenicPdmaState *s = INGENIC_PDMA(dev);

    memset(s->ch, 0, sizeof(s->ch));
    memset(s->global, 0, sizeof(s->global));
    s->busy = s->again = false;
    pdma_update_irq(s);
}

static void ingenic_pdma_init(Object *obj)
{
    IngenicPdmaState *s = INGENIC_PDMA(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_pdma_ops, s,
                          TYPE_INGENIC_PDMA, PDMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), pdma_set_drq, "drq", NREQ);
}

static const VMStateDescription vmstate_ingenic_pdma_chan = {
    .name = "ingenic-pdma-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dsa, IngenicPdmaChan),
        VMSTATE_UINT32(dta, IngenicPdmaChan),
        VMSTATE_UINT32(dtc, IngenicPdmaChan),
        VMSTATE_UINT32(drt, IngenicPdmaChan),
        VMSTATE_UINT32(dcs, IngenicPdmaChan),
        VMSTATE_UINT32(dcm, IngenicPdmaChan),
        VMSTATE_UINT32(dda, IngenicPdmaChan),
        VMSTATE_UINT32(dsd, IngenicPdmaChan),
        VMSTATE_UINT32(unit_done, IngenicPdmaChan),
        VMSTATE_BOOL(loaded, IngenicPdmaChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_pdma = {
    .name = TYPE_INGENIC_PDMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, IngenicPdmaState, NCHAN, 1,
                             vmstate_ingenic_pdma_chan, IngenicPdmaChan),
        VMSTATE_UINT32_ARRAY(global, IngenicPdmaState, GLOBAL_REGS / 4),
        VMSTATE_UINT8_ARRAY(tcsm, IngenicPdmaState, TCSM_SIZE),
        VMSTATE_UINT32_ARRAY(drq, IngenicPdmaState, NREQ / 32),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_pdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_pdma_reset);
    dc->vmsd = &vmstate_ingenic_pdma;
}

static const TypeInfo ingenic_pdma_info = {
    .name = TYPE_INGENIC_PDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicPdmaState),
    .instance_init = ingenic_pdma_init,
    .class_init = ingenic_pdma_class_init,
};

static void ingenic_pdma_register_types(void)
{
    type_register_static(&ingenic_pdma_info);
}

type_init(ingenic_pdma_register_types)
