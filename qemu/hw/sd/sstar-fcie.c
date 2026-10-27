/*
 * MStar infinity3 SD host: SDIO0, an FCIE5 ("SDIO" port), the SD slot.
 *
 * Read off the vendor U-Boot's drivers/mstar/sdmmc (hal_sdmmc_v5.c,
 * hal_card_regs_v5.h, hal_card_platform_iNF3.c, ms_sdmmc_ub.c; the I3 SDK
 * MSC3XX_SDK_V_1711291828), which the kernel's sdmmc driver shares.  RIU
 * banks of 16-bit registers on a 4-byte stride, read a byte at a time
 * where the HAL takes the response bytes.
 *
 * Bank 0x1410 (0x1F282000), registers:
 *   0x00 MIE_EVENT  DATA_END 0, CMD_END 1, ERR_STS 2, BUSY_END 4; written
 *                   ones clear
 *   0x01 MIE_INT_EN the MIE_EVENT layout; the interrupt (INT_IRQ_SDIO) is
 *                   MIE_EVENT & MIE_INT_EN
 *   0x03/0x04 DMA_ADDR (MIU address, DDR from 0)  0x05/0x06 DMA_LEN
 *   0x07 MIE_FUNC_CTL  0x08 JOB_BLK_CNT  0x09 BLK_SIZE
 *   0x0A CMD_RSP_SIZE  [13:8] command bytes (5), [7:0] response bytes
 *   0x0B SD_MODE    CLK_EN 0, 4-bit 1, 8-bit 2, DEST_R2N 4 (data through
 *                   the CIFD buffer, not DMA), DMA_RD_CLK_STOP 7
 *   0x0C SD_CTL     RSPR2_EN 0, RSP_EN 1, CMD_EN 2, DTRX_EN 3, JOB_DIR 4
 *                   (1 = write), ADMA_EN 5, JOB_START 6 (runs the job)
 *   0x0D SD_STS     RD_CERR 0, WR_CERR 1, WR_TOUT 2, CMD_NORSP 3,
 *                   CMDRSP_CERR 4, RD_TOUT 5, written ones clear; DAT0-3
 *                   levels in [11:8], high: the card is never busy here
 *   0x20.. CMD/RSP FIFO, two bytes per register, low byte first: the
 *                   command as 0x40 | index then the argument MSB first;
 *                   the response replaces it, its first byte the index
 *                   (0x3F for R2/R3), then MSB first; R2 is 0x3F and CID
 *                   or CSD bits 127..8 (ms_sdmmc_ub.c mmc_send_cmd)
 *   0x30 CIFD_EVENT WBUF_FULL 0, WBUF_EMPTY_TRIG 1, RBUF_FULL_TRIG 2,
 *                   RBUF_EMPTY 3
 *   0x3F FCIE_RST   bit 0 the reset (active low); [3:1], the MIU, MIE and
 *                   MCU reset states, follow it inverted (_REG_ResetIP)
 * Bank 0x1411: the CIFD, a 64-byte buffer: the read half at regs
 *   0x00-0x1F, the write half at 0x20-0x3F.
 *
 * A job (Hal_SDMMC_SendCmdAndWaitProcess) runs when SD_CTL is written
 * with JOB_START: with CMD_EN the command goes to the card on the SD bus
 * and CMD_END is raised (with CMD_NORSP and ERR_STS when a response was
 * asked for and none came); with DTRX_EN the data moves, BLK_SIZE *
 * JOB_BLK_CNT bytes between the card and DDR at DMA_ADDR, and DATA_END is
 * raised.  A read's command and data are one job; a write's data is a
 * second job (JOB_DIR, DTRX_EN without CMD_EN) once the command is
 * answered.  JOB_START reads back clear: the job is done.
 *
 * DEST_R2N moves the data through the CIFD instead, 64 bytes at a time
 * (_BUF_CIFD_WaitEvent): a read fills the buffer and raises WBUF_FULL,
 * the driver takes it and writes WBUF_EMPTY_TRIG for the next 64; a
 * write is taken from the buffer on RBUF_FULL_TRIG, which raises
 * RBUF_EMPTY.  ADMA (descriptor tables) is not modelled.
 *
 * The card detect, PM_SD_CDZ, is in sstar-i3-gpio: the machine sets its
 * "sd-card" when a card is attached.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_SSTAR_FCIE "sstar-fcie"
OBJECT_DECLARE_SIMPLE_TYPE(SstarFcieState, SSTAR_FCIE)

#define FCIE_REGS           0x80
#define FCIE_SIZE           0x400       /* banks 0x1410 and 0x1411 */
#define DRAM_BASE           0x20000000

#define R_MIE_EVENT         0x00
#define R_MIE_INT_EN        0x01
#define R_DMA_ADDR_L        0x03
#define R_DMA_ADDR_H        0x04
#define R_DMA_LEN_L         0x05
#define R_DMA_LEN_H         0x06
#define R_JOB_BLK_CNT       0x08
#define R_BLK_SIZE          0x09
#define R_CMD_RSP_SIZE      0x0A
#define R_SD_MODE           0x0B
#define R_SD_CTL            0x0C
#define R_SD_STS            0x0D
#define R_CFIFO             0x20
#define R_CIFD_EVENT        0x30
#define R_FCIE_RST          0x3F

#define MIE_DATA_END        (1u << 0)
#define MIE_CMD_END         (1u << 1)
#define MIE_ERR_STS         (1u << 2)
#define MIE_EVENTS          0x003F

#define MODE_DEST_R2N       (1u << 4)

#define CTL_RSPR2_EN        (1u << 0)
#define CTL_RSP_EN          (1u << 1)
#define CTL_CMD_EN          (1u << 2)
#define CTL_DTRX_EN         (1u << 3)
#define CTL_JOB_DIR         (1u << 4)
#define CTL_ADMA_EN         (1u << 5)
#define CTL_JOB_START       (1u << 6)

#define STS_WR_TOUT         (1u << 2)
#define STS_CMD_NORSP       (1u << 3)
#define STS_RD_TOUT         (1u << 5)
#define STS_ERRORS          0x003F
#define STS_DAT_HIGH        0x0F00      /* DAT0-3 */

#define CIFD_WBUF_FULL      (1u << 0)
#define CIFD_WBUF_EMPTY_TRIG (1u << 1)
#define CIFD_RBUF_FULL_TRIG (1u << 2)
#define CIFD_RBUF_EMPTY     (1u << 3)

#define CIFD_BYTES          64

#define RST_N               (1u << 0)
#define RST_STATES          0x000E

struct SstarFcieState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;

    uint16_t regs[FCIE_REGS];
    uint8_t cifd[2][CIFD_BYTES];        /* [0] read half, [1] write half */
    uint32_t r2n_left;                  /* CIFD transfer bytes to go */
    bool r2n_write;
};

static void fcie_update_irq(SstarFcieState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_MIE_EVENT] & s->regs[R_MIE_INT_EN] &
                            MIE_EVENTS));
}

static uint8_t fcie_fifo_get(SstarFcieState *s, unsigned i)
{
    uint16_t v = s->regs[R_CFIFO + i / 2];

    return i & 1 ? v >> 8 : v & 0xFF;
}

static void fcie_fifo_put(SstarFcieState *s, unsigned i, uint8_t b)
{
    uint16_t *r = &s->regs[R_CFIFO + i / 2];

    *r = i & 1 ? (*r & 0x00FF) | (b << 8) : (*r & 0xFF00) | b;
}

static uint32_t fcie_job_len(SstarFcieState *s)
{
    return (uint32_t)s->regs[R_BLK_SIZE] * s->regs[R_JOB_BLK_CNT];
}

static hwaddr fcie_dma_addr(SstarFcieState *s)
{
    return DRAM_BASE + (s->regs[R_DMA_ADDR_L] |
                        ((uint32_t)s->regs[R_DMA_ADDR_H] << 16));
}

/* The command in the FIFO to the card, its response back into the FIFO */
static void fcie_command(SstarFcieState *s, uint16_t ctl)
{
    SDRequest req = {
        .cmd = fcie_fifo_get(s, 0) & 0x3F,
        .arg = (uint32_t)fcie_fifo_get(s, 1) << 24 |
               fcie_fifo_get(s, 2) << 16 | fcie_fifo_get(s, 3) << 8 |
               fcie_fifo_get(s, 4),
    };
    uint8_t r[16];
    int n, i;

    n = sdbus_do_command(&s->sdbus, &req, r, sizeof(r));
    s->regs[R_MIE_EVENT] |= MIE_CMD_END;
    if (!(ctl & CTL_RSP_EN)) {
        return;
    }
    if (n <= 0 || ((ctl & CTL_RSPR2_EN) ? n != 16 : n != 4)) {
        s->regs[R_SD_STS] |= STS_CMD_NORSP;
        s->regs[R_MIE_EVENT] |= MIE_ERR_STS;
        return;
    }
    /* R2: 0x3F, then bits 127..8; the others: the index, then 32 bits */
    fcie_fifo_put(s, 0, n == 16 ? 0x3F : req.cmd);
    for (i = 0; i < (n == 16 ? 15 : 4); i++) {
        fcie_fifo_put(s, 1 + i, r[i]);
    }
}

static void fcie_cifd_fill(SstarFcieState *s)
{
    uint32_t n = MIN(s->r2n_left, CIFD_BYTES);

    memset(s->cifd[0], 0, CIFD_BYTES);
    sdbus_read_data(&s->sdbus, s->cifd[0], n);
    s->r2n_left -= n;
    s->regs[R_CIFD_EVENT] |= CIFD_WBUF_FULL;
    /* The card has sent it all: the last buffer is the driver's to take */
    if (!s->r2n_left) {
        s->regs[R_MIE_EVENT] |= MIE_DATA_END;
    }
}

/*
 * A DMA job, a chunk at a time: the guest sets up to 0xFFFF blocks of
 * 0xFFFF bytes. It ends in a timeout, as the host's would, when the card
 * stops sending or taking data before the job's length, or when the DMA
 * finds nothing at the address.
 */
static void fcie_dma(SstarFcieState *s, uint32_t len, bool write)
{
    uint8_t buf[4096];
    hwaddr addr = fcie_dma_addr(s);
    uint32_t n;
    MemTxResult r;

    for (; len; len -= n, addr += n) {
        n = MIN(len, sizeof(buf));
        if (write) {
            r = dma_memory_read(&address_space_memory, addr, buf, n,
                                MEMTXATTRS_UNSPECIFIED);
            if (r != MEMTX_OK || !sdbus_receive_ready(&s->sdbus)) {
                break;
            }
            sdbus_write_data(&s->sdbus, buf, n);
        } else {
            if (!sdbus_data_ready(&s->sdbus)) {
                break;
            }
            sdbus_read_data(&s->sdbus, buf, n);
            r = dma_memory_write(&address_space_memory, addr, buf, n,
                                 MEMTXATTRS_UNSPECIFIED);
            if (r != MEMTX_OK) {
                break;
            }
        }
    }
    if (len) {
        qemu_log_mask(LOG_GUEST_ERROR, "sstar-fcie: %s job stopped with "
                      "0x%x bytes left at 0x%" HWADDR_PRIx "\n",
                      write ? "write" : "read", len, addr);
        s->regs[R_SD_STS] |= write ? STS_WR_TOUT : STS_RD_TOUT;
        s->regs[R_MIE_EVENT] |= MIE_ERR_STS;
        return;
    }
    s->regs[R_MIE_EVENT] |= MIE_DATA_END;
}

static void fcie_data(SstarFcieState *s, uint16_t ctl)
{
    uint32_t len = fcie_job_len(s);
    bool write = ctl & CTL_JOB_DIR;

    if (ctl & CTL_ADMA_EN) {
        qemu_log_mask(LOG_UNIMP, "sstar-fcie: ADMA job not modelled\n");
        s->regs[R_SD_STS] |= write ? STS_WR_TOUT : STS_RD_TOUT;
        s->regs[R_MIE_EVENT] |= MIE_ERR_STS;
        return;
    }
    if (!len) {
        s->regs[R_MIE_EVENT] |= MIE_DATA_END;
        return;
    }
    if (write ? !sdbus_receive_ready(&s->sdbus)
              : !sdbus_data_ready(&s->sdbus)) {
        s->regs[R_SD_STS] |= write ? STS_WR_TOUT : STS_RD_TOUT;
        s->regs[R_MIE_EVENT] |= MIE_ERR_STS;
        return;
    }
    if (s->regs[R_SD_MODE] & MODE_DEST_R2N) {
        /* Through the CIFD: the driver moves it 64 bytes at a time */
        s->r2n_left = len;
        s->r2n_write = write;
        if (!write) {
            fcie_cifd_fill(s);
        }
        return;
    }
    fcie_dma(s, len, write);
}

static void fcie_job(SstarFcieState *s, uint16_t ctl)
{
    if (!(s->regs[R_FCIE_RST] & RST_N)) {
        return;                         /* held in reset */
    }
    if (ctl & CTL_CMD_EN) {
        fcie_command(s, ctl);
        if (s->regs[R_SD_STS] & STS_ERRORS) {
            return;
        }
    }
    if (ctl & CTL_DTRX_EN) {
        fcie_data(s, ctl);
    }
}

/* CIFD handshakes of a DEST_R2N transfer */
static void fcie_cifd_event(SstarFcieState *s, uint16_t val)
{
    s->regs[R_CIFD_EVENT] &= ~(val & (CIFD_WBUF_FULL | CIFD_RBUF_EMPTY));
    if (!s->r2n_left) {
        return;
    }
    if (!s->r2n_write && (val & CIFD_WBUF_EMPTY_TRIG)) {
        fcie_cifd_fill(s);
    } else if (s->r2n_write && (val & CIFD_RBUF_FULL_TRIG)) {
        uint32_t n = MIN(s->r2n_left, CIFD_BYTES);

        sdbus_write_data(&s->sdbus, s->cifd[1], n);
        s->r2n_left -= n;
        s->regs[R_CIFD_EVENT] |= CIFD_RBUF_EMPTY;
    } else {
        return;
    }
    if (!s->r2n_left) {
        s->regs[R_MIE_EVENT] |= MIE_DATA_END;
    }
}

static uint16_t fcie_read16(SstarFcieState *s, unsigned bank, unsigned reg)
{
    uint16_t v;

    if (bank) {
        const uint8_t *p = s->cifd[reg >= 0x20] + (reg & 0x1F) * 2;

        return p[0] | p[1] << 8;
    }
    v = s->regs[reg];
    switch (reg) {
    case R_SD_STS:
        v = (v & STS_ERRORS) | STS_DAT_HIGH;
        break;
    case R_SD_CTL:
        v &= ~CTL_JOB_START;
        break;
    case R_FCIE_RST:
        v = (v & ~RST_STATES) | ((v & RST_N) ? 0 : RST_STATES);
        break;
    }
    return v;
}

static void fcie_write16(SstarFcieState *s, unsigned bank, unsigned reg,
                         uint16_t val)
{
    if (bank) {
        uint8_t *p = s->cifd[reg >= 0x20] + (reg & 0x1F) * 2;

        p[0] = val;
        p[1] = val >> 8;
        return;
    }
    switch (reg) {
    case R_MIE_EVENT:
        s->regs[reg] &= ~val;
        break;
    case R_SD_STS:
        s->regs[reg] &= ~(val & STS_ERRORS);
        break;
    case R_CIFD_EVENT:
        fcie_cifd_event(s, val);
        break;
    case R_SD_CTL:
        s->regs[reg] = val & ~CTL_JOB_START;
        if (val & CTL_JOB_START) {
            fcie_job(s, val);
        }
        break;
    case R_FCIE_RST:
        s->regs[reg] = val;
        if (!(val & RST_N)) {
            s->regs[R_MIE_EVENT] = 0;
            s->regs[R_SD_STS] = 0;
            s->regs[R_CIFD_EVENT] = 0;
            s->r2n_left = 0;
        }
        break;
    default:
        s->regs[reg] = val;
        break;
    }
    fcie_update_irq(s);
}

static uint64_t fcie_read(void *opaque, hwaddr off, unsigned size)
{
    SstarFcieState *s = opaque;
    uint16_t v;

    if ((off & 3) >= 2) {
        return 0;
    }
    v = fcie_read16(s, off >> 9, (off >> 2) & 0x7F);
    if (off & 1) {
        return v >> 8;
    }
    return size == 1 ? (v & 0xFF) : v;
}

static void fcie_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    SstarFcieState *s = opaque;
    unsigned bank = off >> 9, reg = (off >> 2) & 0x7F;
    uint16_t v;

    if ((off & 3) >= 2) {
        return;
    }
    if (size == 1) {
        /* A byte lane: the rest of the register as it reads, not W1C bits */
        v = bank ? fcie_read16(s, bank, reg) : s->regs[reg];
        if (!bank && (reg == R_MIE_EVENT || reg == R_SD_STS ||
                      reg == R_CIFD_EVENT || reg == R_SD_CTL)) {
            v = 0;
        }
        v = (off & 1) ? (v & 0x00FF) | (val & 0xFF) << 8
                      : (v & 0xFF00) | (val & 0xFF);
    } else {
        v = val;
    }
    fcie_write16(s, bank, reg, v);
}

static const MemoryRegionOps fcie_ops = {
    .read = fcie_read,
    .write = fcie_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void fcie_reset(DeviceState *dev)
{
    SstarFcieState *s = SSTAR_FCIE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->cifd, 0, sizeof(s->cifd));
    s->regs[R_FCIE_RST] = RST_N;
    s->r2n_left = 0;
    s->r2n_write = false;
    fcie_update_irq(s);
}

static void fcie_init(Object *obj)
{
    SstarFcieState *s = SSTAR_FCIE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &fcie_ops, s, TYPE_SSTAR_FCIE,
                          FCIE_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(obj),
              "sd-bus");
}

static int fcie_post_load(void *opaque, int version_id)
{
    fcie_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_fcie = {
    .name = TYPE_SSTAR_FCIE,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = fcie_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SstarFcieState, FCIE_REGS),
        VMSTATE_UINT8_2DARRAY(cifd, SstarFcieState, 2, CIFD_BYTES),
        VMSTATE_UINT32(r2n_left, SstarFcieState),
        VMSTATE_BOOL(r2n_write, SstarFcieState),
        VMSTATE_END_OF_LIST()
    }
};

static void fcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_fcie;
    device_class_set_legacy_reset(dc, fcie_reset);
}

static const TypeInfo fcie_info = {
    .name          = TYPE_SSTAR_FCIE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SstarFcieState),
    .instance_init = fcie_init,
    .class_init    = fcie_class_init,
};

static void fcie_register_types(void)
{
    type_register_static(&fcie_info);
}

type_init(fcie_register_types)
