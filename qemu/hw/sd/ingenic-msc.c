/*
 * Ingenic MSC: the T-series SD/MMC/SDIO host at 0x13450000 (MSC0, INTC
 * 37) and 0x13460000 (MSC1, INTC 36) on T10..T31, the jz4780 block with
 * descriptor DMA ("SDMA").  Registers (kernel drivers/mmc/host/
 * jzmmc_v12.h, U-Boot arch/mips/include/asm/arch-t31/mmc.h):
 *
 *   CTRL 0x00   actions, read as 0: 3 RESET, 2 START_OP, 1:0 clock
 *               (1 stop, 2 start), 7/6 EXIT_MULTIPLE/EXIT_TRANSFER
 *   STAT 0x04   31 AUTO_CMD12_DONE, 16 DMAEND, 15 IS_RESETTING,
 *               13 PRG_DONE, 12 DATA_TRAN_DONE, 11 END_CMD_RES, 8 CLK_EN,
 *               7 FIFO_FULL, 6 FIFO_EMPTY, 5 CRC_RES, 4 CRC_READ,
 *               3:2 CRC_WRITE, 1 TIME_OUT_RES, 0 TIME_OUT_READ
 *   CLKRT 0x08, CMDAT 0x0C (16 AUTO_CMD12, 10:9 bus width, 7 INIT,
 *               6 BUSY, 5 STREAM, 4 WRITE, 3 DATA_EN, 2:0 response
 *               format: 0 none, 1 R1/R1b, 2 R2, 3 R3, ...)
 *   RESTO 0x10, RDTO 0x14, BLKLEN 0x18, NOB 0x1C, SNOB 0x20
 *   IMASK 0x24 (1 = masked), IFLG 0x28 (write 1 to clear): 31
 *               DMA_DATA_DONE, 23 WR_ALL_DONE, 16 DMAEND, 15
 *               AUTO_CMD12_DONE, 12/11/10 CRC res/read/write, 9
 *               TIMEOUT_RES, 8 TIMEOUT_READ, 7 SDIO, 6 TXFIFO_WR_REQ,
 *               5 RXFIFO_RD_REQ, 2 END_CMD_RES, 1 PRG_DONE, 0
 *               DATA_TRAN_DONE
 *   CMD 0x2C, ARG 0x30, RES 0x34 (16-bit pops), RXFIFO 0x38,
 *   TXFIFO 0x3C, LPM 0x40, DMAC 0x44 (0 DMAEN), DMANDA 0x48 (first
 *   descriptor), DMADA/DMALEN/DMACMD 0x4C-0x54 (current descriptor),
 *   CTRL2 0x58, RTCNT 0x5C, DEBUG 0xFC
 *
 * RES holds the response as it came off the CMD line, header byte first:
 * three halfwords for a short one, the last carrying the status' low byte
 * in bits 7:0 (both drivers assemble h0 << 24 | h1 << 8 | (h2 & 0xff));
 * nine for R2, which the drivers read as (a << 24 | b << 8 | c >> 8)
 * with a = the previous c.
 *
 * A DMA descriptor is { nda, da, len, dcmd } (dcmd 0 LINK, 1 ENDI);
 * the kernel (jzmmc_v12.c, CONFIG_JZMMC_V12_SDMA=y on every T10..T31
 * OpenIPC kernel) writes the data setup and START_OP first and DMANDA,
 * then DMAC = DMAEN, last, and never clears DMAEN between requests.  So
 * a data phase goes through DMA when DMAC.DMAEN is written while it is
 * pending, or at START_OP if DMANDA was written since the last DMA;
 * otherwise it is PIO through RXFIFO/TXFIFO, which the kernel uses for
 * transfers under 64 bytes (the SCR) and U-Boot for everything.
 *
 * The card answers at once: START_OP runs the command and sets
 * END_CMD_RES, or TIME_OUT_RES when a response was expected and none
 * came -- every command with no card, so U-Boot's unbounded IFLG poll
 * (jz_mmc.c jz_mmc_send_cmd) ends and "mmc" init fails fast.  The data
 * phase finishes when the last FIFO word moves or within the DMAC write:
 * the auto CMD12 (STAT/IFLG AUTO_CMD12_DONE), then DATA_TRAN_DONE, and
 * DMAEND + DMA_DATA_DONE for a DMA read, PRG_DONE + WR_ALL_DONE for a
 * write.  A write does not raise DMA_DATA_DONE: the driver never clears
 * it on that path and a stale one would end the next read early.
 * START_OP clears the STAT completion and error bits, clearing an IFLG
 * bit clears its STAT twin.  The interrupt is level, IFLG & ~IMASK, with
 * TXFIFO_WR_REQ / RXFIFO_RD_REQ live while PIO data is pending.  Reset
 * is instant (IS_RESETTING reads 0).
 *
 * Card detect is a GPIO on the boards (MSC0: PB27 low = card, PB20 on
 * the T21 "Turkey" board; isvp/<board>/board.h GPIO_MMC_CD_N), not an
 * MSC register: the named output "cd" is 0 while a card is inserted and
 * 1 otherwise, for the machine to wire to that GPIO input.
 *
 * Not modelled: CRC errors, read-wait, stream mode, boot mode, CE-ATA,
 * auto CMD23, SDIO interrupts, timing (RESTO/RDTO/CLKRT are stored).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-core.h"
#include "hw/sd/sd.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define TYPE_INGENIC_MSC "ingenic-msc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMscState, INGENIC_MSC)

#define TYPE_INGENIC_MSC_BUS "ingenic-msc-bus"

#define R_CTRL          0x00
#define R_STAT          0x04
#define R_CLKRT         0x08
#define R_CMDAT         0x0C
#define R_RESTO         0x10
#define R_RDTO          0x14
#define R_BLKLEN        0x18
#define R_NOB           0x1C
#define R_SNOB          0x20
#define R_IMASK         0x24
#define R_IFLG          0x28
#define R_CMD           0x2C
#define R_ARG           0x30
#define R_RES           0x34
#define R_RXFIFO        0x38
#define R_TXFIFO        0x3C
#define R_LPM           0x40
#define R_DMAC          0x44
#define R_DMANDA        0x48
#define R_DMADA         0x4C
#define R_DMALEN        0x50
#define R_DMACMD        0x54
#define R_CTRL2         0x58
#define R_RTCNT         0x5C
#define R_DEBUG         0xFC

#define CTRL_EXIT_MULTIPLE  (1u << 7)
#define CTRL_EXIT_TRANSFER  (1u << 6)
#define CTRL_READWAIT       (3u << 4)
#define CTRL_RESET          (1u << 3)
#define CTRL_START_OP       (1u << 2)
#define CTRL_CLOCK(c)       extract32(c, 0, 2)

#define STAT_AUTO_CMD12_DONE (1u << 31)
#define STAT_DMAEND         (1u << 16)
#define STAT_PRG_DONE       (1u << 13)
#define STAT_DATA_TRAN_DONE (1u << 12)
#define STAT_END_CMD_RES    (1u << 11)
#define STAT_CLK_EN         (1u << 8)
#define STAT_FIFO_EMPTY     (1u << 6)
#define STAT_CRC_WRITE_NOSTS (2u << 2)
#define STAT_TIME_OUT_RES   (1u << 1)
#define STAT_TIME_OUT_READ  (1u << 0)
#define STAT_ERRORS         0x3fu
#define STAT_CLEAR_ON_START (STAT_AUTO_CMD12_DONE | STAT_DMAEND | \
                             STAT_PRG_DONE | STAT_DATA_TRAN_DONE | \
                             STAT_END_CMD_RES | STAT_ERRORS)

#define CMDAT_CEATA         (3u << 30)
#define CMDAT_BOOT          (0xfu << 24)
#define CMDAT_AUTO_CMD23    (1u << 18)
#define CMDAT_AUTO_CMD12    (1u << 16)
#define CMDAT_BUSY          (1u << 6)
#define CMDAT_STREAM        (1u << 5)
#define CMDAT_WRITE         (1u << 4)
#define CMDAT_DATA_EN       (1u << 3)
#define CMDAT_RESPONSE(c)   extract32(c, 0, 3)

#define IFLG_DMA_DATA_DONE  (1u << 31)
#define IFLG_WR_ALL_DONE    (1u << 23)
#define IFLG_DMAEND         (1u << 16)
#define IFLG_AUTO_CMD12_DONE (1u << 15)
#define IFLG_CRC_RES_ERR    (1u << 12)
#define IFLG_CRC_READ_ERR   (1u << 11)
#define IFLG_CRC_WRITE_ERR  (1u << 10)
#define IFLG_TIMEOUT_RES    (1u << 9)
#define IFLG_TIMEOUT_READ   (1u << 8)
#define IFLG_TXFIFO_WR_REQ  (1u << 6)
#define IFLG_RXFIFO_RD_REQ  (1u << 5)
#define IFLG_END_CMD_RES    (1u << 2)
#define IFLG_PRG_DONE       (1u << 1)
#define IFLG_DATA_TRAN_DONE (1u << 0)

#define DMAC_DMAEN          (1u << 0)
#define DMACMD_LINK         (1u << 0)

#define RES_HALFWORDS       9
#define DMA_MAX_DESC        4096    /* a runaway chain stops here */

enum { DATA_NONE, DATA_READ, DATA_WRITE };

struct IngenicMscState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDBus sdbus;
    qemu_irq irq;
    qemu_irq cd;

    uint32_t stat;          /* sticky bits; CLK_EN and FIFO are derived */
    uint32_t clkrt, cmdat, resto, rdto, blklen, nob, snob;
    uint32_t imask, iflg, cmd, arg, lpm;
    uint32_t dmac, dmanda, dmada, dmalen, dmacmd;
    uint32_t ctrl2, rtcnt, debug;
    bool clk_en;

    uint16_t res[RES_HALFWORDS];
    uint32_t res_len, res_pos;

    uint32_t data_dir;      /* DATA_* of the pending data phase */
    uint32_t data_left;     /* bytes still to move */
    bool auto_cmd12;
    bool dma_armed;         /* DMANDA written since the last DMA */
};

/* IFLG bit -> STAT twin, cleared together */
static const uint32_t iflg_stat[][2] = {
    { IFLG_END_CMD_RES, STAT_END_CMD_RES },
    { IFLG_PRG_DONE, STAT_PRG_DONE },
    { IFLG_DATA_TRAN_DONE, STAT_DATA_TRAN_DONE },
    { IFLG_TIMEOUT_RES, STAT_TIME_OUT_RES },
    { IFLG_TIMEOUT_READ, STAT_TIME_OUT_READ },
    { IFLG_CRC_RES_ERR, 1u << 5 },
    { IFLG_CRC_READ_ERR, 1u << 4 },
    { IFLG_CRC_WRITE_ERR, 3u << 2 },
    { IFLG_AUTO_CMD12_DONE, STAT_AUTO_CMD12_DONE },
    { IFLG_DMAEND, STAT_DMAEND },
};

static uint32_t msc_iflg(IngenicMscState *s)
{
    uint32_t v = s->iflg;

    if (s->data_left) {
        if (s->data_dir == DATA_READ) {
            v |= IFLG_RXFIFO_RD_REQ;
        } else if (s->data_dir == DATA_WRITE) {
            v |= IFLG_TXFIFO_WR_REQ;
        }
    }
    return v;
}

static uint32_t msc_stat(IngenicMscState *s)
{
    uint32_t v = s->stat;

    if (s->clk_en) {
        v |= STAT_CLK_EN;
    }
    if (s->data_dir != DATA_READ || !s->data_left) {
        v |= STAT_FIFO_EMPTY;
    }
    return v;
}

static void msc_update_irq(IngenicMscState *s)
{
    qemu_set_irq(s->irq, !!(msc_iflg(s) & ~s->imask));
}

static void msc_flag(IngenicMscState *s, uint32_t stat, uint32_t iflg)
{
    s->stat |= stat;
    s->iflg |= iflg;
}

static void msc_data_abort(IngenicMscState *s)
{
    s->data_dir = DATA_NONE;
    s->data_left = 0;
    s->auto_cmd12 = false;
}

static void msc_data_done(IngenicMscState *s, bool dma)
{
    bool write = s->data_dir == DATA_WRITE;

    if (s->auto_cmd12) {
        SDRequest req = { .cmd = 12, .arg = 0 };
        uint8_t rsp[16];

        sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
        msc_flag(s, STAT_AUTO_CMD12_DONE, IFLG_AUTO_CMD12_DONE);
    }
    s->snob = s->nob;
    msc_flag(s, STAT_DATA_TRAN_DONE, IFLG_DATA_TRAN_DONE);
    if (dma) {
        msc_flag(s, STAT_DMAEND,
                 IFLG_DMAEND | (write ? 0 : IFLG_DMA_DATA_DONE));
    }
    if (write) {
        msc_flag(s, STAT_PRG_DONE, IFLG_PRG_DONE | IFLG_WR_ALL_DONE);
    }
    msc_data_abort(s);
}

static void msc_run_dma(IngenicMscState *s)
{
    AddressSpace *as = &address_space_memory;
    bool write = s->data_dir == DATA_WRITE;
    uint32_t desc = s->dmanda;
    uint8_t buf[4096];
    int n;

    s->dma_armed = false;
    for (n = 0; n < DMA_MAX_DESC && s->data_left; n++) {
        uint32_t d[4];
        uint32_t len, full, da;

        if (dma_memory_read(as, desc, d, sizeof(d),
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: bad DMA "
                          "descriptor address 0x%08x\n", desc);
            break;
        }
        da = le32_to_cpu(d[1]);
        full = le32_to_cpu(d[2]);
        len = MIN(full, s->data_left);
        s->dmalen = full - len;
        s->dmacmd = le32_to_cpu(d[3]);
        while (len) {
            uint32_t chunk = MIN(len, sizeof(buf));

            if (write) {
                dma_memory_read(as, da, buf, chunk, MEMTXATTRS_UNSPECIFIED);
                sdbus_write_data(&s->sdbus, buf, chunk);
            } else {
                sdbus_read_data(&s->sdbus, buf, chunk);
                dma_memory_write(as, da, buf, chunk, MEMTXATTRS_UNSPECIFIED);
            }
            da += chunk;
            len -= chunk;
            s->data_left -= chunk;
        }
        s->dmada = da;
        if (!(s->dmacmd & DMACMD_LINK)) {
            break;
        }
        desc = le32_to_cpu(d[0]);
    }
    if (s->data_left) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: DMA chain ended %u "
                      "bytes short of NOB * BLKLEN\n", s->data_left);
    }
    msc_data_done(s, true);
}

static void msc_start_data(IngenicMscState *s)
{
    uint32_t total = s->nob * (s->blklen & 0xffff);

    s->snob = 0;
    s->data_dir = (s->cmdat & CMDAT_WRITE) ? DATA_WRITE : DATA_READ;
    s->data_left = total;
    s->auto_cmd12 = !!(s->cmdat & CMDAT_AUTO_CMD12);
    if (!total) {
        msc_data_done(s, false);
        return;
    }
    if (s->data_dir == DATA_READ && !sdbus_data_ready(&s->sdbus)) {
        msc_data_abort(s);
        msc_flag(s, STAT_TIME_OUT_READ, IFLG_TIMEOUT_READ);
        return;
    }
    if (s->data_dir == DATA_WRITE && !sdbus_receive_ready(&s->sdbus)) {
        msc_data_abort(s);
        msc_flag(s, STAT_CRC_WRITE_NOSTS, IFLG_CRC_WRITE_ERR);
        return;
    }
    if ((s->dmac & DMAC_DMAEN) && s->dma_armed) {
        msc_run_dma(s);
    }
}

static void msc_start_cmd(IngenicMscState *s)
{
    SDRequest req = { .cmd = s->cmd & 0x3f, .arg = s->arg };
    uint32_t rtype = CMDAT_RESPONSE(s->cmdat);
    uint8_t rsp[16];
    size_t rlen;
    int i;

    s->stat &= ~STAT_CLEAR_ON_START;
    msc_data_abort(s);
    s->res_len = s->res_pos = 0;

    if (s->cmdat & (CMDAT_CEATA | CMDAT_BOOT | CMDAT_AUTO_CMD23 |
                    CMDAT_STREAM)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-msc: CMDAT 0x%08x: CE-ATA, boot, "
                      "auto CMD23 and stream mode are not modelled\n",
                      s->cmdat);
    }

    rlen = sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
    if (rtype && !rlen) {
        msc_flag(s, STAT_TIME_OUT_RES, IFLG_TIMEOUT_RES);
        return;
    }
    if (rlen == 16) {
        uint8_t b[RES_HALFWORDS * 2] = { 0x3f };

        memcpy(b + 1, rsp, 16);
        for (i = 0; i < RES_HALFWORDS; i++) {
            s->res[i] = b[2 * i] << 8 | b[2 * i + 1];
        }
        s->res_len = RES_HALFWORDS;
    } else if (rlen >= 4) {
        uint8_t hdr = rtype == 3 ? 0x3f : req.cmd;

        s->res[0] = hdr << 8 | rsp[0];
        s->res[1] = rsp[1] << 8 | rsp[2];
        s->res[2] = rsp[3];
        s->res_len = 3;
    }
    msc_flag(s, STAT_END_CMD_RES, IFLG_END_CMD_RES);

    if (s->cmdat & CMDAT_DATA_EN) {
        msc_start_data(s);
    } else if (s->cmdat & CMDAT_BUSY) {
        msc_flag(s, STAT_PRG_DONE, IFLG_PRG_DONE);
    }
}

static uint32_t msc_fifo_read(IngenicMscState *s)
{
    uint32_t v = 0, n, i;

    if (s->data_dir != DATA_READ || !s->data_left) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: RXFIFO read with no "
                      "read data pending\n");
        return 0;
    }
    n = MIN(4, s->data_left);
    for (i = 0; i < n; i++) {
        v |= (uint32_t)sdbus_read_byte(&s->sdbus) << (8 * i);
    }
    s->data_left -= n;
    if (!s->data_left) {
        msc_data_done(s, false);
    }
    return v;
}

static void msc_fifo_write(IngenicMscState *s, uint32_t v)
{
    uint32_t n, i;

    if (s->data_dir != DATA_WRITE || !s->data_left) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: TXFIFO write with no "
                      "write data pending\n");
        return;
    }
    n = MIN(4, s->data_left);
    for (i = 0; i < n; i++) {
        sdbus_write_byte(&s->sdbus, v >> (8 * i));
    }
    s->data_left -= n;
    if (!s->data_left) {
        msc_data_done(s, false);
    }
}

static void msc_soft_reset(IngenicMscState *s)
{
    msc_data_abort(s);
    s->stat = 0;
    s->clkrt = 0;
    s->cmdat = 0;
    s->resto = 0x40;
    s->rdto = 0xffff;
    s->blklen = s->nob = s->snob = 0;
    s->imask = 0xffffffff;
    s->iflg = 0;
    s->cmd = s->arg = 0;
    s->lpm = 0;
    s->dmac = s->dmanda = s->dmada = s->dmalen = s->dmacmd = 0;
    s->ctrl2 = s->rtcnt = s->debug = 0;
    s->clk_en = false;
    s->res_len = s->res_pos = 0;
    memset(s->res, 0, sizeof(s->res));
    s->dma_armed = false;
}

static uint64_t ingenic_msc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicMscState *s = opaque;
    uint32_t v = 0;

    switch (off) {
    case R_CTRL:
        break;
    case R_STAT:
        v = msc_stat(s);
        break;
    case R_CLKRT:
        v = s->clkrt;
        break;
    case R_CMDAT:
        v = s->cmdat;
        break;
    case R_RESTO:
        v = s->resto;
        break;
    case R_RDTO:
        v = s->rdto;
        break;
    case R_BLKLEN:
        v = s->blklen;
        break;
    case R_NOB:
        v = s->nob;
        break;
    case R_SNOB:
        v = s->snob;
        break;
    case R_IMASK:
        v = s->imask;
        break;
    case R_IFLG:
        v = msc_iflg(s);
        break;
    case R_CMD:
        v = s->cmd;
        break;
    case R_ARG:
        v = s->arg;
        break;
    case R_RES:
        if (s->res_pos < s->res_len) {
            v = s->res[s->res_pos++];
        }
        break;
    case R_RXFIFO:
        v = msc_fifo_read(s);
        msc_update_irq(s);
        break;
    case R_TXFIFO:
        break;
    case R_LPM:
        v = s->lpm;
        break;
    case R_DMAC:
        v = s->dmac;
        break;
    case R_DMANDA:
        v = s->dmanda;
        break;
    case R_DMADA:
        v = s->dmada;
        break;
    case R_DMALEN:
        v = s->dmalen;
        break;
    case R_DMACMD:
        v = s->dmacmd;
        break;
    case R_CTRL2:
        v = s->ctrl2;
        break;
    case R_RTCNT:
        v = s->rtcnt;
        break;
    case R_DEBUG:
        v = s->debug;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: read of unknown "
                      "register 0x%" HWADDR_PRIx "\n", off);
        break;
    }
    return v;
}

static void ingenic_msc_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicMscState *s = opaque;
    uint32_t v = val;
    int i;

    switch (off) {
    case R_CTRL:
        if (v & CTRL_RESET) {
            msc_soft_reset(s);
        }
        if (CTRL_CLOCK(v) == 1) {
            s->clk_en = false;
        } else if (CTRL_CLOCK(v) == 2) {
            s->clk_en = true;
        }
        if (v & (CTRL_EXIT_MULTIPLE | CTRL_EXIT_TRANSFER)) {
            msc_data_abort(s);
        }
        if (v & CTRL_READWAIT) {
            qemu_log_mask(LOG_UNIMP, "ingenic-msc: read-wait is not "
                          "modelled\n");
        }
        if (v & CTRL_START_OP) {
            s->clk_en = true;
            msc_start_cmd(s);
        }
        break;
    case R_STAT:
    case R_SNOB:
    case R_RES:
    case R_RXFIFO:
    case R_DMADA:
    case R_DMALEN:
    case R_DMACMD:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: write to read-only "
                      "register 0x%" HWADDR_PRIx "\n", off);
        break;
    case R_CLKRT:
        s->clkrt = v & 7;
        break;
    case R_CMDAT:
        s->cmdat = v;
        break;
    case R_RESTO:
        s->resto = v & 0xffff;
        break;
    case R_RDTO:
        s->rdto = v;
        break;
    case R_BLKLEN:
        s->blklen = v & 0xffff;
        break;
    case R_NOB:
        s->nob = v & 0xffff;
        break;
    case R_IMASK:
        s->imask = v;
        break;
    case R_IFLG:
        s->iflg &= ~v;
        for (i = 0; i < ARRAY_SIZE(iflg_stat); i++) {
            if (v & iflg_stat[i][0]) {
                s->stat &= ~iflg_stat[i][1];
            }
        }
        break;
    case R_CMD:
        s->cmd = v & 0x3f;
        break;
    case R_ARG:
        s->arg = v;
        break;
    case R_TXFIFO:
        msc_fifo_write(s, v);
        break;
    case R_LPM:
        s->lpm = v;
        break;
    case R_DMAC:
        s->dmac = v;
        if ((v & DMAC_DMAEN) && s->data_dir != DATA_NONE && s->data_left) {
            msc_run_dma(s);
        }
        break;
    case R_DMANDA:
        s->dmanda = v;
        s->dma_armed = true;
        break;
    case R_CTRL2:
        s->ctrl2 = v;
        break;
    case R_RTCNT:
        s->rtcnt = v;
        break;
    case R_DEBUG:
        s->debug = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-msc: write of unknown "
                      "register 0x%" HWADDR_PRIx " = 0x%08x\n", off, v);
        break;
    }
    msc_update_irq(s);
}

static const MemoryRegionOps ingenic_msc_ops = {
    .read = ingenic_msc_read,
    .write = ingenic_msc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* U-Boot pops RES with 16-bit reads */
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void msc_set_inserted(DeviceState *dev, bool inserted)
{
    IngenicMscState *s = INGENIC_MSC(dev);

    if (!inserted) {
        msc_data_abort(s);
    }
    qemu_set_irq(s->cd, !inserted);
}

static void ingenic_msc_reset(DeviceState *dev)
{
    IngenicMscState *s = INGENIC_MSC(dev);

    msc_soft_reset(s);
    msc_update_irq(s);
    qemu_set_irq(s->cd, !sdbus_get_inserted(&s->sdbus));
}

static void ingenic_msc_init(Object *obj)
{
    IngenicMscState *s = INGENIC_MSC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_msc_ops, s,
                          TYPE_INGENIC_MSC, 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->cd, "cd", 1);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_INGENIC_MSC_BUS,
              DEVICE(obj), "sd-bus");
}

static int ingenic_msc_post_load(void *opaque, int version_id)
{
    IngenicMscState *s = opaque;

    if (s->res_len > RES_HALFWORDS || s->res_pos > s->res_len ||
        s->data_dir > DATA_WRITE) {
        return -EINVAL;
    }
    msc_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_ingenic_msc = {
    .name = TYPE_INGENIC_MSC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_msc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(stat, IngenicMscState),
        VMSTATE_UINT32(clkrt, IngenicMscState),
        VMSTATE_UINT32(cmdat, IngenicMscState),
        VMSTATE_UINT32(resto, IngenicMscState),
        VMSTATE_UINT32(rdto, IngenicMscState),
        VMSTATE_UINT32(blklen, IngenicMscState),
        VMSTATE_UINT32(nob, IngenicMscState),
        VMSTATE_UINT32(snob, IngenicMscState),
        VMSTATE_UINT32(imask, IngenicMscState),
        VMSTATE_UINT32(iflg, IngenicMscState),
        VMSTATE_UINT32(cmd, IngenicMscState),
        VMSTATE_UINT32(arg, IngenicMscState),
        VMSTATE_UINT32(lpm, IngenicMscState),
        VMSTATE_UINT32(dmac, IngenicMscState),
        VMSTATE_UINT32(dmanda, IngenicMscState),
        VMSTATE_UINT32(dmada, IngenicMscState),
        VMSTATE_UINT32(dmalen, IngenicMscState),
        VMSTATE_UINT32(dmacmd, IngenicMscState),
        VMSTATE_UINT32(ctrl2, IngenicMscState),
        VMSTATE_UINT32(rtcnt, IngenicMscState),
        VMSTATE_UINT32(debug, IngenicMscState),
        VMSTATE_BOOL(clk_en, IngenicMscState),
        VMSTATE_UINT16_ARRAY(res, IngenicMscState, RES_HALFWORDS),
        VMSTATE_UINT32(res_len, IngenicMscState),
        VMSTATE_UINT32(res_pos, IngenicMscState),
        VMSTATE_UINT32(data_dir, IngenicMscState),
        VMSTATE_UINT32(data_left, IngenicMscState),
        VMSTATE_BOOL(auto_cmd12, IngenicMscState),
        VMSTATE_BOOL(dma_armed, IngenicMscState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_msc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, ingenic_msc_reset);
    dc->vmsd = &vmstate_ingenic_msc;
}

static void ingenic_msc_bus_class_init(ObjectClass *klass, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = msc_set_inserted;
}

static const TypeInfo ingenic_msc_types[] = {
    {
        .name = TYPE_INGENIC_MSC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicMscState),
        .instance_init = ingenic_msc_init,
        .class_init = ingenic_msc_class_init,
    }, {
        .name = TYPE_INGENIC_MSC_BUS,
        .parent = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
        .class_init = ingenic_msc_bus_class_init,
    },
};

DEFINE_TYPES(ingenic_msc_types)
