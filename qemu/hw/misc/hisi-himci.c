/*
 * HiSilicon himciv200 (DW MMC) controller emulation.
 *
 * Synopsys DesignWare Mobile Storage Host v200 with HiSilicon ADMA3
 * extensions.  Supports both IDMAC (U-Boot) and ADMA3 (kernel) DMA
 * modes.  Connects to QEMU's SD bus for card I/O.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/sd/sd.h"
#include "system/dma.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_HIMCI     "hisi-himci"
#define TYPE_HISI_HIMCI_BUS "hisi-himci-bus"
OBJECT_DECLARE_SIMPLE_TYPE(HisiHimciState, HISI_HIMCI)

#define HIMCI_REG_SIZE  0x1000

/* ── Register offsets ──────────────────────────────────────────────── */

#define MCI_CTRL        0x00
#define MCI_PWREN       0x04
#define MCI_CLKDIV      0x08
#define MCI_CLKSRC      0x0C
#define MCI_CLKENA      0x10
#define MCI_TMOUT       0x14
#define MCI_CTYPE       0x18
#define MCI_BLKSIZ      0x1C
#define MCI_BYTCNT      0x20
#define MCI_INTMASK     0x24
#define MCI_CMDARG      0x28
#define MCI_CMD         0x2C
#define MCI_RESP0       0x30
#define MCI_RESP1       0x34
#define MCI_RESP2       0x38
#define MCI_RESP3       0x3C
#define MCI_MINTSTS     0x40
#define MCI_RINTSTS     0x44
#define MCI_STATUS      0x48
#define MCI_FIFOTH      0x4C
#define MCI_CDETECT     0x50
#define MCI_WRTPRT      0x54
#define MCI_GPIO        0x58
#define MCI_TCBCNT      0x5C
#define MCI_TBBCNT      0x60
#define MCI_DEBNCE      0x64
#define MCI_VERID       0x6C
#define MCI_HCON        0x70
#define MCI_UHS_REG     0x74
#define MCI_RST_N       0x78

/* IDMAC */
#define MCI_BMOD        0x80
#define MCI_PLDMND      0x84
#define MCI_DBADDR      0x88
#define MCI_IDSTS       0x8C
#define MCI_IDINTEN     0x90
#define MCI_DSCADDR     0x94
#define MCI_BUFADDR     0x98

/* ADMA3 (HiSilicon extension) */
#define MCI_ADMA_CTRL     0xB0
#define MCI_ADMA_Q_ADDR   0xB4
#define MCI_ADMA_Q_DEPTH  0xB8
#define MCI_ADMA_Q_RDPTR  0xBC
#define MCI_ADMA_Q_WRPTR  0xC0

#define MCI_CARDTHRCTL    0x100
#define MCI_UHS_REG_EXT   0x108
#define MCI_TUNING_CTRL   0x118

/* FIFO data window: DW MMC exposes its FIFO at a fixed offset.  On
 * HiSilicon V4-family silicon the av300 mask-ROM reads via
 * SDIO0+0x200 (verified by disasm of the PIO read at bootrom
 * 0x0400457c).  We expose 0x200..0x3FF as the data window. */
#define MCI_FIFO_DATA_BASE  0x200
#define MCI_FIFO_DATA_END   0x3FF
#define MCI_FIFO_SIZE       4096   /* enough for a 512-byte block ×8 */

/* ── Bit definitions ───────────────────────────────────────────────── */

/* CTRL */
#define CTRL_RESET       (1u << 0)
#define CTRL_FIFO_RESET  (1u << 1)
#define CTRL_DMA_RESET   (1u << 2)
#define CTRL_RESET_ALL   (CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET)
#define CTRL_INT_EN      (1u << 4)
#define CTRL_USE_IDMAC   (1u << 25)

/* CMD */
#define CMD_RESP_EXPECT    (1u << 6)
#define CMD_RESP_LONG      (1u << 7)
#define CMD_DATA_EXPECTED  (1u << 9)
#define CMD_RW_WRITE       (1u << 10)
#define CMD_UPDATE_CLK     (1u << 21)
#define CMD_START_CMD      (1u << 31)

/* RINTSTS / INTMASK */
#define INT_CDT   (1u << 0)
#define INT_RE    (1u << 1)
#define INT_CD    (1u << 2)
#define INT_DTO   (1u << 3)
#define INT_RTO   (1u << 8)
#define INT_HLE   (1u << 12)

/* BMOD */
#define BMOD_SWR  (1u << 0)
#define BMOD_DE   (1u << 7)

/* IDSTS */
#define IDSTS_TI         (1u << 0)
#define IDSTS_RI         (1u << 1)
#define IDSTS_DU         (1u << 4)
#define IDSTS_CES        (1u << 5)
#define IDSTS_NIS        (1u << 8)
#define IDSTS_AIS        (1u << 9)
#define IDSTS_PACKET_INT (1u << 25)

/* IDMAC descriptor control bits */
#define IDMAC_OWN  (1u << 31)
#define IDMAC_LD   (1u << 2)

/* ADMA3 ctrl bits */
#define ADMA3_EN         (1u << 0)
#define ADMA3_RESTART    (1u << 1)
#define ADMA3_PKTINT_EN  (1u << 2)
#define ADMA3_RDPTR_MOD  (1u << 3)
#define ADMA3_BLOCKING   (1u << 29)

/* Fixed register values */
#define VERID_VALUE  0x5342270A
#define HCON_VALUE   0x00

/* ── State ─────────────────────────────────────────────────────────── */

struct HisiHimciState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    SDBus sdbus;

    uint32_t ctrl, pwren, clkdiv, clksrc, clkena;
    uint32_t tmout, ctype, blksiz, bytcnt;
    uint32_t intmask, cmdarg, cmd;
    uint32_t resp[4];
    uint32_t rintsts, fifoth;
    uint32_t cdetect, wrtprt, gpio, debnce;
    uint32_t uhs_reg, rst_n;

    uint32_t bmod, dbaddr, idsts, idinten;

    uint32_t adma_ctrl, adma_q_addr, adma_q_depth;
    uint32_t adma_q_rdptr, adma_q_wrptr;

    uint32_t cardthrctl, uhs_reg_ext, tuning_ctrl;

    /* PIO FIFO state — populated when a DATA_EXPECTED command runs
     * without DMA (BMOD_DE=0 and CTRL_USE_IDMAC=0).  The guest reads
     * via MCI_FIFO_DATA_BASE; STATUS bits 17..29 expose FIFO_COUNT
     * in 32-bit words, which the av300 mask-ROM polls before each
     * 4-byte FIFO read. */
    uint8_t  fifo_buf[MCI_FIFO_SIZE];
    uint32_t fifo_len;       /* bytes currently in FIFO */
    uint32_t fifo_rd_off;    /* read cursor (bytes) */
};

/* ── IRQ ───────────────────────────────────────────────────────────── */

static void hisi_himci_update_irq(HisiHimciState *s)
{
    bool level = (s->rintsts & s->intmask) != 0;
    level |= (s->idsts & s->idinten) != 0;
    qemu_set_irq(s->irq, level);
}

/* ── IDMAC DMA ─────────────────────────────────────────────────────── */

/* ── Command execution ─────────────────────────────────────────────── */

static void hisi_himci_send_command(HisiHimciState *s, uint32_t cmd_val,
                                    uint32_t arg)
{
    SDRequest req;
    uint8_t response[16];
    size_t rlen;

    req.cmd = cmd_val & 0x3F;
    req.arg = arg;
    req.crc = 0;

    rlen = sdbus_do_command(&s->sdbus, &req, response, sizeof(response));

    if (cmd_val & CMD_RESP_EXPECT) {
        if ((cmd_val & CMD_RESP_LONG) && rlen == 16) {
            s->resp[3] = ldl_be_p(&response[0]);
            s->resp[2] = ldl_be_p(&response[4]);
            s->resp[1] = ldl_be_p(&response[8]);
            s->resp[0] = ldl_be_p(&response[12]);
        } else if (rlen >= 4) {
            s->resp[0] = ldl_be_p(&response[0]);
            s->resp[1] = 0;
            s->resp[2] = 0;
            s->resp[3] = 0;
        } else {
            s->rintsts |= INT_RTO;
        }
    }
}

/*
 * Run a data transfer using an IDMAC descriptor chain starting at
 * the given physical address.
 */
static void hisi_himci_do_data(HisiHimciState *s, uint32_t idmac_addr,
                               bool is_write)
{
    AddressSpace *as = &address_space_memory;
    uint32_t desc_addr = idmac_addr;
    /*
     * BYTCNT 0 is an undefined-length transfer (the DW MSHC databook): the
     * descriptor chain says how much, a CMD12 ends it on the card.  The
     * RV1106 vendor kernel writes that way -- with 0 taken as nothing to
     * move, every CMD25 sent no data and SD writes were lost.
     */
    bool open_ended = s->bytcnt == 0;
    uint32_t remaining = open_ended ? UINT32_MAX : s->bytcnt;
    uint8_t buf[4096];

    /*
     * Budget the walk against the transfer rather than against a fixed
     * descriptor count.  A fixed 1024 used to sit here and silently truncated
     * every transfer past 4 MiB (1024 x 4 KiB): U-Boot's SD recovery path
     * reads a 5 MiB rootfs image in one CMD18 burst, so the tail arrived as
     * stale DRAM and the uImage data CRC failed with "bad data checksum".
     *
     * "Bounded by `remaining`" is not on its own a bound worth having: BS1
     * permits a one-byte descriptor, so a chain of those against a 32-bit
     * BYTCNT is billions of synchronous DMA + SD-bus iterations with the vCPU
     * thread stuck in every one of them.  One descriptor per 512-byte block is
     * the finest granularity a real transfer uses, so allow that many plus
     * slack: the 5 MiB read above needs ~1240 descriptors and is given ~10900,
     * while a pathological chain is stopped in proportion to what it claimed
     * to move rather than to what it could name.
     */
    uint64_t budget = open_ended ? 65536 : (uint64_t)remaining / 512 + 1024;

    while (remaining > 0) {
        if (budget-- == 0) {
            /* Report it the way the controller would, and do not fall through
             * to the completion status below: a transfer that did not finish
             * must not look like one that did. */
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-himci: IDMAC chain exceeded its descriptor "
                          "budget with %u of %u bytes outstanding\n",
                          remaining, s->bytcnt);
            s->idsts |= IDSTS_DU | IDSTS_AIS;
            return;
        }

        uint32_t desc[4];
        dma_memory_read(as, desc_addr, desc, 16, MEMTXATTRS_UNSPECIFIED);

        uint32_t ctrl = le32_to_cpu(desc[0]);
        uint32_t buf_size = le32_to_cpu(desc[1]) & 0x1FFF;
        uint32_t buf_addr = le32_to_cpu(desc[2]);
        uint32_t next_addr = le32_to_cpu(desc[3]);

        if (!(ctrl & IDMAC_OWN)) {
            break;
        }
        if (buf_size > remaining) {
            buf_size = remaining;
        }

        /*
         * BS1 is 13 bits, so a descriptor may declare up to 8191 bytes --
         * twice the bounce buffer.  Move it in bounce-sized chunks instead
         * of clamping, which would drop the tail of the buffer and leave
         * the card's data stream out of step with the descriptor chain.
         */
        for (uint32_t done = 0; done < buf_size; ) {
            uint32_t chunk = MIN(buf_size - done, (uint32_t)sizeof(buf));

            if (is_write) {
                dma_memory_read(as, buf_addr + done, buf, chunk,
                                MEMTXATTRS_UNSPECIFIED);
                sdbus_write_data(&s->sdbus, buf, chunk);
            } else {
                sdbus_read_data(&s->sdbus, buf, chunk);
                dma_memory_write(as, buf_addr + done, buf, chunk,
                                 MEMTXATTRS_UNSPECIFIED);
            }
            done += chunk;
        }

        remaining -= buf_size;

        /* Clear OWN bit */
        desc[0] = cpu_to_le32(ctrl & ~IDMAC_OWN);
        dma_memory_write(as, desc_addr, &desc[0], 4, MEMTXATTRS_UNSPECIFIED);

        if (ctrl & IDMAC_LD) {
            break;
        }
        /* A descriptor that moves nothing cannot advance the transfer. */
        if (buf_size == 0) {
            break;
        }
        desc_addr = next_addr;
        if (desc_addr == 0) {
            break;
        }
    }

    s->idsts |= (is_write ? IDSTS_TI : IDSTS_RI) | IDSTS_NIS;
}

/*
 * Execute a command with optional data transfer via IDMAC at dbaddr.
 * Used by the direct CMD register path (U-Boot) and clock-update bypass.
 */
/*
 * PIO data-transfer fallback used when neither BMOD_DE nor
 * CTRL_USE_IDMAC is set — the av300 mask-ROM reads SD blocks this way.
 * Drain bytcnt bytes from the SD bus into the FIFO buffer; the guest
 * then reads from MCI_FIFO_DATA_BASE 4 bytes at a time.  Writes are
 * symmetric but the mask-ROM never exercises them.
 */
static void hisi_himci_do_pio(HisiHimciState *s, bool is_write)
{
    uint32_t want = s->bytcnt;
    if (want > sizeof(s->fifo_buf)) {
        want = sizeof(s->fifo_buf);
    }
    if (is_write) {
        sdbus_write_data(&s->sdbus, s->fifo_buf, want);
    } else {
        sdbus_read_data(&s->sdbus, s->fifo_buf, want);
    }
    s->fifo_len = want;
    s->fifo_rd_off = 0;
}

static void hisi_himci_exec_cmd(HisiHimciState *s, uint32_t cmd_val,
                                uint32_t arg)
{
    hisi_himci_send_command(s, cmd_val, arg);

    if ((cmd_val & CMD_DATA_EXPECTED) &&
        !(s->rintsts & INT_RTO)) {
        bool is_write = (cmd_val & CMD_RW_WRITE) != 0;
        if ((s->bmod & BMOD_DE) || (s->ctrl & CTRL_USE_IDMAC)) {
            hisi_himci_do_data(s, s->dbaddr, is_write);
        } else {
            /* PIO fallback — drain the bus into the internal FIFO. */
            hisi_himci_do_pio(s, is_write);
        }
        s->rintsts |= INT_DTO;
    }

    s->rintsts |= INT_CD;
}

static void hisi_himci_process_cmd(HisiHimciState *s)
{
    uint32_t cmd_val = s->cmd;

    if (!(cmd_val & CMD_START_CMD)) {
        return;
    }
    s->cmd = cmd_val & ~CMD_START_CMD;

    if (cmd_val & CMD_UPDATE_CLK) {
        s->rintsts |= INT_CD;
        hisi_himci_update_irq(s);
        return;
    }

    hisi_himci_exec_cmd(s, cmd_val, s->cmdarg);
    hisi_himci_update_irq(s);
}

/* ── ADMA3 queue processing ────────────────────────────────────────── */

static void hisi_himci_process_adma3(HisiHimciState *s)
{
    AddressSpace *as = &address_space_memory;

    if (!(s->adma_ctrl & ADMA3_EN)) {
        return;
    }

    uint32_t depth = s->adma_q_depth;
    if (depth == 0) {
        depth = 1;
    }

    while (s->adma_q_rdptr != s->adma_q_wrptr) {
        uint32_t idx = s->adma_q_rdptr % depth;
        uint32_t ent_addr = s->adma_q_addr + idx * 16;

        /* Read entire descriptor */
        uint32_t ent[4];
        dma_memory_read(as, ent_addr, ent, 16, MEMTXATTRS_UNSPECIFIED);
        uint32_t ent_ctrl = le32_to_cpu(ent[0]);
        uint32_t cmd_des_addr = le32_to_cpu(ent[1]);

        /* Read command descriptor: {blk_sz, byte_cnt, arg, cmd} */
        uint32_t cmd_des[4];
        dma_memory_read(as, cmd_des_addr, cmd_des, 16, MEMTXATTRS_UNSPECIFIED);

        s->blksiz = le32_to_cpu(cmd_des[0]);
        s->bytcnt = le32_to_cpu(cmd_des[1]);
        uint32_t arg = le32_to_cpu(cmd_des[2]);
        uint32_t cmd_val = le32_to_cpu(cmd_des[3]);
        s->cmdarg = arg;

        /* Send the command to the SD card */
        hisi_himci_send_command(s, cmd_val, arg);

        /* Data transfer: IDMAC chain follows cmd descriptor in memory */
        if ((cmd_val & CMD_DATA_EXPECTED) &&
            !(s->rintsts & INT_RTO)) {
            bool is_write = (cmd_val & CMD_RW_WRITE) != 0;
            uint32_t idmac_addr = cmd_des_addr + 16;
            hisi_himci_do_data(s, idmac_addr, is_write);
            s->rintsts |= INT_DTO;
        }
        s->rintsts |= INT_CD;

        /* Set CES (Card Error Summary) if any error bits in rintsts */
        if (s->rintsts & (INT_RTO | INT_RE | INT_HLE)) {
            s->idsts |= IDSTS_CES;
        }

        /* Write response back into entire descriptor */
        ent[2] = cpu_to_le32(s->resp[0]);
        dma_memory_write(as, ent_addr + 8, &ent[2], 4,
                         MEMTXATTRS_UNSPECIFIED);

        s->adma_q_rdptr = (s->adma_q_rdptr + 1) % depth;

        if (ent_ctrl & ADMA3_BLOCKING) {
            break;
        }
    }

    s->idsts |= IDSTS_PACKET_INT;
    hisi_himci_update_irq(s);
}

/* ── MMIO ──────────────────────────────────────────────────────────── */

static uint64_t hisi_himci_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiHimciState *s = HISI_HIMCI(opaque);

    switch (offset) {
    case MCI_CTRL:          return s->ctrl;
    case MCI_PWREN:         return s->pwren;
    case MCI_CLKDIV:        return s->clkdiv;
    case MCI_CLKSRC:        return s->clksrc;
    case MCI_CLKENA:        return s->clkena;
    case MCI_TMOUT:         return s->tmout;
    case MCI_CTYPE:         return s->ctype;
    case MCI_BLKSIZ:        return s->blksiz;
    case MCI_BYTCNT:        return s->bytcnt;
    case MCI_INTMASK:       return s->intmask;
    case MCI_CMDARG:        return s->cmdarg;
    case MCI_CMD:           return s->cmd;
    case MCI_RESP0:         return s->resp[0];
    case MCI_RESP1:         return s->resp[1];
    case MCI_RESP2:         return s->resp[2];
    case MCI_RESP3:         return s->resp[3];
    case MCI_MINTSTS:       return s->rintsts & s->intmask;
    case MCI_RINTSTS:       return s->rintsts;
    case MCI_STATUS: {
        /* Expose FIFO_COUNT (in 32-bit words) in bits 17..29 so the
         * mask-ROM's PIO read loop can poll for available data. */
        uint32_t avail_bytes = (s->fifo_rd_off < s->fifo_len)
                             ? (s->fifo_len - s->fifo_rd_off) : 0;
        uint32_t words = avail_bytes / 4;
        if (words > 0x1FFF) words = 0x1FFF;
        return words << 17;
    }
    case MCI_FIFOTH:        return s->fifoth;
    case MCI_CDETECT:       return s->cdetect;
    case MCI_WRTPRT:        return s->wrtprt;
    case MCI_GPIO:          return s->gpio;
    case MCI_TCBCNT:        return 0;
    case MCI_TBBCNT:        return 0;
    case MCI_DEBNCE:        return s->debnce;
    case MCI_VERID:         return VERID_VALUE;
    case MCI_HCON:          return HCON_VALUE;
    case MCI_UHS_REG:       return s->uhs_reg;
    case MCI_RST_N:         return s->rst_n;
    case MCI_BMOD:          return s->bmod;
    case MCI_DBADDR:        return s->dbaddr;
    case MCI_IDSTS:         return s->idsts;
    case MCI_IDINTEN:       return s->idinten;
    case MCI_DSCADDR:       return 0;
    case MCI_BUFADDR:       return 0;
    case MCI_ADMA_CTRL:     return s->adma_ctrl;
    case MCI_ADMA_Q_ADDR:   return s->adma_q_addr;
    case MCI_ADMA_Q_DEPTH:  return s->adma_q_depth;
    case MCI_ADMA_Q_RDPTR:  return s->adma_q_rdptr;
    case MCI_ADMA_Q_WRPTR:  return s->adma_q_wrptr;
    case MCI_CARDTHRCTL:    return s->cardthrctl;
    case MCI_UHS_REG_EXT:   return s->uhs_reg_ext;
    case MCI_TUNING_CTRL:   return s->tuning_ctrl;
    default:
        /* FIFO_DATA window — return next 4 bytes from the staged PIO
         * read buffer, advance the read cursor.  Any offset in the
         * window does this; DW MMC FIFO is "memory-mapped" but uses
         * an internal pointer, not the offset. */
        if (offset >= MCI_FIFO_DATA_BASE && offset <= MCI_FIFO_DATA_END) {
            if (s->fifo_rd_off + 4 <= s->fifo_len) {
                uint32_t v = ldl_le_p(s->fifo_buf + s->fifo_rd_off);
                s->fifo_rd_off += 4;
                return v;
            }
            return 0;
        }
        return 0;
    }
}

static void hisi_himci_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    HisiHimciState *s = HISI_HIMCI(opaque);

    switch (offset) {
    case MCI_CTRL:
        s->ctrl = val & ~CTRL_RESET_ALL;
        break;
    case MCI_PWREN:
        s->pwren = val;
        if (val & 1) {
            sdbus_set_voltage(&s->sdbus, SD_VOLTAGE_3_3V);
        }
        break;
    case MCI_CLKDIV:      s->clkdiv = val;    break;
    case MCI_CLKSRC:      s->clksrc = val;    break;
    case MCI_CLKENA:      s->clkena = val;    break;
    case MCI_TMOUT:       s->tmout = val;     break;
    case MCI_CTYPE:       s->ctype = val;     break;
    case MCI_BLKSIZ:      s->blksiz = val;    break;
    case MCI_BYTCNT:      s->bytcnt = val;    break;
    case MCI_INTMASK:
        s->intmask = val;
        hisi_himci_update_irq(s);
        break;
    case MCI_CMDARG:      s->cmdarg = val;    break;
    case MCI_CMD:
        s->cmd = val;
        hisi_himci_process_cmd(s);
        break;
    case MCI_RINTSTS:
        s->rintsts &= ~(uint32_t)val;
        hisi_himci_update_irq(s);
        break;
    case MCI_FIFOTH:      s->fifoth = val;    break;
    case MCI_GPIO:        s->gpio = val;      break;
    case MCI_DEBNCE:      s->debnce = val;    break;
    case MCI_UHS_REG:     s->uhs_reg = val;   break;
    case MCI_RST_N:       s->rst_n = val;     break;
    case MCI_BMOD:
        s->bmod = val & ~BMOD_SWR;
        break;
    case MCI_PLDMND:
        break;
    case MCI_DBADDR:      s->dbaddr = val;    break;
    case MCI_IDSTS:
        s->idsts &= ~(uint32_t)val;
        hisi_himci_update_irq(s);
        break;
    case MCI_IDINTEN:
        s->idinten = val;
        hisi_himci_update_irq(s);
        break;
    case MCI_ADMA_CTRL:
        s->adma_ctrl = val & ~ADMA3_RESTART;
        break;
    case MCI_ADMA_Q_ADDR: s->adma_q_addr = val;   break;
    case MCI_ADMA_Q_DEPTH: s->adma_q_depth = val;  break;
    case MCI_ADMA_Q_RDPTR: s->adma_q_rdptr = val;  break;
    case MCI_ADMA_Q_WRPTR:
        s->adma_q_wrptr = val;
        hisi_himci_process_adma3(s);
        break;
    case MCI_CARDTHRCTL:   s->cardthrctl = val;    break;
    case MCI_UHS_REG_EXT:  s->uhs_reg_ext = val;   break;
    case MCI_TUNING_CTRL:  s->tuning_ctrl = val;   break;
    default:
        break;
    }
}

static const MemoryRegionOps hisi_himci_ops = {
    .read = hisi_himci_read,
    .write = hisi_himci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── SD bus callbacks ──────────────────────────────────────────────── */

static void hisi_himci_set_inserted(DeviceState *dev, bool inserted)
{
    HisiHimciState *s = HISI_HIMCI(dev);
    s->cdetect = inserted ? 0x0 : 0x1;
    s->rintsts |= INT_CDT;
    hisi_himci_update_irq(s);
}

static void hisi_himci_set_readonly(DeviceState *dev, bool readonly)
{
    HisiHimciState *s = HISI_HIMCI(dev);
    s->wrtprt = readonly ? 0x1 : 0x0;
}

/* ── QOM ───────────────────────────────────────────────────────────── */

static void hisi_himci_init(Object *obj)
{
    HisiHimciState *s = HISI_HIMCI(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_himci_ops, s,
                          TYPE_HISI_HIMCI, HIMCI_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_HISI_HIMCI_BUS,
              DEVICE(obj), "sd-bus");
}

static void hisi_himci_reset(DeviceState *dev)
{
    HisiHimciState *s = HISI_HIMCI(dev);

    s->ctrl = 0;
    s->pwren = 0;
    s->clkdiv = 0;
    s->clksrc = 0;
    s->clkena = 0;
    s->tmout = 0xFFFFFF40;
    s->ctype = 0;
    s->blksiz = 0x200;
    s->bytcnt = 0x200;
    s->intmask = 0;
    s->cmdarg = 0;
    s->cmd = 0;
    memset(s->resp, 0, sizeof(s->resp));
    s->rintsts = 0;
    s->fifoth = 0;
    s->cdetect = sdbus_get_inserted(&s->sdbus) ? 0x0 : 0x1;
    s->wrtprt = sdbus_get_readonly(&s->sdbus) ? 0x1 : 0x0;
    s->gpio = 0;
    s->debnce = 0;
    s->uhs_reg = 0;
    s->rst_n = 0;

    s->bmod = 0;
    s->dbaddr = 0;
    s->idsts = 0;
    s->idinten = 0;

    s->adma_ctrl = 0;
    s->adma_q_addr = 0;
    s->adma_q_depth = 0;
    s->adma_q_rdptr = 0;
    s->adma_q_wrptr = 0;

    s->cardthrctl = 0;
    s->uhs_reg_ext = 0;
    s->tuning_ctrl = 0;

    s->fifo_len = 0;
    s->fifo_rd_off = 0;

    qemu_set_irq(s->irq, 0);
}

static void hisi_himci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_himci_reset);
}

static void hisi_himci_bus_class_init(ObjectClass *klass, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);
    sbc->set_inserted = hisi_himci_set_inserted;
    sbc->set_readonly = hisi_himci_set_readonly;
}

static const TypeInfo hisi_himci_types[] = {
    {
        .name          = TYPE_HISI_HIMCI_BUS,
        .parent        = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
        .class_init    = hisi_himci_bus_class_init,
    },
    {
        .name          = TYPE_HISI_HIMCI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(HisiHimciState),
        .instance_init = hisi_himci_init,
        .class_init    = hisi_himci_class_init,
    },
};

DEFINE_TYPES(hisi_himci_types)
