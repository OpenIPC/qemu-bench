/*
 * Xiongmai SPI-NOR flash controller ("xmsfc"): v1 on XM510, v2 on
 * XM530/XM550.
 *
 * Not a HiSilicon hifmc: Xiongmai's own register block, as driven by
 * drivers/mtd/devices/xmsfc/xmsfc.c in the vendor 3.0.101 kernel and by the
 * board's U-Boot 2014.04.
 *
 * Two regions:
 *  - registers (0x800 at 0x100F0000):
 *      MODE      +0x00   ADDR  +0x04   OP    +0x08   CMD    +0x0C
 *      SRW       +0x10   WRNUM +0x14   RW_SR_BSEL +0x18
 *      DOUT      +0x34   ST    +0x38   SRR   +0x3C   MODE_ST +0x4C
 *      CACHE_CMD +0x80   BUF   +0x400 (256 bytes, one per 32-bit word)
 *    Software loads CMD (the SPI opcode) and ADDR/WRNUM/BUF, writes an
 *    operation number to OP, and polls ST for that operation's done bit:
 *      OP2 erase (CMD 0xD8/0x52/0x20/0xC7/0x60 at ADDR)
 *      OP3 read status/ID: RW_SR_BSEL+1 bytes into SRR, first byte highest
 *          (JEDEC 0x9F with BSEL 2 gives 0xef4017 for a W25Q64)
 *      OP4 write status (SRW, BSEL+1 bytes)
 *      OP6 page program: WRNUM+1 bytes from BUF to ADDR
 *    Every operation completes before the write to OP returns, so ST holds
 *    exactly the done bit of the last one.
 *  - the read window (16 MiB at 0x0): the kernel's mtd read is a memcpy from
 *    it, and U-Boot runs from it in place, so it is a ROM device that reads
 *    the flash array directly.
 *
 * v2 (drivers/mtd/devices/xmsfc/xmsfc_v2.[ch] in the 3.10.103 kernel) is a
 * new register layout over the same idea, at 0x100F4000:
 *      CTRL +0x00  CFG1 +0x04  CFG2 +0x08  ADDR +0x0C  OP   +0x10
 *      CMD  +0x14  SRW  +0x18  RW_NUM +0x1C  PRE_CMD +0x20  PST_CMD +0x24
 *      ST   +0x2C  R_SR_ID +0x30  MODE_ST +0x40  CACHE_CMD +0x44
 *      RAM  +0x2000 (4 KiB, packed four to a word)
 *      OP1 erase; done bit (1 << (op-1)) for every operation.
 *      OP3 read: status/ID (RW_NUM+1 bytes into R_SR_ID), or for a data-read
 *          opcode RW_NUM+1 bytes from ADDR into RAM at ADDR mod 4 KiB.  That
 *          is the XM550 U-Boot's sf read: sixteen 256-byte 0x6B reads at
 *          consecutive addresses, then the whole 4 KiB read out of RAM.
 *      OP4 write: with CMD 0x01 the status register (SRW, RW_NUM+1 bytes --
 *          U-Boot sets QE this way), otherwise a page program of RW_NUM+1
 *          bytes from RAM offset 0 (the kernel's xmsfc_reg_write()).
 * MODE_ST reads 0 there: the driver then uses single-line commands.  The
 * read window is at 0x60000000.
 *
 * Erases and programs are written through to flash-file, so what the guest
 * stores survives a reboot, as on a camera.
 *
 * Status registers, on both versions: 0x05/0x35/0x15 read SR1/SR2/SR3, and a
 * status write (v1 OP4, v2 OP4) with 0x01 (one or two bytes), 0x31 or 0x11
 * writes SR1(+SR2)/SR2/SR3. Every other OP4 opcode on v2 is a page program.
 * With "wps" set the part powers up as Xiongmai bootloaders leave it:
 * SR3.WPS on, so protection comes from per-block lock bits, all locked at
 * power-up, and erases and programs are dropped as on silicon. 0x98 (v2 OP1
 * without an address) unlocks every block, and clearing WPS ends the
 * per-block mode, which is what the OpenIPC U-Boot does at probe.
 *
 * Two limits: the BP bits in SR1 are stored but never enforced, so with WPS
 * clear nothing is protected; and SR3 survives a machine reset but not a
 * QEMU restart -- flash-file holds the array only, and "wps" sets the
 * power-up state of every run.
 *
 * SPI NAND (v2, "nand-file"): the controller in NAND mode, as the vendor
 * U-Boot's xmsfc_v2_nand.c drives it. Raw pages of 2048 data + 64 spare
 * bytes, 64 to a 128 KiB block; a W25N01GV (EF AA 21) unless "nand-id" says
 * otherwise. OP1 runs an addressed command without data (0x13 page read to
 * the cache, 0x10 program execute, 0xD8 block erase, row in ADDR), OP2 a
 * bare command (0xFF reset), OP3 reads (0x9F ID, 0x0F feature at ADDR,
 * 0x03/0x0B the cache from column ADDR into RAM at the same offset) and OP4
 * writes (0x1F feature, 0x02/0x84 RAM into the cache at column ADDR; 0x02
 * clears the cache first). Programs AND into the page. The part powers up
 * with every block protected (feature A0 = 0x7C): a program or erase then
 * fails with P_FAIL/E_FAIL in feature C0 until A0 is cleared, as the
 * vendor driver does before each one.
 *
 * NOTHING DOCUMENTS HOW THE XM530 BOOTS FROM SPI NAND. The model assumes
 * the read window in bus-read mode serves the NAND's page data linearly,
 * spare bytes skipped, so the CPU runs U-Boot in place from it as from NOR
 * (the vendor tree links a NAND U-Boot at 0x60001000, behind a 4 KiB
 * header). It is an unverified guess, here so that a NAND U-Boot can be
 * exercised at all; it says nothing about silicon.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XM_SFC "xm-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(XmSfcState, XM_SFC)

#define R_MODE        0x00
#define R_ADDR        0x04
#define R_OP          0x08
#define R_CMD         0x0C
#define R_SRW         0x10
#define R_WRNUM       0x14
#define R_BSEL        0x18
#define R_DOUT        0x34
#define R_ST          0x38
#define R_SRR         0x3C
#define R_MODE_ST     0x4C
#define R_CACHE_CMD   0x80
#define R_BUF         0x400
#define BUF_BYTES     256
#define REGS_SIZE     0x800

#define WINDOW_SIZE   (16 * MiB)

#define V2_CTRL       0x00
#define V2_CFG1       0x04
#define V2_CFG2       0x08
#define V2_ADDR       0x0C
#define V2_OP         0x10
#define V2_CMD        0x14
#define V2_SRW        0x18
#define V2_RW_NUM     0x1C
#define V2_PRE_CMD    0x20
#define V2_PST_CMD    0x24
#define V2_TDEL       0x28
#define V2_ST         0x2C
#define V2_R_SR_ID    0x30
#define V2_INT_MSK    0x38
#define V2_MODE_ST    0x40
#define V2_CACHE_CMD  0x44
#define V2_RAM        0x2000
#define V2_RAM_BYTES  0x1000
#define V2_REGS_SIZE  (V2_RAM + V2_RAM_BYTES)

struct XmSfcState {
    SysBusDevice parent_obj;

    MemoryRegion regs;
    MemoryRegion window;

    uint32_t mode, addr, cmd, srw, wrnum, bsel, st, srr, cache_cmd;
    uint32_t v2_regs[V2_RAM / 4];   /* v2 registers with no side effect */
    uint8_t v2_ram[V2_RAM_BYTES];
    uint8_t buf[BUF_BYTES];
    uint8_t sr1, sr2, sr3;
    bool blocks_locked;       /* per-block locks, live while SR3.WPS */

    uint8_t *array;           /* the flash, size bytes */
    int fd;                   /* flash-file, or -1 */

    char *flash_file;
    uint32_t size;
    uint32_t jedec;
    uint32_t version;
    bool wps;                 /* power up with SR3.WPS set */

    /* SPI NAND */
    char *nand_file;
    int nand_fd;
    uint8_t *nand;            /* raw pages, data then spare */
    uint32_t nand_pages;
    uint32_t nand_id;
    uint8_t cache[2048 + 64];
    uint8_t feat_a0, feat_b0, feat_c0;
};

#define NAND_DATA       2048
#define NAND_SPARE      64
#define NAND_RAW        (NAND_DATA + NAND_SPARE)
#define NAND_PPB        64      /* pages per block */
#define NAND_A0_BP      0x78    /* BP3..BP0: any set protects (TB aside) */
#define NAND_C0_EFAIL   0x04
#define NAND_C0_PFAIL   0x08

static void xm_sfc_persist(XmSfcState *s, uint32_t off, uint32_t len)
{
    if (s->fd < 0) {
        return;
    }
    if (pwrite(s->fd, s->array + off, len, off) != (ssize_t)len) {
        error_report("xm-sfc: write-through to %s failed at 0x%x: %s",
                     s->flash_file, off, strerror(errno));
    }
}

#define SR3_WPS 0x04

/* Per-block protection drops the operation, as on silicon: no error. */
static bool xm_sfc_locked(XmSfcState *s, const char *what)
{
    if ((s->sr3 & SR3_WPS) && s->blocks_locked) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "xm-sfc: %s at 0x%x dropped: block locked (SR3.WPS)\n",
                      what, s->addr);
        return true;
    }
    return false;
}

static void xm_sfc_erase(XmSfcState *s)
{
    uint32_t len;

    switch (s->cmd) {
    case 0x20:
        len = 4 * KiB;
        break;
    case 0x52:
        len = 32 * KiB;
        break;
    case 0xD8:
        len = 64 * KiB;
        break;
    case 0x98:                /* global block unlock: no address, no data */
        s->blocks_locked = false;
        return;
    case 0x06:                /* write enable / disable: nothing to model */
    case 0x04:
        return;
    case 0xC7:
    case 0x60:
        if (xm_sfc_locked(s, "chip erase")) {
            return;
        }
        memset(s->array, 0xff, s->size);
        memory_region_flush_rom_device(&s->window, 0, s->size);
        xm_sfc_persist(s, 0, s->size);
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: erase opcode 0x%02x\n", s->cmd);
        return;
    }

    const uint32_t off = s->addr & ~(len - 1);
    if (xm_sfc_locked(s, "erase")) {
        return;
    }
    if (off >= s->size) {
        qemu_log_mask(LOG_GUEST_ERROR, "xm-sfc: erase at 0x%x past the end\n",
                      s->addr);
        return;
    }
    len = MIN(len, s->size - off);
    memset(s->array + off, 0xff, len);
    memory_region_flush_rom_device(&s->window, off, len);
    xm_sfc_persist(s, off, len);
}

/* NOR programming can only clear bits, and a page program wraps within its
 * 256-byte page. */
static void xm_sfc_program(XmSfcState *s)
{
    const uint32_t n = MIN(s->wrnum + 1, BUF_BYTES);
    const uint32_t page = s->addr & ~(BUF_BYTES - 1);
    uint32_t i;

    if (page >= s->size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "xm-sfc: program at 0x%x past the end\n", s->addr);
        return;
    }
    if (xm_sfc_locked(s, "program")) {
        return;
    }
    for (i = 0; i < n; i++) {
        const uint32_t off = page + ((s->addr + i) & (BUF_BYTES - 1));
        s->array[off] &= s->buf[i];
    }
    memory_region_flush_rom_device(&s->window, page, BUF_BYTES);
    xm_sfc_persist(s, page, BUF_BYTES);
}

/* RW_SR_BSEL + 1 bytes of the opcode's answer, first byte highest. */
static void xm_sfc_read_reg(XmSfcState *s)
{
    uint8_t out[4] = { 0 };
    const uint32_t n = MIN(s->bsel + 1, 4);
    uint32_t v = 0, i;

    switch (s->cmd) {
    case 0x9F:
        out[0] = s->jedec >> 16;
        out[1] = s->jedec >> 8;
        out[2] = s->jedec;
        break;
    case 0x05:
        out[0] = s->sr1;      /* never busy: every operation is done */
        break;
    case 0x35:
        out[0] = s->sr2;
        break;
    case 0x15:
        out[0] = s->sr3;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: read opcode 0x%02x\n", s->cmd);
        break;
    }
    for (i = 0; i < n; i++) {
        v = (v << 8) | out[i];
    }
    s->srr = v;
}

static bool xm_sfc_is_status_write(uint32_t cmd)
{
    return cmd == 0x01 || cmd == 0x31 || cmd == 0x11;
}

static void xm_sfc_write_reg(XmSfcState *s)
{
    switch (s->cmd) {
    case 0x01:
        if (s->bsel >= 1) {
            s->sr1 = (s->srw >> 8) & 0xfc;
            s->sr2 = s->srw & 0xff;
        } else {
            s->sr1 = s->srw & 0xfc;
        }
        break;
    case 0x31:
        s->sr2 = s->srw & 0xff;
        break;
    case 0x11:
        s->sr3 = s->srw & 0xff;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: write opcode 0x%02x\n", s->cmd);
        break;
    }
}

static void xm_sfc_op(XmSfcState *s, uint32_t op)
{
    switch (op) {
    case 2:
        xm_sfc_erase(s);
        break;
    case 3:
        xm_sfc_read_reg(s);
        break;
    case 4:
        xm_sfc_write_reg(s);
        break;
    case 6:
        xm_sfc_program(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: operation %u\n", op);
        break;
    }
    s->st = op >= 1 && op <= 6 ? 1u << (op - 1) : 0;
}

static uint64_t xm_sfc_regs_read(void *opaque, hwaddr off, unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= R_BUF && off < R_BUF + 4 * BUF_BYTES) {
        return s->buf[(off - R_BUF) / 4];
    }
    switch (off) {
    case R_MODE:
        return s->mode;
    case R_ADDR:
        return s->addr;
    case R_CMD:
        return s->cmd;
    case R_SRW:
        return s->srw;
    case R_WRNUM:
        return s->wrnum;
    case R_BSEL:
        return s->bsel;
    case R_ST:
        return s->st;
    case R_SRR:
        return s->srr;
    case R_MODE_ST:
        return 1;             /* a mode change is done at once */
    case R_CACHE_CMD:
        return s->cache_cmd;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void xm_sfc_regs_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= R_BUF && off < R_BUF + 4 * BUF_BYTES) {
        s->buf[(off - R_BUF) / 4] = val;
        return;
    }
    switch (off) {
    case R_MODE:
        s->mode = val;
        break;
    case R_ADDR:
        s->addr = val;
        break;
    case R_OP:
        xm_sfc_op(s, val);
        break;
    case R_CMD:
        s->cmd = val & 0xff;
        break;
    case R_SRW:
        s->srw = val;
        break;
    case R_WRNUM:
        s->wrnum = val;
        break;
    case R_BSEL:
        s->bsel = val;
        break;
    case R_CACHE_CMD:
        s->cache_cmd = val;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        break;
    }
}

static const MemoryRegionOps xm_sfc_regs_ops = {
    .read = xm_sfc_regs_read,
    .write = xm_sfc_regs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * v2 reuses the v1 engines: erase/program/read-register act on cmd, addr,
 * wrnum (RW_NUM) and buf (RAM), and the answer to a register read lands in
 * srr (R_SR_ID), wrnum + 1 bytes of it.
 */
/* SPI NOR data-read opcodes: single, fast, dual/quad output and I/O. */
static bool xm_sfc_is_data_read(uint32_t cmd)
{
    switch (cmd) {
    case 0x03: case 0x0B: case 0x3B: case 0x6B: case 0xBB: case 0xEB:
        return true;
    default:
        return false;
    }
}

static void xm_sfc_v2_data_read(XmSfcState *s)
{
    const uint32_t n = MIN(s->wrnum + 1, V2_RAM_BYTES);
    uint32_t i;

    for (i = 0; i < n; i++) {
        const uint32_t a = s->addr + i;
        s->v2_ram[a % V2_RAM_BYTES] = a < s->size ? s->array[a] : 0xff;
    }
}

static void xm_sfc_nand_persist(XmSfcState *s, uint32_t page, uint32_t n)
{
    if (s->nand_fd < 0) {
        return;
    }
    if (pwrite(s->nand_fd, s->nand + (size_t)page * NAND_RAW,
               (size_t)n * NAND_RAW, (off_t)page * NAND_RAW) !=
        (ssize_t)n * NAND_RAW) {
        error_report("xm-sfc: write-through to %s failed at page %u: %s",
                     s->nand_file, page, strerror(errno));
    }
}

/* The window shows page data linearly (see the header): keep it in step */
static void xm_sfc_nand_window(XmSfcState *s, uint32_t page, uint32_t n)
{
    uint32_t i;

    for (i = page; i < page + n; i++) {
        const uint64_t lin = (uint64_t)i * NAND_DATA;
        if (lin + NAND_DATA > s->size) {
            break;
        }
        memcpy(s->array + lin, s->nand + (size_t)i * NAND_RAW, NAND_DATA);
        memory_region_flush_rom_device(&s->window, lin, NAND_DATA);
    }
}

static void xm_sfc_nand_op1(XmSfcState *s)
{
    const uint32_t row = s->addr;
    uint32_t i;

    switch (s->cmd) {
    case 0x13:                /* page read to cache */
        if (row >= s->nand_pages) {
            memset(s->cache, 0xff, sizeof(s->cache));
            break;
        }
        memcpy(s->cache, s->nand + (size_t)row * NAND_RAW, NAND_RAW);
        break;
    case 0x10:                /* program execute */
        s->feat_c0 &= ~NAND_C0_PFAIL;
        if (row >= s->nand_pages || (s->feat_a0 & NAND_A0_BP)) {
            qemu_log_mask(LOG_GUEST_ERROR, "xm-sfc: NAND program of page "
                          "0x%x failed (protected or past the end)\n", row);
            s->feat_c0 |= NAND_C0_PFAIL;
            break;
        }
        for (i = 0; i < NAND_RAW; i++) {
            s->nand[(size_t)row * NAND_RAW + i] &= s->cache[i];
        }
        xm_sfc_nand_persist(s, row, 1);
        xm_sfc_nand_window(s, row, 1);
        break;
    case 0xD8: {              /* block erase */
        const uint32_t first = row & ~(NAND_PPB - 1);
        s->feat_c0 &= ~NAND_C0_EFAIL;
        if (first >= s->nand_pages || (s->feat_a0 & NAND_A0_BP)) {
            qemu_log_mask(LOG_GUEST_ERROR, "xm-sfc: NAND erase of page "
                          "0x%x failed (protected or past the end)\n", row);
            s->feat_c0 |= NAND_C0_EFAIL;
            break;
        }
        memset(s->nand + (size_t)first * NAND_RAW, 0xff,
               (size_t)NAND_PPB * NAND_RAW);
        xm_sfc_nand_persist(s, first, NAND_PPB);
        xm_sfc_nand_window(s, first, NAND_PPB);
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: NAND OP1 opcode 0x%02x\n", s->cmd);
        break;
    }
}

static uint8_t *xm_sfc_nand_feature(XmSfcState *s)
{
    switch (s->addr & 0xff) {
    case 0xA0:
        return &s->feat_a0;
    case 0xB0:
        return &s->feat_b0;
    case 0xC0:
        return &s->feat_c0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "xm-sfc: NAND feature 0x%02x\n",
                      s->addr & 0xff);
        return NULL;
    }
}

static void xm_sfc_nand_reset_features(XmSfcState *s)
{
    s->feat_a0 = 0x7C;        /* every block protected at power-up */
    s->feat_b0 = 0x18;        /* on-die ECC on, buffer mode */
    s->feat_c0 = 0;
}

static void xm_sfc_nand_op(XmSfcState *s, uint32_t op)
{
    const uint32_t n = MIN(s->wrnum + 1, V2_RAM_BYTES);
    const uint32_t col = s->addr & 0xfff;
    uint8_t *f;
    uint32_t i;

    switch (op) {
    case 1:
        xm_sfc_nand_op1(s);
        break;
    case 2:
        if (s->cmd == 0xFF) {
            xm_sfc_nand_reset_features(s);
        } else {
            qemu_log_mask(LOG_UNIMP, "xm-sfc: NAND OP2 opcode 0x%02x\n",
                          s->cmd);
        }
        break;
    case 3:
        switch (s->cmd) {
        case 0x9F:            /* ID, first byte highest */
            s->srr = 0;
            for (i = 0; i < MIN(n, 3); i++) {
                s->srr = (s->srr << 8) | ((s->nand_id >> (16 - 8 * i)) & 0xff);
            }
            break;
        case 0x0F:
            f = xm_sfc_nand_feature(s);
            s->srr = f ? *f : 0;
            break;
        case 0x03:
        case 0x0B:            /* cache -> RAM, at the same column */
            for (i = 0; i < n && col + i < V2_RAM_BYTES; i++) {
                s->v2_ram[col + i] = col + i < NAND_RAW ? s->cache[col + i]
                                                        : 0xff;
            }
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "xm-sfc: NAND OP3 opcode 0x%02x\n",
                          s->cmd);
            break;
        }
        break;
    case 4:
        switch (s->cmd) {
        case 0x1F:
            f = xm_sfc_nand_feature(s);
            if (f) {
                *f = s->srw & 0xff;
            }
            break;
        case 0x02:            /* program load: cache to 0xff, then load */
            memset(s->cache, 0xff, sizeof(s->cache));
            /* fall through */
        case 0x84:            /* random program load */
            for (i = 0; i < n && col + i < NAND_RAW; i++) {
                s->cache[col + i] = s->v2_ram[col + i];
            }
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "xm-sfc: NAND OP4 opcode 0x%02x\n",
                          s->cmd);
            break;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: NAND operation %u\n", op);
        break;
    }
    s->st = op >= 1 && op <= 10 ? 1u << (op - 1) : 0;
}

static void xm_sfc_v2_op(XmSfcState *s, uint32_t op)
{
    if (s->nand) {
        xm_sfc_nand_op(s, op);
        return;
    }
    switch (op) {
    case 1:
        xm_sfc_erase(s);
        break;
    case 3:
        if (xm_sfc_is_data_read(s->cmd)) {
            xm_sfc_v2_data_read(s);
            break;
        }
        s->bsel = s->wrnum;
        xm_sfc_read_reg(s);
        break;
    case 4:
        if (xm_sfc_is_status_write(s->cmd)) {
            s->bsel = s->wrnum;
            xm_sfc_write_reg(s);
        } else {
            memcpy(s->buf, s->v2_ram, BUF_BYTES);
            xm_sfc_program(s);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "xm-sfc: v2 operation %u\n", op);
        break;
    }
    s->st = op >= 1 && op <= 10 ? 1u << (op - 1) : 0;
}

static uint64_t xm_sfc_v2_read(void *opaque, hwaddr off, unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= V2_RAM && off < V2_RAM + V2_RAM_BYTES) {
        uint32_t v = 0, i;
        for (i = 0; i < size && off - V2_RAM + i < V2_RAM_BYTES; i++) {
            v |= (uint32_t)s->v2_ram[off - V2_RAM + i] << (8 * i);
        }
        return v;
    }
    switch (off) {
    case V2_ADDR:
        return s->addr;
    case V2_CMD:
        return s->cmd;
    case V2_SRW:
        return s->srw;
    case V2_RW_NUM:
        return s->wrnum;
    case V2_ST:
        return s->st;
    case V2_R_SR_ID:
        return s->srr;
    case V2_MODE_ST:
        return 0;             /* single-line mode */
    default:
        if (off < V2_RAM && !(off & 3)) {
            return s->v2_regs[off / 4];
        }
        qemu_log_mask(LOG_UNIMP, "xm-sfc: v2 read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
}

static void xm_sfc_v2_write(void *opaque, hwaddr off, uint64_t val,
                            unsigned size)
{
    XmSfcState *s = opaque;

    if (off >= V2_RAM && off < V2_RAM + V2_RAM_BYTES) {
        uint32_t i;
        for (i = 0; i < size && off - V2_RAM + i < V2_RAM_BYTES; i++) {
            s->v2_ram[off - V2_RAM + i] = val >> (8 * i);
        }
        return;
    }
    switch (off) {
    case V2_ADDR:
        s->addr = val;
        break;
    case V2_OP:
        xm_sfc_v2_op(s, val);
        break;
    case V2_CMD:
        s->cmd = val & 0xff;
        break;
    case V2_SRW:
        s->srw = val;
        break;
    case V2_RW_NUM:
        s->wrnum = val;
        break;
    default:
        /* CTRL, CFG1/2, PRE/PST_CMD, TDEL, interrupt and cache registers
         * shape the SPI cycle on silicon and change nothing here. */
        if (off < V2_RAM && !(off & 3)) {
            s->v2_regs[off / 4] = val;
            break;
        }
        qemu_log_mask(LOG_UNIMP, "xm-sfc: v2 write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        break;
    }
}

static const MemoryRegionOps xm_sfc_v2_regs_ops = {
    .read = xm_sfc_v2_read,
    .write = xm_sfc_v2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* The read window ignores writes, as the controller does. */
static void xm_sfc_window_write(void *opaque, hwaddr off, uint64_t val,
                                unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "xm-sfc: write to the read window at 0x%" HWADDR_PRIx "\n",
                  off);
}

static const MemoryRegionOps xm_sfc_window_ops = {
    .write = xm_sfc_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void xm_sfc_realize(DeviceState *dev, Error **errp)
{
    XmSfcState *s = XM_SFC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->fd = -1;
    s->nand_fd = -1;
    if (s->nand_file && *s->nand_file) {
        off_t len;

        if (s->version != 2) {
            error_setg(errp, "xm-sfc: SPI NAND needs the v2 controller");
            return;
        }
        s->nand_fd = open(s->nand_file, O_RDWR);
        if (s->nand_fd < 0) {
            error_setg_errno(errp, errno, "xm-sfc: cannot open %s",
                             s->nand_file);
            return;
        }
        len = lseek(s->nand_fd, 0, SEEK_END);
        if (len <= 0 || len % ((off_t)NAND_PPB * NAND_RAW)) {
            error_setg(errp, "xm-sfc: %s is not whole blocks of 64 raw "
                       "pages of %u bytes", s->nand_file, NAND_RAW);
            return;
        }
        s->nand_pages = len / NAND_RAW;
        s->nand = g_malloc(len);
        if (pread(s->nand_fd, s->nand, len, 0) != len) {
            error_setg_errno(errp, errno, "xm-sfc: cannot read %s",
                             s->nand_file);
            return;
        }
        /* The window: page data, linearly, as far as it reaches */
        s->size = MIN((uint64_t)s->nand_pages * NAND_DATA, WINDOW_SIZE);
        if (!s->nand_id) {
            s->nand_id = 0xefaa21;      /* W25N01GV */
        }
    } else if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "xm-sfc: cannot open %s",
                             s->flash_file);
            return;
        }
        const off_t len = lseek(s->fd, 0, SEEK_END);
        if (len > 0 && len <= WINDOW_SIZE) {
            s->size = len;
        }
    }
    /* Whole 64 KiB sectors, as every NOR chip is: erase and page program
     * then never reach past the array. */
    if (!s->size || s->size > WINDOW_SIZE || (s->size & (64 * KiB - 1))) {
        error_setg(errp, "xm-sfc: flash size 0x%x is not a multiple of "
                   "64 KiB in (0, 16 MiB]", s->size);
        return;
    }

    if (s->version != 1 && s->version != 2) {
        error_setg(errp, "xm-sfc: version must be 1 or 2");
        return;
    }
    if (!s->jedec) {
        /* The XM510 (v1) kernel's table has no 16 MiB part at all. */
        s->jedec = s->version == 2 && s->size > 8 * MiB ? 0xef4018 : 0xef4017;
    }
    memory_region_init_io(&s->regs, OBJECT(s),
                          s->version == 2 ? &xm_sfc_v2_regs_ops
                                          : &xm_sfc_regs_ops,
                          s, "xm-sfc.regs",
                          s->version == 2 ? V2_REGS_SIZE : REGS_SIZE);
    sysbus_init_mmio(sbd, &s->regs);

    memory_region_init_rom_device(&s->window, OBJECT(s), &xm_sfc_window_ops,
                                  s, "xm-sfc.window", s->size, errp);
    if (*errp) {
        return;
    }
    s->array = memory_region_get_ram_ptr(&s->window);
    memset(s->array, 0xff, s->size);
    if (s->nand) {
        xm_sfc_nand_window(s, 0, s->size / NAND_DATA);
    } else if (s->fd >= 0 &&
        pread(s->fd, s->array, s->size, 0) != (ssize_t)s->size) {
        error_setg_errno(errp, errno, "xm-sfc: cannot read %s",
                         s->flash_file);
        return;
    }
    sysbus_init_mmio(sbd, &s->window);
    s->sr3 = s->wps ? SR3_WPS : 0;
}

static void xm_sfc_reset(DeviceState *dev)
{
    XmSfcState *s = XM_SFC(dev);

    s->mode = s->addr = s->cmd = s->srw = s->wrnum = 0;
    s->bsel = s->st = s->srr = s->cache_cmd = 0;
    s->sr1 = s->sr2 = 0;
    /* SR3 is non-volatile: what the guest wrote stays. The per-block locks
     * are set again at every power-up while WPS is on. */
    s->blocks_locked = s->sr3 & SR3_WPS;
    memset(s->v2_regs, 0, sizeof(s->v2_regs));
    memset(s->v2_ram, 0, sizeof(s->v2_ram));
    xm_sfc_nand_reset_features(s);
}

static const Property xm_sfc_props[] = {
    DEFINE_PROP_STRING("flash-file", XmSfcState, flash_file),
    DEFINE_PROP_UINT32("size", XmSfcState, size, 8 * MiB),
    /* 0: the Winbond part of the image's size, from the vendor tables --
     * W25Q64CV (8 MiB, both versions) or W25Q128BV (16 MiB, v2 only). */
    DEFINE_PROP_UINT32("jedec", XmSfcState, jedec, 0),
    DEFINE_PROP_UINT32("version", XmSfcState, version, 1),
    /* Power up as a Xiongmai bootloader leaves the part: SR3.WPS set, every
     * block locked, erases and programs dropped until 0x98 or WPS cleared. */
    DEFINE_PROP_BOOL("wps", XmSfcState, wps, false),
    /* SPI NAND instead of NOR (v2): raw pages, 2048 + 64 bytes each */
    DEFINE_PROP_STRING("nand-file", XmSfcState, nand_file),
    /* 0: W25N01GV (EF AA 21) */
    DEFINE_PROP_UINT32("nand-id", XmSfcState, nand_id, 0),
};

static void xm_sfc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xm_sfc_realize;
    device_class_set_legacy_reset(dc, xm_sfc_reset);
    device_class_set_props(dc, xm_sfc_props);
}

static const TypeInfo xm_sfc_info = {
    .name          = TYPE_XM_SFC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XmSfcState),
    .class_init    = xm_sfc_class_init,
};

static void xm_sfc_register_types(void)
{
    type_register_static(&xm_sfc_info);
}

type_init(xm_sfc_register_types)
