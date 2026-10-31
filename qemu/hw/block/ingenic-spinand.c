/*
 * SPI NAND flash on an SSI bus: the generic command set the Ingenic
 * SFC NAND drivers (U-Boot SPL and U-Boot jz_sfc_nand, Linux 3.10
 * jz_sfc_nand, Linux 4.4 ingenic_sfc_v2 nand_device/) drive, with a
 * table of parts they all know:
 *
 *   w25n01gv      Winbond  EF AA 21  2048 + 64,  1024 blocks (1 Gbit)
 *   gd5f1gq4ub    GigaDev  C8 D1     2048 + 128, 1024 blocks (1 Gbit)
 *   gd5f2gq4ub    GigaDev  C8 D2     2048 + 128, 2048 blocks (2 Gbit)
 *   mx35lf2ge4ab  Macronix C2 22     2048 + 64,  2048 blocks, 2 planes
 *
 * Commands (one per chip-select assertion):
 *
 *   9F         READ ID.  All four answer after one address/dummy byte
 *              (00); without it the first byte out is that dummy slot,
 *              00, then the ID.  The drivers try both (addr_len 0, 1).
 *   13 r r r   PAGE READ to the cache, row = 24-bit page number
 *   03/0B/3B/6B c c + 8 dummy clocks: READ FROM CACHE at column c.
 *              The column is 12 bits for a 2 KiB page; the bit above is
 *              the plane select of a two-plane part (the 4.4 driver sets
 *              it from the page number for every part, so a one-plane
 *              part ignores it).  Data runs to the end of page + OOB.
 *   02/32 c c  PROGRAM LOAD: the cache is set to FF, then loaded from c
 *   84/34 c c  PROGRAM LOAD RANDOM DATA: the cache is kept
 *   10 r r r   PROGRAM EXECUTE: page &= cache (bits only go 1 -> 0)
 *   D8 r r r   BLOCK ERASE (64 pages)
 *   0F a / 1F a v  GET / SET FEATURE: A0 protection, B0 configuration
 *              (ECC enable bit 4, BUF bit 3 on Winbond, QE bit 0 on
 *              GigaDevice), C0 status (OIP 0, WEL 1, E_FAIL 2, P_FAIL 3,
 *              ECC 5:4 -- always "no error"), D0/F0 storage.  A GET
 *              FEATURE keeps clocking the register out, which is what
 *              a status-poll phase of the SFC reads.
 *   06 / 04    WRITE ENABLE / DISABLE; program execute and erase need
 *              WEL and clear it.  FF RESET.
 *
 * Dummy clocks are one transfer each, as in m25p80 and as the SFC model
 * sends DMYBITS.  Operations complete at once; with busy-polls = n the
 * first n status reads after a page read, program or erase still show
 * OIP, to exercise the drivers' polls.
 *
 * The protection register comes up with the block-protect bits set (as
 * on the parts); with any of BP bits 6:3 set, program and erase fail
 * with P_FAIL / E_FAIL.  Every driver clears it first.  ECC is on-die
 * and never sees an error; the OOB is plain storage.  All blocks are
 * good: a blank part reads FF everywhere, bad-block markers included.
 *
 * Backing store ("drive"): either the whole raw array, page by page
 * (data then OOB, pages * (page + oob) bytes), or the data alone
 * (pages * page bytes), in which case the OOB lives in RAM and reads
 * FF after every start.  Programs and erases are written through.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qemu/host-utils.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"
#include "qom/object.h"

#define TYPE_INGENIC_SPINAND "ingenic-spinand"
OBJECT_DECLARE_TYPE(SpiNandState, SpiNandClass, INGENIC_SPINAND)

typedef struct SpiNandPart {
    const char *name;
    uint8_t id[3];
    uint32_t page_size, oob_size, pages_per_block, blocks;
    uint8_t planes;
    uint8_t prot_reset, cfg_reset;
} SpiNandPart;

static const SpiNandPart spinand_parts[] = {
    { "w25n01gv",     { 0xef, 0xaa, 0x21 }, 2048, 64,  64, 1024, 1,
      0x7c, 0x18 },
    { "gd5f1gq4ub",   { 0xc8, 0xd1, 0x00 }, 2048, 128, 64, 1024, 1,
      0x38, 0x10 },
    { "gd5f2gq4ub",   { 0xc8, 0xd2, 0x00 }, 2048, 128, 64, 2048, 1,
      0x38, 0x10 },
    { "mx35lf2ge4ab", { 0xc2, 0x22, 0x00 }, 2048, 64,  64, 2048, 2,
      0x38, 0x10 },
};

#define FEAT_PROT   0xa
#define FEAT_CFG    0xb
#define FEAT_STATUS 0xc

#define ST_OIP      (1u << 0)
#define ST_WEL      (1u << 1)
#define ST_EFAIL    (1u << 2)
#define ST_PFAIL    (1u << 3)
#define PROT_BP     0x78

#define DUMMY_CLOCKS 8
#define MAX_RAW     (4096 + 256)

enum { ST_CMD, ST_ADDR, ST_DUMMY, ST_DATA, ST_IGNORE };

struct SpiNandClass {
    SSIPeripheralClass parent_class;
    const SpiNandPart *part;
};

struct SpiNandState {
    SSIPeripheral parent_obj;

    BlockBackend *blk;
    uint32_t busy_polls;

    const SpiNandPart *part;
    uint32_t page_size, oob_size, ppb;
    uint32_t raw;           /* page + oob */
    uint32_t pages;
    uint32_t col_bits;
    bool file_oob;          /* the backing file has the OOB */
    uint8_t *array;         /* pages * raw */

    uint8_t cache[MAX_RAW];
    uint8_t feat[16];
    uint32_t busy;          /* status reads left showing OIP */
    uint32_t row;           /* the page last read into the cache */

    /* the command in progress */
    uint8_t state, cmd, nbytes;
    uint32_t addr;
    uint32_t pos;
};

static uint8_t *spinand_page(SpiNandState *s, uint32_t row)
{
    return s->array + (uint64_t)(row % s->pages) * s->raw;
}

/* Write pages [row, row + n) back to the file. */
static void spinand_sync(SpiNandState *s, uint32_t row, uint32_t n)
{
    uint32_t i;

    if (!s->blk || !blk_is_writable(s->blk)) {
        return;
    }
    row %= s->pages;
    if (s->file_oob) {
        blk_pwrite(s->blk, (int64_t)row * s->raw, (int64_t)n * s->raw,
                   spinand_page(s, row), 0);
        return;
    }
    for (i = 0; i < n; i++) {
        blk_pwrite(s->blk, (int64_t)(row + i) * s->part->page_size,
                   s->part->page_size, spinand_page(s, row + i), 0);
    }
}

static void spinand_set_busy(SpiNandState *s)
{
    s->busy = s->busy_polls;
}

static uint8_t spinand_status(SpiNandState *s)
{
    uint8_t st = s->feat[FEAT_STATUS];

    if (s->busy) {
        s->busy--;
        st |= ST_OIP;
    }
    return st;
}

static bool spinand_locked(SpiNandState *s)
{
    return (s->feat[FEAT_PROT] & PROT_BP) != 0;
}

/*
 * The column of a cache access: the page offset, less the plane bit.  A
 * two-plane part reads the cache of the plane the bit names, which must
 * be the plane (block number parity) of the page last read.
 */
static uint32_t spinand_column(SpiNandState *s, uint32_t col, bool read)
{
    uint32_t plane = (col >> s->col_bits) & 1;

    if (read && s->part->planes > 1 &&
        plane != ((s->row / s->part->pages_per_block) & 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read from cache with plane %u "
                      "after a page read of plane %u\n", s->part->name,
                      plane, (s->row / s->part->pages_per_block) & 1);
    }
    return col & ((1u << s->col_bits) - 1);
}

static void spinand_page_read(SpiNandState *s, uint32_t row)
{
    s->row = row % s->pages;
    memcpy(s->cache, spinand_page(s, row), s->raw);
    spinand_set_busy(s);
}

static void spinand_program(SpiNandState *s, uint32_t row)
{
    uint8_t *page = spinand_page(s, row);
    uint32_t i;

    s->feat[FEAT_STATUS] &= ~ST_PFAIL;
    if (!(s->feat[FEAT_STATUS] & ST_WEL)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: program execute of page %u "
                      "without WEL\n", s->part->name, row);
        return;
    }
    s->feat[FEAT_STATUS] &= ~ST_WEL;
    spinand_set_busy(s);
    if (spinand_locked(s)) {
        s->feat[FEAT_STATUS] |= ST_PFAIL;
        return;
    }
    for (i = 0; i < s->raw; i++) {
        page[i] &= s->cache[i];
    }
    spinand_sync(s, row, 1);
}

static void spinand_erase(SpiNandState *s, uint32_t row)
{
    uint32_t ppb = s->part->pages_per_block;

    row = (row % s->pages) & ~(ppb - 1);
    s->feat[FEAT_STATUS] &= ~ST_EFAIL;
    if (!(s->feat[FEAT_STATUS] & ST_WEL)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: erase of block %u without "
                      "WEL\n", s->part->name, row / ppb);
        return;
    }
    s->feat[FEAT_STATUS] &= ~ST_WEL;
    spinand_set_busy(s);
    if (spinand_locked(s)) {
        s->feat[FEAT_STATUS] |= ST_EFAIL;
        return;
    }
    memset(spinand_page(s, row), 0xff, (size_t)ppb * s->raw);
    spinand_sync(s, row, ppb);
}

static void spinand_reset_state(SpiNandState *s)
{
    memset(s->feat, 0, sizeof(s->feat));
    s->feat[FEAT_PROT] = s->part->prot_reset;
    s->feat[FEAT_CFG] = s->part->cfg_reset;
    s->busy = 0;
    s->state = ST_CMD;
}

/* Chip select released: the commands that act on it do so now. */
static int spinand_cs(SSIPeripheral *ss, bool deselect)
{
    SpiNandState *s = INGENIC_SPINAND(ss);

    if (!deselect) {
        return 0;
    }
    if (s->state == ST_DATA) {     /* all three row bytes are in */
        switch (s->cmd) {
        case 0x13:
            spinand_page_read(s, s->addr);
            break;
        case 0x10:
            spinand_program(s, s->addr);
            break;
        case 0xd8:
            spinand_erase(s, s->addr);
            break;
        }
    }
    s->state = ST_CMD;
    return 0;
}

/* The phase after the address bytes. */
static void spinand_addr_done(SpiNandState *s)
{
    switch (s->cmd) {
    case 0x03: case 0x0b: case 0x3b: case 0x6b:
        s->pos = spinand_column(s, s->addr, true);
        s->nbytes = DUMMY_CLOCKS;
        s->state = ST_DUMMY;
        return;
    case 0x02: case 0x32:
        memset(s->cache, 0xff, s->raw);
        /* fall through */
    case 0x84: case 0x34:
        s->pos = spinand_column(s, s->addr, false);
        break;
    case 0x0f: case 0x1f:
        s->addr = (s->addr >> 4) & 0xf;
        break;
    }
    s->state = ST_DATA;
}

static void spinand_start(SpiNandState *s, uint8_t cmd)
{
    s->cmd = cmd;
    s->addr = 0;
    s->pos = 0;
    s->state = ST_ADDR;
    switch (cmd) {
    case 0x9f:              /* the dummy byte is optional: see below */
        s->state = ST_DATA;
        return;
    case 0x13: case 0x10: case 0xd8:
        s->nbytes = 3;
        return;
    case 0x03: case 0x0b: case 0x3b: case 0x6b:
    case 0x02: case 0x32: case 0x84: case 0x34:
        s->nbytes = 2;
        return;
    case 0x0f: case 0x1f:
        s->nbytes = 1;
        return;
    case 0x06:
        s->feat[FEAT_STATUS] |= ST_WEL;
        break;
    case 0x04:
        s->feat[FEAT_STATUS] &= ~ST_WEL;
        break;
    case 0xff:
        s->feat[FEAT_STATUS] = 0;
        s->busy = 0;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: command 0x%02x\n", s->part->name, cmd);
        break;
    }
    s->state = ST_IGNORE;
}

/*
 * READ ID: the parts put out the ID after one address/dummy byte.  The
 * host's first byte after the opcode is that byte whether it sends an
 * address (it gets 00 back) or reads (it gets the dummy slot, 00); the ID
 * follows either way.
 */
static uint8_t spinand_read_id(SpiNandState *s)
{
    uint32_t i = s->pos++;

    if (i == 0) {
        return 0x00;
    }
    i--;
    return i < 3 ? s->part->id[i] : 0x00;
}

static uint32_t spinand_transfer(SSIPeripheral *ss, uint32_t tx)
{
    SpiNandState *s = INGENIC_SPINAND(ss);
    uint8_t rx = 0xff;

    switch (s->state) {
    case ST_CMD:
        spinand_start(s, tx);
        return 0xff;
    case ST_ADDR:
        s->addr = (s->addr << 8) | (tx & 0xff);
        if (--s->nbytes == 0) {
            spinand_addr_done(s);
        }
        return 0xff;
    case ST_DUMMY:
        if (--s->nbytes == 0) {
            s->state = ST_DATA;
        }
        return 0xff;
    case ST_IGNORE:
        return 0xff;
    case ST_DATA:
        break;
    }

    switch (s->cmd) {
    case 0x9f:
        return spinand_read_id(s);
    case 0x0f:
        if (s->addr == FEAT_STATUS) {
            return spinand_status(s);
        }
        return s->feat[s->addr];
    case 0x1f:
        if (s->addr == FEAT_STATUS) {
            break;          /* read only */
        }
        s->feat[s->addr] = tx;
        s->state = ST_IGNORE;
        break;
    case 0x03: case 0x0b: case 0x3b: case 0x6b:
        if (s->pos < s->raw) {
            rx = s->cache[s->pos++];
        }
        break;
    case 0x02: case 0x32: case 0x84: case 0x34:
        if (s->pos < s->raw) {
            s->cache[s->pos++] = tx;
        }
        break;
    case 0x13: case 0x10: case 0xd8:
        break;              /* extra address bytes: ignored */
    }
    return rx;
}

static void spinand_realize(SSIPeripheral *ss, Error **errp)
{
    SpiNandState *s = INGENIC_SPINAND(ss);
    const SpiNandPart *p = s->part;
    uint64_t data_size, raw_size;

    data_size = (uint64_t)s->pages * p->page_size;
    raw_size = (uint64_t)s->pages * s->raw;
    s->array = g_malloc(raw_size);
    memset(s->array, 0xff, raw_size);

    if (s->blk) {
        uint64_t perm = BLK_PERM_CONSISTENT_READ |
                        (blk_supports_write_perm(s->blk) ? BLK_PERM_WRITE : 0);
        int64_t len;
        uint32_t i;

        if (blk_set_perm(s->blk, perm, BLK_PERM_ALL, errp) < 0) {
            return;
        }
        len = blk_getlength(s->blk);
        if (len == raw_size) {
            s->file_oob = true;
            if (blk_pread(s->blk, 0, raw_size, s->array, 0) < 0) {
                error_setg(errp, "%s: cannot read the image", p->name);
                return;
            }
        } else if (len == data_size) {
            s->file_oob = false;
            for (i = 0; i < s->pages; i++) {
                if (blk_pread(s->blk, (int64_t)i * p->page_size,
                              p->page_size, spinand_page(s, i), 0) < 0) {
                    error_setg(errp, "%s: cannot read the image", p->name);
                    return;
                }
            }
        } else {
            error_setg(errp, "%s: the image is %" PRId64 " bytes; it must "
                       "be %" PRIu64 " (data only) or %" PRIu64
                       " (data + OOB, %u + %u per page)", p->name, len,
                       data_size, raw_size, p->page_size, p->oob_size);
            return;
        }
    }
    spinand_reset_state(s);
}

/* The geometry, for the machine's boot ROM (read only). */
static void spinand_instance_init(Object *obj)
{
    SpiNandState *s = INGENIC_SPINAND(obj);
    const SpiNandPart *p = INGENIC_SPINAND_GET_CLASS(obj)->part;

    s->part = p;
    s->page_size = p->page_size;
    s->oob_size = p->oob_size;
    s->ppb = p->pages_per_block;
    s->raw = p->page_size + p->oob_size;
    s->pages = p->pages_per_block * p->blocks;
    s->col_bits = 32 - clz32(s->raw - 1);
    object_property_add_uint32_ptr(obj, "page-size", &s->page_size,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "oob-size", &s->oob_size,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "pages-per-block", &s->ppb,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "pages", &s->pages,
                                   OBJ_PROP_FLAG_READ);
}

static void spinand_reset(DeviceState *dev)
{
    spinand_reset_state(INGENIC_SPINAND(dev));
}

static const Property spinand_props[] = {
    DEFINE_PROP_DRIVE("drive", SpiNandState, blk),
    DEFINE_PROP_UINT32("busy-polls", SpiNandState, busy_polls, 0),
};

static const VMStateDescription vmstate_spinand = {
    .name = TYPE_INGENIC_SPINAND,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(cache, SpiNandState, MAX_RAW),
        VMSTATE_UINT8_ARRAY(feat, SpiNandState, 16),
        VMSTATE_UINT32(busy, SpiNandState),
        VMSTATE_UINT32(row, SpiNandState),
        VMSTATE_UINT8(state, SpiNandState),
        VMSTATE_UINT8(cmd, SpiNandState),
        VMSTATE_UINT8(nbytes, SpiNandState),
        VMSTATE_UINT32(addr, SpiNandState),
        VMSTATE_UINT32(pos, SpiNandState),
        VMSTATE_END_OF_LIST()
    }
};

static void spinand_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = spinand_realize;
    k->transfer = spinand_transfer;
    k->set_cs = spinand_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_spinand;
    device_class_set_props(dc, spinand_props);
    device_class_set_legacy_reset(dc, spinand_reset);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static void spinand_part_class_init(ObjectClass *klass, const void *data)
{
    INGENIC_SPINAND_CLASS(klass)->part = data;
}

static const TypeInfo spinand_info = {
    .name = TYPE_INGENIC_SPINAND,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SpiNandState),
    .instance_init = spinand_instance_init,
    .class_size = sizeof(SpiNandClass),
    .abstract = true,
    .class_init = spinand_class_init,
};

static void spinand_register_types(void)
{
    int i;

    type_register_static(&spinand_info);
    for (i = 0; i < ARRAY_SIZE(spinand_parts); i++) {
        TypeInfo ti = {
            .name = spinand_parts[i].name,
            .parent = TYPE_INGENIC_SPINAND,
            .class_init = spinand_part_class_init,
            .class_data = &spinand_parts[i],
        };

        type_register_static(&ti);
    }
}

type_init(spinand_register_types)
