/*
 * SPI NOR flash, as a byte-wise PIO master sees it.
 *
 * The Anyka SPI controller (ak-spi.c) has no notion of command phases:
 * the vendor drivers shift the opcode, the address and the dummy byte of a
 * fast read out as ordinary TX bytes, then turn the bus round and clock the
 * data in.  QEMU's m25p80 counts fast-read dummies in clock cycles for some
 * vendors and not at all for others (GigaDevice, which most of these
 * cameras carry), so with it the data comes back shifted.  This model takes
 * every dummy as one byte, which is what eight clocks on one line are and
 * what the drivers send, whatever the data phase's width.
 *
 * It answers to any JEDEC ID: the vendor kernels match the flash against
 * the descriptor in the boot header (SPIP, hw/arm/anyka.c), and parts like
 * the ZB25VQ64 or XM25QH64A have no m25p80 model.  The command set is the
 * common 3-byte-address one plus 4-byte mode for the parts above 16 MiB.
 * Erases and programs are written back to the flash-file.  The file is
 * the flash, so a machine with one cannot be migrated or snapshotted
 * (the destination would reopen it while the source still writes it);
 * without one, the contents travel in the migration stream.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "migration/blocker.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_AK_SPINOR "ak-spinor"
OBJECT_DECLARE_SIMPLE_TYPE(AkSpinorState, AK_SPINOR)

#define SR1_WIP         (1u << 0)
#define SR1_WEL         (1u << 1)
#define AK_SPINOR_PAGE  256

typedef enum {
    ST_CMD,         /* next byte is an opcode */
    ST_ADDR,        /* collecting address bytes */
    ST_DUMMY,       /* swallowing dummy bytes */
    ST_DATA,        /* data phase of the opcode */
    ST_IGNORE,      /* nothing more to say until CS goes away */
} AkSpinorPhase;

struct AkSpinorState {
    SSIPeripheral parent_obj;

    char *flash_file;
    uint32_t jedec;
    uint32_t size;
    int fd;
    uint8_t *array;
    Error *migration_blocker;

    uint8_t phase;
    uint8_t cmd;
    uint8_t addr_bytes;
    uint8_t addr_need;
    uint8_t dummy_need;
    uint32_t addr;
    uint32_t pos;           /* bytes into the data phase */
    uint8_t sr[3];
    bool four_byte;
    /* Range a program touched, written back when CS goes away. */
    uint32_t dirty_lo, dirty_hi;
};

/*
 * The image is the flash: a change the guest sees must be on disk too.  A
 * write that cannot be completed ends the run rather than letting the
 * guest go on as if it had worked (as gk-sflash.c does).
 */
static void ak_spinor_persist(AkSpinorState *s, uint32_t off, uint32_t len)
{
    while (s->fd >= 0 && len) {
        ssize_t n = pwrite(s->fd, s->array + off, len, off);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_report("ak-spinor: write-through to %s failed at 0x%x: %s",
                         s->flash_file, off,
                         n < 0 ? strerror(errno) : "no progress");
            exit(1);
        }
        off += n;
        len -= n;
    }
}

static void ak_spinor_erase(AkSpinorState *s, uint32_t addr, uint32_t len)
{
    addr &= ~(len - 1) & (s->size - 1);
    memset(s->array + addr, 0xFF, len);
    ak_spinor_persist(s, addr, len);
}

static uint32_t ak_spinor_addr_len(AkSpinorState *s)
{
    return s->four_byte ? 4 : 3;
}

/* What an opcode needs after it: address bytes, dummy bytes. */
static void ak_spinor_decode(AkSpinorState *s, uint8_t cmd)
{
    s->cmd = cmd;
    s->addr = 0;
    s->addr_bytes = 0;
    s->addr_need = 0;
    s->dummy_need = 0;
    s->pos = 0;

    switch (cmd) {
    case 0x03:                      /* read */
    case 0x02: case 0x32:           /* page program, quad page program */
    case 0x20: case 0x52: case 0xD8: /* 4K / 32K / 64K erase */
        s->addr_need = ak_spinor_addr_len(s);
        break;
    case 0x0B: case 0x3B: case 0x6B: /* fast / dual / quad output read */
    case 0x5A:                      /* SFDP */
        s->addr_need = cmd == 0x5A ? 3 : ak_spinor_addr_len(s);
        s->dummy_need = 1;
        break;
    case 0xBB:                      /* dual I/O: mode byte */
        s->addr_need = ak_spinor_addr_len(s);
        s->dummy_need = 1;
        break;
    case 0xEB:                      /* quad I/O: mode byte + 4 clocks */
        s->addr_need = ak_spinor_addr_len(s);
        s->dummy_need = 3;
        break;
    case 0x90:                      /* manufacturer / device ID */
        s->addr_need = 3;
        break;
    case 0xAB:                      /* release power-down / device ID */
        s->dummy_need = 3;
        break;
    case 0x4B:                      /* unique ID */
        s->dummy_need = 4;
        break;
    case 0x06:
        s->sr[0] |= SR1_WEL;
        s->phase = ST_IGNORE;
        return;
    case 0x04:
        s->sr[0] &= ~SR1_WEL;
        s->phase = ST_IGNORE;
        return;
    case 0x60: case 0xC7:           /* chip erase */
        if (s->sr[0] & SR1_WEL) {
            memset(s->array, 0xFF, s->size);
            ak_spinor_persist(s, 0, s->size);
            s->sr[0] &= ~SR1_WEL;
        }
        s->phase = ST_IGNORE;
        return;
    case 0xB7:
        s->four_byte = true;
        s->phase = ST_IGNORE;
        return;
    case 0xE9:
        s->four_byte = false;
        s->phase = ST_IGNORE;
        return;
    case 0x9F: case 0x05: case 0x35: case 0x15:
    case 0x01: case 0x31: case 0x11:
        break;
    case 0x50: case 0x66: case 0x99: case 0xB9:
        s->phase = ST_IGNORE;
        return;
    default:
        qemu_log_mask(LOG_UNIMP, "ak-spinor: opcode 0x%02x\n", cmd);
        s->phase = ST_IGNORE;
        return;
    }
    s->phase = s->addr_need ? ST_ADDR : s->dummy_need ? ST_DUMMY : ST_DATA;
}

/* The address is in: commands that act on it alone do so now. */
static void ak_spinor_addressed(AkSpinorState *s)
{
    static const uint32_t erase_len[] = {
        [0x20] = 4 * KiB, [0x52] = 32 * KiB, [0xD8] = 64 * KiB,
    };

    switch (s->cmd) {
    case 0x20: case 0x52: case 0xD8:
        if (s->sr[0] & SR1_WEL) {
            ak_spinor_erase(s, s->addr, erase_len[s->cmd]);
            s->sr[0] &= ~SR1_WEL;
        }
        s->phase = ST_IGNORE;
        return;
    default:
        break;
    }
    s->phase = s->dummy_need ? ST_DUMMY : ST_DATA;
}

static uint8_t ak_spinor_data(AkSpinorState *s, uint8_t in)
{
    uint32_t a;
    uint8_t out = 0xFF;

    switch (s->cmd) {
    case 0x03: case 0x0B: case 0x3B: case 0x6B: case 0xBB: case 0xEB:
        out = s->array[(s->addr + s->pos) & (s->size - 1)];
        break;
    case 0x02: case 0x32:
        if (s->sr[0] & SR1_WEL) {
            /* Within the page: the address wraps at its end. */
            a = (s->addr & ~(AK_SPINOR_PAGE - 1) & (s->size - 1)) |
                ((s->addr + s->pos) & (AK_SPINOR_PAGE - 1));
            s->array[a] &= in;
            s->dirty_lo = MIN(s->dirty_lo, a);
            s->dirty_hi = MAX(s->dirty_hi, a + 1);
        }
        break;
    case 0x9F:
        out = s->jedec >> (8 * (2 - s->pos % 3));
        break;
    case 0x90:
        out = (s->pos + s->addr) & 1 ? s->jedec : s->jedec >> 16;
        break;
    case 0xAB:
        out = (s->jedec & 0xFF) - 1;
        break;
    case 0x4B:
        out = s->pos;
        break;
    case 0x05:
        out = s->sr[0];
        break;
    case 0x35:
        out = s->sr[1];
        break;
    case 0x15:
        out = s->sr[2];
        break;
    case 0x01: case 0x31: case 0x11:
        if (s->sr[0] & SR1_WEL) {
            unsigned n = s->cmd == 0x01 ? s->pos : s->cmd == 0x31 ? 1 : 2;

            if (n < 3) {
                s->sr[n] = n ? in : (in & ~(SR1_WIP | SR1_WEL)) | SR1_WEL;
            }
        }
        break;
    case 0x5A:
    default:
        break;
    }
    s->pos++;
    return out;
}

static uint32_t ak_spinor_transfer(SSIPeripheral *dev, uint32_t val)
{
    AkSpinorState *s = AK_SPINOR(dev);
    uint8_t in = val;

    switch (s->phase) {
    case ST_CMD:
        ak_spinor_decode(s, in);
        return 0xFF;
    case ST_ADDR:
        s->addr = (s->addr << 8) | in;
        if (++s->addr_bytes == s->addr_need) {
            ak_spinor_addressed(s);
        }
        return 0xFF;
    case ST_DUMMY:
        if (!--s->dummy_need) {
            s->phase = ST_DATA;
        }
        return 0xFF;
    case ST_DATA:
        return ak_spinor_data(s, in);
    default:
        return 0xFF;
    }
}

static int ak_spinor_set_cs(SSIPeripheral *dev, bool select)
{
    AkSpinorState *s = AK_SPINOR(dev);

    /* select = true is CS deasserted (SSI_CS_LOW): end of a command. */
    if (select) {
        if (s->phase == ST_DATA &&
            (s->cmd == 0x02 || s->cmd == 0x32 ||
             s->cmd == 0x01 || s->cmd == 0x31 || s->cmd == 0x11)) {
            s->sr[0] &= ~SR1_WEL;
        }
        if (s->dirty_hi > s->dirty_lo) {
            ak_spinor_persist(s, s->dirty_lo, s->dirty_hi - s->dirty_lo);
        }
        s->dirty_lo = UINT32_MAX;
        s->dirty_hi = 0;
        s->phase = ST_CMD;
    }
    return 0;
}

static void ak_spinor_reset(DeviceState *dev)
{
    AkSpinorState *s = AK_SPINOR(dev);

    s->phase = ST_CMD;
    s->sr[0] = 0;
    s->four_byte = false;
    s->dirty_lo = UINT32_MAX;
    s->dirty_hi = 0;
}

static void ak_spinor_realize(SSIPeripheral *dev, Error **errp)
{
    AkSpinorState *s = AK_SPINOR(dev);
    off_t len;

    s->fd = -1;
    if (s->flash_file && *s->flash_file) {
        s->fd = open(s->flash_file, O_RDWR);
        if (s->fd < 0) {
            error_setg_errno(errp, errno, "ak-spinor: cannot open %s",
                             s->flash_file);
            return;
        }
        /*
         * Two machines on one image would each write back from their
         * own copy, undoing the other's erases.
         */
        if (qemu_lock_fd(s->fd, 0, 0, true)) {
            error_setg(errp, "ak-spinor: %s is in use by another process",
                       s->flash_file);
            close(s->fd);
            s->fd = -1;
            return;
        }
        len = lseek(s->fd, 0, SEEK_END);
        if (len < 64 * KiB || len > 128 * MiB || (len & (len - 1))) {
            error_setg(errp, "ak-spinor: %s is not a power-of-two size "
                       "from 64 KiB to 128 MiB", s->flash_file);
            return;
        }
        error_setg(&s->migration_blocker, "ak-spinor: flash-file %s is "
                   "written through; it cannot be migrated", s->flash_file);
        if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
            return;
        }
        s->size = len;
    }
    /* At least one 64 KiB erase block, so no erase reaches past it. */
    if (s->size < 64 * KiB || s->size > 128 * MiB ||
        (s->size & (s->size - 1))) {
        error_setg(errp, "ak-spinor: size must be a power of two from "
                   "64 KiB to 128 MiB");
        return;
    }
    s->array = g_malloc(s->size);
    memset(s->array, 0xFF, s->size);
    if (s->fd >= 0 && pread(s->fd, s->array, s->size, 0) != s->size) {
        error_setg_errno(errp, errno, "ak-spinor: cannot read %s",
                         s->flash_file);
    }
}

static const Property ak_spinor_properties[] = {
    DEFINE_PROP_STRING("flash-file", AkSpinorState, flash_file),
    DEFINE_PROP_UINT32("jedec", AkSpinorState, jedec, 0xC84017),
    DEFINE_PROP_UINT32("size", AkSpinorState, size, 8 * MiB),
};

static bool ak_spinor_array_needed(void *opaque)
{
    AkSpinorState *s = opaque;

    return s->fd < 0;
}

static const VMStateDescription vmstate_ak_spinor_array = {
    .name = TYPE_AK_SPINOR "/array",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = ak_spinor_array_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_VBUFFER_UINT32(array, AkSpinorState, 1, NULL, size),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ak_spinor = {
    .name = TYPE_AK_SPINOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, AkSpinorState),
        VMSTATE_UINT8(phase, AkSpinorState),
        VMSTATE_UINT8(cmd, AkSpinorState),
        VMSTATE_UINT8(addr_bytes, AkSpinorState),
        VMSTATE_UINT8(addr_need, AkSpinorState),
        VMSTATE_UINT8(dummy_need, AkSpinorState),
        VMSTATE_UINT32(addr, AkSpinorState),
        VMSTATE_UINT32(pos, AkSpinorState),
        VMSTATE_UINT8_ARRAY(sr, AkSpinorState, 3),
        VMSTATE_BOOL(four_byte, AkSpinorState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_ak_spinor_array,
        NULL
    }
};

static void ak_spinor_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    device_class_set_legacy_reset(dc, ak_spinor_reset);
    device_class_set_props(dc, ak_spinor_properties);
    dc->vmsd = &vmstate_ak_spinor;
    k->realize = ak_spinor_realize;
    k->transfer = ak_spinor_transfer;
    k->set_cs = ak_spinor_set_cs;
    k->cs_polarity = SSI_CS_LOW;
}

static const TypeInfo ak_spinor_info = {
    .name          = TYPE_AK_SPINOR,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(AkSpinorState),
    .class_init    = ak_spinor_class_init,
};

static void ak_spinor_register_types(void)
{
    type_register_static(&ak_spinor_info);
}

type_init(ak_spinor_register_types)
