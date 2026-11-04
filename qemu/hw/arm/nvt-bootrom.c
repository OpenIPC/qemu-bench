/*
 * Novatek mask ROM, as the loaders see it (NA51089, NA51055).
 *
 * The silicon's ROM picks the boot source from the strap pins, copies the
 * loader -- the first 32 KiB of the SPI NOR, or of SPI NAND block 0 -- to
 * SRAM at 0xF07C0000 and jumps to it, its entry the header's first word
 * (LD9856xA.bin: 0xF07C0028; the header's 0x24 word is the size).  The
 * ROM stays visible at 0 (32 KiB, over the first 32 KiB of DRAM) and the
 * loader calls into it through a table of function pointers at its top
 * (the loader's doRemapLZ*.o veneers, bl_func.c's ROM_LZMA_POSITION):
 *
 *   0x7FD0  lzma_inflate(in, in_size, out, out_size, tmp, tmp_size)
 *   0x7FD4  LZ_Uncompress(in, out, in_size) -> out size
 *   0x7FD8  debug_msg(str)
 *   0x7FDC  debug_msg_var(str, value)
 *   0x7FE0  debug_msg_err(str)
 *   0x7FE4  efuse_read(index) -> word
 *   0x7FEC  Dec2HexStr(value)
 *
 * NA51055's ROM ("layout" 1) has no table: its loaders call three routines
 * at fixed addresses (Remap_LZ52x.a's veneers to *ABS* symbols):
 *
 *   0x14FC  memcpy(dst, src, n)
 *   0x2068  Dec2HexStr(value)
 *   0x51F8  efuse_read(index) -> word
 *
 * NA51103's ("layout" 3) is NA51089's table and, at 0x7FB8,
 * vprintf(putc, fmt, ap) (the loader's printf); its home is 0xF0000000
 * in the loader's 32-bit view, from where the loader copies its top
 * (0x7FB4..0x7FF3) into DRAM, then the two ROM data ranges listed at
 * 0x7FC0..0x7FCC (empty here).
 *
 * NS02501's loaders ("layout" 2) call nothing in the ROM: their
 * rom_efuse_read is their own.  The ROM's part is the copy, the stack and
 * the jump.
 *
 * The ROM's own code is not available, so this one is QEMU's: a reset
 * vector branching to the loader's entry, and for each routine a stub that
 * hands its arguments (r0..r3 and two stack words) to a service window at
 * 0x7000 and returns its result.  The service does the work in C: the LZ
 * and LZMA decoders are the stand-in's (novatek.c, nvt-lzma.c), the
 * messages go to UART 0, and the efuse reads as blank (0).
 *
 * The ROM's home is 0x80000000 ("rom-base"), and it also answers at 0
 * until the remap.  The loader's DDR bring-up copies the 32 KiB from
 * 0x80000000 into DRAM at 0 (so the table keeps working once DRAM is
 * there) and switches 0 over to DRAM.  Here the image is in DRAM at 0
 * from reset -- what that copy leaves -- and read-only at 0x80000000 for
 * the copy to read.  The service window, which neither can hold, sits
 * apart at "svc-base" (an unused address in the I/O space).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/arm/nvt-bootrom.h"
#include "hw/arm/nvt-lzma.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"

#define ROM_SIZE        0x8000
#define ROM_SVC_SIZE    0x100
#define ROM_TABLE       0x7FD0
/*
 * The table's stubs sit just below it, clear of the low words the loaders
 * use as scratch: NA51090's start code keeps the data cache's set and way
 * counts at 0x100 and 0x10 of the ROM's copy in DRAM.
 */
#define ROM_STUBS       0x7C00

enum {
    SVC_ARG0 = 0x00,    /* r0..r3, [sp], [sp + 4] at 0x00..0x14 */
    SVC_CMD = 0x20,
    SVC_RET = 0x24,
};

enum {
    CMD_LZMA = 1, CMD_LZ, CMD_MSG, CMD_MSG_VAR, CMD_MSG_ERR, CMD_EFUSE,
    CMD_HEX, CMD_MEMCPY, CMD_VPRINTF,
};

/* NA51055: routine address -> service command */
static const struct {
    uint32_t addr;
    uint8_t cmd;
} rom_fixed_51055[] = {
    { 0x14FC, CMD_MEMCPY },
    { 0x2068, CMD_HEX },
    { 0x51F8, CMD_EFUSE },
};

/* table slot -> service command (0x7FE8 is unused by the loaders) */
static const uint8_t rom_slots[] = {
    CMD_LZMA, CMD_LZ, CMD_MSG, CMD_MSG_VAR, CMD_MSG_ERR, CMD_EFUSE, 0,
    CMD_HEX,
};

#define LZ_MAX          (32 * MiB)

static void rom_putc(NvtBootRomState *s, uint8_t c)
{
    address_space_stb(&s->as, s->uart, c,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static void rom_puts(NvtBootRomState *s, const char *str)
{
    while (*str) {
        rom_putc(s, *str++);
    }
}

static void rom_put_guest_str(NvtBootRomState *s, uint32_t addr)
{
    uint8_t c;
    int n;

    for (n = 0; n < 256; n++) {
        if (dma_memory_read(&s->as, addr + n, &c, 1,
                            MEMTXATTRS_UNSPECIFIED) || !c) {
            return;
        }
        rom_putc(s, c);
    }
}

static void rom_put_hex(NvtBootRomState *s, uint32_t v)
{
    char buf[16];

    snprintf(buf, sizeof(buf), "%08X", v);
    rom_puts(s, buf);
}

/* LZ_Uncompress: as novatek.c's, but the output length is what it makes */
static uint32_t rom_lz(NvtBootRomState *s, uint32_t in_addr, uint32_t out_addr,
                       uint32_t in_size)
{
    g_autofree uint8_t *in = NULL, *out = NULL;
    uint32_t inpos = 1, outpos = 0, len, off, i, cap = 1 * MiB;
    uint8_t marker, sym;

    if (!in_size || in_size > LZ_MAX) {
        return 0;
    }
    in = g_malloc(in_size);
    if (dma_memory_read(&s->as, in_addr, in, in_size,
                        MEMTXATTRS_UNSPECIFIED)) {
        return 0;
    }
    out = g_malloc(cap);
    marker = in[0];

#define NEED(n) do {                                    \
        while ((n) > cap - outpos) {                    \
            if (cap >= LZ_MAX) {                        \
                goto done;                              \
            }                                           \
            cap *= 2;                                   \
            out = g_realloc(out, cap);                  \
        }                                               \
    } while (0)

    while (inpos < in_size) {
        sym = in[inpos++];
        if (sym != marker) {
            NEED(1);
            out[outpos++] = sym;
        } else if (inpos < in_size && in[inpos] == 0) {
            NEED(1);
            out[outpos++] = marker;
            inpos++;
        } else {
            uint32_t b;

            len = off = 0;
            do {
                b = inpos < in_size ? in[inpos++] : 0;
                len = (len << 7) | (b & 0x7F);
            } while (b & 0x80 && inpos < in_size);
            do {
                b = inpos < in_size ? in[inpos++] : 0;
                off = (off << 7) | (b & 0x7F);
            } while (b & 0x80 && inpos < in_size);
            if (!off || off > outpos || len > LZ_MAX - outpos) {
                break;
            }
            NEED(len);
            for (i = 0; i < len; i++, outpos++) {
                out[outpos] = out[outpos - off];
            }
        }
    }
done:
#undef NEED
    dma_memory_write(&s->as, out_addr, out, outpos,
                     MEMTXATTRS_UNSPECIFIED);
    return outpos;
}

/*
 * lzma_inflate() returns the length it unpacked (0 on failure): NA51090's
 * loader checks it against its header's length; the SDK loaders ignore it.
 */
static uint32_t rom_lzma(NvtBootRomState *s, const uint32_t *a)
{
    uint32_t in_addr = a[0], in_size = a[1], out_addr = a[2];
    uint32_t out_size = a[3];
    g_autofree uint8_t *in = NULL, *out = NULL;
    ssize_t n;

    if (!in_size || in_size > LZ_MAX || !out_size || out_size > LZ_MAX ||
        dma_memory_read(&s->as, in_addr, in = g_malloc(in_size), in_size,
                        MEMTXATTRS_UNSPECIFIED)) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-bootrom: LZMA 0x%x bytes at 0x%x "
                      "to 0x%x bytes at 0x%x cannot be read\n", in_size,
                      in_addr, out_size, out_addr);
        return 0;
    }
    out = g_malloc(out_size);
    n = nvt_lzma_decode(in, in_size, out, out_size);
    if (n <= 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-bootrom: LZMA stream at 0x%x "
                      "does not decode\n", in_addr);
        return 0;
    }
    dma_memory_write(&s->as, out_addr, out, n,
                     MEMTXATTRS_UNSPECIFIED);
    return n;
}

static uint32_t rom_word(NvtBootRomState *s, uint32_t addr)
{
    uint32_t v = 0;

    dma_memory_read(&s->as, addr, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/*
 * NA51103's vprintf(putc, fmt, ap): the loader's printf hands its own
 * putc, the format and its va_list (a pointer to the words, AAPCS); this
 * one formats here and writes UART 0, as that putc would.
 */
static uint32_t rom_vprintf(NvtBootRomState *s, uint32_t fmt, uint32_t ap)
{
    GString *out = g_string_new(NULL);
    uint32_t n = 0, len;
    uint8_t c;

    for (n = 0; n < 1024; n++) {
        char spec[16] = "%";
        int k = 1, lng = 0;
        uint64_t v;

        if (dma_memory_read(&s->as, fmt + n, &c, 1, MEMTXATTRS_UNSPECIFIED) ||
            !c) {
            break;
        }
        if (c != '%') {
            g_string_append_c(out, c);
            continue;
        }
        for (;;) {
            if (dma_memory_read(&s->as, fmt + ++n, &c, 1,
                                MEMTXATTRS_UNSPECIFIED) || !c) {
                goto done;
            }
            if (c == 'l') {
                lng++;
            } else if (strchr("-+ #0123456789.", c) && k < 10) {
                spec[k++] = c;
            } else if (c != 'h' && c != 'z') {
                break;
            }
        }
        switch (c) {
        case 'd': case 'i': case 'u': case 'x': case 'X': case 'o':
            if (lng >= 2) {
                ap = (ap + 7) & ~7;
                v = rom_word(s, ap) | (uint64_t)rom_word(s, ap + 4) << 32;
                ap += 8;
            } else {
                v = rom_word(s, ap);
                ap += 4;
                if (c == 'd' || c == 'i') {
                    v = (int64_t)(int32_t)v;
                }
            }
            spec[k++] = 'l';
            spec[k++] = 'l';
            spec[k++] = c;
            spec[k] = 0;
            g_string_append_printf(out, spec, (long long)v);
            break;
        case 'p':
            g_string_append_printf(out, "0x%08x", rom_word(s, ap));
            ap += 4;
            break;
        case 'c':
            g_string_append_c(out, rom_word(s, ap));
            ap += 4;
            break;
        case 's': {
            uint32_t str = rom_word(s, ap);
            GString *t = g_string_new(NULL);
            int i;

            ap += 4;
            for (i = 0; i < 256; i++) {
                if (dma_memory_read(&s->as, str + i, &c, 1,
                                    MEMTXATTRS_UNSPECIFIED) || !c) {
                    break;
                }
                g_string_append_c(t, c);
            }
            spec[k++] = 's';
            spec[k] = 0;
            g_string_append_printf(out, spec, t->str);
            g_string_free(t, true);
            break;
        }
        default:
            g_string_append_c(out, c);
            break;
        }
    }
done:
    rom_puts(s, out->str);
    len = out->len;
    g_string_free(out, true);
    return len;
}

static void rom_service(NvtBootRomState *s, uint32_t cmd)
{
    const uint32_t *a = s->args;

    switch (cmd) {
    case CMD_LZMA:
        s->ret = rom_lzma(s, a);
        break;
    case CMD_LZ:
        s->ret = rom_lz(s, a[0], a[1], a[2]);
        break;
    case CMD_MSG:
    case CMD_MSG_ERR:
        rom_put_guest_str(s, a[0]);
        s->ret = 0;
        break;
    case CMD_MSG_VAR:
        rom_put_guest_str(s, a[0]);
        rom_puts(s, " 0x");
        rom_put_hex(s, a[1]);
        rom_puts(s, "\r\n");
        s->ret = 0;
        break;
    case CMD_EFUSE:
        s->ret = 0;             /* a blank efuse */
        break;
    case CMD_VPRINTF:
        s->ret = rom_vprintf(s, a[1], a[2]);
        break;
    case CMD_HEX:
        rom_put_hex(s, a[0]);
        s->ret = 0;
        break;
    case CMD_MEMCPY:
        if (a[2] && a[2] <= LZ_MAX) {
            g_autofree uint8_t *buf = g_malloc(a[2]);

            if (!dma_memory_read(&s->as, a[1], buf, a[2],
                                 MEMTXATTRS_UNSPECIFIED)) {
                dma_memory_write(&s->as, a[0], buf, a[2],
                                 MEMTXATTRS_UNSPECIFIED);
            }
        }
        s->ret = a[0];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "nvt-bootrom: service %u\n", cmd);
        s->ret = 0;
        break;
    }
}

static uint64_t rom_svc_read(void *opaque, hwaddr offset, unsigned size)
{
    NvtBootRomState *s = opaque;

    if (offset == SVC_RET) {
        return s->ret;
    }
    if (offset < 6 * 4) {
        return s->args[offset / 4];
    }
    return 0;
}

static void rom_svc_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    NvtBootRomState *s = opaque;

    if (offset < 6 * 4) {
        s->args[offset / 4] = val;
    } else if (offset == SVC_CMD) {
        rom_service(s, val);
    }
}

static const MemoryRegionOps rom_svc_ops = {
    .read = rom_svc_read,
    .write = rom_svc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * The ROM image: at 0 the stack and a jump to the loader's entry; from
 * ROM_STUBS one stub per table slot:
 *     ldr  ip, [pc, #off]     ; the service window
 *     str  r0..r3, [ip, #0..12]
 *     ldr  r1, [sp]       ; str r1, [ip, #16]
 *     ldr  r1, [sp, #4]   ; str r1, [ip, #20]
 *     mov  r1, #cmd       ; str r1, [ip, #32]
 *     ldr  r0, [ip, #36]  ; bx lr
 *     .word svc-base
 */
static void rom_stub(NvtBootRomState *s, uint32_t base, uint8_t cmd)
{
    static const uint32_t body[] = {
        0xE58C0000, 0xE58C1004, 0xE58C2008, 0xE58C300C,
        0xE59D1000, 0xE58C1010, 0xE59D1004, 0xE58C1014,
        0, /* mov r1, #cmd */
        0xE58C1020, 0xE59C0024, 0xE12FFF1E,
    };
    uint32_t *p = (uint32_t *)(s->image + base);
    int n = ARRAY_SIZE(body), i;

    /* ldr ip, [pc, #off]: the literal after the body */
    p[0] = cpu_to_le32(0xE59FC000 | ((n + 1) * 4 - 8));
    for (i = 0; i < n; i++) {
        p[1 + i] = cpu_to_le32(body[i] ? body[i] : 0xE3A01000 | cmd);
    }
    p[1 + n] = cpu_to_le32(s->svc_base);
}

static void rom_build(NvtBootRomState *s, uint32_t entry)
{
    uint32_t *w = (uint32_t *)s->image;
    int slot, i;

    memset(s->image, 0, ROM_SIZE);
    /*
     * Reset: the ROM's stack (BOOT_ROM_SP_LIMIT_ADDR, which NA51055's
     * loaders run on before they set their own), then the loader.
     */
    w[0] = cpu_to_le32(0xE59FD01C);         /* ldr sp, [pc, #28] (0x24) */
    w[1] = cpu_to_le32(0xE59FF014);         /* ldr pc, [pc, #20] (0x20) */
    for (i = 2; i < 8; i++) {
        w[i] = cpu_to_le32(0xEAFFFFFE);     /* the other vectors: b . */
    }
    w[8] = cpu_to_le32(entry);
    w[9] = cpu_to_le32(s->sp);
    if (s->layout == 2) {
        return;
    }
    if (s->layout == 1) {
        for (i = 0; i < ARRAY_SIZE(rom_fixed_51055); i++) {
            rom_stub(s, rom_fixed_51055[i].addr, rom_fixed_51055[i].cmd);
        }
        return;
    }
    for (slot = 0; slot < ARRAY_SIZE(rom_slots); slot++) {
        uint32_t base = ROM_STUBS + slot * 0x40;

        if (!rom_slots[slot]) {
            continue;
        }
        rom_stub(s, base, rom_slots[slot]);
        w[ROM_TABLE / 4 + slot] = cpu_to_le32(base);
    }
    if (s->layout == 3) {
        /*
         * NA51103: the table also has vprintf at 0x7FB8; 0x7FC0..0x7FCC
         * are the ROM's two data ranges (start, end) the loader copies
         * out of it, empty here.
         */
        rom_stub(s, ROM_STUBS + 0x300, CMD_VPRINTF);
        w[0x7FB8 / 4] = cpu_to_le32(ROM_STUBS + 0x300);
    }
}

static void nvt_bootrom_reset(DeviceState *dev)
{
    NvtBootRomState *s = NVT_BOOTROM(dev);
    uint32_t hdr[10], size;

    /* the loader into SRAM, as the ROM copies it */
    nvt_flash_read(s->flash, 0, hdr, sizeof(hdr));
    size = le32_to_cpu(hdr[9]);
    if (!size || size > s->sram_size) {
        size = 32 * KiB;
    }
    s->loader = g_realloc(s->loader, size);
    nvt_flash_read(s->flash, 0, s->loader, size);
    address_space_write(&s->as, s->sram_base,
                        MEMTXATTRS_UNSPECIFIED, s->loader, size);
    rom_build(s, le32_to_cpu(hdr[0]));
    address_space_write(&s->as, 0, MEMTXATTRS_UNSPECIFIED,
                        s->image, ROM_SIZE);
    memory_region_set_dirty(&s->rom, 0, ROM_SIZE);
}

static void nvt_bootrom_realize(DeviceState *dev, Error **errp)
{
    NvtBootRomState *s = NVT_BOOTROM(dev);

    if (!s->flash) {
        error_setg(errp, "nvt-bootrom: no flash");
        return;
    }
    /* the address space its loader sees (NA51103's 32-bit view) */
    address_space_init(&s->as, s->mem ? s->mem : get_system_memory(),
                       "nvt-bootrom");
    memory_region_init_io(&s->svc, OBJECT(s), &rom_svc_ops, s,
                          "nvt-bootrom.svc", ROM_SVC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->svc);
    memory_region_init_ram_ptr(&s->rom, OBJECT(s), "nvt-bootrom.rom",
                               ROM_SIZE, s->image);
    memory_region_set_readonly(&s->rom, true);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->rom);
}

static void nvt_bootrom_init(Object *obj)
{
    NvtBootRomState *s = NVT_BOOTROM(obj);

    s->image = g_malloc0(ROM_SIZE);
}

static const VMStateDescription vmstate_nvt_bootrom = {
    .name = TYPE_NVT_BOOTROM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(args, NvtBootRomState, 6),
        VMSTATE_UINT32(ret, NvtBootRomState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nvt_bootrom_properties[] = {
    DEFINE_PROP_LINK("flash", NvtBootRomState, flash, TYPE_NVT_FLASH,
                     NvtFlashState *),
    DEFINE_PROP_LINK("memory", NvtBootRomState, mem, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT64("sram-base", NvtBootRomState, sram_base, 0xF07C0000),
    DEFINE_PROP_UINT64("sram-size", NvtBootRomState, sram_size, 0x40000),
    DEFINE_PROP_UINT64("uart", NvtBootRomState, uart, 0xF0290000),
    /* the loaders' BOOT_ROM_SP_LIMIT_ADDR */
    DEFINE_PROP_UINT64("sp", NvtBootRomState, sp, 0xF07F8FFC),
    DEFINE_PROP_UINT64("svc-base", NvtBootRomState, svc_base, 0xF07BFF00),
    DEFINE_PROP_UINT64("rom-base", NvtBootRomState, rom_base, 0x80000000),
    DEFINE_PROP_UINT8("layout", NvtBootRomState, layout, 0),
};

static void nvt_bootrom_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nvt_bootrom_realize;
    device_class_set_legacy_reset(dc, nvt_bootrom_reset);
    device_class_set_props(dc, nvt_bootrom_properties);
    dc->vmsd = &vmstate_nvt_bootrom;
}

static const TypeInfo nvt_bootrom_info = {
    .name          = TYPE_NVT_BOOTROM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NvtBootRomState),
    .instance_init = nvt_bootrom_init,
    .class_init    = nvt_bootrom_class_init,
};

static void nvt_bootrom_register_types(void)
{
    type_register_static(&nvt_bootrom_info);
}

type_init(nvt_bootrom_register_types)
