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
 * The silicon's ROM also takes the loader from SD, Ethernet (DHCP and
 * TFTP), USB or UART.  "loader-file" stands for what one of them
 * delivered, a host file copied to SRAM in place of the flash's first
 * bytes.  With "fallback" (NA51089), the ROM's own fallback runs instead
 * when the flash has no valid loader or the straps are not at flash: see
 * "the ROM's fallback" below.
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
#include "qemu/error-report.h"
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
#include "hw/core/cpu.h"
#include "qemu/timer.h"

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

/*
 * The ROM's fallback, as NA51089's mask ROM does it (read from a dump of
 * it, and the NT9856x ROM code application note): with no valid loader in
 * flash it tries SD, Ethernet, USB and UART in turn, and again from SD,
 * printing a letter or two for each on UART 0.  The straps pick where it
 * starts: 0xC is UART, 4 Ethernet, 1 SD, 5 and 0xB USB, the rest flash.
 *
 *   flash     "PIp>", then "K9" for a loader that fails the check below
 *   SD        "S1K2": no card is modelled
 *   Ethernet  "HE", the PHY's registers, "s100FX", then a DHCPDISCOVER
 *             ("S1") every 5 s; six unanswered and it gives up ("K:")
 *   USB       "U"; 30 s and it gives up ("K:")
 *   UART      nothing; 30 s and it gives up ("K:"), unless OPEN_DEVICE
 *
 * The UART stage takes 16-byte commands (the CDB format of the SDK's
 * scsi_op.h): byte 0 0xC0, byte 1 the op, bytes 2..5 an address, 6..9 a
 * size or a value, 14..15 a checksum that brings the eight halfwords plus
 * 0..7 to 0.  As in the ROM, every 16 bytes received are one command and
 * nothing ever resyncs, and once a WRLOADER has started, everything that
 * follows is stored at dest + count -- and still decoded as commands, so a
 * payload chunk that looks like one acts as one:
 *
 *   IS_NVT     (word 0x000081C0) echo, then the chip word (zeros on a
 *              bad checksum)
 *   OPEN_DEVICE (0x000001C0) echo, and the 30 s timeout stops (16 zeros
 *              on a bad checksum)
 *   ADDR_READ  (0x83C0) echo, the address and the word there
 *   ADDR_WRITE (0x03C0) the word at the address, bad checksum or not; echo
 *   WRLOADER   (0x02C0) the store's address and size; the echo comes once
 *              size bytes are in
 *   WAIT_DONE  (word 0x000082C0) echo, then the loader is checked and run
 *
 * Every echo carries a fresh checksum.  The check before the jump is the
 * ROM's: the header's word 1 is the loader's address, word 9 its size,
 * halfword 0x30 is 0xAA55, and its halfwords plus their indices sum to 0.
 * Then "T" and the jump to word 0, or "K9" and the next source.
 *
 * The Ethernet stage (with "eth", the machine's nvt-eqos) is the ROM's
 * DHCP and TFTP client:
 *
 *   - A MAC of 00:00:00 and three random bytes, new at every reset.
 *   - DHCPDISCOVER broadcast ("S1"), again every 5 s.
 *   - The first offer whose xid matches, broadcast or unicast, of any
 *     message type ("a", "OfIP <yiaddr> SRV <siaddr>", "MSK <mask>" if
 *     option 1 is there, else /24 is assumed, then "b" -- "a...b" for any
 *     other frame).  It is taken only if yiaddr and siaddr share a
 *     subnet; if not, the ROM keeps waiting and asks again in 5 s.
 *   - DHCPREQUEST naming siaddr, not the offer's option 54, as the server
 *     ("S2a"; the "a" is lost on silicon's busy FIFO), then any reply
 *     with the xid as the ACK, NAK or not ("cAk"), an ARP request for the
 *     server ("S3"), and one TFTP read request for loader.bin from port
 *     2962 ("S4"), to the MAC the offer came from.
 *   - ARP requests for its address are answered ("Dar p" -- "D" alone
 *     for other ARP).
 *   - From then on it waits for DATA from that server with no timeout and
 *     no retransmission, ignoring errors: a server that never sends holds
 *     it there for good ("C" for every frame, "D" for ARP).
 *   - Each block in order is written to SRAM ("t<block>") and ACKed.
 *   - A short block ends it ("Dn"), then the loader check.
 *
 * Every frame also prints "RI".
 *
 * The CPU waits in WFI in a parked ROM while this runs on timers, and the
 * loader's arrival rewrites the ROM at 0 and wakes it.
 */
enum {
    NVT_SRC_FLASH, NVT_SRC_SD, NVT_SRC_ETH, NVT_SRC_USB, NVT_SRC_UART,
    NVT_SRC_DONE,
};

#define UART_RBR        0x00
#define UART_FCR        0x08
#define UART_LCR        0x0C
#define UART_LSR        0x14
#define UART_POLL_NS    (200 * SCALE_US)

#define DHCP_INTERVAL   5000
#define DHCP_TRIES      6
#define USB_TIMEOUT     30000
#define UART_TIMEOUT    30000

static void rom_stage(NvtBootRomState *s);
static void rom_eth_detach(NvtBootRomState *s);
static void rom_fallback_start(void *opaque);

/*
 * "fast" shortens the waits on the way to UART (DHCP, USB), not the UART
 * window: that is the host's time to find the 16-byte alignment.
 */
static void rom_arm(NvtBootRomState *s, int64_t ms)
{
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
              (s->fast && s->source != NVT_SRC_UART ? ms / 10 : ms));
}

static bool rom_loader_ok(const uint8_t *img, uint32_t len)
{
    uint32_t size, sum = 0, i;

    if (len < 0x34) {
        return false;
    }
    size = ldl_le_p(img + 0x24);
    if (size < 0x34 || size > len || size & 1 ||
        lduw_le_p(img + 0x30) != 0xAA55) {
        return false;
    }
    for (i = 0; i < size / 2; i++) {
        sum += lduw_le_p(img + 2 * i) + i;
    }
    return !(sum & 0xFFFF);
}

/* The parked ROM: wfi, b 0 */
static void rom_park(NvtBootRomState *s)
{
    uint32_t *w = (uint32_t *)s->image;

    memset(s->image, 0, ROM_SIZE);
    w[0] = cpu_to_le32(0xE320F003);
    w[1] = cpu_to_le32(0xEAFFFFFD);
    address_space_write(&s->as, 0, MEMTXATTRS_UNSPECIFIED, s->image, ROM_SIZE);
    memory_region_set_dirty(&s->rom, 0, ROM_SIZE);
}

/* "T": the loader in SRAM is run, as after a flash boot */
static void rom_run(NvtBootRomState *s, uint32_t entry)
{
    CPUState *cs = first_cpu;

    rom_puts(s, "T");
    rom_eth_detach(s);
    s->source = NVT_SRC_DONE;
    timer_del(s->timer);
    timer_del(s->poll);
    rom_build(s, entry);
    address_space_write(&s->as, 0, MEMTXATTRS_UNSPECIFIED, s->image, ROM_SIZE);
    memory_region_set_dirty(&s->rom, 0, ROM_SIZE);
    if (cs) {
        cs->halted = 0;
        qemu_cpu_kick(cs);
    }
}

/* The loader at addr, checked; run, or "K9" and on to the next source */
static void rom_try_loader(NvtBootRomState *s, uint32_t addr, uint32_t len)
{
    g_autofree uint8_t *img = g_malloc0(len);

    if (len >= 0x34 && !dma_memory_read(&s->as, addr, img, len,
                                        MEMTXATTRS_UNSPECIFIED) &&
        rom_loader_ok(img, len) && ldl_le_p(img + 4) == addr) {
        rom_run(s, ldl_le_p(img));
        return;
    }
    rom_puts(s, "K9");
    s->source = s->source == NVT_SRC_UART ? NVT_SRC_SD : s->source + 1;
    rom_stage(s);
}

static void rom_uart_send(NvtBootRomState *s, const uint8_t *buf, int n)
{
    while (n--) {
        rom_putc(s, *buf++);
    }
}

static bool cdb_sum_ok(const uint8_t *c)
{
    uint32_t sum = 0;
    int i;

    for (i = 0; i < 8; i++) {
        sum += lduw_le_p(c + 2 * i) + i;
    }
    return !(sum & 0xFFFF);
}

static void cdb_echo(NvtBootRomState *s, const uint8_t *c)
{
    uint8_t e[16];
    uint32_t sum = 7;
    int i;

    memcpy(e, c, 16);
    for (i = 0; i < 7; i++) {
        sum += lduw_le_p(e + 2 * i) + i;
    }
    stw_le_p(e + 14, -sum);
    rom_uart_send(s, e, 16);
}

static void rom_cdb(NvtBootRomState *s, const uint8_t *c)
{
    static const uint8_t zeros[16];
    uint32_t w0 = ldl_le_p(c), addr = ldl_le_p(c + 2), val = ldl_le_p(c + 6);
    uint16_t h0 = lduw_le_p(c);
    bool ok = cdb_sum_ok(c);
    uint8_t out[8];

    if (w0 == 0x000081C0) {                     /* IS_NVT */
        cdb_echo(s, c);
        stl_le_p(out, ok ? address_space_ldl_le(&s->as, s->top + 0xF0,
                                                MEMTXATTRS_UNSPECIFIED,
                                                NULL) : 0);
        rom_uart_send(s, out, 4);
    } else if (w0 == 0x000001C0) {              /* OPEN_DEVICE */
        if (ok) {
            cdb_echo(s, c);
            timer_del(s->timer);
        } else {
            rom_uart_send(s, zeros, 16);
        }
    } else if (w0 == 0x000082C0) {              /* WAIT_DONE */
        cdb_echo(s, c);
        if (s->storing) {
            uint32_t size = address_space_ldl_le(&s->as, s->dest + 0x24,
                                                 MEMTXATTRS_UNSPECIFIED, NULL);
            rom_try_loader(s, s->dest, MIN(size, s->sram_size));
        } else {
            rom_try_loader(s, s->sram_base, 0);
        }
    } else if (h0 == 0x83C0) {                  /* ADDR_READ */
        cdb_echo(s, c);
        stl_le_p(out, addr);
        stl_le_p(out + 4, address_space_ldl_le(&s->as, addr,
                                               MEMTXATTRS_UNSPECIFIED, NULL));
        rom_uart_send(s, out, 8);
    } else if (h0 == 0x03C0) {                  /* ADDR_WRITE */
        address_space_stl_le(&s->as, addr, val, MEMTXATTRS_UNSPECIFIED, NULL);
        cdb_echo(s, c);
    } else if (h0 == 0x02C0) {                  /* WRLOADER */
        s->storing = true;
        s->echo_due = true;
        s->dest = addr;
        s->size = val;
        s->count = 0;
        memcpy(s->wr_cdb, c, 16);
    }
}

static void rom_uart_byte(NvtBootRomState *s, uint8_t b)
{
    s->chunk[s->nchunk++] = b;
    if (s->nchunk < 16) {
        return;
    }
    s->nchunk = 0;
    if (s->storing) {
        dma_memory_write(&s->as, s->dest + s->count, s->chunk, 16,
                         MEMTXATTRS_UNSPECIFIED);
        s->count += 16;
    }
    rom_cdb(s, s->chunk);
    if (s->source == NVT_SRC_UART && s->echo_due && s->count >= s->size) {
        s->echo_due = false;
        cdb_echo(s, s->wr_cdb);
    }
}

/*
 * The UART stage takes what comes in.  The other stages read nothing, and
 * on silicon what arrives then overruns the FIFO and is lost; QEMU's 16550
 * would hold the backend back instead, and hand the UART stage a backlog
 * of whatever a host sent meanwhile, so it is drained here.
 */
static void rom_uart_poll(void *opaque)
{
    NvtBootRomState *s = opaque;
    int n;
    uint8_t b;

    for (n = 0; n < 64 && s->source != NVT_SRC_DONE; n++) {
        if (!(address_space_ldub(&s->as, s->uart + UART_LSR,
                                 MEMTXATTRS_UNSPECIFIED, NULL) & 1)) {
            break;
        }
        b = address_space_ldub(&s->as, s->uart + UART_RBR,
                               MEMTXATTRS_UNSPECIFIED, NULL);
        if (s->source == NVT_SRC_UART) {
            rom_uart_byte(s, b);
        }
    }
    if (s->source != NVT_SRC_DONE) {
        timer_mod(s->poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  UART_POLL_NS);
    }
}

/* The Ethernet stage: DHCP, then loader.bin over TFTP */
enum { ETH_DISCOVER, ETH_REQUEST, ETH_TFTP, ETH_DONE };

#define ETH_HLEN        14
#define IP_HLEN         20
#define UDP_HLEN        8
#define BOOTP_LEN       300
#define TFTP_PORT       2962
#define TFTP_BLOCK      512

static void eth_ip_udp(NvtBootRomState *s, uint8_t *f, const uint8_t *dmac,
                       uint32_t sip, uint32_t dip, uint16_t sport,
                       uint16_t dport, size_t len)
{
    uint8_t *ip = f + ETH_HLEN, *udp = ip + IP_HLEN;
    uint32_t sum = 0;
    int i;

    memcpy(f, dmac, 6);
    memcpy(f + 6, s->mac, 6);
    stw_be_p(f + 12, 0x0800);
    memset(ip, 0, IP_HLEN);
    ip[0] = 0x45;
    stw_be_p(ip + 2, IP_HLEN + UDP_HLEN + len);
    ip[8] = 255;
    ip[9] = 17;
    stl_be_p(ip + 12, sip);
    stl_be_p(ip + 16, dip);
    for (i = 0; i < IP_HLEN; i += 2) {
        sum += lduw_be_p(ip + i);
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    stw_be_p(ip + 10, ~sum);
    stw_be_p(udp, sport);
    stw_be_p(udp + 2, dport);
    stw_be_p(udp + 4, UDP_HLEN + len);
    stw_be_p(udp + 6, 0);               /* no UDP checksum */
}

static void eth_send_udp(NvtBootRomState *s, const uint8_t *dmac,
                         uint32_t sip, uint32_t dip, uint16_t sport,
                         uint16_t dport, const uint8_t *data, size_t len)
{
    g_autofree uint8_t *f = g_malloc0(ETH_HLEN + IP_HLEN + UDP_HLEN + len);

    eth_ip_udp(s, f, dmac, sip, dip, sport, dport, len);
    memcpy(f + ETH_HLEN + IP_HLEN + UDP_HLEN, data, len);
    nvt_eqos_rom_send(s->eth, f, ETH_HLEN + IP_HLEN + UDP_HLEN + len);
}

static const uint8_t eth_bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* DHCPDISCOVER or DHCPREQUEST, broadcast */
static void eth_dhcp(NvtBootRomState *s, int type)
{
    uint8_t b[BOOTP_LEN] = { 1, 1, 6, 0 };
    uint8_t *o = b + 240;

    stl_be_p(b + 4, s->xid);
    memcpy(b + 28, s->mac, 6);
    stl_be_p(b + 236, 0x63825363);
    *o++ = 53, *o++ = 1, *o++ = type;
    if (type == 3) {
        *o++ = 50, *o++ = 4;
        stl_be_p(o, s->my_ip);
        o += 4;
        *o++ = 54, *o++ = 4;
        stl_be_p(o, s->srv_ip);         /* siaddr, not the offer's option 54 */
        o += 4;
    }
    *o = 255;
    eth_send_udp(s, eth_bcast, 0, 0xFFFFFFFF, 68, 67, b, sizeof(b));
}

static void eth_arp_request(NvtBootRomState *s)
{
    uint8_t f[60] = { 0 };

    memcpy(f, eth_bcast, 6);
    memcpy(f + 6, s->mac, 6);
    stw_be_p(f + 12, 0x0806);
    stw_be_p(f + 14, 1);                /* Ethernet */
    stw_be_p(f + 16, 0x0800);
    f[18] = 6;
    f[19] = 4;
    stw_be_p(f + 20, 1);                /* request */
    memcpy(f + 22, s->mac, 6);
    stl_be_p(f + 28, s->my_ip);
    stl_be_p(f + 38, s->srv_ip);
    nvt_eqos_rom_send(s->eth, f, sizeof(f));
}

/* "arp": the reply to a request for our address */
static void eth_arp_reply(NvtBootRomState *s, const uint8_t *req)
{
    uint8_t f[60] = { 0 };

    rom_puts(s, "a");
    memcpy(f, req + 6, 6);
    memcpy(f + 6, s->mac, 6);
    stw_be_p(f + 12, 0x0806);
    memcpy(f + 14, req + 14, 6);        /* htype, ptype, sizes */
    stw_be_p(f + 20, 2);                /* reply */
    memcpy(f + 22, s->mac, 6);
    stl_be_p(f + 28, s->my_ip);
    memcpy(f + 32, req + 22, 10);       /* the asker's MAC and IP */
    rom_puts(s, "r");
    nvt_eqos_rom_send(s->eth, f, sizeof(f));
    rom_puts(s, "p");
}

static void eth_tftp_rrq(NvtBootRomState *s)
{
    static const char rrq[] = "\0\1loader.bin\0octet";

    eth_send_udp(s, s->srv_mac, s->my_ip, s->srv_ip, TFTP_PORT, 69,
                 (const uint8_t *)rrq, sizeof(rrq));
}

static void eth_tftp_ack(NvtBootRomState *s, uint16_t block)
{
    uint8_t ack[4];

    stw_be_p(ack, 4);
    stw_be_p(ack + 2, block);
    eth_send_udp(s, s->srv_mac, s->my_ip, s->srv_ip, TFTP_PORT, s->srv_port,
                 ack, sizeof(ack));
}

static uint32_t bootp_option(const uint8_t *b, size_t len, uint8_t want,
                             uint32_t dflt)
{
    size_t i = 240;

    while (i + 1 < len && b[i] != 255) {
        if (!b[i]) {
            i++;
            continue;
        }
        if (b[i] == want && b[i + 1] >= 1 && i + 2 + b[i + 1] <= len) {
            return b[i + 1] >= 4 ? ldl_be_p(b + i + 2) : b[i + 2];
        }
        i += 2 + b[i + 1];
    }
    return dflt;
}

static void rom_eth_rx(void *opaque, const uint8_t *f, size_t len)
{
    NvtBootRomState *s = opaque;
    const uint8_t *ip = f + ETH_HLEN, *udp, *data;
    uint32_t sip;
    size_t dlen;
    char buf[64];

    if (s->source != NVT_SRC_ETH || len < ETH_HLEN) {
        return;
    }
    rom_puts(s, "RI\r\n");
    if (s->eth_state == ETH_TFTP) {
        rom_puts(s, "C");
    }
    if (lduw_be_p(f + 12) == 0x0806) {
        if (s->eth_state == ETH_TFTP) {
            rom_puts(s, "D");
            if (len >= 42 && lduw_be_p(f + 20) == 1 &&
                ldl_be_p(f + 38) == s->my_ip) {
                eth_arp_reply(s, f);
            }
        }
        return;
    }
    if (lduw_be_p(f + 12) != 0x0800 || len < ETH_HLEN + IP_HLEN + UDP_HLEN ||
        ip[9] != 17) {
        return;
    }
    sip = ldl_be_p(ip + 12);
    udp = ip + (ip[0] & 0xF) * 4;
    data = udp + UDP_HLEN;
    if (data > f + len) {
        return;
    }
    dlen = MIN(len - (data - f), lduw_be_p(udp + 4) - UDP_HLEN);

    switch (s->eth_state) {
    case ETH_DISCOVER:
        if (lduw_be_p(udp + 2) != 68) {
            return;
        }
        /*
         * The xid is all the ROM checks (net_run, 0x3438): broadcast or
         * unicast, offer or not.  The mask is option 1's, else /24.
         */
        rom_puts(s, "a");
        if (dlen < 240 || ldl_be_p(data + 4) != s->xid) {
            rom_puts(s, "b");
            return;
        }
        s->my_ip = ldl_be_p(data + 16);
        s->srv_ip = ldl_be_p(data + 20);
        snprintf(buf, sizeof(buf), "OfIP %08X SRV %08X\r\n",
                 s->my_ip, s->srv_ip);
        rom_puts(s, buf);
        s->mask = bootp_option(data, dlen, 1, 0);
        if (s->mask) {
            snprintf(buf, sizeof(buf), "MSK %08X\r\n", s->mask);
            rom_puts(s, buf);
        } else {
            s->mask = 0xFFFFFF00;
        }
        memcpy(s->srv_mac, f + 6, 6);
        rom_puts(s, "b");
        if ((s->my_ip ^ s->srv_ip) & s->mask) {
            return;                     /* a server elsewhere: wait, ask again */
        }
        s->eth_state = ETH_REQUEST;
        eth_dhcp(s, 3);
        rom_puts(s, "S2a");
        break;
    case ETH_REQUEST:
        /* any reply with the xid is the ACK: a NAK would do as well */
        if (lduw_be_p(udp + 2) != 68) {
            return;
        }
        rom_puts(s, "c");
        if (dlen < 240 || ldl_be_p(data + 4) != s->xid) {
            return;
        }
        rom_puts(s, "Ak");
        timer_del(s->timer);            /* no timeout from here on */
        s->eth_state = ETH_TFTP;
        s->block = 0;
        s->got = 0;
        s->srv_port = 0;
        eth_arp_request(s);
        rom_puts(s, "S3");
        eth_tftp_rrq(s);
        rom_puts(s, "S4");
        break;
    case ETH_TFTP:
        /* DATA from the server to our port; anything else is ignored */
        if (sip != s->srv_ip || lduw_be_p(udp + 2) != TFTP_PORT ||
            dlen < 4 || lduw_be_p(data) != 3) {
            return;
        }
        if (!s->srv_port) {
            s->srv_port = lduw_be_p(udp);
        }
        if (lduw_be_p(udp) != s->srv_port) {
            return;
        }
        if (lduw_be_p(data + 2) == s->block) {
            eth_tftp_ack(s, s->block);  /* our ACK was lost: again */
            return;
        }
        if (lduw_be_p(data + 2) != (uint16_t)(s->block + 1)) {
            return;
        }
        s->block++;
        dlen -= 4;
        if (s->got + dlen > s->sram_size) {
            dlen = s->sram_size - s->got;
        }
        dma_memory_write(&s->as, s->sram_base + s->got, data + 4, dlen,
                         MEMTXATTRS_UNSPECIFIED);
        s->got += dlen;
        snprintf(buf, sizeof(buf), "t%08X", s->block);
        rom_puts(s, buf);
        eth_tftp_ack(s, s->block);
        if (dlen < TFTP_BLOCK || s->got >= s->sram_size) {
            rom_puts(s, "Dn");
            s->eth_state = ETH_DONE;
            rom_eth_detach(s);
            rom_try_loader(s, s->sram_base, s->got);
        }
        break;
    }
}

static void rom_eth_detach(NvtBootRomState *s)
{
    if (s->eth) {
        nvt_eqos_rom_attach(s->eth, NULL, NULL);
    }
}

/* "S1": a DHCPDISCOVER, if there is a MAC to send it on */
static void rom_eth_discover(NvtBootRomState *s)
{
    rom_puts(s, "S1");
    if (s->eth) {
        s->eth_state = ETH_DISCOVER;
        s->xid = g_random_int();
        eth_dhcp(s, 1);
    }
}

/* A stage's timeout ran out, or the next DHCPDISCOVER is due */
static void rom_timeout(void *opaque)
{
    NvtBootRomState *s = opaque;

    switch (s->source) {
    case -1:                            /* just out of reset */
        rom_fallback_start(s);
        return;
    case NVT_SRC_ETH:
        if (++s->tries < DHCP_TRIES) {
            rom_eth_discover(s);
            rom_arm(s, DHCP_INTERVAL);
            return;
        }
        rom_eth_detach(s);
        break;
    case NVT_SRC_USB:
        break;
    case NVT_SRC_UART:
        break;
    default:
        return;
    }
    rom_puts(s, "K:");
    s->source = s->source == NVT_SRC_UART ? NVT_SRC_SD : s->source + 1;
    rom_stage(s);
}

static void rom_stage(NvtBootRomState *s)
{
    uint32_t size;

    switch (s->source) {
    case NVT_SRC_FLASH:
        rom_puts(s, "PIp>");
        nvt_flash_read(s->flash, 0x24, &size, 4);
        size = MIN(le32_to_cpu(size), s->sram_size);
        s->loader = g_realloc(s->loader, MAX(size, 4));
        nvt_flash_read(s->flash, 0, s->loader, size);
        address_space_write(&s->as, s->sram_base, MEMTXATTRS_UNSPECIFIED,
                            s->loader, size);
        rom_try_loader(s, s->sram_base, size);
        break;
    case NVT_SRC_SD:
        rom_puts(s, "S1K2");
        s->source = NVT_SRC_ETH;
        rom_stage(s);
        break;
    case NVT_SRC_ETH:
        rom_puts(s, "HE00003500000037000000D1E1s100FX");
        s->tries = 0;
        if (s->eth) {
            uint32_t r = g_random_int();

            s->mac[0] = s->mac[1] = s->mac[2] = 0;
            s->mac[3] = r >> 16;
            s->mac[4] = r >> 8;
            s->mac[5] = r;
            nvt_eqos_rom_attach(s->eth, rom_eth_rx, s);
        }
        rom_eth_discover(s);
        rom_arm(s, DHCP_INTERVAL);
        break;
    case NVT_SRC_USB:
        rom_puts(s, "U");
        rom_arm(s, USB_TIMEOUT);
        break;
    case NVT_SRC_UART:
        /*
         * 8N1, FIFOs on and reset.  The receive trigger at 14 bytes: QEMU's
         * 16550 offers its backend only up to the trigger level, so at 1 a
         * 32 KiB loader would take half a minute.
         */
        address_space_stb(&s->as, s->uart + UART_LCR, 0x03,
                          MEMTXATTRS_UNSPECIFIED, NULL);
        address_space_stb(&s->as, s->uart + UART_FCR, 0xC7,
                          MEMTXATTRS_UNSPECIFIED, NULL);
        s->nchunk = 0;
        s->storing = s->echo_due = false;
        rom_arm(s, UART_TIMEOUT);
        break;
    }
}

/* Where the straps send the ROM first */
static int rom_first_source(uint8_t strap)
{
    switch (strap & 0xF) {
    case 0xC:
        return NVT_SRC_UART;
    case 4:
        return NVT_SRC_ETH;
    case 1:
        return NVT_SRC_SD;
    case 5: case 0xB:
        return NVT_SRC_USB;
    default:
        return NVT_SRC_FLASH;
    }
}

static void rom_fallback_start(void *opaque)
{
    NvtBootRomState *s = opaque;

    s->source = rom_first_source(s->strap);
    if (s->source != NVT_SRC_UART) {    /* UART boot prints nothing */
        rom_puts(s, "N");
    }
    timer_mod(s->poll, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + UART_POLL_NS);
    rom_stage(s);
}

static bool rom_flash_loader_ok(NvtBootRomState *s)
{
    uint32_t size;
    g_autofree uint8_t *img = NULL;

    nvt_flash_read(s->flash, 0x24, &size, 4);
    size = le32_to_cpu(size);
    if (size < 0x34 || size > s->sram_size) {
        return false;
    }
    img = g_malloc(size);
    nvt_flash_read(s->flash, 0, img, size);
    return rom_loader_ok(img, size) && ldl_le_p(img + 4) == s->sram_base;
}

static void nvt_bootrom_reset(DeviceState *dev)
{
    NvtBootRomState *s = NVT_BOOTROM(dev);
    uint32_t hdr[10], size;

    timer_del(s->timer);
    timer_del(s->poll);
    rom_eth_detach(s);
    s->source = NVT_SRC_DONE;
    if (s->fallback && !s->loader_file &&
        (rom_first_source(s->strap) != NVT_SRC_FLASH ||
         !rom_flash_loader_ok(s))) {
        rom_park(s);
        /* from the main loop, once every device is out of reset */
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
        s->source = -1;
        return;
    }

    /* the loader into SRAM, as the ROM copies it */
    if (s->loader_file) {
        g_autoptr(GError) err = NULL;
        gsize len;

        g_free(s->loader);
        if (!g_file_get_contents(s->loader_file, (gchar **)&s->loader, &len,
                                 &err) || len < sizeof(hdr) ||
            len > s->sram_size) {
            error_report("nvt-bootrom: cannot use %s as a loader: %s",
                         s->loader_file, err ? err->message : "its size");
            exit(1);
        }
        memcpy(hdr, s->loader, sizeof(hdr));
        size = len;
    } else {
        nvt_flash_read(s->flash, 0, hdr, sizeof(hdr));
        size = le32_to_cpu(hdr[9]);
        if (!size || size > s->sram_size) {
            size = 32 * KiB;
        }
        s->loader = g_realloc(s->loader, size);
        nvt_flash_read(s->flash, 0, s->loader, size);
    }
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
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, rom_timeout, s);
    s->poll = timer_new_ns(QEMU_CLOCK_VIRTUAL, rom_uart_poll, s);
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
    DEFINE_PROP_STRING("loader-file", NvtBootRomState, loader_file),
    DEFINE_PROP_BOOL("fallback", NvtBootRomState, fallback, false),
    DEFINE_PROP_BOOL("fast", NvtBootRomState, fast, false),
    DEFINE_PROP_UINT8("strap", NvtBootRomState, strap, 0),
    DEFINE_PROP_UINT64("top", NvtBootRomState, top, 0xF0010000),
    DEFINE_PROP_LINK("eth", NvtBootRomState, eth, TYPE_DEVICE, DeviceState *),
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
