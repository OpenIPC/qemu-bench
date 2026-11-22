/*
 * HiSilicon Boot ROM fastboot serial protocol emulation.
 *
 * Emulates the boot ROM UART download protocol used by real HiSilicon
 * SoCs.  When no -kernel is provided, this device takes over UART0 and
 * sends 0x20 handshake bytes.  A host tool (e.g. defib) can then upload
 * DDR init code, SPL, and U-Boot via the standard HEAD/DATA/TAIL frame
 * protocol.  After the final transfer, the chardev is handed off to a
 * newly-created PL011 and the CPU is started at the U-Boot entry point.
 *
 * Protocol reference: Protocol 1 "Standard (Classic HiSilicon)" from
 * the defib project's qemu_hisilicon_spec.md.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "hw/char/pl011.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "hw/core/cpu.h"
#include "hw/misc/hisi-fastboot.h"

#define TYPE_HISI_FASTBOOT "hisi-fastboot"
OBJECT_DECLARE_SIMPLE_TYPE(HisiFastbootState, HISI_FASTBOOT)

/* Protocol constants */
#define FB_ACK          0xAA
#define FB_HANDSHAKE    0x20
#define FB_HEAD_MAGIC0  0xFE
#define FB_HEAD_MAGIC1  0x00
#define FB_HEAD_MAGIC2  0xFF
#define FB_HEAD_MAGIC3  0x01
#define FB_DATA_MAGIC   0xDA
#define FB_TAIL_MAGIC   0xED
#define FB_HEAD_SIZE    14
#define FB_TAIL_SIZE    5
#define FB_MAX_PAYLOAD  1024
#define FB_RXBUF_SIZE   2048

/* Handshake timer interval: 1ms (real HW is ~87us at 115200 baud) */
#define FB_HANDSHAKE_INTERVAL_NS  (1 * SCALE_MS)

/*
 * V500 protocol (Goke GK7206): host-initiated handshake.  The host streams
 * the 14-byte frame  BD 00 FF 01  00*8  CRC16  and the ROM answers with
 * BD 00 00 08 00 00 00 02 <chip_id, big-endian> CRC16.  Verified on an XM
 * IVG-K8 (GK7206V11A): reply bd 00 00 08 00 00 00 02 72 06 01 10 b7 99.
 * The HEAD/DATA/TAIL upload that follows is identical to the classic one.
 *
 * After the first (key/HEAD) area the ROM verifies it.  This is a PLAINTEXT
 * header check, not RSA secure boot (confirmed by decoding two V500 stock
 * images byte-for-byte — gk7206 + gk7205v510 — no signature block anywhere in
 * the HEAD).  A header carrying the magic 0x12345678 at key-area offset 0x410
 * passes and the ROM goes on to the aux + boot areas, ending with "Bootrom
 * success" (seen on the IVG-K8 over UART, 2026-11-20).  A header without it —
 * e.g. a raw payload uploaded by an unaware tool — is rejected with
 * "Verify key fail".
 */
#define FB_V500_MAGIC0  0xBD
#define FB_V500_MAGIC1  0x00
#define FB_V500_MAGIC2  0xFF
#define FB_V500_MAGIC3  0x01
#define FB_V500_FRAME_LEN  14
/* V500 boot images load at this base; the header entry field is base + the
 * boot-code file offset (0x40706000 on gk7206, 0x40707000 on gk7205v5x0). */
#define FB_V500_IMAGE_BASE  0x40700000u

/* State machine phases */
enum {
    FB_PHASE_IDLE,
    FB_PHASE_HANDSHAKE,     /* classic: ROM sends 0x20, waits for host ACK   */
    FB_PHASE_V500_HS,       /* V500: wait for the host's BD-00-FF-01 frame   */
    FB_PHASE_RECV_HEAD,
    FB_PHASE_RECV_DATA,
    FB_PHASE_RECV_TAIL,
    FB_PHASE_BOOT,
    FB_PHASE_V500_FAILED,   /* V500: key verify rejected the image, halted   */
};

/* Transfer steps */
enum {
    FB_STEP_DDR,
    FB_STEP_SPL,
    FB_STEP_UBOOT,
    FB_STEP_DONE,
};

/* CRC-16/CCITT lookup table (polynomial 0x1021) */
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

struct HisiFastbootState {
    DeviceState parent_obj;

    /* Chardev frontend */
    CharFrontend chr;
    Chardev *chardev_backend;

    /* Deferred UART0 creation */
    CPUState *cpu;
    hwaddr uart0_base;
    qemu_irq uart0_irq;

    /* Handshake timer */
    QEMUTimer *handshake_timer;

    /* Protocol state */
    int phase;
    int step;

    /* V500 protocol (GK7206) */
    bool v500;              /* use the V500 handshake + header-verify model */
    uint32_t chip_id;       /* returned in the handshake reply            */
    bool v500_resync;       /* drop bytes until the first HEAD magic 0xFE */
    uint8_t v500_head[0x418]; /* first bytes of the key area, for the verify */
    uint32_t v500_entry;    /* absolute boot entry (the header load-addr field) */

    /* Frame accumulation */
    uint8_t rxbuf[FB_RXBUF_SIZE];
    int rxpos;
    int rxexpect;

    /* Current transfer */
    hwaddr xfer_addr;
    uint32_t xfer_length;
    uint32_t xfer_received;
    uint8_t next_seq;

    /* Boot entry */
    hwaddr entry_addr;
};

/* ── Public setup function ───────────────────────────────────────────── */

void hisi_fastboot_setup(DeviceState *dev, CPUState *cpu,
                         Chardev *chardev, hwaddr uart0_base,
                         qemu_irq uart0_irq, bool v500, uint32_t chip_id)
{
    HisiFastbootState *s = HISI_FASTBOOT(dev);
    s->cpu = cpu;
    s->chardev_backend = chardev;
    s->uart0_base = uart0_base;
    s->uart0_irq = uart0_irq;
    s->v500 = v500;
    s->chip_id = chip_id;
}

/* ── CRC ─────────────────────────────────────────────────────────────── */

static uint16_t hisi_fastboot_crc(const uint8_t *data, int len)
{
    uint32_t crc = 0;
    int i;

    for (i = 0; i < len; i++) {
        crc = ((crc << 8) | data[i]) ^ crc_table[(crc >> 8) & 0xFF];
    }
    /* Finalize with two zero-byte padding */
    for (i = 0; i < 2; i++) {
        crc = ((crc << 8) | 0) ^ crc_table[(crc >> 8) & 0xFF];
    }
    return crc & 0xFFFF;
}

static bool hisi_fastboot_verify_crc(const uint8_t *frame, int len)
{
    uint16_t computed, expected;

    if (len < 3) {
        return false;
    }
    computed = hisi_fastboot_crc(frame, len - 2);
    expected = (frame[len - 2] << 8) | frame[len - 1];
    return computed == expected;
}

/* ── TX helpers ──────────────────────────────────────────────────────── */

static void hisi_fastboot_send_byte(HisiFastbootState *s, uint8_t val)
{
    qemu_chr_fe_write_all(&s->chr, &val, 1);
}

static void hisi_fastboot_send_ack(HisiFastbootState *s)
{
    hisi_fastboot_send_byte(s, FB_ACK);
}

static void hisi_fastboot_send_str(HisiFastbootState *s, const char *str)
{
    qemu_chr_fe_write_all(&s->chr, (const uint8_t *)str, strlen(str));
}

/* V500: answer the host's BD-00-FF-01 handshake with the chip id. */
static void hisi_fastboot_v500_reply(HisiFastbootState *s)
{
    uint8_t f[FB_V500_FRAME_LEN];
    uint16_t crc;

    /* BD 00 00 08 00 00 00 02 <chip_id, big-endian> CRC16 — the exact shape
     * the GK7206 ROM returns (defib reads the chip id at bytes 8..11). */
    f[0] = 0xBD; f[1] = 0x00; f[2] = 0x00; f[3] = 0x08;
    f[4] = 0x00; f[5] = 0x00; f[6] = 0x00; f[7] = 0x02;
    f[8]  = (s->chip_id >> 24) & 0xFF;
    f[9]  = (s->chip_id >> 16) & 0xFF;
    f[10] = (s->chip_id >> 8)  & 0xFF;
    f[11] =  s->chip_id        & 0xFF;
    crc = hisi_fastboot_crc(f, 12);
    f[12] = (crc >> 8) & 0xFF;
    f[13] =  crc       & 0xFF;

    qemu_chr_fe_write_all(&s->chr, f, FB_V500_FRAME_LEN);
    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: V500 handshake, chip id 0x%08x\n",
                  s->chip_id);
}

/*
 * V500 (GK7206): the ROM's verify of the uploaded key/HEAD area is a PLAINTEXT
 * header check — it looks for the magic 0x12345678 at key-area offset 0x410
 * (where every stock V500 image carries it), not an RSA signature.  A header
 * without the magic is rejected; reproduce the exact bytes the real bootrom
 * emits, then leave the CPU halted.
 */
static bool hisi_fastboot_v500_header_ok(HisiFastbootState *s)
{
    const uint8_t *h = s->v500_head;
    /* The key area must be long enough to carry both the magic and the entry
     * field (through 0x417); a shorter first transfer would leave those bytes
     * zero-initialised and start the CPU at a bogus entry. */
    if (s->xfer_received < sizeof(s->v500_head)) {
        return false;
    }
    return h[0x410] == 0x78 && h[0x411] == 0x56 &&
           h[0x412] == 0x34 && h[0x413] == 0x12;  /* 0x12345678 LE */
}

static void hisi_fastboot_v500_verify_fail(HisiFastbootState *s)
{
    static const uint8_t status[] = {
        0xee, 0x00, 0x04, 0x80, 0x01, 0x00, 0x00,
    };

    hisi_fastboot_send_str(s, "Verify key fail\r\n");
    qemu_chr_fe_write_all(&s->chr, status, sizeof(status));
    hisi_fastboot_send_str(s, "Ie00000085 80010000\r\n");

    s->phase = FB_PHASE_V500_FAILED;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "hisi-fastboot: V500 header verify failed (no magic "
                  "0x12345678 in the key area); image rejected, CPU halted\n");
}

/*
 * V500 (GK7206): all three areas (key/HEAD, aux/DDR-init, boot) accepted and
 * the header verified.  The real ROM prints "Bootrom success" then runs the
 * boot code at load-addr base + boot_off.  (On hardware the post-success jump
 * into a bare-metal agent has not yet been observed running — tracked
 * separately — but the protocol-visible success is reproduced here.)
 */
static void hisi_fastboot_v500_success(HisiFastbootState *s);

/* ── Frame processors ────────────────────────────────────────────────── */

static void hisi_fastboot_boot(HisiFastbootState *s);

static const char *step_name(int step)
{
    switch (step) {
    case FB_STEP_DDR:   return "DDR";
    case FB_STEP_SPL:   return "SPL";
    case FB_STEP_UBOOT: return "U-Boot";
    default:            return "???";
    }
}

static void hisi_fastboot_process_head(HisiFastbootState *s)
{
    uint8_t *f = s->rxbuf;

    /* Validate magic */
    if (f[0] != FB_HEAD_MAGIC0 || f[1] != FB_HEAD_MAGIC1 ||
        f[2] != FB_HEAD_MAGIC2 || f[3] != FB_HEAD_MAGIC3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: HEAD bad magic %02x %02x %02x %02x\n",
                      f[0], f[1], f[2], f[3]);
        s->rxpos = 0;
        return;
    }

    /* Validate CRC */
    if (!hisi_fastboot_verify_crc(f, FB_HEAD_SIZE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-fastboot: HEAD CRC mismatch\n");
        s->rxpos = 0;
        return;
    }

    /* Extract length and address (big-endian) */
    s->xfer_length = ((uint32_t)f[4] << 24) | ((uint32_t)f[5] << 16) |
                     ((uint32_t)f[6] << 8)  | f[7];
    s->xfer_addr   = ((uint32_t)f[8] << 24) | ((uint32_t)f[9] << 16) |
                     ((uint32_t)f[10] << 8) | f[11];
    s->xfer_received = 0;
    s->next_seq = 1;

    /* Track the boot entry.  Classic: the U-Boot HEAD's own address.  V500
     * differs by family (both confirmed against the respective hardware):
     *   - gk7206: the ROM jumps to the header's ABSOLUTE entry field, wherever
     *     BOOT was loaded — a wrong load base lands the jump off the image.
     *   - gk7205v5x0: the ROM jumps to the BOOT download address + boot_off
     *     (a probe read pc=0x41007000, not the 0x40707000 the field holds), so
     *     the absolute entry's low bits are the only part that matters. */
    if (s->step == FB_STEP_UBOOT) {
        if (!s->v500) {
            s->entry_addr = s->xfer_addr;
        } else if ((s->chip_id & 0xFFFF0000u) == 0x72060000u) {   /* gk7206 */
            s->entry_addr = s->v500_entry;
        } else {                                                  /* gk7205v5x0 */
            uint32_t boot_off = s->v500_entry - FB_V500_IMAGE_BASE;
            s->entry_addr = s->xfer_addr + boot_off;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: %s HEAD len=0x%x addr=0x%" HWADDR_PRIx "\n",
                  step_name(s->step), s->xfer_length, s->xfer_addr);

    hisi_fastboot_send_ack(s);

    /* Set up for DATA frames */
    s->rxpos = 0;
    if (s->xfer_length > 0) {
        uint32_t remaining = s->xfer_length - s->xfer_received;
        uint32_t chunk = remaining < FB_MAX_PAYLOAD ? remaining : FB_MAX_PAYLOAD;
        s->rxexpect = 3 + chunk + 2;  /* DA + seq + ~seq + payload + CRC */
        s->phase = FB_PHASE_RECV_DATA;
    } else {
        s->phase = FB_PHASE_RECV_TAIL;
    }
}

static void hisi_fastboot_process_data(HisiFastbootState *s)
{
    uint8_t *f = s->rxbuf;
    int flen = s->rxexpect;
    uint8_t seq, nseq;
    int payload_len;
    hwaddr dest;
    uint32_t remaining, chunk;

    /* Validate magic */
    if (f[0] != FB_DATA_MAGIC) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: DATA bad magic 0x%02x\n", f[0]);
        s->rxpos = 0;
        return;
    }

    /* Validate sequence */
    seq = f[1];
    nseq = f[2];
    if (nseq != (uint8_t)(~seq)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: DATA seq mismatch %02x ~%02x\n",
                      seq, nseq);
        s->rxpos = 0;
        return;
    }
    if (seq != s->next_seq) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: DATA unexpected seq %d (expected %d)\n",
                      seq, s->next_seq);
        s->rxpos = 0;
        return;
    }

    /* Validate CRC */
    if (!hisi_fastboot_verify_crc(f, flen)) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-fastboot: DATA CRC mismatch\n");
        s->rxpos = 0;
        return;
    }

    /* Write payload to guest memory */
    payload_len = flen - 5;  /* minus DA + seq + ~seq + 2 CRC */
    dest = s->xfer_addr + s->xfer_received;

    address_space_write(&address_space_memory, dest,
                        MEMTXATTRS_UNSPECIFIED, &f[3], payload_len);

    /* V500: capture the head of the key area as it streams, so the verify
     * after its TAIL can check the plaintext header (the real ROM holds this
     * in SRAM, not guest DRAM, so read it from the frames, not memory). */
    if (s->v500 && s->step == FB_STEP_DDR) {
        uint32_t base = s->xfer_received;   /* area offset of this chunk's start */
        for (int k = 0; k < payload_len; k++) {
            uint32_t off = base + k;
            if (off < sizeof(s->v500_head)) {
                s->v500_head[off] = f[3 + k];
            }
        }
    }

    s->xfer_received += payload_len;
    s->next_seq = (uint8_t)(seq + 1);

    hisi_fastboot_send_ack(s);

    s->rxpos = 0;

    if (s->xfer_received >= s->xfer_length) {
        /* All data received, expect TAIL */
        s->phase = FB_PHASE_RECV_TAIL;
    } else {
        /* More DATA frames to come */
        remaining = s->xfer_length - s->xfer_received;
        chunk = remaining < FB_MAX_PAYLOAD ? remaining : FB_MAX_PAYLOAD;
        s->rxexpect = 3 + chunk + 2;
    }
}

static void hisi_fastboot_process_tail(HisiFastbootState *s)
{
    uint8_t *f = s->rxbuf;
    uint8_t seq, nseq;

    /* Validate magic */
    if (f[0] != FB_TAIL_MAGIC) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: TAIL bad magic 0x%02x\n", f[0]);
        s->rxpos = 0;
        return;
    }

    /* Validate sequence complement */
    seq = f[1];
    nseq = f[2];
    if (nseq != (uint8_t)(~seq)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fastboot: TAIL seq mismatch %02x ~%02x\n",
                      seq, nseq);
        s->rxpos = 0;
        return;
    }

    /* Validate CRC */
    if (!hisi_fastboot_verify_crc(f, FB_TAIL_SIZE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "hisi-fastboot: TAIL CRC mismatch\n");
        s->rxpos = 0;
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: %s complete (%u bytes at 0x%" HWADDR_PRIx ")\n",
                  step_name(s->step), s->xfer_received, s->xfer_addr);

    hisi_fastboot_send_ack(s);
    s->rxpos = 0;

    if (s->v500) {
        /*
         * V500 (GK7206): three areas arrive — key/HEAD, aux (DDR init), boot.
         * The ROM verifies the plaintext header when the first area's TAIL
         * lands.  A valid header (magic 0x12345678) continues to the aux and
         * boot areas and ends with "Bootrom success"; an invalid one (a raw
         * payload) is rejected with "Verify key fail".
         */
        if (s->step == FB_STEP_DDR) {
            if (!hisi_fastboot_v500_header_ok(s)) {
                hisi_fastboot_v500_verify_fail(s);
                return;
            }
            /* Header OK: remember the ABSOLUTE boot entry from the load-addr
             * field (@0x414).  The gk7206 ROM jumps there regardless of where
             * BOOT was loaded (confirmed on hardware): loading BOOT at
             * 0x41000000 jumps to 0x40706000 and the agent never runs; loading
             * it at the image's natural base 0x40700000 lands the entry on the
             * agent. */
            s->v500_entry = s->v500_head[0x414]        |
                            s->v500_head[0x415] << 8   |
                            s->v500_head[0x416] << 16  |
                            (uint32_t)s->v500_head[0x417] << 24;
        }
        s->step++;
        if (s->step >= FB_STEP_DONE) {
            hisi_fastboot_v500_success(s);
        } else {
            s->phase = FB_PHASE_RECV_HEAD;
        }
        return;
    }

    /* Advance to next step */
    s->step++;

    if (s->step >= FB_STEP_DONE) {
        /* All three transfers done — boot */
        hisi_fastboot_boot(s);
    } else {
        /* Ready for next HEAD frame */
        s->phase = FB_PHASE_RECV_HEAD;
    }
}

/* ── Boot: hand off chardev to PL011 and start CPU ───────────────────── */

static void hisi_fastboot_boot(HisiFastbootState *s)
{
    CPUState *cs = s->cpu;

    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: booting at 0x%" HWADDR_PRIx "\n",
                  s->entry_addr);

    s->phase = FB_PHASE_BOOT;

    /* Stop handshake timer if still running */
    if (s->handshake_timer) {
        timer_del(s->handshake_timer);
    }

    /*
     * Hand off chardev to a fresh PL011.
     *
     * We must deinit fastboot's frontend FIRST, then let pl011_create
     * claim the same chardev backend.  The chardev backend (e.g. a
     * socket) keeps its connection state — so if a client is still
     * connected, the PL011 can write to it immediately.
     */
    qemu_chr_fe_deinit(&s->chr, false);
    pl011_create(s->uart0_base, s->uart0_irq, s->chardev_backend);

    /* Kick the chardev to re-send OPENED event to the new PL011 frontend,
     * so it knows a client is connected and can accept writes. */
    if (s->chardev_backend->be_open) {
        qemu_chr_be_event(s->chardev_backend, CHR_EVENT_OPENED);
    }

    /* Start CPU at the loaded entry point */
    cpu_set_pc(cs, s->entry_addr);
    cs->halted = 0;
    cpu_resume(cs);
}

/* V500: header verified and all areas accepted — the real ROM prints this,
 * then runs the boot code at entry_addr (xfer_addr + boot_off). */
static void hisi_fastboot_v500_success(HisiFastbootState *s)
{
    hisi_fastboot_send_str(s, "Bootrom success\r\n");
    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: V500 header verified, image accepted; "
                  "booting at 0x%" HWADDR_PRIx "\n", s->entry_addr);
    hisi_fastboot_boot(s);
}

/* ── Chardev receive handler ─────────────────────────────────────────── */

static int hisi_fastboot_can_receive(void *opaque)
{
    HisiFastbootState *s = HISI_FASTBOOT(opaque);

    if (s->phase == FB_PHASE_BOOT || s->phase == FB_PHASE_IDLE) {
        return 0;
    }
    return FB_RXBUF_SIZE - s->rxpos;
}

static void hisi_fastboot_receive(void *opaque, const uint8_t *buf, int size)
{
    HisiFastbootState *s = HISI_FASTBOOT(opaque);
    int i;

    for (i = 0; i < size; i++) {
        uint8_t byte = buf[i];

        if (s->phase == FB_PHASE_HANDSHAKE) {
            if (byte == FB_ACK) {
                /* Host acknowledged — enter download mode */
                timer_del(s->handshake_timer);
                s->phase = FB_PHASE_RECV_HEAD;
                s->step = FB_STEP_DDR;
                s->rxpos = 0;
                qemu_log_mask(LOG_UNIMP,
                              "hisi-fastboot: handshake complete, "
                              "entering download mode\n");
            }
            continue;
        }

        if (s->phase == FB_PHASE_V500_HS) {
            /* Accumulate and scan the trailing window for the 14-byte
             * BD-00-FF-01 handshake frame; answer the first valid one. */
            if (s->rxpos >= FB_RXBUF_SIZE) {
                memmove(s->rxbuf,
                        &s->rxbuf[s->rxpos - (FB_V500_FRAME_LEN - 1)],
                        FB_V500_FRAME_LEN - 1);
                s->rxpos = FB_V500_FRAME_LEN - 1;
            }
            s->rxbuf[s->rxpos++] = byte;
            if (s->rxpos >= FB_V500_FRAME_LEN) {
                uint8_t *f = &s->rxbuf[s->rxpos - FB_V500_FRAME_LEN];
                if (f[0] == FB_V500_MAGIC0 && f[1] == FB_V500_MAGIC1 &&
                    f[2] == FB_V500_MAGIC2 && f[3] == FB_V500_MAGIC3 &&
                    hisi_fastboot_verify_crc(f, FB_V500_FRAME_LEN)) {
                    hisi_fastboot_v500_reply(s);
                    s->phase = FB_PHASE_RECV_HEAD;
                    s->step = FB_STEP_DDR;
                    s->v500_resync = true;  /* drop stray HS bytes before HEAD */
                    s->rxpos = 0;
                }
            }
            continue;
        }

        if (s->phase == FB_PHASE_V500_FAILED) {
            continue;   /* image rejected, ROM wedged — ignore further input */
        }

        if (s->rxpos >= FB_RXBUF_SIZE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "hisi-fastboot: RX buffer overflow, resetting\n");
            s->rxpos = 0;
            continue;
        }

        /* V500: after the handshake reply the host may still have a few
         * queued handshake bytes in flight; skip them until the HEAD magic. */
        if (s->v500_resync && s->phase == FB_PHASE_RECV_HEAD) {
            if (s->rxpos == 0 && byte != FB_HEAD_MAGIC0) {
                continue;
            }
            s->v500_resync = false;
        }

        s->rxbuf[s->rxpos++] = byte;

        switch (s->phase) {
        case FB_PHASE_RECV_HEAD:
            if (s->rxpos >= FB_HEAD_SIZE) {
                hisi_fastboot_process_head(s);
            }
            break;

        case FB_PHASE_RECV_DATA:
            if (s->rxpos >= s->rxexpect) {
                hisi_fastboot_process_data(s);
            }
            break;

        case FB_PHASE_RECV_TAIL:
            if (s->rxpos >= FB_TAIL_SIZE) {
                hisi_fastboot_process_tail(s);
            }
            break;

        default:
            break;
        }
    }
}

/* ── Handshake timer ─────────────────────────────────────────────────── */

static void hisi_fastboot_handshake_tick(void *opaque)
{
    HisiFastbootState *s = HISI_FASTBOOT(opaque);

    if (s->phase != FB_PHASE_HANDSHAKE) {
        return;
    }

    hisi_fastboot_send_byte(s, FB_HANDSHAKE);

    timer_mod(s->handshake_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + FB_HANDSHAKE_INTERVAL_NS);
}

/* ── QOM lifecycle ───────────────────────────────────────────────────── */

static void hisi_fastboot_realize(DeviceState *dev, Error **errp)
{
    HisiFastbootState *s = HISI_FASTBOOT(dev);

    qemu_chr_fe_set_handlers(&s->chr, hisi_fastboot_can_receive,
                             hisi_fastboot_receive, NULL, NULL,
                             s, NULL, true);

    if (s->v500) {
        /* V500 (GK7206): the ROM does not send 0x20 — it waits for the host's
         * BD-00-FF-01 handshake, then replies with its chip id.  No timer. */
        s->phase = FB_PHASE_V500_HS;
        s->rxpos = 0;
        qemu_log_mask(LOG_UNIMP,
                      "hisi-fastboot: V500 ROM, waiting for BD handshake on "
                      "UART0 (chip id 0x%08x)\n", s->chip_id);
        return;
    }

    /* Start handshake (classic: ROM sends 0x20 until the host ACKs) */
    s->phase = FB_PHASE_HANDSHAKE;
    s->handshake_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                       hisi_fastboot_handshake_tick, s);
    timer_mod(s->handshake_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + FB_HANDSHAKE_INTERVAL_NS);

    qemu_log_mask(LOG_UNIMP,
                  "hisi-fastboot: waiting for handshake on UART0\n");
}

static void hisi_fastboot_unrealize(DeviceState *dev)
{
    HisiFastbootState *s = HISI_FASTBOOT(dev);

    if (s->handshake_timer) {
        timer_free(s->handshake_timer);
        s->handshake_timer = NULL;
    }
}

static const Property hisi_fastboot_properties[] = {
    DEFINE_PROP_CHR("chardev", HisiFastbootState, chr),
};

static void hisi_fastboot_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_fastboot_realize;
    dc->unrealize = hisi_fastboot_unrealize;
    device_class_set_props(dc, hisi_fastboot_properties);
    dc->desc = "HiSilicon boot ROM fastboot serial protocol";
}

static const TypeInfo hisi_fastboot_info = {
    .name          = TYPE_HISI_FASTBOOT,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(HisiFastbootState),
    .class_init    = hisi_fastboot_class_init,
};

static void hisi_fastboot_register_types(void)
{
    type_register_static(&hisi_fastboot_info);
}

type_init(hisi_fastboot_register_types)
