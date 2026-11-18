/*
 * Allwinner video engine (cedar VE) at 0x01C0E000 on the V831/V833: its
 * identity, the idle flags CedarC polls, and the H.264 encoder's
 * bitstream writer and frame encode, enough for the vendor media stack's
 * closed CedarC library to produce a valid H.264 stream.
 *
 * Identity: the older parts give their IC version in 0x0F0 bits 31:16;
 * the newer ones read 0 there, a base ID in 0x0E0 and the IC in 0x0E4.
 * CedarC's VE layer keys its chip table on 0x0F0 >> 16, or else on
 * (0x0E0 << 32) | 0x0E4, and asserts on anything else; its H.264 encoder
 * takes LBC-compressed input (what the V831's video input writes) only
 * from an IC above 0x2100F.  Of the entries in the table compiled into the
 * YI firmware, three qualify: two at 600 MHz and 0x21110 / 0x21210 at
 * 300 MHz, the camera-chip clock.  That one is the default; with it the
 * media stack sets the VE to 300 MHz and opens its H.264 encoder on LBC
 * input.  (Chosen from the firmware's table, not read from silicon.)
 *
 * The AVC encoder at 0xB00 (register names from the cedrus H.264 encoder
 * reverse engineering; the handshake from CedarC's PutBits):
 *
 *   0xB18 TRIGGER  bits 3:0 command; 1 = put the low (bits 13:8) bits of
 *                  0xB20 into the bitstream, most significant first
 *   0xB1C STATUS   bit 9 ready for a command; low bits interrupt flags
 *   0xB20 BASIC_BITS
 *   0xB14 INT_EN   bits 2:0
 *   0xB80 VLE_ADDR, 0xB84 VLE_END (256-byte units: CedarC's H264InitMemory
 *   stores the addresses >> 8), 0xB88 VLE_OFFSET (bits), 0xB8C VLE_MAX
 *   (bits), 0xB90 VLE_LENGTH: the write position in bits from VLE_ADDR.
 *   The driver sets VLE_OFFSET where a frame starts, which moves the
 *   write position there, and takes the frame's size as LENGTH - OFFSET.
 *
 *   0xB04 PARAM    bit 4 P slice, bits 11:10 cabac_init_idc
 *   0xB08 QP       bits 5:0 the slice QP
 *   0xA00          picture width / 8 (bits 31:16), height / 8 (15:0)
 *   0xA78          the input picture's (luma) device address: an LBC
 *                  buffer of the video input, aw-qlbc.h's stand-in
 *   0x004          bits 11:9 memory-sync idle, read by veInquireIdle
 *
 * Command 8 encodes a frame and raises STATUS bit 0; the VE interrupt is
 * STATUS & 0xF gated by INT_EN & 7 while 0x000 selects the encoder (bit
 * 7), as the vendor cedar_ve.c handler reads it.
 *
 * The frame encode codes that picture for real (I_16x16 with DC
 * coefficients in I slices, skipped or I_PCM macroblocks in P slices; see
 * the slice coder), keeping a reference per stream.  Buffers are device
 * addresses, behind the IOMMU as its VE master.  "dump=DIR" appends each encoded frame, as the driver will take
 * it (VLE_OFFSET to VLE_LENGTH: its slice header and the slice data), to
 * DIR/avc-<VLE_ADDR>.h264, a file per stream; the SPS and PPS are put once
 * when a stream starts, before the first frame's offset.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/misc/aw-iommu.h"
#include "hw/misc/aw-qlbc.h"
#include "qom/object.h"

#define TYPE_AW_VE "aw-ve"
OBJECT_DECLARE_SIMPLE_TYPE(AwVeState, AW_VE)

#define AW_VE_SIZE          0x1000
#define AW_VE_VERSION       0x0F0
#define AW_VE_BASE_ID       0x0E0
#define AW_VE_IC_ID         0x0E4

#define VE_MODE             0x000
#define VE_MODE_AVC_ENC     (1u << 7)
#define AVC_INT_EN          0xB14
#define AVC_TRIGGER         0xB18
#define AVC_STATUS          0xB1C
#define AVC_BASIC_BITS      0xB20
#define AVC_VLE_ADDR        0xB80
#define AVC_VLE_END         0xB84
#define AVC_VLE_OFFSET      0xB88
#define AVC_VLE_MAX         0xB8C
#define AVC_VLE_LENGTH      0xB90

#define AVC_STATUS_READY    (1u << 9)
#define AVC_CMD_PUT_BITS    1
#define AVC_CMD_ENCODE      8
#define AVC_STATUS_DONE     (1u << 0)
#define AVC_INPUT_Y         0xA78

#define AW_VE_STREAMS       4
/* the largest picture coded, in macroblocks a side (4096 pixels) */
#define AW_VE_MAX_MBS       256

struct AwVeState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t ver_f0, ver_e0, ver_e4;
    char *dump;
    bool trace;
    DeviceState *iommu;
    uint32_t regs[AW_VE_SIZE / 4];
    struct AwVeStream *streams[AW_VE_STREAMS];
    unsigned next_stream;
};

static uint32_t aw_ve_vle_byte(AwVeState *s, uint32_t pos)
{
    uint32_t base = s->regs[AVC_VLE_ADDR >> 2] << 8;
    uint32_t size = (s->regs[AVC_VLE_END >> 2] + 1 -
                     s->regs[AVC_VLE_ADDR >> 2]) << 8;

    return base + (size ? (pos >> 3) % size : pos >> 3);
}

/* Append @n bits of @v (most significant first) to the bitstream. */
static void aw_ve_put_bits(AwVeState *s, uint32_t v, unsigned n)
{
    uint32_t pos = s->regs[AVC_VLE_LENGTH >> 2];
    unsigned i;

    for (i = 0; i < n; i++) {
        uint32_t byte = aw_ve_vle_byte(s, pos);
        uint8_t b;
        unsigned sh = 7 - (pos & 7);

        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE, byte, &b, 1);
        /* counts above 32 put leading zeros */
        b = (b & ~(1u << sh)) |
            ((n - 1 - i < 32 ? (v >> (n - 1 - i)) & 1 : 0) << sh);
        aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_VE, byte, &b, 1);
        pos++;
    }
    s->regs[AVC_VLE_LENGTH >> 2] += n;
}

/* --- H.264 slice coder (begin) --- */
/*
 * The frame encode: the macroblock layer of the slice whose header the
 * driver has just put, CABAC (the firmware's PPS has entropy_coding_mode
 * 1), coding a picture given as 4x4-block averages (AwH264Pic: what the
 * video input passes on in its LBC buffers, see aw-vinc.c).
 *
 * I slices: every macroblock is I_16x16, DC prediction, with the luma and
 * chroma DC coefficients only (no AC), so every 4x4 block of the decoded
 * picture is flat at its average, within the quantiser's step.  The
 * prediction comes from the decoder's reconstruction, which is tracked
 * exactly (8.3.3, 8.3.4, 8.5.10, 8.5.11).
 * P slices: a macroblock whose input is unchanged since the last frame is
 * skipped, the others are I_PCM, exact.
 *
 * CABAC is H.264 9.3; its tables are the standard's (rangeTabLPS,
 * transIdxLPS, and the m, n of every context).
 */
static const uint8_t cabac_range_lps[64][4] = {
    { 128, 176, 208, 240 },
    { 128, 167, 197, 227 },
    { 128, 158, 187, 216 },
    { 123, 150, 178, 205 },
    { 116, 142, 169, 195 },
    { 111, 135, 160, 185 },
    { 105, 128, 152, 175 },
    { 100, 122, 144, 166 },
    { 95, 116, 137, 158 },
    { 90, 110, 130, 150 },
    { 85, 104, 123, 142 },
    { 81, 99, 117, 135 },
    { 77, 94, 111, 128 },
    { 73, 89, 105, 122 },
    { 69, 85, 100, 116 },
    { 66, 80, 95, 110 },
    { 62, 76, 90, 104 },
    { 59, 72, 86, 99 },
    { 56, 69, 81, 94 },
    { 53, 65, 77, 89 },
    { 51, 62, 73, 85 },
    { 48, 59, 69, 80 },
    { 46, 56, 66, 76 },
    { 43, 53, 63, 72 },
    { 41, 50, 59, 69 },
    { 39, 48, 56, 65 },
    { 37, 45, 54, 62 },
    { 35, 43, 51, 59 },
    { 33, 41, 48, 56 },
    { 32, 39, 46, 53 },
    { 30, 37, 43, 50 },
    { 29, 35, 41, 48 },
    { 27, 33, 39, 45 },
    { 26, 31, 37, 43 },
    { 24, 30, 35, 41 },
    { 23, 28, 33, 39 },
    { 22, 27, 32, 37 },
    { 21, 26, 30, 35 },
    { 20, 24, 29, 33 },
    { 19, 23, 27, 31 },
    { 18, 22, 26, 30 },
    { 17, 21, 25, 28 },
    { 16, 20, 23, 27 },
    { 15, 19, 22, 25 },
    { 14, 18, 21, 24 },
    { 14, 17, 20, 23 },
    { 13, 16, 19, 22 },
    { 12, 15, 18, 21 },
    { 12, 14, 17, 20 },
    { 11, 14, 16, 19 },
    { 11, 13, 15, 18 },
    { 10, 12, 15, 17 },
    { 10, 12, 14, 16 },
    { 9, 11, 13, 15 },
    { 9, 11, 12, 14 },
    { 8, 10, 12, 14 },
    { 8, 9, 11, 13 },
    { 7, 9, 11, 12 },
    { 7, 9, 10, 12 },
    { 7, 8, 10, 11 },
    { 6, 8, 9, 11 },
    { 6, 7, 9, 10 },
    { 6, 7, 8, 9 },
    { 2, 2, 2, 2 },
};
static const uint8_t cabac_trans_lps[64] = {
    0, 0, 1, 2, 2, 4, 4, 5, 6, 7, 8, 9, 9, 11, 11, 12,
    13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
    24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
};
/* (m, n) of contexts 0..275 (H.264 tables 9-12 .. 9-33): I slices, then
 * P slices by cabac_init_idc */
static const int8_t cabac_mn_i[276][2] = {
    { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 },
    { 3, 74 }, { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 },
    { 7, 51 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 },
    { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 }, { -9, 83 },
    { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
    { 0, 11 }, { 1, 55 }, { 0, 69 }, { -17, 127 }, { -13, 102 },
    { 0, 82 }, { -7, 74 }, { -21, 107 }, { -27, 127 }, { -31, 127 },
    { -24, 127 }, { -18, 95 }, { -27, 127 }, { -21, 114 }, { -30, 127 },
    { -17, 123 }, { -12, 115 }, { -16, 122 }, { -11, 115 }, { -12, 63 },
    { -2, 68 }, { -15, 84 }, { -13, 104 }, { -3, 70 }, { -8, 93 },
    { -10, 90 }, { -30, 127 }, { -1, 74 }, { -6, 97 }, { -7, 91 },
    { -20, 127 }, { -4, 56 }, { -5, 82 }, { -7, 76 }, { -22, 125 },
    { -7, 93 }, { -11, 87 }, { -3, 77 }, { -5, 71 }, { -4, 63 },
    { -4, 68 }, { -12, 84 }, { -7, 62 }, { -7, 65 }, { 8, 61 },
    { 5, 56 }, { -2, 66 }, { 1, 64 }, { 0, 61 }, { -2, 78 },
    { 1, 50 }, { 7, 52 }, { 10, 35 }, { 0, 44 }, { 11, 38 },
    { 1, 45 }, { 0, 46 }, { 5, 44 }, { 31, 17 }, { 1, 51 },
    { 7, 50 }, { 28, 19 }, { 16, 33 }, { 14, 62 }, { -13, 108 },
    { -15, 100 }, { -13, 101 }, { -13, 91 }, { -12, 94 }, { -10, 88 },
    { -16, 84 }, { -10, 86 }, { -7, 83 }, { -13, 87 }, { -19, 94 },
    { 1, 70 }, { 0, 72 }, { -5, 74 }, { 18, 59 }, { -8, 102 },
    { -15, 100 }, { 0, 95 }, { -4, 75 }, { 2, 72 }, { -11, 75 },
    { -3, 71 }, { 15, 46 }, { -13, 69 }, { 0, 62 }, { 0, 65 },
    { 21, 37 }, { -15, 72 }, { 9, 57 }, { 16, 54 }, { 0, 62 },
    { 12, 72 }, { 24, 0 }, { 15, 9 }, { 8, 25 }, { 13, 18 },
    { 15, 9 }, { 13, 19 }, { 10, 37 }, { 12, 18 }, { 6, 29 },
    { 20, 33 }, { 15, 30 }, { 4, 45 }, { 1, 58 }, { 0, 62 },
    { 7, 61 }, { 12, 38 }, { 11, 45 }, { 15, 39 }, { 11, 42 },
    { 13, 44 }, { 16, 45 }, { 12, 41 }, { 10, 49 }, { 30, 34 },
    { 18, 42 }, { 10, 55 }, { 17, 51 }, { 17, 46 }, { 0, 89 },
    { 26, -19 }, { 22, -17 }, { 26, -17 }, { 30, -25 }, { 28, -20 },
    { 33, -23 }, { 37, -27 }, { 33, -23 }, { 40, -28 }, { 38, -17 },
    { 33, -11 }, { 40, -15 }, { 41, -6 }, { 38, 1 }, { 41, 17 },
    { 30, -6 }, { 27, 3 }, { 26, 22 }, { 37, -16 }, { 35, -4 },
    { 38, -8 }, { 38, -3 }, { 37, 3 }, { 38, 5 }, { 42, 0 },
    { 35, 16 }, { 39, 22 }, { 14, 48 }, { 27, 37 }, { 21, 60 },
    { 12, 68 }, { 2, 97 }, { -3, 71 }, { -6, 42 }, { -5, 50 },
    { -3, 54 }, { -2, 62 }, { 0, 58 }, { 1, 63 }, { -2, 72 },
    { -1, 74 }, { -9, 91 }, { -5, 67 }, { -5, 27 }, { -3, 39 },
    { -2, 44 }, { 0, 46 }, { -16, 64 }, { -8, 68 }, { -10, 78 },
    { -6, 77 }, { -10, 86 }, { -12, 92 }, { -15, 55 }, { -10, 60 },
    { -6, 62 }, { -4, 65 }, { -12, 73 }, { -8, 76 }, { -7, 80 },
    { -9, 88 }, { -17, 110 }, { -11, 97 }, { -20, 84 }, { -11, 79 },
    { -6, 73 }, { -4, 74 }, { -13, 86 }, { -13, 96 }, { -11, 97 },
    { -19, 117 }, { -8, 78 }, { -5, 33 }, { -4, 48 }, { -2, 53 },
    { -3, 62 }, { -13, 71 }, { -10, 79 }, { -12, 86 }, { -13, 90 },
    { -14, 97 },
};
static const int8_t cabac_mn_p[3][276][2] = {
    {
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 },
        { 3, 74 }, { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 },
        { 7, 51 }, { 23, 33 }, { 23, 2 }, { 21, 0 }, { 1, 9 },
        { 0, 49 }, { -37, 118 }, { 5, 57 }, { -13, 78 }, { -11, 65 },
        { 1, 62 }, { 12, 49 }, { -4, 73 }, { 17, 50 }, { 18, 64 },
        { 9, 43 }, { 29, 0 }, { 26, 67 }, { 16, 90 }, { 9, 104 },
        { -46, 127 }, { -20, 104 }, { 1, 67 }, { -13, 78 }, { -11, 65 },
        { 1, 62 }, { -6, 86 }, { -17, 95 }, { -6, 61 }, { 9, 45 },
        { -3, 69 }, { -6, 81 }, { -11, 96 }, { 6, 55 }, { 7, 67 },
        { -5, 86 }, { 2, 88 }, { 0, 58 }, { -3, 76 }, { -10, 94 },
        { 5, 54 }, { 4, 69 }, { -3, 81 }, { 0, 88 }, { -7, 67 },
        { -5, 74 }, { -4, 74 }, { -5, 80 }, { -7, 72 }, { 1, 58 },
        { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 }, { -9, 83 },
        { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
        { 0, 45 }, { -4, 78 }, { -3, 96 }, { -27, 126 }, { -28, 98 },
        { -25, 101 }, { -23, 67 }, { -28, 82 }, { -20, 94 }, { -16, 83 },
        { -22, 110 }, { -21, 91 }, { -18, 102 }, { -13, 93 }, { -29, 127 },
        { -7, 92 }, { -5, 89 }, { -7, 96 }, { -13, 108 }, { -3, 46 },
        { -1, 65 }, { -1, 57 }, { -9, 93 }, { -3, 74 }, { -9, 92 },
        { -8, 87 }, { -23, 126 }, { 5, 54 }, { 6, 60 }, { 6, 59 },
        { 6, 69 }, { -1, 48 }, { 0, 68 }, { -4, 69 }, { -8, 88 },
        { -2, 85 }, { -6, 78 }, { -1, 75 }, { -7, 77 }, { 2, 54 },
        { 5, 50 }, { -3, 68 }, { 1, 50 }, { 6, 42 }, { -4, 81 },
        { 1, 63 }, { -4, 70 }, { 0, 67 }, { 2, 57 }, { -2, 76 },
        { 11, 35 }, { 4, 64 }, { 1, 61 }, { 11, 35 }, { 18, 25 },
        { 12, 24 }, { 13, 29 }, { 13, 36 }, { -10, 93 }, { -7, 73 },
        { -2, 73 }, { 13, 46 }, { 9, 49 }, { -7, 100 }, { 9, 53 },
        { 2, 53 }, { 5, 53 }, { -2, 61 }, { 0, 56 }, { 0, 56 },
        { -13, 63 }, { -5, 60 }, { -1, 62 }, { 4, 57 }, { -6, 69 },
        { 4, 57 }, { 14, 39 }, { 4, 51 }, { 13, 68 }, { 3, 64 },
        { 1, 61 }, { 9, 63 }, { 7, 50 }, { 16, 39 }, { 5, 44 },
        { 4, 52 }, { 11, 48 }, { -5, 60 }, { -1, 59 }, { 0, 59 },
        { 22, 33 }, { 5, 44 }, { 14, 43 }, { -1, 78 }, { 0, 60 },
        { 9, 69 }, { 11, 28 }, { 2, 40 }, { 3, 44 }, { 0, 49 },
        { 0, 46 }, { 2, 44 }, { 2, 51 }, { 0, 47 }, { 4, 39 },
        { 2, 62 }, { 6, 46 }, { 0, 54 }, { 3, 54 }, { 2, 58 },
        { 4, 63 }, { 6, 51 }, { 6, 57 }, { 7, 53 }, { 6, 52 },
        { 6, 55 }, { 11, 45 }, { 14, 36 }, { 8, 53 }, { -1, 82 },
        { 7, 55 }, { -3, 78 }, { 15, 46 }, { 22, 31 }, { -1, 84 },
        { 25, 7 }, { 30, -7 }, { 28, 3 }, { 28, 4 }, { 32, 0 },
        { 34, -1 }, { 30, 6 }, { 30, 6 }, { 32, 9 }, { 31, 19 },
        { 26, 27 }, { 26, 30 }, { 37, 20 }, { 28, 34 }, { 17, 70 },
        { 1, 67 }, { 5, 59 }, { 9, 67 }, { 16, 30 }, { 18, 32 },
        { 18, 35 }, { 22, 29 }, { 24, 31 }, { 23, 38 }, { 18, 43 },
        { 20, 41 }, { 11, 63 }, { 9, 59 }, { 9, 64 }, { -1, 94 },
        { -2, 89 }, { -9, 108 }, { -6, 76 }, { -2, 44 }, { 0, 45 },
        { 0, 52 }, { -3, 64 }, { -2, 59 }, { -4, 70 }, { -4, 75 },
        { -8, 82 }, { -17, 102 }, { -9, 77 }, { 3, 24 }, { 0, 42 },
        { 0, 48 }, { 0, 55 }, { -6, 59 }, { -7, 71 }, { -12, 83 },
        { -11, 87 }, { -30, 119 }, { 1, 58 }, { -3, 29 }, { -1, 36 },
        { 1, 38 }, { 2, 43 }, { -6, 55 }, { 0, 58 }, { 0, 64 },
        { -3, 74 }, { -10, 90 }, { 0, 70 }, { -4, 29 }, { 5, 31 },
        { 7, 42 }, { 1, 59 }, { -2, 58 }, { -3, 72 }, { -3, 81 },
        { -11, 97 }, { 0, 58 }, { 8, 5 }, { 10, 14 }, { 14, 18 },
        { 13, 27 }, { 2, 40 }, { 0, 58 }, { -3, 70 }, { -6, 79 },
        { -8, 85 },
    },
    {
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 },
        { 3, 74 }, { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 },
        { 7, 51 }, { 22, 25 }, { 34, 0 }, { 16, 0 }, { -2, 9 },
        { 4, 41 }, { -29, 118 }, { 2, 65 }, { -6, 71 }, { -13, 79 },
        { 5, 52 }, { 9, 50 }, { -3, 70 }, { 10, 54 }, { 26, 34 },
        { 19, 22 }, { 40, 0 }, { 57, 2 }, { 41, 36 }, { 26, 69 },
        { -45, 127 }, { -15, 101 }, { -4, 76 }, { -6, 71 }, { -13, 79 },
        { 5, 52 }, { 6, 69 }, { -13, 90 }, { 0, 52 }, { 8, 43 },
        { -2, 69 }, { -5, 82 }, { -10, 96 }, { 2, 59 }, { 2, 75 },
        { -3, 87 }, { -3, 100 }, { 1, 56 }, { -3, 74 }, { -6, 85 },
        { 0, 59 }, { -3, 81 }, { -7, 86 }, { -5, 95 }, { -1, 66 },
        { -1, 77 }, { 1, 70 }, { -2, 86 }, { -5, 72 }, { 0, 61 },
        { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 }, { -9, 83 },
        { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
        { 13, 15 }, { 7, 51 }, { 2, 80 }, { -39, 127 }, { -18, 91 },
        { -17, 96 }, { -26, 81 }, { -35, 98 }, { -24, 102 }, { -23, 97 },
        { -27, 119 }, { -24, 99 }, { -21, 110 }, { -18, 102 }, { -36, 127 },
        { 0, 80 }, { -5, 89 }, { -7, 94 }, { -4, 92 }, { 0, 39 },
        { 0, 65 }, { -15, 84 }, { -35, 127 }, { -2, 73 }, { -12, 104 },
        { -9, 91 }, { -31, 127 }, { 3, 55 }, { 7, 56 }, { 7, 55 },
        { 8, 61 }, { -3, 53 }, { 0, 68 }, { -7, 74 }, { -9, 88 },
        { -13, 103 }, { -13, 91 }, { -9, 89 }, { -14, 92 }, { -8, 76 },
        { -12, 87 }, { -23, 110 }, { -24, 105 }, { -10, 78 }, { -20, 112 },
        { -17, 99 }, { -78, 127 }, { -70, 127 }, { -50, 127 }, { -46, 127 },
        { -4, 66 }, { -5, 78 }, { -4, 71 }, { -8, 72 }, { 2, 59 },
        { -1, 55 }, { -7, 70 }, { -6, 75 }, { -8, 89 }, { -34, 119 },
        { -3, 75 }, { 32, 20 }, { 30, 22 }, { -44, 127 }, { 0, 54 },
        { -5, 61 }, { 0, 58 }, { -1, 60 }, { -3, 61 }, { -8, 67 },
        { -25, 84 }, { -14, 74 }, { -5, 65 }, { 5, 52 }, { 2, 57 },
        { 0, 61 }, { -9, 69 }, { -11, 70 }, { 18, 55 }, { -4, 71 },
        { 0, 58 }, { 7, 61 }, { 9, 41 }, { 18, 25 }, { 9, 32 },
        { 5, 43 }, { 9, 47 }, { 0, 44 }, { 0, 51 }, { 2, 46 },
        { 19, 38 }, { -4, 66 }, { 15, 38 }, { 12, 42 }, { 9, 34 },
        { 0, 89 }, { 4, 45 }, { 10, 28 }, { 10, 31 }, { 33, -11 },
        { 52, -43 }, { 18, 15 }, { 28, 0 }, { 35, -22 }, { 38, -25 },
        { 34, 0 }, { 39, -18 }, { 32, -12 }, { 102, -94 }, { 0, 0 },
        { 56, -15 }, { 33, -4 }, { 29, 10 }, { 37, -5 }, { 51, -29 },
        { 39, -9 }, { 52, -34 }, { 69, -58 }, { 67, -63 }, { 44, -5 },
        { 32, 7 }, { 55, -29 }, { 32, 1 }, { 0, 0 }, { 27, 36 },
        { 33, -25 }, { 34, -30 }, { 36, -28 }, { 38, -28 }, { 38, -27 },
        { 34, -18 }, { 35, -16 }, { 34, -14 }, { 32, -8 }, { 37, -6 },
        { 35, 0 }, { 30, 10 }, { 28, 18 }, { 26, 25 }, { 29, 41 },
        { 0, 75 }, { 2, 72 }, { 8, 77 }, { 14, 35 }, { 18, 31 },
        { 17, 35 }, { 21, 30 }, { 17, 45 }, { 20, 42 }, { 18, 45 },
        { 27, 26 }, { 16, 54 }, { 7, 66 }, { 16, 56 }, { 11, 73 },
        { 10, 67 }, { -10, 116 }, { -23, 112 }, { -15, 71 }, { -7, 61 },
        { 0, 53 }, { -5, 66 }, { -11, 77 }, { -9, 80 }, { -9, 84 },
        { -10, 87 }, { -34, 127 }, { -21, 101 }, { -3, 39 }, { -5, 53 },
        { -7, 61 }, { -11, 75 }, { -15, 77 }, { -17, 91 }, { -25, 107 },
        { -25, 111 }, { -28, 122 }, { -11, 76 }, { -10, 44 }, { -10, 52 },
        { -10, 57 }, { -9, 58 }, { -16, 72 }, { -7, 69 }, { -4, 69 },
        { -5, 74 }, { -9, 86 }, { 2, 66 }, { -9, 34 }, { 1, 32 },
        { 11, 31 }, { 5, 52 }, { -2, 55 }, { -2, 67 }, { 0, 73 },
        { -8, 89 }, { 3, 52 }, { 7, 4 }, { 10, 8 }, { 17, 8 },
        { 16, 19 }, { 3, 37 }, { -1, 61 }, { -5, 73 }, { -1, 70 },
        { -4, 78 },
    },
    {
        { 20, -15 }, { 2, 54 }, { 3, 74 }, { 20, -15 }, { 2, 54 },
        { 3, 74 }, { -28, 127 }, { -23, 104 }, { -6, 53 }, { -1, 54 },
        { 7, 51 }, { 29, 16 }, { 25, 0 }, { 14, 0 }, { -10, 51 },
        { -3, 62 }, { -27, 99 }, { 26, 16 }, { -4, 85 }, { -24, 102 },
        { 5, 57 }, { 6, 57 }, { -17, 73 }, { 14, 57 }, { 20, 40 },
        { 20, 10 }, { 29, 0 }, { 54, 0 }, { 37, 42 }, { 12, 97 },
        { -32, 127 }, { -22, 117 }, { -2, 74 }, { -4, 85 }, { -24, 102 },
        { 5, 57 }, { -6, 93 }, { -14, 88 }, { -6, 44 }, { 4, 55 },
        { -11, 89 }, { -15, 103 }, { -21, 116 }, { 19, 57 }, { 20, 58 },
        { 4, 84 }, { 6, 96 }, { 1, 63 }, { -5, 85 }, { -13, 106 },
        { 5, 63 }, { 6, 75 }, { -3, 90 }, { -1, 101 }, { 3, 55 },
        { -4, 79 }, { -2, 75 }, { -12, 97 }, { -7, 50 }, { 1, 60 },
        { 0, 41 }, { 0, 63 }, { 0, 63 }, { 0, 63 }, { -9, 83 },
        { 4, 86 }, { 0, 97 }, { -7, 72 }, { 13, 41 }, { 3, 62 },
        { 7, 34 }, { -9, 88 }, { -20, 127 }, { -36, 127 }, { -17, 91 },
        { -14, 95 }, { -25, 84 }, { -25, 86 }, { -12, 89 }, { -17, 91 },
        { -31, 127 }, { -14, 76 }, { -18, 103 }, { -13, 90 }, { -37, 127 },
        { 11, 80 }, { 5, 76 }, { 2, 84 }, { 5, 78 }, { -6, 55 },
        { 4, 61 }, { -14, 83 }, { -37, 127 }, { -5, 79 }, { -11, 104 },
        { -11, 91 }, { -30, 127 }, { 0, 65 }, { -2, 79 }, { 0, 72 },
        { -4, 92 }, { -6, 56 }, { 3, 68 }, { -8, 71 }, { -13, 98 },
        { -4, 86 }, { -12, 88 }, { -5, 82 }, { -3, 72 }, { -4, 67 },
        { -8, 72 }, { -16, 89 }, { -9, 69 }, { -1, 59 }, { 5, 66 },
        { 4, 57 }, { -4, 71 }, { -2, 71 }, { 2, 58 }, { -1, 74 },
        { -4, 44 }, { -1, 69 }, { 0, 62 }, { -7, 51 }, { -4, 47 },
        { -6, 42 }, { -3, 41 }, { -6, 53 }, { 8, 76 }, { -9, 78 },
        { -11, 83 }, { 9, 52 }, { 0, 67 }, { -5, 90 }, { 1, 67 },
        { -15, 72 }, { -5, 75 }, { -8, 80 }, { -21, 83 }, { -21, 64 },
        { -13, 31 }, { -25, 64 }, { -29, 94 }, { 9, 75 }, { 17, 63 },
        { -8, 74 }, { -5, 35 }, { -2, 27 }, { 13, 91 }, { 3, 65 },
        { -7, 69 }, { 8, 77 }, { -10, 66 }, { 3, 62 }, { -3, 68 },
        { -20, 81 }, { 0, 30 }, { 1, 7 }, { -3, 23 }, { -21, 74 },
        { 16, 66 }, { -23, 124 }, { 17, 37 }, { 44, -18 }, { 50, -34 },
        { -22, 127 }, { 4, 39 }, { 0, 42 }, { 7, 34 }, { 11, 29 },
        { 8, 31 }, { 6, 37 }, { 7, 42 }, { 3, 40 }, { 8, 33 },
        { 13, 43 }, { 13, 36 }, { 4, 47 }, { 3, 55 }, { 2, 58 },
        { 6, 60 }, { 8, 44 }, { 11, 44 }, { 14, 42 }, { 7, 48 },
        { 4, 56 }, { 4, 52 }, { 13, 37 }, { 9, 49 }, { 19, 58 },
        { 10, 48 }, { 12, 45 }, { 0, 69 }, { 20, 33 }, { 8, 63 },
        { 35, -18 }, { 33, -25 }, { 28, -3 }, { 24, 10 }, { 27, 0 },
        { 34, -14 }, { 52, -44 }, { 39, -24 }, { 19, 17 }, { 31, 25 },
        { 36, 29 }, { 24, 33 }, { 34, 15 }, { 30, 20 }, { 22, 73 },
        { 20, 34 }, { 19, 31 }, { 27, 44 }, { 19, 16 }, { 15, 36 },
        { 15, 36 }, { 21, 28 }, { 25, 21 }, { 30, 20 }, { 31, 12 },
        { 27, 16 }, { 24, 42 }, { 0, 93 }, { 14, 56 }, { 15, 57 },
        { 26, 38 }, { -24, 127 }, { -24, 115 }, { -22, 82 }, { -9, 62 },
        { 0, 53 }, { 0, 59 }, { -14, 85 }, { -13, 89 }, { -13, 94 },
        { -11, 92 }, { -29, 127 }, { -21, 100 }, { -14, 57 }, { -12, 67 },
        { -11, 71 }, { -10, 77 }, { -21, 85 }, { -16, 88 }, { -23, 104 },
        { -15, 98 }, { -37, 127 }, { -10, 82 }, { -8, 48 }, { -8, 61 },
        { -8, 66 }, { -7, 70 }, { -14, 75 }, { -10, 79 }, { -9, 83 },
        { -12, 92 }, { -18, 108 }, { -4, 79 }, { -22, 69 }, { -16, 75 },
        { -2, 58 }, { 1, 58 }, { -13, 78 }, { -9, 83 }, { -4, 81 },
        { -13, 99 }, { -13, 81 }, { -6, 38 }, { -13, 62 }, { -6, 58 },
        { -2, 59 }, { -16, 73 }, { -10, 76 }, { -13, 86 }, { -9, 83 },
        { -10, 87 },
    },
};

#define CABAC_CTX_TERMINATE 276

typedef struct AwCabac {
    uint8_t state[CABAC_CTX_TERMINATE];     /* pStateIdx */
    uint8_t mps[CABAC_CTX_TERMINATE];
    uint32_t low, range;
    int outstanding;
    bool first;
    GByteArray *out;
    uint8_t cur;
    int nbits;
} AwCabac;

static void cabac_write_bit(AwCabac *c, int b)
{
    c->cur = (c->cur << 1) | b;
    if (++c->nbits == 8) {
        g_byte_array_append(c->out, &c->cur, 1);
        c->cur = 0;
        c->nbits = 0;
    }
}

static void cabac_put_bit(AwCabac *c, int b)
{
    if (c->first) {
        c->first = false;
    } else {
        cabac_write_bit(c, b);
    }
    for (; c->outstanding > 0; c->outstanding--) {
        cabac_write_bit(c, !b);
    }
}

static void cabac_renorm(AwCabac *c)
{
    while (c->range < 256) {
        if (c->low < 256) {
            cabac_put_bit(c, 0);
        } else if (c->low >= 512) {
            c->low -= 512;
            cabac_put_bit(c, 1);
        } else {
            c->low -= 256;
            c->outstanding++;
        }
        c->range <<= 1;
        c->low <<= 1;
    }
}

static void cabac_start(AwCabac *c)
{
    c->low = 0;
    c->range = 510;
    c->outstanding = 0;
    c->first = true;
}

static void cabac_decision(AwCabac *c, int ctx, int bin)
{
    uint32_t lps = cabac_range_lps[c->state[ctx]][(c->range >> 6) & 3];

    c->range -= lps;
    if (bin != c->mps[ctx]) {
        c->low += c->range;
        c->range = lps;
        if (c->state[ctx] == 0) {
            c->mps[ctx] = !c->mps[ctx];
        }
        c->state[ctx] = cabac_trans_lps[c->state[ctx]];
    } else if (c->state[ctx] < 62) {
        c->state[ctx]++;
    }
    cabac_renorm(c);
}

static void cabac_bypass(AwCabac *c, int bin)
{
    c->low <<= 1;
    if (bin) {
        c->low += c->range;
    }
    if (c->low >= 1024) {
        cabac_put_bit(c, 1);
        c->low -= 1024;
    } else if (c->low < 512) {
        cabac_put_bit(c, 0);
    } else {
        c->low -= 512;
        c->outstanding++;
    }
}

/*
 * end_of_slice_flag, and mb_type's I_PCM bin.  Its 1 flushes the engine
 * (9.3.4.5): the last bit is the rbsp_stop_one_bit, then zero bits to the
 * byte boundary (rbsp_alignment_zero_bit / pcm_alignment_zero_bit).
 */
static void cabac_terminate(AwCabac *c, int bin)
{
    c->range -= 2;
    if (!bin) {
        cabac_renorm(c);
        return;
    }
    c->low += c->range;
    c->range = 2;
    cabac_renorm(c);
    cabac_put_bit(c, (c->low >> 9) & 1);
    cabac_write_bit(c, (c->low >> 8) & 1);
    cabac_write_bit(c, 1);
    while (c->nbits) {
        cabac_write_bit(c, 0);
    }
}

static void cabac_init_contexts(AwCabac *c, bool p_slice, int idc, int qp)
{
    const int8_t (*mn)[2] = p_slice ? cabac_mn_p[idc] : cabac_mn_i;
    int i;

    for (i = 0; i < CABAC_CTX_TERMINATE; i++) {
        int pre = ((mn[i][0] * qp) >> 4) + mn[i][1];

        pre = MIN(MAX(pre, 1), 126);
        c->state[i] = pre <= 63 ? 63 - pre : pre - 64;
        c->mps[i] = pre > 63;
    }
}

/*
 * residual_block_cabac() for a block whose coded_block_flag is 1, the
 * coefficients in scan order; cat 0 (Intra16x16DCLevel) or 3 (chroma DC).
 */
static void cabac_residual(AwCabac *c, const int *coef, int n, int cat)
{
    static const int sig_off[5] = { 0, 15, 29, 44, 47 };
    static const int abs_off[5] = { 0, 10, 20, 30, 39 };
    int i, last = -1, gt1 = 0, eq1 = 0;

    for (i = 0; i < n; i++) {
        if (coef[i]) {
            last = i;
        }
    }
    for (i = 0; i < n - 1; i++) {
        int inc = cat == 3 ? MIN(i, 2) : i;

        cabac_decision(c, 105 + sig_off[cat] + inc, coef[i] != 0);
        if (coef[i]) {
            cabac_decision(c, 166 + sig_off[cat] + inc, i == last);
            if (i == last) {
                break;
            }
        }
    }
    for (i = last; i >= 0; i--) {
        int a;

        if (!coef[i]) {
            continue;
        }
        a = abs(coef[i]) - 1;
        cabac_decision(c, 227 + abs_off[cat] + (gt1 ? 0 : MIN(4, 1 + eq1)),
                       a > 0);
        if (a > 0) {
            int ctx = 227 + abs_off[cat] + 5 + MIN(4 - (cat == 3), gt1);
            int k;

            for (k = 1; k < MIN(a, 14); k++) {
                cabac_decision(c, ctx, 1);
            }
            if (a < 14) {
                cabac_decision(c, ctx, 0);
            } else {
                /* UEG0 suffix, bypass */
                int suf = a - 14;

                k = 0;
                while (suf >= (1 << k)) {
                    cabac_bypass(c, 1);
                    suf -= 1 << k;
                    k++;
                }
                cabac_bypass(c, 0);
                while (k--) {
                    cabac_bypass(c, (suf >> k) & 1);
                }
            }
        }
        cabac_bypass(c, coef[i] < 0);
        if (a == 0) {
            eq1++;
        } else {
            gt1++;
        }
    }
}

/* A picture as 4x4-block values: luma (mbw*4 x mbh*4), Cb, Cr (2 x 2 a MB) */
typedef struct AwH264Pic {
    unsigned mbw, mbh;
    uint8_t *y, *u, *v;
} AwH264Pic;

enum { MB_SKIP, MB_I16, MB_PCM };

typedef struct AwH264Mb {
    uint8_t type, cbf_dc, cbf_c[2], cbpc;
} AwH264Mb;

static void aw_h264_pic_alloc(AwH264Pic *p, unsigned mbw, unsigned mbh)
{
    p->mbw = mbw;
    p->mbh = mbh;
    p->y = g_malloc0(mbw * mbh * 16);
    p->u = g_malloc0(mbw * mbh * 4);
    p->v = g_malloc0(mbw * mbh * 4);
}

static void aw_h264_pic_free(AwH264Pic *p)
{
    g_free(p->y);
    g_free(p->u);
    g_free(p->v);
    p->y = p->u = p->v = NULL;
}

/* zigzag (frame) scan of a 4x4 block: scan index -> y * 4 + x */
static const uint8_t h264_zigzag4[16] = {
    0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};
static const uint8_t h264_chroma_qp[22] = {      /* QPc for qPI 30..51 */
    29, 30, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38,
    39, 39, 39, 39,
};
static const uint8_t h264_v0[6] = { 10, 11, 13, 14, 16, 18 };

static int h264_clip(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static int h264_round_div(int a, int b)
{
    return a >= 0 ? (a + b / 2) / b : -((-a + b / 2) / b);
}

/* Intra16x16 DC prediction of macroblock (mx, my) from @r (8.3.3.3). */
static int h264_luma_dc_pred(const AwH264Pic *r, unsigned mx, unsigned my)
{
    unsigned bw = r->mbw * 4, k;
    int top = 0, left = 0;

    for (k = 0; k < 4; k++) {
        if (my) {
            top += 4 * r->y[(my * 4 - 1) * bw + mx * 4 + k];
        }
        if (mx) {
            left += 4 * r->y[(my * 4 + k) * bw + mx * 4 - 1];
        }
    }
    if (mx && my) {
        return (top + left + 16) >> 5;
    } else if (mx) {
        return (left + 8) >> 4;
    } else if (my) {
        return (top + 8) >> 4;
    }
    return 128;
}

/* Chroma DC prediction of 4x4 block (bx, by) of the MB (8.3.4.1-3). */
static int h264_chroma_dc_pred(const uint8_t *pl, const AwH264Pic *r,
                               unsigned mx, unsigned my, int bx, int by)
{
    unsigned cw = r->mbw * 2;
    int top = 0, left = 0;
    bool t = my > 0, l = mx > 0;

    if (t) {
        top = 4 * pl[(my * 2 - 1) * cw + mx * 2 + bx];
    }
    if (l) {
        left = 4 * pl[(my * 2 + by) * cw + mx * 2 - 1];
    }
    if ((bx == 0 && by == 0) || (bx == 1 && by == 1)) {
        if (t && l) {
            return (top + left + 4) >> 3;
        }
        if (l) {
            return (left + 2) >> 2;
        }
        if (t) {
            return (top + 2) >> 2;
        }
    } else if (bx == 1 && by == 0) {
        if (t) {
            return (top + 2) >> 2;
        }
        if (l) {
            return (left + 2) >> 2;
        }
    } else {
        if (l) {
            return (left + 2) >> 2;
        }
        if (t) {
            return (top + 2) >> 2;
        }
    }
    return 128;
}

/* The decoder's luma DC (8.5.10), Hadamard domain @c (y * 4 + x) in. */
static void h264_luma_dc_dequant(const int *c, int qp, int *dc)
{
    int f[16], t[16], i, j, ls = 16 * h264_v0[qp % 6];

    for (i = 0; i < 4; i++) {           /* rows: t = c H */
        int a = c[i * 4], b = c[i * 4 + 1], d = c[i * 4 + 2], e = c[i * 4 + 3];

        t[i * 4 + 0] = a + b + d + e;
        t[i * 4 + 1] = a + b - d - e;
        t[i * 4 + 2] = a - b - d + e;
        t[i * 4 + 3] = a - b + d - e;
    }
    for (j = 0; j < 4; j++) {           /* columns: f = H t */
        int a = t[j], b = t[4 + j], d = t[8 + j], e = t[12 + j];

        f[j] = a + b + d + e;
        f[4 + j] = a + b - d - e;
        f[8 + j] = a - b - d + e;
        f[12 + j] = a - b + d - e;
    }
    for (i = 0; i < 16; i++) {
        if (qp >= 36) {
            dc[i] = (f[i] * ls) << (qp / 6 - 6);
        } else {
            dc[i] = (f[i] * ls + (1 << (5 - qp / 6))) >> (6 - qp / 6);
        }
    }
}

/* The decoder's chroma DC (8.5.11.2), c[0..3] = ChromaDCLevel order. */
static void h264_chroma_dc_dequant(const int *c, int qpc, int *dc)
{
    int f[4], ls = 16 * h264_v0[qpc % 6], i;

    f[0] = c[0] + c[1] + c[2] + c[3];
    f[1] = c[0] - c[1] + c[2] - c[3];
    f[2] = c[0] + c[1] - c[2] - c[3];
    f[3] = c[0] - c[1] - c[2] + c[3];
    for (i = 0; i < 4; i++) {
        dc[i] = ((f[i] * ls) << (qpc / 6)) >> 5;
    }
}

/*
 * Code one I_16x16 macroblock of an I slice: target @t, reconstruction
 * @r (updated), neighbour info @mbs.
 */
static void h264_code_i16(AwCabac *c, const AwH264Pic *t, AwH264Pic *r,
                          AwH264Mb *mbs, unsigned mx, unsigned my, int qp)
{
    unsigned bw = r->mbw * 4, cw = r->mbw * 2;
    AwH264Mb *mb = &mbs[my * r->mbw + mx];
    AwH264Mb *a = mx ? mb - 1 : NULL, *b = my ? mb - r->mbw : NULL;
    int pred = h264_luma_dc_pred(r, mx, my);
    int qpc = qp < 30 ? qp : h264_chroma_qp[MIN(qp, 51) - 30];
    int lev[16], dc[16], scan[16], i, k, comp, ca, cb;
    int clev[2][4], cdc[4];
    int ls = 16 * h264_v0[qp % 6], lsc = 16 * h264_v0[qpc % 6];

    /* luma: residual per 4x4 block -> DC levels, Hadamard domain */
    {
        int f[16], tt[16];

        for (i = 0; i < 16; i++) {
            int res = t->y[(my * 4 + i / 4) * bw + mx * 4 + i % 4] - pred;
            int dct = 64 * res;

            /* invert dcY = (f * ls [+ round]) >> (6 - qp / 6) */
            f[i] = qp >= 36 ? h264_round_div(dct, ls << (qp / 6 - 6))
                            : h264_round_div(dct << (6 - qp / 6), ls);
        }
        /* c = H f H / 16 */
        for (i = 0; i < 4; i++) {
            int p0 = f[i * 4], p1 = f[i * 4 + 1], p2 = f[i * 4 + 2];
            int p3 = f[i * 4 + 3];

            tt[i * 4 + 0] = p0 + p1 + p2 + p3;
            tt[i * 4 + 1] = p0 + p1 - p2 - p3;
            tt[i * 4 + 2] = p0 - p1 - p2 + p3;
            tt[i * 4 + 3] = p0 - p1 + p2 - p3;
        }
        for (k = 0; k < 4; k++) {
            int p0 = tt[k], p1 = tt[4 + k], p2 = tt[8 + k], p3 = tt[12 + k];

            lev[k] = h264_round_div(p0 + p1 + p2 + p3, 16);
            lev[4 + k] = h264_round_div(p0 + p1 - p2 - p3, 16);
            lev[8 + k] = h264_round_div(p0 - p1 - p2 + p3, 16);
            lev[12 + k] = h264_round_div(p0 - p1 + p2 - p3, 16);
        }
    }
    h264_luma_dc_dequant(lev, qp, dc);
    mb->cbf_dc = 0;
    for (i = 0; i < 16; i++) {
        int y = my * 4 + i / 4, x = mx * 4 + i % 4;

        r->y[y * bw + x] = h264_clip(pred + ((dc[i] + 32) >> 6));
        scan[i] = lev[h264_zigzag4[i]];
        mb->cbf_dc |= lev[i] != 0;
    }

    /* chroma: per component, DC only */
    mb->cbpc = 0;
    for (comp = 0; comp < 2; comp++) {
        const uint8_t *tp = comp ? t->v : t->u;
        uint8_t *rp = comp ? r->v : r->u;
        int pr[4], f[4];

        for (i = 0; i < 4; i++) {
            int bx = i & 1, by = i >> 1;
            int res = tp[(my * 2 + by) * cw + mx * 2 + bx];

            pr[i] = h264_chroma_dc_pred(rp, r, mx, my, bx, by);
            res -= pr[i];
            /* invert dcC = ((f * lsc) << (qpc / 6)) >> 5 */
            f[i] = h264_round_div(64 * res * 32, lsc << (qpc / 6));
        }
        /* c = Hc f Hc / 4 */
        clev[comp][0] = h264_round_div(f[0] + f[1] + f[2] + f[3], 4);
        clev[comp][1] = h264_round_div(f[0] - f[1] + f[2] - f[3], 4);
        clev[comp][2] = h264_round_div(f[0] + f[1] - f[2] - f[3], 4);
        clev[comp][3] = h264_round_div(f[0] - f[1] - f[2] + f[3], 4);
        h264_chroma_dc_dequant(clev[comp], qpc, cdc);
        mb->cbf_c[comp] = 0;
        for (i = 0; i < 4; i++) {
            int bx = i & 1, by = i >> 1;

            rp[(my * 2 + by) * cw + mx * 2 + bx] =
                h264_clip(pr[i] + ((cdc[i] + 32) >> 6));
            mb->cbf_c[comp] |= clev[comp][i] != 0;
        }
        mb->cbpc |= mb->cbf_c[comp];
    }

    /* mb_type I_16x16_2_<cbpc>_0: 1, not PCM, no luma AC, chroma, mode 2 */
    cabac_decision(c, 3 + (a != NULL) + (b != NULL), 1);
    cabac_terminate(c, 0);
    cabac_decision(c, 3 + 3, 0);
    cabac_decision(c, 3 + 4, mb->cbpc != 0);
    if (mb->cbpc) {
        cabac_decision(c, 3 + 5, 0);                /* cbpc 1: DC only */
    }
    cabac_decision(c, 3 + 6, 1);
    cabac_decision(c, 3 + 7, 0);
    cabac_decision(c, 64, 0);                       /* chroma pred DC */
    cabac_decision(c, 60, 0);                       /* mb_qp_delta 0 */

    /* Intra16x16DCLevel: coded_block_flag from the neighbours' */
    ca = !a || a->type == MB_PCM || (a->type == MB_I16 && a->cbf_dc);
    cb = !b || b->type == MB_PCM || (b->type == MB_I16 && b->cbf_dc);
    cabac_decision(c, 85 + ca + 2 * cb, mb->cbf_dc);
    if (mb->cbf_dc) {
        cabac_residual(c, scan, 16, 0);
    }
    if (mb->cbpc) {
        for (comp = 0; comp < 2; comp++) {
            ca = !a || a->type == MB_PCM ||
                 (a->type == MB_I16 && a->cbpc && a->cbf_c[comp]);
            cb = !b || b->type == MB_PCM ||
                 (b->type == MB_I16 && b->cbpc && b->cbf_c[comp]);
            cabac_decision(c, 85 + 12 + ca + 2 * cb, mb->cbf_c[comp]);
            if (mb->cbf_c[comp]) {
                cabac_residual(c, clev[comp], 4, 3);
            }
        }
    }
    mb->type = MB_I16;
}

static bool h264_mb_changed(const AwH264Pic *t, const AwH264Pic *r,
                            unsigned mx, unsigned my)
{
    unsigned bw = r->mbw * 4, cw = r->mbw * 2, i;

    for (i = 0; i < 16; i++) {
        unsigned o = (my * 4 + i / 4) * bw + mx * 4 + i % 4;

        if (t->y[o] != r->y[o]) {
            return true;
        }
    }
    for (i = 0; i < 4; i++) {
        unsigned o = (my * 2 + i / 2) * cw + mx * 2 + i % 2;

        if (t->u[o] != r->u[o] || t->v[o] != r->v[o]) {
            return true;
        }
    }
    return false;
}

/* An I_PCM macroblock in a P slice: the target, exactly. */
static void h264_code_pcm(AwCabac *c, const AwH264Pic *t, AwH264Pic *r,
                          AwH264Mb *mb, unsigned mx, unsigned my)
{
    unsigned bw = r->mbw * 4, cw = r->mbw * 2, x, y;

    cabac_decision(c, 14, 1);               /* P mb_type prefix: intra */
    cabac_decision(c, 17, 1);               /* not I_NxN */
    cabac_terminate(c, 1);                  /* I_PCM, flush, align */
    for (y = 0; y < 16; y++) {
        for (x = 0; x < 16; x++) {
            uint8_t v = t->y[(my * 4 + y / 4) * bw + mx * 4 + x / 4];

            g_byte_array_append(c->out, &v, 1);
        }
    }
    for (x = 0; x < 2; x++) {
        const uint8_t *pl = x ? t->v : t->u;
        unsigned cy, cx;

        for (cy = 0; cy < 8; cy++) {
            for (cx = 0; cx < 8; cx++) {
                uint8_t v = pl[(my * 2 + cy / 4) * cw + mx * 2 + cx / 4];

                g_byte_array_append(c->out, &v, 1);
            }
        }
    }
    cabac_start(c);
    for (y = 0; y < 4; y++) {
        memcpy(&r->y[(my * 4 + y) * bw + mx * 4],
               &t->y[(my * 4 + y) * bw + mx * 4], 4);
    }
    for (y = 0; y < 2; y++) {
        memcpy(&r->u[(my * 2 + y) * cw + mx * 2],
               &t->u[(my * 2 + y) * cw + mx * 2], 2);
        memcpy(&r->v[(my * 2 + y) * cw + mx * 2],
               &t->v[(my * 2 + y) * cw + mx * 2], 2);
    }
    mb->type = MB_PCM;
}

/*
 * The slice data of a whole-frame slice, after cabac_alignment_one_bit:
 * input @t, the previous frame's input @prev and the reconstruction @r
 * (the reference for a P slice), both updated.
 */
static GByteArray *aw_h264_slice_data(const AwH264Pic *t, AwH264Pic *prev,
                                      AwH264Pic *r, bool p_slice, int qp,
                                      int idc)
{
    AwCabac c;
    unsigned n = r->mbw * r->mbh, i;
    AwH264Mb *mbs = g_new0(AwH264Mb, n);

    memset(&c, 0, sizeof(c));
    c.out = g_byte_array_new();
    qp = MIN(MAX(qp, 0), 51);
    cabac_init_contexts(&c, p_slice, idc, qp);
    cabac_start(&c);
    for (i = 0; i < n; i++) {
        unsigned mx = i % r->mbw, my = i / r->mbw;

        if (p_slice) {
            bool a = mx && mbs[i - 1].type != MB_SKIP;
            bool b = my && mbs[i - r->mbw].type != MB_SKIP;
            bool skip = !h264_mb_changed(t, prev, mx, my);

            cabac_decision(&c, 11 + a + b, skip);
            if (skip) {
                mbs[i].type = MB_SKIP;
            } else {
                h264_code_pcm(&c, t, r, &mbs[i], mx, my);
            }
        } else {
            h264_code_i16(&c, t, r, mbs, mx, my, qp);
        }
        cabac_terminate(&c, i == n - 1);        /* end_of_slice_flag */
    }
    g_free(mbs);
    memcpy(prev->y, t->y, n * 16);
    memcpy(prev->u, t->u, n * 4);
    memcpy(prev->v, t->v, n * 4);
    return c.out;
}
/* --- H.264 slice coder (end) --- */

/*
 * A stream's coding state: the reference (the reconstruction) and the
 * last input, kept per bitstream buffer, which is a stream's.
 */
typedef struct AwVeStream {
    uint32_t vle_addr;
    AwH264Pic recon, prev;
} AwVeStream;

static AwVeStream *aw_ve_stream(AwVeState *s, unsigned mbw, unsigned mbh)
{
    uint32_t addr = s->regs[AVC_VLE_ADDR >> 2];
    AwVeStream *st = NULL;
    unsigned i;

    for (i = 0; i < AW_VE_STREAMS; i++) {
        if (s->streams[i] && s->streams[i]->vle_addr == addr) {
            st = s->streams[i];
            break;
        }
    }
    if (!st) {
        /* a new stream: in a free slot, or in place of the oldest */
        i = s->next_stream++ % AW_VE_STREAMS;
        if (!s->streams[i]) {
            s->streams[i] = g_new0(AwVeStream, 1);
        }
        st = s->streams[i];
        st->vle_addr = addr;
        aw_h264_pic_free(&st->recon);
        aw_h264_pic_free(&st->prev);
        st->recon.mbw = st->recon.mbh = 0;
    }
    if (st->recon.mbw != mbw || st->recon.mbh != mbh) {
        aw_h264_pic_free(&st->recon);
        aw_h264_pic_free(&st->prev);
        aw_h264_pic_alloc(&st->recon, mbw, mbh);
        aw_h264_pic_alloc(&st->prev, mbw, mbh);
    }
    return st;
}

/* Copy a plane of block values, repeating the edges to fill @dst. */
static void aw_ve_blocks(uint8_t *dst, unsigned dw, unsigned dh,
                         const uint8_t *src, unsigned sw, unsigned sh)
{
    unsigned x, y;

    for (y = 0; y < dh; y++) {
        const uint8_t *row = src + MIN(y, sh - 1) * sw;

        for (x = 0; x < dw; x++) {
            dst[y * dw + x] = row[MIN(x, sw - 1)];
        }
    }
}

/*
 * The input picture, from the video input's LBC buffer (aw-qlbc.h) at
 * 0xA78; without one there, mid-grey.
 */
static void aw_ve_input(AwVeState *s, AwH264Pic *t)
{
    uint32_t addr = s->regs[AVC_INPUT_Y >> 2];
    AwQlbcHeader hd;
    unsigned bw, bh, cw, ch;
    uint8_t *buf;

    aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE, addr, &hd, sizeof(hd));
    bw = le16_to_cpu(hd.bw);
    bh = le16_to_cpu(hd.bh);
    cw = le16_to_cpu(hd.cw);
    ch = le16_to_cpu(hd.ch);
    if (le32_to_cpu(hd.magic) != AW_QLBC_MAGIC || !bw || !bh || !cw || !ch ||
        cw * 2 < bw || ch * 2 < bh ||
        bw > AW_VE_MAX_MBS * 4 || bh > AW_VE_MAX_MBS * 4) {
        memset(t->y, 128, t->mbw * t->mbh * 16);
        memset(t->u, 128, t->mbw * t->mbh * 4);
        memset(t->v, 128, t->mbw * t->mbh * 4);
        return;
    }
    buf = g_malloc(bw * bh + 2 * cw * ch);
    aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE, addr + sizeof(hd), buf,
                      bw * bh + 2 * cw * ch);
    aw_ve_blocks(t->y, t->mbw * 4, t->mbh * 4, buf, bw, bh);
    aw_ve_blocks(t->u, t->mbw * 2, t->mbh * 2, buf + bw * bh, cw, ch);
    aw_ve_blocks(t->v, t->mbw * 2, t->mbh * 2, buf + bw * bh + cw * ch,
                 cw, ch);
    g_free(buf);
}

static void aw_h264_pic_copy(AwH264Pic *dst, const AwH264Pic *src)
{
    unsigned n = src->mbw * src->mbh;

    aw_h264_pic_alloc(dst, src->mbw, src->mbh);
    memcpy(dst->y, src->y, n * 16);
    memcpy(dst->u, src->u, n * 4);
    memcpy(dst->v, src->v, n * 4);
}

static void aw_h264_pic_move(AwH264Pic *dst, AwH264Pic *src)
{
    aw_h264_pic_free(dst);
    *dst = *src;
    src->y = src->u = src->v = NULL;
}

/* The bytes @data takes with emulation prevention, after @zeros zeros. */
static uint32_t aw_ve_escaped_size(GByteArray *data, int zeros)
{
    uint32_t i, n = data->len;

    for (i = 0; i < data->len; i++) {
        if (zeros >= 2 && data->data[i] <= 3) {
            n++;
            zeros = 0;
        }
        zeros = data->data[i] ? 0 : zeros + 1;
    }
    return n;
}

static void aw_ve_encode(AwVeState *s)
{
    uint32_t dim = s->regs[0xA00 >> 2];
    unsigned w = (((dim >> 16) & 0xFFFF) * 8 + 15) / 16;
    unsigned h = ((dim & 0xFFFF) * 8 + 15) / 16;
    /* AVC_PARAM: bit 4 P slice, bits 11:10 cabac_init_idc (1 in the
     * firmware's P slice headers, with 0x400 here), bit 8 CABAC */
    bool p_slice = s->regs[0xB04 >> 2] & 0x10;
    int idc = (s->regs[0xB04 >> 2] >> 10) & 3;
    int qp = s->regs[0xB08 >> 2] & 0x3F;
    uint32_t ring = (s->regs[AVC_VLE_END >> 2] + 1 -
                     s->regs[AVC_VLE_ADDR >> 2]) << 8;
    uint32_t pos, i, bytes;
    GByteArray *data;
    AwVeStream *st;
    AwH264Pic in, ref = { 0 }, last = { 0 };
    uint8_t prev[2] = { 0xFF, 0xFF };
    int zeros;

    if (!w || !h || w > AW_VE_MAX_MBS || h > AW_VE_MAX_MBS) {
        qemu_log_mask(LOG_GUEST_ERROR, "aw-ve: picture %ux%u MBs\n", w, h);
        return;
    }

    /* cabac_alignment_one_bit */
    while (s->regs[AVC_VLE_LENGTH >> 2] & 7) {
        aw_ve_put_bits(s, 1, 1);
    }
    pos = s->regs[AVC_VLE_LENGTH >> 2];
    if (pos >= 16) {
        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE,
                          aw_ve_vle_byte(s, pos - 16), &prev[0], 1);
        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE,
                          aw_ve_vle_byte(s, pos - 8), &prev[1], 1);
    }
    zeros = (prev[1] == 0) ? (prev[0] == 0 ? 2 : 1) : 0;

    st = aw_ve_stream(s, w, h);
    aw_h264_pic_alloc(&in, w, h);
    aw_ve_input(s, &in);
    if (p_slice) {
        aw_h264_pic_copy(&ref, &st->recon);
        aw_h264_pic_copy(&last, &st->prev);
    }
    data = aw_h264_slice_data(&in, &st->prev, &st->recon, p_slice, qp,
                              MIN(idc, 2));
    bytes = aw_ve_escaped_size(data, zeros);
    if (p_slice && ring &&
        (pos >> 3) - (s->regs[AVC_VLE_OFFSET >> 2] >> 3) + bytes > ring) {
        /*
         * The frame would wrap over itself in the bitstream buffer: send
         * it with every macroblock skipped instead, and the changes with
         * a later frame.
         */
        g_byte_array_free(data, true);
        aw_h264_pic_move(&st->recon, &ref);
        aw_h264_pic_move(&st->prev, &last);
        aw_h264_pic_copy(&ref, &st->prev);
        data = aw_h264_slice_data(&ref, &st->prev, &st->recon, true, qp,
                                  MIN(idc, 2));
        aw_h264_pic_free(&ref);
    } else if (p_slice) {
        aw_h264_pic_free(&ref);
        aw_h264_pic_free(&last);
    }
    aw_h264_pic_free(&in);
    if (ring && (pos >> 3) - (s->regs[AVC_VLE_OFFSET >> 2] >> 3) +
        aw_ve_escaped_size(data, zeros) > ring) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "aw-ve: frame larger than the bitstream buffer\n");
    }
    /* emulation prevention, continuing from the header's last bytes */
    for (i = 0; i < data->len; i++) {
        uint8_t b = data->data[i];

        if (zeros >= 2 && b <= 3) {
            uint8_t epb = 3;

            aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_VE,
                               aw_ve_vle_byte(s, pos), &epb, 1);
            pos += 8;
            zeros = 0;
        }
        aw_iommu_dma_write(s->iommu, AW_IOMMU_MASTER_VE,
                           aw_ve_vle_byte(s, pos), &b, 1);
        pos += 8;
        zeros = b ? 0 : zeros + 1;
    }
    s->regs[AVC_VLE_LENGTH >> 2] = pos;
    if (s->trace) {
        qemu_log("aw-ve: encoded %s slice %ux%u MBs qp %d: %u bytes\n",
                 p_slice ? "P" : "I", w, h, qp, data->len);
    }
    g_byte_array_free(data, true);
}

/* With "dump": the frame just encoded, as the driver takes it. */
static void aw_ve_dump_frame(AwVeState *s)
{
    uint32_t from = s->regs[AVC_VLE_OFFSET >> 2];
    uint32_t to = s->regs[AVC_VLE_LENGTH >> 2];
    g_autofree char *path = NULL;
    uint32_t pos;
    FILE *f;

    if (!s->dump || to <= from) {
        return;
    }
    path = g_strdup_printf("%s/avc-%08x.h264", s->dump,
                           s->regs[AVC_VLE_ADDR >> 2] << 8);
    f = fopen(path, "ab");
    if (!f) {
        return;
    }
    for (pos = from & ~7u; pos < to; pos += 8) {
        uint8_t b;

        aw_iommu_dma_read(s->iommu, AW_IOMMU_MASTER_VE,
                          aw_ve_vle_byte(s, pos), &b, 1);
        fputc(b, f);
    }
    fclose(f);
}

static void aw_ve_update_irq(AwVeState *s)
{
    qemu_set_irq(s->irq, (s->regs[VE_MODE >> 2] & VE_MODE_AVC_ENC) &&
                         (s->regs[AVC_STATUS >> 2] & 0xF) &&
                         (s->regs[AVC_INT_EN >> 2] & 7));
}

static void aw_ve_avc_trigger(AwVeState *s, uint32_t val)
{
    switch (val & 0xF) {
    case AVC_CMD_PUT_BITS:
        if (s->trace) {
            qemu_log("aw-ve: put %u bits 0x%x\n", (val >> 8) & 0x3F,
                     s->regs[AVC_BASIC_BITS >> 2]);
        }
        aw_ve_put_bits(s, s->regs[AVC_BASIC_BITS >> 2], (val >> 8) & 0x3F);
        break;
    case AVC_CMD_ENCODE:
        aw_ve_encode(s);
        aw_ve_dump_frame(s);
        s->regs[AVC_STATUS >> 2] |= AVC_STATUS_DONE;
        aw_ve_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "aw-ve: AVC command 0x%08x\n", val);
        break;
    }
}

static uint64_t aw_ve_read(void *opaque, hwaddr addr, unsigned size)
{
    AwVeState *s = opaque;
    uint32_t v = s->regs[addr >> 2];

    switch (addr) {
    case AW_VE_BASE_ID:
        v = s->ver_e0;
        break;
    case AW_VE_IC_ID:
        v = s->ver_e4;
        break;
    case AW_VE_VERSION:
        v = s->ver_f0;
        break;
    case AVC_STATUS:
        v |= AVC_STATUS_READY;
        break;
    case 0x004:
        v |= 0xE00;     /* memory sync idle, per engine (veInquireIdle) */
        break;
    }
    if (s->trace) {
        qemu_log("aw-ve: read  0x%03x -> 0x%08x\n", (unsigned)addr, v);
    }
    return v;
}

static void aw_ve_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    AwVeState *s = opaque;

    if (s->trace) {
        qemu_log("aw-ve: write 0x%03x <- 0x%08x\n", (unsigned)addr,
                 (uint32_t)val);
    }
    if (addr == AVC_STATUS) {
        s->regs[addr >> 2] &= ~(val & 0xF);
        aw_ve_update_irq(s);
        return;
    }
    s->regs[addr >> 2] = val;
    if (addr == AVC_VLE_OFFSET) {
        s->regs[AVC_VLE_LENGTH >> 2] = val;
    }
    if (addr == AVC_TRIGGER) {
        aw_ve_avc_trigger(s, val);
    } else if (addr == AVC_INT_EN || addr == VE_MODE) {
        aw_ve_update_irq(s);
    }
}

static const MemoryRegionOps aw_ve_ops = {
    .read = aw_ve_read,
    .write = aw_ve_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void aw_ve_reset_hold(Object *obj, ResetType type)
{
    AwVeState *s = AW_VE(obj);
    unsigned i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < AW_VE_STREAMS; i++) {
        if (s->streams[i]) {
            aw_h264_pic_free(&s->streams[i]->recon);
            aw_h264_pic_free(&s->streams[i]->prev);
            g_free(s->streams[i]);
            s->streams[i] = NULL;
        }
    }
}

static void aw_ve_init(Object *obj)
{
    AwVeState *s = AW_VE(obj);

    memory_region_init_io(&s->iomem, obj, &aw_ve_ops, s, TYPE_AW_VE,
                          AW_VE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property aw_ve_properties[] = {
    DEFINE_PROP_UINT32("ver-f0", AwVeState, ver_f0, 0),
    DEFINE_PROP_UINT32("ver-e0", AwVeState, ver_e0, 0x21110),
    DEFINE_PROP_UINT32("ver-e4", AwVeState, ver_e4, 0x21210),
    DEFINE_PROP_BOOL("trace", AwVeState, trace, false),
    DEFINE_PROP_STRING("dump", AwVeState, dump),
    DEFINE_PROP_LINK("iommu", AwVeState, iommu, TYPE_DEVICE, DeviceState *),
};

static void aw_ve_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = aw_ve_reset_hold;
    device_class_set_props(dc, aw_ve_properties);
}

static const TypeInfo aw_ve_info = {
    .name           = TYPE_AW_VE,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(AwVeState),
    .instance_init  = aw_ve_init,
    .class_init     = aw_ve_class_init,
};

static void aw_ve_register_types(void)
{
    type_register_static(&aw_ve_info);
}

type_init(aw_ve_register_types)
