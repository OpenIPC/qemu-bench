/*
 * Ingenic T31 / T23 / T21 image pipeline: MIPI CSI-2 host and D-PHY, VIC
 * (the video input controller) and the ISP core with its MSCA scaler /
 * DMA-out channels -- the part of the camera the vendor tx-isp-<soc>.ko
 * drives.  (T20 and T30 are an older, Apical-based design.)
 *
 * Read off the vendor modules: tx-isp-t31.ko H20221206a of the OpenIPC
 * rootfs and the T31 SDK 1.1.5 build of the same driver with its local
 * symbols (tmp/media-contract.md section 2 has the offsets with their code
 * addresses); tx-isp-t23.ko, from the T23 SDK 1.1.0 build with symbols and
 * the OpenIPC H20240204a; tx-isp-t21.ko H20200509a.  There is no sensor
 * data: each frame the model paints a test picture into the buffers the
 * driver queued.  Register windows (T31 offsets; ingenic_isp_layouts[]
 * has where the others differ):
 *
 *   ISP core  0x13300000, 0x90000 (the module maps 64 KiB, the tuning code
 *             reaches LUT windows above; T21/T23 64 KiB): storage, except
 *     0x0B0   interrupt enable (~0 at stream on, 0 at stream off)
 *     0x0B4   interrupt status: bit n channel n frame done (n = 0..2),
 *             0x1000 frame start (schedules the sensor/AE work)
 *     0x0B8   write the bits read from 0x0B4 to clear them; IRQ 37
 *             (INTC 29) is high while status & enable
 *     0x9804  MSCA channel enable, bit n channel n
 *     C = 0x9900 + 0x100 n, channel n = 0..2:
 *     C+0x00  resize output size, w << 16 | h
 *     C+0x2C  crop size, w << 16 | h: the size the channel writes
 *     C+0x6C  push the Y address of an empty buffer (QBUF)
 *     C+0x84  push its UV address (Y + ALIGN(h, 16) * w for NV12)
 *     C+0x74  pop the Y address of a filled buffer (read side effect)
 *     C+0x7C  bit 0: the done FIFO is empty (the ISR pops until it is)
 *     C+0x80 / C+0x98  Y / UV line strides
 *     C+0x8C / C+0x90  the popped entry's UV address and frame number
 *     C+0x9C..C+0xA8   any write clears the channel's FIFOs
 *   VIC       0x133E0000, 64 KiB: storage, except
 *     0x000   2 and 4 (reset / configure) read back 0 at once -- the
 *             driver spins on 0 with no timeout; 1 starts the input
 *     0x1E0 / 0x1E4  status (bit 0 of 0x1E0: frame done)
 *     0x1E8 / 0x1EC  masks, 1 = masked; IRQ 38 (INTC 30) is high while a
 *                    status bit is unmasked
 *     0x1F0 / 0x1F4  write 1 to clear
 *   (T41: the MSCA channels in their own window, 0x133F0000, mmio 4)
 *   (T40: a second pipe, VIC1 / VIC2 at 0x13390000 / 0x133A0000, mmio 5
 *   and 6, IRQs 2 and 3 -- ingenic_isp_layouts[]'s pipe_* fields)
 *   CSI host  0x10023000 and D-PHY 0x10022000, 4 KiB each: storage (the
 *             driver only writes them, and reads ERR1/ERR2 as 0); T21,
 *             DVP only, has neither
 *
 * The frame clock runs at "fps" while the VIC is started and the ISP's
 * interrupt enable is not zero.  Each frame: frame start; for each
 * enabled channel with a queued buffer, the picture at the channel's size
 * into that buffer, the addresses into the done FIFO and status bit n;
 * VIC frame done.  With no queued buffer the channel drops the frame, as
 * the hardware does.
 *
 * The picture is hw/misc/sstar-pattern.h's scene (the OpenIPC logo on a
 * navy gradient with a dot that slides beneath it), scaled to the
 * channel, plus a frame counter in the top left corner, written NV12.
 * The scene is lit as the sensor's exposure says (the "sensor" link, its
 * rows x gain, times "light"; ingenic_isp_gain()), and each frame the
 * Tiziano AE and AWB statistics of it go to the kernel's banks (T31;
 * ingenic_isp_stats()), so the kernel's AE closes its loop through the
 * sensor's I2C registers and the picture brightens or darkens with it.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "ingenic-isp-paint.h"
#include "hw/i2c/hisi-soi.h"
#include "hw/i2c/ingenic-sensor.h"
#include "qom/object.h"

#define TYPE_INGENIC_ISP "ingenic-isp"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIspState, INGENIC_ISP)

#define ISP_SIZE        0x100000    /* the register file: T41's reaches 0xF0000 */
#define MSCA_WIN_SIZE   0x10000
#define VIC_SIZE        0x10000
#define CSI_SIZE        0x1000
#define PHY_SIZE        0x1000

#define MSCA_CH(n)      (s->l->ch_base + 0x100 * (n))

#define VIC_CTRL        0x000

#define NUM_CH          3           /* a pipe's */
#define MAX_PIPES       2
#define NUM_VIC         3
#define FIFO_DEPTH      16
#define IVDC_Y          0x80000000u   /* direct_mode's channel-0 pair (T41, T23) */

/*
 * What differs between the SoCs.  Every block keeps the T31 meaning of
 * its registers; only where they are, and which exist, changes.
 */
typedef struct IngenicIspLayout {
    const char *name;
    uint32_t isp_size;          /* the core window */
    hwaddr msca_win;            /* core offset of a separate window (T41's
                                   scaler past the VIC), mmio 4; 0: none */
    hwaddr int_en, int_sta, int_clr;
    bool int_mask;              /* int_en is a mask: 1 = masked */
    bool no_vic;                /* a scaler on its own: it runs while a
                                   channel is enabled */
    uint32_t fs_bit;            /* frame start in int_sta, 0: none */
    hwaddr ch_base;             /* channel n's registers at +0x100 n */
    hwaddr ch_en;               /* bit n: channel n enabled */
    hwaddr ch_busy;             /* channel busy bits: read 0; 0: none */
    hwaddr ch_active;           /* reads ch_en's channel bits; 0: none */
    /* in a channel's block; 0 (but osize): none */
    uint16_t osize, csize;      /* scaler output, crop: w << 16 | h */
    uint16_t push_y, push_uv;   /* an empty buffer's addresses */
    uint16_t pop_y, pop_uv;     /* a filled one's: popped by reading pop_y */
    uint16_t pop_num[2];        /* its frame number */
    uint16_t empty;             /* bit 0: the done FIFO is empty */
    uint16_t full;              /* bit 1: the empty-buffer FIFO is full */
    uint16_t ystride, uvstride;
    uint16_t clear[4];          /* a write empties the FIFOs */
    hwaddr vic_sta, vic_mask, vic_clr;
    bool vic_sta2;              /* a second status/mask/ack at +4 */
    uint32_t vic_fs_bit;        /* frame start in vic_sta, 0: none */
    hwaddr req, ack;            /* bit 0 of ack follows bit 0 of req; 0: none */
    /* the Tiziano 3A statistics blocks (AE0, AWB); 0: none */
    hwaddr ae, awb;
    uint8_t ae_static_bit, ae_hist_bit, awb_bit;    /* in int_sta */
    uint8_t stat_gen;           /* 1: T31's bank formats; 2: T40/T41's;
                                   3: T21's */
    hwaddr wb;                  /* the AWB's R and B gains (G << 16 | gain,
                                   Q10), for the trace; 0: none */
    uint8_t xstat;              /* the other Tiziano statistics (AE1, AF,
                                   ADR, defog; ingenic_isp_xstats()): 1
                                   T31's, with WDR; 2 T23's, without;
                                   3 T40/T41's formats */
    /*
     * T40's second pipe (pipes 2): its channels 3..5, their enable and
     * FIFOs at pipe_ch past pipe 0's, its interrupt enable / clear /
     * status at pipe_int past pipe 0's, its AE and AWB blocks at pipe_stat
     * past pipe 0's; fed by VIC1 (mmio 5, its own IRQ), VIC2 (mmio 6)
     * storage with an IRQ of its own
     */
    uint8_t pipes;
    hwaddr pipe_ch, pipe_int, pipe_stat;
    uint8_t ivdc_bit;           /* T41's IVDC: frame start of a channel
                                   writing to the IVDC, in int_sta; 0: none */
    /*
     * a daylight white's raw R and B, per mille of G, where the tuning's
     * AWB looks for it: the default light colour; 0: 550 / 600
     */
    uint16_t day_r, day_b;
    bool hldc;                  /* T41: the HLDC lens correction stage */
} IngenicIspLayout;

static const IngenicIspLayout ingenic_isp_layouts[] = {
    {
        /* tx-isp-t31.ko H20221206a (tmp/media-contract.md section 2) */
        .name = "t31", .isp_size = 0x90000,
        .int_en = 0xB0, .int_sta = 0xB4, .int_clr = 0xB8, .fs_bit = 0x1000,
        .ch_base = 0x9900, .ch_en = 0x9804,
        .osize = 0x00, .csize = 0x2C, .push_y = 0x6C, .push_uv = 0x84,
        .pop_y = 0x74, .pop_uv = 0x8C, .pop_num = { 0x90 }, .empty = 0x7C,
        .ystride = 0x80, .uvstride = 0x98,
        .clear = { 0x9C, 0xA0, 0xA4, 0xA8 },
        .vic_sta = 0x1E0, .vic_mask = 0x1E8, .vic_clr = 0x1F0,
        .vic_sta2 = true,
        /* ae0_interrupt_static / _hist, awb_interrupt_static (SDK .ko) */
        .ae = 0xA000, .awb = 0xB000,
        .ae_static_bit = 26, .ae_hist_bit = 27, .awb_bit = 30, .stat_gen = 1,
        .wb = 0x1804, .xstat = 1,
    }, {
        /*
         * tx-isp-t21.ko H20200509a: the T31 MSCA at 0x2300 (QBUF 0x32650,
         * ISR 0x3280c: one pop per interrupt, no frame-start bit), a
         * channel-stop poll on 0x2308, the VIC's status/mask/ack at
         * 0x100/0x104/0x108 (ISR 0xe5c); DVP only, no CSI.
         */
        .name = "t21", .isp_size = 0x10000,
        .int_en = 0x80, .int_sta = 0x84, .int_clr = 0x88,
        .ch_base = 0x2400, .ch_en = 0x2304, .ch_busy = 0x2308,
        .osize = 0x00, .csize = 0x2C, .push_y = 0x6C, .push_uv = 0x84,
        .pop_y = 0x74, .pop_uv = 0x8C, .pop_num = { 0x78, 0x90 },
        .empty = 0x7C, .ystride = 0x80, .uvstride = 0x98,
        .vic_sta = 0x100, .vic_mask = 0x104, .vic_clr = 0x108,
        /*
         * the T31 3A blocks at 0x700 (AE0) and 0x800 (AWB), banks set by
         * tisp_init 0xe2ec; ae_interrupt_static / _hist and
         * awb_interrupt_static on int_sta bits 16, 17 and 18
         * (tiziano_ae_init 0x2d298, tiziano_awb_init 0x10a24)
         */
        .ae = 0x700, .awb = 0x800,
        .ae_static_bit = 16, .ae_hist_bit = 17, .awb_bit = 18, .stat_gen = 3,
        .wb = 0x604,                    /* JZ_Isp_Awb_Awbg2reg */
    }, {
        /*
         * tx-isp-t23.ko (SDK 1.1.0 build H20230720a with its symbols,
         * OpenIPC H20240204a): the T31 blocks, the MSCA channels moved to
         * 0xD100 + 0x100 n with their own offsets (QBUF SDK 0x6449c, ISR
         * 0x65f70, tisp_channel_main_fifo_clear 0x15530,
         * tisp_msca_init_chx_cfg 0x16f2c).  The pop's UV word carries a
         * tag in bits 2:0, 0 here.
         */
        .name = "t23", .isp_size = 0x10000,
        .int_en = 0xB0, .int_sta = 0xB4, .int_clr = 0xB8, .fs_bit = 0x1000,
        .ch_base = 0xD100, .ch_en = 0xD040,
        .osize = 0x04, .csize = 0x0C, .push_y = 0x30, .push_uv = 0x50,
        .pop_y = 0x38, .pop_uv = 0x58, .empty = 0x3C,
        .ystride = 0x40, .uvstride = 0x60,
        .clear = { 0x44, 0x64, 0x48, 0x68 },
        .vic_sta = 0x1E0, .vic_mask = 0x1E8, .vic_clr = 0x1F0,
        .vic_sta2 = true,
        /* the T31 3A blocks and bits (system_irq_func_set callers) */
        .ae = 0xA000, .awb = 0xB000,
        .ae_static_bit = 26, .ae_hist_bit = 27, .awb_bit = 30, .stat_gen = 1,
        .wb = 0x1804,                   /* Tiziano_awb_set_gain */
        .xstat = 2,
    }, {
        /*
         * T30's scaler (isp-m3, 0x130B0000, IRQ 27 = INTC 19): the T31
         * MSCA channel layout from 0x100, on its own interrupt (status
         * 0x0C, mask 0x10, write 0x14 to clear; ISR tx-isp-t30.ko
         * 0x152a0), fed by the Apical core through the VIC's line buffer
         * (ingenic-isp-apical.c).  QBUF stops at C+0x70 bit 1 (full),
         * and 0x08 reads which channels run.
         */
        .name = "t30", .isp_size = 0x10000,
        .int_en = 0x10, .int_sta = 0x0C, .int_clr = 0x14, .int_mask = true,
        .no_vic = true,
        .ch_base = 0x100, .ch_en = 0x04, .ch_active = 0x08,
        .osize = 0x00, .csize = 0x2C, .push_y = 0x6C, .push_uv = 0x84,
        .pop_y = 0x74, .pop_uv = 0x8C, .pop_num = { 0x78, 0x90 },
        .empty = 0x7C, .full = 0x70, .ystride = 0x80, .uvstride = 0x98,
        .clear = { 0x9C, 0xA0 },
    }, {
        /*
         * tx-isp-t40.ko H20220606a (the T40 SDK 1.2.0 build with symbols,
         * the same constants in OpenIPC's): pipe 0's interrupts at
         * 0x40020 (enable) / 0x40028 (status) / 0x40024 (clear; ISR
         * 0x72f10), the MSCA channels at 0x16100 + 0x100 n with the T31
         * offsets (Tzn_Msca_addr_fifo_write 0x667b4, fifo_read 0x66830),
         * enable 0x16008; VIC0 the T31 one but for its frame start, bit 4
         * of 0x1E0 (main_fs_work, VIC ISR 0x3e2c); no polls.
         */
        .name = "t40", .isp_size = 0x80000,
        .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
        .ch_base = 0x16100, .ch_en = 0x16008,
        .osize = 0x00, .csize = 0x2C, .push_y = 0x6C, .push_uv = 0x84,
        .pop_y = 0x74, .pop_uv = 0x8C, .empty = 0x7C,
        .ystride = 0x80, .uvstride = 0x98,
        .clear = { 0x9C, 0xA0, 0xA4, 0xA8 },
        .vic_sta = 0x1E0, .vic_mask = 0x1E8, .vic_clr = 0x1F0,
        .vic_sta2 = true, .vic_fs_bit = 0x10,
        /* pipe 0's 3A as on T41 (tisp_ae_main_interrupt_static & co.) */
        .ae = 0x19000, .awb = 0x18000,
        .ae_static_bit = 4, .ae_hist_bit = 5, .awb_bit = 3, .stat_gen = 2,
        .wb = 0x4004, .day_r = 460, .day_b = 550, .xstat = 3,
        /*
         * pipe 1 (the second sensor, IMPVI_SEC: tisp_sec_init,
         * tisp_channel_sec_*, tisp_ae_sec_*): ISR 0x72f10 reads its
         * status at 0x40038 and clears it through 0x40034, the T31 bits
         * again (channels 3..5 on bits 0..2, 3A on 3..5); its MSCA
         * channels at 0x17100 + 0x100 n, enable 0x17008; AE 0x19200, AWB
         * 0x18200 (banks from tisp_sec_init).  VIC1 is VIC0 again at
         * +0x10000, its frame start queueing sec_fs_work (VIC ISR 0x4708)
         */
        .pipes = 2, .pipe_ch = 0x1000, .pipe_int = 0x10, .pipe_stat = 0x200,
    }, {
        /*
         * tx-isp-t41.ko H20230630a (T41 SDK 1.2.0): T40's core interrupts
         * (isp_interrupts_enable 0x6b88c, ISR 0x6d3f0) and VIC, but one
         * pipe and one VIC, and the scaler moved to core + 0xF0000 --
         * 0x133F0000, past the VIC, its own window here -- with the same
         * channel offsets from 0xF0100 (tisp_msca_addr_fifo_write
         * 0x5e354, fifo_read 0x5e3b8) and the enable at 0xF0008
         * (tisp_msca_chx_cfg_load 0x5d72c).
         */
        .name = "t41", .isp_size = 0x80000, .msca_win = 0xF0000, .hldc = true,
        .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
        .ch_base = 0xF0100, .ch_en = 0xF0008,
        .osize = 0x00, .csize = 0x2C, .push_y = 0x6C, .push_uv = 0x84,
        .pop_y = 0x74, .pop_uv = 0x8C, .empty = 0x7C,
        .ystride = 0x80, .uvstride = 0x98,
        .clear = { 0x9C, 0xA0, 0xA4, 0xA8 },
        .vic_sta = 0x1E0, .vic_mask = 0x1E8, .vic_clr = 0x1F0,
        .vic_sta2 = true, .vic_fs_bit = 0x10,
        /*
         * ispcore_video_s_stream and ispcore_suspend_module set 0x200 bit
         * 0 and spin, without a timeout, until 0x204 bit 0 is set
         */
        .req = 0x200, .ack = 0x204,
        /*
         * the T31 3A blocks at 0x19000 (AE0) and 0x18000 (AWB), their
         * callbacks on int_sta bits 4, 5 and 3 (irq_func_cb[pipe * 42 +
         * n], tisp_ae_init, tisp_awb_init), wider fields
         */
        .ae = 0x19000, .awb = 0x18000,
        .ae_static_bit = 4, .ae_hist_bit = 5, .awb_bit = 3, .stat_gen = 2,
        .wb = 0x4004, .day_r = 460, .day_b = 550, .xstat = 3,
        /*
         * direct_mode: channel 0 is queued IVDC_Y / IVDC_UV; the ISR sends
         * the frame source its "buffer done" on int_sta bit 20 (the
         * MSCA's first line; bit 13, the vertical blank, with ivdc_vb_type
         * 1) and on bit 0 queues the pair again instead of reading the
         * done FIFO (ispcore_interrupt_service_routine 0x6d4f4, 0x6d66c)
         */
        .ivdc_bit = 20,
    },
};

typedef struct IngenicIspFifo {
    uint32_t a[FIFO_DEPTH];
    uint32_t b[FIFO_DEPTH];     /* the done FIFO's frame numbers */
    uint32_t head, count;
} IngenicIspFifo;

typedef struct IngenicIspChan {
    IngenicIspFifo y, uv;       /* empty buffers, as queued */
    IngenicIspFifo done_y, done_uv;
    uint32_t pop_uv, pop_num;
    uint32_t frames;            /* frames this channel has written */
} IngenicIspChan;

struct IngenicIspState {
    SysBusDevice parent_obj;

    MemoryRegion isp_mr, vic_mr, csi_mr, phy_mr, msca_mr;
    MemoryRegion vic1_mr, vic2_mr;
    qemu_irq isp_irq, vic_irq, vic1_irq, vic2_irq;
    qemu_irq ivdc_frame;        /* a picture into the IVDC's store */
    QEMUTimer *timer;

    uint32_t fps;
    char *soc;
    bool trace;
    bool trace_regs;            /* log every register write */
    uint32_t light;             /* the scene's light, percent */
    uint32_t light_r, light_b;  /* its colour: the raw R and B of a white
                                   over its G, per mille */
    Object *sensor;             /* the image sensor, for its exposure */
    Object *sensor1;            /* T40: the second pipe's sensor */
    Object *sensor2;            /* T40: the third sensor, on VIC2 */
    const IngenicIspLayout *l;

    uint32_t isp[ISP_SIZE / 4];
    uint32_t vic[VIC_SIZE / 4];
    uint32_t vic1[VIC_SIZE / 4];    /* T40's second and third VICs */
    uint32_t vic2[VIC_SIZE / 4];
    uint32_t csi[CSI_SIZE / 4];
    uint32_t phy[PHY_SIZE / 4];
    bool vic_running;
    bool vic1_running;
    bool vic2_running;          /* T40: VIC2 (the third sensor) started */
    uint8_t mdma_slot[3];       /* each VIC's next MDMA buffer slot */
    uint32_t frame;
    IngenicIspChan ch[NUM_CH * MAX_PIPES];  /* pipe p's channel n: 3 p + n */

    IngenicIspPainter paint;
    /* the HLDC's sampling map of the last plane remapped, by its key */
    struct {
        uint32_t key[8];
        int32_t *xy;            /* source x, y per pixel, 1/256 pixel */
        uint32_t n;
    } hldc_map[2];
    uint8_t *hldc_buf;
    uint32_t hldc_buf_size;
    /*
     * T40/T41's statistics RAMs read through ports (GSM, LCE): what
     * this frame left in each, and where the next read of its data
     * register is
     */
    uint32_t ram_gsm[2][256], ram_lce[1440], ram_lce_mean[45];
    uint32_t port_pos[6];
};

/* --- FIFOs -------------------------------------------------------------- */

static bool fifo_push(IngenicIspFifo *f, uint32_t a, uint32_t b)
{
    if (f->count == FIFO_DEPTH) {
        return false;
    }
    f->a[(f->head + f->count) % FIFO_DEPTH] = a;
    f->b[(f->head + f->count) % FIFO_DEPTH] = b;
    f->count++;
    return true;
}

static uint32_t fifo_pop(IngenicIspFifo *f, uint32_t *b)
{
    uint32_t a;

    if (!f->count) {
        if (b) {
            *b = 0;
        }
        return 0;
    }
    a = f->a[f->head];
    if (b) {
        *b = f->b[f->head];
    }
    f->head = (f->head + 1) % FIFO_DEPTH;
    f->count--;
    return a;
}

static void chan_clear(IngenicIspChan *c)
{
    memset(&c->y, 0, sizeof(c->y));
    memset(&c->uv, 0, sizeof(c->uv));
    memset(&c->done_y, 0, sizeof(c->done_y));
    memset(&c->done_uv, 0, sizeof(c->done_uv));
}

/* --- interrupts, frame clock ---------------------------------------------- */

#define ISP_REG(o)      s->isp[(o) >> 2]
#define VIC_REG(o)      s->vic[(o) >> 2]

/* pipe @p's copy of a pipe 0 register: channels, interrupts, 3A */
#define PIPE_CH(p, o)   ((o) + (p) * s->l->pipe_ch)
#define PIPE_INT(p, o)  ((o) + (p) * s->l->pipe_int)
#define PIPE_STAT(p, o) ((o) + (p) * s->l->pipe_stat)

static int ingenic_isp_pipes(IngenicIspState *s)
{
    return MAX(s->l->pipes, 1);
}

/* VIC @v's registers (T40: three VICs, 64 KiB apart) */
static uint32_t *ingenic_vic_regs(IngenicIspState *s, int v)
{
    return v == 0 ? s->vic : v == 1 ? s->vic1 : s->vic2;
}

static bool ingenic_vic_pending(IngenicIspState *s, int v)
{
    const IngenicIspLayout *l = s->l;
    uint32_t *r = ingenic_vic_regs(s, v);
    uint32_t vic = r[l->vic_sta >> 2] & ~r[l->vic_mask >> 2];

    if (l->vic_sta2) {
        vic |= r[(l->vic_sta + 4) >> 2] & ~r[(l->vic_mask + 4) >> 2];
    }
    return !l->no_vic && vic;
}

static void ingenic_isp_update(IngenicIspState *s)
{
    const IngenicIspLayout *l = s->l;
    bool isp = false;
    int p;

    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        uint32_t en = ISP_REG(PIPE_INT(p, l->int_en));

        if (l->int_mask) {
            en = ~en;
        }
        isp |= !!(ISP_REG(PIPE_INT(p, l->int_sta)) & en);
    }
    qemu_set_irq(s->isp_irq, isp);
    qemu_set_irq(s->vic_irq, ingenic_vic_pending(s, 0));
    if (l->pipes > 1) {
        qemu_set_irq(s->vic1_irq, ingenic_vic_pending(s, 1));
        qemu_set_irq(s->vic2_irq, ingenic_vic_pending(s, 2));
    }
}

static int64_t ingenic_isp_period(IngenicIspState *s)
{
    return NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
}

/* Does pipe @p take frames: its VIC started, its interrupts enabled? */
static bool ingenic_isp_pipe_running(IngenicIspState *s, int p)
{
    const IngenicIspLayout *l = s->l;

    if (l->no_vic) {
        return !p && (ISP_REG(l->ch_en) & 7);
    }
    return (p ? s->vic1_running : s->vic_running) &&
           ISP_REG(PIPE_INT(p, l->int_en));
}

static void ingenic_isp_schedule(IngenicIspState *s)
{
    bool run = false;
    int p;

    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        run |= ingenic_isp_pipe_running(s, p);
    }

    if (run && !timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ingenic_isp_period(s));
    } else if (!run) {
        timer_del(s->timer);
    }
}

/* --- the scene's light, 3A statistics ------------------------------------- */

/* @sensor's exposure, rows x gain (1.0 = 1024); 0 if there is none */
static uint64_t ingenic_isp_sensor_level(Object *sensor)
{
    IngenicSensorExposure e = { 0 };
    HisiSoiExposure soi;

    if (!sensor) {
        return 0;
    }
    if (galaxycore_exposure(sensor, &e) ||
        smartsens_exposure(sensor, &e)) {
        return (uint64_t)e.lines * e.gain;
    }
    if (hisi_soi_exposure(sensor, &soi)) {
        return (uint64_t)soi.lines * soi.gain;
    }
    return 0;
}

/* The short exposure of a DOL WDR sensor, as the level above; 0: none */
static uint64_t ingenic_isp_sensor_short(Object *sensor)
{
    IngenicSensorExposure e = { 0 };

    if (sensor && galaxycore_exposure(sensor, &e)) {
        return (uint64_t)e.short_lines * e.gain;
    }
    return 0;
}

/*
 * What the picture is lit by, 1.0 = 256: "light" times the sensor's
 * exposure over EXPOSURE_1X -- the scene as painted is what 1000 rows at
 * gain 1.0 make of it.  Only where the kernel gets statistics to close its
 * AE loop with; elsewhere (and before the sensor has an exposure) "light"
 * alone.
 */
#define EXPOSURE_1X     (1000ull * 1024)

static uint32_t ingenic_isp_gain(IngenicIspState *s, int pipe)
{
    uint64_t g = (uint64_t)s->light * 256 / 100;
    uint64_t level = s->l->ae ?
        ingenic_isp_sensor_level(pipe ? s->sensor1 : s->sensor) : 0;

    if (level) {
        g = g * level / EXPOSURE_1X;
    }
    return MIN(MAX(g, 1), 256 * 64);
}

/*
 * A two-exposure (DOL) WDR mode: tisp_init's linear setup ORs 0x34000009
 * into the module bypasses at 0x0C, its WDR setup clears bit 0 (T31,
 * T23); T40 and T41 keep theirs at 0x40, bit 0 again the WDR's bypass
 * (0xffd00049 linear, 0xff9100e0 in WDR on the T41 SDK image).  The
 * short frame's light, 1.0 = 256, as ingenic_isp_gain() from the
 * sensor's short exposure; 0 outside a WDR mode or before the sensor has
 * one.
 */
#define T31_TOP         0x0C        /* module bypasses */
#define T41_TOP         0x40

static bool ingenic_isp_wdr_mode(IngenicIspState *s)
{
    if (s->l->xstat == 1 || s->l->xstat == 2) {
        return !(ISP_REG(T31_TOP) & 1);
    }
    /* T40 / T41: once tisp_init has set the bypasses at all */
    return s->l->xstat == 3 && ISP_REG(T41_TOP) && !(ISP_REG(T41_TOP) & 1);
}

static uint32_t ingenic_isp_short_gain(IngenicIspState *s, int pipe)
{
    uint64_t level, g;

    if (pipe || !ingenic_isp_wdr_mode(s)) {
        return 0;
    }
    level = ingenic_isp_sensor_short(s->sensor);
    if (!level) {
        return 0;
    }
    g = (uint64_t)s->light * 256 / 100 * level / EXPOSURE_1X;
    return MIN(MAX(g, 1), 256 * 64);
}

/*
 * The statistics the Tiziano AE and AWB take of each frame, written by DMA
 * into the banks tisp_init gives them, as the SDK's tx-isp-t31.ko reads
 * them back (offsets in the AE0 block at l->ae, AWB at l->awb):
 *
 *   +0x04          the zone grid: columns in [31:28], rows in [15:12]
 *                  (0xF001F001: 15 x 15)
 *   +0x08..+0x14   zone widths, a byte each, +0x18..+0x24 heights, in 2x2
 *                  pixel quads (1080p: 64 x 36 quads)
 *   AE +0x28       luma thresholds: [7:0] dark, [15:8] bright
 *   AE +0x2C..+0x38  four zone banks, 0x1000 apart; +0x3C..+0x48 four
 *                  histogram banks, 0x800 apart; +0x4C bank counts - 1,
 *                  zones [7:4], histogram [3:0]; +0x50 the banks last
 *                  written, zones [5:4], histogram [1:0]
 *   AWB +0x3C..+0x48  four zone banks, 0x1000 apart; +0x4C count - 1;
 *                  +0x50 the bank last written
 *
 * An AE zone (tisp_ae0_get_statistics) is 16 bytes, row by row: three
 * 21-bit luma sums -- of the dark, middle and bright quads -- then a
 * 21-bit field the SDK's AE does not use, and 13-bit counts of the dark
 * and the bright quads: bits [20:0] dark sum, [41:21] middle, [62:42]
 * bright, [75:63] dark count, [88:76] bright count, [109:89] unused.
 * ae0_weight_mean2 takes (sums) / (zone width x height) as the zone's
 * mean.  A histogram bank (tisp_ae0_get_hist) is 256 21-bit counts of the
 * frame's quads by luma, twice (the second copy for a second window).
 * An AWB zone (JZ_Isp_Get_Awb_Statistics) is 16 bytes: 21-bit R, G, B
 * and IR sums and a 13-bit count of the quads summed, at bits [20:0],
 * [41:21], [62:42], [83:63], [96:84].  T40 and T41 (stat_gen 2, blocks
 * at 0x19000 / 0x18000, tisp_ae_get_statistics, tisp_awb_get_statistics)
 * widen the AE sums to 22 bits and the counts to 14, and give an AWB zone
 * 128 bytes: eight records of 22-bit R, G, B, IR and a 14-bit count,
 * which the kernel takes as four groups of two -- all eight get the
 * zone's sums here.  T21 (stat_gen 3, blocks at 0x700 / 0x800) keeps
 * T31's AE zones but counts its histogram in 20 bits, and its AWB zone
 * has no IR sum: R, G, B and a 13-bit count at [20:0], [41:21], [62:42],
 * [75:63] (tisp_ae_get_hist, JZ_Isp_Get_Awb_Statistics).
 *
 * The quads are sampled 8 x 6 a zone from the scene under the frame's
 * light, in the sensor's linear raw light (isp_lin()) and colour: the
 * light's colour scales the linear R and B, so that a white reads R
 * "light-r" and B "light-b" per mille of G (0.55 and 0.6 by default, a
 * daylight raw white the T21 and T31 tunings' windows take).  T40 and
 * T41 keep the earlier calibration, the colour applied before the 2.2
 * gamma (a white then reads 0.55^2.2 = 0.27 R and 0.33 B), under which
 * their AWB follows the light.
 * Each bank done raises its int_sta bit.
 */
#define STAT_ZONES      15
#define STAT_SX         8
#define STAT_SY         6

typedef struct IspZone {
    uint64_t sum[3], cnt[3];    /* AE: dark, middle, bright */
    uint64_t rgb[3], n;         /* AWB, the quads the filter passes */
    uint64_t urgb[3], un;       /* AWB, every quad (T40/T41's first record) */
} IspZone;

static void isp_bits(uint8_t *z, int bit, int len, uint64_t v)
{
    int i;

    v = MIN(v, (1ull << len) - 1);
    for (i = 0; i < len; i++, bit++) {
        if (v & (1ull << i)) {
            z[bit >> 3] |= 1 << (bit & 7);
        }
    }
}

/* the zone edges, in quads, from the grid registers at @blk; false: unset */
static bool isp_grid(IngenicIspState *s, hwaddr blk, int *cols, int *rows,
                     int *xs, int *ys)
{
    uint32_t grid = ISP_REG(blk + 0x04);
    int i;

    *cols = MIN(grid >> 28, STAT_ZONES);
    *rows = MIN((grid >> 12) & 0xF, STAT_ZONES);
    if (!*cols || !*rows) {
        return false;
    }
    xs[0] = ys[0] = 0;
    for (i = 0; i < STAT_ZONES; i++) {
        xs[i + 1] = xs[i] + extract32(ISP_REG(blk + 0x08 + (i & ~3)),
                                      8 * (i & 3), 8);
        ys[i + 1] = ys[i] + extract32(ISP_REG(blk + 0x18 + (i & ~3)),
                                      8 * (i & 3), 8);
    }
    return xs[*cols] && ys[*rows];
}

/*
 * The pixels the AWB block sums: tiziano_awb_set_hardware_param programs
 * a filter after the zone sizes (T21 0x828.., T31 0xB028.., from the
 * tuning's _awb_parameter), and only a quad inside it is summed and
 * counted -- the count is of the quads that passed, and the AWB drops a
 * zone whose count is at most its _pixel_cnt_th:
 *   +0x28  raw R/G (Q8) window, [15:0] min, [31:16] max
 *   +0x2C  raw B/G window, the same
 *   +0x30  [15:0] k, [31:16] b: k R/G / 256 + B/G >= b
 *   +0x34  [15:0] k, [31:16] b: k R/G / 256 + B/G <= b
 *   +0x38  luma window, [7:0] low, [15:8] high
 * (the driver's "everything" values -- R/G 1..4095, b 0 and 65535 --
 * read so: its low-light and light-source cases write them).  T40 and
 * T41 (0x18028.., tisp_awb_set_regional_threshold) program the same
 * form (on the T41 SDK image R/G 60..245, B/G 30..200, k 256 with b 170
 * and 320; the luma window in +0x38's bytes 0 and 2, 10..230).  Their
 * zones are 128 bytes: four pairs of 16-byte records
 * (tisp_awb_get_statistics), a pair per statistics position, the one
 * the tuning picks read (bytes 3274/3276 of its block; with 3274 set
 * only that pair is stored, 32 bytes a zone).  Of a pair the AWB's
 * gains follow the second record alone: filling the first filtered or
 * not never moved them, in eight runs under three light colours.
 * Filtered as above in linear light, the second record leaves T41's AWB
 * nothing in this scene -- its AE exposes for the navy, the white logo
 * clips past the luma window, and the AWB keeps its start gains
 * (2212 / 1856) -- so T40 and T41 fill all eight records unfiltered,
 * with the earlier calibration of the light's colour (below), under
 * which their AWB follows the light.  The luma of a quad is taken as
 * (R + 2 G + B) / 4 of its linear raw values, and a filter that is all
 * zero (not yet programmed) passes everything.
 */
typedef struct AwbFilter {
    bool on;
    uint32_t rg_min, rg_max, bg_min, bg_max;
    uint32_t k_lo, b_lo, k_hi, b_hi, y_lo, y_hi;
} AwbFilter;

static void awb_filter_load(IngenicIspState *s, hwaddr blk, AwbFilter *f)
{
    uint32_t r28 = ISP_REG(blk + 0x28), r2c = ISP_REG(blk + 0x2C);
    uint32_t r30 = ISP_REG(blk + 0x30), r34 = ISP_REG(blk + 0x34);
    uint32_t r38 = ISP_REG(blk + 0x38);

    f->on = r28 | r2c | r34 | r38;
    f->rg_min = r28 & 0xFFFF, f->rg_max = r28 >> 16;
    f->bg_min = r2c & 0xFFFF, f->bg_max = r2c >> 16;
    f->k_lo = r30 & 0xFFFF, f->b_lo = r30 >> 16;
    f->k_hi = r34 & 0xFFFF, f->b_hi = r34 >> 16;
    if (s->l->stat_gen == 2) {
        /*
         * tisp_awb_set_lum_th_freq: three luma thresholds in bytes 0..2
         * (interpolated by EV), a frequency in byte 3; the window is the
         * outer two
         */
        f->y_lo = r38 & 0xFF, f->y_hi = (r38 >> 16) & 0xFF;
    } else {
        f->y_lo = r38 & 0xFF, f->y_hi = (r38 >> 8) & 0xFF;
    }
}

static bool awb_quad_valid(const AwbFilter *f, const int *q)
{
    uint32_t rg, bg, y;

    if (!f->on) {
        return true;
    }
    if (!q[1]) {
        return false;
    }
    y = (q[0] + 2 * q[1] + q[2]) / 4;
    rg = q[0] * 256 / q[1];
    bg = q[2] * 256 / q[1];
    return y >= f->y_lo && y <= f->y_hi &&
           rg >= f->rg_min && rg <= f->rg_max &&
           bg >= f->bg_min && bg <= f->bg_max &&
           f->k_lo * rg / 256 + bg >= f->b_lo &&
           f->k_hi * rg / 256 + bg <= f->b_hi;
}

/*
 * AE1 (T31, T23: the short exposure's AE, tisp_ae1_get_statistics /
 * ae1_interrupt_static / _hist on int_sta bits 28 and 29): AE0's block
 * again at AE0 + 0x800 (0xA800; system_reg_write_ae(2) writes its bit 0
 * after every AE1 update -- a latch, not an enable), its banks set by
 * tisp_init after AE0's, the same zone and histogram formats.  It is
 * the WDR short frame's: like the WDR statistics it runs only while
 * 0x0C bit 0 is clear (tisp_init's linear mode sets it).  Raised in a
 * linear mode, ae1_interrupt_hist stores its histogram over the AE's
 * static data and pushes the AE1 process, and AE0 and the AWB stop
 * where they are.  Its statistics are of the short frame, lit by the
 * sensor's short exposure (0x0200:0x0201 on the GC4653) -- the long
 * frame's until the sensor has one.  tisp_ae1_process hands their zone
 * means and histogram to the WDR module; tisp_ae1_expt sets the short
 * exposure from the long one's through the tuning's tables.
 */
#define T31_AE1         0xA800

/*
 * AE zones (16 bytes each, row by row) and the 512-entry histogram bank
 * of the scene lit by @gain, as ingenic_isp_stats() writes AE0's, for the
 * AE block at @blk (its luma thresholds at +0x28): for a second
 * exposure, the WDR short frame's
 */
static void isp_ae_fill(IngenicIspState *s, hwaddr blk, uint32_t frame,
                        uint32_t gain, int cols, int rows, const int *xs,
                        const int *ys, uint8_t *buf, uint32_t *hw)
{
    uint32_t thr = ISP_REG(blk + 0x28) ? ISP_REG(blk + 0x28) :
                   ISP_REG(s->l->ae + 0x28);
    int lo = thr & 0xFF, hi = (thr >> 8) & 0xFF;
    int w = 2 * xs[cols], h = 2 * ys[rows];
    uint64_t hist[256] = { 0 };
    double k = gain / 256.0;
    int zx, zy, i, j, c;

    memset(buf, 0, 16 * rows * cols);
    for (zy = 0; zy < rows; zy++) {
        for (zx = 0; zx < cols; zx++) {
            uint64_t quads = (uint64_t)(xs[zx + 1] - xs[zx]) *
                             (ys[zy + 1] - ys[zy]);
            uint64_t sum[3] = { 0 }, cnt[3] = { 0 };
            uint8_t *o = buf + 16 * (zy * cols + zx);

            for (j = 0; j < STAT_SY; j++) {
                for (i = 0; i < STAT_SX; i++) {
                    int qx = xs[zx] + (2 * i + 1) * (xs[zx + 1] - xs[zx]) /
                             (2 * STAT_SX);
                    int qy = ys[zy] + (2 * j + 1) * (ys[zy + 1] - ys[zy]) /
                             (2 * STAT_SY);
                    uint8_t yuv[3];
                    int y;

                    ingenic_scene_pixel(frame, w, h, 2 * qx, 2 * qy, yuv);
                    y = lround(MIN(isp_lin(yuv[0]) * k, 1) * 255);
                    c = y < lo ? 0 : y > hi ? 2 : 1;
                    sum[c] += y;
                    cnt[c]++;
                    hist[y] += quads;
                }
            }
            for (c = 0; c < 3; c++) {
                sum[c] = sum[c] * quads / (STAT_SX * STAT_SY);
                cnt[c] = cnt[c] * quads / (STAT_SX * STAT_SY);
            }
            isp_bits(o, 0, 21, sum[0]);
            isp_bits(o, 21, 21, sum[1]);
            isp_bits(o, 42, 21, sum[2]);
            isp_bits(o, 63, 13, cnt[0]);
            isp_bits(o, 76, 13, cnt[2]);
        }
    }
    for (i = 0; i < 256; i++) {
        hw[i] = hw[256 + i] = cpu_to_le32(MIN(hist[i] / (STAT_SX * STAT_SY),
                                              0x1FFFFF));
    }
}

static void ingenic_isp_ae1(IngenicIspState *s, const uint8_t *zones,
                            size_t zlen, const uint32_t *hist, size_t hlen,
                            int cols, int rows, uint32_t frame,
                            uint32_t sgain)
{
    uint32_t cnt = ISP_REG(T31_AE1 + 0x4C), idx = ISP_REG(T31_AE1 + 0x50);
    int c1, r1, xs[STAT_ZONES + 1], ys[STAT_ZONES + 1];
    int zb = (((idx >> 4) & 3) + 1) % (((cnt >> 4) & 3) + 1);
    int hb = ((idx & 3) + 1) % ((cnt & 3) + 1);
    static uint8_t sbuf[STAT_ZONES * STAT_ZONES * 16];
    static uint32_t shw[512];

    if ((ISP_REG(T31_TOP) & 1) || !ISP_REG(T31_AE1 + 0x2C) ||
        !isp_grid(s, T31_AE1, &c1, &r1, xs, ys) || c1 != cols || r1 != rows) {
        return;
    }
    if (sgain) {
        /* the short frame's own */
        isp_ae_fill(s, T31_AE1, frame, sgain, cols, rows, xs, ys, sbuf,
                    shw);
        zones = sbuf, hist = shw;
        zlen = 16 * rows * cols, hlen = sizeof(shw);
    }
    dma_memory_write(&address_space_memory,
                     ISP_REG(T31_AE1 + 0x2C + 4 * zb), zones, zlen,
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory,
                     ISP_REG(T31_AE1 + 0x3C + 4 * hb), hist, hlen,
                     MEMTXATTRS_UNSPECIFIED);
    ISP_REG(T31_AE1 + 0x50) = zb << 4 | hb;
    ISP_REG(s->l->int_sta) |= 1u << 28 | 1u << 29;
}

static void ingenic_isp_stats(IngenicIspState *s, uint32_t gain, int pipe,
                              uint32_t frame)
{
    const IngenicIspLayout *l = s->l;
    hwaddr ae_blk = PIPE_STAT(pipe, l->ae), awb_blk = PIPE_STAT(pipe, l->awb);
    hwaddr int_sta = PIPE_INT(pipe, l->int_sta);
    static IspZone zone[STAT_ZONES][STAT_ZONES];
    uint64_t hist[256];
    uint8_t buf[STAT_ZONES * STAT_ZONES * 16];
    static uint8_t abuf[STAT_ZONES * STAT_ZONES * 128];
    int cols, rows, xs[STAT_ZONES + 1], ys[STAT_ZONES + 1];
    int acols, arows, axs[STAT_ZONES + 1], ays[STAT_ZONES + 1];
    uint32_t thr = ISP_REG(ae_blk + 0x28);
    int lo = thr & 0xFF, hi = (thr >> 8) & 0xFF;
    /* T21 has no enable bits: a block runs once it has its banks */
    bool en_bits = l->stat_gen != 3;
    bool ae = (!en_bits || (ISP_REG(ae_blk) & 1)) && ISP_REG(ae_blk + 0x2C);
    bool awb = (!en_bits || (ISP_REG(awb_blk) & 1)) && ISP_REG(awb_blk + 0x3C);
    int zx, zy, i, j, w, h;
    AwbFilter flt;

    if (!ae && !awb) {
        return;
    }
    if (!isp_grid(s, ae_blk, &cols, &rows, xs, ys)) {
        ae = false;
    }
    if (!isp_grid(s, awb_blk, &acols, &arows, axs, ays)) {
        awb = false;
    }
    if (!ae && !awb) {
        return;
    }
    /* the scene at the AE grid's size (the AWB grid is the same one) */
    w = 2 * (ae ? xs[cols] : axs[acols]);
    h = 2 * (ae ? ys[rows] : ays[arows]);
    if (!ae) {
        cols = acols, rows = arows;
        memcpy(xs, axs, sizeof(xs));
        memcpy(ys, ays, sizeof(ys));
    }
    memset(zone, 0, sizeof(zone));
    memset(hist, 0, sizeof(hist));
    awb_filter_load(s, awb_blk, &flt);
    for (zy = 0; zy < rows; zy++) {
        for (zx = 0; zx < cols; zx++) {
            IspZone *z = &zone[zy][zx];
            uint64_t quads = (uint64_t)(xs[zx + 1] - xs[zx]) *
                             (ys[zy + 1] - ys[zy]);
            int c;

            for (j = 0; j < STAT_SY; j++) {
                for (i = 0; i < STAT_SX; i++) {
                    int qx = xs[zx] + (2 * i + 1) * (xs[zx + 1] - xs[zx]) /
                             (2 * STAT_SX);
                    int qy = ys[zy] + (2 * j + 1) * (ys[zy + 1] - ys[zy]) /
                             (2 * STAT_SY);
                    uint8_t yuv[3];
                    double k = gain / 256.0, yn, cb, cr, rgb[3];
                    int y, ch, q[3];

                    ingenic_scene_pixel(frame, w, h, 2 * qx, 2 * qy, yuv);
                    /* the sensor's linear light, 8 bits */
                    y = lround(MIN(isp_lin(yuv[0]) * k, 1) * 255);
                    c = y < lo ? 0 : y > hi ? 2 : 1;
                    z->sum[c] += y;
                    z->cnt[c]++;
                    hist[y] += quads;
                    /* and its colour: the painted RGB, linear, raw */
                    yn = (yuv[0] - 16) / 219.0;
                    cb = (yuv[1] - 128) / 224.0;
                    cr = (yuv[2] - 128) / 224.0;
                    rgb[0] = yn + 1.402 * cr;
                    rgb[1] = yn - 0.344 * cb - 0.714 * cr;
                    rgb[2] = yn + 1.772 * cb;
                    for (ch = 0; ch < 3; ch++) {
                        double lc = ch == 0 ? s->light_r / 1000.0 :
                                    ch == 2 ? s->light_b / 1000.0 : 1;
                        double v = pow(MIN(MAX(rgb[ch], 0), 1), 2.2) * k * lc;

                        q[ch] = lround(MIN(v, 1) * 255);
                        z->urgb[ch] += q[ch];
                    }
                    z->un++;
                    if (awb_quad_valid(&flt, q)) {
                        for (ch = 0; ch < 3; ch++) {
                            z->rgb[ch] += q[ch];
                        }
                        z->n++;
                    }
                }
            }
            /* from the samples to the zone's quads */
            for (c = 0; c < 3; c++) {
                z->sum[c] = z->sum[c] * quads / (STAT_SX * STAT_SY);
                z->cnt[c] = z->cnt[c] * quads / (STAT_SX * STAT_SY);
                z->rgb[c] = z->rgb[c] * quads / (STAT_SX * STAT_SY);
                z->urgb[c] = z->urgb[c] * quads / (STAT_SX * STAT_SY);
            }
            z->n = z->n * quads / (STAT_SX * STAT_SY);
            z->un = z->un * quads / (STAT_SX * STAT_SY);
        }
    }
    if (ae) {
        uint32_t cnt = ISP_REG(ae_blk + 0x4C), idx = ISP_REG(ae_blk + 0x50);
        int zb = (((idx >> 4) & 3) + 1) % (((cnt >> 4) & 3) + 1);
        int hb = ((idx & 3) + 1) % ((cnt & 3) + 1);
        uint32_t hw[512];

        int sb = l->stat_gen == 2 ? 22 : 21, cb = sb - 8;

        memset(buf, 0, sizeof(buf));
        for (zy = 0; zy < rows; zy++) {
            for (zx = 0; zx < cols; zx++) {
                IspZone *z = &zone[zy][zx];
                uint8_t *o = buf + 16 * (zy * cols + zx);

                /*
                 * T21's AE settles at a quarter of the zone means T31's
                 * does with the same scene and exposure: a calibration,
                 * not read off the module (its AE target is a table in
                 * the sensor's tuning file)
                 */
                int dv = l->stat_gen == 3 ? 4 : 1;

                isp_bits(o, 0, sb, z->sum[0] / dv);
                isp_bits(o, sb, sb, z->sum[1] / dv);
                isp_bits(o, 2 * sb, sb, z->sum[2] / dv);
                isp_bits(o, 3 * sb, cb, z->cnt[0]);
                isp_bits(o, 3 * sb + cb, cb, z->cnt[2]);
            }
        }
        dma_memory_write(&address_space_memory,
                         ISP_REG(ae_blk + 0x2C + 4 * zb), buf,
                         16 * rows * cols, MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < 256; i++) {
            hw[i] = hw[256 + i] = cpu_to_le32(MIN(hist[i] / (STAT_SX *
                                                             STAT_SY),
                                                  l->stat_gen == 3 ?
                                                  0xFFFFF : 0x1FFFFF));
        }
        dma_memory_write(&address_space_memory,
                         ISP_REG(ae_blk + 0x3C + 4 * hb), hw, sizeof(hw),
                         MEMTXATTRS_UNSPECIFIED);
        ISP_REG(ae_blk + 0x50) = zb << 4 | hb;
        ISP_REG(int_sta) |= 1u << l->ae_static_bit | 1u << l->ae_hist_bit;
        if ((l->xstat == 1 || l->xstat == 2) && pipe == 0) {
            ingenic_isp_ae1(s, buf, 16 * rows * cols, hw, sizeof(hw), cols,
                            rows, frame, ingenic_isp_short_gain(s, pipe));
        }
    }
    if (awb) {
        uint32_t cnt = ISP_REG(awb_blk + 0x4C), idx = ISP_REG(awb_blk + 0x50);
        int ab = ((idx & 3) + 1) % ((cnt & 3) + 1);
        int zs = l->stat_gen == 2 ? 128 : 16;

        memset(abuf, 0, zs * arows * acols);
        for (zy = 0; zy < arows; zy++) {
            for (zx = 0; zx < acols; zx++) {
                /* the AE zone this AWB zone's centre falls in */
                int ex = 0, ey = 0;
                int cx = (axs[zx] + axs[zx + 1]) / 2;
                int cy = (ays[zy] + ays[zy + 1]) / 2;
                IspZone *z;
                uint8_t *o = abuf + zs * (zy * acols + zx);

                while (ex + 1 < cols && cx >= xs[ex + 1]) {
                    ex++;
                }
                while (ey + 1 < rows && cy >= ys[ey + 1]) {
                    ey++;
                }
                z = &zone[ey][ex];
                if (l->stat_gen == 2) {
                    /* a pair: every quad, then the filtered ones */
                    for (i = 0; i < 8; i++) {
                        const uint64_t *c = i & 1 ? z->rgb : z->urgb;

                        isp_bits(o + 16 * i, 0, 22, c[0]);
                        isp_bits(o + 16 * i, 22, 22, c[1]);
                        isp_bits(o + 16 * i, 44, 22, c[2]);
                        isp_bits(o + 16 * i, 88, 14, i & 1 ? z->n : z->un);
                    }
                } else {
                    isp_bits(o, 0, 21, z->rgb[0]);
                    isp_bits(o, 21, 21, z->rgb[1]);
                    isp_bits(o, 42, 21, z->rgb[2]);
                    isp_bits(o, l->stat_gen == 3 ? 63 : 84, 13, z->n);
                }
            }
        }
        dma_memory_write(&address_space_memory,
                         ISP_REG(awb_blk + 0x3C + 4 * ab), abuf,
                         zs * arows * acols, MEMTXATTRS_UNSPECIFIED);
        ISP_REG(awb_blk + 0x50) = ab;
        ISP_REG(int_sta) |= 1u << l->awb_bit;
    }
    if (s->trace) {
        uint64_t sum = 0, n = 0;

        for (zy = 0; zy < rows; zy++) {
            for (zx = 0; zx < cols; zx++) {
                for (i = 0; i < 3; i++) {
                    sum += zone[zy][zx].sum[i];
                    n += zone[zy][zx].cnt[i];
                }
            }
        }
        qemu_log("ingenic-isp: pipe %d frame %u stats gain %u/256 mean Y %u%s%s"
                 " wb R %u B %u /1024 light %u/%u top 0x%08x short %u%s\n",
                 pipe, frame, gain, n ? (unsigned)(sum / n) : 0,
                 ae ? " ae" : "", awb ? " awb" : "",
                 l->wb ? ISP_REG(l->wb) & 0xFFF : 0,
                 l->wb ? ISP_REG(l->wb + 4) & 0xFFF : 0,
                 s->light_r, s->light_b,
                 ISP_REG(l->xstat == 3 ? T41_TOP : T31_TOP),
                 (unsigned)(ingenic_isp_sensor_short(pipe ? s->sensor1 :
                                                     s->sensor) / 1024),
                 ingenic_isp_wdr_mode(s) ? " wdr" : "");
        if (s->sensor) {
            IngenicSensorExposure e = { 0 };

            if (galaxycore_exposure(s->sensor, &e)) {
                qemu_log("ingenic-isp: exposure long %u rows short %u rows "
                         "gain %u/1024\n", e.lines, e.short_lines, e.gain);
            }
        }
    }
}

/*
 * The other Tiziano statistics of T31 and T23 (tx-isp-t31.ko / -t23.ko,
 * their handlers and the banks tisp_init sets), each raised on its
 * int_sta bit while its block runs:
 *
 * AF (af_interrupt_static, bit 31; tiziano_af_set_hardware_param): the
 *   grid at 0xB804 -- [31:28] columns, [27:16] first column, [15:12]
 *   rows, [11:0] first row, in pixels -- zone widths as bytes at
 *   0xB808.. and heights at 0xB818.. (the frame less the filters'
 *   margins, 15 columns and 3 rows, over the zones); four banks at
 *   0xB8A8.., their count at 0xB8B8 and the last one's number at 0xB8BC.
 *   A zone is 16 bytes (tisp_af_get_statistics): FIR 0 [21:0], FIR 1
 *   [43:22], IIR 0 [65:44], IIR 1 [87:66], the luma sum [110:88] and the
 *   count of bright pixels [125:111].  The model's filters are the
 *   luma's absolute differences to the next pixel and the one after (FIR
 *   0 and 1), and to the next row and the one after (IIR 0 and 1), each
 *   summed over the zone's pixels: what an edge filter answers, larger
 *   with more detail and contrast in the zone.
 * ADR (tiziano_adr_interrupt_static, bit 18; 0x0C bit 7 bypasses it):
 *   four 4 KiB banks at 0x4494.., their count at 0x4490, and 0x44B0 the
 *   physical address of the bank last written (the handler matches it
 *   against the four).  tiziano_adr_get_data reads 6 x 8 blocks, 16
 *   bytes each, 128 to a row: the luma sum [27:0], and a five-bin
 *   histogram of the block's luma in the halfwords at +8, +10, +12, +14
 *   and the next block's +4; then from 0x308 a 512-bin histogram of the
 *   frame's luma, three 21-bit counts to every eight bytes.
 * Defog (tiziano_defog_interrupt_static, bit 20; 0x0C bit 11 bypasses
 *   it): banks at 0x5B84.., count 0x5B80, the last one's address at
 *   0x5BA4.  From +8, 180 blocks (15 x 12) of 16 bytes
 *   (tiziano_defog_get_data): the B sum [23:0], G [47:24], R [59:48] |
 *   [123:112] << 12, and three halfwords at +8, +10, +12 the algorithm
 *   keeps as the block's "hist info" -- filled here with the block's
 *   darkest channel (its dark channel), its brightest luma and its mean
 *   luma, which the module's names do not settle.
 * WDR (T31, tiziano_wdr_interrupt_static, bit 11; only with 0x0C bit 0
 *   clear, the WDR modes): banks of 8 KiB at 0x2010.., count 0x2024, the
 *   last one's address at 0x2680.  The handler first packs eight
 *   516-byte rows, 1 KiB apart, end to end, then reads 256 entries of 16
 *   bytes -- the long and the short exposure's R, G, B histograms, 21
 *   bits each -- then the two exposures' luma counts (dark, middle,
 *   bright, against AE0's thresholds) and luma sums.  In a linear mode
 *   the two exposures see the same frame.
 *
 * Counts and sums are of the 2 x 2 quads, as AE0's are.
 *
 * T40 and T41 (xstat 3; tx-isp-t40.ko / -t41.ko, their symbols): AF is
 * T31's block at 0x1A000 (tisp_af_set_hardware_param writes the same
 * grid word at 0x1A004, the zone widths as halfwords from 0x1A008 and
 * the heights as bytes from 0x1A028), on int_sta bit 6
 * (tisp_af_interrupt_static, T40's tisp_af_main_interrupt_static), four
 * 4 KiB banks at 0x1A0CC.. (tisp_af_malloc_cfg), their count at 0x1A0DC
 * and the last one's number at 0x1A0E0.  tisp_af_get_statistics unpacks
 * zones of 136 bits end to end -- four to 68 bytes -- each FIR 0, FIR 1,
 * IIR 0, IIR 1 in 23 bits, the luma sum in 24 and the bright count in
 * 16, a row of them every 2 * {3,5,7,9,11,13,15,17,20,22,24,26,28,30,32}
 * [columns - 1] words.
 * ADR (tiziano_adr_interrupt_static, bit 9; T40 the same, _main_): four
 *   4 KiB banks at 0x9514.., count 0x9510, the last one's address at
 *   0x9540.  tiziano_adr_read_data reads 4 rows of 6 blocks, 56 bytes
 *   each: two 29-bit words (here the block's luma sum and its quads),
 *   twenty 18-bit counts from +8 (here a 20-bin histogram of the block's
 *   luma) and two 12-bit fields in the word at +52, bits 8 and 20 (here
 *   its darkest and brightest luma, 12 bits); then from +0x548 the
 *   frame's 512-bin luma histogram as on T31.  The meaning of each field
 *   is the model's reading of the reader's widths, not the module's
 *   names.
 * Defog (tisp_defog_interrupt_static, bit 10): four 2 KiB banks at
 *   0xC290.., count 0xC28C, the last one's address at 0xC2B4; from +8,
 *   180 blocks of 8 bytes (tisp_defog_get_data), three 21-bit sums --
 *   T31's B, G, R order, narrower -- without T31's "hist info".
 */
#define T31_AF          0xB800
#define T31_AF_BIT      31
#define T41_AF          0x1A000
#define T41_AF_BIT      6
#define T41_ADR_CNT     0x9510
#define T41_ADR_LAST    0x9540
#define T41_ADR_BIT     9
#define T41_DEFOG_CNT   0xC28C
#define T41_DEFOG_LAST  0xC2B4
#define T41_DEFOG_BIT   10
#define T31_ADR_CNT     0x4490
#define T31_ADR_LAST    0x44B0
#define T31_ADR_BIT     18
#define T31_DEFOG_CNT   0x5B80
#define T31_DEFOG_LAST  0x5BA4
#define T31_DEFOG_BIT   20
#define T31_WDR_BANK    0x2010
#define T31_WDR_CNT     0x2024
#define T31_WDR_LAST    0x2680
#define T31_WDR_BIT     11

static double isp_lin_lut(int v)
{
    static double lut[256];
    static bool done;
    int i;

    if (!done) {
        for (i = 0; i < 256; i++) {
            lut[i] = isp_lin(i);
        }
        done = true;
    }
    return lut[v & 0xFF];
}

/* the sensor's linear luma at (x, y), 0..1 */
static double isp_stat_y(uint32_t frame, int w, int h, int x, int y,
                         double k)
{
    uint8_t yuv[3];

    ingenic_scene_pixel(frame, w, h, MIN(x, w - 1), MIN(y, h - 1), yuv);
    return MIN(isp_lin_lut(yuv[0]) * k, 1);
}

/* and its linear R, G, B under the light's colour, 0..1 */
static void isp_stat_rgb(IngenicIspState *s, uint32_t frame, int w, int h,
                         int x, int y, double k, double *rgb)
{
    uint8_t yuv[3];
    double yn, cb, cr, c[3];
    int ch;

    ingenic_scene_pixel(frame, w, h, x, y, yuv);
    yn = (yuv[0] - 16) / 219.0;
    cb = (yuv[1] - 128) / 224.0;
    cr = (yuv[2] - 128) / 224.0;
    c[0] = yn + 1.402 * cr;
    c[1] = yn - 0.344 * cb - 0.714 * cr;
    c[2] = yn + 1.772 * cb;
    for (ch = 0; ch < 3; ch++) {
        double lc = ch == 0 ? s->light_r / 1000.0 :
                    ch == 2 ? s->light_b / 1000.0 : 1;

        rgb[ch] = MIN(pow(MIN(MAX(c[ch], 0), 1), 2.2) * k * lc, 1);
    }
}

/* the bank after the one whose address @last holds, of @n at @bank0.. */
static int isp_next_bank(IngenicIspState *s, hwaddr bank0, int n,
                         uint32_t last)
{
    int b;

    for (b = 0; b < n; b++) {
        if (ISP_REG(bank0 + 4 * b) == last) {
            return (b + 1) % n;
        }
    }
    return 0;
}

static const uint8_t t41_af_row[15] = {
    3, 5, 7, 9, 11, 13, 15, 17, 20, 22, 24, 26, 28, 30, 32
};

static void ingenic_isp_af(IngenicIspState *s, uint32_t frame, int w, int h,
                           double k)
{
    bool x2 = s->l->xstat == 3;
    hwaddr af = x2 ? T41_AF : T31_AF;
    hwaddr banks = af + (x2 ? 0xCC : 0xA8);
    uint32_t grid = ISP_REG(af + 0x04);
    int cols = MIN(grid >> 28, STAT_ZONES), rows = MIN((grid >> 12) & 0xF,
                                                       STAT_ZONES);
    int x0 = (grid >> 16) & 0xFFF, y0 = grid & 0xFFF;
    uint32_t cnt = ISP_REG(af + (x2 ? 0xDC : 0xB8));
    uint32_t idx = ISP_REG(af + (x2 ? 0xE0 : 0xBC));
    int bank = (idx + 1) % ((cnt & 3) + 1);
    /* T31: 16 bytes a zone; T40/T41: 136 bits, rows of 8-byte units */
    int row = x2 && cols ? 8 * t41_af_row[cols - 1] : 16 * cols;
    static uint8_t buf[STAT_ZONES * 256];
    int zx, zy, i, j, y = y0;

    if ((!x2 && !(ISP_REG(af) & 1)) || !cols || !rows ||
        !ISP_REG(banks + 4 * bank)) {
        return;
    }
    memset(buf, 0, row * rows);
    for (zy = 0; zy < rows; zy++) {
        int zh = x2 ? extract32(ISP_REG(af + 0x28 + (zy & ~3)), 8 * (zy & 3),
                                8)
                    : extract32(ISP_REG(af + 0x18 + (zy & ~3)), 8 * (zy & 3),
                                8);
        int x = x0;

        for (zx = 0; zx < cols; zx++) {
            int zw = x2 ? extract32(ISP_REG(af + 0x08 + 4 * (zx / 2)),
                                    16 * (zx & 1), 16)
                        : extract32(ISP_REG(af + 0x08 + (zx & ~3)),
                                    8 * (zx & 3), 8);
            uint64_t f0 = 0, f1 = 0, i0 = 0, i1 = 0, ys = 0, hi = 0;
            uint64_t px = (uint64_t)zw * zh;
            uint8_t *o = buf + row * zy;
            int b = x2 ? 136 * zx : 128 * zx;
            enum { NX = 16, NY = 8 };

            for (j = 0; j < NY && zh; j++) {
                for (i = 0; i < NX && zw; i++) {
                    int sx = x + (2 * i + 1) * zw / (2 * NX);
                    int sy = y + (2 * j + 1) * zh / (2 * NY);
                    int v = lround(isp_stat_y(frame, w, h, sx, sy, k) * 255);
                    int vx1 = lround(isp_stat_y(frame, w, h, sx + 1, sy, k) *
                                     255);
                    int vx2 = lround(isp_stat_y(frame, w, h, sx + 2, sy, k) *
                                     255);
                    int vy1 = lround(isp_stat_y(frame, w, h, sx, sy + 1, k) *
                                     255);
                    int vy2 = lround(isp_stat_y(frame, w, h, sx, sy + 2, k) *
                                     255);

                    f0 += abs(vx1 - v);
                    f1 += abs(vx2 - v);
                    i0 += abs(vy1 - v);
                    i1 += abs(vy2 - v);
                    ys += v;
                    hi += v >= 230;
                }
            }
            if (x2) {
                isp_bits(o, b, 23, f0 * px / (NX * NY));
                isp_bits(o, b + 23, 23, f1 * px / (NX * NY));
                isp_bits(o, b + 46, 23, i0 * px / (NX * NY));
                isp_bits(o, b + 69, 23, i1 * px / (NX * NY));
                isp_bits(o, b + 92, 24, ys * px / (NX * NY));
                isp_bits(o, b + 116, 16, hi * px / (NX * NY));
            } else {
                isp_bits(o, b, 22, f0 * px / (NX * NY));
                isp_bits(o, b + 22, 22, f1 * px / (NX * NY));
                isp_bits(o, b + 44, 22, i0 * px / (NX * NY));
                isp_bits(o, b + 66, 22, i1 * px / (NX * NY));
                isp_bits(o, b + 88, 23, ys * px / (NX * NY));
                isp_bits(o, b + 111, 15, hi * px / (NX * NY));
            }
            x += zw;
        }
        y += zh;
    }
    dma_memory_write(&address_space_memory, ISP_REG(banks + 4 * bank), buf,
                     row * rows, MEMTXATTRS_UNSPECIFIED);
    ISP_REG(af + (x2 ? 0xE0 : 0xBC)) = bank;
    ISP_REG(s->l->int_sta) |= 1u << (x2 ? T41_AF_BIT : T31_AF_BIT);
}

static void ingenic_isp_adr(IngenicIspState *s, uint32_t frame, int w, int h,
                            double k)
{
    static uint8_t buf[0x1000];
    uint64_t hist[512] = { 0 };
    bool x2 = s->l->xstat == 3;
    hwaddr cnt = x2 ? T41_ADR_CNT : T31_ADR_CNT;
    hwaddr last = x2 ? T41_ADR_LAST : T31_ADR_LAST;
    int n = (ISP_REG(cnt) & 3) + 1;
    int bank = isp_next_bank(s, cnt + 4, n, ISP_REG(last));
    hwaddr dst = ISP_REG(cnt + 4 + 4 * bank);
    /* T31: 8 x 6 blocks; T40/T41: 6 x 4 */
    int nbx = x2 ? 6 : 8, nby = x2 ? 4 : 6;
    uint64_t quads = (uint64_t)w * h / 4 / (nbx * nby);
    enum { NX = 8, NY = 6 };
    int bx, by, i, j;

    if ((!x2 && (ISP_REG(T31_TOP) & (1u << 7))) || !dst) {
        return;
    }
    memset(buf, 0, sizeof(buf));
    for (by = 0; by < nby; by++) {
        for (bx = 0; bx < nbx; bx++) {
            uint64_t sum = 0, bin[20] = { 0 };
            int lo = 255, hi = 0;
            uint8_t *o = x2 ? buf + 56 * (by * nbx + bx)
                            : buf + 128 * by + 16 * bx;

            for (j = 0; j < NY; j++) {
                for (i = 0; i < NX; i++) {
                    int x = (bx * NX + i) * w / (nbx * NX) + w / (2 * nbx * NX);
                    int y = (by * NY + j) * h / (nby * NY) + h / (2 * nby * NY);
                    double l = isp_stat_y(frame, w, h, x, y, k);
                    int v = lround(l * 255);

                    sum += v;
                    bin[x2 ? MIN(v * 20 / 256, 19) : MIN(v * 5 / 256, 4)]++;
                    lo = MIN(lo, v);
                    hi = MAX(hi, v);
                    hist[lround(l * 511)] += quads;
                }
            }
            if (x2) {
                isp_bits(o, 0, 29, sum * quads / (NX * NY));
                isp_bits(o, 32, 29, quads);
                for (i = 0; i < 20; i++) {
                    isp_bits(o, 64 + 18 * i, 18, bin[i] * quads / (NX * NY));
                }
                isp_bits(o, 52 * 8 + 8, 12, lo * 16);
                isp_bits(o, 52 * 8 + 20, 12, hi * 16);
                continue;
            }
            isp_bits(o, 0, 28, sum * quads / (NX * NY));
            for (i = 0; i < 4; i++) {
                stw_le_p(o + 8 + 2 * i,
                         MIN(bin[i] * quads / (NX * NY), 0xFFFF));
            }
            /* the fifth bin in the next block's halfword at +4 */
            stw_le_p(o + 16 + 4, MIN(bin[4] * quads / (NX * NY), 0xFFFF));
        }
    }
    for (i = 0; i < 512; i++) {
        isp_bits(buf + (x2 ? 0x548 : 0x308) + 8 * (i / 3), 21 * (i % 3), 21,
                 hist[i] / (NX * NY));
    }
    dma_memory_write(&address_space_memory, dst, buf, sizeof(buf),
                     MEMTXATTRS_UNSPECIFIED);
    ISP_REG(last) = dst;
    ISP_REG(s->l->int_sta) |= 1u << (x2 ? T41_ADR_BIT : T31_ADR_BIT);
}

static void ingenic_isp_defog(IngenicIspState *s, uint32_t frame, int w,
                              int h, double k)
{
    static uint8_t buf[0x1000];
    bool x2 = s->l->xstat == 3;
    hwaddr cnt = x2 ? T41_DEFOG_CNT : T31_DEFOG_CNT;
    hwaddr last = x2 ? T41_DEFOG_LAST : T31_DEFOG_LAST;
    int n = (ISP_REG(cnt) & 3) + 1;
    int bank = isp_next_bank(s, cnt + 4, n, ISP_REG(last));
    hwaddr dst = ISP_REG(cnt + 4 + 4 * bank);
    uint64_t quads = (uint64_t)w * h / 4 / 180;
    enum { BX = 15, BY = 12, NX = 6, NY = 4 };
    int bx, by, i, j, ch;

    if ((!x2 && (ISP_REG(T31_TOP) & (1u << 11))) || !dst) {
        return;
    }
    memset(buf, 0, sizeof(buf));
    for (by = 0; by < BY; by++) {
        for (bx = 0; bx < BX; bx++) {
            uint64_t sum[3] = { 0 }, ysum = 0;
            int dark = 255, ymax = 0;
            uint8_t *o = buf + 8 + (x2 ? 8 : 16) * (by * BX + bx);

            for (j = 0; j < NY; j++) {
                for (i = 0; i < NX; i++) {
                    int x = (bx * NX + i) * w / (BX * NX) + w / (2 * BX * NX);
                    int y = (by * NY + j) * h / (BY * NY) + h / (2 * BY * NY);
                    double rgb[3];
                    int v[3], l;

                    isp_stat_rgb(s, frame, w, h, x, y, k, rgb);
                    for (ch = 0; ch < 3; ch++) {
                        v[ch] = lround(rgb[ch] * 255);
                        sum[ch] += v[ch];
                    }
                    l = (v[0] + 2 * v[1] + v[2]) / 4;
                    ysum += l;
                    dark = MIN(dark, MIN(v[0], MIN(v[1], v[2])));
                    ymax = MAX(ymax, l);
                }
            }
            if (x2) {
                isp_bits(o, 0, 21, sum[2] * quads / (NX * NY));
                isp_bits(o, 21, 21, sum[1] * quads / (NX * NY));
                isp_bits(o, 42, 21, sum[0] * quads / (NX * NY));
                continue;
            }
            isp_bits(o, 0, 24, sum[2] * quads / (NX * NY));
            isp_bits(o, 24, 24, sum[1] * quads / (NX * NY));
            sum[0] = MIN(sum[0] * quads / (NX * NY), 0xFFFFFF);
            isp_bits(o, 48, 12, sum[0] & 0xFFF);
            isp_bits(o, 112, 12, sum[0] >> 12);
            stw_le_p(o + 8, dark);
            stw_le_p(o + 10, ymax);
            stw_le_p(o + 12, ysum / (NX * NY));
        }
    }
    dma_memory_write(&address_space_memory, dst, buf, x2 ? 0x800 : sizeof(buf),
                     MEMTXATTRS_UNSPECIFIED);
    ISP_REG(last) = dst;
    ISP_REG(s->l->int_sta) |= 1u << (x2 ? T41_DEFOG_BIT : T31_DEFOG_BIT);
}

/*
 * T40 and T41's other statistics (xstat 3), each on its int_sta bit:
 *
 * GSM (tisp_gsm_interrupt_hist, bit 7): when 0x1B004 and 0x1B044 both
 *   read 1 the handler reads two 256-entry RAMs of 22-bit counts
 *   (tisp_gsm_read_ram): writing 0x201 to 0x501A0 / 0x501B0 opens a RAM,
 *   each read of 0x501A8 / 0x501B8 gives its next entry, and the driver
 *   writes 0xFF0202 back and 1 to 0x1B000 / 0x1B040 (done).
 *   tisp_gsm_process adds the two bin by bin and reports "hist error"
 *   unless the total is the sensor's width x height (tisp_gsm_init):
 *   here the frame's 256-bin luma histogram of exactly as many pixels
 *   as 0x80 (width << 16 | height) says, each bin split between them.
 * LCE (tisp_lce_interrupt_static, bit 8): 0x0E014 bit 0 says a frame is
 *   ready and bit 8 which of its two RAM sets to read; tisp_lce_get_data
 *   reads 45 x 32 words through 0x501C8 (set 0; 0x501D8 set 1, opened by
 *   0x501C0 / 0x501D0) and 45 more through 0x50208 (0x50218; 0x50200 /
 *   0x50210), then the handler writes 0x0E010 to acknowledge.  Here
 *   those are 45 blocks (9 x 5) of a 32-bin luma histogram of quads,
 *   then the blocks' mean luma; both sets alike.  lce_std_hist_transform
 *   scales each block's bins to sum to 2^20 and reports "LCE Hist Data
 *   Standard Error" when they come to more: a block counts no more
 *   quads than it has.
 * TMO (tisp_tmo_interrupt_static, bit 17): two 16 KiB banks at 0x1E010
 *   and 0x1E014 (0x1E020 = 1, tisp_tmo_init), 0x1E028 the one written;
 *   tisp_tmo_get_data reads 3750 pairs of halfwords -- here a 75 x 50
 *   grid of each block's mean and brightest luma, 12 bits.
 * The field meanings are the model's reading of the readers; the
 *   modules' names do not settle them.  These three are T41's: T40's
 *   module reads its GSM and LCE through other paths and has no TMO.
 */
#define T41_GSM_FLAG0   0x1B004
#define T41_GSM_FLAG1   0x1B044
#define T41_GSM_BIT     7
#define T41_LCE_STA     0x0E014
#define T41_LCE_BIT     8
#define T41_TMO_BANK    0x1E010
#define T41_TMO_IDX     0x1E028
#define T41_TMO_BIT     17

/* the ports: control, data, the RAM, its length */
static const struct {
    hwaddr ctl, data;
    int ram, len;
} t41_ports[6] = {
    { 0x501A0, 0x501A8, 0, 256 },       /* GSM 0 */
    { 0x501B0, 0x501B8, 1, 256 },       /* GSM 1 */
    { 0x501C0, 0x501C8, 2, 1440 },      /* LCE set 0 */
    { 0x501D0, 0x501D8, 2, 1440 },      /* LCE set 1 */
    { 0x50200, 0x50208, 3, 45 },        /* LCE means, set 0 */
    { 0x50210, 0x50218, 3, 45 },        /* LCE means, set 1 */
};

static uint32_t *t41_port_ram(IngenicIspState *s, int ram)
{
    return ram < 2 ? s->ram_gsm[ram] : ram == 2 ? s->ram_lce : s->ram_lce_mean;
}

static void ingenic_isp_xstats2(IngenicIspState *s, uint32_t frame, int w,
                                int h, double k)
{
    static uint8_t tmo[3750 * 4];
    uint64_t hist[256] = { 0 };
    int i, j, bx, by;

    /* GSM: the frame's luma histogram, sampled 64 x 36 */
    for (j = 0; j < 36; j++) {
        for (i = 0; i < 64; i++) {
            int x = (2 * i + 1) * w / 128, y = (2 * j + 1) * h / 72;

            hist[lround(isp_stat_y(frame, w, h, x, y, k) * 255)]++;
        }
    }
    {
        /*
         * tisp_gsm_process adds the two RAMs bin by bin and wants the
         * total to be the sensor's width x height (tisp_gsm_init): the
         * input size (0x80, width << 16 | height) spread over the bins,
         * each bin's count split between the RAMs
         */
        uint64_t total = (uint64_t)(ISP_REG(0x80) >> 16) *
                         (ISP_REG(0x80) & 0xFFFF), left = total;

        for (i = 0; i < 256; i++) {
            uint64_t c = i == 255 ? left : hist[i] * total / (64 * 36);

            c = MIN(c, left);
            left -= c;
            s->ram_gsm[0][i] = MIN(c / 2, 0x3FFFFF);
            s->ram_gsm[1][i] = MIN(c - c / 2, 0x3FFFFF);
        }
    }
    ISP_REG(T41_GSM_FLAG0) = ISP_REG(T41_GSM_FLAG1) = 1;
    ISP_REG(s->l->int_sta) |= 1u << T41_GSM_BIT;

    /* LCE: 9 x 5 blocks, 32 bins each, then their means */
    for (by = 0; by < 5; by++) {
        for (bx = 0; bx < 9; bx++) {
            uint32_t *bin = &s->ram_lce[32 * (by * 9 + bx)];
            /* in 2 x 2 quads, as every statistics block counts */
            uint64_t sum = 0, px = (uint64_t)w * h / 4 / 45 / 64;

            memset(bin, 0, 32 * sizeof(*bin));
            for (j = 0; j < 8; j++) {
                for (i = 0; i < 8; i++) {
                    int x = (bx * 8 + i) * w / 72 + w / 144;
                    int y = (by * 8 + j) * h / 40 + h / 80;
                    int v = lround(isp_stat_y(frame, w, h, x, y, k) * 255);

                    bin[v >> 3] += px;
                    sum += v;
                }
            }
            s->ram_lce_mean[by * 9 + bx] = sum / 64;
        }
    }
    ISP_REG(T41_LCE_STA) = 1;
    ISP_REG(s->l->int_sta) |= 1u << T41_LCE_BIT;

    /* TMO: the next bank, a 75 x 50 grid */
    if (ISP_REG(T41_TMO_BANK)) {
        int b = (ISP_REG(T41_TMO_IDX) + 1) & 1;
        hwaddr dst = ISP_REG(T41_TMO_BANK + 4 * b);

        if (dst) {
            for (by = 0; by < 50; by++) {
                for (bx = 0; bx < 75; bx++) {
                    int x = (2 * bx + 1) * w / 150, y = (2 * by + 1) * h / 100;
                    double l0 = isp_stat_y(frame, w, h, x, y, k);
                    double l1 = isp_stat_y(frame, w, h, x + w / 300, y, k);

                    stw_le_p(tmo + 4 * (by * 75 + bx),
                             lround((l0 + l1) / 2 * 4095));
                    stw_le_p(tmo + 4 * (by * 75 + bx) + 2,
                             lround(MAX(l0, l1) * 4095));
                }
            }
            dma_memory_write(&address_space_memory, dst, tmo, sizeof(tmo),
                             MEMTXATTRS_UNSPECIFIED);
            ISP_REG(T41_TMO_IDX) = b;
            ISP_REG(s->l->int_sta) |= 1u << T41_TMO_BIT;
        }
    }
}

static void ingenic_isp_wdr(IngenicIspState *s, uint32_t frame, int w,
                            int h, double k, double ks)
{
    static uint8_t logical[0x1020], buf[0x2000];
    uint64_t hist[2][3][256] = { { { 0 } } }, num[2][3] = { { 0 } };
    uint64_t ysum[2] = { 0 };
    int n = (ISP_REG(T31_WDR_CNT) & 3) + 1;
    int bank = isp_next_bank(s, T31_WDR_BANK, n, ISP_REG(T31_WDR_LAST));
    hwaddr dst = ISP_REG(T31_WDR_BANK + 4 * bank);
    uint32_t thr = ISP_REG(s->l->ae + 0x28);
    int lo = thr & 0xFF, hi = (thr >> 8) & 0xFF;
    uint64_t quads = (uint64_t)w * h / 4;
    enum { NX = 64, NY = 36 };
    int i, j, ch, e;

    if ((ISP_REG(T31_TOP) & 1) || !dst) {
        return;
    }
    for (j = 0; j < NY; j++) {
        for (i = 0; i < NX; i++) {
            for (e = 0; e < 2; e++) {
                double rgb[3];
                int v[3], l;

                isp_stat_rgb(s, frame, w, h, (2 * i + 1) * w / (2 * NX),
                             (2 * j + 1) * h / (2 * NY), e ? ks : k, rgb);
                for (ch = 0; ch < 3; ch++) {
                    v[ch] = lround(rgb[ch] * 255);
                    hist[e][ch][v[ch]] += quads;
                }
                l = (v[0] + 2 * v[1] + v[2]) / 4;
                num[e][l < lo ? 0 : l > hi ? 2 : 1] += quads;
                ysum[e] += l * quads;
            }
        }
    }
    memset(logical, 0, sizeof(logical));
    for (i = 0; i < 256; i++) {
        for (e = 0; e < 2; e++) {
            for (ch = 0; ch < 3; ch++) {
                isp_bits(logical + 16 * i + 8 * e, 21 * ch, 21,
                         hist[e][ch][i] / (NX * NY));
            }
        }
    }
    for (e = 0; e < 2; e++) {
        for (ch = 0; ch < 3; ch++) {
            isp_bits(logical + 0x1000 + 8 * e, 21 * ch, 21,
                     num[e][ch] / (NX * NY));
        }
        stl_le_p(logical + 0x1010 + 8 * e, ysum[e] / (NX * NY));
        stl_le_p(logical + 0x1014 + 8 * e, 1);
    }
    /* eight 516-byte rows, 1 KiB apart */
    memset(buf, 0, sizeof(buf));
    for (i = 0; i < 8; i++) {
        memcpy(buf + 1024 * i, logical + 516 * i, 516);
    }
    dma_memory_write(&address_space_memory, dst, buf, sizeof(buf),
                     MEMTXATTRS_UNSPECIFIED);
    ISP_REG(T31_WDR_LAST) = dst;
    ISP_REG(s->l->int_sta) |= 1u << T31_WDR_BIT;
}

/*
 * T40's and T41's WDR statistics (tisp_wdr_interrupt_static, T40's
 * tisp_wdr_main_interrupt_static, int_sta bit 11 on both, while 0x40 bit
 * 0 is clear): four 6 KiB banks at 0x6010.. (T41 tisp_wdr_init, T40
 * tisp_main_init), 0x6030 the one last written.  tisp_wdr_get_data (the
 * same offsets in both modules) reads, from the bank:
 *   +0      456 values of 24 bits end to end, of which the first 450 are
 *           15 rows of 15 long-frame blocks then 15 short-frame blocks
 *           (here each block's luma sum over its quads)
 *   +1352   four 12-bit values (two words, [11:0] and [27:16]) and at
 *           +1360 two 23-bit ones (here the long and the short frame's
 *           mean luma x 16, their bright and dark quad counts)
 *   +1368   160 values to +1848 (left zero: not identified)
 *   +1848   the long frame's histograms, 256 entries of 8 bytes, three
 *           21-bit counts (R, G, B), and from +3896 the short frame's --
 *           read when the tuning asks (its byte 1828)
 */
#define T41_WDR_BANK    0x6010
#define T41_WDR_IDX     0x6030
#define T41_WDR_BIT     11

static void ingenic_isp_wdr_xb2(IngenicIspState *s, uint32_t frame, int w,
                                int h, double k, double ks)
{
    static uint8_t buf[0x1800];
    enum { NB = 15, SX = 6, SY = 4 };
    uint64_t blk[2][NB][NB] = { { { 0 } } }, hist[2][3][256] = { { { 0 } } };
    uint64_t ysum[2] = { 0 }, n[2] = { 0 }, bright[2] = { 0 }, dark[2] = { 0 };
    uint32_t idx = (ISP_REG(T41_WDR_IDX) + 1) & 3;
    hwaddr dst = ISP_REG(T41_WDR_BANK + 4 * idx);
    uint64_t quads = (uint64_t)(w / NB) * (h / NB) / 4;
    int bx, by, i, j, e, ch;

    if (!dst || !ingenic_isp_wdr_mode(s)) {
        return;
    }
    for (by = 0; by < NB; by++) {
        for (bx = 0; bx < NB; bx++) {
            for (j = 0; j < SY; j++) {
                for (i = 0; i < SX; i++) {
                    int x = (bx * SX + i) * w / (NB * SX) + w / (2 * NB * SX);
                    int y = (by * SY + j) * h / (NB * SY) + h / (2 * NB * SY);

                    for (e = 0; e < 2; e++) {
                        double rgb[3];
                        int v[3], l;

                        isp_stat_rgb(s, frame, w, h, x, y, e ? ks : k, rgb);
                        for (ch = 0; ch < 3; ch++) {
                            v[ch] = lround(rgb[ch] * 255);
                            hist[e][ch][v[ch]] += quads / (SX * SY);
                        }
                        l = (v[0] + 2 * v[1] + v[2]) / 4;
                        blk[e][by][bx] += l;
                        ysum[e] += l;
                        n[e]++;
                        bright[e] += l > 240;
                        dark[e] += l < 16;
                    }
                }
            }
        }
    }
    memset(buf, 0, sizeof(buf));
    for (by = 0; by < NB; by++) {
        for (e = 0; e < 2; e++) {
            for (bx = 0; bx < NB; bx++) {
                int v = by * 2 * NB + e * NB + bx;

                isp_bits(buf, 24 * v, 24, blk[e][by][bx] * quads / (SX * SY));
            }
        }
    }
    for (e = 0; e < 2; e++) {
        isp_bits(buf + 1352 + 4 * e, 0, 12, n[e] ? ysum[e] * 16 / n[e] : 0);
        isp_bits(buf + 1352 + 4 * e, 16, 12, n[e] ? bright[e] * 4095 / n[e] : 0);
        isp_bits(buf + 1360 + 4 * e, 0, 23,
                 dark[e] * (uint64_t)w * h / 4 / MAX(n[e], 1));
    }
    for (e = 0; e < 2; e++) {
        for (i = 0; i < 256; i++) {
            for (ch = 0; ch < 3; ch++) {
                isp_bits(buf + 1848 + 2048 * e + 8 * i, 21 * ch, 21,
                         hist[e][ch][i]);
            }
        }
    }
    dma_memory_write(&address_space_memory, dst, buf, sizeof(buf),
                     MEMTXATTRS_UNSPECIFIED);
    ISP_REG(T41_WDR_IDX) = idx;
    ISP_REG(s->l->int_sta) |= 1u << T41_WDR_BIT;
}

static void ingenic_isp_xstats(IngenicIspState *s, uint32_t gain,
                               uint32_t frame)
{
    int cols, rows, xs[STAT_ZONES + 1], ys[STAT_ZONES + 1], w, h;
    double k = gain / 256.0;

    if (!isp_grid(s, s->l->ae, &cols, &rows, xs, ys)) {
        return;
    }
    w = 2 * xs[cols];
    h = 2 * ys[rows];
    ingenic_isp_af(s, frame, w, h, k);
    ingenic_isp_adr(s, frame, w, h, k);
    ingenic_isp_defog(s, frame, w, h, k);
    if (s->l->xstat == 3) {
        uint32_t sg = ingenic_isp_short_gain(s, 0);

        /* T40 reads GSM and LCE otherwise and has no TMO */
        if (!strcmp(s->l->name, "t41")) {
            ingenic_isp_xstats2(s, frame, w, h, k);
        }
        ingenic_isp_wdr_xb2(s, frame, w, h, k, sg ? sg / 256.0 : k);
        return;
    }
    if (s->l->xstat == 1) {
        uint32_t sg = ingenic_isp_short_gain(s, 0);

        ingenic_isp_wdr(s, frame, w, h, k, sg ? sg / 256.0 : k);
    }
}

/*
 * The VIC's MDMA (T40, vic_mdma_enable for the "snapraw" debug command):
 * raw frames of the VIC's sensor into DDR, 16 bits a pixel.
 *
 *   0x300  bit 31 on, 19:16 frames still to store (the driver tops it up
 *          from its own count, 8 at most), bit 5 set with it
 *   0x304  width << 16 | height   0x308  1: reset
 *   0x310 / 0x314  line strides of channel 0 / 1
 *   0x318 + 4 k    channel 0's buffer k, k = 0..4 in turn (the driver
 *                  moves each slot on as its frame lands)
 *   0x340 + 4 k    channel 1's (two-sensor captures; not stored here)
 *   0x380          the buffer last written
 *
 * Each stored frame raises the second status word's bit 0 (0x1E4, ack
 * 0x1F4), which vic_mdma_irq_function0 serves.  The raw is the scene in
 * an RGGB mosaic, 10 bits right-aligned (the bit depth and alignment are
 * not documented; the driver only copies the bytes to /tmp/snapN.raw).
 */
#define VIC_MDMA_CTRL   0x300
#define VIC_MDMA_SIZE   0x304
#define VIC_MDMA_STRIDE 0x310
#define VIC_MDMA_BUF0   0x318
#define VIC_MDMA_DONE   0x380

static void ingenic_vic_mdma(IngenicIspState *s, int v, uint32_t frame)
{
    uint32_t *r = ingenic_vic_regs(s, v);
    uint32_t ctrl = r[VIC_MDMA_CTRL >> 2];
    uint32_t left = extract32(ctrl, 16, 4);
    uint32_t w = r[VIC_MDMA_SIZE >> 2] >> 16, h = r[VIC_MDMA_SIZE >> 2] & 0xFFFF;
    uint32_t stride = r[VIC_MDMA_STRIDE >> 2];
    uint32_t dst = r[(VIC_MDMA_BUF0 >> 2) + s->mdma_slot[v]];
    g_autofree uint16_t *line = NULL;
    uint32_t x, y;

    if (!(ctrl & (1u << 31)) || !left || !w || !h || w > 8192) {
        return;
    }
    line = g_new(uint16_t, w);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t yuv[3];
            int Y, U, V, c;

            ingenic_scene_pixel(frame, w, h, x, y, yuv);
            Y = yuv[0];
            U = yuv[1] - 128;
            V = yuv[2] - 128;
            /* R G / G B */
            c = (y & 1) == 0 && (x & 1) == 0 ? Y + (1436 * V >> 10) :
                (y & 1) && (x & 1) ? Y + (1815 * U >> 10) :
                Y - ((352 * U + 731 * V) >> 10);
            line[x] = cpu_to_le16(MIN(MAX(c, 0), 255) << 2);
        }
        dma_memory_write(&address_space_memory, dst + y * stride, line,
                         2 * w, MEMTXATTRS_UNSPECIFIED);
    }
    r[VIC_MDMA_DONE >> 2] = dst;
    s->mdma_slot[v] = (s->mdma_slot[v] + 1) % 5;
    left--;
    r[VIC_MDMA_CTRL >> 2] = deposit32(ctrl, 16, 4, left);
    r[(s->l->vic_sta + 4) >> 2] |= 1;
}


/*
 * T41's HLDC, the ISP's lens distortion correction (tiziano-hldc.c of
 * tx-isp-t41.ko H20230630a; on while the module bypasses at 0x40 have
 * bit 20 -- IMPISPModuleCtl.bitBypassHLDC -- clear).  tisp_hldc_write_reg
 * writes, from 16 halfwords p[] (the tuning's, or /dev/tisp's module 21):
 *
 *   0x15000  p1 << 16 | p0         (1, 1 from every tuning: stored)
 *   0x15004  k2 << 16 | k1         luma      0x15008  the same, chroma
 *   0x1500C  luma zoom             0x15010   chroma zoom
 *   0x15014  luma 1/r2max          0x15018   chroma 1/r2max
 *   0x1501C  offy << 16 | offx     luma      0x15020  the same, chroma
 *   0x15024 / 0x15028  p14 / p15   (0: stored)    0x15044  3
 *
 * and tisp_hldc_calc_para gives their meaning for a plane of w x h
 * centred at (cx, cy): 1/r2max = 0x3fffffff / r2max with r2max the
 * squared distance to the farthest corner, so that rho = r^2 / r2max is
 * r^2 * (1/r2max) / 2^30; the radial gain f(rho) = 256 - k1 rho / 4 +
 * k2 rho^2 / 4 (Q8, 256 = 1); the zoom 2^19 / min(f(1), f(rho at the
 * nearer side)) - 5, Q11; offx / offy the centre's offset from the
 * plane's middle.  How the stage applies them is not in the driver; the
 * model takes the reading those values imply (inferred): an output pixel
 * at d from the centre shows the picture at d * f(rho) / 256 * zoom /
 * 2048 -- a barrel correction for k1 > 0 that keeps the corners where
 * they are (f's minimum is the zoom's reciprocal) and pulls the middle
 * in -- sampled bilinearly, luma and chroma by their own registers.  It
 * runs on the sensor's picture (0x80, w << 16 | h), before the scaler;
 * a channel's pixels map to it by the channel's scale.
 */
#define HLDC_BASE       0x15000
#define HLDC_BYPASS     (1u << 20)

static bool ingenic_isp_hldc_on(IngenicIspState *s)
{
    return s->l->hldc && ISP_REG(T41_TOP) && !(ISP_REG(T41_TOP) & HLDC_BYPASS) &&
           (ISP_REG(HLDC_BASE + 4) || ISP_REG(HLDC_BASE + 8)) &&
           ISP_REG(0x80);
}

/* The map of plane @uv (0 luma, 1 chroma) of a w x h channel */
static const int32_t *ingenic_isp_hldc_map(IngenicIspState *s, int uv,
                                           uint32_t w, uint32_t h)
{
    uint32_t sw = ISP_REG(0x80) >> 16, sh = ISP_REG(0x80) & 0xFFFF;
    uint32_t k = ISP_REG(HLDC_BASE + 4 + 4 * uv);
    uint32_t zoom = ISP_REG(HLDC_BASE + 0xC + 4 * uv);
    uint32_t inv = ISP_REG(HLDC_BASE + 0x14 + 4 * uv);
    uint32_t off = ISP_REG(HLDC_BASE + 0x1C + 4 * uv);
    uint32_t key[8] = { w, h, sw, sh, k, zoom, inv, off };
    double k1 = (int16_t)k, k2 = (int16_t)(k >> 16), z = zoom / 2048.0;
    /* the plane's sensor size: a chroma plane is half the picture */
    double pw = uv ? sw / 2.0 : sw, ph = uv ? sh / 2.0 : sh;
    double cw = uv ? w / 2.0 : w, ch = uv ? h / 2.0 : h;
    double sx = pw / cw, sy = ph / ch;
    double cx = (int)(pw / 2) + (int16_t)off, cy = (int)(ph / 2) +
                (int16_t)(off >> 16);
    uint32_t n = (uint32_t)cw * (uint32_t)ch, x, y;
    int32_t *m;

    if (!memcmp(s->hldc_map[uv].key, key, sizeof(key)) && s->hldc_map[uv].xy) {
        return s->hldc_map[uv].xy;
    }
    m = g_renew(int32_t, s->hldc_map[uv].xy, 2 * n);
    for (y = 0; y < (uint32_t)ch; y++) {
        for (x = 0; x < (uint32_t)cw; x++) {
            double dx = (x + 0.5) * sx - 0.5 - cx;
            double dy = (y + 0.5) * sy - 0.5 - cy;
            double rho = (dx * dx + dy * dy) * inv / 1073741824.0;
            double g = (256.0 - k1 * rho / 4 + k2 * rho * rho / 4) / 256.0 * z;
            double ux = (cx + dx * g + 0.5) / sx - 0.5;
            double uy = (cy + dy * g + 0.5) / sy - 0.5;

            m[2 * (y * (uint32_t)cw + x)] = lround(ux * 256);
            m[2 * (y * (uint32_t)cw + x) + 1] = lround(uy * 256);
        }
    }
    memcpy(s->hldc_map[uv].key, key, sizeof(key));
    s->hldc_map[uv].xy = m;
    s->hldc_map[uv].n = n;
    return m;
}

/*
 * Plane @uv of the w x h picture at @addr (stride @stride) through the
 * HLDC: luma bytes, or chroma Cb/Cr pairs moved as units
 */
static void ingenic_isp_hldc_plane(IngenicIspState *s, int uv, uint32_t w,
                                   uint32_t h, int32_t stride, hwaddr addr)
{
    uint32_t cw = uv ? w / 2 : w, ch = uv ? h / 2 : h, bpp = uv ? 2 : 1;
    uint32_t row = cw * bpp, x, y, c;
    const int32_t *m = ingenic_isp_hldc_map(s, uv, w, h);
    uint8_t *src, *dst;

    stride = stride ? stride : (int32_t)w;
    if (s->hldc_buf_size < 2 * row * ch) {
        s->hldc_buf = g_realloc(s->hldc_buf, 2 * row * ch);
        s->hldc_buf_size = 2 * row * ch;
    }
    src = s->hldc_buf;
    dst = src + row * ch;
    for (y = 0; y < ch; y++) {
        dma_memory_read(&address_space_memory, addr + (int64_t)y * stride,
                        src + y * row, row, MEMTXATTRS_UNSPECIFIED);
    }
    for (y = 0; y < ch; y++) {
        for (x = 0; x < cw; x++) {
            int32_t ux = m[2 * (y * cw + x)], uy = m[2 * (y * cw + x) + 1];
            int32_t x0 = ux >> 8, y0 = uy >> 8, fx = ux & 0xFF, fy = uy & 0xFF;
            int32_t xa = MIN(MAX(x0, 0), (int32_t)cw - 1);
            int32_t xb = MIN(MAX(x0 + 1, 0), (int32_t)cw - 1);
            int32_t ya = MIN(MAX(y0, 0), (int32_t)ch - 1);
            int32_t yb = MIN(MAX(y0 + 1, 0), (int32_t)ch - 1);

            for (c = 0; c < bpp; c++) {
                int a = src[ya * row + xa * bpp + c];
                int b = src[ya * row + xb * bpp + c];
                int d = src[yb * row + xa * bpp + c];
                int e = src[yb * row + xb * bpp + c];
                int top = a * (256 - fx) + b * fx;
                int bot = d * (256 - fx) + e * fx;

                dst[y * row + x * bpp + c] =
                    (top * (256 - fy) + bot * fy + 32768) >> 16;
            }
        }
    }
    for (y = 0; y < ch; y++) {
        dma_memory_write(&address_space_memory, addr + (int64_t)y * stride,
                         dst + y * row, row, MEMTXATTRS_UNSPECIFIED);
    }
}

/*
 * The frame of pipe @p: frame start, the picture into each enabled
 * channel with a queued buffer, the 3A statistics, VIC frame done.  Pipe
 * 1 paints the scene with its counter 500000 ahead, so that its pictures
 * tell themselves from pipe 0's.
 */
#define PIPE1_FRAME     500000

static void ingenic_isp_pipe_frame(IngenicIspState *s, int p)
{
    const IngenicIspLayout *l = s->l;
    uint32_t en = ISP_REG(PIPE_CH(p, l->ch_en));
    uint32_t gain = ingenic_isp_gain(s, p);
    uint32_t sgain = ingenic_isp_short_gain(s, p);
    uint32_t frame = s->frame + (p ? PIPE1_FRAME : 0);
    hwaddr int_sta = PIPE_INT(p, l->int_sta);
    uint32_t *vic = ingenic_vic_regs(s, p);
    int n;

    ISP_REG(int_sta) |= l->fs_bit;
    if (!l->no_vic) {
        vic[l->vic_sta >> 2] |= l->vic_fs_bit;
    }
    for (n = 0; n < NUM_CH; n++) {
        IngenicIspChan *c = &s->ch[p * NUM_CH + n];
        uint32_t *r = &s->isp[PIPE_CH(p, MSCA_CH(n)) >> 2];
        uint32_t size = r[l->csize >> 2] ? r[l->csize >> 2] :
                        r[l->osize >> 2];
        uint32_t w = size >> 16, h = size & 0xFFFF;
        uint32_t ya, uva;
        bool ivdc;

        if (!(en & (1u << n)) || !c->y.count || !c->uv.count ||
            !w || !h || c->done_y.count == FIFO_DEPTH) {
            continue;
        }
        ya = fifo_pop(&c->y, NULL);
        uva = fifo_pop(&c->uv, NULL);
        ivdc = ya == IVDC_Y;
        ingenic_isp_paint(&s->paint, frame, w, h, r[l->ystride >> 2],
                          r[l->uvstride >> 2], ya, uva, false, gain, sgain);
        if (p == 0 && ingenic_isp_hldc_on(s)) {
            ingenic_isp_hldc_plane(s, 0, w, h, r[l->ystride >> 2], ya);
            ingenic_isp_hldc_plane(s, 1, w, h, r[l->uvstride >> 2], uva);
        }
        if (ivdc) {
            /* a picture into the IVDC's store (RAM at IVDC_Y) */
            qemu_irq_pulse(s->ivdc_frame);
        }
        if (ivdc && l->ivdc_bit) {
            /*
             * T41: the ISR sends the pair to the frame source on ivdc_bit
             * and queues it again itself: nothing to dequeue.  T23's sends
             * it at frame start (fs_bit) and on channel 0's done bit pops
             * the done FIFO only to push the same pair back.
             */
            ISP_REG(l->int_sta) |= 1u << l->ivdc_bit;
        } else {
            fifo_push(&c->done_y, ya, c->frames);
            fifo_push(&c->done_uv, uva, c->frames);
        }
        c->frames++;
        ISP_REG(int_sta) |= 1u << n;
        if (s->trace) {
            qemu_log("ingenic-isp: pipe %d frame %u ch%d %ux%u -> "
                     "0x%08x/0x%08x\n", p, s->frame, p * NUM_CH + n, w, h,
                     ya, uva);
        }
    }
    if (l->ae) {
        ingenic_isp_stats(s, gain, p, frame);
    }
    if (l->xstat && p == 0) {
        ingenic_isp_xstats(s, gain, frame);
    }
    if (!l->no_vic) {
        vic[l->vic_sta >> 2] |= 1;
        ingenic_vic_mdma(s, p, frame);
    }
}

static void ingenic_isp_frame(void *opaque)
{
    IngenicIspState *s = opaque;
    int p;

    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        if (ingenic_isp_pipe_running(s, p)) {
            ingenic_isp_pipe_frame(s, p);
        }
    }
    if (s->l->pipes > 1 && s->vic2_running) {
        /*
         * T40's VIC2: the third sensor feeds no pipe (the driver's VIC2
         * frame start has no work to queue); its frames end, and its MDMA
         * can store them raw
         */
        s->vic2[s->l->vic_sta >> 2] |= 1;
        ingenic_vic_mdma(s, 2, s->frame + 2 * PIPE1_FRAME);
    }
    s->frame++;
    ingenic_isp_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ingenic_isp_period(s));
}

/* --- ISP core ------------------------------------------------------------ */

/* The channel (3 pipe + n) whose registers @off is in; -1: none */
static int ingenic_isp_chan(IngenicIspState *s, hwaddr off)
{
    int p;

    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        if (off >= PIPE_CH(p, MSCA_CH(0)) && off < PIPE_CH(p, MSCA_CH(NUM_CH))) {
            return p * NUM_CH + ((off - PIPE_CH(p, MSCA_CH(0))) >> 8);
        }
    }
    return -1;
}

/* channel @n's register block */
static hwaddr ingenic_isp_chan_base(IngenicIspState *s, int n)
{
    return PIPE_CH(n / NUM_CH, MSCA_CH(n % NUM_CH));
}

/* the pipe whose interrupt register @off is (one of @reg's copies); -1 */
static int ingenic_isp_pipe_reg(IngenicIspState *s, hwaddr off, hwaddr reg)
{
    int p;

    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        if (off == PIPE_INT(p, reg)) {
            return p;
        }
    }
    return -1;
}

static bool ingenic_isp_is_clear(const IngenicIspLayout *l, hwaddr r)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(l->clear); i++) {
        if (l->clear[i] && r == l->clear[i]) {
            return true;
        }
    }
    return false;
}

static uint64_t ingenic_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = s->l;
    int n = ingenic_isp_chan(s, off);
    IngenicIspChan *c;
    hwaddr r;

    if (off >= ISP_SIZE) {
        return 0;
    }
    if (l->ch_busy && off == l->ch_busy) {
        return 0;
    }
    if (l->ch_active && off == l->ch_active) {
        return ISP_REG(l->ch_en) & 7;
    }
    if (l->ack && off == l->ack) {
        return (s->isp[off >> 2] & ~1u) | (ISP_REG(l->req) & 1);
    }
    if (l->xstat == 3 && l->msca_win && (off & ~0xFFF) == 0x50000) {
        int pt;

        for (pt = 0; pt < ARRAY_SIZE(t41_ports); pt++) {
            if (off == t41_ports[pt].data) {
                int i = s->port_pos[pt]++;

                return i < t41_ports[pt].len ?
                       t41_port_ram(s, t41_ports[pt].ram)[i] : 0;
            }
        }
    }
    if (n < 0) {
        return s->isp[off >> 2];
    }
    c = &s->ch[n];
    r = off - ingenic_isp_chan_base(s, n);
    if (r == l->pop_y) {
        c->pop_uv = fifo_pop(&c->done_uv, NULL);
        return fifo_pop(&c->done_y, &c->pop_num);
    } else if (r == l->empty) {
        return !c->done_y.count;
    } else if (l->full && r == l->full) {
        return (c->y.count == FIFO_DEPTH) << 1;
    } else if (r == l->pop_uv) {
        return c->pop_uv;
    } else if (r && (r == l->pop_num[0] || r == l->pop_num[1])) {
        return c->pop_num;
    }
    return s->isp[off >> 2];
}

static void ingenic_isp_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = s->l;
    int n = ingenic_isp_chan(s, off);
    int p;

    if (off >= ISP_SIZE) {
        return;
    }
    if (s->trace_regs) {
        qemu_log("ingenic-isp: core [0x%05x] = 0x%08x\n", (unsigned)off,
                 (uint32_t)val);
    }
    if (s->trace && (n >= 0 ||
                     off == l->ch_en || (off & ~0x7ff) == 0xA000 ||
                     (off & ~0x7ff) == 0xB000 || (off & ~0xff) == 0x1800 ||
                     (off & ~0xff) == 0x18000 || (off & ~0xff) == 0x19000 ||
                     (off & ~0xff) == 0x4000)) {
        qemu_log("ingenic-isp: [0x%04x] = 0x%08x\n", (unsigned)off,
                 (uint32_t)val);
    }
    if (l->xstat == 3 && (off & ~0xFFF) == 0x50000) {
        int pt;

        for (pt = 0; pt < ARRAY_SIZE(t41_ports); pt++) {
            if (off == t41_ports[pt].ctl) {
                s->port_pos[pt] = 0;
            }
        }
    }
    if (ingenic_isp_pipe_reg(s, off, l->int_sta) >= 0) {
        return;
    }
    p = ingenic_isp_pipe_reg(s, off, l->int_clr);
    if (p >= 0) {
        ISP_REG(PIPE_INT(p, l->int_sta)) &= ~val;
        s->isp[off >> 2] = val;
        ingenic_isp_update(s);
        return;
    }
    if (n >= 0) {
        IngenicIspChan *c = &s->ch[n];
        hwaddr r = off - ingenic_isp_chan_base(s, n);

        if (r == l->push_y) {
            if (!fifo_push(&c->y, val, 0)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "ingenic-isp: ch%d Y FIFO full\n", n);
            }
        } else if (r == l->push_uv) {
            if (!fifo_push(&c->uv, val, 0)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "ingenic-isp: ch%d UV FIFO full\n", n);
            }
        } else if (r == l->pop_y || r == l->empty || r == l->pop_uv) {
            return;
        } else if (ingenic_isp_is_clear(l, r)) {
            if (val) {
                chan_clear(c);
            }
        }
    }
    s->isp[off >> 2] = val;
    for (p = 0; p < ingenic_isp_pipes(s); p++) {
        if (off == PIPE_INT(p, l->int_en) || off == PIPE_CH(p, l->ch_en)) {
            ingenic_isp_update(s);
            ingenic_isp_schedule(s);
        }
    }
}

/* --- VIC ------------------------------------------------------------------ */

static uint64_t ingenic_vic_rd(IngenicIspState *s, int v, hwaddr off)
{
    return ingenic_vic_regs(s, v)[off >> 2];
}

static void ingenic_vic_wr(IngenicIspState *s, int v, hwaddr off,
                           uint64_t val)
{
    const IngenicIspLayout *l = s->l;
    uint32_t *r = ingenic_vic_regs(s, v);
    int two = l->vic_sta2 ? 4 : 0;

    if (s->trace_regs) {
        qemu_log("ingenic-isp: vic%d [0x%04x] = 0x%08x\n", v, (unsigned)off,
                 (uint32_t)val);
    }
    if (off == VIC_CTRL) {
        /* 2, 4: reset and configure, done at once; 1: start */
        if (v == 0) {
            s->vic_running = val == 1;
        } else if (v == 1) {
            s->vic1_running = val == 1;
        } else {
            s->vic2_running = val == 1;
        }
        r[0] = 0;
        ingenic_isp_schedule(s);
        return;
    }
    if (off == l->vic_sta || off == l->vic_sta + two) {
        return;
    }
    if (off == l->vic_clr || off == l->vic_clr + two) {
        r[(l->vic_sta + off - l->vic_clr) >> 2] &= ~val;
    }
    if (off == VIC_MDMA_CTRL && (val & (1u << 31)) &&
        !(r[off >> 2] & (1u << 31))) {
        s->mdma_slot[v] = 0;            /* a capture starts at buffer 0 */
    }
    r[off >> 2] = val;
    ingenic_isp_update(s);
}

static uint64_t ingenic_vic_read(void *opaque, hwaddr off, unsigned size)
{
    return ingenic_vic_rd(opaque, 0, off);
}

static void ingenic_vic_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    ingenic_vic_wr(opaque, 0, off, val);
}

static uint64_t ingenic_vic1_read(void *opaque, hwaddr off, unsigned size)
{
    return ingenic_vic_rd(opaque, 1, off);
}

static void ingenic_vic1_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    ingenic_vic_wr(opaque, 1, off, val);
}

static uint64_t ingenic_vic2_read(void *opaque, hwaddr off, unsigned size)
{
    return ingenic_vic_rd(opaque, 2, off);
}

static void ingenic_vic2_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    ingenic_vic_wr(opaque, 2, off, val);
}

/* --- CSI host, D-PHY: storage --------------------------------------------- */

static uint64_t ingenic_csi_read(void *opaque, hwaddr off, unsigned size)
{
    uint32_t *r = opaque;

    return r[off >> 2];
}

static void ingenic_csi_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    uint32_t *r = opaque;

    r[off >> 2] = val;
}

#define INGENIC_ISP_OPS(rd, wr) {                   \
    .read = rd,                                     \
    .write = wr,                                    \
    .endianness = DEVICE_LITTLE_ENDIAN,             \
    .impl.min_access_size = 4,                      \
    .impl.max_access_size = 4,                      \
    .valid.min_access_size = 1,                     \
    .valid.max_access_size = 4,                     \
}

/* T41's scaler window: the core's registers from msca_win */
static uint64_t ingenic_msca_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = opaque;

    return ingenic_isp_read(s, s->l->msca_win + off, size);
}

static void ingenic_msca_write(void *opaque, hwaddr off, uint64_t val,
                               unsigned size)
{
    IngenicIspState *s = opaque;

    ingenic_isp_write(s, s->l->msca_win + off, val, size);
}

static const MemoryRegionOps ingenic_isp_ops =
    INGENIC_ISP_OPS(ingenic_isp_read, ingenic_isp_write);
static const MemoryRegionOps ingenic_vic_ops =
    INGENIC_ISP_OPS(ingenic_vic_read, ingenic_vic_write);
static const MemoryRegionOps ingenic_vic1_ops =
    INGENIC_ISP_OPS(ingenic_vic1_read, ingenic_vic1_write);
static const MemoryRegionOps ingenic_vic2_ops =
    INGENIC_ISP_OPS(ingenic_vic2_read, ingenic_vic2_write);
static const MemoryRegionOps ingenic_msca_ops =
    INGENIC_ISP_OPS(ingenic_msca_read, ingenic_msca_write);
static const MemoryRegionOps ingenic_csi_ops =
    INGENIC_ISP_OPS(ingenic_csi_read, ingenic_csi_write);

/* --- device ---------------------------------------------------------------- */

static void ingenic_isp_reset(DeviceState *dev)
{
    IngenicIspState *s = INGENIC_ISP(dev);
    int n;

    timer_del(s->timer);
    memset(s->isp, 0, sizeof(s->isp));
    memset(s->vic, 0, sizeof(s->vic));
    memset(s->vic1, 0, sizeof(s->vic1));
    memset(s->vic2, 0, sizeof(s->vic2));
    memset(s->csi, 0, sizeof(s->csi));
    memset(s->phy, 0, sizeof(s->phy));
    s->vic_running = false;
    s->vic1_running = false;
    s->vic2_running = false;
    memset(s->mdma_slot, 0, sizeof(s->mdma_slot));
    s->frame = 0;
    for (n = 0; n < NUM_CH * MAX_PIPES; n++) {
        chan_clear(&s->ch[n]);
        s->ch[n].pop_uv = s->ch[n].pop_num = s->ch[n].frames = 0;
    }
    ingenic_isp_update(s);
}

static void ingenic_isp_realize(DeviceState *dev, Error **errp)
{
    IngenicIspState *s = INGENIC_ISP(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(ingenic_isp_layouts); i++) {
        if (!strcmp(s->soc ? s->soc : "t31", ingenic_isp_layouts[i].name)) {
            s->l = &ingenic_isp_layouts[i];
            if (!s->light_r) {
                s->light_r = s->l->day_r ? s->l->day_r : 550;
            }
            if (!s->light_b) {
                s->light_b = s->l->day_b ? s->l->day_b : 600;
            }
        }
    }
    if (!s->l) {
        error_setg(errp, "ingenic-isp: no layout for soc '%s'", s->soc);
        return;
    }
    memory_region_set_size(&s->isp_mr, s->l->isp_size);
}

static void ingenic_isp_init(Object *obj)
{
    IngenicIspState *s = INGENIC_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->isp_mr, obj, &ingenic_isp_ops, s,
                          "ingenic-isp.core", ISP_SIZE);
    memory_region_init_io(&s->vic_mr, obj, &ingenic_vic_ops, s,
                          "ingenic-isp.vic", VIC_SIZE);
    memory_region_init_io(&s->csi_mr, obj, &ingenic_csi_ops, s->csi,
                          "ingenic-isp.csi", CSI_SIZE);
    memory_region_init_io(&s->phy_mr, obj, &ingenic_csi_ops, s->phy,
                          "ingenic-isp.dphy", PHY_SIZE);
    sysbus_init_mmio(sbd, &s->isp_mr);
    sysbus_init_mmio(sbd, &s->vic_mr);
    sysbus_init_mmio(sbd, &s->csi_mr);
    sysbus_init_mmio(sbd, &s->phy_mr);
    memory_region_init_io(&s->msca_mr, obj, &ingenic_msca_ops, s,
                          "ingenic-isp.msca", MSCA_WIN_SIZE);
    sysbus_init_mmio(sbd, &s->msca_mr);
    /* T40: VIC1 and VIC2 (mmio 5, 6; IRQs 2, 3) */
    memory_region_init_io(&s->vic1_mr, obj, &ingenic_vic1_ops, s,
                          "ingenic-isp.vic1", VIC_SIZE);
    memory_region_init_io(&s->vic2_mr, obj, &ingenic_vic2_ops, s,
                          "ingenic-isp.vic2", VIC_SIZE);
    sysbus_init_mmio(sbd, &s->vic1_mr);
    sysbus_init_mmio(sbd, &s->vic2_mr);
    sysbus_init_irq(sbd, &s->isp_irq);
    sysbus_init_irq(sbd, &s->vic_irq);
    sysbus_init_irq(sbd, &s->vic1_irq);
    sysbus_init_irq(sbd, &s->vic2_irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->ivdc_frame, "ivdc-frame", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ingenic_isp_frame, s);
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "sensor2", TYPE_DEVICE, &s->sensor2,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    object_property_add_link(obj, "sensor1", TYPE_DEVICE, &s->sensor1,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    /* the scene's light, percent; qom-set while running */
    s->light = 100;
    object_property_add_uint32_ptr(obj, "light", &s->light,
                                   OBJ_PROP_FLAG_READWRITE);
    /* and its colour, for the AWB statistics; 0: the layout's daylight */
    s->light_r = 0;
    s->light_b = 0;
    object_property_add_uint32_ptr(obj, "light-r", &s->light_r,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-b", &s->light_b,
                                   OBJ_PROP_FLAG_READWRITE);
}

static int ingenic_isp_post_load(void *opaque, int version_id)
{
    IngenicIspState *s = opaque;
    int n;

    for (n = 0; n < NUM_CH * MAX_PIPES; n++) {
        IngenicIspChan *c = &s->ch[n];

        if (c->y.head >= FIFO_DEPTH || c->y.count > FIFO_DEPTH ||
            c->uv.head >= FIFO_DEPTH || c->uv.count > FIFO_DEPTH ||
            c->done_y.head >= FIFO_DEPTH || c->done_y.count > FIFO_DEPTH ||
            c->done_uv.head >= FIFO_DEPTH ||
            c->done_uv.count > FIFO_DEPTH) {
            return -EINVAL;
        }
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_isp_fifo = {
    .name = "ingenic-isp-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(a, IngenicIspFifo, FIFO_DEPTH),
        VMSTATE_UINT32_ARRAY(b, IngenicIspFifo, FIFO_DEPTH),
        VMSTATE_UINT32(head, IngenicIspFifo),
        VMSTATE_UINT32(count, IngenicIspFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_ingenic_isp_chan = {
    .name = "ingenic-isp-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(y, IngenicIspChan, 1, vmstate_ingenic_isp_fifo,
                       IngenicIspFifo),
        VMSTATE_STRUCT(uv, IngenicIspChan, 1, vmstate_ingenic_isp_fifo,
                       IngenicIspFifo),
        VMSTATE_STRUCT(done_y, IngenicIspChan, 1, vmstate_ingenic_isp_fifo,
                       IngenicIspFifo),
        VMSTATE_STRUCT(done_uv, IngenicIspChan, 1, vmstate_ingenic_isp_fifo,
                       IngenicIspFifo),
        VMSTATE_UINT32(pop_uv, IngenicIspChan),
        VMSTATE_UINT32(pop_num, IngenicIspChan),
        VMSTATE_UINT32(frames, IngenicIspChan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_ingenic_isp = {
    .name = TYPE_INGENIC_ISP,
    .version_id = 4,
    .minimum_version_id = 1,
    .post_load = ingenic_isp_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(isp, IngenicIspState, ISP_SIZE / 4),
        VMSTATE_UINT32_ARRAY(vic, IngenicIspState, VIC_SIZE / 4),
        VMSTATE_UINT32_ARRAY(csi, IngenicIspState, CSI_SIZE / 4),
        VMSTATE_UINT32_ARRAY(phy, IngenicIspState, PHY_SIZE / 4),
        VMSTATE_BOOL(vic_running, IngenicIspState),
        VMSTATE_UINT32(frame, IngenicIspState),
        VMSTATE_UINT32_V(light, IngenicIspState, 2),
        VMSTATE_UINT32_V(light_r, IngenicIspState, 2),
        VMSTATE_UINT32_V(light_b, IngenicIspState, 2),
        VMSTATE_STRUCT_SUB_ARRAY(ch, IngenicIspState, 0, NUM_CH, 1,
                                 vmstate_ingenic_isp_chan, IngenicIspChan),
        VMSTATE_STRUCT_SUB_ARRAY(ch, IngenicIspState, NUM_CH, NUM_CH, 3,
                                 vmstate_ingenic_isp_chan, IngenicIspChan),
        VMSTATE_UINT32_ARRAY_V(vic1, IngenicIspState, VIC_SIZE / 4, 3),
        VMSTATE_UINT32_ARRAY_V(vic2, IngenicIspState, VIC_SIZE / 4, 3),
        VMSTATE_BOOL_V(vic1_running, IngenicIspState, 3),
        VMSTATE_BOOL_V(vic2_running, IngenicIspState, 4),
        VMSTATE_UINT8_ARRAY_V(mdma_slot, IngenicIspState, 3, 4),
        VMSTATE_TIMER_PTR(timer, IngenicIspState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ingenic_isp_properties[] = {
    DEFINE_PROP_UINT32("fps", IngenicIspState, fps, 25),
    DEFINE_PROP_STRING("soc", IngenicIspState, soc),
    DEFINE_PROP_BOOL("trace", IngenicIspState, trace, false),
    DEFINE_PROP_BOOL("trace-regs", IngenicIspState, trace_regs, false),
};

static void ingenic_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, ingenic_isp_reset);
    dc->realize = ingenic_isp_realize;
    device_class_set_props(dc, ingenic_isp_properties);
    dc->vmsd = &vmstate_ingenic_isp;
}

static const TypeInfo ingenic_isp_info = {
    .name          = TYPE_INGENIC_ISP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIspState),
    .instance_init = ingenic_isp_init,
    .class_init    = ingenic_isp_class_init,
};

static void ingenic_isp_register_types(void)
{
    type_register_static(&ingenic_isp_info);
}

type_init(ingenic_isp_register_types)
