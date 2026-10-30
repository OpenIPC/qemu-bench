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

#define NUM_CH          3
#define FIFO_DEPTH      16
#define IVDC_Y          0x80000000u   /* T41 direct_mode's channel-0 pair */

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
    uint8_t ivdc_bit;           /* T41's IVDC: frame start of a channel
                                   writing to the IVDC, in int_sta; 0: none */
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
        .wb = 0x1804,
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
        .wb = 0x4004,
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
        .name = "t41", .isp_size = 0x80000, .msca_win = 0xF0000,
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
        .wb = 0x4004,
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
    qemu_irq isp_irq, vic_irq;
    QEMUTimer *timer;

    uint32_t fps;
    char *soc;
    bool trace;
    uint32_t light;             /* the scene's light, percent */
    uint32_t light_r, light_b;  /* its colour: the raw R and B of a white
                                   over its G, per mille */
    Object *sensor;             /* the image sensor, for its exposure */
    const IngenicIspLayout *l;

    uint32_t isp[ISP_SIZE / 4];
    uint32_t vic[VIC_SIZE / 4];
    uint32_t csi[CSI_SIZE / 4];
    uint32_t phy[PHY_SIZE / 4];
    bool vic_running;
    uint32_t frame;
    IngenicIspChan ch[NUM_CH];

    IngenicIspPainter paint;
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

static void ingenic_isp_update(IngenicIspState *s)
{
    const IngenicIspLayout *l = s->l;
    uint32_t vic = VIC_REG(l->vic_sta) & ~VIC_REG(l->vic_mask);
    uint32_t en = l->int_mask ? ~ISP_REG(l->int_en) : ISP_REG(l->int_en);

    if (l->vic_sta2) {
        vic |= VIC_REG(l->vic_sta + 4) & ~VIC_REG(l->vic_mask + 4);
    }
    qemu_set_irq(s->isp_irq, !!(ISP_REG(l->int_sta) & en));
    qemu_set_irq(s->vic_irq, !l->no_vic && vic);
}

static int64_t ingenic_isp_period(IngenicIspState *s)
{
    return NANOSECONDS_PER_SECOND / MAX(s->fps, 1);
}

static void ingenic_isp_schedule(IngenicIspState *s)
{
    bool run = s->l->no_vic ? ISP_REG(s->l->ch_en) & 7 :
               s->vic_running && ISP_REG(s->l->int_en);

    if (run && !timer_pending(s->timer)) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ingenic_isp_period(s));
    } else if (!run) {
        timer_del(s->timer);
    }
}

/* --- the scene's light, 3A statistics ------------------------------------- */

/* The sensor's exposure, rows x gain (1.0 = 1024); 0 if there is none */
static uint64_t ingenic_isp_sensor_level(IngenicIspState *s)
{
    IngenicSensorExposure e;
    HisiSoiExposure soi;

    if (!s->sensor) {
        return 0;
    }
    if (galaxycore_exposure(s->sensor, &e) ||
        smartsens_exposure(s->sensor, &e)) {
        return (uint64_t)e.lines * e.gain;
    }
    if (hisi_soi_exposure(s->sensor, &soi)) {
        return (uint64_t)soi.lines * soi.gain;
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

static uint32_t ingenic_isp_gain(IngenicIspState *s)
{
    uint64_t g = (uint64_t)s->light * 256 / 100;
    uint64_t level = s->l->ae ? ingenic_isp_sensor_level(s) : 0;

    if (level) {
        g = g * level / EXPOSURE_1X;
    }
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
 * light, in the sensor's linear raw light (isp_lin()) and colour: a
 * white reads R "light-r" and B "light-b" per mille of G (0.55 and 0.6
 * by default, as under daylight).
 * Each bank done raises its int_sta bit.
 */
#define STAT_ZONES      15
#define STAT_SX         8
#define STAT_SY         6

typedef struct IspZone {
    uint64_t sum[3], cnt[3];    /* AE: dark, middle, bright */
    uint64_t rgb[3], n;         /* AWB */
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

static void ingenic_isp_stats(IngenicIspState *s, uint32_t gain)
{
    const IngenicIspLayout *l = s->l;
    static IspZone zone[STAT_ZONES][STAT_ZONES];
    uint64_t hist[256];
    uint8_t buf[STAT_ZONES * STAT_ZONES * 16];
    static uint8_t abuf[STAT_ZONES * STAT_ZONES * 128];
    int cols, rows, xs[STAT_ZONES + 1], ys[STAT_ZONES + 1];
    int acols, arows, axs[STAT_ZONES + 1], ays[STAT_ZONES + 1];
    uint32_t thr = ISP_REG(l->ae + 0x28);
    int lo = thr & 0xFF, hi = (thr >> 8) & 0xFF;
    /* T21 has no enable bits: a block runs once it has its banks */
    bool en_bits = l->stat_gen != 3;
    bool ae = (!en_bits || (ISP_REG(l->ae) & 1)) && ISP_REG(l->ae + 0x2C);
    bool awb = (!en_bits || (ISP_REG(l->awb) & 1)) && ISP_REG(l->awb + 0x3C);
    int zx, zy, i, j, w, h;

    if (!ae && !awb) {
        return;
    }
    if (!isp_grid(s, l->ae, &cols, &rows, xs, ys)) {
        ae = false;
    }
    if (!isp_grid(s, l->awb, &acols, &arows, axs, ays)) {
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
                    int y, ch;

                    sstar_pattern_pixel(s->frame, w, h, 2 * qx, 2 * qy, yuv);
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
                    rgb[0] = (yn + 1.402 * cr) * s->light_r / 1000;
                    rgb[1] = yn - 0.344 * cb - 0.714 * cr;
                    rgb[2] = (yn + 1.772 * cb) * s->light_b / 1000;
                    for (ch = 0; ch < 3; ch++) {
                        double v = pow(MIN(MAX(rgb[ch], 0), 1), 2.2) * k;

                        z->rgb[ch] += lround(MIN(v, 1) * 255);
                    }
                    z->n++;
                }
            }
            /* from the samples to the zone's quads */
            for (c = 0; c < 3; c++) {
                z->sum[c] = z->sum[c] * quads / (STAT_SX * STAT_SY);
                z->cnt[c] = z->cnt[c] * quads / (STAT_SX * STAT_SY);
                z->rgb[c] = z->rgb[c] * quads / (STAT_SX * STAT_SY);
            }
            z->n = quads;
        }
    }
    if (ae) {
        uint32_t cnt = ISP_REG(l->ae + 0x4C), idx = ISP_REG(l->ae + 0x50);
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
                         ISP_REG(l->ae + 0x2C + 4 * zb), buf,
                         16 * rows * cols, MEMTXATTRS_UNSPECIFIED);
        for (i = 0; i < 256; i++) {
            hw[i] = hw[256 + i] = cpu_to_le32(MIN(hist[i] / (STAT_SX *
                                                             STAT_SY),
                                                  l->stat_gen == 3 ?
                                                  0xFFFFF : 0x1FFFFF));
        }
        dma_memory_write(&address_space_memory,
                         ISP_REG(l->ae + 0x3C + 4 * hb), hw, sizeof(hw),
                         MEMTXATTRS_UNSPECIFIED);
        ISP_REG(l->ae + 0x50) = zb << 4 | hb;
        ISP_REG(l->int_sta) |= 1u << l->ae_static_bit | 1u << l->ae_hist_bit;
    }
    if (awb) {
        uint32_t cnt = ISP_REG(l->awb + 0x4C), idx = ISP_REG(l->awb + 0x50);
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
                    for (i = 0; i < 8; i++) {
                        isp_bits(o + 16 * i, 0, 22, z->rgb[0]);
                        isp_bits(o + 16 * i, 22, 22, z->rgb[1]);
                        isp_bits(o + 16 * i, 44, 22, z->rgb[2]);
                        isp_bits(o + 16 * i, 88, 14, z->n);
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
                         ISP_REG(l->awb + 0x3C + 4 * ab), abuf,
                         zs * arows * acols, MEMTXATTRS_UNSPECIFIED);
        ISP_REG(l->awb + 0x50) = ab;
        ISP_REG(l->int_sta) |= 1u << l->awb_bit;
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
        qemu_log("ingenic-isp: frame %u stats gain %u/256 mean Y %u%s%s"
                 " wb R %u B %u /1024\n",
                 s->frame, gain, n ? (unsigned)(sum / n) : 0,
                 ae ? " ae" : "", awb ? " awb" : "",
                 l->wb ? ISP_REG(l->wb) & 0xFFF : 0,
                 l->wb ? ISP_REG(l->wb + 4) & 0xFFF : 0);
    }
}

static void ingenic_isp_frame(void *opaque)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = s->l;
    uint32_t en = ISP_REG(l->ch_en);
    uint32_t gain = ingenic_isp_gain(s);
    int n;

    ISP_REG(l->int_sta) |= l->fs_bit;
    if (!l->no_vic) {
        VIC_REG(l->vic_sta) |= l->vic_fs_bit;
    }
    for (n = 0; n < NUM_CH; n++) {
        IngenicIspChan *c = &s->ch[n];
        uint32_t *r = &s->isp[MSCA_CH(n) >> 2];
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
        ivdc = l->ivdc_bit && ya == IVDC_Y;
        ingenic_isp_paint(&s->paint, s->frame, w, h, r[l->ystride >> 2],
                          r[l->uvstride >> 2], ya, uva, false, gain);
        if (ivdc) {
            /* into the IVDC's store (RAM at IVDC_Y), nothing to dequeue */
            ISP_REG(l->int_sta) |= 1u << l->ivdc_bit;
        } else {
            fifo_push(&c->done_y, ya, c->frames);
            fifo_push(&c->done_uv, uva, c->frames);
        }
        c->frames++;
        ISP_REG(l->int_sta) |= 1u << n;
        if (s->trace) {
            qemu_log("ingenic-isp: frame %u ch%d %ux%u -> 0x%08x/0x%08x\n",
                     s->frame, n, w, h, ya, uva);
        }
    }
    if (l->ae) {
        ingenic_isp_stats(s, gain);
    }
    if (!l->no_vic) {
        VIC_REG(l->vic_sta) |= 1;
    }
    s->frame++;
    ingenic_isp_update(s);
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ingenic_isp_period(s));
}

/* --- ISP core ------------------------------------------------------------ */

static int ingenic_isp_chan(IngenicIspState *s, hwaddr off)
{
    if (off < MSCA_CH(0) || off >= MSCA_CH(NUM_CH)) {
        return -1;
    }
    return (off - s->l->ch_base) >> 8;
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
    if (n < 0) {
        return s->isp[off >> 2];
    }
    c = &s->ch[n];
    r = off - MSCA_CH(n);
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

    if (off >= ISP_SIZE) {
        return;
    }
    if (s->trace && ((off >= MSCA_CH(0) && off < MSCA_CH(NUM_CH)) ||
                     off == l->ch_en || (off & ~0x7ff) == 0xA000 ||
                     (off & ~0x7ff) == 0xB000 || (off & ~0xff) == 0x1800 ||
                     (off & ~0xff) == 0x18000 || (off & ~0xff) == 0x19000 ||
                     (off & ~0xff) == 0x4000)) {
        qemu_log("ingenic-isp: [0x%04x] = 0x%08x\n", (unsigned)off,
                 (uint32_t)val);
    }
    if (off == l->int_sta) {
        return;
    }
    if (off == l->int_clr) {
        ISP_REG(l->int_sta) &= ~val;
        s->isp[off >> 2] = val;
        ingenic_isp_update(s);
        return;
    }
    if (n >= 0) {
        IngenicIspChan *c = &s->ch[n];
        hwaddr r = off - MSCA_CH(n);

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
    if (off == l->int_en || off == l->ch_en) {
        ingenic_isp_update(s);
        ingenic_isp_schedule(s);
    }
}

/* --- VIC ------------------------------------------------------------------ */

static uint64_t ingenic_vic_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = opaque;

    return s->vic[off >> 2];
}

static void ingenic_vic_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = s->l;
    int two = l->vic_sta2 ? 4 : 0;

    if (off == VIC_CTRL) {
        /* 2, 4: reset and configure, done at once; 1: start */
        s->vic_running = val == 1;
        s->vic[0] = 0;
        ingenic_isp_schedule(s);
        return;
    }
    if (off == l->vic_sta || off == l->vic_sta + two) {
        return;
    }
    if (off == l->vic_clr || off == l->vic_clr + two) {
        VIC_REG(l->vic_sta + off - l->vic_clr) &= ~val;
    }
    s->vic[off >> 2] = val;
    ingenic_isp_update(s);
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
    memset(s->csi, 0, sizeof(s->csi));
    memset(s->phy, 0, sizeof(s->phy));
    s->vic_running = false;
    s->frame = 0;
    for (n = 0; n < NUM_CH; n++) {
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
    sysbus_init_irq(sbd, &s->isp_irq);
    sysbus_init_irq(sbd, &s->vic_irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ingenic_isp_frame, s);
    object_property_add_link(obj, "sensor", TYPE_DEVICE, &s->sensor,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG);
    /* the scene's light, percent; qom-set while running */
    s->light = 100;
    object_property_add_uint32_ptr(obj, "light", &s->light,
                                   OBJ_PROP_FLAG_READWRITE);
    /* and its colour, for the AWB statistics */
    s->light_r = 550;
    s->light_b = 600;
    object_property_add_uint32_ptr(obj, "light-r", &s->light_r,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "light-b", &s->light_b,
                                   OBJ_PROP_FLAG_READWRITE);
}

static int ingenic_isp_post_load(void *opaque, int version_id)
{
    IngenicIspState *s = opaque;
    int n;

    for (n = 0; n < NUM_CH; n++) {
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
    .version_id = 2,
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
        VMSTATE_STRUCT_ARRAY(ch, IngenicIspState, NUM_CH, 1,
                             vmstate_ingenic_isp_chan, IngenicIspChan),
        VMSTATE_TIMER_PTR(timer, IngenicIspState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ingenic_isp_properties[] = {
    DEFINE_PROP_UINT32("fps", IngenicIspState, fps, 25),
    DEFINE_PROP_STRING("soc", IngenicIspState, soc),
    DEFINE_PROP_BOOL("trace", IngenicIspState, trace, false),
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
