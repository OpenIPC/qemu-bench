/*
 * hisi-v5-venc-userframe: drive the HiSilicon V5 (Hi3516CV610) encoder
 * (VEDU + JPGE) with frames from a VB pool, so the encoder is tested
 * without VI or VPSS -- the V5 counterpart of hisi-venc-userframe.c.
 *
 * The call order is the SDK sample_venc's (sample_comm_venc_start, then
 * get_stream), with ss_mpi_venc_send_frame where the sample binds a VPSS
 * channel.  Each frame is NV21 (OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420):
 * moving vertical bars plus the frame number as a row of 16 binary
 * squares, which the host checks after decoding (check-hisi-venc.py).
 *
 *   hisi-v5-venc-userframe [-w W] [-h H] [-n FRAMES] [-o DIR] [h264|h265|jpeg]...
 *
 * Streams go to DIR/venc-<codec>.<ext>.  Prints one line per frame and
 * "RESULT: PASS" when every channel produced every frame.
 *
 * Built against the Hi3516CV610 SDK's headers and static MPI libraries
 * (qemu-boot/test-hisi-v5-venc.sh), which are not redistributable: a
 * local tool for tracing the encoder's contract, not a CI test.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "ot_common.h"
#include "ot_math.h"
#include "ot_common_vb.h"
#include "ot_common_video.h"
#include "ot_common_venc.h"
#include "ss_mpi_sys.h"
#include "ss_mpi_sys_mem.h"
#include "ss_mpi_vb.h"
#include "ss_mpi_venc.h"

#define MAX_CHN 3

typedef struct {
    ot_payload_type type;
    const char *name, *ext;
    FILE *out;
    unsigned frames, bytes;
} Chn;

static unsigned W = 640, H = 360, N = 10;

/* bars that move 8 pixels a frame, and the frame number in 16 squares */
static void paint(uint8_t *y, uint8_t *vu, unsigned stride, unsigned n)
{
    unsigned r, c, sq = W / 16;

    for (r = 0; r < H; r++) {
        for (c = 0; c < W; c++) {
            uint8_t v = (((c + n * 8) / 32) & 1) ? 200 : 60;

            if (r < sq) {
                v = (n >> (15 - c / sq)) & 1 ? 235 : 16;
            }
            y[r * stride + c] = v;
        }
    }
    memset(vu, 128, stride * H / 2);
}

static int chn_create(int ch, Chn *c)
{
    ot_venc_chn_attr a;
    ot_venc_start_param sp;
    int ret;

    memset(&a, 0, sizeof(a));
    a.venc_attr.type = c->type;
    a.venc_attr.max_pic_width = a.venc_attr.pic_width = W;
    a.venc_attr.max_pic_height = a.venc_attr.pic_height = H;
    a.venc_attr.profile = 0;
    a.venc_attr.is_by_frame = TD_TRUE;
    if (c->type == OT_PT_JPEG) {
        a.venc_attr.buf_size = OT_ALIGN_UP(W, 16) * OT_ALIGN_UP(H, 16) * 4;
        a.venc_attr.jpeg_attr.recv_mode = OT_VENC_PIC_RECV_SINGLE;
        a.gop_attr.gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
    } else {
        a.venc_attr.buf_size = OT_ALIGN_UP(W * H * 3 / 4, 64);
        a.gop_attr.gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
        a.gop_attr.normal_p.ip_qp_delta = 2;
        if (c->type == OT_PT_H264) {
            a.venc_attr.h264_attr.frame_buf_ratio = 70;
            a.rc_attr.rc_mode = OT_VENC_RC_MODE_H264_CBR;
            a.rc_attr.h264_cbr.gop = 30;
            a.rc_attr.h264_cbr.stats_time = 1;
            a.rc_attr.h264_cbr.src_frame_rate = 30;
            a.rc_attr.h264_cbr.dst_frame_rate = 30;
            a.rc_attr.h264_cbr.bit_rate = 1024;
        } else {
            a.venc_attr.h265_attr.frame_buf_ratio = 70;
            a.rc_attr.rc_mode = OT_VENC_RC_MODE_H265_CBR;
            a.rc_attr.h265_cbr.gop = 30;
            a.rc_attr.h265_cbr.stats_time = 1;
            a.rc_attr.h265_cbr.src_frame_rate = 30;
            a.rc_attr.h265_cbr.dst_frame_rate = 30;
            a.rc_attr.h265_cbr.bit_rate = 1024;
        }
    }
    ret = ss_mpi_venc_create_chn(ch, &a);
    if (ret) {
        printf("venc_create_chn %d (%s): %#x\n", ch, c->name, ret);
        return ret;
    }
    sp.recv_pic_num = -1;
    ret = ss_mpi_venc_start_chn(ch, &sp);
    if (ret) {
        printf("venc_start_chn %d: %#x\n", ch, ret);
    }
    return ret;
}

/* everything the channel has, into its file */
static int chn_drain(int ch, Chn *c, int ms)
{
    ot_venc_chn_status st;
    ot_venc_stream s;
    unsigned i;
    int ret;

    ret = ss_mpi_venc_query_status(ch, &st);
    if (ret || !st.cur_packs) {
        return 0;
    }
    memset(&s, 0, sizeof(s));
    s.pack = calloc(st.cur_packs, sizeof(ot_venc_pack));
    s.pack_cnt = st.cur_packs;
    ret = ss_mpi_venc_get_stream(ch, &s, ms);
    if (ret) {
        free(s.pack);
        return 0;
    }
    for (i = 0; i < s.pack_cnt; i++) {
        ot_venc_pack *p = &s.pack[i];

        fwrite(p->addr + p->offset, 1, p->len - p->offset, c->out);
        c->bytes += p->len - p->offset;
    }
    c->frames++;
    printf("%s frame %u: %u packs, seq %u\n", c->name, c->frames,
           s.pack_cnt, s.seq);
    ss_mpi_venc_release_stream(ch, &s);
    free(s.pack);
    return 1;
}

int main(int argc, char **argv)
{
    static const struct { const char *n, *e; ot_payload_type t; } codecs[] = {
        { "h264", "h264", OT_PT_H264 },
        { "h265", "h265", OT_PT_H265 },
        { "jpeg", "jpg", OT_PT_JPEG },
    };
    Chn chn[MAX_CHN];
    const char *dir = ".";
    unsigned nch = 0, stride, size, f, i, pass = 1;
    ot_vb_cfg vb;
    ot_vb_blk blk;
    td_phys_addr_t phys;
    uint8_t *virt;
    int opt;

    while ((opt = getopt(argc, argv, "w:h:n:o:")) != -1) {
        switch (opt) {
        case 'w': W = atoi(optarg); break;
        case 'h': H = atoi(optarg); break;
        case 'n': N = atoi(optarg); break;
        case 'o': dir = optarg; break;
        default: return 2;
        }
    }
    for (; optind < argc && nch < MAX_CHN; optind++) {
        for (i = 0; i < 3; i++) {
            if (!strcmp(argv[optind], codecs[i].n)) {
                char path[256];

                memset(&chn[nch], 0, sizeof(chn[nch]));
                chn[nch].type = codecs[i].t;
                chn[nch].name = codecs[i].n;
                snprintf(path, sizeof(path), "%s/venc-%s.%s", dir,
                         codecs[i].n, codecs[i].e);
                chn[nch].out = fopen(path, "wb");
                nch++;
            }
        }
    }
    if (!nch) {
        chn[0] = (Chn){ OT_PT_H264, "h264", "h264" };
        chn[0].out = fopen("venc-h264.h264", "wb");
        nch = 1;
    }

    stride = OT_ALIGN_UP(W, 16);
    size = stride * H * 3 / 2;
    ss_mpi_sys_exit();
    ss_mpi_vb_exit();
    memset(&vb, 0, sizeof(vb));
    vb.max_pool_cnt = 1;
    vb.common_pool[0].blk_size = size;
    vb.common_pool[0].blk_cnt = 4;
    if (ss_mpi_vb_set_cfg(&vb) || ss_mpi_vb_init() || ss_mpi_sys_init()) {
        printf("sys / vb init failed\nRESULT: FAIL\n");
        return 1;
    }
    for (i = 0; i < nch; i++) {
        if (chn_create(i, &chn[i])) {
            printf("RESULT: FAIL\n");
            return 1;
        }
    }

    blk = ss_mpi_vb_get_blk(OT_VB_INVALID_POOL_ID, size, NULL);
    if (blk == OT_VB_INVALID_HANDLE) {
        printf("vb_get_blk failed\nRESULT: FAIL\n");
        return 1;
    }
    phys = ss_mpi_vb_handle_to_phys_addr(blk);
    virt = ss_mpi_sys_mmap(phys, size);
    for (f = 0; f < N; f++) {
        ot_video_frame_info fi;

        paint(virt, virt + stride * H, stride, f);
        memset(&fi, 0, sizeof(fi));
        fi.pool_id = ss_mpi_vb_handle_to_pool_id(blk);
        fi.mod_id = OT_ID_VENC;
        fi.video_frame.width = W;
        fi.video_frame.height = H;
        fi.video_frame.field = OT_VIDEO_FIELD_FRAME;
        fi.video_frame.pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        fi.video_frame.video_format = OT_VIDEO_FORMAT_LINEAR;
        fi.video_frame.compress_mode = OT_COMPRESS_MODE_NONE;
        fi.video_frame.dynamic_range = OT_DYNAMIC_RANGE_SDR8;
        fi.video_frame.stride[0] = fi.video_frame.stride[1] = stride;
        fi.video_frame.phys_addr[0] = phys;
        fi.video_frame.phys_addr[1] = phys + stride * H;
        fi.video_frame.virt_addr[0] = virt;
        fi.video_frame.virt_addr[1] = virt + stride * H;
        fi.video_frame.time_ref = f * 2;
        fi.video_frame.pts = f * 33333ULL;
        for (i = 0; i < nch; i++) {
            int ret = ss_mpi_venc_send_frame(i, &fi, 1000);

            if (ret) {
                printf("%s send_frame %u: %#x\n", chn[i].name, f, ret);
            }
        }
        for (i = 0; i < nch; i++) {
            chn_drain(i, &chn[i], 2000);
        }
    }
    for (i = 0; i < nch; i++) {
        while (chn[i].frames < N && chn_drain(i, &chn[i], 1000)) {
        }
        printf("%s: %u frames, %u bytes\n", chn[i].name, chn[i].frames,
               chn[i].bytes);
        pass &= chn[i].frames == N;
        ss_mpi_venc_stop_chn(i);
        ss_mpi_venc_destroy_chn(i);
        fclose(chn[i].out);
    }
    ss_mpi_sys_munmap(virt, size);
    ss_mpi_vb_release_blk(blk);
    ss_mpi_sys_exit();
    ss_mpi_vb_exit();
    printf("RESULT: %s\n", pass ? "PASS" : "FAIL");
    return !pass;
}
