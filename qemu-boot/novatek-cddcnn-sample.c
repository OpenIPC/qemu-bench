/*
 * The AI turnkey's pet (cat and dog) detector, cddcnn, on a list of NV12
 * pictures: SDK v1.01.007 has its library (vendor/third_party/cddcnn_ai2)
 * but no sample, so this is one, after the turnkey guide's description of
 * the flow (NT9856x AI Turnkey, 2.1.3 and 2.2.3) and the other detectors'
 * samples.  qemu-boot/build-novatek-hdal-samples.sh builds it with the SDK.
 *
 * It reads /mnt/sd/jpg/cdd_image_list.txt -- "name width height" a line,
 * the pictures in /mnt/sd/jpg/CDD -- and writes the boxes, in the
 * picture's pixels, as "name category score x1 y1 x2 y2": those the
 * network proposes to /mnt/sd/det_results/cdd_raw_results.txt, those
 * limit_fdet's classifier confirms to cdd_test_results.txt.  The models
 * are the turnkey's CNNLib/para/cddcnn (process 3) and CNNLib/para/public
 * (process 1, the 7-class network limit_fdet rejects false boxes with).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "hdal.h"
#include "hd_type.h"
#include "vendor_ai.h"
#include "vendor_ai_cpu/vendor_ai_cpu.h"
#include "cddcnn_lib_ai2.h"
#include "limit_fdet_lib.h"

#define VENDOR_AI_CFG       0x000f0000
#define CDD_RUN_ID          3
#define LIMIT_RUN_ID        1
#define POOL_SIZE           (24 * MBYTE)
#define CDD_MODEL           "/mnt/sd/CNNLib/para/cddcnn/nvt_model.bin"
#define CDD_PARA            "/mnt/sd/CNNLib/para/cddcnn/para.txt"
#define LIMIT_MODEL         "/mnt/sd/CNNLib/para/public/nvt_model.bin"
#define LIMIT_PARA          "/mnt/sd/CNNLib/para/public/para.txt"
#define IMG_LIST            "/mnt/sd/jpg/cdd_image_list.txt"
#define IMG_DIR             "/mnt/sd/jpg/CDD"
#define RESULTS             "/mnt/sd/det_results/cdd_test_results.txt"
#define RAW_RESULTS         "/mnt/sd/det_results/cdd_raw_results.txt"

static VENDOR_AIS_FLOW_MEM_PARM pool;
static HD_COMMON_MEM_VB_BLK pool_blk;

static INT32 file_size(const CHAR *name)
{
    FILE *f = fopen(name, "rb");
    INT32 n;

    if (!f) {
        printf("cannot open %s\n", name);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fclose(f);
    return n;
}

static INT32 load(const CHAR *name, UINT32 va, UINT32 room)
{
    FILE *f = fopen(name, "rb");
    INT32 n;

    if (!f) {
        printf("cannot open %s\n", name);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n < 0 || (UINT32)n > room || (INT32)fread((VOID *)va, 1, n, f) != n) {
        printf("cannot read %s\n", name);
        n = -1;
    }
    fclose(f);
    return n;
}

/*
 * The pool's cursor to a multiple of 64 bytes: the libraries' version
 * checks step a model's address past its 32-byte prefix and round it up,
 * which lands on the model only if it started aligned.
 */
static VOID align(VENDOR_AIS_FLOW_MEM_PARM *buf)
{
    UINT32 pad = (64 - (buf->va & 63)) & 63;

    pad = pad < buf->size ? pad : buf->size;
    buf->va += pad;
    buf->pa += pad;
    buf->size -= pad;
}

/* the options the turnkey's detectors run with */
static VOID net_opt(UINT32 id)
{
    VENDOR_AI_NET_CFG_BUF_OPT buf = { 0 };
    VENDOR_AI_NET_CFG_JOB_OPT job = { 0 };

    buf.method = VENDOR_AI_NET_BUF_OPT_NONE;
    buf.ddr_id = DDR_ID0;
    vendor_ai_net_set(id, VENDOR_AI_NET_PARAM_CFG_BUF_OPT, &buf);
    job.method = VENDOR_AI_NET_JOB_OPT_LINEAR_O1;
    job.wait_ms = 0;
    job.schd_parm = VENDOR_AI_FAIR_CORE_ALL;
    vendor_ai_net_set(id, VENDOR_AI_NET_PARAM_CFG_JOB_OPT, &job);
}

/* a model into its memory, the network opened with its work buffer */
static HD_RESULT net_open(UINT32 id, VENDOR_AIS_FLOW_MEM_PARM *model,
                          VENDOR_AIS_FLOW_MEM_PARM *io,
                          VENDOR_AIS_FLOW_MEM_PARM *buf)
{
    VENDOR_AI_NET_CFG_WORKBUF wbuf = { 0 };
    HD_RESULT ret;

    ret = vendor_ai_net_set(id, VENDOR_AI_NET_PARAM_CFG_MODEL,
                            (VENDOR_AI_NET_CFG_MODEL *)model);
    if (ret == HD_OK) {
        ret = vendor_ai_net_open(id);
    }
    if (ret == HD_OK) {
        ret = vendor_ai_net_get(id, VENDOR_AI_NET_PARAM_CFG_WORKBUF, &wbuf);
    }
    if (ret == HD_OK) {
        ret = get_cdd_mem(buf, io, wbuf.size, 32);
    }
    if (ret == HD_OK) {
        ret = vendor_ai_net_set(id, VENDOR_AI_NET_PARAM_CFG_WORKBUF, io);
    }
    if (ret != HD_OK) {
        printf("ERR: network %lu open fail (%d)\n", (unsigned long)id,
               (int)ret);
    }
    return ret;
}

static HD_RESULT cdd_preset(CDDCNN_MEM *mem, VENDOR_AIS_FLOW_MEM_PARM *buf)
{
    INT32 n = file_size(CDD_MODEL);
    HD_RESULT ret;

    net_opt(CDD_RUN_ID);
    ret = get_cdd_mem(buf, &mem->input_mem,
                      MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * 3 / 2, 32);
    if (ret != HD_OK || n <= 0) {
        return HD_ERR_NG;
    }
    align(buf);
    ret = get_cdd_mem(buf, &mem->model_mem, n, 32);
    if (ret != HD_OK ||
        load(CDD_MODEL, mem->model_mem.va, mem->model_mem.size) < 0) {
        return HD_ERR_NG;
    }
    cddcnn_get_version();
    ret = cddcnn_version_check(&mem->model_mem);
    if (ret != HD_OK) {
        printf("ERR: cddcnn version check fail (%d)\n", (int)ret);
        return ret;
    }
    ret = net_open(CDD_RUN_ID, &mem->model_mem, &mem->io_mem, buf);
    if (ret != HD_OK) {
        return ret;
    }
    return get_post_mem(buf, mem);
}

static HD_RESULT limit_preset(LIMIT_FDET_MEM *mem,
                              VENDOR_AIS_FLOW_MEM_PARM *buf)
{
    INT32 n = file_size(LIMIT_MODEL);
    HD_RESULT ret;

    net_opt(LIMIT_RUN_ID);
    ret = get_limit_fdet_mem(buf, &mem->input_mem,
                             LIMIT_FDET_WIDTH * LIMIT_FDET_HEIGHT * 3 / 2, 32);
    if (ret != HD_OK || n <= 0) {
        return HD_ERR_NG;
    }
    align(buf);
    ret = get_limit_fdet_mem(buf, &mem->model_mem, n, 32);
    if (ret != HD_OK ||
        load(LIMIT_MODEL, mem->model_mem.va, mem->model_mem.size) < 0) {
        return HD_ERR_NG;
    }
    ret = limit_fdet_version_check(&mem->model_mem);
    if (ret != HD_OK) {
        printf("ERR: limit_fdet version check fail (%d)\n", (int)ret);
        return ret;
    }
    ret = net_open(LIMIT_RUN_ID, &mem->model_mem, &mem->io_mem, buf);
    if (ret != HD_OK) {
        return ret;
    }
    return vendor_ai_net_start(LIMIT_RUN_ID);
}

static VOID *detect(VOID *arg)
{
    VENDOR_AIS_FLOW_MEM_PARM buf = pool;
    VENDOR_AIS_FLOW_MEM_PARM scale;
    PROPOSAL_PARAM prop = { 0 };
    LIMIT_PARAM limit = { 0 };
    LIMIT_FDET_PARAM lf = { 0 };
    LIMIT_FDET_MEM lf_mem = { 0 };
    CDDCNN_MEM mem = { 0 };
    BACKBONE_OUT *backbone;
    CHAR line[256], name[128], path[300];
    unsigned int w, h;
    UINT32 i;
    FILE *list = NULL, *out = NULL, *raw = NULL;
    HD_RESULT ret;

    prop.score_thres = 0.55;            /* the guide's defaults */
    prop.nms_thres = 0.1;
    prop.run_id = CDD_RUN_ID;
    lf.run_id = LIMIT_RUN_ID;
    lf.cls_det_threshold = 0.98;
    lf.cls_threshold = 0.7;
    limit_fdet_read_para(&lf.cls_det_threshold, &lf.cls_threshold,
                         &lf.cls_debug_print, LIMIT_PARA);

    /* the picture, scaled to the network's 1024x576, first in the pool */
    if (get_cdd_mem(&buf, &scale, YUV_WIDTH * YUV_HEIGHT * 3 / 2, 32) !=
            HD_OK ||
        cdd_preset(&mem, &buf) != HD_OK ||
        vendor_ai_net_start(CDD_RUN_ID) != HD_OK) {
        printf("ERR: cddcnn preset fail\n");
        goto close;
    }
    backbone = (BACKBONE_OUT *)mem.backbone_output.va;
    if (cddcnn_init(&prop, backbone, &limit, CDD_PARA) != HD_OK) {
        printf("ERR: cddcnn init fail\n");
        goto stop;
    }
    if (limit_preset(&lf_mem, &buf) != HD_OK) {
        printf("ERR: limit_fdet preset fail\n");
        goto stop;
    }
    list = fopen(IMG_LIST, "r");
    out = fopen(RESULTS, "w");
    raw = fopen(RAW_RESULTS, "w");
    if (!list || !out || !raw) {
        printf("ERR: cannot open %s, %s or %s\n", IMG_LIST, RESULTS,
               RAW_RESULTS);
        goto stop;
    }
    while (fgets(line, sizeof(line), list)) {
        VENDOR_AI_BUF src = { 0 }, img = { 0 };
        HD_GFX_IMG_BUF gfx = { 0 };
        CDD_IRECT roi;
        CDDCNN_RESULT *r;
        INT32 n;

        if (sscanf(line, "%127s %u %u", name, &w, &h) != 3) {
            continue;
        }
        snprintf(path, sizeof(path), "%s/%s", IMG_DIR, name);
        n = load(path, mem.input_mem.va, mem.input_mem.size);
        if (n < 0) {
            continue;
        }
        hd_common_mem_flush_cache((VOID *)mem.input_mem.va, n);
        src.sign = MAKEFOURCC('A', 'B', 'U', 'F');
        src.width = w;
        src.height = h;
        src.channel = 2;
        src.line_ofs = ALIGN_CEIL_4(w);
        src.fmt = HD_VIDEO_PXLFMT_YUV420;
        src.pa = mem.input_mem.pa;
        src.va = mem.input_mem.va;
        src.size = src.line_ofs * h * 3 / 2;

        gfx.dim.w = YUV_WIDTH;
        gfx.dim.h = YUV_HEIGHT;
        gfx.format = HD_VIDEO_PXLFMT_YUV420;
        gfx.lineoffset[0] = YUV_WIDTH;
        gfx.lineoffset[1] = YUV_WIDTH;
        gfx.p_phy_addr[0] = scale.pa;
        gfx.p_phy_addr[1] = scale.pa + YUV_WIDTH * YUV_HEIGHT;
        roi.x1 = 0;
        roi.y1 = 0;
        roi.x2 = w;
        roi.y2 = h;
        ret = ai_crop_img(&gfx, &src, HD_GFX_SCALE_QUALITY_NULL, &roi);
        if (ret != HD_OK) {
            printf("ERR: ai_crop_img fail (%d)\n", (int)ret);
            continue;
        }
        cdd_gfx_img_to_vendor(&img, &gfx, scale.va);

        ret = cddcnn_process(&prop, &mem, &limit, backbone, &img, 0);
        if (ret != HD_OK) {
            printf("ERR: cddcnn_process fail (%d)\n", (int)ret);
            continue;
        }
        r = (CDDCNN_RESULT *)mem.out_result.va;
        for (i = 0; i < (UINT32)mem.out_num; i++) {
            FLOAT sx = (FLOAT)w / YUV_WIDTH, sy = (FLOAT)h / YUV_HEIGHT;

            fprintf(raw, "%s %d %f %f %f %f %f\n", name, (int)r[i].category,
                    r[i].score, r[i].x1 * sx, r[i].y1 * sy, r[i].x2 * sx,
                    r[i].y2 * sy);
        }
        printf("%s: %d proposed\n", name, (int)mem.out_num);
        lf.ratiow = 1;
        lf.ratioh = 1;
        lf.cls_id = 1;
        lf.pvdcls = 0;
        lf.proc_num = mem.out_num;
        lf_mem.det_in_mem = mem.out_result;
        lf_mem.det_out_mem = mem.final_result;
        ret = limit_fdet_process(&lf, &lf_mem, &img);
        if (ret != HD_OK) {
            printf("ERR: limit_fdet_process fail (%d)\n", (int)ret);
            continue;
        }
        mem.out_num = lf.proc_num;

        r = (CDDCNN_RESULT *)mem.final_result.va;
        printf("%s: %d confirmed\n", name, (int)mem.out_num);
        for (i = 0; i < (UINT32)mem.out_num; i++) {
            FLOAT sx = (FLOAT)w / YUV_WIDTH, sy = (FLOAT)h / YUV_HEIGHT;

            fprintf(out, "%s %d %f %f %f %f %f\n", name, (int)r[i].category,
                    r[i].score, r[i].x1 * sx, r[i].y1 * sy, r[i].x2 * sx,
                    r[i].y2 * sy);
        }
    }
    vendor_ai_net_stop(LIMIT_RUN_ID);
    vendor_ai_net_close(LIMIT_RUN_ID);
stop:
    vendor_ai_net_stop(CDD_RUN_ID);
close:
    vendor_ai_net_close(CDD_RUN_ID);
    if (list) {
        fclose(list);
    }
    if (out) {
        fclose(out);
    }
    if (raw) {
        fclose(raw);
    }
    printf("cddcnn done\n");
    return NULL;
}

int main(int argc, char *argv[])
{
    HD_COMMON_MEM_INIT_CONFIG cfg = { 0 };
    UINT32 schd = VENDOR_AI_PROC_SCHD_FAIR;
    pthread_t t;

    if (hd_common_init(0) != HD_OK) {
        printf("hd_common_init fail\n");
        return 1;
    }
    hd_common_sysconfig(0, (1 << 16), 0, VENDOR_AI_CFG);     /* AI engines */
    if (hd_gfx_init() != HD_OK) {
        printf("hd_gfx_init fail\n");
        goto uninit;
    }
    cfg.pool_info[0].type = HD_COMMON_MEM_USER_DEFINIED_POOL;
    cfg.pool_info[0].blk_size = POOL_SIZE;
    cfg.pool_info[0].blk_cnt = 1;
    cfg.pool_info[0].ddr_id = DDR_ID0;
    if (hd_common_mem_init(&cfg) != HD_OK) {
        printf("hd_common_mem_init fail\n");
        goto gfx;
    }
    pool_blk = hd_common_mem_get_block(HD_COMMON_MEM_USER_DEFINIED_POOL,
                                       POOL_SIZE, DDR_ID0);
    if (pool_blk == HD_COMMON_MEM_VB_INVALID_BLK) {
        printf("hd_common_mem_get_block fail\n");
        goto mem;
    }
    pool.pa = hd_common_mem_blk2pa(pool_blk);
    pool.va = (UINT32)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE,
                                         pool.pa, POOL_SIZE);
    pool.size = POOL_SIZE;
    if (!pool.pa || !pool.va) {
        printf("pool map fail\n");
        goto block;
    }
    vendor_ai_cfg_set(VENDOR_AI_CFG_PLUGIN_ENGINE, vendor_ai_cpu1_get_engine());
    vendor_ai_cfg_set(VENDOR_AI_CFG_PROC_SCHD, &schd);
    if (vendor_ai_init() != HD_OK) {
        printf("vendor_ai_init fail\n");
        goto unmap;
    }
    if (pthread_create(&t, NULL, detect, NULL) == 0) {
        pthread_join(t, NULL);
    }
    vendor_ai_uninit();
unmap:
    hd_common_mem_munmap((VOID *)pool.va, POOL_SIZE);
block:
    hd_common_mem_release_block(pool_blk);
mem:
    hd_common_mem_uninit();
gfx:
    hd_gfx_uninit();
uninit:
    hd_common_uninit();
    return 0;
}
