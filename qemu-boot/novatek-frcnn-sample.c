/*
 * The AI turnkey's face recognition network, frcnn, on a list of aligned
 * face pictures.  The turnkey package ships its model
 * (CNNLib/para/frcnn/nvt_model.bin) and nothing else: no library, sample,
 * parameters or documentation in any SDK release.  The model says what it
 * is -- input "data" 112x112x3, output "BatchNormalization_fc1_Y" 1x1x128,
 * the InsightFace / ArcFace layout of a face embedding -- so this runs it
 * through vendor_ai_net alone, as ai_net_with_buf does, and writes each
 * face's 128 values.  qemu-boot/build-novatek-hdal-samples.sh builds it
 * with the SDK.
 *
 * It reads /mnt/sd/jpg/fr_image_list.txt -- a name a line, the faces in
 * /mnt/sd/jpg/FR as 112x112 NV12, aligned to the ArcFace landmark template
 * -- and writes "name v0 .. v127", the output in real units, to
 * /mnt/sd/det_results/fr_features.txt.  It exits 1 unless every face went
 * through.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hdal.h"
#include "hd_type.h"
#include "vendor_ai.h"
#include "vendor_ai_cpu/vendor_ai_cpu.h"

#define VENDOR_AI_CFG       0x000f0000
#define FR_RUN_ID           0
#define POOL_SIZE           (16 * 1024 * 1024)
#define FACE_SIZE           112
/*
 * The turnkey's models start with 32 bytes of their own (a type word, then
 * zeros); the network, chip id 4, follows.  The detector libraries' version
 * checks step past them; vendor_ai_net would read chip id 0 and refuse it.
 */
#define TURNKEY_PREFIX      32
#define FR_MODEL            "/mnt/sd/CNNLib/para/frcnn/nvt_model.bin"
#define IMG_LIST            "/mnt/sd/jpg/fr_image_list.txt"
#define IMG_DIR             "/mnt/sd/jpg/FR"
#define RESULTS             "/mnt/sd/det_results/fr_features.txt"

typedef struct {
    UINT32 pa;
    UINT32 va;
    UINT32 size;
} MEM;

static MEM pool;
static HD_COMMON_MEM_VB_BLK pool_blk;

/* the next size bytes of the pool, 64-byte aligned */
static int carve(MEM *from, MEM *to, UINT32 size)
{
    UINT32 pad = (64 - (from->va & 63)) & 63;

    if (pad + size > from->size) {
        printf("ERR: pool too small for %lu bytes\n", (unsigned long)size);
        return -1;
    }
    to->pa = from->pa + pad;
    to->va = from->va + pad;
    to->size = size;
    from->pa += pad + size;
    from->va += pad + size;
    from->size -= pad + size;
    return 0;
}

static INT32 load(const CHAR *name, MEM *m)
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
    if (n < 0 || (UINT32)n > m->size ||
        (INT32)fread((VOID *)m->va, 1, n, f) != n) {
        printf("cannot read %s\n", name);
        n = -1;
    }
    fclose(f);
    if (n > 0) {
        hd_common_mem_flush_cache((VOID *)m->va, n);
    }
    return n;
}

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

/* the model loaded and opened with its work buffer, from the pool */
static HD_RESULT net_open(MEM *buf)
{
    VENDOR_AI_NET_CFG_BUF_OPT bopt = { 0 };
    VENDOR_AI_NET_CFG_JOB_OPT jopt = { 0 };
    VENDOR_AI_NET_CFG_MODEL model = { 0 };
    VENDOR_AI_NET_CFG_WORKBUF wbuf = { 0 };
    INT32 n = file_size(FR_MODEL);
    MEM m;
    HD_RESULT ret;

    bopt.method = VENDOR_AI_NET_BUF_OPT_NONE;
    bopt.ddr_id = DDR_ID0;
    vendor_ai_net_set(FR_RUN_ID, VENDOR_AI_NET_PARAM_CFG_BUF_OPT, &bopt);
    jopt.method = VENDOR_AI_NET_JOB_OPT_LINEAR_O1;
    jopt.wait_ms = 0;
    jopt.schd_parm = VENDOR_AI_FAIR_CORE_ALL;
    vendor_ai_net_set(FR_RUN_ID, VENDOR_AI_NET_PARAM_CFG_JOB_OPT, &jopt);

    if (n <= TURNKEY_PREFIX || carve(buf, &m, n) < 0 ||
        load(FR_MODEL, &m) < 0) {
        return HD_ERR_NG;
    }
    model.pa = m.pa + TURNKEY_PREFIX;
    model.va = m.va + TURNKEY_PREFIX;
    model.size = m.size - TURNKEY_PREFIX;
    ret = vendor_ai_net_set(FR_RUN_ID, VENDOR_AI_NET_PARAM_CFG_MODEL, &model);
    if (ret == HD_OK) {
        ret = vendor_ai_net_open(FR_RUN_ID);
    }
    if (ret == HD_OK) {
        ret = vendor_ai_net_get(FR_RUN_ID, VENDOR_AI_NET_PARAM_CFG_WORKBUF,
                                &wbuf);
    }
    if (ret == HD_OK && carve(buf, &m, wbuf.size) < 0) {
        ret = HD_ERR_NG;
    }
    if (ret == HD_OK) {
        wbuf.pa = m.pa;
        wbuf.va = m.va;
        wbuf.size = m.size;
        ret = vendor_ai_net_set(FR_RUN_ID, VENDOR_AI_NET_PARAM_CFG_WORKBUF,
                                &wbuf);
    }
    if (ret != HD_OK) {
        printf("ERR: frcnn open fail (%d)\n", (int)ret);
    }
    return ret;
}

/* the output buffer as real numbers, whatever its fixed-point format */
static int features(const VENDOR_AI_BUF *o, float *v, UINT32 n)
{
    /* HD_VIDEO_PXLFMT_TYPE() has a parenthesis too many */
    UINT32 type = o->fmt & 0xffff0000;
    INT32 frac = HD_VIDEO_PXLFMT_FRAC(o->fmt);
    float unit = 1.0f;
    UINT32 i;

    for (; frac > 0; frac--) {
        unit /= 2;
    }
    for (; frac < 0; frac++) {
        unit *= 2;
    }
    hd_common_mem_flush_cache((VOID *)o->va, o->size);
    for (i = 0; i < n; i++) {
        switch (type) {
        case HD_VIDEO_PXLFMT_AI_SINT8:
            v[i] = ((INT8 *)o->va)[i] * unit;
            break;
        case HD_VIDEO_PXLFMT_AI_UINT8:
            v[i] = ((UINT8 *)o->va)[i] * unit;
            break;
        case HD_VIDEO_PXLFMT_AI_SINT16:
            v[i] = ((INT16 *)o->va)[i] * unit;
            break;
        case HD_VIDEO_PXLFMT_AI_UINT16:
            v[i] = ((UINT16 *)o->va)[i] * unit;
            break;
        case HD_VIDEO_PXLFMT_AI_FLOAT32:
            v[i] = ((float *)o->va)[i];
            break;
        default:
            printf("ERR: output format 0x%lx\n", (unsigned long)o->fmt);
            return -1;
        }
    }
    return 0;
}

static int recognise(void)
{
    VENDOR_AI_NET_INFO info = { 0 };
    VENDOR_AI_BUF in = { 0 }, out = { 0 };
    UINT32 path = 0, failures = 0, i, n;
    MEM buf = pool, face;
    CHAR line[256], name[128], file[256];
    float v[128];
    FILE *list = NULL, *res = NULL;
    int ok = 0;

    if (carve(&buf, &face, FACE_SIZE * FACE_SIZE * 3 / 2) < 0 ||
        net_open(&buf) != HD_OK) {
        return 0;
    }
    if (vendor_ai_net_start(FR_RUN_ID) != HD_OK) {
        printf("ERR: frcnn start fail\n");
        goto close;
    }
    if (vendor_ai_net_get(FR_RUN_ID, VENDOR_AI_NET_PARAM_INFO, &info) !=
            HD_OK || info.out_buf_cnt != 1 ||
        vendor_ai_net_get(FR_RUN_ID, VENDOR_AI_NET_PARAM_OUT_PATH_LIST,
                          &path) != HD_OK) {
        printf("ERR: frcnn has no single output\n");
        goto stop;
    }
    list = fopen(IMG_LIST, "r");
    res = fopen(RESULTS, "w");
    if (!list || !res) {
        printf("ERR: cannot open %s or %s\n", IMG_LIST, RESULTS);
        goto stop;
    }
    while (fgets(line, sizeof(line), list)) {
        if (sscanf(line, "%127s", name) != 1) {
            continue;
        }
        snprintf(file, sizeof(file), "%s/%s", IMG_DIR, name);
        if (load(file, &face) != FACE_SIZE * FACE_SIZE * 3 / 2) {
            failures++;
            continue;
        }
        in.sign = MAKEFOURCC('A', 'B', 'U', 'F');
        in.width = FACE_SIZE;
        in.height = FACE_SIZE;
        in.channel = 2;
        in.line_ofs = FACE_SIZE;
        in.fmt = HD_VIDEO_PXLFMT_YUV420;
        in.pa = face.pa;
        in.va = face.va;
        in.size = face.size;
        if (vendor_ai_net_set(FR_RUN_ID, VENDOR_AI_NET_PARAM_IN(0, 0), &in) !=
                HD_OK ||
            vendor_ai_net_proc(FR_RUN_ID) != HD_OK ||
            vendor_ai_net_get(FR_RUN_ID, path, &out) != HD_OK) {
            printf("ERR: frcnn on %s fail\n", name);
            failures++;
            continue;
        }
        n = out.width * out.height * out.channel;
        if (n != 128 || features(&out, v, n) < 0) {
            printf("ERR: frcnn output %lux%lux%lu fmt 0x%lx\n",
                   (unsigned long)out.width, (unsigned long)out.height,
                   (unsigned long)out.channel, (unsigned long)out.fmt);
            failures++;
            continue;
        }
        fprintf(res, "%s", name);
        for (i = 0; i < n; i++) {
            fprintf(res, " %g", v[i]);
        }
        fprintf(res, "\n");
        printf("%s: %s fmt 0x%lx\n", name, out.name ? out.name : "?",
               (unsigned long)out.fmt);
    }
    ok = failures == 0;
stop:
    vendor_ai_net_stop(FR_RUN_ID);
close:
    vendor_ai_net_close(FR_RUN_ID);
    if (list) {
        fclose(list);
    }
    if (res) {
        fclose(res);
    }
    printf("frcnn done\n");
    return ok;
}

int main(int argc, char *argv[])
{
    HD_COMMON_MEM_INIT_CONFIG cfg = { 0 };
    UINT32 schd = VENDOR_AI_PROC_SCHD_FAIR;
    int rc = 1;

    if (hd_common_init(0) != HD_OK) {
        printf("hd_common_init fail\n");
        return 1;
    }
    hd_common_sysconfig(0, (1 << 16), 0, VENDOR_AI_CFG);     /* AI engines */
    cfg.pool_info[0].type = HD_COMMON_MEM_USER_DEFINIED_POOL;
    cfg.pool_info[0].blk_size = POOL_SIZE;
    cfg.pool_info[0].blk_cnt = 1;
    cfg.pool_info[0].ddr_id = DDR_ID0;
    if (hd_common_mem_init(&cfg) != HD_OK) {
        printf("hd_common_mem_init fail\n");
        goto uninit;
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
    rc = recognise() ? 0 : 1;
    vendor_ai_uninit();
unmap:
    hd_common_mem_munmap((VOID *)pool.va, POOL_SIZE);
block:
    hd_common_mem_release_block(pool_blk);
mem:
    hd_common_mem_uninit();
uninit:
    hd_common_uninit();
    return rc;
}
