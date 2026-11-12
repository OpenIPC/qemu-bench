/*
 * ive-v5-capture: run the V5 (Hi3516CV608 / CV610) IVE ops that hisi-ive-npu
 * does not model yet on fixed synthetic inputs, and write every output to
 * OUT/<name>.bin, rows packed (no stride padding).  Run on a real board it
 * gives the reference; run under QEMU the same binary gives the model's
 * output, and the two directories must match (ive-v5-capture.sh).
 *
 * Built against the SDK's static MPI libraries and sample helpers
 * (sample_common_ive.c); see ive-v5-capture.sh.
 *
 *   ive-v5-capture OUT
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

#include "sample_common_ive.h"
#include "ss_mpi_ive.h"

static const char *g_out;
static int g_fail;

#define PTR(t, a)   ((t *)(td_uintptr_t)(a))

/* a deterministic picture: shapes with sharp edges, a ramp, some noise */
static void fill_u8(ot_svp_img *img, int seed)
{
    td_u32 lcg = 12345u + seed * 7919u;

    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int v = (x * 3 + y * 2 + seed * 17) & 0x7f;

            if ((x / 12 + y / 10 + seed) % 3 == 0) {
                v += 100;
            }
            if (x > img->width / 3 + seed && x < img->width * 2 / 3 + seed &&
                y > img->height / 4 && y < img->height * 3 / 4) {
                v = 230 - (x + y) % 9;
            }
            lcg = lcg * 1103515245u + 12345u;
            v += (int)((lcg >> 16) & 7) - 3;
            row[x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

static void fill_bytes(td_u64 virt, td_u32 n, int seed)
{
    td_u32 lcg = 777u + seed * 31u;

    for (td_u32 i = 0; i < n; i++) {
        lcg = lcg * 1103515245u + 12345u;
        PTR(td_u8, virt)[i] = lcg >> 16;
    }
}

/*
 * A picture that walks every value: the CSC matrices are fitted from the
 * captures, and a random picture leaves a dozen chroma values unseen,
 * which leaves their rounding constants a few 1024ths wide.
 */
static void fill_ramp(td_u64 virt, td_u32 stride, td_u32 w, td_u32 h, int step)
{
    for (td_u32 y = 0; y < h; y++) {
        td_u8 *row = PTR(td_u8, virt) + y * stride;

        for (td_u32 x = 0; x < w; x++) {
            row[x] = (y * w + x) * step;
        }
    }
}

static int elem_size(ot_svp_img_type t)
{
    switch (t) {
    case OT_SVP_IMG_TYPE_S16C1:
    case OT_SVP_IMG_TYPE_U16C1:
    case OT_SVP_IMG_TYPE_S8C2_PACKAGE:
        return 2;
    case OT_SVP_IMG_TYPE_U8C3_PACKAGE:
        return 3;
    case OT_SVP_IMG_TYPE_S32C1:
    case OT_SVP_IMG_TYPE_U32C1:
        return 4;
    default:
        return 1;
    }
}

static FILE *out_file(const char *name)
{
    char path[512];
    FILE *f;

    snprintf(path, sizeof(path), "%s/%s.bin", g_out, name);
    f = fopen(path, "wb");
    if (!f) {
        printf("FAIL open %s: %s\n", path, strerror(errno));
        g_fail++;
    }
    return f;
}

/* a plane's own row count: 420's chroma plane is half the picture's */
static td_u32 plane_rows(ot_svp_img_type t, int p, td_u32 h)
{
    return p && t == OT_SVP_IMG_TYPE_YUV420SP ? h / 2 : h;
}

/* every plane, rows packed */
static void dump_img(const char *name, const ot_svp_img *img)
{
    int planes = 1, e = elem_size(img->type);
    FILE *f = out_file(name);

    if (!f) {
        return;
    }
    if (img->type == OT_SVP_IMG_TYPE_YUV420SP ||
        img->type == OT_SVP_IMG_TYPE_YUV422SP) {
        planes = 2;
    } else if (img->type == OT_SVP_IMG_TYPE_U8C3_PLANAR) {
        planes = 3;
    }
    if (img->type == OT_SVP_IMG_TYPE_S8C2_PACKAGE) {
        planes = 1;             /* both channels interleaved in plane 0 */
    }
    for (int p = 0; p < planes; p++) {
        td_u32 h = plane_rows(img->type, p, img->height);

        for (td_u32 y = 0; y < h; y++) {
            fwrite(PTR(td_u8, img->virt_addr[p]) + y * img->stride[p] * e,
                   img->width * e, 1, f);
        }
    }
    fclose(f);
}

static void dump_mem(const char *name, td_u64 virt, td_u32 n)
{
    FILE *f = out_file(name);

    if (f) {
        fwrite(PTR(td_u8, virt), n, 1, f);
        fclose(f);
    }
}

/*
 * The two types sample_common_ive_create_image() leaves unallocated: S8C2
 * package (one buffer, channel 1 at +1) and U8C3 planar (three planes)
 */
static int mk_raw(ot_svp_img *img, ot_svp_img_type t, td_u32 w, td_u32 h)
{
    td_u32 stride = sample_common_ive_calc_stride(w, OT_IVE_ALIGN);
    int planes = t == OT_SVP_IMG_TYPE_U8C3_PLANAR ? 3 : 1;
    td_u32 plane = stride * h * (t == OT_SVP_IMG_TYPE_S8C2_PACKAGE ? 2 : 1);
    td_void *virt = TD_NULL;

    img->type = t;
    img->width = w;
    img->height = h;
    if (ss_mpi_sys_mmz_alloc((td_phys_addr_t *)&img->phys_addr[0], &virt, TD_NULL,
                             TD_NULL, plane * planes) != TD_SUCCESS) {
        return -1;
    }
    img->virt_addr[0] = (td_u64)(td_uintptr_t)virt;
    for (int p = 0; p < 3; p++) {
        img->stride[p] = stride;
    }
    if (t == OT_SVP_IMG_TYPE_S8C2_PACKAGE) {
        img->phys_addr[1] = img->phys_addr[0] + 1;
        img->virt_addr[1] = img->virt_addr[0] + 1;
    } else {
        for (int p = 1; p < 3; p++) {
            img->phys_addr[p] = img->phys_addr[p - 1] + plane;
            img->virt_addr[p] = img->virt_addr[p - 1] + plane;
        }
    }
    return 0;
}

static int mk(ot_svp_img *img, ot_svp_img_type t, td_u32 w, td_u32 h)
{
    memset(img, 0, sizeof(*img));
    if (t == OT_SVP_IMG_TYPE_S8C2_PACKAGE || t == OT_SVP_IMG_TYPE_U8C3_PLANAR) {
        if (mk_raw(img, t, w, h)) {
            printf("FAIL create image %ux%u type %d\n", w, h, t);
            g_fail++;
            return -1;
        }
        memset(PTR(td_u8, img->virt_addr[0]), 0, img->stride[0] * h *
               (t == OT_SVP_IMG_TYPE_S8C2_PACKAGE ? 2 : 3));
        return 0;
    }
    if (sample_common_ive_create_image(img, t, w, h) != TD_SUCCESS) {
        printf("FAIL create image %ux%u type %d\n", w, h, t);
        g_fail++;
        return -1;
    }
    for (int p = 0; p < 3; p++) {
        if (img->virt_addr[p]) {
            memset(PTR(td_u8, img->virt_addr[p]), 0,
                   img->stride[p] * plane_rows(t, p, img->height) *
                   elem_size(t));
        }
    }
    return 0;
}

static int mkmem(ot_svp_mem_info *m, td_u32 size)
{
    memset(m, 0, sizeof(*m));
    if (sample_common_ive_create_mem_info(m, size) != TD_SUCCESS) {
        printf("FAIL create mem %u\n", size);
        g_fail++;
        return -1;
    }
    memset(PTR(td_u8, m->virt_addr), 0, size);
    return 0;
}

static void freeimg(ot_svp_img *img)
{
    sample_svp_mmz_free(img->phys_addr[0], img->virt_addr[0]);
}

static void freemem(ot_svp_mem_info *m)
{
    sample_svp_mmz_free(m->phys_addr, m->virt_addr);
}

/* wait for a task the way the samples do */
static int done(const char *what, td_s32 ret, ot_ive_handle h)
{
    td_bool fin = TD_FALSE;

    if (ret != TD_SUCCESS) {
        printf("FAIL %s: %#x\n", what, ret);
        g_fail++;
        return -1;
    }
    do {
        ret = ss_mpi_ive_query(h, &fin, TD_TRUE);
    } while (ret == OT_ERR_IVE_QUERY_TIMEOUT);
    if (ret != TD_SUCCESS) {
        printf("FAIL %s query: %#x\n", what, ret);
        g_fail++;
        return -1;
    }
    printf("ok %s\n", what);
    return 0;
}

static const td_s8 g_sobel[OT_IVE_MASK_NUM] = {
    0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, -2, 0, 2, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0,
};

#define W   64
#define H   64

static void cap_mag_and_ang(void)
{
    ot_svp_img src, mag, ang;
    ot_ive_mag_and_ang_ctrl c = { 0 };
    static const td_u16 thr[] = { 0, 200 };
    ot_ive_handle h;
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&mag, OT_SVP_IMG_TYPE_U16C1, W, H) ||
        mk(&ang, OT_SVP_IMG_TYPE_U8C1, W, H)) {
        return;
    }
    fill_u8(&src, 1);
    memcpy(c.mask, g_sobel, sizeof(c.mask));
    for (int out = 0; out < 2; out++) {
        for (int t = 0; t < 2; t++) {
            c.out_ctrl = out;
            c.threshold = thr[t];
            snprintf(n, sizeof(n), "mag_and_ang_out%d_thr%u", out, thr[t]);
            if (!done(n, ss_mpi_ive_mag_and_ang(&h, &src, &mag, out ? &ang : NULL,
                                                &c, TD_TRUE), h)) {
                snprintf(n, sizeof(n), "mag_and_ang_out%d_thr%u_mag", out, thr[t]);
                dump_img(n, &mag);
                if (out) {
                    snprintf(n, sizeof(n), "mag_and_ang_out%d_thr%u_ang", out, thr[t]);
                    dump_img(n, &ang);
                }
            }
        }
    }
    freeimg(&src);
    freeimg(&mag);
    freeimg(&ang);
}

static void cap_canny(void)
{
    ot_svp_img src, edge;
    ot_svp_mem_info stack;
    ot_ive_canny_hys_edge_ctrl c = { 0 };
    ot_ive_handle h;
    /* a weak band that touches the strong pixels (20 / 60) as well as 50 / 150 */
    static const td_u16 lo[] = { 50, 20, 100 }, hi[] = { 150, 60, 400 };
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&edge, OT_SVP_IMG_TYPE_U8C1, W, H)) {
        return;
    }
    if (mkmem(&stack, src.stride[0] * H * sizeof(ot_svp_point_u16) +
                      sizeof(ot_ive_canny_stack_size)) ||
        mkmem(&c.mem, src.stride[0] * H * 3)) {
        return;
    }
    memcpy(c.mask, g_sobel, sizeof(c.mask));
    for (int s = 0; s < 3; s++) {
        fill_u8(&src, 2 + s * 7);
        for (int t = 0; t < 3; t++) {
            c.low_threshold = lo[t];
            c.high_threshold = hi[t];
            snprintf(n, sizeof(n), "canny_s%d_t%d", s, t);
            if (done(n, ss_mpi_ive_canny_hys_edge(&h, &src, &edge, &stack, &c,
                                                  TD_TRUE), h)) {
                continue;
            }
            snprintf(n, sizeof(n), "canny_s%d_t%d_edge", s, t);
            dump_img(n, &edge);
            snprintf(n, sizeof(n), "canny_s%d_t%d_stack", s, t);
            dump_mem(n, stack.virt_addr, 256);
            if (s == 0 && t == 0) {
                if (ss_mpi_ive_canny_edge(&edge, &stack) == TD_SUCCESS) {
                    dump_img("canny_edge_final", &edge);
                    printf("ok canny_edge\n");
                } else {
                    printf("FAIL canny_edge\n");
                    g_fail++;
                }
            }
        }
    }
    freemem(&stack);
    freemem(&c.mem);
    freeimg(&src);
    freeimg(&edge);
}

static void cap_lbp(void)
{
    ot_svp_img src, dst;
    ot_ive_lbp_ctrl c = { 0 };
    ot_ive_handle h;

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&dst, OT_SVP_IMG_TYPE_U8C1, W, H)) {
        return;
    }
    fill_u8(&src, 3);
    c.mode = OT_IVE_LBP_COMPARE_MODE_NORMAL;
    c.bit_threshold.s8_val = 0;
    if (!done("lbp_normal_0", ss_mpi_ive_lbp(&h, &src, &dst, &c, TD_TRUE), h)) {
        dump_img("lbp_normal_0", &dst);
    }
    c.bit_threshold.s8_val = -5;
    if (!done("lbp_normal_m5", ss_mpi_ive_lbp(&h, &src, &dst, &c, TD_TRUE), h)) {
        dump_img("lbp_normal_m5", &dst);
    }
    c.mode = OT_IVE_LBP_COMPARE_MODE_ABS;
    c.bit_threshold.u8_val = 10;
    if (!done("lbp_abs_10", ss_mpi_ive_lbp(&h, &src, &dst, &c, TD_TRUE), h)) {
        dump_img("lbp_abs_10", &dst);
    }
    freeimg(&src);
    freeimg(&dst);
}

static void cap_norm_grad(void)
{
    ot_svp_img src, dh, dv, dhv;
    ot_ive_norm_grad_ctrl c = { 0 };
    ot_ive_handle h;
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&dh, OT_SVP_IMG_TYPE_S8C1, W, H) ||
        mk(&dv, OT_SVP_IMG_TYPE_S8C1, W, H) || mk(&dhv, OT_SVP_IMG_TYPE_S8C2_PACKAGE, W, H)) {
        return;
    }
    fill_u8(&src, 4);
    memcpy(c.mask, g_sobel, sizeof(c.mask));
    c.norm = 3;
    for (int out = 0; out < 4; out++) {
        c.out_ctrl = out;
        snprintf(n, sizeof(n), "norm_grad_out%d", out);
        if (done(n, ss_mpi_ive_norm_grad(&h, &src, out == 0 || out == 1 ? &dh : NULL,
                                         out == 0 || out == 2 ? &dv : NULL,
                                         out == 3 ? &dhv : NULL, &c, TD_TRUE), h)) {
            continue;
        }
        if (out == 0 || out == 1) {
            snprintf(n, sizeof(n), "norm_grad_out%d_h", out);
            dump_img(n, &dh);
        }
        if (out == 0 || out == 2) {
            snprintf(n, sizeof(n), "norm_grad_out%d_v", out);
            dump_img(n, &dv);
        }
        if (out == 3) {
            dump_img("norm_grad_out3_hv", &dhv);
        }
    }
    freeimg(&src);
    freeimg(&dh);
    freeimg(&dv);
    freeimg(&dhv);
}

static void cap_st(void)
{
    ot_svp_img src, cand;
    ot_svp_mem_info corner;
    ot_ive_st_cand_corner_ctrl c = { 0 };
    ot_ive_st_corner_ctrl cc = { 0 };
    ot_ive_handle h;
    td_u32 stride = sample_common_ive_calc_stride(W, OT_IVE_ALIGN);
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&cand, OT_SVP_IMG_TYPE_U8C1, W, H)) {
        return;
    }
    if (mkmem(&c.mem, sizeof(td_u32) * stride * H +
                      sizeof(ot_ive_st_max_eig_val)) ||
        mkmem(&corner, sizeof(ot_ive_st_corner_info))) {
        return;
    }
    c.quality_level = 25;
    for (int s = 0; s < 2; s++) {
        fill_u8(&src, 5 + s * 9);
        snprintf(n, sizeof(n), "st_s%d_cand_corner", s);
        if (done(n, ss_mpi_ive_st_cand_corner(&h, &src, &cand, &c, TD_TRUE), h)) {
            continue;
        }
        snprintf(n, sizeof(n), "st_s%d_cand", s);
        dump_img(n, &cand);
        /*
         * c.mem is the engine's own buffer: one U32 eigenvalue per pixel
         * (stride-padded) and the picture's maximum after them.  Dumping
         * it gives the raw statistic the U8 candidate map is scaled from.
         */
        snprintf(n, sizeof(n), "st_s%d_eig", s);
        {
            FILE *f = out_file(n);

            if (f) {
                for (td_u32 y = 0; y < H; y++) {
                    fwrite(PTR(td_u8, c.mem.virt_addr) + y * stride * sizeof(td_u32),
                           W * sizeof(td_u32), 1, f);
                }
                fclose(f);
            }
        }
        snprintf(n, sizeof(n), "st_s%d_max_eig", s);
        dump_mem(n, c.mem.virt_addr + sizeof(td_u32) * stride * H, 2);
        cc.max_corner_num = 50;
        cc.min_dist = 5;
        if (ss_mpi_ive_st_corner(&cand, &corner, &cc) == TD_SUCCESS) {
            ot_ive_st_corner_info *ci = PTR(ot_ive_st_corner_info, corner.virt_addr);

            snprintf(n, sizeof(n), "st_s%d_corner", s);
            dump_mem(n, corner.virt_addr,
                     sizeof(td_u16) + 2 + ci->corner_num * sizeof(ot_svp_point_u16));
            printf("ok st_corner (%u)\n", ci->corner_num);
        } else {
            printf("FAIL st_corner\n");
            g_fail++;
        }
    }
    freemem(&c.mem);
    freemem(&corner);
    freeimg(&src);
    freeimg(&cand);
}

#define LK_W    256
#define LK_H    256
#define LK_LVL  2
#define LK_PTS  16

/* pyramid level l of a scene shifted by (dx, dy) / 8 pixels */
static void fill_scene(ot_svp_img *img, int l, int dx, int dy)
{
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int sx = ((int)x << l) * 8 - dx, sy = ((int)y << l) * 8 - dy;
            int v = ((sx / 8 / 16 + sy / 8 / 16) & 1) ? 200 : 40;

            v += (sx / 8) % 16 * 2 + (sy / 8) % 12;
            row[x] = v > 255 ? 255 : v;
        }
    }
}

static void cap_lk(void)
{
    ot_svp_img prev[LK_LVL + 1], next[LK_LVL + 1];
    ot_svp_mem_info pp, np, st, er;
    ot_ive_lk_optical_flow_pyr_ctrl c = { 0 };
    ot_ive_handle h;

    for (int l = 0; l <= LK_LVL; l++) {
        if (mk(&prev[l], OT_SVP_IMG_TYPE_U8C1, LK_W >> l, LK_H >> l) ||
            mk(&next[l], OT_SVP_IMG_TYPE_U8C1, LK_W >> l, LK_H >> l)) {
            return;
        }
        fill_scene(&prev[l], l, 0, 0);
        fill_scene(&next[l], l, 13, -9);       /* +1.6, -1.1 pixels */
    }
    if (mkmem(&pp, sample_common_ive_calc_stride(sizeof(ot_svp_point_s25q7) * LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&np, sample_common_ive_calc_stride(sizeof(ot_svp_point_s25q7) * LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&st, sample_common_ive_calc_stride(LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&er, sample_common_ive_calc_stride(sizeof(td_u9q7) * LK_PTS, OT_IVE_ALIGN))) {
        return;
    }
    for (int i = 0; i < LK_PTS; i++) {
        ot_svp_point_s25q7 *p = PTR(ot_svp_point_s25q7, pp.virt_addr) + i;

        p->x = (48 + (i % 4) * 48) << 7;        /* on the checker corners */
        p->y = (48 + (i / 4) * 48) << 7;
    }
    c.out_mode = OT_IVE_LK_OPTICAL_FLOW_PYR_OUT_MODE_BOTH;
    c.use_init_flow = TD_FALSE;
    c.points_num = LK_PTS;
    c.max_level = LK_LVL;
    c.min_eig_val_threshold = 100;
    c.iter_cnt = 10;
    c.eps = 2;
    if (!done("lk_optical_flow_pyr", ss_mpi_ive_lk_optical_flow_pyr(&h, prev, next, &pp, &np,
                                                                   &st, &er, &c, TD_TRUE), h)) {
        dump_mem("lk_next_points", np.virt_addr, sizeof(ot_svp_point_s25q7) * LK_PTS);
        dump_mem("lk_status", st.virt_addr, LK_PTS);
        dump_mem("lk_err", er.virt_addr, sizeof(td_u9q7) * LK_PTS);
    }
    for (int l = 0; l <= LK_LVL; l++) {
        freeimg(&prev[l]);
        freeimg(&next[l]);
    }
    freemem(&pp);
    freemem(&np);
    freemem(&st);
    freemem(&er);
}

static void cap_gmm2(void)
{
    ot_svp_img src, factor, fg, bg, match;
    ot_svp_mem_info model;
    ot_ive_gmm2_ctrl c = { 0 };
    ot_ive_handle h;
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&factor, OT_SVP_IMG_TYPE_U16C1, W, H) ||
        mk(&fg, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&bg, OT_SVP_IMG_TYPE_U8C1, W, H) ||
        mk(&match, OT_SVP_IMG_TYPE_U8C1, W, H)) {
        return;
    }
    c.var_rate = 1;
    c.model_num = 3;
    c.max_var = (16 * 16) << 7;
    c.min_var = (8 * 8) << 7;
    c.global_sns_factor = 8;
    c.sns_factor_mode = OT_IVE_GMM2_SNS_FACTOR_MODE_GLOBAL;
    c.freq_threshold = 12000;
    c.freq_init_val = 20000;
    c.freq_add_factor = 0xEF;
    c.freq_reduce_factor = 0xFF00;
    c.life_threshold = 500;
    c.life_update_factor_mode = OT_IVE_GMM2_LIFE_UPDATE_FACTOR_MODE_GLOBAL;
    if (mkmem(&model, c.model_num * sizeof(td_u64) * W * H)) {
        return;
    }
    for (int f = 1; f <= 6; f++) {
        fill_u8(&src, 10);
        if (f >= 4) {                           /* something walks in */
            for (td_u32 y = 20; y < 36; y++) {
                memset(PTR(td_u8, src.virt_addr[0]) + y * src.stride[0] + 8 * f, 250, 12);
            }
        }
        c.global_life_update_factor = 0xFFFF / f;
        snprintf(n, sizeof(n), "gmm2_f%d", f);
        if (done(n, ss_mpi_ive_gmm2(&h, &src, &factor, &fg, &bg, &match, &model, &c,
                                    TD_TRUE), h)) {
            continue;
        }
        snprintf(n, sizeof(n), "gmm2_f%d_fg", f);
        dump_img(n, &fg);
        snprintf(n, sizeof(n), "gmm2_f%d_bg", f);
        dump_img(n, &bg);
        snprintf(n, sizeof(n), "gmm2_f%d_match", f);
        dump_img(n, &match);
    }
    dump_mem("gmm2_model", model.virt_addr, c.model_num * sizeof(td_u64) * W * H);
    freemem(&model);
    freeimg(&src);
    freeimg(&factor);
    freeimg(&fg);
    freeimg(&bg);
    freeimg(&match);
}

static void cap_acc_dot_alpha(void)
{
    ot_svp_img a, b, al, d, s1, s2, d32;
    ot_ive_acc_ctrl ac = { 0 };
    ot_ive_image_dot_ctrl dc = { 0 };
    ot_ive_handle h;
    char n[64];

    if (mk(&a, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&b, OT_SVP_IMG_TYPE_U8C1, W, H) ||
        mk(&al, OT_SVP_IMG_TYPE_U8C1, W, H) || mk(&d, OT_SVP_IMG_TYPE_U8C1, W, H) ||
        mk(&s1, OT_SVP_IMG_TYPE_S32C1, W, H) || mk(&s2, OT_SVP_IMG_TYPE_S32C1, W, H) ||
        mk(&d32, OT_SVP_IMG_TYPE_S32C1, W, H)) {
        return;
    }
    fill_u8(&a, 6);
    fill_u8(&b, 7);
    fill_bytes(al.virt_addr[0], al.stride[0] * H, 3);
    /* a few exact 255s for ACC's map mode */
    for (td_u32 y = 0; y < H; y += 5) {
        PTR(td_u8, b.virt_addr[0])[y * b.stride[0] + y] = 255;
    }
    for (int m = 0; m < 3; m++) {
        ac.mode = m;
        snprintf(n, sizeof(n), "acc_mode%d", m);
        if (!done(n, ss_mpi_ive_acc(&h, &a, &b, &d, &ac, TD_TRUE), h)) {
            dump_img(n, &d);
        }
    }
    if (!done("alpha_blending", ss_mpi_ive_alpha_blending(&h, &a, &b, &al, &d, TD_TRUE), h)) {
        dump_img("alpha_blending", &d);
    }
    /* S17Q15 values, both signs, some large */
    for (td_u32 y = 0; y < H; y++) {
        for (td_u32 x = 0; x < W; x++) {
            PTR(td_s32, s1.virt_addr[0])[y * s1.stride[0] + x] = ((int)x - 32) * 4099 + y * 77;
            PTR(td_s32, s2.virt_addr[0])[y * s2.stride[0] + x] = ((int)y - 20) * 3001 - x * 911;
        }
    }
    dc.mode = OT_IVE_IMAGE_DOT_MODE_ROUND_INT;
    if (!done("image_dot", ss_mpi_ive_image_dot(&h, &s1, &s2, &d32, &dc, TD_TRUE), h)) {
        dump_img("image_dot", &d32);
    }
    freeimg(&a);
    freeimg(&b);
    freeimg(&al);
    freeimg(&d);
    freeimg(&s1);
    freeimg(&s2);
    freeimg(&d32);
}

static void cap_csc(void)
{
    ot_svp_img yuv, rgb, rgbp, yuv2;
    ot_ive_csc_ctrl c = { 0 };
    ot_ive_handle h;
    char n[64];

    if (mk(&yuv, OT_SVP_IMG_TYPE_YUV420SP, W, H) || mk(&rgb, OT_SVP_IMG_TYPE_U8C3_PACKAGE, W, H) ||
        mk(&rgbp, OT_SVP_IMG_TYPE_U8C3_PLANAR, W, H) || mk(&yuv2, OT_SVP_IMG_TYPE_YUV420SP, W, H)) {
        return;
    }
    /*
     * Three input pictures per direction: one picture pins the Q10 matrix
     * but leaves its rounding constant a few 1024ths wide, and every
     * further picture narrows it (the coefficients are not in the task,
     * so they are fitted from these outputs).
     */
    for (int s = 0; s < 4; s++) {
        if (s == 3) {
            /* Y over 0..255 four times, and every chroma pair once */
            fill_ramp(yuv.virt_addr[0], yuv.stride[0], W, H, 1);
            fill_ramp(yuv.virt_addr[1], yuv.stride[1], W, H / 2, 1);
        } else {
            fill_u8(&yuv, 8 + s * 5);
            fill_bytes(yuv.virt_addr[1], yuv.stride[1] * H / 2, 8 + s * 5);
        }
        /* V5 has no HSV / LAB modes (4..7): "mode must be in [0, 3] or [8, 12)" */
        for (int m = 0; m < 4; m++) {
            c.mode = m;
            snprintf(n, sizeof(n), "csc_s%d_mode%d_package", s, m);
            if (!done(n, ss_mpi_ive_csc(&h, &yuv, &rgb, &c, TD_TRUE), h)) {
                dump_img(n, &rgb);
            }
        }
        c.mode = OT_IVE_CSC_MODE_VIDEO_BT601_YUV_TO_RGB;
        snprintf(n, sizeof(n), "csc_s%d_mode0_planar", s);
        if (!done(n, ss_mpi_ive_csc(&h, &yuv, &rgbp, &c, TD_TRUE), h)) {
            dump_img(n, &rgbp);
        }
        if (s == 3) {
            fill_ramp(rgb.virt_addr[0], rgb.stride[0] * 3, W * 3, H, 1);
        } else {
            fill_bytes(rgb.virt_addr[0], rgb.stride[0] * H * 3, 9 + s * 5);
        }
        for (int m = 8; m < 12; m++) {
            c.mode = m;
            snprintf(n, sizeof(n), "csc_s%d_mode%d_to_sp420", s, m);
            if (!done(n, ss_mpi_ive_csc(&h, &rgb, &yuv2, &c, TD_TRUE), h)) {
                dump_img(n, &yuv2);
            }
        }
    }
    freeimg(&yuv);
    freeimg(&rgb);
    freeimg(&rgbp);
    freeimg(&yuv2);
}

static void cap_resize(void)
{
    ot_svp_img src[2], dst[2];
    ot_ive_resize_ctrl c = { 0 };
    ot_ive_handle h;
    static const td_u32 dw[] = { 40, 128 }, dh[] = { 24, 96 };

    for (int i = 0; i < 2; i++) {
        if (mk(&src[i], OT_SVP_IMG_TYPE_U8C1, W, H) ||
            mk(&dst[i], OT_SVP_IMG_TYPE_U8C1, dw[i], dh[i])) {
            return;
        }
        fill_u8(&src[i], 11 + i);
    }
    c.mode = OT_IVE_RESIZE_MODE_LINEAR;
    c.num = 2;
    if (mkmem(&c.mem, 12 + 64 * 2)) {
        return;
    }
    if (!done("resize", ss_mpi_ive_resize(&h, src, dst, &c, TD_TRUE), h)) {
        dump_img("resize_40x24", &dst[0]);
        dump_img("resize_128x96", &dst[1]);
    }
    freemem(&c.mem);
    for (int i = 0; i < 2; i++) {
        freeimg(&src[i]);
        freeimg(&dst[i]);
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s OUTDIR\n", argv[0]);
        return 2;
    }
    g_out = argv[1];
    mkdir(g_out, 0755);
    if (sample_common_ive_check_mpi_init() != TD_TRUE) {   /* TD_TRUE, not TD_SUCCESS */
        printf("FAIL mpi init\n");
        return 1;
    }
    cap_mag_and_ang();
    cap_canny();
    cap_lbp();
    cap_norm_grad();
    cap_st();
    cap_lk();
    cap_gmm2();
    cap_acc_dot_alpha();
    cap_csc();
    cap_resize();
    sample_common_ive_mpi_exit();
    printf("=== capture done, %d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
