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
#include <math.h>

#include "sample_common_ive.h"
#include "ss_mpi_ive.h"

static const char *g_out;
static int g_fail;

#define PTR(t, a)   ((t *)(td_uintptr_t)(a))
#define MIN2(a, b)  ((a) < (b) ? (a) : (b))

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

/*
 * A picture whose normalised gradients are a controlled sweep: with
 * src = (x^2 * ax + y^2 * ay) / 64 the Sobel's gx grows with x and gy
 * with y, so a 3x3 window's structure tensor is near-diagonal and its
 * two entries sweep the picture.  That is what makes the eigen op's
 * arithmetic readable: every (A, C) pair appears somewhere, with B ~ 0.
 */
static void fill_quad(ot_svp_img *img, int ax, int ay, int off)
{
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int v = off + ((int)(x * x) * ax + (int)(y * y) * ay) / 64;

            row[x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

/* a tilted plane plus a fine ripple, so the gradient direction turns */
static void fill_swirl(ot_svp_img *img, int k, int amp)
{
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int v = 128 + ((int)x * k) / 4 - ((int)y * k) / 4 +
                    amp * (((x / 3) % 2) ? 1 : -1) * (((y / 3) % 2) ? 1 : -1);

            row[x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

/*
 * A separable picture, src = F(x) + G(y), so the Sobel's gx depends only
 * on x and gy only on y.  A 3x3 window's structure tensor is then
 * A = 3 sum gx^2, C = 3 sum gy^2 and B = (sum gx)(sum gy), all of which
 * these sawtooth ramps drive directly -- which is what makes the eigen
 * op's arithmetic readable over a structured sweep instead of the clumps
 * a random picture gives.
 */
static void fill_separable(ot_svp_img *img, int ax, int ay, int period)
{
    int fx[128], gy[128];
    int v = 0;

    for (td_u32 x = 0; x < img->width && x < 128; x++) {
        int step = ((x / period) % 2) ? -ax : ax;

        fx[x] = v;
        v += step * (1 + (int)(x % 3));
        if (v < 0 || v > 120) {
            v = v < 0 ? 0 : 120;
        }
    }
    v = 0;
    for (td_u32 y = 0; y < img->height && y < 128; y++) {
        int step = ((y / period) % 2) ? -ay : ay;

        gy[y] = v;
        v += step * (1 + (int)(y % 2));
        if (v < 0 || v > 120) {
            v = v < 0 ? 0 : 120;
        }
    }
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int p = fx[x] + gy[y];

            row[x] = p < 0 ? 0 : p > 255 ? 255 : p;
        }
    }
}

/*
 * A picture built from a wanted gradient sequence, to answer one
 * question: does the eigen op's measure depend on the window's
 * structure tensor alone, or on the individual gradients?  With
 * src = F(x) + G(y) the Sobel's gx is (F(x+1) - F(x-1)) / 2, so F can
 * be built backwards from any gx sequence, odd and even columns being
 * independent chains.  The sequences below put triples with the *same*
 * sum and sum of squares -- (1,5,6) against (2,3,7), say -- next to each
 * other: their windows share A, B and C exactly while their gradients
 * differ, so the two answers settle it.
 *
 * Each triple is followed by three columns that walk the ramp back to
 * where it started, because F's odd and even columns are *separate*
 * chains: placing the plain negation cancels neither, and the drift
 * accumulates until the clamp below changes the later triples'
 * gradients, so a pair meant to be matched is not one.  The even chain
 * sees positions 1, 3, 5 of the group and the odd chain 0, 2, 4, so
 * (t0, t1, t2, -t1, -(t0 + t2), 0) returns both to their start while
 * leaving the window at position 1 reading the triple itself.
 */
static const int g_tri[][3] = {
    { 1, 5, 6 }, { 2, 3, 7 }, { 0, 4, 5 }, { 1, 2, 6 },
    { 2, 5, 5 }, { 3, 3, 6 }, { 0, 6, 6 }, { 2, 2, 8 },
    { 3, 6, 6 }, { 4, 4, 7 }, { 0, 3, 3 }, { 1, 1, 4 },
};
#define TRI_NUM     (int)(sizeof(g_tri) / sizeof(g_tri[0]))
#define TRI_PITCH   7
#define TRI_BASE    4
#define TRI_PER_PIC 8                   /* what fits across 64 columns */

/* the column at the centre of the t'th triple placed, where it is read */
static int tri_centre(int t)
{
    return TRI_BASE + t * TRI_PITCH + 1;
}

static void build_ramp(int *out, int n, const int *want, int start)
{
    out[0] = start;
    out[1] = start;
    for (int i = 1; i + 1 < n; i++) {
        out[i + 1] = out[i - 1] + 2 * want[i];
    }
}

static void fill_tensor_probe(ot_svp_img *img, int gy_a, int gy_b, int gy_c,
                              int first)
{
    int fx[256], gyv[256], wx[256] = { 0 }, wy[256] = { 0 };
    int w = MIN2((int)img->width, 256), h = MIN2((int)img->height, 256);

    for (int t = 0; t < TRI_PER_PIC; t++) {
        const int *tri = g_tri[(first + t) % TRI_NUM];
        int c = tri_centre(t);

        if (c + 4 >= w) {
            break;                      /* no room for the triple and its undo */
        }
        for (int i = 0; i < 3; i++) {
            wx[c - 1 + i] = tri[i];
        }
        wx[c + 2] = -tri[1];            /* both chains back to the start */
        wx[c + 3] = -(tri[0] + tri[2]);
        wx[c + 4] = 0;
    }
    /* the rows carry one triple and the same undo, over and over */
    for (int y = 0; y + 5 < h; y += 6) {
        wy[y] = gy_a;
        wy[y + 1] = gy_b;
        wy[y + 2] = gy_c;
        wy[y + 3] = -gy_b;
        wy[y + 4] = -(gy_a + gy_c);
        wy[y + 5] = 0;
    }
    build_ramp(fx, w, wx, 100);
    build_ramp(gyv, h, wy, 0);
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int v = fx[MIN2((int)x, w - 1)] + gyv[MIN2((int)y, h - 1)];

            row[x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

/*
 * The reciprocal probe (#300).  Op 25's measure is a function of the
 * window's trace and determinant alone, homogeneous of degree 1, and
 * `(det / tr) (1 + det / tr^2) / 16` describes it to 0.25% -- about twice
 * the output's own quantisation, so what is left is structure.  The
 * standing guess is a truncating reciprocal of the trace inside the
 * engine, which would show up as `L` departing from that expression by an
 * amount that depends on the trace and not on the determinant.
 *
 * Telling the two apart needs many determinants at *one* trace, which
 * `fill_tensor_probe` cannot give: it places one row triple per picture.
 * So here both axes carry a designed triple and the tensors are the outer
 * product of the two.  With src = F(x) + G(y) the Sobel's gx is a function
 * of x alone and gy of y alone (the cross terms cancel), so for the window
 * at the centre of column group i and row group j
 *
 *      A = 3 (a^2 + b^2 + c^2)     over the column triple (a, b, c)
 *      C = 3 (d^2 + e^2 + f^2)     over the row triple (d, e, f)
 *      B = (a + b + c) (d + e + f)
 *
 * The rows carry (d, 0, -d), so C = 6 d^2 and B = 0; the columns carry
 * (t0, t1, -t0), so A = 3 (2 t0^2 + t1^2), which is dense enough to hit a
 * wanted value.  Each picture's A_i is chosen as `trace - C_i`, so its
 * diagonal holds one tensor per group pair with the *same* trace and a
 * different determinant, and the off-diagonal pairs sweep the trace over
 * everything from trace - 13818 to trace + 13818 as a bonus.
 *
 * Each triple is followed by its undo, as in fill_tensor_probe: F's odd and
 * even columns are separate chains, and (t0, t1, -t0) completes to
 * (t0, t1, -t0, -t1, 0, 0) -- a pitch of 6 columns, which also keeps the
 * ramp inside [60, 252] for gradients up to 48, so nothing clips.
 *
 * Two of the four traces sit either side of 8192 and two either side of
 * 16384: if the reciprocal is bucketed by exponent, the pairs straddle a
 * bucket boundary.
 */
#define RP_W        192
#define RP_H        192
#define RP_PITCH    6
#define RP_BASE     4
#define RP_NMAX     31

typedef struct {
    td_u32 trace;
    td_u32 num;
    td_u8 d[RP_NMAX];               /* the row triple is (d, 0, -d) */
    td_u8 t0[RP_NMAX];              /* the column triple is (t0, t1, -t0) */
    td_u8 t1[RP_NMAX];
} IveRecipDesign;

static const IveRecipDesign g_recip[] = {
    {   /* trace 7977, 25 determinants: L 24..155 */
        7977, 25,
        { 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 34, 35 },
        { 35, 18, 35, 14, 9, 34, 11, 0, 31, 26, 9, 13, 24, 25, 16, 21, 22, 17, 24, 1, 22, 3, 16, 13, 8 },
        { 9, 43, 3, 45, 47, 3, 45, 47, 15, 27, 43, 39, 25, 21, 33, 25, 21, 27, 7, 33, 3, 29, 15, 3, 9 },
    },
    {   /* trace 8313, 25 determinants: L 24..162 */
        8313, 25,
        { 8, 10, 11, 13, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 35, 36 },
        { 29, 35, 30, 34, 34, 33, 28, 29, 32, 29, 20, 29, 26, 7, 26, 23, 17, 20, 11, 20, 19, 16, 13, 10, 7 },
        { 31, 11, 27, 11, 3, 9, 25, 21, 1, 17, 33, 11, 19, 39, 13, 19, 25, 17, 27, 7, 1, 9, 11, 11, 9 },
    },
    {   /* trace 16377, 27 determinants: L 37..319 */
        16377, 27,
        { 10, 14, 16, 17, 19, 20, 22, 23, 26, 28, 29, 31, 32, 33, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47 },
        { 47, 43, 37, 40, 38, 43, 45, 44, 37, 41, 28, 34, 35, 40, 31, 32, 26, 35, 14, 33, 28, 31, 20, 23, 22, 23, 16 },
        { 29, 37, 47, 41, 43, 31, 21, 23, 37, 23, 47, 35, 31, 9, 35, 31, 37, 11, 45, 9, 23, 3, 31, 23, 21, 13, 23 },
    },
    {   /* trace 16425, 20 determinants: L 37..319 */
        16425, 20,
        { 10, 11, 13, 17, 19, 20, 21, 24, 30, 31, 34, 35, 36, 39, 41, 42, 43, 45, 46, 48 },
        { 45, 48, 48, 42, 42, 45, 46, 41, 35, 36, 39, 30, 31, 34, 24, 31, 24, 20, 21, 17 },
        { 35, 25, 23, 37, 35, 25, 19, 31, 35, 31, 11, 35, 31, 11, 31, 5, 25, 25, 19, 17 },
    },
};

/* coverage
 * trace  7977: n=25  C=[384, 486, 600, 726, 864, 1014, 1176, 1350, 1536, 1734, 1944, 2400, 2646, 2904, 3174, 3456, 3750, 4056, 4374, 4704, 5046, 5400, 5766, 6936, 7350]
 *              A=[7593, 7491, 7377, 7251, 7113, 6963, 6801, 6627, 6441, 6243, 6033, 5577, 5331, 5073, 4803, 4521, 4227, 3921, 3603, 3273, 2931, 2577, 2211, 1041, 627]
 *              L=[24, 30, 37, 45, 53, 61, 70, 80, 89, 99, 109, 127, 135, 142, 148, 152, 155, 155, 154, 149, 142, 133, 120, 63, 39]
 * trace  8313: n=25  C=[384, 600, 726, 1014, 1350, 1536, 1734, 1944, 2166, 2400, 2646, 2904, 3174, 3456, 3750, 4056, 4704, 5046, 5400, 5766, 6144, 6534, 6936, 7350, 7776]
 *              A=[7929, 7713, 7587, 7299, 6963, 6777, 6579, 6369, 6147, 5913, 5667, 5409, 5139, 4857, 4563, 4257, 3609, 3267, 2913, 2547, 2169, 1779, 1377, 963, 537]
 *              L=[24, 37, 45, 61, 80, 90, 100, 109, 119, 128, 137, 145, 151, 156, 160, 162, 159, 153, 145, 134, 119, 102, 82, 58, 33]
 * trace 16377: n=27  C=[600, 1176, 1536, 1734, 2166, 2400, 2904, 3174, 4056, 4704, 5046, 5766, 6144, 6534, 6936, 7350, 8214, 8664, 9126, 9600, 10086, 10584, 11094, 11616, 12150, 12696, 13254]
 *              A=[15777, 15201, 14841, 14643, 14211, 13977, 13473, 13203, 12321, 11673, 11331, 10611, 10233, 9843, 9441, 9027, 8163, 7713, 7251, 6777, 6291, 5793, 5283, 4761, 4227, 3681, 3123]
 *              L=[37, 73, 94, 106, 131, 144, 171, 184, 226, 252, 264, 286, 295, 304, 310, 315, 319, 318, 314, 308, 299, 287, 272, 254, 233, 209, 182]
 * trace 16425: n=20  C=[600, 726, 1014, 1734, 2166, 2400, 2646, 3456, 5400, 5766, 6936, 7350, 7776, 9126, 10086, 10584, 11094, 12150, 12696, 13824]
 *              A=[15825, 15699, 15411, 14691, 14259, 14025, 13779, 12969, 11025, 10659, 9489, 9075, 8649, 7299, 6339, 5841, 5331, 4275, 3729, 2601]
 *              L=[37, 45, 63, 106, 131, 144, 157, 198, 276, 286, 311, 316, 319, 315, 300, 288, 274, 235, 211, 155]
 */

/* the column (or row) at the centre of the i'th group, where it is read */
static int rp_centre(int i)
{
    return RP_BASE + i * RP_PITCH + 1;
}

static void fill_recip_probe(ot_svp_img *img, const IveRecipDesign *p)
{
    int wx[RP_W + 8] = { 0 }, wy[RP_H + 8] = { 0 };
    int fx[RP_W + 8], gy[RP_H + 8];

    for (td_u32 i = 0; i < p->num; i++) {
        int c = rp_centre((int)i);

        if (c + 4 >= RP_W) {
            break;
        }
        wx[c - 1] = p->t0[i];
        wx[c] = p->t1[i];
        wx[c + 1] = -(int)p->t0[i];
        wx[c + 2] = -(int)p->t1[i];
        wy[c - 1] = p->d[i];
        wy[c] = 0;
        wy[c + 1] = -(int)p->d[i];
    }
    build_ramp(fx, RP_W, wx, 30);
    build_ramp(gy, RP_H, wy, 30);
    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            int v = fx[MIN2((int)x, RP_W - 1)] + gy[MIN2((int)y, RP_H - 1)];

            row[x] = v < 0 ? 0 : v > 255 ? 255 : v;
        }
    }
}

/*
 * The engine writes op 25's measure as U16 with a 2 x stride byte pitch at
 * ctrl->mem + 2 * stride * h, and its maximum at + 4 * stride * h -- not as
 * the U32 array per pixel that the mem size the API asks for suggests.  So
 * dump it from there, rows packed, and the maximum after it.
 */
static void cap_st_recip(void)
{
    ot_svp_img src, cand;
    ot_ive_st_cand_corner_ctrl c = { 0 };
    ot_ive_handle h;
    td_u32 stride = sample_common_ive_calc_stride(RP_W, OT_IVE_ALIGN);
    char n[64];

    if (mk(&src, OT_SVP_IMG_TYPE_U8C1, RP_W, RP_H)) {
        return;
    }
    if (mk(&cand, OT_SVP_IMG_TYPE_U8C1, RP_W, RP_H)) {
        freeimg(&src);
        return;
    }
    if (mkmem(&c.mem, sizeof(td_u32) * stride * RP_H +
                      sizeof(ot_ive_st_max_eig_val))) {
        freeimg(&src);
        freeimg(&cand);
        return;
    }
    c.quality_level = 25;
    for (td_u32 p = 0; p < sizeof(g_recip) / sizeof(g_recip[0]); p++) {
        td_u32 tr = g_recip[p].trace;
        td_s32 ret;

        fill_recip_probe(&src, &g_recip[p]);
        snprintf(n, sizeof(n), "strec_tr%u_cand_corner", tr);
        /*
         * in its own statement: the handle is written through &h by the call
         * and read again by done(), and the order of a call's arguments is
         * unspecified
         */
        ret = ss_mpi_ive_st_cand_corner(&h, &src, &cand, &c, TD_TRUE);
        if (done(n, ret, h)) {
            continue;
        }
        snprintf(n, sizeof(n), "strec_tr%u_src", tr);
        dump_img(n, &src);
        snprintf(n, sizeof(n), "strec_tr%u_eig", tr);
        {
            FILE *f = out_file(n);

            if (f) {
                td_u64 base = c.mem.virt_addr + 2 * stride * RP_H;

                for (td_u32 y = 0; y < RP_H; y++) {
                    fwrite(PTR(td_u8, base) + y * stride * sizeof(td_u16),
                           RP_W * sizeof(td_u16), 1, f);
                }
                fclose(f);
            }
        }
        snprintf(n, sizeof(n), "strec_tr%u_max_eig", tr);
        dump_mem(n, c.mem.virt_addr + sizeof(td_u32) * stride * RP_H, 2);
    }
    freemem(&c.mem);
    freeimg(&src);
    freeimg(&cand);
}

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
    /*
     * Thresholds chosen to separate the two rules: a band that touches
     * the strong pixels (20 / 60), one that is almost everything
     * (2 / 10), one where weak is wide and strong rare (10 / 600), and
     * one with no weak band at all (300 / 301).
     */
    static const td_u16 lo[] = { 50, 20, 100, 2, 10, 300 },
                        hi[] = { 150, 60, 400, 10, 600, 301 };
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
        for (int t = 0; t < 6; t++) {
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
    for (int s = 0; s < 12; s++) {
        switch (s) {
        case 0: case 1:
            fill_u8(&src, 5 + s * 9);
            break;
        case 2:
            fill_quad(&src, 6, 1, 0);           /* A >> C, both sweeping */
            break;
        case 3:
            fill_quad(&src, 2, 2, 0);           /* A ~ C */
            break;
        case 4:
            fill_swirl(&src, 8, 20);            /* B large */
            break;
        case 5:
            fill_swirl(&src, 2, 60);
            break;
        case 6:
            fill_separable(&src, 1, 1, 7);
            break;
        case 7:
            fill_separable(&src, 3, 1, 5);
            break;
        case 8:
            fill_separable(&src, 2, 5, 11);
            break;
        case 9:
            fill_separable(&src, 6, 4, 3);
            break;
        case 10:
            fill_tensor_probe(&src, 3, 1, 2, 0);
            break;
        default:
            fill_tensor_probe(&src, 5, 4, 6, 4);
            break;
        }
        snprintf(n, sizeof(n), "st_s%d_cand_corner", s);
        if (done(n, ss_mpi_ive_st_cand_corner(&h, &src, &cand, &c, TD_TRUE), h)) {
            continue;
        }
        snprintf(n, sizeof(n), "st_s%d_src", s);
        dump_img(n, &src);
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

#define LK_W    128
#define LK_H    128
#define LK_LVL  1
#define LK_PTS  16

/*
 * The scene LK is tracked in.  The first one moved by whole pixels only
 * -- it divided the shifted coordinate by 8 -- so a sub-pixel shift
 * produced no sub-pixel motion, and every point of a periodic picture
 * answered the same.  This one is a sum of sinusoids whose periods share
 * no factor with the point spacing, sampled at the shifted coordinate,
 * so the motion really is a fraction of a pixel and each point sees its
 * own texture.  The pictures are dumped: the fit needs the exact bytes,
 * and a sine computed here need not match one computed elsewhere to the
 * last bit.  Level l samples the same scene at twice the step, which is
 * what a correctly built pyramid of a smooth picture holds; the shift is
 * in eighths of a level-0 pixel.
 */
static void fill_wave(ot_svp_img *img, int l, int dx8, int dy8)
{
    const double pi2 = 6.283185307179586;

    for (td_u32 y = 0; y < img->height; y++) {
        td_u8 *row = PTR(td_u8, img->virt_addr[0]) + y * img->stride[0];

        for (td_u32 x = 0; x < img->width; x++) {
            double u = (double)((int)x << l) - dx8 / 8.0;
            double v = (double)((int)y << l) - dy8 / 8.0;
            double p = 128 + 44 * sin(pi2 * u / 13.7) +
                             36 * sin(pi2 * v / 9.3) +
                             26 * sin(pi2 * (u + v) / 21.1) +
                             14 * sin(pi2 * (u - 2 * v) / 31.3);
            int q = (int)(p + 0.5);

            row[x] = q < 0 ? 0 : q > 255 ? 255 : q;
        }
    }
}

/* the tracked points: spread, and not on the scene's periods */
static const struct {
    int x, y;
} g_lk_pts[LK_PTS] = {
    { 29, 23 }, { 41, 58 }, { 67, 31 }, { 83, 71 },
    { 37, 89 }, { 59, 43 }, { 95, 61 }, { 71, 97 },
    { 25, 47 }, { 53, 79 }, { 89, 37 }, { 97, 83 },
    { 33, 67 }, { 47, 101 }, { 79, 53 }, { 61, 29 },
};

static void cap_lk(void)
{
    ot_svp_img prev[LK_LVL + 1], next[LK_LVL + 1];
    ot_svp_mem_info pp, np, st, er;
    ot_ive_lk_optical_flow_pyr_ctrl c = { 0 };
    ot_ive_handle h;
    /* in eighths of a pixel: sub-pixel, a half, whole, and longer ones */
    static const int dxs[] = { 3, 4, 8, 12, 25, -9 },
                     dys[] = { 1, -4, 0, 20, 7, -17 };
    /*
     * One level and one iteration first, then more of each: a single
     * step at a single level is what makes the arithmetic readable, and
     * the rest show the iteration and the pyramid.  (max_level 2 on a
     * 128-pixel picture is refused by the MPI, 0xa01d8007.)
     */
    static const int lvls[] = { 0, 0, 0, LK_LVL }, iters[] = { 1, 2, 10, 10 };
    char n[64];

    for (int l = 0; l <= LK_LVL; l++) {
        if (mk(&prev[l], OT_SVP_IMG_TYPE_U8C1, LK_W >> l, LK_H >> l) ||
            mk(&next[l], OT_SVP_IMG_TYPE_U8C1, LK_W >> l, LK_H >> l)) {
            return;
        }
        fill_wave(&prev[l], l, 0, 0);
        snprintf(n, sizeof(n), "lk_prev_l%d", l);
        dump_img(n, &prev[l]);
    }
    if (mkmem(&pp, sample_common_ive_calc_stride(sizeof(ot_svp_point_s25q7) * LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&np, sample_common_ive_calc_stride(sizeof(ot_svp_point_s25q7) * LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&st, sample_common_ive_calc_stride(LK_PTS, OT_IVE_ALIGN)) ||
        mkmem(&er, sample_common_ive_calc_stride(sizeof(td_u9q7) * LK_PTS, OT_IVE_ALIGN))) {
        return;
    }
    c.out_mode = OT_IVE_LK_OPTICAL_FLOW_PYR_OUT_MODE_BOTH;
    c.use_init_flow = TD_FALSE;
    c.points_num = LK_PTS;
    c.min_eig_val_threshold = 100;
    c.eps = 2;
    for (int k = 0; k < (int)(sizeof(dxs) / sizeof(dxs[0])); k++) {
        for (int l = 0; l <= LK_LVL; l++) {
            fill_wave(&next[l], l, dxs[k] >> l, dys[k] >> l);
            snprintf(n, sizeof(n), "lk_next_d%d_l%d", k, l);
            dump_img(n, &next[l]);
        }
        for (int m = 0; m < (int)(sizeof(lvls) / sizeof(lvls[0])); m++) {
            c.max_level = lvls[m];
            c.iter_cnt = iters[m];
            for (int i = 0; i < LK_PTS; i++) {
                ot_svp_point_s25q7 *q = PTR(ot_svp_point_s25q7, pp.virt_addr) + i;

                q->x = g_lk_pts[i].x << 7;
                q->y = g_lk_pts[i].y << 7;
            }
            memset(PTR(td_u8, np.virt_addr), 0, sizeof(ot_svp_point_s25q7) * LK_PTS);
            memset(PTR(td_u8, st.virt_addr), 0, LK_PTS);
            snprintf(n, sizeof(n), "lk_d%d_m%d", k, m);
            if (done(n, ss_mpi_ive_lk_optical_flow_pyr(&h, prev, next, &pp, &np,
                                                       &st, &er, &c, TD_TRUE), h)) {
                continue;
            }
            snprintf(n, sizeof(n), "lk_d%d_m%d_next", k, m);
            dump_mem(n, np.virt_addr, sizeof(ot_svp_point_s25q7) * LK_PTS);
            snprintf(n, sizeof(n), "lk_d%d_m%d_status", k, m);
            dump_mem(n, st.virt_addr, LK_PTS);
            snprintf(n, sizeof(n), "lk_d%d_m%d_err", k, m);
            dump_mem(n, er.virt_addr, sizeof(td_u9q7) * LK_PTS);
        }
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

/*
 * A second GMM2 sequence, built so the model is readable: the picture is
 * one value everywhere, so every pixel shares a history, and it steps
 * twice.  The model slice after each frame shows the update rule, which
 * the long random-scene run above cannot (its pixels all differ).
 */
static void cap_gmm2_flat(void)
{
    ot_svp_img src, factor, fg, bg, match;
    ot_svp_mem_info model;
    ot_ive_gmm2_ctrl c = { 0 };
    ot_ive_handle h;
    static const int val[] = { 100, 100, 100, 100, 150, 150, 100, 220 };
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
    c.global_life_update_factor = 0xFFFF;
    if (mkmem(&model, c.model_num * sizeof(td_u64) * W * H)) {
        goto out;
    }
    for (int f = 0; f < (int)(sizeof(val) / sizeof(val[0])); f++) {
        for (td_u32 y = 0; y < H; y++) {
            memset(PTR(td_u8, src.virt_addr[0]) + y * src.stride[0], val[f], W);
        }
        snprintf(n, sizeof(n), "gmm2flat_f%d", f);
        if (done(n, ss_mpi_ive_gmm2(&h, &src, &factor, &fg, &bg, &match, &model, &c,
                                    TD_TRUE), h)) {
            continue;
        }
        snprintf(n, sizeof(n), "gmm2flat_f%d_fg", f);
        dump_img(n, &fg);
        snprintf(n, sizeof(n), "gmm2flat_f%d_bg", f);
        dump_img(n, &bg);
        snprintf(n, sizeof(n), "gmm2flat_f%d_match", f);
        dump_img(n, &match);
        /* the first pixels' models: every pixel has the same history */
        snprintf(n, sizeof(n), "gmm2flat_f%d_model", f);
        dump_mem(n, model.virt_addr, c.model_num * sizeof(td_u64) * 16);
    }
    freemem(&model);
out:
    freeimg(&src);
    freeimg(&factor);
    freeimg(&fg);
    freeimg(&bg);
    freeimg(&match);
}

/*
 * A third GMM2 sequence, built to show the match test and the updates:
 * the picture settles flat at 100 and then every column steps by its own
 * index, so one frame holds the boundary for every step size at once.
 * A whole row of models is dumped after each frame.
 */
static void cap_gmm2_step(void)
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
    c.global_life_update_factor = 0xFFFF;
    if (mkmem(&model, c.model_num * sizeof(td_u64) * W * H)) {
        goto out;
    }
    for (int f = 0; f < 10; f++) {
        for (td_u32 y = 0; y < H; y++) {
            td_u8 *row = PTR(td_u8, src.virt_addr[0]) + y * src.stride[0];

            for (td_u32 x = 0; x < W; x++) {
                row[x] = f < 4 ? 100 : (td_u8)(100 + x);
            }
        }
        snprintf(n, sizeof(n), "gmm2step_f%d", f);
        if (done(n, ss_mpi_ive_gmm2(&h, &src, &factor, &fg, &bg, &match, &model, &c,
                                    TD_TRUE), h)) {
            continue;
        }
        snprintf(n, sizeof(n), "gmm2step_f%d_fg", f);
        dump_img(n, &fg);
        snprintf(n, sizeof(n), "gmm2step_f%d_bg", f);
        dump_img(n, &bg);
        snprintf(n, sizeof(n), "gmm2step_f%d_match", f);
        dump_img(n, &match);
        snprintf(n, sizeof(n), "gmm2step_f%d_model", f);
        dump_mem(n, model.virt_addr, c.model_num * sizeof(td_u64) * W);
    }
    freemem(&model);
out:
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
    cap_st_recip();
    cap_lk();
    cap_gmm2();
    cap_gmm2_flat();
    cap_gmm2_step();
    cap_acc_dot_alpha();
    cap_csc();
    cap_resize();
    sample_common_ive_mpi_exit();
    printf("=== capture done, %d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
