/* Turn a capture into the set that is applied to the camera and exported.
 *
 *   xm-isp-finish CAPTURE_DIR BASE_DIR OUT_DIR
 *
 * Built against the SDK headers for the build host: these structures hold
 * no pointers or longs, so their layout is the camera's (build.sh checks the
 * sizes against layout.json).
 *
 * What the capture cannot be applied as it stands, and why:
 * - csc, ainr, ae_*: the App writes them from its user settings and its run
 *   time AE control, not from tables (CSC saturation 0 without its colour
 *   settings; AINR thresholds of its day mode; AE limits from its exposure
 *   settings). The streamer owns them; ae-ainr sets the AE curve and AINR
 *   switch points the App runs.
 * - op types: the App fills the sharpen, LCE and CA auto tables and leaves
 *   the op type alone, which on its SDK defaults to auto and on the
 *   ShiMetaPi SDK to manual. Set to auto, as the stock camera's umap shows.
 * - AWB: the App runs its own white balance (bypass 1) on the SDK's
 *   statistics. Its calibration (static gains, curve) is kept and the SDK's
 *   AWB runs on it: bypass 0, colour temperature limits from the base.
 * - BNR: AINR needs BNR's temporal half after it (ainr_pos_switch POST,
 *   tnr on); the App turns that on only when it switches AINR on. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xmedia_awb.h>
#include <xmedia_isp.h>

static const char *const keep[] = {"awb", "awb_ccm", "awb_color_tone",
    "awb_saturation", "awb_scene_adj", "blc", "bnr", "ca", "dehaze",
    "demosaic", "dpc_dynamic", "drc", "gamma", "lcac", "lce",
    "mesh_shading", "sharpen", "statistics", "stnr", NULL};

static int rd(const char *d, const char *n, void *b, size_t sz) {
    char p[512];
    snprintf(p, sizeof p, "%s/%s.bin", d, n);
    FILE *f = fopen(p, "rb");
    if (!f)
        return 0;
    size_t got = fread(b, 1, sz, f);
    fclose(f);
    return got == sz;
}

static int wr(const char *d, const char *n, const void *b, size_t sz) {
    char p[512];
    snprintf(p, sizeof p, "%s/%s.bin", d, n);
    FILE *f = fopen(p, "wb");
    if (!f || fwrite(b, 1, sz, f) != sz) {
        perror(p);
        return 0;
    }
    return fclose(f) == 0;
}

#define FIX(name, type, body)                                                  \
    do {                                                                       \
        type v;                                                                \
        if (rd(out, name, &v, sizeof v)) {                                     \
            body;                                                              \
            wr(out, name, &v, sizeof v);                                       \
        }                                                                      \
    } while (0)

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s CAPTURE_DIR BASE_DIR OUT_DIR\n", argv[0]);
        return 2;
    }
    const char *cap = argv[1], *base = argv[2], *out = argv[3];
    static char buf[1 << 16];
    for (const char *const *n = keep; *n; n++) {
        char p[512];
        snprintf(p, sizeof p, "%s/%s.bin", cap, *n);
        FILE *f = fopen(p, "rb");
        if (!f) {
            printf("%-16s not in the capture\n", *n);
            continue;
        }
        size_t len = fread(buf, 1, sizeof buf, f);
        fclose(f);
        if (!wr(out, *n, buf, len))
            return 1;
    }
    const xmedia_video_operation_mode AUTO = XMEDIA_VIDEO_OPERATION_MODE_AUTO;
    FIX("sharpen", xmedia_isp_sharpen_attr, v.op_type = AUTO);
    FIX("lce", xmedia_isp_lce_attr, v.op_type = AUTO);
    FIX("ca", xmedia_isp_ca_attr, v.op_type = AUTO);
    FIX("bnr", xmedia_isp_bnr_attr,
        v.ainr_pos_switch = XMEDIA_ISP_BNR_AINR_POS_POST;
        v.tnr_enable = XMEDIA_TRUE);
    xmedia_awb_attr b;
    if (!rd(base, "awb", &b, sizeof b)) {
        fprintf(stderr, "%s/awb.bin: the base is needed for the AWB limits\n",
            base);
        return 1;
    }
    FIX("awb", xmedia_awb_attr,
        v.bypass = XMEDIA_FALSE;
        v.auto_attr.high_ct = b.auto_attr.high_ct;
        v.auto_attr.low_ct = b.auto_attr.low_ct);
    printf("%s: ready for attrtool apply\n", out);
    return 0;
}
