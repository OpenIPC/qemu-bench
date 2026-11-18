/* dump: write every attribute as raw bytes to <dir>/<name>.bin
 * apply: read <dir>/<name>.bin for the names given and set them */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xmedia_isp.h>
#include <xmedia_ae.h>
#include <xmedia_awb.h>
typedef int (*getf)(xmedia_u32, void *);
typedef int (*setf)(xmedia_u32, const void *);
#define E(n, t, g, s) {n, sizeof(t), (getf)g, (setf)s}
static struct { const char *n; size_t sz; getf g; setf s; } T[] = {
 E("gamma", xmedia_isp_gamma_attr, xmedia_isp_get_gamma_attr, xmedia_isp_set_gamma_attr),
 E("drc", xmedia_isp_drc_attr, xmedia_isp_get_drc_attr, xmedia_isp_set_drc_attr),
 E("mesh_shading", xmedia_isp_mesh_shading_attr, xmedia_isp_get_mesh_shading_attr, xmedia_isp_set_mesh_shading_attr),
 E("dpc_dynamic", xmedia_isp_dpc_dynamic_attr, xmedia_isp_get_dpc_dynamic_attr, xmedia_isp_set_dpc_dynamic_attr),
 E("bnr", xmedia_isp_bnr_attr, xmedia_isp_get_bnr_attr, xmedia_isp_set_bnr_attr),
 E("dehaze", xmedia_isp_dehaze_attr, xmedia_isp_get_dehaze_attr, xmedia_isp_set_dehaze_attr),
 E("demosaic", xmedia_isp_demosaic_attr, xmedia_isp_get_demosaic_attr, xmedia_isp_set_demosaic_attr),
 E("blc", xmedia_isp_blc_attr, xmedia_isp_get_black_level_attr, xmedia_isp_set_black_level_attr),
 E("lcac", xmedia_isp_lcac_attr, xmedia_isp_get_lcac_attr, xmedia_isp_set_lcac_attr),
 E("wdr", xmedia_isp_wdr_attr, xmedia_isp_get_wdr_attr, xmedia_isp_set_wdr_attr),
 E("lce", xmedia_isp_lce_attr, xmedia_isp_get_lce_attr, xmedia_isp_set_lce_attr),
 E("ca", xmedia_isp_ca_attr, xmedia_isp_get_ca_attr, xmedia_isp_set_ca_attr),
 E("csc", xmedia_isp_csc_attr, xmedia_isp_get_csc_attr, xmedia_isp_set_csc_attr),
 E("sharpen", xmedia_isp_sharpen_attr, xmedia_isp_get_sharpen_attr, xmedia_isp_set_sharpen_attr),
 E("stnr", xmedia_isp_stnr_attr, xmedia_isp_get_stnr_attr, xmedia_isp_set_stnr_attr),
 E("ainr", xmedia_isp_ainr_attr, xmedia_isp_get_ainr_attr, xmedia_isp_set_ainr_attr),
 E("statistics", xmedia_isp_statistics_cfg, xmedia_isp_get_statistics_config, xmedia_isp_set_statistics_config),
 E("smart_info", xmedia_isp_smart_info, xmedia_isp_get_smart_info, xmedia_isp_set_smart_info),
 E("ae_exposure", xmedia_ae_exposure_attr, xmedia_ae_get_exposure_attr, xmedia_ae_set_exposure_attr),
 E("ae_wdr", xmedia_ae_wdr_exposure_attr, xmedia_ae_get_wdr_exposure_attr, xmedia_ae_set_wdr_exposure_attr),
 E("ae_route", xmedia_isp_ae_route, xmedia_ae_get_route_attr, xmedia_ae_set_route_attr),
 E("ae_ext_route", xmedia_isp_ae_ext_route, xmedia_ae_get_ext_route_attr, xmedia_ae_set_ext_route_attr),
 E("ae_smart", xmedia_ae_smart_exposure_attr, xmedia_ae_get_smart_exposure_attr, xmedia_ae_set_smart_exposure_attr),
 E("awb", xmedia_awb_attr, xmedia_awb_get_attr, xmedia_awb_set_attr),
 E("awb_scene_adj", xmedia_awb_scene_adj_attr, xmedia_awb_get_scene_adj_attr, xmedia_awb_set_scene_adj_attr),
 E("awb_ccm", xmedia_awb_ccm_attr, xmedia_awb_get_ccm_attr, xmedia_awb_set_ccm_attr),
 E("awb_color_sector", xmedia_color_sector_attr, xmedia_awb_get_color_sector_attr, xmedia_awb_set_color_sector_attr),
 E("awb_saturation", xmedia_awb_saturation_attr, xmedia_awb_get_saturation_attr, xmedia_awb_set_saturation_attr),
 E("awb_color_tone", xmedia_awb_color_tone_attr, xmedia_awb_get_color_tone_attr, xmedia_awb_set_color_tone_attr),
};
int main(int argc, char **argv) {
    if (argc < 3) return 2;
    int dump = !strcmp(argv[1], "dump");
    for (size_t i = 0; i < sizeof T / sizeof *T; i++) {
        int want = dump;
        for (int a = 3; a < argc; a++) if (!strcmp(argv[a], T[i].n)) want = 1;
        if (!want) continue;
        char p[256]; snprintf(p, sizeof p, "%s/%s.bin", argv[2], T[i].n);
        void *b = calloc(1, T[i].sz);
        if (dump) {
            int r = T[i].g(0, b);
            FILE *f = fopen(p, "wb"); fwrite(b, 1, T[i].sz, f); fclose(f);
            printf("%-18s get %#x %zu\n", T[i].n, r, T[i].sz);
        } else {
            FILE *f = fopen(p, "rb");
            if (!f) { printf("%-18s missing\n", T[i].n); continue; }
            size_t n = fread(b, 1, T[i].sz, f); fclose(f);
            int r = n == T[i].sz ? T[i].s(0, b) : -1;
            printf("%-18s set %#x\n", T[i].n, r);
        }
        free(b);
    }
    return 0;
}
