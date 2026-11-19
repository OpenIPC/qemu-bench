/* Built for the camera, never run: build.sh reads each symbol's size back
 * with nm. sz_<type> is sizeof(type); of_<type>__<field> is its offset + 1. */
#include <stddef.h>
#include <xmedia_isp.h>
#include <xmedia_ae.h>
#include <xmedia_awb.h>
#define S(t) char sz_##t[sizeof(t)];
#define O(t, f) char of_##t##__##f[offsetof(t, f) + 1];
S(xmedia_isp_dpc_dynamic_attr) S(xmedia_isp_demosaic_attr) S(xmedia_isp_lce_attr)
S(xmedia_isp_ca_attr) S(xmedia_isp_sharpen_attr) S(xmedia_isp_stnr_attr)
S(xmedia_isp_bnr_attr) S(xmedia_isp_drc_attr) S(xmedia_isp_dehaze_attr)
S(xmedia_isp_gamma_attr) S(xmedia_isp_blc_attr) S(xmedia_isp_lcac_attr)
S(xmedia_isp_mesh_shading_attr) S(xmedia_isp_wdr_attr) S(xmedia_isp_csc_attr)
S(xmedia_isp_ainr_attr) S(xmedia_isp_statistics_cfg) S(xmedia_isp_smart_info)
S(xmedia_ae_exposure_attr) S(xmedia_isp_ae_route) S(xmedia_isp_ae_ext_route)
S(xmedia_ae_smart_exposure_attr) S(xmedia_ae_wdr_exposure_attr) S(xmedia_ae_exp_info)
S(xmedia_awb_attr) S(xmedia_awb_ccm_attr) S(xmedia_awb_saturation_attr)
S(xmedia_awb_scene_adj_attr) S(xmedia_color_sector_attr)
S(xmedia_awb_color_tone_attr) S(xmedia_awb_info)
O(xmedia_ae_exp_info, exp_time) O(xmedia_ae_exp_info, again)
O(xmedia_ae_exp_info, dgain) O(xmedia_ae_exp_info, isp_dgain)
O(xmedia_ae_exp_info, exposure) O(xmedia_ae_exp_info, avg_luma)
O(xmedia_ae_exp_info, fps) O(xmedia_ae_exp_info, iso)
O(xmedia_ae_exp_info, iso_sf) O(xmedia_ae_exp_info, iso_calibrate)
O(xmedia_awb_info, rgain) O(xmedia_awb_info, grgain) O(xmedia_awb_info, gbgain)
O(xmedia_awb_info, bgain) O(xmedia_awb_info, saturation) O(xmedia_awb_info, ct)
O(xmedia_awb_info, ls0_ct)
