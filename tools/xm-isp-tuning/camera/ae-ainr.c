/* Set what the App's AE and AINR control code applies at run time rather
 * than from its tables, over what is in force on pipe 0.
 *
 *   ae-ainr ISO:COMP[,ISO:COMP...] SPEED BLACK_DELAY MAX_SYS_GAIN GAIN_THD \
 *           AINR_LOW AINR_HIGH
 *
 * The SC465SL values (README) are what the stock App ran, read from its
 * /proc/umap/isp and from the capture at several ISOs. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <xmedia_ae.h>
#include <xmedia_isp.h>

int main(int argc, char **argv) {
    if (argc != 8) {
        fprintf(stderr, "usage: %s ISO:COMP[,...] SPEED BLACK_DELAY "
                        "MAX_SYS_GAIN GAIN_THD AINR_LOW AINR_HIGH\n", argv[0]);
        return 2;
    }
    xmedia_ae_exposure_attr e;
    int r = xmedia_ae_get_exposure_attr(0, &e);
    if (r) {
        fprintf(stderr, "xmedia_ae_get_exposure_attr: %#x\n", r);
        return 1;
    }
    xmedia_ae_auto_attr *t = &e.auto_attr;
    int n = 0;
    for (char *p = strtok(argv[1], ","); p && n < XMEDIA_ISP_ISO_MAX_COUNT;
         p = strtok(NULL, ","), n++) {
        unsigned iso, comp;
        if (sscanf(p, "%u:%u", &iso, &comp) != 2) {
            fprintf(stderr, "bad ISO:COMP %s\n", p);
            return 2;
        }
        t->iso_level[n] = iso;
        t->compensation[n] = (xmedia_u8)comp;
    }
    t->iso_num = (xmedia_u8)n;
    t->speed = (xmedia_u8)atoi(argv[2]);
    t->ae_delay_attr.black_delay_frame = (xmedia_u16)atoi(argv[3]);
    t->sys_gain_range.max = (xmedia_u32)strtoul(argv[4], NULL, 0);
    t->gain_thres_hold = (xmedia_u32)strtoul(argv[5], NULL, 0);
    r = xmedia_ae_set_exposure_attr(0, &e);
    printf("AE: %d ISO points, speed %u: %#x\n", n, t->speed, r);

    xmedia_isp_ainr_attr a;
    if (!(r = xmedia_isp_get_ainr_attr(0, &a))) {
        a.switch_thd_low = (xmedia_u32)strtoul(argv[6], NULL, 0);
        a.switch_thd_high = (xmedia_u32)strtoul(argv[7], NULL, 0);
        r = xmedia_isp_set_ainr_attr(0, &a);
    }
    printf("AINR thresholds %u/%u: %#x\n", a.switch_thd_low, a.switch_thd_high, r);
    return 0;
}
