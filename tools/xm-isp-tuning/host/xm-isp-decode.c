/* Print the fields of a captured or dumped set that tell one tuning from
 * another.   xm-isp-decode DIR */
#include <stdio.h>
#include <stdlib.h>
#include <xmedia_ae.h>
#include <xmedia_awb.h>
#include <xmedia_isp.h>

static void *ld(const char *d, const char *n, size_t sz) {
    char p[512];
    snprintf(p, sizeof p, "%s/%s.bin", d, n);
    FILE *f = fopen(p, "rb");
    if (!f)
        return NULL;
    void *b = calloc(1, sz);
    if (fread(b, 1, sz, f) != sz) {
        free(b);
        b = NULL;
    }
    fclose(f);
    return b;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s DIR\n", argv[0]);
        return 2;
    }
    const char *d = argv[1];
    xmedia_isp_sharpen_attr *h = ld(d, "sharpen", sizeof *h);
    if (h) {
        printf("sharpen: op %d\n", h->op_type);
        for (int i = 0; i < h->auto_attr.iso_num && i < XMEDIA_ISP_ISO_MAX_COUNT; i++)
            printf("  iso %7u edge %4u tex %4u overshoot %3u undershoot %3u coring %3u\n",
                h->auto_attr.iso_level[i], h->auto_attr.edge_str[i],
                h->auto_attr.tex_str[i], h->auto_attr.global_overshoot_str[i],
                h->auto_attr.global_undershoot_str[i], h->auto_attr.edge_coring[i]);
    }
    xmedia_isp_stnr_attr *n = ld(d, "stnr", sizeof *n);
    if (n)
        printf("stnr: on %d, ynr/cnr/tnr op %d/%d/%d\n", n->stnr_enable,
            n->ynr.op_type, n->cnr.op_type, n->tnr.op_type);
    xmedia_isp_bnr_attr *bn = ld(d, "bnr", sizeof *bn);
    if (bn)
        printf("bnr: on %d op %d, ainr position %d, tnr %d\n", bn->enable,
            bn->op_type, bn->ainr_pos_switch, bn->tnr_enable);
    xmedia_isp_dpc_dynamic_attr *p = ld(d, "dpc_dynamic", sizeof *p);
    if (p)
        printf("dpc: op %d, soft threshold %d, slope %u\n", p->op_type,
            p->soft_thd, p->soft_slope);
    xmedia_isp_demosaic_attr *m = ld(d, "demosaic", sizeof *m);
    if (m)
        printf("demosaic: op %d\n", m->op_type);
    xmedia_isp_drc_attr *r = ld(d, "drc", sizeof *r);
    if (r)
        printf("drc: on %d, curve %s, op %d, strength %u\n", r->drc_en,
            r->tm_type ? "user" : "asymmetry", r->op_type,
            r->manual_attr.drc_str);
    xmedia_isp_dehaze_attr *z = ld(d, "dehaze", sizeof *z);
    if (z)
        printf("dehaze: on %d, user lut %d, op %d\n", z->enable,
            z->userlut_enable, z->op_type);
    xmedia_isp_ainr_attr *a = ld(d, "ainr", sizeof *a);
    if (a)
        printf("ainr: on %d op %d, switch %u/%u\n", a->enable, a->op_type,
            a->switch_thd_low, a->switch_thd_high);
    xmedia_ae_exposure_attr *e = ld(d, "ae_exposure", sizeof *e);
    if (e)
        printf("ae: mode %d, compensation %u, speed %u, exposure up to %u, "
               "gain up to %u\n", e->auto_attr.ae_mode,
            e->auto_attr.compensation[0], e->auto_attr.speed,
            e->auto_attr.exp_time_range.max, e->auto_attr.sys_gain_range.max);
    xmedia_awb_attr *w = ld(d, "awb", sizeof *w);
    if (w)
        printf("awb: bypass %d op %d, ref %u K, static gains %u/%u/%u/%u\n",
            w->bypass, w->op_type, w->auto_attr.ref_ct,
            w->auto_attr.static_wb[0], w->auto_attr.static_wb[1],
            w->auto_attr.static_wb[2], w->auto_attr.static_wb[3]);
    xmedia_awb_saturation_attr *s = ld(d, "awb_saturation", sizeof *s);
    if (s) {
        printf("saturation: op %d", s->op_type);
        for (int i = 0; i < s->auto_attr.iso_num && i < XMEDIA_ISP_ISO_MAX_COUNT; i++)
            printf(" %u:%u", s->auto_attr.iso_level[i], s->auto_attr.sat[i]);
        printf("\n");
    }
    xmedia_awb_ccm_attr *c = ld(d, "awb_ccm", sizeof *c);
    if (c) {
        printf("ccm: op %d", c->op_type);
        for (int i = 0; i < c->auto_attr.ccm_tab_num && i < XMEDIA_ISP_CCM_TAB_MAX_NUM; i++)
            printf(" %uK[%u %u %u]", c->auto_attr.ccm_tab[i].ct,
                c->auto_attr.ccm_tab[i].ccm[0], c->auto_attr.ccm_tab[i].ccm[4],
                c->auto_attr.ccm_tab[i].ccm[8]);
        printf("\n");
    }
    return 0;
}
