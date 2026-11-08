/*
 * The world in front of an emulated HiSilicon camera
 * (docs/hisilicon-digital-twin.md, layer 1).
 *
 * One per machine, /machine/scene.  It holds what the sensor looks at and
 * the camera's state for the current frame; hisi-vi measures the scene
 * for the ISP's statistics, hisi-vpss paints it into the output channels.
 *
 *   file     what is in front of the camera: a binary PPM (P6, sRGB,
 *            8 or 16 bits) or a PFM (linear float, so an HDR scene keeps
 *            its range), up to 64 Mpixel; empty for the built-in grey bars (hisi-vpss's
 *            old test picture, eight bars from white to black)
 *   light    white's raw level a row at 1x gain, 16-bit units
 *   light-r  the light's colour as the sensor sees it: R and B
 *   light-b  per mille of G
 *
 * Every property can be changed while the guest runs, e.g.
 *   qom-set /machine/scene light 1200
 * or scripted:
 *   timeline a file of events, seconds from the first frame the ISP
 *            measures; each line a time and properties to set:
 *              8 light=150
 *              16..20 light-r=550->1000 light-b=700->400
 *              30 file=/path/office.ppm
 *            "a..b prop=x->y" ramps a number from x to y over a..b, a
 *            step each frame; '#' starts a comment
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/qdev-core.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "hw/misc/hisi-scene.h"

#define SCENE_BARS          8
#define SCENE_CACHE         4
#define GAMMA_STEPS         4096
#define SCENE_MAX_PIXELS    (64 * 1024 * 1024)  /* 768 MiB decoded */

typedef struct SceneGrid {
    int w, h;
    uint32_t gen;               /* the source it was made from */
    float *rgb;
} SceneGrid;

typedef struct SceneEvent {
    double t0, t1;              /* seconds; t1 > t0 for a ramp */
    char *prop;
    char *value;                /* a step's value */
    double from, to;            /* a ramp's ends */
    bool done;
} SceneEvent;

struct HisiSceneState {
    DeviceState parent_obj;

    char *timeline;
    GArray *events;             /* SceneEvent */
    bool started;
    int64_t start_ns;           /* the first frame, virtual time */

    char *file;
    float *img;                 /* the source, linear, NULL for the bars */
    int iw, ih;
    uint32_t gen;
    SceneGrid cache[SCENE_CACHE];
    int next;

    uint32_t light, light_r, light_b;

    double level;               /* the frame's white, 16-bit raw */
    double tint[3];             /* the light it was captured under */
    double wb[3];               /* the driver's white balance gains */
    uint8_t gamma[GAMMA_STEPS + 1];
};

static const uint8_t bar_y[SCENE_BARS] = { 235, 210, 170, 145, 106, 81, 41, 16 };

/* --- sources ------------------------------------------------------------ */

static float srgb_to_linear(double v)
{
    return v <= 0.04045 ? v / 12.92 : pow((v + 0.055) / 1.055, 2.4);
}

/* the next header token of a PPM / PFM, skipping comments */
static bool pnm_token(const uint8_t *b, size_t len, size_t *pos, char *tok,
                      size_t toklen)
{
    size_t n = 0;

    while (*pos < len) {
        if (b[*pos] == '#') {
            while (*pos < len && b[*pos] != '\n') {
                (*pos)++;
            }
        } else if (g_ascii_isspace(b[*pos])) {
            (*pos)++;
        } else {
            break;
        }
    }
    while (*pos < len && !g_ascii_isspace(b[*pos]) && n + 1 < toklen) {
        tok[n++] = b[(*pos)++];
    }
    tok[n] = 0;
    if (*pos >= len || !g_ascii_isspace(b[*pos])) {
        return false;           /* no delimiter: truncated, or too long */
    }
    (*pos)++;                   /* the single whitespace before the data */
    return n > 0;
}

static float *scene_load(const char *path, int *pw, int *ph, Error **errp)
{
    g_autofree uint8_t *b = NULL;
    char magic[4], tw[16], th[16], tm[32];
    size_t len, pos = 0;
    int w, h, x, y, c;
    float *img;

    if (!g_file_get_contents(path, (char **)&b, &len, NULL)) {
        error_setg(errp, "hisi-scene: cannot read '%s'", path);
        return NULL;
    }
    if (!pnm_token(b, len, &pos, magic, sizeof(magic)) ||
        !pnm_token(b, len, &pos, tw, sizeof(tw)) ||
        !pnm_token(b, len, &pos, th, sizeof(th)) ||
        !pnm_token(b, len, &pos, tm, sizeof(tm))) {
        goto bad;
    }
    w = atoi(tw);
    h = atoi(th);
    /* pos <= len: pnm_token() stops at the end */
    if (w <= 0 || h <= 0 || (int64_t)w * h > SCENE_MAX_PIXELS) {
        goto bad;
    }

    if (!strcmp(magic, "P6")) {
        const int maxval = atoi(tm);
        const int bpc = maxval > 255 ? 2 : 1;

        /* the payload is there before anything is allocated for it */
        if (maxval <= 0 || maxval > 65535 ||
            len - pos < (size_t)w * h * 3 * bpc) {
            goto bad;
        }
        img = g_new(float, (size_t)w * h * 3);
        for (x = 0; x < w * h * 3; x++) {
            const uint8_t *p = b + pos + (size_t)x * bpc;
            const int v = bpc == 2 ? p[0] << 8 | p[1] : p[0];

            img[x] = srgb_to_linear((double)v / maxval);
        }
    } else if (!strcmp(magic, "PF") || !strcmp(magic, "Pf")) {
        const int chans = magic[1] == 'F' ? 3 : 1;
        const bool le = atof(tm) < 0;

        if (len - pos < (size_t)w * h * chans * 4) {
            goto bad;
        }
        img = g_new(float, (size_t)w * h * 3);
        for (y = 0; y < h; y++) {
            /* PFM rows run bottom to top */
            const uint8_t *row = b + pos + (size_t)(h - 1 - y) * w * chans * 4;

            for (x = 0; x < w; x++) {
                for (c = 0; c < 3; c++) {
                    const uint8_t *p = row + (x * chans + (chans == 3 ? c : 0)) * 4;
                    uint32_t u = le ? ldl_le_p(p) : ldl_be_p(p);
                    float f;

                    memcpy(&f, &u, 4);
                    img[((size_t)y * w + x) * 3 + c] =
                        isfinite(f) && f > 0 ? f : 0;
                }
            }
        }
    } else {
        goto bad;
    }
    *pw = w;
    *ph = h;
    return img;

bad:
    error_setg(errp, "hisi-scene: '%s' is not a binary PPM (P6) or a PFM",
               path);
    return NULL;
}

/* --- the grid ----------------------------------------------------------- */

static void scene_render(HisiSceneState *s, float *rgb, int w, int h)
{
    int x, y, c;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            float *o = rgb + ((size_t)y * w + x) * 3;

            if (!s->img) {
                const float r = MAX(bar_y[x * SCENE_BARS / w] - 16, 6) / 219.0;

                o[0] = o[1] = o[2] = r;
                continue;
            }
            /* a box over the source pixels this one covers (or the one
             * nearest, scaling up) */
            {
                const int x0 = (int64_t)x * s->iw / w;
                const int x1 = MAX((int64_t)(x + 1) * s->iw / w, x0 + 1);
                const int y0 = (int64_t)y * s->ih / h;
                const int y1 = MAX((int64_t)(y + 1) * s->ih / h, y0 + 1);
                double sum[3] = { 0 };
                int i, j;

                for (j = y0; j < y1; j++) {
                    for (i = x0; i < x1; i++) {
                        for (c = 0; c < 3; c++) {
                            sum[c] += s->img[((size_t)j * s->iw + i) * 3 + c];
                        }
                    }
                }
                for (c = 0; c < 3; c++) {
                    o[c] = sum[c] / ((x1 - x0) * (y1 - y0));
                }
            }
        }
    }
}

const float *hisi_scene_grid(HisiSceneState *s, int w, int h)
{
    SceneGrid *g;
    int i;

    w = MAX(w, 1);
    h = MAX(h, 1);
    for (i = 0; i < SCENE_CACHE; i++) {
        g = &s->cache[i];
        if (g->rgb && g->w == w && g->h == h && g->gen == s->gen) {
            return g->rgb;
        }
    }
    g = &s->cache[s->next];
    s->next = (s->next + 1) % SCENE_CACHE;
    g_free(g->rgb);
    g->rgb = g_new(float, (size_t)w * h * 3);
    g->w = w;
    g->h = h;
    g->gen = s->gen;
    scene_render(s, g->rgb, w, h);
    return g->rgb;
}

/* --- the timeline ------------------------------------------------------ */

static void scene_events_free(GArray *ev)
{
    int i;

    if (!ev) {
        return;
    }
    for (i = 0; i < ev->len; i++) {
        SceneEvent *e = &g_array_index(ev, SceneEvent, i);

        g_free(e->prop);
        g_free(e->value);
    }
    g_array_free(ev, true);
}

/* a whole finite number, nothing after it */
static bool scene_parse_number(const char *str, double *v)
{
    char *end;

    if (!str || !*str) {
        return false;
    }
    *v = g_ascii_strtod(str, &end);
    return end != str && !*end && isfinite(*v);
}

/* "t" or a rising "a..b" (a ramp), each a whole number of seconds */
static bool scene_parse_time(const char *tok, double *t0, double *t1)
{
    const char *dots = strstr(tok, "..");
    g_autofree char *a = NULL;

    if (!dots) {
        return scene_parse_number(tok, t0) && *t0 >= 0 &&
               (*t1 = *t0, true);
    }
    a = g_strndup(tok, dots - tok);
    return scene_parse_number(a, t0) && scene_parse_number(dots + 2, t1) &&
           *t0 >= 0 && *t1 > *t0;
}

static GArray *scene_timeline_load(const char *path, Error **errp)
{
    g_autofree char *text = NULL;
    g_auto(GStrv) lines = NULL;
    GArray *ev = g_array_new(false, true, sizeof(SceneEvent));
    int n;

    if (!g_file_get_contents(path, &text, NULL, NULL)) {
        error_setg(errp, "hisi-scene: cannot read timeline '%s'", path);
        g_array_free(ev, true);
        return NULL;
    }
    lines = g_strsplit(text, "\n", -1);
    for (n = 0; lines[n]; n++) {
        g_auto(GStrv) tok = NULL;
        char *hash = strchr(lines[n], '#');
        double t0 = 0, t1 = 0;
        int i;

        if (hash) {
            *hash = 0;
        }
        tok = g_strsplit_set(g_strstrip(lines[n]), " \t", -1);
        if (!tok[0] || !*tok[0]) {
            continue;
        }
        if (!scene_parse_time(tok[0], &t0, &t1)) {
            error_setg(errp, "hisi-scene: %s:%d: '%s' is not a time or a "
                       "rising a..b ramp", path, n + 1, tok[0]);
            goto bad;
        }
        for (i = 1; tok[i]; i++) {
            SceneEvent e = { .t0 = t0, .t1 = t1 };
            char *eq = strchr(tok[i], '=');
            char *arrow = eq ? strstr(eq, "->") : NULL;

            if (!*tok[i]) {
                continue;
            }
            if (!eq || eq == tok[i]) {
                error_setg(errp, "hisi-scene: %s:%d: '%s' is not prop=value",
                           path, n + 1, tok[i]);
                goto bad;
            }
            e.prop = g_strndup(tok[i], eq - tok[i]);
            if (!strcmp(e.prop, "timeline")) {
                /* it would free the events while they play */
                error_setg(errp, "hisi-scene: %s:%d: a timeline cannot load "
                           "another", path, n + 1);
                g_free(e.prop);
                goto bad;
            }
            if (t1 > t0) {
                g_autofree char *from = arrow ?
                    g_strndup(eq + 1, arrow - eq - 1) : NULL;

                if (!arrow || !scene_parse_number(from, &e.from) ||
                    !scene_parse_number(arrow + 2, &e.to)) {
                    error_setg(errp, "hisi-scene: %s:%d: a ramp needs "
                               "prop=from->to with two numbers", path, n + 1);
                    g_free(e.prop);
                    goto bad;
                }
            } else {
                e.value = g_strdup(eq + 1);
            }
            g_array_append_val(ev, e);
        }
    }
    return ev;

bad:
    scene_events_free(ev);
    return NULL;
}

static void scene_apply(HisiSceneState *s, const char *prop,
                        const char *value)
{
    Error *err = NULL;

    if (!object_property_parse(OBJECT(s), prop, value, &err)) {
        warn_report_err(err);
    }
}

double hisi_scene_frame_begin(HisiSceneState *s)
{
    const int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double t;
    int i;

    if (!s->started) {
        s->started = true;
        s->start_ns = now;
    }
    t = (double)(now - s->start_ns) / NANOSECONDS_PER_SECOND;
    for (i = 0; s->events && i < s->events->len; i++) {
        SceneEvent *e = &g_array_index(s->events, SceneEvent, i);
        char buf[32];

        if (e->done || t < e->t0) {
            continue;
        }
        if (e->t1 > e->t0) {
            const double f = MIN((t - e->t0) / (e->t1 - e->t0), 1.0);

            snprintf(buf, sizeof(buf), "%.0f",
                     e->from + (e->to - e->from) * f);
            scene_apply(s, e->prop, buf);
            e->done = f >= 1.0;
        } else {
            scene_apply(s, e->prop, e->value);
            e->done = true;
        }
    }
    return t;
}

double hisi_scene_light(HisiSceneState *s, double tint[3])
{
    tint[0] = s->light_r / 1000.0;
    tint[1] = 1.0;
    tint[2] = s->light_b / 1000.0;
    return s->light;
}

void hisi_scene_set_frame(HisiSceneState *s, double level,
                          const double tint[3], double wb_r, double wb_b)
{
    s->level = level;
    memcpy(s->tint, tint, sizeof(s->tint));
    s->wb[0] = wb_r > 0 ? wb_r : 1.0;
    s->wb[1] = 1.0;
    s->wb[2] = wb_b > 0 ? wb_b : 1.0;
}

void hisi_scene_display_row(HisiSceneState *s, const float *rgb, int w,
                            uint8_t *y, uint8_t *vu)
{
    double k[3];
    int x, c;

    for (c = 0; c < 3; c++) {
        /* raw (clipped at the sensor's range) times the WB gain */
        k[c] = s->tint[c] * s->level / 65535.0;
    }
    for (x = 0; x < w; x++) {
        const float *p = rgb + x * 3;
        double e[3], luma;

        for (c = 0; c < 3; c++) {
            const double v = MIN(MIN(p[c] * k[c], 1.0) * s->wb[c], 1.0);

            e[c] = s->gamma[(int)(v * GAMMA_STEPS)] / 255.0;
        }
        luma = 0.299 * e[0] + 0.587 * e[1] + 0.114 * e[2];
        y[x] = 16 + (int)(219 * luma + 0.5);
        if (vu && !(x & 1)) {
            vu[x] = 128 + (int)lrint(MAX(MIN(224 * 0.713 * (e[0] - luma), 112), -112));
            vu[x + 1] = 128 + (int)lrint(MAX(MIN(224 * 0.564 * (e[2] - luma), 112), -112));
        }
    }
}

/* --- properties --------------------------------------------------------- */

static char *scene_get_file(Object *obj, Error **errp)
{
    return g_strdup(HISI_SCENE(obj)->file ? HISI_SCENE(obj)->file : "");
}

static void scene_set_file(Object *obj, const char *value, Error **errp)
{
    HisiSceneState *s = HISI_SCENE(obj);
    float *img = NULL;
    int w = 0, h = 0;

    if (value && *value) {
        img = scene_load(value, &w, &h, errp);
        if (!img) {
            return;
        }
    }
    g_free(s->img);
    g_free(s->file);
    s->img = img;
    s->iw = w;
    s->ih = h;
    s->file = g_strdup(value);
    s->gen++;
}

static char *scene_get_timeline(Object *obj, Error **errp)
{
    return g_strdup(HISI_SCENE(obj)->timeline ? HISI_SCENE(obj)->timeline
                                               : "");
}

static void scene_set_timeline(Object *obj, const char *value, Error **errp)
{
    HisiSceneState *s = HISI_SCENE(obj);
    GArray *ev = NULL;

    if (value && *value) {
        ev = scene_timeline_load(value, errp);
        if (!ev) {
            return;
        }
    }
    scene_events_free(s->events);
    g_free(s->timeline);
    s->events = ev;
    s->timeline = g_strdup(value);
    s->started = false;         /* its clock starts at the next frame */
}

static void scene_get_u32(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    visit_type_uint32(v, name, opaque, errp);
}

static void scene_set_u32(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    uint32_t value;

    if (visit_type_uint32(v, name, &value, errp)) {
        *(uint32_t *)opaque = value;
    }
}

static void hisi_scene_init(Object *obj)
{
    HisiSceneState *s = HISI_SCENE(obj);
    int i;

    s->light = 600;
    s->light_r = 550;
    s->light_b = 700;
    s->level = 65535.0 / 2;
    s->tint[0] = s->light_r / 1000.0;
    s->tint[1] = 1.0;
    s->tint[2] = s->light_b / 1000.0;
    s->wb[0] = s->wb[1] = s->wb[2] = 1.0;
    for (i = 0; i <= GAMMA_STEPS; i++) {
        s->gamma[i] = lrint(255 * pow((double)i / GAMMA_STEPS, 1 / 2.2));
    }
    object_property_add(obj, "light", "uint32", scene_get_u32, scene_set_u32,
                        NULL, &s->light);
    object_property_add(obj, "light-r", "uint32", scene_get_u32,
                        scene_set_u32, NULL, &s->light_r);
    object_property_add(obj, "light-b", "uint32", scene_get_u32,
                        scene_set_u32, NULL, &s->light_b);
}

static void hisi_scene_finalize(Object *obj)
{
    HisiSceneState *s = HISI_SCENE(obj);
    int i;

    for (i = 0; i < SCENE_CACHE; i++) {
        g_free(s->cache[i].rgb);
    }
    g_free(s->img);
    g_free(s->file);
    scene_events_free(s->events);
    g_free(s->timeline);
}

static void hisi_scene_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    object_class_property_add_str(klass, "file", scene_get_file,
                                  scene_set_file);
    object_class_property_add_str(klass, "timeline", scene_get_timeline,
                                  scene_set_timeline);
    dc->desc = "the scene in front of an emulated HiSilicon camera";
    dc->user_creatable = false;
}

static const TypeInfo hisi_scene_info = {
    .name              = TYPE_HISI_SCENE,
    .parent            = TYPE_DEVICE,
    .instance_size     = sizeof(HisiSceneState),
    .instance_init     = hisi_scene_init,
    .instance_finalize = hisi_scene_finalize,
    .class_init        = hisi_scene_class_init,
};

static void hisi_scene_register_types(void)
{
    type_register_static(&hisi_scene_info);
}

type_init(hisi_scene_register_types)
