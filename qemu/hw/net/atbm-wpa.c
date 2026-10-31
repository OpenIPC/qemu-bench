/*
 * WPA2-PSK for the emulated AltoBeam access point (atbm-sdio.c, the SDIO
 * chip; atbm-wifi.c, the USB dongle).
 *
 * With a passphrase the access point advertises RSN (WPA2-PSK, CCMP
 * pairwise and group) and runs the authenticator's side of the 4-way
 * handshake and the group key (IEEE 802.11-2016 12.7): PMK = PBKDF2-
 * HMAC-SHA1(passphrase, SSID, 4096, 256 bits); PTK = PRF-384(PMK,
 * "Pairwise key expansion", min/max of the two addresses and the two
 * nonces); EAPOL-Key frames with descriptor version 2 (HMAC-SHA1-128
 * MICs keyed by the KCK, the GTK wrapped with AES key wrap under the
 * KEK).  A message 2 whose MIC fails -- a wrong passphrase -- is dropped;
 * after a few unanswered retries the station is deauthenticated.
 *
 * Data protection is the chip's job: both atbm drivers (hal_apollo)
 * install the keys with WSM_ADD_KEY, and mac80211 then only inserts the
 * 8-byte CCMP header while the driver leaves 8 bytes of MIC room around
 * a plaintext body.  So the "firmware" here checks that the installed
 * keys are the handshake's, strips that header and room from protected
 * frames, and hands the driver frames flagged AES-decrypted with a CCMP
 * header whose PN rises.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "crypto/aes.h"
#include "qemu/guest-random.h"
#include "atbm-wpa.h"

enum { RSN_NONE, RSN_M1, RSN_M3, RSN_DONE };

#define KEYINFO_VER2        2
#define KEYINFO_PAIRWISE    (1u << 3)
#define KEYINFO_INSTALL     (1u << 6)
#define KEYINFO_ACK         (1u << 7)
#define KEYINFO_MIC         (1u << 8)
#define KEYINFO_SECURE      (1u << 9)
#define KEYINFO_ENC_DATA    (1u << 12)

#define EAPOL_KEY_BODY      95      /* the EAPOL-Key fields before key data */
#define WSM_KEY_AES_GROUP       4
#define WSM_KEY_AES_PAIRWISE    5

#define FC_DATA             0x08
#define CAP_PRIVACY         0x0010

static const uint8_t llc_snap[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

/* RSN IE: version 1, CCMP group, one CCMP pairwise, one PSK AKM */
static const uint8_t rsn_ie[] = {
    0x30, 20, 1, 0,
    0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 2,
    0, 0,
};

static void hmac_sha1(const uint8_t *key, size_t klen,
                      const uint8_t *a, size_t alen,
                      const uint8_t *b, size_t blen, uint8_t out[20])
{
    GHmac *h = g_hmac_new(G_CHECKSUM_SHA1, key, klen);
    gsize n = 20;

    g_hmac_update(h, a, alen);
    if (blen) {
        g_hmac_update(h, b, blen);
    }
    g_hmac_get_digest(h, out, &n);
    g_hmac_unref(h);
}

/* PMK = PBKDF2-HMAC-SHA1(passphrase, ssid, 4096 iterations, 32 bytes) */
static void rsn_pmk(const char *pass, const char *ssid, uint8_t pmk[32])
{
    const size_t plen = strlen(pass), slen = MIN(strlen(ssid), 32);

    for (int blk = 1; blk <= 2; blk++) {
        uint8_t idx[4] = { 0, 0, 0, blk }, u[20], t[20];

        hmac_sha1((const uint8_t *)pass, plen, (const uint8_t *)ssid, slen,
                  idx, 4, u);
        memcpy(t, u, 20);
        for (int i = 1; i < 4096; i++) {
            hmac_sha1((const uint8_t *)pass, plen, u, 20, NULL, 0, u);
            for (int j = 0; j < 20; j++) {
                t[j] ^= u[j];
            }
        }
        memcpy(pmk + 20 * (blk - 1), t, blk == 1 ? 20 : 12);
    }
}

/* PRF-n (12.7.1.2): HMAC-SHA1(K, label || 0 || data || i), i = 0, 1, ... */
static void rsn_prf(const uint8_t *key, size_t klen, const char *label,
                    const uint8_t *data, size_t dlen,
                    uint8_t *out, size_t olen)
{
    const size_t llen = strlen(label);
    g_autofree uint8_t *a = g_malloc(llen + 1 + dlen + 1);

    memcpy(a, label, llen);
    a[llen] = 0;
    memcpy(a + llen + 1, data, dlen);
    for (size_t i = 0, done = 0; done < olen; i++) {
        uint8_t h[20];

        a[llen + 1 + dlen] = i;
        hmac_sha1(key, klen, a, llen + 2 + dlen, NULL, 0, h);
        memcpy(out + done, h, MIN(20, olen - done));
        done += 20;
    }
}

/* AES key wrap (RFC 3394) of n bytes (a multiple of 8) into n + 8 */
static void rsn_aes_wrap(const uint8_t kek[16], const uint8_t *in, size_t n,
                         uint8_t *out)
{
    AES_KEY k;
    uint8_t b[16];
    const size_t blocks = n / 8;

    AES_set_encrypt_key(kek, 128, &k);
    memset(out, 0xa6, 8);
    memcpy(out + 8, in, n);
    for (int j = 0; j < 6; j++) {
        for (size_t i = 1; i <= blocks; i++) {
            uint64_t t = blocks * j + i;

            memcpy(b, out, 8);
            memcpy(b + 8, out + 8 * i, 8);
            AES_encrypt(b, b, &k);
            for (int x = 0; x < 8; x++) {
                b[7 - x] ^= t >> (8 * x);
            }
            memcpy(out, b, 8);
            memcpy(out + 8 * i, b + 8, 8);
        }
    }
}

/* known-answer tests (IEEE 802.11-2016 J.4.2 PSK and J.3 PRF, RFC 3394
 * 4.1 key wrap) */
bool atbm_wpa_selftest(void)
{
    static const uint8_t pmk_kat[32] = {
        0xf4, 0x2c, 0x6f, 0xc5, 0x2d, 0xf0, 0xeb, 0xef,
        0x9e, 0xbb, 0x4b, 0x90, 0xb3, 0x8a, 0x5f, 0x90,
        0x2e, 0x83, 0xfe, 0x1b, 0x13, 0x5a, 0x70, 0xe2,
        0x3a, 0xed, 0x76, 0x2e, 0x97, 0x10, 0xa1, 0x2e,
    };
    static const uint8_t prf_kat[16] = {
        0xbc, 0xd4, 0xc6, 0x50, 0xb3, 0x0b, 0x96, 0x84,
        0x95, 0x18, 0x29, 0xe0, 0xd7, 0x5f, 0x9d, 0x54,
    };
    static const uint8_t kek[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    };
    static const uint8_t plain[16] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    };
    static const uint8_t wrap_kat[24] = {
        0x1f, 0xa6, 0x8b, 0x0a, 0x81, 0x12, 0xb4, 0x47,
        0xae, 0xf3, 0x4b, 0xd8, 0xfb, 0x5a, 0x7b, 0x82,
        0x9d, 0x3e, 0x86, 0x23, 0x71, 0xd2, 0xcf, 0xe5,
    };
    uint8_t key[20], out[64], pmk[32], wrapped[24];

    rsn_pmk("password", "IEEE", pmk);
    memset(key, 0x0b, sizeof(key));
    rsn_prf(key, sizeof(key), "prefix", (const uint8_t *)"Hi There", 8,
            out, 64);
    rsn_aes_wrap(kek, plain, 16, wrapped);
    return !memcmp(pmk, pmk_kat, 32) && !memcmp(out, prf_kat, 16) &&
           !memcmp(wrapped, wrap_kat, 24);
}

static void rsn_derive_ptk(AtbmWpa *w)
{
    uint8_t d[6 + 6 + 32 + 32];
    const bool a_first = memcmp(w->aa, w->spa, 6) < 0;
    const bool an_first = memcmp(w->anonce, w->snonce, 32) < 0;

    memcpy(d, a_first ? w->aa : w->spa, 6);
    memcpy(d + 6, a_first ? w->spa : w->aa, 6);
    memcpy(d + 12, an_first ? w->anonce : w->snonce, 32);
    memcpy(d + 44, an_first ? w->snonce : w->anonce, 32);
    rsn_prf(w->pmk, 32, "Pairwise key expansion", d, sizeof(d),
            w->ptk, sizeof(w->ptk));
}

/* an EAPOL-Key frame to the station, its MIC under the KCK if asked */
static void rsn_send_key(AtbmWpa *w, uint16_t info, const uint8_t *kd,
                         size_t kdlen)
{
    uint8_t f[24 + 8 + 4 + EAPOL_KEY_BODY + 128];
    uint8_t *e = f + 24 + 8, *k = e + 4;
    const size_t elen = 4 + EAPOL_KEY_BODY + kdlen;

    memset(f, 0, sizeof(f));
    f[0] = FC_DATA;
    f[1] = 0x02;                /* FromDS */
    memcpy(f + 4, w->spa, 6);
    memcpy(f + 10, w->aa, 6);
    memcpy(f + 16, w->aa, 6);
    stw_le_p(f + 22, (*w->seq)++ << 4);
    memcpy(f + 24, llc_snap, 6);
    stw_be_p(f + 30, 0x888e);
    e[0] = 2;                   /* 802.1X-2004 */
    e[1] = 3;                   /* EAPOL-Key */
    stw_be_p(e + 2, EAPOL_KEY_BODY + kdlen);
    k[0] = 2;                   /* RSN key descriptor */
    stw_be_p(k + 1, info);
    stw_be_p(k + 3, 16);
    stq_be_p(k + 5, w->replay);
    memcpy(k + 13, w->anonce, 32);
    stw_be_p(k + 93, kdlen);
    memcpy(k + 95, kd, kdlen);
    if (info & KEYINFO_MIC) {
        uint8_t mic[20];

        hmac_sha1(w->ptk, 16, e, elen, NULL, 0, mic);
        memcpy(k + 77, mic, 16);
    }
    w->rx(w->opaque, f, 24 + 8 + elen);
}

static void rsn_send_m1(AtbmWpa *w)
{
    w->replay++;
    rsn_send_key(w, KEYINFO_VER2 | KEYINFO_PAIRWISE | KEYINFO_ACK, NULL, 0);
    w->state = RSN_M1;
}

static void rsn_send_m3(AtbmWpa *w)
{
    uint8_t kd[64], wrapped[64 + 8];
    size_t n = 0;

    memcpy(kd, rsn_ie, sizeof(rsn_ie));
    n += sizeof(rsn_ie);
    /* GTK KDE: key ID 1 */
    kd[n++] = 0xdd;
    kd[n++] = 6 + 16;
    kd[n++] = 0x00;
    kd[n++] = 0x0f;
    kd[n++] = 0xac;
    kd[n++] = 1;
    kd[n++] = 1;
    kd[n++] = 0;
    memcpy(kd + n, w->gtk, 16);
    n += 16;
    if (n % 8) {                /* pad: 0xdd then zeros (12.7.2) */
        kd[n++] = 0xdd;
        while (n % 8) {
            kd[n++] = 0;
        }
    }
    rsn_aes_wrap(w->ptk + 16, kd, n, wrapped);
    w->replay++;
    rsn_send_key(w, KEYINFO_VER2 | KEYINFO_PAIRWISE | KEYINFO_INSTALL |
                 KEYINFO_ACK | KEYINFO_MIC | KEYINFO_SECURE |
                 KEYINFO_ENC_DATA, wrapped, n + 8);
    w->state = RSN_M3;
}

/* message 1 after association, and the retries of 1 and 3 */
static void rsn_timer(void *opaque)
{
    AtbmWpa *w = opaque;

    if (w->state == RSN_DONE) {
        return;
    }
    if (w->tries++ >= 4) {
        if (w->trace) {
            qemu_log("%s: 4-way handshake timeout, deauth\n", w->tag);
        }
        w->state = RSN_NONE;
        w->deauth(w->opaque, 15);       /* 4-way handshake timeout */
        return;
    }
    if (w->state == RSN_M3) {
        rsn_send_m3(w);
    } else {
        rsn_send_m1(w);
    }
    timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

/* an EAPOL frame from the station (e: the 802.1X header onwards) */
static void rsn_rx_eapol(AtbmWpa *w, const uint8_t *e, size_t elen)
{
    uint8_t mic[20], save[16];
    g_autofree uint8_t *c = NULL;
    const uint8_t *k = e + 4;
    uint16_t info;

    if (!w->enabled || elen < 4 + EAPOL_KEY_BODY || e[1] != 3 || k[0] != 2) {
        return;
    }
    elen = MIN(elen, 4 + (size_t)lduw_be_p(e + 2));
    info = lduw_be_p(k + 1);
    if (!(info & KEYINFO_MIC) || (info & KEYINFO_ACK) ||
        ldq_be_p(k + 5) != w->replay) {
        return;
    }
    if (w->state == RSN_M1 && !(info & KEYINFO_SECURE)) {
        memcpy(w->snonce, k + 13, 32);
        rsn_derive_ptk(w);
    } else if (w->state != RSN_M3) {
        return;
    }
    /* the MIC over the frame with its MIC field zeroed */
    c = g_memdup2(e, elen);
    memcpy(save, k + 77, 16);
    memset(c + 4 + 77, 0, 16);
    hmac_sha1(w->ptk, 16, c, elen, NULL, 0, mic);
    if (memcmp(mic, save, 16)) {
        if (w->trace) {
            qemu_log("%s: EAPOL message %d MIC mismatch (wrong passphrase?)\n",
                     w->tag, w->state == RSN_M1 ? 2 : 4);
        }
        return;
    }
    if (w->state == RSN_M1) {
        if (w->trace) {
            qemu_log("%s: EAPOL message 2 ok, sending 3\n", w->tag);
        }
        w->tries = 0;
        rsn_send_m3(w);
        timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
    } else {
        if (w->trace) {
            qemu_log("%s: EAPOL message 4 ok, port open\n", w->tag);
        }
        w->state = RSN_DONE;
        timer_del(w->timer);
    }
}

bool atbm_wpa_init(AtbmWpa *w, const char *psk, const char *ssid,
                   const char *tag, bool trace, void *opaque,
                   void (*rx)(void *, const uint8_t *, size_t),
                   void (*deauth)(void *, uint16_t), uint16_t *seq,
                   Error **errp)
{
    memset(w, 0, sizeof(*w));
    w->tag = tag;
    w->trace = trace;
    w->opaque = opaque;
    w->rx = rx;
    w->deauth = deauth;
    w->seq = seq;
    w->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, rsn_timer, w);
    if (!psk || !*psk) {
        return true;
    }
    if (strlen(psk) < 8 || strlen(psk) > 63) {
        error_setg(errp, "%s: psk must be 8..63 characters", tag);
        return false;
    }
    w->enabled = true;
    rsn_pmk(psk, ssid, w->pmk);
    qemu_guest_getrandom_nofail(w->gtk, sizeof(w->gtk));
    return true;
}

void atbm_wpa_free(AtbmWpa *w)
{
    if (w->timer) {
        timer_free(w->timer);
        w->timer = NULL;
    }
}

uint16_t atbm_wpa_capability(const AtbmWpa *w)
{
    return w->enabled ? CAP_PRIVACY : 0;
}

size_t atbm_wpa_ie(const AtbmWpa *w, uint8_t *out)
{
    if (!w->enabled) {
        return 0;
    }
    memcpy(out, rsn_ie, sizeof(rsn_ie));
    return sizeof(rsn_ie);
}

void atbm_wpa_associated(AtbmWpa *w, const uint8_t *aa, const uint8_t *spa)
{
    if (!w->enabled) {
        return;
    }
    memcpy(w->aa, aa, 6);
    memcpy(w->spa, spa, 6);
    qemu_guest_getrandom_nofail(w->anonce, sizeof(w->anonce));
    w->state = RSN_NONE;
    w->tries = 0;
    w->ptk_installed = w->gtk_installed = false;
    w->pn_unicast = w->pn_group = 0;
    timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

void atbm_wpa_reset(AtbmWpa *w)
{
    w->state = RSN_NONE;
    timer_del(w->timer);
}

bool atbm_wpa_port_open(const AtbmWpa *w)
{
    return !w->enabled || w->state == RSN_DONE;
}

int atbm_wpa_from_sta(AtbmWpa *w, const uint8_t *f, size_t n, size_t hl,
                      size_t *p, size_t *end)
{
    *p = hl;
    *end = n;
    if (f[1] & 0x40) {
        /* Protected: the key in the chip, a plaintext body between the
         * CCMP header and the MIC room */
        if (!w->enabled || w->state != RSN_DONE ||
            n < hl + ATBM_WPA_CCMP_HDR + ATBM_WPA_CCMP_MIC) {
            return ATBM_WPA_DROP;
        }
        *p += ATBM_WPA_CCMP_HDR;
        *end -= ATBM_WPA_CCMP_MIC;
    }
    if (*end < *p + 8 || memcmp(f + *p, llc_snap, 6)) {
        return ATBM_WPA_DROP;
    }
    if (lduw_be_p(f + *p + 6) == 0x888e) {
        rsn_rx_eapol(w, f + *p + 8, *end - *p - 8);
        return ATBM_WPA_DROP;
    }
    return atbm_wpa_port_open(w) ? ATBM_WPA_PASS : ATBM_WPA_DROP;
}

size_t atbm_wpa_to_sta(AtbmWpa *w, bool group, uint8_t *hdr,
                       uint32_t *flags)
{
    uint64_t pn;

    if (!w->enabled) {
        return 0;
    }
    /* "decrypted by the chip": ExtIV, key 0 pairwise / 1 group, a rising
     * PN that mac80211's replay check accepts */
    pn = group ? ++w->pn_group : ++w->pn_unicast;
    hdr[0] = pn;
    hdr[1] = pn >> 8;
    hdr[2] = 0;
    hdr[3] = 0x20 | (group ? 1 << 6 : 0);
    hdr[4] = pn >> 16;
    hdr[5] = pn >> 24;
    hdr[6] = pn >> 32;
    hdr[7] = pn >> 40;
    *flags |= WSM_RX_AES | (group ? WSM_RX_GROUP_KEY : 0);
    return ATBM_WPA_CCMP_HDR;
}

void atbm_wpa_add_key(AtbmWpa *w, const uint8_t *b, size_t n)
{
    if (n < 4 + 24) {
        return;
    }
    switch (b[0]) {
    case WSM_KEY_AES_PAIRWISE:
        w->ptk_installed = !memcmp(b + 12, w->ptk + 32, 16);
        if (w->trace) {
            qemu_log("%s: pairwise key installed, %s the TK\n", w->tag,
                     w->ptk_installed ? "matching" : "NOT matching");
        }
        break;
    case WSM_KEY_AES_GROUP:
        w->gtk_installed = !memcmp(b + 4, w->gtk, 16);
        if (w->trace) {
            qemu_log("%s: group key installed, %s the GTK\n", w->tag,
                     w->gtk_installed ? "matching" : "NOT matching");
        }
        break;
    }
}
