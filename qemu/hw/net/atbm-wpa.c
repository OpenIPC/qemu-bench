/*
 * WPA for the emulated AltoBeam access point (atbm-sdio.c, the SDIO chip;
 * atbm-wifi.c, the USB dongle), from IEEE 802.11-2016 clause 12.
 *
 * With a passphrase the access point advertises RSN (WPA2) or, with
 * proto=wpa, the WPA vendor IE, and runs the authenticator's side of the
 * 4-way handshake and the group key handshake: PMK = PBKDF2-HMAC-SHA1(
 * passphrase, SSID, 4096, 256 bits); PTK = PRF(PMK, "Pairwise key
 * expansion", min/max of the two addresses and the two nonces), 384 bits
 * for CCMP and 512 for TKIP.  EAPOL-Key frames use descriptor version 2
 * for CCMP (HMAC-SHA1-128 MICs keyed by the KCK, key data wrapped with
 * AES key wrap under the KEK) and version 1 for TKIP (HMAC-MD5 MICs, key
 * data encrypted with RC4 keyed by the Key IV and the KEK, the first 256
 * bytes of keystream discarded).  RSN's message 3 carries the GTK (and,
 * with pmf=, the IGTK) in its key data; WPA's carries only the WPA IE and
 * a group key handshake follows message 4.  A message whose MIC fails --
 * a wrong passphrase -- is dropped; after a few unanswered retries the
 * station is deauthenticated.  rekey=N runs a group key handshake every
 * N seconds with a fresh GTK under the other key ID.
 *
 * pmf= (802.11w) sets MFPC (and with "required" MFPR) in the RSN
 * capabilities with BIP-CMAC-128 as the group management cipher, hands
 * the station an IGTK in message 3, and answers its SA Query requests.
 * The access point sends no group-addressed robust management frame, so
 * BIP itself never arises; the station's IGTK install is checked.
 *
 * Data protection is the chip's job: both atbm drivers (hal_apollo)
 * install the keys with WSM_ADD_KEY, and mac80211 then only inserts the
 * IV while the driver leaves room around a plaintext body.  So the
 * "firmware" here checks that the installed keys are the handshakes',
 * strips that header and room from protected frames, and hands the
 * driver frames flagged decrypted with a CCMP / TKIP header whose PN /
 * TSC rises -- and, for TKIP, a Michael MIC over the body, which
 * mac80211 verifies, ahead of the ICV room.
 *
 * Power save needs nothing of the access point: the "chip" is the model,
 * awake whatever mode the driver asks for (see the models' WSM_SET_PM).
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/bitops.h"
#include "crypto/aes.h"
#include "qemu/guest-random.h"
#include "atbm-wpa.h"

enum { RSN_NONE, RSN_M1, RSN_M3, RSN_G1, RSN_DONE };

#define KEYINFO_VER1        1
#define KEYINFO_VER2        2
#define KEYINFO_PAIRWISE    (1u << 3)
#define KEYINFO_IDX_SHIFT   4
#define KEYINFO_INSTALL     (1u << 6)
#define KEYINFO_ACK         (1u << 7)
#define KEYINFO_MIC         (1u << 8)
#define KEYINFO_SECURE      (1u << 9)
#define KEYINFO_ENC_DATA    (1u << 12)

#define EAPOL_KEY_BODY      95      /* the EAPOL-Key fields before key data */
#define DESC_RSN            2
#define DESC_WPA            254

#define WSM_KEY_TKIP_GROUP      2
#define WSM_KEY_TKIP_PAIRWISE   3
#define WSM_KEY_AES_GROUP       4
#define WSM_KEY_AES_PAIRWISE    5
#define WSM_KEY_IGTK_GROUP      8

#define FC_DATA             0x08
#define FC_ACTION           0xd0
#define CAP_PRIVACY         0x0010
#define IGTK_ID             4

static const uint8_t llc_snap[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

/* ---- primitives --------------------------------------------------------- */

static void hmac(GChecksumType t, const uint8_t *key, size_t klen,
                 const uint8_t *a, size_t alen,
                 const uint8_t *b, size_t blen, uint8_t *out, size_t olen)
{
    GHmac *h = g_hmac_new(t, key, klen);
    uint8_t d[32];
    gsize n = sizeof(d);

    g_hmac_update(h, a, alen);
    if (blen) {
        g_hmac_update(h, b, blen);
    }
    g_hmac_get_digest(h, d, &n);
    g_hmac_unref(h);
    memcpy(out, d, MIN(olen, (size_t)n));
}

static void hmac_sha1(const uint8_t *key, size_t klen,
                      const uint8_t *a, size_t alen,
                      const uint8_t *b, size_t blen, uint8_t out[20])
{
    hmac(G_CHECKSUM_SHA1, key, klen, a, alen, b, blen, out, 20);
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

/* RC4 over n bytes in place, the first drop bytes of keystream discarded */
static void rc4(const uint8_t *key, size_t klen, size_t drop,
                uint8_t *d, size_t n)
{
    uint8_t s[256], t;
    unsigned i, j = 0;

    for (i = 0; i < 256; i++) {
        s[i] = i;
    }
    for (i = 0; i < 256; i++) {
        j = (j + s[i] + key[i % klen]) & 0xff;
        t = s[i];
        s[i] = s[j];
        s[j] = t;
    }
    i = j = 0;
    for (size_t k = 0; k < drop + n; k++) {
        i = (i + 1) & 0xff;
        j = (j + s[i]) & 0xff;
        t = s[i];
        s[i] = s[j];
        s[j] = t;
        if (k >= drop) {
            d[k - drop] ^= s[(s[i] + s[j]) & 0xff];
        }
    }
}

/* Michael (12.5.2.3): the TKIP MIC over a message, 8 bytes */
static void michael_block(uint32_t *l, uint32_t *r)
{
    *r ^= rol32(*l, 17);
    *l += *r;
    *r ^= ((*l & 0xff00ff00) >> 8) | ((*l & 0x00ff00ff) << 8);
    *l += *r;
    *r ^= rol32(*l, 3);
    *l += *r;
    *r ^= ror32(*l, 2);
    *l += *r;
}

static void michael(const uint8_t key[8], const uint8_t *m, size_t n,
                    uint8_t out[8])
{
    uint32_t l = ldl_le_p(key), r = ldl_le_p(key + 4), last = 0x5a;
    size_t i;

    for (i = 0; i + 4 <= n; i += 4) {
        l ^= ldl_le_p(m + i);
        michael_block(&l, &r);
    }
    /* the last bytes, then 0x5a, then zeros: 4..7 of them in all */
    last <<= 8 * (n - i);
    for (size_t k = 0; i + k < n; k++) {
        last |= (uint32_t)m[i + k] << (8 * k);
    }
    l ^= last;
    michael_block(&l, &r);
    michael_block(&l, &r);          /* the zero word: l ^= 0 */
    stl_le_p(out, l);
    stl_le_p(out + 4, r);
}

/* Michael over an MSDU: DA, SA, priority and three zeros, then the data */
static void michael_msdu(const uint8_t key[8], const uint8_t *da,
                         const uint8_t *sa, uint8_t prio,
                         const uint8_t *d, size_t n, uint8_t out[8])
{
    g_autofree uint8_t *m = g_malloc(16 + n);

    memcpy(m, da, 6);
    memcpy(m + 6, sa, 6);
    m[12] = prio;
    m[13] = m[14] = m[15] = 0;
    memcpy(m + 16, d, n);
    michael(key, m, 16 + n, out);
}

/*
 * Known-answer tests: IEEE 802.11-2016 J.4.2 PSK and J.3 PRF, RFC 3394
 * 4.1 key wrap, RFC 2202 HMAC-MD5 test 1, the classic RC4 vector, and the
 * Michael test vectors of IEEE 802.11i (each MIC the next one's key)
 */
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
    static const uint8_t md5_kat[16] = {
        0x92, 0x94, 0x72, 0x7a, 0x36, 0x38, 0xbb, 0x1c,
        0x13, 0xf4, 0x8e, 0xf8, 0x15, 0x8b, 0xfc, 0x9d,
    };
    static const uint8_t rc4_kat[9] = {
        0xbb, 0xf3, 0x16, 0xe8, 0xd9, 0x40, 0xaf, 0x0a, 0xd3,
    };
    static const struct {
        const char *msg;
        uint8_t mic[8];
    } mich[] = {
        { "",        { 0x82, 0x92, 0x5c, 0x1c, 0xa1, 0xd1, 0x30, 0xb8 } },
        { "M",       { 0x43, 0x47, 0x21, 0xca, 0x40, 0x63, 0x9b, 0x3f } },
        { "Mi",      { 0xe8, 0xf9, 0xbe, 0xca, 0xe9, 0x7e, 0x5d, 0x29 } },
        { "Mic",     { 0x90, 0x03, 0x8f, 0xc6, 0xcf, 0x13, 0xc1, 0xdb } },
        { "Mich",    { 0xd5, 0x5e, 0x10, 0x05, 0x10, 0x12, 0x89, 0x86 } },
        { "Michael", { 0x0a, 0x94, 0x2b, 0x12, 0x4e, 0xca, 0xa5, 0x46 } },
    };
    uint8_t key[20], out[64], pmk[32], wrapped[24], md5[16], mkey[8] = { 0 };
    uint8_t r[9];
    bool ok;

    rsn_pmk("password", "IEEE", pmk);
    memset(key, 0x0b, sizeof(key));
    rsn_prf(key, sizeof(key), "prefix", (const uint8_t *)"Hi There", 8,
            out, 64);
    rsn_aes_wrap(kek, plain, 16, wrapped);
    hmac(G_CHECKSUM_MD5, key, 16, (const uint8_t *)"Hi There", 8, NULL, 0,
         md5, 16);
    memcpy(r, "Plaintext", 9);
    rc4((const uint8_t *)"Key", 3, 0, r, 9);
    ok = !memcmp(pmk, pmk_kat, 32) && !memcmp(out, prf_kat, 16) &&
         !memcmp(wrapped, wrap_kat, 24) && !memcmp(md5, md5_kat, 16) &&
         !memcmp(r, rc4_kat, 9);
    for (size_t i = 0; i < ARRAY_SIZE(mich); i++) {
        uint8_t m[8];

        michael(mkey, (const uint8_t *)mich[i].msg, strlen(mich[i].msg), m);
        ok &= !memcmp(m, mich[i].mic, 8);
        memcpy(mkey, mich[i].mic, 8);
    }
    return ok;
}

/* ---- the access point's IEs and EAPOL-Key frames ------------------------ */

static size_t gtk_len(const AtbmWpa *w)
{
    return w->tkip ? 32 : 16;
}

static uint16_t key_version(const AtbmWpa *w)
{
    return w->tkip ? KEYINFO_VER1 : KEYINFO_VER2;
}

/* RSN: version 1, one group, one pairwise and one PSK AKM suite, the
 * capabilities; with PMF the group management cipher too */
static size_t rsn_build_ie(const AtbmWpa *w, uint8_t *o)
{
    const uint8_t cs = w->tkip ? 2 : 4;
    size_t n = 2;

    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x0f; o[n++] = 0xac; o[n++] = cs;
    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x0f; o[n++] = 0xac; o[n++] = cs;
    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x0f; o[n++] = 0xac; o[n++] = 2;
    o[n++] = (w->pmf ? 0x80 : 0) | (w->pmf == 2 ? 0x40 : 0);   /* MFPC/R */
    o[n++] = 0;
    if (w->pmf) {
        o[n++] = 0; o[n++] = 0;                         /* no PMKIDs */
        o[n++] = 0x00; o[n++] = 0x0f; o[n++] = 0xac; o[n++] = 6;
    }
    o[0] = 0x30;
    o[1] = n - 2;
    return n;
}

/* WPA (the pre-RSN vendor IE): version 1, group, pairwise, PSK */
static size_t wpa_build_ie(const AtbmWpa *w, uint8_t *o)
{
    const uint8_t cs = w->tkip ? 2 : 4;
    size_t n = 2;

    o[n++] = 0x00; o[n++] = 0x50; o[n++] = 0xf2; o[n++] = 1;
    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x50; o[n++] = 0xf2; o[n++] = cs;
    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x50; o[n++] = 0xf2; o[n++] = cs;
    o[n++] = 1; o[n++] = 0;
    o[n++] = 0x00; o[n++] = 0x50; o[n++] = 0xf2; o[n++] = 2;
    o[0] = 0xdd;
    o[1] = n - 2;
    return n;
}

static size_t ap_ie(const AtbmWpa *w, uint8_t *o)
{
    return w->wpa1 ? wpa_build_ie(w, o) : rsn_build_ie(w, o);
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
            w->ptk, w->tkip ? 64 : 48);
}

/* the EAPOL-Key MIC under the KCK: HMAC-MD5 (v1) or HMAC-SHA1-128 (v2) */
static void eapol_mic(const AtbmWpa *w, const uint8_t *e, size_t n,
                      uint8_t mic[16])
{
    hmac(w->tkip ? G_CHECKSUM_MD5 : G_CHECKSUM_SHA1, w->ptk, 16, e, n,
         NULL, 0, mic, 16);
}

/* encrypt key data under the KEK: AES key wrap (v2, padded) or RC4 keyed
 * by iv || KEK (v1); returns the encrypted length */
static size_t encrypt_kd(const AtbmWpa *w, uint8_t *kd, size_t n,
                         uint8_t *out, uint8_t iv[16])
{
    if (w->tkip) {
        uint8_t k[32];

        qemu_guest_getrandom_nofail(iv, 16);
        memcpy(k, iv, 16);
        memcpy(k + 16, w->ptk + 16, 16);
        memcpy(out, kd, n);
        rc4(k, 32, 256, out, n);
        return n;
    }
    memset(iv, 0, 16);
    if (n % 8 || n < 16) {      /* pad: 0xdd then zeros (12.7.2) */
        kd[n++] = 0xdd;
        while (n % 8 || n < 16) {
            kd[n++] = 0;
        }
    }
    rsn_aes_wrap(w->ptk + 16, kd, n, out);
    return n + 8;
}

/* a protected or clear data frame from the AP to the station, its LLC /
 * SNAP payload body..blen */
static void ap_send_data(AtbmWpa *w, bool protect, const uint8_t *body,
                         size_t blen)
{
    g_autofree uint8_t *f = g_malloc(24 + 8 + blen + ATBM_WPA_TRAILER_MAX);
    uint32_t flags = 0;
    size_t n = 24;

    memset(f, 0, 24);
    f[0] = FC_DATA;
    f[1] = 0x02;                /* FromDS */
    memcpy(f + 4, w->spa, 6);
    memcpy(f + 10, w->aa, 6);
    memcpy(f + 16, w->aa, 6);
    stw_le_p(f + 22, (*w->seq)++ << 4);
    if (protect) {
        n += atbm_wpa_to_sta(w, false, f + n, &flags);
        f[1] |= 0x40;
    }
    memcpy(f + n, body, blen);
    n += blen;
    if (protect) {
        n += atbm_wpa_trailer(w, false, w->spa, w->aa, body, blen, f + n);
    }
    w->rx(w->opaque, f, n, flags);
}

/* an EAPOL-Key frame to the station, its MIC under the KCK if asked;
 * protected once the pairwise key is in use (a group rekey) */
static void rsn_send_key(AtbmWpa *w, uint16_t info, uint16_t keylen,
                         const uint8_t iv[16], const uint8_t *kd,
                         size_t kdlen)
{
    uint8_t b[8 + 4 + EAPOL_KEY_BODY + 160];
    uint8_t *e = b + 8, *k = e + 4;
    const size_t elen = 4 + EAPOL_KEY_BODY + kdlen;

    memset(b, 0, sizeof(b));
    memcpy(b, llc_snap, 6);
    stw_be_p(b + 6, 0x888e);
    e[0] = 2;                   /* 802.1X-2004 */
    e[1] = 3;                   /* EAPOL-Key */
    stw_be_p(e + 2, EAPOL_KEY_BODY + kdlen);
    k[0] = w->wpa1 ? DESC_WPA : DESC_RSN;
    stw_be_p(k + 1, info);
    stw_be_p(k + 3, keylen);
    stq_be_p(k + 5, w->replay);
    memcpy(k + 13, w->anonce, 32);
    if (iv) {
        memcpy(k + 45, iv, 16);
    }
    stw_be_p(k + 93, kdlen);
    memcpy(k + 95, kd, kdlen);
    if (info & KEYINFO_MIC) {
        eapol_mic(w, e, elen, k + 77);
    }
    ap_send_data(w, w->state == RSN_DONE, b, 8 + elen);
}

static void rsn_send_m1(AtbmWpa *w)
{
    w->replay++;
    rsn_send_key(w, key_version(w) | KEYINFO_PAIRWISE | KEYINFO_ACK,
                 w->tkip ? 32 : 16, NULL, NULL, 0);
    w->state = RSN_M1;
}

/* the GTK KDE (and the IGTK KDE with PMF) after the RSN IE */
static size_t rsn_kdes(const AtbmWpa *w, uint8_t *kd, const uint8_t *gtk,
                       uint8_t id, bool igtk)
{
    size_t n = 0;

    kd[n++] = 0xdd;
    kd[n++] = 6 + gtk_len(w);
    kd[n++] = 0x00; kd[n++] = 0x0f; kd[n++] = 0xac; kd[n++] = 1;
    kd[n++] = id & 3;
    kd[n++] = 0;
    memcpy(kd + n, gtk, gtk_len(w));
    n += gtk_len(w);
    if (igtk && w->pmf) {
        kd[n++] = 0xdd;
        kd[n++] = 4 + 2 + 6 + 16;
        kd[n++] = 0x00; kd[n++] = 0x0f; kd[n++] = 0xac; kd[n++] = 9;
        kd[n++] = IGTK_ID;
        kd[n++] = 0;
        memset(kd + n, 0, 6);   /* IPN */
        n += 6;
        memcpy(kd + n, w->igtk, 16);
        n += 16;
    }
    return n;
}

static void rsn_send_m3(AtbmWpa *w)
{
    uint8_t kd[128], enc[128 + 8], iv[16];
    size_t n = ap_ie(w, kd);
    const uint16_t keylen = w->tkip ? 32 : 16;

    w->replay++;
    if (w->wpa1) {
        /* WPA: the IE in the clear; the GTK comes in the group handshake */
        rsn_send_key(w, key_version(w) | KEYINFO_PAIRWISE | KEYINFO_INSTALL |
                     KEYINFO_ACK | KEYINFO_MIC, keylen, NULL, kd, n);
    } else {
        n += rsn_kdes(w, kd + n, w->gtk, w->gtk_id, true);
        n = encrypt_kd(w, kd, n, enc, iv);
        rsn_send_key(w, key_version(w) | KEYINFO_PAIRWISE | KEYINFO_INSTALL |
                     KEYINFO_ACK | KEYINFO_MIC | KEYINFO_SECURE |
                     KEYINFO_ENC_DATA, keylen, iv, enc, n);
    }
    w->state = RSN_M3;
}

/* group key message 1: WPA's after message 4, or a rekey (gtk_next) */
static void rsn_send_g1(AtbmWpa *w)
{
    uint8_t kd[96], enc[96 + 8], iv[16];
    const bool next = w->gpending;
    const uint8_t *gtk = next ? w->gtk_next : w->gtk;
    const uint8_t id = next ? 3 - w->gtk_id : w->gtk_id;
    uint16_t info = key_version(w) | KEYINFO_SECURE | KEYINFO_MIC |
                    KEYINFO_ACK;
    size_t n;

    w->replay++;
    if (w->wpa1) {
        memcpy(kd, gtk, gtk_len(w));
        n = encrypt_kd(w, kd, gtk_len(w), enc, iv);
        info |= id << KEYINFO_IDX_SHIFT;
        rsn_send_key(w, info, gtk_len(w), iv, enc, n);
    } else {
        n = rsn_kdes(w, kd, gtk, id, false);
        n = encrypt_kd(w, kd, n, enc, iv);
        rsn_send_key(w, info | KEYINFO_ENC_DATA, 0, iv, enc, n);
    }
    if (!next) {
        w->state = RSN_G1;
    }
}

static void rsn_done(AtbmWpa *w)
{
    w->state = RSN_DONE;
    w->gpending = 0;
    timer_del(w->timer);
    if (w->rekey_s) {
        timer_mod(w->rekey_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  1000 * (int64_t)w->rekey_s);
    }
}

/* message 1 after association, the retries of 1, 3 and the group
 * message, and their timeout */
static void rsn_timer(void *opaque)
{
    AtbmWpa *w = opaque;

    if (w->state == RSN_DONE && !w->gpending) {
        return;
    }
    if (w->tries++ >= 4) {
        const bool group = w->gpending || w->state == RSN_G1;

        if (w->trace) {
            qemu_log("%s: %s handshake timeout, deauth\n", w->tag,
                     group ? "group key" : "4-way");
        }
        w->state = RSN_NONE;
        w->gpending = 0;
        timer_del(w->rekey_timer);
        /* 4-way or group key handshake timeout */
        w->deauth(w->opaque, group ? 16 : 15);
        return;
    }
    if (w->gpending || w->state == RSN_G1) {
        rsn_send_g1(w);
    } else if (w->state == RSN_M3) {
        rsn_send_m3(w);
    } else {
        rsn_send_m1(w);
    }
    timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

/* every rekey seconds: a fresh GTK under the other key ID */
static void rsn_rekey(void *opaque)
{
    AtbmWpa *w = opaque;

    if (w->state != RSN_DONE || w->gpending) {
        return;
    }
    qemu_guest_getrandom_nofail(w->gtk_next, sizeof(w->gtk_next));
    w->gpending = 1;
    w->tries = 0;
    if (w->trace) {
        qemu_log("%s: group rekey %u: key ID %d\n", w->tag, w->rekeys + 1,
                 3 - w->gtk_id);
    }
    rsn_send_g1(w);
    timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
}

/* an EAPOL frame from the station (e: the 802.1X header onwards) */
static void rsn_rx_eapol(AtbmWpa *w, const uint8_t *e, size_t elen)
{
    uint8_t mic[16], save[16];
    g_autofree uint8_t *c = NULL;
    const uint8_t *k = e + 4;
    uint16_t info;
    int msg;

    if (!w->enabled || elen < 4 + EAPOL_KEY_BODY || e[1] != 3 ||
        k[0] != (w->wpa1 ? DESC_WPA : DESC_RSN)) {
        return;
    }
    elen = MIN(elen, 4 + (size_t)lduw_be_p(e + 2));
    info = lduw_be_p(k + 1);
    if (!(info & KEYINFO_MIC) || (info & KEYINFO_ACK) ||
        ldq_be_p(k + 5) != w->replay) {
        return;
    }
    if (!(info & KEYINFO_PAIRWISE)) {
        if (!(w->gpending || w->state == RSN_G1)) {
            return;
        }
        msg = 6;                /* group message 2 */
    } else if (w->state == RSN_M1) {
        memcpy(w->snonce, k + 13, 32);
        rsn_derive_ptk(w);
        msg = 2;
    } else if (w->state == RSN_M3) {
        msg = 4;
    } else {
        return;
    }
    /* the MIC over the frame with its MIC field zeroed */
    c = g_memdup2(e, elen);
    memcpy(save, k + 77, 16);
    memset(c + 4 + 77, 0, 16);
    eapol_mic(w, c, elen, mic);
    if (memcmp(mic, save, 16)) {
        if (w->trace) {
            qemu_log("%s: EAPOL message %d MIC mismatch%s\n", w->tag,
                     msg == 6 ? 2 : msg,
                     msg == 2 ? " (wrong passphrase?)" : "");
        }
        return;
    }
    w->tries = 0;
    switch (msg) {
    case 2:
        if (w->trace) {
            qemu_log("%s: EAPOL message 2 ok, sending 3\n", w->tag);
        }
        rsn_send_m3(w);
        timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
        break;
    case 4:
        if (w->wpa1) {
            if (w->trace) {
                qemu_log("%s: EAPOL message 4 ok, sending group message 1\n",
                         w->tag);
            }
            rsn_send_g1(w);
            timer_mod(w->timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1000);
        } else {
            if (w->trace) {
                qemu_log("%s: EAPOL message 4 ok, port open\n", w->tag);
            }
            rsn_done(w);
        }
        break;
    case 6:
        if (w->gpending) {
            memcpy(w->gtk, w->gtk_next, sizeof(w->gtk));
            w->gtk_id = 3 - w->gtk_id;
            w->pn_group = 0;
            w->rekeys++;
            if (w->trace) {
                qemu_log("%s: group rekey %u done, key ID %d\n", w->tag,
                         w->rekeys, w->gtk_id);
            }
        } else if (w->trace) {
            qemu_log("%s: group message 2 ok, port open\n", w->tag);
        }
        rsn_done(w);
        break;
    }
}

/* ---- the models' interface ---------------------------------------------- */

bool atbm_wpa_init(AtbmWpa *w, const char *psk, const char *ssid,
                   const AtbmWpaOpts *opts,
                   const char *tag, bool trace, void *opaque,
                   void (*rx)(void *, const uint8_t *, size_t, uint32_t),
                   void (*deauth)(void *, uint16_t), uint16_t *seq,
                   Error **errp)
{
    const char *proto = opts && opts->proto ? opts->proto : "rsn";
    const char *cipher = opts && opts->cipher ? opts->cipher : "ccmp";
    const char *pmf = opts && opts->pmf ? opts->pmf : "off";

    memset(w, 0, sizeof(*w));
    w->tag = tag;
    w->trace = trace;
    w->opaque = opaque;
    w->rx = rx;
    w->deauth = deauth;
    w->seq = seq;
    w->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, rsn_timer, w);
    w->rekey_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, rsn_rekey, w);
    if (!strcmp(proto, "wpa")) {
        w->wpa1 = true;
    } else if (strcmp(proto, "rsn") && strcmp(proto, "wpa2")) {
        error_setg(errp, "%s: proto must be rsn or wpa", tag);
        return false;
    }
    if (!strcmp(cipher, "tkip")) {
        w->tkip = true;
    } else if (strcmp(cipher, "ccmp")) {
        error_setg(errp, "%s: cipher must be ccmp or tkip", tag);
        return false;
    }
    if (!strcmp(pmf, "optional")) {
        w->pmf = 1;
    } else if (!strcmp(pmf, "required")) {
        w->pmf = 2;
    } else if (strcmp(pmf, "off")) {
        error_setg(errp, "%s: pmf must be off, optional or required", tag);
        return false;
    }
    if (w->pmf && (w->wpa1 || w->tkip)) {
        error_setg(errp, "%s: pmf needs proto=rsn and cipher=ccmp", tag);
        return false;
    }
    w->rekey_s = opts ? opts->rekey : 0;
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
    qemu_guest_getrandom_nofail(w->igtk, sizeof(w->igtk));
    w->gtk_id = 1;
    return true;
}

void atbm_wpa_free(AtbmWpa *w)
{
    if (w->timer) {
        timer_free(w->timer);
        w->timer = NULL;
    }
    if (w->rekey_timer) {
        timer_free(w->rekey_timer);
        w->rekey_timer = NULL;
    }
}

uint16_t atbm_wpa_capability(const AtbmWpa *w)
{
    return w->enabled ? CAP_PRIVACY : 0;
}

size_t atbm_wpa_ie(const AtbmWpa *w, uint8_t *out)
{
    return w->enabled ? ap_ie(w, out) : 0;
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
    w->gpending = 0;
    w->ptk_installed = w->gtk_installed = w->igtk_installed = false;
    w->pn_unicast = w->pn_group = 0;
    timer_del(w->rekey_timer);
    timer_mod(w->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

void atbm_wpa_reset(AtbmWpa *w)
{
    w->state = RSN_NONE;
    w->gpending = 0;
    timer_del(w->timer);
    timer_del(w->rekey_timer);
}

bool atbm_wpa_port_open(const AtbmWpa *w)
{
    return !w->enabled || w->state == RSN_DONE;
}

/* a TKIP body from the station: the driver's mac80211 leaves 12 bytes
 * of room after it (its TKIP icv_len: the ICV and the Michael MIC), which
 * the chip fills on the air from the MIC keys it was given; a MIC there
 * is checked (the supplicant's Tx MIC key, the authenticator's Rx one) */
static void tkip_trim_mic(AtbmWpa *w, const uint8_t *f, size_t p,
                          size_t *end)
{
    const uint8_t prio = f[0] == 0x88 ? (f[24] & 7) : 0;
    uint8_t mic[8];

    if (*end < p + 12) {
        return;
    }
    if (w->trace && !w->mic_logged) {
        michael_msdu(w->ptk + 56, f + 16, f + 10, prio, f + p,
                     *end - p - 12, mic);
        w->mic_logged = true;
        qemu_log("%s: TKIP from the station: %s\n", w->tag,
                 memcmp(mic, f + *end - 12, 8) ?
                 "MIC room only (the chip computes Michael)" :
                 "a valid Michael MIC");
    }
    *end -= 12;
}

int atbm_wpa_from_sta(AtbmWpa *w, const uint8_t *f, size_t n, size_t hl,
                      size_t *p, size_t *end)
{
    *p = hl;
    *end = n;
    if (f[1] & 0x40) {
        /* Protected: the key in the chip, a plaintext body after the
         * IV (and, for CCMP, before the MIC room) */
        if (!w->enabled || w->state < RSN_M3 ||
            n < hl + ATBM_WPA_CCMP_HDR + (w->tkip ? 12 : ATBM_WPA_CCMP_MIC)) {
            return ATBM_WPA_DROP;
        }
        *p += ATBM_WPA_CCMP_HDR;    /* the IV: CCMP's and TKIP's are 8 */
        if (w->tkip) {
            tkip_trim_mic(w, f, *p, end);
        } else {
            *end -= ATBM_WPA_CCMP_MIC;
        }
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

bool atbm_wpa_action_from_sta(AtbmWpa *w, const uint8_t *f, size_t n)
{
    size_t p = 24, resp;
    uint8_t r[24 + 8 + 4 + ATBM_WPA_TRAILER_MAX];
    uint32_t flags = 0;
    const bool prot = f[1] & 0x40;

    if (!w->enabled || !w->pmf || w->state != RSN_DONE) {
        return false;
    }
    if (prot) {
        p += ATBM_WPA_CCMP_HDR;
    }
    /* SA Query request (category 8, action 0): the response, protected */
    if (n < p + 4 || f[p] != 8 || f[p + 1] != 0) {
        return false;
    }
    memset(r, 0, 24);
    r[0] = FC_ACTION;
    memcpy(r + 4, w->spa, 6);
    memcpy(r + 10, w->aa, 6);
    memcpy(r + 16, w->aa, 6);
    stw_le_p(r + 22, (*w->seq)++ << 4);
    resp = 24 + atbm_wpa_to_sta(w, false, r + 24, &flags);
    r[1] |= 0x40;
    r[resp] = 8;
    r[resp + 1] = 1;
    memcpy(r + resp + 2, f + p + 2, 2);     /* the transaction ID */
    resp += 4;
    memset(r + resp, 0, ATBM_WPA_CCMP_MIC);
    resp += ATBM_WPA_CCMP_MIC;
    if (w->trace) {
        qemu_log("%s: SA Query request, answered\n", w->tag);
    }
    w->rx(w->opaque, r, resp, flags);
    return true;
}

size_t atbm_wpa_to_sta(AtbmWpa *w, bool group, uint8_t *hdr,
                       uint32_t *flags)
{
    uint64_t pn;
    const uint8_t id = group ? w->gtk_id : 0;

    if (!w->enabled) {
        return 0;
    }
    /* "decrypted by the chip": ExtIV, key 0 pairwise / gtk_id group, a
     * rising PN / TSC that mac80211's replay check accepts */
    pn = group ? ++w->pn_group : ++w->pn_unicast;
    if (w->tkip) {
        hdr[0] = pn >> 8;                       /* TSC1 */
        hdr[1] = ((pn >> 8) | 0x20) & 0x7f;     /* WEP seed */
        hdr[2] = pn;                            /* TSC0 */
    } else {
        hdr[0] = pn;
        hdr[1] = pn >> 8;
        hdr[2] = 0;
    }
    hdr[3] = 0x20 | id << 6;
    hdr[4] = pn >> 16;
    hdr[5] = pn >> 24;
    hdr[6] = pn >> 32;
    hdr[7] = pn >> 40;
    *flags |= (w->tkip ? WSM_RX_TKIP : WSM_RX_AES) |
              (group ? WSM_RX_GROUP_KEY : 0);
    return ATBM_WPA_CCMP_HDR;
}

size_t atbm_wpa_trailer(AtbmWpa *w, bool group, const uint8_t *da,
                        const uint8_t *sa, const uint8_t *body, size_t blen,
                        uint8_t *out)
{
    if (!w->enabled) {
        return 0;
    }
    if (!w->tkip) {
        memset(out, 0, ATBM_WPA_CCMP_MIC);
        return ATBM_WPA_CCMP_MIC;
    }
    /* the authenticator's Tx MIC key: the PTK's or the GTK's */
    michael_msdu(group ? w->gtk + 16 : w->ptk + 48, da, sa, 0, body, blen,
                 out);
    memset(out + 8, 0, 4);                      /* the ICV, room only */
    return 12;
}

void atbm_wpa_add_key(AtbmWpa *w, const uint8_t *b, size_t n)
{
    bool m;

    if (n < 4 + 24) {
        return;
    }
    switch (b[0]) {
    case WSM_KEY_AES_PAIRWISE:
    case WSM_KEY_TKIP_PAIRWISE:
        w->ptk_installed = !memcmp(b + 12, w->ptk + 32, 16);
        if (w->trace) {
            qemu_log("%s: pairwise key installed, %s the TK\n", w->tag,
                     w->ptk_installed ? "matching" : "NOT matching");
        }
        break;
    case WSM_KEY_AES_GROUP:
    case WSM_KEY_TKIP_GROUP:
        /* the current GTK, or during a rekey the next one */
        m = !memcmp(b + 4, w->gtk, 16) ||
            (w->gpending && !memcmp(b + 4, w->gtk_next, 16));
        w->gtk_installed = m;
        if (w->trace) {
            qemu_log("%s: group key installed, %s the GTK (key ID %d)\n",
                     w->tag, m ? "matching" : "NOT matching",
                     b[b[0] == WSM_KEY_AES_GROUP ? 20 : 28]);
        }
        break;
    case WSM_KEY_IGTK_GROUP:
        w->igtk_installed = !memcmp(b + 4, w->igtk, 16) && b[20] == IGTK_ID;
        if (w->trace) {
            qemu_log("%s: IGTK installed, %s (key ID %d)\n", w->tag,
                     w->igtk_installed ? "matching" : "NOT matching", b[20]);
        }
        break;
    }
}
