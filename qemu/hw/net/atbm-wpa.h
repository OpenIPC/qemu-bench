/*
 * The WPA side of the emulated AltoBeam access point, shared by the
 * SDIO (atbm-sdio.c) and USB (atbm-wifi.c) chip models.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ATBM_WPA_H
#define HW_NET_ATBM_WPA_H

#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"

/* WSM: the key-install and power-mode requests, the PM indication, and
 * the RX indication's crypto flags */
#define WSM_ADD_KEY             0x000C
#define WSM_SET_PM              0x0010
#define WSM_SET_PM_IND          0x0809
#define WSM_RX_TKIP             2
#define WSM_RX_AES              3
#define WSM_RX_GROUP_KEY        (1u << 19)

#define ATBM_WPA_IE_MAX         32
#define ATBM_WPA_CCMP_HDR       8
#define ATBM_WPA_CCMP_MIC       8
#define ATBM_WPA_TRAILER_MAX    12      /* TKIP: Michael MIC and ICV */

/* the frame a station sent, as atbm_wpa_from_sta() sorts it */
enum { ATBM_WPA_DROP, ATBM_WPA_PASS };

/* the access point's security, as the models' properties set it */
typedef struct AtbmWpaOpts {
    char *proto;                /* "rsn" (WPA2, the default) or "wpa" */
    char *cipher;               /* "ccmp" (the default) or "tkip" */
    char *pmf;                  /* "off" (the default), "optional",
                                   "required": 802.11w */
    uint32_t rekey;             /* seconds between group rekeys; 0: never */
} AtbmWpaOpts;

#define DEFINE_ATBM_WPA_PROPERTIES(_state, _f)                          \
    DEFINE_PROP_STRING("proto", _state, _f.proto),                      \
    DEFINE_PROP_STRING("cipher", _state, _f.cipher),                    \
    DEFINE_PROP_STRING("pmf", _state, _f.pmf),                          \
    DEFINE_PROP_UINT32("rekey", _state, _f.rekey, 0)

typedef struct AtbmWpa {
    /* set by atbm_wpa_init() */
    bool enabled;               /* a passphrase is configured */
    bool wpa1;                  /* WPA (vendor IE, descriptor 254) */
    bool tkip;                  /* TKIP pairwise and group, else CCMP */
    uint8_t pmf;                /* 0 off, 1 optional (MFPC), 2 required */
    uint32_t rekey_s;
    const char *tag;            /* the model's name, for the trace */
    bool trace;
    void *opaque;
    /* a frame from the AP to the driver (the model adds the WSM RX with
     * these RX flags) */
    void (*rx)(void *opaque, const uint8_t *f, size_t n, uint32_t flags);
    /* deauthenticate the station (the model sends it, forgets the STA) */
    void (*deauth)(void *opaque, uint16_t reason);
    uint16_t *seq;              /* the AP's sequence counter */

    /* the association */
    uint8_t aa[6], spa[6];
    uint8_t pmk[32];
    uint8_t anonce[32], snonce[32];
    uint8_t ptk[64];            /* KCK 0..15, KEK 16..31, TK 32..47,
                                   TKIP: AP Tx MIC 48..55, Rx 56..63 */
    uint8_t gtk[32], gtk_next[32];
    uint8_t gtk_id;             /* the group key's ID, 1 or 2 */
    uint8_t igtk[16];
    uint64_t replay;            /* the EAPOL-Key replay counter */
    uint8_t state, tries;
    uint8_t gpending;           /* a group key handshake under way */
    uint32_t rekeys;            /* group rekeys completed */
    uint64_t pn_unicast, pn_group;
    bool ptk_installed, gtk_installed, igtk_installed;
    bool mic_logged;
    QEMUTimer *timer, *rekey_timer;
} AtbmWpa;

/*
 * Configure: psk NULL or "" leaves the network open.  A passphrase must be
 * 8..63 characters (IEEE 802.11-2016 J.4.1).  opts may be NULL.
 */
bool atbm_wpa_init(AtbmWpa *w, const char *psk, const char *ssid,
                   const AtbmWpaOpts *opts,
                   const char *tag, bool trace, void *opaque,
                   void (*rx)(void *, const uint8_t *, size_t, uint32_t),
                   void (*deauth)(void *, uint16_t), uint16_t *seq,
                   Error **errp);
void atbm_wpa_free(AtbmWpa *w);

/* the capability bits (privacy) and RSN / WPA IE beacons and responses
 * carry */
uint16_t atbm_wpa_capability(const AtbmWpa *w);
size_t atbm_wpa_ie(const AtbmWpa *w, uint8_t *out);

/* the station aa / spa associated: start the 4-way handshake */
void atbm_wpa_associated(AtbmWpa *w, const uint8_t *aa, const uint8_t *spa);
/* the station left (deauth, disassociation, reset) */
void atbm_wpa_reset(AtbmWpa *w);
/* data may flow: an open network, or the handshakes done */
bool atbm_wpa_port_open(const AtbmWpa *w);

/*
 * A data frame from the station, its header hl bytes: unwraps a
 * protected body (the IV and MIC room the driver leaves around the
 * plaintext; the key is in the chip) and takes EAPOL itself.
 * ATBM_WPA_PASS: *p..*end is the LLC/SNAP payload to bridge.
 */
int atbm_wpa_from_sta(AtbmWpa *w, const uint8_t *f, size_t n, size_t hl,
                      size_t *p, size_t *end);

/*
 * An action frame from the station (802.11w SA Query): answers it.
 * True when the frame was taken.
 */
bool atbm_wpa_action_from_sta(AtbmWpa *w, const uint8_t *f, size_t n);

/*
 * A data frame to the station: when protected, the CCMP / TKIP header the
 * chip would have left after decrypting (written at hdr, a rising PN /
 * TSC), the Protected bit to set and the RX flags.  Returns the header
 * length (0: send in the clear).  After the body the caller appends
 * atbm_wpa_trailer()'s bytes: CCMP's MIC room, TKIP's Michael MIC (which
 * mac80211 checks) and ICV room.
 */
size_t atbm_wpa_to_sta(AtbmWpa *w, bool group, uint8_t *hdr,
                       uint32_t *flags);
size_t atbm_wpa_trailer(AtbmWpa *w, bool group, const uint8_t *da,
                        const uint8_t *sa, const uint8_t *body, size_t blen,
                        uint8_t *out);

/* WSM_ADD_KEY's body: the driver installs the handshake's keys */
void atbm_wpa_add_key(AtbmWpa *w, const uint8_t *b, size_t n);

/* known-answer tests of the key derivation and the ciphers, for a
 * "selftest" property */
bool atbm_wpa_selftest(void);

#endif
