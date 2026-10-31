/*
 * The WPA2-PSK side of the emulated AltoBeam access point, shared by the
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

/* WSM: the key-install request, and the RX indication's crypto flags */
#define WSM_ADD_KEY             0x000C
#define WSM_RX_AES              3
#define WSM_RX_GROUP_KEY        (1u << 19)

#define ATBM_WPA_IE_MAX         24
#define ATBM_WPA_CCMP_HDR       8
#define ATBM_WPA_CCMP_MIC       8

/* the frame a station sent, as atbm_wpa_from_sta() sorts it */
enum { ATBM_WPA_DROP, ATBM_WPA_PASS };

typedef struct AtbmWpa {
    /* set by atbm_wpa_init() */
    bool enabled;               /* a passphrase is configured */
    const char *tag;            /* the model's name, for the trace */
    bool trace;
    void *opaque;
    /* a frame from the AP to the driver (the model adds the WSM RX) */
    void (*rx)(void *opaque, const uint8_t *f, size_t n);
    /* deauthenticate the station (the model sends it, forgets the STA) */
    void (*deauth)(void *opaque, uint16_t reason);
    uint16_t *seq;              /* the AP's sequence counter */

    /* the association */
    uint8_t aa[6], spa[6];
    uint8_t pmk[32];
    uint8_t anonce[32], snonce[32];
    uint8_t ptk[48];            /* KCK 0..15, KEK 16..31, TK 32..47 */
    uint8_t gtk[16];
    uint64_t replay;            /* the EAPOL-Key replay counter */
    uint8_t state, tries;
    uint64_t pn_unicast, pn_group;
    bool ptk_installed, gtk_installed;
    QEMUTimer *timer;
} AtbmWpa;

/*
 * Configure: psk NULL or "" leaves the network open.  A passphrase must be
 * 8..63 characters (IEEE 802.11-2016 J.4.1).
 */
bool atbm_wpa_init(AtbmWpa *w, const char *psk, const char *ssid,
                   const char *tag, bool trace, void *opaque,
                   void (*rx)(void *, const uint8_t *, size_t),
                   void (*deauth)(void *, uint16_t), uint16_t *seq,
                   Error **errp);
void atbm_wpa_free(AtbmWpa *w);

/* the capability bits (privacy) and RSN IE beacons and responses carry */
uint16_t atbm_wpa_capability(const AtbmWpa *w);
size_t atbm_wpa_ie(const AtbmWpa *w, uint8_t *out);

/* the station aa / spa associated: start the 4-way handshake */
void atbm_wpa_associated(AtbmWpa *w, const uint8_t *aa, const uint8_t *spa);
/* the station left (deauth, disassociation, reset) */
void atbm_wpa_reset(AtbmWpa *w);
/* data may flow: an open network, or the handshake done */
bool atbm_wpa_port_open(const AtbmWpa *w);

/*
 * A data frame from the station, its header hl bytes: unwraps a
 * protected body (the CCMP header and MIC room the driver leaves around
 * the plaintext; the key is in the chip) and takes EAPOL itself.
 * ATBM_WPA_PASS: *p..*end is the LLC/SNAP payload to bridge.
 */
int atbm_wpa_from_sta(AtbmWpa *w, const uint8_t *f, size_t n, size_t hl,
                      size_t *p, size_t *end);

/*
 * A data frame to the station: when protected, the CCMP header the chip
 * would have left after decrypting (written at hdr, a rising PN), the
 * Protected bit to set and the RX flags; the caller appends
 * ATBM_WPA_CCMP_MIC bytes after the body.  Returns the header length
 * (0: send in the clear).
 */
size_t atbm_wpa_to_sta(AtbmWpa *w, bool group, uint8_t *hdr,
                       uint32_t *flags);

/* WSM_ADD_KEY's body: the driver installs the handshake's keys */
void atbm_wpa_add_key(AtbmWpa *w, const uint8_t *b, size_t n);

/* known-answer tests of the key derivation, for a "selftest" property */
bool atbm_wpa_selftest(void);

#endif
