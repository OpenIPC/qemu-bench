/*
 * The Ingenic security blocks from user space, through the SDKs' sample
 * drivers (sample_aes, sample_hash, sample_rsa, sample_dtrng: /dev/aes,
 * /dev/hash, /dev/rsa, /dev/dtrng).  Prints one line per check, for
 * test-ingenic-crypto.sh to compare on the host:
 *
 *   AES <mode> <hex out>            FIPS-197 / SP 800-38A vectors
 *   HASH <alg> <hex digest>         of a fixed message, padded here
 *   RSA <bits> <hex M^E mod N>      for each "bits e n m" line of a file
 *   DTRNG cpu|irq <hex word>
 *
 * The ioctl numbers and argument structures are the sample drivers' ABI.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define IOCTL_AES_START_EN  _IOW('A', 111, unsigned int)
#define IOCTL_AES_START_DE  _IOW('A', 112, unsigned int)
#define IOCTL_HASH_MD5      _IOW('H', 111, unsigned int)
#define IOCTL_RSA_ENC_DEC   _IOW('R', 110, unsigned int)
#define IOCTL_DTRNG_DMA     _IO('D', 110)
#define IOCTL_DTRNG_CPU     _IO('D', 111)

struct aes_para {
    unsigned int status, enworkmode;
    unsigned int aeskey[4], aesiv[4];
    unsigned char *src, *dst;
    unsigned int datalen, donelen;
};

struct hash_para {
    unsigned int digest_mode;
    unsigned char *src, *dst;
    unsigned int plaintext_len, crypttext_len;
};

struct rsa_data {
    unsigned int *e_or_d, *n;
    unsigned int rsa_mode;
    unsigned int *input;
    unsigned int inlen;
    unsigned int *output;
};

static void hex(const char *tag, const unsigned char *p, int n)
{
    int i;

    printf("%s ", tag);
    for (i = 0; i < n; i++) {
        printf("%02x", p[i]);
    }
    printf("\n");
}

static void unhex(const char *s, unsigned char *p, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        sscanf(s + 2 * i, "%2hhx", &p[i]);
    }
}

/* the AES's memory format: each word's top byte first */
static void swap_words(unsigned char *p, int n)
{
    int i;

    for (i = 0; i < n; i += 4) {
        unsigned char t[4] = { p[i + 3], p[i + 2], p[i + 1], p[i] };
        memcpy(p + i, t, 4);
    }
}

static unsigned int be32(const unsigned char *p)
{
    return (unsigned)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

static int aes(void)
{
    static const char *vec[][5] = {
        /* mode, key, iv, plain, (cipher checked on the host) */
        { "ecb", "000102030405060708090a0b0c0d0e0f", "",
          "00112233445566778899aabbccddeeff" },
        { "cbc", "2b7e151628aed2a6abf7158809cf4f3c",
          "000102030405060708090a0b0c0d0e0f",
          "6bc1bee22e409f96e93d7e117393172a"
          "ae2d8a571e03ac9c9eb76fac45af8e51" },
    };
    int fd = open("/dev/aes", O_RDWR), v, i;

    if (fd < 0) {
        printf("AES no /dev/aes\n");
        return 1;
    }
    for (v = 0; v < 2; v++) {
        struct aes_para p;
        unsigned char key[16], iv[16], in[32], out[32], back[32];
        int n = strlen(vec[v][3]) / 2;
        char tag[32];

        memset(&p, 0, sizeof(p));
        unhex(vec[v][1], key, 16);
        unhex(vec[v][3], in, n);
        for (i = 0; i < 4; i++) {
            p.aeskey[i] = be32(key + 4 * i);
        }
        if (v) {
            unhex(vec[v][2], iv, 16);
            for (i = 0; i < 4; i++) {
                p.aesiv[i] = be32(iv + 4 * i);
            }
        }
        p.enworkmode = v;
        swap_words(in, n);
        p.src = in;
        p.dst = out;
        p.datalen = n;
        if (ioctl(fd, IOCTL_AES_START_EN, &p) < 0) {
            printf("AES %s ioctl failed\n", vec[v][0]);
            continue;
        }
        p.status = 0;
        p.src = out;
        p.dst = back;
        ioctl(fd, IOCTL_AES_START_DE, &p);
        swap_words(out, n);
        snprintf(tag, sizeof(tag), "AES %s", vec[v][0]);
        hex(tag, out, n);
        printf("AES %s roundtrip %s\n", vec[v][0],
               memcmp(back, in, n) ? "bad" : "ok");
    }
    close(fd);
    return 0;
}

static int hash(void)
{
    static const char *alg[] = {
        "md5", "sha1", "sha224", "sha256", "sha384", "sha512",
    };
    static const int words[] = { 4, 5, 7, 8, 12, 16 };
    const char *msg = "The quick brown fox jumps over the lazy dog";
    int fd = open("/dev/hash", O_RDWR), a;

    if (fd < 0) {
        printf("HASH no /dev/hash\n");
        return 1;
    }
    for (a = 0; a < 6; a++) {
        unsigned char buf[256], out[64];
        unsigned int len = strlen(msg), bs = a >= 4 ? 128 : 64;
        unsigned int lb = a >= 4 ? 16 : 8, n, i;
        uint64_t bits = (uint64_t)len * 8;
        struct hash_para p;
        char tag[32];

        memset(buf, 0, sizeof(buf));
        memcpy(buf, msg, len);
        buf[len] = 0x80;
        n = (len + 1 + lb + bs - 1) / bs * bs;
        for (i = 0; i < 8; i++) {
            if (a == 0) {
                buf[n - 8 + i] = bits >> (8 * i);
            } else {
                buf[n - 1 - i] = bits >> (8 * i);
            }
        }
        memset(&p, 0, sizeof(p));
        p.digest_mode = IOCTL_HASH_MD5 + a;
        p.src = buf;
        p.dst = out;
        p.plaintext_len = n;
        if (ioctl(fd, IOCTL_HASH_MD5 + a, &p) < 0) {
            printf("HASH %s ioctl failed\n", alg[a]);
            continue;
        }
        snprintf(tag, sizeof(tag), "HASH %s", alg[a]);
        hex(tag, out, words[a] * 4);
    }
    close(fd);
    return 0;
}

static int rsa(const char *path)
{
    FILE *f = fopen(path, "r");
    int fd = open("/dev/rsa", O_RDWR);
    static char e[600], n[600], m[600];
    int bits;

    if (fd < 0 || !f) {
        printf("RSA no /dev/rsa or %s\n", path);
        return 1;
    }
    while (fscanf(f, "%d %599s %599s %599s%*[^\n]", &bits, e, n, m) == 4) {
        unsigned int ew[64], nw[64], mw[64], out[64];
        unsigned char b[256];
        int k = bits / 32, i;
        struct rsa_data d;
        char tag[32];

        /* the operands as the sample's test passes them: most
           significant word first, each a number */
        unhex(e, b, k * 4);
        for (i = 0; i < k; i++) {
            ew[i] = be32(b + 4 * i);
        }
        unhex(n, b, k * 4);
        for (i = 0; i < k; i++) {
            nw[i] = be32(b + 4 * i);
        }
        unhex(m, b, k * 4);
        for (i = 0; i < k; i++) {
            mw[i] = be32(b + 4 * i);
        }
        memset(&d, 0, sizeof(d));
        d.e_or_d = ew;
        d.n = nw;
        d.rsa_mode = bits;
        d.input = mw;
        d.inlen = k * 4;
        d.output = out;
        if (ioctl(fd, IOCTL_RSA_ENC_DEC, &d) < 0) {
            printf("RSA %d ioctl failed\n", bits);
            continue;
        }
        /* the driver hands back big-endian bytes */
        snprintf(tag, sizeof(tag), "RSA %d", bits);
        hex(tag, (unsigned char *)out, k * 4);
    }
    fclose(f);
    close(fd);
    return 0;
}

static int dtrng(void)
{
    int fd = open("/dev/dtrng", O_RDWR), i;
    unsigned int v;

    if (fd < 0) {
        printf("DTRNG no /dev/dtrng\n");
        return 1;
    }
    for (i = 0; i < 4; i++) {
        v = i;
        ioctl(fd, IOCTL_DTRNG_CPU, &v);
        printf("DTRNG cpu %08x\n", v);
    }
    for (i = 0; i < 2; i++) {
        v = i;
        ioctl(fd, IOCTL_DTRNG_DMA, &v);
        printf("DTRNG irq %08x\n", v);
    }
    close(fd);
    return 0;
}

int main(int argc, char **argv)
{
    int r = 0;

    setvbuf(stdout, NULL, _IOLBF, 0);
    r |= aes();
    r |= hash();
    r |= rsa(argc > 1 ? argv[1] : "/etc/rsa-vectors");
    r |= dtrng();
    printf("CRYPTOTEST %s\n", r ? "INCOMPLETE" : "DONE");
    return 0;
}
