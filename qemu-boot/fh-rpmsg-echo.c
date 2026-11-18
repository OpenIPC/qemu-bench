/*
 * Round trips through FH8626V100's ARC: the RPMsg echo endpoint OpenIPC's
 * ARC firmware announces ("rpmsg-arc-echo", address 1024), reached through
 * the 4.19 kernel's rpmsg_char once the test has bound rpmsg_chrdev to the
 * channel.  Each message goes out on Linux's TX ring, and the ARC copies it
 * back on the RX ring, to the address it came from.
 *
 * No libc: the test copies this out of a flash partition, so it is kept to
 * a few KiB.  Static ARM EABI, raw system calls:
 *
 *   arm-linux-gnueabi-gcc -static -nostdlib -ffreestanding -Os -marm \
 *       -o fh-rpmsg-echo qemu-boot/fh-rpmsg-echo.c
 *
 * Prints "rpmsg echo: <n> ok" when every message came back unchanged, or
 * what went wrong; exits 0 only for the former.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define NR_EXIT         1
#define NR_READ         3
#define NR_WRITE        4
#define NR_OPEN         5
#define NR_CLOSE        6
#define NR_IOCTL        54
#define NR_NANOSLEEP    162

#define O_RDWR          2
#define O_NONBLOCK      04000
#define EAGAIN          11

/* _IOW(0xb5, 0x1, struct rpmsg_endpoint_info), 40 bytes */
#define RPMSG_CREATE_EPT_IOCTL  0x4028b501

#define ECHO_ADDR       1024
#define MAX_PAYLOAD     496     /* 512-byte buffers less the RPMsg header */

struct rpmsg_endpoint_info {
    char name[32];
    unsigned int src;
    unsigned int dst;
};

static long sys(long nr, long a, long b, long c)
{
    register long r7 __asm__("r7") = nr;
    register long r0 __asm__("r0") = a;
    register long r1 __asm__("r1") = b;
    register long r2 __asm__("r2") = c;

    __asm__ volatile("svc 0" : "+r"(r0) : "r"(r7), "r"(r1), "r"(r2)
                     : "memory");
    return r0;
}

/* What the compiler and libgcc call on their own */
void *memset(void *d, int c, unsigned long n)
{
    unsigned char *p = d;

    while (n--) {
        *p++ = (unsigned char)c;
    }
    return d;
}

int raise(int sig)
{
    sys(NR_EXIT, 128 + sig, 0, 0);
    return 0;
}

static unsigned long len(const char *s)
{
    unsigned long n = 0;

    while (s[n]) {
        n++;
    }
    return n;
}

static void put(const char *s)
{
    sys(NR_WRITE, 1, (long)s, len(s));
}

static void put_num(unsigned long v)
{
    char buf[12], *p = buf + sizeof(buf) - 1;

    *p = 0;
    do {
        *--p = '0' + v % 10;
        v /= 10;
    } while (v);
    put(p);
}

static void nap_ms(long ms)
{
    long ts[2] = { 0, ms * 1000000 };

    sys(NR_NANOSLEEP, (long)ts, 0, 0);
}

static void fail(const char *what, long err, unsigned long i)
{
    put("rpmsg echo: ");
    put(what);
    put(" failed (");
    put_num(-err);
    put(") at message ");
    put_num(i);
    put("\n");
    sys(NR_EXIT, 1, 0, 0);
}

/* One message of n bytes, the i-th: a pattern that differs per message. */
static void fill(unsigned char *buf, unsigned long n, unsigned long i)
{
    unsigned long k;

    for (k = 0; k < n; k++) {
        buf[k] = (unsigned char)(i * 7 + k * 13 + 1);
    }
}

void _start(void)
{
    static const unsigned long sizes[] = { 1, 2, 3, 16, 100, 255, 256,
                                           MAX_PAYLOAD };
    struct rpmsg_endpoint_info info = { .src = 0xFFFFFFFF,
                                        .dst = ECHO_ADDR };
    static unsigned char out[MAX_PAYLOAD], in[MAX_PAYLOAD + 16];
    const char *name = "rpmsg-arc-echo";
    unsigned long i, k, n = 64;
    long ctrl, ept = -1, r;

    for (k = 0; name[k]; k++) {
        info.name[k] = name[k];
    }
    ctrl = sys(NR_OPEN, (long)"/dev/rpmsg_ctrl0", O_RDWR, 0);
    if (ctrl < 0) {
        fail("open /dev/rpmsg_ctrl0", ctrl, 0);
    }
    r = sys(NR_IOCTL, ctrl, RPMSG_CREATE_EPT_IOCTL, (long)&info);
    if (r < 0) {
        fail("RPMSG_CREATE_EPT_IOCTL", r, 0);
    }
    /* devtmpfs makes the node from its own thread */
    for (k = 0; k < 50 && ept < 0; k++) {
        ept = sys(NR_OPEN, (long)"/dev/rpmsg0", O_RDWR | O_NONBLOCK, 0);
        if (ept < 0) {
            nap_ms(20);
        }
    }
    if (ept < 0) {
        fail("open /dev/rpmsg0", ept, 0);
    }

    for (i = 0; i < n; i++) {
        unsigned long size = sizes[i % (sizeof(sizes) / sizeof(sizes[0]))];

        fill(out, size, i);
        r = sys(NR_WRITE, ept, (long)out, size);
        if (r != (long)size) {
            fail("write", r < 0 ? r : -1, i);
        }
        for (k = 0; k < 200; k++) {             /* 2 s */
            r = sys(NR_READ, ept, (long)in, sizeof(in));
            if (r != -EAGAIN) {
                break;
            }
            nap_ms(10);
        }
        if (r != (long)size) {
            fail(r == -EAGAIN ? "reply (none in 2 s)" : "read",
                 r < 0 ? r : -1, i);
        }
        for (k = 0; k < size; k++) {
            if (in[k] != out[k]) {
                fail("compare", -1, i);
            }
        }
    }
    sys(NR_CLOSE, ept, 0, 0);
    put("rpmsg echo: ");
    put_num(n);
    put(" ok\n");
    sys(NR_EXIT, 0, 0, 0);
}
