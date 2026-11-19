/*
 * Capture from FH8626V100's camera node, as a streaming application does,
 * to check the CSI -> ISP -> VPU frame path: OpenIPC's 4.19 fh8626-capture
 * driver on the twin's fh-csi / fh-isp models.
 *
 *   fh-v4l2-capture <video node> <sensor subdev> <isp subdev> [frames]
 *
 * Streams NV12 through MMAP buffers and checks each one: the payload is a
 * whole frame, the sequence advances, the picture is not blank, and it
 * changes from frame to frame (the model's scene has a moving box).  The
 * sensor's EXPOSURE and ANALOGUE_GAIN controls and the ISP's red and blue
 * balance (Q9) are read before and after: with fh8626-3ad running, its AE
 * and AWB write them.  Prints
 *
 *   capture: <n> frames, seq <first>..<last>, Y mean <first> -> <last>,
 *   exposure <a> -> <b>, gain <c> -> <d>, wb <r>/<b> -> <r>/<b>
 *
 * then "capture: ok", or what went wrong; exits 0 only for the former.
 *
 * No libc: the test copies this out of a flash partition, so it is kept to
 * a few KiB.  Static ARM EABI, raw system calls:
 *
 *   arm-linux-gnueabi-gcc -static -nostdlib -ffreestanding -Os -marm \
 *       -o fh-v4l2-capture qemu-boot/fh-v4l2-capture.c -lgcc
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define NR_EXIT         1
#define NR_WRITE        4
#define NR_OPEN         5
#define NR_CLOSE        6
#define NR_IOCTL        54
#define NR_MUNMAP       91
#define NR_POLL         168
#define NR_MMAP2        192

#define O_RDWR          2
#define O_NONBLOCK      04000
#define PROT_RW         3
#define MAP_SHARED      1
#define POLLIN          1
#define EAGAIN          11

/* videodev2.h, 32-bit ARM layouts */
#define VIDIOC_REQBUFS  0xC0145608
#define VIDIOC_QUERYBUF 0xC0445609
#define VIDIOC_QBUF     0xC044560F
#define VIDIOC_DQBUF    0xC0445611
#define VIDIOC_STREAMON 0x40045612
#define VIDIOC_STREAMOFF 0x40045613
#define VIDIOC_G_CTRL   0xC008561B
#define BUF_TYPE_CAPTURE 1
#define MEMORY_MMAP     1
#define CID_EXPOSURE    0x00980911
#define CID_ANALOGUE_GAIN 0x009E0903
#define CID_RED_BALANCE 0x0098090E
#define CID_BLUE_BALANCE 0x0098090F

#define NBUF            4
#define WIDTH           1280
#define HEIGHT          720
#define FRAME_BYTES     (WIDTH * HEIGHT * 3 / 2)

struct v4l2_requestbuffers {
    unsigned int count, type, memory, capabilities, reserved;
};

struct v4l2_buffer {
    unsigned int index, type, bytesused, flags, field;
    long tv_sec, tv_usec;
    unsigned int timecode[4];
    unsigned int sequence, memory;
    unsigned int offset;
    unsigned int length, reserved2, reserved;
};

struct v4l2_control {
    unsigned int id;
    int value;
};

struct pollfd {
    int fd;
    short events, revents;
};

static long sys6(long nr, long a, long b, long c, long d, long e, long f)
{
    register long r7 __asm__("r7") = nr;
    register long r0 __asm__("r0") = a;
    register long r1 __asm__("r1") = b;
    register long r2 __asm__("r2") = c;
    register long r3 __asm__("r3") = d;
    register long r4 __asm__("r4") = e;
    register long r5 __asm__("r5") = f;

    __asm__ volatile("svc 0" : "+r"(r0) : "r"(r7), "r"(r1), "r"(r2),
                     "r"(r3), "r"(r4), "r"(r5) : "memory");
    return r0;
}

static long sys(long nr, long a, long b, long c)
{
    return sys6(nr, a, b, c, 0, 0, 0);
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

static void put(const char *s)
{
    unsigned long n = 0;

    while (s[n]) {
        n++;
    }
    sys(NR_WRITE, 1, (long)s, n);
}

static void put_num(long v)
{
    char buf[16], *p = buf + sizeof(buf) - 1;
    int neg = v < 0;
    unsigned long u = neg ? -v : v;

    *p = 0;
    do {
        *--p = '0' + u % 10;
        u /= 10;
    } while (u);
    if (neg) {
        *--p = '-';
    }
    put(p);
}

static void fail(const char *what, long err)
{
    put("capture: ");
    put(what);
    put(" failed (");
    put_num(err);
    put(")\n");
    sys(NR_EXIT, 1, 0, 0);
}

static int get_ctrl(int fd, unsigned int id)
{
    struct v4l2_control c = { .id = id };
    long r = sys(NR_IOCTL, fd, VIDIOC_G_CTRL, (long)&c);

    if (r < 0) {
        fail("VIDIOC_G_CTRL", r);
    }
    return c.value;
}

static unsigned long atou(const char *s)
{
    unsigned long v = 0;

    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s++ - '0');
    }
    return v;
}

/* the Y plane's mean, sampled every 16th pixel */
static unsigned int y_mean(const unsigned char *f)
{
    unsigned long sum = 0, n = 0, i;

    for (i = 0; i < WIDTH * HEIGHT; i += 16, n++) {
        sum += f[i];
    }
    return sum / n;
}

static unsigned long checksum(const unsigned char *f)
{
    unsigned long sum = 0, i;

    for (i = 0; i < FRAME_BYTES; i += 4) {
        sum = sum * 31 + *(const unsigned int *)(f + i);
    }
    return sum;
}

void run(int argc, char **argv);
void run(int argc, char **argv)
{
    struct v4l2_requestbuffers req = { .count = NBUF,
                                       .type = BUF_TYPE_CAPTURE,
                                       .memory = MEMORY_MMAP };
    unsigned char *map[NBUF];
    unsigned long frames = argc > 4 ? atou(argv[4]) : 50, got = 0;
    unsigned long last_sum = 0, changed = 0;
    unsigned int first_seq = 0, last_seq = 0, first_y = 0, last_y = 0;
    int type = BUF_TYPE_CAPTURE, exp0, gain0, exp1, gain1, r0, b0, r1, b1;
    long vfd, sfd, ifd, r;
    unsigned int i;

    if (argc < 4) {
        put("usage: fh-v4l2-capture <video> <sensor subdev> <isp subdev> "
            "[frames]\n");
        sys(NR_EXIT, 2, 0, 0);
    }
    sfd = sys(NR_OPEN, (long)argv[2], O_RDWR, 0);
    if (sfd < 0) {
        fail("open sensor subdev", sfd);
    }
    ifd = sys(NR_OPEN, (long)argv[3], O_RDWR, 0);
    if (ifd < 0) {
        fail("open ISP subdev", ifd);
    }
    exp0 = get_ctrl(sfd, CID_EXPOSURE);
    gain0 = get_ctrl(sfd, CID_ANALOGUE_GAIN);
    r0 = get_ctrl(ifd, CID_RED_BALANCE);
    b0 = get_ctrl(ifd, CID_BLUE_BALANCE);
    vfd = sys(NR_OPEN, (long)argv[1], O_RDWR | O_NONBLOCK, 0);
    if (vfd < 0) {
        fail("open video node", vfd);
    }
    r = sys(NR_IOCTL, vfd, VIDIOC_REQBUFS, (long)&req);
    if (r < 0 || req.count < 2) {
        fail("VIDIOC_REQBUFS", r);
    }
    for (i = 0; i < req.count && i < NBUF; i++) {
        struct v4l2_buffer b = { .index = i, .type = BUF_TYPE_CAPTURE,
                                 .memory = MEMORY_MMAP };

        r = sys(NR_IOCTL, vfd, VIDIOC_QUERYBUF, (long)&b);
        if (r < 0 || b.length < FRAME_BYTES) {
            fail("VIDIOC_QUERYBUF", r < 0 ? r : b.length);
        }
        r = sys6(NR_MMAP2, 0, b.length, PROT_RW, MAP_SHARED, vfd,
                 b.offset >> 12);
        if (r < 0 && r > -4096) {
            fail("mmap", r);
        }
        map[i] = (unsigned char *)r;
        r = sys(NR_IOCTL, vfd, VIDIOC_QBUF, (long)&b);
        if (r < 0) {
            fail("VIDIOC_QBUF", r);
        }
    }
    r = sys(NR_IOCTL, vfd, VIDIOC_STREAMON, (long)&type);
    if (r < 0) {
        fail("VIDIOC_STREAMON", r);
    }
    while (got < frames) {
        struct pollfd p = { .fd = vfd, .events = POLLIN };
        struct v4l2_buffer b = { .type = BUF_TYPE_CAPTURE,
                                 .memory = MEMORY_MMAP };
        unsigned long sum;

        r = sys(NR_POLL, (long)&p, 1, 2000);
        if (r <= 0) {
            fail("a frame (none in 2 s)", r);
        }
        r = sys(NR_IOCTL, vfd, VIDIOC_DQBUF, (long)&b);
        if (r == -EAGAIN) {
            continue;
        }
        if (r < 0) {
            fail("VIDIOC_DQBUF", r);
        }
        if (b.bytesused != FRAME_BYTES || b.index >= NBUF) {
            fail("frame size", b.bytesused);
        }
        if (got && b.sequence <= last_seq) {
            fail("sequence", b.sequence);
        }
        last_seq = b.sequence;
        last_y = y_mean(map[b.index]);
        if (last_y <= 16 || last_y >= 235) {
            fail("picture level", last_y);
        }
        sum = checksum(map[b.index]);
        if (!got) {
            first_seq = b.sequence;
            first_y = last_y;
        } else if (sum != last_sum) {
            changed++;
        }
        last_sum = sum;
        got++;
        r = sys(NR_IOCTL, vfd, VIDIOC_QBUF, (long)&b);
        if (r < 0) {
            fail("VIDIOC_QBUF", r);
        }
    }
    sys(NR_IOCTL, vfd, VIDIOC_STREAMOFF, (long)&type);
    exp1 = get_ctrl(sfd, CID_EXPOSURE);
    gain1 = get_ctrl(sfd, CID_ANALOGUE_GAIN);
    r1 = get_ctrl(ifd, CID_RED_BALANCE);
    b1 = get_ctrl(ifd, CID_BLUE_BALANCE);

    put("capture: ");
    put_num(got);
    put(" frames, seq ");
    put_num(first_seq);
    put("..");
    put_num(last_seq);
    put(", Y mean ");
    put_num(first_y);
    put(" -> ");
    put_num(last_y);
    put(", exposure ");
    put_num(exp0);
    put(" -> ");
    put_num(exp1);
    put(", gain ");
    put_num(gain0);
    put(" -> ");
    put_num(gain1);
    put(", wb ");
    put_num(r0);
    put("/");
    put_num(b0);
    put(" -> ");
    put_num(r1);
    put("/");
    put_num(b1);
    put("\n");
    if (changed < got / 2) {
        fail("frames changing", changed);
    }
    put("capture: ok\n");
    sys(NR_EXIT, 0, 0, 0);
}

/* the kernel's initial stack: argc, then argv[] */
__attribute__((naked)) void _start(void)
{
    __asm__ volatile("ldr r0, [sp]\n"
                     "add r1, sp, #4\n"
                     "b run\n");
}
