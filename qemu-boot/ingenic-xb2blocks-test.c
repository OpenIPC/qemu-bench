/*
 * The T40/T41 DRAWBOX and I2D through their vendor kernel drivers
 * (/dev/dbox, /dev/i2d), the way libimp drives them: a picture in
 * reserved memory (mapped uncached through /dev/mem), IOCTL_DBOX_START
 * with a dbox_param of boxes, IOCTL_I2D_START with an i2d_param that
 * rotates it.  Each result is checked here and reported as
 * "XB2TEST PASS|FAIL: ...".
 *
 *   ingenic-xb2blocks-test PHYS_ADDR      (4 MiB of free RAM at PHYS_ADDR)
 *
 * Built with -DI2D_H_ALIGN for T41, whose i2d_param ends in h_align.
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
#include <sys/mman.h>
#include <unistd.h>

/* drivers/video/ingenic_ipu/ingenic_drawbox.h */
struct dbox_ram {
    unsigned int box_x, box_y, box_w, box_h, line_w, line_l;
    unsigned int box_mode, color_mode;
};
struct dbox_param {
    unsigned int box_pbuf, img_w, img_h, boxs_num, is_rgba;
    struct dbox_ram ram_para[24];
};
#define IOCTL_DBOX_START        _IO('X', 106)

/* drivers/video/ingenic_i2d/ingenic_i2d.h */
struct i2d_param {
    unsigned int src_w, src_h, data_type;
    unsigned int rotate_enable, rotate_angle, flip_enable, mirr_enable;
    unsigned int src_addr_y, src_addr_uv, dst_addr_y, dst_addr_uv;
    unsigned int src_y_strid, src_uv_strid, dst_y_strid, dst_uv_strid;
#ifdef I2D_H_ALIGN
    unsigned int h_align;
#endif
};
#define IOCTL_I2D_START         _IO('D', 106)
#define PIX_FMT_NV12            10

#define W 320
#define H 192

static int fails;

static void check(const char *what, int ok, const char *fmt, int a, int b)
{
    char d[64];

    snprintf(d, sizeof(d), fmt, a, b);
    printf("XB2TEST %s: %s (%s)\n", ok ? "PASS" : "FAIL", what, d);
    fails += !ok;
}

static void test_dbox(uint8_t *v, unsigned phys)
{
    struct dbox_param p;
    uint8_t *y = v, *uv = v + W * H;
    int fd = open("/dev/dbox", O_RDWR), r;

    if (fd < 0) {
        check("open /dev/dbox", 0, "%d", fd, 0);
        return;
    }
    memset(v, 0x80, W * H * 3 / 2);
    memset(&p, 0, sizeof(p));
    p.box_pbuf = phys;
    p.img_w = W;
    p.img_h = H;
    p.boxs_num = 2;
    /* a red 4-pixel rectangle; a green vertical line */
    p.ram_para[0] = (struct dbox_ram){ 40, 20, 120, 80, 4, 0, 0, 0 };
    p.ram_para[1] = (struct dbox_ram){ 200, 40, 0, 100, 2, 0, 3, 2 };
    r = ioctl(fd, IOCTL_DBOX_START, &p);
    check("IOCTL_DBOX_START returns", r == 0, "%d", r, 0);
    check("rectangle top edge: red luma", y[20 * W + 100] == 0x4c,
          "0x%02x", y[20 * W + 100], 0);
    check("rectangle left edge, 4 wide", y[60 * W + 43] == 0x4c &&
          y[60 * W + 44] == 0x80, "0x%02x 0x%02x", y[60 * W + 43],
          y[60 * W + 44]);
    check("rectangle chroma: Cb 0x54 Cr 0xff",
          uv[10 * W + 100] == 0x54 && uv[10 * W + 101] == 0xff,
          "0x%02x 0x%02x", uv[10 * W + 100], uv[10 * W + 101]);
    check("inside untouched", y[60 * W + 100] == 0x80, "0x%02x",
          y[60 * W + 100], 0);
    check("green vertical line", y[100 * W + 200] == 0x95 &&
          y[100 * W + 202] == 0x80 && y[150 * W + 200] == 0x80,
          "0x%02x 0x%02x", y[100 * W + 200], y[150 * W + 200]);
    close(fd);
}

static void test_i2d(uint8_t *v, unsigned phys)
{
    struct i2d_param p;
    uint8_t *src = v, *dst = v + 0x100000;
    int fd = open("/dev/i2d", O_RDWR), r, x, yy, bad = 0;

    if (fd < 0) {
        check("open /dev/i2d", 0, "%d", fd, 0);
        return;
    }
    for (yy = 0; yy < H; yy++) {
        for (x = 0; x < W; x++) {
            src[yy * W + x] = (x + 3 * yy) & 0xff;
        }
    }
    memset(src + W * H, 0x80, W * H / 2);
    memset(dst, 0, W * H * 3 / 2);
    memset(&p, 0, sizeof(p));
    p.src_w = W;
    p.src_h = H;
    p.data_type = PIX_FMT_NV12;
    p.rotate_enable = 1;
    p.rotate_angle = 90;
    p.src_addr_y = phys;
    p.src_addr_uv = phys + W * H;
    p.dst_addr_y = phys + 0x100000;
    p.dst_addr_uv = phys + 0x100000 + W * H;
    p.src_y_strid = p.src_uv_strid = W;
    p.dst_y_strid = p.dst_uv_strid = H;
#ifdef I2D_H_ALIGN
    p.h_align = 16;
#endif
    r = ioctl(fd, IOCTL_I2D_START, &p);
    check("IOCTL_I2D_START returns", r == 0, "%d", r, 0);
    /* 90 clockwise: destination (x, y) holds source (column y, row H-1-x) */
    for (yy = 0; yy < W; yy++) {
        for (x = 0; x < H; x++) {
            bad += dst[yy * H + x] != src[(H - 1 - x) * W + yy];
        }
    }
    check("the picture rotated 90 degrees (pixels wrong)", bad == 0,
          "%d of %d", bad, W * H);
    check("its CbCr plane follows", dst[W * H] == 0x80 &&
          dst[W * H + W * H / 2 - 1] == 0x80, "0x%02x 0x%02x",
          dst[W * H], dst[W * H + W * H / 2 - 1]);
    close(fd);
}

int main(int argc, char **argv)
{
    unsigned phys = argc > 1 ? strtoul(argv[1], NULL, 0) : 0;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    uint8_t *v;

    if (!phys || fd < 0) {
        printf("XB2TEST FAIL: usage / /dev/mem (%d)\n", fd);
        return 1;
    }
    v = mmap(NULL, 4 << 20, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys);
    if (v == MAP_FAILED) {
        printf("XB2TEST FAIL: mmap 0x%x\n", phys);
        return 1;
    }
    test_dbox(v, phys);
    test_i2d(v, phys);
    printf("XB2TEST %s\n", fails ? "FAILED" : "DONE");
    return fails != 0;
}
