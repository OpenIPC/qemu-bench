/*
 * Run the vendor's 3A beside an SDK binary that has none (Iford's RTOS
 * "preview" links no libcus3a).  See docs/sigmastar.md.  Build against the
 * image's libraries, with a start file that calls __uClibc_main:
 *
 *   arm-linux-gnueabihf-gcc -marm -march=armv7-a -mfpu=vfpv3-d16 -nostdlib \
 *     -fno-pic -no-pie -Wl,--dynamic-linker=/lib/ld-uClibc.so.0 \
 *     -Wl,--hash-style=sysv -Wl,--allow-shlib-undefined crt.S this.c \
 *     libcus3a.so libispalgo.so libmi_isp.so libmi_sys.so libmi_common.so \
 *     libcam_os_wrapper.so libcam_fs_wrapper.so libc.so.0 -lgcc
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

int printf(const char *fmt, ...);
int atoi(const char *s);
int usleep(unsigned us);
int MI_SYS_Init(int dev);
int CUS3A_Init(int dev);
int MI_ISP_EnableUserspace3A(int dev, int chn);

int main(int argc, char **argv)
{
    int i, secs = argc > 1 ? atoi(argv[1]) : 3600;

    printf("sys %d\n", MI_SYS_Init(0));
    printf("cus3a %d\n", CUS3A_Init(0));
    printf("userspace3a %x\n", MI_ISP_EnableUserspace3A(0, 0));
    for (i = 0; i < secs; i++) {
        usleep(1000000);
    }
    return 0;
}
