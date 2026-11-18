/*
 * Guest client for test-ingenic-nna.sh: the T40/T41 NNA DMA as Ingenic's
 * libvenus drives it, through libvenus's own dma_func code (linked from
 * the public magik-toolkit's libvenus.a).
 *
 *   nna-test DDR_PADDR
 *
 * DDR_PADDR is a free, physically contiguous DDR area (the test keeps it
 * out of the kernel with mem=).  The client opens /dev/soc-nna (whose
 * open() enables the XBurst2 hardware registers for user mode), maps the
 * NNDMA, its descriptor RAM, the ORAM and the DDR area through /dev/mem
 * as libvenus's __aie_mmap does, then
 *   - moves two blocks DDR -> ORAM with w_d2o_orasetrun (read channel 1)
 *     and waits for it with aie_wait_nndma_rdchn (rdhwr $9),
 *   - moves them back ORAM -> DDR elsewhere on write channel 0
 *     (aie_run_nndma_wrchn / aie_wait_nndma_wrchn, rdhwr $11) from
 *     descriptors in libvenus's format,
 *   - reads rdhwr $4 after an LL, which XBurst2 answers with the LL's
 *     physical address,
 * and prints NNATEST lines for the host to check.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define NNDMA_IO_P      0x12508000u
#define NNDMA_DESRAM_P  0x12500000u
#define ORAM_P          0x12620000u     /* where libvenus maps the ORAM */
#define ORAM_LEN        0x70000u
#define DDR_LEN         0x100000u
#define SOC_NNA_VERSION 0xc0046306u     /* _IOWR('c', 6, int) */

/* what libvenus's dma_func.cc.o expects from __aie_mmap */
void *__nndma_io_vbase, *__nndma_desram_vbase, *__nndma_fastio_vbase;
void *__oram_vbase, *__ddr_vbase, *__ddr_pbase;

void w_d2o_orasetrun(int n, void **ddr, void **oram, int *bytes);
int aie_wait_nndma_rdchn(int ch);
void aie_run_nndma_wrchn(int ch, int des, int bytes);
int aie_wait_nndma_wrchn(int ch);

static void *map(int fd, uint32_t paddr, uint32_t len)
{
    void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, paddr);

    if (p == MAP_FAILED) {
        printf("NNATEST FAIL mmap %08x\n", paddr);
        exit(1);
    }
    return p;
}

static uint32_t ll_rdhwr4(volatile uint32_t *p)
{
    uint32_t v;

    __asm__ __volatile__(".set push\n.set mips32r2\n.set noreorder\n"
                         "ll %0, 0(%1)\n"
                         "rdhwr %0, $4\n"
                         ".set pop\n" : "=r"(v) : "r"(p) : "memory");
    return v;
}

int main(int argc, char **argv)
{
    uint32_t ddr_p = argc > 1 ? strtoul(argv[1], NULL, 0) : 0x07000000;
    struct { uint32_t version, ext, paddr, size; } ver = { 0 };
    cpu_set_t cpus;
    int fd, nna, i, bad;
    uint8_t *ddr, *oram, *src, *dst;
    uint32_t *des;

    /* HWREna is per core: stay on the core that opens the device */
    CPU_ZERO(&cpus);
    CPU_SET(0, &cpus);
    sched_setaffinity(0, sizeof(cpus), &cpus);

    nna = open("/dev/soc-nna", O_RDWR);
    if (nna < 0) {
        printf("NNATEST FAIL open /dev/soc-nna\n");
        return 1;
    }
    printf("NNATEST version ioctl %d: %08x\n",
           ioctl(nna, SOC_NNA_VERSION, &ver), ver.version);
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    __nndma_io_vbase = map(fd, NNDMA_IO_P, 0x1000);
    __nndma_desram_vbase = map(fd, NNDMA_DESRAM_P, 0x4000);
    __oram_vbase = oram = map(fd, ORAM_P, ORAM_LEN);
    __ddr_vbase = ddr = map(fd, ddr_p, DDR_LEN);
    __ddr_pbase = (void *)(uintptr_t)ddr_p;
    __nndma_fastio_vbase = ddr + DDR_LEN - 0x1000;

    src = ddr;
    dst = ddr + 0x40000;
    for (i = 0; i < 12288; i++) {
        src[i] = (uint8_t)(i * 13 + 5);
    }
    memset(dst, 0, 12288);
    memset(oram, 0, 0x20000);

    /* DDR -> ORAM: libvenus's own descriptor writer and channel start */
    {
        void *d[2] = { src, src + 8192 };
        void *o[2] = { oram, oram + 0x10000 };
        int b[2] = { 8192, 4096 };

        w_d2o_orasetrun(2, d, o, b);
        aie_wait_nndma_rdchn(1);
    }
    bad = memcmp(oram, src, 8192) || memcmp(oram + 0x10000, src + 8192, 4096);
    printf("NNATEST d2o: %s\n", bad ? "FAIL" : "ok");

    /*
     * ORAM -> DDR on write channel 0: descriptors 100.. in libvenus's
     * format (the ORAM offset counts from 0x12600000)
     */
    des = (uint32_t *)__nndma_desram_vbase + 2 * 100;
    {
        uint32_t n0 = 8192 / 64 - 1, n1 = 4096 / 64 - 1;

        des[0] = (ddr_p + 0x40000) | ((n0 >> 12) << 1) | 1;
        des[1] = (n0 << 20) | ((ORAM_P - 0x12600000u) >> 6);
        des[2] = (ddr_p + 0x40000 + 8192) | ((n1 >> 12) << 1);
        des[3] = (n1 << 20) | ((ORAM_P + 0x10000 - 0x12600000u) >> 6);
    }
    __sync_synchronize();
    aie_run_nndma_wrchn(0, 100, 0);
    aie_wait_nndma_wrchn(0);
    bad = memcmp(dst, src, 12288);
    printf("NNATEST o2d: %s\n", bad ? "FAIL" : "ok");

    printf("NNATEST rdhwr4: %08x want %08x\n",
           ll_rdhwr4((volatile uint32_t *)(ddr + 0x80)), ddr_p + 0x80);
    printf("NNATEST done\n");
    return 0;
}
