/*
 * IMP_Decoder end to end: the SDK's sample-Decoder-jpeg sequence on JPEG
 * files.  For each decoded picture it prints
 *   JPEGDFRAME n PHYS WIDTH HEIGHT SIZE
 * and holds the frame for a few seconds, while the host saves it from
 * guest memory (pmemsave): the parts' kernels need not have an SD card.
 *
 *   jpegd-test MAXW MAXH FILE...
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <imp/imp_system.h>
#include <imp/imp_decoder.h>

#define SLOT        (4u << 20)

/* built statically against the SDK's libimp.a: no dynamic loading */
void *dlopen(const char *file, int mode)
{
    return NULL;
}

void *dlsym(void *handle, const char *name)
{
    return NULL;
}

char *dlerror(void)
{
    return (char *)"no dynamic loading";
}

int dlclose(void *handle)
{
    return 0;
}

int main(int argc, char **argv)
{
    IMPDecoderCHNAttr attr = { .decAttr = {
        .decType = PT_JPEG, .pixelFormat = PIX_FMT_NV12, .nrKeepStream = 2,
        .frmRateNum = 25, .frmRateDen = 1,
    } };
    int n;

    if (argc < 4) {
        fprintf(stderr, "usage: jpegd-test MAXW MAXH FILE...\n");
        return 2;
    }
    attr.decAttr.maxWidth = atoi(argv[1]);
    attr.decAttr.maxHeight = atoi(argv[2]);
    if (IMP_System_Init() < 0 || IMP_Decoder_CreateChn(0, &attr) < 0 ||
        IMP_Decoder_StartRecvPic(0) < 0) {
        printf("jpegd-test: IMP_System_Init / IMP_Decoder setup failed\n");
        return 1;
    }
    for (n = 3; n < argc; n++) {
        IMPDecoderStream st;
        IMPFrameInfo *f = NULL;
        FILE *in = fopen(argv[n], "rb");
        uint8_t *buf;
        long len;

        if (!in || fseek(in, 0, SEEK_END) || (len = ftell(in)) <= 0) {
            printf("jpegd-test: cannot read %s\n", argv[n]);
            return 1;
        }
        rewind(in);
        buf = malloc(len);
        if (!buf || fread(buf, 1, len, in) != (size_t)len) {
            return 1;
        }
        fclose(in);
        memset(&st, 0, sizeof(st));
        st.decoderNal.i_payload = len;
        st.decoderNal.p_payload = buf;
        st.decoderNal.timeStamp = n;
        if (IMP_Decoder_SendStreamTimeout(0, &st, 2000) < 0 ||
            IMP_Decoder_PollingFrame(0, 5000) < 0 ||
            IMP_Decoder_GetFrame(0, &f) < 0 || !f) {
            printf("jpegd-test: %s not decoded\n", argv[n]);
            return 1;
        }
        printf("JPEGDFRAME %d 0x%08x %u %u %u\n", n - 3, (unsigned)f->phyAddr,
               f->width, f->height, f->width * f->height * 3 / 2);
        fflush(stdout);
        sleep(8);
        IMP_Decoder_ReleaseFrame(0, f);
        free(buf);
    }
    IMP_Decoder_StopRecvPic(0);
    IMP_Decoder_DestroyChn(0);
    IMP_System_Exit();
    printf("jpegd-test: %d decoded\n", argc - 3);
    return 0;
}
