/*
 * hisi-venc-userframe: drive the HiMPP V4 encoder (VEDU + JPGE) with
 * frames from a VB pool, so the encoder is tested without VI or VPSS.
 *
 * The call order is sample_venc's (SAMPLE_COMM_SYS_Init, then
 * SAMPLE_COMM_VENC_Creat / StartRecvFrame / GetStream), with
 * VENC_SendFrame where the sample binds a VPSS channel.  Each
 * frame is NV21 (PIXEL_FORMAT_YVU_SEMIPLANAR_420, the MPP default):
 * moving vertical bars plus the frame number as a row of 16 binary
 * squares, which the host checks after decoding.
 *
 *   hisi-venc-userframe [-w W] [-h H] [-n FRAMES] [-o DIR] [h264|h265|jpeg]...
 *
 * Streams go to DIR/venc-<codec>.<ext>.  Prints one line per frame and
 * "RESULT: PASS" when every channel produced every frame.
 *
 * Built against the public headers of OpenIPC/openhisilicon: their
 * GK_API_* names are libgk_api.so's (the Goke releases), and with
 * -DHISI_LIBMPI they become the HI_MPI_* exports of HiSilicon's libmpi.so.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifdef HISI_LIBMPI
#define GK_API_SYS_Init             HI_MPI_SYS_Init
#define GK_API_SYS_Exit             HI_MPI_SYS_Exit
#define GK_API_SYS_Mmap             HI_MPI_SYS_Mmap
#define GK_API_SYS_Munmap           HI_MPI_SYS_Munmap
#define GK_API_VB_SetConfig         HI_MPI_VB_SetConfig
#define GK_API_VB_Init              HI_MPI_VB_Init
#define GK_API_VB_Exit              HI_MPI_VB_Exit
#define GK_API_VB_GetBlock          HI_MPI_VB_GetBlock
#define GK_API_VB_ReleaseBlock      HI_MPI_VB_ReleaseBlock
#define GK_API_VB_Handle2PhysAddr   HI_MPI_VB_Handle2PhysAddr
#define GK_API_VB_Handle2PoolId     HI_MPI_VB_Handle2PoolId
#define GK_API_VENC_CreateChn       HI_MPI_VENC_CreateChn
#define GK_API_VENC_DestroyChn      HI_MPI_VENC_DestroyChn
#define GK_API_VENC_StartRecvFrame  HI_MPI_VENC_StartRecvFrame
#define GK_API_VENC_StopRecvFrame   HI_MPI_VENC_StopRecvFrame
#define GK_API_VENC_QueryStatus     HI_MPI_VENC_QueryStatus
#define GK_API_VENC_GetStream       HI_MPI_VENC_GetStream
#define GK_API_VENC_ReleaseStream   HI_MPI_VENC_ReleaseStream
#define GK_API_VENC_SendFrame       HI_MPI_VENC_SendFrame
#define GK_API_VENC_GetFd           HI_MPI_VENC_GetFd
#endif

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "gk_api_sys.h"
#include "gk_api_vb.h"
#include "gk_api_venc.h"

/*
 * The vendor libraries were linked against uClibc; under musl
 * these are the symbols they miss (as in test-ive-mpi.c).
 */
int memcpy_s(void *d, size_t dn, const void *s, size_t n)
{
    memcpy(d, s, n < dn ? n : dn);
    return 0;
}
int memset_s(void *d, size_t dn, int c, size_t n)
{
    memset(d, c, n < dn ? n : dn);
    return 0;
}
int memmove_s(void *d, size_t dn, const void *s, size_t n)
{
    memmove(d, s, n < dn ? n : dn);
    return 0;
}
int strncpy_s(char *d, size_t dn, const char *s, size_t n)
{
    strncpy(d, s, n < dn ? n : dn);
    return 0;
}
int snprintf_s(char *d, size_t dn, size_t n, const char *f, ...)
{
    va_list a;
    int r;

    (void)n;
    va_start(a, f);
    r = vsnprintf(d, dn, f, a);
    va_end(a);
    return r;
}
const unsigned short int *__ctype_b;
size_t _stdlib_mb_cur_max(void) { return 0; }
int __fputc_unlocked(int c, FILE *stream) { return fputc(c, stream); }
int __fgetc_unlocked(FILE *stream) { return fgetc(stream); }

/*
 * uClibc's off_t is 32 bits and musl's 64: the MPI library passes the
 * physical address of every MMZ buffer it maps as a 32-bit argument,
 * which musl's mmap() would read with garbage in the high word.  This
 * definition takes the libraries' calls (musl itself uses an internal
 * alias, not this symbol).
 */
void *mmap(void *addr, size_t len, int prot, int flags, int fd, long off)
{
    return (void *)syscall(SYS_mmap2, addr, len, prot, flags, fd,
                           (unsigned long)off >> 12);
}

/*
 * The MPI library also imports the audio voice-quality libraries; musl binds
 * every symbol at load time, so stand-ins keep the encoder-only test
 * free of them.
 */
#define AUDIO_STUB(name) int name(void) { return -1; }
#ifdef HISI_LIBMPI
AUDIO_STUB(HI_DNVQE_Create) AUDIO_STUB(HI_DNVQE_Destroy)
AUDIO_STUB(HI_DNVQE_GetConfig) AUDIO_STUB(HI_DNVQE_ReadFrame)
AUDIO_STUB(HI_DNVQE_WriteFrame) AUDIO_STUB(HI_UPVQE_Create)
AUDIO_STUB(HI_UPVQE_Destroy) AUDIO_STUB(HI_UPVQE_GetConfig)
AUDIO_STUB(HI_UPVQE_GetVolume) AUDIO_STUB(HI_UPVQE_ReadFrame)
AUDIO_STUB(HI_UPVQE_SetVolume) AUDIO_STUB(HI_UPVQE_WriteFrame)
AUDIO_STUB(HI_VOICE_DecodeFrame) AUDIO_STUB(HI_VOICE_DecReset)
AUDIO_STUB(HI_VOICE_EncodeFrame) AUDIO_STUB(HI_VOICE_EncReset)
AUDIO_STUB(HI_VQE_RegisterModuleHandle)
#else
AUDIO_STUB(AUDIO_DNVQE_Create) AUDIO_STUB(AUDIO_DNVQE_Destroy)
AUDIO_STUB(AUDIO_DNVQE_GetConfig) AUDIO_STUB(AUDIO_DNVQE_ReadFrame)
AUDIO_STUB(AUDIO_DNVQE_WriteFrame) AUDIO_STUB(AUDIO_UPVQE_Create)
AUDIO_STUB(AUDIO_UPVQE_Destroy) AUDIO_STUB(AUDIO_UPVQE_GetConfig)
AUDIO_STUB(AUDIO_UPVQE_GetVolume) AUDIO_STUB(AUDIO_UPVQE_ReadFrame)
AUDIO_STUB(AUDIO_UPVQE_SetVolume) AUDIO_STUB(AUDIO_UPVQE_WriteFrame)
AUDIO_STUB(AUDIO_VQE_RegisterModuleHandle)
AUDIO_STUB(VOICE_DecodeFrame) AUDIO_STUB(VOICE_DecReset)
AUDIO_STUB(VOICE_EncodeFrame) AUDIO_STUB(VOICE_EncReset)
#endif

#define ALIGN_UP(x, a) (((x) + (a) - 1) & ~((a) - 1))

enum codec { C_H264, C_H265, C_JPEG, C_COUNT };

static const struct {
    const char *name, *ext;
    PAYLOAD_TYPE_E pt;
} codecs[C_COUNT] = {
    [C_H264] = { "h264", "h264", PT_H264 },
    [C_H265] = { "h265", "h265", PT_H265 },
    [C_JPEG] = { "jpeg", "mjpeg", PT_MJPEG },
};

static int width = 640, height = 368, nframes = 10;
static const char *outdir = "/tmp";

#define CHECK(call) do {                                            \
        GK_S32 _r = (call);                                         \
        if (_r != GK_SUCCESS) {                                     \
            printf("FAIL: %s = 0x%08x (line %d)\n", #call,          \
                   (unsigned)_r, __LINE__);                         \
            return -1;                                              \
        }                                                           \
    } while (0)

static int venc_create(VENC_CHN ch, enum codec c)
{
    VENC_CHN_ATTR_S a;
    VENC_RECV_PIC_PARAM_S recv = { .s32RecvPicNum = -1 };

    memset(&a, 0, sizeof(a));
    a.stVencAttr.enType = codecs[c].pt;
    a.stVencAttr.u32MaxPicWidth = width;
    a.stVencAttr.u32MaxPicHeight = height;
    a.stVencAttr.u32PicWidth = width;
    a.stVencAttr.u32PicHeight = height;
    a.stVencAttr.u32BufSize = ALIGN_UP(width * height * 2, 64);
    a.stVencAttr.u32Profile = 0;
    a.stVencAttr.bByFrame = GK_TRUE;
    a.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    a.stGopAttr.stNormalP.s32IPQpDelta = 0;

    switch (c) {
    case C_H264:
        a.stRcAttr.enRcMode = VENC_RC_MODE_H264FIXQP;
        a.stRcAttr.stH264FixQp.u32Gop = 5;
        a.stRcAttr.stH264FixQp.u32SrcFrameRate = 25;
        a.stRcAttr.stH264FixQp.fr32DstFrameRate = 25;
        a.stRcAttr.stH264FixQp.u32IQp = 26;
        a.stRcAttr.stH264FixQp.u32PQp = 28;
        a.stRcAttr.stH264FixQp.u32BQp = 30;
        break;
    case C_H265:
        a.stRcAttr.enRcMode = VENC_RC_MODE_H265FIXQP;
        a.stRcAttr.stH265FixQp.u32Gop = 5;
        a.stRcAttr.stH265FixQp.u32SrcFrameRate = 25;
        a.stRcAttr.stH265FixQp.fr32DstFrameRate = 25;
        a.stRcAttr.stH265FixQp.u32IQp = 26;
        a.stRcAttr.stH265FixQp.u32PQp = 28;
        a.stRcAttr.stH265FixQp.u32BQp = 30;
        break;
    case C_JPEG:
        a.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        a.stRcAttr.stMjpegFixQp.u32SrcFrameRate = 25;
        a.stRcAttr.stMjpegFixQp.fr32DstFrameRate = 25;
        a.stRcAttr.stMjpegFixQp.u32Qfactor = 90;
        break;
    default:
        return -1;
    }

    CHECK(GK_API_VENC_CreateChn(ch, &a));
    CHECK(GK_API_VENC_StartRecvFrame(ch, &recv));
    return 0;
}

/* Moving bars, then the frame number in binary along the top rows. */
static void paint_nv21(uint8_t *y, uint8_t *vu, int stride, int n)
{
    static const uint8_t bar_y[8] = { 235, 210, 170, 145, 106, 81, 41, 16 };
    int bw = width / 8, x, r;

    for (r = 0; r < height; r++) {
        for (x = 0; x < width; x++) {
            int b = ((x + n * 8) / bw) % 8;
            y[r * stride + x] = bar_y[b];
        }
    }
    for (r = 0; r < height / 2; r++) {
        for (x = 0; x < width / 2; x++) {
            int b = ((x * 2 + n * 8) / bw) % 8;
            vu[r * stride + x * 2] = 128 + (b & 1 ? 60 : -60);     /* V */
            vu[r * stride + x * 2 + 1] = 128 + (b & 2 ? 60 : -60); /* U */
        }
    }
    for (x = 0; x < 16; x++) {
        uint8_t v = (n >> (15 - x)) & 1 ? 235 : 16;
        for (r = 0; r < 32; r++) {
            memset(&y[r * stride + x * (width / 16)], v, width / 16);
        }
    }
}

static int drain(VENC_CHN ch, FILE *f, int frame, enum codec c)
{
    VENC_CHN_STATUS_S st;
    VENC_STREAM_S s;
    VENC_PACK_S packs[16];
    fd_set fds;
    struct timeval tv = { .tv_sec = 10 };
    int fd = GK_API_VENC_GetFd(ch);
    unsigned i, bytes = 0;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) {
        printf("FAIL: %s frame %d: no stream within 10 s\n",
               codecs[c].name, frame);
        return -1;
    }
    CHECK(GK_API_VENC_QueryStatus(ch, &st));
    if (st.u32CurPacks == 0 || st.u32CurPacks > 16) {
        printf("FAIL: %s frame %d: %u packs\n", codecs[c].name, frame,
               st.u32CurPacks);
        return -1;
    }
    memset(&s, 0, sizeof(s));
    s.pstPack = packs;
    s.u32PackCount = st.u32CurPacks;
    CHECK(GK_API_VENC_GetStream(ch, &s, 1000));
    printf("%s frame %d seq %u:", codecs[c].name, frame, s.u32Seq);
    for (i = 0; i < s.u32PackCount; i++) {
        VENC_PACK_S *p = &s.pstPack[i];
        fwrite(p->pu8Addr + p->u32Offset, 1, p->u32Len - p->u32Offset, f);
        bytes += p->u32Len - p->u32Offset;
        printf(" [type %d %u B]", p->DataType.enH264EType,
               p->u32Len - p->u32Offset);
    }
    printf(" = %u B\n", bytes);
    fflush(stdout);
    CHECK(GK_API_VENC_ReleaseStream(ch, &s));
    return bytes ? 0 : -1;
}

int main(int argc, char **argv)
{
    VB_CONFIG_S vb;
    bool want[C_COUNT] = { false };
    FILE *out[C_COUNT] = { NULL };
    int stride, frame_size, opt, c, n, nwant = 0, fails = 0;

    while ((opt = getopt(argc, argv, "w:h:n:o:")) != -1) {
        switch (opt) {
        case 'w': width = atoi(optarg); break;
        case 'h': height = atoi(optarg); break;
        case 'n': nframes = atoi(optarg); break;
        case 'o': outdir = optarg; break;
        default:
            fprintf(stderr, "usage: %s [-w W] [-h H] [-n N] [-o DIR] "
                    "[h264|h265|jpeg]...\n", argv[0]);
            return 2;
        }
    }
    for (; optind < argc; optind++) {
        for (c = 0; c < C_COUNT; c++) {
            if (!strcmp(argv[optind], codecs[c].name)) {
                want[c] = true;
            }
        }
    }
    for (c = 0; c < C_COUNT; c++) {
        nwant += want[c];
    }
    if (!nwant) {
        want[C_H264] = want[C_H265] = want[C_JPEG] = true;
    }

    stride = ALIGN_UP(width, 16);
    frame_size = stride * height * 3 / 2;

    GK_API_SYS_Exit();
    GK_API_VB_Exit();
    memset(&vb, 0, sizeof(vb));
    vb.u32MaxPoolCnt = 1;
    vb.astCommPool[0].u64BlkSize = frame_size;
    vb.astCommPool[0].u32BlkCnt = 4;
    CHECK(GK_API_VB_SetConfig(&vb));
    CHECK(GK_API_VB_Init());
    CHECK(GK_API_SYS_Init());

    for (c = 0; c < C_COUNT; c++) {
        char path[256];

        if (!want[c]) {
            continue;
        }
        if (venc_create(c, c) < 0) {
            return 1;
        }
        snprintf(path, sizeof(path), "%s/venc-%s.%s", outdir,
                 codecs[c].name, codecs[c].ext);
        out[c] = fopen(path, "wb");
        if (!out[c]) {
            printf("FAIL: open %s: %s\n", path, strerror(errno));
            return 1;
        }
    }

    for (n = 0; n < nframes; n++) {
        VIDEO_FRAME_INFO_S fi;
        VB_BLK blk = GK_API_VB_GetBlock(VB_INVALID_POOLID, frame_size, NULL);
        GK_U64 phys;
        uint8_t *virt;

        if (blk == VB_INVALID_HANDLE) {
            printf("FAIL: VB_GetBlock\n");
            return 1;
        }
        phys = GK_API_VB_Handle2PhysAddr(blk);
        virt = GK_API_SYS_Mmap(phys, frame_size);
        paint_nv21(virt, virt + stride * height, stride, n);
        GK_API_SYS_Munmap(virt, frame_size);

        memset(&fi, 0, sizeof(fi));
        fi.u32PoolId = GK_API_VB_Handle2PoolId(blk);
        fi.enModId = MOD_ID_VB;
        fi.stVFrame.u32Width = width;
        fi.stVFrame.u32Height = height;
        fi.stVFrame.enField = VIDEO_FIELD_FRAME;
        fi.stVFrame.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        fi.stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR;
        fi.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
        fi.stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;
        fi.stVFrame.enColorGamut = COLOR_GAMUT_BT709;
        fi.stVFrame.u32Stride[0] = stride;
        fi.stVFrame.u32Stride[1] = stride;
        fi.stVFrame.u64PhyAddr[0] = phys;
        fi.stVFrame.u64PhyAddr[1] = phys + stride * height;
        fi.stVFrame.u32TimeRef = n * 2;
        fi.stVFrame.u64PTS = n * 40000ULL;

        for (c = 0; c < C_COUNT; c++) {
            GK_S32 r;

            if (!want[c]) {
                continue;
            }
            r = GK_API_VENC_SendFrame(c, &fi, 1000);
            if (r != GK_SUCCESS) {
                printf("FAIL: %s frame %d: SendFrame = 0x%08x\n",
                       codecs[c].name, n, (unsigned)r);
                fails++;
                continue;
            }
            if (drain(c, out[c], n, c) < 0) {
                fails++;
            }
        }
        GK_API_VB_ReleaseBlock(blk);
        if (fails) {
            break;
        }
    }

    for (c = 0; c < C_COUNT; c++) {
        if (want[c]) {
            GK_API_VENC_StopRecvFrame(c);
            GK_API_VENC_DestroyChn(c);
            fclose(out[c]);
        }
    }
    GK_API_SYS_Exit();
    GK_API_VB_Exit();

    printf("RESULT: %s\n", fails ? "FAIL" : "PASS");
    return fails ? 1 : 0;
}
