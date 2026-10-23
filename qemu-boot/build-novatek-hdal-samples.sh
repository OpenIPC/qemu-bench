#!/bin/bash
# Build the vendor SDK's HDAL sample programs for qemu-boot/test-novatek-hdal.sh.
#
# The samples (code/hdal/samples in na51089_linux_sdk_*_v1.01.002) drive
# the media engines through the HDAL: capture -> video process -> encode
# (hd_video_record), JPEG snapshots, capture alone, encoding frames from
# a file, the 2D graphics engine, the display (a file, and the sensor
# live), audio capture, playback, live loopback and AAC recording, and the
# AI engines' operators (ai_op) and a network (ai_net).  They are vendor
# code, as is the SDK, so neither is in the repository or in CI.
#
# They run against the OpenIPC release's own libhdal.so, the library
# OpenIPC's kernel modules match.  That library is a uClibc build
# (it needs libc.so.0), and the OpenIPC rootfs provides libc.so.0 from
# musl, whose mmap() takes a 64-bit offset where uClibc's code passes 32
# bits: libhdal's first mmap of /dev/log_vg then gets a garbage offset,
# fails, and libhdal writes through MAP_FAILED.  So the samples are built
# with the SDK's uClibc toolchain (arm-ca9-linux-uclibcgnueabihf-8.4.01)
# and run under its runtime -- ld-uClibc.so.0, libc.so.0, libgcc_s --
# which the output carries in uc/ and the samples find at /tmp/uc.
#
# The sensor samples are set up for an SC501AI in its 2880x1620 mode
# (machine sensor=sc501ai), which is what the release's sensor driver
# offers; the SDK's hd_video_snapshot and hd_video_capture_only name an
# IMX290, and the record sample's 1920x1080 buffers are too small for it.
#
# hd_video_encode_only stops the encoder on "q" while its feed thread may
# still be between its exit check and hd_videoenc_push_in_buf(), whose
# push into the stopped path then fails ("push_in error !!", -19 / -26,
# with enc_exit already set).  The race is the sample's, on silicon as
# here; the build joins the feed thread before the encoder stops.
#
# ai_net classifies ai_pattern's picture with an ImageNet network.  The
# SDK v1.01.002's ai_pattern/para/nvt_model.bin is built for the NT98520
# (model chip id 1), which kflow_ai_net refuses on this chip; v1.01.007
# ships the same network built for it (chip id 4).  --ai-model gives
# that one; a model with another chip id is left out, and ai_net with it.
#
# With --sdk7 (the v1.01.007 uClibc release, the directory above code/)
# and --turnkey (its AI turnkey package's sd/, which holds CNNLib) it also
# builds the turnkey's detectors -- ai2_pdcnn_sample (people),
# alg_pvdcnn_sample (people and vehicles), alg_fdcnn_ai2_sample (faces)
# -- from v1.01.007's sources and prebuilt detector libraries, but against
# v1.01.002's vendor_ai2: v1.01.007's checks for a kernel driver version
# the release modules do not have.  alg_fdcnn_ai2_sample wants a second
# DDR (NN_USE_DRAM2), which these chips do not have; the build turns it
# off.  sd/ then holds what they read: the models, and pictures -- the
# SDK's dashcam frame (YUV420_SP_Huang), scikit-image's astronaut and
# OpenCV's messi5 / basketball1 test pictures (downloaded), and a
# composite of the last two -- as the samples' NV12 files and BMPs.
#
#   bash qemu-boot/build-novatek-hdal-samples.sh --sdk <na51089 SDK> \
#       --rootfs qemu-boot/rootfs.squashfs.nt98566 [--out DIR] \
#       [--ai-model <v1.01.007 SDK>/code/hdal/samples/ai_pattern/para/nvt_model.bin] \
#       [--sdk7 <v1.01.007 SDK> --turnkey <ai_turnkey>/sd] \
#       [--cross /opt/arm-ca9-linux-uclibcgnueabihf-8.4.01/bin/arm-ca9-linux-uclibcgnueabihf-]
set -euo pipefail

SDK=
ROOTFS=
AIMODEL=
SDK7=
TURNKEY=
OUT=$PWD/novatek-hdal-samples
CROSS=/opt/arm-ca9-linux-uclibcgnueabihf-8.4.01/bin/arm-ca9-linux-uclibcgnueabihf-
while [ $# -gt 0 ]; do
    case "$1" in
        --sdk) SDK=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --ai-model) AIMODEL=$2; shift 2 ;;
        --sdk7) SDK7=$2; shift 2 ;;
        --turnkey) TURNKEY=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --cross) CROSS=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ ! -d "$SDK/code/hdal/samples" ]; then
    echo "need --sdk: the NA51089 Linux SDK (code/hdal/samples in it)" >&2
    exit 2
fi
if [ ! -f "$ROOTFS" ]; then
    echo "need --rootfs: the OpenIPC release's rootfs.squashfs.<soc>" >&2
    exit 2
fi
if [ ! -x "${CROSS}gcc" ]; then
    echo "need the SDK's uClibc toolchain (--cross PREFIX, now $CROSS)" >&2
    exit 2
fi

CODE=$SDK/code
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$OUT/smp" "$OUT/uc"

# The runtime: uClibc from the toolchain, libhdal from the release.
SYSROOT=$("${CROSS}gcc" -print-sysroot)
for lib in lib/ld-uClibc.so.0 lib/libc.so.0 lib/libgcc_s.so.1 \
           usr/lib/libstdc++.so.6; do
    cp -L "$SYSROOT/$lib" "$OUT/uc/"
done
unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" usr/lib/libhdal.so >/dev/null
cp "$WORK/rootfs/usr/lib/libhdal.so" "$OUT/uc/"
"${CROSS}strip" "$OUT/uc/"* 2>/dev/null || true

build() {           # build <sample> [sensor]: sensor samples get SC501AI
    local smp=$1 sensor=${2:-} src=$WORK/$1
    mkdir -p "$src"
    cp "$CODE/hdal/samples/$smp/"*.c "$src/"
    if [ -n "$sensor" ]; then
        sed -i -E 's/nvt_sen_imx290/nvt_sen_sc501ai/;
            s/^(#define VDO_SIZE_W[[:space:]]+)1920/\12880/;
            s/^(#define VDO_SIZE_H[[:space:]]+)1080/\11620/' "$src/"*.c
    fi
    if [ "$smp" = hd_video_encode_only ]; then
        # the feed thread first, then the encoder's stop (see the top)
        perl -0pi -e 's{(\n\s*// stop video_record modules \(main\)\n)}{\n\n\t// destroy feed yuv thread, before the stop: it may be pushing\n\tpthread_join(stream[0].yuv_thread_id, NULL);\n$1};
            s{\n\s*// destroy feed yuv thread\n\s*pthread_join\(stream\[0\]\.yuv_thread_id, NULL\);\n}{\n}' \
            "$src/"*.c
        if [ "$(cat "$src/"*.c | grep -c 'pthread_join(stream\[0\].yuv_thread_id')" != 1 ] ||
                ! cat "$src/"*.c | grep -A4 'pthread_join(stream\[0\].yuv_thread_id' |
                grep -c 'hd_videoenc_stop' >/dev/null; then
            echo "hd_video_encode_only: its shutdown is not the SDK's as known" >&2
            exit 1
        fi
    fi
    "${CROSS}gcc" -O2 -D__LINUX -I"$CODE/hdal/include" -I"$CODE/vos/include" \
        -I"$CODE/hdal/vendor/media/include" -o "$OUT/smp/$smp" "$src/"*.c \
        -L"$OUT/uc" -lhdal -lpthread \
        -Wl,--dynamic-linker=/tmp/uc/ld-uClibc.so.0 -Wl,-rpath=/tmp/uc
    echo "built $smp"
}
build hd_video_record sensor
build hd_video_snapshot sensor
build hd_video_capture_only sensor
build hd_video_encode_only
build hd_gfx_only
build hd_video_output_only
build hd_video_liveview sensor
build hd_audio_capture_only
build hd_audio_output_only
build hd_audio_livesound
build hd_audio_record

# The AI samples link the SDK's vendor AI libraries (libvendor_ai2 and its
# open part source_pub, built here, with libvendor_media, libprebuilt_ai
# and libvos) statically; ai_op reads its NV12 test picture from
# jpg/ beside it.
H=$CODE/hdal
AIINC="-I$H/include -I$H/source -I$H/drivers/k_flow/include \
    -I$H/drivers/k_driver/include -I$H/drivers/k_driver/source/include \
    -I$H/vendor/include -I$H/vendor/ai2/include -I$H/vendor/ai2/source_pub/include \
    -I$H/vendor/ai2/source_pub/vendor_ai_cpu -I$H/vendor/ai2/source_pub/vendor_ai_dsp \
    -I$H/vendor/ai2/source/prebuilt/include -I$H/vendor/ai/drivers/k_driver/include \
    -I$H/vendor/ai2/drivers/k_flow/include \
    -I$H/vendor/ai2/drivers/k_flow/source/kflow_ai_net -I$H/vendor/cv/include \
    -I$H/vendor/isp/include -I$H/vendor/dsp/include -I$H/vendor/ai2/source/include \
    -I$H/vendor/ai2/source/vendor_ai_net -I$H/vendor/media/include \
    -I$CODE/vos/drivers/include -I$CODE/vos/include"
mkdir -p "$WORK/aipub"
(
    cd "$H/vendor/ai2/source_pub"
    for f in vendor_ai_util.c vendor_ai_cpu/*.c vendor_ai_cpu/*.cpp vendor_ai_dsp/*.c; do
        "${CROSS}gcc" -c $AIINC -I. -D ARM_ASM -D__LINUX -O2 -fPIC -mfpu=neon-fp16 \
            -Wno-aggressive-loop-optimizations \
            -o "$WORK/aipub/$(echo "$f" | tr / _).o" "$f"
    done
)
"${CROSS}ar" rcs "$WORK/libvendor_ai2_pub.a" "$WORK/aipub/"*.o
build_ai() {        # build_ai <sample>
    "${CROSS}gcc" -O2 -D__LINUX $AIINC -o "$OUT/smp/$1" "$H/samples/$1/"*.c \
        -L"$OUT/uc" -lhdal -Wl,--start-group "$H/vendor/ai2/source/libvendor_ai2.a" \
        "$WORK/libvendor_ai2_pub.a" "$H/vendor/media/source/libvendor_media.a" \
        "$H/vendor/ai/source/prebuilt/lib/uclibc/libprebuilt_ai.a" \
        "$CODE/vos/output/libvos.a" -Wl,--end-group -lpthread -lm -lstdc++ \
        -Wl,--dynamic-linker=/tmp/uc/ld-uClibc.so.0 -Wl,-rpath=/tmp/uc
    echo "built $1"
}
build_ai ai_op
build_ai ai_net
mkdir -p "$OUT/smp/jpg" "$OUT/smp/para" "$OUT/smp/accuracy"
cp "$H/samples/ai_pattern/jpg/YUV420_SP_W512H376.bin" "$OUT/smp/jpg/"
cp "$H/samples/ai_pattern/accuracy/labels.txt" "$OUT/smp/accuracy/"
# the model's chip id: its first 16 bits (4 is this chip, 1 the NT98520)
if [ -z "$AIMODEL" ] && [ -n "$SDK7" ]; then
    AIMODEL=$SDK7/code/hdal/samples/ai_pattern/para/nvt_model.bin
fi
AIMODEL=${AIMODEL:-$H/samples/ai_pattern/para/nvt_model.bin}
if [ "$(od -An -tu2 -N2 "$AIMODEL" | tr -d ' ')" = 4 ]; then
    cp "$AIMODEL" "$OUT/smp/para/nvt_model.bin"
else
    rm -f "$OUT/smp/para/nvt_model.bin"
    echo "$AIMODEL is not built for this chip (chip id 4): no ai_net model" \
         "(--ai-model, from SDK v1.01.007)" >&2
fi

# The turnkey's detectors and the pictures they read
if [ -n "$SDK7" ] && [ -n "$TURNKEY" ]; then
    H7=$SDK7/code/hdal
    T7=$H7/vendor/third_party
    DETINC="$AIINC -I$H/vendor/ai2/source/prebuilt/include/nvtnn \
        -I$T7/pdcnn_ai2/include -I$T7/pvdcnn/include -I$T7/fdcnn_ai2/include \
        -I$T7/limit_fdet/include -I$T7/motion_detection/include \
        -I$H7/vendor/cv/include -I$H7/vendor/cv/include/libmd"
    for smp in ai2_pdcnn_sample alg_pvdcnn_sample alg_fdcnn_ai2_sample; do
        src=$WORK/$smp
        mkdir -p "$src"
        cp "$H7/samples/$smp/"*.c "$src/"
        sed -i 's/^#define NN_USE_DRAM2 .*ENABLE/#define NN_USE_DRAM2 DISABLE/' "$src/"*.c
        "${CROSS}gcc" -O2 -D__LINUX $DETINC -o "$OUT/smp/$smp" "$src/"*.c \
            -L"$OUT/uc" -lhdal -Wl,--start-group "$H/vendor/ai2/source/libvendor_ai2.a" \
            "$WORK/libvendor_ai2_pub.a" "$H/vendor/media/source/libvendor_media.a" \
            "$H/vendor/ai/source/prebuilt/lib/uclibc/libprebuilt_ai.a" \
            "$T7/pdcnn_ai2/prebuilt/uclibc/libprebuilt_pdcnn_ai2.a" \
            "$T7/pvdcnn/prebuilt/uclibc/libprebuilt_pvdcnn.a" \
            "$T7/fdcnn_ai2/prebuilt/uclibc/libprebuilt_fdcnn_ai2.a" \
            "$T7/limit_fdet/prebuilt/uclibc/libprebuilt_limit_fdet.a" \
            "$T7/motion_detection/prebuilt/uclibc/libprebuilt_md.a" \
            "$H7/vendor/cv/source/libvendor_cv.a" \
            "$H7/vendor/cv/source/prebuilt/uclibc/libprebuilt_cv.a" \
            "$H/vendor/isp/source/libvendor_isp.a" \
            "$CODE/vos/output/libvos.a" -Wl,--end-group -lpthread -lm -lstdc++ \
            -Wl,--dynamic-linker=/tmp/uc/ld-uClibc.so.0 -Wl,-rpath=/tmp/uc
        echo "built $smp"
    done
    SD=$OUT/sd
    rm -rf "$SD"
    mkdir -p "$SD/CNNLib/para" "$SD/jpg/PD" "$SD/jpg/PVD" "$SD/FDCNN/faces_y" \
        "$SD/FDCNN/faces_uv" "$WORK/img"
    cp -a "$TURNKEY/CNNLib/para/pdcnn" "$TURNKEY/CNNLib/para/public" \
        "$TURNKEY/CNNLib/para/pvdcnn" "$TURNKEY/CNNLib/para/fdcnn_method2_light" \
        "$SD/CNNLib/para/"
    rm -f "$SD/CNNLib/para/pvdcnn/nvt_model_origin.bin"     # prune37 is used
    cp "$H7/samples/ai_pattern/jpg/YUV420_SP_Huang_W1920H1080.bin" "$SD/jpg/PVD/huang.yuv"
    curl -fsSL --retry 3 -o "$WORK/img/astronaut.png" \
        https://raw.githubusercontent.com/scikit-image/scikit-image/v0.19.3/skimage/data/astronaut.png
    for f in messi5.jpg basketball1.png; do
        curl -fsSL --retry 3 -o "$WORK/img/$f" \
            https://raw.githubusercontent.com/opencv/opencv/4.10.0/samples/data/$f
    done
    python3 - "$WORK/img" "$SD" <<'PY'
# The pictures: a composite of messi5 and basketball1 at two sizes, and
# basketball1 alone, as 1920x1080 NV12 (people, vehicles); astronaut and
# basketball1 as Y / UV BMPs (faces).  BT.601 full range.
import sys
import numpy as np
from PIL import Image
img, sd = sys.argv[1:]

def planes(im):
    im = im.convert('RGB')
    w, h = im.size[0] // 4 * 4, im.size[1] // 2 * 2
    a = np.asarray(im.crop((0, 0, w, h))).astype(float)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = -0.168736 * r - 0.331264 * g + 0.5 * b + 128
    v = 0.5 * r - 0.418688 * g - 0.081312 * b + 128
    uv = np.stack([u, v], -1).reshape(h // 2, 2, w // 2, 2, 2).mean(axis=(1, 3))
    return (y.clip(0, 255).round().astype(np.uint8),
            uv.reshape(h // 2, w).clip(0, 255).round().astype(np.uint8))

def nv12(im, path):
    y, uv = planes(im)
    with open(path, 'wb') as f:
        f.write(y.tobytes() + uv.tobytes())

def fit(im, w=1920, h=1080):
    s = min(w / im.size[0], h / im.size[1])
    im = im.convert('RGB').resize((int(im.size[0] * s) // 2 * 2,
                                   int(im.size[1] * s) // 2 * 2), Image.BILINEAR)
    can = Image.new('RGB', (w, h), (128, 128, 128))
    can.paste(im, ((w - im.size[0]) // 4 * 2, (h - im.size[1]) // 4 * 2))
    return can

messi = Image.open(img + '/messi5.jpg').convert('RGB')
bball = Image.open(img + '/basketball1.png').convert('RGB')
comp = Image.new('RGB', (1920, 1080), (110, 120, 110))
for im, hh, x, y in ((messi, 420, 40, 80), (bball, 420, 760, 80),
                     (messi, 240, 300, 640), (bball, 240, 1300, 640)):
    comp.paste(im.resize((int(im.size[0] * hh / im.size[1]), hh),
                         Image.BILINEAR), (x, y))
for d in ('PD', 'PVD'):
    nv12(comp, '%s/jpg/%s/comp.yuv' % (sd, d))
nv12(fit(bball), sd + '/jpg/PD/bball.yuv')
for i, f in enumerate(('astronaut.png', 'basketball1.png')):
    y, uv = planes(Image.open(img + '/' + f))
    Image.fromarray(y, 'L').save('%s/FDCNN/faces_y/%06d.bmp' % (sd, i))
    Image.fromarray(uv, 'L').save('%s/FDCNN/faces_uv/%06d.bmp' % (sd, i))
PY
    printf 'comp.yuv 1920 1080\nbball.yuv 1920 1080\n' > "$SD/jpg/pd_image_list.txt"
    printf 'huang.yuv 1920 1080\ncomp.yuv 1920 1080\n' > "$SD/jpg/pvdlist.txt"
fi

find "$OUT/smp" -maxdepth 1 -type f -exec "${CROSS}strip" {} +
echo "Samples and their runtime in $OUT"
