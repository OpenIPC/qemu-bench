#!/bin/bash
# Build the vendor SDK's HDAL sample programs for qemu-boot/test-novatek-hdal.sh.
#
# The samples (code/hdal/samples in na51089_linux_sdk_*_v1.01.002) drive
# the media engines through the HDAL: capture -> video process -> encode
# (hd_video_record), JPEG snapshots, capture alone, encoding frames from
# a file, the 2D graphics engine, the display (a file, and the sensor
# live), audio capture, playback, live loopback and AAC recording, and the
# AI engines' operators (ai_op).  They are vendor code, as
# is the SDK, so neither is in the repository or in CI.
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
#   bash qemu-boot/build-novatek-hdal-samples.sh --sdk <na51089 SDK> \
#       --rootfs qemu-boot/rootfs.squashfs.nt98566 [--out DIR] \
#       [--cross /opt/arm-ca9-linux-uclibcgnueabihf-8.4.01/bin/arm-ca9-linux-uclibcgnueabihf-]
set -euo pipefail

SDK=
ROOTFS=
OUT=$PWD/novatek-hdal-samples
CROSS=/opt/arm-ca9-linux-uclibcgnueabihf-8.4.01/bin/arm-ca9-linux-uclibcgnueabihf-
while [ $# -gt 0 ]; do
    case "$1" in
        --sdk) SDK=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
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
        if [ "$(grep -c 'pthread_join(stream\[0\].yuv_thread_id' "$src/"*.c)" != 1 ] ||
                ! grep -A4 'pthread_join(stream\[0\].yuv_thread_id' "$src/"*.c |
                grep -q 'hd_videoenc_stop'; then
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
mkdir -p "$OUT/smp/jpg"
cp "$H/samples/ai_pattern/jpg/YUV420_SP_W512H376.bin" "$OUT/smp/jpg/"
find "$OUT/smp" -maxdepth 1 -type f -exec "${CROSS}strip" {} +
echo "Samples and their runtime in $OUT"
