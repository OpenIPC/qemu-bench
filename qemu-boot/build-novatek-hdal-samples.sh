#!/bin/bash
# Build the vendor SDK's HDAL sample programs for qemu-boot/test-novatek-hdal.sh.
#
# The samples (code/hdal/samples in na51089_linux_sdk_*_v1.01.002) drive
# the media engines through the HDAL: capture -> video process -> encode
# (hd_video_record), JPEG snapshots, capture alone, encoding frames from
# a file, the 2D graphics engine, audio capture, playback, live loopback
# and AAC recording.  They are vendor code, as
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
for lib in ld-uClibc.so.0 libc.so.0 libgcc_s.so.1; do
    cp -L "$SYSROOT/lib/$lib" "$OUT/uc/"
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
build hd_audio_capture_only
build hd_audio_output_only
build hd_audio_livesound
build hd_audio_record
"${CROSS}strip" "$OUT/smp/"*
echo "Samples and their runtime in $OUT"
