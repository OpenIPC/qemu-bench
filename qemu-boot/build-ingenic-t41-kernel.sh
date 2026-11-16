#!/bin/bash
# The T41 SDK's own kernel (4.4.94, isvp_tomcat_defconfig, the one in its
# flash images) with overlayfs, for test-ingenic-majestic.sh --soc t41.
#
# OpenIPC has no T41 release, and the SDK image's kernel has no overlayfs:
# OpenIPC's /init then leaves the root read-only and nothing a camera
# saves (majestic.yaml, the seed, the env) survives, which is not what a
# camera does.  Only CONFIG_OVERLAY_FS is added: struct module stays as the
# SDK's prebuilt modules (tx-isp-t41.ko, avpu.ko, the sensor drivers) were
# built against, so they load into this kernel (enabling KALLSYMS-like
# options would not keep that; OpenIPC/majestic#376).
#
#   build-ingenic-t41-kernel.sh SDK OUT
#     SDK  the extracted Ingenic-SDK-T41-1.2.0-20230713 directory
#     OUT  where uImage.t41 is written
set -eu

SDK=$(cd "$1" && pwd)
OUT=$2
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
B=$OUT/t41-kernel-build
mkdir -p "$B"

if [ ! -d "$B/kernel-4.4.94" ]; then
    # -snld: the tree's dt-bindings symlinks point above their directory
    7z x -y -snld -o"$B" "$SDK/opensource/kernel-4.4.94.7z" >/dev/null
fi
TC=$B/mips-gcc720-uclibc0.9.33.2-r5.1.7
if [ ! -d "$TC" ]; then
    tar -C "$B" -xjf \
        "$SDK/resource/toolchain/gcc_720_r517/mips-gcc720-uclibc0.9.33.2-r5.1.7.tar.bz2"
fi
export PATH=$TC/bin:$PATH
K="make -C $B/kernel-4.4.94 ARCH=mips CROSS_COMPILE=mips-linux-uclibc-"
$K isvp_tomcat_defconfig >"$B/config.log" 2>&1
"$B/kernel-4.4.94/scripts/config" --file "$B/kernel-4.4.94/.config" \
    --enable OVERLAY_FS
$K olddefconfig >>"$B/config.log" 2>&1
# the 2015 dtc defines yylloc twice, which GCC 10's -fno-common refuses
$K HOSTCFLAGS=-fcommon -j"$(nproc)" uImage >"$B/build.log" 2>&1 ||
    { tail -20 "$B/build.log"; exit 1; }
cp "$B/kernel-4.4.94/arch/mips/boot/uImage.lzma" "$OUT/uImage.t41"
echo "$OUT/uImage.t41"
