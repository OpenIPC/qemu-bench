#!/bin/bash
# Encode user frames through the HiMPP V4 VENC on hi3516ev200 (or a
# V4 sibling) and check the hisi-vedu model's streams on the host.
#
# The guest is an initramfs cut from the OpenIPC release: busybox, the
# release's MPI library and media modules (HiSilicon's libmpi.so and
# open_*.ko, or Goke's libgk_api.so and <soc>_*.ko), and
# hisi-venc-userframe as PID 1's only job.  It loads the encoder half of
# load_hisilicon's chain (no VI, no ISP, no sensor), sends test-pattern
# frames with VENC_SendFrame and prints RESULT: PASS once every frame of
# every channel came back.  The model's dump-dir then holds each
# channel's stream, which check-hisi-venc.py decodes and checks.
#
#   test-hisi-venc.sh [--soc hi3516ev200] [--machine M] [--qemu Q]
#                     [--codecs "h264 h265 jpeg"] [--frames N]
#                     [--work DIR] [--trace]
#
# Needs unsquashfs, cpio and ffmpeg.  The OpenIPC release is fetched
# (fetch-openipc-lite.sh); so are the OpenIPC musl toolchain, unless CC
# names one or arm-openipc-linux-musleabi-gcc is on PATH, and the
# OpenIPC/openhisilicon headers, unless OPENHISILICON points at a checkout.
# The release's libmpi.so is uClibc's; hisi-venc-userframe.c carries what
# it needs to run under musl.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SOC=hi3516ev200
MACHINE=
QEMU=$HERE/../qemu-src/build/qemu-system-arm
CODECS="h264 h265 jpeg"
FRAMES=6
WORK=
TRACE=0
CC=${CC:-arm-openipc-linux-musleabi-gcc}
OPENHISILICON=${OPENHISILICON:-}
TOOLCHAIN_URL=https://github.com/OpenIPC/firmware/releases/download/toolchain/toolchain.hisilicon-hi3516ev200.tgz
TOOLCHAIN_SHA256=4c1074928423ae6b7fd79b4b502a0da7f40cd058ca067cb747c804f820298bb6

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --machine) MACHINE=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --codecs) CODECS=$2; shift ;;
    --frames) FRAMES=$2; shift ;;
    --work) WORK=$2; shift ;;
    --trace) TRACE=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
MACHINE=${MACHINE:-$SOC}
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)

# --- release and guest program -------------------------------------
if [ ! -f "$WORK/uImage.$SOC" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$SOC" "$WORK"
fi
ROOTFS=$WORK/rootfs
if [ ! -d "$ROOTFS" ]; then
    unsquashfs -q -d "$ROOTFS" "$WORK/rootfs.squashfs.$SOC" >/dev/null
fi
if ! command -v "$CC" >/dev/null; then
    TC=$WORK/arm-openipc-linux-musleabi_sdk-buildroot
    if [ ! -x "$TC/bin/arm-openipc-linux-musleabi-gcc" ]; then
        curl -fsSL --retry 3 -o "$WORK/toolchain.tgz" "$TOOLCHAIN_URL"
        echo "$TOOLCHAIN_SHA256  $WORK/toolchain.tgz" | sha256sum -c - >/dev/null
        tar xzf "$WORK/toolchain.tgz" -C "$WORK"
        rm -f "$WORK/toolchain.tgz"
    fi
    CC=$TC/bin/arm-openipc-linux-musleabi-gcc
fi
if [ -z "$OPENHISILICON" ]; then
    OPENHISILICON=$WORK/openhisilicon
    [ -d "$OPENHISILICON/include" ] ||
        git clone -q --depth 1 https://github.com/OpenIPC/openhisilicon \
            "$OPENHISILICON"
fi
# HiSilicon releases: libmpi.so and open_*.ko; Goke: libgk_api.so and
# <soc>_*.ko (open_osal and open_sys_config in both)
KODIR=$(ls -d "$ROOTFS"/lib/modules/*)
if [ -f "$ROOTFS/usr/lib/libmpi.so" ]; then
    LIBS="libmpi.so libsecurec.so"
    LINK="-DHISI_LIBMPI -lmpi -lsecurec"
    MOD=$KODIR/extra/open_
    ALLOC=hisi
else
    LIBS="libgk_api.so"
    LINK="-lgk_api"
    MOD=$KODIR/goke/${SOC}_
    ALLOC=gk
fi
# shellcheck disable=SC2086
"$CC" -O2 -Wall -o "$WORK/hisi-venc-userframe" \
    "$HERE/hisi-venc-userframe.c" -I "$OPENHISILICON/include" \
    -include stdbool.h -include stdint.h \
    -L "$ROOTFS/usr/lib" $LINK -Wl,--allow-shlib-undefined \
    2> >(grep -v 'warning: .*needed by' >&2)

# --- initramfs ------------------------------------------------------
INITRD=$WORK/initrd
rm -rf "$INITRD"
mkdir -p "$INITRD"/{bin,sbin,lib,usr/lib,usr/bin,proc,sys,dev,tmp,etc}
cp -a "$ROOTFS"/bin/busybox "$INITRD/bin/"
for l in sh mount insmod cat echo ls mkdir; do
    ln -sf busybox "$INITRD/bin/$l"
done
cp -a "$ROOTFS"/lib/ld-* "$ROOTFS"/lib/libc.so* "$INITRD/lib/"
for f in "$ROOTFS"/lib/lib*.so*; do cp -a "$f" "$INITRD/lib/"; done
for l in $LIBS; do cp -a "$ROOTFS/usr/lib/$l" "$INITRD/usr/lib/"; done
mkdir -p "$INITRD/ko"
cp "$KODIR/extra/open_sys_config.ko" "$KODIR/extra/open_osal.ko" "$INITRD/ko/"
for m in base sys rgn vgs vpss chnl vedu rc venc h264e h265e jpege; do
    cp "$MOD$m.ko" "$INITRD/ko/$m.ko"
done
cp "$WORK/hisi-venc-userframe" "$INITRD/usr/bin/"

cat > "$INITRD/init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
mount -t tmpfs tmp /tmp
cd /ko
insmod open_sys_config.ko chip=$SOC sensors=sc2315e board=demo
insmod open_osal.ko anony=1 mmz_allocator=$ALLOC mmz=anonymous,0,0x42000000,32M
insmod base.ko
insmod sys.ko
insmod rgn.ko
insmod vgs.ko max_vgs_job=20 max_vgs_node=20 max_vgs_task=20
insmod vpss.ko
insmod chnl.ko
insmod vedu.ko
insmod rc.ko
insmod venc.ko VencMaxChnNum=3
insmod h264e.ko
insmod h265e.ko
insmod jpege.ko
cd /
echo "=== venc"
hisi-venc-userframe -n $FRAMES -o /tmp $CODECS
echo "=== done"
echo b > /proc/sysrq-trigger
EOF
chmod +x "$INITRD/init"
(cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/venc.cpio"

# --- run --------------------------------------------------------------
DUMP=$WORK/dump
rm -rf "$DUMP"
mkdir -p "$DUMP"
EXTRA=()
[ "$TRACE" = 1 ] && EXTRA+=(-global hisi-vedu.trace=on)
timeout 300 "$QEMU" -M "$MACHINE" -kernel "$WORK/uImage.$SOC" \
    -initrd "$WORK/venc.cpio" -append "console=ttyAMA0,115200 rdinit=/init" \
    -global hisi-vedu.dump-dir="$DUMP" "${EXTRA[@]}" \
    -nographic -serial mon:stdio -nic user,restrict=on -no-reboot \
    > "$WORK/console.log" 2> "$WORK/qemu.log" || true

grep -E '^(h264|h265|jpeg) frame|^FAIL|^RESULT' "$WORK/console.log" || true
if ! grep -q '^RESULT: PASS' "$WORK/console.log"; then
    echo "guest did not pass; console in $WORK/console.log" >&2
    exit 1
fi

# --- host checks ------------------------------------------------------
python3 "$HERE/check-hisi-venc.py" --frames "$FRAMES" "$DUMP" $CODECS
