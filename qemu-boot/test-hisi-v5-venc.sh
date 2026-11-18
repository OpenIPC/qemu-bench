#!/bin/bash
# Encode user frames through the HiSilicon V5 VENC on hi3516cv610 (or a
# V5 sibling) and check the hisi-vedu model's streams on the host --
# the V5 counterpart of test-hisi-venc.sh.
#
# The guest is the whole OpenIPC hi3516cv6xx rootfs as an initramfs,
# with hisi-v5-venc-userframe added and an /init that runs the release's
# own load_hisilicon (every open_*.ko as on the camera) before the
# program.  The program sends test-pattern frames with
# ss_mpi_venc_send_frame and prints RESULT: PASS once every frame of
# every channel came back; the model's dump-dir then holds each
# channel's stream, which check-hisi-venc.py decodes and checks.
#
#   test-hisi-v5-venc.sh --sdk DIR [--soc hi3516cv610] [--qemu Q]
#                        [--codecs "h264 h265 jpeg"] [--frames N]
#                        [--work DIR] [--trace]
#
# DIR is the Hi3516CV610 SDK (Hi3516CV610_SDK_V1.0.2.0, with
# smp/a7_linux/source/out unpacked): the program links its headers and
# static MPI libraries, which are not redistributable, so this runs
# locally, not in CI.  Needs unsquashfs, cpio, ffmpeg and
# arm-openipc-linux-musleabi-gcc (or CC).
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SOC=hi3516cv610
QEMU=$HERE/../qemu-src/build/qemu-system-arm
CODECS="h264 h265 jpeg"
FRAMES=6
WORK=
TRACE=0
SDK=
CC=${CC:-arm-openipc-linux-musleabi-gcc}

while [ $# -gt 0 ]; do
    case $1 in
    --sdk) SDK=$2; shift ;;
    --soc) SOC=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --codecs) CODECS=$2; shift ;;
    --frames) FRAMES=$2; shift ;;
    --work) WORK=$2; shift ;;
    --trace) TRACE=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
OUT=$SDK/smp/a7_linux/source/out
if [ ! -d "$OUT/include" ] || [ ! -f "$OUT/lib/libss_mpi.a" ]; then
    echo "--sdk: no Hi3516CV610 SDK out/include and out/lib in '$SDK'" >&2
    exit 2
fi
WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
REL=hi3516cv6xx                 # the release covers cv608 / cv610 / cv613

# --- release and guest program -------------------------------------
if [ ! -f "$WORK/uImage.$REL" ]; then
    bash "$HERE/fetch-openipc-lite.sh" "$REL" "$WORK"
fi
"$CC" -O2 -Wall -static -mcpu=cortex-a7 -mfloat-abi=softfp -mfpu=neon-vfpv4 \
    -o "$WORK/hisi-v5-venc-userframe" "$HERE/hisi-v5-venc-userframe.c" \
    -I "$OUT/include" -L "$OUT/lib" -Wl,--start-group -lss_mpi \
    -lss_mpi_sysmem -lss_mpi_sysbind -lss_mpi_syskol -lot_osal -lsecurec \
    -lpthread -lm -Wl,--end-group

# --- initramfs: the release's rootfs, the program, an /init ----------
INITRD=$WORK/initrd
rm -rf "$INITRD"
unsquashfs -q -d "$INITRD" "$WORK/rootfs.squashfs.$REL" >/dev/null
cp "$WORK/hisi-v5-venc-userframe" "$INITRD/usr/bin/"
cat > "$INITRD/init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t devtmpfs dev /dev
mount -t tmpfs tmp /tmp
echo "=== load"
load_hisilicon -i -sensor0 sc4336p
echo "=== venc"
hisi-v5-venc-userframe -n $FRAMES -o /tmp $CODECS
echo "=== done"
echo b > /proc/sysrq-trigger
EOF
chmod +x "$INITRD/init"
(cd "$INITRD" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/venc.cpio"

# --- run --------------------------------------------------------------
# The initramfs is the whole rootfs: give the OS 80 of the 128 MiB, not
# the camera's 32 (the encoder at test sizes needs far less MMZ than 96M)
MEM="mem=80M mmz_allocator=hisi mmz=anonymous,0,0x45000000,48M"
DUMP=$WORK/dump
rm -rf "$DUMP"
mkdir -p "$DUMP"
EXTRA=()
[ "$TRACE" = 1 ] && EXTRA+=(-global hisi-vedu.trace=on)
timeout 600 "$QEMU" -M "$SOC,sensor=sc4336p" -m 128M \
    -kernel "$WORK/uImage.$REL" -initrd "$WORK/venc.cpio" \
    -append "console=ttyAMA0,115200 rdinit=/init $MEM" \
    -global hisi-vedu.dump-dir="$DUMP" "${EXTRA[@]}" \
    -nographic -serial mon:stdio -nic user,restrict=on -no-reboot \
    > "$WORK/console.log" 2> "$WORK/qemu.log" || true

grep -E '^(h264|h265|jpeg) frame|send_frame|venc_|^RESULT' \
    "$WORK/console.log" || true
if ! grep -q '^RESULT: PASS' "$WORK/console.log"; then
    echo "guest did not pass; console in $WORK/console.log" >&2
    exit 1
fi

# --- host checks ------------------------------------------------------
python3 "$HERE/check-hisi-venc.py" --frames "$FRAMES" "$DUMP" $CODECS
