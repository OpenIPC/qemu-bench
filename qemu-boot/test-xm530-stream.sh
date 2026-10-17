#!/bin/bash
#
# majestic's H.264 on the emulated XM530, end to end.
#
# Boots the release kernel and rootfs from a synthetic NOR the way the flash
# tests do (mem= first, as OpenIPC's U-Boot passes it), leaves majestic to
# start on its own, and has the H1 model keep every job it encodes
# (-global xm-h1.dump-dir).  No login, no claim: RTSP and HTTP stay 401 on an
# unclaimed camera, so the stream is judged from the encoder's side.  The
# host then checks:
#
#   1. majestic encoded at least --frames H.264 pictures
#   2. the jobs concatenate into a stream ffmpeg decodes with no errors,
#      frame for frame, at the SPS's size
#   3. it holds more than one IDR, so the SPS/PPS majestic rewrites per GOP
#      are carried and followed
#
# Usage:
#   test-xm530-stream.sh --qemu ./qemu-src/build/qemu-system-arm \
#       --output-dir <dir> [--machine xm530] [--kernel <uImage>] \
#       [--rootfs <squashfs>] [--frames 60] [--timeout 240]
#
set -u

MACHINE=xm530
QEMU="./qemu-src/build/qemu-system-arm"
OUTDIR="."
KERNEL="qemu-boot/uImage.xm530"
ROOTFS="qemu-boot/rootfs.squashfs.xm530"
FRAMES=60
TIMEOUT=240

while [ $# -gt 0 ]; do
    case "$1" in
        --machine)    MACHINE="$2"; shift 2 ;;
        --qemu)       QEMU="$2"; shift 2 ;;
        --output-dir) OUTDIR="$2"; shift 2 ;;
        --kernel)     KERNEL="$2"; shift 2 ;;
        --rootfs)     ROOTFS="$2"; shift 2 ;;
        --frames)     FRAMES="$2"; shift 2 ;;
        --timeout)    TIMEOUT="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

command -v ffmpeg >/dev/null || { echo "=== FAIL: needs ffmpeg ==="; exit 1; }
for f in "$KERNEL" "$ROOTFS"; do
    [ -f "$f" ] || { echo "=== FAIL: missing $f ==="; exit 1; }
done
mkdir -p "$OUTDIR"
DUMP="$OUTDIR/h1-jobs"
rm -rf "$DUMP"; mkdir -p "$DUMP"

BOOT_KB=256; ENV_KB=64; KERN_KB=2048
ROOTFS_KB=$(( ( ($(stat -c %s "$ROOTFS") + 65535) / 65536 ) * 64 ))
FLASH="$OUTDIR/flash.bin"
head -c $((8 * 1024 * 1024)) /dev/zero | tr '\000' '\377' > "$FLASH"
dd if="$ROOTFS" of="$FLASH" bs=1k seek=$((BOOT_KB + ENV_KB + KERN_KB)) conv=notrunc 2>/dev/null
MTDPARTS="xm_sfc:${BOOT_KB}k(boot),${ENV_KB}k(env),${KERN_KB}k(kernel),${ROOTFS_KB}k(rootfs),-(rootfs_data)"
APPEND="mem=32M console=ttyAMA0,115200 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=${MTDPARTS}"

CONSOLE="$OUTDIR/console.txt"; rm -f "$CONSOLE"
echo "=== booting $MACHINE, waiting for $FRAMES H.264 jobs (<= ${TIMEOUT}s) ==="
"$QEMU" -M "$MACHINE",flash-file="$FLASH" -smp 2 -m 64M -kernel "$KERNEL" \
    -global xm-h1.dump-dir="$DUMP" \
    -nographic -serial file:"$CONSOLE" -monitor none \
    -nic user,restrict=on -append "$APPEND" >/dev/null 2>"$OUTDIR/qemu.err" &
pid=$!
t=0
while kill -0 "$pid" 2>/dev/null && [ "$t" -lt "$TIMEOUT" ]; do
    [ "$(find "$DUMP" -name '*.h264' | wc -l)" -ge "$FRAMES" ] && break
    sleep 1; t=$((t + 1))
done
kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null

fail=0
jobs=$(find "$DUMP" -name '*.h264' | wc -l)
if [ "$jobs" -ge "$FRAMES" ]; then
    echo "ok   majestic encoded $jobs H.264 jobs in ${t}s"
else
    echo "FAIL: $jobs H.264 jobs in ${t}s, wanted $FRAMES"; fail=1
fi

STREAM="$OUTDIR/stream.h264"
find "$DUMP" -name '*.h264' | sort | xargs -r cat > "$STREAM"
if [ "$jobs" -gt 0 ]; then
    errs=$(ffmpeg -v error -err_detect explode -i "$STREAM" -f null - 2>&1)
    if [ -z "$errs" ]; then
        echo "ok   ffmpeg decodes the stream with no errors"
    else
        echo "FAIL: ffmpeg: $(echo "$errs" | head -3)"; fail=1
    fi
    info=$(ffprobe -v error -count_frames -show_entries \
           stream=nb_read_frames,width,height,profile -of default=nw=1 "$STREAM")
    decoded=$(echo "$info" | sed -n 's/^nb_read_frames=//p')
    if [ "$decoded" = "$jobs" ]; then
        echo "ok   $decoded frames decoded, one per job ($(echo "$info" |
              grep -E '^(profile|width|height)=' | tr '\n' ' '))"
    else
        echo "FAIL: $decoded frames decoded from $jobs jobs"; fail=1
    fi
    idrs=$(ffprobe -v error -show_entries frame=key_frame -of csv=p=0 "$STREAM" |
           grep -c '^1')
    if [ "$idrs" -ge 2 ]; then
        echo "ok   $idrs IDRs, each with the SPS/PPS majestic wrote for it"
    else
        echo "FAIL: $idrs IDR in the stream, wanted at least 2"; fail=1
    fi
fi

if [ "$fail" = "0" ]; then
    echo "=== PASS: majestic streams H.264 on the emulated $MACHINE ==="
    exit 0
fi
echo "--- console, majestic lines ---"
tr -d '\r' < "$CONSOLE" | grep -aiE 'majestic|venc|error' | tail -30
exit 1
