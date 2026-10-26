#!/bin/bash
# Boot a Grain Media camera's NOR dump from flash -- nsboot, the vendor
# U-Boot, the vendor kernel and the camera's application -- and pull its
# H.264 stream over RTSP, the way a client would.
#
# The whole chain runs on the models: the capture engine paints its test
# pattern, the 3DNR copies it on, the vendor H.264 driver writes the
# headers and the encoder model the slices, and the application's RTSP
# server sends them out over the FTGMAC100.  The checks:
#   - the application's RTSP server comes up;
#   - ffprobe sees an H.264 stream of the main stream's size;
#   - ffmpeg records and decodes it without errors, with frames;
#   - the video graph never "damnit"s (a job the hardware never finished)
#     and the encoder never times out.
#
# The Longse cameras' application gives itself 192.168.1.209 and asks for
# RTSP authentication (admin / admin); the path is /ch0.  The dump is not
# changed: QEMU runs on a copy.
#
#   bash qemu-boot/test-grainmedia-rtsp.sh --flash dump.bin \
#       [--machine gm8136,sensor=ov2715] [--qemu ./qemu-src/build/qemu-system-arm]
#       [--ffmpeg ffmpeg] [--ffprobe ffprobe] [--port 18554] [--timeout 300]
#       [--size 1920x1080] [--url-path /ch0] [--user admin:admin]
set -euo pipefail

FLASH=
MACHINE=gm8136,sensor=ov2715
QEMU=./qemu-src/build/qemu-system-arm
FFMPEG=ffmpeg
FFPROBE=ffprobe
PORT=18554
TIMEOUT=300
SIZE=1920x1080
URL_PATH=/ch0
USER_PW=admin:admin
GUEST_IP=192.168.1.209
while [ $# -gt 0 ]; do
    case "$1" in
        --flash) FLASH=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --ffmpeg) FFMPEG=$2; shift 2 ;;
        --ffprobe) FFPROBE=$2; shift 2 ;;
        --port) PORT=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --size) SIZE=$2; shift 2 ;;
        --url-path) URL_PATH=$2; shift 2 ;;
        --user) USER_PW=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ -z "$FLASH" ]; then
    echo "usage: $0 --flash dump.bin [options]" >&2
    exit 2
fi

QPID=
WORK=$(mktemp -d)
trap '[ -n "$QPID" ] && kill "$QPID" 2>/dev/null; rm -rf "$WORK"' EXIT
cp "$FLASH" "$WORK/flash.bin"
LOG=$WORK/serial.log
URL="rtsp://$USER_PW@127.0.0.1:$PORT$URL_PATH"

timeout "$TIMEOUT" "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" \
    -display none -monitor none -serial "file:$LOG" \
    -nic "user,net=192.168.1.0/24,host=192.168.1.2,hostfwd=tcp:127.0.0.1:$PORT-$GUEST_IP:554" &
QPID=$!

fail=0
pass() { echo "  PASS: $1"; }
bad() { echo "  FAIL: $1"; fail=1; }

for _ in $(seq "$TIMEOUT"); do
    grep -aq 'Rtsp server is running' "$LOG" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
echo "RTSP from $(basename "$FLASH") on $MACHINE:"
if grep -aq 'Rtsp server is running' "$LOG"; then
    pass "the application's RTSP server is up"
else
    bad "the application's RTSP server is up"
fi
sleep 10                                # let the encoder stream settle

probe=$($FFPROBE -v error -rtsp_transport tcp \
        -show_entries stream=codec_name,width,height -of csv "$URL" 2>&1 || true)
if echo "$probe" | grep -q "h264,${SIZE%x*},${SIZE#*x}"; then
    pass "ffprobe: H.264 $SIZE"
else
    bad "ffprobe: H.264 $SIZE ($probe)"
fi

$FFMPEG -v error -rtsp_transport tcp -i "$URL" -an -t 10 -c copy -y \
    "$WORK/stream.h264" >/dev/null 2>&1 || true
errs=$($FFMPEG -v error -i "$WORK/stream.h264" -f null - 2>&1 | head -5 || true)
frames=$($FFPROBE -v error -count_frames -show_entries stream=nb_read_frames \
         -of csv=p=0 "$WORK/stream.h264" 2>/dev/null || echo 0)
if [ -s "$WORK/stream.h264" ] && [ -z "$errs" ] && [ "${frames:-0}" -gt 0 ]; then
    pass "10 s recorded and decoded: $frames frames, no errors"
else
    bad "10 s recorded and decoded (frames ${frames:-0}: $errs)"
fi
if grep -aq 'damnit\|Poll timeout' "$LOG"; then
    bad "no unfinished hardware jobs in the video graph"
else
    pass "no unfinished hardware jobs in the video graph"
fi

if [ $fail -ne 0 ]; then
    echo "--- serial log (tail) ---"
    tail -40 "$LOG"
fi
exit $fail
