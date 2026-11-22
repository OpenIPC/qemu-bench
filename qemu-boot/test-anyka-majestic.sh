#!/bin/bash
# Majestic on the Anyka twins, from the nightly as released.
#
#   --soc ev200  OpenIPC/firmware's openipc.ak3918ev200-nor-lite.tgz on the
#                ak3918ev200 machine (SC1345, the 8290 encoder: ak-cam and
#                ak-venc), with the firmware release's U-Boot
#   --soc ev300  OpenIPC/builder's ak3918ev300_lite_heimvision-pe3020w
#                (the HeimVision PE3020-W of OpenIPC/builder#234: JX-F37,
#                ak_camera, the Allegro core through uio_vcodec and the
#                SDK's encoder library) on the ak3918ev300 machine
#
# The image is the release's: its U-Boot, kernel and squashfs, written to
# an 8 MiB NOR at OpenIPC's offsets (U-Boot at 0, the kernel at 0x50000, the
# root at 0x250000) with rootfs_data erased, so U-Boot boots it with its own
# environment as on a camera. Four changes to the root, and only these:
# root's password is 12345 (an unclaimed camera streams nothing, and is
# claimed here as the Ingenic twin's test claims it), majestic.yaml turns
# the 640x360 second stream on and logs at debug level, S31codec sets the
# main stream's codec (--codec) after the first-boot customizer, which
# sets its own (the PE3020-W's: H.264), and rc.local mirrors
# the syslog, where majestic logs, to the console the checks read.
#
# What is checked:
#   - majestic reaches the first frame ("anyka: first frame") and never
#     logs "Cannot start SDK" or a stuck encoder ("no encoder interrupt");
#   - the main stream is in the codec asked for, by ffprobe;
#   - 50 pictures of each RTSP stream decode, by ffmpeg's own count, with
#     ffmpeg saying nothing at error level (bar the RTP timestamps);
#   - /image.jpg is a JPEG of the main stream's size.
#
#   test-anyka-majestic.sh --soc ev200|ev300 [--firmware TGZ] [--uboot BIN]
#                          [--kernel UIMAGE] [--majestic BIN] [--work DIR]
#                          [--qemu Q] [--timeout SECONDS] [--keep]
#                          [--codec h264|h265] [--sensor NAME]
#                          [--sensor-reset N|none]
#   --firmware  a local package in place of the nightly (the .tgz with the
#               uImage and rootfs.squashfs); --kernel and --majestic replace
#               the one file in it, to try a fix before it is released
#   --sensor    another catalogue sensor in place of the machine's own
#               (sensor=, e.g. gc1054 or sc2335 on EV200), to check that
#               its driver and tuning bring up the stream
#   --sensor-reset  the GPIO the sensor's reset is wired to (sensor-reset=),
#               for a kernel whose device tree is another board's: the
#               generic ak3918ev300_lite kernel boots the EVB's tree, reset
#               49, where the machine models the PE3020-W's 79, and without
#               it the sensor answers its ID but sends no frame
#   --codec     the main stream's codec (h264 by default; h265 on EV300,
#               whose Allegro core has HEVC, the EV200's 8290 has not)
#   --timeout   seconds for the boot to reach the first frame, login prompt
#               included (one deadline, so a hung boot still leaves the step
#               time to print the console)
# The EV200 nightly comes through fetch-openipc-lite.sh, which takes the
# newer of the plain and the split release, so a chip that moves to the split
# one is not tested on its frozen plain archive.
# Needs ffmpeg, ffprobe, curl, unsquashfs, mksquashfs, openssl and file on
# PATH.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
QEMU=$HERE/../qemu-src/build/qemu-system-arm
SOC=
FW=
UBOOT=
KERNEL=
BIN=
WORK=
TIMEOUT=600
KEEP=
CODEC=h264
SENSOR=
SENSOR_RESET=

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --firmware) FW=$2; shift ;;
    --uboot) UBOOT=$2; shift ;;
    --kernel) KERNEL=$2; shift ;;
    --majestic) BIN=$2; shift ;;
    --work) WORK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --keep) KEEP=1 ;;
    --codec) CODEC=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --sensor-reset)
        # it goes into -M's comma list: a GPIO 0..95 or none, nothing else
        case $2 in
        none|[0-9]|[1-8][0-9]|9[0-5]) SENSOR_RESET=$2 ;;
        *) echo "--sensor-reset: a GPIO number 0..95, or none" >&2; exit 2 ;;
        esac
        shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
case $SOC in
ev200) CHIP=ak3918ev200; REPO=OpenIPC/firmware
       ASSET=openipc.ak3918ev200-nor-lite.tgz ;;
ev300) CHIP=ak3918ev300; REPO=OpenIPC/builder
       ASSET=ak3918ev300_lite_heimvision-pe3020w-nor.tgz ;;
*) echo "--soc ev200 or ev300" >&2; exit 2 ;;
esac
case $CODEC in
h264) PROBED=h264 ;;
h265) PROBED=hevc ;;
*) echo "--codec h264 or h265" >&2; exit 2 ;;
esac
for t in ffmpeg ffprobe curl unsquashfs mksquashfs openssl file; do
    command -v $t >/dev/null || { echo "$t is not on PATH" >&2; exit 2; }
done

WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
fetch() {       # fetch REPO ASSET OUT
    curl -sfL --retry 3 -o "$3" \
        "https://github.com/$1/releases/download/latest/$2"
}
if [ -z "$UBOOT" ]; then
    UBOOT=$WORK/u-boot-$CHIP-nor.bin
    fetch OpenIPC/firmware "u-boot-$CHIP-nor.bin" "$UBOOT" ||
        { echo "no u-boot-$CHIP-nor.bin on OpenIPC/firmware" >&2; exit 1; }
fi
rm -rf "$WORK/pkg"
mkdir -p "$WORK/pkg"
if [ -n "$FW" ]; then
    NAME=$(basename "$FW")
    tar xzf "$FW" -C "$WORK/pkg"
elif [ "$SOC" = ev200 ]; then
    NAME="$REPO's latest $CHIP lite NOR release"
    bash "$HERE/fetch-openipc-lite.sh" "$CHIP" "$WORK/pkg" >/dev/null ||
        { echo "no $NAME" >&2; exit 1; }
else
    NAME=$ASSET
    FW=$WORK/$ASSET
    fetch "$REPO" "$ASSET" "$FW" ||
        { echo "no $ASSET on $REPO's latest release" >&2; exit 1; }
    tar xzf "$FW" -C "$WORK/pkg"
fi
KERNEL=${KERNEL:-$WORK/pkg/uImage.$CHIP}
ROOT=$WORK/pkg/rootfs.squashfs.$CHIP
[ -f "$KERNEL" ] && [ -f "$ROOT" ] ||
    { echo "$NAME has no uImage.$CHIP / rootfs.squashfs.$CHIP" >&2; exit 1; }

RFS=$WORK/rfs
rm -rf "$RFS"
unsquashfs -q -d "$RFS" "$ROOT" >/dev/null
if [ -n "$BIN" ]; then
    cp "$BIN" "$RFS/usr/bin/majestic"
    chmod 755 "$RFS/usr/bin/majestic"
fi
[ -x "$RFS/usr/bin/majestic" ] ||
    { echo "FAIL: $NAME has no /usr/bin/majestic"; exit 1; }
cat > "$RFS/etc/majestic.yaml" <<'EOF'
system:
  logLevel: debug
video1:
  enabled: true
  size: 640x360
jpeg:
  enabled: true
EOF
# the codec asked for, over the customizer's (S30) and before majestic (S95)
printf '#!/bin/sh\n[ "$1" = start ] && cli -s .video0.codec %s\nexit 0\n' \
    "$CODEC" > "$RFS/etc/init.d/S31codec"
chmod 755 "$RFS/etc/init.d/S31codec"
# claimed: an unclaimed camera streams nothing
HASH=$(openssl passwd -1 12345)
[ -n "$HASH" ] || { echo "openssl passwd gave no hash" >&2; exit 1; }
sed -i "s|^root:[^:]*:|root:$HASH:|" "$RFS/etc/shadow"
grep -qF "root:$HASH:" "$RFS/etc/shadow" ||
    { echo "/etc/shadow: root's password not set" >&2; exit 1; }
# majestic logs to syslog; the checks read the console
sed -i 's|^exit 0$|logread -f > /dev/console 2>\&1 \&\nexit 0|' "$RFS/etc/rc.local"
grep -q '^logread -f' "$RFS/etc/rc.local" ||
    { echo "rc.local: no exit 0 to insert before" >&2; exit 1; }
mksquashfs "$RFS" "$WORK/root.sq" -comp xz -noappend -quiet -all-root >/dev/null
[ -n "$KEEP" ] || rm -rf "$RFS"

python3 - "$WORK/nor.bin" "$UBOOT" "$KERNEL" "$WORK/root.sq" <<'PY'
import sys
nor, uboot, kernel, root = sys.argv[1:]
img = bytearray(b'\xff' * (8 << 20))
for path, off, room in ((uboot, 0, 0x40000), (kernel, 0x50000, 0x200000),
                        (root, 0x250000, 0x500000)):
    data = open(path, 'rb').read()
    assert len(data) <= room, '%s: %d bytes, the slot has %d' % (path, len(data), room)
    img[off:off + len(data)] = data
open(nor, 'wb').write(img)
PY

read -r RTSP HTTP <<< "$(python3 -c '
import socket
s = [socket.socket() for _ in range(2)]
for x in s: x.bind(("127.0.0.1", 0))
print(*(x.getsockname()[1] for x in s))')"
CON=$WORK/console.log
: > "$CON"
# not restrict=on: on the Anyka machines it keeps the forwarded ports'
# replies from coming back
"$QEMU" -M "$CHIP,flash-file=$WORK/nor.bin${SENSOR:+,sensor=$SENSOR}${SENSOR_RESET:+,sensor-reset=$SENSOR_RESET}" -display none \
    -serial file:"$CON" -monitor none \
    -nic user,hostfwd=tcp:127.0.0.1:$RTSP-:554,hostfwd=tcp:127.0.0.1:$HTTP-:80 \
    </dev/null 2>"$WORK/qemu.err" &
QPID=$!
trap 'kill $QPID 2>/dev/null' EXIT
DEADLINE=$((SECONDS + TIMEOUT))

# waitfor PATTERN: until the boot's one deadline
waitfor() {
    while [ $SECONDS -lt $DEADLINE ]; do
        grep -aq -E "$1" "$CON" 2>/dev/null && return 0
        kill -0 $QPID 2>/dev/null || return 1
        sleep 1
    done
    return 1
}
fail=0
check() {       # check DESCRIPTION COMMAND...
    local what=$1
    shift
    if "$@"; then echo "  PASS: $what"; else echo "  FAIL: $what"; fail=1; fi
}
URL=rtsp://root:12345@127.0.0.1:$RTSP
decodes() {
    local out rc=0 n err
    out=$(timeout -k 10 300 ffmpeg -nostdin -v error -progress pipe:1 \
        -rtsp_transport tcp -allowed_media_types video -i "$URL/$1" \
        -frames:v 50 -f null - \
        2>"$WORK/ffmpeg.err") || rc=$?
    n=$(printf '%s\n' "$out" | sed -n 's/^frame=//p' | tail -1)
    # ffmpeg's "Last message repeated" names no error of its own: a real
    # one that repeats has its own line above it, which is not dropped
    err=$(grep -v -e 'non monotonically increasing dts' \
        -e 'Last message repeated' "$WORK/ffmpeg.err" || true)
    [ -z "$err" ] || echo "$err" | head -3
    if [ "$rc" != 0 ] || [ -n "$err" ] || [ "${n:-0}" -lt 50 ]; then
        echo "    ($1: ffmpeg exit $rc, ${n:-0} pictures)"
        return 1
    fi
}
main_codec() {
    local got
    got=$(timeout -k 10 120 ffprobe -v error -rtsp_transport tcp -select_streams v \
        -show_entries stream=codec_name -of csv=p=0 "$URL/stream=0")
    [ "$got" = "$PROBED" ] ||
        { echo "    (main stream: '${got:-nothing}', not $PROBED)"; return 1; }
}
snapshot() {
    local want got
    want=$(timeout -k 10 120 ffprobe -v error -rtsp_transport tcp -select_streams v \
        -show_entries stream=width,height -of csv=s=x:p=0 "$URL/stream=0")
    curl -s -m 120 -u root:12345 -o "$WORK/snap.jpg" \
        "http://127.0.0.1:$HTTP/image.jpg" || return 1
    got=$(file "$WORK/snap.jpg" | grep -o 'JPEG image data.*' |
        grep -oE '[0-9]+x[0-9]+' | tail -1)     # after any JFIF density
    [ -n "$got" ] && [ "$got" = "$want" ] ||
        { echo "    (/image.jpg: '${got:-not a JPEG}', main stream '$want')"; return 1; }
}
no_sdk_error() {
    ! grep -aE 'Cannot start SDK|no encoder interrupt' "$CON"
}

echo "$SOC: $NAME"
check 'boots to the login prompt' waitfor 'login:'
[ $fail = 0 ] && check 'majestic gets the first frame' waitfor 'anyka: first frame'
check 'majestic starts its SDK and the encoder answers' no_sdk_error
if [ $fail = 0 ]; then
    check "main stream is $CODEC" main_codec
    check 'main stream decodes (50 pictures)' decodes stream=0
    check 'second stream decodes (50 pictures)' decodes stream=1
    check '/image.jpg is a JPEG of the main stream' snapshot
    check 'still no SDK or encoder error' no_sdk_error
fi
if [ $fail != 0 ]; then
    echo "--- console (last 60 lines)"
    tail -n 60 "$CON"
fi
[ -n "$KEEP" ] || [ $fail != 0 ] || rm -rf "$WORK/pkg" "$WORK/root.sq" "$WORK/nor.bin"
exit $fail
