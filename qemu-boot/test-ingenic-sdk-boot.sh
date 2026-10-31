#!/bin/bash
# Boot an Ingenic vendor SDK's prebuilt full flash image (T41, T40, T30:
# resource/image_t*/<GRADE>_gcc*_<libc>_<8|16>M_*.img) the way a camera
# does, from the mask ROM stand-in (flash-file=, no -kernel), and check
# what it did on the way (docs/ingenic.md, "Vendor SDK images"):
#
#   - the SPL and U-Boot banners, U-Boot starting the kernel, the kernel,
#     the root SquashFS mounted, both XBurst2 cores up (T40, T41), the
#     vendor login prompt, no panic / oops, no core OST storm;
#   - the vendor userspace: rcS starts telnetd and app_init.sh gives eth0
#     its static 193.169.4.151, so QEMU's user network is put on that /24
#     and the test logs in over telnet through a hostfwd (TCP both ways
#     through the GMAC; this userspace has no DHCP client) and reads the
#     CPU count, eth0 and the sensor sinfo found;
#   - with a sensor (the default where the machine models one the image
#     has a driver for): start.sh's application (carrier-server) starts,
#     and its RTSP stream (rtsp://<ip>:8554/main, live555) is pulled with
#     ffmpeg through a hostfwd, decoded with -err_detect explode and its
#     first and last pictures checked against the ISP model's scene
#     (ingenic-isp-check.py).  T41's start_param asks for IVDC, the
#     ISP-to-encoder direct path, and gets it; with --no-direct-mode the
#     test rebuilds the root with one line in rcS that writes
#     /tmp/start_param (which start.sh prefers) without direct_mode, so
#     the application takes its frames from memory instead.  In direct
#     mode the IVDC's own counters (/proc/jz/isp/isp-ivdc) must show ISP
#     frames;
#   - a reboot cycle: "reboot" over telnet, the kernel resets through the
#     TCU watchdog, and the SPL, U-Boot and the login prompt come back.
#
# The grade, the DRAM size and the RAM split come from the image: the
# .txt next to it (uboot_model: T41_NQ, driver list) or the file name, and
# the bootargs in U-Boot's default environment.  The SDK images are not
# public: without --image (or with a missing one) the test skips, exit 0.
#
#   bash qemu-boot/test-ingenic-sdk-boot.sh --image T41NQ_..._16M_....img
#       [--soc t41] [--variant t41nq] [--ram 128M] [--sensor gc4663|none]
#       [--qemu PATH] [--ffmpeg CMD] [--timeout 600] [--no-reboot]
#       [--no-rtsp] [--out DIR]
#
# --ffmpeg defaults to ffmpeg on PATH; it must reach the host's 127.0.0.1
# (a docker ffmpeg needs --network host).  Without one the RTSP check is
# skipped, not failed.
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
IMAGE=
SOC=
VARIANT=
RAM=
SENSOR=auto
FFMPEG=
TIMEOUT=600
REBOOT=1
RTSP=1
DIRECT=1
OUT=
GUEST_IP=193.169.4.151

while [ $# -gt 0 ]; do
    case $1 in
    --image) IMAGE=$2; shift ;;
    --soc) SOC=$2; shift ;;
    --variant) VARIANT=$2; shift ;;
    --ram) RAM=$2; shift ;;
    --sensor) SENSOR=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --ffmpeg) FFMPEG=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --no-reboot) REBOOT= ;;
    --no-rtsp) RTSP= ;;
    --direct-mode) DIRECT=1 ;;
    --no-direct-mode) DIRECT= ;;
    --out) OUT=$2; shift ;;
    --guest-ip) GUEST_IP=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
if [ -z "$IMAGE" ] || [ ! -r "$IMAGE" ]; then
    echo "SKIP: no vendor SDK image (--image; the SDK images are not public)"
    exit 0
fi
[ -x "$QEMU" ] || { echo "missing $QEMU" >&2; exit 2; }

# ---- what the image is ------------------------------------------------------
TXT=${IMAGE%.img}.txt
MODEL=
[ -r "$TXT" ] && MODEL=$(sed -n 's/^uboot_model: *\([A-Z0-9_]*\).*/\1/p' "$TXT")
# else the file name: T41NQ_gcc720_... -> T41_NQ
[ -n "$MODEL" ] || MODEL=$(basename "$IMAGE" | sed -n 's/^\(T[0-9][0-9]\)\([A-Z]*\)_.*/\1_\2/p')
case $MODEL in
T41_NQ) S=t41; V=t41nq ;;
T41_LQ) S=t41; V=t41lq ;;
T41_XQ) S=t41; V=t41x ;;     # no XQ code in the EFUSE table; X is the grade
T41_*) S=t41; V=t41n ;;
T40_XP) S=t40; V=t40xp ;;
T40_*) S=t40; V=t40n ;;
T30_*) S=t30; V=t30$(echo "${MODEL#T30_}" | tr 'A-Z' 'a-z') ;;
T31_*) S=t31; V=t31$(echo "${MODEL#T31_}" | tr 'A-Z' 'a-z') ;;
*) S=; V= ;;
esac
SOC=${SOC:-$S}
VARIANT=${VARIANT:-$V}
[ -n "$SOC" ] || { echo "cannot tell the SoC of $IMAGE (--soc)" >&2; exit 2; }
BOOTARGS=$(strings -n 8 "$IMAGE" | grep -m1 '^bootargs=' | sed 's/^bootargs=//')
# the DRAM: the end of the highest of mem= / rmem= / nmem=, a power of two
if [ -z "$RAM" ]; then
    RAM=$(python3 - "$BOOTARGS" <<'PY'
import re, sys
top = 0
for size, base in re.findall(r'\b[nr]?mem=(\d+)M(?:@(0x[0-9a-fA-F]+))?',
                             sys.argv[1]):
    top = max(top, int(base or '0', 16) + (int(size) << 20))
m = 64
while (m << 20) < top:
    m *= 2
print('%dM' % m)
PY
)
fi
case $SOC in
t40|t41) CORES=2 ;;
*) CORES=1 ;;
esac
# the image's drivers: the .txt list, else the module names in the flash
if [ -r "$TXT" ]; then
    DRIVERS=$(sed -n 's/^driver *: *//p' "$TXT")
else
    DRIVERS=$(strings -n 8 "$IMAGE" | sed -n 's/^\(sensor_[a-z0-9]*_t[0-9]*\)\.ko$/\1/p' | sort -u | tr '\n' ' ')
fi
# T30's media path (its IPU-blended OSD) has a test of its own,
# test-ingenic-t30-rtsp.sh: here T30 boots without a sensor by default
[ "$SENSOR" = auto ] && [ "$SOC" = t30 ] && SENSOR=none
if [ "$SENSOR" = auto ]; then
    SENSOR=none
    for s in gc4663 gc2053 jxf37 jxf22 sc2336 gc2083; do
        case " $DRIVERS " in *" sensor_${s}_$SOC "*) SENSOR=$s; break ;; esac
    done
fi
SENSOROPT=
[ "$SENSOR" != none ] && SENSOROPT=,sensor=$SENSOR
case "$FFMPEG" in
'') command -v ffmpeg >/dev/null && FFMPEG=ffmpeg ;;
esac
FFPROBE=${FFMPEG%ffmpeg*}ffprobe${FFMPEG##*ffmpeg}

WORK=$(mktemp -d)
QPID=
trap '[ -n "$QPID" ] && kill $QPID 2>/dev/null; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK}
mkdir -p "$OUT"
LOG=$OUT/serial.log
IMG=$WORK/flash.bin
cp "$IMAGE" "$IMG"

echo "== $(basename "$IMAGE"): -M $SOC,variant=$VARIANT$SENSOROPT -m $RAM"
echo "   bootargs: $BOOTARGS"

# ---- the root: rcS runs app_init.sh at the test's address ------------------
# The images' app_init.sh set different static addresses (193.169.4.151,
# 10.3.33.235) and live in the compressed JFFS2; rcS (in the SquashFS)
# runs a copy with the ifconfig / route lines at the test's address.  With
# a sensor and --no-direct-mode it also writes /tmp/start_param (start.sh
# prefers it) without direct_mode, so that frames go through memory.
read -r ROOTOFF ROOTSIZE <<< "$(python3 - "$BOOTARGS" <<'PY'
import re, sys
m = re.search(r'mtdparts=[^:]*:(\S+)', sys.argv[1])
off = 0
for p in (m.group(1).split(',') if m else []):
    size, name = re.match(r'(-|\d+[kKmM])\((\w+)\)', p).groups()
    n = int(size[:-1]) << (10 if size[-1] in 'kK' else 20) if size != '-' else 0
    if name == 'root':
        print(off, n)
        break
    off += n
PY
)"
NET=${GUEST_IP%.*}
if command -v unsquashfs >/dev/null && command -v mksquashfs >/dev/null &&
   [ -n "${ROOTSIZE:-}" ]; then
    dd if="$IMG" of="$WORK/root.sq" bs=64k skip=$((ROOTOFF >> 16)) \
        count=$((ROOTSIZE >> 16)) status=none
    unsquashfs -q -d "$WORK/rfs" "$WORK/root.sq" >/dev/null 2>&1
    IVDC=
    [ -n "$RTSP" ] && [ "$SENSOR" != none ] && [ -z "$DIRECT" ] && IVDC=1
    python3 - "$WORK/rfs/etc/init.d/rcS" "$GUEST_IP" "$NET.1" "$IVDC" <<'PY'
import sys
p, ip, gw, ivdc = sys.argv[1:5]
s = open(p).read()
old = '/system/init/app_init.sh &'
new = """# qemu: app_init.sh at the test's address
sed -e 's/^ifconfig eth0 [0-9.]*/ifconfig eth0 %s/' \\
    -e 's/^route add default gw [0-9.]*/route add default gw %s/' \\
    /system/init/app_init.sh > /tmp/app_init.sh
chmod +x /tmp/app_init.sh
""" % (ip, gw)
if ivdc:
    new += """# qemu: no IVDC (direct_mode); frames go through memory
sed -e 's/ direct_mode=1//' -e 's/ ivdc_mem_line=[0-9]*//' \\
    -e 's/--dm 1/--dm 0/' /system/init/start_param > /tmp/start_param
"""
if old not in s:
    sys.exit('rcS: no app_init.sh')
s = s.replace(old, '/tmp/app_init.sh &', 1)
s = s.replace('# Run init script\n', new + '\n# Run init script\n', 1)
open(p, 'w').write(s)
PY
    if [ $? = 0 ]; then
        mksquashfs "$WORK/rfs" "$WORK/root.new" -comp xz -noappend -quiet \
            -all-root >/dev/null
        python3 - "$IMG" "$WORK/root.new" "$ROOTOFF" "$ROOTSIZE" <<'PY'
import sys
img = bytearray(open(sys.argv[1], 'rb').read())
r = open(sys.argv[2], 'rb').read()
off, size = int(sys.argv[3]), int(sys.argv[4])
assert len(r) <= size, 'the rebuilt root does not fit'
img[off:off + size] = r + b'\xff' * (size - len(r))
open(sys.argv[1], 'wb').write(img)
PY
        echo "   root rebuilt: app_init.sh at $GUEST_IP${IVDC:+, start_param without direct_mode}"
    fi
else
    echo "   (no squashfs tools: the image keeps its own address, assumed $GUEST_IP)"
fi

# ---- boot ---------------------------------------------------------------------
read -r PTEL PRTSP <<< "$(python3 -c '
import socket
ss = [socket.socket() for _ in range(2)]
for s in ss:
    s.bind(("127.0.0.1", 0))
print(*[s.getsockname()[1] for s in ss])')"
NIC="user,net=$NET.0/24,host=$NET.1"
NIC="$NIC,hostfwd=tcp:127.0.0.1:$PTEL-$GUEST_IP:23"
NIC="$NIC,hostfwd=tcp:127.0.0.1:$PRTSP-$GUEST_IP:8554"
# shellcheck disable=SC2086  # QEMU_EXTRA: extra options, word-split
"$QEMU" -M "$SOC,variant=$VARIANT$SENSOROPT,flash-file=$IMG" -m "$RAM" \
    ${QEMU_EXTRA:-} -display none -monitor none -serial "file:$LOG" -nic "$NIC" \
    > "$OUT/qemu.err" 2>&1 &
QPID=$!

fail=0
pass() { echo "  PASS: $1"; }
bad() { echo "  FAIL: $1"; fail=1; }
check() {
    if grep -aq -- "$2" "$LOG"; then pass "$1"; else bad "$1"; fi
}
count() { local n; n=$(grep -ac -- "$1" "$LOG" 2>/dev/null); echo "${n:-0}"; }
wait_for() {        # pattern, occurrences, seconds
    local i
    for i in $(seq "$3"); do
        [ "$(count "$1")" -ge "$2" ] && return 0
        kill -0 $QPID 2>/dev/null || return 1
        sleep 1
    done
    return 1
}
telnet() {        # a loaded host can keep the guest's telnetd slow: retry
    local try
    for try in 1 2 3; do
        python3 "$HERE/ingenic-telnet.py" "$PTEL" --timeout 90 -- "$@" && return 0
        sleep 10
    done
    return 1
}

wait_for 'login:' 1 "$TIMEOUT"
# the application, when there is a sensor: start.sh's last words
APP=
if [ "$SENSOR" != none ]; then
    for i in $(seq 480); do
        grep -aq 'Play this video stream\|carrier-server exit\|^err: \|^exit$' "$LOG" && break
        kill -0 $QPID 2>/dev/null || break
        sleep 1
    done
    grep -aq 'Play this video stream' "$LOG" && APP=1
fi
sleep 5

echo "boot:"
check "SPL" 'U-Boot SPL'
check "U-Boot starts the kernel" 'Starting kernel'
check "kernel up" 'Linux version'
check "root SquashFS mounted" 'VFS: Mounted root (squashfs'
[ "$CORES" = 2 ] && check "both cores up" 'Brought up 2 CPUs'
check "vendor login prompt" 'login:'
if [ "$SENSOR" != none ]; then
    if [ -n "$APP" ]; then
        pass "the application serves $(grep -a -m1 -o 'rtsp://[^ ]*' "$LOG" | tr -d '\r')"
    else
        bad "the application ($(grep -a -m1 '^err: \|carrier-server exit' "$LOG" | tr -d '\r'))"
    fi
fi

echo "network (telnet through the GMAC):"
if telnet "grep -c ^processor /proc/cpuinfo" "ifconfig eth0 | grep 'inet addr'" \
        "cat /proc/jz/sinfo/info" > "$OUT/telnet.log" 2>&1; then
    pass "telnet login as root"
    n=$(sed -n '/^### grep -c/{n;p}' "$OUT/telnet.log" | tr -d '\r ')
    [ "$n" = "$CORES" ] && pass "$n CPU(s) in /proc/cpuinfo" || bad "CPUs in /proc/cpuinfo: $n, want $CORES"
    grep -q "inet addr:$GUEST_IP" "$OUT/telnet.log" && pass "eth0 is $GUEST_IP" || bad "eth0 address"
    echo "    sinfo: $(sed -n '/^### cat/{n;p}' "$OUT/telnet.log")"
    if [ -n "$DIRECT" ] && [ -n "$APP" ] && [ "$SOC" = t41 ] &&
            telnet "cat /proc/jz/isp/isp-ivdc" "grep -i 'ivdc\|jpeg\|isp' /proc/interrupts" \
                > "$OUT/ivdc.log" 2>&1; then
        fd=$(sed -n 's/.*ISP Frame Done *: *\([0-9]*\).*/\1/p' "$OUT/ivdc.log" | head -1)
        fs=$(sed -n 's/.*ISP Frame Start *: *\([0-9]*\).*/\1/p' "$OUT/ivdc.log" | head -1)
        er=$(sed -n 's/.*\(Overflow\|Err\) *: *\([0-9]*\).*/\2/p' "$OUT/ivdc.log" | awk '{s+=$1} END{print s+0}')
        [ "${fd:-0}" -gt 0 ] && [ "$er" = 0 ] &&
            pass "IVDC: $fs ISP frame starts, $fd frame dones, no errors" ||
            bad "IVDC counters (frame done ${fd:-?}, errors $er)"
    fi
else
    bad "telnet login as root"; sed 's/^/    /' "$OUT/telnet.log"
fi

if [ -n "$APP" ] && [ -n "$RTSP" ]; then
    echo "RTSP:"
    if [ -z "$FFMPEG" ]; then
        echo "  SKIP: no ffmpeg (--ffmpeg)"
    else
        got=
        URL="rtsp://127.0.0.1:$PRTSP/main"
        # live555 answers DESCRIBE without an SDP until the encoder has
        # produced its parameter sets: retry for a few minutes.  The
        # bitstream is copied as it comes (the packets' payloads, Annex B,
        # on a pipe: a container trips over packets without timestamps on
        # T40, and a docker ffmpeg may see the host's directories under
        # other names) and decoded afterwards: decoding 1080p while
        # pulling falls behind the stream, and live555 then drops RTP
        # packets on the TCP connection -- a lost P picture, which inter
        # coding turns into a picture that drifts
        for try in 1 2 3 4 5 6 7 8; do
            timeout 240 $FFMPEG -hide_banner -rtsp_transport tcp \
                -i "$URL" -frames:v 20 -an -c:v copy -f rawvideo - \
                > "$OUT/main.es" 2> "$OUT/ffmpeg.log"
            [ -s "$OUT/main.es" ] && { got=1; break; }
            sleep 20
        done
        if [ -n "$got" ]; then
            case $(grep -m1 -o 'Video: [a-z0-9]*' "$OUT/ffmpeg.log") in
            *hevc*) ESF=hevc ;; *) ESF=h264 ;;
            esac
            $FFMPEG -hide_banner -err_detect explode -f $ESF -i - -an \
                -c:v rawvideo -pix_fmt nv12 -f nut - \
                < "$OUT/main.es" > "$OUT/main.nut" 2> "$OUT/decode.log"
            [ -s "$OUT/main.nut" ] || got=
        fi
        if [ -z "$got" ]; then
            bad "pull rtsp://$GUEST_IP:8554/main"
            grep -v '^docker' "$OUT/ffmpeg.log" | tail -3
        else
            codec=$(grep -m1 -o 'Video: [a-z0-9]*' "$OUT/ffmpeg.log" | cut -d' ' -f2)
            sz=$(grep -m1 'Stream #0:0: Video' "$OUT/ffmpeg.log" |
                grep -o '[0-9]\{2,5\}x[0-9]\{2,5\}' | head -1)
            pass "pulled rtsp://$GUEST_IP:8554/main: $codec $sz"
            $FFMPEG -v error -i - -fps_mode passthrough -f rawvideo -pix_fmt nv12 - \
                < "$OUT/main.nut" > "$OUT/main.nv12" 2> /dev/null
            # the decoder's messages (the pull's are the muxer's)
            grep -iE 'error|invalid|corrupt|missing|outside' "$OUT/decode.log" |
                grep -v '^docker' > "$OUT/dec.log"
            errs=$(grep -v -c '^docker' "$OUT/dec.log")
            n=$(python3 - "$OUT" "$sz" <<'PY'
import sys
d, sz = sys.argv[1], sys.argv[2]
w, h = map(int, sz.split('x'))
fs = w * h * 3 // 2
b = open(d + '/main.nv12', 'rb').read()
n = len(b) // fs
if n:
    open(d + '/first.nv12', 'wb').write(b[:fs])
    open(d + '/last.nv12', 'wb').write(b[(n - 1) * fs:n * fs])
print(n)
PY
)
            [ "$errs" = 0 ] && [ "$n" -gt 1 ] && pass "$n pictures decode, no decoder message" ||
                bad "decode: $n pictures, $errs messages"
            if python3 "$HERE/ingenic-isp-check.py" --advance "$sz:$OUT/first.nv12" \
                    "$sz:$OUT/last.nv12" > "$OUT/chk.log" 2>&1; then
                pass "the ISP's scene (counter, logo, moving dot)"
            else
                bad "the ISP's scene"; sed 's/^/    /' "$OUT/chk.log"
            fi
        fi
    fi
fi

if [ -n "$REBOOT" ]; then
    echo "reboot (the TCU watchdog):"
    spl=$(count 'U-Boot SPL')
    login=$(count 'login:')
    telnet "reboot" > "$OUT/reboot.log" 2>&1
    if wait_for 'U-Boot SPL' $((spl + 1)) 120; then
        pass "the SPL again"
        if wait_for 'login:' $((login + 1)) "$TIMEOUT"; then
            pass "the login prompt again"
        else
            bad "the login prompt again"
        fi
    else
        bad "the SPL again"
    fi
fi

kill $QPID 2>/dev/null
wait $QPID 2>/dev/null
QPID=
if grep -aq 'Kernel panic\|Oops\[#\|Unable to handle kernel' "$LOG"; then
    bad "no kernel panic or oops"; grep -a -m3 'Kernel panic\|Oops\|Unable to handle' "$LOG"
else
    pass "no kernel panic or oops"
fi
n=$(count "ost couldn't find ostfr")
[ "$n" = 0 ] && pass "no core OST interrupt storm" || bad "core OST storm ($n)"
if [ $fail != 0 ]; then
    echo "--- serial log (tail) ---"
    grep -av 'sensor_read\|err sensor\|i2c' "$LOG" | tail -30
    echo FAIL
    exit 1
fi
echo PASS
