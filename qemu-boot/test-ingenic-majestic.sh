#!/bin/bash
# majestic on an emulated Ingenic camera: a digital twin for reproducing
# what a T40 or T41 owner reports (docs/ingenic.md, "majestic").
#
#   t40  a T40N with a GC4653: the OpenIPC t40 lite release's kernel and
#        root, its own majestic or --majestic BIN.
#   t41  a T41NQ with a GC4663 -- the sensor the twin models and the SDK
#        image has a driver for; the owner's camera in OpenIPC/majestic#376
#        has a GC5603, which neither does, so that sensor is not covered
#        here.  OpenIPC has no T41 release, so the root is
#        still the t40 lite release's (the same musl userland; T41's
#        libimp is uClibc-built like T40's, and runs on the same shim), and
#        the rest comes from the T41 SDK 1.2.0 (--sdk): its libimp,
#        libalog and libsysutils, its vendor modules and sensor setting
#        out of the 16 MiB T41NQ image, and its kernel rebuilt with
#        overlayfs (build-ingenic-t41-kernel.sh, or --kernel).  majestic
#        has to be a t41 build: --majestic BIN is required.
#
# The camera boots as a camera does: OpenIPC's /init mounts the JFFS2
# rootfs_data partition as the overlay, so what majestic saves stays saved
# (across a --reboot, too), busybox init runs the release's rc scripts,
# S70vendor loads the vendor modules through load_ingenic (on t41 a
# stand-in with the module parameters and memory split measured on that
# owner's T41NQ), and S95majestic starts majestic under its
# supervisor.  The twin adds three things: S00twin touches /tmp/closesimd
# (libimp's MXU2 paths are not modelled), a creditable seedrng seed (the
# machine gathers almost no entropy, and getrandom() would hold majestic
# before it reads its config for good), and rc.local mirrors the syslog,
# where majestic -s logs, to the console the checks read.  root's password
# is 12345: an unclaimed camera streams nothing.
#
# What is checked, from the console, the network and QEMU:
#   - majestic brings the SDK up ("Ingenic SDK started");
#   - with --audio, with the microphone on (audio.enabled: true), which a
#     T40 build could not do before OpenIPC/majestic#257;
#   - with --audio on t41, the microphone is heard: /audio.pcm carries
#     the codec's 1 kHz tone, and the vendor driver never timed a read out
#     ("get mic timeout!", after which libimp records nothing more);
#   - with --audio on t41, the speaker too: a 1 s 440 Hz tone POSTed to
#     /play_audio comes out of the codec, all of it (OpenIPC/majestic#376:
#     every frame was refused, and a clip shorter than the queue was
#     silent);
#   - 50 pictures of each RTSP stream decode (ffmpeg's own count, with no
#     error), and /image.jpg is a JPEG of the main stream's size;
#   - a setting written through the API reads back, and with --reboot it
#     survives a power cycle;
#   - day/night from a light sensor: PB17 is nightMode.lightSensorPin, and
#     the host pulls it low (night) and lets it go (day) through
#     qom-set /machine/gpio.  majestic must change mode both ways (its
#     night_enabled in /metrics), and the
#     switch to night must reach the ISP: ingenic-isp.trace-regs logs
#     every ISP core write, and the night window must write at least
#     --min-night-regs registers the day before it did not (the night
#     tuning set: about 980 on the T40 release, against 8 to 15 -- the
#     per-frame curves' drift alone -- for a switch that never reached the
#     ISP, as in OpenIPC/majestic#258).
#
#   test-ingenic-majestic.sh [--soc t40|t41] [--majestic BIN] [--audio]
#                            [--sdk DIR] [--kernel UIMAGE] [--reboot]
#                            [--work DIR] [--qemu Q] [--timeout SECONDS]
#                            [--min-night-regs N] [--keep]
#   --sdk  the extracted Ingenic-SDK-T41-1.2.0-20230713 (default $T41_SDK);
#          the test skips (exit 0) without it, as the other T41 tests do
# Needs ffmpeg and ffprobe on PATH.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
QEMU=$HERE/../qemu-src/build/qemu-system-mipsel
SOC=t40
WORK=
BIN=
AUDIO=false
TIMEOUT=360
MIN_NIGHT=100
KEEP=
SDK=${T41_SDK:-}
KERNEL=
REBOOT=

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --majestic) BIN=$2; shift ;;
    --audio) AUDIO=true ;;
    --sdk) SDK=$2; shift ;;
    --kernel) KERNEL=$2; shift ;;
    --reboot) REBOOT=1 ;;
    --work) WORK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    --min-night-regs) MIN_NIGHT=$2; shift ;;
    --keep) KEEP=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
# The speaker too, on T41 only: the twin's T41 audio.ko plays and records
# through its model, T40's stops recording after about a second, and with
# its output on as well the T40 twin's streams stalled after a power cycle
SPEAKER=false
if [ "$AUDIO" = true ] && [ "$SOC" = t41 ]; then
    SPEAKER=true
fi
case $SOC in
t40) MACHINE=t40,variant=t40n,sensor=gc4653; SENSOR=gc4653
     MEM="mem=64M rmem=64M@0x4000000"
     # OpenIPC's 16 MiB NOR
     KERN_KB=2048; ROOT_KB=8192 ;;
t41) MACHINE=t41,variant=t41nq,sensor=gc4663; SENSOR=gc4663
     # the owner's T41NQ in OpenIPC/majestic#376
     MEM="mem=74M@0x0 rmem=45M@0x4A00000 nmem=9M@0x7700000"
     # the SDK kernel is 2.2 MB, more than OpenIPC's 2 MiB slot
     KERN_KB=2560; ROOT_KB=8192
     IMAGE=$SDK/resource/image_t41/T41NQ_gcc720_uclibc_16M_202307061021.img
     if [ -z "$SDK" ] || [ ! -f "$IMAGE" ]; then
         echo "SKIP: no T41 SDK (--sdk or T41_SDK; the image is not public)"
         exit 0
     fi
     [ -n "$BIN" ] || { echo "--soc t41 needs --majestic (a t41 build)" >&2; exit 2; } ;;
*) echo "no majestic twin for $SOC" >&2; exit 2 ;;
esac
for t in ffmpeg ffprobe; do
    command -v $t >/dev/null || { echo "$t is not on PATH" >&2; exit 2; }
done

WORK=${WORK:-$(mktemp -d)}
mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
if [ ! -f "$WORK/uImage.t40" ] || [ ! -f "$WORK/rootfs.squashfs.t40" ]; then
    bash "$HERE/fetch-openipc-lite.sh" t40 "$WORK"
fi
if [ $SOC = t41 ] && [ -z "$KERNEL" ]; then
    KERNEL=$WORK/uImage.t41
    # the build reports its own failure on stderr
    [ -f "$KERNEL" ] || bash "$HERE/build-ingenic-t41-kernel.sh" "$SDK" "$WORK" >/dev/null
fi
KERNEL=${KERNEL:-$WORK/uImage.t40}

RFS=$WORK/rfs
rm -rf "$RFS"
unsquashfs -q -d "$RFS" "$WORK/rootfs.squashfs.t40" >/dev/null
if [ -n "$BIN" ]; then
    cp "$BIN" "$RFS/usr/bin/majestic"
    chmod 755 "$RFS/usr/bin/majestic"
fi
if [ $SOC = t41 ]; then
    cp "$SDK"/sdk/lib/uclibc/{libimp,libalog,libsysutils}.so "$RFS/usr/lib/"
    # the image: SPL + U-Boot, the uImage at 0x40000, the root at 0x2C0000
    # and the vendor modules in the JFFS2 at 0x4C0000 (/system)
    python3 - "$IMAGE" "$WORK/appfs.jffs2" <<'PY'
import sys
open(sys.argv[2], 'wb').write(open(sys.argv[1], 'rb').read()[0x4C0000:0xE00000])
PY
    rm -rf "$WORK/appfs"
    jefferson -d "$WORK/appfs" "$WORK/appfs.jffs2" >/dev/null
    mkdir -p "$RFS/lib/modules/t41"
    cp "$WORK"/appfs/lib/modules/*.ko "$RFS/lib/modules/t41/"
    cp "$WORK/appfs/etc/sensor/$SENSOR-t41.bin" "$RFS/etc/sensor/"
    # OpenIPC's sensor file, for the GC4653's GC4663 sibling (same address
    # and size)
    sed "s/gc4653/$SENSOR/" "$RFS/etc/sensor/gc4653.yaml" \
        > "$RFS/etc/sensor/$SENSOR.yaml"
    cat > "$RFS/usr/bin/load_ingenic" <<'EOF'
#!/bin/sh
# T41 stand-in for the firmware's load_ingenic: the SDK image's modules,
# with the parameters measured on an owner's T41NQ (OpenIPC/majestic#376)
cd /lib/modules/t41 || exit 1
insmod avpu.ko
insmod sinfo.ko
echo 1 > /proc/jz/sinfo/info
insmod tx-isp-t41.ko clk_name=mpll isp_clk=300000000 direct_mode=0
insmod sensor_gc4663_t41.ko
# as the same camera loads it: majestic drives the speaker's enable pin
insmod audio.ko spk_gpio=-1 spk_level=-1
EOF
    chmod 755 "$RFS/usr/bin/load_ingenic"
fi
cat > "$RFS/etc/majestic.yaml" <<EOF
system:
  logLevel: debug
isp:
  sensorConfig: /etc/sensor/$SENSOR.yaml
video0:
  enabled: true
  codec: h264
video1:
  enabled: true
  codec: h264
  size: 640x360
jpeg:
  enabled: true
audio:
  enabled: $AUDIO
  outputEnabled: $SPEAKER
nightMode:
  colorToGray: true
  irCutPin1: 122
  irCutPin2: 123
  lightMonitor: true
  lightSensorPin: 49
EOF
# claimed: an unclaimed camera streams nothing
sed -i "s|^root:[^:]*:|root:$(openssl passwd -1 12345):|" "$RFS/etc/shadow"
# The twin, before S01seedrng on every boot: a creditable seed (the
# machine gathers no entropy of its own, so the seed seedrng leaves for the
# next boot is not creditable, and after a power cycle getrandom() would
# hold majestic for good), and libimp's MXU2 paths off (not modelled)
head -c 512 /dev/urandom > "$RFS/etc/twin-seed"
cat > "$RFS/etc/init.d/S00twin" <<'EOF'
#!/bin/sh
[ "$1" = start ] || exit 0
mkdir -p /var/lib/seedrng
cp /etc/twin-seed /var/lib/seedrng/seed.credit
touch /tmp/closesimd
EOF
chmod 755 "$RFS/etc/init.d/S00twin"
# majestic -s logs to syslog; the checks read the console
sed -i 's|^exit 0$|logread -f > /dev/console 2>\&1 \&\nexit 0|' "$RFS/etc/rc.local"
grep -q '^logread -f' "$RFS/etc/rc.local" || { echo "rc.local: no exit 0 to insert before" >&2; exit 1; }
mksquashfs "$RFS" "$WORK/root.sq" -comp xz -noappend -quiet -all-root >/dev/null
[ -n "$KEEP" ] || rm -rf "$RFS" "$WORK/appfs"

# OpenIPC's NOR layout (on t41 after the SDK image's SPL + U-Boot) with a
# U-Boot environment that names the sensor; rootfs_data erased
python3 - "$WORK/nor.bin" "$KERNEL" "$WORK/root.sq" "$KERN_KB" "$ROOT_KB" \
    "${IMAGE:-}" sensor=$SENSOR soc=$SOC <<'PY'
import struct, sys, zlib
nor, kernel, root, kern_kb, root_kb, image = sys.argv[1:7]
img = bytearray(b'\xff' * (16 << 20))
if image:
    img[:0x40000] = open(image, 'rb').read()[:0x40000]
kern_off = 0x50000
root_off = kern_off + int(kern_kb) * 1024
for path, off, room in ((kernel, kern_off, int(kern_kb) * 1024),
                        (root, root_off, int(root_kb) * 1024)):
    data = open(path, 'rb').read()
    assert len(data) <= room, path + ' does not fit'
    img[off:off + len(data)] = data
env = b''.join(kv.encode() + b'\0' for kv in sys.argv[7:]) + b'\0'
env += b'\0' * (0x10000 - 4 - len(env))
img[0x40000:0x50000] = struct.pack('<I', zlib.crc32(env)) + env
open(nor, 'wb').write(img)
PY

TRACE=$WORK/isp-trace.log
MON=$(mktemp -u /tmp/twin.XXXXXX.sock)     # sun_path is 108 bytes
read -r RTSP HTTP <<< "$(python3 -c '
import socket
s = [socket.socket() for _ in range(2)]
for x in s: x.bind(("127.0.0.1", 0))
print(*(x.getsockname()[1] for x in s))')"
QPID=
# the T40 kernel's SFC driver names the chip jz_sfc, T41's sfc0_nor
[ $SOC = t40 ] && MTDID=jz_sfc || MTDID=sfc0_nor
# One console file per boot: QEMU's -serial file: truncates what it opens.
# Emptied here as well, before QEMU gets to it: a --work directory keeps
# the last run's, and the checks would read its "Ingenic SDK started" and
# go on against a camera still booting.
rm -f "$WORK"/console.*.log
# With --audio the speaker plays into a wav. QEMU writes its header only at
# exit, so the checks read the samples after the 44-byte header themselves.
AUDIODEV=
AUDIOOUT=()
if [ "$SPEAKER" = true ]; then
    AUDIODEV=,audiodev=snd
    AUDIOOUT=(-audiodev "wav,id=snd,path=$WORK/speaker.wav")
fi
NBOOT=0
boot() {
    NBOOT=$((NBOOT + 1))
    CON=$WORK/console.$NBOOT.log
    : > "$CON"
    "$QEMU" -M "$MACHINE,flash-file=$WORK/nor.bin$AUDIODEV" -m 128M \
        "${AUDIOOUT[@]}" \
        -global ingenic-isp.trace-regs=on -display none -serial file:"$CON" \
        -monitor unix:"$MON",server,nowait \
        -nic user,restrict=on,hostfwd=tcp:127.0.0.1:$RTSP-:554,hostfwd=tcp:127.0.0.1:$HTTP-:80 \
        -kernel "$KERNEL" \
        -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=$MTDID:256k(boot),64k(env),${KERN_KB}k(kernel),${ROOT_KB}k(rootfs),-(rootfs_data) $MEM" \
        </dev/null 2>>"$TRACE" &
    QPID=$!
}
: > "$TRACE"
boot
trap 'kill $QPID 2>/dev/null; rm -f "$MON"' EXIT

mon() {
    python3 - "$MON" "$1" <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_UNIX)
s.connect(sys.argv[1])
time.sleep(0.2)
s.recv(65536)
s.sendall(sys.argv[2].encode() + b'\n')
time.sleep(0.5)
PY
}
# the windows the trace is split into
mark() { echo "=== TWIN $1" >> "$TRACE"; }
# waitfor PATTERN SECONDS [LINE]: the console from LINE on
waitfor() {
    for _ in $(seq "$2"); do
        tail -n +"${3:-1}" "$CON" 2>/dev/null | grep -aq -E "$1" && return 0
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
# decodes STREAM: 50 pictures by ffmpeg's own count, ffmpeg exiting 0 and
# saying nothing at error level (bar the RTP timestamps it complains about
# on any live stream).  The video track only: -frames:v ends the video, and
# with the microphone's track live too ffmpeg would read it until killed
decodes() {
    local out rc=0 n err
    out=$(timeout -k 10 180 ffmpeg -nostdin -v error -progress pipe:1 \
        -rtsp_transport tcp -allowed_media_types video -i "$URL/$1" \
        -frames:v 50 -f null - \
        2>"$WORK/ffmpeg.err") || rc=$?
    n=$(printf '%s\n' "$out" | sed -n 's/^frame=//p' | tail -1)
    err=$(grep -v 'non monotonically increasing dts' "$WORK/ffmpeg.err" || true)
    [ -z "$err" ] || echo "$err" | head -3
    if [ "$rc" != 0 ] || [ -n "$err" ] || [ "${n:-0}" -lt 50 ]; then
        echo "    ($1: ffmpeg exit $rc, ${n:-0} pictures)"
        return 1
    fi
}
# /image.jpg: a JPEG, and the main stream's size
snapshot() {
    local want got
    want=$(timeout -k 10 60 ffprobe -v error -rtsp_transport tcp -select_streams v \
        -show_entries stream=width,height -of csv=s=x:p=0 "$URL/stream=0")
    curl -s -m 60 -u root:12345 -o "$WORK/snap.jpg" \
        "http://127.0.0.1:$HTTP/image.jpg" || return 1
    got=$(file "$WORK/snap.jpg" | grep -o 'JPEG image data.*' |
        grep -oE '[0-9]+x[0-9]+' | tail -1)     # after any JFIF density
    [ -n "$got" ] && [ "$got" = "$want" ] ||
        { echo "    (/image.jpg: '${got:-not a JPEG}', main stream '$want')"; return 1; }
}
# saved: written through the API, and read back from it.  A setting
# majestic applies without rebuilding anything (rtsp.keyframeOnSkip,
# default false), so what is tested is the flash and not a pipeline
# rebuild -- on T40 the twin's IMP teardown hangs in one, and the HTTP
# server stops answering with it
SAVED='{"rtsp":{"keyframeOnSkip":true}}'
saved() {
    curl -sf -m 30 -u root:12345 -o /dev/null -X POST \
        -H 'Content-Type: application/json' \
        -d "$SAVED" "http://127.0.0.1:$HTTP/api/v1/config" &&
        [ "$(api_get rtsp keyframeOnSkip)" = True ]
}
# mic: /audio.pcm carries the codec's 1 kHz tone, and no read of it timed
# out in the vendor driver.  It cannot tell IMP_AI device 0 from 1: the
# twin's T41 audio.ko records the codec on both.
mic() {
    curl -s -m 4 -u root:12345 -o "$WORK/mic.pcm" \
        "http://127.0.0.1:$HTTP/audio.pcm" || [ $? = 28 ] || return 1
    ! grep -aq 'get mic timeout' "$CON" || { echo '    (mic: get mic timeout)'; return 1; }
    python3 - "$WORK/mic.pcm" <<'PY'
import struct, sys
d = open(sys.argv[1], 'rb').read()
x = struct.unpack('<%dh' % (len(d) // 2), d[:len(d) // 2 * 2])
r = 8000
s = x[r // 4:]  # past the first quarter second
if len(s) < r or max(abs(v) for v in s) < 200:
    print('    (mic: %d samples, silent or short)' % len(x))
    sys.exit(1)
z = sum(1 for i in range(1, len(s)) if (s[i - 1] < 0) != (s[i] < 0))
hz = z / 2 / (len(s) / r)
print('    (mic: %.2f s at %.0f Hz)' % (len(x) / r, hz))
sys.exit(0 if 950 < hz < 1050 else 1)
PY
}
# speaker: a 1 s 440 Hz tone POSTed to /play_audio comes out of the codec,
# all of it -- the end of a clip is where frames went missing
speaker() {
    python3 -c 'import math, struct, sys
sys.stdout.buffer.write(b"".join(struct.pack("<h", int(6000 * math.sin(2 * math.pi * 440 * i / 8000))) for i in range(8000)))' \
        > "$WORK/tone.pcm"
    local from
    from=$(stat -c %s "$WORK/speaker.wav")
    curl -sf -m 60 -u root:12345 -o /dev/null --data-binary @"$WORK/tone.pcm" \
        "http://127.0.0.1:$HTTP/play_audio" || return 1
    sleep 5
    python3 - "$WORK/speaker.wav" "$from" <<'PY'
import struct, sys
d = open(sys.argv[1], 'rb').read()
off = max(44, int(sys.argv[2]))
off -= (off - 44) % 4
x = struct.unpack('<%dh' % ((len(d) - off) // 2),
                  d[off:off + (len(d) - off) // 2 * 2])[::2]
r = 44100
idx = [i for i, v in enumerate(x) if abs(v) > 200]
if not idx:
    print('    (speaker: silent)')
    sys.exit(1)
a, b = idx[0], idx[-1]
s = x[a:b]
z = sum(1 for i in range(1, len(s)) if (s[i - 1] < 0) != (s[i] < 0))
hz = z / 2 / (len(s) / r)
print('    (speaker: %.2f s at %.0f Hz)' % ((b - a) / r, hz))
sys.exit(0 if (b - a) / r > 0.8 and 420 < hz < 460 else 1)
PY
}
# night_is 0|1: majestic's own answer, from /metrics.  Not the console:
# the console is the syslog mirrored by `logread -f`, which stops following
# when syslogd's ring laps it -- on T40 with --audio the twin's audio.ko
# fills the ring with stack dumps, and a mode change went unseen.
night_is() {
    curl -s -m 10 -u root:12345 "http://127.0.0.1:$HTTP/metrics" |
        grep -qx "night_enabled $1"
}
# pollfor SECONDS COMMAND...: until COMMAND succeeds or the guest is gone
pollfor() {
    local t=$1
    shift
    for _ in $(seq "$t"); do
        "$@" && return 0
        kill -0 $QPID 2>/dev/null || return 1
        sleep 1
    done
    return 1
}
api_get() {     # api_get SECTION KEY
    curl -s -m 30 -u root:12345 "http://127.0.0.1:$HTTP/api/v1/config.json" |
        python3 -c 'import json, sys
try:
    print(json.load(sys.stdin).get(sys.argv[1], {}).get(sys.argv[2]))
except ValueError:
    print("no answer")' "$1" "$2"
}

waitfor 'Ingenic SDK started|Cannot start SDK' "$TIMEOUT" || true
echo "$SOC, majestic (audio: $AUDIO):"
check "majestic starts the SDK" grep -aq 'Ingenic SDK started' "$CON"
if grep -aq 'Ingenic SDK started' "$CON"; then
    sleep 5
    check "the main stream decodes" decodes stream=0
    check "the second stream decodes" decodes stream=1
    check "/image.jpg is a JPEG of the main stream's size" snapshot
    # The speaker on T41, whose SDK audio.ko the twin plays through
    if [ "$SPEAKER" = true ]; then
        check "the microphone is heard on /audio.pcm" mic
        check "/play_audio is heard" speaker
    fi
    mark DAY
    sleep 10
    mark NIGHT
    mon "qom-set /machine/gpio pb 0xfffdffff"
    check "the light sensor turns night on" pollfor 60 night_is 1
    sleep 20
    mark BACK
    mon "qom-set /machine/gpio pb 0xffffffff"
    check "and day again" pollfor 60 night_is 0
    sleep 5
    mark END
    n=$(python3 - "$TRACE" <<'PY'
import collections, re, sys
rx = re.compile(r'ingenic-isp: core \[0x([0-9a-f]+)\] = 0x([0-9a-f]+)')
seen = collections.defaultdict(lambda: collections.defaultdict(set))
win = None
for line in open(sys.argv[1], errors='replace'):
    if line.startswith('=== TWIN '):
        win = line.split()[2]
        continue
    m = rx.search(line)
    if m and win:
        seen[win][m.group(1)].add(m.group(2))
day, night = seen['DAY'], seen['NIGHT']
print(sum(1 for r, v in night.items() if not v <= day.get(r, set())))
PY
)
    echo "  ($n ISP registers written at night that the day did not)"
    check "night reaches the ISP" [ "$n" -ge "$MIN_NIGHT" ]
    # a setting saved: the overlay is the camera's flash
    check "a setting is saved" saved
    sleep 5
    if [ -n "$REBOOT" ] && ! kill -0 $QPID 2>/dev/null; then
        check "the camera is still running for the power cycle" false
    elif [ -n "$REBOOT" ]; then
        # a power cycle: the setting must come back from rootfs_data
        kill $QPID 2>/dev/null || true
        wait $QPID 2>/dev/null || true
        boot
        check "after a power cycle majestic starts again" \
            waitfor 'Ingenic SDK started' "$TIMEOUT"
        sleep 5
        check "the setting survived it" [ "$(api_get rtsp keyframeOnSkip)" = True ]
        check "the main stream decodes again" decodes stream=0
    fi
fi
if cat "$WORK"/console.*.log | grep -aq -E 'IMP_[A-Za-z_]+\(\) failed|Cannot start SDK'; then
    cat "$WORK"/console.*.log | grep -a -E 'IMP_[A-Za-z_]+\(\) failed|Cannot start SDK' | sed 's/\x1b\[[0-9;]*m//g' | head -5
fi
if cat "$WORK"/console.*.log | grep -aq -E 'Oops|Kernel panic'; then
    echo "  FAIL: kernel trouble"
    fail=1
fi
[ $fail = 0 ] || echo "consoles in $WORK/console.*.log, ISP trace in $TRACE"
exit $fail
