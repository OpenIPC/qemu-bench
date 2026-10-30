#!/bin/bash
# Sound through the emulated Ingenic AIC, codec and PDMA, end to end: the
# kernel, rootfs and vendor audio.ko of an OpenIPC release (T31) or
# of the T41 SDK's flash image, and a libimp client
# (qemu-boot/ingenic-audio-test.c) that records through IMP_AI -- the
# AIC's 1 kHz tone -- and plays a 440 Hz sine through IMP_AO into a wav
# audiodev.  On the host the recording's and the wav's frequencies are
# measured (an FFT peak) and checked.
#
#   bash qemu-boot/test-ingenic-audio.sh [--soc t31|t41] [--fw DIR]
#        [--image T41_IMG] [--rate 16000] [--qemu PATH] [--out DIR]
#        [--timeout S]
#
# --fw holds uImage.<soc> and rootfs.squashfs.<soc> (an OpenIPC "lite"
# release, fetch-openipc-lite.sh).  The client is built with the Ingenic
# toolchains (as test-ingenic-isp.sh), statically against the SDK's uClibc
# libimp.a: T31 SDK 1.1.5's on the OpenIPC parts, T41 SDK 1.2.0's on T41.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=
TC=/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller/bin
TC40=/home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8/bin
CROSS=mips-linux-uclibc-
CCFLAGS=
SDK=/home/ai/dimerr/local/ingenic/extract/isvp-t31/software/sdk/Ingenic-SDK-T31-1.1.5-20220514/sdk/5.4.0
SDK41=/home/ai/dimerr/tmp-scratch/t41sdk/Ingenic-SDK-T41-1.2.0-20230713/sdk
IMAGE41=/home/ai/dimerr/local/ingenic/extract/t41-sdk-full/Ingenic-SDK-T41-1.2.0-20230713/resource/image_t41/T41NQ_gcc720_uclibc_16M_202307061021.img
IMAGE=
SOC=t31
RATE=16000
TONE=1000
PLAY=440
RAM=64M
MEM="mem=40M rmem=24M@0x2800000"
VARIANT=
OUT=
TIMEOUT=600

while [ $# -gt 0 ]; do
    case $1 in
    --soc) SOC=$2; shift ;;
    --fw) FW=$2; shift ;;
    --image) IMAGE=$2; shift ;;
    --rate) RATE=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
FW=${FW:-/home/ai/dimerr/local/ingenic/fw/$SOC}
case $SOC in
t41) IMAGE=${IMAGE:-$IMAGE41}; TC=$TC40; CROSS=mips-linux-gnu-
     CCFLAGS="-muclibc -fno-use-linker-plugin"
     RAM=128M; MEM="mem=48M@0x0 rmem=64M@0x3000000 nmem=16M@0x7000000"
     VARIANT=,variant=t41nq
     NEED="$IMAGE $SDK41/lib/uclibc/libimp.a" ;;
t31)
     NEED="$FW/uImage.$SOC $FW/rootfs.squashfs.$SOC" ;;
*) echo "no audio test for $SOC" >&2; exit 2 ;;
esac
for f in $NEED "$QEMU" "$TC/${CROSS}gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== rootfs with the audio client"
CC="$TC/${CROSS}gcc $CCFLAGS"
SRC=$HERE/ingenic-audio-test.c
if [ $SOC = t41 ]; then
    python3 - "$IMAGE" "$WORK" <<'PY'
import struct, sys
d = open(sys.argv[1], 'rb').read()
n = struct.unpack('>I', d[0x40000 + 12:0x40000 + 16])[0]
open(sys.argv[2] + '/uImage', 'wb').write(d[0x40000:0x40000 + 64 + n])
open(sys.argv[2] + '/root.sq', 'wb').write(d[0x2C0000:0x4C0000])
PY
    unsquashfs -q -d "$WORK/rfs" "$WORK/root.sq" >/dev/null
    $CC -O2 -Wall -I"$SDK41/include" -static -s \
        -o "$WORK/rfs/usr/bin/audio-test" "$SRC" \
        "$SDK41/lib/uclibc/libimp.a" "$SDK41/lib/uclibc/libalog.a" \
        "$SDK41/lib/uclibc/libsysutils.a" -lpthread -lm -lrt -ldl -lstdc++ \
        2>&1 | grep -v -i warning || true
    # room for the static client in the 2 MiB root: the vendor libraries'
    # symbols, and Fortran's runtime, which nothing here uses
    rm -f "$WORK"/rfs/lib/libgfortran*
    "$TC/${CROSS}strip" --strip-unneeded "$WORK"/rfs/lib/lib*.so* 2>/dev/null || true
    # the SDK image's start.sh, without its application
    LOADER='mount -t jffs2 /dev/mtdblock3 /system
insmod /system/lib/modules/audio.ko'
    KERNEL=$WORK/uImage
    ROOTDEV=/dev/mtdblock4
    MTDPARTS="sfc0_nor:256k(boot),2560k(kernel),2048k(root),9472k(appfs),-(test)"
else
    unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.$SOC" >/dev/null
    $CC -O2 -Wall -march=mips32r2 -muclibc -I"$SDK/include" -DAUDIO_NO_DL \
        -static -s -o "$WORK/rfs/usr/bin/audio-test" "$SRC" \
        "$SDK/lib/uclibc/libimp.a" "$SDK/lib/uclibc/libalog.a" \
        -lpthread -lm -lrt -lstdc++
    LOADER='modprobe audio spk_gpio=-1 || modprobe audio'
    KERNEL=$FW/uImage.$SOC
    ROOTDEV=/dev/mtdblock3
    MTDPARTS="jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data)"
fi
cat > "$WORK/rfs/usr/bin/audio-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t tmpfs tmp /var
mkdir -p /dev/shm && mount -t tmpfs shm /dev/shm
mkdir -p /var/lock
$LOADER
echo "AUDIOTEST dsp: \$(ls /dev/dsp 2>&1)"
audio-test $RATE 500 $PLAY 2000 /tmp/rec.raw
echo "AUDIOREC BEGIN $RATE"
hexdump -v -e '16/2 "%04x " "\\n"' /tmp/rec.raw
echo "AUDIOREC END"
echo "AUDIOTEST dmesg:"
dmesg | grep -i -E "audio|dma|aic|codec|i2s" | tail -20
echo AUDIOTEST-END
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/audio-init"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
if [ $SOC = t41 ]; then
    python3 - "$WORK/nor.bin" "$IMAGE" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(open(sys.argv[2], 'rb').read())
r = open(sys.argv[3], 'rb').read()
assert len(r) <= 0x200000, 'the root does not fit in 2 MiB'
img[0xE00000:0xE00000 + len(r)] = r
open(sys.argv[1], 'wb').write(img)
PY
else
    python3 - "$WORK/nor.bin" "$FW/uImage.$SOC" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY
fi

echo "== boot ($SOC)"
rm -f "$OUT/console.log" "$OUT/play.wav"
"$QEMU" -M $SOC$VARIANT,flash-file="$WORK/nor.bin",audiodev=snd -m $RAM \
    -audiodev wav,id=snd,path="$OUT/play.wav" \
    -display none -serial file:"$OUT/console.log" -nic none \
    -kernel "$KERNEL" -global ingenic-aic.tone-hz=$TONE \
    -append "console=ttyS1,115200n8 root=$ROOTDEV rootfstype=squashfs init=/usr/bin/audio-init mtdparts=$MTDPARTS $MEM" &
QPID=$!
for i in $(seq "$TIMEOUT"); do
    grep -q AUDIOTEST-END "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 2
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
grep -a "AUDIOTEST" -A21 "$OUT/console.log" | grep -av "^[0-9a-f]\{4\} " || true

echo "== host"
python3 - "$OUT/console.log" "$OUT/play.wav" $TONE $PLAY <<'PY'
import struct, sys, wave
import numpy as np

log, wav, tone, play = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
fail = 0

def peak(x, rate):
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    f = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    return np.argmax(f) * rate / len(x), np.sqrt((x * x).mean())

def check(what, ok, detail):
    global fail
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, detail))
    fail |= not ok

lines = open(log, 'rb').read().decode('latin-1').replace('\r', '').split('\n')
try:
    a = next(i for i, l in enumerate(lines) if l.startswith('AUDIOREC BEGIN'))
    b = next(i for i, l in enumerate(lines) if l.startswith('AUDIOREC END'))
    rate = int(lines[a].split()[2])
    s = [int(w, 16) for l in lines[a + 1:b] for w in l.split()]
    s = [v - 65536 if v >= 32768 else v for v in s]
    f, rms = peak(s, rate)
    check('recorded %d samples at %d Hz' % (len(s), rate),
          len(s) >= rate // 4, '%d' % len(s))
    check('the recording is the %d Hz tone' % tone, abs(f - tone) < 10 and
          rms > 100, 'peak %.1f Hz, rms %.0f' % (f, rms))
except StopIteration:
    check('a recording on the console', False, 'none')
check('the client ran through', 'AUDIOTEST PASS' in lines, '')
try:
    w = wave.open(wav)
    rate, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
    d = np.frombuffer(w.readframes(n), dtype='<i2').reshape(-1, ch)[:, 0]
    loud = np.nonzero(np.abs(d) > 1000)[0]
    seg = d[loud[0]:loud[-1]] if len(loud) else d
    f, rms = peak(seg, rate) if len(seg) > 256 else (0, 0)
    check('the wav audiodev got %.2f s of sound at %d Hz' % (len(seg) / rate, rate),
          len(seg) > rate, '%d frames of %d' % (len(seg), n))
    check('the played sine is %d Hz' % play, abs(f - play) < 10,
          'peak %.1f Hz, rms %.0f' % (f, rms))
except (FileNotFoundError, wave.Error, EOFError) as e:
    check('a wav from the audiodev', False, str(e))
print('PASS' if not fail else 'FAIL')
sys.exit(fail)
PY
