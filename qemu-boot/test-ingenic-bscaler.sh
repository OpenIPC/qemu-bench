#!/bin/bash
# T40's BSCALER model under Ingenic's BSCALER code and the vendor kernel
# driver: a T40 SDK flash image boots as it is (SPL, U-Boot, the SDK
# kernel with ingenic_bscaler built in), and rcS runs a client (qemu-boot/
# ingenic-bscaler-test.c) before the application: resizes (BGRA, NV12,
# NV12 to BGRA / RGBA / ARGB, a many-tile upscale), affine / perspective
# warps and NV12 to colour by lines through the library, in reserved
# memory, checked in the guest.
#
#   bash qemu-boot/test-ingenic-bscaler.sh --image T40_SDK_IMG [--magik DIR]
#        [--qemu PATH] [--out DIR] [--timeout S]
#
# --magik is a checkout of Ingenic's magik-toolkit (gitee.com/ingenic_1_0/
# magik-toolkit), whose InferenceKit/nna1 ships libvenus.a; the client
# links its bscaler_api, bscaler_hal, bscaler_segmentation, bscaler_wrap
# and matrix objects, standing in for the rest.  It is built static with
# the Ingenic gcc 7.2 toolchain against uClibc.  Without --image or the
# toolkit the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
TC=/home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8/bin
IMAGE=
MAGIK=/home/ai/dimerr/tmp-scratch/magik-gitee
SOC=
OUT=
TIMEOUT=400

while [ $# -gt 0 ]; do
    case $1 in
    --image) IMAGE=$2; shift ;;
    --soc) SOC=$2; shift ;;
    --magik) MAGIK=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
if [ -z "$IMAGE" ] || [ ! -r "$IMAGE" ]; then
    echo "SKIP: no SDK image (--image)"
    exit 0
fi
VENUS=$MAGIK/InferenceKit/nna1/mips720-glibc229/lib/uclibc/libvenus.a
if [ ! -r "$VENUS" ]; then
    echo "SKIP: no nna1 libvenus.a (--magik)"
    exit 0
fi
case ${SOC:-$(basename "$IMAGE")} in
t40|T40*) SOC=t40; VARIANT=t40n ;;
*) echo "the BSCALER is T40's ($IMAGE; --soc t40)" >&2; exit 2 ;;
esac
for f in "$QEMU" "$TC/mips-linux-gnu-gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
QPID=
trap '[ -n "$QPID" ] && kill $QPID 2>/dev/null; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"
LOG=$OUT/serial.log

BOOTARGS=$(strings -n 8 "$IMAGE" | grep -m1 '^bootargs=' | sed 's/^bootargs=//')
# the root partition, the DRAM size, and 4 MiB at the top of rmem
read -r ROOTOFF ROOTSIZE RAM PHYS <<< "$(python3 - "$BOOTARGS" <<'PY'
import re, sys
a = sys.argv[1]
m = re.search(r'mtdparts=[^:]*:(\S+)', a)
off, root = 0, None
for p in m.group(1).split(','):
    size, name = re.match(r'(-|\d+[kKmM])\((\w+)\)', p).groups()
    n = int(size[:-1]) << (10 if size[-1] in 'kK' else 20) if size != '-' else 0
    if name == 'root':
        root = (off, n)
        break
    off += n
top = 0
for size, base in re.findall(r'\b[nr]?mem=(\d+)M(?:@(0x[0-9a-fA-F]+))?', a):
    top = max(top, int(base or '0', 16) + (int(size) << 20))
r = re.search(r'\brmem=(\d+)M@(0x[0-9a-fA-F]+)', a)
phys = int(r.group(2), 16) + (int(r.group(1)) << 20) - (4 << 20)
ram = 64
while (ram << 20) < top:
    ram *= 2
print(root[0], root[1], '%dM' % ram, hex(phys))
PY
)"
echo "== $(basename "$IMAGE"): $SOC, root at $ROOTOFF (+$ROOTSIZE), -m $RAM, buffer $PHYS"

OBJS="bscaler_api.cpp.o bscaler_hal.c.o bscaler_segmentation.cpp.o \
    bscaler_wrap.c.o matrix.c.o"
mkdir -p "$WORK/obj"
(cd "$WORK/obj" && ar x "$VENUS" $OBJS)
"$TC/mips-linux-gnu-gcc" -muclibc -O2 -Wall -c -o "$WORK/client.o" \
    "$HERE/ingenic-bscaler-test.c"
# shellcheck disable=SC2086  # the object list
"$TC/mips-linux-gnu-g++" -muclibc -fno-use-linker-plugin -static -s \
    -o "$WORK/bscaler-test" "$WORK/client.o" \
    $(for o in $OBJS; do echo "$WORK/obj/$o"; done) -lpthread -lm
cp "$IMAGE" "$WORK/flash.bin"
dd if="$WORK/flash.bin" of="$WORK/root.sq" bs=64k skip=$((ROOTOFF >> 16)) \
    count=$((ROOTSIZE >> 16)) status=none
unsquashfs -q -d "$WORK/rfs" "$WORK/root.sq" >/dev/null 2>&1
cp "$WORK/bscaler-test" "$WORK/rfs/usr/bin/"
# room for the client: the image's unused Fortran runtime goes
rm -f "$WORK"/rfs/lib/libgfortran*
python3 - "$WORK/rfs/etc/init.d/rcS" "$PHYS" <<'PY'
import sys
p, phys = sys.argv[1:3]
s = open(p).read()
old = '/system/init/app_init.sh &'
if old not in s:
    sys.exit('rcS: no app_init.sh')
s = s.replace(old, """# qemu: the BSCALER test instead of the application
/usr/bin/bscaler-test %s
echo BSCTEST-END
poweroff -f
""" % phys, 1)
open(p, 'w').write(s)
PY
mksquashfs "$WORK/rfs" "$WORK/root.new" -comp xz -noappend -quiet -all-root \
    >/dev/null
python3 - "$WORK/flash.bin" "$WORK/root.new" "$ROOTOFF" "$ROOTSIZE" <<'PY'
import sys
img = bytearray(open(sys.argv[1], 'rb').read())
r = open(sys.argv[2], 'rb').read()
off, size = int(sys.argv[3]), int(sys.argv[4])
assert len(r) <= size, 'the rebuilt root does not fit'
img[off:off + size] = r + b'\xff' * (size - len(r))
open(sys.argv[1], 'wb').write(img)
PY

"$QEMU" -M "$SOC,variant=$VARIANT,flash-file=$WORK/flash.bin" -m "$RAM" \
    -display none -monitor none -serial "file:$LOG" -nic none \
    > "$OUT/qemu.err" 2>&1 &
QPID=$!
for i in $(seq "$TIMEOUT"); do
    grep -aq 'BSCTEST-END' "$LOG" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
QPID=

grep -a '^BSCTEST [PF]' "$LOG" | sed 's/^BSCTEST /  /' || true
grep -a -i 'bscaler' "$LOG" | grep -av '^BSCTEST' | sed 's/^/  log: /' | tail -5
if grep -aq '^BSCTEST DONE' "$LOG" && ! grep -aq '^BSCTEST FAIL' "$LOG"; then
    echo PASS
else
    echo FAIL
    exit 1
fi
