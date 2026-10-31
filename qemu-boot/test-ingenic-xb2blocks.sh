#!/bin/bash
# The T40/T41 DRAWBOX and I2D models under their vendor kernel drivers: a
# T40 or T41 SDK flash image boots as it is (SPL, U-Boot, the SDK kernel
# with ingenic_drawbox and ingenic_i2d built in), and rcS runs a client
# (qemu-boot/ingenic-xb2blocks-test.c) before the application: boxes drawn
# through /dev/dbox and a picture rotated through /dev/i2d, in reserved
# memory, checked in the guest.
#
#   bash qemu-boot/test-ingenic-xb2blocks.sh --image SDK_IMG [--soc t40|t41]
#        [--qemu PATH] [--out DIR] [--timeout S]
#
# The client is built static with the Ingenic gcc 7.2 toolchain against
# uClibc (it runs on the glibc images too).  Without --image (or with a
# missing one) the test skips, exit 0.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
TC=/home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8/bin
IMAGE=
SOC=
OUT=
TIMEOUT=400

while [ $# -gt 0 ]; do
    case $1 in
    --image) IMAGE=$2; shift ;;
    --soc) SOC=$2; shift ;;
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
case ${SOC:-$(basename "$IMAGE")} in
t40|T40*) SOC=t40; VARIANT=t40n; DEF= ;;
t41|T41*) SOC=t41; VARIANT=t41nq; DEF=-DI2D_H_ALIGN ;;
*) echo "cannot tell the SoC of $IMAGE (--soc)" >&2; exit 2 ;;
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

"$TC/mips-linux-gnu-gcc" -muclibc -fno-use-linker-plugin -O2 -Wall $DEF \
    -static -s -o "$WORK/xb2blocks-test" "$HERE/ingenic-xb2blocks-test.c"
cp "$IMAGE" "$WORK/flash.bin"
dd if="$WORK/flash.bin" of="$WORK/root.sq" bs=64k skip=$((ROOTOFF >> 16)) \
    count=$((ROOTSIZE >> 16)) status=none
unsquashfs -q -d "$WORK/rfs" "$WORK/root.sq" >/dev/null 2>&1
cp "$WORK/xb2blocks-test" "$WORK/rfs/usr/bin/"
python3 - "$WORK/rfs/etc/init.d/rcS" "$PHYS" <<'PY'
import sys
p, phys = sys.argv[1:3]
s = open(p).read()
old = '/system/init/app_init.sh &'
if old not in s:
    sys.exit('rcS: no app_init.sh')
s = s.replace(old, """# qemu: the DRAWBOX / I2D test instead of the application
/usr/bin/xb2blocks-test %s
echo XB2TEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
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
    grep -aq 'XB2TEST-END' "$LOG" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
QPID=

grep -a '^XB2TEST [PF]' "$LOG" | sed 's/^XB2TEST /  /' || true
grep -a -i 'dbox\|i2d' "$LOG" | grep -av '^XB2TEST' | sed 's/^/  log: /' | tail -5
if grep -aq '^XB2TEST DONE' "$LOG" && ! grep -aq '^XB2TEST FAIL' "$LOG"; then
    echo PASS
else
    echo FAIL
    exit 1
fi
