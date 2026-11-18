#!/bin/bash
#
# The T40 NNA's DMA (ingenic-nndma), its 1 MiB ORAM and the XBurst2
# hardware registers (patch 0022) in a guest, through Ingenic's own code:
# OpenIPC's t40 kernel and root with its soc-nna.ko, and a static client
# (ingenic-nna-test.c) linked with dma_func.cc.o of libvenus.a from
# Ingenic's public magik-toolkit (InferenceKit/nna1, uclibc).  The client
# moves two blocks DDR -> ORAM with libvenus's w_d2o_orasetrun, back to
# DDR on a write channel, waits with rdhwr $9 / $11, and reads rdhwr $4
# after an LL.  The NNA's own instructions (custom COP2 opcodes) are not
# modelled, so no network runs: see docs/ingenic.md.
#
#   bash qemu-boot/test-ingenic-nna.sh [--magik DIR] [--fw DIR]
#        [--qemu PATH] [--out DIR] [--timeout 300]
#
# --magik is a clone of gitee.com/ingenic_1_0/magik-toolkit; without it
# (or without the T40 firmware) the test skips, exit 0.
#
# Copyright (c) 2026 OpenIPC.
#
# SPDX-License-Identifier: GPL-2.0-or-later

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
QEMU=$TOP/qemu-src/build/qemu-system-mipsel
FW=/home/ai/dimerr/local/ingenic/fw/t40
MAGIK=/home/ai/dimerr/tmp-scratch/magik-gitee
TC=/home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8/bin
OUT=
TIMEOUT=300
DDR=0x07000000                  # above mem=96M: free for the client

while [ $# -gt 0 ]; do
    case $1 in
    --magik) MAGIK=$2; shift ;;
    --fw) FW=$2; shift ;;
    --qemu) QEMU=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TIMEOUT=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
LIBVENUS=$MAGIK/InferenceKit/nna1/mips720-glibc229/lib/uclibc/libvenus.a
for f in "$LIBVENUS" "$FW/uImage.t40" "$FW/rootfs.squashfs.t40"; do
    [ -e "$f" ] || { echo "SKIP: no $f"; exit 0; }
done
for f in "$QEMU" "$TC/mips-linux-gnu-gcc"; do
    [ -e "$f" ] || { echo "missing $f" >&2; exit 2; }
done
WORK=$(mktemp -d)
trap 'kill $QPID 2>/dev/null || true; rm -rf "$WORK"' EXIT
OUT=${OUT:-$WORK/out}
mkdir -p "$OUT"

echo "== the client, with libvenus's dma_func"
(cd "$WORK" && "$TC/mips-linux-gnu-ar" x "$LIBVENUS" dma_func.cc.o)
"$TC/mips-linux-gnu-gcc" -muclibc -O2 -Wall -c -o "$WORK/nna-test.o" \
    "$HERE/ingenic-nna-test.c"
unsquashfs -q -d "$WORK/rfs" "$FW/rootfs.squashfs.t40" >/dev/null
"$TC/mips-linux-gnu-g++" -muclibc -fno-use-linker-plugin -static -s \
    -o "$WORK/rfs/usr/bin/nna-test" "$WORK/nna-test.o" \
    "$WORK/dma_func.cc.o" 2>&1 | grep -v warning || true
KO=$(cd "$WORK/rfs" && find lib -name soc-nna.ko | head -1)
[ -n "$KO" ] || { echo "SKIP: the root has no soc-nna.ko"; exit 0; }
cat > "$WORK/rfs/usr/bin/nna-init" <<EOF
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sys /sys
mount -t tmpfs tmp /tmp
mount -t devtmpfs dev /dev 2>/dev/null
insmod /$KO
echo "NNATEST dev: \$(ls /dev/soc-nna 2>&1)"
echo 1 > /proc/sys/kernel/printk
nna-test $DDR
echo NNATEST-END
# the UART sends at the line rate: let the console drain (the host stops
# QEMU at the end marker)
sleep 60
poweroff -f
EOF
chmod +x "$WORK/rfs/usr/bin/nna-init"
mksquashfs "$WORK/rfs" "$WORK/rootfs.squashfs" -comp xz -noappend -quiet \
    -all-root >/dev/null
python3 - "$WORK/nor.bin" "$FW/uImage.t40" "$WORK/rootfs.squashfs" <<'PY'
import sys
img = bytearray(b'\xff' * (16 << 20))
for path, off in ((sys.argv[2], 0x50000), (sys.argv[3], 0x250000)):
    data = open(path, 'rb').read()
    img[off:off + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

echo "== boot (t40)"
"$QEMU" -M t40,flash-file="$WORK/nor.bin" -m 128M \
    -display none -serial file:"$OUT/console.log" -nic none \
    -kernel "$FW/uImage.t40" \
    -append "console=ttyS1,115200n8 root=/dev/mtdblock3 rootfstype=squashfs init=/usr/bin/nna-init mtdparts=jz_sfc:256k(boot),64k(env),2048k(kernel),8192k(rootfs),-(rootfs_data) mem=96M" &
QPID=$!
for i in $(seq "$TIMEOUT"); do
    grep -q NNATEST-END "$OUT/console.log" 2>/dev/null && break
    kill -0 $QPID 2>/dev/null || break
    sleep 1
done
sleep 1
kill $QPID 2>/dev/null || true
wait $QPID 2>/dev/null || true
grep -a "NNATEST" "$OUT/console.log" | tr -d '\r' || true

echo "== host"
python3 - "$OUT/console.log" "$DDR" <<'PY'
import re, sys
log = open(sys.argv[1], 'rb').read().decode('latin-1').replace('\r', '')
ddr = int(sys.argv[2], 16)
fail = 0

def check(what, ok, detail=''):
    global fail
    print('  %s: %s%s' % ('PASS' if ok else 'FAIL', what,
                          ' (%s)' % detail if detail else ''))
    fail |= not ok

check('soc-nna.ko gives /dev/soc-nna', 'NNATEST dev: /dev/soc-nna' in log)
m = re.search(r'NNATEST version ioctl (-?\d+): ([0-9a-f]+)', log)
check('the soc-nna version ioctl', bool(m) and m.group(1) == '0',
      m.group(2) if m else 'no line')
check("DDR -> ORAM by libvenus's w_d2o_orasetrun, waited with rdhwr $9",
      'NNATEST d2o: ok' in log)
check('ORAM -> DDR on write channel 0, waited with rdhwr $11',
      'NNATEST o2d: ok' in log)
m = re.search(r'NNATEST rdhwr4: ([0-9a-f]+) want ([0-9a-f]+)', log)
check('rdhwr $4 after an LL is its physical address',
      bool(m) and m.group(1) == m.group(2) == '%08x' % (ddr + 0x80),
      m.group(0) if m else 'no line')
check('the client ran through', 'NNATEST done' in log)
print('PASS' if not fail else 'FAIL')
sys.exit(fail)
PY
