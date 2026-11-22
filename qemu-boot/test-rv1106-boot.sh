#!/bin/bash
# An RV1103 / RV1106 camera's whole SPI NOR image, booted as the mask ROM
# does: the ID block at 0x10000, the vendor SPL at DRAM 0 (the DDR blob is
# skipped, its ATAGs recreated), then either of the two vendor chains:
#
#   stock         SPL -> U-Boot FIT (sha256 on the crypto engine) ->
#                 U-Boot -> kernel FIT -> Linux
#   thunder boot  SPL preloads the kernel (gzip, on the hardware
#                 decompressor) and jumps to it: "Jumping to Kernel"
#
# and Linux up to busybox init running rcS and getty.  Every stage has
# to print its milestone in order.  (Vendor images set a root password,
# and their apps flood the console, so no login is attempted.)
#
# Xiongmai firmware keeps the kernel console off unless its U-Boot
# environment says xmuart=0, so for an XM image (its SPL and U-Boot carry
# XM's SPI NOR protection code, "XmSpiNor") the test adds that to the copy's environment (flash 0, 64 KiB,
# CRC32 first).  The image is copied first: the firmware writes its
# settings back to the flash, and the original is left as it was.
#
# The image is vendor code and not in CI -- a camera dump:
#
#   bash qemu-boot/test-rv1106-boot.sh --flash full.bin
#
# An RV1103B / RV1106B image (the newer die) boots the same way on its own
# machine: --machine rv1103b.
#
# An RV1126 / RV1109 camera boots from SPI NAND: --machine rv1109 (or
# rv1126) with the image qemu-boot/rv1126-nand-image.py rebuilds from a
# NAND dump.  Its chain is the machine's SPL stand-in, OP-TEE, U-Boot
# (Android boot image), Linux and buildroot's init; the milestones are
# those instead.
#
# An OpenIPC release boots from SPI NAND on the rv1106 machine with
# -kernel (no vendor loader): --openipc DIR, the directory
# qemu-boot/rv1106-openipc-image.py writes.  Its milestones run to the
# login prompt, with user networking for its DHCP.
#
#   [--machine rv1106] [--qemu ./qemu-src/build/qemu-system-arm]
#   [--timeout 180] [--log console.log]
set -euo pipefail

FLASH=
OPENIPC=
MACHINE=rv1106
QEMU=./qemu-src/build/qemu-system-arm
TIMEOUT=180
LOG=
while [ $# -gt 0 ]; do
    case "$1" in
        --flash) FLASH=$2; shift 2 ;;
        --openipc) OPENIPC=$(realpath "$2"); FLASH=$OPENIPC/nand.bin
                   shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        --log) LOG=$(realpath "$2"); shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ ! -f "$FLASH" ]; then
    echo "need --flash <image>: an RV1103/RV1106 SPI NOR dump (RKNS at 0x10000)" >&2
    exit 2
fi
case "$MACHINE" in
    rv1126|rv1109) FAMILY=rv1126 ;;
    *) FAMILY=rv1106 ;;
esac
[ -n "$OPENIPC" ] && FAMILY=openipc

WORK=$(mktemp -d)
QPID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null
    rm -rf "$WORK"
}
trap cleanup EXIT

cp "$FLASH" "$WORK/flash.bin"

[ "$FAMILY" = rv1106 ] && python3 - "$WORK/flash.bin" <<'PY'
import struct, sys, zlib

path = sys.argv[1]
d = bytearray(open(path, 'rb').read())
if d[0x10000:0x10004] != b'RKNS':
    sys.exit('no RKNS ID block at 0x10000: not an RV1103/RV1106 NOR image')
if b'XmSpiNor' not in d[:0x200000]:
    print('note this is not a Xiongmai image, environment left alone')
    sys.exit(0)
SIZE = 0x10000
env = bytes(d[4:SIZE])
if struct.unpack('<I', d[:4])[0] != zlib.crc32(env):
    sys.exit('Xiongmai image, but no valid U-Boot environment at flash 0')
end = env.find(b'\0\0')
kv = [v for v in env[:end].split(b'\0')
      if v and not v.startswith(b'xmuart=')]
kv.append(b'xmuart=0')
blob = (b'\0'.join(kv) + b'\0\0').ljust(SIZE - 4, b'\0')
d[4:SIZE] = blob
d[0:4] = struct.pack('<I', zlib.crc32(blob))
open(path, 'wb').write(d)
print('note Xiongmai image: xmuart=0 added to the environment copy')
PY

if [ "$FAMILY" = openipc ]; then
    "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin,flash-type=w25n01gv" \
        -kernel "$OPENIPC/zImage" -dtb "$OPENIPC/rv1106.dtb" \
        -append "$(cat "$OPENIPC/cmdline")" -nographic \
        -monitor none -serial "unix:$WORK/ser.sock,server=on,wait=on" \
        -nic user &
else
    "$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -nographic \
        -monitor none -serial "unix:$WORK/ser.sock,server=on,wait=on" \
        -nic none &
fi
QPID=$!

set +e
python3 - "$WORK" "$TIMEOUT" "$FAMILY" <<'PY'
import os, re, socket, sys, time

work, limit, family = sys.argv[1], float(sys.argv[2]), sys.argv[3]
os.chdir(work)              # sun_path is short; the socket name is not
for _ in range(100):
    if os.path.exists('ser.sock'):
        break
    time.sleep(0.1)
s = socket.socket(socket.AF_UNIX)
s.connect('ser.sock')
s.settimeout(0.5)

steps = [
    (b'U-Boot SPL 20', 'SPL runs from DRAM'),
    (re.compile(rb'Trying to boot from (?:MTD2|SPI)'),
     'SPL reads the NOR through the SFC'),
    ((b'Jumping to Kernel',                         # thunder boot
      b'Starting kernel'),                          # SPL -> U-Boot -> kernel
     'kernel loaded and verified'),
    (b"'/etc/init.d/rcS", 'init runs rcS from the NOR root'),
    (b'/sbin/getty', 'init started getty'),
]
if family == 'rv1126':
    steps = [
        (b'OP-TEE version', 'OP-TEE runs in the secure world'),
        (b'U-Boot 20', 'OP-TEE returns to U-Boot'),
        (b'optee api revision', 'U-Boot talks to OP-TEE'),
        (b'Starting kernel', 'kernel loaded from the boot partition'),
        (b'SMP: Total of', 'secondary core up through OP-TEE\'s PSCI'),
        (b'UBIFS (ubi0:0): UBIFS: mounted', 'rootfs mounted from UBI'),
        (b'Run /sbin/init', 'init runs'),
        (b'Starting logging', 'init scripts run'),
    ]
if family == 'openipc':
    steps = [
        (b'Linux version', 'Linux starts in the secure world'),
        (b'Winbond SPI NAND was found', 'SPI NAND on the SFC'),
        (b'ubi0: attached mtd3', 'UBI attached'),
        (b'VFS: Mounted root (squashfs', 'rootfs mounted from ubiblock'),
        (b'Run /sbin/init', 'init runs'),
        (b'Starting network', 'rcS passes fw_printenv'),
        (b'Starting majestic', 'majestic started'),
        (b'login:', 'getty prompts for a login'),
    ]


def find(buf, pats, at):
    """End of the earliest match at or after at, or -1."""
    best = None
    for p in (pats if isinstance(pats, tuple) else (pats,)):
        if isinstance(p, bytes):
            k = buf.find(p, at)
            hit = (k, k + len(p)) if k >= 0 else None
        else:
            m = p.search(buf, at)
            hit = (m.start(), m.end()) if m else None
        if hit and (best is None or hit < best):
            best = hit
    return best[1] if best else -1


buf, t0, i, at = b'', time.time(), 0, 0
while time.time() - t0 < limit:
    try:
        d = s.recv(4096)
        if not d:
            break
        buf += d
    except socket.timeout:
        pass
    while i < len(steps):
        end = find(buf, steps[i][0], at)
        if end < 0:
            break
        at = end
        print('ok   %-45s %6.1fs' % (steps[i][1], time.time() - t0))
        i += 1
    if i == len(steps):
        break

open('console.log', 'wb').write(buf)
if i < len(steps):
    print('FAIL %s (no %r)' % (steps[i][1], steps[i][0]))
    print(buf[-3000:].decode('latin-1'))
    sys.exit(1)
print('PASS')
PY
rc=$?
[ -n "$LOG" ] && cp "$WORK/console.log" "$LOG"
exit $rc
