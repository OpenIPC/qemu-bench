#!/bin/bash
# The SPI NAND of the NT98566 boards, under the vendor's own software: its
# U-Boot erases, programs and reads back the NAND, and its Linux keeps a
# file on the UBIFS root across a reboot.
#
# The vendor firmware is not in CI: the SDK's 566_NAND FW98566A.bin goes
# through qemu-boot/nvtpack2flash.py to a 128 MiB image (the OpenIPC
# nt9856x kernels are built without the SPI NAND driver):
#
#   python3 qemu-boot/nvtpack2flash.py FW98566A.bin nand.bin
#   bash qemu-boot/test-novatek-nand.sh --flash nand.bin
#
# What it checks:
#   - U-Boot ("nvt spinand", W25N01GV) at its prompt -- reached as in
#     test-novatek-uboot-net.sh, a key waiting before the CPU starts --
#     takes 256 KiB by TFTP, erases two blocks past the partitions, writes
#     it there and reads it back, and its crc32 matches the host's;
#   - nvt_boot (its bootcmd; this U-Boot has no "run") brings up the
#     kernel from NAND, which attaches the UBI rootfs and mounts UBIFS
#     read-write; a file written there and synced is
#     still there, same md5, after "reboot -f" and a second autoboot;
#   - the image file holds what U-Boot wrote, at the offset it wrote it.
#
# The image is copied first; the original is left as it was.
#
#   [--machine nt98566] [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

FLASH=
MACHINE=nt98566
QEMU=./qemu-src/build/qemu-system-arm
while [ $# -gt 0 ]; do
    case "$1" in
        --flash) FLASH=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ ! -f "$FLASH" ]; then
    echo "need --flash <image>: a Novatek SPI NAND image with the vendor" \
         "firmware" >&2
    exit 2
fi

WORK=$(mktemp -d)
QPID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null
    rm -rf "$WORK"
}
trap cleanup EXIT

cp "$FLASH" "$WORK/flash.bin"
mkdir "$WORK/tftp"
head -c 262144 /dev/urandom > "$WORK/tftp/blob.bin"
CRC=$(python3 -c "import zlib, sys; print('%08x' % zlib.crc32(open(sys.argv[1], 'rb').read()))" \
      "$WORK/tftp/blob.bin")

"$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -nographic -S \
    -monitor "unix:$WORK/mon.sock,server=on,wait=off" \
    -serial "unix:$WORK/ser.sock,server=on,wait=off" \
    -nic "user,restrict=on,tftp=$WORK/tftp" &
QPID=$!

set +e
python3 - "$WORK" "$CRC" <<'PY'
import re, socket, sys, time

work, crc = sys.argv[1], sys.argv[2]
for _ in range(100):
    try:
        ser = socket.socket(socket.AF_UNIX)
        ser.connect(work + '/ser.sock')
        mon = socket.socket(socket.AF_UNIX)
        mon.connect(work + '/mon.sock')
        break
    except OSError:
        time.sleep(0.05)
else:
    sys.exit('QEMU did not come up')

ser.settimeout(0.2)
log = b''
pos = 0                             # where the next search starts


def until(pattern, secs):
    """Read until pattern (bytes or a compiled regex) shows up past the
    last match, or time out; return the match (True for bytes)."""
    global log, pos
    end = time.time() + secs
    while True:
        if isinstance(pattern, bytes):
            i = log.find(pattern, pos)
            if i >= 0:
                pos = i + len(pattern)
                return True
        else:
            m = pattern.search(log, pos)
            if m:
                pos = m.end()
                return m
        if time.time() >= end:
            return None
        try:
            log += ser.recv(65536)
        except socket.timeout:
            pass


PROMPT = re.compile(rb'(=>|#|\$|:) $')


def prompt(secs):
    """Wait for a prompt ending the output ("nvt@na51089: ", "...$ ")."""
    global log, pos
    end = time.time() + secs
    while time.time() < end:
        if PROMPT.search(log.split(b'\n')[-1]):
            pos = len(log)
            return True
        try:
            log += ser.recv(65536)
        except socket.timeout:
            pass
    return False


def uboot(cmd, expect, secs=30):
    ser.sendall(cmd.encode() + b'\r')
    ok = until(expect, secs)
    prompt(10)
    return bool(ok)


MD5 = re.compile(rb'MD5=([0-9a-f]{32})')
SHELL = b'NVTEVM Linux shell'

ser.sendall(b' ' * 16)              # waiting at the autoboot check
mon.sendall(b'cont\n')
results = []
try:
    results.append(('U-Boot finds the SPI NAND',
                    bool(until(b'nvt spinand', 30))))
    until(b'Hit any key', 30)
    ser.sendall(b'\r')
    results.append(('autoboot stopped at the U-Boot prompt',
                    prompt(10) and b'Starting kernel' not in log))
    results.append(('TFTP 256 KiB',
                    uboot('dhcp 0x1000000 blob.bin',
                          b'Bytes transferred = 262144', 60)))
    results.append(('nand erase', uboot('nand erase 0x4000000 0x40000',
                                        b'OK')))
    results.append(('nand write', uboot('nand write 0x1000000 0x4000000 '
                                        '0x40000', b'written: OK')))
    results.append(('nand read', uboot('nand read 0x2000000 0x4000000 '
                                       '0x40000', b'read: OK')))
    results.append(('crc32 of the read-back matches the host (%s)' % crc,
                    uboot('crc32 0x2000000 0x40000',
                          ('==> ' + crc).encode())))

    def step(what, ok):
        results.append((what, bool(ok)))
        if not ok:
            raise StopIteration     # the rest depends on it

    try:
        ser.sendall(b'nvt_boot\r')
        step('UBIFS root mounted from NAND',
             until(b'Mounted root (ubifs filesystem)', 120))
        step('vendor shell', until(SHELL, 120))
        prompt(20)
        # an MD5 only for the whole 256 KiB, synced
        ser.sendall(b'dd if=/dev/urandom of=/nandtest bs=1024 count=256 '
                    b'2>/dev/null && [ $(wc -c < /nandtest) = 262144 ] && '
                    b'sync && echo MD5=$(md5sum < /nandtest)\r')
        m = until(MD5, 60)
        written = m.group(1) if m else None
        step('a file written and synced to UBIFS', written)
        prompt(10)
        ser.sendall(b'reboot -f\r')
        step('reboot reaches U-Boot again', until(b'U-Boot 2019.04', 60))
        step('and Linux autoboots to the shell', until(SHELL, 180))
        prompt(20)
        ser.sendall(b'echo MD5=$(md5sum < /nandtest)\r')
        m = until(MD5, 30)
        step('the file is there after the reboot, same md5',
             m and m.group(1) == written)
        # (a full image leaves UBI no PEB for a fastmap anchor; harmless)
        step('no UBI/UBIFS corruption reported',
             not re.search(rb'(bad|corrupted) PEBs: [1-9]|'
                           rb'ubi0 error: (?!.*anchor)|UBIFS error', log))
    except StopIteration:
        pass
finally:
    mon.sendall(b'quit\n')

sys.stdout.write(log.decode('latin1'))
print()
fail = 0
for what, ok in results:
    print('%s: %s' % ('PASS' if ok else 'FAIL', what))
    fail |= not ok
sys.exit(fail)
PY
rc=$?
set -e
# Python quits QEMU through the monitor; if it died before that, do it here
# rather than wait on a QEMU still paused under -S.
for _ in $(seq 50); do
    kill -0 "$QPID" 2>/dev/null || break
    sleep 0.1
done
kill "$QPID" 2>/dev/null || true
wait "$QPID" 2>/dev/null || true
QPID=

# What U-Boot wrote at 0x4000000 is in the image file (data only: the
# offset is the file's).
if [ "$rc" = 0 ]; then
    if cmp -s -n 262144 -i 67108864:0 "$WORK/flash.bin" \
           "$WORK/tftp/blob.bin"; then
        echo "PASS: the image file holds U-Boot's write at 0x4000000"
    else
        echo "FAIL: the image file holds U-Boot's write at 0x4000000"
        rc=1
    fi
fi
exit $rc
