#!/bin/bash
# Networking from the vendor U-Boot on emulated NT9856x: stop autoboot, then
# take a DHCP lease and TFTP 1 MiB from the user network, ping its gateway,
# and compare U-Boot's crc32 of what landed with the host's.
#
# The vendor U-Boot (eth_na51089.c over nvt-eqos) comes from a whole flash
# image -- the SDK's FW98562A.bin through qemu-boot/nvtpack2flash.py, or a
# camera dump -- which is vendor code and not in CI:
#
#   python3 qemu-boot/nvtpack2flash.py FW98562A.bin nor.bin
#   bash qemu-boot/test-novatek-uboot-net.sh --flash nor.bin
#
# That U-Boot has bootdelay 0 and its environment nowhere, so the one way in
# is a key already waiting at "Hit any key to stop autoboot": QEMU starts
# paused with the serial line on a socket, a few spaces are queued, and only
# then does the CPU run (qemu/patches/0003 keeps that input flowing across
# U-Boot's UART init).  The image is copied first; the original is left as
# it was.
#
#   [--machine nt98562] [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

FLASH=
MACHINE=nt98562
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
    echo "need --flash <image>: a Novatek NOR image with the vendor U-Boot" >&2
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
head -c 1048576 /dev/urandom > "$WORK/tftp/blob.bin"
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


def until(pattern, secs):
    """Read until pattern shows up after the current point, or time out."""
    global log
    start = len(log)
    end = time.time() + secs
    while time.time() < end:
        if pattern in log[start:]:
            return True
        try:
            log += ser.recv(65536)
        except socket.timeout:
            pass
    return pattern in log[start:]


PROMPT = re.compile(rb'(=>|#|\$|:) $')


def prompt(secs):
    """Wait for a U-Boot prompt ending the output ("nvt@na51089: ", "=> ")."""
    global log
    end = time.time() + secs
    while time.time() < end:
        if PROMPT.search(log.split(b'\n')[-1]):
            return True
        try:
            log += ser.recv(65536)
        except socket.timeout:
            pass
    return False


ser.sendall(b' ' * 16)              # waiting at the autoboot check
mon.sendall(b'cont\n')
results = []
until(b'Hit any key', 30)
ser.sendall(b'\r')
results.append(('autoboot stopped at the U-Boot prompt',
                prompt(10) and b'Starting kernel' not in log))
# dhcp first: the built-in ipaddr (192.168.1.99) is not on SLIRP's net
ser.sendall(b'dhcp 0x1000000 blob.bin\r')
results.append(('DHCP lease', until(b'DHCP client bound', 20)))
results.append(('TFTP 1 MiB', until(b'Bytes transferred = 1048576', 60)))
prompt(10)
ser.sendall(b'ping 10.0.2.2\r')
results.append(('ping the gateway', until(b'is alive', 20)))
prompt(10)
ser.sendall(b'crc32 0x1000000 0x100000\r')
results.append(('crc32 matches the host (%s)' % crc,
                until(('==> ' + crc).encode(), 20)))
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
wait "$QPID" 2>/dev/null || true
QPID=
exit $rc
