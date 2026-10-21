#!/bin/bash
# The vendor U-Boot on emulated NT9856x, networking and SD card: stop
# autoboot, then take a DHCP lease and TFTP 1 MiB from the user network,
# ping its gateway, and compare U-Boot's crc32 of what landed with the
# host's; find the SD card (nvt_ivot_mmc.c over nvt-sdio, plain DMA), read
# its partition table, read 256 KiB the host put on it with a matching
# crc32, and write them back elsewhere on the card, where the host finds
# them.
#
# The vendor U-Boot (eth_na51089.c over nvt-eqos) comes from a whole flash
# image -- the SDK's FW98562A.bin through qemu-boot/nvtpack2flash.py, or a
# camera dump -- which is vendor code and not in CI:
#
#   python3 qemu-boot/nvtpack2flash.py FW98562A.bin nor.bin
#   bash qemu-boot/test-novatek-uboot.sh --flash nor.bin
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
# A 64 MiB card: an MBR with one FAT partition, 256 KiB of host data at
# block 0x14000 (40 MiB).
SD_CRC=$(python3 - "$WORK/sd.img" <<'PY'
import os, struct, sys, zlib
img = bytearray(64 << 20)
img[446:462] = struct.pack('<B3sB3sII', 0, b'\0\0\0', 0x0C, b'\0\0\0', 2048, 30720)
img[510:512] = b'\x55\xaa'
data = os.urandom(256 << 10)
img[40 << 20:(40 << 20) + len(data)] = data
open(sys.argv[1], 'wb').write(img)
print('%08x' % zlib.crc32(data))
PY
)

"$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -nographic -S \
    -monitor "unix:$WORK/mon.sock,server=on,wait=off" \
    -serial "unix:$WORK/ser.sock,server=on,wait=off" \
    -nic "user,restrict=on,tftp=$WORK/tftp" \
    -drive "if=sd,index=0,format=raw,file=$WORK/sd.img" &
QPID=$!

set +e
python3 - "$WORK" "$CRC" "$SD_CRC" <<'PY'
import re, socket, sys, time

work, crc, sd_crc = sys.argv[1], sys.argv[2], sys.argv[3]
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
    last match, or time out."""
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
                return True
        if time.time() >= end:
            return False
        try:
            log += ser.recv(65536)
        except socket.timeout:
            pass


PROMPT = re.compile(rb'(=>|#|\$|:) $')


def prompt(secs):
    """Wait for a U-Boot prompt ending the output ("nvt@na51089: ", "=> ")."""
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


ser.sendall(b' ' * 16)              # waiting at the autoboot check
mon.sendall(b'cont\n')
results = []
try:
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
    prompt(10)
    ser.sendall(b'mmc rescan; mmc info\r')
    results.append(('SD card: 64 MiB, 4-bit',
                    until(b'Capacity: 64 MiB', 20) and
                    until(b'Bus Width: 4-bit', 5)))
    prompt(10)
    ser.sendall(b'mmc part\r')
    results.append(('its DOS partition table',
                    bool(until(re.compile(rb'1\s+2048\s+30720\s+\S+\s+0c'),
                               20))))
    prompt(10)
    ser.sendall(b'mmc read 0x1000000 0x14000 0x200\r')
    results.append(('mmc read of 256 KiB', until(b'512 blocks read: OK', 30)))
    prompt(10)
    ser.sendall(b'crc32 0x1000000 0x40000\r')
    results.append(('and its crc32 matches the host (%s)' % sd_crc,
                    until(('==> ' + sd_crc).encode(), 20)))
    prompt(10)
    ser.sendall(b'mmc write 0x1000000 0x18000 0x200\r')
    results.append(('mmc write of the same 256 KiB',
                    until(b'512 blocks written: OK', 30)))
    prompt(10)
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

# U-Boot's write at block 0x18000 (48 MiB) is in the card image.
if [ "$rc" = 0 ]; then
    if cmp -s -n 262144 -i 50331648:41943040 "$WORK/sd.img" "$WORK/sd.img"; then
        echo "PASS: the card image holds U-Boot's write at block 0x18000"
    else
        echo "FAIL: the card image holds U-Boot's write at block 0x18000"
        rc=1
    fi
fi
exit $rc
