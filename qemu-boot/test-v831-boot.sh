#!/bin/bash
# A V831 / V833 camera's whole SPI NOR image, booted as the mask ROM does:
# boot0 from flash offset 0, DRAM training, the boot package (OP-TEE,
# U-Boot, DTB) over SPI0 with DMA, U-Boot's environment and GPT from the
# flash, the kernel from the "boot" partition, the SquashFS root from
# mtdblock2 (the kernel prints at loglevel 4, so busybox init starting
# rcS is the sign), a shell that answers, and Ethernet: the camera ships
# Wi-Fi only, but the SoC's EMAC and its DT node are there, so with a
# -nic the vendor sunxi-gmac driver takes a DHCP lease and pings the
# gateway.  Every stage has to print its milestone in order.  (About
# 25 s in, the YI supervisor reboots the camera: its media process dies
# without a sensor, and its watch list says "reboot" for it.)
#
# The image is vendor code and not in CI -- a camera dump:
#
#   bash qemu-boot/test-v831-boot.sh --flash full.bin
#
# The Xiaoyi U-Boot moves the kernel console to a disabled UART (ttyS2)
# unless the SD card holds /debug_tty or /Factory -- its U-Boot has no
# setenv to say otherwise -- so the test boots with a card that has
# /debug_tty.  The card is
# built here, FAT16 with the one long-named file, as no FAT tools are
# assumed.  The image is copied first: the firmware writes its settings
# back to the flash, and the original is left as it was.
#
#   [--machine v831] [--qemu ./qemu-src/build/qemu-system-arm]
#   [--timeout 180]
set -euo pipefail

FLASH=
MACHINE=v831
QEMU=./qemu-src/build/qemu-system-arm
TIMEOUT=180
while [ $# -gt 0 ]; do
    case "$1" in
        --flash) FLASH=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --timeout) TIMEOUT=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ ! -f "$FLASH" ]; then
    echo "need --flash <image>: a V831/V833 SPI NOR dump (eGON.BT0 at 0)" >&2
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

# 64 MiB card: MBR, one 32 MiB FAT16 partition at 1 MiB, /debug_tty.
python3 - "$WORK/sd.img" <<'PY'
import struct, sys

SEC, SPC, RSV, NFAT, ROOTENT, TOTAL, START = 512, 4, 1, 2, 512, 65536, 2048
FATSZ = (TOTAL // SPC * 2 + SEC - 1) // SEC
ROOTSEC = ROOTENT * 32 // SEC
img = bytearray(64 << 20)
img[446:462] = struct.pack('<B3sB3sII', 0, b'\0\0\0', 0x06, b'\0\0\0',
                           START, TOTAL)
img[510:512] = b'\x55\xaa'

p = START * SEC
bs = bytearray(SEC)
bs[0:3] = b'\xeb\x3c\x90'
bs[3:11] = b'MSWIN4.1'
bs[11:36] = struct.pack('<HBHBHHBHHHII', SEC, SPC, RSV, NFAT, ROOTENT, 0,
                        0xF8, FATSZ, 63, 255, START, TOTAL)
bs[36:62] = struct.pack('<BBBI11s8s', 0x80, 0, 0x29, 0x5131, b'QEMU SD    ',
                        b'FAT16   ')
bs[510:512] = b'\x55\xaa'
img[p:p + SEC] = bs
for n in range(NFAT):           # clusters 0, 1 reserved; 2 = the file
    f = p + (RSV + n * FATSZ) * SEC
    img[f:f + 6] = struct.pack('<HHH', 0xFFF8, 0xFFFF, 0xFFFF)

short = b'DEBUG_~1   '
csum = 0
for c in short:
    csum = (((csum & 1) << 7) + (csum >> 1) + c) & 0xFF
name = 'debug_tty'.encode('utf-16-le') + b'\0\0'
name += b'\xff' * (26 - len(name))
lfn = (bytes([0x41]) + name[0:10] + bytes([0x0F, 0, csum]) + name[10:22] +
       b'\0\0' + name[22:26])
data = b'1\n'
ent = short + bytes([0x20]) + bytes(14) + struct.pack('<HI', 2, len(data))
root = p + (RSV + NFAT * FATSZ) * SEC
img[root:root + 64] = lfn + ent
dat = root + ROOTSEC * SEC
img[dat:dat + len(data)] = data
open(sys.argv[1], 'wb').write(img)
PY

"$QEMU" -M "$MACHINE,flash-file=$WORK/flash.bin" -nographic \
    -monitor none -serial "unix:$WORK/ser.sock,server=on,wait=on" \
    -drive "if=sd,index=0,format=raw,file=$WORK/sd.img" \
    -nic user,restrict=on &
QPID=$!

set +e
python3 - "$WORK" "$TIMEOUT" <<'PY'
import os, socket, sys, time

work, limit = sys.argv[1], float(sys.argv[2])
os.chdir(work)              # sun_path is short; the socket name is not
for _ in range(100):
    if os.path.exists('ser.sock'):
        break
    time.sleep(0.1)
s = socket.socket(socket.AF_UNIX)
s.connect('ser.sock')
s.settimeout(0.5)

steps = [
    (b'HELLO! BOOT0 is starting', 'boot0 runs from SRAM'),
    (b'DRAM simple test OK', 'boot0 trained DRAM'),
    (b'Jump to second Boot', 'boot0 loaded the boot package over SPI'),
    (b'OP-TEE version', 'OP-TEE runs'),
    (b'U-Boot 2018', 'U-Boot runs'),
    (b'Loading Environment from SUNXI_FLASH... OK', 'U-Boot env via SPI DMA'),
    (b'find /debug_tty', 'U-Boot read the SD card'),
    (b'Starting kernel', 'U-Boot loaded the kernel from the GPT'),
    (b"'/etc/init.d/rcS", 'init runs rcS from the NOR root'),
    (b'BusyBox', 'init started a shell'),
]
# After the boot milestones: commands whose answer is computed, so the
# echo of the command line itself cannot match.
probes = [
    (b'echo QEMU-$((6*7))', b'QEMU-42', 'the shell answers'),
    (b'ifconfig eth0 up; udhcpc -i eth0 -n -q -t 5 && '
     b'ifconfig eth0 10.0.2.15 netmask 255.255.255.0 && '
     b'ping -c 2 10.0.2.2 && echo NET-$((6*7))',
     b'NET-42', 'Ethernet: DHCP lease and ping'),
]
buf, t0, i, j, sent = b'', time.time(), 0, 0, None
while time.time() - t0 < limit:
    try:
        d = s.recv(4096)
        if not d:
            break
        buf += d
    except socket.timeout:
        pass
    while i < len(steps) and steps[i][0] in buf:
        print('ok   %-45s %6.1fs' % (steps[i][1], time.time() - t0))
        i += 1
    if i < len(steps):
        continue
    cmd, want, what = probes[j]
    if sent is None or time.time() - sent > 15:
        mark = len(buf)
        s.sendall(b'\r' + cmd + b'\r')
        sent = time.time()
    if want in buf[mark:]:
        print('ok   %-45s %6.1fs' % (what, time.time() - t0))
        j += 1
        sent = None
        if j == len(probes):
            sys.exit(0)
print('FAIL waiting for: %s' % (steps[i][1] if i < len(steps)
                                 else probes[j][2]))
print(buf[-3000:].decode('latin1'))
sys.exit(1)
PY
RC=$?
exit $RC
