#!/bin/bash
# Boot an OpenIPC Novatek NT9856x kernel to a PID-1 script and check what
# it found, then reboot.
#
# The release's rootfs is SquashFS, and the nt9856x kernel has no RAM disk
# driver, so -initrd cannot carry it; the busybox and libc from it go into
# a small initramfs instead, under an /init that reports:
#
#   - the device tree QEMU built (model, chip-ID node) as the kernel saw it;
#   - the A9 global timer as clocksource (QEMU's A9 is given r4p1, else
#     Linux refuses it);
#   - the vendor pinctrl (needs the pinmux children), GPIO, RTC and
#     watchdog drivers bound, the RTC reading the host's date;
#   - the SPI NOR (nvt-flash) under the vendor spinor driver: the release
#     laid out the OpenIPC way in a synthetic 8 MiB image, the rootfs
#     partition read back and checksummed, rootfs_data erased, a 192 KiB
#     payload flashcp'd onto it (DMA page programs) and 100 bytes dd'd at
#     256 KiB four at a time (PIO), all of which the host then finds byte
#     for byte in the image file, the rest still erased;
#   - "reboot -f": the kernel's reboot notifier arms the RTC power-on
#     alarm and powers off, which resets the machine, and -no-reboot turns
#     that into a clean exit.
#
#   bash qemu-boot/test-novatek-boot.sh --soc nt98562 \
#       [--machine nt98562] [--qemu ./qemu-src/build/qemu-system-arm] \
#       [--dir qemu-boot]
set -euo pipefail

SOC=nt98562
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
while [ $# -gt 0 ]; do
    case "$1" in
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
MACHINE=${MACHINE:-$SOC}
KERNEL=$DIR/uImage.$SOC
ROOTFS=$DIR/rootfs.squashfs.$SOC

if [ ! -f "$KERNEL" ] || [ ! -f "$ROOTFS" ]; then
    echo "need $KERNEL and $ROOTFS: unpack openipc.$SOC-nor-lite.tgz in $DIR" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap '[ -n "${KEEP:-}" ] || rm -rf "$WORK"' EXIT

# The OpenIPC 8 MiB NOR layout: 256k(boot),64k(env),2048k(kernel),
# 5120k(rootfs),-(rootfs_data).
MTDPARTS="spi_nor.0:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
NOR=$WORK/nor.bin
python3 - "$KERNEL" "$ROOTFS" "$NOR" "$WORK/payload.bin" "$WORK/pio.bin" <<'PY'
import os, sys
kernel, rootfs, nor, payload, pio = sys.argv[1:]
img = bytearray(b'\xff' * (8 << 20))
k = open(kernel, 'rb').read()
r = open(rootfs, 'rb').read()
img[0x50000:0x50000 + len(k)] = k
img[0x250000:0x250000 + len(r)] = r
open(nor, 'wb').write(img)
# 192 KiB, three erase blocks: flashcp programs whole 4 KiB buffers (it
# pads a short last one), so this goes out as 256-byte DMA page programs.
open(payload, 'wb').write(os.urandom(192 * 1024))
# Sub-page writes are PIO programs: these go out four bytes at a time.
open(pio, 'wb').write(os.urandom(100))
PY
ROOTFS_MD5=$(dd if="$NOR" bs=64k skip=37 count=80 2>/dev/null | md5sum | cut -d' ' -f1)

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
WDT_KO=$(cd "$WORK/rootfs" && find lib/modules -name na51055_wdt.ko | head -1)
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
cp "$WORK/payload.bin" "$WORK/pio.bin" "$WORK/initramfs/"
# The watchdog driver is a module (CONFIG_NA51055_WATCHDOG=m).
if [ -n "$WDT_KO" ]; then
    cp "$WORK/rootfs/$WDT_KO" "$WORK/initramfs/lib/"
fi
ln -s busybox "$WORK/initramfs/bin/sh"
cat > "$WORK/initramfs/init" <<'INIT'
#!/bin/sh
bb=/bin/busybox
$bb mount -t proc proc /proc
$bb mount -t sysfs sys /sys
$bb mount -t devtmpfs dev /dev
[ -f /lib/na51055_wdt.ko ] && $bb insmod /lib/na51055_wdt.ko
echo "NVT-RTC: $($bb cat /sys/class/rtc/rtc0/name 2>/dev/null) $($bb cat /sys/class/rtc/rtc0/date 2>/dev/null)"
echo "NVT-MODEL: $($bb cat /proc/device-tree/model)"
echo "NVT-CLOCKSOURCE: $($bb cat /sys/devices/system/clocksource/clocksource0/current_clocksource)"
[ -d /proc/nvt_info/nvt_pinmux ] && echo "NVT-PINCTRL: ok"
[ -n "$($bb ls /sys/class/gpio | $bb grep gpiochip)" ] && echo "NVT-GPIO: ok"
[ -c /dev/watchdog ] && echo "NVT-WDT: ok"
$bb grep -q rootfs_data /proc/mtd && echo "NVT-MTD: ok"
echo "NVT-ROOTFS-MD5: $($bb dd if=/dev/mtdblock3 bs=64k 2>/dev/null | $bb md5sum)"
$bb flash_eraseall /dev/mtd4 >/dev/null 2>&1 && echo "NVT-ERASE: ok"
$bb flashcp -v /payload.bin /dev/mtd4 >/dev/null 2>&1 && echo "NVT-FLASHCP: ok"
$bb dd if=/pio.bin of=/dev/mtd4 bs=4 seek=65536 2>/dev/null && echo "NVT-PIO: ok"
echo "NVT-BOOT-OK"
$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE"
set +e
timeout 180 "$QEMU" -M "$MACHINE,flash-file=$NOR" -nographic -monitor none \
    -serial stdio -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init mtdparts=$MTDPARTS" \
    > "$WORK/console.log" 2>&1
rc=$?
set -e
cat "$WORK/console.log"

fail=0
expect() {
    if grep -q "$1" "$WORK/console.log"; then
        echo "PASS: $2"
    else
        echo "FAIL: $2"
        fail=1
    fi
}
expect 'NVT-MODEL: Novatek NA51089' 'device tree from QEMU'
expect 'NVT-CLOCKSOURCE: arm_global_timer' 'A9 global timer is the clocksource'
expect 'NVT-PINCTRL: ok' 'vendor pinctrl probed'
expect 'NVT-GPIO: ok' 'GPIO controller registered'
expect 'NVT-WDT: ok' 'watchdog registered'
expect "NVT-RTC: nvt_rtc .* $(date -u +%Y)-" 'RTC registered, reading host time'
expect 'NVT-MTD: ok' 'SPI NOR probed, mtdparts applied'
expect "NVT-ROOTFS-MD5: $ROOTFS_MD5" 'rootfs partition reads back intact'
expect 'NVT-ERASE: ok' 'flash_eraseall of rootfs_data'
expect 'NVT-FLASHCP: ok' 'flashcp -v of a 192 KiB payload'
expect 'NVT-PIO: ok' '100 bytes written four at a time'
if python3 - "$NOR" "$WORK/payload.bin" "$WORK/pio.bin" <<'PY'
import sys
img = bytearray(open(sys.argv[1], 'rb').read())
pay = open(sys.argv[2], 'rb').read()
pio = open(sys.argv[3], 'rb').read()
base, pio_at = 0x750000, 0x750000 + 0x40000
ok = img[base:base + len(pay)] == pay and img[pio_at:pio_at + 100] == pio
img[base:base + len(pay)] = b'\xff' * len(pay)
img[pio_at:pio_at + 100] = b'\xff' * 100
ok = ok and img[base:] == b'\xff' * (len(img) - base)
sys.exit(0 if ok else 1)
PY
then
    echo "PASS: both writes landed in the image file, the rest erased"
else
    echo "FAIL: the image file does not hold the payload"
    fail=1
fi
expect 'NVT-BOOT-OK' 'PID 1 ran to the end'
expect 'restarting\.\.\.\.\.' 'reboot reached the vendor notifier'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: the reset never came)"
    fail=1
else
    echo "PASS: the reboot reset the machine"
fi
exit $fail
