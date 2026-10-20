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
trap 'rm -rf "$WORK"' EXIT

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
WDT_KO=$(cd "$WORK/rootfs" && find lib/modules -name na51055_wdt.ko | head -1)
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
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
echo "NVT-BOOT-OK"
$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE"
set +e
timeout 180 "$QEMU" -M "$MACHINE" -nographic -monitor none -serial stdio \
    -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init" > "$WORK/console.log" 2>&1
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
expect 'NVT-BOOT-OK' 'PID 1 ran to the end'
expect 'restarting\.\.\.\.\.' 'reboot reached the vendor notifier'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: the reset never came)"
    fail=1
else
    echo "PASS: the reboot reset the machine"
fi
exit $fail
