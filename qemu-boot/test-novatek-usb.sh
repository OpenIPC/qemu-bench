#!/bin/bash
# USB on emulated NT9856x: a kernel with the vendor EHCI driver
# (qemu-boot/build-novatek-test-kernel.sh) boots to a PID-1 script with a
# QEMU usb-storage stick on the FOTG200 host port, and checks that:
#
#   - ehci-na51089.c ("ehci-nvtivot") binds, at 0xFF600000 where it looks
#     on NA51089, and starts the controller -- after the OTP driver gives
#     it the (blank) USB PHY trim through nvt-otp;
#   - the stick enumerates as a high-speed device (speed from the OTG
#     register at +0x80, which the driver reads in place of PORTSC) and
#     usb-storage attaches it as sda;
#   - 1 MiB the host put at 4 MiB reads back intact, 192 KiB written raw at
#     8 MiB is then in the image file, and a vfat made on the partition at
#     16 MiB keeps a file across a remount.
#
# The busybox comes from the OpenIPC release rootfs, as in
# test-novatek-boot.sh.
#
#   bash qemu-boot/test-novatek-usb.sh --kernel uImage.nt9856x-test \
#       [--soc nt98566] [--machine nt98566] [--dir qemu-boot] \
#       [--qemu ./qemu-src/build/qemu-system-arm]
set -euo pipefail

KERNEL=
SOC=nt98566
MACHINE=
QEMU=./qemu-src/build/qemu-system-arm
DIR=qemu-boot
while [ $# -gt 0 ]; do
    case "$1" in
        --kernel) KERNEL=$2; shift 2 ;;
        --soc) SOC=$2; shift 2 ;;
        --machine) MACHINE=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        --dir) DIR=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
MACHINE=${MACHINE:-$SOC}
ROOTFS=$DIR/rootfs.squashfs.$SOC
if [ ! -f "$KERNEL" ]; then
    echo "need --kernel: a kernel with USB host support" \
         "(qemu-boot/build-novatek-test-kernel.sh)" >&2
    exit 2
fi
if [ ! -f "$ROOTFS" ]; then
    echo "need $ROOTFS: unpack openipc.$SOC-nor-lite.tgz in $DIR" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# A 32 MiB stick: an MBR with one FAT partition at 16 MiB, 1 MiB of host
# data at 4 MiB, room at 8 MiB for the guest's raw write.
STICK=$WORK/stick.img
python3 - "$STICK" "$WORK/payload.bin" <<'PY'
import os, struct, sys
img = bytearray(32 << 20)
img[446:462] = struct.pack('<B3sB3sII', 0, b'\0\0\0', 0x0C, b'\0\0\0',
                           32768, 32768)
img[510:512] = b'\x55\xaa'
img[4 << 20:5 << 20] = os.urandom(1 << 20)
open(sys.argv[1], 'wb').write(img)
open(sys.argv[2], 'wb').write(os.urandom(192 << 10))
PY
READ_MD5=$(dd if="$STICK" bs=1M skip=4 count=1 2>/dev/null | md5sum | cut -d' ' -f1)
PAYLOAD_MD5=$(md5sum < "$WORK/payload.bin" | cut -d' ' -f1)

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev,mnt}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
cp "$WORK/payload.bin" "$WORK/initramfs/"
ln -s busybox "$WORK/initramfs/bin/sh"
cat > "$WORK/initramfs/init" <<'INIT'
#!/bin/sh
bb=/bin/busybox
$bb mount -t proc proc /proc
$bb mount -t sysfs sys /sys
$bb mount -t devtmpfs dev /dev
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/sda1 ] && break; $bb sleep 1; done
[ -b /dev/sda1 ] && echo "NVT-USB-DISK: ok"
echo "NVT-USB-SPEED: $($bb cat /sys/bus/usb/devices/1-1/speed 2>/dev/null)"
echo "NVT-USB-READ-MD5: $($bb dd if=/dev/sda bs=1M skip=4 count=1 2>/dev/null | $bb md5sum)"
$bb dd if=/payload.bin of=/dev/sda bs=64k seek=128 conv=fsync 2>/dev/null && echo "NVT-USB-WRITE: ok"
$bb mkfs.vfat /dev/sda1 >/dev/null 2>&1 && $bb mount -t vfat /dev/sda1 /mnt && \
    $bb cp /payload.bin /mnt/payload.bin && $bb umount /mnt && \
    $bb mount -t vfat /dev/sda1 /mnt && \
    echo "NVT-USB-VFAT-MD5: $($bb md5sum < /mnt/payload.bin)" && $bb umount /mnt
echo "NVT-USB-OK"
$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE with a USB stick"
set +e
timeout 180 "$QEMU" -M "$MACHINE" -nographic -monitor none -serial stdio \
    -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init" -nic none \
    -drive if=none,id=stick,format=raw,file="$STICK" \
    -device usb-storage,drive=stick > "$WORK/console.log" 2>&1
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
expect 'ehci-nvtivot .*io mem 0xff600000' 'the vendor EHCI driver binds, at the NA51089 alias'
expect 'USB 2.0 started, EHCI' 'and starts the controller'
expect 'new high-speed USB device number .* using ehci-nvtivot' 'the stick enumerates at high speed'
expect 'NVT-USB-SPEED: 480' 'sysfs agrees: 480 Mb/s'
expect '\[sda\] Attached SCSI' 'usb-storage attaches it as sda'
expect 'NVT-USB-DISK: ok' 'its partition table is read'
expect "NVT-USB-READ-MD5: $READ_MD5" '1 MiB read back from the stick intact'
expect 'NVT-USB-WRITE: ok' '192 KiB written raw to the stick'
if [ "$(dd if="$STICK" bs=64k skip=128 count=3 2>/dev/null | md5sum | cut -d' ' -f1)" = "$PAYLOAD_MD5" ]; then
    echo "PASS: the raw write is in the stick image"
else
    echo "FAIL: the raw write is not in the stick image"
    fail=1
fi
expect "NVT-USB-VFAT-MD5: $PAYLOAD_MD5" 'vfat made on the stick keeps a file across a remount'
expect 'NVT-USB-OK' 'PID 1 ran to the end'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc"
    fail=1
fi
exit $fail
