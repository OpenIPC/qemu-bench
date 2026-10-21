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
#   - an SD card on SDIO1 (nvt-sdio under the release's own
#     mmc_na51055.ko): the card found and its MBR read, 1 MiB the host put
#     there read back intact (descriptor DMA), 192 KiB written raw that the
#     host then finds in the card image, and a vfat made on the partition
#     holding a copied file across a remount;
#   - an eMMC on SDIO3 (the same driver, "nvt,nvt_mmc3"): found as an MMC
#     card on an 8-bit bus, 512 KiB the host put there read back intact,
#     and 192 KiB written raw that the host then finds in the image;
#   - I2C (nvt-i2c under the vendor i2c-nvt driver) with sensor=sc401ai:
#     three adapters, i2cdetect finding the sensor at 0x30 on bus 0, and the
#     release's own ipcinfo naming it sc401ai, as load_novatek asks it to;
#   - Ethernet (nvt-eqos under the vendor DWC_ETH_QOS driver): a DHCP
#     lease from QEMU's user network, then a 1 MiB file fetched over HTTP
#     from the host (guestfwd), checksummed against the original;
#   - the Novatek timer (nvt-timer): timer 0, free-running as the loader
#     leaves it, counts about a million across a one-second sleep, and the
#     release's HDAL kdrv_comm.ko (after kwrap, nvtmem and kdrv_builtin)
#     binds its timer driver to the node;
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
HTTP_PID=
cleanup() {
    [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null
    [ -n "${KEEP:-}" ] || rm -rf "$WORK"
}
trap cleanup EXIT

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

# A 64 MiB card: one FAT partition at 1..16 MiB (the guest formats it),
# 1 MiB of host data at 32 MiB, and 48 MiB left for the guest to write.
SD=$WORK/sd.img
python3 - "$SD" <<'PY'
import os, struct, sys
img = bytearray(64 << 20)
part = struct.pack('<B3sB3sII', 0, b'\0\0\0', 0x0C, b'\0\0\0', 2048, 30720)
img[446:446 + 16] = part
img[510:512] = b'\x55\xaa'
img[32 << 20:33 << 20] = os.urandom(1 << 20)
open(sys.argv[1], 'wb').write(img)
PY
SD_MD5=$(dd if="$SD" bs=1M skip=32 count=1 2>/dev/null | md5sum | cut -d' ' -f1)
# A 32 MiB eMMC: 512 KiB of host data at 2 MiB, room at 8 MiB.
EMMC=$WORK/emmc.img
python3 -c "
import os, sys
d = bytearray(32 << 20); d[2 << 20:(2 << 20) + (512 << 10)] = os.urandom(512 << 10)
open(sys.argv[1], 'wb').write(d)" "$EMMC"
EMMC_MD5=$(dd if="$EMMC" bs=64k skip=32 count=8 2>/dev/null | md5sum | cut -d' ' -f1)
PAYLOAD_MD5=$(md5sum < "$WORK/payload.bin" | cut -d' ' -f1)

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib usr/bin/ipcinfo >/dev/null
WDT_KO=$(cd "$WORK/rootfs" && find lib/modules -name na51055_wdt.ko | head -1)
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev,usr/bin}
cp "$WORK/rootfs/usr/bin/ipcinfo" "$WORK/initramfs/usr/bin/"
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
cp "$WORK/payload.bin" "$WORK/pio.bin" "$WORK/initramfs/"
mkdir -p "$WORK/www"
head -c 1048576 /dev/urandom > "$WORK/www/blob.bin"
BLOB_MD5=$(md5sum < "$WORK/www/blob.bin" | cut -d' ' -f1)
HTTP_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
python3 -m http.server "$HTTP_PORT" --bind 127.0.0.1 --directory "$WORK/www" \
    >/dev/null 2>&1 &
HTTP_PID=$!
# guestfwd connects when QEMU starts: the server must be listening by then.
for _ in $(seq 50); do
    python3 -c "import socket; socket.create_connection(('127.0.0.1', $HTTP_PORT), 1)" \
        2>/dev/null && break
    sleep 0.2
done
cat > "$WORK/initramfs/udhcpc.sh" <<'DHCP'
#!/bin/sh
[ "$1" = bound ] || exit 0
/bin/busybox ifconfig "$interface" "$ip" netmask "$subnet"
/bin/busybox route add default gw "$router"
DHCP
chmod +x "$WORK/initramfs/udhcpc.sh"
# The watchdog driver is a module (CONFIG_NA51055_WATCHDOG=m).
if [ -n "$WDT_KO" ]; then
    cp "$WORK/rootfs/$WDT_KO" "$WORK/initramfs/lib/"
fi
# So is the SD stack: mmc_core, mmc_block and the Novatek host.
for ko in mmc_core mmc_block mmc_na51055; do
    f=$(cd "$WORK/rootfs" && find lib/modules -name "$ko.ko" | head -1)
    [ -n "$f" ] && cp "$WORK/rootfs/$f" "$WORK/initramfs/lib/"
done
# And the HDAL common driver whose timer half binds the Novatek timer,
# with the modules it needs first.
for ko in kwrap nvtmem kdrv_builtin kdrv_comm; do
    f=$(cd "$WORK/rootfs" && find lib/modules -name "$ko.ko" | head -1)
    [ -n "$f" ] && cp "$WORK/rootfs/$f" "$WORK/initramfs/lib/"
done
mkdir -p "$WORK/initramfs/mnt"
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
[ -c /dev/i2c-0 ] && [ -c /dev/i2c-1 ] && [ -c /dev/i2c-2 ] && echo "NVT-I2C: ok"
$bb i2cdetect -y 0 2>/dev/null | $bb grep -q "^30: 30" && echo "NVT-I2C-DETECT: ok"
echo "NVT-SENSOR: $(/usr/bin/ipcinfo --short-sensor 2>/dev/null)"
for ko in mmc_core mmc_block mmc_na51055; do $bb insmod /lib/$ko.ko; done
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/mmcblk0p1 ] && break; $bb sleep 1; done
[ -b /dev/mmcblk0p1 ] && echo "NVT-SD: ok"
echo "NVT-SD-READ-MD5: $($bb dd if=/dev/mmcblk0 bs=1M skip=32 count=1 2>/dev/null | $bb md5sum)"
$bb dd if=/payload.bin of=/dev/mmcblk0 bs=64k seek=768 conv=fsync 2>/dev/null && echo "NVT-SD-WRITE: ok"
for i in 1 2 3 4 5; do [ -b /dev/mmcblk2 ] && break; $bb sleep 1; done
$bb grep -q MMC /sys/class/mmc_host/mmc2/mmc2:*/type 2>/dev/null && echo "NVT-EMMC: ok"
$bb dmesg | $bb grep -q "f0510000.mmc: .*8-bit" && echo "NVT-EMMC-8BIT: ok"
echo "NVT-EMMC-READ-MD5: $($bb dd if=/dev/mmcblk2 bs=64k skip=32 count=8 2>/dev/null | $bb md5sum)"
$bb dd if=/payload.bin of=/dev/mmcblk2 bs=64k seek=128 conv=fsync 2>/dev/null && echo "NVT-EMMC-WRITE: ok"
$bb mkfs.vfat /dev/mmcblk0p1 >/dev/null 2>&1 && $bb mount -t vfat /dev/mmcblk0p1 /mnt && \
    $bb cp /payload.bin /mnt/payload.bin && $bb umount /mnt && \
    $bb mount -t vfat /dev/mmcblk0p1 /mnt && \
    echo "NVT-SD-VFAT-MD5: $($bb md5sum < /mnt/payload.bin)" && $bb umount /mnt
$bb ifconfig lo up
$bb ifconfig eth0 up
$bb udhcpc -i eth0 -n -q -t 5 -s /udhcpc.sh >/dev/null 2>&1 && echo "NVT-DHCP: ok"
echo "NVT-HTTP-MD5: $($bb wget -q -O - http://10.0.2.100:@PORT@/blob.bin | $bb md5sum)"
t0=$($bb devmem 0xF0040108)
$bb sleep 1
echo "NVT-TIMER0-1S: $(( $($bb devmem 0xF0040108) - t0 ))"
for ko in kwrap nvtmem kdrv_builtin kdrv_comm; do $bb insmod /lib/$ko.ko; done
[ -e /sys/bus/platform/drivers/nvt_timer/f0040000.timer ] && echo "NVT-TIMER: ok"
echo "NVT-BOOT-OK"
$bb reboot -f
INIT
sed -i "s/@PORT@/$HTTP_PORT/" "$WORK/initramfs/init"
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE"
set +e
timeout 180 "$QEMU" -M "$MACHINE,flash-file=$NOR,sensor=sc401ai" \
    -nographic -monitor none \
    -serial stdio -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init mtdparts=$MTDPARTS" \
    -nic user,restrict=on,guestfwd=tcp:10.0.2.100:$HTTP_PORT-tcp:127.0.0.1:$HTTP_PORT \
    -drive if=sd,index=0,format=raw,file="$SD" \
    -drive if=sd,index=2,format=raw,file="$EMMC" \
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
expect 'NVT-EMMC: ok' 'eMMC on SDIO3 found as an MMC card'
expect 'NVT-EMMC-8BIT: ok' 'its host in 8-bit mode'
expect "NVT-EMMC-READ-MD5: $EMMC_MD5" '512 KiB read back from the eMMC intact'
expect 'NVT-EMMC-WRITE: ok' '192 KiB written raw to the eMMC'
if [ "$(dd if="$EMMC" bs=64k skip=128 count=3 2>/dev/null | md5sum | cut -d' ' -f1)" = "$PAYLOAD_MD5" ]; then
    echo "PASS: the raw write is in the eMMC image"
else
    echo "FAIL: the raw write is not in the eMMC image"
    fail=1
fi
expect 'NVT-I2C: ok' 'three I2C adapters registered'
expect 'NVT-I2C-DETECT: ok' 'i2cdetect finds the sensor at 0x30'
expect 'NVT-SENSOR: sc401ai' 'ipcinfo identifies it as sc401ai'
expect 'NVT-SD: ok' 'SD card found, its partition table read'
expect "NVT-SD-READ-MD5: $SD_MD5" '1 MiB read back from the card intact'
expect 'NVT-SD-WRITE: ok' '192 KiB written raw to the card'
if [ "$(dd if="$SD" bs=64k skip=768 count=3 2>/dev/null | md5sum | cut -d' ' -f1)" = "$PAYLOAD_MD5" ]; then
    echo "PASS: the raw write is in the card image"
else
    echo "FAIL: the raw write is not in the card image"
    fail=1
fi
expect "NVT-SD-VFAT-MD5: $PAYLOAD_MD5" 'vfat made on the card keeps a file across a remount'
expect 'NVT-DHCP: ok' 'DHCP lease over nvt-eqos'
expect "NVT-HTTP-MD5: $BLOB_MD5" '1 MiB over HTTP, intact'
if grep -q 'NVT-TIMER0-1S: 1[0-9][0-9][0-9][0-9][0-9][0-9]' "$WORK/console.log"; then
    echo "PASS: timer 0 counts about 1000000 across a 1 s sleep (1 MHz)"
else
    echo "FAIL: timer 0 does not count at 1 MHz"
    fail=1
fi
expect 'NVT-TIMER: ok' 'kdrv_comm.ko binds the timer at f0040000'
expect 'NVT-BOOT-OK' 'PID 1 ran to the end'
expect 'restarting\.\.\.\.\.' 'reboot reached the vendor notifier'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: the reset never came)"
    fail=1
else
    echo "PASS: the reboot reset the machine"
fi
exit $fail
