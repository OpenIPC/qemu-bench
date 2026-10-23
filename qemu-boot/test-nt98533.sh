#!/bin/bash
# Boot the NT98533 (NS02501) SDK's own U-Boot and kernel on -M nt98533 and
# check what they find.  Needs qemu-boot/build-ns02501-sdk.sh's output (the
# SDK is not redistributable, so this is not in CI) and an OpenIPC
# nt98562 release rootfs for busybox (armhf musl, which the A7 runs):
#
#   bash qemu-boot/test-nt98533.sh --dir ns02501 \
#       [--rootfs qemu-boot/rootfs.squashfs.nt98562] \
#       [--qemu ./qemu-src/build/qemu-system-arm]
#
# 1. Flash boot of DIR/nor.bin (a copy): the machine stands in for the
#    loader (FDT from 0x10000, U-Boot from partition_uboot, SHMINFO), and
#    the vendor U-Boot 2021.10 probes the SPI NOR, autoboots nvt_boot, loads
#    the kernel from its partition and starts it; Linux brings up the
#    second core through the core communicator's release register.  With a
#    rootfs in the image it mounts it.
# 2. -kernel DIR/zImage -dtb DIR/nvt-evb.dtb with an initramfs whose /init
#    reports:
#    - the SDK tree's model, two CPUs, the generic timer at 12 MHz as the
#      clocksource;
#    - the vendor pinctrl, GPIO and I2C drivers bound;
#    - the SPI NOR under the vendor driver: the kernel partition read back
#      and checksummed, the app partition erased and a 192 KiB payload
#      flashcp'd onto it, which the host then finds in the image file;
#    - the SD card under the SDK's nvt-mmc.ko: 1 MiB the host put there
#      read back intact, 192 KiB written raw that the host then finds;
#    - Ethernet under the SDK's ntkimethmac.ko (DWC EQoS): a DHCP lease
#      and a 1 MiB file over HTTP, checksummed;
#    - the Novatek timer 0 counting about a million across a 1 s sleep.
set -euo pipefail

DIR=ns02501
ROOTFS=qemu-boot/rootfs.squashfs.nt98562
QEMU=./qemu-src/build/qemu-system-arm
while [ $# -gt 0 ]; do
    case "$1" in
        --dir) DIR=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
for f in "$DIR/nor.bin" "$DIR/zImage" "$DIR/nvt-evb.dtb" \
         "$DIR/modules/nvt-mmc.ko" "$ROOTFS"; do
    if [ ! -f "$f" ]; then
        echo "need $f (qemu-boot/build-ns02501-sdk.sh; the OpenIPC" \
             "nt98562 release for the rootfs)" >&2
        exit 2
    fi
done

WORK=$(mktemp -d)
QPID=
HTTP_PID=
cleanup() {
    [ -n "$QPID" ] && kill "$QPID" 2>/dev/null
    [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null
    [ -n "${KEEP:-}" ] || rm -rf "$WORK"
}
trap cleanup EXIT

fail=0
expect() {
    if grep -aq -- "$1" "$3"; then
        echo "PASS: $2"
    else
        echo "FAIL: $2"
        fail=1
    fi
}

# Run QEMU until the log shows $1 or $2 seconds pass, then stop it.
run_until() {
    local marker=$1 secs=$2 log=$3
    shift 3
    "$QEMU" "$@" > "$log" 2>&1 &
    QPID=$!
    for _ in $(seq $((secs * 5))); do
        grep -aq -- "$marker" "$log" && break
        kill -0 "$QPID" 2>/dev/null || break
        sleep 0.2
    done
    kill "$QPID" 2>/dev/null || true
    wait "$QPID" 2>/dev/null || true
    QPID=
}

echo "== flash boot"
cp "$DIR/nor.bin" "$WORK/nor.bin"
HAS_ROOTFS=
[ "$(dd if="$WORK/nor.bin" bs=4 skip=$((0x470000 / 4)) count=1 2>/dev/null)" = hsqs ] &&
    HAS_ROOTFS=1
MARK='Brought up 1 node, 2 CPUs'
[ -n "$HAS_ROOTFS" ] && MARK='Run /sbin/init'
run_until "$MARK" 90 "$WORK/flash.log" -M "nt98533,flash-file=$WORK/nor.bin" \
    -nographic -monitor none -serial stdio -nic user,restrict=on
cat "$WORK/flash.log"
F=$WORK/flash.log
expect 'U-Boot 2021.10' 'the SDK U-Boot runs from partition_uboot' "$F"
expect 'nvt_shminfo_init: .*fdt buffer' 'it takes the device tree from the SHMINFO' "$F"
expect '^256 MiB' 'and 256 MiB of DRAM from nvt_memory_cfg' "$F"
expect 'id =  0xef 0x40 0x18' 'the 16 MiB W25Q probed' "$F"
expect 'Verifying Checksum ... OK' 'nvt_boot loads the uImage from the kernel partition' "$F"
expect 'Booting Linux on physical CPU 0x0' 'and starts it' "$F"
expect 'Machine model: Novatek NS02501' 'with the SDK tree' "$F"
expect "$MARK" 'the second core comes up through CC_CPU2_CPU1_CMDBUF_REG1' "$F"
[ -n "$HAS_ROOTFS" ] &&
    expect 'VFS: Mounted root (squashfs' 'the squashfs root mounts from NOR' "$F"

echo "== -kernel with an initramfs"
NOR=$WORK/nor.bin
KERNEL_MD5=$(dd if="$NOR" bs=64k skip=15 count=56 2>/dev/null | md5sum | cut -d' ' -f1)
head -c $((192 * 1024)) /dev/urandom > "$WORK/payload.bin"
PAYLOAD_MD5=$(md5sum < "$WORK/payload.bin" | cut -d' ' -f1)
SD=$WORK/sd.img
python3 -c "
import os, sys
d = bytearray(64 << 20); d[32 << 20:33 << 20] = os.urandom(1 << 20)
open(sys.argv[1], 'wb').write(d)" "$SD"
SD_MD5=$(dd if="$SD" bs=1M skip=32 count=1 2>/dev/null | md5sum | cut -d' ' -f1)

R=$WORK/initramfs
unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
mkdir -p "$R"/{bin,lib/m,proc,sys,dev,mnt}
cp "$WORK/rootfs/bin/busybox" "$R/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" "$R/lib/"
ln -s busybox "$R/bin/sh"
for ko in mmc_core mmc_block nvt-mmc mii nvt_phy ntkimethmac; do
    cp "$DIR/modules/$ko.ko" "$R/lib/m/"
done
cp "$WORK/payload.bin" "$R/"
mkdir -p "$WORK/www"
head -c 1048576 /dev/urandom > "$WORK/www/blob.bin"
BLOB_MD5=$(md5sum < "$WORK/www/blob.bin" | cut -d' ' -f1)
HTTP_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
python3 -m http.server "$HTTP_PORT" --bind 127.0.0.1 --directory "$WORK/www" \
    >/dev/null 2>&1 &
HTTP_PID=$!
for _ in $(seq 50); do
    python3 -c "import socket; socket.create_connection(('127.0.0.1', $HTTP_PORT), 1)" \
        2>/dev/null && break
    sleep 0.2
done
cat > "$R/udhcpc.sh" <<'DHCP'
#!/bin/sh
[ "$1" = bound ] || exit 0
/bin/busybox ifconfig "$interface" "$ip" netmask "$subnet"
/bin/busybox route add default gw "$router"
DHCP
chmod +x "$R/udhcpc.sh"
cat > "$R/init" <<'INIT'
#!/bin/sh
bb=/bin/busybox
$bb mount -t proc proc /proc
$bb mount -t sysfs sys /sys
$bb mount -t devtmpfs dev /dev
echo "NS-MODEL: $($bb cat /proc/device-tree/model)"
echo "NS-CPUS: $($bb grep -c ^processor /proc/cpuinfo)"
echo "NS-CLOCKSOURCE: $($bb cat /sys/devices/system/clocksource/clocksource0/current_clocksource)"
$bb dmesg | $bb grep -q 'arch_timer: cp15 timer(s) running at 12.00MHz' && echo "NS-ARCH-TIMER: 12 MHz"
[ -d /proc/nvt_info/nvt_pinmux ] && echo "NS-PINCTRL: ok"
[ -n "$($bb ls /sys/class/gpio | $bb grep gpiochip)" ] && echo "NS-GPIO: ok"
[ -c /dev/i2c-0 ] && [ -c /dev/i2c-1 ] && [ -c /dev/i2c-2 ] && echo "NS-I2C: ok"
echo "NS-KERNEL-MD5: $($bb dd if=/dev/mtdblock5 bs=64k 2>/dev/null | $bb md5sum)"
$bb flash_eraseall /dev/mtd7 >/dev/null 2>&1 && echo "NS-ERASE: ok"
$bb flashcp /payload.bin /dev/mtd7 >/dev/null 2>&1 && echo "NS-FLASHCP: ok"
for ko in mmc_core mmc_block nvt-mmc mii nvt_phy ntkimethmac; do
    $bb insmod /lib/m/$ko.ko
done
for i in 1 2 3 4 5 6 7 8 9 10; do [ -b /dev/mmcblk0 ] && break; $bb sleep 1; done
echo "NS-SD-READ-MD5: $($bb dd if=/dev/mmcblk0 bs=1M skip=32 count=1 2>/dev/null | $bb md5sum)"
$bb dd if=/payload.bin of=/dev/mmcblk0 bs=64k seek=768 conv=fsync 2>/dev/null && echo "NS-SD-WRITE: ok"
$bb ifconfig lo up
$bb ifconfig eth0 up
$bb udhcpc -i eth0 -n -q -t 5 -s /udhcpc.sh >/dev/null 2>&1 && echo "NS-DHCP: ok"
echo "NS-HTTP-MD5: $($bb wget -q -O - http://10.0.2.100:@PORT@/blob.bin | $bb md5sum)"
t0=$($bb devmem 0xF0040108)
$bb sleep 1
echo "NS-TIMER0-1S: $(( $($bb devmem 0xF0040108) - t0 ))"
echo "NS-BOOT-OK"
INIT
sed -i "s/@PORT@/$HTTP_PORT/" "$R/init"
chmod +x "$R/init"
(cd "$R" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

run_until 'NS-BOOT-OK' 180 "$WORK/kernel.log" -M "nt98533,flash-file=$NOR" \
    -nographic -monitor none -serial stdio \
    -kernel "$DIR/zImage" -dtb "$DIR/nvt-evb.dtb" \
    -initrd "$WORK/initramfs.cpio" -append "console=ttyS0,115200 rdinit=/init" \
    -nic user,restrict=on,guestfwd=tcp:10.0.2.100:$HTTP_PORT-tcp:127.0.0.1:$HTTP_PORT \
    -drive if=sd,index=0,format=raw,file="$SD"
cat "$WORK/kernel.log"
K=$WORK/kernel.log
expect 'NS-MODEL: Novatek NS02501' 'the SDK board tree' "$K"
expect 'NS-CPUS: 2' 'both cores up' "$K"
expect 'NS-CLOCKSOURCE: arch_sys_counter' 'the generic timer is the clocksource' "$K"
expect 'NS-ARCH-TIMER: 12 MHz' 'at the tree'"'"'s 12 MHz' "$K"
expect 'NS-PINCTRL: ok' 'vendor pinctrl probed' "$K"
expect 'NS-GPIO: ok' 'GPIO controller registered' "$K"
expect 'NS-I2C: ok' 'three I2C adapters registered' "$K"
expect "NS-KERNEL-MD5: $KERNEL_MD5" 'the kernel partition reads back intact' "$K"
expect 'NS-ERASE: ok' 'flash_eraseall of the app partition' "$K"
expect 'NS-FLASHCP: ok' 'flashcp of a 192 KiB payload' "$K"
if [ "$(dd if="$NOR" bs=64k skip=232 count=3 2>/dev/null | md5sum | cut -d' ' -f1)" = "$PAYLOAD_MD5" ]; then
    echo "PASS: the payload is in the image file"
else
    echo "FAIL: the image file does not hold the payload"
    fail=1
fi
expect "NS-SD-READ-MD5: $SD_MD5" '1 MiB read back from the SD card intact' "$K"
expect 'NS-SD-WRITE: ok' '192 KiB written raw to the card' "$K"
if [ "$(dd if="$SD" bs=64k skip=768 count=3 2>/dev/null | md5sum | cut -d' ' -f1)" = "$PAYLOAD_MD5" ]; then
    echo "PASS: the raw write is in the card image"
else
    echo "FAIL: the raw write is not in the card image"
    fail=1
fi
expect 'NS-DHCP: ok' 'DHCP lease over nvt-eqos' "$K"
expect "NS-HTTP-MD5: $BLOB_MD5" '1 MiB over HTTP, intact' "$K"
if grep -Eaq 'NS-TIMER0-1S: 1[0-9]{6}\s*$' "$K"; then
    echo "PASS: timer 0 counts about 1000000 across a 1 s sleep"
else
    echo "FAIL: timer 0 does not count at 1 MHz"
    fail=1
fi
expect 'NS-BOOT-OK' 'PID 1 ran to the end' "$K"
exit $fail
