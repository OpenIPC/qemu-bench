#!/bin/bash
# Boot the s3l machine and check what ipctool and load_ambarella do, against
# what the IP4M-1026E in ipctool#266 reported.  See docs/ambarella.md.
#
#   qemu-boot/test-s3l-boot.sh --kernel zImage --ipctool DIR --firmware DIR \
#       --target DIR [--cc CROSS-GCC] [--qemu PATH]
#
#   --kernel    zImage built from OpenIPC/linux branch ambarella-s3l
#   --ipctool   directory with ARM builds of ipctool and ipcinfo
#   --firmware  an OpenIPC/firmware checkout: the shipped modules and script
#   --target    a built firmware target/ with an ARM bin/busybox and its libc
#   --cc        an ARM gcc: also build the firmware's load_ucode and run it
#               against the real dsp.ko, with made-up microcode
set -u

QEMU=./qemu-src/build/qemu-system-arm
KERNEL= IPCTOOL= FIRMWARE= TARGET= CROSS=
while [ $# -gt 0 ]; do
    case "$1" in
        --kernel) KERNEL=$2; shift 2 ;;
        --ipctool) IPCTOOL=$2; shift 2 ;;
        --firmware) FIRMWARE=$2; shift 2 ;;
        --target) TARGET=$2; shift 2 ;;
        --cc) CROSS=$2; shift 2 ;;
        --qemu) QEMU=$2; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
for v in KERNEL IPCTOOL FIRMWARE TARGET; do
    [ -n "${!v}" ] || { echo "--${v,,} is required" >&2; exit 2; }
done

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
root=$work/root
kdir=$root/lib/modules/3.10.104
pkg=$FIRMWARE/general/package/legacy/ambarella-osdrv-s3l/files

mkdir -p "$root"/{bin,sbin,usr/bin,proc,sys,dev,tmp,lib} "$kdir/ambarella"
cp "$TARGET/bin/busybox" "$root/bin/"
cp -L "$TARGET/lib/ld-musl-arm.so.1" "$TARGET/lib/libc.so" "$root/lib/"
for a in sh mount ls cat grep awk tr logger sleep mknod echo od; do
    ln -s busybox "$root/bin/$a"
done
for a in modprobe insmod; do
    ln -s ../bin/busybox "$root/sbin/$a"
done
cp "$pkg"/kmod/*.ko "$kdir/ambarella/"
install -m 755 "$pkg/script/load_ambarella" "$root/usr/bin/"
install -m 755 "$IPCTOOL/ipctool" "$IPCTOOL/ipcinfo" "$root/usr/bin/"
# The environment: sensor= when the kernel command line asks for it
cat > "$root/usr/bin/fw_printenv" <<'ENV'
#!/bin/sh
case "$(cat /proc/cmdline)" in
*s3l.env_sensor*) [ "$2" = sensor ] && echo ov4689 && exit 0 ;;
esac
exit 1
ENV
chmod +x "$root/usr/bin/fw_printenv"
/usr/sbin/depmod -b "$root" 3.10.104 2>/dev/null

# Reads the window back: the three words the version comes from, and a sum
# over the whole of it, so a bad set can be shown to have changed nothing.
peek_c() {
    cat <<'PEEK'
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
int main(void)
{
    int fd = open("/dev/ucode", O_RDWR);
    uint32_t *m = mmap(0, 5 << 20, PROT_READ, MAP_SHARED, fd, 0);
    uint32_t sum = 0;

    if (m == MAP_FAILED) {
        perror("mmap");
        return 1;
    }
    for (uint32_t i = 0; i < (5 << 20) / 4; i++) {
        sum += m[i] * (i | 1);
    }
    printf("window %08x %u %u sum %08x\n", m[23], m[22], m[24], sum);
    return 0;
}
PEEK
}

ucode_set() {   # dir num ver date orccode-size orcme-size binary-size
    python3 - "$@" <<'SET'
import os
import struct
import sys

d, num, ver, date, a, b, c = sys.argv[1:]
os.makedirs(d, exist_ok=True)
code = bytearray(b'\x5a' * int(a))
if int(a) >= 100:
    struct.pack_into('<III', code, 88, int(num), int(date), int(ver))
open(d + '/orccode.bin', 'wb').write(code)
open(d + '/orcme.bin', 'wb').write(b'\x5b' * int(b))
open(d + '/default_binary.bin', 'wb').write(b'\x5c' * int(c))
SET
}

ucode_run=
if [ -n "$CROSS" ]; then
    src=$FIRMWARE/general/package/legacy/ambarella-osdrv-s3l/src/load_ucode.c
    "$CROSS" -static -O2 -o "$root/usr/bin/load_ucode" "$src" || exit 1
    peek_c | "$CROSS" -static -O2 -x c -o "$root/usr/bin/peek" - || exit 1
    # 2017/2/22
    ucode_set "$root/lib/firmware" 267837 266889 132186646 1648217 201712 399808
    ucode_set "$root/t/big" 1 2 3 1000 655361 1000
    ucode_set "$root/t/empty" 1 2 3 1000 0 1000
    ucode_run=1
fi

cat > "$root/init" <<'INIT'
#!/bin/sh
export PATH=/sbin:/bin:/usr/bin
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs dev /dev
case "$(cat /proc/cmdline)" in
*s3l.held*)
    # The camera at boot: the sensor is in reset until GPIO 107 is released
    g=/sys/class/gpio/gpio107
    echo "### held ipcinfo=[$(ipcinfo -s 2>/dev/null)]"
    echo 107 > /sys/class/gpio/export
    echo out > $g/direction
    echo 1 > $g/value
    echo "### released ipcinfo=[$(ipcinfo -s 2>/dev/null)]"
    echo 0 > $g/value
    echo "### low ipcinfo=[$(ipcinfo -s 2>/dev/null)]"
    echo 107 > /sys/class/gpio/unexport
    # Finds the pin in the device tree and releases it
    load_ambarella -i
    echo "### held load rc=$?"
    awk '{print $1}' /proc/modules | tr '\n' ' '
    echo
    echo "### end"
    sleep 1000
    ;;
esac
echo "### load"
load_ambarella -i
echo "### first rc=$?"
echo "### ipctool"
ipctool
echo "### modules"
awk '{print $1}' /proc/modules | tr '\n' ' '
echo
echo "### nodes"
ls -l /dev/iav /dev/ucode
echo "### clock"
cat /proc/ambarella/clock
echo "### again"
load_ambarella -i
echo "### again rc=$?"
if [ -x /usr/bin/peek ]; then
    # load_ambarella keeps the loader quiet, so once more where it shows
    load_ucode /lib/firmware
    echo "### window"
    peek
    for d in /nonexistent /t/big /t/empty; do
        load_ucode $d
        echo "### bad set rc=$?"
        peek
    done
fi
echo "### end"
sleep 1000
INIT
chmod +x "$root/init"
(cd "$root" && find . | cpio -o -H newc 2>/dev/null | gzip -9) > "$work/rd.gz"

boot() {    # log machine-options extra-cmdline
    "$QEMU" -M "s3l$2" -nographic -monitor none -kernel "$KERNEL" \
        -initrd "$work/rd.gz" \
        -append "console=ttyS0,115200 rdinit=/init loglevel=4 $3" > "$1" 2>&1 &
    pid=$!
    for _ in $(seq 1 120); do
        grep -q '^### end' "$1" && break
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    kill "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
}

# First as the camera is, with nothing in the environment: the sensor held in
# reset until load_ambarella reads the pin from the device tree.  Then with
# sensor= set, so nothing probes and the real driver releases the line.
log=$work/held.log
boot "$log" "" s3l.held

failures=0
want() {
    if grep -qE "$2" "$log"; then
        echo "  PASS: $1"
    else
        echo "  FAIL: $1"
        failures=$((failures + 1))
    fi
}
absent() {
    if grep -qE "$2" "$log"; then
        echo "  FAIL: $1"
        failures=$((failures + 1))
    else
        echo "  PASS: $1"
    fi
}

echo 'sensor held in reset, as on the camera:'
want 'the guest reached the end' '^### end'
want 'held: nothing answers' '^### held ipcinfo=\[\]'
want 'GPIO 107 high: the sensor answers' '^### released ipcinfo=\[ov4689\]'
want 'GPIO 107 low: gone again' '^### low ipcinfo=\[\]'
want 'load_ambarella releases it from the device tree' \
    '^ov4689_mipi .*iav .*dsp'
absent 'no sensor complaint' 'none detected'
absent 'no oops' 'Oops|Kernel panic|Unable to handle'
cp "$log" "$work/held.keep"

echo 'with sensor= in the environment:'
log=$work/console.log
boot "$log" "" s3l.env_sensor

want 'the guest reached the end' '^### end'
want 'vendor' 'vendor: Ambarella'
want 'model' 'model: S3L'
want 'total RAM from /iavmem' 'total: 512M'
want 'media RAM from /iavmem' 'media: 392M'
want 'the sensor' 'model: OV4689'
want 'on bus 0' 'bus: 0'
want 'at its address' 'addr: 0x6c'
if [ -n "$ucode_run" ]; then
    want 'load_ambarella ran the loader' '^### first rc=0'
    want 'and again' '^### again rc=0'
    want 'the driver read the version back' \
        'ucode version 2017/2/22 267837\.266889'
    want 'orccode.bin at its offset' 'orccode\.bin: 1648217 bytes at \+0x000000'
    want 'orcme.bin at its offset' 'orcme\.bin: 201712 bytes at \+0x300000'
    want 'too big a file is refused' 'orcme\.bin is 655361 bytes'
    want 'an empty file is refused' 'orcme\.bin is 0 bytes'
    [ "$(grep -c '^### bad set rc=1' "$log")" = 3 ] && result=true || result=false
    if $result; then
        echo "  PASS: every bad set exits 1"
    else
        echo "  FAIL: every bad set exits 1"
        failures=$((failures + 1))
    fi
    if [ "$(grep '^window ' "$log" | sort -u | wc -l)" = 1 ] &&
       [ "$(grep -c '^window ' "$log")" = 4 ]; then
        echo "  PASS: a bad set leaves the window as it was"
    else
        echo "  FAIL: a bad set leaves the window as it was"
        failures=$((failures + 1))
    fi
else
    want 'no microcode is said out loud' 'No DSP microcode on this image'
    # load_ambarella returns 1 for that, the same both times
    want 'the second run ended as the first' '^### again rc=1'
fi
want 'iav and what it needs came up' '^(.* )?ov4689_mipi .*iav .*dsp'
want '/dev/iav, root only' 'crw------- .*248, +0 '
want '/dev/ucode, root only' 'crw------- .*248, +1 '
want 'gclk_apb' 'gclk_apb:[[:space:]]+90000000 Hz'
want 'gclk_ddr' 'gclk_ddr:[[:space:]]+840000000 Hz'
absent 'no oops' 'Oops|Kernel panic|Unable to handle'

if [ "$failures" -ne 0 ]; then
    echo "--- console, held"
    cat "$work/held.keep"
    echo "--- console"
    cat "$log"
fi
echo "$([ "$failures" -eq 0 ] && echo PASS || echo FAIL): $failures failure(s)"
exit $((failures != 0))
