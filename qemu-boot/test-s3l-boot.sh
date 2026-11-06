#!/bin/bash
# Boot the s3l machine and check what ipctool and load_ambarella do, against
# what the IP4M-1026E in ipctool#266 reported.  See docs/ambarella.md.
#
#   qemu-boot/test-s3l-boot.sh --kernel zImage --ipctool DIR --firmware DIR \
#       --target DIR [--qemu PATH]
#
#   --kernel    zImage built from OpenIPC/linux branch ambarella-s3l
#   --ipctool   directory with ARM builds of ipctool and ipcinfo
#   --firmware  an OpenIPC/firmware checkout: the shipped modules and script
#   --target    a built firmware target/ with an ARM bin/busybox and its libc
set -u

QEMU=./qemu-src/build/qemu-system-arm
KERNEL= IPCTOOL= FIRMWARE= TARGET=
while [ $# -gt 0 ]; do
    case "$1" in
        --kernel) KERNEL=$2; shift 2 ;;
        --ipctool) IPCTOOL=$2; shift 2 ;;
        --firmware) FIRMWARE=$2; shift 2 ;;
        --target) TARGET=$2; shift 2 ;;
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
for a in sh mount ls cat grep awk tr logger sleep mknod; do
    ln -s busybox "$root/bin/$a"
done
for a in modprobe insmod; do
    ln -s ../bin/busybox "$root/sbin/$a"
done
cp "$pkg"/kmod/*.ko "$kdir/ambarella/"
install -m 755 "$pkg/script/load_ambarella" "$root/usr/bin/"
install -m 755 "$IPCTOOL/ipctool" "$IPCTOOL/ipcinfo" "$root/usr/bin/"
printf '#!/bin/sh\nexit 1\n' > "$root/usr/bin/fw_printenv"
chmod +x "$root/usr/bin/fw_printenv"
/usr/sbin/depmod -b "$root" 3.10.104 2>/dev/null

cat > "$root/init" <<'INIT'
#!/bin/sh
export PATH=/sbin:/bin:/usr/bin
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs dev /dev
echo "### ipctool"
ipctool
echo "### load"
load_ambarella -i
echo "### modules"
awk '{print $1}' /proc/modules | tr '\n' ' '
echo
echo "### nodes"
ls -l /dev/iav /dev/ucode
echo "### clock"
cat /proc/ambarella/clock
echo "### again"
load_ambarella -i
echo "### end"
sleep 1000
INIT
chmod +x "$root/init"
(cd "$root" && find . | cpio -o -H newc 2>/dev/null | gzip -9) > "$work/rd.gz"

log=$work/console.log
"$QEMU" -M s3l -nographic -monitor none -kernel "$KERNEL" \
    -initrd "$work/rd.gz" \
    -append 'console=ttyS0,115200 rdinit=/init loglevel=4' > "$log" 2>&1 &
pid=$!
for _ in $(seq 1 120); do
    grep -q '^### end' "$log" && break
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
done
kill "$pid" 2>/dev/null
wait "$pid" 2>/dev/null

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

want 'the guest reached the end' '^### end'
want 'vendor' 'vendor: Ambarella'
want 'model' 'model: S3L'
want 'total RAM from /iavmem' 'total: 512M'
want 'media RAM from /iavmem' 'media: 392M'
want 'the sensor' 'model: OV4689'
want 'on bus 0' 'bus: 0'
want 'at its address' 'addr: 0x6c'
want 'iav and what it needs came up' '^(.* )?ov4689_mipi .*iav .*dsp'
want '/dev/iav, root only' 'crw------- .*248, +0 '
want '/dev/ucode, root only' 'crw------- .*248, +1 '
want 'no microcode is said out loud' 'No DSP microcode on this image'
want 'gclk_apb' 'gclk_apb:[[:space:]]+90000000 Hz'
want 'gclk_ddr' 'gclk_ddr:[[:space:]]+840000000 Hz'
absent 'no oops' 'Oops|Kernel panic|Unable to handle'

if [ "$failures" -ne 0 ]; then
    echo "--- console"
    cat "$log"
fi
echo "$([ "$failures" -eq 0 ] && echo PASS || echo FAIL): $failures failure(s)"
exit $((failures != 0))
