#!/bin/bash
# PWM on emulated NT9856x: a kernel with the vendor PWM driver
# (pwm-na51055.c; qemu-boot/build-novatek-test-kernel.sh) boots to a PID-1
# script that drives channel 0 through /sys/class/pwm and checks what
# reached the controller (nvt-pwm) with devmem:
#
#   - the driver binds and registers a pwmchip;
#   - a 1 ms period at 50 % duty programs base period 30000 and fall 15000
#     (the driver's 120 MHz / 4 = 30 MHz): PERIOD (0x04) holds the low
#     bytes, 0x00309800, and the extended register (0x230) the high ones,
#     0x00753A00;
#   - enable sets channel 0 in ENABLE (0x100), which the driver polls;
#   - disable returns -- the driver waits for the stop interrupt -- and
#     clears it;
#   - the vendor "cycle" attribute: 20 periods, and enable then waits for
#     the channel to stop on its own (the cycle-count interrupt), leaving
#     ENABLE clear.
#
# The busybox comes from the OpenIPC release rootfs.
#
#   bash qemu-boot/test-novatek-pwm.sh --kernel uImage.nt9856x-test \
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
    echo "need --kernel: a kernel with the PWM driver" \
         "(qemu-boot/build-novatek-test-kernel.sh)" >&2
    exit 2
fi
if [ ! -f "$ROOTFS" ]; then
    echo "need $ROOTFS: unpack openipc.$SOC-nor-lite.tgz in $DIR" >&2
    exit 2
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

unsquashfs -q -d "$WORK/rootfs" "$ROOTFS" bin/busybox lib >/dev/null
mkdir -p "$WORK/initramfs"/{bin,lib,proc,sys,dev}
cp "$WORK/rootfs/bin/busybox" "$WORK/initramfs/bin/"
cp -a "$WORK/rootfs/lib/libc.so" "$WORK/rootfs/lib/ld-musl-armhf.so.1" \
    "$WORK/initramfs/lib/"
ln -s busybox "$WORK/initramfs/bin/sh"
cat > "$WORK/initramfs/init" <<'INIT'
#!/bin/sh
bb=/bin/busybox
$bb mount -t proc proc /proc
$bb mount -t sysfs sys /sys
$bb mount -t devtmpfs dev /dev
c=$($bb ls -d /sys/class/pwm/pwmchip* 2>/dev/null | $bb head -1)
[ -n "$c" ] && echo "NVT-PWM-CHIP: $($bb cat $c/npwm)"
echo 0 > $c/export
p=$c/pwm0
echo 1000000 > $p/period && echo 500000 > $p/duty_cycle && echo "NVT-PWM-CONFIG: ok"
echo "NVT-PWM-PERIOD: $($bb devmem 0xF0210004) $($bb devmem 0xF0210230)"
echo 1 > $p/enable && echo "NVT-PWM-ENABLE: $($bb devmem 0xF0210100)"
echo 0 > $p/enable && echo "NVT-PWM-DISABLE: $($bb devmem 0xF0210100)"
echo 20 > $p/cycle && echo 1 > $p/enable && echo "NVT-PWM-CYCLES: $($bb devmem 0xF0210100)"
echo "NVT-PWM-OK"
$bb reboot -f
INIT
chmod +x "$WORK/initramfs/init"
(cd "$WORK/initramfs" && find . | cpio -o -H newc 2>/dev/null) > "$WORK/initramfs.cpio"

echo "Booting $KERNEL on -M $MACHINE"
set +e
timeout 180 "$QEMU" -M "$MACHINE" -nographic -monitor none -serial stdio \
    -no-reboot -kernel "$KERNEL" -initrd "$WORK/initramfs.cpio" \
    -append "console=ttyS0,115200 rdinit=/init" -nic none \
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
expect 'NVT-PWM-CHIP: [0-9]' 'pwm-na51055 registers a pwmchip'
expect 'NVT-PWM-CONFIG: ok' 'period 1 ms, duty 50 %'
expect 'NVT-PWM-PERIOD: 0x00309800 0x00753A00' 'base period 30000, fall 15000 in PERIOD and its extension'
expect 'NVT-PWM-ENABLE: 0x00000001' 'enable starts channel 0'
expect 'NVT-PWM-DISABLE: 0x00000000' 'disable returns on the stop interrupt and stops it'
expect 'NVT-PWM-CYCLES: 0x00000000' 'a 20-cycle run ends on its own, enable waiting for it'
expect 'NVT-PWM-OK' 'PID 1 ran to the end'
if [ $rc -ne 0 ]; then
    echo "FAIL: QEMU exited $rc (124: a wait never ended)"
    fail=1
fi
exit $fail
