#!/bin/bash
# Build the OpenIPC nt9856x kernel with the drivers it leaves out, for
# exercising the emulated USB controller (nvt-usb) and PWM (nvt-pwm) with
# the vendor's own drivers.
#
# Neither the OpenIPC release nor the vendor SDK firmware for NT9856x
# builds a USB host controller driver or a PWM driver (CONFIG_PWM is off
# in both), so nothing shipped can test them.
# This builds what OpenIPC builds -- openipc/linux branch novatek-nt9856x,
# OpenIPC's nt98566.generic.config, its toolchain, in-tree like Buildroot
# does (the OTP driver links a shipped object that an out-of-tree build
# drops) -- plus:
#
#   CONFIG_USB_EHCI_HCD=y      with it USB_EHCI_HCD_NVTIVOT: ehci-na51089.c
#   CONFIG_USB_STORAGE=y, CONFIG_SCSI=y, CONFIG_BLK_DEV_SD=y
#   CONFIG_PWM=y, CONFIG_PWM_NA51055=y, CONFIG_PWM_SYSFS=y: pwm-na51055.c
#
#   bash qemu-boot/build-novatek-test-kernel.sh [--out DIR] [--ref REF]
#       [--cross PREFIX]     (default: download OpenIPC's toolchain)
#
# The result is DIR/uImage.nt9856x-test, for qemu-boot/test-novatek-usb.sh
# and qemu-boot/test-novatek-pwm.sh.
# Downloads about 300 MB and needs about 2 GB of disk in DIR.
set -euo pipefail

OUT=$PWD/novatek-test-kernel
REF=novatek-nt9856x
CROSS=
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT=$2; shift 2 ;;
        --ref) REF=$2; shift 2 ;;
        --cross) CROSS=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)         # --out may be relative: we cd into it
cd "$OUT"

GH=https://github.com/openipc
RAW=https://raw.githubusercontent.com/OpenIPC/firmware/master

# Unpack a tarball into DIR only if it all arrives: a failed or partial
# download leaves no DIR, so the next run fetches it again.
fetch() {
    local url=$1 dir=$2

    [ -d "$dir" ] && return
    rm -rf "$dir.part"
    mkdir "$dir.part"
    curl -fsSL "$url" | tar xz -C "$dir.part" --strip-components=1
    mv "$dir.part" "$dir"
}

if [ -z "$CROSS" ]; then
    echo "OpenIPC's novatek-nt9856x toolchain"
    fetch "$GH/firmware/releases/download/toolchain/toolchain.novatek-nt9856x.tgz" \
        toolchain
    gcc=$(find "$OUT/toolchain/bin" -name '*-gcc' -type f -o -name '*-gcc' -type l |
          grep -v -- '-cc1' | head -1)
    CROSS=${gcc%gcc}
fi
echo "Cross compiler: ${CROSS}gcc"

echo "openipc/linux $REF"
fetch "$GH/linux/archive/$REF.tar.gz" linux

curl -fsSL "$RAW/br-ext-chip-novatek/board/nt9856x/nt98566.generic.config" \
    -o linux/.config
cat >> linux/.config <<'EOF'
CONFIG_USB_EHCI_HCD=y
CONFIG_USB_EHCI_HCD_NVTIVOT=y
CONFIG_USB_STORAGE=y
CONFIG_SCSI=y
CONFIG_BLK_DEV_SD=y
CONFIG_USB_ANNOUNCE_NEW_DEVICES=y
CONFIG_PWM=y
CONFIG_PWM_NA51055=y
CONFIG_PWM_SYSFS=y
EOF

make -C linux ARCH=arm CROSS_COMPILE="$CROSS" olddefconfig >/dev/null
for sym in USB_EHCI_HCD USB_EHCI_HCD_NVTIVOT USB_STORAGE BLK_DEV_SD \
           PWM PWM_NA51055 PWM_SYSFS; do
    grep -q "^CONFIG_$sym=y" linux/.config ||
        { echo "CONFIG_$sym did not stick" >&2; exit 1; }
done
make -C linux ARCH=arm CROSS_COMPILE="$CROSS" -j"$(nproc)" \
    LOADADDR=0x8000 uImage
cp linux/arch/arm/boot/uImage uImage.nt9856x-test
echo "Built $OUT/uImage.nt9856x-test"
