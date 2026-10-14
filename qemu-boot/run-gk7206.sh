#!/bin/bash
#
# Boot the Goke GK7206V12A (codename "xmorca", ShiMetaPi Pico-G1 SoC) in QEMU.
#
# The machine is gk7206v12a (2Gb DDR3L, 256 MiB) to match the SDK target the
# kernel below is built for.  MACHINE=gk7206v10 selects the V10 model (same
# die map, 64 MiB of its own).
#
# Boots the vendor XMedia SDK kernel (linux-5.10.y, xmorca_defconfig) to full
# Linux userspace on the emulated dual-Cortex-A7 MP2.  The kernel image is a
# plain zImage with the vendor xmorca.dtb appended (CONFIG_ARM_APPENDED_DTB);
# QEMU's hisilicon machine patches that DTB in memory at load (GIC CPU-iface
# size, disables lotus,sdhci + NAND) — the on-disk image stays vendor-original.
#
# Build the kernel + dtb from the SDK (github.com/ShiMetaPi/shimetapi_pico_gx):
#   ./run.sh lunch                 # pick xm7206v12a + linux-5.10
#   # build zImage + dtb, skipping the proprietary GMP media driver:
#   PATH=<sdk>/tools/linux/toolchains/arm-gcc12.2.0-linux-uclibceabi/bin:$PATH \
#   make -C source/kernel/linux-5.10.y O=out/xm7206v12a/linux-5.10.y ARCH=arm \
#        CROSS_COMPILE=arm-gcc12.2.0-linux-uclibceabi- CONFIG_GMP= -j zImage
#   cat out/.../arch/arm/boot/zImage out/.../lotus/machine/xmorca/dts/xmorca.dtb \
#       > qemu-boot/gk7206/zImage-xmorca-dtb
#
# The rootfs is the SDK's real busybox userspace, packed as an initramfs cpio.
# NB: the vendor kernel has no CONFIG_RD_GZIP, so the cpio MUST be
# UNCOMPRESSED.
#
# Two boot modes (set FLASH=1 for the second):
#
#  FLASH unset (default) — INITRAMFS boot.  Root is the SDK busybox rootfs
#    packed as an uncompressed cpio (initramfs.cpio = tiny banner init).
#    INITRD/RDINIT override the userspace.  The unpacked busybox rootfs
#    needs more than the machine's default mem=32M, so this mode always
#    runs with -m 256M (whatever MACHINE is), gives Linux 200M and moves
#    the xmedia MMZ above it (0x4c800000, the remaining 56M).
#
#  FLASH=1 — SPI-NOR FLASH boot.  Attaches a 16 MiB NOR image (nor-16m.img)
#    to the emulated fmc100 flash controller; the kernel enumerates the chip
#    (W25Q128), parses the "sfc:" mtdparts, and mounts root=/dev/mtdblock4
#    (jffs2) straight from flash — exactly like the real board's bootargs.
#    Build the image with mk-cv610-nor.sh-style tooling: 16 MiB of 0xFF with
#    the jffs2 rootfs placed at offset 0x500000 (after boot/bootargs/bl31/
#    kernel partitions).
#
# Ctrl-A X to quit QEMU.

set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
QEMU="${QEMU:-$ROOT/qemu-src/build/qemu-system-arm}"
KERNEL="${KERNEL:-$HERE/gk7206/zImage-xmorca-dtb}"
MACHINE="${MACHINE:-gk7206v12a}"

if [ "${FLASH:-0}" = 1 ]; then
    NOR="${NOR:-$HERE/gk7206/nor-16m.img}"
    exec "$QEMU" -M "$MACHINE",flash-file="$NOR" -nographic -display none \
        -serial mon:stdio -monitor none \
        -kernel "$KERNEL" \
        -append "root=/dev/mtdblock4 rootfstype=jffs2 rw mtdparts=sfc:512K(boot),256K(bootargs),256K(bl31),4M(kernel),11M(rootfs)" \
        -d unimp,guest_errors \
        "$@"
fi

INITRD="${INITRD:-$HERE/gk7206/rootfs.cpio}"
RDINIT="${RDINIT:-/sbin/init}"

exec "$QEMU" -M "$MACHINE" -m 256M -nographic -display none \
    -serial mon:stdio -monitor none \
    -kernel "$KERNEL" \
    -initrd "$INITRD" \
    -append "mem=200M mmz_allocator=xmedia mmz=anonymous,0,0x4c800000,56M rdinit=$RDINIT" \
    -d unimp,guest_errors \
    "$@"
