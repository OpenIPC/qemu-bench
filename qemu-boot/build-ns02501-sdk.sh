#!/bin/bash
# Build the NT98532 / NT98533 (NS02501) SDK's U-Boot, kernel and board
# device tree, and lay them out as the SDK's 16 MiB SPI NOR image, for
# -M nt98533.  The SDK is vendor code under NDA terms, so nothing here is
# in CI.
#
#   bash qemu-boot/build-ns02501-sdk.sh --sdk .../ns02501_linux_sdk \
#       --toolchain .../arm-ca9-linux-gnueabihf-8.4 --out ns02501
#
# --toolchain is a directory with bin/<prefix>gcc, an ARMv7 hard-float
# Linux toolchain: the NT9856x SDK's arm-ca9-linux-gnueabihf-8.4 works (the
# NS02501 SDK names arm-ca32-linux-gnueabihf-10.4.0, which it does not
# ship).  The sources are copied to --out; the SDK is not written to.
#
# The board is cfg_532_IPC_NOR_SQUASHFS_BD_EVB, the SDK's NOR layout:
#   0x00000 loader (left erased: the machine stands in for it)
#   0x10000 fdt     nvt-evb.dtb
#   0x50000 uboot   u-boot.bin, plain (nvt-ns02501_a32_nor_defconfig)
#   0xF0000 kernel  uImage (ns02501_ipc_a32_evb_defconfig_nor_release)
#   0x470000 rootfs (--rootfs FILE: a squashfs, optional)
# The SDK ships a ModelConfig.mk only for a NAND board; the script writes
# a NOR one beside the copied board files.
#
# Output: u-boot.bin, zImage, uImage, nvt-evb.dtb, modules/*.ko (SD and
# Ethernet are modules), nor.bin.
set -euo pipefail

SDK=
TC=
OUT=ns02501
ROOTFS=
CFG=cfg_532_IPC_NOR_SQUASHFS_BD_EVB
while [ $# -gt 0 ]; do
    case "$1" in
        --sdk) SDK=$2; shift 2 ;;
        --toolchain) TC=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --rootfs) ROOTFS=$2; shift 2 ;;
        --cfg) CFG=$2; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ -z "$SDK" ] || [ -z "$TC" ]; then
    sed -n '2,25p' "$0" >&2
    exit 2
fi
SDK=$(cd "$SDK" && pwd)
TC=$(cd "$TC" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

GCC=$(ls "$TC"/bin/*-gcc | head -1)
export CROSS_COMPILE=${GCC%gcc}
export PATH=$TC/bin:$PATH
export ARCH=arm
JOBS=$(nproc)

echo "== board files ($CFG)"
rm -rf "$OUT/cfg"
cp -r "$SDK/configs/Linux/$CFG" "$OUT/cfg"
sed -e 's/^EMBMEM_BLK_SIZE.*/EMBMEM_BLK_SIZE = 0x10000/' \
    -e 's/^EMBMEM = .*/EMBMEM = EMBMEM_SPI_NOR/' \
    -e 's/^NVT_ROOTFS_TYPE.*/NVT_ROOTFS_TYPE = NVT_ROOTFS_TYPE_NOR_SQUASH/' \
    -e 's/^NVT_CFG_UBOOT_CFG.*/NVT_CFG_UBOOT_CFG = nvt-ns02501_a32_nor_defconfig/' \
    -e 's/^BIN_NAME = .*/BIN_NAME = FW98532A/' \
    -e 's/^BIN_NAME_T = .*/BIN_NAME_T = FW98532T/' \
    "$SDK/configs/Linux/cfg_533_IPC_NAND_RAMDISK_QD_EVB/ModelConfig.mk" \
    > "$OUT/cfg/ModelConfig.mk"
export NVT_PRJCFG_MODEL_CFG=$OUT/cfg/ModelConfig.mk
(cd "$OUT/cfg" &&
 cpp -nostdinc -D__DTS__ -I "$SDK/BSP/linux-kernel/include" \
     -I "$SDK/configs/include" -undef -x assembler-with-cpp nvt-evb.dts \
     > nvt-evb.tmp.dts &&
 dtc -q -@ -O dtb -b 0 -o "$OUT/nvt-evb.dtb" nvt-evb.tmp.dts)

echo "== U-Boot"
rsync -a --delete "$SDK/BSP/u-boot/" "$OUT/u-boot/"
make -C "$OUT/u-boot" nvt-ns02501_a32_nor_defconfig >/dev/null
make -C "$OUT/u-boot" -j"$JOBS" >"$OUT/u-boot.log" 2>&1 ||
    { tail -20 "$OUT/u-boot.log"; exit 1; }
cp "$OUT/u-boot/u-boot.bin" "$OUT/"

echo "== kernel"
rsync -a --delete "$SDK/BSP/linux-kernel/" "$OUT/linux/"
cp "$OUT/linux/arch/arm/configs/ns02501_ipc_a32_evb_defconfig_nor_release" \
    "$OUT/linux/.config"
# The kernel's version header uses bash's echo -e.
KMAKE=(make -C "$OUT/linux" SHELL=/bin/bash CONFIG_SHELL=/bin/bash)
"${KMAKE[@]}" olddefconfig >/dev/null
"${KMAKE[@]}" -j"$JOBS" uImage modules LOADADDR=0x8000 \
    >"$OUT/linux.log" 2>&1 || { tail -20 "$OUT/linux.log"; exit 1; }
cp "$OUT/linux/arch/arm/boot/zImage" "$OUT/linux/arch/arm/boot/uImage" "$OUT/"
mkdir -p "$OUT/modules"
find "$OUT/linux" -name '*.ko' -exec cp {} "$OUT/modules/" \;

echo "== nor.bin"
args=("$OUT/nvt-evb.dtb@0x10000" "$OUT/u-boot.bin@0x50000"
      "$OUT/uImage@0xF0000")
[ -n "$ROOTFS" ] && args+=("$ROOTFS@0x470000")
python3 "$SCRIPT_DIR/xmimg2flash.py" -o "$OUT/nor.bin" --size 16M "${args[@]}"
ls -l "$OUT"/u-boot.bin "$OUT"/zImage "$OUT"/uImage "$OUT"/nvt-evb.dtb \
    "$OUT"/nor.bin
