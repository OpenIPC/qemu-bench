#!/bin/bash
# Build the NT98321 / NT98323 (NA51068) SDK's U-Boot, kernel and board
# device tree, and lay them out as the board's 32 MiB SPI NOR image, for
# -M nt98323.  The SDK is vendor code under NDA terms, so nothing here is
# in CI.
#
#   bash qemu-boot/build-na51068-sdk.sh --sdk .../na51068_linux_sdk \
#       --toolchain .../arm-ca9-linux-gnueabihf-8.4 --out na51068 \
#       [--initrd rootfs.ramdisk.bin.raw]
#
# --sdk is the NT9832x SDK's software/board/na51068_linux_sdk.tar.bz2
# unpacked.  --toolchain is a directory with bin/<prefix>gcc, an ARMv7
# hard-float Linux toolchain: the NT9856x SDK's arm-ca9-linux-gnueabihf-8.4
# works.  Everything is built in --out; the SDK is not written.
#
# The kernel is na51068_evb_defconfig_nor_release (kernel 4.9.118), U-Boot
# nvt-na51068_evb_nor_defconfig (2016.07), the board
# cfg_98323_DDR4_4GX1_NOR_EVB (--cfg to pick another).  Things the SDK's
# own build hides:
#   - the defconfig's name does not end in _defconfig, so it is copied
#     to .config and olddefconfig'd;
#   - its host dtc predates GCC 10's -fno-common (HOSTCFLAGS=-fcommon);
#   - the OTP driver links a shipped object by a path relative to the
#     object tree (drivers/soc/nvt/otp/na51068/otp_protected.o_shipped),
#     which an out-of-tree build does not have: the *.o_shipped files are
#     copied in;
#   - the SDK has no ModelConfig.mk for any board, which U-Boot's
#     board/novatek/nvt-na51068/config.mk includes for its addresses: the
#     script writes one from the board's nvt-na51068-mem-tbl.dtsi (U-Boot
#     at 0x1E000000, SHMINFO at 0xA00000), SPI NOR, ramdisk root, the
#     environment in the uenv partition;
#   - U-Boot (560 KiB) does not fit the 448 KiB uboot partition plain: it
#     is packed as the SDK does, the SDK's build/nvt-tools/bfc ("c lz",
#     a BCL1 image), which the machine's loader stand-in unpacks.
#
# nor.bin is the board's nvt-na51068-storage-partition.dtsi layout:
#   0x000000 loader (--loader FILE; else erased: the machine stands in)
#   0x010000 fdt     nt98323.dtb
#   0x050000 uboot   u-boot.lz.bin
#   0x0C0000 uenv    (erased: U-Boot's default environment)
#   0x0D0000 kernel  uImage (load and entry 0x8000)
#   0x4E0000 rootfs  --initrd FILE, a legacy RAMDisk uImage (U-Boot's
#                    RAMDISK boot); the NT9852x SDK's
#                    output/raw/rootfs.ramdisk.bin.raw (LZ4 cpio, ARMv7)
#                    boots to a shell, the kernel having CONFIG_RD_LZ4
#
# --nand builds the NT98321 NAND EVB instead, cfg_98321_DDR3_4GX1_NAND_EVB
# (512 MB, the SPI020 in NAND mode): nvt-na51068_evb_nand_defconfig,
# na51068_evb_defconfig_release (CONFIG_MTD_SPI020NAND), the environment
# in NAND, and a 128 MiB nand.bin for the machine's W25N01GV laid out by
# that board's partitions (fdt 0x40000, uboot 0xC0000, kernel 0x300000,
# rootfs 0x760000).  Either kernel also gets CONFIG_RD_LZ4 for the test
# ramdisk.
#
# --loader FILE puts a first-stage loader at flash 0 (either image), for
# the machine's real-loader boot: qemu-boot/build-novatek-loader.sh
# nt98323 builds the SDK's own as a NOR one, nt98321 as the NAND EVB's.
#
# --test-drivers turns on the kernel drivers the SDK leaves off, so that
# the machine's models get exercised: the watchdog, the PWM, the IR
# receiver, the crypto / hash / RSA engines with the crypto self-tests,
# EHCI built in, dmatest, i2c-dev and at24.
#
# Output: u-boot.bin, u-boot.lz.bin, zImage, uImage, nt98323.dtb (the
# board's tree, whichever board), modules/*.ko, nor.bin or nand.bin.  Then:
#   qemu-system-arm -M nt98323,flash-file=na51068/nor.bin -nographic \
#       -serial mon:stdio -nic user
# or, without U-Boot:
#   qemu-system-arm -M nt98323 -kernel na51068/zImage \
#       -dtb na51068/nt98323.dtb -initrd <a RAMDisk uImage or cpio>
set -euo pipefail

SDK=
TC=
OUT=na51068
CFG=
KCFG=
INITRD=
NAND=
LOADER=
TESTDRV=
while [ $# -gt 0 ]; do
    case "$1" in
        --sdk) SDK=$2; shift 2 ;;
        --toolchain) TC=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        --cfg) CFG=$2; shift 2 ;;
        --kernel-config) KCFG=$2; shift 2 ;;
        --initrd) INITRD=$2; shift 2 ;;
        --nand) NAND=1; shift ;;
        --test-drivers) TESTDRV=1; shift ;;
        --loader) LOADER=$(realpath "$2"); shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done
if [ -z "$SDK" ] || [ -z "$TC" ]; then
    sed -n '2,70p' "$0" >&2
    exit 2
fi
if [ -n "$NAND" ]; then
    CFG=${CFG:-cfg_98321_DDR3_4GX1_NAND_EVB}
    KCFG=${KCFG:-na51068_evb_defconfig_release}
    UCFG=nvt-na51068_evb_nand_defconfig
    EMBMEM=EMBMEM_SPI_NAND; BLK=0x20000; ENVSTORE=NAND; CHIP=98321
else
    CFG=${CFG:-cfg_98323_DDR4_4GX1_NOR_EVB}
    KCFG=${KCFG:-na51068_evb_defconfig_nor_release}
    UCFG=nvt-na51068_evb_nor_defconfig
    EMBMEM=EMBMEM_SPI_NOR; BLK=0x10000; ENVSTORE=NOR; CHIP=98323
fi
[ -n "$INITRD" ] && INITRD=$(cd "$(dirname "$INITRD")" && pwd)/$(basename "$INITRD")
SDK=$(cd "$SDK" && pwd)
TC=$(cd "$TC" && pwd)
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
KSRC=$SDK/BSP/linux-kernel
KOUT=$OUT/linux
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
MEMTBL=$SDK/configs/Linux/$CFG/nvt-na51068-mem-tbl.dtsi

GCC=$(ls "$TC"/bin/*-gcc | head -1)
export CROSS_COMPILE=${GCC%gcc}
export PATH=$TC/bin:$PATH
export ARCH=arm
KMAKE=(make -C "$KSRC" O="$KOUT" SHELL=/bin/bash CONFIG_SHELL=/bin/bash
       HOSTCFLAGS=-fcommon)

echo "== board tree ($CFG)"
(cd "$SDK/configs/Linux/$CFG" &&
 cpp -nostdinc -D__DTS__ -I "$KSRC/include" \
     -I "$KSRC/arch/arm/boot/dts/include" -I "$SDK/configs/include" \
     -undef -x assembler-with-cpp nvt-na51068-evb.dts) > "$OUT/nt98323.tmp.dts"
dtc -q -O dtb -b 0 -o "$OUT/nt98323.dtb" "$OUT/nt98323.tmp.dts"

echo "== kernel ($KCFG)"
mkdir -p "$KOUT"
cp "$KSRC/arch/arm/configs/$KCFG" "$KOUT/.config"
# the test userland's ramdisk is LZ4 (the NOR config has it, not NAND's)
"$KSRC/scripts/config" --file "$KOUT/.config" --enable BLK_DEV_INITRD \
    --enable RD_LZ4
if [ -n "$TESTDRV" ]; then
    "$KSRC/scripts/config" --file "$KOUT/.config" \
        --enable USB_EHCI_HCD --enable WATCHDOG_CORE \
        --enable NOVATEK_IVOT_WATCHDOG --enable PWM --enable PWM_NA51068 \
        --enable PWM_SYSFS --enable MEDIA_SUPPORT --enable MEDIA_RC_SUPPORT \
        --enable RC_CORE --enable RC_DEVICES --enable IR_NVT_BK \
        --enable CRYPTO_DEV_NA51068_CRYPTO --enable CRYPTO_DEV_NA51068_HASH \
        --enable CRYPTO_DEV_NA51068_RSA --disable CRYPTO_MANAGER_DISABLE_TESTS \
        --enable DMATEST --enable I2C_CHARDEV --enable EEPROM_AT24
fi
"${KMAKE[@]}" olddefconfig >/dev/null
(cd "$KSRC" && find . -name '*.o_shipped' -not -path './scripts/*') |
    while read -r f; do
        mkdir -p "$KOUT/$(dirname "$f")"
        cp "$KSRC/$f" "$KOUT/$f"
    done
"${KMAKE[@]}" -j"$(nproc)" zImage modules >"$OUT/linux.log" 2>&1 ||
    { tail -20 "$OUT/linux.log"; exit 1; }
cp "$KOUT/arch/arm/boot/zImage" "$OUT/"
mkdir -p "$OUT/modules"
find "$KOUT" -name '*.ko' -exec cp {} "$OUT/modules/" \;

echo "== U-Boot ($UCFG)"
# ModelConfig.mk from the board's memory table: "name { reg = <a s>; };"
reg() {
    sed -n "s/^[[:space:]]*$1[[:space:]]*{[[:space:]]*reg = <\(0x[0-9A-Fa-f]*\) \(0x[0-9A-Fa-f]*\)>.*/\1 \2/p" \
        "$MEMTBL" | head -1
}
mkdir -p "$OUT/cfg"
{
    for r in dram:DRAM fdt:FDT loader:LOADER shmem:SHMEM \
             linuxtmp:LINUXTMP uboot:UBOOT; do
        set -- $(reg "${r%%:*}")
        [ $# -eq 2 ] || { echo "no ${r%%:*} in $MEMTBL" >&2; exit 1; }
        echo "BOARD_${r#*:}_ADDR = $1"
        echo "BOARD_${r#*:}_SIZE = $2"
    done
    # Linux's share: the tree's /memory node
    set -- $(sed -n 's/.*memory {.*reg = <\(0x[0-9A-Fa-f]*\) \(0x[0-9A-Fa-f]*\)>.*/\1 \2/p' \
             "$MEMTBL" | head -1)
    echo "BOARD_LINUX_ADDR = $1"
    echo "BOARD_LINUX_SIZE = $2"
    echo "BOARD_LINUX_MAXBLK_ADDR = $1"
    echo "BOARD_LINUX_MAXBLK_SIZE = $2"
    cat <<'MK'
BOARD_EXTDRAM_ADDR = 0x00000000
BOARD_EXTDRAM_SIZE = 0x00000000
NVT_LINUX_SMP = NVT_LINUX_SMP_ON
NVT_ROOTFS_TYPE = NVT_ROOTFS_TYPE_RAMDISK
MK
    echo "CHIP = $CHIP"
    echo "BIN_NAME = FW${CHIP}A"
    echo "BIN_NAME_T = FW${CHIP}T"
    echo "EMBMEM_BLK_SIZE = $BLK"
    echo "EMBMEM = $EMBMEM"
    echo "NVT_UBOOT_ENV_IN_STORG_SUPPORT = NVT_UBOOT_ENV_IN_STORG_SUPPORT_$ENVSTORE"
} > "$OUT/cfg/ModelConfig.mk"
export NVT_PRJCFG_MODEL_CFG=$OUT/cfg/ModelConfig.mk
rsync -a --delete "$SDK/BSP/u-boot/" "$OUT/u-boot/"
make -C "$OUT/u-boot" "$UCFG" >/dev/null
make -C "$OUT/u-boot" -j"$(nproc)" >"$OUT/u-boot.log" 2>&1 ||
    { tail -20 "$OUT/u-boot.log"; exit 1; }
cp "$OUT/u-boot/u-boot.bin" "$OUT/"
# as build/Makefile's uboot target: the header checksum, then LZ
"$SDK/build/nvt-tools/encrypt_bin" SUM "$OUT/u-boot.bin" 0x350 ub51068 \
    >/dev/null
"$SDK/build/nvt-tools/bfc" c lz "$OUT/u-boot.bin" "$OUT/u-boot.lz.bin" 0 0 \
    >/dev/null

"$OUT/u-boot/tools/mkimage" -A arm -O linux -T kernel -C none -a 0x8000 \
    -e 0x8000 -n Linux -d "$OUT/zImage" "$OUT/uImage" >/dev/null
# the board's partitions: "label = "x"; reg = <0x0 off 0x0 size>;"
part() {
    sed -n "s/.*label = \"$1\";[[:space:]]*reg = <0x0 \(0x[0-9A-Fa-f]*\) .*/\1/p" \
        "$SDK/configs/Linux/$CFG/nvt-na51068-storage-partition.dtsi" | head -1
}
for p in fdt uboot kernel rootfs; do
    [ -n "$(part $p)" ] || { echo "no partition $p in $CFG" >&2; exit 1; }
done
args=("$OUT/nt98323.dtb@$(part fdt)" "$OUT/u-boot.lz.bin@$(part uboot)"
      "$OUT/uImage@$(part kernel)")
[ -n "$INITRD" ] && args+=("$INITRD@$(part rootfs)")
[ -n "$LOADER" ] && args+=("$LOADER@0x0")
if [ -n "$NAND" ]; then
    IMG=$OUT/nand.bin; SIZE=128M
else
    IMG=$OUT/nor.bin; SIZE=32M
fi
echo "== $(basename "$IMG")"
python3 "$SCRIPT_DIR/xmimg2flash.py" -o "$IMG" --size $SIZE \
    "${args[@]}" >/dev/null

ls -l "$OUT"/zImage "$OUT"/uImage "$OUT"/nt98323.dtb "$OUT"/u-boot.lz.bin \
    "$IMG"
