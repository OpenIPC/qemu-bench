#!/bin/bash
# Build a Novatek first-stage loader (LD98xxxA.bin) from the SDK's own
# loader source, with the SDK's makefiles and its arm-eabi toolchain, for
# the machines' real-loader flash boot: the mask ROM model copies it from
# the flash into SRAM and runs it, and it brings DDR up and loads U-Boot.
#
#   build-novatek-loader.sh <machine> <out.bin>
#
# <machine> picks the source and the model settings:
#
#   nt98525   NT9852X_SDK_FULL_from_Marcin_W.tar.gz na51055_ldr, MODEL
#             EMU_EVB (CHIP 525, DDR3, SPI NAND) -- LD98525A.bin
#   nt98528   the same, MODEL EMU_EVB_528 and LOADER_TYPE
#             STAND_ALONE_LOADER_528 -- LD98528A.bin
#   nt98533   nt98533/ns02501_ldr, MODEL EMU_EVB with CHIP 533QD (the
#             SDK's 533QC loader checks for the other DDR die) --
#             LD98533A.bin
#   nt98321   NT9832x_SDK_release_v1.02.024 na51068_loader_v1.08.014,
#             MODEL 321_DDR3_4Gx1_NAND_SYS (SPI NAND) -- LD98321A.bin
#   nt98323   the same source, a NOR model derived from
#             321_DDR4_8GX1_NAND_SYS: CHIP 323, one 4 Gb DDR4 die,
#             DRAM_MUX FB, STORAGEINT SpiFlash -- LD98323A.bin
#
# The NT9856x (nt98562 / nt98566) SDKs ship their loaders prebuilt
# (LD98562A.bin / LD98566A.bin), and NA51103 (nt98332) has none: its
# loader comes with the board's flash.
#
# Environment: SDK (default /home/ai/dimerr/local/ntk/sdk, the directory
# with the archives), TOOLCHAIN (default $SDK/toolchains/gcc-linaro-6.4.1-
# 2018.05-x86_64_arm-eabi/bin), WORK (a scratch directory; default a new
# one under ${TMPDIR:-/tmp}, removed afterwards).
set -euo pipefail

usage() { sed -n '2,/^set -e/{/^#/s/^# \{0,1\}//p}' "$0"; exit 1; }
[ $# -eq 2 ] || usage
MACHINE=$1
OUT=$(realpath -m "$2")
SDK=${SDK:-/home/ai/dimerr/local/ntk/sdk}
TOOLCHAIN=${TOOLCHAIN:-$SDK/toolchains/gcc-linaro-6.4.1-2018.05-x86_64_arm-eabi/bin}
[ -x "$TOOLCHAIN/arm-eabi-gcc" ] || { echo "no arm-eabi-gcc in $TOOLCHAIN" >&2; exit 1; }

if [ -n "${WORK:-}" ]; then
    mkdir -p "$WORK"
else
    WORK=$(mktemp -d "${TMPDIR:-/tmp}/nvt-loader.XXXXXX")
    trap 'rm -rf "$WORK"' EXIT
fi

# set KEY = VALUE in a ModelConfig file
setcfg() {
    grep -q "^$2 = " "$1" || { echo "$1: no $2" >&2; exit 1; }
    sed -i "s|^$2 = .*|$2 = $3|" "$1"
}

case $MACHINE in
nt98525|nt98528)
    tar xzf "$SDK/NT9852X_SDK_FULL_from_Marcin_W.tar.gz" -C "$WORK" \
        NT9852x_SDK_Package/na51055_ldr
    L=$WORK/NT9852x_SDK_Package/na51055_ldr
    CHIP=${MACHINE#nt98}
    if [ "$CHIP" = 528 ]; then
        # as the SDK's make_combo_loader.sh builds the stand-alone 528 one
        MODEL=EMU_EVB_528
        sed -i 's/^#define LOADER_TYPE[ \t].*/#define LOADER_TYPE                     STAND_ALONE_LOADER_528/' \
            "$L/LibExt/LIBExt_src/Ctrl_Flow/bl_func.h"
    else
        MODEL=EMU_EVB
    fi
    ;;
nt98533)
    cp -r "$SDK/nt98533/ns02501_ldr" "$WORK/"
    L=$WORK/ns02501_ldr
    MODEL=EMU_EVB
    setcfg "$L/Project/Model/ModelConfig_EMU_EVB.txt" CHIP 533QD
    CHIP=533QD
    ;;
nt98321|nt98323)
    tar xjf "$SDK/NT9832x_SDK_release_v1.02.024_ns_uclib.tar.bz2" -C "$WORK" \
        --wildcards 'NT9832x_SDK/software/board/loader/source_code/na51068_loader_*.tar.bz2'
    tar xjf "$WORK"/NT9832x_SDK/software/board/loader/source_code/na51068_loader_*.tar.bz2 \
        -C "$WORK"
    L=$WORK/na51068_loader
    M=$L/Project/Model
    if [ "$MACHINE" = nt98321 ]; then
        MODEL=321_DDR3_4Gx1_NAND_SYS
        CHIP=321
    else
        MODEL=323_DDR4_4GX1_NOR_EVB
        CHIP=323
        cp "$M/ModelConfig_321_DDR4_8GX1_NAND_SYS.txt" "$M/ModelConfig_$MODEL.txt"
        setcfg "$M/ModelConfig_$MODEL.txt" CHIP 323
        setcfg "$M/ModelConfig_$MODEL.txt" SDRAM_SIZE 4096
        setcfg "$M/ModelConfig_$MODEL.txt" STORAGEINT SpiFlash
        setcfg "$M/ModelConfig_$MODEL.txt" DRAM_MUX FB
    fi
    ;;
*)
    usage
    ;;
esac

setcfg "$L/Project/Model/ModelConfig.txt" MODEL "$MODEL"
echo "== $MACHINE: MODEL $MODEL in $L"
# clean first: the SDK archives carry prebuilt outputs (NA51055's
# LD98525A.bin is the eMMC one), which make would otherwise keep
rm -f "$L"/Project/Model/Loader*_Data/Release/LD98*.bin
(cd "$L/MakeCommon" && PATH=$TOOLCHAIN:$PATH &&
    make clean > "$WORK/build.log" 2>&1 &&
    make release CROSS_COMPILE="$TOOLCHAIN/arm-eabi-" >> "$WORK/build.log" 2>&1) || {
    tail -20 "$WORK/build.log" >&2
    exit 1
}
BIN=$(ls "$L"/Project/Model/Loader"$CHIP"_Data/Release/LD98*A.bin)
mkdir -p "$(dirname "$OUT")"
cp "$BIN" "$OUT"
echo "== $OUT ($(stat -c %s "$OUT") bytes)"
