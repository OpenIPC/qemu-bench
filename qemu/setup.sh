#!/bin/bash
#
# Set up QEMU build tree with HiSilicon CV300/EV300 SoC emulation.
# Run from the repository root:
#
#   bash qemu/setup.sh
#
# This will:
#   1. Clone QEMU 10.2.0 into qemu-src/
#   2. Copy all HiSilicon source files
#   3. Patch Kconfig and meson.build
#   4. Configure and build (arm target only)
#
set -e

QEMU_VERSION="v10.2.0"
QEMU_DIR="qemu-src"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

# ── 1. Clone QEMU ──────────────────────────────────────────────────────
if [ -d "$QEMU_DIR" ]; then
    echo "qemu-src/ already exists, skipping clone"
else
    echo "Cloning QEMU $QEMU_VERSION..."
    git clone --depth 1 --branch "$QEMU_VERSION" \
        https://gitlab.com/qemu-project/qemu.git "$QEMU_DIR"
fi

# ── 2. Copy source files ───────────────────────────────────────────────
echo "Copying HiSilicon SoC sources..."

cp qemu/hw/arm/hisilicon.c          "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/hisilicon.h  "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/arm/xiongmai.c           "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/xiongmai.h   "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/xm-sfc.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/xm-sysctl.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/xm-ddrc.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/xm-h1.c             "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/xm-vi.c             "$QEMU_DIR/hw/misc/"
cp qemu/hw/i2c/xm-i2c.c             "$QEMU_DIR/hw/i2c/"
cp qemu/hw/net/xm-dwmac.c           "$QEMU_DIR/hw/net/"
cp qemu/include/hw/misc/hisi-fastboot.h "$QEMU_DIR/include/hw/misc/"
cp qemu/hw/misc/hisi-sysctl.c       "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-crg.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-fmc.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-himci.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-regbank.c      "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-sfc350.c      "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-vedu.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-mipi-rx.c      "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-rtc.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-ive.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-nnie.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-hwrng.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-vi-fp.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-fastboot.c     "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-gzip.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-l2cache.c     "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-xmsp804.c     "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-ddrc-v1.c     "$QEMU_DIR/hw/misc/"
cp qemu/hw/net/hisi-femac.c         "$QEMU_DIR/hw/net/"
cp qemu/hw/net/hisi-gmac.c          "$QEMU_DIR/hw/net/"
cp qemu/hw/net/hisi-hieth-sf.c      "$QEMU_DIR/hw/net/"
cp qemu/hw/i2c/hisi-i2c.c          "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx335.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx307.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx291.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx415.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx385.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-f37.c          "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-gc2053.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-sp2305.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-mis2006.c      "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-nvp6124b.c     "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-smartsens.c    "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-i2c-v1.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-i2c-dw.c       "$QEMU_DIR/hw/i2c/"
cp qemu/hw/ssi/hisi-spi.c          "$QEMU_DIR/hw/ssi/"
cp qemu/hw/ssi/hisi-imx122.c       "$QEMU_DIR/hw/ssi/"

# Apply patches to upstream QEMU files
for p in qemu/patches/*.patch; do
    [ -f "$p" ] && (cd "$QEMU_DIR" && git apply --check "../$p" 2>/dev/null && git apply "../$p") || true
done

# ── 3. Patch build system ──────────────────────────────────────────────
echo "Patching build system..."

# hw/arm/Kconfig
if ! grep -q HISILICON "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config HISILICON
    bool
    default y
    depends on TCG && ARM
    select ARM_GIC
    select PL190
    select PL011
    select ARM_TIMER
    select PL061
    select HISI_SPI
    select PL080
    select UNIMP
    select SDHCI
    select HISI_MISC
    select HISI_FEMAC
    select HISI_GMAC
    select HISI_HIETH_SF
    select I2C
    select HISI_I2C
    select CMSDK_APB_WATCHDOG
    select AHCI_SYSBUS
    select USB_EHCI_SYSBUS
    select USB_OHCI_SYSBUS
    select USB_XHCI_SYSBUS
KCONFIG
    echo "  patched hw/arm/Kconfig"
else
    # Existing tree may still have "select PL022" from an older setup.sh
    # — swap it for HISI_SPI inside the HISILICON block only.
    if grep -q '^    select PL022$' "$QEMU_DIR/hw/arm/Kconfig"; then
        sed -i '/^config HISILICON$/,/^$/ s/^    select PL022$/    select HISI_SPI/' \
            "$QEMU_DIR/hw/arm/Kconfig"
        echo "  hw/arm/Kconfig: swapped PL022 -> HISI_SPI in HISILICON block"
    else
        echo "  hw/arm/Kconfig already patched"
    fi
    # DVR/NVR additions: HISI_GMAC + HISI_HIETH_SF + AHCI / USB sysbus.
    for sym in HISI_GMAC HISI_HIETH_SF AHCI_SYSBUS USB_EHCI_SYSBUS USB_OHCI_SYSBUS USB_XHCI_SYSBUS; do
        if ! awk '/^config HISILICON$/,/^$/' "$QEMU_DIR/hw/arm/Kconfig" \
                | grep -q "select $sym"; then
            sed -i "/^config HISILICON$/,/^$/ {
                /^    select CMSDK_APB_WATCHDOG$/ a\\    select $sym
            }" "$QEMU_DIR/hw/arm/Kconfig"
            echo "  hw/arm/Kconfig: added select $sym to HISILICON"
        fi
    done
fi

# hw/arm/meson.build
if ! grep -q hisilicon "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_HISILICON', if_true: files('hisilicon.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  patched hw/arm/meson.build"
else
    echo "  hw/arm/meson.build already patched"
fi

# Xiongmai XM510/XM530/XM550: a family of its own (hw/arm/xiongmai.c), not
# HiSilicon-derived silicon. One guarded append serves a fresh tree and an
# existing one alike, since nothing here predates it.
if ! grep -q "^config XIONGMAI$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config XIONGMAI
    bool
    default y
    depends on TCG && ARM
    select PL190
    select PL011
    select ARM_TIMER
    select CMSDK_APB_WATCHDOG
    select HISI_MISC
    select SDHCI
    select I2C
    select HISI_I2C
KCONFIG
    echo "  hw/arm/Kconfig: added XIONGMAI"
fi
if ! grep -q xm-sfc "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-sfc.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-sfc.c"
fi
if ! grep -q xm-sysctl "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-sysctl.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-sysctl.c"
fi
if ! grep -q xm-ddrc "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-ddrc.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-ddrc.c"
fi
if ! grep -q xm-h1 "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-h1.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-h1.c"
fi
if ! grep -q xm-vi "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-vi.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-vi.c"
fi
if ! grep -q xm-i2c "$QEMU_DIR/hw/i2c/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-i2c.c'))" \
        >> "$QEMU_DIR/hw/i2c/meson.build"
    echo "  hw/i2c/meson.build: added xm-i2c.c"
fi
if ! grep -q xm-dwmac "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-dwmac.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  hw/net/meson.build: added xm-dwmac.c"
fi
if ! grep -q xiongmai "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xiongmai.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added xiongmai.c"
fi

# hw/misc/Kconfig
if ! grep -q HISI_MISC "$QEMU_DIR/hw/misc/Kconfig"; then
    cat >> "$QEMU_DIR/hw/misc/Kconfig" <<'KCONFIG'

config HISI_MISC
    bool
KCONFIG
    echo "  patched hw/misc/Kconfig"
else
    echo "  hw/misc/Kconfig already patched"
fi

# hw/misc/meson.build
if ! grep -q hisi-sysctl "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_MISC', if_true: files('hisi-sysctl.c', 'hisi-crg.c', 'hisi-fmc.c', 'hisi-sfc350.c', 'hisi-himci.c', 'hisi-regbank.c', 'hisi-vedu.c', 'hisi-mipi-rx.c', 'hisi-rtc.c', 'hisi-ive.c', 'hisi-nnie.c', 'hisi-fastboot.c', 'hisi-gzip.c', 'hisi-hwrng.c', 'hisi-vi-fp.c', 'hisi-l2cache.c', 'hisi-xmsp804.c', 'hisi-ddrc-v1.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  patched hw/misc/meson.build"
else
    if ! grep -q hisi-hwrng "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-gzip.c'/'hisi-gzip.c', 'hisi-hwrng.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-hwrng.c"
    fi
    if ! grep -q hisi-vi-fp "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-hwrng.c'/'hisi-hwrng.c', 'hisi-vi-fp.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-vi-fp.c"
    fi
    if ! grep -q hisi-l2cache "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-vi-fp.c'/'hisi-vi-fp.c', 'hisi-l2cache.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-l2cache.c"
    fi
    if ! grep -q hisi-nnie "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-ive.c'/'hisi-ive.c', 'hisi-nnie.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-nnie.c"
    fi
    if ! grep -q hisi-xmsp804 "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-l2cache.c'/'hisi-l2cache.c', 'hisi-xmsp804.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-xmsp804.c"
    fi
    if ! grep -q hisi-ddrc-v1 "$QEMU_DIR/hw/misc/meson.build"; then
        sed -i "s/'hisi-xmsp804.c'/'hisi-xmsp804.c', 'hisi-ddrc-v1.c'/" \
            "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added hisi-ddrc-v1.c"
    fi
fi

# hw/net/Kconfig
if ! grep -q HISI_FEMAC "$QEMU_DIR/hw/net/Kconfig"; then
    cat >> "$QEMU_DIR/hw/net/Kconfig" <<'KCONFIG'

config HISI_FEMAC
    bool

config HISI_GMAC
    bool

config HISI_HIETH_SF
    bool
KCONFIG
    echo "  patched hw/net/Kconfig"
else
    if ! grep -q "config HISI_GMAC" "$QEMU_DIR/hw/net/Kconfig"; then
        cat >> "$QEMU_DIR/hw/net/Kconfig" <<'KCONFIG'

config HISI_GMAC
    bool
KCONFIG
        echo "  hw/net/Kconfig: added HISI_GMAC"
    fi
    if ! grep -q "config HISI_HIETH_SF" "$QEMU_DIR/hw/net/Kconfig"; then
        cat >> "$QEMU_DIR/hw/net/Kconfig" <<'KCONFIG'

config HISI_HIETH_SF
    bool
KCONFIG
        echo "  hw/net/Kconfig: added HISI_HIETH_SF"
    fi
fi

# hw/net/meson.build
if ! grep -q hisi-femac "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_FEMAC', if_true: files('hisi-femac.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  patched hw/net/meson.build (femac)"
else
    echo "  hw/net/meson.build already has femac"
fi
if ! grep -q hisi-gmac "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_GMAC', if_true: files('hisi-gmac.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  patched hw/net/meson.build (gmac)"
else
    echo "  hw/net/meson.build already has gmac"
fi
if ! grep -q hisi-hieth-sf "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_HIETH_SF', if_true: files('hisi-hieth-sf.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  patched hw/net/meson.build (hieth-sf)"
else
    echo "  hw/net/meson.build already has hieth-sf"
fi

# hw/i2c/Kconfig
if ! grep -q HISI_I2C "$QEMU_DIR/hw/i2c/Kconfig"; then
    cat >> "$QEMU_DIR/hw/i2c/Kconfig" <<'KCONFIG'

config HISI_I2C
    bool
    select I2C
KCONFIG
    echo "  patched hw/i2c/Kconfig"
else
    echo "  hw/i2c/Kconfig already patched"
fi

# hw/i2c/meson.build
if ! grep -q hisi-i2c "$QEMU_DIR/hw/i2c/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_I2C', if_true: files('hisi-i2c.c', 'hisi-i2c-v1.c', 'hisi-i2c-dw.c', 'hisi-imx335.c', 'hisi-imx307.c', 'hisi-imx291.c', 'hisi-imx385.c', 'hisi-imx415.c', 'hisi-f37.c', 'hisi-gc2053.c', 'hisi-sp2305.c', 'hisi-mis2006.c', 'hisi-nvp6124b.c', 'hisi-smartsens.c'))" \
        >> "$QEMU_DIR/hw/i2c/meson.build"
    echo "  patched hw/i2c/meson.build"
else
    # Existing tree — splice in any new files that aren't already listed.
    if ! grep -q hisi-i2c-v1 "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-i2c.c', 'hisi-imx335.c'/'hisi-i2c.c', 'hisi-i2c-v1.c', 'hisi-imx335.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-i2c-v1.c"
    fi
    if ! grep -q hisi-i2c-dw "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-i2c-v1.c', 'hisi-imx335.c'/'hisi-i2c-v1.c', 'hisi-i2c-dw.c', 'hisi-imx335.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-i2c-dw.c"
    fi
    if ! grep -q hisi-imx291 "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-imx307.c'/'hisi-imx307.c', 'hisi-imx291.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-imx291.c"
    fi
    if ! grep -q hisi-imx385 "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-imx291.c'/'hisi-imx291.c', 'hisi-imx385.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-imx385.c"
    fi
    if ! grep -q hisi-imx415 "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-imx385.c'/'hisi-imx385.c', 'hisi-imx415.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-imx415.c"
    fi
    if ! grep -q hisi-nvp6124b "$QEMU_DIR/hw/i2c/meson.build"; then
        sed -i "s/'hisi-mis2006.c'/'hisi-mis2006.c', 'hisi-nvp6124b.c'/" \
            "$QEMU_DIR/hw/i2c/meson.build"
        echo "  hw/i2c/meson.build: added hisi-nvp6124b.c"
    fi
fi

# hw/ssi/Kconfig
if ! grep -q HISI_SPI "$QEMU_DIR/hw/ssi/Kconfig"; then
    cat >> "$QEMU_DIR/hw/ssi/Kconfig" <<'KCONFIG'

config HISI_SPI
    bool
    select SSI
KCONFIG
    echo "  patched hw/ssi/Kconfig"
else
    echo "  hw/ssi/Kconfig already patched"
fi

# hw/ssi/meson.build
if ! grep -q hisi-spi "$QEMU_DIR/hw/ssi/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_SPI', if_true: files('hisi-spi.c', 'hisi-imx122.c'))" \
        >> "$QEMU_DIR/hw/ssi/meson.build"
    echo "  patched hw/ssi/meson.build"
else
    echo "  hw/ssi/meson.build already patched"
fi

# ── 4. Patch QEMU SD card model to accept 1.8V ─────────────────────────
# The EV300 kernel DT forces UHS-I / HS200 1.8V signaling.  QEMU's SD
# card model rejects 1.8V by default.  Patch it to accept 1.8V and
# advertise the capability in OCR so card enumeration succeeds.
if ! grep -q '1701.*2000' "$QEMU_DIR/hw/sd/sd.c"; then
    sed -i '/case 2001 \.\.\. 3000:/i\    case 1701 ... 2000: /* SD_VOLTAGE_1_8V */' \
        "$QEMU_DIR/hw/sd/sd.c"
    sed -i 's/sd->ocr = R_OCR_VDD_VOLTAGE_WIN_HI_MASK;/sd->ocr = R_OCR_VDD_VOLTAGE_WIN_HI_MASK | R_OCR_ACCEPT_SWITCH_1V8_MASK;/' \
        "$QEMU_DIR/hw/sd/sd.c"
    echo "  patched hw/sd/sd.c (1.8V voltage support)"
else
    echo "  hw/sd/sd.c already patched"
fi

# ── 5. Build ────────────────────────────────────────────────────────────
# arm-softmmu covers V1–V5 IPC and DVR/NVR (32-bit ARMv7-a / ARM926).
# aarch64-softmmu adds Hi3798CV200 STB (Cortex-A53, ARMv8-a, 64-bit).
echo "Building QEMU (arm + aarch64 targets)..."
cd "$QEMU_DIR"
mkdir -p build && cd build
# Reconfigure if the existing build was scoped to arm-softmmu only.
if [ -f build.ninja ] && ! grep -q "aarch64-softmmu" config-host.mak 2>/dev/null; then
    rm -f config-host.mak
fi
if [ ! -f config-host.mak ]; then
    ../configure --target-list=arm-softmmu,aarch64-softmmu --disable-docs
fi
ninja -j$(nproc)

echo ""
echo "Done! Verify:"
echo "  ./qemu-src/build/qemu-system-arm     -machine help | grep hi3516"
echo "  ./qemu-src/build/qemu-system-aarch64 -machine help | grep hi3798"
