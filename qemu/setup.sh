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
cp qemu/hw/misc/hantro-h264.c qemu/hw/misc/hantro-h264.h qemu/hw/misc/hantro-h264-tables.h \
                                    "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/xm-vi.c             "$QEMU_DIR/hw/misc/"
cp qemu/hw/i2c/xm-i2c.c             "$QEMU_DIR/hw/i2c/"
cp qemu/hw/gpio/xm-gpio.c           "$QEMU_DIR/hw/gpio/"
cp qemu/hw/gpio/xm510-gpio.c        "$QEMU_DIR/hw/gpio/"
cp qemu/hw/watchdog/arm-sp805.c     "$QEMU_DIR/hw/watchdog/"
cp qemu/hw/net/xm-dwmac.c           "$QEMU_DIR/hw/net/"
cp qemu/hw/arm/goke.c               "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/goke.h       "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/gk-vic.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-timer.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-wdt.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-sflash.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-ephy.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-remap.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/gk-dma.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/i2c/gk-idc.c             "$QEMU_DIR/hw/i2c/"
cp qemu/hw/arm/allwinner-v83x.c     "$QEMU_DIR/hw/arm/"
cp qemu/hw/arm/allwinner-cam.c      "$QEMU_DIR/hw/arm/"
for f in $(cd qemu && find hw include -name 'aw-*' 2>/dev/null); do
    cp "qemu/$f" "$QEMU_DIR/$f"
done
cp qemu/hw/arm/anyka.c              "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/anyka.h      "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/ak-sysctrl.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/ak-l2.c             "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/ak-ddrc.c           "$QEMU_DIR/hw/misc/"
cp qemu/include/hw/misc/ak-l2.h     "$QEMU_DIR/include/hw/misc/"
cp qemu/hw/char/ak-uart.c           "$QEMU_DIR/hw/char/"
cp qemu/hw/ssi/ak-spi.c             "$QEMU_DIR/hw/ssi/"
cp qemu/hw/ssi/ak-spinor.c          "$QEMU_DIR/hw/ssi/"
cp qemu/hw/gpio/ak-gpio.c           "$QEMU_DIR/hw/gpio/"
cp qemu/hw/usb/ak-usb.c             "$QEMU_DIR/hw/usb/"
cp qemu/hw/i2c/ak-i2c.c             "$QEMU_DIR/hw/i2c/"
cp qemu/hw/sd/ak-mci.c              "$QEMU_DIR/hw/sd/"
cp qemu/hw/arm/sigmastar.c          "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/sigmastar.h  "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/sstar-riu.c         "$QEMU_DIR/hw/misc/"
cp qemu/include/hw/misc/sstar-riu.h "$QEMU_DIR/include/hw/misc/"
cp qemu/hw/intc/sstar-intc.c        "$QEMU_DIR/hw/intc/"
cp qemu/hw/timer/sstar-timer.c      "$QEMU_DIR/hw/timer/"
cp qemu/hw/char/sstar-uart.c        "$QEMU_DIR/hw/char/"
cp qemu/hw/ssi/sstar-fsp.c          "$QEMU_DIR/hw/ssi/"
cp qemu/include/hw/ssi/sstar-fsp.h  "$QEMU_DIR/include/hw/ssi/"
cp qemu/hw/dma/sstar-bdma.c         "$QEMU_DIR/hw/dma/"
cp qemu/hw/net/sstar-emac.c         "$QEMU_DIR/hw/net/"
cp qemu/hw/display/mstar-ge.c       "$QEMU_DIR/hw/display/"
cp qemu/hw/display/mstar-disp.c     "$QEMU_DIR/hw/display/"
cp qemu/hw/usb/sstar-uhc.c          "$QEMU_DIR/hw/usb/"
cp qemu/hw/misc/sstar-vpu.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-crypto.c      "$QEMU_DIR/hw/misc/"
cp qemu/hw/i2c/sstar-i2c.c          "$QEMU_DIR/hw/i2c/"
cp qemu/hw/misc/sstar-venc.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-isp.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-vif.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-jpe.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-bach.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/sstar-cmdq.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/arm/novatek.c            "$QEMU_DIR/hw/arm/"
cp qemu/hw/arm/nvt-lzma.c           "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/novatek.h    "$QEMU_DIR/include/hw/arm/"
cp qemu/include/hw/arm/nvt-lzma.h   "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/nvt-top.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-cg.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-wdt.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-otp.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-pwm.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-crypto.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-hash.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-rsa.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-media.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-pixel.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-gfx.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-ai.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-md.c            "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-audio.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-h26x.c          "$QEMU_DIR/hw/misc/"
cp qemu/include/hw/misc/nvt-pixel.h "$QEMU_DIR/include/hw/misc/"
cp qemu/include/hw/misc/nvt-gfx.h   "$QEMU_DIR/include/hw/misc/"
cp qemu/include/hw/misc/nvt-ai.h    "$QEMU_DIR/include/hw/misc/"
cp qemu/include/hw/misc/nvt-md.h    "$QEMU_DIR/include/hw/misc/"
cp qemu/include/hw/misc/nvt-cg.h    "$QEMU_DIR/include/hw/misc/"
cp qemu/include/hw/misc/nvt-h26x.h  "$QEMU_DIR/include/hw/misc/"
cp qemu/hw/misc/nvt-dai.c           "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-engine.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-hdmi.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/nvt-h26xdec.c       "$QEMU_DIR/hw/misc/"
cp qemu/hw/display/nvt-lcd300.c     "$QEMU_DIR/hw/display/"
cp qemu/hw/display/nvt-ide.c        "$QEMU_DIR/hw/display/"
cp qemu/hw/display/nvt-dsi.c        "$QEMU_DIR/hw/display/"
cp qemu/hw/timer/nvt-timer.c        "$QEMU_DIR/hw/timer/"
cp qemu/hw/gpio/nvt-gpio.c          "$QEMU_DIR/hw/gpio/"
cp qemu/hw/rtc/nvt-rtc.c            "$QEMU_DIR/hw/rtc/"
cp qemu/hw/ssi/nvt-flash.c          "$QEMU_DIR/hw/ssi/"
cp qemu/hw/sd/nvt-sdio.c            "$QEMU_DIR/hw/sd/"
cp qemu/hw/net/nvt-eqos.c           "$QEMU_DIR/hw/net/"
cp qemu/hw/usb/nvt-usb.c            "$QEMU_DIR/hw/usb/"
cp qemu/hw/i2c/nvt-i2c.c            "$QEMU_DIR/hw/i2c/"
cp qemu/include/hw/ssi/nvt-flash.h  "$QEMU_DIR/include/hw/ssi/"
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
    select XM_DWMAC
KCONFIG
    echo "  hw/arm/Kconfig: added XIONGMAI"
fi
# The DWMAC model serves Goke too: its own symbol, selected by both.
if ! grep -q "^config XM_DWMAC$" "$QEMU_DIR/hw/net/Kconfig"; then
    printf '\nconfig XM_DWMAC\n    bool\n' >> "$QEMU_DIR/hw/net/Kconfig"
    echo "  hw/net/Kconfig: added XM_DWMAC"
fi
if ! sed -n '/^config XIONGMAI$/,/^$/p' "$QEMU_DIR/hw/arm/Kconfig" | grep -q XM_DWMAC; then
    sed -i '/^config XIONGMAI$/,/^$/ s/^    select HISI_I2C$/    select HISI_I2C\n    select XM_DWMAC/' \
        "$QEMU_DIR/hw/arm/Kconfig"
    echo "  hw/arm/Kconfig: XIONGMAI selects XM_DWMAC"
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
if ! grep -q hantro-h264 "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('hantro-h264.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added hantro-h264.c"
fi
if ! grep -q xm-vi "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-vi.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: added xm-vi.c"
fi
if ! grep -q xm-gpio "$QEMU_DIR/hw/gpio/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-gpio.c'))" \
        >> "$QEMU_DIR/hw/gpio/meson.build"
    echo "  hw/gpio/meson.build: added xm-gpio.c"
fi
if ! grep -q xm510-gpio "$QEMU_DIR/hw/gpio/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm510-gpio.c'))" \
        >> "$QEMU_DIR/hw/gpio/meson.build"
    echo "  hw/gpio/meson.build: added xm510-gpio.c"
fi
if ! grep -q arm-sp805 "$QEMU_DIR/hw/watchdog/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('arm-sp805.c'))" \
        >> "$QEMU_DIR/hw/watchdog/meson.build"
    echo "  hw/watchdog/meson.build: added arm-sp805.c"
fi
if ! grep -q xm-i2c "$QEMU_DIR/hw/i2c/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xm-i2c.c'))" \
        >> "$QEMU_DIR/hw/i2c/meson.build"
    echo "  hw/i2c/meson.build: added xm-i2c.c"
fi
if ! grep -q xm-dwmac "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_XM_DWMAC', if_true: files('xm-dwmac.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  hw/net/meson.build: added xm-dwmac.c"
elif grep -q "CONFIG_XIONGMAI', if_true: files('xm-dwmac.c')" "$QEMU_DIR/hw/net/meson.build"; then
    sed -i "s/CONFIG_XIONGMAI', if_true: files('xm-dwmac.c')/CONFIG_XM_DWMAC', if_true: files('xm-dwmac.c')/" \
        "$QEMU_DIR/hw/net/meson.build"
    echo "  hw/net/meson.build: xm-dwmac.c now under XM_DWMAC"
fi
if ! grep -q xiongmai "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_XIONGMAI', if_true: files('xiongmai.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added xiongmai.c"
fi

# Goke GK7101/GK7102: Goke's own Ambarella-IP design (hw/arm/goke.c), seen
# by its software through a register-scrambling HAL. Same guarded appends
# as Xiongmai, for a fresh tree and an existing one alike.
if ! grep -q "^config GOKE$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config GOKE
    bool
    default y
    depends on TCG && ARM
    select SERIAL_MM
    select OR_IRQ
    select PTIMER
    select HISI_MISC
    select SDHCI
    select UNIMP
    select XM_DWMAC
    select I2C
    select HISI_I2C
KCONFIG
    echo "  hw/arm/Kconfig: added GOKE"
fi
for f in gk-vic gk-timer gk-wdt gk-sflash gk-ephy gk-remap gk-dma; do
    if ! grep -q "'$f.c'" "$QEMU_DIR/hw/misc/meson.build"; then
        echo "system_ss.add(when: 'CONFIG_GOKE', if_true: files('$f.c'))" \
            >> "$QEMU_DIR/hw/misc/meson.build"
        echo "  hw/misc/meson.build: added $f.c"
    fi
done
if ! grep -q "'gk-idc.c'" "$QEMU_DIR/hw/i2c/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_GOKE', if_true: files('gk-idc.c'))" \
        >> "$QEMU_DIR/hw/i2c/meson.build"
    echo "  hw/i2c/meson.build: added gk-idc.c"
fi
if ! grep -q "'goke.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_GOKE', if_true: files('goke.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added goke.c"
fi

# Anyka AK39xx: ARM926 camera SoCs (hw/arm/anyka.c).  Same guarded
# appends as Goke, for a fresh tree and an existing one alike.
if ! grep -q "^config ANYKA$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config ANYKA
    bool
    default y
    depends on TCG && ARM
    select PTIMER
    select HISI_MISC
    select UNIMP
    select SSI
    select I2C
    select XM_DWMAC
    select SD
KCONFIG
    echo "  hw/arm/Kconfig: added ANYKA"
else
    for sym in I2C XM_DWMAC SD; do
        if ! sed -n '/^config ANYKA$/,/^$/p' "$QEMU_DIR/hw/arm/Kconfig" | \
                grep -q "select $sym\$"; then
            sed -i "/^config ANYKA\$/,/^\$/{/select SSI\$/a\\    select $sym
}" "$QEMU_DIR/hw/arm/Kconfig"
            echo "  hw/arm/Kconfig: ANYKA now selects $sym"
        fi
    done
fi
for f in misc/ak-sysctrl misc/ak-l2 misc/ak-ddrc char/ak-uart ssi/ak-spi ssi/ak-spinor gpio/ak-gpio usb/ak-usb i2c/ak-i2c sd/ak-mci; do
    if ! grep -q "'${f#*/}.c'" "$QEMU_DIR/hw/${f%/*}/meson.build"; then
        echo "system_ss.add(when: 'CONFIG_ANYKA', if_true: files('${f#*/}.c'))" \
            >> "$QEMU_DIR/hw/${f%/*}/meson.build"
        echo "  hw/${f%/*}/meson.build: added ${f#*/}.c"
    fi
done
if ! grep -q "'anyka.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_ANYKA', if_true: files('anyka.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added anyka.c"
fi

# SigmaStar (MStar) ARMv7: Cortex-A7 SoCs behind the RIU (hw/arm/sigmastar.c).
# Same guarded appends as Anyka, for a fresh tree and an existing one alike.
if ! grep -q "^config SIGMASTAR$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config SIGMASTAR
    bool
    default y
    depends on TCG && ARM
    select A15MPCORE
    select ARM_GIC
    select SERIAL_MM
    select UNIMP
    select USB_EHCI_SYSBUS
    select AHCI_SYSBUS
    select I2C
    select HISI_I2C
KCONFIG
    echo "  hw/arm/Kconfig: added SIGMASTAR"
else
    for sym in A15MPCORE ARM_GIC SERIAL_MM UNIMP USB_EHCI_SYSBUS AHCI_SYSBUS I2C HISI_I2C; do
        if ! sed -n '/^config SIGMASTAR$/,/^$/p' "$QEMU_DIR/hw/arm/Kconfig" | \
                grep -q "select $sym\$"; then
            sed -i "/^config SIGMASTAR\$/,/^\$/{/depends on TCG \&\& ARM\$/a\\    select $sym
}" "$QEMU_DIR/hw/arm/Kconfig"
            echo "  hw/arm/Kconfig: SIGMASTAR now selects $sym"
        fi
    done
fi
for f in misc/sstar-riu misc/sstar-crypto misc/sstar-isp misc/sstar-vif misc/sstar-jpe misc/sstar-bach misc/sstar-cmdq i2c/sstar-i2c intc/sstar-intc timer/sstar-timer char/sstar-uart ssi/sstar-fsp dma/sstar-bdma net/sstar-emac display/mstar-ge display/mstar-disp usb/sstar-uhc; do
    if ! grep -q "'${f#*/}.c'" "$QEMU_DIR/hw/${f%/*}/meson.build"; then
        echo "system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('${f#*/}.c'))" \
            >> "$QEMU_DIR/hw/${f%/*}/meson.build"
        echo "  hw/${f%/*}/meson.build: added ${f#*/}.c"
    fi
done
# The VPU model decodes with libavcodec when it is there (pkg-config), and
# answers its registers without pictures when it is not (FFmpeg 4.0 or
# later, libavcodec 58: the send/receive API).  (The define
# reaches every system_ss file; only sstar-vpu.c looks at it.)
if ! grep -q "'sstar-vpu.c'" "$QEMU_DIR/hw/misc/meson.build"; then
    cat >> "$QEMU_DIR/hw/misc/meson.build" <<'MESON'
sstar_avcodec = dependency('libavcodec', version: '>=58', required: false)
sstar_avutil = dependency('libavutil', version: '>=56', required: false)
if sstar_avcodec.found() and sstar_avutil.found()
  system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: [files('sstar-vpu.c'),
    sstar_avcodec, sstar_avutil,
    declare_dependency(compile_args: '-DSSTAR_VPU_LIBAVCODEC')])
else
  system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('sstar-vpu.c'))
endif
MESON
    echo "  hw/misc/meson.build: added sstar-vpu.c"
fi
# ... and a tree configured before the FFmpeg version floor gets it
if grep -q "dependency('libavcodec', required: false)" \
        "$QEMU_DIR/hw/misc/meson.build"; then
    sed -i "s/dependency('libavcodec', required: false)/dependency('libavcodec', version: '>=58', required: false)/; s/dependency('libavutil', required: false)/dependency('libavutil', version: '>=56', required: false)/" \
        "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: FFmpeg 4.0 floor for sstar-vpu"
fi
# The encoder model encodes with libavcodec's libx264 and libx265 when
# they are there, and writes placeholder frames when they are not.  It
# comes after the VPU block, whose dependencies it shares.  A tree that
# built it unconditionally loses that line first.
if grep -q "^system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('sstar-venc.c'))$" \
        "$QEMU_DIR/hw/misc/meson.build"; then
    sed -i "/^system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('sstar-venc.c'))\$/d" \
        "$QEMU_DIR/hw/misc/meson.build"
fi
if ! grep -q "'sstar-venc.c'" "$QEMU_DIR/hw/misc/meson.build"; then
    cat >> "$QEMU_DIR/hw/misc/meson.build" <<'MESON'
if sstar_avcodec.found() and sstar_avutil.found()
  system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: [files('sstar-venc.c'),
    sstar_avcodec, sstar_avutil,
    declare_dependency(compile_args: '-DSSTAR_VENC_LIBAVCODEC')])
else
  system_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('sstar-venc.c'))
endif
MESON
    echo "  hw/misc/meson.build: added sstar-venc.c"
fi
if ! grep -q "'sigmastar.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_SIGMASTAR', if_true: files('sigmastar.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added sigmastar.c"
fi

# Novatek NT9856x (NA51089): Cortex-A9 camera SoCs (hw/arm/novatek.c).
# Same guarded appends as SigmaStar, for a fresh tree and an existing one
# alike.
if ! grep -q "^config NOVATEK$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config NOVATEK
    bool
    default y
    depends on TCG && ARM
    select A9MPCORE
    select ARM_GIC
    select PL310
    select PTIMER
    select SERIAL_MM
    select UNIMP
    select HISI_MISC
    select SD
    select USB_EHCI_SYSBUS
    select HISI_I2C
    select AHCI_SYSBUS
KCONFIG
    echo "  hw/arm/Kconfig: added NOVATEK"
else
    for sym in A9MPCORE ARM_GIC PL310 PTIMER SERIAL_MM UNIMP HISI_MISC SD USB_EHCI_SYSBUS HISI_I2C AHCI_SYSBUS; do
        if ! sed -n '/^config NOVATEK$/,/^$/p' "$QEMU_DIR/hw/arm/Kconfig" | \
                grep -q "select $sym\$"; then
            sed -i "/^config NOVATEK\$/,/^\$/{/depends on TCG \&\& ARM\$/a\\    select $sym
}" "$QEMU_DIR/hw/arm/Kconfig"
            echo "  hw/arm/Kconfig: NOVATEK now selects $sym"
        fi
    done
fi
for f in misc/nvt-top misc/nvt-cg misc/nvt-wdt misc/nvt-otp misc/nvt-pwm misc/nvt-crypto misc/nvt-hash misc/nvt-rsa misc/nvt-media misc/nvt-pixel misc/nvt-gfx misc/nvt-ai misc/nvt-md misc/nvt-audio misc/nvt-dai misc/nvt-engine misc/nvt-hdmi display/nvt-lcd300 display/nvt-ide display/nvt-dsi timer/nvt-timer gpio/nvt-gpio rtc/nvt-rtc ssi/nvt-flash sd/nvt-sdio net/nvt-eqos usb/nvt-usb i2c/nvt-i2c; do
    if ! grep -q "'${f#*/}.c'" "$QEMU_DIR/hw/${f%/*}/meson.build"; then
        echo "system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('${f#*/}.c'))" \
            >> "$QEMU_DIR/hw/${f%/*}/meson.build"
        echo "  hw/${f%/*}/meson.build: added ${f#*/}.c"
    fi
done
# The H.26x engine codes its pictures with x264 / x265 when pkg-config
# finds them, and writes filler slices when it does not (nvt-h26x.c).
# (The defines reach every system_ss file; only nvt-h26x.c looks.)
if ! grep -q "'nvt-h26x.c'" "$QEMU_DIR/hw/misc/meson.build"; then
    cat >> "$QEMU_DIR/hw/misc/meson.build" <<'MESON'
nvt_x264 = dependency('x264', required: false)
nvt_x265 = dependency('x265', required: false)
nvt_h26x = [files('nvt-h26x.c')]
if nvt_x264.found()
  nvt_h26x += [nvt_x264, declare_dependency(compile_args: '-DNVT_H26X_X264')]
endif
if nvt_x265.found()
  nvt_h26x += [nvt_x265, declare_dependency(compile_args: '-DNVT_H26X_X265')]
endif
system_ss.add(when: 'CONFIG_NOVATEK', if_true: nvt_h26x)
MESON
    echo "  hw/misc/meson.build: added nvt-h26x.c"
fi
if ! grep -q "'novatek.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_NOVATEK', if_true: files('novatek.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added novatek.c"
fi
# The NA51103 decoder writes pictures when libavcodec is there (pkg-config),
# as sstar-vpu does, and completes its jobs without them when it is not.
# A tree that listed it plainly is upgraded.
if grep -q "^system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-h26xdec.c'))$" \
        "$QEMU_DIR/hw/misc/meson.build"; then
    sed -i "/^system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-h26xdec.c'))$/d" \
        "$QEMU_DIR/hw/misc/meson.build"
fi
# An earlier version of the step above also deleted the fallback below.
if grep -q "^nvt_avcodec = " "$QEMU_DIR/hw/misc/meson.build" &&
   ! grep -q "^  system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-h26xdec.c'))$" \
        "$QEMU_DIR/hw/misc/meson.build"; then
    sed -i "/^if nvt_avcodec.found()/,/^endif$/ s/^else$/else\n  system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-h26xdec.c'))/" \
        "$QEMU_DIR/hw/misc/meson.build"
    echo "  hw/misc/meson.build: restored nvt-h26xdec.c without libavcodec"
fi
if ! grep -q "'nvt-h26xdec.c'" "$QEMU_DIR/hw/misc/meson.build"; then
    cat >> "$QEMU_DIR/hw/misc/meson.build" <<'MESON'
nvt_avcodec = dependency('libavcodec', version: '>=58', required: false)
nvt_avutil = dependency('libavutil', version: '>=56', required: false)
if nvt_avcodec.found() and nvt_avutil.found()
  system_ss.add(when: 'CONFIG_NOVATEK', if_true: [files('nvt-h26xdec.c'),
    nvt_avcodec, nvt_avutil,
    declare_dependency(compile_args: '-DNVT_H26XDEC_LIBAVCODEC')])
else
  system_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-h26xdec.c'))
endif
MESON
    echo "  hw/misc/meson.build: added nvt-h26xdec.c"
fi
if ! grep -q "'nvt-lzma.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_NOVATEK', if_true: files('nvt-lzma.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added nvt-lzma.c"
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

# Allwinner V831/V833 (sun8iw19): Cortex-A7 camera SoCs
# (hw/arm/allwinner-v83x.c).  Same guarded appends as Anyka.  Its own
# devices are the hw/*/aw-*.c files, each built under ALLWINNER_V83X.
if ! grep -q "^config ALLWINNER_V83X$" "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config ALLWINNER_V83X
    bool
    default y
    depends on TCG && ARM
    select ARM_GIC
    select SERIAL_MM
    select UNIMP
    select HISI_MISC
    select SSI
    select ALLWINNER_A10_PIT
    select I2C
    select ALLWINNER_I2C
    select OR_IRQ
    select ALLWINNER_SUN8I_EMAC
    select HISI_I2C
KCONFIG
    echo "  hw/arm/Kconfig: added ALLWINNER_V83X"
else
    for sym in ALLWINNER_A10_PIT I2C ALLWINNER_I2C OR_IRQ ALLWINNER_SUN8I_EMAC HISI_I2C; do
        if ! sed -n '/^config ALLWINNER_V83X$/,/^$/p' "$QEMU_DIR/hw/arm/Kconfig" | \
                grep -q "select $sym\$"; then
            sed -i "/^config ALLWINNER_V83X\$/,/^\$/{/select SSI\$/a\\    select $sym
}" "$QEMU_DIR/hw/arm/Kconfig"
            echo "  hw/arm/Kconfig: ALLWINNER_V83X now selects $sym"
        fi
    done
fi
for f in $(cd qemu && find hw -name 'aw-*.c' 2>/dev/null); do
    d=$(dirname "$f"); b=$(basename "$f")
    if ! grep -q "'$b'" "$QEMU_DIR/$d/meson.build"; then
        echo "system_ss.add(when: 'CONFIG_ALLWINNER_V83X', if_true: files('$b'))" \
            >> "$QEMU_DIR/$d/meson.build"
        echo "  $d/meson.build: added $b"
    fi
done
if ! grep -q "'allwinner-v83x.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_ALLWINNER_V83X', if_true: files('allwinner-v83x.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added allwinner-v83x.c"
fi
# Allwinner V3s and V851s (hw/arm/allwinner-cam.c): the V831's devices at
# their addresses, so under the same ALLWINNER_V83X
if ! grep -q "'allwinner-cam.c'" "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_ALLWINNER_V83X', if_true: files('allwinner-cam.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  hw/arm/meson.build: added allwinner-cam.c"
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
