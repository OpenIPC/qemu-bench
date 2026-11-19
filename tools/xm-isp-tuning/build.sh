#!/bin/sh
# Build the camera tools, the host tools and layout.json.
#
#   SDK_INC=<shimetapi_pico_gx>/source/gmp/include \
#   TOOLCHAIN=<toolchain.goke-gk7206v100> tools/xm-isp-tuning/build.sh
#
# SDK_INC: the XMedia SDK's userland headers (ShiMetaPi's shimetapi_pico_gx,
# source/gmp/include). TOOLCHAIN: OpenIPC's toolchain.goke-gk7206v100, whose
# sysroot carries the SDK's libxmedia_*.so. Output in tools/xm-isp-tuning/build.
#
# SDK_STATIC (optional): the SDK's static libraries
# (source/gmp/lib/arm-gcc12.2.0-linux-uclibceabi/nolog/static). With it,
# build/twin/ also gets the static tools twin/xm-twin-export.py types into
# the stock firmware.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
out=$here/build
: "${SDK_INC:?set SDK_INC to the SDK include directory}"
: "${TOOLCHAIN:?set TOOLCHAIN to toolchain.goke-gk7206v100}"
cc=$TOOLCHAIN/bin/arm-openipc-linux-musleabihf-gcc
nm=$TOOLCHAIN/bin/arm-openipc-linux-musleabihf-nm
sysroot=$TOOLCHAIN/arm-openipc-linux-musleabihf/sysroot
mkdir -p "$out"

# The SDK's libraries name none of their own dependencies; link them all.
libs="-lxmedia_isp -lxmedia_ae -lxmedia_awb -lxmedia_vi -lxmedia_cl -lxmedia_npu
      -lxmedia_flatcc -lxmedia_common -lpthread -lm -ldl"
for t in attrtool pqexport ae-ainr; do
	rm -f "$out/$t"
	if ! "$cc" -O1 -rdynamic -I"$SDK_INC" "$here/camera/$t.c" "$here/camera/sdkshim.c" \
		-o "$out/$t" -L"$sysroot/usr/lib" $libs \
		-Wl,--allow-shlib-undefined -Wl,--no-as-needed 2>"$out/$t.log"; then
		cat "$out/$t.log" >&2
		exit 1
	fi
	grep -v 'warning: .*needed by\|warning: .*not found' "$out/$t.log" >&2 || true
done

"$cc" -c -I"$SDK_INC" "$here/host/layout.c" -o "$out/layout.o"
"$nm" -S "$out/layout.o" | python3 -c '
import json, sys
sizes, offsets = {}, {}
for line in sys.stdin:
    f = line.split()
    if len(f) != 4:
        continue
    n, name = int(f[1], 16), f[3]
    if name.startswith("sz_"):
        sizes[name[3:]] = n
    elif name.startswith("of_"):
        t, field = name[3:].split("__")
        offsets.setdefault(t, {})[field] = n - 1
json.dump({"sizes": sizes, "offsets": offsets}, open(sys.argv[1], "w"), indent=1)
' "$out/layout.json"

for t in xm-isp-finish xm-isp-decode; do
	cc -O1 -I"$SDK_INC" "$here/host/$t.c" -o "$out/$t"
done
# The host tools read the camera's structures with the host's layout.
cat > "$out/sizecheck.c" <<'C'
#include <stdio.h>
#include <xmedia_ae.h>
#include <xmedia_awb.h>
#include <xmedia_isp.h>
#define P(t) printf("%s %zu\n", #t, sizeof(t));
int main(void) {
    P(xmedia_isp_sharpen_attr) P(xmedia_isp_lce_attr) P(xmedia_isp_ca_attr)
    P(xmedia_isp_bnr_attr) P(xmedia_awb_attr) P(xmedia_isp_stnr_attr)
    P(xmedia_isp_dpc_dynamic_attr) P(xmedia_isp_demosaic_attr)
    P(xmedia_isp_drc_attr) P(xmedia_isp_dehaze_attr) P(xmedia_isp_ainr_attr)
    P(xmedia_ae_exposure_attr) P(xmedia_awb_saturation_attr)
    P(xmedia_awb_ccm_attr)
    return 0;
}
C
cc -I"$SDK_INC" "$out/sizecheck.c" -o "$out/sizecheck"
"$out/sizecheck" | python3 -c '
import json, sys
s = json.load(open(sys.argv[1]))["sizes"]
bad = [l for l in sys.stdin if s[l.split()[0]] != int(l.split()[1])]
if bad:
    sys.exit("host and camera layouts differ: " + " ".join(bad))
' "$out/layout.json"
if [ -n "${SDK_STATIC:-}" ]; then
	mkdir -p "$out/twin"
	a=$SDK_STATIC
	for t in attrtool pqexport hexout; do
		"$cc" -static -O1 -I"$SDK_INC" "$here/camera/sdkshim_static.c" \
			"$here/camera/$t.c" -o "$out/twin/$t.static" \
			-Wl,--allow-multiple-definition -Wl,--start-group \
			"$a/libxmedia_isp.a" "$a/libxmedia_ae.a" "$a/libxmedia_awb.a" \
			"$a/libxmedia_cl.a" "$a/libxmedia_npu.a" "$a/libxmedia_flatcc.a" \
			"$a/libxmedia_vi.a" "$a/libxmedia_common.a" -Wl,--end-group \
			-lpthread -lm 2>"$out/twin/$t.log" || { cat "$out/twin/$t.log" >&2; exit 1; }
		"$TOOLCHAIN"/bin/arm-openipc-linux-musleabihf-strip "$out/twin/$t.static"
		gzip -9 -n -c "$out/twin/$t.static" > "$out/twin/$t.static.gz"
	done
fi
echo "built in $out"
