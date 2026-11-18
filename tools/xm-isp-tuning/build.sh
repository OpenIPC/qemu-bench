#!/bin/sh
# Build the camera tools, the host tools and layout.json.
#
#   SDK_INC=<shimetapi_pico_gx>/source/gmp/include \
#   TOOLCHAIN=<toolchain.goke-gk7206v100> tools/xm-isp-tuning/build.sh
#
# SDK_INC: the XMedia SDK's userland headers (ShiMetaPi's shimetapi_pico_gx,
# source/gmp/include). TOOLCHAIN: OpenIPC's toolchain.goke-gk7206v100, whose
# sysroot carries the SDK's libxmedia_*.so. Output in tools/xm-isp-tuning/build.
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
	"$cc" -O1 -rdynamic -I"$SDK_INC" "$here/camera/$t.c" "$here/camera/sdkshim.c" \
		-o "$out/$t" -L"$sysroot/usr/lib" $libs \
		-Wl,--allow-shlib-undefined -Wl,--no-as-needed 2>&1 |
		grep -v 'warning: .*needed by\|warning: .*not found' || true
	[ -x "$out/$t" ]
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
echo "built in $out"
