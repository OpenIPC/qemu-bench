#!/bin/bash
#
# MXU2 instruction/context/vendor test for qemu-system-mipsel -M t31.
#
# Builds static MIPS test programs with the Ingenic GCC toolchain, boots
# them as PID 1 (rdinit=/init) under the OpenIPC T31 kernel on the t31
# machine, and checks results on the serial console.  Covers every
# implemented MXU2 instruction (against a portable C reference computed in
# the same binary -- it encodes the same semantic assumptions as the QEMU
# helper), the kernel's lazy CU2 enable, an MXU2 context switch across two
# processes, and a real vendor MXU2 routine (jz_memcpy from the OpenIPC
# libimp.so) executed from its own machine bytes.
#
# Skips cleanly (exit 0 with a message) if the toolchain, kernel or QEMU
# binary is absent.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

TOOLCHAIN="${INGENIC_TOOLCHAIN:-/home/ai/toolchains/ingenic/mips-gcc540-uclibc0.9.33.2-64bit-r3.3.0.smaller}"
CC="$TOOLCHAIN/bin/mips-linux-uclibc-gcc"
QEMU="${QEMU_MIPSEL:-$ROOT/qemu-src/build/qemu-system-mipsel}"
KERNEL="${T31_UIMAGE:-/home/ai/dimerr/local/ingenic/fw/t31/uImage.t31}"
ROOTFS="${T31_ROOTFS:-/home/ai/dimerr/local/ingenic/fw/t31/rootfs.squashfs.t31}"

skip() { echo "SKIP: $*"; exit 0; }
[ -x "$CC" ]     || skip "Ingenic toolchain not found ($CC)"
[ -x "$QEMU" ]   || skip "qemu-system-mipsel not built ($QEMU)"
[ -f "$KERNEL" ] || skip "OpenIPC T31 kernel not found ($KERNEL)"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
CFLAGS="-static -EL -O2 -mmxu2 -march=mips32r2 -flax-vector-conversions"

run_init() {  # $1 = binary ; echoes console output
    local bin="$1" dir="$WORK/ir"
    rm -rf "$dir"; mkdir -p "$dir/dev"
    cp "$bin" "$dir/init"
    ( cd "$dir" && find . | cpio -o -H newc 2>/dev/null | gzip > "$WORK/ir.cpio.gz" )
    timeout 120 "$QEMU" -M t31 -m 64M -nographic -serial mon:stdio -monitor none \
        -kernel "$KERNEL" -initrd "$WORK/ir.cpio.gz" \
        -append "rdinit=/init console=ttyS1,115200n8 panic=1" 2>&1
}

fail=0

# ---- 1. instruction + lazy-CU2 + context-switch test --------------------
cat > "$WORK/test_mxu2.c" <<'EOF'
/*
 * MXU2 instruction test, run as PID 1 under qemu-system-mipsel -M t31.
 *
 * For each instruction it runs the GCC MXU2 intrinsic and compares the
 * 128-bit result against a portable scalar C reference computed in this
 * same file.  The reference encodes the SAME semantic assumptions as the
 * QEMU helper (target/mips/tcg/mxu2_helper.c) -- element size/signedness
 * from <mxu2.h>, rounding/saturation from the MSA analogue; anything the
 * helper flags as a GUESS is mirrored here.  A disagreement therefore
 * means the emulator diverged from those assumptions, not that the test
 * is "more correct" than the silicon.
 *
 * Exit status 0 and the line "MXU2 TESTS: ALL PASS (N)" on success.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sched.h>
#include <math.h>
#include <mxu2.h>

typedef union {
    v16i8 i8; v16u8 u8; v8i16 i16; v8u16 u16;
    v4i32 i32; v4u32 u32; v2i64 i64; v2u64 u64;
    v4f32 f32; v2f64 f64;
    int8_t b[16]; uint8_t ub[16];
    int16_t h[8]; uint16_t uh[8];
    int32_t w[4]; uint32_t uw[4];
    int64_t d[2]; uint64_t ud[2];
    float f[4]; double df[2];
} U;

static int fails, total;

static void check(const char *name, const void *got, const void *ref)
{
    total++;
    if (memcmp(got, ref, 16) != 0) {
        const uint8_t *g = got, *r = ref;
        fails++;
        printf("FAIL %s\n  got:", name);
        for (int i = 0; i < 16; i++) printf(" %02x", g[i]);
        printf("\n  ref:");
        for (int i = 0; i < 16; i++) printf(" %02x", r[i]);
        printf("\n");
    }
}

/* fill with a deterministic but varied pattern */
static void fill(U *x, uint32_t seed)
{
    for (int i = 0; i < 16; i++) {
        x->ub[i] = (uint8_t)(seed * 31 + i * 17 + 3);
    }
}

#define SAT_S(bits, v) ({ int64_t _lo = -((int64_t)1<<((bits)-1)); \
    int64_t _hi = ((int64_t)1<<((bits)-1))-1; \
    int64_t _t=(v); _t<_lo?_lo:_t>_hi?_hi:_t; })
#define SAT_U(bits, v) ({ uint64_t _hi = (bits)==64?~0ULL:(((uint64_t)1<<(bits))-1); \
    uint64_t _t=(v); _t>_hi?_hi:_t; })

int main(void)
{
    U a, b, c, g, r;
    fill(&a, 1); fill(&b, 2); fill(&c, 3);

    /* ---- element-wise integer, byte ---- */
    g.u8 = _mx128_add_b(a.u8, b.u8);
    for (int i=0;i<16;i++) r.ub[i]=a.ub[i]+b.ub[i];
    check("add_b", &g, &r);

    g.i16 = _mx128_add_h(a.i16, b.i16);
    for (int i=0;i<8;i++) r.h[i]=a.h[i]+b.h[i];
    check("add_h", &g, &r);

    g.i32 = _mx128_add_w(a.i32, b.i32);
    for (int i=0;i<4;i++) r.w[i]=a.w[i]+b.w[i];
    check("add_w", &g, &r);

    g.i64 = _mx128_add_d(a.i64, b.i64);
    for (int i=0;i<2;i++) r.d[i]=a.d[i]+b.d[i];
    check("add_d", &g, &r);

    g.u8 = _mx128_sub_b(a.u8, b.u8);
    for (int i=0;i<16;i++) r.ub[i]=a.ub[i]-b.ub[i];
    check("sub_b", &g, &r);

    g.i32 = _mx128_mul_w(a.i32, b.i32);
    for (int i=0;i<4;i++) r.w[i]=a.w[i]*b.w[i];
    check("mul_w", &g, &r);

    g.i8 = _mx128_addss_b(a.i8, b.i8);
    for (int i=0;i<16;i++) r.b[i]=SAT_S(8,(int64_t)a.b[i]+b.b[i]);
    check("addss_b", &g, &r);

    g.u8 = _mx128_adduu_b(a.u8, b.u8);
    for (int i=0;i<16;i++) r.ub[i]=SAT_U(8,(uint64_t)a.ub[i]+b.ub[i]);
    check("adduu_b", &g, &r);

    g.i16 = _mx128_subss_h(a.i16, b.i16);
    for (int i=0;i<8;i++) r.h[i]=SAT_S(16,(int64_t)a.h[i]-b.h[i]);
    check("subss_h", &g, &r);

    g.u8 = _mx128_subuu_b(a.u8, b.u8);
    for (int i=0;i<16;i++){int t=a.ub[i]-b.ub[i]; r.ub[i]=t<0?0:t;}
    check("subuu_b", &g, &r);

    g.i8 = _mx128_maxs_b(a.i8, b.i8);
    for (int i=0;i<16;i++) r.b[i]=a.b[i]>b.b[i]?a.b[i]:b.b[i];
    check("maxs_b", &g, &r);

    g.u8 = _mx128_maxu_b(a.u8, b.u8);
    for (int i=0;i<16;i++) r.ub[i]=a.ub[i]>b.ub[i]?a.ub[i]:b.ub[i];
    check("maxu_b", &g, &r);

    g.i16 = _mx128_mins_h(a.i16, b.i16);
    for (int i=0;i<8;i++) r.h[i]=a.h[i]<b.h[i]?a.h[i]:b.h[i];
    check("mins_h", &g, &r);

    g.u8 = _mx128_minu_b(a.u8, b.u8);
    for (int i=0;i<16;i++) r.ub[i]=a.ub[i]<b.ub[i]?a.ub[i]:b.ub[i];
    check("minu_b", &g, &r);

    /* vector shift (amount mod bits) */
    { U sh; for(int i=0;i<16;i++) sh.ub[i]=i&7;
      g.i16 = _mx128_sll_h(a.i16, sh.i16);
      for(int i=0;i<8;i++) r.h[i]=a.h[i]<<(sh.uh[i]&15);
      check("sll_h", &g, &r);
      g.i16 = _mx128_sra_h(a.i16, sh.i16);
      for(int i=0;i<8;i++) r.h[i]=a.h[i]>>(sh.uh[i]&15);
      check("sra_h", &g, &r);
      g.u16 = _mx128_srl_h(a.u16, sh.u16);
      for(int i=0;i<8;i++) r.uh[i]=a.uh[i]>>(sh.uh[i]&15);
      check("srl_h", &g, &r);
    }

    /* logic */
    g.i8=_mx128_andv(a.i8,b.i8); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]&b.ub[i];
    check("andv",&g,&r);
    g.i8=_mx128_orv(a.i8,b.i8); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]|b.ub[i];
    check("orv",&g,&r);
    g.i8=_mx128_xorv(a.i8,b.i8); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]^b.ub[i];
    check("xorv",&g,&r);
    g.i8=_mx128_norv(a.i8,b.i8); for(int i=0;i<16;i++) r.ub[i]=~(a.ub[i]|b.ub[i]);
    check("norv",&g,&r);
    g.i8=_mx128_andib(a.i8,0x5a); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]&0x5a;
    check("andib",&g,&r);
    g.i8=_mx128_orib(a.i8,0x5a); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]|0x5a;
    check("orib",&g,&r);
    g.i8=_mx128_xorib(a.i8,0x5a); for(int i=0;i<16;i++) r.ub[i]=a.ub[i]^0x5a;
    check("xorib",&g,&r);
    g.i8=_mx128_norib(a.i8,0x5a); for(int i=0;i<16;i++) r.ub[i]=~(a.ub[i]|0x5a);
    check("norib",&g,&r);

    /* compares -> all-ones/all-zero */
    g.i8=_mx128_ceq_b(a.i8,a.i8); for(int i=0;i<16;i++) r.b[i]=-1;
    check("ceq_b_eq",&g,&r);
    g.i8=_mx128_ceq_b(a.i8,b.i8); for(int i=0;i<16;i++) r.b[i]=a.b[i]==b.b[i]?-1:0;
    check("ceq_b",&g,&r);
    g.i8=_mx128_cne_b(a.i8,b.i8); for(int i=0;i<16;i++) r.b[i]=a.b[i]!=b.b[i]?-1:0;
    check("cne_b",&g,&r);
    g.i8=_mx128_clts_b(a.i8,b.i8); for(int i=0;i<16;i++) r.b[i]=a.b[i]<b.b[i]?-1:0;
    check("clts_b",&g,&r);
    g.i8=_mx128_cltu_b(a.u8,b.u8); for(int i=0;i<16;i++) r.b[i]=a.ub[i]<b.ub[i]?-1:0;
    check("cltu_b",&g,&r);
    g.i8=_mx128_cles_b(a.i8,b.i8); for(int i=0;i<16;i++) r.b[i]=a.b[i]<=b.b[i]?-1:0;
    check("cles_b",&g,&r);
    g.i8=_mx128_cleu_b(a.u8,b.u8); for(int i=0;i<16;i++) r.b[i]=a.ub[i]<=b.ub[i]?-1:0;
    check("cleu_b",&g,&r);
    { U z; memset(&z,0,16); z.b[2]=0; z.b[3]=-5; z.b[4]=7;
      g.i8=_mx128_ceqz_b(z.i8); for(int i=0;i<16;i++) r.b[i]=z.b[i]==0?-1:0;
      check("ceqz_b",&g,&r);
      g.i8=_mx128_cnez_b(z.i8); for(int i=0;i<16;i++) r.b[i]=z.b[i]!=0?-1:0;
      check("cnez_b",&g,&r);
      g.i8=_mx128_cltz_b(z.i8); for(int i=0;i<16;i++) r.b[i]=z.b[i]<0?-1:0;
      check("cltz_b",&g,&r);
      g.i8=_mx128_clez_b(z.i8); for(int i=0;i<16;i++) r.b[i]=z.b[i]<=0?-1:0;
      check("clez_b",&g,&r);
    }

    /* immediate shifts */
    g.i16=_mx128_slli_h(a.i16,3); for(int i=0;i<8;i++) r.h[i]=a.h[i]<<3;
    check("slli_h",&g,&r);
    g.i16=_mx128_srai_h(a.i16,3); for(int i=0;i<8;i++) r.h[i]=a.h[i]>>3;
    check("srai_h",&g,&r);
    g.u16=_mx128_srli_h(a.u16,3); for(int i=0;i<8;i++) r.uh[i]=a.uh[i]>>3;
    check("srli_h",&g,&r);

    /* ---- helper integer ---- */
    g.i8=_mx128_maxa_b(a.i8,b.i8);
    for(int i=0;i<16;i++){int aa=a.b[i]<0?-a.b[i]:a.b[i],bb=b.b[i]<0?-b.b[i]:b.b[i];
        r.b[i]=aa>=bb?a.b[i]:b.b[i];}
    check("maxa_b",&g,&r);
    g.i8=_mx128_mina_b(a.i8,b.i8);
    for(int i=0;i<16;i++){int aa=a.b[i]<0?-a.b[i]:a.b[i],bb=b.b[i]<0?-b.b[i]:b.b[i];
        r.b[i]=aa<=bb?a.b[i]:b.b[i];}
    check("mina_b",&g,&r);

    { U sh; for(int i=0;i<16;i++) sh.ub[i]=(i%5)+1;
      g.i16=_mx128_srar_h(a.i16,sh.i16);
      for(int i=0;i<8;i++){unsigned s=sh.uh[i]&15;int32_t v=a.h[i];
        r.h[i]=s?(v>>s)+((v>>(s-1))&1):v;}
      check("srar_h",&g,&r);
      g.u16=_mx128_srlr_h(a.u16,sh.u16);
      for(int i=0;i<8;i++){unsigned s=sh.uh[i]&15;uint32_t v=a.uh[i];
        r.uh[i]=s?(v>>s)+((v>>(s-1))&1):v;}
      check("srlr_h",&g,&r);
    }

    g.i8=_mx128_adda_b(a.i8,b.i8);
    for(int i=0;i<16;i++){int aa=a.b[i]<0?-a.b[i]:a.b[i],bb=b.b[i]<0?-b.b[i]:b.b[i];
        r.b[i]=aa+bb;}
    check("adda_b",&g,&r);
    g.i8=_mx128_addas_b(a.i8,b.i8);
    for(int i=0;i<16;i++){int aa=a.b[i]<0?-a.b[i]:a.b[i],bb=b.b[i]<0?-b.b[i]:b.b[i];
        r.b[i]=SAT_S(8,aa+bb);}
    check("addas_b",&g,&r);
    g.i8=_mx128_subsa_b(a.i8,b.i8);
    for(int i=0;i<16;i++){int d=a.b[i]-b.b[i];r.b[i]=SAT_S(8,d<0?-d:d);}
    check("subsa_b",&g,&r);
    g.i8=_mx128_subua_b(a.u8,b.u8);
    for(int i=0;i<16;i++) r.ub[i]=a.ub[i]>b.ub[i]?a.ub[i]-b.ub[i]:b.ub[i]-a.ub[i];
    check("subua_b",&g,&r);
    g.i8=_mx128_subus_b(a.u8,b.u8);
    for(int i=0;i<16;i++){int t=a.ub[i]-b.ub[i];r.ub[i]=t<0?0:t;}
    check("subus_b",&g,&r);

    g.i8=_mx128_aves_b(a.i8,b.i8);
    for(int i=0;i<16;i++) r.b[i]=(a.b[i]>>1)+(b.b[i]>>1)+(a.b[i]&b.b[i]&1);
    check("aves_b",&g,&r);
    g.u8=_mx128_aveu_b(a.u8,b.u8);
    for(int i=0;i<16;i++) r.ub[i]=(a.ub[i]>>1)+(b.ub[i]>>1)+(a.ub[i]&b.ub[i]&1);
    check("aveu_b",&g,&r);
    g.i8=_mx128_avers_b(a.i8,b.i8);
    for(int i=0;i<16;i++) r.b[i]=(a.b[i]>>1)+(b.b[i]>>1)+((a.b[i]|b.b[i])&1);
    check("avers_b",&g,&r);
    g.u8=_mx128_averu_b(a.u8,b.u8);
    for(int i=0;i<16;i++) r.ub[i]=(a.ub[i]>>1)+(b.ub[i]>>1)+((a.ub[i]|b.ub[i])&1);
    check("averu_b",&g,&r);

    { U x,y; for(int i=0;i<4;i++){x.w[i]=(i-1)*37+11; y.w[i]=(i-2)*5-1;}
      /* GCC guards the divide intrinsics with bnez4w + break 7 (SIGFPE)
       * for a zero lane, so the divisor lanes are kept non-zero here;
       * the trapless divide-by-0 path is a documented helper detail. */
      g.i32=_mx128_divs_w(x.i32,y.i32);
      for(int i=0;i<4;i++) r.w[i]=x.w[i]/y.w[i];
      check("divs_w",&g,&r);
      g.i32=_mx128_mods_w(x.i32,y.i32);
      for(int i=0;i<4;i++) r.w[i]=x.w[i]%y.w[i];
      check("mods_w",&g,&r);
      for(int i=0;i<4;i++){x.uw[i]=i*1234567u+9; y.uw[i]=i*31+7;}
      g.u32=_mx128_divu_w(x.u32,y.u32);
      for(int i=0;i<4;i++) r.uw[i]=x.uw[i]/y.uw[i];
      check("divu_w",&g,&r);
      g.u32=_mx128_modu_w(x.u32,y.u32);
      for(int i=0;i<4;i++) r.uw[i]=x.uw[i]%y.uw[i];
      check("modu_w",&g,&r);
    }

    g.i32=_mx128_madd_w(c.i32,a.i32,b.i32);
    for(int i=0;i<4;i++) r.w[i]=c.w[i]+a.w[i]*b.w[i];
    check("madd_w",&g,&r);
    g.i32=_mx128_msub_w(c.i32,a.i32,b.i32);
    for(int i=0;i<4;i++) r.w[i]=c.w[i]-a.w[i]*b.w[i];
    check("msub_w",&g,&r);

    g.i32=_mx128_dotps_w(a.i16,b.i16);
    for(int i=0;i<4;i++) r.w[i]=(int32_t)a.h[2*i]*b.h[2*i]+(int32_t)a.h[2*i+1]*b.h[2*i+1];
    check("dotps_w",&g,&r);
    g.u32=_mx128_dotpu_w(a.u16,b.u16);
    for(int i=0;i<4;i++) r.uw[i]=(uint32_t)a.uh[2*i]*b.uh[2*i]+(uint32_t)a.uh[2*i+1]*b.uh[2*i+1];
    check("dotpu_w",&g,&r);
    g.i32=_mx128_dadds_w(c.i32,a.i16,b.i16);
    for(int i=0;i<4;i++) r.w[i]=c.w[i]+(int32_t)a.h[2*i]*b.h[2*i]+(int32_t)a.h[2*i+1]*b.h[2*i+1];
    check("dadds_w",&g,&r);
    g.u32=_mx128_daddu_w(c.u32,a.u16,b.u16);
    for(int i=0;i<4;i++) r.uw[i]=c.uw[i]+(uint32_t)a.uh[2*i]*b.uh[2*i]+(uint32_t)a.uh[2*i+1]*b.uh[2*i+1];
    check("daddu_w",&g,&r);
    g.i32=_mx128_dsubs_w(c.i32,a.i16,b.i16);
    for(int i=0;i<4;i++) r.w[i]=c.w[i]-((int32_t)a.h[2*i]*b.h[2*i]+(int32_t)a.h[2*i+1]*b.h[2*i+1]);
    check("dsubs_w",&g,&r);
    g.i32=_mx128_dsubu_w(c.u32,a.u16,b.u16);
    for(int i=0;i<4;i++) r.uw[i]=c.uw[i]-((uint32_t)a.uh[2*i]*b.uh[2*i]+(uint32_t)a.uh[2*i+1]*b.uh[2*i+1]);
    check("dsubu_w",&g,&r);

    /* Q15 fixed-point multiply (h) */
    g.i16=_mx128_mulq_h(a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,p>>15);}
    check("mulq_h",&g,&r);
    g.i16=_mx128_mulqr_h(a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,(p+(1<<14))>>15);}
    check("mulqr_h",&g,&r);
    g.i16=_mx128_maddq_h(c.i16,a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,c.h[i]+(p>>15));}
    check("maddq_h",&g,&r);
    g.i16=_mx128_msubq_h(c.i16,a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,c.h[i]-(p>>15));}
    check("msubq_h",&g,&r);
    g.i16=_mx128_maddqr_h(c.i16,a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,c.h[i]+((p+(1<<14))>>15));}
    check("maddqr_h",&g,&r);
    g.i16=_mx128_msubqr_h(c.i16,a.i16,b.i16);
    for(int i=0;i<8;i++){int64_t p=(int64_t)a.h[i]*b.h[i];
        if(a.h[i]==INT16_MIN&&b.h[i]==INT16_MIN) p=((int64_t)1<<30)-1;
        r.h[i]=SAT_S(16,c.h[i]-((p+(1<<14))>>15));}
    check("msubqr_h",&g,&r);

    /* ---- unary ---- */
    { U x; for(int i=0;i<16;i++) x.ub[i]=(uint8_t)(0xf0>>(i&3))|(i&1?0x03:0);
      g.i8=_mx128_loc_b(x.i8);
      for(int i=0;i<16;i++){uint8_t v=x.ub[i];int n=0;while(n<8&&(v&0x80)){n++;v<<=1;}r.b[i]=n;}
      check("loc_b",&g,&r);
      g.i8=_mx128_lzc_b(x.i8);
      for(int i=0;i<16;i++){uint8_t v=x.ub[i];int n=0;while(n<8&&!(v&0x80)){n++;v<<=1;}r.b[i]=n;}
      check("lzc_b",&g,&r);
      g.i8=_mx128_bcnt_b(x.i8);
      for(int i=0;i<16;i++){uint8_t v=x.ub[i];int n=0;while(v){n+=v&1;v>>=1;}r.b[i]=n;}
      check("bcnt_b",&g,&r);
    }
    g.i16=_mx128_sats_h(a.i16,5);
    for(int i=0;i<8;i++){int64_t lo=-(1<<5),hi=(1<<5)-1,v=a.h[i];r.h[i]=v<lo?lo:v>hi?hi:v;}
    check("sats_h",&g,&r);
    g.u16=_mx128_satu_h(a.u16,5);
    for(int i=0;i<8;i++){uint64_t hi=(1u<<6)-1,v=a.uh[i];r.uh[i]=v>hi?hi:v;}
    check("satu_h",&g,&r);
    g.i16=_mx128_srari_h(a.i16,3);
    for(int i=0;i<8;i++){int32_t v=a.h[i];r.h[i]=(v>>3)+((v>>2)&1);}
    check("srari_h",&g,&r);
    g.u16=_mx128_srlri_h(a.u16,3);
    for(int i=0;i<8;i++){uint32_t v=a.uh[i];r.uh[i]=(v>>3)+((v>>2)&1);}
    check("srlri_h",&g,&r);

    /* ---- permute / fill ---- */
    g.i32=_mx128_li_w(7); for(int i=0;i<4;i++) r.w[i]=7;
    check("li_w",&g,&r);
    g.i16=_mx128_li_h(-300); for(int i=0;i<8;i++) r.h[i]=-300;
    check("li_h",&g,&r);
    g.i32=_mx128_mfcpu_w(0x12345678); for(int i=0;i<4;i++) r.uw[i]=0x12345678;
    check("mfcpu_w",&g,&r);
    g.i32=_mx128_repi_w(a.i32,2); for(int i=0;i<4;i++) r.w[i]=a.w[2];
    check("repi_w",&g,&r);
    g.i32=_mx128_repx_w(a.i32,3); for(int i=0;i<4;i++) r.w[i]=a.w[3];
    check("repx_w",&g,&r);
    g=a; g.i32=_mx128_insfcpu_w(g.i32,1,0x0bad1dea); r=a; r.uw[1]=0x0bad1dea;
    check("insfcpu_w",&g,&r);
    g=a; g.i32=_mx128_insfmxu_w(g.i32,2,b.i32); r=a; r.w[2]=b.w[0];
    check("insfmxu_w",&g,&r);
    { int s=_mx128_mtcpus_h(a.i16,3); int rr=a.h[3];
      total++; if(s!=rr){fails++;printf("FAIL mtcpus_h %d!=%d\n",s,rr);} }
    { unsigned u=_mx128_mtcpuu_b(a.i8,5); unsigned rr=a.ub[5];
      total++; if(u!=rr){fails++;printf("FAIL mtcpuu_b %u!=%u\n",u,rr);} }

    /* shufv: control (elem<<1)|from_vs, vs=first arg selected when bit0=1 */
    { U ctl; for(int i=0;i<16;i++) ctl.ub[i]=((i&7)<<1)|(i<8?1:0);
      g.i8=_mx128_shufv(a.i8,b.i8,ctl.i8);
      for(int i=0;i<16;i++){unsigned cc=ctl.ub[i];unsigned e=(cc>>1)&15;
        r.ub[i]=(cc&1)?a.ub[e]:b.ub[e];}
      check("shufv",&g,&r);
    }
    /* bselv: mask bit 1 -> vt(second arg), 0 -> vs(first) */
    { U m; for(int i=0;i<16;i++) m.ub[i]=(i&1)?0xff:0x00;
      g.i8=_mx128_bselv(a.i8,b.i8,m.i8);
      for(int i=0;i<16;i++) r.ub[i]=(a.ub[i]&~m.ub[i])|(b.ub[i]&m.ub[i]);
      check("bselv",&g,&r);
    }

    /* ---- loads/stores ---- */
    { static uint8_t buf[64] __attribute__((aligned(16)));
      for(int i=0;i<64;i++) buf[i]=i*3+1;
      g.i8=_mx128_lu1q((void*)(buf+8),0); memcpy(&r,buf+8,16);
      check("lu1q",&g,&r);
      _mx128_su1q(a.i8,(void*)(buf+16),0); memcpy(&r,buf+16,16);
      check("su1q",&a,&r);
      g.i8=_mx128_lu1qx((void*)buf,16); memcpy(&r,buf+16,16);
      check("lu1qx",&g,&r);
      _mx128_su1qx(b.i8,(void*)buf,32); memcpy(&r,buf+32,16);
      check("su1qx",&b,&r);
      g.i8=_mx128_la1q((void*)(buf+0),16); memcpy(&r,buf+16,16);
      check("la1q(scaled)",&g,&r);
    }

    /* ---- floating point ---- */
    { U fa,fb,fc;
      for(int i=0;i<4;i++){fa.f[i]=(i+1)*1.5f; fb.f[i]=(i*0.5f)-1.0f; fc.f[i]=0.25f*i;}
      g.f32=_mx128_fadd_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]+fb.f[i];
      check("fadd_w",&g,&r);
      g.f32=_mx128_fsub_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]-fb.f[i];
      check("fsub_w",&g,&r);
      g.f32=_mx128_fmul_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]*fb.f[i];
      check("fmul_w",&g,&r);
      g.f32=_mx128_fdiv_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]/fb.f[i];
      check("fdiv_w",&g,&r);
      g.f32=_mx128_fmadd_w(fc.f32,fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=(float)((double)fa.f[i]*fb.f[i]+fc.f[i]);
      check("fmadd_w",&g,&r);
      g.f32=_mx128_fmsub_w(fc.f32,fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=(float)(-(double)fa.f[i]*fb.f[i]+fc.f[i]);
      check("fmsub_w",&g,&r);
      g.f32=_mx128_fmax_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]>fb.f[i]?fa.f[i]:fb.f[i];
      check("fmax_w",&g,&r);
      g.f32=_mx128_fmin_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.f[i]=fa.f[i]<fb.f[i]?fa.f[i]:fb.f[i];
      check("fmin_w",&g,&r);
      g.i32=_mx128_fceq_w(fa.f32,fa.f32); for(int i=0;i<4;i++) r.w[i]=-1;
      check("fceq_w",&g,&r);
      g.i32=_mx128_fclt_w(fb.f32,fa.f32); for(int i=0;i<4;i++) r.w[i]=fb.f[i]<fa.f[i]?-1:0;
      check("fclt_w",&g,&r);
      g.i32=_mx128_fcle_w(fb.f32,fa.f32); for(int i=0;i<4;i++) r.w[i]=fb.f[i]<=fa.f[i]?-1:0;
      check("fcle_w",&g,&r);
      g.i32=_mx128_fcor_w(fa.f32,fb.f32); for(int i=0;i<4;i++) r.w[i]=-1;
      check("fcor_w",&g,&r);
      U fp; for(int i=0;i<4;i++) fp.f[i]=(i+1)*4.0f;
      g.f32=_mx128_fsqrt_w(fp.f32); for(int i=0;i<4;i++) r.f[i]=sqrtf(fp.f[i]);
      check("fsqrt_w",&g,&r);
      U iv; for(int i=0;i<4;i++) iv.w[i]=(i-1)*1000;
      g.f32=_mx128_vcvtssw(iv.i32); for(int i=0;i<4;i++) r.f[i]=(float)iv.w[i];
      check("vcvtssw",&g,&r);
      g.f32=_mx128_vcvtusw(iv.u32); for(int i=0;i<4;i++) r.f[i]=(float)iv.uw[i];
      check("vcvtusw",&g,&r);
      U fv; for(int i=0;i<4;i++) fv.f[i]=(i-1)*2.7f;
      g.i32=_mx128_vtruncsws(fv.f32); for(int i=0;i<4;i++) r.w[i]=(int)fv.f[i];
      check("vtruncsws",&g,&r);
    }

    /* ---- control register ---- */
    _mx128_ctcmxu(31, 0x0000a5a5);
    { int v=_mx128_cfcmxu(31); total++;
      if((v&0xffff)!=0xa5a5){fails++;printf("FAIL cfcmxu got %08x\n",v);} }

    /*
     * ---- lazy CU2 enable + context switch ----
     * Two processes each own a distinct MXU2 register value and spin
     * accumulating into it across many scheduler slices.  If the kernel's
     * CU2 lazy-enable or MXU2 context save/restore were wrong, the two
     * would clobber each other's $vr state and the running sums diverge.
     * Reaching this code at all also exercises the first-touch CU2 trap
     * (Status.CU2 starts clear for each new process).
     */
    {
        int st0 = 0, st1 = 0, p;
        pid_t pids[2];
        for (p = 0; p < 2; p++) {
            pid_t pid = fork();
            if (pid == 0) {
                /* child: keep a private vector accumulator in $vr */
                int step = (p == 0 ? 7 : 13);
                v4i32 acc = _mx128_mfcpu_w(0);
                v4i32 one = _mx128_mfcpu_w(step);
                uint32_t sum = 0;
                for (int it = 0; it < 200000; it++) {
                    acc = _mx128_add_w(acc, one);
                    /* force use of the FPU path too, between slices */
                    if ((it & 0x3fff) == 0) {
                        sched_yield();
                    }
                }
                sum = (uint32_t)_mx128_mtcpus_w(acc, 0);
                _exit((sum == (uint32_t)(200000 * step)) ? 0 : 1);
            }
            pids[p] = pid;
        }
        waitpid(pids[0], &st0, 0);
        waitpid(pids[1], &st1, 0);
        total++;
        if (!(WIFEXITED(st0) && WEXITSTATUS(st0) == 0 &&
              WIFEXITED(st1) && WEXITSTATUS(st1) == 0)) {
            fails++;
            printf("FAIL ctxswitch st0=%d st1=%d\n", st0, st1);
        }
    }

    if (fails == 0) {
        printf("MXU2 TESTS: ALL PASS (%d)\n", total);
    } else {
        printf("MXU2 TESTS: %d/%d FAILED\n", fails, total);
    }
    fflush(stdout);
    return fails ? 1 : 0;
}
EOF
if ! "$CC" $CFLAGS -I"$TOOLCHAIN/lib/gcc/mips-linux-uclibc/5.4.0/include" \
        "$WORK/test_mxu2.c" -o "$WORK/test_mxu2" -lm 2>"$WORK/cc1.log"; then
    echo "FAIL: could not build test_mxu2"; sed 's/^/  /' "$WORK/cc1.log"; exit 1
fi
OUT="$(run_init "$WORK/test_mxu2")"
if echo "$OUT" | grep -qa 'MXU2 TESTS: ALL PASS'; then
    echo "PASS: $(echo "$OUT" | grep -a 'MXU2 TESTS' | tail -1 | sed 's/\r//')"
else
    echo "FAIL: instruction/context test"; echo "$OUT" | grep -a 'FAIL\|TESTS\|SIGILL\|not syncing' | head; fail=1
fi

# ---- 2. vendor MXU2 binary (jz_memcpy from OpenIPC libimp.so) ------------
if [ -f "$ROOTFS" ] && command -v unsquashfs >/dev/null 2>&1; then
    unsquashfs -q -n -f -d "$WORK/rootfs" "$ROOTFS" usr/lib/libimp.so >/dev/null 2>&1
    LIBIMP="$WORK/rootfs/usr/lib/libimp.so"
    if [ -f "$LIBIMP" ] && python3 - "$LIBIMP" "$WORK/jzmemcpy_bytes.h" <<'PYEXTRACT' 2>/dev/null; then
import sys, struct
f = open(sys.argv[1], 'rb').read()
# map the virtual address of jz_memcpy to a file offset via the program headers
e_phoff = struct.unpack_from('<I', f, 0x1c)[0]
e_phentsize = struct.unpack_from('<H', f, 0x2a)[0]
e_phnum = struct.unpack_from('<H', f, 0x2c)[0]
def v2o(v):
    for i in range(e_phnum):
        o = e_phoff + i * e_phentsize
        if struct.unpack_from('<I', f, o)[0] != 1:
            continue
        p_off, p_va, _pa, p_fsz = struct.unpack_from('<IIII', f, o + 4)
        if p_va <= v < p_va + p_fsz:
            return p_off + (v - p_va)
    return None
# jz_memcpy is a dynamic symbol; find its value in .dynsym
def find_sym(name):
    import struct as st
    # walk section headers for .dynsym/.dynstr
    e_shoff = st.unpack_from('<I', f, 0x20)[0]
    e_shentsize = st.unpack_from('<H', f, 0x2e)[0]
    e_shnum = st.unpack_from('<H', f, 0x30)[0]
    e_shstrndx = st.unpack_from('<H', f, 0x32)[0]
    sh = [st.unpack_from('<IIIIIIIIII', f, e_shoff + i * e_shentsize) for i in range(e_shnum)]
    shstr = sh[e_shstrndx][4]
    def sname(off): 
        e = f.index(b'\0', shstr + off); return f[shstr+off:e].decode()
    dynsym = dynstr = None
    for s in sh:
        nm = sname(s[0])
        if nm == '.dynsym': dynsym = s
        if nm == '.dynstr': dynstr = s
    off, size, entsz, stroff = dynsym[4], dynsym[5], dynsym[9], dynstr[4]
    for i in range(size // entsz):
        st_name, st_value, st_size = st.unpack_from('<III', f, off + i * entsz)
        e = f.index(b'\0', stroff + st_name)
        if f[stroff+st_name:e].decode() == name:
            return st_value, st_size
    return None, None
va, size = find_sym('jz_memcpy')
if va is None:
    sys.exit(1)
size = (size + 15) & ~15 if size else 128
off = v2o(va)
b = f[off:off+size]
arr = ','.join('0x%02x' % x for x in b)
open(sys.argv[2], 'w').write(
    '/* jz_memcpy extracted from OpenIPC T31 /usr/lib/libimp.so '
    '(a self-contained leaf MXU2 memcpy, no relocations) */\n'
    'static const unsigned char jz_memcpy_code[%d] __attribute__((aligned(16))) = {%s};\n'
    % (len(b), arr))
PYEXTRACT
        cat > "$WORK/vendor_mxu2_demo.c" <<'EOF'
/*
 * Vendor MXU2 demonstration.
 *
 * jz_memcpy() from the OpenIPC T31 vendor library /usr/lib/libimp.so is a
 * self-contained leaf routine (no GOT/relocations) that copies memory with
 * the MXU2 128-bit loads/stores lu1q/lu1qx/su1q/su1qx.  Its exact machine
 * bytes are embedded here (see jzmemcpy_bytes.h, extracted with objdump)
 * and executed from an mmap'd RWX page: this runs the vendor's real MXU2
 * instruction stream under QEMU.  If MXU2 were not decoded it would take a
 * Reserved Instruction / SIGILL; a correct copy proves it ran.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/mman.h>
#include "jzmemcpy_bytes.h"

typedef void (*memcpy_fn)(const void *src, void *dst, unsigned long n);

int main(void)
{
    void *page = mmap(NULL, 4096, PROT_READ | PROT_WRITE | PROT_EXEC,
                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (page == MAP_FAILED) {
        printf("VENDOR MXU2: mmap failed\n");
        return 1;
    }
    memcpy(page, jz_memcpy_code, sizeof(jz_memcpy_code));
    __builtin___clear_cache(page, (char *)page + sizeof(jz_memcpy_code));

    static unsigned char src[256], dst[256], ref[256];
    for (int i = 0; i < 256; i++) {
        src[i] = (unsigned char)(i * 7 + 3);
        dst[i] = 0;
    }
    memcpy(ref, src, 200);

    memcpy_fn fn = (memcpy_fn)page;
    fn(src, dst, 200);     /* vendor jz_memcpy(src, dst, n): a0 read, a1 written */

    if (memcmp(dst, ref, 200) == 0) {
        printf("VENDOR MXU2: jz_memcpy (libimp.so) ran OK, 200 bytes copied\n");
        return 0;
    }
    { int k; for(k=0;k<200;k++) if(dst[k]!=ref[k]){printf("VENDOR MXU2: MISMATCH at %d got %02x exp %02x\n",k,dst[k],ref[k]);break;} }
    return 1;
}
EOF
        if "$CC" -static -EL -O2 -march=mips32r2 -I"$WORK" \
                "$WORK/vendor_mxu2_demo.c" -o "$WORK/vendor_demo" 2>"$WORK/cc2.log"; then
            OUT="$(run_init "$WORK/vendor_demo")"
            if echo "$OUT" | grep -qa 'VENDOR MXU2: jz_memcpy'; then
                echo "PASS: $(echo "$OUT" | grep -a 'VENDOR MXU2' | tail -1 | sed 's/\r//')"
            else
                echo "FAIL: vendor MXU2 demo"; echo "$OUT" | grep -a 'VENDOR\|SIGILL' | head; fail=1
            fi
        else
            echo "SKIP: vendor demo build failed"; sed 's/^/  /' "$WORK/cc2.log"
        fi
    else
        echo "SKIP: could not extract jz_memcpy from libimp.so"
    fi
else
    echo "SKIP: vendor rootfs or unsquashfs unavailable (instruction test still ran)"
fi

exit $fail
