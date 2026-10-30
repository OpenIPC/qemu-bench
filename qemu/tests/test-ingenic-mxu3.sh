#!/bin/bash
#
# MXU3 (Ingenic XBurst2 512-bit SIMD) test for the t40/t41 machines.
#
# Builds a static test program with the Ingenic GCC 7.2 toolchain, boots
# it as PID 1 (rdinit=/init) on the t40 machine, and checks that every
# implemented MXU3 instruction matches a portable C reference computed in
# the same program.  It also exercises the kernel's lazy CU2 enable (the
# first MXU3 op traps) and concurrent MXU3 use by two processes across
# many context switches.
#
# T41: the SDK kernel has no initramfs, so the program (built with
# -mmxu3.1, adding T41's three MXU3.1 shuffles) is /init of a SquashFS on
# the NOR (root=/dev/mtdblock0), and the kernel is the T41 SDK flash
# image's own (T41_IMAGE, the uImage at 0x40000) unless $T41_KERNEL names
# one.  The MXU3.1 shuffles' semantics are opaque in every source (UNSPEC
# RTL); the check is that they decode and retire under the lazy enable and
# are reported as approximations, each once under -d unimp and counted.
#
# Skips cleanly (exit 0) if the toolchain or a kernel image is absent.
#
# --ivs additionally runs libimp's IVS move detection (libimp's MXU3 code)
# on emulated camera frames, via qemu-boot/test-ingenic-isp.sh's T40 flow,
# and fails if any approximated MXU3 mnemonic executes there.
#
set -u

IVS=
[ "${1:-}" = "--ivs" ] && IVS=1

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
QEMU="${QEMU:-$ROOT/qemu-src/build/qemu-system-mipsel}"

# Ingenic GCC 7.2 (fp64) toolchain: honour $CROSS, else search common spots.
find_cc() {
    if [ -n "${CROSS:-}" ] && command -v "${CROSS}gcc" >/dev/null 2>&1; then
        echo "${CROSS}gcc"; return
    fi
    for d in /home/ai/toolchains/ingenic/mips-linux-gnu-ingenic-gcc7.2.0-glibc2.29-fp64-r5.1.8 \
             "$HOME"/toolchains/ingenic/*gcc7.2.0*fp64*; do
        if [ -x "$d/bin/mips-linux-gnu-gcc" ]; then
            echo "$d/bin/mips-linux-gnu-gcc"; return
        fi
    done
}
CC="$(find_cc)"
if [ -z "$CC" ]; then
    echo "SKIP: Ingenic GCC 7.2 (fp64) toolchain not found (set \$CROSS)"; exit 0
fi
if [ ! -x "$QEMU" ]; then
    echo "SKIP: $QEMU not built"; exit 0
fi

# Kernel images (t41 optional).  Override with $T40_KERNEL / $T41_KERNEL.
T40_KERNEL="${T40_KERNEL:-/home/ai/dimerr/local/ingenic/fw/t40/uImage.t40}"
T41_KERNEL="${T41_KERNEL:-}"
T41_IMAGE="${T41_IMAGE:-/home/ai/dimerr/local/ingenic/extract/t41-sdk-full/Ingenic-SDK-T41-1.2.0-20230713/resource/image_t41/T41NQ_gcc720_uclibc_16M_202307061021.img}"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

cat > "$WORK/mxu3-test.c" <<'CEOF'
#define _GNU_SOURCE
/* MXU3 self-test: runs on the t40/t41 guest as PID 1.  Each check runs an
 * MXU3 instruction (via the Ingenic GCC intrinsics / inline asm) and
 * compares against a portable C reference computed here.  Also exercises
 * the kernel lazy CU2 enable (the first MXU3 op traps) and a two-process
 * context switch (each process keeps a distinct 512-bit value live in a
 * vector register across many sched_yield()s). */
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <sys/wait.h>
#include <sys/reboot.h>
#include <mxu3.h>

static int fails, checks;
static void out(const char *s){ write(1, s, strlen(s)); }
static void outx(const char *p, uint32_t v){
    char b[16]; int i; out(p);
    for (i=7;i>=0;i--){ int d=(v>>(i*4))&0xf; b[7-i]=d<10?'0'+d:'a'+d-10; }
    b[8]='\n'; write(1,b,9);
}
#define CK(name, cond) do{ checks++; if(!(cond)){ fails++; out("FAIL "name"\n"); } \
                           else out("ok   "name"\n"); }while(0)

/* 64-byte aligned scratch */
static uint8_t A[64] __attribute__((aligned(64)));
static uint8_t B[64] __attribute__((aligned(64)));
static uint8_t R[64] __attribute__((aligned(64)));

int main(void)
{
    int i;
    /* MXU3 is usable only on CPU0 on T40 (the kernel SIGILLs it on CPU1);
     * pin there before the first MXU3 instruction. */
    { cpu_set_t set; CPU_ZERO(&set); CPU_SET(0, &set);
      sched_setaffinity(0, sizeof set, &set); }
    volatile v16i32 *pa=(void*)A,*pb=(void*)B,*pr=(void*)R;

    /* addw: 16 x int32 add (first MXU3 op -> exercises CU2 lazy enable) */
    for(i=0;i<16;i++){ ((int32_t*)A)[i]=i*7-3; ((int32_t*)B)[i]=1000-i*i; }
    *(v16i32*)R=_mx512_addw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=(i*7-3)+(1000-i*i)) ok=0;
      CK("addw", ok); }

    /* subw */
    *(v16i32*)R=_mx512_subw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=(i*7-3)-(1000-i*i)) ok=0;
      CK("subw", ok); }

    /* mulw (low 32 bits) */
    *(v16i32*)R=_mx512_mulw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=(int32_t)((i*7-3)*(1000-i*i))) ok=0;
      CK("mulw", ok); }

    /* maxsw / minsw */
    *(v16i32*)R=_mx512_maxsw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++){ int32_t x=i*7-3,y=1000-i*i; if(((int32_t*)R)[i]!=(x>y?x:y)) ok=0;} CK("maxsw",ok);}
    *(v16i32*)R=_mx512_minsw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++){ int32_t x=i*7-3,y=1000-i*i; if(((int32_t*)R)[i]!=(x<y?x:y)) ok=0;} CK("minsw",ok);}

    /* bitwise */
    *(v64u8*)R=_mx512_andv(*(v64u8*)A,*(v64u8*)B);
    { int ok=1; for(i=0;i<64;i++) if(R[i]!=(A[i]&B[i])) ok=0; CK("andv",ok);}
    *(v64u8*)R=_mx512_orv(*(v64u8*)A,*(v64u8*)B);
    { int ok=1; for(i=0;i<64;i++) if(R[i]!=(A[i]|B[i])) ok=0; CK("orv",ok);}
    *(v64u8*)R=_mx512_xorv(*(v64u8*)A,*(v64u8*)B);
    { int ok=1; for(i=0;i<64;i++) if(R[i]!=(A[i]^B[i])) ok=0; CK("xorv",ok);}

    /* byte add + unsigned min/max on bytes */
    for(i=0;i<64;i++){ A[i]=i*3+1; B[i]=200-i; }
    *(v64i8*)R=_mx512_addb(*(v64i8*)A,*(v64i8*)B);
    { int ok=1; for(i=0;i<64;i++) if(R[i]!=(uint8_t)(A[i]+B[i])) ok=0; CK("addb",ok);}
    *(v64u8*)R=_mx512_maxub(*(v64u8*)A,*(v64u8*)B);
    { int ok=1; for(i=0;i<64;i++){ uint8_t x=A[i],y=B[i]; if(R[i]!=(x>y?x:y)) ok=0;} CK("maxub",ok);}
    *(v64u8*)R=_mx512_minub(*(v64u8*)A,*(v64u8*)B);
    { int ok=1; for(i=0;i<64;i++){ uint8_t x=A[i],y=B[i]; if(R[i]!=(x<y?x:y)) ok=0;} CK("minub",ok);}

    /* halfword add */
    for(i=0;i<32;i++){ ((int16_t*)A)[i]=i*101-500; ((int16_t*)B)[i]=i*7; }
    *(v32i16*)R=_mx512_addh(*(v32i16*)A,*(v32i16*)B);
    { int ok=1; for(i=0;i<32;i++) if(((int16_t*)R)[i]!=(int16_t)((i*101-500)+(i*7))) ok=0; CK("addh",ok);}

    /* immediate shifts on words */
    for(i=0;i<16;i++) ((int32_t*)A)[i]=0x12340000 + i;
    *(v16i32*)R=_mx512_slliw(*pa,4);
    { int ok=1; for(i=0;i<16;i++) if(((uint32_t*)R)[i]!=((uint32_t)(0x12340000+i)<<4)) ok=0; CK("slliw",ok);}
    *(v16i32*)R=_mx512_sraiw(*pa,4);
    { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=((int32_t)(0x12340000+i)>>4)) ok=0; CK("sraiw",ok);}

    /* abs */
    for(i=0;i<16;i++) ((int32_t*)A)[i]=(i&1)? -(i*111):(i*77);
    *(v16i32*)R=_mx512_absw(*pa);
    { int ok=1; for(i=0;i<16;i++){ int32_t x=((i&1)?-(i*111):(i*77)); if(((int32_t*)R)[i]!=(x<0?-x:x)) ok=0;} CK("absw",ok);}

    /* ceqw: produce -1/0 mask */
    for(i=0;i<16;i++){ ((int32_t*)A)[i]=i; ((int32_t*)B)[i]=(i%3==0)?i:999; }
    *(v16i32*)R=_mx512_ceqw(*pa,*pb);
    { int ok=1; for(i=0;i<16;i++){ int32_t e=(i==((i%3==0)?i:999))?-1:0; if(((int32_t*)R)[i]!=e) ok=0;} CK("ceqw",ok);}

    /* extubl: zero-extend low 32 bytes -> 32 halfwords */
    for(i=0;i<64;i++) A[i]=i+1;
    *(v32u16*)R=_mx512_extubl(*(v64u8*)A);
    { int ok=1; for(i=0;i<32;i++) if(((uint16_t*)R)[i]!=(uint16_t)A[i]) ok=0; CK("extubl",ok);}

    /* smulbe: signed even-byte multiply -> 32 halfwords */
    for(i=0;i<64;i++){ ((int8_t*)A)[i]=i-32; ((int8_t*)B)[i]=(i&7)-4; }
    *(v32i16*)R=_mx512_smulbe(*(v64i8*)A,*(v64i8*)B);
    { int ok=1; for(i=0;i<32;i++){ int16_t e=(int16_t)(((int8_t*)A)[2*i]*((int8_t*)B)[2*i]);
        if(((int16_t*)R)[i]!=e) ok=0;} CK("smulbe",ok);}

    /* float add/mul/sub */
    { float fa[16],fb[16]; for(i=0;i<16;i++){ fa[i]=i*1.5f-3.0f; fb[i]=0.25f*i+1.0f; }
      memcpy(A,fa,64); memcpy(B,fb,64);
      *(v16f32*)R=_mx512_faddw(*(v16f32*)A,*(v16f32*)B);
      { int ok=1; float *fr=(float*)R; for(i=0;i<16;i++) if(fr[i]!=fa[i]+fb[i]) ok=0; CK("faddw",ok);}
      *(v16f32*)R=_mx512_fmulw(*(v16f32*)A,*(v16f32*)B);
      { int ok=1; float *fr=(float*)R; for(i=0;i<16;i++) if(fr[i]!=fa[i]*fb[i]) ok=0; CK("fmulw",ok);}
      /* ftsiw: float -> signed int (truncate) */
      *(v16i32*)R=_mx512_ftsiw(*(v16f32*)A);
      { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=(int32_t)fa[i]) ok=0; CK("ftsiw",ok);}
    }

    /* satsuwb: clamp signed word to [0,255] */
    for(i=0;i<16;i++) ((int32_t*)A)[i]=(i-4)*100;
    *(v16i32*)R=_mx512_satsuwb(*pa);
    { int ok=1; for(i=0;i<16;i++){ int32_t x=(i-4)*100,e=x<0?0:x>255?255:x; if(((int32_t*)R)[i]!=e) ok=0;} CK("satsuwb",ok);}

    /* repiw: broadcast word lane 3 */
    for(i=0;i<16;i++) ((int32_t*)A)[i]=0xdead00+i;
    *(v16i32*)R=_mx512_repiw(*pa,3);
    { int ok=1; for(i=0;i<16;i++) if(((int32_t*)R)[i]!=0xdead00+3) ok=0; CK("repiw",ok);}

    /* ilvew: interleave even words */
    for(i=0;i<16;i++){ ((int32_t*)A)[i]=0xA0+i; ((int32_t*)B)[i]=0xB0+i; }
    *(v16u32*)R=_mx512_ilvew(*(v16u32*)A,*(v16u32*)B);
    { int ok=1; for(i=0;i<8;i++){ if(((int32_t*)R)[2*i]!=0xA0+i) ok=0; if(((int32_t*)R)[2*i+1]!=0xB0+i) ok=0;} CK("ilvew",ok);}

    /* gather: gtNx rd[i], rt, j puts rt.x[N*k+j] into block i of rd and
     * keeps the rest (GCC RTL); gtb packs, per 64-bit lane, the even bytes
     * into the low word and the odd bytes into the high word */
    {
        uint32_t s32[16] __attribute__((aligned(64))), d32[16] __attribute__((aligned(64)));
        uint32_t o32[16] __attribute__((aligned(64))), e[16];
        uint64_t *s64 = (uint64_t *)s32, *o64 = (uint64_t *)o32, *d64 = (uint64_t *)d32;
        int ok, k, N, ii, jj, m;
        for (k = 0; k < 16; k++) { s32[k] = 0x1000 + k; d32[k] = 0xd0d0d000 + k; }
#define GTW(nm, n, ib, jb) do { \
        ok = 1; \
        for (ii = 0; ii < n; ii++) for (jj = 0; jj < n; jj++) { \
            memcpy(o32, d32, 64); \
            *(v16i32*)o32 = __builtin_mxu3_##nm(*(v16i32*)o32, ib, *(v16i32*)s32, jb); \
            if (ii != ib || jj != jb) continue; \
            m = 16 / n; memcpy(e, d32, 64); \
            for (k = 0; k < m; k++) e[ib * m + k] = s32[n * k + jb]; \
            if (memcmp(e, o32, 64)) ok = 0; \
        } CK(#nm, ok); } while (0)
        /* immediates must be constants: check representative (i, j) pairs */
        GTW(gt2w, 2, 1, 1); GTW(gt2w, 2, 0, 0); GTW(gt4w, 4, 2, 3); GTW(gt8w, 8, 5, 6);
        /* 64-bit gathers */
        { uint64_t ed[8];
#define GTD(nm, n, ib, jb, words) do { \
            memcpy(o32, d32, 64); \
            *(v16i32*)o32 = __builtin_mxu3_##nm(*(v16i32*)o32, ib, *(v16i32*)s32, jb); \
            memcpy(ed, d64, 64); m = (8 / (words)) / n; \
            for (k = 0; k < m; k++) for (N = 0; N < (words); N++) \
                ed[(ib * m + k) * (words) + N] = s64[(n * k + jb) * (words) + N]; \
            CK(#nm, !memcmp(ed, o64, 64)); } while (0)
          GTD(gt2d, 2, 1, 0, 1); GTD(gt4d, 4, 3, 1, 1); GTD(gt2q, 2, 0, 1, 2);
        }
        /* gtb */
        { uint8_t sb[64] __attribute__((aligned(64))), ob[64] __attribute__((aligned(64))), eb[64];
          for (k = 0; k < 64; k++) sb[k] = k * 7 + 3;
          *(v64i8*)ob = _mx512_gtb(*(v64i8*)sb);
          for (ii = 0; ii < 8; ii++) for (k = 0; k < 4; k++) {
              eb[ii * 8 + k] = sb[ii * 8 + 2 * k]; eb[ii * 8 + 4 + k] = sb[ii * 8 + 2 * k + 1]; }
          CK("gtb", !memcmp(eb, ob, 64));
          /* libimp's narrowing idiom: 32 halfwords -> 32 packed bytes */
          for (k = 0; k < 32; k++) ((uint16_t *)sb)[k] = 0x5500 + k * 5;
          *(v64i8*)ob = _mx512_gtb(*(v64i8*)sb);
          *(v16i32*)ob = __builtin_mxu3_gt2w(*(v16i32*)ob, 0, *(v16i32*)ob, 0);
          ok = 1; for (k = 0; k < 32; k++) if (ob[k] != (uint8_t)(k * 5)) ok = 0;
          CK("gtb+gt2w narrow", ok);
        }
    }

    /* Approximated-op reporting: gth's semantics are not confirmed by any
     * available source; run it twice so the harness can check that QEMU
     * warns once (-d unimp) and counts two executions (QEMU_MXU3_STATS). */
    for (i = 0; i < 2; i++) {
        *(v64i8*)R = _mx512_gth(*(v64i8*)A);
        asm volatile("" ::: "memory");
    }
    out("approx-probe done\n");

#ifdef MXU31
    /* MXU3.1 (T41): gshufvb, gshufwb_1, gshufwb_2.  No source gives their
     * semantics (the builtins are UNSPEC); the model keeps the destination
     * and reports them.  Two of each: decode, retire, one warning each. */
    for (i = 0; i < 2; i++) {
        *(v64u8*)R = _mx512_gshufvb(*(v64u8*)A, *(v64u8*)B, *(v64u8*)R);
        *(v64u8*)R = _mx512_gshufwb_1(*(v64u8*)A, *(v16u32*)B, *(v64u8*)R, 1);
        *(v64u8*)R = _mx512_gshufwb_2(*(v64u8*)A, *(v16u32*)B, *(v64u8*)R, 0);
        asm volatile("" ::: "memory");
    }
    out("mxu3.1-probe done\n");
#endif

    /* ---- two-process concurrent-use test ----
     * Parent and child both repeatedly run an MXU3 computation (operands
     * reloaded from memory each pass) and verify their own result while
     * forcing context switches with sched_yield().  This exercises MXU3
     * being live in two processes across many switches.  (Bit-exact
     * preservation of a register held live across a switch additionally
     * depends on the guest kernel's MXUv3 switch_to save/restore.) */
    {
        pid_t cc = fork();
        int me = (cc == 0) ? 0x11 : 0x22, bad = 0, it;
        int la[16] __attribute__((aligned(64)));
        int lb[16] __attribute__((aligned(64)));
        int lr[16] __attribute__((aligned(64)));
        if (cc == 0) { cpu_set_t s2; CPU_ZERO(&s2); CPU_SET(0,&s2);
                       sched_setaffinity(0, sizeof s2, &s2); }
        for (it = 0; it < 500; it++) {
            for (i = 0; i < 16; i++) { la[i] = me*i + it; lb[i] = (me<<8) - i; }
            *(v16i32*)lr = _mx512_addw(*(v16i32*)la, *(v16i32*)lb);
            for (i = 0; i < 16; i++) if (lr[i] != (me*i+it)+((me<<8)-i)) bad++;
            sched_yield();
        }
        if (cc == 0) { _exit(bad ? 1 : 0); }
        { int st = 0; waitpid(cc, &st, 0);
          CK("concurrent-parent", bad == 0);
          CK("concurrent-child", WIFEXITED(st) && WEXITSTATUS(st) == 0); }
    }

    outx("checks=", checks);
    outx("fails =", fails);
    out(fails? "MXU3-RESULT: FAIL\n" : "MXU3-RESULT: PASS\n");
    /* give the serial time to flush, then halt the machine */
    for(volatile long k=0;k<2000000;k++){}
    reboot(0x4321fedc); /* RB_POWER_OFF */
    for(;;) pause();
    return 0;
}
CEOF

echo "Building MXU3 test with $CC ..."
if ! "$CC" -fno-lto -O2 -mips32r2 -mmxu3 -flax-vector-conversions -static \
        -o "$WORK/init" "$WORK/mxu3-test.c" 2>"$WORK/cc.log"; then
    echo "SKIP: toolchain cannot build the MXU3 test:"; sed 's/^/  /' "$WORK/cc.log"
    exit 0
fi
( cd "$WORK" && mkdir -p irfs && cp init irfs/init && chmod +x irfs/init &&
  ( cd irfs && find . | cpio -o -H newc 2>/dev/null | gzip -9 > ../initramfs.cpio.gz ) )
# T41: the MXU3.1 build, as /init of a SquashFS root (no initramfs there)
T41_ROOT=
if "$CC" -fno-lto -O2 -mips32r2 -mmxu3.1 -DMXU31 -flax-vector-conversions \
        -static -o "$WORK/init31" "$WORK/mxu3-test.c" 2>"$WORK/cc31.log" &&
   command -v mksquashfs >/dev/null; then
    mkdir -p "$WORK/sqfs/dev" "$WORK/sqfs/proc"
    cp "$WORK/init31" "$WORK/sqfs/init"
    mksquashfs "$WORK/sqfs" "$WORK/root31.sq" -noappend -quiet -all-root \
        >/dev/null && T41_ROOT="$WORK/root31.sq"
fi

run_one() {
    local mach="$1"
    local kernel="$2"
    local log="$WORK/$mach.log"
    echo "=== $mach : booting $kernel ==="
    local boot=(-initrd "$WORK/initramfs.cpio.gz"
                -append "rdinit=/init console=ttyS1,115200n8")
    if [ "$mach" = t41 ]; then
        # the SquashFS at 0 of an otherwise blank 8 MiB NOR
        head -c $((8 << 20)) /dev/zero | tr '\0' '\377' > "$WORK/nor41.bin"
        dd if="$T41_ROOT" of="$WORK/nor41.bin" conv=notrunc status=none
        mach="t41,flash-file=$WORK/nor41.bin"
        boot=(-append "console=ttyS1,115200n8 root=/dev/mtdblock0 rootfstype=squashfs init=/init mtdparts=sfc0_nor:-(root)")
        log="$WORK/t41.log"
    fi
    QEMU_MXU3_STATS=1 timeout 150 "$QEMU" -M "$mach" -m 128M -nographic \
        -serial mon:stdio -display none -d unimp -D "$WORK/${mach%%,*}.unimp" \
        -kernel "$kernel" "${boot[@]}" </dev/null \
        > "$log" 2> "$WORK/${mach%%,*}.stats"
    mach=${mach%%,*}
    grep -E "^(ok|FAIL) " "$log" | sed 's/^/  /'
    if grep -q "MXU3-RESULT: PASS" "$log"; then
        # approximated ops: one LOG_UNIMP line per mnemonic, exact count
        local warns cnt
        warns=$(grep -c "mxu3: gth executed" "$WORK/$mach.unimp" 2>/dev/null)
        cnt=$(awk '$1=="mxu3-approx" && $2=="gth" {print $3}' "$WORK/$mach.stats")
        if [ "$warns" = 1 ] && [ "$cnt" = 2 ]; then
            echo "  ok   approx-report (gth: 1 LOG_UNIMP warning, 2 counted)"
        else
            echo "  FAIL approx-report (warnings=$warns count=${cnt:-none})"
            return 1
        fi
        if [ "$mach" = t41 ]; then
            local op ok=1
            grep -q "mxu3.1-probe done" "$log" || ok=0
            for op in gshufvb gshufwb_1 gshufwb_2; do
                warns=$(grep -c "mxu3: $op executed" "$WORK/$mach.unimp" 2>/dev/null)
                cnt=$(awk -v o=$op '$1=="mxu3-approx" && $2==o {print $3}' "$WORK/$mach.stats")
                if [ "$warns" = 1 ] && [ "$cnt" = 2 ]; then
                    echo "  ok   mxu3.1 $op (decodes; 1 LOG_UNIMP warning, 2 counted)"
                else
                    echo "  FAIL mxu3.1 $op (warnings=$warns count=${cnt:-none})"
                    ok=0
                fi
            done
            [ $ok = 1 ] || return 1
        fi
        echo "  $mach: PASS"; return 0
    fi
    # Distinguish an MXU3 failure from the kernel never reaching /init
    # (e.g. an SDK kernel that cannot mount an external initramfs).
    if ! grep -qE "^(ok|FAIL) " "$log" &&        grep -qiE "Unable to mount root fs|Cannot open root device" "$log"; then
        echo "  $mach: SKIP (kernel did not load the initramfs; not an MXU3 result)"
        return 0
    fi
    echo "  $mach: FAIL"; tail -20 "$log" | sed 's/^/  | /'
    return 1
}

# ---- --ivs: libimp's MXU3 code on real frames ---------------------------
# Runs the repo's ISP test flow (qemu-boot/test-ingenic-isp.sh, T40, OpenIPC
# kernel + rootfs, sensor model) from a patched temporary copy: libimp's
# SIMD left on, the capture client replaced by one that also drives IVS
# move detection (embedded below), and QEMU_MXU3_STATS=all.  Passes when
# the synthetic IVS cases are right, camera frames went through IVS, and no
# approximated MXU3 mnemonic executed.
run_ivs() {
    local isp="$ROOT/qemu-boot/test-ingenic-isp.sh"
    local fw="${T40_FW:-$(dirname "$T40_KERNEL")}"
    local d="$WORK/ivs" out frames approx
    [ -r "$isp" ] || { echo "SKIP ivs: $isp not found"; return 0; }
    [ -r "$fw/rootfs.squashfs.t40" ] || { echo "SKIP ivs: no rootfs.squashfs.t40 in $fw"; return 0; }
    mkdir -p "$d"; out="$d/out"
    cat > "$d/ivs-snap.c" <<'IVSEOF'
/*
 * MXU3 IVS check (test-ingenic-mxu3.sh --ivs): qemu-boot/ingenic-isp-snap.c
 * plus, with IVS=1 in the environment, libimp's IVS move detection on
 * channel 1's 640x360 frames -- libimp's MXU3 code -- and, with
 * IVS_SYNTH=1, deterministic synthetic sequences with a PASS/FAIL verdict
 * ("ivs-synth:").  The original description follows.
 *
 * Capture NV12 frames through libimp's FrameSource on an Ingenic T31 --
 * the guest half of qemu-boot/test-ingenic-isp.sh.  Follows the T31 SDK
 * 1.1.5 samples (sample-common.c, sample-Framesource.c): ISP open, add and
 * enable the sensor, IMP_System_Init, tuning on, then FrameSource
 * channel 0 at the sensor's 1920x1080 and channel 1 scaled to 640x360.
 *
 *   ingenic-isp-snap [frames [outdir [sensor [i2c-addr [width height]]]]]
 *
 * (defaults 10, /tmp, gc2053, 0x37, 1920 1080: channel 0's size)
 *
 * Each channel's thread gets @frames frames (default 10), prints a line
 * per frame (sequence, timestamp, a sum of the Y plane) and writes frames
 * 2 and @frames - 1 to <outdir>/ch<n>-<k>.nv12 -- or, when @outdir is a
 * block device (the SD card; a 1080p frame does not fit in the RAM of a
 * 64 MiB camera beside libimp), into 8 MiB slots on it: channel 0's two
 * frames in slots 0 and 1, channel 1's in 2 and 3.  Exits 0 when every
 * channel got all of its frames.
 *
 * Built two ways (test-ingenic-isp.sh does both):
 *   - static, against the SDK's uClibc libimp.a (IMP 1.1.5);
 *   - with -DSNAP_MUSL, dynamically against the rootfs's own libimp.so
 *     (IMP 1.1.6, the build that matches its tx-isp-t31.ko) and musl.
 *     That libimp is a uClibc build, which majestic runs by exporting the
 *     few uClibc-only symbols it calls; this file does the same, and turns
 *     the uClibc crt1.o's call to __uClibc_main into musl's
 *     __libc_start_main (the same first six arguments).  mmap and
 *     ftruncate take a 32-bit off_t in uClibc and a 64-bit one in musl
 *     (in a register pair on o32): libimp's calls would land in musl's
 *     with the arguments shifted, so they go to the system calls here.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#ifdef SNAP_MUSL
#include <ctype.h>
#endif
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_isp.h>
#include <imp/imp_framesource.h>

#ifdef SNAP_MUSL
int __libc_start_main(int (*m)(int, char **, char **), int argc, char **argv,
                      void (*init)(void), void (*fini)(void),
                      void (*ldso_fini)(void));

int __uClibc_main(int (*m)(int, char **, char **), int argc, char **argv,
                  void (*init)(void), void (*fini)(void),
                  void (*rtld_fini)(void), void *stack_end)
{
    return __libc_start_main(m, argc, argv, init, fini, rtld_fini);
}

void *mmap(void *addr, size_t len, int prot, int flags, int fd, off_t off)
{
    return (void *)syscall(__NR_mmap2, addr, len, prot, flags, fd,
                           (unsigned long)off >> 12);
}

int ftruncate(int fd, off_t len)
{
    return syscall(__NR_ftruncate, fd, len);
}

/*
 * The older libalog.so (T20/T21) reads uClibc's C-locale ctype tables
 * directly; these are those tables, -128..255, ASCII only.
 */
static __ctype_mask_t ctype_b[384];
static __ctype_touplow_t ctype_lower[384], ctype_upper[384];
const __ctype_mask_t *__ctype_b = ctype_b + 128;
const __ctype_touplow_t *__ctype_tolower = ctype_lower + 128;
const __ctype_touplow_t *__ctype_toupper = ctype_upper + 128;

static void __attribute__((constructor)) ctype_init(void)
{
    int c;

    for (c = -128; c < 256; c++) {
        __ctype_mask_t m = 0;
        int up = c >= 'A' && c <= 'Z', lo = c >= 'a' && c <= 'z';
        int dig = c >= '0' && c <= '9';

        ctype_lower[c + 128] = up ? c + 32 : c;
        ctype_upper[c + 128] = lo ? c - 32 : c;
        if (c < 0 || c > 127) {
            continue;
        }
        m |= up ? _ISupper : 0;
        m |= lo ? _ISlower : 0;
        m |= up || lo ? _ISalpha | _ISalnum : 0;
        m |= dig ? _ISdigit | _ISalnum | _ISxdigit : 0;
        m |= (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F') ? _ISxdigit : 0;
        m |= c == ' ' || (c >= 9 && c <= 13) ? _ISspace : 0;
        m |= c == ' ' || c == 9 ? _ISblank : 0;
        m |= c < 32 || c == 127 ? _IScntrl : 0;
        m |= c >= 32 && c < 127 ? _ISprint : 0;
        m |= c > 32 && c < 127 ? _ISgraph : 0;
        m |= c > 32 && c < 127 && !up && !lo && !dig ? _ISpunct : 0;
        ctype_b[c + 128] = m;
    }
}

void __assert(const char *a, const char *file, unsigned line, const char *fn)
{
    printf("assertion %s failed: %s:%u %s\n", a, file, line, fn);
    abort();
}

int __fgetc_unlocked(FILE *fp)
{
    return fgetc(fp);
}

void __pthread_register_cancel(__pthread_unwind_buf_t *buf)
{
}

void __pthread_unregister_cancel(__pthread_unwind_buf_t *buf)
{
}

void __pthread_unwind_next(__pthread_unwind_buf_t *buf)
{
    pthread_exit((void *)-1);
}
#endif

static int nframes = 10;
static const char *outdir = "/tmp";
static int chn_ok[2];

/*
 * The IMP 1.1.6 IMPFSChnAttr (the rootfs's libimp) ends with a front crop
 * that the 1.1.5 headers lack; libimp reads it past the 1.1.5 struct and
 * hands it to the kernel, which then crops channel 0 to whatever follows
 * in memory.  Each attribute gets a zeroed one.
 */
static struct {
    IMPFSChnAttr a;
    IMPFSChnCrop fcrop;
} attrs[2] = {
    { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .picWidth = 1920, .picHeight = 1080,
    } }, { {
        .pixFmt = PIX_FMT_NV12, .outFrmRateNum = 25, .outFrmRateDen = 1,
        .nrVBs = 2, .type = FS_PHY_CHANNEL,
        .scaler = { .enable = 1, .outwidth = 640, .outheight = 360 },
        .picWidth = 640, .picHeight = 360,
    } },
};
#define attr(n) (attrs[n].a)

/* O_DIRECT wants aligned buffers: copy through one, 64 KiB at a time */
static int save(int ch, int fd, const unsigned char *p, unsigned size)
{
    static unsigned char buf[2][65536] __attribute__((aligned(4096)));
    unsigned char *b = buf[ch];
    unsigned n;

    while (size) {
        n = size < sizeof(buf[0]) ? size : sizeof(buf[0]);
        memcpy(b, p, n);
        if (write(fd, b, (n + 511) & ~511u) < 0) {
            return -1;
        }
        p += n;
        size -= n;
    }
    return 0;
}

#include <imp/imp_ivs.h>
#include <imp/imp_ivs_move.h>
/* IVS move detection on channel 1's 640x360 frames (env IVS=1) */
static IMPIVSInterface *ivs_if;
static void ivs_start(void)
{
    IMP_IVS_MoveParam p;
    memset(&p, 0, sizeof(p));
    p.skipFrameCnt = 1;
    p.frameInfo.width = 640;
    p.frameInfo.height = 360;
    p.roiRectCnt = 1;
    p.sense[0] = 4;
    p.roiRect[0].p1.x = 639;
    p.roiRect[0].p1.y = 359;
    ivs_if = IMP_IVS_CreateMoveInterface(&p);
    if (!ivs_if || (ivs_if->init && ivs_if->init(ivs_if) < 0)) {
        printf("ivs: create/init failed\n");
        ivs_if = NULL;
    } else {
        printf("ivs: move interface up\n");
    }
}
/*
 * Synthetic sequences through fresh move interfaces with 4 quadrant ROIs:
 * static frames must never flag; a block toggling in one quadrant must
 * flag that quadrant only.  The 16x16 case sits near the detection
 * threshold and is missed if the MXU3 narrowing (gtb + gt2w) drops pixels.
 */
static const struct { int side; int mod; unsigned mask; const char *name; int ph; int q; } synth[] = {
    { 0, 1, 0, "static", 0, -1 },
    { 96, 1, 1, "big-q0", 0, 0 }, { 96, 1, 1, "big-q1", 0, 1 },
    { 96, 1, 1, "big-q2", 0, 2 }, { 96, 1, 1, "big-q3", 0, 3 },
    /* near the detection threshold: seen only if no pixel is lost */
    { 16, 1, 1, "s16@3", 3, 0 },
};
static int synth_bad;
static void ivs_synth(IMPFrameInfo *tmpl)
{
    static unsigned char buf[640 * 360 * 3 / 2] __attribute__((aligned(64)));
    int test, i, q, x, y;

    for (test = 0; test < (int)(sizeof(synth) / sizeof(synth[0])); test++) {
        IMP_IVS_MoveParam p;
        IMPIVSInterface *it;
        IMPFrameInfo f = *tmpl;
        int hits[4] = { 0, 0, 0, 0 };

        memset(&p, 0, sizeof(p));
        p.skipFrameCnt = 1;
        p.frameInfo.width = 640;
        p.frameInfo.height = 360;
        p.roiRectCnt = 4;
        for (q = 0; q < 4; q++) {
            p.sense[q] = 4;
            p.roiRect[q].p0.x = (q & 1) * 320;
            p.roiRect[q].p0.y = (q >> 1) * 180;
            p.roiRect[q].p1.x = (q & 1) * 320 + 319;
            p.roiRect[q].p1.y = (q >> 1) * 180 + 179;
        }
        it = IMP_IVS_CreateMoveInterface(&p);
        if (!it || (it->init && it->init(it) < 0)) {
            printf("synth: create failed\n");
            return;
        }
        f.virAddr = (unsigned int)buf;
        f.width = 640;
        f.height = 360;
        for (i = 0; i < 30; i++) {
            IMP_IVS_MoveOutput *r = NULL;
            int side = synth[test].side;

            memset(buf, 60, 640 * 360);
            memset(buf + 640 * 360, 128, 640 * 180);
            if (side && (i & 1)) {
                int q0 = synth[test].q, ox = (q0 & 1) * 320, oy = (q0 >> 1) * 180;
                for (y = 0; y < side; y++)
                    for (x = 0; x < side; x++) {
                        int px = ox + 128 + synth[test].ph + x;
                        buf[(oy + 64 + y) * 640 + px] = 220;
                    }
            }
            if (it->preProcessSync) it->preProcessSync(it, &f);
            if (it->processAsync(it, &f) < 0) { printf("synth: process failed\n"); break; }
            if (it->getResult(it, (void **)&r) >= 0 && r) {
                if (i >= 4)
                    for (q = 0; q < 4; q++) hits[q] += r->retRoi[q] != 0;
                if (it->releaseResult) it->releaseResult(it, r);
            }
        }
        printf("synth %-9s hits %d %d %d %d\n", synth[test].name,
               hits[0], hits[1], hits[2], hits[3]);
        for (q = 0; q < 4; q++) {
            /* the target's quadrant must see motion, the others never */
            if ((q == synth[test].q) != (hits[q] > 0)) {
                synth_bad = 1;
            }
        }
        if (it->exit) it->exit(it);
        IMP_IVS_DestroyMoveInterface(it);
    }
    printf("ivs-synth: %s\n", synth_bad ? "FAIL" : "PASS");
}

static void ivs_frame(IMPFrameInfo *f, int i)
{
    IMP_IVS_MoveOutput *r = NULL;
    if (!ivs_if) return;
    if (ivs_if->preProcessSync) ivs_if->preProcessSync(ivs_if, f);
    if (ivs_if->processAsync && ivs_if->processAsync(ivs_if, f) < 0) {
        printf("ivs: processAsync failed\n"); return;
    }
    if (ivs_if->getResult && ivs_if->getResult(ivs_if, (void **)&r) >= 0 && r) {
        printf("ivs frame %d: retRoi %d\n", i, r->retRoi[0]);
        if (ivs_if->releaseResult) ivs_if->releaseResult(ivs_if, r);
    }
}

static void *grab(void *arg)
{
    int ch = (int)(long)arg, i;
    IMPFrameInfo *f;
    char path[128];

    if (IMP_FrameSource_SetFrameDepth(ch, attr(ch).nrVBs * 2) < 0) {
        printf("ch%d: SetFrameDepth failed\n", ch);
        return NULL;
    }
    for (i = 0; i < nframes; i++) {
        unsigned long long sum = 0;
        unsigned j;

        if (IMP_FrameSource_GetFrame(ch, &f) < 0) {
            printf("ch%d: GetFrame %d failed\n", ch, i);
            return NULL;
        }
        for (j = 0; j < f->width * f->height; j += 61) {
            sum += ((unsigned char *)f->virAddr)[j];
        }
        if (ch == 1 && getenv("IVS")) {
            if (i == 0 && getenv("IVS_SYNTH")) {
                ivs_synth(f);
            }
            ivs_frame(f, i);
        }
        printf("ch%d frame %d: seq %u %ux%u fmt 0x%x size %u phy 0x%08x "
               "ts %llu ysum %llu\n", ch, i, f->index, f->width, f->height,
               f->pixfmt, f->size, f->phyAddr,
               (unsigned long long)f->timeStamp, sum);
        if (i == 2 || i == nframes - 1) {
            int fd;

            if (!strncmp(outdir, "/dev/", 5)) {
                unsigned long long pos = (2 * ch + (i != 2)) << 23, res;

                snprintf(path, sizeof(path), "%s", outdir);
                /* past the page cache, which has no room for a frame */
                fd = open(path, O_WRONLY | O_DIRECT);
                /* _llseek: no off_t in sight, whichever libc this is */
                if (fd >= 0 && syscall(__NR__llseek, fd,
                                       (unsigned long)(pos >> 32),
                                       (unsigned long)pos, &res,
                                       SEEK_SET) < 0) {
                    close(fd);
                    fd = -1;
                }
            } else {
                snprintf(path, sizeof(path), "%s/ch%d-%d.nv12", outdir, ch,
                         i);
                fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            }
            if (fd < 0 || save(ch, fd, (void *)f->virAddr, f->size) < 0) {
                printf("ch%d: cannot write %s\n", ch, path);
            }
            if (fd >= 0) {
                close(fd);
            }
        }
        IMP_FrameSource_ReleaseFrame(ch, f);
    }
    IMP_FrameSource_SetFrameDepth(ch, 0);
    chn_ok[ch] = 1;
    return NULL;
}

int main(int argc, char **argv)
{
    IMPSensorInfo sensor;
    const char *name = "gc2053";
    pthread_t tid[2];
    int ch;

    if (argc > 1) {
        nframes = atoi(argv[1]);
    }
    if (argc > 2) {
        outdir = argv[2];
    }
    if (argc > 3) {
        name = argv[3];
    }
    memset(&sensor, 0, sizeof(sensor));
    strncpy(sensor.name, name, sizeof(sensor.name) - 1);
    sensor.cbus_type = TX_SENSOR_CONTROL_INTERFACE_I2C;
    strncpy(sensor.i2c.type, name, sizeof(sensor.i2c.type) - 1);
    sensor.i2c.addr = argc > 4 ? strtol(argv[4], NULL, 0) : 0x37;
    if (argc > 6) {
        attr(0).picWidth = atoi(argv[5]);
        attr(0).picHeight = atoi(argv[6]);
    }

#ifdef SNAP_T40
    /* T40's IMP takes the sensor's VI number, adapter and interface */
    sensor.i2c.i2c_adapter_id = 1;
    sensor.rst_gpio = -1;
    sensor.pwdn_gpio = -1;
    sensor.power_gpio = -1;
    sensor.video_interface = IMPISP_SENSOR_VI_MIPI_CSI0;
    sensor.mclk = IMPISP_SENSOR_MCLK1;
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(IMPVI_MAIN, &sensor) < 0 ||
        IMP_ISP_EnableSensor(IMPVI_MAIN, &sensor) < 0) {
#else
    if (IMP_ISP_Open() < 0 || IMP_ISP_AddSensor(&sensor) < 0 ||
        IMP_ISP_EnableSensor() < 0) {
#endif
        printf("ISP / sensor setup failed\n");
        return 1;
    }
    if (IMP_System_Init() < 0) {
        printf("IMP_System_Init failed\n");
        return 1;
    }
    if (IMP_ISP_EnableTuning() < 0) {
        printf("IMP_ISP_EnableTuning failed\n");
        return 1;
    }
#ifndef SNAP_T40
    IMP_ISP_Tuning_SetISPRunningMode(IMPISP_RUNNING_MODE_DAY);
#endif
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_CreateChn(ch, &attr(ch)) < 0 ||
            IMP_FrameSource_SetChnAttr(ch, &attr(ch)) < 0) {
            printf("ch%d: CreateChn failed\n", ch);
            return 1;
        }
    }
    for (ch = 0; ch < 2; ch++) {
        if (IMP_FrameSource_EnableChn(ch) < 0) {
            printf("ch%d: EnableChn failed\n", ch);
            return 1;
        }
    }
    printf("streaming\n");
    if (getenv("IVS")) {
        ivs_start();
    }
    for (ch = 0; ch < 2; ch++) {
        pthread_create(&tid[ch], NULL, grab, (void *)(long)ch);
    }
    for (ch = 0; ch < 2; ch++) {
        pthread_join(tid[ch], NULL);
    }
    /*
     * SNAP_DEVMEM=<physical address>: copy the saved files into 8 MiB
     * slots of memory there (rmem's top half; libimp's buffers are at its
     * bottom), for a host without an SD card to read with pmemsave.
     * /dev/mem's write() refuses memory past mem=, its mmap() does not.
     */
    if (getenv("SNAP_DEVMEM")) {
        unsigned long base = strtoul(getenv("SNAP_DEVMEM"), NULL, 0);
        int mfd = open("/dev/mem", O_RDWR | O_SYNC), slot;

        for (slot = 0; mfd >= 0 && slot < 4; slot++) {
            char path[128];
            unsigned char *dst;
            int fd, n;

            snprintf(path, sizeof(path), "%s/ch%d-%d.nv12", outdir, slot / 2,
                     slot & 1 ? nframes - 1 : 2);
            fd = open(path, O_RDONLY);
            dst = mmap(NULL, 8 << 20, PROT_READ | PROT_WRITE, MAP_SHARED, mfd,
                       base + (slot << 23));
            if (fd < 0 || dst == MAP_FAILED) {
                printf("devmem slot %d: cannot copy %s\n", slot, path);
            } else {
                unsigned off = 0;

                while ((n = read(fd, dst + off, 65536)) > 0) {
                    off += n;
                }
                munmap(dst, 8 << 20);
            }
            if (fd >= 0) {
                close(fd);
            }
        }
        if (mfd >= 0) {
            close(mfd);
        }
    }
    /* the verdict before the teardown, which T40's IMP can hang in */
    printf("isp-snap: frames %s\n", chn_ok[0] && chn_ok[1] ? "PASS" : "FAIL");
    fflush(stdout);
    close(open("/tmp/isp-snap.done", O_WRONLY | O_CREAT, 0644));
    for (ch = 0; ch < 2; ch++) {
        IMP_FrameSource_DisableChn(ch);
    }
    for (ch = 0; ch < 2; ch++) {
        IMP_FrameSource_DestroyChn(ch);
    }
    IMP_ISP_DisableTuning();
    IMP_System_Exit();
#ifdef SNAP_T40
    IMP_ISP_DisableSensor(IMPVI_MAIN);
    IMP_ISP_DelSensor(IMPVI_MAIN, &sensor);
#else
    IMP_ISP_DisableSensor();
    IMP_ISP_DelSensor(&sensor);
#endif
    IMP_ISP_Close();
    printf("isp-snap: %s\n", chn_ok[0] && chn_ok[1] ? "PASS" : "FAIL");
    return chn_ok[0] && chn_ok[1] ? 0 : 1;
}
IVSEOF
    cat > "$d/adapt.py" <<'PYEOF'
import sys
s = open(sys.argv[1]).read()
rep = [
    ('HERE=$(cd "$(dirname "$0")" && pwd)', 'HERE=' + sys.argv[3]),
    ('SRC=$HERE/ingenic-isp-snap.c', 'SRC=${SRC:-$HERE/ingenic-isp-snap.c}'),
    ('touch /tmp/closesimd\n', 'export IVS=1 IVS_SYNTH=1\n'),
    ('    "$QEMU" -M $SOC', '    QEMU_MXU3_STATS=all "$QEMU" -M $SOC'),
    ('$MEM" &', '$MEM" 2> "$OUT/qemu.err" &'),
]
for a, b in rep:
    if a not in s:
        sys.exit("cannot adapt test-ingenic-isp.sh (missing: %r)" % a)
    s = s.replace(a, b, 1)
open(sys.argv[2], 'w').write(s)
PYEOF
    if ! python3 "$d/adapt.py" "$isp" "$d/test-ivs.sh" "$ROOT/qemu-boot"; then
        echo "  ivs: FAIL (could not adapt the ISP test)"; return 1
    fi
    echo "=== t40 ivs : libimp IVS move detection on emulated frames ==="
    SRC="$d/ivs-snap.c" bash "$d/test-ivs.sh" --soc t40 --fw "$fw" \
        --qemu "$QEMU" --frames 12 --out "$out" > "$d/isp.log" 2>&1
    grep -a -E "^synth |ivs-synth:" "$out/console.log" | sed 's/^/  /'
    frames=$(grep -ac "^ivs frame" "$out/console.log")
    approx=$(grep -c "^mxu3-approx" "$out/qemu.err")
    grep "^mxu3-" "$out/qemu.err" | sort -k3 -nr | head -8 | sed 's/^/  /'
    if grep -aq "ivs-synth: PASS" "$out/console.log" && [ "$frames" -gt 0 ] && \
       [ "$approx" = 0 ]; then
        echo "  ok   ivs ($frames camera frames, no approximated MXU3 op executed)"
        return 0
    fi
    echo "  ivs: FAIL (frames=$frames approximated=$approx)"
    tail -5 "$d/isp.log" | sed 's/^/  | /'
    return 1
}

rc=0
if [ -r "$T40_KERNEL" ]; then
    run_one t40 "$T40_KERNEL" || rc=1
else
    echo "SKIP t40: kernel $T40_KERNEL not found"
fi
if [ -z "$T41_KERNEL" ] && [ -r "$T41_IMAGE" ]; then
    # the SDK image's own kernel: the uImage at 0x40000
    python3 - "$T41_IMAGE" "$WORK/uImage.t41" <<'PY'
import struct, sys
d = open(sys.argv[1], 'rb').read()
if d[0x40000:0x40004] == b'\x27\x05\x19\x56':
    n = struct.unpack('>I', d[0x40000 + 12:0x40000 + 16])[0]
    open(sys.argv[2], 'wb').write(d[0x40000:0x40000 + 64 + n])
PY
    [ -s "$WORK/uImage.t41" ] && T41_KERNEL="$WORK/uImage.t41"
fi
if [ -n "$T41_KERNEL" ] && [ -r "$T41_KERNEL" ] && [ -n "$T41_ROOT" ]; then
    run_one t41 "$T41_KERNEL" || rc=1
elif [ -z "$T41_ROOT" ]; then
    echo "SKIP t41: no MXU3.1 build or no mksquashfs"; sed 's/^/  /' "$WORK/cc31.log" 2>/dev/null
else
    echo "SKIP t41: no T41 kernel (\$T41_KERNEL, or the SDK image \$T41_IMAGE;"
    echo "      not public)"
fi
if [ -n "$IVS" ]; then
    run_ivs || rc=1
fi
exit $rc
