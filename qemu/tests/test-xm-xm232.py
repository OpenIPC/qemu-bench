#!/usr/bin/env python3
"""XM232's media blocks under qtest, played the way its drivers do.

No guest: the script writes the registers vi_mu.ko, isp.ko, adc.ko and
Sofia's VTA runtime write, and checks what xm-vimu, xm-vta, xm-gpio's ADC
and the machine's DDR alias do with them:

  - the DDR seen at 0x90000000 as well (vb.ko's media addresses);
  - a VI store channel set up and taken at vi_sel's "go": the next frame
    draws the test picture into its buffers and sets its done word, and
    vi_sel's status reads the raw events through their enable mask; the
    acknowledge clears done;
  - the BPU's zone statistics: 512 zones of R / G / B sums, a quarter of
    each zone's pixels at a grey level, in the buffer the block names;
  - adc.ko's calibration inputs read a ramp, a key channel reads released
    inside it (Sofia's reset key is ADC channel 2);
  - the VTA's colour converter: NV12 to 3-byte pixels by the programmed
    coefficients, done at +0x8F4 (written 1 to clear);
  - a VTA network run: +0x008 set and +0x218 idle (0x30) a millisecond on;
  - dma.ko's copy engine: two blocks as its ioctl and interrupt handler
    run them, the data copied, the done status raised and cleared.

    python3 qemu/tests/test-xm-xm232.py [--qemu PATH]
"""
import argparse, os, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest

VI = 0x30000000
SEL_GO, SEL_RAW, SEL_EN, SEL_ST = 0x10014, 0x10020, 0x10024, 0x10028
STORE0 = 0x73000
ST_EN, ST_Y, ST_YS, ST_YSIZE, ST_C, ST_CS, ST_DONE, ST_ACK = (
    0x00, 0x08, 0x18, 0x20, 0x24, 0x34, 0x90, 0x94)
AWB = 0x4B000
VTA = 0x22000000
GPIO = 0x10020000
DMAC = 0x20020000
BUF = 0x82000000          # in the MMZ, as vb.ko hands buffers out
FPS = 12                  # xm-vimu's default, per sensor; two sensors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    q = QTest(a.qemu, 'xm232')
    try:
        # the media alias
        q.write(BUF, b'\x12\x34\x56\x78')
        check(q.read(BUF + 0x10000000, 4) == b'\x12\x34\x56\x78',
              'the DDR at 0x80000000 reads back at 0x90000000')

        # a store channel, 640 x 360 NV12 at BUF, its addresses as vi_mu.ko
        # writes them: (address + 0x80000000) >> 3, of the media alias
        w, h = 640, 360
        y_pa, c_pa = BUF + 0x10000000, BUF + 0x10000000 + w * h
        q.write(BUF, b'\0' * (w * 4))
        for off, v in ((ST_Y, ((y_pa + 0x80000000) & 0xffffffff) >> 3),
                       (ST_C, ((c_pa + 0x80000000) & 0xffffffff) >> 3),
                       (ST_YS, w >> 3), (ST_CS, w >> 3),
                       (ST_YSIZE, w * h >> 3), (ST_EN, 3)):
            q.writel(VI + STORE0 + off, v)
        q.writel(VI + SEL_EN, 0x3554515)
        q.writel(VI + SEL_GO, 1)
        # the AWB statistics block: a 32 x 16 grid of 58 x 66 zones
        for i in range(33):
            q.writel(VI + AWB + 4 * i, 4 + 58 * i)
        for i in range(17):
            q.writel(VI + AWB + 0xC0 + 4 * i, 66 * i)
        q.writel(VI + AWB + 0x110, 1)
        stats = BUF + 0x100000
        q.writel(VI + AWB + 0x118, (stats - 0x80000000) >> 3)
        q.write(stats, b'\0' * 16)
        # a frame period and a half: a frame end on the way
        q.cmd('clock_step %d' % int(1.5e9 / FPS))
        done = q.readl(VI + STORE0 + ST_DONE)
        check(done == 1, 'the store channel\'s frame done (0x%x)' % done)
        st = q.readl(VI + SEL_ST)
        check(st & 1 and st == q.readl(VI + SEL_RAW) & 0x3554515,
              'vi_sel status is the raw events through the mask (0x%x)' % st)
        row = q.read(BUF + 100 * w, w)
        check(all(20 <= p <= 80 for p in row[:64]),
              'the picture\'s navy background in the luma (%d)' % row[0])
        lum = q.read(BUF + 400 * w // 3 * 1, w * 4)
        check(max(lum) > 200, 'the logo\'s white in the picture')
        q.writel(VI + STORE0 + ST_ACK, 1)
        check(q.readl(VI + STORE0 + ST_DONE) == 0, 'the acknowledge clears done')
        z = q.read(stats, 16)
        r, g, b = (int.from_bytes(z[i:i + 4], 'little') for i in (0, 4, 8))
        n = 58 * 66 // 4
        check(r * 4 // n == 1024 and g * 2 // n == 1024 and b == r,
              'AWB zone 0: grey, a quarter of its %d pixels (%d %d %d)'
              % (58 * 66, r, g, b))

        # adc.ko: calibration inputs 5..14 and key channels
        def adc(inp):
            q.writel(GPIO + 0xCC0, inp << 4 | 0x40db03)
            return q.readl(GPIO + 0xCC4)
        ramp = [adc(i) for i in range(5, 15)]
        check(ramp == sorted(ramp) and len(set(ramp)) == 10,
              'adc.ko\'s calibration inputs read a ramp %s' % ramp)
        k = adc(3)
        check(ramp[0] < k < ramp[-1] and k > 0x3f,
              'the reset key (channel 2) reads released, inside the ramp (%d)' % k)

        # the VTA's colour converter: 4 x 2 NV12 to 3-byte pixels
        yv = bytes([16, 128, 235, 81, 16, 128, 235, 81])
        cv = bytes([128, 128, 90, 240])
        q.write(BUF + 0x200000, yv)
        q.write(BUF + 0x200000 + 8, cv)
        regs = {0x808: 4, 0x80C: 2, 0x810: BUF + 0x200000,
                0x814: BUF + 0x200008, 0x818: 4, 0x81C: BUF + 0x300000,
                0x820: 12,
                0x82C: 0x1010, 0x830: 0x1c76, 0x834: 0xfff1c4c5, 0x838: 13,
                0x83C: 0x1010, 0x840: 0xfffff487, 0x844: 0xfffffa78,
                0x848: 0x00088063, 0x84C: 13,
                0x850: 0x1010, 0x854: 0x1685, 0x858: 0xfff4bd7d, 0x85C: 13,
                0x860: 0, 0x864: 0x7f, 0x868: 0, 0x86C: 0x7f, 0x870: 0,
                0x874: 0x7f}
        for off, v in regs.items():
            q.writel(VTA + off, v)
        q.writel(VTA + 0x8F0, 1)
        q.writel(VTA + 0x800, 1)
        check(q.readl(VTA + 0x8F4) == 1, 'the converter is done')
        out = q.read(BUF + 0x300000, 12)

        def ref(yy, cb, cr):
            def s32(v): return v - (1 << 32) if v & 0x80000000 else v
            c = lambda v: min(max(v, 0), 0x7f)
            return [c((0x1010 * yy + 0x1c76 * cb + s32(0xfff1c4c5)) >> 13),
                    c((0x1010 * yy + s32(0xfffff487) * cr + s32(0xfffffa78) * cb
                       + 0x88063) >> 13),
                    c((0x1010 * yy + 0x1685 * cr + s32(0xfff4bd7d)) >> 13)]
        want = sum((ref(yv[i], cv[(i & ~1)], cv[(i & ~1) + 1]) for i in range(4)), [])
        check(list(out) == want, 'the converted pixels %s are %s' % (list(out), want))
        q.writel(VTA + 0x8F4, 1)
        check(q.readl(VTA + 0x8F4) == 0, 'writing 1 clears done')

        # a network run
        q.writel(VTA + 0x018, 0x2e7e)
        q.writel(VTA + 0x020, BUF + 0x400000)
        q.writel(VTA + 0x000, 1)
        busy = q.readl(VTA + 0x218)
        q.cmd('clock_step 2000000')
        check(busy != 0x30 and q.readl(VTA + 0x218) == 0x30,
              'a run is busy, then idle (0x30) a millisecond on')
        check(q.readl(VTA + 0x008) == 1, 'and its done bit, the interrupt, set')
        q.writel(VTA + 0x008, 0)
        check(q.readl(VTA + 0x008) == 0, 'XmVtaMte.ko\'s 0 clears it')

        # dma.ko: 32 KiB in two 16 KiB blocks, source in the media view
        src, dst, n = BUF + 0x500000, BUF + 0x600000, 0x8000
        pattern = bytes((i * 7 + (i >> 8)) & 0xff for i in range(n))
        q.write(src, pattern)
        q.write(dst, b'\0' * n)
        q.writel(DMAC + 0x010, 0)
        for blk in range(2):
            o = blk * 0x4000
            q.writel(DMAC + 0x100, src + 0x10000000 + o)
            q.writel(DMAC + 0x108, dst + o)
            q.writel(DMAC + 0x110, 0x4000 // 8 - 1)
            q.writel(DMAC + 0x180, 2)
            q.writel(DMAC + 0x190, 2)
            q.writel(DMAC + 0x010, 3)
            q.writel(DMAC + 0x018, q.readl(DMAC + 0x018) | 0x101)
            q.cmd('clock_step 100000')
            st = q.readl(DMAC + 0x188)
            check(st == 2, 'DMA block %d done (status 0x%x)' % (blk, st))
            check(q.readl(DMAC + 0x018) & 1 == 0, 'and its channel off again')
            q.writel(DMAC + 0x198, st)
            check(q.readl(DMAC + 0x188) == 0, 'the handler\'s write clears it')
        check(q.read(dst, n) == pattern, 'the 32 KiB copied')
    finally:
        q.close()
    print('=== %s: xm232 ===' % ('PASS' if not fails else 'FAIL'))
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
