#!/usr/bin/env python3
"""Drive hisi-ive-npu, the V5 (Hi3516CV610) IVE, the way mod_ive.o does.

On V5 an IVE op is a 576-byte task in a ring in DDR that the SVP NPU's
hardware task scheduler runs.  Like ive_init and ive_post_npu_task, this
acks the AI core's soft reset, allocates a stream, points it at a ring
whose tasks link to each other (+6 the id, +8 the next task), fills tasks
as ive_fill_npu_{dma,sad,ccl,add,sub,hist}_task do, pushes them, and waits for the
finish: interrupt status bit 0 on IVE's group, the stream's bit, the last
task's id.  The outputs are checked against the same ops done here.

    python3 qemu/tests/test-hisi-ive-npu.py [--qemu PATH] [--machine M]
"""
import argparse
import os
import struct
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
QTest = import_module('test-xm-h1-jpeg').QTest

NPU = 0x14000000
G1 = NPU + 0x82b0                       # IVE's interrupt group
SQ = NPU + 0x8500                       # stream 0
RING = 0x42000000
NSLOT = 8
TASK = 576
IMG = 0x42100000


def slot(i):
    return RING + i * TASK


def task(i, op, w, h, fields):
    """A ring slot as ive_init links it, then the op's fields."""
    t = bytearray(TASK)
    struct.pack_into('<IHHI', t, 0, 0x4013, 0, i, slot((i + 1) % NSLOT))
    struct.pack_into('<III', t, 288, op, w, h)
    for off, fmt, v in fields:
        struct.pack_into('<' + fmt, t, off, v)
    return bytes(t)


def pattern(w, h, seed):
    return bytes((x * 7 + y * 13 + seed * 31 + (x * y) % 11) & 0xff
                 for y in range(h) for x in range(w))


def sad_ref(a, b, w, h, blk, thr, lo, hi):
    bw, bh = w // blk, h // blk
    sad, out = [], []
    for by in range(bh):
        for bx in range(bw):
            s = sum(abs(a[(by * blk + dy) * w + bx * blk + dx] -
                        b[(by * blk + dy) * w + bx * blk + dx])
                    for dy in range(blk) for dx in range(blk))
            sad.append(min(s, 0xffff))
            out.append(hi if s > thr else lo)
    return sad, out


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--qemu', default=os.path.join(
        here, '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='hi3516cv610')
    args = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('PASS ' if ok else 'FAIL ') + what)
        fails += not ok

    q = QTest(args.qemu, args.machine)
    try:
        # the AI core's soft reset is acked; a stream is allocated
        q.writel(NPU + 0x8108, 1)
        check(q.readl(NPU + 0x8110) & 1 == 1, 'AI core soft reset acked')
        q.writel(NPU + 0x8108, 0)
        sq = q.readl(NPU + 0x8280)
        check(sq == 0, 'stream 0 allocated (%#x)' % sq)
        check(q.readl(NPU + 0x8280) == 1, 'the next allocation is stream 1')

        # drv_ive_npu_mask_irq, drv_ive_npu_set_stream_base_addr
        q.writel(G1, 0x332)
        q.writel(SQ + 0x00, 1 << 5)                # IVE's group
        q.writel(SQ + 0x08, RING | 1)

        W, H = 64, 32
        A, B = pattern(W, H, 1), pattern(W, H, 2)
        q.write(IMG, A)
        q.write(IMG + 0x10000, B)

        # task 0: DMA, direct copy of A
        DMA_DST = IMG + 0x20000
        t0 = task(0, 0, W, H, [(80, 'Q', IMG), (104, 'I', W),
                               (96, 'Q', DMA_DST), (112, 'I', 80),
                               (308, 'I', 0)])
        # task 1: SAD 4x4 of A and B, 16-bit SAD and threshold, the
        # threshold at the median so that both sides show
        SAD16, THR = IMG + 0x30000, IMG + 0x34000
        bw, bh = W // 4, H // 4
        T = sorted(sad_ref(A, B, W, H, 4, 0, 0, 0)[0])[bw * bh // 2]
        t1 = task(1, 27, W, H, [(64, 'Q', IMG), (72, 'Q', IMG + 0x10000),
                                (104, 'I', W), (108, 'I', W),
                                (88, 'Q', SAD16), (116, 'I', 2 * bw),
                                (96, 'Q', THR), (120, 'I', bw),
                                (300, 'I', bw), (304, 'I', bh),
                                (308, 'I', 0), (312, 'I', 0),
                                (316, 'I', T), (320, 'I', 7),
                                (324, 'I', 200)])
        # task 2: CCL, 8-connected, on a binary image with three blobs
        CW, CH = 32, 16
        img = bytearray(CW * CH)
        for (x0, y0, x1, y1) in ((1, 1, 4, 3), (10, 2, 12, 9), (20, 12, 21, 12)):
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    img[y * CW + x] = 255
        CCL_IMG, BLOB = IMG + 0x40000, IMG + 0x44000
        q.write(CCL_IMG, bytes(img))
        q.write(BLOB, b'\xee' * 3052)
        t2 = task(2, 21, CW, CH, [(64, 'Q', CCL_IMG), (88, 'I', CW),
                                  (80, 'Q', BLOB), (300, 'I', 1),
                                  (304, 'I', 3), (308, 'I', 1)])
        # task 3: ADD, half of each
        ADD_DST = IMG + 0x48000
        t3 = task(3, 18, W, H, [(80, 'Q', IMG), (88, 'Q', IMG + 0x10000),
                                (104, 'Q', ADD_DST), (112, 'I', W),
                                (116, 'I', W), (124, 'I', W),
                                (300, 'I', 0x8000), (304, 'I', 0x8000)])
        # task 4: SUB, |A - B|; task 5: HIST of A
        SUB_DST, HIST = IMG + 0x50000, IMG + 0x54000
        t4 = task(4, 9, W, H, [(80, 'Q', IMG), (88, 'Q', IMG + 0x10000),
                               (104, 'Q', SUB_DST), (112, 'I', W),
                               (116, 'I', W), (124, 'I', W), (300, 'I', 0)])
        t5 = task(5, 12, W, H, [(80, 'Q', IMG), (104, 'I', W),
                                (96, 'Q', HIST)])
        for i, t in enumerate((t0, t1, t2, t3, t4, t5)):
            q.write(slot(i), t)

        check(q.readl(G1 + 4) & 1 == 0, 'no finish before the push')
        q.writel(SQ + 0x10, 6 << 8)                # hal_ive_npu_push_task
        check(q.readl(G1 + 4) & 1 == 0, 'the push does not finish at once')
        q.cmd('clock_step 2000000')
        st = q.readl(G1 + 4)
        check(st & 1 == 1, 'finish on IVE\'s group (%#x)' % st)
        check(q.readl(G1 + 0x10) & 1 == 1, 'stream 0 finished')
        check(q.readl(NPU + 0x8204) & 1 == 0, 'not on the svp_npu group')
        last = q.readl(SQ + 0x20) & 0xffff
        check(last == 5, 'last finished task id %d' % last)

        out = q.read(DMA_DST, 80 * (H - 1) + W)
        check(all(out[y * 80:y * 80 + W] == A[y * W:(y + 1) * W]
                  for y in range(H)), 'DMA: direct copy, destination stride')

        sad, thr = sad_ref(A, B, W, H, 4, T, 7, 200)
        got = struct.unpack('<%dH' % (bw * bh), q.read(SAD16, 2 * bw * bh))
        check(list(got) == sad, 'SAD: 16-bit 4x4 sums')
        check(list(q.read(THR, bw * bh)) == thr,
              'SAD: threshold image (%d of %d above %d)' %
              (thr.count(200), len(thr), T))
        check(0 < thr.count(200) < len(thr), 'SAD: blocks on both sides')

        blob = q.read(BLOB, 4 + 3 * 12)
        info = struct.unpack_from('<I', blob)[0]
        n, thr_out = info >> 24, info & 0x7fffff
        # the 2x1 blob (area 2) is below the area threshold 3
        rgns = [struct.unpack_from('<IHHHH', blob, 4 + 12 * i) for i in range(n)]
        check(n == 2 and thr_out == 3 and not info & (1 << 23),
              'CCL: two regions at threshold 3 (info %#x)' % info)
        check(rgns == [(12, 1, 4, 1, 3), (24, 10, 12, 2, 9)],
              'CCL: area, left, right, top, bottom %s' % rgns)
        lab = q.read(CCL_IMG, CW * CH)
        check(lab[2 * CW + 2] == 1 and lab[5 * CW + 11] == 2 and
              lab[12 * CW + 20] == 0, 'CCL: labels written in place')

        add = q.read(ADD_DST, W * H)
        check(list(add) == [(a * 0x8000 + b * 0x8000) >> 16
                            for a, b in zip(A, B)], 'ADD: x = y = 0.5')

        sub = q.read(SUB_DST, W * H)
        check(list(sub) == [abs(a - b) for a, b in zip(A, B)], 'SUB: |A - B|')
        hist = struct.unpack('<256I', q.read(HIST, 1024))
        check(list(hist) == [A.count(bytes([v])) for v in range(256)],
              'HIST: 256 bins of A')

        # the driver clears the stream's finish (hal_ive_npu_clear_stream_int)
        q.writel(SQ + 0x14, 31)
        check(q.readl(G1 + 4) & 1 == 0 and q.readl(G1 + 0x10) == 0,
              'finish cleared')

        # the ring goes on from slot 4; the masked finish still reports
        q.writel(G1, 0x333)
        q.write(slot(6), task(6, 0, W, H, [(80, 'Q', IMG + 0x10000),
                                           (104, 'I', W),
                                           (96, 'Q', DMA_DST), (112, 'I', W),
                                           (308, 'I', 0)]))
        q.writel(SQ + 0x10, 1 << 8)
        q.cmd('clock_step 2000000')
        check(q.readl(SQ + 0x20) & 0xffff == 6, 'the next push runs slot 6')
        check(q.readl(G1 + 4) & 1 == 1, 'a masked finish still shows in status')
        check(q.read(DMA_DST, W * H) == B, 'DMA: the second copy')
        q.writel(SQ + 0x14, 31)

        # DMA interval copy: 2 of every 4 bytes of every 2nd row; then
        # fields that would wrap the row (seg past it) do nothing
        IV = IMG + 0x58000
        q.write(IV, b'\xaa' * 0x800)
        t = task(7, 0, W, H, [(80, 'Q', IMG), (104, 'I', W), (96, 'Q', IV),
                              (112, 'I', W // 2), (308, 'I', 1),
                              (336, 'I', 4), (340, 'I', 2), (344, 'I', 2)])
        bad = task(0, 0, 4096, 2, [(80, 'Q', IMG), (104, 'I', 4096),
                                   (96, 'Q', IV + 0x1000), (112, 'I', 4096),
                                   (308, 'I', 1), (336, 'I', 0xfffff830),
                                   (340, 'I', 3000), (344, 'I', 1)])
        q.write(slot(7), t)
        q.write(slot(0), bad)
        q.write(IV + 0x1000, b'\xbb' * 16)
        q.writel(SQ + 0x10, 2 << 8)
        q.cmd('clock_step 2000000')
        want = b''.join(bytes(A[y * W + x] for x in range(W) if x % 4 < 2)
                        for y in range(0, H, 2))
        got = b''.join(q.read(IV + r * (W // 2), W // 2) for r in range(H // 2))
        check(got == want, 'DMA: interval copy (2 of 4 bytes, every 2nd row)')
        check(q.read(IV + 0x1000, 16) == b'\xbb' * 16,
              'DMA: a segment past the row is refused')
        q.writel(SQ + 0x14, 31)

        # CCL with more regions than the blob holds: 300 dots of areas
        # 1..300 (on every other row, apart); the threshold goes from 1 by 7 straight to
        # 50, the first step value leaving at most 254 (areas 50..300 are
        # 251).  A huge step takes it to its 23-bit limit, no regions.
        BW, BH = 304, 600
        BIG, BIGBLOB = IMG + 0x100000, IMG + 0x140000
        img = bytearray(BW * BH)
        for a in range(1, 301):
            img[2 * (a - 1) * BW:2 * (a - 1) * BW + a] = b'\xff' * a
        for i, (step, thr, n) in enumerate(((7, 50, 251),
                                            (0xffff, 0x10000, 0))):
            q.write(BIG, bytes(img))
            q.write(slot(1 + i), task(1 + i, 21, BW, BH,
                                      [(64, 'Q', BIG), (88, 'I', BW),
                                       (80, 'Q', BIGBLOB), (300, 'I', 0),
                                       (304, 'I', 1), (308, 'I', step)]))
            q.writel(SQ + 0x10, 1 << 8)
            q.cmd('clock_step 2000000')
            info = struct.unpack('<I', q.read(BIGBLOB, 4))[0]
            check(info & 0x7fffff == thr and info >> 24 == n and
                  not info & (1 << 23),
                  'CCL: step %d -> threshold %d, %d regions (info %#x)' %
                  (step, thr, n, info))
            q.writel(SQ + 0x14, 31)

        # More than a call's worth (512) pushed at once: all of them run
        for i in range(NSLOT):
            q.write(slot(i), task(i, 0, 4, 1, [(80, 'Q', IMG), (104, 'I', 4),
                                              (96, 'Q', IV), (112, 'I', 4),
                                              (308, 'I', 0)]))
        start = q.readl(SQ + 0x20) & 0xffff
        for _ in range(40):
            q.writel(SQ + 0x10, 15 << 8)            # 600 tasks
        q.cmd('clock_step 2000000')
        mid = q.readl(SQ + 0x20) & 0xffff
        q.cmd('clock_step 2000000')
        end = q.readl(SQ + 0x20) & 0xffff
        check(end == (start + 600) % NSLOT,
              '600 tasks: the last id %d (from %d, %d after 512)' %
              (end, start, mid))
        check(q.readl(SQ + 0x18) & 0xfff == 0, 'none left queued')
    finally:
        q.close()
    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
