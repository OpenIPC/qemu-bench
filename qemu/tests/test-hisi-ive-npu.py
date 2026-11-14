#!/usr/bin/env python3
"""Drive hisi-ive-npu, the V5 (Hi3516CV610) IVE, the way mod_ive.o does.

On V5 an IVE op is a 576-byte task in a ring in DDR that the SVP NPU's
hardware task scheduler runs.  Like ive_init and ive_post_npu_task, this
acks the AI core's soft reset, allocates a stream, points it at a ring
whose tasks link to each other (+6 the id, +8 the next task), fills tasks
as the ive_fill_npu_*_task functions do, pushes them, and waits for the
finish: interrupt status bit 0 on IVE's group, the stream's bit, the last
task's id.  The outputs are checked against the same ops done here.

    python3 qemu/tests/test-hisi-ive-npu.py [--qemu PATH] [--machine M]
"""
import argparse
import math
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


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def px(img, w, h, x, y):
    """the pixel at (x, y), edges replicated"""
    return img[clamp(y, 0, h - 1) * w + clamp(x, 0, w - 1)]


def correlate(img, w, h, mask, kw, kh):
    return [sum(mask[j * kw + i] * px(img, w, h, x + i - kw // 2, y + j - kh // 2)
                for j in range(kh) for i in range(kw))
            for y in range(h) for x in range(w)]



def octant(gh, gv):
    """the gradient's octant, as MAG_AND_ANG's angle"""
    import math
    if not gh and not gv:
        return 0
    return int(round(math.degrees(math.atan2(gv, gh)) / 45)) % 8


def resize_ref(src, sw, sh, dw, dh):
    """the engine's walk: a Q27 ratio and start offset, Q12 weights"""
    def axis(sn, dn):
        r = (sn << 27) // dn
        off = (r - (1 << 27)) >> 1
        return [(off + i * r) for i in range(dn)]
    xs, ys = axis(sw, dw), axis(sh, dh)
    out = []
    for py in ys:
        y0, y1 = clamp(py >> 27, 0, sh - 1), clamp((py >> 27) + 1, 0, sh - 1)
        ay = (py & 0x7ffffff) >> 15
        for px_ in xs:
            x0, x1 = clamp(px_ >> 27, 0, sw - 1), clamp((px_ >> 27) + 1, 0, sw - 1)
            ax = (px_ & 0x7ffffff) >> 15
            top = (src[y0 * sw + x0] * ((1 << 12) - ax) +
                   src[y0 * sw + x1] * ax + 16) >> 5
            bot = (src[y1 * sw + x0] * ((1 << 12) - ax) +
                   src[y1 * sw + x1] * ax + 16) >> 5
            out.append((((top * ((1 << 12) - ay)) >> 17) +
                        ((bot * ay) >> 17) + 2) >> 2)
    return out


# B, G, R as (Y, U, V, constant) and the output range, per CSC mode
CSC_TO_RGB = {
    0: (((1024, 1774, 0, -226504), (1024, -344, -715, 136040),
         (1024, 0, 1404, -179187)), 16, 235),
    2: (((1192, 2066, 0, -283062), (1192, -400, -833, 139252),
         (1192, 0, 1634, -227750)), 0, 255),
}
# Y, V, U as (B, G, R, constant)
CSC_TO_YUV = {
    8: ((117, 601, 306, 512), (-85, -438, 523, 131584),
        (523, -347, -176, 131584)),
    10: ((100, 516, 263, 16896), (-73, -377, 450, 131584),
         (450, -298, -152, 131584)),
}


OCT_STEP = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))


def canny_ref(mag, ang, w, h, lo, hi):
    """the engine's thinning and hysteresis: classes and the seed list"""
    def m(x, y):
        return 0 if x < 0 or y < 0 or x >= w or y >= h else mag[y * w + x]
    state = [0] * (w * h)
    edge = [1] * (w * h)
    seeds = []
    for y in range(h):
        for x in range(w):
            i = y * w + x
            dx, dy = OCT_STEP[ang[i] & 7]
            v = mag[i]
            n1, n2 = m(x + dx, y + dy), m(x - dx, y - dy)
            if (ang[i] & 1):
                keep = v > n1 and v > n2
            elif dy < 0 or (dy == 0 and dx < 0):
                keep = v > n1 and v >= n2
            else:
                keep = v >= n1 and v > n2
            if not keep or v <= lo:
                continue
            if v <= hi:
                state[i], edge[i] = 1, 0
                continue
            if (x and state[i - 1] in (2, 3)) or (y and state[i - w] == 2):
                state[i], edge[i] = 3, 0
                continue
            state[i], edge[i] = 2, 2
            seeds.append(x | y << 16)
    return edge, seeds


GMM2_CTRL = dict(life_factor=0xffff, life_threshold=500, freq_init=20000,
                 freq_reduce=0xff00, freq_add=0xef, freq_threshold=12000,
                 var_rate=1, max_var=(16 * 16) << 7, min_var=(8 * 8) << 7,
                 sns=8, num=3)


def gmm2_pixel(g, v, c):
    """one pixel's Gaussians updated as the engine does"""
    sample = v << 7
    lead = max(range(len(g)), key=lambda i: (g[i][2], -i))
    hit = -1
    for i, (mean, var, freq, life) in enumerate(g):
        d = sample - mean
        if freq and d * d <= (var * c['sns'] << 7):
            hit = i
            break
    matched = hit >= 0
    if matched:
        mean, var, freq, life = g[hit]
        sh = 6 + max(c['var_rate'], 1)
        d = sample - mean
        dg = -((-d) >> 7)
        var -= (var - ((dg * dg) << 7)) >> sh
        g[hit] = [mean - ((mean - sample) >> sh),
                  min(max(var, c['min_var']), c['max_var']),
                  min(((freq * c['freq_reduce']) >> 16) + c['freq_add'], 0xffff),
                  min(life + c['life_factor'], 0xffff)]
    for i, (mean, var, freq, life) in enumerate(g):
        if i != hit and freq:
            g[i] = [mean, var, (freq * c['freq_reduce']) >> 16, life]
    if not matched:
        slot = next((i for i in range(len(g) - 1, -1, -1) if not g[i][2]), None)
        if slot is None:
            slot = min(range(len(g)), key=lambda i: g[i][2])
        g[slot] = [sample, c['max_var'], c['freq_init'], 1]
        hit = slot
    for q in g:
        if not q[2]:
            q[3] = 0xffff
    return (255 if g[hit][3] < c['life_threshold'] else 0,
            g[lead][0] >> 7, lead << 1 | int(matched))


def band(v, lo, hi, a, b, c):
    r = a if v <= lo else b if v <= hi else c
    return v if r is None else r


def thresh_ref(src, mode, lo, hi, mn, md, mx):
    lut = [(mn, mx, mx), (None, mx, mx), (mn, None, None), (mn, md, mx),
           (None, md, mx), (mn, md, None), (mn, None, mx), (None, md, None)]
    a, b, c = lut[mode]
    top = lo if mode <= 2 else hi
    return [band(v, lo, top, a, b, c) for v in src]


def rnd_q16(v, ratio):
    """v * ratio / 2^16, rounded to nearest, halves away from zero"""
    v = v * ratio + (0x8000 if v >= 0 else -0x8000)
    return v // 0x10000 if v >= 0 else -((-v) // 0x10000)


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

        # --- the rest of part 2a, one task at a time on a small picture --
        SW, SH = 16, 12

        def run(op, fields, w=SW, h=SH):
            i = (q.readl(SQ + 0x20) + 1) % NSLOT
            q.write(slot(i), task(i, op, w, h, fields))
            q.writel(SQ + 0x10, 1 << 8)
            q.cmd('clock_step 2000000')
            q.writel(SQ + 0x14, 31)

        P, Q, R = IMG + 0x200000, IMG + 0x210000, IMG + 0x220000
        LUT = IMG + 0x230000
        a = bytes((x * 37 + y * 11 + x * y) & 0xff for y in range(SH) for x in range(SW))
        b = bytes((x * 5 + y * 23 + 7) & 0xff for y in range(SH) for x in range(SW))
        n = SW * SH
        q.write(P, a)
        q.write(Q, b)

        two = [(80, 'Q', P), (88, 'Q', Q), (104, 'Q', R), (112, 'I', SW),
               (116, 'I', SW), (124, 'I', SW)]
        for op, name, f in ((8, 'AND', lambda x, y: x & y),
                            (10, 'OR', lambda x, y: x | y),
                            (19, 'XOR', lambda x, y: x ^ y)):
            run(op, two)
            check(list(q.read(R, n)) == [f(x, y) for x, y in zip(a, b)], name)
        run(9, two + [(300, 'I', 1)])
        check(list(q.read(R, n)) == [((x - y) >> 1) & 0xff for x, y in zip(a, b)],
              'SUB: shift mode, (a - b) >> 1 as S8')

        one = [(80, 'Q', P), (104, 'I', SW), (96, 'Q', R), (112, 'I', SW)]
        for mode in range(8):
            run(7, one + [(300, 'I', mode), (304, 'I', 60), (308, 'I', 170),
                          (312, 'I', 1), (316, 'I', 128), (320, 'I', 250)])
            check(list(q.read(R, n)) == thresh_ref(a, mode, 60, 170, 1, 128, 250),
                  'THRESH: mode %d' % mode)

        s16 = [clamp((v - 128) * 300, -32768, 32767) for v in a]
        q.write(LUT, struct.pack('<%dh' % n, *s16))
        s16t = [(80, 'Q', LUT), (104, 'I', 2 * SW), (96, 'Q', R), (112, 'I', SW)]
        for mode in range(4):
            mn, md, mx = (-100, 5, 100) if mode < 2 else (3, 70, 240)
            run(13, s16t + [(300, 'I', mode), (304, 'i', -5000), (308, 'i', 9000),
                            (312, 'i', mn), (316, 'i', md), (320, 'i', mx)])
            want = [band(v, -5000, 9000, mn, None if mode & 1 else md, mx) for v in s16]
            want = [clamp(v, -128, 127) & 0xff if mode < 2 else clamp(v, 0, 255)
                    for v in want]
            check(list(q.read(R, n)) == want, 'THRESH_S16: mode %d' % mode)
        u16 = [v * 200 for v in a]
        q.write(LUT, struct.pack('<%dH' % n, *u16))
        for mode in range(2):
            run(14, s16t + [(300, 'I', mode), (304, 'I', 9000), (308, 'I', 30000),
                            (312, 'I', 2), (316, 'I', 90), (320, 'I', 230)])
            want = [clamp(band(v, 9000, 30000, 2, None if mode else 90, 230), 0, 255)
                    for v in u16]
            check(list(q.read(R, n)) == want, 'THRESH_U16: mode %d' % mode)

        # 16 to 8: num / den as the driver's rounded Q16 ratio
        num, den, bias = 3, 700, -20
        ratio = ((den >> 1) + (num << 16)) // den
        for mode in range(4):
            src = u16 if mode == 3 else s16
            q.write(LUT, struct.pack('<%d%s' % (n, 'H' if mode == 3 else 'h'), *src))
            run(15, s16t + [(300, 'I', mode), (304, 'i', bias), (308, 'I', ratio)])
            r = [rnd_q16(abs(v) if mode == 1 else v, ratio) for v in src]
            want = ([clamp(v, -128, 127) & 0xff for v in r] if mode == 0 else
                    [clamp(v + bias, 0, 255) for v in r] if mode == 2 else
                    [clamp(v, 0, 255) for v in r])
            check(list(q.read(R, n)) == want, '16BIT_TO_8BIT: mode %d' % mode)

        # MAP: a U8 table, then an S16 one
        lut8 = bytes((255 - i) ^ 0x5a for i in range(256))
        q.write(LUT, lut8)
        run(17, [(80, 'Q', P), (88, 'Q', LUT), (104, 'Q', R), (112, 'I', SW),
                 (124, 'I', SW), (300, 'I', 0)])
        check(q.read(R, n) == bytes(lut8[v] for v in a), 'MAP: U8 table')
        lut16 = [i * 129 - 16000 for i in range(256)]
        q.write(LUT, struct.pack('<256h', *lut16))
        run(17, [(80, 'Q', P), (88, 'Q', LUT), (104, 'Q', R), (112, 'I', SW),
                 (124, 'I', 2 * SW), (300, 'I', 1)])
        check(list(struct.unpack('<%dh' % n, q.read(R, 2 * n))) ==
              [lut16[v] for v in a], 'MAP: S16 table')

        # INTEG: sums (u32), squares (u64), both packed in a u64
        isum = [sum(a[j * SW + i] for j in range(y + 1) for i in range(x + 1))
                for y in range(SH) for x in range(SW)]
        isq = [sum(a[j * SW + i] ** 2 for j in range(y + 1) for i in range(x + 1))
               for y in range(SH) for x in range(SW)]
        run(11, [(80, 'Q', P), (104, 'I', SW), (96, 'Q', R), (112, 'I', 4 * SW),
                 (300, 'I', 1)])
        check(list(struct.unpack('<%dI' % n, q.read(R, 4 * n))) == isum,
              'INTEG: sums')
        run(11, [(80, 'Q', P), (104, 'I', SW), (96, 'Q', R), (112, 'I', 8 * SW),
                 (300, 'I', 2)])
        check(list(struct.unpack('<%dQ' % n, q.read(R, 8 * n))) == isq,
              'INTEG: sums of squares')
        run(11, [(80, 'Q', P), (104, 'I', SW), (96, 'Q', R), (112, 'I', 8 * SW),
                 (300, 'I', 0)])
        check(list(struct.unpack('<%dQ' % n, q.read(R, 8 * n))) ==
              [(s1 & 0xfffffff) | s2 << 28 for s1, s2 in zip(isum, isq)],
              'INTEG: combined')

        q.write(R + 24, b'\xcd' * 8)              # the struct's reserved tail
        run(20, [(80, 'Q', P), (88, 'Q', Q), (104, 'Q', R), (112, 'I', SW),
                 (116, 'I', SW)])
        check(struct.unpack('<3Q', q.read(R, 24)) ==
              (sum(x * y for x, y in zip(a, b)), sum(x * x for x in a),
               sum(y * y for y in b)), 'NCC: the three sums')
        check(q.read(R + 24, 8) == b'\xcd' * 8, 'NCC: nothing past them')

        # DILATE / ERODE of a binary picture, a plus-shaped 3x3 mask
        binimg = bytes(255 if v > 150 else 0 for v in a)
        q.write(P + 0x8000, binimg)
        mask = [1 if (abs(k % 5 - 2) + abs(k // 5 - 2)) <= 1 else 0 for k in range(25)]
        for op, name, f in ((5, 'DILATE', max), (6, 'ERODE', min)):
            run(op, [(64, 'Q', P + 0x8000), (96, 'I', SW), (80, 'Q', R),
                     (104, 'I', SW)] + [(300 + k, 'B', mask[k]) for k in range(25)])
            want = [f(px(binimg, SW, SH, x + k % 5 - 2, y + k // 5 - 2)
                      for k in range(25) if mask[k])
                    for y in range(SH) for x in range(SW)]
            check(list(q.read(R, n)) == want, name + ': plus mask')

        # FILTER: a norm past 31 (a negative int) is capped, not a wild shift
        run(1, [(64, 'Q', P), (104, 'I', SW), (88, 'Q', R), (116, 'I', SW),
                (300, 'I', 0), (304, 'I', 0xffffffff), (308, 'I', 3),
                (312, 'I', 3)] + [(332 + k, 'b', 1) for k in range(9)])
        check(list(q.read(R, n)) == [0] * n, 'FILTER: norm 0xffffffff shifts by 31')

        # FILTER: a 3x3 blur, then a 5x5 sharpen, normalised by right shift
        for kw, m, norm in ((3, [1, 2, 1, 2, 4, 2, 1, 2, 1], 4),
                            (5, [0, 0, -1, 0, 0, 0, -1, -2, -1, 0, -1, -2, 20,
                                 -2, -1, 0, -1, -2, -1, 0, 0, 0, -1, 0, 0], 3)):
            run(1, [(64, 'Q', P), (104, 'I', SW), (88, 'Q', R), (116, 'I', SW),
                    (300, 'I', 0), (304, 'I', norm), (308, 'I', kw),
                    (312, 'I', kw)] + [(332 + k, 'b', v) for k, v in enumerate(m)])
            want = [clamp(v >> norm, 0, 255) for v in correlate(a, SW, SH, m, kw, kw)]
            check(list(q.read(R, n)) == want, 'FILTER: %dx%d >> %d' % (kw, kw, norm))

        # SOBEL: the SDK sample's mask, both gradients
        sm = [0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, -2, 0, 2, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0]
        smt = [sm[(k % 5) * 5 + k // 5] for k in range(25)]
        run(3, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (88, 'Q', LUT),
                (104, 'I', 2 * SW), (108, 'I', 2 * SW), (328, 'I', 0)] +
            [(300 + k, 'b', v) for k, v in enumerate(sm)])
        check(list(struct.unpack('<%dh' % n, q.read(R, 2 * n))) ==
              correlate(a, SW, SH, sm, 5, 5), 'SOBEL: horizontal')
        check(list(struct.unpack('<%dh' % n, q.read(LUT, 2 * n))) ==
              correlate(a, SW, SH, smt, 5, 5), 'SOBEL: vertical (the transpose)')

        for mode, name, pick in ((0, 'median', 4), (1, 'max', 8), (2, 'min', 0)):
            run(16, one + [(300, 'I', mode)])
            want = [sorted(px(a, SW, SH, x + k % 3 - 1, y + k // 3 - 1)
                           for k in range(9))[pick]
                    for y in range(SH) for x in range(SW)]
            check(list(q.read(R, n)) == want, 'ORD_STAT: ' + name)

        # --- part 2b: the ops fitted against a Hi3516CV608 board ---------
        # (qemu-boot/ive-v5-capture.c; the references are in
        # qemu/tests/data/ive-v5-cv608-ref.tar.xz)

        # ST_BOX_FLT_AND_EIGEN (op 25): the 3x3 box filter of the S8
        # gradient pair's products, reduced to one U16 per pixel plus the
        # picture's maximum.  The rule is t1 + 16 t1^2 / tr with
        # t1 = det // (16 tr); it matches a CV608 on 92.5% of pixels and
        # within one count on 98.0% (#300), so this pins the model, not
        # the silicon.
        grad = bytes(b for i in range(n)
                     for b in (((i * 13) % 61 - 30) & 0xff,
                               ((i * 7) % 47 - 23) & 0xff))
        q.write(P, grad)
        run(25, [(80, 'Q', P), (112, 'I', 2 * SW), (96, 'Q', R),
                 (120, 'I', 2 * SW), (104, 'Q', R + 2 * n)])
        g = [(v - 256 if v > 127 else v) for v in grad]
        want, mx = [], 0
        for y in range(SH):
            for x in range(SW):
                A = B = C = 0
                for dy in (-1, 0, 1):
                    yy = min(max(y + dy, 0), SH - 1)
                    for dx in (-1, 0, 1):
                        xx = min(max(x + dx, 0), SW - 1)
                        gh = g[2 * (yy * SW + xx)]
                        gv = g[2 * (yy * SW + xx) + 1]
                        A += gh * gh
                        B += gh * gv
                        C += gv * gv
                tr, det, v = A + C, A * C - B * B, 0
                if tr > 0 and det > 0:
                    t1 = det // (16 * tr)
                    v = min(t1 + 16 * t1 * t1 // tr, 0xffff)
                want.append(v)
                mx = max(mx, v)
        got = struct.unpack('<%dH' % n, q.read(R, 2 * n))
        check(list(got) == want, 'st_box_flt_and_eigen: the measure')
        check(struct.unpack('<H', q.read(R + 2 * n, 2))[0] == mx,
              'st_box_flt_and_eigen: the picture maximum')
        q.write(P, a)                   # the later ops share this buffer

        # LBP: bit 7 the top-left neighbour, then clockwise
        nb = ((-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0))
        for mode, thr in ((0, 0), (0, -5), (1, 10)):
            run(23, [(80, 'Q', P), (104, 'I', SW), (96, 'Q', R), (112, 'I', SW),
                     (300, 'I', mode), (304, 'i', thr)])
            want = []
            for y in range(SH):
                for x in range(SW):
                    v = 0
                    for k, (dx, dy) in enumerate(nb):
                        d = px(a, SW, SH, x + dx, y + dy) - a[y * SW + x]
                        if (abs(d) if mode else d) >= thr:
                            v |= 1 << (7 - k)
                    want.append(v)
            check(list(q.read(R, n)) == want,
                  'LBP: %s mode, threshold %d' % ('abs' if mode else 'normal', thr))

        # MAG_AND_ANG: |gh| + |gv| thresholded, and the gradient's octant
        gh = correlate(a, SW, SH, sm, 5, 5)
        gv = correlate(a, SW, SH, smt, 5, 5)
        for thr in (0, 200):
            run(4, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (104, 'I', 2 * SW),
                    (88, 'Q', LUT), (108, 'I', SW), (328, 'I', 1),
                    (332, 'I', thr)] + [(300 + k, 'b', v) for k, v in enumerate(sm)])
            mag = [m if m >= thr else 0
                   for m in (min(abs(x) + abs(y), 0xffff) for x, y in zip(gh, gv))]
            check(list(struct.unpack('<%dH' % n, q.read(R, 2 * n))) == mag,
                  'MAG_AND_ANG: magnitude, threshold %d' % thr)
            check(list(q.read(LUT, n)) == [octant(x, y) for x, y in zip(gh, gv)],
                  'MAG_AND_ANG: the angle is not thresholded (%d)' % thr)
        run(4, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (104, 'I', 2 * SW),
                (328, 'I', 0), (332, 'I', 0)] +
            [(300 + k, 'b', v) for k, v in enumerate(sm)])
        check(list(struct.unpack('<%dH' % n, q.read(R, 2 * n))) ==
              [min(abs(x) + abs(y), 0xffff) for x, y in zip(gh, gv)],
              'MAG_AND_ANG: out_ctrl 0, magnitude alone')

        # NORM_GRAD: each gradient shifted right, halves up, saturated
        norm = 3
        want_h = [clamp((v + (1 << (norm - 1))) >> norm, -128, 127) & 0xff
                  for v in gh]
        want_v = [clamp((v + (1 << (norm - 1))) >> norm, -128, 127) & 0xff
                  for v in gv]
        mg = [(300 + k, 'b', v) for k, v in enumerate(sm)]
        run(24, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (104, 'I', SW),
                 (88, 'Q', LUT), (108, 'I', SW), (328, 'I', 0),
                 (336, 'I', norm)] + mg)
        check(list(q.read(R, n)) == want_h and list(q.read(LUT, n)) == want_v,
              'NORM_GRAD: both gradients')
        for ctrl, want in ((1, want_h), (2, want_v)):
            run(24, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (104, 'I', SW),
                     (328, 'I', ctrl), (336, 'I', norm)] + mg)
            check(list(q.read(R, n)) == want,
                  'NORM_GRAD: out_ctrl %d alone' % ctrl)
        run(24, [(64, 'Q', P), (96, 'I', SW), (80, 'Q', R), (104, 'I', 2 * SW),
                 (328, 'I', 3), (336, 'I', norm)] + mg)
        check(list(q.read(R, 2 * n)) ==
              [v for pair in zip(want_h, want_v) for v in pair],
              'NORM_GRAD: out_ctrl 3, interleaved as S8C2')

        # ACC, IMAGE_DOT, ALPHA_BLENDING
        for mode, f in ((0, lambda x, y: x + y), (1, lambda x, y: x - y),
                        (2, lambda x, y: x + (y & 1))):
            run(32, two + [(300, 'I', mode)])
            check(list(q.read(R, n)) == [clamp(f(x, y), 0, 255)
                                         for x, y in zip(a, b)],
                  'ACC: mode %d' % mode)
        run(34, [(80, 'Q', P), (88, 'Q', Q), (96, 'Q', P + 0x8000),
                 (112, 'Q', R), (120, 'I', SW), (124, 'I', SW),
                 (128, 'I', SW), (136, 'I', SW)])
        check(list(q.read(R, n)) ==
              [(x * al + y * (255 - al) + 128) >> 8
               for x, y, al in zip(a, b, binimg)], 'ALPHA_BLENDING')
        d1 = [(x - 32) * 4099 + y * 77 for y in range(SH) for x in range(SW)]
        d2 = [(y - 20) * 3001 - x * 911 for y in range(SH) for x in range(SW)]
        q.write(P + 0x10000, struct.pack('<%di' % n, *d1))
        q.write(Q + 0x10000, struct.pack('<%di' % n, *d2))
        run(33, [(80, 'Q', P + 0x10000), (88, 'Q', Q + 0x10000),
                 (104, 'Q', R), (112, 'I', 4 * SW), (116, 'I', 4 * SW),
                 (124, 'I', 4 * SW)])
        check(list(struct.unpack('<%di' % n, q.read(R, 4 * n))) ==
              [clamp((x * y + (1 << 29)) >> 30, -(1 << 31), (1 << 31) - 1)
               for x, y in zip(d1, d2)], 'IMAGE_DOT: round to S17Q15')

        # CSC both ways, on a YUV420SP / packed-RGB pair
        yuv = bytes((x * 9 + y * 5) & 0xff for y in range(SH) for x in range(SW))
        chroma = bytes((x * 13 + y * 29 + 11) & 0xff
                       for y in range(SH // 2) for x in range(SW))
        q.write(P, yuv)
        q.write(P + 0x1000, chroma)
        for mode in (0, 2):
            run(2, [(80, 'Q', P), (136, 'I', SW), (88, 'Q', P + 0x1000),
                    (140, 'I', SW), (112, 'Q', R), (152, 'I', 3 * SW),
                    (300, 'I', 1), (304, 'I', 2), (308, 'I', mode)])
            rows, lo, hi = CSC_TO_RGB[mode]
            want = []
            for y in range(SH):
                for x in range(SW):
                    c = chroma[(y // 2) * SW + (x & ~1)], chroma[(y // 2) * SW + (x & ~1) + 1]
                    for cy, cu, cv, k in rows:
                        want.append(clamp((cy * yuv[y * SW + x] + cu * c[1] +
                                           cv * c[0] + k) >> 10, lo, hi))
            check(list(q.read(R, 3 * n)) == want,
                  'CSC: mode %d, YUV420SP to packed RGB' % mode)
        rgb = bytes((x * 17 + y * 7 + c * 61) & 0xff
                    for y in range(SH) for x in range(SW) for c in range(3))
        q.write(P, rgb)
        for mode in (8, 10):
            run(2, [(80, 'Q', P), (136, 'I', 3 * SW), (112, 'Q', R),
                    (152, 'I', SW), (120, 'Q', R + 0x1000), (156, 'I', SW),
                    (300, 'I', 3), (304, 'I', 0), (308, 'I', mode)])
            my, mv, mu = CSC_TO_YUV[mode]
            want = [clamp((my[0] * rgb[3 * i] + my[1] * rgb[3 * i + 1] +
                           my[2] * rgb[3 * i + 2] + my[3]) >> 10, 0, 255)
                    for i in range(n)]
            check(list(q.read(R, n)) == want, 'CSC: mode %d, luma' % mode)
            want = []
            for y in range(0, SH, 2):
                for x in range(0, SW, 2):
                    i = 3 * (y * SW + x)
                    for m in (mv, mu):
                        want.append(clamp((m[0] * rgb[i] + m[1] * rgb[i + 1] +
                                           m[2] * rgb[i + 2] + m[3]) >> 10,
                                          0, 255))
            check(list(q.read(R + 0x1000, n // 2)) == want,
                  'CSC: mode %d, the chroma of each block top-left pixel' % mode)

        # CANNY_HYS_EDGE: the task has no picture -- the driver runs a
        # mag_and_ang task into the ctrl's mem first, so the input is a
        # U16 magnitude with the octant angle right after it
        MAGB = IMG + 0x240000
        STACK = IMG + 0x260000
        magv = [min(abs(x), 0xffff) for x in gh]
        angv = [octant(x, y) for x, y in zip(gh, gv)]
        q.write(MAGB, struct.pack('<%dH' % n, *magv))
        q.write(MAGB + 2 * SW * SH, bytes(angv))
        for lo, hi in ((60, 400), (2, 10), (300, 301)):
            q.write(STACK, b'\xee' * (4 * n + 16))
            run(22, [(64, 'Q', MAGB), (96, 'I', 2 * SW), (80, 'Q', R),
                     (104, 'I', SW), (88, 'Q', STACK), (300, 'I', lo),
                     (304, 'I', hi)])
            want, seeds = canny_ref(magv, angv, SW, SH, lo, hi)
            check(list(q.read(R, n)) == want,
                  'CANNY_HYS_EDGE: classes, %d / %d' % (lo, hi))
            got = list(struct.unpack('<%dI' % len(seeds),
                                     q.read(STACK, 4 * len(seeds)))) if seeds else []
            check(got == seeds,
                  'CANNY_HYS_EDGE: %d seeds in raster order' % len(seeds))
            check(struct.unpack('<I', q.read(STACK + 4 * n, 4))[0] == len(seeds),
                  'CANNY_HYS_EDGE: the stack size past the points')

        # GMM2: three frames over a picture that steps, with the model
        # read back each time -- 8 bytes per Gaussian, model_num of them
        # per pixel, pixel by pixel
        MODEL = IMG + 0x280000
        FG, BG, MINFO = R, R + 0x1000, R + 0x2000
        c = GMM2_CTRL
        q.write(MODEL, bytes(8 * c['num'] * n))
        state = [[[0, 0, 0, 0] for _ in range(c['num'])] for _ in range(n)]
        gfields = [(72, 'Q', P), (132, 'I', SW), (80, 'Q', MODEL),
                   (120, 'Q', MODEL), (96, 'Q', MINFO), (144, 'I', SW),
                   (104, 'Q', FG), (148, 'I', SW), (112, 'Q', BG),
                   (152, 'I', SW), (312, 'I', c['life_factor']),
                   (316, 'I', c['life_threshold']), (320, 'I', c['freq_init']),
                   (324, 'I', c['freq_reduce']), (328, 'I', c['freq_add']),
                   (332, 'I', c['freq_threshold']), (336, 'I', c['var_rate']),
                   (340, 'I', c['max_var']), (344, 'I', c['min_var']),
                   (348, 'I', c['sns']), (352, 'I', c['num'])]
        for frame in range(3):
            pic = bytes((100 if frame < 2 else 100 + i % SW) & 0xff
                        for i in range(n))
            q.write(P, pic)
            run(30, gfields)
            want = [gmm2_pixel(state[i], pic[i], c) for i in range(n)]
            got_model = list(struct.unpack('<%dH' % (4 * c['num'] * n),
                                           q.read(MODEL, 8 * c['num'] * n)))
            flat = [v for px in state for g in px for v in g]
            check(got_model == flat, 'GMM2: the model after frame %d' % frame)
            check(list(q.read(FG, n)) == [t[0] for t in want],
                  'GMM2: foreground, frame %d' % frame)
            check(list(q.read(BG, n)) == [t[1] for t in want],
                  'GMM2: background, frame %d' % frame)
            check(list(q.read(MINFO, n)) == [t[2] for t in want],
                  'GMM2: match info, frame %d' % frame)

        # LK_OPTICAL_FLOW_PYR: the task's size fields hold 2 and the
        # maximum level, with the picture's own size at +320 / +324
        LKW, LKH = 64, 64
        LKP, LKN = IMG + 0x300000, IMG + 0x310000
        PIN, POUT = IMG + 0x320000, IMG + 0x320100
        LST, LERR = IMG + 0x320200, IMG + 0x320300
        wave = bytes(clamp(int(128 + 40 * math.sin(x / 2.1) +
                               30 * math.sin(y / 1.7)), 0, 255)
                     for y in range(LKH) for x in range(LKW))
        shifted = bytes(clamp(int(128 + 40 * math.sin((x - 1) / 2.1) +
                                  30 * math.sin(y / 1.7)), 0, 255)
                        for y in range(LKH) for x in range(LKW))
        q.write(LKP, wave)
        q.write(LKN, shifted)
        lkpts = ((21, 17), (33, 29), (45, 41))
        q.write(PIN, b''.join(struct.pack('<ii', x << 7, y << 7)
                              for x, y in lkpts))
        q.write(LST, b'\xee' * len(lkpts))
        i = (q.readl(SQ + 0x20) + 1) % NSLOT
        t = bytearray(task(i, 28, 2, 0, [(64, 'Q', LKP), (176, 'I', LKW),
                                         (96, 'Q', LKN), (192, 'I', LKW),
                                         (128, 'Q', PIN), (136, 'Q', POUT),
                                         (160, 'Q', LST), (168, 'Q', LERR),
                                         (304, 'I', 100), (308, 'I', 2),
                                         (312, 'I', 10), (316, 'I', len(lkpts)),
                                         (320, 'I', LKW), (324, 'I', LKH)]))
        q.write(slot(i), bytes(t))
        q.writel(SQ + 0x10, 1 << 8)
        q.cmd('clock_step 2000000')
        q.writel(SQ + 0x14, 31)
        got = struct.unpack('<%di' % (2 * len(lkpts)), q.read(POUT, 8 * len(lkpts)))
        check(list(q.read(LST, len(lkpts))) == [1] * len(lkpts),
              'LK: every point tracked')
        # a one-pixel shift along x, to an eighth of a pixel
        ok = all(abs(got[2 * k] - ((x + 1) << 7)) <= 16 and
                 abs(got[2 * k + 1] - (y << 7)) <= 16
                 for k, (x, y) in enumerate(lkpts))
        check(ok, 'LK: the flow is the picture\'s shift (%s)' %
              ' '.join('%.2f,%.2f' % (got[2 * k] / 128 - x, got[2 * k + 1] / 128 - y)
                       for k, (x, y) in enumerate(lkpts)))

        # RESIZE: the op whose parameters sit in a block of its own, which
        # is also why its task carries no op code and no size
        BLK = IMG + 0x300000
        for dw, dh in ((10, 6), (SW * 2, SH * 2)):
            q.write(P, a)
            q.write(BLK, struct.pack('<II', 29, 1))
            xr, yr = (SW << 27) // dw, (SH << 27) // dh
            q.write(BLK + 16, struct.pack('<IIIIIIIIIiIi', P, R, SW, dw,
                                          SW, SH, dw, dh, xr,
                                          ((xr - (1 << 27)) >> 1) & 0x7ffffff
                                          if xr >= (1 << 27)
                                          else (xr - (1 << 27)) >> 1,
                                          yr,
                                          ((yr - (1 << 27)) >> 1) & 0x7ffffff
                                          if yr >= (1 << 27)
                                          else (yr - (1 << 27)) >> 1))
            i = (q.readl(SQ + 0x20) + 1) % NSLOT
            t = bytearray(task(i, 0, 0, 0, [(64, 'Q', BLK)]))
            q.write(slot(i), bytes(t))
            q.writel(SQ + 0x10, 1 << 8)
            q.cmd('clock_step 2000000')
            q.writel(SQ + 0x14, 31)
            check(list(q.read(R, dw * dh)) == resize_ref(a, SW, SH, dw, dh),
                  'RESIZE: %dx%d to %dx%d' % (SW, SH, dw, dh))
    finally:
        q.close()
    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
