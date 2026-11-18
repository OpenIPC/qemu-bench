#!/usr/bin/env python3
"""Drive hisi-vedu gen 7 (GK7206 / Xiongmai xmorca, hisi-vedu-xm.c.inc) the
way XM's xm_h264e / xm_h265e / xm_jpege program it, and check that ffmpeg
decodes what the model produces.

The parameter sets, H.265 slice header bits and JFIF header are the HiMPP
V4 ones test-hisi-vedu.py replays (XM's drivers code the same syntax; the
slice header register holds the bits from slice_type on, as V4's part 2,
which a real GK7206V11A's ring confirmed).  What differs and is checked
here: everything over MMIO, START at 0x10, interrupts at 0x90..0x9c, the
ring's pointer cells at 0x370 / 0x374, packets padded to 8 bytes, the
engine's packet header from 0x2d0..0x2e0, the NAL type from 0x300; JPGE's
quantiser write port, its stream registers and the write pointer it
reports at 0x74.

    python3 qemu/tests/test-hisi-vedu-xm.py [--qemu PATH]
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
QTest = import_module('test-xm-h1-jpeg').QTest
hv = import_module('test-hisi-vedu')

VEDU, JPGE = 0x11800000, 0x11420000
W, H = hv.W, hv.H
SRC = 0x44000000
RING_SIZE = 0x80000
RINGS = {0: 0x45000000, 1: 0x45080000, 2: 0x45100000}
CELLS = {c: RINGS[c] + RING_SIZE + 0x40 for c in RINGS}   # driver's struct
H265_HDR = {True: (0x0b, [0x7ee00000]),                  # from V4 part 2
            False: (0x2f, [0x400025fc, 0x212e0000])}


def p_header(poc):
    """The P slice header with another slice_pic_order_cnt_lsb: 16 bits
    after slice_type's 3, POC 1 in V4's."""
    nbits, words = H265_HDR[False]
    v = words[0] << 32 | words[1]
    v = v & ~(0xffff << 45) | poc << 45
    return nbits, [v >> 32, v & 0xffffffff]


def packet(ptype, payload, chn):
    pad = (-len(payload)) % 8
    hdr = struct.pack('<IIBBBBIQ', 64 + len(payload) + pad, pad, ptype, 0, 0,
                      0, chn, 0)
    return hdr.ljust(64, b'\0') + payload + b'\0' * pad

def dqt_tables(hdr):
    """The header's quantisers, natural (row-major) order, by table id."""
    tables, i = {}, hdr.index(b'\xff\xdb') + 4
    while hdr[i] != 0xff:
        nat = [0] * 64
        for k, v in enumerate(hdr[i + 1:i + 65]):
            nat[hv.ZIGZAG[k]] = v
        tables[hdr[i] & 15] = nat
        i += 65
    return tables


def with_dqt(hdr, tables):
    """hdr with tables (natural order) in its DQT."""
    hdr, i = bytearray(hdr), hdr.index(b'\xff\xdb') + 4
    while hdr[i] != 0xff:
        t = tables.get(hdr[i] & 15)
        if t:
            hdr[i + 1:i + 65] = bytes(t[hv.ZIGZAG[k]] for k in range(64))
        i += 65
    return bytes(hdr)


def xm_dht(hdr):
    """hdr with XM's DHT: the luma DC / AC tables as tables 1 too."""
    i = hdr.index(b'\xff\xc4')
    j = i
    segs = {}
    while hdr[j:j + 2] == b'\xff\xc4':
        n = hdr[j + 2] << 8 | hdr[j + 3]
        k = j + 4
        while k < j + 2 + n:
            cnt = sum(hdr[k + 1:k + 17])
            segs[hdr[k]] = hdr[k + 1:k + 17 + cnt]
            k += 17 + cnt
        j += 2 + n
    body = b''.join(bytes([c]) + segs[c & 0x10] for c in (0x00, 0x10, 0x01,
                                                          0x11))
    return hdr[:i] + b'\xff\xc4' + struct.pack('>H', 2 + len(body)) + \
        body + hdr[j:]


def decode_planes(path):
    """A JPEG's Y, Cb, Cr planes (4:2:0, as the decoder outputs them)."""
    pix = subprocess.run(['ffprobe', '-v', 'error', '-f', 'mjpeg',
                          '-show_entries', 'stream=pix_fmt', '-of', 'csv=p=0',
                          path], capture_output=True, text=True).stdout.strip()
    err = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode',
                          '-f', 'mjpeg', '-i', path, '-f', 'null', '-'],
                         capture_output=True, text=True)
    raw = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'mjpeg', '-i', path,
                          '-f', 'rawvideo', '-'], capture_output=True).stdout
    ok = (pix in ('yuvj420p', 'yuv420p') and err.returncode == 0 and
          not err.stderr.strip() and len(raw) == W * H * 3 // 2)
    c = W * H // 4
    return ok, raw[:W * H], raw[W * H:W * H + c], raw[W * H + c:]


def psnr(a, b):
    se = sum((x - y) ** 2 for x, y in zip(a, b))
    return 99.0 if not se else 10 * math.log10(255 * 255 * len(a) / se)


def jpeg(q, check, tmp, pic):
    """JPGE as xm_jpege drives it: the quantisers in natural order (the
    table its DQT carries), the source format in 0x04."""
    # a pattern that changes along x only, in every 8x8 block
    del pic
    y = bytes(int(128 + 60 * math.sin(2 * math.pi * x / 6.3))
              for _ in range(H) for x in range(W))
    u = bytes(int(128 + 50 * math.sin(2 * math.pi * x / 4.7))
              for _ in range(H // 2) for x in range(W // 2))
    v = bytes(int(128 - 40 * math.cos(2 * math.pi * x / 9.1))
              for _ in range(H // 2) for x in range(W // 2))
    vu = bytes(b for pair in zip(v, u) for b in pair)
    std = dqt_tables(hv.JPEG_HDR)
    # vertical bars only change along x: their energy is in the first row
    # of coefficients; a table fine there and coarse elsewhere keeps them
    # only if the engine reads the table the way the header has it
    rows = {t: [2 if k < 8 else 255 for k in range(64)] for t in (0, 1)}
    q.write(SRC, y)
    q.write(SRC + W * H, vu)
    for name, fmt, tables, src_uv in (
            ('NV21 (format 1), the stock tables', 1, std, (u, v)),
            ('NV12 (format 2)', 2, std, (v, u)),
            ('first-row-only quantisers', 1, rows, (u, v))):
        # XM's header: Cb and Cr both on quantiser 1 (V4's has Cr on 2),
        # Huffman tables 1 the same as 0 (Annex K luma)
        hdr = xm_dht(with_dqt(hv.JPEG_HDR, {**tables, 2: tables[1]}))
        # the driver's header stops before SOS: the engine writes it
        sos = hdr.index(b'\xff\xda')
        hdr, xm_sos = hdr[:sos], bytes.fromhex('ffda000c03010002110311003f00')

        ring = RINGS[2]
        p = packet(8, hdr, 2)
        q.write(ring, p)
        data = len(p) + 64
        for t, port in ((0, 0x48), (1, 0x50)):
            for r in range(64):     # register r: natural coefficient r
                coef = (1 << 20) // tables[t][r]
                q.writel(JPGE + port, 1 << 27 | r << 21 | coef)
            q.writel(JPGE + port, 0)
        for off, val in ((0x04, fmt | 1 << 5 | H << 7), (0x24, W), (0x28, H),
                         (0x34, SRC), (0x30, SRC + W * H), (0x40, W),
                         (0x38, W), (0x60, ring), (0x64, ring + RING_SIZE),
                         (0x68, ring + data), (0x70, ring), (0x200, 1),
                         (0x214, 0x323), (0x80, 1)):
            q.writel(JPGE + off, val)
        q.cmd('clock_step 5000000')
        raw, soc = q.readl(JPGE + 0x224), q.readl(JPGE + 0x208)
        check(raw & 2 and soc & 1 and q.readl(JPGE + 0x220) & 2,
              'JPEG %s: frame end, SoC interrupt (0x%x, %d)' % (name, raw, soc))
        wptr, invld = q.readl(JPGE + 0x74), q.readl(JPGE + 0x78)
        scan = q.read(ring + data, wptr - data - invld)
        check(wptr > data and scan.startswith(xm_sos) and
              scan.endswith(b'\xff\xd9'),
              'JPEG %s: write pointer 0x%x behind SOS, scan and EOI'
              % (name, wptr))
        q.writel(JPGE + 0x218, raw)
        q.writel(JPGE + 0x204, 1)
        check(q.readl(JPGE + 0x224) == 0 and q.readl(JPGE + 0x208) == 0,
              'JPEG %s: cleared' % name)
        path = os.path.join(tmp, 'jpeg')
        open(path, 'wb').write(hdr + scan)
        ok, dy, dcb, dcr = decode_planes(path)
        check(ok, 'JPEG %s: decodes cleanly' % name)
        py = hv.luma_psnr(dy, y)
        pcb, pcr = psnr(dcb, src_uv[0]), psnr(dcr, src_uv[1])
        check(min(py, pcb, pcr) > (30 if tables is std else 40),
              'JPEG %s: PSNR Y %.1f Cb %.1f Cr %.1f dB' % (name, py, pcb, pcr))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='gk7206v11a')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    tmp = tempfile.mkdtemp()
    # H.265 P pictures: the default here is skip-or-intra (no motion
    # search below inter-max-pixels = 0); the motion search and all-skip
    # tiers are run after the rest
    q = QTest(a.qemu, a.machine)
    pics = [hv.frame(0), hv.frame(1)]
    refs = [hv.blocks4(p[0]) for p in pics]

    def h265(vedu_job, mode, npics):
        """H.265 under the driver's slice header bits at 0x324 / 0x330;
        pictures past the second code the second's source again."""
        q.writel(CELLS[1] + 0x80, 0)
        stream = b''.join(bytes.fromhex('00000001' + h) for _, h in hv.H265_PS)
        for n in range(npics):
            nbits, words = H265_HDR[True] if n == 0 else p_header(n)
            # a P picture's header at 0x330 (count in 0x324's low half), an
            # IDR's at 0x350 (high half); the other slot holds a stale one
            if n == 0:
                regs = {0x324: 0x2f | nbits << 16}
                regs.update({0x350 + 4 * i: v for i, v in enumerate(words)})
                regs.update({0x330: 0xdeadbeef, 0x334: 0xdeadbeef})
            else:
                regs = {0x324: nbits | 0x0b << 16}
                regs.update({0x330 + 4 * i: v for i, v in enumerate(words)})
                regs.update({0x350: 0xdeadbeef})
            t, nal = vedu_job(min(n, 1), hv.H265_PS if n == 0 else [], 1, regs)
            check(t == (19 if n == 0 else 1), 'H.265 (%s) picture %d: type %d'
                  % (mode, n, t))
            check(nal[:5] == b'\0\0\1' + (b'\x26\x01' if n == 0
                                              else b'\x02\x01'),
                  'H.265 (%s) picture %d: 3-byte start code, NAL header %s'
                  % (mode, n, nal[3:5].hex()))
            stream += nal
        path = os.path.join(tmp, 'h265')
        open(path, 'wb').write(stream)
        ok, raw = hv.decode(path, 'hevc')
        check(ok and len(raw) == npics * W * H,
              'H.265 (%s): %d pictures decode cleanly' % (mode, npics))
        for n in range(len(raw) // (W * H)):
            want = refs[0] if mode == 'all skip' else refs[min(n, 1)]
            p = hv.luma_psnr(raw[n * W * H:(n + 1) * W * H], want)
            check(p > 35, 'H.265 (%s) picture %d: luma PSNR %.1f dB%s' %
                  (mode, n, p, ' against picture 0' if want is refs[0] and n
                   else ''))

    def vedu_job(n, ps, chn, regs):
        ring, cell = RINGS[chn], CELLS[chn]
        wp_cell, rp_cell = cell + 0x80, cell + 0x40
        y, vu = pics[n]
        q.write(SRC, y)
        q.write(SRC + W * H, vu)
        start = q.readl(wp_cell)
        for t, hexs in ps:
            p = packet(t, bytes.fromhex('00000001' + hexs), chn)
            q.write(ring + start, p)
            start += len(p)
        q.writel(wp_cell, start)
        r = {0x24: W | H << 16, 0x240: SRC, 0x244: SRC + W * H,
             0x1b8: W, 0x1d0: W, 0x304: RING_SIZE, 0x308: ring,
             0x310: start, 0x370: wp_cell, 0x374: rp_cell,
             0x2d0: 40000 * n + 7, 0x2d4: 0x6607a, 0x2e0: chn,
             0x300: 0x2601 if n == 0 else 0x0201, 0x90: 0x10a1}
        r.update(regs)
        for off, v in r.items():
            q.writel(VEDU + off, v)
        q.writel(VEDU + 0x10, 0)
        q.writel(VEDU + 0x10, 1)
        check(q.readl(VEDU + 0x9c) == 0, 'picture %d: nothing raised at START'
              % n)
        t0 = time.time()
        q.cmd('clock_step 5000000')
        dt = time.time() - t0
        st = q.readl(VEDU + 0x98)
        check(st == 1, 'picture %d: end of picture in INTSTAT (0x%x, %.0f ms)'
              % (n, st, dt * 1000))
        end = q.readl(wp_cell)
        ln, pad, ptype, _, _, flags, pchn, pts = struct.unpack(
            '<IIBBBBIQ', q.read(ring + start, 24))
        check(end == start + ln and ln % 8 == 0 and pad < 8,
              'picture %d: write pointer behind its packet (0x%x, %d bytes, '
              'pad %d)' % (n, end, ln, pad))
        check(pchn == chn and pts == (0x6607a << 32) | (40000 * n + 7)
              and flags == 1, 'picture %d: channel %d, PTS 0x%x, last slice'
              % (n, pchn, pts))
        check(q.readl(VEDU + 0xac) == ln,
              'picture %d: frame bytes for rate control' % n)
        # xm_rc: mean_qp = 0xd8c / (pixels / 16), mse = (0xd84/88 +
        # 0xd94/98) / pixels, activity per 4x4 block in 0xd6c / 0xd70
        st = {o: q.readl(VEDU + o) for o in range(0xd6c, 0xdc8, 4)}
        mean_qp = st[0xd8c] / (W * H // 16)
        sse = (st[0xd84] | (st[0xd88] & 0x3ff) << 32) + \
            (st[0xd94] | (st[0xd98] & 0x3ff) << 32)
        mse = sse / (W * H)
        check(mean_qp == int(mean_qp) and 0 < mean_qp <= 51 and
              mse < 255 * 255 and st[0xd90] <= W * H // 64 and
              not st[0xdc4] and (n or (mse > 0 and not st[0xd70] and
                                       st[0xd90] == W * H // 64)),
              'picture %d: rate control statistics: mean QP %d, MSE %.2f, '
              'intra %.2f inter %.2f per 4x4, %d 8x8 coded'
              % (n, mean_qp, mse, st[0xd6c] / (W * H / 16),
                 st[0xd70] / (W * H / 16), st[0xd90]))
        q.writel(VEDU + 0x94, q.readl(VEDU + 0x9c))
        check(q.readl(VEDU + 0x98) == 0 and q.readl(VEDU + 0x9c) == 0,
              'picture %d: cleared' % n)
        return ptype, q.read(ring + start + 64, ln - 64 - pad)

    try:
        # H.264: the model writes the slice header itself
        q.writel(CELLS[0] + 0x80, 0)
        stream = b''.join(bytes.fromhex('00000001' + h) for _, h in hv.H264_PS)
        for n in range(2):
            t, nal = vedu_job(n, hv.H264_PS if n == 0 else [], 0, {})
            check(t == (5 if n == 0 else 1), 'H.264 picture %d: type %d' % (n, t))
            stream += nal
        path = os.path.join(tmp, 'h264')
        open(path, 'wb').write(stream)
        ok, raw = hv.decode(path, 'h264')
        check(ok and len(raw) == 2 * W * H, 'H.264: two pictures decode cleanly')
        for n in range(2):
            p = hv.luma_psnr(raw[n * W * H:(n + 1) * W * H], refs[n])
            check(p > 35, 'H.264 picture %d: luma PSNR %.1f dB' % (n, p))

        h265(vedu_job, 'skip or intra', 2)

        # a ring with no room: buffer full, nothing written
        cell = CELLS[1]
        wp = q.readl(cell + 0x80)
        q.writel(cell + 0x40, wp + 64)
        q.write(RINGS[1] + wp, b'\xa5' * 64)
        q.writel(VEDU + 0x10, 0)
        q.writel(VEDU + 0x10, 1)
        q.cmd('clock_step 5000000')
        raw = q.readl(VEDU + 0x9c)
        check(raw & 4, 'full ring: buffer full (0x%x)' % raw)
        check(q.readl(cell + 0x80) == wp and
              q.read(RINGS[1] + wp, 64) == b'\xa5' * 64,
              'full ring: nothing written')
        q.writel(VEDU + 0x94, 0xffffffff)
        q.writel(cell + 0x40, 0)

        # JPEG: the driver's JFIF header packet and the scan's header slot;
        # the engine writes the scan at 0x68 and reports where it ended
        jpeg(q, check, tmp, pics[0])
    finally:
        q.close()

    for mode, extra in (('motion search', ['-global',
                         'hisi-vedu.inter-max-pixels=1000000']),
                        ('all skip', ['-global',
                         'hisi-vedu.skip-min-pixels=0'])):
        q = QTest(a.qemu, a.machine, extra)
        try:
            h265(vedu_job, mode, 3)
        finally:
            q.close()

    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
