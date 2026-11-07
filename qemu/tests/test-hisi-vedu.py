#!/usr/bin/env python3
"""Drive hisi-vedu (VEDU + JPGE) the way the HiMPP V4 driver does, and
check that ffmpeg decodes what the model produces.

Everything the driver writes is replayed from a trace of the real thing
(OpenIPC hi3516ev200, hisi-venc-userframe, -global hisi-vedu.trace=on):
the parameter-set packets it puts in the stream ring, the H.265 slice
header bits it leaves in the register image, the JFIF header it writes in
front of a JPEG scan and the quantisers' reciprocals it programs.

    python3 qemu/tests/test-hisi-vedu.py [--qemu PATH] [--machine M]
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
QTest = import_module('test-xm-h1-jpeg').QTest

VEDU, JPGE = 0x11410000, 0x11420000
W, H = 640, 368                 # the size the captured SPS describe
SRC = 0x41000000                # NV21: Y, then VU
RING_SIZE = 0x80000            # one ring per channel, as the driver has
RINGS = {0: 0x42000000, 1: 0x42080000, 2: 0x42300000}
WP_CELL, RP_CELL = 0x42100080, 0x42100040
IMAGE = 0x42200000

# the driver's packets for an IDR, as hi_h264e / hi_h265e write them
H264_PS = [(7, '6742001e963541405f4dc040404080'),
           (8, '68ce31b2')]
H265_PS = [(32, '40010c01ffff016000000300b0000003000003003fac09'),
           (33, '420101016000000300b0000003000003003fa005020171636bb932'
                'fcdc04040402'),
           (34, '4401c0f2b03b24')]
# H.265 slice header bits from the register image (0x404 / 0x408 / 0x40c)
H265_HDR = {True: (0x000b0002, 0x00004000, [0x7ee00000]),
            False: (0x002f0001, 0x00008000, [0x400025fc, 0x212e0000])}
# the JFIF header hi_jpege writes for 640x368 at Qfactor 90 (SOI .. SOS)
JPEG_HDR = bytes.fromhex(
    'ffd8ffe000104a46494600010100000100010000ffdb00c50003020203020203'
    '03030304030304050805050404050a070706080c0a0c0c0b0a0b0b0d0e12100d'
    '0e110e0b0b1016101113141515150c0f17181614181214151401030404050405'
    '09050509140d0b0d141414141414141414141414141414141414141414141414'
    '1414141414141414141414141414141414141414141414141414020304040504'
    '0509050509140d0b0d1414141414141414141414141414141414141414141414'
    '141414141414141414141414141414141414141414141414141414ffc0001108'
    '0170028003012200021101031102ffc401a20000010501010101010100000000'
    '000000000102030405060708090a0b100002010303020403050504040000017d'
    '01020300041105122131410613516107227114328191a1082342b1c11552d1f0'
    '2433627282090a161718191a25262728292a3435363738393a43444546474849'
    '4a535455565758595a636465666768696a737475767778797a83848586878889'
    '8a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5'
    'c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8'
    'f9fa0100030101010101010101010000000000000102030405060708090a0b11'
    '0002010204040304070504040001027700010203110405213106124151076171'
    '1322328108144291a1b1c109233352f0156272d10a162434e125f11718191a26'
    '2728292a35363738393a434445464748494a535455565758595a636465666768'
    '696a737475767778797a82838485868788898a92939495969798999aa2a3a4a5'
    'a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9da'
    'e2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9faffdd00040000ffda000c03010002'
    '110311003f00')

ZIGZAG = [0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19,
          26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28, 35, 42, 49,
          56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52, 45,
          38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]


def frame(n):
    """hisi-venc-userframe's picture: moving bars and a binary counter."""
    bar_y = [235, 210, 170, 145, 106, 81, 41, 16]
    bw = W // 8
    y = bytearray(W * H)
    vu = bytearray(W * H // 2)
    for r in range(H):
        y[r * W:(r + 1) * W] = bytes(bar_y[((x + n * 8) // bw) % 8]
                                     for x in range(W))
    for r in range(H // 2):
        for x in range(W // 2):
            b = ((x * 2 + n * 8) // bw) % 8
            vu[r * W + x * 2] = 188 if b & 1 else 68
            vu[r * W + x * 2 + 1] = 188 if b & 2 else 68
    for x in range(16):
        v = 235 if (n >> (15 - x)) & 1 else 16
        for r in range(32):
            y[r * W + x * (W // 16):r * W + (x + 1) * (W // 16)] = \
                bytes([v]) * (W // 16)
    return bytes(y), bytes(vu)


def packet(ptype, payload, chn, last=False, slice_type=0):
    pad = (-len(payload)) % 64
    hdr = struct.pack('<IIBBBBIQ', 64 + len(payload) + pad, pad, ptype, 0, 0,
                      (1 if last else 0) | slice_type << 6, chn, 0)
    return hdr.ljust(64, b'\0') + payload + b'\0' * pad


def luma_psnr(a, b):
    if len(a) != len(b):
        return 0.0
    mse = sum((x - y) ** 2 for x, y in zip(a, b)) / len(a)
    return 99.0 if mse == 0 else 10 * math.log10(255 ** 2 / mse)


def blocks4(y):
    """The luma averaged over 4x4 blocks: what DC-only intra can reach."""
    out = bytearray(len(y))
    for by in range(0, H, 4):
        for bx in range(0, W, 4):
            s = sum(y[(by + r) * W + bx + c] for r in range(4) for c in range(4))
            for r in range(4):
                out[(by + r) * W + bx:(by + r) * W + bx + 4] = bytes([(s + 8) // 16]) * 4
    return bytes(out)


def decode(path, fmt):
    """Clean decode?  And the luma planes, in the decoder's own pixel format:
    converting to yuv420p rescales a full-range stream's luma on some
    ffmpeg versions, and gray does it on all of them."""
    err = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode',
                          '-f', fmt, '-i', path, '-f', 'null', '-'],
                         capture_output=True, text=True)
    pix = subprocess.run(['ffprobe', '-v', 'error', '-f', fmt,
                          '-show_entries', 'stream=pix_fmt', '-of', 'csv=p=0',
                          path], capture_output=True, text=True).stdout.strip()
    if pix not in ('yuv420p', 'yuvj420p'):
        return False, b''
    raw = subprocess.run(['ffmpeg', '-v', 'error', '-f', fmt, '-i', path,
                          '-f', 'rawvideo', '-'], capture_output=True).stdout
    luma = b''.join(raw[i:i + W * H] for i in range(0, len(raw), W * H * 3 // 2))
    return err.returncode == 0 and not err.stderr.strip(), luma


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='hi3516ev200')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    tmp = tempfile.mkdtemp()
    q = QTest(a.qemu, a.machine)
    pics = [frame(0), frame(1)]
    refs = [blocks4(p[0]) for p in pics]

    def vedu_job(n, img_extra, ps, chn):
        """One picture: the driver's packets, the image, START, completion.
        Returns the engine's packet (header fields, payload)."""
        RING = RINGS[chn]
        y, vu = pics[n]
        q.write(SRC, y)
        q.write(SRC + W * H, vu)
        start = q.readl(WP_CELL)
        for t, hexs in ps:
            p = packet(t, bytes.fromhex('00000001' + hexs), chn)
            q.write(RING + start, p)
            start += len(p)
        q.writel(WP_CELL, start)
        img = {0x104: SRC, 0x108: SRC + W * H, 0x1f4: W | W << 16,
               0x12c: WP_CELL, 0x130: RP_CELL, 0x16c: RINGS[chn],
               0x4a0: RING_SIZE, 0x4a4: 40000 * n, 0x61c: 26 + 2 * n}
        img.update(img_extra)
        q.write(IMAGE, b'\0' * 0x1000)
        for off, v in img.items():
            q.writel(IMAGE + off, v)
        q.writel(VEDU + 0x0000, 1)                  # INTMASK: end of picture
        q.writel(VEDU + 0x0014, (W - 1) | (H - 1) << 16)
        q.writel(VEDU + 0x0058, IMAGE)
        q.writel(VEDU + 0x0008, 1)
        check(not q.readl(VEDU + 0x3010) & 1, 'picture %d: busy after START' % n)
        q.cmd('clock_step 5000000')
        st = q.readl(VEDU + 0x3000)
        check(st & 1, 'picture %d: end of picture (0x%x)' % (n, st))
        end = q.readl(WP_CELL)
        ln, pad, ptype, _, _, flags, pchn, pts = struct.unpack(
            '<IIBBBBIQ', q.read(RING + start, 24))
        check(end == start + ln, 'picture %d: write pointer at the end of '
              'its packet (0x%x)' % (n, end))
        check(pchn == chn and pts == 40000 * n,
              'picture %d: channel %d, PTS %d' % (n, pchn, pts))
        q.writel(VEDU + 0x0004, 1)
        check(q.readl(VEDU + 0x3000) == 0, 'picture %d: cleared' % n)
        return ptype, flags, q.read(RING + start + 64, ln - 64 - pad)

    try:
        # H.264: the model writes the slice header itself, from the SPS/PPS
        q.writel(WP_CELL, 0)
        stream = b''.join(bytes.fromhex('00000001' + h) for _, h in H264_PS)
        for n in range(2):
            t, flags, nal = vedu_job(n, {}, H264_PS if n == 0 else [], 0)
            check(t == (5 if n == 0 else 1) and flags >> 6 == n and flags & 1,
                  'H.264 picture %d: packet type %d, flags 0x%x' % (n, t, flags))
            stream += nal
        path = os.path.join(tmp, 'h264')
        open(path, 'wb').write(stream)
        ok, raw = decode(path, 'h264')
        check(ok and len(raw) == 2 * W * H, 'H.264: two pictures decode cleanly')
        for n in range(2):
            p = luma_psnr(raw[n * W * H:(n + 1) * W * H], refs[n])
            check(p > 35, 'H.264 picture %d: luma PSNR %.1f dB against the '
                  '4x4 averages' % (n, p))

        # H.265: under the driver's own slice header bits
        q.writel(WP_CELL, 0)
        stream = b''.join(bytes.fromhex('00000001' + h) for _, h in H265_PS)
        for n in range(2):
            ln, p1, p2 = H265_HDR[n == 0]
            img = {0x404: ln, 0x408: p1}
            img.update({0x40c + 4 * i: v for i, v in enumerate(p2)})
            t, flags, nal = vedu_job(n, img, H265_PS if n == 0 else [], 1)
            check(t == (19 if n == 0 else 1),
                  'H.265 picture %d: packet type %d' % (n, t))
            stream += nal
        path = os.path.join(tmp, 'h265')
        open(path, 'wb').write(stream)
        ok, raw = decode(path, 'hevc')
        check(ok and len(raw) == 2 * W * H, 'H.265: two pictures decode cleanly')
        for n in range(2):
            p = luma_psnr(raw[n * W * H:(n + 1) * W * H], refs[n])
            check(p > 35, 'H.265 picture %d: luma PSNR %.1f dB against the '
                  '4x4 averages' % (n, p))

        # JPEG: the driver's JFIF header packet, the scan from the engine
        hdr = JPEG_HDR
        tables, i = {}, hdr.index(b'\xff\xdb') + 4
        while hdr[i] != 0xff:
            tables[hdr[i] & 15] = hdr[i + 1:i + 65]
            i += 65
        q.write(SRC, pics[0][0])
        q.write(SRC + W * H, pics[0][1])
        RING = RINGS[2]
        p = packet(8, hdr, 2)
        q.write(RING, p)
        q.writel(WP_CELL, len(p))
        for t, base in ((0, 0x800), (1, 0x900), (1, 0xa00)):
            nat = [0] * 64
            for k, v in enumerate(tables[t]):
                nat[ZIGZAG[k]] = v
            for r in range(64):     # register r: row r % 8, column r / 8
                q.writel(JPGE + base + 4 * r, (1 << 23) // nat[(r % 8) * 8 + r // 8])
        for off, v in ((0x004, 3), (0x034, (W - 1) | (H - 1) << 16),
                       (0x060, RING_SIZE), (0x164, W | W << 16),
                       (0x200, SRC), (0x204, SRC + W * H), (0x210, RING),
                       (0x214, RP_CELL), (0x218, WP_CELL), (0x02c, 0),
                       (0x02c, 1)):
            q.writel(JPGE + off, v)
        q.cmd('clock_step 5000000')
        st = q.readl(JPGE + 0x000)
        check(st & 1, 'JPEG: done (0x%x)' % st)
        ln, pad, t = struct.unpack('<IIB', q.read(RING + len(p), 9))
        check(t == 5, 'JPEG: an ECS packet (type %d)' % t)
        scan = q.read(RING + len(p) + 64, ln - 64 - pad)
        check(scan.endswith(b'\xff\xd9'), 'JPEG: the scan ends with EOI')
        q.writel(JPGE + 0x00c, 0xffffffff)
        check(q.readl(JPGE + 0x000) == 0, 'JPEG: cleared')
        path = os.path.join(tmp, 'jpeg')
        open(path, 'wb').write(hdr + scan)
        ok, raw = decode(path, 'mjpeg')
        check(ok and len(raw) == W * H, 'JPEG: decodes cleanly')
        p = luma_psnr(raw, pics[0][0])
        check(p > 35, 'JPEG: luma PSNR %.1f dB' % p)
        # A ring with no room left: buffer full, nothing written, the write
        # pointer where it was.  The read pointer sits just past it.
        RING = RINGS[0]
        wp = q.readl(WP_CELL)
        q.writel(RP_CELL, wp + 64)
        q.write(RING + wp, b'\xa5' * 64)
        q.writel(VEDU + 0x0008, 1)
        q.cmd('clock_step 5000000')
        raw = q.readl(VEDU + 0x3004)
        check(raw & 4, 'full ring: buffer full (0x%x)' % raw)
        check(q.readl(WP_CELL) == wp and q.read(RING + wp, 64) == b'\xa5' * 64,
              'full ring: nothing written')
        q.writel(VEDU + 0x0004, 0xffffffff)
        q.writel(RP_CELL, 0)

        # A packet claiming nearly the whole address space, and the driver
        # starting the ring over (a destroyed and recreated channel): the
        # model must neither allocate nor read past the ring, and must find
        # the new channel's parameter sets from offset 0.
        q.write(RING + wp, struct.pack('<II', 0xffffffc0, 0) + b'\0' * 56)
        q.writel(WP_CELL, 0)
        t, flags, nal = vedu_job(0, {}, H264_PS, 0)
        check(t == 5, 'recreated channel: an IDR again (type %d)' % t)

        # JPEG with no room: error and ring full, not done
        RING = RINGS[2]
        wp = q.readl(WP_CELL)
        q.writel(RP_CELL, wp + 64)
        q.writel(JPGE + 0x02c, 0)
        q.writel(JPGE + 0x02c, 1)
        q.cmd('clock_step 5000000')
        raw = q.readl(JPGE + 0x008)
        check(raw & 0x22 == 0x22 and not raw & 1,
              'full JPEG ring: error and ring full, not done (0x%x)' % raw)
        q.writel(JPGE + 0x00c, 0xffffffff)
    finally:
        q.close()

    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
