#!/usr/bin/env python3
"""Drive the xm-hevc HEVC engine models the way the vendor library and
hevc.ko do, and check that ffmpeg decodes what they produce.

No guest: QEMU runs under qtest and this script plays the driver, on each
engine layout (XM535's, XM533's, XM530V200's and XM232's).  The library writes the
parameter sets and slice headers itself, so the engine writes only the slice
data; the model's dump-dir keeps each job as a whole access unit with the
library's headers, and that stream is what ffmpeg decodes.

A GOP of three jobs: an IDR, a P picture of the same source (nearly every
block skipped: small, and nearly the same picture -- the encoder may code a
block again where that beats copying it), and a P picture with a square
added (the blocks around it coded again).  Each P job names the previous job's
reconstruction buffer as its reference, as the library does.

XM232's engine codes H.264 too (xm232-h264), under the library's CAVLC SPS,
PPS and slice header, which the script writes in front of the output as
Sofia's library does, the slice header's bits past its last whole byte in
+0x52C / +0x530; the stream ffmpeg decodes is those headers and the slice
data the guest finds at the output.  xm232-rc-h265 is an H.265 GOP whose
jobs say so as Sofia's do (rate control word +0x3C8 = 0x1E) while the
bytes before the output still hold an earlier H.264 stream's headers, as a
new stream's buffer allocated where an H.264 stream's was does: the model
must code H.265.  On XM232 the last job also takes a
snapshot (+0x5C8): a 4:2:2 JPEG scan under the quantisers given as
65536 / q at +0x1000, which must decode, behind the header the library
would write, to the picture.

    python3 qemu/tests/test-xm-hevc.py [--qemu PATH] [--machine M] [--keep DIR]
"""
import argparse, atexit, math, os, shutil, subprocess, sys, tempfile, time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

HEVC = 0x21000000
IN_ADDR, OUT_ADDR, BUF_SIZE = 0x81000000, 0x82000000, 0x80000
RECON = (0x83000000, 0x83400000)     # only names: the engine keeps the pictures

# the job registers of each layout (xm-hevc.c)
V1 = dict(start=0x000, start_val=1, ack=0x018, ack_val=1, status=0x074,
          w=0x044, h=0x048, sw=0x100, sh=0x104, type=0x054, qp=0x05C,
          free=0x088, y=0x0A0, c=0x0A4, ref=0x0A8, recon=0x0B0, out=0x0D0,
          ys=0x0D8, cs=0x0DC, len=0x0F8)
V3 = dict(start=0x004, start_val=1, ack=0x030, ack_val=3, status=0x058,
          w=0x034, h=0x038, type=0x040, qp=0x044, free=0x1D4, y=0x1E0,
          c=0x1E4, ref=0x1E8, recon=0x1F0, len=0x1D8)
X2 = dict(start=0x000, start_val=1, ack=0x028, ack_val=2, status=0x0B8,
          done=2, w=0x078, h=0x07C, type=0x088, qp=0x090, free=0x0D0,
          y=0x10C, c=0x110, ref=0x114, recon=0x11C, out=0x13C, ys=0x154,
          cs=0x158, len=0x174)
LAYOUTS = {
    'xm535': V1,
    'xm533': dict(V3, out=0x210, ys=0x224, cs=0x228),
    'xm530v200': dict(V3, out=0x200, ys=0x214, cs=0x218),
    'xm232': X2,
    'xm232-h264': dict(X2, h264=True),
    'xm232-rc-h265': dict(X2, stale_h264=True),
}
X2_HDR_BITS, X2_HDR_NBITS = 0x52C, 0x530
X2_RC_CODEC, X2_RC_H265 = 0x3C8, 0x1E
X2_JPEG_ON, X2_JPEG_FREE, X2_JPEG_OUT, X2_JPEG_LEN, X2_JPEG_QT = (
    0x5C8, 0x5D4, 0x5FC, 0x614, 0x1000)
JPEG_ADDR = 0x82800000


class Bits:
    """An RBSP writer, Exp-Golomb included."""
    def __init__(self):
        self.bits = []

    def u(self, v, n):
        self.bits += [(v >> (n - 1 - i)) & 1 for i in range(n)]

    def ue(self, v):
        v += 1
        n = v.bit_length()
        self.u(0, n - 1)
        self.u(v, n)

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def trailing(self):
        self.u(1, 1)
        while len(self.bits) % 8:
            self.u(0, 1)

    def whole(self):
        n = len(self.bits) // 8 * 8
        return bytes(int(''.join(map(str, self.bits[i:i + 8])), 2)
                     for i in range(0, n, 8))


def escape(rbsp):
    o, z = bytearray(), 0
    for b in rbsp:
        if z >= 2 and b <= 3:
            o.append(3)
            z = 0
        o.append(b)
        z = z + 1 if b == 0 else 0
    return bytes(o)


def h264_headers(w, h, f, qp):
    """Sofia's: Main profile, CAVLC, frame_num 10 bits, POC type 2; an IDR
    then P pictures, each with the slice header's last bits apart."""
    out = b''
    if f == 0:
        b = Bits()
        b.u(0x274d0033, 32)
        b.ue(0); b.ue(6); b.ue(2); b.ue(1); b.u(0, 1)
        b.ue(w // 16 - 1); b.ue(h // 16 - 1)
        b.u(1, 1); b.u(1, 1); b.u(0, 1); b.u(0, 1)
        b.trailing()
        out += b'\0\0\0\1' + escape(b.whole())
        b = Bits()
        b.u(0x28, 8)
        b.ue(0); b.ue(0); b.u(0, 1); b.u(0, 1); b.ue(0); b.ue(0); b.ue(0)
        b.u(0, 1); b.u(0, 2); b.se(0); b.se(0); b.se(0)
        b.u(1, 1); b.u(0, 1); b.u(0, 1)
        b.trailing()
        out += b'\0\0\0\1' + b.whole()
    b = Bits()
    b.u(0x25 if f == 0 else 0x21, 8)
    b.ue(0); b.ue(7 if f == 0 else 5); b.ue(0); b.u(f, 10)
    if f == 0:
        b.ue(0); b.u(0, 2)
    else:
        b.u(1, 1); b.ue(0); b.u(0, 1); b.u(0, 1)
    b.se(qp - 26); b.ue(0); b.se(-2); b.se(5)
    whole = b.whole()
    rest = b.bits[len(whole) * 8:]
    pend = int(''.join(map(str, rest)) + '0' * (8 - len(rest)), 2) if rest else 0
    return out + b'\0\0\0\1' + escape(whole), pend, len(rest)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-arm')
    ap.add_argument('--machine', action='append',
                    help='one of %s (default: all)' % ', '.join(LAYOUTS))
    ap.add_argument('--keep', help='leave each machine\'s stream in this dir')
    a = ap.parse_args()
    fails = 0
    for m in a.machine or LAYOUTS:
        fails += run(a, m, LAYOUTS[m])
    print('=== %s: xm-hevc ===' % ('PASS' if not fails else 'FAIL'))
    sys.exit(1 if fails else 0)


def psnr_plane(p, q_):
    mse = sum((x - y) ** 2 for x, y in zip(p, q_)) / len(p)
    return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)


def run(a, machine, L):
    w, h, qp = 640, 360, 30
    fails = 0
    h264 = L.get('h264', False)
    if h264:
        h = 352                     # whole macroblocks: no cropping
    machine_name = machine
    machine = machine.split('-')[0]

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + machine_name + ': ' + what)
        fails += not ok

    src = jt.test_frame(w, h)
    y0, uv0 = jt.to_nv12(src)
    # the second source: a white square at (128, 96), 64 x 64
    y1 = bytearray(y0)
    uv1 = bytearray(uv0)
    for yy in range(96, 160):
        y1[yy * w + 128:yy * w + 192] = b'\xeb' * 64
        if yy % 2 == 0:
            o = (yy // 2) * w + 128
            uv1[o:o + 64] = b'\x80' * 64
    sources = [(y0, uv0), (y0, uv0), (bytes(y1), bytes(uv1))]

    dump = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, dump, True)
    q = jt.QTest(a.qemu, machine, ('-global', 'xm-hevc.dump-dir=' + dump))
    sizes = []
    stream = b''
    try:
        for f, (yp, cp) in enumerate(sources):
            q.write(IN_ADDR, yp)
            q.write(IN_ADDR + w * h, cp)
            q.write(OUT_ADDR, b'\xa5' * 64)
            snap = machine == 'xm232' and f == 2
            if snap:
                lq, cq = jt.scaled(jt.LUMA_QT, 80), jt.scaled(jt.CHROMA_QT, 80)
                for t, tab in enumerate((lq, cq)):
                    for i in range(64):
                        q.writel(HEVC + X2_JPEG_QT + (t * 64 + i) * 4,
                                 round(65536 / tab[jt.ZIGZAG[i]]))
                q.writel(HEVC + X2_JPEG_OUT, JPEG_ADDR)
                q.writel(HEVC + X2_JPEG_FREE, BUF_SIZE)
                q.writel(HEVC + X2_JPEG_ON, 1)
            if h264:
                hdr, pend, nbits = h264_headers(w, h, f, qp)
                q.write(OUT_ADDR - len(hdr), hdr)
                q.writel(HEVC + X2_HDR_BITS, pend)
                q.writel(HEVC + X2_HDR_NBITS, nbits)
            if L.get('stale_h264'):
                # an H.265 job, as its rate control word says, over an
                # earlier H.264 stream's headers
                stale = h264_headers(w, h, f, qp)[0]
                q.write(OUT_ADDR - len(stale), stale)
                q.writel(HEVC + X2_HDR_NBITS, 0)
                q.writel(HEVC + X2_RC_CODEC, X2_RC_H265)
            regs = {'w': w - 1, 'h': h - 1, 'type': 1 if f else 0, 'qp': qp,
                    'free': BUF_SIZE, 'y': IN_ADDR, 'c': IN_ADDR + w * h,
                    'ref': RECON[(f + 1) % 2], 'recon': RECON[f % 2],
                    'out': OUT_ADDR, 'ys': w, 'cs': w}
            if 'sw' in L:
                regs.update(sw=w - 1, sh=h - 1)
            for k, v in regs.items():
                q.writel(HEVC + L[k], v)
            q.writel(HEVC + L['start'], L['start_val'])
            # the job starts a millisecond later and is encoded off the main
            # loop: step the clock until it says done
            st = 0
            for _ in range(2000):
                q.cmd('clock_step 1000000')
                st = q.readl(HEVC + L['status'])
                if st & 3 == L.get('done', 3):
                    break
                time.sleep(0.002)
            check(st & 3 == L.get('done', 3), 'job %d done (status 0x%x)' % (f, st))
            n = q.readl(HEVC + L['len'])
            check(0 < n < BUF_SIZE, 'job %d: %d bytes of slice data' % (f, n))
            sizes.append(n)
            data = q.read(OUT_ADDR, n)
            if snap:
                jn = q.readl(HEVC + X2_JPEG_LEN)
                check(0 < jn < BUF_SIZE, 'job %d: a %d-byte snapshot scan' % (f, jn))
                scan = q.read(JPEG_ADDR, jn)
                hdr422 = jt.jfif_header(w, h, lq, cq).replace(
                    bytes([3, 1, 0x22, 0]), bytes([3, 1, 0x21, 0]), 1)
                jpeg = hdr422 + scan + b'\xff\xd9'
                q.writel(HEVC + X2_JPEG_ON, 0)
            if h264:
                stream += hdr + data
            else:
                dumped = sorted(os.listdir(dump))
                au = open(os.path.join(dump, dumped[-1]), 'rb').read() if dumped else b''
                check(au.endswith(data), 'job %d: the guest\'s slice data ends the dumped '
                      'access unit' % f)
            q.writel(HEVC + L['ack'], L['ack_val'])
            if 'sw' in L:           # XM535's driver also clears start
                q.writel(HEVC + L['start'], 0)
    finally:
        q.close()

    # H.264's P picture also refines the IDR's DC-only macroblocks
    check(len(sizes) == 3 and sizes[1] * (2 if h264 else 10) < sizes[0],
          'an unchanged P picture is small (%s bytes)' % sizes)
    check(len(sizes) == 3 and sizes[1] < sizes[2] < sizes[0],
          'a P picture with a change costs more, still less than the IDR')

    if machine == 'xm232':
        import io
        from PIL import Image
        try:
            im = Image.open(io.BytesIO(jpeg))
            im.load()
            ok = im.size == (w, h)
        except Exception:           # noqa: BLE001
            ok, im = False, None
        check(ok, 'the snapshot decodes, %dx%d, 4:2:2' % (w, h))
        if im:
            yy = im.convert('YCbCr').split()[0].tobytes()
            v = psnr_plane(yy, sources[2][0])
            check(v > 30, 'snapshot luma: %.1f dB against the picture' % v)
    if not h264:
        stream = b''.join(open(os.path.join(dump, f), 'rb').read()
                          for f in sorted(os.listdir(dump)))
    ext = 'h264' if h264 else 'hevc'
    if a.keep:
        os.makedirs(a.keep, exist_ok=True)
        open(os.path.join(a.keep, machine_name + '.' + ext), 'wb').write(stream)
    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'out.' + ext)
    open(path, 'wb').write(stream)
    r = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode',
                        '-i', path, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                       capture_output=True)
    check(r.returncode == 0 and not r.stderr,
          'ffmpeg decodes it cleanly%s' % (': ' + r.stderr.decode()[-300:] if r.stderr else ''))
    fsz = w * h * 3 // 2
    frames = [r.stdout[i * fsz:(i + 1) * fsz] for i in range(len(r.stdout) // fsz)]
    check(len(frames) == 3, 'ffmpeg returns %d of 3 frames' % len(frames))
    if len(frames) != 3:
        return fails

    # DC-only 8x8 blocks: judge the luma against the source averaged the same way
    def blocks(plane):
        ref = bytearray(w * h)
        for by in range(0, h, 8):
            for bx in range(0, w, 8):
                s = sum(plane[(by + j) * w + bx + i] for j in range(8) for i in range(8))
                for j in range(8):
                    ref[(by + j) * w + bx:(by + j) * w + bx + 8] = bytes([(s + 32) // 64]) * 8
        return ref

    def psnr(p, q_):
        mse = sum((x - y) ** 2 for x, y in zip(p, q_)) / len(p)
        return 99.0 if mse == 0 else 10 * math.log10(255 * 255 / mse)

    v = psnr(frames[1], frames[0])
    check(v > 40, 'the unchanged P picture is the IDR\'s: %.1f dB' % v)
    for f in (0, 2):
        # H.264: I_16x16 DC macroblocks, deblocked; HEVC: 8x8 DC blocks
        v = psnr(frames[f][:w * h], blocks(sources[f][0]))
        check(v > (20 if h264 else 25),
              'frame %d luma: %.1f dB against the 8x8 block means' % (f, v))
    sq = frames[2][110 * w + 140]
    check(sq > 200, 'the square is there in the last picture (luma %d)' % sq)
    return fails


if __name__ == '__main__':
    main()
