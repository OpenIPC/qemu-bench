#!/usr/bin/env python3
"""Drive the xm-h1 Hantro H1 model in H.264 mode the way Hantro's encoder
library does, and check that ffmpeg decodes what it produces.

The library writes the SPS and PPS itself, up to an 8-byte aligned output
pointer, and hands the H1 the header bits that did not fit (STR_HDR_REM_*,
counted in RLC_CTRL[28:23]).  The headers below are the ones majestic's XM530
backend writes (Main profile, CABAC); the model has to fit its slices to them.

    python3 qemu/tests/test-xm-h1-h264.py [--qemu PATH] [--keep OUT.h264]
"""
import argparse, atexit, math, os, shutil, subprocess, sys, tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
jt = import_module('test-xm-h1-jpeg')

H1 = jt.H1
REG = dict(jt.REG, ENC_CTRL0=0x040, HDR_REM_MSB=0x058, HDR_REM_LSB=0x05c,
           RLC_CTRL=0x094)
ENC_EN, MODE_H264, PIC_INTRA = 1, 3 << 1, 1 << 3
INT_ERROR, INT_BUF_FULL = 1 << 3, 1 << 5    # Hantro's ASIC_STATUS_ERROR / _BUFF_FULL

# SPS and PPS as the library leaves them: the PPS's last byte is the remainder
HEADERS = bytes.fromhex('00000001274d001f8d6a0b012684000003000400000300ca10'
                        '0000000128ee0b')
REMAINDER = 0xc8
IN_ADDR, BUF_ADDR, BUF_SIZE = 0x81000000, 0x82000000, 0x100000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='xm530')
    ap.add_argument('--keep')
    ap.add_argument('--frames', type=int, default=4)
    a = ap.parse_args()
    w, h, qp = 704, 576, 28           # the SPS's size
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    src = jt.test_frame(w, h)
    y_plane, uv = jt.to_nv12(src)
    dump = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, dump, True)
    q = jt.QTest(a.qemu, a.machine, ('-global', 'xm-h1.dump-dir=' + dump))
    stream = b''
    try:
        q.write(IN_ADDR, y_plane)
        q.write(IN_ADDR + w * h, uv)
        for f in range(a.frames):
            # a GOP of 2, and a QP that moves at the second IDR, the way
            # majestic's rate control does it
            idr = f % 2 == 0
            qp = 24 if f < 2 else 28
            pre = HEADERS if idr else b''
            q.write(BUF_ADDR, pre)
            out = BUF_ADDR + len(pre)
            q.writel(H1 + REG['IN_IMG_CTRL'], (w << 12) | (jt.FMT_YUV420SP << 2))
            q.writel(H1 + REG['OUT'], out)
            q.writel(H1 + REG['STR_BUF_LIMIT'], BUF_SIZE - len(pre))
            q.writel(H1 + REG['IN0'], IN_ADDR)
            q.writel(H1 + REG['IN1'], IN_ADDR + w * h)
            q.writel(H1 + REG['HDR_REM_MSB'], REMAINDER << 24 if idr else 0)
            q.writel(H1 + REG['HDR_REM_LSB'], 0)
            q.writel(H1 + REG['RLC_CTRL'], (8 if idr else 0) << 23)
            q.writel(H1 + REG['ENC_CTRL0'], qp << 26)
            q.writel(H1 + REG['ENC_CTRL'], ((w // 16) << 19) | ((h // 16) << 10) |
                     MODE_H264 | (PIC_INTRA if idr else 0) | ENC_EN)
            irq = q.readl(H1 + REG['INTERRUPT'])
            check(bool(irq & jt.INT_FRAME_RDY), 'frame %d: FRAME_RDY (0x%x)' % (f, irq))
            n = q.readl(H1 + REG['STR_BUF_LIMIT']) // 8
            check(0 < n < BUF_SIZE, 'frame %d: %d stream bytes' % (f, n))
            stream += pre + q.read(out, n)
            q.writel(H1 + REG['INTERRUPT'], 0)

        # a job that cannot be done is not reported as a frame
        def bad_job(what, limit, row, expect):
            q.write(BUF_ADDR, b'\xa5' * 64)
            q.writel(H1 + REG['IN_IMG_CTRL'], (row << 12) | (jt.FMT_YUV420SP << 2))
            q.writel(H1 + REG['OUT'], BUF_ADDR)
            q.writel(H1 + REG['STR_BUF_LIMIT'], limit)
            q.writel(H1 + REG['RLC_CTRL'], 0)
            q.writel(H1 + REG['ENC_CTRL'], ((w // 16) << 19) | ((h // 16) << 10) |
                     MODE_H264 | ENC_EN)
            irq = q.readl(H1 + REG['INTERRUPT'])
            check(irq & expect and not irq & jt.INT_FRAME_RDY,
                  '%s: 0x%x, not FRAME_RDY' % (what, irq))
            check(q.read(BUF_ADDR, 64) == b'\xa5' * 64, '%s: nothing written' % what)
            q.writel(H1 + REG['INTERRUPT'], 0)

        bad_job('a 64-byte stream buffer', 64, w, INT_BUF_FULL)
        bad_job('a row shorter than the picture', BUF_SIZE, w - 16, INT_ERROR)
    finally:
        q.close()

    if a.keep:
        open(a.keep, 'wb').write(stream)
    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'out.h264')
    open(path, 'wb').write(stream)
    r = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode',
                        '-i', path, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                       capture_output=True)
    check(r.returncode == 0 and not r.stderr,
          'ffmpeg decodes it cleanly%s' % (': ' + r.stderr.decode()[-300:] if r.stderr else ''))
    fsz = w * h * 3 // 2                  # yuv420p, so the luma is untouched
    frames = len(r.stdout) // fsz
    check(frames == a.frames, 'ffmpeg returns %d of %d frames' % (frames, a.frames))
    # DC-only coding reproduces each plane at 4x4-block resolution: judge it
    # against the source averaged the same way
    def blocks(plane, pw, ph):
        ref = bytearray(pw * ph)
        for by in range(0, ph, 4):
            for bx in range(0, pw, 4):
                s = sum(plane[(by + j) * pw + bx + i] for j in range(4) for i in range(4))
                for j in range(4):
                    for i in range(4):
                        ref[(by + j) * pw + bx + i] = (s + 8) // 16
        return ref

    def psnr(a, b):
        mse = sum((x - y) ** 2 for x, y in zip(a, b)) / len(a)
        return 99.0 if mse == 0 else 10 * math.log10(255 ** 2 / mse)

    cw, ch = w // 2, h // 2
    refs = [blocks(y_plane, w, h), blocks(uv[0::2], cw, ch), blocks(uv[1::2], cw, ch)]
    for f in range(frames):
        dec = r.stdout[f * fsz:(f + 1) * fsz]
        planes = [dec[:w * h], dec[w * h:w * h + cw * ch], dec[w * h + cw * ch:]]
        p = [psnr(d, ref) for d, ref in zip(planes, refs)]
        check(min(p) > 45, 'frame %d PSNR Y %.1f / Cb %.1f / Cr %.1f dB against the '
              '4x4-averaged source (> 45)' % (f, *p))
    dumped = sorted(x for x in os.listdir(dump) if x.endswith('.h264'))
    check(len(dumped) == a.frames, 'dump-dir holds %d .h264 jobs' % len(dumped))
    # an IDR job's dump carries the SPS/PPS ahead of it: the files alone
    # concatenate into the same stream
    cat = b''.join(open(os.path.join(dump, x), 'rb').read() for x in dumped)
    check(cat == stream, 'the dumped jobs concatenate into the stream')

    print('=== %s: H1 H.264 ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
