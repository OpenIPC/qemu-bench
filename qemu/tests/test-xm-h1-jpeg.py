#!/usr/bin/env python3
"""Drive the xm-h1 Hantro H1 model in JPEG mode the way mainline Linux does,
and check that what it produces decodes to the frame it was given.

No guest: QEMU runs under qtest (-machine accel=qtest) and this script plays
the driver.  The register sequence is hantro_h1_jpeg_enc_run() from
drivers/media/platform/verisilicon/hantro_h1_jpeg_enc.c; the JFIF header in
front of the hardware's scan is assembled the way hantro_jpeg.c does it
(quality-scaled Annex K quantisers, the hardware copy in hw_reorder order),
except that the Huffman tables are taken from a JPEG libjpeg (PIL) writes,
an independent copy of the Annex K tables the model encodes with.

    python3 qemu/tests/test-xm-h1-jpeg.py [--qemu PATH] [--keep OUT.jpg]
"""
import argparse, atexit, io, math, os, shutil, socket, subprocess, sys, tempfile, time

from PIL import Image

H1 = 0x20070000
REG = dict(INTERRUPT=0x004, AXI_CTRL=0x008, OUT=0x014, IN0=0x02c, IN1=0x030,
           IN2=0x034, ENC_CTRL=0x038, IN_IMG_CTRL=0x03c, STR_BUF_LIMIT=0x060,
           LUMA_Q=0x100, CHROMA_Q=0x140, ID=0x000)
INT_FRAME_RDY, ENC_EN, MODE_JPEG, PIC_INTRA = 1 << 2, 1, 2 << 1, 1 << 3
FMT_YUV420SP = 1                      # hantro_enc_fmt: 0 planar, 1 semi-planar

IN_ADDR, OUT_ADDR, OUT_SIZE = 0x81000000, 0x82000000, 0x100000

# T.81 Annex K.1, natural order, as v4l2_jpeg_ref_table_{luma,chroma}_qt
LUMA_QT = [16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
           14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
           18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92,
           49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99]
CHROMA_QT = [17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
             24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99] + [99] * 32
ZIGZAG = [0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
          12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
          35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
          58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]
HW_REORDER = [0, 8, 16, 24, 1, 9, 17, 25, 32, 40, 48, 56, 33, 41, 49, 57,
              2, 10, 18, 26, 3, 11, 19, 27, 34, 42, 50, 58, 35, 43, 51, 59,
              4, 12, 20, 28, 5, 13, 21, 29, 36, 44, 52, 60, 37, 45, 53, 61,
              6, 14, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63]


def scaled(tab, quality):
    """hantro_jpeg.c jpeg_set_quality() / jpeg_scale_qp()."""
    scale = 5000 // quality if quality < 50 else 200 - 2 * quality
    return [min(255, max(1, (q * scale + 50) // 100)) for q in tab]


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX); srv.bind(path); srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest', '-display', 'none',
             '-nodefaults', '-qtest', 'unix:' + path, '-serial', 'null', *extra],
            stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n'); self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith('OK') or r.startswith('FAIL') or r.startswith('ERR'):
                return r

    def writel(self, a, v): self.cmd('writel 0x%x 0x%x' % (a, v))
    def readl(self, a): return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        for o in range(0, len(data), 4096):
            chunk = data[o:o + 4096]
            self.cmd('write 0x%x 0x%x 0x%s' % (a + o, len(chunk), chunk.hex()))

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n)).split()[1][2:])

    def close(self):
        self.proc.kill(); self.proc.wait()
        # QEMU's stderr outlives the dir: test-xm-gpio and test-xm510-gpio
        # check its -d unimp lines after close().
        try:
            self.err = open(os.path.join(self.dir, 'qemu.err')).read()
        except OSError:
            self.err = ''
        shutil.rmtree(self.dir, ignore_errors=True)


def test_frame(w, h):
    """Colour bars over a vertical gradient, with a sharp diagonal edge."""
    bars = [(235, 235, 235), (235, 235, 16), (16, 235, 235), (16, 235, 16),
            (235, 16, 235), (235, 16, 16), (16, 16, 235), (16, 16, 16)]
    img = Image.new('RGB', (w, h))
    px = img.load()
    for y in range(h):
        for x in range(w):
            r, g, b = bars[x * 8 // w]
            k = 0.55 + 0.45 * y / (h - 1)
            px[x, y] = (int(r * k), int(g * k), int(b * k)) if x > y else (128, 64, 32)
    return img


def to_nv12(img):
    w, h = img.size
    ycc = img.convert('YCbCr')
    yy, cb, cr = [p.load() for p in ycc.split()]
    y_plane = bytes(yy[x, y] for y in range(h) for x in range(w))
    uv = bytearray()
    for y in range(0, h, 2):
        for x in range(0, w, 2):
            uv.append((cb[x, y] + cb[x + 1, y] + cb[x, y + 1] + cb[x + 1, y + 1] + 2) // 4)
            uv.append((cr[x, y] + cr[x + 1, y] + cr[x, y + 1] + cr[x + 1, y + 1] + 2) // 4)
    return y_plane, bytes(uv)


def libjpeg_dht():
    """The DHT segments libjpeg writes: its own copy of the Annex K tables."""
    buf = io.BytesIO()
    Image.new('RGB', (16, 16)).save(buf, 'JPEG', quality=90, optimize=False)
    d = buf.getvalue(); out = b''; i = 2
    while i < len(d):
        marker, ln = d[i + 1], int.from_bytes(d[i + 2:i + 4], 'big')
        if marker == 0xc4:
            out += d[i:i + 2 + ln]
        if marker == 0xda:
            break
        i += 2 + ln
    return out


def jfif_header(w, h, lq, cq):
    seg = lambda m, body: bytes([0xff, m]) + (len(body) + 2).to_bytes(2, 'big') + body
    dqt = seg(0xdb, bytes([0]) + bytes(lq[z] for z in ZIGZAG) +
              bytes([1]) + bytes(cq[z] for z in ZIGZAG))
    sof = seg(0xc0, bytes([8]) + h.to_bytes(2, 'big') + w.to_bytes(2, 'big') +
              bytes([3, 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1]))
    sos = seg(0xda, bytes([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0]))
    return b'\xff\xd8' + dqt + sof + libjpeg_dht() + sos


def psnr(a, b):
    pa, pb = a.convert('RGB').tobytes(), b.convert('RGB').tobytes()
    mse = sum((x - y) ** 2 for x, y in zip(pa, pb)) / len(pa)
    return 99.0 if mse == 0 else 10 * math.log10(255 ** 2 / mse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--machine', default='xm530')
    ap.add_argument('--keep')
    ap.add_argument('--width', type=int, default=176)
    ap.add_argument('--height', type=int, default=144)
    a = ap.parse_args()
    w, h, quality = a.width, a.height, 90
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    src = test_frame(w, h)
    y_plane, uv = to_nv12(src)
    lq, cq = scaled(LUMA_QT, quality), scaled(CHROMA_QT, quality)
    hw_l = bytes(lq[i] for i in HW_REORDER)
    hw_c = bytes(cq[i] for i in HW_REORDER)

    dump = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, dump, True)
    q = QTest(a.qemu, a.machine, ('-global', 'xm-h1.dump-dir=' + dump))
    try:
        check(q.readl(H1 + REG['ID']) >> 16 == 0x8290, 'swreg0 reports an H1 (0x8290)')
        q.write(IN_ADDR, y_plane)
        q.write(IN_ADDR + w * h, uv)
        # hantro_h1_jpeg_enc_run()
        q.writel(H1 + REG['ENC_CTRL'], MODE_JPEG)
        q.writel(H1 + REG['IN_IMG_CTRL'], (w << 12) | (FMT_YUV420SP << 2))
        q.writel(H1 + REG['OUT'], OUT_ADDR)
        q.writel(H1 + REG['STR_BUF_LIMIT'], OUT_SIZE)
        q.writel(H1 + REG['IN0'], IN_ADDR)
        q.writel(H1 + REG['IN1'], IN_ADDR + w * h)
        for i in range(16):
            q.writel(H1 + REG['LUMA_Q'] + 4 * i, int.from_bytes(hw_l[4 * i:4 * i + 4], 'big'))
        for i in range(16):
            q.writel(H1 + REG['CHROMA_Q'] + 4 * i, int.from_bytes(hw_c[4 * i:4 * i + 4], 'big'))
        q.writel(H1 + REG['AXI_CTRL'], (1 << 15) | (1 << 14) | (16 << 8) | 0xf)
        q.writel(H1 + REG['ENC_CTRL'], ((w // 16) << 19) | ((h // 16) << 10) |
                 MODE_JPEG | PIC_INTRA | ENC_EN)

        irq = q.readl(H1 + REG['INTERRUPT'])
        check(bool(irq & INT_FRAME_RDY), 'INTERRUPT has FRAME_RDY (0x%x)' % irq)
        check(not q.readl(H1 + REG['ENC_CTRL']) & ENC_EN, 'ENC_CTRL enable bit cleared')
        n = q.readl(H1 + REG['STR_BUF_LIMIT']) // 8   # hantro_h1_jpeg_enc_done()
        check(0 < n < OUT_SIZE, 'STR_BUF_LIMIT reports %d scan bytes' % n)
        scan = q.read(OUT_ADDR, n)
        check(scan.endswith(b'\xff\xd9'), 'scan ends with EOI')
    finally:
        q.close()

    jpg = jfif_header(w, h, lq, cq) + scan
    if a.keep:
        open(a.keep, 'wb').write(jpg)
    try:
        out = Image.open(io.BytesIO(jpg)); out.load()
        check(out.size == (w, h), 'decodes as %dx%d' % out.size)
        # Judge the encoder against libjpeg doing the same job (quality,
        # 4:2:0): the source-frame PSNR is bounded by chroma subsampling, so
        # an absolute threshold would say more about the frame than the model.
        buf = io.BytesIO(); src.save(buf, 'JPEG', quality=quality, subsampling=2)
        ref = Image.open(io.BytesIO(buf.getvalue()))
        p_src, p_ref = psnr(src, out), psnr(ref, out)
        check(p_src >= psnr(src, ref) - 1.0,
              'PSNR against the source %.1f dB, libjpeg gets %.1f' % (p_src, psnr(src, ref)))
        check(p_ref > 40, 'within %.1f dB of libjpeg\'s own encode (> 40)' % p_ref)
        # dump-dir: the model's own JFIF of the same job
        dumped = sorted(f for f in os.listdir(dump) if f.endswith('.jpg'))
        check(len(dumped) == 1, 'dump-dir holds the job as %s' % (dumped or 'nothing'))
        if dumped:
            d = Image.open(os.path.join(dump, dumped[0])); d.load()
            check(d.tobytes() == out.tobytes(), 'the dumped JPEG decodes to the same image')
        check(any(f.endswith('.regs') for f in os.listdir(dump)), 'dump-dir holds the job\'s registers')
    except Exception as e:                  # noqa: BLE001 -- report and fail
        check(False, 'decodes: %s' % e)

    print('=== %s: H1 JPEG ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
