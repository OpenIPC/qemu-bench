#!/usr/bin/env python3
"""Drive the ak-h8290 model (AK3918EV200's Hantro 8290) the way OpenIPC's
ak-venc driver does, and check that what it produces decodes to the input.

No guest: QEMU runs the ak3918ev200 machine under qtest and this script
plays the driver (drivers/media/platform/anyka/ak-venc*.c, branch
anyka-ak3918ev300 of OpenIPC's kernel):

  - the probe: swreg0 is 0x82900760;
  - H.264 (ak_venc_program + akcam_h8290_write_frame): an IDR with the
    SPS / PPS the driver writes into the CAPTURE buffer, the stream base on
    the header's last whole 64-bit word and the rest in swreg22/23/37, then
    P frames with frame_num counting from 1, the two frame stores rotating
    (akcam_h8290_frames_advance), a second IDR with idr_pic_id 1, and a P
    frame after it; registers swreg1..0x3e with the enable clear, then
    swreg14 with it set;
  - completion: not inside the start write, but after virtual time passes;
    swreg1 then has the line and FRAME_READY, the driver's ack drops the
    line, swreg24 is the bits from the base (payload = header whole words +
    those bits), the MB count and QP sum are there for the rate control;
  - every slice NAL carries its own start code, and its header says the
    frame_num / idr_pic_id / slice type / QP that were programmed;
  - ffmpeg decodes the stream with -err_detect explode, every picture,
    and each is close to the input (a moving square over colour bars, so
    the P frames have motion to code; 320x184, so the core codes 192 rows
    and the SPS crops 8);
  - JPEG (ak_venc_program_jpeg): the driver's JFIF header up to SOS, the
    quantisers in the core's load order, the core's scan and EOI after it;
    the file decodes at 320x184 and resembles the input;
  - a stop (swreg14 written with the enable clear) before completion
    cancels the job: no interrupt;
  - dump-dir keeps the jobs.

    python3 qemu/tests/test-ak-h8290.py [--qemu PATH] [--keep OUT.h264]
"""
import argparse, atexit, io, math, os, shutil, subprocess, sys, tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest  # noqa: E402

VENC = 0x20020000
DDR = 0x80000000
IN_ADDR = DDR + 0x01000000          # OUTPUT buffers (I420)
OUT_ADDR = DDR + 0x01400000         # CAPTURE buffers
OUT_SIZE = 0x40000
PRIV = DDR + 0x01800000             # frame stores + NAL table + CABAC
W, H = 320, 184
QP = 28

# ak-venc-regs.h: name -> (swreg, shift, mask), the fields the driver sets
F = dict(
    BURST_LENGTH=(2, 8, 0x3f00), CLOCK_GATING=(2, 4, 0x10),
    IN_SWAP8=(2, 0, 1), IN_SWAP16=(2, 14, 0x4000), IN_SWAP32=(2, 2, 4),
    OUT_SWAP8=(2, 1, 2), OUT_SWAP16=(2, 15, 0x8000), OUT_SWAP32=(2, 3, 8),
    OUT_BASE=(5, 0, 0xffffffff), CTRL_BASE=(6, 0, 0xffffffff),
    REF_Y=(7, 0, 0xffffffff), REF_C=(8, 0, 0xffffffff),
    REC_Y=(9, 0, 0xffffffff), REC_C=(10, 0, 0xffffffff),
    IN_Y=(11, 0, 0xffffffff), IN_CB=(12, 0, 0xffffffff), IN_CR=(13, 0, 0xffffffff),
    TIMEOUT_IRQ=(14, 31, 1 << 31), NAL_SIZES=(14, 29, 1 << 29),
    WIDTH=(14, 19, 0x0ff80000), HEIGHT=(14, 10, 0x0007fc00),
    PIC_TYPE=(14, 3, 0x18), MODE=(14, 1, 6), ENABLE=(14, 0, 1),
    ROW=(15, 12, 0x03fff000), XFILL=(15, 10, 0xc00), YFILL=(15, 6, 0x3c0),
    IN_FMT=(15, 2, 0x3c), ROT=(15, 0, 3),
    PPS_QP=(16, 26, 0xfc000000), ALPHA=(16, 22, 0x03c00000),
    BETA=(16, 18, 0x003c0000), CHROMA_QP=(16, 13, 0x3e000),
    DEBLOCK=(16, 5, 0x60), IDR_PIC_ID=(16, 1, 0x1e), CIP=(16, 0, 1),
    PPS_ID=(17, 24, 0xff000000), I4X4_FAVOR=(17, 16, 0xff0000),
    I16_FAVOR=(17, 0, 0xffff),
    SLICE_SIZE=(18, 23, 0x3f800000), NO_QPEL=(18, 22, 0x400000),
    T8X8=(18, 21, 0x200000), CABAC_IDC=(18, 19, 0x180000),
    CABAC=(18, 18, 0x40000), INTER4X4=(18, 17, 0x20000),
    STREAM_MODE=(18, 16, 0x10000), FRAME_NUM=(18, 0, 0xffff),
    JPEG_MODE=(20, 25, 1 << 25), JPEG_SLICE=(20, 24, 1 << 24),
    JPEG_RST=(20, 16, 0xff0000),
    SKIP_PENALTY=(21, 24, 0xff000000), INTER_FAVOR=(21, 0, 0xffff),
    REM_MSB=(22, 0, 0xffffffff), REM_LSB=(23, 0, 0xffffffff),
    LIMIT=(24, 0, 0xffffffff),
    MAD_QP=(25, 28, 0xf0000000), MAD_THR=(25, 22, 0x0fc00000),
    QP_SUM=(25, 0, 0x1fffff),
    INIT_QP=(27, 26, 0xfc000000), MAX_QP=(27, 20, 0x03f00000),
    MIN_QP=(27, 14, 0xfc000), CP_DIST=(27, 0, 0x1fff),
    REM_BITS=(37, 23, 0x1f800000), RLC=(37, 0, 0x7fffff),
    MAD_UNDER=(38, 16, 0xffff0000), MB_COUNT=(38, 0, 0xffff),
    CABAC_BASE=(51, 0, 0xffffffff),
    INTRA_AREA=(56, 0, 0xffffffff), ROI2=(61, 0, 0xffffffff),
    MVC_VIEW=(62, 13, 0xe000), MVC_ANCHOR=(62, 9, 0x200),
)
IRQ_LINE, IRQ_FRAME_RDY, IRQ_BUS_ERR, IRQ_BUF_FULL = 1, 1 << 2, 1 << 3, 1 << 5


class Regs:
    def __init__(self):
        self.r = [0] * 128

    def set(self, name, v):
        reg, sh, m = F[name]
        self.r[reg] = (self.r[reg] & ~m | ((v << sh) & m)) & 0xffffffff

    def write_frame(self, q):
        """akcam_h8290_write_frame(): 1..0x3e with the enable clear, the
        quantisers for JPEG (each write read back), then swreg14."""
        for n in range(1, 0x3f):
            q.writel(VENC + 4 * n, self.r[n] & ~1 if n == 14 else self.r[n])
        if (self.r[14] >> 1) & 3 == 2:
            for n in range(64, 96):
                q.writel(VENC + 4 * n, self.r[n])
                q.readl(VENC + 4 * n)
        q.writel(VENC + 14 * 4, self.r[14] | 1)


def field(v, name):
    reg, sh, m = F[name]
    return (v & m) >> sh


# ---- ak-venc-h264.c: the driver's SPS / PPS ----------------------------

class Bits:
    def __init__(self):
        self.bits = []

    def put(self, n, v):
        self.bits += [(v >> (n - 1 - i)) & 1 for i in range(n)]

    def ue(self, v):
        v += 1
        n = v.bit_length()
        self.put(n - 1, 0)
        self.put(n, v)

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def trailing(self):
        self.put(1, 1)
        while len(self.bits) % 8:
            self.put(1, 0)

    def bytes(self):
        return bytes(int(''.join(map(str, self.bits[i:i + 8])), 2)
                     for i in range(0, len(self.bits), 8))


def escape(rbsp):
    out, zeros = bytearray(), 0
    for b in rbsp:
        if zeros >= 2 and b <= 3:
            out.append(3)
            zeros = 0
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


def headers(w, h, qp):
    mbw, mbh = (w + 15) // 16, (h + 15) // 16
    s = Bits()
    s.put(8, 66)                        # Baseline
    s.put(1, 1); s.put(1, 1); s.put(1, 1); s.put(1, 0)  # constrained
    s.put(4, 0)
    s.put(8, 31)
    s.ue(0)                             # sps id
    s.ue(12)                            # log2_max_frame_num - 4: 16 bits
    s.ue(2)                             # POC type 2
    s.ue(1)                             # num_ref_frames
    s.put(1, 0)
    s.ue(mbw - 1); s.ue(mbh - 1)
    s.put(1, 1); s.put(1, 1)            # frame_mbs_only, direct_8x8
    cr, cb = (mbw * 16 - w) // 2, (mbh * 16 - h) // 2
    s.put(1, 1 if cr or cb else 0)
    if cr or cb:
        s.ue(0); s.ue(cr); s.ue(0); s.ue(cb)
    s.put(1, 1)                         # VUI: bitstream_restriction only
    s.put(1, 0); s.put(1, 0); s.put(1, 0); s.put(1, 0); s.put(1, 0)
    s.put(1, 0); s.put(1, 0); s.put(1, 0); s.put(1, 1)
    s.put(1, 1); s.ue(0); s.ue(0); s.ue(9); s.ue(7); s.ue(0); s.ue(1)
    s.trailing()
    p = Bits()
    p.ue(0); p.ue(0); p.put(1, 0); p.put(1, 0); p.ue(0); p.ue(0); p.ue(0)
    p.put(1, 0); p.put(2, 0)
    p.se(qp - 26); p.se(0); p.se(2)     # chroma_qp_index_offset 2
    p.put(1, 1); p.put(1, 0); p.put(1, 0)
    p.trailing()
    return (b'\0\0\0\1\x67' + escape(s.bytes()) +
            b'\0\0\0\1\x68' + escape(p.bytes()))


def remainder(hdr):
    """ak_venc_hdr_remainder(): the header's last partial 64-bit word."""
    n = len(hdr) & 7
    if not n:
        return 0, 0, 0
    word = hdr[len(hdr) & ~7:] + bytes(8 - n)
    return n * 8, int.from_bytes(word[:4], 'big'), int.from_bytes(word[4:], 'big')


def config_h264(r, intra, frame_num, idr_pic_id, pps_qp, qp):
    """akcam_h8290_config_h264(), the values ak_venc_program() passes."""
    mbw, mbh = (W + 15) // 16, (H + 15) // 16
    for n in ('IN_SWAP8', 'IN_SWAP16', 'IN_SWAP32', 'OUT_SWAP8', 'OUT_SWAP16',
              'OUT_SWAP32', 'CLOCK_GATING', 'NAL_SIZES', 'TIMEOUT_IRQ',
              'NO_QPEL', 'INTER4X4', 'MVC_ANCHOR'):
        r.set(n, 1)
    r.set('BURST_LENGTH', 16)
    r.set('MODE', 3)
    r.set('PIC_TYPE', 1 if intra else 0)
    r.set('WIDTH', mbw); r.set('HEIGHT', mbh)
    r.set('ROW', W)
    r.set('XFILL', (mbw * 16 - W) // 4); r.set('YFILL', mbh * 16 - H)
    r.set('IN_FMT', 0)
    r.set('PPS_QP', pps_qp)
    r.set('CHROMA_QP', 2)
    r.set('DEBLOCK', 0)
    r.set('PPS_ID', 0)
    r.set('FRAME_NUM', frame_num)
    r.set('IDR_PIC_ID', idr_pic_id)
    r.set('INTER_FAVOR', 335); r.set('I16_FAVOR', 3335)
    r.set('INIT_QP', qp); r.set('MAX_QP', 51); r.set('MIN_QP', 20)
    r.set('MAD_QP', -2 & 0xf); r.set('MAD_THR', 12)
    r.set('INTRA_AREA', (mbw << 24) | (mbw << 16) | (mbh << 8) | mbh)
    r.set('ROI2', (mbw << 24) | (mbw << 16) | (mbh << 8) | mbh)
    r.set('MVC_VIEW', 1)


# ---- ak-venc-jpeg.c: the driver's JFIF header ---------------------------

BASE_LUMA = [16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55,
             14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62,
             18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92,
             49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99]
BASE_CHROMA = [17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
               24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99] + [99] * 32
ZIGZAG = [0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
          12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
          35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
          58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]
DC_L = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
DC_C = [0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
AC_L = [0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d]
AC_C = [0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77]
AC_L_V = bytes.fromhex(
    '01020300041105122131410613516107227114328191a1082342b1c11552d1f0'
    '2433627282090a161718191a25262728292a3435363738393a43444546474849'
    '4a535455565758595a636465666768696a737475767778797a83848586878889'
    '8a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5'
    'c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa')
AC_C_V = bytes.fromhex(
    '000102031104052131061241510761711322328108144291a1b1c109233352f0'
    '156272d10a162434e125f11718191a262728292a35363738393a434445464748'
    '494a535455565758595a636465666768696a737475767778797a828384858687'
    '88898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3'
    'c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9fa')


def quant_round(v):
    v = max(1, min(255, v))
    if v > 128:
        return v & ~7
    if v > 64:
        return v & ~3
    if v > 32:
        return v & ~1
    return v


def quant_tables(quality):
    scale = 5000 // quality if quality < 50 else 200 - 2 * quality
    return ([quant_round((q * scale + 50) // 100) for q in BASE_LUMA],
            [quant_round((q * scale + 50) // 100) for q in BASE_CHROMA])


def jpeg_header(w, h, lq, cq):
    seg = lambda m, body: bytes([0xff, m]) + (len(body) + 2).to_bytes(2, 'big') + body
    app0 = seg(0xe0, b'JFIF\0\x01\x01\0\0\x01\0\x01\0\0')
    dqt = seg(0xdb, bytes([0]) + bytes(lq[z] for z in ZIGZAG) +
              bytes([1]) + bytes(cq[z] for z in ZIGZAG))
    sof = seg(0xc0, bytes([8]) + h.to_bytes(2, 'big') + w.to_bytes(2, 'big') +
              bytes([3, 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1]))
    dht = seg(0xc4, bytes([0]) + bytes(DC_L) + bytes(range(12)) +
              bytes([0x10]) + bytes(AC_L) + AC_L_V +
              bytes([1]) + bytes(DC_C) + bytes(range(12)) +
              bytes([0x11]) + bytes(AC_C) + AC_C_V)
    sos = seg(0xda, bytes([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0]))
    return b'\xff\xd8' + app0 + dqt + sof + dht + sos


def quant_apply(r, lq, cq):
    """akcam_h8290_quant_apply(): the core's load order, MSB first."""
    slot = [0] * 128
    for row in range(8):
        for col in range(8):
            i = 4 * (4 * (col >> 1) + 2 * (row >> 2) + (col & 1)) + (row & 3)
            slot[i] = lq[row * 8 + col]
            slot[64 + i] = cq[row * 8 + col]
    for n in range(32):
        r.r[64 + n] = int.from_bytes(bytes(slot[4 * n:4 * n + 4]), 'big')


# ---- the pictures ------------------------------------------------------

def picture(t):
    """I420: colour bars over a gradient, a square moving 8 px a frame."""
    y = bytearray(W * H)
    cb = bytearray((W // 2) * (H // 2))
    cr = bytearray((W // 2) * (H // 2))
    bars = [(180, 128, 128), (162, 44, 142), (131, 156, 44), (112, 72, 58),
            (84, 184, 198), (65, 100, 212), (35, 212, 114), (16, 128, 128)]
    sx, sy = 24 + 8 * t, 40 + 4 * t
    for j in range(H):
        for i in range(W):
            # a fine texture, so the coding has something to lose
            yy = (bars[i * 8 // W][0] * (0.6 + 0.4 * j / H) +
                  ((i * 7 + j * 13) % 23) - 11)
            if sx <= i < sx + 48 and sy <= j < sy + 48:
                yy = 235 if ((i - sx) // 8 + (j - sy) // 8) & 1 else 40
            y[j * W + i] = max(0, min(255, int(yy)))
    for j in range(H // 2):
        for i in range(W // 2):
            _, u, v = bars[i * 2 * 8 // W]
            if sx <= 2 * i < sx + 48 and sy <= 2 * j < sy + 48:
                u, v = 90, 200
            cb[j * (W // 2) + i] = u
            cr[j * (W // 2) + i] = v
    return bytes(y), bytes(cb), bytes(cr)


def psnr(a, b):
    mse = sum((x - z) ** 2 for x, z in zip(a, b)) / len(a)
    return 99.0 if mse == 0 else 10 * math.log10(255 ** 2 / mse)


def unescape(nal):
    out, zeros = bytearray(), 0
    for b in nal:
        if zeros >= 2 and b == 3:
            zeros = 0
            continue
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


class Reader:
    def __init__(self, data):
        self.d, self.p = data, 0

    def bit(self):
        b = (self.d[self.p // 8] >> (7 - self.p % 8)) & 1
        self.p += 1
        return b

    def u(self, n):
        v = 0
        for _ in range(n):
            v = v << 1 | self.bit()
        return v

    def ue(self):
        z = 0
        while not self.bit():
            z += 1
        return (1 << z) - 1 + self.u(z)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k & 1 else -(k // 2)


def slice_nals(stream):
    """(nal header byte, header fields) for each slice NAL in a stream."""
    out = []
    starts = [i for i in range(len(stream) - 4) if stream[i:i + 4] == b'\0\0\0\1']
    for k, s in enumerate(starts):
        t = stream[s + 4] & 0x1f
        if t not in (1, 5):
            continue
        end = starts[k + 1] if k + 1 < len(starts) else len(stream)
        r = Reader(unescape(stream[s + 5:end]))
        f = dict(first_mb=r.ue(), slice_type=r.ue(), pps_id=r.ue(),
                 frame_num=r.u(16))
        if t == 5:
            f['idr_pic_id'] = r.ue()
        else:
            r.u(2)                      # override, list modification
        r.u(2 if t == 5 else 1)         # dec_ref_pic_marking
        f['qp_delta'] = r.se()
        f['deblock'] = r.ue()
        out.append((t, f))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    ap.add_argument('--keep', help='save the H.264 stream here (and .jpg)')
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    # IDR, P x3, IDR, P: (intra, frame_num, idr_pic_id)
    plan = [(1, 0, 0), (0, 1, 0), (0, 2, 0), (0, 3, 0), (1, 0, 1), (0, 1, 1)]
    pics = [picture(t) for t in range(len(plan))]
    mbw, mbh = (W + 15) // 16, (H + 15) // 16
    luma, chroma = mbw * 16 * mbh * 16, mbw * 16 * mbh * 8
    nal_off = 2 * luma + 2 * chroma
    recon = 1                           # akcam_h8290_frames_init()
    stream, payloads = b'', []

    dump = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, dump, True)
    q = QTest(a.qemu, 'ak3918ev200', ('-global', 'ak-h8290.dump-dir=' + dump))
    try:
        check(q.readl(VENC) == 0x82900760, 'swreg0 is the 8290 ASIC id')
        check(q.readl(VENC + 0x200) == 0x82900760, '0x200 mirrors swreg0')
        check(q.readl(VENC + 0xfc) & 0xfff == 1280, 'swreg63: 1280 pixels wide')

        for k, ((intra, fnum, idr_id), pic) in enumerate(zip(plan, pics)):
            src = IN_ADDR + k % 2 * 0x20000
            dst = OUT_ADDR + k * OUT_SIZE
            q.write(src, b''.join(pic))
            hdr = b''
            if intra:
                hdr = headers(W, H, QP)
                q.write(dst, hdr)
            base = len(hdr) & ~7
            rem, msb, lsb = remainder(hdr)
            r = Regs()
            config_h264(r, intra, fnum, idr_id, QP, QP)
            r.set('IN_Y', src); r.set('IN_CB', src + W * H)
            r.set('IN_CR', src + W * H + W * H // 4)
            r.set('OUT_BASE', dst + base)
            r.set('CTRL_BASE', PRIV + nal_off)
            r.set('CABAC_BASE', PRIV + nal_off + 256)
            r.set('LIMIT', (OUT_SIZE - base) // 8)
            r.set('REM_MSB', msb); r.set('REM_LSB', lsb); r.set('REM_BITS', rem)
            ref = recon ^ 1             # akcam_h8290_frames_apply()
            r.set('REF_Y', PRIV + luma * ref)
            r.set('REC_Y', PRIV + luma * recon)
            r.set('REF_C', PRIV + 2 * luma + chroma * ref)
            r.set('REC_C', PRIV + 2 * luma + chroma * recon)
            r.write_frame(q)

            what = 'frame %d (%s)' % (k, 'IDR' if intra else 'P')
            s1 = q.readl(VENC + 4)
            if k == 0:
                check(not s1 & IRQ_LINE, '%s: no interrupt inside the start write' % what)
            q.cmd('clock_step 5000000')
            s1 = q.readl(VENC + 4)
            check(s1 & IRQ_LINE and s1 & IRQ_FRAME_RDY,
                  '%s: swreg1 0x%x has the line and FRAME_READY' % (what, s1))
            check(not q.readl(VENC + 14 * 4) & 1, '%s: enable bit cleared' % what)
            q.writel(VENC + 4, s1 & ~IRQ_LINE)     # the driver's ack
            check(q.readl(VENC + 4) == s1 & ~IRQ_LINE, '%s: acked, status kept' % what)
            bits = q.readl(VENC + 24 * 4)
            n = base + (bits + 7) // 8          # akcam_h8290_output_bytes()
            mbs = field(q.readl(VENC + 38 * 4), 'MB_COUNT')
            qps = field(q.readl(VENC + 25 * 4), 'QP_SUM')
            check(mbs == mbw * mbh and qps * 2 >= mbs * QP - mbs,
                  '%s: %d bytes, MB count %d, QP sum / 2 %d' % (what, n, mbs, qps))
            data = q.read(dst, n)
            check(data[:len(hdr)] == hdr, '%s: the driver\'s headers are intact' % what)
            check(data[len(hdr):len(hdr) + 5] == (b'\0\0\0\1\x65' if intra else
                                                  b'\0\0\0\1\x41'),
                  '%s: the slice NAL starts with its own start code' % what)
            nal_size = int.from_bytes(q.read(PRIV + nal_off, 4), 'little')
            check(nal_size == n - len(hdr), '%s: the NAL table says %d bytes' % (what, nal_size))
            sl = slice_nals(data)
            if len(sl) == 1:
                t, f = sl[0]
                exp = dict(first_mb=0, slice_type=7 if intra else 5, pps_id=0,
                           frame_num=fnum, qp_delta=0, deblock=0)
                if intra:
                    exp['idr_pic_id'] = idr_id
                check(t == (5 if intra else 1) and f == exp,
                      '%s: slice header %s' % (what, f))
            else:
                check(False, '%s: %d slice NALs' % (what, len(sl)))
            stream += data
            payloads.append(n)
            recon ^= 1                  # akcam_h8290_frames_advance()

        # a stop before completion: ak_venc_stop_core()
        r.set('PIC_TYPE', 0)
        r.write_frame(q)
        q.writel(VENC + 14 * 4, q.readl(VENC + 14 * 4) & ~1)
        q.cmd('clock_step 5000000')
        check(not q.readl(VENC + 4) & IRQ_LINE, 'a stopped job raises no interrupt')

        # JPEG at quality 80, the driver's default
        jsrc = IN_ADDR + 0x40000
        jdst = OUT_ADDR + 8 * OUT_SIZE
        q.write(jsrc, b''.join(pics[0]))
        lq, cq = quant_tables(80)
        jh = jpeg_header(W, H, lq, cq)
        q.write(jdst, jh)
        r = Regs()                      # rebuilt from zero (akcam_h8290_config_jpeg)
        for nm in ('IN_SWAP8', 'IN_SWAP16', 'IN_SWAP32', 'OUT_SWAP8', 'OUT_SWAP16',
                   'OUT_SWAP32', 'CLOCK_GATING', 'TIMEOUT_IRQ'):
            r.set(nm, 1)
        r.set('BURST_LENGTH', 16)
        r.set('MODE', 2); r.set('PIC_TYPE', 1)
        r.set('WIDTH', mbw); r.set('HEIGHT', mbh); r.set('ROW', W)
        r.set('XFILL', (mbw * 16 - W) // 4); r.set('YFILL', mbh * 16 - H)
        quant_apply(r, lq, cq)
        r.set('IN_Y', jsrc); r.set('IN_CB', jsrc + W * H)
        r.set('IN_CR', jsrc + W * H + W * H // 4)
        whole = len(jh) & ~7
        r.set('OUT_BASE', jdst + whole)
        r.set('LIMIT', (OUT_SIZE - whole) // 8)
        rem, msb, lsb = remainder(jh)
        r.set('REM_BITS', rem); r.set('REM_MSB', msb); r.set('REM_LSB', lsb)
        r.write_frame(q)
        q.cmd('clock_step 5000000')
        s1 = q.readl(VENC + 4)
        check(s1 & IRQ_LINE and s1 & IRQ_FRAME_RDY, 'JPEG: swreg1 0x%x' % s1)
        q.writel(VENC + 4, s1 & ~IRQ_LINE)
        jn = whole + (q.readl(VENC + 24 * 4) + 7) // 8
        jpg = q.read(jdst, jn)
        check(jpg.startswith(jh), 'JPEG: the driver\'s header is intact')
        check(jpg.endswith(b'\xff\xd9'), 'JPEG: ends with the core\'s EOI (%d bytes)' % jn)
    finally:
        q.close()

    tmp = tempfile.mkdtemp()
    atexit.register(shutil.rmtree, tmp, True)
    path = os.path.join(tmp, 'out.h264')
    open(path, 'wb').write(stream)
    if a.keep:
        open(a.keep, 'wb').write(stream)
        open(os.path.splitext(a.keep)[0] + '.jpg', 'wb').write(jpg)
    res = subprocess.run(['ffmpeg', '-v', 'error', '-err_detect', 'explode',
                          '-i', path, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                         capture_output=True)
    check(res.returncode == 0 and not res.stderr,
          'ffmpeg decodes the stream cleanly%s' %
          (': ' + res.stderr.decode()[-300:] if res.stderr else ''))
    fsz = W * H * 3 // 2
    frames = len(res.stdout) // fsz
    check(frames == len(plan), 'ffmpeg returns %d of %d pictures, %dx%d' %
          (frames, len(plan), W, H))
    probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
                            'frame=pict_type,width,height', '-of', 'csv=p=0', path],
                           capture_output=True, text=True).stdout.split()
    types = ''.join(x.split(',')[-1] for x in probe)
    check(types == 'IPPPIP', 'picture types %s' % types)
    for f in range(frames):
        dec = res.stdout[f * fsz:(f + 1) * fsz]
        y, cb, cr = pics[f]
        p = (psnr(dec[:W * H], y), psnr(dec[W * H:W * H * 5 // 4], cb),
             psnr(dec[W * H * 5 // 4:], cr))
        check(min(p) > 26, 'picture %d PSNR Y %.1f / Cb %.1f / Cr %.1f dB against '
              'the input (> 26)' % (f, *p))
    # the first P refines the DC-only IDR; after it, only the moving
    # square costs anything
    check(max(payloads[2:4]) < payloads[0],
          'P frames after the first (%s bytes) are smaller than the IDR (%d)' %
          (payloads[2:4], payloads[0]))

    # yuvj420p: the decoder's own full-range samples.  Asking for yuv420p
    # makes ffmpeg 6 (Ubuntu 24.04, CI) squeeze them to limited range and
    # ffmpeg 8 not, so the same JPEG measured 28 dB on one and 33 on the other.
    jres = subprocess.run(['ffmpeg', '-v', 'error', '-f', 'jpeg_pipe', '-i', '-',
                           '-f', 'rawvideo', '-pix_fmt', 'yuvj420p', '-'],
                          input=jpg, capture_output=True)
    check(jres.returncode == 0 and not jres.stderr and len(jres.stdout) == fsz,
          'ffmpeg decodes the JPEG at %dx%d%s' %
          (W, H, ': ' + jres.stderr.decode()[-200:] if jres.stderr else ''))
    if len(jres.stdout) == fsz:
        y = pics[0][0]
        check(psnr(jres.stdout[:W * H], y) > 30,
              'JPEG luma PSNR %.1f dB against the input (> 30)' %
              psnr(jres.stdout[:W * H], y))

    names = sorted(os.listdir(dump))
    h264s = [x for x in names if x.endswith('.h264')]
    check(len(h264s) == len(plan) and any(x.endswith('.jpg') for x in names),
          'dump-dir holds %d .h264 jobs and the JPEG' % len(h264s))
    cat = b''.join(open(os.path.join(dump, x), 'rb').read() for x in h264s)
    check(cat == stream, 'the dumped jobs concatenate into the same stream')

    print('=== %s: AK3918EV200 8290 ===' % ('PASS' if not fails else 'FAIL'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
