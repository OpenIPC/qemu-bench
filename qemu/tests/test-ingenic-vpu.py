#!/usr/bin/env python3
"""Drive the ingenic-vpu model (T20 VPU, T21/T23/T30 HELIX, T30 RADIX) the
way the soc_vpu kernel driver and libimp do, and check that ffmpeg decodes
what it produces into the picture it was given.

Per task, as /dev/soc_vpu's IOCTL_CHANNEL_RUN does it (drivers/video/
soc_vpu/{soc_vpu.c, helix/helix.c, jz_nvpu/jz_nvpu.c, radix/radix.c}):
CPM SRBC stop request and its acknowledge, the soft-reset pulse, the
interrupt enables, the VDMA chain's address with RUN; the completion a
virtual millisecond later (status bit, INTC source), the byte count.  The
chains carry the registers libimp's H264E_T21_SliceInit /
T20_JPEGE_SliceInit / H265E_T30_SliceInit write that the model reads, and
their last entries (EFE_CTRL with RUN and TERM; VDMA 0x8008 on RADIX).
The streams are put together the way libimp does: its parameter sets and
slice headers in front of the engine's bytes (H.264: CABAC, the header
then cabac_alignment_one_bits; H.265: libimp's own VPS/SPS/PPS, CTB 64;
JPEG: SOI..SOS, the scan, EOI), the whole NAL escaped.

    python3 qemu/tests/test-ingenic-vpu.py [--qemu PATH] [--keep DIR]

ffmpeg comes from PATH, or FFMPEG=...; without it the decode checks are
skipped.
"""
import argparse, math, os, shutil, socket, subprocess, sys, tempfile

# ---- qtest --------------------------------------------------------------

class QTest:
    def __init__(self, qemu, machine):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest', '-display',
             'none', '-nodefaults', '-qtest', 'unix:' + path, '-serial',
             'null'], stdout=subprocess.DEVNULL,
            stderr=open(os.path.join(self.dir, 'qemu.err'), 'w'))
        srv.settimeout(20)
        self.sock, _ = srv.accept()
        self.f = self.sock.makefile('rw', buffering=1)

    def cmd(self, line):
        self.f.write(line + '\n')
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if r.startswith(('OK', 'FAIL', 'ERR')):
                return r

    def writel(self, a, v):
        self.cmd('writel 0x%x 0x%x' % (a, v & 0xffffffff))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        for o in range(0, len(data), 4096):
            c = data[o:o + 4096]
            self.cmd('write 0x%x 0x%x 0x%s' % (a + o, len(c), c.hex()))

    def read(self, a, n):
        out = b''
        for o in range(0, n, 65536):
            k = min(65536, n - o)
            out += bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a + o, k))
                                 .split()[1][2:])
        return out

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)

# ---- the picture --------------------------------------------------------

BARS = [(235, 128, 128), (210, 16, 146), (170, 166, 16), (145, 54, 34),
        (106, 202, 222), (81, 90, 240), (41, 240, 110), (16, 128, 128)]


def picture(w, h, f):
    """NV12, stride w: colour bars over a grey ramp, a square that moves."""
    y = bytearray(w * h)
    uv = bytearray(w * h // 2)
    sx, sy = (f * 16) % (w - 64), h // 2
    for r in range(h):
        for x in range(w):
            b = BARS[x * 8 // w]
            v = b[0] if r < h // 2 else 16 + 219 * x // w
            if sx <= x < sx + 64 and sy <= r < sy + 64:
                v = 235
            y[r * w + x] = v
    for r in range(h // 2):
        for x in range(0, w, 2):
            b = BARS[x * 8 // w]
            uv[r * w + x:r * w + x + 2] = bytes(b[1:]) if 2 * r < h // 2 \
                else b'\x80\x80'
    return bytes(y), bytes(uv)


def psnr_y(dec, w, h, f):
    y, _ = picture(w, h, f)
    se = sum((a - b) ** 2 for a, b in zip(dec[:w * h], y))
    return 99.0 if not se else 10 * math.log10(255 * 255 * w * h / se)

# ---- bitstream writers --------------------------------------------------

class Bits:
    def __init__(self):
        self.b = []

    def u(self, v, n):
        self.b += [(v >> (n - 1 - i)) & 1 for i in range(n)]

    def ue(self, v):
        v += 1
        n = v.bit_length()
        self.u(0, n - 1)
        self.u(v, n)

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def trailing(self):
        self.u(1, 1)
        while len(self.b) % 8:
            self.u(0, 1)

    def bytes(self):
        assert len(self.b) % 8 == 0
        return bytes(int(''.join(map(str, self.b[i:i + 8])), 2)
                     for i in range(0, len(self.b), 8))


def escape(rbsp):
    out, z = bytearray(), 0
    for c in rbsp:
        if z >= 2 and c <= 3:
            out.append(3)
            z = 0
        out.append(c)
        z = z + 1 if c == 0 else 0
    return bytes(out)


def nal(hdr, rbsp):
    return b'\0\0\0\1' + hdr + escape(rbsp)


def h264_sps(mbw, mbh, h):
    b = Bits()
    b.u(77, 8); b.u(0, 8); b.u(30, 8)          # Main, level 3.0
    b.ue(0)                                     # sps id
    b.ue(0)                                     # log2_max_frame_num - 4
    b.ue(2)                                     # POC type 2
    b.ue(1)                                     # one reference
    b.u(0, 1)
    b.ue(mbw - 1); b.ue(mbh - 1)
    b.u(1, 1); b.u(1, 1)                        # frames only, direct 8x8
    crop = (mbh * 16 - h) // 2
    b.u(1 if crop else 0, 1)
    if crop:
        b.ue(0); b.ue(0); b.ue(0); b.ue(crop)
    b.u(0, 1)                                   # no VUI
    b.trailing()
    return nal(b'\x67', b.bytes())


def h264_pps():
    b = Bits()
    b.ue(0); b.ue(0)
    b.u(1, 1)                                   # CABAC
    b.u(0, 1); b.ue(0)
    b.ue(0); b.ue(0)
    b.u(0, 1); b.u(0, 2)
    b.se(0); b.se(0); b.se(0)
    b.u(1, 1); b.u(0, 1); b.u(0, 1)             # deblocking control
    b.trailing()
    return nal(b'\x68', b.bytes())


def h264_slice(idr, frame_num, qp, data):
    """libimp's header (x264's slice_header_write), then the engine's
    CABAC bytes after cabac_alignment_one_bits."""
    b = Bits()
    b.ue(0)                                     # first_mb_in_slice
    b.ue(7 if idr else 5)                       # I / P, all slices
    b.ue(0)
    b.u(frame_num & 15, 4)
    if idr:
        b.ue(0)                                 # idr_pic_id
    else:
        b.u(0, 1)                               # num_ref_idx override
        b.u(0, 1)                               # ref_pic_list_modification
    if idr:
        b.u(0, 1); b.u(0, 1)                    # dec_ref_pic_marking
    else:
        b.u(0, 1)
    if not idr:
        b.ue(0)                                 # cabac_init_idc
    b.se(qp - 26)
    b.ue(0)                                     # deblocking on
    b.se(0); b.se(0)
    while len(b.b) % 8:
        b.u(1, 1)                               # cabac_alignment_one_bit
    return nal(b'\x65' if idr else b'\x41', b.bytes() + data)


# libimp's (T30, i265e) VPS / SPS / PPS for 640x360: CTB 64, QG 16, SAO,
# chroma QP offsets -3; read off its own stream
HEVC_PS = bytes.fromhex(
    '0000000140010c01ffff01600000030090000003000003003fba02f495c0'
    '0000000142010101600000030090000003000003003fa00502016965ba924ca2'
    '010000030001000003001908'
    '000000014401c072ce702240')


def hevc_slice(idr, poc, qp, data):
    b = Bits()
    b.u(1, 1)                                   # first slice segment
    if idr:
        b.u(0, 1)                               # no_output_of_prior_pics
    b.ue(0)                                     # pps id
    b.ue(2 if idr else 1)
    if not idr:
        b.u(poc & 255, 8)                       # POC lsb (8 bits)
        b.u(0, 1)                               # st_ref_pic_set here:
        b.ue(1); b.ue(0); b.ue(0); b.u(1, 1)    #   the last picture
    b.u(1, 1); b.u(0, 1)                        # SAO luma, not chroma
    if not idr:
        b.u(0, 1)                               # num_ref_idx override
        b.ue(3)                                 # five_minus_max_merge
    b.se(qp - 26)
    b.u(1, 1)                                   # loop filter across slices
    b.trailing()                                # byte_alignment()
    return nal(b'\x26\x01' if idr else b'\x02\x01', b.bytes() + data)


ZIGZAG = [0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19,
          26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28, 35, 42, 49,
          56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52,
          45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]
QL = [16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14,
      13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29, 51, 87, 80, 62, 18, 22,
      37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92, 49, 64,
      78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99]
QC = [17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24,
      26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99] + [99] * 32
# libimp's DHT segments (Annex K's tables), as its ijpege_write_header
# writes them
DHT = bytes.fromhex(
    'ffc4001f0000010501010101010100000000000000000102030405060708090a0b'
    'ffc400b5100002010303020403050504040000017d01020300041105122131410613'
    '516107227114328191a1082342b1c11552d1f02433627282090a161718191a2526'
    '2728292a3435363738393a434445464748494a535455565758595a636465666768'
    '696a737475767778797a838485868788898a92939495969798999aa2a3a4a5a6a7'
    'a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3'
    'e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa'
    'ffc4001f0100030101010101010101010000000000000102030405060708090a0b'
    'ffc400b51100020102040403040705040400010277000102031104052131061241'
    '510761711322328108144291a1b1c109233352f0156272d10a162434e125f11718'
    '191a262728292a35363738393a434445464748494a535455565758595a63646566'
    '6768696a737475767778797a82838485868788898a92939495969798999aa2a3a4'
    'a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9da'
    'e2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9fa')


def qscale(q, quality):
    s = 5000 // quality if quality < 50 else 200 - 2 * quality
    return [min(max((v * s + 50) // 100, 1), 255) for v in q]


def jpeg_header(w, h, ql, qc):
    """SOI, DQT (zigzag), SOF0 4:2:0, DHT (Annex K), SOS: libimp's
    ijpege_write_header puts these in front of the scan."""
    o = bytearray(b'\xff\xd8')
    for i, q in enumerate((ql, qc)):
        o += b'\xff\xdb\x00\x43' + bytes([i]) + bytes(q[ZIGZAG[k]]
                                                      for k in range(64))
    o += bytes([0xff, 0xc0, 0, 17, 8, h >> 8, h & 255, w >> 8, w & 255, 3,
                1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1])
    o += DHT
    o += bytes([0xff, 0xda, 0, 12, 3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0])
    return bytes(o)


def qmem_word(q):
    """libimp's qmem entry for quantiser q: [13:11] s, [10:0] 2^(11+s)/q
    with the largest s that fits"""
    s = 0
    while s < 7 and (2048 << (s + 1)) // q < 2048:
        s += 1
    return (s << 11) | ((2048 << s) // q)

# ---- the chains ---------------------------------------------------------

VLD, TERM = 1 << 31, 1 << 30
SOCS = {
    # machine: (VPU base, INTC source, TASKRG)
    't20': (0x13200000, 62, 0x10008),
    't21': (0x13200000, 62, 0x84),
    't23': (0x13100000, 40, 0x84),
    't30': (0x13200000, 62, 0x10008),
}
RADIX, RADIX_SRC = 0x13600000, 45
CPM_SRBC, INTC_ISR = 0x100000C4, 0x10001000
CHAIN, SRC, BS = 0x01000000, 0x01100000, 0x01400000


def chain(q, regs, write=None):
    words = []
    for i, (r, v) in enumerate(regs):
        words += [v, VLD | (TERM if i == len(regs) - 1 else 0) | r]
    data = b''.join(w.to_bytes(4, 'little') for w in words)
    if write:
        write(VA, data)
    else:
        q.write(CHAIN, data)


# The engines' TLB: a DMMU table (libdmmu.c) mapping NPAGES of virtual
# space at VA onto pages of POOL in reverse order, so that nothing works
# unless every access is translated page by page.
PT, POOL, VA, NPAGES = 0x01800000, 0x01A00000, 0x40000000, 0x60
VSRC, VBS = VA + 0x1000, VA + 0x30000


def tlb_setup(q):
    pgd = bytearray(4096)
    pgd[4 * (VA >> 22):4 * (VA >> 22) + 4] = (PT + 0x1000 | 1).to_bytes(4,
                                                                       'little')
    pte = bytearray(4096)
    for i in range(NPAGES):
        pte[4 * i:4 * i + 4] = (POOL + (NPAGES - 1 - i) * 0x1000 |
                                1).to_bytes(4, 'little')
    q.write(PT, bytes(pgd))
    q.write(PT + 0x1000, bytes(pte))

    def phys(va):
        return POOL + (NPAGES - 1 - ((va - VA) >> 12)) * 0x1000 + (va & 0xfff)

    def write(va, data):
        while data:
            k = min(len(data), 0x1000 - (va & 0xfff))
            q.write(phys(va), data[:k])
            va, data = va + k, data[k:]

    def read(va, n):
        out = b''
        while n:
            k = min(n, 0x1000 - (va & 0xfff))
            out += q.read(phys(va), k)
            va, n = va + k, n - k
        return out
    return write, read


def intc_level(q, src):
    return (q.readl(INTC_ISR + 0x20 * (src >> 5)) >> (src & 31)) & 1


def run_task(q, base, src, taskrg, radix, check, tlb=0):
    """the kernel's reset, start and wait; returns the byte count.  With
    tlb, the TLB's enables (GLBC bits / RADIX TLBE) and the table at PT,
    and the chain at VA"""
    stp, ack, sr = (27, 26, 28) if radix else (30, 29, 31)
    q.writel(CPM_SRBC, 1 << stp)
    check(q.readl(CPM_SRBC) & (1 << ack), 'SRBC acknowledges the stop request')
    q.writel(CPM_SRBC, (1 << stp) | (1 << sr))
    q.writel(CPM_SRBC, 0)
    start = VA if tlb else CHAIN
    if radix:
        q.writel(base + 0x10, PT if tlb else 0)    # TLB_TBA
        q.writel(base + 0x1C, tlb)                 # TLBE
        q.writel(base + 0x00, 0xffff | 0x0f000000)  # GLB_CTRL: INTE
        q.writel(base + 0x08, start | 1)            # ACM_CTRL: run
        stat, end = base + 0x40, 1 << 8
    else:
        q.writel(base + 0x30, PT if tlb else 0)    # SCH_TLBA
        q.writel(base + 0x00, (1 << 9) | (1 << 29) | (1 << 20) | (1 << 17) |
                 (1 << 16) | tlb)                    # GLBC: HIAXI, INTE
        q.writel(base + taskrg, start | 1)
        stat, end = base + 0x34, 1
    check(not q.readl(stat) & end and not intc_level(q, src),
          'nothing done at the start')
    q.step(2000000)
    check(q.readl(stat) & end and intc_level(q, src),
          'done a millisecond later, interrupt %d up' % (src + 8))
    if radix:
        n = q.readl(base + 0x68024)
        q.writel(base + 0x00, 0xffff)               # the ISR's mask clear
        q.writel(stat, 0)
    else:
        n = q.readl(base + 0x90038)
        q.writel(base + 0x00, 1 << 9)
    check(not intc_level(q, src), 'interrupt down after the ISR')
    return n


def decode(ffmpeg, path, pix='yuv420p'):
    # the stream on stdin: a docker ffmpeg need not see this host's files
    r = subprocess.run([ffmpeg, '-v', 'error', '-err_detect', 'explode',
                        '-i', '-', '-f', 'rawvideo', '-pix_fmt', pix, '-'],
                       input=open(path, 'rb').read(), capture_output=True)
    err = '\n'.join(l for l in r.stderr.decode(errors='replace').splitlines()
                    if not l.startswith('docker:'))
    return r.stdout, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(
        os.path.dirname(__file__), '..', '..', 'qemu-src', 'build',
        'qemu-system-mipsel'))
    ap.add_argument('--keep')
    a = ap.parse_args()
    ffmpeg = os.environ.get('FFMPEG') or shutil.which('ffmpeg')
    out = a.keep or tempfile.mkdtemp()
    os.makedirs(out, exist_ok=True)
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    w, h, frames = 320, 176, 4
    mbw, mbh = w // 16, h // 16
    for soc, (base, src, taskrg) in SOCS.items():
        print('== %s: H.264 (HELIX%s), JPEG' % (soc, '' if soc != 't20'
                                               else ' / VPU'))
        q = QTest(a.qemu, soc)
        try:
            stream = h264_sps(mbw, mbh, h) + h264_pps()
            for f in range(frames):
                y, uv = picture(w, h, f)
                q.write(SRC, y)
                q.write(SRC + w * h, uv)
                idr, qp = f == 0, 28 if f == 0 else 30
                chain(q, [(0x40004, (0 << 24) | ((mbh - 1) << 8) | (mbw - 1)),
                          (0x40010, SRC), (0x40014, SRC + w * h),
                          (0x40038, (w << 16) | w),
                          (0x90024, BS), (0x90028, 0x100000),
                          (0x40000, (0 if idr else 1 << 4) | (qp << 8) |
                           (2 << 2) | 0x23)])
                n = run_task(q, base, src, taskrg, False,
                             check if f == 0 else lambda c, m: None)
                check(0 < n < 0x100000, 'frame %d: %d bytes' % (f, n))
                stream += h264_slice(idr, f, qp, q.read(BS, n))
            # JPEG, quality 80, the JPGC fed by the EFE
            ql, qc = qscale(QL, 80), qscale(QC, 80)
            y, uv = picture(w, h, 0)
            q.write(SRC, y)
            q.write(SRC + w * h, uv)
            regs = [(0xE1400 + 4 * i, qmem_word(ql[ZIGZAG[i]]))
                    for i in range(64)]
            regs += [(0xE1500 + 4 * i, qmem_word(qc[ZIGZAG[i]]))
                     for i in range(64)]
            regs += [(0xE0008, 0), (0xE000C, BS), (0x40004, ((mbh - 1) << 16) |
                     (mbw - 1)), (0x40010, SRC), (0x40014, SRC + w * h),
                     (0x40038, (w << 16) | w), (0x40000, 0x400b)]
            chain(q, regs)
            run_task(q, base, src, taskrg, False, lambda c, m: None)
            jst = q.readl(base + 0xE0008)
            check(jst & (1 << 31) and q.readl(base + 0x34) & 0x10,
                  'JPEG done (JPGC_STAT end, SCH_STAT JPGEND)')
            jpeg = jpeg_header(w, h, ql, qc) + q.read(BS, jst & 0xffffff) + \
                b'\xff\xd9'
            # the same JPEG through the TLB: EFE, JPGC and VDMA translated
            vwrite, vread = tlb_setup(q)
            vwrite(VSRC, y + uv)
            vregs = [r for r in regs[:-5]] + [
                (0xE000C, VBS), (0x40004, ((mbh - 1) << 16) | (mbw - 1)),
                (0x40010, VSRC), (0x40014, VSRC + w * h),
                (0x40038, (w << 16) | w), (0x40000, 0x400b)]
            chain(q, vregs, vwrite)
            tlbe = (1 << 26) | (1 << 23) | (1 << 22)    # JPGC, EFE, VDMA
            run_task(q, base, src, taskrg, False, lambda c, m: None, tlbe)
            vn = q.readl(base + 0xE0008) & 0xffffff
            check(vread(VBS, vn) == q.read(BS, jst & 0xffffff),
                  'JPEG through the TLB (pages reversed): the same %d bytes'
                  % vn)
            check(not q.readl(base + 0x34) & (1 << 10), 'no TLB error')
            # an unmapped page: a TLB error, nothing written
            q.write(PT + 0x1000, bytes(4096))
            run_task(q, base, src, taskrg, False, lambda c, m: None, tlbe)
            check(q.readl(base + 0x34) & (1 << 10),
                  'an unmapped chain: SCH_STAT TLB error')
            if soc == 't30':
                print('== t30: H.265 (RADIX)')
                hevc = HEVC_PS
                for f in range(frames):
                    y, uv = picture(640, 360, f)
                    q.write(SRC, y)
                    q.write(SRC + 640 * 360, uv)
                    idr, qp = f == 0, 27 if f == 0 else 30
                    chain(q, [(0x20000, 0xc28 | (0 if idr else 0x11)),
                              (0x20004, (360 << 16) | 640),
                              (0x2000C, SRC), (0x20010, SRC + 640 * 360),
                              (0x20014, (640 << 16) | 640),
                              (0x20040, ((qp + 10) << 24) | ((qp - 15) << 16) |
                               (qp << 8) | 0x40),
                              (0x68000, (qp << 5) | (0x18 if idr else 0x14)),
                              (0x68008, BS), (0x08008, 0)])
                    n = run_task(q, RADIX, RADIX_SRC, 0, True,
                                 check if f == 0 else lambda c, m: None)
                    check(0 < n < 0x100000, 'H.265 frame %d: %d bytes' % (f, n))
                    if f == 0:
                        idr0, y0, uv0 = q.read(BS, n), y, uv
                    hevc += hevc_slice(idr, f, qp, q.read(BS, n))
                # the IDR picture again through RADIX's TLB
                vwrite, vread = tlb_setup(q)
                vwrite(VSRC, y0 + uv0)
                chain(q, [(0x20000, 0xc28), (0x20004, (360 << 16) | 640),
                          (0x2000C, VSRC), (0x20010, VSRC + 640 * 360),
                          (0x20014, (640 << 16) | 640),
                          (0x20040, (37 << 24) | (12 << 16) | (27 << 8) | 0x40),
                          (0x68000, (27 << 5) | 0x18),
                          (0x68008, VBS), (0x08008, 0)], vwrite)
                vn = run_task(q, RADIX, RADIX_SRC, 0, True, lambda c, m: None,
                              (1 << 16) | (1 << 17) | (1 << 21))
                check(vread(VBS, vn) == idr0,
                      'H.265 IDR through RADIX\'s TLB: the same %d bytes' % vn)
        finally:
            q.close()
        files = [('%s.h264' % soc, stream, w, h, frames, 'yuv420p', 40),
                 ('%s.jpg' % soc, jpeg, w, h, 1, 'yuvj420p', 35)]
        if soc == 't30':
            files.append(('t30.h265', hevc, 640, 360, frames, 'yuv420p', 25))
        for name, data, fw, fh, nf, pix, need in files:
            path = os.path.join(out, name)
            open(path, 'wb').write(data)
            if not ffmpeg:
                print('skip %s: no ffmpeg' % name)
                continue
            raw, err = decode(ffmpeg, path, pix)
            check(not err, 'ffmpeg decodes %s cleanly%s' % (name, ': ' + err[-300:]
                                                         if err else ''))
            fs = fw * fh * 3 // 2
            check(len(raw) == nf * fs, '%d of %d pictures' % (len(raw) // fs, nf))
            for f in range(min(nf, len(raw) // fs)):
                p = psnr_y(raw[f * fs:(f + 1) * fs], fw, fh, f)
                check(p >= need, '%s picture %d: Y PSNR %.1f dB (>= %d)'
                      % (name, f, p, need))
    # T41's JPEG engine: HELIX's JPEG half with its units renumbered (SCH
    # 0, VDMA 1, EFE 2, JPGC 3; the chain sets bit 19), reset through SRBC
    # 28/27/26, core INTC source 0
    print('== t41: JPEG (the HELIX JPEG engine)')
    base, core_isr = 0x13280000, 0x12300000

    def t41(r):
        unit = {0x4: 2, 0xE: 3}[r >> 16]
        return 0x80000 | (unit << 16) | (r & 0xFFFF)
    q = QTest(a.qemu, 't41')
    try:
        ql, qc = qscale(QL, 80), qscale(QC, 80)
        y, uv = picture(w, h, 0)
        q.write(SRC, y)
        q.write(SRC + w * h, uv)
        regs = [(0xE1400 + 4 * i, qmem_word(ql[ZIGZAG[i]])) for i in range(64)]
        regs += [(0xE1500 + 4 * i, qmem_word(qc[ZIGZAG[i]])) for i in range(64)]
        regs += [(0xE0008, 0), (0xE000C, BS),
                 (0x40004, ((mbh - 1) << 16) | (mbw - 1)), (0x40010, SRC),
                 (0x40014, SRC + w * h), (0x40038, (w << 16) | w),
                 (0x40000, 0x400b)]
        chain(q, [(t41(r), v) for r, v in regs])
        q.writel(CPM_SRBC, 1 << 27)
        check(q.readl(CPM_SRBC) & (1 << 26), 'SRBC acknowledges the stop')
        q.writel(CPM_SRBC, (1 << 27) | (1 << 28))
        q.writel(CPM_SRBC, 0)
        q.writel(base + 0x00, (1 << 9) | (1 << 29) | (1 << 20) | (1 << 17) |
                 (1 << 16))
        q.writel(base + 0x84, CHAIN | 1)
        check(not q.readl(base + 0x34) & 1 and not q.readl(core_isr) & 1,
              'nothing done at the start')
        q.step(2000000)
        jst = q.readl(base + 0x30008)
        check(q.readl(base + 0x34) & 0x11 == 0x11 and q.readl(core_isr) & 1,
              'done: SCH_STAT end and JPGEND, core INTC source 0 up')
        check(jst & (1 << 31) and 0 < jst & 0xffffff < 0x100000,
              'JPGC_STAT at +0x30008: end, %d bytes' % (jst & 0xffffff))
        jpeg41 = jpeg_header(w, h, ql, qc) + q.read(BS, jst & 0xffffff) + \
            b'\xff\xd9'
        q.writel(base + 0x00, 1 << 9)
        check(not q.readl(core_isr) & 1, 'interrupt down after the ISR')
    finally:
        q.close()
    path = os.path.join(out, 't41.jpg')
    open(path, 'wb').write(jpeg41)
    if ffmpeg:
        raw, err = decode(ffmpeg, path, 'yuvj420p')
        check(not err and len(raw) == w * h * 3 // 2,
              'ffmpeg decodes t41.jpg cleanly')
        if len(raw) == w * h * 3 // 2:
            p = psnr_y(raw, w, h, 0)
            check(p >= 35, 't41.jpg: Y PSNR %.1f dB (>= 35)' % p)
    if not a.keep:
        shutil.rmtree(out, ignore_errors=True)
    print('PASS' if not fails else 'FAIL (%d)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
