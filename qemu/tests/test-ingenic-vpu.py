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


def texture(w, h, f):
    """NV12, stride w: random 4x4 blocks, the whole picture moved 8 right and
    4 down a frame -- found only by motion compensation."""
    y = bytearray(w * h)
    for r in range(h):
        by = (r - 4 * f) >> 2
        for x in range(w):
            bx = (x - 8 * f) >> 2
            v = ((bx * 73856093) ^ (by * 19349663)) & 0xffffffff
            y[r * w + x] = 32 + ((v * 2654435761) & 0xffffffff) % 192
    return bytes(y), bytes([128]) * (w * h // 2)


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
    b.u(100, 8); b.u(0, 8); b.u(30, 8)         # High, level 3.0, as libimp
    b.ue(0)                                     # sps id
    b.ue(1); b.ue(0); b.ue(0); b.u(0, 1)       # 4:2:0, 8-bit
    b.u(0, 1)                                   # no scaling matrices
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
    b.u(1, 1); b.u(0, 1); b.se(0)               # transform_8x8_mode (High)
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

# ---- JPEG decoding (T23, T30: libimp's IMP_Decoder) ----------------------
#
# libimp parses the file itself (ijpegd_decode_header) and hands the JPGC
# its tables in its own format; jpgc_decode_regs() builds them the same way
# (checked word for word against libimp's parser run on these files).

def jpeg_segments(b):
    i, segs = 2, {'dqt': {}, 'dht': {}, 'dri': 0}
    while i < len(b):
        m, n = b[i + 1], int.from_bytes(b[i + 2:i + 4], 'big')
        d = b[i + 4:i + 2 + n]
        if m == 0xDB:
            for j in range(0, len(d), 65):
                segs['dqt'][d[j] & 15] = list(d[j + 1:j + 65])
        elif m == 0xC4:
            j = 0
            while j < len(d):
                cnt = list(d[j + 1:j + 17])
                segs['dht'][(d[j] >> 4, d[j] & 15)] = (
                    cnt, list(d[j + 17:j + 17 + sum(cnt)]))
                j += 17 + sum(cnt)
        elif m == 0xC0:
            segs['sof'] = d
        elif m == 0xDD:
            segs['dri'] = int.from_bytes(d[:2], 'big')
        elif m == 0xDA:
            segs['sos'], segs['scan'] = d, b[i + 2 + n:]
            break
        i += 2 + n
    return segs


def jpgc_decode_regs(segs, bsa, p0a, p1a):
    """the registers libimp's T20_JPEGD_SliceInit chain writes"""
    hufm, hufb, hufs, qmem = [0] * 16, [0] * 64, [0] * 336, [0] * 256
    offs = {(1, 0): 0, (0, 0): 162, (1, 1): 174, (0, 1): 162}
    for g, k in enumerate([(1, 0), (0, 0), (1, 1), (0, 1)]):
        cnt, sym = segs['dht'].get(k, ([0] * 16, []))
        code = idx = 0
        a = []
        for l in range(16):
            a.append(code & ((1 << min(l + 1, 8)) - 1))
            hufb[16 * g + l] = (idx - code + offs[k]) & 0x1ff
            code, idx = (code + cnt[l]) << 1, idx + cnt[l]
        top = 0
        for l in range(8):
            top = (top << (l + 1)) | a[l]
        hufm[4 * g:4 * g + 4] = [a[12] << 24 | a[13] << 16 | a[14] << 8 |
                                 a[15], a[8] << 24 | a[9] << 16 | a[10] << 8 |
                                 a[11], top & 0xffffffff, top >> 32]
        for i, v in enumerate(sym):
            if k[0]:
                hufs[offs[k] + i] = v
            else:
                hufs[offs[k] + i] |= v << 4 if k[1] else v
    for t, q in segs['dqt'].items():
        qmem[64 * t:64 * t + 64] = q
    sof, sos = segs['sof'], segs['sos']
    h, w, nc = int.from_bytes(sof[1:3], 'big'), int.from_bytes(sof[3:5],
                                                                 'big'), sof[5]
    sel = {sos[1 + 2 * k]: sos[2 + 2 * k] for k in range(sos[0])}
    pc = [0xF0] * 4
    for k in range(nc):
        cid, hv, tq = sof[6 + 3 * k:9 + 3 * k]
        t = sel.get(cid, 0)
        pc[k] = ((((hv >> 4) * (hv & 15) - 1) << 4) & 0xff) | (tq << 2) | \
            ((t & 1) << 1) | ((t >> 4) & 1)
    mode = ((nc - 1) & 3) | (4 if segs['dri'] else 0)
    regs = [(0xC0000, 0), (0xE0004, 1)]
    regs += [(0xE1300 + 4 * i, v) for i, v in enumerate(hufm)]
    regs += [(0xE1200 + 4 * i, v) for i, v in enumerate(hufb)]
    regs += [(0xE1800 + 4 * i, v) for i, v in enumerate(hufs)]
    regs += [(0xE1400 + 4 * i, v) for i, v in enumerate(qmem)]
    regs += [(0xE0008, 0), (0xE000C, bsa), (0xE0010, p0a), (0xE0014, p1a),
             (0xE0028, ((w + 15) // 16) * ((h + 15) // 16) - 1),
             (0xE002C, segs['dri'] - 1 if segs['dri'] else 0)]
    regs += [(0xE0030 + 4 * i, v) for i, v in enumerate(pc)]
    regs += [(0xE0004, 0xA0009 | (mode & 4) | ((mode & 3) << 4)),
             (0xE0000, 7)]
    return w, h, regs


def t420_to_nv12(ty, tc, w, h):
    """libimp's c_copy_frame_t420_to_nv12: macroblock k's luma at 256 k,
    its chroma at 128 k (eight rows of 8 Cb, 8 Cr)"""
    mbw, mbh = (w + 15) // 16, (h + 15) // 16
    y, uv = bytearray(w * h), bytearray(w * h // 2)
    for k in range(mbw * mbh):
        mx, my = (k % mbw) * 16, (k // mbw) * 16
        for r in range(16):
            if my + r < h:
                n = min(16, w - mx)
                y[(my + r) * w + mx:(my + r) * w + mx + n] = \
                    ty[256 * k + 16 * r:256 * k + 16 * r + n]
        for r in range(8):
            if my // 2 + r < h // 2:
                row = tc[128 * k + 16 * r:128 * k + 16 * r + 16]
                for i in range(min(8, (w - mx) // 2)):
                    o = (my // 2 + r) * w + mx + 2 * i
                    uv[o], uv[o + 1] = row[i], row[8 + i]
    return bytes(y), bytes(uv)


def jpeg_decode_task(q, base, src, taskrg, jpeg):
    """the JPEG through the JPGC; returns the NV12 picture"""
    segs = jpeg_segments(jpeg)
    w, h, regs = jpgc_decode_regs(segs, BS, DEC, DEC + 0x100000)
    q.write(BS, segs['scan'])
    chain(q, regs)
    run_task(q, base, src, taskrg, False, lambda c, m: None)
    mbs = ((w + 15) // 16) * ((h + 15) // 16)
    return w, h, t420_to_nv12(q.read(DEC, 256 * mbs),
                              q.read(DEC + 0x100000, 128 * mbs), w, h)


# A 48x32 4:2:0 JPEG with optimised Huffman tables and a restart marker
# every MCU, and the luma libjpeg decodes it to.
DEC_JPEG = bytes.fromhex(
    'ffd8ffe000104a46494600010100000100010000ffdb00430005030404040305'
    '04040405050506070c08070707070f0b0b090c110f1212110f111113161c1713'
    '141a1511111821181a1d1d1f1f1f13172224221e241c1e1f1effdb0043010505'
    '050706070e08080e1e1411141e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e'
    '1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1e1effc0'
    '0011080020003003012200021101031101ffc400190000030101010000000000'
    '000000000000040506070208ffc4002210000104020105010100000000000000'
    '000400052231010203111321519152e1ffc40017010101010100000000000000'
    '000000000006070405ffc4002411000104010303050000000000000000000100'
    '020305310411210761711213415281ffdd00040001ffda000c03010002110311'
    '003f00f2b88d3514e0469a8aa611a6a29b88d3515d08a2586bee71caffd0c184'
    '69a8a70234d455308d3514dc469a8a431449657dce395fffd1cd4469a8a70234'
    'd455308d3514dc469a8a65144acb5f738e57ffd29211a6a29c08d35154a234d4'
    '53811a6a29145128d57dce395fffd3e8469a8a70234d455288d35144e351f87c'
    '71ebdedfd6b5f522d6d868eb22f7b5720637bfcf81927b004a2d596ce7901abf'
    'ffd4b111a6a289c6a3f0c78f5ef6feb5afa8ec0c413e3933d34fceb8e98fea3c'
    '469a8a2b71d4e91fbc556cf48fbbb93f8dc0f277f0120aad601b1795ffd9')
DEC_JPEG_Y = bytes.fromhex(
    '0002030607080a0c0e1013141518191c1d1e2222242629292c2e303134343838'
    '3a3d3f4042434648494c4e5050535457030508090b0d0f1013141619191b1e20'
    '2023242627292c2e3032333637393b3d3f3f424445474a4c4e4e52535455595b'
    '090a0c0e1011141518191c1d1e20222526282a2b2e2e32343537393b3c3e4143'
    '4446484a4b4d4e50525457585a5b5e5f0e1012131416191b1c1e20232326282a'
    '2b2d2f31333436383a3c3e3f41434548494a4d4e4f51545657595b5d5f606364'
    '12151619191c1d1f22242527282b2c2f3032343536383a3c3e40434445474a4b'
    '4e4f51525555595a5d5e606163656769181a1c1d1e21232526282b2c2e2f3233'
    '3436393a3c3d4042444547494b4c4f51535356585a5b5e5e6162666669696c6e'
    '1d1f21222425282a2c2d3032333537383a3c3f40424245464a4b4d4f51525556'
    '58595c5d5f61636566686b6d6e6f717320222426282a2c2e3032333637393a3c'
    '3d3f41444447484a4c4e5152535557595c5d6061626467696b6c6e7171747678'
    '26292b2d2e3031333537383b3b3e4042444548494a4c4f5152535657595a5d5f'
    '62636566696a6d6f6f72747677797a7d2a2b2e3032333637383a3c3e40424345'
    '47494c4d4e5052555557595b5d5e62636467696b6c6e70727476777a7a7d7e81'
    '3031333536393a3d3d4043444646494a4d4f51535555595a5c5e5f6162656668'
    '6a6c6e7171747578787b7c7e808283863537383b3c3d3f42434548494b4b4e50'
    '52545658595a5d5e60636566676a6c6e7071747576787b7d7e7f82848586898a'
    '3a3a3e3f40414445474a4b4d4e51535557585b5c5d5f62636566696b6c6e7071'
    '7375777a7b7c7f8183848688898a8d903e40434445464a4a4c4e515254565859'
    '5c5d6061636467696a6c6d7071727576797b7c7f8082838587898b8d8f909294'
    '434548494b4d4f5051545657595a5d5e6162656768696c6e6f71747577787b7c'
    '7f8082848587898b8d8f90939496989a48494c4d4f50525455575a5b5c5e6163'
    '6567696a6c6d7071727577797b7b7e7f818486878a8a8e8f91929597979a9b9e'
    '4d4e5153545557595c5e5f62626467696a6b6e7070727477797a7d7e7f818385'
    '878a8b8e8e9092949697989a9d9fa2a251535456575a5b5d5f62646567696a6d'
    '6d7071737576787a7c7e8083838687898c8c909092939698999a9fa1a4a4a4a5'
    '5658595c5c5f60626567686b6b6e6f72747578797b7b7f80838386888a8b8d8f'
    '9093959797999c9da2a2a2a4a6aaadb05b5d5e6161646668696c6e6f70737477'
    '797a7d7d8081838487888c8c8e8f92949698999c9c9fa1a2a4a5a8a9acadadad'
    '5f60646467676b6d6e6f737476767a7b7d7f8083848688898b8e909192959799'
    '999c9e9fa2a3a5a6a6a9b0b3b0a8a0996566686a6c6d6f71747577797b7c7f80'
    '82848587888b8d8f90939497989a9b9d9ea1a4a5a6a7aaabb3b0ada8a49d9a97'
    '696c6d7070737477797a7c7e7f82838687898c8d8e8f939396979a9b9e9fa2a2'
    'a4a7a8aaabadb0b2b2aca29b999ba0a36d6f71747477787b7c7e818384858789'
    '8b8d8f90919496989a9c9da0a1a2a5a6a9aaacaeafb1b4b59d9c9c9ea1a4a7aa'
    '73747779797b7d7f81848688898a8d8e9092959697989c9c9fa0a1a2a4a8aaac'
    'abadb1b3b5aea29a9fa0a3a5a6a8abab77797b7c7d7f82848687898b8c8e9193'
    '9396979a9b9c9fa1a2a3a6a9abadadafb4b7b7b0a59f9ea1a3a4a6a9a9abaeaf'
    '7c7d8081838487898a8d8e9092949698999a9e9ea1a1a5a6abaaabacaeb2b7b9'
    'b2aea6a1a0a1a3a5a9a9acadafb1b3b680828487888a8b8d8f92949596989a9c'
    '9e9fa3a3a5a7a9acacaeb1b2b4b5b5b6a4a09b9ea5aaacabadaeb1b2b5b5b9ba'
    '8687898b8c8f90939596989b9c9ea0a2a3a5a6a9a9abaeb0afb2b8bbbab1a8a2'
    '9fa5aaaaa9aaaeb4b1b4b6b7b9bbbdbf8b8c8f90919396989a9b9da0a0a2a5a6'
    'a8aaacadafb0b2b4bab9b6b2ada7a29fa9abadadaeb0b4b5b7b9babcbec0c2c4'
    '8f91939597989b9c9ea0a3a5a5a7a9abadaeb1b3b4b6b8b9b9b5aba5a2a5a8ac'
    'b1adaaafb7bcbab7bdbec0c2c4c5c8c89396979a9a9c9ea1a2a4a7a8a9acadaf'
    'b0b3b4b7b7b9bcbea5a5a5a6aaadb0b2afb3b8babababcbfc0c2c4c5c6c8cacc')

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
DEC = 0x01C00000


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
    decoded = {}
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
            if soc in ('t23', 't30'):
                # the JPEG back through the JPGC, as IMP_Decoder does
                q.write(PT + 0x1000, bytes(4096))
                q.writel(base + 0x30, 0)
                jpeg_d = jpeg_header(w, h, ql, qc) + \
                    q.read(BS, jst & 0xffffff) + b'\xff\xd9'
                dw, dh, (dy, duv) = jpeg_decode_task(q, base, src, taskrg,
                                                     jpeg_d)
                check(q.readl(base + 0xE0008) & (1 << 31) and
                      q.readl(base + 0x34) & 0x10,
                      'JPEG decode done (JPGC_STAT end, SCH_STAT JPGEND)')
                decoded[soc] = (jpeg_d, dy + duv, dw, dh)
                p = psnr_y(dy + duv, dw, dh, 0)
                check(p >= 35, 'decoded JPEG against the source picture: Y '
                      'PSNR %.1f dB (>= 35)' % p)
                # optimised tables, restart markers
                ew, eh, (ey, _) = jpeg_decode_task(q, base, src, taskrg,
                                                   DEC_JPEG)
                se = sum((a_ - b_) ** 2 for a_, b_ in zip(ey, DEC_JPEG_Y))
                p = 99.0 if not se else 10 * math.log10(255 * 255 * len(ey) /
                                                        se)
                check(p >= 40, 'optimised Huffman tables, restart markers: '
                      'Y PSNR %.1f dB against libjpeg (>= 40)' % p)
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
        if soc in decoded and ffmpeg:
            jd, nv12, dw, dh = decoded[soc]
            path = os.path.join(out, '%s-dec.jpg' % soc)
            open(path, 'wb').write(jd)
            raw, err = decode(ffmpeg, path, 'yuvj420p')
            se = sum((a_ - b_) ** 2 for a_, b_ in zip(raw[:dw * dh],
                                                      nv12[:dw * dh]))
            p = 99.0 if not se else 10 * math.log10(255 * 255 * dw * dh / se)
            check(not err and p >= 45, 'the JPGC\'s decode against '
                  'ffmpeg\'s: Y PSNR %.1f dB (>= 45)' % p)
    # inter coding: a textured picture moving (8, 4) pixels a frame, P
    # pictures against the decoder's (deblocked) previous one.  Without
    # motion compensation each P picture would cost about what I_PCM does
    # (1.5 bytes a pixel); with it, the decoded P pictures are the decoded
    # I moved, but for a residual here and there.
    print('== t21: H.264 P pictures, motion')
    base, src, taskrg = SOCS['t21']
    q = QTest(a.qemu, 't21')
    try:
        stream = h264_sps(mbw, mbh, h) + h264_pps()
        sizes = []
        for f in range(frames):
            y, uv = texture(w, h, f)
            q.write(SRC, y)
            q.write(SRC + w * h, uv)
            idr, qp = f == 0, 20
            chain(q, [(0x40004, ((mbh - 1) << 8) | (mbw - 1)),
                      (0x40010, SRC), (0x40014, SRC + w * h),
                      (0x40038, (w << 16) | w),
                      (0x90024, BS), (0x90028, 0x100000),
                      (0x40000, (0 if idr else 1 << 4) | (qp << 8) |
                       (2 << 2) | 0x23)])
            n = run_task(q, base, src, taskrg, False, lambda c, m: None)
            sizes.append(n)
            stream += h264_slice(idr, f, qp, q.read(BS, n))
    finally:
        q.close()
    pcm = w * h * 3 // 2
    check(all(n * 50 < pcm for n in sizes[1:]),
          'P pictures %s bytes (the I %d), under a fiftieth of I_PCM\'s %d'
          % (sizes[1:], sizes[0], pcm))
    path = os.path.join(out, 'motion.h264')
    open(path, 'wb').write(stream)
    if ffmpeg:
        raw, err = decode(ffmpeg, path)
        fs = w * h * 3 // 2
        check(not err and len(raw) == frames * fs,
              'ffmpeg decodes motion.h264 cleanly, %d of %d pictures%s'
              % (len(raw) // fs, frames, ': ' + err[-300:] if err else ''))
        for f in range(1, min(frames, len(raw) // fs)):
            d, i0 = raw[f * fs:], raw
            same = tot = 0
            for r in range(4 * f + 16, h - 16):
                for x in range(8 * f + 16, w - 16):
                    tot += 1
                    same += d[r * w + x] == i0[(r - 4 * f) * w + x - 8 * f]
            check(same * 100 >= tot * 99, 'P picture %d is the I picture '
                  'moved (%d, %d): %d of %d interior samples'
                  % (f, 8 * f, 4 * f, same, tot))
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
