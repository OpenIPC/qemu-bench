#!/usr/bin/env python3
"""The Ingenic AVPU encoder model (hw/misc/ingenic-avpu.c) under qtest.

The script plays libimp's part (SDK 1.1.5 libimp.a: CoreManager.c,
EncSliceParam.c, SliceHdrWriter.c, ChannelMngr.c) on a t31 machine:

  - the core init: IDs, 0x8010 / 0x8054, the W1C status, the mask, the
    ticking timer at 0x8030 libimp spins on, busy reading idle;
  - H.264: the SPS, PPS and slice header written into the stream buffer
    the way FlushNAL does (escaped, the last n header bits handed over in
    Enc2's 0x78 / 0x7C), an Enc1 then an Enc2 command list for an IDR I
    picture and a P picture of a synthetic NV12 source with one macroblock
    changed, in CAVLC and in CABAC (cabac_init_idc 0 and 1), one variant
    through KSEG1 addresses; the status written back into the entries
    (stream bytes, the rate-control counters), interrupt 62 at the INTC;
  - HEVC: VPS / SPS / PPS (CTB 32, SAO) and a byte-aligned slice header
    for an IDR and a P picture, W0 codec 1;
  - JPEG on core 1 (registers + 0x200, done bit 4) from R0-R10 with
    zigzag quantisers in the table buffer;
  - the register block at 0x13208000 on t40 and from 0 at 0x13100000 on
    t41;
  - T41's generation of the core (libimp 1.2): core pages at 0x1000 (c +
    1), 4 KiB list entries with the words where libimp 1.2 puts them, the
    status at 0x5C0, done bits 2c / 2c + 1; an Enc1-coded CABAC IDR and
    P picture, and JPEG from core 1's page (0x40-0x88, bytes at 0x604).

The streams are decoded with ffmpeg when there is one (--ffmpeg, or on
PATH, or the jrottenberg/ffmpeg docker image): no decoder errors, the I
picture's macroblock means within 3 of the source's, the P picture's
too (inter coded, CAVLC and CABAC), a CAVLC sequence that changes every
macroblock likewise; the HEVC pictures' macroblock means within 4; the JPEG's luma PSNR
against the source above 30 dB.

    python3 qemu/tests/test-ingenic-avpu.py [--qemu PATH] [--ffmpeg CMD]
"""
import argparse
import math
import os
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile

INTC = 0x10001000
AVPU = 0x13208000
RAM = 0x00800000                # scratch DRAM for the test's buffers

W, H = 160, 96                  # 10 x 6 macroblocks
PITCH = 192                     # IMP's pitch, W0x48 = 192 / 64 - 1


class QTest:
    def __init__(self, qemu, machine, extra=()):
        self.dir = tempfile.mkdtemp()
        path = os.path.join(self.dir, 'qtest.sock')
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        srv.listen(1)
        self.proc = subprocess.Popen(
            [qemu, '-M', machine, '-machine', 'accel=qtest',
             '-display', 'none', '-nodefaults', '-qtest', 'unix:' + path] +
            list(extra),
            stdout=subprocess.DEVNULL,
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
        self.cmd('writel 0x%x 0x%x' % (a, v))

    def readl(self, a):
        return int(self.cmd('readl 0x%x' % a).split()[1], 16)

    def write(self, a, data):
        for i in range(0, len(data), 0x8000):
            chunk = data[i:i + 0x8000]
            self.cmd('write 0x%x 0x%x 0x%s' % (a + i, len(chunk),
                                                 chunk.hex()))

    def read(self, a, n):
        return bytes.fromhex(self.cmd('read 0x%x 0x%x' % (a, n))
                             .split()[1][2:])

    def step(self, ns):
        self.cmd('clock_step %d' % ns)

    def close(self):
        self.proc.kill()
        self.proc.wait()
        shutil.rmtree(self.dir, ignore_errors=True)


failures = 0


def check(what, got, want):
    global failures
    ok = got == want
    failures += not ok
    if isinstance(got, (bytes, bytearray)):
        shown = bytes(got[:8]).hex()
    elif isinstance(got, int) and not isinstance(got, bool):
        shown = hex(got)
    else:
        shown = str(got)
    print('  %s: %s (%s)' % ('PASS' if ok else 'FAIL', what, shown))
    return ok


# ---- bitstrings ------------------------------------------------------------

class Bits:
    def __init__(self):
        self.b = []

    def u(self, v, n):
        self.b += [(v >> (n - 1 - i)) & 1 for i in range(n)]

    def ue(self, v):
        x = v + 1
        n = x.bit_length()
        self.u(0, n - 1)
        self.u(x, n)

    def se(self, v):
        self.ue(2 * v - 1 if v > 0 else -2 * v)

    def trailing(self):
        self.u(1, 1)
        while len(self.b) % 8:
            self.u(0, 1)

    def bytes(self):
        b = self.b + [0] * (-len(self.b) % 8)
        return bytes(int(''.join(map(str, b[i:i + 8])), 2)
                     for i in range(0, len(b), 8))


def escape(rbsp):
    out, zeros = bytearray(), 0
    for c in rbsp:
        if zeros >= 2 and c <= 3:
            out.append(3)
            zeros = 0
        out.append(c)
        zeros = zeros + 1 if c == 0 else 0
    return bytes(out)


def nal(t, rbsp):
    return b'\x00\x00\x00\x01' + bytes([t]) + escape(rbsp)


def sps(cabac, mbw, mbh):
    b = Bits()
    b.u(77 if cabac else 66, 8); b.u(0, 8); b.u(30, 8)
    b.ue(0)                         # sps id
    b.ue(0)                         # log2_max_frame_num - 4
    b.ue(0); b.ue(0)                # POC type 0, lsb 4 bits
    b.ue(1); b.u(0, 1)              # one reference
    b.ue(mbw - 1); b.ue(mbh - 1)
    b.u(1, 1); b.u(1, 1); b.u(0, 1); b.u(0, 1)
    b.trailing()
    return nal(0x67, b.bytes())


def pps(cabac, cqp):
    b = Bits()
    b.ue(0); b.ue(0); b.u(int(cabac), 1); b.u(0, 1); b.ue(0)
    b.ue(0); b.ue(0); b.u(0, 1); b.u(0, 2)
    b.se(0); b.se(0); b.se(cqp)
    b.u(1, 1); b.u(0, 1); b.u(0, 1)
    b.trailing()
    return nal(0x68, b.bytes())


def slice_header(idr, cabac, idc, qp, frame):
    b = Bits()
    b.ue(0)                         # first_mb_in_slice
    b.ue(7 if idr else 5)
    b.ue(0)                         # pps
    b.u(frame & 15, 4)              # frame_num
    if idr:
        b.ue(0)                     # idr_pic_id
    b.u((2 * frame) & 15, 4)        # pic_order_cnt_lsb
    if not idr:
        b.u(0, 1)                   # num_ref_idx_active_override
        b.u(0, 1)                   # ref_pic_list_modification
    if idr:
        b.u(0, 1); b.u(0, 1)        # no_output_of_prior_pics, long_term
    else:
        b.u(0, 1)                   # adaptive_ref_pic_marking
    if cabac and not idr:
        b.ue(idc)
    b.se(qp - 26)                   # slice_qp_delta
    b.ue(0)                         # deblocking on (libimp's: idc 0)
    b.se(0); b.se(0)                # alpha / beta offsets: W10's (0)
    return b


def flush_header(start, hdr):
    """GenerateAvcSliceHeader + FlushNAL: the slice NAL's start code,
    header byte and escaped header in the stream ending at the offset the
    hardware writes from (with the tail bytes after it), and the tail
    {n, 0x7C} handed to Enc2.  Returns (bytes before offset, n, tail)."""
    bits = len(hdr.b)
    n = bits if bits < 24 else 16 + (bits & 7)
    tail_bytes = (n + 7) // 8
    full = start + escape(hdr.bytes())
    before = full[:len(full) - tail_bytes]
    v = 0
    for c in full[len(before) - 1:len(before) + 3]:
        v = (v << 8) | c
    v <<= 8 * (4 - len(full[len(before) - 1:len(before) + 3]))
    tail = (v >> (8 - (bits & 7))) & 0xFFFFFFFF
    return before, n, tail


# ---- the pictures ---------------------------------------------------------

def picture(changed=False):
    y = bytearray(PITCH * H)
    for r in range(H):
        for x in range(W):
            mb = (r // 16) * 10 + x // 16
            y[r * PITCH + x] = (40 + 17 * mb + (x % 16) + (r % 16)) & 0xFF
    uv = bytearray(PITCH * H // 2)
    for r in range(H // 2):
        for x in range(0, W, 2):
            uv[r * PITCH + x] = (90 + 3 * (x // 16) + (r // 8) * 9) & 0xFF
            uv[r * PITCH + x + 1] = (160 - 4 * (x // 16) - (r // 8) * 5) & 0xFF
    if changed:                     # macroblock (2, 1) goes flat
        for r in range(16, 32):
            for x in range(32, 48):
                y[r * PITCH + x] = 200
    return bytes(y), bytes(uv)


def luma(y):
    return bytes(y[r * PITCH + x] for r in range(H) for x in range(W))


# ---- the encoder ------------------------------------------------------------

def core_init(qt, base=AVPU):
    print('core init')
    check('ID word reads back non-zero', qt.readl(base + 0x00) != 0, True)
    qt.writel(base + 0x10, 4096)
    qt.writel(base + 0x54, 0x80)
    qt.writel(base + 0x18, 0xFFFFFF)
    qt.writel(base + 0x14, qt.readl(base + 0x14) | 1 | 4 | 0x10)
    check('interrupt mask Enc1 | Enc2 | core 1 (JPEG)',
          qt.readl(base + 0x14), 0x15)
    check('busy reads idle', qt.readl(base + 0x3F8), 0)
    qt.writel(base + 0x28, 4)
    t0 = qt.readl(base + 0x30)
    qt.step(10000)
    t1 = qt.readl(base + 0x30)
    check('timer ticks, never zero', t0 != 0 and t1 != t0, True)


def g2_entry(e):
    """A gen 1 entry (128 words) as libimp 1.2 lays it out on T41: the
    inverse of the model's avpu_g2_entry()."""
    r = [0] * 1024
    r[0] = (e[0] & (1 << 31)) | ((e[0] >> 28) & 3) | (e[0] & (7 << 24))
    r[1] = (e[1] & 0x7FF) | (((e[1] >> 12) & 0x7FF) << 16)
    r[0x0C // 4] = e[2]
    r[0x60 // 4] = e[3]
    r[0x64 // 4] = e[4]
    r[0x68 // 4] = e[5]
    for g1, g2 in ((0x80, 0xA0), (0x84, 0xA8), (0x88, 0xD0), (0x90, 0x1C0),
                   (0xA0, 0x108), (0xC0, 0x220), (0xC4, 0x22C),
                   (0xC8, 0x230), (0xCC, 0x234), (0xD0, 0x268),
                   (0xD4, 0x270)):
        r[g2 // 4] = e[g1 // 4]
    r[0x800 // 4] = e[0x6C // 4]
    r[0x804 // 4] = e[0x70 // 4]
    r[0x80C // 4] = e[0x74 // 4]
    r[0x810 // 4] = e[0x78 // 4] & 0xFFFFF
    r[0x814 // 4] = e[0x78 // 4] & (0x1F << 24)
    r[0x808 // 4] = e[0x7C // 4]
    return b''.join(v.to_bytes(4, 'little') for v in r)


def encode(qt, name, cabac, idc, kseg, slot, pics=None, enc1=False,
           gen2=None):
    """An IDR picture and P pictures (by default one, with one macroblock
    changed) through Enc1 + Enc2, as encode1 does -- or, enc1, Enc1 alone
    coding the slice (W8 bit 11; the stream in WC0-WCC, the header tail in
    the Enc1 entry, CABAC alignment written by software: what libimp does
    for one-slice pictures).  Returns the stream."""
    mbw, mbh, qp, cqp = W // 16, H // 16, 30, 2
    a = RAM + slot * 0x100000
    seg = 0xA0000000 if kseg else 0
    interm = a + 0x40000
    cl1, cl2 = a + 0x48000, a + 0x4C000
    bs, bs_size = a + 0x50000, 0x80000
    stream = b''
    off = 0
    pics = pics or [picture(), picture(True)]
    for k, (y, uv) in enumerate(pics):
        cur_src = a + 0x10000 * (k & 1)
        cur_rec = a + 0x20000 + 0x10000 * (k & 1)
        prev_rec = a + 0x20000 + 0x10000 * ((k - 1) & 1)
        qt.write(cur_src, y)
        qt.write(cur_src + PITCH * H, uv)
        idr = k == 0
        start = b''
        if idr:
            start = sps(cabac, mbw, mbh) + pps(cabac, cqp)
        hdr = slice_header(idr, cabac, idc, qp, k)
        nalh = start + b'\x00\x00\x00\x01' + bytes([0x65 if idr else 0x41])
        if enc1 and cabac:
            while len(hdr.b) % 8:
                hdr.u(1, 1)             # cabac_alignment_one_bit
            before, n, tail = nalh + escape(hdr.bytes()), 0, 0
        else:
            before, n, tail = flush_header(nalh, hdr)
        qt.write(bs + off, before)
        hdr_off = off + len(before)
        stype = 2 if idr else 1
        # Enc1: one slice
        e1 = [0] * 128
        e1[0] = (1 << 31) | (0 << 28) | (0 << 24)
        e1[1] = (W // 8 - 1) | ((H // 8 - 1) << 12)
        e1[2] = (idc << 8) | (int(cabac) << 10) | (int(enc1) << 11)
        e1[3] = ((cqp & 31) | ((cqp & 31) << 8) | (qp << 16) |
                 (stype << 28))
        e1[0x88 // 4] = PITCH
        e1[0x80 // 4] = seg | cur_src
        e1[0x84 // 4] = seg | (cur_src + PITCH * H)
        e1[0x90 // 4] = cur_rec
        e1[0x94 // 4] = cur_rec + W * H
        if not idr:
            e1[0xA0 // 4] = prev_rec
            e1[0xA4 // 4] = prev_rec + W * H
        if enc1:
            e1[0x70 // 4] = (idc << 8) | (qp << 16) | (stype << 28)
            e1[0x78 // 4] = (mbw * mbh - 1) | (n << 24)
            e1[0x7C // 4] = tail
            e1[0xC0 // 4] = seg | bs
            e1[0xC4 // 4] = bs_size
            e1[0xC8 // 4] = hdr_off
            e1[0xCC // 4] = ((bs_size - hdr_off) >> 5) << 5
        else:
            e1[0xCC // 4] = 0x8000
            e1[0xD0 // 4] = interm
            e1[0xD4 // 4] = interm + 0x1000
        if gen2 is not None:
            # T41: Enc1 alone, the entry in libimp 1.2's layout, core 0's
            # page; done is bit 0, the status at 0x5C0
            qt.write(cl1, g2_entry(e1))
            qt.writel(gen2 + 0x1000, cl1)
            qt.writel(gen2 + 0x1008, cl1 + 0x5C0)
            qt.writel(gen2 + 0x1018, 2)
            qt.step(1000000)
            st = qt.readl(gen2 + 0x18)
            pic = 'IDR' if idr else 'P'
            check('%s %s: Enc1 done (bit 0)' % (name, pic), st & 3, 1)
            qt.writel(gen2 + 0x18, st)
            check('%s %s: status cleared' % (name, pic),
                  qt.readl(gen2 + 0x18), 0)
            # gen 1's 0x138 (QP min / max, LCUs), 0x11C on at 0x5D4
            c = qt.readl(cl1 + 0x5D4 + 0x138 - 0x11C)
            check('%s %s: QP min / max %d, LCUs in the status' %
                  (name, pic, qp), (c & 0xFF, (c >> 8) & 0xFF, c >> 16),
                  (qp, qp, mbw * mbh))
            count = qt.readl(cl1 + 0x5C0)
            check('%s %s: stream bytes at 0x5C0' % (name, pic),
                  count > 0 and not (count >> 30), True)
            stream += qt.read(bs + off, hdr_off - off + count)
            off = (hdr_off + count + 31) & ~31
            continue
        qt.write(cl1, b''.join(v.to_bytes(4, 'little') for v in e1))
        e2 = [0] * 128
        e2[0] = 1 << 31
        e2[0x70 // 4] = (idc << 8) | (qp << 16) | (stype << 28)
        e2[0x78 // 4] = (mbw * mbh - 1) | (n << 24)
        e2[0x7C // 4] = tail
        e2[0xE8 // 4] = interm
        e2[0xEC // 4] = interm + 0x1000
        e2[0xF0 // 4] = seg | bs
        e2[0xF4 // 4] = bs_size
        e2[0xF8 // 4] = hdr_off
        e2[0xFC // 4] = ((bs_size - hdr_off) >> 5) << 5
        qt.write(cl2, b''.join(v.to_bytes(4, 'little') for v in e2))
        # AL_EncCore_Encode1
        qt.writel(AVPU + 0x3E0, seg | cl1)
        qt.writel(AVPU + 0x3E4, 2)
        if not enc1:
            qt.writel(AVPU + 0x3E0, seg | cl2)
            qt.writel(AVPU + 0x3E4, 8)
        early = qt.readl(AVPU + 0x18)
        qt.step(1000000)
        st = qt.readl(AVPU + 0x18)
        pic = 'IDR' if idr else 'P'
        if k == 0:
            check('%s: not done at the start write' % name, early, 0)
        check('%s %s: Enc1 and Enc2 done' % (name, pic), st & 5,
              1 if enc1 else 5)
        check('%s %s: interrupt 62 raised' % (name, pic),
              (qt.readl(INTC + 0x20) >> 30) & 1, 1)
        qt.writel(AVPU + 0x18, st)
        check('%s %s: status cleared, interrupt down' % (name, pic),
              (qt.readl(AVPU + 0x18), (qt.readl(INTC + 0x20) >> 30) & 1),
              (0, 0))
        c138 = qt.readl(cl1 + 0x138)
        check('%s %s: Enc1 QP min / max %d, LCUs' % (name, pic, qp),
              (c138 & 0xFF, (c138 >> 8) & 0xFF, c138 >> 16),
              (qp, qp, mbw * mbh))
        check('%s %s: Enc1 counters non-zero' % (name, pic),
              all(qt.readl(cl1 + o) for o in (0x10C, 0x13C, 0x158)), True)
        count = qt.readl(cl1 + 0x104) if enc1 else qt.readl(cl2 + 0x1E4)
        check('%s %s: stream bytes, no error flags' % (name, pic),
              count > 0 and not (count >> 30), True)
        count &= 0x3FFFFFFF
        out = qt.read(bs + off, hdr_off - off + count)
        stream += out
        off = (hdr_off + count + 31) & ~31  # the next AU
    return stream


def ffmpeg_cmd(arg, tmp):
    if arg:
        return shlex.split(arg)
    if shutil.which('ffmpeg'):
        return ['ffmpeg']
    if shutil.which('docker'):
        r = subprocess.run(['docker', 'image', 'inspect',
                            'jrottenberg/ffmpeg:6.1-alpine'],
                           capture_output=True)
        if r.returncode == 0:
            return ['docker', 'run', '--rm', '-i', '-v', '%s:%s' % (tmp, tmp),
                    'jrottenberg/ffmpeg:6.1-alpine']
    return None


def decode(ff, path, fmt='yuv420p'):
    r = subprocess.run(ff + ['-v', 'error', '-i', path, '-f', 'rawvideo',
                             '-pix_fmt', fmt, '-'], capture_output=True)
    errs = [l for l in r.stderr.decode(errors='replace').splitlines()
            if l.strip() and not l.startswith(('docker:', 'Unable to find'))]
    return r.returncode, r.stdout, errs


def test_h264(qt, ff, tmp):
    ref_i, ref_p = luma(picture()[0]), luma(picture(True)[0])
    for slot, (cabac, idc, kseg, enc1) in enumerate((
            (False, 0, False, False), (True, 0, False, False),
            (True, 1, True, False), (True, 1, False, True),
            (False, 0, False, True))):
        name = 'cabac%d' % idc if cabac else 'cavlc'
        if kseg:
            name += '-kseg1'
        if enc1:
            name += '-enc1'
        print('H.264 %s' % name)
        stream = encode(qt, name, cabac, idc, kseg, slot, enc1=enc1)
        check('%s: stream starts with the SPS' % name, stream[:5],
              b'\x00\x00\x00\x01\x67')
        path = os.path.join(tmp, name + '.h264')
        open(path, 'wb').write(stream)
        if not ff:
            print('  SKIP: no ffmpeg to decode %s' % name)
            continue
        rc, raw, errs = decode(ff, path)
        check('%s: decodes without errors' % name, rc == 0 and not errs, True)
        for l in errs[:5]:
            print('    ' + l)
        fs = W * H * 3 // 2
        check('%s: two frames' % name, len(raw), 2 * fs)
        if len(raw) != 2 * fs:
            continue
        i, p = raw[:W * H], raw[fs:fs + W * H]
        worst = 0
        for my in range(H // 16):
            for mx in range(W // 16):
                px = [(my * 16 + j) * W + mx * 16 + x
                      for j in range(16) for x in range(16)]
                d = abs(sum(ref_i[q] for q in px) - sum(i[q] for q in px))
                worst = max(worst, d // 256)
        check('%s: I macroblock means within 3 of the source (worst %d)' %
              (name, worst), worst <= 3, True)
        # inter coded, CABAC and CAVLC alike -- every macroblock near its
        # source
        worst = 0
        for my in range(H // 16):
            for mx in range(W // 16):
                px = [(my * 16 + j) * W + mx * 16 + x
                      for j in range(16) for x in range(16)]
                d = abs(sum(ref_p[q] for q in px) - sum(p[q] for q in px))
                worst = max(worst, d // 256)
        check('%s: P macroblock means within 3 of the source (worst %d)' %
              (name, worst), worst <= 3, True)


def test_cavlc_change(qemu, ff, tmp):
    """Every macroblock changes in CAVLC: the P pictures are inter coded,
    the first one already near the new source, and the stream decodes."""
    print('H.264 CAVLC, every macroblock changed')
    qt = QTest(qemu, 't31')
    try:
        qt.writel(AVPU + 0x14, 5)
        y, uv = picture()
        y2 = bytes((c ^ 0x55) for c in y)
        stream = encode(qt, 'change', False, 0, False, 0,
                        [(y, uv)] + [(y2, uv)] * 3)
    finally:
        qt.close()
    path = os.path.join(tmp, 'change.h264')
    open(path, 'wb').write(stream)
    if not ff:
        print('  SKIP: no ffmpeg')
        return
    rc, raw, errs = decode(ff, path)
    fs = W * H * 3 // 2
    check('change: four pictures decode without errors',
          (rc, len(raw), errs), (0, 4 * fs, []))
    if len(raw) != 4 * fs:
        return
    ref = luma(y2)
    for k in (1, 2, 3):
        p = raw[k * fs:k * fs + W * H]
        worst = 0
        for my in range(H // 16):
            for mx in range(W // 16):
                px = [(my * 16 + j) * W + mx * 16 + x
                      for j in range(16) for x in range(16)]
                worst = max(worst, abs(sum(ref[q] for q in px) -
                                       sum(p[q] for q in px)) // 256)
        check('change: P%d macroblock means within 3 of the new source '
              '(worst %d)' % (k, worst), worst <= 3, True)


def shifted(dx, dy):
    """picture() moved by (dx, dy) samples, its edges repeated: a pan"""
    y0, uv0 = picture()
    y = bytearray(PITCH * H)
    uv = bytearray(PITCH * H // 2)
    for r in range(H):
        sr = min(max(r - dy, 0), H - 1)
        for x in range(W):
            y[r * PITCH + x] = y0[sr * PITCH + min(max(x - dx, 0), W - 1)]
    for r in range(H // 2):
        sr = min(max(r - dy // 2, 0), H // 2 - 1)
        for x in range(0, W, 2):
            sx = min(max(x - (dx // 2) * 2, 0), W - 2)
            uv[r * PITCH + x] = uv0[sr * PITCH + sx]
            uv[r * PITCH + x + 1] = uv0[sr * PITCH + sx + 1]
    return bytes(y), bytes(uv)


def test_h264_slices(qemu, ff, tmp):
    """A picture of two slices (three macroblock rows each), CABAC, coded
    by Enc1 (W8 bit 11), the way libimp would split one: an IDR and two P
    pictures of a pan.  Each slice's NAL goes to a region of its own in the
    stream buffer (its header written in front of it beforehand).  The
    decoded pictures must be the model's own reconstruction, sample for
    sample (the dump property), and near the source."""
    print('H.264 two slices a picture, a pan (CABAC, Enc1 codes)')
    dump = os.path.join(tmp, 'slices-dump')
    os.makedirs(dump, exist_ok=True)
    qt = QTest(qemu, 't31', ['-global', 'ingenic-avpu.dump=%s' % dump])
    mbw, mbh, qp, idc = W // 16, H // 16, 30, 1
    a = RAM
    cl1, bs, bs_size = a + 0x48000, a + 0x50000, 0x80000
    pics = [picture(), shifted(4, 2), shifted(8, 4)]
    stream = b''
    try:
        qt.writel(AVPU + 0x14, 5)
        for k, (y, uv) in enumerate(pics):
            src = a + 0x10000 * (k & 1)
            rec = a + 0x20000 + 0x10000 * (k & 1)
            prev = a + 0x20000 + 0x10000 * ((k - 1) & 1)
            qt.write(src, y)
            qt.write(src + PITCH * H, uv)
            idr = k == 0
            entries, regions = [], []
            for sl in range(2):
                region = 0x20000 * sl
                hdr = Bits()
                hdr.ue(sl * 3 * mbw)                    # first_mb_in_slice
                hdr.ue(7 if idr else 5); hdr.ue(0); hdr.u(k & 15, 4)
                if idr:
                    hdr.ue(0)
                hdr.u((2 * k) & 15, 4)
                if not idr:
                    hdr.u(0, 1); hdr.u(0, 1)
                if idr:
                    hdr.u(0, 1); hdr.u(0, 1)
                else:
                    hdr.u(0, 1)
                if not idr:
                    hdr.ue(idc)
                hdr.se(qp - 26); hdr.ue(0); hdr.se(0); hdr.se(0)
                while len(hdr.b) % 8:
                    hdr.u(1, 1)                         # cabac alignment
                start = b''
                if idr and sl == 0:
                    start = sps(True, mbw, mbh) + pps(True, 2)
                nalh = start + b'\x00\x00\x00\x01' + \
                    bytes([0x65 if idr else 0x41])
                before = nalh + escape(hdr.bytes())
                qt.write(bs + region, before)
                regions.append((region, len(before)))
                e = [0] * 128
                e[0] = (1 << 31) if sl == 1 else 0
                e[1] = (W // 8 - 1) | ((H // 8 - 1) << 12)
                e[2] = (idc << 8) | (1 << 10) | (1 << 11)
                e[3] = (2 & 31) | ((2 & 31) << 8) | (qp << 16) | \
                    ((2 if idr else 1) << 28)
                e[5] = (sl * 3) << 12                   # first LCU (0, y)
                e[0x88 // 4] = PITCH
                e[0x80 // 4], e[0x84 // 4] = src, src + PITCH * H
                e[0x90 // 4], e[0x94 // 4] = rec, rec + W * H
                if not idr:
                    e[0xA0 // 4], e[0xA4 // 4] = prev, prev + W * H
                e[0x70 // 4] = (idc << 8) | (qp << 16) | \
                    ((2 if idr else 1) << 28)
                e[0x78 // 4] = 3 * mbw - 1
                e[0xC0 // 4], e[0xC4 // 4] = bs, bs_size
                e[0xC8 // 4] = region + len(before)
                e[0xCC // 4] = 0x10000
                entries.append(e)
            qt.write(cl1, b''.join(v.to_bytes(4, 'little')
                                   for e in entries for v in e))
            qt.writel(AVPU + 0x3E0, cl1)
            qt.writel(AVPU + 0x3E4, 2)
            qt.step(1000000)
            st = qt.readl(AVPU + 0x18)
            qt.writel(AVPU + 0x18, st)
            for sl, (region, hlen) in enumerate(regions):
                count = qt.readl(cl1 + 0x200 * sl + 0x104) & 0x3FFFFFFF
                stream += qt.read(bs + region, hlen + count)
            check('slices %s: Enc1 done' % ('IDR' if idr else 'P%d' % k),
                  st & 1, 1)
    finally:
        qt.close()
    path = os.path.join(tmp, 'slices.h264')
    open(path, 'wb').write(stream)
    if not ff:
        print('  SKIP: no ffmpeg')
        return
    rc, raw, errs = decode(ff, path, 'nv12')
    fs = W * H * 3 // 2
    check('slices: three pictures decode without errors',
          (rc, len(raw), errs), (0, 3 * fs, []))
    rec = open(os.path.join(dump, 'avpu-%dx%d.h264.rec' % (W, H)),
               'rb').read()
    check('slices: every picture is the model\'s reconstruction',
          raw[:3 * fs] == rec[:3 * fs], True)
    for k in (1, 2):
        ref = luma(pics[k][0])
        p = raw[k * fs:k * fs + W * H]
        e = sum((p[i] - ref[i]) ** 2 for i in range(W * H)) / (W * H)
        psnr = 10 * math.log10(255 * 255 / max(e, 1e-9))
        check('slices P%d: luma PSNR %.1f dB against the source (>= 30)' %
              (k, psnr), psnr >= 30, True)


def test_h264_b(qemu, ff, tmp):
    """B pictures (libimp sends none; the command words as its slice buffers
    would carry them: RefA at WA0, RefB at WA8): a pan coded I0 P2 B1 P4 B3,
    two slices a picture, CABAC, each B from the pictures either side.  The
    decoded pictures, in display order, must be the model's own
    reconstructions sample for sample, and the B pictures near the source
    and well under the I picture."""
    print('H.264 B pictures, a pan (CABAC, Enc1 codes, RefA / RefB)')
    dump = os.path.join(tmp, 'b-dump')
    os.makedirs(dump, exist_ok=True)
    qt = QTest(qemu, 't31', ['-global', 'ingenic-avpu.dump=%s' % dump])
    mbw, mbh, qp, idc = W // 16, H // 16, 30, 0
    a = RAM
    cl1, bs, bs_size = a + 0x48000, a + 0x50000, 0x80000
    recs = {t: a + 0x20000 + 0x10000 * t for t in range(5)}
    order = [(0, 2, None, None), (2, 1, 0, None), (1, 0, 0, 2),
             (4, 1, 2, None), (3, 0, 2, 4)]     # t, type (2 I 1 P 0 B), refs
    sizes = {}
    stream = b''

    def sps2():
        b = Bits()
        b.u(77, 8); b.u(0, 8); b.u(30, 8); b.ue(0); b.ue(0); b.ue(0)
        b.ue(0); b.ue(2); b.u(0, 1)             # two references
        b.ue(mbw - 1); b.ue(mbh - 1)
        b.u(1, 1); b.u(1, 1); b.u(0, 1); b.u(0, 1)
        b.trailing()
        return nal(0x67, b.bytes())

    try:
        qt.writel(AVPU + 0x14, 5)
        frame_num = 0
        for t, typ, r0, r1 in order:
            y, uv = shifted(2 * t, t)
            src = a + 0x10000 * (t & 1)
            qt.write(src, y)
            qt.write(src + PITCH * H, uv)
            entries, regions = [], []
            for sl in range(2):
                region = 0x20000 * sl
                hdr = Bits()
                hdr.ue(sl * 3 * mbw)
                hdr.ue({2: 7, 1: 5, 0: 6}[typ]); hdr.ue(0)
                hdr.u(frame_num & 15, 4)
                if typ == 2:
                    hdr.ue(0)
                hdr.u((2 * t) & 15, 4)
                if typ == 0:
                    hdr.u(1, 1)                 # direct_spatial_mv_pred
                if typ != 2:
                    hdr.u(0, 1)                 # override: the PPS's 1 / 1
                    hdr.u(0, 1)                 # ref_pic_list_modification
                    if typ == 0:
                        hdr.u(0, 1)
                if typ == 2:
                    hdr.u(0, 1); hdr.u(0, 1)
                elif typ == 1:
                    hdr.u(0, 1)                 # B is not a reference
                if typ != 2:
                    hdr.ue(idc)
                hdr.se(qp - 26); hdr.ue(0); hdr.se(0); hdr.se(0)
                while len(hdr.b) % 8:
                    hdr.u(1, 1)
                start = b''
                if typ == 2 and sl == 0:
                    start = sps2() + pps(True, 2)
                nalh = start + b'\x00\x00\x00\x01' + \
                    bytes([{2: 0x65, 1: 0x41, 0: 0x01}[typ]])
                before = nalh + escape(hdr.bytes())
                qt.write(bs + region, before)
                regions.append((region, len(before)))
                e = [0] * 128
                e[0] = (1 << 31) if sl == 1 else 0
                e[1] = (W // 8 - 1) | ((H // 8 - 1) << 12)
                e[2] = (idc << 8) | (1 << 10) | (1 << 11)
                e[3] = (2 & 31) | ((2 & 31) << 8) | (qp << 16) | (typ << 28)
                e[5] = (sl * 3) << 12
                e[0x88 // 4] = PITCH
                e[0x80 // 4], e[0x84 // 4] = src, src + PITCH * H
                e[0x90 // 4], e[0x94 // 4] = recs[t], recs[t] + W * H
                if r0 is not None:
                    e[0xA0 // 4], e[0xA4 // 4] = recs[r0], recs[r0] + W * H
                if r1 is not None:
                    e[0xA8 // 4], e[0xAC // 4] = recs[r1], recs[r1] + W * H
                e[0x70 // 4] = (idc << 8) | (qp << 16) | (typ << 28)
                e[0x78 // 4] = 3 * mbw - 1
                e[0xC0 // 4], e[0xC4 // 4] = bs, bs_size
                e[0xC8 // 4] = region + len(before)
                e[0xCC // 4] = 0x10000
                entries.append(e)
            qt.write(cl1, b''.join(v.to_bytes(4, 'little')
                                   for e in entries for v in e))
            qt.writel(AVPU + 0x3E0, cl1)
            qt.writel(AVPU + 0x3E4, 2)
            qt.step(1000000)
            st = qt.readl(AVPU + 0x18)
            qt.writel(AVPU + 0x18, st)
            n = 0
            for sl, (region, hlen) in enumerate(regions):
                count = qt.readl(cl1 + 0x200 * sl + 0x104) & 0x3FFFFFFF
                stream += qt.read(bs + region, hlen + count)
                n += count
            sizes[t] = n
            check('B test: %s%d done' % ('IPB'[2 - typ] if typ else 'B', t),
                  st & 1, 1)
            if typ != 0:
                frame_num += 1
    finally:
        qt.close()
    path = os.path.join(tmp, 'b.h264')
    open(path, 'wb').write(stream)
    print('  slice bytes: %s' % ', '.join('%s%d %d' % (
        'IPB'[2 - typ] if typ else 'B', t, sizes[t])
        for t, typ, _, _ in order))
    check('B pictures inter coded: each well under the I picture',
          max(sizes[1], sizes[3]) * 2 < sizes[0], True)
    if not ff:
        print('  SKIP: no ffmpeg')
        return
    rc, raw, errs = decode(ff, path, 'nv12')
    fs = W * H * 3 // 2
    check('B test: five pictures decode without errors',
          (rc, len(raw), errs), (0, 5 * fs, []))
    rec = open(os.path.join(dump, 'avpu-%dx%d.h264.rec' % (W, H)),
               'rb').read()
    coded = [t for t, _, _, _ in order]
    ok = len(rec) >= 5 * fs and all(
        raw[t * fs:(t + 1) * fs] == rec[i * fs:(i + 1) * fs]
        for i, t in enumerate(coded))
    check('B test: every picture (display order) is the model\'s '
          'reconstruction', ok, True)
    for t in (1, 3):
        ref = luma(shifted(2 * t, t)[0])
        p = raw[t * fs:t * fs + W * H]
        e = sum((p[i] - ref[i]) ** 2 for i in range(W * H)) / (W * H)
        psnr = 10 * math.log10(255 * 255 / max(e, 1e-9))
        check('B%d: luma PSNR %.1f dB against the source (>= 30)' %
              (t, psnr), psnr >= 30, True)


# ---- HEVC: the parameter sets and slice header hevc-enc's coder follows ----

def hnal(t, rbsp):
    return b'\x00\x00\x00\x01' + bytes([t << 1, 1]) + escape(rbsp)


def hevc_ptl(b):
    b.u(0, 2); b.u(0, 1); b.u(1, 5); b.u(0x60000000, 32); b.u(0, 4)
    b.u(0, 32); b.u(0, 12); b.u(63, 8)


def hevc_headers(w, h, beta, tc, libimp):
    """libimp: the fields T31 libimp's own parameter sets have (decoded
    from sample-Encoder-video's stream): no SAO, default scaling lists,
    intra transform depth 1, transform skip, TMVP, strong smoothing."""
    v = Bits()
    v.u(0, 4); v.u(3, 2); v.u(0, 6); v.u(0, 3); v.u(1, 1); v.u(0xffff, 16)
    hevc_ptl(v)
    v.u(1, 1); v.ue(1); v.ue(0); v.ue(0); v.u(0, 6); v.ue(0); v.u(0, 1)
    v.u(0, 1)
    v.trailing()
    s = Bits()
    s.u(0, 4); s.u(0, 3); s.u(1, 1)
    hevc_ptl(s)
    s.ue(0); s.ue(1); s.ue(w); s.ue(h); s.u(0, 1)
    s.ue(0); s.ue(0); s.ue(6)
    s.u(1, 1); s.ue(1); s.ue(0); s.ue(0)
    d = int(libimp)
    s.ue(0); s.ue(2); s.ue(0); s.ue(3); s.ue(d); s.ue(d)     # CTB 32
    s.u(d, 1)                                                # scaling lists
    if libimp:
        s.u(0, 1)                                            #   defaults
    s.u(0, 1); s.u(1 - d, 1); s.u(0, 1)                      # SAO, PCM
    s.ue(1); s.ue(1); s.ue(0); s.ue(0); s.u(1, 1)            # one RPS
    s.u(0, 1); s.u(d, 1); s.u(d, 1); s.u(0, 1); s.u(0, 1)    # TMVP, smoothing
    s.trailing()
    p = Bits()
    p.ue(0); p.ue(0); p.u(0, 1); p.u(0, 1); p.u(0, 3); p.u(0, 1); p.u(1, 1)
    p.ue(0); p.ue(0); p.se(0); p.u(0, 1); p.u(d, 1); p.u(1, 1); p.ue(0)
    p.se(0); p.se(0); p.u(0, 4); p.u(0, 1); p.u(0, 1); p.u(1, 1)
    p.u(1, 1); p.u(1, 1); p.u(0, 1); p.se(beta); p.se(tc)
    p.u(0, 1); p.u(0, 1); p.ue(0); p.u(0, 1); p.u(0, 1)
    p.trailing()
    return hnal(32, v.bytes()) + hnal(33, s.bytes()) + hnal(34, p.bytes())


def hevc_slice_header(p_slice, qp, poc, beta, tc, libimp):
    b = Bits()
    b.u(1, 1)
    if not p_slice:
        b.u(0, 1)
    b.ue(0); b.ue(1 if p_slice else 2)
    if p_slice:
        b.u(poc & 1023, 10); b.u(1, 1)
        if libimp:
            b.u(1, 1)                           # slice_temporal_mvp
    if not libimp:
        b.u(3, 2)                               # SAO luma, chroma
    if p_slice:
        b.u(0, 1); b.u(int(libimp), 1)          # cabac_init_flag
        b.ue(0)                                 # MaxNumMergeCand 5
    b.se(qp - 26)
    b.u(1, 1); b.u(0, 1); b.se(beta); b.se(tc); b.u(1, 1)
    b.u(1, 1)                                   # byte_alignment()
    while len(b.b) % 8:
        b.u(0, 1)
    return b


def test_hevc(qt, ff, tmp, libimp=False):
    name, qp, beta, tc = 'hevc-libimp' if libimp else 'hevc', 30, 5, -5
    print('HEVC (CTB 32, %s)' % ('libimp\'s parameter sets' if libimp
                                 else 'SAO'))
    a = RAM + (0xC00000 if libimp else 0xB00000)
    src = [a, a + 0x10000]
    rec = [a + 0x20000, a + 0x30000]
    interm, cl1, cl2 = a + 0x40000, a + 0x48000, a + 0x4C000
    bs, bs_size = a + 0x50000, 0x80000
    stream, off = b'', 0
    for k in range(2):
        y, uv = picture(k == 1)
        qt.write(src[k], y)
        qt.write(src[k] + PITCH * H, uv)
        start = hevc_headers(W, H, beta, tc, libimp) if k == 0 else b''
        hdr = hevc_slice_header(k == 1, qp, k, beta, tc, libimp)
        before, n, tail = flush_header(
            start + b'\x00\x00\x00\x01' + bytes([(1 if k else 19) << 1, 1]),
            hdr)
        qt.write(bs + off, before)
        hdr_off = off + len(before)
        stype = 1 if k else 2
        e1 = [0] * 128
        e1[0] = (1 << 31) | (1 << 28) | (1 << 24)       # HEVC, CTB 32
        e1[0] |= int(libimp) << 4                       # intra TT depth
        e1[1] = (W // 8 - 1) | ((H // 8 - 1) << 12)
        e1[2] = int(libimp) << 8                        # cabac_init_flag
        e1[4] = (beta & 31) | ((tc & 63) << 8)          # deblocking
        e1[3] = (qp << 16) | (stype << 28)
        e1[0x88 // 4] = PITCH
        e1[0x80 // 4], e1[0x84 // 4] = src[k], src[k] + PITCH * H
        e1[0x90 // 4] = rec[k]
        e1[0xA0 // 4] = rec[k - 1] if k else 0
        e1[0xCC // 4], e1[0xD0 // 4] = 0x8000, interm
        qt.write(cl1, b''.join(v.to_bytes(4, 'little') for v in e1))
        e2 = [0] * 128
        e2[0] = 1 << 31
        e2[0x70 // 4] = (qp << 16) | (stype << 28)
        e2[0x78 // 4] = (5 * 3 - 1) | (n << 24)
        e2[0x7C // 4] = tail
        e2[0xE8 // 4] = interm
        e2[0xF0 // 4], e2[0xF4 // 4], e2[0xF8 // 4] = bs, bs_size, hdr_off
        e2[0xFC // 4] = ((bs_size - hdr_off) >> 5) << 5
        qt.write(cl2, b''.join(v.to_bytes(4, 'little') for v in e2))
        qt.writel(AVPU + 0x3E0, cl1)
        qt.writel(AVPU + 0x3E4, 2)
        qt.writel(AVPU + 0x3E0, cl2)
        qt.writel(AVPU + 0x3E4, 8)
        qt.step(1000000)
        st = qt.readl(AVPU + 0x18)
        check('%s %s: Enc1 and Enc2 done' % (name, 'P' if k else 'IDR'),
              st & 5, 5)
        qt.writel(AVPU + 0x18, st)
        count = qt.readl(cl2 + 0x1E4) & 0x3FFFFFFF
        stream += qt.read(bs + off, hdr_off - off + count)
        off = (hdr_off + count + 31) & ~31
    path = os.path.join(tmp, name + '.h265')
    open(path, 'wb').write(stream)
    if not ff:
        print('  SKIP: no ffmpeg to decode %s' % name)
        return
    rc, raw, errs = decode(ff, path)
    check('%s: decodes without errors' % name, rc == 0 and not errs, True)
    for l in errs[:5]:
        print('    ' + l)
    fs = W * H * 3 // 2
    check('%s: two frames' % name, len(raw), 2 * fs)
    if len(raw) != 2 * fs:
        return
    for k, pic in enumerate((raw[:W * H], raw[fs:fs + W * H])):
        ref = luma(picture(k == 1)[0])
        worst = 0
        for my in range(H // 16):
            for mx in range(W // 16):
                px = [(my * 16 + j) * W + mx * 16 + x
                      for j in range(16) for x in range(16)]
                d = abs(sum(ref[q] for q in px) - sum(pic[q] for q in px))
                worst = max(worst, d // 256)
        check('%s %s: macroblock means within 4 of the source (worst %d)' %
              (name, 'P' if k else 'I', worst), worst <= 4, True)


# the IJG luma / chroma tables at quality 75, natural order
QL = [8, 6, 5, 8, 12, 20, 26, 31, 6, 6, 7, 10, 13, 29, 30, 28, 7, 7, 8, 12,
      20, 29, 35, 28, 7, 9, 11, 15, 26, 44, 40, 31, 9, 11, 19, 28, 34, 55,
      52, 39, 12, 18, 28, 32, 41, 52, 57, 46, 25, 32, 39, 44, 52, 61, 60, 51,
      36, 46, 48, 49, 56, 50, 52, 50]
QC = [9, 9, 12, 24, 50, 50, 50, 50, 9, 11, 13, 33, 50, 50, 50, 50, 12, 13,
      28, 50, 50, 50, 50, 50, 24, 33, 50, 50, 50, 50, 50, 50] + [50] * 32
ZIGZAG = [0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19,
          26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28, 35, 42, 49,
          56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52, 45,
          38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63]


def test_jpeg(qt, ff, tmp):
    """On core 1 (registers + 0x200, done bit 4), where libimp runs it."""
    print('JPEG (core 1)')
    core = AVPU + 0x200
    a = RAM + 0xA00000
    src, tables, bs, size, off = a, a + 0x20000, a + 0x30000, 0x40000, 0x40
    y, uv = picture()
    qt.write(src, y)
    qt.write(src + PITCH * H, uv)
    zz = bytes(QL[ZIGZAG[i]] for i in range(64)) + \
        bytes(QC[ZIGZAG[i]] for i in range(64))
    qt.write(tables, zz + bytes(400))
    regs = [0, (H - 1) | ((W - 1) << 16), 0, PITCH, src, src + PITCH * H,
            tables, bs, size, off, size - off]
    for i, v in enumerate(regs):
        qt.writel(core + 0x200 + 4 * i, v)
    qt.writel(core + 0x3F0, 1)
    qt.writel(core + 0x3E4, 1)
    qt.step(1000000)
    st = qt.readl(AVPU + 0x18)
    check('JPEG done (bit 4)', st & 0x11, 0x10)
    check('interrupt 62 raised', (qt.readl(INTC + 0x20) >> 30) & 1, 1)
    qt.writel(AVPU + 0x18, st)
    n = qt.readl(core + 0x234)
    check('no overflow', qt.readl(core + 0x238) & 2, 0)
    data = qt.read(bs + off, n)
    check('SOI ... EOI', (data[:2], data[-2:]), (b'\xff\xd8', b'\xff\xd9'))
    check('nothing written before the offset', qt.read(bs, off), bytes(off))
    path = os.path.join(tmp, 'avpu.jpg')
    open(path, 'wb').write(data)
    if not ff:
        print('  SKIP: no ffmpeg to decode the JPEG')
        return
    rc, raw, errs = decode(ff, path, 'gray')
    check('JPEG decodes without errors', rc == 0 and not errs, True)
    for l in errs[:5]:
        print('    ' + l)
    if len(raw) != W * H:
        check('JPEG size %dx%d' % (W, H), len(raw), W * H)
        return
    ref = luma(y)
    mse = sum((a - b) ** 2 for a, b in zip(ref, raw)) / (W * H)
    psnr = 99 if not mse else 10 * math.log10(255 * 255 / mse)
    check('JPEG luma PSNR %.1f dB above 30' % psnr, psnr > 30, True)


def test_t41(qemu, ff, tmp):
    """T41's core: H.264 through the gen 2 list layout, and JPEG."""
    base = 0x13100000
    print('t41: the gen 2 core')
    qt = QTest(qemu, 't41')
    try:
        qt.writel(base + 0x14, 0xF)
        check('t41: core 0 / 1 busy idle',
              (qt.readl(base + 0x101C), qt.readl(base + 0x181C),
               qt.readl(base + 0x201C)), (0, 0, 0))
        stream = encode(qt, 't41-cabac1', True, 1, False, 0, enc1=True,
                        gen2=base)
        check('t41: stream starts with the SPS', stream[:5],
              b'\x00\x00\x00\x01\x67')
        path = os.path.join(tmp, 't41.h264')
        open(path, 'wb').write(stream)
        if ff:
            rc, raw, errs = decode(ff, path)
            fs = W * H * 3 // 2
            check('t41: two pictures decode without errors',
                  (rc, len(raw), errs), (0, 2 * fs, []))
            if len(raw) == 2 * fs:
                ref_p = luma(picture(True)[0])
                p = raw[fs:fs + W * H]
                d = sum(p[r * W + x] for r in range(16, 32)
                        for x in range(32, 48)) / 256
                check('t41: P picture: the changed macroblock near the '
                      'source (mean %.1f, 200)' % d, abs(d - 200) <= 3, True)
        else:
            print('  SKIP: no ffmpeg to decode the T41 stream')
        # JPEG on core 1's page
        a = RAM + 0xA00000
        src, tables, bs, size, off = a, a + 0x20000, a + 0x30000, 0x40000, 0x40
        y, uv = picture()
        qt.write(src, y)
        qt.write(src + PITCH * H, uv)
        qt.write(tables, bytes(QL[ZIGZAG[i]] for i in range(64)) +
                 bytes(QC[ZIGZAG[i]] for i in range(64)))
        page = base + 0x2000
        for o, v in ((0x40, 0x101), (0x44, (H - 1) | ((W - 1) << 16)),
                     (0x58, tables), (0x60, bs), (0x68, size), (0x6C, off),
                     (0x70, size - off), (0x74, PITCH), (0x78, PITCH),
                     (0x80, src), (0x88, src + PITCH * H)):
            qt.writel(page + o, v)
        qt.writel(page + 0x18, 1)
        qt.step(1000000)
        st = qt.readl(base + 0x18)
        check('t41: JPEG done (core 1: bit 2)', st & 0xF, 4)
        qt.writel(base + 0x18, st)
        n = qt.readl(page + 0x604)
        data = qt.read(bs + off, n)
        check('t41: JPEG SOI ... EOI at the offset',
              (data[:2], data[-2:]), (b'\xff\xd8', b'\xff\xd9'))
        if ff:
            path = os.path.join(tmp, 't41.jpg')
            open(path, 'wb').write(data)
            rc, raw, errs = decode(ff, path, 'gray')
            check('t41: JPEG decodes without errors', (rc, errs), (0, []))
    finally:
        qt.close()


def test_rebased(qemu):
    for m, base in (('t40', 0x13208000), ('t41', 0x13100000)):
        print('%s: registers at 0x%x' % (m, base))
        qt = QTest(qemu, m)
        try:
            qt.writel(base + 0x14, 5)
            check('%s: mask reads back' % m, qt.readl(base + 0x14), 5)
            qt.writel(base + 0x28, 4)
            check('%s: timer ticking' % m, qt.readl(base + 0x30) != 0, True)
            check('%s: busy idle' % m, qt.readl(base + 0x3F8), 0)
            if m == 't40':
                # the block ends at the registers: T40's RTC is at
                # 0x132A0000, inside the MiB avpu.ko claims
                qt.writel(0x132A003C, 0xA55A)
                check('t40: the RTC past the AVPU still answers (WEN)',
                      qt.readl(0x132A003C) >> 31, 1)
        finally:
            qt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='./qemu-src/build/qemu-system-mipsel')
    ap.add_argument('--ffmpeg', default=None,
                    help='ffmpeg command to decode with (default: ffmpeg on '
                         'PATH, else the jrottenberg/ffmpeg:6.1-alpine '
                         'docker image if present)')
    ap.add_argument('--keep', help='directory to leave the streams in')
    args = ap.parse_args()

    tmp = args.keep or tempfile.mkdtemp()
    os.makedirs(tmp, exist_ok=True)
    ff = ffmpeg_cmd(args.ffmpeg, tmp)
    try:
        qt = QTest(args.qemu, 't31')
        try:
            # the INTC unmasked for source 62 (bank 1 bit 30)
            qt.writel(INTC + 0x2C, 1 << 30)
            core_init(qt)
            test_h264(qt, ff, tmp)
            test_hevc(qt, ff, tmp)
            test_hevc(qt, ff, tmp, libimp=True)
            test_jpeg(qt, ff, tmp)
        finally:
            qt.close()
        test_cavlc_change(args.qemu, ff, tmp)
        test_h264_slices(args.qemu, ff, tmp)
        test_h264_b(args.qemu, ff, tmp)
        test_rebased(args.qemu)
        test_t41(args.qemu, ff, tmp)
    finally:
        if not args.keep:
            shutil.rmtree(tmp, ignore_errors=True)
    print('%d failure(s)' % failures)
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
