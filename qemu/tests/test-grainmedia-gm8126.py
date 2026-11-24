#!/usr/bin/env python3
"""Register-level checks of the Grain Media GM8126 models, no guest needed.

As test-grainmedia.py (whose qtest client this borrows), for the blocks the
gm8126 machine has instead of the GM813x's (docs/grainmedia.md, "GM8126"):

  - the mask ROM: nsboot unswapped from the word-swapped NOR into SRAM;
  - gm-sys8126: chip ID, the DDR size in the AHB controller, the frequency
    change (0x0C bit 2 -> interrupt 8 -> 0x20 bit 17);
  - gm-intc010: both banks, level and edge inputs, through the CPU's view;
  - gm-ssp-spi + the NOR: JEDEC ID by PIO with the GPIO select, 32-bit
    frames most significant byte first, a read by the DMAC on lines 0 / 1;
  - gm-sdc010: an empty slot; gm-cap020: a link list run, the picture, the
    status and the interrupt, a list not yet collected left alone;
  - gm-scal010: a 4:2:2 job; gm-mcp100: MPEG-4 I and P VOPs through the
    mailbox, the command latched, decoded with ffmpeg when it is there and
    compared with the source; gm-h264e mcp200: an IDR and a P picture
    through the header packer, decoded likewise;
  - gm-ftmac110: descriptors both ways over a socket netdev;
  - a migration round trip.

    python3 qemu/tests/test-grainmedia-gm8126.py [--qemu PATH] [--ffmpeg PATH]
"""
import argparse
import importlib.util
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'gm813x', os.path.join(HERE, 'test-grainmedia.py'))
gm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gm)
check, QTest = gm.check, gm.QTest

AHBC, PMU, INTC = 0x90000000, 0x99000000, 0x99500000
GPIO0, SSP, SDC, DMAC = 0x99400000, 0x98200000, 0x92800000, 0x92600000
CAP, SCAL, MAC = 0x90B00000, 0x92400000, 0x92500000
SRAM, NODE, MCP = 0x94300000, 0x94308000, 0x94320000
H264, WRAP = 0x94000000, 0x94100000
BUF = 0x01000000
W, H = 64, 48


def make_image(path):
    img = bytearray(b'\xff' * (16 << 20))
    img[0:24] = struct.pack('<8s4I', b'GM8126', 0x6000, 0x6000, 0xA6000,
                            0xD6000)
    img[0xFE:0x100] = b'\x55\xaa'
    code = struct.pack('<4I', 0xEA00000E, 0x11223344, 0xE1A00000, 0xDEADBEEF)
    img[0x1000:0x1010] = struct.pack('<IIII', 0x805A474D, 0, len(code), 0)
    img[0x1100:0x1100 + len(code)] = code
    img[0x2000:0x2100] = bytes(range(256))
    out = bytearray(len(img))
    for i in range(4):
        out[i::4] = img[3 - i::4]
    with open(path, 'wb') as f:
        f.write(out)
    return code


def test_boot_rom(qt, code):
    print('mask ROM')
    check('nsboot in SRAM', qt.read(SRAM, len(code)), code)
    check('entered at 0', qt.readl(4), SRAM)
    qt.writel(0x10000000 + 0x2000, 0x12345678)
    check('DDR before the remap', qt.readl(0x2000), 0x12345678)


def test_sys(qt):
    print('PMU / AHBC / INTC')
    check('chip ID', qt.readl(PMU) >> 8, 0x812620)
    check('DDR size', qt.readl(AHBC + 0x18) >> 16 & 15, 7)
    qt.writel(INTC + 0x04, 1 << 8)
    qt.writel(PMU + 0x0C, 4)
    check('not at once', qt.readl(INTC + 0x14), 0)
    qt.step(1000000)
    check('interrupt 8', qt.readl(INTC + 0x14), 1 << 8)
    check('status', qt.readl(PMU + 0x20), 1 << 17)
    qt.writel(PMU + 0x20, 1 << 17)
    check('cleared', qt.readl(INTC + 0x14), 0)


def cs(qt, on):
    qt.writel(GPIO0 + 0x08, 1 << 14)
    qt.writel(GPIO0 + (0x14 if on else 0x10), 1 << 14)


def spi_word(qt, v, bits):
    qt.writel(SSP + 4, (bits - 1) << 16)
    qt.writel(SSP + 0x18, v)
    check_n = qt.readl(SSP + 0x0C) >> 4 & 0x1F
    return qt.readl(SSP + 0x18) if check_n else None


def test_spi(qt):
    print('gm-ssp-spi')
    cs(qt, False)
    qt.writel(SSP + 8, 0xD)
    cs(qt, True)
    spi_word(qt, 0x9F, 8)
    check('JEDEC ID', [spi_word(qt, 0, 8) for _ in range(3)],
          [0xC2, 0x20, 0x18])
    cs(qt, False)
    cs(qt, True)
    spi_word(qt, 0x03002000, 32)
    check('32-bit frames undo the swap', spi_word(qt, 0, 32), 0x03020100)
    cs(qt, False)
    # nsboot's: receive on channel 2 (line 0), dummies on 3 (line 1)
    cs(qt, True)
    spi_word(qt, 0x03002000, 32)
    n = 0x100
    qt.writel(BUF - 4, 0)
    qt.writel(DMAC + 0x24, 1)
    for ch, cfg, src, dst, ccr in ((2, 0x80, SSP + 0x18, BUF, 0x12C1),
                                   (3, 0x2200, BUF - 4, SSP + 0x18, 0x12D1)):
        b = DMAC + 0x100 + ch * 0x20
        qt.writel(b + 4, cfg)
        qt.writel(b + 8, src)
        qt.writel(b + 12, dst)
        qt.writel(b + 0x14, n // 4)
        qt.writel(b + 0x10, 0)
        qt.writel(b, ccr)
    qt.writel(SSP + 0x10, 0x1130)
    for _ in range(200):
        qt.step(1000)
        if qt.readl(DMAC + 0x14) & 4:
            break
    check('a DMA read', qt.read(BUF, n), bytes(range(256)))
    qt.writel(SSP + 0x10, 0)
    cs(qt, False)


def test_sdc(qt):
    print('gm-sdc010')
    qt.writel(SDC, 0x400)
    check('reset done', qt.readl(SDC), 0)
    check('idle, no card', qt.readl(SDC + 0x28), 1 << 17)
    qt.writel(SDC, 0x248)
    check('command times out', qt.readl(SDC + 0x28) & 4, 4)
    qt.writel(SDC + 0x2C, 4)
    check('cleared', qt.readl(SDC + 0x28) & 4, 0)


def test_gates(qt):
    print('clock gates')
    qt.writel(SDC + 4, 0x1234)
    qt.writel(0x99000038, 1 << 12)
    check('a gated SDC reads 0', qt.readl(SDC + 4), 0)
    qt.writel(SDC + 4, 0x5678)
    qt.writel(0x99000038, 0)
    check('and kept its register', qt.readl(SDC + 4), 0x1234)


def test_capture(qt):
    print('gm-cap020')
    lst, dst = BUF, BUF + 0x10000
    qt.write(lst, struct.pack('<9I', 0, dst >> 2 | 2 << 30,
                              dst >> 2 | 2 << 30, dst >> 2 | 2 << 30,
                              0x000F0018, W | H << 16,
                              0x000F0218, W * 2 // 4,
                              lst >> 2 | 3 << 30))
    qt.write(dst, bytes(W * H * 2))
    qt.writel(CAP + 0x3E4, 0xFFFFFFEF)
    qt.writel(CAP + 0x284, 0)
    qt.writel(CAP + 0x280, lst >> 2)
    qt.writel(CAP + 0x40, 0)
    qt.writel(CAP, 0x90328000)
    qt.step(40000000)
    check('status: frame done', qt.readl(lst), 0x92000000)
    check('frame-start interrupt', qt.readl(INTC + 0x60) & 1, 1)
    pic = qt.read(dst, W * H * 2)
    check('a picture', len(set(pic)) > 8, True)
    qt.writel(CAP + 0x3E0, 0x10)
    check('acknowledged', qt.readl(INTC + 0x60) & 1, 0)
    qt.write(dst, bytes(W * H * 2))
    qt.step(40000000)
    check('a list not collected is left', qt.read(dst, 64), bytes(64))
    qt.writel(CAP, 0)
    return pic


def test_scaler(qt):
    print('gm-scal010')
    src, dst = BUF, BUF + 0x10000
    row = b''.join(bytes([0x60, 2 * x, 0xA0, 2 * x + 1]) for x in range(32))
    qt.write(src, row * 32)
    for off, v in ((0x68, 64 | 32 << 16), (0x28, 32 | 16 << 16),
                   (0x204, 0), (0x20C, src >> 2), (0x210, 32),
                   (0x250, dst >> 2), (0x21C, 16), (0x3E4, 0xFEFFFFFF)):
        qt.writel(SCAL + off, v)
    qt.writel(SCAL, 1 << 31 | 1 << 24)
    check('busy', qt.readl(SCAL + 0x3E0), 0)
    qt.step(10000000)
    check('done', qt.readl(SCAL + 0x3E0) & 1 << 24, 1 << 24)
    out = qt.read(dst, 64)
    check('every other pixel', list(out[1::2]), list(range(0, 64, 2)))
    qt.writel(SCAL + 0x3E0, 0xFFFFFFFF)


def picture(t):
    """packed UYVY with detail and a block that moves"""
    out = bytearray()
    for y in range(H):
        for x in range(0, W, 2):
            def lum(px):
                if 8 + t * 4 <= px < 24 + t * 4 and 16 <= y < 32:
                    return 220
                return 40 + px * 2 + (y & 8)
            out += bytes([110 + y // 4, lum(x), 150 - x // 4, lum(x + 1)])
    return bytes(out)


def luma(pic):
    return pic[1::2]


def decode(ffmpeg, data, fmt):
    r = subprocess.run([ffmpeg, '-v', 'error', '-xerror', '-f', fmt, '-i',
                        '-', '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                       input=data, capture_output=True, timeout=60)
    return r.stdout, r.stderr.decode()


def compare(ffmpeg, data, fmt, pics, tol):
    if not ffmpeg:
        print('  SKIP: no ffmpeg')
        return
    raw, err = decode(ffmpeg, data, fmt)
    fs = W * H * 3 // 2
    check('decodes', err, '')
    check('frames', len(raw) // fs, len(pics))
    for i, p in enumerate(pics):
        d = raw[i * fs:i * fs + W * H]
        e = sum(abs(a - b) for a, b in zip(d, luma(p))) / (W * H)
        check('frame %d: mean luma error %.2f' % (i, e), e < tol, True)


def test_mpeg4(qt, ffmpeg):
    print('gm-mcp100 MPEG-4')
    src, bs = BUF, BUF + 0x40000
    rec = [BUF + 0x80000, BUF + 0xC0000]
    stream, pics = b'', [picture(t) for t in range(3)]
    qt.writel(MCP + 0x7C, 3 << 18)
    for t, pic in enumerate(pics):
        qt.write(src, pic)
        node = {0x2D8: W, 0x2DC: H, 0x2F0: W, 0x2F4: H, 0x300: W // 16,
                0x304: H // 16, 0x308: 1, 0x30C: 30, 0x310: 0,
                0x384: 0xFFFFFFFF, 0x38C: bs, 0x390: 3, 0x24: 0,
                0x3A4: 2, 0x3B0: 0, 0x3B4: t, 0x3C4: src}
        for base, r in ((0x3A4, rec[t & 1]), (0x3FC, rec[~t & 1])):
            node[base + 0x38] = r
            node[base + 0x3C] = r + W * H
            node[base + 0x40] = r + W * H * 5 // 4
        for off, v in node.items():
            qt.writel(NODE + off, v)
        qt.writel(MCP + 0x78, bs | 1)
        qt.writel(NODE, 5 if t else 4)
        if t == 1:
            qt.writel(NODE, 0)      # another encoder's set-up meanwhile
        qt.step(20000000)
        check('VOP %d done' % t, qt.readl(NODE) & 0xFF, 0x50 if t else 0x40)
        check('interrupt', qt.readl(MCP + 0x98), 0x80000000)
        n = qt.readl(MCP + 0x78) + (qt.readl(MCP + 0x50) & 0x3C) + \
            ((qt.readl(MCP + 0x44) & 0x1F) + 7 >> 3) - bs
        for _ in range(8):              # the host's flush
            qt.writel(MCP + 0x50, 16)
        qt.writel(MCP + 0x98, 0x80000000)
        stream += qt.read(bs, n)
    check('starts with the layer headers', stream[:8],
          bytes.fromhex('0000010000000120'))
    compare(ffmpeg, stream, 'm4v', pics, 3)


def h264_put(qt, v, ctl):
    qt.writel(H264 + 0x1C, v)
    qt.writel(H264 + 0x24, ctl)


def test_h264(qt, ffmpeg):
    print('gm-h264e mcp200')
    src, bs, chain = BUF, BUF + 0x40000, BUF + 0x60000
    rec = [BUF + 0x80000, BUF + 0xC0000]
    U1, UE = 0x141, 0x1C0
    stream, pics = b'', [picture(t) for t in range(3)]
    check('idle', qt.readl(H264 + 0x40) & 1, 1)

    def nal(byte):
        h264_put(qt, 0, 0x48)
        h264_put(qt, 1, 0x58)
        h264_put(qt, byte, 0x48)

    for t, pic in enumerate(pics):
        qt.write(src, pic)
        qt.writel(chain + 0x118, rec[t & 1] | 3)
        qt.writel(chain + 0x88, rec[~t & 1] | 3)
        qt.writel(H264, (W // 16 - 1) | (H // 16 - 1) << 8 |
                  (W * H // 256 - 1) << 16)
        qt.writel(H264 + 0x10, 22 << 4)
        qt.writel(H264 + 0x98, chain)
        for off, v in ((0, 0), (4, 0), (8, W // 16), (0xC, H // 16),
                       (0x18, src >> 2), (0x1C, 0x65), (0x20, 1)):
            qt.writel(WRAP + off, v)
        check('wrapper idle', qt.readl(WRAP + 0x20), 0)
        qt.writel(H264 + 0xA8, bs | 1)
        if not t:
            nal(0x67)
            h264_put(qt, 66, 0x148)
            h264_put(qt, 0, 0x148)
            h264_put(qt, 30, 0x148)
            for v in (0, 0, 0, 4, 1):
                h264_put(qt, v, UE)
            h264_put(qt, 0, U1)
            h264_put(qt, W // 16 - 1, UE)
            h264_put(qt, H // 16 - 1, UE)
            for v in (1, 1, 0, 0):
                h264_put(qt, v, U1)
            qt.writel(H264 + 0x24, qt.readl(H264 + 0x24) | 0x200)
            nal(0x68)
            h264_put(qt, 0, UE)
            h264_put(qt, 0, UE)
            h264_put(qt, 0, U1)
            h264_put(qt, 0, U1)
            for v in (0, 0, 0):
                h264_put(qt, v, UE)
            h264_put(qt, 0, U1)
            h264_put(qt, 0, 0x142)
            for v in (0, 0, 0):
                h264_put(qt, v, UE)
            for v in (1, 0, 0):
                h264_put(qt, v, U1)
            qt.writel(H264 + 0x24, qt.readl(H264 + 0x24) | 0x200)
        nal(0x41 if t else 0x65)
        h264_put(qt, 0, UE)
        h264_put(qt, 5 if t else 7, UE)
        h264_put(qt, 0, UE)
        h264_put(qt, t, 0x144)
        if not t:
            h264_put(qt, 0, UE)
        h264_put(qt, 2 * t, 0x148)
        if t:
            h264_put(qt, 0, U1)
            h264_put(qt, 0, U1)
            h264_put(qt, 0, U1)
        else:
            h264_put(qt, 0, U1)
            h264_put(qt, 0, U1)
        h264_put(qt, 2 * (26 - 22), UE)     # slice_qp_delta, negative
        for v in (0, 0, 0):
            h264_put(qt, v, UE)
        qt.writel(H264 + 0x20, 3 if t else 1)
        qt.step(10000000)
        check('picture %d: idle, interrupt' % t,
              (qt.readl(H264 + 0x40) & 1, qt.readl(H264 + 0x48) & 1), (1, 1))
        qt.writel(H264 + 0x48, 0)
        check('cleared', qt.readl(H264 + 0x48) & 1, 0)
        stream += qt.read(bs, qt.readl(H264 + 0x44))
        qt.writel(H264 + 0x24, qt.readl(H264 + 0x24) | 0x800)
        check('count cleared', qt.readl(H264 + 0x44), 0)
    # I_16x16 DC macroblocks: coarser than MPEG-4's
    compare(ffmpeg, stream, 'h264', pics, 12)


def test_mac(qemu):
    print('gm-ftmac110')
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    tmp = tempfile.mkdtemp()
    pa, pb = os.path.join(tmp, 'a'), os.path.join(tmp, 'b')
    a.close()
    b.close()
    peer = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    peer.bind(pb)
    peer.settimeout(5)
    qt = QTest(qemu, 'gm8126', (
        '-netdev', 'dgram,id=n,local.type=unix,local.path=%s,'
        'remote.type=unix,remote.path=%s' % (pa, pb), '-net', 'none',
        '-global', 'gm-ftmac110.netdev=n'))
    try:
        mac = bytes.fromhex('0200000000aa')
        qt.writel(MAC + 8, mac[0] << 8 | mac[1])
        qt.writel(MAC + 12, int.from_bytes(mac[2:], 'big'))
        qt.writel(MAC + 0x90, 1 << 26 | 1 << 21)
        check('PHY link', qt.readl(MAC + 0x90) & 0x24, 0x24)
        tx, rx, tb, rb = BUF, BUF + 0x100, BUF + 0x1000, BUF + 0x2000
        frame = b'\xff' * 6 + mac + b'\x08\x06' + bytes(range(50))
        qt.write(tb, frame)
        qt.write(tx, struct.pack('<4I', 1 << 31, 1 << 31 | 3 << 27 |
                                 len(frame), tb, 0))
        qt.write(rx, struct.pack('<4I', 1 << 31, 1 << 31 | 0x600, rb, 0))
        qt.writel(MAC + 0x20, tx)
        qt.writel(MAC + 0x24, rx)
        qt.writel(MAC + 4, 0x3FF)
        qt.writel(MAC + 0x88, 0x7E323)
        qt.writel(MAC + 0x18, 1)
        check('sent', peer.recv(2048), frame)
        check('the descriptor is back', qt.readl(tx) >> 31, 0)
        peer.sendto(mac + frame[6:], pa)
        for _ in range(100):
            if not qt.readl(rx) >> 31:
                break
        d = qt.readl(rx)
        check('received', (d >> 28 & 3, d & 0x7FF), (3, len(frame) + 4))
        check('two bytes in', qt.read(rb + 2, len(frame)), mac + frame[6:])
        check('interrupt status', qt.readl(MAC) & 1, 1)
    finally:
        qt.close()
        peer.close()
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default='qemu-src/build/qemu-system-arm')
    ap.add_argument('--ffmpeg', default=shutil.which('ffmpeg'))
    args = ap.parse_args()
    tmp = tempfile.mkdtemp()
    try:
        path = os.path.join(tmp, 'nor.bin')
        code = make_image(path)
        machine = 'gm8126,sensor=ov7725,flash-file=' + path
        qt = QTest(args.qemu, machine, ('-net', 'none'))
        try:
            test_boot_rom(qt, code)
            test_sys(qt)
            test_spi(qt)
            test_sdc(qt)
            test_gates(qt)
            test_capture(qt)
            test_scaler(qt)
            test_mpeg4(qt, args.ffmpeg)
            test_h264(qt, args.ffmpeg)
        finally:
            qt.close()
        test_mac(args.qemu)
        print('migration')
        state = os.path.join(tmp, 'state')
        base = [args.qemu, '-M', machine, '-display', 'none', '-nodefaults',
                '-S', '-monitor', 'stdio']
        subprocess.run(base, input='migrate "exec:cat > %s"\nquit\n' % state,
                       capture_output=True, text=True, timeout=60)
        check('state saved', os.path.getsize(state) > 0, True)
        r = subprocess.run(base + ['-incoming', 'exec:cat %s' % state],
                           input='info status\nquit\n', capture_output=True,
                           text=True, timeout=60)
        check('and loaded back', 'paused' in r.stdout and
              'error' not in (r.stdout + r.stderr).lower(), True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('%d failure(s)' % gm.failures)
    sys.exit(1 if gm.failures else 0)


if __name__ == '__main__':
    main()
