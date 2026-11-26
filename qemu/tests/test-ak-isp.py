#!/usr/bin/env python3
"""Drive the ak-isp model (the Anyka ISP2) the way OpenIPC's kernel does, on
EV300 (ak_camera.c over ak39_isp2.c) and on EV200 (ak-cam), and check what
lands in the buffers -- including what the EV300 release kernel gets wrong.

No guest: QEMU runs the machine under qtest and this script plays the
driver (branch anyka-ak3918ev300 of OpenIPC's kernel, release b0956f0):

  - the JX-F37 on I2C 0 at 0x40 answers 0x0F / 0x37 at 0x0A / 0x0B, and
    nothing while GPIO 79 (its reset on the PE3020-W) is driven low, as
    f22_set_power_on() does it;
  - the EV300 register table (3540 bytes, blocks 4 / 5 at 3188 / 3396),
    the buffer enables at the top of the interrupt-enable register counting
    down, 0x7 << MAIN_UV_MODE, the sub picture at main + w * h * 3 / 2 and
    its size from block 4 word 50 (what VIDIOC_S_PARM raw {1, w, h} sets):
    NV12 frames in buffer 1 then 2, STATUS naming the next buffer;
  - the crop: unset (the release kernel without VIDIOC_S_CROP), nothing
    is written but the frame completes; half the frame, the right half is
    scaled up;
  - the table is fetched at REG_UPDATE, not when the frame is written;
  - UV_MODE clear gives I420 at the U and V addresses;
  - a sensor held in reset, or never set up, sends no frames;
  - EV200: enables in GEN_CTRL, its smaller table, I420.

    python3 qemu/tests/test-ak-isp.py [--qemu PATH]
"""
import argparse, os, struct, sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from qtest_harness import QTest  # noqa: E402

ISP = 0x20000000
I2C0 = 0x20150000
GPIO = 0x20170000
DDR = 0x80000000
TABLE = DDR + 0x00100000
BUF = [DDR + 0x00200000 + n * 0x40000 for n in range(4)]
W, H = 320, 240                 # the sensor window, and the main channel
SW, SH = 160, 120               # the sub channel
FRAME_NS = 40000000             # 25 fps

GEN_CTRL, STATUS, INTEN, DMA_TABLE, REG_UPDATE = 0x00, 0x10, 0x14, 0x18, 0x1C
ISP_ENABLE = 1 << 20
UV_MODES = 0x7 << 10

BARS = [(180, 128, 128), (168, 44, 136), (145, 147, 44), (133, 63, 52),
        (63, 193, 204), (51, 109, 212), (28, 212, 120), (16, 128, 128)]


class Layout:
    def __init__(self, ev300):
        self.ev300 = ev300
        self.size = 3540 if ev300 else 3424
        self.blk4 = 3188 if ev300 else 3152
        self.blk5 = 3396 if ev300 else 3292
        self.main_dst = 48 if ev300 else 31
        self.sub_dst = 50 if ev300 else 33
        self.bits = 13 if ev300 else 12
        self.frame_ctrl = 51 if ev300 else 34


class Table:
    """isp->area as ak39_isp2.c lays it out."""

    def __init__(self, lay):
        self.l = lay
        self.b = bytearray(lay.size)

    def put(self, off, v):
        struct.pack_into('<I', self.b, off, v & 0xffffffff)

    def crop(self, x, y, w, h):          # ak_isp_vi_set_crop()
        self.put(0, x | y << 16)
        self.put(4, w | h << 16)

    def size(self, sub, w, h):           # ak_isp_vo_set_*_channel_scale()
        word = self.l.sub_dst if sub else self.l.main_dst
        self.put(self.l.blk4 + 4 * word, w | h << self.l.bits)

    def frame_ctrl(self, v):             # _isp_set_pp_frame_ctrl(0xff,0xff,0)
        self.put(self.l.blk4 + 4 * self.l.frame_ctrl, v)

    def buffer(self, n, main, sub):      # ak_isp_vo_set_buffer_addr()
        m = [main, main + W * H, main + W * H * 5 // 4]
        s = [sub, sub + SW * SH, sub + SW * SH * 5 // 4]
        if n == 0:
            for i, a in enumerate(m + s):
                self.put(self.l.blk5 + 4 * (25 + i), a)
        else:
            for i in range(3):
                self.put(4 * (31 + 3 * i + n - 1), m[i])
                self.put(4 * (40 + 3 * i + n - 1), s[i])

    def load(self, q):
        q.write(TABLE, bytes(self.b))


def i2c_xfer(q, addr, data=b'', read=0):
    """i2c-ak39.c: one message, the address byte in CMD1."""
    if read:
        q.writel(I2C0 + 0x10, addr << 1 | 1)
        q.writel(I2C0 + 0x00, 1 << 14 | 1 << 13 | (read - 1) << 9)
        return q.readl(I2C0 + 0x20) & ((1 << 8 * read) - 1)
    q.writel(I2C0 + 0x20, int.from_bytes(data[:4], 'little'))
    q.writel(I2C0 + 0x10, addr << 1)
    q.writel(I2C0 + 0x00, 1 << 14 | (len(data) - 1) << 9)
    return None


def sensor_read(q, reg):
    i2c_xfer(q, 0x40, bytes([reg]))
    return i2c_xfer(q, 0x40, read=1)


def sensor_write(q, reg, v):
    i2c_xfer(q, 0x40, bytes([reg, v]))


def gpio79(q, level):
    """ak_sensor_set_pin_dir(79, 1); ak_sensor_set_pin_level(79, level)"""
    q.writel(GPIO + 0x08, q.readl(GPIO + 0x08) | 1 << 15)
    out = q.readl(GPIO + 0x14) & ~(1 << 15)
    q.writel(GPIO + 0x14, out | level << 15)


def f37_window(q, w, h):
    """The tuning's register list: 0x24..0x26 the output window."""
    sensor_write(q, 0x24, (w // 2) & 0xff)
    sensor_write(q, 0x25, h & 0xff)
    sensor_write(q, 0x26, (h >> 8) << 4 | (w // 2) >> 8)


def step(q, ns=FRAME_NS + 1000):
    q.cmd('clock_step %d' % ns)


def expect_bars(w):
    """The top row of the pattern, at the bar each column falls in."""
    return bytes(BARS[x * 8 // w][0] for x in range(w))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--qemu', default=os.path.join(os.path.dirname(__file__),
                    '..', '..', 'qemu-src', 'build', 'qemu-system-arm'))
    a = ap.parse_args()
    fails = 0

    def check(ok, what):
        nonlocal fails
        print(('ok   ' if ok else 'FAIL ') + what)
        fails += not ok

    lay = Layout(True)
    q = QTest(a.qemu, 'ak3918ev300')
    try:
        # ---- the sensor, and its reset on GPIO 79 ---------------------
        check(sensor_read(q, 0x0A) == 0x0F and sensor_read(q, 0x0B) == 0x37,
              'F37 answers 0x0F37 at 0x40 (GPIO 79 at its pull-up)')
        gpio79(q, 0)
        check(sensor_read(q, 0x0A) == 0xFF and sensor_read(q, 0x0B) == 0xFF,
              'GPIO 79 low holds it in reset: the ID reads 0xFF / 0xFF')
        gpio79(q, 1)
        check(sensor_read(q, 0x0A) == 0x0F and sensor_read(q, 0x0B) == 0x37,
              'GPIO 79 high releases it')

        # ---- capture before the sensor is set up -----------------------
        t = Table(lay)
        t.crop(0, 0, W, H)
        t.size(False, W, H)
        t.size(True, SW, SH)
        t.frame_ctrl(0x00FFFF)
        for n in range(2):
            t.buffer(n, BUF[n], BUF[n] + W * H * 3 // 2)
        t.load(q)
        for n in range(2):
            q.write(BUF[n], b'\xaa' * (W * H * 3 // 2 + SW * SH * 3 // 2 +
                                       16))
        q.writel(ISP + DMA_TABLE, TABLE)
        q.writel(ISP + REG_UPDATE, 1)
        check(q.readl(ISP + REG_UPDATE) == 0, 'REG_UPDATE load bit clears')
        # _isp_enable_buffer() for buffers 1, 2; FRAME_ALL_DONE enabled
        q.writel(ISP + INTEN, 1 << 31 | 1 << 30 | 1 << 27 | 1 << 26 | 1)
        q.writel(ISP + GEN_CTRL, UV_MODES | ISP_ENABLE)
        step(q)
        check(q.readl(ISP + STATUS) & 1 == 0,
              'sensor window not set: no frame, no FRAME_ALL_DONE')

        f37_window(q, W, H)
        step(q)
        st = q.readl(ISP + STATUS)
        check(st & 7 == 7, 'frame 0: FRAME_ALL_DONE, MAIN_DONE, SUB_DONE '
              '(STATUS 0x%x)' % st)
        check((st >> 22) & 3 == 1 and (st >> 20) & 3 == 1,
              'STATUS names buffer 2 (index 1) as next')
        y = q.read(BUF[0], W)
        check(y == expect_bars(W), 'main Y row 0: the eight bars')
        uv = q.read(BUF[0] + W * H, W)
        want = b''.join(bytes(BARS[(2 * x) * 8 // W][1:])
                        for x in range(W // 2))
        check(uv == want, 'main chroma: NV12, U V interleaved at Y + w * h')
        tail = q.read(BUF[0] + W * H * 3 // 2 - 4, 4)
        check(tail != b'\xaa' * 4, 'the NV12 plane fills w * h / 2 bytes')
        sy = q.read(BUF[0] + W * H * 3 // 2, SW)
        check(sy == expect_bars(SW), 'sub Y at main + w * h * 3 / 2, '
              '%dx%d' % (SW, SH))
        su = q.read(BUF[0] + W * H * 3 // 2 + SW * SH, SW)
        want = b''.join(bytes(BARS[(2 * x) * 8 // SW][1:])
                        for x in range(SW // 2))
        check(su == want, 'sub chroma NV12 too')
        after = q.read(BUF[0] + W * H * 3 // 2 + SW * SH * 3 // 2, 16)
        check(after == b'\xaa' * 16, 'nothing written past the sub picture')

        q.writel(ISP + STATUS, st | 0xffff)     # ispdrv_vo_clear_irq_status
        st2 = q.readl(ISP + STATUS)
        check(st2 & 0xffff == 0 and st2 >> 20 == st >> 20,
              'STATUS: done bits clear, buffer ids stay')

        step(q)
        st = q.readl(ISP + STATUS)
        check(st & 1 and (st >> 22) & 3 == 0, 'frame 1 into buffer 2, '
              'next is buffer 1 again')
        check(q.read(BUF[1], W) == expect_bars(W), 'buffer 2 written')
        # the counter strip: frame 1 sets the last cell
        last = q.read(BUF[1] + (H - 1) * W, W)
        check(last[-1] == 235 and last[0] == 16, 'frame counter cells: 1')

        # ---- disabling a buffer as irq_handle_continous_mode() does ----
        q.writel(ISP + INTEN, q.readl(ISP + INTEN) & ~(1 << 31 | 1 << 27))
        q.writel(ISP + STATUS, 0xffff)
        q.write(BUF[0], b'\x55' * 16)
        step(q)
        check(q.read(BUF[0], 16) == b'\x55' * 16,
              'buffer 1 disabled: skipped')
        check((q.readl(ISP + STATUS) >> 22) & 3 == 1, 'only buffer 2 left')
        q.writel(ISP + INTEN, q.readl(ISP + INTEN) | 1 << 31 | 1 << 27)

        # ---- the crop the release kernel never sets --------------------
        q.writel(ISP + GEN_CTRL, UV_MODES)
        q.writel(ISP + STATUS, 0xffff)
        t.crop(0, 0, 0, 0)
        t.load(q)
        q.writel(ISP + REG_UPDATE, 1)
        q.write(BUF[0], b'\xaa' * W)
        q.writel(ISP + GEN_CTRL, UV_MODES | ISP_ENABLE)
        step(q)
        check(q.readl(ISP + STATUS) & 1 == 1,
              'empty crop window: the frame still completes')
        check(q.read(BUF[0], W) == b'\xaa' * W,
              'empty crop window: nothing written')

        # ---- the table is fetched at REG_UPDATE -------------------------
        q.writel(ISP + GEN_CTRL, UV_MODES)
        q.writel(ISP + STATUS, 0xffff)
        t.crop(W // 2, 0, W // 2, H)            # the right half, scaled up
        t.load(q)
        q.writel(ISP + GEN_CTRL, UV_MODES | ISP_ENABLE)
        step(q)
        check(q.read(BUF[0], W) == b'\xaa' * W,
              'table changed but not loaded: still the empty crop')
        q.writel(ISP + REG_UPDATE, 4)               # MODE_LEAST, from the irq
        step(q)
        step(q)
        y = q.read(BUF[0], W)
        want = bytes(BARS[(W // 2 + x * (W // 2) // W) * 8 // W][0]
                     for x in range(W))
        check(y == want, 'crop of the right half, scaled to the full width')

        # ---- I420 with UV_MODE clear ------------------------------------
        q.writel(ISP + GEN_CTRL, 0)
        t.crop(0, 0, W, H)
        t.load(q)
        q.writel(ISP + REG_UPDATE, 1)
        q.writel(ISP + STATUS, 0xffff)
        q.writel(ISP + GEN_CTRL, ISP_ENABLE)
        step(q)
        step(q)
        u = q.read(BUF[0] + W * H, W // 2)
        v = q.read(BUF[0] + W * H * 5 // 4, W // 2)
        check(u == bytes(BARS[x * 16 // W][1] for x in range(W // 2)) and
              v == bytes(BARS[x * 16 // W][2] for x in range(W // 2)),
              'UV_MODE clear: I420 planes at the U and V addresses')

        # ---- sensor reset mid-stream -------------------------------------
        q.writel(ISP + STATUS, 0xffff)
        gpio79(q, 0)
        gpio79(q, 1)
        step(q)
        check(q.readl(ISP + STATUS) & 1 == 0,
              'sensor reset (window back to 0): frames stop')
    finally:
        q.close()

    # ---- EV200: enables in GEN_CTRL, its own table, I420 ---------------
    lay = Layout(False)
    q = QTest(a.qemu, 'ak3918ev200,sensor=none')
    try:
        t = Table(lay)
        t.crop(0, 0, W, H)
        t.size(False, W, H)
        t.size(True, SW, SH)
        t.frame_ctrl(0x00FFFF)
        t.buffer(0, BUF[0], BUF[0] + W * H * 3 // 2)
        t.load(q)
        q.writel(ISP + 0x04, W | H << 14)           # CSI_ERR_CHECK_1
        q.writel(ISP + DMA_TABLE, TABLE)
        q.writel(ISP + REG_UPDATE, 1)
        q.writel(ISP + INTEN, 1)
        q.writel(ISP + GEN_CTRL, 1 << 15 | 1 << 21 | ISP_ENABLE)
        step(q, 1000000000 // 15 + 1000)
        st = q.readl(ISP + STATUS)
        check(st & 7 == 7 and (st >> 22) & 3 == 0,
              'EV200: frame into buffer 1, the only one (STATUS 0x%x)' % st)
        check(q.read(BUF[0], W) == expect_bars(W), 'EV200: main Y')
        u = q.read(BUF[0] + W * H, W // 2)
        check(u == bytes(BARS[x * 16 // W][1] for x in range(W // 2)),
              'EV200: I420 U plane')
        check(q.read(BUF[0] + W * H * 3 // 2, SW) == expect_bars(SW),
              'EV200: sub Y')
    finally:
        q.close()

    # ---- fault injection: a core that never finishes its frame ---------
    q = QTest(a.qemu, 'ak3918ev300', ['-global', 'ak-isp.stuck-busy=on'])
    try:
        check(q.readl(ISP + STATUS) & (1 << 14),
              'stuck-busy: STATUS bit 14 (ISP_UNDER_WORKING) reads set')
        q.writel(ISP + STATUS, 1 << 14)
        check(q.readl(ISP + STATUS) & (1 << 14),
              'stuck-busy: and stays set when written')
    finally:
        q.close()
    q = QTest(a.qemu, 'ak3918ev300')
    try:
        check(q.readl(ISP + STATUS) & (1 << 14) == 0,
              'without it, bit 14 reads clear')
    finally:
        q.close()

    print('%d failure(s)' % fails if fails else 'all ok')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
