#!/usr/bin/env python3
"""A SmartSens sensor's timing as a frame clock drives it (sensor-sync.h,
docs/hisilicon-digital-twin.md layer 2), checked at register level on
hi3516ev200 with an SC2315E.

The sensor is written over I2C through hisi-i2c the way i2c-hibvt does
(a command program, bytes from the TX FIFO).  hisi-vi's frames drive it.
What it applies is read back over QMP: the "lines" (rows; the register
counts half rows) / "vts" in effect, "frame" and "violations".

- writes to exposure take effect at the next frame start, not at once;
- a group hold (0x3812 = 0x00 ... 0x30) keeps them back until released;
- the frame length is HTS x VTS / pclk: VTS 1125 is 30 fps, 2250 15;
- an exposure beyond 2 x VTS - 4 half rows and a mode register written
  while streaming count as violations.

    python3 qemu/tests/test-hisi-sensor.py [--qemu PATH]
"""
import argparse
import json
import os
import socket
import sys
import tempfile
import time

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(__file__))
from importlib import import_module
QTest = import_module('test-xm-h1-jpeg').QTest

I2C = 0x12060000                # i2c0, the sensor's bus
VI_CAP, VPSS = 0x11000000, 0x11400000
SENSOR = 0x30
SEC = 1000 * 1000 * 1000

CMD_TX_S, CMD_TX_D1_1, CMD_TX_FIFO, CMD_TX_P, CMD_EXIT = 0x01, 0x05, 0x09, \
    0x1F, 0x00


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

    tmp = tempfile.mkdtemp()
    qmp_path = os.path.join(tmp, 'qmp.sock')
    q = QTest(a.qemu, 'hi3516ev200,sensor=sc2315e',
              ('-qmp', 'unix:%s,server=on,wait=off' % qmp_path))
    qmp = socket.socket(socket.AF_UNIX)
    for _ in range(50):
        try:
            qmp.connect(qmp_path)
            break
        except OSError:
            time.sleep(0.1)
    qf = qmp.makefile('rw')
    qf.readline()

    def qmp_cmd(cmd, **args):
        qf.write(json.dumps({'execute': cmd, 'arguments': args}) + '\n')
        qf.flush()
        while True:
            r = json.loads(qf.readline())
            if 'return' in r or 'error' in r:
                return r
    qmp_cmd('qmp_capabilities')

    # the sensor's QOM path
    sensor = None
    for c in qmp_cmd('qom-list', path='/machine/unattached')['return']:
        if c['type'] == 'child<hisi-smartsens>':
            sensor = '/machine/unattached/' + c['name']
    check(sensor is not None, 'the SC2315E is on the machine: %s' % sensor)

    def get(prop):
        return qmp_cmd('qom-get', path=sensor, property=prop)['return']

    def wr(reg, val):
        """i2c-hibvt's write: START, address, three bytes, STOP"""
        q.writel(I2C + 0x000, 1)                    # GLB: enable
        q.writel(I2C + 0x010, SENSOR << 1)          # DATA1
        for b in (reg >> 8, reg & 0xFF, val):
            q.writel(I2C + 0x020, b)                # TX FIFO
        for i, c in enumerate((CMD_TX_S, CMD_TX_D1_1, CMD_TX_FIFO,
                               CMD_TX_FIFO, CMD_TX_FIFO, CMD_TX_P,
                               CMD_EXIT)):
            q.writel(I2C + 0x030 + 4 * i, c)
        q.writel(I2C + 0x0E0, 0xFFFFFFFF)           # clear status
        q.writel(I2C + 0x0D0, 1)                    # CTRL1: start

    def frames(n, fps=30):
        q.cmd('clock_step %d' % (SEC * n // fps))

    # the SC2315E's mode, as its init table sets it: 1080p, HTS 2200,
    # VTS 1125, then stream
    for reg, val in ((0x320C, 0x08), (0x320D, 0x98), (0x320E, 0x04),
                     (0x320F, 0x65), (0x3208, 0x07), (0x3209, 0x80),
                     (0x320A, 0x04), (0x320B, 0x38), (0x3E01, 0x10),
                     (0x0100, 0x01)):
        wr(reg, val)
    # hisi-vi's frames: the write channel on
    q.writel(VI_CAP + 0x10000, 1 << 31)
    frames(2)
    check(get('vts') == 1125 and get('lines') == 0x80,
          'the mode applies: VTS %d, exposure %d' % (get('vts'),
                                                    get('lines')))

    # a write waits for the next frame
    f0 = get('frame')
    wr(0x3E01, 0x20)
    check(get('lines') == 0x80, 'exposure written mid-frame: still %d' %
          get('lines'))
    frames(1)
    check(get('lines') == 0x100 and get('frame') > f0,
          'next frame: exposure %d' % get('lines'))

    # a group hold keeps writes back until it closes
    wr(0x3812, 0x00)
    wr(0x3E01, 0x30)
    frames(3)
    held = get('lines')
    wr(0x3812, 0x30)
    frames(1)
    check(held == 0x100 and get('lines') == 0x180,
          'group hold: %d while open, %d once closed' % (held, get('lines')))

    # the frame clock follows HTS x VTS / pclk
    f0 = get('frame')
    frames(30)
    n30 = get('frame') - f0
    wr(0x320E, 0x08)                                # VTS 2250
    wr(0x320F, 0xCA)
    frames(2)
    f0 = get('frame')
    frames(30)
    n15 = get('frame') - f0
    check(abs(n30 - 30) <= 1 and abs(n15 - 15) <= 1,
          'frame rate from VTS: %d frames a second at 1125, %d at 2250' %
          (n30, n15))

    # datasheet rules: exposure beyond the frame, a mode register while
    # streaming
    v0 = get('violations')
    wr(0x3E00, 0x01)                                # 0x1300 half rows
    frames(2, 15)
    v1 = get('violations')
    wr(0x3E00, 0x00)
    frames(2, 15)                                   # the fix applies
    v2 = get('violations')
    frames(2, 15)
    v3 = get('violations')
    wr(0x320C, 0x09)                                # HTS while streaming
    v4 = get('violations')
    check(v1 > v0, 'exposure beyond the frame: %d violations' % (v1 - v0))
    check(v3 == v2, 'none once the exposure fits again (%d)' % (v3 - v2))
    check(v4 == v3 + 1, 'HTS written while streaming: %d' % (v4 - v3))

    # the limit in half rows, odd ones included: VTS 100 allows 196
    def expo(half):
        wr(0x3E00, half >> 12)
        wr(0x3E01, (half >> 4) & 0xFF)
        wr(0x3E02, (half & 0xF) << 4)
    wr(0x320A, 0x00)                                # an 80-row window
    wr(0x320B, 80)
    wr(0x320E, 0x00)
    wr(0x320F, 100)
    expo(196)
    frames(3, 15)
    v5 = get('violations')
    frames(2, 15)
    v6 = get('violations')
    expo(197)
    frames(3, 15)
    v7 = get('violations')
    check(v6 == v5 and v7 > v6,
          'VTS 100: 196 half rows fit, 197 do not (+%d, +%d)' %
          (v6 - v5, v7 - v6))

    qmp.close()
    q.close()
    print('%d failure(s)' % fails)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
