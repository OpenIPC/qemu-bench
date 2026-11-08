#!/usr/bin/env python3
"""How a camera's 3A loop answered a scenario, from hisi-vi's record.

  twin-metrics.py RECORD.csv [--tol 0.15] [--settle-max 8] [--awb-tol 0.15]

The record (-global hisi-vi.record=FILE, docs/hisilicon-digital-twin.md
step 3) has a line a measured frame: the scene's clock and light, what the
sensor applied, the statistics' raw R / G / B means and the white balance
gains the driver programmed.

The scenario's segments are found in the record itself: a segment starts
when the light (level or colour) stops changing and ends when it changes
again (a ramp is one change).  For each segment:

- settle: seconds until G stays within --tol of the segment's steady level
  (the median of its last quarter);
- overshoot: the largest deviation from that level after the first
  settled frame, as a fraction;
- level: the steady level against the first segment's, i.e. whether auto
  exposure brought the picture back to its target;
- AWB: R x gain / G and B x gain / G at the end, where the gains are
  visible (online), i.e. how neutral white balance left a grey, and how
  long it took to stay within --awb-tol.

Exit status 1 when a segment after the first does not settle within
--settle-max seconds, its level is off by more than --tol, or its white
balance is off by more than --awb-tol.
"""
import argparse
import csv
import statistics
import sys


def segments(rows):
    """[start, end) row ranges with constant light, transitions excluded"""
    key = [(r['light'], r['light_r'], r['light_b']) for r in rows]
    segs, start = [], 0
    for i in range(1, len(rows)):
        if key[i] != key[i - 1]:
            if i - 1 > start:
                segs.append((start, i))
            start = i
            # a ramp: skip while it keeps changing
    if len(rows) > start + 1:
        segs.append((start, len(rows)))
    # merge away the frames of a ramp (segments of one or two frames)
    return [(a, b) for a, b in segs if b - a > 2]


def analyse(rows, a, b, tol, awb_tol):
    seg = rows[a:b]
    g = [float(r['g']) for r in seg]
    tail = g[len(g) * 3 // 4:] or g
    steady = statistics.median(tail)
    t0 = float(seg[0]['t'])
    settled = None
    for i in range(len(g)):
        if all(abs(v - steady) <= tol * steady for v in g[i:]):
            settled = i
            break
    over = 0.0
    if settled is not None:
        over = max(abs(v - steady) / steady for v in g[settled:])
    end = seg[-1]

    def residual(row):
        r, gg, bb = float(row['r']), float(row['g']), float(row['b'])
        wr, wb = float(row['wb_r']), float(row['wb_b'])
        if wr > 0 and wb > 0 and gg > 0:
            return (r * wr / gg - 1, bb * wb / gg - 1)
        return None

    awb = residual(end)
    awb_settle = None
    if awb:
        res = [residual(row) for row in seg]
        for i in range(len(res)):
            if all(x and max(abs(x[0]), abs(x[1])) <= awb_tol
                   for x in res[i:]):
                awb_settle = float(seg[i]['t']) - t0
                break
    return {
        't0': t0, 't1': float(end['t']),
        'light': '%s %s/%s' % (end['light'], end['light_r'], end['light_b']),
        'steady': steady,
        'settle': None if settled is None else float(seg[settled]['t']) - t0,
        'over': over,
        'lines': end['lines'], 'gain': end['gain'],
        'awb': awb,
        'awb_settle': awb_settle,
        'violations': int(end['violations']),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('record')
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--settle-max', type=float, default=8.0)
    ap.add_argument('--awb-tol', type=float, default=0.15)
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.record)))
    if len(rows) < 10:
        print('FAIL: %d frames in %s' % (len(rows), a.record))
        return 1
    segs = [analyse(rows, x, y, a.tol, a.awb_tol)
            for x, y in segments(rows)]
    ref = segs[0]['steady']
    fails = 0
    print('%-13s %-14s %8s %7s %6s %6s %8s %6s  %s' %
          ('t', 'light', 'G', 'level', 'settle', 'over', 'exposure',
           'gain', 'AWB R/B vs G'))
    for i, s in enumerate(segs):
        level = s['steady'] / ref - 1
        awb = ('%+.0f%% / %+.0f%%' % (s['awb'][0] * 100, s['awb'][1] * 100)
               if s['awb'] else 'n/a')
        if s['awb_settle'] is not None:
            awb += ' (in %.1fs)' % s['awb_settle']
        settle = '%.1fs' % s['settle'] if s['settle'] is not None else 'never'
        print('%5.1f-%-7.1f %-14s %8.0f %+6.0f%% %6s %5.0f%% %8s %6s  %s' %
              (s['t0'], s['t1'], s['light'], s['steady'], level * 100,
               settle, s['over'] * 100, s['lines'], s['gain'], awb))
        if i == 0:
            continue            # the first segment is boot and convergence
        why = []
        if s['settle'] is None or s['settle'] > a.settle_max:
            why.append('did not settle within %.0fs' % a.settle_max)
        if abs(level) > a.tol:
            why.append('level %+.0f%% off the first segment' % (level * 100))
        if s['awb'] and max(abs(x) for x in s['awb']) > a.awb_tol:
            why.append('white balance off by more than %.0f%%' %
                       (a.awb_tol * 100))
        if why:
            print('  FAIL %.1fs: %s' % (s['t0'], '; '.join(why)))
            fails += 1
    if segs[-1]['violations']:
        print('note: the sensor counted %d datasheet violations' %
              segs[-1]['violations'])
    print('%s: %d segments, %d failed' % ('FAIL' if fails else 'PASS',
                                          len(segs), fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
