#!/usr/bin/env python3
"""How a camera's 3A loop answered a scenario, from hisi-vi's record.

  twin-metrics.py RECORD.csv [--timeline FILE] [--tol 0.15]
                  [--settle-max 8] [--awb-tol 0.15] [--no-awb]

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
- AWB: R x gain / G and B x gain / G at the end (the gains the driver
  programmed; offline, from the strips' register images), i.e. how
  neutral white balance left a grey, and how long it took to stay within
  --awb-tol.

Exit status 1 when a segment after the first does not settle within
--settle-max seconds, its level is off by more than --tol, its white
balance is off by more than --awb-tol or has no gains (unless --no-awb;
--awb-advisory reports those without failing),
or it is too short to measure; when the record has no steady first
segment; or, with --timeline, when a change the timeline plans did not
happen.
"""
import argparse
import csv
import statistics
import sys


MIN_FRAMES = 5


def segments(rows):
    """Runs of rows with constant light: (start, end, too_short).  A run of
    one or two frames between other such runs is a ramp's step and left
    out; a short run anywhere else is a real state with too few frames."""
    key = [(r['light'], r['light_r'], r['light_b']) for r in rows]
    runs, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or key[i] != key[i - 1]:
            runs.append((start, i))
            start = i
    out = []
    for k, (a, b) in enumerate(runs):
        short = b - a <= 2
        prev_short = k > 0 and runs[k - 1][1] - runs[k - 1][0] <= 2
        next_short = k + 1 < len(runs) and \
            runs[k + 1][1] - runs[k + 1][0] <= 2
        if short and (prev_short or next_short):
            continue                # inside a ramp
        out.append((a, b, b - a < MIN_FRAMES))
    return out


def planned(path):
    """The times a timeline changes the light: steps, and ramps' ends"""
    times = []
    for line in open(path):
        line = line.split('#')[0].split()
        if not line:
            continue
        t = line[0].split('..')[-1]
        try:
            times.append(float(t))
        except ValueError:
            pass
    return sorted(set(times))


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
    over = None
    if settled is not None and steady > 0:
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
        'frames': len(seg),
        'violations': int(end['violations']),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('record')
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--settle-max', type=float, default=8.0)
    ap.add_argument('--awb-tol', type=float, default=0.15)
    ap.add_argument('--timeline', help='the scenario: each change it plans '
                    'must start a segment within 1.5 s')
    ap.add_argument('--no-awb', action='store_true',
                    help='do not require white balance gains')
    ap.add_argument('--awb-advisory', action='store_true',
                    help='report white balance findings without failing')
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.record)))
    if len(rows) < 10:
        print('FAIL: %d frames in %s' % (len(rows), a.record))
        return 1
    found = segments(rows)
    segs = []
    for x, y, short in found:
        seg = analyse(rows, x, y, a.tol, a.awb_tol)
        seg['short'] = short
        segs.append(seg)
    if not segs or segs[0]['short']:
        print('FAIL: no steady segment to measure against in %s' % a.record)
        return 1
    ref = segs[0]['steady']
    if ref <= 0:
        print('FAIL: the first segment is dark (G 0)')
        return 1
    fails = 0
    if a.timeline:
        for t in planned(a.timeline):
            if not any(t <= s['t0'] <= t + 1.5 for s in segs):
                print('FAIL: the timeline changes the light at %.1fs, the '
                      'record does not' % t)
                fails += 1
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
        over = '%.0f%%' % (s['over'] * 100) if s['over'] is not None else 'n/a'
        print('%5.1f-%-7.1f %-14s %8.0f %+6.0f%% %6s %6s %8s %6s  %s' %
              (s['t0'], s['t1'], s['light'], s['steady'], level * 100,
               settle, over, s['lines'], s['gain'], awb))
        if i == 0:
            continue            # the first segment is boot and convergence
        why, awb_why = [], []
        if s['short']:
            why.append('only %d frames: too few to measure' %
                       (s['frames']))
        if not s['awb'] and not a.no_awb:
            awb_why.append('no white balance gains recorded')
        if s['settle'] is None or s['settle'] > a.settle_max:
            why.append('did not settle within %.0fs' % a.settle_max)
        if abs(level) > a.tol:
            why.append('level %+.0f%% off the first segment' % (level * 100))
        if s['awb'] and max(abs(x) for x in s['awb']) > a.awb_tol:
            awb_why.append('white balance off by more than %.0f%%' %
                           (a.awb_tol * 100))
        if a.awb_advisory and awb_why:
            print('  note %.1fs: %s' % (s['t0'], '; '.join(awb_why)))
        else:
            why += awb_why
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
