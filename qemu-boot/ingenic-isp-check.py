#!/usr/bin/env python3
"""Check NV12 frames that the ingenic-isp model painted (test-ingenic-isp.sh).

The model draws hw/misc/sstar-pattern.h's scene -- the OpenIPC logo on a
navy gradient, a white dot sliding beneath it -- and a six-digit frame
counter in 3x5 cells at the top left (cell = height / 54).  For each file
this reads the counter back, checks that the logo box is mostly bright
where the background is navy (at whatever exposure the kernel's AE
chose), and finds the dot; with --advance the
counters must increase from file to file, the dot where they put it.

    ingenic-isp-check.py [--png DIR] [--advance] WxH:file.nv12 ...
"""
import argparse
import os
import sys

import numpy as np

DIGITS = [
    (7, 5, 5, 5, 7), (2, 6, 2, 2, 7), (7, 1, 7, 4, 7), (7, 1, 7, 1, 7),
    (5, 5, 7, 1, 1), (7, 4, 7, 1, 7), (7, 4, 7, 5, 7), (7, 1, 1, 1, 1),
    (7, 5, 7, 5, 7), (7, 5, 7, 1, 7),
]
PAT_W, PAT_H = 1920, 1080
LOGO_W, LOGO_H = 1200, 331
LOGO_X = (PAT_W - LOGO_W) // 2
LOGO_Y = PAT_H * 2 // 5 - LOGO_H // 2
DOT_Y = PAT_H * 4 // 5
DOT_R = 64


def load(spec):
    size, path = spec.split(':', 1)
    w, h = (int(v) for v in size.split('x'))
    raw = np.fromfile(path, dtype=np.uint8)
    if raw.size < w * h * 3 // 2:
        raise SystemExit('%s: %d bytes, %dx%d NV12 needs %d' %
                         (path, raw.size, w, h, w * h * 3 // 2))
    y = raw[:w * h].reshape(h, w)
    uv = raw[w * h:w * h * 3 // 2].reshape(h // 2, w)
    return path, w, h, y, uv


def counter(y, w, h):
    cell = max(h // 54, 1)
    value = 0
    for d in range(6):
        rows = []
        for r in range(5):
            bits = 0
            for c in range(3):
                px = 2 * cell + (d * 4 + c) * cell + cell // 2
                py = 2 * cell + r * cell + cell // 2
                if y[py, px] > 150:
                    bits |= 4 >> c
            rows.append(bits)
        if tuple(rows) not in DIGITS:
            return None
        value = value * 10 + DIGITS.index(tuple(rows))
    return value


def scaled(v, full, size):
    return v * size // full


def background(y, w, h):
    # the band between the logo and the dot's row
    by0 = scaled(LOGO_Y + LOGO_H + 20, PAT_H, h)
    by1 = scaled(DOT_Y - 80, PAT_H, h)
    bg = y[by0:by1, :]
    return float(bg.mean()), float(bg.std())


def white(bgm):
    # halfway from the navy to white: the exposure the AE chose moves both
    return max(128, (bgm + 235) / 2)


def logo_ok(y, w, h, bgm):
    x0, x1 = scaled(LOGO_X, PAT_W, w), scaled(LOGO_X + LOGO_W, PAT_W, w)
    y0, y1 = scaled(LOGO_Y, PAT_H, h), scaled(LOGO_Y + LOGO_H, PAT_H, h)
    box = y[y0:y1, x0:x1]
    return (box > white(bgm)).mean()


def dot_x(y, w, h, bgm):
    row = y[scaled(DOT_Y, PAT_H, h), :]
    lit = np.nonzero(row > white(bgm))[0]
    return float(lit.mean()) if lit.size else None


def dot_at(frame, w):
    r = LOGO_W - 2 * DOT_R
    t = frame * 8 % (2 * r)
    return (LOGO_X + DOT_R + (t if t < r else 2 * r - t)) * w / PAT_W


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--png', help='write each frame as a PNG here')
    ap.add_argument('--advance', action='store_true',
                    help='counters and the dot must move file to file')
    ap.add_argument('frames', nargs='+',
                    help='WxH:file.nv12[@N]: with @N the counter must be in '
                    '[N, N + 500000) -- T40\'s second pipe paints its '
                    'counter 500000 ahead')
    args = ap.parse_args()

    ok = True
    prev = None
    for spec in args.frames:
        base = None
        if '@' in spec:
            spec, base = spec.rsplit('@', 1)
            base = int(base)
        path, w, h, y, uv = load(spec)
        n = counter(y, w, h)
        if base is not None and n is not None and \
                not base <= n < base + 500000:
            print('%s: counter %d, not from %d: the wrong pipe' %
                  (path, n, base))
            ok = False
        bgm, bgs = background(y, w, h)
        bright = logo_ok(y, w, h, bgm)
        dx = dot_x(y, w, h, bgm)
        # the kernel's AE picks the exposure: a navy anywhere from dim to
        # bright, its blue (Cb above 128) growing with it
        good = (n is not None and bright > 0.15 and 20 < bgm < 170 and
                bgs < 8 and dx is not None and
                132 < float(uv[::2, 0::2].mean()) < 220)
        print('%s: %dx%d counter %s logo %.0f%% bright, background Y %.1f '
              '(sd %.1f), dot at x %s, Cb %.0f Cr %.0f: %s' %
              (path, w, h, n, bright * 100, bgm, bgs,
               'none' if dx is None else '%.0f' % dx,
               uv[:, 0::2].mean(), uv[:, 1::2].mean(),
               'ok' if good else 'BAD'))
        ok &= good
        if args.advance and prev is not None and prev[0] == (w, h):
            # the dot slides 8 pixels a frame across the logo and back:
            # where the counter puts it (two frames symmetric about a turn
            # have it in the same place)
            moved = n is not None and prev[1] is not None and n > prev[1] \
                and dx is not None and \
                abs(dx - dot_at(n, w)) <= 4 + w // 100
            print('  advanced from %s: %s' % (prev[1], 'yes' if moved else
                                              'NO'))
            ok &= moved
        prev = ((w, h), n, dx)
        if args.png:
            from PIL import Image
            os.makedirs(args.png, exist_ok=True)
            cb = np.repeat(np.repeat(uv[:, 0::2], 2, 0), 2, 1)[:h, :w]
            cr = np.repeat(np.repeat(uv[:, 1::2], 2, 0), 2, 1)[:h, :w]
            yf = (y.astype(np.float32) - 16) * 255 / 219
            cbf = cb.astype(np.float32) - 128
            crf = cr.astype(np.float32) - 128
            rgb = np.stack([yf + 1.596 * crf * 255 / 224 * 0.875,
                            yf - 0.392 * cbf - 0.813 * crf,
                            yf + 2.017 * cbf], -1)
            Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(
                os.path.join(args.png,
                             os.path.basename(path).rsplit('.', 1)[0] +
                             '.png'))
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
