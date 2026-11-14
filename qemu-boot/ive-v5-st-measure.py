#!/usr/bin/env python3
"""What V5 IVE op 25 (st_box_flt_and_eigen) computes, measured against the
CV608 reference (#300).

The op is the corner measure in the middle of ss_mpi_ive_st_cand_corner and
is the one V5 IVE op hisi-ive-npu does not model.  ive-v5-capture.c's
cap_st_recip() captures it on designed inputs; this reads that capture back
out of qemu/tests/data/ive-v5-cv608-ref.tar.xz and reports what the measure
is, so a candidate rule can be scored without a board.

    python3 qemu-boot/ive-v5-st-measure.py [REF.tar.xz | DIR]

The probe's four pictures are separable, so each one's tensors are the outer
product of a column triple and a row triple: B comes out zero and the
determinant is an exact product, and each picture's diagonal holds 20 to 35
determinants at a single trace.  That is what makes the two checks below
possible.
"""
import os, re, sys, tarfile
import numpy as np

W = H = 192
DEFAULT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'qemu', 'tests', 'data', 'ive-v5-cv608-ref.tar.xz')


def tensors(src):
    """A, B, C: the 3x3 box sums of the S8 norm_grad products"""
    g = np.array([0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, -2, 0, 2, 0,
                  0, -1, 0, 1, 0, 0, 0, 0, 0, 0], np.int64).reshape(5, 5)
    p = np.pad(src, 2, mode='edge')

    def corr(k):
        o = np.zeros(src.shape, np.int64)
        for j in range(5):
            for i in range(5):
                o += k[j, i] * p[j:j + src.shape[0], i:i + src.shape[1]]
        return o

    gx = np.clip((corr(g) + 4) >> 3, -128, 127)
    gy = np.clip((corr(g.T) + 4) >> 3, -128, 127)

    def box(a):
        q = np.pad(a, 1, mode='edge')
        o = np.zeros(a.shape, np.int64)
        for j in range(3):
            for i in range(3):
                o += q[j:j + a.shape[0], i:i + a.shape[1]]
        return o

    return box(gx * gx), box(gx * gy), box(gy * gy)


def load(path):
    """[(trace, src, measure)] from the tarball or an unpacked directory"""
    blob = {}
    if os.path.isdir(path):
        for n in os.listdir(path):
            if n.startswith('strec_tr'):
                blob[n] = open(os.path.join(path, n), 'rb').read()
    else:
        with tarfile.open(path) as t:
            for m in t.getmembers():
                n = os.path.basename(m.name)
                if n.startswith('strec_tr'):
                    blob[n] = t.extractfile(m).read()
    out = []
    for n in sorted(blob):
        m = re.fullmatch(r'strec_tr(\d+)_eig\.bin', n)
        if not m:
            continue
        tr = int(m.group(1))
        src = np.frombuffer(blob['strec_tr%d_src.bin' % tr], np.uint8)
        out.append((tr, src[:W * H].reshape(H, W).astype(np.int64),
                    np.frombuffer(blob[n], '<u2')[:W * H]
                      .reshape(H, W).astype(np.int64)))
    return out


def rule(tr, det):
    """the best rule so far: 84.6% exact, 95.4% within one count"""
    t1 = det // (16 * tr)
    return t1 + (16 * t1 * t1) // tr


def main(path):
    pics = load(path)
    if not pics:
        print('no strec_* capture in %s' % path)
        return 1
    rows = set()
    for tr0, src, meas in pics:
        a, b, c = tensors(src)
        sl = (slice(3, H - 3), slice(3, W - 3))
        for x, y, z, l in zip(a[sl].ravel(), b[sl].ravel(),
                              c[sl].ravel(), meas[sl].ravel()):
            if x + z > 0 and y == 0:
                rows.add((int(x + z), int(x * z), int(l)))
    rows = sorted(rows)
    print('%d pictures, %d distinct B == 0 tensors, measure up to %d'
          % (len(pics), len(rows), max(r[2] for r in rows)))

    err = np.array([l - rule(t, d) for t, d, l in rows])
    print('\nt1 = det // (16 * trace);  measure = t1 + (16 * t1^2) // trace')
    print('  exact %5.1f%%   within one %5.1f%%   errors %d..%d'
          % (100 * (err == 0).mean(), 100 * (np.abs(err) <= 1).mean(),
             err.min(), err.max()))

    # the reciprocal model this capture was built to test, and refutes
    by = {}
    for t, d, l in rows:
        by.setdefault(t, []).append((d, l))
    feas = bad = 0
    for t, s in by.items():
        s = [(d, l) for d, l in s if l >= 20]
        if len(s) < 4:
            continue
        lo, hi = 0.125 / t, 8.0 / t
        for d, l in s:
            for target, keep in ((l, 'lo'), (l + 1, 'hi')):
                p, q = 0.125 / t, 8.0 / t
                for _ in range(200):
                    mid = 0.5 * (p + q)
                    e = d * mid
                    if e * (1.0 + e * mid) / 16.0 < target:
                        p = mid
                    else:
                        q = mid
                if keep == 'lo':
                    lo = max(lo, 0.5 * (p + q))
                else:
                    hi = min(hi, 0.5 * (p + q))
        if lo >= hi:
            bad += 1
        else:
            feas += 1
    print('\nmeasure = floor(det * R * (1 + det * R^2) / 16), one R per trace:')
    print('  %d traces carry four determinants or more; %d admit such an R, '
          '%d do not' % (feas + bad, feas, bad))
    print('  -- so the engine is not doing that, however R is rounded.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT))
