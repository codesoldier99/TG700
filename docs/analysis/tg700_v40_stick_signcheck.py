#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Clean sign test: pilot stick (an EXOGENOUS input) vs achieved body rate.

Correlating controller output against body rate is confounded by feedback
(out ~ -P*rate whenever the target is near zero), which makes every axis
look "inverted".  The pilot stick is not a function of aircraft state, so
regressing rate on stick deflection gives the true actuator sign.
"""
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/2026-08-30 15-32-20.log"

fmt = {}
rc, rate = [], []
with open(LOG, "r", encoding="utf-8", errors="replace") as f:
    for ln in f:
        if ln.startswith("FMT,"):
            p = [x.strip() for x in ln.split(",")]
            if len(p) >= 6:
                fmt[p[3]] = p[5:]
        elif ln.startswith("RCIN,"):
            rc.append([x.strip() for x in ln.split(",")])
        elif ln.startswith("RATE,"):
            rate.append([x.strip() for x in ln.split(",")])


def col(name, rec):
    c = fmt.get(rec)
    return c.index(name) + 1 if c and name in c else None


def fnum(row, i):
    try:
        return float(row[i])
    except (ValueError, IndexError, TypeError):
        return None


it_rc, it_rt = col("TimeUS", "RCIN"), col("TimeUS", "RATE")

# build a time-sorted series of stick values, then nearest-neighbour join
sticks = {}
for ch, nm in (("C1", "roll"), ("C2", "pitch"), ("C4", "yaw")):
    ic = col(ch, "RCIN")
    if ic is None:
        continue
    s = []
    for r in rc:
        t, v = fnum(r, it_rc), fnum(r, ic)
        if t is not None and v is not None:
            s.append((t, v))
    s.sort()
    sticks[nm] = s

TRIM = {"roll": 1500.0, "pitch": 1500.0, "yaw": 1515.0}   # RC4_TRIM = 1515
RATECOL = {"roll": "R", "pitch": "P", "yaw": "Y"}

rt = []
for r in rate:
    t = fnum(r, it_rt)
    if t is not None:
        rt.append((t, r))
rt.sort()

print("log: %s" % LOG)
print()
print("=" * 74)
print("pilot stick deflection  ->  achieved body rate")
print("=" * 74)
print("  a POSITIVE slope means the aircraft rotates the way the pilot asked")
print()

import bisect
for nm in ("roll", "pitch", "yaw"):
    s = sticks.get(nm)
    if not s:
        print("  %-5s : no RC data" % nm)
        continue
    ts = [x[0] for x in s]
    ic = col(RATECOL[nm], "RATE")
    xs, ys = [], []
    for t, r in rt:
        v = fnum(r, ic)
        if v is None:
            continue
        k = bisect.bisect_left(ts, t)
        if k >= len(ts):
            k = len(ts) - 1
        pwm = s[k][1]
        d = pwm - TRIM[nm]
        if abs(d) < 60:              # outside the deadzone only
            continue
        xs.append(d)
        ys.append(v)
    n = len(xs)
    if n < 30:
        print("  %-5s : insufficient deflection samples (n=%d)" % (nm, n))
        continue
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
    sxx = sum((a - mx) ** 2 for a in xs)
    syy = sum((b - my) ** 2 for b in ys)
    slope = sxy / sxx
    r_ = sxy / (sxx * syy) ** 0.5
    # split-sample means: does full-left vs full-right give opposite rates?
    left = [b for a, b in zip(xs, ys) if a < -150]
    right = [b for a, b in zip(xs, ys) if a > 150]
    ml = sum(left) / len(left) if left else float("nan")
    mr = sum(right) / len(right) if right else float("nan")
    print("  %-5s : slope = %+8.5f deg/s per us   r=%+.3f  n=%d   %s"
          % (nm, slope, r_, n, "OK" if slope > 0 else "*** INVERTED ***"))
    print("          stick LEFT/DOWN (<-150us) mean rate = %+7.2f deg/s (n=%d)" % (ml, len(left)))
    print("          stick RIGHT/UP  (>+150us) mean rate = %+7.2f deg/s (n=%d)" % (mr, len(right)))
