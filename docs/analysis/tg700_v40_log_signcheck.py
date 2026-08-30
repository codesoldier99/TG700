#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verify the full sign chain (pilot stick -> controller -> body rate) in the
only real lift-off log, to separate a *code* sign error from a *hardware*
tilt-installation error.

If gyro/AHRS signs were wrong we would see roll and pitch inverted too.
If only yaw is inverted, the mixer/controller signs are fine and the fault
is mechanical.
"""
import re
import sys

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/2026-08-30 15-32-20.log"

fmt = {}
rate, att, rcin, ctun = [], [], [], []

with open(LOG, "r", encoding="utf-8", errors="replace") as f:
    for ln in f:
        if ln.startswith("FMT,"):
            p = [x.strip() for x in ln.split(",")]
            if len(p) >= 6:
                fmt[p[3]] = p[5:]
            continue
        if ln.startswith("RATE,"):
            rate.append([x.strip() for x in ln.split(",")])
        elif ln.startswith("ATT,"):
            att.append([x.strip() for x in ln.split(",")])
        elif ln.startswith("RCIN,"):
            rcin.append([x.strip() for x in ln.split(",")])

print("log: %s" % LOG)
print("RATE cols: %s" % fmt.get("RATE"))
print("ATT  cols: %s" % fmt.get("ATT"))
print()


def col(name, rec):
    c = fmt.get(rec)
    return c.index(name) + 1 if c and name in c else None


def fnum(row, i):
    try:
        return float(row[i])
    except (ValueError, IndexError, TypeError):
        return None


def corr(xs, ys):
    n = len(xs)
    if n < 10:
        return None, None, n
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
    sxx = sum((a - mx) ** 2 for a in xs)
    syy = sum((b - my) ** 2 for b in ys)
    if sxx <= 0 or syy <= 0:
        return None, None, n
    return sxy / sxx, sxy / (sxx * syy) ** 0.5, n


print("=" * 72)
print("A. controller output  ->  achieved body rate   (per axis)")
print("=" * 72)
print("  slope>0 means the actuator moves the airframe the way the")
print("  controller asked.  slope<0 means that axis is INVERTED.")
print()
for axis, out_c, act_c in (("roll ", "ROut", "R"), ("pitch", "POut", "P"), ("yaw  ", "YOut", "Y")):
    io, ia = col(out_c, "RATE"), col(act_c, "RATE")
    xs, ys = [], []
    for r in rate:
        a, b = fnum(r, io), fnum(r, ia)
        if a is None or b is None:
            continue
        if abs(a) < 0.05:          # ignore near-zero commands
            continue
        xs.append(a)
        ys.append(b)
    s, c, n = corr(xs, ys)
    if s is None:
        print("  %s : insufficient data (n=%d)" % (axis, n))
        continue
    verdict = "OK (same sense)" if s > 0 else "*** INVERTED ***"
    print("  %s : rate = %+8.3f * out %+8.3f      r=%+.3f  n=%-6d %s"
          % (axis, s, 0.0, c, n, verdict))

print()
print("=" * 72)
print("B. desired rate  ->  achieved rate   (closed-loop tracking)")
print("=" * 72)
for axis, des_c, act_c in (("roll ", "RDes", "R"), ("pitch", "PDes", "P"), ("yaw  ", "YDes", "Y")):
    idd, ia = col(des_c, "RATE"), col(act_c, "RATE")
    xs, ys = [], []
    for r in rate:
        a, b = fnum(r, idd), fnum(r, ia)
        if a is None or b is None:
            continue
        if abs(a) < 2.0:
            continue
        xs.append(a)
        ys.append(b)
    s, c, n = corr(xs, ys)
    if s is None:
        print("  %s : insufficient data (n=%d)" % (axis, n))
        continue
    print("  %s : achieved = %+7.4f * desired   r=%+.3f  n=%-6d  tracking %5.1f%%"
          % (axis, s, c, n, s * 100))

print()
print("=" * 72)
print("C. AHRS yaw drift  (is the recorded heading consistent with CCW spin?)")
print("=" * 72)
iy = col("Yaw", "ATT")
ys = [fnum(a, iy) for a in att]
ys = [v for v in ys if v is not None]
if ys:
    unw, off = [ys[0]], 0.0
    for k in range(1, len(ys)):
        d = ys[k] - ys[k - 1]
        if d > 180:
            off -= 360
        elif d < -180:
            off += 360
        unw.append(ys[k] + off)
    print("  heading start %.1f deg   end %.1f deg   net change %+.1f deg"
          % (unw[0], unw[-1], unw[-1] - unw[0]))
    print("  -> %s rotation recorded by the AHRS"
          % ("CLOCKWISE (nose right)" if unw[-1] - unw[0] > 0 else "COUNTER-CLOCKWISE (nose left)"))

print()
print("=" * 72)
print("D. yaw saturation")
print("=" * 72)
io = col("YOut", "RATE")
vals = [abs(v) for v in (fnum(r, io) for r in rate) if v is not None]
if vals:
    sat = sum(1 for v in vals if v > 0.99)
    print("  |YOut| >= 0.99 for %d / %d samples  (%.1f%% of the time)"
          % (sat, len(vals), 100.0 * sat / len(vals)))
    print("  max |YOut| = %.3f" % max(vals))
