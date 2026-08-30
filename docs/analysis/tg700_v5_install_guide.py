# -*- coding: utf-8 -*-
"""
Generate the v5.0 tilt-installation table and cross-check it against the yaw
factors actually compiled into AP_MotorsMatrix_TG700.cpp.

Body frame: X forward, Y right, Z down.
Positive yaw = nose right = clockwise seen from above = rotation about +Z.

Tangential unit vector that produces POSITIVE yaw at position (x, y):
    t_cw  = (-y, +x) / |r|          (derived from v = omega x r with omega = +Z)
and for NEGATIVE yaw:
    t_ccw = (+y, -x) / |r|

Install rule: the tilt force must ADD to the propeller reaction torque.
A CW propeller (seen from above) pushes the airframe CCW (negative yaw), so a CW
motor must be leaned along t_ccw.  A CCW propeller must be leaned along t_cw.
Equivalently: seen from above, the motor top leans OPPOSITE to the way its own
propeller spins.
"""
import math
import os
import re

X_ARM, Y_OUTER, Y_INNER = 1.665, 3.4, 1.6
TILT_TANG_DEG = 5.0

# pair -> (x, y, rotation in v5.0, motor numbers, servo channels, CAN)
PAIRS = [
    ("P1", "front-right-outer", +X_ARM, +Y_OUTER, "CW",  (1, 2),   ("SERVO5", "SERVO6"),   "CAN1 ESC0,1"),
    ("P2", "front-right-inner", +X_ARM, +Y_INNER, "CCW", (3, 4),   ("SERVO7", "SERVO8"),   "CAN1 ESC2,3"),
    ("P3", "front-left-inner",  +X_ARM, -Y_INNER, "CW",  (5, 6),   ("SERVO13", "SERVO14"), "CAN2 ESC0,1"),
    ("P4", "front-left-outer",  +X_ARM, -Y_OUTER, "CCW", (7, 8),   ("SERVO15", "SERVO16"), "CAN2 ESC2,3"),
    ("P5", "rear-right-outer",  -X_ARM, +Y_OUTER, "CCW", (9, 10),  ("SERVO9", "SERVO10"),  "CAN1 ESC4,5"),
    ("P6", "rear-right-inner",  -X_ARM, +Y_INNER, "CW",  (11, 12), ("SERVO11", "SERVO12"), "CAN1 ESC6,7"),
    ("P7", "rear-left-inner",   -X_ARM, -Y_INNER, "CCW", (13, 14), ("SERVO17", "SERVO18"), "CAN2 ESC4,5"),
    ("P8", "rear-left-outer",   -X_ARM, -Y_OUTER, "CW",  (15, 16), ("SERVO19", "SERVO20"), "CAN2 ESC6,7"),
]

# v4.0 rotation: upper = CW, lower = CCW for every pair
V40 = {"upper": "CW", "lower": "CCW"}


def unit_tangential(x, y, sense):
    r = math.hypot(x, y)
    if sense == "+yaw":                 # clockwise from above
        return (-y / r, x / r)
    return (y / r, -x / r)              # counter-clockwise from above


def describe(vx, vy, y_pos):
    """Turn a tilt direction vector into words a technician can act on."""
    fore = "\u524d" if vx > 0 else "\u540e"                       # front / rear
    # outboard means away from the centreline, i.e. same sign as the position's y
    outb = "\u5916" if (vy > 0) == (y_pos > 0) else "\u5185"      # outboard / inboard
    return fore, abs(vx), outb, abs(vy)


print("=" * 96)
print("TG700 v5.0 tilt installation table   (tangential tilt = %.1f deg)" % TILT_TANG_DEG)
print("=" * 96)
print("Rule: seen from above, the motor top leans OPPOSITE to its own propeller's spin.")
print("      Both motors of a coaxial pair lean the SAME way (this is the v5.0 change).")
print()

sin_t = math.sin(math.radians(TILT_TANG_DEG))
hdr = "%-4s %-19s %-4s %-9s %-34s %s"
print(hdr % ("pair", "position", "rot", "yaw sign", "tilt direction (unit vector)", "leans toward"))
print("-" * 96)

rows = []
for pid, name, x, y, rot, mots, servos, can in PAIRS:
    # CW prop -> reaction gives -yaw -> tilt must also give -yaw
    sense = "-yaw" if rot == "CW" else "+yaw"
    vx, vy = unit_tangential(x, y, sense)
    fore, mfore, outb, moutb = describe(vx, vy, y)
    words = "%s%s %.0f%%  +  %s%s %.0f%%" % (
        fore, "", mfore * 100, outb, "", moutb * 100)
    print(hdr % (pid, name, rot, sense, "(%+.3f X, %+.3f Y)" % (vx, vy), words))
    rows.append(dict(pid=pid, name=name, rot=rot, sense=sense, vx=vx, vy=vy,
                     mots=mots, servos=servos, can=can, x=x, y=y))

# ---------------- cross-check against the compiled yaw factors ----------------
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "libraries", "AP_Motors", "AP_MotorsMatrix_TG700.cpp")
src = open(SRC, encoding="utf-8").read()
defines = {n: float(v) for n, v in
           re.findall(r"#define\s+(TG700_\w+)\s+\(?\s*(-?[\d.]+)f?\s*\)?", src)}
yawfac = {}
for m in re.finditer(r"add_motor_raw\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*TG700_\w+\s*,"
                     r"\s*TG700_\w+\s*,\s*(TG700_YAW_\w+)\s*,", src):
    yawfac[int(m.group(1))] = defines[m.group(2)]

print("\n" + "=" * 96)
print("cross-check: does the physical tilt sign match the compiled yaw factor?")
print("=" * 96)
bad = 0
for r in rows:
    # yaw moment per unit thrust from the tilt alone
    L = math.hypot(r["x"], r["y"])
    # signed yaw moment: tangential force (sin_t) dotted with the +yaw tangential dir
    tcx, tcy = unit_tangential(r["x"], r["y"], "+yaw")
    m_yaw = sin_t * (r["vx"] * tcx + r["vy"] * tcy) * L
    for mot in r["mots"]:
        code = yawfac[mot]
        ok = (m_yaw > 0) == (code > 0)
        if not ok:
            bad += 1
        print("  %s mot%-3d physical yaw=%+.4f N.m/N   code yaw_fac=%+.4f   %s"
              % (r["pid"], mot, m_yaw, code, "OK" if ok else "SIGN MISMATCH"))
print("\n  %s" % ("all signs consistent" if bad == 0 else "%d MISMATCHES" % bad))

# ---------------- hardware change list ----------------
print("\n" + "=" * 96)
print("hardware change list: which motors must reverse (v4.0 upper=CW / lower=CCW)")
print("=" * 96)
changes = []
for r in rows:
    for pos, mot, servo in zip(("upper", "lower"), r["mots"], r["servos"]):
        old = V40[pos]
        new = r["rot"]
        if old != new:
            changes.append((r["pid"], pos, mot, servo, r["can"], old, new))
print("  %-4s %-6s %-6s %-8s %-14s %s" % ("pair", "pos", "motor", "servo", "can/esc", "rotation"))
for pid, pos, mot, servo, can, old, new in changes:
    print("  %-4s %-6s MOT_%-2d %-8s %-14s %s -> %s" % (pid, pos, mot, servo, can, old, new))
print("\n  total motors to reverse: %d  (each needs opposite-hand prop + ESC direction flip)" % len(changes))

# prop exchange pairing: an upper losing CW can hand it to a lower needing CW
need_cw = [c for c in changes if c[6] == "CW"]
need_ccw = [c for c in changes if c[6] == "CCW"]
print("\n  propeller exchange (counts stay 8 CW / 8 CCW, so no new props needed):")
for a, b in zip(need_cw, need_ccw):
    print("    swap %s %s MOT_%-2d (%s)  <-->  %s %s MOT_%-2d (%s)"
          % (a[0], a[1], a[2], a[4].split()[0], b[0], b[1], b[2], b[4].split()[0]))
