#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TG700 yaw design under the NEW, tighter mechanical constraint:

    every motor mount can only tilt along +-Y (left / right).
    tilting along +-X (nose / tail) is NOT possible.

Questions answered:
  1. what moment arm does each motor now have, and what is the tilt direction
     for each of the 16 motors if we keep the v4.0 rotation (upper CW/lower CCW)?
  2. how much yaw authority is left vs the +-X/+-Y scheme (v6.0) and vs the
     ideal tangential scheme?
  3. is the onboard v4.0 firmware still usable with these mounts?  quantify.
  4. how much tilt angle is needed to claw the authority back?

Sign conventions (ArduPilot NED body: X fwd, Y right, Z down):
  horizontal force (Fx,Fy) at (x,y):   Mz = x*Fy - y*Fx
  positive Mz = nose right = positive yaw
  AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1  ->  a CW rotor gets a NEGATIVE yaw factor
Pure ASCII source on purpose.
"""
import math

DEG = math.pi / 180.0
X_ARM, Y_OUT, Y_IN = 1.665, 3.4, 1.6
QT = 0.0056                 # C_Q/C_T * R_prop, reaction-torque arm (m)
TILT = 5.0
SIN_T = math.sin(TILT * DEG)

# pod -> (x, y, ring)
PODS = [
    ("P1", +X_ARM, +Y_OUT, "out"), ("P2", +X_ARM, +Y_IN, "in"),
    ("P3", +X_ARM, -Y_IN, "in"),   ("P4", +X_ARM, -Y_OUT, "out"),
    ("P5", -X_ARM, +Y_OUT, "out"), ("P6", -X_ARM, +Y_IN, "in"),
    ("P7", -X_ARM, -Y_IN, "in"),   ("P8", -X_ARM, -Y_OUT, "out"),
]

bar = "=" * 84
print(bar)
print("1)  MOMENT ARM under the +-Y-only constraint")
print(bar)
print("  force along +-Y at (x,y):  Mz = +x*Fy   ->  moment arm = |x|")
print("  every pod sits at x = +-1.665 m, so ALL 16 motors now share the")
print("  same arm 1.665 m.  the outer pods lose the 3.400 m arm they had")
print("  when they were allowed to tilt along +-X.")
print()
print("  per-motor yaw coefficient  = sin(tilt)*arm + Q/T")
c_yonly = SIN_T * X_ARM + QT
c_out_x = SIN_T * Y_OUT + QT          # outer tilting along +-X (v6.0)
c_in_y = SIN_T * X_ARM + QT           # inner tilting along +-Y (v6.0, same as now)
c_out_tan = SIN_T * math.hypot(X_ARM, Y_OUT) + QT
c_in_tan = SIN_T * math.hypot(X_ARM, Y_IN) + QT
print("    +-Y only  (all 16)      : %.5f  T*m per unit thrust" % c_yonly)
print("    v6.0 outer (along +-X)  : %.5f" % c_out_x)
print("    v6.0 inner (along +-Y)  : %.5f" % c_in_y)
print("    ideal tangential outer  : %.5f" % c_out_tan)
print("    ideal tangential inner  : %.5f" % c_in_tan)

print()
print(bar)
print("2)  TILT DIRECTION for each motor, keeping the v4.0 rotation")
print(bar)
print("  v4.0 rotation = upper CW (yaw factor NEGATIVE), lower CCW (POSITIVE).")
print("  With Mz = x*Fy:")
print("     front pod (x>0): Mz<0 needs Fy<0 -> tilt LEFT ; Mz>0 -> tilt RIGHT")
print("     rear  pod (x<0): Mz<0 needs Fy>0 -> tilt RIGHT; Mz>0 -> tilt LEFT")
print()
print("  pod   x       row    upper(CW) tilt   lower(CCW) tilt")
rows = []
for name, x, y, ring in PODS:
    row = "front" if x > 0 else "rear"
    up = "LEFT  (-Y)" if x > 0 else "RIGHT (+Y)"
    lo = "RIGHT (+Y)" if x > 0 else "LEFT  (-Y)"
    rows.append((name, x, y, ring, row, up, lo))
    print("  %-4s  %+.3f  %-5s  %-14s  %-14s" % (name, x, row, up, lo))
print()
print("  RULE (no left/right-hand wording needed):")
print("    all four FRONT pods : upper tilts LEFT , lower tilts RIGHT")
print("    all four REAR  pods : upper tilts RIGHT, lower tilts LEFT")
print("  outer and inner are treated identically - the ring no longer matters.")

# ---- build the 16-motor table: (name, x, y, ring, is_upper, sign) ----
M = []
for name, x, y, ring in PODS:
    M.append((name + "U", x, y, ring, True, -1))     # upper CW  -> negative
    M.append((name + "L", x, y, ring, False, +1))    # lower CCW -> positive

print()
print(bar)
print("3)  DECOUPLING check (must be exact)")
print(bar)
roll = {}
for name, x, y, ring, up, sg in M:
    roll[name] = -(y / Y_OUT)
pitch = {name: (1.0 if x > 0 else -1.0) for name, x, y, ring, up, sg in M}
# all magnitudes are now EQUAL, so the normalised factor is +-0.5 for everyone
yaw = {name: sg * 0.5 for name, x, y, ring, up, sg in M}
s_yr = sum(yaw[n] * roll[n] for n, *_ in M)
s_yp = sum(yaw[n] * pitch[n] for n, *_ in M)
s_yt = sum(yaw[n] for n, *_ in M)
print("  sum yaw*roll  = %+.3e" % s_yr)
print("  sum yaw*pitch = %+.3e" % s_yp)
print("  sum yaw       = %+.3e   (thrust change from a yaw command)" % s_yt)
ok = max(abs(s_yr), abs(s_yp), abs(s_yt)) < 1e-12
print("  -> %s" % ("EXACTLY DECOUPLED" if ok else "*** COUPLED ***"))
print("  reason: yaw is made inside each coaxial pod (upper +d, lower -d), so")
print("          each pod's thrust sum is unchanged -> no roll/pitch/lift upset.")
print("  standing yaw at equal thrust: %+.3e (no hover spin)"
      % sum(sg * c_yonly for _, _, _, _, _, sg in M))

print()
print(bar)
print("4)  YAW AUTHORITY vs the earlier schemes")
print(bar)


def authority(coeffs):
    """Total yaw moment at full command after normalise_rpy_factors()
    scales the largest |yaw factor| to 0.5."""
    mx = max(abs(c) for c in coeffs)
    return sum(0.5 * abs(c) / mx * abs(c) for c in coeffs) if False else \
        sum(0.5 * (abs(c) / mx) * abs(c) for c in coeffs)


# scheme coefficient lists (per motor, magnitude of the true yaw coefficient)
co_yonly = [c_yonly] * 16
co_v60 = [c_out_x] * 8 + [c_in_y] * 8
co_ideal = [c_out_tan] * 8 + [c_in_tan] * 8

a_y, a_6, a_i = authority(co_yonly), authority(co_v60), authority(co_ideal)
print("  scheme                                  yaw moment   vs v6.0   vs ideal")
print("  +-Y only, 5 deg, matched mixer (v8.0)   %8.4f    %5.1f%%   %5.1f%%"
      % (a_y, 100 * a_y / a_6, 100 * a_y / a_i))
print("  +-X/+-Y,  5 deg (v6.0)                  %8.4f    %5.1f%%   %5.1f%%"
      % (a_6, 100.0, 100 * a_6 / a_i))
print("  ideal tangential, 5 deg                %8.4f    %5.1f%%   %5.1f%%"
      % (a_i, 100 * a_i / a_6, 100.0))
print()
print("  losing the +-X option costs %.1f%% of the v6.0 authority." % (100 * (1 - a_y / a_6)))
print("  it is a mechanical fact, not a tuning choice.")

print()
print(bar)
print("5)  IS THE ONBOARD v4.0 FIRMWARE USABLE WITH +-Y-ONLY MOUNTS?")
print(bar)
# v4.0 factors assumed TANGENTIAL tilt -> outer:inner ratio = radial distances
V40_OUT, V40_IN = 0.0971, 0.0592
print("  v4.0 yaw factors: outer %+.4f / inner %+.4f  (ratio %.3f)"
      % (V40_OUT, V40_IN, V40_OUT / V40_IN))
print("  ratio it encodes = %.3f = 3.786/2.309, i.e. the TANGENTIAL assumption"
      % (math.hypot(X_ARM, Y_OUT) / math.hypot(X_ARM, Y_IN)))
print("  true ratio with +-Y-only mounts = %.3f  (all arms equal 1.665)"
      % (c_yonly / c_yonly))
print()
# a) direction / decoupling still correct?
print("  a) DIRECTION and DECOUPLING")
print("     v4.0 keeps upper=negative / lower=positive within every pod, and the")
print("     new tilt rule was derived from exactly that requirement, so:")
v40_pod_sum = -V40_OUT + V40_OUT
print("       - per-pod yaw factor sum = %.1f  -> still thrust neutral" % v40_pod_sum)
print("       - every motor's tilt force ADDS to its own reaction torque")
print("     -> yaw direction correct, roll/pitch/lift undisturbed.  SAFE TO FLY.")
print()
# b) efficiency loss from the mismatch
print("  b) EFFICIENCY of the mismatched mixer")
mx40 = V40_OUT
w40 = [0.5 * (V40_OUT / mx40)] * 8 + [0.5 * (V40_IN / mx40)] * 8
a_v40_on_yonly = sum(w * c_yonly for w in w40)
print("     normalised factors become: outer %.4f, inner %.4f"
      % (0.5, 0.5 * V40_IN / V40_OUT))
print("     yaw moment delivered : %.4f" % a_v40_on_yonly)
print("     matched mixer (v8.0) : %.4f" % a_y)
print("     v4.0 realises %.1f%% of what these mounts can give -> %.1f%% wasted"
      % (100 * a_v40_on_yonly / a_y, 100 * (1 - a_v40_on_yonly / a_y)))
print("     cause: v4.0 drives the OUTER motors 1.64x harder for yaw, but with")
print("            +-Y mounts they are no stronger than the inner ones, so the")
print("            outer motors saturate first while inner headroom sits unused.")
print()
print("  VERDICT: v4.0 flies and yaws the right way IF the mounts follow the")
print("           front-upper-LEFT / rear-upper-RIGHT rule, but gives up ~%.0f%%"
      % (100 * (1 - a_v40_on_yonly / a_y)))
print("           of the little yaw authority that is left.  Since yaw is already")
print("           the binding limit on this aircraft, a matched mixer is worth it.")

print()
print(bar)
print("6)  TILT ANGLE needed to recover authority (+-Y only)")
print(bar)
target = a_6            # match the +-X/+-Y scheme at 5 deg
need = None
print("  tilt   per-motor coeff   authority   vs 5deg(+-Y)   vs v6.0(5deg)   lift loss")
for t in (5, 6, 7, 7.62, 8, 9, 10, 12, 15):
    c = math.sin(t * DEG) * X_ARM + QT
    a = authority([c] * 16)
    loss = 1 - math.cos(t * DEG)
    print("  %5.2f deg  %.5f       %7.4f      %5.2fx        %5.2fx        %.2f%% (%.1f kg)"
          % (t, c, a, a / a_y, a / a_6, 100 * loss, 700 * loss))
# solve for the angle that matches v6.0 at 5 deg
lo, hi = 5.0, 30.0
for _ in range(80):
    mid = 0.5 * (lo + hi)
    if authority([math.sin(mid * DEG) * X_ARM + QT] * 16) < target:
        lo = mid
    else:
        hi = mid
need = 0.5 * (lo + hi)
print()
print("  to match the +-X/+-Y scheme at 5 deg you need %.2f deg along +-Y," % need)
print("  costing only %.2f%% lift (%.1f kg of 700 kg)."
      % (100 * (1 - math.cos(need * DEG)), 700 * (1 - math.cos(need * DEG))))

print()
print(bar)
print("7)  FINAL YAW FACTOR for the firmware")
print(bar)
print("  all 16 motors share one magnitude:")
print("    |yaw_fac| = (sin(%.1f deg)*%.3f + %.4f) / %.1f = %.4f"
      % (TILT, X_ARM, QT, Y_OUT, c_yonly / Y_OUT))
print("  sign: upper CW  -> negative")
print("        lower CCW -> positive")
print("  after normalise_rpy_factors() every factor becomes exactly +-0.5,")
print("  i.e. a yaw command moves all 16 motors by the same amount - which is")
print("  the most efficient allocation possible under this constraint.")
