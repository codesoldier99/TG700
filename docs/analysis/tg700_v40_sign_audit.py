#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TG700 v4.0 (onboard build 3863214b) mixer sign / authority audit.

ArduPilot conventions established from source:
  add_motor(): roll_fac = cos(angle+90) = -sin(angle);  pitch_fac = cos(angle)
               angle measured clockwise from nose.
    -> right side  roll_fac < 0
    -> front       pitch_fac > 0
  AP_MOTORS_MATRIX_YAW_FACTOR_CW  = -1
  AP_MOTORS_MATRIX_YAW_FACTOR_CCW = +1

Rigid-body check (NED body: X fwd, Y right, Z down; thrust acts along -Z):
  upward force F at (x,y):  Mx = -y*F,  My = +x*F
  horizontal force (Fx,Fy) at (x,y):  Mz = x*Fy - y*Fx
  positive Mx = roll right, positive My = nose up, positive Mz = nose right
"""
import math

DEG = math.pi / 180.0
TILT = 5.0
SIN_T = math.sin(TILT * DEG)

# geometry from v4.0 header
Y_OUT, Y_IN, X_ARM = 3.4, 1.6, 1.665
R_OUT = math.hypot(Y_OUT, X_ARM)   # 3.786  (v4.0 assumed tangential arm)
R_IN = math.hypot(Y_IN, X_ARM)     # 2.309
KQ = 0.0056 / 0.296 * SIN_T * R_OUT / R_OUT  # reaction torque arm equivalent (m/N)
KQ = 0.0056                        # C_Q/C_T * R_prop, per earlier derivation

# ---- v4.0 raw factors, transcribed verbatim from the onboard source ----
RO, RI = -1.0, -0.4706          # roll right outer / inner
LI, LO = 0.4706, 1.0            # roll left inner / outer
PF, PR = 1.0, -1.0              # pitch front / rear
YOC, YOW = -0.0971, 0.0971      # yaw outer CW / CCW
YIC, YIW = -0.0592, 0.0592      # yaw inner CW / CCW
THR = 0.9956

# motor: (name, pod, x, y, ring, is_upper, roll, pitch, yaw)
M = [
    ("M1", "P1", +X_ARM, +Y_OUT, "out", True,  RO, PF, YOC),
    ("M2", "P1", +X_ARM, +Y_OUT, "out", False, RO, PF, YOW),
    ("M3", "P2", +X_ARM, +Y_IN,  "in",  True,  RI, PF, YIC),
    ("M4", "P2", +X_ARM, +Y_IN,  "in",  False, RI, PF, YIW),
    ("M5", "P3", +X_ARM, -Y_IN,  "in",  True,  LI, PF, YIC),
    ("M6", "P3", +X_ARM, -Y_IN,  "in",  False, LI, PF, YIW),
    ("M7", "P4", +X_ARM, -Y_OUT, "out", True,  LO, PF, YOC),
    ("M8", "P4", +X_ARM, -Y_OUT, "out", False, LO, PF, YOW),
    ("M9", "P5", -X_ARM, +Y_OUT, "out", True,  RO, PR, YOC),
    ("M10", "P5", -X_ARM, +Y_OUT, "out", False, RO, PR, YOW),
    ("M11", "P6", -X_ARM, +Y_IN,  "in",  True,  RI, PR, YIC),
    ("M12", "P6", -X_ARM, +Y_IN,  "in",  False, RI, PR, YIW),
    ("M13", "P7", -X_ARM, -Y_IN,  "in",  True,  LI, PR, YIC),
    ("M14", "P7", -X_ARM, -Y_IN,  "in",  False, LI, PR, YIW),
    ("M15", "P8", -X_ARM, -Y_OUT, "out", True,  LO, PR, YOC),
    ("M16", "P8", -X_ARM, -Y_OUT, "out", False, LO, PR, YOW),
]

line = "=" * 74
print(line); print("PART 1  SIGN CONVENTION CHECK (vs ArduPilot add_motor)"); print(line)
bad = 0
for n, pod, x, y, ring, up, r, p, yw in M:
    er = "-" if y > 0 else "+"          # right -> negative roll_fac
    ep = "+" if x > 0 else "-"          # front -> positive pitch_fac
    ey = "-" if up else "+"             # upper = CW = negative
    gr = "-" if r < 0 else "+"
    gp = "-" if p < 0 else "+"
    gy = "-" if yw < 0 else "+"
    ok = (er == gr) and (ep == gp) and (ey == gy)
    if not ok:
        bad += 1
    print("  %-4s %s %-3s %-5s  roll %s(exp %s)  pitch %s(exp %s)  yaw %s(exp %s)  %s"
          % (n, pod, ring, "upper" if up else "lower", gr, er, gp, ep, gy, ey,
             "OK" if ok else "*** MISMATCH ***"))
print("\n  sign mismatches: %d / 16" % bad)

print(); print(line); print("PART 2  MAGNITUDE vs GEOMETRY"); print(line)
print("  roll  inner/outer = %.4f   geometric Y_in/Y_out = %.4f   %s"
      % (abs(RI / RO), Y_IN / Y_OUT, "OK" if abs(abs(RI / RO) - Y_IN / Y_OUT) < 1e-3 else "BAD"))
print("  pitch all |X| equal (%.3f m) -> uniform +-1.0                       OK" % X_ARM)
print("  yaw   outer = sin5*R_out/Y_out = %.4f  (code %.4f)" % (SIN_T * R_OUT / Y_OUT, abs(YOC)))
print("  yaw   inner = sin5*R_in /Y_out = %.4f  (code %.4f)" % (SIN_T * R_IN / Y_OUT, abs(YIC)))
print("  -> v4.0 yaw arms assume PURE TANGENTIAL tilt: R_out=%.3f R_in=%.3f" % (R_OUT, R_IN))
print("  -> reaction-torque term (%.4f m/N) is OMITTED from yaw factors" % KQ)

print(); print(line); print("PART 3  POST-NORMALISATION FACTORS (normalise_rpy_factors -> max 0.5)")
print(line)
kr = 0.5 / max(abs(m[6]) for m in M)
kp = 0.5 / max(abs(m[7]) for m in M)
ky = 0.5 / max(abs(m[8]) for m in M)
kt = 1.0 / THR
print("  roll  scale x%.4f     pitch scale x%.4f" % (kr, kp))
print("  yaw   scale x%.4f  <-- yaw factors amplified %.1fx by normalisation" % (ky, ky))
print("  thr   scale x%.4f -> all throttle factors become exactly 1.000 (no effect)" % kt)
print("  normalised yaw: outer +-%.4f   inner +-%.4f" % (0.5, abs(YIC) * ky))
print("  normalised roll: outer +-%.4f   inner +-%.4f" % (0.5, abs(RI) * kr))

print(); print(line); print("PART 4  DECOUPLING (does a yaw command disturb other axes?)")
print(line)
for cmd, idx in (("roll", 6), ("pitch", 7), ("yaw", 8)):
    sc = {6: kr, 7: kp, 8: ky}[idx]
    d = [m[idx] * sc for m in M]
    sMx = sum(-m[4 - 1] * 0 for m in M)  # placeholder
    sMx = sum(-m[3] * d[i] for i, m in enumerate(M))   # y = m[3]
    sMy = sum(m[2] * d[i] for i, m in enumerate(M))    # x = m[2]
    sT = sum(d)
    print("  unit %-5s cmd ->  Mx=%+9.4f   My=%+9.4f   sum(thrust)=%+9.4f"
          % (cmd, sMx, sMy, sT))
print("  (for the yaw row all three must be ~0: upper/lower deltas cancel inside each pod)")

print(); print(line); print("PART 5  TRUE PHYSICAL AUTHORITY PER UNIT NORMALISED COMMAND")
print(line)
Mx = sum(-m[3] * (m[6] * kr) for m in M)
My = sum(m[2] * (m[7] * kp) for m in M)
for label, arm_out, arm_in in (("v4.0 assumed (tangential tilt)", R_OUT, R_IN),
                               ("actual hardware (+-X/+-Y tilt) ", Y_OUT, X_ARM)):
    Mz = 0.0
    for m in M:
        a = arm_out if m[4] == "out" else arm_in
        Mz += abs(m[8] * ky) * (SIN_T * a + KQ)
    print("  %s :  Mz=%7.4f   Mz/Mx=%.4f  -> yaw is %.1fx weaker than roll"
          % (label, Mz, Mz / Mx, Mx / Mz))
print("  reference:  Mx=%.4f   My=%.4f   My/Mx=%.4f (pitch is %.2fx roll)"
      % (Mx, My, My / Mx, My / Mx))

print(); print(line); print("PART 6  REQUIRED PHYSICAL TILT DIRECTION IMPLIED BY v4.0 SIGNS")
print(line)
print("  upper motors have yaw_fac<0 -> their tilt force must give Mz<0 (nose LEFT)")
print("  Mz = x*Fy - y*Fx ; outer pods tilt along +-X, inner pods along +-Y")
print()
print("  pod  ring  x       y       UPPER tilts toward   LOWER tilts toward")
for pod, x, y, ring in (("P1", +X_ARM, +Y_OUT, "out"), ("P2", +X_ARM, +Y_IN, "in"),
                        ("P3", +X_ARM, -Y_IN, "in"), ("P4", +X_ARM, -Y_OUT, "out"),
                        ("P5", -X_ARM, +Y_OUT, "out"), ("P6", -X_ARM, +Y_IN, "in"),
                        ("P7", -X_ARM, -Y_IN, "in"), ("P8", -X_ARM, -Y_OUT, "out")):
    if ring == "out":                 # force along X, Mz = -y*Fx ; want Mz<0 -> Fx same sign as y
        u = "+X (nose)" if y > 0 else "-X (tail)"
        l = "-X (tail)" if y > 0 else "+X (nose)"
    else:                             # force along Y, Mz = +x*Fy ; want Mz<0 -> Fy opposite sign to x
        u = "-Y (left)" if x > 0 else "+Y (right)"
        l = "+Y (right)" if x > 0 else "-Y (left)"
    print("  %-4s %-5s %+.3f  %+.3f  %-20s %s" % (pod, ring, x, y, u, l))
