#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verify BOTH TG700 mixing schemes in AP_MotorsMatrix_TG700.cpp v7.0.

Scheme A "COAX16_AX4"  counter-rotating pods (upper CW / lower CCW)
Scheme B "COAX16_COR"  co-rotating pods, checkerboard across pods  (= v5.0)

Checks performed for each scheme:
  1. roll / pitch factors identical between schemes (structurally enforced)
  2. rotation pattern matches the documented intent
  3. exact decoupling: a yaw command must not disturb roll, pitch or thrust
  4. tilt directions derived from the yaw signs under the +-X / +-Y constraint
  5. delivered yaw moment, compared head to head
  6. hover torque / tilt-force balance (no standing bias)
  7. which motors must have their rotation reversed to move A -> B
"""
import math
import re
import sys

SRC = "libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp"

DEG = math.pi / 180.0
TILT = 5.0
SIN_T = math.sin(TILT * DEG)
QT = 0.0056                      # C_Q/C_T * R_prop, reaction torque arm (m)
Y_OUT, Y_IN, X_ARM = 3.4, 1.6, 1.665

# pod -> (x, y, ring)
POD = {
    "P1": (+X_ARM, +Y_OUT, "out"), "P2": (+X_ARM, +Y_IN, "in"),
    "P3": (+X_ARM, -Y_IN, "in"),   "P4": (+X_ARM, -Y_OUT, "out"),
    "P5": (-X_ARM, +Y_OUT, "out"), "P6": (-X_ARM, +Y_IN, "in"),
    "P7": (-X_ARM, -Y_IN, "in"),   "P8": (-X_ARM, -Y_OUT, "out"),
}
# motor index (0-15) -> (pod, is_upper)
MOTOR_POD = []
for p in ("P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8"):
    MOTOR_POD.append((p, True))
    MOTOR_POD.append((p, False))


def parse_source():
    """Pull the #defines and the two yaw tables straight out of the C++ source."""
    txt = open(SRC, encoding="utf-8").read()

    defs = {}
    for m in re.finditer(r"#define\s+(TG700_\w+)\s+\(?\s*(-?)\s*([0-9.]+)f?\s*\)?", txt):
        defs[m.group(1)] = float(m.group(2) + m.group(3))
    # derived aliases that reference other macros
    for name, base, sign in (("TG700_YO_CW", "TG700_YAW_OUTER", -1),
                             ("TG700_YO_CCW", "TG700_YAW_OUTER", +1),
                             ("TG700_YI_CW", "TG700_YAW_INNER", -1),
                             ("TG700_YI_CCW", "TG700_YAW_INNER", +1)):
        defs[name] = sign * defs[base]

    def table(varname):
        m = re.search(r"yaw_%s\[16\]\s*=\s*\{(.*?)\};" % varname, txt, re.S)
        if not m:
            sys.exit("could not find yaw_%s table in %s" % (varname, SRC))
        toks = re.findall(r"TG700_Y[IO]_C?CW", m.group(1))
        if len(toks) != 16:
            sys.exit("yaw_%s: expected 16 entries, found %d" % (varname, len(toks)))
        return [defs[t] for t in toks], toks

    geom = re.search(r"static const MotorGeom geom\[16\]\s*=\s*\{(.*?)\n    \};", txt, re.S)
    if not geom:
        sys.exit("could not find geom[16] table")
    rows = re.findall(r"\{\s*AP_MOTORS_MOT_(\d+),\s*(TG700_\w+),\s*(TG700_\w+),\s*(\d+)\s*\}",
                      geom.group(1))
    if len(rows) != 16:
        sys.exit("geom: expected 16 rows, found %d" % len(rows))
    roll = [defs[r[1]] for r in rows]
    pitch = [defs[r[2]] for r in rows]
    order = [int(r[3]) for r in rows]
    mot = [int(r[0]) for r in rows]

    a, a_tok = table("counter_rotating")
    b, b_tok = table("co_rotating")
    return defs, roll, pitch, order, mot, a, a_tok, b, b_tok


defs, ROLL, PITCH, ORDER, MOT, YAW_A, TOK_A, YAW_B, TOK_B = parse_source()
THR = defs["TG700_THROTTLE_FACTOR"]

bar = "=" * 86
fail = []


def check(cond, msg):
    print("  %-64s %s" % (msg, "OK" if cond else "*** FAIL ***"))
    if not cond:
        fail.append(msg)


print(bar)
print("0) source parsing")
print(bar)
print("  yaw outer = %+.4f   yaw inner = %+.4f   thr = %.4f"
      % (defs["TG700_YAW_OUTER"], defs["TG700_YAW_INNER"], THR))
check(MOT == list(range(1, 17)), "geom covers AP_MOTORS_MOT_1..16 in order")
check(ORDER == list(range(1, 17)), "motor test order is 1..16")
check(abs(abs(defs["TG700_YAW_OUTER"]) - (SIN_T * Y_OUT + QT) / Y_OUT) < 5e-4,
      "outer yaw magnitude = (sin5*3.400 + Q/T)/3.4")
check(abs(abs(defs["TG700_YAW_INNER"]) - (SIN_T * X_ARM + QT) / Y_OUT) < 5e-4,
      "inner yaw magnitude = (sin5*1.665 + Q/T)/3.4")

print()
print(bar)
print("1) roll / pitch shared between schemes (only yaw may differ)")
print(bar)
exp_roll, exp_pitch = [], []
for pod, _up in MOTOR_POD:
    x, y, ring = POD[pod]
    exp_roll.append(-(y / Y_OUT))          # ArduPilot: right side negative
    exp_pitch.append(1.0 if x > 0 else -1.0)
check(all(abs(ROLL[i] - exp_roll[i]) < 1e-3 for i in range(16)),
      "roll factors equal -Y/Y_max for all 16 motors")
check(all(abs(PITCH[i] - exp_pitch[i]) < 1e-9 for i in range(16)),
      "pitch factors are +1 front / -1 rear")

for name, YAW, TOK in (("A  COAX16_AX4  counter-rotating", YAW_A, TOK_A),
                       ("B  COAX16_COR  co-rotating (v5.0)", YAW_B, TOK_B)):
    print()
    print(bar)
    print("scheme %s" % name)
    print(bar)

    # --- rotation pattern ---
    print("  pod   ring  upper      lower      tilt axis   both/mirrored")
    same_cnt = 0
    for k in range(0, 16, 2):
        pod, _ = MOTOR_POD[k]
        x, y, ring = POD[pod]
        u = "CW " if YAW[k] < 0 else "CCW"
        l = "CW " if YAW[k + 1] < 0 else "CCW"
        same = (YAW[k] > 0) == (YAW[k + 1] > 0)
        same_cnt += same
        axis = "+-X" if ring == "out" else "+-Y"
        print("  %-4s  %-4s  %s(%+.4f) %s(%+.4f)  %-9s  %s"
              % (pod, ring, u, YAW[k], l, YAW[k + 1], axis,
                 "same" if same else "mirrored"))
    if "AX4" in name:
        check(same_cnt == 0, "all 8 pods counter-rotate internally")
    else:
        check(same_cnt == 8, "all 8 pods co-rotate internally")
        cw = [MOTOR_POD[k][0] for k in range(0, 16, 2) if YAW[k] < 0]
        check(cw == ["P1", "P3", "P6", "P8"],
              "CW pods are P1,P3,P6,P8 (checkerboard as specified)")
        n_out = sum(1 for p in cw if POD[p][2] == "out")
        check(n_out == 2, "CW group holds 2 outer + 2 inner (balanced)")

    # --- decoupling ---
    print()
    s_yr = sum(YAW[i] * ROLL[i] for i in range(16))
    s_yp = sum(YAW[i] * PITCH[i] for i in range(16))
    s_yt = sum(YAW[i] for i in range(16))
    s_rp = sum(ROLL[i] * PITCH[i] for i in range(16))
    print("  sum yaw*roll  = %+.3e" % s_yr)
    print("  sum yaw*pitch = %+.3e" % s_yp)
    print("  sum yaw       = %+.3e   (net thrust change from a yaw command)" % s_yt)
    print("  sum roll*pitch= %+.3e" % s_rp)
    check(max(abs(s_yr), abs(s_yp), abs(s_yt), abs(s_rp)) < 1e-9,
          "yaw command produces zero roll / pitch / thrust disturbance")

    # --- implied tilt directions ---
    print()
    print("  required tilt direction implied by the yaw signs:")
    print("  motor  pod   ring  yaw_fac    tilt toward")
    tilts = []
    for i in range(16):
        pod, up = MOTOR_POD[i]
        x, y, ring = POD[pod]
        want_neg = YAW[i] < 0            # need Mz < 0
        if ring == "out":                # force along X: Mz = -y*Fx
            fx_pos = (y > 0) if want_neg else (y < 0)
            d = "+X \u673a\u5934" if fx_pos else "-X \u673a\u5c3e"
        else:                            # force along Y: Mz = +x*Fy
            fy_pos = (x < 0) if want_neg else (x > 0)
            d = "+Y \u53f3" if fy_pos else "-Y \u5de6"
        tilts.append(d)
        print("  M%-4d  %-4s  %-4s  %+.4f    %s"
              % (i + 1, pod, "\u4e0a" if up else "\u4e0b", YAW[i], d))
    if "COR" in name:
        check(all(tilts[k] == tilts[k + 1] for k in range(0, 16, 2)),
              "co-rotating: both motors of a pod tilt the SAME way")
        # outward rule: outer pods lean away from CG along X, inner along Y
        ok = True
        for k in range(0, 16, 2):
            pod, _ = MOTOR_POD[k]
            x, y, ring = POD[pod]
            if ring == "out":
                ok &= tilts[k].startswith("+X") == (x > 0)
            else:
                ok &= tilts[k].startswith("+Y") == (y > 0)
        check(ok, "all pods lean OUTWARD (outer along X, inner along Y)")
    else:
        check(all(tilts[k] != tilts[k + 1] for k in range(0, 16, 2)),
              "counter-rotating: upper and lower of a pod tilt OPPOSITE ways")

    # --- delivered yaw moment + hover balance ---
    ky = 0.5 / max(abs(v) for v in YAW)
    Mz = 0.0
    hover = 0.0
    for i in range(16):
        pod, _ = MOTOR_POD[i]
        x, y, ring = POD[pod]
        arm = abs(y) if ring == "out" else abs(x)
        Mz += abs(YAW[i] * ky) * (SIN_T * arm + QT)
        hover += math.copysign(SIN_T * arm + QT, YAW[i])   # unit thrust everywhere
    kr = 0.5 / max(abs(v) for v in ROLL)
    Mx = sum(-POD[MOTOR_POD[i][0]][1] * ROLL[i] * kr for i in range(16))
    print()
    print("  yaw moment per unit normalised cmd : %.4f  (T*m)" % Mz)
    print("  roll moment per unit normalised cmd: %.4f  -> yaw is %.1fx weaker" % (Mx, Mx / Mz))
    print("  standing yaw moment at equal thrust: %+.3e  (must be 0: no hover spin)" % hover)
    check(abs(hover) < 1e-9, "no standing yaw bias at uniform thrust")
    globals()["MZ_" + name[0]] = Mz

print()
print(bar)
print("2) head-to-head")
print(bar)
print("  scheme A yaw moment = %.4f" % MZ_A)
print("  scheme B yaw moment = %.4f" % MZ_B)
print("  ratio B/A           = %.4f" % (MZ_B / MZ_A))
check(abs(MZ_B / MZ_A - 1.0) < 1e-6,
      "both schemes deliver IDENTICAL yaw authority (as derived analytically)")

print()
print(bar)
print("3) hardware work to move from scheme A (current aircraft) to scheme B")
print(bar)
print("  motors whose rotation must be reversed (ESC direction + opposite-hand prop):")
rev = []
for i in range(16):
    pod, up = MOTOR_POD[i]
    if (YAW_A[i] < 0) != (YAW_B[i] < 0):
        rev.append("%s%s" % (pod, "\u4e0a" if up else "\u4e0b"))
print("   ", ", ".join(rev))
check(len(rev) == 8, "exactly 8 of 16 motors need reversal")

print()
print(bar)
print("RESULT: %s" % ("ALL CHECKS PASSED" if not fail else "%d FAILURE(S)" % len(fail)))
print(bar)
for f in fail:
    print("  FAILED:", f)
sys.exit(1 if fail else 0)
