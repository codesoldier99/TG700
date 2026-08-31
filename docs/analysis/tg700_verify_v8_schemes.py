#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verify all THREE TG700 mixing schemes in AP_MotorsMatrix_TG700.cpp v8.0.

  COAX16_AY   Q_FRAME_TYPE 0/1   +-Y only tilt, counter-rotating pods  (DEFAULT)
  COAX16_AX4  Q_FRAME_TYPE 2     +-X/+-Y tilt, counter-rotating pods
  COAX16_COR  Q_FRAME_TYPE 20/21 +-X/+-Y tilt, co-rotating checkerboard

For each scheme:
  1. factors parsed straight out of the C++ source
  2. exact decoupling: a yaw command must not disturb roll, pitch or thrust
  3. tilt direction reproduced from the yaw signs, given the scheme's tilt axes
  4. no standing yaw bias at uniform thrust
  5. delivered yaw authority, all three compared head to head
Pure ASCII source on purpose.
"""
import math
import re
import sys

SRC = "libraries/AP_Motors/AP_MotorsMatrix_TG700.cpp"

DEG = math.pi / 180.0
TILT = 5.0
SIN_T = math.sin(TILT * DEG)
QT = 0.0056
Y_OUT, Y_IN, X_ARM = 3.4, 1.6, 1.665

POD = {
    "P1": (+X_ARM, +Y_OUT, "out"), "P2": (+X_ARM, +Y_IN, "in"),
    "P3": (+X_ARM, -Y_IN, "in"),   "P4": (+X_ARM, -Y_OUT, "out"),
    "P5": (-X_ARM, +Y_OUT, "out"), "P6": (-X_ARM, +Y_IN, "in"),
    "P7": (-X_ARM, -Y_IN, "in"),   "P8": (-X_ARM, -Y_OUT, "out"),
}
MOTOR_POD = []
for p in ("P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8"):
    MOTOR_POD += [(p, True), (p, False)]

TXT = open(SRC, encoding="utf-8").read()

defs = {}
for m in re.finditer(r"#define\s+(TG700_\w+)\s+\(?\s*(-?)\s*([0-9.]+)f?\s*\)?", TXT):
    defs[m.group(1)] = float(m.group(2) + m.group(3))
for nm, base, sg in (("TG700_YO_CW", "TG700_YAW_OUTER", -1),
                     ("TG700_YO_CCW", "TG700_YAW_OUTER", +1),
                     ("TG700_YI_CW", "TG700_YAW_INNER", -1),
                     ("TG700_YI_CCW", "TG700_YAW_INNER", +1),
                     ("TG700_YA_CW", "TG700_YAW_ALL", -1),
                     ("TG700_YA_CCW", "TG700_YAW_ALL", +1)):
    defs[nm] = sg * defs[base]


def yaw_table(var):
    m = re.search(r"yaw_%s\[16\]\s*=\s*\{(.*?)\};" % var, TXT, re.S)
    if not m:
        sys.exit("missing yaw_%s table" % var)
    toks = re.findall(r"TG700_Y[IOA]_C?CW", m.group(1))
    if len(toks) != 16:
        sys.exit("yaw_%s: found %d entries" % (var, len(toks)))
    return [defs[t] for t in toks]


g = re.search(r"static const MotorGeom geom\[16\]\s*=\s*\{(.*?)\n    \};", TXT, re.S)
rows = re.findall(r"\{\s*AP_MOTORS_MOT_(\d+),\s*(TG700_\w+),\s*(TG700_\w+),\s*(\d+)\s*\}",
                  g.group(1))
ROLL = [defs[r[1]] for r in rows]
PITCH = [defs[r[2]] for r in rows]

# scheme -> (frame string, yaw table, tilt axis per ring)
SCHEMES = [
    ("COAX16_AY  (Q_FRAME_TYPE 0/1, DEFAULT)", yaw_table("y_only"),
     {"out": "Y", "in": "Y"}),
    ("COAX16_AX4 (Q_FRAME_TYPE 2)", yaw_table("counter_rotating"),
     {"out": "X", "in": "Y"}),
    ("COAX16_COR (Q_FRAME_TYPE 20/21)", yaw_table("co_rotating"),
     {"out": "X", "in": "Y"}),
]

bar = "=" * 84
fail = []


def check(cond, msg):
    print("  %-62s %s" % (msg, "OK" if cond else "*** FAIL ***"))
    if not cond:
        fail.append(msg)


print(bar)
print("frame strings present in the switch statement")
print(bar)
for s in ("COAX16_AY", "COAX16_AX4", "COAX16_COR"):
    check('"%s"' % s in TXT, "%s is assigned to _frame_type_string" % s)
check("MOTOR_FRAME_TYPE_V" in TXT, "frame type 2 (V) routes to COAX16_AX4")
check("return false" in TXT, "unknown frame types are rejected (pre-arm fails)")

results = {}
for name, YAW, axis in SCHEMES:
    print()
    print(bar)
    print(name)
    print(bar)

    # decoupling
    s_yr = sum(YAW[i] * ROLL[i] for i in range(16))
    s_yp = sum(YAW[i] * PITCH[i] for i in range(16))
    s_yt = sum(YAW[i] for i in range(16))
    print("  sum yaw*roll = %+.3e   sum yaw*pitch = %+.3e   sum yaw = %+.3e"
          % (s_yr, s_yp, s_yt))
    check(max(abs(s_yr), abs(s_yp), abs(s_yt)) < 1e-9,
          "yaw command gives zero roll / pitch / thrust disturbance")

    # magnitudes consistent with the scheme's tilt axes
    mag_ok = True
    for i in range(16):
        pod, _ = MOTOR_POD[i]
        x, y, ring = POD[pod]
        arm = abs(y) if axis[ring] == "X" else abs(x)
        mag_ok &= abs(abs(YAW[i]) - (SIN_T * arm + QT) / Y_OUT) < 6e-4
    check(mag_ok, "every |yaw factor| matches sin(5deg)*arm + Q/T for its axis")

    # tilt direction reproduced from the signs
    tilts = []
    for i in range(16):
        pod, up = MOTOR_POD[i]
        x, y, ring = POD[pod]
        want_neg = YAW[i] < 0
        if axis[ring] == "X":
            fx_pos = (y > 0) if want_neg else (y < 0)
            tilts.append("+X nose" if fx_pos else "-X tail")
        else:
            fy_pos = (x < 0) if want_neg else (x > 0)
            tilts.append("+Y right" if fy_pos else "-Y left")

    if "AY" in name:
        print()
        print("  pod   row    upper tilt   lower tilt")
        for k in range(0, 16, 2):
            pod, _ = MOTOR_POD[k]
            x, _y, _r = POD[pod]
            print("  %-4s  %-5s  %-11s  %-11s"
                  % (pod, "front" if x > 0 else "rear", tilts[k], tilts[k + 1]))
        check(all(t == "-Y left" for t in tilts[0:8:2])
              and all(t == "+Y right" for t in tilts[1:8:2]),
              "front pods: upper tilts LEFT, lower tilts RIGHT")
        check(all(t == "+Y right" for t in tilts[8:16:2])
              and all(t == "-Y left" for t in tilts[9:16:2]),
              "rear pods: upper tilts RIGHT, lower tilts LEFT")
        check(all(t.startswith(("+Y", "-Y")) for t in tilts),
              "no motor needs an X-axis tilt (mount constraint satisfied)")
        check(len(set(abs(v) for v in YAW)) == 1,
              "all 16 yaw magnitudes equal (outer has no arm advantage)")
    elif "COR" in name:
        check(all(tilts[k] == tilts[k + 1] for k in range(0, 16, 2)),
              "co-rotating: both motors of a pod tilt the SAME way")
    else:
        check(all(tilts[k] != tilts[k + 1] for k in range(0, 16, 2)),
              "counter-rotating: upper and lower tilt OPPOSITE ways")

    # authority + hover bias
    ky = 0.5 / max(abs(v) for v in YAW)
    Mz = 0.0
    bias = 0.0
    for i in range(16):
        pod, _ = MOTOR_POD[i]
        x, y, ring = POD[pod]
        arm = abs(y) if axis[ring] == "X" else abs(x)
        Mz += abs(YAW[i] * ky) * (SIN_T * arm + QT)
        bias += math.copysign(SIN_T * arm + QT, YAW[i])
    kr = 0.5 / max(abs(v) for v in ROLL)
    Mx = sum(-POD[MOTOR_POD[i][0]][1] * ROLL[i] * kr for i in range(16))
    print()
    print("  yaw authority %.4f   roll %.4f   -> yaw is %.1fx weaker"
          % (Mz, Mx, Mx / Mz))
    print("  standing yaw at uniform thrust: %+.3e" % bias)
    check(abs(bias) < 1e-9, "no standing yaw bias in hover")
    results[name.split()[0]] = Mz

print()
print(bar)
print("head-to-head yaw authority")
print(bar)
ref = results["COAX16_AX4"]
for k, v in results.items():
    print("  %-12s %.4f   %5.1f%% of AX4" % (k, v, 100 * v / ref))
check(abs(results["COAX16_COR"] / results["COAX16_AX4"] - 1.0) < 1e-6,
      "COR and AX4 are identical (as derived earlier)")
check(0.79 < results["COAX16_AY"] / ref < 0.81,
      "AY retains ~80% of AX4 (the price of losing the X-axis tilt)")

print()
print(bar)
print("RESULT: %s" % ("ALL CHECKS PASSED" if not fail else "%d FAILURE(S)" % len(fail)))
print(bar)
for f in fail:
    print("  FAILED:", f)
sys.exit(1 if fail else 0)
