# -*- coding: utf-8 -*-
"""
Parse AP_MotorsMatrix_TG700.cpp and verify the mixer matrix is physically sane.

Checks performed:
  1. all 16 motors present, factors resolve
  2. throttle / roll / pitch / yaw axes mutually decoupled (every cross term 0)
  3. each coaxial pair co-rotates (v5.0 requirement)
  4. checkerboard pattern matches the intended P1..P8 spec
  5. CW / CCW motor counts balance 8:8 (net reaction torque zero at trim)
  6. reproduce ArduPilot's normalise_rpy_factors() and report resulting authority
"""
import os
import re
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "libraries", "AP_Motors", "AP_MotorsMatrix_TG700.cpp")

# intended v5.0 rotation spec, from the user's requirement
SPEC = {"P1": "CW", "P2": "CCW", "P3": "CW", "P4": "CCW",
        "P5": "CCW", "P6": "CW", "P7": "CCW", "P8": "CW"}

# which motor numbers belong to which pair, and their ring
PAIR_OF = {}
RING_OF = {}
for pair, mots, ring in [("P1", (1, 2), "outer"), ("P2", (3, 4), "inner"),
                         ("P3", (5, 6), "inner"), ("P4", (7, 8), "outer"),
                         ("P5", (9, 10), "outer"), ("P6", (11, 12), "inner"),
                         ("P7", (13, 14), "inner"), ("P8", (15, 16), "outer")]:
    for m in mots:
        PAIR_OF[m] = pair
        RING_OF[m] = ring

src = open(SRC, encoding="utf-8").read()

# ---- resolve the #defines ----
defines = {}
for name, val in re.findall(r"#define\s+(TG700_\w+)\s+\(?\s*(-?[\d.]+)f?\s*\)?", src):
    defines[name] = float(val)

# ---- parse add_motor_raw calls ----
motors = {}
pat = re.compile(
    r"add_motor_raw\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*(TG700_\w+)\s*,\s*(TG700_\w+)\s*,"
    r"\s*(TG700_\w+)\s*,\s*(\d+)\s*,\s*(TG700_\w+)\s*\)")
for m in pat.finditer(src):
    num, rollv, pitchv, yawv, order, thrv = m.groups()
    num = int(num)
    motors[num] = dict(roll=defines[rollv], pitch=defines[pitchv],
                       yaw=defines[yawv], thr=defines[thrv],
                       yaw_name=yawv, order=int(order))

fail = []
print("=" * 76)
print("TG700 mixer verification  (frame_type_string = %s)"
      % (re.search(r'_frame_type_string\s*=\s*"([^"]+)"', src).group(1)))
print("=" * 76)

# ---- 1. completeness ----
missing = [i for i in range(1, 17) if i not in motors]
if missing:
    fail.append("missing motors: %s" % missing)
print("1. motors parsed : %d/16 %s" % (len(motors), "OK" if not missing else "FAIL"))

# ---- table ----
print("\n   mot  pair  ring   rot   roll     pitch    yaw      thr")
for i in range(1, 17):
    d = motors[i]
    rot = "CW" if d["yaw"] < 0 else "CCW"
    d["rot"] = rot
    print("   %2d   %-4s  %-5s  %-4s  %+.4f  %+.4f  %+.4f  %.4f"
          % (i, PAIR_OF[i], RING_OF[i], rot, d["roll"], d["pitch"], d["yaw"], d["thr"]))

# ---- 2. decoupling ----
print("\n2. axis decoupling (all cross terms must be 0)")
axes = ("thr", "roll", "pitch", "yaw")
TOL = 1e-6
for a in range(len(axes)):
    for b in range(a + 1, len(axes)):
        ka, kb = axes[a], axes[b]
        s = sum(motors[i][ka] * motors[i][kb] for i in range(1, 17))
        ok = abs(s) < TOL
        if not ok:
            fail.append("cross term %s.%s = %.6e" % (ka, kb, s))
        print("   sum(%-5s * %-5s) = %+.3e  %s" % (ka, kb, s, "OK" if ok else "FAIL"))
for k in ("roll", "pitch", "yaw"):
    s = sum(motors[i][k] for i in range(1, 17))
    ok = abs(s) < TOL
    if not ok:
        fail.append("sum(%s) = %.6e" % (k, s))
    print("   sum(%-5s)          = %+.3e  %s" % (k, s, "OK" if ok else "FAIL"))

# ---- 3. pairs co-rotate ----
print("\n3. coaxial pairs co-rotate (v5.0 requirement)")
for pair in ("P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8"):
    ms = [i for i in range(1, 17) if PAIR_OF[i] == pair]
    rots = {motors[i]["rot"] for i in ms}
    ok = len(rots) == 1
    if not ok:
        fail.append("pair %s does not co-rotate: %s" % (pair, rots))
    print("   %s  motors %-7s rot=%-4s  %s"
          % (pair, str(tuple(ms)), "/".join(sorted(rots)), "OK" if ok else "FAIL"))

# ---- 4. matches spec ----
print("\n4. checkerboard matches the requested spec")
for pair, want in SPEC.items():
    ms = [i for i in range(1, 17) if PAIR_OF[i] == pair]
    got = motors[ms[0]]["rot"]
    ok = got == want
    if not ok:
        fail.append("pair %s: want %s got %s" % (pair, want, got))
    print("   %s  want %-4s got %-4s  %s" % (pair, want, got, "OK" if ok else "FAIL"))

# ---- 5. rotation balance ----
ncw = sum(1 for i in range(1, 17) if motors[i]["rot"] == "CW")
nccw = 16 - ncw
ok = ncw == nccw == 8
if not ok:
    fail.append("rotation imbalance CW=%d CCW=%d" % (ncw, nccw))
print("\n5. rotation balance : CW=%d  CCW=%d  %s" % (ncw, nccw, "OK" if ok else "FAIL"))

# ---- 6. reproduce normalise_rpy_factors() ----
print("\n6. after ArduPilot normalise_rpy_factors() (each axis scaled to max 0.5)")
norm = {}
for k in ("roll", "pitch", "yaw"):
    peak = max(abs(motors[i][k]) for i in range(1, 17))
    norm[k] = 0.5 / peak
    print("   %-5s peak raw=%.4f -> scale x%.3f" % (k, peak, norm[k]))
print("   => yaw factors are scaled up %.2fx more than roll, which is exactly how"
      % (norm["yaw"] / norm["roll"]))
print("      the physical yaw weakness gets hidden from the rate PID.")

# group composition (the reason authority is unchanged vs v4.0)
pos = [i for i in range(1, 17) if motors[i]["yaw"] > 0]
neg = [i for i in range(1, 17) if motors[i]["yaw"] < 0]
for grp, lbl in ((pos, "+yaw"), (neg, "-yaw")):
    no = sum(1 for i in grp if RING_OF[i] == "outer")
    ni = sum(1 for i in grp if RING_OF[i] == "inner")
    print("   %s group: %2d motors (%d outer, %d inner)  sum|yaw|=%.4f"
          % (lbl, len(grp), no, ni, sum(abs(motors[i]["yaw"]) for i in grp)))

print("\n" + "=" * 76)
if fail:
    print("RESULT: %d PROBLEM(S)" % len(fail))
    for f in fail:
        print("  - " + f)
    sys.exit(1)
print("RESULT: ALL CHECKS PASSED")
print("=" * 76)
