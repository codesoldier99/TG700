#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解析 AP_MotorsMatrix_TG700.cpp 的 v6.0 混控表, 独立校核:
  1) 16 台电机是否按 上层CW / 下层CCW 交替
  2) roll / pitch / yaw / throttle 因子是否与几何一致
  3) 打偏航舵时 roll / pitch / 油门是否解耦 (各列加权和为 0)
  4) 由 yaw 因子反推的力矩臂是否等于 ±X/±Y 四向约束下的最优值
"""
import math
import os
import re

SRC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "libraries", "AP_Motors", "AP_MotorsMatrix_TG700.cpp"))

raw = open(SRC, "rb").read()
try:
    text = raw.decode("utf-8")
except UnicodeDecodeError:
    text = raw.decode("gbk")

# ---- 取出 #define 常量 ----
consts = {}
for m in re.finditer(r"#define\s+(TG700_\w+)\s+\(?\s*([+-]?[0-9.]+)f?\s*\)?", text):
    consts[m.group(1)] = float(m.group(2))

# ---- 取出 add_motor_raw 调用 ----
pat = re.compile(
    r"add_motor_raw\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*(TG700_\w+)\s*,\s*(TG700_\w+)"
    r"\s*,\s*(TG700_\w+)\s*,\s*(\d+)\s*,\s*(TG700_\w+)\s*\)")
motors = []
for m in pat.finditer(text):
    motors.append(dict(idx=int(m.group(1)),
                       roll=consts[m.group(2)], pitch=consts[m.group(3)],
                       yaw=consts[m.group(4)], yaw_name=m.group(4),
                       order=int(m.group(5)), thr=consts[m.group(6)]))

print("解析到 %d 台电机, %d 个常量" % (len(motors), len(consts)))
assert len(motors) == 16, "电机数量应为 16"

ftype = re.search(r'_frame_type_string\s*=\s*"([^"]+)"', text).group(1)
print("frame_type_string = %s" % ftype)

# ---- 期望几何 ----
PODS = [  # (吊舱, X, Y, 环, Motor上, Motor下)
    ("P1", 1.665,  3.4, "outer",  1,  2),
    ("P2", 1.665,  1.6, "inner",  3,  4),
    ("P3", 1.665, -1.6, "inner",  5,  6),
    ("P4", 1.665, -3.4, "outer",  7,  8),
    ("P5", -1.665,  3.4, "outer",  9, 10),
    ("P6", -1.665,  1.6, "inner", 11, 12),
    ("P7", -1.665, -1.6, "inner", 13, 14),
    ("P8", -1.665, -3.4, "outer", 15, 16),
]
Y_MAX = 3.4
K = 0.0056
SIN5 = math.sin(math.radians(5.0))
by_idx = {m["idx"]: m for m in motors}

print()
print("=" * 88)
print("1) 逐台校核")
print("=" * 88)
print("%-5s%-6s%-7s%9s%9s%10s%9s%12s" %
      ("吊舱", "电机", "层/转向", "roll", "pitch", "yaw", "throttle", "反推力矩臂"))
ok = True
for tag, X, Y, ring, mu, ml in PODS:
    arm = abs(Y) if ring == "outer" else abs(X)      # 四向最优力矩臂
    exp_mag = (SIN5 * arm + K) / Y_MAX
    exp_roll = -Y / Y_MAX
    exp_pitch = 1.0 if X > 0 else -1.0
    for mi, layer, sign in ((mu, "上/CW", -1.0), (ml, "下/CCW", +1.0)):
        m = by_idx[mi]
        # 由 yaw 因子反推力矩臂
        back_arm = (abs(m["yaw"]) * Y_MAX - K) / SIN5
        bad = []
        if abs(m["roll"] - exp_roll) > 1e-3:
            bad.append("roll")
        if abs(m["pitch"] - exp_pitch) > 1e-3:
            bad.append("pitch")
        if abs(m["yaw"] - sign * exp_mag) > 6e-4:
            bad.append("yaw")
        if abs(m["thr"] - math.cos(math.radians(5.0))) > 2e-3:
            bad.append("thr")
        flag = "  <== " + ",".join(bad) if bad else ""
        if bad:
            ok = False
        print("%-5s%-6d%-7s%9.4f%9.4f%10.4f%9.4f%10.3f m%s" %
              (tag, mi, layer, m["roll"], m["pitch"], m["yaw"], m["thr"],
               back_arm, flag))

print()
print("2) 转向交替: ", end="")
alt = all(by_idx[mu]["yaw"] < 0 < by_idx[ml]["yaw"] for _, _, _, _, mu, ml in PODS)
print("通过 (每组上层 yaw<0 = CW, 下层 yaw>0 = CCW)" if alt else "**不通过**")
ok = ok and alt

print()
print("=" * 88)
print("3) 解耦校核: 打偏航舵时 roll / pitch / throttle 的加权和是否为 0")
print("=" * 88)
sr = sum(m["yaw"] * m["roll"] for m in motors)
sp = sum(m["yaw"] * m["pitch"] for m in motors)
st = sum(m["yaw"] * m["thr"] for m in motors)
for name, v in (("Σ yaw×roll ", sr), ("Σ yaw×pitch", sp), ("Σ yaw×thr  ", st)):
    print("  %s = %+.3e  %s" % (name, v, "OK" if abs(v) < 1e-9 else "**耦合!**"))
    ok = ok and abs(v) < 1e-9

sy_r = sum(m["roll"] * m["yaw"] for m in motors)
print("  Σ roll×yaw  = %+.3e  (打横滚舵不产生偏航)" % sy_r)

print()
print("=" * 88)
print("4) 归一化后 (ArduPilot normalise_rpy_factors: max|fac| -> 0.5)")
print("=" * 88)
ymax = max(abs(m["yaw"]) for m in motors)
rmax = max(abs(m["roll"]) for m in motors)
print("  外侧 yaw_fac -> %.4f ;  内侧 yaw_fac -> %.4f  (内/外 = %.4f)"
      % (0.5, 0.5 * min(abs(m["yaw"]) for m in motors) / ymax,
         min(abs(m["yaw"]) for m in motors) / ymax))
print("  归一化前 |yaw|max/|roll|max = %.4f  ->  物理偏航比横滚弱 %.1f 倍"
      % (ymax / rmax, rmax / ymax))

print()
print("总校核结果: %s" % ("全部通过" if ok else "**存在不一致, 见上方标记**"))
