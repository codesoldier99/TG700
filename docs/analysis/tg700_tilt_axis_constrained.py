#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TG700 偏航倾角方案推导 —— 电机座只能沿 ±X / ±Y 四个方向倾斜的约束下

机械约束: 电机座倾斜方向只能平行于 Y 轴(左/右) 或平行于 X 轴(机头/机尾),
          无法做到纯切向(垂直于中心-电机连线)。

坐标系: ArduPilot 体轴系 NED —— +X 机头, +Y 右侧, +Z 向下; 正偏航 = 俯视顺时针。

输出:
  1) 四向约束下每个吊舱的最优倾斜轴与朝向
  2) 相对理想切向方案的偏航权限损失
  3) 力/力矩对消校核 (是否引入平移或滚转俯仰耦合)
  4) 新的混控 yaw 因子
  5) 倾角加大后的权限-升力损失权衡表
"""
import math
from itertools import product

SIN5 = math.sin(math.radians(5.0))

# 桨反扭矩项: C_Q/C_T * R_prop, 单位 m (每单位推力产生的偏航力矩)
K_REACTION = 0.0056

# 吊舱几何: 名称, (X, Y) m, 内外侧
PODS = [
    ("P1", "前右外", ( 1.665,  3.4), "outer"),
    ("P2", "前右内", ( 1.665,  1.6), "inner"),
    ("P3", "前左内", ( 1.665, -1.6), "inner"),
    ("P4", "前左外", ( 1.665, -3.4), "outer"),
    ("P5", "后右外", (-1.665,  3.4), "outer"),
    ("P6", "后右内", (-1.665,  1.6), "inner"),
    ("P7", "后左内", (-1.665, -1.6), "inner"),
    ("P8", "后左外", (-1.665, -3.4), "outer"),
]

# 四个可用倾斜方向 (机体系 XY 平面单位向量)
DIRS = {
    "机头": ( 1.0,  0.0),
    "机尾": (-1.0,  0.0),
    "右":   ( 0.0,  1.0),
    "左":   ( 0.0, -1.0),
}

# 转向 -> 反扭矩符号 (CW 桨的反扭矩使机体逆时针 = 负偏航)
SPIN_SIGN = {"CW": -1.0, "CCW": +1.0}


def yaw_arm(r, d):
    """单位水平力沿方向 d 作用于位置 r 时产生的偏航力矩臂 g = r_x*d_y - r_y*d_x"""
    return r[0] * d[1] - r[1] * d[0]


def motor_yaw_coeff(r, d, spin, tilt_deg):
    """单台电机每单位推力产生的偏航力矩 (N*m/N), 含倾角项与反扭矩项"""
    tilt = math.sin(math.radians(tilt_deg)) * yaw_arm(r, d)
    return tilt + SPIN_SIGN[spin] * K_REACTION


# ---------------------------------------------------------------------------
# 1) 各吊舱可用方向的偏航力矩臂
# ---------------------------------------------------------------------------
print("=" * 78)
print("1) 四个可用倾斜方向的偏航力矩臂 |g|  (单位 m, 越大越好)")
print("=" * 78)
print(f"{'吊舱':<6}{'位置':<8}{'坐标(X,Y)':<18}", end="")
for name in DIRS:
    print(f"{name:>8}", end="")
print(f"{'  最优轴':<10}")

for tag, pos_name, r, ring in PODS:
    print(f"{tag:<6}{pos_name:<8}{str(r):<18}", end="")
    arms = {}
    for name, d in DIRS.items():
        g = yaw_arm(r, d)
        arms[name] = g
        print(f"{g:>8.3f}", end="")
    best = max(arms, key=lambda n: abs(arms[n]))
    axis = "±X (机头/机尾)" if best in ("机头", "机尾") else "±Y (左/右)"
    print(f"  {axis}")

print()
print("结论: |g| = |r_y| (沿±X倾斜) 或 |r_x| (沿±Y倾斜)")
print("      |r_x| = 1.665 m 对所有吊舱相同")
print("      外侧 |r_y| = 3.4 m  > 1.665  -> 外侧四组应沿 ±X 倾斜")
print("      内侧 |r_y| = 1.6 m  < 1.665  -> 内侧四组应沿 ±Y 倾斜")


# ---------------------------------------------------------------------------
# 2) 确定每台电机的具体朝向
#    设计要求: 上层 CW 的 c<0, 下层 CCW 的 c>0, 使差动(上减下增)产生正偏航
# ---------------------------------------------------------------------------
def build_scheme(axis_choice, tilt_deg=5.0):
    """axis_choice: dict pod_tag -> 'X' or 'Y'; 返回每台电机的朝向与系数"""
    out = []
    for tag, pos_name, r, ring in PODS:
        axis = axis_choice[tag]
        cand = ["机头", "机尾"] if axis == "X" else ["右", "左"]
        # 上层 CW: 选使 c 最负的方向; 下层 CCW: 选使 c 最正的方向
        c_up = {n: motor_yaw_coeff(r, DIRS[n], "CW", tilt_deg) for n in cand}
        c_lo = {n: motor_yaw_coeff(r, DIRS[n], "CCW", tilt_deg) for n in cand}
        dir_up = min(c_up, key=lambda n: c_up[n])
        dir_lo = max(c_lo, key=lambda n: c_lo[n])
        out.append(dict(tag=tag, pos=pos_name, r=r, ring=ring, axis=axis,
                        dir_up=dir_up, c_up=c_up[dir_up],
                        dir_lo=dir_lo, c_lo=c_lo[dir_lo]))
    return out


AXIS_OPT = {t: ("X" if ring == "outer" else "Y") for t, _, _, ring in PODS}
AXIS_ALLX = {t: "X" for t, _, _, _ in PODS}
AXIS_ALLY = {t: "Y" for t, _, _, _ in PODS}

scheme = build_scheme(AXIS_OPT)

print()
print("=" * 78)
print("2) 逐臂安装方向表 (倾角 5°, 上层 CW / 下层 CCW)")
print("=" * 78)
print(f"{'吊舱':<6}{'位置':<8}{'倾斜轴':<14}{'上层CW朝向':<12}{'下层CCW朝向':<12}"
      f"{'c_up':>9}{'c_lo':>9}")
for s in scheme:
    axis_txt = "±X 机头/机尾" if s["axis"] == "X" else "±Y 左/右"
    print(f"{s['tag']:<6}{s['pos']:<8}{axis_txt:<14}{s['dir_up']:<12}{s['dir_lo']:<12}"
          f"{s['c_up']:>9.4f}{s['c_lo']:>9.4f}")


# ---------------------------------------------------------------------------
# 3) 偏航权限对比
# ---------------------------------------------------------------------------
def ideal_tangential(tilt_deg=5.0):
    """理想纯切向方案的每台电机系数"""
    res = []
    for tag, pos_name, r, ring in PODS:
        R = math.hypot(*r)
        # 上层 CW 切向单位向量 (+r_y, -r_x)/R  -> g = -R
        c_up = math.sin(math.radians(tilt_deg)) * (-R) - K_REACTION
        c_lo = math.sin(math.radians(tilt_deg)) * (+R) + K_REACTION
        res.append(dict(tag=tag, R=R, c_up=c_up, c_lo=c_lo))
    return res


def authority_equal_delta(coeffs):
    """等差动分配下的总偏航力矩 = Σ|c_i|"""
    return sum(abs(c["c_up"]) + abs(c["c_lo"]) for c in coeffs)


def authority_optimal(coeffs):
    """最优(比例)分配下的品质因数 = Σc_i? / max|c_i|"""
    allc = []
    for c in coeffs:
        allc += [c["c_up"], c["c_lo"]]
    return sum(x * x for x in allc) / max(abs(x) for x in allc)


ideal = ideal_tangential()
cases = {
    "理想纯切向 (机械上做不到)": ideal,
    "外侧±X + 内侧±Y (推荐)": build_scheme(AXIS_OPT),
    "全部±X (次优, 更易统一)": build_scheme(AXIS_ALLX),
    "全部±Y (不推荐)": build_scheme(AXIS_ALLY),
}

print()
print("=" * 78)
print("3) 偏航权限对比 (倾角均为 5°)")
print("=" * 78)
base_eq = authority_equal_delta(ideal)
base_op = authority_optimal(ideal)
print(f"{'方案':<30}{'等差动Σ|c|':>14}{'占比':>9}{'最优分配FoM':>14}{'占比':>9}")
for name, cf in cases.items():
    eq = authority_equal_delta(cf)
    op = authority_optimal(cf)
    print(f"{name:<30}{eq:>14.4f}{eq/base_eq*100:>8.1f}%{op:>14.4f}{op/base_op*100:>8.1f}%")


# ---------------------------------------------------------------------------
# 4) 力 / 滚转 / 俯仰 耦合校核
# ---------------------------------------------------------------------------
print()
print("=" * 78)
print("4) 寄生耦合校核 (打偏航舵时是否产生净力或滚转/俯仰力矩)")
print("=" * 78)

# 上层推力 T0-delta, 下层 T0+delta; 各电机差动量正比于 |c|
T0 = 1.0
c_all = [(s["c_up"], s["c_lo"]) for s in scheme]
cmax = max(max(abs(a), abs(b)) for a, b in c_all)

Fx = Fy = 0.0
Mx_roll = My_pitch = 0.0
Mz = 0.0
for s in scheme:
    r = s["r"]
    for which, dname, c in (("up", s["dir_up"], s["c_up"]),
                            ("lo", s["dir_lo"], s["c_lo"])):
        d = DIRS[dname]
        # 差动: 上层减、下层增, 幅度 ∝ |c|/cmax
        delta = (-1.0 if which == "up" else +1.0) * abs(c) / cmax * 0.2
        T = T0 + delta
        fh = SIN5 * T                     # 水平分力幅值
        Fx += fh * d[0]
        Fy += fh * d[1]
        Mz += c * T
        # 水平力在竖直偏置 z 处产生的滚转/俯仰 (上层 z=-0.5, 下层 z=+0.5 假定)
        z = -0.5 if which == "up" else +0.5
        Mx_roll += -z * fh * d[1]
        My_pitch += z * fh * d[0]

print(f"净水平力    Fx = {Fx:+.6e}   Fy = {Fy:+.6e}   (应为 0)")
print(f"寄生滚转力矩 Mx = {Mx_roll:+.6e}                (应为 0)")
print(f"寄生俯仰力矩 My = {My_pitch:+.6e}                (应为 0)")
print(f"有效偏航力矩 Mz = {Mz:+.6f}  (每单位悬停推力, 20% 差动)")


# ---------------------------------------------------------------------------
# 5) 新的混控 yaw 因子 (沿用代码里 ÷Y_max=3.4 的归一化惯例)
# ---------------------------------------------------------------------------
Y_MAX = 3.4
print()
print("=" * 78)
print("5) 新的混控 yaw 因子 (归一化 ÷ Y_max = 3.4)")
print("=" * 78)
outer = next(s for s in scheme if s["ring"] == "outer")
inner = next(s for s in scheme if s["ring"] == "inner")
f_out = abs(outer["c_up"]) / Y_MAX
f_in = abs(inner["c_up"]) / Y_MAX
print(f"外侧 (±X, |g|=3.4):  |yaw_fac| = (sin5°×3.4 + {K_REACTION}) / 3.4 = {f_out:.4f}")
print(f"内侧 (±Y, |g|=1.665): |yaw_fac| = (sin5°×1.665 + {K_REACTION}) / 3.4 = {f_in:.4f}")
print(f"内/外比 = {f_in/f_out:.4f}   (旧方案切向比 = {0.0592/0.0971:.4f})")
print()
print("  #define TG700_YAW_OUTER_CW    (%+.4ff)" % (-f_out))
print("  #define TG700_YAW_OUTER_CCW   (%+.4ff)" % (+f_out))
print("  #define TG700_YAW_INNER_CW    (%+.4ff)" % (-f_in))
print("  #define TG700_YAW_INNER_CCW   (%+.4ff)" % (+f_in))


# ---------------------------------------------------------------------------
# 6) 加大倾角的权衡表
# ---------------------------------------------------------------------------
print()
print("=" * 78)
print("6) 加大倾角的权衡 (推荐方案: 外侧±X + 内侧±Y)")
print("=" * 78)
ref = authority_equal_delta(build_scheme(AXIS_OPT, 5.0))
ideal5 = authority_equal_delta(ideal_tangential(5.0))
print(f"{'倾角':>6}{'Σ|c|':>10}{'vs本方案5°':>12}{'vs理想切向5°':>14}"
      f"{'升力损失':>10}{'700kg折算':>12}")
for th in (5, 8, 10, 12, 15, 18, 20):
    a = authority_equal_delta(build_scheme(AXIS_OPT, float(th)))
    loss = 1.0 - math.cos(math.radians(th))
    print(f"{th:>5}°{a:>10.4f}{a/ref:>11.2f}x{a/ideal5:>13.2f}x"
          f"{loss*100:>9.2f}%{loss*700:>10.1f}kg")

print()
print("=" * 78)
print("7) 实测标定基准 (来自 logs/2026-08-30 15-32-20.log)")
print("=" * 78)
# 该日志处于"全镜像"状态: 幅值 = |倾角项 - 反扭矩项| / |倾角项 + 反扭矩项|
mirrored_frac = authority_equal_delta(
    [dict(tag=c["tag"],
          c_up=+math.sin(math.radians(5.0)) * c["R"] - K_REACTION,
          c_lo=-math.sin(math.radians(5.0)) * c["R"] + K_REACTION)
     for c in ideal]) / ideal5
print(f"该日志状态(16个全镜像)的权限幅值 = 设计值的 {mirrored_frac*100:.1f}%")
print(f"实测: ThO≈0.21 时饱和偏航率 ≈ 10.7 °/s")
psi_design_021 = 10.7 / mirrored_frac
print(f"折算到设计权限(理想切向5°) @ThO=0.21: {psi_design_021:.1f} °/s")
for tho in (0.30, 0.44):
    v = psi_design_021 * tho / 0.21
    v_new = v * (ref / ideal5)
    print(f"  @ThO={tho:.2f}: 理想切向5° ≈ {v:5.1f} °/s ; "
          f"本方案5° ≈ {v_new:5.1f} °/s")
