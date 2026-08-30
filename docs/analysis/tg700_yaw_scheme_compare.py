# -*- coding: utf-8 -*-
"""
TG700: compare the yaw authority of two coaxial rotation schemes.

Scheme A (current):  every pair is counter-rotating.
                     upper = CW, lower = CCW.
                     Yaw comes from upper-vs-lower differential thrust,
                     so upper/lower must be tilted in OPPOSITE tangential senses.

Scheme B (proposed): every pair co-rotates, checkerboard between pairs.
                     P1 CW, P2 CCW, P3 CW, P4 CCW, P5 CCW, P6 CW, P7 CCW, P8 CW.
                     Yaw comes from CW-group vs CCW-group differential thrust,
                     so the two groups must be tilted in OPPOSITE tangential senses
                     (i.e. both motors of a pair tilt the SAME way).

Yaw sensitivity of motor i:   dM/dT_i = s_i * sin(tilt) * L_i  +  r_i * k
  s_i = tangential tilt sign, r_i = rotation sign, k = Q/T of the propeller.
For maximum authority the installer must make s_i and r_i agree in sign, so that
the tilt force and the propeller reaction torque add instead of cancel.
"""
import math

# ---------------- airframe geometry (measured) ----------------
Y_OUTER, Y_INNER, X_ARM = 3.4, 1.6, 1.665
L_OUTER = math.hypot(X_ARM, Y_OUTER)
L_INNER = math.hypot(X_ARM, Y_INNER)

TILT_DEG = 5.0
SIN_TILT = math.sin(math.radians(TILT_DEG))

MASS_KG = 700.0
N_MOTORS = 16
THST_HOVER = 0.44                        # calibrated from the 2026-08-24 log
T_HOVER = MASS_KG * 9.81 / N_MOTORS      # N per motor
T_MAX = T_HOVER / THST_HOVER             # N per motor at full thrust

# ---------------- pair table ----------------
# name, x, y, ring, L, schemeA(upper_rot, lower_rot), schemeB(both_rot)
PAIRS = [
    ("P1 front-right-outer", +X_ARM, +Y_OUTER, "outer", L_OUTER, ("CW", "CCW"), "CW"),
    ("P2 front-right-inner", +X_ARM, +Y_INNER, "inner", L_INNER, ("CW", "CCW"), "CCW"),
    ("P3 front-left-inner",  +X_ARM, -Y_INNER, "inner", L_INNER, ("CW", "CCW"), "CW"),
    ("P4 front-left-outer",  +X_ARM, -Y_OUTER, "outer", L_OUTER, ("CW", "CCW"), "CCW"),
    ("P5 rear-right-outer",  -X_ARM, +Y_OUTER, "outer", L_OUTER, ("CW", "CCW"), "CCW"),
    ("P6 rear-right-inner",  -X_ARM, +Y_INNER, "inner", L_INNER, ("CW", "CCW"), "CW"),
    ("P7 rear-left-inner",   -X_ARM, -Y_INNER, "inner", L_INNER, ("CW", "CCW"), "CCW"),
    ("P8 rear-left-outer",   -X_ARM, -Y_OUTER, "outer", L_OUTER, ("CW", "CCW"), "CW"),
]

ROT_SIGN = {"CCW": +1.0, "CW": -1.0}     # +1 = reaction torque gives +yaw (nose right)


def build(scheme, k):
    """Return list of motors: (label, x, y, L, rot_sign, tilt_sign, dM/dT)."""
    out = []
    for name, x, y, ring, L, (ru, rl), rb in PAIRS:
        rots = (ru, rl) if scheme == "A" else (rb, rb)
        for pos, rot in zip(("upper", "lower"), rots):
            r = ROT_SIGN[rot]
            # installer aligns tilt with rotation so the two effects add
            s = r
            dMdT = s * SIN_TILT * L + r * k
            out.append(dict(label="%s %s" % (name, pos), x=x, y=y, L=L,
                            ring=ring, rot=rot, r=r, s=s, dMdT=dMdT))
    return out


def roll_fac(y):
    return -y / Y_OUTER          # ArduPilot convention: right side negative


def pitch_fac(x):
    return x / X_ARM


def analyse(scheme, k, verbose=False):
    m = build(scheme, k)
    # mixer yaw factor is proportional to dM/dT, normalised to max magnitude 1
    peak = max(abs(x["dMdT"]) for x in m)
    for x in m:
        x["yf"] = x["dMdT"] / peak

    sum_yf = sum(x["yf"] for x in m)
    sum_yf_roll = sum(x["yf"] * roll_fac(x["y"]) for x in m)
    sum_yf_pitch = sum(x["yf"] * pitch_fac(x["x"]) for x in m)

    # Total yaw moment when every motor is driven to the edge of an equal
    # thrust differential dT (the honest "authority" metric: same differential
    # budget per motor for both schemes).
    dT = 0.20 * T_MAX                     # 20 % of full thrust per motor
    M_total = sum(abs(x["dMdT"]) * dT for x in m)

    if verbose:
        print("  %-28s %-5s %-6s %6s %8s %8s" % ("motor", "rot", "tilt", "L(m)", "dM/dT", "yaw_fac"))
        for x in m:
            print("  %-28s %-5s %-6s %6.3f %8.4f %8.4f"
                  % (x["label"], x["rot"], "%+d" % x["s"], x["L"], x["dMdT"], x["yf"]))
    return dict(sum_yf=sum_yf, sum_yf_roll=sum_yf_roll, sum_yf_pitch=sum_yf_pitch,
                M_total=M_total, motors=m)


print("=" * 78)
print("TG700 yaw scheme comparison")
print("=" * 78)
print("geometry : L_outer=%.3f m  L_inner=%.3f m  tilt=%.1f deg (sin=%.5f)"
      % (L_OUTER, L_INNER, TILT_DEG, SIN_TILT))
print("thrust   : hover=%.0f N/motor  max=%.0f N/motor (THST_HOVER=%.2f)"
      % (T_HOVER, T_MAX, THST_HOVER))

# ---------------- k sensitivity: is the conclusion robust? ----------------
print("\n" + "-" * 78)
print("1. Yaw authority vs propeller reaction coefficient k = Q/T")
print("-" * 78)
print("   k(m)   |  scheme A (N.m)  |  scheme B (N.m)  |  B/A    | decoupled?")
for k in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.15):
    a = analyse("A", k)
    b = analyse("B", k)
    ok_a = max(abs(a["sum_yf"]), abs(a["sum_yf_roll"]), abs(a["sum_yf_pitch"])) < 1e-9
    ok_b = max(abs(b["sum_yf"]), abs(b["sum_yf_roll"]), abs(b["sum_yf_pitch"])) < 1e-9
    print("   %5.3f  |   %12.1f   |   %12.1f   | %6.4f  | A:%s B:%s"
          % (k, a["M_total"], b["M_total"], b["M_total"] / a["M_total"],
             "yes" if ok_a else "NO", "yes" if ok_b else "NO"))

# ---------------- detailed dump at a representative k ----------------
K_REP = 0.08
print("\n" + "-" * 78)
print("2. Per-motor detail at k = %.2f m" % K_REP)
print("-" * 78)
print(" Scheme A (current: counter-rotating pairs, upper/lower tilted oppositely)")
ra = analyse("A", K_REP, verbose=True)
print("\n Scheme B (proposed: co-rotating pairs, checkerboard, pair-groups tilted oppositely)")
rb = analyse("B", K_REP, verbose=True)

print("\n  decoupling residuals (must all be 0):")
for nm, r in (("A", ra), ("B", rb)):
    print("    scheme %s : sum(yf)=%+.2e  sum(yf*roll)=%+.2e  sum(yf*pitch)=%+.2e"
          % (nm, r["sum_yf"], r["sum_yf_roll"], r["sum_yf_pitch"]))

# ---------------- group composition, the reason they tie ----------------
print("\n" + "-" * 78)
print("3. Why they tie: each scheme differentiates 8 motors against 8 motors")
print("-" * 78)
for nm in ("A", "B"):
    m = build(nm, K_REP)
    pos = [x for x in m if x["s"] > 0]
    neg = [x for x in m if x["s"] < 0]
    for grp, lbl in ((pos, "+yaw group"), (neg, "-yaw group")):
        n_out = sum(1 for x in grp if x["ring"] == "outer")
        n_in = sum(1 for x in grp if x["ring"] == "inner")
        print("  scheme %s  %-11s : %2d motors  (%d outer @L=%.3f, %d inner @L=%.3f)"
              % (nm, lbl, len(grp), n_out, L_OUTER, n_in, L_INNER))
print("  -> identical ring composition, so identical sum of |dM/dT|. The tie is exact,")
print("     independent of k, tilt angle and arm length.")

# ---------------- what actually helps: bigger tilt ----------------
print("\n" + "-" * 78)
print("4. What DOES change authority: the tangential tilt angle")
print("-" * 78)
I_ZZ_LO, I_ZZ_HI = 2600.0, 4200.0
print("  tilt |  sin   | authority x |  M_yaw(N.m) | alpha (deg/s^2) | lift loss")
base = None
for deg in (5, 8, 10, 12, 15, 18, 20, 25):
    s = math.sin(math.radians(deg))
    M = 0.0
    dT = 0.20 * T_MAX
    for name, x, y, ring, L, _, _ in PAIRS:
        M += 2 * (s * L + K_REP) * dT
    if base is None:
        base = M
    a_lo = math.degrees(M / I_ZZ_HI)
    a_hi = math.degrees(M / I_ZZ_LO)
    loss = 1.0 - math.cos(math.radians(deg)) * math.cos(math.radians(2.0))
    print("  %3d  | %.4f | %8.2f    | %10.0f  |  %5.1f - %5.1f   | %5.2f%%"
          % (deg, s, M / base, M, a_lo, a_hi, loss * 100))

print("\n" + "=" * 78)
print("CONCLUSION")
print("=" * 78)
print("""
Scheme B has EXACTLY the same yaw authority as scheme A (ratio 1.0000 for every
value of k).  Both are perfectly decoupled from roll / pitch / throttle.

The reason is structural, not numerical: yaw authority is the sum of |dM/dT|
over the motors you differentiate, and |dM/dT| = sin(tilt)*L + k depends only on
where a motor sits, never on which way it spins.  Both schemes split the 16
motors into two groups of 8 with an identical 4-outer / 4-inner composition, so
the sums are identical term by term.

Changing rotation direction therefore cannot improve yaw. The binding constraint
is sin(5 deg) = 0.0872.  Only a larger tangential tilt (or correcting a tilt
that is currently mis-installed) moves the number.
""")
