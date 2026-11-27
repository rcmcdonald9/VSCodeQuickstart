"""Belt-stage sizing for the arm's in-bone reductions (HTD 5M, printed pulleys).

Each stage: small pulley on its own bone-supported hub (driven by a cartridge through KX74), big pulley on the joint hub.
Checks: belt pull at actuator peak vs allowable tension, teeth in mesh, pulley sizes vs bone envelope, belt length.
"""
from __future__ import annotations

import math

from .params import BASELINE, COMP, Design, act

B = COMP["timing_belt_htd5m"]
PITCH = 5.0
MIN_TEETH_IN_MESH = 6
ECC_E = 1.0                         # eccentric-bushing adjuster: ±1.0 mm of centre distance (rnd/parts/belt_stage.py ECC_V2)


def pd(teeth):                      # pitch diameter, mm
    return teeth * PITCH / math.pi


def pitch_length(z_small, z_big, centre):
    """Exact open-belt pitch length (mm): 2·C·cos φ + π(D + d)/2 + φ(D − d), sin φ = (D − d)/(2C)."""
    d, D = pd(z_small), pd(z_big)
    phi = math.asin((D - d) / (2 * centre))
    return 2 * centre * math.cos(phi) + math.pi * (D + d) / 2 + phi * (D - d)


def centre_for(z_small, z_big, belt_teeth):
    """Centre distance (mm) at which a stock belt of belt_teeth × 5 mm pitch length fits with zero slack (bisection on
    pitch_length). Fable review M5 (2026-11-27): the CAD centres are set to this, so the ±1 mm eccentric sits mid-range."""
    lo, hi = (pd(z_big) - pd(z_small)) / 2 + 1.0, 1000.0
    for _ in range(80):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if pitch_length(z_small, z_big, m) < belt_teeth * PITCH else (lo, m)
    return (lo + hi) / 2


# Stock belts (teeth = pitch length / 5) per stage. Gates/Optibelt-class HTD 5M stock lengths (ESTIMATE until ordered).
STOCK_TEETH = {"shoulder_pitch": 71, "shoulder_roll": 95, "upper_yaw": 63, "elbow": 59, "elbow_d20": 76}


def cad_centres():
    """The belt centres the CAD actually uses (mm): pitch shoulder.BELT_C, roll / yaw roll.ROLL_C / YAW_C, elbow arm.BELT_C."""
    from .parts import arm, roll, shoulder
    return {"shoulder_pitch": shoulder.BELT_C, "shoulder_roll": roll.ROLL_C, "upper_yaw": roll.YAW_C, "elbow": arm.BELT_C}


def adjuster(d: Design = BASELINE):
    """Per stage: (stock belt mm, exact centre for it, CAD centre, share of the ±1 mm eccentric used at zero tension)."""
    cad = cad_centres()
    out = []
    for joint, ratio, z, _, _ in STAGES:
        zb = round(z * ratio)
        key = "elbow_d20" if joint == "elbow" and zb == 60 else joint
        n = STOCK_TEETH[key]
        c = centre_for(z, zb, n)
        out.append(dict(joint=joint, belt=n * PITCH, exact=c, cad=cad[joint], used=abs(c - cad[joint]) / ECC_E))
    return out


def sustained_motor_torque(joint, ratio, d: Design = BASELINE):
    """95th-percentile static joint torque over the workspace (1 kg, rnd/spring.py) → motor side."""
    from .spring import workspace_margins
    w = workspace_margins(d)
    return w[joint]["p95"] / ratio / 0.95 if joint in w else act(d.arm.joints[joint][0])["rated"]


def stage(joint, ratio, z_small, centre_mm, width_mm, d: Design = BASELINE):
    name, _ = d.arm.joints[joint]
    a = act(name)
    z_big = round(z_small * ratio)
    r_s = pd(z_small) / 2
    pull_peak = a["peak"] * 1000 / r_s             # N, tight-side pull at actuator peak
    pull_rated = a["rated"] * 1000 / r_s
    allow = B["allowable_tension_n_per_10mm"] * width_mm / 10
    # wrap angle on the small pulley and teeth in mesh
    D, dd = pd(z_big), pd(z_small)
    wrap = math.pi - 2 * math.asin((D - dd) / (2 * centre_mm))
    tim = z_small * wrap / (2 * math.pi)
    length = pitch_length(z_small, z_big, centre_mm)
    work = B["working_tension_n_15mm"] * width_mm / 15
    ult = B["ultimate_tension_n_per_10mm"] * width_mm / 10
    pull_sus = sustained_motor_torque(joint, ratio, d) * 1000 / r_s
    return dict(joint=joint, act=name, ratio_exact=z_big / z_small, z=(z_small, z_big), pd=(dd, D), width=width_mm,
                pull_rated=pull_rated, pull_peak=pull_peak, allow=allow, sf_peak=allow / pull_peak,
                pull_sus=pull_sus, work=work, sf_sus=work / pull_sus, sf_ult=ult / pull_peak,
                tim=tim, centre=centre_mm, length=length, teeth_belt=length / PITCH, outer_big=D + 2 * 3.6)


# centres = the CAD's (cad_centres(); the elbow from params.elbow_belt_c). Fable M5 (2026-11-27): the elbow is set to the
# exact centre for its stock belt (70.70 for 295-5M, D20 83.89 for 380-5M); pitch 70 → 70.15 and yaw 80 → 80.93 belong to
# shoulder.py / roll.py (adjuster() reports what each CAD centre leaves of the ±1 mm eccentric).
STAGES = [("shoulder_pitch", 3.0, 20, 70, 15), ("shoulder_roll", 80 / 18, 18, 103.3, 15), ("upper_yaw", 2.0, 20, 80, 15),
          ("elbow", BASELINE.arm.elbow_teeth / 20, 20, round(BASELINE.arm.elbow_belt_c * 1000, 2), 15)]   # D20 switch: 20T:40T or 20T:60T
from .params import ROLL_DRIVE as _RD                     # D24 (owner, 2026-11-27): the inline GIM8108-36 roll has no belt
if _RD == "inline":
    STAGES = [s for s in STAGES if s[0] != "shoulder_roll"]


def report(d: Design = BASELINE):
    L = ["# Belt reductions (auto-generated by `python -m rnd.run_all`)", "",
         "HTD 5M, 15 mm, printed pulleys, small pulley on its own bearing hub. **Loop 13 re-rating:** the 650 N/10 mm vendor figure used before "
         f"is not a working rating. The checks now are: **sustained pull** (95th-percentile workspace holding torque, 1 kg) vs the **working tension "
         f"{B['working_tension_n_15mm']} N** (Mädler neoprene 5M-15, the conservative published figure), and **peak pull** (actuator peak, short transients; "
         f"the KX seat slips above it) vs **ultimate {B['ultimate_tension_n_per_10mm'] * 1.5:.0f} N / 3**. Precedent: Berkeley Blue's belt reductions.", "",
         "| Joint | Teeth | Pitch Ø small / big | Sustained pull vs working | Peak pull vs ultimate/3 | Teeth in mesh | Centre / belt length |",
         "|---|---|---|---|---|---|---|"]
    for s in (stage(*x, d=d) for x in STAGES):
        f1 = "✅" if s["sf_sus"] >= 1.2 else ("⚠️" if s["sf_sus"] >= 1.0 else "❌")
        f2 = "✅" if s["sf_ult"] >= 3 else ("⚠️" if s["sf_ult"] >= 2 else "❌")
        g = "✅" if s["tim"] >= MIN_TEETH_IN_MESH else "❌"
        L.append(f"| {s['joint']} | {s['z'][0]}:{s['z'][1]} ({s['ratio_exact']:.2f}:1) | {s['pd'][0]:.1f} / {s['pd'][1]:.1f} | "
                 f"{s['pull_sus']:.0f} / {s['work']:.0f} N → {s['sf_sus']:.1f}× {f1} | {s['pull_peak']:.0f} N → {s['sf_ult']:.1f}× ultimate {f2} | {s['tim']:.1f} {g} | "
                 f"{s['centre']} / {s['length']:.0f} mm |")
    L += ["", "## Centre distances vs the stock belts (Fable M5)", "",
          f"Exact pitch length 2C·cos φ + π(D+d)/2 + φ(D−d); the eccentric bushings give ±{ECC_E:.1f} mm. 'Used' = the share of that "
          "range the CAD centre needs just to fit the belt (0 % = mid-range, the whole range left for tension and tolerances).", "",
          "| Joint | Stock belt | Exact centre | CAD centre | Adjuster used |", "|---|---|---|---|---|"]
    for a in adjuster(d):
        f = "✅" if a["used"] <= 0.25 else ("⚠️" if a["used"] <= 0.5 else "❌")
        L.append(f"| {a['joint']} | {a['belt']:.0f}-5M | {a['exact']:.2f} | {a['cad']:.2f} | {a['used']:.0%} {f} |")
    L += ["", "## Shaft joints at the motor peak (Fable M5/M6)", "", "| Joint | Motor | Torque | Capacity | Margin |", "|---|---|---|---|---|"]
    for r in shaft_joints():
        f = "✅" if r[4] >= 1.25 else ("⚠️" if r[4] >= 1.0 else "❌")
        L.append(f"| {r[0]} | {r[1]} | {r[2]:.0f} N·m | {r[3]:.1f} N·m | {r[4]:.2f}× {f} |")
    L += ["", "- Belt lengths round to stock sizes; eccentric bushings set the tension (loop 10).",
          "- If coupon tests show tooth jump or creep: switch to PU steel-cord HTD 5M (1268 N working, Mädler) at the same geometry, or to 25 mm belts.",
          "- **Rejected (loop 3):** remote elbow motor on a long belt."]
    return "\n".join(L)


# Shaft-joint capacities (Fable M5/M6, 2026-11-27). All ESTIMATE (hand calcs, catalogue-class figures to confirm):
#  - set screws on a flat (D13 v1): 2 × M4 cup point, ≈ 1.0 kN holding each (catalogue class), on r 6 mm;
#  - DIN 6885 A 4 × 4 key in the 12 mm bore: hub-side bearing p = 2T / (d · t2 · L), t2 = 1.8 mm; allowable 90 MPa in an
#    aluminium hub, 150 MPa in a steel hub (ESTIMATE, ≈ 0.35 R_m);
#  - clamp hub on a plain Ø12: μ 0.15 × 2 × F × r, F = 4 kN (M4) / 6.5 kN (M5) clamp-screw preload;
#  - Oldham acetal disc: D40 ≈ 9.5 N·m (catalogue class, Fable M5); larger sizes scaled by R² (tongue bearing at equal stress).
KEY = dict(t2=1.8, p_al=90.0, p_steel=150.0)


def _key_cap(L, p):
    return p * 12.0 * KEY["t2"] * L / 2 / 1000      # N·m: T = p · d · t2 · L / 2


def shaft_joints():
    """(joint, motor, torque N·m, capacity N·m, margin) rows for the small-stage shaft joints at the motor peak."""
    disc = lambda od: 9.5 * (od / 40.0) ** 2
    clamp = lambda F: 0.15 * 2 * F * 6.0 / 1000
    rows = [("20T pulley, 2 set screws on a flat (D13 v1)", "GIM6010-8", 11.0, 2 * 1.0 * 6.0),
            ("20T aluminium pulley, 4 × 4 key L 16 + set screw on the key", "GIM6010-8", 11.0, _key_cap(16, KEY["p_al"])),
            ("Oldham clamp hub, plain Ø12 (D24 v1)", "GIM6010-8", 11.0, clamp(4000)),
            ("Oldham clamp hub + 4 × 4 key L 11", "GIM6010-8", 11.0, clamp(4000) + _key_cap(11, KEY["p_al"])),
            ("Oldham disc D40 acetal (D24 v1)", "GIM6010-8", 11.0, disc(40)),
            ("Oldham disc D50 acetal", "GIM6010-8", 11.0, disc(50)),
            ("20T steel pulley, 4 × 4 key L 20", "GIM8108-8 (D20)", 22.0, _key_cap(20, KEY["p_steel"])),
            ("Oldham D63 clamp hub (M5) + 4 × 4 key L 16", "GIM8108-8 (D20)", 22.0, clamp(6500) + _key_cap(16, KEY["p_al"])),
            ("Oldham disc D50 acetal", "GIM8108-8 (D20)", 22.0, disc(50)),
            ("Oldham disc D63 acetal", "GIM8108-8 (D20)", 22.0, disc(63)),
            ("Oldham disc D63 acetal at the 18 N·m flash torque limit", "GIM8108-8 (D20)", 18.0, disc(63))]
    return [(j, m, t, c, c / t) for j, m, t, c in rows]


def score(d: Design = BASELINE):
    ss = [stage(*x, d=d) for x in STAGES]
    return min(s["sf_sus"] for s in ss), min(s["sf_ult"] for s in ss)


if __name__ == "__main__":
    print(report())
