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


def loop_path(circles):
    """Pitch-line path of a closed belt round circles [(x, y, r_pitch, σ)] in loop order, travelling CCW (σ +1: the belt
    wraps the circle with its teeth, the circle is inside the loop; σ −1: a back-side idler outside the loop). Returns
    (length mm, [wrap deg per circle], [(tangent point out, tangent point in of the next circle)])."""
    n = len(circles)
    th = []
    for i in range(n):
        xi, yi, ri, si = circles[i]
        xj, yj, rj, sj = circles[(i + 1) % n]
        D, a, k = math.hypot(xj - xi, yj - yi), math.atan2(yj - yi, xj - xi), sj * rj - si * ri
        t = a - math.asin(k / D)
        L_ = (-math.sin(t), math.cos(t))
        ti = (xi - si * ri * L_[0], yi - si * ri * L_[1])
        tj = (xj - sj * rj * L_[0], yj - sj * rj * L_[1])
        th.append((t, math.sqrt(D * D - k * k), ti, tj))
    length, wraps = sum(s for _, s, _, _ in th), []
    for i in range(n):
        w = (circles[i][3] * (th[i][0] - th[i - 1][0])) % (2 * math.pi)
        wraps.append(math.degrees(w))
        length += circles[i][2] * w
    return length, wraps, [(ti, tj) for _, _, ti, tj in th]


BACK = 0.97                         # HTD 5M: pitch line → belt back (3.6 thick, 2.06 tooth, pitch line 0.57 above the land)
# D26 (2026-11-27): slotted-idler take-up. Fixed centres (the motor is fixed), so the idler covers installation slack,
# preload stretch and the tolerances (all ESTIMATE until a belt is measured):
#   - preload per strand T0 = 0.25 × the peak effective pull (rnd/lift.py PRELOAD), stretch T0·L / EA, EA ≈ 50 kN for a
#     15 mm glass-cord HTD 5M (ESTIMATE; a stiffer belt only shortens the travel needed);
#   - belt length ± 0.46 mm (Gates PowerGrip / HTD centre-distance tolerance class ± 0.009 in for 10–15 in belts, ×2),
#     printed centre distance ± 0.2 mm (× 2 in length), 0.2 mm stretch-in;
#   - installation: no slack is needed. The small pulley keeps only its motor-side flange (the hub-side flange is turned
#     off), so with the idler retracted the belt slides axially, in mesh, off the pulley's teeth onto the hub toward wall B
#     (and back on): Gates' rule for fixed-centre drives (one pulley flangeless on the installation side). The big pulleys
#     (pitch 60T, yaw 40T flangeless; elbow 40T flanged both sides) and the idler's flanges keep the belt tracking.
IDLER_TAKEUP = dict(preload=0.25, EA=50e3, tol_belt=0.46, tol_centre=0.4, stretch_in=0.2)
INSTALL_SLACK = 0.0
# Idlers: 2 × 625-2RS (5 × 16 × 5, SKF 625: C 1.72 kN, C0 0.68 kN, catalogue) on the plain shank of an ISO 4762 M5 × 50
# axle bolt (aluminium Ø8 round spacers to the walls), clamped across both walls through slots; an M4 jack screw in the
# slot's end boss holds the position (the clamp alone creeps, Neon #2 F3). Kinds:
#   back   — printed PETG-CF flat roller Ø26 (r 13), 1.5 mm flanges r + 2.5, on the belt's back outside the loop (adds
#            wrap on the small pulley);
#   inside — printed HTD 5M 18T idler (pitch Ø28.6, root Ø23.4 over the Ø16 bearings), flanges, inside the loop.
# side: +1 = the stage +Y strand, −1 = −Y; f: the idler's foot along the strand from the small pulley (0 … 1).
IDLERS = {"shoulder_pitch": dict(kind="back", side=1, f=0.42), "elbow": dict(kind="inside", side=1, f=0.62),
          "upper_yaw": dict(kind="inside", side=-1, f=0.55), "elbow_d20": dict(kind="inside", side=1, f=0.6),
          "lift": dict(kind="back", side=1, f=0.5)}
IDLER_HW = dict(back=dict(r_roll=13.0, flange=2.5, width=16.0), inside=dict(teeth=18, flange=2.0, width=16.0))


def _strand(z_small, z_big, centre, side):
    """Tangent points (small, big) of the straight open-belt strand on side ±1 (stage frame: small at 0, big at +X)."""
    r1, r2 = pd(z_small) / 2, pd(z_big) / 2
    s = (r2 - r1) / centre
    c = math.sqrt(1 - s * s)
    return (-r1 * s, side * r1 * c), (centre - r2 * s, side * r2 * c)


def idler_circles(z_small, z_big, centre, idler, delta):
    """Loop circles (CCW, belts.loop_path) with the idler pushed delta mm off the straight strand (inward for a back-side
    roller, outward for an inside idler), plus the idler centre."""
    p1, p2 = _strand(z_small, z_big, centre, idler["side"])
    l = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    ux, uy = (p2[0] - p1[0]) / l, (p2[1] - p1[1]) / l
    nx, ny = -uy, ux
    if ny * idler["side"] < 0:
        nx, ny = -nx, -ny                                       # outward normal of the strand
    bx, by = p1[0] + idler["f"] * l * ux, p1[1] + idler["f"] * l * uy
    hw = IDLER_HW[idler["kind"]]
    if idler["kind"] == "back":
        r = hw["r_roll"] + BACK
        c = (bx + (r - delta) * nx, by + (r - delta) * ny, r, -1)
    else:
        r = pd(hw["teeth"]) / 2
        c = (bx - (r - delta) * nx, by - (r - delta) * ny, r, 1)
    small, big = (0.0, 0.0, pd(z_small) / 2, 1), (centre, 0.0, pd(z_big) / 2, 1)
    return ([small, big, c] if idler["side"] > 0 else [small, c, big]), (c[0], c[1]), (nx, ny)


def _solve_delta(z_small, z_big, centre, idler, target):
    lo, hi = 0.0, 30.0
    for _ in range(60):
        m = (lo + hi) / 2
        try:
            L = loop_path(idler_circles(z_small, z_big, centre, idler, m)[0])[0]
        except ValueError:                                  # the idler would overlap a pulley: past any useful travel
            L = 1e9
        lo, hi = (m, hi) if L < target else (lo, m)
    return (lo + hi) / 2


def idler_geom(joint, z_small, z_big, centre, belt_teeth, peak_pull, spec=None):
    """Slotted idler for one stage: nominal / slot-end positions (stage frame), travel, teeth in mesh, loads. The slot runs
    from the installation position (≥ INSTALL_SLACK of slack, or the strand just touched) to the long-belt end."""
    sp = dict(spec or IDLERS[joint])
    hw = IDLER_HW[sp["kind"]]
    tk = IDLER_TAKEUP
    L_belt = belt_teeth * PITCH
    T0 = tk["preload"] * peak_pull
    pre = T0 * L_belt / tk["EA"]
    L2 = pitch_length(z_small, z_big, centre)
    d_lo = _solve_delta(z_small, z_big, centre, sp, L_belt - INSTALL_SLACK) if L2 < L_belt - INSTALL_SLACK else 0.0
    d_nom = _solve_delta(z_small, z_big, centre, sp, L_belt + pre)
    d_hi = _solve_delta(z_small, z_big, centre, sp, L_belt + pre + tk["tol_belt"] + tk["tol_centre"] + tk["stretch_in"])
    d_short = _solve_delta(z_small, z_big, centre, sp, L_belt + pre - tk["tol_belt"] - tk["tol_centre"])
    circ, cen, n = idler_circles(z_small, z_big, centre, sp, d_nom)
    _, w, _ = loop_path(circ)
    i_id = 2 if sp["side"] > 0 else 1
    pos = lambda d: idler_circles(z_small, z_big, centre, sp, d)[1]
    tims = [z_small * loop_path(idler_circles(z_small, z_big, centre, sp, d)[0])[1][0] / 360 for d in (d_short, d_nom, d_hi)]
    T1 = peak_pull + T0                                       # tight strand at the peak (conservative: slack side keeps T0)
    F_id = 2 * T1 * math.sin(math.radians(w[i_id]) / 2)      # idler load if its strand is the tight one
    r_out = (hw["r_roll"] if sp["kind"] == "back" else pd(hw["teeth"]) / 2 - 0.5715) + hw["flange"]
    out = dict(sp, **hw, joint=joint, belt=L_belt, L2=L2, slack0=L_belt - L2, pre=pre, T0=T0, d_lo=d_lo, d_nom=d_nom, d_hi=d_hi,
               d_short=d_short, centre=cen, normal=n, slot=(pos(d_lo), pos(d_hi)), travel=d_hi - d_lo, circles=circ,
               wrap_small=w[0], wrap_idler=w[i_id], tim=tims[1], tim_min=min(tims), F_idler=F_id, T1=T1, r_out=r_out,
               install_ok=L2 < L_belt - INSTALL_SLACK + 1e-6)
    return out


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
    L += ["", "- D20 motor-side hub under its 18 N·m limit: 4 × M3 friction is short (⚠️/❌ above); its output pattern is still ESTIMATE. "
          "If the GIM8108-8 drawing has dowel holes, 2 × ISO 8734 Ø3 dowels go through the hub into them; otherwise the hub face is "
          "bonded to the output with Loctite 638 (ESTIMATE ≈ 10 MPa on ≈ 1.7e3 mm², far above the limit; heat to remove).",
          f"- upper_yaw: the centre for the stock 315-5M is {centre_for(20, 40, 63):.2f} mm; roll.YAW_C (roll track) is still 80.00, "
          "so the eccentric starts 93 % used. No stock 5M length lands at 80.00 (310-5M → 78.38, 315-5M → 80.93); YAW_C has to move."]
    from .parts.cartridge_m import FLANGE_LOAD, flange_rows
    fl = FLANGE_LOAD
    L += ["", "## Cartridge flange ears: tipping and bolt load (Neon #2 finding 2)", "",
          f"Applied moment at the actuator peak = cartridge mass × {fl['g']:.0f} g × motor mid-length + Oldham side load "
          f"(μ {fl['mu_oldham']} · T / (D/3)) × the tenon height; tipping when it beats the ears' preload × their distance from the "
          f"flange's edge line in the worst direction; preload {fl['preload']:.0f} N per M4, and {fl['retained']:.0%} of it after "
          "printed-joint creep (ESTIMATE). Friction torque μ 0.3 · n · F_p · r. Rule: ≥ 3 ears with no gap > 120°, or ≥ 4 with no gap > 180°.", "",
          "| Joint | Motor | Ears (largest gap) | Applied | Tipping at full / 30 % preload | Friction at 30 % vs peak | Max bolt load |",
          "|---|---|---|---|---|---|---|"]
    for j, mot, b, a in flange_rows():
        for tag, r in (("C3", b), ("now", a)):
            ok = r[0]["rule"] and r[1]["sf_tip"] >= 2 and r[1]["sf_T"] >= 1
            L.append(f"| {j} ({tag}) | {mot} | {r[0]['n']} ({r[0]['gap']:.0f}°) {'✅' if r[0]['rule'] else '❌'} | {r[0]['M_app']:.1f} N·m | "
                     f"{r[0]['M_open']:.1f} / {r[1]['M_open']:.1f} N·m → {r[0]['sf_tip']:.1f}× / {r[1]['sf_tip']:.1f}× | "
                     f"{r[1]['T_fric']:.0f} vs {r[0]['T']:.0f} N·m → {r[1]['sf_T']:.2f}× | {r[0]['F_max']:.0f} N ({r[0]['sf_bolt']:.1f}× proof) {'✅' if ok else '❌'} |")
    L += ["", "- pitch and lift 'now' are the pitch4 / lift4 patterns (cartridge_m.EAR_SITES); they need shoulder.py / upper.py geometry "
          "(outer-wall holes, wall-A bosses + cover cut) before those modules switch EAR_SITE.",
          "- D20 at 30 % preload: friction 15 N·m < 22 N·m stall, so the ears go into bearing (≈ 98 N per M4, ≈ 4 MPa on the 6 mm ear) "
          "after a slip; at the 18 N·m flash limit 0.8×. Full preload: 50 N·m (2.3×)."]
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
            ("Oldham disc D63 acetal at the 18 N·m flash torque limit", "GIM8108-8 (D20)", 18.0, disc(63)),
            # Neon #2 finding 2 (2026-11-27): the motor-side half, 4 × M3 into the motor output (PCD 34 / 30, ESTIMATE), friction only
            # (the screws in Ø3.4 holes are the shear backup, but a slip is backlash). Printed ASA: the Ø5.5 head annulus takes ≈ 0.4 kN
            # (≈ 27 MPa), μ 0.3; 6061 (cartridge_m.MOTOR_HUB): 1.0 kN (8.8) / 1.3 kN (10.9), μ 0.2 metal on metal (ESTIMATE).
            ("Oldham motor-side tenon adapter, printed ASA, 4 × M3 at 0.4 kN (D24 v2)", "GIM6010-8", 11.0, 0.3 * 4 * 0.4 * 17.0),
            ("Oldham motor-side hub, 6061, 4 × M3 8.8 at 1.0 kN (Neon #2)", "GIM6010-8", 11.0, 0.2 * 4 * 1.0 * 17.0),
            ("Oldham motor-side hub, 6061, 4 × M3 10.9 at 1.3 kN, at the 18 N·m flash limit", "GIM8108-8 (D20)", 18.0, 0.2 * 4 * 1.3 * 15.0)]
    return [(j, m, t, c, c / t) for j, m, t, c in rows]


def score(d: Design = BASELINE):
    ss = [stage(*x, d=d) for x in STAGES]
    return min(s["sf_sus"] for s in ss), min(s["sf_ult"] for s in ss)


if __name__ == "__main__":
    print(report())
