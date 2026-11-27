"""Hand-deflection compliance chain (queue item 3 + review A8, loop 40).  python -m rnd.stiffness

The scorecard's hand sag (rnd/bones.py) only bends the carbon tubes. The joint encoders sit on the joint outputs, so
they see and correct belt stretch and KX twist, but not anything that bends or tilts **after** a joint output. This
chain adds those terms for the worst static case: 1 kg at the grasp point, arm straight out horizontally.

1. **Printed structure (FEA).** The four parts that carry the whole arm moment are re-solved with quadratic (P2)
   tetrahedra, which the review asked for (P1 tets are too stiff in bending), and with P1 on the same mesh for the
   ratio. Each part's loaded bearing seats are fitted with a rigid motion (translation t, rotation ω); the hand moves
   by t + ω × r (r = seat → hand). FEA loads are the ×1.5 design cases; the chain uses them ÷ 1.5.
2. **Bearing play.** A 61808 pair with C0 radial clearance (≈ 10 µm each, ESTIMATE) tilts by 2·c/span. Shimmed
   (light axial preload) pairs remove most of it: the chain uses 30 % of the free play for shimmed pairs.
3. **Carbon tubes** (rnd/bones.py), the CF segments only: bones.py also bends a printed "roll link" box (pitch → yaw)
   and a printed "elbow housing" box, which are the same parts step 1 already solves by FEA (counted twice until
   2026-11-27: 0.88 of its 1.85 mm).

Static case for the inline roll (the default drive; model correction 2026-11-27): the roll link and the yaw block get the
1 kg straight-out wrench itself instead of their ×1.5 strength cases ÷ 1.5. Pitch 90° with roll 0 puts the roll axis
along gravity, so the roll torque is 0 (the strength case also carries the drive's 35 N·m rated limit, ÷ 1.5 = 23 N·m,
at the same time), and the yaw torque is 0, so the yaw belt carries only its preload, which is there at zero payload
too (the strength case has the 691 N belt peak). The arm moment comes from the twin's point masses
(rnd/arm_sizing.point_masses): everything on the roll output about the roll axis for the roll link, everything hung
from the yaw shaft about the yaw seats' mid-level for the yaw block (the strength cases use 19 N·m ÷ 1.5 = 12.7 N·m for
both, which under-loads the roll link). The arm's ≈ 32 N shear at the yaw seats (< 6 % of the couple) is left out. The
bracket and the elbow housing keep their strength cases ÷ 1.5 (belt peak, a 3 kg forearm load: conservative).

The total is the static error a calibrated gravity model can largely remove (it is repeatable); the wrist camera closes
the loop for grasping in any case. Target R2b: uncompensated ≤ 6 mm, after the gravity model (assumed to remove 80 %) ≤ 2 mm.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
H = 4.5                     # mesh size for the P1/P2 pair (P2 at h 3 would be ≈ 1.5 M dof)
PLAY_MM, SHIM = 0.010, 0.3  # radial clearance per bearing (ESTIMATE), fraction left after shimming
COMP = 0.8                  # fraction removed by a calibrated gravity-deflection model (ESTIMATE until M1 measures it)


def static_moments(d=None):
    """1 kg straight out (pitch 90°, roll 0), static: (arm moment at the roll link, at the yaw seats) in N·m, from the
    twin's point masses. Roll link: everything on the roll output (not the roll body or the static roll motor) about the
    roll axis; yaw block: everything hung from the yaw shaft (not the yaw stage / actuator on the block) about the yaw
    seats' mid-level."""
    from .arm_sizing import point_masses
    from .params import BASELINE, G
    from .parts import roll as rl
    pts, _ = point_masses(d or BASELINE)
    out = [(n, m, x) for n, m, x in pts if n not in ("roll stage", "shoulder_roll")]
    z_seats = -(rl.P1[0] + rl.P1[1] + rl.P2[0] + rl.P2[1]) / 4000
    hung = [(n, m, x) for n, m, x in out if n not in ("yaw stage", "upper_yaw")]
    return G * sum(m * x for _, m, x in out), G * sum(m * (x - z_seats) for _, m, x in hung)


def static_cases():
    """fea.cases() with the inline roll's roll link and yaw block on the static 1 kg straight-out wrench (× 1.5, so that
    run() keeps one ÷ 1.5 for every part)."""
    from . import fea
    from .parts import roll as rl
    C = fea.cases()
    if not rl.INLINE:
        return C
    m_roll, m_yaw = static_moments()
    sub = {"Roll link": fea.inline_roll_cases(M=1.5 * m_roll, T=0.0, belt=0.0, weight=0.0)[0],
           "Clevis + yaw block": fea.inline_roll_cases(M=1.5 * m_yaw, T=0.0, belt=0.0, weight=0.0)[1]}
    return [next((v for k, v in sub.items() if c[0].startswith(k)), c) for c in C]


def tube_term(d=None):
    """Hand deflection from the carbon tubes alone: rnd/bones.py's beam (same point masses, static), with its printed
    segments (the roll-link and elbow-housing boxes, solved by the FEA here) taken as rigid."""
    from . import bones
    from .arm_sizing import point_masses
    from .params import BASELINE, G
    d = d or BASELINE
    pts, _ = point_masses(d)
    xs = np.linspace(0, d.arm.l_upper + d.arm.l_fore, 2000)
    M = np.array([sum(m * G * (x - xp) for _, m, x in pts if x > xp) for xp in xs])
    kappa = np.zeros_like(xs)
    for _name, x0, x1, I, _c, E in bones.segments(d.arm.bone_wall_mm, d, d.arm.cf_tubes):
        if E == bones.E_CF:
            sel = (xs >= x0) & (xs <= x1)
            kappa[sel] = M[sel] / (E * I * 1e-6)
    slope = np.concatenate([[0], np.cumsum((kappa[1:] + kappa[:-1]) / 2 * np.diff(xs))])
    defl = np.concatenate([[0], np.cumsum((slope[1:] + slope[:-1]) / 2 * np.diff(xs))])
    return (defl[-1] + slope[-1] * d.arm.l_hand) * 1000


def run():
    from . import fea
    from .params import BASELINE as d
    a = d.arm
    reach = a.l_upper + a.l_fore + a.l_hand                       # pitch axis → grasp, arm straight out (m)
    want = {"Shoulder pitch bracket": reach * 1000, "Roll link": reach * 1000, "Clevis + yaw block": reach * 1000,
            "Upper bone / elbow housing": (a.l_fore + a.l_hand) * 1000}
    rows = []
    for name, part, h, fixed, loads in static_cases():
        key = next((k for k in want if name.startswith(k)), None)
        if key is None:
            continue
        mesh = fea.mesh_part(part, H)
        r1 = fea.solve2(mesh, fixed, loads, order=1)
        r2 = fea.solve2(mesh, fixed, loads, order=2)
        L = want[key]
        # rotation of the loaded seats (mean over regions), ÷1.5 to the static case; hand moves |ω|·L + |t|
        w2 = np.mean([np.linalg.norm(g["w"]) for g in r2["regions"]]) / 1.5
        t2 = np.mean([np.linalg.norm(g["t"]) for g in r2["regions"]]) / 1.5
        w1 = np.mean([np.linalg.norm(g["w"]) for g in r1["regions"]]) / 1.5
        rows.append(dict(part=key, case=name, n=r2["n"], rot_p1=w1, rot_p2=w2, ratio=w2 / max(w1, 1e-12), t=t2, L=L, hand=w2 * L + t2,
                         vmax_p2=r2["vmax"], vm99_p2=r2["vm99"], vm99_p1=r1["vm99"]))
        print(rows[-1], flush=True)
    return rows


def bearing_terms():
    from .params import BASELINE as d
    from .parts import roll, shoulder
    a = d.arm
    reach = (a.l_upper + a.l_fore + a.l_hand) * 1000
    spans = {"pitch (61808 pair)": abs(shoulder.W_OUT - shoulder.W_IN), "roll (61808 pair)": abs((roll.W2[0] + roll.W2[1]) - (roll.W1[0] + roll.W1[1])) / 2,
             "yaw (61808 pair)": abs((roll.P1[0] + roll.P1[1]) - (roll.P2[0] + roll.P2[1])) / 2, "elbow (61808 + 6708)": 108.0}
    lev = {"pitch (61808 pair)": reach, "roll (61808 pair)": reach, "yaw (61808 pair)": reach, "elbow (61808 + 6708)": (a.l_fore + a.l_hand) * 1000}
    return [(k, s, 2 * PLAY_MM * SHIM / s * lev[k]) for k, s in spans.items()]


def main():
    from . import bones
    from .params import BASELINE
    rows = run()
    br = bearing_terms()
    tube, tube_bones = tube_term(), bones.score(BASELINE)[1]
    m_roll, m_yaw = static_moments()
    tot = sum(r["hand"] for r in rows) + sum(b[2] for b in br) + tube
    after = tot * (1 - COMP)
    L = ["# Hand-deflection compliance chain (generated by `python -m rnd.stiffness`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Static hand deflection, 1 kg straight out: {tot:.1f} mm uncompensated (target ≤ 6), {after:.1f} mm after a gravity model (target ≤ 2).** "
         f"{'✅' if tot <= 6 and after <= 2 else '❌'}", "",
         "## 1. Printed parts (P2 vs P1 on the same mesh)", "",
         "| Part | Elements | Seat rotation P1 → P2 (mrad, static) | P2/P1 | Seat translation (mm) | Lever to hand (mm) | Hand (mm) | Von Mises P2 max / 99.5 % (P1 99.5 %) |",
         "|---|---|---|---|---|---|---|---|"]
    L += [f"| {r['part']} | {r['n']:,} | {r['rot_p1'] * 1000:.2f} → {r['rot_p2'] * 1000:.2f} | {r['ratio']:.2f} | {r['t']:.2f} | {r['L']:.0f} | {r['hand']:.2f} | "
          f"{r['vmax_p2']:.1f} / {r['vm99_p2']:.1f} ({r['vm99_p1']:.1f}) MPa |" for r in rows]
    L += ["", "## 2. Bearing play (shimmed pairs, 30 % of 10 µm C0 clearance)", "", "| Joint | Span (mm) | Hand (mm) |", "|---|---|---|"]
    L += [f"| {k} | {s:.0f} | {h:.2f} |" for k, s, h in br]
    L += ["", f"## 3. Carbon tubes (rnd/bones.py, CF segments only): {tube:.2f} mm "
          f"(bones.py's whole beam, incl. the printed boxes the FEA already solves: {tube_bones:.2f} mm)", "",
          f"Static arm moments, 1 kg straight out (point masses): {m_roll:.1f} N·m at the roll link, {m_yaw:.1f} N·m at the yaw seats. "
          "Cases solved: " + "; ".join(r["case"].split(":")[0] for r in rows) + ".", "",
          "**Reading:** P2 elements roughly double the P1 bending rotations here, as the review said; P1 FEA alone under-states the deflection. "
          "The stress margins in fea.md (P1, 99.5th percentile) are re-checked by the P2 column: compare with the 12 MPa across-layer allowable."]
    (DOCS / "stiffness.md").write_text("\n".join(L))
    (DOCS / "stiffness.json").write_text(json.dumps({"total_mm": tot, "after_mm": after, "rows": rows, "bearings": br, "tube": tube,
                                                         "tube_bones": tube_bones, "m_roll": m_roll, "m_yaw": m_yaw}))
    print("\n".join(L[4:8]))


if __name__ == "__main__":
    main()
