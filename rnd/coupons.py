"""Coupon round C1: test prints for the riskiest new mechanisms (filament only).  python -m rnd.coupons

Writes STLs to rnd/out/coupons/, slices them for time and grams, and writes docs/COUPONS.md (protocol + results table).
Each coupon maps to the parameter(s) its measurement updates.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

from build123d import Align, Box, Cylinder, Pos, Rot, Text, export_stl, extrude

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out" / "coupons"
DOCS = ROOT.parent / "docs"
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def clearance_ladder():
    """Print-in-place clearance ladder: 5 pins standing in 5 holes, radial gaps 0.20 … 0.40 mm (the KX collar uses 0.30).
    After printing, the pins should turn and lift out; the smallest gap that frees cleanly sets the CAD clearance."""
    gaps = (0.20, 0.25, 0.30, 0.35, 0.40)
    plate = Box(5 * 24 + 8, 30, 3, align=MIN)
    for i, g in enumerate(gaps):
        x = -2 * 24 + 24 * i
        plate -= Pos(x, 0, 0) * Cylinder(6 + g, 3, align=MIN)
        plate -= Pos(x, -12, 2.4) * Box(4 + 2 * i, 2, 0.6, align=MIN)          # gap marker (bar length = ladder step)
        plate += Pos(x, 0, 0) * Cylinder(6, 9, align=MIN)                      # free-standing pin, fused only if the gap is too small
    return plate


def pressfit_ladder():
    """Press-fit holes for the KX steel parts: dowel pins Ø3/Ø4/Ø5 (horizontal, as in the hub) and balls Ø6/Ø8/Ø10,
    each at −0.05 / −0.10 / −0.15 mm on the diameter. The best row sets the CAD fit (kx.py)."""
    p = Box(140, 70, 12, align=MIN)
    for r, d in enumerate((3.0, 4.0, 5.0)):
        for c, dd in enumerate((-0.05, -0.10, -0.15)):
            p -= Pos(-50 + 18 * c, -24 + 12 * r, 6) * Rot(0, 90, 0) * Cylinder((d + dd) / 2, 30)
    for r, d in enumerate((6.0, 8.0, 10.0)):
        for c, dd in enumerate((-0.05, -0.10, -0.15)):
            p -= Pos(20 + 16 * c, -22 + 20 * r, 12) * Cylinder((d + dd) / 2, d * 0.6)
    return p


def parts():
    from .parts.belt_stage import eccentric_bushing, pulley
    from .parts.cartridge_m import coupon_motor
    from .parts.hand import fin_ray
    from .parts.kx import KX74, collar, hub_pip, male
    from build123d import Box
    from .parts.upper import carriage
    rs = carriage(750.0) & Pos(-110.0, 23.0, 720.0) * Box(30, 40, 60)          # A15 (lift v2): the steel rack + pawl are bought; the print to prove is the pawl box
    bb = rs.bounding_box()
    return {
        "C1_kx74_socket_and_collar": (hub_pip(KX74) + collar(KX74, locked=False), "PETG", "D15 screwed socket: print hub, collar and retainer separately; press 6 × Ø5×14 pins in radially, 6 × Ø4 coil springs in the step pockets, collar on, retainer 3 × M3"),
        "C2_kx74_male_face": (male(KX74), "PETG", "Snap 3 × Ø10 balls past the equator (1 mm; the rim holds 58 % of the circumference, open only at the pin tracks), retaining compound; pull each ball out with a hook + luggage scale: ≥ 20 N"),
        "C3_clearance_ladder": (clearance_ladder(), "PETG", "Print-in-place; try to turn each pin by hand"),
        "C4_pressfit_ladder": (pressfit_ladder(), "PETG", "Press the real pins/balls in; note the tightest row that still assembles without cracking"),
        "C5_eccentric_bushing": (eccentric_bushing(), "PETG", "Fit a 6804 bearing; rotate in a Ø38.2 test hole"),
        "C6_pulleys_20T_40T": (pulley(20) + Pos(60, 0, 0) * pulley(40), "PETG", "Check against a real HTD 5M belt: engagement, no tooth ride-over"),
        "C7_finray_finger": (fin_ray(0, -37, 0, 18, 28, -1), "TPU 95A", "Push a Ø66 can into the face: the face should wrap ≥ 30° of the can"),
        "C8_pawl_box": (Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * rs, "PETG", "Steel pawl on its Ø4 pin in the printed pawl box, on a length of the m 1 rack: hang 15 kg (×1.2 the moving mass) for 1 h"),
        "C9_gim_boltcircle": (coupon_motor(), "PETG", "Bolt to the GIM6010-8 front and back; read the real PCDs from the ruler ticks"),
    }


PROTOCOL = [
    ("C1", "KX socket: lock twist torque (luggage scale on the collar tab), slip torque (lever + scale), re-seat repeatability over 10 cycles (dial or calipers on a 100 mm arm). "
     "Then **creep hold** (loop 29): leave it locked on the P2S bed at 50 °C for 72 h and measure slip torque again",
     "twist ≤ 3 N·m; slip ≥ 12 N·m; repeatability ≤ 0.05 mm at 100 mm; slip loss after the hold ≤ 5 %; preload from the 6 springs 160 ± 30 N (scale on the collar lip)", "kx.py: clearance, preload_n, ramp_deg, stroke; rnd/kx_mc.py creep ratio"),
    ("C2", "Male face: balls seated flush ±0.05 mm", "no ball proud > 0.1 mm", "kx.py: ball pocket fit"),
    ("C3", "Smallest gap whose pin turns freely after printing", "≤ 0.30 mm frees", "kx.py clearance (0.30)"),
    ("C4", "Tightest press-fit row that assembles without cracking (pins, balls)", "one row works", "kx.py pin/ball hole diameters"),
    ("C5", "Bearing seat fit; eccentric rotates with a lock screw and holds", "no play, holds 300 N belt pull", "belt_stage.ECC"),
    ("C6", "Belt engagement on printed pulleys, ride-over under hand load", "no ride-over at ~300 N", "belt_stage tooth form (DEPTH, GROOVE_R)"),
    ("C7", "Fin-ray wrap angle on a can; return after 100 squeezes", "≥ 30° wrap; no permanent set", "hand.fin_ray (rib count, wall)"),
    ("C8", "Pawl box + pin under 15 kg for 1 h on the steel rack (A15); pawl engagement", "no pin-hole elongation, < 0.5 mm creep", "upper.carriage pawl pocket, upper.lift_pawl"),
    ("C9", "Real GIM6010-8 bolt circles (needs one motor)", "read PCD to ±0.5 mm", "cartridge_m.MOTOR out_pcd / rear_pcd"),
    ("C10", "D16 wrist-roll dog coupling + spindle (loop 40): 2 × 61804 in the printed skirt; 1 kg on a 90 mm lever, rotate ±180° 1000 times by hand drill at low speed",
     "no play you can feel at the gripper flange; dog slot wear ≤ 0.1 mm", "hand.py BRG_WR seats, dog slot width"),
    ("C11", "Clock-spring ribbon life (loop 40, D14 ribbon in a printed yaw cassette): 10,000 cycles over ±150° on a spare GIM or a drill jig",
     "no conductor break (continuity logged); USB still enumerates at the end", "harness.py SPIRAL lengths, ribbon carrier tape"),
]


def main():
    from .print_plan import slice_one
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, (part, mat, note) in parts().items():
        f = OUT / f"{name}.stl"
        export_stl(part, str(f))
        h, g = slice_one(f)
        rows.append((name, mat, h or 0, g or 0, note))
    th, tg = sum(r[2] for r in rows), sum(r[3] for r in rows)
    L = ["# Coupon round C1: test prints before buying hardware (generated by `python -m rnd.coupons`)", "",
         "**Why:** these are the riskiest *new* mechanisms in the design (see the confidence assessment in RND_LOG). They cost only filament. "
         "Print them first and record the measurements below; each result updates the named parameter, and the hourly R&D loop re-runs every check.", "",
         f"**Total:** {tg:.0f} g, {th:.1f} h with PrusaSlicer's generic profile ≈ **{th * 0.35:.1f} h on the P2S** (ESTIMATE).", "",
         "STLs: `rnd/out/coupons/`.", "",
         "| Coupon | Material | Generic time (h) | Filament (g) | How to print |", "|---|---|---|---|---|"]
    L += [f"| {n} | {m} | {h:.1f} | {g:.0f} | {note} |" for n, m, h, g, note in rows]
    L += ["", "## Test protocol: fill in the Result column", "", "| # | Measure | Pass if | Updates | Result |", "|---|---|---|---|---|"]
    L += [f"| {c} | {m} | {p} | {u} | |" for c, m, p, u in PROTOCOL]
    L += ["", "**Small parts to buy for the coupons (≈ $25):**",
          "- a pack of Ø5×14 dowel pins",
          "- Ø10 G25 balls",
          "- one 6804-2RS bearing",
          "- a 6–8 coil-spring assortment",
          "- a 300–400 mm HTD 5M-15 belt",
          "- TPU filament if you don't have it",
          "",
          "C9 needs one GIM6010-8 (part of the M1 bench-arm kit)."]
    (DOCS / "COUPONS.md").write_text("\n".join(L))
    (ROOT.parent / "docs" / "rnd" / "coupons.json").write_text(json.dumps({"hours_p2s": th * 0.35, "grams": tg}))
    print("\n".join(L[:8]))


if __name__ == "__main__":
    main()
