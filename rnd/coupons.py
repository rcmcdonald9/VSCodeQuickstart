"""Coupons: test prints for the riskiest mechanisms (filament + a few bought parts).  python -m rnd.coupons

Writes STLs to rnd/out/coupons/, slices them for time and grams, and writes docs/COUPONS.md (protocol + results table).
Each coupon maps to the parameter(s) its measurement updates.

Fable review #2 (2026-11-28, M4): Step-1 exit criterion 6 had no coupons behind it, and the list still carried the KX socket
(D24 removed it from the arm) and the eccentric bushing (D26 removed it). Now:
- **C12 insert pull-out**, **C13 4-ear flange** (the D24 v2 bolted cartridge) and **C14 drive stub** (D26) are the Step-1
  coupons, each with what it proves, a pass number computed from the repo's own load cases (step1_coupons()) and the print
  and test method; C9 (bolt circles) feeds C14.
- C1 / C2 (KX74 socket + male face) and C5 (eccentric bushing) are retired (RETIRED): not printed, kept as history.
- QUEUED: the joints.md coupons not defined yet (a Step-2 gate).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import Align, Box, Cylinder, Pos, Rot, export_stl

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out" / "coupons"
DOCS = ROOT.parent / "docs"
MIN = (Align.CENTER, Align.CENTER, Align.MIN)
G = 9.81


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


# --- C12: heat-set insert pull-out ----------------------------------------------------------------------------------------
# The insert holes the arm still loads in pull-out (Fable C1 moved the two arm joints off inserts; these remain): hole Ø ×
# depth from the CAD, where it is used, and the screw's build torque → preload (F = T / (K d), K 0.2, as cartridge_m.FLANGE_LOAD).
INSERTS = [
    dict(size="M3", d=4.0, L=4.0, use="rear-cap lugs (cartridge_m.CAP; the screw pushes it into the boss), the D24 v2 cap", T=0.5),
    dict(size="M3", d=4.0, L=6.0, use="elbow +Y seat cap (arm.SEAT_SCREW '+'), bone / housing caps, encoder brackets", T=0.5),
    dict(size="M4", d=5.6, L=5.7, use="the short M4 of the kit (Ø5.6 × 5.7, Step 1a line); flange-side bosses", T=0.6),
    dict(size="M4", d=5.6, L=8.0, use="elbow −Y seat cap (arm.SEAT_SCREW '-'), roll end wall (roll.END_WALL), shoulder web, exit-slot strap", T=0.6),
]
BOSS_WALL = 3.0          # mm round the insert hole: the thinnest load-carrying boss in the arm (the rear-cap lug, cartridge_m.CAP), ESTIMATE
FLOOR_UNDER = 3.0
M3_PULLOUT_EST = 1000.0  # N: the M3 insert pull-out the repo assumed (rnd/joints.py 'before' rows, roll_inline.md yaw row), ESTIMATE
M5_PULLOUT_EST = 3000.0  # N: twice the belt-roll pinch preload, "≈ 50 % of M5 insert pull-out" (rnd/parts/roll.py PINCH), ESTIMATE


def insert_pullout():
    """C12: 4 insert variants (rows) × 3 repeats (columns), each in a free-standing boss with BOSS_WALL of print round the hole,
    on a 3 mm base. Insert axis = print Z (the pull runs across the layers, the weak way). Row k has k + 1 notches on the base."""
    pitch_x, pitch_y = 30.0, 26.0
    nx, ny = 3, len(INSERTS)
    p = Box(nx * pitch_x + 6, ny * pitch_y + 6, 3, align=MIN)
    for k, ins in enumerate(INSERTS):
        y = -(ny - 1) * pitch_y / 2 + k * pitch_y
        for i in range(nx):
            x = -(nx - 1) * pitch_x / 2 + i * pitch_x
            h = ins["L"] + FLOOR_UNDER
            p += Pos(x, y, 3) * Cylinder(ins["d"] / 2 + BOSS_WALL, h, align=MIN)
            p -= Pos(x, y, 3 + h - ins["L"] - 1.0) * Cylinder(ins["d"] / 2, ins["L"] + 1.01, align=MIN)
        for n in range(k + 1):
            p -= Pos(-(nx * pitch_x + 6) / 2 + 3 + 3 * n, y, 2.0) * Box(1.2, 4, 1.01, align=MIN)
    return p


# --- C13: D24 v2 bolted cartridge flange, 4 ears ---------------------------------------------------------------------------
C13_SITE = "elbow"       # 4 plain ears at ±40° / ±140° with pockets on wall A (no reversed or side-fed ear): the base case
C13_ARM_R = 100.0        # mm: the load hole on the test arm


def flange_4ear():
    """C13: (wall-A plate, flange) as two prints. The plate is the stage wall (z −6 … 0) cut by belt_stage.wall_a_cuts_v4 for
    the elbow ear pattern (Ø40.6 hole, lip counterbore, 4 Ø4.5 holes, 4 ISO 4032 nut pockets), with 4 clamp holes. The flange
    is the bolted can (cartridge_m.can_bolted) cut to its lip + flange + 8 mm of nose, plus a TEST-ONLY radial arm at 0°
    (between the ±40° ears) with a Ø6.5 load hole at r C13_ARM_R."""
    from .parts import belt_stage as bs, cartridge_m as cm
    plate = Pos(0, 0, -6) * Box(140, 140, 6, align=MIN) - bs.wall_a_cuts_v4(cm.MOTOR, C13_SITE)
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= Pos(60 * sx, 60 * sy, -10) * Cylinder(3.25, 20, align=MIN)
    can = cm.can_bolted(cm.MOTOR, C13_SITE)
    z1 = cm.EAR["t"] + 8.0
    fl = can & (Pos(0, 0, -5) * Box(200, 200, z1 + 5, align=MIN))
    n, f = cm.nose_flange(cm.MOTOR, C13_SITE)
    r0 = n["r_out"] - 1.0
    arm = Pos((r0 + C13_ARM_R + 10) / 2, 0, f["t"]) * Box(C13_ARM_R + 10 - r0, 24, 12, align=MIN)
    arm -= Pos(C13_ARM_R, 0, 0) * Cylinder(3.25, 40)
    arm -= Pos(0, 0, -5) * Cylinder(n["r_in"], 40, align=MIN)
    return plate, fl + arm


# --- C14: D26 drive stub on the GIM output ---------------------------------------------------------------------------------
C14_LEVER_R = 250.0      # mm: where the weights hang on the lever stock for the tilt and torque loads


def stub_template():
    """C14: a 3 mm drilling template / stand-in flange (Ø42, as belt_stage.STUB) with 8 × Ø3.4 on cartridge_m.MOTOR['out_pcd']
    and the Ø20 pilot: drill the lever stock and the dummy output through it. Re-generate after C9 has set the real PCD."""
    from .parts import belt_stage as bs, cartridge_m as cm
    st, m = bs.STUB, cm.MOTOR
    p = Cylinder(st["flange_od"] / 2, 3, align=MIN) - Cylinder(st["pilot_d"] / 2 + 0.1, 3, align=MIN)
    for k in range(m["out_holes"]):
        a = 2 * math.pi * k / m["out_holes"] + math.pi / 8
        p -= Pos(m["out_pcd"] / 2 * math.cos(a), m["out_pcd"] / 2 * math.sin(a), 0) * Cylinder(1.7, 3, align=MIN)
    p -= Pos(st["flange_od"] / 2 - 1.5, 0, 2.4) * Box(3, 1, 1, align=MIN)          # index mark (hole 1 at +22.5°)
    return p


def stub_tilt_limit():
    """The GIM output tilt stiffness above which the D26 stub between the motor bearing and the wall-B 6001 (belts.md
    'hinged') no longer holds n ≥ 1.5 in fatigue at the 11 N·m peak with the 0.15 mm swap offset. Hand model on
    rnd/belts.py stub_check: the clamped-end moment from an offset δ scales with δ_eff = δ · k_θ / (k_θ + k_rot), where
    k_rot = k_lat · L² is the stub's own end-rotation stiffness (pinned at the 6001); bisect δ_eff for n = 1.5."""
    from . import belts as b
    from .parts import cartridge_m as cm
    L = b.STUB_SPAN()
    hinged = b.stub_check(cm.MOTOR, "v5", 11.0)
    k_rot = hinged["k"] * L * L / 1000.0                       # N·m / rad
    lo, hi = 0.0, b.STUB_MISALIGN["swap"]
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if b.stub_check(cm.MOTOR, "v5", 11.0, clamped=True, delta=mid)["n"] > 1.5 else (lo, mid)
    d_eff, d_sw = lo, b.STUB_MISALIGN["swap"]
    k_max = k_rot * d_eff / (d_sw - d_eff)
    rigid = b.stub_check(cm.MOTOR, "v5", 11.0, clamped=True, delta=d_sw)
    return dict(span=L, k_rot=k_rot, d_eff=d_eff, k_max=k_max, n_hinged=hinged["n"], n_rigid=rigid["n"], R_motor=hinged["Rm"],
                F_shaft=hinged["F"])


def step1_coupons():
    """The Step-1 coupons C12–C14 with their pass numbers from the repo's load cases: list of dict(id, name, proves, passes,
    method, updates, when). Light (no CAD): rnd/m1_readiness.py reads it."""
    from .parts import cartridge_m as cm
    T_pk = 11.0                                                # GIM6010-8 peak (the drives' flash limit is set to rated; the peak is the fault case)
    fj, fj30 = cm.flange_joint(cm.MOTOR, None, C13_SITE), cm.flange_joint(cm.MOTOR, None, C13_SITE, retained=cm.FLANGE_LOAD["retained"])
    Fp = cm.FLANGE_LOAD["preload"]
    st = stub_tilt_limit()
    t_min = 9.8 / st["k_max"]                                   # rad at 9.8 N·m
    pre = {s: 1000 * ins["T"] / (0.2 * (3.0 if s == "M3" else 4.0)) for s, ins in ((i["size"], i) for i in INSERTS)}
    m4_pass = 2 * Fp
    lever = 5.0
    c12 = dict(
        id="C12", name="Heat-set insert pull-out (Fable C1 / #2 M4)", when="1a (no motor)",
        proves="what one insert in the arm's print holds when its screw pulls it out of the boss, the ESTIMATE behind every insert-in-pull-out "
               "joint left in the arm (seat caps, roll end wall, shoulder web, exit-slot strap, cap rims) and behind the 'before' rows of joints.md",
        passes=[f"M3 (Ø4.0 × 4.0 and × 6): mean ≥ {M3_PULLOUT_EST / 1000:.1f} kN, lowest of 3 ≥ {0.8 * M3_PULLOUT_EST / 1000:.1f} kN (the 1.0 kN "
                f"the repo assumed: rnd/joints.py, roll_inline.md). For scale: an M3 tightened to {INSERTS[0]['T']} N·m already pulls ≈ {pre['M3']:.0f} N",
                f"M4 (Ø5.6 × 5.7 and × 8): mean ≥ {m4_pass / 1000:.1f} kN, lowest ≥ {0.8 * m4_pass / 1000:.1f} kN (2 × the {Fp:.0f} N M4 preload the "
                f"repo tightens to, cartridge_m.FLANGE_LOAD, so a seat cap or strap can take its working load on top)",
                f"torque-out: no insert spins below 2 × its build torque (M3 {2 * INSERTS[0]['T']:.1f} N·m, M4 {2 * INSERTS[2]['T']:.1f} N·m)",
                f"M5 (Ø6.4 × 10, belt-roll fallback only, roll.PINCH): ≥ {M5_PULLOUT_EST / 1000:.1f} kN; print it only if `PIP_ROLL_DRIVE=belt` comes back"],
        method=f"Print `C12_insert_pullout` in the arm's material (PETG, print_plan.md), 4 perimeters, flat, insert axes vertical (the pull runs "
               f"across the layers). Press the inserts with the iron at the filament's print temperature, flush ± 0.1. Test each site: a steel washer "
               f"(bore = insert OD + 0.5) on the boss top as the clamped part, a screw through it into the insert, pulled through a {lever:.0f}:1 lever "
               f"(the 300 mm lever stock pivoting on an M5 in the stand, the screw 60 mm from the pivot, a bucket at 300 mm filled with water in 1 L "
               f"steps: {lever * G:.0f} N per litre at the insert). Record the load at failure and the mode (insert out clean / boss split / layers "
               "parted); then torque-out on fresh sites with a torque screwdriver.",
        updates="joints.py M3 pull-out ESTIMATE; roll.py PINCH preload (belt roll); arm.SEAT_SCREW sizes; cartridge_m.CAP insert size; "
                "a failing row moves that joint to a through-bolt + nut (as the D24 v2 ears)")
    c13 = dict(
        id="C13", name="Bolted 4-ear cartridge flange (D24 v2, Neon #2 F2)", when="1a (no motor)",
        proves="the friction μ and the preload retention of the printed flange-to-wall joint that carries every GIM6010 cartridge's torque "
               "and tipping (cartridge_m.flange_joint: μ 0.3 PETG on PETG and 30 % preload after creep are ESTIMATEs), the nut pockets' 2.6 mm floor",
        passes=[f"fresh, 4 × ISO 4762 M4 × 12 at 0.6 N·m (≈ {Fp:.0f} N each): no slip at **{1.5 * T_pk:.1f} N·m** (1.5 × the {T_pk:.0f} N·m GIM6010 peak), "
                "10 × each way, 1 min each (a witness line across flange and plate moves ≤ 0.1 mm at r 48)",
                f"fresh breakaway torque recorded → μ = T_slip / (4 × {Fp / 1000:.2f} kN × r {cm.ear_r():.0f} mm); the model's {fj['T_fric']:.0f} N·m is μ 0.3 "
                f"(the 50 kg luggage scale at r {C13_ARM_R:.0f} reaches 49 N·m; if it does not slip, record > 49)",
                f"tipping: no 0.05 mm feeler enters at any ear under **{1.5 * fj['M_app']:.1f} N·m** bending (1.5 × the {fj['M_app']:.1f} N·m direct-stage "
                f"case, 3 g + the belt pull on the motor bearing; the model opens at {fj30['M_open']:.0f} N·m with 30 % preload)",
                f"after 72 h at 50 °C on the P2S bed (preload only): no slip at **{fj30['T_fric']:.0f} N·m** (the 30 %-retained friction the flange rows "
                f"assume, {fj30['T_fric'] / T_pk:.2f} × the peak); each ear screw's breakaway torque recorded",
                "one spare ear torqued to 1.2 N·m (2 × the preload): the nut does not pull into its pocket, the pocket floor does not crack"],
        method=f"Print `C13_flange_plate` (the stage wall, elbow ear pattern) and `C13_flange_can` (the can's lip, flange and 8 mm of nose, plus a "
               f"test-only arm with a load hole at r {C13_ARM_R:.0f}) in the stage walls' and cans' materials (print_plan.md). Clamp the plate to the "
               "stand by its 4 corner holes. Torque: plate vertical, arm horizontal, weights or the luggage scale on the load hole (1 kg there = "
               f"{G * C13_ARM_R / 1000:.2f} N·m). Tipping: plate horizontal, the load hanging from the arm (the moment is F × r). Creep: the "
               "assembled pair on the bed at 50 °C for 72 h (as the old C1 hold), then re-test.",
        updates="cartridge_m.FLANGE_LOAD mu_face, retained; belts.md flange rows; EAR (pocket depth)")
    c14 = dict(
        id="C14", name="D26 drive stub on the GIM output (Fable #2 M3)", when="1a on the bare motor (unpowered) + the dummy output; 1b on the rig",
        proves="(a) whether the GIM6010-8's output bearing tilts freely enough for the stub to sit between it and the wall-B 6001 (belts.md "
               "'hinged'), its PCD, pilot and thread depth; (b) the 8 × DIN 7984 M3 × 6 + Loctite 243 friction joint at 1.5 × the peak with the "
               "belt's side load; (c) the same joint on the real output at the drive's peak",
        passes=[f"(a) tilt stiffness of the output (secant 0 → 9.8 N·m, the housing's own tilt subtracted) **≤ {st['k_max'] / 1000:.0f} kN·m/rad** "
                f"(≈ {st['k_max'] / 3437.75:.1f} N·m per arcmin), i.e. ≥ {t_min * 3437.75:.1f}′ of tilt at 9.8 N·m ({t_min * 3437.75 * 1.75:.1f} mm of "
                f"spot travel on the optical lever at 3 m; {t_min * 20 * 1000:.0f} µm on a dial at r 20): then the hinged design stands (fatigue {st['n_hinged']:.2f}×). Stiffer: the 0.15 mm swap offset drives the stub to "
                f"{st['n_rigid']:.2f}× (fully rigid), so quote the overhung pinion instead (belts.py STUB_SUPPORT_RULE). Hand model, ESTIMATE",
                "(a) PCD, pilot Ø and thread depth read to ± 0.1 mm (C9 disc + calipers) into cartridge_m.MOTOR['out_pcd'] and belt_stage.STUB; the "
                "thread depth ≥ 4 mm for M3 × 6 under a 1.8 mm counterbore",
                f"(b) on the dummy output: no slip (witness ≤ 0.05 mm at r 17) at **{1.5 * T_pk:.1f} N·m**, 10 × each way, with a **{st['R_motor']:.0f} N** "
                f"radial pull at the flange (the motor bearing's share of the {st['F_shaft']:.0f} N shaft load at the peak, belts.md hinged); "
                f"breakaway of each screw ≥ its build torque after (the model holds 27.2 N·m: μ 0.2 × 8 × 1.0 kN × r 17, 2.47×)",
                f"(c) 1b, on the rig: the drive at its {T_pk:.0f} N·m peak against the hard stop, 10 ×: no slip (witness mark)"],
        method=f"(a) Motor housing bolted to the stand's plate, output up, unpowered. The lever stock across the output (4 × M3 is enough here), "
               f"weights of 1 / 2 / 4 kg hung at r {C14_LEVER_R:.0f} (2.5 / 4.9 / 9.8 N·m, no torque about the axis). Read the tilt on the output "
               "flange itself, not on the lever (the 6 mm bar bends millimetres): the optical lever first (a small mirror on the flange, a laser "
               "spot on a wall 3 m away, 1′ ≈ 1.75 mm of spot), or a 0.002 mm dial test indicator on the flange rim at r ≈ 20 (near its resolution); "
               "the same reading on the housing, subtracted; turn the lever 180° and repeat. Note the free play (tilt at ± 0.5 N·m) separately. "
               "(b) The dummy output: the 6061 offcut drilled and tapped through `C14_stub_template` on the measured PCD, in a vise; the lever stock "
               "counterbored to leave 1.8 mm under the heads (as the stub flange), 8 × M3 × 6 + 243 at ≈ 0.6 N·m (1.0 kN, K 0.2), cured 24 h; "
               f"6.7 kg at r {C14_LEVER_R:.0f} is 16.4 N·m; the radial pull with a ratchet strap and the luggage scale at the hub. (c) On the 1b rig.",
        updates="belts.py stub support rule (hinged or overhung pinion), cartridge_m.MOTOR out_pcd, belt_stage.STUB pilot; the pinion quote")
    return [c12, c13, c14]


def parts():
    from .parts.belt_stage import pulley
    from .parts.cartridge_m import coupon_motor
    from .parts.hand import fin_ray
    from .parts.upper import carriage
    rs = carriage(750.0) & Pos(-110.0, 23.0, 720.0) * Box(30, 40, 60)          # A15 (lift v2): the steel rack + pawl are bought; the print to prove is the pawl box
    bb = rs.bounding_box()
    fplate, fcan = flange_4ear()
    return {
        "C3_clearance_ladder": (clearance_ladder(), "PETG", "Print-in-place; try to turn each pin by hand"),
        "C4_pressfit_ladder": (pressfit_ladder(), "PETG", "Press the real pins/balls in; note the tightest row that still assembles without cracking"),
        "C6_pulleys_20T_40T": (pulley(20) + Pos(60, 0, 0) * pulley(40), "PETG", "Check against a real HTD 5M belt: engagement, no tooth ride-over (the 20T is now the hobbed steel pinion, D26 v5: its print is only a belt gauge)"),
        "C7_finray_finger": (fin_ray(0, -37, 0, 18, 28, -1), "TPU 95A", "Push a Ø66 can into the face: the face should wrap ≥ 30° of the can"),
        "C8_pawl_box": (Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * rs, "PETG", "Steel pawl on its Ø4 pin in the printed pawl box, on a length of the m 1 rack: hang 15 kg (×1.2 the moving mass) for 1 h"),
        "C9_gim_boltcircle": (coupon_motor(), "PETG", "Bolt to the GIM6010-8 front and back; read the real PCDs from the ruler ticks (Step 1a, feeds C14)"),
        "C12_insert_pullout": (insert_pullout(), "PETG", f"Flat, 4 perimeters; rows (notches 1–4) {', '.join(f'{i['size']} Ø{i['d']:.1f} × {i['L']:.1f}' for i in INSERTS)}, 3 bosses each ({BOSS_WALL:.0f} mm wall)"),
        "C13_flange_plate": (fplate, "PETG", "Stage wall A coupon (elbow 4-ear pattern, nut pockets up); flat"),
        "C13_flange_can": (fcan, "ASA", "The bolted can's lip + flange + nose, with a test-only load arm; flange face down"),
        "C14_stub_template": (stub_template(), "PETG", "Drilling template / stand-in flange: 8 × Ø3.4 on MOTOR['out_pcd'] + Ø20 pilot; re-generate after C9"),
    }


PROTOCOL = [
    ("C3", "Smallest gap whose pin turns freely after printing", "≤ 0.30 mm frees", "kx.py clearance (0.30; the head KX44 sockets, D24 took the arm's)"),
    ("C4", "Tightest press-fit row that assembles without cracking (pins, balls)", "one row works", "kx.py pin/ball hole diameters (head KX44: Ø3 pins, Ø6 balls)"),
    ("C6", "Belt engagement on printed pulleys, ride-over under hand load", "no ride-over at ~300 N", "belt_stage tooth form (DEPTH, GROOVE_R) of the printed large pulleys"),
    ("C7", "Fin-ray wrap angle on a can; return after 100 squeezes", "≥ 30° wrap; no permanent set", "hand.fin_ray (rib count, wall)"),
    ("C8", "Pawl box + pin under 15 kg for 1 h on the steel rack (A15); pawl engagement", "no pin-hole elongation, < 0.5 mm creep", "upper.carriage pawl pocket, upper.lift_pawl"),
    ("C9", "Real GIM6010-8 bolt circles (needs one motor: Step 1a)", "read PCD to ±0.5 mm", "cartridge_m.MOTOR out_pcd / rear_pcd"),
    ("C10", "D16 wrist-roll dog coupling + spindle (loop 40): 2 × 61804 in the printed skirt; 1 kg on a 90 mm lever, rotate ±180° 1000 times by hand drill at low speed",
     "no play you can feel at the gripper flange; dog slot wear ≤ 0.1 mm", "hand.py BRG_WR seats, dog slot width"),
    ("C11", "Clock-spring ribbon life (loop 40, D14 ribbon in a printed yaw cassette): 10,000 cycles over ±150° on a spare GIM or a drill jig",
     "no conductor break (continuity logged); USB still enumerates at the end", "harness.py SPIRAL lengths, ribbon carrier tape"),
]
# Retired by a decision (not printed; the numbers are kept for the record)
RETIRED = [
    ("C1", "KX74 socket + collar: lock twist ≤ 3 N·m, slip ≥ 12 N·m, re-seat 0.05 mm, 72 h creep hold", "D24 (owner 2026-11-27): every arm cartridge bolts in; "
     "the wheels bolt to the GIM output (D17) and the lift is a bolted stage (W12), so no KX74 socket is left"),
    ("C2", "KX74 male face: balls flush ± 0.05, ball pull-out ≥ 20 N", "D24, as C1 (the head KX44 male faces are a different size; C4 covers their fits)"),
    ("C5", "Eccentric bushing: 6804 seat, rotates and holds 300 N belt pull", "D26 (owner 2026-11-27): no eccentrics; fixed centres with a tension ring "
     "(elbow, yaw) or a slotted idler (pitch)"),
]
# joints.md "coupons to add" (Fable #2 M4): named, not defined yet; a Step-2 gate (rnd/joints.py owns the numbers)
QUEUED = [
    "printed tube clamped as wave 3 (no sleeves) at 8.8 MPa, 1000 h at 40 °C with a load washer: measures φ, the creep ESTIMATE behind joints.md",
    "the sleeved C1 joint's moment to gap (pitch hub ↔ tube, yaw flange ↔ tube)",
    "the 7075-T6 yaw flange's tapped M3 threads (strip vs the 12.9 screw)",
    "the R17 front stop: a 7075 dog on its block at 3 kN, anodised pair, 100 hits (wear and dent) (W16c; replaces the old stop-bush coupon)",
    "the inline roll output dowels' slip torque on a seller motor (bonded with Loctite 638)",
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
    S = step1_coupons()
    L = ["# Coupons: test prints before buying hardware (generated by `python -m rnd.coupons`)", "",
         "**Why:** these are the riskiest mechanisms in the design. Most cost only filament; C12–C14 use parts in the Step 1a kit "
         "([M1_BENCH_ARM.md](M1_BENCH_ARM.md)). Print them first and record the measurements below; each result updates the named parameter, and the "
         "loop re-runs every check.", "",
         "**Step-1 coupons (M1 exit criterion 6, Fable review #2 M4): C12 insert pull-out, C13 4-ear flange, C14 drive stub**, plus C9 (the bolt "
         "circles C14 needs). Pass numbers come from the repo's load cases (`rnd/coupons.py step1_coupons()`).", "",
         f"**Total:** {tg:.0f} g, {th:.1f} h with PrusaSlicer's generic profile ≈ **{th * 0.35:.1f} h on the P2S** (ESTIMATE).", "",
         "STLs: `rnd/out/coupons/`.", "",
         "| Coupon | Material | Generic time (h) | Filament (g) | How to print |", "|---|---|---|---|---|"]
    L += [f"| {n} | {m} | {h:.1f} | {g:.0f} | {note} |" for n, m, h, g, note in rows]
    L += ["", "## Step-1 coupons (C12–C14)", ""]
    for c in S:
        L += [f"### {c['id']}: {c['name']} — {c['when']}", "", f"**Proves:** {c['proves']}.", "", "**Pass:**", ""]
        L += [f"- {p}" for p in c["passes"]]
        L += ["", f"**Print and test:** {c['method']}", "", f"**Updates:** {c['updates']}.", "",
              "| Result | |", "|---|---|", "| measured | |", "| pass / fail | |", ""]
    L += ["## Other coupons: fill in the Result column", "", "| # | Measure | Pass if | Updates | Result |", "|---|---|---|---|---|"]
    L += [f"| {c} | {m} | {p} | {u} | |" for c, m, p, u in PROTOCOL]
    L += ["", "## Retired (not printed)", "", "| # | Was | Retired by |", "|---|---|---|"]
    L += [f"| {c} | {w} | {d} |" for c, w, d in RETIRED]
    L += ["", "## Queued, not defined yet (joints.md; a Step-2 gate)", ""] + [f"- {q}" for q in QUEUED]
    L += ["", "**Small parts for the coupons:** C12–C14 use the Step 1a kit's inserts, M4 × 12 + washers + nuts, M3 × 6, Loctite 243, the lever "
          "stock, the luggage scale, the dial test indicator and the 6061 offcut. Others (≈ $15): Ø3 × 8 dowel pins and Ø6 G25 balls (C4, the head "
          "KX44), a 300–400 mm HTD 5M-15 belt (C6), TPU filament (C7).", "",
          "C9 and C14 (a) need the Step-1a GIM6010-8; C14 (c) needs the Step-1b rig."]
    (DOCS / "COUPONS.md").write_text("\n".join(L) + "\n")
    (ROOT.parent / "docs" / "rnd" / "coupons.json").write_text(json.dumps(
        {"hours_p2s": th * 0.35, "grams": tg, "step1": [{"id": c["id"], "name": c["name"], "passes": c["passes"]} for c in S]}, indent=1))
    print("\n".join(L[:12]))


if __name__ == "__main__":
    main()
