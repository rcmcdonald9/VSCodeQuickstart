"""Coupons: test prints for the riskiest mechanisms (filament + a few bought parts).  python -m rnd.coupons

Writes STLs to rnd/out/coupons/, slices them for time and grams, and writes docs/COUPONS.md (protocol + results table).
Each coupon maps to the parameter(s) its measurement updates.

Fable review #2 (2026-11-28, M4): Step-1 exit criterion 6 had no coupons behind it, and the list still carried the KX socket
(D24 removed it from the arm) and the eccentric bushing (D26 removed it). Now:
- **C12 insert pull-out**, **C13 4-ear flange** (the D24 v2 bolted cartridge) and **C14 drive stub** (D26) are the Step-1
  coupons, each with what it proves, a pass number computed from the repo's own load cases (step1_coupons()) and the print
  and test method; C9 (bolt circles) feeds C14.
- C1 / C2 (KX74 socket + male face) and C5 (eccentric bushing) are retired (RETIRED): not printed, kept as history.
- C15–C21 (2026-11-28, Fable #2 major 5): the joints.md coupons, defined (joint_coupons()), pass numbers read from the
  load-chain rows in docs/rnd/joints.json (rnd/joints.py writes them): the creep coefficient, the sleeved ties' moment to
  gap, the tapped 7075 threads, the front-stop dog, the output dowels, the elbow core bond and the V-band clamp. A Step-2 gate.
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
        method=f"(a) As docs/M1_RIG.md step 1a-11: motor unpowered on its rig plate, the turned tilt mandrel on the output, a radial load of "
               f"0–200 N through a 6001 at 97 mm (Dyneema cord, luggage scale, turnbuckle), no torque about the axis. Read the tilt on the output "
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
        "C15_creep_tube": (creep_tube(), "PETG", f"The pitch tube's wall r {C15_TUBE['r'][0]:.0f}–{C15_TUBE['r'][1]:.0f} × {C15_TUBE['L']}, axis vertical (as the tube), 4 perimeters"),
        "C20_bond_sleeve": (bond_sleeve(), "PETG", "The elbow shaft's Ø40 tube over the 16 mm bond + a 36 AF hex collar; axis vertical (as the shaft)"),
        "C21_ferrule_male": (Rot(90, 0, 0) * __import__("rnd.parts.roll", fromlist=["x"]).detach_ferrule_male(), "PETG", "Arm-side KX-V ferrule with the 0.2 mm land relief; face down"),
        "C21_ferrule_female": (__import__("rnd.parts.shoulder", fromlist=["x"]).ferrule(male=False), "PETG", "Pitch-side KX-V ferrule (as the pitch hub's); face down"),
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
# joints.md "coupons to add" (Fable #2 M4) are defined since 2026-11-28 as C15–C21 (joint_coupons()); nothing is queued
QUEUED = []
C15_TUBE = dict(r=(11.0, 20.0), L=69.6)      # the pitch tube's clamped wall (shoulder.HUB_TIE tube_r_in … 20, the C1 stack length)
C20_BOND = dict(od=40.0, bore=30.2, L=16.0, collar=24.0)   # the elbow shaft's printed tube, bonded over the 40T's 16 mm


def creep_tube():
    """C15: a printed tube as the pitch tube's wall (r 11–20 × 69.6), clamped bare (no sleeves) between two steel washers on
    an M8 through the bore; 4 witness flats for the dial."""
    r0, r1 = C15_TUBE["r"]
    t = Cylinder(r1, C15_TUBE["L"], align=MIN) - Cylinder(r0, C15_TUBE["L"], align=MIN)
    for k in range(4):
        t -= Rot(0, 0, 90 * k) * Pos(r1 - 0.4, 0, C15_TUBE["L"] / 2) * Box(2, 8, C15_TUBE["L"] + 1)
    return t


def bond_sleeve():
    """C20: the elbow shaft's printed Ø40 tube (Ø30.2 bore, as arm.elbow_pulley_hub + the bond gap) over the 16 mm bond length
    of the 40T, plus a 24 mm hex collar (36 AF) for the vise; the 6061 Ø30 × 1.5 offcut is bonded in the bond length only."""
    from build123d import RegularPolygon, extrude
    c = C20_BOND
    t = Cylinder(c["od"] / 2, c["L"], align=MIN) + Pos(0, 0, c["L"]) * extrude(RegularPolygon(36.0 / math.sqrt(3), 6), c["collar"])
    return t - Cylinder(c["bore"] / 2, c["L"] + c["collar"], align=MIN)


def _jj():
    try:
        return json.loads((DOCS / "rnd" / "joints.json").read_text())
    except (OSError, ValueError):
        return {}


def joint_coupons():
    """C15–C21: the coupons behind the load-chain rows of docs/rnd/joints.md, pass numbers from its JSON (rnd/joints.py)."""
    from .joints import CREEP, joint_models
    from .params import act
    from .parts import arm, roll
    J = _jj()
    rows = J.get("joints", [])
    jm = joint_models()
    svc = lambda j: max((r for r in rows if r["joint"] == j and r["case"] in ("basket × 1.5", "hug × 1.5")), key=lambda r: r["M"], default=dict(M=0.0, N=0.0))
    flt = lambda j: max((r for r in rows if r["joint"] == j and "F_bolt" in r and r["case"] not in ("basket × 1.5", "hug × 1.5")), key=lambda r: r["F_bolt"],
                        default=dict(F_bolt=0.0, strip=0.0))
    mg = {j: min(m.m_gap(0.0) for m in J_["models"]) for j, J_ in jm.items()}
    c15 = dict(id="C15", name="Creep coefficient φ of a print-clamped tube (Neon #3, C1 'before' rows)", when="Step 2 prints (no motor); 6 weeks on the bed",
               proves=f"the creep coefficient φ behind every print-clamped joint (joints.py CREEP: φ {CREEP['phi']} design / {CREEP['phi_nom']} nominal, ESTIMATE "
                      "from printed-PETG data), the reason the C1 ties run in crush sleeves and the D29 V-band needs the W21 disc stack",
               passes=[f"tube r {C15_TUBE['r'][0]:.0f}–{C15_TUBE['r'][1]:.0f} × {C15_TUBE['L']} (the pitch tube's wall) bare on an M8 + Belleville-free steel washers at "
                       f"8.8 MPa (≈ {8.8 * math.pi * (C15_TUBE['r'][1] ** 2 - C15_TUBE['r'][0] ** 2) / 1000:.1f} kN, the wave-3 stress), 1000 h at 40 °C (the P2S bed + a box): "
                       f"the squeeze grows by ≤ {CREEP['phi']:.1f} × its elastic part extrapolated to 2 × 10⁴ h (Findley t^n fit of the 1, 10, 100, 1000 h readings) → φ ≤ {CREEP['phi']}",
                       "φ ≤ 1.0 (the nominal) lets the C1 'before' rows and the V-band's creep term relax; φ > 2.0 fails the design value: the W21 disc stack is resized"],
               method="Print `C15_creep_tube` in the arm's PETG, flat, 4 perimeters. Measure the length on the witness flats with the 0.002 mm dial at 20 °C, "
                      "clamp to the load with a calibrated torque on the M8 (or a dead weight on a lever), read at 1, 10, 100, 1000 h, cool to 20 °C for each reading.",
               updates="joints.py CREEP phi / phi_nom; the V-band life preload (joints.py vband)")
    c16_p = "; ".join(f"{j}: no gap (≤ 5 µm at the tension edge) at **{svc(j)['M']:.0f} N·m** (the worst service row), gap onset ≥ {0.7 * mg[j]:.0f} N·m "
                      f"(0.7 × the model's {mg[j]:.0f} N·m at the life preload)" for j in jm)
    c16 = dict(id="C16", name="Sleeved tie joints: moment to gap (C1 pitch + yaw ties, the bone tie)", when="Step 2, with the turned sleeves and the metal parts",
               proves="the crush-sleeve model (joints.py SleeveClamp): metal stack, preload after embedding, the no-gap moment of each tie",
               passes=[c16_p, "after 72 h at 50 °C under the preload alone, the same no-gap moment within 10 % and each nut's breakaway ≥ its build torque"],
               method="Assemble each joint on its real printed parts (pitch tube + hub; yaw tube + flange; the upper bone's top piece + the yaw flange) with the "
                      "bought studs, sleeves and rings at the build torque. Clamp the metal side in the vise, hang the luggage scale on a 300 mm lever bolted to the "
                      "printed side, a dial test indicator across the interface at the far edge from the load: the gap moment is where the reading leaves the "
                      "linear (closed-joint) slope.",
               updates="joints.py SLEEVE_STOCK / preloads; a short row → a longer sleeve or a higher preload")
    yb, bb = flt("yaw flange ↔ tube"), flt("bone ↔ yaw flange")
    c17 = dict(id="C17", name="Tapped M3 threads in the 7075 flange (yaw tie, bone studs)", when="Step 2, on the flange or a 7075 offcut from the same plate",
               proves="the thread-strip strength (FED-STD-H28 at the 6g/6H limits, joints.py thread_strip) of the two M3 thread sets in the 7075-T6 yaw flange",
               passes=[f"yaw tie (8.3 mm engaged, ISO 4762 12.9): strip ≥ **{1.5 * yb['F_bolt'] / 1e3:.1f} kN** (1.5 × the {yb['F_bolt'] / 1e3:.2f} kN fault bolt force; model {yb['strip'] / 1e3:.1f})",
                       f"bone studs (3.4 mm tapped through the recess floor, DIN 976 10.9 + Loctite 270): strip ≥ **{1.5 * bb['F_bolt'] / 1e3:.1f} kN** (1.5 × {bb['F_bolt'] / 1e3:.2f}; "
                       f"model {bb['strip'] / 1e3:.1f}), and the stud does not turn at 2 × its 0.8 N·m nut torque after 24 h cure"],
               method="Tap a 7075 offcut 5 mm thick through (the bone floor: face it to 3.6 mm) and blind 8.3 mm (the yaw tie) with the production tap; pull each "
                      "screw / stud through a steel spacer with the 5:1 lever and the water bucket (as C12) or a press with the luggage scale; three of each.",
               updates="roll.YAW_TIE / BONE_TIE engagement; joints.py TAU['7075-T6']")
    f = roll.FRONT_STOP
    rc = (f["dog"][0] + f["dog"][1]) / 2
    stall, lim = act("GIM8108-36")["peak"], act("GIM8108-36")["rated"]
    Fst, F35 = stall / (rc / 1000), lim / (rc / 1000)
    c18 = dict(id="C18", name="R17 front stop: 7075 dog on its block (W16c)", when="Step 2, before the inline roll is powered",
               proves="the anodised 7075-on-7075 contact of the roll hard stop (joints.py M1 stop row: one dog takes the stall, 3.91× on strength) in wear and dent",
               passes=[f"static: the dog on its block at **{Fst / 1e3:.2f} kN** (the {stall:.1f} N·m stall at r {rc:.2f}, one dog) for 10 s: no yield (dial ≤ 0.01 mm set), no crack",
                       f"100 hits at the drive limit ({F35 / 1e3:.2f} kN, {lim:.1f} N·m; a 1 kg drop weight from the height that gives the stop's impact energy, ESTIMATE): dent "
                       "≤ 0.05 mm on either face, the hard anodise unbroken (no bright metal under 10× loupe)"],
               method="The two machined parts (an adapter dog and a front-ring block, or offcuts with the same 5.5 mm contact band), hard-anodised, in the press "
                      "with the luggage scale for the static load; the hits with the drop weight in a tube guide. Depth gauge on the contact band before / after.",
               updates="roll.FRONT_STOP (anodise spec, dog width); joints.py M1 stop row")
    c19 = dict(id="C19", name="Inline roll output dowels: slip torque (Neon 6a)", when="Step 2, on the seller GIM8108-36 (or its output boss drawing + a cast offcut)",
               proves="the dowel load sharing and the bearing allowable in the motor's output boss (joints.py DOWEL: 150 MPa cast, ESTIMATE; dowels bonded with Loctite 638)",
               passes=[f"the 7075 adapter on the output, 3 × Ø4 dowels bonded in situ + 3 × M4 × 10 csk: no slip (witness ≤ 0.02 mm at r 30) at **{stall:.0f} N·m** "
                       f"(the stall; model 106 N·m = 1.03×), 10 × each way; dry dowels recorded for comparison (model 69 N·m)"],
               method="Motor unpowered, its housing on the rig plate; the adapter's lever (the 300 mm stock) and the luggage scale at 300 mm (35 kg = 103 N·m; "
                      "a 600 mm lever halves it). Only with the seller's agreement, or on a scrap unit.",
               updates="joints.py DOWEL sb / d_fit; roll.ADAPT")
    el = J.get("elbow", [])
    import re
    tq = lambda tag: next((r for r in el if tag in r["item"]), None)
    r_f = tq("fault")
    Tf = float(re.search(r"T ([0-9.]+) N·m", r_f["case"]).group(1)) if r_f else 62.7
    cc = arm.ELBOW_CORE
    T_bond = lambda tau, L: tau * 2 * math.pi * (cc["od"] / 2) ** 2 * L / 1000
    c20 = dict(id="C20", name="Elbow core bond: 6061 tube in the printed shaft (Neon #2 finding 7)", when="Step 2 prints + the 6061 offcut",
               proves="the epoxy bond between the 6061 Ø30 × 1.5 core and the printed Ø30 bore (joints.py BOND: 3 MPa fault, 1 MPa sustained, ESTIMATE), "
                      "which carries ≈ 86 % of the elbow torque out of the core under the 40T and under the forearm clamp",
               passes=[f"torque-out of the {C20_BOND['L']:.0f} mm bond ≥ **{T_bond(3.0, C20_BOND['L']):.0f} N·m** (3 MPa: the fault row's {Tf:.1f} N·m elbow peak, "
                       f"{arm.elbow_shaft_stress()["share_tors"]:.0%} in the core, needs {arm.elbow_shaft_stress()["share_tors"] * Tf * 1000 / (2 * math.pi * (cc["od"] / 2) ** 2 * C20_BOND["L"]):.2f} MPa under the 40T)",
                       f"sustained: {T_bond(1.0, C20_BOND['L']):.0f} N·m (1 MPa) held 72 h at 50 °C: no rotation (witness line ≤ 0.1 mm at r 20)",
                       "the failure is cohesive in the print or the adhesive, not a clean peel off the abraded bore (else: a knurled core, or the D-flat key)"],
               method="Print `C20_bond_sleeve` axis-vertical as the shaft (layers across the axis), abrade the bore (80 grit) and the tube (Scotch-Brite + IPA), bond "
                      "16 mm of the 6061 offcut with the build epoxy, cure as the datasheet; the hex collar in the vise, the lever on the tube through a cross pin.",
               updates="joints.py BOND; arm.ELBOW_CORE bond note (length under the 40T)")
    vb = J.get("vband_model", {})
    a_ = vb.get("after", {})
    vrow = {r["item"]: r for r in J.get("vband", [])}
    c21 = dict(id="C21", name="Arm detach V-band clamp on the printed ferrules (D29 + W21 disc stack)", when="Step 2 prints + one bought 3\" clamp, before the arm is built",
               proves="that the clamp keeps its preload on two printed flanges (joints.py vband: the band is ≈ 1 µm/kN stiff axially, so embedding and creep take "
                      "a wing nut's whole preload; the W21 stack gives it the stroke) and that the arm's moment does not open the faces",
               passes=[f"stack to its stop: T-bolt **{a_.get('T_set', 410):.0f} N** (stack height on the calibration curve, ± 10 %); the face (outer land r 41–45.5) shows no "
                       f"gap at the rim (dial, ≤ 5 µm) up to **{vb.get('before', {}).get('m_svc', 26.7):.1f} N·m** (the hug row × 1.5), fresh and after 72 h at 50 °C "
                       f"(the model keeps {a_.get('life_lo', 1390) / 1e3:.2f} kN, gap at {a_.get('life_lo', 1390) * a_.get('k_face', 20.6) / 1e3:.0f} N·m)",
                       f"stack height after the hold: ≥ the stop minus {0.5:.1f} mm (the creep budget of the {a_.get('stroke', 2.7):.1f} mm stroke)",
                       f"fault: {vb.get('M_f', 183):.0f} N·m for 10 s (the stall row): no disengagement, no lip crack, the faces re-close",
                       "as designed (D29 wing nut, no stack, full face): record the gap moment fresh at a light hand and after the hold (the model: 13 N·m fresh, ≈ 0 after)",
                       "a timed two-arm join with the lever nut ≤ 30 s (R3)"],
               method="Print `C21_ferrule_male` (with the 0.2 mm land relief) and `C21_ferrule_female` in the arm's PETG; three Ø6 dowels; the clamp with the stack "
                      "and the stop sleeve. Calibrate the stack height against load on the 5:1 lever first. Bolt the female to the stand through its spigot, "
                      "a 1 m lever on the male: the luggage scale gives the moment (1 kg = 9.8 N·m).",
               updates="roll.DETACH (stack, land, band ESTIMATEs); joints.py vband; bom D29 line")
    return [c15, c16, c17, c18, c19, c20, c21]


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
    JC = joint_coupons()
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
    L += ["## Load-chain coupons (C15–C21; Fable #2 majors 4–5, a Step-2 gate)", "",
          "Pass numbers from the load-chain rows of [rnd/joints.md](rnd/joints.md) (`rnd/coupons.py joint_coupons()`).", ""]
    for c in JC:
        L += [f"### {c['id']}: {c['name']} — {c['when']}", "", f"**Proves:** {c['proves']}.", "", "**Pass:**", ""]
        L += [f"- {p}" for p in c["passes"]]
        L += ["", f"**Print and test:** {c['method']}", "", f"**Updates:** {c['updates']}.", "",
              "| Result | |", "|---|---|", "| measured | |", "| pass / fail | |", ""]
    L += ["## Other coupons: fill in the Result column", "", "| # | Measure | Pass if | Updates | Result |", "|---|---|---|---|---|"]
    L += [f"| {c} | {m} | {p} | {u} | |" for c, m, p, u in PROTOCOL]
    L += ["", "## Retired (not printed)", "", "| # | Was | Retired by |", "|---|---|---|"]
    L += [f"| {c} | {w} | {d} |" for c, w, d in RETIRED]
    if QUEUED:
        L += ["", "## Queued, not defined yet (joints.md; a Step-2 gate)", ""] + [f"- {q}" for q in QUEUED]
    L += ["", "**Small parts for the coupons:** C12–C14 use the Step 1a kit's inserts, M4 × 12 + washers + nuts, M3 × 6, Loctite 243, the lever "
          "stock, the luggage scale, the dial test indicator and the 6061 offcut. Others (≈ $15): Ø3 × 8 dowel pins and Ø6 G25 balls (C4, the head "
          "KX44), a 300–400 mm HTD 5M-15 belt (C6), TPU filament (C7).", "",
          "C9 and C14 (a) need the Step-1a GIM6010-8; C14 (c) needs the Step-1b rig."]
    (DOCS / "COUPONS.md").write_text("\n".join(L) + "\n")
    (ROOT.parent / "docs" / "rnd" / "coupons.json").write_text(json.dumps(
        {"hours_p2s": th * 0.35, "grams": tg, "step1": [{"id": c["id"], "name": c["name"], "passes": c["passes"]} for c in S],
         "load_chain": [{"id": c["id"], "name": c["name"], "passes": c["passes"]} for c in JC]}, indent=1))
    print("\n".join(L[:12]))


if __name__ == "__main__":
    main()
