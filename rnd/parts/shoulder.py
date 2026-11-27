"""Shoulder-pitch module v0 (torso side of the arm-detach plane). Left arm; outboard = +Y. Pitch axis = Y at the origin.

- Pitch cartridge M (GIM6010-8) behind the axis (x = -BELT_C), its output facing outboard, bolted to the inner wall
  (D24 v2: 3 × M4 flange ears from the cartridge side into nuts on the wall, side-exit slot; Oldham drive into the D13 small stage).
- HTD 5M 20T → 60T belt (3:1, loop 10) in the belt plane; the 60T is integral with the pitch shaft (D19; Ø40, 2 × 61808
  in the bracket walls, the outer wall a separate bolted print).
- Balancer plane **inboard of the shaft's inboard end** (2026-11-27): an aluminium crank disc bolted to the tube's inboard
  end face carries anchor pin B (b = 20 mm, along the hanging arm); pulley A rides in a vertical slot 40–60 mm above the
  axis between a post on the inner wall and an outer cheek (double shear); the cable runs back over A into a horizontal
  cage with an ISO 10243 Ø25 die spring, 18.7 N/mm (rnd/balancer.py). Nothing coaxial sits in the cable plane, so the
  straight A–B cable never meets the shaft. (Outboard of the tube end the hub → ferrule load path and the harness both
  have to cross the plane on the axis, which the cable sweeps at pitch 0° and 180°: no room for a crank there.)
- Harness: no longer through the pitch-tube bore (the crank closes it). The pitch clock spring sits outboard of the outer
  wall around the pitch hub (static ring on the wall, cavity r 25.5–45 × 12.8 mm); its inner end enters the hub bore
  through a side window and runs to the detach plug on the ferrule.
- Outboard end of the shaft (the bolted pitch hub): the **KX-V ferrule** (3" sanitary profile, 3 dowels): the arm clamps on here with a V-band.
- **C1 (Fable review 2026-11-27): the arm no longer hangs from heat-set inserts.** Four M4 10.9 studs (HUB_TIE) run the
  whole stack, from the aluminium crank (tapped through, the "nut") through the pitch tube's wall and the pitch hub to
  nuts on an aluminium clamp ring recessed in the detach face; since the Neon review each stud runs in steel crush
  sleeves, so the clamp is metal to metal and the creeping print is only sandwiched. Hub and tube are printed parts in compression only; the hub's
  double-D still carries the pitch torque. Capacity vs the basket, hug, 140 N bump and stall wrenches: rnd/joints.py.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley
from ..params import BASELINE as _D

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
EAR_SITE = "pitch4"         # D24 v2 flange-ear pattern + side exit (cartridge_m.EAR_SITES / WALL_EXIT), read by rnd/assembly_seq.py
DRIVE = "direct"            # D26 (2026-11-27): direct stage v4 (belt_stage.small_stage_v4, no Oldham) + back-side slotted idler; read by rnd/assembly_seq.py
MOT = cm.MOTORS[_D.arm.joints["shoulder_pitch"][0]]   # D20 switch (params.ARM_MOTORS): GIM6010-8, or the GIM8108-8 (Ø96 ESTIMATE, Ø50 Oldham, Ø84 flange)
WALL = 1.6
BELT_C = 70.0
# D24 (2026-11-27): the pitch cartridge is bolted (cartridge_m.can_bolted, belt_stage.small_stage_v3). Y_F is now the
# cartridge flange plane = the inboard face of the inner wall. The walls, belt plane and everything outboard stay where
# loop 39 put them; the can rear moves 11 mm outboard (−134.7 → −123.4), away from the lift carriage.
from .belt_stage import V3 as _V3
Y_F = -54.3                # cartridge flange plane (stage z = 0); loop 39's KX74 hub face was at −78
WALL_T = 6.0
W_IN = Y_F - (_V3["jA"][1] + _V3["jA"][0]) / 2 - 0.5               # 6 mm wall, bearing A flush on the pulley side (−54.3 … −48.3)
W_OUT = Y_F - (_V3["jB"][1] + _V3["jB"][0]) / 2 + 0.5              # 6 mm wall, bearing B flush on the pulley side (−20.3 … −14.3)
Y_BELT = Y_F - _V3["belt_c"]                                       # belt plane
Y_DISC = -3.0              # pitch hub shoulder plane (the loop-39 balancer disc; the balancer moved inboard 2026-11-27)
Y_FER = 2.0                # ferrule spigot start; the detach plane (flat face) is at Y_FER + 18 = 20
FERRULE = dict(od=91.0, t=8.0, taper_deg=20.0, dowel_r=30.0)
# D19 (2026-11-27): the 60T is printed in one piece with the pitch tube again. The tube's outboard end (double-D) carries
# the separate pitch hub (balancer disc + lever + pin B + female ferrule), and
# the outer wall is its own print bolted to the web, so: bearing A on the tube's inboard end → tube into the inner wall's
# 61808 → belt → outer wall over bearing B → pitch hub on the double-D.
Y_TUBE_END = Y_DISC - 2.5  # tube end face (−5.5)
# Balancer (2026-11-27): the cable plane sits inboard of the tube's inboard end (y −55.3), past an aluminium crank disc
# (Ø44 × 4, 7075-T6, tapped for the HUB_TIE studs since C1) that carries a Ø6 steel pin B. The cage axis is in the
# cable plane, its tube trimmed to y −75.4 … −48.3 so it stays 3 mm off the belt and ≥ 1 mm off the can sleeve (both motors).
Y_TUBE_IN = W_IN - 4.0     # tube inboard end face (−55.3)
CRANK_T = 4.0
# M3 (Fable, 2026-11-27): the pitch AS5048A board (rnd/parts/encoder.py) sits between the crank's inboard face and the
# cable: 0.5 gap + 0.8 FR4 + 1.2 IC + 0.5 to the cable zone, so the cable plane moved 2.2 → 4.2 mm past the crank face.
# Pin B stands 4.2 mm out: ≤ 374 N cable (M8: 748 N spring / 2:1 tackle; balancer.study Fmax 313 N) × 4.2 mm ≈ 1.6 N·m on the Ø6 pin ≈ 73 MPa (hand
# calc; a hardened ISO 8734 dowel). The cage tube follows the plane inside the same y trim, which opened a 10 mm window
# on its −Y side over the spring. Wave 5: a closing strip (CAGE z_close) shuts it again: the tube's full wall beyond the
# trim, but only above z_close, so the D20 can body (top z ≈ 50.3 under it, from y −76.4) stays ≥ 1 mm away; the full
# tube would come within 0.65 mm of it. The bore is whole again (360°; the wall thins to ≈ 0.7 mm at the window's lower corner).
Y_BAL = Y_TUBE_IN - CRANK_T - 4.2   # cable plane (−63.5): 4.2 mm past the crank face (Ø2 cable: 3.2 mm clear)
PIN_B = (0.0, Y_BAL, -20.0)         # b = 20 mm along the hanging arm (−Z at pitch 0)
CAGE = dict(z=58.0, x0=-15.0, L=135.0, r_out=14.6, r_in=13.0, y_lo=-75.4, y_hi=W_IN + WALL_T / 2, z_close=52.3)   # Ø26 bore for the Ø25 die spring
# M8 (Fable review 2026-11-27): pulley A was a U604ZZ (C0 ≈ 0.3 kN) under up to 748 N. Now a 608-2Z (C0 1.37 kN, SKF) in a
# POM U-groove tyre (PULLEY_A) on a Ø8 steel axle that bears in the post + inner wall (the slot runs through both) and in
# a 7 mm outer cheek; the fork gap is Y_BAL ± 4.5 and the bridge sits higher. Rating table: rnd/balancer.py.
PULLEY_A = dict(brg="608-2Z", d=8.0, D=22.0, B=7.0, od=28.0, w=8.0, r_cable=12.0)
POST_Y = (W_IN - WALL_T / 2, Y_BAL + 4.5)          # wall-side cheek (2.7 mm; the axle also bears in the 6 mm wall) …
CHEEK_Y = (Y_BAL - 11.5, Y_BAL - 4.5)              # … and the outer cheek (7 mm)
PITCH_SPR = (25.5, 45.0)   # pitch clock-spring cavity r_in, r_out (rnd/harness.py reads it)
SPR_Y = (W_OUT + WALL_T / 2, -1.5)                  # cavity y span (12.8 mm; ribbon 9.8 wide): outer wall face → the ring's lip
HUB_Y0 = W_OUT + WALL_T / 2 + 0.5   # pitch hub sleeve starts 0.5 mm outboard of the outer wall (on bearing B's inner race)
# C1 (Fable review 2026-11-27): the pitch hub was pulled onto the tube by 6 × M3 into heat-set inserts in pull-out. Now
# 4 × M4 class 10.9 studs (DIN 976-1, cut to hub_tie_length()) on r 15.5 at 45° (clear of the clock-spring window at 0°
# and of pin B at 270°) run from the crank (7075-T6, tapped M4 through its 4 mm, Loctite 270; stud end flush with its
# inboard face, 2.2 mm off the cable plane) through Ø4.5 holes in the tube wall (r 11–20 over the whole tube) and the hub
# to M4 nuts on a 6061 clamp ring (r 11–21 × 2) recessed in the detach face. Preload 1.5 kN each (≈ 1.3 N·m, Loctite
# 243, ESTIMATE). Capacity vs basket / hug / 140 N bump / stall: rnd/joints.py.
# Neon review 2026-11-27 (finding 3): the printed stack (tube 49.8 + hub 19.8) was the clamped member; with the stud
# stretch (≈ 64 µm) and the print's own squeeze (≈ 0.25 mm) as the springs, a creep coefficient of 2 (40–50 °C, life)
# leaves ≈ 30 % of the preload (rnd/joints.py life_preload). Disc springs in series keep more preload but soften the
# studs, and the studs' stiffness is what carried the bending (rnd/joints.py disc_alternative). Now each stud runs in
# two steel crush sleeves (Ø7 / Ø4.3, turned 42CrMo4+QT: one through the tube wall, one through the hub, meeting at
# the hub ↔ tube face) from the crank to the clamp ring: the 1.5 kN goes crank → sleeves → ring in metal, the print is
# sandwiched with a 0.05 mm squeeze (sleeves that much shorter) and is no longer the spring. The no-gap moment is then the
# metal stack's. Crank: the 4 mm thread with its chamfers and the flush stud end (≈ 2.95 mm effective) strips at
# ≈ 7.6 kN in 7075-T6 — the row in rnd/joints.py says whether that is ≥ 1.5× the fault stud force (crank_mat).
HUB_TIE = dict(n=4, r=15.5, ang0=45.0, d=4.0, pitch=0.7, As=8.78, hole=4.5, grade="10.9", proof=830.0, preload=1500.0,
               ring=(11.0, 21.0, 2.0), nut_h=3.2, face_gap=0.5, tube_r_in=11.0, crank_t=4.0, crank_mat="7075-T6", crank_density=2.81e-6,
               sleeve=(7.0, 4.3), sleeve_hole=7.1, squeeze=0.05, bell="M4", n_bell=0)


def _stack_h(size, n, F):
    """Loaded height (mm) of n DIN 6796 discs in series at F (h max − deflection; rnd/joints.py Disc)."""
    from ..joints import DIN6796, Disc
    return round(n * (DIN6796[size]["h"] - Disc(size).s(F)), 2)


HUB_TIE["stack_h"] = _stack_h(HUB_TIE["bell"], HUB_TIE["n_bell"], HUB_TIE["preload"])
HUB_TIE["recess"] = HUB_TIE["ring"][2] + HUB_TIE["stack_h"] + HUB_TIE["nut_h"] + HUB_TIE["face_gap"]        # 5.7 mm below the detach face
HUB_SCREW_R = HUB_TIE["r"]
WEB_X = -BELT_C / 2 - 10 - 190.0 / 2 + 4                           # back-web centre (x)


def _rot_y(p):            # build along Z, then lay along +Y
    return Rot(-90, 0, 0) * p


def ferrule(male: bool = True) -> Part:
    """3"-style sanitary ferrule flange (printed). Profile ESTIMATE until checked against a real clamp."""
    f = FERRULE
    t_flat = 3.0
    p = Cylinder(f["od"] / 2, t_flat, align=MIN)
    # 20° taper on the back face (where the clamp wedges): approximated by a cone frustum
    from build123d import Cone
    p += Pos(0, 0, t_flat) * Cone(f["od"] / 2, f["od"] / 2 - (f["t"] - t_flat) * math.tan(math.radians(90 - f["taper_deg"])) * 0.35,
                                  f["t"] - t_flat, align=MIN)
    p += Pos(0, 0, f["t"]) * Cylinder(32, 10, align=MIN)                   # spigot toward the shaft / roll link
    for k in range(3):
        a = math.radians(90 + 120 * k)
        hole = Pos(f["dowel_r"] * math.cos(a), f["dowel_r"] * math.sin(a), 0) * Cylinder(3.0 - (0.05 if male else -0.05), 12, align=MIN)
        p -= hole
    return p - Cylinder(15, 40, align=MIN)


def _tie_angles():
    """HUB_TIE stud angles (rad) in the tube's build frame (module x = x, module z = −y); the pattern is symmetric."""
    return [math.radians(HUB_TIE["ang0"] + 360.0 * k / HUB_TIE["n"]) for k in range(HUB_TIE["n"])]


def pitch_shaft() -> Part:
    """D19 (2026-11-27, Astra review #2b finding 1): Ø40 tube with the 60T printed on it in one piece (no keyed joint). Its
    inboard end is plain (bearing A slides on from there); its outboard end is a double-D (36 AF, 8 mm). C1 (2026-11-27):
    the wall is r 11–20 over the whole length and carries the 4 HUB_TIE stud holes (no heat-set inserts at either end),
    so the tube is clamped in compression between the crank and the pitch hub. Loop 40's split 60T on a double-D could not
    be fitted (the Ø40 tube can't pass its 36.4 AF bore)."""
    from .belt_stage import dd_flats, PITCH, PLD, DEPTH
    y0, y1 = W_IN - 4, Y_TUBE_END
    L = y1 - y0
    ri = HUB_TIE["tube_r_in"]
    t = Cylinder(20, L, align=MIN) - Cylinder(ri, L, align=MIN)
    p60 = pulley(60, 16, flange=False)
    r_rim = 60 * PITCH / math.pi / 2 - PLD - DEPTH - 5.0                   # 5 mm web between a Ø56 hub and a 5 mm rim
    for z in (0.0, 10.5):
        p60 -= Pos(0, 0, z) * (Cylinder(r_rim, 5.5, align=MIN) - Cylinder(28, 5.5, align=MIN))
    t += Pos(0, 0, Y_BELT - 8 - y0) * (p60 - Cylinder(ri, 16, align=MIN))
    for a in _tie_angles():
        t -= Pos(HUB_TIE["r"] * math.cos(a), HUB_TIE["r"] * math.sin(a), 0) * Cylinder(HUB_TIE["sleeve_hole"] / 2, L, align=MIN)   # Neon 3: crush sleeve
    t -= dd_flats(HUB_Y0 - y0, y1 - HUB_Y0 + 1)
    return Pos(0, y0, 0) * _rot_y(t)


def pitch_hub() -> Part:
    """D19: clock-spring hub sleeve + female KX-V ferrule in one print, on the tube's double-D (torque). C1 (2026-11-27):
    clamped onto the tube end by the 4 HUB_TIE studs, their nuts on a ring in a 5.7 mm recess (r 11–21; crush sleeves through the hub) in the
    detach face, so the hub is in compression between the ring and the tube end; no heat-set inserts. The sleeve's
    inboard end holds bearing B's inner race. Double-D bearing stress ≈ 8 MPa at the 21 N·m basket hold, ≈ 26 MPa at a 66 N·m fault
    peak (hand calc, ESTIMATE; coupon with the C-series)."""
    from .belt_stage import dd_prism, DD_AF
    h = Y_TUBE_END - HUB_Y0
    sl = Cylinder(25, h, align=MIN) - dd_prism(h, DD_AF + 0.4, 20.2)
    s = Pos(0, HUB_Y0, 0) * _rot_y(sl)
    # 2026-11-27: the balancer disc and pin B moved to the inboard crank; here an r 25 sleeve (the clock spring's rotating
    # hub) runs to the ferrule, with a 7 × 12 side window for the ribbon's inner end into the Ø24 bore
    s += Pos(0, Y_TUBE_END, 0) * _rot_y(Cylinder(25, Y_FER - Y_TUBE_END, align=MIN) - Cylinder(12, 20, align=MIN))
    s -= Pos(19, (Y_TUBE_END + Y_FER - 0.5) / 2, 0) * Box(16, Y_FER - 0.5 - Y_TUBE_END, 12)
    fer = ferrule(male=False) + (Cylinder(15.05, 18, align=MIN) - Cylinder(12, 18, align=MIN))     # Ø24 bore here
    s += Pos(0, Y_FER + 18, 0) * Rot(90, 0, 0) * fer                       # flat face outboard at the detach plane
    y_face = Y_FER + 18
    r0, r1, _t = HUB_TIE["ring"]
    s -= Pos(0, y_face - HUB_TIE["recess"], 0) * _rot_y(Cylinder(r1 + 0.2, HUB_TIE["recess"] + 1, align=MIN) - Cylinder(r0 - 0.2, HUB_TIE["recess"] + 1, align=MIN))
    for a in _tie_angles():
        x, z = HUB_TIE["r"] * math.cos(a), HUB_TIE["r"] * math.sin(a)
        s -= Pos(x, Y_TUBE_END, z) * _rot_y(Cylinder(HUB_TIE["sleeve_hole"] / 2, y_face - Y_TUBE_END, align=MIN))   # Neon 3: crush sleeve
    return s


def hub_clamp_ring() -> Part:
    """C1: clamp ring (6061-T6 plate, laser/waterjet cut, r 11–21 × 2, 4 × Ø4.5) under the HUB_TIE nuts, in the detach-face
    recess: spreads the 4 × 1.5 kN over ≈ 1.0e3 mm² of the printed hub (≈ 6 MPa)."""
    r0, r1, t = HUB_TIE["ring"]
    y = Y_FER + 18 - HUB_TIE["recess"]
    p = Pos(0, y, 0) * _rot_y(Cylinder(r1, t, align=MIN) - Cylinder(r0, t, align=MIN))
    for a in _tie_angles():
        p -= Pos(HUB_TIE["r"] * math.cos(a), y, HUB_TIE["r"] * math.sin(a)) * _rot_y(Cylinder(HUB_TIE["hole"] / 2, t, align=MIN))
    return p


def hub_print_under_ring() -> float:
    """Printed length (mm) of the pitch hub between the tube end and the clamp ring's seat (part of the clamped stack)."""
    return round(Y_FER + 18 - HUB_TIE["recess"] - Y_TUBE_END, 2)


def hub_tie_length() -> float:
    """HUB_TIE stud length (mm): flush with the crank's inboard face → the top of the nut (face_gap under the detach face)."""
    return round((Y_FER + 18 - HUB_TIE["face_gap"]) - (Y_TUBE_IN - CRANK_T), 1)


_ST = Pos(-BELT_C, Y_F, 0) * Rot(90, 0, 0)          # stage frame → module frame (stage +Z → −Y, hub face at Y_F, axis at x = −BELT_C)


def idler():
    """D26: the pitch belt's back-side slotted idler (rnd/belts.py IDLERS["shoulder_pitch"]: on the stage +Y = upper strand,
    0.45 of the way to the 60T; the lower side is the cartridge's exit path through wall A's U-slot). Stage frame."""
    from .. import belts as _b
    from .belt_stage import PITCH as _P
    sp = dict(_b.IDLERS["shoulder_pitch"])
    if MOT is cm.MOTOR_8108:                               # D20: PU steel-cord belt (cartridge_m.parts_list), EA ≈ 150 kN (ESTIMATE)
        sp["EA"] = 150e3
    g = _b.idler_geom("shoulder_pitch", 20, 60, BELT_C, _b.STOCK_TEETH["shoulder_pitch"], MOT_PEAK * 1000 / (20 * _P / math.pi / 2), sp)
    g["wall_b_out"], g["axle_top"] = -40.0, -2.0          # M5 × 40 cut to 38: tip 0.3 mm short of wall A's blind-slot bottom (z −1.7)
    return g


MOT_PEAK = 22.0 if MOT is cm.MOTOR_8108 else 11.0     # actuator peak (rnd/specs/actuators.yaml), for the idler's preload / loads
_XS, _ZS = 190.0, 150.0
_X0 = -BELT_C / 2 - 10
WEB_INSERTS_Z = (-50.0, 10.0, 70.0)                 # 3 × M4 (outer wall → web end face)
# wall lightening windows (x, z, lx, lz). Loop 8 diet had (-75, 62, 96, 22) and (-62, -52, 110, 18) as well; R2b last mile
# (2026-11-27, P2 FEA of the bracket case): closing them takes the hand term 1.54 → 0.84 mm for +40 g
LIGHTEN = ((-128, 0, 14, 60),)
LIGHTEN_OUT = LIGHTEN


def _wall(y: float, inner: bool) -> Part:
    w = Pos(_X0, y, 10) * Rot(-90, 0, 0) * fillet(Box(_XS, _ZS, WALL_T).edges().filter_by(Axis.Z), 20)
    w -= Pos(0, y, 0) * Rot(-90, 0, 0) * Cylinder(26, WALL_T + 1)                      # 61808 seat (OD 52)
    from .belt_stage import idler_cuts, idler_nut_pocket
    g = idler()
    if inner:
        from .belt_stage import wall_a_cuts_v4
        w -= _ST * wall_a_cuts_v4(MOT, EAR_SITE)   # D26: Ø40.6 pulley hole + U-slot exit downward, spigot counterbore, ear bolts + nut pockets
        w -= _ST * idler_cuts(g, -6.5, -1.7)   # D26: blind slot for the idler axle's tip (pulley side; the cartridge face stays whole)
    else:
        w -= Pos(-BELT_C, y, 0) * Rot(-90, 0, 0) * Cylinder(19.1, WALL_T + 1)       # Ø38.2 seat: the bonded 6001 bushing (D26)
        from .belt_stage import wall_b_cuts
        w -= _ST * wall_b_cuts(MOT, EAR_SITE)   # Neon #2 F2: the pitch4 rear ear pair is screwed from the pulley side through the outer wall (Ø10 driver holes)
        w -= _ST * (idler_cuts(g, -40.5, -33.5) + idler_nut_pocket(g))  # D26: idler axle slot (head + jack screw outboard) + captive nut pocket (pulley side)
    # lightening (loop 8 diet): keep ≥ 10 mm of material around both bearing seats and along the back web
    for cx, cz, lx, lz in (LIGHTEN if inner else LIGHTEN_OUT):
        w -= Pos(cx, y, cz) * Rot(-90, 0, 0) * fillet(Box(lx, lz, WALL_T + 1).edges().filter_by(Axis.Z), 5)
    return w


def bracket_body() -> Part:
    """D19: inner wall (pitch bearing A, the bolted pitch cartridge) + back web, one print. Loop 21's torque fork is gone:
    the D24 flange carries the cartridge torque into the inner wall."""
    p = _wall(W_IN, True)
    p += balancer_mount()
    from .encoder import pitch_post
    p += pitch_post()                                                                   # M3: post for the pitch AS5048A strip
    y1 = W_OUT - WALL_T / 2
    p += Pos(WEB_X, (W_IN + y1) / 2, 10) * Box(8, y1 - W_IN, _ZS)
    for z in WEB_INSERTS_Z:
        p -= Pos(WEB_X, y1, z) * Rot(90, 0, 0) * Cylinder(2.8, 8, align=MIN)           # M4 heat-set inserts in the web end
    return p


def bracket_wall_out() -> Part:
    """D19: the outer wall is its own print (pitch bearing B, stage bearing B, balancer post + spring cage), bolted to the
    web end with 3 × M4, so the one-piece pitch shaft can go in before it."""
    p = _wall(W_OUT, False)
    for z in WEB_INSERTS_Z:
        p -= Pos(WEB_X, W_OUT, z) * Rot(90, 0, 0) * Cylinder(2.25, 20)
    return p


def balancer_mount() -> Part:
    """2026-11-27: pulley A's fork (wall-side cheek + outer cheek + top bridge, vertical slot a = 38–62 mm) and the spring
    cage, on the inboard face of the inner wall (part of the bracket-body print). The wall's lightening hole is filled
    where the cage passes, and the cage bore is cut through everything."""
    c = CAGE
    p = Part()
    for y0, y1 in (POST_Y, CHEEK_Y):
        y0, y1 = min(y0, y1), max(y0, y1)
        ch = Pos(0, (y0 + y1) / 2, 60) * Box(16, y1 - y0, 44)
        ch -= Pos(0, (y0 + y1) / 2, 50) * Box(PULLEY_A["d"] + 0.2, 40, 24)               # vertical slot: a = 38–62 mm (Ø8 axle; through the wall too)
        p += ch
    yb0, yb1 = CHEEK_Y[0], POST_Y[0]
    p += Pos(0, (yb0 + yb1) / 2, 80.5) * Box(16, yb1 - yb0, 7)                          # bridge over pulley A (M8: Ø28 at a = 60 → top 74)
    tube = Pos(c["x0"], 0, c["z"]) * Rot(0, -90, 0) * Cylinder(c["r_out"], c["L"], align=MIN)
    tube = tube.translate((0, Y_BAL, 0))
    y_out = Y_BAL - c["r_out"] - 0.5
    p += tube & Pos(0, (c["y_lo"] + c["y_hi"]) / 2, 0) * Box(400, c["y_hi"] - c["y_lo"], 400)
    p += tube & Pos(0, (y_out + c["y_lo"]) / 2, c["z_close"]) * Box(400, c["y_lo"] - y_out, 100, align=(Align.CENTER, Align.CENTER, Align.MIN))   # wave 5: closing strip
    p += Pos(c["x0"] - c["L"] / 2, W_IN, c["z"]) * Box(c["L"], WALL_T, 2 * c["r_out"])  # fill the lightening hole under the cage
    p -= Pos(c["x0"] + 1, Y_BAL, c["z"]) * Rot(0, -90, 0) * Cylinder(c["r_in"], c["L"] + 2, align=MIN)
    return p


def crank() -> Part:
    """Balancer crank (HUB_TIE crank_mat, 7075-T6, ESTIMATE): Ø44 × 4 disc on the tube's inboard end face, tapped M4 through for the 4
    HUB_TIE studs (C1: it is the "nut" of the pitch-hub clamp; the studs end flush with its inboard face), Ø6 steel pin B
    pressed in at b = 20 mm along the hanging arm, standing 5.2 mm into the cable plane. It also holds bearing A's inner
    race. Pin moment ≈ 374 N × 4.2 mm ≈ 1.6 N·m (hand calc, M3 cable plane); the AS5048A magnet sits flush in a Ø6.1 × 2.5
    pocket at the centre of the inboard face (rnd/parts/encoder.py). The balancer torque (K 4.5 N·m) reaches the tube through
    the clamp friction and the studs in shear (≈ 73 N each)."""
    d = Pos(0, Y_TUBE_IN, 0) * Rot(90, 0, 0) * Cylinder(22, CRANK_T, align=MIN)          # y −55.3 … −59.3
    for a in _tie_angles():
        x, z = HUB_TIE["r"] * math.cos(a), HUB_TIE["r"] * math.sin(a)
        d -= Pos(x, Y_TUBE_IN, z) * Rot(90, 0, 0) * Cylinder(1.65, CRANK_T, align=MIN)  # M4 tapped through (Ø3.3 drill)
    d += Pos(PIN_B[0], Y_TUBE_IN - CRANK_T + 1.0, PIN_B[2]) * Rot(90, 0, 0) * Cylinder(3.0, Y_TUBE_IN - CRANK_T + 1.0 - (Y_BAL - 3.0), align=MIN)
    from .encoder import pitch_magnet_pocket
    return d - pitch_magnet_pocket()


def spring_ring() -> Part:
    """Pitch clock spring, static side (printed, screwed to the outer wall's outboard face): ring r 45–46.5 over the cavity
    SPR_Y and a lip r 30–46.5 on its outboard end; the rotating hub is the pitch hub's r 25 sleeve."""
    r_in, r_out = PITCH_SPR
    y0, y1 = SPR_Y
    p = Pos(0, y0, 0) * _rot_y(Cylinder(r_out + 1.5, y1 - y0, align=MIN) - Cylinder(r_out, y1 - y0, align=MIN))
    p += Pos(0, y1, 0) * _rot_y(Cylinder(r_out + 1.5, 1.0, align=MIN) - Cylinder(30.0, 1.0, align=MIN))
    return p


def bracket() -> Part:
    """The assembled bracket (body + bolted outer wall) for the FEA and stiffness cases; the prints are bracket_body() and
    bracket_wall_out()."""
    return bracket_body() + bracket_wall_out()


def pulley_a() -> Part:
    """M8: 608-2Z in a POM U-groove tyre (Ø28 × 8, cable centre r 12) on a Ø8 axle, centred on the cable plane (a = 48)."""
    pa = PULLEY_A
    return Pos(0, Y_BAL, 48) * Rot(-90, 0, 0) * (Cylinder(pa["od"] / 2, pa["w"]) - Cylinder(pa["d"] / 2, pa["w"]))


def module(with_motor: bool = True):
    """(name, part, colour) list in the module frame. Stage frame (belt_stage.small_stage_v3): flange plane z = 0,
    cartridge +Z; mapped so stage +Z → world −Y, flange plane at Y_F, axis at x = −BELT_C."""
    from .belt_stage import small_stage_v4, bolted_cartridge
    from . import encoder as _enc
    out = [("bracket", bracket_body(), "white"), ("bracket_wall", bracket_wall_out(), "white"), ("shaft", pitch_shaft(), "orange"),
           ("pitch_hub", pitch_hub(), "orange"), ("pitch_hub_crank", crank() + _enc.pitch_magnet(), "steel"),   # M3: magnet bonded in the crank ("pitch_hub_collar", hub_clamp_ring(), "steel"),   # name: a "collar"/"hub" mate in rnd.spacing
           ("pulleyA", pulley_a(), "steel"),
           ("bracket_wall_spring_ring", spring_ring(), "dark")]   # screwed to the outer wall (a mate for rnd.spacing)
    out += [(n, _ST * pp, c) for n, pp, c in small_stage_v4(20, MOT, idler())]       # D26: direct stage (stub on the motor, 6001 in the outer wall) + idler
    out += _enc.pitch_parts()                                                          # M3: AS5048A strip board ("enc_pcb")
    if with_motor:
        out += [(n, _ST * pp, c) for n, pp, c in bolted_cartridge(MOT, EAR_SITE)]
    return out


def masses(density=1.27):
    """D24/D19 roll-up (2026-11-27). The bolted cartridge's sleeve is counted here only by what it adds over the loop-10 can
    (nose, flange, rear cap); the motor is the actuator mass."""
    from .belt_stage import small_stage_v4
    st = dict((n, p) for n, p, _ in small_stage_v4(20, MOT, idler()))
    # D26: the turned C45 stub (flange + shaft) and the idler (2 × 625 + printed roller + M5 axle) replace the Oldham drive
    # (the stage shaft was never counted here; the stub's shaft is 11.6 mm longer than the v3 shaft: + 10 g)
    oldham = (st["kx_hub_v2"].volume * 7.85e-3 + 10.0 + st["idler_roller"].volume * 1.27e-3 * 0.7 + st["idler_axle"].volume * 7.85e-3
              + 2 * 4.0)   # g (625-2RS ≈ 4 g each)
    can_extra = (cm.can_bolted(MOT, EAR_SITE).volume + cm.can_cap_bolted(MOT).volume - cm.can().volume) * density * 0.8e-3
    m = {"bracket body + bolted outer wall (walls, web, cage)": bracket().volume / 1e6 * density * 0.6,
         "pitch shaft + integral 60T + pitch hub (ferrule) + clock-spring ring": (pitch_shaft().volume + pitch_hub().volume + spring_ring().volume) / 1e6 * density * 0.7,
         "balancer crank (HUB_TIE crank_mat) + Ø6 pin B": crank().volume * HUB_TIE["crank_density"],
         "C1 hub tie: 4 × M4 10.9 studs + nuts + 8 crush sleeves (steel) + 6061 clamp ring": (4 * 8.78 * hub_tie_length() + 4 * 100.0) * 7.85e-6
         + hub_clamp_ring().volume * 2.7e-6 + HUB_TIE["n"] * math.pi / 4 * (HUB_TIE["sleeve"][0] ** 2 - HUB_TIE["sleeve"][1] ** 2) * (Y_TUBE_END - Y_TUBE_IN + hub_print_under_ring()) * 7.85e-6,
         "D26 drive stub (C45) + idler + can nose/flange/cap": (oldham + can_extra) / 1e3,
         "bearings 2×61808 + 2×61804 (ESTIMATE)": 0.056,
         "spring + cable + 2 × (608-2Z + POM tyre) + Ø8 axles (ESTIMATE; M8: was U604, 0.09)": 0.11,
         "V-band clamp, stainless 3\" (ESTIMATE)": 0.11,
         "screws: 3 × M4 ear bolts + nuts, 3 × M4 wall, inserts (ESTIMATE)": 0.010}
    return m
