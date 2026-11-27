"""Mast, lift, torso and head v0 (robot frame: x forward, z up; mast axis at x = MAST_X).

- Mast: 60×60×2 Al square tube from the base coupling to the head bracket (rnd/mast.py).
- Lift (v2, see the lift section): bolted GIM cartridge on a D13 stage (Ø12 shaft, 2 × 6801, 20T aluminium pulley)
  in a printed bracket behind the mast coupling, axis along −X; the belt loops over an idler in the top bracket and
  clamps to the carriage. Steel rack (A15) on the mast's back face; spring-applied steel pawl + 12 V solenoid on the
  carriage (rnd/lift.py).
- Carriage: printed sleeve with 8 × 625 bearings on the tube faces; the torso shell and both pitch modules bolt to it.
- Torso: two-piece shell with a chest light; electronics tray (Pi 5, CAN, DC-DC, fuses) on the back; e-stop on top.
- Head (fixed on the mast top, D8): top bracket (idler + carry handle) → pan cartridge S (vertical) → yoke →
  tilt cartridge S → head shell with a dark visor, eye lights and the RGB-D camera.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from ..params import BASELINE
from . import cartridge_m as cm, cartridge_s as cs
from .soft import soft_box, soft_shell

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
D = BASELINE
MAST_X = -60.0
W = D.upper.mast_w * 1000
Z_BASE = (D.base.clearance + D.base.height) * 1000
Z_TOP = Z_BASE + D.upper.spine_len * 1000
HEAD_Z = Z_TOP + 10.0
WALL = 2.0
PAN_RANGE = (-90.0, 90.0)       # head pan (deg): software limit for the neck cable loop (ESTIMATE)
TILT_RANGE = (-10.0, 15.0)      # head tilt (deg), D22 pivot: −10° (up) … +15° (down to the table). D25 (owner, 2026-11-27): was +30°.
# Wave 7 check: the 4 tier-1 tilt fails (head_shell × yoke 3 mm at the neck-opening rim, and 3 rows against the tilt sleeve
# inside the head, reached through the opening) did not depend on the upper limit. Neck v2 (below, R17 neck CAD pass)
# closes them: the rim is a surface of revolution about the tilt axis over a concentric saddle (constant 4 mm) and the
# head's interior is out of reach.
TILT_RELIEF = (TILT_RANGE[0] - 2.5, TILT_RANGE[1] + 2.5)   # the shell's clearance cuts (yoke, tilt sleeve, pan sleeve) are swept over this (head())
PAN_RISER = 20.0                # R17 (2026-11-27): the yoke stands on a Ø40 hub this tall, so the yoke plate and the head clear
                                # the pan sleeve and the carry handle by ≥ 25 mm at every pan angle (were 10.6 / 15.2 / 17.4)


def _rbox(x, y, z, r):
    r = min(r, x / 2 - 0.3, y / 2 - 0.3)
    return fillet(Box(x, y, z).edges().filter_by(Axis.Z), r) if r > 0.5 else Box(x, y, z)


def _shell(x, y, z, r, t=WALL):
    return _rbox(x, y, z, r) - _rbox(x - 2 * t, y - 2 * t, z - 2 * t, max(r - t, 1))


def mast() -> Part:
    L = Z_TOP - Z_BASE
    return Pos(MAST_X, 0, Z_BASE) * (Box(W, W, L, align=MIN) - Box(W - 4, W - 4, L, align=MIN))


# ---------------------------------------------------------------------------------------------------------------------
# Lift v2 (2026-11-27, queue item 2 + the R15/R17 lift-bottom clash). The GIM cartridge used to sit on a KX74 seat with
# the 20T straight on it and the belt pull on the motor bearing, and the drive + its cover stood 175 mm tall behind the
# mast, so below lift ≈ 0.46 m the torso, carriage, collars and spine pod ran into it. Now:
# - the lift is a D13/D24 stage (belt_stage.bolted_stage): bolted cartridge → Oldham → Ø12 shaft in 2 × 6801 → 20T
#   aluminium pulley, between two walls of one printed lift bracket that stands on the deck behind the mast coupling;
# - the cartridge axis runs along −X (pointing back, away from the mast), so the belt runs in the YZ plane and the motor
#   sits low (axis 44.5 mm above the deck) and behind everything that rides the lift;
# - the closed cover steps down over the pulley (P zone) and rises over the cartridge only behind x = −146 (C zone),
#   with a chimney round the belt up to LIFT_COVER_TOP, where the static belt duct (covers.belt_duct) stands;
# - A15: the printed ratchet strip becomes a bolted steel rack (module 1, 10 × 10) and the carriage carries a hardened
#   steel pawl in a pocket of its pawl box.
# At lift_min (0.35 m) everything that rides the lift stays ≥ 25 mm above the static lift parts (R17 finger rule) with
# the torso bottom at lift − 75 (it was lift − 105): D23 (3) asked for ≥ 120 mm under the torso, 155 → 275 mm = 120.
LIFT_PD = 20 * 5.0 / math.pi                 # 20T HTD 5M pitch diameter (31.83 mm)
EAR_SITE = "lift4"                           # D24 v2 flange-ear pattern (cartridge_m.EAR_SITES), read by rnd/assembly_seq.py; W8b (Neon #2 finding 2): 4 ears, ±40° + ±140°
LIFT_XF = -151.0                             # cartridge flange plane = wall A's cartridge-side face (stage z = 0)
LIFT_Y = -2.0 - LIFT_PD / 2                  # stage axis y: the belt's rising strand leaves the pulley at y = −2
LIFT_Z = 199.5                               # stage axis height: can bottom 1.2 mm over the deck (Z_BASE 155)
LIFT_WALL_TOP = 226.0                        # both bracket walls (bearing seats Ø38.2: 7.4 mm of wall above)
LIFT_COVER_TOP = 247.0                       # chimney top = belt duct foot; ≥ 25 mm under the collars at lift_min
BELT_T, BELT_W = 3.6, 15.0                   # HTD 5M, 15 mm wide


def lift_stage_T():
    """Stage frame (belt_stage.small_stage_v3: flange plane z = 0, cartridge +Z; D24 v2 ears at ±140° from +X) → world:
    stage +Z → world −X (cartridge points back), stage +X → world +Z (the belt pull is up, between the ±75° bolts)."""
    return Pos(LIFT_XF, LIFT_Y, LIFT_Z) * Rot(0, -90, 0)


def lift_belt_xy():
    """Rising strand: centre x, centre y, size along x (belt width), size along y (thickness)."""
    from .belt_stage import V3
    return LIFT_XF - V3["belt_c"], -2.0, BELT_W, BELT_T


RACK = dict(m=1.0, w=10.0, h=10.0, y=23.0, segs=(500.0, 300.0), z0=Z_BASE + 140, fill=(Z_BASE + 45, Z_TOP - 35))   # A15 rack: two lengths (500 + one cut to 300 from a 2nd 500), ESTIMATE source: BOM line


def ratchet_strip(fillers: bool = True) -> Part:
    """A15: steel rack (module 1, 10 × 10) bolted to the mast's back face (M4 into rivet nuts every 100 mm, ESTIMATE),
    z 295 … 1095: the pawl (lift − 30) meets it from lift_min to lift_max. Teeth not modelled (m 1: π mm pitch). Name kept
    for callers; it is no longer printed."""
    p, z = None, RACK["z0"]
    for L in RACK["segs"]:
        s = Pos(MAST_X - W / 2 - RACK["h"] / 2, RACK["y"], z) * Box(RACK["h"], RACK["w"], L - 0.5, align=MIN)
        p = s if p is None else p + s
        z += L
    if fillers:                                                       # R17: printed 10 × 10 fillers carry the profile on to
        for z0, z1 in ((RACK["fill"][0], RACK["z0"] - 0.5), (z, RACK["fill"][1])):     # where the collars and carriage travel,
            p += Pos(MAST_X - W / 2 - RACK["h"] / 2, RACK["y"], z0) * Box(RACK["h"], RACK["w"], z1 - z0, align=MIN)   # so their gaps stay constant
    return p


def lift_pawl(lift_mm: float) -> Part:
    """A15: hardened steel pawl (6 mm plate, 26 mm long) on a Ø4 pin in the carriage's pawl box, tilted 20°, its tip
    1.2 mm off the rack's tooth tips as drawn (spring-applied into the teeth, the 12 V solenoid pulls it out)."""
    return Pos(-108.0, RACK["y"], lift_mm - 30) * Rot(0, 20, 0) * (Box(5, 6, 26) - Pos(0, 0, 9) * Rot(90, 0, 0) * Cylinder(2.1, 8))


def lift_mount(*_):
    """One printed bracket (PETG-CF, ≈ 74 × 99 × 71 mm) on the deck behind the mast coupling: wall A (cartridge flange,
    2 × M4 ear bolts + nut pockets, eccentric seat for 6801 A: belt_stage.wall_a_cuts), wall B (eccentric seat for 6801 B), a web on
    the −Y side between them (clear of both strands) and a deck foot under the can's nose, 4 × M4 into deck inserts."""
    from .belt_stage import V3, ear_tower, wall_a_cuts
    T = lift_stage_T()
    xa0, xa1 = LIFT_XF, LIFT_XF + 6.0                                          # wall A (stage z −6 … 0)
    xb0, xb1 = LIFT_XF - V3["end"] - 6.0, LIFT_XF - V3["end"]                   # wall B (stage z −40 … −34)
    ya0, ya1 = LIFT_Y - 41.0, LIFT_Y + 41.0                                     # D24 v2: round the ±140° flange ears' nut pockets (r 48, y ±30.9 + 7.5)
    yb0, yb1 = LIFT_Y - 30.0, LIFT_Y + 24.0
    wa = Pos((xa0 + xa1) / 2, (ya0 + ya1) / 2, Z_BASE) * Box(6, ya1 - ya0, LIFT_WALL_TOP - Z_BASE, align=MIN)
    wb = Pos((xb0 + xb1) / 2, (yb0 + yb1) / 2, Z_BASE) * Box(6, yb1 - yb0, LIFT_WALL_TOP - Z_BASE, align=MIN)
    web = Pos((xa1 + xb0) / 2, yb0 + 2.5, Z_BASE) * Box(xb0 - xa1, 5, LIFT_WALL_TOP - Z_BASE, align=MIN)
    foot = Pos((xa0 + xb1) / 2, (ya0 + ya1) / 2, Z_BASE) * Box(xb1 - xa0, ya1 - ya0, 5, align=MIN)           # in front of the can (its belly is 1.2 mm off the deck)
    m = wa + wb + web + foot
    m += T * ear_tower(None, EAR_SITE, (40.0, -40.0))                           # W8b lift4: the ±40° ears stand 18 mm over LIFT_WALL_TOP
    m -= T * wall_a_cuts(None, EAR_SITE)                                         # D24 v2 (W8b lift4): 4 ears at ±40° / ±140° (cartridge_m.EAR_SITES)
    m -= Pos(xb0 - 1, LIFT_Y, LIFT_Z) * Rot(0, 90, 0) * Cylinder(19.1, 8, align=MIN)          # wall B eccentric seat (Ø38.2)
    for x in (xa1 + 4, xb0 - 4):                                                # 4 × M4 inserts, screws from under the deck
        for y in (ya0 + 6, ya1 - 6):
            m -= Pos(x, y, Z_BASE) * Cylinder(2.25, 12)
    return m


def lift_drive():
    from .belt_stage import bolted_stage
    xb, yb, wx, ty = lift_belt_xy()
    out = bolted_stage(lift_stage_T(), 20, "lift_", site=EAR_SITE)
    out += [("lift_belt", Pos(xb, yb, LIFT_Z) * Box(wx, ty, Z_TOP - 10 - LIFT_Z, align=MIN), "dark"),
            ("lift_bracket", lift_mount(), "white")]
    return out


def lift_cover() -> Part:
    """R17 v4: closed PETG cover round the lift stage, one print (≈ 117 × 99 × 89 mm), 3 mm off the deck and 3 mm round
    the mast coupling, 4 × M3 into deck inserts. C zone round the cartridge (behind wall A, top 3.7 mm over the can),
    P zone round the pulley bay (top 5 mm over the walls), a chimney round the rising strand (belt + 3 mm) up to
    LIFT_COVER_TOP where the belt duct stands on it."""
    from .base import coupling
    t = 2.2
    xb, yb, wx, ty = lift_belt_xy()
    from . import cartridge_m as cm
    can_r, can_back = cm.can_r_out(), LIFT_XF - cm.bolted_levels()["rear"]       # bolted can Ø86.6, rear cap at stage z 69.1
    lug_r, lug_h, lug_w = can_r + cm.CAP["lug_out"], cm.CAP["lug_h"], cm.CAP["ear_w"] + 0.4   # D24 v2 rear-cap lugs at stage ±90° = world ±Y
    y0, y1 = LIFT_Y - can_r - 3.0 - t, LIFT_Y + can_r + 3.0 + t + 2.0           # +Y side 2 mm wider: over the deck, not the wheel cut-out
    zc_top = LIFT_Z + can_r + 1.5 + t
    zp_top = LIFT_WALL_TOP + 2.8 + t
    xc0, xc1 = can_back - 3.5 - t, LIFT_XF + 5.2                                 # C zone: x … riser at LIFT_XF + 3 … + 5.2
    xp0, xp1 = LIFT_XF - 2.0, LIFT_XF + 40.0 + 1.3 + t                             # P zone: 1.3 mm + wall past wall B (cut round the coupling)
    ix, iy = wx + 8 + 2 * 2.2, ty + 8 + 2 * 2.2                                 # duct outer footprint (covers.belt_duct)
    zb = Z_BASE + 3.0

    def box(x0, x1, y0_, y1_, z0, z1, r=10, re=6):
        return Pos((x0 + x1) / 2, (y0_ + y1_) / 2, (z0 + z1) / 2) * soft_box(x1 - x0, y1_ - y0_, z1 - z0, r, re)

    outer = box(xc0, xc1, y0, y1, zb - 20, zc_top) + box(xp0, xp1, y0, y1, zb - 20, zp_top)
    outer += Pos(xb, yb, zp_top - 4) * Box(ix, iy, LIFT_COVER_TOP - zp_top + 4, align=MIN)            # chimney
    inner = (box(xc0 + t, xc1 - t, y0 + t, y1 - t, zb - 30, zc_top - t, 10 - t, 6 - t)
             + box(xp0 + t, xp1 - t, y0 + t, y1 - t, zb - 30, zp_top - t, 10 - t, 6 - t))
    for sy in (1, -1):        # C3 (D24 v2): a bump over each rear-cap lug (they stand 6.2 mm proud of the sleeve), 3 mm round it
        ya, yb_ = LIFT_Y + sy * (can_r + 1.0), LIFT_Y + sy * (lug_r + 3.0)
        lo, hi = min(ya, yb_), max(ya, yb_)
        xl1 = can_back + lug_h + 3.0
        outer += box(xc0, xl1 + t, lo - (t if sy < 0 else 0), hi + (t if sy > 0 else 0), LIFT_Z - lug_w / 2 - 3.0 - t, LIFT_Z + lug_w / 2 + 3.0 + t, 4, 2)
        inner += box(xc0 + t, xl1, lo - (0 if sy < 0 else 1), hi + (1 if sy < 0 else 0), LIFT_Z - lug_w / 2 - 3.0, LIFT_Z + lug_w / 2 + 3.0, 3, 1.5)
    c = (outer - inner) & Pos((xc0 + xp1) / 2, 0, zb) * Box(xp1 - xc0 + 2, 400, 200, align=MIN)          # open below, 3 mm off the deck
    c -= Pos(xb, yb, LIFT_Z) * Box(wx + 6.4, ty + 6.4, 200, align=MIN)                                   # belt passage (≥ 3 mm round the belt)
    cb = coupling()[0].bounding_box()
    c -= Pos(cb.center().X, cb.center().Y, cb.center().Z) * Box(cb.size.X + 6, cb.size.Y + 6, cb.size.Z + 6)   # 3 mm round the coupling
    c -= Pos(LIFT_XF + 3.0, LIFT_Y, Z_BASE) * Box(8.0, 84.0, LIFT_WALL_TOP - Z_BASE + 1.0, align=MIN)    # wall A + 1 mm (the riser stands on it)
    from .belt_stage import ear_tower
    c -= lift_stage_T() * ear_tower(None, EAR_SITE, (40.0, -40.0), grow=1.0)                      # W8b lift4: 1 mm round the ±40° ear towers
    sols = c.solids()
    return max(sols, key=lambda q: q.volume) if len(sols) > 1 else c


def carriage(lift_mm: float) -> Part:
    h = 160.0
    zc = lift_mm + 10.0                                               # lift v2: centred 10 mm up, bottom at lift − 70 (the torso's bottom moved to lift − 75)
    c = Pos(MAST_X, 0, zc - h / 2) * (Box(W + 26, W + 26, h, align=MIN) - Box(W + 2, W + 2, h, align=MIN))
    for z in (zc - h / 2 + 18, zc + h / 2 - 18):                      # 625 bearing bosses, 2 per face
        for dx, dy in ((W / 2 + 7, 0), (-W / 2 - 7, 0), (0, W / 2 + 7), (0, -W / 2 - 7)):
            c += Pos(MAST_X + dx, dy, z) * Box(14 if dx else 30, 30 if dx else 14, 24)
    c -= Pos(MAST_X - W / 2 - 6, 23, lift_mm) * Box(14, 18, 400)                 # slot over the rack
    # pawl box (pawl + 12 V solenoid) on the back, beside the slot; A15: pocket for the steel pawl (1.2 mm round it)
    c += Pos(MAST_X - W / 2 - 20, 23, lift_mm - 30) * Box(14, 22, 40)
    c -= Pos(-108.6, RACK["y"], lift_mm - 30) * Box(15.2, 8.4, 30.0)
    if D.arm.arm_motors == "d20":
        # C3 (D24 v2, R14 (d)): the D20 pitch cans' rear ends sit 2.6 mm off the carriage's side faces and must back off
        # 4.5 mm (Oldham tongue) before they drop out through their side exit, so each side wall is 12 → 8 mm thick over
        # a slot under the can (x −66 … −15, carriage bottom → 56 mm over the pitch axis). D20 only; ESTIMATE: re-run the
        # carriage FEA / rnd.stiffness with PIP_ARM_MOTORS=d20 before printing.
        for sy in (1, -1):
            c -= Pos(-40.5, sy * (W / 2 + 12.0), zc - h / 2 - 1) * Box(51.0, 6.0, lift_mm + 56 - (zc - h / 2) + 1, align=(Align.CENTER, Align.CENTER, Align.MIN))   # |y| W/2 + 9 … + 15 (the 625 bosses too)
    return c


TORSO_Z = (-75.0, 105.0)       # torso shell bottom / top relative to the lift height (mm)
# R17 study (2026-11-27, Neon review #2 finding 5; owner decision pending): a conical shoulder recess in each torso side,
# revolved about the pitch axis so pitching never changes it. PIP_TORSO_RECESS=1 builds it (default off). Sized from the
# arm's swept envelope (roll −15…100°, any pitch; rnd/parts/roll.py + covers.roll_frame + the upper bone): the yaw band's
# lower inboard corner reaches 2 mm outboard of the side wall's plane (pitch frame y −3, 77 mm from the pitch axis) at
# roll −15°, so ≥ 25 mm needs the wall at y ≤ −27.7 (pitch frame; the side wall is at −5) out to ≈ 130 mm from the axis.
# The recess: a flat floor TORSO_RECESS["depth"] in from the side wall out to r_floor, a 45° cone out to the wall, 2 mm walls; the
# shoulder bracket's outer wall (shoulder.bracket_wall, pitch frame y −34.3) stays 4.6 mm inside the recess floor.
# It moves only the torso side: the inboard wedge proper (yaw block / band × the arm-side ferrule, 2.5 mm at −15°) is
# between two arm parts and does not change (rnd/pinch.py).
import os as _os
TORSO_RECESS_ON = _os.environ.get("PIP_TORSO_RECESS", "0") == "1"
TORSO_RECESS = dict(depth=22.7, r_floor=130.0, wall=2.0)


def _torso_recess_cut(lift_mm, s, grow=0.0):
    """Revolved recess region about the s-side pitch axis (robot frame): floor at |y| = 195 − depth − grow out to
    r_floor + grow, then a 45° cone to the side wall's plane."""
    from build123d import Cone
    d, rf = TORSO_RECESS["depth"], TORSO_RECESS["r_floor"]
    yf = 195.0 - d - grow                                                        # floor (|y|)
    h = 30.0                                                                     # past the side wall
    r_top = rf + grow + (195.0 + h - yf)                                         # 45° cone: radius grows 1:1 with |y|
    c = Cone(rf + grow, r_top, 195.0 + h - yf, align=(Align.CENTER, Align.CENTER, Align.MIN))
    c = Pos(D.arm.shoulder_x * 1000, s * yf, lift_mm) * Rot(-90 * s, 0, 0) * c
    return c


def torso(lift_mm: float):
    # Looks pass (owner request): a rounded body that encloses both shoulder pitch modules; the arms emerge through
    # round ports at the shoulders. Printed as left/right halves (each 185 mm wide).
    x0 = 15.0
    # lift v2: bottom at lift − 75 (was − 105; the pitch brackets reach − 65): ≥ 120 mm under the torso at lift_min
    # (D23 (3)) and room for the lift stage behind the mast coupling
    zc = lift_mm + TORSO_Z[0] + (TORSO_Z[1] - TORSO_Z[0]) / 2
    if TORSO_RECESS_ON:
        t = TORSO_RECESS["wall"]
        outer = Pos(x0, 0, zc) * soft_box(230, 390, TORSO_Z[1] - TORSO_Z[0], 70, 30)
        inner = Pos(x0, 0, zc) * soft_box(230 - 2 * t, 390 - 2 * t, TORSO_Z[1] - TORSO_Z[0] - 2 * t, 70 - t, 30 - t)
        for s in (1, -1):
            outer -= _torso_recess_cut(lift_mm, s)
            inner -= _torso_recess_cut(lift_mm, s, t)
        shell = outer - inner
    else:
        shell = Pos(x0, 0, zc) * soft_shell(230, 390, TORSO_Z[1] - TORSO_Z[0], 70, 30, 2.0)   # edge r 40 → 30: the shorter shell's bottom edge would cut the pitch brackets' lower corners (+0.12 cm³)
    shell -= Pos(x0, 0, lift_mm) * Box(400, 1.0, 400)                                 # split line
    shell -= Pos(MAST_X, 0, lift_mm) * Box(W + 30, W + 30, 400)                       # mast + carriage passage
    for s in (1, -1):                                                                  # shoulder ports (pitch shaft + V-band)
        shell -= Pos(D.arm.shoulder_x * 1000, s * 195, lift_mm) * Rot(90, 0, 0) * Cylinder(50, 2 * (TORSO_RECESS["depth"] + TORSO_RECESS["wall"] + 10) if TORSO_RECESS_ON else 40)
    # electronics in front of the pitch cartridges (they reach into the torso at x −53 … 33)
    tray = Pos(75, 0, lift_mm - 40) * Box(60, 150, 6)
    pi = Pos(75, -20, lift_mm - 25.5) * Box(56, 85, 22)
    dcdc = Pos(75, 50, lift_mm - 23.5) * Box(50, 46, 26)
    # D10: reserved Jetson-class AI computer bay (inner 98 × 91 × 40). C3 (2026-11-27): raised 39 mm and 3 mm shorter — the
    # D20 pitch cans' rear ends sat 0.9 mm inside it and could not move the 4.5 mm their side-exit swap needs.
    ai_bay = Pos(74, 0, lift_mm + 64) * (Box(102, 95, 40) - Box(98, 91, 40))
    estop = Pos(-30, 80, lift_mm + 105) * Cylinder(15, 14, align=MIN)
    light = Pos(x0 + 116.5, 0, lift_mm - 30) * soft_box(3, 160, 8, 1.2, 0.6)
    return [("torso_shell", shell, "white"), ("ai_bay", ai_bay, "dark"), ("e_tray", tray, "dark"), ("e_pi5", pi, "dark"), ("e_dcdc", dcdc, "dark"),
            ("estop", estop, "orange"), ("chest_light", light, "orange")]


HEAD_MASS_G = {"screen": 150.0, "cam": 99.0}      # 7" board ESTIMATE (confirm on the scale); Gemini 2 99 g (as in masses())
BRG_6805 = dict(d=25.0, D=37.0, B=7.0)            # passive tilt pivot bearing (BOM: 6805-2RS, 25 × 37 × 7)
TILT_FRICTION_NM = 0.3                            # wave-spring + PTFE washer drag at the pivot, ESTIMATE (holds the head with the servo limp)

# ---------------------------------------------------------------------------------------------------------------------
# Neck v2 (R17 neck CAD pass, 2026-11-27). The D22 pivot sits at the head's centre of mass, 103 mm in front of the pan axis,
# so any yoke part that crosses the shell near the pan axis moves 45–60 mm relative to the rim over −10…+15° (the v1 yoke
# plate, posts and riser hub came in through a 110 × 90 back/bottom opening and the rim closed to the 3 mm cut clearance;
# a TPU floor bonded to the hub would have to stretch 12 → 46 mm, +281 %). v2 makes the head's back a surface of
# revolution about the tilt axis, so tilting never moves it relative to anything:
# - **D-profile head**: flat front (screen, unchanged), the back a half-cylinder of HEAD_R about the tilt axis from the
#   top to the bottom (the head is HEAD_R above and below the axis), its side edges rounded HEAD_RE (a torus about the
#   same axis). 252 × 176 × 130 mm, 602 g (v1: 252 × 184 × 160, 669 g). HEAD_R = 88 keeps the round back ≥ 25 mm from
#   the pan sleeve's corners at every pan × tilt (nearest corner 112 mm from the axis, inside the slot there: the saddle
#   is 30 mm from it). The axis sits PIVOT_BACK behind the COM (the lighter back moved the COM 10 mm forward).
# - **Neck bar**: one 30 × 24 mm bar from the riser hub on the pan output straight to the tilt axis (radial, so it only
#   swings about the axis), a Ø36 cross-tube on the axis inside the head carrying the two Ø44 pivot bosses at y ±BOSS_Y
#   (−Y: the 6805, +Y: the tilt cartridge, as in v1 but 10 mm further in, clear of the shell's front corner blend). The
#   neck cable runs in the bar and the cross-tube and enters the head through the hollow −Y pivot stub (on the axis: it
#   only twists ±12.5°).
# - **Neck slot**: the bar enters through a slot in the round back that keeps ≥ NECK_CLEAR (26 mm) between its rim and
#   the bar at every tilt (≈ 72° × 82 mm).
# - **Neck saddle**: a 3 mm PETG cylindrical sector on the bar, concentric with the round back and NECK_GAP (4 mm) inside
#   it, wide enough to stay under the whole slot rim over the tilt range (+ 4° and 8 mm of overlap): the slot is closed
#   to the probe (constant 4 mm gap) and the head's interior (tilt cartridge, stubs, camera, screen) is out of reach.
# The v1 TPU neck boot is retired (nothing to bond it to: no opening is left). The head shell prints as two halves split
# at y = 0 (each ≤ 126 mm wide), closed round the yoke with M3 at the seam; the −Y half's stub goes into the 6805, the +Y
# half's drive stub onto the tilt cartridge output (both along y).
HEAD_R = 88.0                     # round back radius about the tilt axis (mm)
HEAD_W = 252.0                    # head width (y)
HEAD_FRONT = 150.0                # front face (head frame x)
HEAD_RV, HEAD_RE = 58.0, 40.0     # front vertical-edge radius; top/bottom/side-edge radius (also the round back's end edges)
HEAD_T = 2.4                      # shell wall
NECK_BAR = dict(w=30.0, t=24.0)   # neck bar: y width, thickness (in the tilt plane)
NECK_CLEAR = 26.0                 # slot rim ↔ bar at every tilt (R17 ≥ 25)
NECK_GAP, SADDLE_T = 4.0, 3.0     # saddle ↔ shell inner surface (R17 ≤ 8, constant); saddle wall
SADDLE_LAP = (4.0, 8.0)           # saddle overlap past the slot rim at the worst tilt: angle (deg), y (mm)
BOSS_Y = 61.0                     # pivot bosses (8 mm) centred at y ±BOSS_Y (v1: ±71)
BOSS_R, TUBE_R = 22.0, 18.0       # pivot boss (6805 OD 37: 3.5 mm wall; v1 Ø48) and cross-tube radius (Ø28 bore round the Ø25 stub)
PIVOT_BACK = 6.0                  # tilt axis this far behind the head's COM: the bosses clear the screen board by 4 mm; gravity
                                  # torque 0.6 kg × 9.81 × 6 mm ≈ 0.035 N·m (≈ 1 % of the STS3215's stall, under the 0.3 N·m pivot drag)
HUB_TOP = -40.0                   # riser hub top (head frame z): the yoke plate's old level (yoke z 0)


def _cyl_y(px, pz, r, y0, y1):
    """Cylinder about the tilt axis (head frame: along Y through (px, pz)) from y0 to y1."""
    return Pos(px, (y0 + y1) / 2, pz) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)


def _sector(px, pz, r0, r1, a0, a1, y0, y1):
    """Annular sector about the tilt axis: radii r0…r1, polar angles a0…a1 (deg; 0 = +X, 90 = +Z, head frame), y0…y1."""
    from build123d import BuildPart, BuildSketch, Plane, Polygon, extrude
    n = max(8, int(abs(a1 - a0) / 3) + 2)
    step = (a1 - a0) / (n - 1)
    L = (r1 + 2.0) / math.cos(math.radians(abs(step) / 2))
    pts = [(px, pz)] + [(px + L * math.cos(math.radians(a0 + step * k)), pz + L * math.sin(math.radians(a0 + step * k))) for k in range(n)]
    with BuildPart() as bp:
        with BuildSketch(Plane.XZ):
            Polygon(*pts, align=None)
        extrude(amount=(y1 - y0) / 2, both=True)
    wedge = Pos(0, (y0 + y1) / 2, 0) * bp.part
    ring = _cyl_y(px, pz, r1, y0, y1) - (_cyl_y(px, pz, r0, y0 - 1, y1 + 1) if r0 > 0 else Box(0.01, 0.01, 0.01))
    return wedge & ring


def _d_solid(px, pz, R, front, W, rv, re):
    """D-profile head solid (head frame): half-cylinder R about the tilt axis behind it (end edges rounded re), a block
    from the axis to the front face (vertical front edges rv, top/bottom/side edges re), R above and below the axis."""
    cyl = fillet(_cyl_y(px, pz, R, -W / 2, W / 2).edges(), re)
    cyl &= Pos(px - R / 2, 0, pz) * Box(R, W + 2, 2 * R + 2)                      # the back half only
    L = (front - px) + 60.0
    blk = Pos(front - L, 0, pz) * soft_box(2 * L, W, 2 * R, rv, re)
    blk &= Pos((px + front) / 2, 0, pz) * Box(front - px, W + 2, 2 * R + 2)
    return cyl + blk


def _neck_angles(px, pz):
    """Polar angles (deg, about the tilt axis, head frame at tilt 0) of the neck bar, the slot (lo, hi) and the saddle
    (lo, hi). A yoke point at angle φ shows at φ + tilt in the head frame."""
    bar = math.degrees(math.atan2(HUB_TOP - pz, 0.0 - px)) % 360.0
    lo, hi = TILT_RANGE
    w = math.degrees((NECK_BAR["t"] / 2 + NECK_CLEAR) / (HEAD_R - HEAD_T))
    slot = (bar + lo - w, bar + hi + w)
    sad = (slot[0] - hi - SADDLE_LAP[0], slot[1] - lo + SADDLE_LAP[0])
    return bar, slot, sad


def _slot_y():
    return NECK_BAR["w"] / 2 + NECK_CLEAR


def _head_body(px, pz):
    """Head pieces in the head frame (x forward, origin = the v0 tilt axis 40 mm above the yoke plate), for a tilt axis
    at (px, pz). D22: 7" Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (1024 × 600, CAN). Neck v2: D-profile shell (see above)."""
    R, t = HEAD_R, HEAD_T
    shell = _d_solid(px, pz, R, HEAD_FRONT, HEAD_W, HEAD_RV, HEAD_RE) - _d_solid(px, pz, R - t, HEAD_FRONT - t, HEAD_W - 2 * t, HEAD_RV - t, HEAD_RE - t)
    shell -= Pos(150, 0, 40) * Box(20, 236, 124)                                     # bezel opening
    # R15 (2026-11-27): the 182 × 112 board's top corners sat in the shell's corner blend (vertical r 58 meets the top
    # r 40, 0.0 mm). A screen bay 2 mm round the board, from behind it to the bezel opening, clears it; the notch it
    # leaves in the blend is behind the visor (232 × 120), so it doesn't show.
    shell -= Pos(141, 0, 40) * Box(19, 186, 116)
    shell -= Pos(135, 0, -36) * Box(40, 96, 29)                                       # depth-camera window (Gemini 2)
    _, slot, _ = _neck_angles(px, pz)
    ys = _slot_y()
    shell -= _sector(px, pz, R - t - 1.0, R + 1.0, slot[0], slot[1], -ys, ys)        # neck slot (≥ NECK_CLEAR round the bar)
    visor = Pos(146, 0, 40) * soft_box(6, 232, 120, 2.8, 1.5) - Pos(146, 0, 40) * Box(10, 156, 90)   # 7" active area ≈ 154 × 86
    screen = Pos(138, 0, 40) * Box(9, 182, 112)                                       # board outline ESTIMATE (confirm with the drawing)
    eyes = [Pos(142.8, s * 34, 46) * Rot(0, 90, 0) * Cylinder(14, 0.6) for s in (1, -1)]      # drawn on the screen (render only)
    cam = Pos(128, 0, -36) * Box(30, 90, 25)                                         # Gemini 2, below the bezel
    return shell, visor, screen, eyes, cam


_PIVOT = []


def head_pivot():
    """D22 passive pivot: the tilt axis through the head's centre of mass (head frame x, z), so gravity puts ≈ no torque
    on the tilt servo at any angle. Masses: printed shell at 80 % of solid PETG, visor solid PETG, board and camera
    from HEAD_MASS_G. Neck v2: the D-profile shell is built round the axis, so the axis is iterated to a fixed point
    (≤ 0.2 mm, from the v1 value), PIVOT_BACK behind the COM."""
    if not _PIVOT:
        from build123d import CenterOf
        px, pz = 103.0, 26.9
        for _ in range(6):
            shell, visor, screen, _, cam = _head_body(px, pz)
            items = [(shell, shell.volume / 1000 * 1.27 * 0.8), (visor, visor.volume / 1000 * 1.27),
                     (screen, HEAD_MASS_G["screen"]), (cam, HEAD_MASS_G["cam"])]
            m = sum(w for _, w in items)
            cx = sum(p.center(CenterOf.MASS).X * w for p, w in items) / m - PIVOT_BACK
            cz = sum(p.center(CenterOf.MASS).Z * w for p, w in items) / m
            done = math.hypot(cx - px, cz - pz) <= 0.2
            px, pz = round(cx, 1), round(cz, 1)
            if done:
                break
        _PIVOT.append((px, pz, m))
    return _PIVOT[0]


def _yoke(px, pz, g=0.0):
    """Neck v2 yoke in the head frame at tilt 0 (head(): + 40 mm to the yoke frame, z = 0 at the old yoke plate): riser
    hub on the pan output (Ø40, z −60 … HUB_TOP), the neck bar from the hub to the tilt axis, the cross-tube on the axis
    (TUBE_R, Ø28 bore: the −Y pivot stub and the neck cable), the two pivot bosses (BOSS_R) at y ±BOSS_Y and the neck saddle.
    `g` grows it for the clearance cut."""
    bar_a, _, sad = _neck_angles(px, pz)
    u = (math.cos(math.radians(bar_a)), math.sin(math.radians(bar_a)))
    r_end = math.hypot(px, HUB_TOP - pz) + 8.0
    r0 = 12.0
    L = r_end - r0
    f = Pos(0, 0, -60.0 - g) * Cylinder(20 + g, HUB_TOP + 60.0 + g, align=MIN)                     # riser hub
    f += Pos(px + u[0] * (r0 + L / 2), 0, pz + u[1] * (r0 + L / 2)) * Rot(0, -bar_a, 0) * Box(L, NECK_BAR["w"] + 2 * g, NECK_BAR["t"] + 2 * g)
    f += _cyl_y(px, pz, TUBE_R + g, -BOSS_Y + 4 - g, BOSS_Y - 4 + g)                                                # cross-tube
    for s in (1, -1):
        f += _cyl_y(px, pz, BOSS_R + g, s * BOSS_Y - 4 - g, s * BOSS_Y + 4 + g)                                  # pivot boss
    ro = HEAD_R - HEAD_T - NECK_GAP
    ys = _slot_y() + SADDLE_LAP[1]
    f += _sector(px, pz, ro - SADDLE_T - g, ro + g, sad[0] - (2 if g else 0), sad[1] + (2 if g else 0), -ys - g, ys + g)   # neck saddle
    if g == 0:
        f -= _cyl_y(px, pz, TUBE_R - 4.0, -80, 80)                                                       # bore
    f &= Pos(0, 0, -60.0 - g) * Box(800, 800, 400, align=MIN)                                       # nothing below the pan output face
    return f


def head(pan_deg: float = 0.0, tilt_deg: float = 0.0):
    top = Pos(MAST_X, 0, Z_TOP) * Box(W + 16, W + 16, 24, align=MIN)                  # top bracket on the mast
    top -= Pos(MAST_X, 0, Z_TOP) * Box(W + 0.4, W + 0.4, 10, align=MIN)
    handle = Pos(MAST_X - 50.5, 0, Z_TOP + 24) * Rot(90, 0, 0) * (Cylinder(34, 24) - Cylinder(24, 25) - Pos(0, -40, 0) * Box(80, 80, 30))   # R15: x −50 → −50.5 (0.9 → 1.4 mm to the pan sleeve)
    xb, yb, wx, ty = lift_belt_xy()                                                   # lift v2: belt in the YZ plane, idler axis along X
    idler = Pos(xb, yb - 14 - ty / 2, Z_TOP + 2) * Rot(0, 90, 0) * Cylinder(14, wx + 3)        # top 8 mm under the mast cap's top (R17)
    # pan cartridge S, shaft up, on the top bracket; yoke + head ride on it
    pan = Pos(MAST_X, 0, Z_TOP + 24) * Rot(0, 0, 90) * cs.sleeve()
    yz = Z_TOP + 24 + cs.stack_height() + PAN_RISER
    P = Pos(MAST_X, 0, yz) * Rot(0, 0, pan_deg)
    # D22 (owner, 2026-11-27): the 7" head (+≈ 120 g) clashed with the mast bracket at +30° tilt and loaded the tilt servo
    # (75 % before the bigger screen). The tilt axis moves from the neck (40 mm above the yoke) to the head's centre of
    # mass: a passive 6805 pivot on the −Y boss (wave spring + PTFE washer for drag), the tilt cartridge coaxial on the
    # +Y boss. Range TILT_RANGE (−10° up … +15° down; D25, was +30°). Neck v2 (R17): see the section above.
    px, pz, _ = head_pivot()
    zp = 40 + pz                                                                       # pivot height above the yoke plate
    Y = Pos(0, 0, 40)                                                                  # head frame (tilt 0) → yoke frame
    yoke = Y * _yoke(px, pz)
    yoke -= Pos(px, -BOSS_Y, zp) * Rot(90, 0, 0) * Cylinder(BRG_6805["D"] / 2 + 0.05, 20)                   # 6805 seat (−Y boss)
    tilt = Pos(px, BOSS_Y + 4, zp) * Rot(-90, 0, 0) * Rot(0, 0, 180) * cs.sleeve()                              # body points aft, away from the screen
    brg = Pos(px, -BOSS_Y, zp) * Rot(90, 0, 0) * (Cylinder(BRG_6805["D"] / 2, BRG_6805["B"]) - Cylinder(BRG_6805["d"] / 2, BRG_6805["B"]))
    # loop 25: slotted torque pads for the pan and tilt cartridges' Ø4 pins (rnd/parts/torque.py)
    from .torque import with_pin_slot
    top = with_pin_slot(pan, top, 1.95, 2.0)
    yoke = with_pin_slot(tilt, yoke, 1.95, 2.0)
    T = Pos(0, 0, 40) * Pos(px, 0, pz) * Rot(0, tilt_deg, 0) * Pos(-px, 0, -pz)        # head frame → yoke frame
    shell, visor, screen, eyes, cam = _head_body(px, pz)
    # head-side pivot stubs bonded to the shell's inner side walls (head frame): −Y a Ø25 tube through the 6805 with a
    # flange on the wall; +Y a Ø30 disc onto the tilt cartridge's output face
    yi = HEAD_W / 2 - HEAD_T                                                          # shell inner side wall
    stub_m = Pos(px, -(BOSS_Y - 4 + yi) / 2, pz) * Rot(90, 0, 0) * (Cylinder(BRG_6805["d"] / 2, yi - BOSS_Y + 4) - Cylinder(8.5, yi))
    stub_m += Pos(px, -yi + 2, pz) * Rot(90, 0, 0) * Cylinder(22, 4)
    tb = tilt.bounding_box()
    stub_p = Pos(px, (tb.max.Y + 0.8 + yi) / 2, pz) * Rot(90, 0, 0) * Cylinder(15, yi - tb.max.Y - 0.8)
    stub_p += Pos(px, yi - 2, pz) * Rot(90, 0, 0) * Cylinder(22, 4)
    inner = _d_solid(px, pz, HEAD_R - HEAD_T, HEAD_FRONT - HEAD_T, HEAD_W - 2 * HEAD_T, HEAD_RV - HEAD_T, HEAD_RE - HEAD_T)   # R17: the stub
    stub_m, stub_p = stub_m & inner, stub_p & inner                                              # flanges stay inside the shell
    # clearance cuts over the tilt range (and any pan, for the pan sleeve): the yoke grown 3 mm, the tilt sleeve's box,
    # the pan sleeve's cylinder (neck v2: by design they cut nothing; kept as a guard against later edits)
    env_y = Y * _yoke(px, pz, 3.0)
    env_t = Pos(tb.center().X, tb.center().Y, tb.center().Z) * Box(tb.size.X + 4, tb.size.Y + 4, tb.size.Z + 4)
    pb = (Pos(-MAST_X, 0, -yz) * pan).bounding_box()                                 # pan sleeve in the yoke (P) frame
    r_pan = max(math.hypot(x, y) for x in (pb.min.X, pb.max.X) for y in (pb.min.Y, pb.max.Y)) + 2
    env_p = Pos(0, 0, pb.min.Z - 2) * Cylinder(r_pan, pb.size.Z + 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    rlo, rhi = TILT_RELIEF
    for td in [rlo + k * (rhi - rlo) / 8 for k in range(9)]:
        inv = (Pos(0, 0, 40) * Pos(px, 0, pz) * Rot(0, td, 0) * Pos(-px, 0, -pz)).inverse()
        for e in (env_y, env_t, env_p):
            shell -= inv * e
    sols = shell.solids()
    shell = max(sols, key=lambda q: q.volume) if len(sols) > 1 else shell
    out = [("head_bracket", top, "white"), ("handle", handle, "dark"), ("belt_idler", idler, "steel"),
           ("pan_sleeve", pan, "white"), ("yoke", P * yoke, "dark"), ("tilt_sleeve", P * tilt, "white"),
           ("tilt_bearing", P * brg, "steel"),
           ("head_shell", P * T * shell, "white"), ("head_pivot_stub", P * T * stub_m, "dark"), ("head_drive_stub", P * T * stub_p, "dark"),
           ("visor", P * T * visor, "dark"), ("head_screen", P * T * screen, "black"),
           ("eye_l", P * T * eyes[0], "orange"), ("eye_r", P * T * eyes[1], "orange"), ("head_cam", P * T * cam, "dark")]
    return out


def module(lift_m: float = 0.75, head_pan: float = 0.0, head_tilt: float = 0.0):
    L = lift_m * 1000
    return [("mast", mast(), "black"), ("ratchet", ratchet_strip(), "steel"), *lift_drive(), ("lift_cover", lift_cover(), "white"),
            ("carriage", carriage(L), "white"), ("lift_pawl", lift_pawl(L), "steel"), *torso(L), *head(head_pan, head_tilt)]


def masses(density=1.27):
    L = 750.0
    tor = {n: p for n, p, c in torso(L)}
    hd = {n: p for n, p, c in head()}
    return {
        "carriage (printed) + 8×625 bearings + solenoid": carriage(L).volume / 1e6 * density * 0.6 + 8 * 0.012 + 0.05,
        "lift pawl (steel)": lift_pawl(L).volume / 1e6 * 7.85,
        "torso shell + tray (printed)": (tor["torso_shell"].volume + tor["e_tray"].volume) / 1e6 * density,
        "steel rack m1 10 × 10 (A15) + printed end fillers": ratchet_strip(False).volume / 1e6 * 7.85 + (ratchet_strip().volume - ratchet_strip(False).volume) / 1e6 * density,
        "lift bracket + cover + bolted can/cap (printed)": (lift_mount().volume * 0.6 + lift_cover().volume
                                                            + sum(p.volume for n, p, c in lift_drive() if n in ("lift_can", "lift_can_cap")) * 0.7) / 1e6 * density,
        "lift stage metal (Ø12 shaft, 2 × 6801, 20T alu, Oldham; ESTIMATE as the arm stages) + belt 2.4 m + idler": 0.10 + 0.10,
        "head: bracket, handle, yoke, shell, visor (printed)": sum(hd[k].volume for k in ("head_bracket", "handle", "yoke", "head_shell", "visor")) / 1e6 * density * 0.8,
        "head: 2 × STS3215 + sleeves + RGB-D camera (99 g) + eye LEDs": 2 * 0.055 + 2 * 0.02 + 0.10 + 0.02,
    }
