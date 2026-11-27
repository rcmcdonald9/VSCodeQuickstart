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
TILT_RANGE = (-10.0, 30.0)      # head tilt (deg), D22: −10° (up) … +30° (down to the table), on the passive pivot
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
EAR_SITE = "lift"                            # D24 v2 flange-ear pattern (cartridge_m.EAR_SITES), read by rnd/assembly_seq.py
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
    from .belt_stage import V3, wall_a_cuts
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
    m -= T * wall_a_cuts(None, EAR_SITE)                                         # D24 v2: 2 ears at ±140° (cartridge_m.EAR_SITES["lift"])
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


def torso(lift_mm: float):
    # Looks pass (owner request): a rounded body that encloses both shoulder pitch modules; the arms emerge through
    # round ports at the shoulders. Printed as left/right halves (each 185 mm wide).
    x0 = 15.0
    # lift v2: bottom at lift − 75 (was − 105; the pitch brackets reach − 65): ≥ 120 mm under the torso at lift_min
    # (D23 (3)) and room for the lift stage behind the mast coupling
    shell = Pos(x0, 0, lift_mm + TORSO_Z[0] + (TORSO_Z[1] - TORSO_Z[0]) / 2) * soft_shell(230, 390, TORSO_Z[1] - TORSO_Z[0], 70, 30, 2.0)   # edge r 40 → 30: the shorter shell's bottom edge would cut the pitch brackets' lower corners (+0.12 cm³)
    shell -= Pos(x0, 0, lift_mm) * Box(400, 1.0, 400)                                 # split line
    shell -= Pos(MAST_X, 0, lift_mm) * Box(W + 30, W + 30, 400)                       # mast + carriage passage
    for s in (1, -1):                                                                  # shoulder ports (pitch shaft + V-band)
        shell -= Pos(D.arm.shoulder_x * 1000, s * 195, lift_mm) * Rot(90, 0, 0) * Cylinder(50, 40)
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


def _head_body():
    """Head pieces in the head frame (x forward, origin = the v0 tilt axis 40 mm above the yoke plate), before the
    clearance cuts. D22: 7" Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (1024 × 600, CAN). Head 252 × 184 mm."""
    shell = Pos(70, 0, 28) * soft_shell(160, 252, 184, 58, 40, 2.4)                 # pill-shaped head
    shell -= Pos(150, 0, 40) * Box(20, 236, 124)                                     # bezel opening
    # R15 (2026-11-27): the 182 × 112 board's top corners sat in the shell's corner blend (vertical r 58 meets the top
    # r 40, 0.0 mm). A screen bay 2 mm round the board, from behind it to the bezel opening, clears it; the notch it
    # leaves in the blend is behind the visor (232 × 120), so it doesn't show.
    shell -= Pos(141, 0, 40) * Box(19, 186, 116)
    shell -= Pos(135, 0, -36) * Box(40, 96, 29)                                       # depth-camera window (Gemini 2)
    shell -= Pos(-20, 0, -20) * Box(110, 180, 90)                                    # neck opening at the back/bottom (yoke + pan servo)
    visor = Pos(146, 0, 40) * soft_box(6, 232, 120, 2.8, 1.5) - Pos(146, 0, 40) * Box(10, 156, 90)   # 7" active area ≈ 154 × 86
    screen = Pos(138, 0, 40) * Box(9, 182, 112)                                       # board outline ESTIMATE (confirm with the drawing)
    eyes = [Pos(142.8, s * 34, 46) * Rot(0, 90, 0) * Cylinder(14, 0.6) for s in (1, -1)]      # drawn on the screen (render only)
    cam = Pos(128, 0, -36) * Box(30, 90, 25)                                         # Gemini 2, below the bezel
    return shell, visor, screen, eyes, cam


_PIVOT = []


def head_pivot():
    """D22 passive pivot: the tilt axis through the head's centre of mass (head frame x, z), so gravity puts ≈ no torque
    on the tilt servo at any angle. Masses: printed shell at 80 % of solid PETG, visor solid PETG, board and camera
    from HEAD_MASS_G."""
    if not _PIVOT:
        from build123d import CenterOf
        shell, visor, screen, _, cam = _head_body()
        items = [(shell, shell.volume / 1000 * 1.27 * 0.8), (visor, visor.volume / 1000 * 1.27),
                 (screen, HEAD_MASS_G["screen"]), (cam, HEAD_MASS_G["cam"])]
        m = sum(w for _, w in items)
        cx = sum(p.center(CenterOf.MASS).X * w for p, w in items) / m
        cz = sum(p.center(CenterOf.MASS).Z * w for p, w in items) / m
        _PIVOT.append((round(cx, 1), round(cz, 1), m))
    return _PIVOT[0]


def neck_boot(shell, px, pz):
    """R17 (2026-11-27): TPU 95A boot (2 mm) closing the head's neck opening (the cut at the shell's back and bottom where
    the yoke comes in, plus the slots the yoke's tilt sweep cuts in its bottom and back), head frame (it rides the head;
    bonded to the opening's rim). A box behind the opening round the yoke's posts and beams (6 mm behind the yoke, up to
    z 100), a floor 3.5 mm under the yoke plate with a Ø44 hole for the riser hub, out to x 80, and a front lip + side
    walls from the floor down to the shell's bottom. Lip seals 1.5 mm round the yoke
    beams: the boot flexes there with the tilt (−10…+30°; folding not modelled, ESTIMATE). One print, ≈ 116 × 188 × 164 mm."""
    t = NECK_BOOT_T
    zf = -40.0 - 1.5 - 2.0                                                           # floor top (the yoke plate is at z −40 at tilt 0)
    y1 = 94.0
    zt = 100.0                                                                       # up over the yoke-beam slots in the shell's back wall
    rear = Pos((-36.0 - 6.0) / 2, 0, (zf - t + zt) / 2) * soft_shell(30.0, 2 * y1, zt - zf + t, 12, 6, t)
    rear -= Pos(0, 0, 0) * Pos(20.0, 0, 0) * Box(60.0, 400, 400)                     # keep x < −10 (the shell's back plane)
    xl = 80.0                                                                        # past the yoke's swept slots in the shell bottom (x ≤ 75)
    floor = Pos((-10.0 + xl) / 2, 0, zf - t / 2) * Box(xl + 10.0, 2 * y1, t)
    lip = Pos(xl - t / 2, 0, (-64.0 + zf) / 2) * Box(t, 2 * y1, zf - (-64.0))
    sides = [Pos((-10.0 + xl) / 2, s * (y1 - t / 2), (-64.0 + zf) / 2) * Box(xl + 10.0, t, zf - (-64.0)) for s in (1, -1)]
    b = rear + floor + lip + sides[0] + sides[1]
    b -= Pos(0, 0, zf - 10) * Cylinder(22.0, 20)                                     # riser hub (Ø40) + 2 mm
    b -= shell
    b -= Pos(0, 0, -40.0) * _yoke(px, 40.0 + pz, 1.5)                               # 1.5 mm round the yoke at tilt 0
    sols = b.solids()
    return max(sols, key=lambda q: q.volume) if len(sols) > 1 else b


NECK_BOOT_T = 2.0                 # neck boot wall (TPU 95A)


def neck_boot_spans():
    """rnd/pinch.py compliant-cover rule (R17, wave 5): the neck boot is bonded to the head (rear wall foot at x −34, front
    lip at x 78, side walls at y ±92, head frame) and its floor has to follow the yoke's riser hub (Ø44 collar, yoke frame),
    which turns about the D22 pivot (head COM, 67 mm above the yoke plate) by −tilt relative to the head. Free spans of
    the floor between the collar and those bonded edges, as (name, free length, length(tilt)): the hub travels ≈ 84 mm
    over −10…+30°, so no span stays within ±40 % (rear 12 → 83 mm)."""
    px, pz, _ = head_pivot()
    zf = -40.0 - 1.5 - 2.0

    def at(p, tilt):
        x, z = p[0] - px, p[1] - pz
        t = math.radians(-tilt)
        return x * math.cos(t) + z * math.sin(t) + px, -x * math.sin(t) + z * math.cos(t) + pz

    def span(hub, edge, dy=0.0):
        f = lambda tilt: math.dist((*at(hub, tilt), dy), (*edge, 0.0))
        return f(0.0), f

    out = []
    for name, hub, edge, dy in (("floor, collar → rear wall", (-22.0, zf), (-34.0, zf), 0.0),
                                ("floor, collar → front lip", (22.0, zf), (78.0, zf), 0.0),
                                ("floor, collar → side wall", (0.0, zf), (0.0, zf), 70.0)):
        L0, f = span(hub, edge, dy)
        out.append((name, L0, f))
    return out


def _yoke(px, zp, g=0.0):
    """Fork on the pan output (yoke frame, z = 0 on the pan stack): base plate + two 8 mm arms that rise through the
    head's neck opening and run forward inside the head to the pivot bosses (Ø48). The −Y boss holds the 6805 pivot
    bearing; the tilt cartridge bolts to the +Y boss, coaxial. `g` grows it for the clearance cut."""
    f = Pos(0, 0, -g) * Box(60 + 2 * g, 150 + 2 * g, 8 + 2 * g, align=MIN)
    f += Pos(0, 0, -PAN_RISER - g) * Cylinder(20 + g, PAN_RISER + g, align=MIN)      # R17: riser hub on the pan output
    for s in (1, -1):
        f += Pos(0, s * 71, -g) * Box(40 + 2 * g, 8 + 2 * g, zp + g, align=MIN)                     # post
        f += Pos((px - 20) / 2, s * 71, zp) * Box(px + 20 + 2 * g, 8 + 2 * g, 28 + 2 * g)            # beam to the pivot
        f += Pos(px, s * 71, zp) * Rot(90, 0, 0) * Cylinder(24 + g, 8 + 2 * g)                    # pivot boss
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
    # D22 (owner, 2026-11-27): the 7" head (+≈ 120 g, 252 × 184 mm) clashed with the mast bracket at +30° tilt and loaded
    # the tilt servo (75 % before the bigger screen). The tilt axis moves from the neck (40 mm above the yoke) to the
    # head's centre of mass: a passive 6805 pivot on the −Y arm (wave spring + PTFE washer for drag), the tilt cartridge
    # coaxial on the +Y arm. Range TILT_RANGE (−10° up … +30° down).
    px, pz, _ = head_pivot()
    zp = 40 + pz                                                                       # pivot height above the yoke plate
    yoke = _yoke(px, zp)
    yoke -= Pos(px, -71, zp) * Rot(90, 0, 0) * Cylinder(BRG_6805["D"] / 2 + 0.05, 20)                   # 6805 seat (−Y boss)
    tilt = Pos(px, 75, zp) * Rot(-90, 0, 0) * Rot(0, 0, 180) * cs.sleeve()                              # body points aft, away from the screen
    brg = Pos(px, -71, zp) * Rot(90, 0, 0) * (Cylinder(BRG_6805["D"] / 2, BRG_6805["B"]) - Cylinder(BRG_6805["d"] / 2, BRG_6805["B"]))
    # loop 25: slotted torque pads for the pan and tilt cartridges' Ø4 pins (rnd/parts/torque.py)
    from .torque import with_pin_slot
    top = with_pin_slot(pan, top, 1.95, 2.0)
    yoke = with_pin_slot(tilt, yoke, 1.95, 2.0)
    T = Pos(0, 0, 40) * Pos(px, 0, pz) * Rot(0, tilt_deg, 0) * Pos(-px, 0, -pz)        # head frame → yoke frame
    shell, visor, screen, eyes, cam = _head_body()
    # head-side pivot stubs bonded to the shell's inner side walls (head frame): −Y a Ø25 tube through the 6805 with a
    # flange on the wall; +Y a Ø30 disc onto the tilt cartridge's output face
    yi = 124.0 - 2.4                                                                  # shell inner side wall
    stub_m = Pos(px, -(67 + yi) / 2, pz) * Rot(90, 0, 0) * (Cylinder(BRG_6805["d"] / 2, yi - 67) - Cylinder(8.5, yi))
    stub_m += Pos(px, -yi + 2, pz) * Rot(90, 0, 0) * Cylinder(22, 4)
    tb = tilt.bounding_box()
    stub_p = Pos(px, (tb.max.Y + 0.8 + yi) / 2, pz) * Rot(90, 0, 0) * Cylinder(15, yi - tb.max.Y - 0.8)
    stub_p += Pos(px, yi - 2, pz) * Rot(90, 0, 0) * Cylinder(22, 4)
    inner = Pos(70, 0, 28) * soft_box(160 - 4.8, 252 - 4.8, 184 - 4.8, 58 - 2.4, 40 - 2.4)       # R17: the stub flanges stood out
    stub_m, stub_p = stub_m & inner, stub_p & inner                                              # of the shell's corner blend
    # clearance cuts over the tilt range (and any pan, for the pan sleeve): the yoke grown 3 mm, the tilt sleeve's box,
    # the pan sleeve's cylinder
    env_y = _yoke(px, zp, 3.0)
    env_t = Pos(tb.center().X, tb.center().Y, tb.center().Z) * Box(tb.size.X + 4, tb.size.Y + 4, tb.size.Z + 4)
    pb = (Pos(-MAST_X, 0, -yz) * pan).bounding_box()                                 # pan sleeve in the yoke (P) frame
    r_pan = max(math.hypot(x, y) for x in (pb.min.X, pb.max.X) for y in (pb.min.Y, pb.max.Y)) + 2
    env_p = Pos(0, 0, pb.min.Z - 2) * Cylinder(r_pan, pb.size.Z + 4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    lo, hi = TILT_RANGE
    for td in [lo + k * (hi - lo) / 8 for k in range(9)]:
        inv = (Pos(0, 0, 40) * Pos(px, 0, pz) * Rot(0, td, 0) * Pos(-px, 0, -pz)).inverse()
        for e in (env_y, env_t, env_p):
            shell -= inv * e
    out = [("head_bracket", top, "white"), ("handle", handle, "dark"), ("belt_idler", idler, "steel"),
           ("pan_sleeve", pan, "white"), ("yoke", P * yoke, "dark"), ("tilt_sleeve", P * tilt, "white"),
           ("tilt_bearing", P * brg, "steel"),
           ("head_shell", P * T * shell, "white"), ("head_pivot_stub", P * T * stub_m, "dark"), ("head_drive_stub", P * T * stub_p, "dark"),
           ("visor", P * T * visor, "dark"), ("head_screen", P * T * screen, "black"), ("neck_boot", P * T * neck_boot(shell, px, pz), "dark"),
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
