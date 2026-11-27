"""Shoulder-pitch module v0 (torso side of the arm-detach plane). Left arm; outboard = +Y. Pitch axis = Y at the origin.

- Pitch cartridge M (GIM6010-8) behind the axis (x = -BELT_C), its output facing outboard, bolted to the inner wall
  (D24: 6 × M4 flange, Oldham drive into the D13 small stage).
- HTD 5M 20T → 60T belt (3:1, loop 10) in the belt plane; the 60T is integral with the pitch shaft (D19; Ø40, 2 × 61808
  in the bracket walls, the outer wall a separate bolted print).
- Balancer plane, outboard of the bracket: the pitch hub's disc carries anchor pin B (b = 20 mm, along the hanging arm);
  pulley A rides in a vertical slot 40–60 mm above the axis; the cable runs back over A into a horizontal cage with an
  ISO 10243 Ø25 die spring, 18.7 N/mm (rnd/balancer.py, balancer track 2026-11-27). Open: the straight A–B cable passes
  through the Ø40 shaft at every pitch angle, so the balancer plane has to move past the shaft end.
- Outboard end of the shaft (the bolted pitch hub): the **KX-V ferrule** (3" sanitary profile, 3 dowels): the arm clamps on here with a V-band.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
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
Y_BAL = -3.0               # balancer plane (outboard of the outer wall); loop 39: -6 → -3 (post ≥ 7.8 mm, 2.5 mm to the ferrule spigot)
Y_FER = 2.0                # ferrule spigot start; the detach plane (flat face) is at Y_FER + 18 = 20
FERRULE = dict(od=91.0, t=8.0, taper_deg=20.0, dowel_r=30.0)
# D19 (2026-11-27): the 60T is printed in one piece with the pitch tube again. The tube's outboard end (double-D, 6 × M3
# heat-set inserts in its end face) carries the separate pitch hub (balancer disc + lever + pin B + female ferrule), and
# the outer wall is its own print bolted to the web, so: bearing A on the tube's inboard end → tube into the inner wall's
# 61808 → belt → outer wall over bearing B → pitch hub on the double-D.
Y_TUBE_END = Y_BAL - 2.5   # tube end face = inboard face of the balancer disc (−5.5)
HUB_Y0 = W_OUT + WALL_T / 2 + 0.5   # pitch hub sleeve starts 0.5 mm outboard of the outer wall (on bearing B's inner race)
HUB_SCREW_R = 15.5         # 6 × M3 into the tube end (inserts Ø4.0 between the Ø22 end bore and the flats at 18)
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


def pitch_shaft() -> Part:
    """D19 (2026-11-27, Astra review #2b finding 1): Ø40 tube with the 60T printed on it in one piece (no keyed joint). Its
    inboard end is plain (bearing A slides on from there); its outboard end is a double-D (36 AF, 8 mm) with a Ø22 end
    bore and 6 × M3 heat-set inserts in the end face for the pitch hub. Loop 40's split 60T on a double-D could not be
    fitted (the Ø40 tube can't pass its 36.4 AF bore)."""
    from .belt_stage import dd_flats, PITCH, PLD, DEPTH
    y0, y1 = W_IN - 4, Y_TUBE_END
    L = y1 - y0
    t = Cylinder(20, L, align=MIN) - Cylinder(15, L, align=MIN)
    p60 = pulley(60, 16, flange=False)
    r_rim = 60 * PITCH / math.pi / 2 - PLD - DEPTH - 5.0                   # 5 mm web between a Ø56 hub and a 5 mm rim
    for z in (0.0, 10.5):
        p60 -= Pos(0, 0, z) * (Cylinder(r_rim, 5.5, align=MIN) - Cylinder(28, 5.5, align=MIN))
    t += Pos(0, 0, Y_BELT - 8 - y0) * (p60 - Cylinder(15, 16, align=MIN))
    t += Pos(0, 0, L - 12) * (Cylinder(15, 12, align=MIN) - Cylinder(11, 12, align=MIN))     # end bore Ø22 (room for the inserts)
    t -= dd_flats(HUB_Y0 - y0, y1 - HUB_Y0 + 1)
    for k in range(6):
        a = math.pi / 3 * k + math.pi / 6
        t -= Pos(HUB_SCREW_R * math.cos(a), HUB_SCREW_R * math.sin(a), L - 8) * Cylinder(2.0, 8, align=MIN)
    return Pos(0, y0, 0) * _rot_y(t)


def pitch_hub() -> Part:
    """D19: balancer disc + pin B + female KX-V ferrule in one print, on the tube's double-D (torque) and pulled onto
    its end face by 6 × M3 × 30 from the detach face (heads counterbored below the ferrule face). The sleeve's inboard end
    holds bearing B's inner race. Double-D bearing stress ≈ 8 MPa at the 21 N·m basket hold, ≈ 26 MPa at a 66 N·m fault
    peak (hand calc, ESTIMATE; coupon with the C-series)."""
    from .belt_stage import dd_prism, DD_AF
    h = Y_TUBE_END - HUB_Y0
    sl = Cylinder(25, h, align=MIN) - dd_prism(h, DD_AF + 0.4, 20.2)
    s = Pos(0, HUB_Y0, 0) * _rot_y(sl)
    disc = Cylinder(25, 5, align=MIN) - Cylinder(12, 5, align=MIN)
    disc += Pos(0, 20, 0) * Cylinder(3, 9, align=MIN)                       # pin B, b = 20 mm along the hanging arm (−Z): balancer track 2026-11-27 (pin was on the wrong side, spring added to gravity)
    s += Pos(0, Y_BAL - 2.5, 0) * _rot_y(disc)
    s += Pos(0, Y_BAL + 2.5, 0) * _rot_y(Cylinder(20, Y_FER - Y_BAL - 2.5, align=MIN) - Cylinder(12, 20, align=MIN))
    fer = ferrule(male=False) + (Cylinder(15.05, 18, align=MIN) - Cylinder(12, 18, align=MIN))     # Ø24 bore here: room for the screws
    s += Pos(0, Y_FER + 18, 0) * Rot(90, 0, 0) * fer                       # flat face outboard at the detach plane
    for k in range(6):
        a = math.pi / 3 * k + math.pi / 6
        x, z = HUB_SCREW_R * math.cos(a), HUB_SCREW_R * math.sin(a)
        s -= Pos(x, Y_TUBE_END, z) * _rot_y(Cylinder(1.7, Y_FER + 18 - Y_TUBE_END, align=MIN))
        s -= Pos(x, Y_FER + 18 - 4.0, z) * _rot_y(Cylinder(3.0, 4.0, align=MIN))
    return s


_ST = Pos(-BELT_C, Y_F, 0) * Rot(90, 0, 0)          # stage frame → module frame (stage +Z → −Y, hub face at Y_F, axis at x = −BELT_C)
_XS, _ZS = 190.0, 150.0
_X0 = -BELT_C / 2 - 10
WEB_INSERTS_Z = (-50.0, 10.0, 70.0)                 # 3 × M4 (outer wall → web end face)


def _wall(y: float, inner: bool) -> Part:
    w = Pos(_X0, y, 10) * Rot(-90, 0, 0) * fillet(Box(_XS, _ZS, WALL_T).edges().filter_by(Axis.Z), 20)
    w -= Pos(0, y, 0) * Rot(-90, 0, 0) * Cylinder(26, WALL_T + 1)                      # 61808 seat (OD 52)
    if inner:
        from .belt_stage import wall_a_cuts
        w -= _ST * wall_a_cuts()          # D24: eccentric seat, cartridge spigot counterbore, 6 × M4 countersunk clearance
    else:
        w -= Pos(-BELT_C, y, 0) * Rot(-90, 0, 0) * Cylinder(19.1, WALL_T + 1)       # eccentric-bushing seat (Ø38) for bearing B
    # lightening (loop 8 diet): keep ≥ 10 mm of material around both bearing seats and along the back web
    for cx, cz, lx, lz in ((-75, 62, 96, 22), (-62, -52, 110, 18), (-128, 0, 14, 60)):
        w -= Pos(cx, y, cz) * Rot(-90, 0, 0) * fillet(Box(lx, lz, WALL_T + 1).edges().filter_by(Axis.Z), 5)
    return w


def bracket_body() -> Part:
    """D19: inner wall (pitch bearing A, the bolted pitch cartridge) + back web, one print. Loop 21's torque fork is gone:
    the D24 flange carries the cartridge torque into the inner wall."""
    p = _wall(W_IN, True)
    y1 = W_OUT - WALL_T / 2
    p += Pos(WEB_X, (W_IN + y1) / 2, 10) * Box(8, y1 - W_IN, _ZS)
    for z in WEB_INSERTS_Z:
        p -= Pos(WEB_X, y1, z) * Rot(90, 0, 0) * Cylinder(2.8, 8, align=MIN)           # M4 heat-set inserts in the web end
    return p


def bracket_wall_out() -> Part:
    """D19: the outer wall is its own print (pitch bearing B, stage bearing B, balancer post + spring cage), bolted to the
    web end with 3 × M4, so the one-piece pitch shaft can go in before it."""
    p = _wall(W_OUT, False)
    post_y0, post_y1 = W_OUT + WALL_T / 2, Y_BAL - 3.5                                   # post beside pulley A (axle cantilevered)
    post = Pos(0, (post_y0 + post_y1) / 2, 58) * Box(16, post_y1 - post_y0, 40)
    post -= Pos(0, (post_y0 + post_y1) / 2, 50) * Box(4.2, 40, 24)                      # vertical slot: a = 40–60 mm (balancer track 2026-11-27)
    p += post
    cage = Pos(-15, Y_BAL, 58) * Rot(0, -90, 0) * (Cylinder(14.6, 135, align=MIN) - Cylinder(13.0, 135, align=MIN))   # Ø26 bore for the ISO 10243 Ø25 die spring (18.7 N/mm)
    p += cage
    for z in WEB_INSERTS_Z:
        p -= Pos(WEB_X, W_OUT, z) * Rot(90, 0, 0) * Cylinder(2.25, 20)
    return p


def bracket() -> Part:
    """The assembled bracket (body + bolted outer wall) for the FEA and stiffness cases; the prints are bracket_body() and
    bracket_wall_out()."""
    return bracket_body() + bracket_wall_out()


def pulley_a() -> Part:
    return Pos(0, Y_BAL, 48) * Rot(-90, 0, 0) * (Cylinder(8, 6) - Cylinder(2, 6))


def module(with_motor: bool = True):
    """(name, part, colour) list in the module frame. Stage frame (belt_stage.small_stage_v3): flange plane z = 0,
    cartridge +Z; mapped so stage +Z → world −Y, flange plane at Y_F, axis at x = −BELT_C."""
    from .belt_stage import small_stage_v3, bolted_cartridge
    out = [("bracket", bracket_body(), "white"), ("bracket_wall", bracket_wall_out(), "white"), ("shaft", pitch_shaft(), "orange"),
           ("pitch_hub", pitch_hub(), "orange"), ("pulleyA", pulley_a(), "steel")]
    out += [(n, _ST * pp, c) for n, pp, c in small_stage_v3(20)]                     # D24: Oldham drive + D13 stage
    if with_motor:
        out += [(n, _ST * pp, c) for n, pp, c in bolted_cartridge()]
    return out


def masses(density=1.27):
    """D24/D19 roll-up (2026-11-27). The bolted cartridge's sleeve is counted here only by what it adds over the loop-10 can
    (nose, flange, rear cap); the motor is the actuator mass."""
    from .belt_stage import small_stage_v3
    st = dict((n, p) for n, p, _ in small_stage_v3(20))
    oldham = st["flange_coupling"].volume * 2.7e-3 * 0.8 + st["oldham_disc"].volume * 1.41e-3 + st["kx_hub_v2"].volume * density * 0.7e-3
    can_extra = (cm.can_bolted().volume + cm.can_cap_bolted().volume - cm.can().volume) * density * 0.8e-3
    m = {"bracket body + bolted outer wall (walls, web, cage)": bracket().volume / 1e6 * density * 0.6,
         "pitch shaft + integral 60T + pitch hub (balancer disc, ferrule)": (pitch_shaft().volume + pitch_hub().volume) / 1e6 * density * 0.7,
         "D24 Oldham drive (alu hub, acetal disc, ASA adapter) + can nose/flange/cap": (oldham + can_extra) / 1e3,
         "bearings 2×61808 + 2×61804 (ESTIMATE)": 0.056,
         "spring 2 N/mm + cable + pulley A + axle (ESTIMATE)": 0.09,
         "V-band clamp, stainless 3\" (ESTIMATE)": 0.11,
         "screws: 6 × M4 flange, 6 × M3 hub, 3 × M4 wall, inserts (ESTIMATE)": 0.012}
    return m
