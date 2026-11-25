"""Shoulder-pitch module v0 (torso side of the arm-detach plane). Left arm; outboard = +Y. Pitch axis = Y at the origin.

- Pitch cartridge M (GIM6010-8) behind the axis (x = -BELT_C), its output facing outboard, on a small-pulley hub.
- HTD 5M 20T → 50T belt (2.5:1) in the belt plane; the 50T pulley sits on the pitch shaft (Ø40, 2 × 61808 in the bracket walls).
- Balancer plane, outboard of the bracket: a lever on the pitch shaft carries anchor pin B (30 mm); pulley A rides in a vertical slot
  45–65 mm above the axis; the cable runs back over A into a horizontal cage with a 2.7 N/mm compression spring (rnd/balancer.py).
- Outboard end of the shaft: the **KX-V ferrule** (3" sanitary profile, 3 dowels): the arm clamps on here with a V-band.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley, small_hub
from .kx import KX74, collar

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
WALL = 1.6
BELT_C = 70.0
# Layout along Y follows the small-pulley hub stack (hub face at Y_F; cartridge inboard of it):
Y_F = -58.0                # KX74 hub face of the pitch cartridge
W_IN, W_OUT = -43.8, -17.9 # bracket wall centres (8 mm walls; each holds a 61804 for the small hub and a 61808 for the shaft)
Y_BELT = -31.2             # belt plane
Y_BAL = -6.0               # balancer plane (outboard of the outer wall)
Y_FER = 2.0                # ferrule (detach plane) start
FERRULE = dict(od=91.0, t=8.0, taper_deg=20.0, dowel_r=30.0)


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
    """Ø40 hollow shaft: 50T pulley in the belt plane, balancer disc with pin B, ferrule on the outboard end."""
    L = Y_FER - (W_IN - 4)
    s = Pos(0, W_IN - 4, 0) * _rot_y(Cylinder(20, L, align=MIN) - Cylinder(15, L, align=MIN))
    s += Pos(0, Y_BELT - 8, 0) * _rot_y(pulley(50, 16, flange=False, bore=30))   # flanges on the small pulley only
    disc = Cylinder(25, 5, align=MIN) - Cylinder(15, 5, align=MIN)
    disc += Pos(0, -17, 0) * Box(14, 26, 5, align=MIN)                      # lever
    disc += Pos(0, -30, 0) * Cylinder(3, 9, align=MIN)                      # pin B, 30 mm from the axis (points down when the arm hangs)
    s += Pos(0, Y_BAL - 2.5, 0) * _rot_y(disc)
    s += Pos(0, Y_FER, 0) * _rot_y(ferrule(male=False))
    return s


def bracket() -> Part:
    """Printed bracket: two walls with bearing bosses + a back web + the balancer slot + spring cage."""
    wall_t = 8.0
    xs, zs = 190.0, 150.0
    x0 = -BELT_C / 2 - 10
    walls = None
    for y in (W_IN, W_OUT):
        w = Pos(x0, y, 10) * Rot(-90, 0, 0) * fillet(Box(xs, zs, wall_t).edges().filter_by(Axis.Z), 20)
        w -= Pos(0, y, 0) * Rot(-90, 0, 0) * Cylinder(26, wall_t + 1)                  # 61808 seat (OD 52)
        w -= Pos(-BELT_C, y, 0) * Rot(-90, 0, 0) * Cylinder(16, wall_t + 1)            # 61804 seat (OD 32) for the small hub
        w -= Pos(x0, y, 10) * Rot(-90, 0, 0) * Pos(-50, 40, 0) * Cylinder(14, wall_t + 1)   # lightening
        walls = w if walls is None else walls + w
    web = Pos(x0 - xs / 2 + 4, (W_IN + W_OUT) / 2, 10) * Box(8, W_OUT - W_IN, zs)
    p = walls + web
    # balancer slot: pulley A axle rides 35–65 mm above the axis in the inboard wall
    post_y0, post_y1 = W_OUT + 4, Y_BAL - 3.5                                     # post beside pulley A (axle cantilevered)
    post = Pos(0, (post_y0 + post_y1) / 2, 58) * Box(16, post_y1 - post_y0, 40)
    post -= Pos(0, (post_y0 + post_y1) / 2, 55) * Box(4.2, 40, 24)                      # vertical slot: a = 45–65 mm
    p += post
    # spring cage: horizontal tube running back from pulley A (cable leaves A's top toward -X), above the small hub
    cage = Pos(-15, Y_BAL, 58) * Rot(0, -90, 0) * (Cylinder(14, 135, align=MIN) - Cylinder(14 - WALL, 135, align=MIN))
    return p + cage


def pulley_a() -> Part:
    return Pos(0, Y_BAL, 50) * Rot(-90, 0, 0) * (Cylinder(8, 6) - Cylinder(2, 6))


def module(with_motor: bool = True):
    """(name, part, colour) list in the module frame. Stage frame (as in arm.assembly): hub face at z=0, cartridge +Z;
    mapped so stage +Z → world -Y, hub face at Y_F, axis at x = -BELT_C."""
    st = Pos(-BELT_C, Y_F, 0) * Rot(90, 0, 0)
    zc = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    out = [("bracket", bracket(), "white"), ("shaft", pitch_shaft(), "orange"), ("pulleyA", pulley_a(), "steel"),
           ("smallhub", st * small_hub(), "dark"), ("collar", st * collar(KX74), "orange")]
    if with_motor:
        out += [("can", st * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.can(), "white"),
                ("motor", st * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.motor_dummy(), "dark")]
    return out


def masses(density=1.27):
    m = {"bracket (walls 8 mm, web, cage)": bracket().volume / 1e6 * density * 0.6,
         "pitch shaft + 50T pulley + balancer disc + ferrule": pitch_shaft().volume / 1e6 * density * 0.7,
         "small-pulley hub + KX74 socket (collar, retainer)": (small_hub().volume + collar(KX74).volume) / 1e6 * density * 0.7,
         "bearings 2×61808 + 2×61804 (ESTIMATE)": 0.056,
         "spring 2 N/mm + cable + pulley A + axle (ESTIMATE)": 0.09,
         "V-band clamp, stainless 3\" (ESTIMATE)": 0.11}
    return m
