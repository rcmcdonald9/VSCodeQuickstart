"""Arm-side shoulder: KX-V male ferrule → roll link (roll stage 18T:72T) → clevis → yaw stage (20T:40T) → upper-arm flange.

Frame = the twin's pitch body for the LEFT arm (outboard +Y): pitch axis = Y through the origin, detach plane at y = 0.
Roll axis = X through (y = ROLL_Y, z = 0). Yaw axis = Z through (x = 0, y = ROLL_Y), upper-arm flange top at z = -YAW_DROP.
"""
from __future__ import annotations

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley, small_hub
from .kx import KX74, collar
from .shoulder import ferrule

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
from ..params import BASELINE as _D
ROLL_Y, YAW_DROP = _D.arm.roll_offset_y * 1000, _D.arm.yaw_drop * 1000   # loop 10: 50 → 75, 70 → 100 (the Ø114 roll pulley)
ROLL_C, YAW_C = 90.0, 80.0             # belt centres (rnd/belts.py); yaw 70 → 80 so the yaw collar clears the clevis
J1, J2 = (10.2, 18.2), (35.6, 43.6)    # small-hub journal spans below the KX face (stage frame)
PUL_C = 26.8                           # small-pulley centre below the KX face


def _stage(T):
    """Cartridge M + collar + small hub in a stage frame (KX face z = 0, cartridge +Z), mapped by T."""
    zc = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    return [("can", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.can(), "white"),
            ("motor", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.motor_dummy(), "dark"),
            ("collar", T * collar(KX74), "orange"), ("smallhub", T * small_hub(), "dark")]


def _fl(b, r, dims):
    r = min(r, min(dims) / 2 - 0.3)
    return fillet(b.edges().filter_by(Axis.Z), r) if r > 0.5 else b


def _plate_x(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * _fl(Box(z1 - z0, y1 - y0, x1 - x0), r, (z1 - z0, y1 - y0))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * fillet(Box(z1 - z0, y1 - y0, x1 - x0).edges().filter_by(Axis.Z), r)


def _plate_z(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * _fl(Box(x1 - x0, y1 - y0, z1 - z0), r, (x1 - x0, y1 - y0))


# roll stage: cartridge behind (−X), KX face at x = XF; walls at the small-hub journals
XF = -PUL_C
W1 = (XF + J1[0], XF + J1[1])          # inner wall x-span
W2 = (XF + J2[0], XF + J2[1])          # outer wall x-span
ROLL_T = Pos(XF, ROLL_Y, ROLL_C) * Rot(0, -90, 0)        # stage +Z → world −X


def roll_link() -> Part:
    p = Pos(0, 8, 0) * Rot(-90, 0, 0) * Cylinder(45, 4, align=MIN)            # boss behind the ferrule (y 8..12)
    for x0, x1 in (W1, W2):
        w = _plate_x(x0, x1, 8, ROLL_Y + 40, -22, ROLL_C + 32, 14)
        w -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(26, 40)                 # 61808 seat, roll shaft
        w -= Pos(0, ROLL_Y, ROLL_C) * Rot(0, 90, 0) * Cylinder(16, 40)            # 61804 seat, small hub
        p += w
    p += _plate_x(W1[0], W2[1], 8, 12, 46, ROLL_C + 32, 4)                       # web joining the walls to the boss, above the roll pulley
    return p


def roll_shaft() -> Part:
    s = Pos(-29, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(20, 58, align=MIN) - Cylinder(14, 58, align=MIN))
    s += Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * pulley(72, 16, flange=False, bore=28)
    return s


# yaw stage: yaw axis Z at (0, ROLL_Y); belt plane horizontal; cartridge above its hub at x = -YAW_C
ZF = -YAW_DROP + 50.5                  # KX face of the yaw cartridge: bottom plate 1.9 mm above the flange, top plate 2.3 mm below the roll pulley
YAW_T = Pos(-YAW_C, ROLL_Y, ZF)
P1 = (ZF - J1[1], ZF - J1[0])          # top plate z-span
P2 = (ZF - J2[1], ZF - J2[0])          # bottom plate z-span


def clevis_and_yaw_block() -> Part:
    p = None
    for z0, z1 in (P1, P2):
        pl = _plate_z(-YAW_C - 48, 48, ROLL_Y - 46, ROLL_Y + 46, z0, z1, 16)
        pl -= Pos(0, ROLL_Y, 0) * Cylinder(26, 400)                                # 61808 seat, yaw shaft
        pl -= Pos(-YAW_C, ROLL_Y, 0) * Cylinder(16, 400)                          # 61804 seat, small hub
        p = pl if p is None else p + pl
    for x0, x1 in ((W1[0] - 12, W1[0] - 2), (W2[1] + 2, W2[1] + 12)):            # clevis arms outside the walls
        arm = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[0], 26, 10)           # arms end on the top plate (yaw pulley below is clear)
        arm -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.2, 100)           # clamps on the roll shaft ends
        p += arm
    return p


def yaw_shaft() -> Part:
    s = Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(20, P1[1] - (-YAW_DROP) + 4, align=MIN) - Cylinder(14, 200, align=MIN))
    s += Pos(0, ROLL_Y, P2[1] + 0.6) * pulley(40, 16, flange=False, bore=28)
    s += Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(45, 5, align=MIN) - Cylinder(14, 5, align=MIN))   # flange → upper bone
    return s


def module():
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link(), "white"), ("roll_shaft", roll_shaft(), "orange"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("yaw_shaft", yaw_shaft(), "orange")]
    out += [(f"roll_{n}", p, c) for n, p, c in _stage(ROLL_T)]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T)]
    return out


def masses(density=1.27):
    printed = roll_link().volume * 0.6 + clevis_and_yaw_block().volume * 0.6 + (roll_shaft().volume + yaw_shaft().volume) * 0.7
    socket = 2 * (small_hub().volume + collar(KX74).volume) * 0.7
    return {"roll link + clevis/yaw block + shafts/pulleys (printed)": printed / 1e6 * density,
            "2 small hubs + KX74 sockets": socket / 1e6 * density,
            "bearings 4×61808 + 4×61804 (ESTIMATE)": 0.112, "male ferrule + dowels + belts + screws (ESTIMATE)": 0.09}
