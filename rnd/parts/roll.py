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
# small-hub journal spans below the KX face (stage frame), derived from the hub geometry (confidence campaign: the old
# constants 10.2/18.2, 35.6/43.6, 26.8 predate the taller print-in-place hub and cut 2–3 mm into the hub and pulley)
from .kx import hub_pip as _hub_pip
_HUB_H = -_hub_pip(KX74).bounding_box().min.Z          # 12.2
J1 = (_HUB_H + 0.4, _HUB_H + 7.8)                      # top journal 8 mm, bearing in a 7 mm eccentric bushing
J2 = (_HUB_H + 8 + 1.2 + 16 + 1.2 + 0.3, _HUB_H + 33.4 + 0.4)
PUL_C = _HUB_H + 8 + 1.2 + 8                           # small-pulley belt centre below the KX face


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
        w -= Pos(0, ROLL_Y, ROLL_C) * Rot(0, 90, 0) * Cylinder(19.1, 40)          # eccentric-bushing seat, small hub
        p += w
    p += _plate_x(W1[0], W2[1], 8, 12, 46, ROLL_C + 32, 4)                       # web joining the walls to the boss, above the roll pulley
    return p + roll_torque_fork()


ZC = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]      # KX face → can rear


def roll_torque_fork() -> Part:
    """Loop 22: torque reaction for the roll cartridge. Its pin sits 30 mm above the cartridge axis (z = ROLL_C + 30) and
    sticks out of the can rear (−X). The inner wall rises to z 147; a 6 mm plate runs over the can (85 mm wide in Y:
    ≈ 3.8 MPa at 367 N) to a 7 mm fork plate with a slot open downward (toward the cartridge axis). Everything stays
    > 105 mm from the roll axis, outside the swept envelope of the clevis and yaw block."""
    tp = cm.TORQUE_PIN
    x_rear, zp = XF - ZC, ROLL_C + tp["r"]
    y0, y1 = 30.0, ROLL_Y + 40
    riser = Pos((W1[0] + W1[1]) / 2, (y0 + y1) / 2, (ROLL_C + 32 - 2 + 147) / 2) * Box(W1[1] - W1[0], y1 - y0, 147 - (ROLL_C + 30))
    top = Pos((x_rear - 9 + W1[1]) / 2, (y0 + y1) / 2, 144) * Box(W1[1] - (x_rear - 9), y1 - y0, 6)          # clears the KX collar (r 49.2 → z 139.2)
    fork = Pos(x_rear - 5.5, ROLL_Y, (zp - 9 + 144) / 2) * Box(7, 48, 144 - (zp - 9))
    fork -= Pos(x_rear - 5.5, ROLL_Y, zp - 9 + 7) * Box(8, tp["d"] + 0.4, 14 + 0.01)         # slot: open bottom, closed 5 mm above the pin
    return riser + top + fork


def roll_shaft() -> Part:
    s = Pos(-29, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(20, 58, align=MIN) - Cylinder(14, 58, align=MIN))
    s += Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * pulley(72, 16, flange=False, bore=28)
    return s


# yaw stage: yaw axis Z at (0, ROLL_Y); belt plane horizontal; cartridge above its hub at x = -YAW_C
ZF = -YAW_DROP + 52.9                  # KX face of the yaw cartridge: bottom plate 1.9 mm above the flange, top plate 2.3 mm below the roll pulley (journals re-derived: +2.4)
YAW_T = Pos(-YAW_C, ROLL_Y, ZF)
P1 = (ZF - J1[1], ZF - J1[0])          # top plate z-span
P2 = (ZF - J2[1], ZF - J2[0])          # bottom plate z-span


def clevis_and_yaw_block() -> Part:
    p = None
    for z0, z1 in (P1, P2):
        pl = _plate_z(-YAW_C - 48, 48, ROLL_Y - 46, ROLL_Y + 46, z0, z1, 16)
        pl -= Pos(0, ROLL_Y, 0) * Cylinder(26, 400)                                # 61808 seat, yaw shaft
        pl -= Pos(-YAW_C, ROLL_Y, 0) * Cylinder(19.1, 400)                        # eccentric-bushing seat, small hub
        p = pl if p is None else p + pl
    # confidence campaign (FEA): the bottom plate was a free solid. Four Ø14 posts outside the belt loop tie it to the top plate
    # (M5 through-bolts; the belt and both pulleys go in before the bottom plate)
    for xx, yy in ((-YAW_C / 2, ROLL_Y - 38), (-YAW_C / 2, ROLL_Y + 38), (-YAW_C - 24, ROLL_Y - 38), (-YAW_C - 24, ROLL_Y + 38)):
        p += Pos(xx, yy, P2[1]) * (Cylinder(7, P1[0] - P2[1], align=MIN) - Cylinder(2.75, P1[0] - P2[1], align=MIN))
    p += yaw_torque_fork()
    for x0, x1 in ((W1[0] - 12, W1[0] - 2), (W2[1] + 2, W2[1] + 12)):            # clevis arms outside the walls
        arm = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[0], 26, 10)           # arms end on the top plate (yaw pulley below is clear)
        arm -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.2, 100)           # clamps on the roll shaft ends
        arm -= Pos(0, ROLL_Y, 0) * Cylinder(21.0, 400)                            # loop 31: 1 mm clear of the rotating Ø40 yaw shaft
        p += arm
    return p


def yaw_torque_fork() -> Part:
    """Loop 22: torque reaction for the yaw cartridge (axis Z at x = −YAW_C). Its pin sits 30 mm toward the yaw axis
    (x = −YAW_C + 30) above the can rear (z = ZF + ZC). A 7 mm fork plate, open toward the cartridge axis (−X), is fused to
    the inboard clevis arm; the roll-shaft envelope is cut as in the arm. In-plane bending ≈ 2.8 MPa at 367 N."""
    tp = cm.TORQUE_PIN
    xp, z_rear = -YAW_C + tp["r"], ZF + ZC
    xe = W1[0] - 7
    f = Pos((xp - 9 + xe) / 2, ROLL_Y, z_rear + 5.5) * Box(xe - (xp - 9), 48, 7)
    f -= Pos(xp - 9 + 7, ROLL_Y, z_rear + 5.5) * Box(14 + 0.01, tp["d"] + 0.4, 8)             # open slot toward −X, closed 5 mm past the pin
    f -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.2, 60)                 # roll-shaft envelope, only where the shaft is (x ±30)
    return f


def yaw_shaft() -> Part:
    s = Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(20, P1[1] - (-YAW_DROP) + 4, align=MIN) - Cylinder(14, 200, align=MIN))
    s += Pos(0, ROLL_Y, ZF - PUL_C - 8) * pulley(40, 16, flange=False, bore=28)          # on the small pulley's belt plane
    s += Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(45, 5, align=MIN) - Cylinder(14, 5, align=MIN))   # flange → upper bone
    return s


def shoulder_cap() -> Part:
    """Looks pass: rounded cap over the roll cartridge and the top of the roll link (pitch body; open underneath).
    It stays clear of everything that rolls: the clevis stays within 37 mm of the roll axis and the cap starts 62 mm above it."""
    from .soft import soft_shell
    c = Pos(-38, (10 + 140) / 2, (38 + 160) / 2) * soft_shell(142, 130, 122, 42, 30, 2.0)   # loop 22: +8 mm back and up over the roll torque fork
    c -= Pos(-38, 75, 38) * Box(148, 140, 4)                                  # open bottom (rear part)
    c -= Pos(14, 75, 38 + 12) * Box(40, 140, 28)                              # front part opens higher (roll pulley sweep)
    c -= Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(62, 30)               # clearance around the roll pulley
    return c


def yaw_band() -> Part:
    """Rounded band around the yaw-block plates (roll body)."""
    from .soft import soft_box
    x0, x1 = -YAW_C - 50, 50
    zc, hz = (P2[0] + P1[1]) / 2, P1[1] - P2[0] + 3
    b = Pos((x0 + x1) / 2, ROLL_Y, zc) * (soft_box(x1 - x0, 100, hz, 30, 6) - soft_box(x1 - x0 - 4, 96, hz + 2, 28, 0.6))
    return b


def module():
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link(), "white"), ("roll_shaft", roll_shaft(), "orange"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("yaw_shaft", yaw_shaft(), "orange")]
    out += [("shoulder_cap", shoulder_cap(), "white"), ("yaw_band", yaw_band(), "white")]
    out += [(f"roll_{n}", p, c) for n, p, c in _stage(ROLL_T)]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T)]
    return out


def masses(density=1.27):
    printed = roll_link().volume * 0.6 + clevis_and_yaw_block().volume * 0.6 + (roll_shaft().volume + yaw_shaft().volume) * 0.7
    socket = 2 * (small_hub().volume + collar(KX74).volume) * 0.7
    return {"roll link + clevis/yaw block + shafts/pulleys (printed)": printed / 1e6 * density,
            "2 small hubs + KX74 sockets": socket / 1e6 * density,
            "bearings 4×61808 + 4×61804 (ESTIMATE)": 0.112, "male ferrule + dowels + belts + screws (ESTIMATE)": 0.09}
