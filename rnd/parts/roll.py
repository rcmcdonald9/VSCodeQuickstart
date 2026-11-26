"""Arm-side shoulder: KX-V male ferrule → roll link (roll stage 18T:72T) → clevis → yaw stage (20T:40T) → upper-arm flange.

Frame = the twin's pitch body for the LEFT arm (outboard +Y): pitch axis = Y through the origin, detach plane at y = 0.
Roll axis = X through (y = ROLL_Y, z = 0). Yaw axis = Z through (x = 0, y = ROLL_Y), upper-arm flange top at z = -YAW_DROP.
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .belt_stage import pulley, small_hub
from .kx import KX74, collar
from .shoulder import ferrule

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
from ..params import BASELINE as _D
from .belt_stage import V2 as _V2, small_stage_v2
ROLL_Y = _D.arm.roll_offset_y * 1000                 # loop 10: 50 → 75 (the Ø114 roll pulley); loop 40: 83 (the 80T pulley clears the ferrule by 2 mm)
# loop 40 (D13 migration of roll + yaw; yaw ±150° restored with a clock-spring cassette):
#  - roll belt centre 90 → 102: the D13 yaw stage is 11 mm taller, so the yaw cartridge (roll output) reaches 48 mm from the
#    roll axis; the static roll collar/can must sit ≥ 2 mm outside that over the whole roll sweep (rnd/spacing.py swept).
#    Roll belt → stock HTD 475-5M.
#  - roll 72T → 80T (ratio 4.0 → 4.44): the D13 stages put 0.16 kg more on the shoulder, and the whole-workspace roll
#    margin (rated / p95, 1 kg, yaw ±150°) fell to 1.49×; 80T restores ≈ 1.6× with no motor change.
ROLL_TEETH = 80
ROLL_C, YAW_C = 103.3, 80.0          # 80T:18T roll on a stock 475 mm belt
_HUB_H = _V2["hub_h"]                                  # 12.2
JA, JB = _V2["jA"], _V2["jB"]                          # D13 bearing spans (stage frame, below the KX face: negative z)
PUL_C = -_V2["belt_c"]                                 # 40.2: small-pulley belt centre below the KX face


def _stage(T, teeth=20):
    """Cartridge M + collar + D13 small stage v2 in a stage frame (KX face z = 0, cartridge +Z), mapped by T."""
    zc = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    out = [("can", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.can(), "white"),
           ("motor", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.motor_dummy(), "dark"),
           ("collar", T * collar(KX74), "orange")]
    return out + [(n, T * pp, c) for n, pp, c in small_stage_v2(teeth)]


def _fl(b, r, dims):
    r = min(r, min(dims) / 2 - 0.3)
    return fillet(b.edges().filter_by(Axis.Z), r) if r > 0.5 else b


def _plate_x(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * _fl(Box(z1 - z0, y1 - y0, x1 - x0), r, (z1 - z0, y1 - y0))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Rot(0, 90, 0) * fillet(Box(z1 - z0, y1 - y0, x1 - x0).edges().filter_by(Axis.Z), r)


def _plate_z(x0, x1, y0, y1, z0, z1, r=10):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * _fl(Box(x1 - x0, y1 - y0, z1 - z0), r, (x1 - x0, y1 - y0))


# roll stage: cartridge behind (−X), KX face at x = XF; 6 mm walls with the D13 bearings flush on the pulley side
XF = -PUL_C                                            # stage z → world x = XF − z; belt plane x = 0
W1 = (XF - JA[0] - 6.0, XF - JA[0])                    # inner wall (bearing A at x −15.5 … −10.5, flush at −10.5)
W2 = (XF - JB[1], XF - JB[1] + 6.0)                    # outer wall (bearing B at x 17.5 … 22.5)
ROLL_T = Pos(XF, ROLL_Y, ROLL_C) * Rot(0, -90, 0)        # stage +Z → world −X
HEX_AF = 34.0                                          # shaft hex under the split pulleys (corners Ø39.3: passes the 61808 bore)
CAV_H = 9.5                                            # clock-spring cavity height (D14 ribbon 7.8 mm + clearance)
ROLL_SPR = (22.0, 38.0)                                # roll clock spring r_in, r_out
YAW_SPR = (22.5, 40.0)                                 # yaw clock spring (inner wall = the 40T pulley's hub)


def _hex(af, h):
    from build123d import RegularPolygon, extrude
    return extrude(RegularPolygon(af / math.sqrt(3), 6), h)


def roll_link() -> Part:
    p = Pos(0, 8, 0) * Rot(-90, 0, 0) * Cylinder(45, 4, align=MIN)            # boss behind the ferrule (y 8..12)
    for x0, x1 in (W1, W2):
        w = _plate_x(x0, x1, 8, ROLL_Y + 40, -22, ROLL_C + 32, 14)
        w -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(26, 100)                # 61808 seat, roll shaft
        w -= Pos(0, ROLL_Y, ROLL_C) * Rot(0, 90, 0) * Cylinder(19.1, 100)         # eccentric-bushing seat, D13 stage
        p += w
    p += _plate_x(W1[0], W2[1], 8, 12, 46, ROLL_C + 32, 4)                       # web joining the walls to the boss, above the roll pulley
    return p + roll_torque_fork() + roll_spring_static()


ZC = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]      # KX face → can rear


def roll_torque_fork() -> Part:
    """Loop 22: torque reaction for the roll cartridge. Its pin sits 30 mm above the cartridge axis (z = ROLL_C + 30) and
    sticks out of the can rear (−X). The inner wall rises to z 147; a 6 mm plate runs over the can (85 mm wide in Y:
    ≈ 3.8 MPa at 367 N) to a 7 mm fork plate with a slot open downward (toward the cartridge axis). Everything stays
    > 105 mm from the roll axis, outside the swept envelope of the clevis and yaw block."""
    tp = cm.TORQUE_PIN
    x_rear, zp = XF - ZC, ROLL_C + tp["r"]
    y0, y1 = 30.0, ROLL_Y + 40
    zt = ROLL_C + 54                                                        # 144 at ROLL_C 90: clears the KX collar (r 49.2)
    riser = Pos((W1[0] + W1[1]) / 2, (y0 + y1) / 2, (ROLL_C + 30 + zt + 3) / 2) * Box(W1[1] - W1[0], y1 - y0, zt + 3 - (ROLL_C + 30))
    top = Pos((x_rear - 9 + W1[1]) / 2, (y0 + y1) / 2, zt) * Box(W1[1] - (x_rear - 9), y1 - y0, 6)
    fork = Pos(x_rear - 5.5, ROLL_Y, (zp - 9 + zt) / 2) * Box(7, 48, zt - (zp - 9))
    fork -= Pos(x_rear - 5.5, ROLL_Y, zp - 9 + 7) * Box(8, tp["d"] + 0.4, 14 + 0.01)         # slot: open bottom, closed 5 mm above the pin
    return riser + top + fork


def roll_shaft() -> Part:
    """Loop 40 (R14): Ø40 tube, one piece, no integral pulley: it slides in from −X through the −X clevis arm, wall W1's
    61808, the 72T pulley (hex AF 34), wall W2's 61808 and the +X clevis arm, and carries the roll clock-spring hub on
    its +X end."""
    x0, x1 = -29.0, ROLL_SPR_X[1] - 0.5
    s = Cylinder(20, x1 - x0, align=MIN) - Cylinder(14, x1 - x0, align=MIN)
    s -= Pos(0, 0, -10 - x0) * (Cylinder(21, 20, align=MIN) - _hex(HEX_AF, 20))
    return Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * s


def roll_pulley() -> Part:
    """80T HTD 5M, 16 wide, hex bore (AF 34.4), printed PETG; located between the inner races by 2 printed shims."""
    p = pulley(ROLL_TEETH, 16, flange=False) - _hex(HEX_AF + 0.4, 16)
    r_rim = ROLL_TEETH * 5.0 / math.pi / 2 - 0.57 - 2.06 - 5.0
    for z in (0.0, 10.5):                                                    # lightening: 5 mm web between a Ø56 hub and a 5 mm rim
        p -= Pos(0, 0, z) * (Cylinder(r_rim, 5.5, align=MIN) - Cylinder(28, 5.5, align=MIN))
    return Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * p


# roll clock spring (A3): outboard of the +X clevis arm. The rotating hub (disc + sleeve) screws to the clevis arm; the
# static ring + cover hang on a bar that runs inboard (−Y, at z ≈ 0: a direction the roll output never sweeps) to the
# roll-link boss. The ribbon's static end leaves through the ring toward the detach plug; its rotating end runs down a
# groove in the clevis arm to the yaw cassette.
ROLL_SPR_X = (W2[1] + 12 + 1.5, W2[1] + 12 + 1.5 + CAV_H)   # cavity x-span (37.0 … 46.5)


def roll_spring_hub() -> Part:
    x0, x1 = ROLL_SPR_X
    h = Pos(x0 - 1.5, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ROLL_SPR[1] - 0.8, 1.5, align=MIN) - Cylinder(20.1, 1.5, align=MIN))
    h += Pos(x0, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(ROLL_SPR[0], x1 - x0 - 0.5, align=MIN) - Cylinder(20.1, x1 - x0, align=MIN))
    return h


def roll_spring_static() -> Part:
    x0, x1 = ROLL_SPR_X
    r0, r1 = ROLL_SPR
    ring = Pos(x0 + 0.5, ROLL_Y, 0) * Rot(0, 90, 0) * (Cylinder(r1 + 2, x1 - x0 + 1.6 - 0.5, align=MIN) - Cylinder(r1, x1 - x0 - 0.5, align=MIN))
    ring -= Pos(x1 - 1, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(r0 + 0.5, 5, align=MIN)      # cover clears the rotating sleeve
    ring -= Pos((x0 + x1) / 2, ROLL_Y, r1 + 1) * Box(CAV_H - 1, 10, 4)                      # ribbon exit (top)
    bar = Pos((x0 + 1 + x1 + 1.6) / 2, (12 + ROLL_Y - r1) / 2, 0) * Box(x1 + 1.6 - x0 - 1, ROLL_Y - r1 - 12 + 1, 14)
    return ring + bar


# yaw stage: yaw axis Z at (0, ROLL_Y); belt plane horizontal; cartridge above its D13 stage at x = -YAW_C.
# Loop 40: top plate 3 mm under the roll pulley; 6 mm plates with the D13 bearings flush on the pulley side. The yaw clock
# spring sits between the 40T pulley and the bottom plate (cavity 9.5 mm), so the drop added is the D13 stage's length
# and the 80T roll pulley: yaw_drop 100 → 113.0 mm.
_R_ROLL = ROLL_TEETH * 5.0 / math.pi / 2 - 0.5715       # roll pulley outer radius
P1 = (-(_R_ROLL + 3.0) - 6.0, -(_R_ROLL + 3.0))        # top plate z-span (bearing A flush on its underside)
ZF = P1[0] - JA[0]                     # KX face of the yaw cartridge (−36.0)
YAW_T = Pos(-YAW_C, ROLL_Y, ZF)
P2 = (ZF + JB[1] - 6.0, ZF + JB[1])    # bottom plate (−99.7 … −93.7), bearing B flush on its top
YAW_BELT = ZF - PUL_C                  # −76.2
YAW_DROP = round(-(P2[0] - 2.3 - 5.0), 1)   # flange: 5 mm, 2.3 mm under the bottom plate (R15 moving ≥ 2; 113.4 = params.arm.yaw_drop)


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
    t = math.radians(STOP_PIN_AZ)
    p -= Pos(STOP_PIN_R * math.cos(t), ROLL_Y + STOP_PIN_R * math.sin(t), P2[0]) * Cylinder(2.95, 5, align=MIN)   # loop 34: press-fit seat for the stop pin
    for x0, x1 in ((W1[0] - 12, W1[0] - 2), (W2[1] + 2, W2[1] + 12)):            # clevis arms outside the walls
        arm = _plate_x(x0, x1, ROLL_Y - 26, ROLL_Y + 26, P1[0], 26, 10)           # arms end on the top plate (yaw pulley below is clear)
        arm -= Pos(0, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(20.2, 100)           # clamps on the roll shaft ends
        arm -= Pos(0, ROLL_Y, 0) * Cylinder(22.0, 400)                            # loop 31/36: 2 mm clear of the rotating Ø40 yaw shaft (R15)
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
    """Loop 40 (R14): Ø40 tube + bottom flange, one piece; the 40T pulley is separate (hex AF 34). Inserted from below
    through the bottom plate's 61808, the static spring ring, the pulley and the top plate's 61808. The upper bone bolts
    to the flange inside the stop slot (6 × M4 at r 28), so no torque crosses the slot."""
    h = P1[1] + 2 - (-YAW_DROP)
    s = Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(20, h, align=MIN) - Cylinder(14, 200, align=MIN))
    z0 = P2[1]
    s -= Pos(0, ROLL_Y, z0) * (Cylinder(21, YAW_BELT + 8 - z0, align=MIN) - _hex(HEX_AF, YAW_BELT + 8 - z0))
    s += Pos(0, ROLL_Y, -YAW_DROP) * (Cylinder(45, 5, align=MIN) - Cylinder(14, 5, align=MIN))   # flange → upper bone
    # hard-stop slot (blind, 3.5 mm deep in the top of the flange) for the fixed Ø6 stop pin: software ±150°, hard ±155°
    s -= Pos(0, ROLL_Y, -YAW_DROP + 1.5) * _arc_slot(STOP_PIN_R, 3.3, STOP_PIN_AZ, YAW_STOP_DEG, 4.0)
    return s


def _arc_slot(r, hw, az, half, h):
    """Annular slot of half-width hw at radius r, ±half degrees about azimuth az (round ends), height h, from z = 0."""
    from build123d import Polygon, extrude
    ring = Cylinder(r + hw, h, align=MIN) - Cylinder(r - hw, h, align=MIN)
    keep = [(0.0, 0.0)] + [(3 * r * math.cos(math.radians(az + 180 + a)), 3 * r * math.sin(math.radians(az + 180 + a))) for a in range(-(180 - half), 180 - half + 1, 5)]
    gap = extrude(Polygon(*keep, align=None), h)                      # the solid sector the slot must NOT cut (around az + 180)
    ring -= gap
    for a in (-half, half):
        t = math.radians(az + a)
        ring += Pos(r * math.cos(t), r * math.sin(t), 0) * Cylinder(hw, h, align=MIN)
    return ring


def yaw_pulley() -> Part:
    """40T HTD 5M on the belt plane, hex bore, with a Ø45 hub below it that is the clock spring's rotating inner wall
    (r 22.5 ≥ 10 × the Ø2.2 USB cable's OD); the hub stands on bearing B's inner race."""
    p = Pos(0, 0, YAW_BELT - 8) * pulley(40, 16, flange=False)
    p += Pos(0, 0, P2[1]) * Cylinder(21.6, 1.0, align=MIN) + Pos(0, 0, P2[1] + 1.0) * Cylinder(YAW_SPR[0], YAW_BELT - 8 - P2[1] - 1.0, align=MIN)
    p -= Pos(0, 0, P2[1]) * _hex(HEX_AF + 0.4, 40)
    return Pos(0, ROLL_Y, 0) * p


def yaw_spring_ring() -> Part:
    """Static outer wall of the yaw clock spring, screwed to the bottom plate; ribbon exit toward +X (cable comes down
    the +X clevis arm and in between the plates)."""
    r1 = YAW_SPR[1]
    top = YAW_BELT - 8 - 0.8
    g = Pos(0, ROLL_Y, P2[1]) * (Cylinder(r1 + 2, top - P2[1], align=MIN) - Cylinder(r1, top - P2[1], align=MIN))
    g -= Pos(r1 + 1, ROLL_Y, P2[1] + 0.5) * Box(4, 10, CAV_H, align=MIN)
    return g


YAW_STOP_DEG = 155                    # hard stops (software limit ±150°, rnd/twin.py JR); loop 40, was ±75°
STOP_PIN_R, STOP_PIN_AZ = 38.0, 180.0 # stop pin radius and azimuth at yaw 0 (under the bottom plate, toward −X)


def yaw_stop_pin() -> Part:
    """Ø6 × 10 hardened dowel pressed 5 mm into the bottom yaw plate, protruding 1.9 + 3 mm into the flange's blind arc slot.
    A fault drive into the stop at the yaw's 22 N·m peak puts ≈ 580 N on the pin: ≈ 32 MPa bearing over 3 mm of printed
    flange, fault-only (software limit ±150°); a 3 mm stub sees ≈ 1.7 N·m bending, ≈ 80 MPa in the pin."""
    t = math.radians(STOP_PIN_AZ)
    return Pos(STOP_PIN_R * math.cos(t), ROLL_Y + STOP_PIN_R * math.sin(t), P2[0] + 5) * Cylinder(3, 9.9, align=(Align.CENTER, Align.CENTER, Align.MAX))


def shoulder_cap() -> Part:
    """Looks pass: rounded cap over the roll cartridge and the top of the roll link (pitch body; open underneath).
    It stays clear of everything that rolls: the clevis stays within 37 mm of the roll axis and the cap starts 62 mm above it."""
    from .soft import soft_shell
    dz = ROLL_C - 90                                                          # loop 40: roll belt centre 90 → 102
    y1 = ROLL_Y + 70                                                          # loop 40: follows the roll axis (83) and the collar lugs (r 52)
    c = Pos(-40, (10 + y1) / 2, (38 + 160 + dz) / 2) * soft_shell(146, y1 - 10, 122 + dz, 42, 30, 2.0)   # loop 22: +8 mm back and up over the roll torque fork
    c -= Pos(-40, (10 + y1) / 2, 38) * Box(152, y1, 4)                        # open bottom (rear part)
    c -= Pos(14, (10 + y1) / 2, 38 + 12) * Box(40, y1, 28)                    # front part opens higher (roll pulley sweep)
    c -= Pos(-8, ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(_R_ROLL + 5, 30)      # clearance around the roll pulley
    return c - roll_sweep_envelope()


def roll_sweep_envelope(pad=3.0, step=5):
    """Loop 40 (rnd/swept.py): the space the roll output sweeps over roll −15…100° (yaw block + band, yaw cartridge +
    collar, as padded boxes rotated about the roll axis). Static covers are cut by it: the first swept check found the
    shoulder cap in the path of the yaw block and cartridge at roll 55–95° (true since loop 22, invisible to static checks)."""
    from build123d import Axis
    from ..twin import JR
    boxes = [(-130 - pad, 56 + pad, ROLL_Y - 50 - pad, ROLL_Y + 50 + pad, P2[0] - pad, P1[1] + pad),          # plates + band
             (-132 - pad, -30 + pad, ROLL_Y - 53 - pad, ROLL_Y + 53 + pad, ZF - 12 - pad, ZF + ZC + 10 + pad)]   # yaw cartridge, collar, fork
    ax = Axis((0, ROLL_Y, 0), (1, 0, 0))
    env = None
    for t in list(range(JR["roll"][0] - 5, JR["roll"][1] + 6, step)):
        for x0, x1, y0, y1, z0, z1 in boxes:
            b = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
            b = b.rotate(ax, t)
            env = b if env is None else env + b
    return env


def yaw_band() -> Part:
    """Rounded band around the yaw-block plates (roll body)."""
    from .soft import soft_box
    x0, x1 = -YAW_C - 50, 56                  # loop 40: +6 at +X for the ribbon from the roll spring
    zc, hz = (P2[0] + P1[1]) / 2, P1[1] - P2[0] + 3
    b = Pos((x0 + x1) / 2, ROLL_Y, zc) * (soft_box(x1 - x0, 100, hz, 30, 6) - soft_box(x1 - x0 - 4, 96, hz + 2, 28, 0.6))
    return b


def module():
    out = [("ferrule_m", Rot(-90, 0, 0) * ferrule(male=True), "orange"),
           ("roll_link", roll_link(), "white"), ("roll_shaft", roll_shaft(), "orange"), ("roll_pulley72", roll_pulley(), "orange"),
           ("roll_spring_hub", roll_spring_hub(), "dark"),
           ("yaw_block", clevis_and_yaw_block(), "white"), ("yaw_shaft", yaw_shaft(), "orange"), ("yaw_pulley40", yaw_pulley(), "orange"),
           ("yaw_spring_ring", yaw_spring_ring(), "dark")]
    out += [("shoulder_cap", shoulder_cap(), "white"), ("yaw_band", yaw_band(), "white")]
    out += [(f"roll_{n}", p, c) for n, p, c in _stage(ROLL_T, 18)]
    out += [(f"yaw_{n}", p, c) for n, p, c in _stage(YAW_T, 20)]
    out.append(("yaw_stop_pin", yaw_stop_pin(), "steel"))
    return out


def masses(density=1.27, split=False):
    """CAD roll-up (loop 40). Printed parts at 60–80 % fill; D13 standard parts by material (steel shaft, aluminium
    flange coupling and pulley, 6801 at 60 % of solid steel). split=True → (roll-body mass, yaw-stage mass) for params."""
    from .belt_stage import small_stage_v2
    st = dict((n, p) for n, p, _ in small_stage_v2(20))
    d13 = (st["shaft_12"].volume * 7.85e-3 + (st["flange_coupling"].volume + st["al_pulley"].volume) * 2.7e-3
           + (st["brg_A_6801"].volume + st["brg_B_6801"].volume) * 7.85e-3 * 0.6) / 1e3          # one stage, kg
    sock = (st["kx_hub_v2"].volume + collar(KX74).volume) * 0.7 / 1e6 * density
    pr = lambda v, f: v * f / 1e6 * density
    roll = {"roll link + spring ring/bar (printed)": pr(roll_link().volume, 0.6), "roll shaft + 72T pulley + spring hub": pr(roll_shaft().volume + roll_pulley().volume, 0.7) + pr(roll_spring_hub().volume, 0.8),
            "roll D13 stage + KX74 socket": d13 + sock, "2 × 61808 (ESTIMATE)": 0.04, "male ferrule (printed) + dowels + belt + screws + ribbon (ESTIMATE)": pr(67213, 0.7) + 0.05}
    yaw = {"clevis + yaw block plates + posts (printed)": pr(clevis_and_yaw_block().volume, 0.6),
           "yaw shaft + flange + 40T pulley + spring ring": pr(yaw_shaft().volume + yaw_pulley().volume, 0.7) + pr(yaw_spring_ring().volume, 0.8),
           "yaw D13 stage + KX74 socket": d13 + sock, "2 × 61808 + stop pin + belt + screws + ribbon (ESTIMATE)": 0.075}
    if split:
        return sum(roll.values()), sum(yaw.values())
    return {**{"roll: " + k: v for k, v in roll.items()}, **{"yaw: " + k: v for k, v in yaw.items()}}
