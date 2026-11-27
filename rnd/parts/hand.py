"""Wrist + hand v0: DM-J4310 cartridge (bolted, D24), palm with the wrist-roll cartridge S, parallel gripper, wrist camera.

Hand frame: origin on the wrist-pitch axis (Y), hand centre plane y = 0, hand points along -Z. The wrist-pitch cartridge
sits in the forearm's wrist can, bolted to its rim (6 × M3); the palm bolts to the cartridge's output flange (6 × M3
through the palm boss into nuts captive in the flange; D24, 2026-11-27: the KX56 socket is gone, and with it D21's wrist
lift-off problem). Grasp point
at z = -l_hand.
- Palm: holds cartridge S #1 (wrist roll) with its shaft along -Z; its KX44 face is at the palm bottom.
- Gripper (on the wrist-roll KX44 hub): SO-100-style: a fixed jaw + a jaw on the horn of cartridge S #2, which lies on
  its side under the base plate. Wrist camera on the base, looking along -Z past the fixed jaw.
DM-J4310 bolt circles are ESTIMATEs (same approach as the GIM: adapter-only dependency, slotted coupon).
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_s as cs
from .soft import soft_box
from .kx import KX44, KX56, collar, hub_pip as hub, male

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
J4310 = dict(od=57.0, h=46.0, out_holes=6, out_pcd=24.0, rear_holes=4, rear_pcd=46.0)   # PCDs ESTIMATE
WALL, FLOOR, NECK = 2.4, 3.0, 4.0
SLEEVE_TOP = -32.0            # palm: wrist-roll sleeve floor (flipped, horn pointing -Z)
from ..params import BASELINE as _D
L_HAND = _D.arm.l_hand * 1000   # wrist-pitch axis → grasp point (params; loop 10 CAD: 0.13 → 0.15)
WRIST_HUB_Y = 33.0          # loop-10 KX56 hub face (palm +Y side); kept as the datum that places the cartridge (arm.assembly)
# Fable review follow-up (2026-11-27, wave 5): the palm screws went into heat-set inserts in the 4 mm output flange, pressed
# in from the palm side, so the hand's prying load (and the tightening itself) pulled them out toward the palm, as C1 and
# C3 found elsewhere. Now a through-bolt, as the wrist can flange (arm.WRIST_NUT): ISO 4762 M3 × 60 from the palm's −Y face
# through the boss and the flange into ISO 4032 M3 nuts (s 5.5, m 2.4) captive in hex pockets on the flange's motor-side
# face (dropped in before the flange goes onto the motor; recessed, so nothing touches the motor). The flange grew 4 → 6 mm
# (the palm boss is 2 mm shorter; the hand does not move) for a 2.4 mm floor under each nut, as the rim's: palm and flange
# are both in compression, no insert anywhere on the hand load path.
ADAPTER_T = 6.0             # D24 output flange on the motor output (4 → 6: nut pockets on its back face)
PALM_FACE_Y = WRIST_HUB_Y + KX56.gap + NECK + KX56.face_t - ADAPTER_T   # 37.65: palm boss face = output flange face
CAN_FL = dict(r_out=40.0, t=4.0, bolt_r=36.0, clr_d=3.6)                 # can flange: 6 × M3 × 12 through the forearm rim into nuts (D24 v2, arm.WRIST_NUT)
PALM_SCREW_R = 19.0         # 6 × M3 × 60 from the palm's −Y face through the boss and the output flange into captive nuts
PALM_Y_BACK = -20.5         # palm's −Y face (the screw heads' side)
# nut pocket 3.6 deep from the flange's back face (floor 2.4 mm, as arm.WRIST_NUT's rim): the nut sits on the floor and the
# screw tip ends 0.7 mm inside the back face, so the M3 × 60's ± 0.6 length tolerance (js15, ISO 4759-1, as roll.REAR_SCREW)
# neither reaches the motor (≥ 0.1 mm under the face) nor leaves the nut short of thread (≥ 2.3 of its 2.4 mm engaged)
PALM_NUT = dict(af=5.6, pocket=3.6, clr_d=3.4, cb_d=6.4, L=60.0, L_tol=0.6, tip_in=0.7,
                screw="ISO 4762 M3 × 60 + ISO 4032 M3 nut, captive in a hex pocket on the output flange's motor side")
PALM_NUT["cb_depth"] = round(PALM_FACE_Y + ADAPTER_T - PALM_NUT["tip_in"] - PALM_NUT["L"] - PALM_Y_BACK, 2)   # head counterbore in the palm's −Y face (3.45)


def _rbox(x, y, z, r, align=(Align.CENTER, Align.CENTER, Align.CENTER)):
    b = Box(x, y, z, align=align)
    return fillet(b.edges().filter_by(Axis.Z), min(r, x / 2 - 0.2, y / 2 - 0.2))


# ---------------- cartridge D: DM-J4310 in a can, bolted (D24) ----------------
def _can_d_loop10() -> Part:
    od = J4310["od"] + 2 * WALL + 0.6
    h = J4310["h"] + FLOOR
    p = Cylinder(od / 2, h, align=MIN) - Pos(0, 0, FLOOR) * Cylinder(J4310["od"] / 2 + 0.3, h, align=MIN)
    for k in range(J4310["rear_holes"]):
        a = 2 * math.pi * k / J4310["rear_holes"]
        p -= Pos(J4310["rear_pcd"] / 2 * math.cos(a), J4310["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(1.7, 10)
    p -= Cylinder(9, 10)
    p -= Pos(od / 2, 0, h / 2 + 3) * Box(10, 14, h - 8)
    return p


def can_d_loop10_volume() -> float:
    return _can_d_loop10().volume - math.pi * 2.45 ** 2 * FLOOR    # with its old torque-pin seat


def can_d() -> Part:
    """D24: the can runs 4.65 mm past the motor face into a Ø80 × 4 front flange; 6 × M3 from the palm side through the
    flange and the forearm's wrist rim into nuts in hex pockets on its back face (D24 v2, arm.WRIST_NUT), the can's Ø62.4 OD is the spigot in the rim bore (0.3 mm a
    side). No torque pin (the flange carries the 7 N·m peak: ≈ 32 N per screw); the hand's bending goes motor bearings →
    can → flange → forearm, with no lift-off interface in between."""
    od = J4310["od"] + 2 * WALL + 0.6
    h = J4310["h"] + FLOOR
    front = cartridge_d_stack() + WRIST_HUB_Y + KX56.gap - WRIST_Y0_FOREARM + CAN_FL["t"]   # flange front face (can frame, 53.65)
    p = _can_d_loop10()
    p += Pos(0, 0, h) * (Cylinder(od / 2, front - h, align=MIN) - Cylinder(J4310["od"] / 2 + 0.3, front - h, align=MIN))
    fl = Pos(0, 0, front - CAN_FL["t"]) * (Cylinder(CAN_FL["r_out"], CAN_FL["t"], align=MIN) - Cylinder(od / 2 - 0.1, CAN_FL["t"], align=MIN))
    for k in range(6):
        a = math.pi / 3 * k + math.pi / 6
        fl -= Pos(CAN_FL["bolt_r"] * math.cos(a), CAN_FL["bolt_r"] * math.sin(a), 0) * Cylinder(CAN_FL["clr_d"] / 2, 400)
    return p + fl


WRIST_Y0_FOREARM = 43.0     # forearm wrist-rim face (arm.WRIST_Y0): the can flange's back face sits on it


def adapter_d() -> Part:
    """D24 output flange (printed ASA, 6 mm) on the DM-J4310 output: 6 × M3 into the motor (PCD 24, counterbored on the
    palm side, 2 mm under the heads), 6 × Ø3.4 clearance holes at r 19 for the palm screws with hex nut pockets on the
    motor-side face (PALM_NUT; a flat toward the axis), a Ø16 × 2 spigot into the palm boss."""
    from build123d import RegularPolygon, extrude
    p = Cylinder(25, ADAPTER_T, align=MIN) + Pos(0, 0, ADAPTER_T) * Cylinder(8, 2, align=MIN)
    for k in range(J4310["out_holes"]):
        a = 2 * math.pi * k / J4310["out_holes"]
        x, y = J4310["out_pcd"] / 2 * math.cos(a), J4310["out_pcd"] / 2 * math.sin(a)
        p -= Pos(x, y, 0) * Cylinder(1.7, 40)
        p -= Pos(x, y, 2.0) * Cylinder(3.0, 10, align=MIN)
    n = PALM_NUT
    hexp = extrude(RegularPolygon(n["af"] / math.sqrt(3), 6), n["pocket"] + 0.01)
    for k in range(6):
        a = math.pi / 3 * k
        at = Pos(PALM_SCREW_R * math.cos(a), PALM_SCREW_R * math.sin(a), 0)
        p -= at * Cylinder(n["clr_d"] / 2, ADAPTER_T, align=MIN)
        p -= at * Pos(0, 0, -0.01) * Rot(0, 0, 30.0 + math.degrees(a)) * hexp           # nut pocket from the motor side, a flat toward the axis
    return p - Cylinder(3, 40)


def cartridge_d_stack() -> float:
    return FLOOR + J4310["h"] + NECK + KX56.face_t


# ---------------- palm + wrist roll ----------------
def palm() -> Part:
    from .torque import with_pin_slot
    return with_pin_slot(wrist_roll_sleeve(), _palm_body(), 1.95, 2.0) - palm_screw_cuts()   # loop 25: slotted pad for the wrist-roll cartridge's Ø4 torque pin


def palm_screw_cuts() -> Part:
    """D24: 6 × M3 through the palm boss (head counterbores PALM_NUT cb_depth deep in its −Y face) and the Ø16 spigot
    recess in the boss face."""
    c = Pos(0, PALM_FACE_Y + 0.01, 0) * Rot(90, 0, 0) * Cylinder(8.1, 2.2, align=MIN)
    for k in range(6):
        a = math.pi / 3 * k
        x, z = PALM_SCREW_R * math.cos(a), PALM_SCREW_R * math.sin(a)
        c += Pos(x, PALM_Y_BACK, z) * Rot(-90, 0, 0) * Cylinder(1.7, PALM_FACE_Y - PALM_Y_BACK + 0.5, align=MIN)
        c += Pos(x, PALM_Y_BACK - 0.5, z) * Rot(-90, 0, 0) * Cylinder(PALM_NUT["cb_d"] / 2, PALM_NUT["cb_depth"] + 0.5, align=MIN)
    return c


def _palm_body() -> Part:
    p = Pos(0, 0, -6) * Rot(90, 0, 0) * Cylinder(30, 40)               # boss around the wrist axis
    p += Pos(0, 19.5, 0) * Rot(-90, 0, 0) * Cylinder(24, PALM_FACE_Y - 19.5, align=MIN)   # D24: boss extension to the output flange (was the KX56 hub)
    zb = SLEEVE_TOP - cs.SERVO["case_h"] - cs.SERVO["bottom"] - cs.FLOOR + 2      # palm bottom ≈ servo top face
    p += Pos(cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zb + (-6 - zb) / 2) * soft_box(64, 40, -6 - zb, 16, 10)   # looks pass v2
    # pocket for the flipped wrist-roll sleeve (slides in from the bottom, 2 screws)
    p -= Pos(cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zb - 1) * Box(cs.SERVO["l"] + 2 * (cs.FIT + cs.WALL) + 0.6,
                                                                       cs.SERVO["w"] + 2 * (cs.FIT + cs.WALL) + 0.6, SLEEVE_TOP - zb + 1.5, align=MIN)
    return p + wrist_roll_skirt()


def wrist_roll_sleeve() -> Part:
    return Pos(0, 0, SLEEVE_TOP) * Rot(180, 0, 0) * cs.sleeve()


def wrist_roll_face_z() -> float:
    return SLEEVE_TOP - cs.stack_height()


# D16 (loop 40, external review finding 5 → A4): the wrist roll no longer hangs the gripper on the servo's KX44 seat.
# With 1 kg in a horizontal hand the seat saw ≈ 0.9 N·m bending (1.3 N·m dynamic) against ≈ 0.6 N·m lift-off
# (P·r_ball/2 = 80 N × 15.5 mm / 2). The gripper now bolts (4 × M3) to a spindle that runs in 2 × 61804 in a skirt under
# the palm; the servo drives it through a two-dog coupling (torque only, floats axially). Cartridge S still slides out of
# the palm (2 screws); tools (suction cup, …) bolt to the spindle flange. The wrist-pitch KX56 stays: its bending lever is
# only the ≈ 40 mm hub offset (≈ 0.9 N·m dynamic vs ≈ 1.5 N·m lift-off).
BRG_WR = dict(d=20.0, D=32.0, B=7.0)          # 61804 (reused: the D13 stages freed the arm's 61804s)
SPINDLE_DROP = 19.3                            # wrist-roll face → gripper base top (was KX44 gap + 12.6 = 13.2)
# Wrist-roll range (2026-11-27): ±120° (params.Arm.wrist_roll_range_deg; the STS3215 is set to it in firmware, and the
# palm clock spring r 14–24 gives 302° packed vs 288° needed). Kept the palm instead of growing the spiral to r_out 27.5: the
# axis inside the palm is taken by the horn, dog and the two 61804s, so a bigger cassette would be an Ø74 ring round the
# skirt or +≈ 17 mm of hand length (l_hand, wrist torque); the parallel jaw is 180°-symmetric, ±90° covers every grasp.


def _zw():
    return wrist_roll_face_z()


def wrist_roll_skirt() -> Part:
    """Bearing skirt under the palm (part of the palm print), Ø38. Loop 43 (Astra review #2b finding 4): the seat is open
    at the bottom so both 61804s press in from below, and a screwed end cap (wrist_roll_cap) holds the lower outer race;
    the shoulder above the upper bearing stays. Loop 40's skirt had a lip on both sides of the seat: the bearings could
    not be fitted."""
    zf = _zw()
    zP = SLEEVE_TOP - cs.SERVO["case_h"] - cs.SERVO["bottom"] - cs.FLOOR + 2
    sk = Pos(0, 0, zf - 17.0) * Cylinder(19, zP + 1 - (zf - 17.0), align=MIN)
    sk -= Pos(0, 0, zf - 17.0) * Cylinder(BRG_WR["D"] / 2 + 0.05, 2 * BRG_WR["B"], align=MIN)   # bearing seats, open below
    sk -= Pos(0, 0, zf - 3.0) * Cylinder(13.0, 1.5, align=MIN)                     # shoulder over the upper outer race
    sk -= Pos(0, 0, zf - 1.5) * Cylinder(11.0, 40, align=MIN)                      # dog adapter clearance up to the servo
    for k in range(3):                                                             # 3 × M2.5 into heat-set inserts for the cap
        a = 2 * math.pi * k / 3 + math.pi / 6
        sk -= Pos(17.4 * math.cos(a), 17.4 * math.sin(a), zf - 17.0) * Cylinder(1.6, 5, align=MIN)
    return sk


def wrist_roll_cap() -> Part:
    """End cap under the skirt: holds the lower bearing's outer race; the gripper's spacer boss turns inside it."""
    zf = _zw()
    c = Pos(0, 0, zf - 18.5) * (Cylinder(19, 1.5, align=MIN) - Cylinder(13.0, 1.5, align=MIN))
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 6
        c -= Pos(17.4 * math.cos(a), 17.4 * math.sin(a), zf - 18.5) * Cylinder(1.4, 1.5, align=MIN)
    return c


def wrist_roll_parts():
    """Loop 43: the spindle goes in from above (cartridge S out) with a shoulder on the upper bearing's inner race; the
    gripper base bolts to its lower end (4 × M3) through a spacer boss that clamps the lower inner race, so the spindle
    is held both ways. Dog clearance 0.1 mm a side (was 0.4: ≈ ±2.5° lash)."""
    zf = _zw()
    zP = SLEEVE_TOP - cs.SERVO["case_h"] - cs.SERVO["bottom"] - cs.FLOOR + 2
    dog = Pos(0, 0, zf) * Cylinder(9.5, zP - 0.5 - zf, align=MIN)                  # on the Ø20 horn (4 × M3)
    dog += Pos(0, 0, zf - 3.5) * Box(4, 18, 3.5, align=MIN)
    sp = Pos(0, 0, zf - SPINDLE_DROP) * Cylinder(BRG_WR["d"] / 2, SPINDLE_DROP - 0.5, align=MIN)
    sp += Pos(0, 0, zf - 3.0) * Cylinder(11.5, 1.1, align=MIN)                     # shoulder on the upper inner race (0.4 mm under the skirt step)
    sp -= Pos(0, 0, zf - 4.0) * Box(4.2, 30, 10, align=MIN)                         # dog slot
    sp -= Cylinder(4.0, 400)                                                         # wire pass (servo + tactile + ToF)
    for k in range(4):                                                               # 4 × M3 tapped / insert holes for the gripper base
        a = math.pi / 2 * k + math.pi / 4
        sp -= Pos(7.0 * math.cos(a), 7.0 * math.sin(a), zf - SPINDLE_DROP) * Cylinder(1.25, 8, align=MIN)
    brg = lambda z0: Pos(0, 0, z0) * (Cylinder(BRG_WR["D"] / 2, BRG_WR["B"], align=MIN) - Cylinder(BRG_WR["d"] / 2, BRG_WR["B"], align=MIN))
    return [("wr_dog", dog, "dark"), ("wr_spindle", sp, "steel"), ("wr_cap", wrist_roll_cap(), "white"),
            ("wr_brg_U", brg(zf - 3.0 - BRG_WR["B"]), "steel"), ("wr_brg_L", brg(zf - 17.0), "steel")]


# ---------------- fin-ray finger (TPU) ----------------
def fin_ray(top_z: float, tip_z: float, face_x: float, depth: float, thick: float = 26.0, facing: int = +1, ribs: int = 5) -> Part:
    """Fin-ray finger: triangle in the x-z plane. Vertical contact face at x = face_x (facing ±x toward the object); the back
    face slopes from depth at the top to the tip. Thin TPU walls + cross ribs make the contact face wrap around the
    object (Festo FinRay; UMI gripper). Printed in TPU 95A. thickness along y."""
    from build123d import Polygon, extrude
    s = -facing                                                    # back side direction
    pts = [(face_x, top_z), (face_x + s * depth, top_z), (face_x, tip_z)]
    outer = extrude(Polygon(*pts, align=None), thick)
    w = 1.6
    inner_pts = [(face_x + s * w, top_z - w), (face_x + s * (depth - w * 2.2), top_z - w), (face_x + s * w, tip_z + w * 3.2)]
    inner = extrude(Polygon(*inner_pts, align=None), thick)
    shell = outer - inner
    for k in range(1, ribs + 1):
        z = top_z + (tip_z - top_z) * k / (ribs + 1)
        shell += (Pos((face_x + face_x + s * depth) / 2, z, thick / 2) * Rot(0, 0, s * 12) * Box(depth * 1.2, 1.2, thick)) & inner
    res = Rot(90, 0, 0) * shell                     # extrusion direction depends on the polygon winding: re-centre on y = 0
    bb = res.bounding_box()
    return Pos(0, -(bb.min.Y + bb.max.Y) / 2, 0) * res


# ---------------- gripper ----------------
# Precedent: the SO-100 / LeRobot gripper: one fixed jaw + one jaw driven straight off the servo horn (fewest parts).
# The gripper servo lies on its side under the base plate (shaft along Y, its 24.8 mm width along Z) so the hand stays short.
def gripper_parts(jaw_deg: float = 25.0):
    zb = wrist_roll_face_z() - SPINDLE_DROP      # D16: base plate top = spindle flange top (flange in a 4 mm pocket)
    parts = []
    base = Pos(5, 0, zb - 5) * soft_box(104, 46, 10, 18, 3.5)
    base += Pos(0, 0, zb) * (Cylinder(11.5, SPINDLE_DROP - 17.0, align=MIN) - Cylinder(BRG_WR["d"] / 2 + 0.2, 10, align=MIN))   # loop 43: spacer boss on the lower inner race
    base -= Pos(0, 0, zb - 11) * Cylinder(4.0, 20, align=MIN)                     # wire pass
    parts.append(("grip_base", base, "white"))
    zs = zb - 10 - 13.0                          # servo shaft height (servo hangs under the base)
    sx = -24.0                                   # shaft x
    servo = Pos(sx + cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zs) * Box(cs.SERVO["l"] + 4, 36 + 4, 24.8 + 4)
    parts.append(("grip_servo_sleeve", servo - Pos(sx + cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zs) * Box(cs.SERVO["l"], 36, 24.8), "white"))
    parts.append(("grip_servo", Pos(sx + cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zs) * Box(cs.SERVO["l"], 36, 24.8), "dark"))
    # moving jaw: lever on the horn (+Y side) → finger under the servo, rotated by jaw_deg about the shaft
    tip = -L_HAND - zs                                           # jaw tip height relative to the shaft
    # loop 31: the lever stopped 5 mm into the base and the mount plate rubbed the servo sleeve through the jaw sweep.
    # Lever now ends 2 mm under the base, outside the base's width; plate centred on the shaft, low enough for ±40°.
    jaw = Pos(0, 27, (tip + 11) / 2) * Box(12, 6, 11 - tip)       # lever on the horn, outside the servo and the base
    jaw += Pos(-6, 10, -25.5) * Box(12, 40, 7)                   # finger mount plate: 22 mm under the shaft and only on its −x side (a point r below the shaft rises to −r·cos θ as the jaw opens under the servo)
    # gripper v2 (loop 16): fin-ray TPU finger (contact face +x) with a tactile pad window
    jaw += Pos(0, 8, 0) * fin_ray(-25.0, tip - 12, 4.5, 18, 28, +1)   # loop 31: top 25 mm under the shaft (was 19.5) so it clears the sleeve at 45°
    parts.append(("jaw_moving", Pos(sx, 0, zs) * Rot(0, -jaw_deg, 0) * jaw, "dark"))
    # fixed jaw: printed stem + fin-ray TPU finger (contact face −x) with the fingertip tactile pad behind its face
    stem = Pos(31, 0, zb - 10) * Box(10, 26, (zb - 10) + L_HAND - 25, align=(Align.CENTER, Align.CENTER, Align.MAX))
    fr = fin_ray(-L_HAND + 25, -L_HAND - 12, 21.0, 18, 28, -1)
    pad = Pos(24.5, 0, -L_HAND + 2) * Box(3, 20, 14)                     # MLX90393 + magnetic elastomer (tactile)
    parts += [("jaw_fixed", stem, "white"), ("finray_fixed", fr, "orange"), ("pad_fixed", pad, "dark")]
    parts.append(("finger_hook", finger_hook(), "dark"))
    tof = Pos(48, 26, zb - 19) * Box(10, 8, 4)                                  # loop 46: y 21 → 26, clear of the D21 hook rail                                  # palm ToF (VL53L4CD) beside the wrist camera, looking down
    parts.append(("palm_tof", tof, "dark"))
    cam = Pos(52.2, 0, zb - 21) * Box(24, 30, 22)                # loop 31: +2.5 mm clear of the fixed fin-ray
    parts.append(("wrist_cam", cam, "dark"))
    return parts


HOOK = dict(t=8.0, w=14.0, reach=9.0, lip=8.0)       # spine thickness (x), width (y), hook reach past the jaw face, foot height


def finger_hook() -> Part:
    """D21 (laundry basket, R2d): a rigid PETG-CF hook for the fixed jaw. A vertical spine runs down behind the fin-ray
    finger (bridged to the jaw stem), and a foot under the fingertip reaches 9 mm past the contact face toward the
    moving jaw. The fixed jaw goes inside the basket wall, the foot passes out under the handle bar, and the moving jaw
    closes on the bar from outside, so the 4.5 kg per hand hangs on the hook, not on TPU friction.
    Foot: 8 × 14 mm with a Ø3 steel pin along it; bending at the spine ≈ 90 N (2× static) × 28 mm = 2.5 N·m → ≈ 17 MPa in
    the printed section alone (PETG-CF in-layer ≈ 25 MPa; the pin carries most of it). Printed flat, layers along the spine."""
    h = HOOK
    top, tip = -L_HAND + 25, -L_HAND - 12
    x0 = 21.0 + 18.0 + 0.5                                   # just behind the fin-ray's back face (x 39 at its top)
    # two side rails (y ±16.5…20.5) pass either side of the wrist camera (y ±15) and the fin-ray (y ±14)
    out = None
    for sy in (-1, 1):
        yc = sy * 18.5
        rail = Pos(x0 + h["t"] / 2, yc, (top + 6 + tip - h["lip"]) / 2) * Box(h["t"], 4, top + 6 - (tip - h["lip"]))
        bridge = Pos((26 + x0 + h["t"]) / 2, yc, top + 6) * Box(x0 + h["t"] - 26, 4, 12)          # onto the jaw stem's sides
        out = rail + bridge if out is None else out + rail + bridge
    out += Pos(0, 0, top + 6) * Pos((26 + x0 + h["t"]) / 2 - 0, 0, 0) * Box(x0 + h["t"] - 26, 41, 4) - Pos(52.2, 0, top + 6) * Box(27, 33, 6)
    foot = Pos((21.0 - h["reach"] + x0 + h["t"]) / 2, 0, tip - h["lip"] / 2) * Box(x0 + h["t"] - (21.0 - h["reach"]), 41, h["lip"])
    return out + foot


def assembly(jaw_deg: float = 25.0):
    """Hand frame parts: palm (bolted to the wrist cartridge's output flange, D24), wrist-roll cartridge, gripper."""
    out = [("palm", palm(), "white"), ("wr_sleeve", wrist_roll_sleeve(), "white")]
    return out + wrist_roll_parts() + gripper_parts(jaw_deg)


def masses(density=1.27):
    printed = palm().volume + sum(p.volume for n, p, c in gripper_parts() if n in ("grip_base", "grip_servo_sleeve", "jaw_moving", "jaw_fixed"))
    return {"palm + gripper printed": printed / 1e6 * density * 0.7,
            "6 × M3 × 60 palm screws + 6 × M3 nuts (ESTIMATE)": 0.007,
            "D16 wrist-roll spindle + dog (printed) + 2 × 61804": sum(p.volume for n, p, c in wrist_roll_parts() if n in ("wr_dog", "wr_spindle")) / 1e6 * density * 0.9 + 0.016,
            "D21 finger hook (PETG-CF, 80 %) + Ø3 pin": finger_hook().volume / 1e6 * 1.3 * 0.8 + 0.004,
            "wrist camera module (ESTIMATE)": 0.05, "TPU pads, rail screws (ESTIMATE)": 0.02}
