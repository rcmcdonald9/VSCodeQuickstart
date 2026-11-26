"""Wrist + hand v0: DM-J4310 cartridge (KX56), palm with the wrist-roll cartridge S, parallel gripper, wrist camera.

Hand frame: origin on the wrist-pitch axis (Y), hand centre plane y = 0, hand points along -Z. The wrist-pitch cartridge
sits in the forearm's wrist can; its KX56 hub is on the palm's +Y face. Grasp point at z = -l_hand (130 mm).
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
WRIST_HUB_Y = 33.0          # KX56 hub face (palm +Y side); the wrist cartridge's male face arrives from +Y


def _rbox(x, y, z, r, align=(Align.CENTER, Align.CENTER, Align.CENTER)):
    b = Box(x, y, z, align=align)
    return fillet(b.edges().filter_by(Axis.Z), min(r, x / 2 - 0.2, y / 2 - 0.2))


# ---------------- cartridge D: DM-J4310 in a can with a KX56 output ----------------
def can_d() -> Part:
    od = J4310["od"] + 2 * WALL + 0.6
    h = J4310["h"] + FLOOR
    p = Cylinder(od / 2, h, align=MIN) - Pos(0, 0, FLOOR) * Cylinder(J4310["od"] / 2 + 0.3, h, align=MIN)
    for k in range(J4310["rear_holes"]):
        a = 2 * math.pi * k / J4310["rear_holes"]
        p -= Pos(J4310["rear_pcd"] / 2 * math.cos(a), J4310["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(1.7, 10)
    p -= Cylinder(9, 10)
    p -= Pos(od / 2, 0, h / 2 + 3) * Box(10, 14, h - 8)
    p -= Pos(22, 0, 0) * Cylinder(2.45, 2 * FLOOR)                 # Ø5 torque-pin seat
    return p


def adapter_d() -> Part:
    p = Cylinder(25, NECK, align=MIN) + Pos(0, 0, NECK + KX56.face_t) * Rot(180, 0, 0) * male(KX56, bore=6)
    for k in range(J4310["out_holes"]):
        a = 2 * math.pi * k / J4310["out_holes"]
        x, y = J4310["out_pcd"] / 2 * math.cos(a), J4310["out_pcd"] / 2 * math.sin(a)
        p -= Pos(x, y, 0) * Cylinder(1.7, 40)
        p -= Pos(x, y, NECK + KX56.face_t - 3) * Cylinder(3.0, 10, align=MIN)
    return p


def cartridge_d_stack() -> float:
    return FLOOR + J4310["h"] + NECK + KX56.face_t


# ---------------- palm + wrist roll ----------------
def palm() -> Part:
    from .torque import with_pin_slot
    return with_pin_slot(wrist_roll_sleeve(), _palm_body(), 1.95, 2.0)             # loop 25: slotted pad for the wrist-roll cartridge's Ø4 torque pin


def _palm_body() -> Part:
    p = Pos(0, 0, -6) * Rot(90, 0, 0) * Cylinder(30, 40)               # boss around the wrist axis (KX56 hub bolts on +Y)
    zb = SLEEVE_TOP - cs.SERVO["case_h"] - cs.SERVO["bottom"] - cs.FLOOR + 2      # palm bottom ≈ servo top face
    p += Pos(cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zb + (-6 - zb) / 2) * soft_box(64, 40, -6 - zb, 16, 10)   # looks pass v2
    # pocket for the flipped wrist-roll sleeve (slides in from the bottom, 2 screws)
    p -= Pos(cs.SERVO["l"] / 2 - cs.SERVO["shaft_off"], 0, zb - 1) * Box(cs.SERVO["l"] + 2 * (cs.FIT + cs.WALL) + 0.6,
                                                                       cs.SERVO["w"] + 2 * (cs.FIT + cs.WALL) + 0.6, SLEEVE_TOP - zb + 1.5, align=MIN)
    return p


def wrist_roll_sleeve() -> Part:
    return Pos(0, 0, SLEEVE_TOP) * Rot(180, 0, 0) * cs.sleeve()


def wrist_roll_face_z() -> float:
    return SLEEVE_TOP - cs.stack_height()


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
    z0 = wrist_roll_face_z() - KX44.gap          # KX44 hub face (gripper side)
    zb = z0 - 12.6                               # base plate top = bottom of the hub + retainer
    parts = []
    base = Pos(5, 0, zb - 5) * soft_box(104, 46, 10, 18, 3.5)
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
    tof = Pos(48, 21, zb - 19) * Box(10, 8, 4)                                  # palm ToF (VL53L4CD) beside the wrist camera, looking down
    parts.append(("palm_tof", tof, "dark"))
    cam = Pos(52.2, 0, zb - 21) * Box(24, 30, 22)                # loop 31: +2.5 mm clear of the fixed fin-ray
    parts.append(("wrist_cam", cam, "dark"))
    return parts


def assembly(jaw_deg: float = 25.0):
    """Hand frame parts: KX56 hub + collar at the wrist, palm, wrist-roll cartridge, gripper."""
    yf = WRIST_HUB_Y
    out = [("wp_hub", Pos(0, yf, 0) * Rot(-90, 0, 0) * hub(KX56), "dark"),
           ("wp_collar", Pos(0, yf, 0) * Rot(-90, 0, 0) * collar(KX56), "orange"),
           ("palm", palm(), "white"), ("wr_sleeve", wrist_roll_sleeve(), "white"),
           ("wr_collar", Pos(0, 0, wrist_roll_face_z() - KX44.gap) * collar(KX44), "orange"),
           ("gr_hub", Pos(0, 0, wrist_roll_face_z() - KX44.gap) * hub(KX44), "dark")]
    return out + gripper_parts(jaw_deg)


def masses(density=1.27):
    printed = palm().volume + sum(p.volume for n, p, c in gripper_parts() if n in ("grip_base", "grip_servo_sleeve", "jaw_moving", "jaw_fixed"))
    return {"palm + gripper printed": printed / 1e6 * density * 0.7,
            "2 × KX44/KX56 hub+collar (hand side)": (hub(KX56).volume + collar(KX56).volume + collar(KX44).volume) / 1e6 * density * 0.7,
            "wrist camera module (ESTIMATE)": 0.05, "TPU pads, rail screws (ESTIMATE)": 0.02}
