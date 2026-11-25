"""Cartridge M: a GIM6010-8 (or a DM-J4310 with its own adapter) in a printed can.

Load path (KX v1): torque only. Output side: the KX74 male face on a motor adapter. Housing side:
a Ø6 steel torque pin that drops into a radial slot in the bone (precedent: shaft-mounted gearbox torque arm).
Only `motor_adapter` and the can's bolt circle depend on the motor; numbers marked ESTIMATE are
confirmed with `coupon_motor` (radial slots) when the first motor arrives.
"""
from __future__ import annotations

import math

from build123d import Align, Box, Cylinder, Part, Pos, Rot, Axis, SortBy, fillet

from .kx import KX74, male, balls

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MOTOR = dict(od=80.0, h=40.0,                     # with SDC104 driver (manual: Ø80 × 40)
             out_holes=8, out_pcd=34.0,           # 8×M3 on the output (count from the drawing; PCD ESTIMATE)
             rear_holes=6, rear_pcd=57.0)         # 6×M3 on the housing (count from the drawing; PCD ESTIMATE)
WALL, FLOOR = 3.0, 4.0
NECK, NECK_R = 5.0, 33.0      # spacer under the male flange: room for the collar teeth behind the lugs
TORQUE_PIN = dict(d=6.0, r=30.0, stick_out=10.0, ang=0.0)


def can() -> Part:
    od = MOTOR["od"] + 2 * WALL + 0.6
    h = MOTOR["h"] + FLOOR
    body = Cylinder(od / 2, h, align=MIN)
    body -= Pos(0, 0, FLOOR) * Cylinder(MOTOR["od"] / 2 + 0.3, h, align=MIN)
    body = fillet(body.edges().group_by(Axis.Z)[-1].sort_by(SortBy.RADIUS)[-1], 1.2)
    body = fillet(body.edges().group_by(Axis.Z)[0].sort_by(SortBy.RADIUS)[-1], 3.0)
    for k in range(MOTOR["rear_holes"]):
        a = 2 * math.pi * k / MOTOR["rear_holes"]
        body -= Pos(MOTOR["rear_pcd"] / 2 * math.cos(a), MOTOR["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(1.7, 20)
        body -= Pos(MOTOR["rear_pcd"] / 2 * math.cos(a), MOTOR["rear_pcd"] / 2 * math.sin(a), 0) * Cylinder(3.1, 3.0)
    body -= Cylinder(13, 20)                                              # rear vent / cable pass-through
    body -= Pos(od / 2, 0, h / 2 + 4) * Box(12, 16, h - 8)                # side window: CAN + power pigtail
    body -= Pos(0, 0, h - 6) * (Cylinder(od / 2 + 1, 2) - Cylinder(od / 2 - 1.2, 2))   # light-ring groove
    tp = TORQUE_PIN
    body -= Pos(tp["r"], 0, 0) * Cylinder(tp["d"] / 2 - 0.05, 2 * FLOOR)  # press-fit seat for the steel torque pin
    return body


def torque_pin() -> Part:
    tp = TORQUE_PIN
    return Pos(tp["r"], 0, FLOOR - 0.5 - tp["stick_out"] - FLOOR + 0.5) * Cylinder(tp["d"] / 2, tp["stick_out"] + FLOOR - 0.5, align=MIN)


def motor_adapter() -> Part:
    """Bolts to the motor output (8×M3). Male face on top, mating face up (+Z) at z = NECK + face_t."""
    p = Cylinder(NECK_R, NECK, align=MIN) + Pos(0, 0, NECK + KX74.face_t) * Rot(180, 0, 0) * male(KX74, bore=8)
    for k in range(MOTOR["out_holes"]):
        a = 2 * math.pi * k / MOTOR["out_holes"] + math.pi / 8
        x, y = MOTOR["out_pcd"] / 2 * math.cos(a), MOTOR["out_pcd"] / 2 * math.sin(a)
        p -= Pos(x, y, 0) * Cylinder(1.7, 40)
        p -= Pos(x, y, NECK + KX74.face_t - 3.5) * Cylinder(3.1, 10, align=MIN)   # counterbores
    return p


def adapter_balls() -> Part:
    return Pos(0, 0, NECK + KX74.face_t) * Rot(180, 0, 0) * balls(KX74)


def motor_dummy() -> Part:
    return Pos(0, 0, FLOOR) * Cylinder(MOTOR["od"] / 2, MOTOR["h"], align=MIN)


def stack_height() -> float:
    """Can floor to the KX mating plane."""
    return FLOOR + MOTOR["h"] + NECK + KX74.face_t


def coupon_motor() -> Part:
    """3 mm slotted disc: bolt it to the motor's front and back faces to read the real bolt circles."""
    p = Cylinder(MOTOR["od"] / 2, 3, align=MIN) - Cylinder(5, 3, align=MIN)
    for k in range(6):                                   # housing pattern: slots spanning PCD 48–66
        p -= Rot(0, 0, 60 * k) * Pos(28.5, 0, 0) * Box(9 + 3.4, 3.4, 6)
    for k in range(8):                                   # output pattern: slots spanning PCD 26–42
        p -= Rot(0, 0, 45 * k + 22.5) * Pos(17, 0, 0) * Box(8 + 3.4, 3.4, 6)
    for k, r in enumerate(range(13, 42, 2)):             # engraved ruler: one tick per 2 mm of radius
        p -= Rot(0, 0, 30) * Pos(r, 0, 2.6) * Box(0.6, 3 if k % 5 else 5, 1)
    return p
