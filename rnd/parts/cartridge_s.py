"""Cartridge S: an STS3215 in a printed sleeve. The KX44 male face bolts to the servo's Ø20 disc horn (4×M3 on a 9.9 mm square).
Torque pin under the sleeve, as on cartridge M. Servo geometry measured from the SO-ARM100 model (cad/params.scad)."""
from __future__ import annotations

from build123d import Align, Box, Cylinder, Part, Pos, Rot, Axis, fillet

from .kx import KX44, male, balls

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
SERVO = dict(l=45.4, w=24.8, case_h=32.0, bottom=4.0, shaft_off=10.2, horn_d=20.0, horn_t=1.6, horn_sq=9.9)
FIT, WALL, FLOOR = 0.3, 2.4, 2.5
NECK, NECK_R = 4.0, 18.0
TORQUE_PIN = dict(d=4.0, x=22.0, stick_out=8.0)


def _servo_x0():   # x of the servo's near end, with the shaft on the z axis
    return -SERVO["shaft_off"]


def sleeve() -> Part:
    s = SERVO
    L, W = s["l"] + 2 * (FIT + WALL), s["w"] + 2 * (FIT + WALL)
    H = s["case_h"] + s["bottom"] + FLOOR
    cx = _servo_x0() + s["l"] / 2
    body = Pos(cx, 0, 0) * Box(L, W, H, align=MIN)
    body = fillet(body.edges().filter_by(Axis.Z), 4.0)
    body -= Pos(cx, 0, FLOOR) * Box(s["l"] + 2 * FIT, s["w"] + 2 * FIT, H, align=MIN)
    body -= Pos(_servo_x0() + s["l"] - 6, 0, FLOOR + 6) * Box(14, W + 2, 8, align=MIN)     # bus connector window
    body -= Pos(TORQUE_PIN["x"], 0, 0) * Cylinder(TORQUE_PIN["d"] / 2 - 0.05, 2 * FLOOR)
    return body


def servo_top() -> float:
    return FLOOR + SERVO["bottom"] + SERVO["case_h"]


def servo_dummy() -> Part:
    s = SERVO
    return Pos(_servo_x0() + s["l"] / 2, 0, FLOOR + s["bottom"]) * Box(s["l"], s["w"], s["case_h"], align=MIN) + \
        Pos(0, 0, servo_top()) * Cylinder(s["horn_d"] / 2, s["horn_t"], align=MIN)


def horn_adapter() -> Part:
    """Sits on the horn. Its mating face is up (+Z) at z = NECK + face_t (local frame)."""
    s = SERVO
    p = Cylinder(NECK_R, NECK, align=MIN) + Pos(0, 0, NECK + KX44.face_t) * Rot(180, 0, 0) * male(KX44)
    h = s["horn_sq"] / 2
    for x, y in ((h, h), (h, -h), (-h, h), (-h, -h)):
        p -= Pos(x, y, 0) * Cylinder(1.7, 40)
        p -= Pos(x, y, NECK + KX44.face_t - 3.0) * Cylinder(3.0, 10, align=MIN)
    p -= Cylinder(3.6, 40)                                   # horn centre-screw access
    return p


def adapter_balls() -> Part:
    return Pos(0, 0, NECK + KX44.face_t) * Rot(180, 0, 0) * balls(KX44)


def torque_pin() -> Part:
    return Pos(TORQUE_PIN["x"], 0, -TORQUE_PIN["stick_out"]) * Cylinder(TORQUE_PIN["d"] / 2, TORQUE_PIN["stick_out"] + FLOOR - 0.5, align=MIN)


def stack_height() -> float:
    return servo_top() + SERVO["horn_t"] + NECK + KX44.face_t
