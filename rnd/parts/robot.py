"""Whole-robot assembly for renders: CAD arms + shoulder modules on placeholder base/spine/torso/head.

Placeholders are marked by name ("ph_*") until those modules are designed. Frames follow the twin (x forward, z up,
right arm at -y). Units mm. python -m rnd.parts.robot [lift_m]
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

from build123d import Align, Axis, Box, Cylinder, Plane, Pos, Rot, export_stl, fillet, mirror

from ..params import BASELINE
from . import arm, shoulder

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
OUT = Path(__file__).resolve().parents[1] / "out"


def _rbox(x, y, z, r):
    return fillet(Box(x, y, z).edges().filter_by(Axis.Z), r)


def omni_wheel():
    """6" omni placeholder: hub + 14 rollers on the rim (axis along Y)."""
    w = Cylinder(60, 36) - Cylinder(14, 36)
    for k in range(14):
        a = 2 * math.pi * k / 14
        w += Pos(66 * math.cos(a), 66 * math.sin(a), 0) * Rot(0, 0, math.degrees(a) + 90) * Rot(0, 90, 0) * Cylinder(10, 26)
    return Rot(90, 0, 0) * w


def build(lift=None, poses=None):
    d = BASELINE
    b, u, a = d.base, d.upper, d.arm
    lift_mm = (lift if lift is not None else 0.75) * 1000
    out = []
    # ---- base (placeholder shell) + wheels at R 0.21, one pointing forward ----
    z0, h = b.clearance * 1000, b.height * 1000
    out += [("ph_base", Pos(0, 0, z0) * (Cylinder(b.radius * 1000, h, align=MIN) - Pos(0, 0, -1) * Cylinder(b.radius * 1000 - 3, h - 3, align=MIN)), "white"),
            ("ph_bumper", Pos(0, 0, z0 + 18) * (Cylinder(b.radius * 1000 + 4, 10, align=MIN) - Cylinder(b.radius * 1000 - 1, 10, align=MIN)), "orange")]
    for k in range(4):
        ang = 90 * k
        r = b.wheel_radius_pos * 1000
        out.append((f"wheel{k}", Pos(r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang)), b.wheel_d * 500)
                    * Rot(0, 0, ang) * omni_wheel(), "dark"))
    # ---- spine + lift carriage (placeholders) ----
    top = z0 + h
    spine_top = top + u.spine_len * 1000
    out += [("ph_spine", Pos(-60, 0, top) * Box(90, 100, u.spine_len * 1000, align=MIN), "steel")]
    # ---- head: on the mast top (owner suggestion, D8) or riding the torso ----
    hz = spine_top + 10 if u.head_on_mast else lift_mm + 220
    head = [("ph_head", Pos(10, 0, hz) * _rbox(160, 240, 130, 40), "white"),
            ("ph_visor", Pos(86, 0, hz + 5) * _rbox(10, 200, 70, 4), "dark"),
            ("ph_eye_l", Pos(92, 45, hz + 8) * Rot(0, 90, 0) * Cylinder(11, 2), "orange"),
            ("ph_eye_r", Pos(92, -45, hz + 8) * Rot(0, 90, 0) * Cylinder(11, 2), "orange"),
            ("ph_handle", Pos(-20, 0, hz + 65) * (Rot(90, 0, 0) * (Cylinder(38, 26) - Cylinder(27, 27))), "dark")]
    if not u.head_on_mast:
        head.append(("ph_neck", Pos(0, 0, lift_mm + 80) * Cylinder(25, 80, align=MIN), "dark"))
    out += head
    # ---- torso (placeholder) at the shoulder height ----
    out += [("ph_torso", Pos(0, 0, lift_mm) * _rbox(180, 300, 160, 30), "white"),
            ("ph_chest_light", Pos(91, 0, lift_mm - 20) * Box(2, 180, 8), "orange")]
    # ---- arms: shoulder pitch module (CAD) + roll stage (placeholder) + CAD arm ----
    poses = poses or {"l": dict(pitch=-15, elbow=-70, wrist=-20), "r": dict(pitch=-70, elbow=-45, wrist=-25)}
    for side, s in (("l", 1), ("r", -1)):
        P = poses[side]
        sh = Pos(a.shoulder_x * 1000, s * a.shoulder_y * 1000, lift_mm)
        mir = (lambda p: p) if s == 1 else (lambda p: mirror(p, Plane.XZ))
        for n, p, c in shoulder.module():
            out.append((f"{side}_sh_{n}", sh * mir(Pos(0, -20, 0) * p), c))
        piv = sh * Rot(0, P["pitch"], 0)
        roll = [("roll_cart", Pos(0, 50, 0) * Rot(90, 0, 0) * Cylinder(44, 60), "dark"),
                ("roll_collar", Pos(0, 36, 0) * Rot(90, 0, 0) * (Cylinder(49, 12) - Cylinder(40, 12)), "orange"),
                ("roll_link", Pos(0, 50, -40) * _rbox(90, 70, 60, 20), "white")]
        for n, p, c in roll:
            out.append((f"{side}_ph_{n}", piv * mir(p), c))
        for n, p, c in arm.assembly(elbow_deg=P["elbow"], wrist_deg=P["wrist"]):
            out.append((f"{side}_arm_{n}", piv * mir(Pos(0, 50, -70) * p), c))
    return out


def export(lift=None, name="robot"):
    OUT.mkdir(exist_ok=True)
    parts = build(lift)
    for n, p, _ in parts:
        export_stl(p, str(OUT / f"asm_{name}_{n}.stl"))
    (OUT / f"asm_{name}.txt").write_text("\n".join(f"asm_{name}_{n} {c}" for n, _, c in parts))
    return OUT / f"asm_{name}.txt"


if __name__ == "__main__":
    print(export(float(sys.argv[1]) if len(sys.argv) > 1 else None))
