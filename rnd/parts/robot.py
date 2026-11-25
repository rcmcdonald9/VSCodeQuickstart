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
from . import arm, base, roll, shoulder, upper

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
    # ---- base (CAD: rnd/parts/base.py) ----
    out += base.module()
    # ---- mast, lift, carriage, torso, head (CAD: rnd/parts/upper.py) ----
    out += upper.module(lift_mm / 1000)
    # ---- arms: shoulder pitch module (CAD) + roll stage (placeholder) + CAD arm ----
    poses = poses or {"l": dict(pitch=-15, elbow=-70, wrist=-20), "r": dict(pitch=-70, elbow=-45, wrist=-25)}
    for side, s in (("l", 1), ("r", -1)):
        P = poses[side]
        sh = Pos(a.shoulder_x * 1000, s * a.shoulder_y * 1000, lift_mm)
        mir = (lambda p: p) if s == 1 else (lambda p: mirror(p, Plane.XZ))
        for n, p, c in shoulder.module():
            out.append((f"{side}_sh_{n}", sh * mir(Pos(0, -(shoulder.Y_FER + 18), 0) * p), c))   # detach plane at the pitch body origin
        piv = sh * Rot(0, P["pitch"], 0)
        for n, p, c in roll.module():
            out.append((f"{side}_roll_{n}", piv * mir(p), c))
        for n, p, c in arm.assembly(elbow_deg=P["elbow"], wrist_deg=P["wrist"]):
            out.append((f"{side}_arm_{n}", piv * mir(Pos(0, roll.ROLL_Y, -roll.YAW_DROP) * p), c))
    return out


def export(lift=None, name="robot", poses=None):
    OUT.mkdir(exist_ok=True)
    parts = build(lift, poses)
    for n, p, _ in parts:
        export_stl(p, str(OUT / f"asm_{name}_{n}.stl"))
    (OUT / f"asm_{name}.txt").write_text("\n".join(f"asm_{name}_{n} {c}" for n, _, c in parts))
    return OUT / f"asm_{name}.txt"


if __name__ == "__main__":
    print(export(float(sys.argv[1]) if len(sys.argv) > 1 else None))
