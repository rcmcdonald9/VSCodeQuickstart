"""Collision hulls of the CAD arm for the MuJoCo twin (one convex piece per STL; MuJoCo collides convex hulls).

Frames match the twin: upper pieces in the upper-arm body frame (origin at the yaw joint, arm along -Z), forearm pieces in
the forearm body frame (origin on the elbow axis). CAD +Y = the arm's outboard side (forearm side link). Right-arm copies
are mirrored in Y (the right arm sits at -Y). Units: mm; the twin scales by 0.001.
"""
from __future__ import annotations

from pathlib import Path

from build123d import Box, Cylinder, Plane, Pos, Rot, export_stl, mirror

from . import arm, cartridge_m as cm
from .kx import KX74

OUT = Path(__file__).resolve().parents[1] / "out" / "collision"


def pieces():
    h = arm.HOUSING
    cart_len = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    y0 = -20.0                                                     # cartridge KX face plane (arm.assembly)
    fy, link_t = arm.FORE_Y, 22.0
    wy = arm.J4310["h"] + 4 + 6
    return {
        "upper": {
            "neck": Pos(0, 0, h["z1"] - 2) * Cylinder(36, 104 - 6 + 2, align=arm.MIN) + Pos(0, 0, -6) * Cylinder(45, 6, align=arm.MIN),
            "housing": Pos(0, 0, (h["z0"] + h["z1"]) / 2) * Box(h["x"], h["y"] + 16, h["z1"] - h["z0"]),
            "elbow_cart": Pos(0, y0 - cart_len / 2, -arm.ELBOW_Z + arm.BELT_C) * Rot(90, 0, 0) * Cylinder(cm.MOTOR["od"] / 2 + 3.3, cart_len),
        },
        "fore": {
            "link": Pos(0, fy, -arm.FORE_L / 2 + 10) * Box(56, link_t, arm.FORE_L - 2 * 38),
            "wrist": Pos(0, fy + link_t / 2 - (wy + 12) / 2, -arm.FORE_L) * Rot(90, 0, 0) * Cylinder(39, wy + 12),
        },
    }


def export():
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}
    for body, parts in pieces().items():
        for n, p in parts.items():
            for side, q in (("l", p), ("r", mirror(p, Plane.XZ))):
                f = OUT / f"{body}_{n}_{side}.stl"
                export_stl(q, str(f))
                files[(body, n, side)] = f
    return files


if __name__ == "__main__":
    print(len(export()), "collision meshes →", OUT)
