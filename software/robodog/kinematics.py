"""Leg kinematics for Pip v2: 2-DOF planar legs (hip pitch + knee).

Every leg lives in a vertical plane parallel to the body's X-Z plane.
Leg frame: origin on the hip axis, +x forward, +z up.

Joint conventions (degrees in the public API), identical for all 4 legs and
matching cad/assembly.scad:
  pitch > 0 : upper leg swings backward (knee goes back)
  knee  < 0 : knee bends with the knee pointing backward (the only branch used)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

LEGS = ("FR", "FL", "RR", "RL")


class UnreachableError(ValueError):
    pass


@dataclass(frozen=True)
class LegGeometry:
    l2: float = 100.0   # hip -> knee
    l3: float = 100.0   # knee -> foot centre


def leg_ik(x: float, z: float, g: LegGeometry) -> tuple[float, float]:
    """Foot (x, z) in the leg frame -> (pitch, knee) in degrees."""
    h = -z
    d2 = x * x + h * h
    c = (d2 - g.l2 ** 2 - g.l3 ** 2) / (2 * g.l2 * g.l3)
    if c > 1.0 or c < -1.0:
        raise UnreachableError(f"foot out of reach (d={math.sqrt(d2):.1f} mm)")
    knee = -math.acos(c)
    phi = math.atan2(-x, h)
    pitch = phi - math.atan2(g.l3 * math.sin(knee), g.l2 + g.l3 * math.cos(knee))
    return math.degrees(pitch), math.degrees(knee)


def leg_fk(pitch: float, knee: float, g: LegGeometry) -> tuple[float, float]:
    """(pitch, knee) degrees -> foot (x, z) in the leg frame."""
    p, k = math.radians(pitch), math.radians(knee)
    x = -g.l2 * math.sin(p) - g.l3 * math.sin(p + k)
    z = -(g.l2 * math.cos(p) + g.l3 * math.cos(p + k))
    return x, z


def rot_rpy(roll: float, pitch: float, yaw: float):
    """3x3 rotation (radians), R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def mat_t_vec(m, v):
    return tuple(sum(m[r][c] * v[r] for r in range(3)) for c in range(3))


class BodyKinematics:
    """Body-frame foot targets -> joint angles for all four legs."""

    def __init__(self, hip_x: float, foot_y: float, geom: LegGeometry):
        self.geom = geom
        self.hips = {"FR": (hip_x, -foot_y), "FL": (hip_x, foot_y),
                     "RR": (-hip_x, -foot_y), "RL": (-hip_x, foot_y)}

    def neutral_feet(self, height: float) -> dict[str, tuple[float, float, float]]:
        return {leg: (hx, hy, -height) for leg, (hx, hy) in self.hips.items()}

    def solve(self, feet, body_rpy=(0.0, 0.0, 0.0), body_offset=(0.0, 0.0, 0.0)):
        """feet: {leg: (x, y, z)} in the ground-aligned frame under the body.

        Body roll/pitch are produced by lengthening/shortening legs. With no
        hip-roll joints the feet stay in their leg planes, so any requested
        sideways foot offset is ignored.
        Returns {leg: (pitch, knee)} in degrees.
        """
        R = rot_rpy(*body_rpy)
        out = {}
        for leg, (fx, fy, fz) in feet.items():
            hx, _ = self.hips[leg]
            rel = (fx - body_offset[0], fy - body_offset[1], fz - body_offset[2])
            bx, _by, bz = mat_t_vec(R, rel)
            out[leg] = leg_ik(bx - hx, bz, self.geom)
        return out
