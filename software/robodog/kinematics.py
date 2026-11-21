"""Leg inverse/forward kinematics for a 3-DOF (roll, pitch, knee) leg.

Leg frame (same for all four legs): origin at the hip (where the roll and
pitch axes cross), +x forward, +y OUTWARD (away from the body), +z up.

Joint conventions (degrees in the public API, radians inside):
  roll  > 0 : foot swings outward
  pitch > 0 : upper leg swings backward (knee goes back)
  knee  < 0 : knee bends with the knee pointing backward (our only branch)
These match cad/assembly.scad, so a pose you see there is a pose you get.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


class UnreachableError(ValueError):
    pass


@dataclass(frozen=True)
class LegGeometry:
    l1: float = 49.5
    l2: float = 100.0
    l3: float = 100.0


def leg_ik(x: float, y: float, z: float, g: LegGeometry) -> tuple[float, float, float]:
    """Foot position in the leg frame -> (roll, pitch, knee) in degrees."""
    d_yz2 = y * y + z * z
    if d_yz2 < g.l1 * g.l1:
        raise UnreachableError("foot is inside the abduction offset")
    h = math.sqrt(d_yz2 - g.l1 * g.l1)            # in-plane vertical distance
    roll = math.atan2(z, y) - math.atan2(-h, g.l1)

    d2 = x * x + h * h
    c = (d2 - g.l2 ** 2 - g.l3 ** 2) / (2 * g.l2 * g.l3)
    if c > 1.0 or c < -1.0:
        raise UnreachableError(f"foot out of reach (d={math.sqrt(d2):.1f} mm)")
    knee = -math.acos(c)
    phi = math.atan2(-x, h)
    pitch = phi - math.atan2(g.l3 * math.sin(knee), g.l2 + g.l3 * math.cos(knee))
    return math.degrees(roll), math.degrees(pitch), math.degrees(knee)


def leg_fk(roll: float, pitch: float, knee: float, g: LegGeometry) -> tuple[float, float, float]:
    """(roll, pitch, knee) in degrees -> foot position in the leg frame."""
    r, p, k = map(math.radians, (roll, pitch, knee))
    x = -g.l2 * math.sin(p) - g.l3 * math.sin(p + k)
    h = g.l2 * math.cos(p) + g.l3 * math.cos(p + k)
    # rotate (L1, -h) about x by roll
    y = g.l1 * math.cos(r) + h * math.sin(r)
    z = g.l1 * math.sin(r) - h * math.cos(r)
    return x, y, z


def rot_rpy(roll: float, pitch: float, yaw: float):
    """3x3 rotation matrix (radians), applied as R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def mat_t_vec(m, v):
    """Transpose(m) @ v — i.e. the inverse rotation."""
    return tuple(sum(m[r][c] * v[r] for r in range(3)) for c in range(3))


class BodyKinematics:
    """Maps body-frame foot targets to joint angles for all four legs."""

    def __init__(self, hip_x: float, hip_y: float, geom: LegGeometry):
        self.geom = geom
        # (hip position in body frame, side sign: +1 = left, -1 = right)
        self.hips = {
            "FR": ((hip_x, -hip_y, 0.0), -1),
            "FL": ((hip_x, hip_y, 0.0), 1),
            "RR": ((-hip_x, -hip_y, 0.0), -1),
            "RL": ((-hip_x, hip_y, 0.0), 1),
        }

    def neutral_feet(self, height: float) -> dict[str, tuple[float, float, float]]:
        """Feet directly below the hips, in the body frame (ground frame)."""
        out = {}
        for leg, ((hx, hy, _), side) in self.hips.items():
            out[leg] = (hx, hy + side * self.geom.l1, -height)
        return out

    def solve(self, feet: dict[str, tuple[float, float, float]],
              body_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
              body_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        """feet: foot targets in the GROUND-aligned frame centred under the body.

        body_rpy (radians) tilts the body; body_offset shifts it (mm).
        Returns {leg: (roll, pitch, knee)} in degrees.
        """
        R = rot_rpy(*body_rpy)
        result = {}
        for leg, (fx, fy, fz) in feet.items():
            (hx, hy, hz), side = self.hips[leg]
            # foot relative to body origin, expressed in the (tilted) body frame
            rel = (fx - body_offset[0], fy - body_offset[1], fz - body_offset[2])
            bx, by, bz = mat_t_vec(R, rel)
            lx, ly, lz = bx - hx, (by - hy) * side, bz - hz
            result[leg] = leg_ik(lx, ly, lz, self.geom)
        return result
