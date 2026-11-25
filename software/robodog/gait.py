"""Trot gait + static poses. Produces body-frame foot targets every tick."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# Diagonal pairs move together in a trot.
PHASE_OFFSET = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}


@dataclass
class Velocity:
    vx: float = 0.0      # mm/s forward
    vy: float = 0.0      # mm/s left
    yaw: float = 0.0     # rad/s counter-clockwise (turn left)

    def is_zero(self, eps: float = 1e-3) -> bool:
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.yaw) < eps


@dataclass
class Pose:
    """Body pose relative to the neutral stand."""
    height: float
    roll: float = 0.0    # rad
    pitch: float = 0.0   # rad (+ = nose down)
    yaw: float = 0.0
    dx: float = 0.0      # body shift, mm
    dy: float = 0.0
    rear_drop: float = 0.0   # extra mm the rear feet are raised (sit)
    front_drop: float = 0.0  # extra mm the front feet are raised (bow / lie)


def named_pose(name: str, stand_height: float) -> Pose:
    h = stand_height
    poses = {
        "stand": Pose(height=h),
        # v2 knees can swing back only ~50° (the knee servo would hit the tray),
        # so low poses shift the body back to keep the feet ahead of the hips.
        "sit": Pose(height=h, rear_drop=60, dx=-20),
        "lie": Pose(height=h, rear_drop=75, front_drop=75, dx=-30),
        "bow": Pose(height=h, front_drop=60, dx=-20),
        "tall": Pose(height=h + 15),
        "crouch": Pose(height=h - 35),
    }
    if name not in poses:
        raise ValueError(f"unknown pose {name!r}; choose from {sorted(poses)}")
    return poses[name]


POSE_NAMES = ("stand", "sit", "lie", "bow", "tall", "crouch")


@dataclass
class TrotGait:
    neutral: dict                    # leg -> (x, y, z) at stand height (z ignored)
    hips: dict                       # leg -> (hip_x, hip_y)
    period: float = 0.45
    step_height: float = 30.0
    max_step: float = 55.0
    duty: float = 0.5                # fraction of the cycle a foot is on the ground
    yaw_gain: float = 1.0            # >1 exaggerates turning strides (2-DOF legs must skid to turn)
    phase: float = 0.0
    _active: bool = field(default=False, init=False)

    def step_vector(self, leg: str, v: Velocity) -> tuple[float, float]:
        """Total foot travel (mm) during one stance phase for this leg."""
        hx, hy = self.hips[leg]
        # linear velocity + yaw contribution (omega x r)
        vx = v.vx - v.yaw * hy * self.yaw_gain
        vy = v.vy + v.yaw * hx
        t_stance = self.period * self.duty
        sx, sy = vx * t_stance, vy * t_stance
        n = math.hypot(sx, sy)
        if n > self.max_step:
            sx, sy = sx * self.max_step / n, sy * self.max_step / n
        return sx, sy

    def update(self, dt: float, v: Velocity, height: float) -> dict:
        """Advance the gait by dt seconds; returns foot targets {leg: (x, y, z)}."""
        moving = not v.is_zero()
        if moving or self._active:
            self.phase = (self.phase + dt / self.period) % 1.0
        # let the current cycle finish cleanly when we stop: settle at phase 0/0.5
        if not moving and self._active and (self.phase < 0.05 or 0.5 <= self.phase < 0.55):
            self._active = False
        if moving:
            self._active = True

        feet = {}
        for leg, (nx, ny, _) in self.neutral.items():
            if not self._active:
                feet[leg] = (nx, ny, -height)
                continue
            p = (self.phase + PHASE_OFFSET[leg]) % 1.0
            sx, sy = self.step_vector(leg, v)
            swing = 1.0 - self.duty
            if p < swing:                       # swing: lift and move forward
                s = p / swing
                # cycloid-ish progression keeps foot speed 0 at lift-off/touch-down
                prog = s - math.sin(2 * math.pi * s) / (2 * math.pi)
                fx = nx - sx / 2 + sx * prog
                fy = ny - sy / 2 + sy * prog
                fz = -height + self.step_height * math.sin(math.pi * s)
            else:                               # stance: push backward on the ground
                s = (p - swing) / self.duty
                fx = nx + sx / 2 - sx * s
                fy = ny + sy / 2 - sy * s
                fz = -height
            feet[leg] = (fx, fy, fz)
        return feet
