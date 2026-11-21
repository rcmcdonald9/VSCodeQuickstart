"""MotionController: a 50 Hz thread that turns velocity / pose commands into servo angles."""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Callable

from .gait import POSE_NAMES, Pose, TrotGait, Velocity, named_pose
from .kinematics import BodyKinematics, LegGeometry, UnreachableError

log = logging.getLogger(__name__)


def _approach(cur: float, target: float, max_delta: float) -> float:
    if target > cur:
        return min(target, cur + max_delta)
    return max(target, cur - max_delta)


class MotionController:
    def __init__(self, cfg: dict, servo_driver, obstacle_distance: Callable[[], float | None] | None = None):
        g, m = cfg["geometry"], cfg["motion"]
        self.cfg = m
        self.kin = BodyKinematics(g["hip_x"], g["hip_y"], LegGeometry(g["l1"], g["l2"], g["l3"]))
        self.stand_height = float(m["stand_height"])
        neutral = self.kin.neutral_feet(self.stand_height)
        hips = {leg: (p[0], p[1]) for leg, (p, _) in self.kin.hips.items()}
        self.gait = TrotGait(neutral, hips, period=m["gait_period_s"], step_height=m["step_height"],
                             max_step=m["max_step_mm"])
        self.servos = servo_driver
        self.obstacle_distance = obstacle_distance
        self.stop_distance = cfg.get("sensors", {}).get("stop_distance_mm", 250)

        self.max_speed = float(m["max_speed_mm_s"])
        self.max_yaw = math.radians(m["max_yaw_deg_s"])
        self.accel = float(m["accel_mm_s2"])
        self.dt = 1.0 / m["control_hz"]

        self._lock = threading.Lock()
        self._target_v = Velocity()
        self._v = Velocity()
        self._pose_target = named_pose("stand", self.stand_height)
        self._pose = named_pose("lie", self.stand_height)   # we boot lying down
        self.pose_name = "lie"
        self.enabled = False
        self.blocked = False                   # true while the ToF guard is holding us
        self.odometry = [0.0, 0.0, 0.0]        # x mm, y mm, heading rad (dead reckoning)
        self._thread: threading.Thread | None = None
        self._running = False

    # ------------------------------------------------------------ commands
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, yaw: float = 0.0):
        """vx, vy in mm/s; yaw in rad/s. Clamped to configured limits."""
        clamp = lambda v, lim: max(-lim, min(lim, v))
        with self._lock:
            self._target_v = Velocity(clamp(vx, self.max_speed), clamp(vy, self.max_speed * 0.6),
                                      clamp(yaw, self.max_yaw))
            if not self._target_v.is_zero() and self.pose_name != "stand":
                self._pose_target = named_pose("stand", self.stand_height)
                self.pose_name = "stand"

    def stop(self):
        self.set_velocity(0, 0, 0)

    def set_pose(self, name: str):
        if name not in POSE_NAMES:
            raise ValueError(f"unknown pose {name!r}")
        with self._lock:
            self._target_v = Velocity()
            self._pose_target = named_pose(name, self.stand_height)
            self.pose_name = name

    def look(self, pitch_deg: float = 0.0, yaw_deg: float = 0.0):
        """Tilt the body (and so the head camera) while standing."""
        with self._lock:
            p = named_pose("stand", self.stand_height)
            p.pitch = math.radians(max(-15, min(15, pitch_deg)))
            p.yaw = math.radians(max(-15, min(15, yaw_deg)))
            self._pose_target = p

    @property
    def is_moving(self) -> bool:
        return not self._v.is_zero(1.0)

    # ------------------------------------------------------------- thread
    def start(self):
        self.enabled = True
        self._running = True
        self._thread = threading.Thread(target=self._run, name="motion", daemon=True)
        self._thread.start()

    def shutdown(self):
        self.set_pose("lie")
        time.sleep(1.5)
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        self.servos.relax()

    def _run(self):
        next_t = time.monotonic()
        while self._running:
            try:
                self.tick(self.dt)
            except Exception:  # never let the control loop die silently
                log.exception("motion tick failed")
            next_t += self.dt
            time.sleep(max(0.0, next_t - time.monotonic()))

    def tick(self, dt: float):
        with self._lock:
            target = Velocity(self._target_v.vx, self._target_v.vy, self._target_v.yaw)
            pose_t = self._pose_target

        # obstacle guard: never walk forward into something
        self.blocked = False
        if target.vx > 0 and self.obstacle_distance:
            d = self.obstacle_distance()
            if d is not None and d < self.stop_distance:
                target.vx = 0.0
                self.blocked = True

        # ramp velocity
        dv = self.accel * dt
        self._v.vx = _approach(self._v.vx, target.vx, dv)
        self._v.vy = _approach(self._v.vy, target.vy, dv)
        self._v.yaw = _approach(self._v.yaw, target.yaw, dv / 150.0)

        # ease pose (~1 s for a full transition)
        p = self._pose
        k = min(1.0, dt * 3.0)
        for f in ("height", "roll", "pitch", "yaw", "dx", "dy", "rear_drop", "front_drop"):
            setattr(p, f, getattr(p, f) + (getattr(pose_t, f) - getattr(p, f)) * k)

        feet = self.gait.update(dt, self._v, p.height)
        feet = {leg: (x, y, z + (p.front_drop if leg[0] == "F" else p.rear_drop))
                for leg, (x, y, z) in feet.items()}
        try:
            joints = self.kin.solve(feet, (p.roll, p.pitch, p.yaw), (p.dx, p.dy, 0.0))
        except UnreachableError as e:
            log.warning("IK: %s", e)
            return
        self.servos.write_joints(joints)

        # dead-reckoning odometry (rough — for "walk 50 cm" style commands)
        h = self.odometry[2]
        self.odometry[0] += (self._v.vx * math.cos(h) - self._v.vy * math.sin(h)) * dt
        self.odometry[1] += (self._v.vx * math.sin(h) + self._v.vy * math.cos(h)) * dt
        self.odometry[2] += self._v.yaw * dt
