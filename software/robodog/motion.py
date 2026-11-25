"""MotionController: a 50 Hz loop that turns velocity / pose / head commands
into joint angles for the 8 leg servos and the 2 head servos."""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Callable

from .gait import POSE_NAMES, TrotGait, Velocity, named_pose
from .kinematics import BodyKinematics, LegGeometry, UnreachableError

log = logging.getLogger(__name__)


def _approach(cur: float, target: float, max_delta: float) -> float:
    if target > cur:
        return min(target, cur + max_delta)
    return max(target, cur - max_delta)


class MotionController:
    def __init__(self, cfg: dict, servo_driver, obstacle_distance: Callable[[], float | None] | None = None,
                 imu: Callable[[], tuple[float, float] | None] | None = None):
        g, m = cfg["geometry"], cfg["motion"]
        self.cfg = m
        self.kin = BodyKinematics(g["hip_x"], g["foot_y"], LegGeometry(g["l2"], g["l3"]))
        self.stand_height = float(m["stand_height"])
        self.gait = TrotGait(self.kin.neutral_feet(self.stand_height), dict(self.kin.hips),
                             period=m["gait_period_s"], step_height=m["step_height"], max_step=m["max_step_mm"],
                             yaw_gain=float(m.get("yaw_stride_gain", 1.0)))
        self.servos = servo_driver
        self.obstacle_distance = obstacle_distance
        # balance: IMU (roll, pitch in rad) -> level the body by changing leg lengths
        self.imu = imu
        self.bal_kp = float(m.get("balance_kp", 0.0))
        self.bal_kd = float(m.get("balance_kd", 0.0))
        self.drift_comp = float(m.get("gait_drift_comp_mm_s", 0.0))
        self._bal_prev = (0.0, 0.0)
        self._bal = [0.0, 0.0]
        self.stop_distance = cfg.get("sensors", {}).get("stop_distance_mm", 250)

        self.max_speed = float(m["max_speed_mm_s"])
        self.max_yaw = math.radians(m["max_yaw_deg_s"])
        self.accel = float(m["accel_mm_s2"])
        self.head_rate = float(m.get("head_speed_deg_s", 120))
        self.dt = 1.0 / m["control_hz"]
        hj = cfg["servos"]["joints"]
        self.pan_lim = (hj["head.pan"]["min"], hj["head.pan"]["max"])
        self.tilt_lim = (hj["head.tilt"]["min"], hj["head.tilt"]["max"])

        self._lock = threading.Lock()
        self._target_v = Velocity()
        self._v = Velocity()
        self._pose_target = named_pose("stand", self.stand_height)
        self._pose = named_pose("lie", self.stand_height)   # boot lying down
        self.pose_name = "lie"
        self._head_target = [0.0, 0.0]
        self.head = [0.0, 0.0]                  # current pan, tilt (deg)
        self.enabled = False
        self.blocked = False
        self.odometry = [0.0, 0.0, 0.0]         # x mm, y mm, heading rad (dead reckoning)
        self.last_joints: dict[str, float] = {}
        self._thread: threading.Thread | None = None
        self._running = False

    # ------------------------------------------------------------ commands
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, yaw: float = 0.0):
        """vx (mm/s forward), yaw (rad/s, + = left). vy is ignored: v2 legs have no hip roll."""
        clamp = lambda v, lim: max(-lim, min(lim, v))
        with self._lock:
            self._target_v = Velocity(clamp(vx, self.max_speed), 0.0, clamp(yaw, self.max_yaw))
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

    def set_head(self, pan_deg: float | None = None, tilt_deg: float | None = None):
        """pan + = look left, tilt + = look down."""
        with self._lock:
            if pan_deg is not None:
                self._head_target[0] = max(self.pan_lim[0], min(self.pan_lim[1], pan_deg))
            if tilt_deg is not None:
                self._head_target[1] = max(self.tilt_lim[0], min(self.tilt_lim[1], tilt_deg))

    def look(self, pitch_deg: float = 0.0, yaw_deg: float = 0.0):
        self.set_head(yaw_deg, pitch_deg)

    def _balance(self, dt: float) -> tuple[float, float]:
        """PD on measured body tilt: tilt the commanded body the other way."""
        if not self.imu or (self.bal_kp == 0 and self.bal_kd == 0):
            return 0.0, 0.0
        m = self.imu()
        if m is None:
            return 0.0, 0.0
        out = []
        for i, ang in enumerate(m):
            rate = (ang - self._bal_prev[i]) / dt
            self._bal[i] = max(-0.25, min(0.25, -(self.bal_kp * ang + self.bal_kd * rate)))
            out.append(self._bal[i])
        self._bal_prev = tuple(m)
        return out[0], out[1]

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
            except Exception:
                log.exception("motion tick failed")
            next_t += self.dt
            time.sleep(max(0.0, next_t - time.monotonic()))

    def tick(self, dt: float):
        with self._lock:
            target = Velocity(self._target_v.vx, 0.0, self._target_v.yaw)
            pose_t = self._pose_target
            head_t = list(self._head_target)

        self.blocked = False
        if target.vx > 0 and self.obstacle_distance:
            d = self.obstacle_distance()
            if d is not None and d < self.stop_distance:
                target.vx = 0.0
                self.blocked = True

        dv = self.accel * dt
        self._v.vx = _approach(self._v.vx, target.vx, dv)
        self._v.yaw = _approach(self._v.yaw, target.yaw, dv / 150.0)

        p = self._pose
        k = min(1.0, dt * 3.0)
        for f in ("height", "roll", "pitch", "yaw", "dx", "dy", "rear_drop", "front_drop"):
            setattr(p, f, getattr(p, f) + (getattr(pose_t, f) - getattr(p, f)) * k)
        for i in (0, 1):
            self.head[i] = _approach(self.head[i], head_t[i], self.head_rate * dt)

        corr_r, corr_p = self._balance(dt)
        gv = self._v
        if self.drift_comp and not self._v.is_zero():
            # the trot creeps forward on its own; cancel it (measured in sim/, re-measure on your floor)
            gv = Velocity(self._v.vx - self.drift_comp, 0.0, self._v.yaw)
        feet = self.gait.update(dt, gv, p.height)
        feet = {leg: (x, y, z + (p.front_drop if leg[0] == "F" else p.rear_drop)) for leg, (x, y, z) in feet.items()}
        try:
            legs = self.kin.solve(feet, (p.roll + corr_r, p.pitch + corr_p, p.yaw), (p.dx, p.dy, 0.0))
        except UnreachableError as e:
            log.warning("IK: %s", e)
            return
        joints = {f"{leg}.{name}": a for leg, (pt, kn) in legs.items() for name, a in (("pitch", pt), ("knee", kn))}
        joints["head.pan"], joints["head.tilt"] = self.head
        self.last_joints = joints
        self.servos.write(joints)

        h = self.odometry[2]
        self.odometry[0] += self._v.vx * math.cos(h) * dt
        self.odometry[1] += self._v.vx * math.sin(h) * dt
        self.odometry[2] += self._v.yaw * dt
