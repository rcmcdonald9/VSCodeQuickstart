"""Safety supervisor v2 (S1-safety, SW2; Astra review #2b finding 20). Sits between any policy/teleop and the drives,
independent of the policy. Units at the joint output: rad, rad/s, N·m.

v1 clamped only the feed-forward torque, so an MIT-mode command (kp·e + kd·ė + τ_ff) could still ask for full current;
it also accepted NaN or stale feedback and released most faults on the next cycle. v2:

1. **Feedback first.** Every cycle needs measured position and speed for all joints: finite, the right shape, and no older
   than FB_MAX_AGE (10 ms). Missing/NaN/stale feedback → STOP.
2. **Total torque bound.** The torque the drive will produce, |kp·(q_cmd − q_meas)| + |kd·v_meas| + |τ_ff|, must stay under
   the joint's rated torque (peak only with `allow_peak`). The supervisor first trims τ_ff, then lowers kp and kd for that
   cycle until the bound holds; gains are also capped at KP_MAX / KD_MAX. The same limit must be set in the drive as a
   current limit (day-1 checklist): this is the software half.
3. **Joint limits, velocity, workspace.** Targets clamped to the limits (margin 3°) and slewed at v_max; a target that takes
   the hand outside the workspace box is rejected (hold the last target, τ_ff = 0).
4. **Faults, latched:** watchdog (no tick for 50 ms), e-stop, motor error flag, over-temperature (TEMP_C, driver/case
   sensor), measured speed over V_FAULT, tracking error over TRACK_FAULT, position outside the limits by > 8°.
   A latched fault returns STOP (disable the drives, drop the hardware heartbeat) until `reset()`, which needs the e-stop
   button physically released and fresh, in-limit feedback.
6. **Load mode (R2d, 2026-11-27):** `set_payload(kg)`. Above LOAD_KG (1 kg) the hand workspace shrinks to the box the
   basket study passed (rnd/basket.py: load ≤ 0.45 m in front of the base centre, hands ≤ 0.80 m high, ≥ 0.20 m
   up), and `base_accel_limit()` drops from 1.0 to 0.3 m/s² (tip margin while braking, rnd/tip.py). Pointing the load at a
   wheel is the base planner's job; `load_heading_ok(heading)` checks it (within ±20° of a wheel axis).
5. **Heartbeat:** `heartbeat_ok()` is true only after a complete healthy cycle; the loop toggles the hardware watchdog line
   only then (electrical.md A5; the A5 v2 watchdog times out after 8 ms, four missed cycles).
7. **Collision detection (D23 / R17, loop 47):** with a `collision=CollisionDetector(...)` (sw/collision.py) and the measured
   joint torque from the drive current passed as `tau_meas`, a momentum observer compares it with the twin's model torque
   (inertia + Coriolis + gravity + damping). A residual over the per-joint threshold for N_TRIP cycles latches "collision";
   the verdict is then a **compliant hold** (kp = 0, light damping, the twin's gravity torque as feed-forward) instead of a
   stop, so the arm doesn't fall and can be pushed away. Any other fault still stops. reset() clears it.
Every override is counted so a session log shows how often the policy was overruled.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from sw.collision import KD_COMPLIANT, KP_COMPLIANT, T_ANCHOR

WATCHDOG_S, FB_MAX_AGE, TEMP_C = 0.050, 0.010, 70.0     # TEMP_C: driver/case sensor (thermal.md: winding ≤ 69 °C on a task day)
V_FAULT, TRACK_FAULT = 4.0, 0.35                          # rad/s measured, rad tracking error
KP_MAX, KD_MAX = 60.0, 3.0
LOAD_KG = 1.0
LOAD_WS = ((0.10, -0.45, 0.20), (0.45, 0.45, 0.80))               # base frame, m (R2d basket study)                                # N·m/rad, N·m·s/rad at the joint (bench defaults)
DESC = Path(__file__).resolve().parent / "description" / "joints.json"


@dataclass
class Verdict:
    stop: bool
    q: np.ndarray | None = None
    kp: np.ndarray | None = None
    kd: np.ndarray | None = None
    t_ff: np.ndarray | None = None
    reasons: list = field(default_factory=list)
    compliant: bool = False


def _finite(x, n):
    a = np.asarray(x, float) if x is not None else None
    return a is not None and a.shape == (n,) and bool(np.all(np.isfinite(a)))


class Supervisor:
    def __init__(self, v_max=1.5, margin_deg=3.0, ws_lo=(0.10, -0.45, 0.76), ws_hi=(0.80, 0.45, 1.35), clock=time.monotonic, fk=None,
                 collision=None, joints=None, limits=None, torque=None):
        """joints: subset of joint names (default: all of joints.json, in order), e.g. the joints online during bring-up.
        limits: {name: (lower, upper)} overriding joints.json (rad, before the margin), e.g. the travel bounds used while a
        joint is not yet calibrated. torque: {name: N·m} overriding the rated/peak bound (e.g. the low calibration torque)."""
        J = json.loads(DESC.read_text())
        if joints is not None:
            by = {j["name"]: j for j in J}
            J = [by[n] for n in joints]
        self.names = [j["name"] for j in J]
        self.n = len(J)
        m = math.radians(margin_deg)
        lim = limits or {}
        self.lo = np.array([lim.get(j["name"], (j["lower"], j["upper"]))[0] for j in J]) + m
        self.hi = np.array([lim.get(j["name"], (j["lower"], j["upper"]))[1] for j in J]) - m
        tq = torque or {}
        self.rated = np.array([min(j["rated_nm"], tq.get(j["name"], np.inf)) for j in J])
        self.peak = np.array([min(j["peak_nm"], tq.get(j["name"], np.inf)) for j in J])
        self.v_max, self.clock, self.fk = v_max, clock, fk
        self.ws_lo, self.ws_hi = np.array(ws_lo), np.array(ws_hi)
        self.last_tick = None
        self.last_q = None
        self.latched = []                 # fault reasons; empty = healthy
        self.estop_pressed = False
        self.healthy_cycle = False
        self.counts = {}
        self.payload = 0.0
        self._ws_free = (self.ws_lo.copy(), self.ws_hi.copy())
        self.collision = collision
        self._anchor = None

    # ---- load mode (R2d)
    def set_payload(self, kg, box=None):
        """box: (lo, hi) in the supervisor's FK frame; default LOAD_WS (full-robot base frame). Enter load mode only with
        the hand inside the box (the planner moves there first); targets outside it are then rejected."""
        self.payload = float(kg)
        if self.collision is not None and hasattr(self.collision.twin, "set_payload"):
            self.collision.twin.set_payload(self.payload)          # the twin must know what the hand carries
            self.collision.reset()
        if self.payload > LOAD_KG:
            lo, hi = map(np.array, box or LOAD_WS)
            self.ws_lo, self.ws_hi = np.maximum(self._ws_free[0], lo), np.minimum(self._ws_free[1], hi)
        else:
            self.ws_lo, self.ws_hi = self._ws_free

    def base_accel_limit(self):
        return 0.3 if self.payload > LOAD_KG else 1.0

    def load_heading_ok(self, heading_deg):
        """x-drive wheels at 0/90/180/270°: a heavy load must point within ±20° of one of them (tip margin)."""
        if self.payload <= LOAD_KG:
            return True
        r = abs(((heading_deg + 45) % 90) - 45)
        return r <= 20

    # ---- inputs from the loop / hardware
    def _count(self, k):
        self.counts[k] = self.counts.get(k, 0) + 1

    def tick(self):
        self.last_tick = self.clock()

    def press_estop(self):
        self.estop_pressed = True
        self._latch("e-stop")

    def release_estop(self):
        self.estop_pressed = False         # releasing does not clear the latch; reset() does

    def _latch(self, why):
        if why not in self.latched:
            self.latched.append(why)
        self._count(why)
        self.healthy_cycle = False

    def heartbeat_ok(self):
        return self.healthy_cycle and not self.latched

    def reset(self, q_meas, fb_age):
        """Clear latched faults: only with the button released and fresh, finite, in-limit feedback."""
        if self.estop_pressed:
            return False
        if not _finite(q_meas, self.n) or fb_age is None or fb_age > FB_MAX_AGE:
            return False
        q = np.asarray(q_meas, float)
        if np.any(q < self.lo - math.radians(8)) or np.any(q > self.hi + math.radians(8)):
            return False
        self.latched = []
        if self.collision is not None:
            self.collision.reset()
        self.last_q = np.clip(q, self.lo, self.hi)
        self.tick()
        return True

    # ---- the per-cycle check
    def check(self, q_want, kp, kd, t_ff, q_meas, v_meas, fb_age, err=None, temp=None, dt=1 / 500, allow_peak=False,
              tau_meas=None) -> Verdict:
        self.healthy_cycle = False
        now = self.clock()
        if self.last_tick is None or now - self.last_tick > WATCHDOG_S:
            self._latch("watchdog")
        if not (_finite(q_meas, self.n) and _finite(v_meas, self.n)) or fb_age is None or not math.isfinite(fb_age) or fb_age > FB_MAX_AGE:
            self._latch("feedback missing, NaN or stale")
        else:
            qm, vm = np.asarray(q_meas, float), np.asarray(v_meas, float)
            if np.any(qm < self.lo - math.radians(8)) or np.any(qm > self.hi + math.radians(8)):
                self._latch("measured position outside limits")
            if np.any(np.abs(vm) > V_FAULT):
                self._latch("measured speed over limit")
            if self.last_q is not None and np.any(np.abs(self.last_q - qm) > TRACK_FAULT):
                self._latch("tracking error")
            if self.collision is not None and tau_meas is not None and "collision" not in self.latched:
                if not _finite(tau_meas, self.n):
                    self._latch("feedback missing, NaN or stale")
                elif self.collision.update(qm, vm, tau_meas, dt):
                    self._latch("collision")
        if err is not None and np.any(np.asarray(err) != 0):
            self._latch("motor error flag")
        if temp is not None:
            t = np.asarray(temp, float)
            if not np.all(np.isfinite(t)) or np.max(t) > TEMP_C:
                self._latch("over temperature or bad temperature reading")
        if not _finite(q_want, self.n):
            self._latch("non-finite command")
        if self.latched == ["collision"]:                               # compliant hold, not a stop: the arm must not drop
            qm = np.asarray(q_meas, float)
            lim = self.peak if allow_peak else self.rated
            if self._anchor is None:
                self._anchor = qm.copy()
            self._anchor += (qm - self._anchor) * min(dt / T_ANCHOR, 1.0)
            g = self.collision.twin.gravity(qm)
            kp_c = np.full(self.n, KP_COMPLIANT)
            room = np.maximum(lim - np.abs(kp_c * (self._anchor - qm)), 0)
            g = np.clip(g, -room, room)
            self.last_q = qm.copy()
            self.healthy_cycle = True                                    # controlled state: keep the hardware heartbeat
            return Verdict(False, self._anchor.copy(), kp_c, np.full(self.n, KD_COMPLIANT), g, ["collision: compliant hold"], compliant=True)
        self._anchor = None
        if self.latched:
            return Verdict(True, reasons=list(self.latched))
        qm, vm = np.asarray(q_meas, float), np.asarray(v_meas, float)
        # 3. limits, slew, workspace
        q = np.clip(np.asarray(q_want, float), self.lo, self.hi)
        if np.any(q != q_want):
            self._count("joint limit clamp")
        if self.last_q is None:
            self.last_q = qm.copy()
        step = self.v_max * dt
        q2 = self.last_q + np.clip(q - self.last_q, -step, step)
        if np.any(np.abs(q2 - q) > 1e-12):
            self._count("velocity clamp")
        tf = np.zeros(self.n) if t_ff is None else np.nan_to_num(np.asarray(t_ff, float))
        if self.fk is not None:
            p = self.fk(q2)
            if np.any(p < self.ws_lo) or np.any(p > self.ws_hi):
                self._count("workspace reject")
                q2, tf = self.last_q.copy(), np.zeros(self.n)
        # 2. total torque bound
        lim = self.peak if allow_peak else self.rated
        kp_ = np.clip(np.nan_to_num(np.asarray(kp, float)), 0, KP_MAX)
        kd_ = np.clip(np.nan_to_num(np.asarray(kd, float)), 0, KD_MAX)
        if np.any(kp_ != kp) or np.any(kd_ != kd):
            self._count("gain cap")
        e = np.abs(q2 - qm)
        damp = kd_ * np.abs(vm)
        tf = np.clip(tf, -lim, lim)
        over = np.abs(tf) + kp_ * e + damp - lim
        if np.any(over > 0):
            self._count("torque bound")
            room = np.maximum(lim - kp_ * e - damp, 0)                  # trim the feed-forward first
            tf = np.clip(tf, -room, room)
            room = np.maximum(lim - np.abs(tf) - damp, 0)               # then the position gain
            kp_ = np.where(e > 1e-9, np.minimum(kp_, room / np.maximum(e, 1e-9)), kp_)
            room = np.maximum(lim - np.abs(tf) - kp_ * e, 0)             # then the damping
            kd_ = np.where(np.abs(vm) > 1e-9, np.minimum(kd_, room / np.maximum(np.abs(vm), 1e-9)), kd_)
        self.last_q = q2
        self.healthy_cycle = True
        return Verdict(False, q2, kp_, kd_, tf, [])

    @staticmethod
    def drive_torque(v: Verdict, q_meas, v_meas):
        """What an MIT-mode drive will produce for this verdict (for tests and logs)."""
        return v.kp * (v.q - np.asarray(q_meas)) - v.kd * np.asarray(v_meas) + v.t_ff
