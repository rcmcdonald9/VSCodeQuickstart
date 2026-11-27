"""Safety supervisor v2 (S1-safety, SW2; Astra review #2b finding 20). Sits between any policy/teleop and the drives,
independent of the policy. Units at the joint output: rad, rad/s, N·m.

v1 clamped only the feed-forward torque, so an MIT-mode command (kp·e + kd·ė + τ_ff) could still ask for full current;
it also accepted NaN or stale feedback and released most faults on the next cycle. v2:

1. **Feedback first.** Every cycle needs measured position and speed for all joints: finite, the right shape, and no older
   than FB_MAX_AGE (20 ms; was 10, bus-realism runs). Missing/NaN/stale feedback → STOP.
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
8. **People-near mode (D25 / R17, 2026-11-27):** `set_people_near(True)` while a person is close (PeopleNearTrigger below;
   docs/SOFTWARE_PLAN.md). Joints with a people-near range in joints.json (`people_near`, from rnd/twin.JR_PEOPLE_NEAR:
   shoulder roll ≥ +5°; rnd/pinch.py finds the inboard roll wedge ≥ 25 mm from ≈ +1°) get that range as their target bound
   (same margin as the normal limits); targets outside it are clamped like any joint limit. Entering the mode with the arm
   inside the restricted zone does not stop it: the bound starts at the arm's position and rises at PN_EXIT_SPEED
   (0.2 rad/s), so the arm is walked out slowly; the faults (position outside limits, tracking) still use the hard
   limits. Leaving the mode restores the full range at once.
9. **Battery-current budget (W8a, electrical.md *Pack and BMS*; sw/power.py):** every cycle the drives' bus current is
   predicted from the torque the verdict will produce (I ≈ |τ| / (ratio · Kt) + idle per joint, braking joints count 0)
   plus REST_A (9.9 A) for the rest of the robot. Over **70 A** the motoring joints' torque bounds are scaled down (τ_ff
   first, then kp, then kd, as in 2.). The **30 s rolling mean** is held at ≤ **28 A**: a ceiling that falls from 70 A
   towards 28 A as the window's room is used up (and below 28 A in proportion if the mean is ever over) derates the
   per-joint torque limits to k · rated, one k for the motoring joints, floor K_MIN. Neither ever stops the arm or drops
   the heartbeat; counted as "current cap" / "power derate". A BMS over-current or under-voltage flag (`bms=BmsReading(...)`,
   stub of the CAN A telemetry) latches a fault. `power=False` turns the budget off (tests only).
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
from sw.power import BmsReading, CurrentModel, K_MIN, PackBudget, REST_A, derate_factor  # noqa: F401 (BmsReading: callers)

WATCHDOG_S, FB_MAX_AGE, TEMP_C = 0.050, 0.020, 70.0     # TEMP_C: driver/case sensor (thermal.md: winding ≤ 69 °C on a task day)
# FB_MAX_AGE 10 → 20 ms (2026-11-27, docs/sw/bus_realism.md): with 0–9 ms of bus delay each way the feedback age reached 16 ms
# in normal running (p99.9 12 ms) and 10 ms false-tripped; a dead bus is still stopped in 22–30 ms (frames already in flight keep it fresh), next to the drive's own 30 ms timeout
V_FAULT, TRACK_FAULT = 4.0, 0.35                          # rad/s measured, rad tracking error
KP_MAX, KD_MAX = 60.0, 3.0
LOAD_KG = 1.0
LOAD_WS = ((0.10, -0.45, 0.20), (0.45, 0.45, 0.80))               # base frame, m (R2d basket study)                                # N·m/rad, N·m·s/rad at the joint (bench defaults)
PN_EXIT_SPEED = 0.2               # rad/s: how fast entering people-near mode walks a joint out of its restricted zone (D25)
PN_RADIUS, PN_RELEASE, PN_HOLD_S, PN_STALE_S = 1.5, 1.8, 2.0, 0.5   # people-near trigger: on within 1.5 m, off beyond 1.8 m for 2 s
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


def _trim(lim, tf, kp_, kd_, e, vm):
    """Bring |τ_ff| + kp·e + kd·|ė| under lim per joint: trim the feed-forward first, then kp, then kd."""
    damp = kd_ * np.abs(vm)
    room = np.maximum(lim - kp_ * e - damp, 0)                  # trim the feed-forward first
    tf = np.clip(tf, -room, room)
    room = np.maximum(lim - np.abs(tf) - damp, 0)               # then the position gain
    kp_ = np.where(e > 1e-9, np.minimum(kp_, room / np.maximum(e, 1e-9)), kp_)
    room = np.maximum(lim - np.abs(tf) - kp_ * e, 0)             # then the damping
    kd_ = np.where(np.abs(vm) > 1e-9, np.minimum(kd_, room / np.maximum(np.abs(vm), 1e-9)), kd_)
    return tf, kp_, kd_


class Supervisor:
    def __init__(self, v_max=1.5, margin_deg=3.0, ws_lo=(0.10, -0.45, 0.76), ws_hi=(0.80, 0.45, 1.35), clock=time.monotonic, fk=None,
                 collision=None, joints=None, limits=None, torque=None, fb_max_age=FB_MAX_AGE, power=True, rest_a=REST_A):
        """joints: subset of joint names (default: all of joints.json, in order), e.g. the joints online during bring-up.
        limits: {name: (lower, upper)} overriding joints.json (rad, before the margin), e.g. the travel bounds used while a
        joint is not yet calibrated. torque: {name: N·m} overriding the rated/peak bound (e.g. the low calibration torque).
        power: battery-current budget on (default) or off; rest_a: allowance (A) for the rest of the robot (sw/power.py)."""
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
        self._margin = m
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
        # D25 people-near ranges (rad, before the margin; ±inf where a joint has none) and the moving bound while exiting
        pn = [j.get("people_near") for j in J]
        self.pn_lo = np.array([p[0] + m if p else -np.inf for p in pn])
        self.pn_hi = np.array([p[1] - m if p else np.inf for p in pn])
        self.people_near = False
        self._pn_lo, self._pn_hi = None, None
        self.fb_max_age = float(fb_max_age)      # bring-up passes bringup.yaml fb_max_age_s (sized on the bus-realism runs)
        # W8a battery-current budget (sw/power.py)
        self.current = CurrentModel(J, self.rated)
        self.power = PackBudget(rest_a) if power else None
        self.i_pred = None                       # predicted pack current (A) of the last verdict: rest + this arm
        self._arm_last = 0.0                     # this arm's model draw last cycle (A), to split a BMS reading

    # ---- battery-current budget (W8a)
    def _rest(self, bms):
        """Rest-of-robot draw (A): the fixed allowance, or more if the BMS measures more than this arm's model draw."""
        r = self.power.rest_a
        if bms is not None and bms.current_a is not None and math.isfinite(bms.current_a):
            r = max(r, float(bms.current_a) - self._arm_last)
        return r

    def _power_record(self, now, dt, arm_a, bms, extra=()):
        """Log this cycle's pack current into the 30 s windows (drawn: the larger of model and BMS measurement)."""
        rest = self._rest(bms)
        self._arm_last = arm_a
        total = rest + arm_a
        self.i_pred = total
        rec = max([total, *extra])
        if bms is not None and bms.current_a is not None and math.isfinite(bms.current_a):
            rec = max(rec, float(bms.current_a))
        self.power.record(now, rec, dt)

    def _power_limit(self, now, dt, q2, qm, vm, tf, kp_, kd_, e, v_bus, bms, tau_meas):
        """9. Battery budget on a command already inside the torque bound: peak cap (I_PEAK), then the 30 s derate."""
        P, C = self.power, self.current.at(v_bus)
        rest = self._rest(bms)
        base = rest + C.idle
        ceiling = P.update(base + K_MIN * C.full_rated)
        tau = kp_ * (q2 - qm) - kd_ * vm + tf
        dyn = C.dynamic(tau, vm)
        if base + dyn.sum() > P.i_peak:                                  # peak: scale the motoring joints' bounds
            self._count("current cap")
            s = max(P.i_peak - base, 0.0) / dyn.sum()
            lim_c = np.where(dyn > 0, s * np.abs(tau), np.abs(tf) + kp_ * e + kd_ * np.abs(vm))
            tf, kp_, kd_ = _trim(lim_c, np.clip(tf, -lim_c, lim_c), kp_, kd_, e, vm)
            tau = kp_ * (q2 - qm) - kd_ * vm + tf
            dyn = C.dynamic(tau, vm)
        P.k = 1.0
        if ceiling < P.i_peak and base + dyn.sum() > ceiling:            # 30 s mean: common factor k on the torque limits
            # braking joints give energy back: they keep their limit (unchanged, they stay braking and draw 0)
            terms = np.abs(tf) + kp_ * e + kd_ * np.abs(vm)
            mot, tabs = dyn > 0, np.abs(tau)
            k = derate_factor(ceiling - base, C.a * mot, tabs, self.rated)
            P.k = k
            self._count("power derate")
            lim_d = np.where(mot & (tabs > k * self.rated), k * self.rated, terms)
            tf, kp_, kd_ = _trim(lim_d, np.clip(tf, -lim_d, lim_d), kp_, kd_, e, vm)
            tau = kp_ * (q2 - qm) - kd_ * vm + tf
            dyn = C.dynamic(tau, vm)
        extra = ()
        if tau_meas is not None and _finite(tau_meas, self.n):          # what the drives measured last cycle
            extra = (rest + C.total(tau_meas, vm),)
        self._power_record(now, dt, C.idle + float(dyn.sum()), bms, extra=extra)
        return tf, kp_, kd_

    def set_limits(self, limits):
        """Replace the bounds of some joints ({name: (lower, upper)}, rad, before the margin). Used by bring-up after a
        power cycle: an uncalibrated joint without an absolute output encoder reads a new, arbitrary angle on power-up,
        so its travel window has to be re-anchored on the new reading before reset() can accept the feedback."""
        for k, n in enumerate(self.names):
            if n in limits:
                self.lo[k], self.hi[k] = limits[n][0] + self._margin, limits[n][1] - self._margin

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

    # ---- people-near mode (D25)
    def set_people_near(self, on):
        """Restrict the joints that have a people-near range (shoulder roll ≥ +5°) while a person is close. Entering with a
        joint inside the restricted zone walks it out at PN_EXIT_SPEED (no stop); leaving restores the full range."""
        on = bool(on)
        if on and not self.people_near:
            q = self.last_q if self.last_q is not None else np.clip(np.zeros(self.n), self.lo, self.hi)
            self._pn_lo = np.minimum(np.maximum(self.lo, self.pn_lo), q)        # start at the arm, never above it
            self._pn_hi = np.maximum(np.minimum(self.hi, self.pn_hi), q)
            self._count("people-near on")
        elif not on:
            self._pn_lo = self._pn_hi = None
        self.people_near = on

    def target_bounds(self):
        """The target bounds in force this cycle (rad): the joint limits, narrowed by the people-near bound when on."""
        if self._pn_lo is None:
            return self.lo, self.hi
        return np.maximum(self.lo, self._pn_lo), np.minimum(self.hi, self._pn_hi)

    def people_near_exiting(self):
        """True while people-near mode is on and some joint is still being walked out of its restricted zone."""
        if self._pn_lo is None:
            return False
        return bool(np.any(self._pn_lo < np.maximum(self.lo, self.pn_lo) - 1e-12) or np.any(self._pn_hi > np.minimum(self.hi, self.pn_hi) + 1e-12))

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
        # A collision latch is a compliant hold (a controlled state): the hardware heartbeat must keep running, or the
        # contactor would open and the arm drop (Fable review 2026-11-27, C2). Any other latched fault drops it.
        return self.healthy_cycle and not [f for f in self.latched if f != "collision"]

    def reset(self, q_meas, fb_age):
        """Clear latched faults: only with the button released and fresh, finite, in-limit feedback."""
        if self.estop_pressed:
            return False
        if not _finite(q_meas, self.n) or fb_age is None or fb_age > self.fb_max_age:
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
              tau_meas=None, bms=None) -> Verdict:
        self.healthy_cycle = False
        now = self.clock()
        v_bus = bms.voltage_v if bms is not None else None
        if self.last_tick is None or now - self.last_tick > WATCHDOG_S:
            self._latch("watchdog")
        if not (_finite(q_meas, self.n) and _finite(v_meas, self.n)) or fb_age is None or not math.isfinite(fb_age) or fb_age > self.fb_max_age:
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
        if bms is not None and bms.fault:
            self._latch("BMS over-current or under-voltage")
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
            if self.power is not None:                                   # logged, not capped: the hold carries only gravity
                vh = np.asarray(v_meas, float) if _finite(v_meas, self.n) else np.zeros(self.n)
                tau_h = kp_c * (self._anchor - qm) - KD_COMPLIANT * vh + g
                self._power_record(now, dt, self.current.at(v_bus).total(tau_h, vh), bms)
            return Verdict(False, self._anchor.copy(), kp_c, np.full(self.n, KD_COMPLIANT), g, ["collision: compliant hold"], compliant=True)
        self._anchor = None
        if self.latched:
            if self.power is not None:                                   # drives disabled: logic idle only
                self._power_record(now, dt, self.current.at(v_bus).idle, bms)
            return Verdict(True, reasons=list(self.latched))
        qm, vm = np.asarray(q_meas, float), np.asarray(v_meas, float)
        # 3. limits (people-near bound when on), slew, workspace
        if self._pn_lo is not None:                                    # D25: the bound walks to the people-near range
            self._pn_lo = np.minimum(np.maximum(self.lo, self.pn_lo), self._pn_lo + PN_EXIT_SPEED * dt)
            self._pn_hi = np.maximum(np.minimum(self.hi, self.pn_hi), self._pn_hi - PN_EXIT_SPEED * dt)
        lo, hi = self.target_bounds()
        qw = np.asarray(q_want, float)
        q = np.clip(qw, lo, hi)
        if np.any(q != q_want):
            self._count("joint limit clamp")
            if self._pn_lo is not None and np.any((qw < lo) & (qw >= self.lo) | (qw > hi) & (qw <= self.hi)):
                self._count("people-near clamp")
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
        tf = np.clip(tf, -lim, lim)
        if np.any(np.abs(tf) + kp_ * e + kd_ * np.abs(vm) - lim > 0):
            self._count("torque bound")
            tf, kp_, kd_ = _trim(lim, tf, kp_, kd_, e, vm)
        if self.power is not None:
            tf, kp_, kd_ = self._power_limit(now, dt, q2, qm, vm, tf, kp_, kd_, e, v_bus, bms, tau_meas)
        self.last_q = q2
        self.healthy_cycle = True
        return Verdict(False, q2, kp_, kd_, tf, [])

    @staticmethod
    def drive_torque(v: Verdict, q_meas, v_meas):
        """What an MIT-mode drive will produce for this verdict (for tests and logs)."""
        return v.kp * (v.q - np.asarray(q_meas)) - v.kd * np.asarray(v_meas) + v.t_ff


class PeopleNearTrigger:
    """What switches the supervisor's people-near mode (D25; docs/SOFTWARE_PLAN.md). Input: the distances (m, from the
    robot's mast axis) of the people perception currently sees, fused from the head RGB-D person detector and the lidar
    leg tracker; a stub until perception exists (the caller passes the list, or None when perception is down).
    On as soon as anyone is within PN_RADIUS (1.5 m); off only after nobody has been within PN_RELEASE (1.8 m) for PN_HOLD_S
    (2 s). Fail-safe: no perception input, or an input older than PN_STALE_S, counts as a person near."""

    def __init__(self, clock=time.monotonic, radius=PN_RADIUS, release=PN_RELEASE, hold_s=PN_HOLD_S, stale_s=PN_STALE_S):
        self.clock, self.radius, self.release, self.hold_s, self.stale_s = clock, radius, release, hold_s, stale_s
        self.on, self._clear_since, self._last_input = True, None, None        # starts on until perception reports

    def update(self, distances=None):
        """distances: iterable of person distances (m) this frame, or None when perception has nothing new."""
        now = self.clock()
        if distances is not None:
            self._last_input = now
        stale = self._last_input is None or now - self._last_input > self.stale_s
        d = [float(x) for x in distances] if distances is not None else []
        if stale or any(x <= self.radius for x in d):
            self.on, self._clear_since = True, None
        elif self.on:
            if any(x <= self.release for x in d):
                self._clear_since = None
            elif self._clear_since is None:
                self._clear_since = now
            elif now - self._clear_since >= self.hold_s:
                self.on = False
        return self.on
