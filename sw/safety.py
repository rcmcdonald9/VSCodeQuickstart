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
   (0.2 rad/s), never more than one step ahead of the last target (a held step, e.g. SW-T1 walk-in, doesn't let it run
   ahead and the arm then jump), so the arm is walked out slowly; the faults (position outside limits, tracking) still use the hard
   limits. Leaving the mode restores the full range at once.
9. **Battery-current budget (W8a, electrical.md *Pack and BMS*; sw/power.py):** every cycle the drives' bus current is
   predicted from the torque the verdict will produce: electrical input power / V_bus per joint (copper loss k·R·I_ph² +
   τ·ω / η motoring; braking offsets only its own copper loss) + idle
   plus REST_A (9.9 A) for the rest of the robot. Over **70 A** the motoring joints' torque bounds are scaled down (τ_ff
   first, then kp, then kd, as in 2.). The **30 s rolling mean** is held at ≤ **28 A**: a ceiling that falls from 70 A
   towards 28 A as the window's room is used up (and below 28 A in proportion if the mean is ever over) derates the
   per-joint torque limits to k · rated, one k for the motoring joints, floor K_MIN · rated and 1.25 × gravity. Neither ever stops the arm or drops
   the heartbeat; counted as "current cap" / "power derate". A BMS over-current or under-voltage flag (`bms=BmsReading(...)`,
   stub of the CAN A telemetry) latches a fault. `power=False` turns the budget off (tests only).
10. **SW-T1 person within reach (A10 v2, rnd/tip.py):** one people-near mode (`set_people_near(True)`, alias
   `set_person_near`, switched by PeopleNearTrigger at 1.5 m, so it covers "within reach" ≈ 1 m with room) applies both the
   D25 roll bound (8.) and SW-T1: hand targets must stay inside PERSON_WS = LOAD_WS with x ≤ 0.35 m and z ≤ 0.60 m (arms
   in, lift down), intersected with the load box when load mode is on too (the lower limits win); the base is capped at
   ≤ 0.1 m/s and ≤ 0.1 m/s² (`base_speed_limit()`, `base_accel_limit()`), and stopped (0 m/s) with more than PERSON_MAX_KG
   (2.5 kg, e.g. the R2d basket) in the hands: it has to be set down first. Entering the mode with a hand outside the box
   does not freeze the arm: a step is accepted while it brings the hand no further from the box (same for the load box).
11. **SW-T2 caster lock (D18b):** `casters_locked()` is true by default (spring-applied lock, solenoid off, so a fault or
   power loss locks them). `release_casters("threshold")` (D2b crossing, ≤ 2.5 kg, speed ≤ threshold_speed()) or
   `release_casters("transit")` (no load, arms parked: the planner's call, speed ≤ BASE_V_MAX) is refused with a person
   near or a latched fault, and is undone at once if either appears or the payload rises past the mode's limit.
   `lock_casters()` re-locks with a re-seat pulse (CASTER_PULSE_S released, then locked); a pulse also follows every stop
   and every CASTER_RESEAT_M of locked travel (`limit_base()` keeps the count), so a lock never ratchets down into a dip.
   Locked: speed ≤ 0.3 m/s; released or pulsing: accel ≤ CASTER_RELEASED_A (wheels-only diamond). `caster_state()` and
   `caster_solenoid()` are what the planner and the lock output read.
12. **SW-T4 threshold speed:** `threshold_speed()` = 0.12 m/s (was 0.15; ballast mass, rnd/tip.py R4 table): a planner
   limit for D2b crossings, applied as the base speed cap while the casters are released for a threshold. The supervisor
   does not detect thresholds. `limit_base(v, dt)` clamps a planar base velocity command to the speed and accel limits in
   force (a speed above a newly lowered limit is braked at the accel limit it was moving under); any latched fault → 0 m/s.
13. **People-near hand speed (R17 140 N first impact; rnd/hand_speed.py, Astra review #3 Part A):** in people-near mode the
   per-joint slew drops to PN_JOINT_V (0.75 rad/s) and the hand's speed relative to the base to `hand_speed_cap()` =
   V_HAND_PN(payload) − |base speed|: the closing speed at which the arm's directional effective mass (reflected rotor
   inertias included, worst contact normal, worst people-near pose, today's arm and D20) gives ≤ 140 N on a 75 N/mm body
   region. With an fk the hand displacement is measured (every point fk returns); without one it is bounded by
   Σ|Δq_i|·HAND_LEVER_i. A step over the cap is scaled down whole (direction kept), counted as "hand speed cap".
14. **SW-T6 person approaching (A10 from the real states; rnd/tip.py, Astra review #3 finding 2):** a push can arrive before
   the SW-T1 transition is done (person detected at 1.5 m, walking 1.6 m/s: ≈ 0.3 s; the transition takes up to ≈ 1.5 s),
   and from the states the robot is actually in then, 8 of the 16 A10 cases fail. `set_person_approach(True)` (a second
   PeopleNearTrigger at PN_PREP_RADIUS, 3.5 m) applies the SW-T1 part early: the PERSON_WS hand box (walk-in), base ≤
   PERSON_V / PERSON_A after braking at the limit it was moving under, casters locked, the base stopped above PERSON_MAX_KG.
   People-near mode implies it. The D25 roll bound, the roll latch and the hand-speed cap (13.) stay at 1.5 m.
15. **SW-T7 basket carry (R2d, > PERSON_MAX_KG):** the base moves with more than PERSON_MAX_KG in the hands only while
   `set_area_clear(True)` says perception sees nobody within BASKET_CLEAR_R (9 m: the basket has to be set down before a
   walking person can reach the robot; rnd/tip.py). Default False (fail-safe): no clear-area input, no basket travel.
16. **Roll latch (R17 safe power-off, rnd/power_off.py option O2; on with PIP_ROLL_GUARD=latch or roll_latch=True):** a
   spring-applied pin stops the shoulder roll at +5° whenever its solenoid is off. `roll_latch_solenoid()` (the output,
   True = energised = pin retracted) is on only while the supervisor is healthy and nobody is near; until the pin-retracted
   switch confirms (`set_latch_switch`), or LATCH_RETRACT_S has passed without a switch, the roll target bound stays at
   the people-near limit (never above the arm: it only stops the arm being driven into the pin). A person near or any
   latched fault drops the pin at once (the contactor feeds the solenoid, so K1 opening drops it in hardware too).
   While the pin may be in, the roll torque bound is LATCH_T_MAX (16 N·m: its guide's printed load path). ROLL_PARK_DEG:
   where the planner parks the roll when idle or before a planned power-down.
Every override is counted so a session log shows how often the policy was overruled.
"""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from sw.collision import KD_COMPLIANT, KP_COMPLIANT, T_ANCHOR
from sw.power import BmsReading, CurrentModel, K_MIN, PackBudget, REST_A, fit_factor  # noqa: F401 (BmsReading: callers)

WATCHDOG_S, FB_MAX_AGE, TEMP_C = 0.050, 0.020, 70.0     # TEMP_C: driver/case sensor (thermal.md: winding ≤ 69 °C on a task day)
# FB_MAX_AGE 10 → 20 ms (2026-11-27, docs/sw/bus_realism.md): with 0–9 ms of bus delay each way the feedback age reached 16 ms
# in normal running (p99.9 12 ms) and 10 ms false-tripped; a dead bus is still stopped in 22–30 ms (frames already in flight keep it fresh), next to the drive's own 30 ms timeout
V_FAULT, TRACK_FAULT = 4.0, 0.35                          # rad/s measured, rad tracking error
KP_MAX, KD_MAX = 60.0, 3.0
LOAD_KG = 1.0
LOAD_WS = ((0.10, -0.45, 0.20), (0.45, 0.45, 0.80))               # base frame, m (R2d basket study)                                # N·m/rad, N·m·s/rad at the joint (bench defaults)
PN_EXIT_SPEED = 0.2               # rad/s: how fast entering people-near mode walks a joint out of its restricted zone (D25)
PN_RADIUS, PN_RELEASE, PN_HOLD_S, PN_STALE_S = 1.5, 1.8, 2.0, 0.5   # people-near trigger: on within 1.5 m, off beyond 1.8 m for 2 s
# base limits (A10 v2, rnd/tip.py): accel m/s², speed m/s
A_FREE, A_LOAD = 1.0, 0.3         # base accel cap unloaded / in load mode (tip margin while braking)
BASE_V_MAX = 0.8                  # R6 top speed: only with the casters released for transit
PERSON_V, PERSON_A = 0.1, 0.1     # SW-T1: base speed / accel cap with a person within reach
PERSON_X, PERSON_Z = 0.35, 0.60   # SW-T1: hand box x max / z max (m, base frame) with a person within reach
PERSON_WS = (LOAD_WS[0], (PERSON_X, LOAD_WS[1][1], PERSON_Z))
PERSON_MAX_KG = 2.5               # SW-T1: heavier loads (R2d basket) are not carried with a person within reach: base stops
CASTER_LOCKED_V = 0.3             # SW-T2: base speed cap with the casters locked
CASTER_RELEASED_A = 0.5           # SW-T2: accel cap while released / pulsing (wheels only; tip.py: transit passes at 1.0, 99 mm; ESTIMATE margin)
CASTER_PULSE_S, CASTER_RESEAT_M = 0.10, 0.30   # SW-T2: re-seat pulse length; locked travel between pulses
CASTER_TRANSIT_KG = 0.1           # SW-T2: "unloaded" for a transit release
THRESHOLD_SPEED = 0.12            # SW-T4: D2b threshold crossing speed (was 0.15)
THRESHOLD_MAX_KG = 2.5            # SW-T2: heaviest load a threshold release is analysed for (tip.py: 2.5 kg in the load box)
V_STOPPED = 0.01                  # m/s: below this the base counts as stopped (re-seat pulse on stopping)
# 13. people-near hand speed (rnd/hand_speed.py: 140 N on 75 N/mm, worst people-near pose, today's arm and D20, rotors included)
PN_JOINT_V = 0.75                 # rad/s per-joint slew in people-near mode (at the 1.5 rad/s default a folded-elbow pose reaches 140 N at 0.07 m/s, D20)
V_HAND_PN = ((0.0, 0.345), (1.0, 0.210), (2.5, 0.160), (4.535, 0.140))   # (payload kg up to, closing-speed cap m/s); rnd/hand_speed.py caps() (sampled worst × 0.9)
HAND_LEVER = {"pitch": 0.74, "roll": 0.74, "yaw": 0.44, "elbow": 0.44, "wpitch": 0.16, "wroll": 0.03}   # m: max hand distance from each axis (params l_upper 0.30 + l_fore 0.28 + l_hand 0.156)
# 14./15. SW-T6 person approaching, SW-T7 basket carry (rnd/tip.py: R = R_reach 0.7 + 1.6 m/s × (0.2 s perception + transition))
PN_PREP_RADIUS, PN_PREP_RELEASE = 3.5, 4.0     # m: transition ≤ 1.5 s (arms into PERSON_WS at the 1.5 rad/s slew, base braked)
BASKET_CLEAR_R = 9.5                            # m: ≥ 0.7 + 1.6 × (0.2 + 5 s to stop + set the basket down, ESTIMATE) = 9.0
# 16. roll latch (rnd/power_off.py O2)
ROLL_GUARD = os.environ.get("PIP_ROLL_GUARD", "none")   # "none" | "latch" (CAD: rnd/parts/roll.py; default off, owner's call)
LATCH_RETRACT_S = 0.05            # s: solenoid pull-in + pin travel before the roll may go below +5° without a switch (ESTIMATE)
ROLL_PARK_DEG = 8.0               # deg: the roll park angle (idle / planned power-down) above the latch
LATCH_T_MAX = 16.0                # N·m: roll torque cap while the pin may be in (0.39 kN at r 41.5: the printed guide, ≈ 12 MPa; power_off.md)
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


def _box_dist(p, lo, hi):
    """Squared distance of the point(s) p outside the box (0 inside)."""
    p = np.asarray(p, float)
    return float(np.sum(np.maximum(lo - p, 0) ** 2 + np.maximum(p - hi, 0) ** 2))


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
                 collision=None, joints=None, limits=None, torque=None, fb_max_age=FB_MAX_AGE, power=True, rest_a=REST_A, roll_latch=None):
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
        self._load_box = self._person_box = None
        # SW-T2 casters ("locked" | "released" | "reseat") and the base command state (SW-T1/T2/T4)
        self._caster, self._caster_reason, self._pulse_t0, self._locked_travel = "locked", None, None, 0.0
        self._v_base = np.zeros(2)
        self._a_moving = A_FREE
        self.collision = collision
        self._anchor = None
        # D25 people-near ranges (rad, before the margin; ±inf where a joint has none) and the moving bound while exiting
        pn = [j.get("people_near") for j in J]
        self.pn_lo = np.array([p[0] + m if p else -np.inf for p in pn])
        self.pn_hi = np.array([p[1] - m if p else np.inf for p in pn])
        self.people_near = False
        self._pn_lo, self._pn_hi = None, None
        self._approach, self._approach_box, self._near_box = False, None, None     # SW-T6 (14.)
        self.area_clear = False                                                    # SW-T7 (15.): fail-safe
        self._lever = np.array([HAND_LEVER.get(n, 0.8) for n in self.names])
        # 16. roll latch: "engaged" (solenoid off, the default) | "retracting" | "retracted"
        self.roll_latch = (ROLL_GUARD == "latch" if roll_latch is None else bool(roll_latch)) and "roll" in self.names
        self._rl_state, self._latch_t0, self._latch_switch = "engaged", None, None
        self._latch_floor = np.full(self.n, -np.inf)
        k = self.names.index("roll") if "roll" in self.names else None
        self._latch_lo = np.full(self.n, -np.inf)
        if k is not None and np.isfinite(self.pn_lo[k]):
            self._latch_lo[k] = self.pn_lo[k]
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
        """Log this cycle's pack current into the 30 s window (the larger of model, measured torque and BMS current)."""
        rest = self._rest(bms)
        self._arm_last = arm_a
        total = rest + arm_a
        self.i_pred = total
        rec = max([total, *extra])
        if bms is not None and bms.current_a is not None and math.isfinite(bms.current_a):
            rec = max(rec, float(bms.current_a))
        self.power.record(now, rec, dt)

    def _power_limit(self, now, dt, q2, qm, vm, tf, kp_, kd_, e, v_bus, bms, tau_meas):
        """9. Battery budget on a command already inside the torque bound: peak cap (I_PEAK), then the 30 s derate. Only
        motoring joints are trimmed; braking joints keep their command (they stay braking, their draw is unchanged)."""
        P, C = self.power, self.current.at(v_bus)
        rest = self._rest(bms)
        base = rest + C.idle
        ceiling = P.update(base + float(np.sum(C.bound(K_MIN * self.rated, vm))))
        tau = kp_ * (q2 - qm) - kd_ * vm + tf
        dyn = C.dynamic(tau, vm)
        P.k = 1.0
        for kind, cap in (("current cap", P.i_peak), ("power derate", ceiling)):
            if kind == "power derate" and ceiling >= P.i_peak:
                break
            if base + dyn.sum() <= cap:
                continue
            mot = tau * vm >= 0
            tabs = np.abs(tau)
            budget = cap - base - float(np.sum(np.where(mot, 0.0, dyn)))
            if kind == "current cap":                                    # scale the motoring joints' torque bounds
                ref, floor = tabs, np.zeros(self.n)
                x = fit_factor(C, budget, tabs, ref, vm, mot)
            else:                                                        # common factor k on the rated torque limits
                ref, floor = self.rated, K_MIN * self.rated
                if self.collision is not None and hasattr(self.collision, "twin"):   # never below what holds the arm up
                    floor = np.maximum(floor, 1.25 * np.abs(self.collision.twin.gravity(qm)))
                x = fit_factor(C, budget, tabs, ref, vm, mot, lo=K_MIN, floor=floor)
                P.k = x
            self._count(kind)
            cut = np.maximum(x * ref, floor)
            lim = np.where(mot & (tabs > cut), cut, np.abs(tf) + kp_ * e + kd_ * np.abs(vm))
            tf, kp_, kd_ = _trim(lim, np.clip(tf, -lim, lim), kp_, kd_, e, vm)
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
        self._load_box = tuple(map(np.array, box or LOAD_WS)) if self.payload > LOAD_KG else None
        self._update_ws()
        self._caster_tick()                                         # a heavier load may end a caster release

    def _update_ws(self):
        """Workspace in force: the free box, intersected with the load box (load mode) and the SW-T1 box (people near)."""
        lo, hi = self._ws_free[0].copy(), self._ws_free[1].copy()
        for b in (self._load_box, self._person_box if self.approaching() else None):
            if b is not None:
                lo, hi = np.maximum(lo, b[0]), np.minimum(hi, b[1])
        self.ws_lo, self.ws_hi = lo, hi

    # ---- people-near mode (D25)
    def set_people_near(self, on, box=None):
        """People-near mode (D25 + SW-T1, one mode). Restrict the joints that have a people-near range (shoulder roll ≥ +5°);
        entering with a joint inside the restricted zone walks it out at PN_EXIT_SPEED (no stop). Hand targets must stay in
        the SW-T1 box (box: (lo, hi) in the supervisor's FK frame; default PERSON_WS, intersected with the load box in load
        mode); base ≤ PERSON_V / PERSON_A, casters locked. Leaving restores the full range and the free limits at once."""
        on = bool(on)
        self._near_box = tuple(map(np.array, box or PERSON_WS)) if on else None
        self._person_box = self._near_box or self._approach_box
        if on and not self.people_near:
            q = self.last_q if self.last_q is not None else np.clip(np.zeros(self.n), self.lo, self.hi)
            self._pn_lo = np.minimum(np.maximum(self.lo, self.pn_lo), q)        # start at the arm, never above it
            self._pn_hi = np.maximum(np.minimum(self.hi, self.pn_hi), q)
            self._count("people-near on")
        elif not on:
            self._pn_lo = self._pn_hi = None
        self.people_near = on
        self._update_ws()
        self._caster_tick()                                         # a person near ends any caster release

    set_person_near = set_people_near                               # SW-T1 name (rnd/tip.py); the same single mode

    def set_person_approach(self, on, box=None):
        """SW-T6 (14.): a person within PN_PREP_RADIUS. The SW-T1 hand box, base caps, caster lock and basket stop, without
        the D25 roll bound, the roll latch or the hand-speed cap (those come with set_people_near at 1.5 m)."""
        self._approach = bool(on)
        self._approach_box = tuple(map(np.array, box or PERSON_WS)) if self._approach else None
        self._person_box = self._near_box or self._approach_box
        if self._approach:
            self._count("person approaching")
        self._update_ws()
        self._caster_tick()

    def approaching(self):
        """SW-T1 limits in force: a person approaching (SW-T6) or near (people-near mode implies it)."""
        return bool(getattr(self, "people_near", False) or getattr(self, "_approach", False))

    def set_area_clear(self, on):
        """SW-T7 (15.): perception sees nobody within BASKET_CLEAR_R. Needed for the base to move with > PERSON_MAX_KG."""
        self.area_clear = bool(on)

    # ---- people-near hand speed (13.)
    def hand_speed_cap(self):
        """Hand speed cap relative to the base (m/s) in people-near mode: V_HAND_PN for the payload minus the base speed."""
        p = self.payload
        v = next((v for kg, v in V_HAND_PN if p <= kg + 1e-9), None)
        if v is None:                                               # past the table: F ∝ √(m + p) ≥ √p
            v = V_HAND_PN[-1][1] * math.sqrt(V_HAND_PN[-1][0] / p)
        return max(v - float(np.hypot(*self._v_base)), 0.0)

    def _hand_speed(self, dq, dt):
        if self.fk is not None:
            a = np.atleast_2d(self.fk(self.last_q))
            b = np.atleast_2d(self.fk(self.last_q + dq))
            return float(np.max(np.linalg.norm(b - a, axis=1))) / dt
        return float(np.sum(np.abs(dq) * self._lever)) / dt

    def _hand_cap(self, q2, dt):
        dq = q2 - self.last_q
        if not np.any(dq):
            return q2
        cap, s, it = self.hand_speed_cap(), self._hand_speed(dq, dt), 0
        while s > cap + 1e-12 and it < 8:
            dq = dq * (0.999 * cap / s) if s > 0 else dq * 0
            s, it = self._hand_speed(dq, dt), it + 1
        if it:
            self._count("hand speed cap")
        return self.last_q + dq

    # ---- roll latch (16.)
    def set_latch_switch(self, retracted):
        """Pin-retracted switch reading: True / False, or None when the latch has no switch (time-based)."""
        self._latch_switch = None if retracted is None else bool(retracted)

    def _latch_tick(self):
        if not self.roll_latch:
            return
        want = not self.people_near and not self.latched
        if not want:
            if self._rl_state != "engaged":
                self._count("roll latch dropped")
            self._rl_state, self._latch_t0 = "engaged", None
        elif self._rl_state == "engaged":
            self._rl_state, self._latch_t0 = "retracting", self.clock()
        elif self._rl_state == "retracting":
            if self._latch_switch or (self._latch_switch is None and self.clock() - self._latch_t0 >= LATCH_RETRACT_S):
                self._rl_state = "retracted"
        elif self._rl_state == "retracted" and self._latch_switch is False:          # the switch disagrees: treat as not retracted
            self._rl_state, self._latch_t0 = "retracting", self.clock()
            self._count("roll latch switch disagrees")

    def _latch_lim(self, lim):
        """16.: while the pin may be in, the roll torque stays ≤ LATCH_T_MAX (its printed guide's load path, roll.py LATCH)."""
        if not self.roll_latch or self._rl_state == "retracted":
            return lim
        lim = lim.copy()
        k = self.names.index("roll")
        lim[k] = min(lim[k], LATCH_T_MAX)
        return lim

    def roll_latch_solenoid(self):
        """The latch output: True = solenoid energised (pin retracted or retracting). False (the spring-applied pin in) on a
        person near or any latched fault; always False with the latch option off."""
        self._latch_tick()
        return self.roll_latch and self._rl_state != "engaged"

    def latch_state(self):
        self._latch_tick()
        return self._rl_state if self.roll_latch else None

    def target_bounds(self):
        """The target bounds in force this cycle (rad): the joint limits, narrowed by the people-near bound when on."""
        lo, hi = self.lo, self.hi
        if self._pn_lo is not None:
            lo, hi = np.maximum(lo, self._pn_lo), np.minimum(hi, self._pn_hi)
        if self.roll_latch and self._rl_state != "retracted":
            lo = np.maximum(lo, self._latch_floor)
        return lo, hi

    def people_near_exiting(self):
        """True while people-near mode is on and some joint is still being walked out of its restricted zone."""
        if self._pn_lo is None:
            return False
        return bool(np.any(self._pn_lo < np.maximum(self.lo, self.pn_lo) - 1e-12) or np.any(self._pn_hi > np.minimum(self.hi, self.pn_hi) + 1e-12))

    # ---- base limits (SW-T1 / SW-T2 / SW-T4); the planner reads these, limit_base() enforces them on a command
    def base_accel_limit(self):
        """Base accel / decel cap (m/s²): 1.0, 0.3 in load mode, ≤ 0.1 with a person near, ≤ 0.5 with the casters not
        locked. Also callable on a bare namespace with only `payload` (rnd/tip.py reads it that way)."""
        a = A_LOAD if self.payload > LOAD_KG else A_FREE
        if Supervisor.approaching(self):
            a = min(a, PERSON_A)
        if getattr(self, "_caster", "locked") != "locked":
            a = min(a, CASTER_RELEASED_A)
        return a

    def base_speed_limit(self):
        """Base speed cap (m/s): 0.3 with the casters locked (or pulsing), threshold_speed() while released for a threshold,
        BASE_V_MAX released for transit; ≤ 0.1 with a person near, 0 with a person near and > PERSON_MAX_KG in the hands,
        0 with any latched fault."""
        self._caster_tick()
        if self.latched or (self.payload > PERSON_MAX_KG and (self.approaching() or not self.area_clear)):
            return 0.0                                              # SW-T1 / SW-T7: no basket travel with anyone about
        v = CASTER_LOCKED_V
        if self._caster == "released":
            v = THRESHOLD_SPEED if self._caster_reason == "threshold" else BASE_V_MAX
        return min(v, PERSON_V) if self.approaching() else v

    @staticmethod
    def threshold_speed():
        """SW-T4: the planner's D2b threshold crossing speed (m/s)."""
        return THRESHOLD_SPEED

    def limit_base(self, v_want, dt):
        """Clamp a base velocity command (vx, vy[, wz]; m/s, the planar part only) to base_speed_limit() and
        base_accel_limit(). A speed above a newly lowered limit (a person appears, a fault, a re-lock) is braked at the
        accel limit it was moving under. Also keeps the caster re-seat bookkeeping (pulse on stopping, every 0.3 m locked)."""
        self._caster_tick()
        v = np.nan_to_num(np.array(v_want, float), nan=0.0, posinf=0.0, neginf=0.0)
        lin, last = v[:2].copy(), self._v_base
        vmax, amax = self.base_speed_limit(), self.base_accel_limit()
        sp = float(np.hypot(*lin))
        if sp > vmax:
            lin *= vmax / sp
            self._count("base speed clamp")
        a_ok = max(amax, self._a_moving) if np.hypot(*last) > vmax + 1e-9 else amax
        dv = lin - last
        n = float(np.hypot(*dv))
        if n > a_ok * dt:
            lin = last + dv * (a_ok * dt / n)
            self._count("base accel clamp")
        if np.hypot(*lin) <= vmax + 1e-9:
            self._a_moving = amax
        moving_before, moving = np.hypot(*last) > V_STOPPED, np.hypot(*lin) > V_STOPPED
        self._v_base = lin
        if self._caster == "locked":
            self._locked_travel += float(np.hypot(*lin)) * dt
            if (moving_before and not moving) or self._locked_travel >= CASTER_RESEAT_M:
                self._caster_pulse()
        v[:2] = lin
        return v

    # ---- caster lock (SW-T2, D18b)
    def _caster_tick(self):
        """Advance the caster state: end a finished re-seat pulse; lock at once on a fault, a person near, or a payload
        past the release mode's limit."""
        if not hasattr(self, "_caster"):
            return
        if self._caster != "locked":
            r = self._caster_reason
            heavy = self.payload > (CASTER_TRANSIT_KG if r == "transit" else THRESHOLD_MAX_KG)
            if self.latched or self.approaching() or (self._caster == "released" and heavy):
                self._caster, self._caster_reason, self._locked_travel = "locked", None, 0.0
                self._count("caster forced lock")
            elif self._caster == "reseat" and self.clock() - self._pulse_t0 >= CASTER_PULSE_S:
                self._caster, self._locked_travel = "locked", 0.0

    def _caster_pulse(self):
        self._caster, self._caster_reason, self._pulse_t0, self._locked_travel = "reseat", None, self.clock(), 0.0
        self._count("caster re-seat pulse")

    def release_casters(self, mode):
        """Planner request: mode "threshold" (D2b crossing, ≤ THRESHOLD_MAX_KG) or "transit" (no load, arms parked).
        Refused (False) with a person near, a latched fault or too much payload; then the casters stay locked."""
        self._caster_tick()
        lim = {"threshold": THRESHOLD_MAX_KG, "transit": CASTER_TRANSIT_KG}.get(mode)
        if lim is None or self.latched or self.approaching() or self.payload > lim:
            self._count("caster release refused")
            return False
        if self._caster != "released" or self._caster_reason != mode:
            self._count(f"caster release ({mode})")
        self._caster, self._caster_reason = "released", mode
        return True

    def lock_casters(self):
        """Planner: end a release. The lock re-engages with a re-seat pulse (CASTER_PULSE_S), then holds."""
        self._caster_tick()
        if self._caster == "released":
            self._caster_pulse()

    def casters_locked(self):
        """SW-T2 state: True when the casters are locked (the default); False while released or in a re-seat pulse."""
        self._caster_tick()
        return self._caster == "locked"

    def caster_solenoid(self):
        """The lock output: True = solenoids energised (casters free). Spring-applied, so False is the safe state."""
        return not self.casters_locked()

    def caster_state(self):
        """For the planner: {'state': locked | released | reseat, 'mode': threshold | transit | None, 'speed', 'accel'}."""
        self._caster_tick()
        return dict(state=self._caster, mode=self._caster_reason, speed=self.base_speed_limit(), accel=self.base_accel_limit())

    def workspace_ok(self):
        """True when the last target's hand(s) are inside the workspace in force (False while walking into a new box)."""
        return self.fk is None or self.last_q is None or _box_dist(self.fk(self.last_q), self.ws_lo, self.ws_hi) == 0.0

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
            lim = self._latch_lim(self.peak if allow_peak else self.rated)
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
        # 3. limits (people-near bound when on, roll latch floor), slew, hand speed, workspace
        self._latch_tick()
        if self.roll_latch:                                            # 16.: not retracted → no target below the pin (never above the arm)
            if self._rl_state == "retracted":
                self._latch_floor[:] = -np.inf
            else:
                ql = self.last_q if self.last_q is not None else qm
                self._latch_floor = np.maximum(self._latch_floor, np.minimum(self._latch_lo, ql))
        if self._pn_lo is not None:                                    # D25: the bound walks to the people-near range
            ql = self.last_q if self.last_q is not None else qm         # never more than one step ahead of the arm (a held
            self._pn_lo = np.minimum(np.maximum(self.lo, self.pn_lo), np.minimum(self._pn_lo, ql) + PN_EXIT_SPEED * dt)   # target, e.g.
            self._pn_hi = np.maximum(np.minimum(self.hi, self.pn_hi), np.maximum(self._pn_hi, ql) - PN_EXIT_SPEED * dt)   # SW-T1 walk-in)
        lo, hi = self.target_bounds()
        qw = np.asarray(q_want, float)
        q = np.clip(qw, lo, hi)
        if np.any(q != q_want):
            self._count("joint limit clamp")
            if self._pn_lo is not None and np.any((qw < lo) & (qw >= self.lo) | (qw > hi) & (qw <= self.hi)):
                self._count("people-near clamp")
        if self.last_q is None:
            self.last_q = qm.copy()
        step = (np.minimum(self.v_max, PN_JOINT_V) if self.people_near else self.v_max) * dt
        q2 = self.last_q + np.clip(q - self.last_q, -step, step)
        if np.any(np.abs(q2 - q) > 1e-12):
            self._count("velocity clamp")
        if self.people_near:                                           # 13.: hand speed relative to the base
            q2 = self._hand_cap(q2, dt)
        tf = np.zeros(self.n) if t_ff is None else np.nan_to_num(np.asarray(t_ff, float))
        if self.fk is not None:
            p = self.fk(q2)
            if np.any(p < self.ws_lo) or np.any(p > self.ws_hi):
                # outside: accepted only while it brings the hand no further from the box (entering a smaller box, SW-T1)
                if np.any(q2 != self.last_q) and _box_dist(p, self.ws_lo, self.ws_hi) <= _box_dist(self.fk(self.last_q), self.ws_lo, self.ws_hi):
                    self._count("workspace walk-in")
                else:
                    self._count("workspace reject")
                    q2, tf = self.last_q.copy(), np.zeros(self.n)
        # 2. total torque bound
        lim = self._latch_lim(self.peak if allow_peak else self.rated)
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
