"""Battery-current budget for the safety supervisor (W8a, docs/rnd/electrical.md *Pack and BMS*).

The pack (6S2P FPV pack) is rated 35 A continuous / 90 A burst, so the whole robot gets a software cap of **70 A peak** and
**28 A over a 30 s rolling mean** (rnd/electrical.SW_PACK). The supervisor (sw/safety.py) predicts every cycle what its
drives will draw and keeps the total under both, without ever stopping the arm (a power budget, not a fault).

Current model, per joint: I = |τ_joint| / (ratio · Kt) + idle, where τ_joint is the drive torque the verdict will produce
(kp·e − kd·ė + τ_ff at the measured state), ratio the belt/stage ratio outside the actuator (joints.json) and Kt the
actuator's torque constant at its output (actuators.yaml: rated torque / rated current, the lower of rated and peak where
both exist). This is the *phase* current; with the drive's output voltage below the bus voltage the bus current is at most
that (P_elec = I_ph · V_ph ≤ I_ph · V_bus), so the model is an upper bound (several times high when holding still, where
only the copper loss is drawn). A joint whose torque opposes its measured motion (τ·ω < 0, |ω| > W_REGEN) is braking: regen counts
as zero draw (not as a credit for the other joints). The STS3215 wrist roll runs from the elbow node's 12 V buck: its
12 V current is referred to the bus by 12 / (V_bus · η). Every drive also draws its logic idle current.
To this the supervisor adds a fixed allowance for the rest of the robot, REST_A: the sustained column of every non-arm
branch in electrical.md (base wheels 2.2 + lift 2.0 + 12 V servos 2.9 + 5 V compute/sensors 2.0 + AI reserve 0.8 = 9.9 A).
With a BMS reading, the rest of the robot is max(REST_A, measured pack current − this arm's model draw), so the base and
the lift keep priority when they really draw more, and the rolling mean uses the larger of model and measurement.
The budget covers the drives of one Supervisor. On the two-arm robot either one supervisor runs both arms' drives, or
each arm's supervisor gets the other arm's share in `rest_a` (Supervisor(rest_a=...)); two supervisors that each assume
the whole budget would allow twice the arm current.

Two limits (sw/safety.py Supervisor.check, after the per-joint torque bound):
1. **Peak (70 A):** if the predicted total exceeds I_PEAK, every motoring joint's torque bound is lowered to s·|τ| (the
   same s for all, so the arm's draw fits exactly) and the command is trimmed the way the per-joint torque bound trims it:
   feed-forward first, then kp, then kd. Counted as "current cap".
2. **Mean (28 A over 30 s), thermal-style derate:** PackBudget gives a ceiling on the pack current for the cycle (I_PEAK
   while the window has room; it falls towards I_AVG as the room is used up, and to I_AVG · I_AVG / mean if the mean is
   ever over, i.e. lower the further it is over). Over the ceiling, the per-joint torque limits become k · rated, the
   same k for every motoring joint (derate_factor: the largest k that fits), trimmed in the same order; braking joints
   keep theirs. k ≥ K_MIN so the arm keeps holding itself. Counted as "power derate" (cycles).
Neither stops the arm or drops the heartbeat: it is a power budget, not a fault. A BMS over-current or under-voltage flag
(BmsReading) is a fault: Supervisor.check latches it.
The window starts full of REST_A (the robot idles before the first command).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

I_PEAK, I_AVG, T_AVG = 70.0, 28.0, 30.0       # A, A, s (rnd/electrical.SW_PACK; electrical.md *Pack and BMS*)
REST_A = 2.2 + 2.0 + 2.9 + 2.0 + 0.8          # A: base + lift + 12 V servos + 5 V compute/sensors + AI reserve (electrical.md branch table, sustained)
V_MIN, BUCK_EFF = 20.0, 0.90                   # 6S cut-off (3.33 V/cell) when no BMS voltage; 24 → 12 V buck efficiency (ESTIMATE, rnd/electrical.py)
I_IDLE_DRIVE = 0.04                            # A per CAN drive, logic only (ESTIMATE, rnd/electrical.I_IDLE_DRIVER)
W_REGEN = 0.05                                 # rad/s: below this the sign of the measured speed is noise, so no regen credit
K_MIN = 0.25                                   # derate floor (fraction of rated torque): the arm keeps holding itself
BIN, H_BURST, PLAN_MARGIN = 0.1, 1.0, 0.1      # s window bins; s burst horizon of the plan; A kept under I_AVG by the plan

# actuator → (Kt at the actuator output in N·m/A, supply voltage, idle A at that supply). Kt from rnd/specs/actuators.yaml
# (rated torque / rated current; the lower of rated and peak where both are listed); GIM8108-36 has no current in the yaml:
# 3.86 N·m/A from the SDC303 reseller tables (rnd/electrical.MOTOR_CU). sw.test_s1 checks these against the yaml.
ACTUATORS = {
    "GIM6010-8": (5.0 / 10.5, 24.0, I_IDLE_DRIVE),            # 5 N·m rated at 10.5 A continuous
    "GIM8108-36": (3.86, 24.0, I_IDLE_DRIVE),                 # SDC303 (24 V)
    "DM-J4310-2EC_V1.1": (min(3.0 / 2.5, 7.0 / 7.5), 24.0, I_IDLE_DRIVE),   # 3 N·m @ 2.5 A, 7 N·m @ 7.5 A → 0.93
    "STS3215-C018": (2.94 / 2.7, 12.0, 0.18),                 # stall 2.94 N·m @ 2.7 A at 12 V; idle 0.18 A
}


@dataclass
class BmsReading:
    """Stub of the BMS telemetry on CAN A (JK-B2A8S20P): pack current and voltage, and its protection flags."""
    current_a: float | None = None
    voltage_v: float | None = None
    overcurrent: bool = False
    undervoltage: bool = False

    @property
    def fault(self):
        return bool(self.overcurrent or self.undervoltage)


class CurrentModel:
    """Per-joint bus-current model for one set of joints (rows of joints.json); `rated` (N·m) for the derate floor.
    Call at(v_bus) once per cycle (cached): it sets a (A per N·m at the bus, per joint), idle (A) and full_rated (A, the
    dynamic draw with every joint at its rated torque)."""

    def __init__(self, joints, rated):
        try:
            a = [ACTUATORS[j["actuator"]] for j in joints]
        except KeyError as ex:
            raise KeyError(f"no current model for actuator {ex} (sw/power.ACTUATORS)") from None
        self.kt = np.array([x[0] * j["ratio"] for x, j in zip(a, joints)])       # N·m/A at the joint
        self.v_supply = np.array([x[1] for x in a])
        self.idle_supply = np.array([x[2] for x in a])
        self.rated = np.asarray(rated, float)
        self._v = object()
        self.at(None)

    def at(self, v_bus):
        v = V_MIN if v_bus is None or not math.isfinite(v_bus) or v_bus <= 0 else round(float(v_bus), 1)
        if v != self._v:
            self._v = v
            to_bus = np.where(self.v_supply < 20.0, self.v_supply / (v * BUCK_EFF), 1.0)   # buck-fed servos referred to the bus
            self.a = to_bus / self.kt
            self.idle = float(np.sum(self.idle_supply * to_bus))
            self.full_rated = float(np.sum(self.rated * self.a))
        return self

    def dynamic(self, tau, w):
        """Per-joint bus current above idle (A) for joint torques tau and measured speeds w; braking joints draw 0."""
        braking = (tau * w < 0) & (np.abs(w) > W_REGEN)
        return np.where(braking, 0.0, np.abs(tau) * self.a)

    def total(self, tau, w):
        """The set's bus current (A): idle + dynamic."""
        return self.idle + float(np.sum(self.dynamic(np.asarray(tau, float), np.asarray(w, float))))


class PackBudget:
    """The pack's 30 s rolling mean (bins of BIN s, uniform inside a bin) and the derate ceiling.

    Ceiling (A) on the pack current for the coming cycle, the smaller of:
    - plan: the largest current C such that drawing C for the next H_BURST s and then the deepest-derate draw i_low
      keeps the 30 s mean ≤ I_AVG − PLAN_MARGIN at every future instant (old samples leaving the window included). As
      long as the plan is kept the fallback stays possible, so the mean never goes over; bursts up to I_PEAK are allowed
      while the window has room, and under sustained demand the ceiling settles near I_AVG and the mean converges to it
      from below.
    - reactive (above I_AVG): I_AVG · I_AVG / mean, lower the further the measured mean is over (e.g. the BMS reads
      more than the model, or the K_MIN floor bound).
    Supervisor.check turns the ceiling into a common factor k on the per-joint torque limits (derate_factor)."""

    def __init__(self, rest_a=REST_A, i_peak=I_PEAK, i_avg=I_AVG, t_avg=T_AVG):
        self.rest_a, self.i_peak, self.i_avg, self.t_avg = float(rest_a), float(i_peak), float(i_avg), float(t_avg)
        self.n = int(round(self.t_avg / BIN))
        self.bins = np.full(self.n, self.rest_a * BIN)          # A·s per bin, oldest at self.head; the robot idled before
        self.head = 0
        self.bsum = float(self.bins.sum())
        self.cur_area = self.cur_t = 0.0
        self._plan = self.i_peak                                # redone once per bin (a plan holds for H_BURST ≥ BIN)
        self._stale = True
        self.k = 1.0                                            # last derate factor applied (fraction of rated torque)
        self.ceiling = self.i_peak
        self.t_last = None
        self.last_total = self.rest_a
        self.peak_mean = self.rest_a

    def _ordered(self):
        b = np.roll(self.bins, -self.head)
        w = np.ones(self.n)
        w[0] = 1.0 - self.cur_t / BIN                           # part of the oldest bin already out of the window
        return b * w, BIN * w

    @property
    def mean(self):
        return (self.bsum - self.bins[self.head] * self.cur_t / BIN + self.cur_area) / self.t_avg

    def _add(self, dt, i):
        dt = min(dt, self.t_avg)
        while dt > 1e-12:
            take = min(dt, BIN - self.cur_t)
            self.cur_area += i * take
            self.cur_t += take
            dt -= take
            if self.cur_t >= BIN - 1e-12:
                self.bsum += self.cur_area - self.bins[self.head]
                self.bins[self.head] = self.cur_area
                self.head = (self.head + 1) % self.n
                self.cur_area = self.cur_t = 0.0
                self._stale = True
                if self.head == 0:
                    self.bsum = float(self.bins.sum())          # refresh the running sum once per window (rounding)

    def record(self, now, drawn, fallback_dt):
        """Log this cycle's pack current (A) over the interval since the last record."""
        dt = fallback_dt if self.t_last is None else now - self.t_last
        if not math.isfinite(dt) or dt < 0:
            dt = fallback_dt
        self._add(dt, float(drawn))
        self.t_last = now
        self.last_total = float(drawn)
        self.peak_mean = max(self.peak_mean, self.mean)

    def update(self, i_low):
        """Ceiling (A) on the pack current for this cycle (I_PEAK when the budget has room). i_low: the least the robot is
        sure to draw at the deepest derate (rest of the robot + idle + the arm at K_MIN · rated)."""
        m = self.mean
        c_react = self.i_avg * self.i_avg / m if m > self.i_avg else self.i_peak
        if self._stale:
            self._plan = self._make_plan(i_low)
            self._stale = False
        self.ceiling = min(self.i_peak, self._plan, c_react)
        return self.ceiling

    def _make_plan(self, i_low):
        b, d = self._ordered()
        L = np.cumsum(b)                                        # A·s leaving by future time h (oldest first)
        h = np.cumsum(d)
        area = float(L[-1]) + self.cur_area
        ia = self.i_avg - PLAN_MARGIN
        S = ia * self.t_avg - area                              # A·s of room in the window now
        pre = h <= H_BURST
        lo = min(float(i_low), ia)
        c = np.where(pre, (S + L) / np.maximum(h, 1e-9), (S + L - lo * (h - H_BURST)) / H_BURST)
        return float(np.min(c[h > 1e-9])) if np.any(h > 1e-9) else self.i_peak


def derate_factor(budget, a, terms, rated):
    """Largest k (≤ 1) with Σ a_j · min(terms_j, k · rated_j) ≤ budget: a common per-joint torque-limit factor that
    keeps the arm's draw under `budget` A whatever the command does under the limit (a_j: A per N·m at the bus, terms_j:
    |τ_ff| + kp·e + kd·|ė| already allowed, rated_j: N·m). Never below K_MIN (the arm keeps holding itself)."""
    a, t, r = (np.asarray(x, float) for x in (a, terms, rated))
    b = np.where(r > 0, t / np.maximum(r, 1e-12), np.inf)         # breakpoints: joint j saturates at k = b_j
    if np.sum(a * np.minimum(t, r)) <= budget:
        return 1.0
    k = 0.0
    for kb in np.sort(np.unique(np.concatenate([[0.0], b[np.isfinite(b)]]))):
        if kb >= 1.0 or np.sum(a * np.minimum(t, kb * r)) > budget:
            break
        k = kb
    # f is linear on [k, next breakpoint]: f(x) = fixed + x · slope
    sat = b <= k
    fixed = float(np.sum((a * t)[sat]))
    slope = float(np.sum((a * r)[~sat]))
    if slope > 0:
        k = max(k, (budget - fixed) / slope)
    return float(min(max(k, K_MIN), 1.0))
