"""Battery-current budget for the safety supervisor (W8a, docs/rnd/electrical.md *Pack and BMS*).

The pack (6S2P FPV pack) is rated 35 A continuous / 90 A burst, so the whole robot gets a software cap of **70 A peak** and
**28 A over a 30 s rolling mean** (rnd/electrical.SW_PACK). The supervisor (sw/safety.py) predicts every cycle what its
drives will draw from the battery and keeps the total under both, without ever stopping the arm (a power budget, not a fault).

Battery-current model (a PWM drive draws its electrical input power from the bus, not its phase current), per joint:
    I_ph  = |τ| / (ratio · Kt)                       phase current (τ: the drive torque the verdict will produce, at the joint)
    P_cu  = k_cu · R · I_ph²                         copper loss
    P_m   = τ · ω                                    mechanical power at the measured joint speed
    P_in  = P_cu + P_m / η            if P_m ≥ 0      (motoring)
          = max(0, P_cu − η · |P_m|)  if P_m < 0      (braking: regen offsets that joint's own copper loss, never counted
                                                      as a credit for the other joints or the rest of the robot)
    I_bat = P_in / (V_bus · η_conv) + idle
so a still hold draws only its copper loss (a few amps), not the full phase current. Kt (actuator output, N·m/A) from
rnd/specs/actuators.yaml (rated torque / rated current, the lower of rated and peak where both exist; GIM8108-36 from the
SDC303 reseller tables in rnd/electrical.MOTOR_CU); ratio = the belt/stage ratio outside the actuator (joints.json).
R (Ω, per phase) is in neither the yaml nor a datasheet we hold: ESTIMATEs in ACTUATORS with their ranges; k_cu = 1.0 as
in the electrical track's gim_bus_current (rnd/electrical.py). The convention is open until sysid: with I_ph the
peak-amplitude (Iq) current, k_cu = 1.5 for a line-to-neutral R and 0.75 for a terminal-to-terminal R, so 0.75–1.5;
the brushed STS3215 is 1.0. η = 0.85 (ESTIMATE). V_bus: the BMS's measured voltage when
there is one, else 24 V nominal. The STS3215 wrist roll runs from the elbow node's 12 V buck (η_conv = 0.90). Every drive
also draws its logic idle current.
To this the supervisor adds a fixed allowance for the rest of the robot, REST_A: the sustained column of every non-arm
branch in electrical.md (base wheels 2.2 + lift 2.0 + 12 V servos 2.9 + 5 V compute/sensors 2.0 + AI reserve 0.8 = 9.9 A).
With a BMS reading, the measurement overrides the model where it is higher: the rest of the robot is max(REST_A, measured
pack current − this arm's model draw) (so the base and the lift keep priority), and the rolling mean records the larger of
model and measurement.
The budget covers the drives of one Supervisor. On the two-arm robot either one supervisor runs both arms' drives, or
each arm's supervisor sees the other arm's draw through the BMS reading or gets its share in `rest_a`; two supervisors
that each assume the whole budget would allow twice the arm current.

Two limits (sw/safety.py Supervisor.check, after the per-joint torque bound). I_bat is monotone in |τ| at a given speed, so
each limit is the largest common factor x (bisection, fit_factor) that fits the budget with every motoring joint's |τ|
bounded by min(|τ|, x · ref); braking joints keep their command (they stay braking, their draw is unchanged):
1. **Peak (70 A):** ref = |τ| (x ≤ 1 scales the motoring joints' torque bounds). Trimmed the way the per-joint torque bound
   trims: feed-forward first, then kp, then kd. Counted as "current cap".
2. **Mean (28 A over 30 s), thermal-style derate:** PackBudget gives a ceiling on the pack current for the cycle (I_PEAK
   while the window has room; it falls towards I_AVG as the room is used up, and to I_AVG · I_AVG / mean if the mean is
   ever over, i.e. lower the further it is over). Over the ceiling, ref = rated: the per-joint torque limits become
   k · rated, the same k for every motoring joint, never below a floor: K_MIN · rated, and 1.25 × the twin's gravity
   torque when the supervisor has one (collision detector), so a derate never lets the arm fall into a tracking fault.
   Counted as "power derate".
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
V_NOM, BUCK_EFF = 24.0, 0.90                   # V bus when no BMS voltage; 24 → 12 V buck efficiency (ESTIMATE, rnd/electrical.py)
ETA = 0.85                                     # motor + drive efficiency, motoring and regenerating (ESTIMATE)
K_CU3 = 1.0                                    # copper-loss factor of a three-phase drive (ESTIMATE, 0.75–1.5 by R / current convention)
I_IDLE_DRIVE = 0.04                            # A per CAN drive, logic only (ESTIMATE, rnd/electrical.I_IDLE_DRIVER)
K_MIN = 0.25                                   # derate floor (fraction of rated torque): the arm keeps holding itself
BIN, H_BURST, PLAN_MARGIN = 0.1, 1.0, 0.1      # s window bins; s burst horizon of the plan; A kept under I_AVG by the plan

# actuator → (Kt at the actuator output N·m/A, R Ω per phase, k_cu, supply V, idle A at that supply, supply converter η).
# Kt: rnd/specs/actuators.yaml (sw.test_s1 checks it). R: ESTIMATEs (no figure in the yaml), range in the comment.
ACTUATORS = {
    "GIM6010-8": (5.0 / 10.5, 0.35, K_CU3, 24.0, I_IDLE_DRIVE, 1.0),         # 5 N·m @ 10.5 A; R 0.35 (0.25–0.50) as rnd/electrical
    "GIM8108-36": (3.86, 0.22, K_CU3, 24.0, I_IDLE_DRIVE, 1.0),              # SDC303 3.86 N·m/A; R 0.22 reseller tables (0.15–0.30)
    "DM-J4310-2EC_V1.1": (min(3.0 / 2.5, 7.0 / 7.5), 1.0, K_CU3, 24.0, I_IDLE_DRIVE, 1.0),   # 0.93 N·m/A; R 1.0 (0.5–1.5)
    "STS3215-C018": (2.94 / 2.7, 12.0 / 2.7, 1.0, 12.0, 0.18, BUCK_EFF),     # stall 2.94 N·m @ 2.7 A at 12 V → R = 4.4 Ω (3.5–5)
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
    """Per-joint battery-current model for one set of joints (rows of joints.json). Call at(v_bus) once per cycle (cached;
    sets idle, A)."""

    def __init__(self, joints, rated):
        try:
            a = [ACTUATORS[j["actuator"]] for j in joints]
        except KeyError as ex:
            raise KeyError(f"no current model for actuator {ex} (sw/power.ACTUATORS)") from None
        self.kt = np.array([x[0] * j["ratio"] for x, j in zip(a, joints)])       # N·m/A at the joint
        self.rk = np.array([x[1] * x[2] for x in a])                             # k_cu · R
        self.v_supply = np.array([x[3] for x in a])
        self.idle_supply = np.array([x[4] for x in a])
        self.eta_conv = np.array([x[5] for x in a])
        self.rated = np.asarray(rated, float)
        self._v = None
        self.at(None)

    def at(self, v_bus):
        v = V_NOM if v_bus is None or not math.isfinite(v_bus) or v_bus <= 0 else round(float(v_bus), 2)
        if v != self._v:
            self._v = v
            self.inv = 1.0 / (v * self.eta_conv)                                  # A per W at the bus
            self.idle = float(np.sum(self.idle_supply * self.v_supply * self.inv))
        return self

    def dynamic(self, tau, w):
        """Per-joint battery current above idle (A) for joint torques tau and measured speeds w."""
        cu = self.rk * (tau / self.kt) ** 2
        pm = tau * w
        p = np.where(pm >= 0, cu + pm / ETA, np.maximum(cu + pm * ETA, 0.0))
        return p * self.inv

    def bound(self, tabs, w):
        """Per-joint battery current (A) with |τ| = tabs driving along the motion (the worst sign); monotone in tabs."""
        return (self.rk * (tabs / self.kt) ** 2 + tabs * np.abs(w) / ETA) * self.inv

    def total(self, tau, w):
        """The set's battery current (A): idle + dynamic."""
        return self.idle + float(np.sum(self.dynamic(np.asarray(tau, float), np.asarray(w, float))))


def fit_factor(model, budget, tabs, ref, w, mask, lo=0.0, floor=0.0, iters=40):
    """Largest x in [lo, 1] with Σ_mask model.bound(min(tabs, max(x·ref, floor)), w) ≤ budget (lo if even that does not
    fit; the floor (N·m, per joint) is never cut)."""
    def f(x):
        return float(np.sum(np.where(mask, model.bound(np.minimum(tabs, np.maximum(x * ref, floor)), w), 0.0)))
    if f(1.0) <= budget:
        return 1.0
    if f(lo) > budget:
        return lo
    a, b = lo, 1.0
    for _ in range(iters):
        m = 0.5 * (a + b)
        a, b = (m, b) if f(m) <= budget else (a, m)
    return a


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
    Supervisor.check turns the ceiling into a common factor k on the per-joint torque limits (fit_factor)."""

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
