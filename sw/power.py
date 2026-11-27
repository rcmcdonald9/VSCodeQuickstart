"""Battery-current budget for the safety supervisor (W8a, docs/rnd/electrical.md *Pack and BMS*).

The pack (6S2P FPV pack) is rated 35 A continuous / 90 A burst, so the whole robot gets a software cap of **70 A peak** and
**28 A over a 30 s rolling mean** (rnd/electrical.SW_PACK). The supervisor (sw/safety.py) predicts every cycle what its
drives will draw and keeps the total under both, without ever stopping the arm (a power budget, not a fault).

Current model, per joint: I = |τ_joint| / (ratio · Kt) + idle, where τ_joint is the drive torque the verdict will produce
(kp·e − kd·ė + τ_ff at the measured state), ratio the belt/stage ratio outside the actuator (joints.json) and Kt the
actuator's torque constant at its output (actuators.yaml: rated torque / rated current, the lower of rated and peak where
both exist). This is the *phase* current; with the drive's output voltage below the bus voltage the bus current is at most
that (P_elec = I_ph · V_ph ≤ I_ph · V_bus), so the model is an upper bound (≈ 3× high when holding still, where only the
copper loss is drawn). A joint whose torque opposes its measured motion (τ·ω < 0, |ω| > W_REGEN) is braking: regen counts
as zero draw (not as a credit for the other joints). The STS3215 wrist roll runs from the elbow node's 12 V buck: its
12 V current is referred to the bus by 12 / (V_bus · η). Every drive also draws its logic idle current.
To this the supervisor adds a fixed allowance for the rest of the robot, REST_A: the sustained column of every non-arm
branch in electrical.md (base wheels 2.2 + lift 2.0 + 12 V servos 2.9 + 5 V compute/sensors 2.0 + AI reserve 0.8 = 9.9 A).
With a BMS reading, the rest of the robot is max(REST_A, measured pack current − this arm's model draw), so the base and
the lift keep priority when they really draw more, and the rolling mean uses the larger of model and measurement.

Two limits (sw/safety.py Supervisor.check):
1. **Peak (70 A):** if the predicted total exceeds I_PEAK, every motoring joint's torque bound is lowered to s·|τ| (the
   same s for all, so the arm's draw fits exactly) and the command is trimmed the way the per-joint torque bound trims it:
   feed-forward first, then kp, then kd. Counted as "current cap".
2. **Mean (28 A over 30 s), thermal-style derate:** above I_AVG the per-joint torque limit becomes k · rated (the peak
   allowance is dropped first), with k chosen so that the arm at its full derated limits plus the rest of the robot draws
   at most I_AVG · min(1, I_AVG / mean): the further the mean is over, the lower the limit (proportional). While derating,
   nothing above I_AVG enters the window, so the mean is back under I_AVG within one window and stays there under
   sustained demand. k falls at K_SLEW/s (no torque step), never below K_MIN (the arm keeps holding itself), and the
   derate releases only once the mean is under I_RELEASE (hysteresis), after which k recovers at K_RECOVER/s.
   Counted as "power derate" (cycles).
The window starts full of REST_A (the robot idles before the first command).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

I_PEAK, I_AVG, T_AVG = 70.0, 28.0, 30.0       # A, A, s (rnd/electrical.SW_PACK; electrical.md *Pack and BMS*)
REST_A = 2.2 + 2.0 + 2.9 + 2.0 + 0.8          # A: base + lift + 12 V servos + 5 V compute/sensors + AI reserve (electrical.md branch table, sustained)
V_MIN, BUCK_EFF = 20.0, 0.90                   # 6S cut-off (3.33 V/cell) when no BMS voltage; 24 → 12 V buck efficiency (ESTIMATE, rnd/electrical.py)
I_IDLE_DRIVE = 0.04                            # A per CAN drive, logic only (ESTIMATE, rnd/electrical.I_IDLE_DRIVER)
W_REGEN = 0.05                                 # rad/s: below this the sign of the measured speed is noise, so no regen credit
K_MIN, K_SLEW, K_RECOVER = 0.25, 1.0, 0.05     # derate floor (fraction of rated), fall rate and recovery rate (1/s)
I_RELEASE = 0.8 * I_AVG                        # A: the derate stays on until the mean is below this

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
    """Per-joint bus-current model for one set of joints (rows of joints.json)."""

    def __init__(self, joints):
        try:
            a = [ACTUATORS[j["actuator"]] for j in joints]
        except KeyError as ex:
            raise KeyError(f"no current model for actuator {ex} (sw/power.ACTUATORS)") from None
        self.kt = np.array([x[0] * j["ratio"] for x, j in zip(a, joints)])       # N·m/A at the joint
        self.v_supply = np.array([x[1] for x in a])
        self.idle_supply = np.array([x[2] for x in a])

    def _to_bus(self, v_bus):
        v = V_MIN if v_bus is None or not math.isfinite(v_bus) or v_bus <= 0 else float(v_bus)
        return np.where(self.v_supply < 20.0, self.v_supply / (v * BUCK_EFF), 1.0)   # buck-fed servos referred to the bus

    def idle(self, v_bus=None):
        return float(np.sum(self.idle_supply * self._to_bus(v_bus)))

    def dynamic(self, tau, w, v_bus=None):
        """Per-joint bus current above idle (A) for joint torques tau and measured speeds w; braking joints draw 0."""
        tau, w = np.asarray(tau, float), np.asarray(w, float)
        braking = (tau * w < 0) & (np.abs(w) > W_REGEN)
        return np.where(braking, 0.0, np.abs(tau) / self.kt) * self._to_bus(v_bus)

    def full(self, lim, v_bus=None):
        """Dynamic current with every joint at |lim| (the worst case under a torque limit)."""
        return float(np.sum(np.abs(lim) / self.kt * self._to_bus(v_bus)))


class PackBudget:
    """30 s rolling mean of the predicted (or measured) pack current and the derate state."""

    def __init__(self, rest_a=REST_A, i_peak=I_PEAK, i_avg=I_AVG, t_avg=T_AVG):
        self.rest_a, self.i_peak, self.i_avg, self.t_avg = float(rest_a), float(i_peak), float(i_avg), float(t_avg)
        self.buf = deque([[self.t_avg, self.rest_a]])          # [duration s, current A]; starts idle
        self.area = self.t_avg * self.rest_a
        self.k = 1.0
        self.derating = False
        self.t_last = None
        self.last_total = self.rest_a
        self.peak_mean = self.rest_a

    @property
    def mean(self):
        return self.area / self.t_avg

    def add(self, dt, i):
        if dt <= 0:
            return
        dt = min(dt, self.t_avg)
        self.buf.append([dt, i])
        self.area += dt * i
        excess = dt                                             # the window was full: drop `dt` from the oldest end
        while excess > 1e-12:
            d0, i0 = self.buf[0]
            if d0 <= excess:
                self.buf.popleft(); self.area -= d0 * i0; excess -= d0
            else:
                self.buf[0][0] = d0 - excess; self.area -= excess * i0; excess = 0.0
        if len(self.buf) == 1 or (len(self.buf) & 0x3FFF) == 0:   # refresh the running sum now and then (rounding)
            self.area = math.fsum(d * c for d, c in self.buf)
        self.peak_mean = max(self.peak_mean, self.mean)

    def record(self, now, total, fallback_dt):
        """Close the interval since the last record at the current total (A)."""
        dt = fallback_dt if self.t_last is None else now - self.t_last
        if not math.isfinite(dt) or dt < 0:
            dt = fallback_dt
        self.add(dt, float(total))
        self.t_last = now
        self.last_total = float(total)

    def update_derate(self, dt, rest, i_full, idle):
        """Derate factor k for this cycle (fraction of the rated torque), from the mean so far."""
        m = self.mean
        if m > self.i_avg:
            self.derating = True
        elif m < I_RELEASE:
            self.derating = False
        if self.derating:
            ceiling = self.i_avg * min(1.0, self.i_avg / max(m, 1e-9))
            target = (ceiling - rest - idle) / max(i_full, 1e-9)
            target = min(max(target, K_MIN), 1.0)
            self.k = max(target, self.k - K_SLEW * dt) if target < self.k else target     # down slewed, up at once
        else:
            self.k = min(1.0, self.k + K_RECOVER * dt)
        return self.k

    @property
    def active(self):
        return self.derating or self.k < 1.0
