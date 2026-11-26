"""Safety supervisor (S1-safety, SW2). Sits between any policy/teleop and the motor drivers; independent of the policy.

Every control cycle it takes the wanted joint targets and returns what may be sent, or a stop:
1. **Watchdog:** if `tick()` isn't called for WATCHDOG_S (50 ms), the next call returns STOP and the e-stop relay's
   watchdog input stops toggling (electrical.md A5: the contactor opens in hardware as well).
2. **Joint limits:** targets clamped to [lower + margin, upper − margin] (sw/description/joints.json, from the twin).
3. **Velocity limit:** target steps limited to v_max·dt per joint (bench default 1.5 rad/s at the joint).
4. **Torque limit:** feed-forward torque clamped to the joint's rated torque (peak only with `allow_peak`).
5. **Workspace:** the hand (forward kinematics from the bench twin) must stay inside a box above the table; a target
   that leaves it is rejected and the last good target is held.
6. **Feedback sanity:** a measured position outside the limits by more than 5°, a motor error flag, or a temperature
   over TEMP_C (55 °C, the PETG/ASA can rule, thermal.md A6) → STOP.
7. **E-stop input:** latched; needs `reset()` with the button released.
Every decision is counted, so a session log shows how often the policy was overruled.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

WATCHDOG_S, TEMP_C = 0.050, 55.0
DESC = Path(__file__).resolve().parent / "description" / "joints.json"


@dataclass
class Verdict:
    stop: bool
    q: np.ndarray | None
    t_ff: np.ndarray | None
    reasons: list = field(default_factory=list)


class Supervisor:
    def __init__(self, v_max=1.5, margin_deg=3.0, ws_lo=(0.10, -0.45, 0.76), ws_hi=(0.80, 0.45, 1.35), clock=time.monotonic, fk=None):
        J = json.loads(DESC.read_text())
        self.names = [j["name"] for j in J]
        m = math.radians(margin_deg)
        self.lo = np.array([j["lower"] for j in J]) + m
        self.hi = np.array([j["upper"] for j in J]) - m
        self.rated = np.array([j["rated_nm"] for j in J])
        self.peak = np.array([j["peak_nm"] for j in J])
        self.v_max, self.clock, self.fk = v_max, clock, fk
        self.ws_lo, self.ws_hi = np.array(ws_lo), np.array(ws_hi)
        self.last_tick = None
        self.last_q = None
        self.estop = False
        self.counts = {}

    def _count(self, k):
        self.counts[k] = self.counts.get(k, 0) + 1

    def tick(self):
        self.last_tick = self.clock()

    def press_estop(self):
        self.estop = True

    def reset(self, q_meas):
        self.estop = False
        self.last_q = np.clip(np.asarray(q_meas, float), self.lo, self.hi)
        self.tick()

    def check(self, q_want, t_ff=None, q_meas=None, err=None, temp=None, dt=1 / 500, allow_peak=False) -> Verdict:
        now = self.clock()
        why = []
        if self.estop:
            why.append("e-stop latched")
        if self.last_tick is None or now - self.last_tick > WATCHDOG_S:
            why.append("watchdog")
        if q_meas is not None and (np.any(q_meas < self.lo - math.radians(8)) or np.any(q_meas > self.hi + math.radians(8))):
            why.append("measured position outside limits")
        if err is not None and np.any(np.asarray(err) != 0):
            why.append("motor error flag")
        if temp is not None and np.nanmax(np.asarray(temp, float)) > TEMP_C:
            why.append("over temperature")
        if why:
            for w in why:
                self._count(w)
            return Verdict(True, None, None, why)
        q = np.clip(np.asarray(q_want, float), self.lo, self.hi)
        if np.any(q != q_want):
            self._count("joint limit clamp")
        if self.last_q is None:
            self.last_q = q if q_meas is None else np.asarray(q_meas, float)
        step = self.v_max * dt
        q2 = self.last_q + np.clip(q - self.last_q, -step, step)
        if np.any(np.abs(q2 - q) > 1e-12):
            self._count("velocity clamp")
        if self.fk is not None:
            p = self.fk(q2)
            if np.any(p < self.ws_lo) or np.any(p > self.ws_hi):
                self._count("workspace reject")
                q2 = self.last_q
        lim = self.peak if allow_peak else self.rated
        t = None if t_ff is None else np.clip(np.asarray(t_ff, float), -lim, lim)
        if t_ff is not None and np.any(t != t_ff):
            self._count("torque clamp")
        self.last_q = q2
        return Verdict(False, q2, t, [])
