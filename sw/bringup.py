"""M1 day-1 bring-up tool (S1).  python -m sw.bringup --bus virtual            (the MuJoCo bench twin, now)
                                python -m sw.bringup --bus socketcan:can0,can1 --estop gpiod:gpiochip0:17:27   (day 1)

Same code against a python-can virtual bus answered by sw/virtual_arm.py and against the real USB-CAN adapters. Steps, in
order; **each refuses to run unless the one before passed**, and every step that can move a joint runs inside the 500 Hz
loop with the safety supervisor (sw/safety.py) in it, the hardware heartbeat toggled only after a healthy cycle:

0. **GIM protocol discovery** (Fable review M4; separate tool, Step 1 of the M1 kit): `python -m sw.gim_discover` on the
   ONE Step-1 GIM6010-8 alone on the bus: listen, a no-torque MIT probe, a zero-gain motor-mode trace. It decides
   whether the GIM table in sw/drivers/gim.py can be filled at all (MIT / impedance mode or not) before any other GIM
   is bought; this tool then runs with that single GIM plus the DM-J4310 (`--joints yaw,wpitch` on the Step-1 rig).
1. **enumerate** — CAN map check (sw/can_map.py); every motor on the map answers on its command / feedback ID, firmware
   and hardware version, serial, control mode (must be MIT), status; a motor still at its factory ID is reported as such;
   the MIT frame ranges are read from the drive and the joint range × ratio must fit in P_MAX. Joints whose protocol
   table still has TODOs (GIM6010 until the SteadyWin manual is read: sw/drivers/gim.py) are BLOCKED and left out.
2. **limits** — the drive-side torque limit (= the supervisor's bound, rated by default: the hardware half of SW2) and
   the drive's CAN timeout are written, saved to flash and read back.
3. **safety** — e-stop and heartbeat chain: arm, healthy idle, e-stop press → supervisor latched, contactor open, drives
   unpowered; reset refused while the button is pressed; the drive limits survive the power cycle (flash); a hung loop
   (no heartbeat) opens the contactor within the watchdog time; the drives' own CAN timeout disables them.
4. **calibrate** — distal joints first, each against its reference hard stop at low current (calibration torque from
   the twin's gravity over the joint's range + margin, set in the drive): seek at 0.3 rad/s, detect the stall, check it
   is a stiff stop (not gravity or binding), zero = stop angle; optional span check against the opposite stop (catches a
   wrong belt ratio or stop position); back to the operating limit, park.
5. **jog** — each joint in turn, ± its jog amplitude at 10 % of its max speed, supervisor slewing at that speed; tracking
   error, peak speed, direction (the operator confirms which way it moved), torque; the momentum observer
   (sw/collision.py) runs in observe mode and records its peak residual.
6. **gravity** — gravity-compensation hold at the poses in bringup.yaml, each approached from both sides: measured joint
   torque (drive current) vs the twin's gravity torque (balancer included) → static residual (mean), Coulomb friction
   (half the difference), noise; balancer K fitted from the pitch residual; a drive-clamp sag test (limit set below the
   gravity load: the joint must sag, proving the drive enforces its limit); **collision thresholds** for sw/collision.py
   suggested from the residuals and the jog observer peaks.
7. **sysid** (real2sim, sw/sysid.py) — per joint, at low torque: constant-velocity sweeps (Coulomb + viscous friction),
   holds with and without a known test mass (effective torque constant), slow reversals at the zero-gravity angle
   (backlash), torque steps + a chirp (command → response latency) → sw/config/actuators_<serial>.yaml, which
   rnd/twin.py, sw/bench_env.py and sw/virtual_arm.py load instead of the datasheet model.
8. **report** — docs/sw/bringup_<date>[_virtual].md + .json (calibration offsets, limits, thresholds, the fitted actuator
   model, determinism checks: loop-period histogram, CPU temperature / throttling, PREEMPT_RT, CAN error counters; the GIM
   TODO table).

Not covered (no protocol yet): RP2040 joint-encoder nodes, the STS3215 wrist roll / gripper (TTL via the elbow node).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from sw.drivers import protocol as P
from sw.drivers.gim import GIM6010, GIM_TODO_HELP
from sw.drivers.mit import DM_J4310

ROOT = Path(__file__).resolve().parent
ARM = ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll"]
STEPS = ["enumerate", "limits", "safety", "calibrate", "jog", "gravity", "sysid"]
TABLES = {"dm_j4310": DM_J4310, "gim6010": GIM6010}
KD_LIMP = 0.5                    # N·m·s/rad: joints not yet calibrated hang limp with light damping
D2R = math.pi / 180


def load_cfg(path=None):
    return yaml.safe_load(Path(path or ROOT / "config" / "bringup.yaml").read_text())


def joints_json():
    return {j["name"]: j for j in json.loads((ROOT / "description" / "joints.json").read_text())}


class StepFail(Exception):
    pass


@dataclass
class Step:
    name: str
    status: str = "NOT RUN"       # PASS / FAIL / REFUSED / NOT RUN
    summary: str = ""
    checks: list = field(default_factory=list)     # (what, ok, detail)
    data: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)       # informational lines for the report

    def check(self, what, ok, detail=""):
        self.checks.append((what, bool(ok), detail))
        return bool(ok)


# ---------------------------------------------------------------------------------------------------------------------
# twin: the model torques the bring-up compares against (bench twin, gravity on, nominal balancer)
# ---------------------------------------------------------------------------------------------------------------------
def make_twin(cfg):
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from sw.bench_env import BenchEnv, xml
    from sw.collision import MujocoTwin
    from sw.virtual_arm import balancer_sign
    env = BenchEnv(render=False)
    m = mujoco.MjModel.from_xml_string(xml())
    m.body_gravcomp[:] = 0
    bal = cfg.get("balancer") or {}
    sgn = balancer_sign(m, env.qadr, env.dadr)
    kb, jb = float(bal.get("k_nm", 0.0)), ARM.index(bal.get("joint", "pitch"))

    class BalancedTwin(MujocoTwin):
        """MujocoTwin with the shoulder balancer as a known applied torque τ = K·sin(pitch)."""
        k_bal = kb

        def tau_bal(self, q):
            t = np.zeros(len(q))
            t[jb] = sgn * self.k_bal * math.sin(q[jb])
            return t

        def dyn(self, q, qd):
            M, b, tp = super().dyn(q, qd)
            return M, b - self.tau_bal(q), tp

        def gravity(self, q):
            return super().gravity(q) - self.tau_bal(q)

    return BalancedTwin(m, env.qadr, env.dadr)


# ---------------------------------------------------------------------------------------------------------------------
# clocks, safety IO, operator for the real bench
# ---------------------------------------------------------------------------------------------------------------------
class RealClock:
    def __init__(self, dt):
        self.dt, self.next, self.overruns = dt, None, 0

    def __call__(self):
        return time.monotonic()

    def wait(self):
        now = time.monotonic()
        if self.next is None or now > self.next + self.dt:
            if self.next is not None:
                self.overruns += 1
            self.next = now
        self.next += self.dt
        time.sleep(max(0.0, self.next - time.monotonic()))


class GpiodSafetyIO:
    """Pi GPIO (libgpiod v2 Python bindings): heartbeat output to the SN74LVC1G123 watchdog, contactor aux contact input
    (high = main contacts closed). `--estop gpiod:<chip>:<heartbeat line>:<aux line>`. Not testable here (no Pi)."""

    def __init__(self, chip, hb, aux):
        import gpiod
        from gpiod.line import Direction, Value
        self.V = Value
        self.hb, self.aux = int(hb), int(aux)
        self.req = gpiod.request_lines(f"/dev/{chip}", consumer="pip-bringup", config={
            self.hb: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
            self.aux: gpiod.LineSettings(direction=Direction.INPUT)})
        self.level = False

    def heartbeat(self):
        self.level = not self.level
        self.req.set_value(self.hb, self.V.ACTIVE if self.level else self.V.INACTIVE)

    def update(self):
        pass

    def estop_active(self):
        return self.req.get_value(self.aux) != self.V.ACTIVE


class ConsoleOperator:
    """The person at the bench (real hardware). Prompts are non-blocking so the control loop keeps cycling."""

    def __init__(self):
        self._pending = None

    def say(self, text):
        print(f"\n>>> {text}", flush=True)

    def confirm(self, text):
        return input(f"\n>>> {text} [y/N] ").strip().lower().startswith("y")

    def press_estop(self):
        self.say("PRESS the e-stop now (waiting up to 30 s)")

    def release_estop(self):
        self.say("RELEASE the e-stop (twist), then press ARM / reset")

    def arm_contactor(self):
        if self._pending != "arm":
            self.say("Press ARM / reset to close the contactor")
            self._pending = "arm"
        return False

    def watch_start(self, joint):
        self.say(f"Watch {joint}: it moves + first (the arrow on the part), then −. Type + or − when it has moved, then Enter")
        self._answer = None

    def watch_direction(self, joint):
        import select
        if self._answer is None and select.select([sys.stdin], [], [], 0)[0]:
            self._answer = sys.stdin.readline().strip()
        return None if self._answer is None else (1 if self._answer.startswith("+") else -1)

    def hang_test_mass(self, kg):
        """Non-blocking (the loop keeps holding the joints): the operator hangs / removes the weight, then presses Enter."""
        self.say(f"{'HANG the ' + str(kg) + ' kg test weight at the lever end / grasp point' if kg else 'REMOVE the test weight'}, "
                 "hands clear, then press Enter")
        self._mass_done = False

    def test_mass_ready(self):
        import select
        if not getattr(self, "_mass_done", True) and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            self._mass_done = True
        return getattr(self, "_mass_done", True)


# ---------------------------------------------------------------------------------------------------------------------
# the rig: drivers + loop
# ---------------------------------------------------------------------------------------------------------------------
@dataclass
class Fb:
    q: np.ndarray
    v: np.ndarray
    t: np.ndarray
    temp: np.ndarray
    err: np.ndarray
    status: list
    age: float | None


class BringUp:
    def __init__(self, buses, can_map=None, cfg=None, tables=None, io=None, operator=None, clock=None, plant=None,
                 joints=None, limit=None, reg_timeout=0.05, log=print, actuator_dir=None):
        from sw.can_map import load
        self.cfg = cfg or load_cfg()
        self.can_map = can_map or load()
        self.tables = tables or TABLES
        self.buses, self.io, self.op, self.plant = buses, io, operator, plant
        self.dt = 1.0 / self.cfg["control_hz"]
        self.clock = clock or RealClock(self.dt)
        self.want_joints = joints
        self.limit_kind = limit or self.cfg.get("limit", "rated")
        self.reg_timeout = reg_timeout
        self.log = log
        # where the sysid step writes actuators_<serial>.yaml: sw/config on the bench (loaded by the sims from then on);
        # a virtual rehearsal writes next to its outputs so it never replaces a real fit
        self.actuator_dir = Path(actuator_dir) if actuator_dir else (ROOT / "config" if plant is None else ROOT / "out" / "actuators_virtual")
        self.J = joints_json()
        self.steps = {s: Step(s) for s in STEPS}
        self.drivers = {}             # online joints, ARM order
        self.blocked = {}
        self.calibrated = set()
        self.hold_q = {}              # joint → hold target once calibrated
        self.hold_v, self.hold_a = {}, {}
        self.twin = make_twin(self.cfg)
        self.n_cycles = 0
        self.t_start = time.time()
        self.saved = {}
        self._sup = None              # the supervisor in charge of the loop (register waits keep cycling it: _reg_idle)
        self.t_hb = None
        self.hb_gap_max = 0.0
        # Configuration keep-alive: while every drive is disabled (enumerate, limits, between steps) the host keeps the
        # contactor closed so the drives stay powered for register access; no ENABLE or control frame is sent then. Motion
        # always runs with the supervisor-gated heartbeat (start() switches the keep-alive off).
        self.keepalive = False
        if self.plant is None and self.io is not None:
            import threading
            threading.Thread(target=self._keepalive_thread, daemon=True).start()

    def _keepalive_thread(self):
        while True:
            if self.keepalive:
                self.hb()
            time.sleep(self.dt)

    def hb(self):
        now = self.now()
        if self.t_hb is not None and now - self.t_hb > self.hb_gap_max:
            self.hb_gap_max = now - self.t_hb          # longest heartbeat gap outside the deliberate hang test (watchdog sizing)
        self.t_hb = now
        self.io.heartbeat()

    # ---- helpers
    @property
    def names(self):
        return [j for j in ARM if j in self.drivers]

    def jc(self, j):
        return self.cfg["joints"][j]

    def now(self):
        return self.clock()

    def wait(self):
        """End of one control period."""
        if self.keepalive and self.io is not None:
            self.hb()
        if self.plant is not None:
            self.plant.step(self.dt)
        else:
            self.clock.wait()

    def full_q(self, q_online, guess=None):
        """Six joint angles for the twin: online joints measured (uncalibrated ones from `guess`), others at the fixture."""
        fx = self.cfg["fixture_pose"]
        rest = self.cfg["rest_pose"]
        q = np.array([fx[j] * D2R for j in ARM])
        for k, j in enumerate(self.names):
            if j in self.calibrated:
                q[ARM.index(j)] = q_online[k]
            else:
                q[ARM.index(j)] = (guess or {}).get(j, rest[j] * D2R)
        return q

    def gravity_online(self, q_online, guess=None):
        g = self.twin.gravity(self.full_q(q_online, guess))
        return np.array([g[ARM.index(j)] for j in self.names])

    def lim(self, j):
        return self.J[j]["rated_nm"] if self.limit_kind == "rated" else self.J[j]["peak_nm"]

    def feedback(self):
        now = self.now()
        st = [self.drivers[j].state for j in self.names]
        stamps = [self.drivers[j].stamp for j in self.names]
        age = None if any(s is None for s in stamps) else max(now - s for s in stamps)
        return Fb(np.array([s["p"] for s in st]), np.array([s["v"] for s in st]), np.array([s["t"] for s in st]),
                  np.array([s["temp"] if s["temp"] is not None else np.nan for s in st], float),
                  np.array([s["err"] for s in st]), [s.get("status") for s in st], age)

    def poll_all(self):
        for d in self.drivers.values():
            d.poll()

    def refresh_cycle(self, heartbeat=False):
        """An idle period: status requests only (no torque), optional unconditional heartbeat (arming window only)."""
        for d in self.drivers.values():
            d.refresh()
        if heartbeat:
            self.hb()
        self.wait()
        self.poll_all()

    def supervisor(self, limits=None, torque=None, v_max=None):
        from sw.safety import Supervisor
        vm = v_max if v_max is not None else np.array([self.cfg["jog_speed_frac"] * self.J[j]["max_speed_rad_s"] for j in self.names])
        from sw.safety import FB_MAX_AGE
        return Supervisor(v_max=vm, clock=self.clock, joints=self.names, limits=limits, torque=torque,
                          fb_max_age=float(self.cfg.get("fb_max_age_s", FB_MAX_AGE)))

    def cycle(self, sup, q=None, kp=None, kd=None, tff=None, idle=False):
        """One control period with the supervisor in the loop. idle: drives stay disabled (status requests only)."""
        if self.io.estop_active():
            if not sup.estop_pressed:
                sup.press_estop()
        elif sup.estop_pressed:
            sup.release_estop()
        fb = self.feedback()
        n = len(self.names)
        sup.tick()
        qw = fb.q.copy() if q is None else np.asarray(q, float)
        v = sup.check(qw, np.zeros(n) if kp is None else kp, np.zeros(n) if kd is None else kd, tff, fb.q, fb.v, fb.age,
                      err=fb.err, temp=fb.temp, dt=self.dt)
        for k, j in enumerate(self.names):
            d = self.drivers[j]
            if idle:
                d.refresh()
            elif v.stop:
                d.disable()
            else:
                d.command(v.q[k], 0.0, v.kp[k], v.kd[k], v.t_ff[k])
        if sup.heartbeat_ok():
            self.hb()
        self.wait()
        self.poll_all()
        self.n_cycles += 1
        return v, self.feedback()

    def run(self, sup, n, fn):
        """n cycles; fn(k, fb) → (q, kp, kd, tff) or None to stop early. Raises StepFail on a supervisor stop."""
        fb = self.feedback()
        out = []
        for k in range(n):
            r = fn(k, fb)
            if r is None:
                break
            v, fb = self.cycle(sup, *r)
            if v.stop:
                raise StepFail(f"supervisor stop: {v.reasons}")
            out.append(fb)
        return out

    # ---- arming / enabling
    def arm(self, timeout=None):
        """Close the contactor (operator arm/reset) inside a short arming window, then wait for every drive to answer."""
        timeout = timeout or (1.0 if self.plant is not None else 30.0)
        t_end = self.now() + timeout
        was_open = self.io.estop_active()
        while self.io.estop_active():
            self.hb()
            self.op.arm_contactor()
            self.wait()
            if self.now() > t_end:
                raise StepFail("contactor did not close (arm/reset)")
        t_end = self.now() + 0.5                               # drives boot (disabled): arming window, no torque possible
        while True:
            self.refresh_cycle(heartbeat=True)
            if all(d.state.get("fresh") for d in self.drivers.values()):
                break
            if self.now() > t_end:
                raise StepFail("drives silent after power-up")
        if was_open and self.calibrated:
            lost = [j for j in self.calibrated if not self.drivers[j].table.get("ABSOLUTE", True)]
            if lost:
                self.calibrated -= set(lost)
                raise StepFail(f"power cycle lost the zero of {lost} (no absolute output encoder): re-run calibrate")

    def any_enabled(self):
        return any(d.state.get("status") == d.table["STATUS_ENABLED"] for d in self.drivers.values())

    def start(self, sup):
        """Hand the loop to a (new) supervisor. Drives disabled: status requests until the feedback is fresh, reset, a few
        healthy idle cycles. Drives enabled (switching supervisors mid-step): reset on the last cycle's feedback, no gap."""
        enabled = self.any_enabled()
        if not enabled:
            for i in range(25):                             # all drives disabled: configuration keep-alive; on a late bus
                self.refresh_cycle(heartbeat=True)          # wait (<= 50 ms) until every drive's feedback is fresh
                fb = self.feedback()
                if i >= 2 and fb.age is not None and fb.age <= sup.fb_max_age and np.all(np.isfinite(fb.q)):
                    break
        fb = self.feedback()
        self._sup = sup
        if not sup.reset(fb.q, fb.age):
            raise StepFail(f"supervisor reset refused (feedback {fb.q}, age {fb.age}, latched {sup.latched})")
        self.keepalive = False
        if not enabled:
            for _ in range(5):
                v, fb = self.cycle(sup, idle=True)
                if v.stop:
                    raise StepFail(f"supervisor stop while idle: {v.reasons}")
        return fb

    def enable_all(self, sup):
        for d in self.drivers.values():
            d.enable()
        for i in range(25):                                 # the status reply can be a few cycles late on a real bus
            v, fb = self.cycle(sup, idle=True)
            bad = [j for j in self.names if self.drivers[j].state.get("status") != self.drivers[j].table["STATUS_ENABLED"]]
            if not bad or v.stop:
                break
            if i % 8 == 7:
                for j in bad:
                    self.drivers[j].enable()
        if bad:
            raise StepFail(f"did not enable: {bad} (status {[self.drivers[j].state.get('status') for j in bad]})")
        return fb

    def disable_all(self):
        for d in self.drivers.values():
            d.disable()
        self.keepalive = True
        self._sup = None
        self.wait()
        self.poll_all()

    def _reg_idle(self):
        """One control cycle while a register reply is awaited: the keep-alive (all drives disabled) or the supervisor's
        loop (hold commands for enabled drives, status requests otherwise), so the heartbeat never pauses for the bus."""
        sup = self._sup
        if self.keepalive or sup is None or self.io is None:
            self.wait()
            return
        if self.any_enabled():
            self.cycle(sup, *self._cmd_hold(self.feedback()))
        else:
            self.cycle(sup, idle=True)

    def await_fresh(self, drivers=None, timeout=0.04, hb=False):
        """Status requests until every driver in `drivers` has answered (or timeout, bus clock). -> {name: state}."""
        ds = list(drivers if drivers is not None else self.drivers.values())
        for d in ds:                       # discard frames buffered before the request: after a pause the newest queued
            d._take()                      # frame is old (bus-realism finding: a stale "enabled" passed as the answer)
        got = {}
        t_end = self.now() + timeout
        i = 0
        while True:
            if i % 5 == 0:
                for d in ds:
                    if d.name not in got:
                        d.refresh()
            if hb:
                self.hb()
            self.wait()
            for d in ds:
                st = d.poll()
                if st.get("fresh"):
                    got[d.name] = dict(st)
            i += 1
            if len(got) == len(ds) or self.now() >= t_end:
                return {d.name: got.get(d.name, dict(d.state, fresh=False)) for d in ds}

    def clear_errors(self, timeout=0.06):
        """CLEAR_ERROR + DISABLE to every drive, then status requests until each reports an OK status: on a late bus the
        next step must not start on a reply that left the drive before the clear arrived (bus-realism finding)."""
        t_end = self.now() + timeout
        while True:
            for d in self.drivers.values():
                d.clear_error()
                d.disable()
            fr = self.await_fresh(hb=True, timeout=0.02)
            if all(fr[j].get("fresh") and fr[j].get("status") in self.drivers[j].table["STATUS_OK"] for j in self.names):
                return True
            if self.now() >= t_end:
                return False

    def gains(self):
        return (np.array([self.jc(j)["kp"] for j in self.names], float), np.array([self.jc(j)["kd"] for j in self.names], float))

    # =================================================================================================================
    # steps
    # =================================================================================================================
    def require(self, name):
        i = STEPS.index(name)
        if i > 0 and self.steps[STEPS[i - 1]].status != "PASS":
            s = self.steps[name]
            s.status, s.summary = "REFUSED", f"previous step '{STEPS[i - 1]}' did not pass ({self.steps[STEPS[i - 1]].status})"
            return False
        return True

    def step_enumerate(self):
        from sw.can_map import check
        s = self.steps["enumerate"]
        errs, loads = check(self.can_map)
        s.check("CAN map valid (IDs, terminations, ratios, bus load ≤ 50 %)", not errs, "; ".join(errs) or
                ", ".join(f"{k} {v:.0%}" for k, v in loads.items()))
        rows, others = [], []
        if self.io is not None:                                  # the drives are powered through the contactor
            self.keepalive = True
            t_end = self.now() + (1.0 if self.plant is not None else 30.0)
            while self.io.estop_active() and self.now() < t_end:
                self.hb()
                self.op.arm_contactor()
                self.wait()
            for _ in range(int(0.3 / self.dt)):                  # drives boot
                self.wait()
            s.check("contactor closed (drives powered, all disabled)", not self.io.estop_active())
        for bn, b in self.can_map["buses"].items():
            for dv in b["devices"]:
                if dv["kind"] not in self.tables:
                    others.append(f"{dv['name']} ({dv['kind']}): not handled by this tool (no protocol yet)")
                    continue
                if self.want_joints and dv["name"] not in self.want_joints:
                    continue
                tbl = self.tables[dv["kind"]]
                miss = P.missing(tbl)
                row = dict(joint=dv["name"], kind=dv["kind"], bus=bn, id=dv["id"], fb_id=dv["fb_id"], ratio=dv["ratio"])
                if miss:
                    row.update(status="BLOCKED", why=f"protocol table TODO: {', '.join(miss)}")
                    self.blocked[dv["name"]] = miss
                    rows.append(row)
                    continue
                jc = self.jc(dv["name"])
                d = P.TableDriver(self.buses[bn], dv["name"], dv["id"], dv["fb_id"], dv["ratio"], tbl, sign=jc.get("sign", 1))
                d.idle = self._reg_idle
                info = {r: d.read_reg(r, self.reg_timeout) for r in ("esc_id", "mst_id", "ctrl_mode", "fw_version", "hw_version", "serial")
                        if r in tbl["REG"]}
                if info.get("esc_id") is None:
                    row.update(status="FAIL", why="no answer on its command/feedback ID")
                    dflt = tbl.get("DEFAULT_ID")
                    used = {x["id"] for x in b["devices"]}
                    if dflt and dflt[0] not in used:
                        probe = P.TableDriver(self.buses[bn], dv["name"], dflt[0], dflt[1], dv["ratio"], tbl)
                        probe.idle = self._reg_idle
                        if probe.read_reg("esc_id", self.reg_timeout) is not None:
                            row["why"] = (f"a motor answers at the FACTORY ID 0x{dflt[0]:02X}/0x{dflt[1]:02X}: set ESC_ID 0x{dv['id']:02X}, "
                                          f"MST_ID 0x{dv['fb_id']:02X} with the vendor tool")
                    rows.append(row)
                    continue
                st = self.await_fresh([d])[d.name]
                ok_ids = info["esc_id"] == dv["id"] and info["mst_id"] == dv["fb_id"]
                ok_mode = info.get("ctrl_mode") == tbl["CTRL_MODE_MIT"]
                ok_fb = st.get("fresh") and st.get("status") in tbl["STATUS_OK"]
                fw_ok = True
                allowed = (self.cfg.get("expected_fw") or {}).get(dv["kind"])
                if allowed:
                    fw_ok = info.get("fw_version") in allowed
                rg = d.ranges_from_drive() or {}
                span = max(abs(self.jc(dv["name"])["stop"][1]), abs(self.jc(dv["name"])["other"])) * D2R + 0.1
                ok_range = span * dv["ratio"] < d.r.p_max
                fails = [w for w, ok in (("IDs differ from the CAN map", ok_ids), ("control mode is not MIT", ok_mode),
                                         ("no status reply or drive fault", ok_fb), ("firmware not in expected_fw", fw_ok),
                                         (f"joint range × ratio {span * dv['ratio']:.1f} rad > P_MAX {d.r.p_max}: raise P_MAX", ok_range)) if not ok]
                row.update(status="PASS" if not fails else "FAIL", why="; ".join(fails), fw=info.get("fw_version"), hw=info.get("hw_version"),
                           serial=info.get("serial"), ranges=rg, status_code=st.get("status"),
                           temp=st.get("temp"))
                rows.append(row)
                if not fails:
                    self.drivers[dv["name"]] = d
        s.data = dict(devices=rows, not_handled=others, blocked={k: v for k, v in self.blocked.items()})
        for r in rows:
            what = f"{r['joint']} ({r['kind']}, {r['bus']} 0x{r['id']:02X}/0x{r['fb_id']:02X})"
            if r["status"] == "BLOCKED" and not self.want_joints:
                s.notes.append(f"⛔ {what} BLOCKED (left out, held in the fixture): {r['why']}. Protocol discovery first: "
                               f"python -m sw.gim_discover --bus <adapter> --id 0x{r['id']:02X} (docs/sw/gim_protocol_notes.md)")
                continue
            s.check(what, r["status"] == "PASS", f"{r['status']}: {r.get('why', '')}".strip(": ")
                    + (f" (fw 0x{r['fw']:04X}, hw 0x{r['hw']:04X}, serial {r['serial']})" if r.get("fw") is not None else ""))
        s.notes += [f"ℹ️ {o}" for o in others]
        s.check("at least one joint online", bool(self.drivers), ", ".join(self.names))
        s.summary = f"online: {', '.join(self.names) or '—'}; blocked: {', '.join(self.blocked) or '—'}"

    def step_limits(self):
        s = self.steps["limits"]
        rows = []
        for j in self.names:
            d = self.drivers[j]
            t = d.table
            d.disable()
            self.wait()
            lim = self.lim(j)
            raw = d.torque_limit_to_drive(lim)
            tmo = int(round(self.cfg["can_timeout_s"] / t["CAN_TIMEOUT_UNIT_S"]))
            e1 = d.write_reg(t["TORQUE_LIMIT_REG"], raw, self.reg_timeout)
            e2 = d.write_reg("can_timeout", tmo, self.reg_timeout)
            d.save()
            self.wait()
            rb = d.read_reg(t["TORQUE_LIMIT_REG"], self.reg_timeout)
            rt = d.read_reg("can_timeout", self.reg_timeout)
            d.ranges_from_drive()
            ok = rb is not None and abs(rb - raw) <= 1e-4 * max(1.0, abs(raw)) and rt == tmo and e1 is not None and e2 is not None
            rows.append(dict(joint=j, limit_joint_nm=round(lim, 3), drive_reg=t["TORQUE_LIMIT_REG"], written=round(raw, 4),
                             readback=None if rb is None else round(rb, 4), can_timeout_counts=tmo, can_timeout_readback=rt,
                             feedback_t_max=d.r.t_max))
            s.check(f"{j}: torque limit {lim:.2f} N·m at the joint = {raw:.3f} {t['TORQUE_LIMIT_UNIT']} in the drive, "
                    f"CAN timeout {self.cfg['can_timeout_s'] * 1000:.0f} ms", ok, f"read back {rb}, {rt}")
        self.saved = {r["joint"]: (r["written"], r["can_timeout_counts"]) for r in rows}
        s.data = dict(limits=rows, kind=self.limit_kind)
        s.summary = f"{len(rows)} drive(s): {self.limit_kind} torque limit + {self.cfg['can_timeout_s'] * 1000:.0f} ms CAN timeout, saved and read back"

    def step_safety(self):
        s = self.steps["safety"]
        if self.io is None:
            s.check("e-stop / heartbeat IO present", False, "no --estop backend: every motion step is refused")
            return
        sim = self.plant is not None
        self.arm()
        sup = self.supervisor(limits=self._raw_limits())       # not calibrated yet: travel bounds around here
        self.start(sup)
        hb = 0
        for _ in range(100):
            v, fb = self.cycle(sup, idle=True)
            hb += sup.heartbeat_ok()
        s.check("healthy idle: 100 cycles, heartbeat every cycle, contactor closed", hb == 100 and not self.io.estop_active(), f"{hb}/100")
        # e-stop
        t0 = self.now()
        self.op.press_estop()
        t_aux = t_dead = None
        t_end = t0 + (0.5 if sim else 30.0)
        silent = 0
        while self.now() < t_end:
            v, fb = self.cycle(sup, idle=True)
            if t_aux is None and self.io.estop_active():
                t_aux = self.now()
            silent = silent + 1 if not any(d.state.get("fresh") for d in self.drivers.values()) else 0
            if t_aux is not None and silent >= 3 and t_dead is None:
                t_dead = self.now() - 2 * self.dt
            if t_dead is not None and self.now() > t_dead + 0.02:
                break
        s.check("e-stop: contactor opens (aux contact)", t_aux is not None, "" if t_aux is None else
                (f"{(t_aux - t0) * 1000:.0f} ms after the press" if sim else "seen"))
        s.check("e-stop: supervisor latched, no heartbeat", "e-stop" in sup.latched and not sup.heartbeat_ok(), str(sup.latched))
        s.check("e-stop: drives unpowered (no CAN replies)", t_dead is not None,
                "" if t_dead is None or t_aux is None else f"{(t_dead - t_aux) * 1000:.0f} ms after the aux contact")
        if sim and t_aux is not None:
            s.check("e-stop → contactor open within the SW2 budget (50 ms)", t_aux - t0 <= 0.05, f"{(t_aux - t0) * 1000:.0f} ms")
        s.data["estop_ms"] = None if t_aux is None else round((t_aux - t0) * 1000, 1)
        if t_aux is None:
            s.summary = "e-stop did not open the contactor: STOP, fix the stop circuit"
            return
        fb = self.feedback()
        s.check("reset refused while the button is pressed", not sup.reset(fb.q, 0.0))
        self.op.release_estop()
        self.arm()
        sup.release_estop()
        # The e-stop cut the drives' power: an uncalibrated joint whose drive has no absolute output encoder now reads a
        # new arbitrary angle (on the inline 1:1 roll that is up to ±1.5 rad at the joint, vs ±0.34 rad behind the old
        # 4.44:1 belt), so the travel window built before the press is re-anchored on the fresh reading. Same supervisor:
        # the point of this check is that its e-stop latch clears only through reset().
        self.refresh_cycle(heartbeat=True)
        sup.set_limits(self._raw_limits())
        self.start(sup)
        # limits survived the power cycle (flash)
        okp = []
        for j in self.names:
            d = self.drivers[j]
            okp.append((d.read_reg(d.table["TORQUE_LIMIT_REG"], self.reg_timeout), d.read_reg("can_timeout", self.reg_timeout)))
        ok = all(rb is not None and abs(rb - self.saved[j][0]) <= 1e-4 * max(1, abs(rb)) and rt == self.saved[j][1]
                 for j, (rb, rt) in zip(self.names, okp))
        s.check("drive limits survive the power cycle (saved to flash)", ok, str(dict(zip(self.names, okp))))
        for d in self.drivers.values():
            d.ranges_from_drive()
        # hung loop → watchdog
        wd = float(self.cfg.get("watchdog_s", 0.025))
        t_h = self.t_hb                                       # last heartbeat edge
        hang = 2.5 * wd
        if sim:
            self.plant.idle(hang)
        else:
            time.sleep(hang)
        opened = self.io.estop_active()
        t_wd = getattr(self.io, "t_open", None)
        s.check(f"hung loop (no heartbeat {hang * 1000:.0f} ms) → watchdog opens the contactor", opened,
                f"{(t_wd - t_h) * 1000:.1f} ms" if (opened and t_wd is not None) else "")
        if sim and opened and t_wd is not None:
            s.check(f"watchdog: contactor open ≤ {wd * 1.2 * 1000:.0f} ms after the last heartbeat ({wd * 1000:.0f} ms monostable + 20 %)",
                    t_wd - t_h <= wd * 1.2, f"{(t_wd - t_h) * 1000:.1f} ms")
        s.data["watchdog_ms"] = None if not (opened and t_wd is not None) else round((t_wd - t_h) * 1000, 1)
        if not opened:
            s.summary = "the hardware watchdog did not open the contactor: STOP, fix it"
            return
        self.t_hb = None                                      # the hang was deliberate: not a jitter sample
        self.arm()
        sup = self.supervisor(limits=self._raw_limits())
        self.start(sup)
        # drive CAN timeout: enabled with zero torque, then CAN silent (heartbeat kept: the drives' own watchdog is tested)
        self.enable_all(sup)
        n = len(self.names)
        self.run(sup, 50, lambda k, fb: (fb.q, np.zeros(n), np.zeros(n), np.zeros(n)))
        t_s = self.now()
        while self.now() < t_s + 2.5 * self.cfg["can_timeout_s"]:
            self.hb()
            self.wait()
        fr = self.await_fresh(hb=True)
        stat = {j: fr[j].get("status") for j in self.names}
        ok = all(stat[j] in (self.drivers[j].table.get("STATUS_TIMEOUT"), self.drivers[j].table["STATUS_DISABLED"]) for j in self.names)
        s.check(f"drive CAN timeout: drives disable themselves {self.cfg['can_timeout_s'] * 1000:.0f} ms after the last command", ok, str(stat))
        self.clear_errors()
        if self.io.estop_active():
            self.arm()
        s.summary = (f"e-stop → contactor {s.data.get('estop_ms')} ms, watchdog {s.data.get('watchdog_ms')} ms, "
                     f"drive CAN timeout {'OK' if ok else 'FAILED'}, limits kept over a power cycle")

    # ---- calibration
    def _gmax(self, j, q_online):
        """Largest gravity torque at joint j over the travel it will make: from its rest-pose guess (± 15°, the angle is
        not known before calibration) to the reference stop, and over the whole range if the span is checked."""
        k6 = ARM.index(j)
        c = self.jc(j)
        q_r = self.cfg["rest_pose"][j] * D2R
        pts = [q_r - 15 * D2R, q_r + 15 * D2R, c["stop"][1] * D2R] + ([c["other"] * D2R] if c.get("span_check") else [])
        a, b = min(pts), max(pts)
        q6 = self.full_q(q_online)
        g = 0.0
        for x in np.linspace(a, b, 25):
            q6[k6] = x
            g = max(g, abs(self.twin.gravity(q6)[k6]))
        return g

    def _cmd_hold(self, fb, active=None):
        """Targets/gains: calibrated joints track their target (hold_q, hold_v, hold_a) with computed-torque feed-forward
        from the twin (M·q̈ + Coriolis + gravity − balancer at the target) plus kd·q̇_target (the MIT frame's v_des = 0, so the
        damping term is fed forward through τ_ff, which the supervisor bounds); the others hang limp, except `active`."""
        KP, KD = self.gains()
        n = len(self.names)
        q, kp, kd = fb.q.copy(), np.zeros(n), np.full(n, KD_LIMP)
        tff = np.zeros(n)
        qt = fb.q.copy()
        vt, at = np.zeros(n), np.zeros(n)
        for k, j in enumerate(self.names):
            if j in self.calibrated and j != active:
                qt[k], vt[k], at[k] = self.hold_q[j], self.hold_v.get(j, 0.0), self.hold_a.get(j, 0.0)
        q6 = self.full_q(qt)
        v6, a6 = np.zeros(6), np.zeros(6)
        for k, j in enumerate(self.names):
            v6[ARM.index(j)], a6[ARM.index(j)] = vt[k], at[k]
        ff = self.twin.inverse(q6, v6, a6) if (np.any(v6) or np.any(a6)) else self.twin.gravity(q6)
        for k, j in enumerate(self.names):
            if j in self.calibrated and j != active:
                q[k], kp[k], kd[k], tff[k] = qt[k], KP[k], KD[k], ff[ARM.index(j)] + KD[k] * vt[k]
        return q, kp, kd, tff

    def _seek(self, sup, j, side, t_cal, kp_cal, travel):
        k = self.names.index(j)
        fb = self.feedback()
        tgt = fb.q[k]
        speed = self.cfg["cal_speed"]
        stall = 0
        n_max = int((travel + 0.35) / speed / self.dt)
        for _ in range(n_max):
            q, kp, kd, tff = self._cmd_hold(fb, active=j)
            push = side * kp_cal * (tgt - fb.q[k])
            if push < t_cal:
                tgt += side * speed * self.dt
            q[k], kp[k], kd[k], tff[k] = tgt, kp_cal, self.jc(j)["kd"] * 0.5, 0.0
            v, fb = self.cycle(sup, q, kp, kd, tff)
            if v.stop:
                raise StepFail(f"{j}: supervisor stop while seeking the stop: {v.reasons}")
            push = side * kp_cal * (tgt - fb.q[k])
            stall = stall + 1 if (push >= 0.85 * t_cal and abs(fb.v[k]) < 0.03) else 0
            if stall >= int(0.1 / self.dt):
                return fb.q[k], tgt
        raise StepFail(f"{j}: no hard stop within {math.degrees(travel):.0f}° of travel (belt slipping? stop missing?)")

    def _hold_at(self, sup, j, q_j, kp_j, secs):
        k = self.names.index(j)
        out = self.run(sup, int(secs / self.dt), lambda i, fb: _set(self._cmd_hold(fb, active=j), k, q_j, kp_j, self.jc(j)["kd"] * 0.5, 0.0))
        tail = out[len(out) // 2:]
        return float(np.mean([f.q[k] for f in tail])), float(np.mean([f.t[k] for f in tail]))

    def _move(self, sup, targets, speed=None, settle=0.3, each=None):
        """Move calibrated joints to `targets` (dict) on a cosine (smooth) profile whose peak speed is `speed` (rad/s,
        default 95 % of the 10 % jog speed), then hold `settle` s. each(fb, verdict) is called every cycle."""
        fb = self.feedback()
        start = {j: self.hold_q.get(j, fb.q[self.names.index(j)]) for j in targets}
        sp = {j: speed or self.cfg["jog_speed_frac"] * self.J[j]["max_speed_rad_s"] * 0.95 for j in targets}
        T = max([math.pi * abs(targets[j] - start[j]) / (2 * sp[j]) for j in targets] + [self.dt])
        n = int(math.ceil(T / self.dt))
        for i in range(n + int(settle / self.dt)):
            t = min((i + 1) * self.dt, T)
            moving = (i + 1) * self.dt < T
            for j in targets:
                D = targets[j] - start[j]
                self.hold_q[j] = start[j] + D * (1 - math.cos(math.pi * t / T)) / 2
                self.hold_v[j] = D * math.pi / (2 * T) * math.sin(math.pi * t / T) if moving else 0.0
                self.hold_a[j] = D * math.pi ** 2 / (2 * T * T) * math.cos(math.pi * t / T) if moving else 0.0
            v, fb = self.cycle(sup, *self._cmd_hold(self.feedback()))
            if v.stop:
                raise StepFail(f"supervisor stop: {v.reasons}")
            if each is not None:
                each(fb, v)

    def step_calibrate(self):
        s = self.steps["calibrate"]
        if self.io is None or self.io.estop_active():
            self.arm()
        rows = []
        for j in [x for x in self.cfg["order"] if x in self.drivers]:
            d, c = self.drivers[j], self.jc(j)
            side, th_stop = int(c["stop"][0]), c["stop"][1] * D2R
            th_other = c["other"] * D2R
            span = abs(th_stop - th_other)
            rated = self.J[j]["rated_nm"]
            # 1. calibration torque from the twin's gravity (others where they are now; this joint anywhere in its range)
            sup0 = self.supervisor(limits=self._raw_limits(span))
            self.start(sup0)
            fb = self.feedback()
            gmax = self._gmax(j, fb.q)
            t_cal = max(0.15 * rated, 1.4 * gmax + 0.10 * rated)
            row = dict(joint=j, g_max_nm=round(gmax, 3), t_cal_nm=round(t_cal, 3))
            if t_cal > self.cfg["cal_frac_max"] * rated:
                rows.append(row)
                s.check(f"{j}: calibration torque {t_cal:.2f} N·m ≤ {self.cfg['cal_frac_max']:.0%} of rated", False,
                        "gravity over the range is too large at low current: support the arm or choose the other stop")
                break
            self._set_limit(d, t_cal)
            # 2. seek the reference stop with the supervisor bounding torque at 1.5 × t_cal (the drive clamps at t_cal)
            kp_cal = min(c["kp"], t_cal / 0.08)
            sup = self.supervisor(limits=self._raw_limits(span), torque={j: 1.5 * t_cal}, v_max=self._vmax(j))
            self.start(sup)
            self.enable_all(sup)
            q_raw, tgt = self._seek(sup, j, side, t_cal, kp_cal, span + 0.35)
            # 3. stiff stop? push at 1.2 and 0.6 × t_cal: a hard stop barely moves, gravity / friction stalls do
            q1, tau1 = self._hold_at(sup, j, q_raw + side * 1.2 * t_cal / kp_cal, kp_cal, 0.25)
            q2, tau2 = self._hold_at(sup, j, q_raw + side * 0.6 * t_cal / kp_cal, kp_cal, 0.25)
            stiff = abs(q1 - q2) < 0.02
            row.update(stop_raw_rad=round(q1, 5), stop_deg=c["stop"][1], stiffness_dq_deg=round(math.degrees(abs(q1 - q2)), 3),
                       tau_at_stop_nm=round(tau1, 3))
            if not s.check(f"{j}: stall at the reference stop is a stiff hard stop", stiff,
                           f"Δq {math.degrees(abs(q1 - q2)):.2f}° between 1.2× and 0.6× push (limit 1.1°)"):
                rows.append(row)
                break
            s.check(f"{j}: drive torque report consistent with its limit at the stop", abs(tau1) <= 1.12 * t_cal + 0.02 * rated,
                    f"{tau1:.2f} N·m reported vs limit {t_cal:.2f} (the report is mapped to ±T_MAX: the sag test in 'gravity' proves the clamp)")
            # 4. zero
            d.set_offset(d.offset + q1 - th_stop)
            row["offset_rad"] = round(d.offset, 6)
            self.calibrated.add(j)
            # 5. span check against the opposite stop
            stop_lim = {j: (min(th_stop, th_other) - 10 * D2R, max(th_stop, th_other) + 10 * D2R)}
            sup = self.supervisor(limits={**self._raw_limits(span, skip=j), **stop_lim}, torque={j: 1.5 * t_cal}, v_max=self._vmax(j))
            self.start(sup)
            self.enable_all(sup)
            self.hold_q[j] = th_stop
            if c.get("span_check"):
                q_o, _ = self._seek(sup, j, -side, t_cal, kp_cal, span + 0.35)
                q_o1, _ = self._hold_at(sup, j, q_o - side * 1.2 * t_cal / kp_cal, kp_cal, 0.25)
                err = abs(q_o1 - th_stop) / span - 1
                row.update(span_measured_deg=round(math.degrees(abs(q_o1 - th_stop)), 2), span_expected_deg=round(math.degrees(span), 2))
                if not s.check(f"{j}: span stop-to-stop {math.degrees(abs(q_o1 - th_stop)):.1f}° vs {math.degrees(span):.1f}° expected",
                               abs(err) < 0.02, f"{err:+.1%} (belt ratio or stop angles wrong if > 2 %)"):
                    rows.append(row)
                    break
                self.hold_q[j] = q_o1
            # 6. operating limit back, park
            self._set_limit(d, self.lim(j))
            sup = self.supervisor(limits={**self._raw_limits(span, skip=j), **stop_lim}, v_max=self._vmax(j))
            self.start(sup)
            self.enable_all(sup)
            self.hold_q[j] = self.feedback().q[self.names.index(j)]
            self._move(sup, {j: c["park"] * D2R}, speed=self.cfg["cal_speed"])
            rows.append(row)
            s.check(f"{j}: zeroed against the {'upper' if side > 0 else 'lower'} stop ({c['stop'][1]}°), parked at {c['park']}°", True,
                    f"offset {d.offset:+.4f} rad, calibration torque {t_cal:.2f} N·m (gravity max {gmax:.2f})")
        s.data = dict(joints=rows, offsets={j: self.drivers[j].offset for j in self.calibrated},
                      signs={j: self.drivers[j].sign for j in self.calibrated})
        s.check("all online joints calibrated", self.calibrated == set(self.names), ", ".join(sorted(self.calibrated)))
        s.summary = f"{len(self.calibrated)}/{len(self.names)} joints zeroed against their hard stops"

    def _vmax(self, j):
        return np.array([self.cfg["cal_speed"] * 1.2 if x == j else self.cfg["jog_speed_frac"] * self.J[x]["max_speed_rad_s"] for x in self.names])

    def _raw_limits(self, span_unused=None, skip=None):
        """Travel bounds for joints not calibrated yet: ± (their full stop-to-stop span + 10°) around where they are now."""
        fb = self.feedback()
        out = {}
        for k, j in enumerate(self.names):
            if j in self.calibrated or j == skip:
                continue
            c = self.jc(j)
            sp = abs(c["stop"][1] - c["other"]) * D2R + 10 * D2R
            q0 = fb.q[k] if np.isfinite(fb.q[k]) else 0.0
            out[j] = (q0 - sp, q0 + sp)
        return out

    def _set_limit(self, d, t_joint):
        raw = d.torque_limit_to_drive(t_joint)
        d.disable()
        self.wait()
        d.write_reg(d.table["TORQUE_LIMIT_REG"], raw, self.reg_timeout)
        rb = d.read_reg(d.table["TORQUE_LIMIT_REG"], self.reg_timeout)
        if rb is None or abs(rb - raw) > 1e-4 * max(1, abs(raw)):
            raise StepFail(f"{d.name}: torque limit {raw} not read back ({rb})")
        if not d.ranges_from_drive() and d.table.get("RANGE_REGS"):
            raise StepFail(f"{d.name}: ranges not read back")

    # ---- jog
    def step_jog(self):
        from sw.collision import CollisionDetector
        s = self.steps["jog"]
        if self.io.estop_active():
            self.arm()
        sup = self.supervisor()
        self.start(sup)
        self.enable_all(sup)
        det = CollisionDetector(self.twin, np.full(6, np.inf), np.zeros(6))
        rows = []
        for j in self.names:
            c = self.jc(j)
            k = self.names.index(j)
            lo, hi = sup.lo[k], sup.hi[k]
            ctr = c["park"] * D2R
            A = c["jog"] * D2R
            ends = [float(np.clip(ctr + A, lo, hi)), float(np.clip(ctr - A, lo, hi)), ctr]
            vj = self.cfg["jog_speed_frac"] * self.J[j]["max_speed_rad_s"]
            self.op.watch_start(j)
            log = []
            first_dir = None
            det.reset()
            for e in ends:
                def rec(fb, v):
                    q6, v6, t6 = self._six(fb)
                    det.update(q6, v6, t6, self.dt)
                    log.append((self.hold_q[j], fb.q[k], fb.v[k], fb.t[k], det.rf[ARM.index(j)] if det.rf is not None else 0.0, v.q[k]))
                self._move(sup, {j: e}, settle=0.25, each=rec)
                if first_dir is None:                                 # the operator says which way it went (+ expected)
                    t_end = self.now() + (0.0 if self.plant is not None else 20.0)
                    first_dir = self.op.watch_direction(j)
                    while first_dir is None and self.now() < t_end:
                        self.cycle(sup, *self._cmd_hold(self.feedback()))
                        first_dir = self.op.watch_direction(j)
            L = np.array(log)
            err = np.abs(L[:, 5] - L[:, 1])
            vpk = float(np.max(np.abs(L[:, 2])))
            reached = [float(np.min(np.abs(L[:, 1] - e))) for e in ends]
            corr = float(np.corrcoef(L[:, 0], L[:, 1])[0, 1])
            row = dict(joint=j, ends_deg=[round(math.degrees(e), 1) for e in ends], speed_limit=round(vj, 3), peak_speed=round(vpk, 3),
                       max_track_err_deg=round(math.degrees(float(err.max())), 2), rms_track_err_deg=round(math.degrees(float(np.sqrt(np.mean(err ** 2)))), 2),
                       peak_torque_nm=round(float(np.max(np.abs(L[:, 3]))), 3), observer_peak_nm=round(float(np.max(np.abs(L[:, 4]))), 3),
                       direction=first_dir, corr=round(corr, 4), overrides=dict(sup.counts))
            rows.append(row)
            s.check(f"{j}: jog {row['ends_deg']}° at ≤ {vj:.2f} rad/s", vpk <= 1.3 * vj and max(reached) < 0.05 and float(err.max()) < 0.12,
                    f"peak {vpk:.2f} rad/s, tracking max {row['max_track_err_deg']}° rms {row['rms_track_err_deg']}°, ends reached within {math.degrees(max(reached)):.1f}°")
            s.check(f"{j}: moved + first as commanded (operator)", first_dir == 1 and corr > 0.9, f"operator saw {first_dir}, corr {corr:.3f}")
        s.data = dict(joints=rows, supervisor_overrides=dict(sup.counts))
        s.summary = "; ".join(f"{r['joint']} max err {r['max_track_err_deg']}° peak {r['peak_speed']} rad/s" for r in rows)

    def _six(self, fb):
        q6 = self.full_q(fb.q)
        v6, t6 = np.zeros(6), self.twin.gravity(q6)
        for k, j in enumerate(self.names):
            v6[ARM.index(j)] = fb.v[k]
            t6[ARM.index(j)] = fb.t[k]
        return q6, v6, t6

    # ---- gravity hold
    def step_gravity(self):
        s = self.steps["gravity"]
        gc = self.cfg["gravity"]
        if self.io.estop_active():
            self.arm()
        sup = self.supervisor()
        self.start(sup)
        self.enable_all(sup)
        n = len(self.names)
        poses = []
        for p in gc["poses"]:
            tgt = {j: float(np.clip(p[j] * D2R, sup.lo[self.names.index(j)], sup.hi[self.names.index(j)])) for j in self.names}
            meas = []
            for sgn in (+1, -1):
                self._move(sup, {j: tgt[j] + sgn * gc["approach_deg"] * D2R for j in self.names}, settle=0.05)
                self._move(sup, tgt, settle=gc["settle_s"])
                out = self.run(sup, int(gc["avg_s"] / self.dt), lambda k, fb: self._cmd_hold(fb))
                Q = np.array([f.q for f in out])
                T = np.array([f.t for f in out])
                g = np.array([self.gravity_online(qq) for qq in Q])
                meas.append(dict(q=Q.mean(0), tau=T.mean(0), g=g.mean(0), sd=T.std(0)))
            r_up, r_dn = meas[0]["tau"] - meas[0]["g"], meas[1]["tau"] - meas[1]["g"]
            poses.append(dict(pose={j: round(math.degrees(tgt[j]), 1) for j in self.names},
                              q=((meas[0]["q"] + meas[1]["q"]) / 2), twin=((meas[0]["g"] + meas[1]["g"]) / 2),
                              measured=((meas[0]["tau"] + meas[1]["tau"]) / 2), residual=(r_up + r_dn) / 2,
                              friction=np.abs(r_up - r_dn) / 2, noise=np.maximum(meas[0]["sd"], meas[1]["sd"])))
        R = np.array([p["residual"] for p in poses])
        F = np.array([p["friction"] for p in poses])
        S = np.array([p["noise"] for p in poses])
        rated = np.array([self.J[j]["rated_nm"] for j in self.names])
        worst = np.abs(R).max(0)
        for k, j in enumerate(self.names):
            s.check(f"{j}: gravity residual |measured − twin| ≤ {gc['max_residual_frac']:.0%} of rated",
                    worst[k] <= gc["max_residual_frac"] * rated[k],
                    f"max {worst[k]:.3f} N·m ({worst[k] / rated[k]:.1%}), mean {R[:, k].mean():+.3f}, friction ≤ {F[:, k].max():.3f}, noise σ {S[:, k].max():.3f}")
        out = dict(poses=[{k: (v.round(4).tolist() if isinstance(v, np.ndarray) else v) for k, v in p.items()} for p in poses],
                   joints=self.names, residual_max=worst.round(4).tolist(), residual_mean=R.mean(0).round(4).tolist(),
                   friction_max=F.max(0).round(4).tolist(), noise_max=S.max(0).round(4).tolist())
        # effective balancer K: the K that makes the twin's pitch torque match, from the part of the pitch residual that goes
        # like sin(pitch) (residual ≈ −ΔK·sign·sin(pitch)). It absorbs every error with that shape (spring rate ±10 %, slot
        # position, arm mass, torque constant), which is what the gravity feed-forward needs.
        if "pitch" in self.names and self.twin.k_bal:
            kp_ = self.names.index("pitch")
            from sw.virtual_arm import balancer_sign
            sgn = balancer_sign(self.twin.m, self.twin.qadr, self.twin.dadr)
            x = sgn * np.sin([p["q"][kp_] for p in poses])
            dk = -float(np.dot(x, R[:, kp_]) / max(np.dot(x, x), 1e-9))
            out["balancer_k_effective"] = round(self.twin.k_bal + dk, 3)
            out["balancer_k_nominal"] = self.twin.k_bal
        # drive clamp (sag) test at the pose with the largest gravity load per joint
        sag = {}
        for k, j in enumerate(self.names):
            G = np.array([abs(p["twin"][k]) for p in poses])
            i = int(G.argmax())
            if G[i] < 0.08 * rated[k]:
                sag[j] = dict(tested=False, why=f"gravity load ≤ {G[i]:.2f} N·m at every pose: not testable")
                continue
            sag[j] = self._sag_test(sup, j, poses[i], G[i])
            if self.io.estop_active():
                break
            sup = self.supervisor()
            self.start(sup)
            self.enable_all(sup)
        for j, r in sag.items():
            if r.get("tested"):
                s.check(f"{j}: drive enforces its torque limit (limit {r['limit_nm']:.2f} < gravity {r['gravity_nm']:.2f} N·m → sags)",
                        r["sagged"], f"moved {r['moved_deg']:.2f}° in {r['secs']:.2f} s")
        out["sag_test"] = sag
        # collision thresholds for sw/collision.py: 1.5 × max(observer peak in the jog, static residual + friction + 4σ), ≥ 0.3 N·m
        jog = {r["joint"]: r["observer_peak_nm"] for r in self.steps["jog"].data.get("joints", [])}
        thr = np.full(6, np.inf)
        basis = {}
        for k, j in enumerate(self.names):
            stat = worst[k] + F[:, k].max() + 4 * S[:, k].max()
            t = max(0.3, 1.5 * max(jog.get(j, 0.0), stat))
            thr[ARM.index(j)] = round(t, 3)
            basis[j] = dict(observer_peak=jog.get(j), static=round(float(stat), 4))
        out["collision_thresholds"] = dict(thr=[None if not np.isfinite(x) else float(x) for x in thr], rel=0.10, joints=ARM, basis=basis,
                                           rule="thr = max(0.3, 1.5 × max(jog observer peak, |static residual| + friction + 4σ)) N·m; rel 10 % of |τ|")
        s.data = out
        s.summary = ("residual max " + ", ".join(f"{j} {worst[k]:.2f} N·m" for k, j in enumerate(self.names))
                     + (f"; effective balancer K {out['balancer_k_effective']} N·m (nominal {out['balancer_k_nominal']}: set it in bringup.yaml)"
                        if "balancer_k_effective" in out else ""))
        if not self.io.estop_active():                            # end of bring-up: to the hanging rest pose, then limp
            rest = {j: float(np.clip(self.cfg["rest_pose"][j] * D2R, sup.lo[k], sup.hi[k])) for k, j in enumerate(self.names)}
            self._move(sup, rest, settle=0.3)
        self.disable_all()

    def step_sysid(self):
        from sw import sysid
        try:
            sysid.run(self, self.steps["sysid"])
        except RuntimeError as e:
            raise StepFail(str(e))

    def _sag_test(self, sup, j, pose, g):
        """Move to the pose, set the drive limit to half the gravity load, hold: a drive that enforces its limit lets the
        joint sag (stop after 3°); one that doesn't keeps holding. The supervisor stays in the loop throughout."""
        k = self.names.index(j)
        self._move(sup, {x: pose["pose"][x] * D2R for x in self.names}, settle=0.3)
        d = self.drivers[j]
        lim = 0.5 * g
        q0 = self.feedback().q[k]
        self._set_limit(d, lim)
        sup = self.supervisor()
        self.start(sup)
        self.enable_all(sup)
        moved, t0 = 0.0, self.now()
        while self.now() - t0 < 0.4:
            v, fb = self.cycle(sup, *self._cmd_hold(self.feedback()))
            moved = abs(fb.q[k] - q0)
            if v.stop or moved > 3 * D2R:
                break
        secs = self.now() - t0
        self._set_limit(d, self.lim(j))
        return dict(tested=True, limit_nm=lim, gravity_nm=g, moved_deg=math.degrees(moved), secs=secs, sagged=moved > 1 * D2R)

    # =================================================================================================================
    def run_all(self, until=None):
        for name in STEPS:
            if not self.require(name):
                continue
            s = self.steps[name]
            t0 = time.time()
            try:
                getattr(self, f"step_{name}")()
                s.status = "PASS" if s.checks and all(ok for _, ok, _ in s.checks) else "FAIL"
            except StepFail as e:
                s.check("step completed", False, str(e))
                s.status = "FAIL"
                try:
                    self.disable_all()
                except Exception:
                    pass
            except P.ProtocolTodo as e:
                s.check("protocol table complete", False, str(e))
                s.status = "FAIL"
            s.data["wall_s"] = round(time.time() - t0, 2)
            self.log(f"[{s.status:7s}] {name}: {s.summary or ''}" + ("" if s.status == "PASS" else
                     "  ← " + "; ".join(f"{w}: {dd}" for w, ok, dd in s.checks if not ok)))
            if until and name == until:
                break
        try:
            self.disable_all()
        except Exception:
            pass
        return self.steps

    # ---- report
    def report(self, out_dir="docs/sw", tag=None, bus_desc=""):
        date = _dt.date.today().isoformat()
        tag = tag if tag is not None else ("_virtual" if self.plant is not None else "")
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        js = dict(date=date, bus=bus_desc, virtual=self.plant is not None, joints_online=self.names, blocked=self.blocked,
                  steps={k: dict(status=v.status, summary=v.summary, checks=v.checks, notes=v.notes, data=v.data) for k, v in self.steps.items()},
                  calibration=dict(offsets={j: self.drivers[j].offset for j in sorted(self.calibrated)},
                                   signs={j: self.drivers[j].sign for j in sorted(self.calibrated)}),
                  gim_todo=P.missing(self.tables["gim6010"]), cycles=self.n_cycles, wall_s=round(time.time() - self.t_start, 1),
                  heartbeat_gap_max_ms=round(self.hb_gap_max * 1000, 2), watchdog_ms=round(float(self.cfg.get("watchdog_s", 0.025)) * 1000, 1))
        if self.plant is not None:
            js["virtual_truth"] = self.plant.truth()
        jp = out_dir / f"bringup_{date}{tag}.json"
        jp.write_text(json.dumps(js, indent=1, default=_jsonable))
        mp = out_dir / f"bringup_{date}{tag}.md"
        mp.write_text(self._markdown(js, jp.name))
        return mp, jp, js

    def _markdown(self, js, jname):
        L = [f"# M1 bring-up report {js['date']}{' (VIRTUAL: MuJoCo bench twin on a python-can virtual bus)' if js['virtual'] else ''}",
             "", f"Generated by `python -m sw.bringup` ({js['bus']}); data: [{jname}]({jname}). Online: **{', '.join(js['joints_online']) or '—'}**; "
             f"blocked: {', '.join(js['blocked']) or '—'}; {js['cycles']} control cycles at {self.cfg['control_hz']} Hz. "
             f"Longest heartbeat gap {js['heartbeat_gap_max_ms']:.1f} ms against the {js['watchdog_ms']:.0f} ms hardware watchdog"
             f"{'' if js['virtual'] else ' (real loop: if it is over half the watchdog, raise watchdog_s and re-check electrical.md)'}.", "",
             "| Step | Result | Summary |", "|---|---|---|"]
        for k in STEPS:
            v = js["steps"][k]
            L.append(f"| {k} | {'✅' if v['status'] == 'PASS' else '❌' if v['status'] == 'FAIL' else '⛔'} {v['status']} | {v['summary']} |")
        for k in STEPS:
            v = js["steps"][k]
            if not v["checks"] and not v["notes"]:
                continue
            L += ["", f"## {k}", ""] + [f"- {'✅' if ok else '❌'} {w}{(': ' + d) if d else ''}" for w, ok, d in v["checks"]]
            L += [f"- {x}" for x in v["notes"]]
        cal = js["calibration"]["offsets"]
        if cal:
            L += ["", "## Calibration (joint-side zero offsets, rad; q = sign·p_motor/ratio − offset)", "",
                  "| Joint | Offset | Sign |", "|---|---|---|"] + [f"| {j} | {o:+.5f} | {js['calibration']['signs'][j]:+d} |" for j, o in cal.items()]
        ct = js["steps"]["gravity"]["data"].get("collision_thresholds")
        if ct:
            L += ["", "## Suggested collision thresholds (sw/collision.py `CollisionDetector(thr, rel)`)", "", ct["rule"], "",
                  "| Joint | thr N·m | basis |", "|---|---|---|"]
            for j, t in zip(ct["joints"], ct["thr"]):
                if t is not None:
                    L.append(f"| {j} | {t:.2f} | {ct['basis'].get(j)} |")
            L += ["", "Load them with `sw.bringup.load_collision_thresholds('<this json>')`."]
        L += ["", "## GIM6010 protocol table (sw/drivers/gim.py): fields still TODO", ""]
        if js["gim_todo"]:
            L += ["| Field | What to look up in the SteadyWin SDC104 manual |", "|---|---|"] + \
                 [f"| `{k}` | {GIM_TODO_HELP.get(k, '')} |" for k in js["gim_todo"]]
        else:
            L.append("none: the table is complete.")
        if js.get("virtual_truth"):
            L += ["", "## Virtual run: hidden plant values (for the test, not available on hardware)", "",
                  "| Joint | true offset | found | error |", "|---|---|---|---|"]
            for j, t in js["virtual_truth"].items():
                f, tru = cal.get(j), t["offset_joint"]
                found = "—" if f is None else f"{f:+.5f}"
                err = "—" if f is None else f"{math.degrees(f - tru):+.3f}°"
                L.append(f"| {j} | {tru:+.5f} | {found} | {err} |")
        return "\n".join(L) + "\n"


def _set(cmd, k, q, kp, kd, tff):
    a = [x.copy() for x in cmd]
    a[0][k], a[1][k], a[2][k], a[3][k] = q, kp, kd, tff
    return tuple(a)


def _jsonable(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    return str(x)


def load_collision_thresholds(path):
    """(thr, rel) for sw.collision.CollisionDetector from a bring-up JSON (inf for joints not measured)."""
    js = json.loads(Path(path).read_text())
    ct = js["steps"]["gravity"]["data"]["collision_thresholds"]
    return np.array([np.inf if t is None else t for t in ct["thr"]]), np.array([ct["rel"] if t is not None else 0.0 for t in ct["thr"]])


# ---------------------------------------------------------------------------------------------------------------------
def build_virtual(cfg=None, gim_table=None, faults=(), seed=0, joints=None, hand_extra_kg=0.05, log=print, bus="ideal", act=None,
                  rig_only=False, actuator_dir=None, **kw):
    """A BringUp wired to a VirtualArm (python-can virtual bus + MuJoCo plant + simulated e-stop circuit and operator).
    bus: sw/bus_sim.py profile (latency, jitter, drops, duplicates, corruption); act: hidden actuator truth (see
    sw/virtual_arm.py); rig_only: only `joints` are motors in the plant (the Step-1 rig: the rest held in the fixture)."""
    from sw.can_map import load
    from sw.drivers.canbus import CanBus
    from sw.virtual_arm import SimOperator, VirtualArm
    cfg = cfg or load_cfg()
    cm = load()
    tables = dict(TABLES)
    if gim_table is not None:
        tables["gim6010"] = gim_table
    arm = VirtualArm(cm, cfg, tables, seed=seed, faults=faults, hand_extra_kg=hand_extra_kg, bus=bus, act=act,
                     joints=joints if rig_only else None, actuator_dir=actuator_dir)
    buses = {}
    for bn in cm["buses"]:
        b = CanBus(arm.endpoint(bn), clock=arm.clock)
        b.pump = arm.handle_frames
        b.err_source = arm.imp[bn].error_counters
        buses[bn] = b
    # register replies wait in simulated time (the loop keeps cycling: BringUp._reg_idle), so a 50 ms timeout is 25 cycles
    bu = BringUp(buses, cm, cfg, tables, io=arm.io, operator=SimOperator(arm), clock=arm.clock, plant=arm, joints=joints,
                 reg_timeout=0.05, log=log, **kw)
    tables_used = tables
    bu.tables = tables_used
    return bu, arm


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", default="virtual", help="virtual | socketcan:can0,can1 | slcan:/dev/ttyACM0[@bitrate],... (one per bus in "
                                                     "can_map order; a single channel carries every bus)")
    ap.add_argument("--estop", default=None, help="gpiod:<chip>:<heartbeat line>:<aux line> (real bench); virtual uses the simulated circuit")
    ap.add_argument("--joints", default=None, help="comma list (default: every joint whose protocol table is complete)")
    ap.add_argument("--until", default=None, choices=STEPS)
    ap.add_argument("--limit", default=None, choices=["rated", "peak"])
    ap.add_argument("--gim-table", default=None, choices=["sim"], help="virtual only: use the SIM PLACEHOLDER GIM table (exercises the GIM path)")
    ap.add_argument("--fault", action="append", default=[], help="virtual only: inject a plant fault (see sw/virtual_arm.py)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="docs/sw")
    a = ap.parse_args(argv)
    joints = a.joints.split(",") if a.joints else None
    if a.bus == "virtual":
        from sw.virtual_arm import SIM_GIM
        bu, arm = build_virtual(gim_table=SIM_GIM if a.gim_table == "sim" else None, faults=a.fault, seed=a.seed, joints=joints)
        desc = "virtual bus" + (", SIM PLACEHOLDER GIM table" if a.gim_table else "") + (f", faults {a.fault}" if a.fault else "")
        tag = "_virtual" + ("_simgim" if a.gim_table else "")
    else:
        from sw.can_map import load
        from sw.drivers.canbus import CanBus
        cm = load()
        kind, _, chans = a.bus.partition(":")
        chans = chans.split(",")
        names = list(cm["buses"])
        if len(chans) not in (1, len(names)):
            ap.error(f"--bus needs 1 or {len(names)} channels")
        opened = {}
        buses = {}
        for i, bn in enumerate(names):
            ch = chans[i if len(chans) > 1 else 0]
            if ch not in opened:
                opened[ch] = CanBus(f"{kind}:{ch}")
            buses[bn] = opened[ch]
        io = None
        if a.estop and a.estop.startswith("gpiod:"):
            _, chip, hb, aux = a.estop.split(":")
            io = GpiodSafetyIO(chip, hb, aux)
        bu = BringUp(buses, cm, io=io, operator=ConsoleOperator(), joints=joints, limit=a.limit)
        desc, tag = a.bus, ""
        print("Real bench: PSU current-limited, regen clamp fitted, arm in the cradle, e-stop in reach (docs/rnd/electrical.md A5).")
    bu.run_all(until=a.until)
    mp, jp, js = bu.report(a.out, tag=tag, bus_desc=desc)
    print(f"report: {mp}  {jp}")
    ok = all(bu.steps[s].status == "PASS" for s in STEPS[:STEPS.index(a.until) + 1 if a.until else len(STEPS)])
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
