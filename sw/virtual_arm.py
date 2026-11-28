"""Virtual M1 bench arm: the MuJoCo bench twin as the plant, simulated drives on a python-can virtual bus (S1).

sw/bringup.py talks to this exactly as to the real arm: CAN frames through python-can (interface "virtual"), an e-stop /
contactor / heartbeat-watchdog block (`SimSafetyIO`, the A5 v2 circuit of electrical.md) and an operator (`SimOperator`,
who presses the button and watches the joints). Nothing in the bring-up code knows it is simulated.

The plant is not the twin the bring-up uses for its model torques, on purpose: hidden truths the bring-up has to find or
live with —
- every motor has an unknown encoder zero (uniform ±1.5 rad motor side) and the hard stops sit where bringup.yaml says;
- torque constant error ±3 % per drive (the drive reports the torque it commands; the joint gets a bit more or less),
  current-reading noise σ 0.5 % of rated, Coulomb friction (0.25 N·m GIM joints, 0.04 wrist), the balancer's real K
  (5 % above nominal) and an extra hand mass (`hand_extra_kg`, default 0.05 kg) the twin does not know;
- each drive clamps its output to its torque-limit register (unless the `no_clamp` fault is injected), keeps registers in
  RAM until saved to flash, loses RAM on a power cycle, and disables itself after its CAN timeout.
Joints that are not simulated motors (GIM joints while their protocol table is TODO, the STS3215 wrist roll) are held in
the bench fixture at `fixture_pose`.

Faults for the tests (`faults=`): factory_id:<joint> (drive answers at its factory ID), estop_stuck (contactor welded),
watchdog_dead (the monostable never times out), no_clamp (drive ignores its torque limit), reversed:<joint> (motor
mounted the other way round), no_timeout (drive CAN timeout ignored), drift (Coulomb friction +25 %/K of motor temperature
rise: a synthetic degrading drive for the soak test, sw/soak.py).

Bus realism (`bus=`, sw/bus_sim.py): each CAN bus can delay (0–9 ms, FIFO, so frames bunch), drop, duplicate, CRC-reject
(error counters, bus-off) or corrupt frames, in both directions; `bus="ideal"` (default) is the plain virtual bus.

Actuator truth (`act=`): per joint Coulomb friction, viscous friction, torque-constant scale (true joint torque /
drive-reported torque) and backlash (a quasi-static gap between the motor and the joint: the drive's encoder is on the
motor side, so the gap is invisible except as lost motion at torque reversals). `act=None` loads the fitted model
(sw/config/actuators_<serial>.yaml, written by the bring-up's sysid step) when present, else the defaults below;
`act={}` forces the defaults; `act={joint: {...}}` sets hidden values (the sysid rehearsal). `ext_tau` is an external
joint torque (a contact) for the collision tests; `add_test_mass(kg)` hangs a known weight at the grasp site (the lever
weights of the Step-1 rig).

`SIM_GIM` is a **placeholder** GIM protocol table (the DM-J4310 layout with wider ranges) so the GIM code path can be
exercised end to end here. It is not the SteadyWin protocol.
"""
from __future__ import annotations

import math
import os

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

from sw.drivers import protocol as P            # noqa: E402
from sw.drivers.mit import DM_J4310, Ranges, pack_feedback, unpack_cmd   # noqa: E402

SIM_GIM = dict(DM_J4310, NAME="SIM PLACEHOLDER (not the SteadyWin protocol): DM layout, p_max 25 rad",
               RANGES=dict(p_max=25.0, v_max=65.0, t_max=12.0, kp_max=500.0, kd_max=5.0), ABSOLUTE=False)
ARM = ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll"]
MOTOR_PEAK = {"gim6010": 11.0, "dm_j4310": 7.0}          # N·m at the motor output (what the drive could give unclamped)


class SimClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class SimMotor:
    def __init__(self, name, kind, cmd_id, fb_id, ratio, table, rng, rated_joint, sign_true=1, factory=False, clamp=True, timeout=True,
                 act=None):
        self.name, self.kind, self.ratio, self.table, self.rng = name, kind, ratio, table, rng
        self.sign_true, self.clamp, self.timeout_on = sign_true, clamp, timeout
        cid, fid = (table["DEFAULT_ID"] if factory else (cmd_id, fb_id))
        rg = table["RANGES"]
        self.regs = {"esc_id": cid, "mst_id": fid, "ctrl_mode": table["CTRL_MODE_MIT"], "fw_version": 0x0105, "hw_version": 0x0101,
                     "serial": int(rng.integers(1e6, 9e6)), "kt": 0.09, "gear_ratio": 10.0 if kind == "dm_j4310" else 8.0,
                     "p_max": rg["p_max"], "v_max": rg["v_max"], "t_max": rg["t_max"], "can_timeout": 0}
        self.flash = dict(self.regs)
        self.offset_m = float(rng.uniform(-1.5, 1.5))
        self.kt_err = 1 + float(rng.uniform(-0.03, 0.03))
        act = act or {}
        if act.get("kt_scale") is not None:
            self.kt_err = float(act["kt_scale"])
        self.backlash = float(act.get("backlash_rad") or 0.0)     # joint side, total gap
        self.delta = 0.0                                          # motor − joint position inside the gap (joint side, rad)
        self.pending_t = None                                     # host send time of the oldest command not yet answered
        self.noise = 0.005 * rated_joint / ratio
        self.powered, self.enabled, self.status = True, False, table["STATUS_DISABLED"]
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.last_cmd_t = None
        self.tau_cmd = 0.0
        self.want_reply = False
        self.temp = 32.0

    @property
    def id(self):
        return self.regs["esc_id"]

    @property
    def fb(self):
        return self.regs["mst_id"]

    def ranges(self):
        rg = self.table["RANGES"]
        return Ranges(self.regs["p_max"], self.regs["v_max"], self.regs["t_max"], rg["kp_max"], rg["kd_max"])

    def power(self, on, now):
        if on and not self.powered:
            self.regs = dict(self.flash)                       # RAM lost: back to the saved parameters
            if not self.table.get("ABSOLUTE", True):
                self.offset_m = float(self.rng.uniform(-1.5, 1.5))
        self.powered = on
        self.enabled = False
        self.status = self.table["STATUS_DISABLED"]
        self.tau_cmd = 0.0

    # motor-side state from the joint's physical state
    def motor_pos(self, q):
        return self.sign_true * self.ratio * (q + self.delta) + self.offset_m

    def motor_vel(self, qd):
        return self.sign_true * self.ratio * qd

    def handle(self, arb_id, data, now, send):
        """One received frame. Returns True if it was for this motor."""
        t = self.table
        if not self.powered:
            return False
        for key in ("ENABLE", "DISABLE", "SET_ZERO", "CLEAR_ERROR"):
            a, tpl = t[key]
            if arb_id == P.arb(a, self.id, self.fb) and data == P.build(tpl, self.id):
                if key == "ENABLE" and self.status in t["STATUS_OK"]:
                    self.enabled, self.status, self.last_cmd_t = True, t["STATUS_ENABLED"], now
                    self.cmd = (self._p, 0.0, 0.0, 0.0, 0.0)
                elif key == "DISABLE":
                    self.enabled, self.status, self.tau_cmd = False, t["STATUS_DISABLED"], 0.0
                elif key == "CLEAR_ERROR" and self.status not in t["STATUS_OK"]:
                    self.status = t["STATUS_DISABLED"]
                elif key == "SET_ZERO" and not self.enabled:
                    self.offset_m -= self._p
                self.want_reply = True
                return True
        for key in ("REFRESH", "PARAM_READ", "PARAM_WRITE", "PARAM_SAVE"):
            a, tpl = t[key]
            if arb_id != P.arb(a, self.id, self.fb):
                continue
            f = P.parse(tpl, data)
            if f is None or f.get("id") != self.id:
                continue
            if key == "REFRESH":
                self.want_reply = True
            elif key == "PARAM_SAVE":
                if not self.enabled:
                    self.flash = dict(self.regs)
            else:
                name = next((n for n, (rid, _) in t["REG"].items() if rid == f["reg"]), None)
                if name is None:
                    return True
                fmt = t["REG"][name][1]
                if key == "PARAM_WRITE" and not self.enabled:
                    v = P.decode_value(f["value_bytes"], fmt)
                    self.regs[name] = float(v) if fmt == "f32" else int(v)
                ra, rtpl = t["PARAM_REPLY"]
                fmt_v = self.regs[name]
                out = bytearray(P.build(rtpl.replace("*", "00"), self.id, f["reg"], fmt_v, fmt))
                for i, tok in enumerate(rtpl.split()):
                    if tok == "*":
                        out[i] = data[i]                       # echo the opcode
                send(P.arb(ra, self.id, self.fb), bytes(out))
            return True
        a = t["CONTROL"]
        if arb_id == P.arb(a, self.id, self.fb):
            if self.enabled:
                self.cmd = unpack_cmd(data, self.ranges())
                self.last_cmd_t = now
            self.want_reply = True
            return True
        return False

    def torque(self, q, qd, now):
        """Physical joint torque this cycle (and the drive's reported torque in self.tau_cmd, motor side)."""
        if self.enabled and self.timeout_on and self.regs["can_timeout"] > 0 and self.last_cmd_t is not None:
            if now - self.last_cmd_t > self.regs["can_timeout"] * self.table["CAN_TIMEOUT_UNIT_S"] + 1e-9:
                self.enabled, self.status = False, self.table["STATUS_TIMEOUT"]
        if not (self.powered and self.enabled):
            self.tau_cmd = 0.0
            return 0.0
        pd, vd, kp, kd, tff = self.cmd
        if self.backlash > 0:
            # quasi-static gap: a massless rotor sits where the MIT law gives zero torque if that is inside the gap, else on
            # the flank (δ = ±b/2) and the torque goes through. τ_m(δ) = τ_m(0) − kp·s·r·δ
            s_, r = self.sign_true, self.ratio
            old, self.delta = self.delta, 0.0
            tau0 = kp * (pd - self.motor_pos(q)) + kd * (vd - self.motor_vel(qd)) + tff
            h = self.backlash / 2
            if kp > 1e-9:
                d0 = tau0 / (kp * s_ * r)
                self.delta = float(np.clip(d0, -h, h))
            else:
                self.delta = h * float(np.sign(s_ * tau0)) if abs(tau0) > 1e-12 else old
            tau = tau0 - kp * s_ * r * self.delta
            if abs(tau) < 1e-12:
                tau = 0.0
        else:
            p, v = self.motor_pos(q), self.motor_vel(qd)
            tau = kp * (pd - p) + kd * (vd - v) + tff
        lim = min(self.regs["t_max"], MOTOR_PEAK[self.kind]) if self.clamp else MOTOR_PEAK[self.kind]
        tau = float(np.clip(tau, -lim, lim))
        self.tau_cmd = tau
        self.temp += (0.02 * tau * tau - 0.01 * (self.temp - 32.0)) * 0.002
        return self.sign_true * self.ratio * tau * self.kt_err

    def feedback(self, q, qd, send):
        if not (self.powered and self.want_reply):
            return
        self.want_reply = False
        r = self.ranges()
        t_rep = self.tau_cmd + self.rng.normal(0, self.noise)
        send(P.arb(self.table["FEEDBACK"], self.id, self.fb),
             pack_feedback(self.id, self.status, self.motor_pos(q), self.motor_vel(qd), float(np.clip(t_rep, -r.t_max, r.t_max)),
                           int(self.temp), int(self.temp) + 2, r))

    @property
    def _p(self):
        return self._last_p if hasattr(self, "_last_p") else 0.0


class SimSafetyIO:
    """A5 v2 stop circuit: mushroom NC contacts + watchdog monostable in series with the contactor coil; aux contact read
    back. `arm()` is the operator's reset/arm button (monitored manual reset: never closes on its own).
    t_release: contactor release time after the coil is switched off (default 0: opens at once); set WD_S to the
    monostable's worst tolerance (× 1.2) and t_release to the datasheet's 10 ms for timing tests (sw/test_s2.py). Once the
    coil is off the board's LATCH is cleared, so the contactor opens even if the heartbeat comes back in the meantime."""
    WD_S = 0.025                    # default; VirtualArm passes bringup.yaml watchdog_s (electrical.md: sized on sw/loop_jitter.py)

    def __init__(self, clock, stuck=False, wd_dead=False, wd_s=None, t_release=0.0):
        self.clock, self.stuck, self.wd_dead = clock, stuck, wd_dead
        if wd_s is not None:
            self.WD_S = float(wd_s)
        self.t_release = float(t_release)
        self.button = False
        self.closed = False
        self.last_hb = None
        self.t_open = None
        self.t_coil_off = None          # when the coil lost its drive (watchdog / button), before the contacts open
        self.listeners = []

    def heartbeat(self):
        self.last_hb = self.clock()

    def update(self):
        now = self.clock()
        if not self.closed:
            return
        hb_lost = (not self.wd_dead) and (self.last_hb is None or now - self.last_hb > self.WD_S)
        if self.t_coil_off is None and ((self.button and not self.stuck) or hb_lost):
            self.t_coil_off = now
        if self.t_coil_off is not None and now - self.t_coil_off >= self.t_release - 1e-12:
            self.closed = False
            self.t_open = now
            self.t_coil_off = None
            for f in self.listeners:
                f(False)

    def drop(self):
        """The contactor opens without a command: supply trip, coil wire broken (the LATCH clears with it)."""
        if self.closed:
            self.closed = False
            self.t_open = self.clock()
            self.t_coil_off = None
            for f in self.listeners:
                f(False)

    def arm(self):
        now = self.clock()
        if self.button or self.last_hb is None or now - self.last_hb > self.WD_S or self.t_coil_off is not None:
            return False
        if not self.closed:
            self.closed = True
            for f in self.listeners:
                f(True)
        return True

    def estop_active(self):
        return not self.closed

    def press(self):
        self.button = True
        self.update()

    def release(self):
        self.button = False


class SimOperator:
    """Plays the person at the bench: presses/releases the e-stop, re-arms, and reports which way a joint moved."""

    def __init__(self, arm):
        self.arm = arm

    def confirm(self, text):
        return True

    def press_estop(self):
        self.arm.io.press()

    def release_estop(self):
        self.arm.io.release()

    def arm_contactor(self):
        return self.arm.io.arm()

    def watch_start(self, joint):
        self._w = self.arm.q_phys(joint)

    def watch_direction(self, joint):
        """+1 if the joint moved in its positive direction (the arrow on the part) since watch_start, −1 otherwise."""
        return 1 if self.arm.q_phys(joint) - self._w > 0 else -1

    def hang_test_mass(self, kg):
        self.arm.add_test_mass(kg)

    def test_mass_ready(self):
        return True


DEFAULT_COULOMB = {"pitch": 0.25, "roll": 0.25, "yaw": 0.25, "elbow": 0.25, "wpitch": 0.04}   # N·m at the joint (plant default)


class VirtualArm:
    _n = 0

    def __init__(self, can_map, cfg, tables, seed=0, faults=(), hand_extra_kg=0.05, bus="ideal", act=None, joints=None,
                 actuator_dir=None):
        """bus: a sw/bus_sim.py profile name / spec dict (every bus), or {bus name: profile}. act: None = fitted model when
        present (sw/config/actuators_*.yaml or `actuator_dir`), {} = defaults, {joint: {coulomb_nm, viscous_nms, kt_scale,
        backlash_rad}} = hidden truth. joints: only these joints are simulated motors (the others held in the fixture)."""
        import can
        from sw import bus_sim
        VirtualArm._n += 1
        self._prefix = f"pip{os.getpid()}_{VirtualArm._n}"           # python-can virtual channels are process-global
        import mujoco
        from sw.bench_env import BenchEnv
        self.mj = mujoco
        self.rng = np.random.default_rng(seed)
        self.clock = SimClock()
        self.faults = set(faults)
        self.cfg = cfg
        env = BenchEnv(render=False)
        env.reset(0)
        self.env, m, d = env, env.m, env.d
        m.body_gravcomp[:] = 0
        m.actuator_gainprm[:, 0] = 0
        m.actuator_biasprm[:, :] = 0
        m.geom_contype[:] = 0
        m.geom_conaffinity[:] = 0
        self.jid = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in ARM}
        self.qadr, self.dadr = env.qadr, env.dadr
        J = {j["name"]: j for j in _joints()}
        for j, c in cfg["joints"].items():                    # the physical hard stops
            a, b = sorted((math.radians(c["stop"][1]), math.radians(c["other"])))
            m.jnt_range[self.jid[j]] = (a, b)
        self.hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand")
        if hand_extra_kg:
            from sw.collision import add_point_mass
            add_point_mass(m, self.hand, hand_extra_kg, m.site_pos[env.site].copy())
        self._hand0 = (float(m.body_mass[self.hand]), m.body_ipos[self.hand].copy(), m.body_inertia[self.hand].copy())
        self.test_mass = 0.0
        # actuator truth: joint friction in the plant (Coulomb = MuJoCo frictionloss, viscous = dof damping)
        if act is None:
            from rnd.twin import measured_actuators
            act = measured_actuators(actuator_dir)
            self.act_source = "fitted (actuators_*.yaml)" if act else "defaults"
        else:
            self.act_source = "given" if act else "defaults"
        self.act = {}
        for k, j in enumerate(ARM[:5]):
            a = dict((act or {}).get(j) or {})
            fc = a.get("coulomb_nm", DEFAULT_COULOMB[j])
            b = a.get("viscous_nms", float(m.dof_damping[self.dadr[k]]))
            m.dof_frictionloss[self.dadr[k]] = fc
            m.dof_damping[self.dadr[k]] = b
            self.act[j] = dict(coulomb_nm=fc, viscous_nms=b, kt_scale=a.get("kt_scale"), backlash_rad=a.get("backlash_rad", 0.0))
        self.ext_tau = np.zeros(6)                            # external joint torque (a contact), N·m
        self.bal = cfg.get("balancer") or {}
        self.bal_k_true = float(self.bal.get("k_nm", 0.0)) * 1.05
        self.bal_sign = balancer_sign(m, self.qadr, self.dadr)
        # motors
        self.buses = {}
        self.motors = []
        self.can = can
        bus_rng = np.random.default_rng(seed + 7919)          # own stream: an ideal bus leaves the plant's draws unchanged
        self.imp, self.to_drive, self.to_host = {}, {}, {}
        self.rt_log = []                                      # true command → host-visible reply times (s), control frames
        for bn, b in can_map["buses"].items():
            self.buses[bn] = can.Bus(interface="virtual", channel=f"{self._prefix}_{bn}")
            spec = bus.get(bn, "ideal") if isinstance(bus, dict) and not set(bus) & set(_SPEC_KEYS) else bus
            self.imp[bn] = bus_sim.make(spec, bus_rng, bn)
            self.to_drive[bn], self.to_host[bn] = bus_sim.DelayLine(), bus_sim.DelayLine()
            for dv in b["devices"]:
                if dv["kind"] not in tables:
                    continue
                if joints is not None and dv["name"] not in joints:
                    continue                                    # not on this rig: held in the fixture
                tbl = tables[dv["kind"]]
                if P.missing(tbl):
                    continue                                    # protocol unknown: not simulated (joint held in the fixture)
                mt = SimMotor(dv["name"], dv["kind"], dv["id"], dv["fb_id"], dv["ratio"], tbl, self.rng, J[dv["name"]]["rated_nm"],
                              sign_true=-1 if f"reversed:{dv['name']}" in self.faults else 1,
                              factory=f"factory_id:{dv['name']}" in self.faults, clamp="no_clamp" not in self.faults,
                              timeout="no_timeout" not in self.faults, act=self.act.get(dv["name"]))
                mt.bus = bn
                mt.ctrl_arb = P.arb(tbl["CONTROL"], mt.id, mt.fb)
                self.motors.append(mt)
                self.act[dv["name"]]["kt_scale"] = mt.kt_err
        self.sim_joints = [mt.name for mt in self.motors]
        self.io = SimSafetyIO(self.clock, stuck="estop_stuck" in self.faults, wd_dead="watchdog_dead" in self.faults, wd_s=cfg.get("watchdog_s"))
        self.io.listeners.append(lambda on: [mt.power(on, self.clock()) for mt in self.motors])
        for mt in self.motors:
            mt.power(False, 0.0)
        # power-on pose: hanging (rest pose) with the fixture for joints that are not simulated, yaw anywhere
        rest = cfg["rest_pose"] if self.sim_joints and set(self.sim_joints) >= {"pitch", "roll", "elbow"} else cfg["fixture_pose"]
        self.hold = {}
        for k, j in enumerate(ARM):
            q = math.radians(rest[j]) if j in self.sim_joints else math.radians(cfg["fixture_pose"][j])
            if j == "yaw" and j in self.sim_joints:
                q = float(self.rng.uniform(-1.0, 1.0))
            d.qpos[self.qadr[k]] = q
            if j not in self.sim_joints:
                self.hold[k] = math.radians(cfg["fixture_pose"][j])
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.fa = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "fa")]
        self.fa_d = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "fa")]
        # fixture stiffness per joint: 15 Hz, critically damped, from the joint's own inertia (stable at the 2 ms step)
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, M, d.qM) if hasattr(d, "qM") else mujoco.mj_fullM(m, d, M)
        w = 2 * math.pi * 15
        self.fix_gain = {k: (M[self.dadr[k], self.dadr[k]] * w * w, 2 * M[self.dadr[k], self.dadr[k]] * w) for k in range(6)}
        self._settle()

    def _settle(self):
        """Let the limp arm come to rest before power-on."""
        for _ in range(400):
            self._physics(np.zeros(6))

    def endpoint(self, bus_name):
        """The --bus spec the host side opens for this bus."""
        return f"virtual:{self._prefix}_{bus_name}"

    def q_phys(self, joint):
        return float(self.env.d.qpos[self.qadr[ARM.index(joint)]])

    def add_test_mass(self, kg):
        """Hang a known weight at the grasp site (0 removes it): the operator's lever weight."""
        from sw.collision import add_point_mass
        m = self.env.m
        m.body_mass[self.hand], m.body_ipos[self.hand], m.body_inertia[self.hand] = self._hand0[0], self._hand0[1].copy(), self._hand0[2].copy()
        if kg > 0:
            add_point_mass(m, self.hand, kg, m.site_pos[self.env.site].copy())
        self.test_mass = float(kg)

    # ---- the wire: python-can virtual bus + impairments (sw/bus_sim.py)
    def _send(self, bn, meta=None, base=None):
        """A drive's transmit: the frame leaves at `base` (when the drive answered: the arrival of the request it answers)
        and reaches the host after the bus's RX delay."""
        imp, line = self.imp[bn], self.to_host[bn]

        def send(a, data):
            t0 = self.clock() if base is None else base
            line.push([(t, (a, d)) for t, d in imp.pass_frame("rx", t0, bytes(data))], meta)
            self._flush_host(bn)
        return send

    def _flush_host(self, bn):
        now = self.clock()
        for t, (a, d), meta in self.to_host[bn].due(now):
            if meta is not None:
                self.rt_log.append(now - meta)
            self.buses[bn].send(self.can.Message(arbitration_id=a, data=d, is_extended_id=False))

    def handle_frames(self, lookahead=0.0):
        """Deliver the host's frames that have reached the drives. Inside step() `lookahead` = the step: a frame that
        arrives during the 2 ms physics step acts from its start (≤ 1 step early) and is answered from its arrival time,
        so a sub-cycle bus delay costs no extra cycle (a drive answers within ≈ 0.1 ms)."""
        now = self.clock()
        for bn, bus in self.buses.items():
            self._flush_host(bn)
            imp, line = self.imp[bn], self.to_drive[bn]
            while True:
                msg = bus.recv(0.0)
                if msg is None:
                    break
                line.push([(t, (msg.arbitration_id, d)) for t, d in imp.pass_frame("tx", now, bytes(msg.data))], now)
            for t, (a, d), t_sent in line.due(now + lookahead):
                t_arr = max(t, now)
                for mt in self.motors:
                    if mt.bus == bn and mt.handle(a, d, t_arr, self._send(bn, base=t_arr)):
                        mt.t_arr = t_arr
                        if a == mt.ctrl_arb and mt.pending_t is None:
                            mt.pending_t = t_sent
                        break

    def _physics(self, tau_motor_joint):
        m, d, mj = self.env.m, self.env.d, self.mj
        d.qfrc_applied[:] = 0
        q = np.array([d.qpos[a] for a in self.qadr])
        qd = np.array([d.qvel[a] for a in self.dadr])
        for k in range(6):
            d.qfrc_applied[self.dadr[k]] = tau_motor_joint[k] + self.ext_tau[k]
            if k in self.hold:                                    # bench fixture
                kf, df = self.fix_gain[k]
                d.qfrc_applied[self.dadr[k]] += kf * (self.hold[k] - q[k]) - df * qd[k] + d.qfrc_bias[self.dadr[k]]
        if self.bal_k_true:
            k = ARM.index(self.bal.get("joint", "pitch"))
            d.qfrc_applied[self.dadr[k]] += self.bal_sign * self.bal_k_true * math.sin(q[k])
        d.qfrc_applied[self.fa_d] = 200.0 * (0.03 - d.qpos[self.fa]) - 5.0 * d.qvel[self.fa_d] + d.qfrc_bias[self.fa_d]
        mj.mj_step(m, d)

    def step(self, dt):
        """One control period: apply what the drives were told, integrate, then the drives answer with the new state."""
        t_step = self.clock()
        self.handle_frames(lookahead=dt - 1e-9)
        n = max(1, int(round(dt / self.env.m.opt.timestep)))
        for _ in range(n):
            q = np.array([self.env.d.qpos[a] for a in self.qadr])
            qd = np.array([self.env.d.qvel[a] for a in self.dadr])
            tau = np.zeros(6)
            for mt in self.motors:
                k = ARM.index(mt.name)
                mt._last_p = mt.motor_pos(q[k])
                tau[k] = mt.torque(q[k], qd[k], self.clock())
            self._physics(tau)
            self.clock.t += self.env.m.opt.timestep
        self.io.update()
        if "drift" in self.faults:                            # synthetic degradation: Coulomb +25 %/K of motor temperature rise
            for mt in self.motors:
                k = ARM.index(mt.name)
                self.env.m.dof_frictionloss[self.dadr[k]] = self.act[mt.name]["coulomb_nm"] * (1 + 0.25 * max(mt.temp - 32.0, 0.0))
        q = np.array([self.env.d.qpos[a] for a in self.qadr])
        qd = np.array([self.env.d.qvel[a] for a in self.dadr])
        for mt in self.motors:
            k = ARM.index(mt.name)
            mt._last_p = mt.motor_pos(q[k])
            meta = mt.pending_t if mt.want_reply else None
            mt.feedback(q[k], qd[k], self._send(mt.bus, meta, base=max(getattr(mt, "t_arr", t_step), t_step)))
            if meta is not None:
                mt.pending_t = None
        for bn in self.buses:
            self._flush_host(bn)

    def idle(self, seconds):
        """Time passes with the host silent (a hung control loop)."""
        for _ in range(int(round(seconds / 0.002))):
            self.step(0.002)

    def bus_stats(self):
        rt = np.array(self.rt_log) * 1000 if self.rt_log else np.zeros(1)
        return dict(buses={bn: imp.stats() for bn, imp in self.imp.items()},
                    round_trip_ms=dict(n=len(self.rt_log), mean=round(float(rt.mean()), 3), p50=round(float(np.percentile(rt, 50)), 3),
                                       p95=round(float(np.percentile(rt, 95)), 3), p99=round(float(np.percentile(rt, 99)), 3),
                                       max=round(float(rt.max()), 3)))

    def truth(self):
        """Hidden values for the tests: the true zero of every motor in the joint frame, and its actuator model."""
        return {mt.name: dict(offset_joint=mt.sign_true * mt.offset_m / mt.ratio, sign=mt.sign_true, kt_err=mt.kt_err,
                              **{k: v for k, v in self.act[mt.name].items() if k != "kt_scale"}) for mt in self.motors}

    def close(self):
        for b in self.buses.values():
            try:
                b.shutdown()
            except Exception:
                pass


_SPEC_KEYS = ("fixed_ms", "jitter", "tail", "max_ms", "p_drop", "p_dup", "p_crc", "p_corrupt", "bus_off_at", "bus_off_ms")


def _joints():
    import json
    from pathlib import Path
    return json.loads((Path(__file__).resolve().parent / "description" / "joints.json").read_text())


def balancer_sign(m, qadr, dadr):
    """Sign that makes K·sin(pitch) oppose gravity in this model (the balancer carries the hanging-arm moment)."""
    import mujoco
    d = mujoco.MjData(m)
    d.qpos[qadr[0]] = 0.3
    mujoco.mj_forward(m, d)
    return 1.0 if d.qfrc_bias[dadr[0]] > 0 else -1.0
