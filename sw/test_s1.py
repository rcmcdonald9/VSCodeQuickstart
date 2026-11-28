"""S1 software tests that need no hardware.  python -m sw.test_s1

1. MIT frames: pack → unpack round trip within one quantisation step; special frames.
2. Safety supervisor against the bench twin with injected faults: a wild policy (random targets far outside the
   limits and the workspace), a stalled loop (watchdog), an e-stop, an over-temperature report. The arm must never leave
   its limits or the workspace box, and every fault must stop it within one cycle.
3. Virtual-bus round trip for the DM-J4310 driver.
4. People-near mode (D25 / R17): the shoulder-roll soft limit while a person is close. A random policy with the mode on
   never puts the roll below the limit; switching the mode on with the arm inside the restricted zone walks it out
   slowly (no stop, heartbeat kept); the trigger turns on within 1.5 m, fails safe without perception, and releases
   with hysteresis.
5. Collision detection (D23 / R17, sw/collision.py): the bench twin as the plant with the 20 lb payload in hand and model
   errors; no false trips over random motion, every injected contact trips within 20 ms, then a compliant hold.
6. Battery-current budget (W8a, sw/power.py): the model's constants match the electrical track; a random high-torque
   policy (peak torque allowed) never predicts more than 70 A of battery current; under sustained high-speed high-torque
   demand the 30 s mean stays ≤ 28 A and settles just under it, then recovers when the demand stops; normal motion on
   the bench twin gets exactly the same commands as with the budget off; the R2d basket carry (both arms, 4.54 kg each,
   120 s) is neither capped nor derated; nothing stops the arm; BMS over-current / under-voltage flags latch a fault.
7. SW-T1 person within reach (A10 v2, rnd/tip.py), the same people-near mode as 4.: a random arm policy and random base
   commands with the mode on never put the hand outside the SW-T1 box or the base over 0.1 m/s / 0.1 m/s²; with load mode
   on too the box is the intersection and the lower limits win; > 2.5 kg with a person near stops the base; entering with
   the hand outside the box walks it in (the box distance never grows, no stop); a person appearing during a fast transit
   brakes the base at the transit's accel limit down to 0.1 m/s, then holds the SW-T1 limits.
8. SW-T2 caster lock + SW-T4 threshold speed: locked by default; released only on request for a threshold crossing
   (speed ≤ 0.12 m/s) or unloaded transit; refused / forced locked with a person near, a fault or too much payload;
   re-lock, every stop and every 0.3 m of locked travel give a 0.1 s re-seat pulse; accel ≤ 0.5 m/s² while not locked.
"""
from __future__ import annotations

import math
import os

import numpy as np


def test_mit():
    from sw.drivers.mit import Ranges, pack_cmd, unpack_cmd, pack_feedback, unpack_feedback, ENABLE
    r = Ranges()
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(2000):
        x = (rng.uniform(-12, 12), rng.uniform(-29, 29), rng.uniform(0, 500), rng.uniform(0, 5), rng.uniform(-9.9, 9.9))
        y = unpack_cmd(pack_cmd(*x, r), r)
        steps = (2 * r.p_max / 65535, 2 * r.v_max / 4095, r.kp_max / 4095, r.kd_max / 4095, 2 * r.t_max / 4095)
        worst = max(worst, max(abs(a - b) / s for a, b, s in zip(x, y, steps)))
    fb = unpack_feedback(pack_feedback(5, 0, 1.234, -2.5, 3.3, 41, 44, r), r)
    assert fb["id"] == 5 and abs(fb["p"] - 1.234) < 1e-3 and fb["t_rotor"] == 44
    assert ENABLE[-1] == 0xFC and len(ENABLE) == 8
    assert worst <= 0.51, worst
    return f"MIT pack/unpack: worst error {worst:.2f} quantisation steps over 2000 random commands ✅"


class VirtualBus:
    def __init__(self):
        self.sent, self.inbox = [], {}

    def send(self, can_id, data):
        self.sent.append((can_id, bytes(data)))

    def recv_for(self, can_id):
        return self.inbox.pop(can_id, None)


def test_driver():
    from sw.drivers.gim import DMJ4310
    from sw.drivers.mit import pack_feedback, unpack_cmd
    bus = VirtualBus()
    d = DMJ4310(bus, 5)
    d.enable(); d.command(0.5, 0.0, 20.0, 1.0, 0.2)
    assert bus.sent[0][1][-1] == 0xFC
    p, v, kp, kd, t = unpack_cmd(bus.sent[1][1])
    assert abs(p - 0.5) < 1e-3 and abs(kp - 20) < 0.2
    bus.inbox[5] = pack_feedback(5, 0, 0.49, 0.01, 0.2, 35, 37)
    s = d.poll()
    assert abs(s["p"] - 0.49) < 1e-3 and s["temp"] == 37
    return "DM-J4310 driver on a virtual bus: enable, command, feedback ✅"


def test_supervisor():
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from sw.bench_env import BenchEnv, TABLE_Z
    from sw.safety import Supervisor, WATCHDOG_S, FB_MAX_AGE
    env = BenchEnv(render=False)
    env.reset(0)
    d2 = mujoco.MjData(env.m)

    def fk(q):
        d2.qpos[:] = env.d.qpos
        for k, a in enumerate(env.qadr):
            d2.qpos[a] = q[k]
        mujoco.mj_kinematics(env.m, d2)
        return d2.site_xpos[env.site].copy()

    t = [0.0]
    sup = Supervisor(fk=fk, clock=lambda: t[0], ws_lo=(0.05, -0.5, TABLE_Z + 0.01), ws_hi=(0.85, 0.5, TABLE_Z + 0.7))
    q0 = np.array([env.d.qpos[a] for a in env.qadr])
    v0 = np.zeros(6)
    assert sup.reset(q0, 0.002)
    rng = np.random.default_rng(1)
    viol = torque_viol = 0
    n = 3000
    qm = q0.copy()
    for k in range(n):
        t[0] += 0.002
        sup.tick()
        wild = q0 + rng.normal(0, 3.0, 6)                           # a policy gone mad: wild targets, gains and torques
        kp, kd, tf = rng.uniform(0, 200, 6), rng.uniform(0, 10, 6), rng.normal(0, 40, 6)
        vm = rng.normal(0, 0.5, 6)
        v = sup.check(wild, kp, kd, tf, qm, vm, 0.002)
        assert not v.stop, v.reasons
        tau = Supervisor.drive_torque(v, qm, vm)
        if np.any(np.abs(tau) > sup.rated + 1e-6):
            torque_viol += 1
        p = fk(v.q)
        if np.any(v.q < sup.lo - 1e-9) or np.any(v.q > sup.hi + 1e-9) or np.any(p < sup.ws_lo - 1e-6) or np.any(p > sup.ws_hi + 1e-6):
            viol += 1
        qm = v.q + rng.normal(0, 0.002, 6)                         # the arm follows (2 mrad noise)
    assert viol == 0 and torque_viol == 0, (viol, torque_viol)
    # faults: each stops on the very next check and stays latched
    t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm * np.nan, v0, 0.002).stop          # NaN feedback
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0, 0.002).stop                   # still latched
    assert not sup.heartbeat_ok()
    assert sup.reset(qm, 0.002)
    t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0, FB_MAX_AGE * 2).stop           # stale feedback
    assert sup.reset(qm, 0.002)
    t[0] += WATCHDOG_S + 0.001
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0, 0.002).stop                   # stalled loop
    sup.press_estop()
    assert not sup.reset(qm, 0.002)                                                                 # button still pressed
    sup.release_estop()
    assert sup.reset(qm, 0.002)
    t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0, 0.002, temp=[40, 41, 90, 40, 40, 40]).stop
    assert sup.reset(qm, 0.002); t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0, 0.002, err=[0, 0, 0, 1, 0, 0]).stop
    assert sup.reset(qm, 0.002); t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm, v0 + 5.0, 0.002).stop             # measured overspeed
    assert sup.reset(qm, 0.002); t[0] += 0.002; sup.tick()
    assert sup.check(qm, 20 * np.ones(6), np.ones(6), None, qm + 0.5, v0, 0.002).stop             # tracking error
    # load mode (R2d): with a 4.5 kg basket half, targets outside the basket box are rejected and braking is capped
    assert sup.reset(qm, 0.002)
    p0 = fk(qm)                                                    # bench frame: a load box around the current hand position
    sup.set_payload(4.54, box=(p0 - [0.12, 0.15, 0.10], p0 + [0.12, 0.15, 0.10]))
    assert sup.base_accel_limit() == 0.3 and sup.load_heading_ok(10) and not sup.load_heading_ok(45)
    load_viol = 0
    for k in range(800):
        t[0] += 0.002
        sup.tick()
        v = sup.check(q0 + rng.normal(0, 3.0, 6), 30 * np.ones(6), np.ones(6), None, qm, v0, 0.002)
        assert not v.stop, v.reasons
        p = fk(v.q)
        if np.any(p < sup.ws_lo - 1e-6) or np.any(p > sup.ws_hi + 1e-6):
            load_viol += 1
        qm = v.q.copy()
    assert load_viol == 0, load_viol
    sup.set_payload(0)
    assert sup.base_accel_limit() == 1.0
    return (f"Load mode (R2d): with 4.54 kg in hand the hand stays in the load box over 800 random cycles (robot default: ≤ 0.45 m out, ≤ 0.80 m up), braking capped at 0.3 m/s², "
            f"load heading checked against the wheel axes ✅. "
            f"Safety supervisor v2 vs a random policy (random targets, gains 0–200 N·m/rad, τ_ff σ 40 N·m): {n} cycles, 0 limit/workspace "
            f"violations, **0 cycles where the drive torque kp·e + kd·ė + τ_ff exceeds the rated torque**. NaN feedback, stale feedback, watchdog, "
            f"e-stop (reset refused while pressed), over-temperature, motor error, overspeed and tracking error each stop on the next cycle and stay latched ✅ "
            f"(overrides: {sup.counts})")


def test_people_near():
    """D25: people-near shoulder-roll soft limit (sw/safety.py set_people_near) against the bench twin."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from sw.bench_env import BenchEnv, TABLE_Z
    from sw.safety import PN_EXIT_SPEED, PN_HOLD_S, PeopleNearTrigger, Supervisor
    env = BenchEnv(render=False)
    env.reset(0)
    d2 = mujoco.MjData(env.m)

    def fk(q):
        d2.qpos[:] = env.d.qpos
        for k, a in enumerate(env.qadr):
            d2.qpos[a] = q[k]
        mujoco.mj_kinematics(env.m, d2)
        return d2.site_xpos[env.site].copy()

    t, dt = [0.0], 0.002
    sup = Supervisor(fk=fk, clock=lambda: t[0], ws_lo=(0.05, -0.5, TABLE_Z + 0.01), ws_hi=(0.85, 0.5, TABLE_Z + 0.7))
    k = sup.names.index("roll")
    soft = sup.pn_lo[k] - sup._margin                                 # the people-near limit itself (before the margin)
    assert math.isfinite(soft) and soft > sup.lo[k], "joints.json has no people-near roll range (python -m sw.export_description)"
    q0 = np.array([env.d.qpos[a] for a in env.qadr])
    q0[k] = math.radians(-10.0)                                       # the arm deep in the restricted zone
    assert sup.reset(q0, 0.002)
    v0 = np.zeros(6)
    qm = q0.copy()
    rng = np.random.default_rng(5)
    for _ in range(50):                                               # normal running near the stop
        t[0] += dt; sup.tick()
        v = sup.check(q0, 20 * np.ones(6), np.ones(6), None, qm, v0, dt)
        qm = v.q.copy()
    # 1. mode switch with the arm in the zone: walked out at PN_EXIT_SPEED, never stopped, the policy pushing deeper
    sup.set_people_near(True, box=(sup.ws_lo, sup.ws_hi))           # bench frame: SW-T1's hand box is tested in test_person_reach
    r_prev, steps, max_rate = qm[k], 0, 0.0
    while sup.people_near_exiting() or qm[k] < sup.pn_lo[k] - 1e-9:
        t[0] += dt; sup.tick()
        want = qm.copy(); want[k] = math.radians(-15.0)              # a policy still asking for the stop
        v = sup.check(want, 20 * np.ones(6), np.ones(6), None, qm, v0, dt)
        assert not v.stop and sup.heartbeat_ok(), v.reasons
        assert v.q[k] >= r_prev - 1e-12, "people-near exit moved the roll deeper into the zone"
        max_rate = max(max_rate, (v.q[k] - r_prev) / dt)
        r_prev = v.q[k]
        qm = v.q + rng.normal(0, 0.002, 6) * (np.arange(6) != k)
        qm[k] = v.q[k]
        steps += 1
        assert steps < 5000, "people-near exit did not finish"
    t_exit = steps * dt
    t_expect = (sup.pn_lo[k] - math.radians(-10.0)) / PN_EXIT_SPEED
    # the noisy policy re-commands the 2 mrad measurement jitter on the other joints: the people-near hand-speed cap (13.)
    # scales those steps, roll included, so the exit can take longer than the bare PN_EXIT_SPEED time, never shorter
    assert max_rate <= PN_EXIT_SPEED + 1e-6 and t_expect - 0.05 < t_exit < 1.5 * t_expect, (max_rate, t_exit, t_expect)
    assert abs(t_exit - t_expect) < 0.05 or sup.counts.get("hand speed cap", 0) > 0, (t_exit, t_expect)
    # 2. random policy with the mode on: 0 cycles with the roll target or the arm below the limit
    below, n = 0, 3000
    for _ in range(n):
        t[0] += dt; sup.tick()
        wild = q0 + rng.normal(0, 3.0, 6)
        vm = rng.normal(0, 0.5, 6)
        v = sup.check(wild, rng.uniform(0, 200, 6), rng.uniform(0, 10, 6), rng.normal(0, 40, 6), qm, vm, dt)
        assert not v.stop, v.reasons
        if v.q[k] < soft or qm[k] < soft - 0.004:                   # measured: the arm follows with 2 mrad noise
            below += 1
        qm = v.q + rng.normal(0, 0.002, 6)
    assert below == 0, below
    # 3. leaving the mode restores the full range at once
    sup.set_people_near(False)
    lo, _ = sup.target_bounds()
    assert lo[k] == sup.lo[k] and not sup.people_near_exiting()
    # 4. trigger: on within 1.5 m, fail-safe without perception, off after 2 s clear beyond 1.8 m
    tc = [0.0]
    trg = PeopleNearTrigger(clock=lambda: tc[0])
    assert trg.update(None)                                           # no perception yet: on
    seq = []
    for dist, dur in (([3.0], 3.0), ([1.4], 0.5), ([1.7], 3.0), ([2.5], 1.0), ([2.5], 1.5), (None, 1.0)):
        tt = 0.0
        while tt < dur - 1e-9:
            tc[0] += 0.1; tt += 0.1
            on = trg.update(dist if dist is None else list(dist))
        seq.append(on)
    # clear 3 s → off; 1.4 m → on; 1.7 m (inside the release radius) keeps it on; 2.5 m for 1 s still on, 2.5 s off; stale → on
    assert seq == [False, True, True, True, False, True], seq
    return (f"People-near mode (D25/R17): roll soft limit {math.degrees(soft):+.0f}° (target bound {math.degrees(sup.pn_lo[k]):+.0f}° with the margin). "
            f"Switched on with the roll at −10° and the policy still asking for −15°: walked out in {t_exit:.2f} s at ≤ {PN_EXIT_SPEED} rad/s, "
            f"no stop, heartbeat kept; then a random policy for {n} cycles: **0 cycles with the roll below the limit**; leaving the mode restores "
            f"the full range; the trigger (≤ {1.5} m on, > 1.8 m for {PN_HOLD_S:.0f} s off, no or stale perception = on) ✅ "
            f"(overrides: { {a: b for a, b in sup.counts.items() if 'people' in a} })")


class CollisionRig:
    """Bench twin as the plant for collision tests (D23 / R17): gravity on (no gravcomp: the motors carry it), arm actuators
    replaced by applied joint torques (the drive's MIT law), the payload in hand, and model errors the twin doesn't know:
    payload mass +5 % and COM 10 mm off, Coulomb friction 0.3 N·m on the GIM joints (0.05 on the wrist), torque-constant
    error ±4 % per joint, current noise σ = 1 % of rated, encoder noise 0.5 mrad, velocity noise 0.01 rad/s."""
    DT = 0.002
    PAYLOAD_ERR, COM_ERR, FRICTION, KT_ERR, NOISE_REL = 1.05, 0.010, (0.3, 0.05), 0.04, 0.01   # plant vs twin (ESTIMATEs)
    SIG_Q = 5e-4                                               # rad, joint position noise (ESTIMATE)
    # velocity from a 14-bit motor-side encoder differenced at 500 Hz: σ = (2π/2¹⁴)/N/√6/dt at the joint (N = 8 × belt for
    # the GIMs, 10 for the DM-J4310); doubled for margin (ESTIMATE until the drives' velocity readout is logged)
    SIG_V = 2 * 2 * np.pi / 2 ** 14 / np.array([24.0, 35.6, 16.0, 16.0, 10.0, 345.0]) / np.sqrt(6) / 0.002
    Q_C = np.array([-0.2, 0.3, 0.0, -1.3, 0.0, 0.0])         # carry pose, load held close and hanging (R2c/D21), inside the bench twin's peak torques
    AMP = np.array([0.15, 0.20, 0.60, 0.20, 0.05, 0.60])
    KP = np.array([60.0, 60.0, 40.0, 40.0, 15.0, 5.0])
    KD = np.array([3.0, 3.0, 2.0, 2.0, 0.5, 0.2])
    THR = np.array([3.2, 2.4, 1.6, 2.4, 2.0, np.inf])         # N·m; wrist roll (STS3215) not monitored
    REL = np.array([0.10, 0.10, 0.10, 0.10, 0.10, 0.0])       # Kt error + payload-mass error both scale with the torque

    def __init__(self, payload=4.536, seed=0):
        os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco
        from sw.bench_env import BenchEnv, xml
        from sw.collision import MujocoTwin, CollisionDetector, add_point_mass
        from sw.safety import Supervisor
        self.mj = mujoco
        env = BenchEnv(render=False)
        env.reset(0)
        self.env, m = env, env.m
        self.rng = np.random.default_rng(seed)
        m.body_gravcomp[:] = 0
        for g in ("table",):                                       # free space: the carried load hangs below the table top
            gi = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            m.geom_contype[gi] = m.geom_conaffinity[gi] = 0
        m.actuator_gainprm[:6, 0] = 0
        m.actuator_biasprm[:6, :] = 0
        hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand")
        self.fore = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "fore")
        grasp = m.site_pos[env.site].copy()
        add_point_mass(m, hand, payload * self.PAYLOAD_ERR, grasp + [self.COM_ERR, 0.0, 0.0])
        self.dadr, self.qadr = env.dadr, env.qadr
        for k, a in enumerate(self.dadr):
            m.dof_frictionloss[a] = self.FRICTION[0] if k < 4 else self.FRICTION[1]
        tm = mujoco.MjModel.from_xml_string(xml())
        tm.body_gravcomp[:] = 0
        self.twin = MujocoTwin(tm, self.qadr, self.dadr, payload_body=hand, payload_pos=grasp)
        self.det = CollisionDetector(self.twin, self.THR, self.REL)
        self.sup = Supervisor(clock=lambda: self.t, collision=self.det)
        self.sup.set_payload(payload)
        self.kt_err = 1 + self.rng.uniform(-self.KT_ERR, self.KT_ERR, 6)
        self.noise = self.NOISE_REL * self.sup.rated
        self.freq = self.rng.uniform(0.15, 0.45, (6, 2))
        self.phase = self.rng.uniform(0, 2 * np.pi, (6, 2))
        self.t = 0.0
        d = env.d
        for k, a in enumerate(self.qadr):
            d.qpos[a] = self.ref(0.0)[0][k]
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.tau_meas = None
        q0 = np.array([d.qpos[a] for a in self.qadr])
        assert self.sup.reset(q0, 0.002)
        self.shadow, self.shadow_v = None, None        # optional second supervisor fed the same inputs (test_power)
        self.bms = None                                # optional BMS reading passed to the supervisor (test_power)

    def ref(self, t):
        w = 2 * np.pi * self.freq
        s = np.sin(w * t + self.phase)
        q = self.Q_C + self.AMP * s.mean(1)
        qd = self.AMP * (w * np.cos(w * t + self.phase)).mean(1)
        qdd = self.AMP * (-w * w * s).mean(1)
        # the hand hangs (load axial through the wrist, as in the D21 basket grasp): wrist pitch ≈ −(pitch + elbow)
        q[4] = -(q[0] + q[3]) - 0.05 + q[4] - self.Q_C[4]
        qd[4] = -(qd[0] + qd[3]) + qd[4]
        qdd[4] = -(qdd[0] + qdd[3]) + qdd[4]
        return q, qd, qdd

    def step(self, contact_force=None, contact_tau=None):
        m, d, mj = self.env.m, self.env.d, self.mj
        self.t += self.DT
        self.sup.tick()
        qt = np.array([d.qpos[a] for a in self.qadr]); vt = np.array([d.qvel[a] for a in self.dadr])
        qm = qt + self.rng.normal(0, self.SIG_Q, 6); vm = vt + self.rng.normal(0, 1, 6) * self.SIG_V
        q, qd, qdd = self.ref(self.t)
        tff = self.twin.inverse(q, qd, qdd) + self.KD * qd
        v = self.sup.check(q, self.KP, self.KD, tff, qm, vm, 0.002, allow_peak=True, tau_meas=self.tau_meas, bms=self.bms)
        if self.shadow is not None:
            self.shadow.tick()
            self.shadow_v = self.shadow.check(q, self.KP, self.KD, tff, qm, vm, 0.002, allow_peak=True)
        if v.stop:
            tau = np.zeros(6)
        else:
            from sw.safety import Supervisor
            tau = Supervisor.drive_torque(v, qm, vm)
        d.qfrc_applied[:] = 0
        for k, a in enumerate(self.dadr):
            d.qfrc_applied[a] = tau[k] + (0.0 if contact_tau is None else contact_tau[k])
        d.xfrc_applied[:] = 0
        if contact_force is not None:
            d.xfrc_applied[self.fore, :3] = contact_force
        mj.mj_step(m, d)
        self.tau_meas = tau * self.kt_err + self.rng.normal(0, 1, 6) * self.noise
        return v


def test_collision():
    """D23 / R17: current-based collision detection against the bench twin with the 20 lb (4.54 kg) payload in hand."""
    n_free, seeds = 3000, (0, 1, 2)
    false_trips, peak = 0, 0.0
    for sd in seeds:                                   # 1. normal random motion: no trips
        rig = CollisionRig(seed=sd)
        for _ in range(n_free):
            v = rig.step()
            assert not v.stop, v.reasons
        false_trips += int(rig.det.tripped)
        peak = max(peak, rig.det.peak_ratio)
    assert false_trips == 0, false_trips
    # 2. contacts at random moments: a push on the forearm (60 N, random horizontal direction) and a 4 N·m joint torque
    lat_f, lat_t, tau_inj = [], [], []
    for k in range(8):
        rig = CollisionRig(seed=100 + k)
        for _ in range(int(rig.rng.integers(300, 900))):
            rig.step()
        a = rig.rng.uniform(0, 2 * np.pi)
        F = 60.0 * np.array([np.cos(a), np.sin(a), 0.0])
        n = 0
        while not rig.det.tripped and n < 100:
            rig.step(contact_force=F); n += 1
        lat_f.append(n * rig.DT)
        q_trip = np.array([rig.env.d.qpos[a_] for a_ in rig.qadr])
        for _ in range(250):                            # contact gone: the arm holds compliantly, no stop, no fall
            v = rig.step()
            assert v.compliant and not v.stop, v.reasons
            assert rig.sup.heartbeat_ok(), "compliant hold must keep the hardware heartbeat (Fable C2)"
        drift = float(np.max(np.abs(np.array([rig.env.d.qpos[a_] for a_ in rig.qadr]) - q_trip)))
        assert drift < 0.3, drift
        assert rig.sup.reset(np.array([rig.env.d.qpos[a_] for a_ in rig.qadr]), 0.002)
    for j in range(5):
        rig = CollisionRig(seed=200 + j)
        for _ in range(500):
            rig.step()
        tau = np.zeros(6); tau[j] = 2 * (rig.THR[j] + rig.REL[j] * abs(rig.tau_meas[j]))    # twice the trip level at that moment
        tau_inj.append(tau[j])
        n = 0
        while not rig.det.tripped and n < 100:
            rig.step(contact_tau=tau); n += 1
        lat_t.append(n * rig.DT)
    worst = max(lat_f + lat_t)
    assert worst <= 0.020, (lat_f, lat_t)
    return (f"Collision detection (D23/R17, momentum observer vs the twin's model torque, 4.54 kg payload, model errors: payload +5 %, "
            f"COM 10 mm, friction, Kt ±4 %, current noise 1 %): **0 false trips over {len(seeds)} × {n_free} cycles** of random motion "
            f"(peak residual {peak:.0%} of the threshold); a 60 N forearm push trips in {1000 * min(lat_f):.0f}–{1000 * max(lat_f):.0f} ms "
            f"and a joint torque of twice the trip level ({min(tau_inj):.1f}–{max(tau_inj):.1f} N·m) on each monitored joint in {1000 * min(lat_t):.0f}–{1000 * max(lat_t):.0f} ms (limit 20 ms); after a trip the arm holds "
            f"compliantly (no stop, drift < 0.3 rad over 0.5 s) until reset ✅")


def _kin_run(sup, n, dt, t, policy, rng, qm, allow_peak, shadow=None, bms=None, each=None):
    """Drive a supervisor for n cycles on a kinematic plant (the arm follows the verdict with 2 mrad noise)."""
    from sw.safety import Supervisor
    for _ in range(n):
        t[0] += dt
        sup.tick()
        qw, kp, kd, tf, vm = policy(qm)
        v = sup.check(qw, kp, kd, tf, qm, vm, dt, allow_peak=allow_peak, bms=bms)
        assert not v.stop and sup.heartbeat_ok(), v.reasons
        vs = None
        if shadow is not None:
            shadow.tick()
            vs = shadow.check(qw, kp, kd, tf, qm, vm, dt, allow_peak=allow_peak)
        if each is not None:
            each(v, vs, qm, vm, Supervisor.drive_torque(v, qm, vm))
        qm = v.q + rng.normal(0, 0.002, sup.n)
    return qm


def test_power():
    """W8a: battery-current cap (70 A peak, 28 A over 30 s) in the supervisor (sw/power.py)."""
    import json
    from pathlib import Path
    import yaml
    from rnd.electrical import MOTOR_CU, SW_PACK, branches
    from sw.power import ACTUATORS, I_AVG, I_PEAK, REST_A, T_AVG, BmsReading, CurrentModel
    from sw.safety import DESC, Supervisor
    # 0. constants: the electrical track's caps, the non-arm branches, the actuators' torque constants
    assert (I_PEAK, I_AVG, T_AVG) == (SW_PACK["i_peak"], SW_PACK["i_avg"], SW_PACK["t_avg"])
    rest_rnd = sum(r[1] for r in branches() if " arm" not in r[0])
    assert abs(REST_A - rest_rnd) < 0.15, (REST_A, rest_rnd)
    Y = yaml.safe_load((Path(__file__).resolve().parents[1] / "rnd" / "specs" / "actuators.yaml").read_text())
    g, dm, st = Y["GIM6010-8"], Y["DM-J4310-2EC_V1.1"], Y["STS3215-C018"]
    kt_yaml = {"GIM6010-8": g["rated_torque_nm"] / g["continuous_current_a"],
               "DM-J4310-2EC_V1.1": min(dm["rated_torque_nm"] / dm["current_a"]["nominal"], dm["peak_torque_nm"] / dm["current_a"]["peak"]),
               "STS3215-C018": st["stall_torque_nm"] / st["stall_current_a"], "GIM8108-36": MOTOR_CU["GIM8108-36"][0]}
    for name, kt in kt_yaml.items():
        assert abs(ACTUATORS[name][0] - kt) < 1e-9, (name, ACTUATORS[name][0], kt)
    assert abs(ACTUATORS["STS3215-C018"][4] - st["idle_current_a"]) < 1e-9
    assert abs(ACTUATORS["STS3215-C018"][1] - 12.0 / st["stall_current_a"]) < 1e-9          # brushed: R = V / I_stall
    for name in ("GIM6010-8", "GIM8108-36"):                                                 # phase R as the electrical track
        assert ACTUATORS[name][1] == MOTOR_CU[name][1], (name, ACTUATORS[name][1], MOTOR_CU[name][1])

    t = [0.0]
    dt = 0.002
    rng = np.random.default_rng(11)

    def fresh(**kw):
        s = Supervisor(clock=lambda: t[0], **kw)
        q0 = 0.5 * (s.lo + s.hi)
        assert s.reset(q0, 0.002)
        return s, q0

    # 1. random high-torque policy with peak torque allowed: never more than 70 A predicted
    sup, q0 = fresh()
    ref, _ = fresh(power=False)                          # the same commands without the budget: what the policy asks for
    v_sag = 18.0                                         # V: the bus at the 70 A cap, 50 % SoC (electrical.md), from the BMS
    model = CurrentModel(json.loads(DESC.read_text()), sup.rated).at(v_sag)

    def wild(qm):                                         # measured speeds up to 3.5 rad/s (V_FAULT is 4); τ_ff mostly along them
        vm = rng.choice([-1.0, 1.0], 6) * rng.uniform(1.0, 3.5, 6)
        tf = rng.uniform(0.5, 2.0, 6) * sup.peak * np.sign(vm) * np.where(rng.random(6) < 0.9, 1.0, -1.0)
        return q0 + rng.normal(0, 3.0, 6), rng.uniform(0, 200, 6), rng.uniform(0, 1, 6), tf, vm

    st1 = dict(i=0.0, ask=0.0, over=0, tq=0)

    def each1(v, vs, qm, vm, tau):
        i = REST_A + model.total(tau, vm)
        st1["i"] = max(st1["i"], i)
        st1["ask"] = max(st1["ask"], REST_A + model.total(Supervisor.drive_torque(vs, qm, vm), vm))
        st1["over"] += i > I_PEAK + 1e-6
        st1["tq"] += bool(np.any(np.abs(tau) > sup.peak + 1e-6))
        assert abs(sup.i_pred - i) < 1e-6, (sup.i_pred, i)

    n1 = 4000
    _kin_run(sup, n1, dt, t, wild, rng, q0.copy(), True, shadow=ref, bms=BmsReading(voltage_v=v_sag), each=each1)
    assert st1["over"] == 0 and st1["tq"] == 0 and sup.counts.get("current cap", 0) > 0, (st1, sup.counts)
    assert st1["ask"] > I_PEAK + 5.0, st1             # the policy really asks for more than the cap
    caps1 = sup.counts["current cap"]

    # 2. sustained high-speed, high-torque demand (every joint at 1.5–3.5 rad/s, pushed along its motion at twice its peak
    #    torque): 30 s mean ≤ 28 A throughout,
    #    settling just under it; then the demand stops and the budget recovers (ceiling back at 70 A)
    sup, q0 = fresh()
    dt2 = 0.004

    def push(qm):
        vm = rng.choice([-1.0, 1.0], 6) * rng.uniform(1.5, 3.5, 6)
        return q0 + rng.normal(0, 3.0, 6), np.full(6, 200.0), np.zeros(6), 2.0 * sup.peak * np.sign(vm), vm

    means, inst, ks, ceil = [], [], [], []

    def each2(v, vs, qm, vm, tau):
        means.append(sup.power.mean); inst.append(sup.i_pred); ks.append(sup.power.k); ceil.append(sup.power.ceiling)

    qm = _kin_run(sup, int(100.0 / dt2), dt2, t, push, rng, q0.copy(), True, each=each2)
    means, inst, ks = np.array(means), np.array(inst), np.array(ks)
    t_settle = 2 * T_AVG
    late = means[int(t_settle / dt2):]
    assert means.max() <= I_AVG + 1e-6, means.max()
    assert late.min() >= I_AVG - 2.5, late.min()                # converges to the budget, not far under it
    assert inst.max() <= I_PEAK + 1e-6 and sup.counts.get("power derate", 0) > 0
    t_burst = float(np.argmax(np.array(ceil) < I_PEAK - 1.0) * dt2)   # how long the 70 A ceiling lasted before the derate
    assert t_burst > 1.0 and inst[: int(t_burst / dt2)].max() > I_PEAK - 1.0, t_burst   # the burst was really used
    k_late = float(np.median(ks[int(t_settle / dt2):]))

    def light(qm):
        return qm.copy(), np.full(6, 20.0), np.ones(6), np.zeros(6), np.zeros(6)

    _kin_run(sup, int(40.0 / dt2), dt2, t, light, rng, qm, True)
    mean_after = sup.power.mean
    assert sup.power.ceiling == I_PEAK and mean_after < 0.6 * I_AVG, (sup.power.ceiling, mean_after)

    # 3. normal motion (bench twin, carry pose, no payload): exactly the commands it gets with the budget off
    rig = CollisionRig(payload=0.0, seed=3)
    rig.shadow = Supervisor(clock=lambda: rig.t, power=False)
    assert rig.shadow.reset(np.array([rig.env.d.qpos[a] for a in rig.qadr]), 0.002)
    diff, i_norm = 0, []
    n3 = int(32.0 / rig.DT)
    for _ in range(n3):
        v = rig.step()
        assert not v.stop and rig.sup.heartbeat_ok(), v.reasons
        w = rig.shadow_v
        same = all(np.array_equal(getattr(v, f), getattr(w, f)) for f in ("q", "kp", "kd", "t_ff"))
        diff += int(not same)
        i_norm.append(rig.sup.i_pred)
    assert diff == 0 and not {"current cap", "power derate"} & set(rig.sup.counts), (diff, rig.sup.counts)
    i_norm = np.array(i_norm)
    mean_norm = rig.sup.power.mean
    # 4. R2d basket carry: both arms, 4.54 kg in each hand, held 120 s (gentle sway in the carry pose). The other arm is
    #    the mirror of this one: the BMS reads the whole robot (rest + both arms), so this supervisor sees it in its budget.
    class CarryRig(CollisionRig):
        AMP = CollisionRig.AMP * 0.25

    rig = CarryRig(payload=4.536, seed=4)
    i_arm, means_c = [], []
    n4 = int(120.0 / rig.DT)
    for _ in range(n4):
        rig.bms = BmsReading(current_a=REST_A + 2 * rig.sup._arm_last, voltage_v=22.2)
        v = rig.step()
        assert not v.stop and rig.sup.heartbeat_ok(), v.reasons
        i_arm.append(rig.sup._arm_last)
        means_c.append(rig.sup.power.mean)
    i_arm, means_c = np.array(i_arm), np.array(means_c)
    assert not {"current cap", "power derate"} & set(rig.sup.counts), rig.sup.counts
    assert means_c.max() < I_AVG, means_c.max()
    i_robot = REST_A + 2 * i_arm

    # 5. BMS: over-current / under-voltage latch a fault (stop, heartbeat dropped) until reset; a high measured pack
    #    current (the base climbing) leaves the arm less room, and an unmeetable budget still never stops the arm
    sup, q0 = fresh()
    for flag in ("overcurrent", "undervoltage"):
        t[0] += dt; sup.tick()
        v = sup.check(q0, np.full(6, 20.0), np.ones(6), None, q0, np.zeros(6), dt, bms=BmsReading(current_a=12.0, voltage_v=22.0, **{flag: True}))
        assert v.stop and "BMS over-current or under-voltage" in v.reasons and not sup.heartbeat_ok()
        t[0] += dt; sup.tick()
        assert sup.check(q0, np.full(6, 20.0), np.ones(6), None, q0, np.zeros(6), dt, bms=BmsReading(current_a=12.0)).stop   # latched
        assert sup.reset(q0, 0.002)
    st4 = dict(i=0.0)

    def each4(v, vs, qm, vm, tau):
        st4["i"] = max(st4["i"], sup.i_pred)

    _kin_run(sup, 2000, dt, t, wild, rng, q0.copy(), True, bms=BmsReading(current_a=45.0, voltage_v=19.5), each=each4)
    assert st4["i"] <= I_PEAK + 1e-6, st4
    counts = {a: b for a, b in sup.counts.items() if a in ("current cap", "power derate", "BMS over-current or under-voltage")}
    return (f"Battery budget (W8a; battery current = electrical input power / V_bus per joint: k·R·I_ph² + τ·ω/η, braking offsets only its own copper loss, + idle + {REST_A:.1f} A for the rest of the robot): "
            f"a random high-torque policy (peak torque allowed, 1–3.5 rad/s, bus sagged to {v_sag:.0f} V) asks for up to {st1['ask']:.0f} A; **0 of {n1} cycles predict over {I_PEAK:.0f} A** "
            f"(max {st1['i']:.1f} A, capped in {caps1} cycles, feed-forward then gains). Sustained high-speed high-torque demand (1.5–3.5 rad/s, 2 × peak torque along the motion) for 100 s: 70 A allowed for the first {t_burst:.1f} s (burst), "
            f"then **the 30 s mean never exceeds {I_AVG:.0f} A** (max {means.max():.2f} A; {late.min():.1f}–{late.max():.1f} A after {t_settle:.0f} s, "
            f"torque-limit factor k ≈ {k_late:.2f}); demand stopped: back to {mean_after:.1f} A and the full 70 A ceiling in 40 s. "
            f"Normal motion on the bench twin ({n3 * rig.DT:.0f} s, carry pose, no payload): {i_norm.mean():.1f} A mean / {i_norm.max():.1f} A max, "
            f"30 s mean {mean_norm:.1f} A, **commands identical to the budget off** (0 caps, 0 derates). No stop and the heartbeat kept in every run; "
            f"BMS over-current and under-voltage latch a fault; with the BMS reading 45 A the arm still never predicts over 70 A and keeps running. "
            f"**R2d basket carry, both arms with 4.54 kg each held {n4 * rig.DT:.0f} s:** {i_arm.mean():.1f} A per arm "
            f"(max {i_arm.max():.1f} A), robot {i_robot.mean():.1f} A, 30 s mean ≤ {means_c.max():.1f} A: no cap, no derate ✅ (overrides: {counts})")


def _bench_sup():
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from sw.bench_env import BenchEnv, TABLE_Z
    from sw.safety import Supervisor
    env = BenchEnv(render=False)
    env.reset(0)
    d2 = mujoco.MjData(env.m)

    def fk(q):
        d2.qpos[:] = env.d.qpos
        for k, a in enumerate(env.qadr):
            d2.qpos[a] = q[k]
        mujoco.mj_kinematics(env.m, d2)
        return d2.site_xpos[env.site].copy()
    t = [0.0]
    sup = Supervisor(fk=fk, clock=lambda: t[0], ws_lo=(0.05, -0.5, TABLE_Z + 0.01), ws_hi=(0.85, 0.5, TABLE_Z + 0.7))
    q0 = np.array([env.d.qpos[a] for a in env.qadr])
    return sup, fk, t, q0


def test_person_reach():
    """SW-T1 (sw/safety.py set_people_near / set_person_near): hand box, base caps, combined with load mode."""
    from sw.safety import LOAD_WS, PERSON_A, PERSON_V, PERSON_WS, Supervisor
    dt = 0.002
    # 0. the robot-frame boxes: PERSON_WS = LOAD_WS with x ≤ 0.35, z ≤ 0.60; with load mode on too, the intersection
    assert PERSON_WS == (LOAD_WS[0], (0.35, LOAD_WS[1][1], 0.60))
    r = Supervisor(ws_lo=(0.0, -0.6, 0.0), ws_hi=(0.9, 0.6, 1.4), power=False)
    assert Supervisor.set_person_near is Supervisor.set_people_near
    r.set_payload(2.0)
    assert np.allclose(r.ws_hi, LOAD_WS[1]) and r.base_accel_limit() == 0.3
    r.set_person_near(True)
    assert np.allclose(r.ws_lo, PERSON_WS[0]) and np.allclose(r.ws_hi, PERSON_WS[1]) and r.base_accel_limit() == PERSON_A and r.base_speed_limit() == PERSON_V
    r.set_people_near(False)
    assert np.allclose(r.ws_hi, LOAD_WS[1]) and r.base_accel_limit() == 0.3
    r.set_payload(0)
    assert np.allclose(r.ws_hi, (0.9, 0.6, 1.4)) and r.base_accel_limit() == 1.0 and r.base_speed_limit() == 0.3

    sup, fk, t, q0 = _bench_sup()
    k = sup.names.index("roll")
    q0[k] = max(q0[k], sup.pn_lo[k] + 0.05)                          # start clear of the D25 roll zone (tested in 4.)
    assert sup.reset(q0, 0.002)
    p0 = fk(q0)
    # bench-frame boxes around the hand: SW-T1 box (close / low faces just past the hand) and a load box narrower in y
    PB = (p0 - [0.12, 0.25, 0.10], p0 + [0.02, 0.25, 0.02])
    LB = (p0 - [0.12, 0.15, 0.10], p0 + [0.12, 0.15, 0.10])
    rng = np.random.default_rng(11)
    v0 = np.zeros(6)

    def run(n, box, base_sigma=1.0):
        """Random arm policy + random base commands; returns (hand-outside cycles, base over-limit cycles)."""
        qm, vb, out, over = sup.last_q.copy(), sup._v_base.copy(), 0, 0
        for _ in range(n):
            t[0] += dt; sup.tick()
            v = sup.check(q0 + rng.normal(0, 3.0, 6), rng.uniform(0, 200, 6), rng.uniform(0, 10, 6), rng.normal(0, 40, 6), qm, rng.normal(0, 0.5, 6), dt)
            assert not v.stop and sup.heartbeat_ok(), v.reasons
            p = fk(v.q)
            if np.any(p < box[0] - 1e-6) or np.any(p > box[1] + 1e-6) or not sup.workspace_ok():
                out += 1
            vc = sup.limit_base(rng.normal(0, base_sigma, 2) * rng.choice([0.0, 1.0, 5.0]), dt)
            if np.hypot(*vc) > sup.base_speed_limit() + 1e-9 or np.hypot(*(vc - vb)) > sup.base_accel_limit() * dt + 1e-9:
                over += 1
            if sup.release_casters("threshold") or sup.release_casters("transit"):
                over += 1                                                # a release must be refused with a person near
            vb = vc
            qm = v.q + rng.normal(0, 0.002, 6)
        return out, over

    # 1. person near, no load: 0 cycles outside the SW-T1 box or over the base limits
    sup.set_person_near(True, box=PB)
    assert sup.base_speed_limit() == PERSON_V and sup.base_accel_limit() == PERSON_A and sup.casters_locked()
    n1 = 3000
    out1, over1 = run(n1, PB)
    assert out1 == 0 and over1 == 0, (out1, over1)
    # 2. load mode + person near: the intersection, the lower limits
    sup.set_payload(2.0, box=LB)
    want = (np.maximum.reduce([PB[0], LB[0], sup._ws_free[0]]), np.minimum.reduce([PB[1], LB[1], sup._ws_free[1]]))
    assert np.allclose(sup.ws_lo, want[0]) and np.allclose(sup.ws_hi, want[1]) and sup.base_accel_limit() == min(0.3, PERSON_A)
    n2 = 1500
    out2, over2 = run(n2, want)
    assert out2 == 0 and over2 == 0, (out2, over2)
    # heavier than 2.5 kg (R2d basket) with a person near: the base stops
    sup.set_payload(4.54, box=LB)
    assert sup.base_speed_limit() == 0.0
    for _ in range(200):
        t[0] += dt
        vc = sup.limit_base([0.3, 0.0], dt)
    assert np.hypot(*vc) < 1e-12
    sup.set_people_near(False)                                          # person gone: load-mode box and 0.3 m/s²
    assert np.allclose(sup.ws_hi, np.minimum(LB[1], sup._ws_free[1])) and sup.base_accel_limit() == 0.3
    sup.set_payload(0)
    assert np.allclose(sup.ws_hi, sup._ws_free[1]) and sup.base_accel_limit() == 1.0
    # 3. entering with the hand outside the box and the roll in the D25 zone: walked in, the distance to the box never grows,
    #    the roll still walked out at ≤ PN_EXIT_SPEED (held steps don't let the bound run ahead), no stop
    from sw.safety import PN_EXIT_SPEED, _box_dist
    qm = sup.last_q.copy()
    qm[k] = math.radians(-10.0)
    t[0] += dt; sup.tick()
    assert sup.reset(qm, 0.002)
    pq = fk(qm)
    PB2 = (pq - [0.20, 0.25, 0.20], pq + [-0.03, 0.25, -0.02])           # hand 30 mm beyond the x face, 20 mm above the top
    sup.set_people_near(True, box=PB2)
    d_start = d_prev = _box_dist(fk(qm), sup.ws_lo, sup.ws_hi)
    assert d_start > 0 and not sup.workspace_ok()
    grew, n3, inside_at, roll_rate = 0, 3000, None, 0.0
    for i in range(n3):
        t[0] += dt; sup.tick()
        want = qm + rng.normal(0, 0.3, 6)
        want[k] = math.radians(-15.0)                                     # the policy still asking for the roll stop
        v = sup.check(want, 30 * np.ones(6), np.ones(6), None, qm, v0, dt)
        assert not v.stop and sup.heartbeat_ok(), v.reasons
        dn = _box_dist(fk(v.q), sup.ws_lo, sup.ws_hi)
        grew += dn > d_prev + 1e-15
        roll_rate = max(roll_rate, abs(v.q[k] - qm[k]) / dt)
        d_prev = dn
        qm = v.q.copy()
        if dn == 0 and inside_at is None:
            inside_at = i * dt
    assert grew == 0 and d_prev < d_start and roll_rate <= PN_EXIT_SPEED + 1e-6, (grew, d_prev, d_start, roll_rate)
    sup.set_people_near(False)
    # 4. a person appears during a fast unloaded transit (casters released, 0.8 m/s): braked at the transit's 0.5 m/s², then SW-T1
    assert sup.release_casters("transit")
    a_tr = sup.base_accel_limit()
    vb = sup._v_base.copy()
    for _ in range(2500):
        t[0] += dt
        vb = sup.limit_base([0.8, 0.0], dt)
    assert abs(vb[0] - 0.8) < 1e-9
    sup.set_people_near(True, box=PB)
    assert sup.casters_locked()
    t_br, over4, prev = None, 0, vb.copy()
    for i in range(2500):
        t[0] += dt
        vc = sup.limit_base([0.8, 0.0], dt)
        dv = np.hypot(*(vc - prev))
        if t_br is None:
            over4 += dv > a_tr * dt + 1e-9 or np.hypot(*vc) > np.hypot(*prev) + 1e-12
            if np.hypot(*vc) <= PERSON_V + 1e-9:
                t_br = (i + 1) * dt
        else:
            over4 += np.hypot(*vc) > PERSON_V + 1e-9 or dv > PERSON_A * dt + 1e-9
        prev = vc
    assert over4 == 0 and t_br is not None and abs(t_br - (0.8 - PERSON_V) / a_tr) < 0.01, (over4, t_br)
    return (f"SW-T1 person within reach (one people-near mode; `set_person_near` = `set_people_near`): robot-frame box "
            f"x {PERSON_WS[0][0]:.2f}–{PERSON_WS[1][0]:.2f} m, z {PERSON_WS[0][2]:.2f}–{PERSON_WS[1][2]:.2f} m (LOAD_WS ∩ x ≤ 0.35, z ≤ 0.60). "
            f"Random arm policy + random base commands with the mode on: {n1} cycles, **0 cycles with the hand outside the box, 0 over "
            f"{PERSON_V} m/s or {PERSON_A} m/s²**, every caster release refused; with load mode on too (2 kg): {n2} cycles, 0 outside the "
            f"intersection of the two boxes, accel cap {min(0.3, PERSON_A)} m/s²; 4.54 kg with a person near → base held at 0 m/s. "
            f"Entering with the hand 30 mm outside and the roll at −10°: walked in{f' in {inside_at:.2f} s' if inside_at is not None else ''}, box distance never grew, "
            f"roll walked out at ≤ {roll_rate:.2f} rad/s (D25 {PN_EXIT_SPEED}), no stop. "
            f"Person appearing at 0.8 m/s (casters released for transit): casters locked at once, braked at {a_tr} m/s² to {PERSON_V} m/s in "
            f"{t_br:.2f} s, then held to the SW-T1 limits ✅")


def test_casters():
    """SW-T2 caster lock state machine + SW-T4 threshold speed (sw/safety.py)."""
    from sw.safety import (CASTER_LOCKED_V, CASTER_PULSE_S, CASTER_RELEASED_A, CASTER_RESEAT_M, THRESHOLD_SPEED, Supervisor)
    t, dt = [0.0], 0.002
    s = Supervisor(clock=lambda: t[0], power=False)
    q = np.clip(np.zeros(s.n), s.lo, s.hi)
    assert s.reset(q, 0.002)

    def drive(v, secs):
        out = None
        for _ in range(int(round(secs / dt))):
            t[0] += dt
            s.tick()
            out = s.limit_base(v, dt)
        return out

    def wait(secs):
        t[0] += secs
    # default: locked, solenoid off, 0.3 m/s, 1.0 m/s²; SW-T4 value
    assert s.casters_locked() and not s.caster_solenoid() and s.base_speed_limit() == CASTER_LOCKED_V and s.base_accel_limit() == 1.0
    assert Supervisor.threshold_speed() == s.threshold_speed() == THRESHOLD_SPEED == 0.12
    # threshold release: free, speed ≤ 0.12, accel ≤ 0.5 (0.3 loaded)
    assert s.release_casters("threshold")
    st = s.caster_state()
    assert st["state"] == "released" and st["mode"] == "threshold" and s.caster_solenoid() and not s.casters_locked()
    assert st["speed"] == THRESHOLD_SPEED and st["accel"] == CASTER_RELEASED_A
    vmax = max(np.hypot(*drive([0.3, 0.1], 0.2 * i)) for i in range(1, 8))
    assert vmax <= THRESHOLD_SPEED + 1e-9, vmax
    s.set_payload(2.0)
    assert not s.casters_locked() and s.base_accel_limit() == 0.3            # 2 kg: still a threshold release, load-mode accel
    # re-lock: a re-seat pulse, then locked
    n0 = s.counts.get("caster re-seat pulse", 0)
    s.lock_casters()
    assert s.caster_state()["state"] == "reseat" and s.caster_solenoid() and s.counts["caster re-seat pulse"] == n0 + 1
    wait(CASTER_PULSE_S * 0.5)
    assert not s.casters_locked()
    wait(CASTER_PULSE_S * 0.6)
    assert s.casters_locked() and not s.caster_solenoid()
    # transit: refused with a load; with none, 0.8 m/s; a payload ends it at once (no pulse)
    assert not s.release_casters("transit") and not s.release_casters("other")
    s.set_payload(0)
    assert s.release_casters("transit") and s.base_speed_limit() == 0.8
    s.set_payload(1.5)
    assert s.casters_locked() and s.counts.get("caster forced lock", 0) == 1
    s.set_payload(0)
    s.set_payload(3.0)
    assert not s.release_casters("threshold")                                # over the 2.5 kg a threshold release is analysed for
    s.set_payload(0)
    # person near: refused; a person appearing while released locks them
    s.set_people_near(True)
    assert not s.release_casters("threshold") and not s.release_casters("transit") and s.casters_locked()
    s.set_people_near(False)
    assert s.release_casters("threshold")
    s.set_people_near(True)
    assert s.casters_locked()
    s.set_people_near(False)
    # a fault locks them and stops the base; reset does not release them again
    drive([0.0, 0.0], 0.5)
    wait(1.0)
    assert s.release_casters("transit")
    drive([0.5, 0.0], 1.5)
    s.press_estop()
    assert s.casters_locked() and not s.caster_solenoid() and s.base_speed_limit() == 0.0 and not s.release_casters("threshold")
    vb = drive([0.5, 0.0], 2.0)
    assert np.hypot(*vb) < 1e-9
    s.release_estop()
    assert s.reset(q, 0.002) and s.casters_locked()
    wait(1.0)
    # re-seat pulse at every stop and every 0.3 m of locked travel
    s.tick()
    drive([0.0, 0.0], 0.2)
    wait(1.0)
    n0 = s.counts.get("caster re-seat pulse", 0)
    drive([0.2, 0.0], 0.5)                                                   # 0.1 m, then stop
    assert s.counts.get("caster re-seat pulse", 0) == n0
    drive([0.0, 0.0], 0.25)
    assert s.counts.get("caster re-seat pulse", 0) == n0 + 1
    drive([0.0, 0.0], 0.2)
    assert s.casters_locked()
    n1 = s.counts["caster re-seat pulse"]
    dist = 0.0
    for _ in range(int(3.0 / dt)):                                           # 3 s at the locked cap
        t[0] += dt
        s.tick()
        dist += np.hypot(*s.limit_base([0.3, 0.0], dt)) * dt
    pulses = s.counts["caster re-seat pulse"] - n1
    assert int(dist / (CASTER_RESEAT_M + 0.3 * CASTER_PULSE_S)) <= pulses <= int(dist / CASTER_RESEAT_M), (pulses, dist)
    return (f"SW-T2 caster lock: locked by default (solenoid off), {CASTER_LOCKED_V} m/s while locked; threshold release → "
            f"≤ {THRESHOLD_SPEED} m/s (SW-T4 `threshold_speed()`), ≤ {CASTER_RELEASED_A} m/s² (0.3 loaded); transit release only "
            f"unloaded (0.8 m/s), ended at once by a payload; releases refused and ended by a person near, a fault (base → 0 m/s) "
            f"or > 2.5 kg; re-lock, stopping and every {CASTER_RESEAT_M} m locked ({pulses} pulses over {dist:.2f} m) give a "
            f"{CASTER_PULSE_S * 1000:.0f} ms re-seat pulse ✅ (overrides: { {a: b for a, b in s.counts.items() if 'caster' in a or 'base' in a} })")


def test_hand_speed():
    """13. People-near hand-speed cap (R17 140 N first impact, rnd/hand_speed.py) and joint slew."""
    from sw.safety import HAND_LEVER, PN_JOINT_V, PERSON_V, V_HAND_PN, Supervisor
    dt = 0.002
    sup, fk, t, q0 = _bench_sup()
    k = sup.names.index("roll")
    q0[k] = max(q0[k], sup.pn_lo[k] + 0.05)
    assert sup.reset(q0, 0.002)
    rng = np.random.default_rng(13)
    sup.set_people_near(True, box=(sup.ws_lo, sup.ws_hi))
    worst, worst_j, n = 0.0, 0.0, 0
    for payload in (0.0, 1.0, 2.5, 4.54):
        sup.set_payload(payload, box=(sup.ws_lo, sup.ws_hi))
        cap = sup.hand_speed_cap()
        assert payload > V_HAND_PN[-1][0] or abs(cap - next(v for kg, v in V_HAND_PN if payload <= kg + 1e-9) + np.hypot(*sup._v_base)) < 1e-12
        qm = sup.last_q.copy()
        for _ in range(1500):
            t[0] += dt; sup.tick()
            p_last, q_last = fk(sup.last_q), sup.last_q.copy()
            v = sup.check(q0 + rng.normal(0, 2.0, 6), 30 * np.ones(6), np.ones(6), None, qm, np.zeros(6), dt)
            assert not v.stop, v.reasons
            sp = np.linalg.norm(fk(v.q) - p_last) / dt
            worst = max(worst, sp / cap)
            worst_j = max(worst_j, float(np.max(np.abs(v.q - q_last))) / dt)
            n += 1
            qm = v.q.copy()
    assert worst <= 1.0 + 1e-6 and worst_j <= PN_JOINT_V + 1e-9, (worst, worst_j)
    assert sup.counts.get("hand speed cap", 0) > 0
    # the base speed comes off the arm's share: at PERSON_V the empty-hand cap drops by 0.1 m/s
    sup.set_payload(0.0)
    sup._v_base = np.array([PERSON_V, 0.0])
    assert abs(sup.hand_speed_cap() - (V_HAND_PN[0][1] - PERSON_V)) < 1e-12
    sup._v_base = np.zeros(2)
    # off with nobody near: the full joint slew again
    sup.set_people_near(False)
    t[0] += dt; sup.tick()
    q_last = sup.last_q.copy()
    v = sup.check(q_last + 1.0, 30 * np.ones(6), np.ones(6), None, q_last, np.zeros(6), dt)
    assert np.max(np.abs(v.q - q_last)) / dt > PN_JOINT_V + 0.1
    # no fk: the joint-space bound Σ|Δq|·lever ≤ cap
    r = Supervisor(power=False, clock=lambda: t[0])
    qr = np.clip(np.zeros(6), r.lo, r.hi)
    qr[k] = r.pn_lo[k] + 0.1
    assert r.reset(qr, 0.002)
    r.set_people_near(True)
    lever = np.array([HAND_LEVER[nm] for nm in r.names])
    over = 0
    for _ in range(1000):
        t[0] += dt; r.tick()
        ql = r.last_q.copy()
        v = r.check(qr + rng.normal(0, 1.0, 6), 30 * np.ones(6), np.ones(6), None, ql, np.zeros(6), dt)
        over += float(np.sum(np.abs(v.q - ql) * lever)) / dt > r.hand_speed_cap() + 1e-9
    assert over == 0, over
    return (f"People-near hand-speed cap (R17 140 N, rnd/hand_speed.py): V_HAND_PN {V_HAND_PN} (payload kg, m/s) minus the base speed, "
            f"joint slew ≤ {PN_JOINT_V} rad/s. Random policy on the bench twin with 0 / 1 / 2.5 / 4.54 kg: {n} cycles, **worst hand speed "
            f"{worst:.3f} × the cap, worst joint rate {worst_j:.2f} rad/s**; at {PERSON_V} m/s of base speed the cap drops by {PERSON_V}; without "
            f"an fk the bound Σ|Δq|·lever holds over 1000 cycles; off with nobody near ✅")


def test_person_approach():
    """14./15. SW-T6 person approaching (A10 from the real states) and SW-T7 basket carry only with the area clear."""
    from sw.safety import (A_FREE, PERSON_A, PERSON_MAX_KG, PERSON_V, PERSON_WS, PN_PREP_RADIUS, PN_PREP_RELEASE, PeopleNearTrigger,
                           Supervisor)
    dt = 0.002
    t = [0.0]
    s = Supervisor(ws_lo=(0.0, -0.6, 0.0), ws_hi=(0.9, 0.6, 1.4), power=False, clock=lambda: t[0])
    assert s.release_casters("transit")
    for _ in range(2000):                                             # transit at 0.8 m/s
        t[0] += dt
        vb = s.limit_base([0.8, 0.0], dt)
    a_tr = s.base_accel_limit()
    s.set_person_approach(True)
    assert s.approaching() and not s.people_near and s.casters_locked()
    assert np.allclose(s.ws_hi, PERSON_WS[1]) and s.base_accel_limit() == PERSON_A and s.base_speed_limit() == PERSON_V
    k = s.names.index("roll")
    assert s.target_bounds()[0][k] == s.lo[k]                        # no D25 roll bound at the approach radius
    t_br = None
    for i in range(3000):
        t[0] += dt
        vc = s.limit_base([0.8, 0.0], dt)
        if t_br is None and np.hypot(*vc) <= PERSON_V + 1e-9:
            t_br = (i + 1) * dt
    assert t_br is not None and abs(t_br - (0.8 - PERSON_V) / a_tr) < 0.01, t_br
    assert not s.release_casters("threshold")
    s.set_people_near(True)                                           # closer: people-near on top
    s.set_people_near(False)                                          # back out to the approach zone: SW-T1 limits stay
    assert s.approaching() and s.base_speed_limit() == PERSON_V
    s.set_person_approach(False)
    assert not s.approaching() and s.base_accel_limit() == A_FREE and np.allclose(s.ws_hi, (0.9, 0.6, 1.4))
    # SW-T7: the basket (> PERSON_MAX_KG) moves only with the area clear and nobody approaching; default: not clear
    s.set_payload(PERSON_MAX_KG + 2.0)
    assert s.base_speed_limit() == 0.0
    s.set_area_clear(True)
    assert s.base_speed_limit() > 0.0
    s.set_person_approach(True)
    assert s.base_speed_limit() == 0.0
    s.set_person_approach(False)
    s.set_area_clear(False)
    assert s.base_speed_limit() == 0.0
    s.set_payload(PERSON_MAX_KG)                                      # 2.5 kg: travels without the clear-area input
    assert s.base_speed_limit() > 0.0
    # the approach trigger: a PeopleNearTrigger at the prep radius
    tc = [0.0]
    trg = PeopleNearTrigger(clock=lambda: tc[0], radius=PN_PREP_RADIUS, release=PN_PREP_RELEASE)
    assert trg.update(None) and trg.update([PN_PREP_RADIUS - 0.1])
    tc[0] += 0.1
    assert trg.update([PN_PREP_RELEASE + 0.5])
    tc[0] += 2.5
    trg.update([PN_PREP_RELEASE + 0.5])
    tc[0] += 0.1
    assert not trg.update([PN_PREP_RELEASE + 0.5])
    return (f"SW-T6 person approaching (≤ {PN_PREP_RADIUS} m, off beyond {PN_PREP_RELEASE} m): PERSON_WS box, casters locked, base braked "
            f"from 0.8 m/s at the transit's {a_tr} m/s² to {PERSON_V} m/s in {t_br:.2f} s, releases refused, no D25 roll bound; "
            f"people-near on top and off again keeps it. SW-T7: > {PERSON_MAX_KG} kg moves only with `set_area_clear(True)` and nobody "
            f"approaching (default: held at 0 m/s) ✅")


def test_roll_latch():
    """16. Roll latch (R17 safe power-off, rnd/power_off.py O2): off by default; solenoid only healthy + nobody near."""
    from sw.safety import LATCH_RETRACT_S, ROLL_GUARD, Supervisor
    dt = 0.002
    t = [0.0]
    off = Supervisor(power=False, clock=lambda: t[0])
    assert (ROLL_GUARD != "latch") == (not off.roll_latch) and (off.roll_latch or (not off.roll_latch_solenoid() and off.latch_state() is None))
    s = Supervisor(power=False, clock=lambda: t[0], roll_latch=True)
    k = s.names.index("roll")
    pin = s.pn_lo[k]                                                   # +5° + the 3° margin
    q = np.clip(np.zeros(6), s.lo, s.hi)
    q[k] = pin + 0.2
    assert s.reset(q, 0.002)
    assert not s.roll_latch_solenoid() or s.latch_state() == "retracting"

    def run(n, want_roll, qm, switch=None):
        lowest = 9.0
        for _ in range(n):
            t[0] += dt; s.tick()
            s.set_latch_switch(switch)
            w = qm.copy(); w[k] = want_roll
            v = s.check(w, 20 * np.ones(6), np.ones(6), None, qm, np.zeros(6), dt)
            assert not v.stop, v.reasons
            lowest = min(lowest, v.q[k])
            qm = v.q.copy()
        return qm, lowest
    # 1. no switch: retracting for LATCH_RETRACT_S, the roll held ≥ the pin bound meanwhile, then free
    qm, low = run(int(0.8 * LATCH_RETRACT_S / dt), s.lo[k], q)
    assert s.latch_state() == "retracting" and s.roll_latch_solenoid() and low >= pin - 1e-12, (s.latch_state(), low, pin)
    qm, low = run(int(1.0 / dt), s.lo[k], qm)
    assert s.latch_state() == "retracted" and low < pin - 0.1
    # 2. a person near: pin dropped at once; the bound never goes above the arm (it only stops it going lower)
    s.set_people_near(True)
    assert not s.roll_latch_solenoid() and s.latch_state() == "engaged"
    r0 = qm[k]
    qm, low = run(50, s.lo[k], qm)
    assert low >= r0 - 1e-12
    s.set_people_near(False)
    # 3. with a switch: retracted only when it confirms; a switch that disagrees later puts the bound back
    qm, _ = run(int(2.0 / dt), pin + 0.3, qm, switch=False)
    assert s.latch_state() == "retracting" and qm[k] >= pin - 1e-12
    qm, low = run(int(0.5 / dt), s.lo[k], qm, switch=True)
    assert s.latch_state() == "retracted" and low < pin - 0.1
    qm, _ = run(5, pin + 0.3, qm, switch=False)
    assert s.latch_state() == "retracting"
    # 4. any latched fault drops the pin (and the supervisor stops)
    qm, _ = run(int(0.5 / dt), pin + 0.3, qm, switch=True)
    assert s.roll_latch_solenoid()
    s.press_estop()
    assert not s.roll_latch_solenoid()
    return (f"Roll latch (O2, PIP_ROLL_GUARD={ROLL_GUARD!r}: off by default): solenoid on only healthy + nobody near; without a switch the "
            f"roll stays ≥ the +5° bound (+3° margin) for {LATCH_RETRACT_S * 1000:.0f} ms, with one until it confirms; a person near or an "
            f"e-stop drops the pin at once; the bound never pulls the arm up ✅")


def main():
    out = [test_mit(), test_driver(), test_supervisor(), test_people_near(), test_collision(), test_power(), test_person_reach(), test_casters(),
           test_hand_speed(), test_person_approach(), test_roll_latch()]
    from pathlib import Path
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/s1_tests.md").write_text("# S1 software tests (generated by `python -m sw.test_s1`)\n\n" + __doc__.split("\n\n", 1)[1].strip()
                                           + "\n\n" + "\n".join(f"- {o}" for o in out))
    print("\n".join(out))


if __name__ == "__main__":
    main()
