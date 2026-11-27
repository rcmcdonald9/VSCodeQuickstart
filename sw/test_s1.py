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
   policy (peak torque allowed) never predicts more than 70 A; under sustained full demand the 30 s mean stays ≤ 28 A
   and settles just under it, then recovers when the demand stops; normal motion on the bench twin gets exactly the same
   commands as with the budget off; nothing stops the arm; BMS over-current / under-voltage flags latch a fault.
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
    sup.set_people_near(True)
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
    assert max_rate <= PN_EXIT_SPEED + 1e-6 and abs(t_exit - t_expect) < 0.05, (max_rate, t_exit, t_expect)
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
        v = self.sup.check(q, self.KP, self.KD, tff, qm, vm, 0.002, allow_peak=True, tau_meas=self.tau_meas)
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
    assert abs(ACTUATORS["STS3215-C018"][2] - st["idle_current_a"]) < 1e-9

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
    model = CurrentModel(json.loads(DESC.read_text()), sup.rated).at(None)

    def wild(qm):
        return q0 + rng.normal(0, 3.0, 6), rng.uniform(0, 200, 6), rng.uniform(0, 10, 6), rng.normal(0, 40, 6), rng.normal(0, 0.5, 6)

    st1 = dict(i=0.0, ask=0.0, over=0, tq=0)

    def each1(v, vs, qm, vm, tau):
        i = REST_A + model.total(tau, vm)
        st1["i"] = max(st1["i"], i)
        st1["ask"] = max(st1["ask"], REST_A + model.total(Supervisor.drive_torque(vs, qm, vm), vm))
        st1["over"] += i > I_PEAK + 1e-6
        st1["tq"] += bool(np.any(np.abs(tau) > sup.peak + 1e-6))
        assert abs(sup.i_pred - i) < 1e-6, (sup.i_pred, i)

    n1 = 4000
    _kin_run(sup, n1, dt, t, wild, rng, q0.copy(), True, shadow=ref, each=each1)
    assert st1["over"] == 0 and st1["tq"] == 0 and sup.counts.get("current cap", 0) > 0, (st1, sup.counts)
    assert st1["ask"] > 90.0, st1                     # the policy really asks for more than the pack's burst rating
    caps1 = sup.counts["current cap"]

    # 2. sustained full demand (every joint pushed along its motion at twice its peak torque): 30 s mean ≤ 28 A throughout,
    #    settling just under it; then the demand stops and the budget recovers (ceiling back at 70 A)
    sup, q0 = fresh()
    dt2 = 0.004

    def push(qm):
        vm = rng.normal(0, 0.5, 6)
        return q0 + rng.normal(0, 3.0, 6), np.full(6, 200.0), np.zeros(6), 2.0 * sup.peak * np.sign(vm), vm

    means, inst, ks, ceil = [], [], [], []

    def each2(v, vs, qm, vm, tau):
        means.append(sup.power.mean); inst.append(sup.i_pred); ks.append(sup.power.k); ceil.append(sup.power.ceiling)

    qm = _kin_run(sup, int(100.0 / dt2), dt2, t, push, rng, q0.copy(), True, each=each2)
    means, inst, ks = np.array(means), np.array(inst), np.array(ks)
    t_settle = 2 * T_AVG
    late = means[int(t_settle / dt2):]
    assert means.max() <= I_AVG + 1e-6, means.max()
    assert late.min() >= I_AVG - 1.0, late.min()                # converges to the budget, not far under it
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
    # the 20 lb (4.54 kg) one-hand carry of the collision test, for the record (the phase-current model is conservative at hold)
    rig = CollisionRig(payload=4.536, seed=3)
    i_carry = []
    for _ in range(2500):
        rig.step()
        i_carry.append(rig.sup.i_pred)
    i_carry = float(np.mean(i_carry))
    t_derate = T_AVG * (I_AVG - REST_A) / max(i_carry - REST_A, 1e-9)

    # 4. BMS: over-current / under-voltage latch a fault (stop, heartbeat dropped) until reset; a high measured pack
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
    return (f"Battery budget (W8a; model I = |τ|/(ratio·Kt) + idle per joint, braking = 0, + {REST_A:.1f} A for the rest of the robot): "
            f"a random high-torque policy with peak torque allowed asks for up to {st1['ask']:.0f} A; **0 of {n1} cycles predict over {I_PEAK:.0f} A** "
            f"(max {st1['i']:.1f} A, capped in {caps1} cycles, feed-forward then gains). Sustained full demand for 100 s: 70 A allowed for the first {t_burst:.1f} s (burst), "
            f"then **the 30 s mean never exceeds {I_AVG:.0f} A** (max {means.max():.2f} A; {late.min():.1f}–{late.max():.1f} A after {t_settle:.0f} s, "
            f"torque-limit factor k ≈ {k_late:.2f}); demand stopped: back to {mean_after:.1f} A and the full 70 A ceiling in 40 s. "
            f"Normal motion on the bench twin ({n3 * rig.DT:.0f} s, carry pose, no payload): {i_norm.mean():.1f} A mean / {i_norm.max():.1f} A max, "
            f"30 s mean {mean_norm:.1f} A, **commands identical to the budget off** (0 caps, 0 derates). No stop and the heartbeat kept in every run; "
            f"BMS over-current and under-voltage latch a fault; with the BMS reading 45 A the arm still never predicts over 70 A and keeps running. "
            f"For the record: the 4.54 kg one-hand carry predicts {i_carry:.1f} A, so it would be derated after ≈ {t_derate:.0f} s of holding "
            f"(phase-current model, conservative at hold) ✅ (overrides: {counts})")


def main():
    out = [test_mit(), test_driver(), test_supervisor(), test_people_near(), test_collision(), test_power()]
    from pathlib import Path
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/s1_tests.md").write_text("# S1 software tests (generated by `python -m sw.test_s1`)\n\n" + __doc__.split("\n\n", 1)[1].strip()
                                           + "\n\n" + "\n".join(f"- {o}" for o in out))
    print("\n".join(out))


if __name__ == "__main__":
    main()
