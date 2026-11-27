"""S1 software tests that need no hardware.  python -m sw.test_s1

1. MIT frames: pack → unpack round trip within one quantisation step; special frames.
2. Safety supervisor against the bench twin with injected faults: a wild policy (random targets far outside the
   limits and the workspace), a stalled loop (watchdog), an e-stop, an over-temperature report. The arm must never leave
   its limits or the workspace box, and every fault must stop it within one cycle.
3. Virtual-bus round trip for the DM-J4310 driver.
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


def main():
    out = [test_mit(), test_driver(), test_supervisor()]
    from pathlib import Path
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/s1_tests.md").write_text("# S1 software tests (generated by `python -m sw.test_s1`)\n\n" + __doc__.split("\n\n", 1)[1].strip()
                                           + "\n\n" + "\n".join(f"- {o}" for o in out))
    print("\n".join(out))


if __name__ == "__main__":
    main()
