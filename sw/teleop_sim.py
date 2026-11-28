"""S2 (simulated): EEF-delta teleoperation of the bench twin through the safety supervisor, recorded as LeRobot v3.0.

    python -m sw.teleop_sim --episodes 5                  # headless: a scripted leader, 5 episodes → sw/out/pip_teleop_v3
    python -m sw.teleop_sim --keyboard                    # MuJoCo viewer + keyboard (needs a display)

The chain is the one the real arm will use, minus the hardware:
    leader (keyboard / scripted leader / later a leader arm or VR)  →  EEF delta [dx dy dz drx dry drz grip] at 15 Hz
    →  IK on the twin (sw/bench_env.py)  →  joint targets interpolated at 500 Hz  →  **Supervisor.check** every 2 ms
    (joint limits, slew, workspace box, total-torque bound, watchdog, e-stop)  →  MIT law τ = kp·(q* − q) − kd·q̇ + τ_ff
    applied to the twin (bench gains from sw/config/bringup.yaml; the twin's bodies are gravity compensated, as the
    balancer + gravity feed-forward do on the real arm).
Recorded per frame: observation.state (6 joints, gripper, EEF xyz + quaternion), action = the leader's EEF delta (what a
policy will output), wrist + exterior images; written with the same LeRobot writer as sw/to_lerobot_v3.py (`V3Writer`).
An episode in which the supervisor stopped (e-stop, watchdog, fault) is discarded, not saved.

Scripted leader: the twin's pick-and-place planner (bench_env.expert_action) with a human-like hand: first-order lag
(τ 0.05 s), tremor (σ 0.5 mm per step) and a one-step reaction delay, so the recorded actions are not the expert's own
(more lag or delay makes this closed-loop planner oscillate; a person corrects with their eyes instead). Like a person,
it moves on from a free-space waypoint the hand has stopped short of (out of reach, or an IK local minimum).
Teleop-side limits (W14a): the IK is bounded and monotone (never a worse pose than the last target), and the joint step
is shortened so the hand stays inside the supervisor's workspace box: a rejected target makes the supervisor hold, and
its catch-up afterwards at its own v_max can outrun a torque-bounded joint into a tracking-error stop.
A supervisor stop is stop category 0 (electrical.md): torque off, the arm is not held; this twin is gravity compensated,
so after a stop the arm only coasts on its momentum (≈ 0.08 s × the speed at the stop; `drift_after_stop_rad`).
Keyboard: W/S x, A/D y, R/F z, Q/E yaw, I/K pitch, J/L roll, Space gripper, Enter end episode (keep), Backspace discard,
Esc e-stop (release with Esc again, then Backspace).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import deque
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

DST = Path("sw/out/pip_teleop_v3")
TASK = "pick up the cube and place it on the target"
SUB_DT = 0.002


class SupervisedBench:
    """BenchEnv whose joint servos are replaced by the MIT law behind the Supervisor (500 Hz inner loop)."""
    DQ_STEP = 0.12          # rad per 15 Hz step (1.8 rad/s): the IK target moves continuously, the hand lags a big command
    MARGIN_DEG = 1.0        # supervisor joint-limit margin: the twin's verified top-down picks need the wrist pitch within
                            # 1–2° of its 120° limit, so the bench default (3°) cuts the pick workspace (hard stop at 125°)

    def __init__(self, img=96, render=True, fast_render=True):
        import mujoco
        import yaml
        from sw.bench_env import BenchEnv, TABLE_Z
        from sw.safety import Supervisor
        self.mj = mujoco
        self.env = BenchEnv(render=render, img=img)
        m = self.env.m
        if render and fast_render:                          # software GL here: shadows + MSAA cost 10× (135 → 13 ms per frame pair)
            m.light_castshadow[:] = 0
            m.vis.quality.offsamples = 0
            self.env.renderer = mujoco.Renderer(m, img, img)
        self._kp_xml = m.actuator_gainprm[:6, 0].copy()
        m.actuator_gainprm[:6, 0] = 0                       # arm actuators off: torques come from the MIT law below
        m.actuator_biasprm[:6, :] = 0
        cfg = yaml.safe_load((Path(__file__).resolve().parent / "config" / "bringup.yaml").read_text())
        jc = cfg["joints"]
        self.KP = np.array([jc[j]["kp"] for j in ("pitch", "roll", "yaw", "elbow", "wpitch")] + [5.0])
        self.KD = np.array([jc[j]["kd"] for j in ("pitch", "roll", "yaw", "elbow", "wpitch")] + [0.2])
        self.t = 0.0
        self._d2 = mujoco.MjData(m)
        J = json.loads((Path(__file__).resolve().parent / "description" / "joints.json").read_text())
        self.sup = Supervisor(clock=lambda: self.t, fk=self.fk, v_max=np.array([0.5 * j["max_speed_rad_s"] for j in J]), margin_deg=self.MARGIN_DEG,
                              ws_lo=(0.05, -0.5, TABLE_Z + 0.01), ws_hi=(0.85, 0.5, TABLE_Z + 0.7))
        self.env.jrange = np.stack([self.sup.lo, self.sup.hi], 1)          # IK solves inside the supervisor's limits
        self.stopped = False
        self.n_sub = self.env.n_sub
        from sw.collision import MujocoTwin
        self.twin = MujocoTwin(m, self.env.qadr, self.env.dadr)     # same model: computed-torque feed-forward

    def fk(self, q):
        d2, env = self._d2, self.env
        d2.qpos[:] = env.d.qpos
        for k, a in enumerate(env.qadr):
            d2.qpos[a] = q[k]
        self.mj.mj_kinematics(env.m, d2)
        return d2.site_xpos[env.site].copy()

    WS_MARGIN = 0.005        # m inside the supervisor's workspace box

    def _in_box(self, q0, q1):
        """The longest part of the step q0 → q1 whose hand stays inside the supervisor's workspace box (less WS_MARGIN) at
        its end and midpoint. The supervisor rejects a target outside the box by holding the last one; the teleop target
        then runs on, and when the hand is back inside, the supervisor's catch-up at its own v_max (wrist pitch 6.3 rad/s,
        3.5× this loop's slew) outruns the wrist's 3 N·m torque bound: W14a seed 3 (418 rejects → a 20° wrist-pitch lag →
        tracking-error stop). The teleop layer keeps its own targets inside instead."""
        lo, hi = self.sup.ws_lo + self.WS_MARGIN, self.sup.ws_hi - self.WS_MARGIN

        def inside(s):
            return all(np.all((p >= lo) & (p <= hi)) for p in (self.fk(q0 + (q1 - q0) * s), self.fk(q0 + (q1 - q0) * s / 2)))
        if inside(1.0) or not inside(0.0):         # already outside (not from this loop): the supervisor's walk-in rule decides
            return q1
        a, b = 0.0, 1.0
        for _ in range(8):
            m = (a + b) / 2
            a, b = (m, b) if inside(m) else (a, m)
        return q0 + (q1 - q0) * a

    def q(self):
        return np.array([self.env.d.qpos[a] for a in self.env.qadr])

    def qd(self):
        return np.array([self.env.d.qvel[a] for a in self.env.dadr])

    def reset(self, seed):
        o = self.env.reset(seed)
        self.t = 0.0
        self.sup.reset(self.q(), 0.0)
        self.stopped = False
        self.q_prev_target = self.q()
        return o

    def step(self, action, estop=False, stall=False):
        """One 15 Hz teleop step. stall: the leader/loop freezes for this step (watchdog test)."""
        env, m, d = self.env, self.env.m, self.env.d
        a = np.clip(np.asarray(action, float), [-0.02] * 3 + [-0.1] * 3 + [0], [0.02] * 3 + [0.1] * 3 + [1])
        env.target_p = np.clip(env.target_p + a[:3], env.WS_LO, env.WS_HI)
        dR = np.zeros(9)
        self.mj.mju_quat2Mat(dR, env._rotvec_quat(a[3:6]))
        env.target_R = dR.reshape(3, 3) @ env.target_R
        # continuous IK only (warm start from the last target, no re-seeding): a teleop target must never jump to another
        # IK branch; an unreachable command just leaves the hand as close as it gets (bench_env._ik_continuous: bounded
        # and monotone; the plain DLS diverged against the wrist-pitch limit, W14a)
        q_ik = env._ik_continuous(env.ctrl_q, env.target_p, env.target_R, 30)
        dq = q_ik - env.ctrl_q
        q_goal = env.ctrl_q + dq * min(1.0, self.DQ_STEP / max(float(np.max(np.abs(dq))), 1e-9))
        q_goal = self._in_box(env.ctrl_q, q_goal)
        env.ctrl_q = q_goal
        d.ctrl[6] = 0.045 * a[6]
        if estop:
            self.sup.press_estop()
        reasons = set()
        qd_t = (q_goal - self.q_prev_target) / (self.n_sub * SUB_DT)       # target velocity over this 15 Hz step
        for i in range(self.n_sub):
            self.t += SUB_DT
            if not stall:
                self.sup.tick()
            q_t = self.q_prev_target + (q_goal - self.q_prev_target) * (i + 1) / self.n_sub
            qm, vm = self.q(), self.qd()
            if i % 4 == 0:                                                 # model torque at the target (every 8 ms)
                tff = self.twin.inverse(q_t, qd_t, np.zeros(6)) + self.KD * qd_t
            v = self.sup.check(q_t, self.KP, self.KD, tff, qm, vm, 0.0, dt=SUB_DT)
            tau = np.zeros(6) if v.stop else v.kp * (v.q - qm) - v.kd * vm + v.t_ff
            if v.stop:
                self.stopped = True
                reasons |= set(v.reasons)
            d.qfrc_applied[:] = 0
            for k, adr in enumerate(env.dadr):
                d.qfrc_applied[adr] = tau[k]
            self.mj.mj_step(m, d)
        self.q_prev_target = q_goal
        env.t += 1
        o = env.obs()
        success = bool(np.linalg.norm(o["cube_pos"][:2] - env.goal[:2]) < 0.04 and o["cube_pos"][2] < env.goal[2] + 0.03 and a[6] < 0.5)
        return o, success, sorted(reasons)


class ScriptedLeader:
    """The twin's planner played through a human-like hand: lag, tremor, reaction delay (see module docstring)."""

    def __init__(self, bench, seed, lag_s=0.05, tremor=0.0005, delay=1, fps=15):
        self.b, self.rng = bench, np.random.default_rng(seed)
        self.alpha = 1 - math.exp(-1 / (fps * lag_s))
        self.tremor, self.buf = tremor, deque([np.zeros(7)] * delay, maxlen=delay + 1)
        self.y = np.zeros(7)
        self.phase, self.hold = 0, 0
        self.stall, self._eef = 0, deque(maxlen=self.STALL_STEPS)

    FREE = (0, 3, 4, 7)      # free-space moves (above the cube, lift, carry, retreat): no precision needed
    STALL_STEPS = 15         # settled (hand within 5 mm of where it was 1 s before, command at the plan point) for 1 s

    def act(self, o):
        env = self.b.env
        a, ok = env.expert_action(o, self.phase)
        if self.phase in (2, 6):
            self.hold += 1
            ok = self.hold > 6
        # an operator whose hand stops short of a free-space waypoint moves on from where it is; the planner alone would
        # wait for it forever (W14a: seed 3's lift 0.15 m above a cube 0.41 m out needs the wrist pitch past its 119° bound,
        # the hand stops 3 cm low; other seeds stop in an IK local minimum on the 150° turn from home to hand-down, and the
        # next waypoint's target gets them out). Grasp and place heights (phases 1, 5) still need the 6 mm.
        self._eef.append(o["eef_pos"].copy())
        settled = len(self._eef) == self.STALL_STEPS and float(np.linalg.norm(self._eef[-1] - self._eef[0])) < 0.005
        self.stall = self.stall + 1 if (self.phase in self.FREE and np.linalg.norm(a[:3]) < 0.003 and settled) else 0
        if self.stall >= self.STALL_STEPS:
            ok = True
        if ok:
            self.phase, self.hold, self.stall = self.phase + 1, 0, 0
            self._eef.clear()
        self.buf.append(a)
        x = self.buf[0]
        self.y = self.y + self.alpha * (x - self.y)
        out = self.y.copy()
        out[:3] += self.rng.normal(0, self.tremor, 3)
        out[6] = x[6]                                       # the gripper button is binary
        return out, self.phase > 7


def state_of(o):
    return np.concatenate([o["qpos"], [o["gripper"]], o["eef_pos"], o["eef_quat"]]).astype(np.float32)


def record(n_episodes=5, out=DST, img=96, seed=0, max_steps=260, estop_episode=None, log=print):
    """Headless scripted-leader recording. estop_episode: index of an extra episode with an e-stop at step 20 (must be
    discarded and must stop the arm). Returns a summary dict."""
    from sw.to_lerobot_v3 import V3Writer
    from sw.bench_env import CONTROL_HZ
    bench = SupervisedBench(img=img)
    w = V3Writer(out, "local/pip_teleop", CONTROL_HZ, (img, img, 3))
    kept, discarded, eps = 0, 0, []
    t0 = time.time()
    k = 0
    lat = []
    while kept < n_episodes and k < n_episodes + 10:
        o = bench.reset(seed + k)
        leader = ScriptedLeader(bench, seed + k)
        frames, stop_reasons, success = [], [], False
        overrides0 = dict(bench.sup.counts)
        for t in range(max_steps):
            a, done = leader.act(o)
            if done:
                break
            frames.append((state_of(o), a.astype(np.float32), o["image_wrist"], o["image_exterior"]))
            ts = time.perf_counter()
            o, success, reasons = bench.step(a, estop=(k == estop_episode and t == 20))
            lat.append(time.perf_counter() - ts)
            if success and leader.phase >= 7:                  # placed, released, hand lifting: episode done
                break
            if reasons:
                stop_reasons = reasons
                q_stop, v_stop = bench.q(), bench.qd()
                for _ in range(5):                              # the arm must stay stopped (torque off, gravcomp holds it)
                    o, _, _ = bench.step(np.zeros(7))
                drift = float(np.max(np.abs(bench.q() - q_stop)))
                break
        overrides = {kk: v - overrides0.get(kk, 0) for kk, v in bench.sup.counts.items() if v - overrides0.get(kk, 0)}
        placed = bool(success and leader.phase >= 7)      # placed and released (a cube that spawns near the goal is not a success)
        rec = dict(seed=seed + k, steps=len(frames), success=placed, stop=stop_reasons, overrides=overrides)
        if stop_reasons:
            rec["drift_after_stop_rad"] = drift
            rec["speed_at_stop_rad_s"] = float(np.max(np.abs(v_stop)))
            discarded += 1
            bench.sup.release_estop()
        else:
            for f in frames:
                w.add(*f, TASK)
            w.save_episode()
            kept += 1
        eps.append(rec)
        log(f"episode {k}: {rec}")
        k += 1
    w.finalize()
    dt = time.time() - t0
    res = dict(kept=kept, discarded=discarded, episodes=eps, seconds=round(dt, 1), frames=sum(e["steps"] for e in eps if not e["stop"]),
               step_ms_p50=round(1000 * float(np.median(lat)), 2), step_ms_p99=round(1000 * float(np.percentile(lat, 99)), 2),
               success=sum(e["success"] for e in eps if not e["stop"]))
    (Path(out) / "meta" / "pip_teleop.json").write_text(json.dumps(res, indent=1))
    return res


def verify(out=DST, n_expected=None):
    """Reload with the official LeRobot loader; shapes, lengths, action round trip against our own sidecar."""
    import pandas as pd
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("local/pip_teleop", root=out)
    meta = json.loads((Path(out) / "meta" / "info.json").read_text())
    if n_expected is not None:
        assert ds.num_episodes == n_expected, (ds.num_episodes, n_expected)
    x = ds[0]
    shapes = {k: tuple(v.shape) for k, v in x.items() if hasattr(v, "shape")}
    assert shapes["observation.state"] == (14,) and shapes["action"] == (7,), shapes
    # every frame's action reloads within float32 of what was written (parquet column vs the loader)
    files = sorted((Path(out) / "data").rglob("*.parquet"))
    A = np.concatenate([np.stack(pd.read_parquet(f)["action"].to_numpy()) for f in files])
    assert len(A) == len(ds), (len(A), len(ds))
    worst = 0.0
    for i in np.linspace(0, len(ds) - 1, 20).astype(int):
        worst = max(worst, float(np.abs(ds[int(i)]["action"].numpy() - A[i]).max()))
    assert worst < 1e-6, worst
    return dict(episodes=ds.num_episodes, frames=len(ds), fps=ds.fps, codebase=meta["codebase_version"], sample_shapes=shapes,
                max_action_error=worst)


def keyboard(img=160, out=Path("sw/out/pip_teleop_kb_v3")):
    """Interactive: MuJoCo passive viewer + keyboard (GLFW key codes)."""
    import mujoco.viewer
    from sw.to_lerobot_v3 import V3Writer
    from sw.bench_env import CONTROL_HZ
    bench = SupervisedBench(img=img)
    w = V3Writer(out, "local/pip_teleop_kb", CONTROL_HZ, (img, img, 3))
    keys = {87: (0, +1), 83: (0, -1), 65: (1, +1), 68: (1, -1), 82: (2, +1), 70: (2, -1),      # W S A D R F
            74: (3, +1), 76: (3, -1), 73: (4, +1), 75: (4, -1), 81: (5, +1), 69: (5, -1)}      # J L I K Q E
    state = dict(a=np.zeros(7), grip=0.0, end=None, estop=False)

    def on_key(code):
        if code in keys:
            i, sgn = keys[code]
            state["a"][i] += sgn * (0.01 if i < 3 else 0.05)
        elif code == 32:
            state["grip"] = 1.0 - state["grip"]
        elif code == 257:
            state["end"] = "keep"
        elif code == 259:
            state["end"] = "discard"
        elif code == 256:
            state["estop"] = not state["estop"]
    ep = 0
    o = bench.reset(ep)
    frames = []
    with mujoco.viewer.launch_passive(bench.env.m, bench.env.d, key_callback=on_key) as v:
        while v.is_running():
            t0 = time.time()
            a = state["a"].copy()
            a[6] = state["grip"]
            state["a"][:] = 0
            frames.append((state_of(o), a.astype(np.float32), o["image_wrist"], o["image_exterior"]))
            o, success, reasons = bench.step(a, estop=state["estop"])
            v.sync()
            if reasons:
                print("supervisor stop:", reasons, "(Backspace to discard and reset)")
            if state["end"]:
                if state["end"] == "keep" and not bench.stopped:
                    for f in frames:
                        w.add(*f, TASK)
                    w.save_episode()
                    print(f"episode {ep} saved ({len(frames)} frames, success {success})")
                if not state["estop"]:
                    ep += 1
                    o, frames = bench.reset(ep), []
                    state["end"] = None
            time.sleep(max(0.0, 1 / CONTROL_HZ - (time.time() - t0)))
    w.finalize()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--img", type=int, default=96)
    ap.add_argument("--out", default=str(DST))
    ap.add_argument("--keyboard", action="store_true")
    a = ap.parse_args()
    if a.keyboard:
        keyboard(out=Path(a.out))
        return
    res = record(a.episodes, Path(a.out), a.img)
    print(json.dumps(res | {"reload": verify(Path(a.out), a.episodes)}, default=str))


if __name__ == "__main__":
    main()
