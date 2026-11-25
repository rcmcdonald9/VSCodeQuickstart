"""Runs Pip v2 through a set of scenarios in MuJoCo and writes a report.

    python sim/run_sim.py                  # full evaluation -> sim/results/
    python sim/run_sim.py --quick          # main scenario only, no video

The real robodog MotionController (gait, IK, poses, head) drives the sim at
50 Hz; each servo is a motor model (stall torque that falls with speed,
position loop, reflected rotor inertia) stepped at 1 kHz.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "software"))
sys.path.insert(0, str(HERE))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import pip_model  # noqa: E402
from robodog.config import load_config  # noqa: E402
from robodog.gait import named_pose  # noqa: E402
from robodog.motion import MotionController  # noqa: E402
from robodog.servos import MockServoDriver  # noqa: E402

LEG_J = [f"{l}.{j}" for l in ("FR", "FL", "RR", "RL") for j in ("pitch", "knee")]
ALL_J = LEG_J + ["head.pan", "head.tilt"]


@dataclass
class Segment:
    name: str
    t0: float
    t1: float
    cmd: dict = field(default_factory=dict)   # what was asked (for scoring)


class PipSim:
    CONTROL_DT = 0.02

    def __init__(self, cfg, overrides=None, tag="default"):
        self.cfg = cfg
        _, self.info = pip_model.build(cfg, HERE / "build" / tag, overrides)
        self.m = mujoco.MjModel.from_xml_path(str(HERE / "build" / tag / "pip.xml"))
        self.d = mujoco.MjData(self.m)
        sp = self.info["servo"]
        vscale = sp["battery_v"] / 12.0
        self.tau_s = sp["stall_torque_nm_12v"] * vscale
        self.w0 = sp["no_load_speed_rad_s_12v"] * vscale
        self.kp, self.kd = sp["kp_nm_per_rad"], sp["kd_nm_s_per_rad"]
        self.sp = sp
        jid = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ALL_J}
        self.qadr = np.array([self.m.jnt_qposadr[jid[n]] for n in ALL_J])
        self.vadr = np.array([self.m.jnt_dofadr[jid[n]] for n in ALL_J])
        self.aid = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ALL_J])
        self.base = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.drv = MockServoDriver(cfg["servos"])
        self.mc = MotionController(cfg, self.drv, imu=self.imu_reading)
        self.q_ref = np.zeros(len(ALL_J))
        self._init_lying()
        self.log = {k: [] for k in ("t", "pos", "rpy", "q", "qd", "tau", "qref")}
        self.peak_tau = np.zeros(len(ALL_J))
        self.t = 0.0

    def _init_lying(self):
        pose = named_pose("lie", self.mc.stand_height)
        feet = self.mc.kin.neutral_feet(pose.height)
        feet = {l: (x, y, z + (pose.front_drop if l[0] == "F" else pose.rear_drop)) for l, (x, y, z) in feet.items()}
        legs = self.mc.kin.solve(feet, (0, 0, 0), (pose.dx, pose.dy, 0))
        for i, n in enumerate(ALL_J):
            leg, j = n.split(".")
            a = 0.0 if leg == "head" else legs[leg][0 if j == "pitch" else 1]
            self.d.qpos[self.qadr[i]] = math.radians(a)
            self.q_ref[i] = math.radians(a)
        self.d.qpos[2] = (pose.height - pose.front_drop) / 1000 + 0.011 + 0.002
        mujoco.mj_forward(self.m, self.d)

    def imu_reading(self):
        """What an IMU on the body would report: (roll, pitch) in radians."""
        w, x, y, z = self.d.xquat[self.base]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        return roll, pitch

    # ------------------------------------------------------------ physics
    def servo_torque(self):
        q, qd = self.d.qpos[self.qadr], self.d.qvel[self.vadr]
        tau = self.kp * (self.q_ref - q) - self.kd * qd
        hi = np.minimum(self.tau_s * (1 - qd / self.w0), self.tau_s)
        lo = np.maximum(self.tau_s * (-1 - qd / self.w0), -self.tau_s)
        return np.clip(tau, lo, hi)

    def step(self, seconds: float, push=None):
        n = int(round(seconds / self.m.opt.timestep))
        per_ctrl = int(round(self.CONTROL_DT / self.m.opt.timestep))
        for i in range(n):
            if i % per_ctrl == 0:
                self.mc.tick(self.CONTROL_DT)
                self.q_ref = np.radians([self.drv.angles.get(nm, 0.0) for nm in ALL_J])
            tau = self.servo_torque()
            self.d.ctrl[self.aid] = tau
            self.peak_tau = np.maximum(self.peak_tau, np.abs(tau))
            self.d.xfrc_applied[self.base] = 0
            if push and push[0] <= self.t < push[0] + push[1]:
                self.d.xfrc_applied[self.base, :3] = push[2]
            mujoco.mj_step(self.m, self.d)
            self.t += self.m.opt.timestep
            if i % 5 == 0:
                self._record(tau)

    def _record(self, tau):
        quat = self.d.xquat[self.base]
        w, x, y, z = quat
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        L = self.log
        L["t"].append(self.t); L["pos"].append(self.d.xpos[self.base].copy()); L["rpy"].append((roll, pitch, yaw))
        L["q"].append(self.d.qpos[self.qadr].copy()); L["qd"].append(self.d.qvel[self.vadr].copy())
        L["tau"].append(tau.copy()); L["qref"].append(self.q_ref.copy())

    def arrays(self):
        return {k: np.array(v) for k, v in self.log.items()}


# --------------------------------------------------------------- metrics
def fell(A, i0=0, i1=None):
    rpy, pos = A["rpy"][i0:i1], A["pos"][i0:i1]
    bad = (np.abs(rpy[:, 0]) > math.radians(45)) | (np.abs(rpy[:, 1]) > math.radians(45)) | (pos[:, 2] < 0.06)
    return bool(bad.any()), (float(A["t"][i0:i1][bad.argmax()]) if bad.any() else None)


def seg_metrics(sim: PipSim, A, seg: Segment):
    t = A["t"]
    i0, i1 = np.searchsorted(t, seg.t0), np.searchsorted(t, seg.t1)
    if i1 - i0 < 2:
        return {}
    pos, rpy, tau, qd = A["pos"][i0:i1], A["rpy"][i0:i1], A["tau"][i0:i1], A["qd"][i0:i1]
    q, qref = A["q"][i0:i1], A["qref"][i0:i1]
    dt = t[i1 - 1] - t[i0]
    yaw0 = rpy[0, 2]
    dxy = pos[-1, :2] - pos[0, :2]
    fwd = dxy[0] * math.cos(yaw0) + dxy[1] * math.sin(yaw0)
    dyaw = math.degrees(np.unwrap(rpy[:, 2])[-1] - rpy[0, 2])
    leg_tau = np.abs(tau[:, :8])
    sp = sim.sp
    # electrical estimate: current proportional to torque, + Pi & electronics
    cur = sp["stall_current_a"] * np.abs(tau) / sim.tau_s
    p_w = sp["battery_v"] * (cur.sum(axis=1) + 0.05 * len(ALL_J)) + 7.0
    f, ft = fell(A, i0, i1)
    return {
        "segment": seg.name, "seconds": round(dt, 2), "fell": f, "fell_at_s": ft,
        "forward_speed_mm_s": round(fwd / dt * 1000, 1), "turn_rate_deg_s": round(dyaw / dt, 1),
        "lateral_drift_mm": round(float(-dxy[0] * math.sin(yaw0) + dxy[1] * math.cos(yaw0)) * 1000, 1),
        "max_roll_deg": round(math.degrees(np.abs(rpy[:, 0]).max()), 1),
        "max_pitch_deg": round(math.degrees(np.abs(rpy[:, 1] - np.median(rpy[:, 1])).max()), 1),
        "body_height_mm": round(float(pos[:, 2].mean()) * 1000, 1),
        "leg_torque_peak_nm": round(float(leg_tau.max()), 2),
        "leg_torque_rms_worst_nm": round(float(np.sqrt((leg_tau ** 2).mean(axis=0)).max()), 2),
        "worst_joint": LEG_J[int(np.sqrt((leg_tau ** 2).mean(axis=0)).argmax())],
        "pct_time_saturated": round(float((leg_tau > 0.97 * sim.tau_s).mean() * 100), 1),
        "joint_speed_peak_pct_noload": round(float(np.abs(qd[:, :8]).max() / sim.w0 * 100), 0),
        "tracking_err_peak_deg": round(math.degrees(float(np.abs(q[:, :8] - qref[:, :8]).max())), 1),
        "power_w_avg": round(float(p_w.mean()), 1),
        **seg.cmd,
    }


# ------------------------------------------------------------- scenarios
def main_scenario(sim: PipSim, video=None):
    mc = sim.mc
    segs, t = [], 0.0

    def run(name, seconds, setup=None, cmd=None, push=None):
        nonlocal t
        if setup:
            setup()
        if video is None:
            sim.step(seconds, push)
        else:
            n = int(seconds * 30)
            for _ in range(n):
                sim.step(1 / 30, push)
                video.frame(sim)
        segs.append(Segment(name, t, t + seconds, cmd or {}))
        t += seconds

    run("stand up from lying", 2.5, lambda: mc.set_pose("stand"))
    run("stand still", 1.5)
    run("walk forward 150 mm/s", 7, lambda: mc.set_velocity(150, 0, 0), {"cmd_speed_mm_s": 150})
    run("stop", 1.5, mc.stop)
    run("turn left 45 deg/s", 5, lambda: mc.set_velocity(0, 0, math.radians(45)), {"cmd_turn_deg_s": 45})
    run("arc: 120 mm/s + 20 deg/s", 5, lambda: mc.set_velocity(120, 0, math.radians(20)),
        {"cmd_speed_mm_s": 120, "cmd_turn_deg_s": 20})
    run("walk backward 100 mm/s", 4, lambda: mc.set_velocity(-100, 0, 0), {"cmd_speed_mm_s": -100})
    run("stop", 1.5, mc.stop)
    for pan, tilt in [(60, 0), (-60, 0), (0, 25), (0, -25), (0, 0)]:
        run(f"head pan {pan} tilt {tilt}", 1.0, lambda p=pan, q=tilt: mc.set_head(p, q))
    run("walk 150 mm/s while looking left", 4, lambda: (mc.set_head(45, 10), mc.set_velocity(150, 0, 0)),
        {"cmd_speed_mm_s": 150})
    run("stop", 1.5, lambda: (mc.stop(), mc.set_head(0, 0)))
    for pose in ("sit", "stand", "bow", "stand", "lie", "stand"):
        run(f"pose: {pose}", 2.5, lambda p=pose: mc.set_pose(p))
    return segs


def speed_sweep(cfg, speeds):
    out = []
    for v in speeds:
        sim = PipSim(cfg, tag="sweep")
        sim.mc.set_pose("stand"); sim.step(2.5)
        t0 = sim.t
        sim.mc.set_velocity(v, 0, 0); sim.step(6)
        A = sim.arrays()
        m = seg_metrics(sim, A, Segment(f"walk {v} mm/s", t0 + 1.5, sim.t, {"cmd_speed_mm_s": v}))
        out.append(m)
    return out


def push_tests(cfg):
    out = []
    for walking in (False, True):
        for force in (4.0, 8.0, 12.0):
            sim = PipSim(cfg, tag="push")
            sim.mc.set_pose("stand"); sim.step(2.5)
            if walking:
                sim.mc.set_velocity(120, 0, 0); sim.step(1.5)
            t0 = sim.t
            sim.step(3.0, push=(t0 + 0.5, 0.15, np.array([0, force, 0])))
            A = sim.arrays()
            f, _ = fell(A, np.searchsorted(A["t"], t0))
            out.append({"test": f"side push {force:.0f} N x 0.15 s ({force*0.15:.1f} N·s)",
                        "while": "walking 120 mm/s" if walking else "standing", "fell": f})
    return out


def sensitivity(cfg):
    """Re-run a 150 mm/s walk with pessimistic/optimistic servo assumptions."""
    cases = {
        "baseline": {},
        "slower servo (3.0 rad/s)": {"no_load_speed_rad_s_12v": 3.0},
        "softer servo (kp 12)": {"kp_nm_per_rad": 12},
        "heavy rotor (armature 0.012)": {"armature_kg_m2": 0.012},
        "weak servo (stall 2.0 N·m)": {"stall_torque_nm_12v": 2.0},
    }
    out = []
    for name, sp in cases.items():
        sim = PipSim(cfg, overrides={"servo": sp}, tag="sens")
        sim.mc.set_pose("stand"); sim.step(2.5)
        t0 = sim.t
        sim.mc.set_velocity(150, 0, 0); sim.step(6)
        A = sim.arrays()
        m = seg_metrics(sim, A, Segment(name, t0 + 1.5, sim.t, {"cmd_speed_mm_s": 150}))
        m["case"] = name
        out.append(m)
    return out


class Video:
    def __init__(self, sim: PipSim, path: Path):
        import imageio

        self.r = mujoco.Renderer(sim.m, 544, 960)
        self.cam = mujoco.MjvCamera()
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.62, -16, -130
        self.w = imageio.get_writer(str(path), fps=30, codec="libx264", quality=6, macro_block_size=8)
        self.n = 0

    def frame(self, sim: PipSim):
        self.cam.lookat[:] = sim.d.xpos[sim.base] * [1, 1, 0] + [0, 0, 0.12]
        self.cam.azimuth += 0.08
        self.r.update_scene(sim.d, self.cam)
        self.w.append_data(self.r.render())
        self.n += 1

    def still(self, sim, path):
        import imageio

        imageio.imwrite(str(path), self.r.render())

    def close(self):
        self.w.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    cfg = load_config()
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    t_wall = time.time()

    sim = PipSim(cfg)
    video = None if (a.quick or a.no_video) else Video(sim, out / "pip_v2_sim.mp4")
    segs = main_scenario(sim, video)
    if video:
        video.close()
    A = sim.arrays()
    report = {
        "robot": {"total_mass_kg": round(sim.info["total_mass_kg"], 3),
                  "body_masses_kg": {k: round(v, 3) for k, v in sim.info["masses"].items()},
                  "servo_model": sim.info["servo"], "stall_torque_at_battery_v_nm": round(sim.tau_s, 2)},
        "main_scenario": [seg_metrics(sim, A, s) for s in segs],
        "peak_torque_nm_by_joint": dict(zip(ALL_J, np.round(sim.peak_tau, 2).tolist())),
    }
    if not a.quick:
        report["speed_sweep"] = speed_sweep(cfg, [100, 150, 200, 250, 300])
        report["push_tests"] = push_tests(cfg)
        report["sensitivity"] = sensitivity(cfg)
    report["wall_time_s"] = round(time.time() - t_wall, 1)
    (out / "report.json").write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
