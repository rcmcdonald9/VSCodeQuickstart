"""S0.1: bench-arm simulation environment (docs/SOFTWARE_PLAN.md).  python -m sw.bench_env  (self-test)

One Pip arm (right arm kinematics, ranges and masses from rnd/params.py + rnd/twin.JR) on a bench stand at a table, with
a cube to pick. Interface conventions match the large open datasets (DROID / Open X-Embodiment), so their data and
checkpoints transfer:
- action: end-effector pose delta in the base frame [dx, dy, dz, drx, dry, drz] (m, rad per step) + gripper ∈ [0, 1]
  (1 = closed), at CONTROL_HZ. Joint targets come from damped-least-squares IK on the grasp site.
- the arm bodies are gravity-compensated (the real arm has the spring balancer + gravity feed-forward in its joint loops).
- observations: joint positions, EEF position and quaternion, gripper opening; images from a **wrist** camera and one
  **exterior** camera (DROID-style).
Gymnasium-style API: reset(seed) → obs; step(action) → obs, reward, done, info. The scripted expert lives in
`expert_action()` (used by sw/gen_demos.py).
"""
from __future__ import annotations

import math
import os

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from rnd.params import BASELINE, act  # noqa: E402
from rnd.twin import JR, actuator_model  # noqa: E402

CONTROL_HZ = 15
TABLE_Z = 0.75
SHOULDER_Z = TABLE_Z + 0.40         # bench stand: shoulder pitch axis 0.40 m above the table top
IMG = 160


def xml(d=BASELINE) -> str:
    a = d.arm
    m = {j: act(n)["mass"] for j, (n, _) in a.joints.items()}
    s = -1                              # right arm
    A = {j: actuator_model(j) for j in ARM}   # joint damping / armature: fitted actuators_<serial>.yaml when present, else datasheet
    return f"""<mujoco model="pip_bench_arm">
<option timestep="0.002" cone="elliptic" impratio="10"/>
<visual><headlight ambient=".4 .4 .4"/><global offwidth="640" offheight="640"/></visual>
<asset><texture name="grid" type="2d" builtin="checker" rgb1=".55 .5 .45" rgb2=".5 .45 .4" width="64" height="64"/>
 <material name="tbl" texture="grid" texrepeat="6 6"/></asset>
<worldbody>
 <light name="key" pos="0.4 -0.6 2.0" dir="-0.2 0.3 -1" diffuse=".7 .7 .7"/>
 <geom name="floor" type="plane" size="3 3 .1" rgba=".7 .7 .72 1"/>
 <geom name="table" type="box" pos="0.45 0 {TABLE_Z - 0.02}" size="0.35 0.45 0.02" material="tbl" friction="1 0.01 0.001"/>
 <geom name="stand" type="box" pos="0 {-s * 0.10} {(SHOULDER_Z - 0.06) / 2}" size="0.04 0.04 {(SHOULDER_Z - 0.06) / 2}" rgba=".2 .2 .22 1"/>  <!-- top 6 cm under the pitch axis, 0.10 m inboard (the bracket clamps the shoulder from the torso side): with the inline roll (offset 69 mm, was 83) a post centred under the pitch axis sat inside the upper arm's sweep, and the twin's upper arm jammed on it -->
 <camera name="exterior" pos="1.15 -0.55 1.35" xyaxes="0.45 0.89 0 -0.42 0.21 0.88" fovy="55"/>
 <body name="sh" gravcomp="1" pos="0 {-s * 0.0} {SHOULDER_Z}">
  <joint name="pitch" axis="0 1 0" range="{JR['pitch'][0]} {JR['pitch'][1]}" damping="{A["pitch"]["viscous_nms"]}" armature="{A["pitch"]["armature"]}"/>
  <geom type="sphere" size="0.04" mass="{m['shoulder_pitch'] * 0.3}" rgba=".1 .1 .1 1"/>
  <body name="roll" gravcomp="1" pos="0 {s * a.roll_offset_y} 0">
   <joint name="roll" axis="{s} 0 0" range="{JR['roll'][0]} {JR['roll'][1]}" damping="{A["roll"]["viscous_nms"]}" armature="{A["roll"]["armature"]}"/>
   <geom type="sphere" size="0.038" mass="{m['shoulder_roll'] + a.roll_stage_mass}" rgba=".1 .1 .1 1"/>
   <body name="upper" gravcomp="1" pos="0 0 {-a.yaw_drop}">
    <joint name="yaw" axis="0 0 1" range="{JR['yaw'][0]} {JR['yaw'][1]}" damping="{A["yaw"]["viscous_nms"]}" armature="{A["yaw"]["armature"]}"/>
    <geom type="capsule" fromto="0 0 0 0 0 {-(a.l_upper - a.yaw_drop)}" size="0.032" mass="{a.bone_mass_upper + m['upper_yaw'] + a.yaw_stage_mass}" rgba=".9 .9 .9 1"/>
    <body name="fore" gravcomp="1" pos="0 0 {-(a.l_upper - a.yaw_drop)}">
     <joint name="elbow" axis="0 1 0" range="{JR['elbow'][0]} {JR['elbow'][1]}" damping="{A["elbow"]["viscous_nms"]}" armature="{A["elbow"]["armature"]}"/>
     <geom type="capsule" fromto="0 0 0 0 0 {-(a.l_fore - 0.06)}" size="0.026" mass="{a.bone_mass_fore + m['elbow']}" rgba=".9 .9 .9 1"/>
     <body name="wrist" gravcomp="1" pos="0 0 {-a.l_fore}">
      <joint name="wpitch" axis="0 1 0" range="{-a.wrist_pitch_range_deg} {a.wrist_pitch_range_deg}" damping="{A["wpitch"]["viscous_nms"]}" armature="{A["wpitch"]["armature"]}"/>
      <geom type="sphere" size="0.028" mass="{m['wrist_pitch']}" rgba=".1 .1 .1 1"/>
      <body name="hand" gravcomp="1" pos="0 0 -0.04">
       <joint name="wroll" axis="0 0 1" range="{JR['wroll'][0]} {JR['wroll'][1]}" damping="{A["wroll"]["viscous_nms"]}" armature="{A["wroll"]["armature"]}"/>
       <geom type="box" size="0.032 0.02 0.025" pos="0 0 -0.035" mass="{m['wrist_roll'] + m['gripper'] + a.gripper_mech_mass}" rgba=".15 .15 .15 1"/>
       <camera name="wrist" pos="0.05 0 -0.03" xyaxes="0 -1 0 0.95 0 0.3" fovy="80"/>  <!-- looks down past the fingertips (DROID-style wrist view) -->
       <site name="grasp" pos="0 0 {-(a.l_hand - 0.04)}" size="0.005"/>
       <body name="finger_a" gravcomp="1" pos="0.05 0 {-(a.l_hand - 0.04)}"><joint name="fa" type="slide" axis="-1 0 0" range="0 0.045" damping="5"/>
        <geom type="box" size="0.005 0.012 0.025" mass="0.03" friction="1.5 0.02 0.002" rgba=".95 .55 .2 1"/></body>
       <body name="finger_b" gravcomp="1" pos="-0.05 0 {-(a.l_hand - 0.04)}"><joint name="fb" type="slide" axis="1 0 0" range="0 0.045" damping="5"/>
        <geom type="box" size="0.005 0.012 0.025" mass="0.03" friction="1.5 0.02 0.002" rgba=".95 .55 .2 1"/></body>
      </body>
     </body>
    </body>
   </body>
  </body>
 </body>
 <body name="cube" pos="0.45 0 {TABLE_Z + 0.025}"><freejoint/>
  <geom type="box" size="0.022 0.022 0.022" mass="0.1" friction="1.2 0.01 0.001" rgba=".2 .5 .9 1"/></body>
 <site name="goal" pos="0.40 -0.15 {TABLE_Z + 0.002}" size="0.03 0.001" type="cylinder" rgba=".2 .8 .3 .5"/>
</worldbody>
<equality><joint joint1="fb" joint2="fa" polycoef="0 1 0 0 0"/></equality>
<actuator>
 {''.join(f'<position name="a_{j}" joint="{j}" kp="{kp}" kv="{kv}"/>' for j, kp, kv in (("pitch", 600, 40), ("roll", 600, 40), ("yaw", 300, 20), ("elbow", 400, 25), ("wpitch", 120, 6), ("wroll", 40, 2)))}
 <position name="a_grip" joint="fa" kp="300" kv="10" forcerange="-25 25"/>
</actuator></mujoco>"""


ARM = ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll"]


class BenchEnv:
    def __init__(self, render=True, img=IMG):
        self.m = mujoco.MjModel.from_xml_string(xml())
        self.d = mujoco.MjData(self.m)
        self.render_on = render
        self.renderer = mujoco.Renderer(self.m, img, img) if render else None
        self.site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "grasp")
        self.goal = self.m.site_pos[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "goal")].copy()
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
        self.dadr = [self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
        self.cube = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.n_sub = int(round(1 / CONTROL_HZ / self.m.opt.timestep))
        self.rng = np.random.default_rng(0)
        self._d2 = mujoco.MjData(self.m)
        self.jrange = np.array([self.m.jnt_range[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM])

    WS_LO, WS_HI = np.array([0.15, -0.40, TABLE_Z + 0.015]), np.array([0.75, 0.40, TABLE_Z + 0.55])   # commanded-target workspace box

    # ---- helpers
    def eef(self):
        p = self.d.site_xpos[self.site].copy()
        R = self.d.site_xmat[self.site].reshape(3, 3).copy()
        return p, R

    SEEDS = (np.array([-0.8, 0, 0, -1.0, 1.8, 0]), np.array([-0.5, 0.3, 0, -1.4, 1.9, 0]), np.array([-1.0, 0.6, 0.5, -0.8, 1.8, 0]))

    def _ik(self, p_t, R_t, iters=30):
        """Warm-started DLS; if it stalls (> 5 mm), retry from fixed seed poses and keep the best."""
        q = self._ik_from(self.ctrl_q, p_t, R_t, iters)
        if self._ik_err > 0.005:
            best = (self._ik_err, q)
            for sd in self.SEEDS:
                q2 = self._ik_from(sd, p_t, R_t, 60)
                if self._ik_err < best[0]:
                    best = (self._ik_err, q2)
            self._ik_err, q = best
        return q

    def _ik_from(self, q0, p_t, R_t, iters):
        """Damped least squares on a scratch copy of the state; returns joint targets."""
        d2 = self._d2
        d2.qpos[:] = self.d.qpos
        q = np.array(q0, float).copy()
        for k, a in enumerate(self.qadr):
            d2.qpos[a] = q[k]
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(self.m, d2); mujoco.mj_comPos(self.m, d2)
            p = d2.site_xpos[self.site]; R = d2.site_xmat[self.site].reshape(3, 3)
            ep = p_t - p
            er = 0.5 * (np.cross(R[:, 0], R_t[:, 0]) + np.cross(R[:, 1], R_t[:, 1]) + np.cross(R[:, 2], R_t[:, 2]))
            self._ik_err = float(np.linalg.norm(ep))
            if self._ik_err < 1e-4 and np.linalg.norm(er) < 1e-3:
                break
            mujoco.mj_jacSite(self.m, d2, jp, jr, self.site)
            J = np.vstack([jp[:, self.dadr], 0.3 * jr[:, self.dadr]])
            e = np.concatenate([ep, 0.3 * er])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            q = q + dq
            q = np.clip(q, self.jrange[:, 0], self.jrange[:, 1])
            for k, a in enumerate(self.qadr):
                d2.qpos[a] = q[k]
        return q

    def obs(self):
        p, R = self.eef()
        quat = np.zeros(4); mujoco.mju_mat2Quat(quat, R.flatten())
        o = dict(qpos=np.array([self.d.qpos[a] for a in self.qadr]), eef_pos=p, eef_quat=quat,
                 gripper=float(self.d.qpos[self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "fa")]] / 0.045),
                 cube_quat=self.d.xquat[self.cube].copy(),
                 cube_pos=self.d.xpos[self.cube].copy())
        if self.render_on:
            for cam in ("wrist", "exterior"):
                self.renderer.update_scene(self.d, camera=cam)
                o[f"image_{cam}"] = self.renderer.render().copy()
        return o

    # ---- API
    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.m, self.d)
        home = {"pitch": -0.9, "roll": 0.0, "yaw": 0.0, "elbow": -1.2, "wpitch": -0.5, "wroll": 0.0}
        for j, a in zip(ARM, self.qadr):
            self.d.qpos[a] = home[j]
        qa = self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "cube")] if False else self.m.body_jntadr[self.cube]
        adr = self.m.jnt_qposadr[qa]
        # spawn inside the verified top-down workspace of this arm (IK map + tracking: x 0.40–0.50, y −0.20…0.00; across the body, y > 0.02, the arm tracks poorly near its roll/yaw limits)
        self.d.qpos[adr:adr + 3] = [self.rng.uniform(0.40, 0.50), self.rng.uniform(-0.20, 0.0), TABLE_Z + 0.023]
        yaw = self.rng.uniform(-0.6, 0.6)
        self.d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        # domain randomisation (light, cube colour), part of S0.3
        self.m.light_diffuse[0] = self.rng.uniform(0.5, 0.9, 3)
        self.m.geom_rgba[-1][:3] = self.rng.uniform(0.1, 0.9, 3)
        mujoco.mj_forward(self.m, self.d)
        self.ctrl_q = np.array([self.d.qpos[a] for a in self.qadr])
        self.d.ctrl[:6] = self.ctrl_q
        p, R = self.eef()
        self.target_p, self.target_R = p, R
        self.t = 0
        return self.obs()

    def step(self, action):
        a = np.clip(np.asarray(action, float), [-0.02] * 3 + [-0.1] * 3 + [0], [0.02] * 3 + [0.1] * 3 + [1])
        self.target_p = np.clip(self.target_p + a[:3], self.WS_LO, self.WS_HI)
        dR = np.zeros(9); mujoco.mju_quat2Mat(dR, self._rotvec_quat(a[3:6]))
        self.target_R = dR.reshape(3, 3) @ self.target_R
        q = self._ik(self.target_p, self.target_R)
        self.ctrl_q = q
        self.d.ctrl[:6] = q
        self.d.ctrl[6] = 0.045 * a[6]                                 # fingers open 90 mm (real gripper: 100 mm)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.m, self.d)
        self.t += 1
        o = self.obs()
        success = bool(np.linalg.norm(o["cube_pos"][:2] - self.goal[:2]) < 0.04 and o["cube_pos"][2] < TABLE_Z + 0.03 and a[6] < 0.5)
        return o, float(success), success or self.t >= 200, dict(success=success)

    @staticmethod
    def _rotvec_quat(v):
        th = np.linalg.norm(v)
        if th < 1e-9:
            return np.array([1.0, 0, 0, 0])
        ax = v / th
        return np.concatenate([[math.cos(th / 2)], math.sin(th / 2) * ax])

    # ---- scripted expert (S0.3): top-down pick, carry, place
    def expert_action(self, o, phase):
        p = self.target_p; g = self.goal                                  # commanded pose (DROID-style deltas on the command)
        if phase <= 2 or not hasattr(self, "_latch"):
            self._latch = o["cube_pos"].copy()                            # cube position latched at the grasp
        c = self._latch
        q = o["cube_quat"]; cy = 2 * math.atan2(q[3], q[0])          # cube yaw; grasp across faces → align wrist roll modulo 90°
        cy = (cy + math.pi / 4) % (math.pi / 2) - math.pi / 4
        R_down = np.array([[math.cos(cy), -math.sin(cy), 0], [math.sin(cy), math.cos(cy), 0], [0, 0, 1]])   # hand down, rolled to the cube
        R = self.target_R
        er = 0.5 * (np.cross(R[:, 0], R_down[:, 0]) + np.cross(R[:, 1], R_down[:, 1]) + np.cross(R[:, 2], R_down[:, 2]))
        rot = np.clip(er * 0.8, -0.1, 0.1)
        plan = {0: (c + [0, 0, 0.12], 0.0), 1: (c + [0, 0, 0.005], 0.0), 2: (c + [0, 0, 0.005], 1.0), 3: (c + [0, 0, 0.15], 1.0),
                4: (np.array([g[0], g[1], c[2] + 0.13]), 1.0), 5: (np.array([g[0], g[1], TABLE_Z + 0.03]), 1.0), 6: (np.array([g[0], g[1], TABLE_Z + 0.03]), 0.0),
                7: (np.array([g[0], g[1], TABLE_Z + 0.18]), 0.0)}
        tgt, grip = plan[phase]
        e = tgt - p
        act_ = np.concatenate([np.clip(e * 0.6, -0.02, 0.02), rot, [grip]])
        track = np.linalg.norm(o["eef_pos"] - self.target_p) < 0.015                 # the arm has caught up with the command
        done_phase = np.linalg.norm(e) < (0.006 if phase in (1, 5) else 0.015) and np.linalg.norm(er) < 0.08 and track
        return act_, done_phase


def rollout_expert(env, seed, max_steps=200, record=False):
    o = env.reset(seed); phase, hold, frames = 0, 0, []
    for _ in range(max_steps):
        a, ok = env.expert_action(o, phase)
        if phase in (2, 6):
            hold += 1; ok = hold > 6
        if ok:
            phase, hold = phase + 1, 0
        if phase > 7:
            break
        if record:
            frames.append((o, a))
        o, r, done, info = env.step(a)
        if info["success"] and phase >= 7:
            break
    final = env.obs()
    ok = bool(np.linalg.norm(final["cube_pos"][:2] - env.goal[:2]) < 0.04 and final["cube_pos"][2] < TABLE_Z + 0.03)
    return ok, frames


if __name__ == "__main__":
    import time
    env = BenchEnv(render=True)
    t0 = time.time(); res = [rollout_expert(env, s)[0] for s in range(10)]
    dt = time.time() - t0
    print(f"scripted expert: {sum(res)}/10 succeed; {dt / 10:.1f} s per episode with 2 cameras at {IMG}px")
