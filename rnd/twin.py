"""Simulation twin v0 (T04): MuJoCo kinematic/mass model generated from params.

Studies:
  * reach coverage of household targets (floor, table, counter, shelves) with
    gripper orientation constraints, sampling joint space + lift;
  * static tip-over margin with payload, for several wheel layouts;
  * renders of representative poses.
"""
from __future__ import annotations

import math
from pathlib import Path
import os

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

from .budget import mass_budget  # noqa: E402
from .params import BASELINE, Design, act  # noqa: E402

JR = {  # joint ranges (deg). Conventions: arm hangs straight down at 0.
    # pitch about +Y: NEGATIVE swings the arm forward/up.  roll: positive = abduct outward.
    # elbow about +Y: NEGATIVE bends the forearm forward.
    "pitch": (-200, 60), "roll": (-15, 100), "yaw": (-150, 150),        # loop 33: ±150 → ±70 (torsion crossing); loop 40: back to ±150 with a clock-spring cassette (dexterity, owner 2026-11-26)
    "elbow": (-150, 0), "wpitch": (-110, 110),
    "wroll": (-BASELINE.arm.wrist_roll_range_deg, BASELINE.arm.wrist_roll_range_deg)}   # 2026-11-27: ±180 → ±120 (clock-spring capacity, params)
# D25 (owner, 2026-11-27; R17): soft limits while a person is near (sw/safety.py Supervisor.set_people_near). The inboard
# shoulder-roll wedge (yaw band / block against the arm-side ferrule) closes to ≈ 2.5 mm at the −15° stop and no boot can
# span it. Measured with rnd/pinch.py (wave 7, per-pose worst reachable cell): yaw band × ferrule 2.6 / 16.5 / 23.4 / 24.7 /
# 26.1 / 30.2 / > 40 mm at −15 / −5 / 0 / +1 / +2 / +5 / +7.5°, so the wedge is ≥ 25 mm from ≈ +1.2°; the yaw block ×
# ferrule nip opens past 0°. +5° keeps ≈ 5 mm (≈ 4°) of margin for print / assembly tolerance and the probe lattice, and the
# supervisor's 3° margin puts the targets at ≥ +8° (≈ 34 mm). The owner's estimate was ≈ +15° (wedge > 40 mm there).
JR_PEOPLE_NEAR = {"roll": (5.0, JR["roll"][1])}


# ---- actuator model (real2sim, 2026-11-27). Joint-side units: N·m, N·m·s/rad, rad, ms. The datasheet model is the fallback;
# the bring-up's sysid step (sw/bringup.py step "sysid") fits each drive and writes sw/config/actuators_<serial>.yaml,
# which `actuator_model()` returns instead once it exists. Once measured these are NOT domain-randomised (they are a fixed,
# per-serial fact of the robot); payload, contact friction and object properties still are (docs/sw/sim2real_bench.md).
ACTUATOR_DATASHEET = {   # coulomb/viscous/armature: the bench twin's values (sw/bench_env.py) and the virtual plant's; all ESTIMATE
    "pitch":  dict(actuator="GIM6010-8 + 3:1 belt", coulomb_nm=0.25, viscous_nms=2.0, armature=0.05, kt_scale=1.0, backlash_rad=0.0015, latency_ms=None),
    "roll":   dict(actuator="GIM8108-36 inline", coulomb_nm=0.25, viscous_nms=2.0, armature=0.05, kt_scale=1.0, backlash_rad=0.0044, latency_ms=None),
    "yaw":    dict(actuator="GIM6010-8 + 2:1 belt", coulomb_nm=0.25, viscous_nms=1.0, armature=0.03, kt_scale=1.0, backlash_rad=0.0022, latency_ms=None),
    "elbow":  dict(actuator="GIM6010-8 + 2:1 belt", coulomb_nm=0.25, viscous_nms=1.0, armature=0.03, kt_scale=1.0, backlash_rad=0.0022, latency_ms=None),
    "wpitch": dict(actuator="DM-J4310-2EC 10:1", coulomb_nm=0.04, viscous_nms=0.5, armature=0.01, kt_scale=1.0, backlash_rad=0.0035, latency_ms=None),
    "wroll":  dict(actuator="STS3215 (TTL)", coulomb_nm=0.05, viscous_nms=0.3, armature=0.005, kt_scale=1.0, backlash_rad=0.0087, latency_ms=None),
}   # backlash: planetary-stage ESTIMATE (≈ 0.25–0.5° at the motor output, divided by the belt ratio); no vendor figure found
ACTUATOR_DIR = Path(__file__).resolve().parent.parent / "sw" / "config"


def measured_actuators(config_dir=None):
    """{joint: fitted model} from every actuators_<serial>.yaml in config_dir (default sw/config); newest fit per joint."""
    import yaml
    out = {}
    for f in sorted(Path(config_dir or ACTUATOR_DIR).glob("actuators_*.yaml")):
        try:
            a = yaml.safe_load(f.read_text()) or {}
        except Exception:
            continue
        j = a.get("joint")
        if j and (j not in out or str(a.get("date", "")) >= str(out[j].get("date", ""))):
            out[j] = dict(a, file=f.name)
    return out


def actuator_model(joint, config_dir=None):
    """The model the sims use for one joint: the fitted one when its yaml exists, else the datasheet fallback."""
    base = dict(ACTUATOR_DATASHEET[joint], source="datasheet (ESTIMATE)")
    fit = measured_actuators(config_dir).get(joint)
    if fit:
        base.update({k: v for k, v in fit.items() if v is not None})
        base["source"] = f"fitted ({fit['file']})"
    return base


COLL = Path(__file__).resolve().parent / "out" /(("collision" if BASELINE.arm.roll_drive == "belt" else f"collision_{BASELINE.arm.roll_drive}")
                                                 + ("_d20" if BASELINE.arm.arm_motors == "d20" else ""))   # D24: one hull set per roll drive; D20: per arm-motor set
GEOMETRY = "cad"     # "cad": arm collision from the CAD hulls (rnd/parts/collision.py); "capsule": twin v0 primitives


def _cad_meshes():
    if not (COLL / "upper_housing_r.stl").exists():
        from .parts.collision import export
        export()
    return sorted(COLL.glob("*.stl"))


def build_xml(d: Design, geometry: str | None = None) -> str:
    a, b, u = d.arm, d.base, d.upper
    cad = (geometry or GEOMETRY) == "cad"
    meshes = _cad_meshes() if cad else []
    mesh_assets = "".join(f'<mesh name="{f.stem}" file="{f}" scale="0.001 0.001 0.001"/>' for f in meshes)
    cap = ' contype="0" conaffinity="0" group="3"' if cad else ""

    def hull(body, side):
        if not cad:
            return ""
        return "".join(f'<geom type="mesh" mesh="{f.stem}" mass="0.001" rgba=".9 .9 .92 1"/>' for f in meshes
                       if f.stem.startswith(body + "_") and f.stem.endswith("_" + side))
    mb = mass_budget(d)
    base_m = sum(m for _, m in mb["base"]) - b.power_board_mass
    bal = mb.get("ballast (2 removable blocks)", [])        # A10 v2 (rnd/tip.py): removable steel blocks low in the rear sectors
    bx, by, bz = b.ballast_xyz
    ballast_xml = "".join(f'<geom type="box" size="0.046 0.0222 0.0381" pos="{bx} {s * by} {bz}" mass="{m}" rgba=".35 .35 .38 1" contype="0" conaffinity="0"/>'
                          for (_, m), s in zip(bal, (1, -1)))
    if b.power_board_mass > 0:                                # A10 v2: the A5 v2 power stage on the belly (base.power_board)
        ballast_xml += f'<geom type="box" size="0.045 0.028 0.0225" pos="{b.power_board_xyz[0]} {b.power_board_xyz[1]} {b.power_board_xyz[2]}" mass="{b.power_board_mass}" rgba=".2 .2 .2 1" contype="0" conaffinity="0"/>'
    m = {j: act(n)["mass"] for j, (n, _) in a.joints.items()}

    def arm(side):
        s = -1 if side == "r" else 1
        return f"""
      <body name="sh_{side}" pos="{a.shoulder_x} {s*a.shoulder_y} 0">
        <joint name="pitch_{side}" axis="0 1 0" range="{JR['pitch'][0]} {JR['pitch'][1]}"/>
        <geom type="sphere" size="0.04" mass="{m['shoulder_pitch']*0.3}" rgba=".1 .1 .1 1"/>{hull("sh", side)}
        <body name="roll_{side}" pos="0 {s*a.roll_offset_y} 0">
          <joint name="roll_{side}" axis="{s} 0 0" range="{JR['roll'][0]} {JR['roll'][1]}"/>
          <geom type="sphere" size="0.038" mass="{m['shoulder_roll'] + a.roll_stage_mass}" rgba=".1 .1 .1 1"/>{hull("roll", side)}
          <body name="upper_{side}" pos="0 0 {-a.yaw_drop}">
            <joint name="yaw_{side}" axis="0 0 1" range="{JR['yaw'][0]} {JR['yaw'][1]}"/>
            <geom type="sphere" size="0.03" pos="0 0 0" mass="{m['upper_yaw'] + a.yaw_stage_mass}" rgba=".1 .1 .1 1"/>
            <geom type="capsule" fromto="0 0 0 0 0 {-(a.l_upper-a.yaw_drop)}" size="0.032" mass="{a.bone_mass_upper}" rgba=".9 .9 .9 1"{cap}/>{hull("upper", side)}
            {f'<geom type="cylinder" size="0.043 0.025" pos="0 0 {-(a.elbow_motor_at-a.yaw_drop)}" euler="90 0 0" mass="{m["elbow"]}" rgba=".1 .1 .1 1"/>' if a.elbow_motor_at else ''}
            <body name="fore_{side}" pos="0 0 {-(a.l_upper-a.yaw_drop)}">
              <joint name="elbow_{side}" axis="0 1 0" range="{JR['elbow'][0]} {JR['elbow'][1]}"/>
              <geom type="sphere" size="0.036" mass="{m['elbow'] if a.elbow_motor_at is None else a.elbow_hub_mass}" rgba=".1 .1 .1 1"/>
              <geom type="capsule" fromto="0 0 0 0 0 {-(a.l_fore-0.06)}" size="0.026" mass="{a.bone_mass_fore}" rgba=".9 .9 .9 1"{cap}/>{hull("fore", side)}
              <!-- forearm bone ends 60 mm before the wrist axis: CAD requirement so the hand clears ±140° wrist pitch -->
              <body name="wrist_{side}" pos="0 0 {-a.l_fore}">
                <joint name="wpitch_{side}" axis="0 1 0" range="{-a.wrist_pitch_range_deg} {a.wrist_pitch_range_deg}"/>
                <geom type="sphere" size="0.028" mass="{m['wrist_pitch']}" rgba=".1 .1 .1 1"/>{hull("wristb", side)}
                <body name="hand_{side}" pos="0 0 -0.04">
                  <joint name="wroll_{side}" axis="0 0 1" range="{JR['wroll'][0]} {JR['wroll'][1]}"/>
                  <geom type="box" size="0.032 0.02 0.025" pos="0 0 -0.035" mass="{m['wrist_roll'] + m['gripper'] + a.gripper_mech_mass + a.wrist_cam_mass}" rgba=".15 .15 .15 1"{cap}/>
                  <geom type="box" size="0.006 0.012 0.03" pos="0.022 0 -0.07" mass="0.001" rgba=".9 .9 .9 1"{cap}/>
                  <geom type="box" size="0.006 0.012 0.03" pos="-0.022 0 -0.07" mass="0.001" rgba=".9 .9 .9 1"{cap}/>
                  {hull("hand", side)}
                  <site name="grasp_{side}" pos="0 0 {-(a.l_hand-0.04)}"/>
                  <body name="payload_{side}" pos="0 0 {-(a.l_hand-0.04)}">
                    <geom type="box" size="0.03 0.03 0.03" mass="{a.payload if side=='r' else 0.0001}" rgba=".85 .2 .1 {1 if side=='r' else 0}"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>"""

    head_z = b.clearance + b.height + u.spine_len + 0.118     # head centre on the mast top (CAD: bracket + pan cartridge + yoke)

    def head_xml(z):
        return (f'<geom type="box" size="0.08 0.115 0.06" pos="0.01 0 {z}" mass="{u.head_mass}" rgba=".95 .95 .95 1"/>'
                f'<geom type="box" size="0.01 0.1 0.04" pos="0.085 0 {z}" mass="0.001" rgba=".05 .05 .05 1"/>'
                # head depth camera (Gemini 336 class, ~65° vertical FOV), tilted 60° down: the floor-work view
                f'<camera name="head_down" pos="0.095 0 {z}" xyaxes="0 -1 0 0.866 0 0.5" fovy="65"/>'
                f'<camera name="head_level" pos="0.095 0 {z}" xyaxes="0 -1 0 0 0 1" fovy="65"/>')

    torso_m = u.carriage_mass + u.torso_struct_mass + u.compute_mass + u.cable_mass + 2 * a.pitch_module_mass
    spine_m = u.spine_len * u.spine_mass_per_m + u.handle_mass + b.coupling_mass / 2
    return f"""<mujoco model="pip_v4_twin">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81"/>
  <visual><global offwidth="1280" offheight="960"/><headlight ambient=".45 .45 .45" diffuse=".5 .5 .5"/></visual>
  <asset>{mesh_assets}<texture name="g" type="2d" builtin="checker" rgb1=".86 .88 .9" rgb2=".8 .82 .84" width="256" height="256"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1=".95 .96 .98" rgb2=".75 .8 .86" width="128" height="128"/>
    <material name="g" texture="g" texrepeat="12 12"/></asset>
  <worldbody>
    <light pos="1 -1 3" dir="-.3 .3 -1" diffuse=".8 .8 .8"/>
    <geom type="plane" size="5 5 .1" material="g" contype="0" conaffinity="0"/>
    <geom name="table" type="box" pos="0.75 -0.15 0.37" size="0.15 0.3 0.375" rgba=".6 .45 .3 .35" contype="0" conaffinity="0"/>
    <body name="robot">
      <geom type="cylinder" size="{b.radius} {b.height/2}" pos="0 0 {b.clearance + b.height/2}" mass="{base_m}" rgba=".92 .92 .92 1"/>
      <geom type="box" size="0.07 0.05 0.03" pos="-0.03 0 {b.clearance+0.035}" mass="{b.battery_mass}" rgba=".2 .3 .7 1"/>{ballast_xml}
      <geom type="box" size="{u.mast_w/2} {u.mast_w/2} {u.spine_len/2}" pos="-0.06 0 {b.clearance + b.height + u.spine_len/2}" mass="{spine_m}" rgba=".6 .62 .65 1"/>
      <geom type="box" size="0.05 0.05 0.05" pos="-0.06 0 {b.clearance + b.height + 0.06}" mass="{u.lift_motor_mass}" rgba=".2 .2 .2 1"/>
      {head_xml(head_z) if u.head_on_mast else ""}
      <body name="torso" pos="0 0 0">
        <joint name="lift" type="slide" axis="0 0 1" range="{u.lift_min} {u.lift_max}"/>
        <geom type="box" size="0.115 0.195 0.105" pos="0.015 0 0" mass="{torso_m}" rgba=".95 .95 .95 1"/>
        {"" if u.head_on_mast else head_xml(0.22)}
{hull("torso", "l")}{hull("torso", "r")}
                {arm('r')}{arm('l')}
      </body>
    </body>
  </worldbody>
  <contact><exclude body1="robot" body2="torso"/></contact>
</mujoco>"""


def support_polygon(layout: str, r: float, phase_deg: float = 0.0):
    n = 3 if layout == "kiwi" else 4
    return np.array([[r * math.cos(math.radians(phase_deg + i * 360 / n)),
                      r * math.sin(math.radians(phase_deg + i * 360 / n))] for i in range(n)])


def margin(poly, p):
    """Signed distance from p to the edges of a convex CCW polygon (positive = inside)."""
    ds = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        e = b - a
        nrm = np.array([e[1], -e[0]]) / np.linalg.norm(e)   # outward normal for CCW
        ds.append(-np.dot(p - a, nrm))
    return min(ds)


TARGETS = {
    "counter, angled 45° grasp (z 0.95)": dict(z=0.95, xs=np.arange(0.30, 0.66, 0.05), axis=(0.7071, 0, -0.7071)),
    "floor pickup (z 0.03, gripper down)": dict(z=0.03, xs=np.arange(0.30, 0.56, 0.05), down=True),
    "dishwasher low rack (z 0.20, down)": dict(z=0.20, xs=np.arange(0.35, 0.66, 0.05), down=True),
    "table top (z 0.78, down)": dict(z=0.78, xs=np.arange(0.30, 0.66, 0.05), down=True),
    "counter top-down only (z 0.95) [info: counters use top-down OR 45°, see D4b]": dict(z=0.95, xs=np.arange(0.30, 0.66, 0.05), down=True, info=True),
    "shelf (z 1.20, horizontal)": dict(z=1.20, xs=np.arange(0.30, 0.56, 0.05), down=False),
    "high shelf (z 1.40, horizontal)": dict(z=1.40, xs=np.arange(0.30, 0.46, 0.05), down=False),
}
YS = np.arange(-0.35, 0.11, 0.05)


def sample(d: Design, n: int = 250_000, seed: int = 0):
    m = mujoco.MjModel.from_xml_string(build_xml(d))
    dat = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    jn = ["lift"] + [f"{j}_r" for j in ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")]
    adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in jn]
    lo = np.array([m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)][0] for j in jn])
    hi = np.array([m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)][1] for j in jn])
    # (MuJoCo stores compiled hinge ranges in radians already)
    # tuck the left arm: hanging, elbow bent forward
    dat.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "elbow_l")]] = math.radians(-90)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
    hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand_r")
    robot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")
    Q = rng.uniform(lo, hi, size=(n, len(jn)))
    P = np.zeros((n, 3)); A = np.zeros((n, 3)); C = np.zeros((n, 3))
    for i in range(n):
        dat.qpos[adr] = Q[i]
        mujoco.mj_kinematics(m, dat)
        mujoco.mj_comPos(m, dat)
        P[i] = dat.site_xpos[site]
        A[i] = -dat.xmat[hand].reshape(3, 3)[:, 2]          # hand pointing direction
        C[i] = dat.subtree_com[robot]
    return dict(Q=Q, P=P, A=A, C=C, model=m, adr=adr)


def reach_and_tip(S, layouts, tol=0.03, cos_lim=math.cos(math.radians(30))):
    tree = cKDTree(S["P"])
    res = {}
    for name, t in TARGETS.items():
        want = np.array([0, 0, -1.0]) if t["down"] else np.array([1.0, 0, 0])
        ok_orient = S["A"] @ want >= cos_lim
        pts = [(x, y, t["z"]) for x in t["xs"] for y in YS]
        hit, best = 0, {k: [] for k in layouts}
        for p in pts:
            idx = [i for i in tree.query_ball_point(p, tol) if ok_orient[i]]
            if not idx:
                continue
            hit += 1
            for k, poly in layouts.items():
                best[k].append(max(margin(poly, S["C"][i, :2]) for i in idx))
        res[name] = dict(coverage=hit / len(pts), n=len(pts),
                         margin={k: (min(v) if v else float("nan")) for k, v in best.items()})
    return res


def render_pose(d: Design, S, target, path, down=True):
    """Render the sample closest to `target` with the right orientation."""
    m = S["model"]; dat = mujoco.MjData(m)
    want = np.array([0, 0, -1.0]) if down else np.array([1.0, 0, 0])
    good = np.where(S["A"] @ want >= 0.87)[0]
    i = good[np.argmin(np.linalg.norm(S["P"][good] - np.array(target), axis=1))]
    dat.qpos[S["adr"]] = S["Q"][i]
    dat.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "elbow_l")]] = math.radians(-90)
    mujoco.mj_kinematics(m, dat)
    r = mujoco.Renderer(m, 600, 800)
    cam = mujoco.MjvCamera(); cam.lookat[:] = [0.25, -0.1, 0.65]; cam.distance = 2.3; cam.azimuth = -130; cam.elevation = -12
    r.update_scene(dat, cam)
    import imageio
    imageio.imwrite(path, r.render())


# ---------------------------------------------------------------- IK-based reach
from scipy.optimize import least_squares  # noqa: E402


TUCK = {"pitch": 0.0, "roll": math.radians(8), "yaw": 0.0, "elbow": math.radians(-140), "wpitch": math.radians(60), "wroll": 0.0}


# Alternative park poses for the idle arm (used when the default one collides, e.g. the torso is low for floor work)
TUCKS = [TUCK,
         {"pitch": math.radians(-35), "roll": math.radians(12), "yaw": 0.0, "elbow": math.radians(-150), "wpitch": math.radians(70), "wroll": 0.0},
         {"pitch": math.radians(-70), "roll": math.radians(10), "yaw": 0.0, "elbow": math.radians(-150), "wpitch": math.radians(90), "wroll": 0.0},
         {"pitch": math.radians(40), "roll": math.radians(10), "yaw": 0.0, "elbow": math.radians(-120), "wpitch": math.radians(30), "wroll": 0.0}]


def tuck(m, dat, side, pose=None):
    for j, v in (pose or TUCK).items():
        dat.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{j}_{side}")]] = v


def in_collision(m, dat, pen=0.002):
    """True if any robot geoms interpenetrate by more than `pen` (adjacent bodies are filtered by MuJoCo)."""
    mujoco.mj_collision(m, dat)
    return any(dat.contact[i].dist < -pen for i in range(dat.ncon))


class Reacher:
    """Bounded least-squares IK (lift + 6 arm joints) with orientation goal and collision rejection."""

    def __init__(self, d: Design, check_collision: bool = True):
        self.m = mujoco.MjModel.from_xml_string(build_xml(d))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.check = check_collision
        self.jn = ["lift"] + [f"{j}_r" for j in ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")]
        ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.jn]
        self.adr = [m.jnt_qposadr[i] for i in ids]
        self.lo = np.array([m.jnt_range[i][0] for i in ids]); self.hi = np.array([m.jnt_range[i][1] for i in ids])
        tuck(m, self.d, "l")
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
        self.hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hand_r")
        self.robot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self.rejected = 0
        self.park = {}          # solution -> idle-arm park pose, when it wasn't the default
        self.roll_free = self._roll_free() if check_collision else None

    def _roll_free(self, pen=0.002):
        """Smallest shoulder roll (rad) at which the roll body's hulls clear the torso's at every pitch angle.
        With the inline roll (D24) the yaw block sits 19 mm off the detach plane (the belt roll's sat 37 mm off), and at
        the −15° adduction limit its hull is 5 mm into the torso pitch-module hull. So the roll's lower limit depends on
        pitch, which a planner has to respect. The IK below has no collision term: from most starts it presses the arm
        onto the roll limit and is then rejected, even where the target can be reached with less adduction."""
        m, d = self.m, self.d
        rb, tb = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in ("roll_r", "torso"))
        q = np.clip(np.array([0.7, 0, 0, 0, math.radians(-90), 0, 0]), self.lo, self.hi)
        lo = self.lo[2]
        for roll in np.arange(lo, lo + math.radians(30), math.radians(0.5)):
            hit = False
            for pitch in np.arange(self.lo[1], self.hi[1] + 1e-9, math.radians(5)):
                q[1], q[2] = pitch, roll
                self.fk(q)
                mujoco.mj_collision(m, d)
                hit = any(d.contact[i].dist < -pen and {m.geom_bodyid[d.contact[i].geom1], m.geom_bodyid[d.contact[i].geom2]} == {rb, tb}
                          for i in range(d.ncon))
                if hit:
                    break
            if not hit:
                return float(roll) if roll > lo + 1e-9 else None
        return None

    def _accept(self, x):
        """Collision check of an IK solution (the idle arm may re-park; keep the first park pose that clears)."""
        if not (self.check and in_collision(self.m, self.d)):
            return True
        for pose in TUCKS[1:]:
            tuck(self.m, self.d, "l", pose)
            self.fk(x)
            if not in_collision(self.m, self.d):
                self.park[tuple(x)] = pose
                tuck(self.m, self.d, "l")
                return True
        tuck(self.m, self.d, "l")
        self.rejected += 1
        return False

    def fk(self, q):
        self.d.qpos[self.adr] = q
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), -self.d.xmat[self.hand].reshape(3, 3)[:, 2]

    def com(self, q):
        tuck(self.m, self.d, "l", self.park.get(tuple(q)))     # COM with the park pose this solution actually uses
        self.d.qpos[self.adr] = q
        mujoco.mj_kinematics(self.m, self.d); mujoco.mj_comPos(self.m, self.d)
        c = self.d.subtree_com[self.robot].copy()
        tuck(self.m, self.d, "l")
        return c

    def solve(self, target, axis, seeds=8, rng=None):
        """rng: None (default) seeds the random restarts from this call's inputs (call_rng), so a target gets the same
        solutions whatever was solved before it; pass a Generator to override."""
        target, axis = np.asarray(target, float), np.asarray(axis, float)
        rng = call_rng(1, target, axis, seeds) if rng is None else rng

        def res(q):
            p, a = self.fk(q)
            return np.concatenate([p - target, 0.12 * (a - axis)])
        sols = []
        r_ = math.radians
        structured = [  # sensible starts: arm forward/down, elbow bent, wrist compensating
            [0.7, r_(-90), r_(10), 0, r_(-10), r_(90 if axis[2] < 0 else 0), 0],
            [0.5, r_(-45), r_(10), 0, r_(-60), r_(100 if axis[2] < 0 else 10), 0],
            [0.9, r_(-120), r_(10), 0, r_(-30), r_(60 if axis[2] < 0 else -60), 0],
            [0.45, r_(-20), r_(20), 0, r_(-20), r_(40 if axis[2] < 0 else 0), 0]]
        starts = [np.clip(np.array(s_), self.lo, self.hi) for s_ in structured] + \
                 [rng.uniform(self.lo, self.hi) for _ in range(seeds)]
        def ok(x):
            p, a = self.fk(x)
            return np.linalg.norm(p - target) < 0.01 and a @ axis > math.cos(math.radians(20))
        for q0 in starts:
            r = least_squares(res, q0, bounds=(self.lo, self.hi), xtol=1e-9, ftol=1e-10, max_nfev=1500)
            if not ok(r.x):
                continue
            if self._accept(r.x):
                sols.append(r.x)
            elif self.roll_free is not None and r.x[2] < self.roll_free:
                # rejected while adducted into the torso: solve again from there with roll kept clear of it (_roll_free)
                lo = self.lo.copy(); lo[2] = self.roll_free
                r2 = least_squares(res, np.clip(r.x, lo, self.hi), bounds=(lo, self.hi), xtol=1e-9, ftol=1e-10, max_nfev=1500)
                if ok(r2.x) and self._accept(r2.x):
                    sols.append(r2.x)
        return sols


def call_rng(base, *inputs):
    """A Generator seeded from a call's inputs (rounded to 1 µm / 1e-6), for the IK solvers' random restarts. They used to
    share one module-level default Generator across calls (`rng=np.random.default_rng(1)` as a default argument), so a
    target's solutions, and every study built on them (tip, basket, joints, fea, sim, the inline study), depended on what
    had been solved before it in the same process."""
    v = np.concatenate([np.atleast_1d(np.asarray(x, float)).ravel() for x in inputs])
    return np.random.default_rng([int(base)] + [int(k) & 0xFFFFFFFF for k in np.round(v * 1e6).astype(np.int64)])


def ik_study(d: Design, layouts, ys=np.arange(-0.40, -0.04, 0.075), seeds=3):
    R = Reacher(d)
    out = {}
    for name, t in TARGETS.items():
        axis = list(t["axis"]) if "axis" in t else ([0, 0, -1.0] if t["down"] else [1.0, 0, 0])
        pts = [(x, y, t["z"]) for x in t["xs"] for y in ys]
        hit, best, hs = 0, {k: [] for k in layouts}, []
        for p in pts:
            sols = R.solve(p, axis, seeds)
            if not sols:
                continue
            hit += 1
            coms = [R.com(q)[:2] for q in sols]
            hs += [R.com(q)[2] for q in sols]
            for k, poly in layouts.items():
                best[k].append(max(margin(poly, c) for c in coms))
        out[name] = dict(coverage=hit / len(pts), n=len(pts), info=t.get("info", False),
                         margin={k: (min(v) if v else float("nan")) for k, v in best.items()},
                         com_h=float(np.mean(hs)) if hs else float("nan"))
    return out


def render_ik(d: Design, targets, path, layout_r=0.21):
    """Render one image per (label, point, down) target using an IK solution; returns the paths."""
    import imageio
    R = Reacher(d)
    m, dat = R.m, R.d
    out = []
    for label, p, down in targets:
        sols = R.solve(p, [0, 0, -1.0] if down else [1.0, 0, 0], seeds=6)
        if not sols:
            continue
        q = max(sols, key=lambda q: margin(support_polygon("xdrive", layout_r, 0), R.com(q)[:2]))
        R.fk(q)
        r = mujoco.Renderer(m, 540, 720)
        cam = mujoco.MjvCamera(); cam.lookat[:] = [0.2, -0.1, 0.6]; cam.distance = 2.4; cam.azimuth = -125; cam.elevation = -10
        r.update_scene(dat, cam)
        f = f"{path}_{label}.png"
        imageio.imwrite(f, r.render())
        out.append(f)
    return out


class BiReacher:
    """Two-arm IK sharing the lift: both hands to targets with given approach axes, collision-free."""

    def __init__(self, d: Design):
        self.m = mujoco.MjModel.from_xml_string(build_xml(d)); self.d = mujoco.MjData(self.m); m = self.m
        self.jn = ["lift"] + [f"{j}_{s}" for s in ("r", "l") for j in ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")]
        ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.jn]
        self.adr = [m.jnt_qposadr[i] for i in ids]
        self.lo = np.array([m.jnt_range[i][0] for i in ids]); self.hi = np.array([m.jnt_range[i][1] for i in ids])
        self.site = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"grasp_{s}") for s in "rl"}
        self.hand = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"hand_{s}") for s in "rl"}

    def solve(self, tr, ar, tl, al, seeds=10, rng=None):
        tr, ar, tl, al = (np.asarray(x, float) for x in (tr, ar, tl, al))
        rng = call_rng(3, tr, ar, tl, al, seeds) if rng is None else rng     # per call (see Reacher.solve)

        def res(q):
            self.d.qpos[self.adr] = q; mujoco.mj_kinematics(self.m, self.d)
            out = []
            for s, t, a in (("r", tr, ar), ("l", tl, al)):
                out += list(self.d.site_xpos[self.site[s]] - t) + list(0.12 * (-self.d.xmat[self.hand[s]].reshape(3, 3)[:, 2] - a))
            return np.array(out)
        for _ in range(seeds):
            r = least_squares(res, rng.uniform(self.lo, self.hi), bounds=(self.lo, self.hi), max_nfev=3000)
            e = res(r.x)
            if np.linalg.norm(e[0:3]) < 0.01 and np.linalg.norm(e[6:9]) < 0.01 and \
               np.linalg.norm(e[3:6]) < 0.12 * 0.35 and np.linalg.norm(e[9:12]) < 0.12 * 0.35 and not in_collision(self.m, self.d):
                return r.x
        return None


BIMANUAL = {  # (right target, right approach axis, left target, left approach axis)
    "hold a box at the table (hands 24 cm apart, z 0.85)": ((0.42, -0.12, 0.85), (0, 1, 0), (0.42, 0.12, 0.85), (0, -1, 0)),
    "hand-to-hand at chest height (handover)": ((0.38, -0.02, 0.80), (0, 1, 0), (0.38, 0.06, 0.80), (0, -1, 0)),
    "lift a box from the floor (hands 30 cm apart, z 0.10)": ((0.42, -0.15, 0.10), (0, 1, 0), (0.42, 0.15, 0.10), (0, -1, 0)),
    "fold a towel on the table (hands 40 cm apart, down, z 0.80)": ((0.45, -0.20, 0.80), (0, 0, -1), (0.45, 0.20, 0.80), (0, 0, -1)),
}


def bimanual_study(d: Design):
    B = BiReacher(d)
    return {k: B.solve(*v) is not None for k, v in BIMANUAL.items()}
