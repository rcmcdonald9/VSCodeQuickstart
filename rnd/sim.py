"""Twin v1: dynamic MuJoCo simulation of a fetch task, run within the real actuator limits.

Task: drive to a can on the floor, pick it (top-down), drive to a table, place it on the table top, let go, retract.

Model, derived from the kinematic twin (rnd/twin.py):
- **Base:** holonomic planar joints, force-limited to what the four GIM omni wheels can deliver.
- **Arm joints, lift, gripper:** position servos whose force ranges are the joints' *peak* torques (motor × belt × efficiency).
- **Gripper:** two sliding TPU-padded fingers with μ = 1.0 (the CAD jaw is SO-100-style; the slide fingers are a modelling simplification).
- **Robot geoms:** collide with the can but not with each other (self-collision is already guaranteed by the kinematic study).
- **Tipping:** checked with a cart-table ZMP from the whole-robot COM acceleration.

Outputs: docs/rnd/sim.md (success, per-joint torque use vs rated/peak, ZMP margin), a contact sheet, and an MP4.
python -m rnd.sim
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from . import twin  # noqa: E402
from .arm_sizing import capacity  # noqa: E402
from .params import BASELINE, Design, act, G  # noqa: E402

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
CAN = dict(r=0.033, h=0.115, m=0.35, xy=(0.85, -0.20))            # a soda-can-sized object
TABLE = dict(center=(1.65, -0.15), half=(0.25, 0.35), top=0.75)
GRIP_REL = (0.45, -0.20)                                            # where the robot puts the object relative to itself
DT = 0.002
ARM = ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")
JOINT_NAME = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch", "wroll": "wrist_roll"}


def wheel_limits(d: Design):
    g = act(d.base.wheel_motor)
    r = d.base.wheel_d / 2
    f_rated, f_peak = g["rated"] / r, g["peak"] / r
    return dict(fx_rated=2 * f_rated, fx_peak=2 * f_peak, tz_peak=4 * f_peak * d.base.wheel_radius_pos)


def build_xml(d: Design = BASELINE, can_offset=(0.0, 0.0), can_mass=None) -> str:
    root = ET.fromstring(twin.build_xml(d))
    world = root.find("worldbody")
    for g in world.findall("geom"):
        if g.get("type") == "plane":
            g.set("contype", "2"); g.set("conaffinity", "2"); g.set("friction", "0.8 0.01 0.001")
        elif g.get("name") == "table":
            world.remove(g)
    robot = world.find("body[@name='robot']")
    for i, (n, t, ax) in enumerate((("base_x", "slide", "1 0 0"), ("base_y", "slide", "0 1 0"), ("base_yaw", "hinge", "0 0 1"))):
        robot.insert(i, ET.Element("joint", name=n, type=t, axis=ax, damping="30" if t == "slide" else "10"))
    # robot geoms: collide with the can (contype 1) but not with each other or the floor (conaffinity 0)
    for g in robot.iter("geom"):
        if g.get("contype") != "0":
            g.set("contype", "1"); g.set("conaffinity", "0")
    for side in ("r", "l"):
        hand = robot.find(f".//body[@name='hand_{side}']")
        for g in list(hand.findall("geom")):
            if g.get("type") == "mesh":                  # gripper hull would fill the space between the jaws
                g.set("contype", "0"); g.set("conaffinity", "0")
        pay = hand.find(f"body[@name='payload_{side}']")
        if pay is not None:
            hand.remove(pay)
        z = -(d.arm.l_hand - 0.04)
        for k, sgn in (("a", 1), ("b", -1)):
            f = ET.SubElement(hand, "body", name=f"finger_{k}_{side}", pos=f"{sgn * 0.06} 0 {z + 0.02}")
            ET.SubElement(f, "joint", name=f"fing_{k}_{side}", type="slide", axis=f"{-sgn} 0 0", range="0 0.06", damping="8", armature="0.05")
            ET.SubElement(f, "geom", type="box", size="0.006 0.018 0.035", mass="0.03", rgba=".95 .45 .1 1",
                          contype="1", conaffinity="0", friction="1.0 0.02 0.002", condim="6", solref="0.01 1", solimp="0.95 0.99 0.001")
    # objects
    can = ET.SubElement(world, "body", name="can", pos=f"{CAN['xy'][0] + can_offset[0]} {CAN['xy'][1] + can_offset[1]} {CAN['h'] / 2 + 0.001}")
    ET.SubElement(can, "freejoint")
    ET.SubElement(can, "geom", type="cylinder", size=f"{CAN['r']} {CAN['h'] / 2}", mass=str(can_mass or CAN["m"]), rgba=".85 .15 .1 1",
                  contype="3", conaffinity="3", friction="1.0 0.02 0.002", condim="4")
    tc, th = TABLE["center"], TABLE["half"]
    ET.SubElement(world, "geom", name="table", type="box", pos=f"{tc[0]} {tc[1]} {TABLE['top'] / 2}", size=f"{th[0]} {th[1]} {TABLE['top'] / 2}",
                  rgba=".55 .42 .3 1", contype="2", conaffinity="3")     # the robot collides with the table (must not touch it)
    # actuators: position servos limited to each joint's peak torque
    acts = ET.SubElement(root, "actuator")
    wl = wheel_limits(d)
    ET.SubElement(acts, "position", name="a_base_x", joint="base_x", kp="3000", kv="600", forcerange=f"{-wl['fx_peak']:.0f} {wl['fx_peak']:.0f}")
    ET.SubElement(acts, "position", name="a_base_y", joint="base_y", kp="3000", kv="600", forcerange=f"{-wl['fx_peak']:.0f} {wl['fx_peak']:.0f}")
    ET.SubElement(acts, "position", name="a_base_yaw", joint="base_yaw", kp="800", kv="150", forcerange=f"{-wl['tz_peak']:.0f} {wl['tz_peak']:.0f}")
    lift_peak = act(d.upper.lift_motor)["peak"] / 0.0159
    ET.SubElement(acts, "position", name="a_lift", joint="lift", kp="20000", kv="2500", forcerange=f"{-lift_peak:.0f} {lift_peak:.0f}")
    for side in ("r", "l"):
        for j in ARM:
            full = JOINT_NAME[j]
            if full == "wrist_roll":
                peak = act(d.arm.joints[full][0])["peak"]; kp, kv = 20, 1.5
            else:
                peak = capacity(d, full)[3]
                kp, kv = (600, 60) if j in ("pitch", "roll", "yaw", "elbow") else (120, 8)
            ET.SubElement(acts, "position", name=f"a_{j}_{side}", joint=f"{j}_{side}", kp=str(kp), kv=str(kv), forcerange=f"{-peak:.2f} {peak:.2f}")
        ET.SubElement(acts, "position", name=f"a_fing_a_{side}", joint=f"fing_a_{side}", kp="1500", kv="60", forcerange="-50 50")
    eq = ET.SubElement(root, "equality")        # fingers are coupled (rack-and-pinion), so the object can't push the pair sideways
    for side in ("r", "l"):
        ET.SubElement(eq, "joint", joint1=f"fing_b_{side}", joint2=f"fing_a_{side}", polycoef="0 1 0 0 0", solref="0.005 1")
    opt = root.find("option")
    opt.set("timestep", str(DT)); opt.set("integrator", "implicitfast"); opt.set("cone", "elliptic"); opt.set("impratio", "10")
    ET.SubElement(opt, "flag", multiccd="enable")       # several contact points per finger/can pair (no pivoting on one point)
    vis = root.find("visual")
    vis.find("global").set("offwidth", "960"); vis.find("global").set("offheight", "720")
    return ET.tostring(root, encoding="unicode")


# ------------------------------------------------------------------ planning (IK on the kinematic twin)
class Planner:
    def __init__(self, d: Design):
        self.R = twin.Reacher(d)

    def ik(self, target, axis, q0):
        R = self.R

        q0 = np.asarray(q0, float)

        def res(q):
            p, a = R.fk(q)
            return np.concatenate([p - target, 0.15 * (a - axis), 0.02 * (q - q0)])     # continuity: no wrist spin
        r = least_squares(res, q0, bounds=(R.lo, R.hi), xtol=1e-10, ftol=1e-12, max_nfev=800)
        p, a = R.fk(r.x)
        return r.x, float(np.linalg.norm(p - target))

    def solve(self, target, axis):
        sols = self.R.solve(np.asarray(target), np.asarray(axis), seeds=6)
        if not sols:
            raise RuntimeError(f"no collision-free IK for {target}")
        return min(sols, key=lambda q: abs(q[6]))          # least wrist roll

    def line(self, q0, a, b, axis, n=10):
        """Joint waypoints along a straight Cartesian line a→b (continuation IK). q0=None continues the previous line."""
        qs, q = [], (self._last if q0 is None else q0)
        for t in np.linspace(0, 1, n + 1)[1:]:
            q, err = self.ik(np.asarray(a) + t * (np.asarray(b) - np.asarray(a)), np.asarray(axis, float), q)
            qs.append(q.copy())
        self._last = q
        return qs


def min_jerk(t):
    t = min(max(t, 0.0), 1.0)
    return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5


_PLAN_CACHE = {}


def plan(d: Design = BASELINE):
    """Plan once per design (IK seeds are random; the plan doesn't depend on the perception error)."""
    if id(d) not in _PLAN_CACHE:
        _PLAN_CACHE[id(d)] = _plan(d)
    return _PLAN_CACHE[id(d)]


def _plan(d: Design = BASELINE):
    """Return a list of segments: (duration, base_xy_from, base_xy_to, [q waypoints (lift + 6 right-arm joints)], grip)."""
    P = Planner(d)
    tuck = [twin.TUCK[j] for j in ARM]
    down = (0, 0, -1)
    gx, gy = GRIP_REL
    zc = CAN["h"] / 2 + 0.005
    z_place = TABLE["top"] + CAN["h"] / 2 + 0.006
    q_pre = P.solve((gx, gy, zc + 0.12), down)
    grasp = ("REPLAN_GRASP", P, q_pre, (gx, gy), zc)
    up = ("REPLAN_UP",)
    q_pre_place = P.solve((gx, gy, z_place + 0.12), down)
    place = ("REPLAN_PLACE", P, q_pre_place, (gx, gy), z_place)
    retract = ("REPLAN_RETRACT",)
    q_tuck = np.array([0.70] + tuck)
    b0 = (0.0, 0.0)
    b1 = (CAN["xy"][0] - gx, CAN["xy"][1] - gy)
    b2 = (TABLE["center"][0] - TABLE["half"][0] + 0.15 - gx, CAN["xy"][1] - gy)
    b3 = (b2[0] - 0.40, b2[1])
    OPEN, CLOSED = 0.0, 1.0
    return [
        ("settle", 1.0, b0, b0, [q_tuck], OPEN),
        ("drive to the can", 2.5, b0, b1, [q_tuck], OPEN),
        ("reach above the can", 2.5, b1, b1, [q_pre], OPEN),
        ("descend", 1.5, b1, b1, grasp, OPEN),
        ("close the gripper", 1.0, b1, b1, ("HOLD_GRASP",), CLOSED),
        ("lift", 1.5, b1, b1, up, CLOSED),
        ("carry pose", 2.0, b1, b1, ("REPLAN_CARRY", z_place), CLOSED),
        ("drive to the table", 3.0, b1, b2, ("HOLD_PREV",), CLOSED),
        ("lower onto the table", 1.5, b2, b2, place, CLOSED),
        ("release", 1.0, b2, b2, ("HOLD",), OPEN),
        ("retract", 1.5, b2, b2, retract, OPEN),
        ("back away", 1.5, b2, b3, ("HOLD_RETRACT",), OPEN),
        ("tuck", 2.0, b3, b3, [q_tuck], OPEN),
        ("settle", 1.0, b3, b3, [q_tuck], OPEN),
    ]


# ------------------------------------------------------------------ simulation
def apply_perturb(m, pt):
    """Monte Carlo imperfections (rnd/mc.py): torque derating, heavier arms, joint friction (gears/belts), finger friction."""
    names = lambda obj, i: mujoco.mj_id2name(m, obj, i) or ""
    for i in range(m.nu):
        n = names(mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if n.startswith(("a_pitch", "a_roll", "a_yaw", "a_elbow", "a_wpitch", "a_wroll", "a_lift", "a_base")):
            m.actuator_forcerange[i] *= pt.get("derate", 1.0)
    for b in range(m.nbody):
        n = names(mujoco.mjtObj.mjOBJ_BODY, b)
        if n.startswith(("sh_", "roll_", "upper_", "fore_", "wrist_", "hand_")):
            m.body_mass[b] *= pt.get("mass", 1.0)
            m.body_inertia[b] *= pt.get("mass", 1.0)
    for j in range(m.njnt):
        n = names(mujoco.mjtObj.mjOBJ_JOINT, j)
        if n.split("_")[0] in ("pitch", "roll", "yaw", "elbow", "wpitch"):
            m.dof_frictionloss[m.jnt_dofadr[j]] = pt.get("joint_friction", 0.0)
    for g in range(m.ngeom):
        if names(mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]).startswith("finger_"):
            m.geom_friction[g][0] = pt.get("finger_mu", 1.0)
    # body_subtreemass is cached at compile time and MuJoCo's subtree COM divides by it: refresh it after the mass change
    m.body_subtreemass[:] = m.body_mass
    for b in range(m.nbody - 1, 0, -1):
        m.body_subtreemass[m.body_parentid[b]] += m.body_subtreemass[b]


def run(d: Design = BASELINE, video: bool = True, can_offset=(0.0, 0.0), can_mass=None, wrist_cam: bool = True, seed: int = 0,
        perturb: dict | None = None):
    """can_offset: where the can really is vs. where the robot believes it is (perception error)."""
    m = mujoco.MjModel.from_xml_string(build_xml(d, can_offset, can_mass))
    if perturb:
        apply_perturb(m, perturb)
    calib = np.asarray((perturb or {}).get("calib", np.zeros(len(ARM))))      # joint zero errors (rad), unseen by the planner
    dat = mujoco.MjData(m)
    aid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    jad = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    segs = plan(d)
    # initial state: tucked arms, lift 0.70
    park_l = twin.TUCKS[3]                       # idle arm pitched back, out of the way of furniture
    for side in ("r", "l"):
        for j in ARM:
            v = twin.TUCK[j] if side == "r" else park_l[j]
            dat.qpos[jad(f"{j}_{side}")] = v
            dat.ctrl[aid[f"a_{j}_{side}"]] = v
    dat.qpos[jad("lift")] = 0.70
    dat.ctrl[aid["a_lift"]] = 0.70
    for side in ("r", "l"):
        dat.ctrl[aid[f"a_fing_a_{side}"]] = 0.0
    mujoco.mj_forward(m, dat)
    robot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")
    can = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "can")
    table_gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    robot_geoms = {g for g in range(m.ngeom) if m.geom_contype[g] == 1}
    table_hits = []
    poly = twin.support_polygon("xdrive", d.base.wheel_radius_pos, 0)
    names = [f"a_{j}_r" for j in ARM] + ["a_lift", "a_base_x", "a_base_y", "a_fing_a_r"]
    force_log = {n: [] for n in names}
    zmp_log, com_prev, vel_prev, t_log, seg_log, com_log = [], None, None, [], [], []
    renderer = mujoco.Renderer(m, 480, 640) if video else None
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 2.6, -18, -60
    frames = []
    t = 0.0
    q_prev = np.array([0.70] + [twin.TUCK[j] for j in ARM])
    grip_prev = 0.0
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
    last = {}
    trace = []
    rng = np.random.default_rng(seed)
    P_main = segs[3][4][1]                                    # the planner instance (from the grasp marker)
    for name, dur, bA, bB, qs, grip in segs:
        if isinstance(qs, tuple) and qs[0] == "REPLAN_GRASP":
            _, P, q_pg, (gx, gy), zc = qs
            if wrist_cam:      # wrist camera re-measures the can from above (σ 3 mm, ESTIMATE) and the descent is re-planned
                base = np.array([dat.qpos[jad("base_x")], dat.qpos[jad("base_y")]])
                meas = dat.xpos[can][:2] + rng.normal(0, 0.003, 2) - base
                mx, my = float(meas[0]), float(meas[1])
            else:
                mx, my = gx, gy
            qs = P.line(q_pg, (gx, gy, zc + 0.12), (mx, my, zc + 0.12), (0, 0, -1), 3) + \
                P.line(None, (mx, my, zc + 0.12), (mx, my, zc), (0, 0, -1), 8)
            last["grasp"] = (qs[-1], (mx, my, zc), P)
        elif isinstance(qs, tuple) and qs[0] == "HOLD_PREV":
            qs = [q_prev.copy()]
        elif isinstance(qs, tuple) and qs[0] == "HOLD_GRASP":
            qs = [last["grasp"][0]]
        elif isinstance(qs, tuple) and qs[0] == "REPLAN_UP":
            q_g, (mx, my, zc), P = last["grasp"]
            qs = P.line(q_g, (mx, my, zc), (mx, my, zc + 0.25), (0, 0, -1), 8)
            last["up"] = (qs[-1], (mx, my, zc + 0.25))
        elif isinstance(qs, tuple) and qs[0] == "REPLAN_CARRY":
            # Cartesian carry (hand kept pointing down): a joint-space move swung the can through 80° of tilt
            q_u, (mx, my, zu) = last["up"]
            gx, gy = GRIP_REL
            qs = P_main.line(q_u, (mx, my, zu), (gx, gy, qs[1] + 0.12), (0, 0, -1), 14)
        elif isinstance(qs, tuple) and qs[0] == "REPLAN_PLACE":
            _, P, q_pp, (gx, gy), z_place = qs
            q_pp = q_prev
            dz = dat.site_xpos[site][2] - dat.xpos[can][2]          # object offset in the hand, measured
            zt = TABLE["top"] + CAN["h"] / 2 + dz + 0.004
            qs = P.line(q_pp, (gx, gy, z_place + 0.12), (gx, gy, zt), (0, 0, -1), 8)
            last["place"] = (qs[-1], (gx, gy, zt), P)
        elif isinstance(qs, tuple) and qs[0] == "HOLD":
            qs = [last["place"][0]]
        elif isinstance(qs, tuple) and qs[0] == "REPLAN_RETRACT":
            q_end, (gx, gy, zt), P = last["place"]
            qs = P.line(q_end, (gx, gy, zt), (gx, gy, zt + 0.15), (0, 0, -1), 6)
            last["retract"] = qs
        elif isinstance(qs, tuple) and qs[0] == "HOLD_RETRACT":
            qs = [last["retract"][-1]]
        n = int(dur / DT)
        q_from = q_prev.copy()
        g_from = grip_prev
        for i in range(n):
            s = (i + 1) / n
            # joint targets: piecewise min-jerk through the waypoint list
            k = min(int(s * len(qs)), len(qs) - 1)
            seg_s = s * len(qs) - k
            qa = q_from if k == 0 else qs[k - 1]
            q_t = qa + min_jerk(seg_s) * (qs[k] - qa)
            dat.ctrl[aid["a_lift"]] = q_t[0]
            for j, v, c in zip(ARM, q_t[1:], calib):
                dat.ctrl[aid[f"a_{j}_r"]] = v + c
            bx = bA[0] + min_jerk(s) * (bB[0] - bA[0]); by = bA[1] + min_jerk(s) * (bB[1] - bA[1])
            dat.ctrl[aid["a_base_x"]], dat.ctrl[aid["a_base_y"]] = bx, by
            # finger slide q: 0 = fully open (100 mm), 0.05 = closed; closing commands full travel so the
            # force-limited servo squeezes the object
            gq = 0.06 * (g_from + min_jerk(s) * (grip - g_from))
            dat.ctrl[aid["a_fing_a_r"]] = gq
            mujoco.mj_step(m, dat)
            t += DT
            if i % 10 == 0:
                for ci in range(dat.ncon):
                    c = dat.contact[ci]
                    gs = {c.geom1, c.geom2}
                    if table_gid in gs and (gs - {table_gid}) & robot_geoms and c.dist < -0.001:
                        table_hits.append((name, round(t, 2)))
                for nm in names:
                    force_log[nm].append(abs(dat.actuator_force[aid[nm]]))
                com = dat.subtree_com[robot].copy()
                if com_prev is not None:
                    vel = (com - com_prev) / (10 * DT)
                    if vel_prev is not None:
                        acc = (vel - vel_prev) / (10 * DT)
                        zmp = com[:2] - com[2] / G * acc[:2]
                        base = np.array([dat.qpos[jad("base_x")], dat.qpos[jad("base_y")]])
                        zmp_log.append(twin.margin(poly, zmp - base))
                        com_log.append(np.r_[com - np.r_[base, 0], acc])
                        t_log.append(t); seg_log.append(name)
                    vel_prev = vel
                com_prev = com
            if video and i % 50 == 0:
                cam.lookat[:] = [dat.qpos[jad("base_x")] + 0.3, dat.qpos[jad("base_y")], 0.6]
                renderer.update_scene(dat, camera=cam)
                frames.append((name, renderer.render().copy()))
        q_prev = qs[-1].copy()
        grip_prev = grip
        trace.append((name, np.round(dat.xpos[can], 3).tolist(), np.round(dat.site_xpos[site], 3).tolist(),
                      round(float(dat.qpos[jad("fing_a_r")]), 3), round(float(dat.qpos[jad("fing_b_r")]), 3)))
    can_pos = dat.xpos[can].copy()
    tc, th = TABLE["center"], TABLE["half"]
    on_table = (abs(can_pos[0] - tc[0]) < th[0] and abs(can_pos[1] - tc[1]) < th[1] and abs(can_pos[2] - (TABLE["top"] + CAN["h"] / 2)) < 0.02)
    upright = abs(dat.xmat[can].reshape(3, 3)[2, 2]) > 0.95
    return dict(force=force_log, zmp=np.array(zmp_log), com=np.array(com_log), t=np.array(t_log), seg=seg_log, can=can_pos, ok=bool(on_table and upright),
                frames=frames, model=m, trace=trace, table_hits=table_hits, data=dat)


def limits(d: Design = BASELINE):
    L = {}
    for j in ARM:
        full = JOINT_NAME[j]
        if full == "wrist_roll":
            a = act(d.arm.joints[full][0]); L[f"a_{j}_r"] = (a["rated"], a["peak"])
        else:
            _, _, rated, peak, _ = capacity(d, full); L[f"a_{j}_r"] = (rated, peak)
    g = act(d.upper.lift_motor)
    L["a_lift"] = (g["rated"] / 0.0159, g["peak"] / 0.0159)
    wl = wheel_limits(d)
    L["a_base_x"] = L["a_base_y"] = (wl["fx_rated"], wl["fx_peak"])
    L["a_fing_a_r"] = (50.0, 150.0)          # coupled pair: total squeeze, STS3215 0.98 / 2.94 N·m on a 20 mm lever
    return L


def report(res, d: Design = BASELINE):
    L = limits(d)
    rows = ["# Twin v1: dynamic fetch task (auto-generated by `python -m rnd.sim`)", "",
            "Drive to a can on the floor → pick it top-down → drive to a table → place it → release → retract, in MuJoCo with **every actuator "
            "force-limited to its real peak** (joint = motor × belt × efficiency; base = 4 GIM omni wheels; lift = GIM on a 20T HTD pulley). "
            f"Can {CAN['m']} kg, Ø{2 * CAN['r'] * 1000:.0f} × {CAN['h'] * 1000:.0f} mm; table top {TABLE['top']} m.", "",
            f"**Result: {'✅ can placed upright on the table' if res['ok'] else '❌ task failed'}** (final can position {np.round(res['can'], 3).tolist()}). "
            f"Robot–table contacts: {len(res['table_hits'])} {'✅' if not res['table_hits'] else '❌ ' + str(sorted(set(n for n, _ in res['table_hits'])))}.", "",
            "| Actuator | Peak use / peak | RMS use / rated (continuous) |", "|---|---|---|"]
    worst_peak, worst_rms = 0, 0
    for n, f in res["force"].items():
        f = np.array(f)
        rated, peak = L[n]
        pk, rms = f.max() / peak, math.sqrt(np.mean(f ** 2)) / rated
        worst_peak, worst_rms = max(worst_peak, pk), max(worst_rms, rms)
        rows.append(f"| {n[2:]} | {f.max():.1f} / {peak:.1f} ({pk:.0%}) | {math.sqrt(np.mean(f ** 2)):.1f} / {rated:.1f} ({rms:.0%}) |")
    zm = res["zmp"]
    i = int(np.argmin(zm))
    rows += ["", f"**Dynamic tip margin (cart-table ZMP to the wheel polygon edge): min {zm.min() * 1000:.0f} mm** during \"{res['seg'][i]}\" at t = {res['t'][i]:.1f} s.",
             "", "![frames](img/sim_fetch_frames.png)", "", "Video: [img/sim_fetch.mp4](img/sim_fetch.mp4)"]
    return "\n".join(rows), worst_peak, worst_rms, zm.min()


def save_media(res):
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw
    frames = res["frames"]
    imageio.mimwrite(DOCS / "img" / "sim_fetch.mp4", [f for _, f in frames], fps=10, quality=7)
    pick = [frames[int(i)] for i in np.linspace(0, len(frames) - 1, 8)]
    tiles = []
    for name, f in pick:
        im = Image.fromarray(f).resize((400, 300))
        ImageDraw.Draw(im).text((8, 8), name, fill=(10, 10, 10))
        tiles.append(im)
    sheet = Image.new("RGB", (1600, 600), "white")
    for k, im in enumerate(tiles):
        sheet.paste(im, ((k % 4) * 400, (k // 4) * 300))
    sheet.save(DOCS / "img" / "sim_fetch_frames.png")


SWEEP = [((0.015, 0.0), 0.35), ((-0.015, 0.0), 0.35), ((0.0, 0.015), 0.35), ((0.0, -0.015), 0.35),
         ((0.0, 0.0), 0.20), ((0.0, 0.0), 0.60), ((0.010, 0.010), 0.50), ((-0.010, -0.010), 0.50)]


def sweep(d: Design = BASELINE):
    out = []
    for k, (off, mass) in enumerate(SWEEP):
        r0 = run(d, video=False, can_offset=off, can_mass=mass, wrist_cam=False)
        r1 = run(d, video=False, can_offset=off, can_mass=mass, wrist_cam=True, seed=k)
        out.append((off, mass, r0["ok"] and not r0["table_hits"], r1["ok"] and not r1["table_hits"]))
    return out


def main():
    import json
    res = run()
    md, pk, rms, zmin = report(res)
    sw = sweep()
    n0 = sum(1 for *_, a, b in sw if a)
    nok = sum(1 for *_, a, b in sw if b)
    md += ("\n\n## Robustness: perception error and object mass\n\n"
           "The can is moved away from where the robot believes it is. *Open loop* trusts the belief. *Wrist camera* re-measures the can from above "
           "(Gaussian noise σ 3 mm, ESTIMATE) and re-plans the descent: the intended head-camera-coarse / wrist-camera-fine architecture.\n\n"
           "| Can offset vs belief (mm) | Mass (kg) | Open loop | With wrist-camera correction |\n|---|---|---|---|\n")
    md += "\n".join(f"| {off[0] * 1000:+.0f}, {off[1] * 1000:+.0f} | {mass:.2f} | {'✅' if a else '❌'} | {'✅' if b else '❌'} |" for off, mass, a, b in sw)
    md += f"\n\n**Open loop {n0}/{len(sw)}; with the wrist-camera correction {nok}/{len(sw)}.**"
    (DOCS / "sim.md").write_text(md)
    (DOCS / "sim.json").write_text(json.dumps({"ok": res["ok"] and not res["table_hits"], "peak_use": pk, "rms_use": rms,
                                               "zmp_min": float(zmin), "sweep_ok": nok, "sweep_n": len(sw)}))
    save_media(res)
    print(md)


if __name__ == "__main__":
    main()
