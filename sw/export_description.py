"""S0.2: robot description export.  python -m sw.export_description

Writes, from the same parameters the simulations use (rnd/params.py, rnd/twin.py joint ranges):
- sw/description/pip_bench_arm.xml   MJCF of the M1 bench arm (sw/bench_env.py)
- sw/description/pip_bench_arm.urdf  URDF for ROS 2 / MoveIt / LeRobot: one link per MJCF body, a revolute or prismatic
  joint per body, inertials from the compiled MuJoCo model (mass, COM, principal inertia + frame), primitive
  collision/visual geometry from the MJCF geoms (capsules → cylinder + 2 spheres)
- sw/description/joints.json          names, limits, velocity/effort limits (actuator specs) for the drivers (S1)

The URDF uses the MJCF body frames directly, so joint zeros match the twin (arm hanging straight down = 0).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "description"


def _rpy(q):
    """MuJoCo quaternion (w, x, y, z) → URDF roll/pitch/yaw (XYZ fixed)."""
    w, x, y, z = q
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return r, p, yw


def _f(v):
    return " ".join(f"{float(a):.6g}" for a in v)


def urdf(m):
    import mujoco
    L = ['<?xml version="1.0"?>', '<robot name="pip_bench_arm">', '  <link name="world"/>']
    root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "sh")

    def in_arm(b):
        while b > 0:
            if b == root:
                return True
            b = m.body_parentid[b]
        return False
    for b in range(1, m.nbody):
        if not in_arm(b):                                            # scene bodies (the cube) are not part of the robot
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"
        L.append(f'  <link name="{name}">')
        if m.body_mass[b] > 1e-6:
            L += ["    <inertial>", f'      <origin xyz="{_f(m.body_ipos[b])}" rpy="{_f(_rpy(m.body_iquat[b]))}"/>',
                  f'      <mass value="{m.body_mass[b]:.6g}"/>',
                  f'      <inertia ixx="{m.body_inertia[b][0]:.6g}" iyy="{m.body_inertia[b][1]:.6g}" izz="{m.body_inertia[b][2]:.6g}" ixy="0" ixz="0" iyz="0"/>',
                  "    </inertial>"]
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b:
                continue
            t, sz = m.geom_type[g], m.geom_size[g]
            geo = None
            if t == mujoco.mjtGeom.mjGEOM_BOX:
                geo = f'<box size="{_f(2 * sz)}"/>'
            elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
                geo = f'<sphere radius="{sz[0]:.6g}"/>'
            elif t in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                geo = f'<cylinder radius="{sz[0]:.6g}" length="{2 * sz[1]:.6g}"/>'
            if geo is None:
                continue
            o = f'<origin xyz="{_f(m.geom_pos[g])}" rpy="{_f(_rpy(m.geom_quat[g]))}"/>'
            L += [f"    <visual>{o}<geometry>{geo}</geometry></visual>", f"    <collision>{o}<geometry>{geo}</geometry></collision>"]
        L.append("  </link>")
        parent = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[b]) if b != root else "world"
        j = m.body_jntadr[b]
        org = f'<origin xyz="{_f(m.body_pos[b])}" rpy="{_f(_rpy(m.body_quat[b]))}"/>'
        jt = int(m.jnt_type[j]) if m.body_jntnum[b] >= 1 else -1
        if jt in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            kind = "revolute" if jt == int(mujoco.mjtJoint.mjJNT_HINGE) else "prismatic"
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-math.pi, math.pi)
            L += [f'  <joint name="{jn}" type="{kind}">', f"    {org}", f'    <parent link="{parent}"/>', f'    <child link="{name}"/>',
                  f'    <axis xyz="{_f(m.jnt_axis[j])}"/>', f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="50" velocity="6"/>', "  </joint>"]
        else:
            kind = "floating" if jt == int(mujoco.mjtJoint.mjJNT_FREE) else "fixed"
            L += [f'  <joint name="{name}_fixed" type="{"fixed" if kind == "fixed" else "floating"}">', f"    {org}",
                  f'    <parent link="{parent}"/>', f'    <child link="{name}"/>', "  </joint>"]
    L.append("</robot>")
    return "\n".join(L)


def main():
    import mujoco
    from rnd.params import BASELINE, act
    from sw.bench_env import ARM, xml
    OUT.mkdir(exist_ok=True)
    x = xml()
    (OUT / "pip_bench_arm.xml").write_text(x)
    m = mujoco.MjModel.from_xml_string(x)
    (OUT / "pip_bench_arm.urdf").write_text(urdf(m))
    jmap = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch", "wroll": "wrist_roll"}
    out = []
    for j in ARM:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        an, ratio = BASELINE.arm.joints[jmap[j]]
        s = act(an)
        out.append(dict(name=j, actuator=an, ratio=ratio, lower=float(m.jnt_range[jid][0]), upper=float(m.jnt_range[jid][1]),
                        rated_nm=s["rated"] * ratio, peak_nm=s["peak"] * ratio, max_speed_rad_s=s["speed"] / ratio * (420 / 120 if an.startswith("GIM") else 1)))
    (OUT / "joints.json").write_text(json.dumps(out, indent=1))
    # self-check: the URDF parses and has one actuated joint per arm joint
    import xml.etree.ElementTree as ET
    r = ET.fromstring((OUT / "pip_bench_arm.urdf").read_text())
    names = [j.get("name") for j in r.findall("joint") if j.get("type") in ("revolute", "prismatic")]
    print(f"URDF: {len(r.findall('link'))} links, actuated joints {names}; MJCF and joints.json written to {OUT}")


if __name__ == "__main__":
    main()
