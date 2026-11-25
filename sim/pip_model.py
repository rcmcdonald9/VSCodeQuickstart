"""Builds a MuJoCo model of Pip v2 straight from the CAD meshes.

* Geometry: body-frame STLs exported by sim/export_meshes.sh (cad/lib.scad).
* Mass/inertia: each part's mesh scaled to its sliced (or measured) mass,
  plus box models for servos, battery, Pi and electronics (sim/masses.yaml).
* Joints and conventions match the software exactly, so the real
  MotionController drives the sim unchanged.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
LEGS = ("FR", "FL", "RR", "RL")


def load_dims():
    return json.loads((ASSETS / "dims.json").read_text())


def load_masses():
    return yaml.safe_load((HERE / "masses.yaml").read_text())


# ------------------------------------------------------------ geometry helpers
def frame_matrix(o, xs, ys, zs):
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = xs, ys, zs, o
    return M


def frames(d):
    """Servo frames (same as cad/lib.scad), mm."""
    return {
        "pitch_FR": frame_matrix([d["bx"], -d["rim_y"], 0], [0, 0, 1], [-1, 0, 0], [0, -1, 0]),
        "knee_FR": frame_matrix([d["bx"], -d["upper_out"], -d["L2"]], [0, 0, 1], [-1, 0, 0], [0, -1, 0]),
        "pan": frame_matrix([d["pan_x"], 0, d["pan_rim_z"]], [-1, 0, 0], [0, -1, 0], [0, 0, 1]),
        "tilt": frame_matrix([d["pan_x"], d["tilt_top_y"], d["tilt_z"]], [0, 0, -1], [1, 0, 0], [0, -1, 0]),
    }


def servo_box(d, M):
    b = trimesh.creation.box(extents=[d["servo_len"], d["servo_wid"], d["servo_case_h"]])
    b.apply_translation([d["servo_len"] / 2 - d["servo_shaft_off"], 0, -d["servo_case_h"] / 2])
    b.apply_transform(M)
    return b


def box(extents, center):
    b = trimesh.creation.box(extents=extents)
    b.apply_translation(center)
    return b


def mesh(name):
    return trimesh.load(ASSETS / f"{name}.stl")


RZ180 = np.diag([-1.0, -1.0, 1.0, 1.0])


def translate(m, t):
    m = m.copy(); m.apply_translation(t); return m


def transform(m, T):
    m = m.copy(); m.apply_transform(T); return m


def composite(parts):
    """parts: [(trimesh in mm, mass in g)] -> (mass kg, com m, inertia about com kg·m²)."""
    M = sum(m for _, m in parts) / 1000.0
    com = sum(p.center_mass * (m / 1000.0) for p, m in parts) / M / 1000.0
    I = np.zeros((3, 3))
    for p, m in parts:
        mk = m / 1000.0
        mp = p.copy()
        mp.apply_scale(0.001)
        if not mp.is_volume:
            mp = mp.convex_hull
        mp.density = mk / mp.volume
        Ic = mp.moment_inertia            # about its own COM
        r = mp.center_mass - com
        I += Ic + mk * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return M, com, I


# ------------------------------------------------------------------ bodies
def body_parts(d, ms):
    """Returns {body: [(mesh_mm, mass_g, visual_rgba_or_None)]} in each body's
    *reference* placement (right-front leg for leg bodies), body frame."""
    P, B = ms["printed"], ms["bought"]
    F = frames(d)
    sv = B["servo_sts3215"]
    base = []
    hip = mesh("hip_block")
    cap = mesh("servo_cap")
    placements = [np.eye(4), translate_m(-2 * d["bx"]), RZ180 @ translate_m(-2 * d["bx"]), RZ180]  # FR, RR, FL, RL
    for T in placements:
        base.append((transform(hip, T), P["hip_block"] + B["screws_per_leg"] / 2, (0.30, 0.55, 0.96, 1)))
        base.append((transform(servo_box(d, F["pitch_FR"]), T), sv, (0.12, 0.12, 0.12, 1)))
        base.append((transform(transform(cap, F["pitch_FR"]), T), P["servo_cap"], (0.6, 0.72, 0.94, 1)))
    base += [
        (mesh("tray"), P["tray"] + B["screws_body"], (0.91, 0.91, 0.91, 1)),
        (mesh("deck"), P["deck"], (0.82, 0.82, 0.82, 1)),
        (servo_box(d, F["pan"]), sv, (0.12, 0.12, 0.12, 1)),
        (transform(cap, F["pan"]), P["servo_cap"], (0.6, 0.72, 0.94, 1)),
        (box([75, 56, 40], [-35, 0, d["tray_z0"] + 3 + 20]), B["battery_3s2p_18650"], (0.2, 0.25, 0.6, 1)),
        (box([50, 40, 12], [-35, 0, d["tray_z0"] + 3 + 46]), B["bus_adapter_buck_wiring"], (0.1, 0.5, 0.2, 1)),
        (box([85, 56, 18], [-69.5, 0, d["tray_z1"] + d["deck_t"] + 5 + 9]), B["pi5_with_cooler"], (0.1, 0.55, 0.25, 1)),
    ]
    upper = [(mesh("upper_link"), P["upper_link"] + B["screws_per_leg"] / 2, (0.96, 0.65, 0.14, 1)),
             (servo_box(d, F["knee_FR"]), sv, (0.12, 0.12, 0.12, 1)),
             (transform(cap, F["knee_FR"]), P["servo_cap"], (0.97, 0.78, 0.48, 1))]
    lower = [(mesh("lower_link"), P["lower_link"], (0.96, 0.65, 0.14, 1)),
             (mesh("foot"), P["foot"], (0.15, 0.15, 0.15, 1))]
    pan = [(mesh("pan_bracket"), P["pan_bracket"], (0.30, 0.55, 0.96, 1)),
           (mesh("sleeve_cap"), P["sleeve_cap"], (0.6, 0.72, 0.94, 1)),
           (servo_box(d, F["tilt"]), sv, (0.12, 0.12, 0.12, 1))]
    head = [(mesh("head"), P["head"], (0.30, 0.55, 0.96, 1)),
            (mesh("tilt_hub"), P["tilt_hub"], (0.8, 0.85, 0.9, 1)),
            (box([18, 48, 36], [d["pan_x"] + 40, 1.5, d["tilt_z"] + 6]), B["head_electronics"], None)]
    return {"base": base, "upper": upper, "lower": lower, "pan": pan, "head": head}


def translate_m(dx):
    T = np.eye(4); T[0, 3] = dx; return T


# ------------------------------------------------------------------- MJCF
def build(cfg, out_dir: Path | None = None, overrides: dict | None = None) -> tuple[str, dict]:
    """Returns (mjcf_xml, info). Writes per-body visual STLs into out_dir/meshes."""
    d, ms = load_dims(), load_masses()
    sp = dict(ms["servo"]); sp.update((overrides or {}).get("servo", {}))
    out_dir = Path(out_dir or HERE / "build")
    (out_dir / "meshes").mkdir(parents=True, exist_ok=True)
    parts = body_parts(d, ms)

    anchors = {  # joint anchor of each body in the BODY frame (mm), reference placement
        "upper": np.array([d["bx"], -d["foot_y"], 0.0]),
        "lower": np.array([d["bx"], -d["foot_y"], -d["L2"]]),
        "pan": np.array([d["pan_x"], 0.0, d["pan_rim_z"]]),
        "head": np.array([d["pan_x"], 0.0, d["tilt_z"]]),
        "base": np.zeros(3),
    }
    inert, vis_xml = {}, {}
    for name, plist in parts.items():
        M, com, I = composite([(m, g) for m, g, _ in plist])
        com_local = com - anchors[name] / 1000.0
        inert[name] = (M, com_local, I)
        geoms = []
        for i, (m, g, rgba) in enumerate(plist):
            if rgba is None:
                continue
            fn = f"{name}_{i}.stl"
            translate(m, -anchors[name]).export(out_dir / "meshes" / fn)
            geoms.append((fn, rgba))
        vis_xml[name] = geoms

    def inertial(name):
        M, c, I = inert[name]
        return (f'<inertial pos="{c[0]:.5f} {c[1]:.5f} {c[2]:.5f}" mass="{M:.4f}" '
                f'fullinertia="{I[0,0]:.3e} {I[1,1]:.3e} {I[2,2]:.3e} {I[0,1]:.3e} {I[0,2]:.3e} {I[1,2]:.3e}"/>')

    assets, used = [], set()

    def visuals(name):
        out = []
        for fn, rgba in vis_xml[name]:
            if fn not in used:
                assets.append(f'<mesh name="{fn[:-4]}" file="meshes/{fn}" scale="0.001 0.001 0.001"/>')
                used.add(fn)
            out.append(f'<geom class="visual" mesh="{fn[:-4]}" rgba="{" ".join(map(str, rgba))}"/>')
        return "\n".join(out)

    j = cfg["servos"]["joints"]
    rng = lambda n: f'{math.radians(j[n]["min"]):.4f} {math.radians(j[n]["max"]):.4f}'
    L2, L3, fr = d["L2"] / 1000, d["L3"] / 1000, d["foot_d"] / 2000
    legs_xml = []
    for leg in LEGS:
        hx = d["bx"] * (1 if leg[0] == "F" else -1) / 1000
        right = leg[1] == "R"
        hy = -d["foot_y"] / 1000 if right else d["foot_y"] / 1000
        euler = "0 0 0" if right else "0 0 3.14159265"
        axis = "0 1 0" if right else "0 -1 0"     # body +Y in the (possibly rotated) local frame
        legs_xml.append(f"""
      <body name="{leg}_upper" pos="{hx} {hy} 0" euler="{euler}">
        <joint name="{leg}.pitch" axis="{axis}" range="{rng(leg + '.pitch')}"/>
        {inertial('upper')}
        {visuals('upper')}
        <geom name="{leg}_thigh" type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.012"/>
        <geom name="{leg}_kneecup" type="box" pos="0 0.024 -0.0875" size="0.016 0.019 0.031"/>
        <body name="{leg}_lower" pos="0 0 {-L2}">
          <joint name="{leg}.knee" axis="{axis}" range="{rng(leg + '.knee')}"/>
          {inertial('lower')}
          {visuals('lower')}
          <geom name="{leg}_shin" type="capsule" fromto="0 0 0 0 0 {-L3 + 0.02}" size="0.007"/>
          <geom name="{leg}_foot" type="sphere" pos="0 0 {-L3}" size="{fr}" friction="0.9 0.02 0.001"/>
          <site name="{leg}_foot" pos="0 0 {-L3}"/>
        </body>
      </body>""")

    head_dz = (d["tilt_z"] - d["pan_rim_z"]) / 1000
    xml = f"""<mujoco model="pip_v2">
  <compiler angle="radian"/>
  <option timestep="0.001" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/><headlight ambient="0.35 0.35 0.35" diffuse="0.2 0.2 0.2"/></visual>
  <default>
    <joint damping="0.05" armature="{sp['armature_kg_m2']}" frictionloss="{sp['frictionloss_nm']}" limited="true"/>
    <geom contype="2" conaffinity="1" friction="0.8 0.02 0.001" rgba="0.5 0.5 0.5 0"/>
    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="1" density="0"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.88 0.9" rgb2="0.78 0.8 0.83" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="40 40" reflectance="0"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.95 0.96 0.98" rgb2="0.8 0.84 0.9" width="256" height="256"/>
    __ASSETS__
  </asset>
  <worldbody>
    <light directional="true" pos="0 0 3" dir="-0.3 0.4 -1" diffuse="0.75 0.75 0.75" specular="0.1 0.1 0.1" castshadow="true"/>
    <light directional="true" pos="0 0 3" dir="0.5 -0.3 -1" diffuse="0.3 0.3 0.3" specular="0 0 0" castshadow="false"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="grid" contype="1" conaffinity="1" rgba="1 1 1 1"/>
    <body name="base" pos="0 0 0.3">
      <freejoint name="root"/>
      {inertial('base')}
      {visuals('base')}
      <geom name="body_box" type="box" pos="0 0 {(d['tray_z0'] + d['tray_z1']) / 2000}" size="{d['tray_hx']/1000} 0.045 {(d['tray_z1'] - d['tray_z0']) / 2000}"/>
      <site name="imu" pos="0 0 0"/>
      {''.join(legs_xml)}
      <body name="pan" pos="{d['pan_x']/1000} 0 {d['pan_rim_z']/1000}">
        <joint name="head.pan" axis="0 0 1" range="{rng('head.pan')}"/>
        {inertial('pan')}
        {visuals('pan')}
        <body name="head" pos="0 0 {head_dz}">
          <joint name="head.tilt" axis="0 1 0" range="{rng('head.tilt')}"/>
          {inertial('head')}
          {visuals('head')}
          <geom name="head_box" type="box" pos="0.018 0.0015 0.0065" size="0.032 0.0277 0.0225"/>
          <camera name="eye" pos="0.052 0.0015 0.008" xyaxes="0 -1 0 0 0 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    {''.join(f'<motor name="{n}" joint="{n}" ctrllimited="false"/>' for n in j)}
  </actuator>
</mujoco>"""
    xml = xml.replace("__ASSETS__", "\n    ".join(assets))
    total = sum(v[0] for k, v in inert.items() if k in ("base", "pan", "head")) + 4 * (inert["upper"][0] + inert["lower"][0])
    info = {"total_mass_kg": total, "masses": {k: v[0] for k, v in inert.items()}, "servo": sp, "dims": d,
            "battery_capacity_wh": ms.get("battery_capacity_wh", 66)}
    (out_dir / "pip.xml").write_text(xml)
    return xml, info
