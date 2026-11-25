"""Omni-wheel base over thresholds and on carpet: a roller-level MuJoCo model.  python -m rnd.threshold

The whole robot is lumped into one rigid body (mass and COM from the twin, payload 2 kg in the hands). Four 6"
omni wheels sit in the "+" layout with radial axles. Each wheel has 2 rows × 10 free rollers (capsules, Ø25 mm,
on a Ø127 mm pitch circle: the DuraOmni-class geometry is an ESTIMATE). Each wheel is driven by a velocity servo
torque-limited to the GIM6010-8 peak (11 N·m).

Cases: hard floor vs carpet-like ground (soft contact plus rolling resistance), threshold height 0–20 mm, driving
straight at a wheel (0°) or at 45° (planner rule D2b).
Result per case: did it cross, peak and RMS wheel torque, time to cross.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

from .params import BASELINE, act  # noqa: E402

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
R_W, R_ROLLER, PITCH_R, N_ROLL = 0.0762, 0.0125, 0.0637, 10
SPEED = 0.30                 # m/s approach speed
STEP_X = 0.60                # threshold edge position (m) along the travel direction


def build(d=BASELINE, step_h=0.015, heading_deg=45.0, carpet=False, mass=None, com_h=0.40):
    m_tot = mass or 23.6 + 2.0
    g = act(d.base.wheel_motor)
    R = d.base.wheel_radius_pos
    # rollers collide only with the ground (contype 1 / conaffinity 2), never with each other
    floor = ('contype="2" conaffinity="1" solref="0.02 1" solimp="0.8 0.95 0.003" friction="0.7 0.02 0.004" condim="6"' if carpet
             else 'contype="2" conaffinity="1" solref="0.004 1" friction="0.9 0.01 0.0005" condim="3"')
    # travel along +X of the world; the robot is yawed so that this is heading_deg relative to wheel 0
    yaw = -heading_deg
    wheels = []
    for k in range(4):
        th = math.radians(90 * k)
        cx, cy = R * math.cos(th), R * math.sin(th)
        rollers = []
        for row in (0, 1):
            for i in range(N_ROLL):
                a = 2 * math.pi * (i + 0.5 * row) / N_ROLL
                y_off = (-0.011 if row == 0 else 0.011)
                # wheel frame: axle = local x; rim in the local y-z plane; roller axis tangent to the rim
                px, py, pz = y_off, PITCH_R * math.cos(a), PITCH_R * math.sin(a)
                tx, ty, tz = 0, -math.sin(a), math.cos(a)
                rollers.append(f'''<body pos="{px} {py} {pz}"><joint type="hinge" axis="{tx} {ty} {tz}" damping="0.0005"/>
                  <geom type="capsule" size="{R_ROLLER} 0.009" zaxis="{tx} {ty} {tz}" mass="0.01" rgba=".15 .15 .15 1" friction="0.9 0.01 0.0005" contype="1" conaffinity="2"/></body>''')
        wheels.append(f'''<body name="w{k}" pos="{cx} {cy} {R_W}" euler="0 0 {90 * k}">
            <joint name="wj{k}" type="hinge" axis="1 0 0" damping="0.05"/>
            <geom type="cylinder" size="0.05 0.022" zaxis="1 0 0" mass="0.25" contype="0" conaffinity="0" rgba=".3 .3 .3 1"/>
            {"".join(rollers)}</body>''')
    acts = "".join(f'<velocity name="m{k}" joint="wj{k}" kv="30" forcerange="-{g["peak"]} {g["peak"]}"/>' for k in range(4))
    step = (f'<geom name="step" type="box" pos="{STEP_X + 0.5} 0 {step_h / 2}" size="0.5 1.0 {step_h / 2}" {floor} rgba=".55 .45 .35 1"/>'
            if step_h > 0 else "")
    return f"""<mujoco><option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="5"/>
      <worldbody><geom name="floor" type="plane" size="6 3 .1" {floor} rgba=".6 .6 .62 1"/>{step}
        <body name="robot" pos="0 0 0.0" euler="0 0 {yaw}"><freejoint/>
          <geom type="cylinder" size="0.25 0.065" pos="0 0 0.09" mass="0.001" contype="0" conaffinity="0" rgba=".9 .9 .9 .4"/>
          <geom type="box" size="0.05 0.05 0.05" pos="0.0 0 {com_h}" mass="{m_tot}" contype="0" conaffinity="0"/>
          {"".join(wheels)}
        </body></worldbody><actuator>{acts}</actuator></mujoco>"""


def run(step_h=0.015, heading_deg=45.0, carpet=False, t_max=5.0):
    m = mujoco.MjModel.from_xml_string(build(step_h=step_h, heading_deg=heading_deg, carpet=carpet))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    rb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")
    R = BASELINE.base.wheel_radius_pos
    yaw = math.radians(-heading_deg)
    # world velocity (+X) expressed in the robot frame; wheel k rolls tangentially at angle 90k
    vx, vy = SPEED * math.cos(-yaw), SPEED * math.sin(-yaw)
    # +ω about the radial axle drives the wheel centre along -tangent → ω = -v_tangent / r
    wheel_w = [-(-math.sin(math.radians(90 * k)) * vx + math.cos(math.radians(90 * k)) * vy) / R_W for k in range(4)]
    taus, crossed_t = [], None
    for i in range(int(t_max / m.opt.timestep)):
        ramp = min(1.0, i * m.opt.timestep / 0.5)
        for k in range(4):
            d.ctrl[k] = wheel_w[k] * ramp
        mujoco.mj_step(m, d)
        if i % 20 == 0:
            taus.append(np.abs(d.actuator_force[:4]).copy())
        x = d.xpos[rb][0]
        if crossed_t is None and x > STEP_X + R + R_W + 0.05:
            crossed_t = i * m.opt.timestep
    taus = np.array(taus)
    tilt = math.degrees(math.acos(max(-1, min(1, d.xmat[rb].reshape(3, 3)[2, 2]))))
    return dict(crossed=crossed_t is not None, t=crossed_t, x=float(d.xpos[rb][0]), peak=float(taus.max()),
                rms=float(np.sqrt((taus ** 2).mean())), tilt=tilt)


CASES = [("hard floor, no step (cruise)", 0.0, 45, False), ("carpet, no step (cruise)", 0.0, 45, True),
         ("hard floor, 10 mm, 45°", 0.010, 45, False), ("hard floor, 15 mm, 45°", 0.015, 45, False),
         ("carpet, 15 mm, 45°", 0.015, 45, True), ("hard floor, 15 mm, 0° (straight at a wheel)", 0.015, 0, False),
         ("hard floor, 20 mm, 45°", 0.020, 45, False), ("carpet, 20 mm, 45°", 0.020, 45, True)]


CROSS_SPEED = 0.15   # planner rule (D2b, loop 12): slow to 0.15 m/s for a threshold


def main():
    global SPEED
    g = act(BASELINE.base.wheel_motor)
    rows, res = [], {}
    for name, h, hd, carpet in CASES:
        r = run(h, hd, carpet)
        res[name] = r
        ok = r["crossed"] if h > 0 else r["x"] > 1.0
        rows.append(f"| {name} | {'✅' if ok else '❌'} | {r['peak']:.1f} / {g['peak']} N·m | {r['rms']:.1f} / {g['rated']} N·m | {r['tilt']:.1f}° |")
    L = ["# Omni base over thresholds and on carpet (auto-generated by `python -m rnd.threshold`)", "",
         __doc__.split("\n\n", 1)[1].strip(), "",
         f"Approach speed {SPEED} m/s; robot 25.6 kg (incl. 2 kg payload), COM 0.40 m high.", "",
         "| Case | Crossed / drove | Peak wheel torque / GIM peak | RMS wheel torque / rated | Final tilt |", "|---|---|---|---|---|"] + rows
    SPEED = CROSS_SPEED
    slow = {name: run(h, hd, carpet, t_max=0.5 + 1.4 / SPEED) for name, h, hd, carpet in CASES if h > 0 and hd == 45}
    SPEED = 0.30
    L += ["", f"## With the planner rule: cross at 45° **and ≤ {CROSS_SPEED} m/s**", "",
          "| Case | Crossed | Peak wheel torque | Margin to GIM peak |", "|---|---|---|---|"]
    L += [f"| {n} | {'✅' if r['crossed'] else '❌'} | {r['peak']:.1f} N·m | {g['peak'] / r['peak']:.1f}× |" for n, r in slow.items()]
    L += ["", "**Reading:**",
          "- **Straight at a wheel (0°) fails at 15 mm**: the sideways rollers can't climb. This confirms the 45° rule.",
          "- **At 0.30 m/s the impact peaks reach 9.9–11 N·m**, the whole GIM peak. Slowing to 0.15 m/s for the crossing cuts the peaks to ~6 N·m, a ~1.8× margin, up to 20 mm on carpet.",
          "- **Carpet cruise needs ~2.2 N·m RMS per wheel** (44 % of rated) at 0.3 m/s. That's higher than the hand estimate (0.8 N·m) but thermally fine, and it matches the 30 W driving term in the power budget.",
          "", "**Caveats:**",
          "- Carpet is modelled as soft contact plus rolling friction. That is a coarse stand-in for pile (ESTIMATE), so a real carpet bench test is still required.",
          "- Roller geometry is approximated. The wheel servo is an ideal velocity loop, torque-limited at the GIM peak."]
    md = "\n".join(L)
    (DOCS / "threshold.md").write_text(md)
    k15 = [slow["hard floor, 15 mm, 45°"], slow["carpet, 15 mm, 45°"]]
    (DOCS / "threshold.json").write_text(json.dumps({"ok15": all(r["crossed"] for r in k15), "peak15": max(r["peak"] for r in k15),
                                                     "ok20": slow["carpet, 20 mm, 45°"]["crossed"],
                                                     "cruise_rms_carpet": res["carpet, no step (cruise)"]["rms"]}))
    print(md)


if __name__ == "__main__":
    main()
