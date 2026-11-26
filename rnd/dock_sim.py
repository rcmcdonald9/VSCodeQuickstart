"""Self-docking Monte Carlo (queue item 4, loop 26).  python -m rnd.dock_sim [N]

MuJoCo, planar. The base is a 25 kg disc of R 0.25 m on x/y slides + a yaw hinge, driven by velocity servos whose force is
limited to the wheel peak (approach) or to a soft limit (final push).

The dock is a printed V block. Its two faces are at ±DOCK_V_DEG to the dock axis, with the spring contacts on the
centre line, and it is screwed to the floor. The robot's contact pads sit on the skirt at azimuth PAD_AZ (between two
wheels, in view of the head camera).

Each run draws:
- a start pose ±0.3 m / ±20° around a point 1.0 m out
- a dock-pose estimate error: σ 10 mm lateral, σ 1.5° (lidar profile + AprilTag; ESTIMATE)
- wheel slip: speed scale U(0.9, 1.1) and a lateral drift velocity σ 10 mm/s

Approach:
1. Go to a pre-dock point 0.35 m out along the *estimated* axis, with the pads facing the dock.
2. Drive in at 0.10 m/s (60 N push limit) with a yaw hold. A weak lateral hold applies until first contact; after that the
   lateral axis is current-limited to 5 N, so the V faces centre the disc.
3. Stop when both faces push back (> 15 N each), or after 3 s in contact.

Pass: the pad centre lies within ±CAPTURE of the dock contacts laterally, yaw error ≤ 5°, and the disc is pressed into
the V with its centre within 8 mm of the dock axis.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
R = 0.25
DOCK_V_DEG = 60.0          # V faces at ±60° to the dock axis (loop 26: ±40° with 0.2 m faces never touched the R 0.25 disc on its faces)
CAPTURE = 0.025            # pads 60 mm wide vs 8 mm spring leaves → ±25 mm lateral (loop 26: 40 → 60 mm, yaw error dominates)
PAD_AZ = math.radians(45)  # contact pads on the skirt between the wheels at 0° and 90°


def xml():
    # dock at the origin, axis +X (the robot comes from +X, moving toward −X); V faces open toward +X
    a = math.radians(DOCK_V_DEG)
    L = 0.25                   # contact at R / tan(a) = 144 mm from the apex
    walls = []
    for s in (1, -1):
        # inner face from the apex (0, 0) along (cos a, s sin a); the 20 mm box sits behind it (away from the V interior)
        cx = L / 2 * math.cos(a) + 0.01 * math.sin(a)
        cy = s * (L / 2 * math.sin(a) - 0.01 * math.cos(a)) * -1 * -1
        cy = s * L / 2 * math.sin(a) + s * 0.01 * math.cos(a)
        cx = L / 2 * math.cos(a) - 0.01 * math.sin(a)
        walls.append(f'<geom type="box" size="{L/2} 0.01 0.06" pos="{cx} {cy} 0.08" '
                     f'euler="0 0 {math.degrees(s * a)}" rgba=".9 .5 .2 1" friction="0.35 0.01 0.001"/>')
    return f"""<mujoco><option timestep="0.002"/>
<worldbody><geom type="plane" size="5 5 .1" rgba=".8 .8 .8 1"/>{''.join(walls)}
 <body name="base" pos="1 0 0.09"><joint name="x" type="slide" axis="1 0 0" damping="5"/><joint name="y" type="slide" axis="0 1 0" damping="5"/>
  <joint name="yaw" type="hinge" axis="0 0 1" damping="2"/>
  <geom type="cylinder" size="{R} 0.065" mass="25" friction="0.35 0.01 0.001" contype="1" conaffinity="1"/></body></worldbody>
<actuator><velocity name="vx" joint="x" kv="400" forcerange="-289 289"/><velocity name="vy" joint="y" kv="400" forcerange="-289 289"/>
 <velocity name="vw" joint="yaw" kv="60" forcerange="-60 60"/></actuator></mujoco>"""


def run_one(rng, video=False):
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml())
    d = mujoco.MjData(m)
    seat_x = R / math.sin(math.radians(DOCK_V_DEG))                 # disc centre when seated in the V
    # start pose around (1.0, 0) with the pads (azimuth PAD_AZ in the body frame) roughly facing the dock (−X)
    yaw_ok = math.pi - PAD_AZ
    d.qpos[:] = [rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), yaw_ok + math.radians(rng.uniform(-20, 20))]
    est_off, est_yaw = rng.normal(0, 0.010), math.radians(rng.normal(0, 1.5))     # dock-pose estimate error
    scale, drift = rng.uniform(0.9, 1.1), rng.normal(0, 0.010)
    ax = np.array([math.cos(est_yaw), math.sin(est_yaw)])                          # estimated dock axis
    nrm = np.array([-ax[1], ax[0]])
    origin = np.array([0.0, est_off]) - ax * 0.0
    pre = origin + ax * (seat_x - 1.0 + 0.35 + 1.0)
    t, phase, push, t_contact = 0.0, "pre", 0.0, 0.0
    walls = {i for i in range(m.ngeom) if m.geom_type[i] == 6 and m.geom_bodyid[i] == 0}
    yaw_goal = math.atan2(-ax[1], -ax[0]) - PAD_AZ
    for _ in range(int(25 / m.opt.timestep)):
        p = np.array([d.qpos[0] + 1.0, d.qpos[1]])                                 # world position (body starts at x = 1)
        yaw = d.qpos[2]
        ey = (yaw_goal - yaw + math.pi) % (2 * math.pi) - math.pi
        if phase == "pre":
            e = pre - p
            v = np.clip(e * 1.5, -0.3, 0.3)
            m.actuator_forcerange[0] = m.actuator_forcerange[1] = (-289, 289)
            if np.linalg.norm(e) < 0.02 and abs(ey) < math.radians(2):
                phase = "in"
        else:
            lat = np.dot(p - origin, nrm)
            v = -ax * 0.10 - (nrm * lat * 0.5 if t_contact == 0 else 0)            # weak lateral hold until first contact, then the V centres the disc
            m.actuator_forcerange[0] = (-60, 60)                                   # push along the dock axis (≈ world X)
            m.actuator_forcerange[1] = (-5, 5) if t_contact > 0 else (-60, 60)     # lateral compliance in contact (current-limited)
            fw, hit = 0.0, set()                                                  # dock-wall contacts only (the floor always touches)
            f6 = np.zeros(6)
            for ci in range(d.ncon):
                c = d.contact[ci]
                if c.geom1 in walls or c.geom2 in walls:
                    mujoco.mj_contactForce(m, d, ci, f6)
                    fw += abs(f6[0]); hit.add(c.geom1 if c.geom1 in walls else c.geom2)
            if hit:
                t_contact += m.opt.timestep
            if (len(hit) == 2 and fw > 2 * 15) or t_contact > 3.0:          # both faces (each wall gives 2 contact points)
                break
        w = 3.0 * ey                                                              # firm yaw hold throughout (wall friction ≈ 4 N·m ≪ 60)
        d.ctrl[:] = [v[0] * scale, v[1] * scale + drift, w]
        mujoco.mj_step(m, d)
        t += m.opt.timestep
    p = np.array([d.qpos[0] + 1.0, d.qpos[1]])
    pad = p + R * np.array([math.cos(d.qpos[2] + PAD_AZ), math.sin(d.qpos[2] + PAD_AZ)])
    true_yaw_goal = math.pi - PAD_AZ
    yaw_err = abs((true_yaw_goal - d.qpos[2] + math.pi) % (2 * math.pi) - math.pi)
    touching = {c.geom1 if c.geom1 in walls else c.geom2 for c in d.contact[:d.ncon] if (c.geom1 in walls) != (c.geom2 in walls)}
    gap = 0.0 if touching and abs(p[1]) <= 0.008 else 99.0                       # seated = pressed into the V, centre within 8 mm of the axis
    ok = abs(pad[1]) <= CAPTURE and yaw_err <= math.radians(5) and gap == 0.0
    return dict(ok=bool(ok), lat_mm=float(pad[1] * 1000), yaw_deg=float(math.degrees(yaw_err)), gap_mm=float(gap * 1000), t=t, phase=phase, ntouch=len(touching), pos=(round(float(p[0]), 3), round(float(p[1]), 3)))


def main(n=100, seed=3):
    rng = np.random.default_rng(seed)
    rows = [run_one(rng) for _ in range(n)]
    ns = sum(r["ok"] for r in rows)
    z, ph = 1.96, ns / n
    lb = (ph + z * z / (2 * n) - z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n))) / (1 + z * z / n)
    lat = np.array([r["lat_mm"] for r in rows]); yw = np.array([r["yaw_deg"] for r in rows]); gp = np.array([r["gap_mm"] for r in rows])
    L = ["# Self-docking Monte Carlo (generated by `python -m rnd.dock_sim`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**{ns}/{n} dock successfully ({ph:.0%}; 95 % lower bound {lb:.0%}).**",
         f"Pad lateral error: mean {np.mean(np.abs(lat)):.1f} mm, worst {np.max(np.abs(lat)):.1f} mm (capture ±{CAPTURE * 1000:.0f}). "
         f"Yaw error: worst {yw.max():.1f}°. Seated: {sum(r["gap_mm"] == 0 for r in rows)}/{n}.", "",
         "| Run | Result | Pad lateral (mm) | Yaw err (°) | Gap (mm) | Time (s) |", "|---|---|---|---|---|---|"]
    L += [f"| {i} | {'✅' if r['ok'] else '❌'} | {r['lat_mm']:+.1f} | {r['yaw_deg']:.1f} | {r['gap_mm']:.1f} | {r['t']:.1f} |" for i, r in enumerate(rows[:20])]
    (DOCS / "dock_sim.md").write_text("\n".join(L))
    (DOCS / "dock_sim.json").write_text(json.dumps({"n": n, "ok": ns, "lb95": lb, "lat_max_mm": float(np.max(np.abs(lat)))}))
    print("\n".join(L[4:7]))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)
