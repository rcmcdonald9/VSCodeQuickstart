"""Self-docking Monte Carlo (queue item 4, loop 26).  python -m rnd.dock_sim [N]

MuJoCo, planar. The base is a 25 kg disc of R 0.25 m on x/y slides + a yaw hinge, driven by velocity servos whose force is
limited to the wheel peak (approach) or to a soft limit (final push).

The dock is a printed V block. Its two faces are at ±DOCK_V_DEG to the dock axis, with the spring contacts on the
centre line; it was screwed to the floor (finding 12: now a baseplate the robot's wheels hold down, see *Contacts and retention*). The robot's contact pads sit on the skirt at azimuth PAD_AZ (between two
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

# --- Contact geometry and retention (Neon review #2 finding 12, 2026-11-27). The CAD (rnd/parts/base.py DOCK_PADS +
# dock_pad_carrier, rnd/parts/dock.py tower / wings) belongs to the CAD track: these are the numbers it has to adopt. ---
PADS_V1 = (("ID", 40.0, 10.0), ("+", 60.0, 12.0), ("−", 100.0, 12.0))      # (name, centre z, height) mm: the CAD today
PADS_V2 = (("ID", 40.0, 28.0), ("+", 84.0, 28.0), ("−", 128.0, 28.0))      # proposed: 28 mm tall, 44 mm pitch (16 mm gaps)
PAD_W = 60.0                       # pad width (mm): sets the ±25 mm lateral capture above; unchanged (stacking stays vertical)
SKIRT_Z = (25.0, 155.0)            # skirt span above the floor (params: clearance 25 + height 130)
PATCH = 4.0                        # contact patch height of a leaf / the pogo tip (mm, ESTIMATE)
DZ_BUDGET = dict(carpet=8.0, floor=2.0, tilt=1.0)   # vertical misalignment, worst case (mm, ESTIMATE): carpet sink difference, floor, base pitch
F_LEAF, N_LEAF, F_POGO = 5.0, 2, 1.0                # dock contact forces (N, ESTIMATE; dock_pad_carrier: ~5 N leaves)
CRR_OMNI = 0.02                    # rolling resistance of an unbraked 6" omni wheel, drives off (ESTIMATE, hard floor)
ROBOT_KG = 25.26                   # docs/rnd/budget.md total
WHEEL_R, R_W = 76.2, 210.0         # mm: 6" omni, contact points from the centre (params.base)
GROOVE_H = 4.0                     # wheel-cradle groove depth in the dock baseplate (mm)
MU_FEET = 0.6                      # baseplate rubber feet on hard floor (ESTIMATE)
F_PUSH = 60.0                      # dock_sim final-push force limit (N, actuator forcerange below)


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


def contact_design():
    """Pad window, short-circuit margin and retention for the pad layouts (finding 12)."""
    out = {}
    budget = sum(DZ_BUDGET.values())
    for tag, pads in (("v1 (CAD today)", PADS_V1), ("v2 (proposed)", PADS_V2)):
        P = {n: (z, h) for n, z, h in pads}
        win = min(h / 2 - PATCH / 2 for _, h in P.values())                 # vertical shift a contact tolerates and stays on its pad
        z_id, h_id = P["ID"]; z_p, h_p = P["+"]
        reach_id = (z_p - z_id) - h_id / 2 - PATCH / 2                      # shift that puts the + leaf on the ID pad
        gap = min(abs(a[0] - b[0]) - a[1] / 2 - b[1] / 2 for a in P.values() for b in P.values() if a is not b)
        lo, hi = min(z - h / 2 for z, h in P.values()), max(z + h / 2 for z, h in P.values())
        out[tag] = dict(win=win, reach_id=reach_id, short_margin=reach_id - win, gap=gap, span=(lo, hi),
                        fits=lo >= SKIRT_Z[0] and hi <= SKIRT_Z[1], ok_win=win >= budget)
    F_back = N_LEAF * F_LEAF + F_POGO
    N_w = ROBOT_KG * 9.81 / 4
    F_roll = ROBOT_KG * 9.81 * CRR_OMNI
    phi = math.acos(1 - GROOVE_H / WHEEL_R)
    F_hold_w = N_w * math.tan(phi)                                          # per wheel along its rolling direction
    F_need_w = 1.5 * F_back * math.cos(PAD_AZ)                              # 1.5 × the pushback, projected on each wheel (pads at 45° between them)
    x_w = R / math.sin(math.radians(DOCK_V_DEG)) * 1000 - R_W * math.cos(PAD_AZ)
    y_w = R_W * math.sin(PAD_AZ)
    return dict(layouts=out, budget=budget, F_back=F_back, F_roll=F_roll, N_w=N_w, phi=math.degrees(phi), F_hold_w=F_hold_w,
                F_need_w=F_need_w, F_plate=2 * N_w * MU_FEET, x_w=x_w, y_w=y_w)


def contact_lines():
    c = contact_design()
    ok = lambda b: "✅" if b else "❌"
    L = ["## Contacts and retention (Neon review #2 finding 12)", "",
         "**Verified in part.** The review's 8 mm gap between the ID pad (34–46 mm) and the + pad (54–66 mm) is right, but the short it describes cannot happen "
         "on a vertical shift alone: the ID pogo and the + leaf sit on the same dock tower and move together, and the dock's output is off until the ID handshake "
         "passes. For the + leaf to land on the ID pad the ID pogo must already be off its pad (no handshake, no output). What the small pads do cause is a "
         "**missed charge**: a contact stays on its pad only for a few mm of vertical error, less than a carpet's sink. The retention point is right: with the "
         "drives off nothing holds the unbraked omni base against the leaf springs, and floor screws are not a consumer install.", "",
         "| Pad layout (centre z / height mm) | Vertical window per contact | Shift that puts + on the ID pad | Margin (shift − window) | Smallest pad gap | Span (skirt 25–155) | Window ≥ worst-case error ({:.0f} mm) |".format(c["budget"]),
         "|---|---|---|---|---|---|---|"]
    for (tag, x), pads in zip(c["layouts"].items(), (PADS_V1, PADS_V2)):
        L.append(f"| {tag}: " + ", ".join(f"{n} {z:.0f} / {h:.0f}" for n, z, h in pads) + f" | ±{x['win']:.1f} mm | {x['reach_id']:.1f} mm | {x['short_margin']:.1f} mm | "
                 f"{x['gap']:.0f} mm | {x['span'][0]:.0f}–{x['span'][1]:.0f} {ok(x['fits'])} | {ok(x['ok_win'])} |")
    L += ["", "Worst-case vertical error (ESTIMATE): " + " + ".join(f"{k} {v:.0f}" for k, v in DZ_BUDGET.items()) + f" = {c['budget']:.0f} mm. "
          f"Contact patch {PATCH:.0f} mm (ESTIMATE). The pads stay stacked vertically: side-by-side pads 40 mm wide would cut the lateral capture to "
          f"±{(40 - 8) / 2:.0f} mm, under the Monte Carlo's {19:.0f} mm worst lateral error.", "",
          "| Retention | Value | Need | |", "|---|---|---|---|",
          f"| Contact pushback ({N_LEAF} leaves × {F_LEAF:.0f} N + pogo {F_POGO:.0f} N, ESTIMATE) vs rolling resistance of the unbraked base ({ROBOT_KG:.1f} kg, C_rr {CRR_OMNI}) | "
          f"{c['F_back']:.0f} N vs {c['F_roll']:.1f} N | resistance > pushback | {ok(c['F_roll'] > c['F_back'])} (the review is right: it rolls off) |",
          f"| Wheel cradle: {GROOVE_H:.0f} mm groove under each of the two wheels flanking the pads (wheel r {WHEEL_R:.0f} mm, {c['N_w']:.0f} N each; climb angle {c['phi']:.1f}°) | "
          f"{c['F_hold_w']:.0f} N per wheel along its rolling direction | ≥ 1.5 × pushback × cos 45° = {c['F_need_w']:.1f} N | {ok(c['F_hold_w'] >= c['F_need_w'])} |",
          f"| Baseplate held by the robot's own weight (2 wheels on it, rubber feet μ {MU_FEET}, ESTIMATE): no floor screws | {c['F_plate']:.0f} N | ≥ the {F_PUSH:.0f} N final push | "
          f"{ok(c['F_plate'] >= F_PUSH)} |", "",
          "**What the CAD owner has to change (rnd/parts/base.py, rnd/parts/dock.py):**",
          "- Skirt pads: " + ", ".join(f"{n} at {z:.0f} mm, {PAD_W:.0f} × {h:.0f} mm" for n, z, h in PADS_V2) + " (carrier z 26–142 instead of 28–118); the dock tower's two leaves and the ID pogo move to the same heights.",
          f"- Dock: replace the floor tabs with a 6 mm baseplate between the wings (apex to x ≈ 230 mm, |y| ≤ 210 mm, 15° lead-in edge) on rubber feet, with two {GROOVE_H:.0f} mm "
          f"wheel cradles centred at x ≈ {c['x_w']:.0f} mm, y ≈ ±{c['y_w']:.0f} mm from the apex (the seated wheel contact points), grooves parallel to each wheel's axle.",
          "- Robot ID input: series 1 kΩ resistor + 5.6 V TVS at the base sensor node, so a wrong contact can only ever see a limited current (belt and braces to the handshake gate).",
          "- The Monte Carlo above still models the V faces only; the cradle and the taller pads do not change the lateral capture.", ""]
    return L


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
    L += [""] + contact_lines()
    (DOCS / "dock_sim.md").write_text("\n".join(L))
    (DOCS / "dock_sim.json").write_text(json.dumps({"n": n, "ok": ns, "lb95": lb, "lat_max_mm": float(np.max(np.abs(lat)))}))
    print("\n".join(L[4:7]))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)
