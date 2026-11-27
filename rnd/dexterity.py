"""Dexterity map (design target R16, loop 40; owner question "is the arm useful as is?").  python -m rnd.dexterity

The reach metric (R1) asks whether the hand reaches a set of targets with any orientation the planner likes. This asks
the harder question a manipulation policy needs: over a table grid in front of the bench arm, what fraction of points
can the hand reach **with a given grasp orientation**, from any approach heading about vertical?

- grid: x 0.25…0.70 m, y −0.40…+0.20 m (5 cm), two heights (5 and 25 cm above the table): 240 points
- grasps: top-down, 45° tilted, side (horizontal); headings 0/90/180/270° about vertical. The parallel gripper closes
  on the object the same way with either jaw on either side, so each heading counts at wrist roll θ or θ + 180°
  (with the wrist roll at ±120°, ±90° covers every jaw orientation). The jaws are not symmetric about the roll axis
  (rnd/parts/hand.py: fixed jaw face at x = +21 mm, the moving jaw swings onto it), so the grasp centre is the object
  centre, fixed-jaw face − w/2 along the jaw axis, for the bench cube (w 44 mm, sw/bench_env.py). Flipping the jaws
  moves that centre to the other side of the roll axis, and the IK puts it, not the roll axis, on the target.
- IK: the bench env's damped least squares, 8 seeds; success = 5 mm and ≈ 6° orientation error, inside the joint limits
  (sw/bench_env.py builds the arm from rnd/params.py and the twin's joint ranges, so a range change shows up here)
- "across the body" = the points with y > 0 (the other arm's side of the bench)

Targets (set from the loop-32 arm with yaw ±150°: 36 / 69 / 68 / 60 %): no grasp class may drop more than 6 points.
Before 2026-11-27 each heading was tried with one jaw orientation, and the wrist was ±180°. Cutting the wrist to ±120°
made that version lose every heading that needed more than 120° of wrist roll: 69 → 50 % at 45°. It also under-counted
at ±180°. Its orientation error (½ Σ axis × target axis) goes to zero at a 180° roll error, so a seed whose wrist was
half a turn from the target stalled. The jaw-flip version scores the same at ±120° and ±180° (45° / side / across
the body: 78 / 78 / 71 against 78 / 77 / 68 %). The history columns are from the old version.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
TARGETS = {"top-down": 0.30, "45° angled": 0.63, "side (horizontal)": 0.62, "side, across the body": 0.54}


def _R(axis, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) if axis == "y" else np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fixed_jaw_face_x() -> float:
    """Contact face of the fixed jaw's fin-ray finger, x in the hand frame (m), from the hand CAD."""
    from .parts import hand
    return next(p for n, p, _ in hand.gripper_parts() if n == "finray_fixed").bounding_box().min.X / 1000


def run():
    import mujoco
    from sw.bench_env import BenchEnv, TABLE_Z
    env = BenchEnv(render=False)
    env.reset(0)
    w = 2 * env.m.geom_size[env.m.body_geomadr[env.cube]][0]           # bench cube width
    c = np.array([fixed_jaw_face_x() - w / 2, 0, 0])                    # grasp centre in the hand frame (jaw axis = hand x)
    oris = {"top-down": np.eye(3), "45° angled": _R("y", math.radians(45)), "side (horizontal)": _R("y", math.radians(90))}
    xs, ys, zs = np.arange(0.25, 0.71, 0.05), np.arange(-0.40, 0.21, 0.05), (TABLE_Z + 0.05, TABLE_Z + 0.25)
    seeds = tuple(env.SEEDS) + (np.array([-1.3, 0.3, 0.3, -0.6, 0.6, 0]), np.array([-0.6, 0.8, -0.5, -1.8, 1.2, 0.5]))
    res = {}
    for name, R0 in oris.items():
        ok = tot = okx = totx = 0
        for x in xs:
            for y in ys:
                for z in zs:
                    found = False
                    for hd in (0, math.pi / 2, -math.pi / 2, math.pi):
                        for flip in (0, math.pi):                            # either jaw on either side
                            Rt = _R("z", hd) @ R0 @ _R("z", flip)
                            pt = np.array([x, y, z]) - Rt @ c                    # roll-axis site that puts the grasp centre on (x, y, z)
                            for sd in seeds:
                                env._ik_from(sd, pt, Rt, 80)
                                d = env._d2
                                mujoco.mj_kinematics(env.m, d)
                                R = d.site_xmat[env.site].reshape(3, 3)
                                if np.linalg.norm(d.site_xpos[env.site] - pt) < 0.005 and np.linalg.norm(R - Rt) < 0.1:
                                    found = True
                                    break
                            if found:
                                break
                        if found:
                            break
                    tot += 1
                    ok += found
                    if y > 0.0:
                        totx += 1
                        okx += found
        res[name] = (ok / tot, okx / totx)
    out = {"top-down": res["top-down"][0], "45° angled": res["45° angled"][0], "side (horizontal)": res["side (horizontal)"][0],
           "side, across the body": res["side (horizontal)"][1]}
    return out


def score():
    """Cached result for the scorecard (run `python -m rnd.dexterity` after a kinematics change)."""
    f = DOCS / "dexterity.json"
    if not f.exists():
        return None
    return json.loads(f.read_text())


def main():
    from .twin import JR
    r = run()
    ok = all(r[k] >= TARGETS[k] for k in TARGETS)
    L = ["# Dexterity map, R16 (generated by `python -m rnd.dexterity`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Joint ranges:** yaw {JR['yaw'][0]}…{JR['yaw'][1]}°, roll {JR['roll'][0]}…{JR['roll'][1]}°, pitch {JR['pitch'][0]}…{JR['pitch'][1]}°, "
         f"elbow {JR['elbow'][0]}…{JR['elbow'][1]}°, wrist roll {JR['wroll'][0]:.0f}…{JR['wroll'][1]:.0f}°. **{'All grasp classes on target ✅' if ok else 'Below target ❌'}**", "",
         "| Grasp | Reachable (now) | Target | Loop 32 (yaw ±150°, old version) | Loop 33–39 (yaw ±70°, old version) | |", "|---|---|---|---|---|---|"]
    hist = {"top-down": (0.36, 0.34), "45° angled": (0.69, 0.57), "side (horizontal)": (0.68, 0.40), "side, across the body": (0.60, 0.02)}
    for k in TARGETS:
        L.append(f"| {k} | {r[k]:.0%} | ≥ {TARGETS[k]:.0%} | {hist[k][0]:.0%} | {hist[k][1]:.0%} | {'✅' if r[k] >= TARGETS[k] else '❌'} |")
    (DOCS / "dexterity.md").write_text("\n".join(L))
    (DOCS / "dexterity.json").write_text(json.dumps({**r, "ok": ok}))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
