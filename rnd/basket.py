"""R2d: the 20 lb laundry basket (owner, 2026-11-27).  python -m rnd.basket

Basket: ≈ 60 × 45 × 28 cm, 9.07 kg, a hand-hold slot in each short end just under the rim. Both grippers hook into
the end slots with the hands pointing down (the load hangs from a rigid fingertip hook, D21), so each hand carries
4.54 kg directly under its wrist; the hands are ≈ 0.60 m apart (shoulders 0.40 m), so the arms splay outward.

Full 3D statics on the twin (rnd/twin.py: lift + 6-joint right arm, CAD masses; the left arm mirrors it): for each pose
the bounded IK finds the collision-free solutions with the hand pointing down, and the planner keeps the one with the
lowest worst joint load. Joint loads are gravity torques with 4.54 kg in the hand (the shoulder-pitch balancer
included), against the D20 actuators (GIM8108-8 at shoulder pitch and elbow, 3:1). Rules: a basket carry is a duty
of ≤ 2 minutes, not a continuous hold, so each joint needs peak ≥ 2× and a winding rise over 120 s ≤ 15 K (thermal.md
model: ΔT_ss = 60 K·(τ/τ_rated)², τ_th 600 s); the floor pick (a few seconds) needs peak ≥ 1.5×; tip margin ≥ 30 mm
with the load pointed at a wheel.

Poses (handle height = support height + 0.25 m):
- floor pick: basket on the floor, handles at z 0.25, basket centre 0.40–0.50 m in front of the shoulders
- carry: handles at z 0.70–0.85 (basket bottom at hip height), 0.35–0.45 m out
- set down on a bed (0.55 m) or a washer/counter top (0.90 m)
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
PER_HAND = 20 * 0.45359 / 2
HALF_SPAN = 0.30                     # handle slots ±0.30 m from the centre line
POSES = [("floor pick", 0.25, (0.40, 0.45, 0.50), "peak"), ("carry", 0.70, (0.35, 0.40, 0.45), "carry"),
         ("carry, handles at 0.80", 0.80, (0.35, 0.40, 0.45), "carry"), ("set on a bed (0.55 m)", 0.80, (0.40, 0.45, 0.50), "carry"),
         ("set on a washer / counter (0.90 m)", 1.15, (0.40, 0.45, 0.50), "carry")]


def duty_ok(t, rated, peak):
    rise = 60.0 * (t / rated) ** 2 * (1 - math.exp(-120 / 600))
    return peak / max(t, 1e-6), rise
JOINTS = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}


def design():
    from .params import BASELINE
    j = dict(BASELINE.arm.joints)
    j["shoulder_pitch"] = ("GIM8108-8", 3.0)
    j["elbow"] = ("GIM8108-8", 3.0)
    return dataclasses.replace(BASELINE, arm=dataclasses.replace(BASELINE.arm, joints=j, payload=PER_HAND))


def run():
    import mujoco
    from .arm_sizing import capacity
    from .twin import Reacher, margin, support_polygon
    d = design()
    R = Reacher(d)
    m, dat = R.m, R.d
    jid = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{j}_r") for j in JOINTS}
    cap = {j: capacity(d, full) for j, full in JOINTS.items()}
    pay = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload_r")
    poly = support_polygon("xdrive", 0.21, 0)
    rows = []
    for name, z, xs, rule in POSES:
        for x in xs:
            target = np.array([x, -HALF_SPAN, z])
            sols = R.solve(target, [0, 0, -1.0], seeds=10)
            best = None
            for q in sols:
                dat.qpos[R.adr] = q
                dat.qvel[:] = 0
                mujoco.mj_forward(m, dat)
                t = {}
                for j in JOINTS:
                    v = dat.qfrc_bias[m.jnt_dofadr[jid[j]]]
                    if j == "pitch":
                        v -= d.arm.spring_k_nm * math.sin(dat.qpos[m.jnt_qposadr[jid[j]]])
                    t[j] = abs(v)
                use = {j: cap[j][3] / max(t[j], 1e-6) for j in JOINTS}                      # peak margin
                rise = {j: duty_ok(t[j], cap[j][2], cap[j][3])[1] for j in JOINTS}
                need = 1.5 if rule == "peak" else 2.0
                worst = min(min(u / need for u in use.values()), min(15.0 / max(r, 1e-6) for r in rise.values()) if rule == "carry" else 9)
                c = R.com(q)
                pr = dat.xipos[pay].copy()
                pl = pr * np.array([1, -1, 1])                               # the left hand mirrors the right
                M = float(m.body_subtreemass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")])
                ctot = (c * M + pl * PER_HAND) / (M + PER_HAND)
                tip0 = margin(poly, ctot[:2])                                # load toward a wheel (0°)
                if best is None or worst > best["worst"]:
                    best = dict(pose=name, x=x, z=z, rule=rule, t=t, use=use, rise=rise, worst=worst, tip_wheel=float(tip0), com_x=float(ctot[0]))
            rows.append(best or dict(pose=name, x=x, z=z, rule=rule, t=None, use=None, worst=0.0, tip_wheel=float("nan"), com_x=float("nan")))
            print(rows[-1]["pose"], x, z, None if not best else {k: round(v, 2) for k, v in best["use"].items()}, flush=True)
    return rows, cap


def main():
    rows, cap = run()
    ok = all(r["t"] is not None and r["worst"] >= 1.0 and r["tip_wheel"] >= 0.03 for r in rows if r["x"] <= 0.45 and "washer" not in r["pose"] and "0.80" not in r["pose"])
    L = ["# R2d: the 20 lb laundry basket (generated by `python -m rnd.basket`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**{'The basket task passes' if ok else 'The basket task fails'}** (floor pick, carry with handles at 0.70 m, set on a bed, basket centre ≤ 0.45 m out; D20 actuators). Capacity rated / peak: " +
         "; ".join(f"{j} {c[2]:.1f} / {c[3]:.1f} N·m" for j, c in cap.items()), "",
         "| Pose | Basket centre x (m) | Handles z (m) | Rule | Shoulder pitch | Roll | Yaw | Elbow | Wrist pitch | Tip margin, load toward a wheel |",
         ]
    L[-1] = "| Pose | Basket centre x (m) | Handles z (m) | Rule | Shoulder pitch N·m (peak×, 2-min rise) | Roll | Yaw | Elbow | Wrist pitch | Tip, load toward a wheel |"
    L += [
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["t"] is None:
            L.append(f"| {r['pose']} | {r['x']:.2f} | {r['z']:.2f} | {r['rule']} | no collision-free IK | | | | | |")
            continue
        c = lambda j: f"{r['t'][j]:.1f} ({r['use'][j]:.1f}×, {r['rise'][j]:.0f} K)"
        rl = "peak ≥ 1.5×" if r["rule"] == "peak" else "peak ≥ 2×, rise ≤ 15 K"
        L.append(f"| {r['pose']} | {r['x']:.2f} | {r['z']:.2f} | {rl} | {c('pitch')} | {c('roll')} | {c('yaw')} | {c('elbow')} | {c('wpitch')} | "
                 f"{r['tip_wheel'] * 1000:.0f} mm {'✅' if r['worst'] >= 1.0 and r['tip_wheel'] >= 0.03 else '❌'} |")
    (DOCS / "basket.md").write_text("\n".join(L))
    (DOCS / "basket.json").write_text(json.dumps({"ok": ok, "rows": rows}, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
