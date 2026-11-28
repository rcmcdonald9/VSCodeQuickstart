"""Payload study for the owner's new requirement "it needs to hold 20 lb" (2026-11-27).  python -m rnd.payload_study

Planar statics in the arm's vertical plane (masses from rnd/params.py via arm_sizing.point_masses), for poses defined by
the upper-arm angle from vertical (θu) and the forearm angle from vertical (θf); the hand continues the forearm. The
payload sits at the grasp point, or on the forearm/palm for a hug carry. Gravity torque at shoulder pitch (minus the
balancer K·sin θu), elbow and wrist pitch, against the current actuators × ratio × belt efficiency (rated and peak).

Scenarios (20 lb = 9.07 kg):
- A: one hand, 9.07 kg at the grasp point, arm straight out
- B: two arms, 4.54 kg each, hug carry: upper arm hanging, forearm horizontal, load centred 0.20–0.30 m from the elbow
  axis on the forearms/palms (the wrist sees ≈ nothing). **The R2c spec point is 0.25 m** (owner, 2026-11-27: "held close, load
  on the forearms ≤ 0.25 m from the elbows"); 0.20 and 0.30 m bracket it (Astra review #3 Part A: the study must not quote 0.20 m
  as the pass). In this pose the upper arm hangs, so actuator masses on the upper arm or on the elbow axis add nothing to the
  pitch or elbow moment: only the forearm, hand and load count, and the result does not depend on which body carries the
  elbow actuator's housing.
- C: two arms, 4.54 kg each at the grasp points, arm at 45° (lifting a bag off a counter)
Rule: rated / static ≥ 1.5 for holds; peak / static ≥ 1.5 for brief lifts (< 10 s).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .arm_sizing import capacity, point_masses
from .params import BASELINE, G

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
LB20 = 20 * 0.45359


def torques(d, th_u, th_f, payload, load_at=None):
    """Gravity torques (N·m) at shoulder pitch, elbow, wrist pitch. Angles from vertical (rad). load_at: distance of the
    payload from the elbow along the forearm (hug carry); default = the grasp point."""
    a = d.arm
    pts, _ = point_masses(d)
    lu, lf, lh = a.l_upper, a.l_fore, a.l_hand
    su, sf = math.sin(th_u), math.sin(th_f)
    xe = lu * su                                                   # elbow x
    xw = xe + lf * sf                                              # wrist x
    out = {"shoulder_pitch": 0.0, "elbow": 0.0, "wrist_pitch": 0.0}
    for n, m, x in pts:
        if n == "payload":
            continue
        if x <= lu + 1e-9:
            xx = x * su
        elif x <= lu + lf + 1e-9:
            xx = xe + (x - lu) * sf
        else:
            xx = xw + (x - lu - lf) * sf
        out["shoulder_pitch"] += m * G * xx
        if x > lu + 1e-9:
            out["elbow"] += m * G * (xx - xe)
        if x > lu + lf + 1e-9:
            out["wrist_pitch"] += m * G * (xx - xw)
    xp = xe + (load_at if load_at is not None else lf + lh) * sf
    out["shoulder_pitch"] += payload * G * xp
    out["elbow"] += payload * G * (xp - xe)
    if load_at is None:
        out["wrist_pitch"] += payload * G * (xp - xw)
    out["shoulder_pitch"] = abs(out["shoulder_pitch"] - a.spring_k_nm * su)
    return out


CASES = [
    ("Today's spec: 1 kg, one hand, arm straight out", 1.0, math.pi / 2, math.pi / 2, None),
    ("A: 20 lb in one hand, arm straight out", LB20, math.pi / 2, math.pi / 2, None),
    ("A': 20 lb in one hand, elbow at 90° (hand 0.43 m out)", LB20, 0.0, math.pi / 2, None),
    ("B: 20 lb two-handed hug, 10 lb per arm, load 0.20 m out on the forearms", LB20 / 2, 0.0, math.pi / 2, 0.20),
    ("B (R2c spec): same, load 0.25 m out", LB20 / 2, 0.0, math.pi / 2, 0.25),
    ("B: same, load 0.30 m out", LB20 / 2, 0.0, math.pi / 2, 0.30),
    ("C: 20 lb two-handed bag, 10 lb per hand, arm at 45°, elbow straight", LB20 / 2, math.pi / 4, math.pi / 4, None),
]


def run(d=BASELINE):
    cap = {j: capacity(d, j) for j in ("shoulder_pitch", "elbow", "wrist_pitch")}
    rows = []
    for name, m, tu, tf, la in CASES:
        t = torques(d, tu, tf, m, la)
        rows.append(dict(case=name, payload=m, t=t, rated={j: cap[j][2] / max(t[j], 1e-6) for j in t}, peak={j: cap[j][3] / max(t[j], 1e-6) for j in t}))
    return rows, cap


def ripple():
    """Lift, base threshold and static tip with the 9.07 kg hug load (quasi-static, same models as lift.py / drive.py / tip.py)."""
    from . import drive, lift
    from .params import act
    out = ["| Check | Today (2 kg) | With 20 lb | Rule | |", "|---|---|---|---|---|"]
    keep = (lift.PAYLOAD, lift.ACCEL)
    lift.PAYLOAD, lift.ACCEL = 2.0, 1.0
    c0 = lift.option_c(BASELINE)
    lift.PAYLOAD, lift.ACCEL = LB20, 0.3
    c1 = lift.option_c(BASELINE)
    lift.PAYLOAD, lift.ACCEL = keep
    pk = act("GIM6010-8")["peak"]
    out.append(f"| Lift GIM6010: rated / peak margin (20 lb moves at 0.3 m/s²; the pawl holds it at 0 W) | {c0['m_torque']:.2f}× / {pk / c0['torque']:.1f}× | "
               f"{c1['m_torque']:.2f}× / {pk / c1['torque']:.1f}× | peak ≥ 1.5× for moves < 5 s | {'✅' if pk / c1['torque'] >= 1.5 else '❌'} |")
    m0 = drive.robot_mass(BASELINE)
    T = lambda M: drive.step_force(M * G / 4, BASELINE.base.wheel_d / 2, 0.015) / math.cos(math.radians(45)) * BASELINE.base.wheel_d / 2
    out.append(f"| Base: 15 mm threshold, wheel peak margin | {11 / T(m0):.2f}× | {11 / T(m0 + LB20 - 2):.2f}× | ≥ 1.5× | {'✅' if 11 / T(m0 + LB20 - 2) >= 1.5 else '❌'} |")
    M = m0 + LB20 - 2
    x = BASELINE.arm.shoulder_x + 0.25                                  # hug load ≈ 0.25 m in front of the shoulders
    shift = LB20 * x / M
    h = 0.75                                                           # COM height with the load at chest height (ESTIMATE)
    m45 = 0.21 * math.cos(math.radians(45)) - shift
    out.append(f"| Static tip, hug load between wheels (45°): COM shift {shift * 1000:.0f} mm vs 148 mm edge | — | {m45 * 1000:.0f} mm | ≥ 50 mm | {'✅' if m45 >= 0.05 else '❌'} |")
    out.append(f"| Same + braking 0.3 m/s² | — | {(m45 - h / G * 0.3) * 1000:.0f} mm | ≥ 30 mm | {'✅' if m45 - h / G * 0.3 >= 0.03 else '❌'} |")
    out += ["", "Planner rules with a load over 1 kg (A10) still apply: carry it pointing at a wheel, brake ≤ 0.3 m/s², crawl near people."]
    return out


def main():
    rows, cap = run()
    L = ["# Payload study: 20 lb (generated by `python -m rnd.payload_study`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         "Capacity now (rated / peak at the joint): " + "; ".join(f"{j} {c[0]} ×{c[1]:.2f}: {c[2]:.1f} / {c[3]:.1f} N·m" for j, c in cap.items()), "",
         "| Case | Shoulder pitch N·m (rated× / peak×) | Elbow N·m (rated× / peak×) | Wrist pitch N·m (rated× / peak×) |", "|---|---|---|---|"]
    for r in rows:
        cell = lambda j: f"{r['t'][j]:.1f} ({r['rated'][j]:.2f}× / {r['peak'][j]:.2f}×) {'✅' if r['rated'][j] >= 1.5 else ('⚠️ brief only' if r['peak'][j] >= 1.5 else '❌')}"
        L.append(f"| {r['case']} | {cell('shoulder_pitch')} | {cell('elbow')} | {cell('wrist_pitch')} |")
    # D20 candidate: GIM8108-8 at shoulder pitch and elbow, both 3:1 (owner picked scenario B, 2026-11-27)
    from .params import with_arm_motors
    d2 = with_arm_motors(BASELINE, "d20")         # 2026-11-27: D20 motors + the D20 CAD masses (params.ARM_MOTOR_SETS)
    rows2, cap2 = run(d2)
    L += ["", "## D20 candidate: GIM8108-8 at shoulder pitch and elbow, both 3:1", "",
          "Capacity: " + "; ".join(f"{k} {c[2]:.1f} / {c[3]:.1f} N·m" for k, c in cap2.items()), "",
          "| Case | Shoulder pitch (rated× / peak×) | Elbow (rated× / peak×) |", "|---|---|---|"]
    for r in rows2:
        L.append(f"| {r['case']} | {r['rated']['shoulder_pitch']:.2f}× / {r['peak']['shoulder_pitch']:.2f}× | {r['rated']['elbow']:.2f}× / {r['peak']['elbow']:.2f}× |")
    hug = {la: next(r for r in rows2 if r["case"].startswith("B") and f"{la:.2f} m out" in r["case"]) for la in (0.20, 0.25, 0.30)}
    w = lambda r, k="rated": min(r[k]["shoulder_pitch"], r[k]["elbow"])
    h0 = next(r for r in rows if r["case"].startswith("B (R2c spec)"))
    L += ["", "**R2c (owner, 2026-11-27): 20 lb carried two-handed, held close (load on the forearms ≤ 0.25 m from the elbows).** "
          f"At the spec 0.25 m the D20 motors give **{w(hug[0.25]):.2f}× rated** ({w(hug[0.25], 'peak'):.1f}× peak): just over the 1.5× rule, "
          f"with equal sharing between the hands and before thermal derating (0.20 m: {w(hug[0.20]):.2f}×, 0.30 m: {w(hug[0.30]):.2f}×). "
          f"The GIM6010s give {w(h0):.2f}× rated at 0.25 m: a brief lift only ({w(h0, 'peak'):.2f}× peak). Unequal sharing between the "
          "hands: rnd/basket.py share study (docs/rnd/basket.md)."]
    L += ["", "## The rest of the robot with 20 lb aboard (R2c ripple check, loop 45)", ""] + ripple()
    (DOCS / "payload_study.md").write_text("\n".join(L))
    (DOCS / "payload_study.json").write_text(json.dumps(rows, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
