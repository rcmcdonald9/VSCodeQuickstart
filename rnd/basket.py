"""R2d: the 20 lb laundry basket (owner, 2026-11-27).  python -m rnd.basket

Basket: ≈ 60 × 45 × 28 cm, 9.07 kg, a hand-hold slot in each short end just under the rim. Both grippers hook into
the end slots with the hands pointing down (the load hangs from a rigid fingertip hook, D21), so each hand carries
4.54 kg directly under its wrist; the hands are ≈ 0.60 m apart (shoulders 0.40 m), so the arms splay outward.

Full 3D statics on the twin (rnd/twin.py: lift + 6-joint right arm, CAD masses, with the actuator masses on the bodies that
carry them, rnd/inline_study.study_xml, Astra #3 Part A; the left arm mirrors it): for each pose
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

**Unequal shares (Astra review #3 major 10, 2026-11-28).** The equal 4.54 kg per hand is a best case: a tilted basket,
contents piled at one end, one hook seating first, arm tracking error or a handle slipping all move load to one hand. The
share study re-evaluates the same IK solutions (the hand positions do not change) with the heavier hand at 60 % and 70 %
of 9.07 kg (and 100 %: one hook released in the air), the other hand the rest, and a seating factor DYN_SEAT on the
floor pick (the second hook taking up its slack while the lift rises; ESTIMATE). The heavier hand is the right arm (the
twin's IK arm); the tip check carries both hands' loads. Run for D20 (B1) and B1y (D20 + inline DM-J4340-2EC yaw).
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
TOTAL = 20 * 0.45359
PER_HAND = TOTAL / 2
HALF_SPAN = 0.30                     # handle slots ±0.30 m from the centre line
POSES = [("floor pick", 0.25, (0.40, 0.45, 0.50), "peak"), ("carry", 0.70, (0.35, 0.40, 0.45), "carry"),
         ("carry, handles at 0.80", 0.80, (0.35, 0.40, 0.45), "carry"), ("set on a bed (0.55 m)", 0.80, (0.40, 0.45, 0.50), "carry"),
         ("set on a washer / counter (0.90 m)", 1.15, (0.40, 0.45, 0.50), "carry")]
# Share study (Astra #3 major 10). The heavier hand's share of TOTAL; 1.0 = one hook released in the air (must abort).
SHARES = (0.50, 0.55, 0.60, 0.65, 0.70, 1.00)
REPORT_SHARES = (0.50, 0.60, 0.70, 1.00)
DYN_SEAT = 1.3                       # ESTIMATE: floor pick, the late hook seating while the lift rises (slow lift, a few mm of slack
                                     # in the hook + arm compliance); applied to the heavier hand on the peak rule only
TORQUE_SENSE = 0.07                  # ESTIMATE: share uncertainty of a hand-load estimate from joint currents (±10 % torque constant,
                                     # ±0.5 N·m joint friction / belt drag at the elbow and pitch, over ≈ 45 N per hand)
VARIANTS = {"B1": "d20", "B1y": "d20_yaw"}


def duty_ok(t, rated, peak):
    rise = 60.0 * (t / rated) ** 2 * (1 - math.exp(-120 / 600))
    return peak / max(t, 1e-6), rise
JOINTS = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}


def design(motors="d20"):
    from .params import BASELINE, with_arm_motors
    d = with_arm_motors(BASELINE, motors)         # 2026-11-27: the D20 motors with the D20 CAD masses (params.ARM_MOTOR_SETS)
    return dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=PER_HAND))


def _solutions(d):
    """[(pose, x, z, rule, [q…])] and the reacher (IK depends only on geometry, not on the load)."""
    from .twin import Reacher
    R = Reacher(d)
    out = []
    for name, z, xs, rule in POSES:
        for x in xs:
            out.append((name, x, z, rule, R.solve(np.array([x, -HALF_SPAN, z]), [0, 0, -1.0], seeds=10)))
    return R, out


def _evaluate(d, R, sols, load_r, load_l, factor=1.0, verbose=False):
    """Rows (one per pose) for the right hand carrying load_r (× factor on the peak rule), the left load_l."""
    import mujoco
    from .arm_sizing import capacity
    from .twin import margin, support_polygon
    m, dat = R.m, R.d
    jid = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{j}_r") for j in JOINTS}
    cap = {j: capacity(d, full) for j, full in JOINTS.items()}
    pay = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload_r")
    poly = support_polygon("xdrive", 0.21, 0)
    M = float(m.body_subtreemass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "robot")]) - float(m.body_mass[pay])
    keep = float(m.body_mass[pay]), m.body_inertia[pay].copy()
    rows = []
    try:
        for name, x, z, rule, qs in sols:
            load = load_r * (factor if rule == "peak" else 1.0)
            m.body_mass[pay] = max(load, 1e-6)
            m.body_inertia[pay] = keep[1] * max(load, 1e-6) / max(keep[0], 1e-9)
            best = None
            for q in qs:
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
                per = {j: min(use[j] / need, 15.0 / max(rise[j], 1e-6) if rule == "carry" else 9) for j in JOINTS}
                worst = min(per.values())
                c = R.com(q)                                                 # robot COM, the right-hand body at `load`
                pr = dat.xipos[pay].copy()
                pl = pr * np.array([1, -1, 1])                               # the left hand mirrors the right
                ctot = (c * (M + load) - pr * load + pr * load_r + pl * load_l) / (M + load_r + load_l)
                tip0 = margin(poly, ctot[:2])                                # load toward a wheel (0°)
                if best is None or worst > best["worst"]:
                    best = dict(pose=name, x=x, z=z, rule=rule, t=t, use=use, rise=rise, per=per, worst=worst, tip_wheel=float(tip0),
                                com_x=float(ctot[0]), load_r=load, load_l=load_l)
            rows.append(best or dict(pose=name, x=x, z=z, rule=rule, t=None, use=None, per=None, worst=0.0, tip_wheel=float("nan"),
                                     com_x=float("nan"), load_r=load, load_l=load_l))
            if verbose:
                print(rows[-1]["pose"], x, z, None if not best else {k: round(v, 2) for k, v in best["use"].items()}, flush=True)
    finally:
        m.body_mass[pay], m.body_inertia[pay] = keep
    return rows, cap


def run(d=None):
    """Equal shares (4.54 kg per hand): rows, capacity. d.arm.payload is the per-hand load."""
    d = d or design()
    R, sols = _solutions(d)
    return _evaluate(d, R, sols, d.arm.payload, d.arm.payload, verbose=True)


def task_rows(rows):
    """The rows the R2d gate counts: floor pick, carry at 0.70 m, set on a bed, basket centre ≤ 0.45 m."""
    return [r for r in rows if r["x"] <= 0.45 and "washer" not in r["pose"] and "0.80" not in r["pose"]]


def share_study(d, shares=SHARES):
    """{share: dict(rows, worst, fails {joint: worst per-rule margin}, tip)} for the heavier hand at share × TOTAL."""
    R, sols = _solutions(d)
    out = {}
    for s in shares:
        rows, cap = _evaluate(d, R, sols, s * TOTAL, (1 - s) * TOTAL, DYN_SEAT if s != 0.5 else 1.0)
        tr = task_rows(rows)
        ok_rows = [r for r in tr if r["t"] is not None]
        per = {j: min(r["per"][j] for r in ok_rows) for j in JOINTS} if ok_rows else {}
        where = {j: min(ok_rows, key=lambda r: r["per"][j])["pose"] + f" x {min(ok_rows, key=lambda r: r['per'][j])['x']:.2f}" for j in JOINTS} if ok_rows else {}
        out[s] = dict(rows=rows, per=per, where=where, worst=min(per.values()) if per else 0.0,
                      tip=min((r["tip_wheel"] for r in ok_rows), default=float("nan")),
                      fails=[j for j, v in per.items() if v < 1.0], all_ik=all(r["t"] is not None for r in tr))
    return out


def max_share(study):
    """Largest heavier-hand share on the SHARES grid at which every joint still passes (linear between grid points)."""
    ss = sorted(study)
    last = None
    for a, b in zip(ss, ss[1:]):
        wa, wb = study[a]["worst"], study[b]["worst"]
        if wa >= 1.0 and wb < 1.0:
            return a + (b - a) * (wa - 1.0) / (wa - wb)
        if wa >= 1.0:
            last = b if wb >= 1.0 else a
    return last if last is not None else (ss[0] if study[ss[0]]["worst"] >= 1.0 else None)


def rule_text(lims):
    """The planner / supervisor rule for the software owner, from the per-variant share limits."""
    s_lim = min(v for v in lims.values() if v is not None) if any(v is not None for v in lims.values()) else 0.5
    allow = max(0.5, s_lim - TORQUE_SENSE)
    return [
        "## Proposed planner / supervisor rule: measure the share before the basket leaves the floor (for the software owner)", "",
        f"The joints set the limit: the heavier hand may carry at most **{s_lim * 100:.0f} %** of the basket on D20 / B1y before a joint "
        f"leaves its rule (table above). A hand-load estimate from joint currents is uncertain by about ±{TORQUE_SENSE * 100:.0f} percentage "
        f"points of share (ESTIMATE: ±10 % torque constant, ±0.5 N·m friction at pitch / elbow against ≈ 45 N per hand), so the rule refuses "
        f"above **{allow * 100:.0f} %** measured. Until the bench calibrates the estimate, that leaves "
        + ("almost no room: treat R2d as conditional and design the grasp to centre the load (handle capture, a centring pick)." if allow <= 0.52 else "a usable window.")
        + "",
        "",
        "1. **Capture check (before any lift).** Both grippers closed on their handles, the D21 hooks seated: gripper position at the "
        "hook stop, and a 1–2 mm downward hand nudge that loads both hooks (joint-current rise on both arms). One hook not confirmed → abort, "
        "re-grasp. No lift on a single confirmed hook.",
        "2. **Tare lift.** Raise the lift slowly (≤ 20 mm/s) until the summed hand load is ≥ 90 % of the expected basket weight or the lift "
        "has risen 25 mm, then hold 0.5 s: the basket is just clear of the floor, so an abort drops it ≤ 25 mm. Before this lift, record each "
        "arm's gravity-model residual (τ_measured − τ_model(q) with an empty hand) as the zero.",
        "3. **Measure each hand's load** from that arm's joint torques: F_hand = (Jᵀ)⁺ (τ_measured − τ_model(q) − τ_zero), using pitch, roll, "
        "elbow and wrist pitch (the 3:1 belt joints are the best-sensed); vertical components F_r, F_l, total W = F_r + F_l.",
        f"4. **Refuse** (set back down along the pick path, open the hooks) when: the heavier share max(F_r, F_l) / W > **{allow * 100:.0f} %**; "
        f"either hand < 25 % (a hook not really carrying); W > 1.1 × {TOTAL:.2f} kg (overweight for R2d); or the two arms' estimates "
        "disagree with the lift motor's load change by more than 15 %.",
        "5. **Monitor through the carry and the set-down** at the control rate: the same share and total tests with hysteresis "
        f"(abort at {min(s_lim, allow + 0.05) * 100:.0f} %), plus a slip test (one hand's load falling > 30 % in 0.2 s). On abort: lower the lift and "
        "set the basket down along a pre-planned unload trajectory; never open the grippers or drop torque under load, and never let a "
        "thermal or current cut-out be the first response (thermal.md section 3: derate before the limit).",
        "6. **Log** F_r, F_l, W and the refusal reason for every basket lift: the field data that replaces the ESTIMATE seating factor "
        f"({DYN_SEAT}) and the sensing uncertainty.",
        "",
        "The 100 % row is the case the rule must catch in the air (one hook released or slipped): every heavy joint is far past its rule, "
        "so the supervisor's response is a controlled set-down, not 'hold on'. The wrist bearing, the hook and the gripper's own capture "
        "load with one hand carrying 70–100 % are outside these joint checks (Astra #3 major 10: test them tethered).",
    ]


def main():
    from .inline_study import corrected_bodies
    with corrected_bodies():                 # actuator masses on the bodies that carry them (Astra #3 Part A b; rnd/inline_study.study_xml)
        _main()


def _main():
    res, studies, caps = {}, {}, {}
    for k, s in VARIANTS.items():
        d = design(s)
        R, sols = _solutions(d)
        rows, cap = _evaluate(d, R, sols, PER_HAND, PER_HAND, verbose=True)
        res[k], caps[k] = rows, cap
        studies[k] = share_study(d)
    rows, cap = res["B1"], caps["B1"]
    ok = all(r["t"] is not None and r["worst"] >= 1.0 and r["tip_wheel"] >= 0.03 for r in task_rows(rows))
    oky = all(r["t"] is not None and r["worst"] >= 1.0 and r["tip_wheel"] >= 0.03 for r in task_rows(res["B1y"]))
    L = ["# R2d: the 20 lb laundry basket (generated by `python -m rnd.basket`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**{'The basket task passes' if ok else 'The basket task fails'}** with equal shares (floor pick, carry with handles at 0.70 m, set on a bed, "
         f"basket centre ≤ 0.45 m out; D20 actuators); B1y (inline DM-J4340-2EC yaw) **{'passes' if oky else 'fails'}**. Capacity rated / peak: " +
         "; ".join(f"{j} {c[2]:.1f} / {c[3]:.1f} N·m" for j, c in cap.items()) + f" (B1y yaw {caps['B1y']['yaw'][2]:.1f} / {caps['B1y']['yaw'][3]:.1f} N·m)", ""]

    def table(rows):
        T = ["| Pose | Basket centre x (m) | Handles z (m) | Rule | Shoulder pitch N·m (peak×, 2-min rise) | Roll | Yaw | Elbow | Wrist pitch | Tip, load toward a wheel |",
             "|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            if r["t"] is None:
                T.append(f"| {r['pose']} | {r['x']:.2f} | {r['z']:.2f} | {r['rule']} | no collision-free IK | | | | | |")
                continue
            c = lambda j: f"{r['t'][j]:.1f} ({min(r['use'][j], 999):.1f}×, {r['rise'][j]:.0f} K)"
            rl = "peak ≥ 1.5×" if r["rule"] == "peak" else "peak ≥ 2×, rise ≤ 15 K"
            T.append(f"| {r['pose']} | {r['x']:.2f} | {r['z']:.2f} | {rl} | {c('pitch')} | {c('roll')} | {c('yaw')} | {c('elbow')} | {c('wpitch')} | "
                     f"{r['tip_wheel'] * 1000:.0f} mm {'✅' if r['worst'] >= 1.0 and r['tip_wheel'] >= 0.03 else '❌'} |")
        return T
    L += ["## D20 (B1), equal shares", ""] + table(rows)
    L += ["", "## B1y (D20 + inline DM-J4340-2EC yaw), equal shares", ""] + table(res["B1y"])
    lims = {k: max_share(v) for k, v in studies.items()}
    L += ["", "## Unequal shares (Astra review #3 major 10)", "",
          f"Heavier hand at the share of {TOTAL:.2f} kg shown, the other hand the rest; floor pick × {DYN_SEAT} seating factor on the heavier hand "
          "(ESTIMATE) for every unequal share. Cell = the joint's worst margin over the R2d task poses against its rule (≥ 1 passes: "
          "peak ≥ 1.5× on the pick, peak ≥ 2× and 2-min rise ≤ 15 K on the carry), with the pose that sets it.", "",
          "| Variant | Heavier hand | Load (kg) | Shoulder pitch | Roll | Yaw | Elbow | Wrist pitch | Worst | Tip (mm) | Joints that fail |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for k, st in studies.items():
        for s in REPORT_SHARES:
            v = st[s]
            cell = lambda j: f"{v['per'][j]:.2f} {'✅' if v['per'][j] >= 1 else '❌'} ({v['where'][j]})"
            L.append(f"| {k} | {s * 100:.0f} % | {s * TOTAL:.2f} / {(1 - s) * TOTAL:.2f} | " + " | ".join(cell(j) for j in JOINTS) +
                     f" | **{v['worst']:.2f}** | {v['tip'] * 1000:.0f} | {', '.join(v['fails']) or 'none'} |")
    L += ["", "Largest heavier-hand share with every joint passing (interpolated on the " + ", ".join(f"{s * 100:.0f}" for s in SHARES) + " % grid): "
          + "; ".join(f"**{k} {v * 100:.0f} %**" if v is not None else f"**{k}: fails at 50 %**" for k, v in lims.items()) + ". "
          "The limiting rule is the shoulder pitch's 2-minute rise on the carry (rnd/thermal.md's open-air 60 K model; the D20 pitch carries ≈ its rated "
          "torque at 0.45 m with equal shares) and then its peak margin on the pick with the seating factor; the elbow and the B1y yaw keep "
          "margin. A rule on the 2-min rise is a thermal rule: thermal.md section 3 has the same carry in the capped mount.", ""]
    L += rule_text(lims)
    (DOCS / "basket.md").write_text("\n".join(L) + "\n")
    js = {"ok": ok, "ok_b1y": oky, "rows": rows, "variants": res,
          "shares": {k: {f"{s:.2f}": dict(per=v["per"], where=v["where"], worst=v["worst"], tip=v["tip"], fails=v["fails"]) for s, v in st.items()} for k, st in studies.items()},
          "share_rows": {k: {f"{s:.2f}": st[s]["rows"] for s in REPORT_SHARES} for k, st in studies.items()},
          "max_share": lims, "dyn_seat": DYN_SEAT, "torque_sense": TORQUE_SENSE}
    (DOCS / "basket.json").write_text(json.dumps(js, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
