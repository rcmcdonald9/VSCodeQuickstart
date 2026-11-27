"""Tip-over, A10 v2 (review 2026-11-27 finding 4; loop 40 A10 / external review finding 11).  python -m rnd.tip

Quasi-static zero-moment-point **reserve** on the twin's mass model (rnd/twin.py: CAD masses from params, both arms in
the pose, the load split between the hands): ZMP = COM_xy + (h_com / g)·a·u + F·H / (M g)·u, braking deceleration a
and the push F in the same direction u. Loop 40 only looked along the load direction; here u is **swept over 360°**
(the octagon's weak headings are not the load's). Reserve = how much further the ZMP can move before the robot tips:
- wheels only: distance to the x-drive diamond (wheel contacts at r 0.21 m at 0/90/180/270°: 148 mm to the edge
  between wheels, against 210 mm toward a wheel);
- with anti-tip casters at the diagonals: a caster only carries load once the base has tilted about the wheel-to-wheel
  edge far enough to take up its free travel δ (D18: the 5 mm spring stroke to its hard stop; D18b: the lock's take-up),
  θ = δ / d, d = the caster's distance beyond that edge. The tilt first moves the COM h·θ outward, so
  reserve = max(diamond margin, octagon margin at ZMP + h·θ·u). Loop 40 counted the casters as rigid (θ = 0).

Cases (A10, 16): load 1 kg (R12) and 2.5 kg two-handed; the load COM toward a wheel (0°) or between wheels (45°);
braking at the base's acceleration limit; a 30 N push 1.0 m up (a hand on the shoulder); both. The pose and the braking
limit are what the safety supervisor allows (sw/safety.py is read, not copied): load mode above LOAD_KG = hands inside
LOAD_WS and base_accel_limit(); at or under LOAD_KG the arms may be straight out at shoulder height, lift mid (the old
worst pose). Target: reserve ≥ 30 mm in every case (twin v1 dynamic rule); R12 keeps its own 50 mm static rule.
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
G = 9.81
F_PUSH, H_PUSH = 30.0, 1.0
A_LOOP40 = 0.5            # the braking value loop 40 assumed (D2b "while manipulating"; sw/safety.py does not enforce it)
A_PERSON = 0.1            # requested rule SW-T1: base accel / decel cap while a person is within reach
PERSON_X = 0.35           # requested rule SW-T1: hand box x max (m from the base centre) while a person is within reach
PERSON_Z = 0.60           # requested rule SW-T1: hands (so the lift) no higher than this while a person is within reach with a load
CROSS_SLOW = 0.12         # requested planner rule SW-T4: D2b crossing speed with the ballast fitted (was 0.15 m/s)
NEED = 0.030
LOADS, AZS = (1.0, 2.5), (0.0, 45.0)
DIST = ((False, False), (True, False), (False, True), (True, True))     # (braking, push)
ARM_J = ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")
MIRROR = {"pitch": 1, "roll": 1, "yaw": -1, "elbow": 1, "wpitch": 1, "wroll": -1}   # left arm = mirror of the right


def sw_rules():
    """What sw/safety.py enforces today (read from the module, so this follows the software)."""
    from sw import safety as s
    return SimpleNamespace(load_kg=s.LOAD_KG, box=s.LOAD_WS,
                           accel=lambda kg: s.Supervisor.base_accel_limit(SimpleNamespace(payload=kg)),
                           # SW-T1 / SW-T2 count as enforced once the supervisor has these (names proposed below)
                           person=hasattr(s.Supervisor, "set_person_near"), casters=hasattr(s.Supervisor, "casters_locked"))


# ------------------------------------------------------------------------------------------------ designs and polygons
def design_before():
    """The robot as it stood before A10 v2: no power board in any budget, D18 casters unmassed, no ballast."""
    from .params import BASELINE
    return dataclasses.replace(BASELINE, base=dataclasses.replace(BASELINE.base, power_board_mass=0.0, caster_mass=0.0, ballast_on=False))


def design_after(ballast=None):
    from .params import BASELINE
    b = BASELINE.base if ballast is None else dataclasses.replace(BASELINE.base, ballast_mass=ballast, ballast_on=ballast > 0)
    return dataclasses.replace(BASELINE, base=b)


def casters(d, kind):
    b = d.base
    if kind == "D18":
        return dict(r=0.22, take_up=0.005, name="D18 (loop 40): ball transfers, 5 N, hard stop after 5 mm")
    if kind == "D18b":
        return dict(r=b.caster_r, take_up=b.caster_take_up, name=f"D18b v2: lockable PTFE skid feet, locked (take-up {b.caster_take_up * 1000:.1f} mm)")
    return None


def diamond(d):
    from .twin import support_polygon
    return support_polygon("xdrive", d.base.wheel_radius_pos, 0)


def octagon(d, rc):
    rw = d.base.wheel_radius_pos
    return np.array([[(rw if i % 2 == 0 else rc) * math.cos(math.radians(45 * i)), (rw if i % 2 == 0 else rc) * math.sin(math.radians(45 * i))]
                     for i in range(8)])


def reserve(d, cas, c, h, D, az_list=np.arange(0.0, 360.0, 2.0)):
    """Worst reserve (m) over push/brake headings u; c = COM xy, D = |brake + push| ZMP shift."""
    from .twin import margin
    dia = diamond(d)
    oct_ = octagon(d, cas["r"]) if cas else None
    th = tilt(d, cas)
    worst, at = 9.0, 0.0
    for a in az_list:
        u = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        z = c + D * u
        r = margin(dia, z)
        if cas:
            r = max(r, margin(oct_, z + h * th * u))
        if r < worst:
            worst, at = r, a
    return worst, at


# ---------------------------------------------------------------------------------------------------- mass model / poses
class Body:
    """COM of a design with both arms in a pose (left mirrors right) and a load split between the hands."""

    def __init__(self, d):
        import mujoco
        from .twin import build_xml
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_string(build_xml(d))
        self.d = mujoco.MjData(self.m)
        for s in "rl":
            self.m.body_mass[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"payload_{s}")] = 1e-4
        self.adr = {n: self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in
                    ["lift"] + [f"{j}_{s}" for j in ARM_J for s in "rl"]}
        lid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "lift")
        self.lift_mid = 0.5 * (self.m.jnt_range[lid][0] + self.m.jnt_range[lid][1])

    def com(self, q, load):
        """q = [lift, pitch, roll, yaw, elbow, wpitch, wroll] (right arm, rad). Returns M, COM (3), hand midpoint (3)."""
        mj, m, dat = self.mj, self.m, self.d
        dat.qpos[self.adr["lift"]] = q[0]
        for i, j in enumerate(ARM_J):
            dat.qpos[self.adr[f"{j}_r"]] = q[1 + i]
            dat.qpos[self.adr[f"{j}_l"]] = MIRROR[j] * q[1 + i]
        mj.mj_kinematics(m, dat)
        w = m.body_mass[1:]
        M0 = float(w.sum())
        c0 = (w[:, None] * dat.xipos[1:]).sum(axis=0) / M0
        p = 0.5 * (dat.site_xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "grasp_r")] + dat.site_xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "grasp_l")])
        M = M0 + load
        return M, (c0 * M0 + p * load) / M, p.copy()


def poses(d, box):
    """Right-arm joint vectors: 'straight' (both arms straight out at shoulder height, lift mid) and the candidate poses
    with both hands on the far face of the load box (x = box x max), found by the twin's bounded IK."""
    from .twin import TUCK, Reacher
    R = Reacher(d)
    lift_mid = Body(d).lift_mid
    out = {"straight": [np.array([lift_mid, math.radians(-90), 0, 0, 0, 0, 0])],
           "tuck": [np.array([lift_mid] + [TUCK[j] for j in ARM_J])], "box": [], "box_person": []}
    lo, hi = box
    for key, x, ztop in (("box", hi[0], hi[2]), ("box_person", min(hi[0], PERSON_X), min(hi[2], PERSON_Z))):
        for z in (max(lo[2], 0.20), 0.5 * (max(lo[2], 0.20) + ztop), ztop):
            for y in (0.15, 0.25):
                for ax in ([1.0, 0, 0], [0, 0, -1.0], [0.7071, 0, -0.7071]):
                    out[key] += R.solve([x, -y, z], ax, seeds=3)
    return out


def worst_pose(body, qs, load, k=0.0):
    """The tip-critical candidate for this load: the largest forward COM offset + k·h (k = a/g + caster tilt, so a high
    COM counts for the braking and caster-engagement shifts too)."""
    best, score = None, -9.0
    for q in qs:
        M, c, p = body.com(q, load)
        if c[0] + k * c[2] > score:
            best, score = (M, c, p, q), c[0] + k * c[2]
    return best


def tilt(d, cas):
    return cas["take_up"] / (cas["r"] - d.base.wheel_radius_pos * math.cos(math.radians(45))) if cas else 0.0


# ------------------------------------------------------------------------------------------------------------- scenarios
def today(rules, load, push):
    pose = "box" if load > rules.load_kg else "straight"
    return pose, rules.accel(load)


def requested(rules, load, push):
    """Rule SW-T1: a person within reach (the only way a push happens) → hands inside LOAD_WS with x ≤ PERSON_X, base ≤ A_PERSON."""
    pose, a = today(rules, load, push)
    if push:
        pose, a = "box_person", min(a, A_PERSON)
    return pose, a


def case(d, body, P, cas, load, az, brake, push, pose, a, collinear=False):
    M, c, p, q = worst_pose(body, P[pose], load, (a / G if brake else 0.0) + (0.0 if collinear else tilt(d, cas)))
    u = np.array([math.cos(math.radians(az)), math.sin(math.radians(az))])
    cxy = np.linalg.norm(c[:2]) * u                                   # the load COM pointed along az
    D = (c[2] * a / G if brake else 0.0) + (F_PUSH * H_PUSH / (M * G) if push else 0.0)
    if collinear:                                                     # the loop-40 reading: rigid casters, along az only
        from .twin import margin
        poly = octagon(d, cas["r"]) if cas else diamond(d)
        r, at = margin(poly, cxy + D * u), az
    else:
        r, at = reserve(d, cas, cxy, c[2], D)
    return dict(load=load, az=az, brake=brake, push=push, pose=pose, a=a if brake else 0.0, M=M, h=float(c[2]), com=float(np.linalg.norm(c[:2])),
                hand_x=float(p[0]), D=D, reserve=float(r), at=float(at))


def run_cases(d, body, P, cas, rule_fn, rules, a_fixed=None, pose_fixed=None, collinear=False):
    rows = []
    for load in LOADS:
        for az in AZS:
            for brake, push in DIST:
                pose, a = rule_fn(rules, load, push)
                rows.append(case(d, body, P, cas, load, az, brake, push, pose_fixed or pose, a_fixed if a_fixed is not None else a, collinear))
    return rows


def threshold_check(d, body, P, speed=None):
    """rnd/threshold.py's roller-level crossing (15 mm, 45°, 0.15 m/s, hard floor + carpet) at this design's mass
    (+ 2 kg in the hands, threshold.py's convention) and its arms-parked COM height."""
    from . import threshold as T
    M, c, _ = body.com(P["tuck"][0], 2.0)
    orig, sp = T.build, T.SPEED
    T.build = lambda *a, **k: orig(*a, **{**k, "mass": M, "com_h": float(c[2])})
    T.SPEED = speed or T.CROSS_SPEED
    try:
        peaks = [T.run(0.015, 45.0, carpet, t_max=0.5 + 1.4 / T.SPEED) for carpet in (False, True)]
    finally:
        T.build, T.SPEED = orig, sp
    pk = max(r["peak"] for r in peaks)
    from .params import act
    return dict(speed=T.CROSS_SPEED if speed is None else speed, mass=M, com_h=float(c[2]), crossed=all(r["crossed"] for r in peaks), peak=pk, margin=act(d.base.wheel_motor)["peak"] / pk)


def tip_force(d, body, P, cas):
    """Horizontal push 1.0 m up that tips the robot with the arms parked (worst heading)."""
    M, c, _ = body.com(P["tuck"][0], 0.0)
    r, _ = reserve(d, cas, c[:2], c[2], 0.0)
    lo, hi = 0.0, 200.0
    for _ in range(40):
        f = 0.5 * (lo + hi)
        rr, _ = reserve(d, cas, c[:2], c[2], f * H_PUSH / (M * G))
        lo, hi = (f, hi) if rr > 0 else (lo, f)
    return lo


def extras(d, body, P, rules, cas):
    """Modes outside the 16 A10 cases, with the requested rules."""
    tuck = P["tuck"]
    out = []

    def one(name, qs, load, a, push, cs, need=NEED):
        M, c, p, _ = worst_pose(body, qs, load, a / G + tilt(d, cs))
        D = c[2] * a / G + (F_PUSH * H_PUSH / (M * G) if push else 0.0)
        r, at = reserve(d, cs, c[:2], c[2], D)
        out.append(dict(name=name, M=M, reserve=float(r), at=float(at), ok=bool(r >= need)))
    one("No load, arms parked, transit (casters released, > 0.3 m/s allowed), braking 1.0 m/s², no person within reach", tuck, 0.0, 1.0, False, None)
    one("No load, arms parked, person within reach: ≤ 0.1 m/s → casters locked, braking 0.1 m/s² + 30 N push", tuck, 0.0, A_PERSON, True, cas)
    one("Threshold crossing (D2b, casters released) with 2.5 kg in the load box, braking 0.3 m/s², no person within reach", P["box"], 2.5, rules.accel(2.5), False, None)
    one("R2d basket 9.07 kg at the load box face, casters locked, braking 0.3 m/s², no person within reach", P["box"], 9.07, rules.accel(9.07), False, cas)
    one(f"R2d basket 9.07 kg, person within reach: held at x ≤ {PERSON_X:.2f} m, z ≤ {PERSON_Z:.2f} m, braking 0.1 m/s² + 30 N push", P["box_person"], 9.07, A_PERSON, True, cas)
    return out


SW_CHANGES = [
    ("SW-T1 person within reach", "`Supervisor.set_person_near(bool)`: when perception reports a person within reach (≈ 1 m), the supervisor caps the base at ≤ 0.1 m/s and ≤ 0.1 m/s² "
     f"(`base_accel_limit()` → A_PERSON) and rejects hand targets outside LOAD_WS whatever the payload, with x ≤ {PERSON_X:.2f} m and z ≤ {PERSON_Z:.2f} m (the arms come in and the lift goes down first). "
     "A push can only come from a person, so the 8 push cases are evaluated under this rule. With the R2d basket (> 2.5 kg) the base stops and the basket "
     "is set down before the person is within reach. sw/safety.py has no person input today."),
    ("SW-T2 caster lock (D18b)", "Casters stay locked (solenoids off, spring-applied, so an e-stop or power loss locks them) except in two planner modes: "
     "(a) a threshold or rug crossing (D2b: 45°, ≤ 0.15 m/s), started only with no person within reach; (b) unloaded transit with the arms parked above 0.3 m/s. "
     "Release pulse (≈ 100 ms) at every stop and every 0.3 m of locked travel so a lock never ratchets down into a floor dip and lifts a wheel. "
     "Needs a caster-lock output and a `Supervisor.casters_locked()` state the base planner checks before moving. "
     "rnd/tip.py counts SW-T1/T2 as enforced (scorecard gate) once `set_person_near` and `casters_locked` exist in sw/safety.py."),
    ("SW-T4 threshold speed (D2b)", f"Cross thresholds at ≤ {CROSS_SLOW:.2f} m/s (was 0.15): with the ballast the roller model's 15 mm peak rises to the "
     "1.45× margin at 0.15 m/s; 0.12 m/s restores ≥ 1.5× (table above). A planner value, not in sw/safety.py; threshold_mc should be re-run over 0.10–0.14 m/s."),
    ("SW-T3 keep", "Keep LOAD_KG 1.0 kg, LOAD_WS x ≤ 0.45 m / z ≤ 0.80 m and the 0.3 m/s² load-mode cap: the cases below rely on them (they are enforced today). "
     "`load_heading_ok()` (±20° of a wheel) is not called anywhere and is not needed with D18b (the 45° cases pass)."),
]


def fmt(r, w):
    v = r[w] * 1000
    return f"{v:.0f}{' ✅' if r[w] >= NEED - 1e-9 else ' ❌'}"


def main():
    rules = sw_rules()
    d0, d1 = design_before(), design_after()
    b0, b1 = Body(d0), Body(d1)
    P = poses(d1, rules.box)
    c18, c18b = casters(d0, "D18"), casters(d1, "D18b")
    loop40 = run_cases(d0, b0, P, c18, today, rules, a_fixed=A_LOOP40, pose_fixed="straight", collinear=True)
    before = run_cases(d0, b0, P, c18, today, rules)
    after_today = run_cases(d1, b1, P, c18b, requested if (rules.person and rules.casters) else today, rules)
    after = run_cases(d1, b1, P, c18b, requested, rules)
    no_ballast = run_cases(design_after(0.0), Body(design_after(0.0)), P, c18b, requested, rules)
    sens = []
    for bal in (0.0, 2.45, 4.9):
        dd = design_after(bal)
        bb = Body(dd)
        for tu in (0.0005, 0.001, 0.002):
            cs = dict(casters(dd, "D18b"), take_up=tu)
            rr = run_cases(dd, bb, P, cs, requested, rules)
            sens.append((bal, tu, min(r["reserve"] for r in rr), sum(r["reserve"] >= NEED for r in rr)))
    ex = extras(d1, b1, P, rules, c18b)
    th0, th1, th2 = threshold_check(d0, b0, P), threshold_check(d1, b1, P), threshold_check(d1, b1, P, CROSS_SLOW)
    f0, f1 = tip_force(d0, b0, P, None), tip_force(d1, b1, P, c18b)

    def summ(rows):
        return min(r["reserve"] for r in rows), sum(r["reserve"] >= NEED - 1e-9 for r in rows)
    s40, sb, st, sa, snb = map(summ, (loop40, before, after_today, after, no_ballast))
    mb = __import__("rnd.budget", fromlist=["mass_budget"]).mass_budget(d1)
    base_kg = sum(m for _, m in mb["base"])
    from .parts import base as base_cad
    BX, BY, BZ, cad_kg = base_cad.BAL_X, base_cad.BAL_Y, base_cad.BAL_Z, base_cad.ballast_kg()
    if abs(cad_kg - d1.base.ballast_mass) > 0.05 * d1.base.ballast_mass:
        print(f"WARNING: params ballast_mass {d1.base.ballast_mass} kg vs CAD {cad_kg:.2f} kg")
    bal = mb.get("ballast (2 removable blocks)", [])
    L = ["# Tip-over, A10 v2 (generated by `python -m rnd.tip`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Result: worst reserve {sa[0] * 1000:.0f} mm, {sa[1]}/16 cases ≥ 30 mm with the A10 v2 hardware and the requested supervisor rules "
         f"(SW-T1/T2 below); {st[1]}/16 (worst {st[0] * 1000:.0f} mm) with the rules sw/safety.py enforces today.** "
         f"Before: loop 40's reading {s40[0] * 1000:.0f} mm ({s40[1]}/16; the review quoted the older −102 mm); "
         f"the same hardware read honestly (D18 take-up, all headings, today's rules) {sb[0] * 1000:.0f} mm ({sb[1]}/16).", "",
         "## What changed (A10 v2)", "",
         f"1. **Mass low in the base.** The battery already sat on the belly (z ≈ 33–98 mm). The A5 v2 power stage (contactor, pre-charge, "
         f"clamp plate, bulk cap: {d1.base.power_board_mass:.2f} kg, ESTIMATE) was in no budget; it now sits on the belly in the front-right sector "
         f"(`base.power_board`). **Removable steel ballast: 2 × {bal[0][1] if bal else 0:.2f} kg** ({BX[1] - BX[0]:.0f} × {BY[1] - BY[0]:.0f} × {BZ[1] - BZ[0]:.0f} mm blocks, FROM CAD: {cad_kg:.2f} kg) in the two rear "
         f"sectors at z {BZ[0]:.0f}–{BZ[1]:.0f} mm, sitting in printed cradles, lifted out through two deck hatches once the upper core is off (no tools). R3: base module {base_kg:.1f} kg "
         f"(≤ 8 target, 10 piece limit), each ballast block {bal[0][1] if bal else 0:.2f} kg as its own carry piece (≤ 10); robot "
         f"{b0.com(P['tuck'][0], 0)[0]:.1f} → {b1.com(P['tuck'][0], 0)[0]:.1f} kg (twin). Moving mass *down* barely matters here (the braking term is "
         f"h·a/g: 0.3 m/s² × 0.05 m = 1.5 mm); what the ballast buys is **mass** (the push term F·H/(M g) falls "
         f"{F_PUSH * H_PUSH / (b0.com(P['tuck'][0], 0)[0] * G) * 1000:.0f} → {F_PUSH * H_PUSH / (b1.com(P['tuck'][0], 0)[0] * G) * 1000:.0f} mm) and, "
         f"sitting at the rear, a COM that starts {b0.com(P['tuck'][0], 0)[1][0] * 1000:.0f} → {b1.com(P['tuck'][0], 0)[1][0] * 1000:.0f} mm "
         "ahead of the centre with the arms parked, so a forward load has more room.",
         f"2. **Support polygon: not widened.** R5's 70 cm door is set by the arms, not the base: folded, the shoulders and forearms span "
         "0.70–0.78 m (twin CAD hulls, tuck poses; ≥ 0.65 m even at −15° roll), so the door rule is already the arm owner's problem (flagged). A wider base "
         "would not change the door fit but would push the base edge into the R1 floor targets (x ≥ 0.30 m from the centre) and the counter approach, and "
         "the wheels at r 0.21 m already reach the Ø50 cm shell (wheel faces at r 0.233). The 4 casters are what widens the polygon (diamond → octagon), "
         "and only once they carry load.",
         f"3. **Casters D18 → D18b.** D18's ball transfers rode on a 5 N spring with a hard stop after 5 mm: they carry nothing until the base has tilted "
         f"{math.degrees(0.005 / (0.22 - 0.148)):.1f}° about the wheel edge (COM ≈ {0.52 * 0.005 / (0.22 - 0.148) * 1000:.0f} mm outward first), and a "
         "5 mm stroke can't clear a 15 mm threshold, and at the D2b 45° heading a caster is the leading point (review finding 4). D18b v2 (D26 cost-down; "
         "W8d had a 2\" steel ball transfer, ≈ $45 and 0.54 kg of ball each): a printed skid foot on a replaceable Ø19 PTFE glide pad with a "
         f"{d1.base.caster_flank:.0f}° cone flank (PTFE tape) up to Ø60 at 22 mm, at r {d1.base.caster_r * 1000:.0f} mm on a printed guide with **{d1.base.caster_float * 1000:.0f} mm of float** on a "
         "5 N spring, and a **spring-applied one-way rod lock** (caulk-gun plate on a Ø8 steel rod) released by a 24 V pull solenoid. Locked, it can follow "
         "the floor down but not be pushed up, so it is a rigid support after the lock's take-up "
         f"({d1.base.caster_take_up * 1000:.1f} mm, ESTIMATE → {math.degrees(d1.base.caster_take_up / (d1.base.caster_r - 0.148)):.1f}°); released, it "
         f"floats over a 15 mm threshold: the edge meets the flank for any step of 2–22 mm, so the climb takes tan({d1.base.caster_flank:.0f}° + atan {d1.base.caster_mu}) = "
         f"{math.tan(math.radians(d1.base.caster_flank) + math.atan(d1.base.caster_mu)):.2f} × the 5–10 N spring force "
         f"({5 * math.tan(math.radians(d1.base.caster_flank) + math.atan(d1.base.caster_mu)):.0f}–{10 * math.tan(math.radians(d1.base.caster_flank) + math.atan(d1.base.caster_mu)):.0f} N; "
         "the 2\" ball needed 2.2 × (11–22 N), a 1½\" ball 4.6 ×). Locked casters carry only their 5 N preload at rest (20 N of ≈ 300 N off the wheels); "
         f"sliding, that is ≈ {4 * 5 * d1.base.caster_mu:.0f} N of drag on hard floors (PTFE μ ≈ {d1.base.caster_mu}, ESTIMATE; furniture-glide duty), against "
         "the rolling ball's ≈ 1 N. Tip statics are unchanged (a friction force at the contact that carries the tip load has no moment about it).",
         "4. **Planner rules last** (below): the 8 push cases need SW-T1, the casters need SW-T2, the ballast's R4 cost needs SW-T4; the rest is enforced today.", "",
         "| Load | Load COM | Braking | Push 30 N @ 1 m | Pose (rules) | Decel m/s² | COM offset mm | h m | ZMP shift mm | Loop 40 reading | Before, honest | A10 v2, SW today | **A10 v2 + SW-T1/T2** | worst heading |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r40, rb, rt, ra in zip(loop40, before, after_today, after):
        L.append(f"| {ra['load']:.1f} kg | {ra['az']:.0f}° | {'yes' if ra['brake'] else ''} | {'yes' if ra['push'] else ''} | {ra['pose']} | "
                 f"{ra['a']:.1f} | {ra['com'] * 1000:.0f} | {ra['h']:.2f} | {ra['D'] * 1000:.0f} | {fmt(r40, 'reserve')} | {fmt(rb, 'reserve')} | "
                 f"{fmt(rt, 'reserve')} | **{fmt(ra, 'reserve')}** | {ra['at']:.0f}° |")
    L += ["", f"Columns: *loop 40 reading* = the old tip.py (rigid D18, both arms straight out, 0.5 m/s², along the load direction only); "
          "*before, honest* = the same hardware with D18's 5 mm take-up, every heading and the supervisor's real limits (1 kg: arms straight out allowed, "
          "1.0 m/s²; 2.5 kg: load box, 0.3 m/s²); *A10 v2* = power board + ballast + locked D18b casters. "
          f"Without the ballast blocks (A10 v2 + rules): worst {snb[0] * 1000:.0f} mm, {snb[1]}/16.", "",
          f"**Push that tips the robot, arms parked, 1.0 m up, worst heading:** {f0:.0f} N before (wheels only) → {f1:.0f} N (A10 v2, casters locked).", "",
          "## Sensitivity (A10 v2 + rules): worst reserve / cases ≥ 30 mm", "", "| Ballast | lock take-up 0.5 mm | 1.0 mm | 2.0 mm |", "|---|---|---|---|"]
    for bal_kg in (0.0, 2.45, 4.9):
        L.append(f"| {bal_kg:.2f} kg | " + " | ".join(f"{w * 1000:.0f} mm, {n}/16" for b_, t_, w, n in sens if b_ == bal_kg) + " |")
    L += ["", "## Other modes (A10 v2, requested rules)", "", "| Mode | Mass kg | Reserve | |", "|---|---|---|---|"]
    L += [f"| {e['name']} | {e['M']:.1f} | {e['reserve'] * 1000:.0f} mm at {e['at']:.0f}° | {'✅' if e['ok'] else '❌ (rule: not allowed)'} |" for e in ex]
    L += ["", "## R4 cost of the mass (rnd/threshold.py's roller model re-run at each mass and COM height, 15 mm, 45°, hard floor + carpet)", "",
          "| Design | Speed | Mass (+ 2 kg) | COM h | Crossed | Peak wheel torque | Margin to GIM peak (≥ 1.5×) |", "|---|---|---|---|---|---|---|"]
    for n, t in (("before", th0), ("A10 v2", th1), ("A10 v2, SW-T4", th2)):
        L.append(f"| {n} | {t['speed']:.2f} m/s | {t['mass']:.1f} kg | {t['com_h']:.2f} m | {'✅' if t['crossed'] else '❌'} | {t['peak']:.1f} N·m | {t['margin']:.2f}× {'✅' if t['margin'] >= 1.5 else '❌'} |")
    L += ["", "## Supervisor rule changes for the software owner (sw/safety.py is read-only here)", ""]
    L += [f"- **{k}:** {v}" for k, v in SW_CHANGES]
    L += ["", "**Reading.** A 1.3 m robot on a Ø50 cm base can't take a firm push at shoulder height while it holds a load out in front "
          "on hardware alone: the push term alone (≈ 100–120 mm) uses most of the 148 mm between wheels. The hardware changes make every case pass "
          "that the robot can meet without a person next to it, and SW-T1 is what makes the push cases pass. The R2d basket "
          "(9 kg at 0.45 m) is outside A10: it passes with casters locked and nobody within reach, and is not allowed with a person within reach "
          "(SW-T1 then has to stop the base and set the basket down). **Open:** the lock's take-up and the ratchet-down behaviour on a real floor are "
          "ESTIMATES (bench: caster coupon on tile + low pile, measure take-up under 150 N); the quasi-static ZMP ignores the bump of a caster engaging "
          "(a MuJoCo push test with the caster model is the next check); the ball transfer on medium pile when released drags ≈ 0.5 × 10 N per caster."]
    (DOCS / "tip.md").write_text("\n".join(L) + "\n")
    js = dict(worst=sa[0], n_ok=sa[1], n=16, n_fail=16 - sa[1], worst_sw_today=st[0], n_ok_sw_today=st[1], worst_before=sb[0], n_ok_before=sb[1],
              worst_loop40=s40[0], n_ok_loop40=s40[1], worst_no_ballast=snb[0], n_ok_no_ballast=snb[1],
              threshold=dict(before=th0, after=th1, after_slow=th2), push_tip_n=dict(before=f0, after=f1),
              extras=ex, sw_changes=[k for k, _ in SW_CHANGES if "keep" not in k], rows=after)
    (DOCS / "tip.json").write_text(json.dumps(js, indent=1, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
