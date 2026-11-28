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
    B = Body(d)
    lift_mid, lift_top = B.lift_mid, float(B.m.jnt_range[B.mj.mj_name2id(B.m, B.mj.mjtObj.mjOBJ_JOINT, "lift")][1])
    out = {"straight": [np.array([lift_mid, math.radians(-90), 0, 0, 0, 0, 0])],
           "straight_high": [np.array([lift_top, math.radians(-90), 0, 0, 0, 0, 0])],   # A10 real states: the same pose at the lift top
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


# ------------------------------------------------------------------------------- A10 from the real states (Astra #3, F2)
V_WALK = 1.6              # m/s: walking approach speed (ISO 13855's K = 1600 mm/s, commonly cited; ESTIMATE)
R_REACH = 0.7             # m: a person this close (mast axis → their body) can put a hand on the shoulder (ESTIMATE)
T_PERCEPT = 0.2           # s: person detection + fusion latency (ESTIMATE; PeopleNearTrigger counts > 0.5 s stale as near)
V_LIFT = 0.20             # m/s: lift speed at rated rpm (lift.md)
W_SLEW = 1.5              # rad/s: the supervisor's default per-joint slew (Supervisor v_max)
T_SETDOWN = 5.0           # s: stop (0.3 m/s at 0.3 m/s²) + lower a 9 kg basket 0.8 m on the lift + release (ESTIMATE)
V_BASE = {"locked": 0.3, "transit": 0.8, "threshold": 0.12}   # m/s before the person appears (sw/safety.py caps)


def sw_more():
    from sw import safety as s
    return SimpleNamespace(approach=hasattr(s.Supervisor, "set_person_approach"), prep=getattr(s, "PN_PREP_RADIUS", None),
                           near=s.PN_RADIUS, basket_r=getattr(s, "BASKET_CLEAR_R", None), released_a=s.CASTER_RELEASED_A,
                           basket_clear=hasattr(s.Supervisor, "set_area_clear"))


def real_rows(d, body, P, cas, rules):
    """The 16 A10 cases from the state the robot is in when the person appears: today's pose for the load (arms straight
    out at shoulder height with ≤ LOAD_KG, at lift mid *or top*; the load box above), braking at the then-current limit,
    casters locked (the push arrives before any transition has finished: see transition())."""
    rows = []
    for load in LOADS:
        for az in AZS:
            for brake, push in DIST:
                pose, a = today(rules, load, push)
                cand = [case(d, body, P, cas, load, az, brake, push, pz, a) for pz in ((pose, "straight_high") if pose == "straight" else (pose,))]
                rows.append(min(cand, key=lambda r: r["reserve"]))
    return rows


def transition(d, body, P, rules):
    """Time from detection to the SW-T1 state per carry mode, and the detection radius that finishes it before a walking
    person is within reach: R = R_REACH + V_WALK·(T_PERCEPT + t)."""
    def t_arms(qs):
        out = 0.0
        for qa in qs:
            out = max(out, min(max(float(np.max(np.abs(qb[1:] - qa[1:]))) / W_SLEW, abs(float(qb[0] - qa[0])) / V_LIFT) for qb in P["box_person"]))
        return out
    m = sw_more()
    rows = []
    for name, qs, v0, a in (("≤ 1 kg, arms straight out (lift mid / top)", P["straight"] + P["straight_high"], V_BASE["locked"], rules.accel(1.0)),
                            ("2.5 kg in the load box", P["box"], V_BASE["locked"], rules.accel(2.5)),
                            ("unloaded transit, casters released", P["tuck"], V_BASE["transit"], m.released_a),
                            ("2.5 kg threshold crossing, casters released", P["box"], V_BASE["threshold"], min(rules.accel(2.5), m.released_a))):
        tb = max(v0 - A_PERSON, 0.0) / a
        ta = t_arms(qs)
        t = max(tb, ta)
        rows.append(dict(name=name, t_brake=tb, t_arms=ta, t=t, R=R_REACH + V_WALK * (T_PERCEPT + t)))
    rows.append(dict(name="R2d basket 9.07 kg: stop and set it down", t_brake=V_BASE["locked"] / rules.accel(9.07), t_arms=T_SETDOWN, t=T_SETDOWN,
                     R=R_REACH + V_WALK * (T_PERCEPT + T_SETDOWN)))
    t_avail = {r: max((r - R_REACH) / V_WALK - T_PERCEPT, 0.0) for r in (m.near, m.prep or m.near)}
    return rows, t_avail


def real_extras(d, body, P, rules, cas):
    """Modes outside the 16 cases, from the state the robot is in when the person appears, with the push."""
    m = sw_more()
    out = []

    def one(name, qs, load, a, cs):
        M, c, p, _ = worst_pose(body, qs, load, a / G + tilt(d, cs))
        D = c[2] * a / G + F_PUSH * H_PUSH / (M * G)
        r, at = reserve(d, cs, c[:2], c[2], D)
        out.append(dict(name=name, M=M, reserve=float(r), at=float(at), ok=bool(r >= NEED)))
    one("Unloaded transit, arms parked, casters still released, braking at 0.5 m/s² + push", P["tuck"], 0.0, m.released_a, None)
    one("2.5 kg threshold crossing (casters released), braking 0.3 m/s² + push", P["box"], 2.5, min(rules.accel(2.5), m.released_a), None)
    one("R2d basket 9.07 kg at the load box face, casters locked, braking 0.3 m/s² + push", P["box"], 9.07, rules.accel(9.07), cas)
    one("R2d basket 9.07 kg at the load box face, base stopped + push", P["box"], 9.07, 0.0, cas)
    one("R2d basket 9.07 kg pulled into PERSON_WS, base stopped + push", P["box_person"], 9.07, 0.0, cas)
    return out


def tip_decel(d, body, qs, load, cs, need=0.0):
    """Base deceleration (m/s²) at which the reserve falls to `need` (no push), e.g. an e-stop that brakes harder than the planner."""
    lo, hi = 0.0, 12.0
    for _ in range(30):
        a = 0.5 * (lo + hi)
        M, c, p, _ = worst_pose(body, qs, load, a / G + tilt(d, cs))
        r, _ = reserve(d, cs, c[:2], c[2], c[2] * a / G)
        lo, hi = (a, hi) if r > need else (lo, a)
    return lo


def estop_rows(d, body, P, rules, cas):
    rows = []
    for name, qs, load, cs in (("≤ 1 kg, arms straight out, lift top, casters locked", P["straight_high"], 1.0, cas),
                               ("2.5 kg in the load box, casters locked", P["box"], 2.5, cas),
                               ("R2d basket 9.07 kg at the load box face, casters locked", P["box"], 9.07, cas),
                               ("unloaded, arms parked, casters released (transit)", P["tuck"], 0.0, None),
                               ("2.5 kg in PERSON_WS, casters locked (SW-T1 state)", P["box_person"], 2.5, cas)):
        rows.append(dict(name=name, a0=tip_decel(d, body, qs, load, cs), a30=tip_decel(d, body, qs, load, cs, NEED)))
    return rows


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
    ("SW-T6 person approaching (A10 real states)", f"`Supervisor.set_person_approach(bool)`, a second PeopleNearTrigger at PN_PREP_RADIUS: "
     "the SW-T1 box, base caps, caster lock and basket stop from there, so the transition (≤ 1.5 s) is done before a walking person is within reach "
     "(table above). Implemented in sw/safety.py; rnd/tip.py credits the push cases with the SW-T1 state only while it exists and its radius ≥ the "
     "radius the transition needs."),
    ("SW-T7 basket carry", "`Supervisor.set_area_clear(bool)`: the base moves with more than PERSON_MAX_KG only while perception sees nobody within "
     "BASKET_CLEAR_R (set-down before a walking person can arrive). Implemented; fail-safe default False."),
    ("SW-T3 keep", "Keep LOAD_KG 1.0 kg, LOAD_WS x ≤ 0.45 m / z ≤ 0.80 m and the 0.3 m/s² load-mode cap: the cases below rely on them (they are enforced today). "
     "`load_heading_ok()` (±20° of a wheel) is not called anywhere and is not needed with D18b (the 45° cases pass)."),
]


# ------------------------------------------------------------------- D29 #9: one caster solenoid pulling 4 cables (cost_down.md)
# params PIP_CASTER_RELEASE ("four" today: a JF-1039 per caster on its lock plate; "cable": one central solenoid + 4 Ø1 steel
# cables in PTFE liners). Force budget and timing, all ESTIMATE until the bench pull-force test (cost_down.md checks).
F_LOCK = (4.0, 3.0, 5.0)          # N: lock-plate spring at the release point (caulk-gun plate; nominal, lo, hi)
PLATE_STROKE = 0.003              # m: plate tilt travel to free the rod
MU_LINER = (0.15, 0.10, 0.25)     # steel cable in a PTFE liner (capstan μ)
WRAP = math.pi + math.pi / 6      # rad per cable: a 90° turn at the solenoid, a 90° turn down to the plate, + 30° of routing
F_RETURN = 2.0                    # N: plunger return spring (keeps the cables taut, the plunger out when de-energised)
SOLENOIDS = {"JF-1039 (24 V, 25 N, $12.99 Walmart; the per-caster part)": 25.0,
             "JF-1264B (24 V, 55 N, 10 mm, $26.62 eBay listing 2026-11-28)": 55.0}
T_COIL_ON, T_COIL_OFF = 0.020, 0.015   # s: coil current rise to pull-in / decay to drop-out with the flyback diode (JF class, ESTIMATE)
M_MOVING = dict(four=0.020 + 0.005, cable=0.030 + 4 * (0.005 + 0.002))   # kg: plunger (+ equaliser) + plates + cables
MARGIN = 1.5                      # force margin wanted at nominal; ≥ 1.0 at the worst corner


def cable_release():
    """Force budget and timing for both release layouts: dict per layout and solenoid."""
    from sw import safety as s
    pulse = s.CASTER_PULSE_S
    out = []

    def need(n, mu, f):
        return n * f * math.exp(mu * WRAP) + F_RETURN if n > 1 else f + F_RETURN

    for lay, n, sols in (("four", 1, [list(SOLENOIDS)[0]]), ("cable", 4, list(SOLENOIDS))):
        for sname in sols:
            F = SOLENOIDS[sname]
            nom, worst = need(n, MU_LINER[0], F_LOCK[0]), need(n, MU_LINER[2], F_LOCK[2])
            m = M_MOVING[lay]
            t_rel = T_COIL_ON + math.sqrt(2 * PLATE_STROKE * m / max(F - worst, 1e-3)) if F > worst else math.inf
            f_back = (n * F_LOCK[1] * math.exp(-MU_LINER[2] * WRAP) if n > 1 else F_LOCK[1]) + F_RETURN      # weakest spring, worst liner
            t_lock = T_COIL_OFF + math.sqrt(2 * PLATE_STROKE * m / f_back)
            ok_f = F >= MARGIN * nom and F >= worst
            ok_t = t_rel <= 0.5 * pulse and t_lock <= 0.5 * pulse
            out.append(dict(layout=lay, solenoid=sname, F=F, need_nom=nom, need_worst=worst, margin_nom=F / nom, margin_worst=F / worst,
                            t_release=t_rel, t_lock=t_lock, pulse=pulse, ok_force=ok_f, ok_timing=ok_t, ok=ok_f and ok_t))
    return out


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
    real = real_rows(d1, b1, P, c18b, rules)
    trans, t_avail = transition(d1, b1, P, rules)
    rex = real_extras(d1, b1, P, rules, c18b)
    est = estop_rows(d1, b1, P, rules, c18b)
    more = sw_more()
    r_need = max(t["R"] for t in trans if "basket" not in t["name"])
    r_basket = next(t["R"] for t in trans if "basket" in t["name"])
    approach_ok = bool(more.approach and more.prep and more.prep >= r_need - 1e-9)
    basket_ok = bool(more.basket_clear and more.basket_r and more.basket_r >= r_basket - 1e-9)
    th0, th1, th2 = threshold_check(d0, b0, P), threshold_check(d1, b1, P), threshold_check(d1, b1, P, CROSS_SLOW)
    f0, f1 = tip_force(d0, b0, P, None), tip_force(d1, b1, P, c18b)

    def summ(rows):
        return min(r["reserve"] for r in rows), sum(r["reserve"] >= NEED - 1e-9 for r in rows)
    s40, sb, st, sa, snb, sr = map(summ, (loop40, before, after_today, after, no_ballast, real))
    sra = sa if approach_ok else sr
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
         f"**A10 from the real states (Astra review #3, finding 2): conditional.** The 16/16 above credits the push cases with the SW-T1 state "
         f"(arms in, lift down, {A_PERSON} m/s²). A person needn't wait for it: detected at {more.near} m and walking at {V_WALK} m/s they can push "
         f"{t_avail[more.near]:.2f} s later, the transition takes up to {max(t['t'] for t in trans if 'basket' not in t['name']):.1f} s. From the states the "
         f"robot is actually in then (arms out, lift mid or top, casters locked, braking at the then-current limit): **{sr[1]}/16, worst "
         f"{sr[0] * 1000:.0f} mm**. With SW-T6 (person approaching at {more.prep} m ≥ the {r_need:.1f} m the transition needs"
         f"{', in sw/safety.py' if approach_ok else ', NOT in sw/safety.py'}) the push meets the SW-T1 state: **{sra[1]}/16, worst {sra[0] * 1000:.0f} mm**. "
         f"The basket needs nobody within {r_basket:.1f} m (SW-T7, BASKET_CLEAR_R {more.basket_r} m{'' if basket_ok else ': NOT enforced'}). "
         "Still open: the lock take-up and pad sinkage (sensitivity below), an e-stop that brakes harder than the planner (table below), "
         "one failed caster lock, and the ballast interlock (A10 counts the blocks as fitted).", "",
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
    L += ["", "## A10 from the states the robot is in when a person appears", "",
          f"Timeline: the person is detected at {more.near} m (PeopleNearTrigger) and can push once within {R_REACH} m (a hand on the shoulder, "
          f"ESTIMATE), walking {V_WALK} m/s (ISO 13855's 1.6 m/s, commonly cited) after {T_PERCEPT} s of perception latency: "
          f"**{t_avail[more.near]:.2f} s** (0 s if they step out from behind furniture). The transition (arms into PERSON_WS at the "
          f"{W_SLEW} rad/s slew / lift {V_LIFT} m/s, base braked to {A_PERSON} m/s at the limit it was moving under) takes:", "",
          "| Mode when the person appears | Braking (s) | Arms / lift (s) | Transition (s) | Detection radius it needs (m) |", "|---|---|---|---|---|"]
    L += [f"| {t['name']} | {t['t_brake']:.2f} | {t['t_arms']:.2f} | {t['t']:.2f} | {t['R']:.1f} |" for t in trans]
    L += ["", "So the push meets the state the robot was in. The 16 cases from that state (the pose and braking today's rules allow, lift mid "
          "or top, whichever is worse; casters locked):", "",
          "| Load | Load COM | Braking | Push | Pose | Decel m/s² | **From the real state** | Credited (SW-T1 state) | worst heading |", "|---|---|---|---|---|---|---|---|---|"]
    for rr, ra in zip(real, after):
        L.append(f"| {rr['load']:.1f} kg | {rr['az']:.0f}° | {'yes' if rr['brake'] else ''} | {'yes' if rr['push'] else ''} | {rr['pose']} | {rr['a']:.1f} | "
                 f"**{fmt(rr, 'reserve')}** | {fmt(ra, 'reserve')} | {rr['at']:.0f}° |")
    L += ["", "| Other mode, from the real state, + 30 N push | Mass kg | Reserve | |", "|---|---|---|---|"]
    L += [f"| {e['name']} | {e['M']:.1f} | {e['reserve'] * 1000:.0f} mm at {e['at']:.0f}° | {'✅' if e['ok'] else '❌'} |" for e in rex]
    L += ["", "**E-stop / snag braking.** A stop need not respect the planner's limit. Base deceleration at which the reserve falls to 30 mm / "
          "tips (no push):", "", "| State | 30 mm at (m/s²) | Tips at (m/s²) |", "|---|---|---|"]
    L += [f"| {e['name']} | {e['a30']:.2f} | {e['a0']:.2f} |" for e in est]
    L += ["", "Rule changes (sw/safety.py, tested in sw/test_s1.py): **SW-T6** person approaching at PN_PREP_RADIUS "
          f"({more.prep} m ≥ {r_need:.1f} m): the SW-T1 part (PERSON_WS, base caps, casters locked, basket stop) starts there, the D25 roll "
          f"bound / roll latch / hand-speed cap stay at {more.near} m; **SW-T7** the base moves with > 2.5 kg only with perception reporting "
          f"nobody within BASKET_CLEAR_R ({more.basket_r} m ≥ {r_basket:.1f} m; fail-safe default: not clear); **slower while loaded**: the "
          "people-near hand-speed cap falls with payload (rnd/hand_speed.py). Not in software: the base e-stop has to be a controlled stop "
          f"(stop category 1: ramp at ≤ the limit in force, then remove power) with the wheel drives not shorted, since a 2.5 kg load-box "
          f"carry tips at ≈ {est[1]['a0']:.1f} m/s² and the wheels can brake harder on a dry floor (μ·g ≈ 5 m/s², ESTIMATE): electrical / "
          "base-drive track."]
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
    js = dict(worst=sa[0], n_ok=sa[1], n=16, n_fail=16 - sa[1], worst_sw_today=st[0], n_ok_sw_today=st[1], worst_before=sb[0], n_ok_before=sb[1],
              worst_loop40=s40[0], n_ok_loop40=s40[1], worst_no_ballast=snb[0], n_ok_no_ballast=snb[1],
              threshold=dict(before=th0, after=th1, after_slow=th2), push_tip_n=dict(before=f0, after=f1),
              extras=ex, sw_changes=[k for k, _ in SW_CHANGES if "keep" not in k], rows=after,
              real=dict(worst=sr[0], n_ok=sr[1], worst_with_rules=sra[0], n_ok_with_rules=sra[1], approach_enforced=approach_ok,
                        basket_clear_enforced=basket_ok, r_need=r_need, r_basket=r_basket, t_avail=t_avail, transition=trans, rows=real,
                        extras=rex, estop=est))
    # D29 #9: A10 on both caster-release layouts (the lock geometry does not move; only the base mass does) + force / timing
    from .params import CASTER_RELEASES
    var = {}
    for lay, v in CASTER_RELEASES.items():
        dd = dataclasses.replace(d1, base=dataclasses.replace(d1.base, caster_mass=v["caster_mass"], caster_release=lay))
        bb, cs = Body(dd), casters(dd, "D18b")
        ra, rl = run_cases(dd, bb, P, cs, requested, rules), real_rows(dd, bb, P, cs, rules)
        var[lay] = dict(caster_mass=v["caster_mass"], worst=summ(ra)[0], n_ok=summ(ra)[1], real_worst=summ(rl)[0], real_n_ok=summ(rl)[1])
    cr = cable_release()
    best = max((c for c in cr if c["layout"] == "cable"), key=lambda c: (c["ok"], c["F"]))
    a10_ok = var["cable"]["n_ok"] >= var["four"]["n_ok"] and var["cable"]["worst"] >= NEED - 1e-9 and var["cable"]["real_n_ok"] >= var["four"]["real_n_ok"]
    L += ["", f"## D29 #9: one caster solenoid pulling 4 cables (PIP_CASTER_RELEASE; this page uses **{d1.base.caster_release}**)", "",
          f"Lock plates, rods, feet and their positions are unchanged; the base loses {4 * (var['four']['caster_mass'] - var['cable']['caster_mass']):.2f} kg "
          "(4 solenoids → 1 larger one + cables, ESTIMATE). Force budget: the solenoid must pull n × F_lock × e^(μθ) + the plunger return spring "
          f"(F_lock {F_LOCK[0]:g} N, {F_LOCK[1]:g}–{F_LOCK[2]:g}; PTFE liner μ {MU_LINER[0]:g}, {MU_LINER[1]:g}–{MU_LINER[2]:g}; wrap {math.degrees(WRAP):.0f}° per cable; "
          f"return {F_RETURN:g} N), wanted ≥ {MARGIN:g}× at nominal and ≥ 1.0× at the worst corner. Timing: release (coil rise + plate travel) and re-lock "
          f"(drop-out + the springs dragging the cables back, weakest spring, worst liner), each ≤ half the {cr[0]['pulse'] * 1000:.0f} ms re-seat pulse "
          "(sw/safety.py CASTER_PULSE_S; the person-near lock is then ≪ the 1.5 s transition).", "",
          "| Layout | Solenoid | Need: nominal / worst (N) | Margin nominal / worst | Release / re-lock (ms) | |", "|---|---|---|---|---|---|"]
    ms = lambda t: "—" if math.isinf(t) else f"{t * 1000:.0f}"
    L += [f"| {c['layout']} | {c['solenoid']} | {c['need_nom']:.1f} / {c['need_worst']:.1f} | {c['margin_nom']:.2f}× / {c['margin_worst']:.2f}× | "
          f"{ms(c['t_release'])} / {ms(c['t_lock'])} | {'✅' if c['ok'] else '❌'} |" for c in cr]
    L += ["", "| Layout | A10 (requested rules) worst / cases ≥ 30 mm | A10 from the real states (today's rules) worst / cases |", "|---|---|---|"]
    L += [f"| {k} (caster {v['caster_mass']:.3f} kg each) | {v['worst'] * 1000:.0f} mm / {v['n_ok']}/16 | {v['real_worst'] * 1000:.0f} mm / {v['real_n_ok']}/16 |" for k, v in var.items()]
    L += ["", f"**Re-check: A10 {'PASS' if a10_ok else 'FAIL'}; force + timing {'PASS' if best['ok'] else 'FAIL'} with the {best['solenoid'].split(' (')[0]}** "
          f"(the per-caster JF-1039 cannot pull four cables: {next(c for c in cr if c['layout'] == 'cable' and c['F'] == 25.0)['margin_nom']:.2f}× at nominal). "
          "Not modelled: a mechanically stuck plunger or a kinked liner now holds **all four** casters released (one solenoid per caster leaves three "
          "locking), which the A10 rows with locked casters do not cover; a slack cable on a rigid equaliser leaves its caster locked on a threshold "
          "crossing (safe side for tip, a jam for R4). Bench: pull force through the four liners at the lock plates, and the release / re-lock times on a scope."]
    js["d29_caster_release"] = dict(a10_ok=a10_ok, variants=var, release=cr, best=best["solenoid"], force_timing_ok=best["ok"])
    (DOCS / "tip.md").write_text("\n".join(L) + "\n")
    (DOCS / "tip.json").write_text(json.dumps(js, indent=1, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
