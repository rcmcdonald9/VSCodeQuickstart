"""R17 safe power-off (Astra review #3, critical finding 1).  python -m rnd.power_off [--quick]
→ docs/rnd/power_off.md + power_off.json

Question: on a loss of torque (e-stop, a latched fault, power loss: the drives are disabled, the contactor is open) the D25
people-near soft limit (shoulder roll ≥ +5°, sw/safety.py) is gone. Where does each arm joint go, can the inboard
shoulder-roll wedge (yaw band / block against the arm-side ferrule: ≈ 2.5 mm at −15°, ≥ 25 mm from ≈ +1°, rnd/pinch.py)
close, and what is the cheapest mechanical guarantee?

Method. The twin (rnd/twin.py, both arms, CAD masses) with the drives off: each joint gets its back-drive torque as
Coulomb friction (MuJoCo frictionloss) and its reflected rotor inertia as armature (rnd/inline_study.py joint_drive:
N·τ_drag/η_b + gearbox breakaway, ESTIMATE, ×0.5…×2 swept), no damping (phases open: the drive's MCU is unpowered,
ESTIMATE), the shoulder balancer τ = K·sin(pitch) (K = params arm.spring_k_nm, rnd/balancer.py), the lift held by its
spring-applied pawl (lift.md option C), the joint limits at the hard stops (roll −16.5 / +104°, rnd/parts/roll.py
STOP_AT; the others at the soft limit + 3°, ESTIMATE). Start poses are sampled in the states the supervisor allows:
**people near** (roll ≥ +8°: the +5° bound + the 3° margin; both hands inside PERSON_WS, intersected with LOAD_WS above
1 kg) and **nobody near** (roll in the −12…+5° band the soft limit removes, hands in the free workspace / LOAD_WS), at
rest, with 0 / 1 kg (one hand) / 2.5 kg / the 9.07 kg R2d basket (two hands, half each). Each start is let go for
T_SIM s. Contacts are off (the links may pass through each other and the base: joint limits only), so a fall that would
end on the base is reported as passing through it (ESTIMATE: a fall that lands on the base stops earlier). Gripper:
not modelled (an unpowered STS3215 gripper's hold on the payload is not demonstrated: R17 open item).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
ARM_J = ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")
MIRROR = {"pitch": 1, "roll": 1, "yaw": -1, "elbow": 1, "wpitch": 1, "wroll": -1}   # left arm = mirror of the right (rnd/tip.py)
FULL = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
T_WROLL = 0.5             # N·m back-drive of the STS3215 wrist roll (≈ 345:1 hobby servo, unpowered; ESTIMATE)
HARD_STOP_EXTRA = 3.0     # deg past the software limit where the other joints' hard stops are (ESTIMATE)
T_SIM, DT = 2.5, 5e-4
SPREAD = ((1.0, 1.0), (0.5, 1.0), (2.0, 1.0), (1.0, 0.0))   # (back-drive torque ×, viscous ×): inline_study.SPREAD, and no viscous drag
# Inboard wedge gap (mm) vs roll (deg): rnd/pinch.py wave 7, yaw band × arm-side ferrule, per-pose worst reachable cell
# (twin.JR_PEOPLE_NEAR comment); −16.5 is the hard-stop contact (yaw band 2.5 mm into the ferrule at −19°: contact ≈ −17°).
WEDGE = ((-16.5, 1.0), (-15.0, 2.6), (-5.0, 16.5), (0.0, 23.4), (1.0, 24.7), (2.0, 26.1), (5.0, 30.2), (7.5, 40.0))
WEDGE_OK = 25.0           # R17 finger-crush gap
LATCH_DEG = 5.0           # latch / fixed-stop contact angle (= JR_PEOPLE_NEAR roll lower bound)
LEVER_WEDGE = 0.070       # m: the wedge's radius about the roll axis (the yaw band / ferrule cell, r ≈ 60–80 mm; ESTIMATE)
R_STOP = 0.042            # m: stop radius (roll.STOP_R)
K_STOP_LIN = 2.0e6        # N/m: contact stiffness at a stop (steel pin on the stainless wear strip bonded to the printed rear arm; ESTIMATE)


def wedge(roll_deg):
    x, y = zip(*WEDGE)
    return float(np.interp(roll_deg, x, y))


def roll_stops():
    try:
        from .parts.roll import STOP_AT
        return tuple(float(a) for a in STOP_AT)
    except Exception:                                  # CAD unavailable: the values roll.py derives
        return (-16.5, 104.0)


def drive_rows(d):
    from .inline_study import joint_drive
    from .twin import ACTUATOR_DATASHEET
    out = {}
    for k, full in FULL.items():
        g = joint_drive(d, full)
        out[k] = dict(act=g["act"], N=g["N"], T=g["T"], J=g["J"], c=ACTUATOR_DATASHEET[k]["viscous_nms"])
    out["wroll"] = dict(act=d.arm.joints["wrist_roll"][0], N=None, T=T_WROLL, J=0.0, c=ACTUATOR_DATASHEET["wroll"]["viscous_nms"])
    return out


class Fall:
    """The twin with the drives off. guard: None (today), "latch" / "fixed" (roll lower stop at LATCH_DEG)."""

    def __init__(self, d, guard=None, spread=(1.0, 1.0)):
        import mujoco
        from .twin import JR, build_xml
        self.mj, self.d = mujoco, d
        m = mujoco.MjModel.from_xml_string(build_xml(d))
        m.geom_contype[:] = 0
        m.geom_conaffinity[:] = 0
        m.opt.timestep = DT
        self.m, self.dat = m, mujoco.MjData(m)
        self.jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qa = {n: m.jnt_qposadr[self.jid(n)] for n in ["lift"] + [f"{j}_{s}" for j in ARM_J for s in "rl"]}
        self.da = {n: m.jnt_dofadr[self.jid(n)] for n in self.qa}
        self.rows = drive_rows(d)
        lo_r, hi_r = roll_stops()
        if guard in ("latch", "fixed"):
            lo_r = LATCH_DEG
        for s in "rl":
            for j in ARM_J:
                n = f"{j}_{s}"
                r = self.rows[j]
                m.dof_frictionloss[self.da[n]] = spread[0] * r["T"]
                m.dof_armature[self.da[n]] = r["J"]
                m.dof_damping[self.da[n]] = spread[1] * r["c"]
                lo, hi = JR[j]
                lo, hi = (lo_r, hi_r) if j == "roll" else (lo - HARD_STOP_EXTRA, hi + HARD_STOP_EXTRA)
                m.jnt_range[self.jid(n)] = np.radians([lo, hi])
                m.jnt_limited[self.jid(n)] = 1
                m.jnt_solref[self.jid(n)] = (4 * DT, 1.0)       # stiff stops (≈ 1° penetration at the worst fall)
        m.dof_frictionloss[self.da["lift"]] = 1e5        # pawl (lift.md C): held
        m.dof_armature[self.da["lift"]] = 1e3
        self.K = d.arm.spring_k_nm
        self.site = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"grasp_{s}") for s in "rl"}
        self.pay = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"payload_{s}") for s in "rl"}
        lid = self.jid("lift")
        self.lift_rng = tuple(m.jnt_range[lid])

    def set(self, q, loads):
        """q: {lift, pitch, roll, … (right arm, rad)}; loads: (right kg, left kg)."""
        dat = self.dat
        dat.qpos[:] = 0
        dat.qvel[:] = 0
        dat.qpos[self.qa["lift"]] = q["lift"]
        for j in ARM_J:
            dat.qpos[self.qa[f"{j}_r"]] = q[j]
            dat.qpos[self.qa[f"{j}_l"]] = MIRROR[j] * q[j]
        for s, kg in zip("rl", loads):
            self.m.body_mass[self.pay[s]] = max(kg, 1e-4)
        self.mj.mj_forward(self.m, dat)

    def hands(self):
        return {s: self.dat.site_xpos[self.site[s]].copy() for s in "rl"}

    def run(self, t=T_SIM):
        mj, m, dat = self.mj, self.m, self.dat
        n = int(t / DT)
        roll_min = {s: math.degrees(dat.qpos[self.qa[f"roll_{s}"]]) for s in "rl"}
        v_hand, v_roll_stop, prev = 0.0, 0.0, self.hands()
        lo_roll = math.degrees(m.jnt_range[self.jid("roll_r")][0])
        for _ in range(n):
            for s in "rl":
                p = dat.qpos[self.qa[f"pitch_{s}"]]
                dat.qfrc_applied[self.da[f"pitch_{s}"]] = self.K * math.sin(p)     # balancer (rnd/balancer.py law)
            mj.mj_step(m, dat)
            h = self.hands()
            for s in "rl":
                v_hand = max(v_hand, float(np.linalg.norm(h[s] - prev[s])) / DT)
                r = math.degrees(dat.qpos[self.qa[f"roll_{s}"]])
                if r < lo_roll + 0.3:
                    v_roll_stop = max(v_roll_stop, abs(float(dat.qvel[self.da[f"roll_{s}"]])))
                roll_min[s] = min(roll_min[s], r)
            prev = h
        end = {j: math.degrees(dat.qpos[self.qa[f"{j}_r"]]) for j in ARM_J}
        end_l = {j: MIRROR[j] * math.degrees(dat.qpos[self.qa[f"{j}_l"]]) for j in ARM_J}
        moving = float(np.max(np.abs(dat.qvel[[self.da[f"{j}_{s}"] for j in ARM_J for s in "rl"]])))
        return dict(end=end, end_l=end_l, roll_min=min(roll_min.values()), v_hand=v_hand, v_roll_stop=v_roll_stop, moving=moving)

    def gravity_roll(self, q, loads):
        """Roll gravity torque at rest (N·m, + = outward) per side at q, and the hand lever |∂p/∂roll| (m)."""
        self.set(q, loads)
        out = {}
        for s in "rl":
            jac = np.zeros((3, self.m.nv))
            self.mj.mj_jacSite(self.m, self.dat, jac, None, self.site[s])
            out[s] = (-float(self.dat.qfrc_bias[self.da[f"roll_{s}"]]), float(np.linalg.norm(jac[:, self.da[f"roll_{s}"]])))
        return out


def sample_starts(fall, n, mode, load, rng):
    """Rest poses the supervisor allows: mode "near" (roll ≥ +8°, hands in PERSON_WS ∩ load box) or "band" (roll −12…+5°,
    hands in the free workspace ∩ load box)."""
    from sw import safety as S
    from .twin import JR
    lo_f, hi_f = np.array((0.10, -0.45, 0.20)), np.array((0.80, 0.45, 1.35))       # free workspace, base frame (S.Supervisor defaults are the bench FK frame)
    boxes = [(lo_f, hi_f)]
    if load > S.LOAD_KG:
        boxes.append(tuple(map(np.array, S.LOAD_WS)))
    if mode == "near":
        boxes.append(tuple(map(np.array, S.PERSON_WS)))
    lo = np.max([b[0] for b in boxes], axis=0)
    hi = np.min([b[1] for b in boxes], axis=0)
    rr = (8.0, 100.0) if mode == "near" else (-12.0, 5.0)
    loads = (load, 0.0) if load <= S.LOAD_KG else (load / 2, load / 2)
    out, tries = [], 0
    while len(out) < n and tries < 200 * n:
        tries += 1
        q = {"lift": rng.uniform(*fall.lift_rng)}
        for j in ARM_J:
            a, b = (rr if j == "roll" else JR[j])
            if j == "pitch":
                a, b = -150.0, 30.0                          # the working pitch range (over-head poses are not carrying poses)
            q[j] = math.radians(rng.uniform(a, b))
        fall.set(q, loads)
        h = fall.hands()
        if all(np.all(h[s] >= lo) and np.all(h[s] <= hi) for s in "rl"):
            out.append(q)
    return out, loads


LOAD_SET = (0.0, 1.0, 2.5, 9.07)


def study(d, n=24, quick=False, seed=5):
    rng = np.random.default_rng(seed)
    base = Fall(d)
    res = dict(drive=drive_rows(d), K=d.arm.spring_k_nm, cases=[])
    for mode in ("near", "band"):
        for load in LOAD_SET:
            starts, loads = sample_starts(base, n, mode, load, rng)
            for guard in (None, "latch"):
                if guard and mode == "band":
                    continue                                # a latch only acts from ≥ +5° (the band is its residual)
                for sp in (SPREAD[:1] if quick or guard else SPREAD):
                    f = Fall(d, guard, sp)
                    rows = []
                    for q in starts:
                        f.set(q, loads)
                        g = f.gravity_roll(q, loads)
                        f.set(q, loads)
                        r = f.run()
                        r["start_roll"] = math.degrees(q["roll"])
                        r["g_roll"] = min(v[0] for v in g.values())
                        rows.append(r)
                    if not rows:
                        continue
                    rmin = [r["roll_min"] for r in rows]
                    res["cases"].append(dict(mode=mode, load=load, guard=guard or "none", spread=list(sp), n=len(rows),
                                             roll_min=min(rmin), wedge_min=wedge(min(rmin)),
                                             frac_closed=float(np.mean([wedge(x) < WEDGE_OK for x in rmin])),
                                             frac_stop=float(np.mean([x < roll_stops()[0] + 0.5 for x in rmin])),
                                             v_hand=max(r["v_hand"] for r in rows), v_roll_stop=max(r["v_roll_stop"] for r in rows),
                                             end={j: (float(np.min([r["end"][j] for r in rows])), float(np.median([r["end"][j] for r in rows])),
                                                      float(np.max([r["end"][j] for r in rows]))) for j in ARM_J},
                                             still_moving=float(np.mean([r["moving"] > 0.05 for r in rows])),
                                             g_roll_in=float(-min(min(r["g_roll"] for r in rows), 0.0))))
    res["push"] = push_force(base, rng, n)
    res["latch"] = latch_loads(d, res, base)
    return res


def push_force(fall, rng, n):
    """Hand force (N) that back-drives the unpowered roll inward from the hanging rest pose (roll at the friction band's
    edges and at 0): F = (T_backdrive − gravity torque helping) / |∂p_hand/∂roll|; and the force that then reaches a
    finger in the wedge (N) if that person (or another) keeps pushing with 20 N at the hand."""
    T = fall.rows["roll"]["T"]
    out = []
    for load in (0.0, 1.0, 2.5):
        loads = (load, 0.0) if load <= 1.0 else (load / 2, load / 2)
        for rdeg in (5.0, 0.0, -5.0):
            q = dict(lift=fall.lift_rng[1], pitch=0.0, roll=math.radians(rdeg), yaw=0.0, elbow=0.0, wpitch=0.0, wroll=0.0)
            g = fall.gravity_roll(q, loads)["r"]
            for sp in (0.5, 1.0, 2.0):
                F = max(sp * T + g[0], 0.0) / g[1]            # inward push: friction + the outward gravity torque to beat
                out.append(dict(load=load, roll=rdeg, spread=sp, F=F, lever=g[1], g=g[0], finger_20N=20.0 * g[1] / LEVER_WEDGE))
    return out


K_STOP_PU = 0.2e6         # N/m: the same stop with a 3 mm PU 90A cap on the pin (ESTIMATE)
F_GUIDE = 400.0           # N: what the latch guide carries with print behind its sleeve (≈ 12 MPa on 11.5 × 3 mm; ESTIMATE)


def latch_loads(d, res, fall=None):
    """Stop sizing for a latch / fixed stop at +5°, per load: the unpowered fall onto it (peak roll speed at contact from
    the runs; J = the twin's roll inertia hanging, rotor included; peak torque ω·√(J·k_θ) + the inward gravity torque,
    k_θ = k·r², steel and PU-capped; overshoot past the contact ½Jω² = ½k_θδ²), and a powered fault driving into it
    (rated, the drive's current limit, and the GIM8108-36 peak, as the fixed −16.5 / +104° stop bushes are sized)."""
    from .params import act
    fall = fall or Fall(d)
    rows = []
    for c in [c for c in res["cases"] if c["guard"] == "latch"]:
        loads = (c["load"], 0.0) if c["load"] <= 1.0 else (c["load"] / 2, c["load"] / 2)
        fall.set(dict(lift=fall.lift_rng[1], pitch=0.0, roll=math.radians(LATCH_DEG), yaw=0.0, elbow=0.0, wpitch=0.0, wroll=0.0), loads)
        M = np.zeros((fall.m.nv, fall.m.nv))
        if hasattr(fall.dat, "qM"):
            fall.mj.mj_fullM(fall.m, M, fall.dat.qM)
        else:
            fall.mj.mj_fullM(fall.m, fall.dat, M)
        J = float(M[fall.da["roll_r"], fall.da["roll_r"]])
        w, g = c["v_roll_stop"], c["g_roll_in"]
        row = dict(load=c["load"], omega=w, J=J, g=g)
        for key, k in (("steel", K_STOP_LIN), ("pu", K_STOP_PU)):
            kt = k * R_STOP ** 2
            T = w * math.sqrt(J * kt) + g
            dlt = math.degrees(w * math.sqrt(J / kt) + g / kt)
            row[key] = dict(T=T, F=T / R_STOP, over=dlt, wedge=wedge(LATCH_DEG - dlt))
        rows.append(row)
    a = act(d.arm.joints["shoulder_roll"][0])
    return dict(rows=rows, T_rated=a["rated"], T_peak=a["peak"], F_rated=a["rated"] / R_STOP, F_peak=a["peak"] / R_STOP)


# ------------------------------------------------------------------------------------------------------------- options
def dexterity():
    f = DOCS / "dexterity.json"
    if not f.exists():
        return None
    j = json.loads(f.read_text())
    keys = ("top-down", "45° angled", "side (horizontal)", "side, across the body")
    return dict(full=[j[k] for k in keys], near=[j["people_near"][k] for k in keys], keys=keys)


TARGETS = (0.30, 0.63, 0.62, 0.54)    # R16 (RND_PLAN R16 row)


def options(res):
    dx = dexterity()
    near = "/".join(f"{x:.0%}" for x in dx["near"]) if dx else "?"
    full = "/".join(f"{x:.0%}" for x in dx["full"]) if dx else "?"
    miss = sum(x < t for x, t in zip(dx["near"], TARGETS)) if dx else "?"
    L = res["latch"]
    return [
        dict(key="O0", name="Today: D25 software soft limit only", guarantee="none unpowered: the arm rests anywhere in the roll friction band "
             "(≈ ±3…6° about hanging) and a few N at the hand closes the wedge", cost="$0", mass="0", reach=f"full {full} (R16 ✅) with nobody near; {near} with a person near"),
        dict(key="O1", name="Fixed hard stop at +5° (the −16.5° bush moved: STOP_AT[0] = +5°)", guarantee="all states, powered or not, any push",
             cost="≈ $0 (geometry change; the bushes, plate and arm-rear half-width re-derive from STOP_AT)", mass="0",
             reach=f"{near} **always** ({miss} of 4 R16 targets missed; across-body −22 points)"),
        dict(key="O2", name="**Spring-applied latch at +5°, solenoid-retracted** (Ø10 hardened pin in a steel guide bush in the rear plate, "
             "on the same arm edge as the −16.5° bush; 24 V pull solenoid fed from the K1-switched bus through a supervisor-gated FET; "
             "pin-retracted switch)", guarantee="every state that starts at roll ≥ +5°: people near (the soft limit keeps it there), and "
             "every unpowered state that began there; the pin drops in by spring the moment K1 opens or the supervisor drops it. Residual: "
             "an unplanned loss of power while the arm is in the −16.5…+5° band with nobody near (the pin rests on the arm face until the "
             "arm next rises past +5°)", cost="≈ $14 per arm (solenoid $6–9, pin + bush $3, spring $0.5, hall switch + magnet $2, "
             "FET + wiring $1; ESTIMATE) → ≈ $28 robot", mass="≈ 45 g per arm (ESTIMATE)",
             reach=f"full {full} with nobody near; {near} with a person near or unpowered (as D25 today)"),
        dict(key="O2b", name="O2 + band spring (a 5–8 N·m spring plunger acting only over −16.5…+6°, pushes the unpowered arm out of the band "
             "onto the latch)", guarantee="all unpowered states (the arm leaves the band in ≲ 0.5 s, then the latch holds); push-proof once latched",
             cost="O2 + ≈ $3", mass="O2 + ≈ 15 g", reach=f"as O2, but the roll motor holds the spring (≈ 5–8 of 35 N·m) while working in the band"),
        dict(key="O3", name="Spring-applied friction brake on the roll output (≥ 1.5 × the worst roll gravity torque)",
             guarantee="holds gravity + payload at any angle; a push above the brake torque still closes the wedge",
             cost="≈ $60–120 per arm (no catalogue power-off brake fits round the Ø96 motor in the 121 × 96 dome; a custom caliper "
             "on the Ø76 adapter needs ≈ 1 kN spring clamp; ESTIMATE)", mass="≈ 0.3–0.6 kg per arm (ESTIMATE)", reach="none"),
        dict(key="O4", name="Geometry: roll axis ≈ +23 mm outboard (ROLL_Y), wedge ≥ 25 mm at −15° (pinch.md)",
             guarantee="all states", cost="redesign; ≈ $0 parts", mass="≈ +40 g per arm (longer roll link / ferrule boss; ESTIMATE)",
             reach="≈ none (R16 to re-run); +46 mm shoulder span against the R5 70 cm door (folded arms already 0.70–0.78 m, tip.md)"),
        dict(key="O5", name="Bias spring only (no latch): outward spring torque over the whole band", guarantee="stops gravity closing the "
             "wedge, not a push; the unpowered arm settles at the band's edge (≈ +5°) with nothing holding it there", cost="≈ $3",
             mass="≈ 15 g", reach="as O2b (motor torque in the band)"),
    ]


# -------------------------------------------------------------------------------------------------------------- report
def fmt_end(e):
    return ", ".join(f"{j} {e[j][0]:.0f}…{e[j][2]:.0f}° (median {e[j][1]:.0f})" for j in ("pitch", "roll", "elbow", "yaw"))


def report(res, res_d20=None):
    from sw import safety as S
    lo_r, hi_r = roll_stops()
    dr = res["drive"]
    L = ["# R17 safe power-off (generated by `python -m rnd.power_off`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         "## Back-drive model (drives off)", "",
         "| Joint | Actuator | Ratio | Back-drive torque (N·m, mid; ×0.5…×2) | Viscous (N·m·s/rad, twin datasheet) | Reflected rotor inertia (kg·m²) |", "|---|---|---|---|---|---|"]
    for j in ARM_J:
        r = dr[j]
        ratio = "" if r["N"] is None else f"{r['N']:.0f}:1"
        L.append(f"| {j} | {r['act']} | {ratio} |{r['T']:.2f} ({0.5 * r['T']:.2f}…{2 * r['T']:.2f}) | {r['c']:g} | {r['J']:.4f} |")
    L += ["", f"Shoulder balancer K = {res['K']:.1f} N·m (τ = K·sin pitch): it cancels about half of the straight-arm gravity moment, so pitch is "
          "never held by it alone. Lift: spring-applied pawl, held. Hard stops: roll "
          f"{lo_r:g} / {hi_r:g}° (roll.py STOP_AT), the others ±{HARD_STOP_EXTRA:g}° past the soft limits (ESTIMATE).", "",
          "## Where the arm goes unpowered", "",
          f"Runs of {T_SIM:g} s from rest. *Roll min* = the lowest roll angle either arm reached; *wedge* = the inboard gap there "
          f"(pinch.py table: {', '.join(f'{g:g} mm at {a:g}°' for a, g in WEDGE)}); *closed* = share of starts that went below the 25 mm "
          "finger gap; *v hand* = peak hand speed during the fall.", "",
          "| Start state | Load | Roll guard | Back-drive × / viscous × | n | Roll min (°) | Wedge min (mm) | Closed < 25 mm | At the −16.5° stop | Peak hand speed (m/s) | End pose (right arm) |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in res["cases"]:
        st = "people near (roll ≥ +8°, PERSON_WS)" if c["mode"] == "near" else "nobody near (roll −12…+5°)"
        L.append(f"| {st} | {c['load']:g} kg | {c['guard']} | {c['spread'][0]:g} / {c['spread'][1]:g} | {c['n']} | {c['roll_min']:.1f} | {c['wedge_min']:.0f} "
                 f"{'❌' if c['wedge_min'] < WEDGE_OK else '✅'} | {c['frac_closed']:.0%} | {c['frac_stop']:.0%} | {c['v_hand']:.1f} | {fmt_end(c['end'])} |")
    near = [c for c in res["cases"] if c["mode"] == "near" and c["guard"] == "none"]
    worst = min(near, key=lambda c: c["roll_min"])
    L += ["", f"**Reading.** Unpowered, pitch falls to hanging (balancer {res['K']:.1f} N·m against ≈ 10–17 N·m of arm + load), the elbow "
          "straightens below it, and the roll comes to rest wherever its gravity torque drops under the back-drive friction: a band of a few "
          "degrees about hanging (roll ≈ 0). From the people-near poses (roll ≥ +8°) the roll got down to "
          f"**{worst['roll_min']:.1f}°** ({worst['load']:g} kg, back-drive ×{worst['spread'][0]:g}, viscous ×{worst['spread'][1]:g}): the wedge at **{wedge(worst['roll_min']):.0f} mm**; "
          f"{max(c['frac_closed'] for c in near):.0%} of the starts in the worst set ended under 25 mm. From the band the soft limit removes "
          "(only allowed with nobody near), the arm stays in it. Either way **nothing holds it there**: the push table below.", ""]
    L += ["## Pushing the unpowered arm", "", "Hand force that back-drives the roll inward from hanging (arm straight down, lift top): back-drive "
          "friction minus any gravity torque already pushing inward, over the hand's lever about the roll axis. *Finger force* = what a "
          f"20 N push at the hand puts on a finger in the wedge (lever ratio, wedge at r ≈ {LEVER_WEDGE * 1000:.0f} mm).", "",
          "| Load | Roll at rest | Back-drive × | Hand lever (m) | Push to close (N) | Finger force at a 20 N push (N) |", "|---|---|---|---|---|---|"]
    for p in res["push"]:
        if p["spread"] == 1.0 or p["roll"] == 0.0:
            L.append(f"| {p['load']:g} kg | {p['roll']:+.0f}° | {p['spread']:g} | {p['lever']:.2f} | {p['F']:.1f} | {p['finger_20N']:.0f} |")
    lt = res["latch"]
    L += ["", "## Options (mechanical guarantee)", "", "| | Option | Guarantee | Cost | Mass | Reach (R16: top-down / 45° / side / across-body; targets 30/63/62/54 %) |",
          "|---|---|---|---|---|---|"]
    for o in options(res):
        L.append(f"| {o['key']} | {o['name']} | {o['guarantee']} | {o['cost']} | {o['mass']} | {o['reach']} |")
    L += ["", "## Latch sizing (O1 / O2)", "", "Unpowered fall onto a +5° stop, per load (people-near starts, back-drive ×1): contact speed from the "
          f"runs, J = the twin's roll inertia hanging (rotor {dr['roll']['J']:.3f} kg·m² included), stop stiffness steel {K_STOP_LIN / 1e6:.0f} kN/mm or "
          f"PU-capped {K_STOP_PU / 1e6:.1f} kN/mm at r {R_STOP * 1000:.0f} mm (ESTIMATE); overshoot = how far past +5° the arm presses in.", "",
          "| Load | Contact speed (rad/s) | J (kg·m²) | Gravity in (N·m) | Steel: peak N·m / kN, overshoot ° → wedge mm | PU cap: peak N·m / kN, overshoot ° → wedge mm |",
          "|---|---|---|---|---|---|"]
    for r in lt["rows"]:
        L.append(f"| {r['load']:g} kg | {r['omega']:.2f} | {r['J']:.2f} | {r['g']:.1f} | {r['steel']['T']:.0f} / {r['steel']['F'] / 1000:.2f}, "
                 f"{r['steel']['over']:.1f}° → {r['steel']['wedge']:.0f} | {r['pu']['T']:.0f} / {r['pu']['F'] / 1000:.2f}, {r['pu']['over']:.1f}° → {r['pu']['wedge']:.0f} |")
    geo = None
    try:
        from .parts import roll as RP
        if RP.INLINE:
            geo = RP.latch_geometry()
    except Exception:
        geo = None
    L += ["", f"Powered fault into the pin: rated {lt['T_rated']:.0f} N·m → {lt['F_rated'] / 1000:.2f} kN, stall {lt['T_peak']:.0f} N·m → "
          f"{lt['F_peak'] / 1000:.2f} kN (what the fixed −16.5 / +104° bushes are sized for: clamped by their M4 to the motor face, "
          "roll.drive_capacity). **Design (CAD behind PIP_ROLL_GUARD=latch, rnd/parts/roll.py LATCH):** Ø10 hardened head (5 mm into the arm's 8 mm "
          "edge, ISO 8734-style dowel, bending at 2.45 kN × 5 mm ≈ 125 MPa) guided by its hole in the 7075 rear plate and a Ø11.5 42CrMo4 sleeve in the "
          "roll-link floor, a Ø6 stem in the cup ring to the spring and a pull cable to a 24 V solenoid in the side block; retract stroke 8 mm "
          + (f"(pin centre at roll {geo['angle']:.1f}°, r {RP.LATCH['r']:g} mm; {geo['nearest']:.1f} mm to the nearest screw sleeve / bush, "
             f"{geo['to_plate_in']:.2f} / {geo['to_plate_out']:.2f} mm plate ring walls, {geo['to_floor_edge']:.2f} mm print outside the sleeve, "
             f"{geo['to_pilot']:.1f} mm to the driver pilot; the arm covers the pin over roll {geo['covered'][0]}…{geo['covered'][1]}°). " if geo else ". ")
          + "**Its weak link is the print behind the sleeve** (≈ 0.4 kN at 12 MPa), not the pin: sw/safety.py caps the roll torque at LATCH_T_MAX = 16 N·m "
          "(0.39 kN) while the pin may be in (people near, faults: there the drive only has to hold the arm, ≤ 14 N·m with the basket); with the "
          f"PU cap the unpowered falls stay ≤ {F_GUIDE / 1000:.1f} kN up to "
          f"{max([r['load'] for r in lt['rows'] if r['pu']['F'] <= F_GUIDE] or [0]):g} kg and reach "
          + ", ".join(f"{r['pu']['F'] / 1000:.2f} kN at {r['load']:g} kg" for r in lt["rows"] if r["pu"]["F"] > F_GUIDE)
          + ". So a metal path to the motor face (a flanged sleeve clamped under the two neighbouring M4) is the open detail before the "
          "latch can be credited for loaded falls or a stall; falls with the 9 kg basket are also where SW-T7 (basket only with the area clear, "
          "rnd/tip.py) does the work.", "",
          "**Supervisor half (sw/safety.py item 16; `roll_latch`, on with PIP_ROLL_GUARD=latch, default off):** `roll_latch_solenoid()` is "
          "energised (pin retracted) only while the supervisor is healthy and nobody is near; until the pin-retracted switch confirms "
          "(`set_latch_switch`), or LATCH_RETRACT_S (50 ms) without one, the roll target stays ≥ the +5° bound (+3° margin, never above the "
          "arm); a person near or any latched fault drops the pin at once; the roll torque is capped at LATCH_T_MAX while the pin may be in; "
          "ROLL_PARK_DEG (+8°) is where the planner parks the roll when idle or before a planned power-down (closes most of O2's residual "
          "window). Hardware half (electrical track): the solenoid is fed from the K1-switched bus through a supervisor-gated FET, so e-stop "
          "/ K1 open drops it whatever the software does; one GPIO in, one out. Tested in sw/test_s1.py test_roll_latch.", "",
          "**Recommendation (owner's call):** O2 (latch) if the across-body / top-down reach with nobody near matters; O1 if a permanent "
          "R16 loss is acceptable (it is free and has no residual); add O2b if the band residual (unplanned power loss with the arm across the "
          "body and nobody near) is not acceptable. O3 and O5 do not stop a push; O4 spends the door width. Bench: the roll back-drive "
          "torque (both directions, hot and cold), the latch's drop time (spring only) and a dummy-load fall onto the pin.",
          "", "Not covered here: the unpowered pitch fall itself (peak hand speeds above are an impact / crush hazard of their own: Astra's "
          "\"load-rated power-loss arrest\" applies to pitch too; a phase-short at the pitch drive slows it but does not hold it), the "
          "gripper's hold on the payload, and the lift (pawl)."]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    from .params import BASELINE
    res = study(BASELINE, n=8 if a.quick else 24, quick=a.quick)
    md = report(res)
    (DOCS / "power_off.md").write_text(md + "\n")
    (DOCS / "power_off.json").write_text(json.dumps(dict(res, options=options(res)), indent=1, default=float))
    print(md)


if __name__ == "__main__":
    main()
