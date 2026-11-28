"""All-inline arm study (owner, 2026-11-28: "Would moving to a more motor-based design help, and how would it affect cost?
I'm not concerned about the elements we added for rigidity").  python -m rnd.inline_study [--quick]
→ docs/rnd/inline_study.md + inline_study.json

Every big arm joint (shoulder pitch, shoulder roll, upper-arm yaw, elbow) driven by a geared actuator sitting on the joint
axis, as the approved inline GIM8108-36 shoulder roll (D24): no belt, pulley, stub, 6001, idler, cassette or fixed belt
centre. Variants (params ARM_MOTOR_SETS / with_arm_motors(); PIP_ARM_MOTORS=<set> for a params-level run, the default is
unchanged):
- **B0 today**: default `gim6010` set (GIM6010-8 + belt at pitch 3:1, yaw 2:1, elbow 2:1) + the inline GIM8108-36 roll.
- **B1 today + D20**: `d20` (GIM8108-8 + belt 3:1 at pitch and elbow).
- **I1 all-inline SteadyWin** (`inline`): GIM8108-36 at pitch, roll, elbow; GIM6010-48 at yaw.
- **I2a all-inline mixed** (`inline_dm`): GIM8108-36 at pitch, roll, elbow; dual-encoder DM-J4340-2EC at yaw.
- **I2b all-inline mixed** (`inline_dm8009`): as I2a with a dual-encoder DM-J8009-2EC (9:1 single stage) at the elbow.

Method. The torque, tip, basket, hug, thermal and accuracy rows re-run the existing models (rnd/spring.py, payload_study.py,
basket.py, tip.py, thermal.py network, accuracy.py constants) on each variant's params. There is **no inline CAD** for
pitch, yaw and elbow: their masses are today's CAD roll-ups (arm.masses(), shoulder.masses(), roll.masses()) minus the
belt-stage parts each variant removes (FROM CAD) plus ESTIMATE additions (MASS_LINES below; asserted against params);
envelopes are motor drawings + today's wall / cover allowances (ESTIMATE); the door stow re-measures today's stow pose
with the removed belt-stage hulls taken out and the motor cylinders added (pose not re-optimised). Back-drive force and
the collision numbers use a first-order model with cited scaling (reflected inertia J_rotor·N², back-driving
efficiency of a planetary η_b ≈ 2 − 1/η_f) and ESTIMATE rotor inertias / drag torques: **they are what the bench must
measure first**.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
DOCS = ROOT.parent / "docs" / "rnd"
REVIEW = ROOT.parent / "docs" / "review"
KEYS = ["B0", "B1", "I1", "I2a", "I2b"]
SET = {"B0": None, "B1": "d20", "I1": "inline", "I2a": "inline_dm", "I2b": "inline_dm8009"}
NAME = {"B0": "B0 today (GIM6010 belts + inline roll)", "B1": "B1 today + D20 (GIM8108-8 belts at pitch + elbow)",
        "I1": "I1 all-inline SteadyWin (8108-36 ×3, 6010-48 yaw)", "I2a": "I2a all-inline mixed (DM-J4340 yaw, 8108-36 elbow)",
        "I2b": "I2b all-inline mixed (DM-J4340 yaw, DM-J8009 elbow)"}
BIG = ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow")
LEVER = {"shoulder_pitch": 0.60, "shoulder_roll": 0.60, "upper_yaw": 0.43, "elbow": 0.43}   # m, hand moment arm (rnd/accuracy.py LEVER; yaw: forearm + hand with the elbow at 90°)

# ---------------------------------------------------------------------------------------------------------------- masses
# (item, kg, basis). Removals are FROM CAD (2026-11-28 roll-ups: arm.masses(), shoulder.masses(), roll.masses()); additions
# and shrinkage are ESTIMATE. Baselines: bone_mass_upper 0.95 (gim6010 0.94 + inline-roll neck 0.010), pitch module 1.07,
# yaw stage 0.74 (params; Astra #3 m7 integral pinion shafts, 2026-11-28).
NECK_G_PER_MM = 0.69e-3      # kg/mm, the upper-arm neck (params ROLL_DRIVES comment): a longer yaw drop shortens it
ELBOW_LINES = [
    ("elbow D26 direct stage: C45+QT integral pinion shaft, 6001 + bonded bushing", -0.213, "FROM CAD"),   # Astra #3 m7: was −0.159 (stub + alu 20T)
    ("elbow can nose / flange / rear cap", -0.033, "FROM CAD"),
    ("elbow shaft + integral 40T (the motor output carries a printed forearm hub instead)", -0.099, "FROM CAD"),
    ("elbow 6061 core tube (no elbow shaft left to stiffen; owner: rigidity adders not a constraint)", -0.051, "FROM CAD"),
    ("elbow belt + its share of screws / inserts", -0.020, "ESTIMATE"),
    ("one of the two 61808 (the motor's output bearing is the second support)", -0.020, "ESTIMATE"),
    ("elbow housing 168 → ≈ 120 mm tall, cassette plates gone (≈ 30 % of the ≈ 0.25 kg housing)", -0.080, "ESTIMATE"),
    ("7075 output adapter (as the roll's, 61 g FROM CAD) + printed forearm hub", +0.061 + 0.040, "ESTIMATE"),
    ("motor seat ring / rear cover + 8 × M4 (PCD 84)", +0.030, "ESTIMATE"),
]
PITCH_LINES = [
    ("D26 integral pinion shaft + idler + can nose / flange / cap", -0.249, "FROM CAD"),   # Astra #3 m7: was −0.107 (flange only counted)
    ("60T half of the integral pitch shaft (a plain Ø40 tube stays on the 2 × 61808)", -0.081, "ESTIMATE (½ of 0.162 FROM CAD)"),
    ("D26 exit-slot key + 6061 strap (belt-pull bracket stiffness)", -0.020, "FROM CAD"),
    ("pitch belt", -0.020, "ESTIMATE"),
    ("7075 output adapter + motor seat in the bracket (8 × M4)", +0.061 + 0.040, "ESTIMATE"),
]
YAW_LINES = {   # by yaw actuator
    "GIM6010-48": [("yaw D26 direct stage + bolted can flange / nose / cap", -0.252, "FROM CAD"),
                   ("40T half of the yaw shaft", -0.032, "ESTIMATE (½ of 0.063 FROM CAD)"), ("yaw belt", -0.010, "ESTIMATE"),
                   ("yaw block: the off-axis cartridge bay (x −81 … −129) goes, Ø80 seat instead (≈ 35 % of 0.231)", -0.080, "ESTIMATE"),
                   ("Ø60 7075 output adapter + motor seat ring + 6 × M4", +0.060, "ESTIMATE")],
    "DM-J4340-2EC": [("yaw D26 direct stage + bolted can flange / nose / cap", -0.252, "FROM CAD"),
                     ("40T half of the yaw shaft", -0.032, "ESTIMATE (½ of 0.063 FROM CAD)"), ("yaw belt", -0.010, "ESTIMATE"),
                     ("yaw block: the off-axis cartridge bay goes, Ø57 seat instead (≈ 45 % of 0.231)", -0.100, "ESTIMATE"),
                     ("Ø46 7075 output adapter + motor seat ring + 6 × M3", +0.050, "ESTIMATE")],
}
ENC_LINE = ("AS5048A elbow bracket / board / magnet (the DM-J8009-2EC reads its output itself)", -0.012, "params D24 v2 note (ESTIMATE)")


def mass_lines(key):
    """{param: [(item, kg, basis)]} of the deltas from B0, for an inline variant."""
    from .params import ARM_MOTOR_SETS
    s = ARM_MOTOR_SETS[SET[key]]
    yaw = s["upper_yaw"][0]
    el = list(ELBOW_LINES) + [(f"neck {s['yaw_drop_dm'] * 1000:.0f} mm shorter (yaw drop + {s['yaw_drop_dm'] * 1000:.0f} mm)",
                               -NECK_G_PER_MM * s["yaw_drop_dm"] * 1000, "params ROLL_DRIVES neck g/mm")]
    if s["elbow"][0].startswith("DM-"):
        el.append(ENC_LINE)
    return {"bone_mass_upper": el, "pitch_module_mass": list(PITCH_LINES), "yaw_stage_mass": list(YAW_LINES[yaw])}


def variants():
    from .params import BASELINE as B, with_arm_motors
    out = {}
    for k in KEYS:
        out[k] = B if SET[k] is None else with_arm_motors(B, SET[k])
    # the params entries must carry the MASS_LINES sums (rounded to 10 g)
    for k in ("I1", "I2a", "I2b"):
        for p, L in mass_lines(k).items():
            want = getattr(B.arm, p) + sum(m for _, m, _ in L)
            have = getattr(out[k].arm, p)
            assert abs(want - have) <= 0.0055, f"params ARM_MOTOR_SETS[{SET[k]!r}] {p} = {have:.3f}, MASS_LINES give {want:.3f}"
    return out


# --------------------------------------------------------------------------------------------------------------- prices
def prices():
    from . import m1_kit as K
    return {"GIM6010-8": (K.GIM6010[0], K.GIM6010[1]), "GIM8108-8": (K.GIM8108_8[0], K.GIM8108_8[1]),
            "GIM8108-36": (K.GIM8108_36[0], K.GIM8108_36[1]),
            "GIM6010-48": (135.0, "aifitlab $135 (backordered; $156 regular elsewhere, sold out), 2026-11-28 search snippets"),
            "DM-J4340-2EC": (155.0, "aifitlab / rcdrone $155 (backordered / sold out); Robotics Center $215 (24 V), 2026-11-28"),
            "DM-J8009-2EC": (470.0, "aifitlab $470 (backordered); J8009P $395–480, 2026-11-28"),
            "DM-J4310-2EC_V1.1": (125.0, "rnd/bom.py BEST_PRICE"), "STS3215-C018": (19.5, "m1_kit")}


HI = {"GIM6010-8": 94.0, "GIM8108-8": 213.99, "GIM8108-36": 306.99, "GIM6010-48": 156.0, "DM-J4340-2EC": 215.0, "DM-J8009-2EC": 480.0}


def drive_lines(d):
    """Per arm: [(joint, item, qty, unit $, basis)] for the four big joints' actuators, drive parts, joint encoders and
    thermal caps. The roll is the same inline GIM8108-36 in every variant (listed, so the arm totals are whole)."""
    from . import m1_kit as K
    from .params import SPECS
    P = prices()
    out = []
    d20 = d.arm.arm_motors == "d20"
    for j in BIG:
        n, r = d.arm.joints[j]
        out.append((j, f"actuator {n}", 1, P[n][0], P[n][1]))
        dual = bool(SPECS[n].get("output_encoder"))
        if r != 1.0:                                                  # D26 direct belt stage v4 (cartridge_m.parts_list_direct)
            big = d20 and j in ("shoulder_pitch", "elbow")
            out += [(j, "turned C45 drive stub (Ø12 h6 shaft, 8 × M3 flange)", 1, 12.0, "ESTIMATE: JLCCNC-class turning ≥ $5 + bar (m1_kit SLEEVE basis)"),
                    (j, "8 × DIN 7984 M3 × 6 stub screws + key", 1, 1.8, "ESTIMATE (m1_kit M3X6_PACK / KEYS shares)"),
                    (j, "keyed 20T HTD 5M pulley " + ("steel C45" if big else "aluminium"), 1, 18.0 if big else K.KEYED_PULLEY[0], "m1_kit HOLD_A / KEYED_PULLEY"),
                    (j, "6001-2RS + bushing adhesive share", 1, 1.5, "ESTIMATE"),
                    (j, "HTD 5M 15 mm belt " + ("PU steel-cord" if big else "rubber"), 1, 15.0 if big else K.BELT[0], "m1_kit HOLD_A / BELT"),
                    (j, "4 flange-ear M4 screws + nuts + washers", 1, 1.3, "ESTIMATE (bom flange-ear line ÷ 28)"),
                    (j, "aluminium finned rear cap (closed-sleeve thermal fix) + NTC", 1, (18.0 if big else K.AL_CAP[0]) + 0.6, "m1_kit AL_CAP / HOLD_A")]
            if j == "shoulder_pitch":
                out.append((j, "slotted idler: 2 × 625-2RS, M5 axle + nut + spacers, M4 jack screw", 1, 2.5, "ESTIMATE (bom 625ZZ $0.25)"))
            out.append((j, "AS5048A joint encoder + magnet on the shaft end", 1, 12.0, "rnd/bom.py sensor pack v2 line"))
        elif j == "shoulder_roll":
            out.append((j, "inline roll hardware (adapter, 61806, clamp ring, dowels, W8c rear mount; same in every variant)", 1,
                        sum(q * u for _g, it, q, u, _w, _s in K.HOLD_B["yes"][1:] if not any(x in it for x in ("tube", "jig", "Loctite"))),
                        "m1_kit HOLD_B (per arm, robot-wide consumables excluded)"))
            out.append((j, "off-axis joint encoder (ring magnet round the motor)", 1, 20.0, "ESTIMATE (the motor fills the axis; same in every variant)"))
        else:                                                        # inline pitch / yaw / elbow
            small = j == "upper_yaw"
            out.append((j, "7075 output adapter " + ("Ø60" if small else "Ø76") + " (CNC) + dowels + screws", 1, (20.0 if small else 30.0) + (5.0 if small else 9.0),
                        "ESTIMATE (as the roll's adapter + clamp-ring lines, m1_kit HOLD_B)"))
            if not small:
                out.append((j, "far-side support bearing", 1, 3.0, "ESTIMATE (61806 class, as the roll)"))
            if not dual:
                out.append((j, "off-axis joint encoder (ring magnet round the motor; the on-axis AS5048A has no axis to sit on)", 1, 20.0, "ESTIMATE"))
    return out


# ------------------------------------------------------------------------------------------------- back-drive / impact
ETA_STAGE = 0.93            # forward efficiency per planetary stage (ESTIMATE, small QDD class); back-driving η_b = 2 − 1/η_f
ETA_BELT = 0.95             # HTD belt both ways (arm_sizing.BELT_EFF)
DRAG_MOTOR = {"GIM6010-8": 0.010, "GIM6010-48": 0.010, "GIM8108-8": 0.015, "GIM8108-36": 0.015, "DM-J4340-2EC": 0.006,
              "DM-J8009-2EC": 0.015, "DM-J4310-2EC_V1.1": 0.005}   # N·m motor-side drag (bearings, cogging, seal), ESTIMATE
DRAG_GEAR = {1: 0.10, 2: 0.30}  # N·m output-side gearbox breakaway by stage count (ESTIMATE)
DRAG_BELT = 0.05            # N·m at the joint: belt hysteresis + the stage's extra 6001 / 61808 (ESTIMATE)
SPREAD = (0.5, 2.0)         # back-drive torque range for the table (× the mid ESTIMATE): unknown until benched
V_HAND = 0.5                # m/s hand speed for the impact rows (ESTIMATE: brisk reach; supervisor joint v_max 1.5 rad/s)
K_BODY = {"chest": 25e3, "hand": 75e3}   # N/m effective body-region stiffness (ISO/TS 15066 Annex A; ESTIMATE from memory of the table)
F_TRANSIENT = 280.0         # N: ISO/TS 15066 chest / hand quasi-static 140 N × 2 for transient contact (ESTIMATE from memory)
T_DETECT = (0.014, 0.018)   # s: sw/collision.py momentum-observer trip on contact (W6 bench runs, RND_LOG 2026-11-27)
THR = {"shoulder_pitch": 3.2, "shoulder_roll": 2.4, "upper_yaw": 1.6, "elbow": 2.4}   # N·m, sw/test_s1.py collision thresholds


def joint_drive(d, j):
    """Reflected inertia (kg·m²), back-drive torque at the joint (N·m), total ratio, stages, for joint j."""
    from .params import SPECS
    n, r = d.arm.joints[j]
    s = SPECS[n]
    N = s["gear_ratio"] * r
    st = s.get("stages", 1)
    eta_b = max(0.3, 2 - 1 / ETA_STAGE ** st)
    Jr = s.get("rotor_inertia_kgm2", 3e-5) * N ** 2
    T = (s["gear_ratio"] * DRAG_MOTOR.get(n, 0.01) / eta_b + DRAG_GEAR[min(st, 2)]) * (r / ETA_BELT if r != 1.0 else 1.0) + (DRAG_BELT if r != 1.0 else 0.0)
    return dict(act=n, N=N, stages=st, eta_b=eta_b, J=Jr, T=T)


def backdrive(d):
    rows = {}
    for j in BIG:
        g = joint_drive(d, j)
        lv = LEVER[j]
        rows[j] = dict(g, F=g["T"] / lv, F_lo=SPREAD[0] * g["T"] / lv, F_hi=SPREAD[1] * g["T"] / lv, m_hand=g["J"] / lv ** 2,
                       trip_rise=0.5 * g["T"], F_trip=(THR[j] + 0.5 * g["T"]) / lv)
    return rows


POSES = {"reach (pitch −60°, elbow −60°)": dict(pitch=-60, roll=8, yaw=0, elbow=-60, wpitch=0),
         "arm straight out (pitch −90°)": dict(pitch=-90, roll=8, yaw=0, elbow=0, wpitch=0)}


def effective_mass(d):
    """Hand effective mass (kg) along x / y / z and the worst of those that are not singular, from the twin's M(q) with and without the
    reflected rotor inertias as joint armature (right arm; lift and base held). Λ⁻¹ = J M⁻¹ Jᵀ (translation)."""
    import mujoco
    from .twin import build_xml
    m = mujoco.MjModel.from_xml_string(build_xml(d))
    dat = mujoco.MjData(m)
    jn = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
    dof = {k: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_r")] for k in list(jn) + ["wroll"]}
    from .params import SPECS
    arm = {}
    for k, full in jn.items():
        n, r = d.arm.joints[full]
        arm[k] = SPECS[n].get("rotor_inertia_kgm2", 3e-5) * (SPECS[n]["gear_ratio"] * r) ** 2
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
    idx = [dof[k] for k in ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")]
    out = {}
    for pn, pose in POSES.items():
        res = {}
        for lab, use in (("links only", False), ("with reflected rotor inertia", True)):
            m.dof_armature[:] = 0.0
            if use:
                for k, v in arm.items():
                    m.dof_armature[dof[k]] = v
            dat.qpos[:] = 0
            for k, v in pose.items():
                dat.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_r")]] = math.radians(v)
            mujoco.mj_forward(m, dat)
            M = np.zeros((m.nv, m.nv))
            if hasattr(dat, "qM"):                     # MuJoCo ≤ 3.x / 4.x signatures (as sw/collision.py)
                mujoco.mj_fullM(m, M, dat.qM)
            else:
                mujoco.mj_fullM(m, dat, M)
            Jp = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, dat, Jp, None, site)
            Ma = M[np.ix_(idx, idx)]
            Ja = Jp[:, idx]
            Li = Ja @ np.linalg.solve(Ma, Ja.T)
            ax = {c: (float(1 / Li[i, i]) if Li[i, i] > 0.02 else float("inf")) for i, c in enumerate("xyz")}   # Λ⁻¹ ≈ 0: singular along it (arm straight)
            res[lab] = dict(worst=max(v for v in ax.values() if math.isfinite(v)), **ax)
        out[pn] = res
    return out


def impact(m_eff, k):
    """Peak transient contact force (N) and its rise time (s) for a free effective mass at V_HAND on a body spring k."""
    return V_HAND * math.sqrt(k * m_eff), 0.5 * math.pi * math.sqrt(m_eff / k)


# ------------------------------------------------------------------------------------------------------------- accuracy
def accuracy_rows(d, joint_encoders):
    """Per joint (deg, mm at the hand) for the big joints + wrist pitch: belt joints by rnd/accuracy.py; inline joints by
    their reducer backlash (actuators.yaml) and motor encoder ÷ N, unless an output-side encoder reads the joint."""
    from . import accuracy as A
    from .params import SPECS
    base = A.joint_errors(d, joint_encoders)
    out = {}
    for j, lev in A.LEVER.items():
        n, r = d.arm.joints[j]
        s = SPECS[n]
        dual = bool(s.get("output_encoder"))
        if r != 1.0 or j == "wrist_pitch":
            e = base[j][0]
        elif joint_encoders or dual:
            e = A.JOINT_ENC
        else:
            e = math.hypot(s.get("backlash_arcmin", 15) / 60, A.MOTOR_ENC / s["gear_ratio"])
        out[j] = (e, math.radians(e) * lev * 1000)
    rss = math.sqrt(sum(mm ** 2 for _, mm in out.values()))
    return out, rss


# ------------------------------------------------------------------------------------------------------------- envelope
ENV = {   # mm (ESTIMATE unless noted): the joint body with walls + cover allowance; today's from the CAD constants
    "elbow": {"B0": "98 × 108 × 168 (fore-aft × along axis × height; arm.HOUSING, FROM CAD): motor 70 mm above the axis",
              "B1": "109 × 108 × 196 (arm.HOUSING D20, FROM CAD)",
              "I1": "Ø105 × 108 (Ø96 × 57 GIM8108-36 + 4.6 mm wall; along-axis span set by the forearm link plane), ≈ 125 tall with the neck socket",
              "I2a": "Ø105 × 108, ≈ 125 tall (as I1)",
              "I2b": "Ø107 × 108 (Ø98 × 61.7 DM-J8009, Damiao manual), ≈ 128 tall"},
    "yaw": {"B0": "175 × 92 plan (yaw axis → the off-axis GIM6010 cartridge at x −81 + can r 43; roll.py FROM CAD), drop 99 mm",
            "B1": "as B0",
            "I1": "Ø92 plan (Ø80 GIM6010-48 + wall + cover), 67.5 mm tall coaxial under the roll motor: drop 99 → 124 mm",
            "I2a": "Ø92 plan (the plates ±46 stay; Ø57 DM-J4340), ≈ 56 mm tall: drop 99 → 114 mm",
            "I2b": "as I2a"},
    "pitch": {"B0": "GIM6010 cartridge 70 mm off the pitch axis inside the torso, belt to the 60T (inside the 390 mm torso)",
              "B1": "GIM8108-8 cartridge off-axis (inside the torso)",
              "I1": "Ø96 × 57 GIM8108-36 coaxial inboard of the pitch bearings (inside the torso; the balancer cord and spring must re-route)",
              "I2a": "as I1", "I2b": "as I1"},
}
# Collision hulls (rnd/out/collision_inline) the inline variants remove, and the motor cylinders they add (stow overlay)
REMOVED_HULLS = {"I1": ("roll_yawcart", "upper_elbow_cart"), "I2a": ("roll_yawcart", "upper_elbow_cart"), "I2b": ("roll_yawcart", "upper_elbow_cart")}
CYL = {"I1": dict(elbow=(0.0525, 0.070), yaw=(0.046, 0.0675)), "I2a": dict(elbow=(0.0525, 0.070), yaw=(0.046, 0.056)),
       "I2b": dict(elbow=(0.0535, 0.074), yaw=(0.046, 0.056))}   # (radius incl. wall + cover, length) m


def stow_overlay(keys):
    """W(45° ± 2°) of today's door-stow pose (docs/rnd/stow.json) with each variant's hull swap (ESTIMATE: pose not re-optimised)."""
    from . import stow as S
    js = json.loads((DOCS / "stow.json").read_text())
    q = js["stow"]["45.0"]["q"]
    T = S.Twin()
    T.set(q)
    psis = [45 - S.PSI_TOL, 45, 45 + S.PSI_TOL]
    base_w = float(T.widths(psis).max())
    hi, lo = T.extremes(45.0)
    out = {"today": dict(w45=base_w, sets=[hi[1], lo[1]])}
    m, dd = T.m, T.d
    import mujoco
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    for k in keys:
        if k not in CYL:
            continue
        pts, pads = [], []
        for g, P, p in zip(T.geoms, T.points(), T.pad):
            if any(T.gname[g].startswith(h) for h in REMOVED_HULLS[k]):
                continue
            pts.append(P); pads.append(p)
        a = np.linspace(0, 2 * math.pi, 48, endpoint=False)
        for s in "rl":
            r, L = CYL[k]["elbow"]                       # axis along the upper body's y, at the elbow axis
            b = bid(f"fore_{s}")
            loc = np.array([[r * math.cos(t), y, r * math.sin(t)] for t in a for y in (-L / 2, L / 2)])
            pts.append(dd.xpos[b] + loc @ dd.xmat[b].reshape(3, 3).T); pads.append(0.0)
            r, L = CYL[k]["yaw"]                         # axis along the upper body's z, above the yaw interface
            b = bid(f"upper_{s}")
            loc = np.array([[r * math.cos(t), r * math.sin(t), z] for t in a for z in (0.0, L)])
            pts.append(dd.xpos[b] + loc @ dd.xmat[b].reshape(3, 3).T); pads.append(0.0)
        ws = []
        for psi in psis:
            n = np.array([-math.sin(math.radians(psi)), math.cos(math.radians(psi))])
            ws.append(max(float((P[:, :2] @ n).max()) + p for P, p in zip(pts, pads)) - min(float((P[:, :2] @ n).min()) - p for P, p in zip(pts, pads)))
        out[k] = dict(w45=max(ws))
    return out


# ----------------------------------------------------------------------------------------------- parts, prints, build
# Per joint drive, counted per arm. "parts" = distinct pieces to handle (bought + printed); "fasteners" = screws, nuts,
# washers, dowels, keys, set screws. From cartridge_m.parts_list_direct (D26), roll.py / m1_kit HOLD_B (inline), ESTIMATE.
BELT_JOINT = dict(parts=["actuator", "can sleeve (print)", "rear cap (print)", "drive stub", "small pulley", "6001", "wall-B bushing (print)",
                         "2 eccentric bushings (print)", "belt", "Al rear cap", "NTC", "encoder board", "magnet", "encoder bracket (print)"],
                  fasteners=8 + 1 + 1 + 4 * 3 + 5 + 3 + 2)        # stub 8, key, set screw, 4 ears × (screw, nut, washer), rear cap 5, Al cap 3, encoder 2
PITCH_IDLER = dict(parts=["2 × 625-2RS", "idler roller (print)", "2 spacers"], fasteners=4)   # M5 axle + nut, M4 jack + nut
ELBOW_CASSETTE = dict(parts=["cassette plate A (print)", "cassette plate B (print)", "2 cassette spacers (print)", "2 seat caps (print)", "6061 core tube"],
                      fasteners=2 * 2 + 4)                        # 2 × M5 + nyloc, seat-cap screws
INLINE_JOINT = dict(parts=["actuator", "7075 output adapter", "motor seat / rear cover (print)", "far-side bearing"],
                    fasteners=8 + 3 + 3 + 8)                      # 8 × M4 motor (PCD 84), 3 × M4 + 3 dowels at the boss, 8 × M5 adapter → link
INLINE_YAW = dict(parts=["actuator", "7075 output adapter", "motor seat (print)"], fasteners=6 + 3 + 2 + 6)
OFFAXIS_ENC = dict(parts=["ring magnet", "encoder board", "bracket (print)"], fasteners=2)
# printed parts removed per arm (print_plan.md rows, per piece), and the shrink of the parts that stay (ESTIMATE)
PRINT_REMOVED = {"cartridge M can (bolted, D24)": 3, "cartridge M rear cap (D24)": 3, "eccentric bushing": 6, "elbow cassette plate A": 1,
                 "elbow cassette plate": 1, "elbow cassette spacer": 2, "elbow seat cap A (+Y)": 1, "elbow seat cap B (−Y)": 1}
PRINT_SHRINK = {"elbow pulley hub": 0.5, "pitch shaft (integral 60T)": 0.5, "yaw shaft": 0.5, "upper bone": 0.15, "clevis + yaw block": 0.30}
PRINT_ADDED = (3, 45.0, 3.5)       # 3 motor seats, g each, generic h each (ESTIMATE)
# build minutes per arm (rnd/build_guide.py STEPS minutes where they exist)
BUILD = dict(belt_cart=9, belt_stage=15, encoder=7, idler=5, cassette=15, inline_heavy=25, inline_yaw=15, offaxis_enc=10)


def print_rows():
    import re
    rows = {}
    for line in (DOCS / "print_plan.md").read_text().splitlines():
        m = re.match(r"\| (.+?) \| (\d+) \| (\d+) \| ([\d.]+) \|", line)
        if m:
            rows[m.group(1)] = (int(m.group(2)), float(m.group(3)), float(m.group(4)))
    return rows


def simplicity(d, key):
    from .build_guide import arm_hours
    from .print_plan import P2S_FACTOR
    from .params import SPECS
    parts = fast = 0
    minutes = 0
    belt = [j for j in ("shoulder_pitch", "upper_yaw", "elbow") if d.arm.joints[j][1] != 1.0]
    inl = [j for j in ("shoulder_pitch", "upper_yaw", "elbow") if d.arm.joints[j][1] == 1.0]
    for j in belt:
        parts += len(BELT_JOINT["parts"]); fast += BELT_JOINT["fasteners"]; minutes += BUILD["belt_cart"] + BUILD["belt_stage"] + BUILD["encoder"]
        if j == "shoulder_pitch":
            parts += len(PITCH_IDLER["parts"]); fast += PITCH_IDLER["fasteners"]; minutes += BUILD["idler"]
        if j == "elbow":
            parts += len(ELBOW_CASSETTE["parts"]); fast += ELBOW_CASSETTE["fasteners"]; minutes += BUILD["cassette"]
    for j in inl:
        J = INLINE_YAW if j == "upper_yaw" else INLINE_JOINT
        parts += len(J["parts"]); fast += J["fasteners"]
        minutes += BUILD["inline_yaw" if j == "upper_yaw" else "inline_heavy"]
        if not SPECS[d.arm.joints[j][0]].get("output_encoder"):
            parts += len(OFFAXIS_ENC["parts"]); fast += OFFAXIS_ENC["fasteners"]; minutes += BUILD["offaxis_enc"]
    b0 = 2 * (BUILD["belt_cart"] + BUILD["belt_stage"] + BUILD["encoder"]) + BUILD["idler"] + BUILD["belt_cart"] + BUILD["belt_stage"] + BUILD["encoder"] + BUILD["cassette"]
    pr = print_rows()
    g = h = 0.0
    if len(inl) == 3:                                   # every inline variant here has pitch, yaw and elbow inline
        for n, k in PRINT_REMOVED.items():
            if n in pr:
                q, gg, hh = pr[n]
                g += gg / q * k; h += hh / q * k
        for n, f in PRINT_SHRINK.items():
            if n in pr:
                q, gg, hh = pr[n]
                g += gg / q * f; h += hh / q * f
        g -= PRINT_ADDED[0] * PRINT_ADDED[1]; h -= PRINT_ADDED[0] * PRINT_ADDED[2]
    return dict(parts=parts, fasteners=fast, drive_minutes=minutes, arm_hours=arm_hours() + (minutes - b0) / 60,
                print_g_saved=g, print_h_saved_p2s=h * P2S_FACTOR, belt_joints=belt, inline_joints=inl)


# ------------------------------------------------------------------------------------------------------------- the checks
def _fresh_ik():
    from .twin import Reacher
    a = Reacher.solve.__defaults__
    Reacher.solve.__defaults__ = a[:-1] + (np.random.default_rng(1),)


def checks(d, quick=False):
    from . import basket, payload_study, spring, thermal, tip
    from .arm_sizing import point_masses
    from .budget import mass_budget
    from .params import act, G
    for n, env in {"GIM6010-48": (0.080, 0.0675), "DM-J4340-2EC": (0.057, 0.056), "DM-J8009-2EC": (0.098, 0.0617)}.items():
        thermal.ENVELOPE.setdefault(n, env)                           # actuators.yaml frames (in-memory only)
    r = {}
    wm = spring.workspace_margins(d)
    r["wm"] = {j: dict(p95=v["p95"], max=v["max"], rated=v["rated"], peak=v["peak"], m_rated=v["m_rated"], m_peak=v["m_peak"]) for j, v in wm.items()}
    rows, _ = payload_study.run(d)
    hug = next(x for x in rows if x["case"].startswith("B: 20 lb two-handed hug") and "0.20" in x["case"])
    r["hug"] = dict(pitch=hug["rated"]["shoulder_pitch"], elbow=hug["rated"]["elbow"], worst=min(hug["rated"]["shoulder_pitch"], hug["rated"]["elbow"]))
    mb = mass_budget(d)
    r["arm_kg"] = sum(m for _, m in mb["arm (each, detachable)"])
    r["arm_items"] = mb["arm (each, detachable)"]
    pts, _ = point_masses(dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=0.0)))
    r["arm_moment_nm"] = sum(m * G * x for n, m, x in pts if n != "payload")            # arm straight out, about the pitch axis, no balancer
    r["elbow_node_kg"] = act(d.arm.joints["elbow"][0])["mass"]
    r["outboard_elbow_kg"] = sum(m for n, m, x in pts if x >= d.arm.l_upper - 1e-9 and n != "payload")
    r["pitch_side_kg"] = d.arm.pitch_module_mass + act(d.arm.joints["shoulder_pitch"][0])["mass"]
    r["speed_rpm"] = {j: act(d.arm.joints[j][0])["speed"] / d.arm.joints[j][1] * 60 / (2 * math.pi) for j in BIG}
    # thermal: worst static hold in the closed-sleeve network (conservative for an inline motor under a cover)
    th = {}
    for j, v in wm.items():
        k = thermal.network(v_act := d.arm.joints[j][0])
        x2 = (v["max"] / v["rated"]) ** 2
        dts = thermal.dt_rated(k["K"]) * x2
        th[j] = dict(act=v_act, T=thermal.AMBIENT + thermal.COVER_DT + dts, t_limit=thermal.hold_time(dts),
                     cont=v["rated"] * math.sqrt((thermal.T_LIMIT - thermal.AMBIENT - thermal.COVER_DT) / thermal.dt_rated(k["K"])), max=v["max"])
    r["thermal"] = th
    # tip (A10 cases, requested rules) + COM. twin.Reacher.solve keeps one RNG as a default argument, so its IK depends on
    # every solve before it in the process: reset it here and before the basket, as a fresh `python -m rnd.tip` / `rnd.basket` starts
    _fresh_ik()
    rules = tip.sw_rules()
    body = tip.Body(d)
    P = tip.poses(d, rules.box)
    cas = tip.casters(d, "D18b")
    cases = tip.run_cases(d, body, P, cas, tip.requested, rules)
    r["tip"] = dict(worst=min(c["reserve"] for c in cases), n_ok=sum(c["reserve"] >= tip.NEED - 1e-9 for c in cases), n=len(cases))
    ex = tip.extras(d, body, P, rules, cas)
    r["tip"]["basket_extra"] = [dict(name=e["name"], reserve=e["reserve"], ok=e["ok"]) for e in ex if "basket" in e["name"]]
    M, c, _ = body.com(P["tuck"][0], 0.0)
    M1, c1, _ = body.com(P["straight"][0], 0.0)
    r["com"] = dict(M=M, h_parked=float(c[2]), x_parked=float(c[0]), h_straight=float(c1[2]), x_straight=float(c1[0]))
    r["push_tip_n"] = tip.tip_force(d, body, P, cas)
    # basket (R2d) unless quick
    if not quick:
        dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=basket.PER_HAND))
        _fresh_ik()
        brows, _ = basket.run(dd)
        ok = [x for x in brows if x["x"] <= 0.45 and "washer" not in x["pose"] and "0.80" not in x["pose"]]
        r["basket"] = dict(worst=min((x["worst"] for x in ok if x["t"] is not None), default=0.0),
                           all_ik=all(x["t"] is not None for x in ok),
                           tip_wheel=min((x["tip_wheel"] for x in ok if x["t"] is not None), default=float("nan")),
                           rise_max=max((max(x["rise"].values()) for x in ok if x.get("rise")), default=float("nan")))
    r["backdrive"] = backdrive(d)
    r["meff"] = effective_mass(d)
    for enc in (False, True):
        jr, rss = accuracy_rows(d, enc)
        r["acc_enc" if enc else "acc"] = dict(rss=rss, joints={j: dict(deg=e, mm=mm) for j, (e, mm) in jr.items()})
    return r


def cost_summary(d):
    from .bom import CONTINGENCY
    L = drive_lines(d)
    arm = sum(q * u for _, _, q, u, _ in L)
    hi = arm + sum(HI.get(i.split()[1], u) - u for _, i, q, u, _ in L if i.startswith("actuator"))
    act = sum(q * u for _, i, q, u, _ in L if i.startswith("actuator"))
    return dict(lines=L, arm=arm, arm_hi=hi, actuators=act, drive_parts=arm - act, robot_landed=2 * arm * (1 + CONTINGENCY),
                robot_landed_hi=2 * hi * (1 + CONTINGENCY))


def run(quick=False, keys=None):
    V = variants()
    keys = keys or KEYS
    res = {}
    for k in keys:
        d = V[k]
        r = checks(d, quick)
        r["cost"] = cost_summary(d)
        r["simple"] = simplicity(d, k)
        r["joints"] = {j: list(d.arm.joints[j]) for j in BIG}
        r["mass_lines"] = mass_lines(k) if SET[k] and SET[k].startswith("inline") else {}
        res[k] = r
        print(k, f"arm {r['arm_kg']:.2f} kg, tip {r['tip']['worst'] * 1000:.0f} mm {r['tip']['n_ok']}/16, hug {r['hug']['worst']:.2f}, "
              f"basket {r.get('basket', {}).get('worst', float('nan')):.2f}, cost/arm ${r['cost']['arm']:.0f}", flush=True)
    res["stow"] = stow_overlay(keys)
    return res


# --------------------------------------------------------------------------------------------------------------- report
def _fmt_t(t):
    return "unlimited" if math.isinf(t) else f"{t / 60:.0f} min"


def _sd(x):
    """Signed dollars: +$57 / −$12."""
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


def report(res, keys):
    from .bom import CONTINGENCY
    b0 = res["B0"]
    bom_total = json.loads((DOCS / "bom.json").read_text())["total"] if (DOCS / "bom.json").exists() else float("nan")
    L = ["# All-inline arm study (generated by `python -m rnd.inline_study`)", "", __doc__.split("\n\n", 1)[1].strip(), ""]
    hdr = "| | " + " | ".join(NAME[k] for k in keys) + " |"
    L += ["## Summary", "", hdr, "|---|" + "---|" * len(keys)]
    row = lambda lab, f: L.append(f"| {lab} | " + " | ".join(f(res[k], k) for k in keys) + " |")
    row("Pitch / roll / yaw / elbow (actuator × belt)", lambda r, k: ", ".join(f"{n.replace('-2EC', '').replace('_V1.1', '')}{'' if x == 1 else f' ×{x:g}'}" for n, x in r["joints"].values()))
    row("**Arm drives per arm** (4 big joints: actuators + drive parts + joint encoders + thermal caps, $)", lambda r, k: f"**{r['cost']['arm']:,.0f}** ({r['cost']['actuators']:,.0f} + {r['cost']['drive_parts']:,.0f}); dearest listing {r['cost']['arm_hi']:,.0f}")
    row("**Robot, both arms, landed (+15 %)**, Δ vs B0", lambda r, k: f"**{r['cost']['robot_landed']:,.0f}** ({_sd(r['cost']['robot_landed'] - b0['cost']['robot_landed'])})")
    row(f"Robot BOM landed if applied to today's ${bom_total:,.0f} (bom.json, pre-W12)", lambda r, k: f"{bom_total + r['cost']['robot_landed'] - b0['cost']['robot_landed']:,.0f}")
    row("One arm, detachable piece (kg)", lambda r, k: f"{r['arm_kg']:.2f}")
    row("Elbow actuator (kg) / mass outboard of the elbow axis (kg)", lambda r, k: f"{r['elbow_node_kg']:.2f} / {r['outboard_elbow_kg']:.2f}")
    row("Arm gravity moment straight out, no balancer (N·m)", lambda r, k: f"{r['arm_moment_nm']:.1f}")
    row("Torso side per arm: pitch module + pitch actuator (kg)", lambda r, k: f"{r['pitch_side_kg']:.2f}")
    row("Robot mass (kg) / COM height parked, arms straight (m)", lambda r, k: f"{r['com']['M']:.1f} / {r['com']['h_parked']:.3f}, {r['com']['h_straight']:.3f}")
    row("**Tip A10** (16 cases, requested rules): worst reserve, cases ≥ 30 mm", lambda r, k: f"**{r['tip']['worst'] * 1000:.1f} mm, {r['tip']['n_ok']}/16** {'✅' if r['tip']['n_ok'] == 16 else '❌'}")
    row("Push that tips it, arms parked, 1 m up (N)", lambda r, k: f"{r['push_tip_n']:.0f}")
    row("**20 lb hug** (B, load 0.2 m out): rated × pitch / elbow", lambda r, k: f"**{r['hug']['pitch']:.2f} / {r['hug']['elbow']:.2f}** {'✅' if r['hug']['worst'] >= 1.5 else '❌'}")
    row("**Laundry basket** R2d: worst duty margin (≥ 1 passes) / max 2-min rise", lambda r, k: (f"**{r['basket']['worst']:.2f}** {'✅' if r['basket']['worst'] >= 1 else '❌'} / {r['basket']['rise_max']:.1f} K" if "basket" in r else "not run (--quick)"))
    row("Workspace margin rated / p95 (pitch, roll, yaw, elbow, wrist)", lambda r, k: ", ".join(f"{v['m_rated']:.2f}" for v in r["wm"].values()))
    row("Joint speed at rated (rpm): pitch / roll / yaw / elbow", lambda r, k: " / ".join(f"{v:.0f}" for v in r["speed_rpm"].values()))
    row("**Back-drive force at the hand** (N, mid ESTIMATE; ×0.5–2): pitch / roll / yaw / elbow", lambda r, k: "**" + " / ".join(f"{r['backdrive'][j]['F']:.1f}" for j in BIG) + "**")
    row("Reflected rotor inertia (kg·m²): pitch / roll / yaw / elbow", lambda r, k: " / ".join(f"{r['backdrive'][j]['J']:.3f}" for j in BIG))
    pk = list(POSES)[0]
    row(f"**Hand effective mass**, worst direction, {pk} (kg): links only → with rotors", lambda r, k: f"{r['meff'][pk]['links only']['worst']:.2f} → **{r['meff'][pk]['with reflected rotor inertia']['worst']:.2f}**")
    row(f"Impact at {V_HAND} m/s on the chest (k 25 N/mm): peak force (N), rise time (ms)",
        lambda r, k: "{:.0f} N, {:.0f} ms".format(*[(f, t * 1000) for f, t in [impact(r['meff'][pk]['with reflected rotor inertia']['worst'], K_BODY['chest'])]][0]))
    row("**Hand accuracy** (joint RSS, mm): no extra encoders / with joint-side encoders", lambda r, k: f"**{r['acc']['rss']:.1f} / {r['acc_enc']['rss']:.1f}**")
    row("Elbow envelope (mm)", lambda r, k: ENV["elbow"][k])
    row("Yaw envelope (mm)", lambda r, k: ENV["yaw"][k])
    row("R5 door stow W(45° ± 2°), today's pose with the hull swap (mm; ≤ 570)", lambda r, k: f"{res['stow'].get(k, res['stow']['today'])['w45'] * 1000:.0f}" + ("" if k in res["stow"] else " (today's CAD)"))
    row("Drive part count per arm (pitch, yaw, elbow: pieces / fasteners)", lambda r, k: f"{r['simple']['parts']} / {r['simple']['fasteners']}")
    row("Build per arm (h, build_guide model) / print saved per arm (g, P2S h)", lambda r, k: f"{r['simple']['arm_hours']:.1f} / {r['simple']['print_g_saved']:.0f} g, {r['simple']['print_h_saved_p2s']:.1f} h")
    L += [""]
    # recommendation
    L += recommendation(res, keys)
    # detail sections
    L += ["", "## Cost detail (per arm, the four big joints)", "", "Unit prices: rnd/m1_kit.py constants where they exist (GIM6010-8 $90, GIM8108-8 $115 ESTIMATE, "
          "GIM8108-36 $240 with the SDC303), the new actuators at the lowest US-seller listing found (actuators.yaml), belt-stage parts from "
          "m1_kit / bom lines or ESTIMATE as marked. Robot landed = 2 arms × (1 + 15 % contingency, rnd/bom.py).", ""]
    for k in keys:
        c = res[k]["cost"]
        L += [f"**{NAME[k]}: ${c['arm']:,.0f} per arm** (dearest listings ${c['arm_hi']:,.0f})", "", "| Joint | Item | Qty | $ | Basis |", "|---|---|---|---|---|"]
        L += [f"| {j} | {i} | {q} | {u:,.2f} | {s} |" for j, i, q, u, s in c["lines"]]
        L.append("")
    L += ["## Masses (inline variants: deltas from B0's CAD roll-ups)", ""]
    for k in keys:
        ml = res[k]["mass_lines"]
        if not ml:
            continue
        L += [f"**{NAME[k]}**", "", "| Param | Item | Δ kg | Basis |", "|---|---|---|---|"]
        for p, lines in ml.items():
            L += [f"| {p} | {i} | {m:+.3f} | {b} |" for i, m, b in lines]
            L.append(f"| **{p}** | **sum** | **{sum(m for _, m, _ in lines):+.3f}** | |")
        L.append("")
    L += ["Arm piece by item (kg):", "", "| Item | " + " | ".join(keys) + " |", "|---|" + "---|" * len(keys)]
    items = [n for n, _ in res[keys[0]]["arm_items"]]
    for n in items:
        L.append(f"| {n} | " + " | ".join(f"{dict(res[k]['arm_items']).get(n, 0):.3f}" for k in keys) + " |")
    L += ["", "## Hand back-drive and collision", "",
          "Pushing the arm by hand: back-drive torque at the joint T = N_gear·τ_drag,motor / η_b + τ_gear (× belt ratio / η_belt + belt-stage drag for "
          f"the belt joints); η_b = 2 − 1/η_f with η_f = {ETA_STAGE}^stages (the usual estimate for back-driving a planetary that is itself back-drivable); "
          f"motor drag {DRAG_MOTOR['GIM6010-8']}–{DRAG_MOTOR['GIM8108-36']} N·m and gearbox breakaway {DRAG_GEAR[1]} / {DRAG_GEAR[2]} N·m (1 / 2 stages) are ESTIMATES. "
          "Force at the hand = T / lever (pitch and roll 0.60 m, yaw and elbow 0.43 m). Reflected inertia = J_rotor·N² (rotor inertia ESTIMATE from an "
          "80-class actuator's published 579 g·cm², scaled by size; actuators.yaml). The 'trip' columns: an unmodelled friction band of ±50 % of T "
          "(friction of a 2-stage reducer moves with temperature and load) has to be added to the sw/collision.py threshold to keep 0 false trips; "
          "the hand force that trips the joint is then (THR + 0.5 T) / lever.", "",
          "| Variant | Joint | Actuator | N total | η_b | J refl (kg·m²) | at the hand (kg) | T back-drive (N·m) | **Hand force (N)** (range) | Trip force (N) |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for k in keys:
        for j in BIG:
            b = res[k]["backdrive"][j]
            L.append(f"| {k} | {j} | {b['act']} | {b['N']:.0f} | {b['eta_b']:.2f} | {b['J']:.4f} | {b['m_hand']:.2f} | {b['T']:.2f} | **{b['F']:.1f}** ({b['F_lo']:.1f}–{b['F_hi']:.1f}) | {b['F_trip']:.1f} |")
    L += ["", f"Hand effective mass Λ = (J M⁻¹ Jᵀ)⁻¹ on the twin (right arm, lift and base held; the payload body is the twin's 1 kg), and the transient "
          f"contact at {V_HAND} m/s: F = v·√(k·m), rise time t = (π/2)·√(m/k). ISO/TS 15066 transient limit ≈ {F_TRANSIENT:.0f} N for chest / hand (ESTIMATE).", "",
          "| Variant | Pose | m links only: worst / x / y / z (kg) | m with rotors: worst / x / y / z (kg) | chest k 25 N/mm: F, rise | hand k 75 N/mm: F, rise |", "|---|---|---|---|---|---|"]
    for k in keys:
        for pn, v in res[k]["meff"].items():
            a, b = v["links only"], v["with reflected rotor inertia"]
            fc, tc = impact(b["worst"], K_BODY["chest"])
            fh, th = impact(b["worst"], K_BODY["hand"])
            L.append(f"| {k} | {pn} | {a['worst']:.2f} / {a['x']:.2f} / {a['y']:.2f} / {a['z']:.2f} | {b['worst']:.2f} / {b['x']:.2f} / {b['y']:.2f} / {b['z']:.2f} | "
                     f"{fc:.0f} N, {tc * 1000:.0f} ms | {fh:.0f} N, {th * 1000:.0f} ms |")
    L += ["", f"**Reading.** The chest-contact force peaks {impact(b0['meff'][pk]['with reflected rotor inertia']['worst'], K_BODY['chest'])[1] * 1000:.0f}–"
          f"{max(impact(res[k]['meff'][pk]['with reflected rotor inertia']['worst'], K_BODY['chest'])[1] for k in keys) * 1000:.0f} ms after contact, about when "
          f"sw/collision.py trips ({T_DETECT[0] * 1000:.0f}–{T_DETECT[1] * 1000:.0f} ms): the first impact is set by the effective mass and the speed, "
          "not by the detector, so the reflected rotor inertia is part of the impact directly (the compliant hold only shapes what happens after). "
          "After the trip the person pushes the arm away against the back-drive force (a few N in every variant) plus whatever the compliant hold's "
          "gravity feed-forward gets wrong.", ""]
    L += ["## Accuracy (hand error, joint RSS at 0.6 m)", "",
          "Belt joints: rnd/accuracy.py (GIM backlash ÷ belt ratio + belt play + stretch). Inline joints: reducer backlash (actuators.yaml: GIM8108-36 / "
          "GIM6010-48 15 arcmin from listings, DM-J4340 15 and DM-J8009 10 ESTIMATE) with the motor encoder ÷ N, unless the actuator has an output-side "
          "encoder (Damiao 2EC) or a joint-side encoder is fitted (0.05° after calibration). The roll is inline in every variant.", "",
          "| Variant | " + " | ".join(f"{j} (no enc / enc)" for j in res[keys[0]]["acc"]["joints"]) + " | RSS no enc / enc |", "|---|" + "---|" * (len(res[keys[0]]["acc"]["joints"]) + 1)]
    for k in keys:
        a, e = res[k]["acc"], res[k]["acc_enc"]
        L.append(f"| {k} | " + " | ".join(f"{a['joints'][j]['mm']:.1f} / {e['joints'][j]['mm']:.1f}" for j in a["joints"]) + f" | **{a['rss']:.1f} / {e['rss']:.1f}** |")
    L += ["", "With an output-side encoder on a joint that has backlash, the position loop closes around the backlash: expect a small limit cycle or a "
          "dead band where the gravity torque crosses zero (arm-down poses), as accuracy.md says for the roll. The Damiao 2EC actuators read the "
          "output in the actuator; a SteadyWin inline joint needs an off-axis (ring-magnet) encoder because the motor fills the axis the AS5048A uses today.", ""]
    L += ["## Thermal (worst static hold, 1 kg, closed-sleeve network of rnd/thermal.py: conservative for an inline motor under a cover)", "",
          "| Variant | " + " | ".join(f"{j}: worst / continuous N·m, time to 100 °C" for j in ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow")) + " |", "|---|" + "---|" * 4]
    for k in keys:
        t = res[k]["thermal"]
        L.append(f"| {k} | " + " | ".join(f"{t[j]['max']:.1f} / {t[j]['cont']:.1f}, {_fmt_t(t[j]['t_limit'])}" for j in ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow")) + " |")
    L += ["", "## Envelope and door stow", "", "| Joint | " + " | ".join(keys) + " |", "|---|" + "---|" * len(keys)]
    for jn, v in ENV.items():
        L.append(f"| {jn} | " + " | ".join(v[k] for k in keys) + " |")
    st = res["stow"]
    L += ["", f"Door stow (rnd/stow.py pose for 45°, today {st['today']['w45'] * 1000:.0f} mm, set by {st['today']['sets'][0]} and {st['today']['sets'][1]}): "
          "the inline variants take out the off-axis yaw cartridge and the elbow cartridge hulls and add the motor cylinders (elbow on its axis, yaw above "
          "the yaw interface) with wall + cover allowance; the pose is not re-optimised and the other hulls are today's (the 168 mm elbow housing stays, "
          "so this is an upper bound).", ""]
    L += ["## Simplicity", "", "| Variant | Belt joints | Inline joints | Drive pieces | Drive fasteners | Drive build min | Arm build h | Print saved (g / P2S h) |", "|---|---|---|---|---|---|---|---|"]
    for k in keys:
        s = res[k]["simple"]
        L.append(f"| {k} | {', '.join(s['belt_joints']) or '—'} | {', '.join(s['inline_joints']) or 'roll only'} | {s['parts']} | {s['fasteners']} | "
                 f"{s['drive_minutes']} | {s['arm_hours']:.1f} | {s['print_g_saved']:.0f} / {s['print_h_saved_p2s']:.1f} |")
    L += ["", "Counts: a D26 belt joint is the cartridge (can, rear cap, Al cap, NTC), the stub + key + pulley + 6001 + bushing, two eccentric bushings, "
          "the belt and the shaft-end encoder (+ the pitch idler, + the elbow cassette and seat caps); an inline joint is the actuator, a metal output adapter, "
          "a printed seat and a far-side bearing (+ an off-axis encoder for a SteadyWin). Build minutes use rnd/build_guide.py's 9 min cartridge and 15 min "
          "stage; inline heavy joint 25 min (as the roll's 35 min without the crush sleeves and stop bushes), yaw 15, encoder 7 / off-axis 10 (ESTIMATE).", ""]
    L += bench()
    return "\n".join(L) + "\n"


def recommendation(res, keys):
    i1, i2a, i2b, b0, b1 = (res.get(k) for k in ("I1", "I2a", "I2b", "B0", "B1"))
    d = lambda a, b: a["cost"]["robot_landed"] - b["cost"]["robot_landed"]
    L = ["## Recommendation", ""]
    if not all((i1, i2a, i2b, b0, b1)):
        return L + ["(run all five variants for the recommendation)"]
    el = "I2b" if (i2b["hug"]["elbow"] >= 1.5 and i2b["backdrive"]["elbow"]["F"] <= i2a["backdrive"]["elbow"]["F"]) else "I2a"
    pick = res[el]
    L += [f"- **Elbow for I2: {'DM-J8009-2EC' if el == 'I2b' else 'GIM8108-36'}.** The DM-J8009 is 9:1 single-stage: elbow reflected inertia "
          f"{i2a['backdrive']['elbow']['J']:.3f} → {i2b['backdrive']['elbow']['J']:.3f} kg·m², back-drive {i2a['backdrive']['elbow']['F']:.1f} → "
          f"{i2b['backdrive']['elbow']['F']:.1f} N at the hand, an output encoder built in, {i2a['elbow_node_kg'] - i2b['elbow_node_kg']:.2f} kg lighter; "
          f"the hug elbow margin {i2a['hug']['elbow']:.2f} → {i2b['hug']['elbow']:.2f}× rated (still ≥ 1.5) for "
          f"{_sd(d(i2b, i2a))} landed on the robot.",
          f"- **All-inline works on every load check** that today's default fails: the 20 lb hug {b0['hug']['worst']:.2f}× → "
          f"{i1['hug']['worst']:.2f} / {i2a['hug']['worst']:.2f} / {i2b['hug']['worst']:.2f}× (B1/D20 {b1['hug']['worst']:.2f}×)"
          + (f", the basket {b0['basket']['worst']:.2f} → {i1['basket']['worst']:.2f} / {i2a['basket']['worst']:.2f} / {i2b['basket']['worst']:.2f}" if "basket" in b0 else "")
          + f"; tip A10 {b0['tip']['n_ok']}/16 (worst {b0['tip']['worst'] * 1000:.1f} mm) → {i1['tip']['n_ok']}/16 ({i1['tip']['worst'] * 1000:.1f}), "
          f"{i2a['tip']['n_ok']}/16 ({i2a['tip']['worst'] * 1000:.1f}), {i2b['tip']['n_ok']}/16 ({i2b['tip']['worst'] * 1000:.1f}).",
          f"- **Cost:** robot landed, both arms' big-joint drives: B0 ${b0['cost']['robot_landed']:,.0f}, B1 {_sd(d(b1, b0))}, I1 {_sd(d(i1, b0))}, "
          f"I2a {_sd(d(i2a, b0))}, I2b {_sd(d(i2b, b0))}. Against the D20 path the owner already needs for R2c/R2d: I1 {_sd(d(i1, b1))}, I2b {_sd(d(i2b, b1))}.",
          f"- **Mass:** arm piece {b0['arm_kg']:.2f} (B0) / {b1['arm_kg']:.2f} (B1) → {i1['arm_kg']:.2f} / {i2a['arm_kg']:.2f} / {i2b['arm_kg']:.2f} kg; "
          f"the elbow actuator {b0['elbow_node_kg']:.2f} → {i1['elbow_node_kg']:.2f} / {i2b['elbow_node_kg']:.2f} kg sits on the elbow axis instead of 70 mm above it.",
          f"- **Hand-feel and impact:** back-drive at the hand stays a few newtons everywhere (it is gravity you feel when you push an unpowered arm); "
          f"the hand effective mass in the reach pose goes {b0['meff'][list(POSES)[0]]['with reflected rotor inertia']['worst']:.2f} → "
          f"{i1['meff'][list(POSES)[0]]['with reflected rotor inertia']['worst']:.2f} (I1) / {i2b['meff'][list(POSES)[0]]['with reflected rotor inertia']['worst']:.2f} kg (I2b): "
          "36:1 and 48:1 two-stage reducers put more rotor inertia in the hand than the 16:1–24:1 belt joints. The 9:1 J8009 elbow and the light 43-class J4340 yaw "
          "rotor take most of that back.",
          f"- **Accuracy:** without extra encoders {b0['acc']['rss']:.1f} mm (B0) → {i1['acc']['rss']:.1f} (I1) / {i2b['acc']['rss']:.1f} (I2b); with joint-side "
          f"encoders {b0['acc_enc']['rss']:.1f} → {i1['acc_enc']['rss']:.1f} / {i2b['acc_enc']['rss']:.1f} mm. The dual-encoder Damiao joints get there with no extra parts.",
          f"- **Simplicity:** drive pieces per arm {b0['simple']['parts']} → {i1['simple']['parts']} / {i2b['simple']['parts']}; fasteners "
          f"{b0['simple']['fasteners']} → {i1['simple']['fasteners']} / {i2b['simple']['fasteners']} (fewer, larger screws: 8 × M4 motor + 3 × M4 and dowels at the boss + 8 × M5 into the link per heavy joint); arm build "
          f"{b0['simple']['arm_hours']:.1f} → {i1['simple']['arm_hours']:.1f} / {i2b['simple']['arm_hours']:.1f} h; no belt to tension or measure (D26's 0–165 N "
          "fixed-centre preload spread goes away).",
          "",
          f"**Recommendation: go all-inline as I2b** (GIM8108-36 at pitch and roll, dual-encoder DM-J4340-2EC at yaw, dual-encoder DM-J8009-2EC at the elbow), "
          "as the R2c/R2d motor path instead of D20, **gated on the bench** (below), **if the owner accepts the cost: it does not save money** "
          f"({_sd(d(i2b, b1))} landed against D20, {_sd(d(i2b, b0))} against today; the belt-stage parts it removes are ≈ $60–70 a joint, the 36:1 and "
          "dual-encoder actuators cost $150–380 more than a GIM6010-8). It passes the hug and the basket with margin, keeps the tip gate "
          f"(by {(i2b['tip']['worst'] - 0.030) * 1000:.0f} mm: the 1 kg pitch / elbow actuators move the COM forward, and I1 / I2a miss one or two "
          "cases), removes every belt stage on the arm (no tensioning, no fixed-centre preload spread), slims the yaw block and gives output-side "
          "encoders at yaw and elbow without extra parts. "
          f"If the J8009's $470 or its lead time is a problem, I2a (GIM8108-36 elbow, {_sd(d(i2a, b1))} against D20) is the cheaper fallback, with a "
          "stiffer elbow by hand and a tip case to recover (≈ 0.5 mm short: ballast or a planner rule). "
          "I1's GIM6010-48 yaw is the weakest choice: 48:1 two-stage (most reflected inertia at the hand of any yaw option), 20 rpm rated, no output encoder, "
          "conflicting torque listings. Keep B1 (D20) buildable as the fallback if the bench back-drive or backlash numbers come in bad.",
          "- **Cheaper inline motors?** Not found: the GIM6010-36 (18 N·m rated, ≈ 0.57 kg) lists at $180–217 (2026-11-28 snippets), within $60 of the "
          "GIM8108-36, so the inline cost is the price of 36:1 actuators, not of the frame size. Its listing offers a 'secondary encoder' option: a "
          "dual-encoder GIM8108-36 would give I1/I2 pitch the output encoder too (seller question).",
          "", "**Not covered (no inline CAD):** the inline pitch motor inside the torso next to the balancer (cord, die spring, crank: must be "
          "re-routed, the biggest packaging risk), the elbow fold clearance with a Ø105 knee, the stiffness / FEA of the new load paths (the owner "
          "is not holding us to the rigidity adders, but the hand-deflection gate R2b still applies), the R17 pinch sweep, and the 24 V speed of the "
          "Damiao parts under load."]
    return L


def bench():
    return ["## What the bench must measure to confirm it", "",
            "1. **Back-drive torque** of a GIM8108-36, a DM-J4340-2EC and a DM-J8009-2EC, unpowered and in the drive's zero-torque mode: a lever + "
            "luggage scale (the Step-1 rig), breakaway and running torque, both directions, at 20 °C and after a 20-min warm-up. The model above is "
            "the mid of a 4× range; this decides the hand-feel and the collision threshold rise.",
            "2. **Backlash** at the output of each, dial gauge on a 0.3 m lever, ± 1 N·m reversal (the 15 arcmin listing and the 10–15 arcmin ESTIMATEs): "
            "decides the no-encoder hand error and whether the encoder loop limit-cycles.",
            "3. **Rotor inertia** (or the drive's reported inertia from a sysid chirp, sw/sysid.py) → the reflected inertia and the impact rows.",
            "4. **Output-encoder behaviour** of the Damiao 2EC: absolute output angle over a power cycle, its noise, and whether the MIT-mode loop can "
            "use it (dual-loop) or only reports it.",
            "5. **Torque at 24 V** and speed under load for the DM-J4340 / DM-J8009 (catalogue is 24 V rated; confirm the 36 / 100 rpm), and the "
            "GIM8108-36 output bearing's radial / tilting rating for pitch and elbow duty (≥ 612 N radial is the roll's open seller question).",
            "6. **Collision trip** on the bench twin with the measured friction (sw/test_s1.py collision case with the new N, J and friction): trip "
            "time and the hand force at trip."]


# ------------------------------------------------------------------------------------------------------- reviewer brief
def brief(res, keys):
    b0, b1, i1, i2b = (res[k] for k in ("B0", "B1", "I1", "I2b"))
    pk = list(POSES)[0]
    row = lambda lab, f: f"| {lab} | " + " | ".join(f(res[k]) for k in keys) + " |"
    L = ["# Reviewer question: is an all-inline arm better for this robot?", "",
         "*One page. Generated by `python -m rnd.inline_study` (numbers: docs/rnd/inline_study.md / .json).*", "",
         "**The robot.** Pip v4: a 1.3 m home robot on a Ø50 cm omni base, mast lift, two 6-DOF arms (0.30 m upper arm, 0.28 m forearm, 0.156 m hand), "
         "≈ 31 kg with removable rear ballast, 24 V. Requirements that bind here: 1 kg in one hand anywhere in the workspace; R2c 20 lb (9.07 kg) "
         "hugged two-handed; R2d a 20 lb laundry basket from the floor to a bed; tip-over reserve ≥ 30 mm in 16 braking / push cases; R17 "
         "consumer-safe (closed covers, current-based collision detection that trips in 14–18 ms, then a compliant hold); BOM limit $5,500 "
         "(today ≈ $7,500 landed, a cost-down is under way); hobby build, 3D-printed structure.", "",
         "**Today.** Quasi-direct-drive SteadyWin GIM6010-8 (8:1) cartridges with an HTD 5M belt stage at shoulder pitch (3:1), upper-arm yaw (2:1) "
         "and elbow (2:1); the shoulder roll is already an inline GIM8108-36 (36:1, owner-approved). The default motors fail R2c/R2d; the planned fix (D20) "
         "puts GIM8108-8 + 3:1 belts at pitch and elbow.", "",
         "**The question (owner):** would a more motor-based design help, and what would it cost? Every big joint driven by a geared actuator on its "
         "own axis, no belts / pulleys / idlers.", "",
         "| | " + " | ".join(NAME[k] for k in keys) + " |", "|---|" + "---|" * len(keys),
         row("Big-joint drives, robot landed ($)", lambda r: f"{r['cost']['robot_landed']:,.0f}"),
         row("Arm piece (kg)", lambda r: f"{r['arm_kg']:.2f}"),
         row("20 lb hug, rated × (≥ 1.5)", lambda r: f"{r['hug']['worst']:.2f}"),
         row("Basket duty margin (≥ 1)", lambda r: f"{r['basket']['worst']:.2f}" if "basket" in r else "—"),
         row("Tip worst reserve / cases ≥ 30 mm", lambda r: f"{r['tip']['worst'] * 1000:.1f} mm, {r['tip']['n_ok']}/16"),
         row("Back-drive at the hand, N (pitch/roll/yaw/elbow; ESTIMATE)", lambda r: "/".join(f"{r['backdrive'][j]['F']:.1f}" for j in BIG)),
         row("Hand effective mass, reach pose (kg)", lambda r: f"{r['meff'][pk]['with reflected rotor inertia']['worst']:.2f}"),
         row("Hand error, no extra encoders / with (mm)", lambda r: f"{r['acc']['rss']:.1f} / {r['acc_enc']['rss']:.1f}"),
         row("Drive pieces / fasteners per arm", lambda r: f"{r['simple']['parts']} / {r['simple']['fasteners']}"), "",
         f"**Our reading.** All-inline passes every load check today's default fails and removes all belt stages; it costs "
         f"{_sd(i2b['cost']['robot_landed'] - b1['cost']['robot_landed'])} landed against the D20 plan (I2b) and adds rotor inertia at the hand "
         f"({b0['meff'][pk]['with reflected rotor inertia']['worst']:.2f} → {i2b['meff'][pk]['with reflected rotor inertia']['worst']:.2f} kg). "
         "We propose I2b, gated on bench back-drive / backlash / rotor-inertia measurements. Back-drive and inertia figures are first-order "
         "ESTIMATES (J_rotor·N², η_b = 2 − 1/η_f, guessed drag); the inline pitch, yaw and elbow have no CAD yet.", "",
         "**Please answer:**",
         "1. Is all-inline the better architecture for this robot (home use, people nearby, 20 lb two-handed lifts, hobby build), or would you keep "
         "belts at some joints? Which, and why?",
         "2. Are 36:1 two-stage planetaries acceptable at shoulder pitch and elbow for a robot that people push by hand and that must stop on "
         "contact (14–18 ms detection, compliant hold), or would you insist on ≤ 10:1 there? What reflected inertia / effective mass at the hand "
         "would you accept?",
         "3. Our back-drive model (η_b = 2 − 1/η_f, motor drag × N, gearbox breakaway 0.1–0.3 N·m): reasonable, optimistic or pessimistic for "
         "these SteadyWin / Damiao units? Any measured data you know of?",
         "4. Output-side encoders: do you trust the Damiao 2EC output encoder as the position loop's feedback with ~10–15 arcmin of backlash "
         "inside it, or only for absolute homing?",
         "5. Packaging: an inline pitch motor in the torso shares space with a die-spring gravity balancer; a Ø105 elbow knee; a coaxial yaw "
         "under the inline roll motor. Which of these would you change first?",
         "6. What would you change in the variant choice (actuator models, ratios, where the DM-J8009 vs GIM8108-36 goes)?"]
    return "\n".join(L) + "\n"


def _clean(o):
    """JSON-safe copy: inf / nan → None, numpy scalars → float, tuples → lists."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (bool, int, str)) or o is None:
        return o
    try:
        f = float(o)
    except (TypeError, ValueError):
        return str(o)
    return f if math.isfinite(f) else None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="skip the basket IK (slow)")
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args(argv)
    keys = a.only or KEYS
    res = run(a.quick, keys)
    md = report(res, keys)
    (DOCS / "inline_study.md").write_text(md)
    (DOCS / "inline_study.json").write_text(json.dumps(_clean(res), indent=1))
    if set(KEYS) <= set(keys):
        REVIEW.mkdir(parents=True, exist_ok=True)
        (REVIEW / "inline_study_question.md").write_text(brief(res, keys))
    print(md[:6000])


if __name__ == "__main__":
    main()
