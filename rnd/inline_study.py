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
- **B1y D20 + inline yaw** (`d20_yaw`, Astra review #3's recommended next study): B1 with only the upper-arm yaw changed to a
  coaxial dual-encoder DM-J4340-2EC (the yaw belt stage, stub, pulley, 6001, belt, cap and AS5048A removed).

Method. The torque, tip, basket, hug, thermal and accuracy rows re-run the existing models (rnd/spring.py, payload_study.py,
basket.py, tip.py, thermal.py network, accuracy.py constants) on each variant's params. There is **no inline CAD** for
pitch, yaw and elbow: their masses are today's CAD roll-ups (arm.masses(), shoulder.masses(), roll.masses()) minus the
belt-stage parts each variant removes (FROM CAD) plus ESTIMATE additions (MASS_LINES below; asserted against params);
envelopes are motor drawings + today's wall / cover allowances (ESTIMATE); the door stow re-measures today's stow pose
with the removed belt-stage hulls taken out and the motor cylinders added (pose not re-optimised). Back-drive force and
the collision numbers use a first-order model with cited scaling (reflected inertia J_rotor·N², back-driving
efficiency of a planetary η_b ≈ 2 − 1/η_f) and ESTIMATE rotor inertias / drag torques: **they are what the bench must
measure first**.

**Revision after Astra review #3 (2026-11-28, Part A).** (a) The 20 lb hug is judged at the R2c spec point, 0.25 m from the
elbow (0.20 / 0.30 m shown as brackets; the first version promoted 0.20 m to the summary). (b) Actuator masses sit on the
bodies that carry them, with their own COMs and cylinder inertias (study_xml: a belt-driven elbow motor on the upper arm
at its belt centre above the elbow axis; an inline actuator's housing, stator and rotor on the upstream body and its
output carrier on the downstream body; the belt yaw cartridge on the roll output 81 mm behind the yaw axis): the twin-based
rows (tip, basket, workspace torques, effective mass, gravity moment) run on that model. (c) Back-drive is compared as
whole-joint ratio N and reflected rotor inertia J_r·N²; the efficiency-based drag is not a breakaway-friction law and
carries an explicit 4× band. (d) Collision: directional effective mass m(n) = 1 / (nᵀ J M⁻¹ Jᵀ n) over contact normals,
poses, the hand and the elbow, empty hand and 1 kg, against the R17 bound m_max = (F_max / v)² / k = 1.05 kg (140 N,
0.5 m/s, 75 N/mm), and the hand speed v = F_max / √(k·m) that meets it.
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
KEYS = ["B0", "B1", "B1y", "I1", "I2a", "I2b"]
SET = {"B0": None, "B1": "d20", "B1y": "d20_yaw", "I1": "inline", "I2a": "inline_dm", "I2b": "inline_dm8009"}
NAME = {"B0": "B0 today (GIM6010 belts + inline roll)", "B1": "B1 today + D20 (GIM8108-8 belts at pitch + elbow)",
        "I1": "I1 all-inline SteadyWin (8108-36 ×3, 6010-48 yaw)", "I2a": "I2a all-inline mixed (DM-J4340 yaw, 8108-36 elbow)",
        "I2b": "I2b all-inline mixed (DM-J4340 yaw, DM-J8009 elbow)", "B1y": "B1y D20 + inline DM-J4340-2EC yaw"}
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
    """{param: [(item, kg, basis)]} of the deltas from B0 for an all-inline variant, from B1 for B1y (inline yaw only)."""
    from .params import ARM_MOTOR_SETS
    s = ARM_MOTOR_SETS[SET[key]]
    yaw = s["upper_yaw"][0]
    if key == "B1y":
        return {"bone_mass_upper": [(f"neck {s['yaw_drop_dm'] * 1000:.0f} mm shorter (yaw drop + {s['yaw_drop_dm'] * 1000:.0f} mm)",
                                     -NECK_G_PER_MM * s["yaw_drop_dm"] * 1000, "params ROLL_DRIVES neck g/mm")],
                "yaw_stage_mass": list(YAW_LINES[yaw])}
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
    for p, L in mass_lines("B1y").items():                            # B1y: deltas from B1
        want = getattr(out["B1"].arm, p) + sum(m for _, m, _ in L)
        have = getattr(out["B1y"].arm, p)
        assert abs(want - have) <= 0.0055, f"params ARM_MOTOR_SETS['d20_yaw'] {p} = {have:.3f}, MASS_LINES give {want:.3f}"
    # Astra #3 major 15: the programmatic default set must equal the default design (env-selected) arm
    assert dataclasses.replace(with_arm_motors(B, "gim6010").arm) == B.arm, "with_arm_motors(BASELINE, 'gim6010') != BASELINE.arm"
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
    d20 = d.arm.arm_motors in ("d20", "d20_yaw")
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
            if j == "upper_yaw":
                out.append((j, "aluminium finned ring on the rear face through the cover + NTC (thermal: thermal.md section 3, the bare J4340 "
                               "under the cover does not hold the worst 1 kg pose)", 1, K.AL_CAP[0] + 0.6 + 3.0, "m1_kit AL_CAP + ESTIMATE $3 for the ring adapter"))
    return out


# ------------------------------------------------------------------------------------------------- back-drive / impact
ETA_STAGE = 0.93            # forward efficiency per planetary stage (ESTIMATE, small QDD class); back-driving η_b = 2 − 1/η_f
ETA_BELT = 0.95             # HTD belt both ways (arm_sizing.BELT_EFF)
DRAG_MOTOR = {"GIM6010-8": 0.010, "GIM6010-48": 0.010, "GIM8108-8": 0.015, "GIM8108-36": 0.015, "DM-J4340-2EC": 0.006,
              "DM-J8009-2EC": 0.015, "DM-J4310-2EC_V1.1": 0.005}   # N·m motor-side drag (bearings, cogging, seal), ESTIMATE
DRAG_GEAR = {1: 0.10, 2: 0.30}  # N·m output-side gearbox breakaway by stage count (ESTIMATE)
DRAG_BELT = 0.05            # N·m at the joint: belt hysteresis + the stage's extra 6001 / 61808 (ESTIMATE)
SPREAD = (0.5, 2.0)         # back-drive torque band (× the mid ESTIMATE, a 4× wide band): unknown until benched. Astra #3: no verified
                            # measurement of these assemblies supports even this band; it is an illustration of the uncertainty, not a bound
V_HAND = 0.5                # m/s hand speed for the impact rows (ESTIMATE: brisk reach; supervisor joint v_max 1.5 rad/s)
K_BODY = {"chest": 25e3, "hand": 75e3}   # N/m effective body-region stiffness (ISO/TS 15066 Annex A; ESTIMATE from memory of the table)
F_TRANSIENT = 280.0         # N: ISO/TS 15066 chest / hand quasi-static 140 N × 2 for transient contact (ESTIMATE from memory; not Pip's limit)
F_R17 = 140.0               # N: Pip's own R17 limit, "≤ 140 N transient at the hands" (docs/RND_PLAN.md R17). Astra #3: use this, not 280 N
M_BOUND = (F_R17 / V_HAND) ** 2 / K_BODY["hand"]   # kg: fixed-target spring bound m_max = (F/v)²/k = 1.05 kg at 0.5 m/s, 75 N/mm
M_CONSTRAINED = 25.0        # kg: a direction with m(n) above this is a constrained direction of the rigid model (arm straight
                            # along its own axis), reported as such, not as a benign finite worst case
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
    """Per big joint: whole-joint ratio N (actuator × belt), rotor J_r and reflected J_r·N², the efficiency-model drag T with
    its 4× band, and the hand force at the fixed LEVER (a fixed lever is not the hand lever in every pose: Astra #3)."""
    from .params import SPECS
    rows = {}
    for j in BIG:
        g = joint_drive(d, j)
        lv = LEVER[j]
        rows[j] = dict(g, Jr=SPECS[g["act"]].get("rotor_inertia_kgm2", 3e-5), F=g["T"] / lv, F_lo=SPREAD[0] * g["T"] / lv, F_hi=SPREAD[1] * g["T"] / lv,
                       T_lo=SPREAD[0] * g["T"], T_hi=SPREAD[1] * g["T"], m_hand=g["J"] / lv ** 2,
                       trip_rise=0.5 * g["T"], F_trip=(THR[j] + 0.5 * g["T"]) / lv)
    return rows


# ------------------------------------------------------------------------- actuator masses on the right bodies (Astra #3)
F_HOUSING = 0.80            # ESTIMATE: share of a geared actuator's mass that is housing + stator + ring gear + driver + rotor (rides
                            # the upstream body); the output carrier, output flange and bearing inner (0.20) ride the downstream body
DIMS = {"GIM6010-8": (0.080, 0.040), "GIM8108-8": (0.096, 0.045), "GIM8108-36": (0.096, 0.057), "GIM6010-48": (0.080, 0.0675),
        "DM-J4340-2EC": (0.057, 0.056), "DM-J8009-2EC": (0.098, 0.0617), "DM-J4310-2EC_V1.1": (0.056, 0.046)}   # Ø, length (m): actuators.yaml / thermal.ENVELOPE
YAW_BELT_POS = (-0.08118, 0.0, -0.020)   # m in the roll body: the belt yaw cartridge (rnd/parts/roll.py YAW_C 81.18 behind the yaw axis; can
                                         # from the top plate (z −54) up, centre ≈ −20 mm: ESTIMATE from roll.py P1 / ZC)
ROLL_HOUSING_X = -0.030     # m: the inline roll GIM8108-36 housing centre behind the roll output face along the roll axis (ESTIMATE)
YAW_INLINE_COVER = (0.046, 0.056)   # m: collision cylinder of the coaxial J4340 + wall + cover (radius, length), as CYL


def study_xml(d, geometry=None, build=None):
    """twin.build_xml(d) with the actuator masses on the bodies that carry them (Astra #3 Part A): the twin puts every
    actuator as a sphere on its joint's child body at the axis. Here: elbow belt motor (ratio ≠ 1) on the upper arm at
    elbow_belt_c above the elbow axis; inline elbow: housing on the upper arm, output on the forearm; belt yaw cartridge on
    the roll output at YAW_BELT_POS; inline yaw: housing on the roll output coaxial above the yaw interface, output on
    the upper arm; inline roll: housing on the pitch body; wrist pitch: housing on the forearm. Cylinders, so MuJoCo
    gives each its own COM and inertia; the rotor's spin inertia enters as armature J_r·N² where effective mass is
    computed. Inline-yaw sets also get the collision swap: the off-axis yaw cartridge hull goes, a Ø92 coaxial motor
    cylinder comes in, and the upper-arm hulls stay fixed relative to the elbow (the neck shortens)."""
    import xml.etree.ElementTree as ET
    from . import twin
    from .params import act
    a = d.arm
    build = build or getattr(twin.build_xml, "_orig", twin.build_xml)        # never correct twice inside corrected_bodies
    root = ET.fromstring(build(d, geometry))
    bodies = {b.get("name"): b for b in root.iter("body")}

    def cyl(parent, m, D, L, pos, axis, coll=False):
        eul = {"x": "0 90 0", "y": "90 0 0", "z": "0 0 0"}[axis]
        g = ET.SubElement(parent, "geom", type="cylinder", size=f"{D / 2:.4f} {L / 2:.4f}", pos=" ".join(f"{v:.5f}" for v in pos),
                          euler=eul, mass=f"{max(m, 1e-6):.5f}", rgba=".1 .1 .1 1")
        if not coll:
            g.set("contype", "0"); g.set("conaffinity", "0"); g.set("group", "3")
        return g

    def first_sphere(b):
        return next(g for g in b.findall("geom") if g.get("type") == "sphere")
    for side in "rl":
        sg = -1 if side == "r" else 1
        sh, roll, up, fore, wr = (bodies[f"{n}_{side}"] for n in ("sh", "roll", "upper", "fore", "wrist"))
        z_el = -(a.l_upper - a.yaw_drop)
        # elbow
        n, r = a.joints["elbow"]
        me = act(n)["mass"]; D, L = DIMS.get(n, (0.08, 0.05))
        g = first_sphere(fore)
        if a.elbow_motor_at is None:
            if r != 1.0:
                g.set("mass", "1e-6")
                cyl(up, me, D, L, (0, 0, z_el + a.elbow_belt_c), "y")
            else:
                g.set("mass", f"{(1 - F_HOUSING) * me:.5f}")
                cyl(up, F_HOUSING * me, D, L, (0, 0, z_el), "y")
        # yaw
        n, r = a.joints["upper_yaw"]
        my = act(n)["mass"]; D, L = DIMS.get(n, (0.08, 0.05))
        g = first_sphere(up)
        if r != 1.0:
            g.set("mass", f"{a.yaw_stage_mass:.5f}")
            cyl(roll, my, D, L, YAW_BELT_POS, "z")
        else:
            g.set("mass", f"{a.yaw_stage_mass + (1 - F_HOUSING) * my:.5f}")
            cyl(roll, F_HOUSING * my, D, L, (0, 0, -a.yaw_drop + L / 2), "z")
            # collision swap (CAD hulls are exported for today's off-axis yaw at the default drop)
            from .params import BASELINE
            dz = a.yaw_drop - BASELINE.arm.yaw_drop
            for gg in list(roll.findall("geom")):
                if (gg.get("mesh") or "").startswith("roll_yawcart"):
                    roll.remove(gg)
            for gg in up.findall("geom"):
                if (gg.get("mesh") or "").startswith("upper_"):
                    gg.set("pos", f"0 0 {dz:.5f}")
            rc, lc = YAW_INLINE_COVER
            c = ET.SubElement(roll, "geom", type="cylinder", size=f"{rc:.4f} {lc / 2:.4f}", pos=f"0 0 {-a.yaw_drop + lc / 2:.5f}", mass="0.000001",
                              rgba=".9 .9 .92 1")
        # inline roll: housing on the pitch body
        n, r = a.joints["shoulder_roll"]
        if r == 1.0:
            mr = act(n)["mass"]; D, L = DIMS.get(n, (0.096, 0.057))
            g = first_sphere(roll)
            g.set("mass", f"{a.roll_stage_mass + (1 - F_HOUSING) * mr:.5f}")
            cyl(sh, F_HOUSING * mr, D, L, (ROLL_HOUSING_X, sg * a.roll_offset_y, 0), "x")
        # wrist pitch: housing on the forearm
        n, _ = a.joints["wrist_pitch"]
        mw = act(n)["mass"]; D, L = DIMS.get(n, (0.056, 0.046))
        g = first_sphere(wr)
        g.set("mass", f"{(1 - F_HOUSING) * mw:.5f}")
        cyl(fore, F_HOUSING * mw, D, L, (0, 0, -a.l_fore), "y")
    return ET.tostring(root, encoding="unicode")


class corrected_bodies:
    """Context: every twin model built inside (tip.Body, twin.Reacher, spring, the effective mass) uses study_xml."""

    def __enter__(self):
        from . import spring, twin
        self.orig = twin.build_xml
        f = lambda dd, geometry=None, _o=self.orig: study_xml(dd, geometry, _o)
        f._orig = self.orig
        twin.build_xml = spring.build_xml = f
        return self

    def __exit__(self, *exc):
        from . import spring, twin
        twin.build_xml = spring.build_xml = self.orig
        return False


POSES = {"reach (pitch −60°, elbow −60°)": dict(pitch=-60, roll=8, yaw=0, elbow=-60, wpitch=0),
         "arm straight out (pitch −90°)": dict(pitch=-90, roll=8, yaw=0, elbow=0, wpitch=0),
         "hand at chest height, elbow 90° (pitch −45°, elbow −90°)": dict(pitch=-45, roll=8, yaw=0, elbow=-90, wpitch=0),
         "low reach (pitch −30°, elbow −30°)": dict(pitch=-30, roll=8, yaw=0, elbow=-30, wpitch=0),
         "across the body (pitch −60°, roll 15°, yaw 45°, elbow −90°)": dict(pitch=-60, roll=15, yaw=45, elbow=-90, wpitch=0)}
N_NORMALS = 400


def _normals(n=N_NORMALS):
    """Fibonacci sphere of unit contact normals (a normal and its opposite give the same mass; kept for simplicity)."""
    i = np.arange(n) + 0.5
    ph = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(ph), np.sin(th) * np.sin(ph), np.cos(ph)], axis=1)


def v_allow(m, F=F_R17, k=K_BODY["hand"]):
    """Speed (m/s) at which a free effective mass m on a body spring k peaks at F (fixed-target spring model)."""
    return F / math.sqrt(k * max(m, 1e-9))


def effective_mass(d):
    """Directional effective mass (kg) m(n) = 1 / (nᵀ Λ⁻¹ n), Λ⁻¹ = J M⁻¹ Jᵀ (translation), at the hand (grasp site) and at
    the elbow (the forearm origin), over N_NORMALS contact normals, for each pose, with the hand empty and with 1 kg, links
    only and with the reflected rotor inertias as joint armature (right arm; lift and base held; study_xml bodies)."""
    import mujoco
    from .params import SPECS
    jn = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
    arm = {}
    for k, full in jn.items():
        n, r = d.arm.joints[full]
        arm[k] = SPECS[n].get("rotor_inertia_kgm2", 3e-5) * (SPECS[n]["gear_ratio"] * r) ** 2
    nrm = _normals()
    out = {}
    for pay in (0.0, 1.0):
        dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=max(pay, 1e-4)))
        m = mujoco.MjModel.from_xml_string(study_xml(dd))
        dat = mujoco.MjData(m)
        dof = {k: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_r")] for k in list(jn) + ["wroll"]}
        idx = [dof[k] for k in ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")]
        site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
        fore = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "fore_r")
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
                Jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, dat, Jp, None, site)
                Je = np.zeros((3, m.nv)); mujoco.mj_jac(m, dat, Je, None, dat.xpos[fore], fore)
                rr = {}
                # "hand": every arm joint free over the first peak (no controller stiffness); "hand, wrist held": the wrist's
                # position loop holds the hand to the forearm (the conservative case: the light hand cannot swing away)
                for where, J_, ix in (("hand", Jp, idx), ("hand, wrist held", Jp, idx[:4]), ("elbow", Je, idx)):
                    Ma = M[np.ix_(ix, ix)]
                    Ja = J_[:, ix]
                    Li = Ja @ np.linalg.solve(Ma, Ja.T)
                    mn = 1.0 / np.maximum(np.einsum("ij,jk,ik->i", nrm, Li, nrm), 1e-12)
                    fin = mn[mn <= M_CONSTRAINED]
                    ax = {c: float(1 / Li[i, i]) if Li[i, i] > 1 / M_CONSTRAINED else float("inf") for i, c in enumerate("xyz")}
                    ev = np.sort(1.0 / np.maximum(np.linalg.eigvalsh(0.5 * (Li + Li.T)), 1e-12))   # principal effective masses
                    pr = [float(v) if v <= M_CONSTRAINED else float("inf") for v in ev]
                    fin_pr = [v for v in pr if math.isfinite(v)]
                    rr[where] = dict(principal=pr, worst_free=max(fin_pr) if fin_pr else float("inf"), n_constrained=sum(not math.isfinite(v) for v in pr),
                                     worst=float(fin.max()) if fin.size else float("inf"), median=float(np.median(mn)),
                                     least=float(mn.min()), constrained=float((mn > M_CONSTRAINED).mean()),
                                     within=float((mn <= M_BOUND).mean()), **ax)
                res[lab] = rr
            out.setdefault(pn, {})[f"{pay:g} kg"] = res
    return out


def impact(m_eff, k, v=V_HAND):
    """Peak transient contact force (N) and its rise time (s) for a free effective mass at speed v on a body spring k."""
    return v * math.sqrt(k * m_eff), 0.5 * math.pi * math.sqrt(m_eff / k)


# ------------------------------------------------------------------------------------------------------------- accuracy
def accuracy_rows(d, joint_encoders, dual_closes=True):
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
        elif (joint_encoders and not (dual and not dual_closes)) or (dual and dual_closes):
            e = A.JOINT_ENC                     # a 2EC output encoder closing an outer loop (Astra #3: only if its bandwidth / latency are shown)
        else:
            e = math.hypot(s.get("backlash_arcmin", 15) / 60, A.MOTOR_ENC / s["gear_ratio"])
        out[j] = (e, math.radians(e) * lev * 1000)
    rss = math.sqrt(sum(mm ** 2 for _, mm in out.values()))
    return out, rss


# ------------------------------------------------------------------------------------------------------------- envelope
ENV = {   # mm (ESTIMATE unless noted): the joint body with walls + cover allowance; today's from the CAD constants
    "elbow": {"B0": "98 × 108 × 168 (fore-aft × along axis × height; arm.HOUSING, FROM CAD): motor 70 mm above the axis",
              "B1": "109 × 108 × 196 (arm.HOUSING D20, FROM CAD)", "B1y": "as B1",
              "I1": "Ø105 × 108 (Ø96 × 57 GIM8108-36 + 4.6 mm wall; along-axis span set by the forearm link plane), ≈ 125 tall with the neck socket",
              "I2a": "Ø105 × 108, ≈ 125 tall (as I1)",
              "I2b": "Ø107 × 108 (Ø98 × 61.7 DM-J8009, Damiao manual), ≈ 128 tall"},
    "yaw": {"B0": "175 × 92 plan (yaw axis → the off-axis GIM6010 cartridge at x −81 + can r 43; roll.py FROM CAD), drop 99 mm",
            "B1": "as B0",
            "B1y": "Ø92 plan (the plates ±46 stay; Ø57 × 56 DM-J4340 + wall + cover) coaxial under the roll motor: drop 99 → 114 mm (+15; "
                   "Astra: +15–25 mm); the 175 mm fore-aft cartridge bay behind the yaw axis goes",
            "I1": "Ø92 plan (Ø80 GIM6010-48 + wall + cover), 67.5 mm tall coaxial under the roll motor: drop 99 → 124 mm",
            "I2a": "Ø92 plan (the plates ±46 stay; Ø57 DM-J4340), ≈ 56 mm tall: drop 99 → 114 mm",
            "I2b": "as I2a"},
    "pitch": {"B0": "GIM6010 cartridge 70 mm off the pitch axis inside the torso, belt to the 60T (inside the 390 mm torso)",
              "B1": "GIM8108-8 cartridge off-axis (inside the torso)", "B1y": "as B1",
              "I1": "Ø96 × 57 GIM8108-36 coaxial inboard of the pitch bearings (inside the torso; the balancer cord and spring must re-route)",
              "I2a": "as I1", "I2b": "as I1"},
}
# Collision hulls (rnd/out/collision_inline) the inline variants remove, and the motor cylinders they add (stow overlay)
REMOVED_HULLS = {"I1": ("roll_yawcart", "upper_elbow_cart"), "I2a": ("roll_yawcart", "upper_elbow_cart"), "I2b": ("roll_yawcart", "upper_elbow_cart"),
                 "B1y": ("roll_yawcart",)}
CYL = {"I1": dict(elbow=(0.0525, 0.070), yaw=(0.046, 0.0675)), "I2a": dict(elbow=(0.0525, 0.070), yaw=(0.046, 0.056)),
       "I2b": dict(elbow=(0.0535, 0.074), yaw=(0.046, 0.056)), "B1y": dict(yaw=YAW_INLINE_COVER)}   # (radius incl. wall + cover, length) m


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
            if "elbow" in CYL[k]:
                r, L = CYL[k]["elbow"]                   # axis along the upper body's y, at the elbow axis
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
                         "belt", "Al rear cap", "NTC", "encoder board", "magnet", "encoder bracket (print)"],   # D26 fixed centres: no eccentric bushings (Astra #3)
                  fasteners=8 + 1 + 1 + 4 * 3 + 5 + 3 + 2)        # stub 8, key, set screw, 4 ears × (screw, nut, washer), rear cap 5, Al cap 3, encoder 2
PITCH_IDLER = dict(parts=["2 × 625-2RS", "idler roller (print)", "2 spacers"], fasteners=4)   # M5 axle + nut, M4 jack + nut
ELBOW_CASSETTE = dict(parts=["cassette plate A (print)", "cassette plate B (print)", "2 cassette spacers (print)", "2 seat caps (print)", "6061 core tube"],
                      fasteners=2 * 2 + 4)                        # 2 × M5 + nyloc, seat-cap screws
INLINE_JOINT = dict(parts=["actuator", "7075 output adapter", "motor seat / rear cover (print)", "far-side bearing"],
                    fasteners=8 + 3 + 3 + 8)                      # 8 × M4 motor (PCD 84), 3 × M4 + 3 dowels at the boss, 8 × M5 adapter → link
INLINE_YAW = dict(parts=["actuator", "7075 output adapter", "motor seat (print)", "Al finned ring + NTC"], fasteners=6 + 3 + 2 + 6 + 3)
OFFAXIS_ENC = dict(parts=["ring magnet", "encoder board", "bracket (print)"], fasteners=2)
# printed parts removed per arm (print_plan.md rows, per piece), and the shrink of the parts that stay (ESTIMATE)
# per joint made inline: printed pieces removed (print_plan.md rows; the eccentric bushings are not counted: D26 fixed centres
# removed them, belts.md, though print_plan.md still lists the row) and the shrink of the parts that stay (ESTIMATE). Astra #3:
# the gram figure is an ESTIMATE on print_plan rows, not a printed-only mass ledger of the replacement parts.
PRINT_REMOVED_J = {"shoulder_pitch": {"cartridge M can (bolted, D24)": 1, "cartridge M rear cap (D24)": 1},
                   "upper_yaw": {"cartridge M can (bolted, D24)": 1, "cartridge M rear cap (D24)": 1},
                   "elbow": {"cartridge M can (bolted, D24)": 1, "cartridge M rear cap (D24)": 1, "elbow cassette plate A": 1, "elbow cassette plate": 1,
                             "elbow cassette spacer": 2, "elbow seat cap A (+Y)": 1, "elbow seat cap B (−Y)": 1}}
PRINT_SHRINK_J = {"shoulder_pitch": {"pitch shaft (integral 60T)": 0.5}, "upper_yaw": {"yaw shaft": 0.5, "clevis + yaw block": 0.30},
                  "elbow": {"elbow pulley hub": 0.5, "upper bone": 0.15}}
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
    b0 = 3 * (BUILD["belt_cart"] + BUILD["belt_stage"] + BUILD["encoder"]) + BUILD["idler"] + BUILD["cassette"]
    pr = print_rows()
    g = h = 0.0
    for j in inl:
        for n, k in PRINT_REMOVED_J[j].items():
            if n in pr:
                q, gg, hh = pr[n]
                g += gg / q * k; h += hh / q * k
        for n, f in PRINT_SHRINK_J[j].items():
            if n in pr:
                q, gg, hh = pr[n]
                g += gg / q * f; h += hh / q * f
        g -= PRINT_ADDED[1]; h -= PRINT_ADDED[2]
    return dict(parts=parts, fasteners=fast, drive_minutes=minutes, arm_hours=arm_hours() + (minutes - b0) / 60,
                print_g_saved=g, print_h_saved_p2s=h * P2S_FACTOR, belt_joints=belt, inline_joints=inl)


# ------------------------------------------------------------------------------------------------------------- the checks
def _fresh_ik():
    from .twin import Reacher
    a = Reacher.solve.__defaults__
    Reacher.solve.__defaults__ = a[:-1] + (np.random.default_rng(1),)


def gravity_moment(d):
    """Arm straight out (pitch −90°, the rest 0), no payload, no balancer: gravity torque about the pitch axis (N·m) from the
    study_xml twin (actuator masses on their bodies; the belt elbow motor sits up the upper arm)."""
    import mujoco
    dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=1e-4))
    m = mujoco.MjModel.from_xml_string(study_xml(dd))
    dat = mujoco.MjData(m)
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "pitch_r")
    dat.qpos[m.jnt_qposadr[j]] = math.radians(-90)
    mujoco.mj_forward(m, dat)
    return abs(float(dat.qfrc_bias[m.jnt_dofadr[j]]))


def checks(d, quick=False):
    with corrected_bodies():
        return _checks(d, quick)


def _checks(d, quick=False):
    from . import basket, payload_study, spring, thermal, tip
    from .arm_sizing import point_masses
    from .budget import mass_budget
    from .params import act
    r = {}
    wm = spring.workspace_margins(d)
    r["wm"] = {j: dict(p95=v["p95"], max=v["max"], rated=v["rated"], peak=v["peak"], m_rated=v["m_rated"], m_peak=v["m_peak"]) for j, v in wm.items()}
    rows, _ = payload_study.run(d)
    r["hugs"] = {}
    for la in (0.20, 0.25, 0.30):
        hug = next(x for x in rows if x["case"].startswith("B") and f"{la:.2f} m out" in x["case"])
        r["hugs"][f"{la:.2f}"] = dict(pitch=hug["rated"]["shoulder_pitch"], elbow=hug["rated"]["elbow"],
                                      worst=min(hug["rated"]["shoulder_pitch"], hug["rated"]["elbow"]), t=hug["t"]["elbow"])
    r["hug"] = r["hugs"]["0.25"]                                      # Astra #3: the R2c spec point
    mb = mass_budget(d)
    r["arm_kg"] = sum(m for _, m in mb["arm (each, detachable)"])
    r["arm_items"] = mb["arm (each, detachable)"]
    pts, _ = point_masses(dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=0.0)))
    r["arm_moment_nm"] = gravity_moment(d)
    r["arm_moment_point_nm"] = sum(m * 9.81 * x for n, m, x in pts if n != "payload")   # the first version's point-mass figure
    r["elbow_node_kg"] = act(d.arm.joints["elbow"][0])["mass"]
    n_el, r_el = d.arm.joints["elbow"]
    r["elbow_split"] = (dict(upper=r["elbow_node_kg"], fore=0.0, where=f"{d.arm.elbow_belt_c * 1000:.0f} mm up the upper arm") if r_el != 1.0 else
                        dict(upper=F_HOUSING * r["elbow_node_kg"], fore=(1 - F_HOUSING) * r["elbow_node_kg"], where="on the elbow axis"))
    r["outboard_elbow_kg"] = sum(m for n, m, x in pts if x >= d.arm.l_upper - 1e-9 and n != "payload") - r["elbow_node_kg"] + r["elbow_split"]["fore"]
    r["pitch_side_kg"] = d.arm.pitch_module_mass + act(d.arm.joints["shoulder_pitch"][0])["mass"]
    r["speed_rpm"] = {j: act(d.arm.joints[j][0])["speed"] / d.arm.joints[j][1] * 60 / (2 * math.pi) for j in BIG}
    # thermal: worst 1 kg static hold and the R2c hug in the mount the BOM pays for (thermal.mount_of), with the band
    th = {}
    for j, v in wm.items():
        if j not in BIG:
            continue
        n = d.arm.joints[j][0]
        mount, cap = thermal.mount_of(d, j)
        x2 = (v["max"] / v["rated"]) ** 2
        b = thermal.band(n, x2, mount, cap)
        th[j] = dict(act=n, mount=mount + (" + Al cap" if cap else ""), max=v["max"], rated=v["rated"], T=b["nom"], T_hi=b["hi"], T_hot=b["hot"],
                     t_limit=b["t_nom"], t_lo=b["t_lo"], cont=thermal.continuous(v["rated"], b["k_nom"]), cont_lo=thermal.continuous(v["rated"], b["k_hi"]))
        if mount == "cover" and j != "shoulder_roll":                # the inline yaw / elbow with an Al finned ring through the cover
            b2 = thermal.band(n, x2, mount, True)
            th[j]["ring"] = dict(T=b2["nom"], T_hi=b2["hi"], cont=thermal.continuous(v["rated"], b2["k_nom"]), cont_lo=thermal.continuous(v["rated"], b2["k_hi"]),
                                 t_limit=b2["t_nom"], t_lo=b2["t_lo"])
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
    # basket (R2d) unless quick: equal shares + the 60 / 70 % share study (basket.share_study)
    if not quick:
        dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=basket.PER_HAND))
        _fresh_ik()
        brows, _ = basket.run(dd)
        ok = basket.task_rows(brows)
        r["basket"] = dict(worst=min((x["worst"] for x in ok if x["t"] is not None), default=0.0),
                           all_ik=all(x["t"] is not None for x in ok),
                           tip_wheel=min((x["tip_wheel"] for x in ok if x["t"] is not None), default=float("nan")),
                           rise_max=max((max(x["rise"].values()) for x in ok if x.get("rise")), default=float("nan")))
        _fresh_ik()
        st = basket.share_study(dd, (0.5, 0.6, 0.7))
        r["basket"]["shares"] = {f"{k:.2f}": dict(worst=v["worst"], fails=v["fails"], tip=v["tip"]) for k, v in st.items()}
    r["backdrive"] = backdrive(d)
    r["meff"] = effective_mass(d)
    for enc in (False, True):
        jr, rss = accuracy_rows(d, enc)
        r["acc_enc" if enc else "acc"] = dict(rss=rss, joints={j: dict(deg=e, mm=mm) for j, (e, mm) in jr.items()})
    jr, rss = accuracy_rows(d, False, dual_closes=False)            # 2EC output encoder as absolute reference only
    r["acc_ref"] = dict(rss=rss, joints={j: dict(deg=e, mm=mm) for j, (e, mm) in jr.items()})
    jr, rss = accuracy_rows(d, True, dual_closes=False)             # the encoders the BOM buys (AS5048A / off-axis), 2EC as reference only
    r["acc_bom"] = dict(rss=rss, joints={j: dict(deg=e, mm=mm) for j, (e, mm) in jr.items()})
    return r


def reach_check(d_ref, d_new, seeds=3):
    """R1 coverage (twin.ik_study, collision hulls) of two designs on the study_xml twin: the inline-yaw design drops the
    off-axis cartridge hull and gains the coaxial motor cylinder; the kinematics do not change (the yaw axis lies along the
    upper arm and l_upper is kept, so the elbow, wrist and hand frames are the same for every joint vector)."""
    from . import twin
    lay = {"x": twin.support_polygon("xdrive", d_ref.base.wheel_radius_pos, 0)}
    out = {}
    with corrected_bodies():
        for lab, dd in (("ref", d_ref), ("new", d_new)):
            _fresh_ik()
            rr = twin.ik_study(dd, lay, seeds=seeds)
            out[lab] = {k: v["coverage"] for k, v in rr.items() if not v.get("info")}
    # kinematic identity: the grasp site for random joint vectors
    import mujoco
    rng = np.random.default_rng(0)
    ms = [mujoco.MjModel.from_xml_string(study_xml(x)) for x in (d_ref, d_new)]
    ds = [mujoco.MjData(m) for m in ms]
    err = 0.0
    for _ in range(200):
        q = rng.uniform(-1.5, 1.5, ms[0].nq)
        ps = []
        for m, dat in zip(ms, ds):
            dat.qpos[:] = q
            mujoco.mj_kinematics(m, dat)
            ps.append(dat.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")].copy())
        err = max(err, float(np.linalg.norm(ps[0] - ps[1])))
    out["fk_max_diff_mm"] = err * 1000
    return out


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
        r["mass_lines"] = mass_lines(k) if SET[k] and (SET[k].startswith("inline") or k == "B1y") else {}
        res[k] = r
        print(k, f"arm {r['arm_kg']:.2f} kg, tip {r['tip']['worst'] * 1000:.0f} mm {r['tip']['n_ok']}/16, hug {r['hug']['worst']:.2f}, "
              f"basket {r.get('basket', {}).get('worst', float('nan')):.2f}, cost/arm ${r['cost']['arm']:.0f}", flush=True)
    res["stow"] = stow_overlay(keys)
    if "B1" in keys and "B1y" in keys and not quick:
        res["reach"] = reach_check(V["B1"], V["B1y"])
        print("reach B1 vs B1y", res["reach"], flush=True)
    return res


# --------------------------------------------------------------------------------------------------------------- report
def _fmt_t(t):
    return "unlimited" if math.isinf(t) else f"{t / 60:.0f} min"


def _sd(x):
    """Signed dollars: +$57 / −$12."""
    return f"{'+' if x >= 0 else '−'}${abs(x):,.0f}"


PK = "reach (pitch −60°, elbow −60°)"     # the reference pose of the summary rows


def _me(r, pose=PK, pay="0 kg", where="hand, wrist held", lab="with reflected rotor inertia"):
    return r["meff"][pose][pay][lab][where]


def _inf(x, f="{:.2f}"):
    return "∞ (constrained)" if not math.isfinite(x) else f.format(x)


def report(res, keys):
    b0, b1 = res["B0"], res.get("B1")
    bom_total = json.loads((DOCS / "bom.json").read_text())["total"] if (DOCS / "bom.json").exists() else float("nan")
    L = ["# All-inline arm study (generated by `python -m rnd.inline_study`)", "", __doc__.split("\n\n", 1)[1].strip(), ""]
    hdr = "| | " + " | ".join(NAME[k] for k in keys) + " |"
    L += ["## Summary", "", hdr, "|---|" + "---|" * len(keys)]
    row = lambda lab, f: L.append(f"| {lab} | " + " | ".join(f(res[k], k) for k in keys) + " |")
    ok = lambda b: "✅" if b else "❌"
    row("Pitch / roll / yaw / elbow (actuator × belt)", lambda r, k: ", ".join(f"{n.replace('-2EC', '').replace('_V1.1', '')}{'' if x == 1 else f' ×{x:g}'}" for n, x in r["joints"].values()))
    row("**Arm drives per arm** (4 big joints: actuators + drive parts + joint encoders + thermal caps / rings, $)", lambda r, k: f"**{r['cost']['arm']:,.0f}** ({r['cost']['actuators']:,.0f} + {r['cost']['drive_parts']:,.0f}); dearest listing {r['cost']['arm_hi']:,.0f}")
    row("**Robot, both arms, landed (+15 %)**: Δ vs B0 / Δ vs B1", lambda r, k: f"**{r['cost']['robot_landed']:,.0f}** ({_sd(r['cost']['robot_landed'] - b0['cost']['robot_landed'])} / {_sd(r['cost']['robot_landed'] - b1['cost']['robot_landed']) if b1 else '—'})")
    row(f"Increment applied to the scorecard's ${SCORE_TOTAL:,.0f} (illustration, as Astra #3)", lambda r, k: f"{SCORE_TOTAL + r['cost']['robot_landed'] - b0['cost']['robot_landed']:,.0f}")
    row("One arm, detachable piece (kg)", lambda r, k: f"{r['arm_kg']:.2f}")
    row("Elbow actuator (kg): on the upper arm / on the forearm (where)", lambda r, k: f"{r['elbow_node_kg']:.2f}: {r['elbow_split']['upper']:.2f} / {r['elbow_split']['fore']:.2f} ({r['elbow_split']['where']})")
    row("Mass outboard of the elbow axis (forearm side, kg)", lambda r, k: f"{r['outboard_elbow_kg']:.2f}")
    row("Arm gravity moment straight out, no balancer (N·m): study twin (point-mass first version)", lambda r, k: f"{r['arm_moment_nm']:.1f} ({r['arm_moment_point_nm']:.1f})")
    row("Torso side per arm: pitch module + pitch actuator (kg)", lambda r, k: f"{r['pitch_side_kg']:.2f}")
    row("Robot mass (kg) / COM height parked, arms straight (m)", lambda r, k: f"{r['com']['M']:.1f} / {r['com']['h_parked']:.3f}, {r['com']['h_straight']:.3f}")
    row("**Tip A10** (16 cases, requested rules): worst reserve, cases ≥ 30 mm", lambda r, k: f"**{r['tip']['worst'] * 1000:.1f} mm, {r['tip']['n_ok']}/16** {ok(r['tip']['n_ok'] == 16)}")
    row("Push that tips it, arms parked, 1 m up (N)", lambda r, k: f"{r['push_tip_n']:.0f}")
    row("**20 lb hug at the R2c spec 0.25 m**: rated × pitch / elbow (≥ 1.5)", lambda r, k: f"**{r['hug']['pitch']:.2f} / {r['hug']['elbow']:.2f}** {ok(r['hug']['worst'] >= 1.5)}")
    row("Hug brackets: worst rated × at 0.20 / 0.30 m", lambda r, k: f"{r['hugs']['0.20']['worst']:.2f} / {r['hugs']['0.30']['worst']:.2f}")
    row("**Laundry basket** R2d, equal shares: worst duty margin (≥ 1) / max 2-min rise", lambda r, k: (f"**{r['basket']['worst']:.2f}** {ok(r['basket']['worst'] >= 1)} / {r['basket']['rise_max']:.1f} K" if "basket" in r else "not run (--quick)"))
    row("Basket, heavier hand 60 % / 70 % (seating × 1.3 on the pick): worst margin, failing joints", lambda r, k: (
        " / ".join(f"{r['basket']['shares'][s]['worst']:.2f} {ok(r['basket']['shares'][s]['worst'] >= 1)}" + (f" ({', '.join(r['basket']['shares'][s]['fails'])})" if r['basket']['shares'][s]['fails'] else "")
                   for s in ("0.60", "0.70")) if "basket" in r else "—"))
    row("Workspace margin rated / p95 (pitch, roll, yaw, elbow, wrist)", lambda r, k: ", ".join(f"{v['m_rated']:.2f}" for v in r["wm"].values()))
    row("Joint speed at rated (rpm): pitch / roll / yaw / elbow", lambda r, k: " / ".join(f"{v:.0f}" for v in r["speed_rpm"].values()))
    row("**Whole-joint ratio N** (actuator × belt): pitch / roll / yaw / elbow", lambda r, k: "**" + " / ".join(f"{r['backdrive'][j]['N']:.0f}" for j in BIG) + "**")
    row("**Reflected rotor inertia J_r·N²** (kg·m²; J_r ESTIMATE): pitch / roll / yaw / elbow", lambda r, k: "**" + " / ".join(f"{r['backdrive'][j]['J']:.3f}" for j in BIG) + "**")
    row("Back-drive drag at the joint, efficiency model (N·m, mid; band ×0.5–2 = 4× wide, unverified): pitch / roll / yaw / elbow", lambda r, k: " / ".join(f"{r['backdrive'][j]['T']:.2f}" for j in BIG))
    row(f"**Hand effective mass**, {PK}, wrist held, worst direction (kg): empty hand / 1 kg (R17 bound {M_BOUND:.2f} kg)",
        lambda r, k: f"**{_inf(_me(r)['worst_free'])} / {_inf(_me(r, pay='1 kg')['worst_free'])}** {ok(_me(r)['worst_free'] <= M_BOUND)}")
    row(f"Hand speed that keeps 140 N on 75 N/mm, same pose (m/s): empty / 1 kg", lambda r, k: f"{v_allow(_me(r)['worst_free']):.2f} / {v_allow(_me(r, pay='1 kg')['worst_free']):.2f}")
    row(f"Share of contact normals within {M_BOUND:.2f} kg, same pose, empty hand", lambda r, k: f"{_me(r)['within'] * 100:.0f} %")
    row("**Hand accuracy** (joint RSS, mm): no extra encoders, 2EC as reference only / 2EC closing an outer loop / joint-side encoders everywhere",
        lambda r, k: f"**{r['acc_ref']['rss']:.1f} / {r['acc']['rss']:.1f} / {r['acc_enc']['rss']:.1f}**")
    row("**Hand accuracy with the encoders the BOM buys** (AS5048A on belt joints, off-axis on SteadyWin inline joints; 2EC as reference only / closing a loop, mm)",
        lambda r, k: f"**{r['acc_bom']['rss']:.1f} / {r['acc_enc']['rss']:.1f}**")
    row("Worst 1 kg hold in the BOM mount, yaw: winding nominal (band hi), time to 100 °C", lambda r, k: _th(r, "upper_yaw"))
    row("Same, elbow", lambda r, k: _th(r, "elbow"))
    row("Elbow envelope (mm)", lambda r, k: ENV["elbow"][k])
    row("Yaw envelope (mm)", lambda r, k: ENV["yaw"][k])
    row("R5 door stow W(45° ± 2°), today's pose with the hull swap (mm; ≤ 570)", lambda r, k: f"{res['stow'].get(k, res['stow']['today'])['w45'] * 1000:.0f}" + ("" if k in res["stow"] else " (today's CAD)"))
    row("Drive part count per arm (pitch, yaw, elbow: pieces / fasteners)", lambda r, k: f"{r['simple']['parts']} / {r['simple']['fasteners']}")
    row("Build per arm (h, build_guide model) / print saved per arm (g ESTIMATE, P2S h)", lambda r, k: f"{r['simple']['arm_hours']:.1f} / {r['simple']['print_g_saved']:.0f} g, {r['simple']['print_h_saved_p2s']:.1f} h")
    L += [""]
    L += recommendation(res, keys)
    # ---------------------------------------------------------------- hug
    L += ["", "## 20 lb hug (R2c): the spec point is 0.25 m", "",
          "rnd/payload_study.py pose B (upper arm hanging, forearm horizontal, 4.54 kg per forearm at the distance shown from the elbow axis): "
          "the upper arm hangs, so only the forearm, hand and load make the pitch and elbow moments, and the result does not depend on which "
          "body carries the elbow actuator (both sit on the elbow axis or up the hanging upper arm). Equal shares; unequal shares are the "
          "basket section. Astra #3 checked I2b by hand: 20 / 1.72 = 11.63 N·m at 0.20 m, + 4.54 × 9.81 × 0.05 = 2.23 N·m → 1.44×; this run agrees.", "",
          "| Variant | Elbow demand at 0.25 m (N·m) | 0.20 m: pitch / elbow | **0.25 m: pitch / elbow** | 0.30 m: pitch / elbow |", "|---|---|---|---|---|"]
    for k in keys:
        h = res[k]["hugs"]
        L.append(f"| {k} | {h['0.25']['t']:.2f} | {h['0.20']['pitch']:.2f} / {h['0.20']['elbow']:.2f} | **{h['0.25']['pitch']:.2f} / {h['0.25']['elbow']:.2f}** {ok(h['0.25']['worst'] >= 1.5)} | "
                 f"{h['0.30']['pitch']:.2f} / {h['0.30']['elbow']:.2f} |")
    # ---------------------------------------------------------------- cost
    L += ["", "## Cost detail (per arm, the four big joints)", "", "Unit prices: rnd/m1_kit.py constants where they exist (GIM6010-8 $90, GIM8108-8 $115 ESTIMATE, "
          "GIM8108-36 $240 with the SDC303), the new actuators at the lowest US-seller listing found (actuators.yaml), belt-stage parts from "
          "m1_kit / bom lines or ESTIMATE as marked. Robot landed = 2 arms × (1 + 15 % contingency, rnd/bom.py). A 15 % allowance is not a "
          "delivered quote (Astra #3): compare in-stock motors with the specified drivers, adapters, tax and freight on the same date.", ""]
    if "B1y" in res and b1:
        c1, cy = b1["cost"], res["B1y"]["cost"]
        ring = sum(q * u for j, i, q, u, _ in cy["lines"] if j == "upper_yaw" and "finned ring" in i)
        yaw_b1 = sum(q * u for j, i, q, u, _ in c1["lines"] if j == "upper_yaw")
        yaw_y = sum(q * u for j, i, q, u, _ in cy["lines"] if j == "upper_yaw")
        L += [f"**B1y vs B1 at the yaw, per arm:** B1's GIM6010-8 + 2:1 belt stage (stub, pulley, 6001, belt, flange screws, Al cap + NTC, "
              f"AS5048A) ${yaw_b1:,.2f} → B1y's DM-J4340-2EC + Ø60 adapter ${yaw_y - ring:,.2f} (+ the thermal ring ${ring:,.2f}) = ${yaw_y:,.2f}. "
              f"Robot landed: **{_sd(2 * (yaw_y - ring - yaw_b1) * 1.15)} without the ring (Astra's ≈ +$57), {_sd(cy['robot_landed'] - c1['robot_landed'])} with it**; "
              f"at the dearest J4340 listing (${HI['DM-J4340-2EC']:.0f}) {_sd(cy['robot_landed'] - c1['robot_landed'] + 2 * 1.15 * (HI['DM-J4340-2EC'] - prices()['DM-J4340-2EC'][0]))} with the ring.", ""]
    for k in keys:
        c = res[k]["cost"]
        L += [f"**{NAME[k]}: ${c['arm']:,.0f} per arm** (dearest listings ${c['arm_hi']:,.0f})", "", "| Joint | Item | Qty | $ | Basis |", "|---|---|---|---|---|"]
        L += [f"| {j} | {i} | {q} | {u:,.2f} | {s_} |" for j, i, q, u, s_ in c["lines"]]
        L.append("")
    # ---------------------------------------------------------------- masses
    L += ["## Masses (all-inline variants: deltas from B0's CAD roll-ups; B1y: deltas from B1)", ""]
    for k in keys:
        ml = res[k]["mass_lines"]
        if not ml:
            continue
        L += [f"**{NAME[k]}**", "", "| Param | Item | Δ kg | Basis |", "|---|---|---|---|"]
        for p_, lines in ml.items():
            L += [f"| {p_} | {i} | {m:+.3f} | {b_} |" for i, m, b_ in lines]
            L.append(f"| **{p_}** | **sum** | **{sum(m for _, m, _ in lines):+.3f}** | |")
        L.append("")
    L += ["Arm piece by item (kg):", "", "| Item | " + " | ".join(keys) + " |", "|---|" + "---|" * len(keys)]
    items = [n for n, _ in res[keys[0]]["arm_items"]]
    for n in items:
        L.append(f"| {n} | " + " | ".join(f"{dict(res[k]['arm_items']).get(n, 0):.3f}" for k in keys) + " |")
    L += ["", "**Body assignment (Astra #3 Part A b).** The twin (rnd/twin.py) puts every actuator as a sphere on its joint's child body at the "
          "axis. This study's twin (study_xml) moves each to the body that carries it: the belt elbow motor onto the upper arm "
          "(B0 71 mm, B1/B1y 84 mm above the elbow axis, the belt centre); an inline elbow's housing, stator and rotor "
          f"({F_HOUSING:.0%}, ESTIMATE) onto the upper arm and its output ({1 - F_HOUSING:.0%}) onto the forearm, both on the axis; the belt yaw cartridge onto "
          "the roll output 81 mm behind the yaw axis; an inline yaw's housing onto the roll output coaxially above the yaw interface; the inline roll's "
          "housing onto the pitch body; the wrist pitch's onto the forearm. Each is a cylinder of the motor's frame (own COM and inertia); rotor spin "
          "is the joint armature J_r·N². The tip, basket, workspace-torque, gravity-moment and effective-mass rows use this model; the "
          "gravity moment rises against the first version's point-mass figure mainly because the twin places the yaw stage and bones "
          "farther out than the point list did. The detachable-arm mass alone is the wrong number to minimise (Astra): an inline elbow "
          "adds little moment about the elbow but its full mass to the shoulder.", ""]
    # ---------------------------------------------------------------- back-drive
    L += ["## Back-drive: whole-joint ratio and reflected rotor inertia", "",
          "The useful comparison is the whole joint's ratio N (actuator × belt) and its reflected rotor inertia J_r·N², not 'motor' against "
          "'belt' (Astra #3). For the same rotor, inertia scales with N²: 36:1 against B1's 24:1 at pitch / elbow is 2.25× the reflected "
          "inertia, which is **not** 2.25× the static hand force. The drag column is the efficiency model (motor drag × N / η_b with "
          "η_b = 2 − 1/η_f, + gearbox breakaway, × belt ratio / η_belt + belt drag): **back-driving efficiency is not a breakaway-friction "
          "law** (grease, seals, gear load, belt tension, temperature, brake / phase state and the drive's zero-current behaviour all "
          f"matter), so the table carries an explicit **band of ×{SPREAD[0]}–×{SPREAD[1]} (4× wide)**, and no measurement of these exact "
          "assemblies supports even that band. The hand force uses a fixed lever (pitch / roll 0.60 m, yaw / elbow 0.43 m), which is not "
          "the hand lever in every pose. A comfortable gravity-compensated jog is not evidence of low unpowered drag: the bench measures "
          "residual gravity and friction separately.", "",
          "| Variant | Joint | Actuator | N total | J_r (kg·m², ESTIMATE) | **J_r·N² (kg·m²)** | ×B1's same joint | T drag mid (N·m) | **band ×0.5–2 (N·m)** | hand force at the fixed lever (N, band) |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for k in keys:
        for j in BIG:
            bd = res[k]["backdrive"][j]
            rel = bd["J"] / b1["backdrive"][j]["J"] if b1 else float("nan")
            L.append(f"| {k} | {j} | {bd['act']} | {bd['N']:.0f} | {bd['Jr']:.1e} | **{bd['J']:.4f}** | {rel:.2f} | {bd['T']:.2f} | **{bd['T_lo']:.2f}–{bd['T_hi']:.2f}** | {bd['F']:.1f} ({bd['F_lo']:.1f}–{bd['F_hi']:.1f}) |")
    # ---------------------------------------------------------------- collision
    L += ["", "## Collision: directional effective mass against the R17 bound", "",
          f"Fixed-target spring model (Astra #3): a free effective mass m at speed v on a contact stiffness k peaks at F = v·√(k·m), so the "
          f"R17 limit (**{F_R17:.0f} N transient at the hands**, RND_PLAN R17; not the 280 N the first version used) at v = {V_HAND} m/s on "
          f"k = 75 N/mm gives **m_max = (F/v)²/k = {M_BOUND:.2f} kg**, and a mass m meets it at v = F/√(k·m). m(n) = 1 / (nᵀ J M⁻¹ Jᵀ n) is swept "
          f"over {N_NORMALS} contact normals at the grasp point (all joints free, and with the wrist held by its position loop: the conservative "
          "case) and at the elbow, in five poses, empty hand and 1 kg, with the reflected rotor inertias. 'Worst' = the largest principal "
          f"effective mass below {M_CONSTRAINED:.0f} kg; a principal mass above that is a **constrained direction of the rigid model** (arm "
          "straight along its axis, the elbow pushed along the upper arm), shown as ∞: not a benign finite worst case, it is a direction "
          "the rigid model cannot bound and the structure, the skin compliance and the base motion set. A 1 kg payload alone is above the "
          "bound, so carrying anything at 0.5 m/s near people cannot meet 140 N on a hand-stiffness contact: speed and passive compliance "
          "do more for the first peak than the reducer choice. The peak comes 9–10 ms after contact, before the 14–18 ms detector trip, "
          "so the detector does not shape it. Not covered: base motion, structural / skin compliance, active impedance, pressure and "
          "sustained trapping; an instrumented surrogate, not a person, is the test (a calculation or a current trip is not a safety certification).", "",
          "| Variant | Pose | Hand, wrist held: principal m (kg), empty | 1 kg | **v for 140 N, empty / 1 kg (m/s)** | normals ≤ 1.05 kg (empty) | Hand, all joints free: worst empty / 1 kg | Elbow: principal m (kg) | v for 140 N at the elbow (m/s) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for k in keys:
        for pn in POSES:
            h0, h1 = _me(res[k], pn), _me(res[k], pn, "1 kg")
            f0, f1 = _me(res[k], pn, where="hand"), _me(res[k], pn, "1 kg", where="hand")
            e0 = _me(res[k], pn, where="elbow")
            pr = lambda x: " / ".join(_inf(v) for v in x["principal"])
            L.append(f"| {k} | {pn} | {pr(h0)} | {pr(h1)} | **{v_allow(h0['worst_free']):.2f} / {v_allow(h1['worst_free']):.2f}** | {h0['within'] * 100:.0f} % | "
                     f"{_inf(f0['worst_free'])} / {_inf(f1['worst_free'])} | {pr(e0)} | {v_allow(e0['worst_free']):.2f} |")
    if b1:
        L += ["", f"**Reading.** In the reference pose with an empty hand (wrist held) the worst direction is "
              + ", ".join(f"{k} {_me(res[k])['worst_free']:.2f} kg" for k in keys)
              + f": every variant is over the {M_BOUND:.2f} kg bound, and the speed that meets 140 N is "
              + ", ".join(f"{k} {v_allow(_me(res[k])['worst_free']):.2f}" for k in keys)
              + " m/s, before any uncertainty allowance. The variant spread is the reflected rotor inertia at pitch and elbow; B1y adds the J4340's "
              f"{res['B1y']['backdrive']['upper_yaw']['J']:.4f} kg·m² at yaw against B1's {b1['backdrive']['upper_yaw']['J']:.4f}." if "B1y" in res else ""]
    # ---------------------------------------------------------------- accuracy
    L += ["", "## Accuracy (hand error, joint RSS at 0.6 m)", "",
          "Belt joints: rnd/accuracy.py (GIM backlash ÷ belt ratio + belt play + stretch). Inline joints: reducer backlash (actuators.yaml: "
          "GIM8108-36 / GIM6010-48 15 arcmin from listings, DM-J4340 15 and DM-J8009 10 ESTIMATE) with the motor encoder ÷ N. Three columns "
          "(Astra #3): the Damiao 2EC output encoder as an **absolute reference only** (the loop closes on the motor encoder, so the "
          "reducer backlash stays in the hand error); the 2EC **closing a conservatively tuned outer position loop** around the fast inner "
          "loop, *if* its bandwidth, latency and data path are demonstrated (MIT mode does not close on it just because it is present); "
          "and joint-side encoders everywhere (0.05° after calibration). These are calibrated joint-sensing RSS estimates, not accuracy at a "
          "loaded fingertip: an output encoder observes lost motion, it cannot make a reversal transmit torque before the teeth take up, "
          "nor see deflection downstream of itself. 15 arcmin at 0.60 m is 2.62 mm. Test reversals through zero gravity torque for hunting "
          "and repeatability.", "",
          "| Variant | " + " | ".join(f"{j} (ref / loop / enc)" for j in res[keys[0]]["acc"]["joints"]) + " | RSS ref / loop / enc |", "|---|" + "---|" * (len(res[keys[0]]["acc"]["joints"]) + 1)]
    for k in keys:
        a_, e_, f_ = res[k]["acc"], res[k]["acc_enc"], res[k]["acc_ref"]
        L.append(f"| {k} | " + " | ".join(f"{f_['joints'][j]['mm']:.1f} / {a_['joints'][j]['mm']:.1f} / {e_['joints'][j]['mm']:.1f}" for j in a_["joints"])
                 + f" | **{f_['rss']:.1f} / {a_['rss']:.1f} / {e_['rss']:.1f}** |")
    # ---------------------------------------------------------------- thermal
    L += ["", "## Thermal: worst 1 kg static hold in the mount the BOM pays for (rnd/thermal.py network + its sensitivity band)", "",
          "Belt joints: closed sleeve + aluminium finned rear cap; inline joints bare under the R17 cover (thermal.mount_of). Band = every corner "
          "of h_o 8–16 W/m²K, h_i 3–8, cap contact 0.25–2 K/W, F_WC 20–40 %, local air +3…10 K (thermal.BAND), 30 °C room. The inline yaw / elbow "
          "also show the option of an aluminium finned ring on the rear face through the cover (costed in the inline yaw rows). The hug and "
          "basket duties, D20 / B1y, are in thermal.md section 3.", "",
          "| Variant | " + " | ".join(f"{j}: worst / continuous nominal–band (N·m), winding nominal (band hi), to 100 °C" for j in BIG) + " |", "|---|" + "---|" * 4]
    for k in keys:
        t = res[k]["thermal"]
        L.append(f"| {k} | " + " | ".join(_thc(t[j]) for j in BIG) + " |")
    # ---------------------------------------------------------------- envelope, reach
    L += ["", "## Envelope, reach and door stow", "", "| Joint | " + " | ".join(keys) + " |", "|---|" + "---|" * len(keys)]
    for jn, v in ENV.items():
        L.append(f"| {jn} | " + " | ".join(v[k] for k in keys) + " |")
    st = res["stow"]
    L += ["", f"Door stow (rnd/stow.py pose for 45°, today {st['today']['w45'] * 1000:.0f} mm, set by {st['today']['sets'][0]} and {st['today']['sets'][1]}): "
          "the inline variants take out the off-axis yaw cartridge (and, all-inline, the elbow cartridge) hulls and add the motor cylinders with "
          "wall + cover allowance; the pose is not re-optimised and the other hulls are today's. A hull substitution in one old pose with "
          "3 mm to the limit, not a swept, wired, guarded assembly (Astra #3).", ""]
    if "reach" in res:
        rc = res["reach"]
        L += ["**B1y yaw drop (+15 mm) against reach and dexterity.** The yaw axis lies along the upper arm and l_upper is kept (the neck "
              f"shortens), so the joint-to-hand kinematics are identical: the grasp site of 200 random joint vectors differs by "
              f"{rc['fk_max_diff_mm']:.3f} mm between B1 and B1y, and the dexterity map (rnd/dexterity.py, kinematic, sw/bench_env) cannot change. "
              "What can change is collision: R1 coverage on the study twin (twin.ik_study, CAD hulls; B1y without the off-axis yaw cartridge hull, "
              "with a Ø92 × 56 coaxial motor cylinder, upper-arm hulls kept on the elbow):", "",
              "| R1 target | B1 | B1y |", "|---|---|---|"]
        L += [f"| {t} | {rc['ref'][t] * 100:.0f} % | {rc['new'][t] * 100:.0f} % |" for t in rc["ref"]]
        L += ["", "The coaxial motor sits 15 mm lower on the arm than the yaw block's old bottom; its envelope against the torso at full "
              "adduction and the yaw harness / clock spring routing need the CAD (no inline-yaw CAD yet). The extra drop is inside the "
              "+15–25 mm Astra named; at +25 mm the kinematics are still unchanged and only the hull clearance moves.", ""]
    # ---------------------------------------------------------------- simplicity
    L += ["## Simplicity", "", "| Variant | Belt joints | Inline joints | Drive pieces | Drive fasteners | Drive build min | Arm build h | Print saved (g ESTIMATE / P2S h) |", "|---|---|---|---|---|---|---|---|"]
    for k in keys:
        s_ = res[k]["simple"]
        L.append(f"| {k} | {', '.join(s_['belt_joints']) or '—'} | {', '.join(s_['inline_joints']) or 'roll only'} | {s_['parts']} | {s_['fasteners']} | "
                 f"{s_['drive_minutes']} | {s_['arm_hours']:.1f} | {s_['print_g_saved']:.0f} / {s_['print_h_saved_p2s']:.1f} |")
    L += ["", "Counts: a D26 belt joint is the cartridge (can, rear cap, Al cap, NTC), the stub + key + pulley + 6001 + bushing, the belt and "
          "the shaft-end encoder (+ the pitch idler, + the elbow cassette and seat caps); D26's fixed centres removed the eccentric bushings "
          "(belts.md), so they are no longer counted (Astra #3). An inline joint is the actuator, a metal output adapter, a printed seat and a "
          "far-side bearing (+ an off-axis encoder for a SteadyWin, + the thermal ring at the yaw). Build minutes use rnd/build_guide.py's "
          "9 min cartridge and 15 min stage; inline heavy joint 25 min, yaw 15, encoder 7 / off-axis 10 (ESTIMATE). The print saving is "
          "print_plan.md rows per piece, an ESTIMATE until a printed-only mass ledger of the replacement parts exists.", ""]
    L += bench()
    return "\n".join(L) + "\n"


SCORE_TOTAL = 5845.0        # docs/rnd/SCORECARD.md robot total quoted by Astra #3 (illustration only)


def _th(r, j):
    t = r["thermal"][j]
    lim = "unlimited" if t["t_limit"] is None or math.isinf(t["t_limit"]) else f"{t['t_limit'] / 60:.0f} min"
    s_ = f"{t['T']:.0f} °C ({t['T_hi']:.0f}), {lim}" + (" ✅" if t["T_hi"] <= 100 else (" ⚠️" if t["T"] <= 100 else " ❌"))
    if "ring" in t:
        s_ += f"; with the ring {t['ring']['T']:.0f} °C ({t['ring']['T_hi']:.0f})"
    return s_


def _thc(t):
    lim = lambda x: "unlimited" if x is None or math.isinf(x) else f"{x / 60:.0f} min"
    s_ = f"{t['max']:.1f} / {t['cont']:.1f}–{t['cont_lo']:.1f}, {t['T']:.0f} °C ({t['T_hi']:.0f}), {lim(t['t_limit'])} ({t['mount']})"
    if "ring" in t:
        s_ += f"; ring: {t['ring']['cont']:.1f}–{t['ring']['cont_lo']:.1f}, {t['ring']['T']:.0f} °C ({t['ring']['T_hi']:.0f})"
    return s_


def recommendation(res, keys):
    need = ("B0", "B1", "B1y", "I1", "I2a", "I2b")
    L = ["## Recommendation", ""]
    if not all(k in res for k in need) or "basket" not in res["B1"]:
        return L + ["(run all six variants without --quick for the recommendation)"]
    b0, b1, by, i1, i2a, i2b = (res[k] for k in need)
    d = lambda a, b: a["cost"]["robot_landed"] - b["cost"]["robot_landed"]
    th_y, th_1 = by["thermal"]["upper_yaw"], b1["thermal"]["upper_yaw"]
    ring = th_y.get("ring", {})
    rc = res.get("reach", {})
    r1 = (min(rc["new"][t] - rc["ref"][t] for t in rc["ref"]) * 100) if rc else float("nan")
    L += [f"- **The 20 lb hug at the spec 0.25 m:** B0 {b0['hug']['worst']:.2f}×, B1 {b1['hug']['worst']:.2f}×, B1y {by['hug']['worst']:.2f}×, "
          f"I1 {i1['hug']['worst']:.2f}×, I2a {i2a['hug']['worst']:.2f}×, **I2b {i2b['hug']['worst']:.2f}×** (its 20 N·m J8009 elbow: "
          f"{'below' if i2b['hug']['worst'] < 1.5 else 'above'} the 1.5× rule; the first version's 1.72× was at 0.20 m). B1 and B1y share the hug "
          "(the forearm is the same) and pass it only just, with equal shares and before thermal derating.",
          f"- **All-inline is not recommended** (Astra #3 Part A, and this rerun): I1 / I2a miss A10 ({i1['tip']['n_ok']}/16, {i2a['tip']['n_ok']}/16), "
          f"I2b fails the hug at 0.25 m, and every all-inline variant costs {_sd(d(i1, b1))}–{_sd(d(i2b, b1))} landed over B1 for more reflected "
          f"rotor inertia at the hand (reach pose, empty, wrist held: B1 {_me(b1)['worst_free']:.2f} kg → I1 {_me(i1)['worst_free']:.2f}, I2b "
          f"{_me(i2b)['worst_free']:.2f} kg). Keep the inline roll; D20 belts at pitch and elbow.",
          f"- **B1y against B1.** Cost **{_sd(d(by, b1))} landed** ({_sd(d(by, b1) - 2 * 1.15 * sum(q * u for j, i, q, u, _ in by['cost']['lines'] if 'finned ring' in i))} "
          f"before the thermal ring: Astra's ≈ +$57 holds). Arm piece {b1['arm_kg']:.2f} → {by['arm_kg']:.2f} kg; tip {b1['tip']['worst'] * 1000:.1f} → "
          f"{by['tip']['worst'] * 1000:.1f} mm ({by['tip']['n_ok']}/16); basket equal {b1['basket']['worst']:.2f} → {by['basket']['worst']:.2f}, "
          f"70/30 {b1['basket']['shares']['0.70']['worst']:.2f} → {by['basket']['shares']['0.70']['worst']:.2f} (pitch limits both); hand effective mass "
          f"{_me(b1)['worst_free']:.2f} → {_me(by)['worst_free']:.2f} kg (v for 140 N {v_allow(_me(b1)['worst_free']):.2f} → {v_allow(_me(by)['worst_free']):.2f} m/s); "
          f"yaw J_r·N² {b1['backdrive']['upper_yaw']['J']:.4f} → {by['backdrive']['upper_yaw']['J']:.4f} kg·m² (40:1 two-stage against 16:1); "
          f"accuracy with the encoders each BOM buys: B1 {b1['acc_bom']['rss']:.1f} mm (AS5048A on the yaw shaft) → B1y {by['acc_bom']['rss']:.1f} mm with the 2EC as "
          f"an absolute reference only (the J4340's ≈ 15 arcmin backlash stays in the loop), {by['acc_enc']['rss']:.1f} mm only if an outer loop can be closed on the 2EC "
          f"(not yet demonstrated: a regression until it is); R1 coverage change {r1:+.0f} points at worst "
          f"(kinematics identical, FK diff {rc.get('fk_max_diff_mm', float('nan')):.3f} mm); yaw block 175 × 92 → Ø92 plan with +15 mm drop; "
          f"drive pieces {b1['simple']['parts']} → {by['simple']['parts']}, one belt stage and its tension check fewer (D26's 0–156 N yaw spread gone). "
          f"**Thermal is the catch:** the bare J4340 under the cover holds the worst 1 kg yaw pose at {th_y['T']:.0f} °C nominal ({th_y['T_hi']:.0f} band hi; "
          f"continuous {th_y['cont']:.1f}–{th_y['cont_lo']:.1f} N·m against {th_y['max']:.1f}) and reaches 100 °C in "
          f"{'—' if th_y['t_limit'] is None else '%.0f' % (th_y['t_limit'] / 60)} min, where B1's capped GIM6010 sits at {th_1['T']:.0f} °C; with an aluminium "
          f"finned ring through the cover {ring.get('T', float('nan')):.0f} °C ({ring.get('T_hi', float('nan')):.0f} band hi), which is why the ring is costed.",
          f"- **Recommendation: B1 is the build path; B1y is the preferred refinement, adopted only on a returnable J4340 sample** that shows "
          "(1) delivered price with the 24 V driver, (2) output-bearing support for the yaw's moment, (3) dual-encoder operation (output angle "
          "over power cycles; whether an outer loop can close on it), (4) the yaw thermal in a mock cover with the ring (winding rise at rated "
          f"≤ the thermal.md section 3 'bench must show' figure), and (5) the +15 mm drop against the torso / harness in CAD. "
          "B1y earns its place on packaging (the 175 mm cartridge bay and the yaw belt's fixed-centre tension spread go) and an absolute yaw "
          "output angle, not on load, tip or impact, where it is equal or marginally worse. If the sample fails any of these, B1 stays as is: no "
          "decision on the yaw blocks the D20 Step-1 test. If the D20 24 V test fails, reopen payload / gearing / budget; do not revert to B0 "
          "and keep a 20 lb rating.",
          "", "**Not covered (no inline CAD):** the inline yaw's seat, output support and harness / clock-spring routing under the roll motor; the "
          "inline pitch motor next to the balancer (all-inline only: the biggest packaging risk); the elbow fold with a Ø105 knee; the stiffness / "
          "FEA of new load paths; the R17 pinch sweep; the 24 V speed of the Damiao parts under load."]
    return L


def bench():
    return ["## What the bench must measure to confirm it", "",
            "1. **Back-drive torque of the whole joint, not the bare motor:** B1's GIM8108-8 + 3:1 stage first, then the roll sample and a "
            "returnable DM-J4340-2EC (Astra #3: borrow before buying mixed drives). Horizontal-axis or gravity-balanced fixture, calibrated "
            "lever / load cell, independent output angle: breakaway and slow running torque both ways, torque–angle hysteresis at several loads, "
            "cold and hot, phases open / drive disabled / zero-torque mode. This replaces the efficiency model and its 4× band.",
            "2. **Output inertia** from torque / acceleration with two known added inertias (fixture inertia and friction subtracted) → J_r·N² "
            "and the effective-mass rows.",
            "3. **Backlash and reversal:** motor angle, output angle and actual link motion through reversal at ± 1 N·m and through zero "
            "gravity torque; encoder zero over power cycles; whether a 2EC outer loop hunts.",
            "4. **Thermal in the final mount** (thermal.md section 3 bench list): winding rise at rated torque in the closed sleeve + cap and, "
            "for B1y, the J4340 bare and with the ring under a mock cover; cap contact resistance; h_o with vents open / blocked; τ.",
            "5. **Torque at 24 V** and speed under load, at full and depleted pack voltage including harness drop.",
            "6. **Impact on an instrumented compliant target** at progressively increasing speed: peak force, rise time, post-trip force: the "
            f"m_max = (F/v)²/k bound ({M_BOUND:.2f} kg at 0.5 m/s) is a model, the surrogate is the test.",
            "7. **Basket share:** tethered basket with load cells under each handle: pick / carry / set-down with 50/50, 60/40, 70/30 and one hook "
            "released, to calibrate the joint-torque share estimate (basket.md rule) and the seating factor."]


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
    ap.add_argument("--brief", action="store_true", help="also rewrite docs/review/inline_study_question.md (the review-3 question)")
    a = ap.parse_args(argv)
    keys = a.only or KEYS
    res = run(a.quick, keys)
    md = report(res, keys)
    (DOCS / "inline_study.md").write_text(md)
    (DOCS / "inline_study.json").write_text(json.dumps(_clean(res), indent=1))
    if a.brief and set(KEYS) <= set(keys):
        REVIEW.mkdir(parents=True, exist_ok=True)
        (REVIEW / "inline_study_question.md").write_text(brief(res, keys))
    print(md[:6000])


if __name__ == "__main__":
    main()
