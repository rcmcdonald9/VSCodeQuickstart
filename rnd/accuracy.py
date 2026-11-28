"""Hand positioning error budget: what each sensor buys (owner: favour accuracy and dexterity over cost).  python -m rnd.accuracy

Static hand error at 0.6 m reach, root-sum-square of the per-joint angular errors × lever arm, plus bone and
mast bending. Error sources (ESTIMATEs, refine with the coupon/bench tests):
- motor encoder resolution (GIM: 14-bit on the motor, ×reduction)
- planetary backlash inside the GIM (≈ 0.2° at its output, typical for small planetary QDDs) ÷ belt ratio
- belt backlash/tooth play (HTD 5M ≈ 0.15° at the joint) and belt stretch under the holding load
- KX seat repeatability (≈ 0.01° kinematic; negligible), bone bending (rnd/bones.py), mast sway (rnd/mast.py)
A **joint-side absolute encoder** (AS5048A, 14-bit, on the joint hub) measures *after* the gearbox and belt, so the
servo loop removes backlash and belt stretch from the static error. What remains is the encoder (±0.05° after
calibration) and the unmeasured bending.
"""
from __future__ import annotations

import math

from .bones import score as bones_score
from .mast import score as mast_score
from .params import BASELINE

REACH = 0.60
BELT_EA = 1.5e5          # N, HTD 5M 15 mm effective cord stiffness EA (ESTIMATE)
GIM_BACKLASH = 0.20      # deg at the GIM output (ESTIMATE)
BELT_PLAY = 0.15         # deg at the joint (ESTIMATE)
# Joint-side encoder after a calibration table (ESTIMATE), params PIP_JOINT_ENCODER (D29 #6, cost_down.md). Datasheet INL
# before calibration: AS5048A ±0.8° (optimum magnet placement, 25 °C; ±1.2° over temperature and displacement), output noise
# 0.06° rms (2.73 LSB); MT6701 ±1.0° typ / ±1.5° max (MagnTek MT6701 datasheet; one revision copy says ±0.5°), no noise figure
# found. The calibrated residual is taken to scale with the INL the table has to remove: MT6701 0.05 × 1.0 / 0.8 ≈ 0.06°,
# band 0.05 (as good as the AS5048A) … 0.10° (the ±1.5° max over temperature, which a room-temperature table does not remove).
ENCODERS = {"as5048a": dict(cal=0.05, band=(0.05, 0.05), name="AS5048A (SPI)"),
            "mt6701": dict(cal=0.05 * 1.0 / 0.8, band=(0.05, 0.10), name="MT6701 (SSI)")}
from .params import JOINT_ENCODER  # noqa: E402
JOINT_ENC = ENCODERS[JOINT_ENCODER]["cal"]   # deg, after calibration (ESTIMATE); 0.05 = the AS5048A figure used since D9
ROLL_ENC = 0.05          # deg: the inline roll's off-axis ring encoder (bom.py line), not part of the D29 #6 swap (ESTIMATE)
# Inline GIM8108-36 roll (D24, no belt): a two-stage 36:1 planetary with the encoder on the motor side. Reducer backlash
# "15 arcmin" in the reseller spec tables (openelab / rcdrone GIM8108-36 listings, 2026-11-27 search snapshots; no maker
# datasheet); Fable review M1: two-stage planetaries of this class are typically 10–20 arcmin. ESTIMATE until the bench
# measures it. The roll torque crosses zero in the arm-down pose, so the whole backlash is traversed: counted in full.
PLANETARY_BACKLASH = {"GIM8108-36": 15 / 60}          # deg at the output
BACKLASH_RANGE = (10 / 60, 20 / 60)
MOTOR_ENC = 360 / 2 ** 14                               # deg at the motor (14-bit), ÷ reduction
LEVER = {"shoulder_pitch": REACH, "shoulder_roll": REACH, "upper_yaw": 0.45, "elbow": 0.43, "wrist_pitch": 0.15}


def joint_errors(d=BASELINE, joint_encoders=False, backlash=None, enc=None):
    """enc: the joint-side encoder's calibrated error (deg); default JOINT_ENC (params PIP_JOINT_ENCODER)."""
    from .belts import STAGES, pd, sustained_motor_torque
    backlash = backlash or {}
    JE = JOINT_ENC if enc is None else enc
    belt = {j: (r, z, c) for j, r, z, c, w in STAGES}
    out = {}
    for j, lev in LEVER.items():
        name, ratio = d.arm.joints[j]
        if joint_encoders and j in belt:
            e = JE
        elif j in belt:
            r, z, c = belt[j]
            pull = sustained_motor_torque(j, ratio, d) * 1000 / (pd(z) / 2)          # N
            stretch = pull * (c / 1000) / BELT_EA                                   # m
            ang_stretch = math.degrees(stretch / (pd(z * r) / 2000))
            e = math.sqrt((GIM_BACKLASH / ratio) ** 2 + BELT_PLAY ** 2 + ang_stretch ** 2)
        elif name in PLANETARY_BACKLASH:                                             # inline GIM: no belt, motor-side encoder
            e = ROLL_ENC if joint_encoders else math.hypot(backlash.get(name, PLANETARY_BACKLASH[name]), MOTOR_ENC / 36)
        else:
            e = 0.05                                                                 # DM-J4310-2EC: output-side absolute encoder
        out[j] = (e, math.radians(e) * lev * 1000)                                   # deg, mm at the hand
    return out


def hand_error(d=BASELINE, joint_encoders=False, backlash=None, enc=None):
    je = joint_errors(d, joint_encoders, backlash, enc)
    bend = bones_score(d)[1]
    sway = mast_score(d)[0]
    rss = math.sqrt(sum(mm ** 2 for _, mm in je.values()))
    return rss, bend, sway, je


def report(d=BASELINE):
    L = ["# Hand positioning accuracy budget (auto-generated by `python -m rnd.accuracy`)", "",
         __doc__.split("\n\n", 1)[1].strip(), "",
         "| Joint | Without joint encoders: error (°) → at hand (mm) | With joint-side encoders |", "|---|---|---|"]
    a, b = joint_errors(d, False), joint_errors(d, True)
    L += [f"| {j} | {a[j][0]:.2f}° → {a[j][1]:.1f} mm | {b[j][0]:.2f}° → {b[j][1]:.1f} mm |" for j in LEVER]
    r0, bend, sway, _ = hand_error(d, False)
    r1, *_ = hand_error(d, True)
    lo = hand_error(d, False, {"GIM8108-36": BACKLASH_RANGE[0]})[0]
    hi = hand_error(d, False, {"GIM8108-36": BACKLASH_RANGE[1]})[0]
    roll = d.arm.joints["shoulder_roll"][0]
    L += [f"| **Joint RSS** | **{r0:.1f} mm** | **{r1:.1f} mm** |", "",
          (f"**Inline roll ({roll}, Fable review M1):** until 2026-11-27 every joint without a belt was given 0.05° as if it were a "
           f"DM-J4310 with an output-side encoder, so the inline roll showed 0.5 mm with or without joint encoders. It now carries its "
           f"36:1 planetary backlash, {PLANETARY_BACKLASH[roll] * 60:.0f} arcmin (reseller spec tables; ESTIMATE): without joint encoders the "
           f"hand error is **{r0:.1f} mm** ({lo:.1f}–{hi:.1f} mm for 10–20 arcmin), against 3.4 mm before. With a joint-side encoder on the roll "
           f"the static error is the encoder's again ({r1:.1f} mm), but the loop then has the backlash inside it: expect a limit cycle "
           "or a dead band around the arm-down pose where the roll torque crosses zero, and the encoders still have no mount or firmware "
           "(Fable M3). Bench step: measure the roll backlash with a dial gauge at 0.3 m before relying on either number.")
          if roll in PLANETARY_BACKLASH else "", "",
          f"Plus unmeasured bending: bones {bend:.1f} mm, mast {sway:.1f} mm (worst static, 1 kg).",
          "The wrist camera closes the last loop on the object, so those terms matter for blind or fast moves, not for visually-servoed grasps.",
          "", "**D29 #6 (cost_down.md, PIP_JOINT_ENCODER; this page uses " + ENCODERS[JOINT_ENCODER]["name"] + "):** hand error with joint encoders, "
          "calibrated: " + "; ".join(f"{ENCODERS[k]['name']} {v[3]:.3f}° → **{v[0]:.1f} mm** (band {v[1]:.1f}–{v[2]:.1f} mm)" for k, v in encoder_compare(d).items())
          + ". The wrist pitch keeps its DM-J4310 output encoder (0.05°) either way.",
          "", "**Decision (D9):** fit AS5048A joint-side encoders on the 8 arm belt joints and the lift. "
          "That's about $140 including 3 small CAN encoder nodes. Precedent: link-side encoders on Franka and other research arms."]
    return "\n".join(L)


def encoder_compare(d=BASELINE):
    """D29 #6: hand error with joint encoders for each encoder choice: {name: (nominal mm, band lo mm, band hi mm, deg)}."""
    return {k: (hand_error(d, True, enc=v["cal"])[0], hand_error(d, True, enc=v["band"][0])[0], hand_error(d, True, enc=v["band"][1])[0], v["cal"])
            for k, v in ENCODERS.items()}


def score(d=BASELINE):
    return hand_error(d, True)[0], hand_error(d, False)[0]


if __name__ == "__main__":
    print(report())
