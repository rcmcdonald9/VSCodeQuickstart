"""Slim-arm study (owner, 2026-11-27: "the motor casings look much bigger than other robots"; bolting the motors is OK).
python -m rnd.slim_study

Variants (all with the R2c/D20 torque requirement):
- **V0** today: belt stages at pitch / roll / yaw / elbow, every cartridge on a KX twist-lock socket; GIM8108-8 at pitch +
  elbow (D20), GIM6010-8 at roll + yaw, DM-J4310 wrist pitch on a KX56.
- **V1** V0 with every cartridge **bolted** (6 × M4 through a flange into heat-set inserts) instead of a KX socket.
- **V2** V1 + **inline GIM8108-36** (36:1, 35 N·m rated, 103 N·m peak, ≈ 1 kg, 24 V SDC303) on the shoulder pitch and roll
  axes: no belt, no big pulley there.

Geometry and mass deltas are derived from the CAD dimensions (rnd/parts/kx.py, roll.py, arm.py) and marked ESTIMATE
where they depend on an unpublished drawing (the GIM8108 frame diameter). Torque checks re-run the existing models with
the variant's masses and geometry: whole-workspace margins (spring.py), the 20 lb hug (payload_study.py) and the laundry
basket (basket.py, full 3D twin statics).
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
KX_SAVE = 0.065          # kg per KX74 socket replaced by a bolted flange (hub + collar + retainer 85 g → flange 20 g; arm.masses)
KX56_SAVE = 0.040


def variants():
    from .params import BASELINE as B, act
    a = B.arm
    j0 = dict(a.joints); j0["shoulder_pitch"] = ("GIM8108-8", 3.0); j0["elbow"] = ("GIM8108-8", 3.0)
    v0 = dataclasses.replace(B, arm=dataclasses.replace(a, joints=j0))
    v1 = dataclasses.replace(B, arm=dataclasses.replace(
        a, joints=j0, roll_stage_mass=a.roll_stage_mass - KX_SAVE, yaw_stage_mass=a.yaw_stage_mass - KX_SAVE,
        bone_mass_upper=a.bone_mass_upper - KX_SAVE, pitch_module_mass=a.pitch_module_mass - KX_SAVE,
        gripper_mech_mass=a.gripper_mech_mass - KX56_SAVE))
    j2 = dict(j0); j2["shoulder_pitch"] = ("GIM8108-36", 1.0); j2["shoulder_roll"] = ("GIM8108-36", 1.0)
    # V2 roll stage: the 80T pulley, D13 stage and the roll cartridge's offset go; the motor sits on the roll axis in the
    # roll link (its mass is counted as the roll actuator). ROLL_Y: ferrule face 18 mm + motor radius 50 (ESTIMATE) + 2.
    # yaw_drop: the top yaw plate no longer clears a Ø127 pulley, only the motor (r ≈ 50): 113 → ≈ 100 mm.
    v2 = dataclasses.replace(B, arm=dataclasses.replace(
        v1.arm, joints=j2, roll_offset_y=0.070, yaw_drop=0.100, roll_stage_mass=v1.arm.roll_stage_mass - 0.25,
        pitch_module_mass=v1.arm.pitch_module_mass - 0.20))
    return {"V0 today (KX sockets, belts)": v0, "V1 bolted cartridges": v1, "V2 bolted + inline GIM8108-36 at pitch and roll": v2}


ENVELOPE = {   # mm, from the CAD (V0) and the deltas above (ESTIMATE for V1/V2)
    "V0 today (KX sockets, belts)": dict(shoulder_w=172, shoulder_h=210, cartridge_d=113, elbow_w=110, stage_len=64),
    "V1 bolted cartridges": dict(shoulder_w=172, shoulder_h=190, cartridge_d=90, elbow_w=98, stage_len=50),
    "V2 bolted + inline GIM8108-36 at pitch and roll": dict(shoulder_w=128, shoulder_h=120, cartridge_d=90, elbow_w=98, stage_len=50),
}
NOTES = {
    "V0 today (KX sockets, belts)": "tool-free cartridge swap (1 collar twist)",
    "V1 bolted cartridges": "swap = 6 screws; −40 min build per arm (no D15 sockets); wrist KX56 gone, so D21's lift-off issue at the wrist goes with it",
    "V2 bolted + inline GIM8108-36 at pitch and roll": "shoulder roll dome shrinks the most (no Ø127 pulley); 2.25× reflected inertia at pitch/roll (36² vs 24²): stiffer to back-drive by hand; +$117 per arm",
}


def run():
    from . import basket, payload_study, spring
    from .budget import mass_budget
    from .params import act
    out = {}
    for name, d in variants().items():
        wm = spring.workspace_margins(d)
        rows, cap = payload_study.run(d)
        hug = next(r for r in rows if r["case"].startswith("B: 20 lb two-handed hug") and "0.20" in r["case"])
        dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=basket.PER_HAND))
        brows, _ = basket.run(dd)
        bas = min((r["worst"] for r in brows if r["t"] is not None and r["x"] <= 0.45 and "washer" not in r["pose"] and "0.80" not in r["pose"]), default=0)
        arm_kg = sum(m for _, m in mass_budget(d)["arm (each, detachable)"])
        sp = {j: act(n)["speed"] / r * 60 / 6.2832 for j, (n, r) in d.arm.joints.items() if j in ("shoulder_pitch", "shoulder_roll")}
        cost = sum(act(n)["price"] for n, _ in d.arm.joints.values())
        out[name] = dict(wm={k: round(v["m_rated"], 2) for k, v in wm.items()}, hug=min(hug["rated"]["shoulder_pitch"], hug["rated"]["elbow"]),
                         basket=bas, arm_kg=arm_kg, speed_rpm=sp, act_cost=cost, env=ENVELOPE[name], note=NOTES[name])
        print(name, out[name], flush=True)
    return out


def main():
    res = run()
    names = list(res)
    L = ["# Slim-arm study (generated by `python -m rnd.slim_study`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         "| | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    row = lambda label, f: L.append(f"| {label} | " + " | ".join(f(res[n]) for n in names) + " |")
    row("Shoulder dome width / height (mm)", lambda r: f"{r['env']['shoulder_w']} / {r['env']['shoulder_h']}")
    row("Motor bump diameter (collar or flange, mm)", lambda r: f"{r['env']['cartridge_d']}")
    row("Elbow housing width (mm)", lambda r: f"{r['env']['elbow_w']}")
    row("One arm, detachable piece (kg)", lambda r: f"{r['arm_kg']:.2f}")
    row("Workspace margin, rated / p95 (pitch, roll, yaw, elbow, wrist)", lambda r: ", ".join(f"{v:.2f}" for v in r["wm"].values()))
    row("20 lb hug, worst of pitch/elbow (rated ×, load 0.2 m out)", lambda r: f"{r['hug']:.2f}×")
    row("Laundry basket (floor pick, carry, bed): worst duty margin (≥ 1 passes)", lambda r: f"{r['basket']:.2f} {'✅' if r['basket'] >= 1 else '❌'}")
    row("Joint speed at rated, pitch / roll (rpm)", lambda r: " / ".join(f"{v:.0f}" for v in r["speed_rpm"].values()))
    row("Actuators per arm ($)", lambda r: f"{r['act_cost']:.0f}")
    row("Notes", lambda r: r["note"])
    (DOCS / "slim_study.md").write_text("\n".join(L))
    (DOCS / "slim_study.json").write_text(json.dumps(res, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
