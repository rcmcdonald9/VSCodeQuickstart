"""Slim-arm study (owner, 2026-11-27: "the motor casings look much bigger than other robots"; bolting the motors is OK).
python -m rnd.slim_study

Variants (all with the R2c/D20 torque requirement: GIM8108-8 at shoulder pitch + elbow):
- **V0** before wave 1: belt stages at pitch / roll / yaw / elbow, every cartridge on a KX twist-lock socket, GIM6010-8 at
  roll + yaw, DM-J4310 wrist pitch on a KX56.
- **V1** every cartridge **bolted** (D24, adopted: 6 × M4 through a Ø68 flange into heat-set inserts, Oldham drive), belt roll.
- **V1 + inline roll** (D24 proposal, pending the owner): V1 with an **inline GIM8108-36** (36:1, 35 N·m rated, 24 V SDC303)
  on the shoulder-roll axis, no roll belt (`PIP_ROLL_DRIVE=inline`).
- **V2** V1 + inline GIM8108-36 on shoulder pitch **and** roll.

Envelope rows are **measured in the wave-1 CAD** where it exists (wave 1 built the bolted pitch / elbow / wrist, the 98 mm
elbow housing and both roll drives): the motor bump (KX74 collar vs bolted sleeve, rnd/parts/kx.py, cartridge_m.py), the
stage length (belt_stage V2 vs V3), the elbow housing (arm.HOUSING) and the shoulder dome (roll.dome() via
docs/rnd/roll_inline.json). Still ESTIMATE, and marked: the belt-roll dome with a bolted roll cartridge (roll.py still draws
the KX74 roll socket), the GIM8108 frames (unpublished drawings) and V2's inline pitch (no CAD). Masses: params (the wave-1
CAD roll-up), with the KX sockets added back for V0 and taken off the roll / yaw stages for V1 (ESTIMATE until roll.py is
bolted). Torque checks re-run the existing models with each variant's masses and geometry: whole-workspace margins
(spring.py), the 20 lb hug (payload_study.py) and the laundry basket (basket.py, full 3D twin statics).
"""
from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
KX_SAVE = 0.065          # kg per KX74 socket replaced by a bolted flange (hub + collar + retainer 85 g → flange 20 g; arm.masses)
# Wave-1 mass changes from bolting (params comments, CAD roll-ups): what V0 carried on top of today's params
V0_ADD = dict(pitch_module_mass=0.03, bone_mass_upper=0.06, bone_mass_fore=0.03, gripper_mech_mass=0.02)
# Unit prices for the cost row (rnd/m1_kit.py: sourced snapshots, or ESTIMATE within sourced bounds for the GIM8108s)
PRICE = {"GIM6010-8": 90.0, "GIM8108-8": 115.0, "GIM8108-36": 215.0, "DM-J4310-2EC_V1.1": 150.0, "STS3215-C018": 19.5}
ROLL_BELT_PARTS = 7.0 + 12.0 + 9.0 + 2 * 2.02 + 2 * 0.9      # 18T alu pulley, Oldham, 475 mm belt, 2 × 61808, 2 × 6801 (m1_kit hold B)
INLINE_PARTS = 3.0                                            # 61806 rear roll support
NAMES = ["V0 before wave 1 (KX sockets, belts)", "V1 bolted cartridges (D24, belt roll)",
         "V1 + inline GIM8108-36 at roll (D24 proposal)", "V2 bolted + inline GIM8108-36 at pitch and roll"]


def variants():
    from .params import BASELINE as B, with_roll_drive
    a = with_roll_drive(B, "belt").arm
    j = dict(a.joints); j["shoulder_pitch"] = ("GIM8108-8", 3.0); j["elbow"] = ("GIM8108-8", 3.0)
    v0 = dataclasses.replace(B, arm=dataclasses.replace(a, joints=j, **{k: getattr(a, k) + v for k, v in V0_ADD.items()}))
    v1 = dataclasses.replace(B, arm=dataclasses.replace(a, joints=j, roll_stage_mass=a.roll_stage_mass - KX_SAVE,
                                                        yaw_stage_mass=a.yaw_stage_mass - KX_SAVE))
    vi = with_roll_drive(v1, "inline")                       # the inline CAD masses; its yaw stage still has the KX74 socket
    vi = dataclasses.replace(vi, arm=dataclasses.replace(vi.arm, yaw_stage_mass=vi.arm.yaw_stage_mass - KX_SAVE))
    j2 = dict(vi.arm.joints); j2["shoulder_pitch"] = ("GIM8108-36", 1.0)
    # V2 pitch: the 60T pulley, D13 stage and the pitch cartridge's offset go; the motor sits on the pitch axis (ESTIMATE)
    v2 = dataclasses.replace(vi, arm=dataclasses.replace(vi.arm, joints=j2, pitch_module_mass=vi.arm.pitch_module_mass - 0.20))
    return dict(zip(NAMES, (v0, v1, vi, v2)))


def envelope():
    """mm, measured from the CAD; 'est' names the rows that are still ESTIMATE for that variant."""
    from .parts import arm, belt_stage as bs, cartridge_m as cm
    from .parts.kx import KX74, collar
    ri = json.loads((DOCS / "roll_inline.json").read_text()) if (DOCS / "roll_inline.json").exists() else {}
    db, di = ri.get("belt", {}).get("dome", {}), ri.get("inline", {}).get("dome", {})
    rmax = lambda p: 2 * max(math.hypot(v.X, v.Y) for v in p.vertices())
    kx = dict(cartridge_d=rmax(collar(KX74)), elbow_w=110.0, stage_len=-bs.V2["end"], stack=cm.stack_height() - bs.V2["end"])
    bolted = dict(cartridge_d=rmax(cm.can_bolted()), elbow_w=arm.HOUSING["x"], stage_len=-bs.V3["end"],
                  stack=cm.stack_height_bolted() - bs.V3["end"])
    dome_b = dict(shoulder_w=db.get("width_x"), shoulder_h=db.get("height_z"))
    dome_i = dict(shoulder_w=di.get("width_x"), shoulder_h=di.get("height_z"))
    return {NAMES[0]: dict(**dome_b, **kx, est=[]),
            NAMES[1]: dict(**dome_b, **bolted, est=["dome"]),
            NAMES[2]: dict(**dome_i, **bolted, est=["motor"]),
            NAMES[3]: dict(**dome_i, **bolted, est=["motor", "pitch"])}


NOTES = {
    NAMES[0]: "tool-free cartridge swap (1 collar twist); the KX seats slip at ≈ 1.16× peak (a mechanical backstop)",
    NAMES[1]: "swap = 6 screws with the belt on; no D15 sockets to build; the wrist KX56 is gone, so D21's wrist lift-off issue goes with it; "
              "no slip backstop (drive torque limits instead)",
    NAMES[2]: "most of the slimming (no Ø127 80T pulley); same hand-feel at roll (36:1 vs 35.5:1 belt); R16 across-body 62 % (belt 52 %); "
              "output bearing ≈ 280 N radial (rating ESTIMATE until the drawing)",
    NAMES[3]: "2.25× reflected inertia at pitch (36² vs 24²): stiffer to back-drive by hand; no pitch CAD",
}


def run():
    from . import basket, payload_study, spring
    from .budget import mass_budget
    from .params import act
    out = {}
    env = envelope()
    for name, d in variants().items():
        wm = spring.workspace_margins(d)
        rows, cap = payload_study.run(d)
        hug = next(r for r in rows if r["case"].startswith("B: 20 lb two-handed hug") and "0.20" in r["case"])
        dd = dataclasses.replace(d, arm=dataclasses.replace(d.arm, payload=basket.PER_HAND))
        brows, _ = basket.run(dd)
        bas = min((r["worst"] for r in brows if r["t"] is not None and r["x"] <= 0.45 and "washer" not in r["pose"] and "0.80" not in r["pose"]), default=0)
        arm_kg = sum(m for _, m in mass_budget(d)["arm (each, detachable)"])
        sp = {j: act(n)["speed"] / r * 60 / 6.2832 for j, (n, r) in d.arm.joints.items() if j in ("shoulder_pitch", "shoulder_roll")}
        cost = sum(PRICE.get(n, act(n)["price"]) for n, _ in d.arm.joints.values()) + (ROLL_BELT_PARTS if d.arm.roll_drive == "belt" else INLINE_PARTS)
        out[name] = dict(wm={k: round(v["m_rated"], 2) for k, v in wm.items()}, hug=min(hug["rated"]["shoulder_pitch"], hug["rated"]["elbow"]),
                         basket=bas, arm_kg=arm_kg, speed_rpm=sp, act_cost=cost, env=env[name], note=NOTES[name])
        print(name, out[name], flush=True)
    return out


def main():
    res = run()
    names = list(res)
    e = lambda r, k: " (ESTIMATE)" if k in r["env"]["est"] else ""
    L = ["# Slim-arm study (generated by `python -m rnd.slim_study`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         "| | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    row = lambda label, f: L.append(f"| {label} | " + " | ".join(f(res[n]) for n in names) + " |")
    row("Shoulder dome fore-aft × height (mm, CAD roll.dome())",
        lambda r: f"{r['env']['shoulder_w']:.0f} × {r['env']['shoulder_h']:.0f}" + (" (≤; measured with the KX roll socket)" if "dome" in r["env"]["est"] else ""))
    row("Motor bump diameter (KX74 collar or bolted sleeve, mm, CAD)", lambda r: f"{r['env']['cartridge_d']:.1f}")
    row("Elbow housing width (mm, CAD)", lambda r: f"{r['env']['elbow_w']:.0f}")
    row("Belt stage: mounting plane → shaft end (mm, CAD)", lambda r: f"{r['env']['stage_len']:.1f}")
    row("Cartridge rear → stage shaft end (mm, CAD)", lambda r: f"{r['env']['stack']:.1f}")
    row("GIM8108 frame (Ø100 × 60 assumed; one listing Ø96 × 57)", lambda r: "ESTIMATE" if "motor" in r["env"]["est"] else "—")
    row("One arm, detachable piece (kg)", lambda r: f"{r['arm_kg']:.2f}")
    row("Workspace margin, rated / p95 (pitch, roll, yaw, elbow, wrist)", lambda r: ", ".join(f"{v:.2f}" for v in r["wm"].values()))
    row("20 lb hug, worst of pitch/elbow (rated ×, load 0.2 m out)", lambda r: f"{r['hug']:.2f}×")
    row("Laundry basket (floor pick, carry, bed): worst duty margin (≥ 1 passes)", lambda r: f"{r['basket']:.2f} {'✅' if r['basket'] >= 1 else '❌'}")
    row("Joint speed at rated, pitch / roll (rpm)", lambda r: " / ".join(f"{v:.0f}" for v in r["speed_rpm"].values()))
    row("Actuators + roll drive parts per arm ($, m1_kit prices)", lambda r: f"{r['act_cost']:.0f}")
    row("Notes", lambda r: r["note"])
    v0, v1, vi, v2 = (res[n] for n in names)
    L += ["", "**Reading (generated):**",
          f"- **Measured, not estimated, now:** bolting shrinks the motor bump Ø{v0['env']['cartridge_d']:.1f} → Ø{v1['env']['cartridge_d']:.1f} mm "
          f"(the study's ESTIMATE was Ø113 → ≈ Ø90), the elbow housing 110 → {v1['env']['elbow_w']:.0f} mm, the belt stage "
          f"{v0['env']['stage_len']:.1f} → {v1['env']['stage_len']:.1f} mm (mounting plane → shaft end) and the whole cartridge + stage "
          f"{v0['env']['stack']:.1f} → {v1['env']['stack']:.1f} mm. The inline roll's dome is {vi['env']['shoulder_w']:.0f} × "
          f"{vi['env']['shoulder_h']:.0f} mm vs {v1['env']['shoulder_w']:.0f} × {v1['env']['shoulder_h']:.0f} with the belt "
          "(the study's ESTIMATE was 128 × 120 vs 172 × 210).",
          "- **Back-drivability is about the total reduction.** Roll today is GIM6010 8:1 × belt 4.44 = 35.5:1, so an inline 36:1 roll feels "
          "the same by hand. Pitch is GIM8108 8:1 × 3 = 24:1, so inline 36:1 there is (36/24)² = 2.25× the reflected rotor inertia.",
          "- The visible bulk is the shoulder-roll dome: the Ø127 80T pulley and the roll cartridge above it. The pitch module is mostly "
          "inside the torso.",
          f"- **Recommendation (D24): V1 everywhere + inline GIM8108-36 at the shoulder roll only.** Arm {v1['arm_kg']:.2f} → "
          f"{vi['arm_kg']:.2f} kg, roll {v1['speed_rpm']['shoulder_roll']:.0f} → {vi['speed_rpm']['shoulder_roll']:.0f} rpm, "
          f"+${vi['act_cost'] - v1['act_cost']:.0f} per arm at the wave-1 prices (the study's first pass said +$61 with a $171 GIM8108-36; "
          "the current listings are $190 bare to $307). Shoulder pitch stays on its belt (GIM8108-8 × 3).",
          "- Masses for V1 take the KX74 sockets off the roll and yaw stages (ESTIMATE until roll.py draws the bolted roll / yaw cartridges); "
          "pitch, elbow and wrist are the bolted CAD."]
    (DOCS / "slim_study.md").write_text("\n".join(L) + "\n")
    (DOCS / "slim_study.json").write_text(json.dumps(res, default=float))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
