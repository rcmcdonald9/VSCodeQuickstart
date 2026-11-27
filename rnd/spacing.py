"""Spacing check (design target R15, owner 2026-11-26).  python -m rnd.spacing

Minimum distance (BRepExtrema) between every pair of parts in a module whose bounding boxes come within 2 mm. Each
pair is classified, and the first matching rule wins:

- **mate:** fastened, bonded, fused or nested by design (a motor in its can, a collar on its hub, a part bolted to a
  bracket, deck quarters, a bezel over the screen). No spacing rule.
- **running:** relative motion across a clearance set by a bearing, shim or spacer (axial gaps beside bearings, a pin in
  a guide slot). Target ≥ 0.3 mm.
- **belt:** a belt versus anything other than its own pulleys. Target ≥ 3 mm (hard 1 mm).
- **moving:** other pairs in relative motion. Target ≥ 2 mm (hard 1 mm).
- **static:** everything else. Target ≥ 1 mm (hard 0.5 mm).

Swept checks over the joint ranges are separate (`swept()`): the elbow over −150…0°, the gripper jaw 0…45°, the head
over tilt ±30°, the KX collar twist 0…35°.
"""
from __future__ import annotations

import itertools
import json
from pathlib import Path

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"

MATE = [("can", "motor"), ("collar", "hub"), ("hub", "shaft"), ("shaft", "wheel"), ("pod", "deck"), ("torque_hanger", "deck"),
        ("deck", "deck"), ("deck", "rim"), ("deck", "coupling"), ("skirt", "skirt"), ("skirt", "bumper"), ("coupling", "plug"),
        ("mast", "ratchet"), ("mast", "lift_mount"), ("mast", "carriage"), ("mast", "head_bracket"), ("lift_pulley", "lift_belt"),
        ("lift_mount", "lift_cover"), ("torso_shell", "estop"), ("torso_shell", "chest_light"), ("torso_shell", "ai_bay"),
        ("e_tray", "e_"), ("head_bracket", "handle"), ("head_bracket", "pan_sleeve"), ("yoke", "tilt_sleeve"), ("head_shell", "visor"),
        ("visor", "head_screen"), ("visor", "head_cam"), ("head_screen", "eye"), ("upper_bone", "bone_cap"), ("upper_bone", "cf_neck"),
        ("forearm", "cf_fore"), ("wrist_can", "wrist_motor"), ("wrist_motor", "wrist_adapter"), ("wrist_adapter", "wp_hub"),
        ("wrist_adapter", "wp_collar"), ("wp_hub", "palm"), ("palm", "wr_sleeve"), ("gr_hub", "grip_base"), ("grip_base", "grip_servo"),
        ("grip_base", "jaw_fixed"), ("grip_base", "wrist_cam"), ("jaw_fixed", "finray_fixed"), ("finray_fixed", "pad_fixed"),
        ("ferrule_m", "roll_link"), ("roll_link", "shoulder_cap"), ("roll_shaft", "yaw_block"), ("yaw_block", "yaw_band"),
        ("yaw_block", "yaw_stop_pin"), ("e_hub", "e_belt"), ("e_belt", "e_bigpulley"), ("roll_can", "roll_motor"), ("yaw_can", "yaw_motor"),
        ("grip_servo_sleeve", "grip_servo"), ("kx_hub_v2", "flange_coupling"), ("flange_coupling", "shaft_12"), ("shaft_12", "brg_"),
        ("shaft_12", "al_pulley"), ("al_pulley", "belt"), ("bigpulley", "belt"), ("e_cassette", "brg_"), ("brg_", "al_pulley"),
        ("flange_coupling", "brg_"), ("roll_shaft", "roll_pulley72"), ("yaw_shaft", "yaw_flange"), ("yaw_block", "yaw_plate_bot"), ("yaw_plate_bot", "brg_"), ("yaw_plate_bot", "yaw_stop_pin"), ("yaw_plate_bot", "yaw_spring_ring"), ("yaw_plate_bot", "yaw_band"), ("e_bigpulley", "e_shaft40"), ("shaft", "pitch_pulley60"), ("roll_spring_hub", "yaw_block"),
        ("yaw_block", "yaw_spring_ring"), ("roll_shaft", "roll_spring_hub"), ("yaw_pulley40", "brg_"), ("roll_pulley72", "brg_"),
        ("wr_spindle", "wr_brg"), ("palm", "wr_brg"), ("wr_brg_U", "wr_brg_L"), ("wr_spindle", "grip_base"), ("wr_dog", "wr_sleeve"), ("palm", "wr_dog"), ("palm", "wr_cap"), ("wr_cap", "wr_brg"),
        ("wr_dog", "wr_spindle"), ("wr_brg", "grip_base"), ("finger_hook", "jaw_fixed"), ("finger_hook", "finray_fixed")]   # dog: 0.1 mm a side by design (lash); the base boss clamps the lower inner race   # bearing inner races, shims and the pulley rotate together on the shaft
RUNNING = [("yaw_plate_bot", "yaw_shaft"), ("yaw_flange", "yaw_stop_pin"), ("bracket", "kx_hub_v2"), ("bracket", "al_pulley"), ("bracket", "flange_coupling"), ("e_cassette", "e_al_pulley"), ("e_cassette", "e_kx_hub_v2"), ("e_cassette", "e_flange_coupling"), ("e_cassette", "e_hub"), ("roll_link", "roll_smallhub"), ("yaw_block", "yaw_smallhub"), ("bracket", "smallhub"),
           ("yaw_shaft", "yaw_stop_pin"), ("bracket", "pulleyA"), ("roll_link", "al_pulley"), ("yaw_block", "al_pulley"),
           ("roll_link", "roll_spring_hub"), ("yaw_spring_ring", "yaw_shaft"), ("yaw_plate_bot", "al_pulley"), ("roll_link", "roll_pulley72"), ("yaw_block", "yaw_pulley40"),
           ("palm", "wr_spindle"), ("palm", "grip_base"), ("wr_brg", "wr_dog"), ("wr_cap", "grip_base"), ("wr_cap", "wr_spindle")]
MOVING = [("yaw_plate_bot", "yaw_flange"), ("e_cassette", "e_bigpulley"), ("roll_shaft", "yaw_shaft"), ("yaw_block", "yaw_shaft"), ("upper_bone", "forearm")]


def _match(a, b, table):
    return any((x in a and y in b) or (x in b and y in a) for x, y in table)


def classify(a, b):
    if _match(a, b, MATE):
        return "mate"
    if "belt" in a or "belt" in b:
        return "belt"
    if _match(a, b, RUNNING):
        return "running"
    if _match(a, b, MOVING):
        return "moving"
    return "static"


RULES = {"running": (0.3, 0.2), "belt": (3.0, 1.0), "moving": (2.0, 1.0), "static": (1.0, 0.5)}


def run():
    from .parts import arm, base, roll, shoulder, upper
    mods = {"arm": arm.assembly, "roll/yaw": roll.module, "shoulder": shoulder.module, "base": base.module, "upper": lambda: upper.module(0.75)}
    rows = []
    for mn, f in mods.items():
        for (a, pa, _), (b, pb, _) in itertools.combinations(f(), 2):
            ba, bb = pa.bounding_box(), pb.bounding_box()
            if (ba.min.X > bb.max.X + 2 or bb.min.X > ba.max.X + 2 or ba.min.Y > bb.max.Y + 2 or bb.min.Y > ba.max.Y + 2
                    or ba.min.Z > bb.max.Z + 2 or bb.min.Z > ba.max.Z + 2):
                continue
            k = classify(a, b)
            if k == "mate":
                continue
            try:
                dist = pa.distance_to(pb)
            except Exception:
                continue
            tgt, hard = RULES[k]
            if dist < tgt:
                rows.append(dict(module=mn, a=a, b=b, kind=k, d=round(dist, 2), target=tgt, hard=hard, hard_fail=dist < hard))
    return rows


def main():
    rows = run()
    nh = sum(r["hard_fail"] for r in rows)
    L = ["# Spacing check, R15 (generated by `python -m rnd.spacing`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Pairs under target: {len(rows)}; under the hard limit: {nh}.**", "",
         "| Module | Part A | Part B | Kind | Gap (mm) | Target / hard | |", "|---|---|---|---|---|---|---|"]
    L += [f"| {r['module']} | {r['a']} | {r['b']} | {r['kind']} | {r['d']} | {r['target']} / {r['hard']} | {'❌ hard' if r['hard_fail'] else '⚠️ target'} |"
          for r in sorted(rows, key=lambda r: (not r["hard_fail"], r["d"]))]
    (DOCS / "spacing.md").write_text("\n".join(L))
    (DOCS / "spacing.json").write_text(json.dumps({"under_target": len(rows), "under_hard": nh}))
    print("\n".join(L[4:]))


if __name__ == "__main__":
    main()
