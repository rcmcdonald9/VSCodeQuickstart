"""Load and validate the CAN map (the arm, Step 2: sw/config/can_map.yaml; a rig: sw/config/rig_<name>.yaml, sw/rig.py).
python -m sw.can_map [--rig step1]

Checks: IDs unique per bus (command and feedback), every joint of the description (the arm: sw/description/joints.json;
a rig: its own joint rows) appears exactly once, two terminations per bus, ratios match rnd/params.py (the arm) or the
rig's joint description (the Step-1 rig: 1.0, bare outputs, no belt stage), and the bus load (same frame model as
rnd/electrical.can_load) stays ≤ 50 % at the control rate.
"""
from __future__ import annotations

import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
JOINT_PARAM = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}


def load(rig=None):
    """The arm's CAN map, or with rig="step1" the Step-1 rig's (tagged with "rig": the checks then use the rig)."""
    if rig:
        from sw.rig import can_map
        return can_map(rig)
    return yaml.safe_load((ROOT / "config" / "can_map.yaml").read_text())


def check(cfg=None, bits=130, max_load=0.50):
    from rnd.params import BASELINE
    cfg = cfg or load()
    errs, loads = [], {}
    seen_joints = []
    rig = cfg.get("rig")
    desc = {}
    if rig:
        from sw.rig import description
        desc = {j["name"]: j for j in description(rig)}
    for bn, b in cfg["buses"].items():
        ids = []
        for d in b["devices"]:
            ids += [d["id"]] + ([d["fb_id"]] if "fb_id" in d else [])
            if d["name"] in JOINT_PARAM:
                seen_joints.append(d["name"])
                if rig:
                    want, src = (desc[d["name"]]["ratio"], f"rig {rig}") if d["name"] in desc else (None, "")
                    if want is None:
                        errs.append(f"{bn}/{d['name']}: not a joint of rig {rig}")
                        continue
                else:
                    want, src = BASELINE.arm.joints[JOINT_PARAM[d["name"]]][1], "params"
                if abs(d["ratio"] - want) > 0.01:
                    errs.append(f"{bn}/{d['name']}: ratio {d['ratio']} vs {src} {want:.3f}")
        if len(ids) != len(set(ids)):
            errs.append(f"{bn}: duplicate CAN IDs {ids}")
        if len(b["termination"]) != 2:
            errs.append(f"{bn}: needs exactly 2 terminations, has {len(b['termination'])}")
        frames = sum(d["frames_per_cycle"] for d in b["devices"])
        loads[bn] = frames * bits * cfg["control_hz"] / cfg["bitrate"]
        if loads[bn] > max_load:
            errs.append(f"{bn}: load {loads[bn]:.0%} > {max_load:.0%}")
    joints = list(desc) if rig else [j["name"] for j in json.loads((ROOT / "description" / "joints.json").read_text()) if j["name"] != "wroll"]
    for j in joints:
        if seen_joints.count(j) != 1:
            errs.append(f"joint {j} appears {seen_joints.count(j)} times on CAN")
    return errs, loads


if __name__ == "__main__":
    import sys
    e, l = check(load(sys.argv[sys.argv.index("--rig") + 1] if "--rig" in sys.argv else None))
    print("bus load:", {k: f"{v:.0%}" for k, v in l.items()})
    print("OK ✅" if not e else "\n".join("❌ " + x for x in e))
    raise SystemExit(1 if e else 0)
