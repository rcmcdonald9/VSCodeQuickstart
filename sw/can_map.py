"""Load and validate the CAN map (sw/config/can_map.yaml).  python -m sw.can_map

Checks: IDs unique per bus (command and feedback), every arm joint in sw/description/joints.json appears exactly once,
two terminations per bus, ratios match rnd/params.py, and the bus load (same frame model as rnd/electrical.can_load)
stays ≤ 50 % at the control rate.
"""
from __future__ import annotations

import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
JOINT_PARAM = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}


def load():
    return yaml.safe_load((ROOT / "config" / "can_map.yaml").read_text())


def check(cfg=None, bits=130, max_load=0.50):
    from rnd.params import BASELINE
    cfg = cfg or load()
    errs, loads = [], {}
    seen_joints = []
    for bn, b in cfg["buses"].items():
        ids = []
        for d in b["devices"]:
            ids += [d["id"]] + ([d["fb_id"]] if "fb_id" in d else [])
            if d["name"] in JOINT_PARAM:
                seen_joints.append(d["name"])
                want = BASELINE.arm.joints[JOINT_PARAM[d["name"]]][1]
                if abs(d["ratio"] - want) > 0.01:
                    errs.append(f"{bn}/{d['name']}: ratio {d['ratio']} vs params {want:.3f}")
        if len(ids) != len(set(ids)):
            errs.append(f"{bn}: duplicate CAN IDs {ids}")
        if len(b["termination"]) != 2:
            errs.append(f"{bn}: needs exactly 2 terminations, has {len(b['termination'])}")
        frames = sum(d["frames_per_cycle"] for d in b["devices"])
        loads[bn] = frames * bits * cfg["control_hz"] / cfg["bitrate"]
        if loads[bn] > max_load:
            errs.append(f"{bn}: load {loads[bn]:.0%} > {max_load:.0%}")
    joints = [j["name"] for j in json.loads((ROOT / "description" / "joints.json").read_text()) if j["name"] != "wroll"]
    for j in joints:
        if seen_joints.count(j) != 1:
            errs.append(f"joint {j} appears {seen_joints.count(j)} times on CAN")
    return errs, loads


if __name__ == "__main__":
    e, l = check()
    print("bus load:", {k: f"{v:.0%}" for k, v in l.items()})
    print("OK ✅" if not e else "\n".join("❌ " + x for x in e))
    raise SystemExit(1 if e else 0)
