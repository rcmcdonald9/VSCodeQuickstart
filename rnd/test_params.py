"""Configuration-consistency test (Astra review #3, major 15).  python -m rnd.test_params

Two ways of selecting a variant must give identical parameters: the environment (PIP_ROLL_DRIVE / PIP_ARM_MOTORS read
at import, the direct Arm() construction) and the switch functions (params.with_roll_drive / with_arm_motors on the
default BASELINE), in either order, and a switch there and back must return the start. Before the fix with_arm_motors()
dropped the inline roll's 0.010 kg bone_upper_dm.
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _env_design(roll: str, motors: str) -> dict:
    env = dict(os.environ, PIP_ROLL_DRIVE=roll, PIP_ARM_MOTORS=motors)
    code = "import json; from rnd.params import BASELINE; print(json.dumps(BASELINE.to_dict()))"
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True, check=True).stdout
    return json.loads(out.strip().splitlines()[-1])


def _norm(x):
    if isinstance(x, dict):
        return {k: _norm(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_norm(v) for v in x]
    return x


def _diff(a, b, path=""):
    if isinstance(a, dict):
        return [d for k in sorted(set(a) | set(b)) for d in _diff(a.get(k), b.get(k), f"{path}.{k}")]
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [d for i, (x, y) in enumerate(zip(a, b)) for d in _diff(x, y, f"{path}[{i}]")]
    if isinstance(a, float) and isinstance(b, float):
        return [] if math.isclose(a, b, rel_tol=0, abs_tol=1e-9) else [f"{path}: {a} vs {b}"]
    return [] if a == b else [f"{path}: {a} vs {b}"]


def run():
    from .params import ARM_MOTOR_SETS, BASELINE, ROLL_DRIVES, with_arm_motors, with_roll_drive
    fails, n = [], 0
    for roll in ROLL_DRIVES:
        for motors in ARM_MOTOR_SETS:
            ref = _norm(_env_design(roll, motors))
            for lab, d in (("motors→roll", with_roll_drive(with_arm_motors(BASELINE, motors), roll)),
                           ("roll→motors", with_arm_motors(with_roll_drive(BASELINE, roll), motors))):
                n += 1
                dd = _diff(ref, _norm(d.to_dict()))
                if dd:
                    fails.append((roll, motors, lab, dd))
            n += 1                                                  # there and back
            back = with_roll_drive(with_arm_motors(with_arm_motors(with_roll_drive(BASELINE, roll), motors), BASELINE.arm.arm_motors),
                                   BASELINE.arm.roll_drive)
            dd = _diff(_norm(BASELINE.to_dict()), _norm(back.to_dict()))
            if dd:
                fails.append((roll, motors, "round trip", dd))
    return n, fails


if __name__ == "__main__":
    n, fails = run()
    for roll, motors, lab, dd in fails:
        print(f"FAIL {roll} / {motors} ({lab}): " + "; ".join(dd[:6]))
    print(f"{n - len(fails)}/{n} configuration-consistency cases pass")
    sys.exit(1 if fails else 0)
