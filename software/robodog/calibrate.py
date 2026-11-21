"""Servo setup & calibration helper.

  python -m robodog.calibrate --center      # every servo to its mid-point (do this BEFORE fitting horns)
  python -m robodog.calibrate               # interactive: nudge each joint, find dir/trim, save to config.yaml

Lift the robot off the table (or hold it) while calibrating.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from .config import DEFAULT_PATH, JOINTS, LEG_NAMES, load_config
from .servos import make_servo_driver

HELP = """Keys:  a/d = trim -/+1°   A/D = trim -/+5°   f = flip direction   t = test (+20° wiggle)
       n = next joint   p = previous joint   s = save (calibration.yaml)   q = quit"""


def center_all(cfg):
    drv = make_servo_driver(cfg["servos"])
    mid = drv.map.range / 2
    for leg in LEG_NAMES:
        for j in JOINTS:
            drv.set_servo(drv.map.channel(leg, j), mid)
    print(f"All 12 servos at {mid:.0f}° (1500 µs). Now fit the horns/legs in the STAND pose:")
    print("  roll : hip housing level (pitch servo vertical)")
    print("  pitch: upper leg tilted ~30° with the knee pointing BACKWARD")
    print("  knee : lower leg bent ~60° relative to the upper leg so the foot is under the hip")
    print("The 25-tooth spline moves in 14.4° steps — pick the closest tooth; trim fixes the rest.")


def interactive(cfg, path):
    import time

    drv = make_servo_driver(cfg["servos"])
    order = [(leg, j) for leg in LEG_NAMES for j in JOINTS]
    i = 0
    print(HELP)

    def apply(angle_offset=0.0):
        leg, j = order[i]
        jc = cfg["servos"]["legs"][leg][j]
        drv.set_servo(jc["channel"], drv.map.to_servo(leg, j, jc["center"] + angle_offset))

    while True:
        leg, j = order[i]
        jc = cfg["servos"]["legs"][leg][j]
        apply()
        cmd = input(f"[{leg} {j:5s}] ch={jc['channel']:2d} dir={jc['dir']:+d} trim={jc['trim']:+.0f} > ").strip()
        if cmd in ("a", "d", "A", "D"):
            jc["trim"] += {"a": -1, "d": 1, "A": -5, "D": 5}[cmd]
        elif cmd == "f":
            jc["dir"] *= -1
        elif cmd == "t":
            print("  a correct +20° wiggle moves: roll=foot OUTWARD, pitch=knee BACKWARD, knee=leg STRAIGHTER")
            apply(20); time.sleep(0.6); apply(); time.sleep(0.3)
        elif cmd == "n":
            i = (i + 1) % len(order)
        elif cmd == "p":
            i = (i - 1) % len(order)
        elif cmd == "s":
            legs = {leg: {jn: {"dir": cfg["servos"]["legs"][leg][jn]["dir"],
                               "trim": cfg["servos"]["legs"][leg][jn]["trim"]} for jn in JOINTS}
                    for leg in LEG_NAMES}
            with open(path, "w") as f:
                yaml.safe_dump({"servos": {"legs": legs}}, f, sort_keys=False)
            print(f"saved dir/trim to {path}")
        elif cmd == "q":
            drv.relax()
            return
        else:
            print(HELP)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(DEFAULT_PATH))
    ap.add_argument("--center", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.center:
        center_all(cfg)
    else:
        interactive(cfg, Path(a.config).with_name("calibration.yaml"))


if __name__ == "__main__":
    main()
