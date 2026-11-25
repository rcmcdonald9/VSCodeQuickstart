"""Bus-servo setup & calibration for Pip v2. No spline-tooth fiddling: the
encoders tell us where each joint is.

  python -m robodog.calibrate --scan            # list servo IDs on the bus
  python -m robodog.calibrate --set-id 3        # ONE new servo connected: give it ID 3
  python -m robodog.calibrate --jig FR          # FR leg sitting straight in the leg jig -> save zeros
  python -m robodog.calibrate --head            # head looking straight ahead -> save zeros
  python -m robodog.calibrate --test FR.pitch   # +15° wiggle to check direction
  python -m robodog.calibrate --jog 1           # nudge servo ID 1 with a/d (line up horn holes)

Zeros are saved to software/calibration.yaml (merged over config.yaml).
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import yaml

from .config import DEFAULT_PATH, load_config
from .servos import ADDR_ID, ADDR_LOCK, ADDR_TORQUE_ENABLE, StsBusDriver, make_servo_driver

EXPECT = {"pitch": "the knee moves BACKWARD", "knee": "the leg STRAIGHTENS",
          "pan": "the head turns LEFT", "tilt": "the head nods DOWN"}


def _save_zeros(cal_path: Path, zeros: dict[str, int]):
    data = yaml.safe_load(cal_path.read_text()) if cal_path.exists() else {}
    data = data or {}
    joints = data.setdefault("servos", {}).setdefault("joints", {})
    for name, z in zeros.items():
        joints.setdefault(name, {})["zero"] = int(z)
    cal_path.write_text(yaml.safe_dump(data, sort_keys=True))
    print(f"saved {', '.join(f'{k}={v}' for k, v in zeros.items())} -> {cal_path}")


def _raw_bus(cfg):
    import scservo_sdk as scs

    port = scs.PortHandler(cfg["servos"].get("port", "/dev/ttyACM0"))
    ph = scs.PacketHandler(0)
    if not port.openPort() or not port.setBaudRate(int(cfg["servos"].get("baud", 1_000_000))):
        raise SystemExit("cannot open the servo bus")
    return scs, port, ph


def scan(cfg):
    scs, port, ph = _raw_bus(cfg)
    found = [i for i in range(0, 21) if ph.ping(port, i)[1] == scs.COMM_SUCCESS]
    print("servo IDs on the bus:", found or "none")


def set_id(cfg, new_id: int):
    scs, port, ph = _raw_bus(cfg)
    ids = [i for i in range(0, 254) if ph.ping(port, i)[1] == scs.COMM_SUCCESS]
    if len(ids) != 1:
        raise SystemExit(f"connect exactly ONE servo (found {ids})")
    old = ids[0]
    ph.write1ByteTxRx(port, old, ADDR_TORQUE_ENABLE, 0)
    ph.write1ByteTxRx(port, old, ADDR_LOCK, 0)          # unlock EEPROM
    ph.write1ByteTxRx(port, old, ADDR_ID, new_id)
    ph.write1ByteTxRx(port, new_id, ADDR_LOCK, 1)
    print(f"servo {old} -> ID {new_id}. Label it!")


def capture(cfg, names: list[str], cal_path: Path):
    drv = make_servo_driver(cfg["servos"])
    if isinstance(drv, StsBusDriver):
        drv.relax()                                       # limp, so the jig sets the pose
    counts = drv.read_counts(names)
    missing = [n for n in names if n not in counts]
    if missing:
        raise SystemExit(f"no reply from {missing}")
    _save_zeros(cal_path, counts)


def test(cfg, name: str):
    drv = make_servo_driver(cfg["servos"])
    joint = name.split(".")[1]
    print(f"+15° on {name}: correct if {EXPECT[joint]}. If not, set servos.positive_dir: -1")
    drv.write({name: 0.0}); time.sleep(0.8)
    drv.write({name: 15.0}); time.sleep(0.8)
    drv.write({name: 0.0}); time.sleep(0.5)
    drv.relax()


def jog(cfg, sid: int):
    from .servos import ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION

    scs, port, ph = _raw_bus(cfg)
    pos, res, _ = ph.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    if res != scs.COMM_SUCCESS:
        raise SystemExit(f"no servo with ID {sid}")
    ph.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)
    step = round(2 * 4096 / 360)
    while True:
        k = input(f"[ID {sid}] pos={pos} a/d = -/+2°, A/D = -/+10°, q = done > ").strip()
        if k == "q":
            break
        pos = (pos + {"a": -step, "d": step, "A": -5 * step, "D": 5 * step}.get(k, 0)) % 4096
        ph.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, pos)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(DEFAULT_PATH))
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--scan", action="store_true")
    g.add_argument("--set-id", type=int)
    g.add_argument("--jig", choices=["FR", "FL", "RR", "RL"])
    g.add_argument("--head", action="store_true")
    g.add_argument("--test")
    g.add_argument("--jog", type=int)
    a = ap.parse_args()
    cfg = load_config(a.config)
    cal = Path(a.config).with_name("calibration.yaml")
    if a.scan:
        scan(cfg)
    elif a.set_id is not None:
        set_id(cfg, a.set_id)
    elif a.jig:
        capture(cfg, [f"{a.jig}.pitch", f"{a.jig}.knee"], cal)   # jig pose = pitch 0, knee 0
    elif a.head:
        capture(cfg, ["head.pan", "head.tilt"], cal)
    elif a.jog is not None:
        jog(cfg, a.jog)
    else:
        test(cfg, a.test)


if __name__ == "__main__":
    main()
