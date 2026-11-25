"""Entry point: `python -m robodog` on the Pi, `python -m robodog --sim` anywhere."""
from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import time

from .brain import RobotTools, make_brain
from .config import load_config, sim_config
from .motion import MotionController
from .navigation import Navigator
from .sensors import make_sensors
from .servos import make_servo_driver
from .speech import Speaker, make_listener
from .vision import Vision


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pip the robot dog")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--sim", action="store_true", help="no hardware: mock servos/camera, typed commands")
    ap.add_argument("--text", action="store_true", help="type commands instead of using the microphone")
    ap.add_argument("--local", action="store_true", help="offline keyword brain only (no Claude)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    if args.sim:
        cfg = sim_config(cfg)
    if args.text:
        cfg["speech"]["stt"] = "text"
    if args.local:
        cfg["brain"]["provider"] = "local"

    tof, imu = make_sensors(cfg)
    servos = make_servo_driver(cfg["servos"])

    def imu_rad():
        o = imu.orientation()
        return (math.radians(o[0]), math.radians(o[1])) if o else None

    motion = MotionController(cfg, servos, obstacle_distance=tof.distance_mm, imu=imu_rad)
    vision = Vision(cfg)
    speaker = Speaker(cfg)
    nav = Navigator(motion, vision, tof)
    tools = RobotTools(motion, nav, vision, tof, speaker)
    brain = make_brain(cfg, tools)
    listener = make_listener(cfg, speaker)

    def shutdown(*_):
        speaker.say("Going to sleep.")
        nav.stop()
        motion.shutdown()
        vision.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)

    motion.start()
    vision.start()
    motion.set_pose("stand")
    time.sleep(1.5)
    name = cfg["robot"]["name"]
    speaker.say(f"Woof! {name} is awake.")
    try:
        for command in listener.listen():
            logging.info("command: %s", command)
            reply = brain.handle(command)
            speaker.say(reply)
    except KeyboardInterrupt:
        pass
    shutdown()


if __name__ == "__main__":
    main()
