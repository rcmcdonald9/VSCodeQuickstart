"""Servo output: joint angles -> PCA9685 pulses (or a mock for simulation)."""
from __future__ import annotations

import logging

from .config import JOINTS, LEG_NAMES

log = logging.getLogger(__name__)


class JointMap:
    """Converts a joint angle (deg) into a servo angle (deg, 0..range)."""

    def __init__(self, servo_cfg: dict):
        self.range = float(servo_cfg.get("actuation_range", 180))
        self.legs = servo_cfg["legs"]

    def clamp(self, leg: str, joint: str, angle: float) -> float:
        j = self.legs[leg][joint]
        return max(j["min"], min(j["max"], angle))

    def to_servo(self, leg: str, joint: str, angle: float) -> float:
        j = self.legs[leg][joint]
        angle = self.clamp(leg, joint, angle)
        s = self.range / 2 + j["dir"] * (angle - j["center"]) + j.get("trim", 0)
        return max(0.0, min(self.range, s))

    def channel(self, leg: str, joint: str) -> int:
        return int(self.legs[leg][joint]["channel"])


class MockServoDriver:
    def __init__(self, servo_cfg: dict):
        self.map = JointMap(servo_cfg)
        self.last: dict[int, float] = {}

    def set_servo(self, channel: int, servo_deg: float | None):
        self.last[channel] = servo_deg

    def write_joints(self, joints: dict[str, tuple[float, float, float]]):
        for leg, angles in joints.items():
            for joint, a in zip(JOINTS, angles):
                self.set_servo(self.map.channel(leg, joint), self.map.to_servo(leg, joint, a))

    def relax(self):
        for leg in LEG_NAMES:
            for joint in JOINTS:
                self.set_servo(self.map.channel(leg, joint), None)


class PCA9685ServoDriver(MockServoDriver):
    def __init__(self, servo_cfg: dict):
        super().__init__(servo_cfg)
        from adafruit_servokit import ServoKit  # imported lazily: Pi-only dependency

        self.kit = ServoKit(channels=16, address=int(servo_cfg.get("i2c_address", 0x40)))
        lo, hi = servo_cfg.get("min_pulse_us", 500), servo_cfg.get("max_pulse_us", 2500)
        for ch in range(16):
            self.kit.servo[ch].set_pulse_width_range(lo, hi)
            self.kit.servo[ch].actuation_range = int(self.map.range)

    def set_servo(self, channel: int, servo_deg: float | None):
        super().set_servo(channel, servo_deg)
        self.kit.servo[channel].angle = servo_deg  # None = stop pulsing (limp)


def make_servo_driver(servo_cfg: dict):
    if servo_cfg.get("driver") == "pca9685":
        return PCA9685ServoDriver(servo_cfg)
    log.info("Using mock servo driver")
    return MockServoDriver(servo_cfg)
