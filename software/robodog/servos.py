"""Servo output for Pip v2: Feetech STS3215 serial-bus servos (or a mock).

Joint names: "FR.pitch", "FR.knee", ... "RL.knee", "head.pan", "head.tilt".
A joint angle (deg, software convention) maps to a raw position count:

    counts = zero + sign * angle * 4096 / 360

`zero` is the count the servo reads when the joint is at 0° — captured
automatically with the leg in its jig (`python -m robodog.calibrate --jig FR`),
so horns can go onto the spline at any tooth. `sign` comes from geometry:
left-side modules are the right-side module rotated 180°, so their pitch and
knee directions are flipped. `servos.positive_dir` flips everything if your
servos turn the other way (checked once with `calibrate --test`).
"""
from __future__ import annotations

import logging

log = logging.getLogger(__name__)

COUNTS_PER_REV = 4096
LEG_JOINTS = ("pitch", "knee")
HEAD_JOINTS = ("pan", "tilt")

# STS3215 control table (same as LeRobot's STS/SMS series table)
ADDR_ID, ADDR_TORQUE_ENABLE, ADDR_ACCELERATION = 5, 40, 41
ADDR_GOAL_POSITION, ADDR_LOCK, ADDR_PRESENT_POSITION = 42, 55, 56
ADDR_PRESENT_LOAD, ADDR_PRESENT_VOLTAGE, ADDR_PRESENT_TEMPERATURE = 60, 62, 63


def joint_names() -> list[str]:
    return [f"{leg}.{j}" for leg in ("FR", "FL", "RR", "RL") for j in LEG_JOINTS] + \
           [f"head.{j}" for j in HEAD_JOINTS]


class JointMap:
    def __init__(self, servo_cfg: dict):
        self.cfg = servo_cfg["joints"]
        self.positive = int(servo_cfg.get("positive_dir", 1))

    def clamp(self, name: str, angle: float) -> float:
        j = self.cfg[name]
        return max(j["min"], min(j["max"], angle))

    def sign(self, name: str) -> int:
        return int(self.cfg[name]["sign"]) * self.positive

    def to_counts(self, name: str, angle: float) -> int:
        j = self.cfg[name]
        c = j.get("zero", 2048) + self.sign(name) * self.clamp(name, angle) * COUNTS_PER_REV / 360.0
        return int(round(c)) % COUNTS_PER_REV

    def to_angle(self, name: str, counts: int) -> float:
        j = self.cfg[name]
        d = (counts - j.get("zero", 2048) + COUNTS_PER_REV // 2) % COUNTS_PER_REV - COUNTS_PER_REV // 2
        return self.sign(name) * d * 360.0 / COUNTS_PER_REV

    def servo_id(self, name: str) -> int:
        return int(self.cfg[name]["id"])


class MockServoDriver:
    """Records commands; used for simulation and tests."""

    def __init__(self, servo_cfg: dict):
        self.map = JointMap(servo_cfg)
        self.angles: dict[str, float] = {}     # last commanded (clamped) joint angles
        self.counts: dict[str, int] = {}
        self.enabled = True

    def write(self, angles: dict[str, float]):
        for name, a in angles.items():
            self.angles[name] = self.map.clamp(name, a)
            self.counts[name] = self.map.to_counts(name, a)

    def read_counts(self, names=None) -> dict[str, int]:
        return {n: self.counts.get(n, self.map.cfg[n].get("zero", 2048)) for n in (names or self.map.cfg)}

    def relax(self):
        self.enabled = False


class StsBusDriver(MockServoDriver):
    """Real hardware over a USB bus-servo adapter (half-duplex UART, 1 Mbaud)."""

    def __init__(self, servo_cfg: dict):
        super().__init__(servo_cfg)
        import scservo_sdk as scs   # pip install feetech-servo-sdk

        self.scs = scs
        self.port = scs.PortHandler(servo_cfg.get("port", "/dev/ttyACM0"))
        self.ph = scs.PacketHandler(0)          # STS series = protocol end 0 (little endian)
        if not self.port.openPort() or not self.port.setBaudRate(int(servo_cfg.get("baud", 1_000_000))):
            raise RuntimeError(f"cannot open servo bus on {servo_cfg.get('port')}")
        self.sync = scs.GroupSyncWrite(self.port, self.ph, ADDR_GOAL_POSITION, 2)
        accel = int(servo_cfg.get("acceleration", 0))
        for name in self.map.cfg:
            sid = self.map.servo_id(name)
            if accel:
                self.ph.write1ByteTxRx(self.port, sid, ADDR_ACCELERATION, accel)
            self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 1)

    def write(self, angles: dict[str, float]):
        super().write(angles)
        self.sync.clearParam()
        for name in angles:
            c = self.counts[name]
            self.sync.addParam(self.map.servo_id(name), [c & 0xFF, (c >> 8) & 0xFF])
        self.sync.txPacket()

    def read_counts(self, names=None) -> dict[str, int]:
        out = {}
        for name in names or self.map.cfg:
            val, res, err = self.ph.read2ByteTxRx(self.port, self.map.servo_id(name), ADDR_PRESENT_POSITION)
            if res == self.scs.COMM_SUCCESS:
                out[name] = val & 0x0FFF
        return out

    def telemetry(self, name: str) -> dict:
        sid = self.map.servo_id(name)
        v, _, _ = self.ph.read1ByteTxRx(self.port, sid, ADDR_PRESENT_VOLTAGE)
        t, _, _ = self.ph.read1ByteTxRx(self.port, sid, ADDR_PRESENT_TEMPERATURE)
        return {"voltage": v / 10.0, "temperature_c": t}

    def relax(self):
        super().relax()
        for name in self.map.cfg:
            self.ph.write1ByteTxRx(self.port, self.map.servo_id(name), ADDR_TORQUE_ENABLE, 0)


def make_servo_driver(servo_cfg: dict):
    if servo_cfg.get("driver") == "sts":
        return StsBusDriver(servo_cfg)
    log.info("Using mock servo driver")
    return MockServoDriver(servo_cfg)
