"""Single source of truth for Pip v4 design parameters (SI units: m, kg, N·m, s).

Every analysis (budget, arm sizing, drivetrain, twin) and, later, the CAD reads
these values. Change a number here and re-run `python -m rnd.run_all`.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, asdict
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
SPECS = yaml.safe_load((ROOT / "specs" / "actuators.yaml").read_text())
COMP = yaml.safe_load((ROOT / "specs" / "components.yaml").read_text())
G = 9.81


def act(name: str) -> dict:
    """Actuator spec in SI: mass kg, rated/peak N·m, speed rad/s."""
    s = SPECS[name]
    rpm = s.get("rated_speed_rpm") or 45
    return {
        "mass": s["mass_g"] / 1000 if not isinstance(s["mass_g"], list) else sum(s["mass_g"]) / 2000,
        "rated": s.get("rated_torque_nm"),
        "peak": s.get("peak_torque_nm") or s.get("stall_torque_nm"),
        "speed": rpm * 2 * 3.14159 / 60,
        "price": sum(s["price_usd"]) / 2,
    }


@dataclass
class Arm:
    l_upper: float = 0.30          # shoulder pitch axis -> elbow axis (reach sweep: 0.25 -> 0.30)
    l_fore: float = 0.28           # elbow -> wrist pitch axis (reach sweep: 0.23 -> 0.28)
    l_hand: float = 0.13           # wrist pitch -> grasp point
    shoulder_y: float = 0.20       # |y| of shoulder pitch axis from robot centre line
    shoulder_x: float = 0.06       # x of shoulder axis relative to spine
    # actuator choice per joint and belt/linkage reduction in the bone
    joints: dict = field(default_factory=lambda: {
        "shoulder_pitch": ("GIM6010-8", 2.5),   # 2.5:1 printed belt + spring assist (sized in arm_sizing)
        "shoulder_roll":  ("GIM6010-8", 3.0),   # 3:1: weakest joint in v0 sizing (no spring help sideways)
        "upper_yaw":      ("DM-J4310-2EC_V1.1", 1.0),
        "elbow":          ("GIM6010-8", 2.0),
        "wrist_pitch":    ("DM-J4310-2EC_V1.1", 1.0),
        "wrist_roll":     ("STS3215-C018", 1.0),
        "gripper":        ("STS3215-C018", 1.0),
    })
    spring_assist: float = 0.85    # fraction of the arm's own gravity moment the shoulder spring cancels
    bone_mass_upper: float = 0.26  # printed bones + shells, kg (ESTIMATE, scaled to 0.30 m)
    bone_mass_fore: float = 0.19
    gripper_mech_mass: float = 0.10
    wrist_cam_mass: float = 0.05
    payload: float = 1.0           # kg at the grasp point (R2)
    wrist_pitch_range_deg: float = 140.0   # reach sweep: ±110 -> ±140 (free win)


@dataclass
class Base:
    layout: str = "xdrive"         # xdrive (4 omni) — kiwi tip margin was only ~50 mm (twin v0)
    radius: float = 0.25           # shell radius
    wheel_radius_pos: float = 0.21 # wheel contact points from centre
    wheel_d: float = 0.1524        # 6" omni
    wheel_motor: str = "GIM6010-8"
    clearance: float = 0.025
    height: float = 0.13
    struct_mass: float = 1.4       # printed chassis + Al plate + fasteners (ESTIMATE)
    shell_mass: float = 0.45
    electronics_mass: float = 0.35 # DC-DC, CAN, flow sensor, wiring, e-stop
    lidar_mass: float = 0.15
    coupling_mass: float = 0.25    # upper/base coupling + cam lever + connector
    battery_mass: float = 1.5      # 6S 10 Ah (components.yaml)
    battery_wh: float = 222.0


@dataclass
class Upper:
    lift_min: float = 0.35         # shoulder height (z) at lowest (reach sweep)
    lift_max: float = 1.05         # reach sweep: 0.95 -> 1.05
    spine_len: float = 1.06        # extrusion length
    spine_mass_per_m: float = 0.80 # 2040 aluminium extrusion (ESTIMATE ~0.8 kg/m)
    lift_motor_mass: float = 0.9   # NEMA23 + brake (ESTIMATE), or NEMA17 + constant-force spring
    carriage_mass: float = 0.35
    torso_struct_mass: float = 0.9 # torso structure + shells (ESTIMATE)
    compute_mass: float = 0.45     # Pi 5 + Jetson option + CAN + DC-DC
    head_mass: float = 0.60        # 2×STS3215 + Gemini 336 + display + mic + speaker + shell
    cable_mass: float = 0.25
    handle_mass: float = 0.10


@dataclass
class Design:
    arm: Arm = field(default_factory=Arm)
    base: Base = field(default_factory=Base)
    upper: Upper = field(default_factory=Upper)

    def copy(self, **changes) -> "Design":
        d = deepcopy(self)
        for k, v in changes.items():
            obj, attr = k.split(".")
            setattr(getattr(d, obj), attr, v)
        return d

    def to_dict(self):
        return asdict(self)


BASELINE = Design()
