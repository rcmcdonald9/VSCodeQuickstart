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
        "shoulder_roll":  ("GIM6010-8", 3.5),   # loop 5: 3 → 3.5 (whole-workspace check: 1.29× → 1.5×); 20T:70T
        "upper_yaw":      ("GIM6010-8", 2.0),   # loop 5: DM-J4310 direct was under-rated over the workspace (p95 4.6 N·m vs 3 rated)
        "elbow":          ("GIM6010-8", 2.0),
        "wrist_pitch":    ("DM-J4310-2EC_V1.1", 1.0),
        "wrist_roll":     ("STS3215-C018", 1.0),
        "gripper":        ("STS3215-C018", 1.0),
    })
    spring_k_nm: float = 4.0       # shoulder balancer torque at the horizontal arm, τ = K·sin(pitch). Loop 5 (rnd/spring.py):
                                   # replaces the "85 % everywhere" assumption; K tuned for a 0.5 kg typical payload
    bone_mass_upper: float = 0.59  # FROM CAD (loop 4, rnd/parts/arm.py): shell 0.26 + yaw KX56 socket + elbow KX74 socket + pulley hubs + bearings (was 0.26 ESTIMATE)
    bone_mass_fore: float = 0.32   # FROM CAD (loop 4): side-link shell 0.21 + wrist KX56 socket + bearings + fasteners (was 0.19 ESTIMATE)
    gripper_mech_mass: float = 0.10
    wrist_cam_mass: float = 0.05
    payload: float = 1.0           # kg at the grasp point (R2)
    # Elbow motor placement: None = at the elbow; a distance (m from the shoulder pitch axis) = motor sits up the
    # upper arm, just past the yaw joint, and drives the elbow through a long belt (precedent: Berkeley Blue moves
    # motors back along belts). The elbow then carries only a pulley hub + bearings.
    elbow_motor_at: float | None = None
    pitch_module_mass: float = 0.81    # FROM CAD (loop 7, rnd/parts/shoulder.py): bracket, pitch shaft + 50T pulley + balancer lever +
                                       # KX-V ferrule, small hub + KX74 socket, bearings, spring/cable/pulley A, V-band clamp. Torso side.
    roll_stage_mass: float = 0.60      # ESTIMATE scaled from the pitch-module CAD (no spring/clamp, 70T pulley, male ferrule). Arm side.
    yaw_stage_mass: float = 0.20       # upper-arm yaw belt stage (2:1, pulley hubs + bearings + belt), ESTIMATE from elbow-stage CAD
    elbow_hub_mass: float = 0.12   # big pulley + hub + 2×61808 + tensioner when the motor is remote (ESTIMATE)
    wrist_pitch_range_deg: float = 120.0   # ±140 folded the hand into the forearm (collision study); ±120 is realistic and reaches more


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
    # Carry split: each arm detaches at its shoulder-pitch KX socket (one collar twist + a blind-mate plug).
    # The pitch cartridge, its stage and the spring stay on the torso; everything outboard is the "arm" piece.
    arms_detach: bool = True


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
