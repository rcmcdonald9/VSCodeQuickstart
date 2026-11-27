"""Single source of truth for Pip v4 design parameters (SI units: m, kg, N·m, s).

Every analysis (budget, arm sizing, drivetrain, twin) and, later, the CAD reads
these values. Change a number here and re-run `python -m rnd.run_all`.
"""
from __future__ import annotations

import os
from copy import deepcopy
from dataclasses import dataclass, field, asdict, replace
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


# Shoulder-roll drive (D24, proposed 2026-11-27, awaiting the owner's OK): "belt" = GIM6010-8 cartridge + 18T:80T belt
# (loops 10–44); "inline" = a GIM8108-36 on the roll axis, its output flange bolted to the clevis (rnd/parts/roll.py).
# The default stays "belt" until the owner approves. PIP_ROLL_DRIVE=inline selects the other one for a whole run (CAD,
# twin, checks): `PIP_ROLL_DRIVE=inline python -m rnd.roll_inline`. The numbers are the CAD's: roll.py warns if
# roll_offset_y / yaw_drop drift from the geometry it derives.
ROLL_DRIVE = os.environ.get("PIP_ROLL_DRIVE", "belt")
ROLL_DRIVES = {
    "belt": dict(roll_offset_y=0.083, yaw_drop=0.1160, shoulder_roll=("GIM6010-8", 80 / 18), roll_stage_mass=0.72, yaw_stage_mass=0.57),   # D19 caps: +10 g; yaw_drop 113.4 → 116.0: the yaw clock-spring cavity 8.7 → 11.3 mm (A14 ribbon 9.8 wide)
    # FROM CAD (rnd/parts/roll.py, inline): ROLL_Y = ferrule face 18 + 2 + motor r 50 (Ø100 ESTIMATE); the top yaw plate
    # clears the motor (r 50 + 4) instead of the Ø127 80T pulley (yaw_drop 116.0 → 103.9); masses = roll.masses(split=True), the motor (1.0 kg) is the
    # roll actuator mass.
    "inline": dict(roll_offset_y=0.070, yaw_drop=0.1039, shoulder_roll=("GIM8108-36", 1.0), roll_stage_mass=0.28, yaw_stage_mass=0.55),
}
assert ROLL_DRIVE in ROLL_DRIVES, f"PIP_ROLL_DRIVE must be one of {list(ROLL_DRIVES)}"
_RD = ROLL_DRIVES[ROLL_DRIVE]

# D20 actuator switch (R2c, owner 2026-11-27: 20 lb two-handed hug + the R2d laundry basket). "gim6010" = GIM6010-8 at
# shoulder pitch (3:1) and elbow (2:1, 20T:40T), loops 10–46; "d20" = GIM8108-8 at both, 3:1 (elbow 20T:60T). The default
# stays "gim6010" until the seller confirms the GIM8108-8 runs at 24 V (most listings are 48 V) and sends the flange drawing.
# PIP_ARM_MOTORS=d20 selects D20 for a whole run (CAD rnd/parts/arm.py + shoulder.py, twin, checks), as PIP_ROLL_DRIVE does.
# elbow_belt_c: the elbow belt centre distance in the CAD (arm.BELT_C). The masses are the CAD roll-ups (arm.masses(),
# shoulder.masses()) for that motor set.
ARM_MOTORS = os.environ.get("PIP_ARM_MOTORS", "gim6010")
ARM_MOTOR_SETS = {
    "gim6010": dict(shoulder_pitch=("GIM6010-8", 3.0), elbow=("GIM6010-8", 2.0), elbow_teeth=40, elbow_belt_c=0.070,
                    bone_mass_upper=0.79, pitch_module_mass=0.76),
    # FROM CAD (2026-11-27, PIP_ARM_MOTORS=d20): elbow housing 98 → 109 wide (the Ø102.6 can), 12 mm taller at the top (belt centre
    # 70 → 84.0 mm, 76T belt: the can has to clear the −Y 61808 boss) and 16 mm deeper (the integral 60T), Ø50 Oldham,
    # Ø84 flange. CAD deltas over the GIM6010 set: arm.masses() 0.995 → 1.138 kg (+0.14: housing + seat caps +40 g, 60T +37 g,
    # Ø50 Oldham + Ø102.6 can +68 g), shoulder.masses() 0.758 → 0.824 kg (+0.07, Ø50 Oldham + can). Motor +8 g via act().
    "d20": dict(shoulder_pitch=("GIM8108-8", 3.0), elbow=("GIM8108-8", 3.0), elbow_teeth=60, elbow_belt_c=0.0840,
                bone_mass_upper=0.93, pitch_module_mass=0.83),
}
assert ARM_MOTORS in ARM_MOTOR_SETS, f"PIP_ARM_MOTORS must be one of {list(ARM_MOTOR_SETS)}"
_AM = ARM_MOTOR_SETS[ARM_MOTORS]


@dataclass
class Arm:
    l_upper: float = 0.30          # shoulder pitch axis -> elbow axis (reach sweep: 0.25 -> 0.30)
    l_fore: float = 0.28           # elbow -> wrist pitch axis (reach sweep: 0.23 -> 0.28)
    l_hand: float = 0.156          # wrist pitch -> grasp point (loop 10 hand CAD: wrist roll + KX44 + gripper servo stack; was 0.13; loop 40 D16 wrist-roll spindle: +6 mm)
    shoulder_y: float = 0.20       # |y| of shoulder pitch axis from robot centre line
    shoulder_x: float = 0.06       # x of shoulder axis relative to spine
    roll_drive: str = ROLL_DRIVE   # "belt" | "inline" (ROLL_DRIVES above; D24)
    arm_motors: str = ARM_MOTORS   # "gim6010" | "d20" (ARM_MOTOR_SETS above; D20)
    elbow_teeth: int = _AM["elbow_teeth"]        # elbow big pulley (20T small): 40 → 2:1, 60 → 3:1
    elbow_belt_c: float = _AM["elbow_belt_c"]    # elbow belt centre distance (m), = arm.BELT_C
    roll_offset_y: float = _RD["roll_offset_y"]   # roll axis outboard of the pitch body / detach plane (belt: loop 10 roll CAD: 0.05 → 0.075, the 72T pulley; loop 40: 0.083, the 80T pulley; inline 0.070)
    yaw_drop: float = _RD["yaw_drop"]       # belt: roll axis → upper-arm yaw interface (loop 10 roll CAD: 0.07 → 0.10; loop 40 D13 yaw stage + 80T roll pulley: 0.113 = roll.YAW_DROP); the upper bone shortens to keep l_upper
    # actuator choice per joint and belt/linkage reduction in the bone
    joints: dict = field(default_factory=lambda: {
        "shoulder_pitch": _AM["shoulder_pitch"],   # ARM_MOTOR_SETS (D20). GIM6010: loop 10: 2.5 → 3 (20T:60T) after the hand CAD added 0.2 kg at the tip
        "shoulder_roll":  _RD["shoulder_roll"],   # inline: GIM8108-36 direct. Belt: loop 40: 4.0 → 4.44 (18T:80T, D13 mass); loop 10: 3.5 → 4 (18T:72T); loop 5: 3 → 3.5
        "upper_yaw":      ("GIM6010-8", 2.0),   # loop 5: DM-J4310 direct was under-rated over the workspace (p95 4.6 N·m vs 3 rated)
        "elbow":          _AM["elbow"],          # ARM_MOTOR_SETS (D20): GIM6010-8 2:1 or GIM8108-8 3:1
        "wrist_pitch":    ("DM-J4310-2EC_V1.1", 1.0),
        "wrist_roll":     ("STS3215-C018", 1.0),
        "gripper":        ("STS3215-C018", 1.0),
    })
    spring_k_nm: float = 4.5       # shoulder balancer torque at the horizontal arm, τ = K·sin(pitch). Loop 5 (rnd/spring.py):
                                   # replaces the "85 % everywhere" assumption; K tuned for a 0.5 kg typical payload
    bone_mass_upper: float = _AM["bone_mass_upper"]  # ARM_MOTOR_SETS (D20). GIM6010: D24/D19 (2026-11-27, rnd/parts/arm.py masses): 98 mm housing, no elbow KX74 socket / torque arm, Oldham drive + bolted-can nose/flange/cap, integral 40T = −0.06; + 0.10 for the D13 elbow stage metal (steel shaft, alu pulley, 6801s) the loop-38 roll-up never counted. Loop 40: +98 g housing stiffener (A8, P2 FEA; rnd/parts/arm.py housing_stiffener). FROM CAD (loop 23 roll-up: shell + cap + torque arm 216 g, cassette 52 g, CF neck 10 g, 40T hub 92 g, elbow small hub + KX74 108 g, bearings 90 g, hardware 70 g ESTIMATE = 0.638 kg; +9 g for the 110 mm housing = 0.647), rounded up
    bone_mass_fore: float = 0.28   # D24 (2026-11-27): wrist KX56 socket (57 g) → bolted output flange + can flange + forearm rim (+23 g). FROM CAD (loop 9): side link with CF Ø30 tube + wrist KX56 socket + bearings + fasteners
    bone_wall_mm: float = 1.2      # printed shell wall (rnd/bones.py: strength is never binding)
    cf_tubes: bool = True          # carbon-tube neck + forearm (loop 9: bone-bending sag at the hand 4.1 → 1.5 mm)
    gripper_mech_mass: float = 0.28  # D24 (2026-11-27, hand.masses 0.390 → 0.365): KX56 hub + collar → palm boss extension + 6 × M3. FROM CAD (loop 16 gripper v2: fin-ray TPU fingers, tactile pads, palm ToF; rnd/parts/hand.py)
    wrist_cam_mass: float = 0.05
    payload: float = 1.0           # kg at the grasp point (R2)
    # Elbow motor placement: None = at the elbow; a distance (m from the shoulder pitch axis) = motor sits up the
    # upper arm, just past the yaw joint, and drives the elbow through a long belt (precedent: Berkeley Blue moves
    # motors back along belts). The elbow then carries only a pulley hub + bearings.
    elbow_motor_at: float | None = None
    pitch_module_mass: float = _AM["pitch_module_mass"]    # ARM_MOTOR_SETS (D20). GIM6010: D24/D19 (2026-11-27, shoulder.masses 0.758 → 0.729): bolted cartridge + Oldham, no KX74 socket / torque fork, integral 60T, bolted pitch hub + outer wall. FROM CAD (loop 7; loop 8 lightening 0.81 → 0.76; loop 10 60T pulley 0.79, rnd/parts/shoulder.py): bracket, pitch shaft + 50T pulley + balancer lever +
                                       # KX-V ferrule, small hub + KX74 socket, bearings, spring/cable/pulley A, V-band clamp. Torso side.
    roll_stage_mass: float = _RD["roll_stage_mass"]      # inline: ROLL_DRIVES. Belt: FROM CAD (loop 40, rnd/parts/roll.py masses(split=True)): ferrule, roll link + spring ring, roll shaft + split 72T + spring hub, D13 stage + KX74 socket, 2×61808. Loop 10: 0.58
    yaw_stage_mass: float = _RD["yaw_stage_mass"]       # inline: ROLL_DRIVES. Belt: FROM CAD (loop 40, incl. the A8 box walls): clevis + yaw block, yaw shaft + flange + split 40T + spring ring, D13 stage + KX74 socket, 2×61808. Loop 10: 0.47
    elbow_hub_mass: float = 0.12   # big pulley + hub + 2×61808 + tensioner when the motor is remote (ESTIMATE)
    wrist_pitch_range_deg: float = 120.0   # ±140 folded the hand into the forearm (collision study); ±120 is realistic and reaches more
    # Wrist roll ± range (2026-11-27, harness track finding): ±180° needs 432° of clock-spring capacity (× 1.2) and the 14–24 mm
    # palm annulus gives 302° (rnd/harness.py). r_out 27.5 would need an Ø55 cassette on a free r 14 hub, but the palm's axis
    # is taken (servo horn → dog → spindle in 2 × 61804), so the cassette could only go around the Ø38 bearing skirt (r_in 20 →
    # r_out 35.5, Ø74 vs the 46 mm hand) or under the gripper base (+≈ 17 mm of l_hand). The parallel-jaw gripper is
    # symmetric under 180°, so ±90° already reaches every jaw orientation; ±120° keeps 30° of overlap for unwinding → 288° needed.
    wrist_roll_range_deg: float = 120.0


@dataclass
class Base:
    layout: str = "xdrive"         # xdrive (4 omni) — kiwi tip margin was only ~50 mm (twin v0)
    radius: float = 0.25           # shell radius
    wheel_radius_pos: float = 0.21 # wheel contact points from centre
    wheel_d: float = 0.1524        # 6" omni
    wheel_motor: str = "GIM6010-8"
    clearance: float = 0.025
    height: float = 0.13
    struct_mass: float = 2.0       # FROM CAD (build-out, rnd/parts/base.py): pods, wheel shafts, KX74 wheel hubs, ribbed deck, cans, 8×61808, screws (was 1.4)
    shell_mass: float = 0.33       # FROM CAD: four skirt quarters
    electronics_mass: float = 0.35 # DC-DC, CAN, flow sensor, wiring, e-stop
    lidar_mass: float = 0.15
    coupling_mass: float = 0.25    # upper/base coupling + cam lever + connector
    battery_mass: float = 1.10     # loop 13: Lumenier NAV-class 6S 10 Ah 21700 pack 975 g + 30 A BMS + printed sleeve (was 1.5 ESTIMATE)
    battery_wh: float = 222.0


@dataclass
class Upper:
    mast_w: float = 0.060          # square mast tube width
    lift_min: float = 0.35         # shoulder height (z) at lowest (reach sweep)
    lift_max: float = 1.05         # reach sweep: 0.95 -> 1.05
    spine_len: float = 1.06        # extrusion length
    spine_mass_per_m: float = 1.25 # loop 10 (rnd/mast.py): 60×60×2 Al square tube, torso sway 1.4 mm (2040 swayed 5.5 mm)
    lift_motor_mass: float = 0.81  # FROM CAD (build-out, rnd/parts/upper.py): GIM cartridge + 20T + mount + 2.4 m belt + idler + ratchet strip
    lift_motor: str = "GIM6010-8"
    carriage_mass: float = 0.58    # FROM CAD: printed sleeve + 8×625 bearings + pawl + solenoid
    torso_struct_mass: float = 0.867 # FROM CAD: rounded two-piece body enclosing the pitch modules (looks pass) + electronics tray
    compute_mass: float = 0.45     # Pi 5 + Jetson option + CAN + DC-DC
    head_mass: float = 0.94        # FROM CAD (looks pass) 0.82 + D11 (loop 28): 4.3" screen board ~90 g (ESTIMATE), shell +16 mm ~25 g, larger bezel ~8 g, − eye LEDs
    head_on_mast: bool = True      # owner suggestion (2026-11-25): head fixed on top of the spine; torso + arms ride the lift below it
    cable_mass: float = 0.25
    handle_mass: float = 0.0       # now part of the head bracket (FROM CAD)
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


def with_roll_drive(d: Design, drive: str) -> Design:
    """Copy of d with the shoulder-roll drive swapped (offset, yaw drop, actuator, stage masses from ROLL_DRIVES). The CAD
    (rnd/parts/roll.py, arm.py) follows PIP_ROLL_DRIVE at import, so analyses that read the CAD need the env var too."""
    v = ROLL_DRIVES[drive]
    j = dict(d.arm.joints)
    j["shoulder_roll"] = v["shoulder_roll"]
    keep = {k: v[k] for k in ("roll_offset_y", "yaw_drop", "roll_stage_mass", "yaw_stage_mass")}
    return replace(d, arm=replace(d.arm, roll_drive=drive, joints=j, **keep))


def with_arm_motors(d: Design, name: str) -> Design:
    """Copy of d with the D20 actuator set swapped (motors, ratios, elbow pulley, CAD masses from ARM_MOTOR_SETS). The CAD
    follows PIP_ARM_MOTORS at import, so analyses that read the CAD need the env var too."""
    v = ARM_MOTOR_SETS[name]
    j = dict(d.arm.joints)
    j["shoulder_pitch"], j["elbow"] = v["shoulder_pitch"], v["elbow"]
    keep = {k: v[k] for k in ("elbow_teeth", "elbow_belt_c", "bone_mass_upper", "pitch_module_mass")}
    return replace(d, arm=replace(d.arm, arm_motors=name, joints=j, **keep))
