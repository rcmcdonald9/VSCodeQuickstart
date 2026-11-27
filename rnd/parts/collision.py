"""Collision hulls of the CAD arm for the MuJoCo twin (one convex piece per STL; MuJoCo collides convex hulls).

Frames match the twin: upper pieces in the upper-arm body frame (origin at the yaw joint, arm along -Z), forearm pieces in
the forearm body frame (origin on the elbow axis). CAD +Y = the arm's outboard side (forearm side link). Right-arm copies
are mirrored in Y (the right arm sits at -Y). Units: mm; the twin scales by 0.001.
"""
from __future__ import annotations

from pathlib import Path

from build123d import Box, Cylinder, Plane, Pos, Rot, export_stl, mirror

from . import arm, cartridge_m as cm, roll
from .kx import KX74

OUT = Path(__file__).resolve().parents[1] / "out" / (("collision" if not roll.INLINE else "collision_inline") + ("_d20" if arm.D20 else ""))   # D24: one hull set per roll drive, D20: per arm-motor set (= twin.COLL)


def arm_sh_x():
    from ..params import BASELINE
    return BASELINE.arm.shoulder_x * 1000


def arm_sh_y():
    from ..params import BASELINE
    return BASELINE.arm.shoulder_y * 1000


def _sh_inline():
    """D24 inline roll, static side (pitch-body frame): GIM8108-36 + floor, the roll-link frame, the spring ring."""
    x0, x1 = roll.X_FLOOR[0], roll.ROLL_SPR_X[1] + 1.6
    return {"rolllink": Pos((x0 + x1) / 2, 13, 6) * Box(x1 - x0, 10, 68),
            "rollcart": Pos(x0, roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(roll.R_MOT, roll.X_MOT[1] - x0, align=arm.MIN),
            "cap": Pos(roll.ROLL_SPR_X[0], roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(40, x1 - roll.ROLL_SPR_X[0], align=arm.MIN)}


def _roll_inline():
    """D24 inline roll, roll body (origin on the roll axis): yaw block + band, the yaw cartridge + rear arm + fork, the +X arm."""
    x1 = roll.X_PL1 + 8
    zt = roll.P1[1] + 1.5
    return {"yawblock": Pos((-130 + x1) / 2, 0, (-roll.YAW_DROP + zt) / 2) * Box(x1 + 130, 100, zt + roll.YAW_DROP),
            "yawcart": Pos((-132 + roll.X_ARM_R[1]) / 2, 0, (zt + 36) / 2) * Box(roll.X_ARM_R[1] + 132, 106, 36 - zt),
            "xarm": Pos(sum(roll.X_ARM_F) / 2, 0, (zt + roll.R_ARM) / 2) * Box(roll.X_ARM_F[1] - roll.X_ARM_F[0], 2 * roll.R_ARM, roll.R_ARM - zt)}


def pieces():
    h = arm.HOUSING
    cart_len = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + arm.MOT["h"]          # D20: arm.MOT = the elbow motor
    y0 = -20.0                                                     # cartridge KX face plane (arm.assembly)
    fy, link_t = arm.FORE_Y, 22.0
    wy = arm.J4310["h"] + 4 + 6
    return {
        "upper": {
            "neck": Pos(0, 0, h["z1"] - 2) * Cylinder(36, 104 - 6 + 2, align=arm.MIN) + Pos(0, 0, -6) * Cylinder(45, 6, align=arm.MIN),
            "housing": Pos(0, 0, (h["z0"] + h["z1"]) / 2) * Box(h["x"], h["y"] + 16, h["z1"] - h["z0"]),
            "elbow_cart": Pos(0, y0 - cart_len / 2, -arm.ELBOW_Z + arm.BELT_C) * Rot(90, 0, 0) * Cylinder(arm.MOT["od"] / 2 + 3.3, cart_len),
        },
        "fore": {
            "link": Pos(0, fy, -arm.FORE_L / 2 + 10) * Box(56, link_t, arm.FORE_L - 2 * 38),
            "wrist": Pos(0, arm.WRIST_Y0 - 3, -arm.FORE_L) * Rot(-90, 0, 0) * Cylinder(35, arm.WRIST_Y1 - arm.WRIST_Y0 + 3, align=arm.MIN),
        },
        # wrist-pitch body (hand frame = CAD hand frame): palm + KX56 collar; hand (roll) body: gripper, offset +40 mm
        "wristb": {
            "palm": Pos(7, 0, -22) * Box(74, 40, 94),
            "wpcollar": Pos(0, 23, 0) * Rot(-90, 0, 0) * Cylinder(43, 19, align=arm.MIN),
        },
        # pitch body ("sh", frame = detach plane at y 0): roll link + roll cartridge
        "sh": _sh_inline() if roll.INLINE else {
            "rolllink": Pos(0, 61, 38) * Box(92, 106, 168),
            "rollcart": Pos(-50, roll.ROLL_Y, roll.ROLL_C) * Box(66, 106, 102),
            "cap": Pos(-34, 75, 95) * Box(134, 130, 114),                       # looks-pass shoulder cap
        },
        # roll body (origin on the roll axis): clevis + yaw block + yaw cartridge
        "roll": _roll_inline() if roll.INLINE else {
            "yawblock": Pos(-38, 0, (-roll.YAW_DROP + 26) / 2) * Box(172, 92, roll.YAW_DROP + 26),
        },
        # torso body: the torso-side pitch modules (module frame shifted so the detach plane is at the pitch body's y = 0)
        "torso": {
            "pitchmod": Pos(arm_sh_x() - 45, arm_sh_y() - 67.5, 10) * Box(190, 135, 150),
        },
        "hand": {
            "gripbase": Pos(5, 0, 40 - 98) * Box(104, 52, 30),
            "jaws": Pos(-2, 3, 40 - 128) * Box(74, 46, 48),
        },
    }


def export():
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}
    for body, parts in pieces().items():
        for n, p in parts.items():
            for side, q in (("l", p), ("r", mirror(p, Plane.XZ))):
                f = OUT / f"{body}_{n}_{side}.stl"
                export_stl(q, str(f))
                files[(body, n, side)] = f
    return files


if __name__ == "__main__":
    print(len(export()), "collision meshes →", OUT)
