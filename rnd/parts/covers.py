"""D23 concept covers (owner, 2026-11-27: enclose the joints, look consumer-facing, no pinch points).

First pass for looks and envelope only: soft shells over what is still exposed. Each cover lives in the frame of the body
it rides on, so it moves with its joint in robot.build(covers=True). Not yet in the clash/spacing checks; rnd/pinch.py
(queued) will measure gaps, finger access and the swept clearances before these become real parts.
"""
from __future__ import annotations

from build123d import Box, Pos

from .soft import soft_box, soft_shell


def pitch_frame():
    """Rides on the pitch body (static w.r.t. roll): a dome over the roll link, the roll cartridge, the 80T pulley and its
    belt, down to just above the roll axis; open underneath where the yaw block swings out."""
    from . import roll
    x0, x1, y0, y1, z0, z1 = -122.0, 62.0, 4.0, roll.ROLL_Y + 72, -30.0, roll.ROLL_C + 78
    d = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * soft_shell(x1 - x0, y1 - y0, z1 - z0, 46, 34, 2.2)
    d -= Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0 - 6, y1 - y0 - 6, 8)
    d -= Pos(0, roll.ROLL_Y, -30) * Box(x1 - x0 + 10, 120, 70)                         # the arm exits below the roll axis
    return [("cover_shoulder", d, "white")]


def roll_frame():
    """Rides on the roll output: a sleeve round the whole yaw block (both plates, the belt, the yaw cartridge, its KX
    collar and torque fork), open at the top (the clevis arms) and the bottom (the yaw flange)."""
    from . import roll
    z0, z1 = roll.P2[0] - 4, roll.ZF + roll.ZC + 14
    c = Pos(-38, roll.ROLL_Y, (z0 + z1) / 2) * soft_shell(198, 122, z1 - z0, 46, 18, 2.0)
    c -= Pos(-38, roll.ROLL_Y, z0) * Box(194, 118, 6)
    c -= Pos(0, roll.ROLL_Y, z1) * Box(80, 70, 8)                                       # clevis arms rise through the top
    return [("cover_yaw", c, "white")]


def arm_frame():
    """Rides on the upper bone: a dome over the elbow cartridge where it leaves the housing's −Y bay."""
    from . import arm
    zS = -arm.ELBOW_Z + arm.BELT_C
    d = Pos(0, -70, zS) * soft_shell(104, 40, 104, 30, 14, 2.0)
    d -= Pos(0, -52, zS) * Box(108, 6, 108)                                             # open against the housing wall
    door = Pos(56.5, -20.5, zS) * soft_box(4, 27, 98, 1.5, 1.2)                          # door over the KX collar twist window
    return [("cover_elbow_bay", d, "white"), ("cover_collar_door", door, "white")]


def forearm_frame():
    """Rides on the forearm: a sleeve over the side link and its carbon tube (elbow disc to wrist can)."""
    from . import arm
    z0, z1 = -arm.FORE_L + 42, -46
    boot = Pos(0, (arm.WRIST_Y0 + arm.WRIST_Y1) / 2 + 4, -arm.FORE_L) * soft_shell(80, arm.WRIST_Y1 - arm.WRIST_Y0 + 22, 80, 30, 18, 1.8)
    boot -= Pos(0, arm.WRIST_Y0 - 8, -arm.FORE_L) * Box(90, 16, 90)                    # open toward the hand
    return [("cover_forearm", Pos(0, arm.FORE_Y, (z0 + z1) / 2) * soft_shell(48, 36, z1 - z0, 16, 12, 1.6), "white"),
            ("cover_wrist", boot, "white")]


def mast_bellows(lift_mm, z_base_top, z_top, mast_x, w):
    """Telescoping/bellows covers on the mast above and below the torso (lift pinch point, R17)."""
    from . import upper
    out = []
    lo0, lo1 = z_base_top + 8, lift_mm - 120
    hi0, hi1 = lift_mm + 140, z_top - 6
    for n, a, b in (("bellows_low", lo0, lo1), ("bellows_high", hi0, hi1)):
        if b - a > 20:
            out.append((n, Pos(mast_x, 0, (a + b) / 2) * soft_box(w + 14, w + 14, b - a, 8, 2), "dark"))
    return out
