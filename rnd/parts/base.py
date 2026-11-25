"""Base v0: 4 omni wheel modules ("+" layout, one wheel forward, axles radial), deck, skirt, battery bay, mast coupling, lidar.

Wheel module (same pattern as every joint): GIM cartridge (torque only) → KX74 wheel hub on a Ø40 shaft in 2 × 61808 in
the pod plates → wheel flange (6 × #10 on a 1.875" circle, AndyMark DuraOmni) → 6" omni wheel. Wheel loads never reach
the motor bearing. The cartridge points inward; the four cartridges leave a centre bay for the 6S battery (slides out
from the back). Deck and skirt are printed in quarters (each ≤ 256 mm).
"""
from __future__ import annotations

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from ..params import BASELINE
from . import cartridge_m as cm
from .kx import KX74, collar, hub_pip as hub

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
D = BASELINE
R_W = D.base.wheel_radius_pos * 1000          # 210
WHEEL_D, WHEEL_W = D.base.wheel_d * 1000, 46.0
Z_AX = WHEEL_D / 2                            # axle height 76 mm
R_SHELL = D.base.radius * 1000
Z0, Z1 = D.base.clearance * 1000, (D.base.clearance + D.base.height) * 1000
HUB_R = R_W - WHEEL_W / 2 - 48.0              # KX face radius (wheel inner face − hub/bearing stack) = 139
DECK_T = 6.0


def _T(ang):
    """Stage frame (KX face z = 0, cartridge +Z) → world: cartridge points radially inward at wheel angle `ang`."""
    return Rot(0, 0, ang) * Pos(HUB_R, 0, Z_AX) * Rot(0, -90, 0)


def omni_wheel() -> Part:
    """6" DuraOmni placeholder in the stage frame: wheel spans z = -48 … -94 (axis Z)."""
    w = Pos(0, 0, -48 - WHEEL_W) * (Cylinder(60, WHEEL_W, align=MIN) - Cylinder(15, WHEEL_W, align=MIN))
    for row, dz in ((0, -60), (1, -82)):
        for k in range(10):
            a = 2 * math.pi * (k + 0.5 * row) / 10
            w += Pos(66 * math.cos(a), 66 * math.sin(a), dz) * Rot(0, 0, math.degrees(a) + 90) * Rot(0, 90, 0) * Cylinder(10, 26)
    return w


def wheel_shaft() -> Part:
    s = Pos(0, 0, -44) * (Cylinder(20, 31.8, align=MIN) - Cylinder(12, 31.8, align=MIN))       # Ø40 through the bearings (below the hub)
    s += Pos(0, 0, -48) * (Cylinder(30, 4, align=MIN) - Cylinder(12, 4, align=MIN))            # wheel flange
    return s


def pod() -> Part:
    """Two bearing plates (61808 seats) + top bridge, in the stage frame (stage +x = world +z)."""
    zlo, zhi = Z0 - Z_AX, (Z1 - DECK_T) - Z_AX          # plates from the ground-clearance line to the deck underside
    p = None
    for z0 in (-22.0, -40.0):
        pl = Pos((zlo + zhi) / 2, 0, z0) * Box(zhi - zlo, 80, 8, align=MIN)
        pl -= Pos(0, 0, z0) * Cylinder(26, 8, align=MIN)
        pl -= Pos(-40, 0, z0) * Box(22, 50, 8, align=MIN)                  # lightening below the seat
        p = pl if p is None else p + pl
    bridge = Pos(zhi - 4, 0, -40) * Box(8, 80, 26, align=MIN)                # ties the plates under the deck
    return p + bridge


def wheel_module(ang):
    T = _T(ang)
    zc = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    return [(f"w{ang}_can", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.can(), "white"),
            (f"w{ang}_motor", T * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.motor_dummy(), "dark"),
            (f"w{ang}_collar", T * collar(KX74), "orange"), (f"w{ang}_hub", T * hub(KX74), "dark"),
            (f"w{ang}_shaft", T * wheel_shaft(), "orange"), (f"w{ang}_pod", T * pod(), "white"),
            (f"w{ang}_wheel", T * omni_wheel(), "dark")]


def deck_quarter(q) -> Part:
    """Quarter deck (printed, ribbed), with cut-outs for the wheels."""
    r = R_SHELL - 4
    p = Pos(0, 0, Z1 - DECK_T) * Cylinder(r, DECK_T, align=MIN)
    p &= Rot(0, 0, 90 * q + 45) * Pos(r / 2, r / 2, Z1 - DECK_T) * Box(r, r, DECK_T, align=MIN)      # one diagonal quadrant
    for k in range(4):                                                               # wheel cut-outs (the wheels rise to z 152)
        p -= Rot(0, 0, 90 * k) * Pos(R_W, 0, Z1 - DECK_T) * Box(130, 60, DECK_T, align=MIN)
    p -= Pos(-60 + 40, 0, Z1 - 20) * Box(22, 42, 40, align=MIN)                      # blind-mate plug pass-through
    return p


def skirt_quarter(q) -> Part:
    s = Pos(0, 0, Z0) * (Cylinder(R_SHELL, Z1 - Z0 - DECK_T, align=MIN) - Cylinder(R_SHELL - 2, Z1 - Z0 - DECK_T, align=MIN))
    s &= Rot(0, 0, 90 * q + 45) * Pos(R_SHELL / 2 + 1, R_SHELL / 2 + 1, Z0) * Box(R_SHELL + 2, R_SHELL + 2, 200, align=MIN)
    for k in range(4):                                                               # wheel windows
        s -= Rot(0, 0, 90 * k) * Pos(R_SHELL, 0, Z0 - 1) * Box(40, 140, 2 * (Z_AX + 62 - Z0) + 2)
    return s


def battery():
    return Pos(0, 0, Z0 + 8) * Box(150, 75, 65, align=MIN)          # 6S 10 Ah pack envelope (ESTIMATE)


def coupling():
    """Mast socket on the deck at x = -60: printed block + cam lever (placeholder) + blind-mate plug housing."""
    c = Pos(-60, 0, Z1) * (Box(100, 100, 40, align=MIN) - Box(60.6, 60.6, 40, align=MIN))
    lever = Pos(-60, 58, Z1 + 20) * Box(70, 10, 12)
    plug = Pos(-60 + 40, 0, Z1 - 20) * Box(20, 40, 20, align=MIN)
    return c, lever, plug


def module():
    out = []
    for ang in (0, 90, 180, 270):
        out += wheel_module(ang)
    for q in range(4):
        out += [(f"deck{q}", deck_quarter(q), "white"), (f"skirt{q}", skirt_quarter(q), "white")]
    c, lever, plug = coupling()
    out += [("battery", battery(), "dark"), ("coupling", c, "white"), ("cam_lever", lever, "orange"), ("plug", plug, "dark"),
            ("lidar", Pos(R_SHELL - 70, 0, Z1) * Cylinder(28, 41, align=MIN), "dark"),
            ("bumper_light", Pos(0, 0, Z0 + 18) * (Cylinder(R_SHELL + 4, 10, align=MIN) - Cylinder(R_SHELL, 10, align=MIN)), "orange"),
            ("base_electronics", Rot(0, 0, 45) * Pos(135, 0, Z0 + 10) * Box(50, 70, 30, align=MIN), "dark")]
    return out


def masses(density=1.27):
    pods = 4 * pod().volume / 1e6 * density * 0.6
    shafts = 4 * wheel_shaft().volume / 1e6 * density * 0.7
    hubs = 4 * (hub(KX74).volume + collar(KX74).volume) / 1e6 * density * 0.7
    deck = sum(deck_quarter(q).volume for q in range(4)) / 1e6 * density * 0.6
    skirt = sum(skirt_quarter(q).volume for q in range(4)) / 1e6 * density
    c, lever, plug = coupling()
    coup = (c.volume + lever.volume) / 1e6 * density * 0.6
    return {"pods": pods, "wheel shafts + flanges": shafts, "KX74 wheel hubs + collars": hubs, "deck quarters (ribbed)": deck,
            "skirt quarters": skirt, "mast coupling + lever": coup,
            "bearings 8×61808 (ESTIMATE)": 0.16, "cans for 4 wheel cartridges (printed)": 4 * cm.can().volume / 1e6 * density * 0.7,
            "screws, inserts, cam, connector (ESTIMATE)": 0.12}
