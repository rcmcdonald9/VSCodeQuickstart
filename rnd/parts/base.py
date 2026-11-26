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
    hb = -hub(KX74).bounding_box().min.Z                                                       # loop 40: follows the D15 socket depth (13.0)
    s = Pos(0, 0, -44) * (Cylinder(20, 44 - hb, align=MIN) - Cylinder(12, 44 - hb, align=MIN))   # Ø40 through the bearings (below the hub)
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
            (f"w{ang}_hub", T * hub(KX74), "dark"),            # D17 (loop 40, A11): bolted to the GIM output (4 × M4), no KX collar; same envelope
            (f"w{ang}_shaft", T * wheel_shaft(), "orange"), (f"w{ang}_pod", T * pod(), "white"),
            (f"w{ang}_wheel", T * omni_wheel(), "dark"),
            (f"w{ang}_torque_hanger", T * torque_hanger_pose(zc), "white")]


def torque_hanger_pose(zc):
    """Wheel torque hanger in the stage frame: pin at (x 30, y 0) behind the can floor (z = zc … zc + 10); the flange screws
    under the deck (stage x = deck underside); the fork is open downward (stage −x), so the battery top stays clear."""
    tp = cm.TORQUE_PIN
    reach = (Z1 - DECK_T) - Z_AX - tp["r"]
    return Pos(tp["r"], 0, zc) * cm.torque_hanger(reach, slot_in=4.0)


def deck_quarter(q) -> Part:
    """Quarter deck (printed, ribbed), with cut-outs for the wheels."""
    r = R_SHELL - 4
    p = Pos(0, 0, Z1 - DECK_T) * Cylinder(r, DECK_T, align=MIN)
    p &= Rot(0, 0, 90 * q + 45) * Pos(r / 2, r / 2, Z1 - DECK_T) * Box(r, r, DECK_T, align=MIN)      # one diagonal quadrant
    for k in range(4):                                                               # wheel cut-outs (the wheels rise to z 152)
        p -= Rot(0, 0, 90 * k) * Pos(R_W, 0, Z1 - DECK_T) * Box(130, 60, DECK_T, align=MIN)
    p -= Pos(-60 + 40, 0, Z1 - 20) * Box(22, 42, 40, align=MIN)                      # blind-mate plug pass-through
    return p


def rim() -> Part:
    """Looks pass v2: rounded rim ring on the deck edge (printed in 4 arcs with the skirt quarters)."""
    from build123d import fillet as _f
    r = Pos(0, 0, Z1) * (Cylinder(R_SHELL + 2, 8, align=MIN) - Cylinder(R_SHELL - 14, 8, align=MIN))
    try:
        r = _f(r.edges().group_by(Axis.Z)[-1], 3.8)
    except ValueError:
        pass
    for k in range(4):
        r -= Rot(0, 0, 90 * k) * Pos(R_W, 0, Z1 - 5) * Box(130, 60, 20, align=MIN)     # wheel cut-outs
    return r


def skirt_quarter(q) -> Part:
    s = Pos(0, 0, Z0) * (Cylinder(R_SHELL, Z1 - Z0 - DECK_T, align=MIN) - Cylinder(R_SHELL - 2, Z1 - Z0 - DECK_T, align=MIN))
    # loop 27: seams moved into the wheel windows (quarter q spans 90q … 90q + 90°), freeing the 45° azimuth for the dock pads
    s &= Rot(0, 0, 90 * q) * Pos(R_SHELL / 2 + 1, R_SHELL / 2 + 1, Z0) * Box(R_SHELL + 2, R_SHELL + 2, 200, align=MIN)
    for k in range(4):                                                               # wheel windows
        s -= Rot(0, 0, 90 * k) * Pos(R_SHELL, 0, Z0 - 1) * Box(40, 140, 2 * (Z_AX + 62 - Z0) + 2)
    if q == 0:
        s = s + dock_pad_carrier()
        for z, name in DOCK_PADS:
            w, h = (62, 13) if name != "ID" else (42, 11)
            s -= Rot(0, 0, DOCK_PAD_AZ) * Pos(R_SHELL - 0.5, 0, z) * Box(3, w, h)  # pads sit flush in the skirt
    return s


DOCK_PAD_AZ = 45.0              # azimuth of the docking contacts (rnd/dock_sim.py PAD_AZ), between the wheels at 0° and 90°
DOCK_PADS = ((60.0, "+"), (100.0, "−"), (40.0, "ID"))   # pad centre heights above the floor (match rnd/parts/dock.py)


def bumper_light() -> Part:
    """Light ring around the skirt. A12 (loop 40, review finding 13): it stood 4 mm proud at z 43–53 right across the dock
    pads (ID pad at 40, + at 60), so the dock's V faces would have hit the ring, not the pads. The ring now has a 30°
    gap centred on the pads."""
    r = Pos(0, 0, Z0 + 18) * (Cylinder(R_SHELL + 4, 10, align=MIN) - Cylinder(R_SHELL, 10, align=MIN))
    gap = Pos(R_SHELL, 0, Z0 + 13) * Box(2 * R_SHELL * math.sin(math.radians(15)) + 20, 2 * R_SHELL * math.sin(math.radians(15)), 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return r - Rot(0, 0, DOCK_PAD_AZ) * gap


def dock_pad_carrier() -> Part:
    """Loop 27: carrier inside the skirt at 45° for the docking contacts. Two brass pads 60 × 12 mm (+ at 60 mm, − at
    100 mm) and a 40 × 10 mm ID pad (40 mm) sit in 1.5 mm pockets, flush with the skirt; wires leave through Ø4 holes.
    Contact force is only the dock's leaf springs (~5 N each), so the 6 mm carrier is ample."""
    c = Pos(R_SHELL - 2 - 3, 0, 28) * Box(6, 84, 90, align=(Align.CENTER, Align.CENTER, Align.MIN))
    c &= Cylinder(R_SHELL - 1.0, 400)                                                # follow the skirt's inner curvature
    for z, name in DOCK_PADS:
        w, h = (62, 13) if name != "ID" else (42, 11)
        c -= Pos(R_SHELL - 1.5, 0, z) * Box(4, w, h)                               # pad pocket through the skirt wall
        c -= Pos(R_SHELL - 5, 0, z) * Rot(0, 90, 0) * Cylinder(2, 12)              # wire hole
    return Rot(0, 0, DOCK_PAD_AZ) * c


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
    out += [("rim", rim(), "white"), ("battery", battery(), "dark"), ("coupling", c, "white"), ("cam_lever", lever, "orange"), ("plug", plug, "dark"),
            ("lidar", Pos(R_SHELL - 70, 0, Z1) * Cylinder(28, 41, align=MIN), "dark"),
            ("bumper_light", bumper_light(), "orange"),
            ("base_electronics", Rot(0, 0, 45) * Pos(135, 0, Z0 + 10) * Box(50, 70, 30, align=MIN), "dark")]
    return out


def masses(density=1.27):
    pods = 4 * pod().volume / 1e6 * density * 0.6
    shafts = 4 * wheel_shaft().volume / 1e6 * density * 0.7
    hubs = 4 * hub(KX74).volume / 1e6 * density * 0.7                          # D17: no collars on the wheels
    deck = sum(deck_quarter(q).volume for q in range(4)) / 1e6 * density * 0.6
    skirt = sum(skirt_quarter(q).volume for q in range(4)) / 1e6 * density
    c, lever, plug = coupling()
    coup = (c.volume + lever.volume) / 1e6 * density * 0.6
    return {"pods": pods, "wheel shafts + flanges": shafts, "KX74 wheel hubs + collars": hubs, "deck quarters (ribbed)": deck,
            "skirt quarters": skirt, "mast coupling + lever": coup,
            "bearings 8×61808 (ESTIMATE)": 0.16, "cans for 4 wheel cartridges (printed)": 4 * cm.can().volume / 1e6 * density * 0.7,
            "screws, inserts, cam, connector (ESTIMATE)": 0.12}
