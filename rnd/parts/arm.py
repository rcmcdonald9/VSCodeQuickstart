"""Arm bones v0: upper arm and forearm shells built around the KX sockets and belt stages.

Layout (upper-arm frame: origin at the yaw interface, arm hanging along -Z, elbow axis along Y at z = -ELBOW_Z):
- Upper bone: flange (bolts to the yaw hub) → Ø72 neck tube → elbow housing that holds the elbow cartridge M
  (axis Y, 70 mm above the elbow axis), its pulley hub (2 × 61804), the belt, and the 40T elbow pulley on 2 × 61808.
- Forearm: a **side link** bolted to the elbow pulley hub on the +Y side. Its distal housing holds the wrist-pitch
  cartridge with the output facing **back toward the arm's centre plane** (-Y). The hand therefore sits close to the
  centre plane (net lateral offset ≈ 0), and it swings *beside* the forearm link instead of folding into it.
Walls are 1.6 mm (4 perimeters at 0.4 mm), printed solid. Every part fits in 256 mm when split as noted in `PRINT`.
"""
from __future__ import annotations

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .kx import KX56, KX74, hub as kx_hub, collar as kx_collar

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
WALL = 1.6                  # 4 perimeters at 0.4 mm; bending stress in the shells stays < 3 MPa (loop 4: 2.4 → 1.6 to save mass)
ELBOW_Z = 230.0             # yaw interface → elbow axis (params: l_upper 0.30 minus the 0.07 roll→yaw offset)
FORE_L = 280.0              # elbow axis → wrist axis (params: l_fore)
BELT_C = 70.0               # elbow belt centre distance (rnd/belts.py)
HOUSING = dict(x=98.0, y=108.0, z0=-272.0, z1=-100.0)
J4310 = dict(od=57.0, h=46.0)   # DM-J4310 (actuators.yaml)
FORE_Y = HOUSING["y"] / 2 + 12.0  # forearm link centre plane (+Y side link)


def _rbox(x, y, z, r, align=(Align.CENTER, Align.CENTER, Align.CENTER)):
    b = Box(x, y, z, align=align)
    return fillet(b.edges().filter_by(Axis.Z), min(r, x / 2 - 0.1, y / 2 - 0.1))


def _hollow_rbox(x, y, z, r, wall=WALL):
    return _rbox(x, y, z, r) - _rbox(x - 2 * wall, y - 2 * wall, z - 2 * wall, max(r - wall, 0.5))


def _ring(od, idd, t):
    return Cylinder(od / 2, t) - Cylinder(idd / 2, t)


def upper_bone() -> Part:
    h = HOUSING
    p = Pos(0, 0, -6) * Cylinder(45, 6, align=MIN)                                    # yaw-hub flange
    p += Pos(0, 0, h["z1"] - 2) * (Cylinder(36, 96, align=MIN) - Cylinder(36 - WALL, 96, align=MIN))  # neck tube
    hz = h["z1"] - h["z0"]
    p += Pos(0, 0, (h["z0"] + h["z1"]) / 2) * _hollow_rbox(h["x"], h["y"], hz, 18)
    # bearing bosses through the ±Y walls: elbow axis (61808, OD 52) and the pulley-hub journals (61804, OD 32)
    for y in (-h["y"] / 2, h["y"] / 2):
        for z, od in ((-ELBOW_Z, 52.0), (-ELBOW_Z + BELT_C, 32.0)):
            p += Pos(0, y, z) * Rot(90, 0, 0) * _ring(od + 10, od, 8)
    # openings: cartridge bay on -Y (the cartridge drops in from the side), elbow hub exit on +Y
    p -= Pos(0, -h["y"] / 2, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Cylinder(cm.MOTOR["od"] / 2 + 4, 20)
    p -= Pos(0, h["y"] / 2, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26, 20)
    # torque-pin slot for the elbow cartridge (radial slot, 6.4 wide)
    p -= Pos(0, -h["y"] / 2 + 1, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Pos(30 + 5, 0, 0) * Box(16, 6.4, 12)
    return p


def forearm() -> Part:
    """Forearm frame: origin on the elbow axis at the upper arm's centre plane; arm hangs along -Z."""
    y = FORE_Y
    link_t = 22.0
    p = Pos(0, y, 0) * Rot(90, 0, 0) * Cylinder(40, link_t)                            # hub disc on the elbow pulley hub
    p -= Pos(0, y, 0) * Rot(90, 0, 0) * Cylinder(21, link_t)
    body_len = FORE_L - 40 - 38
    p += Pos(0, y, -40 - body_len / 2) * Rot(90, 0, 0) * _hollow_rbox(56, body_len, link_t, 10)
    # wrist housing: holds the wrist cartridge (axis Y, output toward -Y)
    can_d = J4310["od"] + 2 * 3.0 + 0.6
    wy = J4310["h"] + 4 + 6
    p += Pos(0, y + link_t / 2 - wy / 2, -FORE_L) * Rot(90, 0, 0) * (Cylinder(can_d / 2 + WALL, wy) - Cylinder(can_d / 2, wy - WALL))
    p += Pos(0, y, -FORE_L + 38) * Box(56, link_t, 12)                                  # web joining link and wrist can
    return p


def elbow_pulley_hub() -> Part:
    """40T elbow pulley + Ø40 shaft (runs in 2 × 61808) + bolt flange for the forearm."""
    from .belt_stage import pulley
    p = Pos(0, 0, 0) * Rot(90, 0, 0) * Cylinder(20, HOUSING["y"] + 12)
    p += Rot(-90, 0, 0) * Pos(0, 0, -8) * pulley(40, 16)
    return p - Rot(90, 0, 0) * Cylinder(15, 300)              # hollow shaft (cable pass-through to the forearm)


PRINT = {
    "upper_bone": "split at z = -100 (neck/housing) and along the Y centre plane → 3 pieces ≤ 256 mm, bolted + doweled",
    "forearm": "one piece, printed on its side (link plane on the bed), 280 mm diagonal fits the 256 mm bed corner-to-corner",
}


def masses(density=1.27):
    """Printed mass (kg): shells are solid walls; hubs/collars ~70 % effective density (4 perimeters + 25 % infill)."""
    from .belt_stage import small_hub
    from .kx import retainer
    shells = {"upper bone shell": upper_bone().volume, "forearm shell": forearm().volume}
    kxv = lambda k: kx_hub(k).volume + kx_collar(k).volume + retainer(k).volume
    sockets = {"KX74 socket (elbow)": kxv(KX74), "KX56 sockets ×2 (yaw, wrist)": 2 * kxv(KX56),
               "elbow pulley hub (40T)": elbow_pulley_hub().volume, "elbow small-pulley hub": small_hub().volume - kx_hub(KX74).volume}
    out = {k: v / 1e6 * density for k, v in shells.items()}
    out.update({k: v / 1e6 * density * 0.7 for k, v in sockets.items()})
    out["bearings: 2×61808 + 2×61804 + 4×61805 (ESTIMATE)"] = 0.020 * 2 + 0.008 * 2 + 0.012 * 4
    out["belt, pins, balls, screws (ESTIMATE)"] = 0.06
    return out


def assembly(elbow_deg: float = -55.0, wrist_deg: float = -35.0):
    """(name, part, colour) for renders. Upper-arm frame; elbow bends about Y (negative = forward, as in the twin)."""
    from .belt_stage import small_hub, belt_loop, pulley
    from .cartridge_s import sleeve, servo_dummy
    from .kx import collar, balls_mated
    out = [("upper_bone", upper_bone(), "white")]
    # elbow stage, built in the stage frame (cartridge +Z, big pulley at +X), then mapped: +Z→-Y, +X→-Z
    st = Pos(0, -20, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Rot(0, 0, -90)
    zc = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + cm.MOTOR["h"]
    h = small_hub()
    zp = h.bounding_box().min.Z + 7 + 1 + 1.2
    out += [("e_can", st * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.can(), "white"),
            ("e_motor", st * Pos(0, 0, zc) * Rot(180, 0, 0) * cm.motor_dummy(), "dark"),
            ("e_collar", st * collar(KX74), "orange"), ("e_hub", st * h, "dark"),
            ("e_belt", st * Pos(0, 0, zp + 0.5) * belt_loop(31.8, 63.7, BELT_C), "dark"),
            ("e_bigpulley", st * Pos(BELT_C, 0, zp - 1.2) * pulley(40, 16, bore=30), "orange")]
    # yaw cartridge (DM-J4310 in a KX56 cartridge, placeholder) above the flange
    out += [("yaw_cart", Pos(0, 0, 0) * Cylinder(32, 52, align=MIN), "dark"), ("yaw_collar", Pos(0, 0, 0) * Cylinder(39, 12, align=MIN) - Cylinder(30, 12, align=MIN), "orange")]
    fe = Pos(0, 0, -ELBOW_Z) * Rot(0, elbow_deg, 0)
    out += [("forearm", fe * forearm(), "white")]
    # wrist cartridge + hand (hand plane near y = 10, beside the forearm link)
    wr = fe * Pos(0, 0, -FORE_L)
    out += [("wrist_cart", wr * Pos(0, FORE_Y, 0) * Rot(90, 0, 0) * Cylinder(J4310["od"] / 2, J4310["h"]), "dark"),
            ("wrist_collar", wr * Pos(0, FORE_Y - 34, 0) * Rot(90, 0, 0) * (Cylinder(39, 12) - Cylinder(28, 12)), "orange")]
    hand = wr * Pos(0, 12, 0) * Rot(0, wrist_deg, 0)
    out += [("palm", hand * Pos(0, 0, -20) * _rbox(56, 34, 40, 8), "white"),
            ("wroll", hand * Pos(-cs_shift(), 0, -40) * Rot(180, 0, 0) * Pos(0, 0, 0) * sleeve(), "white"),
            ("finger_a", hand * Pos(0, 13, -112) * _rbox(14, 8, 56, 3), "dark"),
            ("finger_b", hand * Pos(0, -13, -112) * _rbox(14, 8, 56, 3), "dark")]
    return out


def cs_shift():
    from .cartridge_s import SERVO
    return SERVO["l"] / 2 - SERVO["shaft_off"]
