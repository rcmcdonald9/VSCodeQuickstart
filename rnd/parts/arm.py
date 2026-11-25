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
from .soft import soft_shell
from .kx import KX56, KX74, hub as kx_hub, collar as kx_collar

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
WALL = 1.2                  # 3 perimeters at 0.4 mm. Loop 9 (rnd/bones.py): strength is never binding; stiffness comes from the CF tubes
from ..params import BASELINE as _D
ELBOW_Z = (_D.arm.l_upper - _D.arm.yaw_drop) * 1000   # yaw interface → elbow axis (loop 10: 230 → 200 mm)
FORE_L = 280.0              # elbow axis → wrist axis (params: l_fore)
BELT_C = 70.0               # elbow belt centre distance (rnd/belts.py)
HOUSING = dict(x=98.0, y=108.0, z0=-ELBOW_Z - 42.0, z1=-ELBOW_Z + 70.0 + 48.0)
J4310 = dict(od=57.0, h=46.0)   # DM-J4310 (actuators.yaml)
FORE_Y = HOUSING["y"] / 2 + 12.0  # forearm link centre plane (+Y side link)
# elbow belt cassette (housing frame, y spans of the two plates): set by the small-pulley hub's journals, whose position
# is fixed by the cartridge stack (hub face at y = -20). Belt plane: small pulley y 1.4 … 17.4.
BELT_Y = 9.4
CAS_A, CAS_B = (-7.4, -0.4), (18.9, 25.9)      # 0.4 / 0.3 mm axial clearance to the hub back face and the pulley flanges
CAS_POST_X, CAS_POST_Z = 32.0, -ELBOW_Z + BELT_C - 10.0
CAP_Z = HOUSING["z0"] + 10.0          # housing / bottom-cap split
WRIST_Y0, WRIST_Y1 = 43.0, 96.0   # wrist can span (holds the DM-J4310 cartridge; its KX56 face at y ≈ 33.6, hand centre at y = 0)


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
    # neck (loop 9): carbon tube Ø40×1 (see cf_neck()) bonded into two printed sockets, one on the flange and one on the housing
    for z in (-6 - 20, h["z1"] - 2):
        p += Pos(0, 0, z) * (Cylinder(23, 22, align=MIN) - Cylinder(20.1, 22, align=MIN))
    hz = h["z1"] - h["z0"]
    p += Pos(0, 0, (h["z0"] + h["z1"]) / 2) * soft_shell(h["x"], h["y"], hz, 34, 26, WALL)     # looks pass: soft on every edge
    # bearing bosses through the ±Y walls: elbow axis (61808, OD 52). The small-pulley hub runs in the elbow belt
    # cassette (confidence campaign: FEA showed the shell can't carry belt tension; the journals never reached the walls)
    for y in (-h["y"] / 2, h["y"] / 2):
        p += Pos(0, y, -ELBOW_Z) * Rot(90, 0, 0) * _ring(52.0 + 10, 52.0, 8)
    # anti-rotation slot block for the cassette pin (slot radial to the elbow axis, so belt tension never reaches the shell)
    p += Pos(0, (CAS_B[1] + 10 + h["y"] / 2 - WALL) / 2, -92) * (Box(16, h["y"] / 2 - WALL - CAS_B[1] - 10, 16)
                                                                - Box(5.4, 40, 12))
    # openings: cartridge bay on -Y (the cartridge drops in from the side), elbow hub exit on +Y
    p -= Pos(0, -h["y"] / 2, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Cylinder(cm.MOTOR["od"] / 2 + 4, 20)
    p -= Pos(0, h["y"] / 2, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26, 20)
    # (the old torque-pin slot cut here sat inside the bay hole and removed nothing: the cartridge torque-reaction path is
    #  open on every stage — see rnd/integration.py and the RND_PLAN queue)
    # bottom cap is a separate print (bone_cap()): the cassette goes in from below
    return p - Pos(0, 0, CAP_Z) * Box(200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))


def bone_cap() -> Part:
    """Screw-on bottom cap of the elbow housing (4 × M3 into heat-set inserts in the corners, ESTIMATE) with a 5 mm
    locating spigot. It closes the opening the elbow belt cassette is inserted through."""
    h = HOUSING
    hz = h["z1"] - h["z0"]
    full = Pos(0, 0, (h["z0"] + h["z1"]) / 2) * soft_shell(h["x"], h["y"], hz, 34, 26, WALL)
    cap = full & Pos(0, 0, CAP_Z) * Box(200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
    lx, ly = h["x"] - 2 * WALL - 12.6, h["y"] - 2 * WALL - 12.6                           # clears the soft bottom edge (r 26)
    lip = _rbox(lx, ly, 5, 26.5) - _rbox(lx - 3.2, ly - 3.2, 5, 24.9)                     # open 1.6 mm ring
    return cap + Pos(0, 0, CAP_Z + 2.5) * lip


def cassette_plate(side: str = "B") -> Part:
    """Elbow belt cassette (confidence campaign): one of two 7 mm plates that carry the small-pulley hub (61804 in an
    eccentric bushing, Ø38.2 seat) and ride on the elbow pulley hub on their own 61808 (Ø52 seat). The belt loop closes
    inside the cassette. Plate B has the anti-rotation pin tab. Housing frame."""
    from build123d import Sketch, Circle, Rectangle, make_hull, extrude, Plane, Locations
    y0, y1 = CAS_A if side == "A" else CAS_B
    zE, zS = -ELBOW_Z, -ELBOW_Z + BELT_C
    t = y1 - y0
    disc = lambda x, z, r: Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, t, align=MIN)
    p = disc(0, zE, 31) + disc(0, zS, 27)
    # web between the seats (hull approximated by a tapered box)
    p += Pos(0, y0 + t / 2, (zE + zS) / 2) * Box(54, t, BELT_C)
    for x in (-CAS_POST_X, CAS_POST_X):
        p += disc(x, CAS_POST_Z, 7.5) + Pos(x / 2, y0 + t / 2, CAS_POST_Z) * Box(abs(x), t, 14)
        p -= disc(x, CAS_POST_Z, 2.75)                                    # M5 through-bolt
    if side == "B":
        p += Pos(0, y0 + t / 2, (zS + 27 - 12 + -92) / 2) * Box(16, t, (-92) - (zS + 27 - 12) + 6)
    p -= disc(0, zE, 26) + disc(0, zS, 19.1)
    return p


def cassette_spacer() -> Part:
    """Spacer tube between the cassette plates (2 per cassette), Ø14 over an M5 bolt."""
    return Pos(0, CAS_A[1], CAS_POST_Z) * Rot(-90, 0, 0) * (Cylinder(7, CAS_B[0] - CAS_A[1], align=MIN)
                                                            - Cylinder(2.75, CAS_B[0] - CAS_A[1], align=MIN))


def cassette() -> Part:
    return cassette_plate("A") + cassette_plate("B") + sum((Pos(x, 0, 0) * cassette_spacer() for x in (-CAS_POST_X, CAS_POST_X)), Part())


def forearm() -> Part:
    """Forearm frame: origin on the elbow axis at the upper arm's centre plane; arm hangs along -Z."""
    y = FORE_Y
    link_t = 22.0
    p = Pos(0, y, 0) * Rot(90, 0, 0) * Cylinder(40, link_t)                            # hub disc on the elbow pulley hub
    p -= Pos(0, y, 0) * Rot(90, 0, 0) * Cylinder(21, link_t)
    body_len = FORE_L - 40 - 38
    # link (loop 9): carbon tube Ø30×1 (see cf_fore()) bonded into a socket on the hub disc and one on the wrist can
    for z0 in (-40 - 24, -FORE_L + 38):
        p += Pos(0, y, z0) * (Cylinder(18, 24, align=MIN) - Cylinder(15.1, 24, align=MIN))
    # wrist housing: holds the wrist cartridge (axis Y, output toward -Y)
    # (loop 10) sized to the real DM-J4310 cartridge (rnd/parts/hand.py): KX56 face at y = WRIST_Y0, open toward -Y
    can_d = J4310["od"] + 2 * 2.4 + 0.6 + 0.6
    wy = WRIST_Y1 - WRIST_Y0
    p += Pos(0, WRIST_Y0, -FORE_L) * Rot(-90, 0, 0) * (Cylinder(can_d / 2 + WALL, wy, align=MIN) - Cylinder(can_d / 2, wy - WALL, align=MIN))
    p += Pos(0, y, -FORE_L + 38) * Box(56, link_t, 12)                                  # web joining link and wrist can
    return p


def cf_neck() -> Part:
    """Carbon tube Ø40×1, flange socket → housing socket."""
    z_top, z_bot = -6.0, HOUSING["z1"] + 20
    return Pos(0, 0, z_bot) * (Cylinder(20, z_top - z_bot, align=MIN) - Cylinder(19, z_top - z_bot, align=MIN))


def cf_fore() -> Part:
    """Carbon tube Ø30×1 along the forearm side link."""
    z_top, z_bot = -40.0, -FORE_L + 38 + 24
    return Pos(0, FORE_Y, z_bot) * (Cylinder(15, z_top - z_bot, align=MIN) - Cylinder(14, z_top - z_bot, align=MIN))


def elbow_pulley_hub() -> Part:
    """40T elbow pulley + Ø40 shaft (runs in 2 × 61808) + bolt flange for the forearm."""
    from .belt_stage import pulley
    p = Pos(0, 0, 0) * Rot(90, 0, 0) * Cylinder(20, HOUSING["y"] + 12)
    p += Pos(0, BELT_Y, 0) * Rot(-90, 0, 0) * Pos(0, 0, -8) * pulley(40, 16)      # on the small pulley's belt plane
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
    cf = {"CF tubes (neck Ø40 + forearm Ø30)": (cf_neck().volume + cf_fore().volume) / 1e6 * 1.55}
    kxv = lambda k: kx_hub(k).volume + kx_collar(k).volume + retainer(k).volume
    sockets = {"KX74 sockets ×2 (elbow, yaw: GIM since loop 5)": 2 * kxv(KX74), "KX56 socket (wrist)": kxv(KX56),
               "elbow pulley hub (40T)": elbow_pulley_hub().volume, "elbow small-pulley hub": small_hub().volume - kx_hub(KX74).volume}
    out = {k: v / 1e6 * density for k, v in shells.items()}
    out.update({k: v / 1e6 * density * 0.7 for k, v in sockets.items()})
    out.update(cf)
    out["bearings: 2×61808 + 2×61804 + 4×61805 (ESTIMATE)"] = 0.020 * 2 + 0.008 * 2 + 0.012 * 4
    out["belt, pins, balls, screws (ESTIMATE)"] = 0.06
    return out


def assembly(elbow_deg: float = -55.0, wrist_deg: float = -35.0):
    """(name, part, colour) for renders. Upper-arm frame; elbow bends about Y (negative = forward, as in the twin)."""
    from .belt_stage import small_hub, belt_loop, pulley
    from .cartridge_s import sleeve, servo_dummy
    from .kx import collar, balls_mated
    out = [("upper_bone", upper_bone(), "white"), ("bone_cap", bone_cap(), "white"), ("e_cassette", cassette(), "dark")]
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
    # (the yaw stage above the flange lives in rnd/parts/roll.py)
    fe = Pos(0, 0, -ELBOW_Z) * Rot(0, elbow_deg, 0)
    out += [("forearm", fe * forearm(), "white"), ("cf_fore", fe * cf_fore(), "dark"), ("cf_neck", cf_neck(), "dark")]
    # wrist cartridge (DM-J4310 + KX56) + hand (rnd/parts/hand.py), hand centre plane on the arm's centre plane (y = 0)
    from . import hand as hd
    from .kx import KX56
    wr = fe * Pos(0, 0, -FORE_L)
    cart = wr * Pos(0, hd.WRIST_HUB_Y + KX56.gap + hd.cartridge_d_stack(), 0) * Rot(90, 0, 0)
    out += [("wrist_can", cart * hd.can_d(), "white"),
            ("wrist_motor", cart * Pos(0, 0, hd.FLOOR) * Cylinder(J4310["od"] / 2, J4310["h"], align=MIN), "dark"),
            ("wrist_adapter", cart * Pos(0, 0, hd.FLOOR + J4310["h"]) * hd.adapter_d(), "dark")]
    hand = wr * Rot(0, wrist_deg, 0)
    out += [(f"hand_{n}", hand * p, c) for n, p, c in hd.assembly()]
    return out


def cs_shift():
    from .cartridge_s import SERVO
    return SERVO["l"] / 2 - SERVO["shaft_off"]
