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

import math

from build123d import Align, Axis, Box, Cylinder, Part, Pos, Rot, fillet

from . import cartridge_m as cm
from .soft import soft_shell
from .kx import KX56, KX74, hub as kx_hub, collar as kx_collar   # (masses: what D24 removed)

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
WALL = 1.2                  # 3 perimeters at 0.4 mm. Loop 9 (rnd/bones.py): strength is never binding; stiffness comes from the CF tubes
from ..params import BASELINE as _D
ELBOW_Z = (_D.arm.l_upper - _D.arm.yaw_drop) * 1000   # yaw interface → elbow axis (loop 10: 230 → 200 mm)
FORE_L = 280.0              # elbow axis → wrist axis (params: l_fore)
BELT_C = 70.0               # elbow belt centre distance (rnd/belts.py)
HOUSING = dict(x=98.0, y=108.0, z0=-ELBOW_Z - 42.0, z1=-ELBOW_Z + 70.0 + 56.0, rv=28.0, rv_minus=14.0)   # D24 (2026-11-27): 110 → 98 wide, the KX collar (r 52.5 lugs, loop 23) is gone; the can (r 43.3) has 4.5 mm inside the skin. Corner r 14 on the −Y half, 28 on the +Y (forearm) half (was 34: at 98 wide that skin hit the cassette posts)
J4310 = dict(od=57.0, h=46.0)   # DM-J4310 (actuators.yaml)
FORE_Y = HOUSING["y"] / 2 + 18.0  # forearm link centre plane (+Y side link); loop 23: +6 so the CF socket/tube clear the housing over the full elbow range
# elbow belt cassette (housing frame, y spans of the two plates): set by the D13 small stage bearings (loop 38). D24: the
# bolted cartridge's flange plane is plate A's −Y face, Y_E = 3.7 (stage frame of belt_stage.small_stage_v3: y = Y_E − z);
# the plates, bearings and belt stay where loop 38 put them (the KX face was at y = −20).
from .belt_stage import V3 as _V2
Y_E = 3.7
_y = lambda z: Y_E - z
BELT_Y = _y(_V2["belt_c"])                                         # 20.2
CAS_A = (_y(_V2["jA"][1]) - 1.0, _y(_V2["jA"][0]))                  # 6 mm plate, bearing A flush on the pulley side (3.7 … 9.7; 0.5 mm to the pulley)
CAS_B = (_y(_V2["jB"][1]), _y(_V2["jB"][0]) + 0.2)                  # bearing B flush on the pulley side; loop 40: 6 → 5.2 mm (the D15 hub moved it 0.8 mm toward the +Y corner skin)
CAS_POST_X, CAS_POST_Z = 35.0, -ELBOW_Z + BELT_C - 10.0     # loop 38: 32 → 35, boss r 7 (R15: belt ≥ 3 mm); D24: plate B's bosses r 5.3 to clear the 98 mm housing's +Y corners
CAP_Z = HOUSING["z0"] + 10.0          # housing / bottom-cap split
STAGE = Pos(0, Y_E, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Rot(0, 0, -90)   # elbow stage frame → housing frame (+Z → −Y, +X → −Z)
WRIST_Y0, WRIST_Y1 = 43.0, 96.0   # wrist can span (holds the DM-J4310 cartridge; its KX56 face at y ≈ 33.6, hand centre at y = 0)


def _rbox(x, y, z, r, align=(Align.CENTER, Align.CENTER, Align.CENTER)):
    b = Box(x, y, z, align=align)
    return fillet(b.edges().filter_by(Axis.Z), min(r, x / 2 - 0.1, y / 2 - 0.1))


def _housing_solid(inset=0.0):
    """Loop 23: housing envelope with big soft corners on the +Y half (the forearm's CF tube and socket swing within
    3 mm of it) and tighter corners on the −Y half (room for the KX collar Ø98 and the cartridge can)."""
    from .soft import soft_box
    h = HOUSING
    hz = h["z1"] - h["z0"]
    c = Pos(0, 0, (h["z0"] + h["z1"]) / 2)
    i = inset
    plus = c * soft_box(h["x"] - 2 * i, h["y"] - 2 * i, hz - 2 * i, max(h["rv"] - i, 1), max(26 - i, 0.6))
    minus = c * soft_box(h["x"] - 2 * i, h["y"] - 2 * i, hz - 2 * i, max(h["rv_minus"] - i, 1), max(26 - i, 0.6))
    return (plus & Pos(0, 100, 0) * Box(400, 200, 600)) + (minus & Pos(0, -100, 0) * Box(400, 200, 600))


def housing_shell():
    return _housing_solid(0.0) - _housing_solid(WALL)


STIFF_T = 2.8                # loop 40 (A8): inner layer on the +Y half and the top (1.2 + 2.8 = 4 mm)


def housing_stiffener() -> Part:
    """Loop 40 (A8, rnd/stiffness.py): with P2 elements the 1.2 mm housing tilted the elbow bosses ≈ 39 mrad under 1 kg
    straight out (≈ 20 mm at the hand; P1 had shown 5 mrad). The forearm load enters the +Y boss and has to reach the
    neck socket on top: the +Y half and the top become 4 mm (an inner 2.8 mm layer), and four 3 mm gussets tie the neck
    socket to the top plate. The −Y half (cartridge bay, KX collar room) keeps its 1.2 mm skin."""
    h = HOUSING
    inner = _housing_solid(WALL - 0.4) - _housing_solid(WALL + STIFF_T)          # 0.4 mm overlap into the skin: coincident faces crash the fuse
    yw = h["y"] / 2 - WALL - STIFF_T - 0.5                                               # +Y face slab only (cassette plate B ends at y 43.7)
    st = (inner & Pos(0, yw + 100, 0) * Box(400, 200, 600)) + (inner & Pos(0, 0, h["z1"] - 10) * Box(400, 400, 20))
    st -= Pos(0, -20, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Cylinder(45.0, 40)          # D24: clear of the bolted can (r 43.3; loop 40 kept r 61 for the KX collar)
    for k in range(4):
        a = 45 + 90 * k
        st += Rot(0, 0, a) * Pos(23 + 14, 0, h["z1"]) * Box(28, 3, 14, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return st


def _hollow_rbox(x, y, z, r, wall=WALL):
    return _rbox(x, y, z, r) - _rbox(x - 2 * wall, y - 2 * wall, z - 2 * wall, max(r - wall, 0.5))


def _ring(od, idd, t):
    return Cylinder(od / 2, t) - Cylinder(idd / 2, t)


def upper_bone() -> Part:
    h = HOUSING
    p = Pos(0, 0, -6) * Cylinder(45, 6, align=MIN)                                    # yaw-hub flange
    # neck (loop 9): carbon tube Ø40×1 (see cf_neck()) bonded into two printed sockets, one on the flange and one on the housing
    # Astra #2b finding 2 (2026-11-27): the tube started where its sockets ended (zero bond). The neck is only ≈ 55 mm, so
    # one continuous printed sleeve runs flange → housing and the tube is bonded over its whole length (≈ 57 mm ≥ 1 × OD)
    p += Pos(0, 0, h["z1"] - 2) * (Cylinder(23, -4 - (h["z1"] - 2), align=MIN) - Cylinder(20.1, -4 - (h["z1"] - 2), align=MIN))
    hz = h["z1"] - h["z0"]
    p += housing_shell()                                                                 # looks pass: soft on every edge (loop 23: two corner radii)
    p += housing_stiffener()                                                             # loop 40 (A8): P2 FEA found the 1.2 mm shell far too soft
    # bearing bosses through the ±Y walls: elbow axis (61808, OD 52). The small-pulley hub runs in the elbow belt
    # cassette (confidence campaign: FEA showed the shell can't carry belt tension; the journals never reached the walls)
    for side in ("-", "+"):                                                              # D19: 61808 bosses with lugs for the seat caps
        p += _seat_boss(side)
    # (D24: loop 23's elbow torque arm and the KX collar-tab window are gone; the bolted flange reacts the torque in plate A)
    # anti-rotation slot block for the cassette pin (slot radial to the elbow axis, so belt tension never reaches the shell)
    p += Pos(0, (CAS_B[1] + 2 + h["y"] / 2 - WALL) / 2, -92) * (Box(16, h["y"] / 2 - WALL - CAS_B[1] - 2, 16)
                                                                - Box(5.4, 40, 12))
    # openings: cartridge bay on -Y (the cartridge drops in from the side), elbow hub exit on +Y
    p -= Pos(0, -h["y"] / 2, -ELBOW_Z + BELT_C) * Rot(90, 0, 0) * Cylinder(cm.MOTOR["od"] / 2 + 4.5, 20)   # loop 36 (R15): 1.2 mm around the can
    p -= Pos(0, h["y"] / 2, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26, 20)
    p -= Pos(0, -h["y"] / 2 + 3, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26, 20, align=MIN)   # loop 40: −Y wall opened for the elbow shaft (was blind; the shaft now goes in from −Y)
    # D19: U-slots under both elbow seats: the whole cassette (shaft + 40T + plates + belt + small stage, 61808s on the tube
    # ends) goes in from below; bolted seat caps (elbow_seat_cap) close them and restore the boss ring
    for side in ("-", "+"):
        p -= _seat_cap_envelope(side, 0.2)
        sc = SEAT_SCREW[side]
        for x in (-sc["x"], sc["x"]):
            p -= Pos(x, sc["y"], -ELBOW_Z - SEAT_LUG_H) * Cylinder(sc["insert_d"] / 2, sc["depth"], align=MIN)
    # bottom cap is a separate print (bone_cap()): the cassette goes in from below
    return p - Pos(0, 0, CAP_Z) * Box(200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))


SEAT_Y = {"-": (-HOUSING["y"] / 2 - 4.0, 8.0), "+": (HOUSING["y"] / 2 - 7.0, 7.0)}   # (y0, thickness) of the ±Y 61808 bosses
SEAT_LUG_R, SEAT_LUG_H = 35.0, 20.0
# cap screws: −Y 2 × M4 (the boss stands 4 mm proud of the −Y face); +Y 2 × M3 placed inside the housing's rounded +Y
# corners (the forearm socket sweeps 3 mm outside them)
SEAT_SCREW = {"-": dict(x=30.5, y=-HOUSING["y"] / 2, insert_d=5.6, clr_d=4.5, head_d=7.5, depth=8.0, head_z=0.0),
              "+": dict(x=29.5, y=HOUSING["y"] / 2 - 4.5, insert_d=4.0, clr_d=3.4, head_d=6.0, depth=6.0, head_z=6.0)}


def _seat_boss(side: str) -> Part:
    y0, t = SEAT_Y[side]
    b = Pos(0, y0 + t / 2, -ELBOW_Z) * Rot(90, 0, 0) * _ring(2 * SEAT_LUG_R, 52.0, t)
    b += Pos(0, y0 + t / 2, -ELBOW_Z - SEAT_LUG_H / 2) * Box(2 * SEAT_LUG_R, t, SEAT_LUG_H)
    return b & _housing_solid(0.0) if side == "+" else b


def _seat_cap_envelope(side: str, pad: float = 0.0) -> Part:
    """T-shaped space under an elbow seat: 52 wide from the axis down to the lugs, 70 wide from the lugs to the cap split."""
    y0, t = SEAT_Y[side]
    zE = -ELBOW_Z
    e = Pos(0, y0 + t / 2, zE - SEAT_LUG_H / 2) * Box(52.0 + 2 * pad, t + 2 * pad, SEAT_LUG_H + 0.01)
    zl = zE - SEAT_LUG_H
    e += Pos(0, y0 + t / 2, (zl + CAP_Z - 1) / 2) * Box(2 * SEAT_LUG_R + 2 * pad, t + 2 * pad, zl - CAP_Z + 1 + pad)
    return e


def elbow_seat_cap(side: str) -> Part:
    """D19: bolted lower half of an elbow 61808 seat (printed), 2 screws from below into heat-set inserts in the boss lugs
    (heads counterbored), so the boss ring is closed again. The belt pull (≈ 0.7 kN peak, toward the cartridge) presses
    the bearing into the housing half, not the cap."""
    y0, t = SEAT_Y[side]
    sc = SEAT_SCREW[side]
    c = _seat_cap_envelope(side, 0.0) & Pos(0, 0, CAP_Z + 0.2) * Box(200, 200, 400, align=MIN)
    if side == "+":
        c &= _housing_solid(0.0)
    c -= Pos(0, y0 + t / 2, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26.0, t + 2)
    for x in (-sc["x"], sc["x"]):
        c -= Pos(x, sc["y"], CAP_Z - 5) * Cylinder(sc["clr_d"] / 2, 60, align=MIN)
        c -= Pos(x, sc["y"], CAP_Z - 5) * Cylinder(sc["head_d"] / 2, 5 + 4.5 + sc["head_z"], align=MIN)
    return c


def upper_bone_with_caps() -> Part:
    """Upper bone + both bolted seat caps as one solid (bolted joints taken as bonded, as shoulder.bracket()) for the FEA."""
    p = upper_bone()
    for side in ("-", "+"):                       # the caps fill their slots exactly (screw holes ignored)
        y0, t = SEAT_Y[side]
        c = _seat_cap_envelope(side, 0.25) & Pos(0, 0, CAP_Z) * Box(200, 200, 400, align=MIN)
        c = c & _housing_solid(0.0) if side == "+" else c
        p += c - Pos(0, y0 + t / 2, -ELBOW_Z) * Rot(90, 0, 0) * Cylinder(26.0, t + 2)
    return p


def bone_cap() -> Part:
    """Screw-on bottom cap of the elbow housing (4 × M3 into heat-set inserts in the corners, ESTIMATE) with a 5 mm
    locating spigot. It closes the opening the elbow belt cassette is inserted through."""
    h = HOUSING
    hz = h["z1"] - h["z0"]
    full = housing_shell()
    cap = full & Pos(0, 0, CAP_Z) * Box(200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
    lx, ly = h["x"] - 2 * WALL - 12.6, h["y"] - 2 * WALL - 12.6                           # clears the soft bottom edge (r 26)
    lip = _rbox(lx, ly, 5, 26.5) - _rbox(lx - 3.2, ly - 3.2, 5, 24.9)                     # open 1.6 mm ring
    return cap + Pos(0, 0, CAP_Z + 2.5) * lip


def cassette_plate(side: str = "B") -> Part:
    """Elbow belt cassette (confidence campaign): one of two 7 mm plates that carry the small-pulley hub (61804 in an
    eccentric bushing, Ø38.2 seat) and ride on the elbow pulley hub on their own thin 6708 (40×50×6, Ø50 seat;
    ~11 g vs 26 g for a 61808, which keeps the shoulder-roll workspace margin ≥ 1.5×; C0 ≈ 2.4 kN vs 0.7 kN belt peak). The belt loop closes
    inside the cassette. Plate B has the anti-rotation pin tab. Housing frame. D24: the elbow cartridge bolts to plate A
    (6 × M4 from the pulley side into the flange inserts), so its torque closes through the M5 posts and plate B's pin."""
    from build123d import Sketch, Circle, Rectangle, make_hull, extrude, Plane, Locations
    y0, y1 = CAS_A if side == "A" else CAS_B
    zE, zS = -ELBOW_Z, -ELBOW_Z + BELT_C
    t = y1 - y0
    disc = lambda x, z, r: Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, t, align=MIN)
    p = disc(0, zE, 31) + disc(0, zS, 36 if side == "A" else 27)          # D24: plate A carries the cartridge flange (Ø68)
    # web between the seats (hull approximated by a tapered box)
    p += Pos(0, y0 + t / 2, (zE + zS) / 2) * Box(54, t, BELT_C)
    for x in (-CAS_POST_X, CAS_POST_X):
        rb = 7.0 if side == "A" else 5.3
        p += disc(x, CAS_POST_Z, rb) + Pos(x / 2, y0 + t / 2, CAS_POST_Z) * Box(abs(x), t, 2 * rb)
        p -= disc(x, CAS_POST_Z, 2.75)                                    # M5 through-bolt
    if side == "B":
        p += Pos(0, y0 + t / 2, (zS + 27 - 12 + -92) / 2) * Box(16, t, (-92) - (zS + 27 - 12) + 6)
    p -= disc(0, zE, 25)                                                  # 6708 (40×50×6) seat
    if side == "A":
        from .belt_stage import wall_a_cuts
        p -= STAGE * wall_a_cuts()                                        # D24: eccentric seat, spigot counterbore, 6 × M4 countersunk
    else:
        p -= disc(0, zS, 19.1)                                            # eccentric-bushing seat
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
    # Astra #2b finding 2 (2026-11-27): the disc now clamps the elbow tube's double-D end (36 AF, y 59 … 83): a double-D
    # bore (positive drive) split by a 1.5 mm slot toward +Z and pinched by 2 × M5 across it (≈ 2 × 18 N·m friction,
    # ESTIMATE, on top of the flats; elbow ≤ 13 N·m in the basket carry, 22 N·m GIM peak)
    from .belt_stage import dd_prism, DD_AF
    p -= Pos(0, y - link_t / 2, 0) * Rot(-90, 0, 0) * dd_prism(link_t, DD_AF + 0.3, 20.15)
    p -= Pos(0, y, 30) * Box(1.5, link_t + 1, 21)
    for yy in (y - 5.5, y + 5.5):
        p -= Pos(0, yy, 29) * Rot(0, 90, 0) * Cylinder(2.75, 60)
    body_len = FORE_L - 40 - 38
    # link (loop 9): carbon tube Ø30×1 (see cf_fore()) bonded into a socket on the hub disc and one on the wrist can
    for z0 in (-40 - FORE_SOCKET, -FORE_L + 38):                                       # sockets ≥ 1 × OD (Astra #2b finding 2)
        p += Pos(0, y, z0) * (Cylinder(18, FORE_SOCKET, align=MIN) - Cylinder(15.1, FORE_SOCKET, align=MIN))
    # wrist housing: holds the wrist cartridge (axis Y, output toward -Y)
    # (loop 10) sized to the real DM-J4310 cartridge (rnd/parts/hand.py): KX56 face at y = WRIST_Y0, open toward -Y
    can_d = J4310["od"] + 2 * 2.4 + 0.6 + 0.6               # D24: the can OD is the spigot in this bore (0.3 a side; loop 36 had 1 mm float)
    wy = WRIST_Y1 - WRIST_Y0
    p += Pos(0, WRIST_Y0, -FORE_L) * Rot(-90, 0, 0) * (Cylinder(can_d / 2 + WALL, wy, align=MIN) - Cylinder(can_d / 2, wy - WALL, align=MIN))
    p += Pos(0, y, -FORE_L + 38) * Box(56, link_t, 12)                                  # web joining link and wrist can
    p -= Pos(0, y, -FORE_L + 38) * Cylinder(15.1, 2 * FORE_SOCKET, align=MIN)           # the CF tube runs 30 mm into the wrist socket
    # D24 (2026-11-27): the wrist cartridge bolts to a 5 mm rim (Ø80) with 6 × M3 heat-set inserts (r 36); loop 24's
    # torque pad and slot are gone (no torque pin)
    rim = Pos(0, WRIST_Y0, -FORE_L) * Rot(-90, 0, 0) * (Cylinder(40.0, 5.0, align=MIN) - Cylinder(can_d / 2, 5.0, align=MIN))
    for k in range(6):
        a = math.pi / 3 * k + math.pi / 6
        rim -= Pos(36.0 * math.cos(a), WRIST_Y0, -FORE_L + 36.0 * math.sin(a)) * Rot(-90, 0, 0) * Cylinder(2.0, 5.0, align=MIN)
    return p + rim


def cf_neck() -> Part:
    """Carbon tube Ø40×1 in the neck sleeve, bonded over its full length (flange → housing)."""
    z_top, z_bot = -6.0, HOUSING["z1"]                       # bonded z1 … −6 (≈ 55 mm ≥ 1 × OD); the housing top plate is below z1
    return Pos(0, 0, z_bot) * (Cylinder(20, z_top - z_bot, align=MIN) - Cylinder(19, z_top - z_bot, align=MIN))


FORE_SOCKET = 30.0


def cf_fore() -> Part:
    """Carbon tube Ø30×1 along the forearm side link, 30 mm (1 × OD) into each socket (it used to stop at the sockets)."""
    z_top, z_bot = -40.0, -FORE_L + 38
    return Pos(0, FORE_Y, z_bot) * (Cylinder(15, z_top - z_bot, align=MIN) - Cylinder(14, z_top - z_bot, align=MIN))


def elbow_pulley_hub() -> Part:
    """Elbow shaft, D19 (2026-11-27): Ø40 tube with the flanged 40T printed on it in one piece (no keyed joint; Astra
    review #2b finding 1: loop 40's split 40T could not pass the tube). The 61808s and 6708s slide on from the two ends;
    the whole cassette (shaft + plates + belt + small stage) goes up into the housing's U-seats, closed by
    elbow_seat_cap()."""
    from .belt_stage import pulley, dd_flats
    y0, y1 = -(HOUSING["y"] / 2 + 12), FORE_Y + 11.0          # −66 … 83: the +Y end runs through the forearm disc (was ±66)
    L = y1 - y0
    t = Cylinder(20, L, align=MIN) - Cylinder(15, L, align=MIN)
    t += Pos(0, 0, y1 - BELT_Y - 8) * (pulley(40, 16, flange=True) - Pos(0, 0, -5) * Cylinder(15, 30, align=MIN))   # local z = y1 − y
    t -= dd_flats(0, y1 - (FORE_Y - 13.0))                    # double-D under the forearm clamp (y 59 … 83)
    return Pos(0, y1, 0) * Rot(90, 0, 0) * t


PRINT = {
    "upper_bone": "split at z = -100 (neck/housing) and along the Y centre plane → 3 pieces ≤ 256 mm, bolted + doweled",
    "forearm": "one piece, printed on its side (link plane on the bed), 280 mm diagonal fits the 256 mm bed corner-to-corner",
}


def masses(density=1.27):
    """Printed mass (kg): shells are solid walls; hubs/collars ~70 % effective density (4 perimeters + 25 % infill).
    D24/D19 (2026-11-27): the elbow and wrist KX sockets are gone (bolted flanges, Oldham drive at the elbow), the 40T is
    integral with the elbow shaft, the D13 v3 metal parts are counted by material, the elbow seat caps are added. The yaw
    KX74 socket stays here until roll.py moves the yaw stage to D24."""
    from . import cartridge_m as cm, hand as hd
    from .belt_stage import small_stage_v3
    from .kx import retainer
    st = dict((n, p) for n, p, _ in small_stage_v3(20))
    shells = {"upper bone shell + D19 bolted seat caps": upper_bone().volume + elbow_seat_cap("-").volume + elbow_seat_cap("+").volume,
              "forearm shell": forearm().volume}
    cf = {"CF tubes (neck Ø40 + forearm Ø30)": (cf_neck().volume + cf_fore().volume) / 1e6 * 1.55}
    kxv = lambda k: kx_hub(k).volume + kx_collar(k).volume + retainer(k).volume
    sockets = {"KX74 socket (yaw, roll.py: still KX)": kxv(KX74), "elbow shaft + integral 40T": elbow_pulley_hub().volume,
               "wrist D24 output flange + can flange ring": hd.adapter_d().volume + hd.can_d().volume - hd.can_d_loop10_volume()}
    out = {k: v / 1e6 * density for k, v in shells.items()}
    out.update({k: v / 1e6 * density * 0.7 for k, v in sockets.items()})
    out["elbow D13 v3 stage (steel Ø12, alu pulley, 6801s) + Oldham (alu hub, acetal disc, ASA adapter)"] = (
        st["shaft_12"].volume * 7.85e-3 + st["al_pulley"].volume * 2.7e-3 + (st["brg_A_6801"].volume + st["brg_B_6801"].volume) * 7.85e-3 * 0.6
        + st["flange_coupling"].volume * 2.7e-3 * 0.8 + st["oldham_disc"].volume * 1.41e-3 + st["kx_hub_v2"].volume * density * 0.7e-3) / 1e3
    out["elbow can nose/flange/rear cap over the loop-10 can"] = (cm.can_bolted().volume + cm.can_cap_bolted().volume - cm.can().volume) * density * 0.8e-6
    out.update(cf)
    out["bearings: 2×61808 + 2×61804 + 4×61805 (ESTIMATE)"] = 0.020 * 2 + 0.008 * 2 + 0.012 * 4
    out["belt, screws, inserts (ESTIMATE)"] = 0.05
    return out


def assembly(elbow_deg: float = -55.0, wrist_deg: float = -35.0):
    """(name, part, colour) for renders. Upper-arm frame; elbow bends about Y (negative = forward, as in the twin)."""
    from .belt_stage import small_hub, belt_loop, pulley
    from .cartridge_s import sleeve, servo_dummy
    from .kx import collar, balls_mated
    out = [("upper_bone", upper_bone(), "white"), ("bone_cap", bone_cap(), "white"), ("e_cassette", cassette(), "dark")]
    # elbow stage, built in the stage frame (cartridge +Z, big pulley at +X), then mapped: +Z→-Y, +X→-Z (STAGE)
    from .belt_stage import bolted_stage, V3
    out += bolted_stage(STAGE, 20, "e_")                                                 # D24: bolted cartridge + Oldham + D13 stage
    bc = V3["belt_c"]
    out += [("e_belt", STAGE * Pos(0, 0, bc - 7.5) * belt_loop(31.8, 63.7, BELT_C, width=15.0), "dark"),
            ("e_seat_cap_m", elbow_seat_cap("-"), "white"), ("e_seat_cap_p", elbow_seat_cap("+"), "white")]
    # (the yaw stage above the flange lives in rnd/parts/roll.py)
    fe = Pos(0, 0, -ELBOW_Z) * Rot(0, elbow_deg, 0)
    out.append(("e_shaft40", fe * elbow_pulley_hub(), "steel"))                           # D19: integral 40T, turns with the forearm (its clamp, double-D)
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
