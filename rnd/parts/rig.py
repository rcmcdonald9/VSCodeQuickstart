"""M1 Step-1 lever rig: stand, motor plates, levers, bolted-through weights, rubber hard stops and the interlocked guard,
plus the Step-1a fixtures (index disc, stall arm, tilt mandrel).  python -m rnd.parts.rig [--render]

Fable review 2026-11-28, Critical C1: the Step-1 rig was a bar "bolted straight to the GIM output" with no stops, no guard
and no CAD, and the software described the arm. This is the rig, built from the Step-1a parts (the GIM6010-8, the
DM-J4310, the 6061 test lever, the stand extrusion, screws, inserts, filament) plus cheap stock (docs/M1_RIG.md lists
what is new). Every number the software also uses (axis height, lever radius, stop angles, masses, bumper) is read from
sw/config/rig_step1.yaml through sw/rig.py, so the CAD, the twin and the bring-up cannot drift apart.

Layout (mm; X along the base rail, Y along the motor axes, Z up from the bench top; θ = lever angle from hanging, +θ
toward +X for both levers):
- **Stand** (40×40 / 40×80 aluminium extrusion): an H base of two 900 side rails directly under the two lever planes
  (their −X ends clamped to the bench outside the guard) and two cross members; a centre post carrying both motor
  plates; a leg at +X and a 40×80 crossbar over them carrying the upper stops; the lower stops on 6061 blocks bolted to
  the side rails (a stop hit pushes along a rail, no torsion on a joint). Joints: M8 core-bore screws + corner brackets.
- **Motors back to back on the post:** the GIM6010-8 on the +Y face, the DM-J4310 on the −Y face, each on a 10 mm 6061
  plate stood off 20 mm (4 × M6 + Ø12 × 20 spacers into the post's slot: room behind for the rear connectors). The motor
  is held by its housing holes (drilled to the measured pattern; seller question 6 says which face).
- **Levers:** a 6061 hub disc Ø64 × 10 on each output (8 × M3, the stub screws) and the 300 × 40 × 6 bar on it (4 × M5);
  the weight bolt (M8 8.8 + nyloc) at 270 mm clamps Ø60 steel plates and Ø60 × 10 discs THROUGH the bar, in pairs, one
  each side (nothing hangs, nothing can slide off).
- **Hard stops at the weight radius** (= the lever's centre of percussion: a stop hit puts almost no load on the motor
  bearing or the hub joint): natural-rubber Ø50 × 50 buffers that the disc rim strikes, the reference stop at −15° (on
  the side-rail blocks) and the span stop at +100° (under the crossbar). Software range −10° … +90°.
- **Guard:** a lift-off box of 3 mm polycarbonate on printed edge clips, ≥ 40 mm outside every swept part, the side rails
  passing through 44 × 42 notches (≤ 2 mm gap) and the cables through a 15 mm slot in the top ≥ 120 mm from the swept
  volume (ISO 13857 Table 4: slot 12–20 mm → ≥ 120 mm); a magnetic safety switch in series with the e-stop chain opens the
  contactor whenever the box is lifted.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from build123d import Align, Box, Cylinder, Part, Pos, Rot, export_step, export_stl

from sw import rig as R

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
PROFILE = R.load("step1")
G = PROFILE["rig"]
OUT = Path(__file__).resolve().parents[1] / "out" / "rig"
IMG = Path(__file__).resolve().parents[2] / "docs" / "img" / "m1_rig.png"
MM = 1000.0
P2S = 256.0                                    # printer bed limit (RND_PLAN guardrail)

Z_AX = G["axis_z"] * MM
R_M = G["r_mass"] * MM
TH_LO, TH_HI = G["stops_deg"]
BAR = dict(l=G["bar"]["l"] * MM, w=G["bar"]["w"] * MM, t=G["bar"]["t"] * MM, tail=G["bar"]["tail"] * MM)
HUB = dict(d=G["hub"]["d"] * MM, t=G["hub"]["t"] * MM)
DISC = dict(d=G["disc"]["d"] * MM, t=G["disc"]["t"] * MM, hole=G["disc"]["hole"] * MM)
CARRIER_T = 3.0                                # Ø60 × 3 steel clamp plates of the weight carrier (with the M8 bolt + nyloc)
BUMP = dict(d=G["bumper"]["d"] * MM, h=G["bumper"]["h"] * MM)
POST, SO, PT = G["frame"]["post"] * MM, G["frame"]["standoff"] * MM, G["frame"]["plate_t"] * MM
STOP_PT = 12.0                                 # 6061 stop plates (bending checked in docs/M1_RIG.md)
PC_T = 3.0                                     # polycarbonate guard sheet
CLEAR = 40.0                                   # guard ≥ this outside every swept part
MOTOR = {j: dict(od=G["motors"][j]["od"] * MM, h=G["motors"][j]["h"] * MM, side=G["motors"][j]["side"]) for j in R.LEVERS}
OUT_PCD = {"yaw": 34.0, "wpitch": 30.0}         # output bolt circles, ESTIMATE until measured (cartridge_m.MOTOR; DM: ESTIMATE)
HOUSING = {"yaw": (6, 57.0), "wpitch": (4, 40.0)}   # housing hole count / PCD, ESTIMATE: drilled to the measured pattern
HUB_M5 = [(sx * 12.0, sz * 22.0) for sx in (-1, 1) for sz in (-1, 1)]   # bar ↔ hub screws (x across the bar, z along it)


# ------------------------------------------------------------------------------------------------ helpers
def box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl_y(x, z, y0, y1, r) -> Part:
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_x(y, z, x0, x1, r) -> Part:
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def cyl_z(x, y, z0, z1, r) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def turn(part, theta_deg):
    """The lever at θ: rotation about its motor axis (along Y through z = Z_AX), +θ toward +X."""
    return Pos(0, 0, Z_AX) * Rot(0, -theta_deg, 0) * Pos(0, 0, -Z_AX) * part


def yc(j):
    return R.lever_plane_y(PROFILE, j) * MM


def disc_centre(theta_deg):
    t = math.radians(theta_deg)
    return R_M * math.sin(t), Z_AX - R_M * math.cos(t)


# ------------------------------------------------------------------------------------------------ geometry (derived)
X_HI, Z_HI = disc_centre(TH_HI)
Z_BUMP_HI = Z_HI + DISC["d"] / 2               # upper bumper face (the disc rim's top at +100°)
Z_STOP_HI = Z_BUMP_HI + BUMP["h"]              # upper stop plate underside
Z_CB = Z_STOP_HI + STOP_PT                     # crossbar underside
X_LO, Z_LO = disc_centre(TH_LO)
X_BUMP_LO = X_LO - DISC["d"] / 2               # lower bumper face (the disc rim's −X side at −15°)
X_STOP_LO = X_BUMP_LO - BUMP["h"]              # lower stop block +X face
LEG_X = (X_HI + DISC["d"] / 2 + 4, X_HI + DISC["d"] / 2 + 4 + POST)        # leg just outside the upper stop plate
BLOCK = dict(x=50.0, y=40.0, z=Z_LO + 25 - POST)                           # lower stop block (6061) on its side rail
BLOCK_X = (X_STOP_LO - BLOCK["x"], X_STOP_LO)
RAIL_X = (-450.0, 450.0)                       # side rails: the −X ends reach 200 mm past the guard for the 2 bench clamps
Y_STOP = max(abs(yc(j)) for j in R.LEVERS) + BUMP["d"] / 2 + 1             # the upper stop plate reaches both lever planes


def stack(j, load):
    """(inner, outer) y of the weight stack on lever j for a session load: carrier plates + discs in pairs."""
    n = PROFILE["rig"]["loads"][load][j] // 2
    s, y = MOTOR[j]["side"], yc(j)
    d = BAR["t"] / 2 + CARRIER_T + n * DISC["t"]
    return y - s * d, y + s * d


# ------------------------------------------------------------------------------------------------ parts: stand
def side_rails() -> Part:
    """Two 40 × 40 rails along X directly under the lever planes (an H base): the lower stop blocks sit on them, so a stop
    hit pushes along a rail instead of twisting a foot joint; the −X ends are clamped to the bench outside the guard."""
    out = None
    for j in R.LEVERS:
        r = box(RAIL_X[0], RAIL_X[1], yc(j) - POST / 2, yc(j) + POST / 2, 0, POST)
        out = r if out is None else out + r
    return out


def cross_members() -> Part:
    """Two 40 × 40 cross members between the side rails (M8 core-bore screws), under the post and under the leg."""
    y0, y1 = sorted(yc(j) for j in R.LEVERS)
    return box(-POST / 2, POST / 2, y0 + POST / 2, y1 - POST / 2, 0, POST) + box(LEG_X[0], LEG_X[1], y0 + POST / 2, y1 - POST / 2, 0, POST)


def post() -> Part:
    return box(-POST / 2, POST / 2, -POST / 2, POST / 2, POST, Z_CB)


def leg() -> Part:
    return box(LEG_X[0], LEG_X[1], -POST / 2, POST / 2, POST, Z_CB)


def crossbar() -> Part:
    """40 × 80 (80 along Y), over the post and the leg."""
    return box(-POST / 2, LEG_X[1], -POST, POST, Z_CB, Z_CB + POST)


def motor_plate(j) -> Part:
    """6061-T6 10 mm, 130 × 150, on 4 × M6 (spacers Ø12 × 20) into the post's slot; motor housing holes drilled to the
    measured pattern (ESTIMATE pattern shown, c'bored on the post side), Ø36 centre bore + a 16 mm cable slot to the edge."""
    s = MOTOR[j]["side"]
    y0 = s * (POST / 2 + SO)
    p = box(-65, 65, y0, y0 + s * PT, Z_AX - 75, Z_AX + 75)
    for dz in (-66, -50, 50, 66):
        p -= cyl_y(0, Z_AX + dz, y0 - s, y0 + s * (PT + 1), 3.3)
    n, pcd = HOUSING[j]
    for k in range(n):
        a = 2 * math.pi * k / n + math.pi / n
        x, z = pcd / 2 * math.cos(a), Z_AX + pcd / 2 * math.sin(a)
        p -= cyl_y(x, z, y0 - s, y0 + s * (PT + 1), 1.7)
        p -= cyl_y(x, z, y0 - s, y0 + s * 3.5, 3.25)
    p -= cyl_y(0, Z_AX, y0 - s, y0 + s * (PT + 1), 18)
    p -= box(-8, 8, y0 - s, y0 + s * (PT + 1), Z_AX - 76, Z_AX)
    return p


def standoffs(j) -> Part:
    s = MOTOR[j]["side"]
    out = None
    for dz in (-66, -50, 50, 66):
        c = cyl_y(0, Z_AX + dz, s * POST / 2, s * (POST / 2 + SO), 6) - cyl_y(0, Z_AX + dz, s * POST / 2, s * (POST / 2 + SO), 3.3)
        out = c if out is None else out + c
    return out


def motor_dummy(j) -> Part:
    """Reference volume of the motor (not made): GIM6010-8 Ø80 × 40, DM-J4310 Ø57 × 46."""
    s, y0 = MOTOR[j]["side"], MOTOR[j]["side"] * (POST / 2 + SO + PT)
    return cyl_y(0, Z_AX, y0, y0 + s * MOTOR[j]["h"], MOTOR[j]["od"] / 2)


# ------------------------------------------------------------------------------------------------ parts: levers and weights
def hub(j) -> Part:
    """6061-T6 Ø64 × 10 on the motor output: 8 × Ø3.4 c'bored on the output PCD (drilled to the measured pattern), 4 × M5
    tapped for the bar, pilot recess to the measured output pilot (ESTIMATE Ø20 × 2)."""
    s = MOTOR[j]["side"]
    y0 = s * (POST / 2 + SO + PT + MOTOR[j]["h"])
    p = cyl_y(0, Z_AX, y0, y0 + s * HUB["t"], HUB["d"] / 2)
    for k in range(8):
        a = 2 * math.pi * k / 8 + math.pi / 8
        x, z = OUT_PCD[j] / 2 * math.cos(a), Z_AX + OUT_PCD[j] / 2 * math.sin(a)
        p -= cyl_y(x, z, y0 - s, y0 + s * (HUB["t"] + 1), 1.7)
        p -= cyl_y(x, z, y0 + s * (HUB["t"] - 3.2), y0 + s * (HUB["t"] + 1), 3.25)
    for dx, dz in HUB_M5:
        p -= cyl_y(dx, Z_AX + dz, y0 - s, y0 + s * (HUB["t"] + 1), 2.1)
    p -= cyl_y(0, Z_AX, y0 - s, y0 + s * 2, 10.1)
    return p


def bar(j) -> Part:
    """The kit's 6061 test lever 300 × 40 × 6, hanging: axis 15 mm from its top end, 4 × Ø5.5 to the hub, Ø8.5 weight
    hole at 270 mm."""
    y = yc(j)
    p = box(-BAR["w"] / 2, BAR["w"] / 2, y - BAR["t"] / 2, y + BAR["t"] / 2, Z_AX - (BAR["l"] - BAR["tail"]), Z_AX + BAR["tail"])
    for dx, dz in HUB_M5:
        p -= cyl_y(dx, Z_AX + dz, y - 5, y + 5, 2.75)
    p -= cyl_y(0, Z_AX - R_M, y - 5, y + 5, DISC["hole"] / 2)
    return p


def weights(j, load) -> Part:
    """Carrier (2 × Ø60 × 3 plates + M8 bolt + nyloc) and the session's Ø60 × 10 discs, through the bar at 270 mm."""
    s, y = MOTOR[j]["side"], yc(j)
    n = PROFILE["rig"]["loads"][load][j] // 2
    z = Z_AX - R_M
    out = None
    for side in (-1, 1):
        y1 = y + side * BAR["t"] / 2
        parts = [cyl_y(0, z, y1, y1 + side * CARRIER_T, DISC["d"] / 2)]
        y1 += side * CARRIER_T
        for _ in range(n):
            parts.append(cyl_y(0, z, y1, y1 + side * DISC["t"], DISC["d"] / 2))
            y1 += side * DISC["t"]
        parts.append(cyl_y(0, z, y1, y1 + side * (5.3 if side == s else 8.0), 6.5))      # bolt head outside, nyloc inside
        for q in parts:
            out = q if out is None else out + q
    i, o = stack(j, load)
    out += cyl_y(0, z, i, o, 4.0)
    return out


def lever(j, load, theta=0.0) -> Part:
    return turn(hub(j) + bar(j) + weights(j, load), theta)


# ------------------------------------------------------------------------------------------------ parts: stops
def bumpers_upper() -> Part:
    out = None
    for j in R.LEVERS:
        b = cyl_z(X_HI, yc(j), Z_BUMP_HI, Z_STOP_HI, BUMP["d"] / 2) + cyl_z(X_HI, yc(j), Z_STOP_HI, Z_CB + 8, 5)
        out = b if out is None else out + b
    return out


def stop_plate_upper() -> Part:
    """6061-T6 12 × 60 × 2·Y_STOP under the crossbar: 2 × M6 into its two bottom slots (y ±20), 2 × Ø10.5 for the bumper studs."""
    p = box(X_HI - 30, X_HI + 30, -Y_STOP, Y_STOP, Z_STOP_HI, Z_CB)
    for y in (-20, 20):
        p -= cyl_z(X_HI, y, Z_STOP_HI - 1, Z_CB + 1, 3.3)
    for j in R.LEVERS:
        p -= cyl_z(X_HI, yc(j), Z_STOP_HI - 1, Z_CB + 1, 5.25)
    return p


def bumpers_lower() -> Part:
    out = None
    for j in R.LEVERS:
        b = cyl_x(yc(j), Z_LO, X_STOP_LO, X_BUMP_LO, BUMP["d"] / 2) + cyl_x(yc(j), Z_LO, X_STOP_LO - 15, X_STOP_LO, 5)
        out = b if out is None else out + b
    return out


def stop_block_lower(j) -> Part:
    """6061-T6 block 50 × 40 × (to 25 mm over the bumper axis) on lever j's side rail: 2 × M8 × 110 through it into M8
    T-nuts in the rail's top slot (preload ≈ 2 × 14 kN: tipping capacity ≈ 0.5 kN·m about its −X edge, slip ≈ 5.6 kN),
    an M10 tapped hole 18 deep in the +X face for the bumper stud."""
    y = yc(j)
    p = box(BLOCK_X[0], BLOCK_X[1], y - BLOCK["y"] / 2, y + BLOCK["y"] / 2, POST, POST + BLOCK["z"])
    for dx in (12.0, 38.0):
        p -= cyl_z(BLOCK_X[0] + dx, y, POST - 1, POST + BLOCK["z"] + 1, 4.5)
        p -= cyl_z(BLOCK_X[0] + dx, y, POST + BLOCK["z"] - 8.5, POST + BLOCK["z"] + 1, 7.0)
    p -= cyl_x(y, Z_LO, X_STOP_LO - 18, X_STOP_LO + 1, 4.25)
    return p


def stop_blocks_lower() -> Part:
    return stop_block_lower("yaw") + stop_block_lower("wpitch")


# ------------------------------------------------------------------------------------------------ parts: guard
def swept_extent(load="soak"):
    """Bounding extent (x0, x1, y0, y1, z1) of everything that moves, over the full stop-to-stop range."""
    xs, zs = [], []
    for th in [TH_LO + 0.5 * k for k in range(int((TH_HI - TH_LO) / 0.5) + 1)]:
        t = math.radians(th)
        for r, half in ((R_M, DISC["d"] / 2), (BAR["l"] - BAR["tail"], BAR["w"] / 2 * 1.42), (-BAR["tail"], BAR["w"] / 2 * 1.42)):
            cx, cz = r * math.sin(t), Z_AX - r * math.cos(t)
            xs += [cx - half, cx + half]
            zs += [cz - half, cz + half]
    ys = [y for j in R.LEVERS for y in stack(j, load)]
    ys = [y + math.copysign(9.0, y) for y in ys]                  # bolt head / nyloc
    return min(xs), max(xs), min(ys), max(ys), max(zs + [Z_AX + HUB["d"] / 2])


def guard_box():
    """Inner faces of the guard (x0, x1, y0, y1, z1): CLEAR outside the swept volume and outside the frame."""
    x0, x1, y0, y1, z1 = swept_extent()
    gx0 = min(x0, BLOCK_X[0]) - CLEAR
    gx1 = max(x1, LEG_X[1]) + CLEAR
    gy = max(abs(y0), abs(y1), Y_STOP) + CLEAR
    gz = max(z1, Z_CB + POST + 8) + CLEAR
    return gx0, gx1, -gy, gy, gz


NOTCH = (44.0, 42.0)                            # base-rail notch in the end panels (w × h): ≤ 2 mm gap around the 40 × 40
SLOT = (60.0, 15.0)                             # cable slot in the top panel (x × y)
SLOT_Y = 52.0                                   # over the GIM side of the post, beside the crossbar


def guard_panels() -> dict:
    gx0, gx1, gy0, gy1, gz = guard_box()
    t = PC_T
    def end(x0, x1):
        p = box(x0, x1, gy0, gy1, 0, gz)
        for j in R.LEVERS:                                          # the two side rails pass through
            p -= box(x0 - 1, x1 + 1, yc(j) - NOTCH[0] / 2, yc(j) + NOTCH[0] / 2, -1, NOTCH[1])
        return p
    top = box(gx0 - t, gx1 + t, gy0 - t, gy1 + t, gz, gz + t) - box(-SLOT[0] / 2, SLOT[0] / 2, SLOT_Y - SLOT[1] / 2, SLOT_Y + SLOT[1] / 2, gz - 1, gz + t + 1)
    return {"guard_front": box(gx0, gx1, gy1, gy1 + t, 0, gz), "guard_back": box(gx0, gx1, gy0 - t, gy0, 0, gz),
            "guard_end_minus_x": end(gx0 - t, gx0), "guard_end_plus_x": end(gx1, gx1 + t), "guard_top": top}


def guard_clip(length=120.0) -> Part:
    """Printed PETG edge clip: a 20 × 20 angle, 3 mm walls, with 2 M4 heat-set insert bosses per leg (the PC is screwed to
    it with M4 × 8 button heads). Eight on the four vertical edges, four on the top edges, plus the four corner cubes."""
    a = box(0, 20, 0, 3, 0, length) + box(0, 3, 0, 20, 0, length)
    for z in (length * 0.25, length * 0.75):
        a -= cyl_y(12, z, -1, 4, 2.8)
        a -= cyl_x(12, z, -1, 4, 2.8)
    return a


def guard_corner() -> Part:
    """Printed PETG top corner: three 40 × 40 × 3 walls meeting (the top panel and two side panels)."""
    return box(0, 40, 0, 40, 0, 3) + box(0, 40, 0, 3, 0, 40) + box(0, 3, 0, 40, 0, 40)


def interlock_riser() -> Part:
    """Printed riser for the magnetic safety switch on the crossbar top: brings the switch to a 5 mm gap under the actuator
    on the guard's top panel (typical sensing distance ≥ 8 mm ON / ≤ 15 mm OFF, ESTIMATE: check the bought switch)."""
    return box(0, 30, 0, 90, 0, 10)


# ------------------------------------------------------------------------------------------------ parts: Step-1a fixtures
def index_disc() -> Part:
    """1a: printed Ø90 × 3 dial on the hub's 4 × M5 (bare output, no lever): a pointer and 36 tick slots, so the operator
    sees the output angle and direction. 12 g: nothing to stop or guard."""
    p = Cylinder(45, 3) + Pos(50, 0, 0) * Box(14, 6, 3)
    for k in range(36):
        a = 2 * math.pi * k / 36
        p -= Pos(41 * math.cos(a), 41 * math.sin(a), 0) * Rot(0, 0, math.degrees(a)) * Box(6 if k % 9 else 9, 1.2, 4)
    for dx, dz in HUB_M5:
        p -= Pos(dx, dz, 0) * Cylinder(2.75, 4)
    return p - Cylinder(10.5, 4)


def stall_arm() -> Part:
    """1a: printed PETG-CF 130 × 30 × 10 arm on the hub's 4 × M5, a Ø6 hole at r = 100 mm for the luggage scale's hook:
    the drive's torque limit and the torque constant at stall, the output blocked by the scale and its anchor."""
    p = Pos(50, 0, 0) * Box(130, 30, 10)
    for dx, dz in HUB_M5:
        p -= Pos(dz, dx, 0) * Cylinder(2.75, 11)
    p -= Pos(100, 0, 0) * Cylinder(3.0, 11)
    return p - Cylinder(10.5, 11)


def tilt_mandrel() -> Part:
    """1a (Fable M3): 6061-T6 turned mandrel on the GIM output (8 × M3 on the measured PCD): Ø50 × 8 flange, Ø25 boss to
    92 mm, Ø12 × 10 end for a 6001-2RS loading bearing; the side load goes on that bearing at 97 mm from the output face."""
    p = Cylinder(25, 8, align=MIN) + Pos(0, 0, 8) * Cylinder(12.5, 84, align=MIN) + Pos(0, 0, 92) * Cylinder(6, 10, align=MIN)
    for k in range(8):
        a = 2 * math.pi * k / 8 + math.pi / 8
        p -= Pos(OUT_PCD["yaw"] / 2 * math.cos(a), OUT_PCD["yaw"] / 2 * math.sin(a), 0) * Cylinder(1.7, 20)
    return p


# ------------------------------------------------------------------------------------------------ assembly + checks
def static_parts(load="soak") -> dict:
    d = {"side_rails": side_rails(), "cross_members": cross_members(), "post": post(), "leg": leg(), "crossbar": crossbar(),
         "stop_plate_upper": stop_plate_upper(), "stop_blocks_lower": stop_blocks_lower(),
         "bumpers_upper": bumpers_upper(), "bumpers_lower": bumpers_lower()}
    for j in R.LEVERS:
        d[f"motor_plate_{j}"] = motor_plate(j)
        d[f"standoffs_{j}"] = standoffs(j)
        d[f"motor_{j}"] = motor_dummy(j)
    d.update(guard_panels())
    return d


def check(load="soak", step=7.5):
    """Interference of each lever (with the load's weights) against every static part over the software range and up to
    the stops, the stop contact angles, guard clearances, the cable slot distance and the printed-part sizes."""
    fixed = static_parts(load)
    rows, worst = [], 0.0
    for j in R.LEVERS:
        lv0 = hub(j) + bar(j) + weights(j, load)
        own = {f"motor_{j}"}                                       # the hub sits on its own motor's output face (touching)
        angles = [TH_LO + 1.0] + [a for a in [TH_LO + 1 + step * k for k in range(1, 40)] if a < TH_HI - 1] + [TH_HI - 1.0]
        for th in angles:
            lv = turn(lv0, th)
            for n, p in fixed.items():
                if n in own:
                    continue
                v = (lv & p).volume
                if v > 1e-3:
                    rows.append(dict(lever=j, theta=round(th, 1), part=n, volume_mm3=round(v, 2)))
                    worst = max(worst, v)
    # stop contacts: the weights touch the bumpers within ±1° of the stop angles (and the bar never does)
    contact = {}
    for j in R.LEVERS:
        w0 = weights(j, load)
        for name, th_s, bump in (("lower", TH_LO, bumpers_lower()), ("upper", TH_HI, bumpers_upper())):
            inside = th_s - math.copysign(1.0, th_s - 40) * 1.0         # 1° before reaching the stop
            beyond = th_s + math.copysign(1.0, th_s - 40) * 1.0         # 1° past it (rubber compressed)
            contact[f"{j}_{name}"] = dict(stop_deg=th_s, clear_1deg_before=(turn(w0, inside) & bump).volume < 1e-3,
                                          touches_1deg_past=(turn(w0, beyond) & bump).volume > 1e-3,
                                          bar_clear_1deg_past=(turn(bar(j) + hub(j), beyond) & bump).volume < 1e-3)
    gx0, gx1, gy0, gy1, gz = guard_box()
    x0, x1, y0, y1, z1 = swept_extent(load)
    guard = dict(inner=dict(x=(round(gx0, 1), round(gx1, 1)), y=(round(gy0, 1), round(gy1, 1)), z_top=round(gz, 1)),
                 outer_mm=(round(gx1 - gx0 + 2 * PC_T, 1), round(gy1 - gy0 + 2 * PC_T, 1), round(gz + PC_T, 1)),
                 clearance_mm=dict(x_minus=round(x0 - gx0, 1), x_plus=round(gx1 - x1, 1), y_minus=round(y0 - gy0, 1),
                                   y_plus=round(gy1 - y1, 1), top=round(gz - z1, 1)),
                 slot_to_swept_mm=round(gz - z1, 1), slot_mm=SLOT, notch_gap_mm=round((NOTCH[0] - POST) / 2, 1),
                 pc_area_m2=round(2 * (gx1 - gx0) * gz / 1e6 + 2 * (gy1 - gy0) * gz / 1e6 + (gx1 - gx0) * (gy1 - gy0) / 1e6, 3))
    printed = {"guard_clip": guard_clip(), "guard_corner": guard_corner(), "interlock_riser": interlock_riser(),
               "index_disc": index_disc(), "stall_arm": stall_arm()}
    sizes = {}
    for n, p in printed.items():
        bb = p.bounding_box()
        sizes[n] = round(max(bb.size.X, bb.size.Y, bb.size.Z), 1)
    geo = dict(axis_z=Z_AX, r_mass=R_M, stops_deg=(TH_LO, TH_HI), upper_bumper_face_z=round(Z_BUMP_HI, 1), crossbar_z=(round(Z_CB, 1), round(Z_CB + POST, 1)),
               lower_bumper_face_x=round(X_BUMP_LO, 1), lever_planes_y={j: round(yc(j), 1) for j in R.LEVERS},
               leg_x=tuple(round(v, 1) for v in LEG_X), lower_block_x=tuple(round(v, 1) for v in BLOCK_X),
               extrusion_mm={"40x40": dict(side_rails=f"2 × {RAIL_X[1] - RAIL_X[0]:.0f}",
                                           cross_members=f"2 × {abs(yc('yaw') - yc('wpitch')) - POST:.0f}",
                                           post=round(Z_CB - POST, 1), leg=round(Z_CB - POST, 1)),
                             "40x80": dict(crossbar=round(LEG_X[1] + POST / 2, 1))},
               stop_plate_upper_mm=(60, round(2 * Y_STOP, 1), STOP_PT),
               stop_block_lower_mm=(BLOCK["x"], BLOCK["y"], round(BLOCK["z"], 1)),
               bottom_clearance_mm=round(Z_AX - R_M - DISC["d"] / 2, 1))
    ok = (not rows and all(all(v for k, v in c.items() if k != "stop_deg") for c in contact.values())
          and min(guard["clearance_mm"].values()) >= CLEAR - 0.5 and guard["slot_to_swept_mm"] >= 120 and guard["notch_gap_mm"] <= 2.0
          and all(s <= P2S for s in sizes.values()))
    return dict(ok=ok, interference=rows, contact=contact, guard=guard, printed_max_mm=sizes, geometry=geo, load=load,
                sizing=R.sizing(PROFILE))


def assembly(load="soak", theta=None, guard=False) -> dict:
    """{name: (part, colour)} for a render: levers at θ (default: GIM 70°, DM 30°)."""
    th = theta or {"yaw": 70.0, "wpitch": 30.0}
    col = {"motor": "dark", "bumpers": "dark", "standoffs": "steel"}
    out = {}
    for n, p in static_parts(load).items():
        if n.startswith("guard") and not guard:
            continue
        out[n] = (p, next((c for k, c in col.items() if n.startswith(k)), "white"))
    for j in R.LEVERS:
        out[f"hub_{j}"] = (turn(hub(j), th[j]), "white")
        out[f"bar_{j}"] = (turn(bar(j), th[j]), "white")
        out[f"weights_{j}"] = (turn(weights(j, load), th[j]), "steel")
    return out


def export(load="soak"):
    OUT.mkdir(parents=True, exist_ok=True)
    made = {"motor_plate_gim6010": motor_plate("yaw"), "motor_plate_dmj4310": motor_plate("wpitch"), "hub_gim6010": hub("yaw"),
            "hub_dmj4310": hub("wpitch"), "lever_bar": bar("yaw"), "stop_plate_upper": stop_plate_upper(),
            "stop_block_lower": stop_block_lower("yaw"), "tilt_mandrel": tilt_mandrel()}
    for n, p in made.items():
        export_step(p, str(OUT / f"{n}.step"))
    printed = {"guard_clip": guard_clip(), "guard_corner": guard_corner(), "interlock_riser": interlock_riser(),
               "index_disc": index_disc(), "stall_arm": stall_arm()}
    for n, p in printed.items():
        export_stl(p, str(OUT / f"{n}.stl"))
    asm = assembly(load)
    rdir = OUT.parent
    for n, (p, _) in asm.items():
        export_stl(p, str(rdir / f"asm_rig_{n}.stl"))                # asm_*: render inputs, git-ignored
    (rdir / "asm_rig_colors.txt").write_text("\n".join(f"asm_rig_{n} {c}" for n, (_, c) in asm.items()))
    return list(made) + list(printed)


def render(samples=32):
    """Blender (rnd/render_parts.py) of the assembly without the guard → docs/img/m1_rig.png."""
    import subprocess
    IMG.parent.mkdir(parents=True, exist_ok=True)
    tgt = "0.06,0,0.29"
    off = "0.55,1.45,0.55"
    subprocess.run(["blender", "-b", "--factory-startup", "--python", str(Path(__file__).resolve().parents[1] / "render_parts.py"), "--",
                    str(IMG), str(samples), "asm_rig_colors.txt", tgt, off], check=True, capture_output=True)
    return IMG


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    res = check()
    files = export()
    (OUT / "rig_check.json").write_text(json.dumps(res, indent=1, default=str))
    g = res["geometry"]
    print(f"Step-1 rig CAD: axis {g['axis_z']} mm, weight bolt r {g['r_mass']} mm, stops {g['stops_deg']}°, lever planes {g['lever_planes_y']} mm, "
          f"bottom clearance {g['bottom_clearance_mm']} mm")
    print(f"  extrusion cut list {g['extrusion_mm']}; upper stop plate {g['stop_plate_upper_mm']}, lower stop blocks {g['stop_block_lower_mm']}")
    print(f"  interference over the range: {len(res['interference'])} hits {res['interference'][:4]}")
    for k, c in res["contact"].items():
        print(f"  stop {k}: {c}")
    print(f"  guard: {res['guard']}")
    print(f"  printed parts, largest dimension (≤ {P2S:.0f}): {res['printed_max_mm']}")
    print(f"  wrote {len(files)} parts to {OUT}")
    if "--render" in argv:
        print(f"  render: {render()}")
    print("OK ✅" if res["ok"] else "FAIL ❌")
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
